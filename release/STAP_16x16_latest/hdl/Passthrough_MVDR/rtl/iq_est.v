// -----------------------------------------------------------------------------
// iq_est.v  --  blind estimate of each receiver channel's IQ image coefficient
//
//   Runs on the maths clock, once per covariance window (on cov_prep's
//   out_valid, when the window values and cov_prep's jammer flag are both
//   ready). Closed loop around iq_corr.v, which removes the image in the
//   sample stream: the window sums seen here are of the CORRECTED stream.
//
//   For x = s + g s* (image coefficient g, s proper) corrected with b:
//       xc = s (1 - b g*) + s* (g - b)
//       q_kk = sum xc^2 = 2 (g - b) r_kk  (to first order)
//   so one step   b <- b + q_kk / (2 r_kk)   lands on g, up to the estimate's
//   own error.
//   * Estimate error: over a finite window a CW jammer's own sum s^2 is not 0:
//     |sum e^{2jwn}| / N ~ 1 / (N sin w)  (-69 dB at 3.1 MHz offset), so a
//     single estimate limits the image to ~ -74 dBc. That residue rotates with
//     the jammer phase from window to window, so averaging cancels it:
//       first update after a clear/reset: full step (fast first correction),
//       later updates: b <- b + delta / 2^mu_sh   (default 2: error / ~2.6,
//       settles from -50 dBc in ~10 updates = 1.7 ms of jammer).
//
//   * Update only on windows with a STRONG jammer (cov_prep `strong_jam`, JNR >~
//     21 dB): there the jammer makes the estimate precise (noise error ~
//     1/(2 sqrt(N*JNR)), < -65 dBc) and it is the only case where the image
//     matters. A weak or noise-only window would give ~ -45 dBc of error,
//     worse than the AD9361's own image, so it must not update b. Without a jammer,
//     b is held: IQ imbalance is a fixed property of the hardware, so the next
//     jammer is corrected from its first window.
//   * After an update, NSUB+1 windows are skipped (settle): windows that still
//     hold samples corrected with the old b would read as a false residual.
//   * cfg_en = 0 or cfg_clear: b = 0 (iq_corr is then a pure x * 2^IQF).
//   * A window with r_kk = 0 does not update channel k.
//   * |b| is limited to the B_W range (|b| < 2^(B_W-1-B_FRAC) = 1/8).
//   Not suited to a CW jammer within a few kHz of the LO (its own x^2 does not
//   average to zero over the window); mean removal suppresses that case anyway.
//
// Timing: 4 divisions on one real_divider, ~290 clocks per update; windows
// arrive >= 1600 clocks apart. A trigger while busy is ignored (n_drop).
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module iq_est #(
    parameter ACC_W  = 44,
    parameter B_W    = 18,
    parameter B_FRAC = 20,
    parameter NSUB   = 8
)(
    input  wire                     clk,
    input  wire                     rst,

    input  wire                     cfg_en,
    input  wire                     cfg_clear,
    input  wire        [2:0]        cfg_mu_sh,     // refinement step 2^-mu_sh (0 = full)

    input  wire                     trig,          // cov_prep out_valid
    input  wire                     jam,           // cov_prep `strong_jam` (same window): JNR >~ 21 dB
    input  wire signed [ACC_W-1:0]  r11,
    input  wire signed [ACC_W-1:0]  r22,
    input  wire signed [ACC_W-1:0]  q11_re,
    input  wire signed [ACC_W-1:0]  q11_im,
    input  wire signed [ACC_W-1:0]  q22_re,
    input  wire signed [ACC_W-1:0]  q22_im,

    output reg                      b_upd,         // 1-clk: new b set
    output reg  signed [B_W-1:0]    b1_re,
    output reg  signed [B_W-1:0]    b1_im,
    output reg  signed [B_W-1:0]    b2_re,
    output reg  signed [B_W-1:0]    b2_im,
    output reg  [31:0]              n_upd,
    output reg  [31:0]              n_drop,
    output wire                     busy
);

    localparam NUM_W = ACC_W + B_FRAC + 1;
    localparam DEN_W = ACC_W + 1;
    localparam signed [NUM_W-1:0] BMAX =  ({{(NUM_W-1){1'b0}}, 1'b1} << (B_W-1)) - 1;
    localparam signed [NUM_W-1:0] BMIN = -({{(NUM_W-1){1'b0}}, 1'b1} << (B_W-1));

    localparam [2:0] S_IDLE = 3'd0, S_LOAD = 3'd1, S_GO = 3'd2, S_WAIT = 3'd3,
                     S_SUM  = 3'd4, S_OUT  = 3'd5;
    reg [2:0]  st;
    assign busy = (st != S_IDLE);

    reg [4:0]  settle;
    reg        fresh;                               // no update since clear/reset
    reg [2:0]  ush;
    reg [1:0]  ix;                                  // 0: q11 re, 1: q11 im, 2: q22 re, 3: q22 im
    reg signed [ACC_W-1:0] qv [0:3];
    reg        [ACC_W-1:0] rv [0:1];
    reg signed [NUM_W-1:0] dl [0:3];                // increments
    reg                    ok [0:1];

    reg                    dv_start;
    reg signed [NUM_W-1:0] dv_num;
    reg        [DEN_W-1:0] dv_den;
    wire                   dv_done, dv_div0;
    wire signed [NUM_W-1:0] dv_quo;
    real_divider #(.NUM_W(NUM_W), .DEN_W(DEN_W), .ROUND(1)) u_div (
        .clk(clk), .rst(rst), .start(dv_start), .num(dv_num), .den(dv_den),
        .busy(), .done(dv_done), .quo(dv_quo), .div0(dv_div0));

    function signed [B_W-1:0] satb;
        input signed [NUM_W-1:0] v;
        begin
            satb = (v > BMAX) ? BMAX[B_W-1:0] : ((v < BMIN) ? BMIN[B_W-1:0] : v[B_W-1:0]);
        end
    endfunction

    wire signed [NUM_W-1:0] e1r = {{(NUM_W-B_W){b1_re[B_W-1]}}, b1_re};
    wire signed [NUM_W-1:0] e1i = {{(NUM_W-B_W){b1_im[B_W-1]}}, b1_im};
    wire signed [NUM_W-1:0] e2r = {{(NUM_W-B_W){b2_re[B_W-1]}}, b2_re};
    wire signed [NUM_W-1:0] e2i = {{(NUM_W-B_W){b2_im[B_W-1]}}, b2_im};

    integer k;
    always @(posedge clk) begin
        b_upd    <= 1'b0;
        dv_start <= 1'b0;
        if (rst) begin
            st <= S_IDLE;
            settle <= 5'd0;
            fresh  <= 1'b1;
            b1_re <= 0; b1_im <= 0; b2_re <= 0; b2_im <= 0;
            n_upd <= 32'd0;  n_drop <= 32'd0;
        end else begin
            if (trig && st != S_IDLE) n_drop <= n_drop + 32'd1;
            case (st)
            S_IDLE: if (trig) begin
                if (!cfg_en || cfg_clear) begin
                    if (b1_re != 0 || b1_im != 0 || b2_re != 0 || b2_im != 0) b_upd <= 1'b1;
                    b1_re <= 0; b1_im <= 0; b2_re <= 0; b2_im <= 0;
                    settle <= 5'd0;
                    fresh  <= 1'b1;
                end else if (settle != 5'd0) begin
                    settle <= settle - 5'd1;
                end else if (jam) begin
                    qv[0] <= q11_re;  qv[1] <= q11_im;  qv[2] <= q22_re;  qv[3] <= q22_im;
                    rv[0] <= r11;     rv[1] <= r22;
                    ok[0] <= (r11 > 0);  ok[1] <= (r22 > 0);
                    ix <= 2'd0;
                    ush <= fresh ? 3'd0 : cfg_mu_sh;
                    st <= S_LOAD;
                end
            end
            S_LOAD: begin
                // num = q << B_FRAC ; den = 2 r   ->  quotient = q / (2r) in Q(B_FRAC)
                dv_num <= {{(B_FRAC+1){qv[ix][ACC_W-1]}}, qv[ix]} <<< B_FRAC;
                dv_den <= {rv[ix[1]], 1'b0};
                st <= S_GO;
            end
            S_GO: begin dv_start <= 1'b1; st <= S_WAIT; end
            S_WAIT: if (dv_done) begin
                dl[ix] <= (ok[ix[1]] && !dv_div0) ? dv_quo : {NUM_W{1'b0}};
                if (ix == 2'd3) st <= S_SUM;
                else begin ix <= ix + 2'd1; st <= S_LOAD; end
            end
            S_SUM: begin
                dl[0] <= e1r + (dl[0] >>> ush);  dl[1] <= e1i + (dl[1] >>> ush);
                dl[2] <= e2r + (dl[2] >>> ush);  dl[3] <= e2i + (dl[3] >>> ush);
                st <= S_OUT;
            end
            S_OUT: begin
                b1_re <= satb(dl[0]);  b1_im <= satb(dl[1]);
                b2_re <= satb(dl[2]);  b2_im <= satb(dl[3]);
                b_upd  <= 1'b1;
                n_upd  <= n_upd + 32'd1;
                settle <= NSUB + 1;
                fresh  <= 1'b0;
                st <= S_IDLE;
            end
            default: st <= S_IDLE;
            endcase
        end
    end

endmodule
