// -----------------------------------------------------------------------------
// stap_fir.v  --  Space-Time Adaptive Processing filter (the "apply weights" part)
//
//   y[n] = sum_{m=0}^{M-1} sum_{k=0}^{N-1} conj(w[m*N+k]) * x_m[n+k]
//
// This is exactly  y = w_opt' * Xm  from STAP_constraint.m (MATLAB tap ordering:
// tap index t = m*N + k, element m, delay k).
//
//  * M antenna channels, N taps per channel  ->  T = M*N complex weights (7*16=112)
//  * Complex I/Q input, DW-bit signed per component
//  * Complex weights, WW-bit signed per component (18 bit -> one DSP48 port)
//  * FOLD : time-multiplexing factor. Clock must be >= FOLD * sample rate and
//           in_valid may be asserted at most once every FOLD clocks.
//           P = ceil(T/FOLD) complex MACs (4 real multipliers each) are built.
//             FOLD=1  -> 112 cMAC = 448 multipliers, 1 sample/clock
//             FOLD=4  ->  28 cMAC = 112 multipliers, e.g. 240 MHz clk / 60 MSPS
//  * Weights are double buffered: write the shadow bank at any time, pulse
//    w_commit, the new bank becomes active on a sample boundary (no tearing).
//  * Output = saturate( (sum) >>> out_shift , OW bits )   (arithmetic shift, floor)
//
// Latency (in_valid -> out_valid): FOLD + 4 + clog2(P) + 1 clocks.
// Verilog-2005, vendor neutral (multipliers are inferred -> DSP48 / DSP blocks).
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
module stap_fir #(
    parameter M    = 7,
    parameter N    = 16,
    parameter DW   = 16,
    parameter WW   = 18,
    parameter OW   = 16,
    parameter FOLD = 1
)(
    input  wire                 clk,
    input  wire                 rst,
    // sample stream (all M channels in parallel, channel m at bits [m*DW +: DW])
    input  wire                 in_valid,
    input  wire [M*DW-1:0]      in_re,
    input  wire [M*DW-1:0]      in_im,
    // shadow weight bank write port
    input  wire                 w_we,
    input  wire [7:0]           w_addr,     // tap index t = m*N+k  (T <= 256)
    input  wire                 w_sel_im,   // 0: real part, 1: imaginary part
    input  wire [WW-1:0]        w_data,
    input  wire                 w_commit,   // copy shadow -> active at next sample boundary
    output reg                  w_commit_pending,
    input  wire [5:0]           out_shift,
    // output stream
    output reg                  out_valid,
    output reg  signed [OW-1:0] out_re,
    output reg  signed [OW-1:0] out_im,
    output reg                  overrun,    // sticky: in_valid faster than 1/FOLD
    input  wire                 overrun_clr
);
    localparam T    = M*N;
    localparam P    = (T + FOLD - 1) / FOLD;         // number of complex MAC units
    localparam TP   = P * FOLD;                      // padded tap count
    localparam LV   = (P > 1) ? $clog2(P) : 0;       // adder-tree levels
    localparam PP   = 1 << LV;                       // padded unit count (power of 2)
    localparam PHW  = (FOLD > 1) ? $clog2(FOLD) : 1;
    localparam PW   = DW + WW;                       // real product width
    localparam SW   = PW + 1;                        // complex product component width
    localparam AW   = DW + WW + 2 + $clog2(TP);      // accumulator width (no overflow)

    integer i, lv;

    // ------------------------------------------------------------------ delay line
    // dl[m*N+k] holds x_m[n+k]  where n is the oldest sample in the window,
    // i.e. dl[m*N+N-1] is the newest sample of channel m.
    reg signed [DW-1:0] dl_re [0:TP-1];
    reg signed [DW-1:0] dl_im [0:TP-1];

    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < TP; i = i + 1) begin
                dl_re[i] <= {DW{1'b0}};
                dl_im[i] <= {DW{1'b0}};
            end
        end else if (in_valid) begin
            for (i = 0; i < T; i = i + 1) begin
                if ((i % N) == N-1) begin
                    dl_re[i] <= in_re[(i/N)*DW +: DW];
                    dl_im[i] <= in_im[(i/N)*DW +: DW];
                end else begin
                    dl_re[i] <= dl_re[i+1];
                    dl_im[i] <= dl_im[i+1];
                end
            end
        end
    end

    // ------------------------------------------------------------------ phase control
    reg [PHW-1:0] ph;
    reg           rd_act;          // MAC units read tap group 'ph' in this cycle

    always @(posedge clk) begin
        if (rst) begin
            rd_act  <= 1'b0;
            ph      <= {PHW{1'b0}};
            overrun <= 1'b0;
        end else begin
            if (overrun_clr) overrun <= 1'b0;
            if (in_valid) begin
                if (rd_act && (ph != FOLD-1)) overrun <= 1'b1;
                rd_act <= 1'b1;
                ph     <= {PHW{1'b0}};
            end else if (rd_act) begin
                if (ph == FOLD-1) rd_act <= 1'b0;
                else              ph     <= ph + 1'b1;
            end
        end
    end

    // ------------------------------------------------------------------ weights
    reg signed [WW-1:0] ws_re [0:TP-1];   // shadow
    reg signed [WW-1:0] ws_im [0:TP-1];
    reg signed [WW-1:0] wa_re [0:TP-1];   // active
    reg signed [WW-1:0] wa_im [0:TP-1];

    wire apply_commit = w_commit_pending & (in_valid | ~rd_act);

    always @(posedge clk) begin
        if (rst) begin
            w_commit_pending <= 1'b0;
            for (i = 0; i < TP; i = i + 1) begin
                ws_re[i] <= {WW{1'b0}};  ws_im[i] <= {WW{1'b0}};
                wa_re[i] <= {WW{1'b0}};  wa_im[i] <= {WW{1'b0}};
            end
        end else begin
            if (w_we && (w_addr < T)) begin
                if (w_sel_im) ws_im[w_addr] <= w_data;
                else          ws_re[w_addr] <= w_data;
            end
            if (apply_commit) begin
                w_commit_pending <= 1'b0;
                for (i = 0; i < T; i = i + 1) begin
                    wa_re[i] <= ws_re[i];
                    wa_im[i] <= ws_im[i];
                end
            end else if (w_commit) begin
                w_commit_pending <= 1'b1;
            end
        end
    end

    // ------------------------------------------------------------------ control pipeline
    reg vA, fA, lA, vB, fB, lB, vC, fC, lC;
    always @(posedge clk) begin
        if (rst) begin
            vA <= 1'b0; vB <= 1'b0; vC <= 1'b0;
            fA <= 1'b0; fB <= 1'b0; fC <= 1'b0;
            lA <= 1'b0; lB <= 1'b0; lC <= 1'b0;
        end else begin
            vA <= rd_act; fA <= (ph == 0); lA <= (ph == FOLD-1);
            vB <= vA;     fB <= fA;        lB <= lA;
            vC <= vB;     fC <= fB;        lC <= lB;
        end
    end

    // ------------------------------------------------------------------ complex MAC units
    wire [PP*AW-1:0] acc_re_flat;
    wire [PP*AW-1:0] acc_im_flat;

    genvar p;
    generate
        for (p = 0; p < PP; p = p + 1) begin : g_mac
            if (p < P) begin : g_unit
                reg signed [DW-1:0] xr, xi;          // stage A : operand select
                reg signed [WW-1:0] wr, wi;
                reg signed [PW-1:0] rr, ii, ri, ir;  // stage B : products (DSP M-reg)
                reg signed [SW-1:0] cr, ci;          // stage C : conj(w)*x
                reg signed [AW-1:0] ar, ai;          // stage D : accumulate over FOLD phases
                integer f;

                always @(posedge clk) begin
                    // stage A: select tap p*FOLD+ph
                    xr <= dl_re[p*FOLD]; xi <= dl_im[p*FOLD];
                    wr <= wa_re[p*FOLD]; wi <= wa_im[p*FOLD];
                    for (f = 1; f < FOLD; f = f + 1) begin
                        if (ph == f) begin
                            xr <= dl_re[p*FOLD+f]; xi <= dl_im[p*FOLD+f];
                            wr <= wa_re[p*FOLD+f]; wi <= wa_im[p*FOLD+f];
                        end
                    end
                    // stage B: 4 real multiplies
                    rr <= wr * xr;
                    ii <= wi * xi;
                    ri <= wr * xi;
                    ir <= wi * xr;
                    // stage C: conj(w)*x = (wr*xr + wi*xi) + j(wr*xi - wi*xr)
                    cr <= rr + ii;
                    ci <= ri - ir;
                    // stage D: accumulate the FOLD partial products
                    if (vC) begin
                        if (fC) begin ar <= cr;      ai <= ci;      end
                        else    begin ar <= ar + cr; ai <= ai + ci; end
                    end
                end
                assign acc_re_flat[p*AW +: AW] = ar;
                assign acc_im_flat[p*AW +: AW] = ai;
            end else begin : g_pad
                assign acc_re_flat[p*AW +: AW] = {AW{1'b0}};
                assign acc_im_flat[p*AW +: AW] = {AW{1'b0}};
            end
        end
    endgenerate

    // ------------------------------------------------------------------ pipelined adder tree
    reg signed [AW-1:0] tr_re [0:(LV+1)*PP-1];
    reg signed [AW-1:0] tr_im [0:(LV+1)*PP-1];
    reg [LV:0]          tv;          // tv[0]: accumulators hold final sums

    always @(posedge clk) begin
        if (rst) tv <= {(LV+1){1'b0}};
        else begin
            tv[0] <= vC & lC;
            for (lv = 1; lv <= LV; lv = lv + 1) tv[lv] <= tv[lv-1];
        end
    end

    always @(posedge clk) begin
        for (lv = 1; lv <= LV; lv = lv + 1) begin
            for (i = 0; i < PP; i = i + 1) begin
                if (i < (PP >> lv)) begin
                    if (lv == 1) begin
                        tr_re[PP+i] <= $signed(acc_re_flat[(2*i)*AW +: AW]) + $signed(acc_re_flat[(2*i+1)*AW +: AW]);
                        tr_im[PP+i] <= $signed(acc_im_flat[(2*i)*AW +: AW]) + $signed(acc_im_flat[(2*i+1)*AW +: AW]);
                    end else begin
                        tr_re[lv*PP+i] <= tr_re[(lv-1)*PP+2*i] + tr_re[(lv-1)*PP+2*i+1];
                        tr_im[lv*PP+i] <= tr_im[(lv-1)*PP+2*i] + tr_im[(lv-1)*PP+2*i+1];
                    end
                end
            end
        end
    end

    wire signed [AW-1:0] sum_re = (LV == 0) ? $signed(acc_re_flat[AW-1:0]) : tr_re[LV*PP];
    wire signed [AW-1:0] sum_im = (LV == 0) ? $signed(acc_im_flat[AW-1:0]) : tr_im[LV*PP];

    // ------------------------------------------------------------------ scale + saturate
    localparam signed [AW-1:0] MAXV = (1 << (OW-1)) - 1;
    localparam signed [AW-1:0] MINV = -(1 << (OW-1));

    wire signed [AW-1:0] sh_re = sum_re >>> out_shift;
    wire signed [AW-1:0] sh_im = sum_im >>> out_shift;

    always @(posedge clk) begin
        if (rst) begin
            out_valid <= 1'b0;
            out_re    <= {OW{1'b0}};
            out_im    <= {OW{1'b0}};
        end else begin
            out_valid <= tv[LV];
            out_re    <= (sh_re > MAXV) ? MAXV[OW-1:0] : (sh_re < MINV) ? MINV[OW-1:0] : sh_re[OW-1:0];
            out_im    <= (sh_im > MAXV) ? MAXV[OW-1:0] : (sh_im < MINV) ? MINV[OW-1:0] : sh_im[OW-1:0];
        end
    end

endmodule
