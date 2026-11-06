// -----------------------------------------------------------------------------
// stap_pi_filter.v
//
// Real-time space-time filter of a STAP power-inversion (STAP-PI) CRPA
// anti-jam processor:
//
//     y(n) = sum_{m=0}^{N_EL-1} sum_{k=0}^{N_TAPS-1} conj(w[m*N_TAPS+k]) * x_m(n-k)
//
// The adaptive weights w are computed per block (see python/stap_pi/core.py or
// matlab/stap_pi_process.m: covariance estimate + Cholesky solve) and loaded
// through a double-buffered register file:
//   * w_we/w_addr/w_re/w_im write the SHADOW bank (address = m*N_TAPS + k),
//   * w_commit copies SHADOW -> ACTIVE in one clock.  Assert it together
//     with in_valid of the first sample of the new block; that sample is the
//     first one filtered with the new weights.  A shadow write in the commit
//     cycle is not part of that commit (it lands for the next one).
//
// Number formats (bit-exact model: python/stap_pi/fixed_point.py):
//   in_re/in_im : IW-bit two's complement per element, element m in bits
//                 [m*IW +: IW]
//   w_re/w_im   : WW-bit two's complement, Q(WW-WFRAC).WFRAC (default Q1.16;
//                 the PI reference tap is exactly 1.0 = 2^16)
//   accumulator : full precision, AW = IW + WW + 1 + clog2(D) bits
//   out_re/im   : OW bits, round-half-up(acc / 2^out_shift), saturated.
//                 out_shift = WFRAC gives unity gain; WFRAC-g adds 6.02*g dB.
//
// Pipeline: 4 clocks from in_valid to out_valid
//   A: tap delay lines   B: complex multiplies   C: adder tree   D: round/sat
// The sample rate may be any integer fraction of clk (in_valid gaps allowed).
//
// Resources: 4*N_EL*N_TAPS real multipliers (56 DSP slices for 2 x 7 taps).
// For clk >> 150 MHz pipeline the adder tree (stage C) or use DSP cascades.
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps
`default_nettype none

module stap_pi_filter #(
    parameter integer N_EL   = 2,
    parameter integer N_TAPS = 7,
    parameter integer IW     = 16,
    parameter integer WW     = 18,
    parameter integer OW     = 16,
    parameter integer SW     = 6,
    parameter integer AWW    = 8      // weight address width
) (
    input  wire                   clk,
    input  wire                   rst,          // synchronous, active high

    input  wire                   in_valid,
    input  wire [N_EL*IW-1:0]     in_re,
    input  wire [N_EL*IW-1:0]     in_im,

    input  wire                   w_we,
    input  wire [AWW-1:0]         w_addr,
    input  wire signed [WW-1:0]   w_re,
    input  wire signed [WW-1:0]   w_im,
    input  wire                   w_commit,

    input  wire [SW-1:0]          out_shift,

    output reg                    out_valid,
    output reg signed [OW-1:0]    out_re,
    output reg signed [OW-1:0]    out_im
);

    localparam integer D    = N_EL * N_TAPS;
    localparam integer PW   = IW + WW + 1;          // conj(w)*x, one rail
    localparam integer AW   = PW + $clog2(D);       // accumulator
    localparam integer OMAX = (1 << (OW - 1)) - 1;
    localparam integer OMIN = -(1 << (OW - 1));

    // ---------------------------------------------------------------- storage
    reg signed [IW-1:0] dl_re [0:D-1];               // tap delay lines, [m*N_TAPS + k] = x_m(n-k)
    reg signed [IW-1:0] dl_im [0:D-1];
    reg signed [WW-1:0] ws_re [0:D-1];               // shadow weights
    reg signed [WW-1:0] ws_im [0:D-1];
    reg signed [WW-1:0] wa_re [0:D-1];               // active weights
    reg signed [WW-1:0] wa_im [0:D-1];
    reg signed [PW-1:0] p_re  [0:D-1];               // stage-B products
    reg signed [PW-1:0] p_im  [0:D-1];
    reg signed [AW-1:0] acc_re, acc_im;              // stage-C sums
    reg                 v_a, v_b, v_c;

    integer ia, ka, ib, ic, icm;

    // -------------------------------------------------- stage A: delay lines
    always @(posedge clk) begin
        if (rst) begin
            v_a <= 1'b0;
            for (ia = 0; ia < D; ia = ia + 1) begin
                dl_re[ia] <= {IW{1'b0}};
                dl_im[ia] <= {IW{1'b0}};
            end
        end else begin
            v_a <= in_valid;
            if (in_valid) begin
                for (ia = 0; ia < N_EL; ia = ia + 1) begin
                    dl_re[ia * N_TAPS] <= in_re[ia * IW +: IW];
                    dl_im[ia * N_TAPS] <= in_im[ia * IW +: IW];
                    for (ka = 1; ka < N_TAPS; ka = ka + 1) begin
                        dl_re[ia * N_TAPS + ka] <= dl_re[ia * N_TAPS + ka - 1];
                        dl_im[ia * N_TAPS + ka] <= dl_im[ia * N_TAPS + ka - 1];
                    end
                end
            end
        end
    end

    // ------------------------------------------------ weights: shadow/active
    always @(posedge clk) begin
        if (w_we && (w_addr < D)) begin
            ws_re[w_addr] <= w_re;
            ws_im[w_addr] <= w_im;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            for (icm = 0; icm < D; icm = icm + 1) begin
                wa_re[icm] <= {WW{1'b0}};
                wa_im[icm] <= {WW{1'b0}};
            end
        end else if (w_commit) begin
            for (icm = 0; icm < D; icm = icm + 1) begin
                wa_re[icm] <= ws_re[icm];
                wa_im[icm] <= ws_im[icm];
            end
        end
    end

    // ------------------------------------- stage B: conj(w) * x per tap
    //   re = wr*xr + wi*xi ,  im = wr*xi - wi*xr
    always @(posedge clk) begin
        if (rst) v_b <= 1'b0;
        else     v_b <= v_a;
        for (ib = 0; ib < D; ib = ib + 1) begin
            p_re[ib] <= wa_re[ib] * dl_re[ib] + wa_im[ib] * dl_im[ib];
            p_im[ib] <= wa_re[ib] * dl_im[ib] - wa_im[ib] * dl_re[ib];
        end
    end

    // ---------------------------------------------- stage C: adder tree
    reg signed [AW-1:0] sum_re, sum_im;
    always @* begin
        sum_re = {AW{1'b0}};
        sum_im = {AW{1'b0}};
        for (ic = 0; ic < D; ic = ic + 1) begin
            sum_re = sum_re + p_re[ic];
            sum_im = sum_im + p_im[ic];
        end
    end

    always @(posedge clk) begin
        if (rst) v_c <= 1'b0;
        else     v_c <= v_b;
        acc_re <= sum_re;
        acc_im <= sum_im;
    end

    // ------------------------------------- stage D: round half up + saturate
    function signed [OW-1:0] round_sat;
        input signed [AW-1:0] a;
        input        [SW-1:0] sh;
        reg   signed [AW:0]   ext, half, r;
        begin
            ext  = {a[AW-1], a};
            half = {(AW + 1){1'b0}};
            if (sh != 0) half[sh - 1] = 1'b1;
            r = (ext + half) >>> sh;
            if (r > OMAX)      round_sat = OMAX;
            else if (r < OMIN) round_sat = OMIN;
            else               round_sat = r[OW-1:0];
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            out_valid <= 1'b0;
            out_re    <= {OW{1'b0}};
            out_im    <= {OW{1'b0}};
        end else begin
            out_valid <= v_c;
            if (v_c) begin
                out_re <= round_sat(acc_re, out_shift);
                out_im <= round_sat(acc_im, out_shift);
            end
        end
    end

endmodule

`default_nettype wire
