// ============================================================================
//  gnss_pulse_blank.v  --  output pulse blanker and RX peak hold (SMI-PI 4.7)
//  ANTSDR E310 V1 / GNSS CRPA project
//
//  gnss_pulse_blank
//    A jammer that is on for only part of the time (pulsed barrage) leaves
//    bursts at the filter output whenever the weights do not null them fully
//    (weights solved from a block that held no pulse, a low duty cycle, a pulse
//    that arrives before the weights do). A GNSS receiver loses lock on such
//    bursts. This stage zeroes the output sample stream while a burst is
//    present and a few samples either side of it: the satellites lose the
//    duty cycle of the burst (10*log10(1-d) dB, 0.5 dB for d = 10 %) instead
//    of the lock.
//
//      p[n]  = I^2 + Q^2 of the output sample
//      S[n]  = p[n] + ... + p[n-W+1]                 (W = 8, sliding sum)
//      ex[n] = S[n] > THR                            (THR from the firmware: about 6 dB
//                                                     above the output noise, 8 samples)
//      sample j is zeroed if ex[] is set anywhere from 7 samples after j back to H samples
//      before it: the output is delayed by D = 16 samples so that the decision
//      is known BEFORE the burst reaches the output.
//
//    The delay is always there (also with en = 0) so switching the blanker on
//    or off never moves the code phase. Samples that are not part of a burst
//    pass unchanged. Strobe in = strobe out, 4 clocks + 16 samples later.
//    en = 0: nothing is blanked, thr and the counters are still maintained.
//
//    A CONTINUOUS jammer that the filter has nulled leaves noise only (the
//    residual is 0.2 dB in the field log) and is never blanked. A continuous
//    jammer the filter has not nulled would be blanked all the time; the
//    firmware watches the counters and switches the blanker off then.
//
//  gnss_peak_hold
//    Largest |I| or |Q| over both RX channels in each window of 2^19 sample
//    strobes (8.5 ms at 61.44 MS/s), published at the end of each window, so
//    the RX gain loop sees every pulse instead of a 256-sample snapshot.
//    win_idx counts the windows (16 bits, wraps) so the firmware knows when a
//    value is new.
// ============================================================================
`timescale 1ns/100ps

module gnss_pulse_blank #(
    parameter W = 8,          // power window [samples]
    parameter D = 16,         // output delay [samples]
    parameter H = 16          // hold after a burst [samples]
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        clr,           // clears the counters
    input  wire        in_valid,
    input  wire [15:0] in_i,          // signed, 12-bit range
    input  wire [15:0] in_q,
    input  wire        en,
    input  wire [31:0] thr,           // threshold on the sum of W samples of I^2+Q^2
    output reg         out_valid,
    output reg  [15:0] out_i,
    output reg  [15:0] out_q,
    output reg  [31:0] blk_cnt,       // samples blanked
    output reg  [31:0] burst_cnt      // bursts (rising edges of the blank flag)
);
    localparam M = D + H + 1;

    // stage 0: input register
    reg               v0 = 1'b0;
    reg signed [15:0] i0 = 16'sd0, q0 = 16'sd0;
    // stage 1: power
    reg               v1 = 1'b0;
    reg signed [15:0] i1 = 16'sd0, q1 = 16'sd0;
    reg        [31:0] p1 = 32'd0;
    // stage 2: sliding sum
    reg               v2 = 1'b0;
    reg signed [15:0] i2 = 16'sd0, q2 = 16'sd0;
    reg        [35:0] sum = 36'd0;
    reg        [31:0] pdl [0:W-1];
    // stage 3: exceed flags and the delay line
    reg               v3 = 1'b0;
    reg  [M-1:0]      E  = {M{1'b0}};
    reg signed [15:0] di [0:D];
    reg signed [15:0] dq [0:D];
    reg               blk_prev = 1'b0;

    integer k;
    initial begin
        for (k = 0; k < W; k = k + 1) pdl[k] = 32'd0;
        for (k = 0; k <= D; k = k + 1) begin di[k] = 16'sd0; dq[k] = 16'sd0; end
        out_valid = 1'b0; out_i = 16'd0; out_q = 16'd0; blk_cnt = 32'd0; burst_cnt = 32'd0;
    end

    wire signed [31:0] pi_w = i0 * i0;
    wire signed [31:0] pq_w = q0 * q0;
    wire [35:0] sum_new = sum + {4'd0, p1} - {4'd0, pdl[W-1]};
    wire        ex_w    = (sum > {4'd0, thr});
    wire        blk_w   = en & (|E[D+H:8]);

    always @(posedge clk) begin
        if (!rst_n) begin
            v0 <= 1'b0; v1 <= 1'b0; v2 <= 1'b0; v3 <= 1'b0; out_valid <= 1'b0;
            sum <= 36'd0; E <= {M{1'b0}}; blk_prev <= 1'b0;
            for (k = 0; k < W; k = k + 1) pdl[k] <= 32'd0;
            for (k = 0; k <= D; k = k + 1) begin di[k] <= 16'sd0; dq[k] <= 16'sd0; end
            out_i <= 16'd0; out_q <= 16'd0;
            blk_cnt <= 32'd0; burst_cnt <= 32'd0;
        end else begin
            if (clr) begin blk_cnt <= 32'd0; burst_cnt <= 32'd0; end
            // stage 0
            v0 <= in_valid;
            if (in_valid) begin i0 <= in_i; q0 <= in_q; end
            // stage 1: p = I^2 + Q^2
            v1 <= v0;
            if (v0) begin
                i1 <= i0; q1 <= q0;
                p1 <= pi_w[31:0] + pq_w[31:0];
            end
            // stage 2: S[n] = S[n-1] + p[n] - p[n-W]
            v2 <= v1;
            if (v1) begin
                sum <= sum_new;
                pdl[0] <= p1;
                for (k = 1; k < W; k = k + 1) pdl[k] <= pdl[k-1];
                i2 <= i1; q2 <= q1;
            end
            // stage 3: exceed flag of sample n, delay line
            v3 <= v2;
            if (v2) begin
                E <= {E[M-2:0], ex_w};
                di[0] <= i2; dq[0] <= q2;
                for (k = 1; k <= D; k = k + 1) begin di[k] <= di[k-1]; dq[k] <= dq[k-1]; end
            end
            // stage 4: output (sample n-D), zero while a burst is near
            out_valid <= v3;
            if (v3) begin
                out_i <= blk_w ? 16'd0 : di[D];
                out_q <= blk_w ? 16'd0 : dq[D];
                blk_prev <= blk_w;
                if (blk_w && !clr) blk_cnt <= blk_cnt + 1'b1;
                if (blk_w && !blk_prev && !clr) burst_cnt <= burst_cnt + 1'b1;
            end
        end
    end
endmodule


module gnss_peak_hold #(
    parameter WIN_BITS = 19
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        valid,
    input  wire [15:0] a0, b0, a1, b1,    // I0 Q0 I1 Q1, signed
    output reg  [15:0] peak_win = 16'd0,
    output reg  [15:0] win_idx  = 16'd0
);
    reg [WIN_BITS-1:0] cnt = {WIN_BITS{1'b0}};
    reg [15:0]         run = 16'd0;

    function [15:0] absv(input [15:0] x);
        begin
            if (x[15]) absv = (x == 16'h8000) ? 16'h7FFF : (~x + 16'd1);
            else       absv = x;
        end
    endfunction

    wire [15:0] m0 = (absv(a0) > absv(b0)) ? absv(a0) : absv(b0);
    wire [15:0] m1 = (absv(a1) > absv(b1)) ? absv(a1) : absv(b1);
    wire [15:0] mx = (m0 > m1) ? m0 : m1;
    wire [15:0] nr = (mx > run) ? mx : run;

    always @(posedge clk) begin
        if (!rst_n) begin
            cnt <= {WIN_BITS{1'b0}}; run <= 16'd0; peak_win <= 16'd0; win_idx <= 16'd0;
        end else if (valid) begin
            cnt <= cnt + 1'b1;
            if (&cnt) begin
                peak_win <= nr;
                win_idx  <= win_idx + 1'b1;
                run <= 16'd0;
            end else begin
                run <= nr;
            end
        end
    end
endmodule
