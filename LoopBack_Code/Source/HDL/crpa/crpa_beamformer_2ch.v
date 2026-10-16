// ============================================================================
//  crpa_beamformer_2ch.v
//  2-element narrowband beamformer   y = w^H x = conj(w1) x1 + conj(w2) x2
//
//    conj(w) x :  re = wr*xr + wi*xi
//                 im = wr*xi - wi*xr
//
//  FORMATS
//    x1, x2  12-bit signed RX samples (ad_datafmt output, right-aligned)
//    w1, w2  Q8.16, 25-bit signed        (25 x 12 -> one DSP48E1 each)
//    y       12-bit signed, SATURATED    (never wraps, see the OUTPUT SAMPLE
//                                         ALIGNMENT note in gnss_passthrough.v)
//
//  PIPELINE  (advances every clock, so data and valid stay aligned for any
//             valid pattern)
//    1 register x and w
//    2 eight multiplies, registered
//    3 per-antenna complex sums
//    4 antenna sum + rounding, >>> 16
//    5 saturate to 12 bits
//  Latency: LATENCY = 5 clocks, x -> y.
//
//  Original work for the ANTSDR E310 V1 GNSS CRPA project.
// ============================================================================

`timescale 1ns/100ps

module crpa_beamformer_2ch (
  input                    clk,
  input                    rst,

  input                    in_valid,
  input  signed [11:0]     x1_re,
  input  signed [11:0]     x1_im,
  input  signed [11:0]     x2_re,
  input  signed [11:0]     x2_im,

  input  signed [24:0]     w1_re,
  input  signed [24:0]     w1_im,
  input  signed [24:0]     w2_re,
  input  signed [24:0]     w2_im,

  output reg               out_valid,
  output reg signed [11:0] y_re,
  output reg signed [11:0] y_im,
  output reg               out_sat      // 1 = this output sample was clipped
);

  localparam LATENCY = 5;

  // ---- stage 1: input registers ---------------------------------------------
  reg signed [11:0] a1r, a1i, a2r, a2i;
  reg signed [24:0] b1r, b1i, b2r, b2i;

  // ---- stage 2: products (Q.16) ---------------------------------------------
  reg signed [36:0] m1_rr, m1_ii, m1_ri, m1_ir;
  reg signed [36:0] m2_rr, m2_ii, m2_ri, m2_ir;

  // ---- stage 3: per-antenna sums ---------------------------------------------
  reg signed [37:0] s1_re, s1_im, s2_re, s2_im;

  // ---- stage 4: total, rounded to integer ------------------------------------
  reg signed [38:0] t_re, t_im;
  wire signed [38:0] tot_re = s1_re + s2_re + 39'sd32768;   // + 0.5 LSB
  wire signed [38:0] tot_im = s1_im + s2_im + 39'sd32768;

  reg [LATENCY-2:0] vpipe;

  // ---- stage 5: saturation ---------------------------------------------------
  wire re_hi = (t_re >  39'sd2047);
  wire re_lo = (t_re < -39'sd2048);
  wire im_hi = (t_im >  39'sd2047);
  wire im_lo = (t_im < -39'sd2048);

  always @(posedge clk) begin
    if (rst) begin
      a1r <= 0; a1i <= 0; a2r <= 0; a2i <= 0;
      b1r <= 0; b1i <= 0; b2r <= 0; b2i <= 0;
      m1_rr <= 0; m1_ii <= 0; m1_ri <= 0; m1_ir <= 0;
      m2_rr <= 0; m2_ii <= 0; m2_ri <= 0; m2_ir <= 0;
      s1_re <= 0; s1_im <= 0; s2_re <= 0; s2_im <= 0;
      t_re  <= 0; t_im  <= 0;
      vpipe     <= 0;
      out_valid <= 1'b0;
      y_re      <= 12'sd0;
      y_im      <= 12'sd0;
      out_sat   <= 1'b0;
    end else begin
      // 1
      a1r <= x1_re;  a1i <= x1_im;  a2r <= x2_re;  a2i <= x2_im;
      b1r <= w1_re;  b1i <= w1_im;  b2r <= w2_re;  b2i <= w2_im;

      // 2
      m1_rr <= b1r * a1r;   m1_ii <= b1i * a1i;
      m1_ri <= b1r * a1i;   m1_ir <= b1i * a1r;
      m2_rr <= b2r * a2r;   m2_ii <= b2i * a2i;
      m2_ri <= b2r * a2i;   m2_ir <= b2i * a2r;

      // 3
      s1_re <= m1_rr + m1_ii;
      s1_im <= m1_ri - m1_ir;
      s2_re <= m2_rr + m2_ii;
      s2_im <= m2_ri - m2_ir;

      // 4
      t_re <= tot_re >>> 16;
      t_im <= tot_im >>> 16;

      // 5
      y_re    <= re_hi ? 12'sd2047 : re_lo ? -12'sd2048 : t_re[11:0];
      y_im    <= im_hi ? 12'sd2047 : im_lo ? -12'sd2048 : t_im[11:0];
      out_sat <= vpipe[LATENCY-2] & (re_hi | re_lo | im_hi | im_lo);

      vpipe     <= {vpipe[LATENCY-3:0], in_valid};
      out_valid <= vpipe[LATENCY-2];
    end
  end

endmodule
