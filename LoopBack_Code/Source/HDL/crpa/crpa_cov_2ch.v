// ============================================================================
//  crpa_cov_2ch.v
//  2x2 sample covariance estimator for the LCMV CRPA (sample-clock domain).
//
//  The LCMV of R. Li, X. Zhao, X. W. Shi (PIER C vol. 4, 2008) is
//      w = R^-1 C (C^H R^-1 C)^-1 f,      R = E[x x^H]
//  This block estimates R from the array snapshots x = [x1, x2]^T over a
//  block of N = 2^LOG2N sample pairs:
//
//      R11 = 1/N sum |x1|^2                       (real)
//      R22 = 1/N sum |x2|^2                       (real)
//      R12 = 1/N sum x1 conj(x2)                  (complex; R21 = conj(R12))
//
//  The PS reads one block (R11, R22, R12, block count), solves the LCMV
//  weights and writes them back (gnss_passthrough registers 0x80..0x8C).
//
//  FORMAT
//    Outputs are block AVERAGES in Q.4 (LSB^2 * 16) of the raw 12-bit ADC
//    samples. Full scale |x|^2 = 2 * 2048^2 = 2^23, so Q.4 needs 28 bits.
//    All four words change together on the clock 'blocks' increments.
//
//  Original work for the ANTSDR E310 V1 GNSS CRPA project.
// ============================================================================

`timescale 1ns/100ps

module crpa_cov_2ch #(
  parameter LOG2N = 12                    // block = 4096 pairs (133 us @ 30.72 MSPS)
) (
  input                    clk,
  input                    rst,
  input                    in_valid,
  input  signed [11:0]     x1_re,
  input  signed [11:0]     x1_im,
  input  signed [11:0]     x2_re,
  input  signed [11:0]     x2_im,

  output reg        [31:0] r11,           // Q.4, unsigned
  output reg        [31:0] r22,           // Q.4, unsigned
  output reg signed [31:0] r12_re,        // Q.4
  output reg signed [31:0] r12_im,        // Q.4
  output reg        [31:0] blocks         // completed blocks
);

  localparam SH = LOG2N - 4;              // average, keep 4 fractional bits

  // ---- stage 1: register the snapshot ---------------------------------------
  reg               v1 = 1'b0;
  reg signed [11:0] a, b, c, d;           // x1 = a + jb, x2 = c + jd

  // ---- stage 2: the eight 12x12 products (one DSP48 each) ---------------------
  reg               vm = 1'b0;
  reg signed [23:0] m_aa, m_bb, m_cc, m_dd, m_ac, m_bd, m_bc, m_ad;

  // ---- stage 3: sums ----------------------------------------------------------
  reg               v2 = 1'b0;
  reg signed [24:0] p11, p22;             // |x1|^2, |x2|^2   (<= 2^23)
  reg signed [24:0] p12r, p12i;           // x1 conj(x2) = (ac+bd) + j(bc-ad)

  // ---- stage 4: block accumulation -------------------------------------------
  reg signed [47:0] acc11, acc22, acc12r, acc12i;
  reg  [LOG2N-1:0]  cnt;

  wire signed [47:0] s11  = acc11  + p11;
  wire signed [47:0] s22  = acc22  + p22;
  wire signed [47:0] s12r = acc12r + p12r;
  wire signed [47:0] s12i = acc12i + p12i;

  always @(posedge clk) begin
    if (rst) begin
      v1 <= 1'b0; vm <= 1'b0; v2 <= 1'b0;
      a <= 12'sd0; b <= 12'sd0; c <= 12'sd0; d <= 12'sd0;
      p11 <= 25'sd0; p22 <= 25'sd0; p12r <= 25'sd0; p12i <= 25'sd0;
      acc11 <= 48'sd0; acc22 <= 48'sd0; acc12r <= 48'sd0; acc12i <= 48'sd0;
      cnt <= {LOG2N{1'b0}};
      r11 <= 32'd0; r22 <= 32'd0; r12_re <= 32'sd0; r12_im <= 32'sd0;
      blocks <= 32'd0;
    end else begin
      v1 <= in_valid;
      if (in_valid) begin
        a <= x1_re; b <= x1_im; c <= x2_re; d <= x2_im;
      end

      vm <= v1;
      if (v1) begin
        m_aa <= a * a;  m_bb <= b * b;
        m_cc <= c * c;  m_dd <= d * d;
        m_ac <= a * c;  m_bd <= b * d;
        m_bc <= b * c;  m_ad <= a * d;
      end

      v2 <= vm;
      if (vm) begin
        p11  <= m_aa + m_bb;
        p22  <= m_cc + m_dd;
        p12r <= m_ac + m_bd;
        p12i <= m_bc - m_ad;
      end

      if (v2) begin
        if (cnt == {LOG2N{1'b1}}) begin
          r11    <= s11[SH+31:SH];
          r22    <= s22[SH+31:SH];
          r12_re <= s12r[SH+31:SH];
          r12_im <= s12i[SH+31:SH];
          blocks <= blocks + 1'b1;
          acc11 <= 48'sd0; acc22 <= 48'sd0; acc12r <= 48'sd0; acc12i <= 48'sd0;
        end else begin
          acc11 <= s11; acc22 <= s22; acc12r <= s12r; acc12i <= s12i;
        end
        cnt <= cnt + 1'b1;
      end
    end
  end

endmodule
