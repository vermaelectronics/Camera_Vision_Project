// ============================================================================
//  lcmv_crpa_model.v  --  simulation model of the Vitis HLS IP lcmv_crpa
//
//  Same ports as the RTL that Vitis HLS exports from Source/HLS/lcmv/
//  lcmv_crpa.cpp (ap_ctrl_none, axis in1/in2/out_r, ap_none scalars) and the
//  same arithmetic, bit for bit:
//    y   = conj(w1) x1 + conj(w2) x2, (y + 2^15) >> 16, saturated to 12 bits
//    R   = 4096-pair block averages in Q.4
//  Timing like the HLS core: II = 2 (TREADY low every other clock) and a
//  fixed latency, so the gnss_passthrough handshake is exercised.
//  For simulation only; the bitstream uses the HLS-generated RTL.
// ============================================================================
`timescale 1ns/100ps

module lcmv_crpa_model (
  input             ap_clk,
  input             ap_rst_n,
  input      [31:0] in1_TDATA,
  input             in1_TVALID,
  output            in1_TREADY,
  input      [31:0] in2_TDATA,
  input             in2_TVALID,
  output            in2_TREADY,
  output reg [31:0] out_r_TDATA = 32'd0,
  output reg        out_r_TVALID = 1'b0,
  input             out_r_TREADY,
  input      [31:0] w1_re,
  input      [31:0] w1_im,
  input      [31:0] w2_re,
  input      [31:0] w2_im,
  output reg [31:0] cov_r11 = 32'd0,
  output reg [31:0] cov_r22 = 32'd0,
  output reg [31:0] cov_r12_re = 32'd0,
  output reg [31:0] cov_r12_im = 32'd0,
  output reg [31:0] cov_blocks = 32'd0,
  output reg [31:0] sat_count = 32'd0
);
  localparam LOG2N = 12, SH = LOG2N - 4;

  reg phase = 1'b0;                          // II = 2
  assign in1_TREADY = ap_rst_n & ~phase & in2_TVALID;
  assign in2_TREADY = ap_rst_n & ~phase & in1_TVALID;
  wire take = in1_TVALID & in1_TREADY;       // both streams read together

  wire signed [11:0] x1r = in1_TDATA[11:0],  x1i = in1_TDATA[27:16];
  wire signed [11:0] x2r = in2_TDATA[11:0],  x2i = in2_TDATA[27:16];
  wire signed [24:0] a1r = w1_re[24:0], a1i = w1_im[24:0];
  wire signed [24:0] a2r = w2_re[24:0], a2i = w2_im[24:0];

  wire signed [39:0] yr = a1r * x1r + a1i * x1i + a2r * x2r + a2i * x2i;
  wire signed [39:0] yi = a1r * x1i - a1i * x1r + a2r * x2i - a2i * x2r;
  wire signed [40:0] yrs = ($signed({yr[39], yr}) + 41'sd32768) >>> 16;
  wire signed [40:0] yis = ($signed({yi[39], yi}) + 41'sd32768) >>> 16;
  wire cr = (yrs > 41'sd2047) || (yrs < -41'sd2048);
  wire ci = (yis > 41'sd2047) || (yis < -41'sd2048);
  wire signed [11:0] or12 = (yrs > 41'sd2047) ? 12'sd2047 : (yrs < -41'sd2048) ? -12'sd2048 : yrs[11:0];
  wire signed [11:0] oi12 = (yis > 41'sd2047) ? 12'sd2047 : (yis < -41'sd2048) ? -12'sd2048 : yis[11:0];

  wire signed [24:0] p11  = x1r * x1r + x1i * x1i;
  wire signed [24:0] p22  = x2r * x2r + x2i * x2i;
  wire signed [24:0] p12r = x1r * x2r + x1i * x2i;
  wire signed [24:0] p12i = x1i * x2r - x1r * x2i;

  reg signed [47:0] acc11 = 0, acc22 = 0, acc12r = 0, acc12i = 0;
  reg [LOG2N-1:0]   cnt = 0;
  wire signed [47:0] s11 = acc11 + p11, s22 = acc22 + p22;
  wire signed [47:0] s12r = acc12r + p12r, s12i = acc12i + p12i;

  // latency: output 3 clocks after the read
  reg [31:0] d1 = 0, d2 = 0;
  reg        v1 = 0, v2 = 0;

  always @(posedge ap_clk) begin
    if (!ap_rst_n) begin
      phase <= 1'b0; v1 <= 0; v2 <= 0; out_r_TVALID <= 1'b0;
      acc11 <= 0; acc22 <= 0; acc12r <= 0; acc12i <= 0; cnt <= 0;
      cov_r11 <= 0; cov_r22 <= 0; cov_r12_re <= 0; cov_r12_im <= 0;
      cov_blocks <= 0; sat_count <= 0;
    end else begin
      phase <= take ? 1'b1 : 1'b0;
      v1 <= take;  d1 <= {{4{oi12[11]}}, oi12, {4{or12[11]}}, or12};
      v2 <= v1;    d2 <= d1;
      out_r_TVALID <= v2;  out_r_TDATA <= d2;
      if (take) begin
        if (cr || ci) sat_count <= sat_count + 1;
        if (&cnt) begin
          cov_r11    <= s11[SH+31:SH];
          cov_r22    <= s22[SH+31:SH];
          cov_r12_re <= s12r[SH+31:SH];
          cov_r12_im <= s12i[SH+31:SH];
          cov_blocks <= cov_blocks + 1;
          acc11 <= 0; acc22 <= 0; acc12r <= 0; acc12i <= 0;
        end else begin
          acc11 <= s11; acc22 <= s22; acc12r <= s12r; acc12i <= s12i;
        end
        cnt <= cnt + 1'b1;
      end
    end
  end
endmodule
