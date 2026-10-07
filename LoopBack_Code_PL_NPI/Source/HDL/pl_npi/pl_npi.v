// ============================================================================
//  pl_npi.v  --  block-design wrapper for the PL-NPI CRPA core
//  ANTSDR E310 V1 / GNSS CRPA project
// ----------------------------------------------------------------------------
//  Packages rtl/pi_power_inversion_pl_npi.v (PL-NPI: Piecewise-Linear
//  Normalized Power Inversion, Jia et al., IEEE Access vol. 11, 2023, Eq. 11)
//  as its own block-design IP, pl_npi_0, next to gnss_passthrough -- the same
//  arrangement as the PI-NLMS core (pi_nlms_0). The core files under rtl/ are
//  used UNCHANGED; this wrapper only adapts the ports.
//
//  PORTS
//    s_axis_x   AXI4-Stream in, 64 bits: {rx2_q, rx2_i, rx1_q, rx1_i}, each a
//               signed 16-bit sample at the raw ADC scale (12-bit RX samples
//               sign-extended, NOT shifted up). Element 0 = RX1 is the
//               reference element: the quiescent weight vector is [1, 0].
//    m_axis_y   AXI4-Stream out, 32 bits: {y_q, y_i}, the array output
//               s(n) = sum_i w_i x_i at the same raw ADC scale, saturated to
//               signed 16 bits.
//    gamma      regulariser of mu_NPI = 1/(x^T x + gamma); 0 is treated as 1
//               (the core divides by 2*x^T x + gamma, which must be >= 1).
//    adapt_en   1 = weights adapt, 0 = weights frozen at their current value.
//    gain_band  diagnostic: Eq. 11 gain segment of the last sample
//               (0/1/2/3 = x1.00/1.05/1.10/1.20).
//
//  FLOW CONTROL
//    The core takes a sample on every cycle s_axis_x_tvalid is high and never
//    stalls, so s_axis_x_tready is tied high. Its output cannot be held
//    either, so m_axis_y_tready is ignored: the receiver must accept every
//    beat (gnss_passthrough ties it high).
//
//  CLOCK / RESET
//    aclk is the AD9361 sample clock (axi_ad9361/l_clk). aresetn is
//    synchronous, active low; gnss_passthrough holds it low while the core is
//    switched off, so enabling it restarts from the quiescent weights.
// ============================================================================
`timescale 1ns/1ps

module pl_npi (
  input  wire        aclk,
  input  wire        aresetn,

  input  wire [63:0] s_axis_x_tdata,
  input  wire        s_axis_x_tvalid,
  output wire        s_axis_x_tready,

  output wire [31:0] m_axis_y_tdata,
  output wire        m_axis_y_tvalid,
  input  wire        m_axis_y_tready,

  input  wire [31:0] gamma,
  input  wire        adapt_en,
  output wire [1:0]  gain_band
);

  localparam integer M           = 2;
  localparam integer DATA_W      = 16;
  localparam integer WEIGHT_W    = 32;
  localparam integer WEIGHT_FRAC = 20;
  // Widths the core derives from the parameters above (see its header).
  localparam integer S_W         = (DATA_W + WEIGHT_W + 1) + 1;   // 50
  localparam integer GAMMA_W     = 2*DATA_W + 2 + 2;              // 36

  wire [15:0] rx1_i = s_axis_x_tdata[15:0];
  wire [15:0] rx1_q = s_axis_x_tdata[31:16];
  wire [15:0] rx2_i = s_axis_x_tdata[47:32];
  wire [15:0] rx2_q = s_axis_x_tdata[63:48];

  wire [31:0]        gamma_nz = (gamma == 32'd0) ? 32'd1 : gamma;
  wire [GAMMA_W-1:0] gamma_in = {{(GAMMA_W-32){1'b0}}, gamma_nz};

  wire signed [S_W-1:0] s_re, s_im;
  wire                  s_valid;

  pi_power_inversion_pl_npi #(
    .M           (M),
    .DATA_W      (DATA_W),
    .WEIGHT_W    (WEIGHT_W),
    .WEIGHT_FRAC (WEIGHT_FRAC)
  ) u_core (
    .clk             (aclk),
    .aresetn         (aresetn),
    .x_re            ({rx2_i, rx1_i}),
    .x_im            ({rx2_q, rx1_q}),
    .sample_valid    (s_axis_x_tvalid),
    .gamma_in        (gamma_in),
    .adapt_en        (adapt_en),
    .s_re            (s_re),
    .s_im            (s_im),
    .s_valid         (s_valid),
    .w_re            (),
    .w_im            (),
    .weights_valid   (),
    .alpha_mag       (),
    .alpha_mag_valid (),
    .pl_gain_band    (gain_band)
  );

  // s is Q(S_W-WEIGHT_FRAC).WEIGHT_FRAC: drop the fraction to get back to
  // the input sample scale, then saturate to signed 16 bits.
  function [15:0] sat16(input signed [S_W-1:0] v);
    reg signed [S_W-1:0] t;
    begin
      t = v >>> WEIGHT_FRAC;
      if      (t >  $signed(32767))  sat16 = 16'h7FFF;
      else if (t < -$signed(32768))  sat16 = 16'h8000;
      else                           sat16 = t[15:0];
    end
  endfunction

  assign s_axis_x_tready = 1'b1;
  assign m_axis_y_tdata  = {sat16(s_im), sat16(s_re)};
  assign m_axis_y_tvalid = s_valid;

endmodule
