// ============================================================================
//  tb_gnss_passthrough_lcmv.v
//  System-level, self-checking testbench for gnss_passthrough v4.0 with the
//  LCMV CRPA HLS IP lcmv_0 (modelled by lcmv_crpa_model.v, same ports and
//  arithmetic as Source/HLS/lcmv/lcmv_crpa.cpp). Exercises the block exactly
//  as the firmware does: AXI4-Lite (100 MHz) and ADC/DAC strobes (125 MHz).
//
//  Scene: 2-element array, d = lambda/2, a(theta) = [1, exp(j*pi*sin(theta))]
//    jammer   : noise-like, 40 deg, 400 LSB rms per I/Q (JNR ~ 46 dB)
//    GPS-like : CW tone at 10 deg, 20 LSB  (stands in for the look direction)
//    noise    : 2 LSB rms per I/Q per channel
//
//  Tests
//    T1  ID, VERSION 0x00040000, angle registers reset 30/60 (PS storage)
//    T2  LCMV enabled but no PS weights committed yet: TX0 stays RX1
//    T3  Jammer-angle LCMV (paper eq. 9, GPS 10 / jammer 40): PS weights over
//        AXI -> lcmv_0: residual < -40 dB, tone 0 dB, TX0 left-aligned
//    T4  Covariance LCMV exactly as the firmware runs it: read R (0x90..0xA0)
//        from lcmv_0, solve, write 0x80..0x8C: < -40 dB, tone 0 dB
//    T5  Jammer moves to -25 deg, re-read R, re-solve: null follows
//    T6  LCMV off: TX0 = RX1, TX2 = RX2, no FIFO over/underflow, no drop
//
//  Run (from LoopBack_Code/Sim):
//    iverilog -g2012 -o tb.vvp -s tb_gnss_passthrough_lcmv \
//      ../Source/HDL/gnss_passthrough.v lcmv_crpa_model.v \
//      tb_gnss_passthrough_lcmv.v && vvp -n tb.vvp
// ============================================================================

`timescale 1ns/100ps

module tb_gnss_passthrough_lcmv;

  localparam real PI        = 3.14159265358979;
  localparam real JAM_AMP   = 400.0;
  localparam real TONE_AMP  = 20.0;
  localparam real NOISE_AMP = 2.0;
  localparam real TONE_CYC  = 0.0371;     // cycles per sample
  localparam integer DECIM  = 4;          // one sample every 4 clk

  // ---- clocks / resets -----------------------------------------------------
  reg clk = 0;        always #4 clk = ~clk;          // 125 MHz l_clk
  reg aclk = 0;       always #5 aclk = ~aclk;        // 100 MHz sys_cpu_clk
  reg rst = 1;
  reg aresetn = 0;

  // ---- DUT ports --------------------------------------------------------------
  reg  [15:0] adc_i0 = 0, adc_q0 = 0, adc_i1 = 0, adc_q1 = 0;
  reg         adc_v  = 0;
  reg         dac_v  = 0;
  wire [15:0] dac_i0, dac_q0, dac_i1, dac_q1;

  reg  [15:0] awaddr = 0, araddr = 0;
  reg         awvalid = 0, wvalid = 0, bready = 1, arvalid = 0, rready = 1;
  reg  [31:0] wdata = 0;
  wire        awready, wready, bvalid, arready, rvalid;
  wire [1:0]  bresp, rresp;
  wire [31:0] rdata;

  // ---- lcmv_0 (Vitis HLS IP) model ---------------------------------------------
  wire [31:0] l1d, l2d, lyd, lw1r, lw1i, lw2r, lw2i;
  wire [31:0] lc11, lc22, lc12r, lc12i, lcblk, lsat;
  wire        l1v, l1r, l2v, l2r, lyv, lyr, lrstn;

  lcmv_crpa_model lcmv_0 (
    .ap_clk(clk), .ap_rst_n(lrstn),
    .in1_TDATA(l1d), .in1_TVALID(l1v), .in1_TREADY(l1r),
    .in2_TDATA(l2d), .in2_TVALID(l2v), .in2_TREADY(l2r),
    .out_r_TDATA(lyd), .out_r_TVALID(lyv), .out_r_TREADY(lyr),
    .w1_re(lw1r), .w1_im(lw1i), .w2_re(lw2r), .w2_im(lw2i),
    .cov_r11(lc11), .cov_r22(lc22), .cov_r12_re(lc12r), .cov_r12_im(lc12i),
    .cov_blocks(lcblk), .sat_count(lsat)
  );

  gnss_passthrough dut (
    .clk(clk), .rst(rst),
    .adc_enable_i0(1'b1), .adc_valid_i0(adc_v), .adc_data_i0(adc_i0),
    .adc_enable_q0(1'b1), .adc_valid_q0(adc_v), .adc_data_q0(adc_q0),
    .adc_enable_i1(1'b1), .adc_valid_i1(adc_v), .adc_data_i1(adc_i1),
    .adc_enable_q1(1'b1), .adc_valid_q1(adc_v), .adc_data_q1(adc_q1),
    .dac_enable_i0(1'b1), .dac_valid_i0(dac_v),
    .dac_enable_q0(1'b1), .dac_valid_q0(dac_v),
    .dac_enable_i1(1'b1), .dac_valid_i1(dac_v),
    .dac_enable_q1(1'b1), .dac_valid_q1(dac_v),
    .dma_dac_data_i0(16'h1230), .dma_dac_data_q0(16'h4560),
    .dma_dac_data_i1(16'h7890), .dma_dac_data_q1(16'hABC0),
    .dac_data_i0(dac_i0), .dac_data_q0(dac_q0),
    .dac_data_i1(dac_i1), .dac_data_q1(dac_q1),
    .m_axis_lcmv1_tdata(l1d), .m_axis_lcmv1_tvalid(l1v), .m_axis_lcmv1_tready(l1r),
    .m_axis_lcmv2_tdata(l2d), .m_axis_lcmv2_tvalid(l2v), .m_axis_lcmv2_tready(l2r),
    .s_axis_lcmv_tdata(lyd), .s_axis_lcmv_tvalid(lyv), .s_axis_lcmv_tready(lyr),
    .lcmv_w1_re(lw1r), .lcmv_w1_im(lw1i), .lcmv_w2_re(lw2r), .lcmv_w2_im(lw2i),
    .lcmv_rst_n(lrstn),
    .lcmv_cov_r11(lc11), .lcmv_cov_r22(lc22), .lcmv_cov_r12_re(lc12r), .lcmv_cov_r12_im(lc12i),
    .lcmv_cov_blocks(lcblk), .lcmv_sat_count(lsat),
    .s_axi_aclk(aclk), .s_axi_aresetn(aresetn),
    .s_axi_awvalid(awvalid), .s_axi_awaddr(awaddr), .s_axi_awprot(3'd0),
    .s_axi_awready(awready),
    .s_axi_wvalid(wvalid), .s_axi_wdata(wdata), .s_axi_wstrb(4'hF),
    .s_axi_wready(wready),
    .s_axi_bvalid(bvalid), .s_axi_bresp(bresp), .s_axi_bready(bready),
    .s_axi_arvalid(arvalid), .s_axi_araddr(araddr), .s_axi_arprot(3'd0),
    .s_axi_arready(arready),
    .s_axi_rvalid(rvalid), .s_axi_rresp(rresp), .s_axi_rdata(rdata),
    .s_axi_rready(rready)
  );

  // ---- AXI4-Lite master tasks ------------------------------------------------
  task axi_write(input [15:0] a, input [31:0] d);
    begin
      @(posedge aclk);
      awaddr <= a; wdata <= d; awvalid <= 1; wvalid <= 1;
      @(posedge aclk);
      while (!awready) @(posedge aclk);
      awvalid <= 0; wvalid <= 0;
      while (!bvalid) @(posedge aclk);
      @(posedge aclk);
    end
  endtask

  task axi_read(input [15:0] a, output [31:0] d);
    begin
      @(posedge aclk);
      araddr <= a; arvalid <= 1;
      @(posedge aclk);
      while (!arready) @(posedge aclk);
      arvalid <= 0;
      while (!rvalid) @(posedge aclk);
      d = rdata;
      @(posedge aclk);
    end
  endtask

  // ---- scene generator (sample clock) ----------------------------------------
  integer seed = 777;
  real    jam_deg  = 40.0;
  real    tone_deg = 10.0;
  real    rx2_err_deg  = 0.0;    // RX2 channel phase error (hardware mismatch)
  real    rx2_err_gain = 1.0;    // RX2 channel gain error
  integer n = 0;
  integer div = 0;

  function real gauss(input integer dummy);
    gauss = $dist_normal(seed, 0, 10000) / 10000.0;
  endfunction

  function [15:0] adc12(input real v);       // 12-bit, right-aligned, sign-ext
    real r;
    integer k;
    begin
      r = (v >= 0.0) ? v + 0.5 : v - 0.5;
      if (r > 2047.0)  r = 2047.0;
      if (r < -2048.0) r = -2048.0;
      k = $rtoi(r);
      adc12 = {{4{k[11]}}, k[11:0]};
    end
  endfunction

  real jr, ji, tr, ti, pj, pt, x2r, x2i, er, ei, x1r, x1i;

  always @(posedge clk) begin
    adc_v <= 1'b0;
    dac_v <= 1'b0;
    div   <= (div == DECIM-1) ? 0 : div + 1;

    if (div == 0) begin
      jr = JAM_AMP * gauss(0);
      ji = JAM_AMP * gauss(0);
      tr = TONE_AMP * $cos(2.0*PI*TONE_CYC*n);
      ti = TONE_AMP * $sin(2.0*PI*TONE_CYC*n);
      pj = PI * $sin(jam_deg  * PI / 180.0);
      pt = PI * $sin(tone_deg * PI / 180.0);

      x1r = jr + tr + NOISE_AMP * gauss(0);
      x1i = ji + ti + NOISE_AMP * gauss(0);
      // element 2: jammer and tone with their spatial phases
      x2r = (jr*$cos(pj) - ji*$sin(pj)) + (tr*$cos(pt) - ti*$sin(pt));
      x2i = (jr*$sin(pj) + ji*$cos(pj)) + (tr*$sin(pt) + ti*$cos(pt));
      // RX2 channel phase error
      er  = rx2_err_gain * (x2r*$cos(rx2_err_deg*PI/180.0) - x2i*$sin(rx2_err_deg*PI/180.0));
      ei  = rx2_err_gain * (x2r*$sin(rx2_err_deg*PI/180.0) + x2i*$cos(rx2_err_deg*PI/180.0));

      adc_i0 <= adc12(x1r);
      adc_q0 <= adc12(x1i);
      adc_i1 <= adc12(er + NOISE_AMP * gauss(0));
      adc_q1 <= adc12(ei + NOISE_AMP * gauss(0));
      adc_v  <= 1'b1;
      n = n + 1;
    end
    if (div == 2) dac_v <= 1'b1;            // DAC request, different phase
  end

  // ---- measurement on the DAC side --------------------------------------------
  real    p_in, p_out0, p_out1, c_re, c_im;
  integer m_in, m_out, k_out;
  integer nib_err;
  reg     measuring = 0;
  real    yr, yi, y1r, y1i;

  always @(posedge clk) begin
    if (measuring && adc_v) begin
      p_in = p_in + 1.0*$signed(adc_i0[11:0])*$signed(adc_i0[11:0])
                  + 1.0*$signed(adc_q0[11:0])*$signed(adc_q0[11:0]);
      m_in = m_in + 1;
    end
    if (measuring && dac_v) begin
      yr  = $signed(dac_i0[15:4]);
      yi  = $signed(dac_q0[15:4]);
      y1r = $signed(dac_i1[15:4]);
      y1i = $signed(dac_q1[15:4]);
      if (dac_i0[3:0] != 0 || dac_q0[3:0] != 0) nib_err = nib_err + 1;
      p_out0 = p_out0 + yr*yr + yi*yi;
      p_out1 = p_out1 + y1r*y1r + y1i*y1i;
      // tone correlation (TX index k_out)
      c_re = c_re + yr*$cos(2.0*PI*TONE_CYC*k_out) + yi*$sin(2.0*PI*TONE_CYC*k_out);
      c_im = c_im + yi*$cos(2.0*PI*TONE_CYC*k_out) - yr*$sin(2.0*PI*TONE_CYC*k_out);
      k_out = k_out + 1;
      m_out = m_out + 1;
    end
  end

  real out_db, out1_db, tone_db, res_db, tone_pow;

  task measure(input integer nsamp);
    begin
      p_in = 0; p_out0 = 0; p_out1 = 0; c_re = 0; c_im = 0;
      m_in = 0; m_out = 0; k_out = 0; nib_err = 0;
      measuring = 1;
      repeat (nsamp * DECIM) @(posedge clk);
      measuring = 0;
      out_db  = 10.0*$log10((p_out0/m_out + 1e-9) / (p_in/m_in));
      out1_db = 10.0*$log10((p_out1/m_out + 1e-9) / (p_in/m_in));
      tone_db = 20.0*$log10($sqrt(c_re*c_re + c_im*c_im) / m_out / TONE_AMP + 1e-12);
      // residual = output minus the (wanted) look-direction tone
      tone_pow = (c_re*c_re + c_im*c_im) / (1.0*m_out*m_out);
      res_db  = 10.0*$log10((p_out0/m_out - tone_pow + 1e-9) / (p_in/m_in));
    end
  endtask

  // ---- checks -----------------------------------------------------------------
  integer errors = 0;
  reg [31:0] r;

  task check(input cond, input [8*64-1:0] what);
    begin
      if (!cond) begin
        errors = errors + 1;
        $display("  FAIL: %0s", what);
      end
    end
  endtask

  reg signed [31:0] w1r, w1i, w2r, w2i;

  // ---- PS model: covariance LCMV (same maths as lcmv_math.c) ------------------
  real cr11, cr22, cr12r, cr12i, lmin, lload, ca, cd, cdet;
  real s2r, s2i, b1r, b1i, b2r, b2i, g;
  real ww1r, ww1i, ww2r, ww2i, jnr_db;
  reg [31:0] blk0, blk1, cr;

  task read_cov;
    begin
      axi_read(16'hA0, blk0);
      axi_read(16'h90, cr);  cr11  = cr / 16.0;
      axi_read(16'h94, cr);  cr22  = cr / 16.0;
      axi_read(16'h98, cr);  cr12r = $signed(cr) / 16.0;
      axi_read(16'h9C, cr);  cr12i = $signed(cr) / 16.0;
      axi_read(16'hA0, blk1);
    end
  endtask

  task wait_new_block;
    reg [31:0] b;
    begin
      axi_read(16'hA0, b);
      blk0 = b;
      while (blk0 == b) begin
        repeat (2000) @(posedge clk);
        axi_read(16'hA0, blk0);
      end
      // and one more full block measured entirely in the new scene
      b = blk0;
      while (blk0 == b) begin
        repeat (2000) @(posedge clk);
        axi_read(16'hA0, blk0);
      end
    end
  endtask

  function integer q16(input real v);
    q16 = $rtoi(v * 65536.0 + ((v >= 0) ? 0.5 : -0.5));
  endfunction

  task solve_and_load(input real look_deg);
    begin
      // eigenvalues, loading at the noise floor
      lmin  = 0.5 * (cr11 + cr22 - $sqrt((cr11-cr22)*(cr11-cr22) + 4.0*(cr12r*cr12r + cr12i*cr12i)));
      lload = (lmin > 1e-6*(cr11+cr22)) ? lmin : 1e-6*(cr11+cr22);
      jnr_db = 10.0*$log10(0.5*(cr11+cr22+$sqrt((cr11-cr22)*(cr11-cr22)
                + 4.0*(cr12r*cr12r + cr12i*cr12i))) / lload);
      ca = cr11 + lload;  cd = cr22 + lload;
      cdet = ca*cd - (cr12r*cr12r + cr12i*cr12i);
      // a_s = [1, s], R^-1 a_s = 1/det [cd - R12 s, ca s - conj(R12)]
      s2r = $cos(PI*$sin(look_deg*PI/180.0));
      s2i = $sin(PI*$sin(look_deg*PI/180.0));
      b1r = (cd - (cr12r*s2r - cr12i*s2i)) / cdet;
      b1i = (   - (cr12r*s2i + cr12i*s2r)) / cdet;
      b2r = (ca*s2r - cr12r) / cdet;
      b2i = (ca*s2i + cr12i) / cdet;
      g   = b1r + (s2r*b2r + s2i*b2i);          // a_s^H R^-1 a_s (real)
      ww1r = b1r / g;  ww1i = b1i / g;
      ww2r = b2r / g;  ww2i = b2i / g;
      $display("     R11 %0.0f R22 %0.0f R12 %0.0f%+0.0fj LSB^2, JNR %0.1f dB, w1 %0.5f%+0.5fj w2 %0.5f%+0.5fj",
               cr11, cr22, cr12r, cr12i, jnr_db, ww1r, ww1i, ww2r, ww2i);
      axi_write(16'h80, q16(ww1r));
      axi_write(16'h84, q16(ww1i));
      axi_write(16'h88, q16(ww2r));
      axi_write(16'h8C, q16(ww2i));             // commit
    end
  endtask

  task load_weights(input real a1r, input real a1i, input real a2r, input real a2i);
    begin
      axi_write(16'h80, q16(a1r));
      axi_write(16'h84, q16(a1i));
      axi_write(16'h88, q16(a2r));
      axi_write(16'h8C, q16(a2i));             // commit
    end
  endtask

  initial begin
    repeat (20) @(posedge aclk);
    aresetn = 1;
    repeat (5) @(posedge clk);
    rst = 0;
    repeat (50) @(posedge clk);

    // ---------------- T1 ----------------
    axi_read(16'h00, r);  $display("T1 ID      = 0x%08h", r);
    check(r == 32'h47435031, "ID");
    axi_read(16'h04, r);  $display("T1 VERSION = 0x%08h", r);
    check(r == 32'h00040000, "VERSION 4.0");
    axi_read(16'h40, r);  check(r == 32'd30, "GPS angle register reset 30");
    axi_read(16'h44, r);  check(r == 32'd60, "jammer angle register reset 60");

    // ---------------- T2 ----------------
    axi_write(16'h0C, 32'h31);                 // pass_en | lcmv_en | lcmv_sw
    repeat (3000) @(posedge clk);
    axi_read(16'h10, r);
    $display("T2 STATUS = 0x%08h (lcmv_en=%0d lcmv_active=%0d) - no PS weights yet", r, r[17], r[19]);
    check(r[17] == 1'b1 && r[19] == 1'b0, "not active before the first PS weight set");
    measure(3000);
    $display("T2 TX0/RX1 power %6.2f dB (identity until weights arrive)", out_db);
    check(out_db > -0.2 && out_db < 0.2, "TX0 = RX1 before weights");

    // ---------------- T3 ----------------
    // paper eq. 9, look 10 deg + null 40 deg, eps 0: w = C^-H [1 0]^T
    load_weights(0.50000, 0.55098, 0.71333, -0.21156);
    repeat (1000) @(posedge clk);
    axi_read(16'h10, r);
    check(r[19] == 1'b1, "STATUS[19] lcmv_active after the commit");
    axi_read(16'h54, w1r); axi_read(16'h58, w1i);
    axi_read(16'h5C, w2r); axi_read(16'h60, w2i);
    $display("T3 weights at lcmv_0: w1 = %0.5f %+0.5fj  w2 = %0.5f %+0.5fj",
             w1r/65536.0, w1i/65536.0, w2r/65536.0, w2i/65536.0);
    measure(6000);
    $display("T3 jammer-angle LCMV: residual %6.1f dB re RX1, tone gain %5.2f dB, nibble errors %0d",
             res_db, tone_db, nib_err);
    check(res_db < -40.0, "jammer-angle null < -40 dB");
    check(tone_db > -1.0 && tone_db < 1.0, "look gain 0 dB +/- 1 dB");
    check(nib_err == 0, "TX0 left-aligned");
    $display("T3 TX2 power re RX1 %6.2f dB (TX2 = RX2 unprocessed)", out1_db);
    check(out1_db > -0.5 && out1_db < 0.5, "TX2 carries RX2");

    // ---------------- T4 ----------------
    wait_new_block;
    read_cov;
    $display("T4 covariance blocks %0d..%0d (coherent read: %0d)", blk0, blk1, blk0 == blk1);
    check(blk0 == blk1, "coherent covariance read");
    check(cr11 > 250000.0 && cr11 < 400000.0, "R11 = jammer + tone + noise power");
    solve_and_load(10.0);
    repeat (1000) @(posedge clk);
    measure(6000);
    $display("T4 covariance LCMV: residual %6.1f dB re RX1, tone gain %5.2f dB", res_db, tone_db);
    check(jnr_db > 20.0, "jammer detected from the eigenvalues");
    check(res_db < -40.0, "covariance LCMV null < -40 dB");
    check(tone_db > -1.0 && tone_db < 1.0, "covariance LCMV look gain 0 dB");

    // ---------------- T5 ----------------
    jam_deg = -25.0;
    wait_new_block;
    read_cov;
    solve_and_load(10.0);
    repeat (1000) @(posedge clk);
    measure(6000);
    $display("T5 jammer moved to -25 deg, re-solved from R: residual %6.1f dB, tone gain %5.2f dB",
             res_db, tone_db);
    check(res_db < -40.0, "null follows the jammer");
    check(tone_db > -1.0 && tone_db < 1.0, "look gain 0 dB after the move");
    axi_read(16'h68, r);  $display("LCMV_UPDATES (PS weight sets) = %0d", r);
    check(r == 3, "three PS weight sets committed");

    // ---------------- T6 ----------------
    axi_write(16'h0C, 32'h01);                 // LCMV off
    repeat (1000) @(posedge clk);
    measure(4000);
    $display("T6 LCMV off: TX0/RX1 power %6.2f dB, nibble errors %0d", out_db, nib_err);
    check(out_db > -0.2 && out_db < 0.2, "identity passthrough");
    axi_read(16'h10, r);
    $display("T6 STATUS = 0x%08h", r);
    check(r[19] == 1'b0, "STATUS[19] clear");
    check(r[10] == 1'b0, "no RX pair dropped at lcmv_0");
    axi_read(16'h1C, r);  $display("OVERFLOW_COUNT = %0d", r);
    check(r == 0, "no FIFO overflow");
    axi_read(16'h20, r);  $display("UNDERFLOW_COUNT = %0d", r);
    check(r == 0, "no FIFO underflow");

    if (errors == 0) $display("PASS");
    else             $display("FAIL (%0d errors)", errors);
    $finish;
  end

endmodule
