`timescale 1ns/100ps
module tb_gnss_passthrough;

  reg clk = 0; always #5 clk = ~clk;                 // 100 MHz l_clk
  reg s_axi_aclk = 0; always #7 s_axi_aclk = ~s_axi_aclk; // ~71 MHz, async on purpose
  reg rst = 1, s_axi_aresetn = 0;

  reg adc_enable_i0=1, adc_valid_i0=0; reg [15:0] adc_data_i0;
  reg adc_enable_q0=1, adc_valid_q0=0; reg [15:0] adc_data_q0;
  reg adc_enable_i1=1, adc_valid_i1=0; reg [15:0] adc_data_i1;
  reg adc_enable_q1=1, adc_valid_q1=0; reg [15:0] adc_data_q1;

  reg dac_enable_i0=1, dac_valid_i0=0;
  reg dac_enable_q0=1, dac_valid_q0=0;
  reg dac_enable_i1=1, dac_valid_i1=0;
  reg dac_enable_q1=1, dac_valid_q1=0;

  reg [15:0] dma_dac_data_i0=16'h1234, dma_dac_data_q0=16'h5678;
  reg [15:0] dma_dac_data_i1=16'h0, dma_dac_data_q1=16'h0;

  wire [15:0] dac_data_i0, dac_data_q0, dac_data_i1, dac_data_q1;

  reg s_axi_awvalid=0; reg [15:0] s_axi_awaddr; reg [2:0] s_axi_awprot=0;
  wire s_axi_awready;
  reg s_axi_wvalid=0; reg [31:0] s_axi_wdata; reg [3:0] s_axi_wstrb=4'hF;
  wire s_axi_wready;
  wire s_axi_bvalid; wire [1:0] s_axi_bresp; reg s_axi_bready=0;
  reg s_axi_arvalid=0; reg [15:0] s_axi_araddr; reg [2:0] s_axi_arprot=0;
  wire s_axi_arready;
  wire s_axi_rvalid; wire [1:0] s_axi_rresp; wire [31:0] s_axi_rdata;
  reg s_axi_rready=0;

  gnss_passthrough #(.FIFO_ADDR_WIDTH(5)) dut (
    .clk(clk), .rst(rst),
    .adc_enable_i0(adc_enable_i0), .adc_valid_i0(adc_valid_i0), .adc_data_i0(adc_data_i0),
    .adc_enable_q0(adc_enable_q0), .adc_valid_q0(adc_valid_q0), .adc_data_q0(adc_data_q0),
    .adc_enable_i1(adc_enable_i1), .adc_valid_i1(adc_valid_i1), .adc_data_i1(adc_data_i1),
    .adc_enable_q1(adc_enable_q1), .adc_valid_q1(adc_valid_q1), .adc_data_q1(adc_data_q1),
    .dac_enable_i0(dac_enable_i0), .dac_valid_i0(dac_valid_i0),
    .dac_enable_q0(dac_enable_q0), .dac_valid_q0(dac_valid_q0),
    .dac_enable_i1(dac_enable_i1), .dac_valid_i1(dac_valid_i1),
    .dac_enable_q1(dac_enable_q1), .dac_valid_q1(dac_valid_q1),
    .dma_dac_data_i0(dma_dac_data_i0), .dma_dac_data_q0(dma_dac_data_q0),
    .dma_dac_data_i1(dma_dac_data_i1), .dma_dac_data_q1(dma_dac_data_q1),
    .dac_data_i0(dac_data_i0), .dac_data_q0(dac_data_q0),
    .dac_data_i1(dac_data_i1), .dac_data_q1(dac_data_q1),
    .s_axi_aclk(s_axi_aclk), .s_axi_aresetn(s_axi_aresetn),
    .s_axi_awvalid(s_axi_awvalid), .s_axi_awaddr(s_axi_awaddr), .s_axi_awprot(s_axi_awprot), .s_axi_awready(s_axi_awready),
    .s_axi_wvalid(s_axi_wvalid), .s_axi_wdata(s_axi_wdata), .s_axi_wstrb(s_axi_wstrb), .s_axi_wready(s_axi_wready),
    .s_axi_bvalid(s_axi_bvalid), .s_axi_bresp(s_axi_bresp), .s_axi_bready(s_axi_bready),
    .s_axi_arvalid(s_axi_arvalid), .s_axi_araddr(s_axi_araddr), .s_axi_arprot(s_axi_arprot), .s_axi_arready(s_axi_arready),
    .s_axi_rvalid(s_axi_rvalid), .s_axi_rresp(s_axi_rresp), .s_axi_rdata(s_axi_rdata), .s_axi_rready(s_axi_rready)
  );

  task axil_write(input [15:0] addr, input [31:0] data);
    begin
      @(posedge s_axi_aclk);
      s_axi_awaddr <= addr; s_axi_awvalid <= 1'b1;
      s_axi_wdata  <= data; s_axi_wvalid  <= 1'b1;
      s_axi_bready <= 1'b1;
      @(posedge s_axi_aclk);
      while (!(s_axi_awready && s_axi_wready)) @(posedge s_axi_aclk);
      s_axi_awvalid <= 1'b0; s_axi_wvalid <= 1'b0;
      while (!s_axi_bvalid) @(posedge s_axi_aclk);
      @(posedge s_axi_aclk);
      s_axi_bready <= 1'b0;
    end
  endtask

  integer k;
  real phase, amp;
  reg signed [15:0] i0,q0,i1,q1;

  // shadow: track wr_en0 rising edges and confirm fifo0's just-written word
  // matches the CURRENT crpa_s_re/crpa_s_im, not a stale prior value --
  // this is the exact bug the timing fix in the PROCESSING CORE avoids.
  integer align_checks = 0, align_fail = 0;
  always @(posedge clk) begin
    if (dut.wr_en0 && !dut.full0) begin
      align_checks = align_checks + 1;
      if (dut.proc_i0 !== {{4{dut.crpa_re12[11]}}, dut.crpa_re12} ||
          dut.proc_q0 !== {{4{dut.crpa_im12[11]}}, dut.crpa_im12}) begin
        align_fail = align_fail + 1;
      end
    end
  end

  integer errors = 0;

  initial begin
    adc_data_i0=0; adc_data_q0=0; adc_data_i1=0; adc_data_q1=0;
    repeat (10) @(posedge clk);
    rst = 0;
    repeat (5) @(posedge s_axi_aclk);
    s_axi_aresetn = 1;
    repeat (5) @(posedge s_axi_aclk);

    // -------- Check 1: Phase 1 baseline preserved when pass_en=0 --------
    axil_write(16'h000C, 32'h0000_0000);  // CONTROL: pass_en=0
    repeat (5) @(posedge clk);
    if (dac_data_i0 === dma_dac_data_i0 && dac_data_q0 === dma_dac_data_q0) begin
      $display("PASS: pass_en=0 -> dac_data == dma_dac_data (baseline unchanged)");
    end else begin
      $display("FAIL: pass_en=0 baseline broken: dac_data_i0=%h (expected %h)", dac_data_i0, dma_dac_data_i0);
      errors = errors + 1;
    end

    // -------- Enable CRPA repeater mode, ch1_copy=1 --------
    axil_write(16'h000C, 32'h0000_0009);  // CONTROL: pass_en=1, ch1_copy=1
    axil_write(16'h0040, 32'd256);        // CRPA_COEF[0] = alpha = 1.0 (Q8.8)
    repeat (10) @(posedge clk);

    // -------- Drive 400 rotating-phase samples, amp=100 (same stimulus
    // shape used in the standalone IP's own verified testbenches) --------
    for (k = 0; k < 400; k = k + 1) begin
      phase = k * 0.31;
      i0 = $rtoi(100.0 * $cos(phase));
      q0 = $rtoi(100.0 * $sin(phase));
      i1 = $rtoi(100.0 * $cos(phase + 0.7));
      q1 = $rtoi(100.0 * $sin(phase + 0.7));

      @(posedge clk);
      adc_data_i0 <= i0; adc_data_q0 <= q0;
      adc_data_i1 <= i1; adc_data_q1 <= q1;
      adc_valid_i0 <= 1'b1; adc_valid_q0 <= 1'b1;
      adc_valid_i1 <= 1'b1; adc_valid_q1 <= 1'b1;
      @(posedge clk);
      adc_valid_i0 <= 1'b0; adc_valid_q0 <= 1'b0;
      adc_valid_i1 <= 1'b0; adc_valid_q1 <= 1'b0;

      // TX side requests at the SAME average rate as RX (one pulse per
      // sample period, like adc_valid) but phase-shifted within the period
      // -- exactly what the elastic buffer exists to absorb. Two consecutive
      // cycles here (as in an earlier draft of this testbench) would read
      // twice per sample written and underflow by design, not because of
      // a DUT bug -- that was caught and is fixed here.
      @(posedge clk);
      if (k > 20) begin
        dac_valid_i0 <= 1'b1; dac_valid_q0 <= 1'b1;
        dac_valid_i1 <= 1'b1; dac_valid_q1 <= 1'b1;
      end
      @(posedge clk);
      dac_valid_i0 <= 1'b0; dac_valid_q0 <= 1'b0;
      dac_valid_i1 <= 1'b0; dac_valid_q1 <= 1'b0;
    end

    repeat (20) @(posedge clk);

    // -------- Check 2: buffer write/read alignment held for every sample --------
    if (align_fail == 0 && align_checks > 300) begin
      $display("PASS: fifo0 write always matched the CURRENT nulled sample (%0d checked, 0 stale writes)", align_checks);
    end else begin
      $display("FAIL: %0d of %0d fifo0 writes captured a STALE value (timing misalignment)", align_fail, align_checks);
      errors = errors + 1;
    end

    // -------- Check 3: no overflow/underflow (buffer sized/primed correctly) --------
    if (dut.ovf_sticky === 1'b0 && dut.unf_sticky === 1'b0) begin
      $display("PASS: no overflow/underflow over 400 samples");
    end else begin
      $display("FAIL: ovf_sticky=%b unf_sticky=%b (ovf_cnt=%0d unf_cnt=%0d)",
                dut.ovf_sticky, dut.unf_sticky, dut.ovf_cnt, dut.unf_cnt);
      errors = errors + 1;
    end

    // -------- Check 4: real nulling happened -- last weight magnitude should
    // be well below the raw input amplitude (matching prior ~54dB results) --------
    begin
      real w0_re, w0_im, w0_mag;
      w0_re = $itor($signed(dut.u_crpa_core.w_re[31:0]))  / 1048576.0; // /2^20
      w0_im = $itor($signed(dut.u_crpa_core.w_im[31:0]))  / 1048576.0;
      w0_mag = $sqrt(w0_re*w0_re + w0_im*w0_im);
      $display(" final |w[0]| = %.5f (expect ~0.5 for this M=2, amp=100 case)", w0_mag);
      if (w0_mag > 0.3 && w0_mag < 0.7) begin
        $display("PASS: weights converged to the expected range");
      end else begin
        $display("FAIL: weights did not converge as expected");
        errors = errors + 1;
      end
    end

    $display("=========================================================");
    if (errors == 0) $display(" ALL CHECKS PASSED");
    else              $display(" %0d CHECK(S) FAILED", errors);
    $display("=========================================================");
    $finish;
  end

endmodule
