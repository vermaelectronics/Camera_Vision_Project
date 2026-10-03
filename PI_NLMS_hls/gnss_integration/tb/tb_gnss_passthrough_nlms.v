`timescale 1ns/100ps
// gnss_passthrough v1.3 with the HLS pi_nlms core selected (CONTROL[4]=1).
//
// Feeds stim.hex (12-bit two-element CW jammer + noise, from gen_vectors) one
// sample every 4 clk (axi_ad9361 2R2T cadence) and checks:
//   1. AXI readback: CORE_VERSION v1.3, STATUS core_sel / cfg_done
//   2. every fifo0 write equals sat12(pi_nlms C model output) -- bit exact
//   3. no fifo overflow/underflow, no sample refused by the HLS core
//   4. jammer suppression on the buffered (pre-DAC) stream
//   5. DAC output carries the buffered result in the 12-bit DAC format
module tb_gnss_passthrough_nlms;

  parameter integer N_SAMPLES = 20480;
  parameter integer MU_SHIFT  = -3;      // must match gen_vectors argument
  parameter real    MIN_SUPP_DB = 25.0;

  reg clk = 0; always #5 clk = ~clk;                       // 100 MHz l_clk
  reg s_axi_aclk = 0; always #7 s_axi_aclk = ~s_axi_aclk;  // async CPU clock
  reg rst = 1, s_axi_aresetn = 0;

  reg adc_enable_i0=1, adc_valid_i0=0; reg [15:0] adc_data_i0=0;
  reg adc_enable_q0=1, adc_valid_q0=0; reg [15:0] adc_data_q0=0;
  reg adc_enable_i1=1, adc_valid_i1=0; reg [15:0] adc_data_i1=0;
  reg adc_enable_q1=1, adc_valid_q1=0; reg [15:0] adc_data_q1=0;
  reg dac_enable_i0=1, dac_valid_i0=0;
  reg dac_enable_q0=1, dac_valid_q0=0;
  reg dac_enable_i1=1, dac_valid_i1=0;
  reg dac_enable_q1=1, dac_valid_q1=0;
  reg [15:0] dma_dac_data_i0=16'h1234, dma_dac_data_q0=16'h5678;
  reg [15:0] dma_dac_data_i1=16'h0,    dma_dac_data_q1=16'h0;
  wire [15:0] dac_data_i0, dac_data_q0, dac_data_i1, dac_data_q1;

  reg s_axi_awvalid=0; reg [15:0] s_axi_awaddr=0; reg [2:0] s_axi_awprot=0;
  wire s_axi_awready;
  reg s_axi_wvalid=0; reg [31:0] s_axi_wdata=0; reg [3:0] s_axi_wstrb=4'hF;
  wire s_axi_wready;
  wire s_axi_bvalid; wire [1:0] s_axi_bresp; reg s_axi_bready=0;
  reg s_axi_arvalid=0; reg [15:0] s_axi_araddr=0; reg [2:0] s_axi_arprot=0;
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

  task axil_read(input [15:0] addr, output [31:0] data);
    begin
      @(posedge s_axi_aclk);
      s_axi_araddr <= addr; s_axi_arvalid <= 1'b1; s_axi_rready <= 1'b1;
      @(posedge s_axi_aclk);
      while (!s_axi_arready) @(posedge s_axi_aclk);
      s_axi_arvalid <= 1'b0;
      while (!s_axi_rvalid) @(posedge s_axi_aclk);
      data = s_axi_rdata;
      @(posedge s_axi_aclk);
      s_axi_rready <= 1'b0;
    end
  endtask

  function signed [11:0] sat12(input signed [15:0] v);
    sat12 = (v > 16'sd2047) ? 12'sd2047 : (v < -16'sd2048) ? -12'sd2048 : v[11:0];
  endfunction

  reg [63:0] stim [0:N_SAMPLES-1];
  reg [31:0] expv [0:N_SAMPLES-1];

  // ---- monitor: compare every fifo0 write against the C model ------------
  integer wr_idx = 0, mismatches = 0;
  real    p_first = 0.0, p_last = 0.0;
  always @(posedge clk) begin
    if (dut.core_sel && dut.wr_en0 && !dut.full0) begin
      if (wr_idx < N_SAMPLES) begin
        if ($signed(dut.proc_i0[11:0]) !== sat12(expv[wr_idx][15:0]) ||
            $signed(dut.proc_q0[11:0]) !== sat12(expv[wr_idx][31:16])) begin
          if (mismatches < 5)
            $display("MISMATCH @%0d: rtl=(%0d,%0d) model=(%0d,%0d)", wr_idx,
                     $signed(dut.proc_i0[11:0]), $signed(dut.proc_q0[11:0]),
                     sat12(expv[wr_idx][15:0]), sat12(expv[wr_idx][31:16]));
          mismatches = mismatches + 1;
        end
        // block 1 (after the first weight update window) vs the last block
        if (wr_idx >= 1024 && wr_idx < 2048)
          p_first = p_first + $itor($signed(dut.proc_i0[11:0]))**2 + $itor($signed(dut.proc_q0[11:0]))**2;
        if (wr_idx >= N_SAMPLES - 1024)
          p_last  = p_last  + $itor($signed(dut.proc_i0[11:0]))**2 + $itor($signed(dut.proc_q0[11:0]))**2;
      end
      wr_idx = wr_idx + 1;
    end
  end

  // ---- monitor: DAC output must be the buffered word in DAC format -------
  integer dac_checks = 0, dac_fail = 0;
  always @(posedge clk) begin
    if (dut.core_sel && dut.pass_en && dut.rd_en0 && !dut.empty0) begin
      @(negedge clk);
      dac_checks = dac_checks + 1;
      if (dac_data_i0 !== {dut.rd_data0[11:0], 4'b0} ||
          dac_data_q0 !== {dut.rd_data0[27:16], 4'b0} ||
          dac_data_i1 !== dac_data_i0 || dac_data_q1 !== dac_data_q0)
        dac_fail = dac_fail + 1;
    end
  end

  integer k, errors = 0;
  reg [31:0] rd;
  real supp_db;

  initial begin
    $readmemh("stim.hex", stim);
    $readmemh("exp.hex",  expv);

    repeat (10) @(posedge clk);
    rst = 0;
    repeat (5) @(posedge s_axi_aclk);
    s_axi_aresetn = 1;
    repeat (5) @(posedge s_axi_aclk);

    axil_read(16'h0004, rd);
    if (rd === 32'h00010003) $display("PASS: CORE_VERSION = %h", rd);
    else begin $display("FAIL: CORE_VERSION = %h", rd); errors = errors + 1; end

    axil_read(16'h0044, rd);
    if (rd[15:0] === 16'hFFFD) $display("PASS: CRPA_COEF[1] (mu_shift_ctrl) default = -3");
    else begin $display("FAIL: CRPA_COEF[1] default = %h", rd); errors = errors + 1; end

    // mu_shift_ctrl, then pass_en | ch1_copy | core_sel(NLMS)
    axil_write(16'h0044, MU_SHIFT);
    axil_write(16'h000C, 32'h0000_0019);

    // wait for the HLS core's mu register to be programmed (STATUS[12])
    rd = 0;
    while (!rd[12]) axil_read(16'h0010, rd);
    if (rd[10] && rd[16]) $display("PASS: STATUS core_sel=1, cfg_done=1, pass_en=1 (status=%h)", rd);
    else begin $display("FAIL: STATUS = %h", rd); errors = errors + 1; end
    if (dut.u_nlms_core.mu_shift_ctrl !== MU_SHIFT[15:0]) begin
      $display("FAIL: HLS core mu_shift_ctrl = %0d", $signed(dut.u_nlms_core.mu_shift_ctrl));
      errors = errors + 1;
    end

    for (k = 0; k < N_SAMPLES; k = k + 1) begin
      @(posedge clk);
      {adc_data_q1, adc_data_i1, adc_data_q0, adc_data_i0} <= stim[k];
      adc_valid_i0 <= 1'b1; adc_valid_q0 <= 1'b1;
      adc_valid_i1 <= 1'b1; adc_valid_q1 <= 1'b1;
      @(posedge clk);
      adc_valid_i0 <= 1'b0; adc_valid_q0 <= 1'b0;
      adc_valid_i1 <= 1'b0; adc_valid_q1 <= 1'b0;
      @(posedge clk);
      if (k > 20) begin
        dac_valid_i0 <= 1'b1; dac_valid_q0 <= 1'b1;
        dac_valid_i1 <= 1'b1; dac_valid_q1 <= 1'b1;
      end
      @(posedge clk);
      dac_valid_i0 <= 1'b0; dac_valid_q0 <= 1'b0;
      dac_valid_i1 <= 1'b0; dac_valid_q1 <= 1'b0;
    end
    repeat (40) @(posedge clk);

    if (wr_idx == N_SAMPLES && mismatches == 0)
      $display("PASS: %0d buffered samples bit-exact vs pi_nlms C model", wr_idx);
    else begin
      $display("FAIL: %0d writes (expected %0d), %0d mismatches", wr_idx, N_SAMPLES, mismatches);
      errors = errors + 1;
    end

    if (!dut.ovf_sticky && !dut.unf_sticky && !dut.nlms_drop_sticky)
      $display("PASS: no overflow / underflow / refused samples");
    else begin
      $display("FAIL: ovf=%b unf=%b nlms_drop=%b", dut.ovf_sticky, dut.unf_sticky, dut.nlms_drop_sticky);
      errors = errors + 1;
    end

    supp_db = 10.0 * $log10(p_first / p_last);
    if (supp_db >= MIN_SUPP_DB) $display("PASS: jammer suppression %.1f dB (block 1 -> last block)", supp_db);
    else begin $display("FAIL: jammer suppression only %.1f dB", supp_db); errors = errors + 1; end

    if (dac_checks > N_SAMPLES - 100 && dac_fail == 0)
      $display("PASS: DAC output matched buffered result for %0d samples", dac_checks);
    else begin
      $display("FAIL: DAC check %0d failures of %0d", dac_fail, dac_checks);
      errors = errors + 1;
    end

    $display("=========================================================");
    if (errors == 0) $display(" ALL CHECKS PASSED");
    else             $display(" %0d CHECK(S) FAILED", errors);
    $display("=========================================================");
    $finish;
  end

endmodule
