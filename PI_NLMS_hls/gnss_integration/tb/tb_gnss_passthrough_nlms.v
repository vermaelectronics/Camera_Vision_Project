`timescale 1ns/100ps
// gnss_passthrough v1.3: bypass (core_sel=0) and PI-NLMS (core_sel=2).
//
// Samples arrive every SAMPLE_CLKS clocks (2 = axi_ad9361 2R2T LVDS cadence:
// adc_valid is high every second l_clk) with dac_valid at the same rate.
//
//   1. identity: VERSION, CRPA_COEF defaults
//   2. bypass  : every fifo write is the raw RX sample of its own channel and
//                the DAC carries it left-aligned -- v1.1 behaviour unchanged
//   3. PI-NLMS : STATUS core_sel/cfg_done; every fifo0 write equals
//                sat12(pi_nlms C model output) bit-exactly (stim.hex/exp.hex
//                from gen_vectors); no overflow/underflow/refused samples;
//                jammer suppression; DAC carries the buffered result
module tb_gnss_passthrough_nlms;

  parameter integer N_SAMPLES   = 20480;
  parameter integer MU_SHIFT    = -3;     // must match gen_vectors argument
  parameter integer SAMPLE_CLKS = 2;
  parameter integer N_BYPASS    = 400;
  parameter real    MIN_SUPP_DB = 25.0;

  reg clk = 0; always #4 clk = ~clk;                       // 125 MHz l_clk (8 ns)
  reg s_axi_aclk = 0; always #5 s_axi_aclk = ~s_axi_aclk;  // 100 MHz, async
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

  // One sample period of SAMPLE_CLKS clocks: adc_valid on the first clock,
  // dac_valid on the second, idle after that.
  task drive_sample(input [63:0] smp, input dac_req);
    integer c;
    begin
      @(posedge clk);
      {adc_data_q1, adc_data_i1, adc_data_q0, adc_data_i0} <= smp;
      {adc_valid_i0, adc_valid_q0, adc_valid_i1, adc_valid_q1} <= 4'hF;
      {dac_valid_i0, dac_valid_q0, dac_valid_i1, dac_valid_q1} <= 4'h0;
      @(posedge clk);
      {adc_valid_i0, adc_valid_q0, adc_valid_i1, adc_valid_q1} <= 4'h0;
      {dac_valid_i0, dac_valid_q0, dac_valid_i1, dac_valid_q1} <= {4{dac_req}};
      for (c = 2; c < SAMPLE_CLKS; c = c + 1) begin
        @(posedge clk);
        {dac_valid_i0, dac_valid_q0, dac_valid_i1, dac_valid_q1} <= 4'h0;
      end
    end
  endtask

  function signed [11:0] sat12(input signed [15:0] v);
    sat12 = (v > 16'sd2047) ? 12'sd2047 : (v < -16'sd2048) ? -12'sd2048 : v[11:0];
  endfunction

  reg [63:0] stim [0:N_SAMPLES-1];
  reg [31:0] expv [0:N_SAMPLES-1];

  // ---- bypass monitor: fifo writes must be the raw sample per channel -------
  integer byp_wr0 = 0, byp_wr1 = 0, byp_fail = 0;
  always @(posedge clk) begin
    if (dut.pass_en && dut.core_sel == 2'd0) begin
      if (dut.wr_en0) begin
        byp_wr0 = byp_wr0 + 1;
        if ({dut.proc_q0, dut.proc_i0} !== {adc_data_q0, adc_data_i0} || !adc_valid_i0) byp_fail = byp_fail + 1;
      end
      if (dut.wr_en1) begin
        byp_wr1 = byp_wr1 + 1;
        if ({dut.proc_q1, dut.proc_i1} !== {adc_data_q1, adc_data_i1} || !adc_valid_i1) byp_fail = byp_fail + 1;
      end
    end
  end

  // ---- NLMS monitor: compare every fifo0 write against the C model ----------
  integer wr_idx = 0, mismatches = 0;
  real    p_first = 0.0, p_last = 0.0;
  always @(posedge clk) begin
    if (dut.use_nlms && dut.wr_en0 && !dut.full0) begin
      if (wr_idx < N_SAMPLES) begin
        if ($signed(dut.proc_i0[11:0]) !== sat12(expv[wr_idx][15:0]) ||
            $signed(dut.proc_q0[11:0]) !== sat12(expv[wr_idx][31:16]) ||
            {dut.proc_q1, dut.proc_i1} !== {dut.proc_q0, dut.proc_i0}) begin
          if (mismatches < 5)
            $display("MISMATCH @%0d: rtl=(%0d,%0d) model=(%0d,%0d)", wr_idx,
                     $signed(dut.proc_i0[11:0]), $signed(dut.proc_q0[11:0]),
                     sat12(expv[wr_idx][15:0]), sat12(expv[wr_idx][31:16]));
          mismatches = mismatches + 1;
        end
        if (wr_idx >= 1024 && wr_idx < 2048)
          p_first = p_first + $itor($signed(dut.proc_i0[11:0]))**2 + $itor($signed(dut.proc_q0[11:0]))**2;
        if (wr_idx >= N_SAMPLES - 1024)
          p_last  = p_last  + $itor($signed(dut.proc_i0[11:0]))**2 + $itor($signed(dut.proc_q0[11:0]))**2;
      end
      wr_idx = wr_idx + 1;
    end
  end

  // ---- DAC monitor: output = buffered word in the left-aligned DAC format ---
  integer dac_checks = 0, dac_fail = 0;
  always @(negedge clk) begin
    if (dut.pass_en && !dut.mute && dut.ch1_copy && dut.rd_en0 === 1'b0 && dut.primed0) begin
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

    // ---------------- 1. identity / defaults ----------------
    axil_read(16'h0004, rd);
    if (rd === 32'h00010003) $display("PASS: VERSION = %h", rd);
    else begin $display("FAIL: VERSION = %h", rd); errors = errors + 1; end
    axil_read(16'h0040, rd);
    if (rd === 32'h00000100) $display("PASS: CRPA_COEF[0] (PI alpha) default = 1.0");
    else begin $display("FAIL: CRPA_COEF[0] = %h", rd); errors = errors + 1; end
    axil_read(16'h0044, rd);
    if (rd === 32'hFFFFFFFD) $display("PASS: CRPA_COEF[1] (NLMS mu_shift_ctrl) default = -3");
    else begin $display("FAIL: CRPA_COEF[1] = %h", rd); errors = errors + 1; end

    // ---------------- 2. bypass: v1.1 behaviour ----------------
    axil_write(16'h000C, 32'h0000_0001);           // pass_en only, core_sel = 0
    repeat (10) @(posedge clk);
    for (k = 0; k < N_BYPASS; k = k + 1)
      drive_sample(stim[k], k > 20);
    @(posedge clk) {dac_valid_i0, dac_valid_q0, dac_valid_i1, dac_valid_q1} <= 4'h0;
    repeat (20) @(posedge clk);
    if (byp_fail == 0 && byp_wr0 == N_BYPASS && byp_wr1 == N_BYPASS)
      $display("PASS: bypass wrote %0d raw samples per channel unchanged (v1.1 identity)", byp_wr0);
    else begin
      $display("FAIL: bypass writes ch0=%0d ch1=%0d, %0d wrong", byp_wr0, byp_wr1, byp_fail);
      errors = errors + 1;
    end
    if (dac_data_i0 === {dut.rd_data0[11:0], 4'b0} && dac_data_i1 === {dut.rd_data1[11:0], 4'b0})
      $display("PASS: bypass DAC outputs are the per-channel buffered samples, left-aligned");
    else begin $display("FAIL: bypass DAC output"); errors = errors + 1; end

    // ---------------- 3. PI-NLMS ----------------
    axil_write(16'h000C, 32'h0000_0000);           // stop, flush
    repeat (10) @(posedge clk);
    axil_write(16'h0044, MU_SHIFT);
    axil_write(16'h000C, 32'h0000_0029);           // pass_en | ch1_copy | core_sel=2
    rd = 0;
    while (!rd[13]) axil_read(16'h0010, rd);       // wait for nlms_cfg_done
    if (rd[11:10] == 2'd2 && rd[16] && !rd[12])
      $display("PASS: STATUS core_sel=2, cfg_done=1, pass_en=1 (status=%h)", rd);
    else begin $display("FAIL: STATUS = %h", rd); errors = errors + 1; end
    if (dut.u_nlms_core.mu_shift_ctrl !== MU_SHIFT[15:0]) begin
      $display("FAIL: HLS core mu_shift_ctrl = %0d", $signed(dut.u_nlms_core.mu_shift_ctrl));
      errors = errors + 1;
    end

    // The buffer still holds the bypass phase's samples (pass_en=0 does not
    // flush it, as in v1.1), so TX keeps requesting from the first sample --
    // on the board dac_valid never stops.
    for (k = 0; k < N_SAMPLES; k = k + 1)
      drive_sample(stim[k], 1'b1);
    @(posedge clk) {dac_valid_i0, dac_valid_q0, dac_valid_i1, dac_valid_q1} <= 4'h0;
    repeat (60) @(posedge clk);

    if (wr_idx == N_SAMPLES && mismatches == 0)
      $display("PASS: %0d buffered samples bit-exact vs pi_nlms C model", wr_idx);
    else begin
      $display("FAIL: %0d writes (expected %0d), %0d mismatches", wr_idx, N_SAMPLES, mismatches);
      errors = errors + 1;
    end

    axil_read(16'h0010, rd);   // STATUS through the CDC path
    if (!dut.ovf_sticky && !dut.unf_sticky && !dut.nlms_drop_sticky && !rd[12])
      $display("PASS: no overflow / underflow / refused samples at %0d clocks per sample", SAMPLE_CLKS);
    else begin
      $display("FAIL: ovf=%b unf=%b nlms_drop=%b", dut.ovf_sticky, dut.unf_sticky, dut.nlms_drop_sticky);
      errors = errors + 1;
    end

    supp_db = 10.0 * $log10(p_first / p_last);
    if (supp_db >= MIN_SUPP_DB) $display("PASS: jammer suppression %.1f dB (block 1 -> last block)", supp_db);
    else begin $display("FAIL: jammer suppression only %.1f dB", supp_db); errors = errors + 1; end

    if (dac_checks > N_SAMPLES && dac_fail == 0)
      $display("PASS: DAC output matched the buffered result on %0d clocks", dac_checks);
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
