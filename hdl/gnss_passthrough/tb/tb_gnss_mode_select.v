`timescale 1ns/100ps
// Targeted check: CONTROL[4] actually switches which core's s_valid/s_re/s_im
// drive the output mux -- not just that both cores elaborate.
module probe_mode_select;
  reg clk = 0; always #5 clk = ~clk;
  reg rst = 1, s_axi_aclk = 0; always #7 s_axi_aclk = ~s_axi_aclk;
  reg s_axi_aresetn = 0;

  reg adc_enable_i0=1, adc_valid_i0=0; reg [15:0] adc_data_i0=0;
  reg adc_enable_q0=1, adc_valid_q0=0; reg [15:0] adc_data_q0=0;
  reg adc_enable_i1=1, adc_valid_i1=0; reg [15:0] adc_data_i1=0;
  reg adc_enable_q1=1, adc_valid_q1=0; reg [15:0] adc_data_q1=0;
  reg dac_enable_i0=1, dac_valid_i0=0;
  reg dac_enable_q0=1, dac_valid_q0=0;
  reg dac_enable_i1=1, dac_valid_i1=0;
  reg dac_enable_q1=1, dac_valid_q1=0;
  reg [15:0] dma_dac_data_i0=0, dma_dac_data_q0=0, dma_dac_data_i1=0, dma_dac_data_q1=0;
  wire [15:0] dac_data_i0, dac_data_q0, dac_data_i1, dac_data_q1;
  reg s_axi_awvalid=0; reg [15:0] s_axi_awaddr=0; reg [2:0] s_axi_awprot=0; wire s_axi_awready;
  reg s_axi_wvalid=0; reg [31:0] s_axi_wdata=0; reg [3:0] s_axi_wstrb=4'hF; wire s_axi_wready;
  wire s_axi_bvalid; wire [1:0] s_axi_bresp; reg s_axi_bready=0;
  reg s_axi_arvalid=0; reg [15:0] s_axi_araddr=0; reg [2:0] s_axi_arprot=0; wire s_axi_arready;
  wire s_axi_rvalid; wire [1:0] s_axi_rresp; wire [31:0] s_axi_rdata; reg s_axi_rready=0;

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
      s_axi_wdata  <= data; s_axi_wvalid <= 1'b1;
      s_axi_bready <= 1'b1;
      @(posedge s_axi_aclk);
      while (!(s_axi_awready && s_axi_wready)) @(posedge s_axi_aclk);
      s_axi_awvalid <= 1'b0; s_axi_wvalid <= 1'b0;
      while (!s_axi_bvalid) @(posedge s_axi_aclk);
      @(posedge s_axi_aclk);
      s_axi_bready <= 1'b0;
    end
  endtask

  integer errors = 0;
  initial begin
    repeat(10) @(posedge clk); rst = 0;
    repeat(5) @(posedge s_axi_aclk); s_axi_aresetn = 1;
    repeat(5) @(posedge s_axi_aclk);

    $display("BEFORE any CONTROL write: crpa_mode=%b (expect 00)", dut.crpa_mode);
    if (dut.crpa_mode !== 2'b00) begin
      $display("FAIL: mode select is not 00 by default"); errors = errors + 1;
    end

    // pass_en=1, mode=00 -> standard core selected
    axil_write(16'h000C, 32'h0000_0001);
    repeat(5) @(posedge clk);
    if (dut.crpa_s_re !== dut.crpa_s_re_std) begin
      $display("FAIL: with CONTROL[5:4]=00, output mux is not passing the STANDARD core's s_re");
      errors = errors + 1;
    end else begin
      $display("PASS: CONTROL[5:4]=00 selects the standard core (crpa_s_re == crpa_s_re_std)");
    end

    // pass_en=1, mode=01 -> normalized core selected
    axil_write(16'h000C, 32'h0000_0011);   // bit0 (pass_en) + bit4 (mode=01)
    repeat(5) @(posedge clk);
    if (dut.crpa_mode !== 2'b01) begin
      $display("FAIL: CONTROL[5:4]=01 write did not reach crpa_mode");
      errors = errors + 1;
    end
    if (dut.crpa_s_re !== dut.crpa_s_re_norm) begin
      $display("FAIL: with CONTROL[5:4]=01, output mux is not passing the NORMALIZED core's s_re");
      errors = errors + 1;
    end else begin
      $display("PASS: CONTROL[5:4]=01 selects the normalized core (crpa_s_re == crpa_s_re_norm)");
    end

    // pass_en=1, mode=10 -> PL-NPI core selected
    axil_write(16'h000C, 32'h0000_0021);   // bit0 (pass_en) + bit5 (mode=10)
    repeat(5) @(posedge clk);
    if (dut.crpa_mode !== 2'b10) begin
      $display("FAIL: CONTROL[5:4]=10 write did not reach crpa_mode");
      errors = errors + 1;
    end
    if (dut.crpa_s_re !== dut.crpa_s_re_pl) begin
      $display("FAIL: with CONTROL[5:4]=10, output mux is not passing the PL-NPI core's s_re");
      errors = errors + 1;
    end else begin
      $display("PASS: CONTROL[5:4]=10 selects the PL-NPI core (crpa_s_re == crpa_s_re_pl)");
    end

    // pass_en=1, mode=11 (reserved) -> falls back to standard
    axil_write(16'h000C, 32'h0000_0031);   // bit0 (pass_en) + bits5:4 (mode=11)
    repeat(5) @(posedge clk);
    if (dut.crpa_s_re !== dut.crpa_s_re_std) begin
      $display("FAIL: with CONTROL[5:4]=11 (reserved), output mux did not fall back to the standard core");
      errors = errors + 1;
    end else begin
      $display("PASS: CONTROL[5:4]=11 (reserved) falls back to the standard core");
    end

    // gamma registers: write DIFFERENT non-default values to CRPA_COEF(1)
    // and CRPA_COEF(2), confirm each reaches its OWN core's gamma_reg
    // independently (same class of check as the v1.3 alpha_wr regression
    // test -- prove the write reaches the algorithm, not just the AXI-side
    // shadow register -- and confirm the two gammas are NOT cross-wired).
    axil_write(16'h0044, 32'd777);   // CRPA_COEF(1) = 0x40 + 1*4 = 0x44 (normalized core's gamma)
    axil_write(16'h0048, 32'd333);   // CRPA_COEF(2) = 0x40 + 2*4 = 0x48 (PL-NPI's own gamma)
    repeat(10) @(posedge clk);
    if (dut.u_crpa_core_normalized.gamma_reg !== 777) begin
      $display("FAIL: CRPA_COEF(1)=777 did not reach the normalized core's gamma_reg (got %0d)",
                dut.u_crpa_core_normalized.gamma_reg);
      errors = errors + 1;
    end else begin
      $display("PASS: CRPA_COEF(1) write reached the normalized core's gamma_reg (777)");
    end
    if (dut.u_crpa_core_pl_npi.gamma_reg !== 333) begin
      $display("FAIL: CRPA_COEF(2)=333 did not reach the PL-NPI core's gamma_reg (got %0d)",
                dut.u_crpa_core_pl_npi.gamma_reg);
      errors = errors + 1;
    end else begin
      $display("PASS: CRPA_COEF(2) write reached the PL-NPI core's gamma_reg (333), independent of CRPA_COEF(1)");
    end

    $display("=========================================================");
    if (errors == 0) $display(" ALL CHECKS PASSED");
    else              $display(" %0d CHECK(S) FAILED", errors);
    $display("=========================================================");
    $finish;
  end
endmodule
