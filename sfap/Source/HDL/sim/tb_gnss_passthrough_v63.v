`timescale 1ns/100ps
module tb;
  reg aclk = 0, aresetn = 0, clk = 0, rst = 1;
  always #5 aclk = ~aclk;
  always #2 clk = ~clk;
  reg awvalid = 0, wvalid = 0, bready = 1, arvalid = 0, rready = 1;
  reg [15:0] awaddr = 0, araddr = 0; reg [31:0] wdata = 0;
  wire awready, wready, bvalid, arready, rvalid; wire [1:0] bresp, rresp; wire [31:0] rdata;
  reg [2:0] wm_a0 = 0, wm_a1 = 0; reg wm_ce0 = 0, wm_ce1 = 0; wire [31:0] wm_q0, wm_q1;
  reg [7:0] wb_a0 = 0; reg wb_ce0 = 0; wire [31:0] wb_q0;
  wire [31:0] cfg, nrm;
  gnss_passthrough dut (
    .clk(clk), .rst(rst),
    .adc_enable_i0(1'b0), .adc_valid_i0(1'b0), .adc_data_i0(16'd0), .adc_enable_q0(1'b0), .adc_valid_q0(1'b0), .adc_data_q0(16'd0),
    .adc_enable_i1(1'b0), .adc_valid_i1(1'b0), .adc_data_i1(16'd0), .adc_enable_q1(1'b0), .adc_valid_q1(1'b0), .adc_data_q1(16'd0),
    .dac_enable_i0(1'b0), .dac_valid_i0(1'b0), .dac_enable_q0(1'b0), .dac_valid_q0(1'b0),
    .dac_enable_i1(1'b0), .dac_valid_i1(1'b0), .dac_enable_q1(1'b0), .dac_valid_q1(1'b0),
    .dma_dac_data_i0(16'd0), .dma_dac_data_q0(16'd0), .dma_dac_data_i1(16'd0), .dma_dac_data_q1(16'd0),
    .dac_data_i0(), .dac_data_q0(), .dac_data_i1(), .dac_data_q1(),
    .m_axis_smi_tdata(), .m_axis_smi_tvalid(), .m_axis_smi_tready(1'b1),
    .s_axis_smi_tdata(32'd0), .s_axis_smi_tvalid(1'b0), .s_axis_smi_tready(), .smi_rst_n(),
    .aj_wb_address0(wb_a0), .aj_wb_ce0(wb_ce0), .aj_wb_q0(wb_q0), .aj_wb_address1(8'd0), .aj_wb_ce1(1'b0), .aj_wb_q1(),
    .aj_wc1_address0(8'd0), .aj_wc1_ce0(1'b0), .aj_wc1_q0(), .aj_wc1_address1(8'd0), .aj_wc1_ce1(1'b0), .aj_wc1_q1(),
    .aj_wc2_address0(8'd0), .aj_wc2_ce0(1'b0), .aj_wc2_q0(), .aj_wc2_address1(8'd0), .aj_wc2_ce1(1'b0), .aj_wc2_q1(),
    .aj_wm_address0(wm_a0), .aj_wm_ce0(wm_ce0), .aj_wm_q0(wm_q0), .aj_wm_address1(wm_a1), .aj_wm_ce1(wm_ce1), .aj_wm_q1(wm_q1),
    .aj_cfg(cfg), .aj_nrm(nrm),
    .s_axi_aclk(aclk), .s_axi_aresetn(aresetn),
    .s_axi_awvalid(awvalid), .s_axi_awaddr(awaddr), .s_axi_awprot(3'd0), .s_axi_awready(awready),
    .s_axi_wvalid(wvalid), .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wready(wready),
    .s_axi_bvalid(bvalid), .s_axi_bresp(bresp), .s_axi_bready(bready),
    .s_axi_arvalid(arvalid), .s_axi_araddr(araddr), .s_axi_arprot(3'd0), .s_axi_arready(arready),
    .s_axi_rvalid(rvalid), .s_axi_rresp(rresp), .s_axi_rdata(rdata), .s_axi_rready(rready));

  integer errors = 0;
  task wr(input [15:0] a, input [31:0] d); begin
    @(posedge aclk); awaddr <= a; wdata <= d; awvalid <= 1; wvalid <= 1;
    @(posedge aclk); while (!awready) @(posedge aclk);
    awvalid <= 0; wvalid <= 0;
    while (!bvalid) @(posedge aclk);
    @(posedge aclk);
  end endtask
  task rd(input [15:0] a, output [31:0] d); begin
    @(posedge aclk); araddr <= a; arvalid <= 1;
    @(posedge aclk); while (!arready) @(posedge aclk);
    arvalid <= 0;
    while (!rvalid) @(posedge aclk);
    d = rdata; @(posedge aclk);
  end endtask
  task chk(input [255:0] what, input [31:0] got, input [31:0] exp); begin
    if (got !== exp) begin errors = errors + 1; $display("FAIL %0s: got %h expected %h", what, got, exp); end
    else $display("ok   %0s = %h", what, got);
  end endtask
  task rdmask(input integer port, input [2:0] a, output [31:0] d); begin
    @(posedge aclk);
    if (port == 0) begin wm_a0 <= a; wm_ce0 <= 1; end else begin wm_a1 <= a; wm_ce1 <= 1; end
    @(posedge aclk); wm_ce0 <= 0; wm_ce1 <= 0;
    #1 d = (port == 0) ? wm_q0 : wm_q1;
  end endtask

  reg [31:0] v; integer j;
  initial begin
    repeat (5) @(posedge aclk); aresetn <= 1; rst <= 0;
    rd(16'h0004, v); chk("VERSION", v, 32'h00060003);
    rd(16'h0184, v); chk("AJ_INFO", v, 32'h08070C05);
    // set 1: mask words = 0xA5000000 + j, weight b_3 = 0x12345678, cfg with blanker thr 700
    for (j = 0; j < 8; j = j + 1) wr(16'h1C00 + 4*j, 32'hA5000000 + j);
    wr(16'h1000 + 4*3, 32'h12345678);
    wr(16'h0188, (32'd700 << 16) | 32'h0000_0200);
    // before commit: the core still reads bank 0 = zeros, cfg = reset bypass
    rdmask(0, 3'd5, v); chk("mask[5] before commit", v, 32'h0);
    chk("cfg before commit", cfg, 32'h1);
    wr(16'h0180, 1);
    for (j = 0; j < 8; j = j + 1) begin rdmask(j & 1, j[2:0], v); chk("mask after commit", v, 32'hA5000000 + j); end
    @(posedge aclk); wb_a0 <= 3; wb_ce0 <= 1; @(posedge aclk); wb_ce0 <= 0; #1 chk("b_3 after commit", wb_q0, 32'h12345678);
    chk("cfg active (thr 700, shift 2)", cfg, (32'd700 << 16) | 32'h200);
    // set 2 written to the spare bank: the running mask must not change until commit
    for (j = 0; j < 8; j = j + 1) wr(16'h1C00 + 4*j, 32'h5A000000 + j);
    rdmask(1, 3'd7, v); chk("mask[7] while set 2 pending", v, 32'hA5000007);
    wr(16'h0180, 1);
    rdmask(1, 3'd7, v); chk("mask[7] after 2nd commit", v, 32'h5A000007);
    // a commit with nothing written keeps the bank
    wr(16'h0180, 1);
    rdmask(0, 3'd0, v); chk("mask[0] after empty commit", v, 32'h5A000000);
    // mask words read back 0 over AXI (write-only)
    rd(16'h1C04, v); chk("AJ_MASK[1] AXI read", v, 32'h0);
    // address 0x1C20 (word 0x708) is outside the mask: must not mark dirty
    wr(16'h1C20, 32'hFFFFFFFF); wr(16'h0180, 1);
    rdmask(0, 3'd0, v); chk("mask[0] after out-of-range write+commit", v, 32'h5A000000);
    if (errors == 0) $display("HDL_TB: PASS"); else $display("HDL_TB: FAIL (%0d)", errors);
    $finish;
  end
endmodule
