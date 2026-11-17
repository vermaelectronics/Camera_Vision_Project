// ============================================================================
//  tb_gnss_passthrough_v7.v  --  register map / commit test of gnss_passthrough
//  v7.0 (CRPA build), Icarus Verilog 11+. Run by run_iverilog.sh once per CRPA
//  algorithm, with the same parameters the IP is packaged with:
//    -DALGO=27 -DNT=8 -DREF=4 -DNWTS=16 -DNW_USED=16 -DNWCJ=6
//  Checks: VERSION / AJ_INFO / AJ_DIMS, every weight and image word reaching
//  the core's wts / wcj ports only on AJ_COMMIT (together with cfg, nrm,
//  gain), unused port words held at 0, shadow read-back, a pending set not
//  disturbing the committed one, out-of-range writes ignored, reset values.
//  Prints HDL_TB: PASS.
// ============================================================================
`timescale 1ns/100ps
`ifndef ALGO
`define ALGO 27
`define NT 8
`define REF 4
`define NWTS 16
`define NW_USED 16
`define NWCJ 6
`endif
module tb;
  localparam NWTS = `NWTS, NW_USED = `NW_USED, NWCJ = `NWCJ;
  reg aclk = 0, aresetn = 0, clk = 0, rst = 1;
  always #5 aclk = ~aclk;
  always #2 clk = ~clk;
  reg awvalid = 0, wvalid = 0, bready = 1, arvalid = 0, rready = 1;
  reg [15:0] awaddr = 0, araddr = 0; reg [31:0] wdata = 0;
  wire awready, wready, bvalid, arready, rvalid; wire [1:0] bresp, rresp; wire [31:0] rdata;
  wire [32*NWTS-1:0] wts; wire [32*NWCJ-1:0] wcj;
  wire [31:0] cfg, nrm, gain;
  gnss_passthrough #(.CRPA_ALGO(`ALGO), .CRPA_NT(`NT), .CRPA_REF(`REF),
                     .NWTS(NWTS), .NW_USED(NW_USED), .NWCJ(NWCJ)) dut (
    .clk(clk), .rst(rst),
    .adc_enable_i0(1'b0), .adc_valid_i0(1'b0), .adc_data_i0(16'd0), .adc_enable_q0(1'b0), .adc_valid_q0(1'b0), .adc_data_q0(16'd0),
    .adc_enable_i1(1'b0), .adc_valid_i1(1'b0), .adc_data_i1(16'd0), .adc_enable_q1(1'b0), .adc_valid_q1(1'b0), .adc_data_q1(16'd0),
    .dac_enable_i0(1'b0), .dac_valid_i0(1'b0), .dac_enable_q0(1'b0), .dac_valid_q0(1'b0),
    .dac_enable_i1(1'b0), .dac_valid_i1(1'b0), .dac_enable_q1(1'b0), .dac_valid_q1(1'b0),
    .dma_dac_data_i0(16'd0), .dma_dac_data_q0(16'd0), .dma_dac_data_i1(16'd0), .dma_dac_data_q1(16'd0),
    .dac_data_i0(), .dac_data_q0(), .dac_data_i1(), .dac_data_q1(),
    .m_axis_smi_tdata(), .m_axis_smi_tvalid(), .m_axis_smi_tready(1'b1),
    .s_axis_smi_tdata(32'd0), .s_axis_smi_tvalid(1'b0), .s_axis_smi_tready(), .smi_rst_n(),
    .aj_wts(wts), .aj_wcj(wcj), .aj_cfg(cfg), .aj_nrm(nrm), .aj_gain(gain),
    .s_axi_aclk(aclk), .s_axi_aresetn(aresetn),
    .s_axi_awvalid(awvalid), .s_axi_awaddr(awaddr), .s_axi_awprot(3'd0), .s_axi_awready(awready),
    .s_axi_wvalid(wvalid), .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wready(wready),
    .s_axi_bvalid(bvalid), .s_axi_bresp(bresp), .s_axi_bready(bready),
    .s_axi_arvalid(arvalid), .s_axi_araddr(araddr), .s_axi_arprot(3'd0), .s_axi_arready(arready),
    .s_axi_rvalid(rvalid), .s_axi_rresp(rresp), .s_axi_rdata(rdata), .s_axi_rready(rready));

  integer errors = 0, checks = 0;
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
    checks = checks + 1;
    if (got !== exp) begin errors = errors + 1; $display("FAIL %0s: got %h expected %h", what, got, exp); end
  end endtask
  function [31:0] pat(input integer set, input integer k);
    pat = (set == 1) ? (32'hA5000000 | (k << 8) | k) : (32'h5A000000 | (k << 8) | (k ^ 8'hFF));
  endfunction

  reg [31:0] v; integer j;
  initial begin
    repeat (5) @(posedge aclk); aresetn <= 1; rst <= 0;
    rd(16'h0004, v); chk("VERSION", v, 32'h00070000);
    rd(16'h0184, v); chk("AJ_INFO", v, {8'd`ALGO, 8'd`NT, 8'd`REF, 8'h07});
    rd(16'h019C, v); chk("AJ_DIMS", v, {8'd0, 8'd`NWCJ, 8'd`NW_USED, 8'd`NWTS});
    // reset values
    chk("cfg reset (bypass)", cfg, 32'h1); chk("nrm reset", nrm, 32'h2000); chk("gain reset", gain, 32'h4000);
    for (j = 0; j < NWTS; j = j + 1) chk("wts reset", wts[32*j +: 32], 32'h0);
    // set 1 into the shadows
    for (j = 0; j < NW_USED; j = j + 1) wr(16'h1000 + 4*j, pat(1, j));
    for (j = 0; j < NWCJ; j = j + 1)    wr(16'h1100 + 4*j, pat(1, 64 + j));
    wr(16'h0188, 32'h0000_3200); wr(16'h018C, 32'h0000_1C00); wr(16'h0190, 32'h0000_5A82);
    // nothing reaches the core before the commit
    for (j = 0; j < NW_USED; j = j + 1) chk("wts before commit", wts[32*j +: 32], 32'h0);
    for (j = 0; j < NWCJ; j = j + 1)    chk("wcj before commit", wcj[32*j +: 32], 32'h0);
    chk("cfg before commit", cfg, 32'h1);
    // shadows read back
    for (j = 0; j < NW_USED; j = j + 1) begin rd(16'h1000 + 4*j, v); chk("AJ_W read-back", v, pat(1, j)); end
    for (j = 0; j < NWCJ; j = j + 1)    begin rd(16'h1100 + 4*j, v); chk("AJ_WC read-back", v, pat(1, 64 + j)); end
    wr(16'h0180, 1);
    @(posedge aclk);
    for (j = 0; j < NWTS; j = j + 1)
      chk("wts after commit", wts[32*j +: 32], (j < NW_USED) ? pat(1, j) : 32'h0);
    for (j = 0; j < NWCJ; j = j + 1) chk("wcj after commit", wcj[32*j +: 32], pat(1, 64 + j));
    chk("cfg active", cfg, 32'h3200); chk("nrm active", nrm, 32'h1C00); chk("gain active", gain, 32'h5A82);
    rd(16'h0198, v); chk("AJ_CFG_ACTIVE", v, 32'h3200);
    rd(16'h0180, v); chk("commit count", v, 32'd1);
    // set 2 pending: the committed set must not move
    for (j = 0; j < NW_USED; j = j + 1) wr(16'h1000 + 4*j, pat(2, j));
    for (j = 0; j < NWCJ; j = j + 1)    wr(16'h1100 + 4*j, pat(2, 64 + j));
    wr(16'h0188, 32'h0000_0001);
    for (j = 0; j < NW_USED; j = j + 1) chk("wts while set 2 pending", wts[32*j +: 32], pat(1, j));
    chk("cfg while set 2 pending", cfg, 32'h3200);
    wr(16'h0180, 1); @(posedge aclk);
    for (j = 0; j < NW_USED; j = j + 1) chk("wts after 2nd commit", wts[32*j +: 32], pat(2, j));
    for (j = 0; j < NWCJ; j = j + 1)    chk("wcj after 2nd commit", wcj[32*j +: 32], pat(2, 64 + j));
    chk("cfg after 2nd commit (bypass)", cfg, 32'h1);
    // writes outside the weight / image ranges change nothing
    wr(16'h1000 + 4*NW_USED, 32'hFFFFFFFF); wr(16'h1100 + 4*NWCJ, 32'hFFFFFFFF);
    wr(16'h1200, 32'hFFFFFFFF);
    wr(16'h0180, 1); @(posedge aclk);
    for (j = 0; j < NWTS; j = j + 1)
      chk("wts after out-of-range writes", wts[32*j +: 32], (j < NW_USED) ? pat(2, j) : 32'h0);
    rd(16'h1000 + 4*NW_USED, v); chk("unused AJ_W reads 0", v, (NW_USED < 64) ? 32'h0 : pat(2, 0));
    if (errors == 0) $display("HDL_TB: PASS  crpa%0d  (%0d checks; NT %0d, REF %0d, wts %0d words, %0d used, wcj %0d)",
                              `ALGO, checks, `NT, `REF, NWTS, NW_USED, NWCJ);
    else $display("HDL_TB: FAIL  crpa%0d  (%0d of %0d checks)", `ALGO, errors, checks);
    $finish;
  end
endmodule
