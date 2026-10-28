`timescale 1ns/100ps
module tb;
  reg clk = 0, aclk = 0;
  always #2.5 clk = ~clk;        // l_clk 200 MHz (50 MS/s, 2R2T)
  always #5   aclk = ~aclk;      // sys_cpu_clk 100 MHz
  reg rst = 1, aresetn = 0;

  // ADC side: valid every 4 l_clk
  reg [1:0] ph = 0;
  reg adc_valid = 0;
  reg [11:0] cnt = 0;
  always @(posedge clk) begin
    ph <= ph + 1;
    adc_valid <= (ph == 0);
    if (ph == 0) cnt <= cnt + 1;
  end
  wire [15:0] i0 = {{4{cnt[11]}}, cnt};
  wire [15:0] q0 = 16'd5;
  wire [15:0] i1 = 16'd7, q1 = 16'd9;
  // DAC side: request every 4 l_clk, offset phase
  wire dac_valid = (ph == 2);

  wire [63:0] m_tdata; wire m_tvalid; reg m_tready = 1;
  wire [31:0] s_tdata; wire s_tvalid; wire s_tready;
  wire npi_rst_n; wire [31:0] npi_gamma, npi_cfg, npi_nrm, npi_thr, npi_hold; wire npi_adapt_en; wire [191:0] npi_ncoef; wire [95:0] npi_eqcoef;
  reg [31:0] npi_st = 32'h00070006; reg [191:0] npi_wout = {32'h66666666,32'h55555555,32'h44444444,32'h33333333,32'h22222222,32'h11111111}; reg [31:0] npi_pow = 32'h00001234;
  wire [15:0] di0, dq0, di1, dq1;

  // AXI-Lite master
  reg awvalid = 0, wvalid = 0, bready = 1, arvalid = 0, rready = 1;
  reg [15:0] awaddr = 0, araddr = 0; reg [31:0] wdata = 0;
  wire awready, wready, bvalid, arready, rvalid; wire [1:0] bresp, rresp; wire [31:0] rdata;

  gnss_passthrough dut(
    .clk(clk), .rst(rst),
    .adc_enable_i0(1'b1), .adc_valid_i0(adc_valid), .adc_data_i0(i0),
    .adc_enable_q0(1'b1), .adc_valid_q0(adc_valid), .adc_data_q0(q0),
    .adc_enable_i1(1'b1), .adc_valid_i1(adc_valid), .adc_data_i1(i1),
    .adc_enable_q1(1'b1), .adc_valid_q1(adc_valid), .adc_data_q1(q1),
    .dac_enable_i0(1'b1), .dac_valid_i0(dac_valid), .dac_enable_q0(1'b1), .dac_valid_q0(dac_valid),
    .dac_enable_i1(1'b1), .dac_valid_i1(dac_valid), .dac_enable_q1(1'b1), .dac_valid_q1(dac_valid),
    .dma_dac_data_i0(16'd0), .dma_dac_data_q0(16'd0), .dma_dac_data_i1(16'd0), .dma_dac_data_q1(16'd0),
    .dac_data_i0(di0), .dac_data_q0(dq0), .dac_data_i1(di1), .dac_data_q1(dq1),
    .m_axis_npi_tdata(m_tdata), .m_axis_npi_tvalid(m_tvalid), .m_axis_npi_tready(m_tready),
    .s_axis_npi_tdata(s_tdata), .s_axis_npi_tvalid(s_tvalid), .s_axis_npi_tready(s_tready),
    .npi_gamma(npi_gamma), .npi_adapt_en(npi_adapt_en), .npi_rst_n(npi_rst_n), .npi_cfg(npi_cfg), .npi_nrm(npi_nrm),
    .npi_det_thr(npi_thr), .npi_det_hold(npi_hold), .npi_ncoef(npi_ncoef), .npi_eqcoef(npi_eqcoef), .npi_st(npi_st), .npi_wout(npi_wout), .npi_det_pow(npi_pow),
    .s_axi_aclk(aclk), .s_axi_aresetn(aresetn),
    .s_axi_awvalid(awvalid), .s_axi_awaddr(awaddr), .s_axi_awprot(3'd0), .s_axi_awready(awready),
    .s_axi_wvalid(wvalid), .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wready(wready),
    .s_axi_bvalid(bvalid), .s_axi_bresp(bresp), .s_axi_bready(bready),
    .s_axi_arvalid(arvalid), .s_axi_araddr(araddr), .s_axi_arprot(3'd0), .s_axi_arready(arready),
    .s_axi_rvalid(rvalid), .s_axi_rresp(rresp), .s_axi_rdata(rdata), .s_axi_rready(rready));

  // model of FIFO -> core -> FIFO: identity on RX1 with a 70-l_clk latency
  // (two async FIFOs plus the filter pipeline), one output per input
  reg [31:0] dl_data [0:127]; reg dl_v [0:127];
  integer k, m;
  initial for (k = 0; k < 128; k = k + 1) begin dl_v[k] = 0; dl_data[k] = 0; end
  always @(posedge clk) begin
    for (m = 127; m > 0; m = m - 1) begin dl_v[m] <= dl_v[m-1]; dl_data[m] <= dl_data[m-1]; end
    dl_v[0] <= m_tvalid & m_tready & npi_rst_n;
    dl_data[0] <= m_tdata[31:0];
  end
  assign s_tvalid = dl_v[70];
  assign s_tdata = dl_data[70];

  task axi_write(input [15:0] a, input [31:0] d);
    begin
      @(posedge aclk); awaddr <= a; wdata <= d; awvalid <= 1; wvalid <= 1;
      @(posedge aclk); while (!awready) @(posedge aclk);
      awvalid <= 0; wvalid <= 0;
      @(posedge aclk); while (!bvalid) @(posedge aclk);
      @(posedge aclk);
    end
  endtask
  task axi_read(input [15:0] a, output [31:0] d);
    begin
      @(posedge aclk); araddr <= a; arvalid <= 1;
      @(posedge aclk); while (!arready) @(posedge aclk);
      arvalid <= 0;
      while (!rvalid) @(posedge aclk);
      d = rdata;
      @(posedge aclk);
    end
  endtask

  integer errors = 0, unf_base;
  reg [31:0] r;
  // DAC output continuity check: i0 output is RX1 i (cnt) left-aligned; it
  // must increase by exactly 1 per DAC request once running (no repeats/skips)
  reg [11:0] last = 0; reg have = 0; integer jumps = 0; reg checking = 0;
  always @(posedge clk) if (dac_valid && checking) begin
    // value visible after the read register updates: sample on the next request
    if (have && (di0[15:4] != last + 1'b1)) jumps = jumps + 1;
    last <= di0[15:4]; have <= 1;
  end

  initial begin
    #100 rst = 0; aresetn = 1;
    #100;
    axi_read(16'h0004, r); if (r !== 32'h00020005) begin $display("VERSION %h", r); errors = errors + 1; end
    axi_read(16'h0184, r); if (r !== 32'h03020803) begin $display("NPI_INFO %h", r); errors = errors + 1; end
    if (npi_cfg !== 32'hD || npi_nrm !== 32'h2000 || npi_thr !== 32'hFFFFFFFF || npi_hold !== 32'd0) begin $display("reset cfg %h nrm %h thr %h hold %h", npi_cfg, npi_nrm, npi_thr, npi_hold); errors = errors + 1; end
    if (npi_gamma !== 32'd1 || npi_adapt_en !== 1'b1) begin $display("reset gamma %h adapt %b", npi_gamma, npi_adapt_en); errors = errors + 1; end
    axi_write(16'h0048, 32'd123456789);
    if (npi_gamma !== 32'd123456789) begin $display("gamma %h", npi_gamma); errors = errors + 1; end
    axi_write(16'h000C, 32'h40);
    if (npi_adapt_en !== 1'b0) begin $display("freeze did not clear adapt_en"); errors = errors + 1; end
    axi_write(16'h000C, 32'h0);
    axi_write(16'h0188, 32'h00000204);
    axi_write(16'h018C, 32'h00007FFF);
    axi_write(16'h0190, 32'd6400);
    axi_write(16'h0194, 32'd30720000);
    if (npi_cfg !== 32'hD || npi_thr !== 32'hFFFFFFFF) begin $display("changed before commit"); errors = errors + 1; end
    axi_read(16'h0188, r); if (r !== 32'h204) begin $display("CFG shadow %h", r); errors = errors + 1; end
    axi_read(16'h0198, r); if (r !== 32'hD) begin $display("CFG_ACTIVE before commit %h", r); errors = errors + 1; end
    if (npi_ncoef !== 192'd0) begin $display("ncoef reset %h", npi_ncoef); errors = errors + 1; end
    if (npi_eqcoef !== 96'd0) begin $display("eqcoef reset %h", npi_eqcoef); errors = errors + 1; end
    axi_write(16'h01D8, 32'h1111_0A0A); axi_write(16'h01DC, 32'h2222_0B0B); axi_write(16'h01E0, 32'h3333_0C0C);
    if (npi_eqcoef !== 96'd0) begin $display("eqcoef changed before commit"); errors = errors + 1; end
    axi_read(16'h01DC, r); if (r !== 32'h2222_0B0B) begin $display("EQCOEF1 shadow %h", r); errors = errors + 1; end
    axi_write(16'h01C0, 32'hA0A0_0101); axi_write(16'h01C4, 32'hB1B1_0202); axi_write(16'h01C8, 32'hC2C2_0303);
    axi_write(16'h01CC, 32'hD3D3_0404); axi_write(16'h01D0, 32'hE4E4_0505); axi_write(16'h01D4, 32'hF5F5_0606);
    if (npi_ncoef !== 192'd0) begin $display("ncoef changed before commit"); errors = errors + 1; end
    axi_read(16'h01CC, r); if (r !== 32'hD3D3_0404) begin $display("NCOEF3 shadow %h", r); errors = errors + 1; end
    axi_write(16'h0180, 32'd1);
    if (npi_eqcoef !== {32'h3333_0C0C, 32'h2222_0B0B, 32'h1111_0A0A}) begin $display("eqcoef after commit %h", npi_eqcoef); errors = errors + 1; end
    if (npi_ncoef !== {32'hF5F5_0606, 32'hE4E4_0505, 32'hD3D3_0404, 32'hC2C2_0303, 32'hB1B1_0202, 32'hA0A0_0101}) begin $display("ncoef after commit %h", npi_ncoef); errors = errors + 1; end
    if (npi_cfg !== 32'h204 || npi_nrm !== 32'h7FFF || npi_thr !== 32'd6400 || npi_hold !== 32'd30720000) begin $display("commit: cfg %h nrm %h thr %h hold %h", npi_cfg, npi_nrm, npi_thr, npi_hold); errors = errors + 1; end
    axi_read(16'h0180, r); if (r !== 32'd1) begin $display("commit count %h", r); errors = errors + 1; end
    axi_read(16'h0198, r); if (r !== 32'h204) begin $display("CFG_ACTIVE %h", r); errors = errors + 1; end
    axi_read(16'h019C, r); if (r !== 32'h00070006) begin $display("live status %h", r); errors = errors + 1; end
    axi_read(16'h0010, r); if (r[21:20] !== 2'd1) begin $display("STATUS band %h", r); errors = errors + 1; end
    axi_write(16'h019C, 32'd1);
    npi_wout = 192'd0; npi_pow = 32'd0; npi_st = 32'd0;
    axi_read(16'h01A0, r); if (r !== 32'h11111111) begin $display("W0 %h", r); errors = errors + 1; end
    axi_read(16'h01B4, r); if (r !== 32'h66666666) begin $display("W5 %h", r); errors = errors + 1; end
    axi_read(16'h01B8, r); if (r !== 32'h00001234) begin $display("POW %h", r); errors = errors + 1; end
    axi_read(16'h01BC, r); if (r !== 32'h00070006) begin $display("ST latched %h", r); errors = errors + 1; end
    axi_read(16'h019C, r); if (r !== 32'h0) begin $display("live status after change %h", r); errors = errors + 1; end
    @(posedge aclk) aresetn = 0; repeat (3) @(posedge aclk); aresetn = 1;
    if (npi_cfg !== 32'hD || npi_nrm !== 32'h2000 || npi_thr !== 32'hFFFFFFFF || npi_gamma !== 32'd1 || npi_ncoef !== 192'd0 || npi_eqcoef !== 96'd0) begin $display("reset did not restore"); errors = errors + 1; end
    // datapath: passthrough on, then switch the core in and out
    axi_write(16'h000C, 32'h1);           // pass_en
    #20000; checking = 1;
    #20000;
    axi_read(16'h0020, r); unf_base = r;
    $display("plain passthrough: jumps %0d, underflows %0d", jumps, r);
    checking = 0; jumps = 0; have = 0;
    axi_write(16'h000C, 32'h21);          // pass_en + npi_en
    #3000; checking = 1;                  // after the one-off re-prime gap
    #40000;
    axi_read(16'h0020, r);
    $display("core in path (70-clock latency): jumps %0d, new underflows %0d", jumps, r - unf_base);
    if (jumps != 0 || r != unf_base) errors = errors + 1;
    axi_read(16'h0010, r); if (!r[18]) begin $display("STATUS npi_en not set %h", r); errors = errors + 1; end
    if (r[10]) begin $display("drop flag set"); errors = errors + 1; end
    checking = 0; jumps = 0; have = 0;
    axi_write(16'h000C, 32'h1);           // core out again
    #3000; checking = 1;
    #20000;
    axi_read(16'h0020, r);
    $display("core out again: jumps %0d, new underflows %0d", jumps, r - unf_base);
    if (jumps != 0 || r != unf_base) errors = errors + 1;
    if (errors == 0) $display("PT_SIM: PASS"); else $display("PT_SIM: FAIL (%0d)", errors);
    $finish;
  end
endmodule
