// ============================================================================
//  tb_loopback_datapath.v  --  RX -> TX datapath of gnss_passthrough (CONTROL[0]
//  pass_en = 1, CONTROL[5] = 0: the loop-back), Icarus Verilog.
//
//  Case A (`DAC_DIV == ADC_DIV, the AD9361 as it should run): every RX sample of
//    both channels must come out on the DAC, in order, as {sample[11:0], 4'b0},
//    the elastic buffer must sit near half full (16 of 32), and the overflow /
//    underflow counters must stay 0.
//  Case B (-DDAC_DIV=1: TX requests 4x faster than RX delivers): the buffer runs
//    empty and the underflow counter grows -- the symptom of a TX/RX sample-rate
//    mismatch (gnss_fifo? on the board).
//  Prints DATAPATH_TB: PASS / FAIL.
// ============================================================================
`timescale 1ns/100ps
`ifndef ADC_DIV
`define ADC_DIV 4          // adc_valid every 4 l_clk (2R2T LVDS: l_clk = 4 fs)
`endif
`ifndef DAC_DIV
`define DAC_DIV 4          // dac_valid every 4 l_clk (DAC rate control 3)
`endif
module tb;
  reg aclk = 0, aresetn = 0, clk = 0, rst = 1;
  always #5 aclk = ~aclk;          // 100 MHz AXI
  always #2 clk  = ~clk;           // 250 MHz l_clk
  reg awvalid = 0, wvalid = 0, bready = 1, arvalid = 0, rready = 1;
  reg [15:0] awaddr = 0, araddr = 0; reg [31:0] wdata = 0;
  wire awready, wready, bvalid, arready, rvalid; wire [1:0] bresp, rresp; wire [31:0] rdata;

  reg  [7:0]  adc_cnt = 0, dac_cnt = 0;
  reg         adc_valid = 0, dac_valid = 0;
  reg  [15:0] ai0 = 0, aq0 = 0, ai1 = 0, aq1 = 0;
  wire [15:0] di0, dq0, di1, dq1;

  gnss_passthrough #(.CRPA_ALGO(0), .CRPA_NT(1), .CRPA_REF(0), .NWTS(1), .NW_USED(1), .NWCJ(1)) dut (
    .clk(clk), .rst(rst),
    .adc_enable_i0(1'b1), .adc_valid_i0(adc_valid), .adc_data_i0(ai0),
    .adc_enable_q0(1'b1), .adc_valid_q0(adc_valid), .adc_data_q0(aq0),
    .adc_enable_i1(1'b1), .adc_valid_i1(adc_valid), .adc_data_i1(ai1),
    .adc_enable_q1(1'b1), .adc_valid_q1(adc_valid), .adc_data_q1(aq1),
    .dac_enable_i0(1'b1), .dac_valid_i0(dac_valid), .dac_enable_q0(1'b1), .dac_valid_q0(dac_valid),
    .dac_enable_i1(1'b1), .dac_valid_i1(dac_valid), .dac_enable_q1(1'b1), .dac_valid_q1(dac_valid),
    .dma_dac_data_i0(16'h1111), .dma_dac_data_q0(16'h2222), .dma_dac_data_i1(16'h3333), .dma_dac_data_q1(16'h4444),
    .dac_data_i0(di0), .dac_data_q0(dq0), .dac_data_i1(di1), .dac_data_q1(dq1),
    .m_axis_smi_tdata(), .m_axis_smi_tvalid(), .m_axis_smi_tready(1'b0),
    .s_axis_smi_tdata(32'd0), .s_axis_smi_tvalid(1'b0), .s_axis_smi_tready(), .smi_rst_n(),
    .aj_wts(), .aj_wcj(), .aj_cfg(), .aj_nrm(), .aj_gain(),
    .s_axi_aclk(aclk), .s_axi_aresetn(aresetn),
    .s_axi_awvalid(awvalid), .s_axi_awaddr(awaddr), .s_axi_awprot(3'd0), .s_axi_awready(awready),
    .s_axi_wvalid(wvalid), .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wready(wready),
    .s_axi_bvalid(bvalid), .s_axi_bresp(bresp), .s_axi_bready(bready),
    .s_axi_arvalid(arvalid), .s_axi_araddr(araddr), .s_axi_arprot(3'd0), .s_axi_arready(arready),
    .s_axi_rvalid(rvalid), .s_axi_rresp(rresp), .s_axi_rdata(rdata), .s_axi_rready(rready));

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

  // RX source: a counting sample per channel, 12-bit signed, right-aligned
  reg [11:0] seq = 12'd0;
  integer sent = 0, running = 0;
  always @(posedge clk) begin
    adc_cnt <= (adc_cnt == `ADC_DIV - 1) ? 8'd0 : adc_cnt + 1'b1;
    dac_cnt <= (dac_cnt == `DAC_DIV - 1) ? 8'd0 : dac_cnt + 1'b1;
    adc_valid <= running && (adc_cnt == 0);
    dac_valid <= running && (dac_cnt == (`DAC_DIV > 1 ? 1 : 0));   // a phase offset to RX
    if (running && adc_cnt == 0) begin
      seq <= seq + 1'b1;
      ai0 <= {{4{seq[11]}}, seq};
      aq0 <= {{4{~seq[11]}}, ~seq};
      ai1 <= {{4{seq[10]}}, seq ^ 12'h5A5};
      aq1 <= {{4{seq[9]}}, seq + 12'd100};
      sent <= sent + 1;
    end
  end

  // TX sink: check that the samples come out in order, left-aligned
  integer got = 0, bad = 0, first = 1;
  reg [11:0] exp = 12'd0;
  reg dv_d = 0;
  always @(posedge clk) begin
    dv_d <= dac_valid;
    if (dv_d && running && di0[3:0] == 4'd0 && (di0 != 16'd0 || got > 0)) begin
      if (first) begin exp = di0[15:4]; first = 0; end
      if (di0[15:4] !== exp || dq0[15:4] !== ~exp || di1[15:4] !== (exp ^ 12'h5A5) || dq1[15:4] !== exp + 12'd100) begin
        if (bad < 5) $display("  mismatch at %0d: ch0 %h/%h ch1 %h/%h, expected %h", got, di0, dq0, di1, dq1, exp);
        bad = bad + 1;
        exp = di0[15:4];
      end
      exp = exp + 1'b1;
      got = got + 1;
    end
  end

  reg [31:0] v, lvl, ovf, unf, st; integer ok;
  initial begin
    repeat (5) @(posedge aclk); aresetn <= 1; rst <= 0;
    wr(16'h000C, 32'h1);                      // CONTROL: pass_en = 1, crpa_en = 0
    repeat (20) @(posedge aclk);
    running = 1;
    repeat (40000) @(posedge clk);            // 10000 samples
    repeat (600) @(posedge clk);              // the status snapshot refreshes every 256 clk
    rd(16'h002C, lvl); rd(16'h001C, ovf); rd(16'h0020, unf); rd(16'h0010, st);
    $display("DATAPATH: ADC every %0d clk, DAC every %0d clk: %0d samples in, %0d out, %0d out of order",
             `ADC_DIV, `DAC_DIV, sent, got, bad);
    $display("DATAPATH: buffer level ch0 %0d ch1 %0d, overflows %0d, underflows %0d, STATUS %h",
             lvl[5:0], lvl[13:8], ovf, unf, st);
    if (`DAC_DIV == `ADC_DIV)
      ok = (bad == 0) && (got > sent - 40) && (ovf == 0) && (unf == 0) &&
           (lvl[5:0] >= 8) && (lvl[5:0] <= 24) && (lvl[13:8] >= 8) && (lvl[13:8] <= 24);
    else
      ok = (unf > 0) && (lvl[5:0] <= 1);     // the mismatch must show as underflow / empty buffer
    if (`DAC_DIV == `ADC_DIV)
      $display("DATAPATH_TB: %s  matched rates: every sample out in order, buffer half full", ok ? "PASS" : "FAIL");
    else
      $display("DATAPATH_TB: %s  TX faster than RX: buffer empty, underflows counted (the gnss_fifo? symptom)", ok ? "PASS" : "FAIL");
    $finish;
  end
endmodule
