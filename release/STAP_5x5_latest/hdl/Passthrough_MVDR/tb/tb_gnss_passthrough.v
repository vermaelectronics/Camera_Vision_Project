`timescale 1ns/1ps
// System test of gnss_passthrough_bd (MVDR2 top) as wired in the LVDS block design:
// adc_valid every 4th l_clk, dac_valid every 4th l_clk at another phase.
module tb;
  reg aclk = 0, clk = 0;
  always #5 aclk = ~aclk;          // 100 MHz
  always #2.035 clk = ~clk;        // 245.76 MHz
  reg aresetn = 0, rst = 1;
  reg  [15:0] awaddr = 0, araddr = 0;  reg awvalid = 0, wvalid = 0, bready = 1, arvalid = 0, rready = 1;
  reg  [31:0] wdata = 0;
  wire awready, wready, bvalid, arready, rvalid;  wire [1:0] bresp, rresp;  wire [31:0] rdata;
  reg        av = 0, dv = 0;  reg [1:0] ph = 0;
  reg [15:0] i0 = 0, q0 = 0, i1 = 0, q1 = 0;
  wire [15:0] d0, dq0, d1, dq1;

  gnss_passthrough_bd dut (.clk(clk), .rst(rst),
    .adc_enable_i0(1'b1), .adc_valid_i0(av), .adc_data_i0(i0),
    .adc_enable_q0(1'b1), .adc_valid_q0(av), .adc_data_q0(q0),
    .adc_enable_i1(1'b1), .adc_valid_i1(av), .adc_data_i1(i1),
    .adc_enable_q1(1'b1), .adc_valid_q1(av), .adc_data_q1(q1),
    .dac_enable_i0(1'b1), .dac_valid_i0(dv), .dac_enable_q0(1'b1), .dac_valid_q0(dv),
    .dac_enable_i1(1'b1), .dac_valid_i1(dv), .dac_enable_q1(1'b1), .dac_valid_q1(dv),
    .dma_dac_data_i0(16'h1111), .dma_dac_data_q0(16'h2222), .dma_dac_data_i1(16'h3333), .dma_dac_data_q1(16'h4444),
    .dac_data_i0(d0), .dac_data_q0(dq0), .dac_data_i1(d1), .dac_data_q1(dq1),
    .s_axi_aclk(aclk), .s_axi_aresetn(aresetn),
    .s_axi_awvalid(awvalid), .s_axi_awaddr(awaddr), .s_axi_awprot(3'd0), .s_axi_awready(awready),
    .s_axi_wvalid(wvalid), .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wready(wready),
    .s_axi_bvalid(bvalid), .s_axi_bresp(bresp), .s_axi_bready(bready),
    .s_axi_arvalid(arvalid), .s_axi_araddr(araddr), .s_axi_arprot(3'd0), .s_axi_arready(arready),
    .s_axi_rvalid(rvalid), .s_axi_rresp(rresp), .s_axi_rdata(rdata), .s_axi_rready(rready));

  task wr(input [15:0] a, input [31:0] d); begin
    @(posedge aclk); awaddr <= a; wdata <= d; awvalid <= 1; wvalid <= 1;
    wait(awready && wready); @(posedge aclk); awvalid <= 0; wvalid <= 0;
    wait(bvalid); @(posedge aclk);
  end endtask
  task rd(input [15:0] a, output [31:0] d); begin
    @(posedge aclk); araddr <= a; arvalid <= 1;
    wait(arready); @(posedge aclk); arvalid <= 0;
    wait(rvalid); d = rdata; @(posedge aclk);
  end endtask

  function [15:0] s12(input integer t); begin
    if (t > 2047) t = 2047; if (t < -2048) t = -2048; s12 = {{4{t[11]}}, t[11:0]}; end endfunction
  integer n = 0;
  always @(posedge clk) begin
    ph <= ph + 1;
    av <= (ph == 2'd3) && !rst;
    dv <= (ph == 2'd1) && !rst;
    if (ph == 2'd3) begin
      n = n + 1;
      i0 <= s12((n * 37) % 4000 - 2000);  q0 <= s12((n * 53) % 4000 - 2000);
      i1 <= s12((n * 71) % 4000 - 2000);  q1 <= s12((n * 13) % 4000 - 2000);
    end
  end

  // golden: every DAC strobe must carry a sample that was on the ADC, x16
  integer err = 0, k, hit;
  reg [31:0] v;
  reg [15:0] hist_i [0:63]; reg [15:0] hist_q [0:63]; integer hp = 0;
  always @(posedge clk) if (av) begin hist_i[hp % 64] <= {i0[11:0],4'b0}; hist_q[hp % 64] <= {q0[11:0],4'b0}; hp <= hp + 1; end
  integer lastk = -1, jumps = 0, checked = 0, j;
  initial begin
    repeat (20) @(posedge aclk); aresetn = 1;
    repeat (20) @(posedge clk);  rst = 0;
    rd(16'h000, v); $display("ID       %h", v); if (v !== 32'h47435031) err = err + 1;
    rd(16'h004, v); $display("VERSION  %h", v);
    rd(16'h100, v); $display("MVDR ID  %h", v); if (v !== 32'h4D564452) err = err + 1;
    rd(16'h104, v); $display("MVDR VER %h", v);
    rd(16'hED0, v); $display("SMI INFO %h", v); if (v[7:0] !== 8'h53) err = err + 1;
    repeat (10) @(posedge clk);
    if (d0 !== 16'h1111 || dq1 !== 16'h4444) begin $display("FAIL dma path"); err = err + 1; end
    wr(16'h00C, 32'd1);                                // pass_en
    repeat (400) @(posedge clk);
    // every dac_valid: output equals one of the recent RX1 samples x16, consecutive ones
    for (k = 0; k < 200; k = k + 1) begin
      @(posedge clk); if (dv) begin
        #0.1; hit = -1;
        for (j = 0; j < 64; j = j + 1) if (hist_i[j] === d0 && hist_q[j] === dq0) hit = j;
        if (hit < 0) begin err = err + 1; $display("FAIL tx sample %h %h not from RX", d0, dq0); end
        else begin if (lastk >= 0 && hit != (lastk + 1) % 64) jumps = jumps + 1; lastk = hit; checked = checked + 1; end
      end
    end
    $display("passthrough: %0d DAC strobes checked, %0d discontinuities", checked, jumps);
    if (jumps > 0) err = err + 1;
    rd(16'h010, v); $display("STATUS   %h", v);
    rd(16'h02C, v); $display("FIFO lvl %h", v);
    rd(16'h01C, v); $display("OVF      %0d", v); rd(16'h020, v); $display("UNF      %0d", v);
    wr(16'h00C, 32'd0);
    repeat (40) @(posedge clk);
    if (d0 !== 16'h1111) begin $display("FAIL dma path after pass off"); err = err + 1; end
    $display("errors = %0d", err);
    $finish;
  end
endmodule
