`timescale 1ns/1ps
module tb_core;
  parameter NS = 40000;                 // samples
  parameter MK = 0;
  parameter SH = 3'd5;
  parameter LOAD = 32'd30000;
  parameter signed [15:0] K1R = 0, K1I = 0, K2R = 0, K2I = 0;
  wire [191:0] iq_stat;
  reg clk = 0; always #2 clk = ~clk;
  reg en = 0, clr = 0;
  reg in_v = 0;
  reg signed [15:0] x1i, x1q, x2i, x2q;
  wire out_v; wire signed [15:0] out_i, out_q;
  wire ready, underrun, overrun; wire [15:0] frames; wire [31:0] in_mag, out_mag;
  sfap_core dut (.clk(clk), .en(en), .clr(clr), .sh_lam(SH), .load(LOAD), .mk(MK[2:0]),
      .in_v(in_v), .x1i(x1i), .x1q(x1q), .x2i(x2i), .x2q(x2q), .k1r(K1R), .k1i(K1I), .k2r(K2R), .k2i(K2I), .iq_stat(iq_stat),
      .out_v(out_v), .out_i(out_i), .out_q(out_q), .ready(ready), .underrun(underrun),
      .overrun(overrun), .frames(frames), .in_mag(in_mag), .out_mag(out_mag));
  reg [47:0] mem [0:NS-1];
  integer n, fo, c;
  initial begin
    $readmemh("core_in.hex", mem);
    fo = $fopen("core_out.txt", "w");
    repeat (10) @(negedge clk);
    en = 1;
    repeat (10) @(negedge clk);
    for (n = 0; n < NS + 9000; n = n + 1) begin
      // one sample every 4 clocks
      @(negedge clk);
      if (n < NS) begin {x2q, x2i, x1q, x1i} = {mem[n][47:36] , mem[n][35:24], mem[n][23:12], mem[n][11:0]};
        x1i = {{4{mem[n][11]}}, mem[n][11:0]}; x1q = {{4{mem[n][23]}}, mem[n][23:12]};
        x2i = {{4{mem[n][35]}}, mem[n][35:24]}; x2q = {{4{mem[n][47]}}, mem[n][47:36]}; end
      else begin x1i = 0; x1q = 0; x2i = 0; x2q = 0; end
      in_v = 1;
      @(negedge clk); in_v = 0;
      repeat (2) @(negedge clk);
    end
    $display("IQSTAT %0d %0d %0d %0d %0d %0d", $signed(iq_stat[31:0]), $signed(iq_stat[63:32]), iq_stat[95:64], $signed(iq_stat[127:96]), $signed(iq_stat[159:128]), iq_stat[191:160]);
    $display("frames=%0d ready=%b underrun=%b overrun=%b in_mag=%0d out_mag=%0d", frames, ready, underrun, overrun, in_mag, out_mag);
    $fclose(fo); $finish;
  end
  integer k = 0;
  always @(posedge clk) if (out_v) begin $fwrite(fo, "%0d %0d %0d\n", k, out_i, out_q); k = k + 1; end
endmodule
