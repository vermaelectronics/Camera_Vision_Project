`timescale 1ns/1ps
module tb_bin;
  parameter NFR = 12;
  reg clk = 0; always #2 clk = ~clk;
  reg v0 = 0; reg [9:0] t0 = 0;
  reg signed [24:0] x1r, x1i, x2r, x2i;
  reg init0 = 1; reg [2:0] sh0 = 3'd3; reg [31:0] load0 = 32'd2000;
  wire ov; wire [9:0] ot; wire signed [24:0] yr, yi;
  sfap_bin dut (.clk(clk), .v0(v0), .t0(t0), .x1r0(x1r), .x1i0(x1i), .x2r0(x2r), .x2i0(x2i),
                .init0(init0), .sh0(sh0), .load0(load0), .ov(ov), .ot(ot), .yr(yr), .yi(yi));
  reg [99:0] mem [0:NFR*1024-1];
  integer n, fo, fr, k;
  initial begin
    $readmemh("bin_in.hex", mem);
    fo = $fopen("bin_out.txt", "w");
    for (fr = 0; fr < NFR; fr = fr + 1) begin
      init0 = (fr == 0);
      for (k = 0; k < 1024; k = k + 1) begin
        @(negedge clk);
        v0 = 1; t0 = k;
        {x2i, x2r, x1i, x1r} = mem[fr*1024 + k];
      end
      // idle gap between frames (like the real schedule)
      @(negedge clk); v0 = 0;
      repeat (1100) @(negedge clk);
    end
    repeat (100) @(negedge clk);
    $fclose(fo); $finish;
  end
  always @(posedge clk) if (ov) $fwrite(fo, "%0d %0d %0d\n", ot, yr, yi);
endmodule
