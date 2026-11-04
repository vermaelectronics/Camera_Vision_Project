`timescale 1ns/1ps
module tb_fft;
  parameter NF = 3; parameter LN = 10; parameter [LN-1:0] SC = 10'b1000000000; localparam N = (1<<LN);
  reg clk = 0; always #2 clk = ~clk;
  reg signed [24:0] xr, xi;
  reg [LN-1:0] idx = 0;
  wire signed [24:0] yr, yi; wire [LN-1:0] yt;
  reg [49:0] mem [0:NF*2*N-1];
  integer n, fo;
  sfap_fft #(.WD(25), .LN(LN), .SCALE(SC)) dut (.clk(clk), .in_re(xr), .in_im(xi), .in_idx(idx),
      .out_re(yr), .out_im(yi), .out_idx(yt));
  initial begin
    $readmemh("fft_in.hex", mem);
    fo = $fopen("fft_out.txt", "w");
    for (n = 0; n < NF*2*N + 3*N; n = n + 1) begin
      @(negedge clk);
      if (n < NF*2*N) begin xr = mem[n][24:0]; xi = mem[n][49:25]; end else begin xr = 0; xi = 0; end
      idx = n[LN-1:0];
      #1;
    end
    $fclose(fo); $finish;
  end
  integer c = 0;
  always @(posedge clk) begin
    c <= c + 1;
    $fwrite(fo, "%0d %0d %0d %0d\n", c, yr, yi, yt);
  end
endmodule
