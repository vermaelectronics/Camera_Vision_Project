//-----------------------------------------------------------------------------
// tb_fp_units : drives random operand vectors through the floating point
// building blocks and dumps the results for bit-exact comparison against
// numpy float32 (see model/check_fp_units.py).
//-----------------------------------------------------------------------------
`timescale 1ns/1ps
module tb_fp_units;
    parameter integer NVEC = 20000;

    reg clk = 1'b0, rst_n = 1'b0;
    always #5 clk = ~clk;

    reg  [31:0] va [0:NVEC-1];
    reg  [31:0] vb [0:NVEC-1];
    reg  [47:0] vi [0:NVEC-1];
    reg  [31:0] a, b;
    reg  [47:0] iv;
    wire [31:0] y_add, y_mul, y_i2f, y_rcp;
    wire        rcp_done;
    reg         rcp_start;

    stap_fp_add u_add (.clk(clk), .a(a), .b(b), .y(y_add));
    stap_fp_mul u_mul (.clk(clk), .a(a), .b(b), .y(y_mul));
    stap_int2fp #(.W(48), .EXP_OFS(-12)) u_i2f (.clk(clk), .x(iv), .y(y_i2f));
    stap_fp_recip u_rcp (.clk(clk), .rst_n(rst_n), .start(rcp_start), .x(a),
                         .done(rcp_done), .y(y_rcp));

    integer i, fo;
    initial begin
        $readmemh("fp_a.hex", va);
        $readmemh("fp_b.hex", vb);
        $readmemh("fp_i.hex", vi);
        fo = $fopen("fp_out.txt", "w");
        rcp_start = 1'b0;
        a = 0; b = 0; iv = 0;
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        for (i = 0; i < NVEC; i = i + 1) begin
            @(negedge clk);
            a = va[i]; b = vb[i]; iv = vi[i];
            @(negedge clk); @(negedge clk);   // 2-cycle latency for add/mul
            $fwrite(fo, "%08x %08x %08x", y_add, y_mul, y_i2f);
            if (i < 2000) begin               // reciprocal is slow: subset
                rcp_start = 1'b1;
                @(negedge clk);
                rcp_start = 1'b0;
                while (!rcp_done) @(negedge clk);
                $fwrite(fo, " %08x", y_rcp);
            end
            $fwrite(fo, "\n");
        end
        $fclose(fo);
        $finish;
    end
endmodule
