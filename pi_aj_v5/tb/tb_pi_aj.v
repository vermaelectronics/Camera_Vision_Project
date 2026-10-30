// ============================================================================
// tb_pi_aj.v  --  PI-AJ v5 testbench
//   +IN=<stimulus.hex>   lines "I0 Q0 I1 Q1" (16-bit hex, from gen_stimulus.py)
//   +OUT=<prefix>        writes <prefix>_pi.txt  (PI output, pre-AGC)
//                               <prefix>_dac.txt (TX1 DAC words)
//   +CTRL=<hex> +ADAPT=<hex>   optional register overrides
//   +CPS=<n>             clocks per sample (default 5)
// ============================================================================
`timescale 1ns / 1ps
module tb_pi_aj;
    reg clk = 0, aclk = 0;
    always #5    clk  = ~clk;      // 100 MHz sample-side clock
    always #8.33 aclk = ~aclk;     // ~60 MHz AXI clock (asynchronous)

    reg rst = 1, aresetn = 0;

    reg         adc_valid = 0;
    reg  [15:0] i0 = 0, q0 = 0, i1 = 0, q1 = 0;
    wire        dac_valid;
    wire [15:0] dac_i0, dac_q0;

    reg  [6:0]  awaddr = 0; reg awvalid = 0; wire awready;
    reg  [31:0] wdata = 0;  reg wvalid = 0;  wire wready;
    wire [1:0]  bresp;      wire bvalid;     reg bready = 1;
    reg  [6:0]  araddr = 0; reg arvalid = 0; wire arready;
    wire [31:0] rdata;      wire [1:0] rresp; wire rvalid; reg rready = 1;

`ifndef TB_NL
`define TB_NL 5
`endif
`ifndef TB_N2
`define TB_N2 1
`endif
    pi_aj_top #(.NL(`TB_NL), .HAS_NOTCH2(`TB_N2)) dut (
        .clk(clk), .rst(rst),
        .adc_valid(adc_valid), .adc_i0(i0), .adc_q0(q0), .adc_i1(i1), .adc_q1(q1),
        .dac_valid(dac_valid), .dac_i0(dac_i0), .dac_q0(dac_q0),
        .s_axi_aclk(aclk), .s_axi_aresetn(aresetn),
        .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
        .s_axi_araddr(araddr), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready)
    );

    task axi_write(input [6:0] a, input [31:0] d);
        begin
            @(posedge aclk);
            awaddr <= a; awvalid <= 1; wdata <= d; wvalid <= 1;
            fork
                begin wait (awready); @(posedge aclk); awvalid <= 0; end
                begin wait (wready);  @(posedge aclk); wvalid  <= 0; end
            join
            wait (bvalid); @(posedge aclk);
        end
    endtask

    reg [31:0] rd;
    task axi_read(input [6:0] a);
        begin
            @(posedge aclk);
            araddr <= a; arvalid <= 1;
            wait (arready); @(posedge aclk); arvalid <= 0;
            wait (rvalid); rd = rdata; @(posedge aclk);
        end
    endtask

    integer fin, fpi, fdac, r, n, cps, k;
    reg [1023:0] fname, oname;
    reg [31:0] ctrl, adapt;
    reg [15:0] a, b, c, d;
    real fs;

    // dump the PI output (pre-AGC) for analysis
    always @(posedge clk)
        if (dut.pi_v)
            $fwrite(fpi, "%0d %0d\n", dut.pi_re, dut.pi_im);
    always @(posedge clk)
        if (dac_valid)
            $fwrite(fdac, "%0d %0d\n", $signed(dac_i0), $signed(dac_q0));

    function real todb(input real v);
        todb = (v > 0) ? 10.0 * $log10(v) : -999.0;
    endfunction

    real p1, po, ar, ai;
    initial begin
        if (!$value$plusargs("IN=%s", fname)) begin
            $display("need +IN="); $finish;
        end
        if (!$value$plusargs("OUT=%s", oname)) oname = "out";
        if (!$value$plusargs("CPS=%d", cps)) cps = 5;
        fin  = $fopen(fname, "r");
        fpi  = $fopen({oname, "_pi.txt"}, "w");
        fdac = $fopen({oname, "_dac.txt"}, "w");

        repeat (10) @(posedge aclk);
        aresetn = 1;
        repeat (5) @(posedge clk);
        rst = 0;
        if ($value$plusargs("CTRL=%h", ctrl))   axi_write(7'h08, ctrl);
        if ($value$plusargs("ADAPT=%h", adapt)) axi_write(7'h0C, adapt);
        axi_read(7'h00); $display("ID      = %08h", rd);
        axi_read(7'h04); $display("VERSION = %08h", rd);
        repeat (50) @(posedge clk);

        n = 0;
        while (!$feof(fin)) begin
            r = $fscanf(fin, "%h %h %h %h\n", a, b, c, d);
            if (r == 4) begin
                @(posedge clk);
                adc_valid <= 1; i0 <= a; q0 <= b; i1 <= c; q1 <= d;
                @(posedge clk);
                adc_valid <= 0;
                for (k = 2; k < cps; k = k + 1) @(posedge clk);
                n = n + 1;
            end
        end
        repeat (200) @(posedge clk);

        fs = 18.0e6;
        axi_read(7'h20); $display("STATUS  = %08h (notch1 %0d, image %0d, notch2 %0d, overload %0d, late %0d, overrun %0d, agc_shift %0d)",
                                   rd, rd[0], rd[1], rd[2], rd[3], rd[4], rd[5], $signed(rd[12:8]));
        axi_read(7'h24); p1 = rd; $display("P_RX1   = %0d", rd);
        axi_read(7'h28);          $display("P_RX2   = %0d", rd);
        axi_read(7'h2C);          $display("P_NOTCH = %0d", rd);
        axi_read(7'h30); po = rd; $display("P_OUT   = %0d", rd);
        $display("RMS suppression (status regs) = %0.1f dB", todb(p1) - todb(po));
        axi_read(7'h34);
        ar = $signed(rd[15:0]); ai = $signed(rd[31:16]);
        $display("NOTCH1_A = %08h -> %0.1f Hz", rd, $atan2(ai, ar) / 6.283185307 * fs);
        axi_read(7'h38);
        ar = $signed(rd[15:0]); ai = $signed(rd[31:16]);
        $display("NOTCH2_A = %08h -> %0.1f Hz", rd, $atan2(ai, ar) / 6.283185307 * fs);
        axi_read(7'h3C); $display("CLIPCNT = %0d", rd);
        axi_write(7'h14, 32'd4);       // weight 4 = RX2 tap 0
        repeat (40) @(posedge aclk);
        axi_read(7'h40); ar = $signed(rd) / 268435456.0;
        axi_read(7'h44); ai = $signed(rd) / 268435456.0;
        $display("W[4] (RX2, tap0) = %0.4f %+0.4fj", ar, ai);
        axi_read(7'h48); $display("SAMPLES = %0d (sent %0d)", rd, n);
        $fclose(fpi); $fclose(fdac);
        $finish;
    end
endmodule
