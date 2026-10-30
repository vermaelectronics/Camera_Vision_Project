// ============================================================================
// tb_pi_aj_axis.v  --  PI-AJ v5 AXI-Stream wrapper (smi_pi_0 replacement)
//
// Drives pi_aj_axis the way the AntSDR block design does: samples arrive at a
// fractional rate (+FS_MHZ, e.g. 30.72) against the 100 MHz sys_cpu_clk and
// queue in a model of smi_fifo_in (depth 64). Registers are written as the
// firmware does (CONTROL, ADAPT with the make-up shift).
//   +IN=<stimulus.hex> +OUT=<prefix> +FS_MHZ=<rate> +CTRL=<hex> +ADAPT=<hex>
// Writes <prefix>_y.txt (12-bit TX1 samples) and prints status/CAPS.
// ============================================================================
`timescale 1ns / 1ps
module tb_pi_aj_axis;
    reg aclk = 0;
    always #5 aclk = ~aclk;        // 100 MHz
    reg aresetn = 0;

    reg  [63:0] x_tdata = 0;
    wire        x_tvalid;
    wire        x_tready;
    wire [31:0] y_tdata;
    wire        y_tvalid;

    reg  [11:0] awaddr = 0; reg awvalid = 0; wire awready;
    reg  [31:0] wdata = 0;  reg wvalid = 0;  wire wready;
    wire [1:0]  bresp;      wire bvalid;     reg bready = 1;
    reg  [11:0] araddr = 0; reg arvalid = 0; wire arready;
    wire [31:0] rdata;      wire [1:0] rresp; wire rvalid; reg rready = 1;

    pi_aj_axis dut (
        .aclk(aclk), .aresetn(aresetn),
        .s_axis_x_tdata(x_tdata), .s_axis_x_tvalid(x_tvalid), .s_axis_x_tready(x_tready),
        .m_axis_y_tdata(y_tdata), .m_axis_y_tvalid(y_tvalid), .m_axis_y_tready(1'b1),
        .s_axi_awaddr(awaddr), .s_axi_awprot(3'd0), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
        .s_axi_araddr(araddr), .s_axi_arprot(3'd0), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready)
    );

    task axi_write(input [11:0] a, input [31:0] d);
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
    task axi_read(input [11:0] a);
        begin
            @(posedge aclk);
            araddr <= a; arvalid <= 1;
            wait (arready); @(posedge aclk); arvalid <= 0;
            wait (rvalid); rd = rdata; @(posedge aclk);
        end
    endtask

    // ---- stimulus, streamed from the file into a queue -------------------------
    reg [63:0] q [0:255];
    integer nin, fin, r;
    reg [15:0] a, b, c, d;
    reg [1023:0] fname, oname;
    real fs_mhz, acc;
    reg [31:0] ctrl, adapt;
    integer prod, cons, nout, maxq, fy;
    reg running;

    // producer: one sample becomes due every 100/fs clocks (fractional);
    // it is read from the file and queued
    reg eof = 0;
    always @(posedge aclk) begin
        if (running && !eof) begin
            acc = acc + fs_mhz;
            if (acc >= 100.0) begin
                acc = acc - 100.0;
                r = $fscanf(fin, "%h %h %h %h\n", a, b, c, d);
                if (r == 4) begin
                    q[prod & 255] = {d, c, b, a};
                    prod = prod + 1;
                end else
                    eof = 1;
            end
        end
    end
    // smi_fifo_in model: present the oldest due sample (updated after each edge)
    assign x_tvalid = (cons < prod);
    always @(posedge aclk) begin
        if (x_tvalid && x_tready)
            cons = cons + 1;
        if (prod - cons > maxq) maxq = prod - cons;
        #1 x_tdata = q[cons & 255];
    end
    always @(posedge aclk)
        if (y_tvalid) begin
            nout = nout + 1;
            $fwrite(fy, "%0d %0d\n", $signed(y_tdata[15:0]), $signed(y_tdata[31:16]));
        end

    initial begin
        if (!$value$plusargs("IN=%s", fname)) begin $display("need +IN="); $finish; end
        if (!$value$plusargs("OUT=%s", oname)) oname = "axis";
        if (!$value$plusargs("FS_MHZ=%f", fs_mhz)) fs_mhz = 30.72;
        if (!$value$plusargs("CTRL=%h", ctrl)) ctrl = 32'h0000041B;    // firmware: AGC off
        if (!$value$plusargs("ADAPT=%h", adapt)) adapt = 32'h0008C600;
        fin = $fopen(fname, "r");
        nin = 0;
        fy = $fopen({oname, "_y.txt"}, "w");
        prod = 0; cons = 0; nout = 0; maxq = 0; acc = 0.0; running = 0;

        repeat (10) @(posedge aclk);
        aresetn = 1;
        repeat (10) @(posedge aclk);
        axi_write(12'h008, ctrl);
        axi_write(12'h00C, adapt);
        axi_write(12'h018, 32'd1);          // fresh start, as the firmware does
        repeat (20) @(posedge aclk);
        running = 1;
        wait (eof && cons == prod);
        nin = prod;
        repeat (100) @(posedge aclk);

        axi_read(12'h04C);
        $display("CAPS     = %08h (min %0d clocks/sample, NL %0d, L %0d, NW %0d) -> max %0.2f MS/s @100 MHz",
                 rd, rd[31:24], rd[23:16], rd[15:8], rd[7:0], 100.0 / rd[31:24]);
        axi_read(12'h020);
        $display("STATUS   = %08h (notch1 %0d image %0d notch2 %0d overload %0d late %0d overrun %0d)",
                 rd, rd[0], rd[1], rd[2], rd[3], rd[4], rd[5]);
        axi_read(12'h024); $display("P_RX1    = %0d", rd);
        axi_read(12'h030); $display("P_OUT    = %0d", rd);
        $display("samples in %0d, out %0d, input queue max %0d (smi_fifo_in depth 64)", nin, nout, maxq);
        $fclose(fy);
        $finish;
    end
endmodule
