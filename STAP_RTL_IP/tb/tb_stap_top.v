//-----------------------------------------------------------------------------
// tb_stap_top : system level testbench for the STAP IP
//
//   1. programs the IP over APB (steering vector, diagonal load, mode, start)
//   2. streams the array samples produced by model/stap_model.py (gen)
//   3. waits for the adaptation interrupt, reads back status and weights
//   4. streams the remaining samples through the adapted filter
//   Dumps: rtl_cov.hex rtl_w.hex rtl_wfix.hex rtl_status.txt rtl_out.txt
//   which model/stap_model.py (check) verifies bit-exactly.
//-----------------------------------------------------------------------------
`timescale 1ns/1ps
module tb_stap_top;
    parameter integer M     = 4;
    parameter integer N     = 4;
    parameter integer LOG2L = 10;
    parameter integer DW    = 16;
    parameter integer WW    = 18;
    parameter integer WFRAC = 16;
    parameter integer OW    = 16;
    parameter integer MAXS  = 8192;
    parameter integer TIMEOUT_CYC = 50_000_000;

    localparam integer NDIM = M * N;
    localparam integer NTRI = NDIM * (NDIM + 1) / 2;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #2.5 clk = ~clk;                       // 200 MHz

    // APB
    reg         psel = 1'b0, penable = 1'b0, pwrite = 1'b0;
    reg  [15:0] paddr = 16'd0;
    reg  [31:0] pwdata = 32'd0;
    wire [31:0] prdata;
    wire        pready, pslverr;
    // streams
    reg                 s_valid = 1'b0;
    reg  [M*2*DW-1:0]   s_data = {M*2*DW{1'b0}};
    wire                m_valid;
    wire [2*OW-1:0]     m_data;
    wire                irq;

    stap_top #(.M(M), .N(N), .DW(DW), .LOG2L(LOG2L), .WW(WW), .WFRAC(WFRAC), .OW(OW)) dut (
        .clk(clk), .rst_n(rst_n),
        .psel(psel), .penable(penable), .pwrite(pwrite), .paddr(paddr),
        .pwdata(pwdata), .prdata(prdata), .pready(pready), .pslverr(pslverr),
        .s_valid(s_valid), .s_data(s_data), .m_valid(m_valid), .m_data(m_data),
        .irq(irq)
    );

    //------------------------------------------------------------- APB tasks
    task apb_write(input [15:0] a, input [31:0] d);
        begin
            @(posedge clk);
            psel <= 1'b1; pwrite <= 1'b1; paddr <= a; pwdata <= d; penable <= 1'b0;
            @(posedge clk);
            penable <= 1'b1;
            @(posedge clk);
            while (!pready) @(posedge clk);
            psel <= 1'b0; penable <= 1'b0; pwrite <= 1'b0;
        end
    endtask

    task apb_read(input [15:0] a, output [31:0] d);
        begin
            @(posedge clk);
            psel <= 1'b1; pwrite <= 1'b0; paddr <= a; penable <= 1'b0;
            @(posedge clk);
            penable <= 1'b1;
            @(posedge clk);
            while (!pready) @(posedge clk);
            d = prdata;
            psel <= 1'b0; penable <= 1'b0;
        end
    endtask

    //------------------------------------------------------------- stimulus
    reg [M*2*DW-1:0] stim  [0:MAXS-1];
    reg [31:0]       steer [0:2*NDIM-1];
    reg [31:0]       cfg   [0:4];

    task stream(input integer first, input integer last);
        integer n;
        begin
            for (n = first; n < last; n = n + 1) begin
                @(posedge clk);
                s_valid <= 1'b1;
                s_data  <= stim[n];
            end
            @(posedge clk);
            s_valid <= 1'b0;
        end
    endtask

    //------------------------------------------------------------- monitors
    integer fo, out_idx = 0;
    always @(posedge clk) begin
        if (m_valid) begin
            $fwrite(fo, "%0d %08x %0d\n", out_idx, m_data, dut.r_updates);
            out_idx = out_idx + 1;
        end
        if (dut.u_cov.done) begin
            $writememh("rtl_cov.hex", dut.u_rram.mem);
            $display("[%0t] covariance done (%0d cycles since start)", $time, dut.r_cyc_cnt);
        end
        if (dut.u_sol.done)
            $display("[%0t] solver done   (%0d cycles since start) err=%0d",
                     $time, dut.r_cyc_cnt, dut.u_sol.err);
    end

    integer cyc = 0;
    always @(posedge clk) begin
        cyc = cyc + 1;
        if (cyc > TIMEOUT_CYC) begin
            $display("ERROR: timeout");
            $finish;
        end
    end

    //------------------------------------------------------------- sequence
    integer i, fw;
    reg [31:0] rd, st, wexp, alpha, cycles, upd, info;
    initial begin
        $readmemh("stim.hex", stim);
        $readmemh("steer.hex", steer);
        $readmemh("cfg.hex", cfg);
        fo = $fopen("rtl_out.txt", "w");

        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        apb_read(16'h001C, info);
        $display("INFO: M=%0d N=%0d LOG2L=%0d version=%02x",
                 info[7:0], info[15:8], info[23:16], info[31:24]);
        for (i = 0; i < 2 * NDIM; i = i + 1)
            apb_write(16'h1000 + 4 * i, steer[i]);
        apb_write(16'h0008, cfg[0]);                    // diagonal load
        apb_write(16'h000C, cfg[1]);                    // output shift
        apb_write(16'h0000, cfg[2]);                    // start | mode | irq_en

        repeat (5) @(posedge clk);
        stream(0, cfg[3]);                              // capture + some more

        wait (irq === 1'b1);
        @(posedge clk);
        apb_read(16'h0004, st);
        apb_read(16'h0014, wexp);
        apb_read(16'h0018, alpha);
        apb_read(16'h0024, cycles);
        apb_read(16'h0010, upd);
        $display("STATUS=%08x updates=%0d wexp=%0d alpha=%08x adaptation=%0d cycles",
                 st, upd, $signed(wexp), alpha, cycles);
        fw = $fopen("rtl_status.txt", "w");
        $fwrite(fw, "status=%0d wexp=%0d alpha=%0d cycles=%0d updates=%0d\n",
                st, $signed(wexp), alpha, cycles, upd);
        $fclose(fw);

        fw = $fopen("rtl_w.hex", "w");
        for (i = 0; i < 2 * NDIM; i = i + 1) begin
            apb_read(16'h2000 + 4 * i, rd);
            $fwrite(fw, "%08x\n", rd);
        end
        $fclose(fw);

        fw = $fopen("rtl_wfix.hex", "w");
        for (i = 0; i < NDIM; i = i + 1)
            $fwrite(fw, "%05x\n%05x\n", dut.u_bf.wa_re[i] & 18'h3FFFF,
                    dut.u_bf.wa_im[i] & 18'h3FFFF);
        $fclose(fw);
        apb_write(16'h0020, 32'h3);                     // clear IRQ

        if (st[2]) begin
            $display("ERROR: solver reported err_code=%0d", st[5:4]);
        end
        stream(cfg[3], cfg[4]);
        repeat (40) @(posedge clk);
        $fclose(fo);
        $display("simulation finished: %0d output samples", out_idx);
        $finish;
    end

endmodule
