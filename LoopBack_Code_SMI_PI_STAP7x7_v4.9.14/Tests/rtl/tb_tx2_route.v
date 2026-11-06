// Testbench for the 4.9.14 TX2 routing in gnss_passthrough.v:
//   CONTROL[3] ch1_raw  0 = TX2 fed from ch0 (the CRPA output), 1 = raw RX2
//   CONTROL[4] tx2_on   0 = TX2 forced to 0x0000 (muted),  1 = TX2 live
//   CONTROL[1] mute     still mutes both channels.
// smi_pi_0 (a separate block-design IP) is replaced by a stub that returns
// y = RX1 + 1000 (I) and RX1 - 300 (Q), so a TX word can be told apart:
//   CRPA output : I in [1000, 1511]      raw RX1 : I in [0, 511]
//   raw RX2     : I in [-1024, -513]     DMA     : 0x5550 / 0x6660 words
// Run with Tests/rtl/run_tx2_rtl.sh. Prints TX2_RTL: PASS or TX2_RTL: FAIL.
`timescale 1ns/100ps
module tb_tx2_route;
    reg clk = 0, s_clk = 0, rst = 1, s_rstn = 0;
    always #4 clk   = ~clk;     // sample-interface clock
    always #5 s_clk = ~s_clk;   // sys_cpu_clk (AXI-Lite)

    // ---- RX / TX strobes: one sample pair every 2nd clk ---------------------
    reg        ph = 0;
    reg [15:0] n  = 0;
    always @(posedge clk) begin
        ph <= ~ph;
        if (ph) n <= n + 1'b1;
    end
    wire        adc_v = ph;
    wire        dac_v = ~ph;
    wire [15:0] rx1_i = {7'd0, n[8:0]};                      //    0 ..  511
    wire [15:0] rx1_q = {8'd0, n[7:0]} + 16'd64;             //   64 ..  319
    wire [15:0] rx2_i = 16'hFC00 + {7'd0, n[8:0]};           // -1024 .. -513
    wire [15:0] rx2_q = 16'hFE00 + {8'd0, n[7:0]};           //  -512 .. -257

    // ---- smi_pi_0 stub (AXI-Stream, 3-clock latency) ------------------------
    wire [63:0] m_tdata;  wire m_tvalid;  wire smi_rst_n;
    reg  [31:0] y_d0 = 0, y_d1 = 0, y_d2 = 0;
    reg         v_d0 = 0, v_d1 = 0, v_d2 = 0;
    always @(posedge clk) begin
        v_d0 <= m_tvalid & smi_rst_n;
        y_d0 <= {m_tdata[31:16] - 16'd300, m_tdata[15:0] + 16'd1000};
        v_d1 <= v_d0; y_d1 <= y_d0;
        v_d2 <= v_d1; y_d2 <= y_d1;
    end

    // ---- AXI-Lite master ----------------------------------------------------
    reg         awvalid = 0, wvalid = 0, bready = 1;
    reg  [15:0] awaddr  = 0;
    reg  [31:0] wdata   = 0;
    wire        awready, wready, bvalid;
    wire [ 1:0] bresp, rresp;
    wire        arready, rvalid;
    wire [31:0] rdata;
    task axi_write(input [15:0] a, input [31:0] d);
        begin
            @(posedge s_clk);
            awaddr <= a; wdata <= d; awvalid <= 1; wvalid <= 1;
            @(posedge s_clk);
            while (!awready) @(posedge s_clk);
            awvalid <= 0; wvalid <= 0;
            while (!bvalid) @(posedge s_clk);
            @(posedge s_clk);
        end
    endtask

    wire [15:0] d_i0, d_q0, d_i1, d_q1;
    gnss_passthrough dut (
        .clk(clk), .rst(rst),
        .adc_enable_i0(1'b1), .adc_valid_i0(adc_v), .adc_data_i0(rx1_i),
        .adc_enable_q0(1'b1), .adc_valid_q0(adc_v), .adc_data_q0(rx1_q),
        .adc_enable_i1(1'b1), .adc_valid_i1(adc_v), .adc_data_i1(rx2_i),
        .adc_enable_q1(1'b1), .adc_valid_q1(adc_v), .adc_data_q1(rx2_q),
        .dac_enable_i0(1'b1), .dac_valid_i0(dac_v), .dac_enable_q0(1'b1), .dac_valid_q0(dac_v),
        .dac_enable_i1(1'b1), .dac_valid_i1(dac_v), .dac_enable_q1(1'b1), .dac_valid_q1(dac_v),
        .dma_dac_data_i0(16'h5550), .dma_dac_data_q0(16'h5560),
        .dma_dac_data_i1(16'h6660), .dma_dac_data_q1(16'h6670),
        .dac_data_i0(d_i0), .dac_data_q0(d_q0), .dac_data_i1(d_i1), .dac_data_q1(d_q1),
        .m_axis_smi_tdata(m_tdata), .m_axis_smi_tvalid(m_tvalid), .m_axis_smi_tready(1'b1),
        .s_axis_smi_tdata(y_d2), .s_axis_smi_tvalid(v_d2), .s_axis_smi_tready(),
        .smi_rst_n(smi_rst_n), .aj_wts(), .aj_wcj(), .aj_cfg(), .aj_nrm(),
        .s_axi_aclk(s_clk), .s_axi_aresetn(s_rstn),
        .s_axi_awvalid(awvalid), .s_axi_awaddr(awaddr), .s_axi_awprot(3'd0), .s_axi_awready(awready),
        .s_axi_wvalid(wvalid), .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wready(wready),
        .s_axi_bvalid(bvalid), .s_axi_bresp(bresp), .s_axi_bready(bready),
        .s_axi_arvalid(1'b0), .s_axi_araddr(16'd0), .s_axi_arprot(3'd0), .s_axi_arready(arready),
        .s_axi_rvalid(rvalid), .s_axi_rresp(rresp), .s_axi_rdata(rdata), .s_axi_rready(1'b1));

    // ---- classification of a TX word ----------------------------------------
    localparam C_ZERO = 0, C_CRPA = 1, C_RX1 = 2, C_RX2 = 3, C_DMA0 = 4, C_DMA1 = 5, C_OTHER = 6;
    function integer cls(input [15:0] i, input [15:0] q, input integer ch);
        reg signed [11:0] si;
        begin
            si = i[15:4];
            if (i == 16'd0 && q == 16'd0)                         cls = C_ZERO;
            else if (i == 16'h5550 && q == 16'h5560)              cls = C_DMA0;
            else if (i == 16'h6660 && q == 16'h6670)              cls = C_DMA1;
            else if (i[3:0] != 0 || q[3:0] != 0)                  cls = C_OTHER;
            else if (si >= 1000 && si <= 1511)                    cls = C_CRPA;
            else if (si >= 0 && si <= 511)                        cls = C_RX1;
            else if (si >= -1024 && si <= -513)                   cls = C_RX2;
            else                                                  cls = C_OTHER;
        end
    endfunction

    integer errors = 0;
    integer k, c0, c1, same, bad0, bad1;
    // Write CONTROL, let the buffers settle, then check 300 DAC strobes.
    //   exp0/exp1 : the class every TX1/TX2 word must have
    //   eq        : 1 = TX2 must equal TX1 word for word
    task phase(input [31:0] ctrl, input integer exp0, input integer exp1, input integer eq,
               input [8*40-1:0] name);
        begin
            axi_write(16'h000C, ctrl);
            repeat (400) @(posedge clk);
            same = 0; bad0 = 0; bad1 = 0;
            for (k = 0; k < 300; k = k + 1) begin
                @(posedge clk);
                if (dac_v) begin
                    c0 = cls(d_i0, d_q0, 0);
                    c1 = cls(d_i1, d_q1, 1);
                    if (c0 != exp0) bad0 = bad0 + 1;
                    if (c1 != exp1) bad1 = bad1 + 1;
                    if (eq && (d_i1 != d_i0 || d_q1 != d_q0)) same = same + 1;
                end
            end
            if (bad0 || bad1 || same) begin
                errors = errors + 1;
                $display("TX2_RTL: %0s CONTROL=0x%02h FAIL  TX1 wrong %0d, TX2 wrong %0d, TX2!=TX1 %0d (last TX1 0x%04h/0x%04h TX2 0x%04h/0x%04h)",
                         name, ctrl, bad0, bad1, same, d_i0, d_q0, d_i1, d_q1);
            end else
                $display("TX2_RTL: %0s CONTROL=0x%02h ok   (TX1 0x%04h/0x%04h, TX2 0x%04h/0x%04h)",
                         name, ctrl, d_i0, d_q0, d_i1, d_q1);
        end
    endtask

    initial begin
        repeat (10) @(posedge s_clk);
        rst = 0; s_rstn = 1;
        repeat (10) @(posedge s_clk);
        //      CONTROL  TX1      TX2      TX2==TX1
        phase(32'h21, C_CRPA, C_ZERO, 0, "CRPA on, TX2 default (muted)   ");
        phase(32'h31, C_CRPA, C_CRPA, 1, "CRPA on, tx2_on = 1 (CRPA copy)");
        phase(32'h39, C_CRPA, C_RX2,  0, "CRPA on, ch1_raw + tx2_on      ");
        phase(32'h33, C_ZERO, C_ZERO, 0, "mute (both channels)           ");
        phase(32'h29, C_CRPA, C_ZERO, 0, "ch1_raw alone: TX2 still muted ");
        phase(32'h01, C_RX1,  C_ZERO, 0, "CRPA off (TX1 = RX1), TX2 muted");
        phase(32'h11, C_RX1,  C_RX1,  1, "CRPA off, tx2_on = 1 (TX1 copy)");
        phase(32'h00, C_DMA0, C_ZERO, 0, "vendor DMA, TX2 muted          ");
        phase(32'h10, C_DMA0, C_DMA1, 0, "vendor DMA, tx2_on = 1         ");
        phase(32'h21, C_CRPA, C_ZERO, 0, "back to the firmware default   ");
        if (errors == 0) $display("TX2_RTL: PASS");
        else             $display("TX2_RTL: FAIL (%0d phases)", errors);
        $finish;
    end
endmodule
