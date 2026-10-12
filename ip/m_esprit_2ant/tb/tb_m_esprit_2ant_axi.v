`timescale 1ns/1ps
// ============================================================================
// tb_m_esprit_2ant_axi -- smoke test of the IP top (AXI4-Lite + AXI-Stream)
//
// Programs the registers over AXI4-Lite from cfg.vh, streams stim.hex over
// S_AXIS, and for every window reads the result registers after the
// interrupt, clears STATUS.result_valid, and writes rtl_results.txt in the
// same format as the core testbench (so model/m_esprit_model.py check works).
// It also checks that the M_AXIS beat matches the register values.
// ============================================================================
module tb_m_esprit_2ant_axi;

`include "cfg.vh"

`ifndef STIM_DIR
`define STIM_DIR "./"
`endif

    reg aclk = 1'b0;
    reg aresetn = 1'b0;
    always #5 aclk = ~aclk;

    reg  [63:0] mem [0:N_SAMPLES-1];

    reg  [63:0] s_tdata = 0;
    reg         s_tvalid = 0;
    wire        s_tready;
    wire [63:0] m_tdata;
    wire [3:0]  m_tuser;
    wire        m_tlast, m_tvalid;

    reg  [5:0]  awaddr = 0;  reg awvalid = 0; wire awready;
    reg  [31:0] wdata = 0;   reg wvalid = 0;  wire wready;
    wire [1:0]  bresp;       wire bvalid;     reg bready = 1;
    reg  [5:0]  araddr = 0;  reg arvalid = 0; wire arready;
    wire [31:0] rdata;       wire [1:0] rresp; wire rvalid; reg rready = 1;
    wire        irq;

    m_esprit_2ant_axi dut (
        .aclk (aclk), .aresetn (aresetn),
        .s_axis_tdata (s_tdata), .s_axis_tvalid (s_tvalid), .s_axis_tready (s_tready),
        .m_axis_tdata (m_tdata), .m_axis_tuser (m_tuser), .m_axis_tlast (m_tlast),
        .m_axis_tvalid (m_tvalid), .m_axis_tready (1'b1),
        .s_axi_awaddr (awaddr), .s_axi_awprot (3'd0), .s_axi_awvalid (awvalid), .s_axi_awready (awready),
        .s_axi_wdata (wdata), .s_axi_wstrb (4'hF), .s_axi_wvalid (wvalid), .s_axi_wready (wready),
        .s_axi_bresp (bresp), .s_axi_bvalid (bvalid), .s_axi_bready (bready),
        .s_axi_araddr (araddr), .s_axi_arprot (3'd0), .s_axi_arvalid (arvalid), .s_axi_arready (arready),
        .s_axi_rdata (rdata), .s_axi_rresp (rresp), .s_axi_rvalid (rvalid), .s_axi_rready (rready),
        .irq (irq)
    );

    task axi_write(input [5:0] a, input [31:0] d);
        begin
            @(posedge aclk);
            awaddr <= a; awvalid <= 1'b1; wdata <= d; wvalid <= 1'b1;
            @(posedge aclk);
            while (!awready) @(posedge aclk);
            awvalid <= 1'b0; wvalid <= 1'b0;
            while (!bvalid) @(posedge aclk);
        end
    endtask

    task axi_read(input [5:0] a, output [31:0] d);
        begin
            @(posedge aclk);
            araddr <= a; arvalid <= 1'b1;
            @(posedge aclk);
            while (!arready) @(posedge aclk);
            arvalid <= 1'b0;
            while (!rvalid) @(posedge aclk);
            d = rdata;
        end
    endtask

    // capture M_AXIS beats
    reg [63:0] last_beat;
    integer    beats = 0;
    always @(posedge aclk) if (m_tvalid) begin last_beat <= m_tdata; beats = beats + 1; end

    // stream snapshots in the background (paced so each window's result
    // is read before the next window ends)
    integer k = 0;
    reg     stream_go = 0;
    always @(posedge aclk) begin
        if (stream_go && k < N_SAMPLES) begin
            s_tvalid <= 1'b1;
            s_tdata  <= mem[k];
            k = k + 1;
        end else begin
            s_tvalid <= 1'b0;
        end
    end

    integer fo, w, errors;
    reg [31:0] r, phi, psi, th, deg, st, id;

    initial begin
        errors = 0;
        $readmemh({`STIM_DIR, "stim.hex"}, mem);
        fo = $fopen({`STIM_DIR, "rtl_results.txt"}, "w");
        repeat (5) @(posedge aclk);
        aresetn <= 1'b1;
        repeat (2) @(posedge aclk);

        axi_read(6'h3C, id);
        if (id !== 32'h4D455332) begin $display("ERROR: ID = %h", id); errors = errors + 1; end
        axi_write(6'h08, CFG_LOG2_SNAP);
        axi_write(6'h0C, CFG_P_LOG2);
        axi_write(6'h10, CFG_MU_Q30);
        axi_write(6'h14, CFG_K_Q30);
        axi_write(6'h00, 32'h5 | (CFG_USE_COV << 3));   // enable, irq_en
        axi_read(6'h14, r);
        if (r !== CFG_K_Q30) begin $display("ERROR: K readback %h", r); errors = errors + 1; end

        stream_go = 1;
        for (w = 0; w < N_WINDOWS; w = w + 1) begin
            while (!irq) @(posedge aclk);
            axi_read(6'h18, phi);
            axi_read(6'h1C, psi);
            axi_read(6'h20, th);
            axi_read(6'h24, deg);
            axi_read(6'h04, st);
            axi_write(6'h04, 32'h1);                    // W1C result_valid
            $fwrite(fo, "%08x %08x %08x %08x %01x\n", phi, psi, th, deg, st[2:1]);
            $display("AXI TB: window %0d theta = %f deg, STATUS = %h, M_AXIS = %h",
                     w, $signed(deg) / 65536.0, st, last_beat);
            if (last_beat !== {deg, phi}) begin
                $display("ERROR: M_AXIS beat mismatch"); errors = errors + 1;
            end
            if (st[31:16] != w + 1) begin
                $display("ERROR: result counter %0d", st[31:16]); errors = errors + 1;
            end
        end
        @(posedge aclk);
        if (irq) begin $display("ERROR: irq not cleared"); errors = errors + 1; end
        $fclose(fo);
        $display("AXI TB: done, %0d M_AXIS beats, %0d errors", beats, errors);
        $finish;
    end

endmodule
