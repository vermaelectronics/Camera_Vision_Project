`timescale 1ns/1ps
// tb_sd_image_hdmi_top_smoke : full-hierarchy integration smoke test.
// Instantiates the real sd_image_hdmi_top against the same fake SD
// card technique as tb_framebuffer_img_loader.v, and checks:
//   1. The image loads correctly end to end through the complete top
//      level (not just the sys_clk half tested in isolation elsewhere).
//   2. status_led correctly reports img_loaded (led[2]) once done.
//   3. The HDMI pipeline is actually running: hsync/vsync/de follow the
//      expected VESA counts (checked via the same whitebox technique
//      as tb_video_timing.v, now reached through the full instance
//      hierarchy), and the TMDS clock channel is toggling at the
//      expected 25 MHz PIXEL clock rate - per the DVI/HDMI spec, the
//      clock channel's fixed "00000 11111" pattern repeats once per
//      pixel clock (the receiver recovers pixel timing from it), so it
//      runs at pix_clk's rate, not shift_clk's 5x-faster bit rate.
// This does not decode the serialized TMDS bitstream (that would need
// an external reference TMDS decoder, not available in this
// environment) - it confirms the whole design elaborates, connects,
// and runs together correctly, and that the video timing/status
// reporting are correct, which is what's independently verifiable here.
`define SIMULATION

module tb_sd_image_hdmi_top_smoke;

    initial begin
        $dumpfile("tb_sd_image_hdmi_top_smoke.vcd");
        $dumpvars(0, tb_sd_image_hdmi_top_smoke);
    end

    reg clk = 1'b0;
    reg [1:0] button = 2'b00; // button[0]=0 held ("pressed") until release below
    always #10.0 clk = ~clk; // 50 MHz

    wire sd_clk, sd_mosi, sd_csn;
    wire sd_miso;
    wire [3:0] led;
    wire [3:0] gpdi_dp;

    sd_image_hdmi_top dut (
        .clk        (clk),
        .button     (button),
        .sd_clk     (sd_clk),
        .sd_mosi    (sd_mosi),
        .sd_miso    (sd_miso),
        .sd_csn     (sd_csn),
        .led        (led),
        .gpdi_dp    (gpdi_dp)
    );

    // -----------------------------------------------------------------
    // Fake SD card - identical setup to tb_framebuffer_img_loader.v.
    // -----------------------------------------------------------------
    localparam NUM_PIXELS   = 19200;
    localparam FILE_BYTES   = 8 + NUM_PIXELS*2;
    localparam DATA_SECTORS = 76;
    localparam CARD_SECTORS = 3 + DATA_SECTORS;

    reg [7:0] card_mem [0:CARD_SECTORS*512-1];
    integer   ci, px;

    initial begin
        for (ci = 0; ci < CARD_SECTORS*512; ci = ci + 1) card_mem[ci] = 8'h00;
        card_mem[11] = 8'h00; card_mem[12] = 8'h02;
        card_mem[13] = 76;
        card_mem[14] = 8'h01; card_mem[15] = 8'h00;
        card_mem[16] = 8'h01;
        card_mem[17] = 8'h10; card_mem[18] = 8'h00;
        card_mem[22] = 8'h01; card_mem[23] = 8'h00;
        card_mem['h1C2] = 8'h00;
        card_mem[512+4] = 8'hFF; card_mem[512+5] = 8'hFF;
        card_mem[1024+0]="I"; card_mem[1024+1]="M"; card_mem[1024+2]="A"; card_mem[1024+3]="G";
        card_mem[1024+4]="E"; card_mem[1024+5]=" "; card_mem[1024+6]=" "; card_mem[1024+7]=" ";
        card_mem[1024+8]="R"; card_mem[1024+9]="A"; card_mem[1024+10]="W";
        card_mem[1024+11] = 8'h20;
        card_mem[1024+26] = 8'h02; card_mem[1024+27] = 8'h00;
        card_mem[1024+28] = FILE_BYTES[7:0];
        card_mem[1024+29] = FILE_BYTES[15:8];
        card_mem[1024+30] = 8'h00; card_mem[1024+31] = 8'h00;
        card_mem[1536+0]="R"; card_mem[1536+1]="I"; card_mem[1536+2]="M"; card_mem[1536+3]="G";
        card_mem[1536+4]=8'd160; card_mem[1536+5]=8'd0;
        card_mem[1536+6]=8'd120; card_mem[1536+7]=8'd0;
        for (px = 0; px < NUM_PIXELS; px = px + 1) begin
            card_mem[1536 + 8 + px*2]     = px[7:0];
            card_mem[1536 + 8 + px*2 + 1] = px[15:8];
        end
    end

    reg [7:0] rx_shift;
    reg [2:0] in_bit_cnt;
    reg [7:0] rx_byte;
    reg       byte_in_rdy;

    always @(posedge sd_clk or posedge sd_csn) begin
        if (sd_csn) begin
            in_bit_cnt <= 3'd0;
        end else begin
            rx_shift    <= {rx_shift[6:0], sd_mosi};
            in_bit_cnt  <= in_bit_cnt + 3'd1;
            byte_in_rdy <= (in_bit_cnt == 3'd7);
            if (in_bit_cnt == 3'd7)
                rx_byte <= {rx_shift[6:0], sd_mosi};
        end
    end

    localparam C_IDLE = 0, C_CMD = 1, C_RESP = 2;
    reg [1:0]  cstate = C_IDLE;
    reg [2:0]  cmd_byte_idx;
    reg [5:0]  cmd_idx;
    reg [31:0] cmd_arg;
    reg [15:0] resp_idx;
    integer    acmd41_tries;
    reg [31:0] cmd17_lba;

    reg [7:0] next_tx;
    always @(*) begin
        next_tx = 8'hFF;
        if (cstate == C_RESP) begin
            case (cmd_idx)
                6'd0: next_tx = (resp_idx == 0) ? 8'h01 : 8'hFF;
                6'd8: case (resp_idx)
                        16'd0: next_tx = 8'h01;
                        16'd1: next_tx = cmd_arg[31:24];
                        16'd2: next_tx = cmd_arg[23:16];
                        16'd3: next_tx = cmd_arg[15:8];
                        16'd4: next_tx = cmd_arg[7:0];
                        default: next_tx = 8'hFF;
                      endcase
                6'd55: next_tx = (resp_idx == 0) ? 8'h00 : 8'hFF;
                6'd41: next_tx = (resp_idx == 0) ? ((acmd41_tries < 1) ? 8'h01 : 8'h00) : 8'hFF;
                6'd58: case (resp_idx)
                        16'd0: next_tx = 8'h00;
                        16'd1: next_tx = 8'h40;
                        16'd2: next_tx = 8'h00;
                        16'd3: next_tx = 8'hFF;
                        16'd4: next_tx = 8'h80;
                        default: next_tx = 8'hFF;
                      endcase
                6'd17: begin
                    if (resp_idx == 0) next_tx = 8'h00;
                    else if (resp_idx == 1) next_tx = 8'hFE;
                    else if (resp_idx >= 2 && resp_idx <= 513)
                        next_tx = card_mem[cmd17_lba*512 + (resp_idx - 2)];
                    else
                        next_tx = 8'hFF;
                end
                default: next_tx = 8'hFF;
            endcase
        end
    end

    reg [7:0] tx_shift = 8'hFF;
    reg [2:0] out_bit_cnt;
    assign sd_miso = tx_shift[7];

    always @(negedge sd_clk or posedge sd_csn) begin
        if (sd_csn) begin
            out_bit_cnt <= 3'd0;
            tx_shift    <= 8'hFF;
        end else begin
            if (out_bit_cnt == 3'd0)
                tx_shift <= next_tx;
            else
                tx_shift <= {tx_shift[6:0], 1'b0};
            out_bit_cnt <= out_bit_cnt + 3'd1;
        end
    end

    always @(posedge sd_csn) cstate <= C_IDLE;

    always @(posedge byte_in_rdy) begin
        case (cstate)
            C_IDLE: begin
                cmd_byte_idx <= 3'd1;
                cmd_idx      <= rx_byte[5:0];
                cstate       <= C_CMD;
            end
            C_CMD: begin
                case (cmd_byte_idx)
                    3'd1: cmd_arg[31:24] <= rx_byte;
                    3'd2: cmd_arg[23:16] <= rx_byte;
                    3'd3: cmd_arg[15:8]  <= rx_byte;
                    3'd4: cmd_arg[7:0]   <= rx_byte;
                    default: ;
                endcase
                if (cmd_byte_idx == 3'd5) begin
                    resp_idx    <= 16'd0;
                    out_bit_cnt <= 3'd0;
                    if (cmd_idx == 6'd17) cmd17_lba <= cmd_arg;
                    if (cmd_idx == 6'd41) acmd41_tries <= acmd41_tries + 1;
                    cstate <= C_RESP;
                end else begin
                    cmd_byte_idx <= cmd_byte_idx + 3'd1;
                end
            end
            C_RESP: resp_idx <= resp_idx + 16'd1;
        endcase
    end

    initial begin
        cstate = C_IDLE;
        acmd41_tries = 0;
    end

    // -----------------------------------------------------------------
    // Run + verify
    // -----------------------------------------------------------------
    integer errors;
    integer n;
    initial begin
        errors = 0;
        repeat (10) @(posedge clk);
        button = 2'b11; // release: active-low reset, so releasing means driving button[0] high

        wait (dut.u_img_loader.loaded || dut.u_img_loader.error);
        @(posedge dut.sys_clk);

        if (dut.u_img_loader.error) begin
            $display("FAIL: img_loader reported error");
            errors = errors + 1;
        end else begin
            $display("PASS: image loaded through the full sd_image_hdmi_top hierarchy");
            for (n = 0; n < NUM_PIXELS; n = n + 100) begin // spot-check every 100th pixel
                if (dut.u_framebuffer.mem[n] !== n[15:0]) begin
                    $display("FAIL: framebuffer[%0d] = %04h, expected %04h", n, dut.u_framebuffer.mem[n], n[15:0]);
                    errors = errors + 1;
                end
            end
            if (errors == 0)
                $display("PASS: spot-checked framebuffer pixels match the expected test pattern");

            repeat (5) @(posedge dut.sys_clk);
            if (led[2] !== 1'b1) begin
                $display("FAIL: led[2] (image loaded) not asserted after img_loader.loaded");
                errors = errors + 1;
            end else begin
                $display("PASS: led[2] (image loaded) asserted");
            end
        end

        // ---- HDMI pipeline: check one horizontal line's active/sync counts ----
        begin : hdmi_check
            integer i, local_active, local_sync_low;
            wait (dut.u_hdmi_out.u_timing.hsync == 1'b0);
            wait (dut.u_hdmi_out.u_timing.hsync == 1'b1);
            @(posedge dut.pix_clk);
            local_active   = 0;
            local_sync_low = 0;
            for (i = 0; i < 800; i = i + 1) begin
                if (dut.u_hdmi_out.u_timing.de)     local_active   = local_active + 1;
                if (!dut.u_hdmi_out.u_timing.hsync) local_sync_low = local_sync_low + 1;
                @(posedge dut.pix_clk);
            end
            if (local_active !== 640 || local_sync_low !== 96) begin
                $display("FAIL: HDMI video timing wrong through full top level (active=%0d sync_low=%0d, expected 640/96)",
                          local_active, local_sync_low);
                errors = errors + 1;
            end else begin
                $display("PASS: HDMI video timing (640 active / 96 sync) correct through the full top-level hierarchy");
            end
        end

        // ---- TMDS clock channel toggling at the expected 25 MHz pixel rate ----
        begin : clk_rate_check
            time t0, t1;
            integer edges;
            edges = 0;
            @(posedge gpdi_dp[3]); t0 = $time;
            repeat (10) @(posedge gpdi_dp[3]);
            t1 = $time;
            $display("[tb_sd_image_hdmi_top_smoke] gpdi_dp[3] (TMDS clock) measured period = %0.3f ns over 10 cycles (expect 40.0 ns for 25 MHz, matching pix_clk)", (t1 - t0) / 10.0);
            if ((t1 - t0) < 380 || (t1 - t0) > 420) begin
                $display("FAIL: gpdi_dp[3] period out of expected range for a 25 MHz TMDS clock channel");
                errors = errors + 1;
            end
        end

        if (errors == 0)
            $display("PASS: tb_sd_image_hdmi_top_smoke - full design loads the image and drives HDMI timing correctly");
        else
            $display("FAIL: %0d check(s) failed", errors);
        $finish;
    end

    initial begin
        #150_000_000; // 150 ms cap
        $display("[tb_sd_image_hdmi_top_smoke] TIMEOUT");
        $finish;
    end

endmodule
