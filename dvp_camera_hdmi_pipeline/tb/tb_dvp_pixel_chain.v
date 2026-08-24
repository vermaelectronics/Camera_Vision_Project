// ============================================================================
// tb_dvp_pixel_chain.v -- end-to-end check of dvp_capture + pixel_formatter
// ----------------------------------------------------------------------------
// Drives a synthetic HREF/VSYNC/D[7:0] byte sequence representing one video
// line for each supported camera format and checks the resulting RGB888
// pixel stream against hand-computed expected values.
//
// Signal-timing convention used throughout: every stimulus signal (d, href,
// vsync) is assigned with a blocking assignment *mid-cycle* (right after a
// previous @(posedge clk) resumes), then the code waits a full cycle with
// @(posedge clk) before the DUT samples it -- this avoids the classic
// same-edge race between testbench stimulus and DUT sampling.
// ============================================================================
`timescale 1ns / 1ps

module tb_dvp_pixel_chain;

    reg clk = 0;
    always #5 clk = ~clk;

    reg  rst = 1;

    // ---------------- RGB565 chain ----------------
    reg  href = 0, vsync = 0;
    reg  [7:0] d = 0;

    wire byte_valid_r, line_start_r, frame_start_r;
    wire [7:0] byte_data_r;

    dvp_capture #(.HREF_POL(1'b1), .VSYNC_POL(1'b1)) cap_rgb (
        .pclk(clk), .rst_async(rst), .href(href), .vsync(vsync), .d(d),
        .byte_valid(byte_valid_r), .byte_data(byte_data_r),
        .line_start(line_start_r), .frame_start(frame_start_r)
    );

    wire pix_valid_r; wire [23:0] pix_rgb_r;
    pixel_formatter #(.FORMAT("RGB565"), .BYTE_SWAP(1'b0)) fmt_rgb (
        .pclk(clk), .rst(rst),
        .byte_valid(byte_valid_r), .byte_data(byte_data_r),
        .line_start(line_start_r), .frame_start(frame_start_r),
        .pixel_valid(pix_valid_r), .rgb(pix_rgb_r),
        .pixel_line_start(), .pixel_frame_start()
    );

    integer errors = 0;
    integer checks = 0;

    reg [15:0] exp_queue [0:15];
    integer exp_wr = 2; // two entries pre-loaded below
    integer exp_rd = 0;

    initial begin
        exp_queue[0] = 16'hF800;
        exp_queue[1] = 16'h07E0;
    end

    reg [4:0] r5; reg [5:0] g6; reg [4:0] b5;
    reg [7:0] exp_r, exp_g, exp_b;

    always @(posedge clk) begin
        if (pix_valid_r) begin
            r5 = exp_queue[exp_rd][15:11];
            g6 = exp_queue[exp_rd][10:5];
            b5 = exp_queue[exp_rd][4:0];
            exp_r = {r5, r5[4:2]};
            exp_g = {g6, g6[5:4]};
            exp_b = {b5, b5[4:2]};
            checks = checks + 1;
            if (pix_rgb_r !== {exp_r, exp_g, exp_b}) begin
                errors = errors + 1;
                $display("ERROR RGB565: got %06x expected %02x%02x%02x", pix_rgb_r, exp_r, exp_g, exp_b);
            end else begin
                $display("OK RGB565: pixel16=%04x -> rgb888=%06x", exp_queue[exp_rd], pix_rgb_r);
            end
            exp_rd = exp_rd + 1;
        end
    end

    // ---------------- YUYV422 chain ----------------
    reg  href2 = 0, vsync2 = 0;
    reg  [7:0] d2 = 0;
    wire byte_valid_y, line_start_y, frame_start_y;
    wire [7:0] byte_data_y;

    dvp_capture #(.HREF_POL(1'b1), .VSYNC_POL(1'b1)) cap_yuv (
        .pclk(clk), .rst_async(rst), .href(href2), .vsync(vsync2), .d(d2),
        .byte_valid(byte_valid_y), .byte_data(byte_data_y),
        .line_start(line_start_y), .frame_start(frame_start_y)
    );

    wire pix_valid_y; wire [23:0] pix_rgb_y;
    pixel_formatter #(.FORMAT("YUYV422")) fmt_yuv (
        .pclk(clk), .rst(rst),
        .byte_valid(byte_valid_y), .byte_data(byte_data_y),
        .line_start(line_start_y), .frame_start(frame_start_y),
        .pixel_valid(pix_valid_y), .rgb(pix_rgb_y),
        .pixel_line_start(), .pixel_frame_start()
    );

    integer y_checks = 0, y_errors = 0;
    always @(posedge clk) begin
        if (pix_valid_y) begin
            y_checks = y_checks + 1;
            // Group: Y0=128,U=128,Y1=200,V=128 -> neutral chroma (Cb=Cr=0),
            // so pixel0 = (128,128,128) and pixel1 = (200,200,200) exactly.
            if (y_checks == 1) begin
                if (pix_rgb_y !== {8'd128,8'd128,8'd128}) begin
                    y_errors = y_errors + 1;
                    $display("ERROR YUYV pixel0: got %06x expected 808080", pix_rgb_y);
                end else $display("OK YUYV pixel0: %06x", pix_rgb_y);
            end else if (y_checks == 2) begin
                if (pix_rgb_y !== {8'd200,8'd200,8'd200}) begin
                    y_errors = y_errors + 1;
                    $display("ERROR YUYV pixel1: got %06x expected c8c8c8", pix_rgb_y);
                end else $display("OK YUYV pixel1: %06x", pix_rgb_y);
            end
        end
    end

    initial begin
        $dumpfile("tb_dvp_pixel_chain.vcd");
        $dumpvars(0, tb_dvp_pixel_chain);

        rst = 1; href = 0; vsync = 0; d = 0;
        href2 = 0; vsync2 = 0; d2 = 0;
        repeat (3) @(posedge clk);
        rst = 0;
        @(posedge clk);

        // ---- RGB565 stimulus: VSYNC pulse, then one line of two pixels ----
        // Safe stimulus pattern: each @(posedge clk) waits for the edge that
        // samples the *currently held* value; the assignment right after it
        // sets up the value for the *next* edge, giving a full clock period
        // of setup margin (avoids same-edge testbench/DUT races).
        @(posedge clk); #1; vsync = 1;
        @(posedge clk); #1; vsync = 0; href = 1; d = 8'hF8; // high byte of 0xF800
        @(posedge clk); #1; d = 8'h00;                       // low byte -> emits 0xF800
        @(posedge clk); #1; d = 8'h07;                       // high byte of 0x07E0
        @(posedge clk); #1; d = 8'hE0;                       // low byte -> emits 0x07E0
        @(posedge clk); #1; href = 0;
        @(posedge clk);
        repeat (4) @(posedge clk);

        // ---- YUYV422 stimulus: VSYNC pulse, then one 2-pixel group -------
        @(posedge clk); #1; vsync2 = 1;
        @(posedge clk); #1; vsync2 = 0; href2 = 1; d2 = 8'd128; // Y0
        @(posedge clk); #1; d2 = 8'd128;                         // U
        @(posedge clk); #1; d2 = 8'd200;                         // Y1
        @(posedge clk); #1; d2 = 8'd128;                         // V -> emits pixel0
        @(posedge clk); #1; d2 = 8'hAA;                          // next Y0 -> emits pixel1
        @(posedge clk); #1; href2 = 0;
        @(posedge clk);
        repeat (4) @(posedge clk);

        if (errors == 0 && checks == 2 && y_errors == 0 && y_checks == 2)
            $display("TB_DVP_PIXEL_CHAIN: PASS (rgb565 checks=%0d, yuyv checks=%0d)", checks, y_checks);
        else
            $display("TB_DVP_PIXEL_CHAIN: FAIL (rgb565 checks=%0d errors=%0d, yuyv checks=%0d errors=%0d)",
                       checks, errors, y_checks, y_errors);

        $finish;
    end

endmodule
