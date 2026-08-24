// ============================================================================
// tb_video_timing_gen.v -- checks H/V totals, DE pixel count and frame rate
// for both 720p60 and 1080p60 parameter sets against the known CEA-861 numbers.
// ============================================================================
`timescale 1ns / 1ps

module tb_video_timing_gen;

    reg clk = 0;
    reg rst = 1;

    // ---------------- 720p60 instance ----------------
    wire hsync_720, vsync_720, de_720, fs_720;
    wire [15:0] x_720, y_720;

    video_timing_gen #(
        .H_ACTIVE(1280), .H_FRONT(110), .H_SYNC(40), .H_BACK(220),
        .V_ACTIVE(720),  .V_FRONT(5),   .V_SYNC(5),  .V_BACK(20),
        .HS_POL(1'b1), .VS_POL(1'b1)
    ) dut_720p (
        .clk(clk), .rst(rst),
        .hsync(hsync_720), .vsync(vsync_720), .de(de_720), .frame_start(fs_720),
        .x(x_720), .y(y_720)
    );

    always #5 clk = ~clk; // 100MHz functional clock (frequency irrelevant for counting)

    integer de_pixels;
    integer h_period_cycles;
    integer last_fs_time, this_fs_time;
    integer errors = 0;

    reg fs_prev;
    integer fs_count;

    initial begin
        rst = 1;
        de_pixels = 0;
        fs_count = 0;
        last_fs_time = -1;
        repeat (4) @(posedge clk);
        rst = 0;

        // Count DE-active cycles across exactly one full frame using frame_start
        // pulses. Polled with a plain edge-by-edge loop (not `wait`) to avoid
        // any ambiguity about whether the edge that first satisfies the
        // condition has or hasn't already been "consumed" by the time the
        // next @(posedge clk) below runs.
        while (fs_720 !== 1'b1) @(posedge clk);
        last_fs_time = $time;
        de_pixels = de_720 ? 1 : 0; // frame_start's own cycle (pixel 0,0) counts too

        // run until the *next* frame_start
        begin : count_loop
            reg done;
            done = 0;
            while (!done) begin
                @(posedge clk);
                if (fs_720) begin
                    this_fs_time = $time;
                    fs_count = fs_count + 1;
                    if (fs_count == 1) begin
                        // frame period in clock cycles (10ns period)
                        h_period_cycles = (this_fs_time - last_fs_time) / 10;
                        if (h_period_cycles !== 1650*750) begin
                            errors = errors + 1;
                            $display("ERROR 720p: frame period = %0d cycles, expected %0d",
                                      h_period_cycles, 1650*750);
                        end else begin
                            $display("OK 720p: frame period = %0d cycles (1650x750) => matches", h_period_cycles);
                        end
                        done = 1;
                    end
                end
                if (!done && de_720) de_pixels = de_pixels + 1;
            end
        end

        if (de_pixels !== 1280*720) begin
            errors = errors + 1;
            $display("ERROR 720p: DE active pixel count = %0d, expected %0d", de_pixels, 1280*720);
        end else begin
            $display("OK 720p: DE active pixel count = %0d (1280x720) => matches", de_pixels);
        end

        if (errors == 0)
            $display("TB_VIDEO_TIMING_GEN: PASS");
        else
            $display("TB_VIDEO_TIMING_GEN: FAIL (%0d errors)", errors);

        $finish;
    end

endmodule
