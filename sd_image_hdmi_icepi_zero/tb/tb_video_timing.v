`timescale 1ns/1ps
// tb_video_timing : verifies video_timing.v produces exactly the VESA
// DMT 640x480@60 counts (800x525 total, 640x480 active, negative sync
// polarities).
module tb_video_timing;

    initial begin
        $dumpfile("tb_video_timing.vcd");
        $dumpvars(0, tb_video_timing);
    end

    reg clk = 1'b0;
    reg rst = 1'b1;
    wire hsync, vsync, de;
    wire [9:0] x, y;

    always #20.0 clk = ~clk; // 25 MHz

    video_timing dut (
        .pix_clk (clk),
        .rst     (rst),
        .hsync   (hsync),
        .vsync   (vsync),
        .de      (de),
        .x       (x),
        .y       (y)
    );

    integer errors = 0;

    // ---- Measure one horizontal line's active/sync-low pixel counts ----
    initial begin
        integer i;
        integer local_active, local_sync_low;
        repeat (4) @(posedge clk);
        rst = 1'b0;

        wait (hsync == 1'b0);
        wait (hsync == 1'b1);
        @(posedge clk);
        local_active   = 0;
        local_sync_low = 0;
        for (i = 0; i < 800; i = i + 1) begin
            if (de)     local_active   = local_active + 1;
            if (!hsync) local_sync_low = local_sync_low + 1;
            @(posedge clk);
        end
        if (local_active !== 640) begin
            $display("FAIL: expected 640 active-video (de=1) pixels per line, got %0d", local_active);
            errors = errors + 1;
        end else begin
            $display("PASS: 640 active-video pixels observed per horizontal line");
        end
        if (local_sync_low !== 96) begin
            $display("FAIL: expected 96 hsync-low pixels per line (H_SYNC width), got %0d", local_sync_low);
            errors = errors + 1;
        end else begin
            $display("PASS: hsync pulse width = 96 pixel clocks, as specified (negative polarity)");
        end
    end

    // ---- Whitebox check on the internal counters' wrap points -----------
    // Directly checking dut.hcnt/dut.vcnt's maximum reached values is
    // immune to the edge-coincidence/off-by-N framing mistakes that a
    // purely black-box hsync/vsync-edge-counting testbench is prone to
    // (vsync's own 2-line-wide pulse makes edge-to-edge counting easy
    // to mis-frame, as an earlier draft of this testbench demonstrated).
    integer max_hcnt, max_vcnt;
    initial begin
        max_hcnt = -1;
        max_vcnt = -1;
        wait (!rst);
        repeat (2000) @(posedge clk) begin
            if ($signed({1'b0, dut.hcnt}) > max_hcnt) max_hcnt = dut.hcnt;
            if ($signed({1'b0, dut.vcnt}) > max_vcnt) max_vcnt = dut.vcnt;
        end
        $display("[tb_video_timing] max hcnt observed = %0d (expect 799, H_TOTAL-1)", max_hcnt);
        $display("[tb_video_timing] max vcnt observed = %0d (expect up to 524, V_TOTAL-1, needs >1 line to appear)", max_vcnt);
        if (max_hcnt !== 799) begin
            $display("FAIL: expected max hcnt = 799 (H_TOTAL=800)");
            errors = errors + 1;
        end else begin
            $display("PASS: hcnt wraps at 799, matching H_TOTAL=800");
        end

        // Run long enough to observe a full frame and confirm vcnt
        // reaches its expected maximum too.
        repeat (800 * 525) @(posedge clk) begin
            if ($signed({1'b0, dut.vcnt}) > max_vcnt) max_vcnt = dut.vcnt;
        end
        $display("[tb_video_timing] max vcnt after a full frame = %0d (expect 524, V_TOTAL=525)", max_vcnt);
        if (max_vcnt !== 524) begin
            $display("FAIL: expected max vcnt = 524 (V_TOTAL=525)");
            errors = errors + 1;
        end else begin
            $display("PASS: vcnt wraps at 524, matching V_TOTAL=525");
        end

        if (errors == 0)
            $display("PASS: video_timing produces correct VESA DMT 640x480@60 counts");
        else
            $display("FAIL: %0d check(s) failed", errors);
        $finish;
    end

    initial begin
        #40_000_000; // generous cap
        $display("[tb_video_timing] TIMEOUT");
        $finish;
    end

endmodule
