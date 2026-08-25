// ============================================================================
// tb_cam_power_sequencer.v -- checks power-up sequencing order and timing
// ----------------------------------------------------------------------------
// Uses small overridden RST_MS/SETTLE_MS-equivalent cycle counts (via a
// low CLK_FREQ_HZ) so this runs in a reasonable number of simulated cycles
// while exercising the exact same state machine as the real 24MHz config.
// ============================================================================
`timescale 1ns / 1ps

module tb_cam_power_sequencer;

    reg clk = 0;
    always #5 clk = ~clk; // 100MHz sim clock, frequency irrelevant to the logic

    reg rst_async = 1;
    reg mclk_locked = 0;
    wire cam_pwdn, cam_rst_n, seq_done;

    // CLK_FREQ_HZ=1000 with RST_MS=2/SETTLE_MS=20 -> RST_CYCLES=2, SETTLE_CYCLES=20
    // (tiny counts, fast to simulate, same RTL path as the real 24MHz config)
    cam_power_sequencer #(
        .CLK_FREQ_HZ(1000), .RST_MS(2), .SETTLE_MS(20)
    ) dut (
        .clk(clk), .mclk_locked(mclk_locked), .rst_async(rst_async),
        .cam_pwdn(cam_pwdn), .cam_rst_n(cam_rst_n), .seq_done(seq_done)
    );

    integer errors = 0;
    integer t_mclk_locked, t_pwdn_low, t_rstn_high, t_seq_done;

    initial begin
        $dumpfile("tb_cam_power_sequencer.vcd");
        $dumpvars(0, tb_cam_power_sequencer);

        // Before reset deasserts / mclk locks: PWDN must be high (powered
        // down), RESET must be low (asserted), seq_done must be low.
        repeat (3) @(posedge clk);
        if (cam_pwdn !== 1'b1) begin errors = errors + 1; $display("ERROR: cam_pwdn not high pre-lock"); end
        if (cam_rst_n !== 1'b0) begin errors = errors + 1; $display("ERROR: cam_rst_n not low pre-lock"); end
        if (seq_done  !== 1'b0) begin errors = errors + 1; $display("ERROR: seq_done asserted too early"); end

        rst_async = 0;
        repeat (5) @(posedge clk);

        // Still no mclk_locked -- must still be held in the power-down/reset state.
        if (cam_pwdn !== 1'b1 || cam_rst_n !== 1'b0 || seq_done !== 1'b0) begin
            errors = errors + 1;
            $display("ERROR: state changed before mclk_locked was asserted");
        end

        mclk_locked = 1;
        t_mclk_locked = $time;

        // Wait for PWDN to go low (power up)
        wait (cam_pwdn === 1'b0);
        t_pwdn_low = $time;
        if (cam_rst_n !== 1'b0) begin
            errors = errors + 1;
            $display("ERROR: cam_rst_n released before/at the same time as PWDN going low");
        end

        // Wait for RESET to release
        wait (cam_rst_n === 1'b1);
        t_rstn_high = $time;
        if (seq_done !== 1'b0) begin
            errors = errors + 1;
            $display("ERROR: seq_done asserted at/before reset release (no settle time)");
        end
        if (t_rstn_high <= t_pwdn_low) begin
            errors = errors + 1;
            $display("ERROR: reset released no later than PWDN going low (RST_CYCLES not honoured)");
        end

        // Wait for the sequencer to finish
        wait (seq_done === 1'b1);
        t_seq_done = $time;
        if (t_seq_done <= t_rstn_high) begin
            errors = errors + 1;
            $display("ERROR: seq_done asserted no later than reset release (SETTLE_CYCLES not honoured)");
        end

        // seq_done must stay asserted (sticky)
        repeat (10) @(posedge clk);
        if (seq_done !== 1'b1) begin
            errors = errors + 1;
            $display("ERROR: seq_done did not stay asserted");
        end

        $display("Timing: mclk_locked@%0t pwdn_low@%0t rst_n_high@%0t seq_done@%0t",
                   t_mclk_locked, t_pwdn_low, t_rstn_high, t_seq_done);

        if (errors == 0)
            $display("TB_CAM_POWER_SEQUENCER: PASS");
        else
            $display("TB_CAM_POWER_SEQUENCER: FAIL (%0d errors)", errors);

        $finish;
    end

    initial begin
        #100000;
        $display("TB_CAM_POWER_SEQUENCER: WATCHDOG TIMEOUT");
        $finish;
    end

endmodule
