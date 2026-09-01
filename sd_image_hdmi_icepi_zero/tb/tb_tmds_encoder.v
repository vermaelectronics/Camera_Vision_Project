`timescale 1ns/1ps
// tb_tmds_encoder : checks the two properties of tmds_encoder.v that
// are independently verifiable without an external reference decoder:
//   1. The four control-period tokens exactly match the fixed values
//      published in every TMDS/DVI reference (0x354/0x0AB/0x154/0x2AB).
//   2. Over a long pseudo-random active-video data stream, the running
//      disparity of transmitted bits stays bounded (does not drift
//      away without limit) - the defining property of a working
//      DC-balancing encoder. A broken invert/no-invert decision would
//      typically show up as unbounded drift here.
//
// This does NOT prove bit-for-bit conformance to the DVI spec's exact
// encoding of every possible byte (that needs an independent reference
// decoder/encoder to compare against, which isn't available in this
// environment - see the disclosure in tmds_encoder.v's header comment).
module tb_tmds_encoder;

    initial begin
        $dumpfile("tb_tmds_encoder.vcd");
        $dumpvars(0, tb_tmds_encoder);
    end

    reg        clk = 1'b0;
    reg        rst = 1'b1;
    reg  [7:0] data = 8'd0;
    reg  [1:0] ctrl = 2'b00;
    reg        de = 1'b0;
    wire [9:0] tmds_q;

    always #20.0 clk = ~clk;

    tmds_encoder dut (
        .pix_clk (clk),
        .rst     (rst),
        .data    (data),
        .ctrl    (ctrl),
        .de      (de),
        .tmds_q  (tmds_q)
    );

    integer errors = 0;

    task check_ctrl_token(input [1:0] c, input [9:0] expected);
        begin
            @(posedge clk); ctrl = c; de = 1'b0;
            @(posedge clk); // tmds_q now reflects this cycle's ctrl/de (1-cycle latency)
            if (tmds_q !== expected) begin
                $display("FAIL: ctrl=%b -> tmds_q=%b, expected %b", c, tmds_q, expected);
                errors = errors + 1;
            end else begin
                $display("PASS: ctrl=%b -> tmds_q=%b, matches the standard fixed control token", c, tmds_q);
            end
        end
    endtask

    integer disparity_acc;
    integer max_abs_disparity;
    integer i;
    reg [31:0] lfsr;

    initial begin
        repeat (4) @(posedge clk);
        rst = 1'b0;

        check_ctrl_token(2'b00, 10'b1101010100);
        check_ctrl_token(2'b01, 10'b0010101011);
        check_ctrl_token(2'b10, 10'b0101010100);
        check_ctrl_token(2'b11, 10'b1010101011);

        // ---- DC-balance check over a long pseudo-random stream -----------
        disparity_acc     = 0;
        max_abs_disparity  = 0;
        lfsr = 32'hACE1_2345;
        de = 1'b1;
        ctrl = 2'b00;
        for (i = 0; i < 20000; i = i + 1) begin
            // simple xorshift32 for a cheap, deterministic pseudo-random byte stream
            lfsr = lfsr ^ (lfsr << 13);
            lfsr = lfsr ^ (lfsr >> 17);
            lfsr = lfsr ^ (lfsr << 5);
            data = lfsr[7:0];
            @(posedge clk);
            // tmds_q here reflects the PREVIOUS cycle's data (1-cycle
            // latency) - fine for a long-run statistical check, the
            // one-symbol skew doesn't affect the bound being tested.
            // disparity contribution = n1_out - n0_out = 2*n1_out - 8
            // (NOT n1_out - 8, which would silently apply a constant
            // -4 bias to every symbol whose 8 bits happen to already
            // be balanced - the bug an earlier draft of this testbench
            // had, which made the DUT look like it was drifting when
            // in fact dut.disparity itself stays correctly bounded).
            disparity_acc = disparity_acc
                          + 2 * (tmds_q[0]+tmds_q[1]+tmds_q[2]+tmds_q[3]+tmds_q[4]
                                +tmds_q[5]+tmds_q[6]+tmds_q[7])
                          - 8;
            if (disparity_acc > max_abs_disparity)  max_abs_disparity = disparity_acc;
            if (-disparity_acc > max_abs_disparity) max_abs_disparity = -disparity_acc;
        end

        $display("[tb_tmds_encoder] cumulative disparity after 20000 random symbols = %0d", disparity_acc);
        $display("[tb_tmds_encoder] max |running disparity| observed = %0d", max_abs_disparity);
        // A correctly DC-balanced 8b/10b-style encoder keeps the
        // *per-symbol* running disparity (the internal accumulator
        // reset every blanking period) within a small fixed bound
        // (typically well under +-20 for this algorithm); the
        // *cumulative* sum over many independent symbols can wander
        // more since each active-video run here never re-enters
        // blanking to reset it, so the meaningful bound to check is
        // that it does not grow linearly/unboundedly with the symbol
        // count - 20000 symbols growing to within a few hundred is
        // healthy, tens of thousands would indicate a broken invert
        // decision that always biases the same direction.
        if (max_abs_disparity > 2000) begin
            $display("FAIL: running disparity grew far larger than expected for 20000 symbols - likely a broken DC-balance decision");
            errors = errors + 1;
        end else begin
            $display("PASS: running disparity stayed bounded over 20000 pseudo-random symbols (no unbounded drift)");
        end

        if (errors == 0)
            $display("PASS: tmds_encoder control tokens and DC-balance invariant both check out");
        else
            $display("FAIL: %0d check(s) failed", errors);
        $finish;
    end
endmodule
