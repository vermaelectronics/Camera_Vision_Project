`timescale 1ns/1ps
// Standalone check: pi_reciprocal against real division, before it's
// trusted inside pi_power_inversion_normalized.v.
module tb_pi_reciprocal;
    localparam integer D_W    = 38;
    localparam integer Q_FRAC = 32;

    reg clk = 0; always #5 clk = ~clk;
    reg aresetn = 0;
    reg d_valid;
    reg [D_W-1:0] d_in;
    wire q_valid;
    wire [Q_FRAC:0] q_out;

    pi_reciprocal #(.D_W(D_W), .Q_FRAC(Q_FRAC)) dut (
        .clk(clk), .aresetn(aresetn),
        .d_valid(d_valid), .d_in(d_in),
        .q_valid(q_valid), .q_out(q_out)
    );

    // Track which D value was submitted each cycle so it's still known
    // Q_FRAC+1 cycles later when the matching q_out arrives.
    reg [D_W-1:0] d_hist [0:2*Q_FRAC+8];
    integer wp = 0, rp = 0;
    integer errors = 0;
    integer checked = 0;
    // Exact reference in 64-bit integer arithmetic -- NOT real/$itor. Q_FRAC
    // itself is 32, so q_out is 33 bits wide; $itor silently mishandles
    // anything past the traditional 32-bit integer boundary (confirmed: it
    // printed D=2's correct q_out=2147483648 back as -2147483648, exactly
    // 2^31 reinterpreted as a signed 32-bit value). Verilog's / operator on
    // plain regs does exact unsigned integer division with no such limit,
    // so comparing q_out against a 64-bit-computed expected value bit for
    // bit is both correct AND a strictly tighter check than any float
    // tolerance could be.
    reg [63:0] expected_q;

    always @(posedge clk) begin
        if (d_valid) begin
            d_hist[wp] = d_in;
            wp = wp + 1;
        end
        if (q_valid) begin
            expected_q = (64'd1 << Q_FRAC) / {32'd0, d_hist[rp]};   // floor(2^Q_FRAC / D), exact
            if (expected_q[Q_FRAC:0] !== q_out) begin
                $display("FAIL: D=%0d expected=%0d got=%0d",
                          d_hist[rp], expected_q[Q_FRAC:0], q_out);
                errors = errors + 1;
            end
            checked = checked + 1;
            rp = rp + 1;
        end
    end

    integer k;
    initial begin
        d_valid = 0; d_in = 0;
        repeat (5) @(posedge clk);
        aresetn = 1;
        repeat (5) @(posedge clk);

        $display("=========================================================");
        $display(" pi_reciprocal -- standalone check vs real division");
        $display("=========================================================");

        // Sweep: powers of two, small values near GAMMA_INIT=1, and a
        // realistic spread up toward this module's actual D_W-38 max, all
        // submitted BACK TO BACK (one per cycle) to prove the pipeline is
        // genuinely 1/cycle throughput, not secretly serialising.
        @(posedge clk);
        for (k = 0; k < 40; k = k + 1) begin
            d_valid <= 1'b1;
            case (k % 8)
                0: d_in <= 1;
                1: d_in <= 2;
                2: d_in <= 7;
                3: d_in <= 1000;
                4: d_in <= 32'd1_000_000;
                5: d_in <= 32'd268_435_456;      // 2^28
                6: d_in <= {D_W{1'b1}} >> 2;     // near max
                7: d_in <= 3 + k;                // walking value
            endcase
            @(posedge clk);
        end
        d_valid <= 1'b0;

        // let the pipeline drain (Q_FRAC+1 cycles plus margin)
        repeat (Q_FRAC + 20) @(posedge clk);

        $display("=========================================================");
        $display(" checked=%0d errors=%0d", checked, errors);
        if (checked < 35) begin
            $display(" FAIL: fewer results drained than submitted -- pipeline is not 1/cycle throughput");
            errors = errors + 1;
        end
        if (errors == 0) $display(" ALL CHECKS PASSED");
        else              $display(" %0d CHECK(S) FAILED", errors);
        $display("=========================================================");
        $finish;
    end
endmodule
