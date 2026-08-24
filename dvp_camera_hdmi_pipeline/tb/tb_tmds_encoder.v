// ============================================================================
// tb_tmds_encoder.v -- self-checking testbench for tmds_encoder.v
// ----------------------------------------------------------------------------
// Strategy: drive the DUT with every possible 8-bit sample (256 values) many
// times over so the internal running-disparity accumulator visits a wide
// range of states, independently DECODE each 10-bit symbol the DUT produces
// (using the textbook inverse of the DVI 8b/10b algorithm, written fresh
// here rather than reusing any encoder code) and assert the recovered byte
// matches what was sent. Also checks the four fixed control-period codes and
// that the running disparity accumulator never leaves its legal +/-4 range.
// ============================================================================
`timescale 1ns / 1ps

module tb_tmds_encoder;

    reg        clk = 0;
    reg        rst = 1;
    reg  [7:0] din = 0;
    reg  [1:0] ctrl = 0;
    reg        de = 0;
    wire [9:0] tmds;

    integer errors = 0;
    integer checks = 0;

    tmds_encoder dut (
        .clk(clk), .rst(rst), .din(din), .ctrl(ctrl), .de(de), .tmds(tmds)
    );

    always #5 clk = ~clk; // 100MHz sim clock, frequency irrelevant here

    // ---- independent reference decoder (inverse of the DVI algorithm) ----
    function [7:0] tmds_decode;
        input [9:0] sym;
        reg          use_xnor;
        reg [7:0]    qm;
        integer      i;
        begin
            use_xnor = ~sym[8];
            qm = sym[9] ? ~sym[7:0] : sym[7:0];
            tmds_decode[0] = qm[0];
            for (i = 1; i <= 7; i = i + 1)
                tmds_decode[i] = use_xnor ? ~(qm[i] ^ qm[i-1]) : (qm[i] ^ qm[i-1]);
        end
    endfunction

    // tmds_encoder is now a 2-stage pipeline (see its header comment): din/
    // ctrl/de must be delayed by TWO clocks to align with the registered
    // tmds output, not one.
    reg [7:0] din_d1, din_d;
    reg       de_d1, de_d;
    reg [1:0] ctrl_d1, ctrl_d;

    always @(posedge clk) begin
        din_d1  <= din;   de_d1  <= de;   ctrl_d1 <= ctrl;
        din_d   <= din_d1; de_d  <= de_d1; ctrl_d  <= ctrl_d1;
    end

    reg [7:0] expected_ctrl_code_hi; // unused placeholder to keep style consistent

    task check_disparity_bounds;
        begin
            // balance_acc is internal; access via hierarchical reference for
            // simulation-only white-box sanity checking.
            if (dut.balance_acc > 4 || dut.balance_acc < -4) begin
                $display("[%0t] ERROR: balance_acc out of range: %0d", $time, dut.balance_acc);
                errors = errors + 1;
            end
        end
    endtask

    integer rep;
    integer value; // 0..255 loop counter (wider than din to avoid 8-bit wraparound)

    initial begin
        $dumpfile("tb_tmds_encoder.vcd");
        $dumpvars(0, tb_tmds_encoder);

        // reset
        de = 0; ctrl = 2'b00; din = 8'h00;
        repeat (3) @(posedge clk);
        rst = 0;
        @(posedge clk);

        // ---- 1) control-code check: all four combinations -----------------
        de = 0;
        for (rep = 0; rep < 4; rep = rep + 1) begin
            ctrl = rep[1:0];
            @(posedge clk); // 2-stage pipeline: hold ctrl steady across both stages
            @(posedge clk);
            #1;
            case (ctrl)
                2'b00: if (tmds !== 10'b1101010100) begin errors=errors+1; $display("ERROR ctrl00 got %b", tmds); end
                2'b01: if (tmds !== 10'b0010101011) begin errors=errors+1; $display("ERROR ctrl01 got %b", tmds); end
                2'b10: if (tmds !== 10'b0101010100) begin errors=errors+1; $display("ERROR ctrl10 got %b", tmds); end
                2'b11: if (tmds !== 10'b1010101011) begin errors=errors+1; $display("ERROR ctrl11 got %b", tmds); end
            endcase
            checks = checks + 1;
        end

        // ---- 2) exhaustive round-trip check across many disparity states --
        de = 1;
        ctrl = 2'b00;
        for (rep = 0; rep < 6; rep = rep + 1) begin // 6 full sweeps => varied accumulator history
            for (value = 0; value < 256; value = value + 1) begin
                din = value[7:0];
                @(posedge clk);
                #1; // let registered tmds settle (already combinational->registered)
                check_disparity_bounds;
                if (de_d) begin
                    if (tmds_decode(tmds) !== din_d) begin
                        errors = errors + 1;
                        $display("[%0t] ERROR: sent 8'h%02x, decoded 8'h%02x from tmds=%b",
                                  $time, din_d, tmds_decode(tmds), tmds);
                    end
                    checks = checks + 1;
                end
            end
        end

        // one extra cycle to flush the last decode
        @(posedge clk); #1;
        if (tmds_decode(tmds) !== din_d) begin
            errors = errors + 1;
            $display("[%0t] ERROR (flush): sent 8'h%02x, decoded 8'h%02x", $time, din_d, tmds_decode(tmds));
        end
        checks = checks + 1;

        if (errors == 0)
            $display("TB_TMDS_ENCODER: PASS (%0d checks, 0 errors)", checks);
        else
            $display("TB_TMDS_ENCODER: FAIL (%0d checks, %0d errors)", checks, errors);

        $finish;
    end

endmodule
