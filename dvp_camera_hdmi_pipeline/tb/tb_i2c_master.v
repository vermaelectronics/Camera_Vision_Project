// ============================================================================
// tb_i2c_master.v -- smoke test for i2c_master.v against a minimal behavioral
// I2C slave model that always ACKs and records the bytes it receives.
// Tests BOTH addressing modes: ADDR_BYTES=1 (8-bit reg addressing, e.g.
// OV7670/OV2640-class) and ADDR_BYTES=2 (16-bit reg addressing, e.g.
// OV5640/OV5647-class -- what the Waveshare OV5640 module needs).
// ============================================================================
`timescale 1ns / 1ps

module tb_i2c_master;

    reg clk = 0;
    always #10 clk = ~clk; // 50MHz

    reg rst = 1;
    integer errors = 0;

    // =========================================================================
    // Instance A: ADDR_BYTES=1 (8-bit register addressing)
    // =========================================================================
    reg  start_a = 0;
    reg  [15:0] reg_addr_a = 16'h00AA;
    reg  [7:0]  reg_data_a = 8'h55;
    wire busy_a, done_a, nack_error_a;
    wire scl_a, sda_a;
    reg  sda_slave_drive_a = 1'b0;
    assign sda_a = sda_slave_drive_a ? 1'b0 : 1'bz;
    pullup(scl_a);
    pullup(sda_a);

    i2c_master #(
        .CLK_FREQ_HZ(50_000_000), .I2C_FREQ_HZ(400_000),
        .DEV_ADDR7(7'h21), .ADDR_BYTES(1)
    ) dut_a (
        .clk(clk), .rst(rst),
        .start(start_a), .reg_addr(reg_addr_a), .reg_data(reg_data_a),
        .busy(busy_a), .done(done_a), .nack_error(nack_error_a),
        .scl(scl_a), .sda(sda_a)
    );

    reg [7:0] captured_a [0:2];
    integer   cap_idx_a = 0;
    reg [7:0] shift_in_a;
    integer   bit_cnt_a;
    reg       in_byte_a;
    reg       scl_prev_a, sda_prev_a, scl_d_a;

    always @(posedge clk) begin
        scl_prev_a <= scl_a;
        sda_prev_a <= sda_a;
        scl_d_a    <= scl_a;
    end
    wire start_cond_a = scl_a && sda_prev_a && !sda_a && !sda_slave_drive_a;
    wire stop_cond_a  = scl_a && !sda_prev_a && sda_a && !sda_slave_drive_a;
    wire scl_rising_a  = scl_a && !scl_d_a;
    wire scl_falling_a = !scl_a && scl_d_a;

    always @(posedge clk) begin
        if (start_cond_a) begin
            in_byte_a <= 1; bit_cnt_a <= 0; sda_slave_drive_a <= 0;
        end else if (stop_cond_a) begin
            in_byte_a <= 0; sda_slave_drive_a <= 0;
        end
    end
    always @(posedge clk) begin
        if (scl_rising_a && in_byte_a) begin
            if (bit_cnt_a < 8) begin
                shift_in_a <= {shift_in_a[6:0], sda_a};
                bit_cnt_a  <= bit_cnt_a + 1;
            end else begin
                if (cap_idx_a < 3) captured_a[cap_idx_a] <= shift_in_a;
                cap_idx_a <= cap_idx_a + 1;
                sda_slave_drive_a <= 1'b1;
                bit_cnt_a <= 0;
            end
        end
    end
    always @(posedge clk) if (scl_falling_a) sda_slave_drive_a <= 1'b0;

    // =========================================================================
    // Instance B: ADDR_BYTES=2 (16-bit register addressing -- OV5640-class)
    // =========================================================================
    reg  start_b = 0;
    reg  [15:0] reg_addr_b = 16'h3103; // a real OV5640 register address, for realism
    reg  [7:0]  reg_data_b = 8'h11;
    wire busy_b, done_b, nack_error_b;
    wire scl_b, sda_b;
    reg  sda_slave_drive_b = 1'b0;
    assign sda_b = sda_slave_drive_b ? 1'b0 : 1'bz;
    pullup(scl_b);
    pullup(sda_b);

    i2c_master #(
        .CLK_FREQ_HZ(50_000_000), .I2C_FREQ_HZ(400_000),
        .DEV_ADDR7(7'h3C), .ADDR_BYTES(2)
    ) dut_b (
        .clk(clk), .rst(rst),
        .start(start_b), .reg_addr(reg_addr_b), .reg_data(reg_data_b),
        .busy(busy_b), .done(done_b), .nack_error(nack_error_b),
        .scl(scl_b), .sda(sda_b)
    );

    reg [7:0] captured_b [0:3];
    integer   cap_idx_b = 0;
    reg [7:0] shift_in_b;
    integer   bit_cnt_b;
    reg       in_byte_b;
    reg       scl_prev_b, sda_prev_b, scl_d_b;

    always @(posedge clk) begin
        scl_prev_b <= scl_b;
        sda_prev_b <= sda_b;
        scl_d_b    <= scl_b;
    end
    wire start_cond_b = scl_b && sda_prev_b && !sda_b && !sda_slave_drive_b;
    wire stop_cond_b  = scl_b && !sda_prev_b && sda_b && !sda_slave_drive_b;
    wire scl_rising_b  = scl_b && !scl_d_b;
    wire scl_falling_b = !scl_b && scl_d_b;

    always @(posedge clk) begin
        if (start_cond_b) begin
            in_byte_b <= 1; bit_cnt_b <= 0; sda_slave_drive_b <= 0;
        end else if (stop_cond_b) begin
            in_byte_b <= 0; sda_slave_drive_b <= 0;
        end
    end
    always @(posedge clk) begin
        if (scl_rising_b && in_byte_b) begin
            if (bit_cnt_b < 8) begin
                shift_in_b <= {shift_in_b[6:0], sda_b};
                bit_cnt_b  <= bit_cnt_b + 1;
            end else begin
                if (cap_idx_b < 4) captured_b[cap_idx_b] <= shift_in_b;
                cap_idx_b <= cap_idx_b + 1;
                sda_slave_drive_b <= 1'b1;
                bit_cnt_b <= 0;
            end
        end
    end
    always @(posedge clk) if (scl_falling_b) sda_slave_drive_b <= 1'b0;

    // =========================================================================
    initial begin
        $dumpfile("tb_i2c_master.vcd");
        $dumpvars(0, tb_i2c_master);

        repeat (5) @(posedge clk);
        rst = 0;
        repeat (5) @(posedge clk);

        // ---- Test A: 8-bit addressing (3-byte transaction) --------------------
        start_a = 1;
        @(posedge clk); #1; start_a = 0;
        wait (done_a === 1'b1);
        repeat (2) @(posedge clk);

        if (nack_error_a) begin
            errors = errors + 1;
            $display("ERROR (A): nack_error asserted, expected clean ACK'd transaction");
        end
        if (cap_idx_a < 3) begin
            errors = errors + 1;
            $display("ERROR (A): slave only captured %0d of 3 expected bytes", cap_idx_a);
        end else begin
            if (captured_a[0] !== {7'h21, 1'b0}) begin
                errors = errors + 1;
                $display("ERROR (A): byte0 (addr+W) = %02x, expected %02x", captured_a[0], {7'h21,1'b0});
            end
            if (captured_a[1] !== reg_addr_a[7:0]) begin
                errors = errors + 1;
                $display("ERROR (A): byte1 (reg_addr) = %02x, expected %02x", captured_a[1], reg_addr_a[7:0]);
            end
            if (captured_a[2] !== reg_data_a) begin
                errors = errors + 1;
                $display("ERROR (A): byte2 (reg_data) = %02x, expected %02x", captured_a[2], reg_data_a);
            end
        end
        if (errors == 0)
            $display("OK (ADDR_BYTES=1): addr+W=%02x reg_addr=%02x reg_data=%02x, all ACKed",
                       captured_a[0], captured_a[1], captured_a[2]);

        // ---- Test B: 16-bit addressing (4-byte transaction) -------------------
        start_b = 1;
        @(posedge clk); #1; start_b = 0;
        wait (done_b === 1'b1);
        repeat (2) @(posedge clk);

        if (nack_error_b) begin
            errors = errors + 1;
            $display("ERROR (B): nack_error asserted, expected clean ACK'd transaction");
        end
        if (cap_idx_b < 4) begin
            errors = errors + 1;
            $display("ERROR (B): slave only captured %0d of 4 expected bytes", cap_idx_b);
        end else begin
            if (captured_b[0] !== {7'h3C, 1'b0}) begin
                errors = errors + 1;
                $display("ERROR (B): byte0 (addr+W) = %02x, expected %02x", captured_b[0], {7'h3C,1'b0});
            end
            if (captured_b[1] !== reg_addr_b[15:8]) begin
                errors = errors + 1;
                $display("ERROR (B): byte1 (reg_addr hi) = %02x, expected %02x", captured_b[1], reg_addr_b[15:8]);
            end
            if (captured_b[2] !== reg_addr_b[7:0]) begin
                errors = errors + 1;
                $display("ERROR (B): byte2 (reg_addr lo) = %02x, expected %02x", captured_b[2], reg_addr_b[7:0]);
            end
            if (captured_b[3] !== reg_data_b) begin
                errors = errors + 1;
                $display("ERROR (B): byte3 (reg_data) = %02x, expected %02x", captured_b[3], reg_data_b);
            end
        end
        if (errors == 0)
            $display("OK (ADDR_BYTES=2): addr+W=%02x reg_addr=%02x%02x reg_data=%02x, all ACKed",
                       captured_b[0], captured_b[1], captured_b[2], captured_b[3]);

        if (errors == 0)
            $display("TB_I2C_MASTER: PASS");
        else
            $display("TB_I2C_MASTER: FAIL (%0d errors)", errors);

        $finish;
    end

    // watchdog
    initial begin
        #600000;
        $display("TB_I2C_MASTER: WATCHDOG TIMEOUT");
        $finish;
    end

endmodule
