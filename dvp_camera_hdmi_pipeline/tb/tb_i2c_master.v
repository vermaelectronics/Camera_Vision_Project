// ============================================================================
// tb_i2c_master.v -- smoke test for i2c_master.v against a minimal behavioral
// I2C slave model that always ACKs and records the bytes it receives.
// ============================================================================
`timescale 1ns / 1ps

module tb_i2c_master;

    reg clk = 0;
    always #10 clk = ~clk; // 50MHz

    reg rst = 1;
    reg start = 0;
    reg [7:0] reg_addr = 8'hAA;
    reg [7:0] reg_data = 8'h55;
    wire busy, done, nack_error;

    wire scl, sda;
    reg sda_slave_drive = 1'b0;
    reg sda_slave_out   = 1'b0;

    // slave drives SDA low only to ACK; otherwise releases (open-drain model)
    assign sda = sda_slave_drive ? 1'b0 : 1'bz;
    pullup(scl);
    pullup(sda);

    i2c_master #(
        .CLK_FREQ_HZ(50_000_000),
        .I2C_FREQ_HZ(400_000),
        .DEV_ADDR7(7'h21)
    ) dut (
        .clk(clk), .rst(rst),
        .start(start), .reg_addr(reg_addr), .reg_data(reg_data),
        .busy(busy), .done(done), .nack_error(nack_error),
        .scl(scl), .sda(sda)
    );

    // ---- minimal behavioral I2C slave: always ACK, capture bytes ----------
    reg [7:0] captured [0:2];
    integer   cap_idx = 0;
    reg [7:0] shift_in;
    integer   bit_cnt;
    reg       in_byte;

    // Detect START (SDA falls while SCL high) and STOP (SDA rises while SCL high)
    reg scl_prev, sda_prev;
    always @(posedge clk) begin
        scl_prev <= scl;
        sda_prev <= sda;
    end
    // Exclude transitions caused by the slave's own ACK drive -- an SDA fall
    // while SCL is high looks identical to a repeated START unless the
    // detector knows it was the slave itself pulling the line low.
    wire start_cond = scl && sda_prev && !sda && !sda_slave_drive;
    wire stop_cond  = scl && !sda_prev && sda && !sda_slave_drive;

    initial begin
        cap_idx = 0;
        in_byte = 0;
        bit_cnt = 0;
    end

    always @(posedge clk) begin
        if (start_cond) begin
            in_byte <= 1;
            bit_cnt <= 0;
            sda_slave_drive <= 0;
        end else if (stop_cond) begin
            in_byte <= 0;
            sda_slave_drive <= 0;
        end
    end

    // Sample SDA on SCL rising edges while receiving a byte (simple level-based model)
    reg scl_d;
    always @(posedge clk) scl_d <= scl;
    wire scl_rising = scl && !scl_d;

    always @(posedge clk) begin
        if (scl_rising && in_byte) begin
            if (bit_cnt < 8) begin
                shift_in <= {shift_in[6:0], sda};
                bit_cnt  <= bit_cnt + 1;
            end else begin
                // 9th clock = ACK slot: drive SDA low to ACK
                if (cap_idx < 3) captured[cap_idx] <= shift_in;
                cap_idx <= cap_idx + 1;
                sda_slave_drive <= 1'b1;
                bit_cnt <= 0;
            end
        end
    end
    // release ACK on next SCL falling edge
    wire scl_falling = !scl && scl_d;
    always @(posedge clk) if (scl_falling) sda_slave_drive <= 1'b0;

    integer errors = 0;

    initial begin
        $dumpfile("tb_i2c_master.vcd");
        $dumpvars(0, tb_i2c_master);

        repeat (5) @(posedge clk);
        rst = 0;
        repeat (5) @(posedge clk);

        start = 1;
        @(posedge clk); #1; start = 0;

        wait (done === 1'b1);
        repeat (2) @(posedge clk);

        if (nack_error) begin
            errors = errors + 1;
            $display("ERROR: nack_error asserted, expected clean ACK'd transaction");
        end

        if (cap_idx < 3) begin
            errors = errors + 1;
            $display("ERROR: slave only captured %0d of 3 expected bytes", cap_idx);
        end else begin
            if (captured[0] !== {7'h21, 1'b0}) begin
                errors = errors + 1;
                $display("ERROR: byte0 (addr+W) = %02x, expected %02x", captured[0], {7'h21,1'b0});
            end
            if (captured[1] !== reg_addr) begin
                errors = errors + 1;
                $display("ERROR: byte1 (reg_addr) = %02x, expected %02x", captured[1], reg_addr);
            end
            if (captured[2] !== reg_data) begin
                errors = errors + 1;
                $display("ERROR: byte2 (reg_data) = %02x, expected %02x", captured[2], reg_data);
            end
        end

        if (errors == 0)
            $display("TB_I2C_MASTER: PASS (addr+W=%02x reg_addr=%02x reg_data=%02x, all ACKed)",
                       captured[0], captured[1], captured[2]);
        else
            $display("TB_I2C_MASTER: FAIL (%0d errors)", errors);

        $finish;
    end

    // watchdog
    initial begin
        #500000;
        $display("TB_I2C_MASTER: WATCHDOG TIMEOUT");
        $finish;
    end

endmodule
