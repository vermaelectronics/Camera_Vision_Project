`timescale 1ns/1ps
// status_led : diagnostic LEDs, sys_clk domain.
//   led[0] = SD init OK        (sticky)
//   led[1] = SPI activity      (stretched, same technique as the
//                                sd_uart_top project's led_ctrl.v)
//   led[2] = image fully loaded into the framebuffer (sticky)
//   led[3] = error - init/mount/search/format/size failure (sticky)
module status_led (
    input  wire       clk,          // sys_clk, 20 MHz
    input  wire       rst,
    input  wire       sd_init_ok,
    input  wire       spi_activity,
    input  wire       img_loaded,
    input  wire       error_in,
    output wire [3:0] led
);
    localparam STRETCH_CYCLES = 22'd3_000_000; // ~150 ms @ 20 MHz

    reg        led0_latch, led2_latch, led3_latch;
    reg [21:0] activity_cnt;
    reg        activity_prev;

    always @(posedge clk) begin
        if (rst) begin
            led0_latch    <= 1'b0;
            led2_latch    <= 1'b0;
            led3_latch    <= 1'b0;
            activity_cnt  <= 22'd0;
            activity_prev <= 1'b0;
        end else begin
            if (sd_init_ok) led0_latch <= 1'b1;
            if (img_loaded) led2_latch <= 1'b1;
            if (error_in)   led3_latch <= 1'b1;

            activity_prev <= spi_activity;
            if (spi_activity && !activity_prev) begin
                activity_cnt <= STRETCH_CYCLES;
            end else if (activity_cnt != 22'd0) begin
                activity_cnt <= activity_cnt - 22'd1;
            end
        end
    end

    assign led[0] = led0_latch;
    assign led[1] = (activity_cnt != 22'd0) || spi_activity;
    assign led[2] = led2_latch;
    assign led[3] = led3_latch;

endmodule
