`timescale 1ns/1ps
// spi_master : generic byte-wide SPI master, mode 0 (CPOL=0, CPHA=0),
// MSB first, for the sys_clk (20 MHz) domain. Chip-select is NOT
// generated here: SD commands hold CS across multi-byte sequences, so
// CS is driven by the caller (sd_spi_init / sd_block_read) and muxed
// onto the physical pin in sd_uart_top.
//
// SCLK frequency = CLK_FREQ_HZ / (2*(clk_div+1)), set at runtime via
// clk_div so the same core can run slow during card bring-up and fast
// afterwards, without hard-coding either speed into the module.
module spi_master #(
    parameter CLK_FREQ_HZ = 20_000_000
)(
    input  wire        clk,
    input  wire        rst,
    input  wire [15:0] clk_div,
    input  wire        start,
    input  wire [7:0]  tx_data,
    output reg  [7:0]  rx_data,
    output reg         busy,
    output reg         done,
    output reg         sclk,
    output reg         mosi,
    input  wire        miso
);
    localparam S_IDLE = 2'd0,
               S_LOW  = 2'd1,
               S_HIGH = 2'd2;

    reg [1:0]  state;
    reg [15:0] div_cnt;
    reg [3:0]  bit_cnt;
    reg [7:0]  shreg;

    always @(posedge clk) begin
        if (rst) begin
            state   <= S_IDLE;
            sclk    <= 1'b0;
            mosi    <= 1'b1;
            busy    <= 1'b0;
            done    <= 1'b0;
            div_cnt <= 16'd0;
            bit_cnt <= 4'd0;
            rx_data <= 8'd0;
            shreg   <= 8'd0;
        end else begin
            done <= 1'b0;
            case (state)
                S_IDLE: begin
                    sclk <= 1'b0;
                    busy <= 1'b0;
                    if (start) begin
                        shreg   <= tx_data;
                        mosi    <= tx_data[7];
                        bit_cnt <= 4'd0;
                        div_cnt <= 16'd0;
                        busy    <= 1'b1;
                        state   <= S_LOW;
                    end
                end

                // CPHA=0: data is driven while SCLK is low and sampled
                // on the rising edge.
                S_LOW: begin
                    if (div_cnt == clk_div) begin
                        div_cnt <= 16'd0;
                        sclk    <= 1'b1;
                        rx_data <= {rx_data[6:0], miso};
                        state   <= S_HIGH;
                    end else begin
                        div_cnt <= div_cnt + 16'd1;
                    end
                end

                // Data changes on the falling edge (end of S_HIGH),
                // ready for the next rising-edge sample.
                S_HIGH: begin
                    if (div_cnt == clk_div) begin
                        div_cnt <= 16'd0;
                        sclk    <= 1'b0;
                        if (bit_cnt == 4'd7) begin
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            state <= S_IDLE;
                        end else begin
                            bit_cnt <= bit_cnt + 4'd1;
                            shreg   <= {shreg[6:0], 1'b0};
                            mosi    <= shreg[6];
                            state   <= S_LOW;
                        end
                    end else begin
                        div_cnt <= div_cnt + 16'd1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
