// ============================================================================
// spi_master.v -- simple SPI master, mode 0 (CPOL=0, CPHA=0), MSB-first,
// 8 bits per transfer, runtime-programmable clock divider.
// ----------------------------------------------------------------------------
// SD cards in SPI mode need a slow (<=400kHz) clock during initialization
// and can go much faster (multi-MHz) once initialized -- the divider is a
// live input rather than a synthesis-time parameter so one module instance
// serves both phases (see sdcard_spi.v).
//
// Mode 0: MOSI is driven on the falling edge (or before the very first
// rising edge), MISO is sampled on the rising edge. That is exactly what
// the SD SPI protocol expects.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module spi_master (
    input  wire        clk,
    input  wire        rst,

    input  wire [15:0] div,       // sclk half-period = (div+1) clk cycles
    input  wire        start,     // pulse for one cycle to begin a byte transfer
    input  wire [7:0]  tx_byte,
    output reg         busy,
    output reg         done,      // one-cycle pulse; rx_byte valid this cycle
    output reg  [7:0]  rx_byte,

    output reg          sclk,
    output reg          mosi,
    input  wire          miso
);

    localparam ST_IDLE = 1'b0, ST_RUN = 1'b1;

    reg        state;
    reg [15:0] div_cnt;
    reg        sclk_phase;   // 0 = waiting to rise, 1 = waiting to fall
    reg [3:0]  bits_left;    // 8 down to 1
    reg [7:0]  shift_out;
    reg [7:0]  shift_in;

    always @(posedge clk) begin
        done <= 1'b0;
        if (rst) begin
            state   <= ST_IDLE;
            busy    <= 1'b0;
            sclk    <= 1'b0;
            mosi    <= 1'b1;
            div_cnt <= 16'd0;
        end else begin
            case (state)
                ST_IDLE: begin
                    sclk <= 1'b0;
                    if (start) begin
                        busy       <= 1'b1;
                        shift_out  <= tx_byte;
                        mosi       <= tx_byte[7];
                        bits_left  <= 4'd8;
                        div_cnt    <= 16'd0;
                        sclk_phase <= 1'b0;
                        state      <= ST_RUN;
                    end
                end

                ST_RUN: begin
                    if (div_cnt == div) begin
                        div_cnt <= 16'd0;
                        if (!sclk_phase) begin
                            // rising edge: sample MISO into the shift-in reg
                            sclk       <= 1'b1;
                            shift_in   <= {shift_in[6:0], miso};
                            sclk_phase <= 1'b1;
                        end else begin
                            // falling edge: one bit period complete
                            sclk       <= 1'b0;
                            sclk_phase <= 1'b0;
                            if (bits_left == 4'd1) begin
                                rx_byte <= shift_in; // already holds all 8 sampled bits
                                done    <= 1'b1;
                                busy    <= 1'b0;
                                state   <= ST_IDLE;
                            end else begin
                                shift_out <= {shift_out[6:0], 1'b0};
                                mosi      <= shift_out[6];
                                bits_left <= bits_left - 1'b1;
                            end
                        end
                    end else begin
                        div_cnt <= div_cnt + 1'b1;
                    end
                end
            endcase
        end
    end

endmodule

`default_nettype wire
