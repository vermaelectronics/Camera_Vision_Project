// ============================================================================
// i2c_master.v -- simple polled I2C master for camera register configuration
// ----------------------------------------------------------------------------
// Minimal single-master, write-only (register-configuration-style) I2C
// engine: START, 7-bit address + W, an 8-bit or 16-bit register address
// (ADDR_BYTES), 8-bit data, STOP, with ACK checking. Open-drain SDA/SCL
// modeled with tri-state buffers (external 4.7k-10k pull-ups to camera-side
// VDDIO are required on the PCB, as with any I2C bus).
//
// ADDR_BYTES=1: 8-bit register addressing (OV7670/OV2640/GC0308-class parts)
// ADDR_BYTES=2: 16-bit register addressing (OV5640/OV5647-class parts --
//               e.g. this is what the Waveshare OV5640 module needs)
//
// This is deliberately generic/sensor-agnostic beyond that one parameter:
// it just walks a table of {reg_addr, reg_data} pairs handed to it by
// cam_config_rom.v. Pair that module's table with your specific sensor's
// documented register list.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module i2c_master #(
    parameter CLK_FREQ_HZ = 25_000_000,
    parameter I2C_FREQ_HZ = 100_000,     // standard mode; 400_000 for fast mode
    parameter DEV_ADDR7   = 7'h21,       // default 7-bit device address (edit per sensor)
    parameter ADDR_BYTES  = 1            // 1 = 8-bit reg addressing, 2 = 16-bit
) (
    input  wire       clk,
    input  wire        rst,

    // simple command interface
    input  wire         start,       // pulse to begin one register write
    input  wire [15:0]  reg_addr,    // only reg_addr[7:0] used when ADDR_BYTES=1
    input  wire [7:0]   reg_data,
    output reg           busy,
    output reg           done,        // one-cycle pulse when the transaction completes
    output reg           nack_error,  // latched: set if any byte was NACKed

    inout  wire          scl,
    inout  wire          sda
);

    localparam integer QUARTER_PERIOD = CLK_FREQ_HZ / (I2C_FREQ_HZ * 4);
    // byte_idx sequence: 0=addr+W, 1..ADDR_BYTES=register address byte(s)
    // (MSB first when ADDR_BYTES=2), LAST_BYTE_IDX=register data byte.
    localparam integer LAST_BYTE_IDX = ADDR_BYTES + 1;

    reg scl_o = 1'b1, sda_o = 1'b1;
    assign scl = scl_o ? 1'bz : 1'b0; // open-drain
    assign sda = sda_o ? 1'bz : 1'b0;
    wire sda_i = sda;

    reg [15:0] tick_cnt;
    reg [1:0]  quarter; // 0..3 within one SCL period
    reg        tick;

    always @(posedge clk) begin
        if (rst) begin
            tick_cnt <= 0;
            tick     <= 0;
        end else if (tick_cnt == QUARTER_PERIOD - 1) begin
            tick_cnt <= 0;
            tick     <= 1;
        end else begin
            tick_cnt <= tick_cnt + 1;
            tick     <= 0;
        end
    end

    localparam S_IDLE       = 0,
               S_START      = 1,
               S_SHIFT_BIT  = 2,
               S_ACK        = 3,
               S_STOP1      = 4,
               S_STOP2      = 5,
               S_DONE       = 6;

    reg [3:0]  state = S_IDLE;
    reg [1:0]  qphase;             // sub-state within a bit-time (0..3)
    reg [2:0]  bit_idx;
    reg [2:0]  byte_idx;           // 0=addr+W, 1..ADDR_BYTES=reg addr byte(s), LAST_BYTE_IDX=data
    reg [7:0]  shift_reg;

    // Next byte to shift out, given the byte we just finished (byte_idx,
    // pre-increment). Handles both 1-byte and 2-byte register addressing.
    function [7:0] next_byte;
        input [2:0] finished_idx;
        begin
            if (finished_idx == 0) begin
                // just sent addr+W -> send first (and, if ADDR_BYTES=1, only) address byte
                next_byte = (ADDR_BYTES == 2) ? reg_addr[15:8] : reg_addr[7:0];
            end else if (ADDR_BYTES == 2 && finished_idx == 1) begin
                // just sent address MSB -> send address LSB
                next_byte = reg_addr[7:0];
            end else begin
                // just sent the last address byte -> send data
                next_byte = reg_data;
            end
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            state      <= S_IDLE;
            scl_o      <= 1'b1;
            sda_o      <= 1'b1;
            busy       <= 1'b0;
            done       <= 1'b0;
            nack_error <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
                S_IDLE: begin
                    scl_o <= 1'b1;
                    sda_o <= 1'b1;
                    if (start) begin
                        busy       <= 1'b1;
                        nack_error <= 1'b0;
                        byte_idx   <= 0;
                        shift_reg  <= {DEV_ADDR7, 1'b0}; // 7-bit addr + W(0)
                        qphase     <= 0;
                        state      <= S_START;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                S_START: begin
                    // SDA falls while SCL is high
                    if (tick) begin
                        case (qphase)
                            0: sda_o <= 1'b0;
                            1: scl_o <= 1'b0;
                            default: begin
                                bit_idx <= 7;
                                qphase  <= 0;
                                state   <= S_SHIFT_BIT;
                            end
                        endcase
                        if (qphase != 2) qphase <= qphase + 1;
                    end
                end

                S_SHIFT_BIT: begin
                    if (tick) begin
                        case (qphase)
                            0: begin sda_o <= shift_reg[bit_idx]; scl_o <= 1'b0; end
                            1: scl_o <= 1'b1; // SCL rises, data sampled by slave
                            2: scl_o <= 1'b1; // hold high
                            3: begin
                                scl_o <= 1'b0;
                                if (bit_idx == 0) begin
                                    state  <= S_ACK;
                                end else begin
                                    bit_idx <= bit_idx - 1;
                                end
                            end
                        endcase
                        qphase <= qphase + 1;
                    end
                end

                S_ACK: begin
                    if (tick) begin
                        case (qphase)
                            0: begin sda_o <= 1'b1; scl_o <= 1'b0; end // release SDA
                            1: scl_o <= 1'b1;
                            2: begin
                                if (sda_i == 1'b1) nack_error <= 1'b1; // slave should pull low
                            end
                            3: begin
                                scl_o <= 1'b0;
                                if (nack_error) begin
                                    state <= S_STOP1;
                                end else if (byte_idx == LAST_BYTE_IDX) begin
                                    state <= S_STOP1;
                                end else begin
                                    shift_reg <= next_byte(byte_idx);
                                    byte_idx  <= byte_idx + 1;
                                    bit_idx   <= 7;
                                    state     <= S_SHIFT_BIT;
                                end
                            end
                        endcase
                        qphase <= qphase + 1;
                    end
                end

                S_STOP1: begin
                    if (tick) begin
                        case (qphase)
                            0: begin sda_o <= 1'b0; scl_o <= 1'b0; end
                            1: scl_o <= 1'b1;
                            default: state <= S_STOP2;
                        endcase
                        if (qphase != 2) qphase <= qphase + 1; else qphase <= 0;
                    end
                end

                S_STOP2: begin
                    if (tick) begin
                        case (qphase)
                            0: sda_o <= 1'b1; // SDA rises while SCL high -> STOP
                            default: state <= S_DONE;
                        endcase
                        qphase <= qphase + 1;
                    end
                end

                S_DONE: begin
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
