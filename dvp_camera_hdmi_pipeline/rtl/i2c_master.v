// ============================================================================
// i2c_master.v -- simple polled I2C master for camera register configuration
// ----------------------------------------------------------------------------
// Minimal single-master, write-only (register-configuration-style) I2C
// engine: START, 7-bit address + W, 8-bit register address, 8-bit data,
// STOP, with ACK checking. Open-drain SDA/SCL modeled with tri-state
// buffers (external 4.7k-10k pull-ups to camera-side VDDIO are required on
// the PCB, as with any I2C bus).
//
// This is deliberately generic/sensor-agnostic: it just walks a table of
// {reg_addr, reg_data} pairs handed to it by cam_config_rom.v. Pair that
// module's table with your specific sensor's documented register list.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module i2c_master #(
    parameter CLK_FREQ_HZ = 25_000_000,
    parameter I2C_FREQ_HZ = 100_000,     // standard mode; 400_000 for fast mode
    parameter DEV_ADDR7   = 7'h21        // default 7-bit device address (edit per sensor)
) (
    input  wire       clk,
    input  wire        rst,

    // simple command interface
    input  wire         start,       // pulse to begin one register write
    input  wire [7:0]   reg_addr,
    input  wire [7:0]   reg_data,
    output reg           busy,
    output reg           done,        // one-cycle pulse when the transaction completes
    output reg           nack_error,  // latched: set if any byte was NACKed

    inout  wire          scl,
    inout  wire          sda
);

    localparam integer QUARTER_PERIOD = CLK_FREQ_HZ / (I2C_FREQ_HZ * 4);

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
    reg [1:0]  byte_idx;           // 0=addr+W, 1=reg_addr, 2=reg_data
    reg [7:0]  shift_reg;
    reg [3:0]  return_state;

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
                                end else if (byte_idx == 2) begin
                                    state <= S_STOP1;
                                end else begin
                                    byte_idx  <= byte_idx + 1;
                                    shift_reg <= (byte_idx == 0) ? reg_addr : reg_data;
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
