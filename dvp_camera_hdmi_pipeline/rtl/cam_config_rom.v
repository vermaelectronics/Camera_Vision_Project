// ============================================================================
// cam_config_rom.v -- sensor register-table sequencer
// ----------------------------------------------------------------------------
// Walks a small ROM of {8'h_reg_addr, 8'h_reg_data} pairs out through
// i2c_master.v, one register write at a time, on power-up/reset. This is
// intentionally sensor-agnostic scaffolding: the TABLE below is a
// PLACEHOLDER (a handful of illustrative entries in the style of an
// OV5640/OV2640-class sensor's 8-bit register map) -- replace it with your
// exact sensor's documented initialization sequence before relying on this
// to actually configure a camera. Search your sensor's datasheet/vendor SDK
// for its "DVP 720p60"/"1080p60" register list and drop it in here.
//
// NUM_REGS is a parameter so nextpnr/yosys size the ROM to whatever table
// you provide; the array is initialized with a Verilog $readmemh-style
// literal list for easy editing.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module cam_config_rom #(
    parameter NUM_REGS = 8
) (
    input  wire       clk,
    input  wire        rst,
    input  wire        go,          // pulse once (e.g. after PLL lock) to start
    output reg          config_done, // stays high once the whole table is written

    output reg           i2c_start,
    output reg  [7:0]    i2c_reg_addr,
    output reg  [7:0]    i2c_reg_data,
    input  wire          i2c_busy,
    input  wire          i2c_done,
    input  wire          i2c_nack_error
);

    // ---------------------------------------------------------------------
    // PLACEHOLDER register table -- REPLACE with your sensor's real init
    // sequence. Format: {reg_addr[7:0], reg_data[7:0]} -- 8-bit register
    // addressing, matching this sequencer's i2c_reg_addr[7:0] port (fits
    // 8-bit-addressed sensor families such as OV7670/OV2640/GC0308-class
    // parts). If your sensor uses 16-bit register addresses instead (e.g.
    // OV5640/OV5647-class parts), widen i2c_reg_addr to [15:0] here and in
    // i2c_master.v's address-byte phase -- see README.md "Adapting to your
    // sensor".
    // ---------------------------------------------------------------------
    reg [15:0] table_rom [0:NUM_REGS-1];
    initial begin
        table_rom[0] = {8'h12, 8'h80}; // example: software reset             (placeholder)
        table_rom[1] = {8'h12, 8'h04}; // example: enable DVP RGB/YUV output  (placeholder)
        table_rom[2] = {8'h11, 8'h01}; // example: PCLK/system clock divider  (placeholder)
        table_rom[3] = {8'h3A, 8'h04}; // example: output format select       (placeholder)
        table_rom[4] = {8'h17, 8'h00}; // example: horizontal window start    (placeholder)
        table_rom[5] = {8'h18, 8'hFF}; // example: horizontal window end      (placeholder)
        table_rom[6] = {8'h19, 8'h00}; // example: vertical window start      (placeholder)
        table_rom[7] = {8'h1A, 8'hFF}; // example: vertical window end        (placeholder)
        // ... extend NUM_REGS and this table with your sensor's actual list.
    end

    localparam S_IDLE = 0, S_ISSUE = 1, S_WAIT = 2, S_NEXT = 3, S_DONE = 4;
    reg [2:0] state = S_IDLE;
    reg [$clog2(NUM_REGS>1?NUM_REGS:2)-1:0] idx;

    always @(posedge clk) begin
        if (rst) begin
            state        <= S_IDLE;
            config_done  <= 1'b0;
            i2c_start    <= 1'b0;
            idx          <= 0;
        end else begin
            i2c_start <= 1'b0;
            case (state)
                S_IDLE: begin
                    config_done <= 1'b0;
                    if (go) begin
                        idx   <= 0;
                        state <= S_ISSUE;
                    end
                end

                S_ISSUE: begin
                    if (!i2c_busy) begin
                        i2c_reg_addr <= table_rom[idx][15:8];
                        i2c_reg_data <= table_rom[idx][7:0];
                        i2c_start    <= 1'b1;
                        state        <= S_WAIT;
                    end
                end

                S_WAIT: begin
                    if (i2c_done) begin
                        // i2c_nack_error is available here for retry/error
                        // handling if you want it; this reference sequencer
                        // simply continues (see README for a retry-capable
                        // variant note).
                        state <= S_NEXT;
                    end
                end

                S_NEXT: begin
                    if (idx == NUM_REGS - 1) begin
                        state <= S_DONE;
                    end else begin
                        idx   <= idx + 1;
                        state <= S_ISSUE;
                    end
                end

                S_DONE: begin
                    config_done <= 1'b1;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
