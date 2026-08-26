// ============================================================================
// cam_config_rom.v -- sensor register-table sequencer
// ----------------------------------------------------------------------------
// Walks a ROM of {16'h_reg_addr, 8'h_reg_data} pairs out through
// i2c_master.v (with ADDR_BYTES=2, matching this module's 16-bit
// i2c_reg_addr port), one register write at a time, on power-up/reset.
//
// The table below targets a specific sensor: the Waveshare OV5640 DVP
// module, configured for DVP RGB565 output at 1280x720. If you're using a
// different sensor, replace the whole table -- see README.md "Adapting to
// your sensor" (and switch i2c_master's ADDR_BYTES back to 1 if your part
// uses 8-bit register addressing instead, e.g. OV7670/OV2640-class parts).
//
// IMPORTANT HONESTY NOTE: unlike the DVI timing/TMDS-encoding math
// elsewhere in this project (verified by simulation and hand computation),
// this register table cannot be verified the same way -- there is no
// simulation model of real OV5640 silicon. It is reproduced from general
// knowledge of the commonly-published/community OV5640 DVP-RGB565 init
// sequence (a table that shows up, in similar form, across many public
// OV5640 reference projects), NOT hardware-tested against a real sensor in
// this session. Treat it as a strong, structurally-correct starting point
// -- not a guaranteed-correct final sequence. Before relying on it:
//   - Cross-check against Waveshare's own official demo code for this
//     exact board (search their wiki for "OV5640 camera board") if
//     anything doesn't work.
//   - The two most likely things to need adjustment for YOUR board/copy of
//     the sensor are marked below: the format byte-order register (0x4300
//     -- if colours come out channel-swapped, try the alternate value
//     noted there, or flip pixel_formatter.v's BYTE_SWAP parameter
//     instead, which fixes the same class of issue), and the output-window
//     registers (0x3808-0x3821 -- if the image is shifted, cropped
//     wrong, or doesn't appear at all).
//   - Sensor "core"/analog tuning (AWB gains, lens-shading correction,
//     gamma curve, sharpness/denoise) is deliberately NOT included below
//     to keep this table to registers whose function is well-established
//     -- the sensor's power-on analog defaults are generally sane enough
//     to get a recognisable (if not colour/exposure-perfect) image with
//     just clock/format/window configured correctly. Add tuning registers
//     from OV5640's full datasheet/app-note register list once basic
//     capture is confirmed working.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module cam_config_rom #(
    parameter NUM_REGS = 61
) (
    input  wire       clk,
    input  wire        rst,
    input  wire        go,          // pulse once (e.g. after PLL lock + power-up sequencer done) to start
    output reg          config_done, // stays high once the whole table is written

    output reg           i2c_start,
    output reg  [15:0]   i2c_reg_addr,
    output reg  [7:0]    i2c_reg_data,
    input  wire          i2c_busy,
    input  wire          i2c_done,
    input  wire          i2c_nack_error
);

    // ---------------------------------------------------------------------
    // OV5640 DVP RGB565 @ 1280x720 init table.
    // Format: {reg_addr[15:0], reg_data[7:0]}. See header comment above.
    // ---------------------------------------------------------------------
    reg [23:0] table_rom [0:NUM_REGS-1];
    initial begin
        // ---- Reset / wake -------------------------------------------------
        table_rom[ 0] = {16'h3103, 8'h11}; // system clock from PAD (pre-reset default)
        table_rom[ 1] = {16'h3008, 8'h82}; // software reset
        table_rom[ 2] = {16'h3008, 8'h42}; // software power-up / normal operation

        // ---- System clock / PLL (sets the internal PCLK rate) -------------
        table_rom[ 3] = {16'h3103, 8'h03}; // system clock now from PLL
        table_rom[ 4] = {16'h3017, 8'hFF}; // FREX/VSYNC/HREF/PCLK/D[9:6] pad output enable
        table_rom[ 5] = {16'h3018, 8'hFF}; // D[5:0]/GPIO pad output enable
        table_rom[ 6] = {16'h3034, 8'h1A}; // PLL charge pump / bit-width control
        table_rom[ 7] = {16'h3035, 8'h11}; // system clock divider
        table_rom[ 8] = {16'h3036, 8'h46}; // PLL multiplier
        table_rom[ 9] = {16'h3037, 8'h13}; // PLL pre-divider / root divider
        table_rom[10] = {16'h3108, 8'h01}; // PCLK root divider

        // ---- Sensor core timing/analog (widely-cited "standard" values) ---
        table_rom[11] = {16'h3630, 8'h36};
        table_rom[12] = {16'h3631, 8'h0E};
        table_rom[13] = {16'h3632, 8'hE2};
        table_rom[14] = {16'h3633, 8'h12};
        table_rom[15] = {16'h3621, 8'hE0};
        table_rom[16] = {16'h3704, 8'hA0};
        table_rom[17] = {16'h3703, 8'h5A};
        table_rom[18] = {16'h3715, 8'h78};
        table_rom[19] = {16'h3717, 8'h01};
        table_rom[20] = {16'h370B, 8'h60};
        table_rom[21] = {16'h3705, 8'h1A};
        table_rom[22] = {16'h3905, 8'h02};
        table_rom[23] = {16'h3906, 8'h10};
        table_rom[24] = {16'h3901, 8'h0A};
        table_rom[25] = {16'h3731, 8'h12};
        table_rom[26] = {16'h3600, 8'h08};
        table_rom[27] = {16'h3601, 8'h33};
        table_rom[28] = {16'h302D, 8'h60};
        table_rom[29] = {16'h3620, 8'h52};
        table_rom[30] = {16'h371B, 8'h20};
        table_rom[31] = {16'h3635, 8'h13};
        table_rom[32] = {16'h3636, 8'h03};
        table_rom[33] = {16'h3634, 8'h40};
        table_rom[34] = {16'h3622, 8'h01};

        // ---- AEC/AGC basics -------------------------------------------------
        table_rom[35] = {16'h3A08, 8'h01};
        table_rom[36] = {16'h3A09, 8'h27};
        table_rom[37] = {16'h3A0A, 8'h00};
        table_rom[38] = {16'h3A0B, 8'hF6};
        table_rom[39] = {16'h3A0D, 8'h04};
        table_rom[40] = {16'h3A0E, 8'h03};
        table_rom[41] = {16'h3A0F, 8'h30};
        table_rom[42] = {16'h3A10, 8'h28};
        table_rom[43] = {16'h3A1B, 8'h30};
        table_rom[44] = {16'h3A1E, 8'h26};

        // ---- Output window: 1280x720 (see header note on this block) ------
        table_rom[45] = {16'h3800, 8'h00}; // X_ADDR_START hi
        table_rom[46] = {16'h3801, 8'h00}; // X_ADDR_START lo
        table_rom[47] = {16'h3802, 8'h00}; // Y_ADDR_START hi
        table_rom[48] = {16'h3803, 8'h00}; // Y_ADDR_START lo
        table_rom[49] = {16'h3804, 8'h0A}; // X_ADDR_END hi
        table_rom[50] = {16'h3805, 8'h3F}; // X_ADDR_END lo (0x0A3F = 2623)
        table_rom[51] = {16'h3806, 8'h07}; // Y_ADDR_END hi
        table_rom[52] = {16'h3807, 8'h9B}; // Y_ADDR_END lo (0x079B = 1947)
        table_rom[53] = {16'h3808, 8'h05}; // DVP output width hi  (0x0500 = 1280)
        table_rom[54] = {16'h3809, 8'h00}; // DVP output width lo
        table_rom[55] = {16'h380A, 8'h02}; // DVP output height hi (0x02D0 = 720)
        table_rom[56] = {16'h380B, 8'hD0}; // DVP output height lo
        table_rom[57] = {16'h3814, 8'h31}; // X subsample increment
        table_rom[58] = {16'h3815, 8'h31}; // Y subsample increment

        // ---- Format select: RGB565 over DVP --------------------------------
        // If colours come out channel-swapped on your board, try 8'h6F here
        // instead of 8'h61 (or leave this as-is and set pixel_formatter.v's
        // BYTE_SWAP=1 instead -- either fixes the same underlying issue).
        table_rom[59] = {16'h4300, 8'h61};

        // ---- ISP output-format mux select: RGB (not raw/Bayer passthrough) --
        // 0x501F selects what the ISP pipeline actually hands to the DVP
        // output stage: 0x00 = ISP bypass (raw Bayer), 0x01 = RGB, 0x02 =
        // raw DPC, 0x03 = snapshot. 0x4300 above only sets RGB565's byte
        // order/channel layout -- it has no effect unless the ISP is
        // actually muxed to output RGB. Missing this register is one of
        // the most common OV5640 bring-up mistakes: I2C ACKs every write,
        // PCLK/HREF/VSYNC/data all toggle and look like a valid stream, but
        // the pixel content itself is meaningless (commonly reads back as
        // solid black) because the sensor is still in its raw/ISP-bypass
        // default. Must come after the format-select register above.
        table_rom[60] = {16'h501F, 8'h01};

        // ... extend NUM_REGS and this table further (AWB/gamma/lens-shading
        // tuning, mirror/flip, sharpness) once basic capture is confirmed.
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
                        i2c_reg_addr <= table_rom[idx][23:8];
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
