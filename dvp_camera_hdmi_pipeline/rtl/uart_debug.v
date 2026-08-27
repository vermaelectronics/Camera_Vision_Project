// ============================================================================
// uart_debug.v -- live hardware-status text over UART, for a PuTTY/minicom
// terminal on your PC (default 115200 8N1)
// ----------------------------------------------------------------------------
// Sends a one-time startup banner, then a refreshed single-line status
// report roughly once per second, e.g.:
//
//   PLL=1 MCLK=1 SEQ=1 CFG=1 NACK=0 BUF=1 MODE=C FRAMES=0x4B ACT=1 RAW=A5C3F02D
//
//   PLL   = pixel-clock PLL locked
//   MCLK  = camera MCLK PLL locked
//   SEQ   = cam_power_sequencer finished (PWDN/RESET sequencing done)
//   CFG   = cam_config_rom finished walking the I2C register table
//   NACK  = cumulative count of NACKed I2C transactions this run (hex
//           digit, saturates at F) -- 0 means every register write ACKed
//   BUF   = CDC buffer pre-filled and ready (live video should be visible)
//   MODE  = C(amera) or P(attern) -- mirrors led[4]/pattern_sel
//   FRAMES= hex count of camera VSYNC pulses seen since reset -- proves
//           the sensor is actually delivering frames, which the 5 status
//           LEDs alone can't show (wraps at 0xFF; that's fine, it's a
//           liveness indicator, not a precise frame count)
//   ACT   = capture activity indicator: lit whenever real pixels have been
//           captured (cam_pixel_valid pulsing) recently -- stretched to a
//           human-visible duration (see CAM_CLK_HZ_APPROX below), so it
//           reads as solidly lit during continuous capture rather than an
//           imperceptible flicker. Also driven out to a real output pin
//           (`cap_led`, see the top level) if you want a physical LED
//           instead of/alongside this text field -- the OV5640 module's
//           own onboard LEDs (next to the lens) are NOT usable for this:
//           on essentially every OV5640 breakout, those are fixed
//           power-on indicators wired straight to the 3.3V rail with no
//           GPIO/register connection at all, so nothing digital -- I2C
//           included -- can control them. This ACT field/cap_led output
//           is the real, working equivalent of what those fixed LEDs
//           can't do.
//   RAW   = the last 4 bytes actually captured off cam_d[7:0] (oldest byte
//           first), refreshed live -- this is the ONLY field that shows
//           real sensor data content directly, rather than a status flag
//           derived from it. If this is stuck at a fixed value (0x00000000,
//           0xFFFFFFFF, or any other constant that never changes across
//           refreshes), the data bus isn't delivering real varying data --
//           a wiring or sensor-not-streaming problem. If it visibly varies
//           refresh to refresh, real bytes are arriving and any remaining
//           image problem is in decode/format, not the data bus itself.
//
// This is exactly the same status information the top level's LEDs already
// expose, plus a live frame counter -- richer, and readable in a terminal
// instead of having to interpret 5 LEDs by eye.
//
// All status inputs may come from other clock domains (clk_pixel, cam_mclk,
// cam_pclk) -- they're resynchronized into this module's own `clk` domain
// via simple 2FF synchronizers (single-bit level signals, safe this way).
// `clk` is meant to be the always-running 50MHz board oscillator, so status
// reporting (including the startup banner) works even before the pixel PLL
// locks. FRAMES is crossed from cam_vsync via a toggle+2FF+edge-detect
// scheme -- the same class of CDC technique used for pulses elsewhere in
// this project (see cam_power_sequencer.v's reset synchronizer, or
// async_fifo.v's Gray-code pointer crossings for the general idea).
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module uart_debug #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer BAUD        = 115200,
    parameter integer TICK_HZ     = 1,         // status-line refresh rate
    parameter integer STRETCH_MS  = 200        // ACT/cap_led visible-on duration per activity pulse
) (
    input  wire clk,          // free-running domain (board oscillator)
    input  wire rst,

    input  wire pll_locked,
    input  wire mclk_locked,
    input  wire cam_seq_done,
    input  wire cfg_done,
    input  wire i2c_nack,
    input  wire buf_ready,
    input  wire pattern_sel,
    input  wire cam_vsync,    // raw, cam_pclk domain
    input  wire cam_pixel_valid,  // raw, cam_pclk domain -- pulses once per captured pixel
    input  wire [31:0] raw_bytes, // last 4 captured cam_d[7:0] bytes, cam_pclk domain, oldest first

    output wire tx,
    output wire cap_led       // lit while pixel capture is actively happening (stretched, see header)
);

    // ---- resynchronize single-bit status signals into `clk` domain -------
    reg [1:0] pll_sync, mclk_sync, seq_sync, cfg_sync, nack_sync, buf_sync, pat_sync;
    always @(posedge clk) begin
        pll_sync  <= {pll_sync[0],  pll_locked};
        mclk_sync <= {mclk_sync[0], mclk_locked};
        seq_sync  <= {seq_sync[0],  cam_seq_done};
        cfg_sync  <= {cfg_sync[0],  cfg_done};
        nack_sync <= {nack_sync[0], i2c_nack};
        buf_sync  <= {buf_sync[0],  buf_ready};
        pat_sync  <= {pat_sync[0],  pattern_sel};
    end
    wire pll_s  = pll_sync[1];
    wire mclk_s = mclk_sync[1];
    wire seq_s  = seq_sync[1];
    wire cfg_s  = cfg_sync[1];
    wire nack_s = nack_sync[1];
    wire buf_s  = buf_sync[1];
    wire pat_s  = pat_sync[1];

    // ---- resynchronize the raw byte snapshot into `clk` domain -----------
    // Plain per-bit 2FF synchronization of a 32-bit bus is not glitch-free
    // in general (different bits can resolve on different cycles, so a
    // snapshot caught mid-transition could show a torn mix of an old and
    // new value for one refresh). That's an acceptable, explicitly-chosen
    // tradeoff here: RAW is a human-readable diagnostic display refreshed
    // once per second, not something anything else in the design depends
    // on -- a rare, single-refresh glitch self-corrects on the next tick
    // and does not affect video output or any other logic.
    reg [31:0] raw_sync1, raw_sync2;
    always @(posedge clk) begin
        raw_sync1 <= raw_bytes;
        raw_sync2 <= raw_sync1;
    end

    // ---- cumulative NACK count: edge-detect each new NACKed transaction --
    reg       nack_s_d1;
    reg [3:0] nack_count;
    always @(posedge clk) begin
        if (rst) begin
            nack_s_d1  <= 1'b0;
            nack_count <= 4'h0;
        end else begin
            nack_s_d1 <= nack_s;
            if (nack_s && !nack_s_d1 && nack_count != 4'hF)
                nack_count <= nack_count + 1'b1;
        end
    end

    // ---- frame counter: cam_vsync (cam_pclk domain) -> clk domain --------
    reg vsync_toggle = 1'b0;
    always @(posedge cam_vsync) vsync_toggle <= ~vsync_toggle;

    reg [1:0] vtog_sync;
    reg       vtog_sync_d1;
    reg [7:0] frame_cnt;
    always @(posedge clk) begin
        if (rst) begin
            vtog_sync    <= 2'b00;
            vtog_sync_d1 <= 1'b0;
            frame_cnt    <= 8'h00;
        end else begin
            vtog_sync    <= {vtog_sync[0], vsync_toggle};
            vtog_sync_d1 <= vtog_sync[1];
            if (vtog_sync[1] != vtog_sync_d1)
                frame_cnt <= frame_cnt + 1'b1;
        end
    end

    // ---- capture-activity indicator: cam_pixel_valid (cam_pclk domain) ---
    // Same toggle+2FF+edge-detect CDC technique as the frame counter above.
    // The stretch timer then runs entirely in the fixed `clk` domain (not
    // cam_pclk, whose actual frequency varies by sensor and isn't known at
    // compile time), so STRETCH_MS is an exact duration regardless of the
    // camera's real pixel clock rate.
    reg pixval_toggle = 1'b0;
    always @(posedge cam_pixel_valid) pixval_toggle <= ~pixval_toggle;

    reg [1:0] pvtog_sync;
    reg       pvtog_sync_d1;
    always @(posedge clk) begin
        if (rst) begin
            pvtog_sync    <= 2'b00;
            pvtog_sync_d1 <= 1'b0;
        end else begin
            pvtog_sync    <= {pvtog_sync[0], pixval_toggle};
            pvtog_sync_d1 <= pvtog_sync[1];
        end
    end
    wire activity_pulse = (pvtog_sync[1] != pvtog_sync_d1);

    localparam integer STRETCH_CYCLES = (CLK_FREQ_HZ / 1000) * STRETCH_MS;
    localparam integer STRETCH_W = (STRETCH_CYCLES <= 1) ? 1 : $clog2(STRETCH_CYCLES);
    reg [STRETCH_W-1:0] stretch_cnt;
    reg                 act_led_r;
    always @(posedge clk) begin
        if (rst) begin
            stretch_cnt <= 0;
            act_led_r   <= 1'b0;
        end else if (activity_pulse) begin
            stretch_cnt <= STRETCH_CYCLES - 1;
            act_led_r   <= 1'b1;
        end else if (stretch_cnt != 0) begin
            stretch_cnt <= stretch_cnt - 1'b1;
            act_led_r   <= 1'b1;
        end else begin
            act_led_r   <= 1'b0;
        end
    end

    assign cap_led = act_led_r;

    // ---- 1Hz (or TICK_HZ) status-line refresh tick ------------------------
    localparam integer TICK_DIV = CLK_FREQ_HZ / TICK_HZ;
    localparam integer TICK_W   = (TICK_DIV <= 1) ? 1 : $clog2(TICK_DIV);
    reg [TICK_W-1:0] tick_cnt;
    reg              tick;
    always @(posedge clk) begin
        if (rst) begin
            tick_cnt <= 0;
            tick     <= 1'b0;
        end else if (tick_cnt == TICK_DIV-1) begin
            tick_cnt <= 0;
            tick     <= 1'b1;
        end else begin
            tick_cnt <= tick_cnt + 1'b1;
            tick     <= 1'b0;
        end
    end

    // ---- snapshot latch: freezes one status line's values for the whole
    // ~5ms it takes to transmit, so the line can't show a torn mix of two
    // different moments' status.
    reg       snap_pll, snap_mclk, snap_seq, snap_cfg, snap_buf, snap_pat, snap_act;
    reg [3:0] snap_nack;
    reg [7:0] snap_frame;
    reg [31:0] snap_raw;

    // ---- character generators ---------------------------------------------
    function [7:0] hex_ascii;
        input [3:0] nib;
        begin
            hex_ascii = (nib < 10) ? (8'h30 + nib) : (8'h41 + (nib - 4'd10));
        end
    endfunction

    function [7:0] bit_ascii;
        input b;
        begin
            bit_ascii = b ? 8'h31 : 8'h30;
        end
    endfunction

    // Fixed-text templates, held as one wide packed string constant each --
    // computed/verified byte-for-byte (see commit message / dev notes) so
    // BANNER_LEN and STATUS_LEN exactly match their string literal's byte
    // length (Verilog packs the first character into the most-significant
    // byte of a same-width reg). Live fields overwrite fixed placeholder
    // characters in the template at fixed indices -- see status_char().
    localparam integer BANNER_LEN = 68;
    localparam [8*BANNER_LEN-1:0] BANNER_STR =
        "\r\n=== DVP Camera->HDMI Pipeline (720p60, OV5640) -- UART Debug ===\r\n";

    localparam integer STATUS_LEN = 77;
    localparam [8*STATUS_LEN-1:0] STATUS_TEMPLATE =
        "PLL=0 MCLK=0 SEQ=0 CFG=0 NACK=0 BUF=0 MODE=C FRAMES=0x00 ACT=0 RAW=00000000\r\n";

    function [7:0] banner_char;
        input [6:0] idx;
        begin
            banner_char = BANNER_STR[8*(BANNER_LEN-1-idx) +: 8];
        end
    endfunction

    function [7:0] status_char;
        input [6:0] idx;
        begin
            case (idx)
                7'd4:  status_char = bit_ascii(snap_pll);
                7'd11: status_char = bit_ascii(snap_mclk);
                7'd17: status_char = bit_ascii(snap_seq);
                7'd23: status_char = bit_ascii(snap_cfg);
                7'd30: status_char = hex_ascii(snap_nack);
                7'd36: status_char = bit_ascii(snap_buf);
                7'd43: status_char = snap_pat ? "P" : "C";
                7'd54: status_char = hex_ascii(snap_frame[7:4]);
                7'd55: status_char = hex_ascii(snap_frame[3:0]);
                7'd61: status_char = bit_ascii(snap_act);
                7'd67: status_char = hex_ascii(snap_raw[31:28]);
                7'd68: status_char = hex_ascii(snap_raw[27:24]);
                7'd69: status_char = hex_ascii(snap_raw[23:20]);
                7'd70: status_char = hex_ascii(snap_raw[19:16]);
                7'd71: status_char = hex_ascii(snap_raw[15:12]);
                7'd72: status_char = hex_ascii(snap_raw[11:8]);
                7'd73: status_char = hex_ascii(snap_raw[7:4]);
                7'd74: status_char = hex_ascii(snap_raw[3:0]);
                default: status_char = STATUS_TEMPLATE[8*(STATUS_LEN-1-idx) +: 8];
            endcase
        end
    endfunction

    // ---- top-level send sequencer ------------------------------------------
    localparam S_BANNER_ISSUE = 3'd0,
               S_BANNER_WAIT  = 3'd1,
               S_TICK_WAIT    = 3'd2,
               S_STATUS_ISSUE = 3'd3,
               S_STATUS_WAIT  = 3'd4;

    reg [2:0] state;
    reg [6:0] char_idx;
    reg [7:0] tx_byte;
    reg       tx_start;
    wire      tx_busy;

    uart_tx #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD(BAUD)) u_tx (
        .clk(clk), .rst(rst), .data(tx_byte), .start(tx_start),
        .tx(tx), .busy(tx_busy)
    );

    always @(posedge clk) begin
        if (rst) begin
            state    <= S_BANNER_ISSUE;
            char_idx <= 0;
            tx_start <= 1'b0;
            snap_pll <= 1'b0; snap_mclk <= 1'b0; snap_seq <= 1'b0;
            snap_cfg <= 1'b0; snap_buf  <= 1'b0; snap_pat <= 1'b0;
            snap_nack <= 4'h0; snap_frame <= 8'h00; snap_raw <= 32'h0;
            snap_act <= 1'b0;
        end else begin
            tx_start <= 1'b0; // default: single-cycle pulse

            case (state)
                S_BANNER_ISSUE: begin
                    tx_byte  <= banner_char(char_idx);
                    tx_start <= 1'b1;
                    state    <= S_BANNER_WAIT;
                end

                S_BANNER_WAIT: begin
                    if (!tx_busy && !tx_start) begin
                        if (char_idx == BANNER_LEN-1) begin
                            char_idx <= 0;
                            state    <= S_TICK_WAIT;
                        end else begin
                            char_idx <= char_idx + 1'b1;
                            state    <= S_BANNER_ISSUE;
                        end
                    end
                end

                S_TICK_WAIT: begin
                    if (tick) begin
                        snap_pll   <= pll_s;
                        snap_mclk  <= mclk_s;
                        snap_seq   <= seq_s;
                        snap_cfg   <= cfg_s;
                        snap_nack  <= nack_count;
                        snap_buf   <= buf_s;
                        snap_pat   <= pat_s;
                        snap_frame <= frame_cnt;
                        snap_act   <= act_led_r;
                        snap_raw   <= raw_sync2;
                        char_idx   <= 0;
                        state      <= S_STATUS_ISSUE;
                    end
                end

                S_STATUS_ISSUE: begin
                    tx_byte  <= status_char(char_idx);
                    tx_start <= 1'b1;
                    state    <= S_STATUS_WAIT;
                end

                S_STATUS_WAIT: begin
                    if (!tx_busy && !tx_start) begin
                        if (char_idx == STATUS_LEN-1) begin
                            char_idx <= 0;
                            state    <= S_TICK_WAIT;
                        end else begin
                            char_idx <= char_idx + 1'b1;
                            state    <= S_STATUS_ISSUE;
                        end
                    end
                end

                default: state <= S_BANNER_ISSUE;
            endcase
        end
    end

endmodule

`default_nettype wire
