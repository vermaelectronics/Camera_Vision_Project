// ============================================================================
// video_line_buffer.v -- camera-clock -> video-clock frame bridge, a "line
// FIFO": whole rows are claimed by the display strictly in the order the
// camera finished writing them, never by matching a row-number value
// between the two independent clock domains.
// ----------------------------------------------------------------------------
// REPLACES video_cdc_buffer.v in dvp_camera_hdmi_top.v (video_cdc_buffer.v
// itself is unchanged and still used by dvp_camera_hdmi_top_ext.v -- see
// README.md "Known limitations").
//
// ROOT-CAUSE CONTEXT (why this module exists): video_cdc_buffer.v bridges
// the camera and display clock domains as a flat async_fifo -- a stream of
// pixels in capture order, with NO relationship to actual image (x,y)
// position on either side. Critically, dvp_camera_hdmi_top.v never wired
// pixel_formatter.v's pixel_line_start/pixel_frame_start outputs to
// anything (left as `.pixel_line_start(), .pixel_frame_start()` -- an
// empty, unconnected port). Because the camera's own internal line/frame
// blanking timing (its HTS/VTS registers) is never configured to exactly
// match the display's fixed 1280x720@60 timing, and the camera PCLK and
// display pixel clock are genuinely free-running, unrelated oscillators,
// the old design's read/write phase relationship inside the FIFO had
// nothing pinning it to actual row/column boundaries and would drift
// continuously. Real hardware testing during this project's development
// confirmed the resulting symptom precisely: VSYNC/HREF framing correct,
// I2C configuration NACK-free, pixel data actively streaming (confirmed via
// uart_debug.v's ACT/RAW fields) -- yet the displayed image showed no
// recognizable structure at all, even with a close, high-contrast subject.
// That is a spatial-alignment bug, not a color, exposure, or format one.
//
// DESIGN HISTORY -- three real bugs were found and fixed via simulation
// (tb/tb_video_line_buffer.v) before any of this reached real hardware:
//   1. A first attempt had the *display* side pick a ring slot by
//      comparing its own row number (out_y) against the camera's
//      synchronized row counter. That only works if both clocks progress
//      through their frames at a similar phase -- but a camera row and a
//      display row of the "same number" start at essentially arbitrary
//      relative phase (two free-running domains, no genlock). At
//      realistic near-equal clock rates this meant a requested row was
//      frequently not ready until most of its own read window had already
//      elapsed. Fixed by switching to the current design: rows are opaque,
//      ordered FIFO items, never matched by number.
//   2. A write-address off-by-one: resetting the column counter to 0 *and*
//      using its old (pre-reset) value as this same pixel's write address
//      in the same always block, on the cycle a new line starts. Fixed by
//      computing the write address as a combinational "predictive pointer"
//      (0 on a fresh line, else previous+1) -- the same style async_fifo.v
//      already uses for its own write/read pointers.
//   3. No back-pressure at all from the read side to the write side: the
//      write side would happily keep reusing (overwriting) ring slots
//      even ones the read side was still actively mid-way through
//      reading, once it got N_LINES rows ahead of what had been claimed.
//      Real hardware cannot pause a camera's PCLK/HREF to apply back-
//      pressure -- the fix mirrors video_cdc_buffer.v's own established
//      "drop rather than corrupt" policy instead: the write side
//      synchronizes the read side's claim count back into its own domain
//      and simply stops advancing into a new row's slot (repeatedly
//      refreshing the last safely-buffered row with newer pixels instead)
//      whenever doing so would risk overwriting a row that hasn't been
//      claimed with enough margin yet.
//
// CONSEQUENCE: with all three fixed, any camera/display line-rate mismatch
// degrades into, at worst, occasional whole-line repeats -- never wrong
// columns, never a row's data split across two different source rows,
// never corruption from a write racing ahead of a read -- a far more
// benign, recognizable degradation than the old flat-FIFO design's total
// spatial incoherence. Because rows are claimed independent of which
// camera *frame* they belonged to, a single displayed frame's rows can, in
// principle, straddle two camera frames if timing slips -- ordinary
// "tearing", a normal and far more benign video artifact than what this
// module replaces.
//
// CDC: both cross-domain counters (the write side's row-completion count,
// the read side's row-claim count) are Gray-coded and synchronized with
// the same 2-flop-synchronizer technique used throughout this project
// (async_fifo.v's pointers, uart_debug.v's frame counter) -- safe to treat
// as slow-changing level signals because they only advance once per video
// LINE (tens of microseconds at 720p60 PCLK rates), many clock cycles
// between changes in either domain.
//
// LATENCY: exactly two out_pclk cycles from (out_x, out_de) to the
// corresponding out_rgb (one from dp_line_ram's own registered read port,
// one more from the ring-slot mux/hold register) -- ONE MORE than
// video_cdc_buffer.v's async_fifo-based one-cycle read latency, so
// dvp_camera_hdmi_top.v's existing hsync_r/vsync_r/de_r/pattern_rgb_r
// realignment register was widened from one stage to two to match (see
// its own comment there).
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module video_line_buffer #(
    parameter WIDTH   = 1280,          // pixels per line
    parameter N_LINES = 4,             // ring/FIFO depth in whole lines (must be a power of 2)
    parameter ADDR_W  = $clog2(WIDTH),
    parameter SLOT_W  = $clog2(N_LINES)
) (
    // camera (write) side
    input  wire         cam_pclk,
    input  wire         cam_rst,
    input  wire         cam_pixel_valid,
    input  wire [23:0]  cam_rgb,
    input  wire         cam_line_start,   // pixel_formatter.v's pixel_line_start -- fires on every
                                            // line boundary, including a frame's first (that's also
                                            // always a line boundary), which is all this module needs.

    // video (read) side
    input  wire         out_pclk,
    input  wire         out_rst,
    input  wire [15:0]  out_x,            // video_timing_gen's x (0..WIDTH-1 during active video)
    input  wire         out_de,
    output reg  [23:0]  out_rgb,
    output reg          out_ready         // 1 once at least one row has ever been claimed
);

    // ------------------------------------------------------------------
    // Forward declarations (declared here, defined further down, purely
    // so each domain's cross-domain synchronizer can reference the other
    // domain's counter before its own defining always block appears --
    // same portability precedent as async_fifo.v's own header comment on
    // this, for older Icarus Verilog builds that reject a same-module
    // forward reference to a reg not yet declared).
    // ------------------------------------------------------------------
    reg [15:0] wr_row;       // write domain: index of the row currently being written
    reg [15:0] rd_pop_count; // read domain: count of rows claimed so far

    // ------------------------------------------------------------------
    // Write side: column address (a combinational "predictive pointer" --
    // see DESIGN HISTORY item 2 above) and a free-running (never frame-
    // reset) row-completion counter, back-pressured (see item 3) by the
    // read side's synchronized claim count.
    // ------------------------------------------------------------------
    reg [ADDR_W-1:0] wr_x; // column of the most recently written pixel

    wire [ADDR_W-1:0] wr_addr_now =
        cam_line_start ? {ADDR_W{1'b0}} :
        (wr_x == WIDTH-1) ? wr_x : (wr_x + 1'b1); // clamp: a stray extra
                                                    // pixel_valid pulse
                                                    // beyond WIDTH per line
                                                    // rewrites the last
                                                    // column harmlessly.
    wire wr_en_this_pixel = cam_pixel_valid;

    // Gray-code rd_pop_count for CDC into the write domain.
    wire [15:0] rd_pop_gray = (rd_pop_count >> 1) ^ rd_pop_count;
    reg  [15:0] rd_pop_gray_s1, rd_pop_gray_s2;
    always @(posedge cam_pclk) begin
        if (cam_rst) begin
            rd_pop_gray_s1 <= 16'd0;
            rd_pop_gray_s2 <= 16'd0;
        end else begin
            rd_pop_gray_s1 <= rd_pop_gray;
            rd_pop_gray_s2 <= rd_pop_gray_s1;
        end
    end
    reg [15:0] rd_pop_synced;
    integer gi_w;
    always @(*) begin
        rd_pop_synced = rd_pop_gray_s2;
        for (gi_w = 1; gi_w < 16; gi_w = gi_w + 1)
            rd_pop_synced = rd_pop_synced ^ (rd_pop_gray_s2 >> gi_w);
    end

    // Safe to advance into row (wr_row+1) only if doing so keeps the
    // write side within N_LINES of what the read side has claimed --
    // otherwise the slot about to be reused could still be mid-read. See
    // DESIGN HISTORY item 3.
    //
    // Registered, not combinational: this check (16-bit Gray decode +
    // subtract + compare) only needs to be fresh once per LINE, not once
    // per PIXEL -- both wr_row and rd_pop_synced change far slower than
    // cam_pclk. Feeding the raw combinational version straight into
    // wr_row_now/wr_slot below put that whole chain in series with every
    // single pixel write and blew real hardware's cam_pclk timing budget
    // (confirmed against real nextpnr-ecp5 STA: cam_pclk dropped to
    // ~74MHz against a 75MHz target). Registering it here breaks that
    // chain at essentially zero real cost -- one extra cam_pclk cycle of
    // staleness on a signal that only actually matters once every WIDTH+
    // blanking cycles.
    wire wr_room_for_next_row = ((wr_row + 16'd1) - rd_pop_synced) < N_LINES[15:0];
    reg  wr_room_for_next_row_r;
    always @(posedge cam_pclk) begin
        if (cam_rst) wr_room_for_next_row_r <= 1'b1; // optimistic default; settles within a cycle or two of real operation
        else          wr_room_for_next_row_r <= wr_room_for_next_row;
    end
    wire wr_advances_this_row = cam_line_start && wr_room_for_next_row_r;

    // wr_slot: the SAME "predictive pointer" fix as wr_addr_now above,
    // and for the identical reason. wr_row's own register only reflects
    // the new row starting the cycle AFTER the line-start edge -- but
    // dp_line_ram's write for THIS SAME edge's column-0 pixel (addressed
    // by wr_addr_now, which correctly jumps to 0 immediately) still reads
    // wr_row's *old* (pre-update) value at that exact edge, since both
    // updates are ordinary same-edge non-blocking assignments. Without
    // this fix, every row's own column 0 lands in the *previous* row's
    // slot instead of its own -- caught by tb/tb_video_line_buffer.v.
    wire [15:0]        wr_row_now  = wr_advances_this_row ? (wr_row + 16'd1) : wr_row;
    wire [SLOT_W-1:0] wr_slot     = wr_row_now[SLOT_W-1:0];

    always @(posedge cam_pclk) begin
        if (cam_rst) begin
            wr_x   <= {ADDR_W{1'b0}};
            wr_row <= 16'd0;
        end else begin
            if (cam_pixel_valid) wr_x <= wr_addr_now;
            if (wr_advances_this_row) wr_row <= wr_row + 16'd1;
            // else (no room yet): wr_row holds -- further pixels for what
            // would be the next row(s) simply keep refreshing the last
            // safely-buffered row's slot with newer camera data instead
            // of corrupting a row the read side hasn't finished with.
        end
    end

    // Gray-code wr_row for CDC into the read domain.
    wire [15:0] wr_row_gray = (wr_row >> 1) ^ wr_row;
    reg  [15:0] wr_row_gray_s1, wr_row_gray_s2;
    always @(posedge out_pclk) begin
        if (out_rst) begin
            wr_row_gray_s1 <= 16'd0;
            wr_row_gray_s2 <= 16'd0;
        end else begin
            wr_row_gray_s1 <= wr_row_gray;
            wr_row_gray_s2 <= wr_row_gray_s1;
        end
    end
    reg [15:0] wr_row_synced;
    integer gi_r;
    always @(*) begin
        wr_row_synced = wr_row_gray_s2;
        for (gi_r = 1; gi_r < 16; gi_r = gi_r + 1)
            wr_row_synced = wr_row_synced ^ (wr_row_gray_s2 >> gi_r);
    end

    // ------------------------------------------------------------------
    // N_LINES independent single-line RAMs, muxed by wr_slot / the read
    // side's claimed slot.
    // ------------------------------------------------------------------
    wire [23:0] slot_rd_data [0:N_LINES-1];

    genvar gv;
    generate
        for (gv = 0; gv < N_LINES; gv = gv + 1) begin : G_SLOT
            dp_line_ram #(.DEPTH(WIDTH), .DATA_W(24), .ADDR_W(ADDR_W)) u_ram (
                .wr_clk  (cam_pclk),
                .wr_en   (wr_en_this_pixel && (wr_slot == gv[SLOT_W-1:0])),
                .wr_addr (wr_addr_now),
                .wr_data (cam_rgb),

                .rd_clk  (out_pclk),
                .rd_en   (1'b1), // always sample; the claim logic below gates whether it's *used*
                .rd_addr (out_x[ADDR_W-1:0]),
                .rd_data (slot_rd_data[gv])
            );
        end
    endgenerate

    // ------------------------------------------------------------------
    // Read side: claim rows strictly in FIFO order, once per display line.
    // ------------------------------------------------------------------
    reg out_de_prev;
    wire out_row_start = out_de && !out_de_prev; // video_timing_gen drops
                                                   // `de` during h-blank on
                                                   // every line -- this
                                                   // rising edge fires at
                                                   // every line boundary.

    reg [SLOT_W-1:0] rd_slot_pending;  // slot decided at the most recent claim
    reg [SLOT_W-1:0] rd_slot_latched;  // rd_slot_pending, delayed one more
                                         // cycle so its transition lines up
                                         // exactly with dp_line_ram's own
                                         // one-cycle read latency -- without
                                         // this, the very first pixel of
                                         // each claimed row would briefly
                                         // mux in the previous row's still-
                                         // stale RAM output.

    always @(posedge out_pclk) begin
        if (out_rst) begin
            out_de_prev      <= 1'b0;
            rd_pop_count     <= 16'd0;
            rd_slot_pending  <= {SLOT_W{1'b0}};
            rd_slot_latched  <= {SLOT_W{1'b0}};
        end else begin
            out_de_prev <= out_de;

            if (out_row_start && (wr_row_synced > rd_pop_count)) begin
                // A new row has been fully written and not yet claimed --
                // claim the oldest unclaimed one, strictly in order.
                rd_slot_pending <= rd_pop_count[SLOT_W-1:0];
                rd_pop_count    <= rd_pop_count + 16'd1;
            end
            // else (no new row ready at this line boundary): keep
            // rd_slot_pending unchanged -- repeats the last claimed row
            // for this display line, the graceful whole-line-repeat
            // degradation described in the header comment.

            rd_slot_latched <= rd_slot_pending; // one more cycle of delay, always
        end
    end

    always @(posedge out_pclk) begin
        if (out_rst) begin
            out_rgb   <= 24'h0;
            out_ready <= 1'b0;
        end else begin
            if (out_de && (rd_pop_count > 16'd0))
                out_rgb <= slot_rd_data[rd_slot_latched];
            // else: hold the previous value -- either blanking (doesn't
            // matter what's shown, gated by `de` further downstream), or
            // no row has ever been claimed yet, in which case out_rgb
            // must NOT pick up whatever happens to be sitting in a never-
            // written block RAM slot.
            out_ready <= (rd_pop_count > 16'd0);
        end
    end

endmodule

`default_nettype wire
