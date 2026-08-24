// ============================================================================
// video_timing_gen.v -- CEA-861 video timing generator
// ----------------------------------------------------------------------------
// Free-running H/V counter generating hsync, vsync, data-enable (de) and the
// active-video pixel coordinates for one CEA-861 timing mode, parametrized
// so a single module instance covers 720p60, 1080p60 and 1080p30 by
// instantiating it three different ways (see rtl/video_modes_pkg.vh).
//
//                     |<------------- H_TOTAL -------------->|
//   ACTIVE VIDEO ---->|<-FP->|<-SYNC->|<--------BP---------->|
//
// hsync/vsync polarity is parametrized (CEA-861 720p/1080p modes both use
// positive polarity, but the field is kept generic).
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module video_timing_gen #(
    parameter H_ACTIVE = 1280,
    parameter H_FRONT  = 110,
    parameter H_SYNC   = 40,
    parameter H_BACK   = 220,
    parameter V_ACTIVE = 720,
    parameter V_FRONT  = 5,
    parameter V_SYNC   = 5,
    parameter V_BACK   = 20,
    parameter HS_POL   = 1'b1,   // 1 = sync pulse is active-high
    parameter VS_POL   = 1'b1
) (
    input  wire        clk,      // pixel clock
    input  wire        rst,      // synchronous, active high

    output wire         hsync,
    output wire         vsync,
    output wire         de,       // active video (data enable)
    output wire         frame_start, // one-cycle pulse at start of active video, line 0
    output wire [15:0] x,        // 0..H_ACTIVE-1 during active video
    output wire [15:0] y         // 0..V_ACTIVE-1 during active video
);

    localparam H_TOTAL = H_ACTIVE + H_FRONT + H_SYNC + H_BACK;
    localparam V_TOTAL = V_ACTIVE + V_FRONT + V_SYNC + V_BACK;

    localparam H_SYNC_START = H_ACTIVE + H_FRONT;
    localparam H_SYNC_END   = H_SYNC_START + H_SYNC;
    localparam V_SYNC_START = V_ACTIVE + V_FRONT;
    localparam V_SYNC_END   = V_SYNC_START + V_SYNC;

    reg [15:0] hcnt = 0;
    reg [15:0] vcnt = 0;

    wire h_active = (hcnt < H_ACTIVE);
    wire v_active = (vcnt < V_ACTIVE);

    always @(posedge clk) begin
        if (rst) begin
            hcnt <= 0;
            vcnt <= 0;
        end else begin
            if (hcnt == H_TOTAL - 1) begin
                hcnt <= 0;
                vcnt <= (vcnt == V_TOTAL - 1) ? 16'd0 : vcnt + 16'd1;
            end else begin
                hcnt <= hcnt + 16'd1;
            end
        end
    end

    wire hsync_active = (hcnt >= H_SYNC_START) && (hcnt < H_SYNC_END);
    wire vsync_active = (vcnt >= V_SYNC_START) && (vcnt < V_SYNC_END);
    wire frame_start_comb = (hcnt == 0) && (vcnt == 0);
    wire [15:0] x_comb = h_active ? hcnt : 16'd0;
    wire [15:0] y_comb = v_active ? vcnt : 16'd0;

    // One pipeline register stage between the (fairly deep, wide-comparator)
    // sync/active-video decode logic and this module's outputs. At 1080p60-
    // class pixel rates (148.5MHz, 6.7ns budget) the raw combinational chain
    // through these 16-bit magnitude comparisons plus whatever a downstream
    // consumer (e.g. test_pattern_gen) appends is too deep to close timing
    // on the LFE5U-25F-6 -- confirmed against real nextpnr-ecp5 STA. This
    // register caps this module's own worst-case output path length
    // regardless of what's downstream, at the cost of one clock of latency
    // (irrelevant for real-time video). All outputs are delayed together, so
    // relative alignment between hsync/vsync/de/x/y is preserved.
    reg hsync_r, vsync_r, de_r, frame_start_r;
    reg [15:0] x_r, y_r;

    always @(posedge clk) begin
        if (rst) begin
            hsync_r       <= ~HS_POL;
            vsync_r       <= ~VS_POL;
            de_r          <= 1'b0;
            frame_start_r <= 1'b0;
            x_r           <= 16'd0;
            y_r           <= 16'd0;
        end else begin
            hsync_r       <= hsync_active ? HS_POL : ~HS_POL;
            vsync_r       <= vsync_active ? VS_POL : ~VS_POL;
            de_r          <= h_active && v_active;
            frame_start_r <= frame_start_comb;
            x_r           <= x_comb;
            y_r           <= y_comb;
        end
    end

    assign hsync       = hsync_r;
    assign vsync       = vsync_r;
    assign de           = de_r;
    assign frame_start = frame_start_r;
    assign x             = x_r;
    assign y             = y_r;

endmodule

`default_nettype wire
