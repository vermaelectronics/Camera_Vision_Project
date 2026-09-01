`timescale 1ns/1ps
// img_loader : drives the SD/FAT bring-up sequence, then streams
// IMAGE.RAW's contents into the framebuffer, sys_clk domain.
//
// IMAGE.RAW format (see tools/convert_image.py for the writer side):
//   offset 0..3 : magic "RIMG" (ASCII)
//   offset 4..5 : width,  16-bit little-endian (must be 160)
//   offset 6..7 : height, 16-bit little-endian (must be 120)
//   offset 8..  : 160*120 pixels, RGB565, 16-bit little-endian each,
//                 row-major, top row first, left pixel first.
// Total file size = 8 + 160*120*2 = 38408 bytes.
//
// This is a deliberately simple, custom, uncompressed format - real
// image formats (JPEG/PNG/GIF) require entropy decoding far beyond
// what's reasonable to implement in FPGA fabric for this project; BMP
// was also considered but its variable header size, bottom-up row
// order and per-row padding add real parsing complexity for no benefit
// over just defining a trivial fixed format and shipping a converter
// script alongside it.
//
// Runs once at power-on/reset (not once per displayed frame): the
// framebuffer holds the whole image, so after `loaded` goes high,
// hdmi_out.v scans it out continuously with no further SD activity.
module img_loader (
    input  wire       clk,          // sys_clk, 20 MHz
    input  wire       rst,

    input  wire       init_done,
    input  wire       init_ok,
    output reg        fat_start,

    input  wire       mount_done,
    input  wire       mount_ok,
    input  wire       file_found,
    input  wire       file_done,
    input  wire       fat_error,
    input  wire [7:0] file_data,
    input  wire       file_data_valid,

    output reg         fb_wr_en,
    output reg  [14:0] fb_wr_addr,
    output reg  [15:0] fb_wr_data,

    output reg         loaded,   // sticky: full valid image is in the framebuffer
    output reg         error,    // sticky: init/mount/search/format/size error
    output reg         busy      // 1 while anything is still in progress
);
    localparam S_IDLE         = 4'd0,
               S_WAIT_MOUNT   = 4'd1,
               S_WAIT_FOUND   = 4'd2,
               S_HEADER       = 4'd3,
               S_PIXELS_LO    = 4'd4,
               S_PIXELS_HI    = 4'd5,
               S_DONE         = 4'd6,
               S_ERROR        = 4'd7;

    reg [3:0]  state;
    reg [2:0]  hdr_idx;      // 0..7
    reg        hdr_bad;
    reg [7:0]  pix_lo_byte;
    reg [14:0] pix_count;    // 0..19200

    function [7:0] magic_byte;
        input [1:0] idx;
        begin
            case (idx)
                2'd0: magic_byte = "R";
                2'd1: magic_byte = "I";
                2'd2: magic_byte = "M";
                default: magic_byte = "G";
            endcase
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            state      <= S_IDLE;
            fat_start  <= 1'b0;
            hdr_idx    <= 3'd0;
            hdr_bad    <= 1'b0;
            pix_count  <= 15'd0;
            fb_wr_en   <= 1'b0;
            fb_wr_addr <= 15'd0;
            fb_wr_data <= 16'd0;
            loaded     <= 1'b0;
            error      <= 1'b0;
            busy       <= 1'b0;
        end else begin
            fat_start <= 1'b0;
            fb_wr_en  <= 1'b0;

            if (loaded || error) begin
                // terminal - hold latched outputs until reset
            end else if (fat_error && state != S_IDLE) begin
                error <= 1'b1;
                busy  <= 1'b0;
            end else if (file_done && state != S_DONE && state != S_IDLE) begin
                // fat_reader finished streaming before the expected
                // 19200th pixel was consumed - undersized/malformed
                // file. Checked here (against the pre-transition state,
                // like the fat_error check above) rather than as a
                // trailing check after the case block below, so it
                // can't race against S_PIXELS_HI's own same-cycle
                // transition into S_DONE on the file's last byte.
                error <= 1'b1;
                busy  <= 1'b0;
            end else begin
                busy <= 1'b1;
                case (state)
                    S_IDLE: begin
                        busy <= 1'b0;
                        if (init_done) begin
                            if (init_ok) begin
                                fat_start <= 1'b1;
                                state     <= S_WAIT_MOUNT;
                            end else begin
                                error <= 1'b1;
                            end
                        end
                    end

                    S_WAIT_MOUNT: begin
                        if (mount_done) begin
                            if (mount_ok)
                                state <= S_WAIT_FOUND;
                            else
                                error <= 1'b1;
                        end
                    end

                    S_WAIT_FOUND: begin
                        if (file_found) begin
                            hdr_idx   <= 3'd0;
                            hdr_bad   <= 1'b0;
                            pix_count <= 15'd0;
                            state     <= S_HEADER;
                        end
                        // fat_error (file not found) is caught by the
                        // global check above.
                    end

                    S_HEADER: begin
                        if (file_data_valid) begin
                            case (hdr_idx)
                                3'd0, 3'd1, 3'd2, 3'd3:
                                    if (file_data != magic_byte(hdr_idx[1:0])) hdr_bad <= 1'b1;
                                3'd4: if (file_data != 8'd160) hdr_bad <= 1'b1; // width low byte
                                3'd5: if (file_data != 8'd0)   hdr_bad <= 1'b1; // width high byte
                                3'd6: if (file_data != 8'd120) hdr_bad <= 1'b1; // height low byte
                                3'd7: if (file_data != 8'd0)   hdr_bad <= 1'b1; // height high byte
                                default: ;
                            endcase
                            if (hdr_idx == 3'd7) begin
                                state <= hdr_bad ? S_ERROR : S_PIXELS_LO;
                            end else begin
                                hdr_idx <= hdr_idx + 3'd1;
                            end
                        end
                    end

                    S_PIXELS_LO: begin
                        if (file_data_valid) begin
                            pix_lo_byte <= file_data;
                            state       <= S_PIXELS_HI;
                        end
                    end

                    // fb_wr_addr is driven from pix_count's value as it
                    // stands BEFORE this cycle's increment below (plain
                    // nonblocking-assignment semantics: the RHS of both
                    // statements reads pix_count's old value) - i.e.
                    // "the address for the pixel currently being
                    // written". A separate, independently-incremented
                    // address register here would run one step ahead of
                    // fb_wr_data (both driven from the same cycle, but
                    // the address for the NEXT write instead of this
                    // one) - exactly the off-by-one this design had
                    // before switching to deriving the address from
                    // pix_count directly.
                    S_PIXELS_HI: begin
                        if (file_data_valid) begin
                            fb_wr_en   <= 1'b1;
                            fb_wr_addr <= pix_count[14:0];
                            fb_wr_data <= {file_data, pix_lo_byte};
                            pix_count  <= pix_count + 15'd1;
                            if (pix_count == 15'd19199) begin
                                state <= S_DONE;
                            end else begin
                                state <= S_PIXELS_LO;
                            end
                        end
                    end

                    S_DONE: begin
                        if (file_done) begin
                            loaded <= 1'b1;
                            busy   <= 1'b0;
                        end
                        // Any trailing bytes fat_reader streams after the
                        // 19200th pixel (an oversized file) are simply
                        // ignored here until file_done arrives - harmless.
                    end

                    default: error <= 1'b1;
                endcase
            end
        end
    end

endmodule
