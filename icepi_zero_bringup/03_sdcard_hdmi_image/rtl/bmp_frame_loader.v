// ============================================================================
// bmp_frame_loader.v -- parses a 24bpp uncompressed BMP file's header and
// streams its pixel data into a frame buffer, converting BGR888 -> RGB565
// as it goes.
// ----------------------------------------------------------------------------
// Scope, deliberately: exactly IMG_W x IMG_H pixels, exactly 24 bits/pixel,
// exactly BI_RGB (uncompressed, compression field = 0), exactly a standard
// 40-byte BITMAPINFOHEADER, and bottom-up row order (positive height field
// -- the overwhelmingly common case; top-down BMPs, compression, and other
// bit depths are rejected rather than silently mishandled). At 24bpp with
// IMG_W*3 already a multiple of 4, BMP's row-padding-to-4-bytes rule adds
// no padding here, so row length is a plain constant -- no runtime padding
// math needed.
//
// Consumes fat16_reader's data_valid/data_byte stream directly (same byte
// order the file is stored in) -- the BMP format doesn't care what found
// the bytes, so fat16_reader.v is reused completely unmodified from the
// sibling 02_sdcard_text_reader sub-project.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module bmp_frame_loader #(
    parameter integer IMG_W = 160,
    parameter integer IMG_H = 120
) (
    input  wire        clk,
    input  wire        rst,

    // ---- from fat16_reader ----
    input  wire        data_valid,
    input  wire [7:0]  data_byte,
    input  wire        file_done,   // fat16_reader's `done`, one-cycle pulse
    input  wire        file_error,  // fat16_reader's `error`

    // ---- status ----
    output reg          busy,
    output reg          done,   // one-cycle pulse: full frame loaded successfully
    output reg          error,  // sticky until the next load starts

    // ---- to the frame buffer (dp_line_ram, this module's clk domain) ----
    output reg           fb_wr_en,
    output reg  [14:0]   fb_wr_addr,  // 0 .. IMG_W*IMG_H-1
    output reg  [15:0]   fb_wr_data   // RGB565
);

    localparam integer ROW_BYTES  = IMG_W * 3; // no padding: IMG_W*3 already a multiple of 4
    localparam integer PIXEL_COUNT = IMG_W * IMG_H;

    localparam
        S_IDLE   = 3'd0,
        S_HEADER = 3'd1,
        S_SKIP   = 3'd2,
        S_PIXELS = 3'd3,
        S_DONE   = 3'd4,
        S_ERROR  = 3'd5;

    reg [2:0] state;

    reg [31:0] byte_count;
    reg [15:0] signature;
    reg [31:0] px_offset;
    reg [31:0] dib_size;
    reg [31:0] width_f;
    reg [31:0] height_f;
    reg [15:0] planes_f;
    reg [15:0] bpp_f;
    reg [31:0] compression_f;

    reg [1:0]  pix_byte_idx; // 0=B, 1=G, 2=R (BMP stores BGR)
    reg [7:0]  pix_b, pix_g;
    reg [15:0] col;
    reg [15:0] row;

    always @(posedge clk) begin
        fb_wr_en <= 1'b0;
        done     <= 1'b0;

        if (rst) begin
            state      <= S_IDLE;
            busy       <= 1'b0;
            error      <= 1'b0;
            byte_count <= 32'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    // caller starts us implicitly by starting fat16_reader;
                    // we just track byte_count from the first data_valid.
                    if (data_valid) begin
                        busy       <= 1'b1;
                        error      <= 1'b0;
                        byte_count <= 32'd1;
                        signature[15:8] <= data_byte;
                        state      <= S_HEADER;
                    end
                    if (file_error) begin
                        error <= 1'b1;
                    end
                end

                S_HEADER: begin
                    if (data_valid) begin
                        case (byte_count)
                            32'd1:  signature[7:0]      <= data_byte;
                            32'd10: px_offset[7:0]       <= data_byte;
                            32'd11: px_offset[15:8]      <= data_byte;
                            32'd12: px_offset[23:16]     <= data_byte;
                            32'd13: px_offset[31:24]     <= data_byte;
                            32'd14: dib_size[7:0]        <= data_byte;
                            32'd15: dib_size[15:8]       <= data_byte;
                            32'd16: dib_size[23:16]      <= data_byte;
                            32'd17: dib_size[31:24]      <= data_byte;
                            32'd18: width_f[7:0]         <= data_byte;
                            32'd19: width_f[15:8]        <= data_byte;
                            32'd20: width_f[23:16]       <= data_byte;
                            32'd21: width_f[31:24]       <= data_byte;
                            32'd22: height_f[7:0]        <= data_byte;
                            32'd23: height_f[15:8]       <= data_byte;
                            32'd24: height_f[23:16]      <= data_byte;
                            32'd25: height_f[31:24]      <= data_byte;
                            32'd26: planes_f[7:0]        <= data_byte;
                            32'd27: planes_f[15:8]       <= data_byte;
                            32'd28: bpp_f[7:0]           <= data_byte;
                            32'd29: bpp_f[15:8]          <= data_byte;
                            32'd30: compression_f[7:0]   <= data_byte;
                            32'd31: compression_f[15:8]  <= data_byte;
                            32'd32: compression_f[23:16] <= data_byte;
                            32'd33: compression_f[31:24] <= data_byte;
                            default: ;
                        endcase
                        byte_count <= byte_count + 1'b1;

                        if (byte_count == 32'd34) begin
                            // all fields captured as of the byte that just arrived
                            // (byte index 33) -- validate now.
                            if (signature != 16'h424D ||         // 'B','M'
                                dib_size  != 32'd40 ||            // BITMAPINFOHEADER
                                width_f   != IMG_W ||
                                height_f  != IMG_H ||              // positive = bottom-up
                                planes_f  != 16'd1 ||
                                bpp_f     != 16'd24 ||
                                compression_f != 32'd0) begin
                                state <= S_ERROR;
                            end else begin
                                col          <= 16'd0;
                                row          <= IMG_H - 1; // bottom-up: first file row -> bottom
                                pix_byte_idx <= 2'd0;
                                state        <= S_SKIP;
                            end
                        end
                    end
                    if (file_error) state <= S_ERROR;
                end

                // consume any remaining header/color-table bytes up to px_offset
                S_SKIP: begin
                    if (data_valid) begin
                        byte_count <= byte_count + 1'b1;
                        if (byte_count + 1'b1 == px_offset) begin
                            state <= S_PIXELS;
                        end
                    end
                    if (file_error) state <= S_ERROR;
                end

                S_PIXELS: begin
                    if (data_valid) begin
                        case (pix_byte_idx)
                            2'd0: begin pix_b <= data_byte; pix_byte_idx <= 2'd1; end
                            2'd1: begin pix_g <= data_byte; pix_byte_idx <= 2'd2; end
                            2'd2: begin
                                fb_wr_en   <= 1'b1;
                                fb_wr_addr <= row * IMG_W + col;
                                fb_wr_data <= {data_byte[7:3], pix_g[7:2], pix_b[7:3]};
                                pix_byte_idx <= 2'd0;

                                if (col == IMG_W - 1) begin
                                    col <= 16'd0;
                                    if (row == 16'd0) begin
                                        state <= S_DONE;
                                    end else begin
                                        row <= row - 1'b1;
                                    end
                                end else begin
                                    col <= col + 1'b1;
                                end
                            end
                        endcase
                    end
                    if (file_error) state <= S_ERROR;
                end

                S_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= S_IDLE;
                    byte_count <= 32'd0;
                end

                S_ERROR: begin
                    error <= 1'b1;
                    busy  <= 1'b0;
                    state <= S_IDLE;
                    byte_count <= 32'd0;
                end

                default: state <= S_ERROR;
            endcase
        end
    end

endmodule

`default_nettype wire
