// ============================================================================
// fat16_reader.v -- minimal FAT16 volume reader: mounts a FAT16-formatted
// SD card (via sdcard_spi's single-block read interface), finds one file
// by its 8.3 name in the root directory, and streams the file's contents
// out one byte at a time, following the FAT cluster chain as needed.
// ----------------------------------------------------------------------------
// Scope, deliberately: FAT16 only (not FAT32 -- FAT32 has no fixed-size
// root directory and 32-bit cluster entries, meaningfully more logic for a
// bring-up demo), root directory only (no subdirectories), 8.3 short names
// only (no long-filename entries -- those are skipped, not parsed), and a
// single fixed target filename picked at synthesis time via a parameter.
// All BPB (BIOS Parameter Block) fields this reader depends on are parsed
// from the real boot sector at runtime rather than hardcoded, so it works
// with any spec-correct FAT16 volume, not just one particular layout --
// EXCEPT bytes-per-sector, which is required to be exactly 512 (checked;
// anything else is flagged as an error rather than silently mishandled).
// Because bytes-per-sector is fixed at 512, every division the FAT16 spec
// technically calls for (root-dir sector count, FAT sector/offset from a
// cluster number) reduces to a shift, so this reader needs no hardware
// divider.
//
// Most SD cards ship formatted FAT32 by default -- reformat as FAT16 (e.g.
// `mkfs.vfat -F 16 /dev/sdX1` on Linux, or the FAT16 option in Windows'
// format dialog on older/smaller cards) for this reader to mount it. See
// the sub-project README for the exact steps.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module fat16_reader #(
    parameter [8*11-1:0] FILENAME = "HELLO   TXT" // 8.3 name, space-padded, no dot
) (
    input  wire        clk,
    input  wire        rst,

    // ---- to sdcard_spi ----
    output reg          sd_read_req,
    output reg  [31:0]  sd_block_addr,
    input  wire          sd_rd_busy,
    input  wire          sd_rd_done,
    input  wire          sd_rd_error,
    input  wire          sd_byte_valid,
    input  wire [7:0]    sd_byte_data,
    input  wire [8:0]    sd_byte_index,
    input  wire          sd_ready,

    // ---- control ----
    input  wire        start,       // pulse (while sd_ready && !busy) to mount + stream
    input  wire        fifo_ready_for_block, // downstream FIFO has room for one more 512B block
    output reg          busy,
    output reg          done,        // one-cycle pulse: whole file streamed successfully
    output reg          error,       // sticky until the next `start`: mount or read failed
    output reg  [31:0]  file_size,

    // ---- file data stream ----
    output reg          data_valid,
    output reg  [7:0]   data_byte
);

    // ---- target 8.3 filename, indexable byte-by-byte via a variable part-select ----
    wire [8*11-1:0] fname_bytes = FILENAME;

    // ---- BPB fields parsed from the boot sector ----
    reg [15:0] bytes_per_sector;
    reg [7:0]  sectors_per_cluster;
    reg [15:0] reserved_sectors;
    reg [7:0]  num_fats;
    reg [15:0] root_entries;
    reg [15:0] sectors_per_fat;
    reg [15:0] boot_sig;

    // ---- derived volume layout (sectors, all 512B-sector units) ----
    reg [31:0] fat_start;
    reg [31:0] root_dir_start;
    reg [31:0] root_dir_sectors;
    reg [31:0] data_start;

    // ---- root-directory search state ----
    reg [7:0]  entry_buf [0:31];
    reg        entry_done_r;
    reg        found;
    reg [31:0] root_sector_i;
    reg [15:0] start_cluster;

    // ---- file streaming state ----
    reg [15:0] cur_cluster;
    reg [15:0] next_cluster;
    reg [31:0] bytes_remaining;
    reg [7:0]  cluster_sector_offset;
    reg [31:0] fat_sector;
    reg [8:0]  fat_entry_offset;

    integer k;
    reg      name_match;
    reg      attr_ok;

    localparam
        S_IDLE        = 4'd0,
        S_BOOT_REQ    = 4'd1,
        S_BOOT_WAIT   = 4'd2,
        S_BOOT_DONE   = 4'd3,
        S_ROOT_REQ    = 4'd4,
        S_ROOT_WAIT   = 4'd5,
        S_ROOT_DONE   = 4'd6,
        S_STREAM_SETUP= 4'd7,
        S_DATA_REQ    = 4'd8,
        S_DATA_WAIT   = 4'd9,
        S_FAT_REQ     = 4'd10,
        S_FAT_WAIT    = 4'd11,
        S_DONE        = 4'd12,
        S_ERROR       = 4'd13;

    reg [3:0] state;

    always @(posedge clk) begin
        sd_read_req <= 1'b0;
        done        <= 1'b0;
        data_valid  <= 1'b0;
        entry_done_r <= (state == S_ROOT_WAIT) && sd_byte_valid && (sd_byte_index[4:0] == 5'd31);

        if (rst) begin
            state <= S_IDLE;
            busy  <= 1'b0;
            error <= 1'b0;
        end else begin
            case (state)

                S_IDLE: begin
                    if (start && sd_ready && !busy) begin
                        busy  <= 1'b1;
                        error <= 1'b0;
                        state <= S_BOOT_REQ;
                    end
                end

                S_BOOT_REQ: begin
                    sd_block_addr <= 32'd0;
                    sd_read_req   <= 1'b1;
                    state <= S_BOOT_WAIT;
                end

                S_BOOT_WAIT: begin
                    if (sd_byte_valid) begin
                        case (sd_byte_index)
                            9'd11:  bytes_per_sector[7:0]   <= sd_byte_data;
                            9'd12:  bytes_per_sector[15:8]  <= sd_byte_data;
                            9'd13:  sectors_per_cluster      <= sd_byte_data;
                            9'd14:  reserved_sectors[7:0]    <= sd_byte_data;
                            9'd15:  reserved_sectors[15:8]   <= sd_byte_data;
                            9'd16:  num_fats                  <= sd_byte_data;
                            9'd17:  root_entries[7:0]         <= sd_byte_data;
                            9'd18:  root_entries[15:8]        <= sd_byte_data;
                            9'd22:  sectors_per_fat[7:0]      <= sd_byte_data;
                            9'd23:  sectors_per_fat[15:8]     <= sd_byte_data;
                            9'd510: boot_sig[7:0]             <= sd_byte_data;
                            9'd511: boot_sig[15:8]            <= sd_byte_data;
                            default: ;
                        endcase
                    end
                    if (sd_rd_done) begin
                        if (sd_rd_error) begin
                            state <= S_ERROR;
                        end else begin
                            state <= S_BOOT_DONE;
                        end
                    end
                end

                S_BOOT_DONE: begin
                    if (bytes_per_sector != 16'd512 || boot_sig != 16'hAA55 ||
                        num_fats == 8'd0 || sectors_per_cluster == 8'd0) begin
                        state <= S_ERROR; // not a 512B-sector FAT volume we understand
                    end else begin
                        fat_start        <= {16'd0, reserved_sectors};
                        root_dir_start   <= {16'd0, reserved_sectors} + num_fats * sectors_per_fat;
                        root_dir_sectors <= (root_entries + 16'd15) >> 4; // ceil(entries/16), 512B sectors
                        // data_start needs root_dir_start + root_dir_sectors, both computed
                        // this same cycle above -- finish the add next cycle once they're settled.
                        root_sector_i <= 32'd0;
                        found         <= 1'b0;
                        state         <= S_ROOT_REQ;
                    end
                end

                S_ROOT_REQ: begin
                    // data_start = root_dir_start + root_dir_sectors (both valid by now)
                    data_start    <= root_dir_start + root_dir_sectors;
                    sd_block_addr <= root_dir_start + root_sector_i;
                    sd_read_req   <= 1'b1;
                    state         <= S_ROOT_WAIT;
                end

                S_ROOT_WAIT: begin
                    if (sd_byte_valid) begin
                        entry_buf[sd_byte_index[4:0]] <= sd_byte_data;
                    end

                    if (entry_done_r && !found) begin
                        name_match = 1'b1;
                        for (k = 0; k < 11; k = k + 1)
                            if (entry_buf[k] != fname_bytes[8*(11-k)-1 -: 8])
                                name_match = 1'b0;
                        attr_ok = (entry_buf[0] != 8'h00) && (entry_buf[0] != 8'hE5) &&
                                  (entry_buf[11] != 8'h0F) && !(entry_buf[11][3]);

                        if (name_match && attr_ok) begin
                            found         <= 1'b1;
                            start_cluster <= {entry_buf[27], entry_buf[26]};
                            file_size     <= {entry_buf[31], entry_buf[30], entry_buf[29], entry_buf[28]};
                        end
                    end

                    if (sd_rd_done) begin
                        if (sd_rd_error) begin
                            state <= S_ERROR;
                        end else begin
                            state <= S_ROOT_DONE;
                        end
                    end
                end

                S_ROOT_DONE: begin
                    if (found) begin
                        state <= S_STREAM_SETUP;
                    end else if (root_sector_i == root_dir_sectors - 1'b1) begin
                        state <= S_ERROR; // whole root directory searched, file not found
                    end else begin
                        root_sector_i <= root_sector_i + 1'b1;
                        state <= S_ROOT_REQ;
                    end
                end

                S_STREAM_SETUP: begin
                    if (start_cluster < 16'd2) begin
                        state <= S_ERROR; // zero-length or corrupt directory entry
                    end else begin
                        cur_cluster           <= start_cluster;
                        cluster_sector_offset <= 8'd0;
                        bytes_remaining       <= file_size;
                        state <= S_DATA_REQ;
                    end
                end

                S_DATA_REQ: begin
                    if (bytes_remaining == 32'd0) begin
                        state <= S_DONE;
                    end else if (fifo_ready_for_block) begin
                        sd_block_addr <= data_start + (cur_cluster - 16'd2) * sectors_per_cluster
                                         + cluster_sector_offset;
                        sd_read_req   <= 1'b1;
                        state         <= S_DATA_WAIT;
                    end
                    // else: stay here, waiting for the downstream FIFO to drain --
                    // this is the design's only back-pressure point, sized so one
                    // full 512-byte block always fits once permission is granted.
                end

                S_DATA_WAIT: begin
                    if (sd_byte_valid && bytes_remaining != 32'd0) begin
                        data_byte       <= sd_byte_data;
                        data_valid      <= 1'b1;
                        bytes_remaining <= bytes_remaining - 1'b1;
                    end

                    if (sd_rd_done) begin
                        if (sd_rd_error) begin
                            state <= S_ERROR;
                        end else if (bytes_remaining == 32'd0) begin
                            state <= S_DONE;
                        end else if (cluster_sector_offset == sectors_per_cluster - 1'b1) begin
                            fat_sector       <= fat_start + {16'd0, cur_cluster[15:8]};
                            fat_entry_offset <= {cur_cluster[7:0], 1'b0};
                            state <= S_FAT_REQ;
                        end else begin
                            cluster_sector_offset <= cluster_sector_offset + 1'b1;
                            state <= S_DATA_REQ;
                        end
                    end
                end

                S_FAT_REQ: begin
                    sd_block_addr <= fat_sector;
                    sd_read_req   <= 1'b1;
                    state         <= S_FAT_WAIT;
                end

                S_FAT_WAIT: begin
                    if (sd_byte_valid) begin
                        if (sd_byte_index == fat_entry_offset)
                            next_cluster[7:0] <= sd_byte_data;
                        else if (sd_byte_index == fat_entry_offset + 1'b1)
                            next_cluster[15:8] <= sd_byte_data;
                    end
                    if (sd_rd_done) begin
                        if (sd_rd_error) begin
                            state <= S_ERROR;
                        end else if (next_cluster >= 16'hFFF8) begin
                            state <= S_ERROR; // FAT says EOF but file_size claims more data
                        end else begin
                            cur_cluster           <= next_cluster;
                            cluster_sector_offset <= 8'd0;
                            state <= S_DATA_REQ;
                        end
                    end
                end

                S_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= S_IDLE;
                end

                S_ERROR: begin
                    error <= 1'b1;
                    busy  <= 1'b0;
                    state <= S_IDLE;
                end

                default: state <= S_ERROR;
            endcase
        end
    end

endmodule

`default_nettype wire
