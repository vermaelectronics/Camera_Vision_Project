`timescale 1ns/1ps
// fat_reader : FAT16/FAT32 root-directory search + file streaming.
//
// No BPB field is assumed fixed: bytes-per-sector, sectors-per-cluster,
// FAT size, number of FATs and (for FAT32) the root cluster are all
// read from the actual boot sector on the card. Sector 0 is inspected
// for both an MBR partition table and a superfloppy (partition-less)
// VBR, and whichever is actually present is followed.
//
// Only 8.3 short-name directory entries are matched (LFN entries have
// attr==0x0F and are skipped); deleted (0xE5) and volume-label entries
// never match "IMAGE   RAW" and are naturally skipped by the compare.
//
// BytesPerSec is required to be 512: SD/SDHC/SDXC CMD17 always
// addresses fixed 512-byte physical blocks, so any FAT volume actually
// usable through sd_block_read has a 512-byte logical sector too (this
// is mandated by the SD File System Specification for SDHC/SDXC). The
// value is still read from the card and checked, never hard-coded in.
module fat_reader (
    input  wire        clk,        // sys_clk, 20 MHz
    input  wire        rst,
    input  wire        start,      // pulse: begin mount + search + read

    // sd_block_read request/response (point-to-point, no arbitration
    // needed: fat_reader is the only client of sd_block_read)
    output reg         blk_start,
    output reg  [31:0] blk_lba,
    input  wire        blk_done,
    input  wire        blk_error,
    input  wire        blk_data_valid,
    input  wire [7:0]  blk_data_in,

    output reg         mount_done,
    output reg         mount_ok,
    output reg         file_found,
    output reg         file_done,
    output reg         error,

    output reg         data_valid, // file content byte stream, out to msg_ctrl
    output reg  [7:0]  data_out
);
    localparam
        P_IDLE            = 4'd0,
        P_READ_BOOT_WAIT  = 4'd1,
        P_CHECK_SIG       = 4'd2,
        P_READ_VBR_WAIT   = 4'd3,
        P_CALC_FATAREA    = 4'd4,
        P_FINISH_BPB      = 4'd5,
        P_ROOT_INIT       = 4'd6,
        P_ROOT_READ       = 4'd7,
        P_ROOT_READ_WAIT  = 4'd8,
        P_ROOT_NEXT       = 4'd9,
        P_ROOT_NOTFOUND   = 4'd10,
        P_FILE_INIT       = 4'd11,
        P_FILE_READ       = 4'd12,
        P_FILE_READ_WAIT  = 4'd13,
        P_FAT_LOOKUP       = 4'd14,
        P_FAT_LOOKUP_WAIT  = 4'd15;
    // There is no separate P_ERROR/P_DONE state: completion and error
    // are both terminal conditions captured by the `halted` flag below,
    // which freezes `state` and holds the latched status outputs.

    reg [3:0]  state;
    reg [9:0]  sbyte;          // byte offset within current 512B sector
    reg        halted;         // true once P_DONE/P_ERROR reached; freezes state

    // ---- BPB fields (read from the card, never assumed) ------------
    reg [15:0] bytes_per_sec;
    reg [7:0]  sec_per_clus;
    reg [15:0] rsvd_sec_cnt;
    reg [7:0]  num_fats;
    reg [15:0] root_ent_cnt;
    reg [15:0] fat_sz16;
    reg [31:0] fat_sz32;
    reg [31:0] root_clus;
    reg        is_fat32;

    reg [7:0]  mbr_part_type;
    reg [31:0] mbr_part_lba;

    reg [31:0] part_start;
    reg [31:0] fat_start;
    reg [31:0] data_start;
    reg [31:0] root_dir_start;   // FAT16 only
    reg [15:0] root_dir_sectors; // FAT16 only
    reg [3:0]  bps_shift;
    reg [3:0]  spc_shift;

    reg [31:0] fat_area_acc;
    reg [15:0] fat_area_i;

    // ---- root-dir / cluster-chain walk state ------------------------
    reg [31:0] cur_cluster;
    reg [31:0] cur_root_sector;
    reg [15:0] root_sector_remaining;
    reg [7:0]  sec_in_clus;
    reg        fat_lookup_for_file;

    // ---- directory entry parsing ------------------------------------
    reg        entry_match_ok;
    reg [7:0]  entry_first_byte;
    reg [7:0]  entry_attr;
    reg [15:0] entry_clus_hi, entry_clus_lo;
    reg [31:0] entry_size;
    reg        root_end_reached, match_found;
    reg [15:0] found_clus_hi, found_clus_lo;
    reg [31:0] found_size;

    // ---- FAT entry lookup --------------------------------------------
    reg [31:0] next_cluster_raw;

    // ---- file streaming -----------------------------------------------
    reg [31:0] bytes_remaining;

    function [3:0] clog2_16;
        input [15:0] v;
        integer i;
        begin
            clog2_16 = 4'd0;
            for (i = 0; i < 16; i = i + 1)
                if (v[i]) clog2_16 = i[3:0];
        end
    endfunction

    function [3:0] clog2_8;
        input [7:0] v;
        integer i;
        begin
            clog2_8 = 4'd0;
            for (i = 0; i < 8; i = i + 1)
                if (v[i]) clog2_8 = i[3:0];
        end
    endfunction

    // Target file is IMAGE.RAW (8.3 short name "IMAGE   RAW") in this
    // project, rather than sd_uart_top's TEST.TXT.
    function [7:0] target_name_byte;
        input [3:0] idx;
        begin
            case (idx)
                4'd0:  target_name_byte = "I";
                4'd1:  target_name_byte = "M";
                4'd2:  target_name_byte = "A";
                4'd3:  target_name_byte = "G";
                4'd4:  target_name_byte = "E";
                4'd5:  target_name_byte = " ";
                4'd6:  target_name_byte = " ";
                4'd7:  target_name_byte = " ";
                4'd8:  target_name_byte = "R";
                4'd9:  target_name_byte = "A";
                4'd10: target_name_byte = "W";
                default: target_name_byte = " ";
            endcase
        end
    endfunction

    // Capture the BPB fields present at a fixed set of byte offsets in
    // the current sector; called for both the boot sector (sector 0)
    // and, when an MBR is present, the real VBR it points to.
    task capture_bpb_byte;
        input [9:0] off;
        input [7:0] d;
        begin
            case (off)
                10'h0B: bytes_per_sec[7:0]   <= d;
                10'h0C: bytes_per_sec[15:8]  <= d;
                10'h0D: sec_per_clus         <= d;
                10'h0E: rsvd_sec_cnt[7:0]    <= d;
                10'h0F: rsvd_sec_cnt[15:8]   <= d;
                10'h10: num_fats             <= d;
                10'h11: root_ent_cnt[7:0]    <= d;
                10'h12: root_ent_cnt[15:8]   <= d;
                10'h16: fat_sz16[7:0]        <= d;
                10'h17: fat_sz16[15:8]       <= d;
                10'h24: fat_sz32[7:0]        <= d;
                10'h25: fat_sz32[15:8]       <= d;
                10'h26: fat_sz32[23:16]      <= d;
                10'h27: fat_sz32[31:24]      <= d;
                10'h2C: root_clus[7:0]       <= d;
                10'h2D: root_clus[15:8]      <= d;
                10'h2E: root_clus[23:16]     <= d;
                10'h2F: root_clus[31:24]     <= d;
                default: ;
            endcase
        end
    endtask

    // Parse one byte of a 32-byte directory entry (16 entries/sector).
    task parse_dirent_byte;
        input [9:0] off;
        input [7:0] d;
        reg   [4:0] eoff;
        begin
            eoff = off[4:0];
            case (eoff)
                5'd0:  begin entry_first_byte <= d; entry_match_ok <= (d == target_name_byte(0)); end
                5'd1:  entry_match_ok <= entry_match_ok & (d == target_name_byte(1));
                5'd2:  entry_match_ok <= entry_match_ok & (d == target_name_byte(2));
                5'd3:  entry_match_ok <= entry_match_ok & (d == target_name_byte(3));
                5'd4:  entry_match_ok <= entry_match_ok & (d == target_name_byte(4));
                5'd5:  entry_match_ok <= entry_match_ok & (d == target_name_byte(5));
                5'd6:  entry_match_ok <= entry_match_ok & (d == target_name_byte(6));
                5'd7:  entry_match_ok <= entry_match_ok & (d == target_name_byte(7));
                5'd8:  entry_match_ok <= entry_match_ok & (d == target_name_byte(8));
                5'd9:  entry_match_ok <= entry_match_ok & (d == target_name_byte(9));
                5'd10: entry_match_ok <= entry_match_ok & (d == target_name_byte(10));
                5'd11: entry_attr          <= d;
                5'd20: entry_clus_hi[7:0]  <= d;
                5'd21: entry_clus_hi[15:8] <= d;
                5'd26: entry_clus_lo[7:0]  <= d;
                5'd27: entry_clus_lo[15:8] <= d;
                5'd28: entry_size[7:0]     <= d;
                5'd29: entry_size[15:8]    <= d;
                5'd30: entry_size[23:16]   <= d;
                5'd31: entry_size[31:24]   <= d;
                default: ;
            endcase
            if (eoff == 5'd31) begin
                if (entry_first_byte == 8'h00)
                    root_end_reached <= 1'b1;
                else if (entry_match_ok && entry_attr != 8'h0F && !entry_attr[3]) begin
                    // Snapshot the matching entry's fields right here,
                    // not in P_ROOT_NEXT (once per sector): later
                    // entries in the same 512-byte sector overwrite
                    // entry_clus_hi/lo/size, so waiting until the
                    // whole sector finishes would capture whichever
                    // entry happened to be parsed last, not this one.
                    // entry_size[31:24] itself hasn't taken this
                    // cycle's write yet (nonblocking), so build the
                    // final byte in directly rather than reading it
                    // from entry_size.
                    match_found   <= 1'b1;
                    found_clus_hi <= entry_clus_hi;
                    found_clus_lo <= entry_clus_lo;
                    found_size    <= {d, entry_size[23:0]};
                end
            end
        end
    endtask

    // Capture the 2 (FAT16) or 4 (FAT32) little-endian bytes of the FAT
    // entry located at fat_off within the current FAT sector.
    task capture_fat_entry_byte;
        input [9:0] off;
        input [9:0] fat_off;
        input       fat32;
        input [7:0] d;
        reg   [9:0] rel;
        begin
            if (off >= fat_off && off < fat_off + (fat32 ? 10'd4 : 10'd2)) begin
                rel = off - fat_off;
                case (rel)
                    10'd0: next_cluster_raw[7:0]   <= d;
                    10'd1: next_cluster_raw[15:8]  <= d;
                    10'd2: next_cluster_raw[23:16] <= d;
                    10'd3: next_cluster_raw[31:24] <= d;
                    default: ;
                endcase
            end
        end
    endtask

    wire [31:0] clus_sector   = data_start + ((cur_cluster - 32'd2) << spc_shift) + {24'd0, sec_in_clus};
    wire [31:0] fat_byte_off_w = is_fat32 ? (cur_cluster << 2) : (cur_cluster << 1);
    wire [31:0] fat_sector_w   = fat_start + (fat_byte_off_w >> bps_shift);
    wire [9:0]  fat_off_w      = fat_byte_off_w[9:0] & (bytes_per_sec[9:0] - 10'd1);

    always @(posedge clk) begin
        if (rst) begin
            state       <= P_IDLE;
            blk_start   <= 1'b0;
            blk_lba     <= 32'd0;
            mount_done  <= 1'b0;
            mount_ok    <= 1'b0;
            file_found  <= 1'b0;
            file_done   <= 1'b0;
            error       <= 1'b0;
            data_valid  <= 1'b0;
            data_out    <= 8'd0;
            sbyte       <= 10'd0;
            halted      <= 1'b0;
        end else begin
            blk_start  <= 1'b0;
            data_valid <= 1'b0;

            if (halted) begin
                // terminal - hold latched status outputs until reset
            end else if (blk_error &&
                (state == P_READ_BOOT_WAIT || state == P_READ_VBR_WAIT ||
                 state == P_ROOT_READ_WAIT || state == P_FILE_READ_WAIT ||
                 state == P_FAT_LOOKUP_WAIT)) begin
                error  <= 1'b1;
                halted <= 1'b1;
            end else begin
                case (state)
                    P_IDLE: begin
                        if (start) begin
                            blk_start <= 1'b1;
                            blk_lba   <= 32'd0;
                            sbyte     <= 10'd0;
                            state     <= P_READ_BOOT_WAIT;
                        end
                    end

                    // Capture both BPB-as-VBR and MBR partition-0
                    // fields from sector 0; P_CHECK_SIG decides which
                    // interpretation is actually present.
                    P_READ_BOOT_WAIT: begin
                        if (blk_data_valid) begin
                            capture_bpb_byte(sbyte, blk_data_in);
                            case (sbyte)
                                10'h1C2: mbr_part_type       <= blk_data_in;
                                10'h1C6: mbr_part_lba[7:0]   <= blk_data_in;
                                10'h1C7: mbr_part_lba[15:8]  <= blk_data_in;
                                10'h1C8: mbr_part_lba[23:16] <= blk_data_in;
                                10'h1C9: mbr_part_lba[31:24] <= blk_data_in;
                                default: ;
                            endcase
                            sbyte <= sbyte + 10'd1;
                        end
                        if (blk_done)
                            state <= P_CHECK_SIG;
                    end

                    P_CHECK_SIG: begin
                        case (mbr_part_type)
                            8'h01, 8'h04, 8'h06, 8'h0B, 8'h0C, 8'h0E: begin
                                part_start <= mbr_part_lba;
                                blk_start  <= 1'b1;
                                blk_lba    <= mbr_part_lba;
                                sbyte      <= 10'd0;
                                state      <= P_READ_VBR_WAIT;
                            end
                            default: begin
                                part_start <= 32'd0; // sector 0 IS the VBR
                                state      <= P_CALC_FATAREA;
                            end
                        endcase
                    end

                    P_READ_VBR_WAIT: begin
                        if (blk_data_valid) begin
                            capture_bpb_byte(sbyte, blk_data_in);
                            sbyte <= sbyte + 10'd1;
                        end
                        if (blk_done)
                            state <= P_CALC_FATAREA;
                    end

                    P_CALC_FATAREA: begin
                        if (bytes_per_sec != 16'd512) begin
                            error  <= 1'b1; // not a supported FAT volume
                            halted <= 1'b1;
                        end else begin
                            is_fat32     <= (fat_sz16 == 16'd0);
                            fat_area_acc <= 32'd0;
                            fat_area_i   <= 16'd0;
                            state        <= P_FINISH_BPB;
                        end
                    end

                    // Sequentially accumulate num_fats * FATSz (FATSz16
                    // or FATSz32) without a hardware multiplier.
                    P_FINISH_BPB: begin
                        if (fat_area_i < {8'd0, num_fats}) begin
                            fat_area_acc <= fat_area_acc + (is_fat32 ? fat_sz32 : {16'd0, fat_sz16});
                            fat_area_i   <= fat_area_i + 16'd1;
                        end else begin
                            bps_shift <= clog2_16(bytes_per_sec);
                            spc_shift <= clog2_8(sec_per_clus);
                            fat_start <= part_start + rsvd_sec_cnt;
                            if (is_fat32) begin
                                data_start <= part_start + rsvd_sec_cnt + fat_area_acc;
                            end else begin
                                root_dir_start   <= part_start + rsvd_sec_cnt + fat_area_acc;
                                root_dir_sectors <= (({16'd0, root_ent_cnt} << 5) + (bytes_per_sec - 16'd1)) >> clog2_16(bytes_per_sec);
                                // data_start = root_dir_start + root_dir_sectors, finished in P_ROOT_INIT
                                data_start <= part_start + rsvd_sec_cnt + fat_area_acc;
                            end
                            mount_done <= 1'b1;
                            mount_ok   <= 1'b1;
                            state      <= P_ROOT_INIT;
                        end
                    end

                    // Only sets up registers; the actual sd_block_read
                    // request is issued from P_ROOT_READ one cycle later
                    // so cur_cluster/cur_root_sector are already updated
                    // when clus_sector/blk_lba are evaluated.
                    P_ROOT_INIT: begin
                        if (!is_fat32) begin
                            data_start            <= data_start + root_dir_sectors;
                            cur_root_sector       <= root_dir_start;
                            root_sector_remaining <= root_dir_sectors;
                        end else begin
                            cur_cluster <= root_clus;
                            sec_in_clus <= 8'd0;
                        end
                        root_end_reached <= 1'b0;
                        match_found      <= 1'b0;
                        state            <= P_ROOT_READ;
                    end

                    P_ROOT_READ: begin
                        blk_start <= 1'b1;
                        blk_lba   <= !is_fat32 ? cur_root_sector : clus_sector;
                        sbyte     <= 10'd0;
                        state     <= P_ROOT_READ_WAIT;
                    end

                    P_ROOT_READ_WAIT: begin
                        if (blk_data_valid) begin
                            parse_dirent_byte(sbyte, blk_data_in);
                            sbyte <= sbyte + 10'd1;
                        end
                        if (blk_done)
                            state <= P_ROOT_NEXT;
                    end

                    P_ROOT_NEXT: begin
                        if (match_found) begin
                            // found_clus_hi/lo/size were already
                            // snapshotted at match time in
                            // parse_dirent_byte - not re-read here.
                            state <= P_FILE_INIT;
                        end else if (root_end_reached) begin
                            state <= P_ROOT_NOTFOUND;
                        end else if (!is_fat32) begin
                            if (root_sector_remaining <= 16'd1) begin
                                state <= P_ROOT_NOTFOUND;
                            end else begin
                                cur_root_sector       <= cur_root_sector + 32'd1;
                                root_sector_remaining <= root_sector_remaining - 16'd1;
                                state <= P_ROOT_READ;
                            end
                        end else begin
                            if (sec_in_clus == sec_per_clus - 8'd1) begin
                                fat_lookup_for_file <= 1'b0;
                                state <= P_FAT_LOOKUP;
                            end else begin
                                sec_in_clus <= sec_in_clus + 8'd1;
                                state <= P_ROOT_READ;
                            end
                        end
                    end

                    P_ROOT_NOTFOUND: begin
                        error  <= 1'b1;
                        halted <= 1'b1;
                    end

                    P_FILE_INIT: begin
                        cur_cluster     <= {found_clus_hi, found_clus_lo};
                        bytes_remaining <= found_size;
                        sec_in_clus     <= 8'd0;
                        file_found      <= 1'b1;
                        if (found_size == 32'd0 || {found_clus_hi, found_clus_lo} == 32'd0) begin
                            file_done <= 1'b1;
                            halted    <= 1'b1;
                        end else begin
                            state <= P_FILE_READ;
                        end
                    end

                    P_FILE_READ: begin
                        blk_start <= 1'b1;
                        blk_lba   <= clus_sector;
                        sbyte     <= 10'd0;
                        state     <= P_FILE_READ_WAIT;
                    end

                    P_FILE_READ_WAIT: begin
                        if (blk_data_valid) begin
                            if (bytes_remaining != 32'd0) begin
                                data_valid      <= 1'b1;
                                data_out        <= blk_data_in;
                                bytes_remaining <= bytes_remaining - 32'd1;
                            end
                            sbyte <= sbyte + 10'd1;
                        end
                        if (blk_done) begin
                            if (bytes_remaining == 32'd0) begin
                                file_done <= 1'b1;
                                halted    <= 1'b1;
                            end else if (sec_in_clus == sec_per_clus - 8'd1) begin
                                fat_lookup_for_file <= 1'b1;
                                state <= P_FAT_LOOKUP;
                            end else begin
                                sec_in_clus <= sec_in_clus + 8'd1;
                                state <= P_FILE_READ;
                            end
                        end
                    end

                    P_FAT_LOOKUP: begin
                        blk_start        <= 1'b1;
                        blk_lba          <= fat_sector_w;
                        sbyte            <= 10'd0;
                        next_cluster_raw <= 32'd0;
                        state            <= P_FAT_LOOKUP_WAIT;
                    end

                    P_FAT_LOOKUP_WAIT: begin
                        if (blk_data_valid) begin
                            capture_fat_entry_byte(sbyte, fat_off_w, is_fat32, blk_data_in);
                            sbyte <= sbyte + 10'd1;
                        end
                        if (blk_done) begin
                            if (is_fat32) begin
                                if ((next_cluster_raw & 32'h0FFF_FFFF) >= 32'h0FFF_FFF8) begin
                                    if (fat_lookup_for_file && bytes_remaining == 32'd0) begin
                                        file_done <= 1'b1;
                                        halted    <= 1'b1;
                                    end else begin
                                        error  <= 1'b1; // chain ended early, or root chain exhausted
                                        halted <= 1'b1;
                                    end
                                end else begin
                                    cur_cluster <= next_cluster_raw & 32'h0FFF_FFFF;
                                    sec_in_clus <= 8'd0;
                                    state <= fat_lookup_for_file ? P_FILE_READ : P_ROOT_READ;
                                end
                            end else begin
                                if (next_cluster_raw[15:0] >= 16'hFFF8) begin
                                    if (fat_lookup_for_file && bytes_remaining == 32'd0) begin
                                        file_done <= 1'b1;
                                        halted    <= 1'b1;
                                    end else begin
                                        error  <= 1'b1;
                                        halted <= 1'b1;
                                    end
                                end else begin
                                    cur_cluster <= {16'd0, next_cluster_raw[15:0]};
                                    sec_in_clus <= 8'd0;
                                    state <= fat_lookup_for_file ? P_FILE_READ : P_ROOT_READ;
                                end
                            end
                        end
                    end

                    default: begin
                        error  <= 1'b1;
                        halted <= 1'b1;
                    end
                endcase
            end
        end
    end

endmodule
