#!/usr/bin/env python3
# ============================================================================
# gen_fat16_image.py -- builds a small, spec-correct synthetic FAT16 volume
# image for tb_sdcard_text_reader.v's behavioral SD card model.
# ----------------------------------------------------------------------------
# Not part of the hardware build -- simulation support only. Regenerate with:
#   python3 gen_fat16_image.py
# which (re)writes sd_image.hex (one byte per line, $readmemh format, fed
# into sd_card_model's `blocks` memory) and expected_content.hex (the file
# HELLO.TXT's contents, same format, for the testbench to compare against
# fat16_reader's streamed output).
#
# Deliberately minimal: 1 FAT, 1 sector/cluster, a 16-entry root directory,
# a single file ("HELLO.TXT") spanning 2 clusters so the testbench exercises
# real FAT cluster-chain following, not just a single-cluster file.
# ============================================================================
import struct

SECTOR = 512
NUM_BLOCKS = 8

reserved_sectors  = 1
num_fats          = 1
sectors_per_fat    = 1
root_entries       = 16
sectors_per_cluster = 1

fat_start      = reserved_sectors
root_dir_start = fat_start + num_fats * sectors_per_fat
root_dir_sectors = (root_entries * 32 + SECTOR - 1) // SECTOR
data_start     = root_dir_start + root_dir_sectors

content = (b"Hello from the IcePi-Zero FAT16 SD card bring-up demo!\r\n"
           b"This text file was read off a real (simulated) FAT16 volume\r\n"
           b"by fat16_reader.v, one byte at a time, following the FAT\r\n"
           b"cluster chain across two clusters, and streamed out over the\r\n"
           b"UART. If you can read this on a real board, the whole chain\r\n"
           b"-- SPI, FAT16 parsing, and the UART link -- is working.\r\n")
# pad/truncate to exactly 600 bytes so it deterministically spans clusters 2 and 3
content = (content * 8)[:600]
file_size = len(content)

blocks = bytearray(NUM_BLOCKS * SECTOR)

def put(sector, offset, data):
    base = sector * SECTOR + offset
    blocks[base:base + len(data)] = data

# ---- boot sector ----
boot = bytearray(SECTOR)
boot[0:3]   = b"\xEB\x3C\x90"
boot[3:11]  = b"MSWIN4.1"
struct.pack_into("<H", boot, 11, SECTOR)          # BPB_BytsPerSec
boot[13]    = sectors_per_cluster                  # BPB_SecPerClus
struct.pack_into("<H", boot, 14, reserved_sectors)  # BPB_RsvdSecCnt
boot[16]    = num_fats                              # BPB_NumFATs
struct.pack_into("<H", boot, 17, root_entries)      # BPB_RootEntCnt
struct.pack_into("<H", boot, 19, 64)                # BPB_TotSec16 (32KB nominal volume)
boot[21]    = 0xF8                                  # BPB_Media
struct.pack_into("<H", boot, 22, sectors_per_fat)   # BPB_FATSz16
boot[36]    = 0x80                                  # BS_DrvNum
boot[38]    = 0x29                                  # BS_BootSig
struct.pack_into("<I", boot, 39, 0x12345678)        # BS_VolID
boot[43:54] = b"ICEPI ZERO "                        # BS_VolLab (11 bytes)
boot[54:62] = b"FAT16   "                           # BS_FilSysType
boot[510]   = 0x55
boot[511]   = 0xAA
blocks[0:SECTOR] = boot

# ---- FAT ----
fat = bytearray(SECTOR)
struct.pack_into("<H", fat, 0, 0xFFF8)  # FAT[0] = 0xFF00 | media
struct.pack_into("<H", fat, 2, 0xFFFF)  # FAT[1] = reserved
struct.pack_into("<H", fat, 4, 3)       # FAT[2] (cluster 2) -> next cluster 3
struct.pack_into("<H", fat, 6, 0xFFFF)  # FAT[3] (cluster 3) -> EOF
put(fat_start, 0, fat)

# ---- root directory: one 32-byte entry for HELLO.TXT ----
root = bytearray(SECTOR)
entry = bytearray(32)
entry[0:11] = b"HELLO   TXT"
entry[11]   = 0x20                       # ATTR_ARCHIVE
struct.pack_into("<H", entry, 26, 2)     # first cluster = 2
struct.pack_into("<I", entry, 28, file_size)
root[0:32] = entry
put(root_dir_start, 0, root)

# ---- file data: cluster 2 = data_start sector, cluster 3 = data_start+1 ----
put(data_start, 0, content[0:512])
put(data_start + 1, 0, content[512:600])

with open("sd_image.hex", "w") as f:
    for b in blocks:
        f.write("%02x\n" % b)

with open("expected_content.hex", "w") as f:
    for b in content:
        f.write("%02x\n" % b)

print("fat_start=%d root_dir_start=%d root_dir_sectors=%d data_start=%d"
      % (fat_start, root_dir_start, root_dir_sectors, data_start))
print("file_size=%d bytes, spans clusters 2 and 3" % file_size)
