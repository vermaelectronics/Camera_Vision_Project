#!/usr/bin/env python3
# ============================================================================
# gen_bmp_fat16_image.py -- builds a synthetic FAT16 volume (same layout
# style as the sibling 02_sdcard_text_reader sub-project's generator)
# containing one 160x120 24bpp uncompressed BMP file, "IMAGE.BMP", plus the
# golden expected RGB565 frame-buffer contents bmp_frame_loader.v should
# produce from it -- for tb_bmp_display.v to check against.
# ----------------------------------------------------------------------------
# Not part of the hardware build -- simulation support only.
# ============================================================================
import struct

SECTOR = 512
IMG_W, IMG_H = 160, 120

reserved_sectors   = 1
num_fats           = 1
sectors_per_fat     = 1
root_entries        = 16
sectors_per_cluster = 1

fat_start      = reserved_sectors
root_dir_start = fat_start + num_fats * sectors_per_fat
root_dir_sectors = (root_entries * 32 + SECTOR - 1) // SECTOR
data_start     = root_dir_start + root_dir_sectors

# ---- build the BMP pixel data (bottom-up, BGR888) + the golden RGB565 frame ----
row_bytes = IMG_W * 3
pixel_data = bytearray(row_bytes * IMG_H)
golden565 = [0] * (IMG_W * IMG_H)

for py in range(IMG_H):
    for px in range(IMG_W):
        r = (px * 255) // (IMG_W - 1)
        g = (py * 255) // (IMG_H - 1)
        b = ((px + py) * 255) // (IMG_W + IMG_H - 2)
        r5 = r >> 3
        g6 = g >> 2
        b5 = b >> 3
        golden565[py * IMG_W + px] = (r5 << 11) | (g6 << 5) | b5

        file_row = IMG_H - 1 - py  # bottom-up
        off = file_row * row_bytes + px * 3
        pixel_data[off + 0] = b
        pixel_data[off + 1] = g
        pixel_data[off + 2] = r

header_size = 14 + 40
file_size = header_size + len(pixel_data)

bmp = bytearray(file_size)
bmp[0:2] = b"BM"
struct.pack_into("<I", bmp, 2, file_size)
struct.pack_into("<I", bmp, 10, header_size)  # pixel data offset
struct.pack_into("<I", bmp, 14, 40)           # BITMAPINFOHEADER size
struct.pack_into("<i", bmp, 18, IMG_W)
struct.pack_into("<i", bmp, 22, IMG_H)        # positive = bottom-up
struct.pack_into("<H", bmp, 26, 1)            # planes
struct.pack_into("<H", bmp, 28, 24)           # bpp
struct.pack_into("<I", bmp, 30, 0)            # BI_RGB
bmp[header_size:] = pixel_data

num_clusters = (len(bmp) + SECTOR - 1) // SECTOR
NUM_BLOCKS = data_start + num_clusters + 4  # a little headroom

blocks = bytearray(NUM_BLOCKS * SECTOR)

def put(sector, offset, data):
    base = sector * SECTOR + offset
    blocks[base:base + len(data)] = data

# ---- boot sector ----
boot = bytearray(SECTOR)
boot[0:3]  = b"\xEB\x3C\x90"
boot[3:11] = b"MSWIN4.1"
struct.pack_into("<H", boot, 11, SECTOR)
boot[13]   = sectors_per_cluster
struct.pack_into("<H", boot, 14, reserved_sectors)
boot[16]   = num_fats
struct.pack_into("<H", boot, 17, root_entries)
struct.pack_into("<H", boot, 19, min(NUM_BLOCKS, 65535))
boot[21]   = 0xF8
struct.pack_into("<H", boot, 22, sectors_per_fat)
boot[38]   = 0x29
boot[43:54] = b"ICEPI ZERO "
boot[54:62] = b"FAT16   "
boot[510]  = 0x55
boot[511]  = 0xAA
blocks[0:SECTOR] = boot

# ---- FAT: chain clusters 2 .. 2+num_clusters-1 sequentially, then EOF ----
fat = bytearray(SECTOR)
struct.pack_into("<H", fat, 0, 0xFFF8)
struct.pack_into("<H", fat, 2, 0xFFFF)
for i in range(num_clusters):
    cluster = 2 + i
    nxt = 0xFFFF if i == num_clusters - 1 else cluster + 1
    struct.pack_into("<H", fat, cluster * 2, nxt)
put(fat_start, 0, fat)

# ---- root directory ----
root = bytearray(SECTOR)
entry = bytearray(32)
entry[0:11] = b"IMAGE   BMP"
entry[11]   = 0x20
struct.pack_into("<H", entry, 26, 2)  # first cluster
struct.pack_into("<I", entry, 28, len(bmp))
root[0:32] = entry
put(root_dir_start, 0, root)

# ---- file data ----
for i in range(num_clusters):
    chunk = bmp[i * SECTOR:(i + 1) * SECTOR]
    put(data_start + i, 0, chunk)

with open("sd_image_bmp.hex", "w") as f:
    for b in blocks:
        f.write("%02x\n" % b)

with open("golden_frame565.hex", "w") as f:
    for v in golden565:
        f.write("%04x\n" % v)

print("file_size=%d bytes, num_clusters=%d, NUM_BLOCKS=%d" % (len(bmp), num_clusters, NUM_BLOCKS))
