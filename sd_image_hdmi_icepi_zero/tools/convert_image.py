#!/usr/bin/env python3
"""Convert any image into IMAGE.RAW, the raw format img_loader.v expects.

Format (see rtl/img_loader.v for the RTL-side reader):
    offset 0..3 : magic "RIMG" (ASCII)
    offset 4..5 : width,  16-bit little-endian (must be 160)
    offset 6..7 : height, 16-bit little-endian (must be 120)
    offset 8..  : 160*120 pixels, RGB565, 16-bit little-endian each,
                  row-major, top row first, left pixel first.

160x120 was chosen so the whole frame fits in the LFE5U-25F's on-chip
block RAM (no external SDRAM in this design) - see README.md for the
budget math. The source image is resized to fill 160x120 exactly,
cropping any excess to preserve aspect ratio (no letterboxing/stretch
distortion) - if you don't want cropping, pre-resize/pad the source
image to a 4:3 aspect ratio yourself before running this.

Usage:
    python3 convert_image.py input.jpg IMAGE.RAW
"""
import sys
import struct

try:
    from PIL import Image
except ImportError:
    sys.exit("This script needs Pillow: pip install pillow")

WIDTH = 160
HEIGHT = 120


def rgb888_to_rgb565(r, g, b):
    r5 = r >> 3
    g6 = g >> 2
    b5 = b >> 3
    return (r5 << 11) | (g6 << 5) | b5


def main():
    if len(sys.argv) != 3:
        sys.exit(f"usage: {sys.argv[0]} <input image> <output IMAGE.RAW>")

    src_path, dst_path = sys.argv[1], sys.argv[2]

    img = Image.open(src_path).convert("RGB")

    # Resize to fill 160x120 exactly, cropping the excess on the longer
    # axis to preserve aspect ratio (avoids squashing/stretching).
    src_w, src_h = img.size
    target_ratio = WIDTH / HEIGHT
    src_ratio = src_w / src_h

    if src_ratio > target_ratio:
        # source is wider than target: crop left/right
        new_w = int(src_h * target_ratio)
        left = (src_w - new_w) // 2
        img = img.crop((left, 0, left + new_w, src_h))
    else:
        # source is taller than target: crop top/bottom
        new_h = int(src_w / target_ratio)
        top = (src_h - new_h) // 2
        img = img.crop((0, top, src_w, top + new_h))

    img = img.resize((WIDTH, HEIGHT), Image.LANCZOS)

    with open(dst_path, "wb") as f:
        f.write(b"RIMG")
        f.write(struct.pack("<HH", WIDTH, HEIGHT))
        pixels = img.load()
        for y in range(HEIGHT):
            for x in range(WIDTH):
                r, g, b = pixels[x, y]
                f.write(struct.pack("<H", rgb888_to_rgb565(r, g, b)))

    expected_size = 8 + WIDTH * HEIGHT * 2
    print(f"Wrote {dst_path}: {expected_size} bytes ({WIDTH}x{HEIGHT} RGB565)")
    print("Copy this file to the SD card's root directory as IMAGE.RAW "
          "(8.3 filename, all-caps, on a FAT16/FAT32-formatted card).")


if __name__ == "__main__":
    main()
