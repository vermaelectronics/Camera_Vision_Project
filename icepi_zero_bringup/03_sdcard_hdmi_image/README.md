# IcePi-Zero bring-up #3: SD card → HDMI image display

Loads a 160×120 24bpp BMP off a FAT16-formatted microSD card and displays
it over native GPDI/HDMI, nearest-neighbor upscaled to fill a real
1280×720p60 signal.

## What it does

1. `sdcard_spi.v` + `fat16_reader.v` — reused **byte-for-byte, unmodified**
   from the sibling `02_sdcard_text_reader` sub-project — mount the card
   and find `IMAGE.BMP` by name in the root directory. A FAT16 file's
   bytes are just bytes in order regardless of what's making sense of
   them; the storage stack doesn't need to know or care that this time
   they're pixels instead of text.
2. `bmp_frame_loader.v` parses the BMP header, validates it's exactly the
   shape this design supports (see "BMP requirements" below), and streams
   each pixel's BGR888 bytes into a 160×120 RGB565 frame buffer
   (`dp_line_ram.v`, reused from the camera project — same true-dual-port
   block-RAM building block, just resized).
3. Once loaded, the display pipeline — `clk_gen_dvi.v`, `video_timing_gen.v`,
   `tmds_encoder.v`, `tmds_serial_gearbox.v`, all reused **unmodified** from
   `dvp_camera_hdmi_pipeline` — reads the frame buffer through
   `nn_scale_addr.v`, which maps each of the 1280×720 output pixels back to
   its 160×120 source pixel (8×6 nearest-neighbor upscale — both exact
   integer factors, chosen specifically so this needs no division: 1280/160
   is a power of 2 (a plain bit-slice) and 720/120 is tracked with a
   6-deep row counter instead of a runtime divide).
4. Until an image has successfully loaded (or if loading fails), the
   display shows `test_pattern_gen.v`'s color-bar pattern (also reused
   unmodified from the camera project) instead of a blank/garbage screen.

## Why 160×120

The ECP5-25F's entire block-RAM budget is ~896Kbit (56 × 16Kbit DP16KD
blocks). A frame buffer needs to fit comfortably inside that alongside
everything else the design uses — no external SDRAM controller exists in
this sub-project. 160×120 RGB565 = 19,200 × 16 bits ≈ 300Kbit (19 of the
56 blocks, see "Verified" below) — a healthy fraction, but leaves most of
the chip's block RAM free. It also gives clean, division-free upscale
factors to 1280×720 (8× and 6×), which a less deliberately chosen
resolution likely wouldn't.

The image will look blocky — each source pixel becomes a visible 8×6
block on screen. That's expected and inherent to the resolution choice,
not a bug; the point of this sub-project is proving the SD→BMP→HDMI
pipeline works end to end, not photographic quality.

## BMP requirements

This is a minimal BMP reader, not a general one — it validates every field
below and refuses (error LED) anything that doesn't match exactly, rather
than guessing:

- Exactly **160×120** pixels
- Exactly **24 bits/pixel**, uncompressed (`BI_RGB`, compression field = 0)
- A standard 40-byte `BITMAPINFOHEADER`
- **Positive height** (bottom-up row order — the default for essentially
  every BMP encoder; top-down BMPs, indexed-color BMPs, and any other bit
  depth are rejected)

Producing a compliant file with ImageMagick:

```bash
convert your_photo.jpg -resize 160x120! -type TrueColor -depth 8 \
        -define bmp:format=bmp3 IMAGE.BMP
```

(`!` forces the exact 160×120 size regardless of aspect ratio — crop/pad
first if you want to avoid stretching.) Copy `IMAGE.BMP` onto a
**FAT16**-formatted card — see the sibling sub-project's README
("Preparing the SD card") for how to reformat one; the same FAT16-only
scope limitation applies here.

## LEDs (all real, onboard, no external wiring)

| LED | Meaning |
|---|---|
| `led[0]` | SD card detected + initialized |
| `led[1]` | Reading the file / parsing the BMP |
| `led[2]` | Last load failed — sticky (SD error, FAT16 error, or bad BMP) |
| `led[3]` | Image loaded and now on screen — sticky |
| `led[4]` | 1Hz heartbeat |

Press `button[1]` to reload the image (e.g. after swapping the card);
`button[0]` resets the whole design.

## Verified

- **Simulation** (Icarus Verilog), two testbenches — the pixel/HDMI
  datapath itself is **not** simulated (see "Why no full top-level
  testbench" below):
  - `make sim_bmp` (`tb_bmp_display.v`): the full SD→FAT16→BMP storage
    stack against the same behavioral SD card model the sibling
    sub-project uses, mounting a synthetic-but-spec-correct FAT16 volume
    (`tb/gen_bmp_fat16_image.py`) containing a real 160×120 24bpp BMP
    (a deterministic RGB gradient) whose 57,654-byte file size spans
    **113 FAT clusters** — genuinely exercising the cluster-chain-follow
    path repeatedly, not just once. **`TB_BMP_DISPLAY: PASS`** — every one
    of the 19,200 written frame-buffer pixels exactly matches the RGB565
    values computed independently in Python, including the BGR→RGB
    channel-order fix and the bottom-up row inversion.
  - `make sim_scale` (`tb_nn_scale_addr.v`): `nn_scale_addr.v` alone,
    checking `small_x` combinationally across all 1280 columns and
    `small_y` across all 720 row-boundary events of a full frame (plus the
    next-frame wraparound), each against the equivalent floor-division
    computed independently. **`TB_NN_SCALE_ADDR: PASS`**.
- **Synthesis** (`make synth`, Yosys `synth_ecp5`): 0 CHECK-pass problems.
  4940 cells, including 19 DP16KD (the frame buffer — 19 of 56 available
  block RAM blocks, as budgeted above), 1 EHXPLLL (the pixel-clock PLL),
  3+1 ODDRX2F/ODDRX1F and 1 ECLKSYNCB (the GPDI serializer, from
  `tmds_serial_gearbox.v`).
- **Place & route** (`make pnr`, `nextpnr-ecp5 --25k --package CABGA256
  --speed 6 --seed 1`): closes timing on **all three clock domains** —
  `clk` (50MHz target): 67.29MHz achieved. `clk_pixel` (74.29MHz target,
  the pixel clock): **84.32MHz achieved**. `u_ser.sclk` (185.74MHz target,
  the 5×-pixel SERDES clock): 206.91MHz achieved. 0 errors, "Program
  finished normally."
- **Not yet flashed to real hardware in this session** — same caveat as
  the sibling sub-project: `ecppack`/`openFPGALoader` aren't installed in
  this sandbox. Everything upstream (RTL logic, synthesis, real place &
  route against the real board LPF) is verified as above.

### A real timing bug found and fixed via P&R (not simulation)

The first synthesis+P&R pass **failed** the `clk_pixel` timing budget
(47–59MHz achieved vs. 74.29MHz needed, depending on seed) — a long
combinational path ran straight from the frame buffer's own registered
block-RAM output, through the RGB565→888 bit-replication expansion,
through the display/test-pattern mux, into `tmds_encoder`'s internal
XOR-chain logic, all within one clock period. `dvp_camera_hdmi_pipeline`'s
own top module hit and documented this exact class of bug for its own
video-buffer path (see that project's README, "Timing closure notes") —
the fix here is the same one applied there: widen the pixel-domain
realignment from one pipeline register to two, so the frame-buffer read
path and the sync/test-pattern path both take exactly two `clk_pixel`
cycles from `(x,y)` to reaching the encoder, instead of an unbalanced one
vs. two. Real STA (`nextpnr-ecp5`), not simulation, is what catches this
class of bug — Icarus has no timing model at all, and there's no open
behavioral model for the ECP5 primitives this datapath uses (see below),
so simulation could never have found it here.

### Why no full top-level testbench

`image_display_top.v` instantiates a real ECP5 `EHXPLLL` primitive (via
`clk_gen_dvi.v`) to generate the pixel clock, which has no open-source
behavioral simulation model — the exact same situation
`dvp_camera_hdmi_pipeline` is in, which also has no full top-level
testbench for the same reason (check that project's `tb/` directory: every
testbench there covers one piece, never the assembled top module). The
pixel/HDMI datapath's correctness rests on real synthesis + place & route
against the real board LPF (above), same as it does there.

## Building

```bash
cd 03_sdcard_hdmi_image
make check     # both simulations — should print PASS x2
make prog      # synth + P&R + pack + flash in one step
```

## Module reference

| File | Role |
|---|---|
| `rtl/spi_master.v`, `rtl/sdcard_spi.v`, `rtl/fat16_reader.v` | Storage stack — identical to `02_sdcard_text_reader` |
| `rtl/bmp_frame_loader.v` | BMP header validation + BGR888→RGB565 pixel stream → frame buffer |
| `rtl/dp_line_ram.v` | Frame buffer (reused from the camera project, resized) |
| `rtl/nn_scale_addr.v` | 160×120 → 1280×720 nearest-neighbor address generator |
| `rtl/clk_gen_dvi.v`, `rtl/video_timing_gen.v`, `rtl/tmds_encoder.v`, `rtl/async_fifo.v`, `rtl/tmds_serial_gearbox.v` | HDMI/TMDS pipeline — reused unmodified from the camera project |
| `rtl/test_pattern_gen.v` | Fallback pattern shown before/without a loaded image |
| `rtl/image_display_top.v` | Top-level: wiring, debounce, realignment, status LEDs |
| `tb/sd_card_model.v` | Simulation-only behavioral SD card SPI slave (shared design with sub-project 2) |
| `tb/gen_bmp_fat16_image.py` | Generates the synthetic BMP + FAT16 test volume + golden frame |

## Known limitations

- BMP support is intentionally narrow — see "BMP requirements" above.
- Same FAT16-only scope as the sibling sub-project (root directory, 8.3
  names, no subdirectories).
- Fixed 160×120 output resolution, chosen for the block-RAM/scale-factor
  reasons above — not runtime-configurable without resynthesizing.
- Same untested-in-simulation SDSC (non-HC) addressing path noted in the
  sibling sub-project's README — this sub-project's behavioral SD model
  also only emulates SDHC/SDXC.
- No partial/progressive display — the whole frame buffer loads before
  anything but the fallback test pattern appears; a very large/slow card
  read would show the pattern for the whole load duration, not a
  progressively-filling image.
