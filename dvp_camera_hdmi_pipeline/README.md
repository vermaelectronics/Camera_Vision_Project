# DVP Camera → HDMI Pipeline for IcePi‑Zero (Lattice ECP5 LFE5U‑25F‑6BG256C)

A complete, synthesizable Verilog RTL pipeline that takes a parallel (DVP)
camera sensor's output and displays it live on an HDMI/DVI monitor, targeting
the **IcePi‑Zero** board (a Raspberry‑Pi‑Zero‑form‑factor ECP5 FPGA board).
Supports **720p60** and **1080p** video, built and verified entirely with the
open‑source FPGA toolchain (Yosys + nextpnr‑ecp5), including full
place‑and‑route and static timing analysis against the real chip.

**Everything in this repository was actually run** through `iverilog`
(simulation), `yosys` (synthesis) and `nextpnr-ecp5` (place & route + timing
closure) against the real `LFE5U-25F-6BG256C` part during development — see
[Verification](#verification) for the exact results. This is not a
paper design.

---

## Table of contents

1. [What you get](#what-you-get)
2. [Why two different 1080p60 delivery paths](#why-two-different-1080p60-delivery-paths)
3. [Architecture](#architecture)
4. [Clocking](#clocking)
5. [Module reference](#module-reference)
6. [Wiring the camera](#wiring-the-camera)
7. [Adapting to your sensor](#adapting-to-your-sensor)
8. [Building](#building)
9. [Simulating](#simulating)
10. [Verification](#verification)
11. [Timing closure notes](#timing-closure-notes)
12. [Bring-up checklist](#bring-up-checklist)
13. [Known limitations / future work](#known-limitations--future-work)
14. [License](#license)

---

## What you get

```
dvp_camera_hdmi_pipeline/
├── rtl/                       14 synthesizable Verilog modules (see Module reference)
├── tb/                        6 self-checking testbenches (all passing, see Verification)
├── constraints/
│   └── icepi_zero.lpf         Real pin/site constraints for the IcePi-Zero board
├── Makefile                   sim / synth / pnr / bit / prog targets
└── README.md                  this file
```

Two top-level designs, both built from the same shared RTL blocks:

| Top level | Resolution(s) | Output | Needs |
|---|---|---|---|
| `dvp_camera_hdmi_top.v` | **1280×720@60Hz** and **1920×1080@30Hz** | native GPDI/TMDS (the board's onboard mini-HDMI-alike connector) | nothing extra — drives the connector directly; includes a 24MHz MCLK PLL + PWDN/RESET sequencer for the Waveshare OV5640 |
| `dvp_camera_hdmi_top_ext.v` | **1920×1080@60Hz** (true 60Hz) | 24-bit parallel RGB + HSYNC/VSYNC/DE + pixel clock | an external HDMI/DVI transmitter IC (ADV7511/ADV7513, IT6613/IT66121, SiI9022-class); does not yet include the MCLK/power-sequencer additions, see [Known limitations](#known-limitations--future-work) |

Both include:
- An on-chip 8‑bar colour‑bar **test pattern generator** (hold button[1] to
  show it) so you can verify the whole HDMI/DVI output chain works before a
  camera is even connected.
- A generic **I²C master + register-table sequencer** for camera
  configuration, supporting both 8-bit and 16-bit register addressing
  (`dvp_camera_hdmi_top.v` ships pre-configured for the Waveshare OV5640;
  see [Adapting to your sensor](#adapting-to-your-sensor) for other
  sensors).
- Status LEDs for PLL lock, camera-config-done, buffer-ready and I²C-NACK.

---

## Why two different 1080p60 delivery paths

This is the single most important engineering fact about this repository,
so it's worth explaining up front rather than glossing over.

HDMI/DVI's TMDS signalling needs **10 bits serialized per pixel clock** on
each of 3 data lanes. For 1080p60 that's a pixel clock of 148.5 MHz, so the
serializer has to run at a **1.485 Gbit/s** bit rate per lane.

The **LFE5U‑25F** used on IcePi‑Zero is a *plain* ECP5 part — it has **no**
built‑in multi‑gigabit SERDES/DCU block (that's only present on the "5G"
ECP5 parts, e.g. `LFE5UM5G‑25F`). Without a hardware SERDES, the only way to
serialize TMDS is the documented ECP5 **fabric DDR gearbox**
(`ODDRX2F` + `ECLKSYNCB` + `CLKDIVF`, see `tmds_serial_gearbox.v`), which
needs an edge‑clock (`ECLK`) running at **5× the pixel clock**. For 1080p60
that's **742.5 MHz** — far beyond what the ECP5's edge‑clock network can
run at (realistically ~350–400 MHz on a `-6` speed‑grade part; this is the
same ceiling DDR3‑800 SDRAM controllers on ECP5 run into). **This is a real
silicon limit, not a limitation of this RTL.**

720p60 and 1080p30 both need only **74.25 MHz** pixel clock (yes — CEA‑861
defines 1080p at 30/25/24 Hz using the *same* horizontal/vertical blanking
totals as the 60/50/48 Hz modes, just at half the pixel clock), so their
ECLK requirement is 371.25 MHz — comfortably inside the ECP5's real
capability, and verified to place, route and close timing in this
repository (see [Verification](#verification)).

So, two honest options are provided instead of one design that would quietly
not work on real hardware:

1. **`dvp_camera_hdmi_top.v`** — native GPDI/TMDS output, for 720p60 (full
   native frame rate) or 1080p30 (full native resolution, half frame rate)
   directly out of the board's own connector. **Recommended default.**
2. **`dvp_camera_hdmi_top_ext.v`** — true 1080p60, by running the pixel
   pipeline at the real 148.5 MHz‑class pixel clock and handing the
   **parallel** RGB+sync bus to an external HDMI transmitter IC, which does
   the TMDS serialization in its own silicon (no ECLK ceiling — the
   parallel bus only has to run at 1× pixel clock, which the ECP5 handles
   easily; this is exactly how the overwhelming majority of embedded
   HDMI‑output products that use a non‑SERDES FPGA/SoC actually do it).

If you specifically need true 1080p60 straight out of the GPDI connector
with no extra chip, that requires either an ECP5‑5G part (with SERDES) or a
different FPGA family — it is not achievable on the plain LFE5U‑25F, by
anyone, in any RTL.

---

## Architecture

```
 DVP camera                                            clk_pixel domain (74.25MHz-class)
 (its own PCLK  ┌──────────────┐   ┌────────────────┐  ┌──────────────────┐
  domain)   ───▶│ dvp_capture  │──▶│ pixel_formatter │  │ video_timing_gen │──▶ hsync/vsync/de/x/y
  HREF/VSYNC/D[]│ (byte stream)│   │ RGB565/YUYV422  │  └─────────┬────────┘
                └──────────────┘   │   -> RGB888     │            │
                                    └────────┬────────┘            ▼
                                             │ cam_pixel_valid,   test_pattern_gen
                                             │ cam_rgb[23:0]        (bring-up aid)
                                             ▼                       │
                                    ┌──────────────────┐             ▼
                                    │ video_cdc_buffer  │────▶ pixel mux ──▶ pixel_rgb
                                    │ (async_fifo-based)│                       │
                                    │ camera clk ⇄ pixel │            ┌─────────┴─────────┐
                                    │ clk, elastic       │            ▼                   ▼
                                    └────────────────────┘   3x tmds_encoder      (top_ext: straight to
                                                              (R,G,B, 2-stage      hdmi_d/hdmi_pclk/
                                                               pipelined)          hdmi_hsync/vsync/de)
                                                                       │
                                                                       ▼
                                                          tmds_serial_gearbox
                                                          (ECLKSYNCB+CLKDIVF+
                                                           ODDRX2F, async_fifo
                                                           CDC into SCLK domain)
                                                                       │
                                                                       ▼
                                                                  gpdi_dp[3:0]
```

Key design decisions, and why:

- **The camera↔pixel-clock crossing is a real, generic async FIFO**
  (`async_fifo.v`, verified independently — see below), not a hand-derived
  static phase assumption between unrelated clocks. The camera's own PCLK
  and the FPGA's PLL-generated pixel clock are genuinely independent
  oscillators; treating them as such is the only sound way to build this.
  It degrades gracefully under any clock-rate mismatch: if the buffer runs
  dry, the last pixel is held (a frozen line, not garbage); if it fills up,
  new camera data is simply dropped (no corruption). See
  `video_cdc_buffer.v`'s header comment for the full reasoning.

- **The TMDS-domain crossing (pixel clock → SCLK, a non-integer 2.5:1
  ratio) is *also* a real async FIFO**, for the same reason — even though
  both clocks come from the same PLL (so they're "mesochronous"), this
  repository does not rely on any assumed static phase relationship between
  PLL output taps. See `tmds_serial_gearbox.v`'s header.

- **The TMDS encoder is internally 2-stage pipelined.** A single-cycle
  implementation of the DVI 8b/10b algorithm does *not* close timing at
  74.25 MHz on this exact `-6` speed-grade part — confirmed against real
  `nextpnr-ecp5` static timing analysis, not assumed. See
  [Timing closure notes](#timing-closure-notes).

- **`video_timing_gen` registers its outputs internally** for the same
  reason (needed specifically for the `top_ext` 1080p60 variant's tighter
  6.7 ns budget).

---

## Clocking

Board reference oscillator: **50 MHz** (confirmed from the IcePi‑Zero
board's own published constraints file, pin `M1`).

| Clock domain | Frequency | Generated by | Used for |
|---|---|---|---|
| `clk` (board osc) | 50 MHz | crystal | PLL reference |
| `cam_mclk` | 24.000 MHz (exact — 50MHz divides evenly) | `clk_gen_mclk.v` (`EHXPLLL`) | camera XCLK/MCLK — required by sensors with no onboard crystal (e.g. OV5640); also clocks `cam_power_sequencer.v` |
| `cam_pclk` | sensor-dependent (≤ ~155 MHz assumed) | camera sensor | DVP capture |
| `clk_pixel` (720p60/1080p30 path) | 74.286 MHz (target 74.25, +0.048%) | `clk_gen_dvi.v` (`EHXPLLL`) | video timing, TMDS encode |
| `clk_eclk` | 371.429 MHz (target 371.25, +0.048%) | same PLL, `CLKOP` | TMDS serializer edge clock |
| `u_ser.sclk` | = `clk_eclk`/2 ≈ 185.7 MHz | `CLKDIVF` | ODDRX2F SCLK |
| `clk_pixel` (top_ext 1080p60 path) | 150.000 MHz (target 148.5, +1.01%; alt. 148.148 MHz, −0.24%, see `clk_gen_1080p60.v`) | `clk_gen_1080p60.v` | video timing, parallel RGB out |

`EHXPLLL` divider values, the formula used to derive them, and the achieved
frequency vs. the CEA‑861 nominal are all documented in the header comments
of `clk_gen_dvi.v` and `clk_gen_1080p60.v`. **Re-verify (or regenerate)
these dividers with Lattice Diamond's Clarity Designer PLL wizard, or the
open-source `ecppll` utility from prj-trellis, before relying on them in
production** — this repository's environment could run `yosys` and
`nextpnr-ecp5` but not `ecppll` (that requires building prj-trellis from
source, which wasn't done here to keep the build environment minimal); the
divider math is shown worked out by hand and is internally consistent, but a
second, independent tool cross-check costs nothing and catches transcription
errors.

---

## Module reference

All in `rtl/`:

| Module | Purpose | Verified |
|---|---|---|
| `async_fifo.v` | Generic dual-clock FIFO (Gray-code pointers), used for every clock-domain crossing in the design | `tb_async_fifo.v`: exact-depth fill/drain + 300-word burst across two genuinely unrelated clocks (17ns/23ns periods), zero errors |
| `tmds_encoder.v` | DVI/HDMI 8b/10b TMDS encoder, 2-stage pipelined | `tb_tmds_encoder.v`: exhaustive round-trip (encode→independently decode) across all 256 input codes × 6 disparity-history sweeps = 1540 checks, zero errors |
| `video_timing_gen.v` | CEA-861 H/V timing generator, parametrized | `tb_video_timing_gen.v`: exact frame period & active-pixel count check |
| `dvp_capture.v` | DVP bus front end (PCLK/HREF/VSYNC/D[7:0] → byte stream) | `tb_dvp_pixel_chain.v` |
| `pixel_formatter.v` | RGB565 or YUYV422 (BT.601) → RGB888 | `tb_dvp_pixel_chain.v`: both formats, exact pixel values checked |
| `video_cdc_buffer.v` | Camera-clock → pixel-clock elastic buffer (wraps `async_fifo`) | covered via full-design synthesis + P&R (see below) |
| `clk_gen_dvi.v` | `EHXPLLL` wrapper: 50MHz → 74.25MHz-class pixel + 371.25MHz-class ECLK | covered via full-design P&R |
| `clk_gen_1080p60.v` | `EHXPLLL` wrapper: 50MHz → 148.5MHz-class pixel (no ECLK needed) | covered via full-design P&R |
| `tmds_serial_gearbox.v` | ECP5 native TMDS serializer (`ECLKSYNCB`+`CLKDIVF`+`ODDRX2F`) | covered via full-design synthesis + P&R |
| `test_pattern_gen.v` | 8-bar colour bars + ramp, for bring-up without a camera | visual (this is a bring-up aid, not part of the core signal path) |
| `clk_gen_mclk.v` | `EHXPLLL` wrapper: 50MHz → 24.000MHz exact, camera MCLK/XCLK | `tb_cam_power_sequencer.v` exercises the downstream consumer of `locked`; the PLL itself covered via full-design P&R |
| `cam_power_sequencer.v` | PWDN/RESET power-up sequencing state machine (wait-for-MCLK → power-up → reset-release → settle → done) | `tb_cam_power_sequencer.v`: checks strict ordering (PWDN low before RESET release, RESET release before `seq_done`) and that each phase honours its configured minimum duration, 0 errors |
| `i2c_master.v` | Polled I²C master (START/addr+W/reg-addr(8 or 16-bit)/data/STOP, ACK-checked) | `tb_i2c_master.v`: two parallel DUTs (`ADDR_BYTES=1` and `ADDR_BYTES=2`) each run a full transaction against an independent behavioral I²C slave model, all bytes and ACKs verified |
| `cam_config_rom.v` | Walks a register table out through `i2c_master` | covered via full-design synthesis + P&R; **ships with an OV5640-specific table by default now, see below** |
| `dvp_camera_hdmi_top.v` | Top level: native GPDI, 720p60/1080p30 — includes MCLK PLL + power sequencer, targets the OV5640 out of the box | full synthesis + P&R + timing closure, both resolutions |
| `dvp_camera_hdmi_top_ext.v` | Top level: external-transmitter, true 1080p60 — **not yet updated with MCLK/power-sequencer/16-bit-I²C**, see [Known limitations](#known-limitations--future-work) | full synthesis + P&R + timing closure |

---

## Wiring the camera

The IcePi‑Zero is a Raspberry‑Pi‑Zero‑form‑factor board — it has a generic
40‑pin GPIO header, **not** a dedicated parallel-camera connector. This
repository wires the 13 DVP signals onto 13 of that header's GPIO sites
(`constraints/icepi_zero.lpf`), chosen from the board's own published pin
list:

| Signal | FPGA net | Header pin |
|---|---|---|
| `cam_pclk`  | gpio[0]  | 27 |
| `cam_href`  | gpio[1]  | 28 |
| `cam_vsync` | gpio[2]  | 3  |
| `cam_d[0]`  | gpio[3]  | 5  |
| `cam_d[1]`  | gpio[4]  | 7  |
| `cam_d[2]`  | gpio[5]  | 29 |
| `cam_d[3]`  | gpio[6]  | 31 |
| `cam_d[4]`  | gpio[7]  | 26 |
| `cam_d[5]`  | gpio[8]  | 24 |
| `cam_d[6]`  | gpio[9]  | 21 |
| `cam_d[7]`  | gpio[10] | 19 |
| `cam_scl`   | gpio[11] | 23 |
| `cam_sda`   | gpio[12] | 32 |
| `cam_mclk`  | gpio[16] | 36 |
| `cam_rst_n` | gpio[17] | 11 |
| `cam_pwdn`  | gpio[18] | 12 |

Freely reassign these in `constraints/icepi_zero.lpf` to whatever's
convenient for your camera breakout — nothing in the RTL cares which
physical pins these land on.

**IO voltage caveat:** the IcePi‑Zero's GPIO bank is fixed at 3.3V
(`SYSCONFIG CONFIG_IOVOLTAGE=3.3` in the LPF). Many small camera modules
run their raw DVP interface at **1.8V** and need a level shifter — but the
**Waveshare OV5640 camera module** this design targets by default carries
its own onboard regulators/level-shifting and is designed to interface
directly at 3.3V (per Waveshare's own documentation for this board); no
external level shifter should be needed. Still, check your specific
breakout's silkscreen/datasheet before connecting power — driving a true
1.8V-only camera input at 3.3V can damage the sensor.

**Wiring the Waveshare OV5640 module specifically:**

| OV5640 module pin | Connect to | Notes |
|---|---|---|
| `D9`..`D2` | `cam_d[7]`..`cam_d[0]` | The module silkscreens its 8-bit parallel output as `D9:D2` (its own internal numbering, not a typo) — `D9` is the MSB and maps to `cam_d[7]`, down to `D2`→`cam_d[0]` |
| `PCLK` | `cam_pclk` | |
| `HREF` (or `HS`) | `cam_href` | |
| `VSYNC` | `cam_vsync` | |
| `SIOC`/`SCL` | `cam_scl` | |
| `SIOD`/`SDA` | `cam_sda` | |
| `XCLK`/`MCLK` | `cam_mclk` | **must** be driven — this module has no onboard crystal; `clk_gen_mclk.v` supplies the required 24 MHz |
| `RESET`/`RESETB` | `cam_rst_n` | active-low; sequenced by `cam_power_sequencer.v` |
| `PWDN` | `cam_pwdn` | active-high (powered down when high); sequenced by `cam_power_sequencer.v` |
| `VCC` (3.3V) | board 3.3V rail | **header pin 1** — free, not used by any signal above |
| `GND` | board ground | **header pin 6** — free, not used by any signal above |

These two are pure power rails, not FPGA I/O, so they aren't (and don't
need to be) in `icepi_zero.lpf` — wire them directly. The OV5640's peak
current draw during active capture can be a few hundred mA; the header's
3.3V rail is normally fine, but if you see brownouts/resets while the
camera is streaming, power the module from a separate 3.3V source and
only share `GND` with the board.

Do **not** tie `RESET`/`PWDN` to fixed levels or leave them floating —
without `cam_power_sequencer.v` actively driving the documented
power-up order (MCLK stable → PWDN low → hold RESET low briefly → release
RESET → settle → only then start I²C), the sensor's internal PLL/SCCB
interface may never come up reliably.

---

## Adapting to your sensor

`dvp_camera_hdmi_top.v` ships configured **for the Waveshare OV5640 module
by default**: `I2C_DEV_ADDR7 = 7'h3C`, `ADDR_BYTES = 2` (16-bit register
addressing), a 24 MHz `clk_gen_mclk.v` output, `cam_power_sequencer.v`
driving `PWDN`/`RESET`, and `cam_config_rom.v` pre-loaded with an
OV5640 RGB565 @ 1280×720 init table. If you're using that exact camera,
you likely only need the wiring in the section above. To adapt this
design to a **different** sensor:

1. **Set `CAMERA_FORMAT`** on the top-level instantiation to `"RGB565"` or
   `"YUYV422"` to match your sensor's configured DVP output format. If your
   sensor sends the low byte of an RGB565 pixel first, set `BYTE_SWAP=1`.
2. **Set `HREF_POL`/`VSYNC_POL`** to match your sensor's datasheet (most are
   active-high, matching the defaults).
3. **Set `ADDR_BYTES`** on both the `i2c_master` and `cam_config_rom`
   instantiations to `1` or `2` to match whether your sensor uses 8-bit
   register addressing (OV7670/OV2640/GC0308-class) or 16-bit (OV5640/
   OV5647-class) — `i2c_master.v` supports both natively via this
   parameter, no RTL surgery needed either way.
4. **Replace `cam_config_rom.v`'s register table** with your sensor's own
   init sequence (from its datasheet or a vendor reference driver) —
   widen/narrow `table_rom`'s `reg_addr` field to match `ADDR_BYTES`.
5. **Set `I2C_DEV_ADDR7`** to your sensor's 7-bit I²C address.
6. **If your sensor has no onboard crystal** (needs a host-supplied
   MCLK/XCLK, like the OV5640), keep using `clk_gen_mclk.v` +
   `cam_power_sequencer.v` — just retune `clk_gen_mclk.v`'s `EHXPLLL`
   dividers if your sensor's required XCLK isn't 24 MHz, and adjust
   `cam_power_sequencer.v`'s `RST_MS`/`SETTLE_MS` to your datasheet's
   power-up timing. If your sensor has its own crystal and defaults PWDN/
   RESET sensibly on power-up, these two modules (and the `cam_mclk`/
   `cam_rst_n`/`cam_pwdn` ports) can simply be left unconnected/removed.
7. **Configure your sensor, out-of-band, to output exactly 1280×720@60 or
   1920×1080@60/30** (whichever top level you're using) over its DVP
   interface. This design does not include a scaler — it assumes the
   camera's active-video window already matches the display resolution
   1:1. (Adding a line-buffer-based scaler is a natural extension point;
   see [Known limitations](#known-limitations--future-work).)

**A word of caution on the shipped OV5640 table:** it's reproduced from
general, widely-published/community knowledge of a typical OV5640
DVP‑RGB565 1280×720 init sequence — it has **not** been hardware-verified
against a real sensor in this development environment (no physical OV5640
was available here). Cross-check it against Waveshare's own official demo
code/register list for this exact module before relying on it, especially
`0x4300` (RGB byte-order — try `0x6F` if red/blue channels look swapped)
and the `0x3800`–`0x3821` output-window registers (most likely to need
adjustment if the image is offset, mirrored, or the wrong size). See the
extensive comments at the top of `cam_config_rom.v` for the full
per-register rationale.

---

## Building

Requires: `iverilog`, `yosys`, `nextpnr-ecp5` (all open-source, `apt install
iverilog yosys nextpnr-ecp5` on Debian/Ubuntu ≥ 22.04), and for a real
bitstream, `ecppack`/`ecppll` from
[prj-trellis](https://github.com/YosysHQ/prjtrellis) (not packaged by apt —
either build from source or grab the prebuilt
[oss-cad-suite](https://github.com/YosysHQ/oss-cad-suite-build) bundle,
which includes all four tools plus `openFPGALoader`).

```sh
make sim            # run every testbench (should print PASS for all 5)
make synth           # Yosys synthesis, 720p60/1080p30 top level
make pnr               # + nextpnr-ecp5 place&route + timing closure report
make synth_ext          # Yosys synthesis, external-transmitter (true 1080p60) top level
make pnr_ext              # + place&route (edit the LPF's hdmi_* section first, see Makefile)
make bit                    # pack a .bit bitstream (needs ecppack)
make prog                     # flash it (needs openFPGALoader)
```

Building the **1080p30** variant of `dvp_camera_hdmi_top.v`: pass
`RESOLUTION="1080P30"` as a module parameter override. From the CLI,
`chparam -set RESOLUTION 1080P30` on a string parameter can be finicky
across Yosys versions — the robust way (used during this project's own
verification) is a two-line wrapper module:

```verilog
module top_1080p30_wrap (/* same ports as dvp_camera_hdmi_top */);
    dvp_camera_hdmi_top #(.RESOLUTION("1080P30")) u_dut (/* ... */);
endmodule
```

and point `synth_ecp5 -top` at the wrapper instead.

---

## Simulating

```sh
make sim
```

runs all five testbenches under Icarus Verilog and prints a `PASS`/`FAIL`
line for each (see [Verification](#verification) for what they check). Each
also writes a `.vcd` waveform you can open in GTKWave for inspection.

Note: there is **no top-level testbench** simulating `dvp_camera_hdmi_top.v`
as a whole. It instantiates several ECP5 hard primitives (`EHXPLLL`,
`ODDRX2F`, `ECLKSYNCB`, `CLKDIVF`) that Icarus Verilog cannot simulate
directly (no timing/functional model without vendor libraries). Every
module was instead verified individually in simulation (functional
correctness) and the complete top-level was verified through real synthesis
+ place-and-route + static timing analysis against the actual chip (physical
implementability and timing closure) — see below. This combination is the
same one professional FPGA teams use for exactly this class of primitive.

---

## Verification

Everything below was actually executed in this repository's own development
environment (`iverilog` 12.0, `yosys` 0.33, `nextpnr-ecp5` 0.6), not
predicted or assumed.

**Simulation (`make sim`), all passing:**

```
TB_ASYNC_FIFO:            PASS  (exact-depth fill/drain + 300-word burst, two independent clocks, 0 errors)
TB_TMDS_ENCODER:          PASS  (1540 exhaustive round-trip checks, 0 errors)
TB_VIDEO_TIMING_GEN:      PASS  (frame period & active-pixel-count exact match, 720p60)
TB_DVP_PIXEL_CHAIN:       PASS  (RGB565 + YUYV422, exact pixel values)
TB_I2C_MASTER:            PASS  (ADDR_BYTES=1 and ADDR_BYTES=2 modes, both against a behavioral I2C slave, all ACKed)
TB_CAM_POWER_SEQUENCER:   PASS  (PWDN/RESET/seq_done ordering and minimum-duration checks, 0 errors)
```

Run all six with `make check` — see [Building](#building).

**Synthesis (`yosys synth_ecp5`), all three top levels, 0 CHECK-pass
problems:**

| Top level | LUT4 | FF | DP16KD (of 56) | EHXPLLL | Notes |
|---|---|---|---|---|---|
| `dvp_camera_hdmi_top` (720p60) | 798 | 545 | 6 | 2 | 3.3% LUT utilization; 2nd `EHXPLLL` is `clk_gen_mclk.v`'s 24MHz camera MCLK |
| `dvp_camera_hdmi_top` (1080p30) | 760 | 553 | 12 | 2 | larger CDC buffer (8192 vs 4096 deep) |
| `dvp_camera_hdmi_top_ext` (1080p60) | 398 | 311 | 12 | 1 | no TMDS gearbox needed; **not yet updated with MCLK/power-sequencer**, see [Known limitations](#known-limitations--future-work) |

(Cell counts for `dvp_camera_hdmi_top` grew from the design's original
720p60-only/8-bit-I²C form after adding `clk_gen_mclk.v` +
`cam_power_sequencer.v` + the wider `ADDR_BYTES`-capable `i2c_master.v` +
the larger 60-entry OV5640 `cam_config_rom.v` table — all still a small
fraction of the LFE5U-25F's ~24,300 LUT4-equivalents and 56 DP16KD
blocks.)

**Place & route + static timing analysis (`nextpnr-ecp5 --25k --package
CABGA256 --speed 6`), against `constraints/icepi_zero.lpf`:**

| Design | Clock domain | Target | Achieved | Result |
|---|---|---|---|---|
| 720p60 | `clk_pixel` | 74.29 MHz | 79.62 MHz (seed 5) | **PASS** |
| 720p60 | `cam_pclk` | 75.00 MHz | 195.73 MHz | **PASS** |
| 720p60 | `cam_mclk` | 24.00 MHz | 177.40 MHz | **PASS** |
| 720p60 | TMDS `sclk` | 185.74 MHz | 199.64 MHz | **PASS** |
| 1080p30 | `clk_pixel` | 74.29 MHz | 77.12 MHz (seed 5) | **PASS** |
| 1080p30 | `cam_pclk` | 75.00 MHz | 186.50 MHz | **PASS** |
| 1080p30 | `cam_mclk` | 24.00 MHz | 178.99 MHz | **PASS** |
| 1080p30 | TMDS `sclk` | 185.74 MHz | 216.17 MHz | **PASS** |
| 1080p60 (ext) | `hdmi_pclk` | 150.01 MHz | 157.18 MHz (seed 6) | **PASS** |
| 1080p60 (ext) | `cam_pclk` | 75.00 MHz | 187.06 MHz | **PASS** |

(720p60 numbers re-verified directly against `make synth` + `make pnr`'s
actual output. 1080p30 numbers verified via a temporary one-file wrapper
instantiating `dvp_camera_hdmi_top` with `RESOLUTION="1080P30"` — see
`make synth_1080p30`'s in-repo note on why the CLI `chparam` approach
doesn't work directly, and build such a wrapper yourself if you need this
configuration; the RTL itself is exactly what ships, this only affects how
its numbers were collected here. `pnr` uses `--seed 5` (re-tuned after
adding the camera-MCLK PLL, power sequencer and widened I²C logic — see
"Timing closure notes" below); `pnr_ext` uses `--seed 6` — the ext top
level is a different, unmodified netlist with its own independently-tuned
best seed, checked with `--lpf-allow-unconstrained` since its `hdmi_*`
pins aren't assigned real sites by default.)

All three designs fully place, route and close timing on the real
`LFE5U-25F-6BG256C` part with the real board pin constraints (or, for the
external-transmitter variant's `hdmi_*` bus, a free-placement timing check —
see Makefile note on assigning those pins to your actual wiring).

---

## Timing closure notes

Two real timing bugs were found and fixed during development by actually
running `nextpnr-ecp5`'s static timing analysis — worth recording here both
as documentation and as a demonstration that these numbers are earned, not
assumed:

1. **`tmds_encoder.v` needed internal pipelining.** A straightforward
   single-cycle implementation of the DVI 8b/10b algorithm (transition
   minimization → running-disparity control, all combinational) measured
   at ~56 MHz max on this part — nowhere near the 74.25 MHz needed. Splitting
   the "minimize transitions" stage and the "DC-balance control" stage
   across a register boundary (one extra clock of latency, functionally
   irrelevant for video) fixed it. This is standard practice in real TMDS
   cores at this pixel rate; a lot of tutorial-grade encoders online skip
   it because they're only ever tested at lower pixel rates (e.g. 25 MHz
   VGA-class), where the unpipelined version is fine.
2. **`video_timing_gen.v` needed internal output registers.** The raw
   combinational sync/active-video decode (two 16-bit magnitude
   comparators) chained directly into whatever a downstream consumer
   (`test_pattern_gen.v`) does with `x`/`y` was too deep for the
   `top_ext` variant's tighter 150 MHz / 6.7 ns budget. Registering
   `hsync`/`vsync`/`de`/`x`/`y` as this module's own output stage bounds
   its worst-case combinational depth independent of whatever's downstream.

**Place-and-route has real run-to-run variance** (`nextpnr-ecp5`'s
timing-driven placer isn't perfectly deterministic across seeds/runs) —
the same design measured anywhere from ~68 MHz to ~83 MHz on `clk_pixel`
across different `--seed` values during testing. The Makefile currently
pins `--seed 5`, empirically checked to give comfortable margin (~7%). If
you modify the RTL and a build reports FAIL, **try a few different
`--seed N` values before concluding the design doesn't fit** — this is
normal FPGA workflow, not a red flag. (Netlist cell ordering -- e.g. which
files get read, and in what order, even ones that turn out unused by a
given top level -- can shift a placer's tie-break heuristics enough to
move the achieved frequency for an otherwise-identical circuit. Always
compare `--seed N` results against the *actual* `make synth`/`make pnr`
output, not a hand-rolled synthesis script with a different file list --
doing that during development briefly looked like a real regression until
re-checking against the real Makefile target showed the original numbers
still held exactly.)

**This seed was re-tuned once already, and will likely need re-tuning
again if you change the RTL.** `--seed 4` was the original pick (before
`clk_gen_mclk.v`, `cam_power_sequencer.v`, and the widened
`ADDR_BYTES`-capable `i2c_master.v`/`cam_config_rom.v` were added for
OV5640 support) and gave ~77.65 MHz with good margin at the time. Adding
that logic shifted the netlist enough that `--seed 4` alone dropped to
74.04 MHz — a hair under the 74.29 MHz target — while every other seed
tried (1, 2, 3, 5–14) still passed comfortably (74.56–79.62 MHz on the
real `make synth` output). This is the run-to-run variance described
above, not a sign the design has gotten marginal — a fresh sweep just
needed to be re-run after growing the netlist, exactly per the advice in
the paragraph above. `--seed 5` was picked as the best margin found in
that fresh sweep and confirmed reproducible across repeated runs.

**Declare-before-use matters for portability, even though standard
Verilog doesn't require it.** This repository's development environment's
Icarus Verilog (12.0) happily elaborates a signal used earlier in a
module than where it's declared, but at least one other real Icarus
build (encountered directly, via a user running `make check` locally)
rejects that with `Unable to bind wire/reg/memory 'X' ... Check for
declaration after use`. Three instances of this existed (`async_fifo.v`'s
`rd_gray_ptr`, `tmds_serial_gearbox.v`'s `fifo_rd_en`, and the same
pattern in `tb_async_fifo.v`'s scoreboard variables) and are now fixed by
moving each declaration ahead of its first use -- a purely textual
reorder with no effect on simulated behavior or synthesized logic
(re-verified: all 6 testbenches still pass, and `make synth`/`make pnr`
still produce the exact same achieved frequencies shown in the table
above). A systematic scan of every `.v` file in `rtl/` and `tb/`
(matching every `reg`/`wire`/`integer`/`genvar` declaration against the
first line each identifier is actually used on) confirms no other
instances remain.

---

## Bring-up checklist

1. `make pnr` — confirm all three clock domains report **PASS**.
2. Flash the bitstream with no camera connected. `led[0]` (PLL lock)
   should light immediately.
3. Hold `button[1]` (pattern-select) — you should see 8 colour bars plus a
   grayscale ramp strip on the display. **This alone proves the entire
   PLL → video-timing → TMDS-encode → GPDI chain works**, independent of any
   camera hardware.
4. Connect the camera, power up. `cam_mclk` should be running immediately
   (it only depends on `clk_gen_mclk.v`'s PLL lock, not on the camera).
   `cam_power_sequencer.v` then drives `cam_pwdn` low and, after the
   configured `RST_MS`/`SETTLE_MS` delays, releases `cam_rst_n` — I²C
   configuration only starts after that sequence completes
   (`cfg_go`/`seq_done`), so expect a short (tens of ms) pause after
   power-up before `led[1]` reacts. `led[1]` (config done) should then
   light once `cam_config_rom` finishes walking its register table — check
   `led[3]` (I²C NACK) is *not* lit; if it is, check wiring/address/
   pull-ups before debugging anything else. If `led[1]` never lights and
   `led[3]` never lights either, suspect the power sequencing itself
   (verify `cam_mclk`/`cam_rst_n`/`cam_pwdn` with a scope/logic analyzer
   against the order in [Wiring the camera](#wiring-the-camera)) rather
   than the I²C transaction.
5. Release `button[1]`. `led[2]` (buffer ready) should light once the CDC
   buffer has pre-filled, and live video should appear.
6. If the image is present but has wrong colours, check `CAMERA_FORMAT`/
   `BYTE_SWAP`. If it's present but rolls/tears, check the camera is
   actually configured for the exact resolution the chosen top level
   expects (see [Adapting to your sensor](#adapting-to-your-sensor) step 4).

---

## Known limitations / future work

- **No scaler.** The camera's active-video window must already match the
  display resolution. A line-buffer-based (or full frame-buffer, using the
  board's onboard SDRAM) scaler is a natural extension.
- **`cam_config_rom.v`'s OV5640 register table hasn't been hardware-tested
  against a real sensor** in this development environment — cross-check
  it against Waveshare's official demo code before relying on it. See
  [Adapting to your sensor](#adapting-to-your-sensor).
- **`dvp_camera_hdmi_top_ext.v` doesn't yet have the MCLK PLL /
  power-sequencer / 16-bit-I²C support** that `dvp_camera_hdmi_top.v` has
  — it still uses the original 8-bit-only I²C addressing and has no
  `cam_mclk`/`cam_rst_n`/`cam_pwdn` ports. If you need true 1080p60 (the
  external-transmitter path) with the OV5640, port the same additions
  from `dvp_camera_hdmi_top.v` across (the three new modules —
  `clk_gen_mclk.v`, `cam_power_sequencer.v`, and the widened
  `i2c_master.v`/`cam_config_rom.v` — are already generic and reusable
  as-is; only the top-level instantiation/wiring needs duplicating).
- **PLL dividers are hand-derived** — cross-check with `ecppll` or Lattice
  Diamond before production use (see [Clocking](#clocking)).
- **No audio.** This is a video-only DVI-class TMDS stream (HDMI-connector
  compatible, displays fine on HDMI sinks, but carries no audio/InfoFrame
  packets). Adding HDMI audio is a substantial further undertaking (audio
  clock regeneration, IEC60958 packetization in the data-island period) and
  out of scope here.
- **No HDCP/EDID handling.** `gpdi_sda`/`gpdi_sck` (the DDC/I²C pair to read
  the monitor's EDID) aren't used by this design; it transmits a fixed,
  known-good timing mode unconditionally rather than negotiating with the
  sink. Works with essentially all real displays in practice (fixed timings
  are exactly what a plain "DVI-D" source does), but isn't spec-complete
  HDMI.

## License

No specific license is asserted here beyond what your project already uses;
treat this code as part of your repository under whatever license governs
it. The TMDS encoding algorithm implemented in `tmds_encoder.v` follows the
publicly documented DVI 1.0 specification (a standard, not proprietary
code) and is independently written and verified as described above.
