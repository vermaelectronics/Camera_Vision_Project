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
7. [UART debug interface](#uart-debug-interface)
8. [Adapting to your sensor](#adapting-to-your-sensor)
9. [Building](#building)
10. [Simulating](#simulating)
11. [Verification](#verification)
12. [Timing closure notes](#timing-closure-notes)
13. [Bring-up checklist](#bring-up-checklist)
14. [Known limitations / future work](#known-limitations--future-work)
15. [License](#license)

---

## What you get

```
dvp_camera_hdmi_pipeline/
├── rtl/                       16 synthesizable Verilog modules (see Module reference)
├── tb/                        8 self-checking testbenches (all passing, see Verification)
├── constraints/
│   └── icepi_zero.lpf         Real pin/site constraints for the IcePi-Zero board
├── Makefile                   sim / synth / pnr / bit / prog targets
└── README.md                  this file
```

Two top-level designs, both built from the same shared RTL blocks:

| Top level | Resolution(s) | Output | Needs |
|---|---|---|---|
| `dvp_camera_hdmi_top.v` | **1280×720@60Hz** (the only resolution this top level supports) | native GPDI/TMDS (the board's onboard mini-HDMI-alike connector) | nothing extra — drives the connector directly; includes a 24MHz MCLK PLL + PWDN/RESET sequencer for the Waveshare OV5640, and a UART debug output |
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

720p60 needs only **74.25 MHz** pixel clock, so its ECLK requirement is
371.25 MHz — comfortably inside the ECP5's real capability, and verified to
place, route and close timing in this repository (see
[Verification](#verification)). (CEA‑861 also defines 1080p at 30/25/24 Hz
using the *same* horizontal/vertical blanking totals as the 60/50/48 Hz
modes, just at half the pixel clock, so 1080p30 would fit the same ECLK
budget as 720p60 — an earlier revision of `dvp_camera_hdmi_top.v` supported
both via a resolution parameter; that was dropped to keep the design fixed
to a single, simpler-to-debug configuration. Re-adding 1080p30 would be a
small change if you want it back — see the header comment history in
`dvp_camera_hdmi_top.v`.)

So, two honest options are provided instead of one design that would quietly
not work on real hardware:

1. **`dvp_camera_hdmi_top.v`** — native GPDI/TMDS output, fixed at 720p60,
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
 DVP camera                                             clk_pixel domain (74.25MHz-class)
 (its own PCLK   ┌──────────────┐   ┌─────────────────┐  ┌──────────────────┐
  domain)    ───▶│ dvp_capture  │──▶│ pixel_formatter │  │ video_timing_gen │──▶ hsync/vsync/de/x/y
  HREF/VSYNC/D[] │ (byte stream)│   │ RGB565/YUYV422  │  └─────────┬────────┘
                 └──────────────┘   │   -> RGB888     │            │
                                     └────────┬────────┘            ▼
                                              │ cam_pixel_valid,   test_pattern_gen
                                              │ cam_rgb[23:0]        (bring-up aid)
                                              ▼                       │
                                     ┌──────────────────┐             ▼
                                     │ video_line_buffer│────▶ pixel mux ──▶ pixel_rgb
                                     │ ("line FIFO": rows│                     │
                                     │  claimed strictly │        ┌───────────┴───────────┐
                                     │  in arrival order,│        ▼                        ▼
                                     │  N_LINES-deep ring│  3x tmds_encoder      (top_ext: straight to
                                     │  of dp_line_ram)  │  (R,G,B, 2-stage       hdmi_d/hdmi_pclk/
                                     └──────────────────┘   pipelined)          hdmi_hsync/vsync/de)
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

(`dvp_camera_hdmi_top_ext.v` still uses the older, one-cycle-latency
`video_cdc_buffer.v` for this same bridge -- see [Known limitations](#known-limitations--future-work).)

Key design decisions, and why:

- **The camera↔pixel-clock crossing is `video_line_buffer.v`, a "line
  FIFO"**: whole rows, claimed by the display strictly in the order the
  camera finished writing them -- never a raw pixel-order stream, and never
  matched by comparing a row *number* between the two independent clock
  domains. An earlier design (`video_cdc_buffer.v`, still used by
  `dvp_camera_hdmi_top_ext.v`) bridged the two clocks as a flat
  `async_fifo` of pixels in capture order, with literally nothing in the
  design tying the display's read position to the camera's actual (row,
  column) position -- and critically, `pixel_formatter.v`'s
  `pixel_line_start`/`pixel_frame_start` outputs were never even wired to
  anything. Because the camera's own internal line/frame blanking timing is
  never configured to exactly match the display's fixed 1280x720@60 timing,
  and the two clocks are genuinely free-running, that design's read/write
  phase relationship inside the FIFO had nothing pinning it to real frame
  boundaries and drifted continuously. **This was found on real hardware,
  not caught in simulation first**: VSYNC/HREF framing correct, I²C
  configuration NACK-free, pixel data actively streaming (confirmed via
  `uart_debug.v`'s `ACT`/`RAW` fields) -- yet the displayed image showed no
  recognizable structure at all, even pointed at a close, high-contrast
  subject. `video_line_buffer.v` replaces it: a display pixel at (x,y)
  always reads column x of whichever camera row most recently finished
  being written -- horizontal alignment is *always* correct, and any
  camera/display line-rate mismatch degrades into, at worst, occasional
  whole-line repeats, never the old design's total spatial incoherence. See
  `video_line_buffer.v`'s own header comment (including a "DESIGN HISTORY"
  section covering three further, more subtle bugs simulation caught before
  this ever reached real hardware) for the full reasoning.

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
| `clk_pixel` (720p60 path) | 74.286 MHz (target 74.25, +0.048%) | `clk_gen_dvi.v` (`EHXPLLL`) | video timing, TMDS encode |
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
| `dp_line_ram.v` | One video line of true dual-port RAM, independent read/write clocks | covered via `tb_video_line_buffer.v` (instantiated N_LINES times by that module) and full-design synthesis + P&R |
| `video_line_buffer.v` | Camera-clock → pixel-clock "line FIFO": rows claimed by the display strictly in arrival order, replacing the older flat-pixel-order `video_cdc_buffer.v` in `dvp_camera_hdmi_top.v` — see [Architecture](#architecture) | `tb_video_line_buffer.v`: deterministic per-(row,col) pattern across a genuine CDC (two unrelated clock periods) with realistic near-matched line rates, interior columns bit-exact, 0 errors |
| `video_cdc_buffer.v` | Camera-clock → pixel-clock elastic buffer (wraps `async_fifo`) — still used by `dvp_camera_hdmi_top_ext.v` only, see [Known limitations](#known-limitations--future-work) | covered via full-design synthesis + P&R (see below) |
| `clk_gen_dvi.v` | `EHXPLLL` wrapper: 50MHz → 74.25MHz-class pixel + 371.25MHz-class ECLK | covered via full-design P&R |
| `clk_gen_1080p60.v` | `EHXPLLL` wrapper: 50MHz → 148.5MHz-class pixel (no ECLK needed) | covered via full-design P&R |
| `tmds_serial_gearbox.v` | ECP5 native TMDS serializer (`ECLKSYNCB`+`CLKDIVF`+`ODDRX2F`) | covered via full-design synthesis + P&R |
| `test_pattern_gen.v` | 8-bar colour bars + ramp, for bring-up without a camera | visual (this is a bring-up aid, not part of the core signal path) |
| `clk_gen_mclk.v` | `EHXPLLL` wrapper: 50MHz → 24.000MHz exact, camera MCLK/XCLK | `tb_cam_power_sequencer.v` exercises the downstream consumer of `locked`; the PLL itself covered via full-design P&R |
| `cam_power_sequencer.v` | PWDN/RESET power-up sequencing state machine (wait-for-MCLK → power-up → reset-release → settle → done) | `tb_cam_power_sequencer.v`: checks strict ordering (PWDN low before RESET release, RESET release before `seq_done`) and that each phase honours its configured minimum duration, 0 errors |
| `i2c_master.v` | Polled I²C master (START/addr+W/reg-addr(8 or 16-bit)/data/STOP, ACK-checked) | `tb_i2c_master.v`: two parallel DUTs (`ADDR_BYTES=1` and `ADDR_BYTES=2`) each run a full transaction against an independent behavioral I²C slave model, all bytes and ACKs verified |
| `cam_config_rom.v` | Walks a register table out through `i2c_master` | covered via full-design synthesis + P&R; **ships with an OV5640-specific table by default now, see below** |
| `uart_tx.v` | Generic 8N1 UART transmitter | `tb_uart_tx.v`: independently-written behavioral receiver checks bit framing/timing and byte value for 3 test bytes, 0 errors |
| `uart_debug.v` | Streams a live hardware-status line (PLL/MCLK lock, I2C config progress, NACK count, buffer-ready, camera mode, live frame counter, capture-activity indicator, raw captured bytes) to a UART terminal, and drives a real `cap_led` output pin for the activity indicator — see [UART debug interface](#uart-debug-interface) | `tb_uart_debug.v`: independently-typed expected banner + status line, byte-for-byte match against a captured UART receive stream, including a real frame-counter CDC crossing, NACK-counter edge-detect, and a check of the real `cap_led` pin (not just the transmitted text), 0 errors |
| `dvp_camera_hdmi_top.v` | Top level: native GPDI, fixed 720p60 — includes MCLK PLL + power sequencer + UART debug, targets the OV5640 out of the box | full synthesis + P&R + timing closure |
| `dvp_camera_hdmi_top_ext.v` | Top level: external-transmitter, true 1080p60 — **not yet updated with MCLK/power-sequencer/16-bit-I²C/UART debug**, see [Known limitations](#known-limitations--future-work) | full synthesis + P&R + timing closure |

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
| `cap_led`   | gpio[19] | 35 |

(`uart_tx` isn't in this table — it's routed to the board's onboard
USB-JTAG programmer chip, not the 40-pin header. See
[UART debug interface](#uart-debug-interface).)

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

**The module's own onboard LED(s) (next to the lens on small breakouts
like this one) aren't in the pin table above — they're not wired to the
FPGA at all.** Confirmed via Waveshare's own official demo code for this
module: they're wired to the OV5640 sensor's own GPIO pin, controlled
entirely over I²C (registers `0x3016`/`0x301C`/`0x3019` — the same
sequence their `OV5640_Flash_Lamp()` function uses). `cam_config_rom.v`
turns it on once configuration completes — see its header comment for the
full explanation and how to remove this if you don't want it lit. This is
a **static** indicator (on once config succeeds, stays on) — for a
**live, real-time** "capture is happening" indicator instead, use
`cap_led` (a genuine FPGA output pin) or the UART debug output's `ACT`
field — see [UART debug interface](#uart-debug-interface).

Do **not** tie `RESET`/`PWDN` to fixed levels or leave them floating —
without `cam_power_sequencer.v` actively driving the documented
power-up order (MCLK stable → PWDN low → hold RESET low briefly → release
RESET → settle → only then start I²C), the sensor's internal PLL/SCCB
interface may never come up reliably.

---

## UART debug interface

`dvp_camera_hdmi_top.v` streams a live, human-readable hardware-status line
out of `uart_tx` once per second, at **115200 baud, 8 data bits, no
parity, 1 stop bit (8N1)** — the same information the 5 status LEDs give
you, plus a live camera-frame counter, all readable in a terminal instead
of interpreted by eye:

```
=== DVP Camera->HDMI Pipeline (720p60, OV5640) -- UART Debug ===
PLL=1 MCLK=1 SEQ=1 CFG=1 NACK=0 BUF=1 MODE=C FRAMES=0x4B ACT=1 RAW=A5C3F02D
PLL=1 MCLK=1 SEQ=1 CFG=1 NACK=0 BUF=1 MODE=C FRAMES=0x4C ACT=1 RAW=91D07AE4
PLL=1 MCLK=1 SEQ=1 CFG=1 NACK=0 BUF=1 MODE=C FRAMES=0x4D ACT=1 RAW=3B2C9F10
...
```

| Field | Meaning |
|---|---|
| `PLL` | pixel-clock PLL locked (mirrors `led[0]`) |
| `MCLK` | camera MCLK PLL locked |
| `SEQ` | `cam_power_sequencer` finished (PWDN/RESET sequencing done) |
| `CFG` | `cam_config_rom` finished walking the I²C register table (mirrors `led[1]`) |
| `NACK` | cumulative count of NACKed I²C transactions this run, one hex digit (saturates at `F`) — `0` means every register write ACKed |
| `BUF` | CDC buffer pre-filled and ready — live video should be visible (mirrors `led[2]`) |
| `MODE` | `C`(amera) or `P`(attern) — mirrors `led[4]`/`pattern_sel` |
| `FRAMES` | hex count of camera VSYNC pulses seen since reset — proves the sensor is actually delivering frames, which the LEDs alone can't show (wraps at `0xFF`; it's a liveness indicator, not a precise frame count) |
| `ACT` | capture-activity indicator: `1` whenever real pixels have been captured (`cam_pixel_valid` pulsing) recently, stretched to a visible duration (200ms default) so it reads as solidly lit during continuous capture. Also driven out to a real pin, `cap_led` — see below. |
| `RAW` | the last 4 bytes actually captured off `cam_d[7:0]` (oldest first), refreshed live -- the only field showing real sensor data content directly rather than a derived status flag. **Stuck at a fixed value (`00000000`, `FFFFFFFF`, or any other constant that never changes)** → the data bus isn't delivering real varying data (wiring or sensor-not-streaming problem). **Visibly varies refresh to refresh** → real bytes are arriving; any remaining image problem is downstream, in decode/format configuration, not the data bus itself. This is the single most useful field for diagnosing "image is garbage" symptoms, since it bypasses the whole decode pipeline and shows exactly what's on the wire. |

**Two different "activity" indicators exist in this design, and it's
worth being clear about what each one actually is:**

- **`cap_led` (gpio[19], header pin 35)** — a real FPGA output pin, lit
  by genuine live pixel-capture activity (`cam_pixel_valid` pulsing,
  stretched to stay visibly lit during continuous capture). This is a
  true real-time indicator: it reflects what's happening *right now*.
  Wire an external LED (+ series resistor, to `GND`) here if you want a
  physical indicator away from the camera module itself — the same
  information is also in the UART's `ACT` field with zero extra wiring.
- **The camera module's own onboard LED**, on modules where it's wired
  to the OV5640's GPIO pin (confirmed for the Waveshare module this
  project targets, via their own official demo code's
  `OV5640_Flash_Lamp()` function — see `cam_config_rom.v`'s header
  comment). `cam_config_rom.v` turns it on with a one-shot register write
  (`0x3016`/`0x301C`/`0x3019`) once configuration completes. This is a
  **static** "camera configured successfully" indicator — it turns on
  once and stays on — not a live per-frame activity light like `cap_led`.
  Making it track real-time activity too would need an ongoing (not
  one-shot) I²C write path, a materially bigger feature than a init-time
  register write. If your specific module's LED isn't wired to the
  sensor's GPIO pin, these three register writes are harmless no-ops for
  you; delete them from `cam_config_rom.v` if you'd rather it stay off.

**Wiring: none needed.** `uart_tx` is routed to the IcePi‑Zero's own
onboard USB‑JTAG programmer chip's UART channel (`LOCATE COMP "uart_tx"
SITE "K15"` in `constraints/icepi_zero.lpf` — that site/IOBUF spec is
taken directly from the board's own published LPF, where it's named
`usb_tx`, "Transmit to ftdi"). That chip exposes **two** interfaces over
the same USB cable you already use for `make prog`/openFPGALoader: the
JTAG/SPI programming channel, and a plain UART bridged straight to your
PC as a second COM port/`/dev/ttyUSB*` device. No external USB‑TTL
adapter, no extra wires — just the one cable.

**Viewing it in PuTTY** (Windows): plug in the board's USB cable, check
Device Manager for the *second* COM port it enumerates (the first is
usually the JTAG/programming interface — if unsure, unplug/replug and
watch which port numbers appear/disappear, or just try both), then in
PuTTY: Connection type = **Serial**, Serial line = that COM port (e.g.
`COM6`), Speed = **115200**, then under Connection → Serial confirm Data
bits = 8, Stop bits = 1, Parity = None, Flow control = None. Click Open.
You should see the banner once at power-up, then a refreshed status line
every second.

On Linux/macOS, the board should enumerate as two devices, e.g.
`/dev/ttyUSB0` (JTAG) and `/dev/ttyUSB1` (UART) — check `dmesg` after
plugging in to see which is which, then: `minicom -D /dev/ttyUSB1 -b
115200` (or `screen /dev/ttyUSB1 115200`).

If nothing appears: you're likely on the wrong COM port/device (try the
other one) — the baud rate/8N1 settings matching exactly also matters (a
wrong baud rate produces garbled/no text, not an error message).

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
7. **Configure your sensor, out-of-band, to output exactly 1280×720@60**
   (`dvp_camera_hdmi_top.v`) **or 1920×1080@60** (`dvp_camera_hdmi_top_ext.v`)
   over its DVP interface. This design does not include a scaler — it assumes the
   camera's active-video window already matches the display resolution
   1:1. (Adding a line-buffer-based scaler is a natural extension point;
   see [Known limitations](#known-limitations--future-work).)

**A word on the shipped OV5640 table's provenance:** its format-select and
ISP-enable registers (`0x4300`, `0x501F`, `0x5000`, `0x5001`, `0x300E`,
and a few others) are now cross-checked directly against Waveshare's own
official demo code for this exact module (their real `ov5640.c`/
`ov5640cfg.h`) — this caught a real bug (`0x4300` was `0x61`, a
community-sourced guess; the vendor's own table uses `0x6F`) and added
several ISP master-enable registers that were missing entirely. The
PLL/analog/timing/output-window registers (`0x3800`–`0x3821`, most likely
to still need adjustment if the image is offset, mirrored, or the wrong
size) remain this project's own independently-derived 720p60 configuration
— the vendor's own RGB565 table targets a different resolution/frame rate
(1280×800 @ 15fps) than this design's 1280×720 @ 60fps, so those values
weren't copied wholesale. See the extensive comments at the top of
`cam_config_rom.v` for the full per-register provenance and rationale.

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
make sim            # run every testbench (should print PASS for all 8)
make synth           # Yosys synthesis, 720p60 top level (the only resolution this design supports)
make pnr               # + nextpnr-ecp5 place&route + timing closure report
make synth_ext          # Yosys synthesis, external-transmitter (true 1080p60) top level
make pnr_ext              # + place&route (edit the LPF's hdmi_* section first, see Makefile)
make bit                    # pack a .bit bitstream (needs ecppack)
make prog                     # flash it (needs openFPGALoader)
```

---

## Simulating

```sh
make sim
```

runs all eight testbenches under Icarus Verilog and prints a `PASS`/`FAIL`
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
TB_UART_TX:               PASS  (bit framing/timing + byte value, independent behavioral receiver, 3 test bytes, 0 errors)
TB_UART_DEBUG:            PASS  (banner + status-line content, frame-counter CDC, NACK-counter edge-detect, raw-byte field, activity-indicator CDC + real cap_led pin check, 0 errors)
TB_VIDEO_LINE_BUFFER:     PASS  (deterministic per-(row,col) pattern across a genuine CDC -- two unrelated clock periods, realistic near-matched line rates -- interior columns bit-exact, 0 errors)
```

Run all nine with `make check` — see [Building](#building).

**Synthesis (`yosys synth_ecp5`), both top levels, 0 CHECK-pass
problems:**

| Top level | LUT4 | FF | DP16KD (of 56) | EHXPLLL | Notes |
|---|---|---|---|---|---|
| `dvp_camera_hdmi_top` (720p60, only resolution supported) | 1666 | 893 | 12 | 2 | 6.9% LUT utilization; 2nd `EHXPLLL` is `clk_gen_mclk.v`'s 24MHz camera MCLK |
| `dvp_camera_hdmi_top_ext` (1080p60) | 398 | 311 | 12 | 1 | no TMDS gearbox needed; **not yet updated with MCLK/power-sequencer/UART debug**, see [Known limitations](#known-limitations--future-work) |

(Cell counts for `dvp_camera_hdmi_top` grew from the design's original
720p60-only/8-bit-I²C form after adding `clk_gen_mclk.v` +
`cam_power_sequencer.v` + the wider `ADDR_BYTES`-capable `i2c_master.v` +
the 73-entry OV5640 `cam_config_rom.v` table + `uart_tx.v`/`uart_debug.v`,
and again after replacing `video_cdc_buffer.v` with `video_line_buffer.v`
(the DP16KD jump from 6 to 12 is mostly `video_line_buffer.v`'s 4
`dp_line_ram.v` instances, ~1280×24bit each) — all still a small fraction
of the LFE5U-25F's ~24,300 LUT4-equivalents and 56 DP16KD blocks. The
1080p30 configuration this table used to also list was removed — see [Why
two different 1080p60 delivery paths](#why-two-different-1080p60-delivery-paths).)

**Place & route + static timing analysis (`nextpnr-ecp5 --25k --package
CABGA256 --speed 6`), against `constraints/icepi_zero.lpf`:**

| Design | Clock domain | Target | Achieved | Result |
|---|---|---|---|---|
| 720p60 | `clk_pixel` | 74.29 MHz | 92.07 MHz (seed 18) | **PASS** |
| 720p60 | `cam_pclk` | 75.00 MHz | 93.89 MHz | **PASS** |
| 720p60 | `cam_mclk` | 24.00 MHz | 185.74 MHz | **PASS** |
| 720p60 | `clk` (board osc, drives `uart_debug`) | 50.00 MHz | 107.35 MHz | **PASS** |
| 720p60 | `cam_vsync` (frame-counter clock edge in `uart_debug`) | 1.00 MHz (documentation-only; real rate ≈60Hz) | 894.45 MHz | **PASS** |
| 720p60 | `cam_pixel_valid` (activity-indicator clock edge in `uart_debug`) | 12.00 MHz (nextpnr auto-inferred default — internal net, not a top-level port, so no LPF `FREQUENCY` constraint applies; real rate is the pixel rate, far above this) | 894.45 MHz | **PASS** |
| 720p60 | TMDS `sclk` | 185.74 MHz | 223.86 MHz | **PASS** |
| 1080p60 (ext) | `hdmi_pclk` | 150.01 MHz | 157.18 MHz (seed 6) | **PASS** |
| 1080p60 (ext) | `cam_pclk` | 75.00 MHz | 187.06 MHz | **PASS** |

(720p60 numbers above are from THIS project's own development-sandbox
toolchain (apt-installed Yosys 0.33), re-measured after
`video_cdc_buffer.v` was replaced with `video_line_buffer.v` in
`dvp_camera_hdmi_top.v` — a big enough netlist/timing change that the
previous round's real-`oss-cad-suite`-toolchain numbers (Yosys 0.68+106,
gathered from a user's real hardware in an earlier round, for the
*previous* netlist) no longer apply and shouldn't be trusted for this one.
`--seed 18` (unchanged from before this netlist change) still closes
timing cleanly here with comfortable margin on every domain, including
`cam_pclk` — which needed a real fix this round: the new
`video_line_buffer.v` write-side back-pressure logic (see its own "DESIGN
HISTORY" comment) initially put a 16-bit Gray-decode+compare chain
straight into the per-pixel write path and dropped `cam_pclk` to a
razor-thin FAIL (~74 vs 75 MHz); registering that check (it only needs to
be fresh once per video line, not once per pixel) fixed it with room to
spare. **This has not yet been re-verified against a real oss-cad-suite
toolchain on real hardware** — do that (and re-sweep `--seed N` if it
reports a FAIL there, exactly as documented in "Timing closure notes")
before trusting these specific numbers for production, the same caveat
this repository has needed every time the netlist changes meaningfully.
`pnr_ext` uses `--seed 6` — the ext top level is a different, unmodified
netlist with its own independently-tuned best seed, checked with
`--lpf-allow-unconstrained` since its `hdmi_*` pins aren't assigned real
sites by default.)

Both designs fully place, route and close timing on the real
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
pins `--seed 18`, empirically checked to give comfortable margin
(`clk_pixel` 82.55 MHz vs. 74.29 MHz target, ~11%; `sclk` 226.71 MHz vs.
185.74 MHz target, ~22%) — see episode 7 below for how this was found. If
you modify the RTL *or the LPF* and a build reports FAIL, **try a few
different `--seed N` values before concluding the design doesn't fit** —
this is normal FPGA workflow, not a red flag. (Netlist cell ordering --
e.g. which files get read, and in what order, even ones that turn out
unused by a given top level -- can shift a placer's tie-break heuristics
enough to move the achieved frequency for an otherwise-identical circuit;
a pin/site reassignment in the LPF can do the same, since it changes where
the placer has to route to, not just what it's routing. Always compare
`--seed N` results against the *actual* `make synth`/`make pnr` output,
not a hand-rolled synthesis script with a different file list -- doing
that during development briefly looked like a real regression until
re-checking against the real Makefile target showed the original numbers
still held exactly.)

**This seed has been re-tuned eight times already, and will likely need
re-tuning again if you change the RTL or LPF.** `--seed 4` was the
original pick (before `clk_gen_mclk.v`, `cam_power_sequencer.v`, and the
widened `ADDR_BYTES`-capable `i2c_master.v`/`cam_config_rom.v` were added
for OV5640 support) and gave ~77.65 MHz with good margin at the time.
Adding that logic shifted the netlist enough that `--seed 4` alone
dropped to 74.04 MHz — a hair under the 74.29 MHz target — while every
other seed tried still passed comfortably, and `--seed 5` was picked as
the best margin found in that sweep (79.62 MHz). Adding `uart_tx.v` +
`uart_debug.v` (and dropping the unused 1080p30 `RESOLUTION` branch) grew
the netlist again, and a fresh sweep landed on `--seed 2` (83.51 MHz).
Moving `uart_tx` from a generic `gpio[19]` header site to the board's
dedicated onboard-FTDI UART site (`K15`, so the status output works over
the same USB cable used for flashing, with no external adapter) shifted
routing enough that `--seed 2` alone dropped to a thinner ~78 MHz margin;
a fresh sweep against that exact LPF change landed on `--seed 5` (82.45
MHz) -- a coincidental seed-number repeat from an earlier, unrelated
sweep, not a sign anything reverted. Adding the `RAW` raw-byte diagnostic
field to `uart_debug.v` (a 32-bit synchronizer plus 8 more hex digits of
status-line logic) grew the netlist again, and a fresh sweep landed on
`--seed 10` (79.06 MHz). Adding the `ACT` capture-activity indicator +
`cap_led` output pin (another toggle+2FF+edge-detect CDC path plus a
stretch timer, mirroring the frame counter's own technique) grew the
netlist once more and also added a new I/O pin, shifting placement enough
that every seed's margin thinned noticeably (the sweep this time ranged
73.99–78.47 MHz rather than the wider spread seen in earlier sweeps); a
fresh sweep against this exact netlist landed on `--seed 1` (78.47 MHz).
Cross-checking `cam_config_rom.v`'s format/ISP-enable registers against
Waveshare's vendor code and adding the module-LED control registers grew
the table by 12 entries; `--seed 1` alone dropped to a razor-thin 74.67
MHz (0.5% margin) on this new netlist, so a fresh sweep was run and
landed on `--seed 14` (81.71 MHz).

**Episode 7 — toolchain-version variance, not just seed variance.** All
six episodes above were swept and verified on this project's own
development sandbox, whose `apt`-installed Yosys is version 0.33 — quite
old. A user building this exact repo on a real, current `oss-cad-suite`
install (Yosys 0.68+106, a current `nextpnr-ecp5`) ran `make pnr` with the
`--seed 14` pick from episode 6 and hit a **FAIL**: `Max frequency for
clock '$glbnet$u_ser.sclk': 185.70 MHz (FAIL at 185.74 MHz)` — a razor-thin
0.02% miss. Nothing about the RTL, LPF, or netlist had changed; the
*toolchain version* itself was the variable. Different Yosys/nextpnr-ecp5
releases use different (and improving, over time) placement/routing
heuristics, so a `--seed N` value swept on one version's placer is not
guaranteed to reproduce on another's — the seed only pins the RNG *within*
a given placer's algorithm, not the algorithm itself. (Separately: this
FAIL also exposed that `make bit`'s `nextpnr-ecp5 ... | tee ...log`
pipeline doesn't propagate nextpnr's exit code through `tee` to `make`, so
a timing FAIL here did **not** stop `ecppack` from still packing a `.bit`
file — flashing that specific bitstream would have been a mistake. Treat
any `FAIL` line in `pnr`'s console output as a hard stop regardless of
whether `make` itself reports an error, until this pipeline gotcha is
fixed.) The user re-swept seeds 1–20 directly on their own real toolchain
and pasted the complete results back; `--seed 18` was the best margin
found (`sclk` 226.71 MHz, `clk_pixel` 82.55 MHz — both comfortably above
target), confirmed reproducible with a second, independent `nextpnr-ecp5`
run on the same machine (`sclk` 229.94/226.71 MHz and `clk_pixel`
77.48/82.55 MHz across the two runs — both **PASS**, run-to-run variance
within a single toolchain version being much smaller than variance across
versions). The Makefile and the Verification table above now reflect
`--seed 18`, sourced directly from that real-toolchain output. **The
practical lesson: if you're building on a different `oss-cad-suite`/Yosys/
nextpnr-ecp5 version than whatever produced the seed pinned in this repo
at the time you're reading it, re-sweep seeds on your own toolchain before
trusting a FAIL (or a thin PASS) as final** — this is a toolchain-portability
issue, not evidence the design itself is marginal.

**Episode 8 — a new clock domain failure, from a genuine combinational-depth
bug, not placement variance.** Replacing `video_cdc_buffer.v` with
`video_line_buffer.v` (see [Architecture](#architecture)) grew the netlist
again and, on this sandbox's own toolchain, `--seed 18` alone dropped
`cam_pclk` to a real **FAIL** (~74 vs. 75 MHz) — a *different* clock domain
than any previous episode had ever touched. Unlike episodes 1–6 (placement
variance) this one had a real root cause: `video_line_buffer.v`'s new
write-side back-pressure logic fed a 16-bit Gray-decode-plus-compare chain
straight into the same-cycle path that gates every pixel write, needlessly
putting a check that only needs to be fresh once per video *line* in series
with logic that runs once per *pixel*. Registering that check (one cycle of
staleness on a signal with generous margin to spare) removed it from the
critical path entirely and brought `cam_pclk` back to a comfortable PASS
(93.89 MHz) with `--seed 18` unchanged — no re-sweep was needed once the
actual combinational-depth problem was fixed. The lesson generalizes: a
timing FAIL after adding logic to a clock domain that previously had
comfortable margin is worth a first look at what got added to *that specific
domain's* combinational depth before reaching for a seed sweep — a seed
sweep fixes placement-variance FAILs, not a genuine new critical path.

Each of these episodes was the same run-to-run (and now, run-to-toolchain)
variance described above working as intended, not a sign the design has
gotten marginal — re-sweep after any netlist- or placement-shifting
change, or any toolchain-version change, exactly per the advice in the
paragraph above (and check for an actual new critical path first, per
episode 8, before assuming a sweep is even the right fix).

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

1. `make pnr` — confirm every clock domain reports **PASS** (see the
   [Verification](#verification) table for the full list).
2. (Optional, but recommended — and no extra wiring needed, see [UART
   debug interface](#uart-debug-interface)) open PuTTY/minicom on the
   board's second COM port before flashing. The status line gives you a
   live, readable view of everything the LEDs show plus a frame counter,
   which makes the rest of this checklist much faster to diagnose than
   reading LEDs alone.
3. Flash the bitstream with no camera connected. `led[0]` (PLL lock)
   should light immediately; the UART banner should print, followed by a
   status line showing `PLL=1` and everything else `0`.
4. Hold `button[1]` (pattern-select) — you should see 8 colour bars plus a
   grayscale ramp strip on the display. **This alone proves the entire
   PLL → video-timing → TMDS-encode → GPDI chain works**, independent of any
   camera hardware.
5. Connect the camera, power up. `cam_mclk` should be running immediately
   (it only depends on `clk_gen_mclk.v`'s PLL lock, not on the camera).
   `cam_power_sequencer.v` then drives `cam_pwdn` low and, after the
   configured `RST_MS`/`SETTLE_MS` delays, releases `cam_rst_n` — I²C
   configuration only starts after that sequence completes
   (`cfg_go`/`seq_done`), so expect a short (tens of ms) pause after
   power-up before `led[1]`/`CFG=1` reacts. `led[1]` (config done) /
   `CFG=1` should then light once `cam_config_rom` finishes walking its
   register table — check `led[3]`/`NACK=0` (no NACKed transactions); if
   `NACK` is nonzero, check wiring/address/pull-ups before debugging
   anything else. If `CFG` never goes to `1` and `NACK` stays `0` too,
   suspect the power sequencing itself (`SEQ` should read `1` once
   `cam_power_sequencer.v` finishes — if it doesn't, verify
   `cam_mclk`/`cam_rst_n`/`cam_pwdn` with a scope/logic analyzer against
   the order in [Wiring the camera](#wiring-the-camera)) rather than the
   I²C transaction itself.
6. Release `button[1]`. `led[2]`/`BUF=1` should light once the CDC buffer
   has pre-filled, and live video should appear. Watch `FRAMES` in the
   UART output tick up — if it's incrementing but the display is still
   wrong/blank, the problem is downstream of capture (formatting/register
   configuration), not the DVP wiring; if `FRAMES` stays at `0x00`, the
   problem is upstream (PCLK/HREF/VSYNC/data wiring or polarity).
7. If the image is present but has wrong colours, check `CAMERA_FORMAT`/
   `BYTE_SWAP`. If it's present but rolls/tears, check the camera is
   actually configured for the exact resolution the chosen top level
   expects (see [Adapting to your sensor](#adapting-to-your-sensor) step 7).

---

## Known limitations / future work

- **No scaler.** The camera's active-video window must already match the
  display resolution. A line-buffer-based (or full frame-buffer, using the
  board's onboard SDRAM) scaler is a natural extension.
- **`cam_config_rom.v`'s format-select and ISP-enable registers (0x4300,
  0x501F, 0x5000, 0x5001, 0x300E, and others) are now cross-checked
  against Waveshare's own official demo code** for this exact module
  (their real `ov5640.c`/`ov5640cfg.h`) — this fixed a real bug (0x4300
  was 0x61, a community guess; the vendor's own table uses 0x6F) and
  added several ISP master-enable registers that were missing entirely.
  The PLL/analog/timing/output-window registers are still this project's
  own independently-derived values (the vendor's own RGB565 table targets
  a different resolution/frame rate than this design's 720p60), so they
  remain reproduced from general community knowledge, not vendor-sourced.
  See [Adapting to your sensor](#adapting-to-your-sensor) and the header
  comment in `cam_config_rom.v` for the full provenance breakdown.
- **`dvp_camera_hdmi_top_ext.v` doesn't yet have the MCLK PLL /
  power-sequencer / 16-bit-I²C / UART debug support** that
  `dvp_camera_hdmi_top.v` has — it still uses the original 8-bit-only I²C
  addressing and has no `cam_mclk`/`cam_rst_n`/`cam_pwdn`/`uart_tx` ports.
  If you need true 1080p60 (the external-transmitter path) with the
  OV5640, port the same additions from `dvp_camera_hdmi_top.v` across (the
  five new modules — `clk_gen_mclk.v`, `cam_power_sequencer.v`,
  `uart_tx.v`, `uart_debug.v`, and the widened `i2c_master.v`/
  `cam_config_rom.v` — are already generic and reusable as-is; only the
  top-level instantiation/wiring needs duplicating).
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
