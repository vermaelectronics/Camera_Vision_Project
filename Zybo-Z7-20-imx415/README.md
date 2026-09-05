# Zybo Z7-20 + Sony IMX415 — Bare-metal Vitis Application

This folder is an adaptation of Digilent's **Zybo-Z7-20-pcam-5c** bare-metal
Vitis reference application (originally written for the **OV5640** sensor on
the Pcam 5C module) so that it talks to a **Sony IMX415** MIPI CSI-2 image
sensor instead.

**Scope of this change: software only.** I edited the C/C++ Vitis
application (the `src/` tree you get when you export a Vitis "system
project" for the ARM Cortex-A9 on the Zynq-7000). I did **not** — and, given
only this software export, **could not** — regenerate the FPGA bitstream
(`system_wrapper.bit`) or the Vivado hardware design that the original
project was built against. Read **§0** (a likely reset-wiring gap on your
specific board), **§3 "Hardware side"**, and **§4 "Demosaic and
resolution"** below before you power anything up.

## 0. Your hardware — now confirmed from the datasheet + schematic

You provided the Sony **IMX415-AAQR-C datasheet** and the **schematic for
the "IMX415 CAM R1" board** (`SCH_IMX415_MIPI_FFC_CAM_REV1.pdf`), on top of
the earlier photos of it plugged into a Zybo Z7-20 via a Raspberry-Pi-style
"Standard-Mini" adapter cable into the board's Pcam MIPI connector. That
resolved almost everything §0 used to flag as an assumption:

* **I2C address is 0x37, not 0x1A.** The schematic's silkscreened note and
  its resistor strapping (both address-select pins pulled low) both pointed
  at 0x1A, and I trusted that in earlier passes. **You then measured the
  actual assembled board and found SLAMODE0/SLAMODE1 are both HIGH** —
  per the datasheet's SLAMODE0/SLAMODE1 slave-address truth table, that's
  `0110111`b = **0x37** (7-bit), a completely different address than the
  schematic implied. Real hardware measurement overrides schematic
  inference here — `IMX415::dev_address_` in `IMX415.h` is now `0x37`.
  (Plausible explanations for the mismatch: a board rework, a differently
  -stuffed resistor than the schematic shows, or this unit being a
  different revision than the schematic — I can't tell which from here,
  and it doesn't change what to do about it.)
* **2-lane operation is confirmed correct**, and *why* is now concrete
  rather than inferred from connector-standard folklore: this board's 22-pin
  FPC connector carries all 4 of the sensor's CSI-2 lanes, but per the
  datasheet, "In 2 Lane mode, data is output from Lane1 and Lane2" —  and
  the 15-pin "Mini" connector standard the Zybo's Pcam header uses only ever
  carries 2 of those lanes through. `NUM_DATA_LANES = 2` in `IMX415.h` is
  correct for this cable/connector combination.
* **INCK is generated on the camera board itself**, not derived from the
  Zybo: an always-on active oscillator (Kyocera part `X1G0048010002`, wired
  straight to the 1.8V rail with its enable pin tied high — no host control
  at all) feeds the sensor's INCK pin directly. **You confirmed it's 24MHz**
  (one of only 5 frequencies the datasheet says the sensor supports at
  all). `IMX415_cfg::INCK_HZ` in `IMX415.h` is set to this. That also meant
  the driver's original "720 or 891 Mbps/lane" pairing was wrong for this
  board — 891Mbps has no 24MHz option in the datasheet at all — so the two
  lane-rate modes are now **720 and 1440 Mbps/lane**, both genuinely valid
  at 24MHz. See §2.
* **A real, concrete, likely bring-up blocker was found**: this board's
  connector separates `CAM_RST` (which actually drives the sensor's reset
  pin) from `CAM_GPIO` (which, on this board, connects to nothing but a bare
  test point). The Zybo's single existing Pcam GPIO pin conventionally
  reaches the connector's `CAM_GPIO` position, not `CAM_RST` — meaning the
  driver's existing `reset()` may well be toggling a pin that goes nowhere
  on this board, leaving the sensor's actual reset line floating. See §5 —
  this is now the single most likely reason bring-up would fail, more
  likely than any lane/timing issue.
* **Your actual Vivado hardware design is not the bare capture-only
  pipeline** I originally analyzed from the `.xsa` bundled in the first IDE
  zip. You confirmed it's the real Zybo-Z7-20-pcam-5c block design, and
  once you pointed me at Digilent's actual reference page I found the
  real source for it — [`Digilent/Zybo-Z7-20-pcam-5c`](https://github.com/Digilent/Zybo-Z7-20-pcam-5c)
  on GitHub (archived, but the HDL/TCL/constraints are all there). Reading
  the real source instead of just the block-diagram images resolved the
  two biggest open hardware questions with certainty — and both turned out
  to need real Vivado work, not just a settings check. See §3/§4.

All of `IMX415.h`'s register *values* were also independently cross-checked
byte-for-byte against the datasheet in §2 below — see there for the one
inconsistency found (in Sony's own document, not in this port).

## 1. What actually changed

| Original (OV5640 / Pcam 5C)                          | This project (IMX415)                                        | Why |
|--------------------------------------------------------|----------------------------------------------------------------|-----|
| `src/ov5640/OV5640.h`, `OV5640.cpp`                     | `src/imx415/IMX415.h`, `IMX415.cpp`                             | New driver class, register map, and register *values* — see §2 |
| `src/main.cc` — includes `ov5640/OV5640.h`, instantiates `OV5640 cam(...)` | `src/main.cc` — includes `imx415/IMX415.h`, instantiates `IMX415 cam(...)` | Swap the driver actually used by the app |
| Menu: **a. Change Resolution** (720p/1080p15/1080p30) | Menu: **a. Change MIPI Lane Rate** (720/1440 Mbps per lane) | The IMX415 has one native readout size (full sensor array) — see §2. There's no sensor-side resolution to pick, only the lane rate the same fixed-size frame is clocked out at. |
| Menu: **b. Change Liquid Lens Focus** | **removed** | That's the Pcam 5C's variable-focus liquid-lens IC, a separate chip on *that* board. Your "IMX415 CAM R1" module has a fixed M12 lens, not a liquid lens. |
| Menu: **d. Change Image Format (Raw or RGB)**, **h. Change AWB Settings** | **removed** | Both were OV5640-internal-ISP features (Bayer→RGB conversion, auto white balance) controlled purely by I2C register writes to the sensor. The IMX415 has no on-sensor ISP, so there's nothing to write — the equivalent functionality (demosaic) now lives in your FPGA's `AXI_BayerToRGB` core instead — see §4. |
| Menu: **e/f** (write/read sensor register) | kept, renumbered **b/c** | Still very useful for IMX415 bring-up/debug. |
| Menu: **g** (gamma factor) | kept, renumbered **d** | Drives the FPGA's `AXI_GammaCorrection` core, not the sensor — unrelated to which camera is attached. Same core as before, still directly usable once a real image is flowing through it. |
| Live HDMI preview wired up in `pipeline_mode_change()` | **Live HDMI preview wired up in `main()`** instead, brought up once rather than inside `pipeline_mode_change()` | Resolution doesn't depend on MIPI lane rate, so it doesn't need re-locking the video clock on every menu-driven mode change. See §4 for the new `Resolution::R2040_2192_24_NP` timing this needed. |
| `ov5640/PS_IIC.h`, `PS_GPIO.h`, `I2C_Client.h`, `GPIO_Client.h`, `ScuGicInterruptController.h`, `AXI_VDMA.h` | copied to `imx415/` **unchanged**, only the folder moved | Generic Zynq PS peripheral drivers (I2C, GPIO, interrupt controller, VDMA) — not sensor-specific. |
| `hdmi/VideoOutput.h`, `platform/*`, `lscript.ld`, `Xilinx.spec` | `VideoOutput.h` gained one new `Resolution` entry (§4); the rest unchanged | Generic HDMI-timing / Zynq PS bring-up / linker infrastructure, independent of sensor choice — `VideoOutput.h`'s existing table-driven design just needed a new row for IMX415's non-standard cropped resolution. |
| `.project` / `.cproject` | renamed to `Zybo-Z7-20-imx415`, cleaned of stale absolute developer paths | — |

## 2. Where the IMX415 register data comes from (important)

The first version of this port used placeholder register values with
explicit `TODO/VERIFY` markers, because I didn't want to present guessed
numbers as trustworthy. I then found something much better — the
**mainline Linux kernel has a real, maintained IMX415 driver**,
[`drivers/media/i2c/imx415.c`](https://github.com/torvalds/linux/blob/master/drivers/media/i2c/imx415.c)
(GPL-2.0-only, © 2023 WolfVision GmbH) — and ported its actual register
addresses, its ~76-entry "magic"/undocumented analog tuning table, its
per-lane-rate MIPI D-PHY timing tables, and its per-(lane-rate, INCK)
clock-configuration tables into `IMX415.h`, as plain numeric configuration
data.

**Since then, with the official Sony IMX415-AAQR-C datasheet you provided in
hand, I cross-checked every register/value pair used here (the 720Mbps,
891Mbps and 1440Mbps clock-configuration and D-PHY-timing tables) against
the datasheet's own "INCK Setting" section, byte-for-byte. Every value
matches exactly**, confirming the Linux-driver port was accurate — this is
no longer just "a shipping driver's values," it's independently verified
against Sony's primary source. (891Mbps ended up not being usable on this
board once the INCK was confirmed — see below — but the cross-check stands
as evidence the port itself is correct, not just the two values actually
wired up.)

**You confirmed the board's INCK is 24MHz.** That closes out the last
board-specific unknown from §0 — but it also means the driver's original
"720 or 891 Mbps/lane" pairing was wrong for this board: per the datasheet's
own INCK-Setting tables, **891Mbps/lane has no 24MHz option at all** (only
27/37.125/74.25MHz). The two lane-rate modes are now **720Mbps and
1440Mbps**, both genuinely valid at 24MHz per the datasheet, and both
cross-checked the same way. The 891Mbps tables are kept in `IMX415_cfg` as
reference material (useful if you ever pair this driver with a
27/37.125/74.25MHz-INCK board) but are no longer wired into `set_mode()`.

**One inconsistency turned up, in the datasheet itself, not in this port**:
its master "Register Map" section places `SYS_MODE` at address `0x3033`
(matching the Linux driver), but its own "INCK Setting" summary tables say
`0x3034` for the same register — eight times, consistently, in that one
section. This driver uses `0x3033` (two independent sources agree over
one). If bring-up gets past the chip-ID check but timing looks wrong,
try `0x3034` as a troubleshooting step — see the comment on `REG_SYS_MODE`
in `IMX415.h`.

I also pulled Sony's official **Power-on Sequence** timing table (exact
minimums: `XCLR` held low ≥500ns after power stable, ≥1µs before `INCK`
needs to be running, ≥20µs before the first I2C transaction, ≥24ms after
leaving standby before the image stabilizes) into the code comments in
`IMX415.h`'s `reset()`/`init()`, replacing the earlier version's vaguer
"the Linux driver empirically uses ~80ms" phrasing with Sony's actual
numbers.

This also clarified two things the first version of this port got wrong:

* **The IMX415 has exactly one native readout mode implemented here**: a
  full-pixel-array raw Bayer scan at **3864×2192, RAW10** (`WINMODE` stays
  0 — "all-pixel readout" — always). There is no real "1080p mode" or "4K
  mode" to select, unlike what the first version of this README implied.
  (The sensor hardware *does* also support a window-cropping mode and
  2/2-line binning per the datasheet — this driver just doesn't implement
  them, matching the upstream Linux driver's scope.) What *is* selectable
  is the **MIPI lane rate** (720 or 1440 Mbps/lane, both confirmed valid at
  this board's 24MHz INCK and both datasheet-verified — see above) and the
  **lane count** (2 or 4 — see §0 for why 2 is correct for this board).
* **There's a real, documented chip-ID register**: `SENSOR_INFO` at
  `0x3F12` (16-bit), masked `0xFFF`, expected `0x514`. `IMX415::init()`
  checks this (matching the OV5640 driver's own ID-check pattern) instead
  of the earlier version's indirect standby-readback heuristic.

### What's still genuinely unresolved

| Item | Where | Status |
|---|---|---|
| **Sensor reset wiring (`CAM_RST` vs `CAM_GPIO`)** | `IMX415::reset()` in `IMX415.h` | Likely gap, not yet fixed in code — see §5, this is the top bring-up risk. |
| **Vivado D-PHY/CSI-2 RX IP configuration** | Hardware design (not in this software export) | Still unconfirmed whether it's built for 720 or 1440 Mbps/lane, or something else entirely — see §3. 1440Mbps in particular is a real step up from the OV5640-era IP's ballpark. |

None of these being wrong will damage the sensor — worst case is no image,
garbled data, or an I2C NACK.

## 3. Hardware side — what you still need to do

**Which hardware design applies to you matters a lot here, and there are
two candidates:**

1. The `.xsa`/bitstream actually bundled in the original Vitis IDE zip you
   first gave me — a bare capture-only pipeline (`MIPI_D_PHY_RX →
   MIPI_CSI_2_RX → AXI_VDMA → VTC → HDMI TX`, no image processing IP at
   all). Its `xparameters.h` lists only `MIPI_D_PHY_RX_0`, `MIPI_CSI_2_RX_0`,
   `AXI_VDMA_0`, `VIDEO_DYNCLK`, `AXI_GAMMACORRECTION_0`, `VTC_0`.
2. **Your actual Vivado source project** (block design you shared): `MIPI_
   CSI_2_RX_0 → AXI_BayerToRGB_1 → AXI_GammaCorrection_0 → AXI_VDMA →
   [DDR] → AXI_VDMA → v_axi4s_vid_out_0 → rgb2dvi_0 → HDMI`, plus
   `DVIClocking_0` alongside the clocking-wizard (`video_dynclk`). This is
   the one you confirmed is real and what you're building from.

Since **#2 is your actual hardware**, treat the rest of this section (and
§4) as being about that design, not #1.

I found the actual Digilent source for this design —
[`Digilent/Zybo-Z7-20-pcam-5c`](https://github.com/Digilent/Zybo-Z7-20-pcam-5c)
on GitHub (archived/no-longer-maintained, but the source is all there) — and
went through the real HDL/TCL/constraints instead of just the block-diagram
images. That resolved the two biggest open items with certainty, and the
answer to both is more work than "just check a setting":

1. **D-PHY line rate — CONFIRMED, fixed, and re-implemented. ✅ Done.**
   `src/constraints/timing.xdc` originally contained:
   ```
   # MIPI D-PHY data rate 420Mbps/lane = 210 MHz HS_Clk
   create_clock -period 4.761 -name dphy_hs_clock_p -waveform {0.000 2.380} ...
   ```
   That was the **only** rate this bitstream's timing closure had ever been
   verified against — exactly the OV5640's default boot-mode rate. Neither
   IMX415 option (720/1440Mbps/lane at this board's confirmed 24MHz INCK)
   matched it.

   **Fixed and implemented for 720Mbps/lane:**
   ```
   # MIPI D-PHY data rate 720Mbps/lane = 360 MHz HS_Clk
   create_clock -period 2.778 -name dphy_hs_clock_p -waveform {0.000 1.389} ...
   ```
   Both numbers had to change, not just the period — the waveform's second
   value must be half the period for a correct 50% duty cycle (`2.778/2 =
   1.389`); the first pass at this fix updated the period but left the old
   `2.380` waveform edge in place, which would have told Vivado's static
   timing analysis the HS clock had an ~86/14 duty cycle instead of the
   real symmetric one. Re-synthesized and re-implemented successfully.
   1440Mbps/lane is the same edit again (`1.389`/`0.6945`) whenever you
   move to that stage.
2. **`AXI_BayerToRGB`'s Bayer/CFA phase — CONFIRMED from both the VHDL and
   the datasheet, mismatched, fixed, and re-implemented. ✅ Done.** The
   VHDL's `AssignOutputs` process (`repo/local/ip/AXI_BayerToRGB/hdl/
   AXI_BayerToRGB.vhd`, Digilent/Ioan Catuna, MIT-licensed) has no AXI4-Lite
   control port at all — the phase is a compile-time `case` statement.
   Tracing it against `sCrntPositionIndicator` (line-parity, column-parity)
   shows it assumes **BGGR** — position (even,even) is Blue.

   The IMX415-AAQR-C datasheet's own "Color Coding of Physical Pixel
   Array" diagram says otherwise: row 0 reads `Gb, B, Gb, B…`, row 1 reads
   `R, Gr, R, Gr…` — position (0,0) is **Gb, a Green pixel**. That's
   **GBRG**, one column shifted from what the VHDL assumes. A second
   datasheet figure ("Window Cropping Mode") independently confirms it at
   the sensor's own crop-window origin, and Sony states directly: *"The
   first readout pixel color is G."*

   **The fix, applied and implemented:** flip the column-parity bit
   feeding the case selector —
   ```vhdl
   case (sCrntPositionIndicatorDly3 xor "01") is   -- was: case sCrntPositionIndicatorDly3 is
   ```
   one line, in `AssignOutputs`. Nothing else in the file changed — same
   four `when` branches, same AXI handshaking, same line-buffer logic.
   This assumes the sensor's crop window stays at full-array default so
   pixel (0,0) delivered over MIPI lines up with the datasheet's native
   (0,0) — true given point 3's fix, since the crop below is horizontal
   only and IMX415's window-cropping origin still lines up with the
   datasheet's native (0,0).
3. **`AXI_BayerToRGB`'s line-buffer width limit — CONFIRMED and FIXED,
   via a sensor-side register crop. ✅ Done.** Its own header comment
   says it plainly: `Maximum resolution: 2048 x <any value> pixels`.
   That's backed by the RTL, not just the comment: `sCntColumns` and
   `sLineBufferCrntAddr` are both `UNSIGNED(10 downto 0)` (11 bits,
   0–2047), and `LineBufferInst` instantiates `LineBuffer.vhd`'s RAM with
   `kLineBufferWidth => 2048` to match. IMX415's native width is
   **3864px** — 1816px past that limit. Since this block sits ahead of
   the VDMA write side, this would corrupt the DDR-captured frame too:
   at column 2048 the 11-bit counter wraps to 0 while the real line
   still has 1816 columns left, so the tail of every line overwrites the
   line-buffer addresses its own head just wrote.

   Two independent fixes existed; **this driver now implements the
   sensor-side crop:**
   * **Sensor-side crop (chosen — no Vivado resynthesis of this
     block):** `IMX415.h` now sets `REG_WINMODE=0x04` (Window Cropping
     mode, was `0x00`/all-pixel) plus `REG_PIX_HST=912`/
     `REG_PIX_HWIDTH=2040` (`IMX415_cfg::CROP_HSTART`/`CROP_WIDTH`) —
     centering a 2040px-wide crop (the largest multiple of 24, the
     register's own hardware constraint, that's still ≤2048px) in the
     3864px array. `main.cc`'s `vdma_driver.configureWrite()` was
     updated to match — it now passes `CROP_WIDTH`, not
     `PIXEL_ARRAY_WIDTH`, since that's what the sensor actually streams
     once cropped. `PIXEL_ARRAY_WIDTH` itself is untouched — it still
     correctly describes the sensor's true physical array size, just no
     longer what you tell VDMA. Vertical is left uncropped (full
     2192-line height; `PIX_VST`/`PIX_VWIDTH` are simply never written,
     staying at their power-on defaults) — there's no equivalent height
     limit, and the datasheet's `VMAX ≥ (PIX_VWIDTH/2)+46 = 2238`
     restriction for full height was already satisfied by the existing
     `VMAX_DEFAULT` (2250) before this change.
   * **Widen the block itself instead (real RTL change, full
     alternative — not what this driver does, but valid if you'd rather
     keep the sensor at full resolution):** in `LineBuffer.vhd`, widen
     `pWriteAddr`/`pReadAddr` from `STD_LOGIC_VECTOR(10 downto 0)` to
     `(11 downto 0)` and bump the generic default to `4096`; in
     `AXI_BayerToRGB.vhd`, widen `sCntColumns`, `sLineBufferWriteAddr`,
     `sLineBufferReadAddr`, and `sLineBufferCrntAddr` from 11 to 12
     bits, and change `LineBufferInst`'s
     `generic map(kLineBufferWidth => 2048)` to `4096`. Leave
     `sCntLines` alone — it's only ever read as its bit-0 parity, which
     stays correct through overflow regardless of width. Nothing else
     in either file changes, and neither file's AXI4-Stream port list
     changes, so nothing upstream or downstream needs touching. If you
     go this route instead, revert `IMX415.h`'s `REG_WINMODE`/
     `PIX_HST`/`PIX_HWIDTH` writes back to full-array (`WINMODE=0x00`,
     drop the two new register writes) and change `main.cc` back to
     `PIXEL_ARRAY_WIDTH`.
     Needs a full resynthesis of `AXI_BayerToRGB_1` (real hardware
     change, not just a constraint), though the resource cost is trivial
     on this device (a few KB of BRAM).
4. **Output pixel format — known precisely, worth knowing if you process
   the DDR buffer yourself.** `AXI_BayerToRGB`'s output is **not**
   8-bit RGB888. Per its VHDL: 32-bit words, packed as `[31:30]` unused,
   `[29:20]`=Red (10-bit), `[19:10]`=Blue (10-bit), `[9:0]`=Green (10-bit,
   already averaged/scaled from the two green samples in each Bayer
   block). Input side takes 4 Bayer samples/clock, 10 bits each (40-bit
   `s_axis_video_tdata`), RAW10 — matching what this driver already
   configures (`ADBIT`/`MDBIT` = RAW10). Confirm `AXI_GammaCorrection`/
   `AXI_VDMA` downstream are built for this exact 32-bit-word/10-bit-per-
   channel layout (they should be, since it's what the OV5640-era design
   already used) rather than assuming standard 24-bit RGB888.
5. Re-export the hardware platform (**File → Export → Export Hardware**,
   include bitstream) and re-associate this application's `system_wrapper`
   platform project with the new export. Needed for points 1 and 2 above
   (already done, if you've re-exported since); the sensor-side crop
   option in point 3 is a register write, not a Vivado change, so it
   doesn't trigger this — the VHDL-widening option does.
6. See **§4** below on resolution/timing before expecting a full-resolution
   live picture — there's still a real gap there, separate from the three
   above.
7. Double-check your IMX415 module's power-up sequencing (rail order,
   reset/XCLR timing) against its vendor documentation if you have any —
   see §5.

If your goal right now is just to validate **I2C bring-up and the chip-ID
check**, you can do that regardless of which hardware variant is
programmed, without touching Vivado at all — that path doesn't depend on
the D-PHY line rate or the demosaic phase.

## 4. Demosaic and resolution — what's actually still missing

**Correction from earlier in this project:** I previously said the raw-
sensor/no-demosaic problem was unconditional — "regardless of lane count,
data rate, or resolution, you'll get scrambled Bayer noise." That was true
for hardware variant #1 in §3 (no image-processing IP at all), but **your
actual design already has a demosaic block** (`AXI_BayerToRGB`, feeding
`AXI_GammaCorrection`, both already wired between the CSI-2 RX and the
VDMA write side, confirmed from the real VHDL source — see §3). The "no ISP
anywhere in the system" framing doesn't apply to you — the FPGA is already
doing the OV5640-internal-ISP's job, generically, for whatever raw sensor
feeds it. What lands in DDR is demosaiced RGB (packed 10-bit/channel, per
§3 point 3), not raw Bayer data — once the gaps below are closed.

**Status update: all three original gaps are now closed** (D-PHY line
rate, the Bayer-phase mismatch, and the line-buffer width limit that
replaced the original third item — see §3 points 1–3 for exactly what
changed in each). **One genuinely optional item remains for anyone
chasing live HDMI:**

1. **`AXI_BayerToRGB`'s line-buffer width limit (§3 point 3) — the one
   that actually gates a correct capture on every path, DDR-only
   included. ✅ Fixed, via a sensor-side crop.** Not something either
   of the other two fixes could have caught, since it's independent of
   both timing and phase — the block's line buffer is hard-limited to
   2048 pixels wide, and IMX415's native width is 3864. `IMX415.h` now
   crops the sensor to 2040×2192 (`IMX415_cfg::CROP_WIDTH`/
   `PIXEL_ARRAY_HEIGHT`) — see §3 point 3 for the exact register writes,
   and for the VHDL-widening alternative if you'd rather keep the
   sensor at full resolution instead. **If you're on an older build of
   this software without the crop, do this before judging the D-PHY or
   Bayer-phase fixes from a captured frame** — without it, the right
   ~47% of every line is corrupted regardless of whether those two are
   correct, and it's easy to misattribute that corruption to one of
   them instead.
2. **Resolution/pixel-clock mismatch — optional, live-HDMI-only, and now
   implemented. ✅ Done, entirely in software.** The cropped 2040×2192
   frame didn't match any entry in `hdmi/VideoOutput.h`'s timing table,
   so a new one was added: `Resolution::R2040_2192_24_NP`, timed with
   the VESA CVT standard formula (verified with the `cvt` reference
   tool — `cvt 2040 2192 24` — not hand-derived) at **23.96Hz, pixel
   clock 143.75MHz**. This project's own `timing.xdc` states the real
   ceiling directly: *"Maximum targeted pixel clock frequency for
   dynamic video clock generator is 148.5 MHz"* — 143.75MHz clears it
   with ~4.75MHz to spare. (25Hz's CVT timing for this exact resolution
   is already 150MHz — over the ceiling — which is why this lands on
   24Hz rather than a rounder-looking 25 or 30.) `video_dynclk` is
   explicitly built as a **runtime-reconfigurable** clock generator
   (DRP-driven, AXI-Lite controlled), not a fixed one, so none of this
   needed a Vivado change:
   * **AXI_VDMA** — `imx415/AXI_VDMA.h`'s `configureRead(h_res, v_res)`
     now called with `CROP_WIDTH`/`PIXEL_ARRAY_HEIGHT`, the same pair
     already used for `configureWrite()`.
   * **VTC** — new timing (front/back porch, sync widths, polarity) set
     at runtime via `XVtc_SetGeneratorTiming()` inside
     `VideoOutput::configure()` — the exact mechanism this file already
     used for its other three resolutions.
   * **`video_dynclk`** — new MMCM factors (`mul=14.375`, `divclk=2`,
     `clkout_div0=1.0`, landing on a 718.75MHz VCO — 5× the 143.75MHz
     pixel clock, since the MMCM's `CLKOUT0` feeds the DVI serializer
     at 5× before a BUFR divides it back down) written via
     `XClk_Wiz_WriteReg()`, the same dynamic-reconfiguration calls
     `VideoOutput.h` already made for its other three resolutions. The
     100MHz `video_dynclk` reference input and the 5× relationship were
     both derived by back-solving the three existing cases, not
     assumed — see `VideoOutput.h`'s comment on this new case for the
     arithmetic.
   * **AXI4S Video Out** (`v_axi4s_vid_out_0`) — turned out to need
     nothing at all: nothing in this project's original OV5640-era
     HDMI-working code ever configured it independently of VTC, so
     there was nothing to add here either.

   A Vivado/XDC change would only be needed to go *above* that
   148.5MHz ceiling — not the case here. `main()` now brings this up
   once, right after the first `pipeline_mode_change()` call, rather
   than inside that function — resolution doesn't depend on MIPI lane
   rate, so redoing the clock lock on every menu-driven lane-rate
   switch would be wasteful and could visibly glitch the display.

Both points are done in this build.

## 5. Wiring & GPIO notes

### The reset-wiring gap (read this first — likely bring-up blocker)

The Pcam 5C connector drove the OV5640's power-down/reset with a single PS
GPIO line (`PS_GPIO`'s `CAM_GPIO0`, EMIO pin 54 in `imx415/PS_GPIO.h`), and
`IMX415::reset()` still reuses that exact same single-GPIO toggle sequence.
That was fine for the Pcam 5C/OV5640, but the "IMX415 CAM R1" board's own
schematic shows something the Pcam 5C didn't have to deal with:

* Its 22-pin FPC connector carries **two separate** signals in the position
  a standard Raspberry-Pi-style camera connector normally has: `CAM_GPIO`
  (pin 5) and `CAM_RST` (pin 6).
* **`CAM_RST` is the one that matters** — it connects, through a 4.7kΩ
  series resistor and with no pull resistor, straight to the sensor's
  `XCLR`/reset pin.
* **`CAM_GPIO` connects to nothing but a bare test point (`TP3`)** on this
  board — it's unused.
* Everything else the sensor needs (power-supply sequencing in the
  datasheet-required 1.1V→1.8V→2.9V order, and the `INCK` clock) happens
  **automatically** on this board via an on-board RC network and an
  always-on oscillator respectively — neither needs a GPIO from the host.

The Zybo's single existing Pcam GPIO pin conventionally maps to a
Raspberry-Pi-standard connector's `CAM_GPIO` position, not `CAM_RST`. If
that holds true through your Standard-Mini adapter cable — which I can't
confirm without the Zybo's own Pcam-connector schematic or the cable's pin
map, neither of which I have — then **`IMX415::reset()` is currently
toggling a pin that goes nowhere on this board, and the sensor's actual
reset line is left floating** (no pull resistor either way, so its state at
power-up is genuinely undefined). That would explain a chip-ID check that
never passes, with everything else (I2C bus, power, clock) actually fine.

**To check**: probe `TP3` (silkscreened on the camera board) with a
scope/multimeter while `reset()` runs — if it toggles in sync, the gap is
confirmed. **To fix**: wire a spare Zybo GPIO/PMOD pin directly to the
camera board's `CAM_RST` net (there's a resistor `R16` right at the
connector you can tap, or the connector pin 6 itself), then extend
`GPIO_Client::Bits` in `GPIO_Client.h` with a second bit, wire it to that
second EMIO/MIO pin in `PS_GPIO.h`, and drive it (instead of, or alongside,
`CAM_GPIO0`) inside `IMX415::reset()`.

### Other notes

* No liquid-lens/motorized-focus I2C device is assumed (your module has a
  fixed M12 lens per the photos). If a future module of yours does have
  one, you can reintroduce a `writeRegLiquid()`-style method following the
  pattern the original OV5640 driver used, and add a menu option for it.

## 6. Building in Vitis

1. Import (or keep) the `system_wrapper` platform project — either the
   original OV5640-era one (fine for I2C/chip-ID bring-up per §3), or your
   rebuilt IMX415-ready one.
2. In Vitis: **File → Import → Git Repository / Existing Vitis application
   project**, or **File → New → Application Project**, pointing at this
   `Zybo-Z7-20-imx415` folder. `.project`/`.cproject` depend on a platform
   project named exactly `system_wrapper` — either import one with that
   name, or update the references (search for `system_wrapper` in both
   files) to match your platform project's actual name.
3. Build `Debug` or `Release` as normal. Required BSP drivers: `xiicps`,
   `xgpiops`, `xscugic`, `xaxivdma`, `xvtc`, `xclk_wiz`, plus whatever
   `MIPI_D_PHY_RX`/`MIPI_CSI_2_RX` driver your platform's hardware design
   generates — unchanged from what the OV5640 project needed.
4. Program the FPGA with your (existing or rebuilt) bitstream, then run/debug
   the ELF on the Cortex-A9, with a serial terminal (115200 8N1) on the
   board's UART.

## 7. Using it

On boot the app brings up the sensor at **720 Mbps/lane, 2-lane**, captures
to DDR at `MEM_BASE_ADDR`, brings up live HDMI at 2040×2192@24Hz, and
prints:

```
Video init done. Capturing to DDR at 0x0a000000 and live on HDMI at 2040x2192@24Hz.
```

Then a serial menu repeats:

```
IMX415 MAIN OPTIONS

Please press the key corresponding to the desired option:
  a. Change MIPI Lane Rate (sensor always outputs full 3864x2192 RAW10)
  b. Write a Register Inside the Image Sensor
  c. Read a Register Inside the Image Sensor
  d. Change Gamma Correction Factor Value
```

* **a** → `1` for 720 Mbps/lane or `2` for 1440 Mbps/lane (both 2-lane,
  both @ this board's confirmed 24MHz INCK, both full 3864×2192 RAW10 —
  see §2).
* **b** / **c** → poke/peek any IMX415 register directly over I2C. Good for
  confirming bring-up: e.g. read `3F12`/`3F13` and check you get `0x514`
  masked with `0xFFF`, or watch `STANDBY` (`3000`) toggle.
* **d** → cycles the FPGA gamma-correction IP core's factor. Feeds directly
  into whatever `AXI_BayerToRGB` demosaics — see §4 for what's still
  unconfirmed about that path (Bayer phase, resolution/timing) before it
  produces a full live picture.

A `HardwareError` thrown from `init()`/`set_mode()` prints an I2C-NACK or
chip-ID-mismatch message over serial — see §8.

## 8. Known limitations / explicitly out of scope here

* **No FPGA/bitstream changes made by this Vitis project itself** — but
  two have since been made and re-implemented on the Vivado side, outside
  this software: the D-PHY line rate reconstrained/re-timed for
  720Mbps/lane, and `AXI_BayerToRGB`'s Bayer-phase `case` statement fixed
  for IMX415's actual GBRG output. See §3 points 1–2 for exactly what
  changed. **A third, more urgent item — `AXI_BayerToRGB`'s line buffer
  being hard-limited to 2048px wide against IMX415's 3864px native width
  — is fixed too, but on the software side of this project, not
  Vivado:** `IMX415.h` now crops the sensor to 2040px wide before it ever
  reaches that block. See §3 point 3 for the exact registers, and for
  the VHDL-widening alternative if you'd rather resynthesize instead of
  crop.
* **HDMI output is enabled by default now** — see §4. All three
  original hardware-side gaps (D-PHY timing, Bayer phase, line-buffer
  width) plus the resolution/pixel-clock mismatch are closed. `main()`
  brings up `Resolution::R2040_2192_24_NP` (2040×2192 @ 23.96Hz,
  143.75MHz pixel clock) once, right after the sensor/capture side is
  brought up.
* **`IMX415::reset()` doesn't yet drive `CAM_RST` explicitly** — it still
  only toggles the single GPIO inherited from the Pcam 5C/OV5640 driver,
  which may not reach this board's actual reset line at all — see §5. This
  is a real, likely gap, not yet fixed in code (fixing it needs a spare
  Zybo GPIO wired to the camera board, which isn't something I can do from
  software alone).
* **No AWB/AE/color-processing** — the IMX415 has no ISP to configure for
  this, and I don't know whether your `AXI_BayerToRGB`/`AXI_GammaCorrection`
  chain includes any (nothing in the block diagram suggests statistics/
  feedback logic). If you want real auto-exposure/AWB, that's additional
  work — either an FPGA statistics+gain-control addition, or a software
  loop on the PS adjusting sensor gain/exposure registers (`GAIN_PCG_0`,
  `SHR0`) using the existing I2C infrastructure.
* **No test-pattern-generator control** — the OV5640 driver had a
  `set_test()` color-bar helper; the IMX415 does have TPG registers
  (`TPG_EN_DUOUT`/`TPG_PATSEL_DUOUT` at `0x30E4`/`0x30E6` per the datasheet)
  but I didn't wire up a menu option for it here — straightforward to add
  following the same pattern as `writeReg`/`writeConfig`.

## 9. Troubleshooting quick-reference

| Symptom | Likely cause |
|---|---|
| `HardwareError::WRONG_ID` from `init()`, especially if it *never* passes no matter what | **Start with §5's reset-wiring gap** — the sensor's actual reset line may simply never be released. Probe `TP3` on the camera board while `reset()` runs to confirm. |
| `HardwareError::WRONG_ID`, other causes | I2C address now uses the measured 0x37 (see §0) rather than the schematic's nominal 0x1A, so this shouldn't be it anymore — but if you rework/restrap SLAMODE0/1 later, re-measure rather than assuming. Otherwise: INCK not present (check the oscillator, §0), or a genuinely dead sensor. A chip ID read back as `0x000` or `0xFFF` usually means "nothing answered," consistent with the reset-wiring gap above. |
| `HardwareError::IIC_NACK` on register read/write | Bus contention, or sensor asleep/unpowered/held in reset. |
| Chip-ID check passes, but the CSI-2/D-PHY receiver never locks (no image data at all) | If you're on a fresh/unmodified bitstream: this bitstream's only originally-timing-closed rate was 420Mbps/lane against 720/1440Mbps/lane IMX415 modes — see §3 point 1. If you've already reconstrained and re-implemented for 720Mbps/lane (as this project now has) and it still doesn't lock, double-check the `-waveform` argument on `dphy_hs_clock_p` was updated to match the new period, not just the period itself — a stale waveform value doesn't stop the build, but it does make the timing report unreliable. |
| Image data flows and looks mostly right, but the right ~40-50% of every line is corrupted/repeating/garbled | **This is the `AXI_BayerToRGB` line-buffer width limit from §3 point 3, not a D-PHY or Bayer-phase problem.** The block's line buffer is fixed at 2048px; IMX415's native width is 3864px, so the tail of every line overwrites the buffer addresses its own head just wrote. Fix with a sensor-side crop or the VHDL line-buffer widening — don't chase this as a timing or phase issue, it's neither. |
| Chip-ID check passes but streaming/timing seems off | Try `REG_SYS_MODE = 0x3034` instead of `0x3033` — see §2's note on the datasheet's internal inconsistency for that one register. |
| HDMI shows nothing, or a blank/black screen, at 2040×2192@24Hz | Check the monitor actually accepts this exact custom timing — it's not a VESA/CEA standard mode, so some displays/scalers may reject it outright even with a mathematically valid signal. Confirm `video_dynclk` reports lock (`XClk_Wiz_ReadReg(...,0x4) & 0x1`, polled inside `VideoOutput::configure()`) before assuming the timing itself is wrong. |
| HDMI shows a picture but it's torn, rolling, or mis-timed | Double-check the new `Resolution::R2040_2192_24_NP` row in `hdmi/VideoOutput.h` against this README's §4 values (`h_fp=120, h_sync=208, h_bp=328, v_fp=3, v_sync=10, v_bp=20`) — a transcription slip in any one of those fields will misalign sync relative to active video. |
| The D-PHY locks and a picture shows on HDMI, but colors look like fine false-color checkerboarding, not a simple tint | The Bayer-phase fix from §3 point 2 (`xor "01"` in `AssignOutputs`) either hasn't been applied yet, or the sensor's crop window has moved off full-array default (the fix assumes pixel (0,0) delivered over MIPI is the sensor's true native (0,0)). |
| Want to confirm frames are actually landing in DDR | Use a debugger memory view at `MEM_BASE_ADDR` (`DDR_BASE_ADDR + 0x0A000000`) after streaming starts, or add your own readback code — there's no on-screen path yet to eyeball it. Remember the packed 10-bit-per-channel/32-bit-word format from §3 point 3 if you parse it yourself. |

## 10. References

* Sony **IMX415-AAQR-C datasheet** (you provided this) — primary source for
  the register map, the "INCK Setting" clock-configuration tables, the
  Power-on Sequence timing, and the SLAMODE0/1 I2C-address table used to
  cross-check §2 and §0's claims.
* **`SCH_IMX415_MIPI_FFC_CAM_REV1.pdf`** (you provided this) — schematic for
  the "IMX415 CAM R1" carrier board; source for §0/§5's I2C-address
  strapping, MIPI lane wiring, oscillator, and `CAM_RST`/`CAM_GPIO` findings.
* **[`Digilent/Zybo-Z7-20-pcam-5c`](https://github.com/Digilent/Zybo-Z7-20-pcam-5c)**
  on GitHub — the real, complete source for your Vivado hardware design
  (archived/no-longer-maintained per its own README, which points to
  [Digilent's reference page](https://digilent.com/reference/programmable-logic/zybo-z7/demos/pcam-5c)
  — note `digilent.com` itself was unreachable from this environment, the
  GitHub source was not). Primary source for §3/§4: `src/constraints/
  timing.xdc` (the 420Mbps/lane D-PHY constraint), `src/bd/system.tcl`
  (block-design instantiation, confirming `CONFIG.kNoOfDataLanes {2}`),
  and `repo/local/ip/AXI_BayerToRGB/hdl/AXI_BayerToRGB.vhd` (MIT-licensed,
  © 2017 Digilent/Ioan Catuna — the actual demosaic RTL, confirming no
  runtime register and the exact output packing).
* **`Digilent/vivado-library`** (the `repo/vivado-library` submodule of the
  repo above) — specifically `ip/MIPI_D_PHY_RX/docs/mipi_d_phy_rx.pdf`, the
  IP user guide with the "tested... 1344Mbps total data rate" (672Mbps/
  lane) figure cited in §3.
* Sony IMX415 mainline Linux driver (original source of the register data
  in `IMX415.h`, later cross-checked against the datasheet above):
  [`drivers/media/i2c/imx415.c`](https://github.com/torvalds/linux/blob/master/drivers/media/i2c/imx415.c),
  GPL-2.0-only, © 2023 WolfVision GmbH.
