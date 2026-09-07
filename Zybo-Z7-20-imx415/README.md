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
| Live HDMI preview wired up in `pipeline_mode_change()` | **Live HDMI preview wired up in `main()`** instead, brought up once rather than inside `pipeline_mode_change()` | Resolution doesn't depend on MIPI lane rate, so it doesn't need re-locking the video clock on every menu-driven mode change. See §4 for the resolution/timing this needed — currently the pre-existing standard `Resolution::R1920_1080_60_PP`, after an earlier custom-timing attempt was confirmed rejected by a real display. |
| `ov5640/PS_IIC.h`, `PS_GPIO.h`, `I2C_Client.h`, `GPIO_Client.h`, `ScuGicInterruptController.h`, `AXI_VDMA.h` | copied to `imx415/` **unchanged**, only the folder moved | Generic Zynq PS peripheral drivers (I2C, GPIO, interrupt controller, VDMA) — not sensor-specific. |
| `hdmi/VideoOutput.h` | gained one new `Resolution` entry, `R2040_2192_24_NP` (§4) — present but **not currently used** by `main.cc`, which uses the pre-existing `R1920_1080_60_PP` instead | Generic HDMI-timing infrastructure, independent of sensor choice — `VideoOutput.h`'s existing table-driven design needed a new row only for the larger, custom-timing crop option; the current default crop (1920×1080) matches a resolution this file already had. |
| `platform/*`, `lscript.ld`, `Xilinx.spec` | unchanged | Zynq PS bring-up / linker infrastructure, independent of sensor choice. |
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
     mode, was `0x00`/all-pixel) plus `REG_PIX_HST=972`/
     `REG_PIX_HWIDTH=1920` (`IMX415_cfg::CROP_HSTART`/`CROP_WIDTH`) and
     `REG_PIX_VST=1112`/`REG_PIX_VWIDTH=2160`
     (`IMX415_cfg::CROP_VSTART`/`CROP_HEIGHT`, ×2 for the register's own
     Line×2 encoding — see below) — centering a 1920×1080 crop in the
     3864×2192 array. 1920 is a multiple of 24 (`PIX_HWIDTH`'s hardware
     constraint) and comfortably under the 2048px line-buffer ceiling
     (2040 would be the largest multiple of 24 that still clears it, but
     1920 was chosen instead — see §4 for why: it makes the crop exactly
     the standard 1920×1080 HDMI resolution). `main.cc`'s
     `vdma_driver.configureWrite()`/`configureRead()` were updated to
     match — both now pass `CROP_WIDTH`/`CROP_HEIGHT`, not
     `PIXEL_ARRAY_WIDTH`/`PIXEL_ARRAY_HEIGHT`, since that's what the
     sensor actually streams once cropped. `PIXEL_ARRAY_WIDTH`/
     `PIXEL_ARRAY_HEIGHT` themselves are untouched — they still correctly
     describe the sensor's true physical array size, just no longer what
     you tell VDMA.
     `PIX_VST`/`PIX_VWIDTH` (0x3044/0x3046 — confirmed from the
     datasheet's own register table, not assumed to mirror
     `PIX_HST`/`PIX_HWIDTH`'s addressing) encode **Line×2** units, unlike
     the horizontal pair's plain-pixel encoding — confirmed from their
     own reset defaults (`PIX_VWIDTH=0x1120=4384`, `4384/2=2192=
     PIXEL_ARRAY_HEIGHT`). Register value must be a multiple of 4 (i.e.
     actual line count even); `CROP_VSTART=556`/`CROP_HEIGHT=1080` are
     both even, satisfying that. The datasheet's `VMAX ≥
     (PIX_VWIDTH_reg/2)+46 = 1080+46 = 1126` restriction for this crop is
     comfortably satisfied by the existing `VMAX_DEFAULT` (2250) —
     unchanged, since this only changes which window is read out, not
     frame/line timing.
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
     `PIX_HST`/`PIX_HWIDTH`/`PIX_VST`/`PIX_VWIDTH` writes back to
     full-array (`WINMODE=0x00`, drop all four new register writes) and
     change `main.cc` back to `PIXEL_ARRAY_WIDTH`/`PIXEL_ARRAY_HEIGHT`.
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
   crops the sensor to 1920×1080 (`IMX415_cfg::CROP_WIDTH`/
   `CROP_HEIGHT`) — see §3 point 3 for the exact register writes,
   and for the VHDL-widening alternative if you'd rather keep the
   sensor at full resolution instead. **If you're on an older build of
   this software without the crop, do this before judging the D-PHY or
   Bayer-phase fixes from a captured frame** — without it, the right
   ~47% of every line is corrupted regardless of whether those two are
   correct, and it's easy to misattribute that corruption to one of
   them instead.
2. **Resolution/pixel-clock mismatch — optional, live-HDMI-only, and now
   implemented. ✅ Done, entirely in software — and now on a standard
   timing.** This went through two iterations, worth knowing about if
   you're comparing against an earlier build or commit:
   * **First iteration:** the sensor was cropped to 2040×2192 (the
     largest crop the 2048px line-buffer ceiling allows), which didn't
     match any entry in `hdmi/VideoOutput.h`'s timing table, so a
     custom one was added — `Resolution::R2040_2192_24_NP`, timed with
     the VESA CVT standard formula (`cvt 2040 2192 24`) at 23.96Hz,
     143.75MHz pixel clock, clearing `video_dynclk`'s documented
     148.5MHz ceiling with ~4.75MHz to spare.
   * **Confirmed on real hardware: some displays reject a custom,
     non-VESA/CEA-standard timing outright**, even though it's
     mathematically valid and the FPGA-side signal chain works
     correctly. A Dell monitor connected to this exact board/bitstream
     showed *"The current input timing is not supported by the monitor
     display. Please change your input timing to 1920x1080, 60Hz or any
     other monitor listed timing"* — this is a real, observed risk, not
     a hypothetical one. (Notably, the monitor read real H/V timing
     info off the link rather than reporting "no signal" — confirming
     `video_dynclk`/`VTC`/`rgb2dvi_0` were all working correctly; only
     the specific timing choice was rejected.)
   * **Current build: cropped to exactly 1920×1080 instead**, so it
     uses `Resolution::R1920_1080_60_PP` — a **pre-existing, standard
     VESA/CEA 1920×1080@60Hz timing** already in `hdmi/VideoOutput.h`
     (the same one the original OV5640-era pipeline used), not a custom
     one. This sidesteps the compatibility risk entirely — every HDMI
     display accepts 1920×1080@60Hz — at the cost of a smaller capture:
     DDR-buffered frames are also 1920×1080 now (down from 2040×2192),
     since `main.cc` uses the same crop for VDMA's write (DDR) and read
     (HDMI) sides. `Resolution::R2040_2192_24_NP` is still present in
     `VideoOutput.h` if you want to go back to the larger, custom-timing
     crop (e.g. testing on a more permissive display, or need the
     resolution and don't need live HDMI).
   * Either way, `video_dynclk` is explicitly built as a
     **runtime-reconfigurable** clock generator (DRP-driven, AXI-Lite
     controlled), not a fixed one, so no Vivado/XDC change was needed
     for the resolution switch itself:
     * **AXI_VDMA** — `imx415/AXI_VDMA.h`'s `configureRead(h_res, v_res)`
       now called with `CROP_WIDTH`/`CROP_HEIGHT`, the same pair used
       for `configureWrite()`.
     * **VTC** — timing (front/back porch, sync widths, polarity) set at
       runtime via `XVtc_SetGeneratorTiming()` inside
       `VideoOutput::configure()` — for `R1920_1080_60_PP` this is the
       exact, already-validated case this mechanism ran for the
       original OV5640-era 1080p HDMI output.
     * **`video_dynclk`** — for `R1920_1080_60_PP`, this reuses the
       pre-existing `case 148500000: mul=37.125; divclk=5;
       clkout_div0=1.0;` MMCM factors (742.5MHz VCO, 5× the 148.5MHz
       pixel clock) — no new derivation needed, unlike
       `R2040_2192_24_NP`'s custom 718.75MHz-VCO case (still in
       `VideoOutput.h`, documented with its own derivation, if you use
       that resolution instead).
     * **AXI4S Video Out** (`v_axi4s_vid_out_0`) — needs nothing at all
       either way: nothing in this project's original OV5640-era
       HDMI-working code ever configured it independently of VTC.

   `main()` brings this up once, right after the first
   `pipeline_mode_change()` call, rather than inside that function —
   resolution doesn't depend on MIPI lane rate, so redoing the clock
   lock on every menu-driven lane-rate switch would be wasteful and
   could visibly glitch the display.

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

**No scope/multimeter handy?** §9's troubleshooting table has a degraded
diagnostic mode for exactly this case — a failed chip-ID check no longer
aborts the whole program; it drops into the interactive menu with
register read/write (`b`/`c`) still usable, so you can poke other
registers over I2C to narrow things down without any test equipment.

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
to DDR at `MEM_BASE_ADDR`, brings up live HDMI at 1920×1080@60Hz, and
prints:

```
Video init done. Capturing to DDR at 0x0a000000 and live on HDMI at 1920x1080@60Hz.
```

Then a serial menu repeats:

```
IMX415 MAIN OPTIONS

Please press the key corresponding to the desired option:
  a. Change MIPI Lane Rate (sensor outputs cropped 1920x1080 RAW10 - see IMX415.h CROP_WIDTH/CROP_HEIGHT)
  b. Write a Register Inside the Image Sensor
  c. Read a Register Inside the Image Sensor
  d. Change Gamma Correction Factor Value
  e. Pan Capture Window (moves field of view - not zoom, see below)
```

* **a** → `1` for 720 Mbps/lane or `2` for 1440 Mbps/lane (both 2-lane,
  both @ this board's confirmed 24MHz INCK, both cropped 1920×1080 RAW10 —
  see §2 for lane-rate details, §3 point 3/§4 for why 1920×1080).
* **b** / **c** → poke/peek any IMX415 register directly over I2C. Good for
  confirming bring-up: e.g. read `3F12`/`3F13` and check you get `0x514`
  masked with `0xFFF`, or watch `STANDBY` (`3000`) toggle.
* **d** → cycles the FPGA gamma-correction IP core's factor. Feeds directly
  into whatever `AXI_BayerToRGB` demosaics — see §4 for what's still
  unconfirmed about that path (Bayer phase, resolution/timing) before it
  produces a full live picture.
* **e** → interactive `w`/`a`/`s`/`d` pan control (`r` recenters, `x`
  exits) that moves the fixed 1920×1080 capture window around within
  the sensor's full 3864×2192 array, live, no restart needed
  (`IMX415::setCropOrigin()` — the datasheet lists `PIX_HST`/`PIX_VST`'s
  reflection timing as "V", meaning the sensor itself latches a new
  value at the next frame). **This is panning, not zooming** — the
  window *size* is fixed by `AXI_BayerToRGB`'s 2048px line-buffer limit
  (§3 point 3), so this only changes *which* ~50%-width/~49%-height
  slice of the sensor's field of view you're looking at, not how
  magnified it looks. See "Why the image looks zoomed in" below for
  the real fix if you want a wider field of view instead.

A `HardwareError` thrown from `init()`/`set_mode()` prints an I2C-NACK or
chip-ID-mismatch message over serial — see §8.

### Why the image looks zoomed in

**Expected, not a bug.** The sensor's native array is 3864×2192; this
project crops it to 1920×1080 (§3 point 3, §4) — roughly half the width
and half the height. Whatever the lens projects onto the full sensor,
you're only reading out the center ~50%×~49% of it, so the displayed
image is magnified by roughly **2.0× horizontally, 2.0× vertically**
relative to the sensor's true field of view. This isn't optical or
electronic zoom (no lens movement, no scaling) — it's simply a smaller
window read out at native resolution.

* **Pan around within that ~2× window**: menu option `e` above — free,
  live, no hardware change.
* **A real wider field of view (less magnification)**: needs the crop
  window itself to be wider than `AXI_BayerToRGB`'s 2048px line-buffer
  ceiling can accept — the only way there is the VHDL line-buffer
  widening documented as the alternative fix in §3 point 3
  (`LineBuffer.vhd`/`AXI_BayerToRGB.vhd`, widening `kLineBufferWidth`
  from 2048 to e.g. 4096 and the addressing signals from 11 to 12
  bits). **This is a real hardware change** — real RTL edits,
  resynthesis, a new bitstream, and a new `.xsa` re-exported and
  re-associated with the Vitis platform project. Software alone cannot
  get you a wider field of view than this project's current 1920px (or
  even the line-buffer-maximizing 2040px) crop allows.
* Physically moving the camera back, or a wider-FOV lens if your module
  supports swapping it, are the non-electronic alternatives — this
  project has no optical zoom/focus control (§5, fixed M12 lens).

### Why the image has a violet/magenta color cast

**Also expected, not a new bug** — and actually a good sign about
everything else: a real, spatially coherent image (not fine
checkerboard noise) confirms the D-PHY line-rate fix and the
Bayer-phase fix (§3 points 1–2) are both working correctly. A phase
error looks like per-pixel false-color noise, not a smooth, uniform
tint across a recognizable picture.

The tint itself is because **this pipeline has no white balance
anywhere** — not on the sensor (`IMX415.h` says so explicitly: *"the
IMX415 has no internal ISP - it always outputs raw Bayer data. AWB/
gain/format conversion must happen downstream... not on the sensor
itself"*), and not in the FPGA fabric either: `AXI_BayerToRGB` only
demosaics (converts the Bayer pattern to RGB, one color sample per
pixel instead of one color per pixel-position), and `AXI_GammaCorrection`
only applies a single shared nonlinear gamma curve (menu option `d`) —
neither does the simple per-channel (R/G/B) linear gain multiplication
that real white balance needs. Raw demosaiced Bayer data with unequal
R/G/B channel sensitivity (from the sensor's color filter array
response, scene lighting, or lack of an IR-cut filter) reads out looking
tinted exactly like this — it's the expected appearance of "no ISP
anywhere," which was true of this design from the start.

**To actually correct it, in the live HDMI path, needs new FPGA hardware
— there's no software/register lever for it in the current pipeline**:
neither the sensor nor any existing block exposes independent R/G/B
gain. A real fix means adding a new AWB/color-correction IP block (3
multiplier coefficients, one per channel, applied after
`AXI_BayerToRGB` and before `AXI_GammaCorrection` or `AXI_VDMA`) —
**a real hardware change**: new IP, new connections in the block
design, resynthesis, new bitstream, new `.xsa`. That's a genuine
departure from this project's "no new block needed" finding so far
(§"Why no new block" in the signal-path review) — worth deciding
deliberately rather than adding speculatively. If you only need this
for DDR-captured frames (not live HDMI), the same gray-world/white-patch
correction math can instead be applied in software after reading the
raw buffer out of DDR — no hardware change needed for that path.

### Concrete hardware pipeline changes, if you want either

Neither of these is done — both are real Vivado work, specified here to
the same level of detail as the fixes already implemented this project,
so there's a concrete starting point if you decide to pursue one.

**1. Wider field of view — widen `AXI_BayerToRGB`'s line buffer**

Already the documented alternative to this project's sensor-side crop
(§3 point 3) — restated here as the "hardware pipeline change" this
specific question is about:

* `LineBuffer.vhd`: widen `pWriteAddr`/`pReadAddr` from
  `STD_LOGIC_VECTOR(10 downto 0)` to `(11 downto 0)`; bump the generic
  default from `2048` to `4096`.
* `AXI_BayerToRGB.vhd`: widen `sCntColumns`, `sLineBufferWriteAddr`,
  `sLineBufferReadAddr`, `sLineBufferCrntAddr` from 11 to 12 bits;
  change `LineBufferInst`'s `generic map(kLineBufferWidth => 2048)` to
  `4096`. Leave `sCntLines` alone (only its bit-0 parity is read,
  correct through overflow at any width).
* Nothing else in either file changes, and neither file's AXI4-Stream
  port list changes — **no rewiring in the block diagram, no new
  block, no new AXI-Lite address** — just this one IP's internals.
  4096 is comfortably past IMX415's full 3864px native width, so
  `IMX415.h`'s crop could then go all the way back to `WINMODE=0x00`
  (all-pixel readout) — full native field of view, no crop needed at
  all (though you'd then need a *different* HDMI timing again, since
  3864×2192 isn't standard either — see the `R2040_2192_24_NP`
  precedent in §4 for how to derive one, or keep a smaller crop for a
  standard timing while still gaining real margin over today's 1920px).
* Resource cost is trivial on this device (a few KB more BRAM for the
  doubled buffer). Resynthesize `AXI_BayerToRGB_1`, re-implement,
  **export a new `.xsa`**, re-associate the Vitis platform project
  with it (§6).

**2. Real color correction — new AXI-Stream gain block**

Not started; this is a genuinely new IP, not a modification of an
existing one, so it needs both new RTL and new block-diagram wiring:

* **Position**: insert between `AXI_BayerToRGB_1` and
  `AXI_GammaCorrection_0` — white balance belongs in the *linear*
  domain, before gamma's nonlinear tone curve, matching standard ISP
  ordering.
* **Data interface**: identical to what already connects those two
  blocks — an AXI4-Stream, 32-bit `TDATA` packed exactly as
  `AXI_BayerToRGB` already outputs it (`[29:20]`=Red 10-bit,
  `[19:10]`=Blue 10-bit, `[9:0]`=Green 10-bit, per §3 point 4) — so
  this new block is a pure passthrough format-wise: unpack, multiply,
  saturate, repack, forward `TVALID`/`TREADY`/`TLAST` unchanged. No
  line buffer needed here at all (unlike `AXI_BayerToRGB`) — it's a
  per-pixel, single-cycle operation, so it doesn't reintroduce any
  line-width ceiling.
* **Control interface**: AXI4-Lite slave, same pattern as
  `AXI_GammaCorrection_0`'s. Minimum viable register map — 3 writable
  32-bit registers, one per channel gain, fixed-point (e.g. Q2.8:
  8 fractional bits, default `0x100` = 1.0×, giving a 0–4× range):
  `0x00` = Red gain, `0x04` = Green gain (usually left at 1.0× as the
  reference channel), `0x08` = Blue gain. Correct gain values can't be
  guessed blind — they need at least one captured frame's actual
  average R/G/B levels (classic gray-world: `gain_R = avg_G/avg_R`,
  `gain_B = avg_G/avg_B`) or a white/gray reference object in frame.
* **VHDL skeleton** (structurally complete; adapt signal names to
  match this project's actual `AXI_BayerToRGB_1`/`AXI_GammaCorrection_0`
  port names, which aren't in front of me to copy verbatim):

  ```vhdl
  entity AXI_WhiteBalance is
    generic (
      C_S_AXIS_TDATA_WIDTH : integer := 32;
      C_M_AXIS_TDATA_WIDTH : integer := 32;
      C_S_AXI_DATA_WIDTH   : integer := 32;
      C_S_AXI_ADDR_WIDTH   : integer := 4  -- 3 registers -> 2 bits needed, 4 is a safe/common default
    );
    port (
      -- AXI4-Stream slave (from AXI_BayerToRGB)
      s_axis_aclk    : in  std_logic;
      s_axis_aresetn : in  std_logic;
      s_axis_tvalid  : in  std_logic;
      s_axis_tready  : out std_logic;
      s_axis_tdata   : in  std_logic_vector(C_S_AXIS_TDATA_WIDTH-1 downto 0);
      s_axis_tlast   : in  std_logic;
      -- AXI4-Stream master (to AXI_GammaCorrection)
      m_axis_tvalid  : out std_logic;
      m_axis_tready  : in  std_logic;
      m_axis_tdata   : out std_logic_vector(C_M_AXIS_TDATA_WIDTH-1 downto 0);
      m_axis_tlast   : out std_logic;
      -- AXI4-Lite slave (gain registers)
      s_axi_aclk     : in  std_logic;
      s_axi_aresetn  : in  std_logic;
      s_axi_awaddr   : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
      s_axi_awvalid  : in  std_logic;
      s_axi_awready  : out std_logic;
      s_axi_wdata    : in  std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
      s_axi_wvalid   : in  std_logic;
      s_axi_wready   : out std_logic;
      s_axi_bresp    : out std_logic_vector(1 downto 0);
      s_axi_bvalid   : out std_logic;
      s_axi_bready   : in  std_logic;
      s_axi_araddr   : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
      s_axi_arvalid  : in  std_logic;
      s_axi_arready  : out std_logic;
      s_axi_rdata    : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
      s_axi_rresp    : out std_logic_vector(1 downto 0);
      s_axi_rvalid   : out std_logic;
      s_axi_rready   : in  std_logic
    );
  end entity;
  -- Architecture: three Q2.8 unsigned multiplies (gain_reg * channel,
  -- >>8 to rescale, clamp to 10 bits/0x3FF), one register stage so
  -- TVALID/TREADY handshake stays combinational-safe. Trivial LUT/DSP
  -- cost - three 18x10 multiplies at most.
  ```

  Vivado's **Tools → Create and Package New IP** wizard generates the
  AXI4-Lite slave boilerplate (address decode, `awready`/`wready`/
  `bvalid`/etc. state machine) automatically if you start from its
  "AXI4 peripheral" template — you'd only need to write the 3-register
  read/write logic and the actual multiply/clamp/repack datapath by
  hand, not the whole AXI4-Lite protocol machinery.
* **Block-diagram wiring** (beyond just adding the IP): its AXI4-Lite
  slave port needs a master port on `ps7_0_axi_periph` to connect
  to — check that interconnect's current master-port count first;
  adding a 7th peripheral where it was generated for 6 means
  regenerating `ps7_0_axi_periph` itself with one more master port,
  not just dropping the new IP in. After wiring, run Vivado's
  **Address Editor** to assign the new AXI-Lite slave a base address
  (becomes a new `XPAR_..._BASEADDR` in `xparameters.h` on the next
  `.xsa` export) — same mechanism `GAMMA_BASE_ADDR` already uses in
  `main.cc`.
* **Software side**, once that's done: a 3-line `Xil_Out32()` driver
  (identical pattern to how `main.cc` already drives
  `GAMMA_BASE_ADDR`) plus a new menu option to adjust gains live,
  mirroring option `d`'s structure.
* Resynthesize the whole design (new block + widened interconnect),
  implement, **export a new `.xsa`**, re-associate the Vitis platform
  project with it (§6).

## 8. Known limitations / explicitly out of scope here

* **No FPGA/bitstream changes made by this Vitis project itself** — but
  two have since been made and re-implemented on the Vivado side, outside
  this software: the D-PHY line rate reconstrained/re-timed for
  720Mbps/lane, and `AXI_BayerToRGB`'s Bayer-phase `case` statement fixed
  for IMX415's actual GBRG output. See §3 points 1–2 for exactly what
  changed. **A third, more urgent item — `AXI_BayerToRGB`'s line buffer
  being hard-limited to 2048px wide against IMX415's 3864px native width
  — is fixed too, but on the software side of this project, not
  Vivado:** `IMX415.h` now crops the sensor to 1920px wide (well under
  the 2048px limit) before it ever reaches that block. See §3 point 3 for
  the exact registers, and for the VHDL-widening alternative if you'd
  rather resynthesize instead of crop.
* **HDMI output is enabled by default now** — see §4. All three
  original hardware-side gaps (D-PHY timing, Bayer phase, line-buffer
  width) plus the resolution/pixel-clock mismatch are closed. `main()`
  brings up `Resolution::R1920_1080_60_PP` (standard 1920×1080@60Hz,
  148.5MHz pixel clock — the same case the original OV5640-era pipeline
  used) once, right after the sensor/capture side is brought up. An
  earlier build used a custom 2040×2192@23.96Hz timing
  (`Resolution::R2040_2192_24_NP`, still present in `VideoOutput.h` but
  unused by default) — switched away from after a real display rejected
  it as an unsupported input timing; see §4.
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
| `HardwareError::WRONG_ID`, other causes | I2C address now uses the measured 0x37 (see §0) rather than the schematic's nominal 0x1A, so this shouldn't be it anymore — but if you rework/restrap SLAMODE0/1 later, re-measure rather than assuming. Otherwise: INCK not present (check the oscillator, §0), or a genuinely dead sensor. A chip ID read back as `0x000` or `0xFFF` usually means "nothing answered," consistent with the reset-wiring gap above. **Caveat, now fixed:** `readReg()`/`writeReg()` had a retry-loop bug (their final-attempt-failure `throw HardwareError(IIC_NACK, ...)` was unreachable dead code) that made a *totally unresponsive* I2C bus silently look identical to a *successful* read of a genuinely-zero register — both surfaced as `WRONG_ID: got 0x000` with no `IIC_NACK` ever raised. Fixed now, so on a rebuilt image `WRONG_ID: got 0x000` and `IIC_NACK` are distinguishable again — see the next row. |
| `HardwareError::IIC_NACK` on register read/write | Bus contention, or sensor asleep/unpowered/held in reset — i.e. genuinely zero ACKs across all `retry_count_` (10) attempts. If you now see this instead of `WRONG_ID: got 0x000` after rebuilding with the retry-loop fix above, that's a real, previously-hidden signal that the I2C bus itself is dead, not that a live sensor answered with zeroed registers — it points more strongly at §5's reset-wiring gap (or a genuinely wrong/unpowered sensor) than a `WRONG_ID` result would have. |
| Program used to abort entirely (`terminate called after throwing...`) on a failed chip-ID check, with no way to poke registers afterward to dig further — **no scope/multimeter handy for the `TP3` probe above** | **Fixed — there's now a degraded diagnostic mode for exactly this.** `IMX415`'s constructor no longer calls `init()` itself (moved to `pipeline_mode_change()`, which already called it), so a failed chip-ID check no longer prevents `cam` from existing. `main()` now catches `HardwareError` around both call sites (initial boot, and the menu's lane-rate option `a`), prints which kind it was (`WRONG_ID` vs `IIC_NACK`) plus the message, skips capture/HDMI bring-up, and **still reaches the interactive menu** — so options `b`/`c` (write/read any sensor register directly) work even on a failed boot. Useful next step if you're stuck at `WRONG_ID: got 0x000` with no way to probe hardware: try reading a few *other* registers (e.g. `3000` = `REG_MODE`, which `init()` itself last wrote to `01`/standby before the exception) via option `c`. If every register you try reads back `0x000`/`0xFFF` too, that's consistent with a sensor that's genuinely not there or fully unresponsive; if some read back plausible values and only `3F12`/`3F13` (`SENSOR_INFO`) look wrong, that points at an addressing quirk specific to that register instead (similar in spirit to the already-documented `SYS_MODE` 0x3033/0x3034 datasheet inconsistency — worth trying alternate documented addresses for `SENSOR_INFO` too, if this comes up). |
| Chip-ID check passes, but the CSI-2/D-PHY receiver never locks (no image data at all) | If you're on a fresh/unmodified bitstream: this bitstream's only originally-timing-closed rate was 420Mbps/lane against 720/1440Mbps/lane IMX415 modes — see §3 point 1. If you've already reconstrained and re-implemented for 720Mbps/lane (as this project now has) and it still doesn't lock, double-check the `-waveform` argument on `dphy_hs_clock_p` was updated to match the new period, not just the period itself — a stale waveform value doesn't stop the build, but it does make the timing report unreliable. |
| Image data flows and looks mostly right, but the right ~40-50% of every line is corrupted/repeating/garbled | **This is the `AXI_BayerToRGB` line-buffer width limit from §3 point 3, not a D-PHY or Bayer-phase problem.** The block's line buffer is fixed at 2048px; IMX415's native width is 3864px, so the tail of every line overwrites the buffer addresses its own head just wrote. Fix with a sensor-side crop or the VHDL line-buffer widening — don't chase this as a timing or phase issue, it's neither. |
| A real, coherent (not checkerboard-noisy) picture shows on HDMI, but it looks zoomed in / magnified vs. what the lens actually sees | **Expected** — see §7 "Why the image looks zoomed in". Use menu option `e` to pan within the current field of view; a genuinely wider field of view needs the VHDL line-buffer widening (real hardware change, new `.xsa`), not a software fix. |
| The picture is spatially correct (real shapes/edges/detail, not checkerboarding) but has a strong violet/magenta/other color cast | **Expected — actually good news about the Bayer-phase fix (§3 point 2), which is what a checkerboard artifact would look like instead.** See §7 "Why the image has a violet/magenta color cast". This pipeline has no white balance anywhere (sensor or fabric) — fixing it live needs a new FPGA AWB block (real hardware change, new `.xsa`); DDR-captured frames can be corrected in software post-processing instead, no hardware change needed. |
| Chip-ID check passes but streaming/timing seems off | Try `REG_SYS_MODE = 0x3034` instead of `0x3033` — see §2's note on the datasheet's internal inconsistency for that one register. |
| Monitor shows "input timing not supported" / "change to 1920x1080, 60Hz or any other monitor listed timing" | **Confirmed on real hardware** — this is exactly why the default build now uses `Resolution::R1920_1080_60_PP` instead of the custom `R2040_2192_24_NP` timing (see §4). If you're still seeing this on a current build, you're likely still using the custom-timing build/branch — switch `main()`'s `vid.configure(...)` call (and `IMX415.h`'s `CROP_WIDTH`/`CROP_HEIGHT`/`CROP_HSTART`/`CROP_VSTART`) back to the 1920×1080 values. If you *want* the larger custom-timing crop and are seeing this, your specific display just doesn't accept non-standard timings — try a different one, or fall back to DDR-only capture (see the "confirm frames are landing in DDR" row below), which doesn't depend on the monitor at all. |
| HDMI shows nothing, or a blank/black screen | With the current 1920×1080@60Hz standard timing this would be unusual — check `video_dynclk` reports lock (`XClk_Wiz_ReadReg(...,0x4) & 0x1`, polled inside `VideoOutput::configure()`) and that the HDMI cable/monitor input is actually selected, before suspecting the timing itself. If you've switched to the custom `R2040_2192_24_NP` resolution instead, see the row above first. |
| HDMI shows a picture but it's torn, rolling, or mis-timed | If you're on the default `Resolution::R1920_1080_60_PP`, this is a pre-existing, previously-validated timing (from the OV5640-era pipeline) — a mis-timing here more likely points at `video_dynclk` not actually locking, or a VDMA read/write frame-buffer race, than a transcription error in the timing table. If you've switched to `Resolution::R2040_2192_24_NP` instead, double-check that row in `hdmi/VideoOutput.h` against this README's §4 values (`h_fp=120, h_sync=208, h_bp=328, v_fp=3, v_sync=10, v_bp=20`) — a transcription slip in any one of those fields will misalign sync relative to active video. |
| The D-PHY locks and a picture shows on HDMI, but colors look like fine false-color checkerboarding, not a simple tint | The Bayer-phase fix from §3 point 2 (`xor "01"` in `AssignOutputs`) either hasn't been applied yet, or the sensor's crop window has moved off full-array default (the fix assumes pixel (0,0) delivered over MIPI is the sensor's true native (0,0)). |
| Want to confirm frames are actually landing in DDR | Use a debugger memory view at `MEM_BASE_ADDR` (`DDR_BASE_ADDR + 0x0A000000`) after streaming starts, or add your own readback code — there's no on-screen path yet to eyeball it. Remember the packed 10-bit-per-channel/32-bit-word format from §3 point 3 if you parse it yourself. |
| Build fails with `'cout' is not a member of 'std'` / `'endl' is not a member of 'std'` in `AXI_VDMA.h`, plus a cascading `make: *** [all] Error 2` | **Pre-existing Digilent bug, already fixed in this tree.** `AXI_VDMA.h`'s four IRQ handlers (`readHandler`/`writeHandler`/`readErrorHandler`/`writeErrorHandler`) use `std::cout`/`std::endl` but the header never included `<iostream>` itself — it only worked before if some other header happened to pull `<iostream>` in first. This project's `main.cc` include chain never does, so it fails outright. Fixed by adding `#include <iostream>` to `AXI_VDMA.h`. If you're seeing this on an older copy of the file (e.g. the original OV5640/pcam-5c project, which has the same bug), add that one include line and it goes away — it's not related to D-PHY/Bayer/crop/HDMI at all. |
| Vitis Problems view also shows "Invalid project path: Include path not found" warnings (`pathentry`) | Unrelated to the above — a stale include-path entry in the project's stored C/C++ build settings, typically left over from copying/importing the project without the platform/BSP being freshly generated at this exact location. Fix: project → **C/C++ Build Settings → Paths and Symbols → Includes**, remove or re-point the missing entries, then **Project → C/C++ Index → Rebuild**. Building the platform project first (so the BSP include folders actually exist) often clears these on its own. |

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
