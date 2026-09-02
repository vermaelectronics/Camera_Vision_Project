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
specific board), **§3 "Hardware side"**, and **§4 "The raw-sensor
problem"** below before you power anything up.

## 0. Your hardware — now confirmed from the datasheet + schematic

You provided the Sony **IMX415-AAQR-C datasheet** and the **schematic for
the "IMX415 CAM R1" board** (`SCH_IMX415_MIPI_FFC_CAM_REV1.pdf`), on top of
the earlier photos of it plugged into a Zybo Z7-20 via a Raspberry-Pi-style
"Standard-Mini" adapter cable into the board's Pcam MIPI connector. That
resolved almost everything §0 used to flag as an assumption:

* **I2C address 0x1A** — confirmed three independent ways: an explicit note
  on the schematic itself, cross-checked against the datasheet's
  SLAMODE0/SLAMODE1 slave-address truth table, and cross-checked against the
  board's own resistor strapping (both address-select pins pulled low via
  10kΩ, no pull-up populated). This is not a guess.
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
  at all) feeds the sensor's INCK pin directly. I could not confirm this
  exact part's frequency from Kyocera's public data in this environment;
  `INCK_HZ` still defaults to the best-effort 24MHz (one of only 5
  frequencies the datasheet says the sensor supports at all, and the most
  common choice for this class of board). **Check the frequency printed on
  the oscillator package itself** (it's the small 4-pin component labeled
  Y1 near the connector) to close this out for certain.
* **A real, concrete, likely bring-up blocker was found**: this board's
  connector separates `CAM_RST` (which actually drives the sensor's reset
  pin) from `CAM_GPIO` (which, on this board, connects to nothing but a bare
  test point). The Zybo's single existing Pcam GPIO pin conventionally
  reaches the connector's `CAM_GPIO` position, not `CAM_RST` — meaning the
  driver's existing `reset()` may well be toggling a pin that goes nowhere
  on this board, leaving the sensor's actual reset line floating. See §5 —
  this is now the single most likely reason bring-up would fail, more
  likely than any lane/timing issue.

All of `IMX415.h`'s register *values* were also independently cross-checked
byte-for-byte against the datasheet in §2 below — see there for the one
inconsistency found (in Sony's own document, not in this port).

## 1. What actually changed

| Original (OV5640 / Pcam 5C)                          | This project (IMX415)                                        | Why |
|--------------------------------------------------------|----------------------------------------------------------------|-----|
| `src/ov5640/OV5640.h`, `OV5640.cpp`                     | `src/imx415/IMX415.h`, `IMX415.cpp`                             | New driver class, register map, and register *values* — see §2 |
| `src/main.cc` — includes `ov5640/OV5640.h`, instantiates `OV5640 cam(...)` | `src/main.cc` — includes `imx415/IMX415.h`, instantiates `IMX415 cam(...)` | Swap the driver actually used by the app |
| Menu: **a. Change Resolution** (720p/1080p15/1080p30) | Menu: **a. Change MIPI Lane Rate** (720/891 Mbps per lane) | The IMX415 has one native readout size (full sensor array) — see §2. There's no sensor-side resolution to pick, only the lane rate the same fixed-size frame is clocked out at. |
| Menu: **b. Change Liquid Lens Focus** | **removed** | That's the Pcam 5C's variable-focus liquid-lens IC, a separate chip on *that* board. Your "IMX415 CAM R1" module has a fixed M12 lens, not a liquid lens. |
| Menu: **d. Change Image Format (Raw or RGB)**, **h. Change AWB Settings** | **removed** | Both are OV5640-internal-ISP features (Bayer→RGB conversion, auto white balance). The IMX415 has no on-sensor ISP at all — see §4, this is a bigger deal than just "fewer menu options." |
| Menu: **e/f** (write/read sensor register) | kept, renumbered **b/c** | Still very useful for IMX415 bring-up/debug. |
| Menu: **g** (gamma factor) | kept, renumbered **d** | Drives the FPGA's `AXI_GammaCorrection` core, not the sensor — unrelated to which camera is attached (though currently not wired to anything meaningful — see §4). |
| Live HDMI preview wired up in `pipeline_mode_change()` | **HDMI output intentionally disabled** in `pipeline_mode_change()`; only the sensor→DDR capture path is brought up | See §4 — this isn't a config bug to fix, it's a real architectural gap (missing Bayer demosaic) that needs new FPGA IP. |
| `ov5640/PS_IIC.h`, `PS_GPIO.h`, `I2C_Client.h`, `GPIO_Client.h`, `ScuGicInterruptController.h`, `AXI_VDMA.h` | copied to `imx415/` **unchanged**, only the folder moved | Generic Zynq PS peripheral drivers (I2C, GPIO, interrupt controller, VDMA) — not sensor-specific. |
| `hdmi/VideoOutput.h`, `platform/*`, `lscript.ld`, `Xilinx.spec` | unchanged | Generic HDMI-timing / Zynq PS bring-up / linker infrastructure, independent of sensor choice. |
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
hand, I cross-checked every register/value pair used here (the 720Mbps and
891Mbps clock-configuration and D-PHY-timing tables) against the datasheet's
own "INCK Setting" section, byte-for-byte. Every value matches exactly**,
confirming the Linux-driver port was accurate — this is no longer just "a
shipping driver's values," it's independently verified against Sony's
primary source.

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
  is the **MIPI lane rate** (720 or 891 Mbps/lane are both fully specified
  and now datasheet-verified; 1440 Mbps/lane also exists but needs a faster
  D-PHY, so I didn't wire it up here) and the **lane count** (2 or 4 — see
  §0 for why 2 is correct for this board).
* **There's a real, documented chip-ID register**: `SENSOR_INFO` at
  `0x3F12` (16-bit), masked `0xFFF`, expected `0x514`. `IMX415::init()`
  checks this (matching the OV5640 driver's own ID-check pattern) instead
  of the earlier version's indirect standby-readback heuristic.

### What's still genuinely unresolved

| Item | Where | Status |
|---|---|---|
| **INCK (XVCLK) frequency** | `IMX415_cfg::INCK_HZ` in `IMX415.h` | The board generates this itself from an always-on oscillator (see §0) — defaults to **24 MHz** as a best-effort default (unconfirmed; check the part's markings). If it's actually one of the other 4 datasheet-supported frequencies, `set_mode()` needs the matching `cfg_clk_*` table from `IMX415_cfg` swapped in (several are already included: 720Mbps@24/72MHz, 891Mbps@27/37.125/74.25MHz). |
| **Sensor reset wiring (`CAM_RST` vs `CAM_GPIO`)** | `IMX415::reset()` in `IMX415.h` | Likely gap, not yet fixed in code — see §5, this is the top bring-up risk. |
| **Vivado D-PHY/CSI-2 RX IP configuration** | Hardware design (not in this software export) | Still unconfirmed whether it's built for 720 or 891 Mbps/lane, or something else entirely — see §3. |

None of these being wrong will damage the sensor — worst case is no image,
garbled data, or an I2C NACK.

## 3. Hardware side — what you still need to do

This project's `system_wrapper.bit` bitstream was originally built around
the **OV5640's** MIPI characteristics (2-lane, RAW10, ≤336MHz-class MIPI
clock) and 1920×1080-and-below HDMI output timing. For the IMX415:

1. **Confirm/reconfigure the `MIPI_D_PHY_RX`/`MIPI_CSI_2_RX` IP** in Vivado
   for the lane rate you select in `IMX415.h` (720 or 891 Mbps/lane, 2-lane
   by default). If it doesn't match, the CSI-2 receiver simply won't lock —
   the sensor will still respond fine over I2C, but no valid pixel data will
   arrive.
2. **Confirm the AXI4-Stream width** feeding the `AXI_VDMA` write channel
   matches RAW10 (this driver keeps `ADBIT`/`MDBIT` at RAW10 to match what
   the OV5640-era IP was generated for).
3. Re-export the hardware platform (**File → Export → Export Hardware**,
   include bitstream) and re-associate this application's `system_wrapper`
   platform project with the new export.
4. See **§4** below before spending time on HDMI output timing — there's a
   more fundamental gap to close first.
5. Double-check your IMX415 module's power-up sequencing (rail order,
   reset/XCLR timing) against its vendor documentation if you have any —
   see §5.

If your goal right now is just to validate **I2C bring-up and the chip-ID
check**, you can do that on the existing OV5640-era hardware platform
without touching Vivado at all.

## 4. The raw-sensor problem (read this before chasing HDMI output)

This is the most important correction from the first version of this port.

The original OV5640/Pcam-5C design pipes the sensor's MIPI output straight
through `MIPI_D_PHY_RX → MIPI_CSI_2_RX → AXI_VDMA → VTC → HDMI TX`, with
**no image-processing IP in the FPGA at all**. That only produces a real
picture because the **OV5640 has an internal ISP** that demosaics its Bayer
sensor data into ready-to-display RGB565 *before it ever leaves the chip*
(see the OV5640 driver's `{0x4300, 0x6f}` RGB565 / `ISP_FORMAT_MUX_CONTROL`
registers — this project's original menu even had a "Raw or RGB" option
controlling exactly that on-sensor conversion).

**The IMX415 has no internal ISP.** It only ever outputs raw, undemosaiced
Bayer data (RAW10). Feeding that directly into the same passthrough
pipeline — regardless of whether you get the lane count, data rate, and
resolution all correct — will not produce a viewable color picture. You'll
get a scrambled, grainy, roughly-monochrome-looking Bayer-pattern texture on
screen, not a photo.

Because of this (and because 3864×2192's pixel rate is well beyond what this
hardware platform's clocking wizard/HDMI TX were sized for — see §3),
`main.cc`'s `pipeline_mode_change()` **deliberately does not bring up the
VTC/HDMI output stage** in this version. It brings up the sensor and the
VDMA **write** (capture-to-DDR) side only. Live HDMI preview needs one of:

* **Add a Bayer-demosaic/ISP IP core in the FPGA fabric** between the CSI-2
  RX output and the VDMA write side (Xilinx's Video/Imaging IP library has
  a demosaic core; you'd typically also want gamma/CSC after it — the
  existing `AXI_GammaCorrection` core could potentially be reused
  downstream of a demosaic block), **or**
* Capture raw frames to DDR (which this version does) and **demosaic in
  software** (on the Cortex-A9, or by pulling the buffer off over
  JTAG/debugger and demosaicing on a host PC) rather than expecting a live
  preview at all.

Either path is a real hardware/software project in its own right, well
beyond a register-table port — I'm flagging it clearly rather than leaving
you to discover it after chasing phantom lane-count bugs.

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
to DDR at `MEM_BASE_ADDR`, and prints:

```
Video init done. Capturing to DDR at 0x0a000000 (see README.md - no HDMI preview yet).
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

* **a** → `1` for 720 Mbps/lane or `2` for 891 Mbps/lane (both 2-lane; both
  full 3864×2192 RAW10 — see §2).
* **b** / **c** → poke/peek any IMX415 register directly over I2C. Good for
  confirming bring-up: e.g. read `3F12`/`3F13` and check you get `0x514`
  masked with `0xFFF`, or watch `STANDBY` (`3000`) toggle.
* **d** → cycles the FPGA gamma-correction IP core's factor. Currently not
  wired to anything downstream of a working image path — see §4.

A `HardwareError` thrown from `init()`/`set_mode()` prints an I2C-NACK or
chip-ID-mismatch message over serial — see §8.

## 8. Known limitations / explicitly out of scope here

* **No FPGA/bitstream changes** — see §3.
* **No live HDMI preview** — see §4; this needs new FPGA demosaic IP or a
  software debayer step, not a config fix.
* **`IMX415::reset()` doesn't yet drive `CAM_RST` explicitly** — it still
  only toggles the single GPIO inherited from the Pcam 5C/OV5640 driver,
  which may not reach this board's actual reset line at all — see §5. This
  is a real, likely gap, not yet fixed in code (fixing it needs a spare
  Zybo GPIO wired to the camera board, which isn't something I can do from
  software alone).
* **INCK frequency is still a best-effort default** (24MHz) — the board
  generates it locally from an oscillator whose exact frequency I couldn't
  confirm — see §0.
* **No AWB/AE/color-processing** — no ISP on the sensor to configure (see
  §4); if you add a demosaic path you'll likely want auto-exposure/AWB
  logic downstream of it too, which isn't included here.
* **No test-pattern-generator control** — the OV5640 driver had a
  `set_test()` color-bar helper; the IMX415 does have TPG registers
  (`TPG_EN_DUOUT`/`TPG_PATSEL_DUOUT` at `0x30E4`/`0x30E6` per the datasheet)
  but I didn't wire up a menu option for it here — straightforward to add
  following the same pattern as `writeReg`/`writeConfig`.

## 9. Troubleshooting quick-reference

| Symptom | Likely cause |
|---|---|
| `HardwareError::WRONG_ID` from `init()`, especially if it *never* passes no matter what | **Start with §5's reset-wiring gap** — the sensor's actual reset line may simply never be released. Probe `TP3` on the camera board while `reset()` runs to confirm. |
| `HardwareError::WRONG_ID`, other causes | Wrong I2C address (shouldn't be it — see §0), INCK not present (check the oscillator, §0), or a genuinely dead sensor. A chip ID read back as `0x000` or `0xFFF` usually means "nothing answered," consistent with the reset-wiring gap above. |
| `HardwareError::IIC_NACK` on register read/write | Bus contention, or sensor asleep/unpowered/held in reset. |
| Chip-ID check passes but streaming/timing seems off | Try `REG_SYS_MODE = 0x3034` instead of `0x3033` — see §2's note on the datasheet's internal inconsistency for that one register. |
| Everything initializes and the register menu works, but you never see anything meaningful on HDMI | Expected on the stock hardware — see §4. This isn't a bug to chase; there's no image pipeline wired to HDMI in this version at all. |
| Want to confirm frames are actually landing in DDR | Use a debugger memory view at `MEM_BASE_ADDR` (`DDR_BASE_ADDR + 0x0A000000`) after streaming starts, or add your own readback code — there's no on-screen path yet to eyeball it. |

## 10. References

* Sony **IMX415-AAQR-C datasheet** (you provided this) — primary source for
  the register map, the "INCK Setting" clock-configuration tables, the
  Power-on Sequence timing, and the SLAMODE0/1 I2C-address table used to
  cross-check §2 and §0's claims.
* **`SCH_IMX415_MIPI_FFC_CAM_REV1.pdf`** (you provided this) — schematic for
  the "IMX415 CAM R1" carrier board; source for §0/§5's I2C-address
  strapping, MIPI lane wiring, oscillator, and `CAM_RST`/`CAM_GPIO` findings.
* Sony IMX415 mainline Linux driver (original source of the register data
  in `IMX415.h`, later cross-checked against the datasheet above):
  [`drivers/media/i2c/imx415.c`](https://github.com/torvalds/linux/blob/master/drivers/media/i2c/imx415.c),
  GPL-2.0-only, © 2023 WolfVision GmbH.
* Original Digilent reference design this was adapted from: **Zybo Z7-20 +
  Pcam 5C** (OV5640) Vitis bare-metal application — see Digilent's
  `Zybo-Z7-20-pcam-5c` GitHub repository / project reference page for the
  companion Vivado hardware design.
