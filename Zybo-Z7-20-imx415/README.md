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
project was built against. Read **§3 "Hardware side"** and **§4 "The
raw-sensor problem"** below before you power anything up.

## 0. Your hardware, as photographed

From the photos: a Zybo Z7 board (HDMI RX populated ⇒ the Z7-20 variant this
project targets) with a Raspberry-Pi-style camera ribbon ("Standard-Mini,
200mm") running from the board's **Pcam MIPI connector** (the 15-pin FPC
header next to the SD-card slot / between HDMI TX and HDMI RX) to a small
carrier board silkscreened **"IMX415 CAM R1"** with an 8MP M12 lens fitted.

That's a genuinely useful data point: the Zybo's Pcam MIPI connector is the
same 15-pin FPC connector standard as the original Raspberry Pi camera
connector, which **only ever routes 2 CSI-2 data lanes**. So your module is
almost certainly wired for **2-lane** operation — which is what this
driver now defaults to (see §2). That doesn't resolve the lane-*rate*
(Mbps/lane) or INCK-frequency questions in §3, but it does substantially
de-risk the lane-*count* question I couldn't previously answer. I don't know
which specific vendor sells the "IMX415 CAM R1" board, so I can't pull its
exact INCK crystal value or a vendor-supplied register list — if you have a
product page, datasheet, or vendor example code for it, that would let us
replace the remaining assumptions in §3 with confirmed values.

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
numbers as trustworthy. Since then I found something much better: the
**mainline Linux kernel has a real, maintained IMX415 driver**,
[`drivers/media/i2c/imx415.c`](https://github.com/torvalds/linux/blob/master/drivers/media/i2c/imx415.c)
(GPL-2.0-only, © 2023 WolfVision GmbH). `IMX415.h` in this project now ports
its actual register addresses, its ~76-entry "magic"/undocumented analog
tuning table, its per-lane-rate MIPI D-PHY timing tables, and its
per-(lane-rate, INCK) clock-configuration tables — as plain numeric
configuration data, the same way embedded projects routinely port init
tables between drivers/languages. This is a categorically stronger basis
than a datasheet-derived guess: it's what a shipping driver actually writes
to real IMX415 hardware.

What this clarified, correcting assumptions in the first version of this
port:

* **The IMX415 has exactly one native readout mode**: a full-pixel-array
  raw Bayer scan at **3864×2192, RAW10** (`WINMODE` stays 0 — "all-pixel
  readout" — always; there's no sensor-side crop/binning mode). There is no
  real "1080p mode" or "4K mode" to select, unlike what the first version of
  this README implied. What *is* selectable is the **MIPI lane rate**
  (720 or 891 Mbps/lane are both fully specified in the upstream driver;
  1440 Mbps/lane also exists but needs a faster D-PHY, so I didn't wire it
  up here) and the **lane count** (2 or 4).
* **There's a real, documented chip-ID register**: `SENSOR_INFO` at
  `0x3F12` (16-bit), masked `0xFFF`, expected `0x514`. `IMX415::init()` now
  checks this (matching the OV5640 driver's own ID-check pattern) instead
  of the earlier version's indirect standby-readback heuristic.
* **Real power-up/reset timing**: the upstream driver only needs ~1µs after
  releasing reset before enabling the clock, then 100–200µs before the
  first I2C transaction, and 80ms after leaving standby before further
  access. `IMX415::reset()`/`init()`/`set_mode()` are annotated with these
  real numbers (the busy-wait loop itself is still uncalibrated — see the
  code comment on swapping in the Xilinx BSP's real `usleep()`).
* **I2C address 0x1A is confirmed** (mainline device-tree examples use
  `sony,imx415 @ 0x1a`), not just carried over as a guess.

### What's still genuinely unresolved (see §3)

Porting the register *values* correctly doesn't resolve board-specific
*parameters* those values are conditioned on:

| Item | Where | Status |
|---|---|---|
| **INCK (XVCLK) frequency** | `IMX415_cfg::INCK_HZ` in `IMX415.h` | Defaults to **24 MHz** (a common generic-breakout-board default) with the matching real clock-config table wired up for the 720 Mbps mode; a 27 MHz table is wired up for the 891 Mbps mode. If your "IMX415 CAM R1" board's crystal is different, `set_mode()` needs the matching `cfg_clk_*` table from `IMX415_cfg` swapped in (several are already included: 720Mbps@24/72MHz, 891Mbps@27/37.125/74.25MHz — add more from the upstream driver if needed). |
| **MIPI lane count** | `IMX415_cfg::NUM_DATA_LANES`, `LANEMODE_2LANE`/`4LANE` in `IMX415.h` | Defaults to **2-lane**, which §0 suggests is likely correct for your board's connector, but isn't independently confirmed. |
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

* The Pcam 5C connector drove the OV5640's power-down/reset with a single PS
  GPIO line (`PS_GPIO`'s `CAM_GPIO0`, EMIO pin 54 in `imx415/PS_GPIO.h`).
  `IMX415::reset()` reuses that exact same single-GPIO toggle sequence.
* If your "IMX415 CAM R1" board exposes separate power-enable and
  `XCLR`/reset pins, or needs power/clock stable *before* `XCLR` is
  released, extend `GPIO_Client::Bits` in `GPIO_Client.h` with a second bit,
  wire it to a second EMIO/MIO pin in `PS_GPIO.h`, and drive both in the
  correct order inside `IMX415::reset()`.
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
* **INCK frequency and lane count are still assumptions**, not confirmed
  for your specific "IMX415 CAM R1" board — see §0 and §2.
* **No AWB/AE/color-processing** — no ISP on the sensor to configure (see
  §4); if you add a demosaic path you'll likely want auto-exposure/AWB
  logic downstream of it too, which isn't included here.
* **No test-pattern-generator control** — the OV5640 driver had a
  `set_test()` color-bar helper; the IMX415 does have TPG registers
  (`TPG_EN_DUOUT`/`TPG_PATSEL_DUOUT` at `0x30E4`/`0x30E6` per the upstream
  driver) but I didn't wire up a menu option for it here — straightforward
  to add following the same pattern as `writeReg`/`writeConfig`.

## 9. Troubleshooting quick-reference

| Symptom | Likely cause |
|---|---|
| `HardwareError::WRONG_ID` from `init()` | Wrong I2C address, sensor not powered, `reset()` sequencing wrong for your board, INCK not present, or a genuinely different/dead sensor. A chip ID read back as `0x000` or `0xFFF` usually means "nothing answered," not "wrong sensor." |
| `HardwareError::IIC_NACK` on register read/write | Bus contention, wrong address, or sensor asleep/unpowered. |
| Everything initializes and the register menu works, but you never see anything meaningful on HDMI | Expected on the stock hardware — see §4. This isn't a bug to chase; there's no image pipeline wired to HDMI in this version at all. |
| Want to confirm frames are actually landing in DDR | Use a debugger memory view at `MEM_BASE_ADDR` (`DDR_BASE_ADDR + 0x0A000000`) after streaming starts, or add your own readback code — there's no on-screen path yet to eyeball it. |

## 10. References

* Sony IMX415 mainline Linux driver (source of the register data in
  `IMX415.h`): [`drivers/media/i2c/imx415.c`](https://github.com/torvalds/linux/blob/master/drivers/media/i2c/imx415.c),
  GPL-2.0-only, © 2023 WolfVision GmbH.
* Original Digilent reference design this was adapted from: **Zybo Z7-20 +
  Pcam 5C** (OV5640) Vitis bare-metal application — see Digilent's
  `Zybo-Z7-20-pcam-5c` GitHub repository / project reference page for the
  companion Vivado hardware design.
