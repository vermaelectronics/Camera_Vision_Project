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
project was built against. Read the **"Hardware side — what you still need
to do"** section below before you power anything up; it explains exactly why
that matters for the IMX415 and what to change.

---

## 1. What actually changed

| Original (OV5640 / Pcam 5C)                          | This project (IMX415)                                        | Why |
|--------------------------------------------------------|----------------------------------------------------------------|-----|
| `src/ov5640/OV5640.h`, `OV5640.cpp`                     | `src/imx415/IMX415.h`, `IMX415.cpp`                             | New driver class: different register map, different sensor architecture (raw sensor vs. sensor+ISP) |
| `src/main.cc` — includes `ov5640/OV5640.h`, instantiates `OV5640 cam(...)` | `src/main.cc` — includes `imx415/IMX415.h`, instantiates `IMX415 cam(...)` | Swap the driver actually used by the app |
| Menu option **b. Change Liquid Lens Focus** (writes I2C address `0x46>>1`) | **removed** | That's the Pcam 5C's variable-focus liquid-lens IC, a separate chip on *that* board. Most IMX415 breakout/carrier boards don't have one. See §5 if yours does. |
| Menu option **d. Change Image Format (Raw or RGB)** | **removed** | Only meaningful because the OV5640 has an internal ISP that can convert Bayer → RGB on-sensor. The IMX415 has no ISP; it only ever outputs raw Bayer data. |
| Menu option **h. Change AWB Settings** | **removed** | Same reason — AWB in the original code is an OV5640 ISP feature (register writes into its ISP block). The IMX415 has nothing equivalent to configure. |
| Menu options **e/f** (write/read sensor register) | kept, renumbered to **b/c** | Generic I2C register poke tool — extremely useful for IMX415 bring-up/debug, so it's kept as-is. |
| Menu option **g** (gamma factor) | kept, renumbered to **d** | This drives the `AXI_GammaCorrection` IP core downstream in the FPGA fabric, not the sensor. Nothing sensor-specific about it. |
| Resolution menu: 720p60 / 1080p15 / 1080p30 (OV5640 register tables) | Resolution menu: 1080p30 / "4K30 (capture-only)" (IMX415 register tables) | New sensor, new native modes. See §3 for the crucial caveat on the 4K option. |
| `ov5640/PS_IIC.h`, `PS_GPIO.h`, `I2C_Client.h`, `GPIO_Client.h`, `ScuGicInterruptController.h`, `AXI_VDMA.h` | copied to `imx415/` **unchanged**, only the folder moved | These are generic Zynq PS peripheral drivers (I2C, GPIO, interrupt controller, VDMA). Nothing in them is OV5640-specific — they talk to *whatever* `I2C_Client`/`GPIO_Client` implementation you hand them, which is now `IMX415` instead of `OV5640`. |
| `hdmi/VideoOutput.h` | unchanged | Drives the HDMI output timing (VTC + clock wizard). Independent of which sensor feeds the pipeline. |
| `platform/*`, `lscript.ld`, `Xilinx.spec` | unchanged | Generic Zynq-7000 PS bring-up / linker script generated from the (unchanged) hardware platform. |
| `.project` / `.cproject` | copied, renamed to `Zybo-Z7-20-imx415`, still depends on the `system_wrapper` hardware platform project | See §3 — you may need to re-export `system_wrapper` from a rebuilt Vivado design. |

The new sensor driver, `IMX415.h`, deliberately mirrors the **shape** of
`OV5640.h` (same `reset()` / `init()` / `set_mode()` / `readReg()` /
`writeReg()` interface, same config-table pattern) so the rest of the
pipeline code (`main.cc`, VDMA, video output) barely had to change. What's
different is the actual register map, because these are very different
sensors:

* **OV5640**: has an internal ISP (auto white balance, RGB/YUV formatting,
  color matrix, gamma, etc.) and a readable chip-ID register pair
  (`0x300A`/`0x300B` = `0x56 0x40`).
* **IMX415**: a "dumb" raw sensor. It outputs Bayer RAW10/RAW12 only, has no
  on-chip ISP, and (per Sony's public documentation) has no simple chip-ID
  register the way OV/Aptina sensors do. `IMX415::init()` therefore does a
  **communication sanity check** instead of an ID check: it writes the
  `STANDBY` register and reads it back to confirm the sensor is present and
  answering on I2C.

---

## 2. Files in this project

```
Zybo-Z7-20-imx415/
├── .project, .cproject        Vitis application-project metadata (renamed from Zybo-Z7-20-pcam-5c)
├── .gitignore                 Ignores Debug/Release build output
├── README.md                  This file
└── src/
    ├── main.cc                 Application entry point + serial menu (adapted for IMX415)
    ├── lscript.ld               Linker script (generic, from the hardware platform's memory map)
    ├── Xilinx.spec               Linker spec file (generic)
    ├── platform/
    │   ├── platform.c/.h         Zynq PS bring-up (cache enable, PL reset, ...) — generic
    │   └── platform_config.h
    ├── hdmi/
    │   └── VideoOutput.h          VTC + clocking-wizard driver for HDMI output — generic
    └── imx415/
        ├── IMX415.h / .cpp       *** NEW: IMX415 sensor driver (replaces ov5640/OV5640.*) ***
        ├── I2C_Client.h           Abstract I2C interface (generic)
        ├── GPIO_Client.h          Abstract GPIO interface (generic)
        ├── PS_IIC.h                Zynq PS I2C controller driver, implements I2C_Client (generic)
        ├── PS_GPIO.h                Zynq PS GPIO driver, implements GPIO_Client (generic)
        ├── ScuGicInterruptController.h   GIC wrapper used by PS_IIC/PS_GPIO/AXI_VDMA (generic)
        └── AXI_VDMA.h              AXI VDMA driver moving pixel data sensor↔DDR↔HDMI (generic)
```

I did **not** copy the hardware platform export (`system_wrapper/`,
`sdx_export_metadata/`, the `.bit`/`.xsa`, the FSBL, etc.) from the original
zip into this repository. That's ~large, mostly-binary, Vivado-generated
content, it's exactly what this repo's top-level `.gitignore` already says
not to track (`*.bit`, `*.log`, `.Xil/`, `*.runs/`, …), and — see next
section — it needs to be **rebuilt**, not merely copied, if you want proper
IMX415 support.

---

## 3. Hardware side — what you still need to do

This is the single most important thing to understand: **the original
`system_wrapper.bit` bitstream was built around the OV5640's MIPI
characteristics**, specifically:

* **2 MIPI D-PHY / CSI-2 lanes** (the OV5640 modes in the original project
  top out at 2-lane, ≤336 MHz MIPI serial clock),
* **RAW10** pixel format matched to the AXI4-Stream / VDMA word width the
  `MIPI_CSI_2_RX` and `MIPI_D_PHY_RX` IP cores were generated for,
* HDMI output timing entries in `hdmi/VideoOutput.h` that only go up to
  **1920×1080p60**.

The IMX415 is commonly run at **4 MIPI lanes** and a higher data rate to get
its full 3840×2160 resolution out at usable frame rates. If you plug an
IMX415 module into hardware that's still running the OV5640-era bitstream:

* The software in this project will **compile and run** fine (it doesn't
  depend on the sensor's lane count at the C++ level).
* The sensor will power up and accept I2C register writes (you can bring it
  up, read/write registers with the `b`/`c` menu options, and confirm it's
  alive).
* **But no valid image data will arrive** if your board's IMX415 module is
  wired for 4 lanes while the `MIPI_D_PHY_RX`/`MIPI_CSI_2_RX` IP in the
  bitstream is only configured/wired for 2, or if the data rate is outside
  what the D-PHY IP was generated for. The CSI-2 receiver simply won't lock
  onto the stream.

**To get a fully working IMX415 capture path you need to, in Vivado:**

1. Regenerate/reconfigure the `MIPI_D_PHY_RX` and `MIPI_CSI_2_RX` IP cores
   for your IMX415 module's actual lane count and MIPI clock, matching the
   `DATARATE_SEL` you choose in `IMX415.h`.
2. Confirm/adjust the AXI4-Stream data width feeding the `AXI_VDMA` write
   channel matches RAW10 (8 lanes × 10-bit packed, matching this project's
   default `BIT_DEPTH_SEL = 0` in `IMX415.h`) — or widen it and change
   `BIT_DEPTH_SEL` to `1` if you'd rather run RAW12.
3. If you want a **live HDMI preview at 4K**, add a `3840×2160` entry to the
   `timing[]` table in `hdmi/VideoOutput.h` and make sure the clocking
   wizard / VTC / HDMI output IP and your HDMI sink actually support it.
   Without this, `main.cc`'s 4K mode only exercises the sensor → DDR capture
   path (VDMA "write" side); the on-screen HDMI preview still runs at the
   1080p timing (see the comments in `main.cc`, case `'a'` → `'2'`).
4. Re-run **File → Export → Export Hardware** (include bitstream) in Vivado,
   which regenerates `system_wrapper.xsa`; re-associate this application's
   `system_wrapper` platform project in Vitis with the new export.
5. Check your IMX415 module's power-up sequencing (rail order, reset/XCLR
   timing) against its datasheet or vendor application note, and adjust
   `IMX415::reset()` in `IMX415.h` if it needs more than a single GPIO
   toggle (see §5).

If your goal is only to validate the **software architecture and I2C
bring-up** (confirm the sensor answers on I2C, register menu works, etc.)
you can do that on the existing OV5640-era hardware platform without
touching Vivado — just don't expect a clean image until the MIPI IP matches
your sensor's actual lane count/data rate.

---

## 4. Values you must confirm before first power-up

I do not have your exact carrier board or a copy of Sony's IMX415 datasheet
in front of me while writing this, so I left the sensor-specific numeric
constants in `src/imx415/IMX415.h` as **clearly marked placeholders**
(`TODO`/`VERIFY` comments) rather than inventing numbers and presenting them
as verified working values. Before you trust this on real hardware, fill
in/confirm:

| Item | Where | What to do |
|---|---|---|
| **I2C address** | `IMX415::dev_address_` (`0x1A` 7-bit) | This is the commonly documented IMX415 address, but some vendor breakout boards strap an alternate address. If `init()` throws `HardwareError::NO_RESPONSE`, scan the bus or check your board's schematic. |
| **INCK_SEL** | `IMX415_cfg::INCK_SEL` | Encodes your board's actual XVCLK input frequency to the sensor. Depends entirely on your carrier board's oscillator/clock source — check the schematic and the datasheet's `INCK_SEL` table. |
| **DATARATE_SEL** | `IMX415_cfg::DATARATE_SEL` | Encodes your target MIPI Mbps/lane. Must be chosen consistently with the D-PHY IP configuration in Vivado (§3). |
| **Lane count bitfield** | `IMX415_cfg::LANE_SEL_2LANE` / `LANE_SEL_4LANE` | Confirm the exact bit pattern for register `0x3A01` (or wherever your datasheet revision documents lane-count select) against your datasheet copy. |
| **"Black-box" analog/timing register block** | `IMX415_cfg::cfg_common_init_` (marked with a large comment block) | Sony's IMX415 register-setting application note supplies ~100 additional analog-tuning/timing registers as a fixed table per INCK/data-rate/lane/bit-depth combination. These aren't independently derivable and are **intentionally not fabricated here** — copy them verbatim from the application note (NDA'd through Sony/your module vendor) or from a reference open-source driver for the exact same INCK/lane/data-rate/bit-depth combination you're using (e.g. the mainline Linux kernel's `imx415.c` driver uses the same register set and is a good cross-check once you've picked your operating point). |
| **VMAX / HMAX (frame/line timing)** | `cfg_1080p_30fps_` / `cfg_4k_30fps_` in `IMX415.h` | Marked `TODO/VERIFY`. Recompute using the datasheet's frame-rate formula for your chosen INCK and DATARATE_SEL — don't trust the placeholder values for exact timing. |
| **CSI-2/D-PHY IP lane count & speed** | Vivado hardware design | Must match whatever lane count/data rate you picked above — see §3. |

None of these being "wrong" will damage the sensor — worst case is no image,
a garbled image, or an I2C NACK — but you do need to get them right (or at
least self-consistent) before you'll see a picture.

---

## 5. Wiring & GPIO notes

* The Pcam 5C connector drove the OV5640's power-down/reset with a single PS
  GPIO line (`PS_GPIO`'s `CAM_GPIO0`, mapped to EMIO pin 54 in
  `imx415/PS_GPIO.h`). `IMX415::reset()` reuses that exact same single-GPIO
  toggle sequence.
* Many IMX415 breakout boards expose **separate** power-enable and
  `XCLR`/reset pins, and/or need the sensor's clock and power rails stable
  *before* `XCLR` is released (check your board's power-up sequencing
  requirements). If that's your board, extend `GPIO_Client::Bits` in
  `GPIO_Client.h` with a second bit, wire it to a second EMIO/MIO pin in
  `PS_GPIO.h`, and drive both pins in the correct order inside
  `IMX415::reset()`.
* If your particular IMX415 carrier board *does* include a liquid-lens or
  motorized-focus driver IC (uncommon, but some do), you can bring back the
  removed "Change Liquid Lens Focus" menu option: add a `writeRegLiquid()`
  method to `IMX415` following the same pattern as the original
  `OV5640::writeRegLiquid()`, then re-add the `case 'e':` block from the
  original `main.cc` (see the [Digilent original](https://github.com/Digilent/Zybo-Z7-20-pcam-5c), if you have access to it) using your lens IC's I2C address.

---

## 6. Building in Vitis

1. Import (or keep) the `system_wrapper` platform project — either the one
   exported from the original OV5640-era Vivado design (fine for software
   bring-up per §3), or your rebuilt IMX415-ready one.
2. In Vitis: **File → Import → Git Repository / Existing Vitis application
   project**, or **File → New → Application Project**, pointing at this
   `Zybo-Z7-20-imx415` folder. `.project`/`.cproject` are already set up to
   depend on a platform project named exactly `system_wrapper` — either
   import a platform with that name, or update the references in
   `.project`/`.cproject` (search for `system_wrapper`) to match your
   platform project's actual name.
3. Build the `Debug` or `Release` configuration as normal. The BSP must
   include: `xiicps`, `xgpiops`, `xscugic`, `xaxivdma`, `xvtc`, `xclk_wiz`
   drivers, plus whatever custom `MIPI_D_PHY_RX`/`MIPI_CSI_2_RX` driver your
   platform's hardware design generates — these are the same BSP
   dependencies the original OV5640 project needed, unchanged.
4. Program the FPGA with your (existing or rebuilt) bitstream, then run/debug
   the ELF on the Cortex-A9 as usual, with a serial terminal (115200 8N1)
   attached to the board's UART.

---

## 7. Using it

On boot the app brings up the pipeline in **1920×1080 @ 30fps** and prints:

```
Video init done.
```

Then a serial menu repeats:

```
IMX415 MAIN OPTIONS

Please press the key corresponding to the desired option:
  a. Change Resolution
  b. Write a Register Inside the Image Sensor
  c. Read a Register Inside the Image Sensor
  d. Change Gamma Correction Factor Value
```

* **a** → choose `1` for 1920×1080p30 (live HDMI preview) or `2` for
  3840×2160p30 (DDR capture only on stock hardware timing — see §3).
* **b** / **c** → poke/peek any IMX415 register directly over I2C. This is
  the fastest way to sanity-check bring-up: write `STANDBY` (`3000`) and
  read it back, confirm `INCK_SEL`/`DATARATE_SEL` landed correctly, etc.
* **d** → cycles the downstream FPGA gamma-correction IP core's factor
  (1, 1/1.2, 1/1.5, 1/1.8, 1/2.2) — unrelated to the sensor itself.

If option **a** or a sensor register read fails with `HardwareError`, the
error message printed over serial will tell you whether it was an I2C NACK
(wiring/address problem) or (for `init()`) a `STANDBY` read-back mismatch
(sensor not responding as expected — check power sequencing, clock, and I2C
address first).

---

## 8. Known limitations / explicitly out of scope here

* **No FPGA/bitstream changes** — see §3. This is a software-only edit of
  the Vitis application source, as requested.
* **No verified numeric register table** for the IMX415's analog/timing
  "black box" registers — see §4. The driver architecture is complete and
  correct; the exact numeric fill-in is left to you with a clear checklist,
  rather than guessed and presented as trustworthy.
* **No AWB/AE/color-processing** — the IMX415 has no ISP, so (unlike the
  OV5640 build) there's nothing on-sensor to configure for this. If you need
  auto-exposure/AWB, it has to be implemented either in the FPGA fabric or
  in software on captured frames — neither is included here.
* **No true 4K HDMI output** on the stock hardware platform — 4K sensor
  capture to DDR is wired up in software, but there's no 4K entry in the
  HDMI output timing table (`hdmi/VideoOutput.h`) unless you add one and the
  supporting clocking, per §3.
* **No test-pattern-generator control** — the OV5640 driver had a
  `set_test()` color-bar helper; I didn't carry an equivalent over because I
  don't have a verified IMX415 test-pattern register address to offer
  (rather than guess one).

---

## 9. Troubleshooting quick-reference

| Symptom | Likely cause |
|---|---|
| `HardwareError::NO_RESPONSE` thrown from `init()` | Wrong I2C address, sensor not powered, `reset()` sequencing wrong for your board, or I2C bus not wired/pulled up correctly. |
| `HardwareError::IIC_NACK` on register read/write | Bus contention, wrong address, or sensor asleep/unpowered. |
| Everything initializes, menu works, but HDMI shows black/garbage/no signal | Almost certainly the §3 lane-count/data-rate mismatch between the sensor's actual MIPI output and the `MIPI_D_PHY_RX`/`MIPI_CSI_2_RX` IP the bitstream was built with. |
| Image present but colors/exposure look wrong | Expected without AWB/AE — see §8. Also double-check `VMAX`/`HMAX` (§4) if the image is torn/rolling. |

---

## 10. References

* Original Digilent reference design this was adapted from: **Zybo Z7-20 +
  Pcam 5C** (OV5640) Vitis bare-metal application — see Digilent's
  `Zybo-Z7-20-pcam-5c` GitHub repository / project reference page for the
  companion Vivado hardware design.
* Sony **IMX415** datasheet / register-setting application note (obtain from
  Sony or your sensor module vendor) — authoritative source for §4's
  placeholder values.
* Mainline Linux kernel `drivers/media/i2c/imx415.c` — a useful independent
  cross-check for the IMX415's standard control-register addresses and
  common operating-point register tables once you've picked your
  INCK/lane/data-rate/bit-depth combination.
