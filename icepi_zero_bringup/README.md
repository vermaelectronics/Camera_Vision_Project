# IcePi-Zero bring-up collection

Four independent sub-projects for the IcePi-Zero board (Lattice ECP5
LFE5U-25F-6BG256C, Raspberry-Pi-Zero form factor —
[github.com/cheyao/icepi-zero](https://github.com/cheyao/icepi-zero)),
each its own self-contained RTL project: real testbench-verified RTL,
run through the real open-source ECP5 toolchain (synthesis + place &
route), with a README documenting exactly what's confirmed and what
isn't. Ordered roughly by how much they build on each other:

| # | Sub-project | What it proves | Status |
|---|---|---|---|
| 1 | [`01_led_blink`](01_led_blink/) | The whole toolchain/board/flashing chain works at all | Sim + synth + P&R verified |
| 2 | [`02_sdcard_text_reader`](02_sdcard_text_reader/) | SD card SPI + FAT16 + UART | Sim (3 testbenches) + synth + P&R verified |
| 3 | [`03_sdcard_hdmi_image`](03_sdcard_hdmi_image/) | SD card → BMP → HDMI display | Sim (2 testbenches) + synth + P&R verified |
| 4 | [`04_usb_mouse`](04_usb_mouse/) | Bit-banged USB host reading a mouse | PHY layer sim-verified; host-controller layer synthesis-only (see its README) |

The sibling [`dvp_camera_hdmi_pipeline`](../dvp_camera_hdmi_pipeline/)
project (camera → HDMI, the first thing built in this repo) isn't part of
this collection's numbering, but is included alongside it below — it's
the most hardware-battle-tested project here, having gone through several
real-hardware debugging rounds documented in its own README.

## Shared design and discipline

- **Real testbench verification before synthesis, always.** Every
  sub-project's RTL is checked against Icarus Verilog testbenches before
  being trusted, matching the discipline established in the camera
  project. Where a piece genuinely can't be simulated (an ECP5 PLL
  primitive with no open behavioral model; a full USB device with no
  behavioral model in this repo), that's stated plainly rather than
  glossed over — see each sub-project's own "Verified" section.
- **Real toolchain, every time.** `synth_ecp5` (Yosys) and `nextpnr-ecp5`
  are run for real against each sub-project's real constraints file, not
  assumed to work.
- **Reuse across sub-projects, not copy-and-diverge:**
  - `02_sdcard_text_reader`'s `sdcard_spi.v` + `fat16_reader.v` are reused
    byte-for-byte, unmodified, by `03_sdcard_hdmi_image` — a FAT16 file's
    bytes are just bytes in order, regardless of whether a UART or a BMP
    parser makes sense of them.
  - `03_sdcard_hdmi_image` reuses `dvp_camera_hdmi_pipeline`'s HDMI/TMDS
    pipeline (`clk_gen_dvi.v`, `video_timing_gen.v`, `tmds_encoder.v`,
    `async_fifo.v`, `tmds_serial_gearbox.v`, `test_pattern_gen.v`,
    `dp_line_ram.v`) unmodified.
  - `uart_tx.v` and the button-debounce/status-LED pattern are shared
    across `02`, `03`, and `04`.
- **Real board pin sites everywhere**, taken directly from the IcePi-Zero
  board's own published LPF (`gateware/icepi-zero.lpf` at
  [github.com/cheyao/icepi-zero](https://github.com/cheyao/icepi-zero)) —
  never guessed. The board has a real onboard microSD slot wired for SPI
  mode (used by `02`/`03`) and two raw USB D+/D- pin pairs with
  switchable pull resistors (used by `04`) — no breakout boards or
  external wiring needed for any of this collection's peripherals.

## Toolchain install (one-time, shared by every sub-project)

The full open-source ECP5 flow: [Icarus
Verilog](http://iverilog.icarus.com/) for simulation, and
[Yosys](https://github.com/YosysHQ/yosys) + [Project
Trellis](https://github.com/YosysHQ/prjtrellis) +
[nextpnr-ecp5](https://github.com/YosysHQ/nextpnr) for synthesis/P&R/pack,
plus [openFPGALoader](https://github.com/trion137/openFPGALoader) to
flash. The easiest path is the bundled
[OSS CAD Suite](https://github.com/YosysHQ/oss-cad-suite-build) release,
which ships all of the above prebuilt — it's a userspace application
toolchain (not an OS or bare-metal SDK), so it installs and runs like any
other set of command-line tools on your existing Linux/macOS/Windows
install.

```bash
# example: OSS CAD Suite
wget https://github.com/YosysHQ/oss-cad-suite-build/releases/latest/download/oss-cad-suite-linux-x64-<date>.tgz
tar xzf oss-cad-suite-linux-x64-*.tgz
source oss-cad-suite/environment
```

Then, in any sub-project directory:

```bash
make check     # simulate -- always do this first
make prog      # synth + place&route + pack + flash, in one step
```

## Which sub-project to build first

If this is a fresh board, or you're not sure the toolchain/flashing flow
even works yet: **start with `01_led_blink`.** It's deliberately trivial
— if it doesn't blink after flashing, the problem is almost certainly
toolchain/board/wiring, not RTL logic, and every other sub-project here
depends on that same chain working.
