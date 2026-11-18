# PASS_MVDR_STAP: latest STAP build package

This package holds the latest firmware and hardware for the AntSDR E310 LVDS MVDR / SMI-STAP anti-jam board. It builds `BOOT.bin` from a command prompt; no Vitis IDE clicks are needed.

| Part | Version | Notes |
|---|---|---|
| Hardware | `hw/PASS_MVDR_STAP.xsa` (Vivado 2023.2, built 11 Nov 2026 18:31) | Contains the bitstream. `gnss_passthrough` is at 0x43C10000 with `STAP_LMS=0`. |
| Firmware | `sw/src/`: **app_ant_mvdr2 MOD13** (18 Nov 2026) | Byte-for-byte the same as the last version built in Vitis. |
| Project settings | `sw/config/lscript.ld`, `sw/config/UserConfig.cmake` | Same as the Vitis project: -O0, -g3, -Wall -Wextra, links `libm`. |
| Reference output | `prebuilt/BOOT.bin`, `prebuilt/PASS_MVDR_STAP_test.elf`, `prebuilt/fsbl.elf` | Made by the Vitis IDE on 18 Nov 2026 from the same XSA and MOD13. |
| Vivado project | `hdl/Passthrough_MVDR/passthrough_mvdr.xpr` | The full project: block design, RTL, generated IP and reports. See "Open the hardware in Vivado" below. |

Nothing in the firmware or hardware was changed for this package.

## What you need

- **AMD Vitis 2023.2**, the same version that built the original. This tool set includes `vitis`, `bootgen` and the ARM compiler.
- A folder path with **no spaces**, for example `C:\work\PASS_MVDR_STAP_latest`.

---

## Windows: step-by-step commands (Command Prompt)

**1. Unzip the package to a path without spaces.**

```bat
mkdir C:\work
tar -xf %USERPROFILE%\Downloads\PASS_MVDR_STAP_latest.zip -C C:\work
```

**2. Open Command Prompt and go into the package.**

```bat
cd /d C:\work\PASS_MVDR_STAP_latest
```

**3. Load the Vitis 2023.2 tools.** Change the path if Vitis is installed somewhere else.

```bat
call C:\Xilinx\Vitis\2023.2\settings64.bat
```

**4. Check that the tools are found.**

```bat
where vitis
where bootgen
```

**5. Build the platform, the FSBL, the firmware and BOOT.bin.** This takes about 5 to 10 minutes.

```bat
vitis -s scripts\build_fw.py
```

**6. The result is in `output\`.**

```bat
dir output
```

You should see `BOOT.bin`, `boot.bif`, `fsbl.elf`, `PASS_MVDR_STAP.bit` and `PASS_MVDR_STAP_test.elf`.

**Short way:** steps 3 to 6 in one command.

```bat
build.bat C:\Xilinx\Vitis\2023.2
```

## Linux: step-by-step commands (terminal)

```bash
unzip PASS_MVDR_STAP_latest.zip -d ~/work
```

```bash
cd ~/work/PASS_MVDR_STAP_latest
```

```bash
source /tools/Xilinx/Vitis/2023.2/settings64.sh
```

```bash
vitis -s scripts/build_fw.py
```

```bash
ls -l output/
```

**Short way:**

```bash
./build.sh /tools/Xilinx/Vitis/2023.2
```

---

## What `scripts/build_fw.py` does

These are the same steps the Vitis IDE ran (see the Vitis `logs/builder*.py`).

1. It creates a fresh workspace in `build/ws`. Each run deletes the old one.
2. It runs `create_platform_component(name="PASS_MVDR_STAP", hw=hw/PASS_MVDR_STAP.xsa, os="standalone", cpu="ps7_cortexa9_0")` and builds it. This gives the `standalone_ps7_cortexa9_0` domain, the `zynq_fsbl` boot domain and `fsbl.elf`.
3. It runs `create_app_component(name="PASS_MVDR_STAP_test", domain="standalone_ps7_cortexa9_0")` with the empty_application template. It then imports `sw/src` and copies in `lscript.ld` and `UserConfig.cmake`.
4. It runs `comp.build()`, which produces `PASS_MVDR_STAP_test.elf`.
5. It runs `bootgen -arch zynq -image output/boot.bif -o output/BOOT.bin -w on`.

The `boot.bif` it writes has the same order as the IDE's:

```
the_ROM_image:
{
    [bootloader]fsbl.elf
    PASS_MVDR_STAP.bit
    PASS_MVDR_STAP_test.elf
}
```

### Running bootgen by hand

If the script stops after the build step, you can run bootgen yourself. The files are already in `output\`.

```bat
bootgen -arch zynq -image output\boot.bif -o output\BOOT.bin -w on
```

## Load BOOT.bin onto the board

1. Copy `output\BOOT.bin` to the root of a FAT32 SD card. The file must be named `BOOT.bin`.
2. Set the E310 boot switch to **SD**, insert the card and power on.
3. Open the console on the UART (USB serial) at **115200 8N1**. Type `?` to see the menu, and `stap=1` to start the SMI-STAP automatic start.

To compare with the known-good build, `prebuilt/BOOT.bin` is the image Vitis produced on 18 Nov from the same sources.

---

## Open the hardware in Vivado

The Vivado project is at `hdl/Passthrough_MVDR/passthrough_mvdr.xpr`.

```bash
./open_vivado.sh /tools/Xilinx/Vivado/2023.2
```

Or open it by hand:

```bash
source /tools/Xilinx/Vivado/2023.2/settings64.sh
```

```bash
cd hdl/Passthrough_MVDR
```

```bash
vivado passthrough_mvdr.xpr &
```

On Windows, run `open_vivado.bat C:\Xilinx\Vivado\2023.2`.

- To see the block design, go to **Flow Navigator → IP Integrator → Open Block Design** (`system.bd`).
- The RTL is under **Sources → Design Sources** (`system_top.v`, `rtl/*.v`).

Vivado will warn that the IP repository `../../library` is missing, and the Analog Devices IP blocks (axi_ad9361, axi_dmac, util_cpack2/upack2, axi_vcxo_ctrl) will show as **locked**. That is expected. You can view and edit the block design and RTL. To regenerate those blocks or rebuild the bitstream, you need the antsdr HDL `library` folder (see the next section).

## Optional: rebuild the hardware (XSA) from HDL

You don't need this to make `BOOT.bin`; the XSA in `hw/` is already the latest hardware. Use it only if the HDL changes.

The project uses the Analog Devices HDL build flow from `antsdr-fw-patch/plutosdr-fw/hdl` with Vivado 2023.2. Linux, or Cygwin/WSL on Windows, is needed for `make`.

```bash
cp -r hdl/Passthrough_MVDR <antsdr-fw-patch>/plutosdr-fw/hdl/projects/Pass_MVDR_STAP
```

```bash
cd <antsdr-fw-patch>/plutosdr-fw/hdl/projects/Pass_MVDR_STAP
```

```bash
source /tools/Xilinx/Vivado/2023.2/settings64.sh
```

```bash
make clean && make
```

Without `make`, run Vivado directly:

```bash
vivado -mode batch -source system_project.tcl
```

The output is `passthrough_mvdr.sdk/system_top.xsa`. Copy it to `hw/PASS_MVDR_STAP.xsa` and run the firmware build again.

The last implementation run met timing with zero slack (WNS 0.000 ns, WHS 0.025 ns). See `hdl/Passthrough_MVDR/timing_report.txt` and `utilization_report.txt`.

## Troubleshooting

| Problem | Fix |
|---|---|
| `'vitis' is not recognized` | Run `settings64.bat` first, in the same Command Prompt window. |
| Errors about paths | Move the package to a short path with no spaces. |
| `bootgen not on PATH` | Run the bootgen command from "Running bootgen by hand" above. |
| Version warning about the XSA | Use Vitis **2023.2**, the version the XSA was made with. |
| Need a clean rebuild | Delete the `build` and `output` folders and run again. |
