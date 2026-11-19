# STAP_16x16_latest: STAP 16 + 16 + 2 (MOD13) build package

This package builds **one SD-card image** for the AntSDR E310 LVDS MVDR / SMI-STAP anti-jam board, which **boots with 16 taps per antenna + 2 image taps (16 + 16 + 2 = 34 weights)**. It runs from a command prompt; no Vitis IDE clicks are needed.

The companion package **STAP_5x5_latest** is identical except for the taps at boot: same bitstream, same FSBL, same firmware source.

| Part | What | Notes |
|---|---|---|
| Hardware | `hw/PASS_MVDR_STAP.xsa` | **Replace it with your corrected XSA** (power optimization + waivers + DDR fix; timing WNS 0.056 / WHS 0.007, 0 critical warnings): `~/antsdr/antsdr-fw-patch/plutosdr-fw/hdl/projects/Pass_MVDR_STAP/PASS_MVDR_STAP.xsa`. The XSA shipped here is the original 11 Nov build. |
| Firmware | `sw/src/` = **MOD13** + one build option | `gnss_smi.c`: `GNSS_SMI_TAPS_BOOT` = taps per antenna at boot. Not set means all 16 (the MOD13 behaviour). Nothing else changed. |
| This package's option | **`GNSS_SMI_TAPS_BOOT = not set (16)`** | Set by `scripts/build_fw.py` in the app's `UserConfig.cmake` |
| Project settings | `sw/config/lscript.ld`, `sw/config/UserConfig.cmake` | As MOD13's Vitis project: -O0 -g3, libm |
| Vivado project | `hdl/Passthrough_MVDR/` | Includes `run_to_xsa.tcl`, `improve_hw.tcl` and `fix_ddr_drc.tcl` (the corrections) |
| Expected suppression (12 Nov rooftop) | 27–32 dB against a wideband barrage, 40–47 dB against CW | `taps=1..16` still changes it live |

## Build (Linux)

**1. Unzip:**
```bash
unzip -o ~/Downloads/STAP_16x16_latest.zip -d ~/work
```

**2. Use your corrected hardware:**
```bash
cp ~/antsdr/antsdr-fw-patch/plutosdr-fw/hdl/projects/Pass_MVDR_STAP/PASS_MVDR_STAP.xsa ~/work/STAP_16x16_latest/hw/PASS_MVDR_STAP.xsa
```

**3. Build the firmware and `BOOT.bin`** (about 5 to 10 minutes):
```bash
cd ~/work/STAP_16x16_latest && ./build.sh /tools/Xilinx/Vitis/2023.2 2>&1 | tee fw_build.log
```

**4. Check:**
```bash
ls -l output/
```
```bash
grep -n "GNSS_SMI_TAPS_BOOT" build/ws/STAP_16x16/src/UserConfig.cmake build/ws/STAP_16x16/src/gnss_smi.c | head
```
```bash
grep -nE "error:|undefined reference" fw_build.log | head
```

**5. Copy it to the SD card:**
```bash
cp ~/work/STAP_16x16_latest/output/BOOT.bin /media/$USER/<card>/BOOT.bin && sync && umount /media/$USER/<card>
```

## On the board (115200 baud)
```
taps?        -> should say 16 per antenna at boot
stap?        -> "2 antennas x 16 taps in use of 16 + 2 image taps = 34"
taps=16      -> switch live to 16 + 16 + 2
taps=5       -> switch live to 5 + 5 + 2
```

## Windows
```bat
build.bat C:\Xilinx\Vitis\2023.2
```

## Rebuild the hardware (optional)
The Vivado project is in `hdl/Passthrough_MVDR/`. The corrections used for the current hardware are the scripts there: `run_to_xsa.tcl`, `improve_hw.tcl` (power optimization + methodology waivers) and `fix_ddr_drc.tcl` (PS7 DDR DRC waiver). Run them in the antsdr tree project `hdl/projects/Pass_MVDR_STAP`, as before.
