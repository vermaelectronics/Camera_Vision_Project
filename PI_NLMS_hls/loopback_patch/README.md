# LoopBack_Code patch: gnss_passthrough v1.3 with the PI-NLMS (HLS) core

This folder mirrors the `LoopBack_Code` tree. Copy it over your `LoopBack_Code`
and run `build_crpa_linux.sh`. Tools: Vivado, Vitis HLS and Vitis **2021.1**,
with the ADI tree `Vendor/MicroPhase_E310_V1/hdl`.

## What changes

```
RX1 ─┐                          ┌─ bypass  (core 0, default: v1.1 identity, unchanged)
     ├─ axi_ad9361 ─ gnss_passthrough ─┼─ PI      (core 1, pi_power_inversion.v, RTL)
RX2 ─┘                          └─ PI-NLMS (core 2, pi_nlms, Vitis HLS)
                                         └─ 12-bit sat → elastic FIFO → TX1 / TX2
```

- **Default is bypass.** A v1.3 bitstream with core 0 behaves bit-for-bit like
  v1.1, so the current loopback keeps working until you select a core.
- **Combined output.** Cores 1 and 2 combine RX1 + RX2 into one nulled stream.
  That stream drives both TX1 and TX2.
- **Clock.** The HLS core runs on `axi_ad9361/l_clk`, which is constrained at
  8 ns (`rx_clk`). It is built for 8 ns with II=2. That fits because in 2R2T
  LVDS mode `adc_valid` is high every second `l_clk`.

| File | Change |
|------|--------|
| `Source/HDL/gnss_passthrough.v` | v1.3: core select, both cores, new registers. Built on your v1.1 file, docs kept |
| `Source/HDL/pi_power_inversion.v`, `pi_cmul.v` | new (PI core from gnss_passthrough_crpa) |
| `Source/HLS/pi_nlms/*` | new: HLS source, C testbench, `run_hls.tcl` (8 ns, II=2) |
| `Source/IP/gnss_passthrough/gnss_passthrough_ip.tcl` | packages the PI files and the HLS RTL too |
| `Source/XDC/gnss_passthrough_cdc.xdc`, `Source/IP/.../gnss_passthrough_constr.xdc` | false paths for the two new parameter synchronisers |
| `Source/Firmware/app_gnss_e310/gnss_passthrough.[ch]` | core / step-size API, expects version 1.3 |
| `Source/Firmware/app_gnss_e310/command.[ch]` | console: `crpa_core?`, `crpa_core=`, `crpa_nlms_mu?`, `crpa_nlms_mu=` |
| `Source/Firmware/app_gnss_e310/main.c` | optional `GNSS_DEPLOY_CRPA_CORE`: power-on core for auto-TX images |
| `Vitis/Scripts/build_boot_2021.tcl` | new: FSBL + BOOT.BIN with XSCT/bootgen 2021.1 |
| `build_crpa_linux.sh` | new: whole Linux build, step by step |

## Registers (base 0x43C0_0000)

| Offset | Register | v1.3 |
|--------|----------|------|
| 0x04 | VERSION | `0x00010003` |
| 0x0C | CONTROL | `[5:4]` core: 0 bypass, 1 PI, 2 PI-NLMS |
| 0x10 | STATUS | `[11:10]` core in use, `[12]` PI-NLMS refused a sample (sticky, must stay 0), `[13]` PI-NLMS configured |
| 0x40 | CRPA_COEF[0] | PI alpha, Q8.8, default 0x100 = 1.0. This now really drives the PI core; in v1.2 it was ignored |
| 0x44 | CRPA_COEF[1] | PI-NLMS `mu_shift_ctrl`, signed, default −3 |

## Apply and build

```bash
LB=~/PI_NLMS_2021/LoopBack_Code_S/LoopBack_Code
tar czf ~/LoopBack_Code_backup_$(date +%Y%m%d).tgz -C "$(dirname "$LB")" LoopBack_Code
cp -r ~/Camera_Vision_Project/PI_NLMS_hls/loopback_patch/. "$LB"/
cd "$LB"
./build_crpa_linux.sh libs hls ip project bit     # hardware: about 30-60 min
./build_crpa_linux.sh sw boot                     # software + Build/BOOT/BOOT.BIN
```

To build an SD image that transmits at power-on, set `DEPLOY_AUTO_TX=1`. Add
`DEPLOY_CORE=2` to start in PI-NLMS mode. Use conducted, attenuated coax only.

## Check

1. `Build/hls.log` should show `HLS_TIMING: ... interval 2` and `HLS_RESULT: PASS`.
2. `Build/build.log` should show `BUILD_TIMING: MET`.
3. RTL simulation against your HLS output:
   `make -C ~/Camera_Vision_Project/PI_NLMS_hls/gnss_integration sim HLS_RTL=$LB/Build/hls/pi_nlms/verilog`
4. On the board console (115200 8N1):
   ```
   gnss_status?          -> version 1.3, nulling core: bypass
   crpa_core=2           -> PI-NLMS
   crpa_core?            -> crpa_core=2, configured=1 refused_samples=0
   gnss_status?          -> underflow count must not grow
   crpa_core=0           -> back to the v1.1 path
   ```
   - **Underflow grows with `configured=1`:** the core is not getting samples,
     which means RX2 (I and Q) is not enabled in `axi_ad9361`.
   - **Hardware for nulling:** you need two antennas, one on RX1 and one on RX2.
