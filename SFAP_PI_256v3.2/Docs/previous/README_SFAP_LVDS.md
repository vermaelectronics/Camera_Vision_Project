# Passthrough_LVDS + SFAP-PI v6.0 (256 bins) — ANTSDR E310, two-antenna GNSS anti-jam

The same design as the working 64-bin `SFAP_PassLVDS` (AD9361 LVDS, 61.44 MS/s,
Vivado 2023.2, antsdr-fw-patch ADI tree, VCXO, axi_tdd), with the SFAP-PI
filter bank increased from **64 to 256 frequency bins**.

## 1. What changed compared with the 64-bin build

| | 64 bins (v5.0) | **256 bins (v6.0)** |
|---|---|---|
| FFT size = number of bins | 64 | **256** |
| Bin width at 61.44 MS/s | 0.96 MHz | **0.24 MHz** |
| Frame / hop (50 % overlap) | 64 / 32 samples | 256 / 128 samples |
| Filter delay RX→TX (filter in or bypassed) | 190 samples = 3.1 µs | **766 samples = 12.5 µs** |
| Weights | 3 × 64 (registers) | **3 × 256** in a double-buffered block RAM |
| gnss_passthrough | v5.0, 4 KB map | **v6.0, 8 KB map**, weights at 0x1000–0x1BFC (write-only) |
| sfap_pi HLS core | 3.0 (weight buses) | **4.0** (weights through memory ports) |
| ARM capture per solve | 4096 samples (127 frames) | **8192 samples (63 frames)** |
| ARM solves per second (expected) | ~80 | **~35–40** |
| Bin gating (new) | — | bins with J/N < 0 dB keep zero weight |
| Vivado project in the ADI tree | `projects/passthrough_lvds_sfap` | **`projects/passthrough_lvds_sfap256`** (the 64-bin one is not touched) |

**Why 256 bins:** the notch that the null cuts around a narrowband (CW) jammer is
4× narrower. When the satellites share the jammer's null (jammer broadside, or
in the satellites' direction), a CW at 1575.42 MHz removes much less of GPS C/A.

## 2. C-simulation results (all 10 scenarios PASS)

| # | Test | 256 bins |
|---|---|---|
| 1 | Core accuracy vs. double-precision model | 0.42 LSB rms error, latency exactly 766, zero weights = exact x1 |
| 2 | Wideband jammer J/N 30 dB | 44.5 dB suppression, satellite loss −1.3 dB |
| 3 | No jammer | bypass exact; filter in: 0.26 dB loss |
| 4 | CW J/N 40 dB | 64.3 dB suppression, 0.37 dB loss |
| 5 | Wideband 30 dB + reflection | 43.6 dB |
| 6 | Wideband 50 dB + I/Q imbalance | 58.0 dB |
| 7 | RX2 25 dB weaker, wideband 40 dB | 38.9 dB |
| 8 | Wideband 60 dB + I/Q imbalance | 58.9 dB |
| 9 | Two CW jammers, two directions | 65.0 dB |
| 10 | **CW on the satellite's own frequency, satellite in the null (bad geometry)** | 61.9 dB suppression, **satellite loss 8.3 dB (64 bins: 23 dB)** |

## 3. Console commands

Same as the 64-bin build (`sfap-pi=1/0`, `sfap-pi?`, `sfap-pi_aj?`,
`sfap-pi_report?`, `sfap-pi_log=`, `sfap-pi_csv=`, `sfap-pi_mode=`,
`sfap-pi_freeze=`, `gnss_tx=`, `gnss_status?`, `gnss_band=`, `gnss_rx_gain=`,
`gnss_rx_agc=` …). Status lines now start with `SFAP-PI:` (was `GNSS_SMI:`).

`gnss_status?` must show `id/version : 0x47435031 / 6.0` and layout `0x08070c04`.
The firmware refuses to start the anti-jam on a 64-bin bitstream (and vice versa
the old firmware on this bitstream), so BOOT.BIN must come from this package.

## 4. Build: sources → BOOT.BIN (same flow as before)

```bash
cd ~/CRPA_Project/NEW_PASSTHROUGH/NISHANT
mkdir -p sfap256 && cd sfap256
unzip SFAP_PassLVDS_256.zip
cd SFAP_PassLVDS_256/Automation/Linux
chmod +x *.sh

source /tools/Xilinx/Vivado/2023.2/settings64.sh
source /tools/Xilinx/Vitis_HLS/2023.2/settings64.sh
source /tools/Xilinx/Vitis/2023.2/settings64.sh
# ADI_HDL_DIR defaults to ~/CRPA_Project/NEW_PASSTHROUGH/workspace/antsdr-fw-patch/plutosdr-fw/hdl
# (override with export ADI_HDL_DIR=... if yours is elsewhere)

./build_all.sh --only check
./build_all.sh 2>&1 | tee ../../Build/build_console.log
```

Result: `SFAP_PassLVDS_256/Build/boot/BOOT.BIN`. The hdl stage now prints the
worst setup / hold slack (`HDL: timing WNS … ns, WHS … ns`) and stops if either
is negative. The hls stage stops if II = 1 at 10 ns is not met or the weight
memory ports are missing from the generated RTL.

## 5. Firmware 6.1.0: faster re-nulling when the jammer's path changes

Software only (same bitstream): `./build_all.sh --from sw`.

* **Fast re-null.** Each update measures how well the running weights still
  null the jammer on the new capture. If they leave more than 3 dB of
  residual (`AJ_FAST_DB`), the jammer's path has changed (obstacle, people
  moving, new multipath) and that capture is solved on its own instead of
  being averaged with the older statistics. Simulated sudden path change:
  back to a full null after **1 update** (before: 6–7 updates with the jammer
  leaking 15–25 dB meanwhile).
* **Release build (-O2)** of the ARM program instead of Debug (-O0): more
  weight updates per second (`sfap-pi_report?` → `engine: … solves/s`).
  `SFAP_SW_CONFIG=debug ./build_all.sh --from sw` builds the Debug version.
* **Gated bins are not solved** (J/N below the noise floor): much less work
  per update for CW and partial-band jammers.
* Reporting: `sfap-pi_report?` has a `fast re-null` line, the status line
  shows `… solves/s, N fast`, and the CSV (`sfap-pi_csv=`) has a last column
  `fast_per_s`.
* A path that keeps changing (someone walking near the antennas) is limited by
  the update rate: the weights are always one update old. Suppression is about
  −20·log10(2·sin(Δφ/2)) for a phase change Δφ of the jammer's RX2/RX1 path
  between two updates (14° → ~12 dB, 4° → ~23 dB, 1° → ~35 dB).

## 6. Things to watch

* **Not yet through Vivado.** Checked here: the 10 C-simulation scenarios, an
  RTL simulation of the new double-buffered weight RAM through the AXI bus
  (write → commit → read ports; a commit without new weights keeps them), the
  firmware compiled for the Cortex-A9, and every block-design connection
  against the RTL ports.
* **DSP use:** about 160 DSP48 for the filter (the W8 FFT stage is done with
  adders), of 220 on the xc7z020; the hdl stage prints the placed utilisation.
* **Delay 12.5 µs** on TX1 (filter in or bypassed) — a common delay on every
  satellite, absorbed by the receiver's clock bias. `sfap-pi=0` keeps the plain
  passthrough with no filter delay.
* **Fast-changing jammers** (pulsed, fast sweep/chirp): frames are 4× longer and
  the ARM re-solves ~35–40 times per second, so tracking is slower than with 64
  bins. Steady CW / barrage are unaffected.
* **Geometry still matters:** with two antennas a narrowband null also notches
  satellites in the same cone (jammer broadside). 256 bins make that notch
  narrow, but endfire jammer placement is still the best test setup.
