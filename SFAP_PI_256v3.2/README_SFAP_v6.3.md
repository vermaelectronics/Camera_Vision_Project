# SFAP_PI_256v3.2 — SFAP-PI v6.3: pulse blanking + bin excision (ANTSDR E310, 256 bins)

This is the full project tree (as in `SFAP_PassLVDS_256v2`), carrying the
v6.2.2 firmware from `SFAP_PI_256v3` and two additions on top:

* **Pulse blanker.** It sits in front of the filter bank. A sample of either
  channel with |I| or |Q| at or above a threshold zeroes **both** channels,
  from 16 samples before it to 16 samples after it.
* **Bin excision.** After each solve, the firmware zeroes any bin that still
  holds two jammers from two directions. Two antennas give only one null per
  bin, so one of those jammers would otherwise leak through.

Both act on RX1 and RX2 together, so the spatial null is not disturbed.

**This is a new bitstream, not a firmware-only update.** The HLS core, the
`gnss_passthrough` HDL, the block design and the firmware have all changed.
Rebuild everything with `./build_all.sh`. The firmware refuses to start on an
older bitstream.

| | v6.2.2 (`SFAP_PI_256v3`) | **v6.3 (this package)** |
|---|---|---|
| `gnss_passthrough` VERSION / AJ_INFO | 0x00060000 / 0x08070C04 | **0x00060003 / 0x08070C05** |
| `gnss_passthrough` IP | 2.0 | **3.0** (mask bank, `aj_wm_*` ports) |
| `sfap_pi` HLS IP | 4.0 | **5.0** (blanker, `wm` port) |
| Filter latency RX→TX (filter in and in bypass) | 766 samples | **782 samples** (+16 for the blanker's look-ahead) |
| AJ_CFG | [0] bypass, [10:8] shift | **+ [27:16] blanker threshold (ADC LSB, 0 = off)** |
| Register map | weights 0x1000–0x1BFC | **+ excision mask 0x1C00–0x1C1C** (8 words, same double buffer) |
| Firmware | 6.2.2 | **6.3** |
| C simulation | 18 scenarios | **23 scenarios** (19–23 new) |

## 1. Why: results from the C simulation

The C-sim uses the real HLS core and the real firmware solver. The new
scenarios run each case four ways on the same signal. The metric is satellite
SNR loss at a correlator, against RX1 with no jammer. It counts the coherent
loss from blanked samples and the residual jammer that falls inside the
satellite's spectrum. All GNSS satellites in the band share that spectrum.

| # | Scenario | A: v6.2 | B: + excision | C: + blanking | **D: v6.3** |
|---|---|---|---|---|---|
| 19 | Wideband 40 dB + CW 40 dB on the GNSS band, two directions | 42.4 | 9.3 | 42.4 | **9.3** (10 bins) |
| 20 | Wideband 40 dB + 1.5 MHz narrowband 40 dB, two directions | 50.7 | 14.3 | 50.7 | **14.3** (27 bins) |
| 21 | Wideband 30 dB + pulses 50 dB (5 µs every 100 µs), two directions | 40.2 | 41.7 | −1.0 | **−1.0** (6 % blanked) |
| 22 | Pulses 60 dB alone | 6.4 | 6.4 | 0.3 | **0.3** |
| 23 | No jammer / wideband 40 dB / CW 40 dB (no harm) | −0.04 / −0.37 / −1.02 | same | same | **same**, 0 bins, 0 % blanked |

All values are satellite SNR loss in dB. A small negative value is a little
array gain toward the satellite. It is the same in every column.

**What each part fixes:**

- **Pulses (21, 22).** A strong pulsed jammer dominates the per-bin
  statistics of every capture it is in. The solver then points the single null
  at the pulses and lets the continuous jammer through. The pulses also drive
  the peak-based gain loop down by 20–30 dB.
  - Blanking the captures and the stream removes both problems.
  - The gain loop now ignores blanked samples. In the simulation, ignoring
    them *without* blanking makes things worse (28 dB loss), because the
    pulses then clip. The two changes must go together.
- **Two jammers in the same bins (19, 20).** These now cost the excised bins,
  9–14 dB here, instead of 40–50 dB. The cost is high in these cases because
  the jammer sits on the C/A main lobe.
  - The sine window's leakage decides how many bins one CW needs. A 40 dB CW
    that lands on a bin centre leaks above the noise over about ±15 bins; one
    halfway between bins leaks over about ±2.
- **Single jammers (23, and scenarios 1–18).** Nothing changes. Scenarios 1–18
  reproduce the v6.2.1 C-sim output to within 0.01 dB.
- **Not fixed.** Three jammers, e.g. wideband + CW + pulses only ~7 dB above
  them, are still beyond what two antennas can handle.

The full output is in `test/csim_results_v6.3.txt`.

## 2. What changed, file by file

### HLS core: `Source/HLS/sfap_pi/sfap_pi.cpp`, `.h` (IP 5.0)

- **Blanker.**
  - Both channels pass through a 16-sample register delay.
  - A raw sample with |I| or |Q| ≥ `cfg[27:16]` restarts a 33-sample counter.
    While the counter runs, the delayed samples are zeroed.
  - The bypass path taps the delay before the blanker, so it is never blanked.
- **Mask.**
  - New `wm[8]` ap_memory port with two read ports, like `wb`/`wc1`/`wc2`.
  - After the weights, `Y_k = 0` if bit k is set.
- **Alignment.** The frame clock runs 16 samples behind the input counter.
  Frames therefore start on the same input samples as in v6.0, and
  `SFAP_LAT` = 766 + 16 = 782 in both modes.
- **Cost.** About 800 FF and four 12-bit compares, plus a mux per bin. No DSP48.
- **`build_hls.tcl`.**
  - Runs C-sim scenarios 1–23.
  - Exports IP 5.0.
  - Checks that the `wm_*` ports exist in the generated RTL.

### HDL: `Source/HDL/gnss_passthrough.v` (v6.3, IP 3.0)

- **Mask RAM.** Words 0x700–0x707 (byte addresses 0x1C00–0x1C1C) go to a new
  16×32 LUT RAM, one copy per read port.
  - It sits in the same spare/live bank as the weights and is swapped by the
    same `AJ_COMMIT`.
  - A commit that changes the weights now needs all **776** words; the driver
    writes them.
- **New ports.** `aj_wm_address0/1[2:0]`, `aj_wm_ce0/1`, `aj_wm_q0/1[31:0]`.
- **IDs.** VERSION 0x00060003, AJ_INFO 0x08070C05.
- **AJ_CFG.** Bits [27:16] were already carried in full to `sfap_pi_0/cfg`,
  so the blanker threshold needs no new register.
- **Testbench.** `Source/HDL/sim/tb_gnss_passthrough_v63.v` is an Icarus
  testbench, run with `Source/HDL/sim/run_iverilog.sh`. It checks:
  - the IDs;
  - mask and weights reaching the core only after commit;
  - a pending set not disturbing the live one;
  - an empty commit keeping the bank;
  - a write outside the mask range not arming a swap;
  - the committed cfg.

  It prints `HDL_TB: PASS`; the output is in `test/hdl_tb_results_v6.3.txt`.

### Block design and IP packaging

- `Source/BlockDesign/system_bd.tcl` uses VLNVs `gnss_passthrough:3.0` and
  `sfap_pi:5.0`, and connects `wm` the same way as the weight arrays.
- `gnss_passthrough_ip.tcl` packages version 3.0.

### Firmware: `Source/Firmware/app_sfap_lvds` (6.3)

**`gnss_aj_math.c/.h`**

- `aj_blank_thr()`: 5 × robust σ, where σ = median of |I|,|Q| of both
  channels / 0.6745. Computed from a histogram, with no sort. Clamped to
  12…2047.
- `aj_blank_buf()`: the FPGA's rule, bit for bit. It also returns the number
  of samples over the threshold.
- **Excision decision** in `aj_compute()` when `r->excise = 1`:
  - Per bin, it compares the output power of the new weights on the averaged
    statistics against the bin's expected output noise (the learned per-bin
    floor × (1 + |w|²)).
  - A bin goes in after **2 solves in a row** more than 6 dB above that noise,
    and stays while it is within 3 dB of the limit.
  - At most 32 bins, worst first.
  - The solver's own model of the FPGA output (the out-of-sample residual)
    applies the same mask, so the residual, the make-up gain and the
    "jammer leaking" warning stay correct.
- `AJ_LAT` = 782.

**`gnss_aj.c` (engine)**

- **Blanking every capture.**
  - Each capture is blanked in a copy (`aj_work`) before detection and the
    solve. The DMA buffer itself is never written, so no dirty cache line can
    land on the next capture.
  - That capture's threshold is committed with the next weights.
  - If more than 40 % of the capture would be blanked, the signal is not
    pulsed, and nothing is blanked.
  - `noise_ref` is scaled by the kept fraction.
- **Pulses as a detection reason.**
  - New detection reason 5: **pulses**, meaning ≥ 8 samples over the
    threshold in a capture. Noise alone gives an isolated hit in about 1 % of
    captures.
  - Pulses count as present for **100 ms** after the last capture that held
    them, so a low-rate pulsed source (e.g. a 1 kHz radar, which appears in
    only a few % of the 67 µs captures) still switches the filter in and keeps
    it in.
  - With pulses alone, the filter is in with near-zero weights, so TX1
    carries the blanked RX1.
- **Gain step.** When the gain loop steps the RX gain, the engine rescales the
  blanker threshold by the step at once. It commits only the cfg; the running
  weights and mask stay. Without this, a gain step between two captures would
  blank the continuous signal or let pulses through for up to one capture.
- **Mask commit.** The mask is committed with every weight set.

**`gnss_smi_agc.c`**

- While the filter is in and the blanker is on, snapshot reads at or above
  the blanker threshold are left out of the peak, as long as they are
  ≤ 30 % of the reads. Pulses may then clip; they are blanked anyway.
- Every gain step calls `gnss_aj_gain_stepped()`.

**`gnss_passthrough.c/.h`**

- `gnss_pt_write_aj(re, im, mask, cfg, g2)` writes the 8 mask words with the
  weights. With `re`/`im` NULL it changes only cfg and g2.

**Console, `gnss_info.c`, status lines and CSV**

- New commands: `sfap-pi_blank=0/1` and `sfap-pi_excise=0/1` (both default 1).
  Setting both to 0 gives the v6.2 behaviour on the v6.3 bitstream.
- `sfap-pi_report?` prints the blanker threshold, hits and blanked %, the
  number of captures with pulses, and the excised bins.
- The status line shows `+ PULSES (x % blanked)` and `| N bins excised`.
- The CSV gains two columns: `blank_pct`, `exc_bins`.

## 3. Build (same flow as before)

```bash
cd SFAP_PI_256v3.2/Automation/Linux
chmod +x *.sh
source /tools/Xilinx/Vivado/2023.2/settings64.sh
source /tools/Xilinx/Vitis_HLS/2023.2/settings64.sh
source /tools/Xilinx/Vitis/2023.2/settings64.sh
./build_all.sh --only check
./build_all.sh 2>&1 | tee ../../Build/build_console.log   # hls (23 csim) → ip → hdl → sw → boot
./csim_gcc.sh             # optional: the 23 scenarios with plain g++
../../Source/HDL/sim/run_iverilog.sh   # optional: the register-map testbench
```

- The ADI project name is still `projects/passthrough_lvds_sfap256`, so this
  build replaces the v6.0/v6.2 one there.
- After booting, `gnss_status?` must show `0x00060003` and layout
  `0x08070c05`.

## 4. On the bench

1. Run `sfap-pi_report?` and check that the `pulse blanker` and `excision`
   lines are present.
2. A pulsed source, alone or together with a continuous jammer, should show
   `detected because pulses …`, `+ PULSES` on the status line, and the RX gain
   staying near where the continuous signal puts it.
3. A continuous jammer plus a CW from another direction should show
   `N bins excised` and a residual close to 0 dB, where v6.2 warned
   "jammer leaking".
4. To compare against v6.2 on the same bitstream, set
   `sfap-pi_blank=0 sfap-pi_excise=0`.

## 5. What is verified, what is not

- **C-sim:** all 23 scenarios pass. The build used g++ with the open-source
  Xilinx `ap_int` headers, the real core and the real `gnss_aj_math.c`.
- **HDL:** the register-map, mask-bank and commit testbench passes in Icarus
  Verilog 12.
- **Firmware:** the changed files (`gnss_aj.c`, `gnss_smi_agc.c`,
  `gnss_passthrough.c`, `command.c`, `gnss_info.c`, `gnss_aj_math.c`) pass a
  `-Wall -Wextra` syntax check against stub BSP headers. They add no new
  warnings.
- **Not done:**
  - Vitis HLS synthesis of core 5.0, so II = 1 at 10 ns and the `wm_*` ports
    are unconfirmed. `build_hls.tcl` stops if either fails.
  - Vivado implementation and timing.
  - An ARM toolchain build.
  - Any run on the board.
  - The engine's state machine (pulse detection, the 100 ms hold, the
    gain-step rescale) only runs on the board. It is not exercised by the
    C-sim.

## 6. Limits and things to watch

- **Blanking needs pulses that stand out.** They must be at least ~10 dB
  above everything else at the ADC. Weaker pulses are only partly blanked.
- **Long pulses cost samples.** The coherent loss from blanking is about the
  blanked fraction: 6 % costs 0.27 dB and 21 % costs about 1 dB. Above 40 %
  nothing is blanked, because the "pulses" are then the signal.
- **Excision costs satellite energy.** It removes up to 32 bins (1/8 of the
  band). A CW on the C/A main lobe still costs about 9 dB, against about 42 dB
  without excision. Wider signals (E1, B1C, L5) lose less.
- **The threshold can be one capture old after a gain step.** It is rescaled
  by the exact gain step, but between captures it does not follow a jammer
  whose own level changes.
- **Version check.** The firmware checks VERSION ≥ 0x00060003 and AJ_INFO =
  0x08070C05. An old BOOT.BIN is refused (`sfap-pi: … needs 6.3 / 0x08070C05`).

Earlier READMEs are in `Docs/previous/`. Diffs against the v6.2.2 sources are
in `patches/`.
