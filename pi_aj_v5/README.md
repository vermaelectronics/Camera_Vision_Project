# PI-AJ v5 — 2-element GNSS anti-jam core (power inversion) for AntSDR / AD9361

PI-AJ v5 is Verilog-2001 RTL for a **2-antenna anti-jam front end at 1575.42 MHz**. It is
built for a CW ("sharp") jammer at 1575.4 MHz and for barrage/noise jamming. The algorithm
is still **power inversion**: RX1 is the reference element with its weight fixed to 1, and
every other weight adapts to minimise the total output power. Two things are added around it:
- **Adaptive CW notches** in front of the power inversion. They remove a sharp jammer in
  time/frequency, so the single spatial degree of freedom of a 2-element array is not
  spent on it.
- **Widely-linear space-time weights** (5 taps per channel, plus conjugate taps). These
  null what a plain 2-element SMI-PI cannot: the jammer's I/Q-imbalance image, the
  RX1/RX2 filter mismatch across the band, and jammer multipath.

The weights adapt **every sample in the FPGA** (normalised LMS). Software does no matrix
inversion and no block weight commits.

```
 RX1 ─┐  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌─────────────────────┐  ┌─────┐
      ├─►│ frontend │─►│ CW notch │─►│  image   │─►│ CW notch │─►│ widely-linear       │─►│ TX  │─► TX1
 RX2 ─┘  │ scale,   │  │   #1     │  │  notch   │  │   #2     │  │ space-time POWER    │  │ AGC │
         │ clip det │  │ self-    │  │ conj(#1) │  │ (opt.)   │  │ INVERSION (2ch×5tap │  └─────┘
         └──────────┘  │ tuned    │  └──────────┘  └──────────┘  │ +conj, 19 weights,  │
                       └──────────┘                              │ NLMS every sample)  │
   same filter applied to RX1 and RX2 ──────────────────────────►└─────────────────────┘
```

---

## 1. What your test data says, and why 4.x stopped at about the 30 dB setting

| Observation (your sheets/terminal) | Meaning |
|---|---|
| `peak J/N +61.1 dB` | With the jammer near full scale on a 12-bit ADC, the thermal noise sits at only ~1 LSB. **The jammer already uses the whole ADC dynamic range.** Each extra dB of jammer either clips the ADC or forces the RX gain down, which pushes the satellites under the ADC quantisation noise. |
| `WARNING residual +4.7 dB … not fully nulled` | A 2-element SMI-PI has **one** spatial null. Each AD9361 channel has its own I/Q imbalance, so a CW at −20 kHz also produces an image at +20 kHz (~−34 dBc). The image has a *different* spatial signature, so one null cannot remove both. Jammer multipath adds to this. A residual of a few dB is the expected symptom. |
| 1575.4 MHz sheet: RMS suppression 10 dB, only 4 GPS, TTFF 116 s | A sharp CW in the middle of the C/A main lobe. The spatial null alone was not deep enough. |
| GLONASS / BeiDou lost first | With LO 1575.42 and 18 MS/s, the processed band is 1566–1584 MHz. **GLONASS L1 (1598–1606) and BeiDou B1I (1561) are outside it.** They reach the receiver only through the unprotected path, so jamming removes them first. This is band-plan, not algorithm. See §6. |

## 2. Results (bit-accurate RTL simulation, Icarus Verilog)

Each scenario was produced by `sim/gen_stimulus.py`, which models:
- the 2-element array (d = 9.52 cm = 0.5 λ);
- a satellite at −20° and the jammer at +55°, plus a ground-reflected copy (−10 dB, −35°);
- RX1/RX2 gain, phase, filter and delay mismatch;
- per-channel I/Q imbalance and DC offset;
- jammer phase noise;
- a 12-bit ADC with the RX gain set so the jammer just fits (as you must set it in the field).

C/N0 is measured from the **RTL output** with a real correlator (1 ms coherent blocks). The
jammer-free reference is ≈49.7 dB-Hz. *Read the change in C/N0, not the absolute value.*

| Scenario (RTL output, AntSDR configuration) | sample rate | make-up shift | **TX1 C/N0 (dB-Hz)** |
|---|---|---|---|
| no jammer (reference) | 30.72 MS/s | 0 | 48.5 |
| CW 1575.40 MHz, J/N 61 dB (~30 dB setting) | 30.72 MS/s | 1 | 46.3 |
| CW 1575.40 MHz, J/N 71 dB (~20 dB setting) | 30.72 MS/s | 3 | 42.0 |
| CW 1575.40 MHz, J/N 76 dB (~15 dB setting) | 30.72 MS/s | 4 | 38.8 |
| CW exactly 1575.42 MHz (DC), J/N 71 dB | 30.72 MS/s | 3 | 39.3 |
| CW 1582.5 MHz (+7.08 MHz), J/N 61 dB | 30.72 MS/s | 1 | 49.9 |
| barrage noise 18 MHz, J/N 40 dB | 30.72 MS/s | 0 | 50.8 |
| barrage noise 18 MHz, J/N 50 dB | 30.72 MS/s | 1 | 47.7 |
| barrage noise 2 MHz, J/N 50 dB | 30.72 MS/s | 1 | 36.2 |
| CW 1575.40 MHz, J/N 71 dB (~20 dB setting) | 18.00 MS/s | 3 | 42.0 |

This was simulated with:
- `rtl/pi_aj_axis.v` at NL = 10, DELAY = 4, without notch 2, on a 100 MHz clock;
- samples arriving through a FIFO at the stated sample rate;
- registers set as the AntSDR firmware sets them;
- the 12-bit TX1 output measured with the correlator.

RMS suppression is 61.6 dB at J/N 61 and 68.6 dB at J/N 71 (`doc/rtl_regression_results.md`).
Every row reports notch #1 frequency exactly (−20.11 kHz for the 1575.40 MHz jammer,
0 Hz at 1575.42, +7.081 MHz at 1582.5). No ADC overload, PI late or overrun flags were set.

How the jammer settings map to J/N: your terminal showed J/N ≈ +61 dB at the 30 dB setting.
Every 10 dB less attenuation adds 10 dB of J/N.

**How to read this for your field receiver.** Your satellites run at ~36–39 dB-Hz average and
42–44 dB-Hz maximum. Subtract the C/N0 loss from the table:

| Jammer setting | Expected outcome |
|---|---|
| **30 dB** (J/N 61) | Loss ≈ 2 dB. GPS and Galileo in the processed band track normally. |
| **20 dB** (J/N 71) | Loss ≈ 6.5 dB. **GPS tracks** (strong satellites ~34–36 dB-Hz). Hot/warm start works. Cold-start acquisition of weak satellites is marginal. This is the target asked for, and it is the practical limit of this hardware. |
| **15 dB** (J/N 76) | Loss ≈ 11 dB. Only the strongest satellites may hold lock. Not reliable. |
| **≤ 10 dB … full power** | **Not achievable by any digital algorithm on a 12-bit AD9361** (see §5). |
| **Barrage** | J/N 40: no loss. J/N 50 over the full band: ≈2 dB loss. **J/N 50 concentrated in 2 MHz around L1:** ≈12 dB loss, which is the hardest case for 2 elements. |

## 3. Files

| Path | Content |
|---|---|
| `rtl/pi_aj_top.v` | Top level: ports for axi_ad9361 style RX/TX samples + AXI4-Lite |
| `rtl/pi_aj_frontend.v` | 12-bit → internal scale, ADC clip detector, optional blanking, optional DC blocker |
| `rtl/pi_aj_notch.v` | Adaptive complex notch: self-tuned (lag-1 autocorrelation), self-gated, shared by both channels |
| `rtl/pi_aj_wlpi.v` | **Widely-linear space-time power inversion**, time-multiplexed NLMS engine |
| `rtl/pi_aj_outagc.v` | TX1 level control (6 dB steps, ≤1 step/ms) |
| `rtl/pi_aj_axil_regs.v` | AXI4-Lite register file (map below) |
| `rtl/pi_aj_cdc_bus.v` | Clock-domain crossing for config/status |
| `tb/tb_pi_aj.v` | Testbench (stimulus file in, PI/DAC samples out, register readback) |
| `sim/gen_stimulus.py` | AD9361 + array + jammer scenario generator |
| `sim/model.py` | Floating-point reference model, used for tuning |
| `sim/analyze.py` | Correlator C/N0 and suppression measurement |
| `sim/run_rtl.py` | Runs all scenarios on the RTL and prints the table above |
| `sw/pi_aj.c/.h` | Vitis bare-metal driver: init, tuning, `pi_aj_print()` status report |
| `vivado/add_pi_aj.tcl` | Adds the sources to a Vivado project |
| `rtl/pi_aj_axis.v` | AXI-Stream wrapper: drop-in for the SMI-PI HLS core `smi_pi_0` |
| `tb/tb_pi_aj_axis.v` | Wrapper testbench: fractional sample rate through a FIFO, firmware register settings |
| `sim/run_axis.py` | Runs the scenarios through the wrapper (the AntSDR board configuration) |

## 4. Integration (AntSDR / ADI `axi_ad9361` design)

**Ready-made for your LoopBack_Code project:**
- `deliverables/LoopBack_Code_PI_AJ_v5.0.0.zip` (hardware + firmware + build scripts);
- `deliverables/PI_AJ_v5.0.0_firmware_only.zip`;
- `deliverables/PI_AJ_v5_standalone_rtl.zip` (this folder).

There, `rtl/pi_aj_axis.v` replaces `smi_pi_0` in the block design: same AXI-Stream
ports and async FIFOs, on `sys_cpu_clk` with `NL=10` and `DELAY=4`, registers at
0x43C1_0000. `gnss_passthrough` is unchanged. See `README_PI_AJ_v5.md` and
`BUILD_STEPS.md` inside that zip. `tb/tb_pi_aj_axis.v` and `sim/run_axis.py` verify
that exact configuration.

For a different design:

1. Put `pi_aj_top` where your current `gnss_passthrough` / SMI-PI core sits:
   - inputs `adc_valid, adc_i0/q0/i1/q1` come from the axi_ad9361 ADC channels (RX1, RX2);
   - outputs `dac_valid, dac_i0/q0` go to the DAC channel 0 data path (TX1).
   
   Your existing core's port list is not in this repository. Map the names to your block
   design. Keep TX1 in 2R2T/1T mode as today.
2. **Clock.** `clk` is the clock the sample strobes are synchronous to. With the default
   `NL=5` the core needs **clk ≥ 5 × fs** (90 MHz at 18 MS/s). If your `l_clk` is only 4 × fs,
   build with `NL=10` (needs 3 × fs; costs about 40 more DSP48). STATUS bits `late`/`overrun`
   report a violation. `s_axi_aclk` may be any clock; the CDC is built in.
3. **Data format.** `IN_SHIFT=4` assumes 12-bit, LSB-aligned ADC words (axi_ad9361 default).
   TX words are 16-bit full scale (the DAC uses the 12 MSBs). In bypass, TX1 = RX1 (`CONTROL[0]=0`).
4. **Software.** Call `pi_aj_init(BASE)` once. Map a terminal command (e.g. `aj_status?`) to
   `pi_aj_print(BASE, 18e6)`. It prints suppression, jammer frequency, clip count and the
   RX2 weight (magnitude and phase), which are exactly the fields of your test sheet.

### RX gain — the most important field setting
The algorithm cannot recover anything the ADC has clipped.
- Set `gnss_rx_gain` (manual gain) so that `CLIPCNT = 0` and the jammer peak is about
  1500–1800 LSB (−2 to −3 dBFS).
- **Every time you raise the jammer by X dB, lower the RX gain by X dB.** For the 20 dB
  setting that is about 10 dB below what you used at 30 dB (e.g. 30 → 20 dB).
- `STATUS[3]` (overload) or the `OVERLOAD` line in `pi_aj_print()` tells you the gain is
  too high.

## 5. Limitations

1. **ADC dynamic range is the hard wall.** At J/N ≈ 61 dB the jammer already fills the
   12-bit ADC. Beyond that, the thermal noise (and the satellites in it) drop below ADC
   quantisation. Every dB of J/N above ~65 dB costs about 1 dB of C/N0, *after a perfect
   null*. The 20 dB setting (~71 dB) is the practical limit. The 15 dB setting is marginal.
   The full 40 dB jammer at 6.5 m can only be handled by changing the RF side:
   - more distance or shielding between jammer and array;
   - a higher dynamic-range receiver (14/16-bit);
   - analog/RF pre-nulling ahead of the LNA.
2. **One spatial degree of freedom.** Two elements give one spatial null. CW jammers do not
   use it, because the notches handle them: up to 2 CW lines (notch #1 + notch #2) + 1
   wideband jammer. **Two wideband jammers from different directions cannot both be
   nulled.**
3. **Null cone.** A 2-element null is a cone around the array axis. Satellites near the
   jammer's cone lose 10–20 dB. Some satellites are always lost in a given geometry.
4. **Wideband null depth ≈ 30–35 dB.** Channel mismatch, multipath and the LMS convergence
   of the weak eigen-modes set this limit. Barrage J/N above ~55 dB across the band, or
   above ~45 dB concentrated in the C/A main lobe, will degrade tracking. QRD-RLS weights
   would go deeper but cost ~3× the DSPs.
5. **Swept or chirp jammers.** The notch frequency tracker settles in ~0.2–1 ms. A fast
   sweep is handled only by the power-inversion stage, as a wideband jammer.
6. **Pulsed jammers.** Enable blanking (`CONTROL[7]`). It zeroes clipped samples plus
   `blank_hold` samples.
7. **Processed band.** 18 MS/s at LO 1575.42 covers only GPS L1 C/A and Galileo E1. See §6.
8. **Notch cost.** When a CW notch is on, it removes ~90 kHz of the C/A main lobe (default `rho_sh=6`; ≈0.4 dB of GPS signal). `rho_sh=8` (22 kHz) costs less but leaves more of a noisy jammer's phase-noise skirt.
9. **Simulation only.** These numbers come from simulation. A real AD9361 adds harmonic/IM
   spurs, LO leakage and gain-table effects. Expect the field numbers to be a few dB worse;
   the *relative* improvement should hold. Pre-silicon synthesis was checked with yosys
   (see §7). Run Vivado synthesis + timing on your full design before field use.

## 6. Protecting GLONASS / BeiDou too
To cover GPS + Galileo + GLONASS + BeiDou B1 in one band (1559–1606 MHz):
- set LO ≈ 1582 MHz, fs = 61.44 MS/s and analog BW ≈ 50 MHz;
- build with `NL=10` (3 clocks/sample → clk ≥ 185 MHz);
- set `rho_sh` to 8 to keep the notch at about the same width in Hz (~90 kHz).

A clock of 185 MHz is aggressive on a -1 Zynq-7020. Treat it as a separate step after the
L1 test.

## 7. Resources (yosys `synth_xilinx`, xc7, defaults)

| Resource | `pi_aj_axis` (AntSDR: NL=10, no notch 2) | Zynq-7020 |
|---|---|---|
| DSP48E1 | 134 | 220 |
| LUT | ≈23.7k | 53.2k |
| FF | ≈11.4k | 106.4k |

All multipliers fit one DSP48 (25×18) each. The long paths are pipelined for
100 MHz:
- log-depth leading-one detectors;
- a split weight update with forwarding;
- a two-stage lane adder.

Build parameters (`pi_aj_top`):

| Parameter | Default | Meaning |
|---|---|---|
| `NL` | 5 | PI lanes. 5 needs clk ≥ 5·fs; 10 needs clk ≥ 3·fs with `DELAY=4` |
| `L` | 5 | Taps per channel |
| `DELAY` | 3 | PI update delay; must satisfy DELAY × clocks-per-sample ≥ K + 9 |
| `WL` | 1 | Widely-linear (conjugate) taps |
| `HAS_NOTCH2` | 1 | Second CW notch present |
| `IN_SHIFT` | 4 | ADC word alignment |

These are yosys estimates; expect Vivado to differ. They fit a Zynq-7020 (220 DSP48,
53k LUT) next to the ADI reference design.

## 8. Register map (AXI4-Lite)
See the header of `rtl/pi_aj_axil_regs.v`. Main entries:
- `0x08 CONTROL` (default `0x43B`: core, notch #1, image notch, PI, TX AGC, auto-μ)
- `0x0C ADAPT` (default `0x0008C600`)
- `0x24/0x30 P_RX1/P_OUT` → RMS suppression
- `0x34 NOTCH1_A` → jammer frequency
- `0x40/0x44` → weights

## 9. Field test procedure (fills your CRPA sheet)
1. Jammer off, `CONTROL=0` (bypass): baseline TTFF/C/N0 (section 3.1).
2. `CONTROL=0x43B`, jammer off: AJ-enabled baseline (section 3.2). Expect no C/N0 loss.
3. Jammer CW at 1575.4, 30 dB setting. Set the RX gain as in §4. Record `aj_status?`:
   - suppression → "RMS suppression";
   - RX2 weight → "Phase / Weight magnitude / Complex weight".
4. Repeat at 25, 20 and 15 dB, **lowering the RX gain by the same step each time**.
5. Barrage: as step 3, with the noise jammer.

## 10. Re-running the verification
```bash
cd pi_aj_v5/sim && python3 run_rtl.py        # needs iverilog + numpy; ~15 min on 4 cores
```
