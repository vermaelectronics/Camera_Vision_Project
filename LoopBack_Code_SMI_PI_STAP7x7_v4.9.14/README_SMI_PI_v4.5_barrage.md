# SMI-PI 4.5.0: barrage (noise) jammer 50 MHz wide on 1575.4 MHz, J/N about 60 dB

Firmware 4.5.0, bitstream / IP **smi_pi 2.3**, AJ_INFO layout 4. Written after the
test of 29-Oct-2026 with firmware 4.4.0 (screenshots: custom band `gnss_band=5`,
LO 1575.42 MHz, 18 MS/s, RF bandwidth 20 MHz, TX attenuation 34 dB, RX gain 30 dB; jammer
patch antenna, 50 MHz noise on 1575.4 MHz, setting "30" = 10 dB below its maximum of 40).
**Everything below the board's own lines is C simulation of the real filter core and the
board's own weight solver.** Vitis HLS synthesis, Vivado and the board were not available here.

## 1. What the board showed

* `jammer OFF after 87.2 s (peak J/N +61.1 dB)`: the jammer was **61 dB above the noise**.
* Every 5 s: `WARNING residual +4.6 ... +4.9 dB: the jammer is not fully nulled`.
* Receiver: one GPS and one Galileo satellite, locked but fluctuating.
* `gnss_status?`: bitstream layout 0x10080c03 (4.4), filter in, make-up shift 1, TX attenuation 34 dB.

## 2. What limits it

**a) The ADC (hardware limit, not fixable in code).** To keep a noise-like jammer off
full scale, the RX gain is cut until the jammer sits at about 64 dB above the ADC's own
quantisation noise (12 bits, crest factor of noise about 4). The receiver's thermal noise is
J/N below the jammer, so once J/N passes about 54 dB the ADC's noise is no longer below it:

| J/N | extra noise from the ADC |
|---|---|
| 50 dB | about 0 dB |
| 56 dB | +0.8 dB |
| 61 dB | +1.8 dB |
| 66 dB | +4.1 dB |
| 71 dB | +7.8 dB |

This is also why the suppression the simulation prints stops at about 58 dB for every J/N:
the jammer's own quantisation noise is what is left.

**b) The level into the receiver (found in the log; fixed in 4.5.0).** With the jammer on, the
RX gain is cut by roughly 30 to 36 dB. The satellites and the noise then reach the TX output
that much weaker, and the GNSS receiver's own noise starts to dominate. Two things restore
the level: a digital make-up gain in the filter core, and the TX attenuation ("level hold").

* 4.4.0 allowed the make-up gain only while the residual was below 3 dB, and only up to
  +24 dB. At the board's residual of +4.8 dB the make-up was **0 dB**.
* The level hold never goes below 30 dB attenuation (the safe-start level). With the TX
  attenuation set to 34 dB (20 dB in the first field test) it can add only 4 dB (0 dB).
* So with the jammer at J/N 61 dB the level into the receiver was about **30 dB below the
  clean level** (cut 34 dB, make-up 0, hold up to 4). Whether that costs satellites depends
  on how far the clean level sits above the receiver's own noise, which I do not know, but at
  this size it would explain a receiver that loses most satellites and keeps one or two weak ones.

I am inferring (b) from the numbers in the log and the code, not from a measured output level.
`gnss_report?` in 4.5.0 prints the level (`output level ... dB re the clean level`) so you can check.

## 3. What 4.5.0 changes

| | 4.4.0 | 4.5.0 |
|---|---|---|
| Make-up gain | 0..24 dB, only while residual < 3 dB | **0..42 dB**, restores the clean output level, never more than 3 dB above it (`aj_makeup_shift`) |
| Filter core | smi_pi 2.2, make-up field 0..4 | **smi_pi 2.3**, 0..7 |
| Warning | every 5 s, always "check RX2 antenna ..." | every 30 s; when RX gain is cut by 25 dB or more and J/N is 54 dB or more it says the ADC limit and prints the expected ADC excess and the output level |
| `gnss_report?` | | new line `output level : x dB re the clean level (RX gain cut, make-up, TX hold)` |
| Old bitstream | | the firmware reads AJ_INFO layout: below 4 it keeps the make-up at 4 and prints a NOTE |

The weight scale of 4.4.0 (finer weights for a CW) is unchanged and still needed.

## 4. Result of the simulation at the field-test settings

Barrage jammer filling the whole 18 MS/s band, I/Q imbalance, satellite at the LO,
RX2 path delay 0.055 sample (3 ns: cable and antenna mismatch; the 9.5 cm spacing alone is 0.3 ns). The make-up
shift is the firmware's own rule on the simulated residual. "Level" = output into the
receiver against the clean level; 4.4.0 with TX attenuation 34 dB (hold +4 dB at most).

| J/N | RX gain cut | suppression | residual | satellite SNR loss | level 4.4.0 | level 4.5.0 |
|---|---|---|---|---|---|---|
| 41 dB | 16 dB | 56 | +0.1 dB | -0.8 dB | -4 dB | -1.2 dB |
| 46 dB | 21 dB | 58 | +0.1 dB | -0.5 dB | -3 dB | -0.1 dB |
| 51 dB | 26 dB | 58 | +0.1 dB | 0.1 dB | -2 dB | +1.2 dB |
| 56 dB | 31 dB | 58 | +0.2 dB | 1.1 dB | -3 dB | +3.0 dB |
| **61 dB** | 36 dB | 58 | **+2.5 dB** (board: +4.8) | **2.9 dB** | **-8 dB** (-32 dB at the board's residual of +4.8) | **-0.2 dB** |
| 66 dB | 41 dB | 58 | +13.9 dB | 5.5 dB | -37 dB | +3.9 dB |
| 71 dB | 46 dB | 58 | (ADC floor) | 9.8 dB | -42 dB | +9.2 dB |

Reading it:
* **Jammer at "20" (about J/N 51 dB or less): the filter works.** Loss about 0.1 to 0.3 dB, output level restored.
* **"25" (about 56 dB): about 1 dB loss.**
* **"30" (61 dB, your test): about 3 dB loss from the ADC floor and the null, plus whatever the
  level restoration was costing.** The residual the board printed (+4.8 dB) is above the
  simulated +2.5 dB; the extra is real-world mismatch and the AD9361's noise figure at low gain.
* **Above about 66 dB the ADC decides** (5 dB and more), whatever the algorithm.

## 5. Limitations (read before the next test)

1. **12-bit ADC.** J/N above 60 dB costs 2 dB and more in C/N0 on every satellite, and nothing in
   the filter can give it back. An attenuator or a band-stop before the board does.
2. **Two antennas, one null.** One jammer direction (plus the paths that look like it) is
   removed. Satellites in that direction go with it. A second jammer, or strong reflections from
   other directions, cannot be nulled; the 16 taps per antenna absorb delay differences only.
3. **Front end.** If the jammer overloads the AD9361 input or an active antenna's LNA, no digital
   filter helps. Check `ADC peak` and `rx gain` in `gnss_report?`.
4. **Level into the receiver.** 4.5.0 holds it near the clean level up to a cut of 42 dB, within
   6 dB (one make-up step); the TX hold adds up to 4 dB more with a TX attenuation of 34 dB, and
   up to 40 dB with the default 70 dB. If the receiver needs a hotter clean level, raise it with
   `tx_att_db=` **before** the jammer test, not during.
5. **Hardware not tested.** All numbers above except the board's own lines are simulation.

## 6. Why not a different power-inversion method

The solver is the closed-form block solution (SMI, Cholesky with diagonal loading) of the
space-time power-inversion problem on 4096-sample captures; the filter has 16 taps on each
antenna plus two image taps. On the same wideband test (J/N 30 dB, scenario 2) the old
one-weight form (what NLMS-type power inversion converges to) reaches **14.5 dB** suppression
and loses 12.9 dB of satellite SNR; the space-time form reaches **43 dB** with no loss.
LMS / NLMS / RLS only reach the same optimum more slowly or less exactly, so they cannot beat it.
What limits the result at J/N 60 dB is the ADC and the number of antennas, which no method changes.

## 7. Changed files

`Source/HLS/smi_pi/smi_pi.cpp/.h` (make-up 0..7, IP 2.3), `Source/Firmware/.../gnss_aj_math.c/.h`
(`aj_makeup_shift`, `aj_adc_excess_db`), `gnss_aj.c` (rule, warning, report line, layout check),
`Source/HDL/gnss_passthrough.v` (`CORE_VERSION` 0x00040005, `AJ_INFO` 0x10080C04),
`Source/BlockDesign/system_bd.tcl`, `Source/HLS/smi_pi/testbench.cpp` (scenarios 11 to 14),
`Tests/run_csim.sh`, docs.
