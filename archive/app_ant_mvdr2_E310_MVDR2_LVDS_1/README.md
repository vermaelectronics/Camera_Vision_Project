# app_ant_mvdr2: your MVDR2 firmware on the LVDS ant board

Use with the **Passthrough_MVDR** bitstream (gnss_passthrough.v at 0x43C10000).

## What it is
- **Unchanged MVDR2 modules** (from src_mvdr2_4): gnss_autostart, gnss_smi, gnss_passthrough,
  gnss_report, gnss_dash, gnss_info, gnss_mvdr_auto, gnss_mvdr_steer, gnss_l1, gnss_capture,
  gnss_txdma, gnss_config.h, command.c. So the RX gain auto-level, fast/slow gain and
  clip guard, AOLC -24 dBFS, TX1 34000 mdB, SMI-STAP (16 + 2 image taps, KEXP 8, loading 3,
  covariance averaging beta 1/4), events and reports are the ones that held the satellites.
- **Board layer** (new): `ant_platform.c` (VCXO, EMIO pins, band switches, LVDS init at
  61.44 MSPS), `main.c` (the proven SDT boot of the LVDS app + the MVDR2 main loop),
  the ad9361 / no-OS / xilinx_* drivers of the LVDS app (SPI InputClockHz fix).
- **Operator layer** (new): `gnss_ant.c` = gnss_tx / pass / stap, the main menu, aliases.

## Changes inside MVDR2 files (all small)
| File | Change |
|---|---|
| gnss_passthrough.h | base address fixed to 0x43C10000 |
| command.c / .h | old gnss_tx= body renamed gnss_tx_raw (used by pass=1); old gnss_tx? = gnss_tx_hw_print; `?` shows the new menu; new table entries; cmd_no unsigned |
| gnss_autostart.c | tx_silence() calls gnss_tx_raw(0) (same function as before) |
| gnss_l1.c | band-polarity probe at boot off (GNSS_BOOT_BAND_PROBE 0) |
| gnss_config.h | GNSS_AUTO_MVDR 0 (boot silent; stap=1 runs the automatic start) |
| gnss_info.c | address text 0x43C10000 |
| console.c | ',' or ' ' between parameters, at most 8, bounded buffer |

## Console (v2, 12 Nov 2026)
Only what is needed to run and check the anti-jam. Every page reads ONE snapshot,
so every figure has one source and the same value everywhere.

| Command | What it does |
|---|---|
| Enter / `?` | main menu: state, radio, RX gain + noise + ADC peak, jammer, null |
| `gnss_tx=1/0` | TX1 on (muted) / everything off |
| `pass=1/0` | raw RX1 on TX1 (sets gnss_tx=1, stap=0) / mute |
| `stap=1/0` | anti-jam: RX gain level + output level + SMI-STAP null (sets gnss_tx=1, pass=1) / back to raw RX1 |
| `status?` | one page: radio, RX, noise, jammer, null, TX1, last 5 events |
| `rx?` | RX gain control: gains, AGC state/reason, hunts, ADC peaks, noise vs internal |
| `stap?` | anti-jam: suppression, J/N, jammer type/frequency, RX2/RX1, solver health |
| `selftest?` | checks data path, ADC clock, internal noise, RX1/RX2 match, headroom, null, TX; prints a TO-DO list |
| `band?` / `band=N` | radio presets (3 = all four systems) |
| `tx1_att=mdB` `agc=1/0` `rxgain=dB` | settings |
| `smi=K,A` `smiavg=N` `gnss_kpi?` | advanced / logger |
| `help?` | what every command does |

**SUPPRESSION** = beam max - null of the space-time filter for the jammer's own
spectrum (gnss_smi_null_analysis): P(psi) = w^H D(psi) Rj D(psi)^H w with
Rj = R - lambda_min I and D rotating the RX2 taps; Pnull = P(0), Pmax = max P;
'>' when the jammer left at the output is under 1/10 of the output noise.
Host check (model covariance, real solver): CW 30 dB >40 dB (true 78),
barrage 0.9 fs J/N 30 >43 dB (true 44), J/N 40 with 1.3-sample RX2 delay >52 dB
(true 49), J/N 50 with -40 dBc uncorrelated RX2 jammer >55 dB (true 52).

Engine output (autostart, l1, passthrough, command, report, ...) goes to a
capture buffer (gnss_quiet.c); a FAILED step prints its captured detail.
Live lines: only JAMMER ON/OFF, NULLED, RESIDUAL, ADC CLIP.

## AGC (fast but stable)
- clip -> cut at once (6/10/15 dB), then the fast loop waits 30 ms for a window
  taken after the step (no cascade of cuts on stale clip counts)
- with a jammer the gain rises in ONE step to 1 dB under the ADC limit (1-s
  peak max-hold), at most every 5 s (was 6-dB steps every second): a jammer
  onset costs 2-3 gain steps
- MOD-31 anti-hunt lock (10..80 s) and covariance averaging kept
