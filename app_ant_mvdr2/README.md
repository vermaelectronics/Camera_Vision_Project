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

## Console (v3, 12 Nov 2026)
All AD9361 vendor commands and every MVDR2 `gnss_` command are kept. The new
commands are added on top; four MVDR2 names now open the new pages, and the
MVDR2 versions keep working under a new name:
`gnss_status_full?` (was gnss_status?), `gnss_jam_est?` (was gnss_jam?),
`gnss_kpi_full?` (was gnss_kpi?), `commands?` (was help?: lists ALL commands).

| Command | What it does |
|---|---|
| Enter / `?` | main menu: state, radio, RX gain + noise + ADC peak, jammer, null |
| `gnss_tx=1/0` | TX1 on (muted) / everything off |
| `gnss_tx?` | is TX1 transmitting, raw RX1 or MVDR, attenuation (read from hardware) |
| `pass=1/0` | raw RX1 on TX1 (sets gnss_tx=1, stap=0) / mute |
| `stap=1/0` | anti-jam (sets gnss_tx=1, pass=1) / back to raw RX1 |
| `gnss_status?` | RX/TX LO, RF BW, sample rate, FIR, ADC/DAC clock, gain mode, RX1/RX2 gain, TX1/TX2 attenuation (AD9361 read-back) + state |
| `gnss_jam?` | jammer: type/width/RF, RX1/RX2 total, jammer and noise power (dBm after gnss_jpcal=), RX2-RX1 phase, angle, J/N in/out, SUPPRESSION, theory, C/N0 cost |
| `gnss_mvdr?` `gnss_spec?` / `gnss_spec=G` | MVDR2 pages (MVDR state, noise spectrum) |
| `status?` `rx?` `stap?` `selftest?` | one-page status, RX gain control, anti-jam, self-test |
| `band?` / `band=N` | radio presets (3 = all four systems) |
| `tx1_att=mdB` `agc=1/0` `rxgain=dB` | settings |
| `gnss_agcstep=dB,s` | AGC rise with a jammer (default 1 dB every 1 s) |
| `gnss_ajbw=MHz` | wideband jammer for 2 s: narrow the AD9361 RX filter to MHz (0 = off; try 24); back 10 s after it goes |
| `smiavg=N[,S]` | covariance averaging 1/2^N, steady 1/2^S 1 s after a power step (default 2,4) |
| `gnss_agcclip=ppm` | barrage AGC: gain up to this ADC clip rate (default 100 ppm; 0 = old -2 dBFS peak rule) |
| `gnss_rxoff=dB` / `rxoff?` | RX2 gain = RX1 gain - dB when one channel is hotter; `rxoff?` measures and suggests it |
| `taps=N` / `taps?` | STAP taps per antenna in use, 1..16, live (others set to 0) |
| `smi=K,A` `smiavg=N` `gnss_kpi?` | advanced / logger |
| `help?` / `commands?` | main commands / ALL commands |

The new commands print short pages (engine output captured, gnss_quiet.c); the
vendor and MVDR2 commands print their full text as before.

**SUPPRESSION** S = (J/N)in / (J/N)out (standard definition):
(J/N)in at RX1, (J/N)out = w^H (R - sn2 I) w / (sn2 |w|^2),
sn2 = lambda_min / (1 - sqrt(N/K))^2. '>' when the jammer left is under what the
window can measure (3/sqrt(K)). See the project doc suppression_jn_theory_12nov26.md.

## AGC (fast but stable)
- clip -> cut at once (6/10/15 dB), then the fast loop waits 30 ms for a window
  taken after the step (no cascade of cuts on stale clip counts)
- with a jammer the gain rises 1 dB every 1 s (gnss_agcstep=dB,s), only with
  >= 3 dB of ADC room and never closer than 1 dB to the limit. A 1-dB step is
  under the 1.5-dB step that restarts the covariance averaging and does not
  move the MVDR weights, so the null holds while the gain rises
- without a jammer: back to the noise target at up to 10 dB per 0.3 s
- MOD-31 anti-hunt lock (10..80 s) and covariance averaging kept

## Scheduler and solver timing (14 Nov 2026)
- The main loop now runs on time (global timer): SMI solve whenever a window is
  published (no 1 ms delay after a solve), AGC tick every 10 ms, events every
  0.5 s. Before, the AGC tick was counted in loops: with a ~5 ms solve its
  "10 ms" was ~60 ms, so the AGC fast loop, slow loop and rise ran ~6x slower
  than designed.
- gnss_smi.c is built -O2 by a pragma (also in a Debug configuration).
- `stap?` shows the solver timing (read, average, solve, write, figures) and the
  weight-update interval; `gnss_kpi?` adds tsolve=, tupd=, steady=, ajbw=.
- `gnss_status?` shows the AD9361 temperature and the tracking calibrations.
