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

## Wideband fixes (16 Nov 2026)
- Covariance average is reset on every RX gain change (gnss_smi_gain_changed()
  from set_rx_gain_both). Before, the steady average mixed covariance from
  before and after a gain step, so the null leaked for ~1 s after each step
  (the DAC clipping seen after gain steps in the 14 Nov log).
- Barrage AGC: after any barrage cut (fast cut, -2 dB or -1 dB rate cut) the
  gain may rise only once per 5 s for 60 s (agc_bar_slow). Stops the 10-22 x
  hunting seen with WB 15 dB.
- Console garbling after stap=1 fixed: vprintf/putchar now also go through
  the quiet capture in every engine file.
- rx? and selftest show "gain cut N dB by the jammer = C/N0 cost", the
  dominant wideband loss (dynamic range), next to the residual J/N out.
- New preset band=6 "anti-jam: L1/E1/B1C 10 MHz @61.44": LO 1577.92 MHz,
  RF BW 10 MHz, 61.44 MSPS. Oversampled narrow band: the AD9361 floor stays
  spread over 61 MHz, and the barrage outside 10 MHz is filtered before the
  ADC, so the AGC can keep ~6-8 dB more gain. No GLONASS / B1I in this band.

## 17 Nov 2026 (MOD-33 b)
- SMI skips the covariance windows that straddle an RX gain step (the one being
  accumulated at the step and the next): ~0.6 ms at 61.44 MSPS with the old
  weights kept. Before, the first solve after each AGC step mixed two gains and
  leaked the jammer to the DAC rails (DAC clip counts of 100k-1.6M in the
  16/17 Nov logs). stap? shows "after gain steps N".
- stap?: "skipped" renamed "windows passed over" (normal: a window every
  0.31 ms, a solve every 1.5 ms).

## 17 Nov 2026 (MOD-33c): clean hold -- faster satellite lock with no jammer
- With no jammer, power inversion with 16 time taps keeps re-shaping (whitening)
  the band 667 times a second from a noisy estimate: the satellites see a slowly
  wandering amplitude / phase / group delay and the receiver locks a little slower.
- Now: SMI J/N under -3 dB for 2 s -> TX1 gets the fixed RX1 reference tap (the
  pass=1 signal) and nothing more is written. The solver still runs every window;
  the first window with J/N over 0 dB writes the nulling weights again (~1.5 ms).
- gnss_clean=1 (default) / gnss_clean=0 (old behaviour) / gnss_clean=1,in,out (J/N dB).
- stap? shows "clean HOLDING ..."; gnss_kpi? has clean=0 off / 1 on / 2 holding.
- Host test (gnss_smi.c with simulated windows): hold after 2 s, 0 weight loads
  while holding, 20 dB jammer -> nulling weights on the first window, 30 dB S.

## MOD6 (18 Nov 2026) -- firmware only, same bitstream
Package: app_ant_mvdr2_vitis_MOD6.zip. selftest? / gnss_report? print "firmware MOD6".
1. Pulsed jammers (gnss_pulse=1, default on): 4 pulse starts within 2 s -> pulse mode.
   Only jammer-on windows are averaged and solved; the average does not restart
   when a pulse ends; the null stays loaded between pulses (no leak at each pulse
   edge); clean hold is off; the AGC treats the jammer as present and rises at most
   1 dB per 5 s. Ends 3 s after the last pulse. Short pulses (many per 0.31 ms
   window) look like a steady jammer and need nothing.
2. Moving jammer / platform (gnss_track=1, default on): each new window is checked
   with the current weights; leak 3 dB over usual -> fast averaging for 1 s, 6 dB
   -> solve from that window alone. "Jammer present" also from window power over
   the learned no-jammer power, so a fast-moving jammer (smeared average, low J/N
   reading) cannot switch tracking off.
3. AGC: stable jammer class (about 0.2 s, hysteresis) for the barrage rule, so a CW
   no longer gets the 5-s barrage rise. After a deep cut a CW/narrow jammer rises
   up to 3 dB a step while >= 7 dB under the ADC limit (was 1 dB). The jammer counts
   as present for 3 s after it was last seen. Barrage dead band wider (cut at 6x the
   clip limit, rise under 0.15x): fewer gain steps.
4. Clean hold also ends if the window power rises 3 dB over its value at entry.
5. stap?: jammer type (pulsed / steady / wideband) and tracking (leak, usual, fast,
   restarts). gnss_kpi?: pulse=, wide=, leakfast=. New: gnss_track=, gnss_pulse=.
Host tests (gnss_smi.c on simulated windows): static jammer 0 false triggers; pulsed
(4 on / 16 off windows) 0 weight loads between pulses, first window of the next pulse
already nulled (leak -0.5 dB); fast-moving jammer: 3-6 dB left with tracking vs
13-23 dB without; CW classed narrow, barrage wide; clean hold unchanged.

## MOD7 (18 Nov 2026) -- firmware only, same bitstream
Package: app_ant_mvdr2_vitis_MOD7.zip. selftest? / gnss_report? print "firmware MOD7".
Every setting below is AUTOMATIC by default; the jammer type decides it.
1. Jammer type from ~0.3 s of raw windows (lag correlations + centre frequency):
   CW, MULTI-TONE, WIDEBAND (noise), CHIRP, SAWTOOTH, each also PULSED.
   status?/stap?: "TYPE PULSED SAWTOOTH" etc.; gnss_kpi?: jtype=, sweep=.
2. Per type:
   | type            | averaging                         | AGC rule            | gnss_ajbw (auto) |
   | CW / MULTI-TONE | steady + tracking (moving)        | peak -4 dBFS        | never            |
   | WIDEBAND        | steady + tracking                 | barrage (clip rate) | if it reaches out|
   | CHIRP / SAWTOOTH| whole-sweep (beta 1/64..1/128),   | peak -4 dBFS        | if it reaches out|
   |                 | no restarts, no tracking, no clean|                     |                  |
   | any PULSED      | + MOD6 pulse mode (null kept between pulses, AGC 1 dB / 5 s)  |
   A sweep (18 Nov: suppression 0.5 dB, J/N out +36 dB) is nulled when the average
   holds the whole sweep: then it is one direction at every frequency = a spatial null.
   AGC steps rescale the sweep average instead of restarting it.
3. Tracking fix: the noise reference = antenna part (moves with gain) + the AD9361
   floor (does not; from the auto-start floor measurement). MOD6 lowered all of it
   with the gain: after a deep cut it saw a 10-35 dB "leak" (52k restarts, 65k fast
   on 18 Nov). Host test, 30 dB cut: MOD6 15 dB false leak, 28 restarts/s; MOD7 0 dB, 0.
   Restarts at most 1 per 30 ms.
4. gnss_ajbw=1 AUTO is the default (0 off, 4..56 fixed MHz). Width keeps L1/E1/B1C
   2.5 MHz inside the filter: band 3 -> 20 MHz, band 2 -> 24 MHz (fixed 20 cut L1 on
   band 2). Only for WIDEBAND/CHIRP/SAWTOOTH reaching past the narrowed filter; the
   covariance restarts (rescales, swept) at each change. Back 10 s after the jammer.
5. Self-test / auto-start: no RX1/RX2 mismatch or COHERENT warning while a jammer is
   present or just left (18 Nov: false warnings, the jammer reached RX2 4.5 dB stronger).
Host tests (gnss_smi.c, simulated windows, J/N 30 dB): CW, 3 tones, barrage, triangle
chirp 20 ms, sawtooth 10 ms, pulsed barrage, pulsed CW -> correct type (fast sawtooth
may read CHIRP: same handling); jammer left on a fresh window -0.4..+0.5 dB for all.

## MOD8 (18 Nov 2026) -- firmware only
Package: app_ant_mvdr2_vitis_MOD8.zip. selftest? / gnss_report? print "firmware MOD8".
1. Power-up: after the boot check and the radio plan, stap=1 runs by itself
   (gnss_tx=1, pass=1, RX gain level, output level, SMI-STAP null on TX1).
   GNSS_CFG_BOOT_STAP 0 (main.c) = TX1 silent at power-up as before.
2. stap=0 leaves plain pass running (RX1 on TX1, unmuted, no nulling).
3. Automatic printing: gnss_kpi_full? every 10 s and gnss_report? every 11 s.
   gnss_autoprint=K,R changes it (0 = off), e.g. gnss_autoprint=10,0.
   The text is captured into a 16 KB spool and sent from the main loop at the
   UART's own speed (<= 64 bytes per pass, ~11.5 bytes/ms): a ~6 KB report takes
   ~0.5 s on the wire but never stops the SMI solves or the AGC.

## MOD9 (18 Nov 2026) -- main menu (Enter) rewritten, firmware only
Package: app_ant_mvdr2_vitis_MOD9.zip. Every line fits an 80-column PuTTY window.
Sections: NOW (state, radio, RX, jammer + detected TYPE, null) / JAMMER TYPES
(how each type is detected and its automatic averaging, AGC rule and RX filter,
'*' on the type seen now) / AUTOMATIC FEATURES (clean hold, pulse mode, tracking,
RX filter, printing: state + switch command) / CONTROL / CHECK / SETTINGS / HELP.
