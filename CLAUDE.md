# Notes for Claude

## antsdr-fw-patch release zips

Name each new design-A (pass-through + CRPA) package with the next number:
`crpa1`, `crpa2`, `crpa3`, ...

- `crpa1` (2026-11-10): design A modified, SMI-PI v4.9.14 6 + 6 taps, LVDS 2R2T,
  band 7, `axi_tdd` removed (timing fix). Commit 960625c.
- `crpa2` (2026-11-10): crpa1 + design A on bare metal (`./build.sh baremetal`,
  `antsdr-fw-patch/baremetal/`): SMI-PI firmware on the design-A FPGA, GPIO 67 /
  86..93, gnss_passthrough 0x43C10000, VCXO 23000, band 7 saved on SD.
- `crpa3` (2026-11-12): crpa2 with SMI-PI 7 + 7 taps (reference delay 4,
  AJ_INFO 0x07040C07) instead of 6 + 6, Linux and bare metal. Commit 7fb1b60.
- `crpa4` (2026-11-12): crpa3 with SMI-PI 8 + 8 taps (reference delay 4,
  AJ_INFO 0x08040C07), Linux and bare metal. Commit 9917160.
- `crpa5` (2026-11-12): crpa4 (8 + 8) + automatic CW notch (smi_pi 2.6, AJ_INFO
  0x08040C08, regs 0x1B0/0x1B4, gnss_notch? / gnss_notch=0|1 / gnss_notch_rf=),
  gnss_rx_agc / gnss_aj_avg saved, manual-gain clipping warning, %lu fix.
  Commit eaa2d1a.
- `crpa6` (2026-11-12): design A with **SFAP-PI** (user's Source.zip, SFAP_PassLVDS
  v5.0) instead of SMI-PI: 128 frequency bins by default, `SFAP_BINS=64` for 64
  (hdl 0004 patch, HLS define, firmware gnss_sfap_cfg.h). 4.9.14 firmware kept
  (band 7 etc.), solver = SFAP v5.0 with the 4.9.14 engine interface. AJ_INFO
  0x80400C09 (64: 0x40200C09), weights from 0x200. hls/smi_pi removed. The user
  asked 128, then "hold for 64 FFT", then 128 again: built switchable.
  Commit 3b9e63e.
- `crpa7` (2026-11-13): back to SMI-PI STAP, **10 + 10 taps** (reference delay 5,
  AJ_INFO 0x0A050C07, 65 DSP48), **no CW notch** (user's choice), SFAP removed
  from the tree. FPGA core = crpa4 smi_pi 2.5 with SMI_NT 10; firmware = crpa5
  without gnss_notch (saved rx_agc / aj_avg, clipping warning kept). Linux and
  bare metal. csim: delay 5 never below 8 + 8 (delay 4 lost 2.5 dB on the
  reflection). Commit dc0836f.
- `crpa8` (2026-11-13): crpa7 (10 + 10, REF 5) + **weight smoothing** in the FPGA
  (smi_pi 2.7, AJ_CFG[19:16] k, time constant 2^(k+7) samples, gnss_aj_ramp=,
  default 10, first set after bypass k = 1), **fine output gain** AJ_GAIN 0x1B0
  (Q2.14, 2 DSP48, 67 total), **ADC clip counters** AJ_CLIP1/2 0x1B8/0x1BC (used by
  gnss_rx_agc=2 and the manual-gain warning, clip_ppm in CSV), weight file 20
  words, AJ_INFO 0x0A050C0A (layout 10), Linux solver in a worker thread
  (GNSS_AJ_THREAD). Eigen solver studied, not included (worse in csim 8).
  Commit 5990f02.
- `crpa9` (2026-11-13): crpa8 + **3 + 3 image taps** (delays R-1, R, R+1; SMI_NIMG /
  AJ_NIMG 3, smi_pi 2.8, AJ_WCX0..3 0x1C0-0x1CC, 79 DSP48) + **FPGA power meters**
  AJ_PWR_IN/OUT/IDX 0x1D0-0x1D8 (res_fpga_db / jn_fpga_db in CSV), AJ_INFO
  0x0A050C0B (layout 11). Studied and left out: 18-bit weights (no gain), NLMS
  (not built). csim 25 (I/Q error across the band, J/N 61): residual 8.6 -> 3.2 dB.
  Commit 342c6d7.
- `crpa10` (2026-11-13): **new design** on crpa9 with **8 + 8 taps** (REF 4) and the
  **hardware NLMS** (item 7): smi_pi 3.0, delayed LMS D = 24, anchored to the SMI
  weights by the smoothing, AJ_NLMS 0x1E0 (m, sh, enable), gnss_aj_nlms=0|1 (default
  off), gnss_aj_nlms_mu= (default 0.031), 132 DSP48 in the core (~166 of 220 in all),
  AJ_INFO 0x08040C0C (layout 12), `SMI_NLMS=0 ./build.sh hls` = core without NLMS.
  csim 26: drifting jammer residual 24.9 -> 1.7 dB; 27: fixed jammer +0.09 dB.
  crpa9 stays the 10 + 10 design without NLMS. Commit d7f3db0.
- `crpa11` (2026-11-13): crpa10 with the **NLMS HLS timing fix** (user's crpa10 hls run:
  estimated clock 63.552 ns, FAIL): error history = 32-slot circular buffer (LUTRAM)
  with DEPENDENCE distance=24 instead of a shift register; NLMS step saturated before
  the weight update; smi_pi 3.1. C simulation bit-exact, unchanged. Not yet
  re-run in Vitis by the user. Commit ca0cd02.
- `crpa12` (2026-11-13): crpa11 **without the second ARM core** (user: "I don't want to
  use the 2nd ARM of the Zynq 7020"): worker thread / GNSS_AJ_THREAD / -lpthread removed,
  gnss_crpa pinned to CPU 0. All crpa8/9/10/11 features kept (user asked to keep the
  crpa9 features). Nothing ever moved between FPGA and ARM. Commit 46c1a18.
- `crpa13` (2026-11-13): **NLMS only, no SMI** (user: crpa12 hls failed, "remove SMI from this
  design, only remain NLMS"; "STOP BARE Metal"). New HLS core `nlms_pi` 1.0 (hls/nlms_pi, replaces
  smi_pi): crpa12 filter 8 + 8 taps, REF 4, 3 + 3 image taps, g2, shift, fine gain; weights adapted
  by delayed NLMS only (D = 64, LUTRAM delay line + error history, starts from 0, cleared on bypass
  or ws change), no weight inputs, no smoothing; weight monitor `wmon` (ap_vld, 1 word per sample)
  read back at 0x400.., 0x190/0x194, 0x1C0..0x1CC. AJ_INFO 0x08040C0D (layout 13). Firmware (Linux
  only): SMI solver removed from gnss_aj_math.c (aj_detect kept), step alpha from |rho1| (1/64 CW,
  1/16 wideband, aj_nlms_alpha), ws rule, residual/suppression from FPGA meters, divergence guard
  (clear + halve step), gnss_aj_nlms_mu= (0 = auto, saved as aj_nlms_step_e6), gnss_aj_nlms=0|1
  (hold), removed gnss_aj_avg/ramp/sweep/mode. baremetal/ removed, `./build.sh baremetal` refuses.
  csim 16 scenarios pass (10 ms): J/N 41 +1.3 dB, 51 +4.2, 61 +12 dB residual; 100 ms: 51 +0.8,
  61 +7.6 (SMI optimum +0.7 / +4.0). Not yet run in Vitis by the user. Commit 6d9aadd.
  The paused SFAP-PI 128 BRAM work is commit 01f279c (WIP, not a package).
- `crpa14` (2026-11-13): crpa13 with **6 + 6 taps** (user: "only use 6x6 taps", then chose
  "Only 6 x 6" and keep the 3 + 3 image taps): NL_NT / AJ_NT 6, NL_REF / AJ_REF 3, nlms_pi 1.1,
  AJ_INFO 0x06030C0D, weights 0x400..0x42C + 6 image regs (AJ_NST 12 in gnss_passthrough), 17
  adaptive weights, ~108 DSP48 in the core (~142 in all). csim 16 pass (scenario 5 limit for 6 + 6:
  reflection out of reach, 22.5 dB vs optimum 23.8): field J/N 41 / 51 / 61 residual +1.6 / +5.5 /
  +14.0 dB at 10 ms. NLMS only, no SMI, Linux only. Not yet run in Vitis by the user. Commit 9d13533.
- 2026-11-13: user asked for three Linux designs with the weights on the ARM: `crpa15` = SFAP-PI
  256 (user's SFAP_PI_256v3 = v6.2.1, bare-metal design B, sfap_pi 4.0, weights on the ARM per bin,
  BRAM weight RAM), `crpa16` = STAP 8 + 8 (3 + 3 image), `crpa17` = STAP 6 + 6 (3 + 3 image); STAP
  weights by "NLMS on ARM" (user's choice), three separate packages. Built in the order 16, 17, 15.
- `crpa16` (2026-11-13): HLS core `stap_pi` 1.0 (hls/stap_pi, replaces nlms_pi): filter only, weights
  on ap_none ports (wts 32 x ST_NST, wcj 192) from passthrough shadow/active registers (crpa12 RTL
  without AJ_NLMS and smoothing, aj_wts 32 x AJ_NWTS, parameter AJ_NWTS_P), AJ_INFO 0x08040C0E
  (layout 14). Firmware: aj_nlms_solve() in gnss_aj_math.c (normalised LMS over each capture, 2
  passes, weights carried capture to capture, ws = finest that fits), step aj_anlms_alpha(J/N):
  0.05 < 35 dB, 0.1 < 50, 0.2; gnss_aj_nlms_mu= / gnss_aj_nlms_pass= (saved as aj_arm_step_e6 /
  aj_arm_pass), gnss_aj_nlms=0|1; GNSS_PT_AJ_INFO_EXPECTED built from AJ_NT/AJ_REF. csim 16 pass
  (20 captures): field J/N 41 / 51 / 61 residual +0.6 / +1.6 / +7.0 dB. ~101 DSP48 in all.
- Next package: `crpa17` (then `crpa15`).

Files per package: `crpaN-full-src.zip.part1`, `crpaN-full-src.zip.part2`,
`crpaN-kit.zip`, `SHA256SUMS-crpaN.txt`.

## Field results (user's tests, keep for comparisons)

Both crpa1 tests below were started with a **cold start** in GNSS Viewer, so
the TTFF values are cold-start times and can be compared directly.
All results below are with a **wideband** jammer (user, 2026-11-12); the
5 / 10 / 15 dB values are the user's jammer power settings.

- 2026-11-11, rooftop, `crpa1` (design A Linux, SMI-PI 6 + 6 taps), at the
  user's "5 dB" setting: **3D fix** in GNSS Viewer (TTFF 99 s, scatter about
  +-2 m, GPS L1 C/A and BeiDou B1I tracked at C/N0 about 30-38 dB-Hz).
  Band set by hand: `gnss_band=7,1568.5,30.72,18,30,50` (LO 1568.5 MHz between
  BeiDou B1I 1561.098 and GPS L1 1575.42, 30.72 MS/s, 18 MHz RF bandwidth,
  TX1 30 dB, RX 50 dB). Status: passthrough ON, smi-pi ON (TX1 = space-time
  filter), ADC/DAC enable 1/1, RX1 RSSI 69.5 dB, underflow 0, overflow count
  large. Log: `[AJ ...] WARNING residual +9.3 dB ... Two antennas null ONE
  direction`. Compare crpa3 (7 + 7) and crpa4 (8 + 8) with the same command.
- 2026-11-11, same place and band command, `crpa1` (6 + 6), jammer at the
  user's "10 dB" setting: **still a 3D fix**, but TTFF 225 s (99 s at 5 dB),
  fewer GPS satellites and lower C/N0 (GPS about 25-29, BeiDou B1I about
  28-33 dB-Hz), scatter still about +-2 m. RX1 RSSI 66.0 dB (69.5 at 5 dB;
  the AD9361 RSSI number falls when the input gets stronger), underflow 0,
  overflow count the same sticky value as in the 5 dB test (83654138), so no
  new overflows between the two tests.
- 2026-11-12, user's report (no photos), same band command, cold starts:
  `crpa3` (7 + 7) at 5 dB (corrected by the user): cold-start fix in **83 s**
  with 6 BeiDou used + 1 GPS seen but not used; it struggled to keep the lock
  even without a cold start. Photo 2026-11-12 05:18 UTC: TTFF 83 s, console
  shows `layout 0x07040c07` (7 + 7, so crpa3; the user labelled it crpa4),
  BeiDou C/N0 37/38/35/39/40/36 dB-Hz (PRN 7, 10, 24, 26, 33, 42), one GPS
  at 27 dB-Hz tracked but not used, 16588 weight commits. The photo at
  05:31 UTC with TTFF 47 s (BeiDou only, C/N0 about 27-39) is the crpa4 run. `crpa4` (8 + 8) at 5 dB: cold fix in
  **47 s**; at 10 dB it kept an existing lock (5 BeiDou + 1 GPS) but after a
  cold start no fix (only 2 BeiDou seen, not locked).
  User's targets: 5 dB faster than before, 10 dB cold fix in 80-90 s, 15 dB
  cold fix within 120 s. Advice given: antenna baseline towards the jammer at
  lambda/2 (9.5 cm), LO 1568.26 with 24 MHz BW (or one constellation, narrow),
  smoother weights (gnss_aj_avg 0.95-0.98) and a gnss_smi_freeze=1 test,
  read J/N and residual from gnss_aj?, log with gnss_aj_csv.
- 2026-11-12, PuTTY log from the user (crpa1 6 + 6, band 7 1568.5/30.72/18,
  RX 50 dB MANUAL): J/N 39.6 -> suppression 26 dB, residual ~9 dB (ADC fine,
  RX1 586510 LSB^2); J/N 43.3 -> residual 16-18 dB; RSSI 63.75 (J/N ~46) ->
  residual 23-26 dB, RX samples at +-1600..2048. **Diagnosis: ADC clipping.**
  A clipping simulation (12-bit, logged powers) gives residual 3.3 / 16.7 /
  25.8 dB for the three levels, matching the log. Manual gain = no gain cut,
  and `ADC peak 0 of 2047` because the peak is only measured with
  gnss_rx_agc=2. Changing gnss_band while the jammer was on re-learned the
  floor with the jammer ("NOT A JAMMER ... background") -> filter bypassed.
  The same log also has two design-B runs (gnss_passthrough 0x44A00000, 7 + 7
  and 5 + 5, band 0 61.44/56 MHz): RX1 5.08e6 LSB^2 (rms above full scale,
  ~36 % clipped), residual 36 dB, SAMPLES DROPPED (STATUS[10]), FIFO ch0 16 vs
  ch1 31, coherence peak at lag -1.
  Advice: gnss_rx_agc=2 or gain cut so RX1 power stays < ~250000 LSB^2,
  never change the band with the jammer on, then 8 + 8 for the residual.

