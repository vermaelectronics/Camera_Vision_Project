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
- Next package: `crpa5`.

Files per package: `crpaN-full-src.zip.part1`, `crpaN-full-src.zip.part2`,
`crpaN-kit.zip`, `SHA256SUMS-crpaN.txt`.

## Field results (user's tests, keep for comparisons)

Both crpa1 tests below were started with a **cold start** in GNSS Viewer, so
the TTFF values are cold-start times and can be compared directly.

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
  `crpa3` (7 + 7) at 5 dB: fix in 86 s with 5 BeiDou + 1 GPS, struggled to
  keep the lock even without a cold start. `crpa4` (8 + 8) at 5 dB: cold fix in
  **47 s**; at 10 dB it kept an existing lock (5 BeiDou + 1 GPS) but after a
  cold start no fix (only 2 BeiDou seen, not locked).
  User's targets: 5 dB faster than before, 10 dB cold fix in 80-90 s, 15 dB
  cold fix within 120 s. Advice given: antenna baseline towards the jammer at
  lambda/2 (9.5 cm), LO 1568.26 with 24 MHz BW (or one constellation, narrow),
  smoother weights (gnss_aj_avg 0.95-0.98) and a gnss_smi_freeze=1 test,
  read J/N and residual from gnss_aj?, log with gnss_aj_csv.

