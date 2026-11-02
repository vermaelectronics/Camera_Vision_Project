# Field test of 30-Oct-2026 by jammer level (running log)

AntSDR with SMI-PI, Venus 8 receiver (GNSS Viewer V2.0.397). Jammer: patch antenna, 50 MHz noise on 1575.4 MHz,
steps 10, 15, 20, 25, 30, 35 dB. Values are read by eye from photographs (about +-2 dB-Hz per bar).
The ten earlier unlabeled photos (`FIELD_TEST_SMI-PI_vs_edge_CRPA.md`, 10:51 to 11:01) will be assigned to
levels as the user tells us; this file collects the labeled ones.

| Level | Photo | Time | TTFF | HDOP | GPS | GLONASS | BeiDou | Galileo | Sats | Mean C/N0 | Scatter |
|---|---|---|---|---|---|---|---|---|---|---|---|
| **10 dB** | `field_photos/level_10dB_104620.jpg` | 10:46:20 | 67 s | 0.70 | 6: 36 33 36 36 25 34 (PRN 2 7 8 9 14 30) | 6: 36 36 38 32 38 39 (69 78 79 81 82 88) | 3: 36 36 37 (24 33 34) | 4: 36 32 32 33 (2 3 8 32) | **19** | **34.8 dB-Hz** | 254/300 points, 2D RMS 3.24 m, CEP50 2.67 m |

Notes for 10 dB: speed 0.00, position Fix 3D shown, all four constellations tracked, no bar below 25 dB-Hz.
The scatter (2D RMS 3.2 m) is larger than in the later photos (about 1 m): the fix was only about
a minute old (TTFF 67 s from the cold start), not a sign of jammer damage.
The jammer level is as stated by the user ("this is reading at 10 dB"); no `gnss_report?` was seen.
