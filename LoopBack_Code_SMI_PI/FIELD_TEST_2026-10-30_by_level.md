# Field test of 30-Oct-2026 by jammer level (running log)

AntSDR with SMI-PI, Venus 8 receiver (GNSS Viewer V2.0.397). Jammer: patch antenna, 50 MHz noise on 1575.4 MHz,
steps 10, 15, 20, 25, 30, 35 dB. Values are read by eye from photographs (about +-2 dB-Hz per bar).
The ten earlier unlabeled photos (`FIELD_TEST_SMI-PI_vs_edge_CRPA.md`, 10:51 to 11:01) will be assigned to
levels as the user tells us; this file collects the labeled ones.

| Level | Photo | Time | TTFF | HDOP | GPS | GLONASS | BeiDou | Galileo | Sats | Mean C/N0 | Scatter |
|---|---|---|---|---|---|---|---|---|---|---|---|
| **10 dB** | `field_photos/level_10dB_104620.jpg` | 10:46:20 | 67 s | 0.70 | 6: 36 33 36 36 25 34 (PRN 2 7 8 9 14 30) | 6: 36 36 38 32 38 39 (69 78 79 81 82 88) | 3: 36 36 37 (24 33 34) | 4: 36 32 32 33 (2 3 8 32) | **19** | **34.8 dB-Hz** | 254/300 points, 2D RMS 3.24 m, CEP50 2.67 m |
| **15 dB** | `field_photos/level_15dB_104948.jpg` | 10:49:48 | 67 s | 0.80 | 6: 30 29 28 31 25 31 (PRN 2 7 8 9 14 30; one weak 20 not used) | 6: 30 34 37 33 31 36 (69 78 79 81 82 88) | 2: 30 31 (24 33; four more bars of 5 to 31 are weak) | **0** tracked (weak bars only) | **14** | **31.1 dB-Hz** | 300/300 points, 2D RMS 0.93 m, CEP50 0.76 m |

Notes for 10 dB: speed 0.00, position Fix 3D shown, all four constellations tracked, no bar below 25 dB-Hz.
The scatter (2D RMS 3.2 m) is larger than in the later photos (about 1 m): the fix was only about
a minute old (TTFF 67 s from the cold start), not a sign of jammer damage.
The jammer level is as stated by the user ("this is reading at 10 dB"); no `gnss_report?` was seen.

Notes for 15 dB: speed 0.00, HDOP 0.80. **Galileo is gone** (only weak bars of 5 to 31 dB-Hz, none in use)
and BeiDou fell from 3 satellites to 2. GPS and GLONASS are all still there, 3 to 5 dB lower than at 10 dB.
Edge CRPA (rear laptop) in the same photo: too blurred to read the C/N0 (about 8 GPS bars and 2 other bars).

Change 10 dB to 15 dB: satellites 19 to 14, mean C/N0 34.8 to 31.1 dB-Hz (-3.7 dB for +5 dB of jammer).
