# Field test of 30-Oct-2026 by jammer level (running log)

AntSDR with SMI-PI, Venus 8 receiver (GNSS Viewer V2.0.397). Jammer: patch antenna, 50 MHz noise on 1575.4 MHz,
steps 10, 15, 20, 25, 30, 35 dB. Values are read by eye from photographs (about +-2 dB-Hz per bar).
The ten earlier unlabeled photos (`FIELD_TEST_SMI-PI_vs_edge_CRPA.md`, 10:51 to 11:01) will be assigned to
levels as the user tells us; this file collects the labeled ones.

| Level | Photo | Time | TTFF | HDOP | GPS | GLONASS | BeiDou | Galileo | Sats | Mean C/N0 | Scatter |
|---|---|---|---|---|---|---|---|---|---|---|---|
| **10 dB** | `field_photos/level_10dB_104620.jpg` | 10:46:20 | 67 s | 0.70 | 6: 36 33 36 36 25 34 (PRN 2 7 8 9 14 30) | 6: 36 36 38 32 38 39 (69 78 79 81 82 88) | 3: 36 36 37 (24 33 34) | 4: 36 32 32 33 (2 3 8 32) | **19** | **34.8 dB-Hz** | 254/300 points, 2D RMS 3.24 m, CEP50 2.67 m |
| **15 dB** | `field_photos/level_15dB_104948.jpg` | 10:49:48 | 67 s | 0.80 | 6: 30 29 28 31 25 31 (PRN 2 7 8 9 14 30; one weak 20 not used) | 6: 30 34 37 33 31 36 (69 78 79 81 82 88) | 2: 30 31 (24 33; four more bars of 5 to 31 are weak) | **0** tracked (weak bars only) | **14** | **31.1 dB-Hz** | 300/300 points, 2D RMS 0.93 m, CEP50 0.76 m |
| **20 dB** | `field_photos/level_20dB_105149.jpg` | 10:51:49 | 67 s | 0.80 | 6: 32 32 35 31 29 33 (PRN 2 7 8 9 14 30; one weak 22 not used) | 6: 26 29 36 25 28 32 (69 78 79 81 82 88) | 2: 36 24 (24 33; weak bars 25, 27) | 1: 33 (PRN 2) | **15** | **30.7 dB-Hz** | 300/300 points, 2D RMS about 0.9 m |
| **25 dB** | `field_photos/level_25dB_105624.jpg` | 10:56:24 | 2 s (hot restart) | 0.80 | 6: 25 24 27 25 23 25 (PRN 2 7 8 9 14 30; weak 8) | 5: 27 25 31 31 30 (69 78 79 82 88; weak 22) | 2: 25 22 (24 33; weak 24, 26) | 2: 26 27 (PRN 2 32) | **15** | **26.2 dB-Hz** | 300/300 points, 2D RMS 1.05 m, CEP50 0.87 m |

Notes for 10 dB: speed 0.00, position Fix 3D shown, all four constellations tracked, no bar below 25 dB-Hz.
The scatter (2D RMS 3.2 m) is larger than in the later photos (about 1 m): the fix was only about
a minute old (TTFF 67 s from the cold start), not a sign of jammer damage.
The jammer level is as stated by the user ("this is reading at 10 dB"); no `gnss_report?` was seen.

Notes for 15 dB: speed 0.00, HDOP 0.80. **Galileo is gone** (only weak bars of 5 to 31 dB-Hz, none in use)
and BeiDou fell from 3 satellites to 2. GPS and GLONASS are all still there, 3 to 5 dB lower than at 10 dB.
Edge CRPA (rear laptop) in the same photo: too blurred to read the C/N0 (about 8 GPS bars and 2 other bars).

Change 10 dB to 15 dB: satellites 19 to 14, mean C/N0 34.8 to 31.1 dB-Hz (-3.7 dB for +5 dB of jammer).

Notes for 20 dB: speed 0.00, HDOP 0.80. Almost the same as 15 dB (15 satellites against 14, 30.7 against 31.1 dB-Hz): Galileo is
back with one satellite, BeiDou has 2. Edge CRPA (rear laptop) in the same photo: TTFF 19 s, HDOP 0.80, speed 0.72, C/N0 not readable.

**Mapping of the earlier unlabeled photos:** the three photos at 10:51:24, 10:51:47 and 10:51:49 (`05_Field_Test_Photos`,
numbers 1, 2 and 3 of `FIELD_TEST_SMI-PI_vs_edge_CRPA.md`) are the **20 dB** step (same time stamp 10:51:49, TTFF 67, HDOP 0.80).
The edge CRPA readings in photo 2 (10:51:47: TTFF 19, HDOP 1.10, GPS about 21 dB-Hz) therefore belong to 20 dB.
Time order so far: 10 dB 10:46, 15 dB 10:49, 20 dB 10:51; the 10:56 photos (two) and the 10:59 photos (four) and 11:01 (one) are
expected to be 25, 30 and 35 dB; confirm when the levels are sent.

Summary so far (satellites in use, mean C/N0 of the bars): 10 dB 19 / 34.8; 15 dB 14 / 31.1; 20 dB 15 / 30.7; 25 dB 15 / 26.2.

Notes for 25 dB: speed 0.00, HDOP 0.80, position scatter 1.05 m RMS. The satellite count is the same as at 20 dB (15), but all
four constellations are present (GPS 6, GLONASS 5, BeiDou 2, Galileo 2) and the mean C/N0 is 4.5 dB lower than at 20 dB; no bar is above 31 dB-Hz.
The two photos at 10:56:18 and 10:56:24 (numbers 4 and 5 of `FIELD_TEST_SMI-PI_vs_edge_CRPA.md`) are the **25 dB** step: at 10:56:18
28.4 dB-Hz (18 bars), at 10:56:24 26.2 dB-Hz on the solid bars (15 satellites).
