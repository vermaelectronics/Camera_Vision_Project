# Field test (second sheet): CW jammer ON the GPS L1 frequency, 1575.4 MHz

Source: `Untitled_spreadsheet_-_Google_Sheets.pdf` (uploaded by the user, "also remember
this"). It is a partial copy of the same CRPA field-test log as
`FIELD_TEST_2026-10-28_CW_partial_jamming.md` (jammer section 2 and test 4 only).
Date, board, firmware, band profile, receiver model and jammer power are **not** on
this sheet. Hardware measurement by the user.

## Entered values

| Item | Value |
|---|---|
| Jammer | CW, centre **1575.4 MHz** (the GPS L1 / Galileo E1 centre is 1575.42), "Tx power: Up-to 40 dB" (not a single level; unit unclear), distance 6.5 m |
| Interference suppression, RMS | **10 dB** (as typed) |
| Weights, angles, antenna gain, J/S, fix type, HDOP | not entered |

## Result (jammer on, nulling active)

| Start | TTFF | Tracked / used | Avg C/N0 | Max C/N0 |
|---|---|---|---|---|
| Normal | **116 s** | 4 / 4, **GPS only** | 35 | 37 |
| Hot | 4 s | 4 / 4, GPS only | 35 | 37 |
| Cold | **116 s** | 4 / 4, GPS only | 35 | 37 |

GLONASS, BeiDou and Galileo: 0 satellites in all three cases. The sheet's "Total avg
C/N0 = 8.8" is the mean of four constellation rows including three zeros (35 / 4 =
8.75); the real value is 35 dB-Hz.

## Comparison with the first report (same sheet family)

* CW at **1582.5 MHz** (off the signals, partial jamming): 28 satellites with the jammer
  off, 13 at 15 dBm, 8 at 20 dBm, 4 at 25 dBm; GPS, GLONASS, BeiDou and Galileo all
  present at the lower levels.
* CW at **1575.4 MHz** (on L1): only 4 GPS satellites at 35 dB-Hz, no other system, normal
  and cold start 116 s, hot start 4 s. Galileo E1 and BeiDou B1C share the L1 frequency
  and would be lost with the CW on top of them; GLONASS and BeiDou B1I are not at that
  frequency, so their absence points to the band profile used (an L1-only profile such as
  `gnss_band=0` would not carry them) or to the receiver gain being pulled down by the
  jammer. The profile and `gnss_report?` were not recorded: unknown.

## Still missing to analyse it

`gnss_report?` and `radio?` during the test, band profile, firmware / zip version, jammer
level in dBm at the jammer, what the 10 dB suppression was measured with.
