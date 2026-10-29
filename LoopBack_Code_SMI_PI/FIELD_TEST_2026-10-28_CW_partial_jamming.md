# Field test 28-Oct-2026: CW "partial jamming" (CRPA Field Test Log, Sandeep)

Source: `CRPA_Field_Test_Log_1_-_Google_Sheets.pdf` (uploaded by the user, tested by
Sandeep, 28/10/2026). The user calls this the **report of partial jamming**. Hardware
measurements by the user; nothing here is simulated.

## Setup as entered in the sheet

| Item | Value |
|---|---|
| Place / weather | 28.5118865 / 77.4201639, clear sky |
| CRPA | digital, AntSDR, 2 elements, spacing 9.52 cm |
| RF setup | bandwidth 56 MHz, 61.44 MS/s (this is `gnss_band=1`) |
| Firmware field | "Vitis IDE 2023.2" (the firmware / zip version was **not** recorded; the Downloads folder shows `SMI_PI_v4.0.0.zip` saved on 28 Oct 16:42 and 16:48, so it may have been 4.0.0; to be confirmed) |
| Jammer | **CW at 1582.5 MHz**, 6.5 m from the board; test levels 15, 20, 25, 30 dBm (field "Tx power" says "Up-to 40 dB"; unit unclear) |
| Not recorded | receiver model, board S/N, DOA method, jammer angles, antenna gain, J/S, suppression, weights, position error, fix type / HDOP, observations |

1582.5 MHz is also the LO of profile 1, so the CW sits at 0 Hz in the sampled
baseband (on the DC / LO-leakage bin), between GPS L1 (1575.42) and GLONASS G1 (1598+).

## Results (satellites tracked / used in fix, average C/N0 as entered, max C/N0)

| Case | Normal start TTFF | Tracked / used | Avg C/N0 | Max | Hot start TTFF | Hot tracked / used | Cold start TTFF | Cold tracked / used |
|---|---|---|---|---|---|---|---|---|
| Jammer OFF, pass-through (AJ bypassed) | 39 s | 16 / 15 | 36.26 | 39 | 2 s | 15 / 15 | 53 s | 15 / 15 |
| Jammer OFF, AJ enabled | 74 s | 28 / 28 | 37.2 | 43 | within 1 s | 26 / 26 | 36 s | 27 / 27 |
| Test 1, 15 dBm, AJ on | 55 s | 13 / 12 | 34.5 | 38 | within 1 s | 14 / 14 | 55 s | 13 / 12 |
| Test 2, 20 dBm, AJ on | 178 s | 8 / 8 | 23.3 | 36 | within 1 s | 10 / 10 | 152 s | 11 / 11 |
| Test 3, 30 dBm, AJ on | **no lock** | 3 / 0 | 29.8 | 31 | not done | | not done | |
| Test 4, 25 dBm, AJ on | 189 s | 4 / 4 | 7.4 (see note) | 31 | not done | | not done | |

Per constellation, normal start (tracked / used, avg C/N0):

| Case | GPS | GLONASS | BeiDou B1I | Galileo |
|---|---|---|---|---|
| OFF, pass-through | 7/6, 36.83 | 4/4, 36.75 | 0/0 | 5/5, 35.2 |
| OFF, AJ | 9/9, 36.88 | 5/5, 38.0 | 8/8, 38.87 | 6/6, 35.17 |
| 15 dBm | 6/6, 36.0 | 4/3, 33.33 | 1/1, 33.0 | 2/2, 35.5 |
| 20 dBm | 6/6, 32.17 | 1/1, 29.0 | 0/0 | 1/1, 32.0 |
| 30 dBm | 3/0, 29.75 | - | - | - |
| 25 dBm | 4/4, 29.75 | 0 | 0 | 0 |

Satellites tracked, change against "jammer OFF, AJ enabled" (28): -15, -20, -25, -24
for 15, 20, 30, 25 dBm.

## Notes for reading it

* The sheet's "Total avg C/N0" is the mean of the four constellation averages
  **including constellations with 0 satellites**, so it is biased low whenever a system
  has none. Test 4 (25 dBm): 7.4 in the sheet, real value 29.75 (GPS only). Test 2:
  23.3 in the sheet, satellite-weighted about 31.8. Hot start pass-through 27.6, real about
  36.5. The deltas -13.9 (Test 2) and -29.8 (Test 4) in section 5 are affected.
* Pass-through shows BeiDou 0 satellites, AJ enabled shows 8; also normal start TTFF was
  39 s (pass-through) against 74 s (AJ). Not explained yet (different state of the
  receiver / ephemeris, FIR or profile in the bypass path were not recorded).
* With the jammer on, satellites fall steadily with power: 28 -> 13 -> 8 -> 4 -> 3.
  BeiDou B1I (-21 MHz from the LO) and GLONASS suffer most (BeiDou 8 -> 1 -> 0).
* Hot start keeps working under jamming (within 1 s); cold start does not within 35 s:
  55 s at 15 dBm, 152 s at 20 dBm; normal start 178 s at 20 dBm; nothing at 30 dBm.

## What would be needed to turn this into a result

Firmware / zip version, which algorithm (SMI-PI or PL-NPI), `gnss_report?` and `radio?`
output during each test, J/S estimate and jammer unit (dBm at the jammer or dB?), receiver
model, the pass-through and AJ baselines repeated with the same receiver state.
