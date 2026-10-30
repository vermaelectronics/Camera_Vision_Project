# CRPA Anti-Jamming Board — Field Test Notes (28-Oct-2026)

Source PDFs (Google Sheets exports) are in `raw/`:
- `raw/2026-10-28_CRPA_Field_Test_Log_1.pdf` — full field test report (baseline + 4 jammer tests)
- `raw/CRPA_Jammer_1575.4MHz_Untitled_Sheet.pdf` — separate jammer-on run at 1575.4 MHz

## Setup

| Item | Value |
|---|---|
| Date / tester | 28-Oct-2026, Sandeep |
| Location | 28.5118865, 77.4201639 — clear sky |
| CRPA type | Digital, AntSDR board, 2 elements |
| Element spacing | 9.52 cm |
| Firmware / tools | Vitis IDE 2023.2 |
| Bandwidth / sample rate | 56 MHz / 61.44 MS/s |
| Jammer | CW, 1582.5 MHz (Log 1); 1575.4 MHz (second sheet) |
| Jammer distance | 6.5 m; Tx power up to 40 dBm |

Not recorded: board S/N, GNSS receiver model, DOA method, jammer azimuth/elevation, antenna gain, J/S, position error, HDOP, AJ weights/phase.

## Log 1 — 1582.5 MHz CW jammer

TTFF in seconds. Sats = tracked / used in fix. C/N₀ in dB-Hz (avg across constellations as reported).

| Condition | Normal TTFF | Hot TTFF | Cold TTFF | Sats (normal) | GPS | GLO | BDS | GAL | Avg C/N₀ | Max C/N₀ |
|---|---|---|---|---|---|---|---|---|---|---|
| Jammer OFF, pass-through | 39 | 2 | 53 | 16 / 15 | 7/6 | 4/4 | 0/0 | 5/5 | 36.26 | 39 |
| Jammer OFF, AJ enabled | 74 | <1 | 36 | 28 / 28 | 9/9 | 5/5 | 8/8 | 6/6 | 37.2 | 43 |
| Test 1 — 15 dBm, AJ on | 55 | <1 | 55 | 13 / 12 | 6/6 | 4/3 | 1/1 | 2/2 | 34.5 | 38 |
| Test 2 — 20 dBm, AJ on | 178 | <1 | 152 | 8 / 8 | 6/6 | 1/1 | 0/0 | 1/1 | 23.3* | 36 |
| Test 4 — 25 dBm, AJ on | 189 | — | — | 4 / 4 | 4/4 | 0/0 | 0/0 | 0/0 | 7.4* (GPS 29.75) | 31 |
| Test 3 — 30 dBm, AJ on | **No lock** | — | — | 3 / 0 | 3/0 | — | — | — | 29.8 (GPS only) | 31 |

\* Sheet's "total avg C/N₀" averages in zero-valued constellations, so it understates the real per-satellite C/N₀ (e.g. Test 4 GPS avg is 29.75, not 7.4). Same issue in the 3.1.2/3.1.3 baseline rows (27.6).

Hot/cold start detail (sats tracked/used, avg C/N₀):
- Pass-through hot: 15/15 (GPS 37.33, GLO 38, GAL 35); cold: 15/15 (GPS 37.33, GLO 37.5, GAL 35.6)
- AJ-on, jammer off hot: 26/26 (GPS 37.38, GLO 39, BDS 39.86, GAL 35.83); cold: 27/27 (GPS 37.37, GLO 38.4, BDS 39, GAL 36.33)
- Test 1 hot: 14/14 (avg 33.9); cold: identical to normal start (likely copied — re-check)
- Test 2 hot: 10/10 (avg 23.9*); cold: 11/11 (avg 23.6*)

## Second sheet — 1575.4 MHz CW jammer (L1 centre), AJ on

- Interference suppression (RMS before vs after nulling): **10 dB**
- Normal start TTFF 116 s, hot 4 s, cold 116 s
- Only GPS: 4 tracked / 4 used, avg C/N₀ 35, max 37 (GLONASS/BeiDou/Galileo: 0)
- Jammer Tx power for this run not recorded.

## Key takeaways

1. With jammer off, enabling the AJ path *improved* reception: 16→28 sats, BeiDou 0→8, max C/N₀ 39→43 dB-Hz.
2. Lock is maintained up to ~25 dBm Tx at 6.5 m, but satellite count and TTFF degrade steeply: 28 → 13 (15 dBm) → 8 (20 dBm) → 4 (25 dBm); no lock at 30 dBm.
3. BeiDou B1I (1561 MHz) and Galileo/GLONASS are lost first; GPS L1 C/A survives longest.
4. Only one suppression figure was measured (10 dB RMS, 1575.4 MHz run). Suppression, J/S, weights and position error are blank for Log 1 tests.

## Data gaps / to do next time

- Fill RMS/peak suppression and complex weights for every jammer test.
- Record jammer azimuth/elevation, Tx antenna gain, and compute J/S.
- Record position error and HDOP; GNSS receiver model and board S/N.
- Hot/cold starts for Tests 3 and 4 not done; Test 1 cold-start values duplicate normal start.
- Fix the sheet's total avg C/N₀ formula to ignore constellations with 0 satellites.
- "Tx power: Up-to 40 dB" should read dBm.
