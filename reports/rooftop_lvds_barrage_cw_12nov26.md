# Rooftop test, LVDS MVDR-STAP: barrage 0 dB and CW (12 Nov 2026, 13:28–14:05)

Logs (e310_gnss_logger):
- `20261112_132821_LVDS_MVDR_STAP_Patch_12Nov26`: barrage, generator 0 dB
- `20261112_135026_..._CWNOISE`: CW
- `20261112_135659_..._CW_20dB_12nov`: CW

## Results (C/N0 = best-4 mean, from gnss_1s.csv; S and J/N from gnss_kpi?)
| Case | J/N in (ADC) | S | J/N out | RX gain | GPS / GAL / BDS C/N0 | Fix / TTFF |
|---|---|---|---|---|---|---|
| Clean (jammer off) | — | — | — | 66–68 | 41 / 39 / 42 | 17 sats |
| Barrage, band 3 (56 MHz), 16 taps | 24–27 | 27–30 | −3 … +1 | 37 → 43 | GPS 27–34, GLO 23–29, GAL/BDS not tracked | no fix in 6 min |
| Barrage, band 3, 5 taps | 25–28 | 18–21 | +5 … +8.5 | 43–48 | — | — |
| Barrage, band 4 (20 MHz), 16 taps | 27–32 | 28–32 | −3.7 … +0.9 | 43–48 | 29–32 / 25–32 / 30–34 | TTFF ≈ 258 s, 8 sats |
| Barrage, band 4, 8 taps | 27–32 | 23–28 | +2.4 … +6.8 | 44–45 | 29 / 26 / 25–26 | fix 57–67 % of the time |
| CW 1575.42 MHz (on L1), 8 or 16 taps | 31–40 | 40–47 | −4 … −9 | 45–53 | 36–38 / 33–35 / 39–41 | TTFF 32 s, 12 sats at 72 s |

## Findings
1. **CW works well.** S is 40–47 dB and the jammer is left 4–9 dB under the noise. Taps make no difference. GPS C/N0 drops 3–4 dB and cold start takes 32 s.
2. **Barrage: the null is limited to S ≈ J/N in + 1..3 dB, so J/N out stays near −1 dB** (about 2.5 dB of C/N0) whatever the J/N.
   - **Depends on taps:** about +5 dB of S per doubling (5 → 19, 8 → 25, 16 → 30).
   - **Not on bandwidth:** 56 and 20 MHz give the same S, so it is not simple dispersion.
   - **Not on the loading:** the theory gives 51–54 dB.
   - **Likely causes:** rooftop multipath of the jammer, or a frequency-dependent RX1/RX2 response.
   - **Variation:** the jammer RX2−RX1 phase moved between 26° and 48° from one reading to the next.
3. **The biggest barrage loss is the RX gain cut:** 66–68 → 37–48 dB. The AD9361 input-referred noise is about 5–7 dB worse at 44 dB gain (MOD-30 table: 0.5 dB per dB of gain cut). With the jammer residual, that adds up to the measured 9–12 dB C/N0 loss.
4. **RX2 is about 6 dB hotter than RX1:** both noise (−28 vs −34 dBFS) and jammer (RX2/RX1 +4..+6 dB). The AGC gain is set by RX2's peak (−4..−6 dBFS) while RX1 (the reference) sits at −10..−12, so about 5 dB of gain is lost to the imbalance. `stap=1` already warned that RX1/RX2 noise differ.
5. **Onsets:** each jammer onset clips the TX output for 3–9 ms (120–220 k samples). This is transient, but it hits every satellite.
6. **Logger artefact:** two `!cold` 3 s apart gave a false "no fix" in ttff.csv. `gnss!` and `gnss_taps?` were sent to the E310 (invalid commands).

## Recommended
1. **Find why RX2 is 6 dB hotter** (antenna/LNA supply, cable, connector). Balanced channels should give about 5 dB more gain under a barrage, which is about 2–3 dB of C/N0.
2. **Keep `taps=16` for a barrage.** 8 taps costs about 5 dB of S and 3–4 dB of C/N0.
3. **Next test:**
   - a barrage power ramp: generator −10, −5, 0, +5 dB, with `!cold` at each;
   - a barrage from a different jammer position (clear line of sight), to check the multipath idea.
4. **Possible firmware options:**
   - AGC peak limit for a barrage closer to full scale, with a clip-rate guard;
   - an RX1/RX2 gain offset (per-channel gain); this may cost null depth.
