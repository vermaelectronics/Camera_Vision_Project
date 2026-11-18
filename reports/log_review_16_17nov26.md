# Log review 16–17 Nov 2026 (STAP_16_Nov26Test, STAP_17Nov26_1 + GNSS Viewer)

## Results
| Case | RX gain (clean) | J/N at ADC | S | J/N out | Post-null floor vs antenna noise | Viewer |
| --- | --- | --- | --- | --- | --- | --- |
| No jammer, band 3 | 67 dB | — | — | — | +0.5 dB | TTFF 31 s, C/N0 35–45 |
| WB "10 dB", band 3 (56 MHz) | 44 (67) | 32–35 dB | 33–34 dB | +1.1 dB | about +15 dB | C/N0 17–27, lock comes and goes, no cold start |
| WB, band 4 (20 MHz @ 22.5) | 48–49 (66) | 31–37 dB | 34–37 dB | −3.3..+0.3 dB | about +10–11 dB | not captured |
| CW 31–45 dB J/N | — | — | — | — | — | nulled in 1.5–27 ms |

## Findings
- The null works as the theory says (S ≈ J/N in, J/N out ≈ 0 dB). The loss is the post-null floor.
- The post-null floor is 4–7 dB above what the gain cut alone predicts (−39.6 vs −46.4 dBFS on band 3; −43 vs −47.8 on band 4). This is jammer-derived uncorrelated noise. Suspects: ADC clipping (clip rate 260–480 ppm vs the 100 ppm limit; peaks at −0.3 dBFS, 1.5 dB headroom), AD9361 nonlinearity.
- J/N at the ADC tops out at 31–37 dB for every wideband case: that is the dynamic-range ceiling, not the jammer setting.
- DAC clipping: 389k samples by 664 s (16 Nov) and 1.6M at a CW onset (17 Nov). Cause: the first solve after a gain step mixed two gains. Fixed in MOD-33b (skip straddling windows).
- 16 Nov 1204 s: CW onset cut the gain 68 → 36, then rose 1 dB per 5 s (barrage slow rule applied to a CW). Check the jbw_class at CW onset.
- 17 Nov: the USB COM port dropped when the wideband jammer came on (also "COM12 removed/arrived" in the viewer). RF pickup on the USB link.
- TX1 attenuation set to 20 dB / 25 dB: the output noise is then about −37 / −42 dBm; above −60 dBm unless an external pad is used.

## Next tests
1. Band 4 vs band 6 vs band 3 under the same WB jammer: C/N0 in the viewer, cold-start TTFF.
2. gnss_agcclip=100 vs 30 vs 0: does the post-null floor drop and C/N0 rise?
3. With MOD-33b, check that the DAC clip counter stops rising during steady jamming.
