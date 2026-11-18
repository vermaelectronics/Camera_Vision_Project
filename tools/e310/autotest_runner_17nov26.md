# e310_autotest.py — automatic CW / wideband test runner (17 Nov 2026)

Built on e310_gnss_logger.py (same folder). It drives the E310 console, cold-starts the SkyTraq receiver itself (binary restart 0x01, mode 3), times TTFF from GGA, and reads the best-6 C/N0 60 s after each fix. The operator only changes the jammer when prompted.

## Run
    python e310_autotest.py --e310 COM12 --gnss COM4 [--share COM20] [--plan full|baseline|cw|wb] [--runs 3]

## Full plan
1. Baseline, no jammer: bands 3, 4, 6.
2. CW at 1575.42 MHz: 0/10/20/30/40 dB; then the top level at the LO (1582.5 MHz).
3. Wideband 10 dB on bands 3/4/6. The best is picked by score = 100 × fix rate + best-6 C/N0 − TTFF/30.
4. On the best band: gnss_agcclip 30 and 0 against 100. Best picked by the same score.
5. Wideband sweep 0/5/15/20 dB on the best band and clip limit.
6. Optional: --extra "pulse barrage 20% duty,chirp" at 10 and 20 dB.

Each step: settle 60 s, then 3 cold starts (timeout 300 s), with C/N0 averaged over 20 s after a 60 s wait.

## Output
- results.csv: one row per cold start, with the E310 KPI and the DAC clips during the run.
- summary.csv: one row per step.
- report.txt: the table, the C/N0 loss against the baseline, and the recommended band and gnss_agcclip.
- Plus the logger files (console, NMEA, gnss_1s.csv, chart).

Verified end to end on simulated serial ports (fake E310 + fake SkyTraq).
