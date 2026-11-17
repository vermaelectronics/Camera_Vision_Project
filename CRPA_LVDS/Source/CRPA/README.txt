CRPA anti-jam algorithm only (no Linux, DMA, console or build pipeline), exactly as in each package.

Each folder:
  arm/gnss_aj_math.c, arm/gnss_aj_math.h  - the weight solver that runs on the ARM
  fpga/<core>.cpp, fpga/<core>.h          - the FPGA space-time filter (Vitis HLS C++)

crpa1  (commit 960625c): SMI-PI 6 + 6. arm = SMI solver (averaged statistics, pulse gate,
       sweep detector, aj_compute); fpga = smi_pi core.
crpa27 (d0df4f5) 8 + 8, crpa28 (f098977) 10 + 10, crpa29 (f4b7b91) 14 + 14,
crpa30 (1095c71) 16 + 16, crpa31 (77032e2) 18 + 18: STAP_NLMS_PI with the moving-jammer fix.
       arm = aj_direct_solve (Wiener + Cholesky, start), aj_nlms_solve (NLMS per capture),
       aj_track_* (re-solve when the null is lost), aj_pred_* (weight prediction), aj_detect;
       fpga = stap_pi core (filter only, weights from the ARM).
       crpa27-31 are the same code: only AJ_NT / AJ_REF (gnss_aj_math.h) and ST_NT / ST_REF
       (stap_pi.h) differ; crpa30/31 stap_pi.h also set AP_INT_MAX_W 2048.
