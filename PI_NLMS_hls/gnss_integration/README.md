# gnss_passthrough v1.3 — simulation

The RTL being simulated is in `../loopback_patch/Source/HDL`. The register map
and build steps are in `../loopback_patch/README.md`.

```sh
make sim HLS_RTL=<LoopBack_Code>/Build/hls/pi_nlms/verilog
```

- `sim_pi`: the original v1.2 testbench, run with core_sel=1 (PI).
- `sim_nlms`, at 2 clocks per sample (the 2R2T cadence):
  - bypass (core_sel=0) is bit-for-bit the v1.1 identity path;
  - PI-NLMS (core_sel=2): 20,480 buffered samples match the C model bit-for-bit
    (stimulus from `gen_vectors.cpp`);
  - no FIFO overflow or underflow, and no sample refused by the HLS core;
  - jammer suppression ≥ 25 dB;
  - DAC formatting is correct.
