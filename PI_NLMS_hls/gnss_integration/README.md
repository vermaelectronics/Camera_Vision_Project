# gnss_passthrough v1.3 — PI-NLMS (HLS) core inside the CRPA repeater

`rtl/gnss_passthrough.v` keeps the v1.2 ports, register map and RX → buffer → TX
structure, and adds the HLS `pi_nlms` core as a second, run-time selectable
nulling core next to the existing `pi_power_inversion` RTL core.

```
axi_ad9361 RX (ch0, ch1, 12-bit)
   ├── pi_power_inversion (RTL)   ─┐  CONTROL[4] = 0
   └── pi_nlms (HLS, AXIS)        ─┤  CONTROL[4] = 1
                                   └─ sat 12-bit → elastic FIFO → TX mux → axi_ad9361 DAC
```

## Register changes (offsets from the gnss_passthrough base address)

| Offset | Register      | Change in v1.3 |
|--------|---------------|----------------|
| 0x04   | CORE_VERSION  | `0x00010003` |
| 0x0C   | CONTROL       | bit 4 `core_sel`: 0 = PI RTL core, 1 = PI-NLMS HLS core |
| 0x10   | STATUS        | bit 10 `core_sel`, bit 11 `nlms_drop` (sticky, a sample was refused), bit 12 `nlms_cfg_done` |
| 0x44   | CRPA_COEF[1]  | PI-NLMS `mu_shift_ctrl` (signed, bits 15:0). Default −3 |

`mu_shift_ctrl`: −3 converges about 4× faster than 0 at the same final depth.
Values below −4 behave like −4, because the core clamps the total shift at 4.

Example (CRPA repeater, NLMS core, ch1 mirrors ch0):

```sh
devmem <base>+0x44 32 0xFFFFFFFD   # mu_shift_ctrl = -3 (already the default)
devmem <base>+0x0C 32 0x19         # pass_en | ch1_copy | core_sel
devmem <base>+0x10                 # STATUS: expect bit12 (cfg_done) = 1, bit11 = 0
```

## Build in Vivado 2021.1

1. Replace the old `gnss_passthrough.v` in your project with `rtl/gnss_passthrough.v`.
   `pi_power_inversion.v` and `pi_cmul.v` stay the same.
2. Add the HLS-generated Verilog as plain sources. It is created by `csynth_design`:
   `~/PI_NLMS_2021/PI_NLMS_hls/solution1/syn/verilog/*.v`.
   You don't need the packaged IP or a block-design cell. The core is
   instantiated directly inside `gnss_passthrough`.
3. The HLS core runs on `clk` (axi_ad9361 `l_clk`). It was synthesized for
   10 ns (≈103 MHz estimated). If your `l_clk` is faster, re-run HLS with that
   clock period and check timing in Vivado.

## Simulation

```sh
make sim HLS_RTL=~/PI_NLMS_2021/PI_NLMS_hls/solution1/syn/verilog
```

- `sim_pi`: the original v1.2 testbench with the PI core selected. It checks that nothing changed.
- `sim_nlms`: drives 20,480 samples of a two-element 12-bit CW jammer, one sample every
  4 clocks. It checks every buffered output bit-for-bit against the `pi_nlms` C model,
  confirms no FIFO overflow or refused samples, checks jammer suppression
  (≥ 25 dB, measured 30.4 dB), and checks the DAC formatting.
