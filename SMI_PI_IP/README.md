# SMI_PI IP: closed-form power inversion (Vitis HLS)

Stand-alone source of the `smi_pi` anti-jam IP. Builds on its own; nothing
from the E310 project is needed.

| File | What it is |
|---|---|
| `smi_pi.h` | top-level function and port description |
| `smi_pi.cpp` | the IP (synthesisable C++) |
| `testbench.cpp` | C simulation: 4 scenarios, prints `SMI_PI_CSIM: PASS/FAIL` |
| `run_hls.tcl` | csim, synthesis, II/clock check, export to `./ip` |

## Build

```bash
source /tools/Xilinx/Vitis_HLS/2023.2/settings64.sh
cd SMI_PI_IP
vitis_hls -f run_hls.tcl
```

Result: `ip/component.xml` (VLNV `antsdr:gnss:smi_pi:1.0`) and
`ip/smi_pi_csynth.rpt`. Last line `SMI_PI: PASS`.
Measured on the target PC: estimated clock 6.98 ns at an 8 ns target, II 2.

## Algorithm

Power inversion with RX1's weight fixed at 1 (minimise output power):

    R12 = E[x1 conj(x2)]     R22 = E|x2|^2     (exponential average, 2^K samples)
    a   = R12 / (R22 + L)
    y   = x1 - a * x2

Only three 36-bit accumulators carry state from sample to sample; the divider
and complex multiplies are feed-forward pipeline stages, so no multiplier sits
in a feedback loop.

## Ports

| Port | Dir | Width | Meaning |
|---|---|---|---|
| `ap_clk`, `ap_rst_n` | in | 1 | clock (8 ns), active-low reset (clears the averages: a = 0) |
| `s_axis_x` | AXI-Stream in | 64 | `{rx2_q, rx2_i, rx1_q, rx1_i}`, each signed 16-bit, 12-bit ADC values |
| `m_axis_y` | AXI-Stream out | 32 | `{y_q, y_i}`, signed 16-bit, saturated |
| `load` | in | 32 | diagonal loading L in LSB^2 (0 acts as 1; typical 16) |
| `adapt_en` | in | 1 | 1 = averages track, 0 = weight frozen |
| `wt_band` | out | 2 | abs(a): 0 below 0.25, 1 below 1, 2 below 4, 3 otherwise |

Throughput: one sample pair every 2 clocks (II = 2). Latency about 60 clocks.
Averaging time constant: `SMI_PI_K` (default 10, i.e. 1024 samples).

## C-simulation results (CW jammer, BPSK test signal, +-20 LSB noise)

| Scenario | Result |
|---|---|
| jammer 600 LSB | null -52.7 dB within 1000 samples; SINR -6.80 dB (optimum -6.83) |
| no jammer | output 24.97 dB vs RX1 25.00 dB (passes RX1 unchanged) |
| jammer 2000 LSB switched on/off | settles after every switch |
| weak jammer 60 LSB | SINR -7.16 dB (optimum -6.83) |
