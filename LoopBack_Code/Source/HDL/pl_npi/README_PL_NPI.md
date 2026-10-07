# pi_power_inversion_pl_npi -- PL-NPI CRPA core (standalone)

The PL-NPI (Piecewise-Linear Normalized Power Inversion) adaptive-array
nulling core, packaged on its own: current, fully-fixed state as it exists
in the `gnss_passthrough` v1.6 IP, extracted with its direct dependencies
and its own standalone testbench. No algorithm changes in this package --
this is a snapshot, not a revision.

## Algorithm

Implements Jia, Ni, Luo, Zhang, Mao, "FPGA Implementation of Variable Step
Power Inversion Array for BeiDou Receiver", IEEE Access, vol. 11, 2023,
Section III.B/C, Eq. (11).

PL-NPI starts from the paper's own NPI (Section III.A, Eq. 4-6) -- the same
algorithm `pi_power_inversion_normalized.v` already implements:

```
mu_NPI(n)   = 1 / (x^T(n)x(n) + gamma)
w(n+1)      = w(n) - mu_NPI(n) * x^T(n)w(n) * x(n)
```

PL-NPI's only change is multiplying `mu_NPI(n)` by a per-sample gain
selected from the amplitude of `y_I(n)` -- the in-phase component of the
array output `y(n) = e(n)` **alone**, per Eq. (11):

```
mu_PL-NPI(n) = mu_NPI(n) * 1.20   if |y_I(n)| >  T3
             = mu_NPI(n) * 1.10   if |y_I(n)| >  T2
             = mu_NPI(n) * 1.05   if |y_I(n)| >  T1
             = mu_NPI(n)          otherwise   (|y_I(n)| in [0, T1])
```

**The whole point, per the paper's own words, is avoiding a magnitude.**
"Convergence of the algorithm can be ensured by simply judging the
threshold of the output error signal in the same phase path... without
requiring the modulus of the complex signal." The paper's other candidate,
APE-NPI (Eq. 9-10), needs `|y(n)| = sqrt(y_I^2 + y_Q^2)` and a log/exp-shaped
adaptive parameter every sample -- a sqrt or CORDIC plus a transcendental
function, exactly the resource cost the paper's Section III.C explains
PL-NPI is built to avoid. This core follows that discipline: `y_I(n)` is
`s_re` alone (never `s_im`, never a magnitude of the two), and gain
selection is a priority-encoded comparator cascade against three constants
-- nothing else.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `M` | 2 | antenna elements (this board's RX1/RX2) |
| `DATA_W` | 16 | raw ADC I/Q sample width, signed integer |
| `WEIGHT_W` | 32 | adaptive weight width, signed |
| `WEIGHT_FRAC` | 20 | weight fractional bits |
| `LPF_SHIFT` | 18 | leaky-integrator shift (same role as the other two cores) |
| `Q_FRAC` | 32 | `pi_reciprocal`'s fixed-point precision |
| `ALPHA_GAIN_SHIFT` | 17 | empirical calibration constant -- see `pi_power_inversion_normalized.v`'s header for its derivation; PL-NPI inherits it unchanged |
| `GAMMA_INIT` | 1 | Eq. (4)'s regulariser, `gamma > 0` |
| `GAIN_FRAC` | 16 | fractional bits of the PL gain constants |
| `THRESH1/2/3` | `8/16/32 <<< WEIGHT_FRAC` | Eq. (11)'s thresholds against `\|y_I(n)\|`, in the same fixed-point scale as `s_re`/`s_im` |

**Thresholds are runtime parameters, not the paper's literal 8/16/32 --
same calibration issue as `ALPHA_GAIN_SHIFT`.** The paper's own hardware
used 20-bit signed data words for `x(n)` directly (Section V.A); its `y(n)`
therefore lives at a different absolute scale than this project's 16-bit
raw-ADC-scale samples. The specific numbers 8/16/32 are calibrated to
*their* signal scale, not published as scale-independent constants, and
nothing in the paper gives a formula to rescale them. `THRESH1/2/3` are
therefore parameters here, with defaults picked to be proportionally
reasonable at this project's own established test amplitude (100) and
reported, not asserted, as calibrated -- re-tune before trusting them at a
materially different signal scale.

## Fixed-point format of the gain

`GAIN_FRAC`-bit unsigned fixed point (default 16), constants rounded to
the nearest representable value: `1.00 -> 65536`, `1.05 -> 68813`,
`1.10 -> 72090`, `1.20 -> 78643` (all exact to within 1 LSB of `2^-16`).
Folded into the same multiply/shift chain `pi_power_inversion_normalized`
uses for `alpha_mag`, by widening the final shift by `GAIN_FRAC` bits --
no extra pipeline stage, no extra latency.

## Fix history (already applied in this package)

Two real bugs were found and fixed in this core's development; both are
already fixed in the code here.

1. **`THRESH1/2/3` out-of-range bit-slice (silent `'x'` propagation).**
   First declared as plain `integer` (always exactly 32 bits), then sliced
   as `THRESHn[S_W-1:0]` with `S_W=50` for this project's default
   `M`/`DATA_W`/`WEIGHT_W`. Verilog returns `'x'` for any bit selected
   *outside* a vector's declared width, not zero -- every comparison
   against these thresholds silently became `'x'`, `pl_gain_band` read as
   literal `'x'` every cycle, and the weights never moved at all. Caught
   by this package's own testbench's gain-band-coverage check
   (`band_seen[]` all reading 0), not by inspection. Fixed by declaring
   them explicitly 64 bits wide.

2. **`GAMMA_INIT` out-of-range bit-slice (same bug class, caught by real
   Vivado synthesis, not simulation).** Same declaration mistake, this
   time in the sibling file (`pi_power_inversion_normalized.v`) first --
   Icarus Verilog stayed silent, but Vivado's `synth_design` hit it
   directly: `ERROR: [Synth 8-524] part-select [35:0] out of range of
   prefix 'GAMMA_INIT'`. Fixed proactively here too, before the same
   synthesis error could recur in this file.

3. **Undefined weight-overflow at excessive step size.** The weight update
   previously truncated its full-width leaky-integrator result straight to
   `WEIGHT_W` bits with a plain Verilog slice. Confirmed on real hardware
   (on the sibling standard core, which has a fixed alpha and so hits this
   far more easily): when the true result doesn't fit, that slice silently
   wraps via 2's complement, which can flip a weight's sign and jump it to
   an arbitrary magnitude on the very next sample. Fixed by adding
   `sat_weight()`, which computes the update at the full `ALPHA_PROD_W`
   width and clamps into `WEIGHT_W` bits instead of wrapping -- same
   pattern as `gnss_passthrough.v`'s own `crpa_sat12()`. PL-NPI's
   self-scaling step size makes this failure mode much harder to trigger
   than the standard core's fixed alpha, but the fix is applied here too
   for defense in depth, at no cost to normal-range behavior.

## Files

- `rtl/pi_power_inversion_pl_npi.v` -- the PL-NPI core itself.
- `rtl/pi_reciprocal.v` -- pipelined fixed-point reciprocal (`Q_FRAC+1`-stage
  systolic restoring divider) PL-NPI needs for `mu_NPI(n)`.
- `rtl/pi_cmul.v` -- combinational complex multiplier.
- `rtl/pi_power_inversion.v`, `rtl/pi_power_inversion_normalized.v` --
  **comparison baselines only**, not part of the PL-NPI algorithm. Included
  because the testbench below benchmarks PL-NPI's convergence speed against
  both of them; not needed if you only want PL-NPI's own logic.
- `tb/tb_pi_power_inversion_pl_npi.v` -- standalone testbench: 3-way
  convergence-speed comparison (standard / normalized / PL-NPI) plus a
  gain-band coverage check.

## Build and run (Icarus Verilog)

```sh
iverilog -g2005-sv -o tb_pl.vvp tb/tb_pi_power_inversion_pl_npi.v \
  rtl/pi_power_inversion_pl_npi.v rtl/pi_power_inversion_normalized.v \
  rtl/pi_reciprocal.v rtl/pi_cmul.v rtl/pi_power_inversion.v
vvp tb_pl.vvp
```

If you only want PL-NPI's own logic elsewhere (e.g. instantiated directly
in another design, the way `gnss_passthrough.v` does), only
`pi_power_inversion_pl_npi.v`, `pi_reciprocal.v`, and `pi_cmul.v` are
required -- the other two RTL files and the testbench are for this
package's own verification only.

## Measured results (amp=100 calibration point, just re-run, current code)

```
GAIN BAND COVERAGE (Eq. 11's four segments, over 2000 samples)
  band 0 (x1.00, |y_I|<=T1): 1984 samples
  band 1 (x1.05, T1<|y_I|<=T2): 2 samples
  band 2 (x1.10, T2<|y_I|<=T3): 4 samples
  band 3 (x1.20, |y_I|>T3): 10 samples
PASS: gain cascade is exercised across more than one band (not stuck)

CONVERGENCE RESULT (sample index first entering the test's tolerance band
AND remaining in it through sample 1999; -1 = never settled)
  A: standard      alpha=1 -> k = 28
  C: normalized    gamma=1 -> k = 18
  E: PL-NPI        gamma=1 -> k = 17
PASS: PL-NPI converged and stayed within the test band
```

PL-NPI converges one sample ahead of the plain normalized core, both well
ahead of the standard core's hand-tuned `alpha=1`. Gain-band coverage
matches the paper's own description of the mechanism: the highest gain
(x1.20) fires during the large initial transient (weights still near the
quiescent start, `|y_I|` large), settling almost entirely into the lowest
gain (x1.00, 1984/2000 samples) once converged, with brief excursions into
the middle bands in between -- bigger steps while the error is large,
shrinking back as it settles.

This is one measured data point at this project's own amp=100/M=2 test
point, not a proof the default `THRESH1/2/3` are well-chosen generally --
re-run the testbench before trusting them at a materially different signal
scale (see the calibration note above).

## Where this fits in the larger project

This core is one of two selectable CRPA modes in `gnss_passthrough.v` v1.6
(the other being `pi_power_inversion_normalized.v`; the original
fixed-alpha standard core was removed in v1.6 -- unstable above a small
alpha at real signal levels, confirmed on hardware). Selected via
`CONTROL[4]=1`; its own regulariser (`gamma_in`) is independent of the
normalized core's, driven from a separate register (`CRPA_COEF(2)` in the
full IP) so tuning one mode never silently perturbs the other. See the
`hdl/gnss_passthrough/` project for the full merged IP, its register map,
and integration testbenches -- this package is just the one algorithm,
extracted and documented on its own.
