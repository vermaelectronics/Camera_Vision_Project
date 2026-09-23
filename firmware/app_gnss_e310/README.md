# app_gnss_e310 -- GNSS-CRPA MOD-11 firmware patch

These five files are a **patch**, not a full Vitis workspace checkout. They
apply on top of the `app_gnss_e310` no-OS application source (the Vitis
bare-metal app for the ANTSDR E310 V1 GNSS-CRPA project) to add firmware-side
control for the power-inversion CRPA nulling core that MOD-11 adds to the
`gnss_passthrough` IP (see `gnss_passthrough.v`, `pi_power_inversion.v`,
`pi_cmul.v`, verified by `tb_gnss_passthrough.v`; all four checks pass).

The rest of the application (ad9361 driver, no-OS shims, gnss_l1/gnss_capture/
gnss_txdma, main.c, etc.) is unchanged vendor/project source and is not
duplicated here -- drop these five files into the existing Vitis workspace,
overwriting the originals.

## What changed and why

The `gnss_passthrough` core's AXI-Lite register map already reserved
`CRPA_COEF(0..15)` at `0x40-0x7C` for a future CRPA algorithm (see the
`gnss_info.c` register-map menu, pre-MOD-11: "Stored but unused today"). The
RTL side of MOD-11 wires `CRPA_COEF(0)` to the new two-element power-inversion
nulling core's `alpha_in` (the adaptation step size, Q8.8 fixed point,
default 256 == 1.0) and bumps `CORE_VERSION` from `0x00010001` (v1.1) to
`0x00010003` (v1.3 -- see below for why not v1.2). `CRPA_COEF(1..15)` remain
reserved RW scratch -- nothing in the core reads them; there is currently no
AXI-visible readback of the core's internal weights or nulled output
(`w_re`/`w_im`/`s_re`/`s_im` are not wired to any register).

**v1.2 -> v1.3: a real bug found on real hardware, not a cosmetic bump.**
The first MOD-11 bitstream (v1.2, `0x00010002`) instantiated
`u_crpa_core` with `.alpha_wr(1'b0)` -- hardwired low. Inside
`pi_power_inversion.v`, `alpha_reg` (the register the adaptation math
actually reads) only loads `alpha_in` when `alpha_wr` pulses; tied to
`1'b0`, it never does, so `alpha_reg` stays at its reset value (`ALPHA_INIT`,
1.0) forever. `CRPA_COEF(0)` writes and reads both worked correctly --
software saw a fully functional control register -- but the value never
reached the algorithm: the core nulled, always at a fixed alpha=1.0, no
matter what `gnss_crpa_alpha=` sent. This was invisible to
`tb_gnss_passthrough.v` because its one alpha write (`256`) is numerically
identical to the reset default, so "before" and "after" the write were
indistinguishable. Found by tracing real console output from a live board
(`crpa alpha : raw=0 (0.0000)` after boot, with no way to change nulling
behavior) back through the RTL, then confirmed with a standalone simulation
probe before touching anything. v1.3 fixes it: `.alpha_wr(1'b1)` (correct,
since `alpha_in` is already a stable, CDC-synchronized value with no
strobe/handshake needed), and `tb_gnss_passthrough.v` gained a targeted
regression check -- write a **non-default** alpha (512, not 256) and assert
`u_crpa_core.alpha_reg` actually changed -- that fails against the old RTL
and passes against the fix, so this bug class can't hide behind a passing
testbench again.

**If your board currently reports v1.2**: it nulls, but `gnss_crpa_alpha=`
will not do anything until it is reflashed with the v1.3 bitstream
(`hdl/gnss_passthrough/`, rebuilt in Vivado). The firmware's boot-time
`gnss_pt_probe()` and `gnss_status?` both say so explicitly rather than
silently reporting the write as having worked.

This patch:

- **gnss_passthrough.h / .c** -- bumps `GNSS_PT_EXPECTED_VERSION` to
  `0x00010003` (v1.3), adds `GNSS_PT_REG_CRPA_ALPHA`, and adds
  `gnss_pt_set_crpa_alpha()` / `gnss_pt_get_crpa_alpha()` (float, real alpha)
  and `_raw()` variants (uint16_t, Q8.8). `gnss_pt_get_state()` /
  `gnss_pt_print_state()` now report the current alpha, with explicit
  version-specific warnings distinguishing pre-1.2 (no CRPA core at all),
  1.2 (nulls, alpha fixed at 1.0, `gnss_crpa_alpha=` a no-op), and 1.3+
  (alpha genuinely live).
- **command.c / command.h** -- adds `gnss_crpa_alpha?` / `gnss_crpa_alpha=`
  console commands (e.g. `gnss_crpa_alpha=1.0`), following the existing
  `gnss_tx=` / `gnss_ddr_tx=` pattern.
- **gnss_info.c** -- updates the register-map, live-health, "why it exists",
  "what has *not* been proven", and RX2-wiring sections that previously
  described the CRPA block as not yet existing. The nulling core is verified
  in RTL simulation only (fixed-point accuracy vs. a floating-point shadow
  model); it has not been run against a real interferer on hardware, and that
  distinction is called out explicitly rather than overclaimed.

## Second bug, found the same way: `%u`/`%lu` print nothing in this console

Confirmed live on hardware after the alpha_wr fix above: `gnss_crpa_alpha=1.0`
printed `GNSS_CRPA_ALPHA: set to 1.0000 (raw=)` -- the raw value missing
entirely. `console.c`'s `console_print()` is not real `printf`; it's a
hand-rolled formatter whose `switch` only implements `%c`/`%s`/`%d`/`%x`/`%f`.
There is no `%u` case, so it silently consumes nothing and prints nothing;
`%lu` is worse -- the unhandled `l` falls through, and the following `u` gets
emitted as a literal character, garbling whatever text follows it too. Fixed
by switching to `%d` (which this formatter reads as a `long`), matching every
other command in `command.c` (e.g. `get_gnss_tx`'s `(long)pass_en` pattern) --
this file's own convention was the correct one from the start.

## Known gap, called out rather than hidden

There is no AXI-Lite readback of the core's actual nulling weights or output
samples today -- only alpha (the input knob) is observable from software.
Diagnosing convergence at runtime (beyond "did RX/TX counters advance") needs
new RTL register wiring (`w_re`/`w_im`/`s_re`/`s_im`/`weights_valid` onto
spare `CRPA_COEF` slots or new registers) before firmware can add it; that is
out of scope for this patch.
