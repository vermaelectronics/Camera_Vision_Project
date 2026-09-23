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
`0x00010002` (v1.2). `CRPA_COEF(1..15)` remain reserved RW scratch -- nothing
in the core reads them; there is currently no AXI-visible readback of the
core's internal weights or nulled output (`w_re`/`w_im`/`s_re`/`s_im` are not
wired to any register).

This patch:

- **gnss_passthrough.h / .c** -- bumps `GNSS_PT_EXPECTED_VERSION` to
  `0x00010002`, adds `GNSS_PT_REG_CRPA_ALPHA`, and adds
  `gnss_pt_set_crpa_alpha()` / `gnss_pt_get_crpa_alpha()` (float, real alpha)
  and `_raw()` variants (uint16_t, Q8.8). `gnss_pt_get_state()` /
  `gnss_pt_print_state()` now report the current alpha, with an explicit
  warning when the loaded bitstream reports a pre-1.2 version (alpha writes
  still succeed on those boards -- the register is plain RW storage even
  then -- but nothing consumes them, so no nulling occurs).
- **command.c / command.h** -- adds `gnss_crpa_alpha?` / `gnss_crpa_alpha=`
  console commands (e.g. `gnss_crpa_alpha=1.0`), following the existing
  `gnss_tx=` / `gnss_ddr_tx=` pattern.
- **gnss_info.c** -- updates the register-map, live-health, "why it exists",
  "what has *not* been proven", and RX2-wiring sections that previously
  described the CRPA block as not yet existing. The nulling core is verified
  in RTL simulation only (fixed-point accuracy vs. a floating-point shadow
  model); it has not been run against a real interferer on hardware, and that
  distinction is called out explicitly rather than overclaimed.

## Known gap, called out rather than hidden

There is no AXI-Lite readback of the core's actual nulling weights or output
samples today -- only alpha (the input knob) is observable from software.
Diagnosing convergence at runtime (beyond "did RX/TX counters advance") needs
new RTL register wiring (`w_re`/`w_im`/`s_re`/`s_im`/`weights_valid` onto
spare `CRPA_COEF` slots or new registers) before firmware can add it; that is
out of scope for this patch.
