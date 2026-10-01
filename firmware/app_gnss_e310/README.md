# app_gnss_e310 -- GNSS-CRPA MOD-11/MOD-12 firmware patch

These five files are a **patch**, not a full Vitis workspace checkout. They
apply on top of the `app_gnss_e310` no-OS application source (the Vitis
bare-metal app for the ANTSDR E310 V1 GNSS-CRPA project) to add firmware-side
control for the power-inversion CRPA nulling core(s) in the `gnss_passthrough`
IP (see `hdl/gnss_passthrough/` for the current RTL and its own README/
testbenches) -- MOD-11 added control for the original fixed-alpha core;
MOD-12 (below) replaces that control after v1.6 removed that core in favor
of two self-scaling ones.

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

## v1.6 update: the fixed-alpha core this patch controlled is now GONE

`gnss_passthrough.v` v1.6 removed the standard/traditional core entirely --
its fixed-alpha loop gain has a narrow stable range at this board's real
signal levels, confirmed twice: in simulation (diverges at `alpha=100`, the
paper's own "fast" case) and independently **on real hardware** (GPS
satellites vanished from the GNSS viewer at `alpha=100` with no jammer
present). Two cores remain, both with a self-scaling per-sample step size
instead of a fixed alpha: normalized PI (the new default) and PL-NPI,
selected by `CONTROL[4]` (was `CONTROL[5:4]`, 2 bits, in the intervening
v1.4/v1.5 that added these cores alongside the standard one before it was
removed). `CORE_VERSION` is now `0x00010006`.

This update:

- **gnss_passthrough.h / .c** -- bumps `GNSS_PT_EXPECTED_VERSION` to
  `0x00010006` (v1.6) and extends the version-history comment through
  1.4/1.5/1.6. `GNSS_PT_REG_CRPA_ALPHA` / `gnss_pt_set_crpa_alpha()` /
  `gnss_pt_get_crpa_alpha()` are **kept, not removed** (the register is
  still harmless RW storage; nothing calling them will break), but are now
  documented as unused as of v1.6 -- the core they fed is gone. Adds
  `GNSS_PT_CTRL_CRPA_MODE` (`CONTROL[4]`), `GNSS_PT_REG_CRPA_GAMMA_NORM`
  (`CRPA_COEF(1)`), `GNSS_PT_REG_CRPA_GAMMA_PL` (`CRPA_COEF(2)`, independent
  of `_NORM`), and their `gnss_pt_set/get_crpa_mode()` /
  `gnss_pt_set/get_crpa_gamma_norm()` / `gnss_pt_set/get_crpa_gamma_pl()`
  accessors. Unlike alpha, gamma is a **plain unsigned integer**, not
  Q-format -- it shares the fixed-point alignment of the power sum
  `sum|x_i|^2` inside each core, which has 0 fractional bits because the raw
  ADC samples do. `gnss_pt_get_state()` / `gnss_pt_print_state()` now report
  mode and both gammas alongside the (now-inert) alpha reading.
- **command.c / command.h** -- adds `gnss_crpa_mode?` / `gnss_crpa_mode=`,
  `gnss_crpa_gamma_norm?` / `gnss_crpa_gamma_norm=`, and
  `gnss_crpa_gamma_pl?` / `gnss_crpa_gamma_pl=`, following the same pattern
  `gnss_crpa_alpha?`/`=` established. `gnss_crpa_alpha?`/`=` are kept
  (still write/read the register correctly) but now print an explicit note
  on a v1.6 board that the write succeeded and did nothing, rather than
  silently implying it still controls nulling.
- **gnss_info.c** -- updates every section that named a specific version
  number, described `CRPA_COEF(0)` as the only live slot, or told the reader
  `gnss_crpa_alpha=` controls nulling speed: the boot-time hardware summary,
  "why it exists", the "what has *not* been proven" changelog, the RX2
  wiring caution, the register map, and both the short and long command
  menus (including the very first thing a new user sees, the `?` main
  menu's CONTROL section).

**If your board currently reports v1.4 or v1.5**: it still has the standard
core and `gnss_crpa_alpha=` is genuinely live there -- but `gnss_crpa_mode=`
writes `CONTROL[4]` alone, while a v1.5 board still expects `CONTROL[5:4]`
as a 2-bit field (`00`=standard/`01`=normalized/`10`=PL-NPI/`11`=reserved),
not the single bit this firmware sends. Reflash to v1.6 before relying on
mode/gamma control, or use direct AXI-Lite writes matching that board's own
encoding in the meantime. `gnss_pt_probe()`'s boot-time warning and
`gnss_crpa_alpha?`/`gnss_crpa_alpha=`'s own output both spell this out.

**Firmware syntax-checked, not hardware-tested.** This environment has no
Vitis/Xilinx toolchain, so these changes were verified with
`gcc -fsyntax-only` against a real copy of the vendor no-OS headers
(`ad9361_api.h`, `axi_dac_core.h`, `console.h`, `parameters.h`, etc., pulled
from an earlier delivered bundle) plus minimal stubs for the three
project-specific headers not present in that bundle (`gnss_l1.h`,
`gnss_txdma.h`, `gnss_info.h` -- just their declarations, not full
semantics). All five edited files compiled with zero errors; the only
warnings were a pre-existing intentional `#warning` (the base-address
fallback) and one pre-existing, unrelated `ad9361_spi_read` pointer-type
mismatch from a no-OS driver version difference in the borrowed headers,
in code this update did not touch. That confirms the C is well-formed, not
that it has run against real hardware -- build it in the actual Vitis
workspace and re-run `gnss_pt_probe()` / `gnss_status?` before trusting it
on a board.

## Third bug, found the same way AGAIN: gamma's reset default is silently 0, not 1

Confirmed live on hardware right after the v1.6 update above:
`gnss_status?` printed `crpa gamma : normalized=0  pl-npi=0` -- not the
documented default of `1`. Root cause: `CRPA_COEF(1)`/`CRPA_COEF(2)` (the
AXI-side `crpa_coef[]` array in `gnss_passthrough.v`) reset to `0`, not `1`
-- only the RTL core's *own* `gamma_reg` resets to `GAMMA_INIT=1`. Gamma is
deliberately loaded continuously with no write-strobe (the fix for the
v1.2 `alpha_wr`-hardwired bug class above -- no enable line left to tie off
wrong), so the instant the design leaves reset, `gamma_reg` stops holding
`GAMMA_INIT` and starts tracking `crpa_coef[1]`/`[2]`, which is `0`. The
documented "default 1" was only ever true for the first instant of reset,
never in sustained real operation. Not catastrophic (Eq. 13's denominator
still has `2*sum|x_i|^2`, nonzero for any real signal) but it removes the
intended regularisation floor against near-zero-power segments.
**Workaround, no rebuild needed**: `gnss_crpa_gamma_norm=1` /
`gnss_crpa_gamma_pl=1` once after every boot -- the write path itself is
correct, only the hardware reset value is wrong. A proper fix (firmware
auto-writing gamma=1 right after `gnss_pt_probe()`, or an RTL change to
`crpa_coef`'s own reset value) has been offered but not yet applied as of
this commit.

**Fourth bug, immediately after fixing the third**: the hardware readback
above (fixed correctly) exposed that `gnss_crpa_gamma_norm=1` itself
printed `GNSS_CRPA_GAMMA_NORM: set to u` -- the value missing entirely.
This is the EXACT SAME `%u`/`%lu` bug as the "Second bug" section above,
regressed into the four new gamma get/set functions in `command.c`
(all used `%lu`), plus a second, separate, pre-existing instance in
`gnss_info.c`'s "1 Hardware" screen (`CRPA alpha : raw=%u`) that predates
this session entirely and was never caught because `command.c`'s
`gnss_crpa_alpha?`/`=` were fixed correctly back then, but this separate
info-screen copy of the same read was not. Swept the entire project tree
for every remaining `console_print` call using `%u`/`%lu`; these six
instances were the only ones (confirmed via `grep` across all `.c` files,
not just the ones touched this session). All six now use `%d`, matching
this formatter's actual supported types and the project's own established
convention. Re-verified with `gcc -fsyntax-only`: zero errors, and a
follow-up grep confirms no `%u`/`%lu` remains in any `console_print` call
anywhere in the tree.
