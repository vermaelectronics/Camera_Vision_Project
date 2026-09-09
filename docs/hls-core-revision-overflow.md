# HLS `core_revision` integer overflow (Vitis HLS 2021.1)

## Symptom

`make syn` fails during HLS IP packaging (`export_design -format ip_catalog`)
with:

```
ERROR: '2609091256' is an invalid argument. Please specify an integer value.
"rdi::set_property core_revision 2609091256 {component component_1}"
```

The failure happens for any HLS IP (e.g. `v_demosaic`), only during
packaging, after `csynth_design` has already completed successfully.

## Root cause

On every `export_design -format ip_catalog`, Vitis HLS 2021.1 regenerates
`<ip>/prj/impl/ip/run_ippack.tcl` from an internal template baked into the
Vitis HLS *installation itself* (not this repo, not `runhls.tcl`, not the
HLS C++ source). That template stamps the IP's `core_revision` with the
current wall-clock time, formatted `YYMMDDHHMM`:

```tcl
set Revision    "2609091256"      # e.g. 2026-09-09 12:56
...
set_property core_revision $Revision $core
```

This is provably a live timestamp and not a stored project value: two
exports run a few minutes apart produced `2609091256` then `2609091351`.
Grepping `runhls.tcl` and the whole HLS project tree for `Revision` turns up
nothing, because none of this logic lives in project-controlled files.

Vivado's `core_revision` IP property is a bounded 32-bit integer (max
`2147483647`). Any `YYMMDDHHMM` timestamp from **2022-01-01** onward is
already `>= 2200000000`, past that bound — so this has been broken for
every export since then. It's a defect in the Vitis HLS 2021.1
IP-packaging backend that was never patched for that (now EOL) release.

Because the file is regenerated from scratch on every export, hand-editing
`run_ippack.tcl` (or decrementing the value by one) only survives until the
next `make syn` — it is never a persistent fix.

## Fix

**[`scripts/fix_hls_core_revision.sh`](../scripts/fix_hls_core_revision.sh)**
clamps `core_revision` back to a valid value in the generated
`run_ippack.tcl`. It's idempotent (safe to call even if the value is
already valid) and takes the file, not the whole project, so it works no
matter which HLS IP produced it.

### Fastest recovery from a build that already failed on this error

HLS C-synthesis already succeeded (RTL generation + the Fmax report
completed) — there's no need to redo it. Just patch and re-run *only* the
Vivado packaging step:

```bash
./scripts/fix_hls_core_revision.sh \
    design_1_v_demosaic_0_0/prj/impl/ip/run_ippack.tcl --repackage
```

### Permanent fix — wire it into the build

Add a call to the script right after the HLS export step in the `syn`
Makefile target, so it runs unconditionally on every build and no one has
to hand-edit the generated file again, e.g.:

```makefile
syn:
	vitis_hls -f runhls.tcl
	./scripts/fix_hls_core_revision.sh \
	    design_1_v_demosaic_0_0/prj/impl/ip/run_ippack.tcl --repackage
```

The exact hook point depends on the real `syn` target (how it invokes HLS,
whether packaging is a separate recipe line) — adjust to match rather than
copying this verbatim.

### Root-cause option (edits the Xilinx install, not this repo)

The same `clock format ... -format {%y%m%d%H%M}` → `set Revision` template
lives somewhere under the Vitis HLS installation. Find it with:

```bash
grep -RIl 'set Revision' "$XILINX_HLS" 2>/dev/null
```

(`$XILINX_HLS` is typically `/tools/Xilinx/Vitis_HLS/<version>`.) Editing
that file to emit a small integer (e.g. `set Revision 0`) instead of the
timestamp fixes every project built with that install, permanently — but
it's a change to vendor-shipped files, so it's lost on reinstall/upgrade,
and it's not something this repo can carry. The script above is what
should actually run as part of the normal build.
