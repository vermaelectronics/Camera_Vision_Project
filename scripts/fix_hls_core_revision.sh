#!/usr/bin/env bash
#
# fix_hls_core_revision.sh
#
# Works around a Vitis HLS 2021.1 IP-packaging defect: on every
# `export_design -format ip_catalog`, HLS regenerates
# <ip>/prj/impl/ip/run_ippack.tcl from an internal template baked into the
# Vitis HLS installation itself. That template stamps the IP's
# `core_revision` property with the current wall-clock time, formatted as
# YYMMDDHHMM, e.g.:
#
#     set Revision    "2609091256"
#     ...
#     set_property core_revision $Revision $core
#
# Vivado's `core_revision` property is a bounded 32-bit integer (max
# 2147483647 / 0x7FFFFFFF). Any date from 2022-01-01 onward produces a
# YYMMDDHHMM value >= 2200000000, already past that bound, so the
# set_property call always fails with:
#
#     ERROR: '<value>' is an invalid argument. Please specify an integer value.
#
# This is a defect in the Vitis HLS 2021.1 IP-packaging backend (part of the
# Xilinx install, not project source), so it can't be fixed by editing
# runhls.tcl, the HLS C++ source, or run_ippack.tcl by hand: HLS overwrites
# run_ippack.tcl from scratch, with a fresh timestamp, on every export. This
# script patches the value after each export completes, unconditionally, so
# you never have to hand-edit it again.
#
# Usage:
#   fix_hls_core_revision.sh <path-to-run_ippack.tcl> [--repackage]
#
#   --repackage   Also re-run only the Vivado IP-packaging step on the
#                 patched file. Skips re-running HLS C-synthesis (which has
#                 already succeeded if you hit this error) -- much faster
#                 than re-running the whole `make syn`.

set -euo pipefail

if [[ $# -lt 1 ]]; then
    echo "usage: $0 <path-to-run_ippack.tcl> [--repackage]" >&2
    exit 1
fi

TCL_FILE="$1"
MODE="${2:-}"

if [[ ! -f "$TCL_FILE" ]]; then
    echo "error: not found: $TCL_FILE" >&2
    exit 1
fi

CURRENT="$(grep -oE '^set Revision[[:space:]]+"[0-9]+"' "$TCL_FILE" | grep -oE '[0-9]+' || true)"

if [[ -z "$CURRENT" ]]; then
    echo "warning: no 'set Revision \"<digits>\"' line found in $TCL_FILE -- nothing to patch" >&2
elif (( CURRENT <= 2147483647 )); then
    echo "core_revision ($CURRENT) is already within range, leaving as-is"
else
    sed -i -E 's/^(set Revision[[:space:]]+)"[0-9]+"/\1"0"/' "$TCL_FILE"
    echo "patched core_revision: $CURRENT -> 0 (was overflowing Vivado's 32-bit int property)"
fi

if [[ "$MODE" == "--repackage" ]]; then
    echo "re-running IP packaging only (HLS C-synthesis is NOT re-run)..."
    vivado -mode batch -source "$TCL_FILE"
fi
