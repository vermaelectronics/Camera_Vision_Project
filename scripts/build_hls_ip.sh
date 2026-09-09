#!/usr/bin/env bash
#
# build_hls_ip.sh
#
# Drop-in replacement for running `vitis_hls -f runhls.tcl` directly.
# Vitis HLS 2021.1 stamps the packaged IP's core_revision with a
# YYMMDDHHMM timestamp on every export_design -format ip_catalog. That
# value has been >= Vivado's 32-bit int bound (2147483647) for every date
# since 2022-01-01, so IP packaging always fails at the very last step,
# even though C-synthesis itself succeeds. See
# docs/hls-core-revision-overflow.md for the full root-cause writeup.
#
# This script runs runhls.tcl as normal. If it fails specifically on that
# error, it patches the generated run_ippack.tcl and re-runs *only* the
# Vivado IP-packaging step on it -- C-synthesis is not repeated.
#
# Usage:
#   ./scripts/build_hls_ip.sh [runhls.tcl] [ip_dir]
#
#   runhls.tcl   HLS Tcl script to run (default: runhls.tcl)
#   ip_dir       HLS IP project directory (default: design_1_v_demosaic_0_0)

set -uo pipefail

TCL_SCRIPT="${1:-runhls.tcl}"
IP_DIR="${2:-design_1_v_demosaic_0_0}"
PACKAGE_TCL="$IP_DIR/prj/impl/ip/run_ippack.tcl"

echo "==> vitis_hls -f $TCL_SCRIPT"
vitis_hls -f "$TCL_SCRIPT"
status=$?

if [[ $status -ne 0 ]]; then
    if [[ -f "$PACKAGE_TCL" ]] \
        && grep -q 'core_revision \$Revision' "$PACKAGE_TCL" \
        && grep -qE '^set Revision[[:space:]]+"[0-9]{9,}"' "$PACKAGE_TCL"
    then
        echo "==> vitis_hls failed at IP packaging on an oversized core_revision"
        echo "==> patching $PACKAGE_TCL and re-running packaging only (no re-synthesis)"
        sed -i -E 's/^(set Revision[[:space:]]+)"[0-9]+"/\1"0"/' "$PACKAGE_TCL"
        vivado -mode batch -source "$PACKAGE_TCL"
        status=$?
        if [[ $status -eq 0 ]]; then
            echo "==> IP packaging recovered successfully"
        fi
    else
        echo "==> vitis_hls failed for a different reason -- not auto-fixing, see the log above"
    fi
fi

exit $status
