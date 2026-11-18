#!/usr/bin/env bash
# v6.3: register map / excision-mask bank / commit test of gnss_passthrough.v
# (Icarus Verilog 11+). Prints HDL_TB: PASS.
set -e
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
iverilog -g2005 -o /tmp/tb_gnss_pt "$here/tb_gnss_passthrough_v63.v" "$here/../gnss_passthrough.v"
vvp -n /tmp/tb_gnss_pt | tee /tmp/tb_gnss_pt.log
grep -q "HDL_TB: PASS" /tmp/tb_gnss_pt.log
