#!/usr/bin/env bash
# build_hw.sh - Vivado: block design -> bitstream -> PASS_SFAP.xsa, in the
# antsdr HDL tree (it needs the ADI library and project scripts from there).
#
# Usage (package root, Vivado 2023.2 settings loaded, after build_ip.sh):
#   ./scripts/build_hw.sh ~/antsdr/antsdr-fw-patch/plutosdr-fw/hdl
#
# Steps
#   1. checks the HDL tree (ADI scripts, axi_tdd, library IPs; builds missing
#      library IPs with make)
#   2. copies project/Pass_SFAP (+ ip_repo) to <hdl>/projects/Pass_SFAP
#      (an existing one is renamed Pass_SFAP.bak.<date>)
#   3. system_project.tcl: project, block design, synthesis, implementation
#      (power_opt_design on, DDR DRC waiver), bitstream          ~1-2 h
#   4. finish_hw.tcl: methodology waivers, implementation again, reports,
#      PASS_SFAP.xsa only if timing is met                         ~0.5-1 h
#   4b. only if timing is not met: fix_timing.tcl (4 strategies in parallel)
#   5. copies PASS_SFAP.xsa to hw/
set -e
HDL="${1:?usage: $0 <path to plutosdr-fw/hdl>}"
HDL="$(cd "$HDL" && pwd)"
cd "$(dirname "$0")/.."
PKG="$(pwd)"
PRJ="$HDL/projects/Pass_SFAP"

command -v vivado >/dev/null || { echo "ERROR: vivado not on PATH - source <Vivado 2023.2>/settings64.sh first"; exit 1; }

echo "=== 1. HDL tree checks: $HDL ==="
for f in scripts/adi_env.tcl projects/scripts/adi_project_xilinx.tcl projects/scripts/adi_board.tcl \
         library/axi_tdd/scripts/axi_tdd.tcl library/common/ad_iobuf.v; do
  test -f "$HDL/$f" || { echo "ERROR: $HDL/$f not found - is this the antsdr plutosdr-fw/hdl folder?"; exit 1; }
done
for lib in axi_ad9361 axi_dmac util_pack/util_cpack2 util_pack/util_upack2 axi_tdd; do
  if [ ! -f "$HDL/library/$lib/component.xml" ]; then
    echo "library IP $lib not built yet -> make -C library/$lib"
    make -C "$HDL/library/$lib"
  fi
done
if ! grep -rqls --include=component.xml "<spirit:name>axi_vcxo_ctrl</spirit:name>" "$HDL/library" ; then
  echo "WARNING: axi_vcxo_ctrl (user.org:user:axi_vcxo_ctrl:1.0) not found in $HDL/library."
  echo "         The block design needs it (as Pass_MVDR_STAP). Use the antsdr tree that built Pass_MVDR_STAP."
fi
for ip in sfap_pi gnss_passthrough; do
  test -f "$PKG/project/Pass_SFAP/ip_repo/$ip/component.xml" || { echo "ERROR: ip_repo/$ip missing - run ./scripts/build_ip.sh first"; exit 1; }
done

echo "=== 2. project folder: $PRJ ==="
if [ -e "$PRJ" ]; then
  BAK="$PRJ.bak.$(date +%Y%m%d_%H%M%S)"
  echo "existing $PRJ -> $BAK"
  mv "$PRJ" "$BAK"
fi
cp -r "$PKG/project/Pass_SFAP" "$PRJ"

cd "$PRJ"
export GNSS_CRPA_IP_REPO="$PRJ/ip_repo"
echo "=== 3. system_project.tcl (synthesis + implementation + bitstream) ==="
vivado -mode batch -nojournal -log system_project.log -source system_project.tcl

echo "=== 4. finish_hw.tcl (waivers, implementation again, reports, XSA) ==="
vivado -mode batch -nojournal -log finish_hw.log -source finish_hw.tcl
grep -E "WAIVED|TIMING:|CRITICAL WARNINGS LEFT|LEFT:|NOT MET|XSA written" finish_hw.log || true

if [ ! -f PASS_SFAP.xsa ]; then
  echo "=== 4b. timing not met -> fix_timing.tcl (4 other strategies in parallel, best one kept) ==="
  vivado -mode batch -nojournal -log fix_timing.log -source fix_timing.tcl
  grep -E "RESULTS|impl_[0-9]  |BEST:|TIMING:|CRITICAL WARNINGS LEFT|LEFT:|NOT MET|NO RUN|XSA written" fix_timing.log || true
fi

test -f PASS_SFAP.xsa || { echo "ERROR: PASS_SFAP.xsa not written (timing not met) - see finish_hw.log, fix_timing.log and reports*/"; exit 1; }
cp PASS_SFAP.xsa "$PKG/hw/PASS_SFAP.xsa"
echo
echo "XSA: $PKG/hw/PASS_SFAP.xsa"
echo "Vivado project: $PRJ/passthrough_sfap.xpr"
echo "Reports: $PRJ/reports/"
