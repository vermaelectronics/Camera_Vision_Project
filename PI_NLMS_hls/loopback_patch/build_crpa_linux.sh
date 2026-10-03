#!/usr/bin/env bash
# ============================================================================
#  build_crpa_linux.sh  --  build LoopBack_Code with gnss_passthrough v1.3
#  (bypass / PI / PI-NLMS) on Linux with the Xilinx 2021.1 tools.
#
#  Linux counterpart of the Automation/PowerShell flow, calling the same Tcl
#  scripts under Vivado/Scripts and Vitis/Scripts.
#
#  USAGE (from anywhere)
#    ./build_crpa_linux.sh                 all steps
#    ./build_crpa_linux.sh hls ip project bit      selected steps, in this order:
#        libs     build the ADI library IPs the block design needs (skipped if built)
#        hls      Vitis HLS: pi_nlms -> Build/hls/pi_nlms/verilog
#        ip       package gnss_passthrough -> Build/ip_repo
#        project  recreate Vivado/Project/antsdr_e310_gnss.xpr
#        bit      synthesis + implementation -> Build/system_top.bit, .xsa
#        sw       bare-metal application -> Build/ELF/e310_gnss_app.elf
#        boot     FSBL + BOOT.BIN -> Build/BOOT/BOOT.BIN
#
#  ENVIRONMENT (all optional)
#    XILINX_ROOT       default /tools/Xilinx
#    ADI_HDL_DIR       default <root>/Vendor/MicroPhase_E310_V1/hdl (2021.1 tree)
#    JOBS              default nproc
#    DEPLOY_AUTO_TX=1  build the SD image that starts transmitting at power-on
#                      (GNSS_DEPLOY_AUTO_TX) -- CONDUCTED, ATTENUATED COAX ONLY
#    DEPLOY_CORE=0|1|2 with DEPLOY_AUTO_TX: nulling core selected at power-on
# ============================================================================
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XILINX_ROOT="${XILINX_ROOT:-/tools/Xilinx}"
export ADI_HDL_DIR="${ADI_HDL_DIR:-$ROOT/Vendor/MicroPhase_E310_V1/hdl}"
export GNSS_CRPA_ROOT="$ROOT"
export GNSS_CRPA_IP_REPO="$ROOT/Build/ip_repo"
export REQUIRED_VIVADO_VERSION="2021.1"
JOBS="${JOBS:-$(nproc)}"
STEPS="${*:-libs hls ip project bit sw boot}"

VIVADO_DIR="$XILINX_ROOT/Vivado/2021.1"
HLS_DIR="$XILINX_ROOT/Vitis_HLS/2021.1"
VITIS_DIR="$XILINX_ROOT/Vitis/2021.1"

die()  { echo "BUILD_CRPA: FAIL - $*" >&2; exit 2; }
step() { echo; echo "==================== $* ===================="; }
want() { [[ " $STEPS " == *" $1 "* ]]; }

[[ -x "$VIVADO_DIR/bin/vivado" ]] || die "Vivado 2021.1 not found at $VIVADO_DIR (set XILINX_ROOT)"
[[ -f "$ADI_HDL_DIR/library/common/ad_iobuf.v" ]] || die "ADI hdl tree not found at $ADI_HDL_DIR"

# The Xilinx settings scripts reference unset variables.
set +u
# shellcheck disable=SC1091
source "$VIVADO_DIR/settings64.sh"
[[ -f "$HLS_DIR/settings64.sh" ]]   && source "$HLS_DIR/settings64.sh"
[[ -f "$VITIS_DIR/settings64.sh" ]] && source "$VITIS_DIR/settings64.sh"
set -u
# 2021.1 first on PATH: the ADI makefiles call plain "vivado", and another
# installed release must not be picked up instead.
export PATH="$VIVADO_DIR/bin:$PATH"
export XILINX_VIVADO="$VIVADO_DIR"
vivado -version | head -1 | grep -q "v2021.1" || die "'vivado' on PATH is not 2021.1: $(vivado -version | head -1)"

mkdir -p "$ROOT/Build"
cd "$ROOT"

# ---------------------------------------------------------------------------
if want libs; then
  step "ADI library IPs ($ADI_HDL_DIR)"
  for lib in axi_ad9361 axi_dmac axi_gpreg axi_sysid sysid_rom xilinx/util_clkdiv \
             util_pack/util_cpack2 util_pack/util_upack2 util_rfifo util_tdd_sync util_wfifo; do
    if [[ -f "$ADI_HDL_DIR/library/$lib/component.xml" ]]; then
      echo "LIB: $lib already built"
    else
      echo "LIB: building $lib"
      make -C "$ADI_HDL_DIR/library/$lib" xilinx || die "ADI library $lib failed (see its *.log)"
    fi
  done
fi

# ---------------------------------------------------------------------------
if want hls; then
  step "Vitis HLS: pi_nlms (8 ns, II=2)"
  command -v vitis_hls >/dev/null || die "vitis_hls 2021.1 not found at $HLS_DIR"
  vitis_hls -f "$ROOT/Source/HLS/pi_nlms/run_hls.tcl" -tclargs "$ROOT" 2>&1 | tee "$ROOT/Build/hls.log" || true
  grep -q "HLS_RESULT: PASS" "$ROOT/Build/hls.log" || die "HLS build (see Build/hls.log)"
fi

# ---------------------------------------------------------------------------
if want ip; then
  step "Package gnss_passthrough IP"
  vivado -mode batch -nojournal -log "$ROOT/Build/ip_package.log" \
    -source "$ROOT/Source/IP/gnss_passthrough/gnss_passthrough_ip.tcl" \
    -tclargs "$ROOT/Source" "$GNSS_CRPA_IP_REPO" "$ROOT/Build/hls/pi_nlms/verilog" || true
  [[ -f "$GNSS_CRPA_IP_REPO/gnss_passthrough/component.xml" ]] || die "IP packaging (see Build/ip_package.log)"
fi

# ---------------------------------------------------------------------------
if want project; then
  step "Create Vivado project"
  mkdir -p "$ROOT/Vivado/Project"
  ( cd "$ROOT/Vivado/Project" && \
    vivado -mode batch -nojournal -log create_project.log -source ../Scripts/create_project.tcl ) || true
  grep -q "CREATE_RESULT: PASS" "$ROOT/Vivado/Project/create_project.log" || die "create_project (see Vivado/Project/create_project.log)"
fi

# ---------------------------------------------------------------------------
if want bit; then
  step "Synthesis + implementation ($JOBS jobs)"
  vivado -mode batch -nojournal -log "$ROOT/Build/build.log" \
    -source "$ROOT/Vivado/Scripts/build.tcl" \
    -tclargs "$ROOT/Vivado/Project/antsdr_e310_gnss.xpr" \
             "$ROOT/Build/system_top.bit" "$ROOT/Build/system_top.xsa" "$JOBS" || true
  grep -q "BUILD_RESULT: PASS" "$ROOT/Build/build.log" || die "build (see Build/build.log)"
  grep "BUILD_TIMING" "$ROOT/Build/build.log" || true
fi

# ---------------------------------------------------------------------------
if want sw; then
  step "Bare-metal application"
  command -v xsct >/dev/null || die "xsct not found: install Vitis 2021.1 at $VITIS_DIR"
  fw="$ROOT/Build/fw_src"
  rm -rf "$fw" && cp -r "$ROOT/Source/Firmware/app_gnss_e310" "$fw"
  {
    echo "#ifndef GNSS_BUILD_DEFINES_H_"
    echo "#define GNSS_BUILD_DEFINES_H_"
    if [[ "${DEPLOY_AUTO_TX:-0}" == "1" ]]; then
      echo "#define GNSS_DEPLOY_AUTO_TX 1"
      [[ -n "${DEPLOY_CORE:-}" ]] && echo "#define GNSS_DEPLOY_CRPA_CORE ${DEPLOY_CORE}U"
    fi
    echo "#endif"
  } > "$fw/gnss_build_defines.h"
  [[ "${DEPLOY_AUTO_TX:-0}" == "1" ]] && echo "WARNING: DEPLOY_AUTO_TX=1 -- this image transmits at power-on."
  rm -rf "$ROOT/Vitis/Workspace/e310_gnss_platform" "$ROOT/Vitis/Workspace/e310_gnss_app"
  xsct "$ROOT/Vitis/Scripts/build_software.tcl" \
       "$ROOT/Build/system_top.xsa" "$ROOT/Vitis/Workspace" "$fw" \
       "$ROOT/Build/ELF/e310_gnss_app.elf" 2>&1 | tee "$ROOT/Build/sw.log" || true
  grep -q "SW_RESULT: PASS" "$ROOT/Build/sw.log" || die "software build (see Build/sw.log)"
fi

# ---------------------------------------------------------------------------
if want boot; then
  step "FSBL + BOOT.BIN"
  command -v xsct >/dev/null || die "xsct not found: install Vitis 2021.1 at $VITIS_DIR"
  xsct "$ROOT/Vitis/Scripts/build_boot_2021.tcl" \
       "$ROOT/Build/system_top.xsa" "$ROOT/Build/system_top.bit" \
       "$ROOT/Build/ELF/e310_gnss_app.elf" "$ROOT/Build/BOOT" 2>&1 | tee "$ROOT/Build/boot.log" || true
  grep -q "BOOT_RESULT: PASS" "$ROOT/Build/boot.log" || die "boot image (see Build/boot.log)"
fi

echo
echo "BUILD_CRPA: PASS ($STEPS)"
