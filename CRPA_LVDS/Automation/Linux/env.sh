#!/usr/bin/env bash
# ============================================================================
#  env.sh  --  paths for the Passthrough_LVDS + CRPA build.
#  Sourced by build_all.sh. Edit the defaults here or override any of them in
#  the environment, e.g.
#     ADI_HDL_DIR=/home/me/AntSDR/antsdr-fw-patch/plutosdr-fw/hdl ./build_all.sh --algo crpa27
#  Per-algorithm paths (Build/<algo>/...) are set by build_all.sh once the
#  algorithm is known (crpa_env_for_algo below).
# ============================================================================

# Package root (holds Source/, Automation/, Build/). Found from this script.
export CRPA_ROOT="${CRPA_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"

# Xilinx tools. passthrough_lvds is a Vivado 2023.2 design (antsdr-fw-patch
# ADI hdl tree), so Vivado, Vitis and Vitis HLS 2023.2 are used.
export XILINX_ROOT="${XILINX_ROOT:-/tools/Xilinx}"
export XILINX_VER="${XILINX_VER:-2023.2}"

# The ADI hdl tree that already builds passthrough_lvds:
#   <...>/antsdr-fw-patch/plutosdr-fw/hdl   (contains projects/passthrough_lvds,
#   library/axi_ad9361, library/axi_vcxo_ctrl ...)
# A full package (Automation/Linux/bundle_full.sh) carries its own copy in
# Vendor/adi_hdl; it is used when present and ADI_HDL_DIR is not set.
if [[ -z "${ADI_HDL_DIR:-}" && -f "$CRPA_ROOT/Vendor/adi_hdl/scripts/adi_env.tcl" ]]; then
  export ADI_HDL_DIR="$CRPA_ROOT/Vendor/adi_hdl"
fi
export ADI_HDL_DIR="${ADI_HDL_DIR:-$HOME/antsdr/antsdr-fw-patch/plutosdr-fw/hdl}"

# Build outputs: one tree per algorithm, Build/<algo>/
export CRPA_BUILD_ROOT="${CRPA_BUILD_ROOT:-$CRPA_ROOT/Build}"

# Firmware source folder and application name
export CRPA_APP_NAME="${CRPA_APP_NAME:-app_crpa_lvds}"

# ADI checks the Vivado version (2023.2 for this tree). Set to 1 only if you
# deliberately run another Vivado version.
# (Only exported when you set it: some ADI releases treat "defined" as "on".)
if [[ "${ADI_IGNORE_VERSION_CHECK:-0}" == "1" ]]; then
  export ADI_IGNORE_VERSION_CHECK=1
else
  unset ADI_IGNORE_VERSION_CHECK
fi

# Parallel jobs for the ADI library builds.
export CRPA_JOBS="${CRPA_JOBS:-4}"

# Per-algorithm paths. The Vivado project is generated INSIDE the ADI tree,
# next to passthrough_lvds (the ADI scripts find themselves by relative path),
# as projects/passthrough_lvds_<algo>; passthrough_lvds itself is not touched.
crpa_env_for_algo() {   # <algo>
  local a="$1"
  export CRPA_ALGO="$a"
  export CRPA_BUILD="$CRPA_BUILD_ROOT/$a"
  export GNSS_CRPA_IP_REPO="$CRPA_BUILD/ip_repo"
  export CRPA_SW_WS="$CRPA_BUILD/vitis_ws"
  export CRPA_FW_STAGE="$CRPA_BUILD/fw_src"
  export CRPA_BOOT_DIR="$CRPA_BUILD/boot"
  export CRPA_LOG_DIR="$CRPA_BUILD/logs"
  export CRPA_HDL_PRJ_NAME="passthrough_lvds_$a"
  export CRPA_HDL_PRJ_DIR="$ADI_HDL_DIR/projects/$CRPA_HDL_PRJ_NAME"
}
