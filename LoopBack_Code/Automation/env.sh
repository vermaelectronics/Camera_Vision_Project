# ============================================================================
#  env.sh  --  environment for building LoopBack_Code by hand (Linux)
#
#      source Automation/env.sh            # from the LoopBack_Code folder
#
#  Replaces what Automation/PowerShell/Build_All.ps1 set on Windows. Read by
#  Vivado/Scripts/create_project.tcl and Source/BlockDesign/system_bd.tcl.
#  Edit XILINX_SETTINGS if Vitis is installed somewhere else.
#  TOOL_VER=2023.2 (default) or 2026.1 selects the matching ADI HDL release.
# ============================================================================

: "${TOOL_VER:=2023.2}"
if [ "$TOOL_VER" = 2026.1 ]; then
  : "${XILINX_SETTINGS:=/tools/Xilinx/2026.1/Vitis/settings64.sh}"
else
  : "${XILINX_SETTINGS:=/tools/Xilinx/Vitis/2023.2/settings64.sh}"
fi

if [ -f "$XILINX_SETTINGS" ]; then
  # shellcheck disable=SC1090
  source "$XILINX_SETTINGS"
else
  echo "env.sh: $XILINX_SETTINGS not found - set XILINX_SETTINGS to your settings64.sh"
fi

export GNSS_CRPA_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export GNSS_CRPA_IP_REPO="$GNSS_CRPA_ROOT/Build/ip_repo"
export ADI_HDL_DIR="$GNSS_CRPA_ROOT/Build/VendorWork/hdl"   # working copy, Vendor/ stays pristine
export REQUIRED_VIVADO_VERSION="$TOOL_VER"
export ADI_IGNORE_VERSION_CHECK=1
if [ "$TOOL_VER" = 2026.1 ]; then
  export ADI_VENDOR="$GNSS_CRPA_ROOT/Vendor/ADI_hdl_2026_r1_update"
else
  export ADI_VENDOR="$GNSS_CRPA_ROOT/Vendor/ADI_hdl_2023_r2"   # git clone -b hdl_2023_r2
fi

echo "GNSS_CRPA_ROOT    = $GNSS_CRPA_ROOT"
echo "GNSS_CRPA_IP_REPO = $GNSS_CRPA_IP_REPO"
echo "ADI_HDL_DIR       = $ADI_HDL_DIR   (working copy of $ADI_VENDOR)"
echo "TOOL_VER          = $TOOL_VER"
for t in vivado xsct vitis bootgen; do
  command -v "$t" >/dev/null && echo "$t: $(command -v "$t")" || echo "$t: NOT FOUND"
done
