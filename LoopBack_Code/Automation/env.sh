# ============================================================================
#  env.sh  --  environment for building LoopBack_Code by hand (Linux)
#
#      source Automation/env.sh            # from the LoopBack_Code folder
#
#  Replaces what Automation/PowerShell/Build_All.ps1 set on Windows. Read by
#  Vivado/Scripts/create_project.tcl and Source/BlockDesign/system_bd.tcl.
#  Edit XILINX_SETTINGS if Vitis is installed somewhere else.
# ============================================================================

: "${XILINX_SETTINGS:=/tools/Xilinx/2026.1/Vitis/settings64.sh}"

if [ -f "$XILINX_SETTINGS" ]; then
  # shellcheck disable=SC1090
  source "$XILINX_SETTINGS"
else
  echo "env.sh: $XILINX_SETTINGS not found - set XILINX_SETTINGS to your settings64.sh"
fi

export GNSS_CRPA_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export GNSS_CRPA_IP_REPO="$GNSS_CRPA_ROOT/Build/ip_repo"
export ADI_HDL_DIR="$GNSS_CRPA_ROOT/Build/VendorWork/hdl"   # working copy, Vendor/ stays pristine
export REQUIRED_VIVADO_VERSION="2026.1"                      # ADI 2026_r1 expects 2025.1
export ADI_IGNORE_VERSION_CHECK=1

echo "GNSS_CRPA_ROOT    = $GNSS_CRPA_ROOT"
echo "GNSS_CRPA_IP_REPO = $GNSS_CRPA_IP_REPO"
echo "ADI_HDL_DIR       = $ADI_HDL_DIR"
for t in vivado vitis bootgen; do
  command -v "$t" >/dev/null && echo "$t: $(command -v "$t")" || echo "$t: NOT FOUND"
done
