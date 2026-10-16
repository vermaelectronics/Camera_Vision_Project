#!/usr/bin/env bash
# ============================================================================
#  setup_antsdr_lcmv.sh  --  create ~/AntSDR_LCMV from the downloaded zip
#
#      bash ~/Downloads/setup_antsdr_lcmv.sh          # set up only
#      BUILD=1 bash ~/Downloads/setup_antsdr_lcmv.sh  # set up, then build BOOT.BIN
#
#  Result:
#      ~/AntSDR_LCMV/LoopBack_Code/            v2.0 sources (from the zip)
#      ~/AntSDR_LCMV/LoopBack_Code/Vendor/     copied from your old tree (if found)
#                                              + ADI hdl_2023_r2 (fetched by build_all.sh)
#      ~/AntSDR_LCMV/LoopBack_Code/Build/Boot/BOOT.BIN   (with BUILD=1)
#
#  Overridable:
#      DEST=~/AntSDR_LCMV
#      ZIP=~/Downloads/LoopBack_Code_LCMV_v2.zip    (default: newest match)
#      OLD_VENDOR=~/Desktop/AntSDR2/LoopBack_Code/Vendor
#      TOOL_VER=2023.2 (default) | 2026.1
#      XILINX_SETTINGS=/tools/Xilinx/Vitis/2023.2/settings64.sh
#      DEPLOY=1   (with BUILD=1: SD image that transmits at boot)
# ============================================================================
set -euo pipefail

DEST="${DEST:-$HOME/AntSDR_LCMV}"
OLD_VENDOR="${OLD_VENDOR:-$HOME/Desktop/AntSDR2/LoopBack_Code/Vendor}"
TOOL_VER="${TOOL_VER:-2023.2}"
if [ "$TOOL_VER" = 2026.1 ]; then
  XILINX_SETTINGS="${XILINX_SETTINGS:-/tools/Xilinx/2026.1/Vitis/settings64.sh}"
else
  XILINX_SETTINGS="${XILINX_SETTINGS:-/tools/Xilinx/Vitis/2023.2/settings64.sh}"
fi

# Newest download, so "LoopBack_Code_LCMV_v2 (1).zip" is found too.
if [ -z "${ZIP:-}" ]; then
  ZIP="$(ls -t "$HOME"/Downloads/LoopBack_Code_LCMV_v2*.zip 2>/dev/null | head -1 || true)"
fi
[ -n "$ZIP" ] && [ -f "$ZIP" ] || { echo "Zip not found in ~/Downloads (set ZIP=...)"; exit 2; }
HAVE_OLD_VENDOR=0
[ -d "$OLD_VENDOR" ] && HAVE_OLD_VENDOR=1
if [ "$HAVE_OLD_VENDOR" = 0 ] && [ "$TOOL_VER" = 2026.1 ]; then
  echo "Vendor tree not found at $OLD_VENDOR (needed for TOOL_VER=2026.1)"
  echo "Set OLD_VENDOR to the folder that contains ADI_hdl_2026_r1_update/"; exit 2
fi

for t in unzip rsync make git; do
  command -v "$t" >/dev/null || { echo "Missing '$t':  sudo apt install -y unzip rsync make git"; exit 2; }
done

echo "zip        : $ZIP"
echo "tool       : Vivado/Vitis $TOOL_VER"
echo "vendor from: $OLD_VENDOR $( [ "$HAVE_OLD_VENDOR" = 1 ] || echo '(not found - not needed for 2023.2)')"
echo "destination: $DEST"

# 1. sources
mkdir -p "$DEST"
if [ -d "$DEST/LoopBack_Code" ]; then
  echo "Existing $DEST/LoopBack_Code - updating sources, keeping Vendor/"
fi
unzip -q -o "$ZIP" -d "$DEST"
ROOT="$DEST/LoopBack_Code"

# 2. vendor tree (copy, so the old project is never touched). For 2023.2 the
#    ADI hdl_2023_r2 tree is cloned into Vendor/ by build_all.sh; the old tree
#    is kept for reference (MicroPhase originals) and for TOOL_VER=2026.1.
if [ "$HAVE_OLD_VENDOR" = 1 ]; then
  echo "Copying vendor tree (~75 MB) ..."
  rsync -a "$OLD_VENDOR/" "$ROOT/Vendor/"
fi
mkdir -p "$ROOT/Vendor"

# 3. permissions, stale outputs from older builds
chmod +x "$ROOT"/Automation/*.sh "$ROOT"/Sim/run_sim.sh
rm -rf "$ROOT"/Build/ip_repo "$ROOT"/Build/VendorWork \
       "$ROOT"/Vivado/Project/antsdr_e310_gnss.* "$ROOT"/Vitis/Workspace/fsbl "$ROOT"/Vitis/Workspace/app

# 4. tools
if [ -f "$XILINX_SETTINGS" ]; then
  echo "Xilinx settings: $XILINX_SETTINGS"
else
  echo "WARNING: $XILINX_SETTINGS not found. Before building:"
  echo "         export XILINX_SETTINGS=<your Vitis $TOOL_VER install>/settings64.sh"
fi

echo
echo "Set up: $ROOT"
echo "Next:   cd $ROOT && TOOL_VER=$TOOL_VER ./Automation/build_all.sh"

# 5. optional build
if [ "${BUILD:-0}" = 1 ]; then
  cd "$ROOT"
  TOOL_VER="$TOOL_VER" XILINX_SETTINGS="$XILINX_SETTINGS" DEPLOY="${DEPLOY:-0}" ./Automation/build_all.sh
fi
