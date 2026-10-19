#!/usr/bin/env bash
# ============================================================================
#  build_all.sh  --  LoopBack_Code (gnss_passthrough v3.0 + LCMV CRPA)
#                    sources  ->  bitstream / XSA  ->  FSBL + app ELF  ->  BOOT.BIN
#
#  Linux (Ubuntu 22.04). Default tool version: Vivado + Vitis 2023.2.
#
#      ./Automation/build_all.sh                 # bench build  (no auto-TX)
#      DEPLOY=1 ./Automation/build_all.sh        # SD deployment (auto-TX at boot,
#                                                #  see SD_Image/README_SD_CARD.md)
#      DRY_RUN=1 ./Automation/build_all.sh       # print the commands only
#
#  TOOL_VER   2023.2 (default) or 2026.1
#               2023.2 : ADI hdl_2023_r2 (cloned into Vendor/ if missing),
#                        FSBL + firmware with XSCT (Vitis classic)
#               2026.1 : ADI_hdl_2026_r1_update, FSBL + firmware with the
#                        Vitis Python API (XSCT no longer exists in 2026.1)
#  XILINX_SETTINGS   settings64.sh of that Vitis install
#               2023.2 default: /tools/Xilinx/Vitis/2023.2/settings64.sh
#               2026.1 default: /tools/Xilinx/2026.1/Vitis/settings64.sh
#  ADI_VENDOR  override the ADI HDL tree (absolute path)
#  JOBS=8   SKIP_ADI_LIBS=1 (reuse already packaged ADI libraries)
#
#  Stages (each stops the script on failure; logs in Build/Logs/):
#    1  working copy of the ADI HDL tree      Build/VendorWork/hdl
#    2  package the ADI library IPs           make -C <lib>
#    3  package gnss_passthrough (+ crpa/)    Build/ip_repo/gnss_passthrough
#    4  create the Vivado project             Vivado/Project/antsdr_e310_gnss.xpr
#    5  synth / impl / bitstream / XSA        Build/Bit/system_top.bit, Build/XSA/system.xsa
#    6  FSBL                                  Build/ELF/fsbl.elf
#    7  application                           Build/ELF/app_gnss_e310.elf
#    8  BOOT.BIN                              Build/Boot/BOOT.BIN
# ============================================================================
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TOOL_VER="${TOOL_VER:-2023.2}"
JOBS="${JOBS:-8}"
DEPLOY="${DEPLOY:-0}"
DRY_RUN="${DRY_RUN:-0}"
SKIP_ADI_LIBS="${SKIP_ADI_LIBS:-0}"

case "$TOOL_VER" in
  2023.2)
    : "${XILINX_SETTINGS:=/tools/Xilinx/Vitis/2023.2/settings64.sh}"
    : "${ADI_VENDOR:=$ROOT/Vendor/ADI_hdl_2023_r2}"
    ADI_GIT_BRANCH="hdl_2023_r2"
    SW_FLOW="xsct" ;;
  2026.1)
    : "${XILINX_SETTINGS:=/tools/Xilinx/2026.1/Vitis/settings64.sh}"
    : "${ADI_VENDOR:=$ROOT/Vendor/ADI_hdl_2026_r1_update}"
    ADI_GIT_BRANCH=""
    SW_FLOW="python" ;;
  *) echo "TOOL_VER must be 2023.2 or 2026.1 (got $TOOL_VER)"; exit 2 ;;
esac

WORK_HDL="$ROOT/Build/VendorWork/hdl"
IP_REPO="$ROOT/Build/ip_repo"
LOGS="$ROOT/Build/Logs"
BIT="$ROOT/Build/Bit/system_top.bit"
XSA="$ROOT/Build/XSA/system.xsa"
FSBL_ELF="$ROOT/Build/ELF/fsbl.elf"
APP_ELF="$ROOT/Build/ELF/app_gnss_e310.elf"
BOOT_DIR="$ROOT/Build/Boot"
XPR="$ROOT/Vivado/Project/antsdr_e310_gnss.xpr"

# ADI libraries instantiated by Source/BlockDesign/system_bd.tcl. Their own
# Makefiles package dependencies (util_cdc, util_axis_fifo, ...) automatically.
ADI_LIBS=(
  axi_ad9361 axi_dmac axi_gpreg axi_sysid sysid_rom
  util_pack/util_cpack2 util_pack/util_upack2
  util_rfifo util_wfifo util_tdd_sync xilinx/util_clkdiv
)

say()  { printf '\n==== %s\n' "$*"; }
run()  {                       # run <logname> <cmd...>
  local log="$LOGS/$1.log"; shift
  echo "+ $*"
  [ "$DRY_RUN" = 1 ] && return 0
  if ! "$@" > "$log" 2>&1; then
    echo "FAILED - see $log"; tail -25 "$log"; exit 2
  fi
}
need() {                       # need <log> <marker>: build scripts print a PASS marker
  [ "$DRY_RUN" = 1 ] && return 0
  grep -q "$2" "$LOGS/$1.log" || { echo "FAILED - '$2' not in $LOGS/$1.log"; tail -25 "$LOGS/$1.log"; exit 2; }
}

echo "Tool version : $TOOL_VER  (software flow: $SW_FLOW)"
echo "Settings     : $XILINX_SETTINGS"
echo "ADI HDL tree : $ADI_VENDOR"

# ---- tools -------------------------------------------------------------------
if [ -f "$XILINX_SETTINGS" ]; then
  # shellcheck disable=SC1090
  source "$XILINX_SETTINGS"
elif [ "$DRY_RUN" != 1 ]; then
  echo "XILINX_SETTINGS=$XILINX_SETTINGS does not exist."
  echo "Find yours with:  ls /tools/Xilinx/Vitis/*/settings64.sh /tools/Xilinx/*/Vitis/settings64.sh"
  exit 2
fi
if [ "$SW_FLOW" = xsct ]; then TOOLS="vivado xsct bootgen make rsync"; else TOOLS="vivado vitis bootgen make rsync"; fi
if [ "$DRY_RUN" != 1 ]; then
  for t in $TOOLS; do
    command -v "$t" >/dev/null || { echo "'$t' not found. Check XILINX_SETTINGS (and: sudo apt install make rsync)"; exit 2; }
  done
  have="$(vivado -version 2>/dev/null | head -1 || true)"
  case "$have" in
    *"v$TOOL_VER"*) echo "Vivado       : $have" ;;
    *) echo "WARNING: TOOL_VER=$TOOL_VER but '$have' is on PATH" ;;
  esac
fi

# ---- ADI HDL tree for this tool version -----------------------------------------
if [ ! -d "$ADI_VENDOR/library" ]; then
  if [ -n "$ADI_GIT_BRANCH" ]; then
    say "0/8 fetching ADI HDL $ADI_GIT_BRANCH -> $ADI_VENDOR"
    command -v git >/dev/null || { echo "git not found: sudo apt install git"; exit 2; }
    if [ "$DRY_RUN" = 1 ]; then
      echo "+ git clone --depth 1 -b $ADI_GIT_BRANCH https://github.com/analogdevicesinc/hdl.git $ADI_VENDOR"
    else
      git clone --depth 1 -b "$ADI_GIT_BRANCH" https://github.com/analogdevicesinc/hdl.git "$ADI_VENDOR"
    fi
  else
    echo "ADI HDL tree missing: $ADI_VENDOR"; exit 2
  fi
fi
mkdir -p "$LOGS" "$IP_REPO" "$(dirname "$BIT")" "$(dirname "$XSA")" "$(dirname "$APP_ELF")" "$BOOT_DIR"

# ---- environment read by create_project.tcl / system_bd.tcl -------------------
export GNSS_CRPA_ROOT="$ROOT"
export GNSS_CRPA_IP_REPO="$IP_REPO"
export ADI_HDL_DIR="$WORK_HDL"
export REQUIRED_VIVADO_VERSION="$TOOL_VER"
export ADI_IGNORE_VERSION_CHECK=1          # tolerate 2023.2.x updates / 2026.1 vs 2025.1

# ---- 1  ADI working copy (Vendor/ stays pristine) -----------------------------
say "1/8 ADI working copy -> $WORK_HDL"
run 1_vendor_copy rsync -a --exclude .git "$ADI_VENDOR/" "$WORK_HDL/"

# ---- 2  ADI library IPs -------------------------------------------------------
if [ "$SKIP_ADI_LIBS" != 1 ]; then
  for lib in "${ADI_LIBS[@]}"; do
    say "2/8 package ADI library $lib"
    run "2_lib_${lib//\//_}" make -C "$WORK_HDL/library/$lib"
  done
else
  say "2/8 ADI libraries skipped (SKIP_ADI_LIBS=1)"
fi

# ---- 3  custom IP: gnss_passthrough v4.0 (AXI-Stream ports to the LCMV IP) ------
say "3/8 package gnss_passthrough v4.0"
run 3_gnss_ip bash -c "cd '$LOGS' && vivado -mode batch -nojournal -nolog \
  -source '$ROOT/Source/IP/gnss_passthrough/gnss_passthrough_ip.tcl' \
  -tclargs '$ROOT/Source' '$IP_REPO'"
need 3_gnss_ip "IP_PACKAGE_SELFCHECK: OK - LCMV AXI-Stream interfaces"
need 3_gnss_ip "IP_PACKAGE_OK"

# ---- 3b LCMV CRPA HLS core (lcmv_0 in the block design) -----------------------
# Vitis HLS C-sim + synthesis + export to $IP_REPO/lcmv_crpa. SKIP_HLS=1 reuses
# an existing $IP_REPO/lcmv_crpa (it only changes when Source/HLS/lcmv changes).
if [ "${SKIP_HLS:-0}" = 1 ] && [ -f "$IP_REPO/lcmv_crpa/component.xml" ]; then
  say "3b/8 LCMV HLS skipped (SKIP_HLS=1, $IP_REPO/lcmv_crpa exists)"
else
  command -v vitis_hls >/dev/null || { echo "vitis_hls not found: source the Vitis $TOOL_VER settings64.sh"; exit 2; }
  say "3b/8 build LCMV CRPA HLS core (vitis_hls, ~5 min)"
  run 3b_lcmv_hls bash -c "cd '$ROOT/Source/HLS/lcmv' && GNSS_CRPA_IP_REPO='$IP_REPO' vitis_hls -f build_hls.tcl"
  need 3b_lcmv_hls "LCMV_HLS: PASS"
fi

# ---- 4  Vivado project ----------------------------------------------------------
say "4/8 create Vivado project"
run 4_create_project bash -c "cd '$ROOT/Vivado/Project' && rm -rf antsdr_e310_gnss.* && \
  vivado -mode batch -source ../Scripts/create_project.tcl"
need 4_create_project "CREATE_RESULT: PASS"

# ---- 5  bitstream + XSA -----------------------------------------------------------
say "5/8 synthesis, implementation, bitstream, XSA  (long)"
run 5_build bash -c "cd '$ROOT/Vivado/Project' && \
  vivado -mode batch -source ../Scripts/build.tcl -tclargs '$XPR' '$BIT' '$XSA' $JOBS"
need 5_build "BUILD_RESULT: PASS"
[ "$DRY_RUN" = 1 ] || grep -E "BUILD_TIMING|BUILD_WNS|BUILD_WHS" "$LOGS/5_build.log" || true

# ---- 6  FSBL ------------------------------------------------------------------------
say "6/8 FSBL ($SW_FLOW)"
if [ "$SW_FLOW" = xsct ]; then
  run 6_fsbl xsct "$ROOT/Vitis/Scripts/build_fsbl.tcl" \
    "$XSA" "$ROOT/Vitis/Workspace/fsbl" "$FSBL_ELF"
else
  run 6_fsbl vitis -s "$ROOT/Vitis/Scripts/build_fsbl.py" -- \
    "$XSA" "$ROOT/Vitis/Workspace/fsbl" "$FSBL_ELF"
fi
need 6_fsbl "FSBL_RESULT: PASS"

# ---- 7  application ----------------------------------------------------------------
DEFINES=""
[ "$DEPLOY" = 1 ] && DEFINES="GNSS_DEPLOY_AUTO_TX"
say "7/8 application firmware ($SW_FLOW) ${DEFINES:+defines: $DEFINES}"
if [ "$SW_FLOW" = xsct ]; then
  run 7_app xsct "$ROOT/Vitis/Scripts/build_software.tcl" \
    "$XSA" "$ROOT/Vitis/Workspace/app" "$ROOT/Source/Firmware/app_gnss_e310" "$APP_ELF" "$DEFINES"
else
  run 7_app vitis -s "$ROOT/Vitis/Scripts/build_software.py" -- \
    "$XSA" "$ROOT/Vitis/Workspace/app" "$ROOT/Source/Firmware/app_gnss_e310" "$APP_ELF" "$DEFINES"
fi
need 7_app "SW_RESULT: PASS"

# ---- 8  BOOT.BIN -----------------------------------------------------------------
say "8/8 BOOT.BIN"
cat > "$BOOT_DIR/boot.bif" <<EOF
the_ROM_image:
{
  [bootloader] $FSBL_ELF
  $BIT
  $APP_ELF
}
EOF
run 8_bootgen bootgen -arch zynq -image "$BOOT_DIR/boot.bif" -o "$BOOT_DIR/BOOT.BIN" -w on
[ "$DRY_RUN" = 1 ] || [ -s "$BOOT_DIR/BOOT.BIN" ] || { echo "FAILED - BOOT.BIN not produced"; exit 2; }

say "DONE ($TOOL_VER)"
echo "  bitstream : $BIT"
echo "  XSA       : $XSA"
echo "  FSBL      : $FSBL_ELF"
echo "  app       : $APP_ELF  $( [ "$DEPLOY" = 1 ] && echo '(DEPLOY: transmits at boot)')"
echo "  BOOT.BIN  : $BOOT_DIR/BOOT.BIN   -> copy to a FAT32 SD card, set boot mode SD"
echo "  check timing: $ROOT/Build/Bit/timing_impl.rpt"
