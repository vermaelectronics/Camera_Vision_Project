#!/usr/bin/env bash
# ============================================================================
#  build_all.sh  --  LoopBack_Code (gnss_passthrough v2.0 + LCMV CRPA)
#                    sources  ->  bitstream / XSA  ->  FSBL + app ELF  ->  BOOT.BIN
#
#  Linux (Ubuntu 22.04), Vivado + Vitis 2026.1. Run from anywhere:
#
#      export XILINX_SETTINGS=/tools/Xilinx/2026.1/Vitis/settings64.sh   # your install
#      ./Automation/build_all.sh                 # bench build  (no auto-TX)
#      DEPLOY=1 ./Automation/build_all.sh        # SD deployment (auto-TX at boot,
#                                                #  same as SD_Image/README_SD_CARD.md)
#      DRY_RUN=1 ./Automation/build_all.sh       # print the commands only
#
#  Optional:  JOBS=8   SKIP_ADI_LIBS=1 (reuse already packaged ADI libraries)
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
JOBS="${JOBS:-8}"
DEPLOY="${DEPLOY:-0}"
DRY_RUN="${DRY_RUN:-0}"
SKIP_ADI_LIBS="${SKIP_ADI_LIBS:-0}"

VENDOR_SRC="$ROOT/Vendor/ADI_hdl_2026_r1_update"   # has projects/scripts/auto_timing_fix_xilinx.tcl
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

# ---- tools -------------------------------------------------------------------
if [ -n "${XILINX_SETTINGS:-}" ]; then
  # shellcheck disable=SC1090
  source "$XILINX_SETTINGS"
fi
if [ "$DRY_RUN" != 1 ]; then
  for t in vivado vitis bootgen make rsync; do
    command -v "$t" >/dev/null || { echo "'$t' not found. Set XILINX_SETTINGS to your settings64.sh"; exit 2; }
  done
fi
[ -d "$VENDOR_SRC" ] || { echo "Vendor tree missing: $VENDOR_SRC"; exit 2; }
mkdir -p "$LOGS" "$IP_REPO" "$(dirname "$BIT")" "$(dirname "$XSA")" "$(dirname "$APP_ELF")" "$BOOT_DIR"

# ---- environment read by create_project.tcl / system_bd.tcl -------------------
export GNSS_CRPA_ROOT="$ROOT"
export GNSS_CRPA_IP_REPO="$IP_REPO"
export ADI_HDL_DIR="$WORK_HDL"
export REQUIRED_VIVADO_VERSION="2026.1"   # ADI 2026_r1 expects 2025.1
export ADI_IGNORE_VERSION_CHECK=1

# ---- 1  ADI working copy (Vendor/ stays pristine) -----------------------------
say "1/8 ADI working copy -> $WORK_HDL"
run 1_vendor_copy rsync -a "$VENDOR_SRC/" "$WORK_HDL/"

# ---- 2  ADI library IPs -------------------------------------------------------
if [ "$SKIP_ADI_LIBS" != 1 ]; then
  for lib in "${ADI_LIBS[@]}"; do
    say "2/8 package ADI library $lib"
    run "2_lib_${lib//\//_}" make -C "$WORK_HDL/library/$lib"
  done
else
  say "2/8 ADI libraries skipped (SKIP_ADI_LIBS=1)"
fi

# ---- 3  custom IP: gnss_passthrough v2.0 + crpa/ ------------------------------
say "3/8 package gnss_passthrough (LCMV CRPA)"
run 3_gnss_ip bash -c "cd '$LOGS' && vivado -mode batch -nojournal -nolog \
  -source '$ROOT/Source/IP/gnss_passthrough/gnss_passthrough_ip.tcl' \
  -tclargs '$ROOT/Source' '$IP_REPO'"
need 3_gnss_ip "IP_PACKAGE_SELFCHECK: OK - LCMV CRPA sources packaged"
need 3_gnss_ip "IP_PACKAGE_OK"

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
say "6/8 FSBL"
run 6_fsbl vitis -s "$ROOT/Vitis/Scripts/build_fsbl.py" -- \
  "$XSA" "$ROOT/Vitis/Workspace/fsbl" "$FSBL_ELF"
need 6_fsbl "FSBL_RESULT: PASS"

# ---- 7  application ----------------------------------------------------------------
DEFINES=""
[ "$DEPLOY" = 1 ] && DEFINES="GNSS_DEPLOY_AUTO_TX"
say "7/8 application firmware ${DEFINES:+(defines: $DEFINES)}"
run 7_app vitis -s "$ROOT/Vitis/Scripts/build_software.py" -- \
  "$XSA" "$ROOT/Vitis/Workspace/app" "$ROOT/Source/Firmware/app_gnss_e310" "$APP_ELF" "$DEFINES"
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

say "DONE"
echo "  bitstream : $BIT"
echo "  XSA       : $XSA"
echo "  FSBL      : $FSBL_ELF"
echo "  app       : $APP_ELF  $( [ "$DEPLOY" = 1 ] && echo '(DEPLOY: transmits at boot)')"
echo "  BOOT.BIN  : $BOOT_DIR/BOOT.BIN   -> copy to a FAT32 SD card, set boot mode SD"
echo "  check timing: $ROOT/Build/Bit/timing_impl.rpt"
