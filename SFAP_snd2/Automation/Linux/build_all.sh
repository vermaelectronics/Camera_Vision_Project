#!/usr/bin/env bash
# ============================================================================
#  build_all.sh  --  SFAP-PI (E310 V1): sources -> BOOT.BIN, Vivado/Vitis 2023.2
#
#  STAGES (in order)
#    check  tools on PATH, Vendor tree present, sources present
#    hls    Vitis HLS: C simulation (9 scenarios) + synthesis + IP export of
#           sfap_pi 3.0                         -> Build/ip_repo/sfap_pi
#    ip     Vivado: package gnss_passthrough    -> Build/ip_repo/gnss_passthrough
#    hdl    ADI make: library IPs + Vivado project, synthesis, implementation,
#           bitstream, XSA (fails on a timing violation)
#                                               -> Build/hdl/system_top.{bit,xsa}
#    sw     XSCT: platform, FSBL, standalone BSP, app_gnss_e310.elf
#                                               -> Build/sw/{fsbl,app_gnss_e310}.elf
#    boot   bootgen: FSBL + bitstream + app     -> Build/boot/BOOT.BIN
#
#  USAGE
#    ./build_all.sh                    everything
#    ./build_all.sh --from hdl         this stage and the ones after it
#    ./build_all.sh --only sw          one stage
#    ./build_all.sh --skip-csim        HLS without the C simulation
#    ./build_all.sh --jtag             after the build, run it over JTAG
#
#  Each stage logs to Build/logs/<stage>.log and prints one PASS/FAIL line.
# ============================================================================
set -uo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$here/env.sh"

STAGES=(check hls ip hdl sw boot)
from="check"; only=""; skip_csim=0; jtag=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --from)      from="$2"; shift 2 ;;
    --only)      only="$2"; shift 2 ;;
    --skip-csim) skip_csim=1; shift ;;
    --jtag)      jtag=1; shift ;;
    -h|--help)   sed -n 2,26p "$0"; exit 0 ;;
    *)           echo "unknown option $1"; exit 2 ;;
  esac
done

SRC="$SFAP_ROOT/Source"
mkdir -p "$SFAP_BUILD" "$SFAP_LOG_DIR" "$GNSS_CRPA_IP_REPO" "$SFAP_BOOT_DIR" \
         "$SFAP_BUILD/hdl" "$SFAP_BUILD/sw"

say()  { printf '\n==== [%s] %s\n' "$(date +%H:%M:%S)" "$*"; }
die()  { printf '\n**** BUILD FAILED in stage %s: %s\n     log: %s\n' "$1" "$2" "$SFAP_LOG_DIR/$1.log"; exit 1; }
want() {
  local s="$1"
  [[ -n "$only" ]] && { [[ "$s" == "$only" ]]; return; }
  local seen=0
  for x in "${STAGES[@]}"; do
    [[ "$x" == "$from" ]] && seen=1
    [[ "$x" == "$s" ]] && { [[ $seen -eq 1 ]]; return; }
  done
  return 1
}

# ---- tools ---------------------------------------------------------------------
load_tool() {   # <command> <settings script>
  if ! command -v "$1" >/dev/null 2>&1 && [[ -f "$2" ]]; then
    # shellcheck disable=SC1090
    source "$2" >/dev/null 2>&1
  fi
}
load_tool vivado    "$XILINX_ROOT/Vivado/$XILINX_VER/settings64.sh"
load_tool vitis_hls "$XILINX_ROOT/Vitis_HLS/$XILINX_VER/settings64.sh"
load_tool xsct      "$XILINX_ROOT/Vitis/$XILINX_VER/settings64.sh"
load_tool bootgen   "$XILINX_ROOT/Vitis/$XILINX_VER/settings64.sh"

# ---- check ------------------------------------------------------------------------
if want check; then
  say "check"
  {
    ok=1
    for t in vivado vitis_hls xsct bootgen make; do
      if command -v "$t" >/dev/null 2>&1; then echo "tool $t: $(command -v $t)"
      else echo "tool $t: MISSING"; ok=0; fi
    done
    v=$(vivado -version 2>/dev/null | head -1)
    echo "vivado: $v (ADI expects $REQUIRED_VIVADO_VERSION)"
    for f in HLS/sfap_pi/sfap_pi.cpp HLS/sfap_pi/build_hls.tcl HDL/gnss_passthrough.v \
             HDL/system_top.v BlockDesign/system_bd.tcl XDC/system.xdc \
             XDC/gnss_passthrough_cdc.xdc IP/gnss_passthrough/gnss_passthrough_ip.tcl \
             Firmware/app_gnss_e310/main.c Firmware/app_gnss_e310/gnss_aj_math.c; do
      [[ -f "$SRC/$f" ]] && echo "source $f: ok" || { echo "source $f: MISSING"; ok=0; }
    done
    if [[ -d "$ADI_HDL_DIR/library/axi_ad9361" && -f "$ADI_HDL_DIR/scripts/adi_env.tcl" ]]; then
      echo "ADI hdl: $ADI_HDL_DIR"
    else
      echo "ADI hdl: MISSING at $ADI_HDL_DIR -- run ./setup_vendor.sh"; ok=0
    fi
    [[ $ok -eq 1 ]]
  } 2>&1 | tee "$SFAP_LOG_DIR/check.log"
  [[ ${PIPESTATUS[0]} -eq 0 ]] || die check "missing tool, source or vendor tree"
  echo "CHECK: PASS"
fi

# ---- hls ----------------------------------------------------------------------------
if want hls; then
  say "hls: Vitis HLS sfap_pi (csim $([[ $skip_csim -eq 1 ]] && echo skipped || echo '9 scenarios'), csynth, export)"
  ( cd "$SRC/HLS/sfap_pi" && SFAP_PI_SKIP_CSIM=$skip_csim vitis_hls -f build_hls.tcl ) \
      > "$SFAP_LOG_DIR/hls.log" 2>&1
  grep -E "SFAP_PI_CSIM|SFAP_PI_HLS|suppression|bypass|error" "$SFAP_LOG_DIR/hls.log" | grep -v "^INFO" | tail -40
  grep -q "SFAP_PI_HLS: PASS" "$SFAP_LOG_DIR/hls.log" || die hls "see SFAP_PI_HLS / SFAP_PI_CSIM lines"
  grep -q "SFAP_PI_CSIM: FAIL" "$SFAP_LOG_DIR/hls.log" && die hls "a C simulation scenario failed"
  echo "HLS: PASS -> $GNSS_CRPA_IP_REPO/sfap_pi"
fi

# ---- ip ---------------------------------------------------------------------------------
if want ip; then
  say "ip: package gnss_passthrough"
  ( cd "$SFAP_BUILD" && vivado -mode batch -nojournal -nolog \
      -source "$SRC/IP/gnss_passthrough/gnss_passthrough_ip.tcl" \
      -tclargs "$SRC" "$GNSS_CRPA_IP_REPO" ) > "$SFAP_LOG_DIR/ip.log" 2>&1
  grep -E "IP_PACKAGE|ERROR" "$SFAP_LOG_DIR/ip.log" | tail -10
  grep -q "IP_PACKAGE_OK" "$SFAP_LOG_DIR/ip.log" || die ip "IP packaging"
  echo "IP: PASS -> $GNSS_CRPA_IP_REPO/gnss_passthrough"
fi

# ---- hdl --------------------------------------------------------------------------------
if want hdl; then
  say "hdl: ADI libraries + Vivado project in $SFAP_HDL_PRJ_DIR (30-90 min)"
  [[ -f "$GNSS_CRPA_IP_REPO/sfap_pi/component.xml" ]] || die hdl "sfap_pi IP missing: run --from hls"
  [[ -f "$GNSS_CRPA_IP_REPO/gnss_passthrough/component.xml" ]] || die hdl "gnss_passthrough IP missing: run --from ip"
  rm -rf "$SFAP_HDL_PRJ_DIR"
  mkdir -p "$SFAP_HDL_PRJ_DIR"
  cp "$here/hdl_project/system_project.tcl" "$here/hdl_project/Makefile" "$SFAP_HDL_PRJ_DIR/"
  cp "$SRC/BlockDesign/system_bd.tcl" "$SRC/HDL/system_top.v" \
     "$SRC/XDC/system.xdc" "$SRC/XDC/gnss_passthrough_cdc.xdc" "$SFAP_HDL_PRJ_DIR/"
  sed -i "s/^PROJECT_NAME := .*/PROJECT_NAME := $SFAP_HDL_PRJ_NAME/" "$SFAP_HDL_PRJ_DIR/Makefile"
  ( cd "$SFAP_HDL_PRJ_DIR" && make -j"$SFAP_JOBS" ) > "$SFAP_LOG_DIR/hdl.log" 2>&1
  rc=$?
  plog="$SFAP_HDL_PRJ_DIR/${SFAP_HDL_PRJ_NAME}_vivado.log"
  [[ -f "$plog" ]] && cp "$plog" "$SFAP_LOG_DIR/hdl_vivado.log"
  tail -15 "$SFAP_LOG_DIR/hdl.log"
  xsa="$SFAP_HDL_PRJ_DIR/${SFAP_HDL_PRJ_NAME}.sdk/system_top.xsa"
  bit="$SFAP_HDL_PRJ_DIR/${SFAP_HDL_PRJ_NAME}.runs/impl_1/system_top.bit"
  if [[ -f "$SFAP_HDL_PRJ_DIR/${SFAP_HDL_PRJ_NAME}.sdk/system_top_bad_timing.xsa" ]]; then
    grep -E "WNS|TNS|VIOLATED" "$SFAP_LOG_DIR/hdl_vivado.log" | tail -5
    die hdl "timing NOT met (see timing_impl.log in $SFAP_HDL_PRJ_DIR)"
  fi
  [[ $rc -eq 0 && -f "$xsa" && -f "$bit" ]] || die hdl "make failed (also check $SFAP_LOG_DIR/hdl_vivado.log)"
  cp "$xsa" "$bit" "$SFAP_BUILD/hdl/"
  rpt="$SFAP_HDL_PRJ_DIR/${SFAP_HDL_PRJ_NAME}.runs/impl_1/system_top_utilization_placed.rpt"
  [[ -f "$rpt" ]] && grep -E "^\| (Slice LUTs|Slice Registers|Block RAM Tile|DSPs) " "$rpt" | head -4
  echo "HDL: PASS -> $SFAP_BUILD/hdl/system_top.{bit,xsa} (timing met)"
fi

# ---- sw ---------------------------------------------------------------------------------
if want sw; then
  say "sw: XSCT platform + FSBL + app_gnss_e310"
  xsa="$SFAP_BUILD/hdl/system_top.xsa"
  [[ -f "$xsa" ]] || die sw "no XSA: run --from hdl"
  xsct "$here/xsct_build_sw.tcl" "$SFAP_SW_WS" "$xsa" "$SRC/Firmware/app_gnss_e310" \
      > "$SFAP_LOG_DIR/sw.log" 2>&1
  grep -E "SFAP_SW|error:|undefined reference" "$SFAP_LOG_DIR/sw.log" | tail -20
  elf=$(grep -o "SFAP_SW: PASS .*" "$SFAP_LOG_DIR/sw.log" | awk '{print $3}')
  fsbl=$(grep -o "SFAP_SW: FSBL .*" "$SFAP_LOG_DIR/sw.log" | awk '{print $3}')
  [[ -n "$elf" && -f "$elf" && -n "$fsbl" && -f "$fsbl" ]] || die sw "software build"
  cp "$elf" "$SFAP_BUILD/sw/app_gnss_e310.elf"
  cp "$fsbl" "$SFAP_BUILD/sw/fsbl.elf"
  init=$(find "$SFAP_SW_WS/e310_sfap_plat" -name ps7_init.tcl | head -1)
  [[ -n "$init" ]] && cp "$init" "$SFAP_BUILD/sw/"
  echo "SW: PASS -> $SFAP_BUILD/sw/{fsbl,app_gnss_e310}.elf"
fi

# ---- boot -------------------------------------------------------------------------------
if want boot; then
  say "boot: bootgen BOOT.BIN"
  for f in "$SFAP_BUILD/sw/fsbl.elf" "$SFAP_BUILD/hdl/system_top.bit" "$SFAP_BUILD/sw/app_gnss_e310.elf"; do
    [[ -f "$f" ]] || die boot "missing $f"
  done
  cat > "$SFAP_BOOT_DIR/boot.bif" <<BIF
the_ROM_image:
{
  [bootloader] $SFAP_BUILD/sw/fsbl.elf
  $SFAP_BUILD/hdl/system_top.bit
  $SFAP_BUILD/sw/app_gnss_e310.elf
}
BIF
  bootgen -arch zynq -image "$SFAP_BOOT_DIR/boot.bif" -o "$SFAP_BOOT_DIR/BOOT.BIN" -w on \
      > "$SFAP_LOG_DIR/boot.log" 2>&1
  [[ -f "$SFAP_BOOT_DIR/BOOT.BIN" ]] || die boot "bootgen"
  ( cd "$SFAP_BOOT_DIR" && sha256sum BOOT.BIN > BOOT.BIN.sha256 )
  ls -l "$SFAP_BOOT_DIR/BOOT.BIN"
  echo "BOOT: PASS -> $SFAP_BOOT_DIR/BOOT.BIN  (copy to a FAT32 SD card, boot jumpers on SD)"
fi

# ---- optional JTAG run ------------------------------------------------------------------
if [[ $jtag -eq 1 ]]; then
  say "jtag: program + run"
  xsct "$here/xsct_jtag_run.tcl" "$SFAP_BUILD/hdl/system_top.bit" \
       "$SFAP_BUILD/sw/ps7_init.tcl" "$SFAP_BUILD/sw/app_gnss_e310.elf" 2>&1 | tee "$SFAP_LOG_DIR/jtag.log"
fi

say "done"
