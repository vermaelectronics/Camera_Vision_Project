#!/usr/bin/env bash
# ============================================================================
#  build_all.sh  --  ANTSDR E310 Passthrough_LVDS + CRPA space-time anti-jam:
#                    sources -> BOOT.BIN, ONE CRPA ALGORITHM PER BUILD, with
#                    Vivado / Vitis / Vitis HLS 2023.2
#
#  ALGORITHMS (Automation/Linux/crpa_algos.sh; sources Source/CRPA/<algo>)
#    crpa1   smi_pi   6 + 6 taps   SMI solver
#    crpa27  stap_pi  8 + 8 taps   direct start + NLMS + moving-jammer tracking
#    crpa28  stap_pi 10 + 10       and weight prediction (same code, more taps)
#    crpa29  stap_pi 14 + 14
#    crpa30  stap_pi 16 + 16
#    crpa31  stap_pi 18 + 18
#
#  STAGES (in order), all outputs in Build/<algo>/
#    check  tools on PATH, ADI hdl tree present, the algorithm's sources, and
#           that their tap count / reference delay match crpa_algos.sh
#    hls    Vitis HLS: C simulation (6 scenarios, the core with the ARM
#           solver) + synthesis + IP export of the core -> ip_repo/<core>
#    ip     Vivado: package gnss_passthrough with the algorithm's sizes
#                                               -> ip_repo/gnss_passthrough
#    hdl    ADI make in <hdl>/projects/passthrough_lvds_<algo>: library IPs +
#           Vivado project, synthesis, implementation, bitstream, XSA
#           (fails on a timing violation)       -> hdl/system_top.{bit,xsa}
#    sw     firmware staged (Source/Firmware/app_crpa_lvds + the algorithm's
#           gnss_aj_math.{c,h} + crpa_build.h) -> fw_src/, then XSCT:
#           platform, FSBL, standalone BSP, app  -> sw/{fsbl,app_crpa_lvds}.elf
#    boot   bootgen: FSBL + bitstream + app     -> boot/BOOT.BIN
#
#  USAGE
#    ./build_all.sh --algo crpa27                  everything for crpa27
#    ./build_all.sh --algo all                     every algorithm, one after the other
#    ./build_all.sh --algo crpa27 --from hdl       this stage and the ones after it
#    ./build_all.sh --algo crpa27 --only sw        one stage
#    ./build_all.sh --algo crpa27 --skip-csim      HLS without the C simulation
#    ./build_all.sh --algo crpa27 --jtag           after the build, run it over JTAG
#    ./build_all.sh --list                         the algorithms
#
#  Each stage logs to Build/<algo>/logs/<stage>.log and prints one PASS/FAIL line.
# ============================================================================
set -uo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$here/env.sh"
source "$here/crpa_algos.sh"

STAGES=(check hls ip hdl sw boot)
algo=""; from="check"; only=""; skip_csim=0; jtag=0; pass_args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --algo)      algo="$2"; shift 2 ;;
    --from)      from="$2"; pass_args+=(--from "$2"); shift 2 ;;
    --only)      only="$2"; pass_args+=(--only "$2"); shift 2 ;;
    --skip-csim) skip_csim=1; pass_args+=(--skip-csim); shift ;;
    --jtag)      jtag=1; shift ;;
    --list)      printf '%s\n' "${CRPA_ALGOS[@]}"; exit 0 ;;
    -h|--help)   sed -n 2,42p "$0"; exit 0 ;;
    *)           echo "unknown option $1 (see --help)"; exit 2 ;;
  esac
done
[[ -n "$algo" ]] || { echo "build_all.sh: --algo is required: one of ${CRPA_ALGOS[*]} or all"; exit 2; }

# ---- all algorithms: one full build each, a summary at the end ------------------
if [[ "$algo" == all ]]; then
  [[ $jtag -eq 1 ]] && { echo "--jtag needs one algorithm"; exit 2; }
  declare -A result
  for a in "${CRPA_ALGOS[@]}"; do
    "$here/build_all.sh" --algo "$a" "${pass_args[@]}" && result[$a]=PASS || result[$a]=FAIL
  done
  echo
  echo "==== summary"
  rc=0
  for a in "${CRPA_ALGOS[@]}"; do
    printf '  %-7s %s  %s\n' "$a" "${result[$a]}" "$CRPA_BUILD_ROOT/$a/boot/BOOT.BIN"
    [[ "${result[$a]}" == PASS ]] || rc=1
  done
  exit $rc
fi

crpa_algo_params "$algo" || { echo "build_all.sh: unknown algorithm '$algo': one of ${CRPA_ALGOS[*]} or all"; exit 2; }
crpa_env_for_algo "$algo"

SRC="$CRPA_ROOT/Source"
ALG="$SRC/CRPA/$algo"
mkdir -p "$CRPA_BUILD" "$CRPA_LOG_DIR" "$GNSS_CRPA_IP_REPO" "$CRPA_BOOT_DIR" \
         "$CRPA_BUILD/hdl" "$CRPA_BUILD/sw"

say()  { printf '\n==== [%s] %s: %s\n' "$(date +%H:%M:%S)" "$algo" "$*"; }
die()  { printf '\n**** BUILD FAILED (%s) in stage %s: %s\n     log: %s\n' "$algo" "$1" "$2" "$CRPA_LOG_DIR/$1.log"; exit 1; }
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

# value of '#define NAME value' in a header (first match)
hdef() { grep -E "^#define[[:space:]]+$2[[:space:]]" "$1" | head -1 | awk '{print $3}'; }

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

say "CRPA algorithm $algo: core $CRPA_CORE $CRPA_CORE_VER, $CRPA_NT + $CRPA_NT taps, reference delay $CRPA_REF, AJ_INFO $CRPA_AJ_INFO"

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
    echo "vivado: $v (this design: $XILINX_VER)"
    echo "$v" | grep -q "$XILINX_VER" || echo "WARNING: Vivado is not $XILINX_VER; the ADI version check will stop the hdl stage (ADI_IGNORE_VERSION_CHECK=1 to override)"
    for f in CRPA/$algo/fpga/$CRPA_CORE.cpp CRPA/$algo/fpga/$CRPA_CORE.h \
             CRPA/$algo/arm/gnss_aj_math.c CRPA/$algo/arm/gnss_aj_math.h \
             HLS/build_hls.tcl HLS/testbench.cpp HDL/gnss_passthrough.v \
             HDL/system_top.v BlockDesign/system_bd.tcl XDC/system_constr.xdc \
             XDC/gnss_passthrough_cdc.xdc IP/gnss_passthrough/gnss_passthrough_ip.tcl \
             Firmware/$CRPA_APP_NAME/main.c Firmware/$CRPA_APP_NAME/gnss_aj.c \
             Firmware/$CRPA_APP_NAME/e310_lvds_board.c; do
      [[ -f "$SRC/$f" ]] && echo "source $f: ok" || { echo "source $f: MISSING"; ok=0; }
    done
    # the algorithm's own sizes must match the table (core and ARM solver)
    if [[ "$CRPA_CORE" == smi_pi ]]; then cnt=SMI_NT; cref=SMI_REF; else cnt=ST_NT; cref=ST_REF; fi
    for pair in "$ALG/fpga/$CRPA_CORE.h:$cnt:$CRPA_NT" "$ALG/fpga/$CRPA_CORE.h:$cref:$CRPA_REF" \
                "$ALG/arm/gnss_aj_math.h:AJ_NT:$CRPA_NT" "$ALG/arm/gnss_aj_math.h:AJ_REF:$CRPA_REF"; do
      IFS=: read -r hf name exp <<<"$pair"
      got=$(hdef "$hf" "$name")
      if [[ "$got" == "$exp" ]]; then echo "$algo: $name = $got ($(basename "$hf")): ok"
      else echo "$algo: $name = '$got' in $hf, crpa_algos.sh says $exp: MISMATCH"; ok=0; fi
    done
    if [[ -d "$ADI_HDL_DIR/library/axi_ad9361" && -f "$ADI_HDL_DIR/scripts/adi_env.tcl" \
          && -f "$ADI_HDL_DIR/projects/scripts/project-xilinx.mk" ]]; then
      echo "ADI hdl: $ADI_HDL_DIR"
    else
      echo "ADI hdl: NOT FOUND at $ADI_HDL_DIR"
      echo "         set ADI_HDL_DIR to <...>/antsdr-fw-patch/plutosdr-fw/hdl (the tree"
      echo "         that holds projects/passthrough_lvds), in env.sh or on the command line"; ok=0
    fi
    if [[ -d "$ADI_HDL_DIR/library" ]]; then
      if grep -rqs "axi_vcxo_ctrl" --include=component.xml "$ADI_HDL_DIR/library"; then
        echo "ADI hdl: axi_vcxo_ctrl IP found"
      else
        echo "ADI hdl: axi_vcxo_ctrl IP (user.org:user:axi_vcxo_ctrl:1.0) NOT found in $ADI_HDL_DIR/library"; ok=0
      fi
      [[ -d "$ADI_HDL_DIR/projects/passthrough_lvds" ]] && echo "ADI hdl: projects/passthrough_lvds present (reference, not modified)"
    fi
    [[ $ok -eq 1 ]]
  } 2>&1 | tee "$CRPA_LOG_DIR/check.log"
  [[ ${PIPESTATUS[0]} -eq 0 ]] || die check "missing tool, source, vendor tree or a size mismatch"
  echo "CHECK: PASS"
fi

# ---- hls ----------------------------------------------------------------------------
if want hls; then
  say "hls: Vitis HLS $CRPA_CORE (csim $([[ $skip_csim -eq 1 ]] && echo skipped || echo '6 scenarios'), csynth, export)"
  ( cd "$SRC/HLS" && CRPA_SKIP_CSIM=$skip_csim vitis_hls -f build_hls.tcl ) \
      > "$CRPA_LOG_DIR/hls.log" 2>&1
  grep -E "CRPA_CSIM|CRPA_HLS|^scenario|error" "$CRPA_LOG_DIR/hls.log" | grep -v "^INFO" | tail -40
  grep -q "CRPA_HLS: PASS" "$CRPA_LOG_DIR/hls.log" || die hls "see CRPA_HLS / CRPA_CSIM lines"
  grep -q "CRPA_CSIM: FAIL" "$CRPA_LOG_DIR/hls.log" && die hls "a C simulation scenario failed"
  echo "HLS: PASS -> $GNSS_CRPA_IP_REPO/$CRPA_CORE"
fi

# ---- ip ---------------------------------------------------------------------------------
if want ip; then
  say "ip: package gnss_passthrough for $algo"
  ( cd "$CRPA_BUILD" && vivado -mode batch -nojournal -nolog \
      -source "$SRC/IP/gnss_passthrough/gnss_passthrough_ip.tcl" \
      -tclargs "$SRC" "$GNSS_CRPA_IP_REPO" "$CRPA_NUM" "$CRPA_NT" "$CRPA_REF" \
               "$CRPA_NWTS" "$CRPA_NW_USED" "$CRPA_NWCJ" ) > "$CRPA_LOG_DIR/ip.log" 2>&1
  grep -E "IP_PACKAGE|ERROR" "$CRPA_LOG_DIR/ip.log" | tail -10
  grep -q "IP_PACKAGE_OK" "$CRPA_LOG_DIR/ip.log" || die ip "IP packaging"
  echo "IP: PASS -> $GNSS_CRPA_IP_REPO/gnss_passthrough"
fi

# ---- hdl --------------------------------------------------------------------------------
if want hdl; then
  say "hdl: ADI libraries + Vivado project in $CRPA_HDL_PRJ_DIR (30-90 min)"
  [[ -f "$GNSS_CRPA_IP_REPO/$CRPA_CORE/component.xml" ]] || die hdl "$CRPA_CORE IP missing: run --algo $algo --from hls"
  [[ -f "$GNSS_CRPA_IP_REPO/gnss_passthrough/component.xml" ]] || die hdl "gnss_passthrough IP missing: run --algo $algo --from ip"
  case "$(basename "$CRPA_HDL_PRJ_DIR")" in passthrough_lvds|passthrough_lvds_sfap|passthrough_lvds_sfap256) die hdl "refusing to overwrite projects/$(basename "$CRPA_HDL_PRJ_DIR") (another build)" ;; esac
  rm -rf "$CRPA_HDL_PRJ_DIR"
  mkdir -p "$CRPA_HDL_PRJ_DIR"
  cp "$here/hdl_project/system_project.tcl" "$here/hdl_project/Makefile" "$CRPA_HDL_PRJ_DIR/"
  cp "$SRC/BlockDesign/system_bd.tcl" "$SRC/HDL/system_top.v" \
     "$SRC/XDC/system_constr.xdc" "$SRC/XDC/gnss_passthrough_cdc.xdc" "$CRPA_HDL_PRJ_DIR/"
  sed -i "s/^PROJECT_NAME := .*/PROJECT_NAME := $CRPA_HDL_PRJ_NAME/" "$CRPA_HDL_PRJ_DIR/Makefile"
  ( cd "$CRPA_HDL_PRJ_DIR" && make -j"$CRPA_JOBS" ) > "$CRPA_LOG_DIR/hdl.log" 2>&1
  rc=$?
  plog="$CRPA_HDL_PRJ_DIR/${CRPA_HDL_PRJ_NAME}_vivado.log"
  [[ -f "$plog" ]] && cp "$plog" "$CRPA_LOG_DIR/hdl_vivado.log"
  tail -15 "$CRPA_LOG_DIR/hdl.log"
  xsa="$CRPA_HDL_PRJ_DIR/${CRPA_HDL_PRJ_NAME}.sdk/system_top.xsa"
  bit="$CRPA_HDL_PRJ_DIR/${CRPA_HDL_PRJ_NAME}.runs/impl_1/system_top.bit"
  if [[ -f "$CRPA_HDL_PRJ_DIR/${CRPA_HDL_PRJ_NAME}.sdk/system_top_bad_timing.xsa" ]] || \
     { [[ -f "$CRPA_LOG_DIR/hdl_vivado.log" ]] && grep -v "^#" "$CRPA_LOG_DIR/hdl_vivado.log" | grep -qi "timing constraints.*not met"; }; then
    grep -E "WNS|TNS|VIOLATED" "$CRPA_LOG_DIR/hdl_vivado.log" | tail -5
    die hdl "timing NOT met (see timing_impl.log in $CRPA_HDL_PRJ_DIR)"
  fi
  [[ $rc -eq 0 && -f "$xsa" && -f "$bit" ]] || die hdl "make failed (also check $CRPA_LOG_DIR/hdl_vivado.log)"
  # Worst setup / hold slack from the implemented design's timing summary.
  tlog="$CRPA_HDL_PRJ_DIR/timing_impl.log"
  if [[ -f "$tlog" ]]; then
    read -r wns whs < <(awk '/WNS\(ns\)/ {getline; getline; print $1, $5; exit}' "$tlog")
    echo "HDL: timing WNS ${wns:-?} ns, WHS ${whs:-?} ns"
    if [[ "${wns:-0}" == -* || "${whs:-0}" == -* ]]; then
      die hdl "timing NOT met (WNS $wns ns, WHS $whs ns; see $tlog)"
    fi
  else
    echo "HDL: WARNING - $tlog not found, timing could not be confirmed"
  fi
  cp "$xsa" "$bit" "$CRPA_BUILD/hdl/"
  rpt="$CRPA_HDL_PRJ_DIR/${CRPA_HDL_PRJ_NAME}.runs/impl_1/system_top_utilization_placed.rpt"
  [[ -f "$rpt" ]] && grep -E "^\| (Slice LUTs|Slice Registers|Block RAM Tile|DSPs) " "$rpt" | head -4
  echo "HDL: PASS -> $CRPA_BUILD/hdl/system_top.{bit,xsa} (timing met)"
fi

# ---- sw ---------------------------------------------------------------------------------
if want sw; then
  say "sw: stage firmware + XSCT platform + FSBL + $CRPA_APP_NAME"
  xsa="$CRPA_BUILD/hdl/system_top.xsa"
  [[ -f "$xsa" ]] || die sw "no XSA: run --algo $algo --from hdl"
  # The firmware = the common engine (Source/Firmware/app_crpa_lvds) + this
  # algorithm's solver (Source/CRPA/<algo>/arm) + crpa_build.h, in one folder.
  rm -rf "$CRPA_FW_STAGE"; mkdir -p "$CRPA_FW_STAGE"
  cp "$SRC/Firmware/$CRPA_APP_NAME/"* "$CRPA_FW_STAGE/"
  cp "$ALG/arm/gnss_aj_math.c" "$ALG/arm/gnss_aj_math.h" "$CRPA_FW_STAGE/"
  cat > "$CRPA_FW_STAGE/crpa_build.h" <<HDR
/* Generated by build_all.sh --algo $algo. Do not edit. */
#ifndef CRPA_BUILD_H_
#define CRPA_BUILD_H_
#define CRPA_ALGO_NUM   $CRPA_NUM
#define CRPA_ALGO_STR   "$algo"
#define CRPA_CORE_STR   "$CRPA_CORE"
#define CRPA_AJ_INFO    ${CRPA_AJ_INFO}U
#endif
HDR
  xsct "$here/xsct_build_sw.tcl" "$CRPA_SW_WS" "$xsa" "$CRPA_FW_STAGE" "$CRPA_APP_NAME" \
      > "$CRPA_LOG_DIR/sw.log" 2>&1
  grep -E "CRPA_SW|error:|undefined reference" "$CRPA_LOG_DIR/sw.log" | tail -20
  elf=$(grep -o "CRPA_SW: PASS .*" "$CRPA_LOG_DIR/sw.log" | awk '{print $3}')
  fsbl=$(grep -o "CRPA_SW: FSBL .*" "$CRPA_LOG_DIR/sw.log" | awk '{print $3}')
  [[ -n "$elf" && -f "$elf" && -n "$fsbl" && -f "$fsbl" ]] || die sw "software build"
  cp "$elf" "$CRPA_BUILD/sw/$CRPA_APP_NAME.elf"
  cp "$fsbl" "$CRPA_BUILD/sw/fsbl.elf"
  init=$(find "$CRPA_SW_WS/e310_lvds_plat" -name ps7_init.tcl | head -1)
  [[ -n "$init" ]] && cp "$init" "$CRPA_BUILD/sw/"
  echo "SW: PASS -> $CRPA_BUILD/sw/{fsbl,$CRPA_APP_NAME}.elf"
fi

# ---- boot -------------------------------------------------------------------------------
if want boot; then
  say "boot: bootgen BOOT.BIN"
  for f in "$CRPA_BUILD/sw/fsbl.elf" "$CRPA_BUILD/hdl/system_top.bit" "$CRPA_BUILD/sw/$CRPA_APP_NAME.elf"; do
    [[ -f "$f" ]] || die boot "missing $f"
  done
  cat > "$CRPA_BOOT_DIR/boot.bif" <<BIF
the_ROM_image:
{
  [bootloader] $CRPA_BUILD/sw/fsbl.elf
  $CRPA_BUILD/hdl/system_top.bit
  $CRPA_BUILD/sw/$CRPA_APP_NAME.elf
}
BIF
  bootgen -arch zynq -image "$CRPA_BOOT_DIR/boot.bif" -o "$CRPA_BOOT_DIR/BOOT.BIN" -w on \
      > "$CRPA_LOG_DIR/boot.log" 2>&1
  [[ -f "$CRPA_BOOT_DIR/BOOT.BIN" ]] || die boot "bootgen"
  ( cd "$CRPA_BOOT_DIR" && sha256sum BOOT.BIN > BOOT.BIN.sha256 )
  ls -l "$CRPA_BOOT_DIR/BOOT.BIN"
  echo "BOOT: PASS -> $CRPA_BOOT_DIR/BOOT.BIN  ($algo; copy to a FAT32 SD card, boot jumpers on SD)"
fi

# ---- optional JTAG run ------------------------------------------------------------------
if [[ $jtag -eq 1 ]]; then
  say "jtag: program + run"
  xsct "$here/xsct_jtag_run.tcl" "$CRPA_BUILD/hdl/system_top.bit" \
       "$CRPA_BUILD/sw/ps7_init.tcl" "$CRPA_BUILD/sw/$CRPA_APP_NAME.elf" 2>&1 | tee "$CRPA_LOG_DIR/jtag.log"
fi

say "done"
