#!/usr/bin/env bash
# ============================================================================
#  build_report.sh  --  what passed, what failed, and what it means
#
#  Reads the logs and reports that build_all.sh leaves in Build/ and prints,
#  for every stage, PASS / FAIL / NOT RUN plus what the result means for the
#  design on the board: HLS core timing and II, Vivado timing (WNS/TNS/WHS,
#  per clock), where the failing paths are, resource use, and a final verdict.
#
#  USAGE (from the design folder, any terminal)
#    ./Automation/Linux/build_report.sh            summary from the logs (seconds)
#    ./Automation/Linux/build_report.sh --paths    also opens the routed design in
#                                                  Vivado (1-2 min) and lists EVERY
#                                                  failing path, grouped by block
#
#  The report is also written to Build/Logs/build_report.txt.
# ============================================================================

set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG="$ROOT/Build/Logs"
OUT="$ROOT/Build/Output"
ELF="$ROOT/Build/ELF"
IPR="$ROOT/Build/ip_repo"
DEEP=0
[[ "${1:-}" == "--paths" ]] && DEEP=1

# ---- which design is this? --------------------------------------------------
if   [[ -d "$ROOT/Source/HLS/smi_pi"  ]]; then CORE=smi_pi;  CELL=smi_pi_0;  STAGE=smi;  MARK="SMI_PI_HLS";  NAME="SMI-PI (closed-form power inversion)"
elif [[ -d "$ROOT/Source/HLS/pl_npi"  ]]; then CORE=pl_npi;  CELL=pl_npi_0;  STAGE=npi;  MARK="PL_NPI_HLS";  NAME="PL-NPI (pipelined HLS)"
elif [[ -d "$ROOT/Source/HLS/pi_nlms" ]]; then CORE=pi_nlms; CELL=pi_nlms_0; STAGE=nlms; MARK="PI_NLMS_RTL"; NAME="PI-NLMS"
else CORE=""; CELL="__none__"; STAGE=""; MARK=""; NAME="unknown"; fi

mkdir -p "$LOG"
REPORT="$LOG/build_report.txt"

main() {

VERDICT_BAD=()      # reasons the bitstream must not be used
VERDICT_WARN=()     # reasons to be careful

hr()  { printf '%s\n' "------------------------------------------------------------------------------"; }
sec() { echo; hr; echo " $*"; hr; }
has() { [[ -f "$1" ]] && grep -q "$2" "$1" 2>/dev/null; }

echo "BUILD REPORT  $(date '+%Y-%m-%d %H:%M')"
echo "design folder : $ROOT"
echo "processing    : $NAME  (block $CELL)"

# ---- 1. stage table --------------------------------------------------------
sec "1. STAGES"
printf ' %-8s %-8s %s\n' "stage" "result" "meaning if not PASS"
stage() {   # name log marker artifact meaning
  local name="$1" log="$LOG/$2.log" marker="$3" art="$4" why="$5" res
  if [[ ! -f "$log" ]] && [[ -z "$art" || ! -e "$art" ]]; then
    res="NOT RUN"
  elif [[ -n "$marker" ]] && has "$log" "$marker"; then
    res="PASS"
  elif [[ -z "$marker" && -n "$art" && -e "$art" ]]; then
    res="PASS"
  elif [[ -n "$art" && -e "$art" && ! -f "$log" ]]; then
    res="PASS?"          # artifact present, log missing (built elsewhere)
  else
    res="FAIL"
  fi
  if [[ "$res" == PASS* ]]; then
    printf ' %-8s %-8s %s\n' "$name" "$res" ""
  else
    printf ' %-8s %-8s %s\n' "$name" "$res" "$why"
    VERDICT_BAD+=("stage $name: $res - $why")
  fi
}
stage vendor  vendor  ""                  "$ROOT/Build/VendorWork/hdl/library" "no ADI library copy; nothing else can build"
libip_ok="$ROOT/Build/VendorWork/hdl/library/axi_ad9361/component.xml"
stage libip   libip_axi_ad9361 ""         "$libip_ok"                          "ADI IPs (AD9361 interface, DMA) not packaged; block design cannot be created"
[[ -n "$STAGE" ]] && stage "$STAGE" "$STAGE" "$MARK: PASS" "$IPR/$CORE/component.xml" \
                                          "anti-jam core IP missing; the block design will not build (or uses an OLD core)"
stage ip      ip      "IP_PACKAGE_OK"     "$IPR/gnss_passthrough/component.xml" "gnss_passthrough IP missing; no RX->TX datapath"
stage project project "CREATE_RESULT: PASS" "$ROOT/Vivado/Project/antsdr_e310_gnss.xpr" "no Vivado project / block design"
stage build   build   "BUILD_RESULT: PASS" "$OUT/system_top.bit"              "no bitstream / XSA; nothing to load on the FPGA"
stage sw      sw      "SW_RESULT: PASS"   "$ELF/e310_gnss_app.elf"            "no application: no console commands, AD9361 not configured"
stage fsbl    fsbl    "FSBL_RESULT: PASS" "$ELF/fsbl.elf"                     "no boot loader; BOOT.BIN cannot be made"
stage boot    boot    ""                  "$OUT/BOOT.BIN"                     "no BOOT.BIN for the SD card"
if has "$LOG/build.log" "BUILD_TIMING: VIOLATED"; then
  echo " note: stage build made a bitstream, but BUILD_TIMING: VIOLATED (section 3)"
elif has "$LOG/build.log" "BUILD_TIMING: MET"; then
  echo " note: stage build reports BUILD_TIMING: MET"
fi

# Stale-output check: an old core IP or old bitstream silently re-used.
if [[ -f "$OUT/BOOT.BIN" && -f "$OUT/system_top.bit" && "$OUT/system_top.bit" -nt "$OUT/BOOT.BIN" ]]; then
  VERDICT_WARN+=("BOOT.BIN is OLDER than system_top.bit: re-run stage boot so the SD card gets the new bitstream")
fi
for e in "$ELF/e310_gnss_app.elf" "$ELF/fsbl.elf"; do
  if [[ -f "$OUT/BOOT.BIN" && ( ! -f "$e" || "$e" -nt "$OUT/BOOT.BIN" ) ]]; then
    VERDICT_WARN+=("BOOT.BIN does not match $(basename "$e") (missing or newer): it is an OLD image; re-run stages sw/fsbl/boot")
  fi
done
if [[ -n "$CORE" && -f "$IPR/$CORE/component.xml" && -f "$OUT/system_top.bit" && "$IPR/$CORE/component.xml" -nt "$OUT/system_top.bit" ]]; then
  VERDICT_WARN+=("the $CORE IP is NEWER than the bitstream: re-run from stage project so the bitstream contains it")
fi

# ---- 2. HLS core -----------------------------------------------------------
if [[ -n "$STAGE" ]]; then
  sec "2. ANTI-JAM CORE ($CORE, Vitis HLS)"
  L="$LOG/$STAGE.log"
  if [[ -f "$L" ]]; then
    echo " C simulation (algorithm check):"
    grep -E "_CSIM: (PASS|FAIL)|^FAIL:|null|SINR|output .* vs|jammer .* LSB|gamma" "$L" | grep -v "^INFO" | sed 's/^/   /' | head -20
    ncs_fail=$(grep -c "_CSIM: FAIL" "$L" || true)
    if [[ "$ncs_fail" -gt 0 ]]; then
      VERDICT_BAD+=("C simulation failed $ncs_fail scenario(s): the algorithm does not meet its own test")
    fi
    echo
    echo " Synthesis:"
    grep -E "$MARK: (HLS clock|estimated)|Pipelining result|Estimated Fmax" "$L" | sed 's/^/   /'
    est=$(grep -oE "estimated clock period [0-9.]+" "$L" | grep -oE "[0-9.]+$" | tail -1)
    ii=$(grep -oE "achieved II [0-9]+" "$L" | grep -oE "[0-9]+$" | tail -1)
    if [[ -n "$est" ]]; then
      echo "   -> HLS estimate $est ns for an 8 ns clock: margin $(awk -v e="$est" 'BEGIN{printf "%.2f", 8-e}') ns."
      echo "      This is an estimate before place-and-route; the real answer is the"
      echo "      Vivado timing in section 3 (paths inside $CELL)."
    fi
    if [[ -n "$ii" && "$ii" -gt 2 ]]; then
      VERDICT_BAD+=("HLS II=$ii > 2: the core cannot take every AD9361 sample pair; pairs are dropped (STATUS[10])")
    fi
  else
    echo " no $STAGE.log: stage not run in this folder."
  fi
  RPT="$IPR/$CORE/${CORE}_csynth.rpt"
  if [[ -f "$RPT" ]]; then
    echo
    echo " Core resources (HLS estimate):"
    awk '/== Utilization Estimates/{f=1} f&&/^\|Name|^\|Total|^\|Available|^\|Utilization/{print "   "$0} /== Interface/{f=0}' "$RPT" | head -6
  fi
fi

# ---- 3. Vivado timing --------------------------------------------------------
sec "3. FPGA TIMING (after place and route)"
T="$OUT/timing_impl.rpt"
if [[ ! -f "$T" ]]; then
  echo " Build/Output/timing_impl.rpt not found: stage build has not finished."
else
  read -r WNS TNS TNSF TNST WHS THS THSF THST <<<"$(awk '/WNS\(ns\) +TNS\(ns\)/{getline; getline; print $1,$2,$3,$4,$5,$6,$7,$8; exit}' "$T")"
  printf ' Setup : WNS %8s ns   TNS %10s ns   failing endpoints %s of %s\n' "$WNS" "$TNS" "$TNSF" "$TNST"
  printf ' Hold  : WHS %8s ns   THS %10s ns   failing endpoints %s of %s\n' "$WHS" "$THS" "$THSF" "$THST"
  echo
  echo " Per clock (setup):"
  awk '/^\| Intra Clock Table/{f=1;next} /^\| Inter Clock Table|^\| Other Path Groups Table|^\| Timing Details/{f=0} f&&NF>=4&&$1!~/^-/&&$1!="Clock"{printf "   %-16s WNS %9s ns   TNS %11s ns   failing %s\n",$1,$2,$3,$4}' "$T"
  awk '/^\| Inter Clock Table/{f=1;next} /^\| Other Path Groups Table|^\| User Ignored|^\| Timing Details/{f=0} f&&NF>=5&&$1!~/^-/&&$1!="From"{printf "   %-12s -> %-12s WNS %9s ns   failing %s   (clock crossing)\n",$1,$2,$3,$5}' "$T"
  echo
  setup_bad=$(awk -v w="$WNS" 'BEGIN{print (w<0)?1:0}')
  hold_bad=$(awk -v w="$WHS" 'BEGIN{print (w<0)?1:0}')
  if [[ "$hold_bad" == 1 ]]; then
    echo " HOLD VIOLATION: data changes too early. This fails at ANY clock speed and"
    echo " gives random errors on the board. Must be fixed before use."
    VERDICT_BAD+=("hold violation WHS=$WHS ns")
  fi
  if [[ "$setup_bad" == 0 ]]; then
    echo " Setup timing MET (WNS >= 0): every path finishes inside the 8 ns sample"
    echo " clock. The FPGA behaves exactly like the simulation."
  else
    echo " Setup timing NOT met. The worst path of each clock group:"
    awk '/^Slack \(VIOLATED\)/{s=$4} /^  Source:/{src=$2} /^  Destination:/{dst=$2} /^  Path Group:/{if(s!=""){printf "   slack %-10s %s\n     -> %s   [group %s]\n",s,src,dst,$3}; s=""}' "$T" | head -40
    echo
    # classify the worst path of each group by block
    worst=$(awk '/^Slack \(VIOLATED\)/{s=1} s&&/^  Destination:/{print $2; s=0}' "$T")
    in_core=0; in_pt=0; in_adi=0; other=0
    for d in $worst; do
      case "$d" in
        *"$CELL"*) in_core=1 ;;
        *gnss_passthrough*) in_pt=1 ;;
        *axi_ad9361*|*util_*|*axi_*dma*|*axi_hp*|*axi_cpu*) in_adi=1 ;;
        *) other=1 ;;
      esac
    done
    echo " What it means:"
    small=$(awk -v w="$WNS" 'BEGIN{print (w>-0.3)?1:0}')
    if [[ $in_core == 1 ]]; then
      echo "   * Failing paths inside $CELL (the anti-jam core): its arithmetic can"
      echo "     latch half-finished results, so the weights / output can be wrong."
      echo "     With the core OFF (gnss_*=0) TX1 is plain RX1 and is NOT affected."
    fi
    if [[ $in_pt == 1 ]]; then
      echo "   * Failing paths inside gnss_passthrough: the RX->TX sample path itself"
      echo "     can be corrupted even with the core off -> satellites can be lost."
    fi
    if [[ $in_adi == 1 ]]; then
      echo "   * Failing paths in the ADI AD9361 / DMA blocks: RX or TX samples or DMA"
      echo "     transfers can be corrupted -> whole RF chain unreliable."
    fi
    if [[ $other == 1 ]]; then
      echo "   * Other failing paths (see the list above); run --paths for the full list."
    fi
    if [[ "$small" == 1 ]]; then
      echo "   * WNS is small (> -0.3 ns): usually works on the bench at room temperature,"
      echo "     but there is no margin (temperature, another board). Fix: rebuild the core"
      echo "     with a tighter HLS clock, e.g. ${MARK%%_HLS}_CLK_NS=7, or an Explore strategy."
      VERDICT_WARN+=("small setup violation WNS=$WNS ns ($TNSF endpoints)")
    else
      echo "   * WNS is large (< -0.3 ns): results on the board are NOT trustworthy."
      VERDICT_BAD+=("setup violation WNS=$WNS ns, $TNSF failing endpoints")
    fi
  fi
fi

# ---- 4. every failing path (optional, needs Vivado) ---------------------------
if [[ $DEEP == 1 ]]; then
  sec "4. ALL FAILING PATHS (from the routed design)"
  DCP=$(ls -t "$ROOT"/Vivado/Project/*.runs/impl_1/*_routed.dcp 2>/dev/null | head -1)
  if [[ -z "$DCP" ]]; then
    echo " no routed checkpoint (*_routed.dcp) found: run stage build first."
  else
    if ! command -v vivado >/dev/null 2>&1; then
      for b in "${XILINX_INSTALL_DIR:-}" /tools/Xilinx /opt/Xilinx "$HOME/Xilinx"; do
        [[ -n "$b" && -f "$b/Vivado/2023.2/settings64.sh" ]] && { set +u; source "$b/Vivado/2023.2/settings64.sh" >/dev/null 2>&1; set -u; break; }
      done
    fi
    CSV="$OUT/failing_paths.csv"
    TCL="$LOG/.failing_paths.tcl"
    cat > "$TCL" <<EOF
open_checkpoint {$DCP}
set f [open {$CSV} w]
foreach p [get_timing_paths -setup -max_paths 100000 -nworst 1 -slack_lesser_than 0 -sort_by slack] {
  puts \$f "setup,[get_property SLACK \$p],[get_property GROUP \$p],[get_property STARTPOINT_PIN \$p],[get_property ENDPOINT_PIN \$p]"
}
foreach p [get_timing_paths -hold -max_paths 100000 -nworst 1 -slack_lesser_than 0 -sort_by slack] {
  puts \$f "hold,[get_property SLACK \$p],[get_property GROUP \$p],[get_property STARTPOINT_PIN \$p],[get_property ENDPOINT_PIN \$p]"
}
close \$f
report_timing -max_paths 20 -nworst 1 -slack_lesser_than 0 -sort_by slack -file {$OUT/failing_paths_top20.rpt}
EOF
    echo " opening $DCP in Vivado (1-2 min) ..."
    vivado -mode batch -nojournal -nolog -source "$TCL" >/dev/null 2>&1
    if [[ ! -s "$CSV" ]]; then
      echo " no failing paths (or Vivado could not open the checkpoint)."
    else
      echo " failing endpoints by block (setup/hold, count, worst slack ns):"
      awk -F, -v core="$CELL" '
        { b="other";
          if (index($5,core)) b=core" (anti-jam core)";
          else if ($5 ~ /gnss_passthrough/) b="gnss_passthrough (RX->TX path)";
          else if ($5 ~ /axi_ad9361/) b="axi_ad9361 (AD9361 interface)";
          else if ($5 ~ /dma|fifo|util_/) b="DMA / FIFO";
          else if ($5 ~ /interconnect|ps7|axi_/) b="AXI / processor";
          k=$1" | "b; n[k]++; if(!(k in w)||$2<w[k]) w[k]=$2 }
        END { for(k in n) printf "   %-48s %6d   %s\n", k, n[k], w[k] }' "$CSV" | sort
      echo
      echo " full list : $CSV"
      echo " top 20    : $OUT/failing_paths_top20.rpt"
    fi
  fi
fi

# ---- 5. resources -----------------------------------------------------------
U="$OUT/utilization_impl.rpt"
if [[ -f "$U" ]]; then
  sec "5. FPGA RESOURCES (xc7z020)"
  RES='^\| (Slice LUTs|Slice Registers|Block RAM Tile|DSPs)\*? +\|'
  grep -E "$RES" "$U" | head -4 | \
    awk -F'|' '{n=$2; gsub(/^ +| +$/,"",n); u=$3; a=$(NF-2); p=$(NF-1); gsub(/ /,"",u); gsub(/ /,"",a); gsub(/ /,"",p); printf "   %-18s used %8s of %8s  (%s%%)\n",n,u,a,p}'
  hi=$(grep -E "$RES" "$U" | head -4 | awk -F'|' '{v=$(NF-1)+0; if(v>80) print $2}')
  if [[ -n "$hi" ]]; then
    VERDICT_WARN+=("resource use above 80%: routing gets hard, timing may suffer")
  fi
fi

# ---- 6. verdict -------------------------------------------------------------
sec "6. VERDICT"
if [[ ${#VERDICT_BAD[@]} -eq 0 ]]; then
  if [[ ${#VERDICT_WARN[@]} -eq 0 ]]; then
    echo " READY: all stages passed and timing is met. Copy Build/Output/BOOT.BIN to the SD card."
  else
    echo " USABLE WITH CARE:"
    for w in "${VERDICT_WARN[@]}"; do echo "   - $w"; done
  fi
else
  echo " NOT READY:"
  for b in "${VERDICT_BAD[@]}"; do echo "   - $b"; done
  for w in "${VERDICT_WARN[@]}"; do echo "   - (warning) $w"; done
fi
echo
echo " On the board, confirm: console 'gnss_pt: present ... version', 'SELFTEST: PASS',"
case "$STAGE" in smi) Q="gnss_smi?";; npi) Q="gnss_npi?";; nlms) Q="gnss_nlms?";; *) Q="the core status command";; esac
echo " and that '$Q' does not report 'SAMPLES DROPPED' / a nonzero drop count."
echo " (report saved to Build/Logs/build_report.txt)"
}

main 2>&1 | tee "$REPORT"
