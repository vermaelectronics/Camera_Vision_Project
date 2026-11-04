#!/usr/bin/env bash
# ============================================================================
#  check_report.sh  --  LoopBack_STAP_CRPA2: check a finished (or failed) build
#
#  Reads Build/Logs/*.log, Build/Output/*.rpt and the outputs, and prints one
#  summary:
#    1 stage results        PASS marker of every stage that ran
#    2 address map          gnss_stap 0x43C10000 / gnss_passthrough 0x43C00000
#    3 timing               WNS/TNS/WHS/THS, per-clock table, inter-clock paths
#    4 CDC constraints      gnss_stap_cdc.xdc / gnss_passthrough_cdc.xdc applied,
#                           no "No cells matched" warnings for them
#    5 utilisation          LUT / FF / BRAM / DSP against the XC7Z020
#    6 warnings             CRITICAL WARNING / ERROR counts per log
#    7 outputs              BOOT.BIN, .bit, .xsa, ELF variant, SHA256
#  and writes the same text to Build/Logs/report_summary.txt -- send that file
#  (plus Build/Output/timing_impl.rpt if timing failed) when asking for help.
#
#  USAGE
#    ./Automation/Linux/check_report.sh
#  EXIT STATUS 0 = PASS, 1 = at least one FAIL, 2 = INCOMPLETE (no BOOT.BIN)
# ============================================================================
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG="$ROOT/Build/Logs"
OUT="$ROOT/Build/Output"
SUM="$LOG/report_summary.txt"
mkdir -p "$LOG"
: > "$SUM"

NFAIL=0; NWARN=0
p()    { echo "$*" | tee -a "$SUM"; }
ok()   { p "  [ OK ] $*"; }
bad()  { p "  [FAIL] $*"; NFAIL=$((NFAIL+1)); }
wrn()  { p "  [WARN] $*"; NWARN=$((NWARN+1)); }
none() { p "  [ -- ] $*"; }
hdr()  { p ""; p "== $* =="; }

p "LoopBack_STAP_CRPA2 build report  ($(date -Iseconds))"
p "project: $ROOT"

# ---------------------------------------------------------------- 1 stages
hdr "1 stage results"
stage() {   # name  logfile  marker
  local f="$LOG/$2"
  if [[ ! -f "$f" ]]; then none "$1: not run ($2 missing)"; return; fi
  if grep -q "$3" "$f"; then ok "$1"; else bad "$1: '$3' not found in Build/Logs/$2"; fi
}
stage "sim     RTL simulation"      sim.log       "PASS tb_gnss_stap"
n_lib=0
for f in "$LOG"/libip_*.log; do [[ -f "$f" && "$f" != *_vivado.log ]] && n_lib=$((n_lib+1)); done
if [[ "$n_lib" -gt 0 ]]; then ok "libip   $n_lib ADI library logs present"; else none "libip: no logs"; fi
stage "ip      gnss_passthrough IP"  ip.log        "IP_PACKAGE_OK"
stage "ip      gnss_stap IP"         ip_stap.log   "IP_PACKAGE_OK"
if [[ -f "$LOG/ip_stap.log" ]]; then
  if grep -q "exactly one address block" "$LOG/ip_stap.log"; then ok "ip      gnss_stap has one address block"
  else bad "ip      gnss_stap single-address-block self-check missing (old gnss_stap_ip.tcl?)"; fi
fi
stage "project create_project"       project.log   "CREATE_RESULT: PASS"
stage "build   synth/impl/bitstream"  build.log     "BUILD_RESULT: PASS"
stage "sw      application ELF"      sw.log        "SW_RESULT: PASS"
stage "fsbl    FSBL"                 fsbl.log      "FSBL_RESULT: PASS"
if [[ -s "$OUT/BOOT.BIN" ]]; then ok "boot    BOOT.BIN"; else none "boot: no Build/Output/BOOT.BIN"; fi

# first error of a failed stage, if any
for f in project build sw fsbl ip ip_stap; do
  [[ -f "$LOG/$f.log" ]] || continue
  e=$(grep -m1 -E "^ERROR:|_RESULT: FAIL|FAIL -" "$LOG/$f.log" || true)
  [[ -n "$e" ]] && p "         first error in $f.log: ${e:0:200}"
done

# ---------------------------------------------------------------- 2 address map
hdr "2 address map"
if [[ -f "$LOG/project.log" ]]; then
  for pair in "gnss_stap:0x43C10000" "gnss_passthrough:0x43C00000"; do
    inst=${pair%%:*}; addr=${pair##*:}
    line=$(grep -m1 "^CRPA: $inst at" "$LOG/project.log" || true)
    if [[ -z "$line" ]]; then none "$inst: no 'CRPA: $inst at' line (older system_bd.tcl?)"
    elif grep -qi "$addr" <<<"$line"; then ok "$line"
    else bad "$line  (expected $addr)"; fi
  done
  if grep -q "BD 41-1075" "$LOG/project.log"; then bad "[BD 41-1075] address overlap in project.log - re-run with --from ip"; fi
  grep "^CRPA: address map:" "$LOG/project.log" | sed 's/^CRPA: address map:/        /' | tee -a "$SUM"
else
  none "project.log missing"
fi

# ---------------------------------------------------------------- 3 timing
hdr "3 timing (implementation)"
T="$OUT/timing_impl.rpt"
if [[ -f "$LOG/build.log" ]]; then
  bt=$(grep -oE "BUILD_TIMING: [A-Z]+" "$LOG/build.log" | tail -1 | awk '{print $2}')
  bw=$(grep -oE "BUILD_WNS: [-0-9.]+" "$LOG/build.log" | tail -1 | awk '{print $2}')
  bh=$(grep -oE "BUILD_WHS: [-0-9.]+" "$LOG/build.log" | tail -1 | awk '{print $2}')
  if [[ "$bt" == "MET" ]]; then ok "timing MET (WNS $bw ns, WHS $bh ns)"
  elif [[ -n "$bt" ]]; then bad "timing $bt (WNS $bw ns, WHS $bh ns)"
  else none "no BUILD_TIMING line in build.log"; fi
fi
if [[ -f "$T" ]]; then
  # Design Timing Summary: the value line two lines below the WNS(ns) header
  vals=$(awk '/Design Timing Summary/{d=1} d&&/WNS\(ns\)/{getline; getline; print; exit}' "$T")
  if [[ -n "$vals" ]]; then
    read -r WNS TNS TNSF TNST WHS THS THSF THST WPWS _ TPWSF TPWST <<<"$vals"
    p "        setup : WNS ${WNS} ns  TNS ${TNS} ns  failing ${TNSF}/${TNST} endpoints"
    p "        hold  : WHS ${WHS} ns  THS ${THS} ns  failing ${THSF}/${THST} endpoints"
    p "        pulse : WPWS ${WPWS} ns  failing ${TPWSF}/${TPWST}"
    awk -v w="$WNS" -v h="$WHS" -v pw="$WPWS" 'BEGIN{exit !(w<0||h<0||pw<0)}' && bad "negative slack in timing_impl.rpt"
  else
    wrn "could not parse the Design Timing Summary in timing_impl.rpt"
  fi
  p "        per clock (WNS / WHS):"
  awk '/Intra Clock Table/{f=1;next}
       f&&/^Clock /{h=1;next} f&&h&&/^-----/{next}
       f&&h&&NF==0{exit}
       f&&h{printf "          %-24s WNS %8s  WHS %8s  setup-fail %s  hold-fail %s\n",$1,$2,$6,$4,$8}' "$T" | tee -a "$SUM"
  inter=$(awk '/Inter Clock Table/{f=1;next} f&&/^From Clock/{h=1;next} f&&h&&/^-----/{next} f&&h&&NF==0{exit} f&&h{print}' "$T")
  if [[ -n "$inter" ]]; then
    p "        inter-clock paths (bounded by the CDC set_max_delay constraints; slack must be >= 0):"
    while IFS= read -r l; do p "          $l"; done <<<"$inter"
    while read -r from to wns _; do
      awk -v v="$wns" 'BEGIN{exit !(v+0<0)}' 2>/dev/null && bad "inter-clock $from -> $to WNS $wns ns (CDC constraint not applied?)"
    done <<<"$inter"
  else
    ok "no failing/timed inter-clock paths listed"
  fi
else
  none "Build/Output/timing_impl.rpt missing (stage 5 not run or failed)"
fi

# ---------------------------------------------------------------- 4 CDC
hdr "4 CDC constraints"
IMPL_LOG=""; SYN_LOG=""
for f in "$ROOT"/Vivado/Project/*.runs/impl_1/runme.log;  do [[ -f "$f" ]] && IMPL_LOG="$f"; done
for f in "$ROOT"/Vivado/Project/*.runs/synth_1/runme.log; do [[ -f "$f" ]] && SYN_LOG="$f"; done
if [[ -n "$IMPL_LOG" ]]; then
  for x in gnss_stap_cdc.xdc gnss_passthrough_cdc.xdc; do
    if grep -q "Finished Parsing XDC File.*$x" "$IMPL_LOG"; then ok "$x applied in implementation"
    else bad "$x not parsed in implementation (impl_1/runme.log)"; fi
  done
  nm=$(grep -E "No (cells|pins|nets) matched" "$IMPL_LOG" | grep -cE "gnss_stap|gnss_passthrough" || true)
  if [[ "$nm" -gt 0 ]]; then
    bad "$nm 'No cells matched' warnings for the gnss_* CDC constraints:"
    grep -E "No (cells|pins|nets) matched" "$IMPL_LOG" | grep -E "gnss_stap|gnss_passthrough" | head -5 | sed 's/^/          /' | tee -a "$SUM"
  else ok "every gnss_* CDC constraint matched cells"; fi
else
  none "Vivado/Project/*.runs/impl_1/runme.log not found"
fi

# ---------------------------------------------------------------- 5 utilisation
hdr "5 utilisation (XC7Z020)"
U="$OUT/utilization_impl.rpt"
if [[ -f "$U" ]]; then
  for site in "Slice LUTs" "Slice Registers" "Block RAM Tile" "DSPs"; do
    row=$(grep -m1 -E "^\| ${site}[ *]*\|" "$U" || true)
    if [[ -z "$row" ]]; then wrn "$site: row not found"; continue; fi
    used=$(awk -F'|' '{gsub(/ /,"",$3); print $3}' <<<"$row")
    avail=$(awk -F'|' '{gsub(/ /,"",$(NF-2)); print $(NF-2)}' <<<"$row")
    util=$(awk -F'|' '{gsub(/ /,"",$(NF-1)); print $(NF-1)}' <<<"$row")
    line=$(printf "%-16s %7s / %-7s %6s %%" "$site" "$used" "$avail" "$util")
    if awk -v u="$util" 'BEGIN{exit !(u+0>100)}'; then bad "$line  (over capacity)"
    elif awk -v u="$util" 'BEGIN{exit !(u+0>85)}'; then wrn "$line  (high - routing/timing may suffer)"
    else ok "$line"; fi
  done
else
  none "Build/Output/utilization_impl.rpt missing"
fi

# ---------------------------------------------------------------- 6 warnings
hdr "6 critical warnings / errors"
for f in "$LOG/project.log" "$LOG/build.log" "$SYN_LOG" "$IMPL_LOG" "$LOG/sw.log" "$LOG/fsbl.log"; do
  [[ -n "$f" && -f "$f" ]] || continue
  cw=$(grep -c "^CRITICAL WARNING" "$f" || true); er=$(grep -c "^ERROR" "$f" || true)
  rel=${f#"$ROOT"/}
  if [[ "$er" -gt 0 ]]; then bad "$rel: $er ERROR, $cw CRITICAL WARNING"
  elif [[ "$cw" -gt 0 ]]; then wrn "$rel: $cw CRITICAL WARNING"
  else ok "$rel: clean"; fi
  if [[ "$cw" -gt 0 ]]; then
    grep "^CRITICAL WARNING" "$f" | cut -c1-140 | sort | uniq -c | sort -rn | head -5 | sed 's/^/          /' | tee -a "$SUM"
  fi
done

# ---------------------------------------------------------------- 7 outputs
hdr "7 outputs"
for f in BOOT.BIN system_top.bit system.xsa; do
  if [[ -s "$OUT/$f" ]]; then ok "$f  $(stat -c %s "$OUT/$f") bytes  sha256 $(sha256sum "$OUT/$f" | cut -c1-16)..."
  else none "$f missing"; fi
done
ELF="$ROOT/Build/ELF/e310_gnss_app.elf"
if [[ -s "$ELF" ]]; then
  if grep -aq "gnss_stap: CRPA stage present" "$ELF"; then ok "ELF contains the CRPA driver"; else bad "ELF has no CRPA driver"; fi
  if grep -aq "LOOPBACK DEPLOYMENT BUILD" "$ELF"; then
    if grep -aq "CRPA anti-jam ON" "$ELF"; then wrn "ELF variant: DEPLOY - TX1 transmits at power-on, CRPA ON"
    else wrn "ELF variant: DEPLOY (no CRPA) - TX1 transmits at power-on"; fi
  else ok "ELF variant: dev (TX silent at power-on)"; fi
else
  none "Build/ELF/e310_gnss_app.elf missing"
fi
REL=""
for d in "$ROOT"/Build/Release/*/; do [[ -d "$d" && ( -z "$REL" || "$d" -nt "$REL" ) ]] && REL="$d"; done
[[ -n "$REL" ]] && p "        latest release: ${REL%/}"

# ---------------------------------------------------------------- verdict
hdr "verdict"
RC=0
if [[ $NFAIL -gt 0 ]]; then
  p "  REPORT: FAIL ($NFAIL failing item(s), $NWARN warning(s))"; RC=1
elif [[ ! -s "$OUT/BOOT.BIN" ]]; then
  p "  REPORT: INCOMPLETE - no BOOT.BIN yet (build not finished, or stopped before stage 8)"; RC=2
else
  p "  REPORT: PASS ($NWARN warning(s))"
fi
p "  summary written to Build/Logs/report_summary.txt"
exit $RC
