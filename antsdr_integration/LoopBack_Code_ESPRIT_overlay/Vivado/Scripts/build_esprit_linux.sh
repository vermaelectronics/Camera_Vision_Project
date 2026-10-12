#!/usr/bin/env bash
# ============================================================================
#  build_esprit_linux.sh -- Linux driver for the LoopBack_Code + ESPRIT build
#
#  The project's own flow (Automation/PowerShell/Build_All.ps1) is Windows.
#  This script performs the same Vivado/Vitis steps on Linux, in order, and
#  stops at the first step that does not report PASS.
#
#  USAGE
#    source <Vivado>/settings64.sh            # the Vivado used for this design
#    export ADI_HDL_DIR=<packaged ADI hdl working copy>   # same as before ESPRIT
#    Vivado/Scripts/build_esprit_linux.sh [step ...]
#
#  STEPS (default: sim ip project build report)
#    sim      ESPRIT DOA system testbench in xsim         -> DOA_SIM_RESULT
#    ip       re-package gnss_passthrough v1.2 (+ESPRIT)  -> IP_PACKAGE_OK
#    project  re-create the Vivado project                -> CREATE_RESULT
#    build    synth + impl + bitstream + XSA              -> BUILD_RESULT
#    report   DOA utilisation / timing / CDC in the build -> DOA_IMPL lines
#    sw       Vitis firmware build (needs vitis on PATH)  -> SW_RESULT
#
#  Logs: Build/logs/<step>.log
# ============================================================================
set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
LOG="$ROOT/Build/logs"; mkdir -p "$LOG"
export GNSS_CRPA_ROOT="$ROOT"
export GNSS_CRPA_IP_REPO="$ROOT/Build/ip_repo"
STEPS=("$@"); [[ ${#STEPS[@]} -eq 0 ]] && STEPS=(sim ip project build report)

need() { command -v "$1" >/dev/null || { echo "BUILD_ESPRIT: '$1' not on PATH (source settings64.sh)"; exit 2; }; }
run()  { local step=$1 marker=$2; shift 2
         echo "BUILD_ESPRIT: ===== $step ====="
         "$@" 2>&1 | tee "$LOG/$step.log" | grep -E "RESULT|_OK|PASS|FAIL|ERROR|DOA_|BUILD_WNS|BUILD_WHS|BUILD_TIMING"
         grep -q "$marker" "$LOG/$step.log" || { echo "BUILD_ESPRIT: $step FAILED (see $LOG/$step.log)"; exit 1; } }

for s in "${STEPS[@]}"; do
  case "$s" in
    sim)     need vivado
             run sim "DOA_SIM_RESULT: PASS" vivado -mode batch -nojournal -nolog \
                 -source "$ROOT/Source/Sim/ESPRIT/sim_doa.tcl" ;;
    ip)      need vivado
             run ip "IP_PACKAGE_OK" vivado -mode batch -nojournal -nolog \
                 -source "$ROOT/Source/IP/gnss_passthrough/gnss_passthrough_ip.tcl" \
                 -tclargs "$ROOT/Source" "$GNSS_CRPA_IP_REPO" ;;
    project) need vivado
             : "${ADI_HDL_DIR:?set ADI_HDL_DIR to the packaged ADI hdl working copy}"
             mkdir -p "$ROOT/Vivado/Project"
             ( cd "$ROOT/Vivado/Project" && \
               run project "CREATE_RESULT: PASS" vivado -mode batch -nojournal -nolog \
                   -source "$ROOT/Vivado/Scripts/create_project.tcl" ) || exit 1 ;;
    build)   need vivado
             run build "BUILD_RESULT: PASS" vivado -mode batch -nojournal -nolog \
                 -source "$ROOT/Vivado/Scripts/build.tcl" -tclargs \
                 "$ROOT/Vivado/Project/antsdr_e310_gnss.xpr" \
                 "$ROOT/Build/BIT/system_top.bit" "$ROOT/Build/XSA/system_top.xsa" 8 ;;
    report)  need vivado
             tcl="$LOG/report_wrapper.tcl"
             printf 'open_project {%s}\nopen_run impl_1\nsource {%s}\n' \
                 "$ROOT/Vivado/Project/antsdr_e310_gnss.xpr" "$ROOT/Vivado/Scripts/report_doa.tcl" > "$tcl"
             run report "DOA_IMPL: design WNS" vivado -mode batch -nojournal -nolog -source "$tcl" ;;
    sw)      need vitis
             run sw "SW_RESULT: PASS" vitis -s "$ROOT/Vitis/Scripts/build_software.py" -- \
                 "$ROOT/Build/XSA/system_top.xsa" "$ROOT/Vitis/Workspace" \
                 "$ROOT/Source/Firmware/app_gnss_e310" "$ROOT/Build/ELF/e310_gnss_app.elf" ;;
    *)       echo "BUILD_ESPRIT: unknown step '$s'"; exit 2 ;;
  esac
done
echo "BUILD_ESPRIT: all requested steps PASSED"
