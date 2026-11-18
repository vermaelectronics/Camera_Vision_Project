#!/usr/bin/env bash
# ============================================================================
#  crpa_algos.sh  --  the CRPA algorithms this pipeline can build, one
#  bitstream + firmware (BOOT.BIN) per algorithm. Sourced by build_all.sh,
#  csim_gcc.sh and Source/HDL/sim/run_iverilog.sh.
#
#    algo    core     taps (RX1 + RX2)  REF  weights from the ARM
#    crpa1   smi_pi    6 +  6           3    SMI (averaged statistics, Cholesky)
#    crpa27  stap_pi   8 +  8           4    direct (Wiener) start + NLMS,
#    crpa28  stap_pi  10 + 10           5      moving-jammer tracking and
#    crpa29  stap_pi  14 + 14           7      weight prediction
#    crpa30  stap_pi  16 + 16           8
#    crpa31  stap_pi  18 + 18           9
#    loopback  (none)  -                -    no anti-jam: RX1 -> TX1, RX2 -> TX2
#                                             (./build_all.sh --algo loopback)
#
#  The algorithm sources are Source/CRPA/<algo>/{fpga,arm}, exactly as
#  delivered; NT / REF here must match their ST_NT / SMI_NT and AJ_NT / AJ_REF
#  (build_all.sh --only check verifies it).
#
#  CRPA_NWTS    = 32-bit words of the core's wts port (smi_pi: 2048 bits = 64)
#  CRPA_NW_USED = weight words the core reads (2 x NT)
#  CRPA_NWCJ    = image (conjugate) tap words (smi_pi: 2, stap_pi: 2 x 3)
# ============================================================================
CRPA_ALGOS=(crpa1 crpa27 crpa28 crpa29 crpa30 crpa31)

crpa_algo_params() {   # <algo> -> CRPA_NUM CRPA_CORE CRPA_NT CRPA_REF CRPA_NWTS CRPA_NW_USED CRPA_NWCJ CRPA_HAS_GAIN CRPA_CORE_VER
  case "$1" in
    crpa1)  CRPA_NUM=1;  CRPA_CORE=smi_pi;  CRPA_NT=6;  CRPA_REF=3 ;;
    crpa27) CRPA_NUM=27; CRPA_CORE=stap_pi; CRPA_NT=8;  CRPA_REF=4 ;;
    crpa28) CRPA_NUM=28; CRPA_CORE=stap_pi; CRPA_NT=10; CRPA_REF=5 ;;
    crpa29) CRPA_NUM=29; CRPA_CORE=stap_pi; CRPA_NT=14; CRPA_REF=7 ;;
    crpa30) CRPA_NUM=30; CRPA_CORE=stap_pi; CRPA_NT=16; CRPA_REF=8 ;;
    crpa31) CRPA_NUM=31; CRPA_CORE=stap_pi; CRPA_NT=18; CRPA_REF=9 ;;
    # plain RX -> TX loop-back: no filter core, no solver (gnss_passthrough only)
    loopback) CRPA_NUM=0; CRPA_CORE=none; CRPA_NT=1; CRPA_REF=0 ;;
    *) return 1 ;;
  esac
  CRPA_NW_USED=$((2 * CRPA_NT))
  if [[ "$CRPA_CORE" == none ]]; then
    CRPA_NWTS=1; CRPA_NW_USED=1; CRPA_NWCJ=1; CRPA_HAS_GAIN=0; CRPA_CORE_VER=0
  elif [[ "$CRPA_CORE" == smi_pi ]]; then
    CRPA_NWTS=64; CRPA_NWCJ=2; CRPA_HAS_GAIN=0; CRPA_CORE_VER=4.9
  else
    CRPA_NWTS=$CRPA_NW_USED; CRPA_NWCJ=6; CRPA_HAS_GAIN=1; CRPA_CORE_VER=1.0
  fi
  CRPA_AJ_INFO=$(printf '0x%02X%02X%02X07' "$CRPA_NUM" "$CRPA_NT" "$CRPA_REF")
  export CRPA_NUM CRPA_CORE CRPA_NT CRPA_REF CRPA_NWTS CRPA_NW_USED CRPA_NWCJ CRPA_HAS_GAIN CRPA_CORE_VER CRPA_AJ_INFO
  return 0
}
