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
#    stap7x7 stap_pi   7 +  7           3    the crpa27 code, 7 + 7 taps
#    stap6x6 stap_pi   6 +  6           3    the crpa27 code, 6 + 6 taps
#    loopback  (none)  -                -    no anti-jam: RX1 -> TX1, RX2 -> TX2
#    sfap    sfap_pi   256 bins, 1 weight + 2 image taps per bin, pulse blanker,
#                      bin excision (SFAP-PI v6.3, Source/SFAP)
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
CRPA_ALGOS=(crpa1 crpa27 crpa28 crpa29 crpa30 crpa31 stap7x7 stap6x6)

crpa_algo_params() {   # <algo> -> CRPA_NUM CRPA_CORE CRPA_NT CRPA_REF CRPA_NWTS CRPA_NW_USED CRPA_NWCJ CRPA_HAS_GAIN CRPA_CORE_VER
  case "$1" in
    crpa1)  CRPA_NUM=1;  CRPA_CORE=smi_pi;  CRPA_NT=6;  CRPA_REF=3 ;;
    crpa27) CRPA_NUM=27; CRPA_CORE=stap_pi; CRPA_NT=8;  CRPA_REF=4 ;;
    crpa28) CRPA_NUM=28; CRPA_CORE=stap_pi; CRPA_NT=10; CRPA_REF=5 ;;
    crpa29) CRPA_NUM=29; CRPA_CORE=stap_pi; CRPA_NT=14; CRPA_REF=7 ;;
    crpa30) CRPA_NUM=30; CRPA_CORE=stap_pi; CRPA_NT=16; CRPA_REF=8 ;;
    crpa31) CRPA_NUM=31; CRPA_CORE=stap_pi; CRPA_NT=18; CRPA_REF=9 ;;
    # STAP-NLMS 7 + 7, the crpa27 code with NT 7 / REF 3 (Source/CRPA/README_stap7x7.txt)
    stap7x7) CRPA_NUM=77; CRPA_CORE=stap_pi; CRPA_NT=7; CRPA_REF=3 ;;
    # STAP-NLMS 6 + 6, the crpa27 code with NT 6 / REF 3 (Source/CRPA/README_stap6x6.txt)
    stap6x6) CRPA_NUM=66; CRPA_CORE=stap_pi; CRPA_NT=6; CRPA_REF=3 ;;
    # plain RX -> TX loop-back: no filter core, no solver (gnss_passthrough only)
    loopback) CRPA_NUM=0; CRPA_CORE=none; CRPA_NT=1; CRPA_REF=0 ;;
    # SFAP-PI v6.3: 256-bin space-frequency filter (sources in Source/SFAP)
    sfap) CRPA_NUM=0; CRPA_CORE=sfap_pi; CRPA_NT=1; CRPA_REF=0 ;;
    *) return 1 ;;
  esac
  CRPA_NW_USED=$((2 * CRPA_NT))
  if [[ "$CRPA_CORE" == none ]]; then
    CRPA_NWTS=1; CRPA_NW_USED=1; CRPA_NWCJ=1; CRPA_HAS_GAIN=0; CRPA_CORE_VER=0
  elif [[ "$CRPA_CORE" == sfap_pi ]]; then
    CRPA_NWTS=1; CRPA_NW_USED=1; CRPA_NWCJ=1; CRPA_HAS_GAIN=0; CRPA_CORE_VER=5.0
  elif [[ "$CRPA_CORE" == smi_pi ]]; then
    CRPA_NWTS=64; CRPA_NWCJ=2; CRPA_HAS_GAIN=0; CRPA_CORE_VER=4.9
  else
    CRPA_NWTS=$CRPA_NW_USED; CRPA_NWCJ=6; CRPA_HAS_GAIN=1; CRPA_CORE_VER=1.0
  fi
  CRPA_AJ_INFO=$(printf '0x%02X%02X%02X07' "$CRPA_NUM" "$CRPA_NT" "$CRPA_REF")
  [[ "$CRPA_CORE" == sfap_pi ]] && CRPA_AJ_INFO=0x08070C05
  # lower-power options of the block design (system_bd.tcl):
  #   CRPA_PWR_C = 1: axi_ad9361 without DDS / IQ correction / DC filter (unused)
  #   CRPA_PWR_B = 1: ARM 333 MHz, DDR 400 MHz (loop-back only: no solver on the ARM)
  CRPA_PWR_C=0; CRPA_PWR_B=0
  case "$1" in
    loopback) CRPA_PWR_C=1; CRPA_PWR_B=1 ;;
    stap7x7|stap6x6)  CRPA_PWR_C=1 ;;
  esac
  export CRPA_PWR_C CRPA_PWR_B CRPA_NUM CRPA_CORE CRPA_NT CRPA_REF CRPA_NWTS CRPA_NW_USED CRPA_NWCJ CRPA_HAS_GAIN CRPA_CORE_VER CRPA_AJ_INFO
  return 0
}
