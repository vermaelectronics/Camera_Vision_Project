#!/usr/bin/env bash
# Host test of the anti-jam engine (gnss_aj.c + gnss_passthrough.c + the
# algorithm's gnss_aj_math.c) against simulated DMA captures, register file
# and filter, once per CRPA algorithm (gcc only, no Xilinx tools).
#   ./run_engine_test.sh                all algorithms
#   ./run_engine_test.sh crpa27 -v      one, with the engine's console output
set -uo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$here/../../.."
source "$root/Automation/Linux/crpa_algos.sh"
verbose=0; list=()
for a in "$@"; do [[ "$a" == -v ]] && verbose=1 || list+=("$a"); done
[[ ${#list[@]} -eq 0 ]] && list=("${CRPA_ALGOS[@]}")
fw="$root/Source/Firmware/app_crpa_lvds"
tmp="${TMPDIR:-/tmp}/crpa_engine_tb_$$"; mkdir -p "$tmp"
fail=0
for a in "${list[@]}"; do
  crpa_algo_params "$a" || { echo "unknown algorithm $a"; fail=1; continue; }
  d="$tmp/$a"; mkdir -p "$d"
  cp "$root/Source/CRPA/$a/arm/gnss_aj_math.c" "$root/Source/CRPA/$a/arm/gnss_aj_math.h" "$d/"
  printf '#define CRPA_ALGO_NUM %s\n#define CRPA_ALGO_STR "%s"\n#define CRPA_CORE_STR "%s"\n#define CRPA_AJ_INFO %sU\n' \
         "$CRPA_NUM" "$a" "$CRPA_CORE" "$CRPA_AJ_INFO" > "$d/crpa_build.h"
  echo 'extern unsigned int tb_capbuf[];' > "$d/tb_buf.h"
  gcc -O2 -std=gnu99 -Wall -Wextra -Wno-unused-parameter -Wno-pointer-to-int-cast -Wno-int-to-pointer-cast \
      -include "$d/tb_buf.h" -DGNSS_AJ_BUF_ADDR='((uintptr_t)tb_capbuf)' ${TB_CFLAGS:-} \
      -I"$d" -I"$here/stub" -I"$fw" \
      "$here/tb_engine.c" "$fw/gnss_aj.c" "$fw/gnss_passthrough.c" "$d/gnss_aj_math.c" -lm -o "$d/tb" || { fail=1; continue; }
  if [[ $verbose -eq 1 ]]; then "$d/tb" || fail=1
  else "$d/tb" > "$d/log" 2>&1 || fail=1; grep -E "ENGINE_TB" "$d/log"; fi
done
rm -rf "$tmp"
[[ $fail -eq 0 ]] && echo "ENGINE_TB: ALL PASS" || { echo "ENGINE_TB: FAIL"; exit 1; }
