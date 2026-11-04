#!/usr/bin/env bash
# Full regression: C solver -> golden model/vectors -> RTL simulation (FOLD=1 and FOLD=4)
# Requires: gcc, python3 + numpy, iverilog
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
B="$ROOT/build"
mkdir -p "$B"

echo "== building C weight solver"
gcc -O2 -std=c99 -Wall -Wextra -o "$B/stap_solve_cli" "$ROOT/sw/stap_solve_cli.c" "$ROOT/sw/stap_solver.c" -lm
gcc -O2 -std=c99 -Wall -Wextra -c -o "$B/stap_driver.o" "$ROOT/sw/stap_driver.c"

echo "== golden model + test vectors"
python3 "$ROOT/model/stap_golden.py" --out "$B" "$@"

for F in ${FOLDS:-1 4}; do
  echo "== RTL simulation FOLD=$F"
  iverilog -g2005 -Wall -DFOLD_VAL=$F -I "$B" -o "$B/tb_fold$F.vvp" \
      "$ROOT/tb/tb_stap_top.v" "$ROOT/rtl/stap_top.v" "$ROOT/rtl/stap_fir.v" "$ROOT/rtl/stap_corr.v" 2>&1 | grep -v timescale || true
  (cd "$B" && vvp -n "tb_fold$F.vvp") | tee "$B/sim_fold$F.txt"
  grep -q "\*\*\* PASS" "$B/sim_fold$F.txt"
done
echo "== all simulations passed"
