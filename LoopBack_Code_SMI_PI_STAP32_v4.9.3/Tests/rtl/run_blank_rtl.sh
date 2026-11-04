#!/usr/bin/env bash
# RTL check of gnss_pulse_blank against its Python model (needs iverilog).
set -e
cd "$(dirname "$0")"
W=${TMPDIR:-/tmp}/blank_rtl; mkdir -p "$W"; cp tb_pulse_blank.v blank_model.py "$W"/; cd "$W"
iverilog -g2012 -o tb tb_pulse_blank.v "$OLDPWD/../../Source/HDL/gnss_pulse_blank.v"
rc=0
for cfg in "20000 1" "20000 0" "3000 1"; do
  set -- $cfg
  python3 blank_model.py $1 $2 | tee model.txt
  vvp tb +thr=$1 +en=$2 | grep BLK_CNT | tee rtl.txt
  # the RTL stream is the model's stream (the first output samples line up with the model's n = 0..)
  python3 - <<'PY' || rc=1
import re
ref=[l.strip() for l in open("blank_ref.hex")]
out=[l.strip() for l in open("blank_out.hex")]
n=min(len(ref),len(out)); bad=sum(1 for a,b in zip(ref[:n],out[:n]) if a!=b)
m=re.findall(r"\d+",open("model.txt").read()); r=re.findall(r"\d+",open("rtl.txt").read())
print("  compared %d samples: %d differ; model blk %s bursts %s | rtl blk %s bursts %s" % (n,bad,m[0],m[1],r[0],r[1]))
import sys
sys.exit(0 if bad==0 and m[0]==r[0] and m[1]==r[1] else 1)
PY
done
[ $rc = 0 ] && echo "PULSE_BLANK_RTL: PASS" || echo "PULSE_BLANK_RTL: FAIL"
exit $rc
