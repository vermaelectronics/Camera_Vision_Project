#!/bin/bash
cd "$(dirname "$0")"
run() { python3 core_jam.py $1 $2 $3 5 2>&1 | grep -E "J/N|summary" > out_$1_$2_$3.txt; }
echo "barrage 30 clean
barrage 40 mp
barrage 50 mp
barrage 60 mp
cw 40 mp
cw 60 mp
chirp100 40 mp
chirp1000 40 mp
pulse 40 clean
pulse 50 mp" | xargs -P 4 -L 1 bash -c 'python3 core_jam.py $0 $1 $2 5 2>&1 | grep -E "J/N|summary" > out_$0_$1_$2.txt'
cat out_*.txt > batch.out
