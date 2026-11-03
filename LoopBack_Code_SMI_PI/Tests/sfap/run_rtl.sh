#!/bin/bash
# SFAP RTL regression (iverilog + numpy). Run from the project root or from here.
#   1. FFT against numpy, random data and a tone with noise at the ADC level, back to back
#   2. bin processor against a floating-point model of the same recursion
#   3. the whole core: no jammer (output is a copy of antenna 1) and a 40 dB multipath barrage
set -e
cd "$(dirname "$0")/rtl"
echo "== 1. FFT"
python3 fft_check.py | tail -3
python3 fft_b2b.py | tail -4
echo "== 2. bin processor"
python3 bin_check.py | tail -4
echo "== 3. core: barrage 40 dB with multipath (about 2 min)"
python3 core_jam.py barrage 40 mp 5 | grep -E "J/N|summary"
