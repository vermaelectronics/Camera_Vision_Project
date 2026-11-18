#!/usr/bin/env bash
# Installs the packages once, then asks for the ports and starts the test.
cd "$(dirname "$0")"
python3 -m pip install -q -r requirements.txt 2>/dev/null || python3 -m pip install --user -q -r requirements.txt 2>/dev/null
python3 e310lab.py
