#!/usr/bin/env bash
# build_all.sh - SFAP-PI LVDS: IP -> Vivado (XSA) -> Vitis (firmware) -> BOOT.bin
# Usage (package root):
#   ./build_all.sh <path to plutosdr-fw/hdl> [Xilinx tools folder, default /tools/Xilinx]
# e.g.
#   ./build_all.sh ~/antsdr/antsdr-fw-patch/plutosdr-fw/hdl
set -e
HDL="${1:?usage: $0 <path to plutosdr-fw/hdl> [Xilinx tools folder]}"
XIL="${2:-/tools/Xilinx}"
cd "$(dirname "$0")"
source "$XIL/Vivado/2023.2/settings64.sh"
./scripts/build_ip.sh
./scripts/build_hw.sh "$HDL"
./scripts/build_fw.sh "$XIL/Vitis/2023.2"
