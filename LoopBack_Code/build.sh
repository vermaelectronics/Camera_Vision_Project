#!/usr/bin/env bash
# Complete, non-interactive Vivado bitstream build - Linux/macOS terminal entry point.
# Everything happens from the command line; no Vivado GUI is ever opened.
# Usage:
#   ./build.sh                    full build, produces Build/Bitstream/antsdr_e310_gnss.bit
#   ./build.sh -FromStage 5       re-run just the synth/impl/bitstream stage
#   ./build.sh -CleanVendorWork   force a fresh copy of the vendor HDL tree
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! command -v pwsh >/dev/null 2>&1; then
  echo "ERROR: PowerShell 7 (pwsh) was not found on PATH." >&2
  echo "Install it (see https://aka.ms/powershell) and re-run this script." >&2
  exit 1
fi

exec pwsh -NoProfile -File "$SCRIPT_DIR/Automation/PowerShell/Build_All.ps1" "$@"
