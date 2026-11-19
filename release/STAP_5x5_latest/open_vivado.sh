#!/usr/bin/env bash
# Open the PASS_MVDR_STAP Vivado project (hdl/Passthrough_MVDR/passthrough_mvdr.xpr).
# Usage:  ./open_vivado.sh [path-to-Vivado-2023.2]      (default /tools/Xilinx/Vivado/2023.2)
VIVADO_DIR="${1:-/tools/Xilinx/Vivado/2023.2}"
if [ ! -f "$VIVADO_DIR/settings64.sh" ]; then
  echo "ERROR: $VIVADO_DIR/settings64.sh not found. Pass your Vivado 2023.2 folder: ./open_vivado.sh /opt/Xilinx/Vivado/2023.2"
  exit 1
fi
source "$VIVADO_DIR/settings64.sh"
cd "$(dirname "$0")/hdl/Passthrough_MVDR"
vivado passthrough_mvdr.xpr &
