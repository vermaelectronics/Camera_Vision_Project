#!/usr/bin/env bash
# ============================================================================
#  setup_vendor.sh  --  puts the Analog Devices HDL library (hdl_2023_r2) in
#  Vendor/ADI_hdl_2023_r2. Run once.
#
#    ./setup_vendor.sh                       clone from GitHub
#    ./setup_vendor.sh /path/to/ADI_hdl_2023_r2
#                                            reuse an existing copy (e.g. the
#                                            one your STAP project built with):
#                                            it is COPIED, so the SFAP project
#                                            never writes into the STAP tree
# ============================================================================
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$here/env.sh"

mkdir -p "$SFAP_ROOT/Vendor"
if [[ -d "$ADI_HDL_DIR/library/axi_ad9361" ]]; then
  echo "setup_vendor: $ADI_HDL_DIR already present"
  exit 0
fi
if [[ $# -ge 1 ]]; then
  src="$(cd "$1" && pwd)"
  [[ -d "$src/library/axi_ad9361" ]] || { echo "setup_vendor: $src is not an ADI hdl tree"; exit 2; }
  echo "setup_vendor: copying $src -> $ADI_HDL_DIR (built library IPs included)"
  mkdir -p "$ADI_HDL_DIR"
  rsync -a --exclude 'projects/gnss_sfap' "$src/" "$ADI_HDL_DIR/"
else
  echo "setup_vendor: cloning analogdevicesinc/hdl branch hdl_2023_r2 -> $ADI_HDL_DIR"
  git clone --depth 1 -b hdl_2023_r2 https://github.com/analogdevicesinc/hdl.git "$ADI_HDL_DIR"
fi
echo "setup_vendor: OK"
