#!/usr/bin/env bash
# build_ip.sh - fill project/Pass_SFAP/ip_repo with the two custom IPs:
#   sfap_pi 5.0          the SFAP-PI HLS core (prebuilt export from Source, or
#                        rebuilt with Vitis HLS when --hls is given)
#   gnss_passthrough 3.0 packaged from source/HDL/gnss_passthrough.v
#
# Usage (package root, Vivado 2023.2 settings loaded):
#   ./scripts/build_ip.sh          # prebuilt sfap_pi + package gnss_passthrough
#   ./scripts/build_ip.sh --hls    # also re-run Vitis HLS for sfap_pi (~20-40 min)
set -e
cd "$(dirname "$0")/.."
PKG="$(pwd)"
IP="$PKG/project/Pass_SFAP/ip_repo"
mkdir -p "$IP"

command -v vivado >/dev/null || { echo "ERROR: vivado not on PATH - source <Vivado 2023.2>/settings64.sh first"; exit 1; }

echo "=== sfap_pi (HLS core) ==="
rm -rf "$IP/sfap_pi"
if [ "$1" = "--hls" ]; then
  command -v vitis_hls >/dev/null || { echo "ERROR: vitis_hls not on PATH - source <Vitis 2023.2>/settings64.sh"; exit 1; }
  (cd "$PKG/source/HLS/sfap_pi" && GNSS_CRPA_IP_REPO="$IP" SFAP_PI_SKIP_CSIM=1 vitis_hls -f build_hls.tcl)
else
  cp -r "$PKG/source/HLS/sfap_pi/prebuilt_ip/sfap_pi" "$IP/sfap_pi"
  echo "prebuilt sfap_pi 5.0 copied (Vitis HLS 2023.2 export from Source.zip)"
fi
test -f "$IP/sfap_pi/component.xml" || { echo "ERROR: $IP/sfap_pi/component.xml missing"; exit 1; }

echo "=== gnss_passthrough (packaged from source/HDL/gnss_passthrough.v) ==="
mkdir -p "$PKG/build/logs"
vivado -mode batch -nojournal -log "$PKG/build/logs/package_gnss_passthrough.log" \
  -source "$PKG/source/IP/gnss_passthrough/gnss_passthrough_ip.tcl" -tclargs "$PKG/source" "$IP"
test -f "$IP/gnss_passthrough/component.xml" || { echo "ERROR: gnss_passthrough was not packaged, see build/logs/package_gnss_passthrough.log"; exit 1; }

echo
echo "IP repository ready: $IP"
ls "$IP"
