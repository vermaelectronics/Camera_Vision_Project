#!/usr/bin/env bash
# ============================================================================
#  env.sh  --  paths for the Passthrough_LVDS + SFAP-PI build.
#  Sourced by build_all.sh. Edit the defaults here or override any of them in
#  the environment, e.g.
#     ADI_HDL_DIR=/home/me/AntSDR/antsdr-fw-patch/plutosdr-fw/hdl ./build_all.sh
# ============================================================================

# Package root (holds Source/, Automation/, Build/). Found from this script.
export SFAP_ROOT="${SFAP_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"

# Xilinx tools. passthrough_lvds is a Vivado 2023.2 design (antsdr-fw-patch
# ADI hdl tree), so Vivado, Vitis and Vitis HLS 2023.2 are used.
export XILINX_ROOT="${XILINX_ROOT:-/tools/Xilinx}"
export XILINX_VER="${XILINX_VER:-2023.2}"

# The ADI hdl tree that already builds passthrough_lvds:
#   <...>/antsdr-fw-patch/plutosdr-fw/hdl   (contains projects/passthrough_lvds,
#   library/axi_ad9361, library/axi_vcxo_ctrl ...)
export ADI_HDL_DIR="${ADI_HDL_DIR:-$HOME/CRPA_Project/NEW_PASSTHROUGH/workspace/antsdr-fw-patch/plutosdr-fw/hdl}"

# The Vivado project is generated INSIDE the ADI tree, next to passthrough_lvds
# (the ADI scripts find themselves by relative path). passthrough_lvds itself
# is not touched.
export SFAP_HDL_PRJ_NAME="${SFAP_HDL_PRJ_NAME:-passthrough_lvds_sfap256}"
export SFAP_HDL_PRJ_DIR="${SFAP_HDL_PRJ_DIR:-$ADI_HDL_DIR/projects/$SFAP_HDL_PRJ_NAME}"

# Build outputs
export SFAP_BUILD="${SFAP_BUILD:-$SFAP_ROOT/Build}"
export GNSS_CRPA_IP_REPO="${GNSS_CRPA_IP_REPO:-$SFAP_BUILD/ip_repo}"
export SFAP_SW_WS="${SFAP_SW_WS:-$SFAP_BUILD/vitis_ws}"
export SFAP_BOOT_DIR="${SFAP_BOOT_DIR:-$SFAP_BUILD/boot}"
export SFAP_LOG_DIR="${SFAP_LOG_DIR:-$SFAP_BUILD/logs}"

# Firmware source folder and application name
export SFAP_APP_NAME="${SFAP_APP_NAME:-app_sfap_lvds}"

# ADI checks the Vivado version (2023.2 for this tree). Set to 1 only if you
# deliberately run another Vivado version.
# (Only exported when you set it: some ADI releases treat "defined" as "on".)
if [[ "${ADI_IGNORE_VERSION_CHECK:-0}" == "1" ]]; then
  export ADI_IGNORE_VERSION_CHECK=1
else
  unset ADI_IGNORE_VERSION_CHECK
fi

# Parallel jobs for the ADI library builds.
export SFAP_JOBS="${SFAP_JOBS:-4}"
