#!/usr/bin/env bash
# ============================================================================
#  env.sh  --  paths for the SFAP-PI build. Sourced by build_all.sh.
#  Edit the defaults here or override any of them in the environment.
# ============================================================================

# Project root (the directory that holds Source/, Automation/, Vendor/, Build/)
export SFAP_ROOT="${SFAP_ROOT:-/home/manoranjan/CRPA_Project/SFAP/SFAP_snd2}"

# Xilinx 2023.2 install. settings64.sh of each tool is sourced if the tool is
# not already on PATH.
export XILINX_ROOT="${XILINX_ROOT:-/tools/Xilinx}"
export XILINX_VER="${XILINX_VER:-2023.2}"

# Analog Devices HDL library, branch hdl_2023_r2 (setup_vendor.sh fetches it).
export ADI_HDL_DIR="${ADI_HDL_DIR:-$SFAP_ROOT/Vendor/ADI_hdl_2023_r2}"

# Where the Vivado project is generated INSIDE the ADI tree (the ADI scripts
# locate themselves by relative path, so the project must live there).
export SFAP_HDL_PRJ_DIR="${SFAP_HDL_PRJ_DIR:-$ADI_HDL_DIR/projects/gnss_sfap/e310}"
export SFAP_HDL_PRJ_NAME="${SFAP_HDL_PRJ_NAME:-gnss_sfap_e310}"

# Build outputs
export SFAP_BUILD="${SFAP_BUILD:-$SFAP_ROOT/Build}"
export GNSS_CRPA_IP_REPO="${GNSS_CRPA_IP_REPO:-$SFAP_BUILD/ip_repo}"
export SFAP_SW_WS="${SFAP_SW_WS:-$SFAP_BUILD/vitis_ws}"
export SFAP_BOOT_DIR="${SFAP_BOOT_DIR:-$SFAP_BUILD/boot}"
export SFAP_LOG_DIR="${SFAP_LOG_DIR:-$SFAP_BUILD/logs}"

# ADI checks the Vivado version against this.
export REQUIRED_VIVADO_VERSION="${REQUIRED_VIVADO_VERSION:-2023.2}"
# Set to 1 only if you deliberately run another Vivado version.
export ADI_IGNORE_VERSION_CHECK="${ADI_IGNORE_VERSION_CHECK:-0}"

# Parallel jobs for the ADI library builds.
export SFAP_JOBS="${SFAP_JOBS:-4}"
