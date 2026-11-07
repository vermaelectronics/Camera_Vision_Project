# shellcheck shell=bash
#
# Xilinx Vivado + Vitis 2023.2 environment for the plutosdr-fw / antsdr build.
#
# The plutosdr-fw Makefile runs every Xilinx tool as
#     bash -c "source $(VIVADO_SETTINGS) && <tool> ..."
# and needs three tools from that one file:
#     vivado   - HDL project -> bitstream -> system_top.xsa   (Vivado)
#     xsct     - FSBL (fsbl.elf) from the XSA                  (Vitis)
#     bootgen  - boot.bin / BOOT.bin                           (Vivado or Vitis)
# In 2023.2, xsct ships with Vitis, so Vivado's settings64.sh alone is often
# not enough. Point VIVADO_SETTINGS at this file instead:
#
#     export VIVADO_SETTINGS=/path/to/antsdr-fw-patch/xilinx_env.sh
#
# Optional overrides (export before running make / build.sh):
#     XILINX_DIR      Xilinx install root (auto-detected: /tools/Xilinx, /opt/Xilinx, ~/Xilinx)
#     XILINX_VERSION  Tool version (default 2023.2)
#
# Do NOT source this in your interactive shell before running the full
# firmware build: buildroot must not see the Xilinx tools on PATH. The
# Makefile sources it only for the Xilinx steps.

XILINX_VERSION="${XILINX_VERSION:-2023.2}"

if [ -z "${XILINX_DIR:-}" ]; then
	for _xd in /tools/Xilinx /opt/Xilinx "$HOME/Xilinx" /eda/Xilinx; do
		if [ -f "$_xd/Vivado/$XILINX_VERSION/settings64.sh" ]; then
			XILINX_DIR="$_xd"
			break
		fi
	done
	unset _xd
fi

if [ -z "${XILINX_DIR:-}" ] || [ ! -f "$XILINX_DIR/Vivado/$XILINX_VERSION/settings64.sh" ]; then
	echo "xilinx_env.sh: Vivado $XILINX_VERSION not found." >&2
	echo "xilinx_env.sh: set XILINX_DIR to the folder that contains Vivado/$XILINX_VERSION (e.g. /tools/Xilinx)." >&2
	# shellcheck disable=SC2317  # 'exit' only runs if executed instead of sourced
	return 1 2>/dev/null || exit 1
fi

# shellcheck disable=SC1090
source "$XILINX_DIR/Vivado/$XILINX_VERSION/settings64.sh"

# Vitis provides xsct (needed for the FSBL). Its settings64.sh also sets up
# Vivado, so sourcing it after Vivado's is safe.
if [ -f "$XILINX_DIR/Vitis/$XILINX_VERSION/settings64.sh" ]; then
	# shellcheck disable=SC1090
	source "$XILINX_DIR/Vitis/$XILINX_VERSION/settings64.sh"
fi

export XILINX_DIR XILINX_VERSION

# The last command decides the exit status seen by the Makefile.
command -v vivado >/dev/null 2>&1
