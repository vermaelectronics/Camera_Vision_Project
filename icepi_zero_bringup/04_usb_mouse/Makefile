# ============================================================================
# Makefile -- open-source ECP5 flow (Yosys + nextpnr-ecp5 + ecppack)
# ----------------------------------------------------------------------------
# make sim       simulate the TX->RX packet loopback (tb_packet_loopback.v)
# make check     alias for sim
# make wave      run sim then open its .vcd in GTKWave
# make synth     synthesize usb_mouse_top with Yosys
# make pnr       place & route against constraints/icepi_zero.lpf
# make bit       pack a .bit bitstream (needs ecppack from prjtrellis)
# make prog      flash the bitstream with openFPGALoader
# make clean     remove all build artifacts
#
# NOTE: only the PHY layer (usb_nco.v/usb_packet_tx.v/usb_packet_rx.v) is
# simulated here -- usb_host_ctrl.v's enumeration+polling sequence has no
# behavioral USB device model to simulate against in this repo. See the
# README's "Verified" section for exactly what that does and doesn't cover.
# ============================================================================

TOP    = usb_mouse_top
LPF    = constraints/icepi_zero.lpf
BUILD  = build

RTL_SOURCES = rtl/usb_nco.v rtl/usb_packet_tx.v rtl/usb_packet_rx.v \
              rtl/usb_host_ctrl.v rtl/uart_tx.v rtl/usb_mouse_top.v

.PHONY: all sim check wave synth pnr bit prog clean

all: pnr

$(BUILD):
	mkdir -p $(BUILD)

sim: $(BUILD)
	iverilog -g2005 -o $(BUILD)/tb_packet_loopback.vvp \
	  rtl/usb_nco.v rtl/usb_packet_tx.v rtl/usb_packet_rx.v tb/tb_packet_loopback.v
	cd tb && vvp ../$(BUILD)/tb_packet_loopback.vvp

check: sim

wave: sim
	gtkwave tb/tb_packet_loopback.vcd &

synth: $(BUILD)
	yosys -p "read_verilog -sv $(RTL_SOURCES); synth_ecp5 -top $(TOP) -json $(BUILD)/$(TOP).json" \
	          2>&1 | tee $(BUILD)/synth.log

pnr: synth
	nextpnr-ecp5 --25k --package CABGA256 --speed 6 --seed 1 \
	  --json $(BUILD)/$(TOP).json --lpf $(LPF) \
	  --textcfg $(BUILD)/$(TOP).config \
	  2>&1 | tee $(BUILD)/pnr.log

bit: pnr
	ecppack --compress $(BUILD)/$(TOP).config $(BUILD)/$(TOP).bit

prog: bit
	openFPGALoader -b ulx3s $(BUILD)/$(TOP).bit || \
	openFPGALoader $(BUILD)/$(TOP).bit
	@echo "If your board isn't auto-detected, pass -b <board> explicitly --"
	@echo "see 'openFPGALoader --list-boards'."

clean:
	rm -rf $(BUILD) tb/*.vcd
