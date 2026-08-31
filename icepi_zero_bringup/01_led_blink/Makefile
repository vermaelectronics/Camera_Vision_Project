# ============================================================================
# Makefile -- open-source ECP5 flow (Yosys + nextpnr-ecp5 + ecppack)
# ----------------------------------------------------------------------------
# make sim      simulate tb_led_blink with Icarus Verilog
# make check    same as sim, but named to match the sibling projects' convention
# make wave     run the testbench then open its .vcd in GTKWave
# make synth    synthesize led_blink_top with Yosys
# make pnr      place & route against constraints/icepi_zero.lpf
# make bit      pack a .bit bitstream (needs ecppack from prjtrellis)
# make prog     flash the bitstream with openFPGALoader
# make clean    remove all build artifacts
#
# Tool requirements: iverilog, yosys, nextpnr-ecp5, ecppack/ecppll (prjtrellis
# or the oss-cad-suite bundle), openFPGALoader. Same toolchain as the sibling
# dvp_camera_hdmi_pipeline project in this repo -- see its own Makefile
# header for install notes.
# ============================================================================

TOP    = led_blink_top
LPF    = constraints/icepi_zero.lpf
BUILD  = build

RTL_SOURCES = rtl/led_blink_top.v

.PHONY: all sim check wave synth pnr bit prog clean

all: pnr

$(BUILD):
	mkdir -p $(BUILD)

sim: $(BUILD)
	iverilog -g2005 -o $(BUILD)/tb_led_blink.vvp rtl/led_blink_top.v tb/tb_led_blink.v
	vvp $(BUILD)/tb_led_blink.vvp

check: sim

wave: sim
	gtkwave tb_led_blink.vcd &

synth: $(BUILD)
	yosys -p "read_verilog -sv $(RTL_SOURCES); synth_ecp5 -top $(TOP) -json $(BUILD)/$(TOP).json" \
	          2>&1 | tee $(BUILD)/synth.log

pnr: synth
	nextpnr-ecp5 --25k --package CABGA256 --speed 6 --seed 1 \
	  --json $(BUILD)/$(TOP).json --lpf $(LPF) \
	  --textcfg $(BUILD)/$(TOP).config \
	  2>&1 | tee $(BUILD)/pnr.log
	@echo ""
	@echo "This tiny design should close timing trivially on any seed -- if"
	@echo "it somehow doesn't, something else is wrong (LPF/toolchain), not"
	@echo "placement variance."

bit: pnr
	ecppack --compress $(BUILD)/$(TOP).config $(BUILD)/$(TOP).bit

prog: bit
	openFPGALoader -b ulx3s $(BUILD)/$(TOP).bit || \
	openFPGALoader $(BUILD)/$(TOP).bit
	@echo "If your board isn't auto-detected, pass -b <board> explicitly --"
	@echo "see 'openFPGALoader --list-boards'."

clean:
	rm -rf $(BUILD) *.vcd
