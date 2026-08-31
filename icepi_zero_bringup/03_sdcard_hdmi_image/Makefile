# ============================================================================
# Makefile -- open-source ECP5 flow (Yosys + nextpnr-ecp5 + ecppack)
# ----------------------------------------------------------------------------
# make sim_scale   simulate the nearest-neighbor scale-address generator
# make sim_bmp     simulate SD SPI + FAT16 + BMP parse -> frame buffer
# make check       run both simulations
# make wave        run sim_bmp then open its .vcd in GTKWave
# make synth       synthesize image_display_top with Yosys
# make pnr         place & route against constraints/icepi_zero.lpf
# make bit         pack a .bit bitstream (needs ecppack from prjtrellis)
# make prog        flash the bitstream with openFPGALoader
# make image       regenerate tb/sd_image_bmp.hex + tb/golden_frame565.hex
# make clean       remove all build artifacts
#
# NOTE: image_display_top instantiates a real ECP5 EHXPLLL primitive (via
# clk_gen_dvi.v) which has no open-source behavioral simulation model --
# same situation as the sibling dvp_camera_hdmi_pipeline project. There is
# therefore no full top-level testbench here; the pixel/HDMI datapath is
# verified via real synthesis + place & route, and the two testable pieces
# (the SD/FAT16/BMP storage stack, and the scale-address generator) are
# simulated on their own.
# ============================================================================

TOP    = image_display_top
LPF    = constraints/icepi_zero.lpf
BUILD  = build

RTL_SOURCES = rtl/spi_master.v rtl/sdcard_spi.v rtl/fat16_reader.v \
              rtl/bmp_frame_loader.v rtl/dp_line_ram.v rtl/nn_scale_addr.v \
              rtl/clk_gen_dvi.v rtl/video_timing_gen.v rtl/tmds_encoder.v \
              rtl/async_fifo.v rtl/tmds_serial_gearbox.v rtl/test_pattern_gen.v \
              rtl/image_display_top.v

.PHONY: all sim_scale sim_bmp check wave synth pnr bit prog image clean

all: pnr

$(BUILD):
	mkdir -p $(BUILD)

image:
	cd tb && python3 gen_bmp_fat16_image.py

sim_scale: $(BUILD)
	iverilog -g2005 -o $(BUILD)/tb_nn_scale_addr.vvp rtl/nn_scale_addr.v tb/tb_nn_scale_addr.v
	cd tb && vvp ../$(BUILD)/tb_nn_scale_addr.vvp

sim_bmp: $(BUILD)
	iverilog -g2005 -o $(BUILD)/tb_bmp_display.vvp \
	  rtl/spi_master.v rtl/sdcard_spi.v rtl/fat16_reader.v rtl/bmp_frame_loader.v rtl/dp_line_ram.v \
	  tb/sd_card_model.v tb/tb_bmp_display.v
	cd tb && vvp ../$(BUILD)/tb_bmp_display.vvp

check: sim_scale sim_bmp

wave: sim_bmp
	gtkwave tb/tb_bmp_display.vcd &

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
