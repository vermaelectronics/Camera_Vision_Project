# ============================================================================
# Makefile -- open-source ECP5 flow (Yosys + nextpnr-ecp5 + ecppack)
# ----------------------------------------------------------------------------
# make sim      simulate the FAT16-mount-and-stream integration test
# make sim_top  simulate the full top-level (SD SPI + FAT16 + FIFO + UART)
# make uart     simulate uart_tx alone
# make check    run all three simulations
# make wave     run sim_top then open its .vcd in GTKWave
# make synth    synthesize sdcard_text_reader_top with Yosys
# make pnr      place & route against constraints/icepi_zero.lpf
# make bit      pack a .bit bitstream (needs ecppack from prjtrellis)
# make prog     flash the bitstream with openFPGALoader
# make image    regenerate tb/sd_image.hex + tb/expected_content.hex
# make clean    remove all build artifacts
#
# Tool requirements: iverilog, yosys, nextpnr-ecp5, ecppack/ecppll (prjtrellis
# or the oss-cad-suite bundle), openFPGALoader, python3 (for `make image`
# only -- not needed for the real hardware build).
# ============================================================================

TOP    = sdcard_text_reader_top
LPF    = constraints/icepi_zero.lpf
BUILD  = build

RTL_SOURCES = rtl/spi_master.v rtl/sdcard_spi.v rtl/fat16_reader.v \
              rtl/sync_fifo.v rtl/uart_tx.v rtl/sdcard_text_reader_top.v

.PHONY: all sim sim_top uart check wave synth pnr bit prog image clean

all: pnr

$(BUILD):
	mkdir -p $(BUILD)

image:
	cd tb && python3 gen_fat16_image.py

sim: $(BUILD)
	iverilog -g2005 -o $(BUILD)/tb_sdcard_text_reader.vvp \
	  rtl/spi_master.v rtl/sdcard_spi.v rtl/fat16_reader.v \
	  tb/sd_card_model.v tb/tb_sdcard_text_reader.v
	cd tb && vvp ../$(BUILD)/tb_sdcard_text_reader.vvp

sim_top: $(BUILD)
	iverilog -g2005 -o $(BUILD)/tb_sdcard_text_reader_top.vvp \
	  $(RTL_SOURCES) tb/sd_card_model.v tb/tb_sdcard_text_reader_top.v
	cd tb && vvp ../$(BUILD)/tb_sdcard_text_reader_top.vvp

uart: $(BUILD)
	iverilog -g2005 -o $(BUILD)/tb_uart_tx.vvp rtl/uart_tx.v tb/tb_uart_tx.v
	cd tb && vvp ../$(BUILD)/tb_uart_tx.vvp

check: sim sim_top uart

wave: sim_top
	gtkwave tb/tb_sdcard_text_reader_top.vcd &

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
