# ============================================================================
# Makefile -- open-source ECP5 flow (OSS CAD Suite: Yosys + nextpnr-ecp5 +
# Project Trellis ecppack + openFPGALoader), plus Icarus Verilog simulation
# and GTKWave waveform viewing for all four testbenches.
# ----------------------------------------------------------------------------
# Quick start:
#   make check      run all four testbenches, print PASS/FAIL for each
#   make wave-top    run tb_sd_image_hdmi_top_smoke then open its .vcd
#   make prog        synth + place&route + pack + flash, in one step
#
# Tool requirements (all provided by the OSS CAD Suite bundle --
# https://github.com/YosysHQ/oss-cad-suite-build):
#   iverilog, vvp, gtkwave  (simulation + waveform viewing)
#   yosys, nextpnr-ecp5, ecppack/ecppll (prjtrellis)  (synthesis/P&R/pack)
#   openFPGALoader                                     (flashing)
#
#   # one-time setup:
#   wget https://github.com/YosysHQ/oss-cad-suite-build/releases/latest/download/oss-cad-suite-linux-x64-<date>.tgz
#   tar xzf oss-cad-suite-linux-x64-*.tgz
#   source oss-cad-suite/environment
# ============================================================================

TOP    = sd_image_hdmi_top
LPF    = constraints/icepi_zero_hdmi.lpf
BUILD  = build

# Both PLL wrappers are Diamond/Clarity-Designer output; their scuba_vhi/
# scuba_vlo ties are now plain `assign` statements (see icepi_clk_wiz_sys.v /
# icepi_clk_wiz_video.v) rather than VHI/VLO primitive instantiations, so no
# separate stub source is needed here -- some OSS CAD Suite builds already
# define VHI/VLO inside synth_ecp5's own ECP5 cell library, and a same-named
# stub module read in separately collided with that on those builds.
RTL_SOURCES = rtl/icepi_clk_wiz_sys.v rtl/icepi_clk_wiz_video.v rtl/clk_reset_gen.v \
              rtl/spi_master.v rtl/sd_spi_init.v rtl/sd_block_read.v rtl/fat_reader.v \
              rtl/img_loader.v rtl/framebuffer.v rtl/video_timing.v rtl/tmds_encoder.v \
              rtl/tmds_serializer.v rtl/hdmi_out.v rtl/status_led.v rtl/sd_image_hdmi_top.v

.PHONY: all check clean synth pnr bit prog \
        sim-timing sim-tmds sim-img sim-top \
        wave-timing wave-tmds wave-img wave-top

all: pnr

$(BUILD):
	mkdir -p $(BUILD)

# ---------------------------------------------------------------------------
# Simulation -- one target per testbench, matching the exact commands
# documented in README.md's own "Simulation" section. Each testbench writes
# its own .vcd (via $dumpfile/$dumpvars) into $(BUILD)/ for `make wave-*`.
# ---------------------------------------------------------------------------

sim-timing: $(BUILD)
	iverilog -g2012 -o $(BUILD)/tb_video_timing.vvp \
	  rtl/video_timing.v tb/tb_video_timing.v
	cd $(BUILD) && vvp tb_video_timing.vvp

sim-tmds: $(BUILD)
	iverilog -g2012 -o $(BUILD)/tb_tmds_encoder.vvp \
	  rtl/tmds_encoder.v tb/tb_tmds_encoder.v
	cd $(BUILD) && vvp tb_tmds_encoder.vvp

sim-img: $(BUILD)
	iverilog -g2012 -DSIMULATION -o $(BUILD)/tb_framebuffer_img_loader.vvp \
	  rtl/spi_master.v rtl/sd_spi_init.v rtl/sd_block_read.v rtl/fat_reader.v \
	  rtl/img_loader.v rtl/framebuffer.v tb/tb_framebuffer_img_loader.v
	cd $(BUILD) && vvp tb_framebuffer_img_loader.vvp

sim-top: $(BUILD)
	iverilog -g2012 -DSIMULATION -o $(BUILD)/tb_sd_image_hdmi_top_smoke.vvp \
	  $(RTL_SOURCES) tb/tb_sd_image_hdmi_top_smoke.v
	cd $(BUILD) && vvp tb_sd_image_hdmi_top_smoke.vvp

# run every testbench; make -k so one failure doesn't hide the rest
check:
	$(MAKE) -k sim-timing sim-tmds sim-img sim-top

# ---------------------------------------------------------------------------
# GTKWave -- opens the .vcd each sim-* target above just produced. Run the
# matching sim-* target first (each wave-* target does that automatically).
# ---------------------------------------------------------------------------

wave-timing: sim-timing
	gtkwave $(BUILD)/tb_video_timing.vcd &

wave-tmds: sim-tmds
	gtkwave $(BUILD)/tb_tmds_encoder.vcd &

wave-img: sim-img
	gtkwave $(BUILD)/tb_framebuffer_img_loader.vcd &

wave-top: sim-top
	gtkwave $(BUILD)/tb_sd_image_hdmi_top_smoke.vcd &

# ---------------------------------------------------------------------------
# Synthesis / place & route / pack / flash -- real OSS CAD Suite toolchain.
# ---------------------------------------------------------------------------

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
	rm -rf $(BUILD)
