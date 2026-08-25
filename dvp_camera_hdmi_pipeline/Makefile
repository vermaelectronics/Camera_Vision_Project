# ============================================================================
# Makefile -- open-source ECP5 flow (Yosys + nextpnr-ecp5 + ecppack)
# ----------------------------------------------------------------------------
# Targets:
#   make sim            simulate every testbench in tb/ with Icarus Verilog
#                          (stops at the first FAIL -- a Makefile recipe line
#                          failing aborts the target)
#   make check           runs all 5 testbenches regardless of individual
#                          failures, then prints one PASS/FAIL summary table
#                          and exits non-zero if anything failed -- the one
#                          command to run to confirm "everything passes"
#   make wave_<name>       run one testbench then open its .vcd in GTKWave
#                          (or: make wave TB=<name>); valid <name>s: async_fifo,
#                          tmds_encoder, video_timing_gen, dvp_pixel_chain, i2c_master
#   make synth           synthesize dvp_camera_hdmi_top (720p60) with Yosys
#   make synth_1080p30    synthesize dvp_camera_hdmi_top for 1080p30
#   make synth_ext        synthesize dvp_camera_hdmi_top_ext (external-transmitter 1080p60)
#   make pnr               place & route the 720p60 build against constraints/icepi_zero.lpf
#   make bit                pack a .bit bitstream (needs ecppack from prjtrellis, see below)
#   make prog               flash the bitstream with openFPGALoader (or fallback dfu-util/tinyfpgab paths)
#   make clean               remove all build artifacts
#
# Tool requirements (all open-source):
#   iverilog, vvp        (Icarus Verilog)      apt install iverilog
#   yosys                (synthesis)             apt install yosys
#   nextpnr-ecp5          (place & route)         apt install nextpnr-ecp5
#   ecppack, ecppll       (from project trellis)  build from https://github.com/YosysHQ/prjtrellis
#                                                   or install the oss-cad-suite bundle, which ships
#                                                   all four tools prebuilt: https://github.com/YosysHQ/oss-cad-suite-build
#   openFPGALoader         (flashing)              apt install openfpgaloader  (or build from source)
#
# This exact flow (yosys synth_ecp5 -> nextpnr-ecp5 --25k --package CABGA256
# --speed 6 -> ecppack) was used to validate every RTL module in this
# repository during development -- see README.md "Verification" for the
# achieved timing numbers on the real LFE5U-25F-6BG256C part.
# ============================================================================

TOP720        = dvp_camera_hdmi_top
TOP1080P30    = dvp_camera_hdmi_top
TOPEXT        = dvp_camera_hdmi_top_ext
LPF           = constraints/icepi_zero.lpf
BUILD         = build

RTL_SOURCES = \
	rtl/async_fifo.v \
	rtl/tmds_encoder.v \
	rtl/video_timing_gen.v \
	rtl/dvp_capture.v \
	rtl/pixel_formatter.v \
	rtl/video_cdc_buffer.v \
	rtl/clk_gen_dvi.v \
	rtl/clk_gen_1080p60.v \
	rtl/tmds_serial_gearbox.v \
	rtl/test_pattern_gen.v \
	rtl/i2c_master.v \
	rtl/cam_config_rom.v

.PHONY: all sim check synth synth_1080p30 synth_ext pnr pnr_ext bit prog clean wave sim_% wave_%

all: pnr

$(BUILD):
	mkdir -p $(BUILD)

# ---------------------------------------------------------------------------
# Simulation (Icarus Verilog) -- one target per testbench, plus `sim` to run
# all of them. Each testbench calls $dumpfile/$dumpvars, so running any of
# these also (re)writes its .vcd waveform -- see the `wave` targets below.
# ---------------------------------------------------------------------------
TB_async_fifo        = rtl/async_fifo.v tb/tb_async_fifo.v
TB_tmds_encoder       = rtl/tmds_encoder.v tb/tb_tmds_encoder.v
TB_video_timing_gen    = rtl/video_timing_gen.v tb/tb_video_timing_gen.v
TB_dvp_pixel_chain      = rtl/dvp_capture.v rtl/pixel_formatter.v tb/tb_dvp_pixel_chain.v
TB_i2c_master             = rtl/i2c_master.v tb/tb_i2c_master.v
TB_NAMES = async_fifo tmds_encoder video_timing_gen dvp_pixel_chain i2c_master

sim: $(foreach t,$(TB_NAMES),sim_$(t))

sim_%: $(BUILD)
	@echo "== tb_$* =="
	iverilog -g2005 -o $(BUILD)/tb_$*.vvp $(TB_$*)
	vvp $(BUILD)/tb_$*.vvp

# ---------------------------------------------------------------------------
# `make check` -- runs all 5 testbenches, does NOT stop at the first failure
# (unlike `make sim`, which does -- a Makefile recipe line failing aborts
# that target), collects every PASS/FAIL, prints one summary table at the
# end, and exits non-zero if anything failed. This is the one command to
# run to confirm "everything passes" in a single terminal invocation.
# ---------------------------------------------------------------------------
check: $(BUILD)
	@overall=0; \
	run_one() { \
	  name=$$1; shift; \
	  echo ""; \
	  echo "== tb_$$name =="; \
	  if ! iverilog -g2005 -o $(BUILD)/tb_$$name.vvp "$$@" > $(BUILD)/tb_$$name.compile.log 2>&1; then \
	    echo "COMPILE ERROR (see $(BUILD)/tb_$$name.compile.log):"; \
	    cat $(BUILD)/tb_$$name.compile.log; \
	    eval "result_$$name=FAIL_COMPILE"; \
	    overall=1; \
	    return; \
	  fi; \
	  out=$$(vvp $(BUILD)/tb_$$name.vvp 2>&1); \
	  echo "$$out"; \
	  if echo "$$out" | grep -qE "^TB_[A-Z0-9_]+: PASS"; then \
	    eval "result_$$name=PASS"; \
	  else \
	    eval "result_$$name=FAIL"; \
	    overall=1; \
	  fi; \
	}; \
	run_one async_fifo        $(TB_async_fifo); \
	run_one tmds_encoder       $(TB_tmds_encoder); \
	run_one video_timing_gen    $(TB_video_timing_gen); \
	run_one dvp_pixel_chain      $(TB_dvp_pixel_chain); \
	run_one i2c_master             $(TB_i2c_master); \
	echo ""; \
	echo "============================================================"; \
	echo " SUMMARY"; \
	echo "============================================================"; \
	for n in async_fifo tmds_encoder video_timing_gen dvp_pixel_chain i2c_master; do \
	  eval "r=\$$result_$$n"; \
	  printf "  %-20s %s\n" "$$n" "$$r"; \
	done; \
	echo "============================================================"; \
	if [ "$$overall" -eq 0 ]; then \
	  echo " ALL 5 TESTBENCHES PASSED"; \
	else \
	  echo " AT LEAST ONE TESTBENCH FAILED -- see log above"; \
	fi; \
	echo "============================================================"; \
	exit $$overall

# ---------------------------------------------------------------------------
# Waveform viewing (Icarus Verilog + GTKWave)
# ---------------------------------------------------------------------------
# `make wave_<name>` (re)runs that one testbench, then opens the .vcd it
# just wrote in GTKWave. `make wave TB=<name>` does the same thing via a
# variable, e.g.:
#   make wave TB=tmds_encoder
# Valid <name> values: async_fifo, tmds_encoder, video_timing_gen,
# dvp_pixel_chain, i2c_master (same names as the sim_<name> targets above).
# Icarus writes each testbench's .vcd to the directory `make` was invoked
# from (the project root), not into $(BUILD) -- that's where GTKWave is
# pointed below. Needs `gtkwave` on PATH (`apt install gtkwave`, or it's
# bundled in oss-cad-suite).
wave_%: sim_%
	gtkwave tb_$*.vcd &

wave:
	@if [ -z "$(TB)" ]; then \
	  echo "usage: make wave TB=<async_fifo|tmds_encoder|video_timing_gen|dvp_pixel_chain|i2c_master>"; \
	  exit 1; \
	fi
	$(MAKE) wave_$(TB)

# ---------------------------------------------------------------------------
# Synthesis
# ---------------------------------------------------------------------------
synth: $(BUILD)
	yosys -p "read_verilog -sv $(RTL_SOURCES) rtl/$(TOP720).v; \
	          synth_ecp5 -top $(TOP720) -json $(BUILD)/$(TOP720)_720p60.json" \
	          2>&1 | tee $(BUILD)/synth_720p60.log

synth_1080p30: $(BUILD)
	yosys -p "read_verilog -sv $(RTL_SOURCES); \
	          chparam -set RESOLUTION 1080P30 dvp_camera_hdmi_top 2>/dev/null; \
	          read_verilog -sv -Dresolution_1080p30 rtl/$(TOP1080P30).v; \
	          synth_ecp5 -top $(TOP1080P30) -json $(BUILD)/$(TOP1080P30)_1080p30.json" \
	          2>&1 | tee $(BUILD)/synth_1080p30.log
	@echo "NOTE: chparam string-parameter overrides can be finicky from the CLI."
	@echo "      If this target errors, instantiate dvp_camera_hdmi_top with"
	@echo "      #(.RESOLUTION(\"1080P30\")) from a tiny wrapper module instead"
	@echo "      (see README.md 'Building the 1080p30 variant')."

synth_ext: $(BUILD)
	yosys -p "read_verilog -sv $(RTL_SOURCES) rtl/$(TOPEXT).v; \
	          synth_ecp5 -top $(TOPEXT) -json $(BUILD)/$(TOPEXT).json" \
	          2>&1 | tee $(BUILD)/synth_ext.log

# ---------------------------------------------------------------------------
# Place & route + static timing analysis against the real LFE5U-25F-6BG256C
# ---------------------------------------------------------------------------
pnr: synth
	nextpnr-ecp5 --25k --package CABGA256 --speed 6 --seed 4 \
	  --json $(BUILD)/$(TOP720)_720p60.json --lpf $(LPF) \
	  --textcfg $(BUILD)/$(TOP720)_720p60.config \
	  2>&1 | tee $(BUILD)/pnr_720p60.log
	@echo ""
	@echo "If 'Max frequency for clock clk_pixel' reports FAIL, re-run with a"
	@echo "different --seed N (place & route has run-to-run variance) -- this"
	@echo "was verified to reliably pass with margin across several seeds"
	@echo "during development, see README.md 'Timing closure notes'."

pnr_ext: synth_ext
	nextpnr-ecp5 --25k --package CABGA256 --speed 6 --seed 4 \
	  --json $(BUILD)/$(TOPEXT).json --lpf $(LPF) \
	  --textcfg $(BUILD)/$(TOPEXT).config \
	  2>&1 | tee $(BUILD)/pnr_ext.log
	@echo "NOTE: the external-transmitter hdmi_* pins are NOT assigned real"
	@echo "sites in constraints/icepi_zero.lpf by default (commented out) --"
	@echo "edit that file to match how you actually wire your transmitter"
	@echo "breakout before running this target for real, or pass"
	@echo "--lpf-allow-unconstrained to nextpnr-ecp5 for a free-placement"
	@echo "timing-only check."

# ---------------------------------------------------------------------------
# Bitstream packing (needs ecppack from prj-trellis; not installed by apt)
# ---------------------------------------------------------------------------
bit: pnr
	ecppack --compress $(BUILD)/$(TOP720)_720p60.config $(BUILD)/$(TOP720)_720p60.bit

prog: bit
	openFPGALoader -b ulx3s $(BUILD)/$(TOP720)_720p60.bit || \
	openFPGALoader $(BUILD)/$(TOP720)_720p60.bit
	@echo "If your board isn't auto-detected, pass -b <board> explicitly --"
	@echo "see 'openFPGALoader --list-boards'."

clean:
	rm -rf $(BUILD) *.vcd
