# gnss_passthrough v1.3 (PI RTL core + PI-NLMS HLS core) -- simulation
#
#   make sim HLS_RTL=<dir with HLS-generated pi_nlms*.v>
#       e.g. HLS_RTL=~/PI_NLMS_2021/PI_NLMS_hls/solution1/syn/verilog
#
#   make sim_nlms   NLMS mode: bit-exact vs C model, suppression, DAC path
#   make sim_pi     original v1.2 testbench (PI core selected) -- regression
#
# Needs g++ and Icarus Verilog (sudo apt install iverilog).

HLS_RTL  ?= $(error set HLS_RTL to the HLS-generated Verilog directory)
N        ?= 20480
MU       ?= -3
BUILD    := build
AP_TYPES := ../third_party/HLS_arbitrary_Precision_Types
CXXFLAGS := -std=c++11 -O2 -Wno-unknown-pragmas -I../src -I../csim_include -I$(AP_TYPES)/include
RTL      := rtl/gnss_passthrough.v rtl/pi_power_inversion.v rtl/pi_cmul.v

.PHONY: sim sim_nlms sim_pi clean

sim: sim_pi sim_nlms

$(AP_TYPES)/include/ap_int.h:
	$(MAKE) -C .. third_party/HLS_arbitrary_Precision_Types/include/ap_int.h

$(BUILD)/gen_vectors: gen_vectors.cpp ../src/pi_nlms.cpp ../src/pi_nlms.h | $(AP_TYPES)/include/ap_int.h
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) ../src/pi_nlms.cpp gen_vectors.cpp -o $@

$(BUILD)/stim.hex: $(BUILD)/gen_vectors
	cd $(BUILD) && ./gen_vectors $(N) $(MU) .

sim_nlms: $(BUILD)/stim.hex
	iverilog -g2012 -P tb_gnss_passthrough_nlms.N_SAMPLES=$(N) -P tb_gnss_passthrough_nlms.MU_SHIFT=$(MU) \
	    -o $(BUILD)/tb_nlms tb/tb_gnss_passthrough_nlms.v $(RTL) $(wildcard $(HLS_RTL)/*.v)
	cd $(BUILD) && vvp -n tb_nlms

sim_pi:
	@mkdir -p $(BUILD)
	iverilog -g2012 -o $(BUILD)/tb_pi tb/tb_gnss_passthrough.v $(RTL) $(wildcard $(HLS_RTL)/*.v)
	vvp -n $(BUILD)/tb_pi

clean:
	rm -rf $(BUILD)
