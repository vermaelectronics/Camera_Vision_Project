# PI-NLMS HLS project
#
#   make csim   - C simulation with g++ (no Xilinx tools needed)
#   make hls    - csim + csynth + IP export with Vitis HLS (needs vitis_hls on PATH)
#   make cosim  - same as hls plus C/RTL co-simulation
#   make clean

CXX      ?= g++
AP_TYPES := third_party/HLS_arbitrary_Precision_Types
CXXFLAGS := -std=c++14 -O2 -Wno-unknown-pragmas \
            -Isrc -Icsim_include -I$(AP_TYPES)/include
BUILD    := build

.PHONY: all csim hls cosim clean

all: csim

$(AP_TYPES)/include/ap_int.h:
	git clone --depth 1 https://github.com/Xilinx/HLS_arbitrary_Precision_Types.git $(AP_TYPES)

$(BUILD)/csim.exe: src/pi_nlms.cpp src/pi_nlms.h tb/testbench.cpp $(wildcard csim_include/*.h) | $(AP_TYPES)/include/ap_int.h
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) src/pi_nlms.cpp tb/testbench.cpp -o $@

csim: $(BUILD)/csim.exe
	cd $(BUILD) && ./csim.exe

hls:
	vitis_hls -f run_hls.tcl

cosim:
	vitis_hls -f run_hls.tcl -tclargs cosim

clean:
	rm -rf $(BUILD) pi_nlms_prj vitis_hls.log
