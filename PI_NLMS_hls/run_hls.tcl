# Vitis HLS 2023.2 batch script for the PI-NLMS core.
# Usage (from this directory):
#   vitis_hls -f run_hls.tcl                # csim + csynth + export IP
#   vitis_hls -f run_hls.tcl -tclargs cosim # also run C/RTL co-simulation

open_project -reset pi_nlms_prj
set_top pi_nlms
add_files src/pi_nlms.cpp
add_files src/pi_nlms.h
add_files -tb tb/testbench.cpp -cflags "-Isrc -Wno-unknown-pragmas"

open_solution -reset solution1 -flow_target vivado
set_part xc7z020clg400-2
create_clock -period 10 -name default
set_clock_uncertainty 1.25

config_cosim -tool xsim
config_export -format ip_catalog -rtl verilog \
    -vendor Digvijay -version 1.0 -display_name PI_NLMS_v1 \
    -description "PI-NLMS 2-element null-steering anti-jam core"

csim_design
csynth_design
if {[lsearch $argv cosim] >= 0} {
    cosim_design
}
export_design -format ip_catalog
exit
