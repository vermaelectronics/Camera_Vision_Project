# Vitis HLS 2021.1 batch script for the PI-NLMS core.
# Usage (from this directory, after: source <Xilinx>/Vitis_HLS/2021.1/settings64.sh):
#   vitis_hls -f run_hls.tcl                # csim + csynth + export IP
#   vitis_hls -f run_hls.tcl -tclargs cosim # also run C/RTL co-simulation
#
# Note: export_design in 2021.1 fails on dates after 2021-12-31 unless the
# Xilinx y2k22 patch (AMD Answer Record 76960) is applied to the install.

open_project -reset pi_nlms_prj
set_top pi_nlms
add_files src/pi_nlms.cpp
add_files src/pi_nlms.h
add_files -tb tb/testbench.cpp -cflags "-Isrc -Wno-unknown-pragmas"

open_solution -reset solution1 -flow_target vivado
set_part {xc7z020clg400-2}
create_clock -period 10 -name default
set_clock_uncertainty 1.25

config_export -format ip_catalog -rtl verilog \
    -vendor Digvijay -library hls -version 1.0 -display_name PI_NLMS_v1 \
    -description "PI-NLMS 2-element null-steering anti-jam core"

csim_design
csynth_design
if {[lsearch $argv cosim] >= 0} {
    cosim_design -rtl verilog -tool xsim
}
if {[catch {export_design -format ip_catalog -rtl verilog} err]} {
    puts "ERROR: export_design failed: $err"
    puts "       If this is Vitis HLS 2021.1, apply the y2k22 patch (AR 76960) and re-run."
    exit 1
}
exit
