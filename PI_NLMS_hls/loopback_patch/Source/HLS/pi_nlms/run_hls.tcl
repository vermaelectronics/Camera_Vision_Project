# ============================================================================
#  run_hls.tcl  --  Vitis HLS 2021.1 build of the PI-NLMS core used inside
#  gnss_passthrough (Source/HDL/gnss_passthrough.v, core_sel = 2).
#
#  INVOCATION (from anywhere)
#    vitis_hls -f <root>/Source/HLS/pi_nlms/run_hls.tcl -tclargs <root>
#      <root>  absolute path of LoopBack_Code
#
#  OUTPUT
#    <root>/Build/hls/pi_nlms_prj/          the Vitis HLS project
#    <root>/Build/hls/pi_nlms/verilog/*.v   RTL consumed by gnss_passthrough_ip.tcl
#
#  CLOCK AND THROUGHPUT
#    The core runs on axi_ad9361/l_clk, constrained as rx_clk = 8 ns. In
#    AD9361 2R2T LVDS mode adc_valid is high every second l_clk, so the core is
#    built with II=2 (PI_NLMS_II) for timing margin at 8 ns. gnss_passthrough
#    latches a refused sample in STATUS[12] if that assumption is ever broken.
# ============================================================================

if {[llength $argv] < 1} {
  puts "HLS_RESULT: FAIL - usage: vitis_hls -f run_hls.tcl -tclargs <LoopBack_Code root>"
  exit 2
}
set root [file normalize [lindex $argv end]]
set src  [file join $root Source HLS pi_nlms]
set out  [file join $root Build hls]
set ii   2
set period 8

foreach f [list $src/pi_nlms.cpp $src/pi_nlms.h $src/testbench.cpp] {
  if {![file exists $f]} {
    puts "HLS_RESULT: FAIL - missing source: $f"
    exit 2
  }
}

file mkdir $out
cd $out

open_project -reset pi_nlms_prj
set_top pi_nlms
add_files $src/pi_nlms.cpp -cflags "-DPI_NLMS_II=$ii"
add_files $src/pi_nlms.h
add_files -tb $src/testbench.cpp -cflags "-I$src -Wno-unknown-pragmas"

open_solution -reset solution1 -flow_target vivado
set_part {xc7z020clg400-2}
create_clock -period $period -name default

csim_design
csynth_design

# ---- check what synthesis achieved -----------------------------------------
set rpt [file join $out pi_nlms_prj solution1 syn report pi_nlms_csynth.xml]
set fh [open $rpt r]; set xml [read $fh]; close $fh
regexp {<EstimatedClockPeriod>([0-9.]+)</EstimatedClockPeriod>} $xml -> est
regexp {<Interval-max>([0-9]+)</Interval-max>} $xml -> ii_got
puts "HLS_TIMING: target ${period}.000 ns, estimated $est ns, interval $ii_got (wanted $ii)"
if {$ii_got > $ii} {
  puts "HLS_RESULT: FAIL - pipeline interval $ii_got > $ii: the core could not keep up with 2R2T samples."
  puts "HLS_HINT: check the log for 'Pipelining result : Target II = $ii'."
  exit 2
}
if {$est > $period} {
  puts "HLS_WARNING: estimated clock period $est ns exceeds $period ns -- check Vivado timing after implementation."
}

# ---- hand the RTL to the IP packager ----------------------------------------
set rtl_out [file join $out pi_nlms verilog]
file delete -force $rtl_out
file mkdir $rtl_out
foreach f [glob [file join $out pi_nlms_prj solution1 syn verilog *.v]] {
  file copy -force $f $rtl_out
}
puts "HLS_RTL: $rtl_out ([llength [glob $rtl_out/*.v]] files)"
puts "HLS_RESULT: PASS"
exit 0
