# Adds the PI-AJ v5 RTL to the open Vivado project (2023.2).
#   source <path>/pi_aj_v5/vivado/add_pi_aj.tcl
# Then instantiate pi_aj_top in the block design as an RTL module
# (right-click canvas -> Add Module... -> pi_aj_top), or in your HDL top.
set here [file dirname [file normalize [info script]]]
set rtl  [file join $here .. rtl]
add_files -norecurse [list \
    [file join $rtl pi_aj_frontend.v] \
    [file join $rtl pi_aj_notch.v] \
    [file join $rtl pi_aj_wlpi.v] \
    [file join $rtl pi_aj_outagc.v] \
    [file join $rtl pi_aj_cdc_bus.v] \
    [file join $rtl pi_aj_axil_regs.v] \
    [file join $rtl pi_aj_top.v] ]
update_compile_order -fileset sources_1
puts "PI-AJ v5 sources added. Top module: pi_aj_top"
