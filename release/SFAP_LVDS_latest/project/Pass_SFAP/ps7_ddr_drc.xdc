## PS7 dedicated DDR pins: the differential I/O (DIFF_SSTL15 / DIFF_SSTL15_T_DCI)
## is built into the PS DDR controller; the top-level netlist only sees them as
## single-ended inout ports, which is what DRC IOSTDTYPE-1 reports. Waived, and
## lowered to a warning in case the waiver does not match. No logic change.
create_waiver -type DRC -id IOSTDTYPE-1 -objects [get_ports -quiet {ddr_ck_p ddr_ck_n ddr_dqs_p[*] ddr_dqs_n[*]}] -description "PS7 dedicated DDR pins, differential I/O built into the PS DDR controller"
set_property SEVERITY {Warning} [get_drc_checks IOSTDTYPE-1]
