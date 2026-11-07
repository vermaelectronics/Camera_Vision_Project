# ============================================================================
#  xsct_jtag_run.tcl  --  development run over JTAG (no SD card):
#  program the bitstream, initialise the PS, download and start the ELF.
#  xsct xsct_jtag_run.tcl <system_top.bit> <ps7_init.tcl> <app.elf>
#  Boot-mode jumpers on JTAG. Console: UART1 at 115200 8N1.
# ============================================================================
set bit [lindex $argv 0]; set init [lindex $argv 1]; set elf [lindex $argv 2]
connect
targets -set -nocase -filter {name =~ "APU*"}
rst -system
after 2000
targets -set -nocase -filter {name =~ "xc7z0*"}
fpga -file $bit
targets -set -nocase -filter {name =~ "APU*"}
source $init
ps7_init
ps7_post_config
targets -set -nocase -filter {name =~ "*A9*#0"}
rst -processor
dow $elf
con
puts "SFAP_JTAG: running [file tail $elf]"
exit 0
