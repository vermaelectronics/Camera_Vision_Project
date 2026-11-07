# ============================================================================
#  xsct_build_sw.tcl  --  Vitis 2023.2 (classic XSCT): platform + FSBL +
#  standalone BSP + app_gnss_e310.elf from the Vivado XSA.
#
#  xsct xsct_build_sw.tcl <workspace> <system_top.xsa> <app source dir>
#
#  Prints "SFAP_SW: PASS <elf>" and "SFAP_SW: FSBL <elf>" at the end.
# ============================================================================
if {$argc < 3} { puts "usage: xsct xsct_build_sw.tcl <ws> <xsa> <src>"; exit 2 }
set ws  [file normalize [lindex $argv 0]]
set xsa [file normalize [lindex $argv 1]]
set src [file normalize [lindex $argv 2]]
set plat e310_sfap_plat
set app  app_gnss_e310

file delete -force $ws
file mkdir $ws
setws $ws

# Platform: standalone on CPU0, plus the zynq_fsbl boot domain (created
# automatically for a Zynq-7000 standalone platform).
platform create -name $plat -hw $xsa -proc ps7_cortexa9_0 -os standalone -out $ws
set dom ""
if {[catch {set dom [domain active]}] || $dom eq ""} { set dom standalone_domain }
puts "SFAP_SW: application domain '$dom'"
platform write
platform generate

app create -name $app -platform $plat -domain $dom -template {Empty Application(C)} -lang c
importsources -name $app -path $src
# the no-OS AD9361 driver links against libm on some paths; harmless otherwise
app config -name $app -add libraries m

# The AD9361 driver allocates its state with malloc: the template's 8 KiB heap
# is too small. 1 MiB heap, 128 KiB stack (all in DDR, far below the capture
# buffers at 0x10000000 and the anti-jam buffer at 0x11000000).
set ld [file join $ws $app src lscript.ld]
set fh [open $ld r]; set t [read $fh]; close $fh
regsub {_STACK_SIZE : 0x[0-9A-Fa-f]+} $t {_STACK_SIZE : 0x20000} t
regsub {_HEAP_SIZE : 0x[0-9A-Fa-f]+}  $t {_HEAP_SIZE : 0x100000} t
set fh [open $ld w]; puts -nonewline $fh $t; close $fh

app build -name $app

set elf [glob -nocomplain [file join $ws $app Debug ${app}.elf] [file join $ws $app Release ${app}.elf]]
if {[llength $elf] == 0} { puts "SFAP_SW: FAIL - no ELF"; exit 1 }
set fsbl [glob -nocomplain [file join $ws $plat export $plat sw $plat boot fsbl.elf] \
                           [file join $ws $plat zynq_fsbl fsbl.elf] \
                           [file join $ws $plat zynq_fsbl Debug fsbl.elf]]
if {[llength $fsbl] == 0} { puts "SFAP_SW: FAIL - no FSBL"; exit 1 }
puts "SFAP_SW: FSBL [lindex $fsbl 0]"
puts "SFAP_SW: PASS [lindex $elf 0]"
exit 0
