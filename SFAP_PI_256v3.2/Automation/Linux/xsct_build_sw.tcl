# ============================================================================
#  xsct_build_sw.tcl  --  Vitis 2023.2 (classic XSCT flow): platform + FSBL +
#  standalone BSP + app_sfap_lvds.elf from the Vivado XSA.
#
#  xsct xsct_build_sw.tcl <workspace> <system_top.xsa> <app source dir> [app name]
#
#  Prints "SFAP_SW: PASS <elf>" and "SFAP_SW: FSBL <elf>" at the end.
# ============================================================================
if {$argc < 3} { puts "usage: xsct xsct_build_sw.tcl <ws> <xsa> <src>"; exit 2 }
set ws  [file normalize [lindex $argv 0]]
set xsa [file normalize [lindex $argv 1]]
set src [file normalize [lindex $argv 2]]
set plat e310_lvds_plat
set app  app_sfap_lvds
if {$argc >= 4} { set app [lindex $argv 3] }

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

# the template's name differs between Vitis releases
if {[catch {app create -name $app -platform $plat -domain $dom -template {Empty Application(C)} -lang c}]} {
  app create -name $app -platform $plat -domain $dom -template {Empty Application} -lang c
}
importsources -name $app -path $src
# Release build (-O2): the anti-jam solver runs 2-3x faster than in the
# Debug (-O0) build, i.e. more weight updates per second. SFAP_SW_CONFIG=debug
# selects the Debug build (for stepping through the code in the debugger).
set cfg release
if {[info exists ::env(SFAP_SW_CONFIG)] && [string tolower $::env(SFAP_SW_CONFIG)] eq "debug"} { set cfg debug }
app config -name $app build-config $cfg
puts "SFAP_SW: application build configuration '$cfg'"
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

set cdir [expr {$cfg eq "debug" ? "Debug" : "Release"}]
set elf [glob -nocomplain [file join $ws $app $cdir ${app}.elf]]
if {[llength $elf] == 0} { puts "SFAP_SW: FAIL - no ELF"; exit 1 }
set fsbl [glob -nocomplain [file join $ws $plat export $plat sw $plat boot fsbl.elf] \
                           [file join $ws $plat zynq_fsbl fsbl.elf] \
                           [file join $ws $plat zynq_fsbl Debug fsbl.elf]]
if {[llength $fsbl] == 0} { puts "SFAP_SW: FAIL - no FSBL"; exit 1 }
puts "SFAP_SW: FSBL [lindex $fsbl 0]"
puts "SFAP_SW: PASS [lindex $elf 0]"
exit 0
