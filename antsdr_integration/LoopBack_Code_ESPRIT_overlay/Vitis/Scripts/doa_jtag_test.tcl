# ============================================================================
#  doa_jtag_test.tcl -- ESPRIT DOA hardware test over JTAG with XSDB
#
#  Talks to the DOA registers at 0x43C0_0100 through the Zynq debug access
#  port, so it works whenever the FPGA is programmed and the PS is configured
#  (running firmware, or just FSBL/ps7_init). Independent of the console.
#
#  USAGE (Vitis/XSDB shell, board on JTAG):
#      xsdb Vitis/Scripts/doa_jtag_test.tcl
#  or inside xsdb:   source Vitis/Scripts/doa_jtag_test.tcl
#  Optional:  set DOA_BASE 0x43C00000   before sourcing (default below)
#
#  Prints DOA_JTAG: lines and DOA_JTAG_RESULT: PASS|FAIL.
# ============================================================================
if {![info exists DOA_BASE]} { set DOA_BASE 0x43C00000 }

proc doa_connect {} {
  if {[catch {targets} t] || $t eq ""} { connect }
  # select the first Cortex-A9 so mrd/mwr use the PS memory map
  if {[catch {targets -set -nocase -filter {name =~ "*Cortex-A9*#0"}}]} {
    targets -set -nocase -filter {name =~ "*A9*0*"}
  }
}
proc rd  {off}   { global DOA_BASE; return [mrd -force -value [expr {$DOA_BASE + $off}]] }
proc wr  {off v} { global DOA_BASE; mwr -force [expr {$DOA_BASE + $off}] $v }
proc s32 {v}     { if {$v & 0x80000000} { return [expr {$v - 0x100000000}] } ; return $v }
proc deg {v}     { return [format %8.3f [expr {[s32 $v] / 65536.0}]] }

set fails 0; set checks 0
proc chk {name ok} {
  global fails checks
  incr checks
  if {$ok} { puts [format "DOA_JTAG: %-56s PASS" $name] } else { puts [format "DOA_JTAG: %-56s FAIL" $name]; incr fails }
}
proc wait_avg {n} {
  set reg [expr {([rd 0x194] & 2) ? 0x170 : 0x14C}]
  set b [rd $reg]
  for {set t 0} {$t < 300} {incr t} {
    if {(([rd $reg] - $b) & 0xFFFFFFFF) >= $n} { return 1 }
    after 10
  }
  return 0
}

doa_connect
puts "DOA_JTAG: ===== ESPRIT DOA JTAG test, base [format 0x%08X $DOA_BASE] ====="
chk "gnss_passthrough ID = GCP1"        [expr {[rd 0x000] == 0x47435031}]
chk "gnss_passthrough VERSION = 1.2"    [expr {[rd 0x004] == 0x00010002}]
chk "DOA_ID = DOA1"                     [expr {[rd 0x100] == 0x444F4131}]
set caps [rd 0x194]
puts [format "DOA_JTAG: CAPS 0x%08X (I-UESPRIT %d, M-ESPRIT %d)" $caps [expr {$caps & 1}] [expr {($caps >> 1) & 1}]]

# save state
set sv {}
foreach o {0x108 0x110 0x118 0x11C 0x124 0x128} { lappend sv $o [rd $o] }

# test-generator sweep (d = lambda/2): DPHASE = -sin(theta)/2 * 2^32
wr 0x108 0
wr 0x110 341782638
wr 0x118 0
wr 0x11C 0x00007FFF
wr 0x128 0x00000008
wr 0x108 [expr {0x1 | 0x2 | (2 << 4) | (10 << 8) | (2 << 16)}]
set worst 0.0
foreach {th dph} {-60 0x6ED9EBA1 -30 0x40000000 0 0x00000000 15 0xDEDF047D 45 0xA57D8666} {
  wr 0x124 $dph
  if {![wait_avg 2]} { chk "result for $th deg (timeout)" 0; continue }
  set di [expr {[s32 [rd 0x150]] / 65536.0}]
  set dm [expr {[s32 [rd 0x174]] / 65536.0}]
  puts [format "DOA_JTAG:   set %4d deg -> I-UESPRIT %8.3f  M-ESPRIT %8.3f  |psi| %.4f" $th $di $dm [expr {[s32 [rd 0x164]] / 16777216.0}]]
  foreach v [list $di $dm] { set e [expr {abs($v - $th)}]; if {$e > $worst} { set worst $e } }
}
chk [format "test-generator sweep, worst error %.4f deg < 0.05" $worst] [expr {$worst < 0.05}]

# live RX
wr 0x108 0
wr 0x108 [expr {0x1 | (2 << 4) | (10 << 8) | (4 << 16)}]
after 20
set c0 [rd 0x130]; after 100; set c1 [rd 0x130]
puts "DOA_JTAG: live RX1/RX2 snapshots in ~100 ms: [expr {($c1 - $c0) & 0xFFFFFFFF}]"
chk "live RX samples reach the estimators" [expr {(($c1 - $c0) & 0xFFFFFFFF) > 10000}]
if {[wait_avg 1]} {
  puts "DOA_JTAG: live DOA  I-UESPRIT [deg [rd 0x150]] deg   M-ESPRIT [deg [rd 0x174]] deg   |psi| [format %.4f [expr {[s32 [rd 0x164]] / 16777216.0}]]  off-circle [rd 0x180]"
}

# restore
wr 0x108 0
foreach {o v} $sv { if {$o != 0x108} { wr $o $v } }
wr 0x108 [dict get $sv 0x108]
puts "DOA_JTAG_RESULT: [expr {$fails ? "FAIL" : "PASS"}] ([expr {$checks - $fails}]/$checks)"
