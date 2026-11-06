# ============================================================================
#  patch_vendor_constraints.tcl  --  SMI-PI 4.9.13, Vivado methodology TIMING-54
#
#  WHAT IT DOES
#    Edits two constraint files of the DISPOSABLE vendor working copy
#    (Build/VendorWork/hdl, made by build_all.sh stage 1). Vendor/ itself is
#    never touched (requirements 34, 71).
#
#      library/xilinx/util_clkdiv/util_clkdiv_constr.xdc
#        removes 1 statement:  set_clock_groups -logically_exclusive between the
#                              two divided clocks clk_div_sel_0_s / clk_div_sel_1_s
#      library/axi_dmac/axi_dmac_constr.ttcl
#        removes 3 statements: the set_max_delay -datapath_only constraints that
#                              go FROM one clock, -through a distributed RAM
#                              (eot_mem_src, burst_len_mem, eot_mem_dest), TO
#                              another clock
#
#    Each removed statement stays in the file as a comment, after a marker line.
#
#  WHY (the 7 TIMING-54 critical warnings of 4.9.9 .. 4.9.12)
#    TIMING-54 = "Scoped false path, clock group or max delay datapath only
#    constraint between clocks". Both files are packaged with their IP and
#    Vivado applies them SCOPED to the IP instance. Clocks are global objects,
#    so a constraint between two clocks in a scoped file acts on every path
#    between those clocks in the whole design, and Vivado flags it. In this
#    design that was:
#      util_clkdiv (util_ad9361_divclk)  1  the logically-exclusive clock group
#      axi_ad9361_adc_dma                3  eot_mem_src (request -> source),
#                                           burst_len_mem and eot_mem_dest
#                                           (source -> destination)
#      axi_ad9361_dac_dma                2  burst_len_mem and eot_mem_dest
#      gnss_passthrough (our own IP)     1  set_clock_groups in its packaged XDC
#                                           (removed in Source/IP, 4.9.13)
#                                       ---
#                                        7
#
#    What replaces them:
#      util_clkdiv: the identical clock group, unscoped, in
#        Source/XDC/gnss_passthrough_cdc.xdc (section 5).
#      axi_dmac: nothing is needed. Every one of those paths runs between
#        clk_fpga_0 (sys_cpu_clk) and the clk_div_sel_* clocks, and the project
#        already declares those clocks asynchronous (set_clock_groups, also in
#        gnss_passthrough_cdc.xdc, since 4.9.9). A clock group takes precedence
#        over set_max_delay, so these three constraints had no effect on the
#        timing analysis of this design; removing them changes no timing.
#
#  USAGE
#    tclsh  patch_vendor_constraints.tcl <hdl working copy root>
#    vivado -mode batch -source patch_vendor_constraints.tcl -tclargs <root>
#    or, from another script:  set ::gnss_vp_no_main 1; source <this file>;
#                              gnss_patch_vendor_lib <library dir> <library name>
#
#    Idempotent: a file that already carries the marker is left alone. If the
#    vendor files do not contain exactly the expected statements (another ADI
#    release), nothing is written and the script FAILS, so a changed vendor tree
#    cannot silently bring the warnings back or remove the wrong constraint.
#    Prints VENDOR_PATCH: PASS or VENDOR_PATCH: FAIL - <reason>.
# ============================================================================

namespace eval ::gnss_vp {
  variable marker "GNSS-CRPA 4.9.13 TIMING-54"
}

# A statement = one line plus the lines it continues onto with a trailing backslash.
proc ::gnss_vp::continues {line} {
  return [regexp {\\\r?$} $line]
}

proc ::gnss_vp::drop_clkdiv {stmt} {
  return [regexp {^\s*set_clock_groups\M} $stmt]
}

proc ::gnss_vp::drop_dmac {stmt} {
  # set_max_delay ... -from $<x>_clk ... -through [...] ... -to $<y>_clk ...
  return [expr {[regexp {^\s*set_max_delay\M} $stmt]
                && [regexp -- {-through\M} $stmt]
                && [regexp -- {-from\s+\$[a-z_]+_clk\M} $stmt]
                && [regexp -- {-to\s+\$[a-z_]+_clk\M} $stmt]}]
}

# Returns the number of removed statements, or "already" if the file was patched before.
proc ::gnss_vp::patch_file {path drop expected} {
  variable marker
  if {![file exists $path]} { return -code error "$path not found" }
  set fh [open $path r]
  fconfigure $fh -translation binary
  set text [read $fh]
  close $fh
  if {[string first $marker $text] >= 0} { return already }

  set out  {}
  set stmt {}
  set n    0
  foreach line [split $text "\n"] {
    lappend stmt $line
    if {[continues $line]} { continue }
    if {[$drop [join $stmt "\n"]]} {
      incr n
      lappend out "# $marker: removed from this IP-scoped file by Vivado/Scripts/patch_vendor_constraints.tcl"
      foreach l $stmt { lappend out "#   [string trimright $l " \t\r\\"]" }
    } else {
      foreach l $stmt { lappend out $l }
    }
    set stmt {}
  }
  foreach l $stmt { lappend out $l }

  if {$n != $expected} {
    return -code error "$path: expected $expected statement(s) to remove, found $n (different ADI release?); nothing written"
  }
  set fh [open $path w]
  fconfigure $fh -translation binary
  puts -nonewline $fh [join $out "\n"]
  close $fh
  return $n
}

# Patch one library directory (used by package_library_ip.tcl, which runs inside it).
proc gnss_patch_vendor_lib {libdir libname} {
  switch -- $libname {
    util_clkdiv { return [::gnss_vp::patch_file [file join $libdir util_clkdiv_constr.xdc] ::gnss_vp::drop_clkdiv 1] }
    axi_dmac    { return [::gnss_vp::patch_file [file join $libdir axi_dmac_constr.ttcl]   ::gnss_vp::drop_dmac   3] }
    default     { return none }
  }
}

# Patch the whole working copy (used by build_all.sh stage 1).
proc gnss_patch_vendor_tree {root} {
  foreach {rel name} {library/xilinx/util_clkdiv util_clkdiv  library/axi_dmac axi_dmac} {
    set r [gnss_patch_vendor_lib [file join $root $rel] $name]
    puts "VENDOR_PATCH: $rel/ -> [expr {$r eq "already" ? "already patched" : "$r statement(s) moved out"}]"
  }
}

if {![info exists ::gnss_vp_no_main]} {
  if {$::argc < 1} {
    puts "VENDOR_PATCH: FAIL - usage: patch_vendor_constraints.tcl <hdl working copy root>"
    exit 2
  }
  if {[catch {gnss_patch_vendor_tree [lindex $::argv 0]} err]} {
    puts "VENDOR_PATCH: FAIL - $err"
    exit 2
  }
  puts "VENDOR_PATCH: PASS"
  exit 0
}
