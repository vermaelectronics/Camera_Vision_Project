// ============================================================================
// lattice_scuba_stubs.v -- trivial behavioral stand-ins for VHI/VLO, the
// fixed logic-high/logic-low tie primitives Lattice Diamond's "scuba" IP
// generator (Clarity Designer) emits alongside a Diamond-generated PLL
// wrapper (see icepi_clk_wiz_sys.v/icepi_clk_wiz_video.v's own header
// comments -- both are real Diamond/Clarity Designer output, reused as-is
// per this project's README).
// ----------------------------------------------------------------------------
// VHI/VLO are part of Diamond's own synthesis library, not the open-source
// Project Trellis ECP5 primitive set Yosys knows about -- without this
// file, `synth_ecp5` fails immediately with "Module `\VLO' ... is not part
// of the design." Synthesizing these as plain constant-driving modules is
// the standard, well-known fix for bringing Diamond/Clarity-Designer PLL
// output into the open-source Yosys/nextpnr-ecp5 flow: Yosys's own constant
// propagation collapses each instantiation away during optimization, so
// this costs zero real logic -- it only exists to satisfy elaboration.
// Only needed for synthesis; simulation never reaches these (both PLL
// wrappers are entirely bypassed under `` `ifdef SIMULATION `` in favor of
// behavioral clock/lock stand-ins).
// ============================================================================
`default_nettype none

module VHI (output wire Z);
    assign Z = 1'b1;
endmodule

module VLO (output wire Z);
    assign Z = 1'b0;
endmodule

`default_nettype wire
