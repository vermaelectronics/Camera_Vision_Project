// ============================================================================
// clk_gen_mclk.v -- EHXPLLL wrapper: 50MHz board oscillator -> camera MCLK/XCLK
// ----------------------------------------------------------------------------
// Many DVP camera modules (including the Waveshare OV5640 board this
// project targets) have no onboard crystal -- they need the host to supply
// a master clock (MCLK/XCLK) before their internal PLL, I2C interface, or
// pixel output will work at all. 24 MHz is the OV5640's standard/most
// widely-used XCLK frequency (the value assumed by essentially every
// published OV5640 init register table, including cam_config_rom.v's).
//
// ---- Divider derivation (Fvco = Fin * CLKFB_DIV / CLKI_DIV * CLKOP_DIV) ---
//   Fin        = 50 MHz (board reference oscillator)
//   CLKI_DIV   = 25   -> PFD = 50/25 = 2.0 MHz
//   CLKFB_DIV  = 12   -> CLKOP = 50*12/25 = 24.000 MHz EXACT (no PLL-locked
//                        frequency error at all -- 50MHz divides evenly by
//                        this ratio, unlike the DVI pixel-clock PLLs
//                        elsewhere in this project)
//   CLKOP_DIV  = 20   -> Fvco = CLKOP * CLKOP_DIV = 480 MHz (within the
//                        ECP5 EHXPLLL ~400-800MHz VCO range)
//
// As with the other PLL wrappers in this project: re-verify (or regenerate)
// these divider values with Lattice Diamond's Clarity Designer PLL wizard,
// or the open-source `ecppll` utility, before committing to production --
// see README.md.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module clk_gen_mclk (
    input  wire clk_in,    // 50MHz board reference oscillator
    output wire cam_mclk,  // 24.000 MHz, exact
    output wire locked
);

    (* FREQUENCY_PIN_CLKI="50" *)
    (* FREQUENCY_PIN_CLKOP="24" *)
    (* ICP_CURRENT="12" *) (* LPF_RESISTOR="8" *)
    (* MFG_ENABLE_FILTEROPAMP="1" *) (* MFG_GMCREF_SEL="2" *)
    /* verilator lint_off PINCONNECTEMPTY */
    EHXPLLL #(
        .PLLRST_ENA        ("DISABLED"),
        .INTFB_WAKE        ("DISABLED"),
        .STDBY_ENABLE      ("DISABLED"),
        .DPHASE_SOURCE     ("DISABLED"),
        .OUTDIVIDER_MUXA   ("DIVA"),
        .OUTDIVIDER_MUXB   ("DIVB"),
        .OUTDIVIDER_MUXC   ("DIVC"),
        .OUTDIVIDER_MUXD   ("DIVD"),
        .CLKI_DIV          (25),
        .CLKOP_ENABLE      ("ENABLED"),
        .CLKOP_DIV         (20),
        .CLKOP_CPHASE      (0),
        .CLKOP_FPHASE      (0),
        .CLKOS_ENABLE      ("DISABLED"),
        .FEEDBK_PATH       ("CLKOP"),
        .CLKFB_DIV         (12)
    ) pll_i (
        .RST          (1'b0),
        .STDBY        (1'b0),
        .CLKI         (clk_in),
        .CLKOP        (cam_mclk),
        .CLKOS        (),
        .CLKFB        (cam_mclk),
        .CLKINTFB     (),
        .PHASESEL0    (1'b0),
        .PHASESEL1    (1'b0),
        .PHASEDIR     (1'b1),
        .PHASESTEP    (1'b1),
        .PHASELOADREG (1'b1),
        .PLLWAKESYNC  (1'b0),
        .ENCLKOP      (1'b0),
        .ENCLKOS      (1'b0),
        .ENCLKOS2     (1'b0),
        .ENCLKOS3     (1'b0),
        .LOCK         (locked)
    );

endmodule

`default_nettype wire
