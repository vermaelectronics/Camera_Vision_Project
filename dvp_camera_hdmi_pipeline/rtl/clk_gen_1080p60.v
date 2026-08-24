// ============================================================================
// clk_gen_1080p60.v -- EHXPLLL wrapper: 50MHz board oscillator -> 1080p60
//                       parallel-RGB pixel clock (external HDMI transmitter)
// ----------------------------------------------------------------------------
// Generates a single-rate ~148.5 MHz pixel clock for driving an external
// dedicated HDMI/DVI transmitter IC (e.g. ADV7511/ADV7513, ITE IT6613/66121,
// SiI9022 class parts) over a standard 24-bit parallel-RGB + HSYNC/VSYNC/DE
// video bus. This path needs NO on-chip TMDS serialization at all -- the
// transmitter chip does that in its own silicon -- so it sidesteps the
// LFE5U-25F's edge-clock ceiling entirely. See README.md "Why two different
// 1080p60 delivery paths" for why this exists alongside clk_gen_dvi.v.
//
// ---- Divider derivation (Fvco = Fin * CLKFB_DIV / CLKI_DIV * CLKOP_DIV) ---
// True 148.5 MHz cannot be hit exactly from a 50 MHz reference with EHXPLLL's
// integer dividers (148.5/50 = 297/100 does not reduce to small integers),
// so two practical options are given -- pick whichever your transmitter/
// display combination tolerates better and edit PLL_OPT below:
//
//   OPTION A (default): safe PFD, +1.01% pixel clock
//     CLKI_DIV=9, CLKFB_DIV=27, CLKOP_DIV=3
//     PFD = 50/9 = 5.556 MHz   CLKOP = 50*27/9 = 150.000 MHz   Fvco=450MHz
//
//   OPTION B: tight frequency match, low PFD (verify against Diamond/ecppll
//             before relying on this in production -- see README.md)
//     CLKI_DIV=27, CLKFB_DIV=80, CLKOP_DIV=3
//     PFD = 50/27 = 1.852 MHz  CLKOP = 50*80/27 = 148.148 MHz  Fvco=444.4MHz
//
// Either way, essentially every HDMI transmitter IC locks its internal TMDS
// PLL to whatever pixel clock it is actually handed, so a fraction of a
// percent of deviation from the CEA-861 nominal 148.5 MHz does not break the
// link; it is simply not bit-exact to the standard.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module clk_gen_1080p60 #(
    parameter PLL_OPT = "A"   // "A" (150.000MHz, safe PFD) or "B" (148.148MHz, tight)
) (
    input  wire clk_in,      // 50MHz board reference oscillator
    output wire clk_pixel,   // ~148.5 MHz parallel-RGB pixel clock
    output wire locked
);

    localparam integer CLKI_DIV  = (PLL_OPT == "B") ? 27 : 9;
    localparam integer CLKFB_DIV = (PLL_OPT == "B") ? 80 : 27;
    localparam integer CLKOP_DIV = 3;

    (* FREQUENCY_PIN_CLKI="50" *)
    (* FREQUENCY_PIN_CLKOP="148.5" *)
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
        .CLKI_DIV          (CLKI_DIV),
        .CLKOP_ENABLE      ("ENABLED"),
        .CLKOP_DIV         (CLKOP_DIV),
        .CLKOP_CPHASE      (0),
        .CLKOP_FPHASE      (0),
        .CLKOS_ENABLE      ("DISABLED"),
        .FEEDBK_PATH       ("CLKOP"),
        .CLKFB_DIV         (CLKFB_DIV)
    ) pll_i (
        .RST          (1'b0),
        .STDBY        (1'b0),
        .CLKI         (clk_in),
        .CLKOP        (clk_pixel),
        .CLKOS        (),
        .CLKFB        (clk_pixel),
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
