// ============================================================================
// clk_gen_dvi.v -- EHXPLLL wrapper: 50MHz board oscillator -> pixel + ECLK
// ----------------------------------------------------------------------------
// Generates the two clocks needed for *native* GPDI/TMDS output at a pixel
// clock of 74.25 MHz -- i.e. both CEA-861 720p60 (1280x720, 74.25MHz) and
// 1080p30 (1920x1080, 74.25MHz -- CEA-861 defines 1080p at 30/25/24Hz using
// the SAME H/V blanking totals as the 60/50/48Hz modes, just half the pixel
// clock, so this one PLL config serves both).
//
//   clk_pixel : pixel clock                              (target 74.25 MHz)
//   clk_eclk  : 5x pixel clock, routed to the ECP5 dedicated Edge-Clock
//               network by the caller (via ECLKSYNCB) for the TMDS
//               serializer gearbox                        (target 371.25 MHz)
//
// ---- Divider derivation (Fvco = Fin * CLKFB_DIV / CLKI_DIV * CLKOP_DIV) ---
//   Fin        = 50 MHz (board reference oscillator, confirmed against the
//                IcePi-Zero reference design's own "clk" pin @ site M1)
//   CLKI_DIV   = 7    -> PFD = 50/7    = 7.143 MHz  (healthy, comfortably
//                                                     inside typical PLL
//                                                     phase-detector range)
//   CLKFB_DIV  = 52   -> CLKOP = 50 * 52 / 7          = 371.4286 MHz  (ECLK)
//   CLKOP_DIV  = 2    -> Fvco  = CLKOP * CLKOP_DIV     = 742.8571 MHz
//                        (within the ECP5 EHXPLLL VCO range of ~400-800MHz)
//   CLKOS_DIV  = 10   -> CLKOS = Fvco / CLKOS_DIV       = 74.2857 MHz (pixel)
//
// Achieved frequencies are 371.429 MHz / 74.286 MHz against nominal
// 371.25 MHz / 74.25 MHz -- a +0.048% error, comfortably inside the +/-0.5%
// clock tolerance essentially all DVI/HDMI sinks accept, and far better than
// is achievable from most non-multiple reference oscillators.
//
// IMPORTANT: as with any hand-derived PLL configuration, re-verify (or
// regenerate) these exact divider values with Lattice Diamond's Clarity
// Designer PLL wizard, or the open-source `ecppll` utility from
// prj-trellis, before committing to silicon -- see README.md.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module clk_gen_dvi (
    input  wire clk_in,     // 50MHz board reference oscillator
    output wire clk_pixel,  // ~74.286 MHz
    output wire clk_eclk,   // ~371.429 MHz (5x clk_pixel) -> route via ECLKSYNCB
    output wire locked
);

    (* FREQUENCY_PIN_CLKI="50" *)
    (* FREQUENCY_PIN_CLKOP="371.429" *)
    (* FREQUENCY_PIN_CLKOS="74.286" *)
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
        .CLKI_DIV          (7),
        .CLKOP_ENABLE      ("ENABLED"),
        .CLKOP_DIV         (2),
        .CLKOP_CPHASE      (0),
        .CLKOP_FPHASE      (0),
        .CLKOS_ENABLE      ("ENABLED"),
        .CLKOS_DIV         (10),
        .CLKOS_CPHASE      (0),
        .CLKOS_FPHASE      (0),
        .FEEDBK_PATH       ("CLKOP"),
        .CLKFB_DIV         (52)
    ) pll_i (
        .RST          (1'b0),
        .STDBY        (1'b0),
        .CLKI         (clk_in),
        .CLKOP        (clk_eclk),   // 371.429 MHz -> ECLK
        .CLKOS        (clk_pixel),  // 74.286 MHz  -> pixel clock
        .CLKFB        (clk_eclk),
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
