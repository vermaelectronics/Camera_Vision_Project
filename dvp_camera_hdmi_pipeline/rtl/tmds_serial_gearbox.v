// ============================================================================
// tmds_serial_gearbox.v -- ECP5 native TMDS bit-serializer (GPDI output)
// ----------------------------------------------------------------------------
// Serializes three registered 10-bit-per-pixel-clock TMDS symbols (R,G,B)
// plus a forwarded pixel clock onto four single-ended GPDI/TMDS pins, using
// the ECP5's documented DDR output gearbox: EHXPLLL -> ECLKSYNCB -> CLKDIVF
// -> ODDRX2F (Lattice TN1265, "ECP5 and ECP5-5G High-Speed I/O Interface").
//
//   clk_pixel : pixel clock (P)            e.g. 74.286 MHz
//   clk_eclk  : 5x pixel clock, RAW PLL output, NOT yet on the edge-clock
//               network -- this module routes it there via ECLKSYNCB.
//
// ODDRX2F is a 4:1 DDR gearbox: every SCLK cycle it serializes 4 bits
// (D0..D3) at the DDR rate set by ECLK (ECLK = 2x SCLK by construction of
// CLKDIVF DIV="2.0"). 4 bits/SCLK x (SCLK = 2.5x pixel_clock) = 10
// bits/pixel-clock, exactly the TMDS requirement.
//
// Crossing from the pixel-clock domain (where the TMDS symbols are produced,
// one every pixel clock) into the SCLK domain (2.5x pixel clock, a
// *non-integer* ratio) is done through a small dual-clock FIFO
// (async_fifo.v) rather than any hand-derived static phase assumption
// between PLL taps -- see video_cdc_buffer.v's header for the same
// philosophy. This makes the crossing correct regardless of the actual
// silicon phase relationship between the pixel and edge-clock domains.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module tmds_serial_gearbox (
    input  wire       clk_pixel,
    input  wire        clk_eclk,     // raw PLL 5x-pixel output (pre ECLKSYNCB)
    input  wire        rst_pixel,    // synchronous to clk_pixel

    input  wire [9:0]  tmds_r,       // registered symbols, one new value/pixel clock
    input  wire [9:0]  tmds_g,
    input  wire [9:0]  tmds_b,

    output wire [3:0]  gpdi_dp       // [0]=Blue [1]=Green [2]=Red [3]=Clock
                                      // (matches the IcePi-Zero board's GPDI
                                      // channel-to-pin wiring)
);

    // ------------------------------------------------------------------
    // Edge-clock network + SCLK divider (shared by all three channels)
    // ------------------------------------------------------------------
    wire eclk;   // buffered, on-network edge clock (= clk_eclk, ~371 MHz)
    wire sclk;   // = eclk/2 = 2.5x pixel clock (~185.7 MHz), CLKDIVF-derived

    ECLKSYNCB u_eclksyncb (
        .ECLKI (clk_eclk),
        .STOP  (1'b0),
        .ECLKO (eclk)
    );

    CLKDIVF #(
        .GSR ("DISABLED"),
        .DIV ("2.0")
    ) u_clkdivf (
        .CLKI    (eclk),
        .RST     (rst_pixel),  // async-ish assert is fine; CDIVX free-runs once released
        .ALIGNWD (1'b0),
        .CDIVX   (sclk)
    );

    // reset synchronizer into the sclk domain
    reg [1:0] rst_sclk_sr = 2'b11;
    always @(posedge sclk or posedge rst_pixel)
        if (rst_pixel) rst_sclk_sr <= 2'b11;
        else           rst_sclk_sr <= {rst_sclk_sr[0], 1'b0};
    wire rst_sclk = rst_sclk_sr[1];

    // ------------------------------------------------------------------
    // Pixel-domain -> SCLK-domain CDC: one 30-bit word (R,G,B symbols)
    // written every pixel clock, read once every 5 SCLK cycles.
    // ------------------------------------------------------------------
    wire [29:0] fifo_wdata = {tmds_r, tmds_g, tmds_b};
    wire [29:0] fifo_rdata;
    wire        fifo_empty, fifo_full;

    // mod-5 nibble counter: position within the current 20-bit (2-pixel)
    // window. Declared here, ahead of the async_fifo instantiation below
    // that connects fifo_rd_en to a port, for declare-before-use
    // portability (see async_fifo.v's header comment on the same issue).
    reg [2:0] nibble_cnt = 3'd0;
    wire      wrap = (nibble_cnt == 3'd4);
    wire      fifo_rd_en = wrap & ~fifo_empty;

    async_fifo #(
        .WIDTH(30),
        .DEPTH(16)
    ) u_sym_fifo (
        .wr_clk  (clk_pixel),
        .wr_rst  (rst_pixel),
        .wr_en   (1'b1),        // a fresh symbol is produced every pixel clock
        .wr_data (fifo_wdata),
        .wr_full (fifo_full),
        .wr_level(),

        .rd_clk  (sclk),
        .rd_rst  (rst_sclk),
        .rd_en   (fifo_rd_en),
        .rd_data (fifo_rdata),
        .rd_empty(fifo_empty)
    );

    reg [9:0] sym_a_r, sym_b_r, sym_a_g, sym_b_g, sym_a_b, sym_b_b;

    always @(posedge sclk) begin
        if (rst_sclk) begin
            nibble_cnt <= 3'd0;
            sym_a_r <= 10'd0; sym_b_r <= 10'd0;
            sym_a_g <= 10'd0; sym_b_g <= 10'd0;
            sym_a_b <= 10'd0; sym_b_b <= 10'd0;
        end else begin
            nibble_cnt <= wrap ? 3'd0 : nibble_cnt + 3'd1;
            if (wrap && !fifo_empty) begin
                sym_a_r <= sym_b_r; sym_b_r <= fifo_rdata[29:20];
                sym_a_g <= sym_b_g; sym_b_g <= fifo_rdata[19:10];
                sym_a_b <= sym_b_b; sym_b_b <= fifo_rdata[9:0];
            end
        end
    end

    wire [19:0] window_r = {sym_b_r, sym_a_r};
    wire [19:0] window_g = {sym_b_g, sym_a_g};
    wire [19:0] window_b = {sym_b_b, sym_a_b};

    wire [3:0] nibble_r = window_r[nibble_cnt*4 +: 4];
    wire [3:0] nibble_g = window_g[nibble_cnt*4 +: 4];
    wire [3:0] nibble_b = window_b[nibble_cnt*4 +: 4];

    // ------------------------------------------------------------------
    // 4:1 DDR serializers (ODDRX2F): D0 is transmitted first, D3 last,
    // matching TMDS bit order (LSB of the 10-bit symbol transmitted first).
    // ------------------------------------------------------------------
    wire ser_r, ser_g, ser_b, ser_clk;

    ODDRX2F u_oddr_r (.SCLK(sclk), .ECLK(eclk), .RST(rst_sclk),
                       .D0(nibble_r[0]), .D1(nibble_r[1]), .D2(nibble_r[2]), .D3(nibble_r[3]),
                       .Q(ser_r));
    ODDRX2F u_oddr_g (.SCLK(sclk), .ECLK(eclk), .RST(rst_sclk),
                       .D0(nibble_g[0]), .D1(nibble_g[1]), .D2(nibble_g[2]), .D3(nibble_g[3]),
                       .Q(ser_g));
    ODDRX2F u_oddr_b (.SCLK(sclk), .ECLK(eclk), .RST(rst_sclk),
                       .D0(nibble_b[0]), .D1(nibble_b[1]), .D2(nibble_b[2]), .D3(nibble_b[3]),
                       .Q(ser_b));

    // TMDS clock channel does not carry 8b/10b-encoded data -- it is simply
    // a 50% duty square wave at the pixel-clock rate, forwarded directly.
    ODDRX1F u_oddr_clk (.SCLK(clk_pixel), .RST(rst_pixel),
                         .D0(1'b1), .D1(1'b0), .Q(ser_clk));

    assign gpdi_dp[0] = ser_b;
    assign gpdi_dp[1] = ser_g;
    assign gpdi_dp[2] = ser_r;
    assign gpdi_dp[3] = ser_clk;

endmodule

`default_nettype wire
