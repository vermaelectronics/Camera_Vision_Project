// ============================================================================
//  doa_async_fifo.v
//  ANTSDR E310 V1 / GNSS CRPA project -- ESPRIT DOA integration
// ----------------------------------------------------------------------------
//  Dual-clock FIFO carrying RX snapshots from the AD9361 sample domain
//  (axi_ad9361/l_clk) into the ESPRIT processing domain (sys_cpu_clk,
//  100 MHz).
//
//  Classic Cummings gray-pointer design:
//    * binary + gray write/read pointers, (AW+1) bits
//    * each gray pointer crosses through a two-flop synchroniser
//      (registers named gs_meta_* / gs_sync_*, see the XDC)
//    * distributed RAM, written in wclk, read asynchronously into a
//      register in rclk
//    * full/empty computed from the synchronised gray pointers
//      (conservative: never over/under-runs)
//
//  The read side is a simple "pop when not empty" interface: rd_valid pulses
//  for one rclk cycle with rd_data. The ESPRIT cores accept one snapshot per
//  clock, and the 100 MHz read clock is faster than any AD9361 sample rate
//  this design uses (30.72 MS/s), so the FIFO never fills in normal use.
//  A write into a full FIFO is dropped and latched in wr_overflow (wclk),
//  which is also re-synchronised to rclk as rd_overflow.
//
//  CDC constraints: Source/XDC/gnss_passthrough_cdc.xdc, section 4.
//  Original work for this project.
// ============================================================================

`timescale 1ns/100ps

module doa_async_fifo #(
  parameter DW = 64,
  parameter AW = 4                    // depth = 2**AW
) (
  // write side (sample domain)
  input               wclk,
  input               wrst,           // active high, synchronous to wclk
  input               wr_en,
  input      [DW-1:0] wr_data,
  output              wr_full,
  output reg          wr_overflow,    // sticky until wrst

  // read side (processing domain)
  input               rclk,
  input               rrst,           // active high, synchronous to rclk
  output reg          rd_valid,
  output reg [DW-1:0] rd_data,
  output              rd_overflow     // wr_overflow synchronised to rclk
);

  (* ram_style = "distributed" *) reg [DW-1:0] mem [0:(1<<AW)-1];

  // --------------------------------------------------------------------------
  // write domain
  // --------------------------------------------------------------------------
  reg  [AW:0] wbin  = {(AW+1){1'b0}};
  reg  [AW:0] wgray = {(AW+1){1'b0}};
  (* ASYNC_REG = "TRUE" *) reg [AW:0] gs_meta_r2w = {(AW+1){1'b0}};
  (* ASYNC_REG = "TRUE" *) reg [AW:0] gs_sync_r2w = {(AW+1){1'b0}};

  wire [AW:0] wbin_nx  = wbin + 1'b1;
  wire [AW:0] wgray_nx = (wbin_nx >> 1) ^ wbin_nx;
  reg  [AW:0] rgray;                                   // declared below

  // full when the next write would make the write gray pointer equal to the
  // read gray pointer with the two MSBs inverted
  assign wr_full = (wgray == {~gs_sync_r2w[AW:AW-1], gs_sync_r2w[AW-2:0]});

  always @(posedge wclk) begin
    gs_meta_r2w <= rgray;
    gs_sync_r2w <= gs_meta_r2w;
  end

  always @(posedge wclk) begin
    if (wrst) begin
      wbin        <= {(AW+1){1'b0}};
      wgray       <= {(AW+1){1'b0}};
      wr_overflow <= 1'b0;
    end else if (wr_en) begin
      if (!wr_full) begin
        mem[wbin[AW-1:0]] <= wr_data;
        wbin  <= wbin_nx;
        wgray <= wgray_nx;
      end else begin
        wr_overflow <= 1'b1;
      end
    end
  end

  // --------------------------------------------------------------------------
  // read domain
  // --------------------------------------------------------------------------
  reg  [AW:0] rbin = {(AW+1){1'b0}};
  initial rgray = {(AW+1){1'b0}};
  (* ASYNC_REG = "TRUE" *) reg [AW:0] gs_meta_w2r = {(AW+1){1'b0}};
  (* ASYNC_REG = "TRUE" *) reg [AW:0] gs_sync_w2r = {(AW+1){1'b0}};
  (* ASYNC_REG = "TRUE" *) reg        ovf_meta    = 1'b0;
  (* ASYNC_REG = "TRUE" *) reg        ovf_sync    = 1'b0;

  wire        rd_empty = (rgray == gs_sync_w2r);
  wire [AW:0] rbin_nx  = rbin + 1'b1;
  wire [AW:0] rgray_nx = (rbin_nx >> 1) ^ rbin_nx;

  always @(posedge rclk) begin
    gs_meta_w2r <= wgray;
    gs_sync_w2r <= gs_meta_w2r;
    ovf_meta    <= wr_overflow;
    ovf_sync    <= ovf_meta;
  end

  assign rd_overflow = ovf_sync;

  always @(posedge rclk) begin
    if (rrst) begin
      rbin     <= {(AW+1){1'b0}};
      rgray    <= {(AW+1){1'b0}};
      rd_valid <= 1'b0;
    end else begin
      rd_valid <= 1'b0;
      if (!rd_empty) begin
        rd_data  <= mem[rbin[AW-1:0]];
        rd_valid <= 1'b1;
        rbin     <= rbin_nx;
        rgray    <= rgray_nx;
      end
    end
  end

endmodule
