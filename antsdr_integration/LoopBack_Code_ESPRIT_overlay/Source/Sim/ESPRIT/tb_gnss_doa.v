// ============================================================================
//  tb_gnss_doa.v  --  system testbench: gnss_passthrough v1.2 + ESPRIT DOA
// ----------------------------------------------------------------------------
//  Drives the REAL gnss_passthrough top (the module packaged into the block
//  design) with
//    * an AD9361-like RX stream in the l_clk domain (8.138 ns, one snapshot
//      every 4 clocks = 30.72 MS/s, 12-bit right-aligned samples),
//    * an AXI4-Lite master in the sys_cpu_clk domain (10 ns),
//  and checks, against expected values from gen_doa_stim.py:
//
//    T1  identity: GCP1 ID, VERSION 1.2, DOA_ID, CAPS, SCRATCH r/w
//    T2  live RX:  6 windows x 1024 snapshots, both estimators vs. model
//    T3  stream integrity: IN_COUNT, CDC FIFO overflow, peak detector
//    T4  test generator sweep: 8 known angles, averaged results
//    T5  calibration rotation: +90 deg rotation cancels a -90 deg phase
//    T6  averaging statistics: standard deviation from AVG_SQ
//    T7  decimation: DECIM = 3 keeps exactly 1 of 4 snapshots
//    T8  passthrough untouched: TX path counters and registers still work
//
//  Every check prints  "DOA_TB: <name> ... PASS|FAIL"  and the run ends with
//  "DOA_TB_RESULT: PASS" or "DOA_TB_RESULT: FAIL (<n> failures)".
//
//  Plusargs:  +STIM=<path to doa_stim.hex>
//  Include :  doa_tb_expected.vh (pass its directory with -i / -I)
// ============================================================================
`timescale 1ns/1ps

module tb_gnss_doa;

  // ---------------------------------------------------------------- clocks
  reg l_clk = 1'b0;   always #4.069 l_clk = ~l_clk;     // 122.88 MHz
  reg aclk  = 1'b0;   always #5.000 aclk  = ~aclk;      // 100 MHz
  reg rst     = 1'b1;
  reg aresetn = 1'b0;

  // ---------------------------------------------------------------- RX stream
  localparam integer NMAX = 8192;
  reg  [63:0] mem [0:NMAX-1];
  reg  [8*512-1:0] stim_file;
  integer n_stim;

  reg         adc_valid = 1'b0;
  reg  [15:0] adc_i0 = 0, adc_q0 = 0, adc_i1 = 0, adc_q1 = 0;
  reg         stream_go = 1'b0;
  integer     stream_idx = 0, stream_end = 0;
  reg  [1:0]  ph = 2'd0;

  always @(posedge l_clk) begin
    ph        <= ph + 1'b1;
    adc_valid <= 1'b0;
    if (stream_go && ph == 2'd0 && stream_idx < stream_end) begin
      adc_valid <= 1'b1;
      {adc_q1, adc_i1, adc_q0, adc_i0} <= mem[stream_idx];
      stream_idx <= stream_idx + 1;
    end
  end

  // ---------------------------------------------------------------- AXI-Lite
  reg  [15:0] awaddr = 0;  reg awvalid = 0;  wire awready;
  reg  [31:0] wdata  = 0;  reg wvalid  = 0;  wire wready;
  wire        bvalid;      wire [1:0] bresp;
  reg  [15:0] araddr = 0;  reg arvalid = 0;  wire arready;
  wire        rvalid;      wire [1:0] rresp; wire [31:0] rdata;

  wire [15:0] dac_i0, dac_q0, dac_i1, dac_q1;
  reg         dac_valid = 1'b0;
  always @(posedge l_clk) dac_valid <= (ph == 2'd2);

  gnss_passthrough dut (
    .clk (l_clk), .rst (rst),
    .adc_enable_i0 (1'b1), .adc_valid_i0 (adc_valid), .adc_data_i0 (adc_i0),
    .adc_enable_q0 (1'b1), .adc_valid_q0 (adc_valid), .adc_data_q0 (adc_q0),
    .adc_enable_i1 (1'b1), .adc_valid_i1 (adc_valid), .adc_data_i1 (adc_i1),
    .adc_enable_q1 (1'b1), .adc_valid_q1 (adc_valid), .adc_data_q1 (adc_q1),
    .dac_enable_i0 (1'b1), .dac_valid_i0 (dac_valid),
    .dac_enable_q0 (1'b1), .dac_valid_q0 (dac_valid),
    .dac_enable_i1 (1'b1), .dac_valid_i1 (dac_valid),
    .dac_enable_q1 (1'b1), .dac_valid_q1 (dac_valid),
    .dma_dac_data_i0 (16'h0), .dma_dac_data_q0 (16'h0),
    .dma_dac_data_i1 (16'h0), .dma_dac_data_q1 (16'h0),
    .dac_data_i0 (dac_i0), .dac_data_q0 (dac_q0),
    .dac_data_i1 (dac_i1), .dac_data_q1 (dac_q1),
    .s_axi_aclk (aclk), .s_axi_aresetn (aresetn),
    .s_axi_awvalid (awvalid), .s_axi_awaddr (awaddr), .s_axi_awprot (3'd0), .s_axi_awready (awready),
    .s_axi_wvalid (wvalid), .s_axi_wdata (wdata), .s_axi_wstrb (4'hF), .s_axi_wready (wready),
    .s_axi_bvalid (bvalid), .s_axi_bresp (bresp), .s_axi_bready (1'b1),
    .s_axi_arvalid (arvalid), .s_axi_araddr (araddr), .s_axi_arprot (3'd0), .s_axi_arready (arready),
    .s_axi_rvalid (rvalid), .s_axi_rresp (rresp), .s_axi_rdata (rdata), .s_axi_rready (1'b1)
  );

  task wr(input [15:0] a, input [31:0] d);
    begin
      @(posedge aclk);
      awaddr <= a; wdata <= d; awvalid <= 1'b1; wvalid <= 1'b1;
      @(posedge aclk);
      while (!awready) @(posedge aclk);
      awvalid <= 1'b0; wvalid <= 1'b0;
      @(posedge aclk);
      while (!bvalid) @(posedge aclk);
    end
  endtask

  task rd(input [15:0] a, output [31:0] d);
    begin
      @(posedge aclk);
      araddr <= a; arvalid <= 1'b1;
      @(posedge aclk);
      while (!rvalid) @(posedge aclk);
      d = rdata;
      arvalid <= 1'b0;
      @(posedge aclk);
    end
  endtask

  // ---------------------------------------------------------------- registers
  localparam [15:0] PT_ID = 16'h000, PT_VER = 16'h004, PT_SCRATCH = 16'h008,
                    PT_CONTROL = 16'h00C, PT_RXCNT0 = 16'h014;
  localparam [15:0] DOA_ID = 16'h100, DOA_VER = 16'h104, DOA_CTRL = 16'h108,
                    DOA_MU = 16'h10C, DOA_K = 16'h110, DOA_UCTOL = 16'h114,
                    DOA_DECIM = 16'h118, DOA_CAL = 16'h11C, DOA_FTW = 16'h120,
                    DOA_DPH = 16'h124, DOA_TGCFG = 16'h128, DOA_STATUS = 16'h12C,
                    DOA_INCNT = 16'h130, DOA_PEAK = 16'h134,
                    I_SEQ = 16'h138, I_PHI = 16'h13C, I_PSI = 16'h140, I_TH = 16'h144,
                    I_DEG = 16'h148, I_ASEQ = 16'h14C, I_ADEG = 16'h150,
                    I_ASQL = 16'h154, I_ASQH = 16'h158,
                    M_SEQ = 16'h15C, M_PHI = 16'h160, M_PSI = 16'h164, M_TH = 16'h168,
                    M_DEG = 16'h16C, M_ASEQ = 16'h170, M_ADEG = 16'h174,
                    M_ASQL = 16'h178, M_ASQH = 16'h17C, M_OFFC = 16'h180,
                    DOA_CAPS = 16'h194;

  // ctrl word: enable | src_test<<1 | p_log2<<4 | log2_snap<<8 | avg_log2<<16
  function [31:0] ctrl;
    input en, src;
    input [1:0] p;
    input [4:0] snap;
    input [3:0] avg;
    begin
      ctrl = {12'd0, avg, 3'd0, snap, 2'd0, p, 2'd0, src, en};
    end
  endfunction

  // ---------------------------------------------------------------- expected
  real live_true [0:15];
  real live_exp_i [0:15];
  real live_exp_m [0:15];
  real live_exp_mag [0:15];
  real tg_theta [0:15];
  reg  [31:0] tg_dphase [0:15];
`include "doa_tb_expected.vh"

  integer fails = 0, checks = 0;

  task check(input [8*64-1:0] name, input ok);
    begin
      checks = checks + 1;
      if (ok) $display("DOA_TB: %0s ... PASS", name);
      else begin
        $display("DOA_TB: %0s ... FAIL", name);
        fails = fails + 1;
      end
    end
  endtask

  function real q16;  input [31:0] v; begin q16 = $signed(v) / 65536.0;    end endfunction
  function real q24;  input [31:0] v; begin q24 = $signed(v) / 16777216.0; end endfunction
  function real absr; input real v;   begin absr = (v < 0.0) ? -v : v;     end endfunction

  // wait until a sequence register reaches `target`
  task wait_seq(input [15:0] a, input [31:0] target, output [31:0] got);
    integer guard;
    begin
      guard = 0;
      rd(a, got);
      while (got < target && guard < 200000) begin
        rd(a, got);
        guard = guard + 1;
      end
    end
  endtask

  // ---------------------------------------------------------------- test
  reg  [31:0] r, s, si, sm, d_i, d_m, mag, base_i, base_m;
  reg  [31:0] sql, sqh;
  real e_i, e_m, avg, var_q, sd;
  integer w, k;
  reg [8*64-1:0] nm;

  initial begin
    if (!$value$plusargs("STIM=%s", stim_file)) stim_file = "doa_stim.hex";
    $readmemh(stim_file, mem);
    load_expected;
    n_stim = LIVE_WINDOWS << LIVE_LOG2_SNAP;
    $display("DOA_TB: ===== gnss_passthrough v1.2 + ESPRIT DOA system test =====");
    $display("DOA_TB: stimulus %0d snapshots from %0s", n_stim, stim_file);

    repeat (10) @(posedge aclk);
    rst <= 1'b0; aresetn <= 1'b1;
    repeat (10) @(posedge aclk);

    // ---------------- T1 identity ----------------
    rd(PT_ID, r);       check("T1 passthrough ID = GCP1", r == 32'h47435031);
    rd(PT_VER, r);      check("T1 passthrough VERSION = 1.2", r == 32'h00010002);
    rd(DOA_ID, r);      check("T1 DOA_ID = DOA1", r == 32'h444F4131);
    rd(DOA_CAPS, r);    check("T1 CAPS: both estimators built", r[1:0] == 2'b11 && r[15:8] == 8'd14);
    rd(DOA_CTRL, r);    check("T1 DOA_CTRL reset value 0x00040A20", r == 32'h00040A20);
    wr(PT_SCRATCH, 32'hC0FFEE01); rd(PT_SCRATCH, r);
    check("T1 SCRATCH read/write", r == 32'hC0FFEE01);
    wr(DOA_K, 32'd341782638); rd(DOA_K, r); check("T1 DOA_K write/read", r == 32'd341782638);

    // ---------------- T2 live RX ----------------
    $display("DOA_TB: --- T2 live RX, %0d windows of %0d snapshots, p = 4 ---",
             LIVE_WINDOWS, 1 << LIVE_LOG2_SNAP);
    wr(DOA_CTRL, ctrl(1, 0, 2'd2, LIVE_LOG2_SNAP, 4'd0));
    repeat (20) @(posedge aclk);                  // let the control bits cross
    stream_idx = 0; stream_end = n_stim; stream_go = 1'b1;
    for (w = 0; w < LIVE_WINDOWS; w = w + 1) begin
      wait_seq(I_SEQ, w + 1, si);
      wait_seq(M_SEQ, w + 1, sm);
      rd(I_DEG, d_i); rd(M_DEG, d_m); rd(M_PSI, mag);
      e_i = absr(q16(d_i) - live_exp_i[w]);
      e_m = absr(q16(d_m) - live_exp_m[w]);
      $display("DOA_TB:   win %0d true %7.2f | I-UESPRIT %9.4f (model %9.4f, err %.4f) | M-ESPRIT %9.4f (model %9.4f, err %.4f) |psi| %.4f",
               w, live_true[w], q16(d_i), live_exp_i[w], e_i, q16(d_m), live_exp_m[w], e_m, q24(mag));
      check("T2 live window sequence (no window lost)", si == w + 1 && sm == w + 1);
      check("T2 I-UESPRIT angle within 0.05 deg of model", e_i < 0.05);
      check("T2 M-ESPRIT angle within 0.05 deg of model", e_m < 0.05);
      check("T2 M-ESPRIT |psi| within 0.01 of model", absr(q24(mag) - live_exp_mag[w]) < 0.01);
    end
    while (stream_idx < stream_end) @(posedge l_clk);
    repeat (50) @(posedge aclk);

    // ---------------- T3 stream integrity ----------------
    rd(DOA_INCNT, r);  check("T3 IN_COUNT equals snapshots streamed", r == n_stim);
    rd(DOA_STATUS, r); check("T3 no CDC FIFO overflow", r[0] == 1'b0);
    check("T3 STATUS shows enable, live source, both cores", r[4:1] == 4'b0111);
    rd(DOA_PEAK, r);
    $display("DOA_TB:   peak |x0| = %0d, peak |x1| = %0d (Q1.15)", r[15:0], r[31:16]);
    check("T3 peak detector sees the signal", r[15:0] > 16'd4000 && r[31:16] > 16'd4000);
    stream_go = 1'b0;

    // ---------------- T4 test generator sweep ----------------
    $display("DOA_TB: --- T4 built-in test generator, %0d angles, 512-snapshot windows, average of 2 ---", TG_POINTS);
    wr(DOA_CTRL, 32'd0);
    wr(DOA_TGCFG, 32'h00000008);                 // 8 clk/sample, full amplitude, no noise
    wr(DOA_CAL, 32'h00007FFF);
    wr(DOA_DPH, tg_dphase[0]);
    wr(DOA_CTRL, ctrl(1, 1, 2'd2, 5'd9, 4'd1));
    for (k = 0; k < TG_POINTS; k = k + 1) begin
      wr(DOA_DPH, tg_dphase[k]);
      rd(I_ASEQ, base_i); rd(M_ASEQ, base_m);
      wait_seq(I_ASEQ, base_i + 2, s);             // 1st block may straddle the change
      wait_seq(M_ASEQ, base_m + 2, s);
      rd(I_ADEG, d_i); rd(M_ADEG, d_m);
      e_i = absr(q16(d_i) - tg_theta[k]);
      e_m = absr(q16(d_m) - tg_theta[k]);
      $display("DOA_TB:   set %7.2f deg (DPHASE 0x%08h) | I-UESPRIT %9.4f (err %.4f) | M-ESPRIT %9.4f (err %.4f)",
               tg_theta[k], tg_dphase[k], q16(d_i), e_i, q16(d_m), e_m);
      check("T4 test-gen I-UESPRIT within 0.05 deg", e_i < 0.05);
      check("T4 test-gen M-ESPRIT within 0.05 deg", e_m < 0.05);
    end

    // ---------------- T5 calibration rotation ----------------
    // TG phase -90 deg (DPHASE = -1/4 turn, theta = +30 deg); CAL rotates
    // element 1 by +90 deg (c = 0, s = 1), so the corrected phase is 0 -> 0 deg.
    $display("DOA_TB: --- T5 calibration rotation ---");
    wr(DOA_DPH, 32'hC0000000);
    wr(DOA_CAL, 32'h7FFF0000);
    rd(I_ASEQ, base_i); rd(M_ASEQ, base_m);
    wait_seq(I_ASEQ, base_i + 2, s); wait_seq(M_ASEQ, base_m + 2, s);
    rd(I_ADEG, d_i); rd(M_ADEG, d_m);
    $display("DOA_TB:   raw phase -90 deg + CAL +90 deg -> I %8.4f  M %8.4f deg (expect 0)", q16(d_i), q16(d_m));
    check("T5 CAL cancels the phase (I-UESPRIT, |theta| < 0.05)", absr(q16(d_i)) < 0.05);
    check("T5 CAL cancels the phase (M-ESPRIT,  |theta| < 0.05)", absr(q16(d_m)) < 0.05);
    wr(DOA_CAL, 32'h00007FFF);
    rd(I_ASEQ, base_i); wait_seq(I_ASEQ, base_i + 2, s);
    rd(I_ADEG, d_i);
    $display("DOA_TB:   CAL removed -> I %8.4f deg (expect 30)", q16(d_i));
    check("T5 CAL identity restores +30 deg", absr(q16(d_i) - 30.0) < 0.05);

    // ---------------- T6 averaging statistics ----------------
    $display("DOA_TB: --- T6 averaging statistics (noise on, average of 16) ---");
    wr(DOA_CTRL, 32'd0);
    wr(DOA_TGCFG, 32'h00300008);                 // noise >> 3
    wr(DOA_DPH, tg_dphase[5]);                   // +30 deg
    wr(DOA_CTRL, ctrl(1, 1, 2'd2, 5'd7, 4'd4) | 32'h8);   // noise enable, 128-snapshot windows
    rd(M_ASEQ, base_m);
    wait_seq(M_ASEQ, base_m + 2, s);
    rd(M_ADEG, d_m); rd(M_ASQL, sql); rd(M_ASQH, sqh); rd(M_OFFC, r);
    avg   = q16(d_m);
    var_q = (sqh * 4294967296.0 + sql) / 4294967296.0 - avg * avg;
    sd    = (var_q > 0.0) ? $sqrt(var_q) : 0.0;
    $display("DOA_TB:   M-ESPRIT mean %8.4f deg, std %.4f deg, off-circle %0d/16", avg, sd, r);
    check("T6 noisy mean within 0.5 deg of 30", absr(avg - 30.0) < 0.5);
    check("T6 std deviation computed and small (< 1 deg)", sd < 1.0);

    // ---------------- T7 decimation ----------------
    $display("DOA_TB: --- T7 decimation, DECIM = 3 ---");
    wr(DOA_CTRL, 32'd0);
    wr(DOA_DECIM, 32'd3);
    wr(DOA_CTRL, ctrl(1, 0, 2'd2, 5'd10, 4'd0));
    repeat (20) @(posedge aclk);
    stream_idx = 0; stream_end = 4096; stream_go = 1'b1;
    while (stream_idx < stream_end) @(posedge l_clk);
    repeat (400) @(posedge aclk);
    rd(DOA_INCNT, r);
    $display("DOA_TB:   4096 snapshots streamed, %0d delivered", r);
    check("T7 DECIM=3 delivers exactly 1 of 4 (1024)", r == 32'd1024);
    wait_seq(M_SEQ, 1, s);
    check("T7 one window completed from decimated data", s == 32'd1);
    stream_go = 1'b0;
    wr(DOA_DECIM, 32'd0);

    // ---------------- T8 passthrough untouched ----------------
    wr(PT_CONTROL, 32'h1);                       // pass_en
    stream_idx = 0; stream_end = 2048; stream_go = 1'b1;
    while (stream_idx < stream_end) @(posedge l_clk);
    repeat (600) @(posedge aclk);
    rd(PT_RXCNT0, r);
    check("T8 passthrough RX counter advances", r > 32'd1000);
    check("T8 passthrough DAC output is left-aligned (low nibble 0)", dac_i0[3:0] == 4'd0 && dac_q0[3:0] == 4'd0);
    rd(PT_VER, r);
    check("T8 passthrough registers still respond", r == 32'h00010002);

    $display("DOA_TB: ============================================================");
    if (fails == 0) $display("DOA_TB_RESULT: PASS (%0d checks)", checks);
    else            $display("DOA_TB_RESULT: FAIL (%0d of %0d checks failed)", fails, checks);
    $finish;
  end

  // watchdog
  initial begin
    #20000000;
    $display("DOA_TB_RESULT: FAIL (watchdog timeout)");
    $finish;
  end

endmodule
