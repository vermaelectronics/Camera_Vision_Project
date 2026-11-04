// ============================================================================
//  tb_gnss_stap.v -- integration testbench:
//      AD9361 model --> gnss_stap --> gnss_passthrough --> DAC model
//  Two asynchronous clocks: l_clk (61.44 MHz, adc_valid every 2nd clock =
//  30.72 MSPS 2R2T, the vendor rate) and sys_cpu_clk (100 MHz AXI).
//
//  A. register access: GSTP wrapper ID (local), STAP ID/CONFIG through the
//     CDC bridge, GCP1 passthrough ID
//  B. bypass (USER_CTRL.aj_en = 0): TX samples == RX samples << 4
//  C. covariance: CORR_START, 868->93 results and the raw edge window
//     compared bit-exactly with the golden model
//  D. weights from the C solver, commit, aj_en = 1: every gnss_stap output
//     compared bit-exactly with the golden model, and the DAC stream checked
//     to carry the same samples (<< 4)
//  E. bridge timeout: with l_clk stopped, a STAP read returns 0xDEADC10C and
//     does not hang the bus; the local registers still answer
//
//  Vectors: Source/Sim/model/stap_golden.py (see Automation/Linux/run_sim.sh)
// ============================================================================
`timescale 1ns/100ps
`include "tb_params.vh"

module tb_gnss_stap;
  localparam M = 2, N = 16, HT = 2*N-2, OW = 12;
  localparam NS = `TB_SL + `TB_K;
  localparam NBYP = 400;

  // ---------------------------------------------------------------- clocks
  reg l_clk = 0, s_clk = 0, l_run = 1;
  always #8.138 if (l_run) l_clk = ~l_clk;     // 61.44 MHz
  always #5.000 s_clk = ~s_clk;                // 100 MHz
  reg rst = 1, aresetn = 0;

  // ---------------------------------------------------------------- AD9361 model
  reg        adc_valid = 0;
  reg [15:0] adc_i0 = 0, adc_q0 = 0, adc_i1 = 0, adc_q1 = 0;
  reg        dac_valid = 0;
  reg        ph = 0;
  always @(posedge l_clk) begin ph <= ~ph; dac_valid <= ph; end   // every 2nd clock

  // ---------------------------------------------------------------- DUT
  wire        aj_en_i0, aj_en_q0, aj_en_i1, aj_en_q1;
  wire        aj_v_i0, aj_v_q0, aj_v_i1, aj_v_q1;
  wire [15:0] aj_i0, aj_q0, aj_i1, aj_q1;
  wire [15:0] dac_i0, dac_q0, dac_i1, dac_q1;

  // AXI master signals, one set per slave (0 = gnss_stap, 1 = gnss_passthrough)
  reg  [15:0] awaddr[0:1], araddr[0:1];
  reg         awvalid[0:1], wvalid[0:1], bready[0:1], arvalid[0:1], rready[0:1];
  reg  [31:0] wdata[0:1];
  wire        awready0, wready0, bvalid0, arready0, rvalid0;
  wire        awready1, wready1, bvalid1, arready1, rvalid1;
  wire [31:0] rdata0, rdata1;
  wire [1:0]  bresp0, rresp0, bresp1, rresp1;

  gnss_stap dut (
    .clk(l_clk), .rst(rst),
    .adc_enable_i0(1'b1), .adc_valid_i0(adc_valid), .adc_data_i0(adc_i0),
    .adc_enable_q0(1'b1), .adc_valid_q0(adc_valid), .adc_data_q0(adc_q0),
    .adc_enable_i1(1'b1), .adc_valid_i1(adc_valid), .adc_data_i1(adc_i1),
    .adc_enable_q1(1'b1), .adc_valid_q1(adc_valid), .adc_data_q1(adc_q1),
    .aj_enable_i0(aj_en_i0), .aj_valid_i0(aj_v_i0), .aj_data_i0(aj_i0),
    .aj_enable_q0(aj_en_q0), .aj_valid_q0(aj_v_q0), .aj_data_q0(aj_q0),
    .aj_enable_i1(aj_en_i1), .aj_valid_i1(aj_v_i1), .aj_data_i1(aj_i1),
    .aj_enable_q1(aj_en_q1), .aj_valid_q1(aj_v_q1), .aj_data_q1(aj_q1),
    .s_axi_aclk(s_clk), .s_axi_aresetn(aresetn),
    .s_axi_awvalid(awvalid[0]), .s_axi_awaddr(awaddr[0]), .s_axi_awprot(3'd0), .s_axi_awready(awready0),
    .s_axi_wvalid(wvalid[0]), .s_axi_wdata(wdata[0]), .s_axi_wstrb(4'hF), .s_axi_wready(wready0),
    .s_axi_bvalid(bvalid0), .s_axi_bresp(bresp0), .s_axi_bready(bready[0]),
    .s_axi_arvalid(arvalid[0]), .s_axi_araddr(araddr[0]), .s_axi_arprot(3'd0), .s_axi_arready(arready0),
    .s_axi_rvalid(rvalid0), .s_axi_rresp(rresp0), .s_axi_rdata(rdata0), .s_axi_rready(rready[0])
  );

  gnss_passthrough pt (
    .clk(l_clk), .rst(rst),
    .adc_enable_i0(aj_en_i0), .adc_valid_i0(aj_v_i0), .adc_data_i0(aj_i0),
    .adc_enable_q0(aj_en_q0), .adc_valid_q0(aj_v_q0), .adc_data_q0(aj_q0),
    .adc_enable_i1(aj_en_i1), .adc_valid_i1(aj_v_i1), .adc_data_i1(aj_i1),
    .adc_enable_q1(aj_en_q1), .adc_valid_q1(aj_v_q1), .adc_data_q1(aj_q1),
    .dac_enable_i0(1'b1), .dac_valid_i0(dac_valid), .dac_enable_q0(1'b1), .dac_valid_q0(dac_valid),
    .dac_enable_i1(1'b1), .dac_valid_i1(dac_valid), .dac_enable_q1(1'b1), .dac_valid_q1(dac_valid),
    .dma_dac_data_i0(16'h1111), .dma_dac_data_q0(16'h2222),
    .dma_dac_data_i1(16'h3333), .dma_dac_data_q1(16'h4444),
    .dac_data_i0(dac_i0), .dac_data_q0(dac_q0), .dac_data_i1(dac_i1), .dac_data_q1(dac_q1),
    .s_axi_aclk(s_clk), .s_axi_aresetn(aresetn),
    .s_axi_awvalid(awvalid[1]), .s_axi_awaddr(awaddr[1]), .s_axi_awprot(3'd0), .s_axi_awready(awready1),
    .s_axi_wvalid(wvalid[1]), .s_axi_wdata(wdata[1]), .s_axi_wstrb(4'hF), .s_axi_wready(wready1),
    .s_axi_bvalid(bvalid1), .s_axi_bresp(bresp1), .s_axi_bready(bready[1]),
    .s_axi_arvalid(arvalid[1]), .s_axi_araddr(araddr[1]), .s_axi_arprot(3'd0), .s_axi_arready(arready1),
    .s_axi_rvalid(rvalid1), .s_axi_rresp(rresp1), .s_axi_rdata(rdata1), .s_axi_rready(rready[1])
  );

  // ---------------------------------------------------------------- vectors
  reg [2*M*16-1:0] stim [0:NS-1];
  reg [127:0]      cexp [0:`TB_NRES-1];
  reg [63:0]       wts  [0:`TB_T-1];
  reg [2*OW-1:0]   yexp [0:`TB_K-1];
  reg [31:0]       edg  [0:2*M*HT-1];
  initial begin
    $readmemh("stim.hex", stim);
    $readmemh("corr_exp.hex", cexp);
    $readmemh("weights.hex", wts);
    $readmemh("yexp.hex", yexp);
    $readmemh("edges.hex", edg);
  end
  // stim line = {re ch1, re ch0, im ch1, im ch0}
  function [15:0] s_re(input integer i, input integer ch); s_re = stim[i][2*M*16-1-16*(1-ch) -: 16]; endfunction
  function [15:0] s_im(input integer i, input integer ch); s_im = stim[i][M*16-1-16*(1-ch) -: 16]; endfunction

  // ---------------------------------------------------------------- AXI tasks
  integer k;
  initial for (k = 0; k < 2; k = k + 1) begin
    awaddr[k] = 0; araddr[k] = 0; awvalid[k] = 0; wvalid[k] = 0; bready[k] = 0;
    arvalid[k] = 0; rready[k] = 0; wdata[k] = 0;
  end
  wire awready_m[0:1], wready_m[0:1], bvalid_m[0:1], arready_m[0:1], rvalid_m[0:1];
  wire [31:0] rdata_m[0:1];
  assign awready_m[0] = awready0; assign awready_m[1] = awready1;
  assign wready_m[0]  = wready0;  assign wready_m[1]  = wready1;
  assign bvalid_m[0]  = bvalid0;  assign bvalid_m[1]  = bvalid1;
  assign arready_m[0] = arready0; assign arready_m[1] = arready1;
  assign rvalid_m[0]  = rvalid0;  assign rvalid_m[1]  = rvalid1;
  assign rdata_m[0]   = rdata0;   assign rdata_m[1]   = rdata1;

  task axi_wr(input integer sl, input [15:0] a, input [31:0] d);
    begin
      @(posedge s_clk);
      awaddr[sl] <= a; awvalid[sl] <= 1; wdata[sl] <= d; wvalid[sl] <= 1; bready[sl] <= 1;
      @(posedge s_clk);                      // valids now visible
      while (awvalid[sl] || wvalid[sl]) begin
        if (awready_m[sl] && awvalid[sl]) awvalid[sl] <= 0;   // handshake at this edge
        if (wready_m[sl]  && wvalid[sl])  wvalid[sl]  <= 0;
        @(posedge s_clk);
      end
      while (!bvalid_m[sl]) @(posedge s_clk);
      @(posedge s_clk); bready[sl] <= 0;
    end
  endtask

  task axi_rd(input integer sl, input [15:0] a, output [31:0] d);
    begin
      @(posedge s_clk);
      araddr[sl] <= a; arvalid[sl] <= 1; rready[sl] <= 1;
      @(posedge s_clk);
      while (!arready_m[sl]) @(posedge s_clk);
      arvalid[sl] <= 0;
      while (!rvalid_m[sl]) @(posedge s_clk);
      d = rdata_m[sl];
      @(posedge s_clk); rready[sl] <= 0;
    end
  endtask

  // ---------------------------------------------------------------- RX stream driver
  // drives samples [from, to) with adc_valid on every 2nd l_clk (on ph == 1)
  task stream(input integer from, input integer to);
    integer i;
    begin
      i = from;
      while (i < to) begin
        @(posedge l_clk);
        if (ph == 1'b0) begin
          adc_valid <= 1;
          adc_i0 <= s_re(i, 0); adc_q0 <= s_im(i, 0);
          adc_i1 <= s_re(i, 1); adc_q1 <= s_im(i, 1);
          i = i + 1;
        end else adc_valid <= 0;
      end
      @(posedge l_clk); adc_valid <= 0;
    end
  endtask

  // ---------------------------------------------------------------- monitors
  integer phase = 0;
  // D: gnss_stap output vs golden
  integer aj_cnt = 0, aj_err = 0;
  always @(posedge l_clk) if (phase == 4 && aj_v_i0) begin
    if ({aj_i0[11:0], aj_q0[11:0]} !== yexp[aj_cnt] ||
        aj_i0[15:12] !== {4{aj_i0[11]}} || aj_q0[15:12] !== {4{aj_q0[11]}} ||
        aj_i1 !== aj_i0 || aj_q1 !== aj_q0) begin
      aj_err = aj_err + 1;
      if (aj_err <= 8) $display("  AJ MISMATCH #%0d: got %h %h exp %h", aj_cnt, aj_i0, aj_q0, yexp[aj_cnt]);
    end
    aj_cnt = aj_cnt + 1;
  end

  // DAC stream checker: find the first sample of the expected sequence, then
  // require the following samples to follow it one by one
  reg  [31:0] dac_exp [0:`TB_K-1];
  integer dac_len = 0, dac_pos = -1, dac_ok = 0, dac_err = 0;
  always @(posedge l_clk) if (dac_valid && dac_len > 0) begin
    if (dac_pos < 0) begin
      if ({dac_i0, dac_q0} === dac_exp[0]) begin dac_pos = 1; dac_ok = 1; end
    end else if (dac_pos < dac_len - 40) begin
      if ({dac_i0, dac_q0} === dac_exp[dac_pos]) dac_ok = dac_ok + 1;
      else dac_err = dac_err + 1;
      dac_pos = dac_pos + 1;
    end
  end

  // ---------------------------------------------------------------- sequence
  integer i, m, p, fails, corr_err, raw_err;
  reg [31:0] r0, r1, r2, r3;
  reg [15:0] t_re, t_im;
  initial begin
    fails = 0;
    repeat (20) @(posedge s_clk);
    rst <= 0; aresetn <= 1;
    repeat (20) @(posedge s_clk);

    // ---- A. identities
    axi_rd(0, 16'hF000, r0);
    axi_rd(0, 16'h0000, r1);
    axi_rd(0, 16'h0004, r2);
    axi_rd(1, 16'h0000, r3);
    $display("A: GSTP=%h STAP=%h CONFIG=%h GCP1=%h", r0, r1, r2, r3);
    if (r0 !== 32'h47535450 || r1 !== 32'h53544150 || r2[7:0] !== 8'd2 || r3 !== 32'h47435031) begin
      $display("A: FAIL"); fails = fails + 1;
    end

    // ---- B. bypass through gnss_passthrough
    for (i = 0; i < NBYP; i = i + 1) begin
      t_re = s_re(i, 0); t_im = s_im(i, 0);
      dac_exp[i] = {t_re[11:0], 4'h0, t_im[11:0], 4'h0};
    end
    axi_wr(1, 16'h000C, 32'h1);             // pass_en
    dac_pos = -1; dac_ok = 0; dac_err = 0; dac_len = NBYP;
    stream(0, NBYP);
    repeat (200) @(posedge l_clk);
    $display("B: bypass DAC samples matched %0d, mismatched %0d", dac_ok, dac_err);
    if (dac_ok < NBYP - 60 || dac_err != 0) begin $display("B: FAIL"); fails = fails + 1; end
    dac_len = 0;

    // ---- C. covariance
    axi_wr(0, 16'h0018, `TB_L);
    axi_wr(0, 16'h0010, 32'h1);             // CORR_START (bresp => reached the core)
    stream(0, `TB_SL);
    r0 = 0;
    while (!(r0 & 32'h2)) begin repeat (100) @(posedge s_clk); axi_rd(0, 16'h0014, r0); end
    corr_err = 0;
    for (i = 0; i < `TB_NRES; i = i + 1) begin
      axi_rd(0, 16'h8000 + 16*i + 0,  r0);
      axi_rd(0, 16'h8000 + 16*i + 4,  r1);
      axi_rd(0, 16'h8000 + 16*i + 8,  r2);
      axi_rd(0, 16'h8000 + 16*i + 12, r3);
      if ({r1, r0, r3, r2} !== cexp[i]) begin
        corr_err = corr_err + 1;
        if (corr_err <= 5) $display("  CORR MISMATCH %0d: %h%h %h%h exp %h", i, r1, r0, r3, r2, cexp[i]);
      end
    end
    raw_err = 0;
    for (p = 0; p < 2; p = p + 1) begin
      axi_wr(0, 16'h0020, p ? `TB_L : 0);
      for (m = 0; m < M; m = m + 1)
        for (i = 0; i < HT; i = i + 1) begin
          axi_rd(0, 16'h0100 + 4*(64*m + i), r0);
          if (r0 !== edg[p*M*HT + m*HT + i]) raw_err = raw_err + 1;
        end
    end
    $display("C: correlations %0d/%0d mismatches, edge samples %0d/%0d mismatches",
             corr_err, `TB_NRES, raw_err, 2*M*HT);
    if (corr_err || raw_err) begin $display("C: FAIL"); fails = fails + 1; end

    // ---- D. anti-jam output
    for (i = 0; i < `TB_T; i = i + 1) begin
      axi_wr(0, 16'h1000 + 8*i,     wts[i][63:32]);
      axi_wr(0, 16'h1000 + 8*i + 4, wts[i][31:0]);
    end
    axi_wr(0, 16'h001C, `TB_SHIFT);
    axi_wr(0, 16'h0010, 32'h2);             // W_COMMIT
    axi_wr(0, 16'h0024, 32'h3);             // aj_en + ch1 copy
    repeat (40) @(posedge l_clk);
    for (i = 0; i < `TB_K; i = i + 1)
      dac_exp[i] = {yexp[i][23:12], 4'h0, yexp[i][11:0], 4'h0};
    dac_pos = -1; dac_ok = 0; dac_err = 0; dac_len = `TB_K;
    phase = 4;
    stream(`TB_SL, NS);
    repeat (200) @(posedge l_clk);
    phase = 5;
    axi_rd(0, 16'h0014, r0);
    $display("D: STAP outputs %0d checked, %0d mismatches; DAC matched %0d, mismatched %0d; overrun=%0d",
             aj_cnt, aj_err, dac_ok, dac_err, r0[3]);
    if (aj_err || aj_cnt != `TB_K || dac_err || dac_ok < `TB_K - 60 || r0[3]) begin
      $display("D: FAIL"); fails = fails + 1;
    end

    // ---- E. bridge timeout with l_clk stopped
    l_run = 0;
    #100000;                                // > 8192 s_clk cycles: l_clk declared dead
    axi_rd(0, 16'h0000, r0);
    axi_rd(0, 16'hF008, r1);
    axi_rd(0, 16'hF000, r2);
    $display("E: l_clk stopped: STAP ID read %h, WRAP_STATUS %h, GSTP %h", r0, r1, r2);
    if (r0 !== 32'hDEADC10C || r1[0] !== 1'b0 || r1[31:16] == 0 || r2 !== 32'h47535450) begin
      $display("E: FAIL"); fails = fails + 1;
    end
    l_run = 1;

    if (fails == 0) $display("*** PASS tb_gnss_stap ***");
    else            $display("*** FAIL tb_gnss_stap (%0d) ***", fails);
    $finish;
  end

  initial begin #200_000_000; $display("*** FAIL: timeout ***"); $finish; end
endmodule
