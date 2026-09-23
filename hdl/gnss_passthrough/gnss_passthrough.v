`timescale 1ns/100ps

module gnss_passthrough #(
  parameter FIFO_ADDR_WIDTH = 5   // elastic buffer depth = 2**FIFO_ADDR_WIDTH
) (
  // ---- sample-domain clock / reset (axi_ad9361 l_clk / rst) ----------------
  input               clk,
  input               rst,               // active high, from axi_ad9361/rst

  // ---- RX: from axi_ad9361 ADC interface ----------------------------------
  input               adc_enable_i0,
  input               adc_valid_i0,
  input      [15:0]   adc_data_i0,
  input               adc_enable_q0,
  input               adc_valid_q0,
  input      [15:0]   adc_data_q0,
  input               adc_enable_i1,
  input               adc_valid_i1,
  input      [15:0]   adc_data_i1,
  input               adc_enable_q1,
  input               adc_valid_q1,
  input      [15:0]   adc_data_q1,

  // ---- TX: request strobes from axi_ad9361 DAC interface ------------------
  input               dac_enable_i0,
  input               dac_valid_i0,
  input               dac_enable_q0,
  input               dac_valid_q0,
  input               dac_enable_i1,
  input               dac_valid_i1,
  input               dac_enable_q1,
  input               dac_valid_q1,

  // ---- TX: vendor DMA/DDS sourced data (from util_rfifo dout_data_*) ------
  input      [15:0]   dma_dac_data_i0,
  input      [15:0]   dma_dac_data_q0,
  input      [15:0]   dma_dac_data_i1,
  input      [15:0]   dma_dac_data_q1,

  // ---- TX: muxed data driven into axi_ad9361 ------------------------------
  output     [15:0]   dac_data_i0,
  output     [15:0]   dac_data_q0,
  output     [15:0]   dac_data_i1,
  output     [15:0]   dac_data_q1,

  // ---- AXI4-Lite slave (sys_cpu_clk domain) -------------------------------
  input               s_axi_aclk,
  input               s_axi_aresetn,
  input               s_axi_awvalid,
  input      [15:0]   s_axi_awaddr,
  input      [ 2:0]   s_axi_awprot,
  output              s_axi_awready,
  input               s_axi_wvalid,
  input      [31:0]   s_axi_wdata,
  input      [ 3:0]   s_axi_wstrb,
  output              s_axi_wready,
  output              s_axi_bvalid,
  output     [ 1:0]   s_axi_bresp,
  input               s_axi_bready,
  input               s_axi_arvalid,
  input      [15:0]   s_axi_araddr,
  input      [ 2:0]   s_axi_arprot,
  output              s_axi_arready,
  output              s_axi_rvalid,
  output     [ 1:0]   s_axi_rresp,
  output     [31:0]   s_axi_rdata,
  input               s_axi_rready
);

  localparam [31:0] CORE_ID      = 32'h47435031;   // "GCP1"
  // v1.3 -- fixes a bug in v1.2: u_crpa_core's alpha_wr was tied to 1'b0
  // below, so alpha_reg inside pi_power_inversion.v NEVER loaded alpha_in
  // and stayed at its reset value (ALPHA_INIT, 1.0) forever. CRPA_COEF(0)
  // writes reached crpa_alpha_sync (the CDC output) correctly and read
  // back exactly what was written -- so software saw a working control
  // register -- but the algorithm itself always ran at the fixed default,
  // no matter what alpha was commanded. Confirmed with a targeted
  // simulation probe (alpha_reg stuck at 256 after writing 512) before
  // this fix; tb_gnss_passthrough.v now asserts alpha_reg tracks a
  // non-default write so this class of bug can't hide behind a testbench
  // that only ever wrote back the reset value.
  // v1.4 -- adds the NORMALIZED power-inversion core (pi_power_inversion_normalized.v,
  // Eq. 13 of the paper) as a runtime-selectable ALTERNATIVE to the standard
  // core, not a replacement: CONTROL[4]=0 (the reset default) selects the
  // standard core, exactly the already-verified v1.3 behavior, unchanged.
  // CONTROL[4]=1 selects the normalized core instead. Both cores run
  // continuously and adapt together whenever pass_en=1 regardless of which
  // one is selected -- only the OUTPUT mux changes, so switching modes at
  // runtime is bumpless (the newly-selected core is not starting cold) and
  // reset/pass_en behavior is identical for both. CRPA_COEF(1) (previously
  // reserved, unconsumed like CRPA_COEF(0) was before v1.3) now carries the
  // normalized core's gamma (Eq. 13's regulariser), same CDC pattern as
  // alpha, same continuously-loaded fix (no write-strobe to get wrong).
  // v1.5 -- adds a THIRD selectable CRPA core, pi_power_inversion_pl_npi.v
  // (PL-NPI, Jia et al. IEEE Access 2023 Eq. 11). CONTROL[5:4] is now a
  // 2-bit mode select, not a single bit: 00=standard (default, unchanged),
  // 01=normalized, 10=PL-NPI, 11=reserved (falls back to standard, the
  // safest choice for an undefined encoding). All three cores run and
  // adapt continuously regardless of which is selected, same bumpless-
  // switching reasoning as v1.4. CRPA_COEF(2) (previously reserved) carries
  // PL-NPI's OWN gamma, independent of CRPA_COEF(1)'s normalized-core
  // gamma -- deliberately not shared, so tuning one mode's regulariser
  // never silently perturbs the other's behaviour when you switch modes.
  localparam [31:0] CORE_VERSION = 32'h00010005;   // v1.5 -- adds selectable PL-NPI CRPA core
  localparam        AW           = FIFO_ADDR_WIDTH;

  // ==========================================================================
  //  AXI4-Lite slave  (s_axi_aclk domain)  -- unchanged from Phase 1
  // ==========================================================================
  reg         axi_awready = 1'b0;
  reg         axi_wready  = 1'b0;
  reg         axi_bvalid  = 1'b0;
  reg         axi_arready = 1'b0;
  reg         axi_rvalid  = 1'b0;
  reg  [31:0] axi_rdata   = 32'd0;
  reg  [13:0] axi_awaddr_r = 14'd0;
  reg  [13:0] axi_araddr_r = 14'd0;

  assign s_axi_awready = axi_awready;
  assign s_axi_wready  = axi_wready;
  assign s_axi_bvalid  = axi_bvalid;
  assign s_axi_bresp   = 2'b00;
  assign s_axi_arready = axi_arready;
  assign s_axi_rvalid  = axi_rvalid;
  assign s_axi_rresp   = 2'b00;
  assign s_axi_rdata   = axi_rdata;

  wire        wr_hs = s_axi_awvalid & s_axi_wvalid & ~axi_bvalid &
                      ~axi_awready & ~axi_wready;

  always @(posedge s_axi_aclk) begin
    if (s_axi_aresetn == 1'b0) begin
      axi_awready  <= 1'b0;
      axi_wready   <= 1'b0;
      axi_bvalid   <= 1'b0;
      axi_awaddr_r <= 14'd0;
    end else begin
      axi_awready <= wr_hs;
      axi_wready  <= wr_hs;
      if (wr_hs)
        axi_awaddr_r <= s_axi_awaddr[15:2];
      if (axi_awready)
        axi_bvalid <= 1'b1;
      else if (s_axi_bready && axi_bvalid)
        axi_bvalid <= 1'b0;
    end
  end

  always @(posedge s_axi_aclk) begin
    if (s_axi_aresetn == 1'b0) begin
      axi_arready  <= 1'b0;
      axi_rvalid   <= 1'b0;
      axi_araddr_r <= 14'd0;
    end else begin
      axi_arready <= s_axi_arvalid & ~axi_arready & ~axi_rvalid;
      if (s_axi_arvalid && axi_arready)
        axi_araddr_r <= s_axi_araddr[15:2];
      if (s_axi_arvalid && axi_arready)
        axi_rvalid <= 1'b1;
      else if (axi_rvalid && s_axi_rready)
        axi_rvalid <= 1'b0;
    end
  end

  wire        reg_wr    = axi_awready;
  wire [11:0] reg_waddr = axi_awaddr_r[11:0];

  reg  [31:0] reg_scratch = 32'd0;
  reg  [31:0] reg_control = 32'd0;
  reg  [31:0] crpa_coef [0:15];

  integer ci;
  initial for (ci = 0; ci < 16; ci = ci + 1) crpa_coef[ci] = 32'd0;

  always @(posedge s_axi_aclk) begin
    if (s_axi_aresetn == 1'b0) begin
      reg_scratch <= 32'd0;
      reg_control <= 32'd0;
    end else if (reg_wr) begin
      case (reg_waddr)
        12'h002: reg_scratch <= s_axi_wdata;
        12'h003: reg_control <= s_axi_wdata;
        default: ;
      endcase
      if (reg_waddr >= 12'h010 && reg_waddr <= 12'h01F)
        crpa_coef[reg_waddr[3:0]] <= s_axi_wdata;
    end
  end

  wire [31:0] v_status, v_rx_cnt0, v_tx_cnt0, v_ovf_cnt, v_unf_cnt;
  wire [31:0] v_rx_snap0, v_tx_snap0, v_level, v_rx_cnt1, v_tx_cnt1, v_rx_snap1;

  always @(posedge s_axi_aclk) begin
    if (s_axi_arvalid && axi_arready) begin
    case (s_axi_araddr[13:2])
      12'h000: axi_rdata <= CORE_ID;
      12'h001: axi_rdata <= CORE_VERSION;
      12'h002: axi_rdata <= reg_scratch;
      12'h003: axi_rdata <= reg_control;
      12'h004: axi_rdata <= v_status;
      12'h005: axi_rdata <= v_rx_cnt0;
      12'h006: axi_rdata <= v_tx_cnt0;
      12'h007: axi_rdata <= v_ovf_cnt;
      12'h008: axi_rdata <= v_unf_cnt;
      12'h009: axi_rdata <= v_rx_snap0;
      12'h00A: axi_rdata <= v_tx_snap0;
      12'h00B: axi_rdata <= v_level;
      12'h00C: axi_rdata <= v_rx_cnt1;
      12'h00D: axi_rdata <= v_tx_cnt1;
      12'h00E: axi_rdata <= v_rx_snap1;
      12'h010, 12'h011, 12'h012, 12'h013,
      12'h014, 12'h015, 12'h016, 12'h017,
      12'h018, 12'h019, 12'h01A, 12'h01B,
      12'h01C, 12'h01D, 12'h01E, 12'h01F:
               axi_rdata <= crpa_coef[s_axi_araddr[5:2]];
      default: axi_rdata <= 32'd0;
    endcase
    end
  end

  // ==========================================================================
  //  CDC: control  s_axi_aclk -> clk   (two-flop synchronisers) -- unchanged
  // ==========================================================================
  (* ASYNC_REG = "TRUE" *) reg [8:0] ctrl_meta = 9'd0;
  (* ASYNC_REG = "TRUE" *) reg [8:0] ctrl_sync = 9'd0;
  // v1.5: bits [5:4] (previously bit 4 alone in v1.4, bit 5 was unused
  // 4'b0000 padding before that) now carry the 2-bit CRPA mode select.
  wire [8:0] ctrl_raw = {reg_control[8], 2'b00, reg_control[5:4], reg_control[3:0]};

  always @(posedge clk) begin
    ctrl_meta <= ctrl_raw;
    ctrl_sync <= ctrl_meta;
  end

  wire pass_en   = ctrl_sync[0];
  wire mute      = ctrl_sync[1];
  wire swap_iq   = ctrl_sync[2];
  wire ch1_copy  = ctrl_sync[3];
  // 2'b00 = standard (default), 2'b01 = normalized, 2'b10 = PL-NPI,
  // 2'b11 = reserved/undefined -> falls back to standard below.
  wire [1:0] crpa_mode = ctrl_sync[5:4];
  wire cnt_clear = ctrl_sync[8];

  // ==========================================================================
  //  CDC: CRPA_COEF[0] (alpha) s_axi_aclk -> clk, same two-flop pattern as
  //  CONTROL above -- alpha is quasi-static software config, not a per-
  //  sample signal, so bit-wise synchronisation is sufficient here too.
  // ==========================================================================
  (* ASYNC_REG = "TRUE" *) reg [15:0] crpa_alpha_meta = 16'd256;   // 256/2^8 = 1.0
  (* ASYNC_REG = "TRUE" *) reg [15:0] crpa_alpha_sync = 16'd256;
  always @(posedge clk) begin
    crpa_alpha_meta <= crpa_coef[0][15:0];
    crpa_alpha_sync <= crpa_alpha_meta;
  end

  // ==========================================================================
  //  CDC: CRPA_COEF[1] (gamma, v1.4) s_axi_aclk -> clk, identical pattern to
  //  alpha above -- continuously loaded every cycle, no write-strobe, so
  //  there is no enable line left to hardwire wrong the way alpha's was in
  //  v1.2 (see CORE_VERSION's history above). Feeds the normalized core's
  //  regulariser (Eq. 13's gamma > 0); CRPA_GAMMA_W is wider than the 32-bit
  //  AXI register, so this only zero-extends -- software can still supply
  //  any value up to 2^32-1, far more range than a regularisation floor
  //  needs.
  // ==========================================================================
  localparam integer CRPA_GAMMA_W = 2*16 + 2 + 2;   // matches pi_power_inversion_normalized's
                                                     // own POW_W/GAMMA_W derivation for
                                                     // DATA_W=16, M=2: 2*DATA_W+clog2(2*M)+2
  (* ASYNC_REG = "TRUE" *) reg [CRPA_GAMMA_W-1:0] crpa_gamma_meta = {{(CRPA_GAMMA_W-1){1'b0}}, 1'b1};   // gamma=1
  (* ASYNC_REG = "TRUE" *) reg [CRPA_GAMMA_W-1:0] crpa_gamma_sync = {{(CRPA_GAMMA_W-1){1'b0}}, 1'b1};
  always @(posedge clk) begin
    crpa_gamma_meta <= {{(CRPA_GAMMA_W-32){1'b0}}, crpa_coef[1]};
    crpa_gamma_sync <= crpa_gamma_meta;
  end

  // ==========================================================================
  //  CDC: CRPA_COEF[2] (PL-NPI's OWN gamma, v1.5) s_axi_aclk -> clk, same
  //  pattern again -- deliberately a SEPARATE register from CRPA_COEF[1]
  //  above (see CORE_VERSION's v1.5 comment for why: independent tuning
  //  per mode, no cross-mode surprise when switching).
  // ==========================================================================
  (* ASYNC_REG = "TRUE" *) reg [CRPA_GAMMA_W-1:0] crpa_gamma2_meta = {{(CRPA_GAMMA_W-1){1'b0}}, 1'b1};
  (* ASYNC_REG = "TRUE" *) reg [CRPA_GAMMA_W-1:0] crpa_gamma2_sync = {{(CRPA_GAMMA_W-1){1'b0}}, 1'b1};
  always @(posedge clk) begin
    crpa_gamma2_meta <= {{(CRPA_GAMMA_W-32){1'b0}}, crpa_coef[2]};
    crpa_gamma2_sync <= crpa_gamma2_meta;
  end

  // ==========================================================================
  //  Elastic buffer (one per channel) -- unchanged from Phase 1
  // ==========================================================================
  (* ram_style = "distributed" *) reg [31:0] fifo0 [0:(1<<AW)-1];
  (* ram_style = "distributed" *) reg [31:0] fifo1 [0:(1<<AW)-1];
  reg  [AW:0] wptr0 = 0, rptr0 = 0;
  reg  [AW:0] wptr1 = 0, rptr1 = 0;

  wire [AW:0] level0 = wptr0 - rptr0;
  wire [AW:0] level1 = wptr1 - rptr1;
  wire        full0  = (level0 == (1<<AW));
  wire        empty0 = (level0 == 0);
  wire        full1  = (level1 == (1<<AW));
  wire        empty1 = (level1 == 0);

  reg  primed0 = 1'b0, primed1 = 1'b0;
  always @(posedge clk) begin
    if (rst || !pass_en) begin
      primed0 <= 1'b0;
      primed1 <= 1'b0;
    end else begin
      if (level0 >= (1<<(AW-1))) primed0 <= 1'b1;
      if (level1 >= (1<<(AW-1))) primed1 <= 1'b1;
    end
  end

  // ==========================================================================
  //  PROCESSING CORE -- M=2 Power Inversion (CRPA), replacing Phase 1 identity.
  //
  //  Array mapping: element 0 = (adc_data_i0, adc_data_q0), element 1 =
  //  (adc_data_i1, adc_data_q1) -- both already in RX format (12-bit signed,
  //  right-aligned, sign-extended into [15:12]). pi_power_inversion treats
  //  them as plain signed integers; the algorithm is correct on any
  //  consistent linear fixed-point representation, so no reformatting is
  //  needed going in.
  //
  //  The core needs BOTH elements' samples together to produce one result,
  //  so its own trigger is the AND of all four RX valid/enable strobes --
  //  NOT the same per-channel adc_valid_i0/adc_valid_i1 the elastic buffer's
  //  write-enables used in Phase 1. See the wr_en0/wr_en1 note below: this
  //  is the one change outside this section a correct integration requires.
  //
  //  Result is a SINGLE combined stream (M elements nulled to 1) -- there
  //  is no independent second nulled value for ch1. CONTROL[3] ch1_copy
  //  (already existing, unchanged) mirrors ch0's result onto ch1's DAC;
  //  that is the intended way to run this with a 2-element array, not a
  //  new mechanism. proc_i1/proc_q1 mirror the same result so fifo1 stays
  //  populated with a well-defined value if ch1_copy is ever cleared.
  //
  //  Alpha comes from CRPA_COEF[0] (previously reserved, unconsumed).
  //  Adaptation runs only while pass_en=1, matching how every other stage
  //  in this file is already gated on pass_en.
  //
  //  v1.4: a SECOND core (pi_power_inversion_normalized, Eq. 13) runs in
  //  parallel, fed the identical sample stream and reset/enable, gamma from
  //  CRPA_COEF[1]. CONTROL[4] (crpa_mode_normalized) selects which core's
  //  s_re/s_im/s_valid actually reach the saturate-and-output stage below;
  //  the other keeps running and adapting unselected, which is what makes
  //  switching modes bumpless rather than a cold restart.
  //
  //  v1.5: a THIRD core (pi_power_inversion_pl_npi, Eq. 11) runs alongside
  //  the other two the same way, gamma from CRPA_COEF[2] (its own, not
  //  shared with the normalized core's CRPA_COEF[1]). CONTROL[5:4] now
  //  selects among all three; see CORE_VERSION's v1.5 comment.
  // ==========================================================================
  localparam integer CRPA_DATA_W      = 16;
  localparam integer CRPA_WEIGHT_W    = 32;
  localparam integer CRPA_WEIGHT_FRAC = 20;
  localparam integer CRPA_ALPHA_W     = 16;
  localparam integer CRPA_ALPHA_FRAC  = 8;
  localparam integer CRPA_LPF_SHIFT   = 18;
  // Must match pi_power_inversion's own S_W for M=2: PROD1_W = DATA_W+WEIGHT_W+1,
  // SUM_GROWTH = clog2(2) = 1, S_W = PROD1_W + SUM_GROWTH.
  localparam integer CRPA_S_W         = CRPA_DATA_W + CRPA_WEIGHT_W + 2;

  wire signed [CRPA_S_W-1:0] crpa_s_re_std, crpa_s_im_std;
  wire                       crpa_s_valid_std;
  wire [2*CRPA_WEIGHT_W-1:0] crpa_w_re_std, crpa_w_im_std;
  wire                       crpa_weights_valid_std;

  wire signed [CRPA_S_W-1:0] crpa_s_re_norm, crpa_s_im_norm;
  wire                       crpa_s_valid_norm;
  wire [2*CRPA_WEIGHT_W-1:0] crpa_w_re_norm, crpa_w_im_norm;
  wire                       crpa_weights_valid_norm;
  wire [32:0]                crpa_alpha_mag_norm;         // diagnostic only, not read back yet
  wire                       crpa_alpha_mag_valid_norm;

  wire signed [CRPA_S_W-1:0] crpa_s_re_pl, crpa_s_im_pl;
  wire                       crpa_s_valid_pl;
  wire [2*CRPA_WEIGHT_W-1:0] crpa_w_re_pl, crpa_w_im_pl;
  wire                       crpa_weights_valid_pl;
  wire [32:0]                crpa_alpha_mag_pl;           // diagnostic only, not read back yet
  wire                       crpa_alpha_mag_valid_pl;
  wire [1:0]                 crpa_pl_gain_band;           // diagnostic only, not read back yet

  wire                       crpa_aresetn = ~rst & pass_en;

  // All four RX strobes together -- both elements must be sampled in lockstep
  // for the array math to be meaningful; on this board's AD9361 interface
  // they arrive on the same cycle in normal 2R2T operation.
  wire crpa_sample_valid = adc_valid_i0 & adc_enable_i0
                          & adc_valid_q0 & adc_enable_q0
                          & adc_valid_i1 & adc_enable_i1
                          & adc_valid_q1 & adc_enable_q1;

  pi_power_inversion #(
      .M           (2),
      .DATA_W      (CRPA_DATA_W),
      .WEIGHT_W    (CRPA_WEIGHT_W),
      .WEIGHT_FRAC (CRPA_WEIGHT_FRAC),
      .ALPHA_W     (CRPA_ALPHA_W),
      .ALPHA_FRAC  (CRPA_ALPHA_FRAC),
      .ALPHA_INIT  (1 << CRPA_ALPHA_FRAC),
      .LPF_SHIFT   (CRPA_LPF_SHIFT)
  ) u_crpa_core (
      .clk           (clk),
      .aresetn       (crpa_aresetn),
      .x_re          ({adc_data_i1[CRPA_DATA_W-1:0], adc_data_i0[CRPA_DATA_W-1:0]}),
      .x_im          ({adc_data_q1[CRPA_DATA_W-1:0], adc_data_q0[CRPA_DATA_W-1:0]}),
      .sample_valid  (crpa_sample_valid),
      .alpha_in      (crpa_alpha_sync),
      // v1.3 fix: was 1'b0 (see CORE_VERSION comment above). crpa_alpha_sync
      // is already a stable, CDC-synchronized value that only changes when
      // software writes CRPA_COEF(0), so continuously loading it every
      // cycle is correct -- no separate strobe/handshake is needed, and
      // there is no metastability risk here that alpha_wr pulsing would
      // have avoided and this doesn't.
      .alpha_wr      (1'b1),
      .adapt_en      (pass_en),
      .s_re          (crpa_s_re_std),
      .s_im          (crpa_s_im_std),
      .s_valid       (crpa_s_valid_std),
      .w_re          (crpa_w_re_std),
      .w_im          (crpa_w_im_std),
      .weights_valid (crpa_weights_valid_std)
  );

  pi_power_inversion_normalized #(
      .M           (2),
      .DATA_W      (CRPA_DATA_W),
      .WEIGHT_W    (CRPA_WEIGHT_W),
      .WEIGHT_FRAC (CRPA_WEIGHT_FRAC),
      .LPF_SHIFT   (CRPA_LPF_SHIFT)
      // Q_FRAC, ALPHA_GAIN_SHIFT, GAMMA_INIT left at their module defaults
      // (32, 17, 1) -- the same values tb_pi_power_inversion_normalized.v
      // verified converge correctly at this project's amp=100 calibration
      // point. See that module's header for the honest amplitude-
      // generalisation caveat before assuming this holds at every signal
      // scale this board might see.
  ) u_crpa_core_normalized (
      .clk            (clk),
      .aresetn        (crpa_aresetn),
      .x_re           ({adc_data_i1[CRPA_DATA_W-1:0], adc_data_i0[CRPA_DATA_W-1:0]}),
      .x_im           ({adc_data_q1[CRPA_DATA_W-1:0], adc_data_q0[CRPA_DATA_W-1:0]}),
      .sample_valid   (crpa_sample_valid),
      .gamma_in       (crpa_gamma_sync[CRPA_GAMMA_W-1:0]),
      .adapt_en       (pass_en),
      .s_re           (crpa_s_re_norm),
      .s_im           (crpa_s_im_norm),
      .s_valid        (crpa_s_valid_norm),
      .w_re           (crpa_w_re_norm),
      .w_im           (crpa_w_im_norm),
      .weights_valid  (crpa_weights_valid_norm),
      .alpha_mag      (crpa_alpha_mag_norm),
      .alpha_mag_valid(crpa_alpha_mag_valid_norm)
  );

  pi_power_inversion_pl_npi #(
      .M           (2),
      .DATA_W      (CRPA_DATA_W),
      .WEIGHT_W    (CRPA_WEIGHT_W),
      .WEIGHT_FRAC (CRPA_WEIGHT_FRAC),
      .LPF_SHIFT   (CRPA_LPF_SHIFT)
      // Q_FRAC, ALPHA_GAIN_SHIFT, GAMMA_INIT, GAIN_FRAC, THRESH1/2/3 left at
      // their module defaults -- the same values
      // tb_pi_power_inversion_pl_npi.v verified converge (one sample
      // faster than the plain normalized core) at this project's amp=100
      // calibration point. See that module's header for the measured
      // gain-band coverage and the same amplitude-calibration caveat the
      // normalized core's own header documents.
  ) u_crpa_core_pl_npi (
      .clk            (clk),
      .aresetn        (crpa_aresetn),
      .x_re           ({adc_data_i1[CRPA_DATA_W-1:0], adc_data_i0[CRPA_DATA_W-1:0]}),
      .x_im           ({adc_data_q1[CRPA_DATA_W-1:0], adc_data_q0[CRPA_DATA_W-1:0]}),
      .sample_valid   (crpa_sample_valid),
      .gamma_in       (crpa_gamma2_sync[CRPA_GAMMA_W-1:0]),
      .adapt_en       (pass_en),
      .s_re           (crpa_s_re_pl),
      .s_im           (crpa_s_im_pl),
      .s_valid        (crpa_s_valid_pl),
      .w_re           (crpa_w_re_pl),
      .w_im           (crpa_w_im_pl),
      .weights_valid  (crpa_weights_valid_pl),
      .alpha_mag      (crpa_alpha_mag_pl),
      .alpha_mag_valid(crpa_alpha_mag_valid_pl),
      .pl_gain_band   (crpa_pl_gain_band)
  );

  // ---- output mux: CONTROL[5:4] selects which core's result actually
  //      reaches the DAC (00=standard, 01=normalized, 10=PL-NPI,
  //      11=reserved -> standard). All three cores keep running and
  //      adapting either way (see the v1.4/v1.5 comments above) -- only
  //      this selection changes. ----
  reg signed [CRPA_S_W-1:0] crpa_s_re;
  reg signed [CRPA_S_W-1:0] crpa_s_im;
  reg                       crpa_s_valid;
  always @(*) begin
    case (crpa_mode)
      2'b01:   begin crpa_s_re = crpa_s_re_norm; crpa_s_im = crpa_s_im_norm; crpa_s_valid = crpa_s_valid_norm; end
      2'b10:   begin crpa_s_re = crpa_s_re_pl;   crpa_s_im = crpa_s_im_pl;   crpa_s_valid = crpa_s_valid_pl;   end
      default: begin crpa_s_re = crpa_s_re_std;  crpa_s_im = crpa_s_im_std;  crpa_s_valid = crpa_s_valid_std;  end
    endcase
  end

  // Round-to-nearest, saturate to the 12-bit signed range this file's RX
  // format actually carries -- combinational, so the saturated value is
  // ready the SAME cycle crpa_s_valid pulses (s_re/s_im are already stable
  // that cycle, straight off pi_power_inversion's own registered output).
  // Doing this here, not at the OUTPUT SAMPLE ALIGNMENT stage below, is
  // exactly what that stage's own comment requires: "it must SATURATE
  // before reaching this point... truncating to [11:0] here would wrap."
  function signed [11:0] crpa_sat12(input signed [CRPA_S_W-1:0] v);
    reg signed [CRPA_S_W-1:0] rounded;
    localparam signed [CRPA_S_W-1:0] SAT_MAX = 2047;
    localparam signed [CRPA_S_W-1:0] SAT_MIN = -2048;
    begin
      rounded = (v + (1 <<< (CRPA_WEIGHT_FRAC-1))) >>> CRPA_WEIGHT_FRAC;
      if (rounded > SAT_MAX)      crpa_sat12 = SAT_MAX[11:0];
      else if (rounded < SAT_MIN) crpa_sat12 = SAT_MIN[11:0];
      else                        crpa_sat12 = rounded[11:0];
    end
  endfunction

  wire signed [11:0] crpa_re12 = crpa_sat12(crpa_s_re);
  wire signed [11:0] crpa_im12 = crpa_sat12(crpa_s_im);

  wire [15:0] proc_i0 = {{4{crpa_re12[11]}}, crpa_re12};
  wire [15:0] proc_q0 = {{4{crpa_im12[11]}}, crpa_im12};
  wire [15:0] proc_i1 = proc_i0;   // mirrors ch0's result; ch1_copy=1 is the
  wire [15:0] proc_q1 = proc_q0;   // intended way this runs with a 2-element array
  // ======================= END PROCESSING CORE ==============================

  // wr_en0/wr_en1 sit OUTSIDE the marked section above, but changing what
  // drives proc_i0/proc_q0 requires changing these too: Phase 1's identity
  // core needed no time to compute, so gating the buffer write on the RAW
  // adc_valid_i0/adc_valid_i1 strobes was correct -- proc_i0 WAS adc_data_i0,
  // same cycle. The CRPA core is NOT combinational: a new result is ready
  // exactly when crpa_s_valid pulses, not when the raw RX sample arrived.
  // Gating the write on adc_valid_i0/i1 unchanged would write fifo0/fifo1
  // with the PREVIOUS result, one core-latency late, every single sample.
  // Both channels' writes now fire together off the one combined result.
  wire wr_en0 = pass_en & crpa_s_valid;
  wire wr_en1 = pass_en & crpa_s_valid;
  wire rd_en0 = pass_en & dac_valid_i0 & primed0;   // unchanged: TX-side
  wire rd_en1 = pass_en & dac_valid_i1 & primed1;   // consumption, independent

  reg  [31:0] rd_data0 = 32'd0;
  reg  [31:0] rd_data1 = 32'd0;

  always @(posedge clk) begin
    if (rst) begin
      wptr0 <= 0; rptr0 <= 0;
      wptr1 <= 0; rptr1 <= 0;
    end else begin
      if (wr_en0 && !full0) begin
        fifo0[wptr0[AW-1:0]] <= {proc_q0, proc_i0};
        wptr0 <= wptr0 + 1'b1;
      end
      if (rd_en0 && !empty0) begin
        rd_data0 <= fifo0[rptr0[AW-1:0]];
        rptr0 <= rptr0 + 1'b1;
      end
      if (wr_en1 && !full1) begin
        fifo1[wptr1[AW-1:0]] <= {proc_q1, proc_i1};
        wptr1 <= wptr1 + 1'b1;
      end
      if (rd_en1 && !empty1) begin
        rd_data1 <= fifo1[rptr1[AW-1:0]];
        rptr1 <= rptr1 + 1'b1;
      end
    end
  end

  // ==========================================================================
  //  Output mux -- unchanged from Phase 1
  // ==========================================================================
  wire [15:0] pt_i0 = swap_iq ? rd_data0[31:16] : rd_data0[15:0];
  wire [15:0] pt_q0 = swap_iq ? rd_data0[15:0]  : rd_data0[31:16];
  wire [31:0] src1  = ch1_copy ? rd_data0 : rd_data1;
  wire [15:0] pt_i1 = swap_iq ? src1[31:16] : src1[15:0];
  wire [15:0] pt_q1 = swap_iq ? src1[15:0]  : src1[31:16];

  // ==========================================================================
  //  OUTPUT SAMPLE ALIGNMENT -- unchanged from Phase 1. Correct here for any
  //  value ONLY because the processing core above already saturated to the
  //  12-bit signed range before this point.
  // ==========================================================================
  wire [15:0] al_i0 = {pt_i0[11:0], 4'b0000};
  wire [15:0] al_q0 = {pt_q0[11:0], 4'b0000};
  wire [15:0] al_i1 = {pt_i1[11:0], 4'b0000};
  wire [15:0] al_q1 = {pt_q1[11:0], 4'b0000};

  assign dac_data_i0 = mute ? 16'd0 : (pass_en ? al_i0 : dma_dac_data_i0);
  assign dac_data_q0 = mute ? 16'd0 : (pass_en ? al_q0 : dma_dac_data_q0);
  assign dac_data_i1 = mute ? 16'd0 : (pass_en ? al_i1 : dma_dac_data_i1);
  assign dac_data_q1 = mute ? 16'd0 : (pass_en ? al_q1 : dma_dac_data_q1);

  // ==========================================================================
  //  Counters and sticky flags -- unchanged from Phase 1
  // ==========================================================================
  reg [31:0] rx_cnt0 = 32'd0, tx_cnt0 = 32'd0;
  reg [31:0] rx_cnt1 = 32'd0, tx_cnt1 = 32'd0;
  reg [31:0] ovf_cnt = 32'd0, unf_cnt = 32'd0;
  reg [31:0] rx_snap0 = 32'd0, tx_snap0 = 32'd0, rx_snap1 = 32'd0;
  reg        ovf_sticky = 1'b0, unf_sticky = 1'b0;

  always @(posedge clk) begin
    if (rst || cnt_clear) begin
      rx_cnt0 <= 32'd0; tx_cnt0 <= 32'd0;
      rx_cnt1 <= 32'd0; tx_cnt1 <= 32'd0;
      ovf_cnt <= 32'd0; unf_cnt <= 32'd0;
      ovf_sticky <= 1'b0; unf_sticky <= 1'b0;
      rx_snap0 <= 32'd0; tx_snap0 <= 32'd0; rx_snap1 <= 32'd0;
    end else begin
      if (adc_valid_i0) begin
        rx_cnt0  <= rx_cnt0 + 1'b1;
        rx_snap0 <= {adc_data_q0, adc_data_i0};
      end
      if (adc_valid_i1) begin
        rx_cnt1  <= rx_cnt1 + 1'b1;
        rx_snap1 <= {adc_data_q1, adc_data_i1};
      end
      if (dac_valid_i0) begin
        tx_cnt0  <= tx_cnt0 + 1'b1;
        tx_snap0 <= {dac_data_q0, dac_data_i0};
      end
      if (dac_valid_i1)
        tx_cnt1 <= tx_cnt1 + 1'b1;

      if (wr_en0 && full0) begin
        ovf_cnt    <= ovf_cnt + 1'b1;
        ovf_sticky <= 1'b1;
      end
      if (rd_en0 && empty0) begin
        unf_cnt    <= unf_cnt + 1'b1;
        unf_sticky <= 1'b1;
      end
    end
  end

  wire [31:0] status_w;
  assign status_w = { 15'd0, pass_en,
                      6'd0, unf_sticky, ovf_sticky,
                      full1, empty1, full0, empty0,
                      dac_enable_q0, dac_enable_i0,
                      adc_enable_q0, adc_enable_i0 };

  wire [5:0] level0_6 = level0;
  wire [5:0] level1_6 = level1;
  wire [31:0] level_w = { 18'd0, level1_6, 2'd0, level0_6 };

  // ==========================================================================
  //  CDC: status  clk -> s_axi_aclk -- unchanged from Phase 1
  // ==========================================================================
  reg  [7:0]  cap_div    = 8'd0;
  reg         cap_toggle = 1'b0;
  reg  [31:0] cap_status, cap_rx0, cap_tx0, cap_ovf, cap_unf;
  reg  [31:0] cap_rxs0, cap_txs0, cap_lvl, cap_rx1, cap_tx1, cap_rxs1;

  always @(posedge clk) begin
    cap_div <= cap_div + 1'b1;
    if (cap_div == 8'd0) begin
      cap_status <= status_w;
      cap_rx0    <= rx_cnt0;
      cap_tx0    <= tx_cnt0;
      cap_ovf    <= ovf_cnt;
      cap_unf    <= unf_cnt;
      cap_rxs0   <= rx_snap0;
      cap_txs0   <= tx_snap0;
      cap_lvl    <= level_w;
      cap_rx1    <= rx_cnt1;
      cap_tx1    <= tx_cnt1;
      cap_rxs1   <= rx_snap1;
      cap_toggle <= ~cap_toggle;
    end
  end

  (* ASYNC_REG = "TRUE" *) reg tog_meta = 1'b0;
  (* ASYNC_REG = "TRUE" *) reg tog_sync = 1'b0;
  reg                         tog_prev = 1'b0;

  reg [31:0] h_status = 32'd0, h_rx0 = 32'd0, h_tx0 = 32'd0;
  reg [31:0] h_ovf = 32'd0, h_unf = 32'd0, h_rxs0 = 32'd0, h_txs0 = 32'd0;
  reg [31:0] h_lvl = 32'd0, h_rx1 = 32'd0, h_tx1 = 32'd0, h_rxs1 = 32'd0;

  always @(posedge s_axi_aclk) begin
    tog_meta <= cap_toggle;
    tog_sync <= tog_meta;
    tog_prev <= tog_sync;
    if (tog_sync != tog_prev) begin
      h_status <= cap_status;
      h_rx0    <= cap_rx0;
      h_tx0    <= cap_tx0;
      h_ovf    <= cap_ovf;
      h_unf    <= cap_unf;
      h_rxs0   <= cap_rxs0;
      h_txs0   <= cap_txs0;
      h_lvl    <= cap_lvl;
      h_rx1    <= cap_rx1;
      h_tx1    <= cap_tx1;
      h_rxs1   <= cap_rxs1;
    end
  end

  assign v_status   = h_status;
  assign v_rx_cnt0  = h_rx0;
  assign v_tx_cnt0  = h_tx0;
  assign v_ovf_cnt  = h_ovf;
  assign v_unf_cnt  = h_unf;
  assign v_rx_snap0 = h_rxs0;
  assign v_tx_snap0 = h_txs0;
  assign v_level    = h_lvl;
  assign v_rx_cnt1  = h_rx1;
  assign v_tx_cnt1  = h_tx1;
  assign v_rx_snap1 = h_rxs1;

endmodule
