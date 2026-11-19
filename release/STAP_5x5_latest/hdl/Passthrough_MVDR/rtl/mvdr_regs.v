// -----------------------------------------------------------------------------
// mvdr_regs.v  --  MVDR control/status registers with clock-domain crossing
//
//   s_axi_aclk domain (100 MHz PS clock)  <-->  clk domain (AD9361 l_clk)
//
// Register map (byte offset inside this block's aperture; word address = offset/4)
//
//   0x00 ID          RO   0x4D564452 ("MVDR")
//   0x04 VERSION     RO   0x0001000F  (1.15: cov2x2 v6 self-healing window, SMI 2 x 16 taps;
//                                     1.14: SMI-MVDR STAP window at bytes 0x400..0xFFC;
//                                      1.1: adds COV_R12_* at 0x84..0x90,
//                                         1.2: adds the jammer estimator 0x94..0xAC, 0xC0..0xCC,
//                                         1.3: sliding covariance window + COV_CLIP at 0xB0,
//                                         1.4: OUT_GAIN at 0xD0,
//                                         1.5: maths on the 100 MHz AXI clock;
//                                              register map unchanged,
//                                         1.6: covariance v3 -- CTRL[4] dc_keep,
//                                              COV_PEAK at 0xB4, STATUS[8],
//                                         1.7: cov_prep v2 -- EPS_NF at 0xD4 (noise-referenced
//                                              loading, default x1), NF_EST at 0xB8/0xBC,
//                                              EPS_REL default 63 (off),
//                                         1.8: ctrl_fsm v2 -- SMOOTH at 0xD8 (jammer
//                                              detect, null hold, glide), STATUS[11:9],
//                                              COH_L2 in 0xBC[21:16], N_JAM 0xE0, N_HOLD 0xE4,
//                                         1.9: receiver IQ-image correction -- IQ_CTRL 0xDC,
//                                              IQ_B* 0xE8..0xF4, N_IQ_UPD 0xF8. Covariance
//                                              readbacks / EPS_ABS / NF_EST now in units of
//                                              (raw LSB x 4)^2 = 16 x the raw-sample units),
//                                         1.10: output_scaler v3 -- AOLC fields in OUT_GAIN
//                                              0xD0, OUT_LVL readback 0xFC,
//                                         1.11: jammer_est v2 -- second register page at
//                                              0x200 (MVDR byte offset 0x100): per-channel
//                                              jammer / noise power and jammer PEAK power,
//                                         1.12: timing closure at l_clk = 250 MHz (4.0 ns):
//                                              pipelined output_scaler v4, cov2x2_accum v5,
//                                              iq_corr input register, FIFO flags from flops.
//                                              Register map unchanged; output latency 18,
//                                         1.13: STAP -- space-time power inversion,
//                                              STAP_CTRL 0x10C, STAP_ST/WNORM/NWSAT
//                                              0x110..0x118 (page 2).)
//   0x08 CTRL        RW   [1:0] mode (0 bypass, 1 adaptive, 2 fixed, 3 = bypass)
//                         [2]   freeze (1 = hold current weights)
//                         [3]   cnt_clear (1 = hold all counters cleared)
//                         [4]   dc_keep (0 = DEFAULT: covariance estimated with the
//                               per-channel mean (DC) removed. Residual AD9361 DC otherwise
//                               steals the single degree of freedom when no jammer is
//                               present: -6.6 dB on a satellite in simulation.
//                               1 = raw covariance, only to null a jammer exactly at the LO.
//                               Polarity chosen so firmware that writes CTRL = mode keeps
//                               mean removal ON.)
//   0x0C COMMIT      W    write bit0 = 1: apply ALL config registers below atomically
//                    R    bit0 = 1 while a commit has not yet reached the datapath
//                              (a COMMIT written while bit0 = 1 is ignored: poll first)
//   0x10 A1          RW   [15:0] a1_re  [31:16] a1_im     steering vector element 1 (Q14)
//   0x14 A2          RW   [15:0] a2_re  [31:16] a2_im     steering vector element 2 (Q14)
//   0x18 EPS_ABS_LO  RW   extra absolute loading, window-sum units, bits [31:0] (default 0)
//   0x1C EPS_ABS_HI  RW   bits [ACC_W-1:32]
//   0x20 EPS_REL     RW   [5:0] extra loading = mean power >> n (DEFAULT 63 = off).
//                         Do NOT use as the main loading: it scales with the jammer and
//                         caps suppression at ~6n dB (n = 10: jammer leaks above the
//                         noise for JNR > 60 dB). Kept for experiments only.
//   0x24 FIXW1_RE    RW   signed 18-bit (Q15) weights used in mode 2
//   0x28 FIXW1_IM    RW
//   0x2C FIXW2_RE    RW
//   0x30 FIXW2_IM    RW
//   --- status (read-only; a coherent snapshot refreshed about every 2 us) ---
//   0x40 STATUS      RO   [1:0] mode in effect  [2] have_w  [3] last_sat  [4] last_skip
//                         [5] skew (sticky)  [6] weight calc busy  [7] commit pending
//                         [8] covariance in use is mean-removed (dc_rm in effect)
//                         [9] jammer detected in the latest window (coherence test)
//                         [10] HOLD: jammer gone, null kept   [11] GLIDE: easing back
//   0x44 N_BLOCKS    0x48 N_UPD    0x4C N_SKIP   0x50 N_SAT
//   0x54 N_DROP      0x58 N_ZERO   0x5C N_SKEW   0x60 N_OSAT
//   0x64 W1_RE  0x68 W1_IM  0x6C W2_RE  0x70 W2_IM      current weights (sign-extended Q15)
//   0x74 COV_R11_LO 0x78 COV_R11_HI 0x7C COV_R22_LO 0x80 COV_R22_HI   latest raw block sums
//                    (measure with the jammer OFF, then set EPS_ABS to about 1x..10x of that)
//   0x84 COV_R12_RE_LO 0x88 COV_R12_RE_HI 0x8C COV_R12_IM_LO 0x90 COV_R12_IM_HI
//                    latest raw cross-covariance sum x1*conj(x2), same block and same
//                    snapshot as R11/R22 (sign-extended 64-bit pairs)
//   --- jammer estimator status (jammer_est.v; same coherent snapshot) ---
//   0x94 JAM_STATUS  RO   [0] jammer present  [1] estimate valid  [2] sin(theta) clipped
//                         [3] no signal (R = 0)  [4] noise estimate 0 (JNR off scale)
//   0x98 JAM_ANGLE   RO   [15:0] theta, 0.01 deg, signed (0 = broadside, + = towards RX2)
//                         [31:16] psi = calibrated RX1/RX2 phase, 0.01 deg, signed
//   0x9C JAM_PHRAW   RO   [15:0] uncalibrated phase, binary angle (32768 = 180 deg)
//                         [31:16] the same in 0.01 deg
//   0xA0 JAM_POWER   RO   [15:0] jammer power per element, dBFS, Q8 (1/256 dB) signed
//                         [31:16] noise floor per element, dBFS, Q8 signed
//                         (0x8000 = no energy)
//   0xA4 JAM_JNR     RO   [15:0] JNR dB, Q8 signed (0x7FFF = above range)
//   0xA8 JAM_N_EST   RO   estimates computed
//   0xAC JAM_N_DROP  RO   averaging windows dropped (estimator busy; should stay 0)
//   --- covariance input quality (cov2x2_accum v2; same coherent snapshot) ---
//   0xB0 COV_CLIP    RO   [15:0] RX1 samples on an ADC rail in the current covariance window
//                         (2^LOG2_N samples: 8192 in the LOG2_N=13 build)
//                         [31:16] RX2 samples on an ADC rail in the same window
//                         (non-zero: the ADC is clipping -> reduce RX gain;
//                          a clipped jammer cannot be nulled)
//   0xB4 COV_PEAK    RO   [11:0] RX1 max(|I|,|Q|) in the window, [27:16] RX2 the same.
//                         Headroom to ADC full scale = 20*log10(2048/peak) dB: keep it
//                         >= ~3 dB with the jammer on, BEFORE COV_CLIP starts counting.
//   0xB8 NF_EST_LO   RO   noise-floor estimate det(R)/trace(R), window-sum units, [31:0]
//   0xBC NF_EST_HI   RO   [5:0] upper bits. (= noise eigenvalue with a jammer, half of it
//                         without; per-sample noise power per element ~ NF_EST / 8192)
//                         [21:16] COH_L2 ~ -log2(RX1/RX2 coherence) of the latest window,
//                         +-1 (63 = none). Noise only reads ~13; a jammer reads 0..8.
//                         Calibrate: with NO jammer, COH_SH must stay below this value.
//   --- jammer estimator configuration (RW, applied by COMMIT like the others) ---
//   0xC0 JAM_CTRL    RW   [0] enable  [1] phase invert  [11:8] averaging, log2 of blocks (0..8)
//   0xC4 JAM_CAL     RW   [15:0] phase calibration, binary angle (32768 = 180 deg):
//                         write JAM_PHRAW[15:0] measured with a source at broadside
//   0xC8 JAM_K       RW   [15:0] lambda/(2d) in Q14  (94.6 mm @ 1584.5 MHz -> 16384)
//   0xCC JAM_THR     RW   [15:0] JNR detect threshold, dB Q8  [31:16] JNR release threshold
//   --- output level (RW, applied by COMMIT) ---
//   0xD4 EPS_NF      RW   [7:0] k: diagonal loading = k/8 x noise floor (NF_EST).
//                         DEFAULT 8 (x1, 0 dB above noise). 80 = x10 (+10 dB, more robust,
//                         slightly shallower nulls on weak jammers). 0 = off.
//                         Loading tracks the noise, not the jammer, so the null depth no
//                         longer collapses at high JNR (residual ~40 dB under the noise).
//   0xD8 SMOOTH      RW   weight policy (adaptive mode), for carrier-phase continuity:
//                         [15:0]  HOLD: after the jammer disappears keep the null for
//                                 HOLD x 64 hops (x 1.19 ms @ 55 MSPS). DEFAULT 252 (0.3 s).
//                         [19:16] REL_SH: then glide back with time constant 2^REL_SH hops.
//                                 DEFAULT 14 (0.3 s).
//                         [27:24] COH_SH: jammer present if coherence > 2^-COH_SH.
//                                 DEFAULT 8 (detects JNR >~ -12 dB, no false alarms).
//                         HOLD = 0 and REL_SH = 0: v1 behaviour (every weight set applied).
//   0xDC IQ_CTRL     RW   receiver IQ-image correction (null depth above ~45 dB JNR):
//                         [0] IQ_OFF (0 = DEFAULT, correction on)  [1] clear (b = 0)
//                         [6:4] MU_SH refinement step 2^-n, DEFAULT 2. The first estimate
//                         after a clear/reset is a full step; later ones average.
//                         Estimates update only while a jammer is detected.
//   0xE8 IQ_B1_RE    RO   RX1 image coefficient in use, Q20 sign-extended (image = 20log10|b|)
//   0xEC IQ_B1_IM    RO   0xF0 IQ_B2_RE   0xF4 IQ_B2_IM   (RX2)
//   0xF8 N_IQ_UPD    RO   image-coefficient updates
//   0xE0 N_JAM       RO   weight runs with the jammer flag set
//   0xE4 N_HOLD      RO   weight runs absorbed by a hold
//   --- page 2 (MVDR window offset 0x100.., AXI 0x200..) : jammer power, jammer_est v2 ---
//   0x100 JAM_PWR_CH  RO  [15:0] jammer power RX1, [31:16] RX2: dBFS Q8, AVERAGE over
//                         the averaging period (= RMS^2 power; for CW = its actual power)
//   0x104 JAM_PEAK    RO  [15:0] RX1, [31:16] RX2: dBFS Q8, PEAK = strongest 149 us window
//                         in the period (= ON power of a pulsed / intermittent jammer)
//   0x108 NOISE_CH    RO  [15:0] noise floor RX1, [31:16] RX2: dBFS Q8
//                         All: 0x8000 = no energy. dBm = dBFS + full-scale dBm at the
//                         current RX gain (firmware). Instantaneous ADC peak: COV_PEAK 0xB4.
//                         The page-1 JAM_POWER / JAM_JNR fields are now gain-imbalance
//                         proof (coherence method) and equal the geometric mean RX1/RX2.
//   --- page 2, block 1.13: STAP (space-time power inversion, stap_lms) ---
//   0x10C STAP_CTRL   RW  (applied by COMMIT, like every other setting)
//                         [0]     STAP_EN: 1 = the output comes from STAP (2 antennas x 5
//                                 taps, NLMS) instead of the single-weight beamformer.
//                                 DEFAULT 0 (= the 1.12 behaviour). Robust against CW AND
//                                 wideband / barrage jammers; MODE/A1/A2/fixed weights do
//                                 not apply while it is on (the covariance, NF_EST, jammer
//                                 estimator keep running for the reports).
//                         [7:4]   MU_SH: step 2^-n, 5..15 (DEFAULT 5; < 5 acts as 5: stability;
//                                 6..8 = slower, slightly deeper null)
//                         [12:8]  LEAK_SH: weight leakage 2^-n per sample (DEFAULT 18, 0 = off)
//                         [16]    CLEAR: weights held at 0 while set
//                         [17]    HOLD_OFF: 1 = no normalisation hold. DEFAULT 0 = hold on:
//                                 the null survives the OFF time of a pulsed jammer
//                                 (~14 ms for a 35 dB jammer at 56 MSPS)
//   0x110 STAP_ST     RO  [5:0] PL: normalisation exponent, input power ~ 2^(PL-2) in
//                                 (raw LSB x 4)^2 units  [8] STAP active (applied in clk)
//                                 [31] 1 = NLMS STAP NOT built (v1.14 default; SMI only)
//   0x114 STAP_WNORM  RO  sum |w|^2 over the 9 auxiliary taps, Q8.24 (unsigned).
//                         Output noise = noise x (1 + WNORM/2^24): the noise cost of the null.
//   0x118 STAP_NWSAT  RO  weight saturation events (should stay 0)
//   0xFC OUT_LVL     RO   [15:0] measured output power per sample, log2, Q4 (signed):
//                         output level dBFS = 3.0103 * (OUT_LVL[15:0]/16 - 23)
//                         [31:16] output gain in use, Q8 octaves (signed): dB = 6.0206 * x/256
//   0xD0 OUT_GAIN    RW   [2:0] MVDR output gain = 2^n (0 = unity .. 7 = +42 dB, 6.02 dB/step).
//                         [8]  AOLC_EN: automatic output level control (DEFAULT 0 = off).
//                              Holds the output noise floor at TARGET dBFS with a smooth
//                              0.02 dB-step gain (immune to the short onset leak). Removes the
//                              0.5..4.8 dB C/N0 loss of a low output level in the 12-bit output
//                              and DAC/TX chain. RAISES THE TX1 LEVEL (up to ~45 dB at low RX
//                              gain): set the AD9361 TX attenuation first.
//                         [15:12] AGC_SH loop speed (DEFAULT 8: time constant ~19 ms)
//                         [23:16] TARGET: output noise floor in dB below full scale (DEFAULT 18)
//                         Use it to restore the TX1 level after lowering the RX gain to
//                         stop ADC clipping. Watch N_OSAT: it counts clipped output samples.
//                         Applies to the MVDR path only (CONTROL[4] = 1), not raw RX1.
//
// Reset defaults: mode 0 (bypass), freeze 0, dc_keep 0 (mean removal on), a = [1,0],
//                 eps_nf 8 (loading = noise floor), eps_rel 63 (off), eps_abs 0,
//                 fixed weights [1,0].
//                 Jammer: enabled, no invert, 16-block averaging, cal 0, K 16384,
//                 detect 6 dB, release 4 dB.
//
// CLOCKING (v1.5)
//   The MVDR maths (cov_prep, weight calc, control FSM, jammer_est) now runs
//   on s_axi_aclk, so most configuration is consumed in the AXI domain
//   straight from the committed snapshot (acfg_* outputs = sn_*), and the
//   status capture runs on st_clk = s_axi_aclk. Only out_gain and cnt_clear
//   still cross to the sample clock (cfg_out_gain / cfg_cnt_clear); the other
//   cfg_* outputs remain for compatibility and are unused (trimmed).
//   The coherent-snapshot logic below is unchanged: with st_clk = s_axi_aclk
//   the "crossing" is simply a same-clock copy.
//
// CDC method
//   config  (aclk -> clk): toggle handshake.  COMMIT copies the shadow registers
//     into a snapshot and flips cfg_tog; the datapath domain sees the toggle
//     through a 2-flop synchroniser, copies the snapshot, and flips cfg_ack
//     back.  The snapshot cannot change until the ack has returned, so the copy
//     is always coherent, and a commit made while the datapath clock is stopped
//     is simply applied when the clock starts.  The datapath-side config is NOT
//     reset by the datapath `rst`, so a reset of the radio interface does not
//     silently drop the software configuration.
//   status  (clk -> aclk): every 256 clk cycles all status words are captured
//     at one instant and a toggle flips; the AXI side copies the whole set after
//     seeing it (snapshot is stable for 256 clocks, far longer than the sync).
//
// The AXI-Lite write/read decoding stays in the parent: it passes a one-clock
// write strobe with word address/data, and a combinational read address.
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module mvdr_regs #(
    parameter ACC_W = 38,               // must match mvdr_top ACC_W (<= 64); 38 for LOG2_N=13
    parameter W_W   = 18,
    parameter A_W   = 16,
    parameter HAVE_STAP = 1             // 0: stap_lms not built -> STAP_ST[31] = 1 (absent)
)(
    // ---------------- AXI-Lite clock domain ----------------
    input  wire                     s_axi_aclk,
    input  wire                     s_axi_aresetn,
    input  wire                     wr_en,          // 1-clock write strobe
    input  wire [7:0]               wr_addr,        // word address (byte offset / 4)
    input  wire [31:0]              wr_data,
    input  wire [7:0]               rd_addr,        // word address, combinational read
    output reg  [31:0]              rd_data,

    // ---------------- datapath clock domain ----------------
    input  wire                     clk,
    input  wire                     st_clk,         // clock of every status input (v1.5: s_axi_aclk)
    output reg  [1:0]               cfg_mode,
    output reg                      cfg_freeze,
    output reg                      cfg_cnt_clear,
    output reg  signed [A_W-1:0]    cfg_a1_re,
    output reg  signed [A_W-1:0]    cfg_a1_im,
    output reg  signed [A_W-1:0]    cfg_a2_re,
    output reg  signed [A_W-1:0]    cfg_a2_im,
    output reg         [ACC_W-1:0]  cfg_eps_abs,
    output reg         [5:0]        cfg_eps_rel_sh,
    output reg  signed [W_W-1:0]    cfg_fix_w1_re,
    output reg  signed [W_W-1:0]    cfg_fix_w1_im,
    output reg  signed [W_W-1:0]    cfg_fix_w2_re,
    output reg  signed [W_W-1:0]    cfg_fix_w2_im,
    output reg                      cfg_jam_en,
    output reg                      cfg_jam_inv,
    output reg         [3:0]        cfg_jam_avg,
    output reg  signed [15:0]       cfg_jam_cal,
    output reg         [15:0]       cfg_jam_k,
    output reg  signed [15:0]       cfg_jam_on,
    output reg  signed [15:0]       cfg_jam_off,
    output reg         [2:0]        cfg_out_gain,
    output reg                      cfg_agc_en,
    output reg         [3:0]        cfg_agc_sh,
    output reg         [7:0]        cfg_tdb,
    output reg                      cfg_dc_rm,
    output reg                      cfg_stap_en,
    output reg         [3:0]        cfg_stap_mu,
    output reg         [4:0]        cfg_stap_leak,
    output reg                      cfg_stap_clr,
    output reg                      cfg_stap_hoff,

    // committed configuration in the AXI domain (v1.5, for the maths domain)
    output wire [1:0]               acfg_mode,
    output wire                     acfg_freeze,
    output wire                     acfg_cnt_clear,
    output wire signed [A_W-1:0]    acfg_a1_re,
    output wire signed [A_W-1:0]    acfg_a1_im,
    output wire signed [A_W-1:0]    acfg_a2_re,
    output wire signed [A_W-1:0]    acfg_a2_im,
    output wire        [ACC_W-1:0]  acfg_eps_abs,
    output wire        [5:0]        acfg_eps_rel_sh,
    output wire        [7:0]        acfg_eps_nf_k,
    output wire        [15:0]       acfg_hold,
    output wire        [3:0]        acfg_rel_sh,
    output wire        [3:0]        acfg_coh_sh,
    output wire                     acfg_iq_en,
    output wire                     acfg_iq_clear,
    output wire        [2:0]        acfg_iq_mu,
    output wire signed [W_W-1:0]    acfg_fix_w1_re,
    output wire signed [W_W-1:0]    acfg_fix_w1_im,
    output wire signed [W_W-1:0]    acfg_fix_w2_re,
    output wire signed [W_W-1:0]    acfg_fix_w2_im,
    output wire                     acfg_jam_en,
    output wire                     acfg_jam_inv,
    output wire        [3:0]        acfg_jam_avg,
    output wire signed [15:0]       acfg_jam_cal,
    output wire        [15:0]       acfg_jam_k,
    output wire signed [15:0]       acfg_jam_on,
    output wire signed [15:0]       acfg_jam_off,

    // status (st_clk domain)
    input  wire [1:0]               st_mode,
    input  wire                     st_have_w,
    input  wire                     st_last_sat,
    input  wire                     st_last_skip,
    input  wire                     st_skew,
    input  wire                     st_wc_busy,
    input  wire [31:0]              n_blocks,
    input  wire [31:0]              n_upd,
    input  wire [31:0]              n_skip,
    input  wire [31:0]              n_sat,
    input  wire [31:0]              n_drop,
    input  wire [31:0]              n_zero,
    input  wire [31:0]              n_skew,
    input  wire [31:0]              n_osat,
    input  wire signed [W_W-1:0]    w_cur_w1_re,
    input  wire signed [W_W-1:0]    w_cur_w1_im,
    input  wire signed [W_W-1:0]    w_cur_w2_re,
    input  wire signed [W_W-1:0]    w_cur_w2_im,
    input  wire signed [ACC_W-1:0]  cov_r11,
    input  wire signed [ACC_W-1:0]  cov_r22,
    input  wire signed [ACC_W-1:0]  cov_r12_re,
    input  wire signed [ACC_W-1:0]  cov_r12_im,
    // jammer estimator results (st_clk domain)
    input  wire [4:0]               jam_flags,      // {noise0, nosig, clip, valid, present}
    input  wire signed [15:0]       jam_theta,
    input  wire signed [15:0]       jam_psi,
    input  wire signed [15:0]       jam_psi_raw_bin,
    input  wire signed [15:0]       jam_psi_raw_cd,
    input  wire signed [15:0]       jam_dbfs,
    input  wire signed [15:0]       jam_noise_dbfs,
    input  wire signed [15:0]       jam_jnr,
    input  wire [31:0]              jam_n_est,
    input  wire [31:0]              jam_n_drop,
    // covariance input quality (st_clk domain)
    input  wire [15:0]              cov_clip1,
    input  wire [15:0]              cov_clip2,
    input  wire [15:0]              cov_peak1,
    input  wire [15:0]              cov_peak2,
    input  wire                     cov_dc_rm,
    input  wire        [ACC_W-1:0]  nf_est,         // st_clk domain
    input  wire        [5:0]        coh_l2,
    input  wire                     st_jam,
    input  wire                     st_hold,
    input  wire                     st_glide,
    input  wire [31:0]              n_jam,
    input  wire [31:0]              n_hold,
    input  wire signed [17:0]       iq_b1_re,
    input  wire signed [17:0]       iq_b1_im,
    input  wire signed [17:0]       iq_b2_re,
    input  wire signed [17:0]       iq_b2_im,
    input  wire [31:0]              n_iq_upd,
    input  wire [31:0]              out_lvl,
    input  wire signed [15:0]       jam_j1,
    input  wire signed [15:0]       jam_j2,
    input  wire signed [15:0]       jam_n1,
    input  wire signed [15:0]       jam_n2,
    input  wire signed [15:0]       jam_pk1,
    input  wire signed [15:0]       jam_pk2,
    input  wire                     stap_active,    // st_clk domain
    input  wire        [5:0]        stap_pl,
    input  wire        [31:0]       stap_wnorm,
    input  wire        [31:0]       stap_nwsat
);

    localparam [31:0] REG_ID      = 32'h4D564452;
    localparam [31:0] REG_VERSION = 32'h0001000F;     // 1.15: 1.14b SMI 16 taps + self-healing 2x2 window
                                                      // 1.13: STAP (0x10C..0x118, page 2)
    localparam integer NXS = 14;                        // extra status words (0xE0..0xFC, page 2)
    localparam [15:0] HOLD_DEF = 16'd252;               // 252 x 64 hops = 0.3 s @ 55 MSPS
    localparam [3:0]  RELS_DEF = 4'd14;                 // glide tau 2^14 hops = 0.3 s
    localparam [3:0]  COHS_DEF = 4'd8;                  // coherence > 1/256
    localparam integer NSW = 32;                        // number of status words (0x40..0xBC)
    localparam [7:0]  NFK_DEF = 8'd8;                   // loading = 1.0 x noise floor
    localparam [5:0]  REL_DEF = 6'd63;                  // relative loading off

    // jammer estimator config defaults
    localparam [3:0]  JAM_AVG_DEF = 4'd4;               // 16 blocks
    localparam [15:0] JAM_K_DEF   = 16'd16384;          // lambda/(2d) = 1.0 in Q14
    localparam [15:0] JAM_ON_DEF  = 16'd1536;           // 6 dB in Q8
    localparam [15:0] JAM_OFF_DEF = 16'd1024;           // 4 dB in Q8
    localparam [17:0] STAP_DEF    = 18'h09280;          // off, mu 5, leak 18, hold on

    localparam signed [A_W-1:0] A_ONE  = 16'sh4000;     // 1.0 in Q14
    localparam signed [W_W-1:0] W_ONE  = 32768;         // 1.0 in Q15

    // ==========================================================================
    //  AXI domain: shadow registers
    // ==========================================================================
    reg  [1:0]              sh_mode   = 2'd0;
    reg                     sh_freeze = 1'b0;
    reg                     sh_clr    = 1'b0;
    reg                     sh_dcrm   = 1'b0;     // = CTRL[4] dc_keep
    reg  [A_W-1:0]          sh_a1r    = A_ONE;
    reg  [A_W-1:0]          sh_a1i    = {A_W{1'b0}};
    reg  [A_W-1:0]          sh_a2r    = {A_W{1'b0}};
    reg  [A_W-1:0]          sh_a2i    = {A_W{1'b0}};
    reg  [ACC_W-1:0]        sh_eps    = {ACC_W{1'b0}};
    reg  [5:0]              sh_rel    = REL_DEF;
    reg  [7:0]              sh_nfk    = NFK_DEF;
    reg  [15:0]             sh_hold   = HOLD_DEF;
    reg  [3:0]              sh_rels   = RELS_DEF;
    reg  [3:0]              sh_cohs   = COHS_DEF;
    reg  [6:0]              sh_iq     = 7'h20;          // [0] off [1] clear [6:4] mu = 2
    reg  [W_W-1:0]          sh_f1r    = W_ONE;
    reg  [W_W-1:0]          sh_f1i    = {W_W{1'b0}};
    reg  [W_W-1:0]          sh_f2r    = {W_W{1'b0}};
    reg  [W_W-1:0]          sh_f2i    = {W_W{1'b0}};
    reg                     sh_jen    = 1'b1;
    reg                     sh_jinv   = 1'b0;
    reg  [3:0]              sh_javg   = JAM_AVG_DEF;
    reg  [15:0]             sh_jcal   = 16'd0;
    reg  [15:0]             sh_jk     = JAM_K_DEF;
    reg  [15:0]             sh_jon    = JAM_ON_DEF;
    reg  [15:0]             sh_joff   = JAM_OFF_DEF;
    reg  [2:0]              sh_ogain  = 3'd0;
    reg                     sh_agc    = 1'b0;
    reg  [3:0]              sh_agcsh  = 4'd8;
    reg  [7:0]              sh_tdb    = 8'd18;
    reg  [17:0]             sh_stap   = STAP_DEF;

    // snapshot (stable from a commit until the datapath acknowledges it)
    reg  [1:0]              sn_mode   = 2'd0;
    reg                     sn_freeze = 1'b0;
    reg                     sn_clr    = 1'b0;
    reg                     sn_dcrm   = 1'b0;
    reg  [A_W-1:0]          sn_a1r    = A_ONE;
    reg  [A_W-1:0]          sn_a1i    = {A_W{1'b0}};
    reg  [A_W-1:0]          sn_a2r    = {A_W{1'b0}};
    reg  [A_W-1:0]          sn_a2i    = {A_W{1'b0}};
    reg  [ACC_W-1:0]        sn_eps    = {ACC_W{1'b0}};
    reg  [5:0]              sn_rel    = REL_DEF;
    reg  [7:0]              sn_nfk    = NFK_DEF;
    reg  [15:0]             sn_hold   = HOLD_DEF;
    reg  [3:0]              sn_rels   = RELS_DEF;
    reg  [3:0]              sn_cohs   = COHS_DEF;
    reg  [6:0]              sn_iq     = 7'h20;
    reg  [W_W-1:0]          sn_f1r    = W_ONE;
    reg  [W_W-1:0]          sn_f1i    = {W_W{1'b0}};
    reg  [W_W-1:0]          sn_f2r    = {W_W{1'b0}};
    reg  [W_W-1:0]          sn_f2i    = {W_W{1'b0}};
    reg                     sn_jen    = 1'b1;
    reg                     sn_jinv   = 1'b0;
    reg  [3:0]              sn_javg   = JAM_AVG_DEF;
    reg  [15:0]             sn_jcal   = 16'd0;
    reg  [15:0]             sn_jk     = JAM_K_DEF;
    reg  [15:0]             sn_jon    = JAM_ON_DEF;
    reg  [15:0]             sn_joff   = JAM_OFF_DEF;
    reg  [2:0]              sn_ogain  = 3'd0;
    reg                     sn_agc    = 1'b0;
    reg  [3:0]              sn_agcsh  = 4'd8;
    reg  [7:0]              sn_tdb    = 8'd18;
    reg  [17:0]             sn_stap   = STAP_DEF;

    // committed config, AXI domain (the maths domain uses these directly)
    assign acfg_mode       = sn_mode;
    assign acfg_freeze     = sn_freeze;
    assign acfg_cnt_clear  = sn_clr;
    assign acfg_a1_re      = sn_a1r;
    assign acfg_a1_im      = sn_a1i;
    assign acfg_a2_re      = sn_a2r;
    assign acfg_a2_im      = sn_a2i;
    assign acfg_eps_abs    = sn_eps;
    assign acfg_eps_rel_sh = sn_rel;
    assign acfg_eps_nf_k   = sn_nfk;
    assign acfg_hold       = sn_hold;
    assign acfg_rel_sh     = sn_rels;
    assign acfg_coh_sh     = sn_cohs;
    assign acfg_iq_en      = ~sn_iq[0];
    assign acfg_iq_clear   = sn_iq[1];
    assign acfg_iq_mu      = sn_iq[6:4];
    assign acfg_fix_w1_re  = sn_f1r;
    assign acfg_fix_w1_im  = sn_f1i;
    assign acfg_fix_w2_re  = sn_f2r;
    assign acfg_fix_w2_im  = sn_f2i;
    assign acfg_jam_en     = sn_jen;
    assign acfg_jam_inv    = sn_jinv;
    assign acfg_jam_avg    = sn_javg;
    assign acfg_jam_cal    = sn_jcal;
    assign acfg_jam_k      = sn_jk;
    assign acfg_jam_on     = sn_jon;
    assign acfg_jam_off    = sn_joff;

    reg                     cfg_tog   = 1'b0;           // aclk domain
    reg                     cfg_ack   = 1'b0;           // clk domain
    (* ASYNC_REG = "TRUE" *) reg ack_m = 1'b0;
    (* ASYNC_REG = "TRUE" *) reg ack_s = 1'b0;
    wire                    pending   = (cfg_tog != ack_s);

    always @(posedge s_axi_aclk) begin
        ack_m <= cfg_ack;
        ack_s <= ack_m;

        if (!s_axi_aresetn) begin
            sh_mode <= 2'd0;  sh_freeze <= 1'b0;  sh_clr <= 1'b0;  sh_dcrm <= 1'b0;
            sh_a1r <= A_ONE;  sh_a1i <= {A_W{1'b0}};
            sh_a2r <= {A_W{1'b0}};  sh_a2i <= {A_W{1'b0}};
            sh_eps <= {ACC_W{1'b0}};  sh_rel <= REL_DEF;  sh_nfk <= NFK_DEF;
            sh_hold <= HOLD_DEF;  sh_rels <= RELS_DEF;  sh_cohs <= COHS_DEF;  sh_iq <= 7'h20;
            sh_f1r <= W_ONE;  sh_f1i <= {W_W{1'b0}};
            sh_f2r <= {W_W{1'b0}};  sh_f2i <= {W_W{1'b0}};
            sh_jen <= 1'b1;  sh_jinv <= 1'b0;  sh_javg <= JAM_AVG_DEF;
            sh_jcal <= 16'd0;  sh_jk <= JAM_K_DEF;
            sh_jon <= JAM_ON_DEF;  sh_joff <= JAM_OFF_DEF;
            sh_ogain <= 3'd0;  sh_agc <= 1'b0;  sh_agcsh <= 4'd8;  sh_tdb <= 8'd18;
            sh_stap <= STAP_DEF;
        end else if (wr_en) begin
            case (wr_addr)
                8'd2:  begin sh_mode <= wr_data[1:0]; sh_freeze <= wr_data[2]; sh_clr <= wr_data[3];
                             sh_dcrm <= wr_data[4]; end
                8'd3:  begin
                    if (wr_data[0] && !pending) begin
                        sn_mode <= sh_mode;  sn_freeze <= sh_freeze;  sn_clr <= sh_clr;
                        sn_dcrm <= sh_dcrm;
                        sn_a1r <= sh_a1r;  sn_a1i <= sh_a1i;  sn_a2r <= sh_a2r;  sn_a2i <= sh_a2i;
                        sn_eps <= sh_eps;  sn_rel <= sh_rel;  sn_nfk <= sh_nfk;
                        sn_hold <= sh_hold;  sn_rels <= sh_rels;  sn_cohs <= sh_cohs;
                        sn_iq   <= sh_iq;
                        sn_f1r <= sh_f1r;  sn_f1i <= sh_f1i;  sn_f2r <= sh_f2r;  sn_f2i <= sh_f2i;
                        sn_jen <= sh_jen;  sn_jinv <= sh_jinv;  sn_javg <= sh_javg;
                        sn_jcal <= sh_jcal;  sn_jk <= sh_jk;
                        sn_jon <= sh_jon;  sn_joff <= sh_joff;
                        sn_ogain <= sh_ogain;
                        sn_agc <= sh_agc;  sn_agcsh <= sh_agcsh;  sn_tdb <= sh_tdb;
                        sn_stap <= sh_stap;
                        cfg_tog <= ~cfg_tog;
                    end
                end
                8'd4:  begin sh_a1r <= wr_data[15:0]; sh_a1i <= wr_data[31:16]; end
                8'd5:  begin sh_a2r <= wr_data[15:0]; sh_a2i <= wr_data[31:16]; end
                8'd6:  sh_eps[31:0] <= wr_data;
                8'd7:  sh_eps[ACC_W-1:32] <= wr_data[ACC_W-33:0];
                8'd8:  sh_rel <= wr_data[5:0];
                8'd9:  sh_f1r <= wr_data[W_W-1:0];
                8'd10: sh_f1i <= wr_data[W_W-1:0];
                8'd11: sh_f2r <= wr_data[W_W-1:0];
                8'd12: sh_f2i <= wr_data[W_W-1:0];
                8'd48: begin sh_jen <= wr_data[0]; sh_jinv <= wr_data[1]; sh_javg <= wr_data[11:8]; end
                8'd49: sh_jcal <= wr_data[15:0];
                8'd50: sh_jk   <= wr_data[15:0];
                8'd51: begin sh_jon <= wr_data[15:0]; sh_joff <= wr_data[31:16]; end
                8'd52: begin sh_ogain <= wr_data[2:0];  sh_agc <= wr_data[8];
                             sh_agcsh <= wr_data[15:12];  sh_tdb <= wr_data[23:16]; end
                8'd53: sh_nfk   <= wr_data[7:0];
                8'd54: begin sh_hold <= wr_data[15:0]; sh_rels <= wr_data[19:16];
                             sh_cohs <= wr_data[27:24]; end
                8'd55: sh_iq <= {wr_data[6:4], 2'b00, wr_data[1:0]};
                8'd67: sh_stap <= {wr_data[17], wr_data[16], wr_data[12:8], wr_data[7:4], 6'd0, wr_data[0]};
                default: ;
            endcase
        end
    end

    // ==========================================================================
    //  clk domain: apply the snapshot
    // ==========================================================================
    (* ASYNC_REG = "TRUE" *) reg tog_m = 1'b0;
    (* ASYNC_REG = "TRUE" *) reg tog_s = 1'b0;

    initial begin
        cfg_mode = 2'd0;  cfg_freeze = 1'b0;  cfg_cnt_clear = 1'b0;
        cfg_a1_re = A_ONE;  cfg_a1_im = 0;  cfg_a2_re = 0;  cfg_a2_im = 0;
        cfg_eps_abs = 0;  cfg_eps_rel_sh = REL_DEF;
        cfg_fix_w1_re = W_ONE;  cfg_fix_w1_im = 0;  cfg_fix_w2_re = 0;  cfg_fix_w2_im = 0;
        cfg_jam_en = 1'b1;  cfg_jam_inv = 1'b0;  cfg_jam_avg = JAM_AVG_DEF;
        cfg_jam_cal = 0;  cfg_jam_k = JAM_K_DEF;
        cfg_jam_on = JAM_ON_DEF;  cfg_jam_off = JAM_OFF_DEF;
        cfg_out_gain = 3'd0;  cfg_agc_en = 1'b0;  cfg_agc_sh = 4'd8;  cfg_tdb = 8'd18;
        cfg_dc_rm = 1'b1;
        cfg_stap_en = 1'b0;  cfg_stap_mu = STAP_DEF[10:7];
        cfg_stap_leak = STAP_DEF[15:11];  cfg_stap_clr = 1'b0;  cfg_stap_hoff = 1'b0;
    end

    always @(posedge clk) begin
        tog_m <= cfg_tog;
        tog_s <= tog_m;
        if (tog_s != cfg_ack) begin
            cfg_mode       <= sn_mode;
            cfg_freeze     <= sn_freeze;
            cfg_cnt_clear  <= sn_clr;
            cfg_a1_re      <= sn_a1r;   cfg_a1_im <= sn_a1i;
            cfg_a2_re      <= sn_a2r;   cfg_a2_im <= sn_a2i;
            cfg_eps_abs    <= sn_eps;
            cfg_eps_rel_sh <= sn_rel;
            cfg_fix_w1_re  <= sn_f1r;   cfg_fix_w1_im <= sn_f1i;
            cfg_fix_w2_re  <= sn_f2r;   cfg_fix_w2_im <= sn_f2i;
            cfg_jam_en     <= sn_jen;   cfg_jam_inv   <= sn_jinv;
            cfg_jam_avg    <= sn_javg;  cfg_jam_cal   <= sn_jcal;
            cfg_jam_k      <= sn_jk;
            cfg_jam_on     <= sn_jon;   cfg_jam_off   <= sn_joff;
            cfg_out_gain   <= sn_ogain;
            cfg_agc_en     <= sn_agc;
            cfg_agc_sh     <= sn_agcsh;
            cfg_tdb        <= sn_tdb;
            cfg_dc_rm      <= ~sn_dcrm;          // dc_keep = 0 -> mean removal on
            cfg_stap_en    <= sn_stap[0];
            cfg_stap_mu    <= sn_stap[10:7];
            cfg_stap_leak  <= sn_stap[15:11];
            cfg_stap_clr   <= sn_stap[16];
            cfg_stap_hoff  <= sn_stap[17];
            cfg_ack        <= tog_s;
        end
    end

    // ==========================================================================
    //  status (st_clk) -> AXI domain: coherent status snapshot
    // ==========================================================================
    reg  [7:0]                cap_div = 8'd0;
    reg                       cap_tog = 1'b0;
    reg  [32*(NSW+NXS)-1:0]   cap_vec = {(32*(NSW+NXS)){1'b0}};

    wire [63:0] r11_x = {{(64-ACC_W){cov_r11[ACC_W-1]}}, cov_r11};
    wire [63:0] r22_x = {{(64-ACC_W){cov_r22[ACC_W-1]}}, cov_r22};
    wire [63:0] r12r_x = {{(64-ACC_W){cov_r12_re[ACC_W-1]}}, cov_r12_re};
    wire [63:0] r12i_x = {{(64-ACC_W){cov_r12_im[ACC_W-1]}}, cov_r12_im};
    wire [63:0] nf_x   = {{(64-ACC_W){1'b0}}, nf_est};

    always @(posedge st_clk) begin
        cap_div <= cap_div + 8'd1;
        if (cap_div == 8'd0) begin
            cap_vec[32* 0 +: 32] <= {20'd0, st_glide, st_hold, st_jam, cov_dc_rm, 1'b0,
                                     st_wc_busy, st_skew, st_last_skip,
                                     st_last_sat, st_have_w, st_mode};
            cap_vec[32* 1 +: 32] <= n_blocks;
            cap_vec[32* 2 +: 32] <= n_upd;
            cap_vec[32* 3 +: 32] <= n_skip;
            cap_vec[32* 4 +: 32] <= n_sat;
            cap_vec[32* 5 +: 32] <= n_drop;
            cap_vec[32* 6 +: 32] <= n_zero;
            cap_vec[32* 7 +: 32] <= n_skew;
            cap_vec[32* 8 +: 32] <= n_osat;
            cap_vec[32* 9 +: 32] <= {{(32-W_W){w_cur_w1_re[W_W-1]}}, w_cur_w1_re};
            cap_vec[32*10 +: 32] <= {{(32-W_W){w_cur_w1_im[W_W-1]}}, w_cur_w1_im};
            cap_vec[32*11 +: 32] <= {{(32-W_W){w_cur_w2_re[W_W-1]}}, w_cur_w2_re};
            cap_vec[32*12 +: 32] <= {{(32-W_W){w_cur_w2_im[W_W-1]}}, w_cur_w2_im};
            cap_vec[32*13 +: 32] <= r11_x[31:0];
            cap_vec[32*14 +: 32] <= r11_x[63:32];
            cap_vec[32*15 +: 32] <= r22_x[31:0];
            cap_vec[32*16 +: 32] <= r22_x[63:32];
            cap_vec[32*17 +: 32] <= r12r_x[31:0];     // 0x84
            cap_vec[32*18 +: 32] <= r12r_x[63:32];    // 0x88
            cap_vec[32*19 +: 32] <= r12i_x[31:0];     // 0x8C
            cap_vec[32*20 +: 32] <= r12i_x[63:32];    // 0x90
            cap_vec[32*21 +: 32] <= {27'd0, jam_flags};                    // 0x94
            cap_vec[32*22 +: 32] <= {jam_psi, jam_theta};                  // 0x98
            cap_vec[32*23 +: 32] <= {jam_psi_raw_cd, jam_psi_raw_bin};     // 0x9C
            cap_vec[32*24 +: 32] <= {jam_noise_dbfs, jam_dbfs};            // 0xA0
            cap_vec[32*25 +: 32] <= {16'd0, jam_jnr};                      // 0xA4
            cap_vec[32*26 +: 32] <= jam_n_est;                             // 0xA8
            cap_vec[32*27 +: 32] <= jam_n_drop;                            // 0xAC
            cap_vec[32*28 +: 32] <= {cov_clip2, cov_clip1};                // 0xB0
            cap_vec[32*29 +: 32] <= {cov_peak2, cov_peak1};                // 0xB4
            cap_vec[32*30 +: 32] <= nf_x[31:0];                            // 0xB8
            cap_vec[32*31 +: 32] <= {10'd0, coh_l2, nf_x[47:32]};          // 0xBC
            cap_vec[32*32 +: 32] <= n_jam;                                 // 0xE0
            cap_vec[32*33 +: 32] <= n_hold;                                // 0xE4
            cap_vec[32*34 +: 32] <= {{14{iq_b1_re[17]}}, iq_b1_re};        // 0xE8
            cap_vec[32*35 +: 32] <= {{14{iq_b1_im[17]}}, iq_b1_im};        // 0xEC
            cap_vec[32*36 +: 32] <= {{14{iq_b2_re[17]}}, iq_b2_re};        // 0xF0
            cap_vec[32*37 +: 32] <= {{14{iq_b2_im[17]}}, iq_b2_im};        // 0xF4
            cap_vec[32*38 +: 32] <= n_iq_upd;                              // 0xF8
            cap_vec[32*39 +: 32] <= out_lvl;                               // 0xFC
            cap_vec[32*40 +: 32] <= {jam_j2, jam_j1};                      // page 2 0x100
            cap_vec[32*41 +: 32] <= {jam_pk2, jam_pk1};                    // page 2 0x104
            cap_vec[32*42 +: 32] <= {jam_n2, jam_n1};                      // page 2 0x108
            cap_vec[32*43 +: 32] <= {(HAVE_STAP == 0), 22'd0, stap_active, 2'd0, stap_pl};   // page 2 0x110, [31] = no NLMS STAP
            cap_vec[32*44 +: 32] <= stap_wnorm;                            // page 2 0x114
            cap_vec[32*45 +: 32] <= stap_nwsat;                            // page 2 0x118
            cap_tog <= ~cap_tog;
        end
    end

    (* ASYNC_REG = "TRUE" *) reg st_m = 1'b0;
    (* ASYNC_REG = "TRUE" *) reg st_s = 1'b0;
    reg                       st_p = 1'b0;
    reg  [32*(NSW+NXS)-1:0]   h_vec = {(32*(NSW+NXS)){1'b0}};

    always @(posedge s_axi_aclk) begin
        st_m <= cap_tog;
        st_s <= st_m;
        st_p <= st_s;
        if (st_s != st_p) h_vec <= cap_vec;
    end

    // ==========================================================================
    //  read mux (combinational; the parent registers it with its own read logic)
    // ==========================================================================
    wire [63:0] eps_x = {{(64-ACC_W){1'b0}}, sh_eps};
    wire [7:0]  sidx  = rd_addr - 8'd16;

    always @* begin
        rd_data = 32'd0;
        case (rd_addr)
            8'd0:  rd_data = REG_ID;
            8'd1:  rd_data = REG_VERSION;
            8'd2:  rd_data = {27'd0, sh_dcrm, sh_clr, sh_freeze, sh_mode};
            8'd3:  rd_data = {31'd0, pending};
            8'd4:  rd_data = {sh_a1i, sh_a1r};
            8'd5:  rd_data = {sh_a2i, sh_a2r};
            8'd6:  rd_data = eps_x[31:0];
            8'd7:  rd_data = eps_x[63:32];
            8'd8:  rd_data = {26'd0, sh_rel};
            8'd9:  rd_data = {{(32-W_W){sh_f1r[W_W-1]}}, sh_f1r};
            8'd10: rd_data = {{(32-W_W){sh_f1i[W_W-1]}}, sh_f1i};
            8'd11: rd_data = {{(32-W_W){sh_f2r[W_W-1]}}, sh_f2r};
            8'd12: rd_data = {{(32-W_W){sh_f2i[W_W-1]}}, sh_f2i};
            8'd48: rd_data = {20'd0, sh_javg, 6'd0, sh_jinv, sh_jen};
            8'd49: rd_data = {16'd0, sh_jcal};
            8'd50: rd_data = {16'd0, sh_jk};
            8'd51: rd_data = {sh_joff, sh_jon};
            8'd52: rd_data = {8'd0, sh_tdb, sh_agcsh, 3'd0, sh_agc, 5'd0, sh_ogain};
            8'd53: rd_data = {24'd0, sh_nfk};
            8'd54: rd_data = {4'd0, sh_cohs, 4'd0, sh_rels, sh_hold};
            8'd56: rd_data = h_vec[32*NSW +: 32];                    // 0xE0 N_JAM
            8'd57: rd_data = h_vec[32*(NSW+1) +: 32];                // 0xE4 N_HOLD
            8'd55: rd_data = {25'd0, sh_iq};                          // 0xDC IQ_CTRL
            8'd58: rd_data = h_vec[32*(NSW+2) +: 32];                // 0xE8
            8'd59: rd_data = h_vec[32*(NSW+3) +: 32];                // 0xEC
            8'd60: rd_data = h_vec[32*(NSW+4) +: 32];                // 0xF0
            8'd61: rd_data = h_vec[32*(NSW+5) +: 32];                // 0xF4
            8'd62: rd_data = h_vec[32*(NSW+6) +: 32];                // 0xF8
            8'd63: rd_data = h_vec[32*(NSW+7) +: 32];                // 0xFC OUT_LVL
            8'd64: rd_data = h_vec[32*(NSW+8) +: 32];                // p2 0x100 JAM_PWR_CH
            8'd65: rd_data = h_vec[32*(NSW+9) +: 32];                // p2 0x104 JAM_PEAK
            8'd66: rd_data = h_vec[32*(NSW+10) +: 32];               // p2 0x108 NOISE_CH
            8'd67: rd_data = {14'd0, sh_stap[17:16], 3'd0, sh_stap[15:11], sh_stap[10:7], 3'd0, sh_stap[0]};  // p2 0x10C STAP_CTRL
            8'd68: rd_data = h_vec[32*(NSW+11) +: 32];               // p2 0x110 STAP_ST
            8'd69: rd_data = h_vec[32*(NSW+12) +: 32];               // p2 0x114 STAP_WNORM
            8'd70: rd_data = h_vec[32*(NSW+13) +: 32];               // p2 0x118 STAP_NWSAT
            default: begin
                if (rd_addr >= 8'd16 && rd_addr < (8'd16 + NSW)) begin
                    rd_data = h_vec[sidx*32 +: 32];
                    if (rd_addr == 8'd16) rd_data[7] = pending;      // commit pending (AXI domain)
                end
            end
        endcase
    end

endmodule
