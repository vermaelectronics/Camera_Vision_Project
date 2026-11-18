// -----------------------------------------------------------------------------
// jammer_est.v  --  jammer detection, power and angle-of-arrival from the
//                   2x2 covariance block sums of cov2x2_accum
//
// INPUT  : raw block sums r11, r22, r12 = sum x1*conj(x2)  (one set per block,
//          cov_valid pulse), straight from mvdr_top's cov_blk / cov_r* ports.
//          Independent of the MVDR mode: it runs in bypass, adaptive or fixed.
//
// STEP 1  Averaging: 2**avg_log2 consecutive blocks are summed (0..AVG_MAX),
//         default 16 blocks = 65536 samples = 1.3 ms at 50 MSPS.
//
// STEP 2  (v2) Jammer / noise split from the RX1-RX2 COHERENCE
//             g = sqrt(r11 r22) ,  m = |r12| ,  gamma = m / g
//             jammer power  RX_k :  J_k = gamma * r_kk      (k = 1, 2)
//             noise  power  RX_k :  N_k = (1 - gamma) * r_kk
//             JNR             = gamma / (1 - gamma) = m / (g - m)
//             jam / noise (legacy fields) = geometric mean of the channels:
//                                 m  and  g - m
//         One jammer + independent noise, any RX1/RX2 gain ratio: a gain
//         difference scales jammer AND noise of that chain, so gamma and JNR
//         are unaffected. v1 used the eigenvalues of R, which assume equal
//         gains: with RX2 10 dB hotter and NO jammer it reported JNR +6.5 dB
//         (a false detection); with a jammer it blended the two channels into
//         one wrong number. With equal gains v1 and v2 agree exactly.
//         R is first right-shifted by a common `sh` so that T < 2^32 (block
//         floating point); `sh` is added back in the log domain.
//
// STEP 2b (v2) PEAK jammer power: max over the averaged windows of |r12_w|/N
//         (one window = 2^LOG2_N samples, 149 us). The average (STEP 2) is the
//         right number for a continuous jammer and for the C/N0 it costs; for a
//         jammer that is on only part of the time it reads the duty-cycle
//         average. The peak reads its ON power (pulse longer than a window;
//         shorter pulses: use the instantaneous ADC peak, COV_PEAK).
//         |r12_w|^2 is computed exactly per window by a small shift-add
//         squarer (no DSP), alongside the sequencer.
//
// STEP 3  Phase difference between the elements:
//             arg(r12)  by CORDIC (vectoring, 22 iterations)
//             psi_raw   = -arg(r12)            (cfg_phase_inv = 0, see below)
//             psi       =  psi_raw - phase_cal
//         For a plane wave reaching RX2 first (source on the RX2 side),
//         x1 = x2 * exp(-j*psi) with psi = 2*pi*d*sin(theta)/lambda, so
//         r12 = P*exp(-j*psi) and psi = -arg(r12) > 0.
//
// STEP 4  Angle from broadside, positive towards RX2:
//             sin(theta) = (psi/pi) * K ,   K = lambda/(2d)   (Q14 register)
//             cos(theta) = sqrt(1 - sin^2)
//             theta      = atan2(sin, cos)  by the same CORDIC
//         d = 94.6 mm, LO 1584.5 MHz -> lambda = 189.2 mm -> K = 1.00002
//         -> 16384.  |sin| > 1 (bad calibration / ambiguity) is clipped to
//         +-90 deg and flagged.
//
// STEP 5  Levels in dB (log2 by normalise + 128-entry table, error < 0.02 dB):
//             jam_dbfs   = 10log10( m / (Ntot * FS) )   (geometric mean RX1/RX2)
//             noise_dbfs = 10log10( (g-m) / (Ntot * FS) )
//             jnr_db     = 10log10( m / (g - m) )
//             j1/j2  = jam_dbfs   +/- 10log10(r11/r22)/2   (per channel)
//             n1/n2  = noise_dbfs +/- the same
//             pk1/pk2 = 10log10(max_w |r12_w| / (N FS)) +/- the same
//         All are AVERAGE (= RMS^2) POWER of a complex signal relative to a
//         full-scale complex tone. For a CW jammer average, peak and RMS power
//         are the same number (constant envelope).
//         FS = 2^(2*(DATA_W-1)) = power of a full-scale complex tone
//         (|I|,|Q| amplitude 2048 for 12 bits), Ntot = 2^(LOG2_N+avg).
//         So jam_dbfs is "dB relative to ADC full scale" per element; add the
//         RX gain / RF-chain calibration in software to get dBm.
//
// STEP 6  Decision with hysteresis:
//             present  ->  stays while jnr_db >= jnr_off
//             absent   ->  becomes present when jnr_db >= jnr_on
//
// OUTPUT FORMATS (all signed 16-bit)
//   theta_cdeg, psi_cdeg, psi_raw_cdeg : 0.01 degree
//   psi_raw_bin                         : binary angle, 32768 = pi
//                                         (write this value to PHASE_CAL to
//                                          calibrate with a source at broadside)
//   jam_dbfs_q8, noise_dbfs_q8, jnr_db_q8 : 1/256 dB  (-32768 = no energy)
//   j1/j2, n1/n2, pk1/pk2_dbfs_q8           : 1/256 dB per channel (v2)
//
// IMPLEMENTATION
//   Fully sequential micro-sequencer: one 32x32 multiplier (4-stage pipe),
//   one bit-serial square root, one iterative CORDIC, one iterative log2.
//   About 650 clocks per estimate (v2), far less than the window hop (>= 1600
//   clocks), so every averaging period is processed.
//   NOTE (since covariance v3): published windows overlap (8192 samples,
//   hop 1024). Averaging 2^avg of them spans 2^avg hops; the POWER scale is
//   exact (each window holds N samples), but the averaged estimate is built
//   from only ~(2^avg + 7) x 1024 independent samples.  The widest
//   single-cycle operation is a 46-bit add; every other path is <= 36 bits.
//   TIMING (v1.1): the rounded/shifted multiplier results (dB, 0.01-degree and
//   sin scaling) are REGISTERED one stage after the product (m_v is 5 deep), so
//   no state ever does multiply-output -> round -> shift -> negate -> saturate
//   in one clock.  That chain was the slowest jammer_est path (+0.54 ns at
//   8 ns on rx_clk); it costs one extra clock per multiply.
//   An averaging window that completes while the sequencer is still busy is
//   dropped and counted in n_drop (should stay 0).
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module jammer_est #(
    parameter DATA_W  = 12,
    parameter LOG2_N  = 12,
    parameter ACC_W   = 2*DATA_W + LOG2_N + 1,   // must match cov2x2_accum
    parameter AVG_MAX = 8                        // max averaging = 2^AVG_MAX blocks
)(
    input  wire                     clk,
    input  wire                     rst,            // synchronous, active high

    // ---- covariance block sums (from mvdr_top) ----
    input  wire                     cov_valid,
    input  wire signed [ACC_W-1:0]  r11,
    input  wire signed [ACC_W-1:0]  r22,
    input  wire signed [ACC_W-1:0]  r12_re,
    input  wire signed [ACC_W-1:0]  r12_im,

    // ---- configuration (already synchronous to clk) ----
    input  wire                     cfg_enable,
    input  wire                     cfg_phase_inv,  // 1 = psi_raw = +arg(r12)
    input  wire [3:0]               cfg_avg_log2,   // 0..AVG_MAX (clamped)
    input  wire signed [15:0]       cfg_phase_cal,  // binary angle, 32768 = pi
    input  wire [15:0]              cfg_k_q14,      // lambda/(2d), Q14
    input  wire signed [15:0]       cfg_jnr_on_q8,  // detect threshold, dB Q8
    input  wire signed [15:0]       cfg_jnr_off_q8, // release threshold, dB Q8

    // ---- results (all change on the same clock, with est_done) ----
    output reg                      est_done,
    output reg                      jam_present,
    output reg                      st_valid,       // at least one estimate since enable
    output reg                      st_sin_clip,    // |sin(theta)| > 1 was clipped
    output reg                      st_no_signal,   // R was all zero
    output reg                      st_noise_zero,  // lambda2 = 0 (JNR beyond range)
    output reg  signed [15:0]       theta_cdeg,
    output reg  signed [15:0]       psi_cdeg,
    output reg  signed [15:0]       psi_raw_bin,
    output reg  signed [15:0]       psi_raw_cdeg,
    output reg  signed [15:0]       jam_dbfs_q8,
    output reg  signed [15:0]       noise_dbfs_q8,
    output reg  signed [15:0]       jnr_db_q8,
    output reg  signed [15:0]       j1_dbfs_q8,     // jammer power RX1 (average)
    output reg  signed [15:0]       j2_dbfs_q8,     // jammer power RX2 (average)
    output reg  signed [15:0]       n1_dbfs_q8,     // noise power RX1
    output reg  signed [15:0]       n2_dbfs_q8,     // noise power RX2
    output reg  signed [15:0]       pk1_dbfs_q8,    // jammer peak (on) power RX1
    output reg  signed [15:0]       pk2_dbfs_q8,    // jammer peak (on) power RX2
    output reg  [31:0]              n_est,
    output reg  [31:0]              n_drop
);

    // =========================================================================
    //  Constant tables
    // =========================================================================
    // atan(2^-i) as a 24-bit binary angle (2^23 = pi)
    function [23:0] atan_lut;
        input [4:0] i;
        begin
            case (i)
            5'd0 : atan_lut = 24'd2097152;
            5'd1 : atan_lut = 24'd1238021;
            5'd2 : atan_lut = 24'd654136;
            5'd3 : atan_lut = 24'd332050;
            5'd4 : atan_lut = 24'd166669;
            5'd5 : atan_lut = 24'd83416;
            5'd6 : atan_lut = 24'd41718;
            5'd7 : atan_lut = 24'd20860;
            5'd8 : atan_lut = 24'd10430;
            5'd9 : atan_lut = 24'd5215;
            5'd10: atan_lut = 24'd2608;
            5'd11: atan_lut = 24'd1304;
            5'd12: atan_lut = 24'd652;
            5'd13: atan_lut = 24'd326;
            5'd14: atan_lut = 24'd163;
            5'd15: atan_lut = 24'd81;
            5'd16: atan_lut = 24'd41;
            5'd17: atan_lut = 24'd20;
            5'd18: atan_lut = 24'd10;
            5'd19: atan_lut = 24'd5;
            5'd20: atan_lut = 24'd3;
            5'd21: atan_lut = 24'd1;
            default: atan_lut = 24'd0;
            endcase
        end
    endfunction

    // log2(1 + (k+0.5)/128) in Q10
    function [9:0] log_lut;
        input [6:0] k;
        begin
            case (k)
            7'd0  : log_lut = 10'd6;
            7'd1  : log_lut = 10'd17;
            7'd2  : log_lut = 10'd29;
            7'd3  : log_lut = 10'd40;
            7'd4  : log_lut = 10'd51;
            7'd5  : log_lut = 10'd62;
            7'd6  : log_lut = 10'd73;
            7'd7  : log_lut = 10'd84;
            7'd8  : log_lut = 10'd95;
            7'd9  : log_lut = 10'd106;
            7'd10 : log_lut = 10'd116;
            7'd11 : log_lut = 10'd127;
            7'd12 : log_lut = 10'd138;
            7'd13 : log_lut = 10'd148;
            7'd14 : log_lut = 10'd159;
            7'd15 : log_lut = 10'd169;
            7'd16 : log_lut = 10'd179;
            7'd17 : log_lut = 10'd189;
            7'd18 : log_lut = 10'd199;
            7'd19 : log_lut = 10'd209;
            7'd20 : log_lut = 10'd219;
            7'd21 : log_lut = 10'd229;
            7'd22 : log_lut = 10'd239;
            7'd23 : log_lut = 10'd249;
            7'd24 : log_lut = 10'd259;
            7'd25 : log_lut = 10'd268;
            7'd26 : log_lut = 10'd278;
            7'd27 : log_lut = 10'd288;
            7'd28 : log_lut = 10'd297;
            7'd29 : log_lut = 10'd306;
            7'd30 : log_lut = 10'd316;
            7'd31 : log_lut = 10'd325;
            7'd32 : log_lut = 10'd334;
            7'd33 : log_lut = 10'd343;
            7'd34 : log_lut = 10'd353;
            7'd35 : log_lut = 10'd362;
            7'd36 : log_lut = 10'd371;
            7'd37 : log_lut = 10'd380;
            7'd38 : log_lut = 10'd388;
            7'd39 : log_lut = 10'd397;
            7'd40 : log_lut = 10'd406;
            7'd41 : log_lut = 10'd415;
            7'd42 : log_lut = 10'd424;
            7'd43 : log_lut = 10'd432;
            7'd44 : log_lut = 10'd441;
            7'd45 : log_lut = 10'd449;
            7'd46 : log_lut = 10'd458;
            7'd47 : log_lut = 10'd466;
            7'd48 : log_lut = 10'd475;
            7'd49 : log_lut = 10'd483;
            7'd50 : log_lut = 10'd491;
            7'd51 : log_lut = 10'd500;
            7'd52 : log_lut = 10'd508;
            7'd53 : log_lut = 10'd516;
            7'd54 : log_lut = 10'd524;
            7'd55 : log_lut = 10'd532;
            7'd56 : log_lut = 10'd540;
            7'd57 : log_lut = 10'd548;
            7'd58 : log_lut = 10'd556;
            7'd59 : log_lut = 10'd564;
            7'd60 : log_lut = 10'd572;
            7'd61 : log_lut = 10'd580;
            7'd62 : log_lut = 10'd587;
            7'd63 : log_lut = 10'd595;
            7'd64 : log_lut = 10'd603;
            7'd65 : log_lut = 10'd610;
            7'd66 : log_lut = 10'd618;
            7'd67 : log_lut = 10'd626;
            7'd68 : log_lut = 10'd633;
            7'd69 : log_lut = 10'd641;
            7'd70 : log_lut = 10'd648;
            7'd71 : log_lut = 10'd656;
            7'd72 : log_lut = 10'd663;
            7'd73 : log_lut = 10'd670;
            7'd74 : log_lut = 10'd678;
            7'd75 : log_lut = 10'd685;
            7'd76 : log_lut = 10'd692;
            7'd77 : log_lut = 10'd699;
            7'd78 : log_lut = 10'd707;
            7'd79 : log_lut = 10'd714;
            7'd80 : log_lut = 10'd721;
            7'd81 : log_lut = 10'd728;
            7'd82 : log_lut = 10'd735;
            7'd83 : log_lut = 10'd742;
            7'd84 : log_lut = 10'd749;
            7'd85 : log_lut = 10'd756;
            7'd86 : log_lut = 10'd763;
            7'd87 : log_lut = 10'd770;
            7'd88 : log_lut = 10'd776;
            7'd89 : log_lut = 10'd783;
            7'd90 : log_lut = 10'd790;
            7'd91 : log_lut = 10'd797;
            7'd92 : log_lut = 10'd803;
            7'd93 : log_lut = 10'd810;
            7'd94 : log_lut = 10'd817;
            7'd95 : log_lut = 10'd823;
            7'd96 : log_lut = 10'd830;
            7'd97 : log_lut = 10'd837;
            7'd98 : log_lut = 10'd843;
            7'd99 : log_lut = 10'd850;
            7'd100: log_lut = 10'd856;
            7'd101: log_lut = 10'd863;
            7'd102: log_lut = 10'd869;
            7'd103: log_lut = 10'd875;
            7'd104: log_lut = 10'd882;
            7'd105: log_lut = 10'd888;
            7'd106: log_lut = 10'd894;
            7'd107: log_lut = 10'd901;
            7'd108: log_lut = 10'd907;
            7'd109: log_lut = 10'd913;
            7'd110: log_lut = 10'd919;
            7'd111: log_lut = 10'd926;
            7'd112: log_lut = 10'd932;
            7'd113: log_lut = 10'd938;
            7'd114: log_lut = 10'd944;
            7'd115: log_lut = 10'd950;
            7'd116: log_lut = 10'd956;
            7'd117: log_lut = 10'd962;
            7'd118: log_lut = 10'd968;
            7'd119: log_lut = 10'd974;
            7'd120: log_lut = 10'd980;
            7'd121: log_lut = 10'd986;
            7'd122: log_lut = 10'd992;
            7'd123: log_lut = 10'd998;
            7'd124: log_lut = 10'd1004;
            7'd125: log_lut = 10'd1010;
            7'd126: log_lut = 10'd1015;
            7'd127: log_lut = 10'd1021;
            default: log_lut = 10'd0;
            endcase
        end
    endfunction

    // saturate a signed value to 16 bits
    function signed [15:0] sat16;
        input signed [39:0] v;
        begin
            if (v > 40'sd32767)       sat16 = 16'sh7FFF;
            else if (v < -40'sd32768) sat16 = 16'sh8000;
            else                      sat16 = v[15:0];
        end
    endfunction

    // =========================================================================
    //  Block averaging
    // =========================================================================
    localparam        AW       = ACC_W + AVG_MAX;   // averaged-sum width
    localparam [3:0]  AVG_MAX4 = AVG_MAX;

    wire [3:0] avg_cfg = (cfg_avg_log2 > AVG_MAX4) ? AVG_MAX4 : cfg_avg_log2;

    reg  [AVG_MAX:0]        bcnt;
    reg  [3:0]              avg_l;
    reg  signed [AW-1:0]    a11, a22, are, aim;

    wire                    first   = (bcnt == {(AVG_MAX+1){1'b0}});
    wire [3:0]              avg_use = first ? avg_cfg : avg_l;
    wire [AVG_MAX:0]        blk_tgt = {{AVG_MAX{1'b0}}, 1'b1} << avg_use;
    wire [AVG_MAX:0]        bcnt_n  = bcnt + 1'b1;
    wire                    last    = (bcnt_n == blk_tgt);

    wire signed [AW-1:0] r11x = {{AVG_MAX{r11[ACC_W-1]}},    r11};
    wire signed [AW-1:0] r22x = {{AVG_MAX{r22[ACC_W-1]}},    r22};
    wire signed [AW-1:0] rrex = {{AVG_MAX{r12_re[ACC_W-1]}}, r12_re};
    wire signed [AW-1:0] rimx = {{AVG_MAX{r12_im[ACC_W-1]}}, r12_im};

    wire signed [AW-1:0] s11 = (first ? {AW{1'b0}} : a11) + r11x;
    wire signed [AW-1:0] s22 = (first ? {AW{1'b0}} : a22) + r22x;
    wire signed [AW-1:0] sre = (first ? {AW{1'b0}} : are) + rrex;
    wire signed [AW-1:0] sim = (first ? {AW{1'b0}} : aim) + rimx;

    reg                     go;
    reg  signed [AW-1:0]    e11, e22, ere, eim;
    reg  [3:0]              e_avg;
    wire                    fsm_idle;

    always @(posedge clk) begin
        go <= 1'b0;
        if (rst) begin
            bcnt   <= {(AVG_MAX+1){1'b0}};
            avg_l  <= 4'd0;
            n_drop <= 32'd0;
        end else if (!cfg_enable) begin
            bcnt   <= {(AVG_MAX+1){1'b0}};
        end else if (cov_valid) begin
            if (first) avg_l <= avg_cfg;
            if (last) begin
                bcnt <= {(AVG_MAX+1){1'b0}};
                if (fsm_idle && !go) begin
                    e11 <= s11;  e22 <= s22;  ere <= sre;  eim <= sim;
                    e_avg <= avg_use;
                    go    <= 1'b1;
                end else begin
                    n_drop <= n_drop + 32'd1;
                end
            end else begin
                a11 <= s11;  a22 <= s22;  are <= sre;  aim <= sim;
                bcnt <= bcnt_n;
            end
        end
    end

    // =========================================================================
    //  Peak tracker: max over windows of |r12_w|^2 (exact, shift-add, ~92 clk)
    // =========================================================================
    localparam PKW = 2*ACC_W + 1;
    reg  [1:0]             pk_st;
    reg  [ACC_W-1:0]       pk_im;
    reg  [PKW-1:0]         pk_cand, pk_acc, pk_sq, pk_max;
    reg  [ACC_W-1:0]       pk_mul;
    reg  [7:0]             pk_cnt;
    reg                    pk_clr;
    wire [ACC_W-1:0]       re_abs_in = r12_re[ACC_W-1] ? (~r12_re + 1'b1) : r12_re;
    wire [ACC_W-1:0]       im_abs_in = r12_im[ACC_W-1] ? (~r12_im + 1'b1) : r12_im;

    always @(posedge clk) begin
        if (rst || !cfg_enable) begin
            pk_st <= 2'd0;  pk_max <= {PKW{1'b0}};
        end else begin
            case (pk_st)
                2'd0: if (cov_valid) begin               // square |re|
                    pk_cand <= {{(PKW-ACC_W){1'b0}}, re_abs_in};
                    pk_mul  <= re_abs_in;   pk_im <= im_abs_in;
                    pk_acc  <= {PKW{1'b0}}; pk_cnt <= 8'd0;  pk_st <= 2'd1;
                end
                2'd1, 2'd2: begin
                    if (pk_mul[0]) pk_acc <= pk_acc + pk_cand;
                    pk_cand <= pk_cand << 1;
                    pk_mul  <= pk_mul >> 1;
                    if (pk_cnt == ACC_W - 1) begin
                        pk_cnt <= 8'd0;
                        if (pk_st == 2'd1) begin                 // then square |im|
                            pk_sq   <= pk_mul[0] ? pk_acc + pk_cand : pk_acc;
                            pk_cand <= {{(PKW-ACC_W){1'b0}}, pk_im};
                            pk_mul  <= pk_im;
                            pk_acc  <= {PKW{1'b0}};
                            pk_st   <= 2'd2;
                        end else begin
                            pk_sq <= pk_sq + (pk_mul[0] ? pk_acc + pk_cand : pk_acc);
                            pk_st <= 2'd3;
                        end
                    end else pk_cnt <= pk_cnt + 8'd1;
                end
                2'd3: begin
                    if (pk_sq > pk_max) pk_max <= pk_sq;
                    pk_st <= 2'd0;
                end
            endcase
            if (pk_clr) pk_max <= {PKW{1'b0}};
        end
    end

    // =========================================================================
    //  Shared 32x32 unsigned multiplier, 4 register stages after the operands
    // =========================================================================
    reg  [31:0] m_a, m_b;          // written by the sequencer together with mgo
    reg         mgo;
    reg  [63:0] m_p1, m_p2, m_p3, m_p4;
    reg  [4:0]  m_v;               // [3]: m_p4 valid, [4]: registered scalings valid

    always @(posedge clk) begin
        if (rst) m_v <= 5'd0;
        else     m_v <= {m_v[3:0], mgo};
        m_p1 <= m_a * m_b;
        m_p2 <= m_p1;
        m_p3 <= m_p2;
        m_p4 <= m_p3;
    end
    wire [63:0] m_p = m_p4;

    // =========================================================================
    //  Sequencer
    // =========================================================================
    localparam CW   = 36;          // CORDIC x/y width
    localparam ITER = 22;          // CORDIC iterations

    localparam [6:0]
        S_IDLE   = 7'd0,  S_NORM   = 7'd1,  S_PREP   = 7'd2,
        S_SQ0    = 7'd3,  S_SQ1    = 7'd4,  S_SQ2    = 7'd5,  S_SQ3   = 7'd6,  S_SQ4 = 7'd7,
        S_LAM    = 7'd8,  S_PSI    = 7'd9,  S_PSI2   = 7'd10,
        S_SIN    = 7'd11, S_SIN2   = 7'd12, S_COS    = 7'd13,
        S_THETA  = 7'd14, S_THETA2 = 7'd15,
        S_LG_M   = 7'd16, S_LG_D   = 7'd17, S_LG_A   = 7'd18, S_LG_B  = 7'd19,
        S_PK1    = 7'd20, S_PK2    = 7'd21, S_LG_P   = 7'd22,
        S_DB0    = 7'd23, S_DB1    = 7'd24, S_DB2    = 7'd25, S_DB3   = 7'd26,
        S_DB4    = 7'd27, S_DB5    = 7'd28, S_DB6    = 7'd29,
        S_DEG2   = 7'd30, S_DEG3   = 7'd31, S_PUB    = 7'd32,
        // subroutines (return through `ret`)
        S_MW     = 7'd40,                               // multiplier wait
        S_RT_INI = 7'd41, S_RT_IT  = 7'd42,             // square root
        S_CO_PRE = 7'd43, S_CO_SH  = 7'd44, S_CO_ADD = 7'd45,   // CORDIC
        S_LG_INI = 7'd46, S_LG_NRM = 7'd47, S_LG_LUT = 7'd48;   // log2

    reg  [6:0]  st, ret;
    assign fsm_idle = (st == S_IDLE);

    // ---- working registers ----
    reg  [AW:0]             tt;                 // trace, then normalised trace
    reg  signed [AW-1:0]    x11, x22, xre, xim;
    reg  [5:0]              sh;
    reg  [3:0]              avg_q;
    reg  [31:0]             t32;
    reg  [31:0]             dmag, remag, immag;
    reg  signed [32:0]      cre, cim;
    reg  [64:0]             qacc;
    reg  [31:0]             a32, b32;           // r11, r22 (normalised)
    reg  [31:0]             gg, mm, dg;         // sqrt(r11 r22), |r12|, g - m
    reg  [PKW-1:0]          pkv;
    reg  [7:0]              pk_bl, pk_e;
    reg  [15:0]             r_lm, r_ld, r_la, r_lb;
    reg  [17:0]             r_lp;
    reg                     r_lmz, r_ldz, r_lpz;
    reg  signed [19:0]      l_hd, l_pk;
    reg  signed [15:0]      r_hd_db, r_pk_db;

    // square root
    reg  [63:0]             sq_rad;
    reg  [34:0]             sq_rem;
    reg  [31:0]             sq_root;
    reg  [5:0]              sq_cnt;

    // CORDIC
    reg  signed [CW-1:0]    cx, cy, cxs, cys;
    reg  [23:0]             cz;
    reg  [4:0]              ci;

    // log2
    reg  [32:0]             lg_x;
    reg  [5:0]              lg_e;
    reg  [15:0]             lg_res;             // Q10
    reg                     lg_zero;

    // results in progress
    reg  signed [15:0]      r_psi_raw, r_psi;
    reg  [16:0]             r_smag;
    reg                     r_ssgn, r_clip;
    reg  signed [23:0]      r_theta;

    reg  signed [19:0]      l_pj, l_nf, l_jnr;
    reg  signed [15:0]      r_pj_db, r_nf_db, r_jnr_db;
    reg  signed [15:0]      r_psi_cd, r_psi_raw_cd, r_theta_cd;

    // ---- combinational helpers ----
    wire signed [AW-1:0]    dd      = x11 - x22;
    wire [AW-1:0]           dd_abs  = dd[AW-1]  ? (~dd  + 1'b1) : dd;
    wire [AW-1:0]           xre_abs = xre[AW-1] ? (~xre + 1'b1) : xre;
    wire [AW-1:0]           xim_abs = xim[AW-1] ? (~xim + 1'b1) : xim;

    wire [64:0]             qsum    = qacc + {1'b0, m_p};

    wire [34:0]             rt_remn  = {sq_rem[32:0], sq_rad[63:62]};
    wire [34:0]             rt_trial = {1'b0, sq_root, 2'b01};

    wire [23:0]             cz_rnd    = cz + 24'd128;
    wire signed [15:0]      arg16     = cz_rnd[23:8];
    wire signed [15:0]      psi_raw_w = cfg_phase_inv ? arg16 : -arg16;

    wire [16:0]             psi_abs     = r_psi[15]     ? (17'd0 - {r_psi[15], r_psi})         : {1'b0, r_psi};
    wire [16:0]             psi_raw_abs = r_psi_raw[15] ? (17'd0 - {r_psi_raw[15], r_psi_raw}) : {1'b0, r_psi_raw};
    wire [23:0]             theta_abs   = r_theta[23]   ? (24'd0 - r_theta) : r_theta;

    // Registered one clock after m_p4 (valid when m_v[4] = 1). m_a/m_b are held
    // while a multiply is outstanding, so m_p4 is still the same product then.
    reg  [49:0]             smag_w;

    // log-domain offset (Q10):  1 (the /2) + LOG2_N + avg + 2*(DATA_W-1) - sh
    localparam signed [19:0] L_BASE = 1024 * (LOG2_N + 2*(DATA_W-1));   // v2: no /2
    wire signed [19:0]      l_off   = L_BASE
                                      + $signed({6'd0, avg_q, 10'd0})
                                      - $signed({4'd0, sh, 10'd0});

    wire [19:0]             l_pj_abs  = l_pj[19]  ? (20'd0 - l_pj)  : l_pj;
    wire [19:0]             l_nf_abs  = l_nf[19]  ? (20'd0 - l_nf)  : l_nf;
    wire [19:0]             l_jnr_abs = l_jnr[19] ? (20'd0 - l_jnr) : l_jnr;
    wire [19:0]             l_hd_abs  = l_hd[19]  ? (20'd0 - l_hd)  : l_hd;
    wire [19:0]             l_pk_abs  = l_pk[19]  ? (20'd0 - l_pk)  : l_pk;

    function [7:0] bitlen_pk;
        input [PKW-1:0] v;
        integer i;
        begin
            bitlen_pk = 8'd0;
            for (i = 0; i < PKW; i = i + 1) if (v[i]) bitlen_pk = i + 1;
        end
    endfunction

    // dB = log2(Q10) * 10*log10(2)/4  ->  Q8 ;  49321 = 0.752575 * 2^16
    localparam [31:0] DB_K  = 32'd49321;
    localparam [31:0] CD_K  = 32'd18000;        // binary angle -> 0.01 degree
    reg  [39:0]             db_mag, cd15_mag, cd23_mag;

    always @(posedge clk) begin
        smag_w   <= (m_p[49:0] + 50'd8192)    >> 14;
        db_mag   <= (m_p[39:0] + 40'd32768)   >> 16;
        cd15_mag <= (m_p[39:0] + 40'd16384)   >> 15;   // 16-bit angle
        cd23_mag <= (m_p[39:0] + 40'd4194304) >> 23;   // 24-bit angle
    end

    always @(posedge clk) begin
        mgo      <= 1'b0;
        est_done <= 1'b0;
        pk_clr   <= 1'b0;

        if (rst) begin
            st            <= S_IDLE;
            ret           <= S_IDLE;
            jam_present   <= 1'b0;
            st_valid      <= 1'b0;
            st_sin_clip   <= 1'b0;
            st_no_signal  <= 1'b0;
            st_noise_zero <= 1'b0;
            theta_cdeg    <= 16'sd0;
            psi_cdeg      <= 16'sd0;
            psi_raw_bin   <= 16'sd0;
            psi_raw_cdeg  <= 16'sd0;
            jam_dbfs_q8   <= 16'sh8000;
            noise_dbfs_q8 <= 16'sh8000;
            jnr_db_q8     <= 16'sh8000;
            j1_dbfs_q8 <= 16'sh8000;  j2_dbfs_q8 <= 16'sh8000;
            n1_dbfs_q8 <= 16'sh8000;  n2_dbfs_q8 <= 16'sh8000;
            pk1_dbfs_q8 <= 16'sh8000; pk2_dbfs_q8 <= 16'sh8000;
            n_est         <= 32'd0;
        end else begin
            if (!cfg_enable) begin
                jam_present <= 1'b0;
                st_valid    <= 1'b0;
            end

            case (st)
            // -----------------------------------------------------------------
            S_IDLE: if (go) begin
                tt    <= {e11[AW-1], e11} + {e22[AW-1], e22};
                x11   <= e11;  x22 <= e22;  xre <= ere;  xim <= eim;
                sh    <= 6'd0;
                avg_q <= e_avg;
                st    <= S_NORM;
            end

            // common right shift until T < 2^32
            S_NORM: if (|tt[AW:32]) begin
                tt  <= tt >> 1;
                x11 <= x11 >>> 1;  x22 <= x22 >>> 1;
                xre <= xre >>> 1;  xim <= xim >>> 1;
                sh  <= sh + 6'd1;
            end else begin
                st  <= S_PREP;
            end

            S_PREP: begin
                t32   <= tt[31:0];
                a32   <= x11[31:0];             // 0 <= r11, r22 <= T < 2^32
                b32   <= x22[31:0];
                remag <= (|xre_abs[AW-1:32]) ? 32'hFFFF_FFFF : xre_abs[31:0];
                immag <= (|xim_abs[AW-1:32]) ? 32'hFFFF_FFFF : xim_abs[31:0];
                cre   <= xre[32:0];
                cim   <= xim[32:0];
                st    <= S_SQ0;
            end

            // g = sqrt(r11 r22) ; m = sqrt(re^2 + im^2) = |r12|
            S_SQ0: begin m_a <= a32; m_b <= b32; mgo <= 1'b1; ret <= S_SQ1; st <= S_MW; end
            S_SQ1: begin sq_rad <= m_p; ret <= S_SQ2; st <= S_RT_INI; end
            S_SQ2: begin gg <= sq_root;
                         m_a <= remag; m_b <= remag; mgo <= 1'b1; ret <= S_SQ3; st <= S_MW; end
            S_SQ3: begin qacc <= {1'b0, m_p};
                         m_a <= immag; m_b <= immag; mgo <= 1'b1; ret <= S_SQ4; st <= S_MW; end
            S_SQ4: begin
                sq_rad <= qsum[64] ? 64'hFFFF_FFFF_FFFF_FFFF : qsum[63:0];
                ret    <= S_LAM;
                st     <= S_RT_INI;
            end

            S_LAM: begin
                mm  <= (sq_root > gg) ? gg : sq_root;            // |r12| <= sqrt(r11 r22)
                dg  <= (gg > sq_root) ? (gg - sq_root) : 32'd0;
                cx  <= {{(CW-33){cre[32]}}, cre};
                cy  <= {{(CW-33){cim[32]}}, cim};
                ret <= S_PSI;
                st  <= S_CO_PRE;
            end

            S_PSI: begin
                r_psi_raw <= psi_raw_w;
                r_psi     <= psi_raw_w - cfg_phase_cal;
                st        <= S_PSI2;
            end

            // sin(theta) = psi/pi * K
            S_PSI2: begin m_a <= {15'd0, psi_abs}; m_b <= {16'd0, cfg_k_q14};
                          mgo <= 1'b1; ret <= S_SIN; st <= S_MW; end

            S_SIN: begin
                if (smag_w > 50'd32768) begin
                    r_smag <= 17'd32768;
                    r_clip <= 1'b1;
                end else begin
                    r_smag <= smag_w[16:0];
                    r_clip <= 1'b0;
                end
                r_ssgn <= r_psi[15];
                st     <= S_SIN2;
            end

            S_SIN2: begin m_a <= {15'd0, r_smag}; m_b <= {15'd0, r_smag};
                          mgo <= 1'b1; ret <= S_COS; st <= S_MW; end

            // cos = sqrt(2^30 - sin^2)  (Q15)
            S_COS: begin
                sq_rad <= {32'd0, (32'd1 << 30) - m_p[31:0]};
                ret    <= S_THETA;
                st     <= S_RT_INI;
            end

            S_THETA: begin
                cx  <= {{(CW-32){1'b0}}, sq_root};
                cy  <= r_ssgn ? -$signed({{(CW-17){1'b0}}, r_smag})
                              :  $signed({{(CW-17){1'b0}}, r_smag});
                ret <= S_THETA2;
                st  <= S_CO_PRE;
            end

            S_THETA2: begin
                r_theta <= cz;
                lg_x    <= {1'b0, mm};
                ret     <= S_LG_M;
                st      <= S_LG_INI;
            end
            S_LG_M: begin r_lm <= lg_res; r_lmz <= lg_zero;
                          lg_x <= {1'b0, dg};  ret <= S_LG_D; st <= S_LG_INI; end
            S_LG_D: begin r_ld <= lg_res; r_ldz <= lg_zero;
                          lg_x <= {1'b0, a32}; ret <= S_LG_A; st <= S_LG_INI; end
            S_LG_A: begin r_la <= lg_res;
                          lg_x <= {1'b0, b32}; ret <= S_LG_B; st <= S_LG_INI; end
            S_LG_B: begin r_lb <= lg_res;
                          pkv <= pk_max; pk_clr <= 1'b1; st <= S_PK1; end
            S_PK1:  begin pk_bl <= bitlen_pk(pkv); st <= S_PK2; end
            S_PK2:  begin
                lg_x <= (pk_bl > 8'd33) ? (pkv >> (pk_bl - 8'd33)) : pkv[32:0];
                pk_e <= (pk_bl > 8'd33) ? (pk_bl - 8'd33) : 8'd0;
                ret  <= S_LG_P;
                st   <= S_LG_INI;
            end
            S_LG_P: begin
                r_lp  <= {2'b00, lg_res} + {pk_e[7:0], 10'd0};
                r_lpz <= lg_zero;
                st    <= S_DB0;
            end

            // log2 values relative to full scale, Q10
            S_DB0: begin
                l_pj  <= $signed({4'd0, r_lm}) - l_off;
                l_nf  <= $signed({4'd0, r_ld}) - l_off;
                l_jnr <= $signed({4'd0, r_lm}) - $signed({4'd0, r_ld});
                l_hd  <= ($signed({4'd0, r_la}) - $signed({4'd0, r_lb})) >>> 1;
                l_pk  <= ($signed({2'd0, r_lp}) >>> 1) - L_BASE;
                st    <= S_DB1;
            end
            S_DB1: begin m_a <= {12'd0, l_pj_abs}; m_b <= DB_K;
                         mgo <= 1'b1; ret <= S_DB2; st <= S_MW; end
            S_DB2: begin
                r_pj_db <= sat16(l_pj[19] ? -$signed(db_mag) : $signed(db_mag));
                m_a <= {12'd0, l_nf_abs}; m_b <= DB_K;
                mgo <= 1'b1; ret <= S_DB3; st <= S_MW;
            end
            S_DB3: begin
                r_nf_db <= sat16(l_nf[19] ? -$signed(db_mag) : $signed(db_mag));
                m_a <= {12'd0, l_jnr_abs}; m_b <= DB_K;
                mgo <= 1'b1; ret <= S_DB4; st <= S_MW;
            end
            S_DB4: begin
                r_jnr_db <= sat16(l_jnr[19] ? -$signed(db_mag) : $signed(db_mag));
                m_a <= {12'd0, l_hd_abs}; m_b <= DB_K;
                mgo <= 1'b1; ret <= S_DB5; st <= S_MW;
            end
            S_DB5: begin
                r_hd_db <= sat16(l_hd[19] ? -$signed(db_mag) : $signed(db_mag));
                m_a <= {12'd0, l_pk_abs}; m_b <= DB_K;
                mgo <= 1'b1; ret <= S_DB6; st <= S_MW;
            end
            // angles to 0.01 degree
            S_DB6: begin
                r_pk_db <= sat16(l_pk[19] ? -$signed(db_mag) : $signed(db_mag));
                m_a <= {15'd0, psi_abs}; m_b <= CD_K;
                mgo <= 1'b1; ret <= S_DEG2; st <= S_MW;
            end
            S_DEG2: begin
                r_psi_cd <= sat16(r_psi[15] ? -$signed(cd15_mag) : $signed(cd15_mag));
                m_a <= {15'd0, psi_raw_abs}; m_b <= CD_K;
                mgo <= 1'b1; ret <= S_DEG3; st <= S_MW;
            end
            S_DEG3: begin
                r_psi_raw_cd <= sat16(r_psi_raw[15] ? -$signed(cd15_mag) : $signed(cd15_mag));
                m_a <= {8'd0, theta_abs}; m_b <= CD_K;
                mgo <= 1'b1; ret <= S_PUB; st <= S_MW;
            end

            // publish everything on one clock
            S_PUB: begin
                theta_cdeg    <= sat16(r_theta[23] ? -$signed(cd23_mag) : $signed(cd23_mag));
                psi_cdeg      <= r_psi_cd;
                psi_raw_bin   <= r_psi_raw;
                psi_raw_cdeg  <= r_psi_raw_cd;
                jam_dbfs_q8   <= r_lmz ? 16'sh8000 : r_pj_db;
                noise_dbfs_q8 <= r_ldz ? 16'sh8000 : r_nf_db;
                jnr_db_q8     <= r_lmz ? 16'sh8000 : (r_ldz ? 16'sh7FFF : r_jnr_db);
                j1_dbfs_q8    <= r_lmz ? 16'sh8000 : sat16($signed(r_pj_db) + $signed(r_hd_db));
                j2_dbfs_q8    <= r_lmz ? 16'sh8000 : sat16($signed(r_pj_db) - $signed(r_hd_db));
                n1_dbfs_q8    <= r_ldz ? 16'sh8000 : sat16($signed(r_nf_db) + $signed(r_hd_db));
                n2_dbfs_q8    <= r_ldz ? 16'sh8000 : sat16($signed(r_nf_db) - $signed(r_hd_db));
                pk1_dbfs_q8   <= r_lpz ? 16'sh8000 : sat16($signed(r_pk_db) + $signed(r_hd_db));
                pk2_dbfs_q8   <= r_lpz ? 16'sh8000 : sat16($signed(r_pk_db) - $signed(r_hd_db));
                st_sin_clip   <= r_clip;
                st_no_signal  <= (t32 == 32'd0);
                st_noise_zero <= r_ldz;

                if (t32 == 32'd0 || r_lmz)
                    jam_present <= 1'b0;
                else if (r_ldz)
                    jam_present <= 1'b1;
                else if (jam_present)
                    jam_present <= (r_jnr_db >= cfg_jnr_off_q8);
                else
                    jam_present <= (r_jnr_db >= cfg_jnr_on_q8);

                st_valid <= cfg_enable;
                n_est    <= n_est + 32'd1;
                est_done <= 1'b1;
                st       <= S_IDLE;
            end

            // -----------------------------------------------------------------
            //  Subroutine: multiplier wait. On return m_p holds the product and
            //  smag_w / db_mag / cd15_mag / cd23_mag hold its scaled versions.
            // -----------------------------------------------------------------
            S_MW: if (m_v[4]) st <= ret;

            // -----------------------------------------------------------------
            //  Subroutine: sq_root = floor(sqrt(sq_rad)), 32 iterations
            // -----------------------------------------------------------------
            S_RT_INI: begin
                sq_rem  <= 35'd0;
                sq_root <= 32'd0;
                sq_cnt  <= 6'd32;
                st      <= S_RT_IT;
            end
            S_RT_IT: begin
                if (rt_remn >= rt_trial) begin
                    sq_rem  <= rt_remn - rt_trial;
                    sq_root <= {sq_root[30:0], 1'b1};
                end else begin
                    sq_rem  <= rt_remn;
                    sq_root <= {sq_root[30:0], 1'b0};
                end
                sq_rad <= sq_rad << 2;
                sq_cnt <= sq_cnt - 6'd1;
                if (sq_cnt == 6'd1) st <= ret;
            end

            // -----------------------------------------------------------------
            //  Subroutine: cz = atan2(cy, cx), binary angle (2^23 = pi)
            // -----------------------------------------------------------------
            S_CO_PRE: begin
                if (cx[CW-1]) begin                 // left half plane: rotate by pi
                    cx <= -cx;
                    cy <= -cy;
                    cz <= 24'h800000;
                end else begin
                    cz <= 24'd0;
                end
                ci <= 5'd0;
                st <= S_CO_SH;
            end
            S_CO_SH: begin
                cxs <= cx >>> ci;
                cys <= cy >>> ci;
                st  <= S_CO_ADD;
            end
            S_CO_ADD: begin
                if (!cy[CW-1]) begin
                    cx <= cx + cys;
                    cy <= cy - cxs;
                    cz <= cz + atan_lut(ci);
                end else begin
                    cx <= cx - cys;
                    cy <= cy + cxs;
                    cz <= cz - atan_lut(ci);
                end
                if (ci == ITER-1) begin
                    st <= ret;
                end else begin
                    ci <= ci + 5'd1;
                    st <= S_CO_SH;
                end
            end

            // -----------------------------------------------------------------
            //  Subroutine: lg_res = log2(lg_x) in Q10 (lg_x < 2^33)
            // -----------------------------------------------------------------
            S_LG_INI: begin
                if (lg_x == 33'd0) begin
                    lg_zero <= 1'b1;
                    lg_res  <= 16'd0;
                    st      <= ret;
                end else begin
                    lg_zero <= 1'b0;
                    lg_e    <= 6'd32;
                    st      <= S_LG_NRM;
                end
            end
            S_LG_NRM: begin
                if (!lg_x[32]) begin
                    lg_x <= lg_x << 1;
                    lg_e <= lg_e - 6'd1;
                end else begin
                    st   <= S_LG_LUT;
                end
            end
            S_LG_LUT: begin
                lg_res <= {lg_e, 10'd0} + {6'd0, log_lut(lg_x[31:25])};
                st     <= ret;
            end

            default: st <= S_IDLE;
            endcase
        end
    end

endmodule
