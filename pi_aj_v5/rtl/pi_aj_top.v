// ============================================================================
// pi_aj_top.v  --  PI-AJ v5 : 2-element GNSS anti-jam core (AD9361 / AntSDR)
//
//   RX1,RX2 --> frontend --> CW notch #1 --> image notch --> CW notch #2 -->
//               (scale,      (self-tuned,    (conj of #1,    (self-tuned,
//                clip, DC)    self-gated)     I/Q image)      2nd tone)
//
//           --> widely-linear space-time POWER INVERSION (2 ch x 5 taps,
//               NLMS, every sample) --> TX AGC --> TX1
//
// All filters before the power-inversion stage apply the SAME linear filter
// to RX1 and RX2, so the array geometry (jammer direction) is unchanged.
// The notches remove narrowband (CW / sharp) jammers in time-frequency,
// which costs no spatial degree of freedom; the power inversion then nulls
// whatever is left (wideband/barrage jammer, residual CW, multipath) in
// space-time. With 2 elements one spatial null exists, the notches remove
// that limit for CW jammers.
//
// Clocking: clk = sample-side clock (e.g. axi_ad9361 l_clk or a faster
// clock with in_valid strobes). Minimum clk / fs = 5 with the defaults
// (NL=5). s_axi_aclk is independent (all config/status cross via handshake).
// ============================================================================
`timescale 1ns / 1ps
module pi_aj_top #(
    parameter IN_SHIFT = 4,      // 4: 12-bit LSB-aligned ADC words (axi_ad9361)
    parameter L        = 5,      // PI taps per channel
    parameter NL       = 5,      // PI multiplier lanes
    parameter DELAY    = 3,      // PI update delay
    parameter WL       = 1,      // widely-linear PI
    parameter HAS_NOTCH2 = 1     // 0: drop CW notch #2 (saves ~35 DSP48)
) (
    input  wire         clk,
    input  wire         rst,            // active high, sync to clk
    // RX samples (AD9361 RX1, RX2)
    input  wire         adc_valid,
    input  wire [15:0]  adc_i0, adc_q0, adc_i1, adc_q1,
    // TX1 samples
    output reg          dac_valid,
    output reg  [15:0]  dac_i0, dac_q0,
    // AXI4-Lite
    input  wire         s_axi_aclk,
    input  wire         s_axi_aresetn,
    input  wire [6:0]   s_axi_awaddr,
    input  wire         s_axi_awvalid,
    output wire         s_axi_awready,
    input  wire [31:0]  s_axi_wdata,
    input  wire [3:0]   s_axi_wstrb,
    input  wire         s_axi_wvalid,
    output wire         s_axi_wready,
    output wire [1:0]   s_axi_bresp,
    output wire         s_axi_bvalid,
    input  wire         s_axi_bready,
    input  wire [6:0]   s_axi_araddr,
    input  wire         s_axi_arvalid,
    output wire         s_axi_arready,
    output wire [31:0]  s_axi_rdata,
    output wire [1:0]   s_axi_rresp,
    output wire         s_axi_rvalid,
    input  wire         s_axi_rready
);
    localparam DW = 18;
    localparam OW = 20;
    localparam NWT  = WL ? (4*L - 1) : (2*L - 1);
    localparam KCYC = (NWT + NL - 1) / NL;
    localparam [7:0] CAP_CPS = KCYC + 1;
    localparam [7:0] CAP_NL  = NL;
    localparam [7:0] CAP_L   = L;
    localparam [7:0] CAP_NW  = NWT;
    wire [31:0] caps = {CAP_CPS, CAP_NL, CAP_L, CAP_NW};

    // ------------------------------------------------------------ registers
    wire [31:0]  r_control, r_adapt, r_levels;
    wire [5:0]   r_wsel;
    wire         r_clr_tog;
    wire [351:0] st_axi;

    pi_aj_axil_regs u_regs (
        .s_axi_aclk(s_axi_aclk), .s_axi_aresetn(s_axi_aresetn),
        .s_axi_awaddr(s_axi_awaddr), .s_axi_awvalid(s_axi_awvalid), .s_axi_awready(s_axi_awready),
        .s_axi_wdata(s_axi_wdata), .s_axi_wstrb(s_axi_wstrb), .s_axi_wvalid(s_axi_wvalid),
        .s_axi_wready(s_axi_wready), .s_axi_bresp(s_axi_bresp), .s_axi_bvalid(s_axi_bvalid),
        .s_axi_bready(s_axi_bready), .s_axi_araddr(s_axi_araddr), .s_axi_arvalid(s_axi_arvalid),
        .s_axi_arready(s_axi_arready), .s_axi_rdata(s_axi_rdata), .s_axi_rresp(s_axi_rresp),
        .s_axi_rvalid(s_axi_rvalid), .s_axi_rready(s_axi_rready),
        .r_control(r_control), .r_adapt(r_adapt), .r_levels(r_levels),
        .r_wsel(r_wsel), .r_clr_tog(r_clr_tog), .st(st_axi), .caps(caps)
    );

    // config -> clk domain
    localparam CFGW = 32 + 32 + 32 + 6 + 1;
    wire [CFGW-1:0] cfg;
    pi_aj_cdc_bus #(.W(CFGW), .INIT({1'b0, 6'd0, 32'h0B1007F8, 32'h0008C600, 32'h0000043B}))
    u_cfg_cdc (
        .src_clk(s_axi_aclk), .src_rst(~s_axi_aresetn),
        .src_data({r_clr_tog, r_wsel, r_levels, r_adapt, r_control}),
        .dst_clk(clk), .dst_rst(rst), .dst_data(cfg)
    );
    wire [31:0] c_control = cfg[31:0];
    wire [31:0] c_adapt   = cfg[63:32];
    wire [31:0] c_levels  = cfg[95:64];
    wire [5:0]  c_wsel    = cfg[101:96];
    wire        c_clr_tog = cfg[102];

    wire core_en   = c_control[0];
    wire notch_en  = c_control[1];
    wire notch2_en = c_control[2];
    wire image_en  = c_control[3];
    wire pi_en     = c_control[4];
    wire agc_en    = c_control[5];
    wire dcblk_en  = c_control[6];
    wire blank_en  = c_control[7];
    wire freeze    = c_control[8];
    wire n_force   = c_control[9];
    wire auto_mu   = c_control[10];

    reg clr_d;
    always @(posedge clk) clr_d <= rst ? 1'b0 : c_clr_tog;
    wire clr = (clr_d != c_clr_tog);

    // ------------------------------------------------------------ frontend
    wire fe_v, fe_clip;
    wire signed [DW-1:0] fe0r, fe0i, fe1r, fe1i;
    pi_aj_frontend #(.DW(DW), .IN_SHIFT(IN_SHIFT)) u_fe (
        .clk(clk), .rst(rst),
        .clip_thr(c_levels[15:0]), .dcblk_en(dcblk_en), .blank_en(blank_en),
        .blank_hold(c_levels[23:16]),
        .in_valid(adc_valid), .in_i0(adc_i0), .in_q0(adc_q0), .in_i1(adc_i1), .in_q1(adc_q1),
        .out_valid(fe_v), .x0_re(fe0r), .x0_im(fe0i), .x1_re(fe1r), .x1_im(fe1i), .clip(fe_clip)
    );

    // ------------------------------------------------------------ notches
    wire na_v, nb_v, nc_v;
    wire signed [DW-1:0] na0r, na0i, na1r, na1i;
    wire signed [DW-1:0] nb0r, nb0i, nb1r, nb1i;
    wire signed [DW-1:0] nc0r, nc0i, nc1r, nc1i;
    wire signed [17:0]   a1_re, a1_im, ab_re, ab_im, a2_re, a2_im;
    wire                 na_act, nb_act, nc_act;
    wire [31:0]          pin0_a, pout_a, pin_b, pout_b, pin_c, pout_c;

    pi_aj_notch #(.DW(DW), .MODE(0)) u_notch1 (
        .clk(clk), .rst(rst), .clear(clr), .en(notch_en & core_en), .force_on(n_force),
        .freeze(freeze), .rho_sh(c_adapt[11:8]), .r_sh(c_adapt[15:12]), .a_sh(c_adapt[19:16]),
        .ext_a_re(18'sd0), .ext_a_im(18'sd0), .ext_act(1'b0),
        .in_valid(fe_v), .x0_re(fe0r), .x0_im(fe0i), .x1_re(fe1r), .x1_im(fe1i),
        .out_valid(na_v), .y0_re(na0r), .y0_im(na0i), .y1_re(na1r), .y1_im(na1i),
        .a_re_o(a1_re), .a_im_o(a1_im), .active(na_act), .pin_o(pin0_a), .pout_o(pout_a)
    );

    pi_aj_notch #(.DW(DW), .MODE(1)) u_notch_img (
        .clk(clk), .rst(rst), .clear(clr), .en(image_en & core_en), .force_on(1'b0),
        .freeze(freeze), .rho_sh(c_adapt[11:8]), .r_sh(c_adapt[15:12]), .a_sh(c_adapt[19:16]),
        .ext_a_re(a1_re), .ext_a_im(a1_im), .ext_act(na_act),
        .in_valid(na_v), .x0_re(na0r), .x0_im(na0i), .x1_re(na1r), .x1_im(na1i),
        .out_valid(nb_v), .y0_re(nb0r), .y0_im(nb0i), .y1_re(nb1r), .y1_im(nb1i),
        .a_re_o(ab_re), .a_im_o(ab_im), .active(nb_act), .pin_o(pin_b), .pout_o(pout_b)
    );

    // notch #2 must not sit on top of notch #1 (it would only chase the
    // residual of the same line): inhibit it while cos(w1-w2) > 1-2^-13,
    // i.e. while the two lines are closer than ~2^-6 rad/sample (~45 kHz).
    reg signed [36:0] a12_dot;
    reg               n2_inhibit;
    always @(posedge clk) begin
        a12_dot    <= a1_re * a2_re + a1_im * a2_im;            // Q32
        n2_inhibit <= na_act & (a12_dot > 37'sd4294443008);    // 2^32 - 2^19
    end

    generate
        if (HAS_NOTCH2) begin : g_n2
        pi_aj_notch #(.DW(DW), .MODE(0)) u_notch2 (
            .clk(clk), .rst(rst), .clear(clr), .en(notch2_en & core_en), .force_on(n_force),
            .freeze(freeze), .rho_sh(c_adapt[11:8]), .r_sh(c_adapt[15:12]), .a_sh(c_adapt[19:16]),
            .ext_a_re(18'sd0), .ext_a_im(18'sd0), .ext_act(n2_inhibit),
            .in_valid(nb_v), .x0_re(nb0r), .x0_im(nb0i), .x1_re(nb1r), .x1_im(nb1i),
            .out_valid(nc_v), .y0_re(nc0r), .y0_im(nc0i), .y1_re(nc1r), .y1_im(nc1i),
            .a_re_o(a2_re), .a_im_o(a2_im), .active(nc_act), .pin_o(pin_c), .pout_o(pout_c)
        );
        end else begin : g_no_n2
            assign nc_v = nb_v;
            assign nc0r = nb0r; assign nc0i = nb0i; assign nc1r = nb1r; assign nc1i = nb1i;
            assign a2_re = 18'sd65536; assign a2_im = 18'sd0;
            assign nc_act = 1'b0; assign pin_c = 32'd0; assign pout_c = 32'd0;
        end
    endgenerate

    // ------------------------------------------------------------ power inversion
    // auto_mu: a CW jammer (notch #1 active) wants a 2x smaller PI step than
    // a broadband/barrage jammer (whose slow eigen-modes need the large step)
    wire [3:0] mu_eff = (auto_mu && na_act && c_adapt[3:0] != 4'd15) ? c_adapt[3:0] + 4'd1
                                                                       : c_adapt[3:0];
    wire pi_v;
    wire signed [OW-1:0] pi_re, pi_im;
    wire signed [31:0]   w_re, w_im;
    wire                 pi_overrun, pi_late;
    pi_aj_wlpi #(.DW(DW), .L(L), .DELAY(DELAY), .NL(NL), .WL(WL), .OW(OW)) u_pi (
        .clk(clk), .rst(rst), .wreset(clr), .en(pi_en & core_en), .freeze(freeze),
        .mu_sh(mu_eff),
        .in_valid(nc_v), .x0_re(nc0r), .x0_im(nc0i), .x1_re(nc1r), .x1_im(nc1i),
        .out_valid(pi_v), .y_re(pi_re), .y_im(pi_im),
        .rd_sel(c_wsel), .rd_w_re(w_re), .rd_w_im(w_im),
        .overrun(pi_overrun), .late(pi_late)
    );

    // ------------------------------------------------------------ TX AGC
    wire ag_v;
    wire signed [15:0] ag_i, ag_q;
    wire signed [4:0]  ag_sh;
    wire [31:0]        p_out;
    pi_aj_outagc #(.IW(OW)) u_agc (
        .clk(clk), .rst(rst), .agc_en(agc_en), .man_shift(c_adapt[24:20]),
        .target(c_levels[27:24]),
        .in_valid(pi_v), .in_re(pi_re), .in_im(pi_im),
        .out_valid(ag_v), .out_i(ag_i), .out_q(ag_q), .shift_o(ag_sh), .pow_o(p_out)
    );

    // ------------------------------------------------------------ output mux
    // bypass: TX1 = RX1 (16-bit, 12-bit ADC word moved to the DAC MSBs)
    function [15:0] sat16;
        input signed [DW-1:0] v;
        if (v > 32767)       sat16 = 16'h7FFF;
        else if (v < -32768) sat16 = 16'h8000;
        else                 sat16 = v[15:0];
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            dac_valid <= 1'b0; dac_i0 <= 0; dac_q0 <= 0;
        end else if (core_en) begin
            dac_valid <= ag_v;
            if (ag_v) begin dac_i0 <= ag_i; dac_q0 <= ag_q; end
        end else begin
            dac_valid <= fe_v;
            if (fe_v) begin dac_i0 <= sat16(fe0r); dac_q0 <= sat16(fe0i); end
        end
    end

    // ------------------------------------------------------------ monitors
    // RX2 and post-notch powers
    reg signed [DW-1:0] m1r, m1i, mnr, mni;
    reg                 mv;
    reg [2*DW+12:0]     p_rx2, p_nt;
    wire [2*DW:0]       q_rx2 = m1r*m1r + m1i*m1i;
    wire [2*DW:0]       q_nt  = mnr*mnr + mni*mni;
    always @(posedge clk) begin
        if (rst) begin
            mv <= 0; p_rx2 <= 0; p_nt <= 0;
        end else begin
            mv <= nc_v;
            if (nc_v) begin
                m1r <= fe1r; m1i <= fe1i;       // RX2 (time alignment irrelevant)
                mnr <= nc0r; mni <= nc0i;
            end
            if (mv) begin
                if ({q_rx2, 12'd0} >= p_rx2) p_rx2 <= p_rx2 + (({q_rx2, 12'd0} - p_rx2) >> 12);
                else                         p_rx2 <= p_rx2 - ((p_rx2 - {q_rx2, 12'd0}) >> 12);
                if ({q_nt, 12'd0} >= p_nt)   p_nt  <= p_nt  + (({q_nt, 12'd0} - p_nt) >> 12);
                else                         p_nt  <= p_nt  - ((p_nt - {q_nt, 12'd0}) >> 12);
            end
        end
    end

    // ADC clip statistics
    reg [19:0] win;
    reg [31:0] clip_acc, clip_cnt;
    reg [31:0] samples;
    always @(posedge clk) begin
        if (rst) begin
            win <= 0; clip_acc <= 0; clip_cnt <= 0; samples <= 0;
        end else if (fe_v) begin
            samples <= samples + 1;
            win     <= win + 1;
            if (win == 20'hFFFFF) begin
                clip_cnt <= clip_acc + fe_clip;
                clip_acc <= 0;
            end else begin
                clip_acc <= clip_acc + fe_clip;
            end
        end
    end
    wire overload = (clip_cnt > 32'd1048);

    wire [31:0] status = {19'd0, ag_sh, 2'b00, pi_overrun, pi_late, overload,
                          nc_act, nb_act, na_act};
    wire [31:0] p_rx2_o = p_rx2[2*DW+12:12] > 32'hFFFFFFFF ? 32'hFFFFFFFF : p_rx2[43:12];
    wire [31:0] p_nt_o  = p_nt [2*DW+12:12] > 32'hFFFFFFFF ? 32'hFFFFFFFF : p_nt [43:12];
    wire [351:0] st_clk = {samples, w_im, w_re, clip_cnt,
                           a2_im[17:2], a2_re[17:2], a1_im[17:2], a1_re[17:2],
                           p_out, p_nt_o, p_rx2_o, pin0_a, status};

    pi_aj_cdc_bus #(.W(352)) u_st_cdc (
        .src_clk(clk), .src_rst(rst), .src_data(st_clk),
        .dst_clk(s_axi_aclk), .dst_rst(~s_axi_aresetn), .dst_data(st_axi)
    );
endmodule
