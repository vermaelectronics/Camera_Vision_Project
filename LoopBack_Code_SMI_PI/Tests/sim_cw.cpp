// ============================================================================
//  sim_cw.cpp  --  SMI-PI against a sharp CW on top of the GPS signal
//
//  Uses the real HLS core (smi_pi.cpp) and the firmware's own weight solver
//  (gnss_aj_math.c) exactly as Source/HLS/smi_pi/testbench.cpp does, but with
//  the board's RF profile 1 (61.44 MS/s) and the field-test situation of
//  28-Oct-2026: a CW at 1575.4 MHz = -7.1 MHz from the LO, on top of the GPS
//  C/A / Galileo E1 signals.
//
//  Three weak BPSK "satellites" (1.023 Mchip/s, independent codes, different
//  directions) are in the data:
//     A  at the CW frequency (like GPS L1 with the CW at 1575.4)
//     B  4 MHz above the CW
//     C  -21.4 MHz from the LO (BeiDou B1I)
//  For each, the SNR after the filter is measured by correlation with its known
//  waveform and compared with the SNR on RX1 without any jammer ("loss").
//
//  Build (from LoopBack_Code_SMI_PI):
//    g++ -O2 -w -I$HLS_INCLUDE -I<dir with hls_stream.h> -ISource/HLS/smi_pi \
//        -ISource/Firmware/app_gnss_e310 Tests/sim_cw.cpp \
//        Source/HLS/smi_pi/smi_pi.cpp Source/Firmware/app_gnss_e310/gnss_aj_math.c -o sim_cw
//  Run:  ./sim_cw [J/N dB ...]          (default 40 50 55 60 65 70)
// ============================================================================
#define main tb_main
#include "../Source/HLS/smi_pi/testbench.cpp"
#undef main

struct SatDef { const char *name; double f_mhz; double dir; };
struct CwCfg  { double fs_mhz; double cw_mhz; bool iq; double rx2_db; int shift_mode; };

struct Res { double supp, res, loss[3], cut; int ws; bool ok; };

// returns suppression and fills the losses; prints one line
static Res run_cw(double jnr_db, const CwCfg &cfg, bool verbose) {
    Res out; std::memset(&out, 0, sizeof out);
    const int N = 3 * AJ_NSAMP + 20000;
    const double nrms = 20.0;
    const double IQ1_EPS = 0.004, IQ1_PH = 0.3, IQ2_EPS = -0.003, IQ2_PH = -0.4;
    const double fs = cfg.fs_mhz;
    SatDef sats[3] = {{"A on CW", cfg.cw_mhz, -1.1}, {"B +4 MHz", cfg.cw_mhz + 4.0, 0.7}, {"C -21.4 MHz", -21.4, 2.4}};
    cvec J(N), J2, n1(N), n2(N), s1[3], s2[3];
    for (int k = 0; k < 3; k++) { s1[k].assign(N, 0.0); s2[k].assign(N, 0.0); }
    for (int i = 0; i < N; i++) J[i] = std::polar(1.0, 2 * M_PI * (cfg.cw_mhz / fs) * i + 0.3);
    cvec h2 = frac_delay(0.15, 21);
    cvec hd = {cd(0.03, 0.01), 1.0, cd(-0.02, 0.015)};
    J2 = fir(fir(J, h2, 10), hd, 1);
    cd g2p = std::polar(std::pow(10, 0.8 / 20), 40 * M_PI / 180);
    for (auto &v : J2) v *= g2p;
    double pj = pwr(J, 0), pn = 2 * nrms * nrms;
    double js = std::sqrt(pn * std::pow(10, jnr_db / 10) / pj);
    for (int i = 0; i < N; i++) { J[i] *= js; J2[i] *= js; }
    double chipstep = 1.023 / fs;
    int chip[3] = {1, 1, 1}; double cph[3] = {0, 0.3, 0.6};
    for (int i = 0; i < N; i++) {
        n1[i] = cd(grand(), grand()) * nrms;
        n2[i] = cd(grand(), grand()) * nrms;
        for (int k = 0; k < 3; k++) {
            cph[k] += chipstep; if (cph[k] >= 1) { cph[k] -= 1; chip[k] = urand() < 0.5 ? -1 : 1; }
            s1[k][i] = std::polar(0.1 * nrms * chip[k], 2 * M_PI * (sats[k].f_mhz / fs) * i);
            s2[k][i] = s1[k][i] * std::polar(1.0, sats[k].dir);
        }
    }
    double a2 = std::pow(10, cfg.rx2_db / 20);
    for (int i = 0; i < N; i++) { J2[i] *= a2; n2[i] *= a2; for (int k = 0; k < 3; k++) s2[k][i] *= a2; }

    double g2 = 1.0, rx2n = 1.0;
    {
        cvec q1(N), q2(N);
        for (int i = 0; i < N; i++) { q1[i] = n1[i]; q2[i] = n2[i]; }
        if (cfg.iq) { iq_imbalance(q1, IQ1_EPS, IQ1_PH); iq_imbalance(q2, IQ2_EPS, IQ2_PH); }
        for (int i = 0; i < N; i++) {
            q1[i] = cd(q12(q1[i].real()), q12(q1[i].imag()));
            q2[i] = cd(q12(q2[i].real()), q12(q2[i].imag()));
        }
        g2 = std::sqrt(pwr(q1, 0) / pwr(q2, 0));
        g2 = std::floor(g2 * 8192 + 0.5) / 8192;
        rx2n = g2 * g2 * pwr(q2, 0) / pwr(q1, 0);
    }
    cvec x1(N), x2(N), jx1(N), jx2(N), nx1(N), nx2(N);
    cvec sx1[3], sx2[3];
    for (int i = 0; i < N; i++) {
        cd t1 = 0, t2 = 0;
        for (int k = 0; k < 3; k++) { t1 += s1[k][i]; t2 += s2[k][i]; }
        x1[i] = J[i] + n1[i] + t1;  x2[i] = J2[i] + n2[i] + t2;
        jx1[i] = J[i]; jx2[i] = J2[i]; nx1[i] = n1[i]; nx2[i] = n2[i];
    }
    if (cfg.iq) {
        iq_imbalance(x1, IQ1_EPS, IQ1_PH);  iq_imbalance(x2, IQ2_EPS, IQ2_PH);
        iq_imbalance(jx1, IQ1_EPS, IQ1_PH); iq_imbalance(jx2, IQ2_EPS, IQ2_PH);
        iq_imbalance(nx1, IQ1_EPS, IQ1_PH); iq_imbalance(nx2, IQ2_EPS, IQ2_PH);
    }
    double pk = 0;
    for (int i = 0; i < N; i++)
        pk = std::max(pk, std::max(std::max(std::fabs(x1[i].real()), std::fabs(x1[i].imag())),
                                   std::max(std::fabs(x2[i].real()), std::fabs(x2[i].imag()))));
    double g = 1.0;
    while (pk * g > 1850) g *= std::pow(10, -1.0 / 20);
    double cut_db = -20 * std::log10(g);
    for (int i = 0; i < N; i++) {
        J[i] *= g; n1[i] *= g; for (int k = 0; k < 3; k++) s1[k][i] *= g;
        jx1[i] *= g; jx2[i] *= g; nx1[i] *= g; nx2[i] *= g;
        x1[i] = cd(q12((x1[i] * g).real()), q12((x1[i] * g).imag()));
        x2[i] = cd(q12((x2[i] * g).real()), q12((x2[i] * g).imag()));
    }
    aj_result_t r;
    {
        static uint32_t buf[2 * AJ_NSAMP];
        std::memset(&r, 0, sizeof r);
        r.mode = AJ_MODE_STAP; r.sig2_prev = -1; r.noise_ref = pn * g * g + 0.17; r.g2 = g2;
        r.rx2_noise = rx2n;
        aj_reset();
        for (int b = 0; b < 3; b++) {
            for (int t = 0; t < AJ_NSAMP; t++) {
                cd a = x1[b * AJ_NSAMP + t], c = x2[b * AJ_NSAMP + t];
                buf[2 * t]     = (uint32_t)(uint16_t)(int16_t)a.real() | ((uint32_t)(uint16_t)(int16_t)a.imag() << 16);
                buf[2 * t + 1] = (uint32_t)(uint16_t)(int16_t)c.real() | ((uint32_t)(uint16_t)(int16_t)c.imag() << 16);
            }
            if (aj_compute(buf, AJ_NSAMP, &r) != 0 || !r.ok) { std::printf("solver failed\n"); return out; }
            r.sig2_prev = r.sig2;
        }
    }
    if (verbose) {
        // frequency response of the solved filter around the CW: RX1 path (a_k + reference)
        // and RX2 path (b_k, normalised RX2) separately, power in dB
        double sa = 0, sb = 0;
        for (int k = 0; k < 16; k++) {
            if (k != SMI_REF) sa += std::pow(std::hypot(r.wre[k] / 4096.0, r.wim[k] / 4096.0), 2);
            sb += std::pow(std::hypot(r.wre[16 + k] / 4096.0, r.wim[16 + k] / 4096.0), 2);
        }
        std::printf("  weights: sum|a_k|^2 %.4f (RX1 taps)  sum|b_k|^2 %.4f (RX2 taps)  image %.4f %.4f\n", sa, sb,
                    std::hypot(r.wre[32] / 4096.0, r.wim[32] / 4096.0), std::hypot(r.wre[33] / 4096.0, r.wim[33] / 4096.0));
        std::printf("  |H1(f)|^2 dB re 1 at offsets from the CW [MHz]:");
        double offs[] = {0, 0.1, 0.25, 0.5, 1.0, 2.0, 4.0, -1.0, -2.0};
        for (double o : offs) {
            double w = 2 * M_PI * (cfg.cw_mhz + o) / fs;
            cd h = std::polar(1.0, -w * SMI_REF);
            for (int k = 0; k < 16; k++) if (k != SMI_REF) h -= cd(r.wre[k] / 4096.0, r.wim[k] / 4096.0) * std::polar(1.0, -w * k);
            std::printf(" %+.2f:%.1f", o, 10 * std::log10(std::norm(h) + 1e-12));
        }
        std::printf("\n");
    }
    const int e0 = 3 * AJ_NSAMP;
    auto seg = [&](const cvec &v) { return cvec(v.begin() + e0, v.end()); };
    int shift = (int)(cut_db / 6.0); if (shift > SMI_MAXSH) shift = SMI_MAXSH; if (r.res_db > 3.0) shift = 0;
    double sc4 = std::pow(4.0, shift);
    set_weights(r.wre, r.wim);
    set_g2((int)std::floor(g2 * 8192 + 0.5), 0);
    set_cfg(0, shift, r.ws);
    cvec yj = run_core_c(seg(jx1), seg(jx2));
    cvec yx = run_core_c(seg(x1), seg(x2));
    cvec yn = run_core_c(seg(nx1), seg(nx2));
    double supp = 10 * std::log10(pwr(seg(J), 0) / std::max(pwr(yj, 32) / sc4, 1e-9));
    double res  = 10 * std::log10(pwr(yx, 32) / std::max(pwr(yn, 32), 1e-9));
    double loss[3];
    cvec n1s = seg(n1);
    for (int k = 0; k < 3; k++) {
        cvec ref = seg(s1[k]);
        auto snr = [&](const cvec &y, int dly) {
            cd c = 0; double pr = 0, py = 0; int cnt = 0;
            for (size_t i = 32; i < y.size(); i++) {
                cd rr = ref[i - dly];
                c += y[i] * std::conj(rr); pr += std::norm(rr); py += std::norm(y[i]); cnt++;
            }
            double ps = std::norm(c) / pr;
            return (ps / cnt) / (py / cnt - ps / cnt);
        };
        cvec clean(ref.size());
        for (size_t i = 0; i < ref.size(); i++) clean[i] = n1s[i] / g + ref[i] / g;
        // the other satellites are just part of the "noise" for this measurement
        loss[k] = 10 * std::log10(snr(clean, 0) / snr(yx, SMI_REF));
    }
    out.supp = supp; out.res = res; out.cut = cut_db; out.ws = r.ws; out.ok = true;
    for (int k = 0; k < 3; k++) out.loss[k] = loss[k];
    if (verbose) {
        std::printf("J/N %2.0f dB: gain cut %4.1f dB, suppression %5.1f dB, residual %+5.2f dB | satellite SNR loss  A(on CW) %5.2f  B(+4 MHz) %5.2f  C(-21 MHz) %5.2f dB",
                    jnr_db, cut_db, supp, res, loss[0], loss[1], loss[2]);
        std::printf("  [firmware: J/N %.1f, foff %+.3f MHz, |w|^2 %.2f, weight scale 2^%d]\n", r.jn_db, r.foff * fs, r.wn2, r.ws);
    }
    return out;
}

int main(int argc, char **argv) {
    CwCfg cfg = {61.44, -7.06, true, 0.0, 0};
    std::vector<double> jn;
    int seeds = 1;
    for (int i = 1; i < argc; i++) {
        if (!std::strcmp(argv[i], "-noiq")) cfg.iq = false;
        else if (!std::strcmp(argv[i], "-cw") && i + 1 < argc) cfg.cw_mhz = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "-seeds") && i + 1 < argc) seeds = std::atoi(argv[++i]);
        else jn.push_back(std::atof(argv[i]));
    }
    if (jn.empty()) jn = {40, 50, 55, 60, 65, 70};
    std::printf("CW at %+.2f MHz from the LO, fs %.2f MS/s, I/Q imbalance %s, %d seed(s)\n", cfg.cw_mhz, cfg.fs_mhz, cfg.iq ? "on (~49 dB image rejection)" : "off", seeds);
    for (double j : jn) {
        if (seeds == 1) { run_cw(j, cfg, true); continue; }
        double sres = 0, sres_max = -99, ssup = 0, sl[3] = {0, 0, 0}, worstA = -99; int n = 0, bad = 0;
        for (int sd = 0; sd < seeds; sd++) {
            rng_s = 88172645463325252ULL + 7919ULL * (sd + 1);
            Res q = run_cw(j, cfg, false);
            if (!q.ok) continue;
            n++; sres += q.res; ssup += q.supp; if (q.res > sres_max) sres_max = q.res;
            for (int k = 0; k < 3; k++) sl[k] += q.loss[k];
            if (q.loss[0] > worstA) worstA = q.loss[0];
            if (q.res > 3.0) bad++;
        }
        std::printf("J/N %2.0f dB: mean suppression %5.1f dB, mean residual %+5.2f dB (worst %+5.2f, %d of %d seeds above +3 dB) | mean satellite SNR loss  A(on CW) %5.2f  B(+4 MHz) %5.2f  C(-21 MHz) %5.2f dB, worst A %5.2f\n",
                    j, ssup / n, sres / n, sres_max, bad, n, sl[0] / n, sl[1] / n, sl[2] / n, worstA);
    }
    return 0;
}
