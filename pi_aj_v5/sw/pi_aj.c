/*
 * pi_aj.c -- PI-AJ v5 anti-jam core driver (Vitis / bare-metal, Zynq-7000)
 *
 * Hook pi_aj_print() to a terminal command (e.g. "aj_status?") next to your
 * existing gnss_status? handler. All numbers it prints map 1:1 onto the
 * CRPA field-test sheet (RMS suppression, weight magnitude/phase).
 */
#include <math.h>
#include <stdio.h>
#include "xil_io.h"
#include "sleep.h"
#include "pi_aj.h"

static inline uint32_t rd(uintptr_t b, uint32_t o) { return Xil_In32(b + o); }
static inline void wr(uintptr_t b, uint32_t o, uint32_t v) { Xil_Out32(b + o, v); }

static double db(double v) { return v > 0.0 ? 10.0 * log10(v) : -999.0; }

static double coef_hz(uint32_t v, double fs)
{
    double re = (double)(int16_t)(v & 0xFFFF);
    double im = (double)(int16_t)(v >> 16);
    return atan2(im, re) / (2.0 * M_PI) * fs;
}

int pi_aj_init(uintptr_t base)
{
    if (rd(base, PI_AJ_ID) != 0x50494A35u)
        return -1;
    wr(base, PI_AJ_CONTROL, PI_AJ_CONTROL_DEFAULT);
    wr(base, PI_AJ_ADAPT,   0x0008C600u);     /* mu 0 (+1 auto on CW), rho 6, r 12, a 8 */
    wr(base, PI_AJ_LEVELS,  0x0B1007F8u);     /* clip 2040, hold 16, target 2^11 */
    pi_aj_reset_weights(base);
    return 0;
}

void pi_aj_set_control(uintptr_t base, uint32_t ctrl) { wr(base, PI_AJ_CONTROL, ctrl); }

void pi_aj_set_mu(uintptr_t base, unsigned mu_sh)
{
    uint32_t v = rd(base, PI_AJ_ADAPT);
    wr(base, PI_AJ_ADAPT, (v & ~0xFu) | (mu_sh & 0xFu));
}

void pi_aj_set_notch(uintptr_t base, unsigned rho_sh, unsigned r_sh, unsigned a_sh)
{
    uint32_t v = rd(base, PI_AJ_ADAPT) & ~0x000FFF00u;
    v |= ((rho_sh & 0xF) << 8) | ((r_sh & 0xF) << 12) | ((a_sh & 0xF) << 16);
    wr(base, PI_AJ_ADAPT, v);
}

void pi_aj_set_levels(uintptr_t base, unsigned clip_thr, unsigned blank_hold, unsigned agc_target)
{
    wr(base, PI_AJ_LEVELS, (clip_thr & 0xFFFF) | ((blank_hold & 0xFF) << 16) |
                           ((agc_target & 0xF) << 24));
}

void pi_aj_reset_weights(uintptr_t base) { wr(base, PI_AJ_CMD, 1); }

void pi_aj_read(uintptr_t base, double fs, pi_aj_state_t *s)
{
    s->status     = rd(base, PI_AJ_STATUS);
    s->p_rx1_db   = db(rd(base, PI_AJ_P_RX1) / 256.0);    /* dB re 1 ADC LSB^2 */
    s->p_rx2_db   = db(rd(base, PI_AJ_P_RX2) / 256.0);
    s->p_notch_db = db(rd(base, PI_AJ_P_NOTCH) / 256.0);
    s->p_out_db   = db(rd(base, PI_AJ_P_OUT) / 256.0);
    s->supp_notch_db = s->p_rx1_db - s->p_notch_db;
    s->supp_total_db = s->p_rx1_db - s->p_out_db;
    s->notch1_hz  = coef_hz(rd(base, PI_AJ_NOTCH1_A), fs);
    s->notch2_hz  = coef_hz(rd(base, PI_AJ_NOTCH2_A), fs);
    s->clipcnt    = rd(base, PI_AJ_CLIPCNT);
    s->agc_shift  = (int)((s->status >> 8) & 0x1F);
    if (s->agc_shift > 15) s->agc_shift -= 32;
}

void pi_aj_read_weight(uintptr_t base, unsigned idx, double *re, double *im)
{
    wr(base, PI_AJ_WSEL, idx);
    usleep(5);                                   /* CDC snapshot refresh */
    *re = (int32_t)rd(base, PI_AJ_W_RE) / 268435456.0;
    *im = (int32_t)rd(base, PI_AJ_W_IM) / 268435456.0;
}

void pi_aj_print(uintptr_t base, double fs)
{
    pi_aj_state_t s;
    double wr_, wi_;
    pi_aj_read(base, fs, &s);
    printf("\r\n--- PI-AJ v5 (notch + widely-linear space-time power inversion) ---\r\n");
    printf(" notch1 / image / notch2 : %s / %s / %s\r\n",
           (s.status & 1) ? "ON" : "off", (s.status & 2) ? "ON" : "off",
           (s.status & 4) ? "ON" : "off");
    printf(" jammer line (notch1)    : %+.1f kHz from LO\r\n", s.notch1_hz / 1e3);
    printf(" notch2 line             : %+.1f kHz from LO\r\n", s.notch2_hz / 1e3);
    printf(" RX1 / RX2 power         : %.1f / %.1f dB(LSB^2)\r\n", s.p_rx1_db, s.p_rx2_db);
    printf(" after notches           : %.1f dB  (%.1f dB removed)\r\n", s.p_notch_db, s.supp_notch_db);
    printf(" PI output               : %.1f dB\r\n", s.p_out_db);
    printf(" RMS suppression total   : %.1f dB\r\n", s.supp_total_db);
    printf(" ADC clipped samples     : %lu / 1048576 %s\r\n", (unsigned long)s.clipcnt,
           (s.status & 8) ? " <-- OVERLOAD: lower gnss_rx_gain" : "");
    printf(" TX AGC shift            : %+d (x%d dB)\r\n", s.agc_shift, 6 * s.agc_shift);
    if (s.status & 0x30)
        printf(" WARNING: PI %s%s -> clk/fs too low for the lane count\r\n",
               (s.status & 0x10) ? "late " : "", (s.status & 0x20) ? "overrun" : "");
    pi_aj_read_weight(base, 6, &wr_, &wi_);      /* RX2 centre tap (index L-1+D) */
    printf(" RX2 centre-tap weight   : %.4f %+.4fj  |w| %.3f  phase %.1f deg\r\n",
           wr_, wi_, sqrt(wr_ * wr_ + wi_ * wi_), atan2(wi_, wr_) * 180.0 / M_PI);
    printf("--------------------------------------------------------------\r\n");
}
