/*
 * stap_driver.c -- bare-metal / Linux-UIO driver loop for stap_top
 *
 * Call stap_update() periodically (e.g. every 1..10 ms, or from the corr_done
 * interrupt). It
 *   1. triggers a covariance capture,
 *   2. waits for the engine,
 *   3. reads the 868 correlation results + 2x30 edge samples per channel,
 *   4. solves the MVDR (or power-inversion) weights,
 *   5. writes + commits them (the core switches on a sample boundary).
 *
 * 'base' is the AXI-Lite base address of stap_top (from the Vivado address
 * editor, or mmap() of /dev/uioX on Linux).
 */
#include <stdint.h>
#include "stap_solver.h"

#define STAP_REG_ID         0x0000u
#define STAP_REG_CTRL       0x0010u
#define STAP_REG_STATUS     0x0014u
#define STAP_REG_CORR_LEN   0x0018u
#define STAP_REG_OUT_SHIFT  0x001Cu
#define STAP_REG_RAW_BASE   0x0020u
#define STAP_RAW_WINDOW     0x0100u
#define STAP_WEIGHT_BASE    0x1000u
#define STAP_CORR_BASE      0x8000u

#define STAP_CTRL_CORR_START   (1u << 0)
#define STAP_CTRL_W_COMMIT     (1u << 1)
#define STAP_CTRL_CLR_OVERRUN  (1u << 2)
#define STAP_STAT_BUSY         (1u << 0)
#define STAP_STAT_DONE         (1u << 1)
#define STAP_STAT_PENDING      (1u << 2)

typedef struct {
    volatile uint32_t *base;      /* AXI-Lite base                               */
    uint32_t L;                   /* snapshots per estimate (<= 8162)             */
    int      use_power_inversion; /* 1: S=[1 0 ..], 0: MVDR towards theta_deg     */
    double   theta_deg;           /* look direction                              */
    double   d_over_lambda;       /* 0.5                                         */
    double   fif_over_fs;         /* IF / fs  (10/60)                            */
    int      ww, ow;              /* weight / output widths of the core (18, 16) */
    int      headroom_bits;       /* output back-off (3 -> -18 dBFS RMS)         */
} stap_dev;

static inline void     wr32(stap_dev *d, uint32_t off, uint32_t v) { d->base[off >> 2] = v; }
static inline uint32_t rd32(stap_dev *d, uint32_t off)             { return d->base[off >> 2]; }

static int64_t   g_cre[STAP_NRES], g_cim[STAP_NRES];
static stap_cplx g_R[STAP_T][STAP_T];
static stap_edges g_ed;

static void read_edges(stap_dev *d, uint32_t base, int32_t re[STAP_M][STAP_HT], int32_t im[STAP_M][STAP_HT])
{
    int m, i;
    wr32(d, STAP_REG_RAW_BASE, base);
    for (m = 0; m < STAP_M; m++)
        for (i = 0; i < STAP_HT; i++) {
            uint32_t v = rd32(d, STAP_RAW_WINDOW + 4u * (uint32_t)(64 * m + i));
            re[m][i] = (int16_t)(v >> 16);
            im[m][i] = (int16_t)(v & 0xFFFFu);
        }
}

int stap_init(stap_dev *d)
{
    if (rd32(d, STAP_REG_ID) != 0x53544150u) return -1;
    wr32(d, STAP_REG_CORR_LEN, d->L);
    return 0;
}

int stap_update(stap_dev *d)
{
    stap_cplx S[STAP_T], w[STAP_T];
    int32_t   wr[STAP_T], wi[STAP_T];
    int       e, t, sh;
    uint32_t  timeout = 100000000u;

    /* 1+2: capture and correlate */
    wr32(d, STAP_REG_CTRL, STAP_CTRL_CORR_START);
    while (!(rd32(d, STAP_REG_STATUS) & STAP_STAT_DONE))
        if (--timeout == 0) return -1;

    /* 3: read results (64-bit sign-extended per component) */
    for (t = 0; t < STAP_NRES; t++) {
        uint32_t o = STAP_CORR_BASE + 16u * (uint32_t)t;
        g_cre[t] = (int64_t)(((uint64_t)rd32(d, o + 4) << 32) | rd32(d, o + 0));
        g_cim[t] = (int64_t)(((uint64_t)rd32(d, o + 12) << 32) | rd32(d, o + 8));
    }

    read_edges(d, 0,    g_ed.head_re, g_ed.head_im);
    read_edges(d, d->L, g_ed.tail_re, g_ed.tail_im);

    /* 4: solve */
    stap_build_cov(g_cre, g_cim, &g_ed, d->L, g_R);
    if (d->use_power_inversion) stap_steering_pi(S);
    else stap_steering(d->d_over_lambda, d->theta_deg, d->fif_over_fs, S);
    if (stap_solve_mvdr(g_R, S, 1e-9, w) != 0) return -2;
    e  = stap_quantize(w, d->ww, wr, wi);
    sh = stap_out_shift(g_R, w, e, d->ow, d->headroom_bits);

    /* 5: write shadow bank + commit */
    for (t = 0; t < STAP_T; t++) {
        wr32(d, STAP_WEIGHT_BASE + 8u * (uint32_t)t,      (uint32_t)wr[t]);
        wr32(d, STAP_WEIGHT_BASE + 8u * (uint32_t)t + 4u, (uint32_t)wi[t]);
    }
    wr32(d, STAP_REG_OUT_SHIFT, (uint32_t)sh);
    wr32(d, STAP_REG_CTRL, STAP_CTRL_W_COMMIT);
    return 0;
}
