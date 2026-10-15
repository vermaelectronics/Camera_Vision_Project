/******************************************************************************
 *  gnss_doa.h
 *  Driver for the ESPRIT direction-of-arrival monitor inside gnss_passthrough
 *  (v1.2, registers 0x100..0x194 of the gnss_passthrough aperture).
 *
 *  Hardware : Source/HDL/gnss_doa_esprit.v  (I-UESPRIT + M-ESPRIT)
 *  Algorithm: J. Li et al., "Modified ESPRIT and Unitary-Transformation Form
 *             for Computationally Efficient DOA Estimation on FPGA",
 *             IEEE Trans. Instrum. Meas., vol. 74, 2025, Art. 2003209.
 *  Docs     : Docs/ESPRIT/README_ESPRIT.md
 *
 *  The register map here MUST stay in step with gnss_doa_esprit.v and
 *  Source/Config/board_e310_v1.json.
 *
 *  Original work for the ANTSDR E310 V1 GNSS CRPA project.
 *****************************************************************************/
#ifndef GNSS_DOA_H_
#define GNSS_DOA_H_

#include <stdint.h>
#include "gnss_passthrough.h"      /* GNSS_PT_BASEADDR, gnss_pt_read/write */

/* ---- register offsets (from GNSS_PT_BASEADDR) ---------------------------- */
#define DOA_REG_ID          0x100U  /* RO 0x444F4131 "DOA1"                  */
#define DOA_REG_VERSION     0x104U  /* RO                                   */
#define DOA_REG_CTRL        0x108U  /* RW see DOA_CTRL_*                    */
#define DOA_REG_MU          0x10CU  /* RW regulariser, Q30                  */
#define DOA_REG_K           0x110U  /* RW lambda/(2 pi d), Q2.30            */
#define DOA_REG_UCTOL       0x114U  /* RW unit-circle tolerance, Q8.24      */
#define DOA_REG_DECIM       0x118U  /* RW keep 1 of (N+1) snapshots         */
#define DOA_REG_CAL         0x11CU  /* RW {sin_q15, cos_q15} for element 1  */
#define DOA_REG_TG_FTW      0x120U  /* RW test tone frequency word          */
#define DOA_REG_TG_DPHASE   0x124U  /* RW test inter-element phase (2^32=360)*/
#define DOA_REG_TG_CFG      0x128U  /* RW [15:0] div [19:16] amp [23:20] noise */
#define DOA_REG_STATUS      0x12CU  /* RO                                   */
#define DOA_REG_IN_COUNT    0x130U  /* RO snapshots delivered               */
#define DOA_REG_PEAK        0x134U  /* RO {peak1, peak0} Q1.15              */
#define DOA_REG_I_SEQ       0x138U
#define DOA_REG_I_PHI       0x13CU  /* Q28 rad                              */
#define DOA_REG_I_PSI       0x140U  /* Q8.24, tan(phi/2)                    */
#define DOA_REG_I_THETA     0x144U  /* Q28 rad                              */
#define DOA_REG_I_DEG       0x148U  /* Q16 deg                              */
#define DOA_REG_I_AVG_SEQ   0x14CU
#define DOA_REG_I_AVG_DEG   0x150U  /* Q16 deg                              */
#define DOA_REG_I_AVG_SQ_LO 0x154U  /* deg^2 Q32                            */
#define DOA_REG_I_AVG_SQ_HI 0x158U
#define DOA_REG_M_SEQ       0x15CU
#define DOA_REG_M_PHI       0x160U
#define DOA_REG_M_PSIMAG    0x164U  /* Q8.24, |psi| ~ 1 for a real source   */
#define DOA_REG_M_THETA     0x168U
#define DOA_REG_M_DEG       0x16CU
#define DOA_REG_M_AVG_SEQ   0x170U
#define DOA_REG_M_AVG_DEG   0x174U
#define DOA_REG_M_AVG_SQ_LO 0x178U
#define DOA_REG_M_AVG_SQ_HI 0x17CU
#define DOA_REG_M_OFFCIRC   0x180U
#define DOA_REG_C00         0x184U
#define DOA_REG_C11         0x188U
#define DOA_REG_C01_RE      0x18CU
#define DOA_REG_C01_IM      0x190U
#define DOA_REG_CAPS        0x194U  /* RO [0] I [1] M [2] HLS IP [15:8] max log2 */

#define DOA_EXPECTED_ID     0x444F4131U

/* ---- DOA_CTRL fields ------------------------------------------------------ */
#define DOA_CTRL_ENABLE      (1U << 0)
#define DOA_CTRL_SRC_TEST    (1U << 1)
#define DOA_CTRL_USE_COV     (1U << 2)
#define DOA_CTRL_TG_NOISE    (1U << 3)
#define DOA_CTRL_P(n)        (((uint32_t)(n) & 0x3U)  << 4)
#define DOA_CTRL_SNAP(n)     (((uint32_t)(n) & 0x1FU) << 8)
#define DOA_CTRL_AVG(n)      (((uint32_t)(n) & 0xFU)  << 16)
#define DOA_CTRL_SOFT_RESET  (1U << 24)
#define DOA_CTRL_CLR_STATS   (1U << 25)

/* ---- return codes --------------------------------------------------------- */
#define DOA_OK               0
#define DOA_ERR_ID          -1
#define DOA_ERR_TIMEOUT     -2
#define DOA_ERR_SELFTEST    -3
#define DOA_ERR_NO_INPUT    -4
#define DOA_ERR_INCOHERENT  -5      /* calibration refused: RX1/RX2 not the same signal */

/* ---- phase / jammer verdict (doa_phase?, doa_watch=) ----------------------- */
#define DOA_VERDICT_NO_DATA    0    /* no estimator results (DOA off / RX idle)  */
#define DOA_VERDICT_NO_SIGNAL  1    /* all-zero input                            */
#define DOA_VERDICT_NOISE      2    /* noise only: no jammer (satellites are below noise) */
#define DOA_VERDICT_UNCERTAIN  3    /* partly coherent / unequal gains / 2 sources */
#define DOA_VERDICT_SOURCE     4    /* one strong coherent source = jammer / test signal */

/* ---- one estimator's view ------------------------------------------------- */
typedef struct {
    uint32_t seq;
    int32_t  phi_q28;
    int32_t  psi_q24;         /* I: tan(phi/2)   M: |psi|                  */
    int32_t  deg_q16;
    uint32_t avg_seq;
    int32_t  avg_deg_q16;
    int32_t  std_mdeg;        /* standard deviation of the averaged block  */
    uint32_t flags;
} doa_est_t;

typedef struct {
    uint32_t  ctrl, status, in_count, peak;
    doa_est_t i;              /* I-UESPRIT */
    doa_est_t m;              /* M-ESPRIT  */
    uint32_t  m_offcirc;
    int       has_i, has_m;
} doa_state_t;

/* ---- API ------------------------------------------------------------------ */
int32_t gnss_doa_probe(void);                        /* ID + capabilities      */
void    gnss_doa_defaults(void);                     /* p=4, 1024, avg 16, ... */
void    gnss_doa_enable_live(int enable);            /* RX1/RX2 -> estimators  */
void    gnss_doa_set_window(uint32_t log2_snap);     /* 7..14                  */
void    gnss_doa_set_p(uint32_t p_log2);             /* 0..3 -> p = 1,2,4,8    */
void    gnss_doa_set_avg(uint32_t avg_log2);         /* 0..10                  */
void    gnss_doa_set_cov(int use_cov);
void    gnss_doa_set_decim(uint32_t n);
void    gnss_doa_set_spacing(double spacing_m, double freq_hz);
void    gnss_doa_get_state(doa_state_t *st);
void    gnss_doa_print_result(void);                 /* "doa?"                 */
void    gnss_doa_print_regs(void);                   /* "doa_regs?"            */
void    gnss_doa_print_status(void);                 /* "doa_status?"          */
int32_t gnss_doa_monitor(uint32_t n);                /* stream N averages      */
int32_t gnss_doa_test_angle(double theta_deg);       /* test generator, 1 angle*/
int32_t gnss_doa_selftest(int verbose);              /* full PASS/FAIL sweep   */
int32_t gnss_doa_calibrate(void);                    /* zero the RX1/RX2 phase */
void    gnss_doa_cal_reset(void);
void    gnss_doa_print_cal(void);                    /* "doa_cal?"             */
int32_t gnss_doa_print_phase(uint32_t nwin);         /* "doa_phase?" -> verdict */
int32_t gnss_doa_watch(uint32_t n);                  /* "doa_watch=N"          */
void    gnss_doa_set_expect(double theta_deg);       /* "doa_expect=<deg>"     */
int32_t gnss_doa_boot_check(void);                   /* probe + quick selftest */

/* fixed-point helpers (no libm) */
int32_t gnss_doa_sin_q15(int32_t ang_q28);
int32_t gnss_doa_cos_q15(int32_t ang_q28);
int32_t gnss_doa_atan2_q28(int32_t y, int32_t x);

#endif /* GNSS_DOA_H_ */
