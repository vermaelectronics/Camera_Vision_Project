/*
 * pi_aj.h -- PI-AJ v5 anti-jam core driver (Vitis / bare-metal, Zynq-7000)
 */
#ifndef PI_AJ_H
#define PI_AJ_H

#include <stdint.h>

#define PI_AJ_ID          0x00
#define PI_AJ_VERSION     0x04
#define PI_AJ_CONTROL     0x08
#define PI_AJ_ADAPT       0x0C
#define PI_AJ_LEVELS      0x10
#define PI_AJ_WSEL        0x14
#define PI_AJ_CMD         0x18
#define PI_AJ_STATUS      0x20
#define PI_AJ_P_RX1       0x24
#define PI_AJ_P_RX2       0x28
#define PI_AJ_P_NOTCH     0x2C
#define PI_AJ_P_OUT       0x30
#define PI_AJ_NOTCH1_A    0x34
#define PI_AJ_NOTCH2_A    0x38
#define PI_AJ_CLIPCNT     0x3C
#define PI_AJ_W_RE        0x40
#define PI_AJ_W_IM        0x44
#define PI_AJ_SAMPLES     0x48

/* CONTROL bits */
#define PI_AJ_CORE_EN     (1u << 0)
#define PI_AJ_NOTCH_EN    (1u << 1)
#define PI_AJ_NOTCH2_EN   (1u << 2)
#define PI_AJ_IMAGE_EN    (1u << 3)
#define PI_AJ_PI_EN       (1u << 4)
#define PI_AJ_AGC_EN      (1u << 5)
#define PI_AJ_DCBLK_EN    (1u << 6)
#define PI_AJ_BLANK_EN    (1u << 7)
#define PI_AJ_FREEZE      (1u << 8)
#define PI_AJ_NOTCH_FORCE (1u << 9)
#define PI_AJ_AUTO_MU     (1u << 10)
#define PI_AJ_CONTROL_DEFAULT 0x43Bu   /* core,notch1,image,PI,AGC,auto_mu */

#define PI_AJ_NUM_WEIGHTS 19   /* L=5, widely linear: 4L-1 */

typedef struct {
    uint32_t status;
    double   p_rx1_db, p_rx2_db, p_notch_db, p_out_db;
    double   supp_notch_db, supp_total_db;
    double   notch1_hz, notch2_hz;
    uint32_t clipcnt;
    int      agc_shift;
} pi_aj_state_t;

int  pi_aj_init(uintptr_t base);                       /* 0 = ok, -1 = core not found */
void pi_aj_set_control(uintptr_t base, uint32_t ctrl);
void pi_aj_set_mu(uintptr_t base, unsigned mu_sh);    /* PI step 2^-mu, 0..15 */
void pi_aj_set_notch(uintptr_t base, unsigned rho_sh, unsigned r_sh, unsigned a_sh);
void pi_aj_set_levels(uintptr_t base, unsigned clip_thr, unsigned blank_hold, unsigned agc_target);
void pi_aj_reset_weights(uintptr_t base);
void pi_aj_read(uintptr_t base, double fs_hz, pi_aj_state_t *s);
void pi_aj_read_weight(uintptr_t base, unsigned idx, double *re, double *im);
void pi_aj_print(uintptr_t base, double fs_hz);       /* terminal report */

#endif
