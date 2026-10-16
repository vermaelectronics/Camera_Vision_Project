/******************************************************************************
 *  gnss_passthrough.h
 *  Driver for the project's custom FPGA processing block.
 *
 *  Original work for the ANTSDR E310 V1 GNSS CRPA project.  Not derived from
 *  Analog Devices or MicroPhase source.
 *
 *  The register map here MUST stay in step with:
 *    Source/HDL/gnss_passthrough.v          (the hardware)
 *    Source/Config/board_e310_v1.json       (the shared description)
 *  Change one, change all three.
 *****************************************************************************/
#ifndef GNSS_PASSTHROUGH_H_
#define GNSS_PASSTHROUGH_H_

#include <stdint.h>

/* --------------------------------------------------------------------------
 * Base address.
 *
 * Prefer the value Vitis generates from the block design.  Fall back to the
 * address this project assigns in board_e310_v1.json only if xparameters.h
 * does not define it -- and say so loudly at compile time, because a silent
 * fallback would be exactly the kind of guess requirement 32 forbids.
 * -------------------------------------------------------------------------- */
/* app_config.h FIRST. It is what defines XILINX_PLATFORM, and without it the
 * guard below is false, xparameters.h is never included, and the fallback
 * silently wins -- which is precisely what the comment above forbids.
 *
 * That is not hypothetical: every build from 2026-09-11 to 2026-09-14 emitted
 * the #warning below and used the hard-coded address. gnss_passthrough.c
 * includes this header first and nothing had pulled in app_config.h yet, and
 * the -DXILINX_PLATFORM in build_software.py's USER_COMPILE_FLAGS was not
 * reaching the compile line. The address happened to be correct, so nothing
 * broke -- the check was simply not doing its job.
 *
 * app_config.h has its own include guard, so including it here is harmless. */
#include "app_config.h"

#ifdef XILINX_PLATFORM
#include <xparameters.h>
#endif

#if defined(XPAR_GNSS_PASSTHROUGH_0_BASEADDR)
#define GNSS_PT_BASEADDR   XPAR_GNSS_PASSTHROUGH_0_BASEADDR
#elif defined(XPAR_GNSS_PASSTHROUGH_BASEADDR)
#define GNSS_PT_BASEADDR   XPAR_GNSS_PASSTHROUGH_BASEADDR
#elif defined(XPAR_GNSS_PASSTHROUGH_S_AXI_BASEADDR)
/* Classic (non-SDT) BSP naming, e.g. Vitis 2023.2 XSCT flow. */
#define GNSS_PT_BASEADDR   XPAR_GNSS_PASSTHROUGH_S_AXI_BASEADDR
#else
#warning "gnss_passthrough base address not found in xparameters.h; using the project-assigned 0x43C00000. Verify this against the generated XSA before trusting any register read."
#define GNSS_PT_BASEADDR   0x43C00000U
#endif

/* ---- register offsets ---------------------------------------------------- */
#define GNSS_PT_REG_ID              0x00U   /* RO  0x47435031 "GCP1"          */
#define GNSS_PT_REG_VERSION         0x04U   /* RO  {major,minor}              */
#define GNSS_PT_REG_SCRATCH         0x08U   /* RW                             */
#define GNSS_PT_REG_CONTROL         0x0CU   /* RW                             */
#define GNSS_PT_REG_STATUS          0x10U   /* RO                             */
#define GNSS_PT_REG_RX_COUNT_CH0    0x14U   /* RO                             */
#define GNSS_PT_REG_TX_COUNT_CH0    0x18U   /* RO                             */
#define GNSS_PT_REG_OVERFLOW_COUNT  0x1CU   /* RO                             */
#define GNSS_PT_REG_UNDERFLOW_COUNT 0x20U   /* RO                             */
#define GNSS_PT_REG_RX_SNAPSHOT_CH0 0x24U   /* RO  {Q[31:16], I[15:0]}        */
#define GNSS_PT_REG_TX_SNAPSHOT_CH0 0x28U   /* RO  {Q[31:16], I[15:0]}        */
#define GNSS_PT_REG_FIFO_LEVEL      0x2CU   /* RO  [5:0] ch0, [13:8] ch1      */
#define GNSS_PT_REG_RX_COUNT_CH1    0x30U   /* RO                             */
#define GNSS_PT_REG_TX_COUNT_CH1    0x34U   /* RO                             */
#define GNSS_PT_REG_RX_SNAPSHOT_CH1 0x38U   /* RO                             */
#define GNSS_PT_REG_CRPA_COEF(n)    (0x40U + ((n) * 4U))  /* RW, n = 0..15    */
/* v3.0 LCMV CRPA (2-element, PIER C 2008 look + enforcement constraints).
 * The LCMV registers sit in the CRPA_COEF window; 0x48 is reserved. */
#define GNSS_PT_REG_LCMV_GPS_ANGLE  0x40U   /* RW  [15:0] signed deg, look      */
#define GNSS_PT_REG_LCMV_JAM_ANGLE  0x44U   /* RW  [15:0] signed deg, null      */
#define GNSS_PT_REG_LCMV_EPS        0x4CU   /* RW  [15:0] Q0.16, 0 = ideal null */
#define GNSS_PT_REG_LCMV_RX2_CAL    0x50U   /* RW  {im,re} Q2.14, reset 1.0     */
#define GNSS_PT_REG_LCMV_W1_RE      0x54U   /* RO  Q16.16 sign-extended         */
#define GNSS_PT_REG_LCMV_W1_IM      0x58U
#define GNSS_PT_REG_LCMV_W2_RE      0x5CU   /* w2 includes the RX2 calibration  */
#define GNSS_PT_REG_LCMV_W2_IM      0x60U
#define GNSS_PT_REG_LCMV_STATUS     0x64U   /* RO  [0]en [1]wvalid [2]busy [3]sat */
#define GNSS_PT_REG_LCMV_UPDATES    0x68U   /* RO  weight sets computed         */
#define GNSS_PT_REG_LCMV_SAT_COUNT  0x6CU   /* RO  output samples clipped       */
#define GNSS_PT_LCMV_CAL_ONE        0x4000  /* 1.0 in Q2.14                     */

/* ---- expected identity --------------------------------------------------- */
#define GNSS_PT_EXPECTED_ID         0x47435031U
/* 1.0 = Phase-1 identity passthrough.
 * 1.1 = adds the RX->TX sample alignment stage (RX is right-aligned 12-in-16,
 *       the AD9361 DAC consumes [15:4]). A board reporting 1.0 is running a
 *       bitstream whose TX output is 24 dB low with 4 bits discarded. */
/* 3.0 = LCMV CRPA build: 2-element LCMV beamformer on TX1 (CONTROL[4],
 *       registers 0x40/0x44, 0x4C..0x6C). No PL-NPI / PI core. */
#define GNSS_PT_EXPECTED_VERSION    0x00030000U

/* ---- CONTROL bits -------------------------------------------------------- */
#define GNSS_PT_CTRL_PASS_EN        (1U << 0)  /* 1 = RX->TX passthrough      */
#define GNSS_PT_CTRL_MUTE           (1U << 1)  /* 1 = drive zeros to the DAC  */
#define GNSS_PT_CTRL_SWAP_IQ        (1U << 2)
#define GNSS_PT_CTRL_CH1_COPY       (1U << 3)  /* ch1 TX fed from ch0         */
#define GNSS_PT_CTRL_LCMV_EN        (1U << 4)  /* ch0 TX = LCMV(RX1, RX2)     */
#define GNSS_PT_CTRL_CNT_CLEAR      (1U << 8)  /* held, not self-clearing     */

/* ---- STATUS bits --------------------------------------------------------- */
#define GNSS_PT_ST_ADC_EN_I0        (1U << 0)
#define GNSS_PT_ST_ADC_EN_Q0        (1U << 1)
#define GNSS_PT_ST_DAC_EN_I0        (1U << 2)
#define GNSS_PT_ST_DAC_EN_Q0        (1U << 3)
#define GNSS_PT_ST_FIFO0_EMPTY      (1U << 4)
#define GNSS_PT_ST_FIFO0_FULL       (1U << 5)
#define GNSS_PT_ST_FIFO1_EMPTY      (1U << 6)
#define GNSS_PT_ST_FIFO1_FULL       (1U << 7)
#define GNSS_PT_ST_OVERFLOW         (1U << 8)
#define GNSS_PT_ST_UNDERFLOW        (1U << 9)
#define GNSS_PT_ST_PASS_EN_SYNCED   (1U << 16)
#define GNSS_PT_ST_LCMV_EN_SYNCED   (1U << 17)
#define GNSS_PT_ST_LCMV_ACTIVE      (1U << 19) /* LCMV drives ch0 TX now      */
#define GNSS_PT_ST_LCMV_SAT         (1U << 22) /* LCMV output clipped, sticky */

/* ---- return codes -------------------------------------------------------- */
#define GNSS_PT_OK                   0
#define GNSS_PT_ERR_ID              -1   /* core not present / wrong ID       */
#define GNSS_PT_ERR_NO_RX           -2   /* no RX samples arriving            */
#define GNSS_PT_ERR_NO_TX           -3   /* no TX requests arriving           */
#define GNSS_PT_ERR_SCRATCH         -4   /* register read/write failed        */
#define GNSS_PT_ERR_ALIGNMENT       -5   /* TX samples are not left-aligned   */

/* ---- API ----------------------------------------------------------------- */

uint32_t gnss_pt_read (uint32_t offset);
void     gnss_pt_write(uint32_t offset, uint32_t value);

/* Verifies the core responds with the expected ID and that SCRATCH holds a
 * written value.  Call before anything else. */
int32_t  gnss_pt_probe(void);

/* Enable or disable RX->TX passthrough.  Disabled restores the untouched
 * vendor behaviour, where the DAC is driven by the DMA/DDS path. */
void     gnss_pt_set_passthrough(int enable);
void     gnss_pt_set_mute(int mute);
void     gnss_pt_clear_counters(void);

/* LCMV CRPA (bitstream v3.0+). Enabled, channel 0 TX
 * carries y = w1^H x1 + w2^H (c x2): gain 1 toward the GPS angle, a null
 * toward the jammer angle; channel 1 TX carries RX2 as before. The jammer
 * angle comes from software: the FPGA does no DOA estimation.
 * Angles are degrees from broadside, -90..90; theta > 0 reaches RX2 first. */
void     gnss_pt_lcmv_enable(int enable);
void     gnss_pt_lcmv_set_angles(int gps_deg, int jam_deg);
/* eps = gain left toward the jammer, 0.0 .. <1.0 (0 = ideal null). */
void     gnss_pt_lcmv_set_eps(float eps);
/* RX2 gain/phase correction c, applied as w2 * conj(c). |re|,|im| < 2. */
void     gnss_pt_lcmv_set_rx2_cal(float re, float im);

typedef struct {
    int      enabled;        /* CONTROL[4]                             */
    int      active;         /* STATUS[19]: LCMV is driving TX ch0     */
    int      weights_valid;
    int      busy;
    int      sat_sticky;
    int      gps_deg, jam_deg;
    float    eps;
    float    cal_re, cal_im;
    float    w1_re, w1_im, w2_re, w2_im;
    uint32_t updates;
    uint32_t sat_count;
} gnss_pt_lcmv_state_t;

void     gnss_pt_lcmv_get_state(gnss_pt_lcmv_state_t *st);
void     gnss_pt_lcmv_print_state(void);

/* Snapshot of everything worth observing at runtime (requirement 58). */
typedef struct {
    uint32_t id;
    uint32_t version;
    uint32_t control;
    uint32_t status;
    uint32_t rx_count_ch0;
    uint32_t tx_count_ch0;
    uint32_t rx_count_ch1;
    uint32_t tx_count_ch1;
    uint32_t overflow_count;
    uint32_t underflow_count;
    uint32_t fifo_level_ch0;
    uint32_t fifo_level_ch1;
    int16_t  rx_i0, rx_q0;
    int16_t  tx_i0, tx_q0;
    int      passthrough_enabled;
    int      overflow_sticky;
    int      underflow_sticky;
} gnss_pt_state_t;

void     gnss_pt_get_state(gnss_pt_state_t *st);
void     gnss_pt_print_state(void);

/* Confirms samples are actually moving through the block by sampling the
 * counters twice, separated by delay_ms (requirement 59).
 * Returns GNSS_PT_OK only if both RX and TX counters advanced, and -- when
 * passthrough is on and unmuted -- the TX snapshot is left-aligned. */
int32_t  gnss_pt_check_dataflow(uint32_t delay_ms);

#endif /* GNSS_PASSTHROUGH_H_ */
