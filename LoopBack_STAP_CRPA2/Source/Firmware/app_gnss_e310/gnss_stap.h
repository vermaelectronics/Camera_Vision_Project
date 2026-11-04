/******************************************************************************
 *  gnss_stap.h
 *  Driver for the CRPA anti-jam stage (gnss_stap, Source/HDL/gnss_stap.v).
 *  Original work for the ANTSDR E310 V1 GNSS CRPA pipeline.
 *
 *      AD9361 RX1/RX2 -> gnss_stap (PL) -> gnss_passthrough (PL) -> TX1
 *
 *  The PL estimates the 32x32 space-time covariance of RX1/RX2; this driver
 *  reads it, solves the weights on the ARM (stap_solver.c), writes them back
 *  and switches the PL output from bypass to the beamformer.
 *
 *  The register map here MUST stay in step with Source/HDL/gnss_stap.v and
 *  Source/HDL/stap/stap_top.v.
 *****************************************************************************/
#ifndef GNSS_STAP_H_
#define GNSS_STAP_H_

#include <stdint.h>
#include "app_config.h"
#include "stap_solver.h"

#ifdef XILINX_PLATFORM
#include <xparameters.h>
#endif

#if defined(XPAR_GNSS_STAP_0_BASEADDR)
#define GNSS_STAP_BASEADDR  XPAR_GNSS_STAP_0_BASEADDR
#elif defined(XPAR_GNSS_STAP_BASEADDR)
#define GNSS_STAP_BASEADDR  XPAR_GNSS_STAP_BASEADDR
#elif defined(XPAR_GNSS_STAP_S_AXI_BASEADDR)
#define GNSS_STAP_BASEADDR  XPAR_GNSS_STAP_S_AXI_BASEADDR
#else
#warning "gnss_stap base address not found in xparameters.h; using the project-assigned 0x43C10000. Verify this against the generated XSA."
#define GNSS_STAP_BASEADDR  0x43C10000U
#endif

/* ---- stap_top registers (through the l_clk CDC bridge) ------------------ */
#define GNSS_STAP_REG_ID         0x0000U   /* 0x53544150 "STAP"            */
#define GNSS_STAP_REG_CONFIG     0x0004U   /* [7:0]M [15:8]N [23:16]DW [31:24]WW */
#define GNSS_STAP_REG_CONFIG2    0x0008U   /* [7:0]FOLD [15:8]OW [23:16]CW [31:24]LMAX_LOG2 */
#define GNSS_STAP_REG_CONFIG3    0x000CU   /* [7:0]LAGP                    */
#define GNSS_STAP_REG_CTRL       0x0010U   /* pulses                       */
#define GNSS_STAP_REG_STATUS     0x0014U
#define GNSS_STAP_REG_CORR_LEN   0x0018U
#define GNSS_STAP_REG_OUT_SHIFT  0x001CU
#define GNSS_STAP_REG_RAW_BASE   0x0020U
#define GNSS_STAP_REG_USER_CTRL  0x0024U
#define GNSS_STAP_RAW_WINDOW     0x0100U   /* + 4*(64*m + i)               */
#define GNSS_STAP_WEIGHT_BASE    0x1000U   /* + 8*t (re), + 8*t + 4 (im)   */
#define GNSS_STAP_CORR_BASE      0x8000U   /* + 16*e                       */
/* ---- wrapper registers (sys_cpu_clk, always answer) ---------------------- */
#define GNSS_STAP_REG_WRAP_ID    0xF000U   /* 0x47535450 "GSTP"            */
#define GNSS_STAP_REG_WRAP_VER   0xF004U
#define GNSS_STAP_REG_WRAP_STAT  0xF008U   /* [0] l_clk alive, [31:16] bridge timeouts */
#define GNSS_STAP_REG_SMP_COUNT  0xF00CU

#define GNSS_STAP_ID             0x53544150U
#define GNSS_STAP_WRAP_ID        0x47535450U
#define GNSS_STAP_TIMEOUT_DATA   0xDEADC10CU

#define GNSS_STAP_CTRL_CORR_START  (1U << 0)
#define GNSS_STAP_CTRL_W_COMMIT    (1U << 1)
#define GNSS_STAP_CTRL_CLR_OVERRUN (1U << 2)
#define GNSS_STAP_ST_BUSY          (1U << 0)
#define GNSS_STAP_ST_DONE          (1U << 1)
#define GNSS_STAP_ST_PENDING       (1U << 2)
#define GNSS_STAP_ST_OVERRUN       (1U << 3)
#define GNSS_STAP_UC_AJ_EN         (1U << 0)   /* 1 = beamformer output     */
#define GNSS_STAP_UC_CH1_COPY      (1U << 1)   /* ch1 = beamformer output   */

#define GNSS_STAP_OK               0
#define GNSS_STAP_ERR_ABSENT      -1   /* wrapper not in the bitstream       */
#define GNSS_STAP_ERR_NO_CLK      -2   /* AD9361 l_clk not running           */
#define GNSS_STAP_ERR_CONFIG      -3   /* RTL M/N differ from stap_config.h  */
#define GNSS_STAP_ERR_TIMEOUT     -4   /* covariance engine did not finish   */
#define GNSS_STAP_ERR_SOLVE       -5   /* covariance not positive definite   */
#define GNSS_STAP_ERR_NO_WEIGHTS  -6   /* enable refused before first update */

/* background adaptation period used by the SD-card deployment build */
#ifndef GNSS_DEPLOY_STAP_PERIOD_MS
#define GNSS_DEPLOY_STAP_PERIOD_MS  100U
#endif

/* operating mode of the weight solver */
#define GNSS_STAP_MODE_PI          0   /* power inversion: no geometry needed */
#define GNSS_STAP_MODE_MVDR        1   /* needs spacing + look direction       */

typedef struct {
    int       present;          /* probe succeeded                          */
    int       mode;             /* GNSS_STAP_MODE_*                         */
    double    theta_deg;        /* MVDR look direction                      */
    double    d_over_lambda;    /* MVDR element spacing                     */
    double    fif_over_fs;      /* signal offset from LO / fs (0: baseband) */
    uint32_t  corr_len;         /* snapshots per estimate                   */
    int       headroom_bits;    /* output RMS headroom below 12-bit FS      */
    uint32_t  auto_period_ms;   /* 0 = no background adaptation             */
    int       ch1_copy;         /* TX2 carries the beamformer too           */
    /* results of the last update */
    int       have_weights;
    uint32_t  updates, errors;
    int       weight_exp, out_shift;
    int32_t   in_power_cdb[STAP_M];   /* input power per element, centi-dBFS   */
    int32_t   out_power_cdb;          /* predicted output power, centi-dBFS    */
    int32_t   suppression_cdb;        /* element 0 input / output, centi-dB    */
} gnss_stap_state_t;

extern gnss_stap_state_t gnss_stap_st;

uint32_t gnss_stap_read (uint32_t offset);
void     gnss_stap_write(uint32_t offset, uint32_t value);

/* Checks the wrapper, the l_clk, the core ID and that M/N match. */
int32_t  gnss_stap_probe(void);
/* One complete adaptation, blocking (< 10 ms). */
int32_t  gnss_stap_update(void);
/* Non-blocking background adaptation; call from any idle loop. */
void     gnss_stap_service(void);
/* 1 = TX gets the beamformer output, 0 = bypass (raw RX1). */
int32_t  gnss_stap_set_enable(int enable);
int      gnss_stap_get_enable(void);
/* Clears the sticky STATUS.overrun flag (set e.g. while the AD9361 interface
 * is being tuned at boot or retuned by gnss_band=). */
void     gnss_stap_clear_overrun(void);
void     gnss_stap_print_state(void);

/* console commands (command.c table) */
void get_stap_status(double *param, char param_no);
void get_stap_en    (double *param, char param_no);
void set_stap_en    (double *param, char param_no);
void set_stap_update(double *param, char param_no);
void get_stap_auto  (double *param, char param_no);
void set_stap_auto  (double *param, char param_no);
void set_stap_mode  (double *param, char param_no);
void set_stap_theta (double *param, char param_no);
void set_stap_len   (double *param, char param_no);
void set_stap_ch1   (double *param, char param_no);

#endif /* GNSS_STAP_H_ */
