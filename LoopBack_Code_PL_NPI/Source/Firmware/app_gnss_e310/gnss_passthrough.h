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
/* CRPA_COEF[2] is the PL-NPI regulariser gamma (unsigned; 0 acts as 1). */
#define GNSS_PT_REG_NPI_GAMMA       GNSS_PT_REG_CRPA_COEF(2)
/* PL-NPI v2.3 (pl_npi_0 v2.0). NPI_CFG/NORM/DET_THR/DET_HOLD are SHADOW
 * registers; a write to NPI_COMMIT hands all four to the core in one clock.
 * A write to NPI_SNAP latches the core's weights, detector power and status
 * into NPI_W/NPI_DET_POW/NPI_ST_LATCHED; a read of NPI_SNAP is the live
 * status. */
#define GNSS_PT_REG_NPI_COMMIT      0x180U  /* W: commit, R: count            */
#define GNSS_PT_REG_NPI_INFO        0x184U  /* RO 0x03020801                  */
#define GNSS_PT_REG_NPI_CFG         0x188U  /* RW [1:0] mode, [2] image, [10:8] shift */
#define GNSS_PT_REG_NPI_NORM        0x18CU  /* RW g2, Q3.13 {im, re}          */
#define GNSS_PT_REG_NPI_DET_THR     0x190U  /* RW detector threshold [LSB^2]  */
#define GNSS_PT_REG_NPI_DET_HOLD    0x194U  /* RW detector hold [samples]     */
#define GNSS_PT_REG_NPI_CFG_ACTIVE  0x198U  /* RO NPI_CFG as committed        */
#define GNSS_PT_REG_NPI_SNAP        0x19CU  /* W: latch, R: live core status  */
#define GNSS_PT_REG_NPI_W(k)        (0x1A0U + ((k) * 4U)) /* RO k = 0..5, Q12.20 */
#define GNSS_PT_REG_NPI_DET_POW     0x1B8U  /* RO latched detector power      */
#define GNSS_PT_REG_NPI_ST_LATCHED  0x1BCU  /* RO latched core status         */
#define GNSS_PT_NPI_INFO_EXPECTED   0x03020801U
/* NPI_CFG */
#define GNSS_PT_NPI_MODE_AUTO       0x0U    /* power detector switches it     */
#define GNSS_PT_NPI_MODE_BYPASS     0x1U    /* TX1 = RX1, weights 0           */
#define GNSS_PT_NPI_MODE_ON         0x2U    /* filter in, adapting            */
#define GNSS_PT_NPI_MODE_HOLD       0x3U    /* filter in, weights frozen      */
#define GNSS_PT_NPI_CFG_IMAGE       0x4U    /* image taps on                  */
#define GNSS_PT_NPI_CFG_SHIFT(s)    ((uint32_t)((s) & 7) << 8)
/* core status (NPI_SNAP / NPI_ST_LATCHED) */
#define GNSS_PT_NPI_ST_DET          0x1U    /* power detector on              */
#define GNSS_PT_NPI_ST_ACTIVE       0x2U    /* filter in the path             */
#define GNSS_PT_NPI_ST_ADAPT        0x10U   /* weights adapting               */
#define GNSS_PT_NPI_ST_CNT(st)      ((uint32_t)(st) >> 16)  /* detector events */

/* ---- expected identity --------------------------------------------------- */
#define GNSS_PT_EXPECTED_ID         0x47435031U
/* 1.0 = Phase-1 identity passthrough.
 * 1.1 = adds the RX->TX sample alignment stage (RX is right-aligned 12-in-16,
 *       the AD9361 DAC consumes [15:4]). A board reporting 1.0 is running a
 *       bitstream whose TX output is 24 dB low with 4 bits discarded. */
/* 2.0 = PL-NPI-only build: the PL-NPI power-inversion core is a separate
 *       block-design IP (pl_npi_0, CONTROL[5], gamma in 0x48). This design
 *       has no PI-NLMS core.
 * 2.1 = pl_npi_0 is the pipelined Vitis HLS core (II=2 on l_clk).
 * 2.3 = pl_npi_0 v2.0 on sys_cpu_clk (II=1) behind async FIFOs (61.44 MS/s),
 *       RX2 normalisation, image taps, gated adaptation, make-up gain:
 *       NPI_CFG/NORM/DET_* + NPI_COMMIT, NPI_SNAP/NPI_W read-back. */
#define GNSS_PT_EXPECTED_VERSION    0x00020003U

/* ---- CONTROL bits -------------------------------------------------------- */
#define GNSS_PT_CTRL_PASS_EN        (1U << 0)  /* 1 = RX->TX passthrough      */
#define GNSS_PT_CTRL_MUTE           (1U << 1)  /* 1 = drive zeros to the DAC  */
#define GNSS_PT_CTRL_SWAP_IQ        (1U << 2)
#define GNSS_PT_CTRL_CH1_COPY       (1U << 3)  /* ch1 TX fed from ch0         */
#define GNSS_PT_CTRL_NPI_EN         (1U << 5)  /* ch0 TX = PL-NPI(RX1, RX2)   */
#define GNSS_PT_CTRL_NPI_FREEZE     (1U << 6)  /* PL-NPI weights frozen       */
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
#define GNSS_PT_ST_NPI_DROP         (1U << 10) /* v2.1+: pair lost at pl_npi_0 */
#define GNSS_PT_ST_PASS_EN_SYNCED   (1U << 16)
#define GNSS_PT_ST_NPI_EN_SYNCED    (1U << 18)
#define GNSS_PT_ST_NPI_BAND_SHIFT   20         /* [21:20] PL-NPI gain band (live) */
#define GNSS_PT_ST_NPI_BAND_MASK    0x3U

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

/* PL-NPI power inversion (bitstream v2.0+). Enabled, channel 0 TX carries the
 * PL-NPI core output (v2.3: bypassed = RX1 while no jammer, see NPI_CFG)
 * instead of RX1 alone; passthrough must also be on. */
void     gnss_pt_set_npi(int enable);
/* v2.3: writes NPI_CFG, NPI_NORM (g2 Q3.13, real), NPI_DET_THR and
 * NPI_DET_HOLD and commits them in one go. */
void     gnss_pt_npi_write(uint32_t cfg, int32_t g2_q13, uint32_t det_thr, uint32_t det_hold);
/* v2.3: latches and reads the core's weights (Q12.20: w2 re, w2 im, c1 re,
 * c1 im, c2 re, c2 im), detector power and status as one coherent set. */
typedef struct { uint32_t st, pow; int32_t w[6]; } gnss_pt_npi_snap_t;
void     gnss_pt_npi_snapshot(gnss_pt_npi_snap_t *s);
/* Regulariser gamma of mu_NPI = 1/(2*x^T x + gamma). 1 is the value the
 * PL-NPI testbench was measured with; 0 is used as 1 by the hardware. */
void     gnss_pt_set_npi_gamma(uint32_t gamma);
/* 1 = freeze the PL-NPI weights at their current value, 0 = adapt. */
void     gnss_pt_set_npi_freeze(int freeze);

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
