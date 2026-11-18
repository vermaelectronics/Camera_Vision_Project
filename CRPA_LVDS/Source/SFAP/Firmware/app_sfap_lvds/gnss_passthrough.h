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
#warning "gnss_passthrough base address not found in xparameters.h; using 0x44A00000 (what Vivado assigned in the Passthrough_LVDS + SFAP-PI build). Verify this against the generated XSA before trusting any register read."
#define GNSS_PT_BASEADDR   0x44A00000U
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
/* SFAP-PI v5 per-bin weights (see gnss_aj_math.h for the layout).
 * AJ_WB, AJ_WC1, AJ_WC2, AJ_CFG and AJ_NORM are SHADOW copies; a write to
 * AJ_COMMIT hands all of them to the filter in one clock. AJ_COMMIT reads
 * back a commit counter. */
#define GNSS_PT_REG_AJ_COMMIT       0x180U  /* W: commit, R: count            */
#define GNSS_PT_REG_AJ_INFO         0x184U  /* RO 0x08070C04                  */
#define GNSS_PT_REG_AJ_CFG          0x188U  /* RW [0] bypass, [10:8] shift,   */
                                            /*    [27:16] blank thr (v6.3)   */
#define GNSS_PT_REG_AJ_NORM         0x18CU  /* RW g2, Q3.13 {im, re}          */
#define GNSS_PT_REG_AJ_CFG_ACTIVE   0x198U  /* RO AJ_CFG as committed         */
/* v6.0: 3 x 256 words, WRITE-ONLY (read 0), double-buffered in the FPGA:
 * writes go to the spare bank, AJ_COMMIT swaps banks if any weight was
 * written since the last commit. So every commit that changes weights must
 * write ALL 768 words (gnss_pt_write_aj does). */
#define GNSS_PT_REG_AJ_WB(k)        (0x1000U + ((k) * 4U)) /* W b_k,  k = 0..255 */
#define GNSS_PT_REG_AJ_WC1(k)       (0x1400U + ((k) * 4U)) /* W c1_k, k = 0..255 */
#define GNSS_PT_REG_AJ_WC2(k)       (0x1800U + ((k) * 4U)) /* W c2_k, k = 0..255 */
#define GNSS_PT_AJ_NBIN             256U
#define GNSS_PT_AJ_NWORD            768U    /* b, c1, c2 x 256 bins           */
/* v6.3: bin excision mask, 8 words after the weights, same double buffer.
 * Word j bit b = bin 32 j + b; 1 = that bin's filter output is zero. A
 * commit that changes the weights writes all 768 + 8 words. */
#define GNSS_PT_REG_AJ_MASK(j)      (0x1C00U + ((j) * 4U)) /* W j = 0..7         */
#define GNSS_PT_AJ_NMASK            8U
/* AJ_INFO v6: {8'd log2 N, 8'd log2 hop, 8'd weight fraction bits, 8'd layout} */
#define GNSS_PT_AJ_INFO_EXPECTED    0x08070C05U  /* v6.3: layout 5 (blanker + mask) */
#define GNSS_PT_AJ_CFG_BYPASS       0x1U
#define GNSS_PT_AJ_CFG_SHIFT(s)     ((uint32_t)((s) & 7) << 8)
#define GNSS_PT_AJ_CFG_BLANK(t)     ((uint32_t)((t) & 0xFFF) << 16)  /* v6.3, 0 = off */

/* ---- expected identity --------------------------------------------------- */
#define GNSS_PT_EXPECTED_ID         0x47435031U
/* 1.0 = Phase-1 identity passthrough.
 * 1.1 = adds the RX->TX sample alignment stage (RX is right-aligned 12-in-16,
 *       the AD9361 DAC consumes [15:4]). A board reporting 1.0 is running a
 *       bitstream whose TX output is 24 dB low with 4 bits discarded. */
/* 2.x = PL-NPI builds (LoopBack_Code_PL_NPI).
 * 3.0 = SMI-PI build: closed-form power inversion y = x1 - a*x2 in a separate
 *       block-design IP (smi_pi_0, CONTROL[5], loading L in 0x48, weight band
 *       in STATUS[21:20]). No PI-NLMS or PL-NPI core.
 * 4.0 = SMI-PI space-time build: 16+16-tap filter (smi_pi_0 v2.0) whose 32
 *       weights the ARM computes and writes to AJ_W / AJ_COMMIT. No 0x48
 *       loading, no weight band.
 * 4.1 = adds AJ_CFG (bypass, output make-up shift), AJ_NORM (RX2 input
 *       normalisation) and the image taps AJ_WC (smi_pi_0 v2.1).
 * 5.0 = SFAP-PI build: 64-bin space-frequency filter (sfap_pi_0 v3.0), 3 x 64
 *       per-bin weights AJ_WB/AJ_WC1/AJ_WC2 at 0x200-0x4FC, latency 190
 *       samples; AJ_W (0x100) and AJ_WC (0x190) are gone.
 * 6.0 = SFAP-PI 256 bins (sfap_pi_0 v4.0): 3 x 256 weights at 0x1000-0x1BFC
 *       in a double-buffered RAM (write-only), 8 KB register map, latency 766
 *       samples, AJ_INFO 0x08070C04.
 * 6.3 = sfap_pi_0 v5.0: pulse blanker (AJ_CFG[27:16]) and per-bin excision
 *       mask at 0x1C00-0x1C1C, latency 782 samples, AJ_INFO 0x08070C05. */
#define GNSS_PT_EXPECTED_VERSION    0x00060003U

/* ---- CONTROL bits -------------------------------------------------------- */
#define GNSS_PT_CTRL_PASS_EN        (1U << 0)  /* 1 = RX->TX passthrough      */
#define GNSS_PT_CTRL_MUTE           (1U << 1)  /* 1 = drive zeros to the DAC  */
#define GNSS_PT_CTRL_SWAP_IQ        (1U << 2)
#define GNSS_PT_CTRL_CH1_COPY       (1U << 3)  /* ch1 TX fed from ch0         */
#define GNSS_PT_CTRL_SMI_EN         (1U << 5)  /* ch0 TX = SFAP-PI(RX1, RX2)  */
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
#define GNSS_PT_ST_SMI_DROP         (1U << 10) /* pair lost at sfap_pi_0     */
#define GNSS_PT_ST_PASS_EN_SYNCED   (1U << 16)
#define GNSS_PT_ST_SMI_EN_SYNCED    (1U << 18)

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

/* SFAP-PI filter (bitstream v5.x). Enabled, channel 0 TX carries the filter
 * output (RX1 and RX2 combined per frequency bin) instead of RX1 alone;
 * passthrough must also be on. The weights are whatever was last committed.
 * (The name is kept from the SMI-PI builds: same CONTROL bit.) */
void     gnss_pt_set_smi(int enable);
/* Writes the 768 weight words (256 b_k, 256 c1_k, 256 c2_k, Q3.12, see
 * gnss_aj_math.h), the 8 excision-mask words (v6.3; mask NULL = no bin
 * excised), cfg (GNSS_PT_AJ_CFG_*) and the RX2 normalisation g2 (Q3.13 real
 * part; imaginary 0), and commits them in one go: the filter never runs a
 * half-written set. re/im NULL: nothing goes into the weight RAM, so the
 * commit keeps the running weights and mask and only changes cfg / g2
 * (bypass switching, a blanker threshold update). */
void     gnss_pt_write_aj(const int16_t *re, const int16_t *im, const uint32_t *mask,
                          uint32_t cfg, int32_t g2_q13);

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
