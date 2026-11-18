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

/* LVDS / ant board (Passthrough_MVDR): ad_cpu_interconnect 0x43C10000 in
 * system_bd.tcl, because 0x43C00000 is the ant VCXO controller. Fixed here
 * rather than taken from xparameters.h: a module-reference cell gets no
 * reliable XPAR_ name in the SDT flow. */
#define GNSS_PT_BASEADDR   0x43C10000U

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

/* ---- MVDR adaptive beamformer window (GNSS-CRPA MOD-11) -------------------
 * Offset 0x100..0x1FC inside this core. Independent of CRPA_COEF above, which
 * belongs to an earlier fixed-beamforming experiment and is left untouched.
 * MUST stay in step with Source/HDL/mvdr_regs.v -- see its header comment for
 * the authoritative bit-for-bit map. */
#define GNSS_PT_MVDR_BASE            0x100U

#define GNSS_PT_REG_MVDR_ID          (GNSS_PT_MVDR_BASE + 0x00U)  /* RO  0x4D564452 "MVDR"        */
#define GNSS_PT_REG_MVDR_VERSION     (GNSS_PT_MVDR_BASE + 0x04U)  /* RO  {major,minor}             */
#define GNSS_PT_REG_MVDR_CTRL        (GNSS_PT_MVDR_BASE + 0x08U)  /* RW  mode/freeze/cnt_clear      */
#define GNSS_PT_REG_MVDR_COMMIT      (GNSS_PT_MVDR_BASE + 0x0CU)  /* W=1 commits; R bit0=pending    */
#define GNSS_PT_REG_MVDR_A1          (GNSS_PT_MVDR_BASE + 0x10U)  /* RW  {a1_im,a1_re} Q14          */
#define GNSS_PT_REG_MVDR_A2          (GNSS_PT_MVDR_BASE + 0x14U)  /* RW  {a2_im,a2_re} Q14          */
#define GNSS_PT_REG_MVDR_EPS_ABS_LO  (GNSS_PT_MVDR_BASE + 0x18U)  /* RW  diagonal loading, lo32     */
#define GNSS_PT_REG_MVDR_EPS_ABS_HI  (GNSS_PT_MVDR_BASE + 0x1CU)  /* RW  diagonal loading, hi bits  */
#define GNSS_PT_REG_MVDR_EPS_REL     (GNSS_PT_MVDR_BASE + 0x20U)  /* RW  [5:0] shift, 63 = off      */
#define GNSS_PT_REG_MVDR_FIXW1_RE    (GNSS_PT_MVDR_BASE + 0x24U)  /* RW  Q15, mode 2 (fixed) only   */
#define GNSS_PT_REG_MVDR_FIXW1_IM    (GNSS_PT_MVDR_BASE + 0x28U)  /* RW                              */
#define GNSS_PT_REG_MVDR_FIXW2_RE    (GNSS_PT_MVDR_BASE + 0x2CU)  /* RW                              */
#define GNSS_PT_REG_MVDR_FIXW2_IM    (GNSS_PT_MVDR_BASE + 0x30U)  /* RW                              */
#define GNSS_PT_REG_MVDR_STATUS      (GNSS_PT_MVDR_BASE + 0x40U)  /* RO, see GNSS_PT_MVDR_ST_* below */
#define GNSS_PT_REG_MVDR_N_BLOCKS    (GNSS_PT_MVDR_BASE + 0x44U)  /* RO  covariance blocks seen      */
#define GNSS_PT_REG_MVDR_N_UPD       (GNSS_PT_MVDR_BASE + 0x48U)  /* RO  weight sets applied         */
#define GNSS_PT_REG_MVDR_N_SKIP      (GNSS_PT_MVDR_BASE + 0x4CU)  /* RO  runs skipped (d<=0)         */
#define GNSS_PT_REG_MVDR_N_SAT       (GNSS_PT_MVDR_BASE + 0x50U)  /* RO  applied sets that clamped   */
#define GNSS_PT_REG_MVDR_N_DROP      (GNSS_PT_MVDR_BASE + 0x54U)  /* RO  starts dropped (calc busy)  */
#define GNSS_PT_REG_MVDR_N_ZERO      (GNSS_PT_MVDR_BASE + 0x58U)  /* RO  all-zero covariance blocks  */
#define GNSS_PT_REG_MVDR_N_SKEW      (GNSS_PT_MVDR_BASE + 0x5CU)  /* RO  RX1/RX2 valid mismatches    */
#define GNSS_PT_REG_MVDR_N_OSAT      (GNSS_PT_MVDR_BASE + 0x60U)  /* RO  output samples clamped      */
#define GNSS_PT_REG_MVDR_W1_RE       (GNSS_PT_MVDR_BASE + 0x64U)  /* RO  current weight, Q15, signed */
#define GNSS_PT_REG_MVDR_W1_IM       (GNSS_PT_MVDR_BASE + 0x68U)  /* RO                              */
#define GNSS_PT_REG_MVDR_W2_RE       (GNSS_PT_MVDR_BASE + 0x6CU)  /* RO                              */
#define GNSS_PT_REG_MVDR_W2_IM       (GNSS_PT_MVDR_BASE + 0x70U)  /* RO                              */
#define GNSS_PT_REG_MVDR_COV_R11_LO  (GNSS_PT_MVDR_BASE + 0x74U)  /* RO  latest raw block sum, ch1   */
#define GNSS_PT_REG_MVDR_COV_R11_HI  (GNSS_PT_MVDR_BASE + 0x78U)  /* RO  (64-bit sign-extended pair) */
#define GNSS_PT_REG_MVDR_COV_R22_LO  (GNSS_PT_MVDR_BASE + 0x7CU)  /* RO  latest raw block sum, ch2   */
#define GNSS_PT_REG_MVDR_COV_R22_HI  (GNSS_PT_MVDR_BASE + 0x80U)  /* RO                              */
/* Cross-covariance R12 = sum over the block of x1 * conj(x2), real and imag
 * parts, each a 64-bit sign-extended LO/HI pair like R11/R22. Added in MVDR
 * v1.1 so the firmware can check the hardware weight against the covariance
 * it was computed from. On an MVDR v1.0 bitstream these offsets are not
 * decoded; the driver checks the MVDR version and does not read them. */
#define GNSS_PT_REG_MVDR_COV_R12_RE_LO (GNSS_PT_MVDR_BASE + 0x84U) /* RO                            */
#define GNSS_PT_REG_MVDR_COV_R12_RE_HI (GNSS_PT_MVDR_BASE + 0x88U) /* RO                            */
#define GNSS_PT_REG_MVDR_COV_R12_IM_LO (GNSS_PT_MVDR_BASE + 0x8CU) /* RO                            */
#define GNSS_PT_REG_MVDR_COV_R12_IM_HI (GNSS_PT_MVDR_BASE + 0x90U) /* RO                            */

/* ---- jammer estimator (GNSS-CRPA MOD-12, jammer_est.v, MVDR >= 1.2) ------ */
#define GNSS_PT_REG_JAM_STATUS       (GNSS_PT_MVDR_BASE + 0x94U)  /* RO  see GNSS_PT_JAM_ST_*        */
#define GNSS_PT_REG_JAM_ANGLE        (GNSS_PT_MVDR_BASE + 0x98U)  /* RO  [15:0] theta [31:16] psi, 0.01 deg */
#define GNSS_PT_REG_JAM_PHRAW        (GNSS_PT_MVDR_BASE + 0x9CU)  /* RO  [15:0] bin [31:16] 0.01 deg  */
#define GNSS_PT_REG_JAM_POWER        (GNSS_PT_MVDR_BASE + 0xA0U)  /* RO  [15:0] jam [31:16] noise, dBFS Q8 */
#define GNSS_PT_REG_JAM_JNR          (GNSS_PT_MVDR_BASE + 0xA4U)  /* RO  [15:0] JNR dB Q8             */
#define GNSS_PT_REG_JAM_N_EST        (GNSS_PT_MVDR_BASE + 0xA8U)  /* RO  estimates computed           */
#define GNSS_PT_REG_JAM_N_DROP       (GNSS_PT_MVDR_BASE + 0xACU)  /* RO  windows dropped (busy)       */
#define GNSS_PT_REG_JAM_CTRL         (GNSS_PT_MVDR_BASE + 0xC0U)  /* RW  [0] en [1] inv [11:8] avg    */
#define GNSS_PT_REG_JAM_CAL          (GNSS_PT_MVDR_BASE + 0xC4U)  /* RW  [15:0] phase cal, bin angle  */
#define GNSS_PT_REG_JAM_K            (GNSS_PT_MVDR_BASE + 0xC8U)  /* RW  [15:0] lambda/(2d) Q14       */
#define GNSS_PT_REG_JAM_THR          (GNSS_PT_MVDR_BASE + 0xCCU)  /* RW  [15:0] on [31:16] off, dB Q8 */

/* ---- covariance input quality (cov2x2_accum v2, MVDR >= 1.3) --------------
 * Number of samples sitting on an ADC rail (+2047 / -2048 on I or Q) inside
 * the current covariance window (4096 samples before MVDR 1.6, 8192 since). Anything non-zero means the ADC
 * is clipping: a clipped jammer is no longer a plane wave and cannot be
 * nulled by a single complex weight. Reduce the RX gain (gnss_rxgain=). */
#define GNSS_PT_REG_MVDR_COV_CLIP    (GNSS_PT_MVDR_BASE + 0xB0U)  /* RO  [15:0] RX1 [31:16] RX2       */

/* ---- MVDR output gain (output_scaler v2, MVDR >= 1.4) ---------------------
 * [2:0] n: the MVDR output is multiplied by 2^n (6.02 dB/step, 0 = unity,
 * 7 = +42 dB) before the 12-bit saturation. MVDR path only (CONTROL[4] = 1);
 * raw RX1 (gnss_mvdr=0) is not affected. Applied by COMMIT. Watch N_OSAT:
 * it counts output samples that clipped. */
#define GNSS_PT_REG_MVDR_OUT_GAIN    (GNSS_PT_MVDR_BASE + 0xD0U)  /* RW  [2:0] gain = 2^n, AOLC [23:8] */
#define GNSS_PT_MVDR_OUT_GAIN_MAX    7U

/* ---- MVDR 1.6 .. 1.11 additions (block v1.12 bitstream) --------------------
 * All configuration registers below are applied by COMMIT like the others.
 * Authoritative map: header of mvdr_regs.v. */
#define GNSS_PT_REG_MVDR_COV_PEAK    (GNSS_PT_MVDR_BASE + 0xB4U)  /* RO  1.6  [11:0] RX1 [27:16] RX2 max(|I|,|Q|), raw LSB */
#define GNSS_PT_REG_MVDR_NF_EST_LO   (GNSS_PT_MVDR_BASE + 0xB8U)  /* RO  1.7  noise floor det(R)/tr(R), window-sum units */
#define GNSS_PT_REG_MVDR_NF_EST_HI   (GNSS_PT_MVDR_BASE + 0xBCU)  /* RO  1.7  [15:0] upper bits; 1.8: [21:16] COH_L2 */
#define GNSS_PT_REG_MVDR_EPS_NF      (GNSS_PT_MVDR_BASE + 0xD4U)  /* RW  1.7  [7:0] loading = k/8 x noise floor */
#define GNSS_PT_REG_MVDR_SMOOTH      (GNSS_PT_MVDR_BASE + 0xD8U)  /* RW  1.8  hold / glide / coherence detect   */
#define GNSS_PT_REG_MVDR_IQ_CTRL     (GNSS_PT_MVDR_BASE + 0xDCU)  /* RW  1.9  IQ-image correction control       */
#define GNSS_PT_REG_MVDR_N_JAM       (GNSS_PT_MVDR_BASE + 0xE0U)  /* RO  1.8  weight runs with a jammer       */
#define GNSS_PT_REG_MVDR_N_HOLD      (GNSS_PT_MVDR_BASE + 0xE4U)  /* RO  1.8  weight runs absorbed by a hold  */
#define GNSS_PT_REG_MVDR_IQ_B1_RE    (GNSS_PT_MVDR_BASE + 0xE8U)  /* RO  1.9  RX1 image coefficient, Q20 signed */
#define GNSS_PT_REG_MVDR_IQ_B1_IM    (GNSS_PT_MVDR_BASE + 0xECU)  /* RO                                   */
#define GNSS_PT_REG_MVDR_IQ_B2_RE    (GNSS_PT_MVDR_BASE + 0xF0U)  /* RO  1.9  RX2                          */
#define GNSS_PT_REG_MVDR_IQ_B2_IM    (GNSS_PT_MVDR_BASE + 0xF4U)  /* RO                                   */
#define GNSS_PT_REG_MVDR_N_IQ_UPD    (GNSS_PT_MVDR_BASE + 0xF8U)  /* RO  1.9  image-coefficient updates      */
#define GNSS_PT_REG_MVDR_OUT_LVL     (GNSS_PT_MVDR_BASE + 0xFCU)  /* RO  1.10 [15:0] log2 out power Q4, [31:16] gain Q8 oct */
/* page 2 of the MVDR window (core offset 0x200..0x2FC), jammer_est v2, 1.11 */
#define GNSS_PT_REG_JAM_PWR_CH       (GNSS_PT_MVDR_BASE + 0x100U) /* RO  [15:0] RX1 [31:16] RX2 average jammer power, dBFS Q8 */
#define GNSS_PT_REG_JAM_PEAK         (GNSS_PT_MVDR_BASE + 0x104U) /* RO  [15:0] RX1 [31:16] RX2 peak (on) jammer power, dBFS Q8 */
#define GNSS_PT_REG_NOISE_CH         (GNSS_PT_MVDR_BASE + 0x108U) /* RO  [15:0] RX1 [31:16] RX2 noise floor, dBFS Q8 */
/* MOD-24, MVDR window 1.13: STAP = space-time power inversion (stap_lms.v,
 * 2 antennas x 5 taps, NLMS). Runs beside the single-weight beamformer;
 * STAP_EN picks which one drives the output. Applied by COMMIT. */
#define GNSS_PT_REG_STAP_CTRL        (GNSS_PT_MVDR_BASE + 0x10CU) /* RW  [0] EN [7:4] MU_SH [12:8] LEAK_SH [16] CLEAR */
#define GNSS_PT_REG_STAP_ST          (GNSS_PT_MVDR_BASE + 0x110U) /* RO  [5:0] PL (input power ~2^(PL-2)) [8] active */
#define GNSS_PT_REG_STAP_WNORM       (GNSS_PT_MVDR_BASE + 0x114U) /* RO  sum |w|^2 of the 9 aux taps, Q8.24 */
#define GNSS_PT_REG_STAP_NWSAT       (GNSS_PT_MVDR_BASE + 0x118U) /* RO  updates limited by the anti-windup */
#define GNSS_PT_STAP_EN              (1U << 0)
#define GNSS_PT_STAP_MU(n)           (((uint32_t)(n) & 0xFU) << 4)   /* step 2^-n            */
#define GNSS_PT_STAP_LEAK(n)         (((uint32_t)(n) & 0x1FU) << 8)  /* leak 2^-n, 16..31    */
#define GNSS_PT_STAP_CLEAR           (1U << 16)
#define GNSS_PT_STAP_ST_ACTIVE       (1U << 8)
#define GNSS_PT_STAP_ST_ABSENT       (1U << 31)  /* 1.14: NLMS STAP not built (SMI-only bitstream) */
/* MU 5: settles a CW jammer in ~1000 samples (~20 us) and a 56 MHz barrage in
 * ~4000 (~70 us); 6..7 = slower / slightly deeper. 4 is NOT allowed (the RTL
 * clamps it to 5): with the update delay it diverges on a CW jammer. */
#define GNSS_PT_STAP_MU_DEFAULT      5U
#define GNSS_PT_STAP_LEAK_DEFAULT    18U

/* OUT_GAIN fields (1.10) */
#define GNSS_PT_OG_AOLC_EN           (1U << 8)                      /* automatic output level, default off */
#define GNSS_PT_OG_AGC_SH(n)         (((uint32_t)(n) & 0xFU) << 12) /* loop speed, default 8 (~19 ms)      */
#define GNSS_PT_OG_TARGET_DB(n)      (((uint32_t)(n) & 0xFFU) << 16)/* output floor, dB below FS           */
#define GNSS_PT_AOLC_SH_DEFAULT      8U
#define GNSS_PT_AOLC_TARGET_DEFAULT  24U                            /* -24 dBFS (MOD-20) */

/* EPS_NF (1.7): diagonal loading = k/8 x noise floor. 8 = x1 (recommended). */
#define GNSS_PT_EPS_NF_DEFAULT       8U
#define GNSS_PT_EPS_REL_OFF          63U   /* EPS_REL: 63 = off. Do NOT use 10 on >= 1.7 */

/* SMOOTH fields (1.8) */
#define GNSS_PT_SM_HOLD(n)           ((uint32_t)(n) & 0xFFFFU)        /* x 64 hops (1.46 ms @ 45 MSPS) */
#define GNSS_PT_SM_REL_SH(n)         (((uint32_t)(n) & 0xFU) << 16)   /* glide tau 2^n hops            */
#define GNSS_PT_SM_COH_SH(n)         (((uint32_t)(n) & 0xFU) << 24)   /* jammer if coherence > 2^-n    */
#define GNSS_PT_SM_HOLD_DEFAULT      252U   /* 0.37 s at 45 MSPS */
#define GNSS_PT_SM_REL_DEFAULT       14U    /* 0.37 s at 45 MSPS */
/* MOD-19: was 8. Coherence 2^-8 = 0.004 is reached by the SATELLITES
 * themselves (all GNSS together are ~22 dB under the noise in 40 MHz, i.e.
 * coherence 0.003..0.006 between two antennas), so 8 flagged a "jammer" on
 * and off every few hundred ms with no jammer present (COH_L2 7..9 in the
 * 2026-10-30 log). 4 = coherence > 1/16: a signal about -12 dB vs the noise
 * or stronger -- every jammer that matters, never the satellites. */
#define GNSS_PT_SM_COH_DEFAULT       4U
#define GNSS_PT_SMOOTH_DEFAULT       (GNSS_PT_SM_HOLD(GNSS_PT_SM_HOLD_DEFAULT) | \
                                      GNSS_PT_SM_REL_SH(GNSS_PT_SM_REL_DEFAULT) | \
                                      GNSS_PT_SM_COH_SH(GNSS_PT_SM_COH_DEFAULT))

/* IQ_CTRL fields (1.9) */
#define GNSS_PT_IQ_OFF               (1U << 0)   /* 0 = correction ON (default) */
#define GNSS_PT_IQ_CLEAR             (1U << 1)   /* hold b = 0                  */
#define GNSS_PT_IQ_MU_SH(n)          (((uint32_t)(n) & 7U) << 4)
#define GNSS_PT_IQ_MU_DEFAULT        2U

#define GNSS_PT_MVDR_EXPECTED_ID     0x4D564452U   /* "MVDR" */
/* MVDR window versions, each adds to the previous one (1.6 .. 1.12: see the
 * GNSS_PT_MVDR_VERSION_* list below and the mvdr_regs.v header):
 *   1.1  COV_R12_* readback
 *   1.2  jammer estimator (0x94..0xAC, 0xC0..0xCC)
 *   1.3  sliding covariance window (4096 samples, a new window every 1024
 *        samples) and COV_CLIP at 0xB0; weight sets that would overflow are
 *        SCALED by 2^-k as a whole (null kept) instead of clamped
 *   1.4  OUT_GAIN at 0xD0
 *   1.5  cov_prep / weight calc / control / jammer_est moved to the fixed
 *        100 MHz AXI clock (timing met at any sample rate); same register map
 * The driver checks the version before touching a newer register, so it
 * still runs on an older bitstream -- it just prints less. */
#define GNSS_PT_MVDR_VERSION_R12     0x00010001U   /* 1.1 */
#define GNSS_PT_MVDR_VERSION_JAM     0x00010002U   /* 1.2 */
#define GNSS_PT_MVDR_VERSION_CLIP    0x00010003U   /* 1.3 */
#define GNSS_PT_MVDR_VERSION_GAIN    0x00010004U   /* 1.4 */
#define GNSS_PT_MVDR_VERSION_XDOM    0x00010005U   /* 1.5: maths on the 100 MHz AXI clock */
#define GNSS_PT_MVDR_VERSION_DCRM    0x00010006U   /* 1.6: mean removal, 8192 window, COV_PEAK */
#define GNSS_PT_MVDR_VERSION_EPSNF   0x00010007U   /* 1.7: noise-referenced loading, NF_EST */
#define GNSS_PT_MVDR_VERSION_SMOOTH  0x00010008U   /* 1.8: jammer detect, null hold, glide */
#define GNSS_PT_MVDR_VERSION_IQ      0x00010009U   /* 1.9: IQ-image correction, cov units x16 */
#define GNSS_PT_MVDR_VERSION_AOLC    0x0001000AU   /* 1.10: automatic output level, OUT_LVL */
#define GNSS_PT_MVDR_VERSION_JAM2    0x0001000BU   /* 1.11: per-channel / peak jammer power */
#define GNSS_PT_MVDR_VERSION_TIMING  0x0001000CU   /* 1.12: timing closed at 250 MHz l_clk */
#define GNSS_PT_MVDR_VERSION_STAP    0x0001000DU   /* 1.13: STAP (space-time power inversion) */
#define GNSS_PT_MVDR_VERSION_SMI     0x0001000EU   /* 1.14: SMI-MVDR STAP (0x400 window)      */
#define GNSS_PT_MVDR_VERSION_COVHEAL 0x0001000FU   /* 1.15: 2x2 window self-healing, 16-tap SMI */
#define GNSS_PT_MVDR_EXPECTED_VERSION GNSS_PT_MVDR_VERSION_COVHEAL

/* JAM_STATUS bits */
#define GNSS_PT_JAM_ST_PRESENT       (1U << 0)   /* jammer present (with hysteresis)     */
#define GNSS_PT_JAM_ST_VALID         (1U << 1)   /* at least one estimate since enable   */
#define GNSS_PT_JAM_ST_SIN_CLIP      (1U << 2)   /* |sin(theta)| > 1 was clipped          */
#define GNSS_PT_JAM_ST_NO_SIGNAL     (1U << 3)   /* covariance all zero                   */
#define GNSS_PT_JAM_ST_NOISE_ZERO    (1U << 4)   /* noise estimate 0, JNR off scale       */

/* JAM_CTRL fields */
#define GNSS_PT_JAM_CTRL_EN          (1U << 0)
#define GNSS_PT_JAM_CTRL_INV         (1U << 1)
#define GNSS_PT_JAM_CTRL_AVG_SHIFT   8U
#define GNSS_PT_JAM_CTRL_AVG_MASK    (0xFU << GNSS_PT_JAM_CTRL_AVG_SHIFT)
/* With the sliding covariance window (MVDR >= 1.3) a new window arrives
 * every 1024 samples instead of every 4096, so the reset default of 2^4
 * windows now spans only ~0.4 ms. gnss_mvdr=1 sets 2^6 = 64 windows
 * (65536 samples, ~1.46 ms at 45 MSPS): the same estimate rate and noise as
 * the v1.2 build the jammer numbers were validated on. */
#define GNSS_PT_JAM_AVG_LOG2_DEFAULT 6U

/* "no energy" marker in the dBFS Q8 fields */
#define GNSS_PT_JAM_Q8_NONE          (-32768)

/* dBFS -> dBm board offset, hundredths of a dB. GNSS_PT_JAM_K_UNSET means
 * "not calibrated" and dBm is not printed. After gnss_jpcal= prints a value,
 * put it here (or -D it on the compile line) to keep it across reboots.
 * IF YOUR OLD gnss_passthrough.h HAD A CALIBRATED VALUE, COPY IT HERE. */
#define GNSS_PT_JAM_K_UNSET          (-999999L)
#ifndef GNSS_PT_JAM_DBM_K_C2_DEFAULT
#define GNSS_PT_JAM_DBM_K_C2_DEFAULT GNSS_PT_JAM_K_UNSET
#endif

/* ---- axi_ad9361 interface clock (l_clk) -----------------------------------
 * The whole gnss_passthrough / MVDR datapath runs on l_clk, which comes from
 * the AD9361 DATA_CLK. In LVDS 2R2T mode l_clk = 4 x the RX sample rate
 * (180 MHz at 45 MSPS). axi_ad9361's common register REG_CLK_FREQ (0x54)
 * counts it against the 100 MHz AXI clock: MHz = value * 100 / 65536.
 *
 * GNSS_PT_LCLK_CONSTRAINT_MHZ is the frequency the timing analysis was closed
 * at (the rx_clk create_clock period in ADI's XDC: 4.0 ns = 250 MHz). If the
 * measured l_clk is higher, "timing met" in Vivado does not cover the
 * hardware: change the constraint to the real period and close timing
 * again. Edit this number when you change the XDC. */
#if defined(XPAR_AXI_AD9361_BASEADDR)
#define GNSS_PT_AD9361_BASEADDR      XPAR_AXI_AD9361_BASEADDR
#elif defined(XPAR_AXI_AD9361_0_BASEADDR)
#define GNSS_PT_AD9361_BASEADDR      XPAR_AXI_AD9361_0_BASEADDR
#else
#define GNSS_PT_AD9361_BASEADDR      0x79020000U   /* project address map */
#endif
#define GNSS_PT_AD9361_REG_CLK_FREQ  0x0054U   /* ADC common: l_clk counter */
#define GNSS_PT_LCLK_CONSTRAINT_MHZ  250U      /* ADI rx_clk -period 4.000 (block v1.12 closes timing there) */
#define GNSS_PT_LCLK_PER_SAMPLE      4U        /* LVDS 2R2T */

/* MVDR CTRL fields (GNSS_PT_REG_MVDR_CTRL) */
#define GNSS_PT_MVDR_MODE_BYPASS     0U   /* [1,0] weights: ch0 = RX1 unmodified   */
#define GNSS_PT_MVDR_MODE_ADAPTIVE   1U   /* computed MVDR weights                 */
#define GNSS_PT_MVDR_MODE_FIXED      2U   /* FIXW1/FIXW2 weights, no adaptation    */
#define GNSS_PT_MVDR_CTRL_FREEZE     (1U << 2)  /* hold the current weights        */
#define GNSS_PT_MVDR_CTRL_CNT_CLEAR  (1U << 3)  /* held, not self-clearing         */
#define GNSS_PT_MVDR_CTRL_DC_KEEP    (1U << 4)  /* 1.6: 1 = raw covariance (mean removal off) */

/* MVDR STATUS bits (GNSS_PT_REG_MVDR_STATUS) */
#define GNSS_PT_MVDR_ST_MODE_MASK    0x3U        /* [1:0] mode in effect right now */
#define GNSS_PT_MVDR_ST_HAVE_W       (1U << 2)   /* an adaptive weight set has been applied */
#define GNSS_PT_MVDR_ST_LAST_SAT     (1U << 3)   /* most recent applied set was SCALED by 2^-k
                                                    (MVDR >= 1.3; clamped on older builds) */
#define GNSS_PT_MVDR_ST_LAST_SKIP    (1U << 4)   /* most recent run was skipped (sticky)    */
#define GNSS_PT_MVDR_ST_SKEW         (1U << 5)   /* RX1/RX2 valid mismatch seen (sticky)    */
#define GNSS_PT_MVDR_ST_WC_BUSY      (1U << 6)   /* weight calculator running right now     */
#define GNSS_PT_MVDR_ST_PENDING      (1U << 7)   /* a COMMIT has not yet reached the datapath */
#define GNSS_PT_MVDR_ST_DC_RM        (1U << 8)   /* 1.6: covariance is mean-removed          */
#define GNSS_PT_MVDR_ST_JAM          (1U << 9)   /* 1.8: jammer detected (coherence test)    */
#define GNSS_PT_MVDR_ST_HOLD         (1U << 10)  /* 1.8: jammer gone, null held              */
#define GNSS_PT_MVDR_ST_GLIDE        (1U << 11)  /* 1.8: easing back to the unjammed weights */

/* Covariance window length. Every COV_* register is a sum over one whole
 * window, so this turns a raw SUM into a mean power PER SAMPLE. The window
 * SLIDES: a new sum (and a new weight set) every GNSS_PT_MVDR_HOP_N samples.
 *   MVDR 1.3 .. 1.5 : 4096-sample window (LOG2_N = 12, LOG2_SUB = 2)
 *   MVDR >= 1.6     : 8192-sample window (LOG2_N = 13, LOG2_SUB = 3)
 * Hop 1024 in both. Since MVDR 1.9 the covariance is computed on the
 * IQ-corrected stream, which carries 2 extra fraction bits: every COV_*,
 * NF_EST and EPS_ABS value is in (raw LSB x 4)^2 = 16 x raw-sample units.
 * Use gnss_pt_mvdr_block_n() / gnss_pt_mvdr_cov_scale(); the constants are
 * the current build's values. */
#define GNSS_PT_MVDR_BLOCK_N         8192U
#define GNSS_PT_MVDR_BLOCK_N_OLD     4096U     /* MVDR < 1.6 */
#define GNSS_PT_MVDR_HOP_N           1024U
#define GNSS_PT_MVDR_COV_SCALE       16U       /* MVDR >= 1.9 */

/* ---- expected identity --------------------------------------------------- */
#define GNSS_PT_EXPECTED_ID         0x47435031U
/* 1.0 = Phase-1 identity passthrough.
 * 1.1 = adds the RX->TX sample alignment stage (RX is right-aligned 12-in-16,
 *       the AD9361 DAC consumes [15:4]).
 * 1.2 = adds the MVDR adaptive beamformer window above and CONTROL[4]
 *       (mvdr_sel). A board reporting 1.0 or 1.1 does not have gnss_mvdr=
 *       available; gnss_mvdr= checks GNSS_PT_REG_MVDR_ID itself and refuses
 *       rather than guessing. */
#define GNSS_PT_EXPECTED_VERSION    0x00010002U

/* ---- CONTROL bits -------------------------------------------------------- */
#define GNSS_PT_CTRL_PASS_EN        (1U << 0)  /* 1 = RX->TX passthrough      */
#define GNSS_PT_CTRL_MUTE           (1U << 1)  /* 1 = drive zeros to the DAC  */
#define GNSS_PT_CTRL_SWAP_IQ        (1U << 2)
#define GNSS_PT_CTRL_CH1_COPY       (1U << 3)  /* ch1 TX fed from ch0         */
#define GNSS_PT_CTRL_MVDR_SEL       (1U << 4)  /* 1 = ch0 fed by the MVDR output, needs v1.2 */
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
#define GNSS_PT_ST_MVDR_SEL_SYNCED  (1U << 17)  /* mvdr_sel, as seen in the clk domain, needs v1.2 */

/* ---- return codes -------------------------------------------------------- */
#define GNSS_PT_OK                   0
#define GNSS_PT_ERR_ID              -1   /* core not present / wrong ID       */
#define GNSS_PT_ERR_NO_RX           -2   /* no RX samples arriving            */
#define GNSS_PT_ERR_NO_TX           -3   /* no TX requests arriving           */
#define GNSS_PT_ERR_SCRATCH         -4   /* register read/write failed        */
#define GNSS_PT_ERR_ALIGNMENT       -5   /* TX samples are not left-aligned   */
#define GNSS_PT_ERR_MVDR_ID         -6   /* MVDR window not present (old bitstream) */
#define GNSS_PT_ERR_MVDR_COMMIT     -7   /* MVDR COMMIT never cleared pending */
#define GNSS_PT_ERR_JAM_MISSING     -8   /* jammer estimator not in bitstream (MVDR < 1.2) */
#define GNSS_PT_ERR_JAM_NO_SOURCE   -9   /* no source detected / no estimates */
#define GNSS_PT_ERR_JAM_LEVEL      -10   /* source clipping or too weak       */
#define GNSS_PT_ERR_MVDR_OLD       -11   /* feature needs a newer MVDR window */

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

/* ---- MVDR adaptive beamformer (GNSS-CRPA MOD-11) -------------------------- */

/* Everything worth observing about the MVDR core at runtime. Values that come
 * from a shadow register (CTRL, A1, A2) are what was last WRITTEN, which may
 * not have reached the datapath yet if a COMMIT is still pending -- check
 * commit_pending, or read mode_live (from STATUS) for what is actually
 * running right now. */
typedef struct {
    uint32_t id;
    uint32_t version;
    int      mvdr_sel;        /* CONTROL/STATUS: is ch0 fed by the MVDR output */
    int      mode_shadow;     /* CTRL[1:0] as last written                     */
    int      mode_live;       /* STATUS[1:0] actually running in the datapath  */
    int      freeze;
    int      commit_pending;
    int      have_weights;    /* an adaptive weight set has been applied      */
    int      last_sat;
    int      last_skip;
    int      skew_sticky;
    int      wc_busy;
    uint32_t n_blocks, n_upd, n_skip, n_sat, n_drop, n_zero, n_skew, n_osat;
    int16_t  a1_re, a1_im, a2_re, a2_im;      /* steering vector, Q14          */
    int32_t  w1_re, w1_im, w2_re, w2_im;      /* current weights, Q15, signed  */
    int64_t  cov_r11, cov_r22;                /* latest raw block sums, signed */
    int64_t  cov_r12_re, cov_r12_im;          /* cross-covariance, MVDR >= 1.1 */
    int      have_r12;        /* bitstream exposes COV_R12_* (MVDR >= 1.1)     */
    int      cov_coherent;    /* R11/R22/R12/clip all read from the SAME window */
    int      have_clip;       /* COV_CLIP present (MVDR >= 1.3)                */
    uint32_t clip1, clip2;    /* RX1/RX2 samples on an ADC rail, same window   */
    int      have_ogain;      /* OUT_GAIN present (MVDR >= 1.4)                */
    uint32_t out_gain;        /* OUT_GAIN[2:0] as last written (shadow)        */
    /* ---- MVDR 1.6 .. 1.11 ---- */
    uint32_t block_n;         /* window length of this bitstream (4096/8192)    */
    uint32_t cov_scale;       /* covariance units per raw LSB^2 (1 or 16)       */
    uint32_t status_raw;      /* MVDR STATUS word                               */
    int      dc_rm;           /* 1.6: mean removal in effect                    */
    int      have_peak;       /* 1.6: COV_PEAK                                  */
    uint32_t peak1, peak2;    /* max(|I|,|Q|) in the window, raw LSB            */
    int      have_nf;         /* 1.7: NF_EST / EPS_NF                           */
    uint64_t nf_est;          /* noise floor estimate, window-sum units         */
    uint32_t eps_nf_k;        /* EPS_NF shadow                                  */
    int      have_smooth;     /* 1.8: jammer flag, hold, glide, COH_L2          */
    int      st_jam, st_hold, st_glide;
    uint32_t coh_l2;          /* ~ -log2(RX1/RX2 coherence), 63 = none          */
    uint32_t smooth;          /* SMOOTH shadow                                  */
    uint32_t n_jam, n_hold;
    int      have_iq;         /* 1.9: IQ-image correction                       */
    uint32_t iq_ctrl;         /* IQ_CTRL shadow                                 */
    int32_t  iq_b1_re, iq_b1_im, iq_b2_re, iq_b2_im;   /* Q20                   */
    uint32_t n_iq_upd;
    int      have_aolc;       /* 1.10: AOLC + OUT_LVL                           */
    uint32_t out_gain_reg;    /* whole OUT_GAIN word (shadow)                   */
    int16_t  out_lvl_q4;      /* log2 output power per sample, Q4               */
    int16_t  out_gain_q8;     /* output gain in use, Q8 octaves                 */
    /* ---- MVDR 1.13: STAP ---- */
    int      have_stap;       /* STAP core in this bitstream                    */
    uint32_t stap_ctrl;       /* STAP_CTRL shadow                               */
    int      stap_active;     /* STAP is driving the output (applied)           */
    uint32_t stap_pl;         /* normalisation exponent                         */
    uint32_t stap_wnorm;      /* sum |w|^2 of the aux taps, Q8.24               */
    uint32_t stap_nwsat;      /* anti-windup events                             */
    /* ---- MVDR 1.14: SMI-MVDR STAP (gnss_smi.c) ---- */
    int      have_smi;        /* stap_smi in this bitstream                     */
    int      smi_active;      /* SMI drives the output (STAT.ACTIVE)            */
    double   smi_wnorm;       /* sum |w|^2 of all 16 taps as applied            */
    double   smi_ref_gain;    /* |w_REF|^2                                      */
    double   smi_lam_dbfs;    /* lambda_min per tap, dBFS (thermal floor)       */
    uint32_t smi_count;       /* snapshots in the last solved window            */
} gnss_pt_mvdr_state_t;

/* a space-time core (STAP or SMI) drives the output */
#define GNSS_PT_ST_ACTIVE(s)  (((s)->have_stap && (s)->stap_active) || ((s)->have_smi && (s)->smi_active))
#define GNSS_PT_ST_NAME(s)    (((s)->have_smi && (s)->smi_active) ? "SMI" : "STAP")

/* Window length and covariance units of the bitstream that is loaded. */
uint32_t gnss_pt_mvdr_block_n(uint32_t mvdr_version);
uint32_t gnss_pt_mvdr_cov_scale(uint32_t mvdr_version);

/* Writes the recommended adaptive configuration for THIS bitstream's version
 * (steering [1,0] power inversion, loading, hold/glide, IQ correction) to the
 * shadow registers. Does not commit and does not touch CTRL / OUT_GAIN. */
void     gnss_pt_mvdr_write_defaults(void);

/* Writes COMMIT=1 and polls it until the bit clears (the config has reached
 * the datapath) or the poll gives up. Does nothing else -- callers write the
 * shadow registers first. */
int32_t  gnss_pt_mvdr_commit(void);

void     gnss_pt_mvdr_get_state(gnss_pt_mvdr_state_t *st);

/* Prints everything in gnss_pt_mvdr_state_t plus a plain-English verdict:
 * whether ch0 is using the MVDR output, whether it is adaptive and applying
 * weights, and warnings for a stuck RX1/RX2 skew or frequent saturation.
 * Decodes the steering vector and current weights to a magnitude percentage
 * and a phase in degrees using <math.h> (sqrt/atan2); if your linker does not
 * already pull in libm (the AD9361 driver's own calibration math almost
 * always does), add -lm. */
void     gnss_pt_mvdr_print_state(void);

/* RX input level check, for setting the RX gain: P_in / P_rx2 in dBFS from
 * the covariance, noise per channel, ADC clip counts, and what to do.
 * Target: the noise floor at gnss_pt_get_rx_target_dbfs() (gnss_config.h's
 * GNSS_CFG_RX_NF_TARGET_DBFS, raised by the RX auto-level when the AD9361's
 * own floor is too high for it). */
void     gnss_pt_mvdr_print_levels(void);
void     gnss_pt_set_rx_target_dbfs(double t);
/* MOD-17: sample rate of the radio plan; the MVDR timings follow it */
void     gnss_pt_set_fs(uint32_t fs);
uint32_t gnss_pt_get_fs(void);
uint32_t gnss_pt_hold_units(void);    /* SMOOTH hold, 0.3 s              */
uint32_t gnss_pt_rel_sh(void);        /* SMOOTH glide, ~0.3 s            */
uint32_t gnss_pt_jam_avg_log2(void);  /* jammer estimate average, 1.5 ms */
uint32_t gnss_pt_aolc_sh(void);       /* AOLC loop speed, ~19 ms         */
int32_t  gnss_pt_mvdr_aolc_fast(int fast);  /* MOD-18: fast AOLC after a gain step */
/* MOD-21: null profile, 0 = MVDR (all add-ons), 1 = plain power inversion,
 * MOD-24: 2 = STAP (space-time power inversion, MVDR window >= 1.13; on an
 * older bitstream 2 falls back to 0).
 * MOD-25: 3 = SMI-MVDR STAP (sample-matrix inversion, MVDR window >= 1.14;
 * falls back to 2, then 0). Takes effect at the next
 * gnss_pt_mvdr_write_defaults() + commit. */
int      gnss_pt_profile_effective(void);   /* the profile the hardware can run */
int      gnss_pt_get_profile(void);
void     gnss_pt_set_profile(int p);
int      gnss_pt_stap_available(void);
uint32_t gnss_pt_get_stap_mu(void);
void     gnss_pt_set_stap_mu(uint32_t mu);
/* MOD-24: the jammer-suppression report (both cores). Prints the print-scale
 * suppression on the 0..72 dB (12-bit) scale, the JNR at the ADC (its
 * ceiling), the ACTUAL cancellation and why it reads below 72 dB.
 * Returns 0, or -1 if there is nothing to report (no MVDR core / no data). */
int      gnss_pt_print_suppression(void);
/* One-line summary for the STATUS line: fills the three figures in dB
 * (actual = -1 when the residual is under the noise: then *actual_lo is the
 * lower bound). Returns 0 if valid. */
int      gnss_pt_supp_summary(double *print_db, double *jnr_db,
                              double *actual_db, int *actual_lo);
/* MOD-25: every suppression figure, for the gnss_check? dashboard */
typedef struct {
    int    ok;            /* figures valid (coherent window, NF_EST, weights) */
    int    have_jam;      /* jammer above the noise                           */
    int    space_time;    /* STAP or SMI drives the output                    */
    double print_db, jnr_db, actual_db, res_db;
    int    actual_lo;     /* actual_db is a lower bound                       */
    double noise_dbfs;    /* floor the figures use                            */
    int    nf_src;        /* 0 covariance, 1 SMI lambda_min, 2 learned        */
    double cov_floor_dbfs;
    double jfloor_dbc;    /* jammer-derived floor vs jammer, -99 = none       */
    double jam_dbfs;      /* jammer at the ADC                                */
    double coh_lim_db;    /* one-weight (1-coh) limit                          */
} gnss_pt_supp_info_t;
int      gnss_pt_supp_info(gnss_pt_supp_info_t *i);
/* MOD-26: thermal floor for the SMI-based report (see gnss_passthrough.c) */
int      gnss_pt_thermal_now(double smi_lam_dbfs, int smi_ok, int32_t gain_now,
                             double *nf_dbfs, int *src);
int      gnss_pt_thermal_clean(double *nf_dbfs, int32_t *gain_db);
/* MOD-26b: R11/R22 plausibility (1 sane, 0 corrupt, -1 no window) and
 * whether a corrupt one was ever seen (with its powers, dBFS) */
int      gnss_pt_cov_check(const gnss_pt_mvdr_state_t *s, double *p1_dbfs, double *p2_dbfs);
int      gnss_pt_cov_bad_seen(double *p1_dbfs, double *p2_dbfs);
double   gnss_pt_get_rx_target_dbfs(void);

/* OUT_GAIN (MVDR >= 1.4). n = 0..7, gain 2^n. Writes and commits.
 * Also turns AOLC off (manual gain). */
int32_t  gnss_pt_mvdr_set_out_gain(uint32_t n);
/* Automatic output level control (MVDR >= 1.10). target_db = 0: AOLC off
 * (back to the manual 2^n that was set); otherwise the output noise floor is
 * held at -target_db dBFS (6..40). RAISES TX1 by up to ~45 dB. */
int32_t  gnss_pt_mvdr_set_aolc(uint32_t target_db);
/* MOD-25: output-clipping back-off. The monitor lowers the AOLC target by
 * step_db (> 0) while the DAC clips and raises it back (< 0) when clean;
 * the operator's target (gnss_aolc= / boot) is the base, the back-off is
 * 0..GNSS_PT_AOLC_BACKOFF_MAX dB on top (target -24 -> up to -36 dBFS).
 * Returns the back-off now in use (dB), or -1 if AOLC is off. */
#define GNSS_PT_AOLC_BACKOFF_MAX     12U
int      gnss_pt_aolc_backoff(int step_db);
uint32_t gnss_pt_aolc_backoff_db(void);
/* MOD-25: THERMAL noise floor for the suppression figures. NF_EST (the
 * covariance floor) also contains jammer energy that RX1/RX2 do not share
 * linearly (IQ image, decorrelation) and then hides it as "noise". The
 * monitor learns the floor with no jammer (gnss_pt_thermal_learn) and keeps
 * the RX gain current (gnss_pt_thermal_gain); with SMI running its
 * lambda_min is a measured floor even under a jammer and is used first. */
void     gnss_pt_thermal_learn(double nf_dbfs, int32_t gain_db);
void     gnss_pt_thermal_learn_floor(double floor_dbfs, int32_t gain_db);   /* MOD-26b */
void     gnss_pt_thermal_gain(int32_t gain_db);
/* IQ-image correction (MVDR >= 1.9): mode 1 = on, 0 = off, 2 = clear + on. */
int32_t  gnss_pt_mvdr_set_iq(int mode);
void     gnss_pt_mvdr_print_iq(void);
/* Jammer coherence detect threshold COH_SH (MVDR >= 1.8), 1..15. */
int32_t  gnss_pt_mvdr_set_coh(uint32_t coh_sh);
/* Prints OUT_GAIN, output clip counter, the estimated DAC level and a
 * suggested n for a noise-only DAC level of about -30 dBFS. */
void     gnss_pt_mvdr_print_out_gain(void);

/* Measured l_clk in Hz (0 if the counter reads 0). */
uint32_t gnss_pt_lclk_hz(void);
/* Prints measured l_clk, the value expected from rx_rate_hz (4 x fs in
 * LVDS 2R2T; pass 0 if unknown) and a warning when it exceeds the
 * frequency the design's timing was closed at. */
void     gnss_pt_print_lclk(uint32_t rx_rate_hz);

/* ---- jammer estimator (GNSS-CRPA MOD-12) ---------------------------------- */
typedef struct {
    int      available;       /* MVDR window >= 1.2                            */
    int      coherent;        /* all results from ONE estimate                 */
    int      present, valid, sin_clip, no_signal, noise_zero;
    int16_t  theta_cdeg;      /* angle from broadside, 0.01 deg                */
    int16_t  psi_cdeg;        /* calibrated RX1-RX2 phase, 0.01 deg            */
    int16_t  psi_raw_bin;     /* uncalibrated phase, binary angle              */
    int16_t  psi_raw_cdeg;    /* uncalibrated phase, 0.01 deg                  */
    int16_t  jam_dbfs_q8;     /* jammer power per element, dBFS Q8             */
    int16_t  noise_dbfs_q8;   /* noise floor per element, dBFS Q8              */
    int16_t  jnr_db_q8;       /* JNR, dB Q8                                    */
    uint32_t n_est, n_drop;
    int      cfg_enable, cfg_invert, cfg_avg_log2;
    int16_t  cfg_cal_bin;
    uint16_t cfg_k_q14;
    int16_t  cfg_on_q8, cfg_off_q8;
    /* MVDR >= 1.11: per channel, dBFS Q8 (0x8000 = none) */
    int      have_ch;
    int16_t  j1_q8, j2_q8;    /* average jammer power RX1 / RX2                */
    int16_t  pk1_q8, pk2_q8;  /* peak (on) jammer power RX1 / RX2              */
    int16_t  n1_q8, n2_q8;    /* noise floor RX1 / RX2                         */
} gnss_pt_jam_state_t;

long     gnss_pt_jam_get_dbm_k(void);
void     gnss_pt_jam_set_dbm_k(long k_c2);
void     gnss_pt_jam_get_state(gnss_pt_jam_state_t *st);
void     gnss_pt_jam_print_state(int32_t g1, int32_t g2, int gain_ok);
void     gnss_pt_jam_monitor(uint32_t count, uint32_t period_ms,
                             int32_t g1, int32_t g2, int gain_ok);
int32_t  gnss_pt_jam_calibrate(int capture);
int32_t  gnss_pt_jam_set_threshold(int detect_db);
int32_t  gnss_pt_jam_set_invert(int invert);
int32_t  gnss_pt_jam_set_avg(uint32_t avg_log2);
int32_t  gnss_pt_jam_power_calibrate(long gen_dbm_c2, int32_t g1, int32_t g2);

#endif /* GNSS_PASSTHROUGH_H_ */
