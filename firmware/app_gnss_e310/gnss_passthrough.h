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

/* GNSS-CRPA MOD-11: CRPA_COEF(0) was the adaptation step size (alpha) fed
 * to the two-element power-inversion nulling core, Q8.8 fixed point
 * (alpha_raw / 256.0), reset default 256 (== 1.0).
 *
 * v1.6 REMOVES the standard/traditional core this fed (see
 * GNSS_PT_EXPECTED_VERSION's history below) -- CRPA_COEF(0) is now
 * unused/reserved. It is still plain RW storage on the AXI-Lite side (reads
 * back whatever was last written, same as any other CRPA_COEF slot), so
 * gnss_pt_set_crpa_alpha()/gnss_pt_get_crpa_alpha() below still work as
 * register accessors, but nothing in a v1.6+ bitstream reads this value
 * back into any algorithm. Kept for backward compatibility with anything
 * still calling them; see GNSS_PT_REG_CRPA_GAMMA_NORM/_PL and
 * GNSS_PT_CTRL_CRPA_MODE below for what actually controls nulling now. */
#define GNSS_PT_REG_CRPA_ALPHA      GNSS_PT_REG_CRPA_COEF(0)   /* RW, Q8.8, UNUSED as of v1.6 */
#define GNSS_PT_CRPA_ALPHA_FRAC_BITS 8
#define GNSS_PT_CRPA_ALPHA_DEFAULT   256U   /* 256 / 2^8 = 1.0               */

/* v1.6: the standard core's alpha is replaced by two INDEPENDENT
 * regularisers (Eq. 13's gamma), one per remaining core, so tuning one
 * mode's regulariser never silently perturbs the other's behaviour when
 * you switch modes (CRPA_COEF(1) and CRPA_COEF(2) feed separate CDC
 * synchronisers in gnss_passthrough.v -- not shared). Unlike alpha, gamma
 * is a PLAIN UNSIGNED INTEGER, not Q-format: it shares the fixed-point
 * alignment of the power sum sum|x_i|^2 inside each core, which itself has
 * 0 fractional bits because the raw ADC samples do. There is no
 * raw/float split to expose here the way alpha had one. */
#define GNSS_PT_REG_CRPA_GAMMA_NORM GNSS_PT_REG_CRPA_COEF(1)  /* RW, plain uint, normalized core's gamma */
#define GNSS_PT_REG_CRPA_GAMMA_PL   GNSS_PT_REG_CRPA_COEF(2)  /* RW, plain uint, PL-NPI core's OWN gamma  */
#define GNSS_PT_CRPA_GAMMA_DEFAULT  1U   /* matches GAMMA_INIT in hardware: Eq. 13's gamma > 0, smallest safe regulariser */

/* ---- expected identity --------------------------------------------------- */
#define GNSS_PT_EXPECTED_ID         0x47435031U
/* 1.0 = Phase-1 identity passthrough.
 * 1.1 = adds the RX->TX sample alignment stage (RX is right-aligned 12-in-16,
 *       the AD9361 DAC consumes [15:4]). A board reporting 1.0 is running a
 *       bitstream whose TX output is 24 dB low with 4 bits discarded.
 * 1.2 = replaces the Phase-1 identity core with the two-element power-
 *       inversion CRPA nulling core (see u_crpa_core in gnss_passthrough.v),
 *       but has a bug: u_crpa_core's alpha_wr was hardwired to 1'b0, so the
 *       core's internal alpha_reg never loaded CRPA_COEF(0) and stayed at
 *       its fixed reset value (1.0) forever. CRPA_COEF(0) writes and reads
 *       both worked -- the register itself is fine -- they just never
 *       reached the algorithm. A board reporting 1.2 nulls, but always at
 *       alpha=1.0; gnss_crpa_alpha= has NO EFFECT on it.
 * 1.3 = fixes the 1.2 alpha_wr bug. gnss_crpa_alpha= is genuinely live.
 *       A board reporting 1.1 or earlier still passes RX through
 *       UNMODIFIED -- alpha writes are accepted (SCRATCH-like RW) but
 *       nothing on that board consumes them, so no nulling happens no
 *       matter what firmware sends.
 * 1.4 = adds a SECOND, runtime-selectable core: normalized PI (Eq. 13,
 *       per-sample step size instead of a fixed alpha). CONTROL[4] selects
 *       it (1) vs. the standard core (0, default); CRPA_COEF(1) carries its
 *       gamma (Eq. 13's regulariser). Both cores run and adapt
 *       continuously regardless of which is selected -- switching is
 *       bumpless, not a cold restart.
 * 1.5 = adds a THIRD core, PL-NPI (piecewise-linear variable step).
 *       CONTROL[5:4] becomes a 2-bit mode select: 00=standard, 01=normalized,
 *       10=PL-NPI, 11=reserved (falls back to standard). CRPA_COEF(2)
 *       carries PL-NPI's OWN gamma, independent of CRPA_COEF(1).
 * 1.6 = REMOVES the standard/traditional core. The fixed-alpha core's loop
 *       gain has a narrow stable range at this board's real signal levels
 *       (confirmed on hardware: GPS satellites vanished at alpha=100 with
 *       no jammer present) and the normalized/PL-NPI cores cover the same
 *       ground with a self-scaling step and no alpha to mistune.
 *       CONTROL[5:4] shrinks back to CONTROL[4] (1 bit): 0 (reset default)
 *       = normalized, 1 = PL-NPI. CRPA_COEF(0)/alpha is now unused/
 *       reserved -- gnss_crpa_alpha= still writes the register (harmless)
 *       but has NO EFFECT on any core. A board reporting 1.4 or 1.5 still
 *       has the standard core AND alpha control; a board reporting 1.3 or
 *       earlier has neither CONTROL[4] mode select nor gamma control at
 *       all -- gnss_crpa_mode=/gnss_crpa_gamma_norm=/gnss_crpa_gamma_pl=
 *       write CONTROL/CRPA_COEF bits that board's bitstream does not
 *       decode the same way, if at all. */
#define GNSS_PT_EXPECTED_VERSION    0x00010006U

/* ---- CONTROL bits -------------------------------------------------------- */
#define GNSS_PT_CTRL_PASS_EN        (1U << 0)  /* 1 = RX->TX passthrough      */
#define GNSS_PT_CTRL_MUTE           (1U << 1)  /* 1 = drive zeros to the DAC  */
#define GNSS_PT_CTRL_SWAP_IQ        (1U << 2)
#define GNSS_PT_CTRL_CH1_COPY       (1U << 3)  /* ch1 TX fed from ch0         */
/* v1.6+ ONLY: CRPA core mode select. 0 = normalized PI (reset default),
 * 1 = PL-NPI. On a v1.4/v1.5 bitstream this bit (or CONTROL[5:4] on v1.5)
 * selected between the (still-present) standard core and normalized/PL-NPI
 * -- see GNSS_PT_EXPECTED_VERSION's history above before assuming this
 * meaning on anything but a v1.6+ board. On v1.3 and earlier this bit does
 * nothing at all. */
#define GNSS_PT_CTRL_CRPA_MODE      (1U << 4)
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

/* GNSS-CRPA MOD-11: CRPA nulling adaptation step size (alpha).
 * Larger alpha adapts faster but is noisier / less stable; smaller alpha is
 * slower but steadier. Raw form is the Q8.8 value the hardware takes
 * directly; the float form is alpha_raw / 256.0 for convenience on the
 * console ("gnss_crpa_alpha=1.0"). Values above 65535/256 (255.996) are
 * clamped to 16 bits -- the register cannot hold more.
 *
 * v1.6+: UNUSED. These still read/write CRPA_COEF(0) correctly (it is
 * harmless RW storage either way) but no core in a v1.6+ bitstream
 * consumes it -- see GNSS_PT_REG_CRPA_ALPHA's comment above. Kept only for
 * backward compatibility with existing callers/scripts. */
void     gnss_pt_set_crpa_alpha_raw(uint16_t alpha_q8);
uint16_t gnss_pt_get_crpa_alpha_raw(void);
void     gnss_pt_set_crpa_alpha(double alpha);
double   gnss_pt_get_crpa_alpha(void);

/* v1.6+: CONTROL[4] mode select between the two remaining CRPA cores.
 * pl_npi=0 -> normalized PI (reset default), pl_npi=nonzero -> PL-NPI.
 * Both cores run and adapt continuously regardless of which is selected,
 * so switching at runtime is bumpless, not a cold restart. */
void     gnss_pt_set_crpa_mode(int pl_npi);
int      gnss_pt_get_crpa_mode(void);   /* returns 0 (normalized) or 1 (PL-NPI) */

/* v1.6+: independent regularisers (Eq. 13's gamma), one per remaining core
 * -- CRPA_COEF(1) for normalized, CRPA_COEF(2) for PL-NPI, deliberately not
 * shared so tuning one mode never silently perturbs the other. Plain
 * unsigned integers, NOT Q-format like alpha was -- see
 * GNSS_PT_REG_CRPA_GAMMA_NORM's comment above for why there is no
 * raw/float split here. */
void     gnss_pt_set_crpa_gamma_norm(uint32_t gamma);
uint32_t gnss_pt_get_crpa_gamma_norm(void);
void     gnss_pt_set_crpa_gamma_pl(uint32_t gamma);
uint32_t gnss_pt_get_crpa_gamma_pl(void);

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
    uint16_t crpa_alpha_raw;      /* GNSS-CRPA MOD-11, Q8.8. v1.6+: unused, see above */
    int      crpa_mode;           /* v1.6+: 0=normalized, 1=PL-NPI. 0 on pre-v1.6 boards too, harmlessly */
    uint32_t crpa_gamma_norm_raw; /* v1.6+: normalized core's gamma, plain uint */
    uint32_t crpa_gamma_pl_raw;   /* v1.6+: PL-NPI core's OWN gamma, plain uint */
} gnss_pt_state_t;

void     gnss_pt_get_state(gnss_pt_state_t *st);
void     gnss_pt_print_state(void);

/* Confirms samples are actually moving through the block by sampling the
 * counters twice, separated by delay_ms (requirement 59).
 * Returns GNSS_PT_OK only if both RX and TX counters advanced, and -- when
 * passthrough is on and unmuted -- the TX snapshot is left-aligned. */
int32_t  gnss_pt_check_dataflow(uint32_t delay_ms);

#endif /* GNSS_PASSTHROUGH_H_ */
