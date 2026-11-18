/******************************************************************************
 *  gnss_passthrough.c
 *  Driver for the project's custom FPGA processing block.
 *  Original work for the ANTSDR E310 V1 GNSS CRPA project.
 *****************************************************************************/

#include "gnss_passthrough.h"
#include "gnss_smi.h"         /* MOD-25: SMI-MVDR STAP */
#include "gnss_report.h"      /* MOD-26: SMI figures for gnss_supp? */
#include "gnss_mvdr_steer.h"  /* MOD-25c: steering latch (gnss_steer= / gnss_beam=) */
#include "gnss_config.h"   /* MOD-15c: RX noise target */
#include "no_os_axi_io.h"
#include "no_os_delay.h"
#include <stdio.h>
/* sqrt()/atan2() for decoding the steering vector and current weights to a
 * magnitude percentage and a phase in degrees in gnss_pt_mvdr_print_state().
 * Only the CALCULATION uses floating point -- every printed value below is
 * cast to a plain long first, so this does not depend on printf() having
 * float support. It does need libm linked in for sqrt/atan2 themselves; the
 * AD9361 driver's own filter/calibration code already uses floating point
 * extensively, so libm is almost certainly already linked. If you see
 * "undefined reference to sqrt/atan2", add -lm to the linker flags. */
#include <math.h>
#include <stdlib.h>   /* labs() */
#include <string.h>   /* memset() */
#include "gnss_quiet.h"     /* LVDS console: engine output captured, see gnss_quiet.h */
#define GNSS_QUIET_THIS_FILE
#undef  printf
#define printf        gnss_qprintf
#undef  console_print
#define console_print gnss_qprintf

uint32_t gnss_pt_read(uint32_t offset)
{
    uint32_t v = 0;
    no_os_axi_io_read(GNSS_PT_BASEADDR, offset, &v);
    return v;
}

void gnss_pt_write(uint32_t offset, uint32_t value)
{
    no_os_axi_io_write(GNSS_PT_BASEADDR, offset, value);
}

int32_t gnss_pt_probe(void)
{
    uint32_t id  = gnss_pt_read(GNSS_PT_REG_ID);
    uint32_t ver = gnss_pt_read(GNSS_PT_REG_VERSION);

    if (id != GNSS_PT_EXPECTED_ID) {
        printf("gnss_pt: ID mismatch at 0x%08lx: read 0x%08lx, expected 0x%08lx\n",
               (unsigned long)GNSS_PT_BASEADDR,
               (unsigned long)id, (unsigned long)GNSS_PT_EXPECTED_ID);
        printf("gnss_pt: the bitstream may not contain gnss_passthrough, or the\n"
               "         base address does not match the block design.\n");
        return GNSS_PT_ERR_ID;
    }

    /* A live read/write proves the AXI path works in both directions, which an
     * ID read alone does not. */
    const uint32_t pattern = 0xA5A5F00DU;
    uint32_t saved = gnss_pt_read(GNSS_PT_REG_SCRATCH);
    gnss_pt_write(GNSS_PT_REG_SCRATCH, pattern);
    uint32_t back = gnss_pt_read(GNSS_PT_REG_SCRATCH);
    gnss_pt_write(GNSS_PT_REG_SCRATCH, saved);

    if (back != pattern) {
        printf("gnss_pt: SCRATCH readback failed: wrote 0x%08lx, read 0x%08lx\n",
               (unsigned long)pattern, (unsigned long)back);
        return GNSS_PT_ERR_SCRATCH;
    }

    printf("gnss_pt: present at 0x%08lx, version %lu.%lu\n",
           (unsigned long)GNSS_PT_BASEADDR,
           (unsigned long)(ver >> 16), (unsigned long)(ver & 0xFFFFU));

    /* A version mismatch is not fatal -- the register map is compatible -- but
     * it means the bitstream on the board is not the one this firmware was
     * built against, and the differences matter for TX. Say so rather than
     * letting a stale bitstream masquerade as a current one. */
    if (ver != GNSS_PT_EXPECTED_VERSION) {
        printf("gnss_pt: WARNING version mismatch. Firmware expects %lu.%lu,\n"
               "         the bitstream reports %lu.%lu. Known differences:\n"
               "         1.0 -- TX output is missing the sample alignment\n"
               "                stage and is 24 dB low.\n"
               "         1.1 -- no MVDR window; gnss_mvdr= will report the\n"
               "                core missing and refuse to run.\n"
               "         Reprogram the FPGA to match this firmware.\n",
               (unsigned long)(GNSS_PT_EXPECTED_VERSION >> 16),
               (unsigned long)(GNSS_PT_EXPECTED_VERSION & 0xFFFFU),
               (unsigned long)(ver >> 16), (unsigned long)(ver & 0xFFFFU));
    }

    /* The MVDR window has its own version (mvdr_regs.v). Report it at boot
     * so a stale bitstream is visible before anyone reads a C/N0 number. */
    if (gnss_pt_read(GNSS_PT_REG_MVDR_ID) == GNSS_PT_MVDR_EXPECTED_ID) {
        uint32_t mv = gnss_pt_read(GNSS_PT_REG_MVDR_VERSION);
        printf("gnss_pt: MVDR window version %lu.%lu%s\n",
               (unsigned long)(mv >> 16), (unsigned long)(mv & 0xFFFFU),
               (mv < GNSS_PT_MVDR_EXPECTED_VERSION)
                   ? "  -- OLDER than this firmware expects (1.14, SMI-STAP): reprogram the FPGA"
                   : "");
    }
    return GNSS_PT_OK;
}

void gnss_pt_set_passthrough(int enable)
{
    uint32_t c = gnss_pt_read(GNSS_PT_REG_CONTROL);
    if (enable) { c |= GNSS_PT_CTRL_PASS_EN; }
    else        { c &= ~GNSS_PT_CTRL_PASS_EN; }
    gnss_pt_write(GNSS_PT_REG_CONTROL, c);
    printf("gnss_pt: passthrough %s\n", enable ? "ENABLED (RX -> TX)"
                                               : "disabled (vendor DMA drives TX)");
}

void gnss_pt_set_mute(int mute)
{
    uint32_t c = gnss_pt_read(GNSS_PT_REG_CONTROL);
    if (mute) { c |= GNSS_PT_CTRL_MUTE; }
    else      { c &= ~GNSS_PT_CTRL_MUTE; }
    gnss_pt_write(GNSS_PT_REG_CONTROL, c);
    printf("gnss_pt: output %s\n", mute ? "MUTED" : "unmuted");
}

void gnss_pt_clear_counters(void)
{
    uint32_t c = gnss_pt_read(GNSS_PT_REG_CONTROL);
    gnss_pt_write(GNSS_PT_REG_CONTROL, c |  GNSS_PT_CTRL_CNT_CLEAR);
    no_os_mdelay(1);
    gnss_pt_write(GNSS_PT_REG_CONTROL, c & ~GNSS_PT_CTRL_CNT_CLEAR);
}

void gnss_pt_get_state(gnss_pt_state_t *st)
{
    if (!st) { return; }

    uint32_t lvl = gnss_pt_read(GNSS_PT_REG_FIFO_LEVEL);
    uint32_t rxs = gnss_pt_read(GNSS_PT_REG_RX_SNAPSHOT_CH0);
    uint32_t txs = gnss_pt_read(GNSS_PT_REG_TX_SNAPSHOT_CH0);

    st->id              = gnss_pt_read(GNSS_PT_REG_ID);
    st->version         = gnss_pt_read(GNSS_PT_REG_VERSION);
    st->control         = gnss_pt_read(GNSS_PT_REG_CONTROL);
    st->status          = gnss_pt_read(GNSS_PT_REG_STATUS);
    st->rx_count_ch0    = gnss_pt_read(GNSS_PT_REG_RX_COUNT_CH0);
    st->tx_count_ch0    = gnss_pt_read(GNSS_PT_REG_TX_COUNT_CH0);
    st->rx_count_ch1    = gnss_pt_read(GNSS_PT_REG_RX_COUNT_CH1);
    st->tx_count_ch1    = gnss_pt_read(GNSS_PT_REG_TX_COUNT_CH1);
    st->overflow_count  = gnss_pt_read(GNSS_PT_REG_OVERFLOW_COUNT);
    st->underflow_count = gnss_pt_read(GNSS_PT_REG_UNDERFLOW_COUNT);

    st->fifo_level_ch0  =  lvl        & 0x3FU;
    st->fifo_level_ch1  = (lvl >> 8)  & 0x3FU;
    st->rx_i0 = (int16_t)( rxs        & 0xFFFFU);
    st->rx_q0 = (int16_t)((rxs >> 16) & 0xFFFFU);
    st->tx_i0 = (int16_t)( txs        & 0xFFFFU);
    st->tx_q0 = (int16_t)((txs >> 16) & 0xFFFFU);

    st->passthrough_enabled = (st->status & GNSS_PT_ST_PASS_EN_SYNCED) ? 1 : 0;
    st->overflow_sticky     = (st->status & GNSS_PT_ST_OVERFLOW)  ? 1 : 0;
    st->underflow_sticky    = (st->status & GNSS_PT_ST_UNDERFLOW) ? 1 : 0;
}

void gnss_pt_print_state(void)
{
    gnss_pt_state_t s;
    gnss_pt_get_state(&s);

    printf("\n--- gnss_passthrough @ 0x%08lx ---\n", (unsigned long)GNSS_PT_BASEADDR);
    printf("  id/version     : 0x%08lx / %lu.%lu\n",
           (unsigned long)s.id, (unsigned long)(s.version >> 16),
           (unsigned long)(s.version & 0xFFFFU));
    printf("  control        : 0x%08lx\n", (unsigned long)s.control);
    printf("  status         : 0x%08lx\n", (unsigned long)s.status);
    printf("  passthrough    : %s\n", s.passthrough_enabled ? "ON" : "off");
    printf("  adc enable i/q : %d / %d\n",
           (s.status & GNSS_PT_ST_ADC_EN_I0) ? 1 : 0,
           (s.status & GNSS_PT_ST_ADC_EN_Q0) ? 1 : 0);
    /* dac_enable is axi_ad9361's read-back of (dac_data_sel == 4'h2), i.e.
     * "this DAC channel is taking the data we hand it". It is NOT driven by
     * the valid strobes, so TX COUNT can advance happily while these read 0
     * and every sample is discarded. Spell that out -- reading 0/0 as
     * harmless is exactly the mistake made on 2026-09-11. */
    printf("  dac enable i/q : %d / %d   %s\n",
           (s.status & GNSS_PT_ST_DAC_EN_I0) ? 1 : 0,
           (s.status & GNSS_PT_ST_DAC_EN_Q0) ? 1 : 0,
           ((s.status & GNSS_PT_ST_DAC_EN_I0) &&
            (s.status & GNSS_PT_ST_DAC_EN_Q0))
               ? "(DAC source = DMA: our samples reach the AD9361)"
               : "(DAC source NOT DMA: the AD9361 is DISCARDING our samples)");
    printf("  rx count ch0   : %lu\n", (unsigned long)s.rx_count_ch0);
    printf("  tx count ch0   : %lu\n", (unsigned long)s.tx_count_ch0);
    printf("  rx count ch1   : %lu\n", (unsigned long)s.rx_count_ch1);
    printf("  tx count ch1   : %lu\n", (unsigned long)s.tx_count_ch1);
    printf("  fifo level     : ch0=%lu ch1=%lu\n",
           (unsigned long)s.fifo_level_ch0, (unsigned long)s.fifo_level_ch1);
    printf("  overflow       : count=%lu sticky=%d\n",
           (unsigned long)s.overflow_count, s.overflow_sticky);
    printf("  underflow      : count=%lu sticky=%d\n",
           (unsigned long)s.underflow_count, s.underflow_sticky);
    /* The two lines below are in DIFFERENT formats, and that is correct.
     * RX is 12-bit right-aligned in 16 bits; the AD9361 DAC consumes [15:4],
     * so the block left-aligns on the way out.
     *
     * DO NOT expect tx == 16 * rx. The two snapshots latch on different
     * strobes -- rx on adc_valid, tx on dac_valid -- with the elastic buffer
     * between them, so they are almost never the same sample. The check that
     * IS valid is that a left-aligned value has its low four bits zero; see
     * gnss_pt_check_dataflow(), which tests exactly that. */
    printf("  last rx sample : I=%6d  Q=%6d   (12-bit right-aligned)\n",
           s.rx_i0, s.rx_q0);
    printf("  last tx sample : I=%6d  Q=%6d   (12-bit left-aligned, low nibble 0)\n",
           s.tx_i0, s.tx_q0);
    printf("-----------------------------------\n");
}

int32_t gnss_pt_check_dataflow(uint32_t delay_ms)
{
    gnss_pt_state_t a, b;

    gnss_pt_get_state(&a);
    no_os_mdelay(delay_ms);
    gnss_pt_get_state(&b);

    uint32_t d_rx = b.rx_count_ch0 - a.rx_count_ch0;   /* wraps correctly */
    uint32_t d_tx = b.tx_count_ch0 - a.tx_count_ch0;

    printf("gnss_pt: over %lu ms, rx advanced %lu, tx advanced %lu\n",
           (unsigned long)delay_ms, (unsigned long)d_rx, (unsigned long)d_tx);

    if (d_rx == 0) {
        printf("gnss_pt: NO RX SAMPLES. The AD9361 RX path is not delivering data.\n"
               "         Check AD9361 init, ENSM state and that RX is enabled.\n");
        return GNSS_PT_ERR_NO_RX;
    }
    if (d_tx == 0) {
        printf("gnss_pt: NO TX REQUESTS. The AD9361 DAC side is not requesting data.\n"
               "         Check that TX is enabled and the ENSM is in a TX state.\n");
        return GNSS_PT_ERR_NO_TX;
    }
    if (b.overflow_sticky) {
        printf("gnss_pt: WARNING overflow flagged - RX is outrunning TX.\n");
    }
    if (b.underflow_sticky) {
        printf("gnss_pt: WARNING underflow flagged - TX is outrunning RX.\n");
    }

    /* ---- TX sample alignment -------------------------------------------
     * axi_ad9361's DAC consumes dma_data[15:4], so every word this block
     * drives in passthrough must be left-aligned: {sample[11:0], 4'b0000}.
     * Its low four bits are therefore ALWAYS zero.
     *
     * That is a property of a single word, so unlike a comparison against the
     * RX snapshot it does not depend on the two snapshots holding the same
     * sample -- they do not, and cannot, with an elastic buffer between them.
     *
     * Only meaningful while passthrough is on and the output is not muted;
     * muted drives 0x0000, which passes trivially and proves nothing. */
    if (b.passthrough_enabled && !(b.control & GNSS_PT_CTRL_MUTE)) {
        uint32_t txs = gnss_pt_read(GNSS_PT_REG_TX_SNAPSHOT_CH0);
        uint32_t ti  =  txs        & 0xFFFFU;
        uint32_t tq  = (txs >> 16) & 0xFFFFU;

        if (((ti & 0xFU) != 0U) || ((tq & 0xFU) != 0U)) {
            printf("gnss_pt: TX ALIGNMENT FAIL - snapshot I=0x%04lx Q=0x%04lx has\n"
                   "         non-zero low bits. The samples are NOT left-aligned,\n"
                   "         so the DAC is receiving sample>>4: 24 dB low with the\n"
                   "         four LSBs discarded. Expect a v1.0 bitstream.\n",
                   (unsigned long)ti, (unsigned long)tq);
            return GNSS_PT_ERR_ALIGNMENT;
        }
        printf("gnss_pt: TX alignment OK - snapshot I=0x%04lx Q=0x%04lx, low nibble\n"
               "         zero on both, i.e. 12-bit data left-aligned into [15:4]\n",
               (unsigned long)ti, (unsigned long)tq);
    }

    return GNSS_PT_OK;
}

/******************************************************************************
 *  MVDR adaptive beamformer (GNSS-CRPA MOD-11).
 *  Register map: see gnss_passthrough.h. Authoritative bit layout: the header
 *  comment of Source/HDL/mvdr_regs.v.
 *****************************************************************************/

/* ---- MOD-25c: steering latch for the single-weight MVDR (A1/A2) ----------
 * Written by gnss_steer= (gnss_mvdr_steer.c) and gnss_beam= (gnss_mvdr_auto.c),
 * re-applied by gnss_pt_mvdr_write_defaults() on every MVDR (re)start. */
static uint32_t steer_a1 = 0x00004000U, steer_a2 = 0x00000000U;
static int      steer_src = GNSS_PT_STEER_PI;
static long     steer_cdeg = 0;
static int      steer_have = 0;

int32_t gnss_pt_steer_apply(uint32_t a1, uint32_t a2, int src, long angle_cdeg, int have_angle)
{
    steer_a1 = a1;  steer_a2 = a2;  steer_src = src;
    steer_cdeg = angle_cdeg;  steer_have = have_angle ? 1 : 0;
    if (gnss_pt_read(GNSS_PT_REG_MVDR_ID) != GNSS_PT_MVDR_EXPECTED_ID) {
        return GNSS_PT_ERR_MVDR_COMMIT;          /* latched; used at the next MVDR start */
    }
    gnss_pt_write(GNSS_PT_REG_MVDR_A1, a1);
    gnss_pt_write(GNSS_PT_REG_MVDR_A2, a2);
    return gnss_pt_mvdr_commit();
}

int32_t gnss_pt_steer_reset(void)
{
    return gnss_pt_steer_apply(0x00004000U, 0x00000000U, GNSS_PT_STEER_PI, 0, 0);
}

void gnss_pt_steer_get(uint32_t *a1, uint32_t *a2, int *src, long *angle_cdeg, int *have_angle)
{
    if (a1)         { *a1 = steer_a1; }
    if (a2)         { *a2 = steer_a2; }
    if (src)        { *src = steer_src; }
    if (angle_cdeg) { *angle_cdeg = steer_cdeg; }
    if (have_angle) { *have_angle = steer_have; }
}

const char *gnss_pt_steer_src_name(int src)
{
    switch (src) {
    case GNSS_PT_STEER_MANUAL: return "manual (gnss_steer=)";
    case GNSS_PT_STEER_AUTO:   return "auto noise eigenvector (gnss_beam=)";
    case GNSS_PT_STEER_BROAD:  return "auto broadside beam (gnss_beam=2)";
    default:                   return "power inversion [1,0]";
    }
}

int32_t gnss_pt_mvdr_commit(void)
{
    uint32_t v;
    int i;

    gnss_pt_write(GNSS_PT_REG_MVDR_COMMIT, 1U);
    /* This is a tight poll, not a timed wait: on real hardware the datapath
     * clock is free-running, so a commit reaches it within a handful of
     * clocks (microseconds at most) and the loop below exits almost
     * immediately. The bound just stops a genuinely stuck commit (e.g. the
     * MVDR window is not actually present) from hanging the caller forever. */
    for (i = 0; i < 100000; i++) {
        v = gnss_pt_read(GNSS_PT_REG_MVDR_COMMIT);
        if ((v & 1U) == 0U) {
            return GNSS_PT_OK;
        }
    }
    printf("gnss_pt: MVDR COMMIT did not clear after %ld reads; config not applied\n",
           (long)i);
    return GNSS_PT_ERR_MVDR_COMMIT;
}

/* One 64-bit register pair. HI is already the sign-extended upper half, so
 * assembling as unsigned and casting once is exact (and avoids left-shifting
 * a negative signed value, which is undefined behaviour in C). */
static int64_t gnss_pt_read_s64(uint32_t lo_off, uint32_t hi_off)
{
    uint32_t lo = gnss_pt_read(lo_off);
    uint32_t hi = gnss_pt_read(hi_off);
    return (int64_t)(((uint64_t)hi << 32) | (uint64_t)lo);
}

uint32_t gnss_pt_mvdr_block_n(uint32_t mvdr_version)
{
    return (mvdr_version >= GNSS_PT_MVDR_VERSION_DCRM) ? GNSS_PT_MVDR_BLOCK_N
                                                       : GNSS_PT_MVDR_BLOCK_N_OLD;
}

uint32_t gnss_pt_mvdr_cov_scale(uint32_t mvdr_version)
{
    return (mvdr_version >= GNSS_PT_MVDR_VERSION_IQ) ? GNSS_PT_MVDR_COV_SCALE : 1U;
}

/* The recommended adaptive configuration, version by version. Written every
 * time adaptive mode is started rather than trusting what an earlier session
 * left behind. On >= 1.7 the loading is noise-referenced (EPS_NF = 8, x1 the
 * noise floor) and the old relative loading MUST stay off (EPS_REL = 63):
 * EPS_REL = 10 scales with the jammer and caps the null at ~60 dB JNR. */
/* ---- MOD-17: sample-rate-dependent timings ---------------------------------
 * The MVDR core counts in samples (windows of 8192, hops of 1024, AOLC blocks
 * of 4096), so at 4.5 MSPS everything would run 10x slower than at 45 MSPS.
 * These keep the TIMES constant at any rate (set by the radio plan). */
static uint32_t gnss_pt_fs = (uint32_t)GNSS_CFG_SAMP_RATE_HZ;
void     gnss_pt_set_fs(uint32_t fs) { if (fs > 0U) { gnss_pt_fs = fs; } }
uint32_t gnss_pt_get_fs(void)        { return gnss_pt_fs; }

static long gnss_pt_lround2(double x)            /* round(log2(x)) */
{
    return (x <= 0.0) ? 0L : (long)floor(log(x) / log(2.0) + 0.5);
}
static uint32_t gnss_pt_clampu(long v, long lo, long hi)
{
    return (uint32_t)((v < lo) ? lo : ((v > hi) ? hi : v));
}
/* null hold after the jammer leaves: 0.3 s, in units of 64 hops */
uint32_t gnss_pt_hold_units(void)
{
    return gnss_pt_clampu((long)(0.3 * (double)gnss_pt_fs / 65536.0 + 0.5), 1, 65535);
}
/* glide back to the unjammed weights: tau ~0.3 s = 2^n hops */
uint32_t gnss_pt_rel_sh(void)
{
    return gnss_pt_clampu(gnss_pt_lround2(0.3 * (double)gnss_pt_fs / 1024.0), 1, 15);
}
/* jammer estimator average: ~1.5 ms = 2^n windows (hop 1024) */
uint32_t gnss_pt_jam_avg_log2(void)
{
    return gnss_pt_clampu(gnss_pt_lround2(1.5e-3 * (double)gnss_pt_fs / 1024.0), 0, 8);
}
/* AOLC loop: tau ~19 ms (AGC_SH 8 at 55.3 MSPS; one step per 4096 samples) */
uint32_t gnss_pt_aolc_sh(void)
{
    return gnss_pt_clampu(8L + gnss_pt_lround2((double)gnss_pt_fs / 55.3e6), 1, 15);
}

static uint32_t gnss_pt_coh_user = 0;   /* MOD-19: gnss_coh= value, 0 = default */
/* MOD-21: null profile and IQ default.
 *   profile 0 = MVDR with every add-on (noise-referenced loading x1, null
 *               hold 0.3 s + glide) -- the design default
 *   profile 1 = PLAIN power inversion: the same R^-1 [1,0] weights with only
 *               a light loading (x1/8 noise), no hold, no glide -- the
 *               classic PI behaviour, for A/B tests against other builds
 * IQ-image correction is OFF by default since MOD-21: in the field a wideband
 * jammer near the LO made its estimate run away (-18..-28 dBc), and the
 * wrong correction ADDS a mirrored copy of the jammer to both channels -- a
 * second source that a 2-element array cannot null together with the first.
 * gnss_iq=1 turns it on (kept until reboot). */
/* MOD-24: profile 2 = STAP (space-time power inversion, MVDR window 1.13):
 *   2 antennas x 5 taps, adapted every sample (NLMS). The single-weight
 *   beamformer keeps running beside it (its covariance, NF_EST, coherence and
 *   jammer estimate feed every report), but the OUTPUT comes from STAP.
 *   Why: one complex weight per antenna can only null a jammer that RX1 and
 *   RX2 see as an exact scaled copy of each other. True for CW; NOT for a
 *   wideband barrage, where antenna/LNA/SAW, cable and AD9361 filter
 *   differences and the array delay change the RX1/RX2 relation across the
 *   band (field: 56 MHz barrage, coherence 0.47 -> a 2.8 dB null). The 5-tap
 *   FIRs equalise that difference. Simulated (RTL): CW 40 dB JNR -> residual
 *   0.6 dB over the noise (single weight 3.5 dB); barrage 30 dB JNR with a
 *   0.35-sample delay + filter mismatch -> 4.8 dB (single weight 23 dB, i.e.
 *   satellites 15 dB better); no jammer -> satellites unchanged.
 *   On a bitstream older than 1.13 profile 2 behaves as profile 0. */
static int      gnss_pt_profile = GNSS_CFG_NULL_PROFILE;
static uint32_t gnss_pt_stap_mu = GNSS_PT_STAP_MU_DEFAULT;
static int gnss_pt_iq_user = -1;        /* -1 = default (GNSS_CFG_IQ_DEFAULT) */
int  gnss_pt_get_profile(void)       { return gnss_pt_profile; }
void gnss_pt_set_profile(int p)      { gnss_pt_profile = (p < 0) ? 0 : (p > 3) ? 3 : p; }
/* MOD-25: 3 -> 2 -> 0 by what the bitstream has */
int  gnss_pt_profile_effective(void)
{
    int p = gnss_pt_profile;
    if (p == 3 && !gnss_smi_available()) { p = 2; }
    if (p == 2 && !gnss_pt_stap_available()) { p = 0; }
    return p;
}
int  gnss_pt_stap_available(void)
{
    /* MOD-25: the 1.14 bitstream leaves the NLMS STAP out by default (it and
     * SMI do not both fit the XC7Z020): STAP_ST[31] = 1 then */
    return (gnss_pt_read(GNSS_PT_REG_MVDR_ID) == GNSS_PT_MVDR_EXPECTED_ID &&
            gnss_pt_read(GNSS_PT_REG_MVDR_VERSION) >= GNSS_PT_MVDR_VERSION_STAP &&
            !(gnss_pt_read(GNSS_PT_REG_STAP_ST) & GNSS_PT_STAP_ST_ABSENT)) ? 1 : 0;
}
uint32_t gnss_pt_get_stap_mu(void)   { return gnss_pt_stap_mu; }
void gnss_pt_set_stap_mu(uint32_t mu){ gnss_pt_stap_mu = (mu < 5U) ? 5U : (mu > 10U) ? 10U : mu; }  /* 4 is unstable at 1 sample/clock */

void gnss_pt_mvdr_write_defaults(void)
{
    uint32_t v = gnss_pt_read(GNSS_PT_REG_MVDR_VERSION);

    /* MOD-25c: the steering latch (power inversion [1,0] unless gnss_steer=
     * or gnss_beam= set a vector), so a restart keeps the operator's choice */
    gnss_pt_write(GNSS_PT_REG_MVDR_A1,         steer_a1);
    gnss_pt_write(GNSS_PT_REG_MVDR_A2,         steer_a2);
    gnss_pt_write(GNSS_PT_REG_MVDR_EPS_ABS_LO, 0x00000000U);
    gnss_pt_write(GNSS_PT_REG_MVDR_EPS_ABS_HI, 0x00000000U);
    if (v >= GNSS_PT_MVDR_VERSION_EPSNF) {
        gnss_pt_write(GNSS_PT_REG_MVDR_EPS_REL, GNSS_PT_EPS_REL_OFF);
        gnss_pt_write(GNSS_PT_REG_MVDR_EPS_NF,  (gnss_pt_profile == 1) ? 1U : GNSS_PT_EPS_NF_DEFAULT);
    } else {
        gnss_pt_write(GNSS_PT_REG_MVDR_EPS_REL, 10U);             /* the pre-1.7 setting */
    }
    if (v >= GNSS_PT_MVDR_VERSION_SMOOTH) {
        /* MOD-19: the default unless the operator set gnss_coh= this session
         * (the register's reset value is the old 8, so it cannot be "kept") */
        uint32_t coh = gnss_pt_coh_user ? gnss_pt_coh_user : GNSS_PT_SM_COH_DEFAULT;
        gnss_pt_write(GNSS_PT_REG_MVDR_SMOOTH,
                      ((gnss_pt_profile == 1) ? 0U :         /* PI: HOLD 0, REL_SH 0 = v1 */
                       (GNSS_PT_SM_HOLD(gnss_pt_hold_units()) |
                        GNSS_PT_SM_REL_SH(gnss_pt_rel_sh()))) | GNSS_PT_SM_COH_SH(coh));
    }
    if (v >= GNSS_PT_MVDR_VERSION_IQ) {
        int on = (gnss_pt_iq_user >= 0) ? gnss_pt_iq_user : GNSS_CFG_IQ_DEFAULT;
        gnss_pt_write(GNSS_PT_REG_MVDR_IQ_CTRL, GNSS_PT_IQ_MU_SH(GNSS_PT_IQ_MU_DEFAULT) |
                      (on ? 0U : GNSS_PT_IQ_OFF));
    }
    if (v >= GNSS_PT_MVDR_VERSION_SMI) {
        /* MOD-25: SMI-MVDR STAP drives the output in profile 3 (its own
         * CTRL, effective at once; gnss_smi_poll() keeps it solved) */
        (void)gnss_smi_enable(gnss_pt_profile_effective() == 3);
    }
    if (v >= GNSS_PT_MVDR_VERSION_STAP) {
        /* MOD-24: STAP drives the output in profile 2, otherwise held at 0 */
        gnss_pt_write(GNSS_PT_REG_STAP_CTRL,
                      ((gnss_pt_profile_effective() == 2) ? GNSS_PT_STAP_EN : 0U) |
                      GNSS_PT_STAP_MU(gnss_pt_stap_mu) |
                      GNSS_PT_STAP_LEAK(GNSS_PT_STAP_LEAK_DEFAULT));
    }
}

void gnss_pt_mvdr_get_state(gnss_pt_mvdr_state_t *st)
{
    uint32_t a1, a2, status_top, mvdr_status, b0, b1;
    int tries;

    if (!st) { return; }
    memset(st, 0, sizeof(*st));

    st->id      = gnss_pt_read(GNSS_PT_REG_MVDR_ID);
    st->version = gnss_pt_read(GNSS_PT_REG_MVDR_VERSION);
    st->block_n   = gnss_pt_mvdr_block_n(st->version);
    st->cov_scale = gnss_pt_mvdr_cov_scale(st->version);

    if (st->id != GNSS_PT_MVDR_EXPECTED_ID) {
        /* Core not present (old bitstream). Everything else stays zero
         * rather than printing register values that do not mean anything. */
        return;
    }

    status_top   = gnss_pt_read(GNSS_PT_REG_STATUS);
    st->mvdr_sel = (status_top & GNSS_PT_ST_MVDR_SEL_SYNCED) ? 1 : 0;

    st->mode_shadow = (int)(gnss_pt_read(GNSS_PT_REG_MVDR_CTRL) & 0x3U);
    st->freeze      = (gnss_pt_read(GNSS_PT_REG_MVDR_CTRL) & GNSS_PT_MVDR_CTRL_FREEZE) ? 1 : 0;

    mvdr_status = gnss_pt_read(GNSS_PT_REG_MVDR_STATUS);
    st->status_raw    = mvdr_status;
    st->mode_live     = (int)(mvdr_status & GNSS_PT_MVDR_ST_MODE_MASK);
    st->have_weights  = (mvdr_status & GNSS_PT_MVDR_ST_HAVE_W)    ? 1 : 0;
    st->last_sat      = (mvdr_status & GNSS_PT_MVDR_ST_LAST_SAT)  ? 1 : 0;
    st->last_skip     = (mvdr_status & GNSS_PT_MVDR_ST_LAST_SKIP) ? 1 : 0;
    st->skew_sticky   = (mvdr_status & GNSS_PT_MVDR_ST_SKEW)      ? 1 : 0;
    st->wc_busy       = (mvdr_status & GNSS_PT_MVDR_ST_WC_BUSY)   ? 1 : 0;
    st->commit_pending= (mvdr_status & GNSS_PT_MVDR_ST_PENDING)   ? 1 : 0;

    st->n_blocks = gnss_pt_read(GNSS_PT_REG_MVDR_N_BLOCKS);
    st->n_upd    = gnss_pt_read(GNSS_PT_REG_MVDR_N_UPD);
    st->n_skip   = gnss_pt_read(GNSS_PT_REG_MVDR_N_SKIP);
    st->n_sat    = gnss_pt_read(GNSS_PT_REG_MVDR_N_SAT);
    st->n_drop   = gnss_pt_read(GNSS_PT_REG_MVDR_N_DROP);
    st->n_zero   = gnss_pt_read(GNSS_PT_REG_MVDR_N_ZERO);
    st->n_skew   = gnss_pt_read(GNSS_PT_REG_MVDR_N_SKEW);
    st->n_osat   = gnss_pt_read(GNSS_PT_REG_MVDR_N_OSAT);

    /* ---- MVDR 1.6 .. 1.10 ---- */
    st->have_peak   = (st->version >= GNSS_PT_MVDR_VERSION_DCRM)   ? 1 : 0;
    st->have_nf     = (st->version >= GNSS_PT_MVDR_VERSION_EPSNF)  ? 1 : 0;
    st->have_smooth = (st->version >= GNSS_PT_MVDR_VERSION_SMOOTH) ? 1 : 0;
    st->have_iq     = (st->version >= GNSS_PT_MVDR_VERSION_IQ)     ? 1 : 0;
    st->have_aolc   = (st->version >= GNSS_PT_MVDR_VERSION_AOLC)   ? 1 : 0;
    if (st->have_peak) {
        st->dc_rm = (mvdr_status & GNSS_PT_MVDR_ST_DC_RM) ? 1 : 0;
    }
    if (st->have_nf) {
        st->eps_nf_k = gnss_pt_read(GNSS_PT_REG_MVDR_EPS_NF) & 0xFFU;
    }
    if (st->have_smooth) {
        st->st_jam   = (mvdr_status & GNSS_PT_MVDR_ST_JAM)   ? 1 : 0;
        st->st_hold  = (mvdr_status & GNSS_PT_MVDR_ST_HOLD)  ? 1 : 0;
        st->st_glide = (mvdr_status & GNSS_PT_MVDR_ST_GLIDE) ? 1 : 0;
        st->smooth   = gnss_pt_read(GNSS_PT_REG_MVDR_SMOOTH);
        st->n_jam    = gnss_pt_read(GNSS_PT_REG_MVDR_N_JAM);
        st->n_hold   = gnss_pt_read(GNSS_PT_REG_MVDR_N_HOLD);
    }
    if (st->have_iq) {
        st->iq_ctrl  = gnss_pt_read(GNSS_PT_REG_MVDR_IQ_CTRL);
        st->iq_b1_re = (int32_t)gnss_pt_read(GNSS_PT_REG_MVDR_IQ_B1_RE);
        st->iq_b1_im = (int32_t)gnss_pt_read(GNSS_PT_REG_MVDR_IQ_B1_IM);
        st->iq_b2_re = (int32_t)gnss_pt_read(GNSS_PT_REG_MVDR_IQ_B2_RE);
        st->iq_b2_im = (int32_t)gnss_pt_read(GNSS_PT_REG_MVDR_IQ_B2_IM);
        st->n_iq_upd = gnss_pt_read(GNSS_PT_REG_MVDR_N_IQ_UPD);
    }
    if (st->have_aolc) {
        uint32_t ol = gnss_pt_read(GNSS_PT_REG_MVDR_OUT_LVL);
        st->out_lvl_q4  = (int16_t)( ol        & 0xFFFFU);
        st->out_gain_q8 = (int16_t)((ol >> 16) & 0xFFFFU);
    }
    st->have_stap = (st->version >= GNSS_PT_MVDR_VERSION_STAP &&
                     !(gnss_pt_read(GNSS_PT_REG_STAP_ST) & GNSS_PT_STAP_ST_ABSENT)) ? 1 : 0;
    if (st->have_stap) {
        uint32_t sst   = gnss_pt_read(GNSS_PT_REG_STAP_ST);
        st->stap_ctrl   = gnss_pt_read(GNSS_PT_REG_STAP_CTRL);
        st->stap_active = (sst & GNSS_PT_STAP_ST_ACTIVE) ? 1 : 0;
        st->stap_pl     = sst & 0x3FU;
        st->stap_wnorm  = gnss_pt_read(GNSS_PT_REG_STAP_WNORM);
        st->stap_nwsat  = gnss_pt_read(GNSS_PT_REG_STAP_NWSAT);
    }
    st->have_smi = (st->version >= GNSS_PT_MVDR_VERSION_SMI && gnss_smi_available()) ? 1 : 0;
    if (st->have_smi) {
        gnss_smi_state_t ss;
        gnss_smi_get_state(&ss);
        st->smi_active   = ss.hw_active;
        st->smi_wnorm    = ss.updates ? ss.wnorm : 1.0;
        st->smi_ref_gain = ss.updates ? ss.ref_gain : 1.0;
        st->smi_lam_dbfs = ss.updates ? ss.lam_dbfs : 0.0;
        st->smi_count    = ss.updates ? ss.count : 0U;
    }

    a1 = gnss_pt_read(GNSS_PT_REG_MVDR_A1);
    a2 = gnss_pt_read(GNSS_PT_REG_MVDR_A2);
    st->a1_re = (int16_t)( a1        & 0xFFFFU);
    st->a1_im = (int16_t)((a1 >> 16) & 0xFFFFU);
    st->a2_re = (int16_t)( a2        & 0xFFFFU);
    st->a2_im = (int16_t)((a2 >> 16) & 0xFFFFU);

    /* W1_RE..W2_IM are already sign-extended to 32 bits by mvdr_regs, so a
     * plain cast is the correct signed value -- no manual sign extension. */
    st->w1_re = (int32_t)gnss_pt_read(GNSS_PT_REG_MVDR_W1_RE);
    st->w1_im = (int32_t)gnss_pt_read(GNSS_PT_REG_MVDR_W1_IM);
    st->w2_re = (int32_t)gnss_pt_read(GNSS_PT_REG_MVDR_W2_RE);
    st->w2_im = (int32_t)gnss_pt_read(GNSS_PT_REG_MVDR_W2_IM);

    /* Covariance sums. Since MVDR 1.3 a new window (4096 samples, 8192
     * since 1.6) completes every 1024 samples (~23 us at 45 MSPS), and these are separate 32-bit
     * reads, so a naive read can mix two windows -- R11 from one, R22 from
     * the next, or even the LO and HI halves of one sum from different
     * windows. That would make the coherence / expected-w2 check meaningless.
     *
     * mvdr_regs publishes all status words as one snapshot every 256 clocks;
     * bracket the reads with N_BLOCKS: if it did not move, every covariance
     * register (and COV_CLIP, which is in the same snapshot) came from the
     * same window. The burst is a few microseconds of AXI reads, so one of
     * the retries almost always succeeds. */
    st->have_r12   = (st->version >= GNSS_PT_MVDR_VERSION_R12)  ? 1 : 0;
    st->have_clip  = (st->version >= GNSS_PT_MVDR_VERSION_CLIP) ? 1 : 0;
    st->have_ogain = (st->version >= GNSS_PT_MVDR_VERSION_GAIN) ? 1 : 0;
    st->out_gain_reg = st->have_ogain ? gnss_pt_read(GNSS_PT_REG_MVDR_OUT_GAIN) : 0U;
    st->out_gain   = st->out_gain_reg & 0x7U;
    st->clip1 = st->clip2 = 0;
    st->cov_coherent = 0;
    for (tries = 0; tries < 16; tries++) {
        b0 = gnss_pt_read(GNSS_PT_REG_MVDR_N_BLOCKS);
        st->cov_r11 = gnss_pt_read_s64(GNSS_PT_REG_MVDR_COV_R11_LO,
                                       GNSS_PT_REG_MVDR_COV_R11_HI);
        st->cov_r22 = gnss_pt_read_s64(GNSS_PT_REG_MVDR_COV_R22_LO,
                                       GNSS_PT_REG_MVDR_COV_R22_HI);
        if (st->have_r12) {
            st->cov_r12_re = gnss_pt_read_s64(GNSS_PT_REG_MVDR_COV_R12_RE_LO,
                                              GNSS_PT_REG_MVDR_COV_R12_RE_HI);
            st->cov_r12_im = gnss_pt_read_s64(GNSS_PT_REG_MVDR_COV_R12_IM_LO,
                                              GNSS_PT_REG_MVDR_COV_R12_IM_HI);
        } else {
            st->cov_r12_re = st->cov_r12_im = 0;
        }
        if (st->have_clip) {
            uint32_t c = gnss_pt_read(GNSS_PT_REG_MVDR_COV_CLIP);
            st->clip1 =  c        & 0xFFFFU;
            st->clip2 = (c >> 16) & 0xFFFFU;
        }
        if (st->have_peak) {
            uint32_t p = gnss_pt_read(GNSS_PT_REG_MVDR_COV_PEAK);
            st->peak1 =  p        & 0xFFFU;
            st->peak2 = (p >> 16) & 0xFFFU;
        }
        if (st->have_nf) {
            uint32_t lo = gnss_pt_read(GNSS_PT_REG_MVDR_NF_EST_LO);
            uint32_t hi = gnss_pt_read(GNSS_PT_REG_MVDR_NF_EST_HI);
            st->nf_est = ((uint64_t)(hi & 0xFFFFU) << 32) | (uint64_t)lo;
            st->coh_l2 = st->have_smooth ? ((hi >> 16) & 0x3FU) : 63U;
        }
        b1 = gnss_pt_read(GNSS_PT_REG_MVDR_N_BLOCKS);
        if (b0 == b1) {
            st->cov_coherent = 1;
            break;
        }
    }
}

/* Magnitude as a percentage of full scale (100 = magnitude 1.0), and phase in
 * degrees 0..359. full_scale is 16384 for the Q14 steering vector, 32768 for
 * the Q15 weights. Returns a plain long so the caller never has to touch a
 * float when printing. */
static long gnss_pt_mvdr_mag_pct(long re, long im, long full_scale)
{
    double mag = sqrt((double)re * (double)re + (double)im * (double)im);
    return (long)((mag * 100.0) / (double)full_scale + 0.5);
}

static long gnss_pt_mvdr_deg(long re, long im)
{
    double deg;
    if (re == 0 && im == 0) { return 0; }
    deg = atan2((double)im, (double)re) * (180.0 / 3.14159265358979323846);
    if (deg < 0.0) { deg += 360.0; }
    return (long)(deg + 0.5);
}

/* Prints v with a sign and 3 decimals WITHOUT printf float support: the
 * value is scaled to a long first, same approach as the helpers above.
 * Clamped to +/-999999.999 so a wild value cannot overflow a 32-bit long. */
static void gnss_pt_print_f3(double v)
{
    long m;
    if (v >  999999.0) { v =  999999.0; }
    if (v < -999999.0) { v = -999999.0; }
    m = (long)(v * 1000.0 + ((v >= 0.0) ? 0.5 : -0.5));
    printf("%c%ld.%03ld", (m < 0) ? '-' : '+', labs(m) / 1000L, labs(m) % 1000L);
}

/* 0 dBFS = a full-scale complex tone on the 12-bit converter, i.e.
 * |x|^2 = 2048^2 LSB^2 (the usual AD9361 convention). */
#define GNSS_PT_FS2   (2048.0 * 2048.0)

static double gnss_pt_db10(double ratio)
{
    if (ratio < 1e-12) { ratio = 1e-12; }
    return 10.0 * log10(ratio);
}

/* Reads one coherent covariance snapshot and returns 1 when R11/R22 are
 * usable; the mean powers per sample (LSB^2) go to *pin / *prx2. */
static int gnss_pt_mvdr_levels(gnss_pt_mvdr_state_t *s, double *pin, double *prx2)
{
    gnss_pt_mvdr_get_state(s);
    if (s->id != GNSS_PT_MVDR_EXPECTED_ID || !s->cov_coherent) { return 0; }
    if (s->cov_r11 <= 0 || s->cov_r22 <= 0) { return 0; }
    *pin  = (double)s->cov_r11 / ((double)s->block_n * (double)s->cov_scale);
    *prx2 = (double)s->cov_r22 / ((double)s->block_n * (double)s->cov_scale);
    return 1;
}

/* Clip counts of one window as a plain sentence. Returns 1 if any clipped. */
static int gnss_pt_print_clip(const gnss_pt_mvdr_state_t *s)
{
    if (!s->have_clip) {
        printf("  ADC clipping   : not measured (MVDR window < 1.3)\n");
        return 0;
    }
    printf("  ADC clipping   : RX1 %lu, RX2 %lu of %lu samples on a rail%s\n",
           (unsigned long)s->clip1, (unsigned long)s->clip2,
           (unsigned long)s->block_n,
           (s->clip1 || s->clip2) ? "   <-- REDUCE RX GAIN (gnss_rxgain=)" : "   (none: good)");
    return (s->clip1 || s->clip2) ? 1 : 0;
}

/* ADC headroom from COV_PEAK (MVDR >= 1.6): dB between the largest |I|/|Q| in
 * the window and full scale. Keep >= 3 dB with the jammer on. */
static void gnss_pt_print_headroom(const gnss_pt_mvdr_state_t *s)
{
    double h1, h2;
    if (!s->have_peak) { return; }
    h1 = s->peak1 ? 20.0 * log10(2048.0 / (double)s->peak1) : 99.0;
    h2 = s->peak2 ? 20.0 * log10(2048.0 / (double)s->peak2) : 99.0;
    printf("  ADC headroom   : RX1 ");
    gnss_pt_print_f3(h1);
    printf(" dB, RX2 ");
    gnss_pt_print_f3(h2);
    printf(" dB   (peak %lu / %lu LSB of 2048)%s\n",
           (unsigned long)s->peak1, (unsigned long)s->peak2,
           (h1 < 3.0 || h2 < 3.0) ? "   <-- < 3 dB: lower the RX gain" : "");
}

/* Weight consistency check: recompute the weight on the ARM from the
 * covariance the hardware reported, and compare it with what the hardware
 * applied.
 *
 * Power inversion (a = [1, 0]) with diagonal loading eps:
 *     w ~ (R + eps I)^-1 a  ->  w2 / w1 = -R21 / (R22 + eps)
 * With R12 = sum x1*conj(x2), R21 = conj(R12), so
 *     w2/w1 = -conj(R12) / (R22 + eps).
 * By Cauchy-Schwarz |R12|^2 <= R11*R22, hence |w2/w1| <= sqrt(R11/R22).
 *
 * The RATIO w2/w1 is compared, not w2 alone: since MVDR 1.3 a weight set
 * that would overflow the 18-bit Q15 range is scaled as a whole by 2^-k
 * (SAT_SCALE). That changes w1 and w2 together and leaves the ratio -- and
 * therefore the null -- exactly where it was.
 *
 * The window length cancels in every ratio, so raw sums are used as-is.
 * Only valid for power inversion (A1=[1,0], A2=0); skipped otherwise. */
static void gnss_pt_supp_report(const gnss_pt_mvdr_state_t *s);   /* MOD-24, below */
static double gnss_pt_mvdr_pout(const gnss_pt_mvdr_state_t *s);

static void gnss_pt_mvdr_print_w2_check(const gnss_pt_mvdr_state_t *s)
{
    double r11, r22, r12r, r12i, coh, bound, eps;
    double ewr, ewi, emag;              /* expected w2/w1          */
    double w1r, w1i, w2r, w2i, w1m2;    /* hardware weights        */
    double hrr, hri, hmag;              /* hardware w2/w1          */
    double dr, di, dmag, dphase;        /* hardware / expected     */
    uint32_t rel, elo, ehi;

    printf("  --- weight consistency check (ratio w2/w1) ---\n");

    if (!s->have_r12) {
        printf("  R12            : not available (MVDR window v%lu.%lu; needs v1.1+).\n",
               (unsigned long)(s->version >> 16),
               (unsigned long)(s->version & 0xFFFFU));
        return;
    }

    printf("  cov R12 re     : raw hi:lo=0x%08lx:%08lx   mean=%ld LSB^2/sample\n",
           (unsigned long)((uint32_t)((uint64_t)s->cov_r12_re >> 32)),
           (unsigned long)((uint32_t)((uint64_t)s->cov_r12_re & 0xFFFFFFFFU)),
           (long)(s->cov_r12_re / (int64_t)(s->block_n * s->cov_scale)));
    printf("  cov R12 im     : raw hi:lo=0x%08lx:%08lx   mean=%ld LSB^2/sample\n",
           (unsigned long)((uint32_t)((uint64_t)s->cov_r12_im >> 32)),
           (unsigned long)((uint32_t)((uint64_t)s->cov_r12_im & 0xFFFFFFFFU)),
           (long)(s->cov_r12_im / (int64_t)(s->block_n * s->cov_scale)));
    printf("  snapshot       : %s\n", s->cov_coherent
           ? "R11/R22/R12/clip from the same window"
           : "NOT coherent (window rolled over during every retry) -- ignore below");

    if (!s->cov_coherent) { return; }

    r11  = (double)s->cov_r11;
    r22  = (double)s->cov_r22;
    r12r = (double)s->cov_r12_re;
    r12i = (double)s->cov_r12_im;

    if (r11 <= 0.0 || r22 <= 0.0) {
        printf("  R11 or R22 <= 0 in this window: cannot compute.\n"
               "    If RX is running, a sum of |x|^2 can never be <= 0 -- the\n"
               "    covariance itself is corrupt. First suspect: the datapath\n"
               "    clock is faster than the timing constraint (see l_clk in\n"
               "    the TXRX parameters below).\n");
        return;
    }

    w1r  = (double)s->w1_re / 32768.0;
    w1i  = (double)s->w1_im / 32768.0;
    w2r  = (double)s->w2_re / 32768.0;
    w2i  = (double)s->w2_im / 32768.0;
    w1m2 = w1r * w1r + w1i * w1i;
    coh  = (r12r * r12r + r12i * r12i) / (r11 * r22);

    /* Power levels in dBFS, from the covariance and the weights actually
     * applied (the beamformer computes y = conj(w1)*x1 + conj(w2)*x2):
     *   P_out = (|w1|^2 R11 + |w2|^2 R22 + 2 Re(conj(w1) w2 R12)) / N
     *   P_dac = P_out * output gain               (what reaches TX1)
     * N includes the covariance unit scale (x16 since MVDR 1.9), so every
     * figure below is per sample in raw ADC LSB^2.
     * Noise-like signals start to clip at about -12 dBFS (peak/rms ~ 4). */
    {
        const double N = (double)s->block_n * (double)s->cov_scale;
        double cr   = w1r * w2r + w1i * w2i;      /* conj(w1)*w2 */
        double ci   = w1r * w2i - w1i * w2r;
        double pin  = r11 / N;
        double prx2 = r22 / N;
        double pout = (w1m2 * r11 + (w2r * w2r + w2i * w2i) * r22
                    + 2.0 * (cr * r12r - ci * r12i)) / N;
        /* power gain of the output stage: 4^n manual, or the AOLC gain in use */
        double g    = s->have_aolc ? pow(2.0, 2.0 * (double)s->out_gain_q8 / 256.0)
                                   : (double)(1UL << (2U * s->out_gain));
        double pdac;
        const int stap = GNSS_PT_ST_ACTIVE(s) ? 1 : 0;
        if (stap) { pout = gnss_pt_mvdr_pout(s); }   /* MOD-24: measured (OUT_LVL) */
        if (pout < 1e-3) { pout = 1e-3; }
        pdac = pout * g;

        printf("  P_in  (RX1)    : "); gnss_pt_print_f3(gnss_pt_db10(pin  / GNSS_PT_FS2));
        printf(" dBFS   (%ld LSB^2/sample)\n", (long)pin);
        printf("  P_rx2 (RX2)    : "); gnss_pt_print_f3(gnss_pt_db10(prx2 / GNSS_PT_FS2));
        printf(" dBFS   (%ld LSB^2/sample)\n", (long)prx2);
        printf("  P_out (%s)   : ", stap ? GNSS_PT_ST_NAME(s) : "MVDR"); gnss_pt_print_f3(gnss_pt_db10(pout / GNSS_PT_FS2));
        printf(" dBFS   before OUT_GAIN%s\n", stap ? " (measured, OUT_LVL)" : "");
        printf("  P_dac (to TX1) : "); gnss_pt_print_f3(gnss_pt_db10(pdac / GNSS_PT_FS2));
        printf(" dBFS   after the output gain ("); gnss_pt_print_f3(gnss_pt_db10(g));
        printf(" dB)  <- compare with jammer OFF\n");
        /* MOD-24: the suppression figures (print / JNR / ACTUAL, on the
         * 0..72 dB scale, with why it is not 72) -- see gnss_pt_supp_report */
        gnss_pt_supp_report(s);
        if (stap) {
            printf("  RX1 path gain  : ");
            gnss_pt_print_f3(gnss_pt_db10(g));
            printf(" dB vs raw RX1 (STAP reference tap = 1, output gain only)\n");
        } else if (w1m2 > 1e-12) {
            printf("  RX1 path gain  : ");
            gnss_pt_print_f3(gnss_pt_db10(w1m2 * g));
            printf(" dB vs raw RX1 (gnss_mvdr=0): |w1| ");
            gnss_pt_print_f3(gnss_pt_db10(w1m2));
            printf(" dB, output gain ");
            gnss_pt_print_f3(gnss_pt_db10(g));
            printf(" dB\n");
        }

        if (!(s->clip1 || s->clip2) &&
            (pin > GNSS_PT_FS2 / 16.0 || prx2 > GNSS_PT_FS2 / 16.0)) {
            printf("  WARNING: an input is above -12 dBFS: the ADC peaks are close\n"
                   "           to the rails. Lower the RX gain (gnss_rxgain=).\n");
        }
        if (pdac > GNSS_PT_FS2 / 16.0) {
            printf("  WARNING: P_dac is above -12 dBFS: the 12-bit output clips\n"
                   "           (N_OSAT counts it). %s\n",
                   (s->out_gain_reg & GNSS_PT_OG_AOLC_EN)
                       ? "Lower the AOLC target (e.g. gnss_aolc=24)."
                       : "Lower gnss_ogain= by one, or use gnss_aolc=24.");
        }
    }

    if (s->a1_re != 16384 || s->a1_im != 0 || s->a2_re != 0 || s->a2_im != 0) {
        printf("  (steering vector is not power inversion; ratio check skipped)\n");
        return;
    }
    if (w1m2 < 1e-12) {
        printf("  w1 = 0: no ratio to check (no weight applied yet?)\n");
        return;
    }

    /* Same diagonal loading the hardware adds (cov_prep), window-sum units:
     *   eps = EPS_ABS + ((R11 + R22) / 2) >> EPS_REL      (63 = relative off)
     *       + (EPS_NF / 8) * NF_EST                       (MVDR >= 1.7)  */
    rel = gnss_pt_read(GNSS_PT_REG_MVDR_EPS_REL) & 0x3FU;
    elo = gnss_pt_read(GNSS_PT_REG_MVDR_EPS_ABS_LO);
    ehi = gnss_pt_read(GNSS_PT_REG_MVDR_EPS_ABS_HI);
    eps = (double)elo + 4294967296.0 * (double)ehi;
    if (rel < 63U) { eps += 0.5 * (r11 + r22) / (double)(1ULL << rel); }
    if (s->have_nf) { eps += ((double)s->eps_nf_k / 8.0) * (double)s->nf_est; }

    bound = sqrt(r11 / r22);
    ewr   = -r12r / (r22 + eps);
    ewi   =  r12i / (r22 + eps);
    emag  = sqrt(ewr * ewr + ewi * ewi);
    hrr   = (w2r * w1r + w2i * w1i) / w1m2;
    hri   = (w2i * w1r - w2r * w1i) / w1m2;
    hmag  = sqrt(hrr * hrr + hri * hri);

    printf("  coherence      : |R12|^2/(R11*R22) = ");
    gnss_pt_print_f3(coh);
    printf("   (must be <= 1.000)\n");
    printf("  |w2/w1| bound  : sqrt(R11/R22)     = ");
    gnss_pt_print_f3(bound);
    printf("%s\n", (bound > 3.9) ? "   (> 3.9: RX2 is >12 dB weaker than RX1!)" : "");
    printf("  loading eps    : %ld LSB^2/sample (EPS_REL=%lu%s, EPS_NF=%lu)\n",
           (long)(eps / ((double)s->block_n * (double)s->cov_scale)), (unsigned long)rel,
           (rel < 63U && s->have_nf) ? " -- should be 63 (off) on this build" : "",
           (unsigned long)s->eps_nf_k);

    printf("  w2/w1 expected : ");
    gnss_pt_print_f3(ewr); printf(" "); gnss_pt_print_f3(ewi);
    printf("j   |.|="); gnss_pt_print_f3(emag);
    printf(" @ %ld deg\n", gnss_pt_mvdr_deg((long)(ewr * 32768.0), (long)(ewi * 32768.0)));

    printf("  w2/w1 hardware : ");
    gnss_pt_print_f3(hrr); printf(" "); gnss_pt_print_f3(hri);
    printf("j   |.|="); gnss_pt_print_f3(hmag);
    printf(" @ %ld deg\n", gnss_pt_mvdr_deg((long)(hrr * 32768.0), (long)(hri * 32768.0)));

    {
        double m1 = sqrt(w1m2);
        long   k  = (long)floor(-log(m1) / log(2.0) + 0.5);
        printf("  |w1|           : ");
        gnss_pt_print_f3(m1);
        if (k > 0) {
            printf("   = 2^-%ld: this set was SCALED (SAT_SCALE), null kept\n", k);
        } else {
            printf("   (not scaled)\n");
        }
    }

    dmag = dphase = 0.0;
    if (emag > 1e-9) {
        dr     = hrr * ewr + hri * ewi;     /* h * conj(e) */
        di     = hri * ewr - hrr * ewi;
        dmag   = hmag / emag;
        dphase = atan2(di, dr) * (180.0 / 3.14159265358979323846);
        printf("  hw/expected    : |ratio| = ");
        gnss_pt_print_f3(dmag);
        printf("   phase diff = ");
        gnss_pt_print_f3(dphase);
        printf(" deg   (ideal 1.000 / 0)\n");
    }

    /* Diagnosis, in order of what to trust first. */
    if (s->st_hold || s->st_glide) {
        printf("  DIAG: weights are %s by design (jammer just left): they are not\n"
               "        expected to match this window's covariance. Normal.\n",
               s->st_hold ? "HELD" : "GLIDING back");
    } else if (coh > 1.001) {
        printf("  DIAG: coherence > 1 is impossible -- R12 is scaled differently\n"
               "        from R11/R22, or the sums are corrupt (timing).\n");
    } else if (hmag > bound * 1.05 + 0.001) {
        printf("  DIAG: covariance is consistent but the HARDWARE |w2/w1| exceeds\n"
               "        the bound -- the weight calculator is wrong (or its\n"
               "        inputs are corrupted by a timing failure).\n");
    } else if (emag > 1e-3 && (dmag < 0.8 || dmag > 1.25 ||
                               dphase > 20.0 || dphase < -20.0)) {
        printf("  DIAG: hardware differs from this window's covariance. Normal\n"
               "        while the scene changes (weights lag ~1 window = 20 us).\n"
               "        If it stays like this with a STEADY jammer, read again;\n"
               "        persistent mismatch points at timing (l_clk below).\n");
    } else {
        printf("  DIAG: OK -- the hardware weight matches the covariance.\n");
    }
}

void gnss_pt_mvdr_print_state(void)
{
    gnss_pt_mvdr_state_t s;
    static const char *mode_name[4] = {
        "bypass (RX1 raw)", "adaptive", "fixed weights", "bypass (reserved)"
    };

    gnss_pt_mvdr_get_state(&s);

    printf("\n--- gnss_passthrough MVDR window @ 0x%08lx ---\n",
           (unsigned long)(GNSS_PT_BASEADDR + GNSS_PT_MVDR_BASE));

    if (s.id != GNSS_PT_MVDR_EXPECTED_ID) {
        printf("  MVDR ID mismatch: read 0x%08lx, expected 0x%08lx -- this\n"
               "  bitstream does not have the MVDR core. Reprogram the FPGA.\n",
               (unsigned long)s.id, (unsigned long)GNSS_PT_MVDR_EXPECTED_ID);
        printf("-----------------------------------\n");
        return;
    }

    printf("  id/version     : 0x%08lx / %lu.%lu\n",
           (unsigned long)s.id, (unsigned long)(s.version >> 16),
           (unsigned long)(s.version & 0xFFFFU));
    if (s.version < GNSS_PT_MVDR_EXPECTED_VERSION) {
        printf("  WARNING: firmware expects MVDR %lu.%lu. This bitstream lacks:%s%s%s%s%s%s\n"
               "           Reprogram the FPGA with the current build.\n",
               (unsigned long)(GNSS_PT_MVDR_EXPECTED_VERSION >> 16),
               (unsigned long)(GNSS_PT_MVDR_EXPECTED_VERSION & 0xFFFFU),
               (s.version < GNSS_PT_MVDR_VERSION_JAM)  ? " jammer-estimator" : "",
               (s.version < GNSS_PT_MVDR_VERSION_CLIP) ? " sliding-window/SAT_SCALE/COV_CLIP" : "",
               (s.version < GNSS_PT_MVDR_VERSION_GAIN) ? " OUT_GAIN" : "",
               (s.version < GNSS_PT_MVDR_VERSION_XDOM) ? " 100MHz-maths(timing@50MSPS)" : "",
               (s.version < GNSS_PT_MVDR_VERSION_STAP) ? " STAP(barrage nulling, gnss_null=2)" : "",
               (s.version < GNSS_PT_MVDR_VERSION_SMI)  ? " SMI-STAP(gnss_null=3)" : "");
    }
    printf("  ch0 DAC source : %s\n",
           s.mvdr_sel ? "MVDR beamformer output" : "raw RX1 (MVDR bypassed)");
    printf("  mode commanded : %ld (%s)   freeze=%ld   commit pending=%ld\n",
           (long)s.mode_shadow, mode_name[s.mode_shadow & 3],
           (long)s.freeze, (long)s.commit_pending);
    printf("  mode in effect : %ld (%s)   weight-calc busy=%ld\n",
           (long)s.mode_live, mode_name[s.mode_live & 3], (long)s.wc_busy);
    printf("  last run       : scaled=%ld   skipped(d<=0)=%ld\n",
           (long)s.last_sat, (long)s.last_skip);
    printf("  RX1/RX2 skew   : %s (sticky since the last cnt_clear)\n",
           s.skew_sticky ? "MISMATCH SEEN" : "none");

    if (s.have_peak) {
        printf("  mean removal   : %s\n", s.dc_rm ? "ON (covariance centred, default)"
                                                  : "OFF (CTRL dc_keep=1: raw covariance)");
    }
    if (s.have_smooth) {
        printf("  jammer (MVDR)  : %s%s%s   coherence COH_L2=%lu (detect when < %lu)\n",
               s.st_jam ? "DETECTED" : "none",
               s.st_hold ? "   HOLD (null kept after the jammer left)" : "",
               s.st_glide ? "   GLIDE (easing back)" : "",
               (unsigned long)s.coh_l2, (unsigned long)((s.smooth >> 24) & 0xFU));
        printf("                   runs with jammer=%lu  absorbed by hold=%lu\n",
               (unsigned long)s.n_jam, (unsigned long)s.n_hold);
        if (!s.st_jam && s.coh_l2 < 6U) {     /* MOD-19: satellites alone read 7..9 */
            printf("  WARNING: RX1/RX2 coherent with NO jammer detected (COH_L2 %lu).\n"
                   "           Splitter bench? The null will take the satellites too.\n"
                   "           See gnss_coh= and the guide (cable offset).\n",
                   (unsigned long)s.coh_l2);
        }
    }
    if (s.have_nf) {
        printf("  noise floor    : ");
        gnss_pt_print_f3(gnss_pt_db10((double)s.nf_est /
                         ((double)s.block_n * (double)s.cov_scale * GNSS_PT_FS2)));
        printf(" dBFS per element (NF_EST)   loading EPS_NF=%lu (x%lu/8)\n",
               (unsigned long)s.eps_nf_k, (unsigned long)s.eps_nf_k);
    }
    if (s.have_iq) {
        printf("  IQ image       : RX1 ");
        gnss_pt_print_f3(gnss_pt_db10(((double)s.iq_b1_re * s.iq_b1_re +
                                       (double)s.iq_b1_im * s.iq_b1_im) / 1099511627776.0));
        printf(" dBc  RX2 ");
        gnss_pt_print_f3(gnss_pt_db10(((double)s.iq_b2_re * s.iq_b2_re +
                                       (double)s.iq_b2_im * s.iq_b2_im) / 1099511627776.0));
        printf(" dBc  correction %s, %lu updates\n",
               (s.iq_ctrl & GNSS_PT_IQ_OFF) ? "OFF" : "on", (unsigned long)s.n_iq_upd);
    }
    printf("  counters       : windows=%lu  updates=%lu  skipped=%lu  scaled=%lu\n",
           (unsigned long)s.n_blocks, (unsigned long)s.n_upd,
           (unsigned long)s.n_skip, (unsigned long)s.n_sat);
    printf("                   dropped=%lu  zero=%lu  skew=%lu  out_clipped=%lu\n",
           (unsigned long)s.n_drop, (unsigned long)s.n_zero,
           (unsigned long)s.n_skew, (unsigned long)s.n_osat);
    printf("                   (window = %lu samples, a new one every %lu:\n"
           "                    %lu us at %lu.%lu MSPS. scaled = weight set divided\n"
           "                    by 2^k to fit 18 bits; the null is kept)\n",
           (unsigned long)s.block_n, (unsigned long)GNSS_PT_MVDR_HOP_N,
           (unsigned long)(1024000000ULL / (gnss_pt_fs ? gnss_pt_fs : 1U)),
           (unsigned long)(gnss_pt_fs / 1000000U), (unsigned long)((gnss_pt_fs % 1000000U) / 100000U));
    if (s.have_aolc && (s.out_gain_reg & GNSS_PT_OG_AOLC_EN)) {
        printf("  output level   : AOLC ON, target -%lu dBFS: measured ",
               (unsigned long)((s.out_gain_reg >> 16) & 0xFFU));
        gnss_pt_print_f3(3.0103 * ((double)s.out_lvl_q4 / 16.0 - 23.0));
        printf(" dBFS, gain ");
        gnss_pt_print_f3(6.0206 * (double)s.out_gain_q8 / 256.0);
        printf(" dB (gnss_aolc=)\n");
    } else if (s.have_ogain) {
        printf("  output gain    : OUT_GAIN=%lu  (x%lu, +%lu dB on the MVDR path; gnss_ogain=)\n",
               (unsigned long)s.out_gain, (unsigned long)(1UL << s.out_gain),
               (unsigned long)(6UL * s.out_gain));
        if (s.have_aolc) {
            printf("                   measured output ");
            gnss_pt_print_f3(3.0103 * ((double)s.out_lvl_q4 / 16.0 - 23.0));
            printf(" dBFS (AOLC off; gnss_aolc=24 recommended)\n");
        }
    }

    printf("  steering A1    : re=%ld im=%ld   (%ld%% @ %ld deg)\n",
           (long)s.a1_re, (long)s.a1_im,
           gnss_pt_mvdr_mag_pct(s.a1_re, s.a1_im, 16384),
           gnss_pt_mvdr_deg(s.a1_re, s.a1_im));
    printf("  steering A2    : re=%ld im=%ld   (%ld%% @ %ld deg)\n",
           (long)s.a2_re, (long)s.a2_im,
           gnss_pt_mvdr_mag_pct(s.a2_re, s.a2_im, 16384),
           gnss_pt_mvdr_deg(s.a2_re, s.a2_im));
    printf("  weight w1      : re=%ld im=%ld   (%ld%% @ %ld deg)\n",
           (long)s.w1_re, (long)s.w1_im,
           gnss_pt_mvdr_mag_pct(s.w1_re, s.w1_im, 32768),
           gnss_pt_mvdr_deg(s.w1_re, s.w1_im));
    printf("  weight w2      : re=%ld im=%ld   (%ld%% @ %ld deg)\n",
           (long)s.w2_re, (long)s.w2_im,
           gnss_pt_mvdr_mag_pct(s.w2_re, s.w2_im, 32768),
           gnss_pt_mvdr_deg(s.w2_re, s.w2_im));
    printf("                   (100%% = full scale; w1/w2 are Q15, 32768=1.0;\n"
           "                    A1/A2 are Q14, 16384=1.0. a2=0 is power\n"
           "                    inversion: no direction-of-arrival needed.)\n");

    /* cov_r11/cov_r22 can be too wide for a 32-bit print on some inputs, so
     * show the exact raw halves in hex AND a derived mean power per sample
     * (block sum / block length), which is what is actually useful for
     * judging signal presence and for choosing eps_abs. */
    printf("  cov R11 (ch1)  : raw hi:lo=0x%08lx:%08lx   mean power=%ld LSB^2/sample\n",
           (unsigned long)((uint32_t)((uint64_t)s.cov_r11 >> 32)),
           (unsigned long)((uint32_t)(s.cov_r11 & 0xFFFFFFFFU)),
           (long)(s.cov_r11 / (int64_t)(s.block_n * s.cov_scale)));
    printf("  cov R22 (ch2)  : raw hi:lo=0x%08lx:%08lx   mean power=%ld LSB^2/sample\n",
           (unsigned long)((uint32_t)((uint64_t)s.cov_r22 >> 32)),
           (unsigned long)((uint32_t)(s.cov_r22 & 0xFFFFFFFFU)),
           (long)(s.cov_r22 / (int64_t)(s.block_n * s.cov_scale)));
    if (s.cov_scale != 1U) {
        printf("                   (readbacks are in (LSB x 4)^2 units since MVDR 1.9;\n"
               "                    the means above are converted back to raw LSB^2)\n");
    }
    (void)gnss_pt_print_clip(&s);
    gnss_pt_print_headroom(&s);

    gnss_pt_mvdr_print_w2_check(&s);

    /* Plain-English verdict, leading with what the HARDWARE says rather than
     * what was commanded -- same habit as gnss_pt_print_state() above, for
     * the same reason (ISSUE-0003). */
    if (!s.mvdr_sel) {
        printf("  VERDICT: the MVDR core is running but ch0 is NOT using its\n"
               "           output (mvdr_sel=0). gnss_mvdr=1 routes it to TX1.\n");
    } else if (s.mode_live != (int)GNSS_PT_MVDR_MODE_ADAPTIVE) {
        printf("  VERDICT: ch0 uses the MVDR path but it is not adaptive\n"
               "           (mode=%ld). RX1 is passed through with static\n"
               "           weights, no nulling is happening.\n", (long)s.mode_live);
    } else if (s.n_blocks == 0U) {
        printf("  VERDICT: adaptive mode is set but N_BLOCKS is still 0 -- no\n"
               "           covariance window has completed yet.\n"
               "           Check back in a moment; if it stays 0, RX2 is most\n"
               "           likely not enabled at the AD9361 2R2T init level.\n"
               "           rx2_gc_mode=/rx2_rf_gain=/gnss_mvdr= cannot fix that\n"
               "           at runtime -- it is set when the radio is brought up.\n");
    } else if (!s.have_weights) {
        printf("  VERDICT: covariance blocks are arriving (N_BLOCKS=%lu) but no\n"
               "           weight update has landed yet. Read again shortly.\n",
               (unsigned long)s.n_blocks);
    } else {
        printf("  VERDICT: MVDR is ADAPTIVE and applying computed weights\n"
               "           (%lu updates so far). w2's magnitude above shows\n"
               "           how strongly channel 2 is being combined to cancel\n"
               "           the dominant signal on the pair.\n",
               (unsigned long)s.n_upd);
    }
    if (s.skew_sticky) {
        printf("  WARNING: a RX1/RX2 valid-strobe mismatch was seen. The\n"
               "           covariance may be wrong; confirm RX2 gc_mode/gain\n"
               "           match RX1 (rx2_gc_mode? / rx2_rf_gain?).\n");
    }
    if (s.n_upd > 0U && ((100UL * (unsigned long)s.n_sat) / (unsigned long)s.n_upd) > 10UL) {
        if (s.version >= GNSS_PT_MVDR_VERSION_CLIP) {
            printf("  NOTE: %lu%% of weight sets needed scaling (|w2/w1| > ~4). The\n"
                   "        null is kept, but |w2/w1| > 4 means RX2 is much weaker\n"
                   "        than RX1: check the RX2 antenna/cable and that\n"
                   "        rx1_rf_gain = rx2_rf_gain (gnss_rxgain? shows both).\n",
                   (100UL * (unsigned long)s.n_sat) / (unsigned long)s.n_upd);
        } else {
            printf("  WARNING: %lu%% of applied weight sets were CLAMPED (MVDR < 1.3:\n"
                   "           clamping moves the null). Load the current bitstream.\n",
                   (100UL * (unsigned long)s.n_sat) / (unsigned long)s.n_upd);
        }
    }
    if (s.n_osat > 0U) {
        printf("  NOTE: out_clipped=%lu output samples hit the 12-bit rails since\n"
               "        the last cnt_clear. Rising fast -> a lower output level:\n"
               "        gnss_aolc=24 (AOLC on) or gnss_ogain= one lower (manual).\n",
               (unsigned long)s.n_osat);
    }
    printf("-----------------------------------\n");
}

/* MVDR output power per sample (LSB^2) BEFORE OUT_GAIN, from one coherent
 * snapshot and the weights applied. Negative if it cannot be computed. */
static double gnss_pt_mvdr_pout(const gnss_pt_mvdr_state_t *s)
{
    double w1r, w1i, w2r, w2i, p;
    if (GNSS_PT_ST_ACTIVE(s) && s->have_aolc) {
        /* STAP: there is no 2x2 weight set; the output power is measured by
         * output_scaler (OUT_LVL, after the gain): remove the gain */
        p = pow(2.0, (double)s->out_lvl_q4 / 16.0)
          / pow(2.0, 2.0 * (double)s->out_gain_q8 / 256.0);
        return (p < 1e-3) ? 1e-3 : p;
    }
    if (!s->cov_coherent || !s->have_r12 || s->cov_r11 <= 0 || s->cov_r22 <= 0) {
        return -1.0;
    }
    w1r = (double)s->w1_re / 32768.0;  w1i = (double)s->w1_im / 32768.0;
    w2r = (double)s->w2_re / 32768.0;  w2i = (double)s->w2_im / 32768.0;
    p = ((w1r * w1r + w1i * w1i) * (double)s->cov_r11
       + (w2r * w2r + w2i * w2i) * (double)s->cov_r22
       + 2.0 * ((w1r * w2r + w1i * w2i) * (double)s->cov_r12_re
              - (w1r * w2i - w1i * w2r) * (double)s->cov_r12_im))
      / ((double)s->block_n * (double)s->cov_scale);
    return (p < 1e-3) ? 1e-3 : p;
}

/* ===========================================================================
 *  MOD-24: JAMMER SUPPRESSION REPORT  (gnss_supp?, gnss_mvdr?, STATUS line)
 * ===========================================================================
 *
 *  READ THIS FIRST -- what the numbers are, and why none of them says 72 dB
 *  -----------------------------------------------------------------------
 *  The "72 dB" is the textbook dynamic range of a 12-bit converter:
 *      6.02 dB x 12 bits = 72.2 dB   (+1.76 dB for a full-scale sine = 74 dB)
 *  It is the ratio between the LARGEST signal the ADC can hold (full scale)
 *  and its QUANTISATION noise (about -72..-74 dBFS over the whole band).
 *  It is printed below only as the SCALE (0 .. 72 dB) of the bars.
 *
 *  1) "print suppression"  -- THIS IS ONLY A PRINT (a display figure).
 *         print = (jammer + noise going in) / (jammer residual + noise going out)
 *     It is what an engineer measures with a spectrum analyser on RX1 and on
 *     TX1. The NOISE is never cancelled (it is independent on the two
 *     antennas -- no beamformer can remove it, and it must not: the
 *     satellites are under it). So even a PERFECT null leaves the noise, and
 *     this figure can never exceed
 *  2) "JNR at the ADC" = jammer power / noise power at the converter.
 *     This is the CEILING of the print. When print == JNR the jammer is
 *     already under the noise floor: there is nothing more to take away.
 *  3) "ACTUAL cancellation" = jammer in / jammer residual out.
 *     The real depth of the null. Computed from the noise estimate (NF_EST)
 *     and the noise gain of the weights:
 *        noise_out   = noise x (|w1|^2 + |w2|^2)      single-weight MVDR/PI
 *        noise_out   = noise x (1 + sum |w_aux|^2)      STAP (WNORM register)
 *        residual    = P_out - noise_out
 *        actual      = jammer_in x (reference-path gain) / residual
 *     Once the residual is far under the noise its exact value is lost in
 *     the measurement error, so the report then prints a LOWER BOUND
 *     ("> X dB": residual < 1/10 of the output noise). Field logs (MOD-22):
 *     the actual cancellation was ~46..62 dB, the residual 20..30 dB UNDER
 *     the noise -- more than enough; deeper would not add one satellite.
 *
 *  WHY THE PRINT IS ~30..40 dB AND NOT 72 dB (this board, AD9361)
 *  -------------------------------------------------------------
 *    a) The noise floor at the ADC is NOT the quantisation floor. The
 *       antenna/LNA thermal noise plus the AD9361's own noise sit at about
 *       -47 dBFS on this board (measured, MOD-15c/20), ~25 dB above the
 *       -72 dBFS quantisation floor. Those 25 dB of the 72 are gone before
 *       any processing: they are occupied by real noise, not by the ADC.
 *    b) The jammer must stay UNDER full scale. Its peaks may not clip (a
 *       clipped jammer is no longer a copy on both antennas and cannot be
 *       nulled), so the RX gain is lowered until the peaks are 3..6 dB below
 *       0 dBFS. A noise-like/barrage jammer has peaks ~10..12 dB above its
 *       rms, a CW ~3 dB: its average power ends up at about -6..-15 dBFS.
 *    c) => JNR at the ADC = jammer dBFS - noise dBFS ~ -10 - (-47) ~ 35..40 dB
 *       which is exactly the "~40 dB maximum suppression" seen on the board.
 *       The report prints this budget:
 *          72 - (noise floor above -72 dBFS) - (jammer below 0 dBFS) = JNR
 *    d) A stronger jammer does NOT raise it: the gain goes down by the same
 *       amount (the AD9361 floor is ~constant in dBFS), so the JNR at the
 *       ADC stays ~35..40 dB however strong the jammer is in the air.
 *    e) The satellites are ~20..30 dB UNDER the noise (spread spectrum). What
 *       decides whether they track is the residual jammer vs the noise
 *       (a residual 10 dB under the noise costs < 0.5 dB of C/N0) and how
 *       much the noise floor itself rose because the jammer forced the RX
 *       gain down (MOD-20) -- not the 72 dB.
 *    f) Single weight on a WIDEBAND jammer: the null is also limited by the
 *       RX1/RX2 coherence across the band (1-coh limit, MOD-23). STAP
 *       (gnss_null=2) removes that limit; it does not change a)..e).
 * ======================================================================== */
#define GNSS_PT_SUPP_SCALE_DB  72.0      /* 12 bits x 6.02 dB: bar scale only */

typedef struct {
    int    ok;            /* figures valid                                    */
    int    stap;          /* the STAP core is driving the output              */
    int    have_jam;      /* a jammer is above 1/20 of the noise              */
    double pin;           /* RX1 power in, raw LSB^2 per sample               */
    double nf;            /* noise per element, raw LSB^2 (NF_EST)            */
    double jin;           /* jammer power at RX1                              */
    double pout;          /* output power before the output gain              */
    double nout;          /* noise at the output                              */
    double ref_gain;      /* power gain of the reference path (|w1|^2 or 1)   */
    double res;           /* jammer residual at the output                    */
    double print_db;      /* print suppression                                */
    double jnr_db;        /* JNR at the ADC                                   */
    double actual_db;     /* actual cancellation (or its lower bound)         */
    int    actual_lo;     /* 1 = actual_db is a lower bound ("> X")           */
    double res_db;        /* residual vs output noise                         */
    double noise_dbfs;    /* noise floor at the ADC                           */
    double jam_dbfs;      /* jammer average power at the ADC                  */
    double coh_lim_db;    /* single-weight 1-coh limit                        */
    double nf_cov;        /* covariance floor (NF_EST), raw LSB^2             */
    int    nf_src;        /* nf from: 0 covariance, 1 SMI lambda_min, 2 learned */
    double jfloor_dbc;    /* jammer-derived floor (nf_cov - nf) vs jammer, dBc */
} gnss_pt_supp_t;

/* Thermal floor. NF_EST = det(R)/trace(R) is the noise eigenvalue with a
 * jammer but HALF the per-element noise without one (mvdr_regs.v), so the
 * no-jammer learning uses 2 x NF_EST (the caller passes NF_EST, +3.01 dB
 * here). SMI's lambda_min is not the noise floor either: the noise is
 * band-limited (40 MHz in 45 MSPS), so across the time taps lambda_min
 * picks the quietest edge of the band (field: 4..6 dB low). Its offset to
 * the real floor is learned with no jammer and applied under a jammer. */
static double  th_ref_dbfs = 0.0;          /* per-element floor learned w/o jammer */
static int32_t th_ref_gain = 0, th_gain_now = 0;
static int     th_have = 0;
static double  th_smi_off_db = 0.0;        /* floor - SMI lambda_min, learned      */
static int     th_smi_have = 0;
void gnss_pt_thermal_learn(double nf_est_dbfs, int32_t gain_db)
{
    /* no jammer: NF_EST is half the noise */
    gnss_pt_thermal_learn_floor(nf_est_dbfs + 3.0103, gain_db);
}
/* MOD-26b: the floor itself (per element, dBFS), e.g. the SMI RX1 power
 * with no jammer -- does not depend on the 2 x 2 covariance */
void gnss_pt_thermal_learn_floor(double nf, int32_t gain_db)
{
    gnss_smi_state_t ss;
    if (th_have && gain_db == th_ref_gain) { th_ref_dbfs = 0.8 * th_ref_dbfs + 0.2 * nf; }
    else                                   { th_ref_dbfs = nf; th_ref_gain = gain_db; }
    th_have = 1;
    th_gain_now = gain_db;
    if (gnss_smi_available()) {
        gnss_smi_get_state(&ss);
        if (ss.hw_active && ss.updates > 0U) {
            double off = th_ref_dbfs - ss.lam_dbfs;
            th_smi_off_db = th_smi_have ? 0.8 * th_smi_off_db + 0.2 * off : off;
            th_smi_have = 1;
        }
    }
}
void gnss_pt_thermal_gain(int32_t gain_db) { th_gain_now = gain_db; }

/* MOD-26b: plausibility of the single-weight 2 x 2 covariance (R11/R22),
 * per element in dBFS. A field build (4 Nov) read R22 = 0 or +11 dBFS
 * (impossible: full scale is +3 dBFS) while RX1, R12 and the ADC peaks were
 * normal: everything downstream of it -- the jammer estimator, the per-
 * channel noise, the RX auto-level target, NF_EST -- was then wrong. Returns
 * 1 sane, 0 corrupt, -1 no window yet. */
static uint32_t cov_bad_n = 0, cov_ok_n = 0;
static double   cov_bad_p1 = 0.0, cov_bad_p2 = 0.0;
int gnss_pt_cov_check(const gnss_pt_mvdr_state_t *s, double *p1_dbfs, double *p2_dbfs)
{
    double N, p1, p2;
    int ok;
    if (s->id != GNSS_PT_MVDR_EXPECTED_ID || s->block_n == 0U || s->cov_scale == 0U) { return -1; }
    N  = (double)s->block_n * (double)s->cov_scale;
    p1 = (s->cov_r11 > 0) ? gnss_pt_db10((double)s->cov_r11 / N / GNSS_PT_FS2) : -200.0;
    p2 = (s->cov_r22 > 0) ? gnss_pt_db10((double)s->cov_r22 / N / GNSS_PT_FS2) : -200.0;
    if (p1_dbfs) { *p1_dbfs = p1; }
    if (p2_dbfs) { *p2_dbfs = p2; }
    ok = (p1 > -150.0 && p2 > -150.0 && p1 <= 3.5 && p2 <= 3.5 &&
          p1 - p2 < 20.0 && p2 - p1 < 20.0) ? 1 : 0;
    if (ok) { cov_ok_n++; } else { cov_bad_n++; cov_bad_p1 = p1; cov_bad_p2 = p2; }
    return ok;
}
int gnss_pt_cov_bad_seen(double *p1_dbfs, double *p2_dbfs)
{
    if (p1_dbfs) { *p1_dbfs = cov_bad_p1; }
    if (p2_dbfs) { *p2_dbfs = cov_bad_p2; }
    return (int)((cov_bad_n > 0U) ? 1 : 0);
}

/* MOD-26: the floor for the SMI-based report (gnss_report.c), per element in
 * dBFS at the present gain. src: 1 SMI lambda_min + learned offset,
 * 2 learned without jammer and scaled with the RX gain, 3 lambda_min + the
 * typical offset (nothing learned yet). */
int gnss_pt_thermal_now(double smi_lam_dbfs, int smi_ok, int32_t gain_now, double *nf_dbfs, int *src)
{
    if (smi_ok && th_smi_have) {
        *nf_dbfs = smi_lam_dbfs + th_smi_off_db;  *src = 1;
    } else if (th_have) {
        *nf_dbfs = th_ref_dbfs + (double)(gain_now - th_ref_gain);  *src = 2;
    } else if (smi_ok) {
        *nf_dbfs = smi_lam_dbfs + 5.0;  *src = 3;
    } else {
        return -1;
    }
    return 0;
}
/* the floor learned without a jammer and the gain it was learned at */
int gnss_pt_thermal_clean(double *nf_dbfs, int32_t *gain_db)
{
    if (!th_have) { return -1; }
    *nf_dbfs = th_ref_dbfs;
    *gain_db = th_ref_gain;
    return 0;
}

/* thermal floor per element (raw LSB^2), never above the RX1 input power */
static double gnss_pt_thermal(const gnss_pt_mvdr_state_t *s, double nf_cov, double pin, int *src)
{
    double t;
    *src = 0;
    if (s->have_smi && s->smi_active && s->smi_count >= 64U && th_smi_have) {
        t = pow(10.0, (s->smi_lam_dbfs + th_smi_off_db) / 10.0) * GNSS_PT_FS2;   /* measured */
        *src = 1;
    } else if (th_have) {
        t = pow(10.0, (th_ref_dbfs + (double)(th_gain_now - th_ref_gain)) / 10.0) * GNSS_PT_FS2;
        *src = 2;
    } else {
        t = nf_cov;                 /* nothing learned yet: the covariance floor */
    }
    return (t < pin) ? t : pin;
}

static int gnss_pt_supp_compute(const gnss_pt_mvdr_state_t *s, gnss_pt_supp_t *o)
{
    double N, w1m2 = 1.0, w2m2 = 0.0, coh = 0.0;

    memset(o, 0, sizeof(*o));
    if (s->id != GNSS_PT_MVDR_EXPECTED_ID || !s->cov_coherent || !s->have_nf ||
        s->nf_est == 0U || s->cov_r11 <= 0 || s->cov_r22 <= 0) {
        return -1;
    }
    N      = (double)s->block_n * (double)s->cov_scale;
    o->stap = GNSS_PT_ST_ACTIVE(s) ? 1 : 0;
    o->pin  = (double)s->cov_r11 / N;
    o->nf_cov = (double)s->nf_est / N;
    o->nf   = gnss_pt_thermal(s, o->nf_cov, o->pin, &o->nf_src);   /* MOD-25 */
    o->jin  = o->pin - o->nf;
    o->jfloor_dbc = (o->nf_cov > 1.05 * o->nf && o->jin > 0.0)
                  ? gnss_pt_db10((o->nf_cov - o->nf) / o->jin) : -99.0;
    o->pout = gnss_pt_mvdr_pout(s);
    if (o->pout <= 0.0) { return -1; }
    if (s->have_r12) {
        coh = ((double)s->cov_r12_re * (double)s->cov_r12_re +
               (double)s->cov_r12_im * (double)s->cov_r12_im) /
              ((double)s->cov_r11 * (double)s->cov_r22);
    }
    o->coh_lim_db = (coh < 0.999999) ? -10.0 * log10(1.0 - coh) : 60.0;
    if (o->stap && s->have_smi && s->smi_active) {
        /* MOD-25 SMI: the constraint a^H w = 1 holds the reference tap at
         * unity (power inversion), the noise passes with sum |w|^2 */
        o->ref_gain = s->smi_ref_gain;
        o->nout     = o->nf * s->smi_wnorm;
    } else if (o->stap) {
        o->ref_gain = 1.0;                                       /* r = RX1 centre tap */
        o->nout     = o->nf * (1.0 + (double)s->stap_wnorm / 16777216.0);
    } else {
        double w1r = (double)s->w1_re / 32768.0, w1i = (double)s->w1_im / 32768.0;
        double w2r = (double)s->w2_re / 32768.0, w2i = (double)s->w2_im / 32768.0;
        w1m2 = w1r * w1r + w1i * w1i;
        w2m2 = w2r * w2r + w2i * w2i;
        if (w1m2 < 1e-12) { return -1; }
        o->ref_gain = w1m2;
        o->nout     = o->nf * (w1m2 + w2m2);
    }
    o->noise_dbfs = gnss_pt_db10(o->nf / GNSS_PT_FS2);
    o->have_jam   = (o->jin > (o->nf_src ? 0.5 : 0.05) * o->nf) ? 1 : 0;   /* JNR > -3 dB */
    o->jam_dbfs   = o->have_jam ? gnss_pt_db10(o->jin / GNSS_PT_FS2) : -200.0;
    /* the print figure, referred to the RX1 path (output gain removed) */
    o->print_db   = gnss_pt_db10(o->pin * o->ref_gain / o->pout);
    if (o->print_db < 0.0) { o->print_db = 0.0; }
    o->jnr_db     = o->have_jam ? gnss_pt_db10(o->jin / o->nf) : 0.0;
    o->res        = o->pout - o->nout;
    if (!o->have_jam) {
        o->actual_db = 0.0;  o->actual_lo = 0;  o->res_db = -99.0;
    } else if (o->res > 0.1 * o->nout) {
        o->actual_db = gnss_pt_db10(o->jin * o->ref_gain / o->res);
        o->actual_lo = 0;
        o->res_db    = gnss_pt_db10(o->res / o->nout);
    } else {
        o->actual_db = gnss_pt_db10(o->jin * o->ref_gain / (0.1 * o->nout));
        o->actual_lo = 1;
        o->res_db    = -10.0;
    }
    o->ok = 1;
    return 0;
}

/* "12.3" / "+12.3" / ">12.3" into buf (fixed-point, no float printf needed) */
static const char *gnss_pt_f1(char *buf, double v, char prefix)
{
    long m = (long)(v * 10.0 + ((v >= 0.0) ? 0.5 : -0.5));
    char *p = buf;
    if (prefix && !(prefix == '+' && m < 0)) { *p++ = prefix; }
    if (m < 0) { *p++ = '-'; m = -m; }
    sprintf(p, "%ld.%ld", m / 10L, m % 10L);
    return buf;
}

/* |#####.......| 0..72 dB, 2 dB per character; '>' marks a lower bound */
static void gnss_pt_supp_bar(double db, int lower_bound)
{
    int i, n = (int)(db / 2.0 + 0.5);
    if (n < 0) { n = 0; }
    if (n > 36) { n = 36; }
    printf("|");
    for (i = 0; i < 36; i++) {
        printf("%c", (i < n) ? ((lower_bound && i == n - 1) ? '>' : '#') : '.');
    }
    printf("|");
}

static void gnss_pt_supp_report(const gnss_pt_mvdr_state_t *s)
{
    gnss_pt_supp_t o;
    char b1[24], b2[24], b3[24];

    printf("  --- jammer suppression (bar scale 0..72 dB = 12 bits x 6.02 dB) ---\n");
    if (gnss_pt_supp_compute(s, &o) != 0) {
        printf("  (not available: needs a coherent window, NF_EST and weights)\n");
        return;
    }
    printf("  output from        : %s\n", !o.stap
           ? "single complex weight per antenna (MVDR / power inversion)"
           : (s->have_smi && s->smi_active)
           ? "SMI-MVDR STAP, sample-matrix inversion (2 antennas x 8 taps)"
           : "STAP, space-time power inversion (2 antennas x 5 taps)");
    if (!o.have_jam) {
        printf("  no jammer above the noise (RX1 noise %s dBFS): nothing to suppress.\n",
               gnss_pt_f1(b1, o.noise_dbfs, 0));
        return;
    }
    printf("  print suppression  : %6s dB ", gnss_pt_f1(b1, o.print_db, 0));
    gnss_pt_supp_bar(o.print_db, 0);
    printf(" PRINT only: (J+N in)/(res+N out)\n");
    printf("  JNR at the ADC     : %6s dB ", gnss_pt_f1(b1, o.jnr_db, 0));
    gnss_pt_supp_bar(o.jnr_db, 0);
    printf(" = ceiling of the print\n");
    /* MOD-25: which floor the figures use, and the jammer energy that the
     * covariance floor was hiding (IQ image / RX1-RX2 decorrelation) */
    printf("  noise floor        : %s dBFS (%s)\n", gnss_pt_f1(b1, o.noise_dbfs, 0),
           (o.nf_src == 1) ? "thermal, SMI lambda_min + learned offset" :
           (o.nf_src == 2) ? "thermal, learned without jammer, scaled with RX gain" :
                             "covariance floor: may include jammer energy");
    if (o.nf_src != 0) {
        printf("  covariance floor   : %s dBFS", gnss_pt_f1(b1, gnss_pt_db10(o.nf_cov / GNSS_PT_FS2), 0));
        if (o.jfloor_dbc > -90.0) {
            printf(" -> jammer-derived floor %s dBc (IQ image / decorrelation)\n",
                   gnss_pt_f1(b2, o.jfloor_dbc, 0));
        } else {
            printf(" (= thermal: no jammer-derived floor)\n");
        }
    }
    printf("  ACTUAL cancellation: %6s dB ", gnss_pt_f1(b1, o.actual_db, o.actual_lo ? '>' : 0));
    gnss_pt_supp_bar(o.actual_db, o.actual_lo);
    printf(" jammer in / residual\n");
    printf("  12-bit ideal       : %6s dB ", "72.2");
    gnss_pt_supp_bar(GNSS_PT_SUPP_SCALE_DB, 0);
    printf(" 6.02 x 12 (+1.76 = 74 sine)\n");
    if (o.actual_lo) {
        printf("  jammer residual    : more than 10 dB UNDER the output noise. The null is\n"
               "                       complete; only the noise floor limits the satellites.\n");
    } else {
        printf("  jammer residual    : %s dB vs the output noise%s\n", gnss_pt_f1(b1, o.res_db, '+'),
               (o.res_db > 0.0) ? "  <-- ABOVE the noise: satellites suffer" :
               (o.res_db > -6.0) ? "  (close to the noise: some C/N0 lost)" : "");
    }
    printf("  why not 72 dB      : 72.2 - %s (noise floor %s dBFS, not -72)\n"
           "                            - %s (jammer %s dBFS, kept under full scale)\n"
           "                            = %s dB = JNR at the ADC = the most the PRINT can show:\n"
           "                       the noise is never cancelled. The jammer itself is cancelled\n"
           "                       by the ACTUAL figure. A stronger jammer only lowers the RX gain.\n",
           gnss_pt_f1(b1, o.noise_dbfs + 72.2, 0), gnss_pt_f1(b2, o.noise_dbfs, 0),
           gnss_pt_f1(b3, -o.jam_dbfs, 0), gnss_pt_f1(b3 + 12, o.jam_dbfs, 0),
           gnss_pt_f1(b2 + 12, o.jnr_db, 0));
    if (!o.stap && o.jnr_db > o.coh_lim_db + 6.0) {
        printf("  WIDEBAND LIMIT     : one weight per antenna stops at %s dB (1-coh): RX1/RX2\n"
               "                       differ across the band. gnss_null=3 (SMI) / 2 (STAP) equalise that.\n",
               gnss_pt_f1(b1, o.coh_lim_db, 0));
    } else if (o.stap && o.jnr_db > o.coh_lim_db + 6.0) {
        printf("  wideband jammer    : one weight would stop at %s dB (1-coh); %s is not\n"
               "                       limited by it.\n", gnss_pt_f1(b1, o.coh_lim_db, 0), GNSS_PT_ST_NAME(s));
    }
    if (o.stap && s->have_smi && s->smi_active) {
        printf("  SMI                : sum |w|^2 %s (noise cost %s dB), |w_ref|^2 %s; gnss_smi?\n",
               gnss_pt_f1(b1, s->smi_wnorm, 0),
               gnss_pt_f1(b2, gnss_pt_db10(s->smi_wnorm), '+'),
               gnss_pt_f1(b3, s->smi_ref_gain, 0));
    } else if (o.stap) {
        printf("  STAP               : sum |w|^2 %s (noise cost %s dB), PL %lu, mu 2^-%lu,\n"
               "                       anti-windup %lu\n",
               gnss_pt_f1(b1, (double)s->stap_wnorm / 16777216.0, 0),
               gnss_pt_f1(b2, gnss_pt_db10(1.0 + (double)s->stap_wnorm / 16777216.0), '+'),
               (unsigned long)s->stap_pl, (unsigned long)((s->stap_ctrl >> 4) & 0xFU),
               (unsigned long)s->stap_nwsat);
    }
}

int gnss_pt_print_suppression(void)
{
    gnss_pt_mvdr_state_t s;
    gnss_pt_mvdr_get_state(&s);
    if (s.id != GNSS_PT_MVDR_EXPECTED_ID) {
        printf("MVDR core not in this bitstream.\n");
        return -1;
    }
    printf("----- JAMMER SUPPRESSION -----\n");
    if (s.have_smi && s.smi_active) {
        /* MOD-26: the SMI figures (from the window the weights are solved from) */
        gnss_rep_print_supp();
        printf("  (single-weight view below, for comparison; gnss_report? = everything)\n");
    }
    gnss_pt_supp_report(&s);
    printf("------------------------------\n");
    return 0;
}

int gnss_pt_supp_info(gnss_pt_supp_info_t *i)
{
    gnss_pt_mvdr_state_t s;
    gnss_pt_supp_t o;
    memset(i, 0, sizeof(*i));
    gnss_pt_mvdr_get_state(&s);
    if (gnss_pt_supp_compute(&s, &o) != 0) { return -1; }
    i->ok = 1;               i->have_jam = o.have_jam;   i->space_time = o.stap;
    i->print_db = o.print_db; i->jnr_db = o.jnr_db;     i->actual_db = o.actual_db;
    i->actual_lo = o.actual_lo; i->res_db = o.res_db;   i->noise_dbfs = o.noise_dbfs;
    i->nf_src = o.nf_src;
    i->cov_floor_dbfs = gnss_pt_db10(o.nf_cov / GNSS_PT_FS2);
    i->jfloor_dbc = o.jfloor_dbc;  i->jam_dbfs = o.jam_dbfs;  i->coh_lim_db = o.coh_lim_db;
    return 0;
}

int gnss_pt_supp_summary(double *print_db, double *jnr_db, double *actual_db, int *actual_lo)
{
    gnss_pt_mvdr_state_t s;
    gnss_pt_supp_t o;
    gnss_pt_mvdr_get_state(&s);
    if (gnss_pt_supp_compute(&s, &o) != 0 || !o.have_jam) { return -1; }
    if (print_db)  { *print_db  = o.print_db; }
    if (jnr_db)    { *jnr_db    = o.jnr_db; }
    if (actual_db) { *actual_db = o.actual_db; }
    if (actual_lo) { *actual_lo = o.actual_lo; }
    return 0;
}

/* RX gain target, jammer OFF (block v1.12, FIRMWARE_GUIDE section 3):
 * noise floor per element about -60 dBFS (-63..-57). Each 6 dB lower buys
 * 6 dB more jammer headroom (12-bit ADC: a CW jammer up to ~57 dB JNR before
 * clipping at -60) for a satellite loss of 0.1..0.35 dB. The automatic output
 * level (gnss_aolc=) restores the TX1 level, so the RX gain no longer has to
 * be high for the output's sake. */
/* MOD-15c: the target in USE. It starts at gnss_config.h's value and the
 * RX auto-level (gnss_autostart.c) raises it when the AD9361's own floor is
 * too high for it (antenna noise must stay >= 10 dB above that floor). */
static double gnss_pt_rx_target = (double)GNSS_CFG_RX_NF_TARGET_DBFS;
void   gnss_pt_set_rx_target_dbfs(double t) { gnss_pt_rx_target = t; }
double gnss_pt_get_rx_target_dbfs(void)     { return gnss_pt_rx_target; }
#define GNSS_PT_RX_TARGET_DBFS   (gnss_pt_rx_target)

void gnss_pt_mvdr_print_levels(void)
{
    gnss_pt_mvdr_state_t s;
    gnss_pt_jam_state_t  j;
    double pin = 0.0, prx2 = 0.0, d1, d2, n1 = 0.0, n2 = 0.0;
    int    have_noise = 0, jam_on;
    long   step;

    printf("\n--- RX input level (from the MVDR covariance) ---\n");
    if (!gnss_pt_mvdr_levels(&s, &pin, &prx2)) {
        if (s.id != GNSS_PT_MVDR_EXPECTED_ID) {
            printf("  MVDR core not in this bitstream: no level measurement.\n");
        } else {
            printf("  No usable covariance window (windows=%lu; R11/R22 <= 0 or\n"
                   "  not coherent). Is RX running, is RX2 enabled at the AD9361\n"
                   "  2R2T level, and is l_clk within the timing constraint?\n",
                   (unsigned long)s.n_blocks);
        }
        printf("-----------------------------------\n");
        return;
    }
    d1 = gnss_pt_db10(pin  / GNSS_PT_FS2);
    d2 = gnss_pt_db10(prx2 / GNSS_PT_FS2);

    printf("  P_in  (RX1)    : "); gnss_pt_print_f3(d1); printf(" dBFS   total input\n");
    printf("  P_rx2 (RX2)    : "); gnss_pt_print_f3(d2); printf(" dBFS\n");
    printf("  RX1 - RX2      : "); gnss_pt_print_f3(d1 - d2); printf(" dB\n");

    gnss_pt_jam_get_state(&j);
    jam_on = (j.available && j.valid && j.present) ? 1 : 0;
    /* Noise floor per channel (MVDR >= 1.11) is valid with the jammer ON
     * as well: the jammer is separated out by its RX1/RX2 coherence. */
    if (j.have_ch && j.n1_q8 != (int16_t)GNSS_PT_JAM_Q8_NONE &&
        j.n2_q8 != (int16_t)GNSS_PT_JAM_Q8_NONE) {
        n1 = (double)j.n1_q8 / 256.0;
        n2 = (double)j.n2_q8 / 256.0;
        have_noise = 1;
        printf("  noise RX1/RX2  : "); gnss_pt_print_f3(n1);
        printf(" / "); gnss_pt_print_f3(n2); printf(" dBFS   (jammer removed)\n");
    }
    (void)gnss_pt_print_clip(&s);
    gnss_pt_print_headroom(&s);
    printf("  jammer         : %s\n",
           !j.available ? "estimator not in this bitstream"
           : jam_on ? "PRESENT" : "not detected");

    printf("  TARGET         : noise floor ");
    gnss_pt_print_f3(GNSS_PT_RX_TARGET_DBFS);
    printf(" dBFS (+/-3)%s.\n"
           "                   gnss_rxauto=1 sets it (jammer ON or OFF%s). The\n"
           "                   ADC clips about %ld dB above it; ADC clipping must\n"
           "                   stay 0 -- the clip guard lowers the RX gain by\n"
           "                   itself when a jammer clips. TX1 level: gnss_aolc=.\n",
           (GNSS_PT_RX_TARGET_DBFS > (double)GNSS_CFG_RX_NF_TARGET_DBFS + 0.5)
               ? " (raised: AD9361 floor high)" : "",
           have_noise ? "" : ": set it with the jammer OFF on this build",
           (long)(-GNSS_PT_RX_TARGET_DBFS));
    if (s.clip1 || s.clip2) {
        printf("  ACTION         : ADC IS CLIPPING. The clip guard (gnss_guard?)\n"
               "                   steps the RX gain down by itself; by hand:\n"
               "                   gnss_rxauto=1 or gnss_rxgain= 6 dB lower.\n");
    } else if (have_noise || !jam_on) {
        double ref = have_noise ? 0.5 * (n1 + n2) : d1;
        step = (long)floor(GNSS_PT_RX_TARGET_DBFS - ref + 0.5);
        if (step >= -3 && step <= 3) {
            printf("  ACTION         : none, the level is on target.\n");
        } else {
            printf("  ACTION         : change the RX gain by %c%ld dB: gnss_rxauto=1\n",
                   (step < 0) ? '-' : '+', labs(step));
        }
    } else {
        printf("  ACTION         : jammer present and no per-channel noise readback\n"
               "                   (MVDR < 1.11): set the gain with the jammer OFF.\n");
    }
    if ((d1 - d2) > 3.0 || (d1 - d2) < -3.0) {
        printf("  NOTE           : RX1 and RX2 differ by more than 3 dB. With equal\n"
               "                   gains that is an antenna/cable difference; MVDR\n"
               "                   still works, but |w2/w1| grows with it.\n");
    }
    printf("-----------------------------------\n");
}

int32_t gnss_pt_mvdr_set_out_gain(uint32_t n)
{
    if (gnss_pt_read(GNSS_PT_REG_MVDR_ID) != GNSS_PT_MVDR_EXPECTED_ID) {
        return GNSS_PT_ERR_MVDR_ID;
    }
    if (gnss_pt_read(GNSS_PT_REG_MVDR_VERSION) < GNSS_PT_MVDR_VERSION_GAIN) {
        return GNSS_PT_ERR_MVDR_OLD;
    }
    if (n > GNSS_PT_MVDR_OUT_GAIN_MAX) { n = GNSS_PT_MVDR_OUT_GAIN_MAX; }
    /* Manual gain: AOLC off, its speed/target fields kept for gnss_aolc=. */
    gnss_pt_write(GNSS_PT_REG_MVDR_OUT_GAIN,
                  (gnss_pt_read(GNSS_PT_REG_MVDR_OUT_GAIN) & 0x00FFF000U) | n);
    return gnss_pt_mvdr_commit();
}

static uint32_t aolc_base_db = 0;          /* operator / boot target, 0 = off */
static uint32_t aolc_back_db = 0;          /* MOD-25 clip back-off on top     */

static int32_t aolc_apply(uint32_t target_db);

int32_t gnss_pt_mvdr_set_aolc(uint32_t target_db)
{
    aolc_base_db = target_db;
    aolc_back_db = 0;
    return aolc_apply(target_db);
}

int gnss_pt_aolc_backoff(int step_db)
{
    long b;
    if (aolc_base_db == 0U) { return -1; }
    b = (long)aolc_back_db + step_db;
    if (b < 0) { b = 0; }
    if (b > (long)GNSS_PT_AOLC_BACKOFF_MAX) { b = GNSS_PT_AOLC_BACKOFF_MAX; }
    if (aolc_base_db + (uint32_t)b > 40U) { b = (aolc_base_db < 40U) ? (long)(40U - aolc_base_db) : 0; }
    if ((uint32_t)b != aolc_back_db) {
        aolc_back_db = (uint32_t)b;
        (void)aolc_apply(aolc_base_db + aolc_back_db);
    }
    return (int)aolc_back_db;
}
uint32_t gnss_pt_aolc_backoff_db(void) { return aolc_back_db; }

static int32_t aolc_apply(uint32_t target_db)
{
    uint32_t og;

    if (gnss_pt_read(GNSS_PT_REG_MVDR_ID) != GNSS_PT_MVDR_EXPECTED_ID) {
        return GNSS_PT_ERR_MVDR_ID;
    }
    if (gnss_pt_read(GNSS_PT_REG_MVDR_VERSION) < GNSS_PT_MVDR_VERSION_AOLC) {
        return GNSS_PT_ERR_MVDR_OLD;
    }
    og = gnss_pt_read(GNSS_PT_REG_MVDR_OUT_GAIN) & 0x7U;      /* keep the manual n */
    if (target_db != 0U) {
        og |= GNSS_PT_OG_AOLC_EN | GNSS_PT_OG_AGC_SH(gnss_pt_aolc_sh()) |
              GNSS_PT_OG_TARGET_DB(target_db);
    } else {
        og |= GNSS_PT_OG_AGC_SH(gnss_pt_aolc_sh()) |
              GNSS_PT_OG_TARGET_DB(GNSS_PT_AOLC_TARGET_DEFAULT);
    }
    gnss_pt_write(GNSS_PT_REG_MVDR_OUT_GAIN, og);
    return gnss_pt_mvdr_commit();
}

/* MOD-18: AOLC loop speed-up for ~0.15 s after an RX gain step, so TX1 (and
 * every satellite in it) is back at its level in ~3 ms instead of ~40 ms.
 * Per AOLC block the loop moves 2^-(SH+1) of the error; SH >= 2 keeps it
 * well damped with the 4-block minimum filter. L (the gain) is not reset by
 * a commit, only by entering AOLC, so switching speed is seamless. */
int32_t gnss_pt_mvdr_aolc_fast(int fast)
{
    uint32_t og, sh;

    if (gnss_pt_read(GNSS_PT_REG_MVDR_ID) != GNSS_PT_MVDR_EXPECTED_ID ||
        gnss_pt_read(GNSS_PT_REG_MVDR_VERSION) < GNSS_PT_MVDR_VERSION_AOLC) {
        return GNSS_PT_ERR_MVDR_OLD;
    }
    og = gnss_pt_read(GNSS_PT_REG_MVDR_OUT_GAIN);
    if (!(og & GNSS_PT_OG_AOLC_EN)) { return GNSS_PT_OK; }
    sh = gnss_pt_aolc_sh();
    if (fast) { sh = (sh >= 6U) ? sh - 4U : 2U; }
    og = (og & ~(0xFU << 12)) | GNSS_PT_OG_AGC_SH(sh);
    gnss_pt_write(GNSS_PT_REG_MVDR_OUT_GAIN, og);
    return gnss_pt_mvdr_commit();
}

int32_t gnss_pt_mvdr_set_iq(int mode)
{
    uint32_t c;

    if (gnss_pt_read(GNSS_PT_REG_MVDR_ID) != GNSS_PT_MVDR_EXPECTED_ID) {
        return GNSS_PT_ERR_MVDR_ID;
    }
    if (gnss_pt_read(GNSS_PT_REG_MVDR_VERSION) < GNSS_PT_MVDR_VERSION_IQ) {
        return GNSS_PT_ERR_MVDR_OLD;
    }
    c = GNSS_PT_IQ_MU_SH(GNSS_PT_IQ_MU_DEFAULT);
    gnss_pt_iq_user = (mode == 0) ? 0 : 1;     /* MOD-21: kept across restarts */
    if (mode == 0) { c |= GNSS_PT_IQ_OFF; }
    if (mode == 2) {
        /* clear: hold b = 0 for one commit, then release (next estimate is a
         * full step) */
        gnss_pt_write(GNSS_PT_REG_MVDR_IQ_CTRL, c | GNSS_PT_IQ_CLEAR);
        if (gnss_pt_mvdr_commit() != GNSS_PT_OK) { return GNSS_PT_ERR_MVDR_COMMIT; }
        no_os_mdelay(1);
    }
    gnss_pt_write(GNSS_PT_REG_MVDR_IQ_CTRL, c);
    return gnss_pt_mvdr_commit();
}

void gnss_pt_mvdr_print_iq(void)
{
    gnss_pt_mvdr_state_t s;

    gnss_pt_mvdr_get_state(&s);
    printf("\n--- Receiver IQ-image correction ---\n");
    if (s.id != GNSS_PT_MVDR_EXPECTED_ID || !s.have_iq) {
        printf("  Not in this bitstream (needs MVDR 1.9).\n");
        printf("-----------------------------------\n");
        return;
    }
    printf("  state          : %s%s\n",
           (s.iq_ctrl & GNSS_PT_IQ_OFF) ? "OFF" : "ON",
           (s.iq_ctrl & GNSS_PT_IQ_CLEAR) ? " (held at 0: clear)" : "");
    printf("  RX1 image b1   : %ld %ldj (Q20)  = ", (long)s.iq_b1_re, (long)s.iq_b1_im);
    gnss_pt_print_f3(gnss_pt_db10(((double)s.iq_b1_re * s.iq_b1_re +
                                   (double)s.iq_b1_im * s.iq_b1_im) / 1099511627776.0));
    printf(" dBc\n");
    printf("  RX2 image b2   : %ld %ldj (Q20)  = ", (long)s.iq_b2_re, (long)s.iq_b2_im);
    gnss_pt_print_f3(gnss_pt_db10(((double)s.iq_b2_re * s.iq_b2_re +
                                   (double)s.iq_b2_im * s.iq_b2_im) / 1099511627776.0));
    printf(" dBc\n");
    printf("  updates        : %lu   (refined only while a strong jammer is present)\n",
           (unsigned long)s.n_iq_upd);
    printf("  WHAT IT DOES   : the AD9361 leaves a mirror image of every signal at\n"
           "                   about -50 dBc. With one null, that image of a strong\n"
           "                   jammer is a second interferer and limits the null to\n"
           "                   ~50 dB JNR. The correction removes it per channel.\n"
           "                   Expect -45..-55 dBc on both once a jammer has been on.\n");
    printf("-----------------------------------\n");
}

int32_t gnss_pt_mvdr_set_coh(uint32_t coh_sh)
{
    uint32_t sm;

    if (gnss_pt_read(GNSS_PT_REG_MVDR_ID) != GNSS_PT_MVDR_EXPECTED_ID) {
        return GNSS_PT_ERR_MVDR_ID;
    }
    if (gnss_pt_read(GNSS_PT_REG_MVDR_VERSION) < GNSS_PT_MVDR_VERSION_SMOOTH) {
        return GNSS_PT_ERR_MVDR_OLD;
    }
    if (coh_sh < 1U)  { coh_sh = 1U; }
    if (coh_sh > 15U) { coh_sh = 15U; }
    gnss_pt_coh_user = coh_sh;
    sm = gnss_pt_read(GNSS_PT_REG_MVDR_SMOOTH);
    sm = (sm & ~(0xFU << 24)) | GNSS_PT_SM_COH_SH(coh_sh);
    gnss_pt_write(GNSS_PT_REG_MVDR_SMOOTH, sm);
    return gnss_pt_mvdr_commit();
}

void gnss_pt_mvdr_print_out_gain(void)
{
    gnss_pt_mvdr_state_t s;
    double pout, pdac, d;
    long   sug;
    int    aolc;

    gnss_pt_mvdr_get_state(&s);
    printf("\n--- MVDR output level ---\n");
    if (s.id != GNSS_PT_MVDR_EXPECTED_ID || !s.have_ogain) {
        printf("  OUT_GAIN not in this bitstream (needs MVDR 1.4).\n");
        printf("-----------------------------------\n");
        return;
    }
    aolc = (s.have_aolc && (s.out_gain_reg & GNSS_PT_OG_AOLC_EN)) ? 1 : 0;
    if (aolc) {
        printf("  mode           : AUTOMATIC (AOLC), target -%lu dBFS output noise floor\n",
               (unsigned long)((s.out_gain_reg >> 16) & 0xFFU));
        if (aolc_back_db) {
            printf("  clip back-off  : %lu dB (output clipped; given back 1 dB per 30 s clean)\n",
                   (unsigned long)aolc_back_db);
        }
    } else {
        printf("  mode           : manual, OUT_GAIN %lu  (x%lu, +%lu dB, MVDR path only)\n",
               (unsigned long)s.out_gain, (unsigned long)(1UL << s.out_gain),
               (unsigned long)(6UL * s.out_gain));
    }
    if (s.have_aolc) {
        printf("  measured       : output ");
        gnss_pt_print_f3(3.0103 * ((double)s.out_lvl_q4 / 16.0 - 23.0));
        printf(" dBFS (quietest of the last 4 blocks), gain in use ");
        gnss_pt_print_f3(6.0206 * (double)s.out_gain_q8 / 256.0);
        printf(" dB\n");
    }
    printf("  out_clipped    : %lu samples since the last cnt_clear\n",
           (unsigned long)s.n_osat);

    if (s.have_aolc) {
        printf("  RECOMMENDED    : gnss_aolc=24 -- holds the output at -24 dBFS whatever\n"
               "                   the RX gain or null state: no C/N0 lost in the 12-bit\n"
               "                   output and DAC (0.5..4.8 dB otherwise). It RAISES TX1\n"
               "                   by up to ~45 dB: set tx1_attenuation= for -24 dBFS\n"
               "                   into your receiver FIRST.%s\n",
               aolc ? "  (already on)" : "");
        printf("-----------------------------------\n");
        return;
    }

    pout = gnss_pt_mvdr_pout(&s);
    if (pout < 0.0) {
        printf("  P_dac          : not computable right now (no coherent window)\n");
    } else {
        pdac = pout * (double)(1UL << (2U * s.out_gain));
        d    = gnss_pt_db10(pdac / GNSS_PT_FS2);
        printf("  P_dac (to TX1) : "); gnss_pt_print_f3(d); printf(" dBFS\n");
        {
            gnss_pt_jam_state_t j;
            gnss_pt_jam_get_state(&j);
            if (j.available && j.valid && j.present) {
                printf("  suggestion     : none while a jammer is present -- set\n"
                       "                   OUT_GAIN with the jammer OFF.\n");
                printf("-----------------------------------\n");
                return;
            }
        }
        sug = (long)s.out_gain + (long)floor((-30.0 - d) / 6.02 + 0.5);
        if (sug < 0) { sug = 0; }
        if (sug > (long)GNSS_PT_MVDR_OUT_GAIN_MAX) { sug = (long)GNSS_PT_MVDR_OUT_GAIN_MAX; }
        printf("  suggestion     : gnss_aolc=%lu (automatic, recommended), or by hand\n"
               "                   gnss_ogain=%ld for a DAC level near -30 dBFS.\n"
               "                   Only meaningful with the jammer OFF (a nulled\n"
               "                   jammer leaves the noise floor, which is what\n"
               "                   this sets). Each step is +6 dB at TX1 too:\n"
               "                   add 6000 mdB to tx1_attenuation= per step if\n"
               "                   the receiver should see the same level.\n",
               (unsigned long)GNSS_CFG_AOLC_TARGET_DB, sug);
    }
    printf("-----------------------------------\n");
}

uint32_t gnss_pt_lclk_hz(void)
{
    uint32_t v = 0;
    no_os_axi_io_read(GNSS_PT_AD9361_BASEADDR, GNSS_PT_AD9361_REG_CLK_FREQ, &v);
    /* REG_CLK_FREQ counts l_clk during 65536 cycles of the 100 MHz AXI clock */
    return (uint32_t)(((uint64_t)v * 100000000ULL) >> 16);
}

/* Prints Hz as  123.45 MHz  (integer maths). */
static void gnss_pt_print_mhz(uint32_t hz)
{
    printf("%lu.%02lu MHz", (unsigned long)(hz / 1000000UL),
           (unsigned long)((hz % 1000000UL) / 10000UL));
}

void gnss_pt_print_lclk(uint32_t rx_rate_hz)
{
    uint32_t f = gnss_pt_lclk_hz();

    printf("  l_clk measured : ");
    gnss_pt_print_mhz(f);
    printf("   (axi_ad9361 CLK_FREQ: the MVDR datapath clock)\n");
    if (rx_rate_hz != 0U) {
        printf("  l_clk expected : ");
        gnss_pt_print_mhz(rx_rate_hz * GNSS_PT_LCLK_PER_SAMPLE);
        printf("   (%lu x RX rate, LVDS 2R2T)\n", (unsigned long)GNSS_PT_LCLK_PER_SAMPLE);
    }
    if (f == 0U) {
        printf("  WARNING: l_clk reads 0 -- the AD9361 interface clock is not\n"
               "           running (radio not initialised?).\n");
    } else if (f > GNSS_PT_LCLK_CONSTRAINT_MHZ * 1000000UL) {
        printf("  *** WARNING: l_clk is above the %lu MHz the timing was closed\n"
               "      at (rx_clk create_clock in the XDC). Vivado's 'timing met'\n"
               "      does NOT cover this board: the covariance and weights can\n"
               "      be corrupted at random. Constrain rx_clk at the REAL period\n"
               "      (ADI's 4.000 ns covers up to 250 MHz) and close timing again.\n"
               "      Then edit GNSS_PT_LCLK_CONSTRAINT_MHZ in gnss_passthrough.h. ***\n",
               (unsigned long)GNSS_PT_LCLK_CONSTRAINT_MHZ);
    } else {
        printf("  timing         : l_clk within the %lu MHz constraint: OK\n",
               (unsigned long)GNSS_PT_LCLK_CONSTRAINT_MHZ);
    }
}

/******************************************************************************
 *  Jammer estimator (GNSS-CRPA MOD-12).
 *  The FPGA (jammer_est.v) does all the maths: jammer/noise split of the 2x2
 *  covariance (eigenvalues before 1.11, RX1/RX2 coherence since -- correct
 *  with unequal channel gains), CORDIC phase, arcsin, dB conversion and the
 *  present/absent decision. This code only reads the results and prints them. Every printed
 *  number is integer arithmetic -- no printf float support needed.
 *****************************************************************************/

/* Prints a value given in hundredths as  +12.34 / -0.05 . */
static void gnss_pt_print_c2(long v100)
{
    long a = labs(v100);
    printf("%c%ld.%02ld", (v100 < 0) ? '-' : '+', a / 100L, a % 100L);
}

/* dB in Q8 -> hundredths of a dB, rounded. */
static long gnss_pt_q8_to_c2(int16_t q8)
{
    long v = (long)q8 * 100L;
    return (v >= 0) ? (v + 128L) / 256L : -((-v + 128L) / 256L);
}

static void gnss_pt_print_db_q8(int16_t q8, const char *unit)
{
    if (q8 == (int16_t)GNSS_PT_JAM_Q8_NONE) {
        printf("  (no energy)");
        return;
    }
    gnss_pt_print_c2(gnss_pt_q8_to_c2(q8));
    printf(" %s", unit);
}

/* ---- dBFS -> dBm ------------------------------------------------------- */
static long gnss_pt_jam_k_c2 = GNSS_PT_JAM_DBM_K_C2_DEFAULT;

long gnss_pt_jam_get_dbm_k(void)      { return gnss_pt_jam_k_c2; }
void gnss_pt_jam_set_dbm_k(long k_c2) { gnss_pt_jam_k_c2 = k_c2; }

/* The jammer power is per element; with equal gains the two channels read the
 * same. If they differ, use the mean -- and the report warns about it. */
static long gnss_pt_jam_gain_c2(int32_t g1, int32_t g2)
{
    return ((long)g1 + (long)g2) * 50L;      /* mean gain, hundredths of a dB */
}

/* Prints  -57.04 dBm   or  "dBm not calibrated"  for a dBFS Q8 value. */
static void gnss_pt_print_dbm(int16_t dbfs_q8, int32_t g1, int32_t g2, int gain_ok)
{
    if (dbfs_q8 == (int16_t)GNSS_PT_JAM_Q8_NONE) {
        printf("(no energy)");
        return;
    }
    if (!gain_ok) {
        printf("dBm n/a (RX gain unreadable)");
        return;
    }
    if (gnss_pt_jam_k_c2 == GNSS_PT_JAM_K_UNSET) {
        printf("dBm not calibrated");
        return;
    }
    gnss_pt_print_c2(gnss_pt_q8_to_c2(dbfs_q8) - gnss_pt_jam_gain_c2(g1, g2)
                     + gnss_pt_jam_k_c2);
    printf(" dBm");
}

/* Input-referred power in dB:  P[dB] = P[dBFS] - RX gain[dB]  (hundredths).
 * Removes the RX gain, so the number does NOT change when rx1/rx2_rf_gain
 * is changed -- it follows the signal at the antenna connector, like the
 * attenuation values follow TX power. Reference: 0 dB = the input level that
 * reads 0 dBFS (ADC full scale) at 0 dB RX gain. It differs from dBm only by
 * a fixed board constant, which gnss_jpcal= measures. */
static long gnss_pt_jam_in_db_c2(int16_t dbfs_q8, int32_t g1, int32_t g2)
{
    return gnss_pt_q8_to_c2(dbfs_q8) - gnss_pt_jam_gain_c2(g1, g2);
}

/* Prints  -57.04 dB (-57040 mdB)  -- the same mdB style as tx1_attenuation=. */
static void gnss_pt_print_in_db(int16_t dbfs_q8, int32_t g1, int32_t g2,
                                int gain_ok, int with_mdb)
{
    long v;

    if (dbfs_q8 == (int16_t)GNSS_PT_JAM_Q8_NONE) { printf("(no energy)"); return; }
    if (!gain_ok) { printf("dB n/a (RX gain unreadable)"); return; }
    v = gnss_pt_jam_in_db_c2(dbfs_q8, g1, g2);
    gnss_pt_print_c2(v);
    printf(" dB");
    if (with_mdb) { printf(" (%ld mdB)", v * 10L); }
}

/* dBm only when calibrated; prints nothing otherwise. */
static void gnss_pt_print_dbm_if_cal(int16_t dbfs_q8, int32_t g1, int32_t g2, int gain_ok)
{
    if (dbfs_q8 == (int16_t)GNSS_PT_JAM_Q8_NONE || !gain_ok ||
        gnss_pt_jam_k_c2 == GNSS_PT_JAM_K_UNSET) {
        return;
    }
    printf(" = ");
    gnss_pt_print_dbm(dbfs_q8, g1, g2, gain_ok);
}

static int gnss_pt_jam_available(void)
{
    if (gnss_pt_read(GNSS_PT_REG_MVDR_ID) != GNSS_PT_MVDR_EXPECTED_ID) { return 0; }
    return (gnss_pt_read(GNSS_PT_REG_MVDR_VERSION) >= GNSS_PT_MVDR_VERSION_JAM) ? 1 : 0;
}

void gnss_pt_jam_get_state(gnss_pt_jam_state_t *st)
{
    uint32_t n0, n1, flags = 0, ang = 0, raw = 0, pwr = 0, jnr = 0, c, thr;
    int tries;

    if (!st) { return; }
    memset(st, 0, sizeof(*st));

    st->available = gnss_pt_jam_available();
    if (!st->available) { return; }

    /* All result registers come from one estimate when JAM_N_EST did not
     * move across the reads (estimates arrive every ~1.3 ms, the reads take
     * a few microseconds, so the first try almost always succeeds). */
    for (tries = 0; tries < 8; tries++) {
        n0    = gnss_pt_read(GNSS_PT_REG_JAM_N_EST);
        flags = gnss_pt_read(GNSS_PT_REG_JAM_STATUS);
        ang   = gnss_pt_read(GNSS_PT_REG_JAM_ANGLE);
        raw   = gnss_pt_read(GNSS_PT_REG_JAM_PHRAW);
        pwr   = gnss_pt_read(GNSS_PT_REG_JAM_POWER);
        jnr   = gnss_pt_read(GNSS_PT_REG_JAM_JNR);
        n1    = gnss_pt_read(GNSS_PT_REG_JAM_N_EST);
        if (n0 == n1) { st->coherent = 1; break; }
    }

    st->present       = (flags & GNSS_PT_JAM_ST_PRESENT)    ? 1 : 0;
    st->valid         = (flags & GNSS_PT_JAM_ST_VALID)      ? 1 : 0;
    st->sin_clip      = (flags & GNSS_PT_JAM_ST_SIN_CLIP)   ? 1 : 0;
    st->no_signal     = (flags & GNSS_PT_JAM_ST_NO_SIGNAL)  ? 1 : 0;
    st->noise_zero    = (flags & GNSS_PT_JAM_ST_NOISE_ZERO) ? 1 : 0;
    st->theta_cdeg    = (int16_t)( ang        & 0xFFFFU);
    st->psi_cdeg      = (int16_t)((ang >> 16) & 0xFFFFU);
    st->psi_raw_bin   = (int16_t)( raw        & 0xFFFFU);
    st->psi_raw_cdeg  = (int16_t)((raw >> 16) & 0xFFFFU);
    st->jam_dbfs_q8   = (int16_t)( pwr        & 0xFFFFU);
    st->noise_dbfs_q8 = (int16_t)((pwr >> 16) & 0xFFFFU);
    st->jnr_db_q8     = (int16_t)( jnr        & 0xFFFFU);
    st->n_est         = n1;
    st->n_drop        = gnss_pt_read(GNSS_PT_REG_JAM_N_DROP);

    c   = gnss_pt_read(GNSS_PT_REG_JAM_CTRL);
    thr = gnss_pt_read(GNSS_PT_REG_JAM_THR);
    st->cfg_enable   = (c & GNSS_PT_JAM_CTRL_EN)  ? 1 : 0;
    st->cfg_invert   = (c & GNSS_PT_JAM_CTRL_INV) ? 1 : 0;
    st->cfg_avg_log2 = (int)((c & GNSS_PT_JAM_CTRL_AVG_MASK) >> GNSS_PT_JAM_CTRL_AVG_SHIFT);
    st->cfg_cal_bin  = (int16_t)(gnss_pt_read(GNSS_PT_REG_JAM_CAL) & 0xFFFFU);
    st->cfg_k_q14    = (uint16_t)(gnss_pt_read(GNSS_PT_REG_JAM_K) & 0xFFFFU);
    st->cfg_on_q8    = (int16_t)( thr        & 0xFFFFU);
    st->cfg_off_q8   = (int16_t)((thr >> 16) & 0xFFFFU);

    /* MVDR >= 1.11: per-channel average / peak jammer power and noise floor.
     * Read separately from the coherent group above; they belong to the same
     * estimate unless one completed in between (~1 ms apart). */
    if (gnss_pt_read(GNSS_PT_REG_MVDR_VERSION) >= GNSS_PT_MVDR_VERSION_JAM2) {
        uint32_t a = gnss_pt_read(GNSS_PT_REG_JAM_PWR_CH);
        uint32_t p = gnss_pt_read(GNSS_PT_REG_JAM_PEAK);
        uint32_t n = gnss_pt_read(GNSS_PT_REG_NOISE_CH);
        st->have_ch = 1;
        st->j1_q8  = (int16_t)( a        & 0xFFFFU);
        st->j2_q8  = (int16_t)((a >> 16) & 0xFFFFU);
        st->pk1_q8 = (int16_t)( p        & 0xFFFFU);
        st->pk2_q8 = (int16_t)((p >> 16) & 0xFFFFU);
        st->n1_q8  = (int16_t)( n        & 0xFFFFU);
        st->n2_q8  = (int16_t)((n >> 16) & 0xFFFFU);
    }
}

/* Binary angle (32768 = 180 deg) -> hundredths of a degree. */
static long gnss_pt_bin_to_cdeg(int16_t b)
{
    long v = (long)b * 18000L;
    return (v >= 0) ? (v + 16384L) / 32768L : -((-v + 16384L) / 32768L);
}

void gnss_pt_jam_print_state(int32_t g1, int32_t g2, int gain_ok)
{
    gnss_pt_jam_state_t s;

    gnss_pt_jam_get_state(&s);

    printf("\n--- Jammer estimator (RX1/RX2 covariance) ---\n");
    if (!s.available) {
        printf("  Not available: this bitstream's MVDR window is older than 1.2\n"
               "  (no jammer_est.v). Reprogram the FPGA with the current build.\n");
        printf("-----------------------------------\n");
        return;
    }
    if (!s.cfg_enable) {
        printf("  Estimator is DISABLED (JAM_CTRL[0] = 0).\n");
        printf("-----------------------------------\n");
        return;
    }
    if (!s.valid || s.n_est == 0U) {
        printf("  No estimate yet. If this persists, RX samples are not reaching\n"
               "  the MVDR core (check gnss_mvdr? N_BLOCKS).\n");
        printf("-----------------------------------\n");
        return;
    }

    /* ---- verdict first ---- */
    if (s.no_signal) {
        printf("  RESULT     : NO SIGNAL -- covariance is zero (RX1/RX2 delivering\n"
               "               no samples, or both inputs dead).\n");
    } else if (s.present) {
        printf("  RESULT     : *** JAMMER PRESENT ***\n");
        printf("  Angle      : ");
        gnss_pt_print_c2(s.theta_cdeg);
        printf(" deg from broadside (+ = towards RX2, - = towards RX1)%s\n",
               s.sin_clip ? "  [CLIPPED]" : "");
        printf("  Power      : ");
        gnss_pt_print_in_db(s.jam_dbfs_q8, g1, g2, gain_ok, 1);
        gnss_pt_print_dbm_if_cal(s.jam_dbfs_q8, g1, g2, gain_ok);
        printf("   per element\n");
        printf("               (ADC reads ");
        gnss_pt_print_db_q8(s.jam_dbfs_q8, "dBFS");
        printf(" at the current RX gain)\n");
        printf("  JNR        : ");
        if (s.noise_zero) { printf("above range (noise estimate = 0)\n"); }
        else { gnss_pt_print_db_q8(s.jnr_db_q8, "dB"); printf("\n"); }
        if (s.have_ch) {
            /* per element: RX1 and RX2 separately, average and peak (on) power */
            printf("  RX1 / RX2  : average ");
            gnss_pt_print_in_db(s.j1_q8, g1, g1, gain_ok, 0);
            printf(" / ");
            gnss_pt_print_in_db(s.j2_q8, g2, g2, gain_ok, 0);
            printf("   (ADC ");
            gnss_pt_print_db_q8(s.j1_q8, "/");
            gnss_pt_print_db_q8(s.j2_q8, "dBFS)");
            printf("\n               peak    ");
            gnss_pt_print_in_db(s.pk1_q8, g1, g1, gain_ok, 0);
            printf(" / ");
            gnss_pt_print_in_db(s.pk2_q8, g2, g2, gain_ok, 0);
            printf("   (strongest 182 us window: the ON power of a pulsed jammer)\n");
            if (s.j1_q8 != (int16_t)GNSS_PT_JAM_Q8_NONE &&
                s.pk1_q8 != (int16_t)GNSS_PT_JAM_Q8_NONE &&
                (long)s.pk1_q8 - (long)s.j1_q8 > 3L * 256L) {
                printf("               peak > average by more than 3 dB: the jammer is\n"
                       "               PULSED or intermittent (STATUS HOLD keeps the null).\n");
            }
        }
    } else {
        printf("  RESULT     : No jammer present\n");
        printf("  JNR        : ");
        gnss_pt_print_db_q8(s.jnr_db_q8, "dB");
        printf("   (detect threshold ");
        gnss_pt_print_c2(gnss_pt_q8_to_c2(s.cfg_on_q8));
        printf(" dB)\n");
    }
    printf("  Noise floor: ");
    gnss_pt_print_in_db(s.noise_dbfs_q8, g1, g2, gain_ok, 1);
    gnss_pt_print_dbm_if_cal(s.noise_dbfs_q8, g1, g2, gain_ok);
    printf("   per element, whole RX band\n");
    if (s.have_ch) {
        printf("               RX1 ");
        gnss_pt_print_db_q8(s.n1_q8, "dBFS");
        printf(", RX2 ");
        gnss_pt_print_db_q8(s.n2_q8, "dBFS");
        printf("   (target ");
        gnss_pt_print_f3(GNSS_PT_RX_TARGET_DBFS);
        printf(" dBFS: gnss_rxgain?)\n");
    }
    printf("               (ADC reads ");
    gnss_pt_print_db_q8(s.noise_dbfs_q8, "dBFS");
    printf(")\n");
    if (gain_ok) {
        printf("  RX gain       : RX1 %ld dB, RX2 %ld dB", (long)g1, (long)g2);
        if (gnss_pt_jam_k_c2 == GNSS_PT_JAM_K_UNSET) {
            printf(",  dBm cal: NOT DONE\n");
        } else {
            printf(",  dBm offset=");
            gnss_pt_print_c2(gnss_pt_jam_k_c2);
            printf(" dB\n");
        }
    }

    /* ---- detail ---- */
    printf("  Phase RX1-RX2 : raw ");
    gnss_pt_print_c2(s.psi_raw_cdeg);
    printf(" deg (bin %ld),  cal ", (long)s.psi_raw_bin);
    gnss_pt_print_c2(gnss_pt_bin_to_cdeg(s.cfg_cal_bin));
    printf(" deg,  used ");
    gnss_pt_print_c2(s.psi_cdeg);
    printf(" deg\n");
    {
        /* MVDR >= 1.3: windows overlap (one every 1024 samples), so 2^n of
         * them span N + 1024*(2^n - 1) samples, not N * 2^n. */
        uint32_t mv = gnss_pt_read(GNSS_PT_REG_MVDR_VERSION);
        long bn = (long)gnss_pt_mvdr_block_n(mv);
        long span = (mv >= GNSS_PT_MVDR_VERSION_CLIP)
                  ? bn + (long)GNSS_PT_MVDR_HOP_N * ((1L << s.cfg_avg_log2) - 1L)
                  : bn << s.cfg_avg_log2;
        printf("  Settings      : avg 2^%ld windows (%ld samples), K=%lu (Q14, lambda/2d),\n"
               "                  detect ",
               (long)s.cfg_avg_log2, span, (unsigned long)s.cfg_k_q14);
    }
    gnss_pt_print_c2(gnss_pt_q8_to_c2(s.cfg_on_q8));
    printf(" dB / release ");
    gnss_pt_print_c2(gnss_pt_q8_to_c2(s.cfg_off_q8));
    printf(" dB, phase invert=%ld\n", (long)s.cfg_invert);
    printf("  Counters      : estimates=%lu  dropped=%lu%s\n",
           (unsigned long)s.n_est, (unsigned long)s.n_drop,
           s.coherent ? "" : "   (read NOT coherent -- values may mix two estimates)");

    if (s.present && s.sin_clip) {
        printf("  NOTE: the measured phase is larger than a plane wave can produce\n"
               "        for this spacing, so the angle was clipped to +-90 deg.\n"
               "        Usually means the RX1/RX2 phase is not calibrated\n"
               "        (gnss_jcal=1 with a source at broadside).\n");
    }
    if (s.cfg_cal_bin == 0) {
        printf("  NOTE: phase calibration is 0 (not done). Angles include the\n"
               "        RX1/RX2 cable/LO phase offset until gnss_jcal=1 is run.\n");
    }
    if (gain_ok && g1 != g2) {
        printf("  NOTE: RX1 and RX2 gains differ. Set them equal (manual gain)\n"
               "        or the angle and the power are both biased.\n");
    }
    if (gnss_pt_jam_k_c2 == GNSS_PT_JAM_K_UNSET) {
        printf("  NOTE: dBm needs a one-time calibration: feed a CW tone of known\n"
               "        level, e.g. -60 dBm, into RX1 and RX2 (splitter) and run\n"
               "        gnss_jpcal=-60.\n");
    }
    printf("  (2 elements: angle is relative to the RX1-RX2 line only, -90..+90;\n"
           "   front/back and elevation cannot be resolved.\n"
           "   Power dB = dBFS - RX gain: referred to the RX connector, so it\n"
           "   does not change with rx1/rx2_rf_gain. 0 dB = the level that fills\n"
           "   the ADC at 0 dB gain. dBm = dB + offset (offset from gnss_jpcal=).)\n");
    printf("-----------------------------------\n");
}

void gnss_pt_jam_monitor(uint32_t count, uint32_t period_ms,
                         int32_t g1, int32_t g2, int gain_ok)
{
    gnss_pt_jam_state_t s;
    uint32_t i;

    if (!gnss_pt_jam_available()) {
        printf("gnss_jmon: jammer estimator not in this bitstream (MVDR < 1.2)\n");
        return;
    }
    for (i = 1; i <= count; i++) {
        gnss_pt_jam_get_state(&s);
        printf("[%4lu] ", (unsigned long)i);
        if (!s.cfg_enable) {
            printf("estimator disabled\n");
        } else if (!s.valid || s.n_est == 0U) {
            printf("no estimate yet\n");
        } else if (s.no_signal) {
            printf("NO SIGNAL (covariance zero)\n");
        } else if (s.present) {
            printf("JAMMER  angle=");
            gnss_pt_print_c2(s.theta_cdeg);
            printf(" deg%s  power=", s.sin_clip ? "(clip)" : "");
            gnss_pt_print_in_db(s.jam_dbfs_q8, g1, g2, gain_ok, 0);
            gnss_pt_print_dbm_if_cal(s.jam_dbfs_q8, g1, g2, gain_ok);
            printf("  JNR=");
            if (s.noise_zero) { printf(">max"); }
            else { gnss_pt_print_db_q8(s.jnr_db_q8, "dB"); }
            printf("\n");
        } else {
            printf("No jammer present  (JNR=");
            gnss_pt_print_db_q8(s.jnr_db_q8, "dB");
            printf(", noise=");
            gnss_pt_print_in_db(s.noise_dbfs_q8, g1, g2, gain_ok, 0);
            gnss_pt_print_dbm_if_cal(s.noise_dbfs_q8, g1, g2, gain_ok);
            printf(")\n");
        }
        if (i < count) { no_os_mdelay(period_ms); }
    }
}

int32_t gnss_pt_jam_calibrate(int capture)
{
    gnss_pt_jam_state_t s;
    double sc = 0.0, ss = 0.0, a;
    uint32_t last = 0;
    int got = 0, waits = 0;
    int16_t cal;
    const int NAVG = 32;

    if (!gnss_pt_jam_available()) { return GNSS_PT_ERR_JAM_MISSING; }

    if (!capture) {
        cal = 0;
    } else {
        /* Average NAVG separate estimates on the unit circle (a plain mean
         * of angles breaks at +-180 deg). Each must have the source detected,
         * otherwise the phase is just noise. */
        gnss_pt_jam_get_state(&s);
        last = s.n_est;
        while (got < NAVG && waits < 2000) {
            no_os_mdelay(2);
            waits++;
            gnss_pt_jam_get_state(&s);
            if (!s.coherent || s.n_est == last) { continue; }
            last = s.n_est;
            if (!s.present) {
                printf("gnss_jcal: no source detected (JNR ");
                gnss_pt_print_db_q8(s.jnr_db_q8, "dB");
                printf("). Put a CW/jammer source at broadside and retry.\n");
                return GNSS_PT_ERR_JAM_NO_SOURCE;
            }
            a   = (double)s.psi_raw_bin * (3.14159265358979323846 / 32768.0);
            sc += cos(a);
            ss += sin(a);
            got++;
        }
        if (got < NAVG) {
            printf("gnss_jcal: only %ld estimates arrived; calibration aborted\n", (long)got);
            return GNSS_PT_ERR_JAM_NO_SOURCE;
        }
        a = atan2(ss, sc) * (32768.0 / 3.14159265358979323846);
        if (a >  32767.0) { a =  32767.0; }
        if (a < -32768.0) { a = -32768.0; }
        cal = (int16_t)(long)((a >= 0.0) ? a + 0.5 : a - 0.5);
        printf("gnss_jcal: averaged %ld estimates, phase spread R=", (long)got);
        gnss_pt_print_c2((long)(100.0 * sqrt(sc * sc + ss * ss) / (double)got + 0.5));
        printf(" (1.00 = perfectly steady)\n");
    }

    gnss_pt_write(GNSS_PT_REG_JAM_CAL, (uint32_t)(uint16_t)cal);
    if (gnss_pt_mvdr_commit() != GNSS_PT_OK) { return GNSS_PT_ERR_MVDR_COMMIT; }
    printf("gnss_jcal: JAM_CAL = %ld (", (long)cal);
    gnss_pt_print_c2(gnss_pt_bin_to_cdeg(cal));
    printf(" deg) applied.%s\n",
           capture ? " The broadside source should now read ~0 deg." : " Calibration cleared.");
    return GNSS_PT_OK;
}

int32_t gnss_pt_jam_set_threshold(int detect_db)
{
    long on, off;

    if (!gnss_pt_jam_available()) { return GNSS_PT_ERR_JAM_MISSING; }
    on  = (long)detect_db * 256L;
    off = on - 512L;                          /* 2 dB hysteresis */
    gnss_pt_write(GNSS_PT_REG_JAM_THR,
                  ((uint32_t)(uint16_t)off << 16) | (uint32_t)(uint16_t)on);
    if (gnss_pt_mvdr_commit() != GNSS_PT_OK) { return GNSS_PT_ERR_MVDR_COMMIT; }
    printf("gnss_jthr: detect at JNR >= %ld dB, release below %ld dB\n",
           (long)detect_db, (long)detect_db - 2L);
    return GNSS_PT_OK;
}

int32_t gnss_pt_jam_set_invert(int invert)
{
    uint32_t c;

    if (!gnss_pt_jam_available()) { return GNSS_PT_ERR_JAM_MISSING; }
    c = gnss_pt_read(GNSS_PT_REG_JAM_CTRL);
    if (invert) { c |=  GNSS_PT_JAM_CTRL_INV; }
    else        { c &= ~GNSS_PT_JAM_CTRL_INV; }
    gnss_pt_write(GNSS_PT_REG_JAM_CTRL, c);
    if (gnss_pt_mvdr_commit() != GNSS_PT_OK) { return GNSS_PT_ERR_MVDR_COMMIT; }
    printf("gnss_jinv: phase invert = %ld%s\n", (long)(invert ? 1 : 0),
           invert ? "" : " (default)");
    printf("           Re-run gnss_jcal=1 after changing this.\n");
    return GNSS_PT_OK;
}

int32_t gnss_pt_jam_set_avg(uint32_t avg_log2)
{
    uint32_t c;

    if (!gnss_pt_jam_available()) { return GNSS_PT_ERR_JAM_MISSING; }
    if (avg_log2 > 8U) { avg_log2 = 8U; }
    c = gnss_pt_read(GNSS_PT_REG_JAM_CTRL);
    c = (c & ~GNSS_PT_JAM_CTRL_AVG_MASK) | (avg_log2 << GNSS_PT_JAM_CTRL_AVG_SHIFT);
    gnss_pt_write(GNSS_PT_REG_JAM_CTRL, c);
    return gnss_pt_mvdr_commit();
}

int32_t gnss_pt_jam_power_calibrate(long gen_dbm_c2, int32_t g1, int32_t g2)
{
    gnss_pt_jam_state_t s;
    uint32_t last;
    long sum_c2 = 0, dbfs_c2, k_c2;
    int got = 0, waits = 0;
    const int NAVG = 16;

    if (!gnss_pt_jam_available()) { return GNSS_PT_ERR_JAM_MISSING; }

    gnss_pt_jam_get_state(&s);
    last = s.n_est;
    while (got < NAVG && waits < 2000) {
        no_os_mdelay(2);
        waits++;
        gnss_pt_jam_get_state(&s);
        if (!s.coherent || s.n_est == last) { continue; }
        last = s.n_est;

        if (!s.present) {
            printf("gnss_jpcal: no source detected (JNR ");
            gnss_pt_print_db_q8(s.jnr_db_q8, "dB");
            printf("). Is the generator on and connected?\n");
            return GNSS_PT_ERR_JAM_NO_SOURCE;
        }
        dbfs_c2 = gnss_pt_q8_to_c2(s.jam_dbfs_q8);
        if (dbfs_c2 > -1000L) {
            printf("gnss_jpcal: source reads ");
            gnss_pt_print_c2(dbfs_c2);
            printf(" dBFS -- the ADC is clipping. Lower the generator\n"
                   "            level or the RX gain until it reads -15..-40 dBFS.\n");
            return GNSS_PT_ERR_JAM_LEVEL;
        }
        if (s.jnr_db_q8 < (int16_t)(20 * 256)) {
            printf("gnss_jpcal: JNR only ");
            gnss_pt_print_db_q8(s.jnr_db_q8, "dB");
            printf(" -- need >= 20 dB so noise does not bias the\n"
                   "            reading. Raise the generator level or RX gain.\n");
            return GNSS_PT_ERR_JAM_LEVEL;
        }
        sum_c2 += dbfs_c2;
        got++;
    }
    if (got < NAVG) {
        printf("gnss_jpcal: only %ld estimates arrived; aborted\n", (long)got);
        return GNSS_PT_ERR_JAM_NO_SOURCE;
    }

    dbfs_c2 = sum_c2 / (long)got;
    k_c2    = gen_dbm_c2 - dbfs_c2 + gnss_pt_jam_gain_c2(g1, g2);
    gnss_pt_jam_k_c2 = k_c2;

    printf("gnss_jpcal: generator ");
    gnss_pt_print_c2(gen_dbm_c2);
    printf(" dBm reads ");
    gnss_pt_print_c2(dbfs_c2);
    printf(" dBFS at RX gain %ld/%ld dB\n", (long)g1, (long)g2);
    printf("            dBm offset = ");
    gnss_pt_print_c2(k_c2);
    printf(" dB  -> dBm reporting is now ON (until reboot).\n");
    printf("            To keep it permanently, set in gnss_passthrough.h:\n"
           "              #define GNSS_PT_JAM_DBM_K_C2_DEFAULT  (%ldL)\n", k_c2);
    printf("            Valid for this LO frequency and RX path. The gain\n"
           "            is read live, so changing rx*_rf_gain later is fine.\n");
    return GNSS_PT_OK;
}
