/******************************************************************************
 *  gnss_passthrough.c
 *  Driver for the project's custom FPGA processing block.
 *  Original work for the ANTSDR E310 V1 GNSS CRPA project.
 *****************************************************************************/

#include "gnss_passthrough.h"
#include "no_os_axi_io.h"
#include "no_os_delay.h"
#include <stdio.h>

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
               "         the bitstream reports %lu.%lu. If the board reports\n"
               "         1.0, its TX output is missing the sample alignment\n"
               "         stage and is 24 dB low. Reprogram the FPGA.\n",
               (unsigned long)(GNSS_PT_EXPECTED_VERSION >> 16),
               (unsigned long)(GNSS_PT_EXPECTED_VERSION & 0xFFFFU),
               (unsigned long)(ver >> 16), (unsigned long)(ver & 0xFFFFU));
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

/* ---- LCMV CRPA ----------------------------------------------------------- */

void gnss_pt_lcmv_enable(int enable)
{
    uint32_t c = gnss_pt_read(GNSS_PT_REG_CONTROL);
    if (enable) { c |= GNSS_PT_CTRL_LCMV_EN; }
    else        { c &= ~GNSS_PT_CTRL_LCMV_EN; }
    gnss_pt_write(GNSS_PT_REG_CONTROL, c);
}

static int lcmv_clamp_deg(int d)
{
    if (d >  90) { return  90; }
    if (d < -90) { return -90; }
    return d;
}

void gnss_pt_lcmv_set_angles(int gps_deg, int jam_deg)
{
    gnss_pt_write(GNSS_PT_REG_LCMV_GPS_ANGLE, (uint32_t)lcmv_clamp_deg(gps_deg) & 0xFFFFU);
    gnss_pt_write(GNSS_PT_REG_LCMV_JAM_ANGLE, (uint32_t)lcmv_clamp_deg(jam_deg) & 0xFFFFU);
}

void gnss_pt_lcmv_set_eps(float eps)
{
    if (eps < 0.0f)        { eps = 0.0f; }
    if (eps > 0.99998f)    { eps = 0.99998f; }
    gnss_pt_write(GNSS_PT_REG_LCMV_EPS, (uint32_t)(eps * 65536.0f + 0.5f) & 0xFFFFU);
}

static uint32_t lcmv_q214(float v)
{
    float s = v * 16384.0f;
    if (s >  32767.0f) { s =  32767.0f; }
    if (s < -32768.0f) { s = -32768.0f; }
    int32_t q = (int32_t)(s < 0.0f ? s - 0.5f : s + 0.5f);
    return (uint32_t)q & 0xFFFFU;
}

void gnss_pt_lcmv_set_rx2_cal(float re, float im)
{
    gnss_pt_write(GNSS_PT_REG_LCMV_RX2_CAL, (lcmv_q214(im) << 16) | lcmv_q214(re));
}

void gnss_pt_lcmv_get_state(gnss_pt_lcmv_state_t *st)
{
    if (!st) { return; }
    uint32_t ctl = gnss_pt_read(GNSS_PT_REG_CONTROL);
    uint32_t sts = gnss_pt_read(GNSS_PT_REG_STATUS);
    uint32_t ls  = gnss_pt_read(GNSS_PT_REG_LCMV_STATUS);
    uint32_t cal = gnss_pt_read(GNSS_PT_REG_LCMV_RX2_CAL);

    st->enabled       = (ctl & GNSS_PT_CTRL_LCMV_EN) ? 1 : 0;
    st->active        = (sts & GNSS_PT_ST_LCMV_ACTIVE) ? 1 : 0;
    st->weights_valid = (ls >> 1) & 1U;
    st->busy          = (ls >> 2) & 1U;
    st->sat_sticky    = (ls >> 3) & 1U;
    st->gps_deg = (int16_t)(gnss_pt_read(GNSS_PT_REG_LCMV_GPS_ANGLE) & 0xFFFFU);
    st->jam_deg = (int16_t)(gnss_pt_read(GNSS_PT_REG_LCMV_JAM_ANGLE) & 0xFFFFU);
    st->eps     = (float)(gnss_pt_read(GNSS_PT_REG_LCMV_EPS) & 0xFFFFU) / 65536.0f;
    st->cal_re  = (float)(int16_t)(cal & 0xFFFFU) / 16384.0f;
    st->cal_im  = (float)(int16_t)(cal >> 16)     / 16384.0f;
    st->w1_re = (float)(int32_t)gnss_pt_read(GNSS_PT_REG_LCMV_W1_RE) / 65536.0f;
    st->w1_im = (float)(int32_t)gnss_pt_read(GNSS_PT_REG_LCMV_W1_IM) / 65536.0f;
    st->w2_re = (float)(int32_t)gnss_pt_read(GNSS_PT_REG_LCMV_W2_RE) / 65536.0f;
    st->w2_im = (float)(int32_t)gnss_pt_read(GNSS_PT_REG_LCMV_W2_IM) / 65536.0f;
    st->updates   = gnss_pt_read(GNSS_PT_REG_LCMV_UPDATES);
    st->sat_count = gnss_pt_read(GNSS_PT_REG_LCMV_SAT_COUNT);
}

/* Fixed-point to "-0.12345" text without floating-point printf, so the
 * output does not depend on the C library's float support. */
static void lcmv_fmt(char *buf, int32_t q, int frac_bits)
{
    int64_t  v   = q;
    int      neg = v < 0;
    if (neg) { v = -v; }
    int64_t  ip  = v >> frac_bits;
    int64_t  fp  = ((v & ((1LL << frac_bits) - 1)) * 100000 + (1LL << (frac_bits - 1))) >> frac_bits;
    if (fp >= 100000) { ip++; fp -= 100000; }
    sprintf(buf, "%s%ld.%05ld", neg ? "-" : "+", (long)ip, (long)fp);
}

void gnss_pt_lcmv_print_state(void)
{
    gnss_pt_lcmv_state_t s;
    char a[16], b[16];
    uint32_t sts = gnss_pt_read(GNSS_PT_REG_STATUS);
    uint32_t cal = gnss_pt_read(GNSS_PT_REG_LCMV_RX2_CAL);
    gnss_pt_lcmv_get_state(&s);

    printf("\n--- LCMV CRPA (2 antennas, look + null) ---\n");
    printf("  enabled        : %s\n", s.enabled ? "yes" : "no");
    printf("  driving TX1    : %s\n", s.active ? "YES" :
           (s.enabled && !(sts & GNSS_PT_ST_PASS_EN_SYNCED)) ? "no (passthrough off, gnss_tx=1)" : "no");
    printf("  gps / jam angle: %d / %d deg\n", s.gps_deg, s.jam_deg);
    lcmv_fmt(a, (int32_t)(gnss_pt_read(GNSS_PT_REG_LCMV_EPS) & 0xFFFFU), 16);
    printf("  eps            : %s\n", a);
    lcmv_fmt(a, (int16_t)(cal & 0xFFFFU), 14);
    lcmv_fmt(b, (int16_t)(cal >> 16), 14);
    printf("  rx2 cal        : %s %sj\n", a, b);
    lcmv_fmt(a, (int32_t)gnss_pt_read(GNSS_PT_REG_LCMV_W1_RE), 16);
    lcmv_fmt(b, (int32_t)gnss_pt_read(GNSS_PT_REG_LCMV_W1_IM), 16);
    printf("  w1             : %s %sj\n", a, b);
    lcmv_fmt(a, (int32_t)gnss_pt_read(GNSS_PT_REG_LCMV_W2_RE), 16);
    lcmv_fmt(b, (int32_t)gnss_pt_read(GNSS_PT_REG_LCMV_W2_IM), 16);
    printf("  w2 (with cal)  : %s %sj\n", a, b);
    printf("  weights        : %s%s, updates=%lu\n", s.weights_valid ? "valid" : "NOT valid",
           s.busy ? " (solving)" : "", (unsigned long)s.updates);
    printf("  output clipped : %lu samples%s\n", (unsigned long)s.sat_count,
           s.sat_sticky ? "  <- lower the RX gain" : "");
    printf("-------------------------------------------\n");
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
    printf("  lcmv           : %s%s\n",
           (s.status & GNSS_PT_ST_LCMV_EN_SYNCED) ? "enabled" : "off",
           (s.status & GNSS_PT_ST_LCMV_ACTIVE) ? " (ACTIVE on TX1, see gnss_lcmv?)" : "");
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
