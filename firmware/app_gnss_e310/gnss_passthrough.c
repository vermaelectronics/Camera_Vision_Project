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
               "         stage and is 24 dB low. If it reports 1.1, the CRPA\n"
               "         nulling core is not present -- RX passes through\n"
               "         UNMODIFIED no matter what gnss_crpa_alpha= is sent.\n"
               "         If it reports 1.2, the core nulls, but ALWAYS at\n"
               "         alpha=1.0 -- that bitstream's alpha_wr input is\n"
               "         hardwired, so gnss_crpa_alpha= writes succeed and\n"
               "         read back correctly but never reach the algorithm.\n"
               "         If it reports 1.4 or 1.5, gnss_crpa_alpha= IS live\n"
               "         (the standard core is still present there) but\n"
               "         gnss_crpa_mode=/gnss_crpa_gamma_norm=/\n"
               "         gnss_crpa_gamma_pl= write CONTROL/CRPA_COEF bits\n"
               "         that bitstream does not decode the v1.6 way -- a\n"
               "         1.5 board still expects CONTROL[5:4] as 2 bits\n"
               "         (00=standard/01=normalized/10=PL-NPI/11=reserved),\n"
               "         not the single CONTROL[4] bit this firmware sends.\n"
               "         If it reports 1.3 or earlier, mode/gamma control\n"
               "         does not exist on that board at all. Reprogram the\n"
               "         FPGA.\n",
               (unsigned long)(GNSS_PT_EXPECTED_VERSION >> 16),
               (unsigned long)(GNSS_PT_EXPECTED_VERSION & 0xFFFFU),
               (unsigned long)(ver >> 16), (unsigned long)(ver & 0xFFFFU));
    }

    /* v1.6+: CRPA_COEF(1)/(2) (gamma) reset to 0 in hardware, not the
     * documented default of 1 -- the AXI-side crpa_coef[] array's OWN
     * reset value, not the RTL core's GAMMA_INIT=1 (gamma is loaded
     * continuously with no write-strobe, same fix as the v1.2 alpha_wr
     * bug above, so there's no enable line left to distinguish "never
     * written" from "written as 0"; see the firmware README's "Third
     * bug" section for the full account -- confirmed live on hardware,
     * not theoretical). Correct it here, once, right after confirming
     * this is a v1.6+ board (older boards don't have these registers
     * wired to anything meaningful), so every boot starts at the
     * documented safe default instead of silently running with gamma=0
     * until a human remembers to set it from the console. */
    if (ver == GNSS_PT_EXPECTED_VERSION) {
        gnss_pt_set_crpa_gamma_norm(GNSS_PT_CRPA_GAMMA_DEFAULT);
        gnss_pt_set_crpa_gamma_pl(GNSS_PT_CRPA_GAMMA_DEFAULT);
        printf("gnss_pt: gamma initialized to the safe default (%lu) for\n"
               "         both cores -- hardware resets CRPA_COEF(1)/(2) to\n"
               "         0, not %lu, so this firmware corrects it on every\n"
               "         boot rather than leaving the regularisation floor\n"
               "         silently off until gnss_crpa_gamma_norm=/\n"
               "         gnss_crpa_gamma_pl= is sent by hand.\n",
               (unsigned long)GNSS_PT_CRPA_GAMMA_DEFAULT,
               (unsigned long)GNSS_PT_CRPA_GAMMA_DEFAULT);
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

/* GNSS-CRPA MOD-11: CRPA_COEF(0) is plain RW storage on the AXI-Lite side --
 * the FPGA hands back whatever was last written, so this readback is a true
 * confirmation of what the core's alpha_in currently holds (modulo the CDC
 * into the l_clk domain, which is synchronous and adds at most two l_clk
 * cycles of latency -- not observable from software). It is NOT a readback
 * of any live nulling result; the core has no AXI-visible output today. */
void gnss_pt_set_crpa_alpha_raw(uint16_t alpha_q8)
{
    gnss_pt_write(GNSS_PT_REG_CRPA_ALPHA, (uint32_t)alpha_q8);
}

uint16_t gnss_pt_get_crpa_alpha_raw(void)
{
    return (uint16_t)(gnss_pt_read(GNSS_PT_REG_CRPA_ALPHA) & 0xFFFFU);
}

void gnss_pt_set_crpa_alpha(double alpha)
{
    double raw = alpha * (double)(1U << GNSS_PT_CRPA_ALPHA_FRAC_BITS);

    if (raw < 0.0) {
        raw = 0.0;
    } else if (raw > 65535.0) {
        raw = 65535.0;
    }
    gnss_pt_set_crpa_alpha_raw((uint16_t)(raw + 0.5));
}

double gnss_pt_get_crpa_alpha(void)
{
    return (double)gnss_pt_get_crpa_alpha_raw() /
           (double)(1U << GNSS_PT_CRPA_ALPHA_FRAC_BITS);
}

/* v1.6+: CONTROL[4] mode select. Read-modify-write, same pattern as
 * gnss_pt_set_passthrough()/gnss_pt_set_mute() above, so this never
 * disturbs pass_en/mute/swap_iq/ch1_copy while changing only the mode bit. */
void gnss_pt_set_crpa_mode(int pl_npi)
{
    uint32_t c = gnss_pt_read(GNSS_PT_REG_CONTROL);
    if (pl_npi) { c |= GNSS_PT_CTRL_CRPA_MODE; }
    else        { c &= ~GNSS_PT_CTRL_CRPA_MODE; }
    gnss_pt_write(GNSS_PT_REG_CONTROL, c);
    printf("gnss_pt: CRPA mode %s\n", pl_npi ? "PL-NPI" : "normalized");
}

int gnss_pt_get_crpa_mode(void)
{
    return (gnss_pt_read(GNSS_PT_REG_CONTROL) & GNSS_PT_CTRL_CRPA_MODE) ? 1 : 0;
}

/* v1.6+: independent gamma registers. Plain uint32_t in, uint32_t out --
 * no Q-format scaling, unlike alpha (see the header comment on
 * GNSS_PT_REG_CRPA_GAMMA_NORM for why). */
void gnss_pt_set_crpa_gamma_norm(uint32_t gamma)
{
    gnss_pt_write(GNSS_PT_REG_CRPA_GAMMA_NORM, gamma);
}

uint32_t gnss_pt_get_crpa_gamma_norm(void)
{
    return gnss_pt_read(GNSS_PT_REG_CRPA_GAMMA_NORM);
}

void gnss_pt_set_crpa_gamma_pl(uint32_t gamma)
{
    gnss_pt_write(GNSS_PT_REG_CRPA_GAMMA_PL, gamma);
}

uint32_t gnss_pt_get_crpa_gamma_pl(void)
{
    return gnss_pt_read(GNSS_PT_REG_CRPA_GAMMA_PL);
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
    st->crpa_alpha_raw      = gnss_pt_get_crpa_alpha_raw();
    st->crpa_mode           = (st->control & GNSS_PT_CTRL_CRPA_MODE) ? 1 : 0;
    st->crpa_gamma_norm_raw = gnss_pt_get_crpa_gamma_norm();
    st->crpa_gamma_pl_raw   = gnss_pt_get_crpa_gamma_pl();
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
    /* v1.1 and earlier bitstreams accept this write (CRPA_COEF0 is plain RW
     * storage even then) but have no core wired to consume it -- and as of
     * v1.6, NO board consumes it any more (the core alpha fed was removed).
     * Shown for backward-compat visibility only -- cross-check the version
     * line above before reading anything into a non-default value here. */
    printf("  crpa alpha     : raw=%u (%.4f)  %s\n",
           (unsigned)s.crpa_alpha_raw,
           (double)s.crpa_alpha_raw / (double)(1U << GNSS_PT_CRPA_ALPHA_FRAC_BITS),
           (s.version == GNSS_PT_EXPECTED_VERSION)
               ? "(v1.6: UNUSED, standard core removed -- see gamma/mode below)"
               : "(see gnss_pt_probe()'s boot-time warning for what this "
                 "board's version actually does with alpha)");
    /* v1.6+ only -- meaningless bit/register positions on an older board,
     * so gate the whole block on the version actually matching. */
    if (s.version == GNSS_PT_EXPECTED_VERSION) {
        printf("  crpa mode      : %s (CONTROL[4]=%d)\n",
               s.crpa_mode ? "PL-NPI" : "normalized", s.crpa_mode);
        printf("  crpa gamma     : normalized=%lu  pl-npi=%lu\n",
               (unsigned long)s.crpa_gamma_norm_raw,
               (unsigned long)s.crpa_gamma_pl_raw);
    }
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
