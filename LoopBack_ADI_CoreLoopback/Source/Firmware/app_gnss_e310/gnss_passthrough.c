/******************************************************************************
 *  gnss_passthrough.c  --  CORE-LOOPBACK version (no custom FPGA block)
 *
 *  This pipeline has NO gnss_passthrough IP. The RX -> TX loopback is done by
 *  the stock ADI axi_ad9361 core itself:
 *
 *    axi_ad9361_tx_channel.v   case (dac_data_sel)
 *                                4'h8: dac_data_out_int <= adc_data;   // RX sample of the same channel
 *                                4'h2: dac_data_out_int <= dma_data[15:4];
 *                                4'h3: dac_data_out_int <= 12'd0;
 *
 *  i.e. DAC channel c (CHAN_CNTRL_7 at 0x0418 + c*0x40 in the DAC register
 *  space) set to 8 retransmits the raw 12-bit RX sample of channel c: TX1 = RX1,
 *  TX2 = RX2, a few clock cycles of delay, no DMA, no custom logic.
 *
 *  To keep the rest of the firmware unchanged, this file keeps the old API
 *  (gnss_pt_*) but implements it WITHOUT touching 0x43C0_0000 (there is no
 *  hardware there any more; a read would fault):
 *
 *    pass_en  (gnss_pt_set_passthrough)  -> loopback wanted
 *    mute     (gnss_pt_set_mute)         -> zeros wanted
 *    the source the firmware asks for with gnss_l1_set_dac_source():
 *      DMA  + pass_en + !mute  -> written as 8 (ADC loopback)
 *      DMA  + mute             -> written as 3 (zero)
 *      DMA  + !pass_en         -> written as 2 (real DMA, e.g. the DDR round trip)
 *      anything else           -> written as asked
 *
 *  gnss_pt_read() returns a SYNTHESISED value for the registers the firmware
 *  still reads (ID, VERSION, CONTROL, STATUS); STATUS bits [2]/[3] are set when
 *  the DAC really carries samples (read back from CHAN_CNTRL_7 = 2 or 8).
 *****************************************************************************/
#include "gnss_passthrough.h"
#include "ad9361_api.h"
#include "axi_dac_core.h"
#include "axi_adc_core.h"
#include "no_os_axi_io.h"
#include "no_os_delay.h"
#include <stdio.h>

extern struct ad9361_rf_phy *ad9361_phy;

#define DAC_CHAN_CNTRL_7(c)   (0x0418U + (uint32_t)(c) * 0x40U)
#define ADC_REG_CLK_FREQ      0x0054U
#define ADC_REG_STATUS        0x005CU

static int      pass_en;
static int      mute;
static int      req_sel = AXI_DAC_DATA_SEL_ZERO;   /* what the firmware last asked for */
static uint32_t scratch;

static int have_dac(void) { return ad9361_phy && ad9361_phy->tx_dac; }

/* Read back DAC channel 0's data select (what the DAC really uses). -1 if unknown. */
int32_t gnss_pt_dac_sel_readback(void)
{
    uint32_t v = 0;
    if (!have_dac()) return -1;
    if (no_os_axi_io_read(ad9361_phy->tx_dac->base, DAC_CHAN_CNTRL_7(0), &v) != 0) return -1;
    return (int32_t)(v & 0xFU);
}

/* Map the requested source onto the core-loopback scheme and write it. Returns the select written. */
int32_t gnss_pt_dac_route(int32_t sel)
{
    int32_t eff = sel;
    req_sel = sel;
    if (sel == AXI_DAC_DATA_SEL_DMA) {
        if (mute)          eff = AXI_DAC_DATA_SEL_ZERO;
        else if (pass_en)  eff = AXI_DAC_DATA_SEL_LB;
    }
    if (have_dac())
        axi_dac_set_datasel(ad9361_phy->tx_dac, -1, (enum axi_dac_data_sel)eff);
    return eff;
}

static void reapply(void)
{
    if (req_sel == AXI_DAC_DATA_SEL_DMA) (void)gnss_pt_dac_route(req_sel);
}

uint32_t gnss_pt_read(uint32_t offset)
{
    uint32_t st = 0;
    int32_t  s;
    switch (offset) {
    case GNSS_PT_REG_ID:      return GNSS_PT_EXPECTED_ID;
    case GNSS_PT_REG_VERSION: return GNSS_PT_EXPECTED_VERSION;
    case GNSS_PT_REG_SCRATCH: return scratch;
    case GNSS_PT_REG_CONTROL: return (pass_en ? GNSS_PT_CTRL_PASS_EN : 0U) | (mute ? GNSS_PT_CTRL_MUTE : 0U);
    case GNSS_PT_REG_STATUS:
        s = gnss_pt_dac_sel_readback();
        if (s == AXI_DAC_DATA_SEL_DMA || s == AXI_DAC_DATA_SEL_LB)
            st |= GNSS_PT_ST_DAC_EN_I0 | GNSS_PT_ST_DAC_EN_Q0;
        st |= GNSS_PT_ST_ADC_EN_I0 | GNSS_PT_ST_ADC_EN_Q0;
        if (pass_en) st |= GNSS_PT_ST_PASS_EN_SYNCED;
        return st;
    default:
        return 0;   /* counters, snapshots, FIFO level: not available in this pipeline */
    }
}

void gnss_pt_write(uint32_t offset, uint32_t value)
{
    if (offset == GNSS_PT_REG_SCRATCH) scratch = value;
    else if (offset == GNSS_PT_REG_CONTROL) {
        pass_en = (value & GNSS_PT_CTRL_PASS_EN) ? 1 : 0;
        mute    = (value & GNSS_PT_CTRL_MUTE) ? 1 : 0;
        reapply();
    }
}

int32_t gnss_pt_probe(void)
{
    if (!have_dac()) {
        printf("gnss_pt: no AD9361 DAC core -- cannot use the core loopback\n");
        return GNSS_PT_ERR_ID;
    }
    printf("gnss_pt: CORE-LOOPBACK pipeline (no custom IP): RX->TX by axi_ad9361 DAC data select 8\n");
    return GNSS_PT_OK;
}

void gnss_pt_set_passthrough(int enable) { pass_en = enable ? 1 : 0; reapply(); }
void gnss_pt_set_mute(int m)             { mute = m ? 1 : 0; reapply(); }
void gnss_pt_clear_counters(void)        { }

void gnss_pt_get_state(gnss_pt_state_t *st)
{
    if (!st) return;
    *st = (gnss_pt_state_t){0};
    st->id = GNSS_PT_EXPECTED_ID;
    st->version = GNSS_PT_EXPECTED_VERSION;
    st->control = gnss_pt_read(GNSS_PT_REG_CONTROL);
    st->status = gnss_pt_read(GNSS_PT_REG_STATUS);
    st->passthrough_enabled = pass_en;
}

/* AD9361 interface clock from the ADC core's clock monitor: f = 100 MHz * reg / 2^16. */
static uint32_t if_clk_khz(uint32_t *status)
{
    uint32_t f = 0, s = 0;
    if (!ad9361_phy || !ad9361_phy->rx_adc) { if (status) *status = 0; return 0; }
    axi_adc_read(ad9361_phy->rx_adc, ADC_REG_CLK_FREQ, &f);
    axi_adc_read(ad9361_phy->rx_adc, ADC_REG_STATUS, &s);
    if (status) *status = s;
    return (uint32_t)(((uint64_t)f * 100000ULL) >> 16);
}

void gnss_pt_print_state(void)
{
    static const char *nm[] = { "DDS", "SED", "DMA", "ZERO", "PN7", "PN15", "PN23", "PN31", "ADC LOOPBACK", "PNXX" };
    int32_t s = gnss_pt_dac_sel_readback();
    uint32_t st = 0, khz = if_clk_khz(&st);
    printf("---- core loopback (no custom IP) ----\n");
    printf("  loopback wanted : %d   mute: %d\n", pass_en, mute);
    printf("  DAC data select : %ld (%s)%s\n", (long)s,
           (s >= 0 && s <= 9) ? nm[s] : "?",
           (s == AXI_DAC_DATA_SEL_LB) ? "  -> TX1 = RX1, TX2 = RX2" : "");
    printf("  AD9361 interface: %lu.%03lu MHz, ADC core status 0x%lx (bit0 = interface OK)\n",
           (unsigned long)(khz / 1000U), (unsigned long)(khz % 1000U), (unsigned long)st);
    printf("  (no sample counters / snapshots: they were inside gnss_passthrough)\n");
    printf("--------------------------------------\n");
}

/* No counters in this pipeline: the data flow is confirmed by the AD9361 interface clock
 * running and the ADC core reporting its interface OK. */
int32_t gnss_pt_check_dataflow(uint32_t delay_ms)
{
    uint32_t st = 0, k1, k2;
    k1 = if_clk_khz(&st);
    no_os_mdelay(delay_ms);
    k2 = if_clk_khz(&st);
    printf("gnss_pt: AD9361 interface clock %lu / %lu kHz, ADC status 0x%lx\n",
           (unsigned long)k1, (unsigned long)k2, (unsigned long)st);
    if (k2 == 0U || !(st & 1U)) {
        printf("gnss_pt: FAIL - no AD9361 interface clock or interface not OK\n");
        return GNSS_PT_ERR_NO_RX;
    }
    return GNSS_PT_OK;
}
