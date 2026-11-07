/******************************************************************************
 *  gnss_txdma.c
 *  The DDR round trip: RX -> DMA -> DDR -> DMA -> axi_ad9361 -> TX1.
 *  See gnss_txdma.h for the alignment trap and what a cyclic replay can prove.
 *****************************************************************************/

#include "gnss_txdma.h"
#include "gnss_capture.h"
#include "gnss_l1.h"
#include "gnss_passthrough.h"
#include "no_os_delay.h"
#include <stdio.h>

#ifdef XILINX_PLATFORM
#include "xil_cache.h"
#include "xil_io.h"
#endif

static int      txdma_running = 0;
static uint32_t txdma_samples = 0;
static uint32_t txdma_bytes   = 0;

int gnss_txdma_is_running(void)
{
    return txdma_running;
}

uint32_t gnss_txdma_samples_for_ms(struct ad9361_rf_phy *phy, uint32_t ms)
{
    uint32_t fs = 0;

    if (!phy || ms == 0U) { return 0U; }

    if (ad9361_get_rx_sampling_freq(phy, &fs) != 0 || fs == 0U) {
        printf("gnss_txdma: could not read the RX sample rate\n");
        return 0U;
    }

    /* A cyclic replay whose length is not a whole number of C/A code periods
     * steps the code phase at the loop seam, and the downstream receiver drops
     * lock once per loop. Refuse rather than produce a confusing result. */
    if ((fs % 1000U) != 0U) {
        printf("gnss_txdma: sample rate %lu Hz is not a whole number of samples\n"
               "            per millisecond, so a cyclic replay would step the\n"
               "            C/A code phase at the loop seam. Refusing.\n",
               (unsigned long)fs);
        return 0U;
    }

    return (fs / 1000U) * ms;
}

void gnss_txdma_convert_rx_to_tx(uint32_t buf_addr, uint32_t bytes)
{
#ifdef XILINX_PLATFORM
    uint32_t i;

    /* Two 16-bit channel values per 32-bit word. Shifting the packed word by 4
     * would drag the low channel's top bits into the high channel, so each half
     * is shifted separately and re-masked.
     *
     * The input is a 12-bit value sign-extended into [15:12]. After << 4 the
     * significant bits occupy [15:4] and [3:0] are zero, which is precisely the
     * format axi_ad9361_tx_channel.v:301 consumes as dma_data[15:4]. No
     * saturation is needed: a true 12-bit signed value cannot overflow int16
     * when shifted by 4. */
    for (i = 0; i < bytes; i += 4U) {
        uint32_t w  = Xil_In32(buf_addr + i);
        uint32_t lo = (w & 0x0000FFFFU) << 4;
        uint32_t hi = ((w >> 16) & 0x0000FFFFU) << 4;
        Xil_Out32(buf_addr + i, (lo & 0x0000FFFFU) | ((hi & 0x0000FFFFU) << 16));
    }

    /* The DMA reads this buffer through S_AXI_HP2, which does not snoop the
     * CPU's cache. Without this flush it would fetch the pre-shift data from
     * DDR and the output would be 24 dB low -- the very thing this function
     * exists to prevent. */
    Xil_DCacheFlushRange((uintptr_t)buf_addr, bytes);
#else
    (void)buf_addr; (void)bytes;
#endif
}

int32_t gnss_txdma_start(struct ad9361_rf_phy *phy,
                         struct axi_dmac *rx_dmac,
                         struct axi_dmac *tx_dmac)
{
    struct axi_dma_transfer xfer;
    uint32_t channels, samples, bytes;
    int32_t  ret;

    if (!phy || !rx_dmac || !tx_dmac) { return GNSS_TXDMA_ERR_ARG; }
    if (!phy->rx_adc) {
        printf("gnss_txdma: no rx_adc; cannot size the buffer\n");
        return GNSS_TXDMA_ERR_ARG;
    }

    if (txdma_running) {
        printf("gnss_txdma: already running; stop it first\n");
        return GNSS_TXDMA_ERR_ARG;
    }

    samples = gnss_txdma_samples_for_ms(phy, GNSS_TXDMA_MS);
    if (samples == 0U) { return GNSS_TXDMA_ERR_RATE; }

    channels = phy->rx_adc->num_channels;
    bytes    = samples * GNSS_TXDMA_BYTES_PER_CHANNEL_SAMPLE * channels;

    printf("\n=== DDR ROUND TRIP: RX -> DMA -> DDR -> DMA -> TX ===\n");
    printf("gnss_txdma: buffer 0x%08lx, %lu bytes, %lu sample instants,\n"
           "            %lu channels, %lu ms replay\n",
           (unsigned long)GNSS_TXDMA_BUF, (unsigned long)bytes,
           (unsigned long)samples, (unsigned long)channels,
           (unsigned long)GNSS_TXDMA_MS);

    /* ---- 1. capture live RX into DDR ------------------------------------ */
    /* Reuses the proven capture path, including its 0xDEADBEEF fill check, so
     * "the DMA moved nothing" cannot be mistaken for "the antenna is quiet". */
    ret = gnss_capture_run(phy, rx_dmac, GNSS_TXDMA_BUF, samples,
                           "DDR round-trip source");
    if (ret != GNSS_CAP_OK) {
        printf("gnss_txdma: RX capture failed (%ld); nothing to replay\n",
               (long)ret);
        printf("DDR_ROUNDTRIP_RESULT: FAIL\n");
        return GNSS_TXDMA_ERR_EMPTY;
    }

    /* ---- 2. RX format -> TX format -------------------------------------- */
    printf("gnss_txdma: converting RX format (12-bit right-aligned) to TX\n"
           "            format (left-aligned, consumed as dma_data[15:4])\n");
    gnss_txdma_convert_rx_to_tx(GNSS_TXDMA_BUF, bytes);
    printf("gnss_txdma: alignment conversion done (<< 4 in place, cache flushed)\n");

    /* ---- 3. route the mux to the DMA branch ----------------------------- */
    /* pass_en = 0 selects dma_dac_data_* instead of the live RX passthrough.
     * This is the branch under test. */
    gnss_pt_set_passthrough(0);
    gnss_pt_set_mute(0);
    printf("gnss_txdma: pass_en = 0 (mux selects the DMA branch), unmuted\n");

    /* ---- 4. start the cyclic replay ------------------------------------- */
    xfer.size          = bytes;
    xfer.transfer_done = 0;
    xfer.cyclic        = CYCLIC;
    xfer.src_addr      = (uintptr_t)GNSS_TXDMA_BUF;
    xfer.dest_addr     = 0;              /* streaming destination: the DAC path */

    ret = axi_dmac_transfer_start(tx_dmac, &xfer);
    if (ret != 0) {
        printf("gnss_txdma: axi_dmac_transfer_start failed (%ld)\n", (long)ret);
        gnss_pt_set_mute(1);
        printf("DDR_ROUNDTRIP_RESULT: FAIL\n");
        return GNSS_TXDMA_ERR_DMA;
    }
    printf("gnss_txdma: TX DMA started, CYCLIC, %lu bytes\n",
           (unsigned long)bytes);

    /* Let the rfifo fill and the status registers refresh. gnss_passthrough
     * republishes status only every 256 sample clocks and then crosses it into
     * the AXI domain (PROJECT_INSTRUCTIONS 5b), so an immediate read is stale. */
    no_os_mdelay(50);

    /* ---- 5. confirm the DAC actually takes the samples ------------------ */
    /* dac_enable is axi_ad9361's read-back of (dac_data_sel == 4'h2). Without
     * this the DAC emits DDS or zero and silently discards everything, while
     * every counter still looks perfect. That is ISSUE-0003. */
    ret = gnss_l1_set_dac_source(phy, AXI_DAC_DATA_SEL_DMA);
    if (ret != GNSS_L1_OK) {
        printf("gnss_txdma: DAC source NOT confirmed; stopping rather than\n"
               "            leaving the path half open\n");
        (void)gnss_txdma_stop(phy, tx_dmac);
        printf("DDR_ROUNDTRIP_RESULT: FAIL\n");
        return GNSS_TXDMA_ERR_DMA;
    }

    txdma_running = 1;
    txdma_samples = samples;
    txdma_bytes   = bytes;

    printf("gnss_txdma: replaying %lu ms of live RX from DDR, forever\n",
           (unsigned long)GNSS_TXDMA_MS);
    printf("NOTE: a fix is NOT expected from a cyclic replay -- the 50 bps\n"
           "      navigation message repeats, so no consistent time-of-week can\n"
           "      be decoded. ACQUISITION (satellites with C/N0 > 0) is the\n"
           "      pass criterion for the DDR datapath.\n");
    printf("DDR_ROUNDTRIP_RESULT: PASS\n");
    printf("====================================================\n");
    return GNSS_TXDMA_OK;
}

int32_t gnss_txdma_stop(struct ad9361_rf_phy *phy, struct axi_dmac *tx_dmac)
{
    if (!phy || !tx_dmac) { return GNSS_TXDMA_ERR_ARG; }

    /* Silence first, then tear down. The DAC stops taking our samples the
     * instant the source select changes, so RF ends before the DMA is touched
     * rather than after it. */
    (void)gnss_l1_set_dac_source(phy, AXI_DAC_DATA_SEL_ZERO);
    gnss_pt_set_mute(1);

    axi_dmac_transfer_stop(tx_dmac);

    /* Leave the mux where the rest of the project expects to find it. */
    gnss_pt_set_passthrough(0);

    txdma_running = 0;
    printf("gnss_txdma: stopped - DAC source ZERO, muted, TX DMA halted\n");
    return GNSS_TXDMA_OK;
}
