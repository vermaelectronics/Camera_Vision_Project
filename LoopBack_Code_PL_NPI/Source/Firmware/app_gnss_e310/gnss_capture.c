/******************************************************************************
 *  gnss_capture.c
 *  RX I/Q capture to DDR over the vendor DMA path. See gnss_capture.h.
 *****************************************************************************/

#include "gnss_capture.h"
#include "gnss_l1.h"
#include "no_os_delay.h"
#include <stdio.h>

#ifdef XILINX_PLATFORM
#include "xil_cache.h"
#include "xil_io.h"
#endif

/* A value the DMA will never plausibly leave everywhere. Written across the
 * buffer before the transfer so "the DMA silently did nothing" cannot be
 * mistaken for "the antenna is quiet" -- the two look identical in a file of
 * zeros, and that is exactly the sort of confusion that cost us three days on
 * ISSUE-0003. */
#define FILL_PATTERN  0xDEADBEEFU

static void fill_buffer(uint32_t addr, uint32_t bytes)
{
#ifdef XILINX_PLATFORM
    uint32_t i;
    for (i = 0; i < bytes; i += 4) {
        Xil_Out32(addr + i, FILL_PATTERN);
    }
    Xil_DCacheFlushRange((uintptr_t)addr, bytes);
#else
    (void)addr; (void)bytes;
#endif
}

/* Returns non-zero if every word still reads as the fill pattern. Sampled
 * rather than exhaustive -- enough to catch a transfer that never happened. */
static int buffer_untouched(uint32_t addr, uint32_t bytes)
{
#ifdef XILINX_PLATFORM
    uint32_t i;
    uint32_t step = bytes / 64U;
    if (step < 4U) { step = 4U; }
    step &= ~3U;
    for (i = 0; i < bytes; i += step) {
        if (Xil_In32(addr + i) != FILL_PATTERN) { return 0; }
    }
    return 1;
#else
    (void)addr; (void)bytes;
    return 0;
#endif
}

int32_t gnss_capture_run(struct ad9361_rf_phy *phy,
                         struct axi_dmac *dmac,
                         uint32_t buf_addr,
                         uint32_t samples,
                         const char *label)
{
    struct axi_dma_transfer xfer;
    struct rf_rssi  rssi;
    struct rf_rx_gain gain;
    uint64_t rx_lo = 0;
    uint32_t rx_fs = 0;
    uint32_t channels, bytes;
    int32_t  ret;

    if (!phy || !dmac || samples == 0U) { return GNSS_CAP_ERR_ARG; }
    if (!phy->rx_adc) {
        printf("gnss_cap: no rx_adc; the ADC core was never initialised\n");
        return GNSS_CAP_ERR_ARG;
    }

    channels = phy->rx_adc->num_channels;
    bytes    = samples * GNSS_CAP_BYTES_PER_CHANNEL_SAMPLE * channels;

    printf("\ngnss_cap: capturing '%s'\n", label ? label : "(unnamed)");
    printf("          buffer 0x%08lx, %lu bytes, %lu sample instants, %lu channels\n",
           (unsigned long)buf_addr, (unsigned long)bytes,
           (unsigned long)samples, (unsigned long)channels);

    fill_buffer(buf_addr, bytes);

    xfer.size          = bytes;
    xfer.transfer_done = 0;
    xfer.cyclic        = NO;
    xfer.src_addr      = 0;              /* streaming source: the ADC path */
    xfer.dest_addr     = (uintptr_t)buf_addr;

    ret = axi_dmac_transfer_start(dmac, &xfer);
    if (ret != 0) {
        printf("gnss_cap: axi_dmac_transfer_start failed (%ld)\n", (long)ret);
        printf("CAPTURE_RESULT: FAIL\n");
        return GNSS_CAP_ERR_DMA;
    }

    /* At 30.72 MSPS an 8 MiB capture takes ~34 ms. Allow generously more so a
     * slow sample rate does not look like a hang. */
    ret = axi_dmac_transfer_wait_completion(dmac, 5000);
    if (ret != 0) {
        printf("gnss_cap: DMA did not complete (%ld)\n", (long)ret);
        printf("gnss_cap: is the AD9361 RX actually delivering samples? check\n"
               "          'adc enable i/q' and the RX counters.\n");
        printf("CAPTURE_RESULT: FAIL\n");
        return GNSS_CAP_ERR_DMA;
    }

#ifdef XILINX_PLATFORM
    /* The DMA wrote through the HP port, behind the CPU's back. Invalidate so
     * anything the CPU reads next is the captured data and not a stale line,
     * and so no dirty line is later evicted over the top of it. */
    Xil_DCacheInvalidateRange((uintptr_t)buf_addr, bytes);
#endif

    if (buffer_untouched(buf_addr, bytes)) {
        printf("gnss_cap: buffer still reads as the 0x%08X fill pattern --\n"
               "          the DMA reported success but moved nothing.\n",
               FILL_PATTERN);
        printf("CAPTURE_RESULT: FAIL\n");
        return GNSS_CAP_ERR_EMPTY;
    }

    /* ---- manifest -------------------------------------------------------
     * Everything needed to interpret the dumped file later, printed next to
     * it in the console log. A capture whose settings you cannot reconstruct
     * is not evidence. */
    (void)ad9361_get_rx_lo_freq(phy, &rx_lo);
    (void)ad9361_get_rx_sampling_freq(phy, &rx_fs);

    printf("gnss_cap: MANIFEST '%s'\n", label ? label : "(unnamed)");
    printf("          CAPTURE_ADDR   : 0x%08lx\n", (unsigned long)buf_addr);
    printf("          CAPTURE_BYTES  : %lu\n",     (unsigned long)bytes);
    printf("          CAPTURE_SAMPLES: %lu\n",     (unsigned long)samples);
    printf("          CAPTURE_CHANS  : %lu (I0,Q0,I1,Q1 order, 16-bit each)\n",
           (unsigned long)channels);
    printf("          CAPTURE_FORMAT : int16 LE, 12-bit right-aligned, sign-extended\n");
    printf("          CAPTURE_RX_LO  : %llu Hz\n", (unsigned long long)rx_lo);
    printf("          CAPTURE_FS     : %lu Hz\n",  (unsigned long)rx_fs);
    if (ad9361_get_rx_rf_gain(phy, 0, &gain.gain_db) == 0) {
        printf("          CAPTURE_RXGAIN : %ld dB\n", (long)gain.gain_db);
    }
    if (ad9361_get_rx_rssi(phy, 0, &rssi) == 0 && rssi.multiplier > 0) {
        printf("          CAPTURE_RSSI   : %lu.%02lu dB\n",
               (unsigned long)(rssi.symbol / (uint32_t)rssi.multiplier),
               (unsigned long)(rssi.symbol % (uint32_t)rssi.multiplier));
    }
    printf("CAPTURE_RESULT: PASS\n");
    return GNSS_CAP_OK;
}

int32_t gnss_capture_both_polarities(struct ad9361_rf_phy *phy,
                                     struct axi_dmac *dmac)
{
    int32_t ra, rb;

    if (!phy || !dmac) { return GNSS_CAP_ERR_ARG; }

    printf("\n=== Dual-polarity I/Q capture ===\n");
    printf("Receive only. Nothing is transmitted.\n");
    printf("Two captures, one per candidate band-select polarity. Correlate them\n"
           "offline: GPS L1 is ~20 dB under the noise floor, so only despreading\n"
           "can tell the two apart -- RSSI provably cannot.\n");

    /* Candidate A: the vendor firmware polarity, our current default. */
    gnss_l1_set_band_polarity(phy, 0, 1);
    no_os_mdelay(200);                    /* switch settle + AGC re-converge */
    ra = gnss_capture_run(phy, dmac, GNSS_CAP_BUF_A, GNSS_CAP_SAMPLES,
                          "A: h=0 l=1 vendor firmware");

    /* Candidate B: what the ES2 schematic's printed truth table says. */
    gnss_l1_set_band_polarity(phy, 1, 0);
    no_os_mdelay(200);
    rb = gnss_capture_run(phy, dmac, GNSS_CAP_BUF_B, GNSS_CAP_SAMPLES,
                          "B: h=1 l=0 ES2 schematic");

    /* Leave the board in the configured default whatever happened. */
    gnss_l1_set_band_polarity(phy,
                              GNSS_L1_BAND_LOW_CTRL_H, GNSS_L1_BAND_LOW_CTRL_L);

    printf("\ngnss_cap: dump both with Automation\\PowerShell\\Dump-Capture.ps1\n");
    printf("          A = 0x%08lx, B = 0x%08lx\n",
           (unsigned long)GNSS_CAP_BUF_A, (unsigned long)GNSS_CAP_BUF_B);
    printf("          restored default polarity h=%d l=%d\n",
           (int)GNSS_L1_BAND_LOW_CTRL_H, (int)GNSS_L1_BAND_LOW_CTRL_L);
    printf("===========================================\n");

    return (ra == GNSS_CAP_OK && rb == GNSS_CAP_OK) ? GNSS_CAP_OK : GNSS_CAP_ERR_DMA;
}
