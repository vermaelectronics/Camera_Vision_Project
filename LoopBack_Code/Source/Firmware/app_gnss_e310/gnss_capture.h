/******************************************************************************
 *  gnss_capture.h
 *  RX I/Q capture to DDR over the vendor DMA path.
 *
 *  Original work for the ANTSDR E310 V1 GNSS CRPA project.
 *
 *  WHAT THIS IS FOR
 *    The product datapath is RX -> PL -> TX entirely in programmable logic and
 *    this module is NOT part of it. This is a MEASUREMENT INSTRUMENT: it parks
 *    a block of raw I/Q in DDR so it can be pulled off the board over JTAG and
 *    analysed on a PC.
 *
 *    The RX tap into gnss_passthrough is a FAN-OUT (MOD-2), so the vendor DMA
 *    path still sees exactly the samples the custom block sees. Capture
 *    therefore runs WHILE the PL passthrough runs, and is a genuine
 *    cross-check rather than a different mode.
 *
 *  WHY IT MATTERS RIGHT NOW
 *    ISSUE-0008: the band-select GPIO polarity is unresolved and the RSSI
 *    register cannot settle it, because GPS L1 sits ~20 dB BELOW the noise
 *    floor and no power measurement can see it. Captured I/Q can be correlated
 *    offline, which recovers ~30 dB of despreading gain. If satellites acquire
 *    under one polarity and not the other, the question is answered.
 *    gnss_capture_both_polarities() takes one capture under each.
 *
 *  DATA FORMAT
 *    util_cpack2 packs the enabled ADC channels, 16 bits each, in channel
 *    order I0, Q0, I1, Q1. So one "sample instant" is
 *        2 bytes * rx_adc->num_channels          (8 bytes for 2R2T)
 *    Each 16-bit value is a 12-bit sample RIGHT-aligned and sign-extended into
 *    [15:12] -- the RX format described in Docs/Architecture/RF_BAND_SELECT.md.
 *    Little-endian on this platform.
 *****************************************************************************/
#ifndef GNSS_CAPTURE_H_
#define GNSS_CAPTURE_H_

#include <stdint.h>
#include "ad9361_api.h"
#include "axi_dmac.h"
#include "axi_adc_core.h"   /* struct axi_adc -- needed for rx_adc->num_channels */

/* --------------------------------------------------------------------------
 *  DDR buffers.
 *
 *  The application ELF occupies roughly 0x0010_0000 .. 0x0014_A0FF. These sit
 *  far above it, at 256 MB and 264 MB into the 1 GB DDR, so a buffer overrun
 *  cannot reach code, and nothing the BSP allocates lands here either.
 *  Both are 32-byte cache-line aligned, which Xil_DCache*Range requires.
 * -------------------------------------------------------------------------- */
#define GNSS_CAP_BUF_A        0x10000000U   /* polarity h=0 l=1 (vendor)      */
#define GNSS_CAP_BUF_B        0x10800000U   /* polarity h=1 l=0 (ES2 sheet)   */
#define GNSS_CAP_BUF_STRIDE   0x00800000U   /* 8 MiB apart                    */

/* 1 Mi sample instants = 8 MiB at 2R2T. At 30.72 MSPS that is ~34 ms, which is
 * comfortably more than the few ms a GPS L1 C/A acquisition needs, and small
 * enough that a JTAG dump finishes in a couple of minutes. */
#define GNSS_CAP_SAMPLES      1048576U

#define GNSS_CAP_BYTES_PER_CHANNEL_SAMPLE  2U

/* Return codes */
#define GNSS_CAP_OK            0
#define GNSS_CAP_ERR_ARG      -1
#define GNSS_CAP_ERR_DMA      -2
#define GNSS_CAP_ERR_EMPTY    -3   /* buffer still reads as the fill pattern */

/* One capture into `buf_addr`. `label` is echoed into the console manifest so
 * a log can be matched to a dumped file. Prints CAPTURE_RESULT: PASS/FAIL. */
int32_t gnss_capture_run(struct ad9361_rf_phy *phy,
                         struct axi_dmac *dmac,
                         uint32_t buf_addr,
                         uint32_t samples,
                         const char *label);

/* ISSUE-0008 resolver: capture once under each candidate band-select polarity,
 * into GNSS_CAP_BUF_A and GNSS_CAP_BUF_B, then restore the configured default.
 * Receive-only; transmits nothing. */
int32_t gnss_capture_both_polarities(struct ad9361_rf_phy *phy,
                                     struct axi_dmac *dmac);

#endif /* GNSS_CAPTURE_H_ */
