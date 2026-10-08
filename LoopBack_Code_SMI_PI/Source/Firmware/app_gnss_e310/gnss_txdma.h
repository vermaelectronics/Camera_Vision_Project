/******************************************************************************
 *  gnss_txdma.h
 *  The DDR round trip:  RX -> DMA -> DDR -> DMA -> axi_ad9361 -> TX1
 *
 *  Original work for the ANTSDR E310 V1 GNSS CRPA project.
 *
 *  WHAT THIS COMPLETES
 *    Three IP cores sit in the running bitstream, correctly parameterised and
 *    wired, and had never moved a single real sample before this module:
 *
 *        axi_ad9361_dac_dma   axi_dmac, SRC=memory DEST=stream, CYCLIC=1,
 *                             ctrl 0x7C42_0000, IRQ ps-12
 *        util_ad9361_dac_upack  util_upack2, inverse of the RX channel packer
 *        axi_ad9361_dac_fifo    util_rfifo, DATA path (its enable/valid strobes
 *                               were always live -- they drive the AD9361 DAC
 *                               request timing even in passthrough mode)
 *
 *    TEST-025 (2026-09-15) proved RX -> PL -> TX with pass_en = 1, which routes
 *    LIVE RX samples and ignores dma_dac_data_*. This module exercises the other
 *    branch of that mux.
 *
 *  THE ALIGNMENT TRAP -- read this before changing anything
 *    Source/HDL/gnss_passthrough.v:452-455
 *
 *        assign dac_data_i0 = mute ? 16'd0 : (pass_en ? al_i0 : dma_dac_data_i0);
 *
 *    `al_*` is the ISSUE-0004 alignment stage. It applies ONLY to the
 *    passthrough branch, because util_rfifo data is already left-aligned and
 *    shifting it again would be wrong.
 *
 *    But a buffer captured by gnss_capture.c holds RX-FORMAT samples: 12-bit
 *    RIGHT-aligned, sign-extended into [15:12]. Replaying those down the DMA
 *    branch hands the DAC sample>>4 -- 24 dB low, four LSBs gone. Exactly the
 *    ISSUE-0004 defect, in the one path the HDL fix deliberately does not cover.
 *
 *    So gnss_txdma_convert_rx_to_tx() left-shifts by 4 in place. It is not
 *    optional and it is not a workaround; it is the format conversion the HDL
 *    cannot do for this branch.
 *
 *  WHAT A CYCLIC REPLAY CAN AND CANNOT ACHIEVE
 *    The buffer is replayed forever by the DMA with no CPU involvement. For a
 *    GNSS receiver downstream that means:
 *
 *      - ACQUISITION AND TRACKING SHOULD WORK. The C/A code repeats every 1 ms,
 *        so if the buffer is an exact whole number of milliseconds the code
 *        phase is CONTINUOUS across the loop seam and correlation is unbroken.
 *        gnss_txdma_samples_for_ms() enforces that.
 *
 *      - A POSITION FIX SHOULD NOT BE EXPECTED. The 50 bps navigation message
 *        carries ephemeris and time-of-week and takes 30 s for one subframe set.
 *        A buffer of tens of milliseconds replays the same fragment forever, so
 *        the receiver can never decode a consistent TOW and cannot solve for
 *        position. If a fix DOES appear it is almost certainly the receiver
 *        coasting on state it acquired earlier -- treat it with suspicion.
 *
 *    THEREFORE the pass criterion for this module is SATELLITES ACQUIRED WITH
 *    C/N0 > 0 at the external receiver, not a fix. Absence of a fix here does
 *    NOT indicate a broken datapath.
 *****************************************************************************/
#ifndef GNSS_TXDMA_H_
#define GNSS_TXDMA_H_

#include <stdint.h>
#include "ad9361_api.h"
#include "axi_dmac.h"

/* --------------------------------------------------------------------------
 *  DDR buffer.
 *
 *  Placed at 272 MB, clear of the application ELF (~0x0010_0000..0x0014_A0FF)
 *  and clear of the two 8 MiB capture buffers at 0x1000_0000 and 0x1080_0000,
 *  so a round-trip test does not destroy captures that have not been dumped.
 *  32-byte aligned, which Xil_DCache*Range requires.
 * -------------------------------------------------------------------------- */
#define GNSS_TXDMA_BUF        0x11000000U

/* Replay length in whole milliseconds.
 *
 * 20 ms is chosen deliberately: it is a whole number of C/A code periods (1 ms)
 * AND a whole number of navigation bit periods (20 ms), so neither the code
 * phase nor the bit boundary steps discontinuously at the loop seam. At
 * 30.72 MSPS 2R2T that is 614400 sample instants = 4915200 bytes, which fits
 * inside the 8 MiB stride with room to spare. */
#define GNSS_TXDMA_MS         20U

#define GNSS_TXDMA_BYTES_PER_CHANNEL_SAMPLE  2U

/* Return codes */
#define GNSS_TXDMA_OK          0
#define GNSS_TXDMA_ERR_ARG    -1
#define GNSS_TXDMA_ERR_DMA    -2
#define GNSS_TXDMA_ERR_RATE   -3   /* sample rate not a whole number per ms   */
#define GNSS_TXDMA_ERR_EMPTY  -4   /* capture produced nothing                */

/* Sample instants for `ms` milliseconds at the CURRENT RX sample rate.
 * Returns 0 and prints why if the rate is not a whole number of samples per
 * millisecond -- in that case a cyclic replay would step the code phase at the
 * seam and the downstream receiver would lose lock once per loop. */
uint32_t gnss_txdma_samples_for_ms(struct ad9361_rf_phy *phy, uint32_t ms);

/* Convert a captured RX-format buffer to TX format IN PLACE: each 16-bit
 * channel value is left-shifted by 4. See "THE ALIGNMENT TRAP" above. */
void gnss_txdma_convert_rx_to_tx(uint32_t buf_addr, uint32_t bytes);

/* Capture live RX into GNSS_TXDMA_BUF, convert it to TX format, and start a
 * CYCLIC DMA replay of it into axi_ad9361. Sets pass_en = 0 so gnss_passthrough
 * selects dma_dac_data_*, and confirms the AD9361 DAC actually accepts the
 * samples (dac_enable i/q must read 1/1).
 *
 * TRANSMITS. The caller is responsible for TX attenuation; the console wrapper
 * forces maximum attenuation before calling this. */
int32_t gnss_txdma_start(struct ad9361_rf_phy *phy,
                         struct axi_dmac *rx_dmac,
                         struct axi_dmac *tx_dmac);

/* Stop the cyclic replay and silence TX. Safe to call when not running. */
int32_t gnss_txdma_stop(struct ad9361_rf_phy *phy, struct axi_dmac *tx_dmac);

/* Non-zero while a cyclic replay is running. */
int gnss_txdma_is_running(void);

#endif /* GNSS_TXDMA_H_ */
