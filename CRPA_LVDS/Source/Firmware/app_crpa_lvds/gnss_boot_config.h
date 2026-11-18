/******************************************************************************
 *  gnss_boot_config.h  --  what the board does by itself after power-up.
 *
 *  At power-up the radio is set to RF profile 1 (gnss_band=1): LO, sample rate
 *  and RF bandwidth below, RX and TX the same; the radio guard puts them back
 *  within 1 s if anything else changes them.
 *  With GNSS_BOOT_AUTOSTART = 1 the firmware, once the radio is up, without
 *  any console command:
 *    1. locks BOTH RX gains to GNSS_BOOT_RX_GAIN_DB (manual mode: no AGC, and
 *       the anti-jam engine keeps a manual gain as it is);
 *    2. sets TX2 attenuation to GNSS_BOOT_TX2_ATTEN_MDB;
 *    3. runs gnss_tx=1: TX1 attenuation to the maximum first, conducted-path
 *       interlock, then GNSS_TX_START_ATTEN_MDB (= GNSS_BOOT_TX1_ATTEN_MDB),
 *       RX -> TX passthrough on, DAC source = our samples.
 *  gnss_tx=0 / tx1_attenuation= / tx2_attenuation= / gnss_rx_gain= /
 *  gnss_rx_agc= still change everything afterwards. A later gnss_tx=1 goes to
 *  GNSS_TX_START_ATTEN_MDB as well.
 *
 *  SAFETY: TX1 transmits on the GNSS frequency from power-up, with no operator.
 *  Conducted, attenuated coax only. At 30 dB attenuation TX1 puts out about
 *  -23 dBm (vs. about -63 dBm at the 70 dB measured working level): use enough
 *  external attenuation in front of the GNSS receiver.
 *
 *  Larger attenuation = quieter. Units: mdB (30000 = 30 dB).
 *  Set GNSS_BOOT_AUTOSTART to 0 for the old behaviour (RF-silent until gnss_tx=1).
 *****************************************************************************/
#ifndef GNSS_BOOT_CONFIG_H_
#define GNSS_BOOT_CONFIG_H_

#ifndef GNSS_BOOT_AUTOSTART
#define GNSS_BOOT_AUTOSTART        1
#endif
#ifndef GNSS_BOOT_TX1_ATTEN_MDB
#define GNSS_BOOT_TX1_ATTEN_MDB    30000U   /* TX1: 30 dB                      */
#endif
#ifndef GNSS_BOOT_TX2_ATTEN_MDB
#define GNSS_BOOT_TX2_ATTEN_MDB    89450U   /* TX2: 89.45 dB                   */
#endif
#ifndef GNSS_BOOT_RX_GAIN_DB
#define GNSS_BOOT_RX_GAIN_DB       40       /* RX1 and RX2: 40 dB, manual      */
#endif

/* RF profile 1 (gnss_band=1, applied at power-up and kept by the 1 s radio
 * guard): RX and TX LO, sample rate and RF bandwidth, the same on both sides.
 * 1568.5 MHz puts GPS L1 / Galileo E1 at +6.92 MHz and BeiDou B1I at -7.40 MHz
 * from the LO, inside the flat part of 30.72 MS/s / 20 MHz, and keeps the
 * AD9361's LO leakage off the L1 centre. GLONASS G1 is outside this profile. */
#ifndef GNSS_BOOT_LO_HZ
#define GNSS_BOOT_LO_HZ            1568500000ULL   /* 1568.5 MHz, RX and TX   */
#endif
#ifndef GNSS_BOOT_FS_HZ
#define GNSS_BOOT_FS_HZ            30720000U       /* 30.72 MS/s, RX and TX   */
#endif
#ifndef GNSS_BOOT_BW_HZ
#define GNSS_BOOT_BW_HZ            20000000U       /* 20 MHz, RX and TX       */
#endif

/* After gnss_tx=1 at power-up: measure the RX and TX sample rates of the
 * passthrough (gnss_fifo?) and, if TX requests samples at a different rate
 * than RX delivers them, set the DAC rate control so they match, then check
 * again (up to 3 times). Matched, the RX -> TX buffers sit about half full
 * (16 of 32) instead of empty, and no sample is repeated or lost. */
#ifndef GNSS_BOOT_FIFO_AUTOTUNE
#define GNSS_BOOT_FIFO_AUTOTUNE    1
#endif

/* gnss_tx=1 (at boot and later) goes to this TX1 attenuation */
#ifndef GNSS_TX_START_ATTEN_MDB
#define GNSS_TX_START_ATTEN_MDB    GNSS_BOOT_TX1_ATTEN_MDB
#endif

#endif
