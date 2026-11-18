/******************************************************************************
 *  gnss_boot_config.h  --  what the board does by itself after power-up.
 *
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

/* gnss_tx=1 (at boot and later) goes to this TX1 attenuation */
#ifndef GNSS_TX_START_ATTEN_MDB
#define GNSS_TX_START_ATTEN_MDB    GNSS_BOOT_TX1_ATTEN_MDB
#endif

#endif
