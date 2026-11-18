/***************************************************************************//**
 * @file   gnss_config.h
 * @brief  GNSS-CRPA MOD-15: ONE place for the radio plan and the boot
 *         behaviour. command.c, gnss_autostart.c, gnss_info.c and main.c all
 *         take their numbers from here. Change a value, rebuild, done.
 *
 * Every value can also be overridden on the compiler command line
 * (-DGNSS_CFG_TX1_ATTEN_MDB=60000 etc.) without editing this file.
 ******************************************************************************/
#ifndef GNSS_CONFIG_H_
#define GNSS_CONFIG_H_

/* ---- radio plan (RX1, RX2 and TX1 all use the same) ------------------------
 * LO 1575.42 MHz: GPS L1 / Galileo E1 sit at 0 Hz (baseband DC).
 *   40 MHz RF bandwidth = +/-20 MHz: covers GPS L1 C/A + L1C, Galileo E1 and
 *   BeiDou B1I (1561.098 MHz = -14.3 MHz). GLONASS L1 (1598..1606 MHz,
 *   +22.6..+30.6 MHz) is OUTSIDE this band.
 *   THINGS TO KNOW ABOUT LO = L1 (not a bug, a property of zero-IF):
 *   - The AD9361 TX LO leakage lands exactly on L1 as a CW tone. Its TX
 *     quadrature calibration suppresses it, but whatever is left is a CW
 *     tone in the middle of the C/A main lobe. If C/N0 is worse than with
 *     the old 1584.5 MHz plan, this is the first suspect: set
 *     GNSS_CFG_LO_HZ to 1584500000 (GPS then sits at -9.08 MHz) to compare.
 *   - RX DC-offset tracking removes a few kHz around 0 Hz: < 0.1 dB of
 *     C/A power, harmless.
 * 45 MSPS: l_clk = 4 x 45 = 180 MHz, well inside the 250 MHz the FPGA
 *   timing (block v1.12) is closed at. Hold/glide = 0.37 s, jammer
 *   estimate every 1.46 ms at this rate. */
#ifndef GNSS_CFG_LO_HZ
#define GNSS_CFG_LO_HZ              1575420000ULL
#endif
#ifndef GNSS_CFG_RF_BW_HZ
#define GNSS_CFG_RF_BW_HZ           40000000UL
#endif
#ifndef GNSS_CFG_SAMP_RATE_HZ
#define GNSS_CFG_SAMP_RATE_HZ       45000000UL
#endif

/* ---- receive gain ------------------------------------------------------------
 * MANUAL (gc_mode 0) and the SAME on RX1 and RX2: a requirement of MVDR (the
 * covariance assumes both channels keep one gain for the whole window), not
 * a style choice. AGC is never used.
 * GNSS_CFG_RX_GAIN_START_DB is where the boot auto-level starts; it then
 * steps the gain until the noise floor reads GNSS_CFG_RX_NF_TARGET_DBFS:
 *   -60 dBFS leaves ~57 dB above the noise for a jammer before the 12-bit
 *   ADC clips, for < 0.35 dB loss on the satellites. */
#ifndef GNSS_CFG_RX_GC_MODE
#define GNSS_CFG_RX_GC_MODE         0           /* manual */
#endif
#ifndef GNSS_CFG_RX_GAIN_START_DB
#define GNSS_CFG_RX_GAIN_START_DB   50
#endif
#ifndef GNSS_CFG_RX_GAIN_MIN_DB
#define GNSS_CFG_RX_GAIN_MIN_DB     0
#endif
#ifndef GNSS_CFG_RX_GAIN_MAX_DB
#define GNSS_CFG_RX_GAIN_MAX_DB     71          /* AD9361 gain table, 1.3-4 GHz */
#endif
#ifndef GNSS_CFG_RX_NF_TARGET_DBFS
#define GNSS_CFG_RX_NF_TARGET_DBFS  (-60)
#endif
/* When the AD9361's own noise floor is too high for the target above (seen on
 * this board: about -48 dBFS at any gain, at 45 MSPS / 40 MHz / LO = L1), the
 * antenna noise is levelled this many dB ABOVE that floor instead: 15 dB
 * costs the satellites ~0.14 dB. The price is less ADC headroom for a jammer,
 * which the clip guard (below) takes care of. */
#ifndef GNSS_CFG_RX_FLOOR_MARGIN_DB
#define GNSS_CFG_RX_FLOOR_MARGIN_DB 15
#endif
#ifndef GNSS_CFG_RX_NF_TOL_DB
#define GNSS_CFG_RX_NF_TOL_DB       3           /* on target within +/- this */
#endif

/* ---- output level and transmit ---------------------------------------------
 * AOLC holds the MVDR output noise floor at -GNSS_CFG_AOLC_TARGET_DB dBFS,
 * whatever the RX gain and whatever the jammer does.
 * TX1 attenuation (mdB, BIGGER = QUIETER): with the AD9361's nominal +7 dBm
 * full scale, the noise leaving TX1 is about
 *       7 - AOLC_TARGET - ATTEN/1000  dBm   = -51 dBm at 34000 mdB.
 * A GNSS receiver loses no C/N0 when that noise, at ITS input, is >= 10 dB
 * above its own noise floor (-95 dBm in 40 MHz for a 3 dB NF receiver) and
 * is not so loud it compresses. 40000 mdB is the value found best on the
 * bench (with the cabling / attenuators there); 64000 was the first,
 * theoretical guess. With AOLC on, this does NOT need changing when a
 * jammer comes or goes: the output noise level is held constant.
 * Peak (full-scale) output at 40000 is -33 dBm. */
#ifndef GNSS_CFG_AOLC_TARGET_DB
#define GNSS_CFG_AOLC_TARGET_DB     24          /* MOD-20: was 18. -24 dBFS costs no
                                                   C/N0 and leaves 6 dB more room for
                                                   jammer residue / level bursts (the
                                                   field log clipped 10k..125k output
                                                   samples a second at -18) */
#endif
#ifndef GNSS_CFG_TX1_ATTEN_MDB
#define GNSS_CFG_TX1_ATTEN_MDB      34000U      /* MOD-20: 40000 found best on the bench
                                                   at AOLC -18 dBFS; AOLC is now -24, so
                                                   6 dB less attenuation = same level */
#endif
#define GNSS_CFG_TX_FS_DBM          7           /* AD9361 nominal, for estimates only */
#define GNSS_CFG_RX_NOISE_MIN_DBM   (-85)       /* receiver-input noise window used */
#define GNSS_CFG_RX_NOISE_MAX_DBM   (-60)       /*  by the boot check's advice      */

/* ---- boot behaviour --------------------------------------------------------
 * GNSS_AUTO_MVDR 1 : after programming, the board brings MVDR up by itself
 *                    and prints a PASS / WARN / FAIL report of every step.
 *                    TX1 opens ONLY if every critical check passed.
 *                    0 = the old behaviour: boot RF-silent, gnss_mvdr=1 by hand.
 * GNSS_AUTO_RX_GAIN 1 : the boot sequence levels the RX gain to the noise
 *                    target. 0 = use GNSS_CFG_RX_GAIN_START_DB as is.
 * GNSS_BOOT_CAPTURE 1 : run the old dual-polarity I/Q capture to DDR at boot
 *                    (diagnostic, ~seconds, flips the band-select GPIOs).
 * TX1 IS AN RF OUTPUT ON THE GPS L1 FREQUENCY. Conducted, attenuated coax
 * into the receiver only -- nothing in software can see what is attached. */
#ifndef GNSS_AUTO_MVDR
#define GNSS_AUTO_MVDR              0   /* LVDS / ant board: boot silent; stap=1 runs the automatic start */
#endif
#ifndef GNSS_AUTO_RX_GAIN
#define GNSS_AUTO_RX_GAIN           1
#endif
#ifndef GNSS_BOOT_CAPTURE
#define GNSS_BOOT_CAPTURE           0
#endif

/* GNSS_CFG_CLIP_GUARD 1 : while running, a jammer that makes the ADC clip
 *                    lowers the RX gain by itself (6 dB steps), and the gain
 *                    returns to the auto-level's value when it is gone.
 *                    Needs the polled console (Xilinx PS UART); gnss_guard=0/1. */
#ifndef GNSS_CFG_CLIP_GUARD
#define GNSS_CFG_CLIP_GUARD         1
#endif

/* GNSS_CFG_IQ_DEFAULT : RX IQ-image correction at start-up. 0 (MOD-21):
 *   a wideband jammer near the LO misleads its estimator in the field and the
 *   wrong correction adds a mirrored jammer. The AD9361's own image
 *   (~-45..-50 dBc) limits the null far less. gnss_iq=1 to use it.
 * GNSS_CFG_NULL_PROFILE : 0 = MVDR with loading x1 + null hold/glide,
 *   1 = plain power inversion (light loading, no hold/glide),
 *   2 = STAP (MOD-24, DEFAULT): space-time power inversion, 2 antennas x 5
 *       taps, adapted every sample. Robust against CW AND wideband / barrage
 *       jammers (one weight per antenna cannot null a barrage across 56 MHz:
 *       MOD-23, coherence 0.47 -> 2.8 dB). Needs the MVDR 1.13 bitstream; on
 *       an older one it runs as 0.
 *   3 = SMI-MVDR STAP (MOD-25, DEFAULT): the FPGA estimates the N x N
 *       space-time covariance (2 antennas x L taps + 2 image taps; MVDR
 *       1.14 L = 8 / N = 18, 1.14b L = 16 / N = 34), the ARM solves the
 *       loaded MVDR weights each window (gnss_smi.c). Deepest null on CW and
 *       barrage, no step size to tune. Needs MVDR 1.14; falls back to 2, 0.
 *       gnss_null=. */
#ifndef GNSS_CFG_IQ_DEFAULT
#define GNSS_CFG_IQ_DEFAULT         0
#endif
#ifndef GNSS_CFG_NULL_PROFILE
#define GNSS_CFG_NULL_PROFILE       3
#endif

/* GNSS_CFG_TX2_ATTEN_MDB : TX2 is not used (TX1 carries the GNSS output to
 * the receiver). MOD-26: held at this attenuation always -- at every radio
 * plan, checked every second by the monitor; tx2_attenuation= cannot open it. */
#ifndef GNSS_CFG_TX2_ATTEN_MDB
#define GNSS_CFG_TX2_ATTEN_MDB      89000U
#endif

/* GNSS_CFG_BOOT_BAND : radio preset used at boot (gnss_band? lists them):
 *   0 = LO 1575.42 MHz, 40 MHz, 45 MSPS
 *   1 = LO 1575.42 MHz,  4 MHz, 4.5 MSPS  (GPS L1 C/A only, narrow)
 *   2 = LO 1584.5  MHz, 51 MHz, 51 MSPS
 *   3 = LO 1582.5  MHz, 56 MHz, 61.44 MSPS (MOD-26 default: all four systems,
 *       BeiDou B1I at -21.4 MHz, GLONASS to +22.9 MHz; l_clk 245.76 MHz:
 *       needs a bitstream whose rx_clk timing closes at 4.0 ns)
 *   4 = LO 1568.5  MHz, 20 MHz, 22.5 MSPS (barrage: BeiDou B1I + GPS L1/Galileo E1)
 *   5 = LO 1577.92 MHz,  8 MHz, 10 MSPS   (barrage: GPS L1 + Galileo E1 only)
 * Presets 0..3 are in command.c (gnss_bands[]); gnss_plan= sets any other. */
/* GNSS_CFG_RX_NF_TARGET_MAX_DBFS : MOD-26b. The auto-level raises its noise
 * target above GNSS_CFG_RX_NF_TARGET_DBFS when the AD9361's own floor is high,
 * but never above this: a jammer needs headroom over the noise. */
#ifndef GNSS_CFG_RX_NF_TARGET_MAX_DBFS
#define GNSS_CFG_RX_NF_TARGET_MAX_DBFS  (-25)
#endif

#ifndef GNSS_CFG_BOOT_BAND
#define GNSS_CFG_BOOT_BAND          3     /* MOD-26: GPS, Galileo, BeiDou B1I/B1C, GLONASS */
#endif

/* ---- derived ---------------------------------------------------------------- */
#define GNSS_CFG_LCLK_HZ            (4UL * GNSS_CFG_SAMP_RATE_HZ)   /* LVDS 2R2T */

#endif /* GNSS_CONFIG_H_ */
