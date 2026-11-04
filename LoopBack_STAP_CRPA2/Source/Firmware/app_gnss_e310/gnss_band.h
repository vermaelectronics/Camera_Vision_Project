/******************************************************************************
 *  gnss_band.h -- five preset GNSS bands, selected with  gnss_band=N
 *  ANTSDR E310 V1 GNSS CRPA pipeline (LoopBack_STAP_CRPA2).
 *
 *  One command sets, together and in a safe order:
 *    RX + TX centre frequency (always the SAME, so the loopback comes out on
 *    the frequency it went in on), sample rate (RX = TX), RX and TX RF
 *    bandwidth, RX1 and RX2 gain (manual, the same on both CRPA elements),
 *    TX1 attenuation.
 *
 *  SD-card build: the band GNSS_DEFAULT_BAND is applied at power-on, then the
 *  CRPA anti-jam is switched on with background weight updates. Choose the
 *  power-on band at build time:
 *      ./Automation/Linux/build_bin.sh --band 2
 *  or at run time on the console:  gnss_band=2
 *
 *  EDIT THE TABLE in gnss_band.c (gnss_bands[]) to change a band, then
 *  rebuild the firmware only:   ./Automation/Linux/build_bin.sh --from sw
 *****************************************************************************/
#ifndef GNSS_BAND_H_
#define GNSS_BAND_H_

#include <stdint.h>
#include "app_config.h"

#define GNSS_BAND_COUNT          5

/* band applied at power-on by the SD-card (deploy) build */
#ifndef GNSS_DEFAULT_BAND
#define GNSS_DEFAULT_BAND        1
#endif

/* Limits of THIS bitstream / board, checked before anything is applied. */
#define GNSS_BAND_MAX_FS_HZ      30720000U   /* 2R2T LVDS: DATA_CLK = 4 x fs,
                                                FPGA rx_clk timed at 125 MHz   */
#define GNSS_BAND_MIN_FS_HZ      2083334U    /* AD9361 without FIR decimation  */
#define GNSS_BAND_MIN_LO_HZ      1100000000ULL  /* L band: the RF band switches */
#define GNSS_BAND_MAX_LO_HZ      1700000000ULL  /* set at boot stay valid       */
#define GNSS_BAND_MIN_ATT_MDB    10000U      /* 10 dB: loudest a band may use  */
#define GNSS_BAND_MAX_ATT_MDB    89750U

/* gain control, same values as the vendor rx1_gc_mode= command */
#define GNSS_GC_MANUAL           0
#define GNSS_GC_FAST_AGC         1
#define GNSS_GC_SLOW_AGC         2
#define GNSS_GC_HYBRID_AGC       3

typedef struct {
    const char *name;
    uint64_t    lo_hz;        /* RX and TX LO                            */
    uint32_t    fs_hz;        /* RX and TX sample rate                   */
    uint32_t    rx_bw_hz;     /* RX RF bandwidth                         */
    uint32_t    tx_bw_hz;     /* TX RF bandwidth                         */
    uint8_t     gc_mode;      /* GNSS_GC_* for RX1 and RX2               */
    int32_t     rx_gain_db;   /* RX1 = RX2 gain, used when gc_mode = 0   */
    uint32_t    tx_att_mdb;   /* TX1 attenuation, bigger = quieter       */
} gnss_band_t;

extern const gnss_band_t gnss_bands[GNSS_BAND_COUNT];

/* Apply band 1..GNSS_BAND_COUNT. Returns 0 on success. */
int32_t gnss_band_apply(int band);
/* 0 = none selected since boot (boot configuration in effect) */
int     gnss_band_current(void);

/* TX -> RX leakage check: RX1/RX2 input power with the DAC muted and with it
 * running. A same-frequency repeater on one board feeds its own output back
 * into its receivers if the TX level is too high for the board's TX-RX
 * isolation. Returns the rise in centi-dB of the larger channel, or a
 * negative value if it could not be measured. */
int32_t gnss_band_loop_check(int verbose);

/* SD-card start-up: band, TX, leakage check, CRPA on, background updates */
int32_t gnss_band_autostart(void);

/* console commands */
void get_gnss_band(double *param, char param_no);
void set_gnss_band(double *param, char param_no);
void get_gnss_loop(double *param, char param_no);

#endif /* GNSS_BAND_H_ */
