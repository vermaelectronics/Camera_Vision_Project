/******************************************************************************
 *  gnss_mode.h -- five preset radio modes, selected with  gnss_mode=N
 *  ANTSDR E310 V1 GNSS CRPA pipeline (LoopBack_STAP_CRPA1).
 *
 *  One command sets, together and in a safe order:
 *    RX + TX centre frequency (always the SAME, so the loopback comes out on
 *    the frequency it went in on), sample rate (RX = TX), RX and TX RF
 *    bandwidth, RX1 and RX2 gain control and gain (the same on both CRPA
 *    elements), TX1 attenuation.
 *
 *  EDIT THE TABLE in gnss_mode.c (gnss_modes[]) to change a mode, then rebuild
 *  the firmware only:   ./Automation/Linux/build_bin.sh --from sw
 *****************************************************************************/
#ifndef GNSS_MODE_H_
#define GNSS_MODE_H_

#include <stdint.h>

#define GNSS_MODE_COUNT          5

/* Limits of THIS bitstream / board, checked before anything is applied. */
#define GNSS_MODE_MAX_FS_HZ      30720000U   /* 2R2T LVDS: DATA_CLK = 4 x fs,
                                                FPGA rx_clk timed at 125 MHz   */
#define GNSS_MODE_MIN_FS_HZ      2083334U    /* AD9361 without FIR decimation  */
#define GNSS_MODE_MIN_LO_HZ      1100000000ULL  /* L band: the RF band switches */
#define GNSS_MODE_MAX_LO_HZ      1700000000ULL  /* set at boot stay valid       */
#define GNSS_MODE_MIN_ATT_MDB    40000U      /* refuse anything louder         */
#define GNSS_MODE_MAX_ATT_MDB    89750U

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
} gnss_mode_t;

extern const gnss_mode_t gnss_modes[GNSS_MODE_COUNT];

/* Apply mode 1..GNSS_MODE_COUNT. Returns 0 on success. */
int32_t gnss_mode_apply(int mode);
/* 0 = none selected since boot (boot configuration in effect) */
int     gnss_mode_current(void);

/* console commands */
void get_gnss_mode(double *param, char param_no);
void set_gnss_mode(double *param, char param_no);

#endif /* GNSS_MODE_H_ */
