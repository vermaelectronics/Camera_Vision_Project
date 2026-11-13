/******************************************************************************
 *  gnss_aj.c  --  SMI-PI v4.1 anti-jam engine. See gnss_aj.h.
 *****************************************************************************/

#include <stdio.h>
#include "gnss_aj.h"
#include "gnss_aj_math.h"
#ifdef GNSS_AJ_THREAD
#include <pthread.h>
#include <string.h>
#endif
#include "gnss_smi_agc.h"
#include "gnss_passthrough.h"
#include "gnss_l1.h"          /* 4.9.13: GNSS_L1_TX2_ATTEN_MDB */
#include "gnss_txdma.h"
#include "gnss_log.h"
#include "ad9361_api.h"
#include "axi_dmac.h"
#include "console.h"
#include "xtime_l.h"
#include "xil_cache.h"

extern struct ad9361_rf_phy *ad9361_phy;
extern struct axi_dmac *rx_dmac;

#define AJ_BYTES          (AJ_NSAMP * 8U)        /* 2 channels x 4 bytes        */
#ifdef GNSS_LINUX
/* gnss_crpa: the capture lands in the DDR shadow buffer (lx_io.c) */
#define GNSS_AJ_BUF_PTR   ((const uint32_t *)lx_ddr(GNSS_AJ_BUF_ADDR, AJ_BYTES))
#else
#define GNSS_AJ_BUF_PTR   ((const uint32_t *)GNSS_AJ_BUF_ADDR)
#endif
#define AJ_MIN_PERIOD_US  5000U                  /* 4.9.9: at most 200 captures/s (was 500): less DDR traffic and ARM load, less power */
#define AJ_DMA_TIMEOUT_US 50000U

/* CLEAR -> JAMMED: any one of these, AJ_ON_COUNT captures in a row */
#define AJ_ON_COH_DB      (-3.0)  /* RX1/RX2 coherence J/N (noise alone: ~-15)  */
#define AJ_ON_DB          6.0     /* power J/N over the learned noise floor     */
#define AJ_SETTLE_US      2000000U /* gain loop settling after start: a cut is not a jammer */
#define AJ_CUT_DB         3       /* RX gain cut by the matched gain loop       */
#define AJ_ON_COUNT       2
/* JAMMED -> CLEAR (bypass): all of these for AJ_OFF_US */
#define AJ_OFF_COH_DB     (-6.0)
#define AJ_OFF_DB         3.0
#define AJ_OFF_US         500000U
/* learning (noise floor, RX2/RX1 ratio): only this quiet */
#define AJ_QUIET_COH_DB   (-9.0)
#define AJ_NF_RISE_DB_S   0.05    /* learned noise floor may creep up this fast */
#define AJ_G2_ALPHA       0.02    /* RX1/RX2 ratio average, ~50 captures         */
#define AJ_G2_MAX         (32767.0 / 8192.0)      /* AJ_NORM is Q3.13           */
#define AJ_G2_MIN         0.125
/* output make-up gain, status */
#define AJ_NULL_RES_DB    3.0     /* NULLED when the residual first drops below */
#define AJ_WARN_RES_DB    3.0     /* residual above this while jammed ...        */
#define AJ_WARN_US        1000000U /* ... for this long: warn                    */
#define AJ_ANNOUNCE_US    100000U /* JAMMER ON is printed this long after onset,  */
                                  /* when the gain loop has absorbed the jammer: */
                                  /* the first captures can still be clipped     */
#define AJ_NEV            16      /* event history for gnss_report?             */

enum { ST_CLEAR = 0, ST_JAMMED = 1 };
enum { EV_ON = 1, EV_NULLED, EV_OFF, EV_LEAK, EV_TX, EV_START, EV_BG, EV_FLOOR };

/* ---- state --------------------------------------------------------------- */
static int      aj_on, aj_user_off, aj_frozen, aj_mode = AJ_MODE_STAP;
static aj_sweep_t sweep;            /* slow-chirp detector (4.6)                       */
static int      sweep_en = 1;       /* gnss_aj_sweep=0 turns the long memory off        */
static double   sweep_avg = 0.0;    /* r->avg for the next solve                        */
static int      aj_log_s = 0, aj_csv_ms = 0, csv_hdr;
/* Periodic lines (status, CSV) are held back while a command is being typed
 * and for this long after the last key, so they never run through it. */
#define AJ_KEY_QUIET_US   20000000U
static unsigned int keys_seen;
static XTime    t_key;
static int      dma_busy;
static XTime    dma_t0, next_start, t_rate, t_log, t_csv, t_warn, t_learn, t_boot;
static uint32_t gain_mark, gain_seen;
static aj_result_t res;
static int      res_valid, res_solved;

static uint32_t n_upd, n_det, n_fail, n_skip, cnt_upd, cnt_det, upd_rate, det_rate;
static int      sh_max = 4;        /* largest make-up shift the bitstream has (layout 4: 7, older: 4) */
static int      ws_hw = 1;         /* the bitstream has the weight scale (layout >= 3) */
static int      state, on_cnt, on_why, jam_announced, jam_nulled, shift, on_ev;
static XTime    jam_t0, quiet_t0, res_bad_t0, clear_t0, trig_t0;
static double   jam_peak, null_ms = -1.0, null_ms_last = -1.0;
static int      tx_noted = -1;
static XTime    tx_t0;

/* receiver noise floor, normalised to 0 dB RX gain [LSB^2 / linear gain] */
static double   nf0 = -1.0;
static double   rho_n_re, rho_n_im;
/* RX2 input normalisation: g2 (the Q3.13 value in the FPGA), what is left of
 * the RX2/RX1 noise ratio after it, and where g2 came from */
static double   ratio12 = -1.0, g2 = 1.0, rx2n = 1.0;
static int      g2_src;          /* 0 default (1.0), 1 noise floors, 2 jammer */

static int      band = GNSS_BAND_DEFAULT;

static void print_band_cover(void);       /* RF profiles section, below */

/* ---- crpa8: smoothing of the weights in the FPGA (AJ_CFG[19:16]) and the fine output gain ----
 * ramp_k: 0 = off (a new weight set acts at once, as crpa7), 1..15 = time constant 2^(k+7)
 * samples. The first weights after the bypass go in with AJ_RAMP_FAST so a new jammer is
 * nulled at once; after that every new set glides in with ramp_k. */
#define AJ_RAMP_DEFAULT   10       /* 2^17 samples: 4.3 ms at 30.72 MS/s, 2.1 ms at 61.44 MS/s */
#define AJ_RAMP_FAST       1       /* 256 samples (8 us at 30.72 MS/s) */
static int      ramp_k = AJ_RAMP_DEFAULT;
static int      ramp_from_bypass = 1;    /* the committed set is the bypass (all weights 0) */
static int      ramp_hw;                 /* bitstream has the smoothing (AJ_INFO layout 10) */
static double   fine_db;                 /* fine output gain on top of the 6 dB make-up shift [dB] */
static uint32_t gain_q14 = GNSS_PT_AJ_GAIN_ONE;

/* ---- crpa8: the ARM's work per capture in three steps: prepare (here), compute (aj_detect /
 * aj_compute: pure maths on a copy), finish (here). On Linux the compute step runs on the
 * second Cortex-A9 core (GNSS_AJ_THREAD) while this thread keeps the console, the capture
 * and the gain loop going; bare metal runs the three steps in a row as before. */
#define JOB_DETECT 1
#define JOB_SOLVE  2
static uint32_t solve_us_last, solve_us_max, job_gen;
#ifdef GNSS_AJ_THREAD
static pthread_mutex_t math_mx = PTHREAD_MUTEX_INITIALIZER;   /* the solver's statistics */
static pthread_mutex_t job_mx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  job_cv = PTHREAD_COND_INITIALIZER;
static int             job_started, job_pending, job_running, job_done, job_kind, job_rc;
static uint32_t        job_gen_at, n_busy;
static XTime           job_now;
static aj_result_t     job_res;
static uint32_t        job_buf[2 * AJ_NSAMP];
#define MATH_LOCK()   pthread_mutex_lock(&math_mx)
#define MATH_UNLOCK() pthread_mutex_unlock(&math_mx)
#else
#define MATH_LOCK()   do { } while(0)
#define MATH_UNLOCK() do { } while(0)
#endif

struct aj_event { uint32_t ms; int type; double v1, v2, v3; };
static struct aj_event ev[AJ_NEV];
static int      ev_n;

/* ---- helpers --------------------------------------------------------------- */
static XTime us(uint32_t u)
{
	return (XTime)((uint64_t)COUNTS_PER_SECOND * u / 1000000U);
}

static uint32_t ms_since(XTime t0, XTime now)
{
	return (uint32_t)((uint64_t)(now - t0) * 1000U / COUNTS_PER_SECOND);
}

static void stats_reset(void)
{
	MATH_LOCK();
	aj_reset();
	job_gen++;                 /* a compute step still running is from before: its result is dropped */
	MATH_UNLOCK();
}

static double db2lin(double db)          /* 10^(db/10) without libm */
{
	double r = 1.0;
	while(db >= 10.0)  { r *= 10.0; db -= 10.0; }
	while(db <= -10.0) { r *= 0.1;  db += 10.0; }
	while(db >= 1.0)   { r *= 1.2589254117941673; db -= 1.0; }
	while(db <= -1.0)  { r *= 0.7943282347242815; db += 1.0; }
	/* |db| < 1: e^(db ln10/10), series */
	{
		double x = db * 0.23025850929940458, t = 1.0, s = 1.0;
		int k;
		for(k = 1; k < 12; k++) { t *= x / k; s += t; }
		r *= s;
	}
	return r;
}

/* Number text for console_print (which has no float format): dec decimals,
 * '+' in front of positive values if plus. Sixteen rotating buffers so one
 * print can carry several values (the CSV line has nine). */
static char *fmt(double v, int dec, int plus)
{
	static char buf[16][24];
	static int  idx;
	char *b = buf[idx++ & 15], tmp[16];
	double a = (v < 0.0) ? -v : v, sc = 1.0;
	long t, ip, fr;
	int n = 0, i = 0, k;
	for(k = 0; k < dec; k++) sc *= 10.0;
	if(a > 2.0e9 / sc) a = 2.0e9 / sc;
	t  = (long)(a * sc + 0.5);
	ip = t / (long)sc;
	fr = t % (long)sc;
	if(v < 0.0 && t) b[i++] = '-';
	else if(plus)    b[i++] = '+';
	do { tmp[n++] = (char)('0' + ip % 10); ip /= 10; } while(ip);
	while(n) b[i++] = tmp[--n];
	if(dec > 0) {
		b[i++] = '.';
		for(k = dec - 1; k >= 0; k--) { tmp[k] = (char)('0' + fr % 10); fr /= 10; }
		for(k = 0; k < dec; k++) b[i++] = tmp[k];
	}
	b[i] = 0;
	return b;
}
#define f1(v)   fmt((v), 1, 1)
#define f1u(v)  fmt((v), 1, 0)
#define f2u(v)  fmt((v), 2, 0)
#define f3u(v)  fmt((v), 3, 0)

/* residual for display: below 0 dB just means "at the noise", show 0.0 */
static double res_disp(double r) { return (r < 0.0) ? 0.0 : r; }

static char *supp_txt(double s)          /* ">60.0" when capped */
{
	static char b[2][16];
	static int  idx;
	char *o = b[idx++ & 1], *v = f1u(s >= 59.9 ? 60.0 : s);
	int i = 0;
	if(s >= 59.9) o[i++] = '>';
	while(*v && i < 15) o[i++] = *v++;
	o[i] = 0;
	return o;
}

static int32_t cur_gain(void)  { return gnss_rxg_gain(); }
static int     cut_db(void)
{
	int c = (int)(gnss_rxg_ref() - cur_gain());
	return (gnss_rxg_mode() == GNSS_RXG_MATCHED && c > 0) ? c : 0;
}

static double tx_att_db(void)
{
	uint32_t att = 0;
	if(ad9361_phy) ad9361_get_tx_attenuation(ad9361_phy, 0, &att);
	return (double)att / 1000.0;
}

static int transmitting(void)
{
	uint32_t st = gnss_pt_read(GNSS_PT_REG_STATUS);
	return (st & GNSS_PT_ST_DAC_EN_I0) && (st & GNSS_PT_ST_DAC_EN_Q0) &&
	       (st & GNSS_PT_ST_PASS_EN_SYNCED);
}

/* Noise at the ADC for the present RX gain: learned floor x gain, plus the
 * 12-bit quantisation floor. <= 0 while nothing has been learned. */
static double noise_now(void)
{
	if(nf0 <= 0.0) return -1.0;
	return nf0 * db2lin((double)cur_gain()) + AJ_QFLOOR;
}

static int32_t g2_q13(void) { return (int32_t)(g2 * 8192.0 + 0.5); }

/* ---- pulse blanker (bitstream 4.6) and pulsed jammers (firmware 4.7) ------------------------
 * A jammer that is on only part of the time (pulsed barrage) leaves bursts at the filter output
 * whenever the weights are not right for it, and the receiver loses lock on them. The FPGA
 * blanker (gnss_pulse_blank.v) zeroes the output while a burst is present; the threshold is set
 * here from the output noise: 8 samples of I^2+Q^2 above (noise x gains) x 10^(blk_db/10).
 * It is also armed while the filter is bypassed (no jammer detected yet), so a pulse the
 * detector never sees is still blanked. If it ends up blanking most of the output (a continuous
 * jammer the filter does not null) it is switched off for AJ_BLK_OFF_US. The burst counter is
 * also the evidence of a PULSED jammer: the weights are then solved from the blocks that contain
 * a pulse only (aj_result_t.gate), JAMMER OFF is delayed, and the RX gain loop keeps the gain
 * down (gnss_rxg_pulsy). */
#define AJ_BLK_DB_DEFAULT   7.0         /* threshold above the output noise [dB]               */
#define AJ_BLK_DUTY_MAX     0.6         /* blanked more than this for AJ_BLK_GUARD_US: not pulses */
#define AJ_BLK_GUARD_US     1000000U
#define AJ_BLK_OFF_US       10000000U
#define AJ_BLK_POLL_US      100000U
#define AJ_BLK_BURSTS_MIN   4           /* bursts in one poll that count as a pulse train      */
#define AJ_PULSE_HOLD_US    3000000U    /* pulses seen this recently: pulsed jammer            */
#define AJ_OFF_PULSE_US     3000000U    /* JAMMER OFF delay while it is pulsed                 */

static int      blk_hw, blk_mode = 0, blk_seeded, blk_guard_on;   /* 4.9.3: the pulse blanker is OFF by default (gnss_aj_blank=1 turns it on) */
static double   blk_db = AJ_BLK_DB_DEFAULT, blk_duty;
static uint32_t blk_thr_set = GNSS_PT_BLK_THR_OFF, blk_c0, blk_b0, blk_r0, n_gate;
static XTime    blk_t0, blk_guard_t0, blk_off_until, blk_ref_t, pulse_t;

static uint32_t blk_threshold(int filtered)
{
	double nz, nout, thr;
	int k;
	XTime now;

	if(!blk_hw || blk_mode == 0) return GNSS_PT_BLK_THR_OFF;
	XTime_GetTime(&now);
	if(blk_mode == 1 && now < blk_off_until) return GNSS_PT_BLK_THR_OFF;
	nz = noise_now();
	if(nz <= 0.0 && filtered && res.sig2 > 0.0) nz = res.sig2;   /* no clean floor yet */
	if(nz <= 0.0) return GNSS_PT_BLK_THR_OFF;
	nout = nz * (filtered ? 1.0 + res.wn2 : 1.0);
	for(k = 0; k < shift; k++) nout *= 4.0;
	thr = 8.0 * nout * db2lin(blk_db);
	if(thr > 4.0e9) thr = 4.0e9;
	if(thr < 16.0) thr = 16.0;
	return (uint32_t)thr;
}

static uint32_t blk_cfg_bit(uint32_t thr)
{
	return (blk_hw && thr != GNSS_PT_BLK_THR_OFF) ? GNSS_PT_AJ_CFG_BLANK : 0U;
}

static int pulsed(XTime now)
{
	if(gnss_rxg_pulsy()) return 1;
	return pulse_t != 0 && now - pulse_t < us(AJ_PULSE_HOLD_US);
}

static void stamp(XTime now)
{
	console_print("[AJ %s s] ", f3u((double)ms_since(t_boot, now) / 1000.0));
}

/* Returns the event's number (for filling its values in later). */
static int add_event(XTime now, int type, double v1, double v2, double v3)
{
	struct aj_event *e = &ev[ev_n % AJ_NEV];
	e->ms = ms_since(t_boot, now);
	e->type = type; e->v1 = v1; e->v2 = v2; e->v3 = v3;
	return ev_n++;
}

/* ---- FPGA filter ------------------------------------------------------------ */
static void commit_bypass(void)
{
	/* 4.9.13: zero weights while bypassed. The output is x1[n-REF] either way (bit for
	 * bit), but with the last jammer's weights still loaded the filter's multipliers kept
	 * multiplying every sample by them for nothing. With zero weights their products and
	 * the adder tree stay at 0: less switching power in the state the board spends most
	 * of its time in (no jammer). commit_weights() loads real weights again on a jammer. */
	static const int16_t zero_w[AJ_NW] = {0};
	shift = 0;
	fine_db = 0.0; gain_q14 = GNSS_PT_AJ_GAIN_ONE;
	gnss_rxg_set_digital_db(0);
	blk_thr_set = blk_threshold(0);
	/* crpa8: with smoothing the filter fades out (the weights glide to 0) */
	gnss_pt_write_aj(zero_w, zero_w, GNSS_PT_AJ_CFG_BYPASS | blk_cfg_bit(blk_thr_set) | GNSS_PT_AJ_CFG_RAMP(ramp_hw ? ramp_k : 0),
	                 g2_q13(), blk_thr_set, gain_q14);
	ramp_from_bypass = 1;
}

static void commit_weights(void)
{
	/* crpa8: the first set after the bypass goes in fast (a new jammer is nulled at once),
	 * every later one glides in with the time constant of gnss_aj_ramp= */
	int k = !ramp_hw ? 0 : ramp_from_bypass ? (ramp_k ? AJ_RAMP_FAST : 0) : ramp_k;
	gnss_rxg_set_digital_db(6 * shift + (int)(fine_db + 0.5));
	blk_thr_set = blk_threshold(1);
	gnss_pt_write_aj(res.wre, res.wim, GNSS_PT_AJ_CFG_SHIFT(shift) | GNSS_PT_AJ_CFG_WS(res.ws) | blk_cfg_bit(blk_thr_set) |
	                 GNSS_PT_AJ_CFG_RAMP(k), g2_q13(), blk_thr_set, ramp_hw ? gain_q14 : GNSS_PT_AJ_GAIN_ONE);
	ramp_from_bypass = 0;
}

/* crpa8: Q2.14 gain word for a fine gain of db (0 .. 6 dB) */
static uint32_t gain_word(double db)
{
	double g = aj_sqrt(db2lin(db)) * 16384.0 + 0.5;
	if(g < 1.0) g = 1.0;
	if(g > 65535.0) g = 65535.0;
	return (uint32_t)g;
}

/* ---- DMA --------------------------------------------------------------------- */
static void dma_start(void)
{
	uint32_t v = 0;
	axi_dmac_read(rx_dmac, AXI_DMAC_REG_CTRL, &v);
	if(!(v & AXI_DMAC_CTRL_ENABLE)) {
		axi_dmac_write(rx_dmac, AXI_DMAC_REG_CTRL, 0x0);
		axi_dmac_write(rx_dmac, AXI_DMAC_REG_CTRL, AXI_DMAC_CTRL_ENABLE);
	}
	axi_dmac_write(rx_dmac, AXI_DMAC_REG_IRQ_MASK, 0x0);
	/* Clear SOT/EOT left over from any earlier transfer (W1C): the polled
	 * driver path never clears them, so a stale EOT would end our wait at
	 * once and we would read the previous buffer. */
	axi_dmac_write(rx_dmac, AXI_DMAC_REG_IRQ_PENDING, AXI_DMAC_IRQ_SOT | AXI_DMAC_IRQ_EOT);
	axi_dmac_read(rx_dmac, AXI_DMAC_REG_TRANSFER_SUBMIT, &v);
	if(v & AXI_DMAC_QUEUE_FULL) {
		XTime now;
		n_fail++;
		XTime_GetTime(&now);
		next_start = now + us(10000U);
		return;
	}
	axi_dmac_write(rx_dmac, AXI_DMAC_REG_DEST_ADDRESS, GNSS_AJ_BUF_ADDR);
	axi_dmac_write(rx_dmac, AXI_DMAC_REG_DEST_STRIDE, 0x0);
	axi_dmac_write(rx_dmac, AXI_DMAC_REG_X_LENGTH, AJ_BYTES - 1U);
	axi_dmac_write(rx_dmac, AXI_DMAC_REG_Y_LENGTH, 0x0);
	axi_dmac_write(rx_dmac, AXI_DMAC_REG_TRANSFER_SUBMIT, AXI_DMAC_TRANSFER_SUBMIT);
	dma_busy = 1;
	XTime_GetTime(&dma_t0);
	gain_mark = gnss_rxg_changes();
}

static int dma_done(void)
{
	uint32_t v = 0;
	axi_dmac_read(rx_dmac, AXI_DMAC_REG_IRQ_PENDING, &v);
	return (v & AXI_DMAC_IRQ_EOT) ? 1 : 0;
}

void gnss_aj_dma_release(void)
{
	XTime t0, now;
	if(!dma_busy || !rx_dmac) return;
	XTime_GetTime(&t0);
	do {
		if(dma_done()) break;
		XTime_GetTime(&now);
	} while(now - t0 < us(AJ_DMA_TIMEOUT_US));
	axi_dmac_write(rx_dmac, AXI_DMAC_REG_IRQ_PENDING, AXI_DMAC_IRQ_SOT | AXI_DMAC_IRQ_EOT);
	dma_busy = 0;
}

/* ---- jammer description ------------------------------------------------------ */
static void radio(uint32_t *fs, uint64_t *lo)
{
	*fs = 0; *lo = 0;
	if(!ad9361_phy) return;
	ad9361_get_rx_sampling_freq(ad9361_phy, fs);
	ad9361_get_rx_lo_freq(ad9361_phy, lo);
}

static void print_type(void)
{
	uint32_t fs; uint64_t lo;
	double bw_mhz, f_mhz, rf_mhz;
	radio(&fs, &lo);
	bw_mhz = res.bw * (double)fs / 1e6;
	f_mhz  = res.foff * (double)fs / 1e6;
	rf_mhz = (double)lo / 1e6 + f_mhz;
	if(bw_mhz < 0.2)
		console_print("CW tone at %s MHz (RF %s MHz)", f1(f_mhz), f1u(rf_mhz));
	else if(res.bw > 0.75)
		/* A jammer that fills the whole receive band has no meaningful
		 * centre: the lag-1 correlation is ~0 and its phase is noise. */
		console_print("WIDEBAND, fills the whole %s MHz band (~%s MHz seen)",
			      f1u((double)fs / 1e6), f1u(bw_mhz));
	else
		console_print("%s ~%s MHz wide, centre %s MHz (RF %s MHz)",
			      (char*)(bw_mhz < 5.0 ? "NARROWBAND" : "PARTIAL-BAND"),
			      f1u(bw_mhz), f1(f_mhz), f1u(rf_mhz));
}

static const char *why_txt(int why)
{
	switch(why) {
	case 1:  return "RX1/RX2 coherence (both antennas see the same signal)";
	case 2:  return "RX power above the learned noise floor";
	case 3:  return "the RX gain loop (it had to cut the gain)";
	default: return "?";
	}
}

static void print_line(XTime now)
{
	stamp(now);
	if(state == ST_JAMMED && res_solved) {
		console_print("JAMMED  J/N %s dB  suppression %s dB  residual %s dB | ",
			      f1(res.jn_db), supp_txt(res.supp_db), f1u(res_disp(res.res_db)));
		print_type();
		console_print(" | RX %d/%d dB  make-up +%s dB  TX att %s dB | %d solves/s%s\n",
			      (long)cur_gain(), (long)gnss_rxg_ref(), f1u(6.0 * shift + fine_db),
			      f2u(tx_att_db()), (long)upd_rate, (char*)(aj_frozen ? " FROZEN" : ""));
	} else {
		console_print("clear   noise %s LSB^2  coherence %s dB  power J/N %s | RX %d/%d dB"
			      "  TX att %s dB%s | g2 %s dB | bypass, %d checks/s%s\n",
			      f1u(res.p1), f1(res.jn_coh_db < -30.0 ? -30.0 : res.jn_coh_db),
			      (char*)(nf0 > 0.0 ? f1(res.jn_db < -9.9 ? -9.9 : res.jn_db) : "(learning)"),
			      (long)cur_gain(), (long)gnss_rxg_ref(), f2u(tx_att_db()),
			      (char*)(transmitting() ? "" : " (TX off)"),
			      f1(aj_db10(g2 * g2)), (long)det_rate, (char*)(aj_frozen ? " FROZEN" : ""));
	}
}

static void print_csv(XTime now)
{
	uint32_t fs; uint64_t lo;
	if(!csv_hdr) {
		csv_hdr = 1;
		console_print("AJCSV,t_s,state,jn_db,coh_db,supp_db,res_db,rx_gain_db,cut_db,"
			      "makeup_db,tx_att_db,g2_db,rate_per_s,bw_mhz,centre_mhz,clip_ppm,fine_db\n");
	}
	radio(&fs, &lo);
	console_print("AJCSV,%s,%s,%s,%s,%s,%s,%d,%d,%d,%s,%s,%d,%s,%s,%d,%s\n",
		      f3u((double)ms_since(t_boot, now) / 1000.0),
		      (char*)(state == ST_JAMMED ? "JAMMED" : "CLEAR"),
		      fmt(res.jn_db < -99.0 ? -99.0 : res.jn_db, 1, 0),
		      fmt(res.jn_coh_db < -30.0 ? -30.0 : res.jn_coh_db, 1, 0),
		      fmt(res_solved ? res.supp_db : 0.0, 1, 0),
		      fmt(res_solved ? res_disp(res.res_db) : 0.0, 1, 0),
		      (long)cur_gain(), (long)cut_db(), (long)(6 * shift),
		      f2u(tx_att_db()), fmt(aj_db10(g2 * g2), 1, 0),
		      (long)(state == ST_JAMMED ? upd_rate : det_rate),
		      fmt(res.bw * (double)fs / 1e6, 1, 0), fmt(res.foff * (double)fs / 1e6, 1, 0),
		      (long)gnss_rxg_clip_ppm(), f2u(fine_db));
}

/* 4.9.6: the CSV header and one line, whatever gnss_aj_csv= says (for gnss_log.c); leaves the periodic CSV alone. */
void gnss_aj_csv_once(void)
{
	XTime now;
	int keep = csv_hdr;
	XTime_GetTime(&now);
	csv_hdr = 0;
	print_csv(now);
	csv_hdr = keep;
}

/* ---- learning ------------------------------------------------------------------ */
static void update_g2(void)
{
	double g = aj_sqrt(ratio12);
	if(g > AJ_G2_MAX) g = AJ_G2_MAX;
	if(g < AJ_G2_MIN) g = AJ_G2_MIN;
	g2 = (double)((int32_t)(g * 8192.0 + 0.5)) / 8192.0;
	/* what is left of RX2's noise relative to RX1's after the scaling */
	rx2n = g2 * g2 / ratio12;
}

static void learn(XTime now)
{
	/* Only with no jammer and the RX gain at its clean reference: then the
	 * two powers ARE the receivers' noise (the satellites are ~30 dB under
	 * it). The learned floor follows it down at once and up only slowly, so
	 * a jammer that creeps in cannot drag the floor up with it. */
	double g = db2lin((double)cur_gain());
	double cand = (res.p1 - AJ_QFLOOR) / g;
	double dt;
	if(cand <= 0.0) return;
	if(nf0 <= 0.0 || cand < nf0) {
		nf0 = cand;
	} else {
		dt = (double)(now - t_learn) / (double)COUNTS_PER_SECOND;
		if(dt > 1.0) dt = 1.0;
		{
			double lim = nf0 * db2lin(AJ_NF_RISE_DB_S * dt);
			nf0 = (cand < lim) ? cand : lim;
		}
	}
	t_learn = now;
	/* the noise's own lag-1 correlation (the AD9361 filters colour it) */
	rho_n_re = 0.95 * rho_n_re + 0.05 * res.rho1_re;
	rho_n_im = 0.95 * rho_n_im + 0.05 * res.rho1_im;
	/* RX1/RX2 noise ratio -> the input normalisation g2 */
	if(res.p2 > AJ_QFLOOR) {
		double r = res.p1 / res.p2;
		ratio12 = (ratio12 <= 0.0) ? r : (1.0 - AJ_G2_ALPHA) * ratio12 + AJ_G2_ALPHA * r;
		update_g2();
		g2_src = 1;
	}
}

/* ---- console etiquette ------------------------------------------------------
 * Event lines wait while the operator types (and AJ_KEY_QUIET_US after the
 * last key) and are counted instead; gnss_aj_events=0 silences them (they
 * are still kept for gnss_report?). A signal whose 3rd switch-on comes
 * within AJ_FLAP_US of the 1st is "intermittent": one summary line every
 * AJ_FLAP_SUM_US instead of an ON/NULLED/OFF block each time. */
#define AJ_FLAP_US        20000000U
#define AJ_FLAP_SUM_US    30000000U
static int    aj_events = 1, n_held, flap_mode, flap_n, have_off, jam_silent;
static double flap_peak = -99.0;
static XTime  last_off_t, flap_t0, on_hist[2];
static int    n_on_hist;

static void key_track(XTime now)
{
	if(console_key_count() != keys_seen) {
		keys_seen = console_key_count();
		t_key = now;
	}
}

static int ev_print(XTime now)
{
	key_track(now);
	if(!aj_events) return 0;
	if(console_typing() || now - t_key < us(AJ_KEY_QUIET_US)) {
		n_held++;
		return 0;
	}
	return 1;
}

/* At each jammer announcement: 1 = intermittent, do not print it. */
static int flap_check(XTime now)
{
	int third = n_on_hist >= 2 && now - on_hist[0] < us(AJ_FLAP_US);
	on_hist[0] = on_hist[1];
	on_hist[1] = now;
	if(n_on_hist < 2) n_on_hist++;
	if(!flap_mode && third) {
		flap_mode = 1;
		flap_n = 0;
		flap_peak = -99.0;
		flap_t0 = now;
		if(ev_print(now)) {
			stamp(now);
			console_print("signal keeps switching on and off: one summary line every %d s from now\n"
				      "     (gnss_report? lists each event)\n", (long)(AJ_FLAP_SUM_US / 1000000U));
		}
	}
	if(flap_mode) {
		flap_n++;
		if(res.jn_db > flap_peak) flap_peak = res.jn_db;
	}
	return flap_mode;
}

static void flap_poll(XTime now)
{
	if(n_held > 0 && aj_events && !console_typing() && now - t_key >= us(AJ_KEY_QUIET_US)) {
		stamp(now);
		console_print("%d event line(s) held back while you typed: gnss_report? lists them\n", (long)n_held);
		n_held = 0;
	}
	if(!aj_events) n_held = 0;
	if(!flap_mode || now - flap_t0 < us(AJ_FLAP_SUM_US)) return;
	if(flap_n == 0 && !(state == ST_JAMMED)) {
		flap_mode = 0;
		if(ev_print(now)) {
			stamp(now);
			console_print("intermittent signal quiet for %d s: one line per event again\n",
				      (long)(AJ_FLAP_SUM_US / 1000000U));
		}
	} else if(ev_print(now)) {
		stamp(now);
		console_print("intermittent: %d on/off in the last %d s, peak J/N %s dB, %s now\n",
			      (long)flap_n, (long)(AJ_FLAP_SUM_US / 1000000U), f1(flap_peak),
			      (char*)((state == ST_JAMMED) ? "ON" : "off"));
	}
	flap_n = 0;
	flap_peak = -99.0;
	flap_t0 = now;
}

/* ---- incoherent noise rise --------------------------------------------------
 * Power up in RX1 with no RX1/RX2 coherence and no gain cut, for AJ_RISE_US:
 * the receivers' own noise rose (active antenna LNA, its bias supply, a
 * cable), not a jammer: a jammer is one source, both antennas see it. The
 * learned floor is moved up to it at once. */
#define AJ_RISE_US        3000000U
#define AJ_PWR_COH_DB     (-9.0)  /* power detection needs at least this coherence */
static int   rise_on;
static XTime rise_t0;

static void rise_track(XTime now)
{
	int c = nf0 > 0.0 && res.jn_db >= AJ_OFF_DB && res.jn_coh_db < AJ_PWR_COH_DB &&
	        cut_db() == 0 && gnss_rxg_ref() == cur_gain();
	if(!c) {
		rise_on = 0;
		return;
	}
	if(!rise_on) {
		rise_on = 1;
		rise_t0 = now;
		return;
	}
	if(now - rise_t0 >= us(AJ_RISE_US)) {
		double cand = (res.p1 - AJ_QFLOOR) / db2lin((double)cur_gain()), up;
		rise_on = 0;
		if(cand <= nf0) return;
		up = aj_db10(cand / nf0);
		nf0 = cand;
		add_event(now, EV_FLOOR, up, res.jn_coh_db, 0.0);
		if(ev_print(now)) {
			stamp(now);
			console_print("noise floor rose %s dB without RX1/RX2 coherence: not a jammer\n"
				      "     (active antenna LNA / its bias supply / a cable?). Floor re-learned.\n",
				      f1(up));
		}
	}
}

/* ---- the state machine ----------------------------------------------------------- */
static void announce(XTime now)
{
	int show;
	jam_announced = 1;
	jam_peak = res.jn_db;
	jam_silent = flap_check(now);
	show = !jam_silent && ev_print(now);
	if(show) console_print("\n");
	if(show) stamp(now);
	if(show) console_print("*** JAMMER ON ***  J/N %s dB, ", f1(res.jn_db));
	if(show) print_type();
	if(show) console_print("\n           detected because %s; RX gain cut %d dB\n",
		      (char*)why_txt(on_why), (long)cut_db());
	/* the ON event was logged at the onset; its figures are the settled ones */
	if(on_ev > ev_n - AJ_NEV) {
		ev[on_ev % AJ_NEV].v1 = res.jn_db;
		ev[on_ev % AJ_NEV].v2 = res.bw;
	}
	if(jam_nulled) {
		if(show) stamp(now);
		if(show) console_print("NULLED in %s ms: suppression %s dB, residual %s dB, make-up +%d dB\n",
			      fmt(null_ms, 0, 0), supp_txt(res.supp_db), f1u(res_disp(res.res_db)),
			      (long)(6 * shift));
	}
}

static void enter_jammed(XTime now)
{
	state = ST_JAMMED;
	on_cnt = 0;
	/* the jammer's onset is the first capture that saw it */
	jam_t0 = trig_t0;
	quiet_t0 = res_bad_t0 = now;
	on_ev = add_event(trig_t0, EV_ON, res.jn_db, res.bw, (double)on_why);
	jam_announced = 0;
	jam_nulled = 0;
	null_ms = -1.0;
	jam_peak = -99.0;
	shift = 0;
	/* No clean period since start-up (the jammer was there first): g2 from
	 * the jammer itself, which both antennas see. */
	if(g2_src == 0 && res.p2 > AJ_QFLOOR) {
		ratio12 = res.p1 / res.p2;
		update_g2();
		rx2n = 1.0;
		g2_src = 2;
	}
	stats_reset();
	res.sig2_prev = -1.0;
	gain_seen = gnss_rxg_changes();
	aj_sweep_reset(&sweep);
	sweep_avg = 0.0;
}

static void leave_jammed(XTime now)
{
	uint32_t ms = ms_since(jam_t0, now);
	state = ST_CLEAR;
	on_cnt = 0;
	clear_t0 = now;
	if(!aj_frozen) commit_bypass();
	stats_reset();
	if(!jam_announced) announce(now);
	if(!jam_silent && ev_print(now)) {
		console_print("\n");
		stamp(now);
		console_print("jammer OFF after %s s (peak J/N %s dB) -> filter bypassed, TX1 = RX1\n",
			      f1u((double)ms / 1000.0), f1(jam_peak));
	}
	last_off_t = now;
	have_off = 1;
	if(flap_mode && jam_peak > flap_peak) flap_peak = jam_peak;
	add_event(now, EV_OFF, (double)ms / 1000.0, jam_peak, 0.0);
	if(null_ms >= 0.0) null_ms_last = null_ms;
}

/* ---- common background on RX1 and RX2 ---------------------------------------
 * A signal BOTH channels carry all the time at about the noise level (one
 * source through a splitter, a GNSS simulator, one active antenna, or TX1
 * leaking back into RX) makes the coherence detector fire with no rise in
 * power and no RX gain cut. After AJ_BG_US of exactly that it becomes the
 * background: a jammer must then add AJ_BG_MARGIN_DB of coherence over it,
 * or raise the power, or make the gain loop cut the gain. Forgotten once the
 * coherence has stayed 6 dB below it for AJ_BG_CLEAR_US. */
#define AJ_BG_US          10000000U
#define AJ_BG_MARGIN_DB   6.0
#define AJ_BG_CLEAR_US    30000000U
static double coh_bg = -99.0, bg_max;
static int    bg_cand;
static XTime  bg_t0, bg_gone_t0;

static double coh_on(void)    { double t = coh_bg + AJ_BG_MARGIN_DB; return t > AJ_ON_COH_DB ? t : AJ_ON_COH_DB; }
static double coh_off(void)   { double t = coh_bg + 3.0; return t > AJ_OFF_COH_DB ? t : AJ_OFF_COH_DB; }
static double coh_quiet(void) { double t = coh_bg + 3.0; return t > AJ_QUIET_COH_DB ? t : AJ_QUIET_COH_DB; }

/* Every capture. jammed: held jammed by the firmware's detector. Returns 1
 * when that state is only the background and must be released. */
static int bg_track(XTime now, int jammed)
{
	/* not quiet only because of the coherence; with no learned floor yet
	 * (no clean moment since start-up) the power cannot be judged */
	int cand = jammed && res.jn_coh_db >= coh_off() &&
	           (nf0 <= 0.0 || res.jn_db < AJ_OFF_DB) && cut_db() == 0;
	if(!cand) {
		bg_cand = 0;
	} else if(!bg_cand) {
		bg_cand = 1;
		bg_t0 = now;
		bg_max = res.jn_coh_db;
	} else {
		if(res.jn_coh_db > bg_max) bg_max = res.jn_coh_db;
		if(now - bg_t0 >= us(AJ_BG_US)) {
			coh_bg = bg_max;
			bg_cand = 0;
			bg_gone_t0 = now;
			if(nf0 <= 0.0 && res.p1 > AJ_QFLOOR)      /* the floor is this */
				nf0 = (res.p1 - AJ_QFLOOR) / db2lin((double)cur_gain());
			console_print("\n");
			stamp(now);
			console_print("NOT A JAMMER: RX1 and RX2 have carried the same signal for %d s at the\n"
				      "     noise level (coherence %s dB, power J/N %s dB, no RX gain cut).\n"
				      "     Treated as background, filter bypassed. A jammer must now reach\n"
				      "     coherence %s dB or power J/N %s dB, or cut the RX gain.\n"
				      "     Usual causes: both RX ports fed from ONE source (splitter, GNSS\n"
				      "     simulator, one antenna/LNA): then the satellites are common too and\n"
				      "     no 2-antenna anti-jam can work; or TX1 leaking back into RX\n"
				      "     (test: gnss_tx=0, the coherence should drop).\n",
				      (long)(AJ_BG_US / 1000000U), f1(coh_bg), f1(res.jn_db),
				      f1(coh_on()), f1(AJ_ON_DB));
			add_event(now, EV_BG, coh_bg, res.jn_db, 0.0);
			return 1;
		}
	}
	if(coh_bg > -90.0) {
		if(res.jn_coh_db >= coh_bg - 6.0) {
			bg_gone_t0 = now;
		} else if(now - bg_gone_t0 >= us(AJ_BG_CLEAR_US)) {
			stamp(now);
			console_print("background gone (coherence %s dB for %d s): normal detection again\n",
				      f1(res.jn_coh_db), (long)(AJ_BG_CLEAR_US / 1000000U));
			coh_bg = -99.0;
			add_event(now, EV_BG, -99.0, res.jn_db, 0.0);
		}
	}
	return 0;
}

/* crpa8: the compute step (pure maths on buf and r; on Linux in the worker thread) */
static int run_math(int kind, const uint32_t *buf, aj_result_t *r)
{
	XTime t0, t1;
	uint32_t us_;
	int rc;
	MATH_LOCK();
	XTime_GetTime(&t0);
	rc = (kind == JOB_DETECT) ? aj_detect(buf, AJ_NSAMP, r) : aj_compute(buf, AJ_NSAMP, r);
	XTime_GetTime(&t1);
	us_ = (uint32_t)((uint64_t)(t1 - t0) * 1000000U / COUNTS_PER_SECOND);
	if(kind == JOB_SOLVE) {                 /* read under the same lock (gnss_report?) */
		solve_us_last = us_;
		if(us_ > solve_us_max) solve_us_max = us_;
	}
	MATH_UNLOCK();
	return rc;
}

static void prep_detect(void)
{
	res.noise_ref = noise_now();
	res.rho_noise_re = rho_n_re;
	res.rho_noise_im = rho_n_im;
}

/* CLEAR: detection only. Returns 1 when the jammer is confirmed. */
static int finish_detect(XTime now, int rc)
{
	int cut = cut_db(), why = 0, quiet;

	if(rc != 0 || !res.ok) {
		n_fail++;
		return 0;
	}
	res_valid = 1;
	res_solved = 0;
	n_det++;
	cnt_det++;

	bg_track(now, 0);
	rise_track(now);
	if(res.jn_coh_db >= coh_on())                  why = 1;
	else if(nf0 > 0.0 && res.jn_db >= AJ_ON_DB &&
	        res.jn_coh_db >= AJ_PWR_COH_DB)        why = 2;
	else if(cut >= AJ_CUT_DB && now - t_boot >= us(AJ_SETTLE_US)) why = 3;
	quiet = !why && res.jn_coh_db < coh_quiet() &&
	        (nf0 <= 0.0 || res.jn_db < AJ_OFF_DB) && cut == 0;
	if(quiet && gnss_rxg_ref() == cur_gain())
		learn(now);

	on_cnt = why ? on_cnt + 1 : 0;
	if(on_cnt == 1) trig_t0 = now;
	if(why) on_why = why;
	if(on_cnt >= AJ_ON_COUNT) {
		enter_jammed(now);
		return 1;
	}
	return 0;
}

static void jammed_state(XTime now)
{
	int quiet;

	if(!jam_nulled && res.res_db <= AJ_NULL_RES_DB) {
		jam_nulled = 1;
		null_ms = (double)ms_since(jam_t0, now);
		if(jam_announced && !jam_silent && ev_print(now)) {
			stamp(now);
			console_print("NULLED in %s ms: suppression %s dB, residual %s dB\n",
				      fmt(null_ms, 0, 0), supp_txt(res.supp_db), f1u(res_disp(res.res_db)));
		}
		add_event(now, EV_NULLED, null_ms, res.supp_db, res.res_db);
	}
	if(!jam_announced && now - jam_t0 >= us(AJ_ANNOUNCE_US))
		announce(now);
	if(jam_announced && res.jn_db > jam_peak) jam_peak = res.jn_db;

	if(bg_track(now, 1)) {
		leave_jammed(now);
		return;
	}
	rise_track(now);
	quiet = res.jn_coh_db < coh_off() && res.jn_db < AJ_OFF_DB && cut_db() == 0;
	if(!quiet) {
		quiet_t0 = now;
	} else if(now - quiet_t0 >= us(pulsed(now) ? AJ_OFF_PULSE_US : AJ_OFF_US)) {
		leave_jammed(now);
		return;
	}

	if(res.res_db <= AJ_WARN_RES_DB)
		res_bad_t0 = now;
	if(jam_announced && now - res_bad_t0 >= us(AJ_WARN_US) &&
	   now - t_warn >= us(30000000U) && !jam_silent && ev_print(now)) {
		t_warn = now;
		stamp(now);
		if(cut_db() >= 25 && res.jn_db >= 54.0) {
			console_print("WARNING residual %s dB at J/N %s dB: the jammer is this far above the noise that the RX gain\n"
				      "     is cut by %d dB to keep it off the 12-bit ADC, and the ADC's own quantisation noise is part\n"
				      "     of the residual (about +%s dB at this J/N). That is the limit of the hardware, not leakage.\n"
				      "     Satellites in the jammer's direction are nulled with it. Output level %s dB re clean.\n"
				      "     Less jammer power or an attenuator before the board helps (gnss_report?).\n",
				      f1(res.res_db), f1(res.jn_db), (long)cut_db(), f1u(aj_adc_excess_db(res.jn_db)),
				      f1(-cut_db() + 6 * shift + gnss_rxg_comp_db()));
		} else {
		console_print("WARNING residual %s dB: the jammer is not fully nulled.\n"
			      "     Two antennas null ONE direction (plus its reflections). Check that\n"
			      "     RX2 has its antenna, that there is a single jammer, and the ADC peak\n"
			      "     (gnss_rx_gain?).%s\n", f1(res.res_db),
			      (char*)(aj_mode == AJ_MODE_STAP ? "" : " gnss_aj_mode=0 uses all 63+2 taps."));
		}
		add_event(now, EV_LEAK, res.res_db, res.jn_db, 0.0);
	}
}

static int detect_capture(XTime now)
{
	prep_detect();
	return finish_detect(now, run_math(JOB_DETECT, GNSS_AJ_BUF_PTR, &res));
}

/* JAMMED: solve this capture and commit the weights. */
static void prep_solve(XTime now)
{
	if(gnss_rxg_changes() != gain_seen) {
		/* the RX gain stepped since the last solve: the two channels' phase
		 * can step differently, so do not average across it */
		gain_seen = gnss_rxg_changes();
		stats_reset();
		res.sig2_prev = -1.0;
	}
	res.mode = aj_mode;
	res.noise_ref = noise_now();
	res.rho_noise_re = rho_n_re;
	res.rho_noise_im = rho_n_im;
	res.g2 = g2;
	res.rx2_noise = rx2n;
	res.avg = sweep_en ? sweep_avg : 0.0;
	res.gate = pulsed(now);                         /* a pulsed jammer: skip the blocks in its gaps */
	res.gate_coh_db = AJ_OFF_COH_DB;
	res.gate_pwr = 1.0 + db2lin(AJ_OFF_DB);
}

static void finish_solve(XTime now, int rc)
{
	int want;

	if(rc == 1 && res.gated) {
		n_gate++;
		res_valid = 1;
		jammed_state(now);                          /* weights and statistics unchanged */
		return;
	}
	if(rc != 0 || !res.ok) {
		n_fail++;
		return;
	}
	sweep_avg = sweep_en ? aj_sweep_update(&sweep, &res) : 0.0;
	res.sig2_prev = res.sig2;
	res_valid = res_solved = 1;
	n_upd++;
	cnt_upd++;

	/* Output make-up gain: 6 dB per step of RX gain cut (up to 42 dB), so the
	 * output into the receiver stays at the clean level; never more than 3 dB
	 * above it (aj_makeup_shift). Down at once, up one step per update. */
	want = aj_makeup_shift(cut_db(), res.res_db + aj_db10(1.0 + res.wn2), sh_max);
	if(want < shift)      shift = want;
	else if(want > shift) shift++;
	/* crpa8: the rest of the RX gain cut (0 .. 6 dB) through the fine gain, same headroom rule */
	fine_db = ramp_hw ? aj_makeup_fine_db(cut_db(), res.res_db + aj_db10(1.0 + res.wn2), shift) : 0.0;
	gain_q14 = gain_word(fine_db);
	if(!aj_frozen)
		commit_weights();
	jammed_state(now);
}

static void solve_capture(XTime now)
{
	prep_solve(now);
	finish_solve(now, run_math(JOB_SOLVE, GNSS_AJ_BUF_PTR, &res));
}

#ifdef GNSS_AJ_THREAD
/* ---- crpa8: the worker thread (second core) ------------------------------------------ */
static void *job_worker(void *arg)
{
	(void)arg;
	for(;;) {
		int kind;
		pthread_mutex_lock(&job_mx);
		while(!job_pending) pthread_cond_wait(&job_cv, &job_mx);
		job_pending = 0; job_running = 1; kind = job_kind;
		pthread_mutex_unlock(&job_mx);
		{
			int rc = run_math(kind, job_buf, &job_res);
			pthread_mutex_lock(&job_mx);
			job_rc = rc; job_running = 0; job_done = 1;
			pthread_mutex_unlock(&job_mx);
		}
	}
	return 0;
}

static int job_idle(void)
{
	int idle;
	pthread_mutex_lock(&job_mx);
	idle = !job_pending && !job_running && !job_done;
	pthread_mutex_unlock(&job_mx);
	return idle;
}

/* hand res (prepared) and the capture in job_buf to the worker */
static void job_start(int kind, XTime now)
{
	if(!job_started) {
		pthread_t t;
		if(pthread_create(&t, 0, job_worker, 0) != 0) {      /* no thread: do it here */
			if(kind == JOB_DETECT) { if(finish_detect(now, run_math(JOB_DETECT, job_buf, &res))) { prep_solve(now); finish_solve(now, run_math(JOB_SOLVE, job_buf, &res)); } }
			else finish_solve(now, run_math(JOB_SOLVE, job_buf, &res));
			return;
		}
		pthread_detach(t);
		job_started = 1;
	}
	pthread_mutex_lock(&job_mx);
	job_res = res;
	job_kind = kind; job_now = now; job_gen_at = job_gen;
	job_pending = 1;
	pthread_cond_signal(&job_cv);
	pthread_mutex_unlock(&job_mx);
}

/* a finished compute step: take its result and do the finish step here */
static void job_poll(void)
{
	int done, rc, kind;
	XTime now;
	pthread_mutex_lock(&job_mx);
	done = job_done; rc = job_rc; kind = job_kind; now = job_now;
	if(done) { job_done = 0; }
	pthread_mutex_unlock(&job_mx);
	if(!done) return;
	if(job_gen_at != job_gen || !aj_on) { n_skip++; return; }   /* reset / band change / off meanwhile */
	res = job_res;
	if(kind == JOB_DETECT) {
		if(finish_detect(now, rc)) {                 /* confirmed: solve the same capture */
			prep_solve(now);
			job_start(JOB_SOLVE, now);
		}
	} else {
		finish_solve(now, rc);
	}
}
#endif

/* Rewrite the threshold (and the enable) without new weights. */
static void blk_refresh(void)
{
	if(!blk_hw) return;
	if(state == ST_CLEAR)
		commit_bypass();
	else if(res_solved && !aj_frozen)
		commit_weights();
}

/* Every capture: watch the blanker's counters, keep the threshold in step with the noise. */
static void blk_service(XTime now)
{
	uint32_t c, b, r, dc, dn, dr, thr;

	if(!blk_hw) return;
	if(now - blk_t0 < us(AJ_BLK_POLL_US)) return;
	blk_t0 = now;
	c = gnss_pt_read(GNSS_PT_REG_AJ_BLK_COUNT);
	b = gnss_pt_read(GNSS_PT_REG_AJ_BLK_BURSTS);
	r = gnss_pt_read(GNSS_PT_REG_RX_COUNT_CH0);
	dc = c - blk_c0; dn = b - blk_b0; dr = r - blk_r0;
	blk_c0 = c; blk_b0 = b; blk_r0 = r;
	if(!blk_seeded) { blk_seeded = 1; return; }
	blk_duty = dr ? (double)dc / (double)dr : 0.0;

	/* pulse train: several bursts, and most of the time unblanked */
	if(blk_thr_set != GNSS_PT_BLK_THR_OFF && dn >= AJ_BLK_BURSTS_MIN && blk_duty < AJ_BLK_DUTY_MAX)
		pulse_t = now;

	/* blanking most of the output for a second is not a pulse train */
	if(blk_mode == 1 && blk_thr_set != GNSS_PT_BLK_THR_OFF && blk_duty > AJ_BLK_DUTY_MAX) {
		if(!blk_guard_on) {
			blk_guard_on = 1;
			blk_guard_t0 = now;
		} else if(now - blk_guard_t0 >= us(AJ_BLK_GUARD_US)) {
			blk_guard_on = 0;
			blk_off_until = now + us(AJ_BLK_OFF_US);
			blk_refresh();
			stamp(now);
			console_print("pulse blanker OFF for %d s: it was blanking %s %% of the output. That is a jammer the\n"
				      "     filter does not null (see 'residual' in gnss_report?), not a pulse train.\n",
				      (long)(AJ_BLK_OFF_US / 1000000U), f1(100.0 * blk_duty));
		}
	} else {
		blk_guard_on = 0;
	}

	/* the threshold follows the noise floor and the gain (every 0.5 s, only when it moved) */
	if(now - blk_ref_t >= us(500000U)) {
		blk_ref_t = now;
		thr = blk_threshold(state == ST_CLEAR ? 0 : 1);
		if((thr == GNSS_PT_BLK_THR_OFF) != (blk_thr_set == GNSS_PT_BLK_THR_OFF) ||
		   (thr != GNSS_PT_BLK_THR_OFF && ((double)thr > 1.3 * (double)blk_thr_set ||
		                                   (double)thr < 0.77 * (double)blk_thr_set)))
			blk_refresh();
	}
}

void gnss_aj_set_blank(int mode)
{
	blk_mode = (mode < 0) ? 0 : (mode > 2 ? 2 : mode);
	blk_off_until = 0;
	blk_guard_on = 0;
	blk_refresh();
}

void gnss_aj_set_blank_db(double db)
{
	if(db < 3.0) db = 3.0;
	if(db > 30.0) db = 30.0;
	blk_db = db;
	blk_refresh();
}

int gnss_aj_blank_available(void) { return blk_hw; }

void gnss_aj_poll(void)
{
	XTime now;

	if(!aj_on || !rx_dmac || !ad9361_phy) return;
	if(gnss_txdma_is_running()) return;       /* the DDR replay owns the DMA */
#ifdef GNSS_AJ_THREAD
	job_poll();
#endif
	XTime_GetTime(&now);

	if(!dma_busy) {
		if(now >= next_start) dma_start();
		return;
	}
	if(!dma_done()) {
		if(now - dma_t0 > us(AJ_DMA_TIMEOUT_US)) {
			n_fail++;
			dma_busy = 0;
			axi_dmac_write(rx_dmac, AXI_DMAC_REG_CTRL, 0x0);   /* reset the queue */
			next_start = now + us(100000U);
		}
		return;
	}
	dma_busy = 0;
	next_start = dma_t0 + us(AJ_MIN_PERIOD_US);
	if(gnss_rxg_changes() != gain_mark) {
		/* the RX gain stepped during the capture: the block mixes two gains */
		n_skip++;
		return;
	}
	Xil_DCacheInvalidateRange((uintptr_t)GNSS_AJ_BUF_ADDR, AJ_BYTES);

#ifdef GNSS_AJ_THREAD
	if(!job_idle()) {
		n_busy++;                                 /* the second core is still solving the last one */
	} else {
		memcpy(job_buf, (const void *)GNSS_AJ_BUF_PTR, sizeof(job_buf));
		if(state == ST_CLEAR) { prep_detect(); job_start(JOB_DETECT, now); }
		else                  { prep_solve(now); job_start(JOB_SOLVE, now); }
	}
#else
	if(state == ST_CLEAR) {
		/* the capture that confirmed the jammer is solved straight away */
		if(detect_capture(now))
			solve_capture(now);
	} else {
		solve_capture(now);
	}
#endif

	XTime_GetTime(&now);
	if(now - t_rate >= us(1000000U)) {
		upd_rate = cnt_upd; det_rate = cnt_det;
		cnt_upd = cnt_det = 0;
		t_rate = now;
	}
	if(console_key_count() != keys_seen) {
		keys_seen = console_key_count();
		t_key = now;
	}
	if(console_typing() || now - t_key < us(AJ_KEY_QUIET_US)) {
		t_log = now;                     /* no burst once typing is over */
		t_csv = now;
	}
	if(aj_log_s > 0 && now - t_log >= us((uint32_t)aj_log_s * 1000000U)) {
		t_log = now;
		print_line(now);
	}
	if(aj_csv_ms > 0 && now - t_csv >= us((uint32_t)aj_csv_ms * 1000U)) {
		t_csv = now;
		print_csv(now);
	}
	flap_poll(now);
	blk_service(now);
}

/* ---- control ------------------------------------------------------------------ */
void gnss_aj_init(void)
{
	XTime now;
	XTime_GetTime(&now);
	t_boot = t_rate = t_log = t_csv = t_warn = t_learn = next_start = clear_t0 = now;
	keys_seen = console_key_count();
	t_key = now - us(AJ_KEY_QUIET_US);
	res.sig2_prev = -1.0;
	stats_reset();
	/* The weight scale (AJ_CFG[14:12]) exists from bitstream 4.4 (AJ_INFO layout 3).
	 * An older bitstream would multiply the filter by 2^ws, so it is never used there. */
	ws_hw = ((gnss_pt_read(GNSS_PT_REG_AJ_INFO) & 0xFFU) >= 3U);
	aj_set_max_ws(ws_hw ? AJ_MAXWS : 0);
	sh_max = ((gnss_pt_read(GNSS_PT_REG_AJ_INFO) & 0xFFU) >= 4U) ? AJ_MAXSH : 4;
	blk_hw = ((gnss_pt_read(GNSS_PT_REG_AJ_INFO) & 0xFFU) >= GNSS_PT_AJ_LAYOUT_BLANK);
	ramp_hw = ((gnss_pt_read(GNSS_PT_REG_AJ_INFO) & 0xFFU) == GNSS_PT_AJ_LAYOUT_RAMP);
	blk_seeded = 0;
	if(!blk_hw)
		console_print("[AJ] NOTE: the bitstream has no pulse blanker (AJ_INFO layout %d, needs 5 = bitstream 4.6): pulsed\n"
			      "     jammers are only handled by the weights. Rebuild the whole project for it.\n",
			      (long)(gnss_pt_read(GNSS_PT_REG_AJ_INFO) & 0xFFU));
	if(sh_max < AJ_MAXSH)
		console_print("[AJ] NOTE: the bitstream is older than 4.5 (AJ_INFO layout %d): %s make-up shift limited to %d (+%d dB).\n"
			      "     Rebuild the whole project (not firmware only) for the full 4.5 behaviour.\n",
			      (long)(gnss_pt_read(GNSS_PT_REG_AJ_INFO) & 0xFFU),
			      (char*)(ws_hw ? "" : "weight scale off,"), (long)sh_max, (long)(6 * sh_max));
}

void gnss_aj_command(int on)
{
	aj_user_off = on ? 0 : 1;
	gnss_aj_enable(on);
}

void gnss_aj_enable(int on)
{
	XTime now;
	XTime_GetTime(&now);
	if(on) {
		uint32_t ver = gnss_pt_read(GNSS_PT_REG_VERSION);
		if(ver < 0x00040001U) {
			console_print("gnss_aj: the FPGA reports gnss_passthrough %d.%d; SMI-PI v4.1 needs 4.1\n"
				      "         (bypass, RX2 normalisation and image taps). Anti-jam NOT started:\n"
				      "         program the BOOT.BIN / bitstream built from this source.\n",
				      (long)(ver >> 16), (long)(ver & 0xFFFFU));
			return;
		}
		{
			/* The reference tap's delay is compiled into BOTH the core and this firmware (AJ_REF); a
			 * mismatch would put every weight on the wrong tap. */
			uint32_t info = gnss_pt_read(GNSS_PT_REG_AJ_INFO);
			if(((info >> 24) & 0xFFU) != (uint32_t)AJ_NT) {
				console_print("gnss_aj: the bitstream has %d taps per channel, this firmware needs %d (AJ_INFO 0x%x).\n"
					      "         Anti-jam NOT started: rebuild the WHOLE project (./build.sh reset && ./build.sh all) so the\n"
					      "         bitstream and the firmware match.\n",
					      (long)((info >> 24) & 0xFFU), (long)AJ_NT, (unsigned long)info);
				return;
			}
			if((info & 0xFFU) != GNSS_PT_AJ_LAYOUT_RAMP) {
				console_print("gnss_aj: the bitstream's AJ_INFO layout is %d, this firmware (crpa8) needs %d: weight\n"
					      "         smoothing, AJ_GAIN and the clip counters (AJ_INFO 0x%x). Anti-jam NOT started:\n"
					      "         rebuild the WHOLE project (./build.sh reset, then the full build) so they match.\n",
					      (long)(info & 0xFFU), (long)GNSS_PT_AJ_LAYOUT_RAMP, (unsigned long)info);
				return;
			}
			if(((info >> 16) & 0xFFU) != (uint32_t)AJ_REF) {
				console_print("gnss_aj: the bitstream's reference tap is at delay %d, this firmware needs %d (AJ_INFO 0x%x).\n"
					      "         Anti-jam NOT started: rebuild the WHOLE project (./build.sh reset && ./build.sh all) so the\n"
					      "         bitstream and the firmware match.\n",
					      (long)((info >> 16) & 0xFFU), (long)AJ_REF, (unsigned long)info);
				return;
			}
		}
		/* The weights assume both channels always have the same gain: two
		 * independent AD9361 AGCs would step them apart. A manual gain the
		 * operator locked (gnss_rx_agc=0) is matched too and is kept. */
		/* 4.9.13: the RX gain and TX attenuation are the band's FIXED values (gnss_band=N). The matched loop
		 * (gnss_rx_agc=2) moved both with the jammer level; it is only used when the operator asks for it. */
		if(gnss_rxg_mode() == GNSS_RXG_AD9361)
			gnss_rxg_set_mode(GNSS_RXG_MANUAL);
		if(!aj_on) {
			state = ST_CLEAR;
			on_cnt = 0;
			clear_t0 = now;
			stats_reset();
			res.sig2_prev = -1.0;
			commit_bypass();                 /* TX1 = RX1 until a jammer */
			add_event(now, EV_START, 0.0, 0.0, 0.0);
		}
		aj_on = 1;
		gnss_pt_set_smi(1);
	} else {
		gnss_aj_dma_release();
		aj_on = 0;
		gnss_rxg_stop_tx_hold();
		gnss_rxg_set_digital_db(0);
		gnss_pt_set_smi(0);
		if(state == ST_JAMMED)
			console_print("[AJ] anti-jam switched off (jammer state no longer monitored)\n");
		state = ST_CLEAR;
		shift = 0;
	}
	/* The enable crosses into the sample domain and back; wait (<= ~1 ms)
	 * until STATUS confirms it, so a status print straight after is right. */
	{
		uint32_t want = on ? GNSS_PT_ST_SMI_EN_SYNCED : 0U;
		int n;
		for(n = 0; n < 10000; n++)
			if((gnss_pt_read(GNSS_PT_REG_STATUS) & GNSS_PT_ST_SMI_EN_SYNCED) == want)
				break;
	}
}

int  gnss_aj_enabled(void)   { return aj_on; }
int  gnss_aj_user_off(void)  { return aj_user_off; }
int  gnss_aj_jammer_on(void) { return state == ST_JAMMED; }

void gnss_aj_freeze(int on)  { aj_frozen = on ? 1 : 0; }

void gnss_aj_set_mode(int mode)
{
	if(mode < 0 || mode >= AJ_NMODES) return;
	aj_mode = mode;
	stats_reset();                   /* the old taps are not comparable */
}

void gnss_aj_set_sweep(int on)
{
	sweep_en = on ? 1 : 0;
	aj_sweep_reset(&sweep);
	sweep_avg = 0.0;
}

int gnss_aj_sweep_active(void) { return sweep_en && sweep_avg > 0.0; }

void gnss_aj_set_events(int on)
{
	aj_events = on ? 1 : 0;
	n_held = 0;
}

void gnss_aj_set_log(int seconds)
{
	if(seconds < 0) seconds = 0;
	if(seconds > 3600) seconds = 3600;
	aj_log_s = seconds;
}

/* 4.9.9: gnss_aj_avg= : manual averaging factor, or auto */
/* ---- crpa8: smoothing of the weights / gain (gnss_aj_ramp=) ------------------------------- */
void gnss_aj_ramp_line(void)
{
	uint32_t fs; uint64_t lo;
	radio(&fs, &lo);
	if(!ramp_hw)
		console_print("smoothing     : not in this bitstream (needs AJ_INFO layout 10)\n");
	else if(ramp_k == 0)
		console_print("smoothing     : OFF (gnss_aj_ramp=0): every new weight set acts at once; fine gain +%s dB\n", f2u(fine_db));
	else
		console_print("smoothing     : gnss_aj_ramp=%d: time constant 2^%d = %d samples = %s ms at %s MS/s; fine gain +%s dB\n",
			      (long)ramp_k, (long)(ramp_k + 7), (long)(1L << (ramp_k + 7)),
			      f2u(fs ? (double)(1L << (ramp_k + 7)) * 1000.0 / (double)fs : 0.0), f2u((double)fs / 1e6), f2u(fine_db));
}

int gnss_aj_ramp(void) { return ramp_k; }

void gnss_aj_set_ramp(int k)
{
	if(k < 0) k = 0;
	if(k > 15) k = 15;
	ramp_k = k;
	blk_refresh();                 /* commit the new time constant now (state unchanged) */
	gnss_aj_ramp_line();
}

void gnss_aj_set_avg(double v, int automatic)
{
	MATH_LOCK();
	if(automatic) aj_set_av_auto(); else aj_set_av(v);
	MATH_UNLOCK();
	MATH_LOCK();
	{
		int is_auto = aj_av_is_auto();
		MATH_UNLOCK();
		console_print("GNSS_AJ averaging factor: %s\n", is_auto ? "AUTO (adapted from the solves)" : "MANUAL");
	}
	gnss_aj_avg_print();
}
void gnss_aj_avg_print(void)
{
	int is_auto;
	double av;
	MATH_LOCK();
	is_auto = aj_av_is_auto(); av = aj_av_value();
	MATH_UNLOCK();
	console_print("averaging     : %s, factor %s now (memory about %s solves; 0.5 = 2 solves, 0.9 = 10)\n",
		      is_auto ? "AUTO" : "MANUAL", fmt(av, 3, 0), fmt(1.0 / (1.0 - av), 1, 0));
}

void gnss_aj_set_csv(int ms)
{
	if(ms < 0) ms = 0;
	if(ms > 0 && ms < 100) ms = 100;
	if(ms > 60000) ms = 60000;
	aj_csv_ms = ms;
	csv_hdr = 0;
}

void gnss_aj_note_tx(int on)
{
	XTime now;
	XTime_GetTime(&now);
	tx_noted = on ? 1 : 0;
	tx_t0 = now;
	add_event(now, EV_TX, on ? 1.0 : 0.0, tx_att_db(), 0.0);
	stamp(now);
	console_print("TX1 %s\n", (char*)(on ? "ON: the receiver's time to fix counts from here" : "OFF"));
}

/* 4.9.13: the tap counts come from AJ_NT (the 4.9.12 text still said "63 + 2", the
 * 32-tap layout of 4.9.7, while the filter had 5 + 5 taps). Single caller per print. */
static const char *mode_name(int m)
{
	static char txt[80];
	switch(m) {
	case AJ_MODE_STAP: snprintf(txt, sizeof txt, "space-time %d + 2 image taps (gnss_aj_mode=0)", 2 * AJ_NT - 1); break;
	case AJ_MODE_RX2:  snprintf(txt, sizeof txt, "RX2 %d + 2 image taps (gnss_aj_mode=1)", AJ_NT); break;
	case AJ_MODE_NB:   return "one weight, the old v3 SMI-PI (gnss_aj_mode=2)";
	default:           snprintf(txt, sizeof txt, "space-time %d taps, no image taps (gnss_aj_mode=3)", 2 * AJ_NT - 1); break;
	}
	return txt;
}

static const char *g2_src_txt(void)
{
	return g2_src == 1 ? "from the noise floors" :
	       g2_src == 2 ? "from the jammer (no clean period yet)" : "default, not learned yet";
}

void gnss_aj_print_short(void)
{
	console_print("GNSS_SMI: anti-jam %s, %s%s, jammer %s",
		      (char*)(aj_on ? "ON" : "off"),
		      (char*)(state == ST_JAMMED ? "FILTER IN" : "bypass"),
		      (char*)(aj_frozen ? ", FROZEN" : ""),
		      (char*)(!aj_on ? "not monitored" : state == ST_JAMMED ? "ON" : "none"));
	if(aj_on && res_valid && res_solved)
		console_print(", J/N %s dB, suppression %s dB, residual %s dB", f1(res.jn_db),
			      supp_txt(res.supp_db), f1u(res_disp(res.res_db)));
	if(gnss_pt_read(GNSS_PT_REG_STATUS) & GNSS_PT_ST_SMI_DROP)
		console_print(", SAMPLES DROPPED (STATUS[10])");
	console_print("\n");
}

void gnss_aj_print(void)
{
	XTime now;
	gnss_aj_print_short();
	console_print("  filter      : %s\n", (char*)mode_name(aj_mode));
	if(!res_valid) {
		console_print("  (no capture processed yet%s)\n", (char*)(aj_on ? "" : ": gnss_smi=1 starts it"));
		return;
	}
	XTime_GetTime(&now);
	if(state == ST_JAMMED) {
		console_print("  jammer      : ON for %s s, J/N %s dB (peak %s), ",
			      f1u((double)ms_since(jam_t0, now) / 1000.0), f1(res.jn_db), f1(jam_peak));
		print_type();
		console_print("\n");
		if(res_solved)
			console_print("  suppression : %s dB, residual %s dB above the output noise (0 = fully nulled)%s\n",
				      supp_txt(res.supp_db), f1u(res_disp(res.res_db)),
				      (char*)(res.res_db > AJ_WARN_RES_DB ? "  <-- jammer leaking" : ""));
	} else {
		console_print("  jammer      : none for %s s (coherence %s dB, ON above %s; power J/N %s, ON above %s)\n",
			      f1u((double)ms_since(clear_t0, now) / 1000.0),
			      f1(res.jn_coh_db < -30.0 ? -30.0 : res.jn_coh_db), f1(coh_on()),
			      (char*)(nf0 > 0.0 ? f1(res.jn_db < -9.9 ? -9.9 : res.jn_db) : "(learning)"),
			      f1u(AJ_ON_DB));
	}
	console_print("  more        : gnss_report? prints everything (paste it into a report)\n");
}

void gnss_aj_report(void)
{
	uint32_t fs, bw = 0, info, cfg, ver;
	uint64_t lo;
	XTime now;
	int k, kmax = 0, i, first;
	double wmax = 0.0;

	XTime_GetTime(&now);
	ver  = gnss_pt_read(GNSS_PT_REG_VERSION);
	info = gnss_pt_read(GNSS_PT_REG_AJ_INFO);
	cfg  = gnss_pt_read(GNSS_PT_REG_AJ_CFG_ACTIVE);
	radio(&fs, &lo);
	if(ad9361_phy) ad9361_get_rx_rf_bandwidth(ad9361_phy, &bw);

	console_print("\n===== GNSS-AJ REPORT (copy from here) =====\n");
	console_print("firmware      : SMI-PI %s, bitstream %d.%d, layout 0x%x, uptime %s s\n",
		      (char*)GNSS_AJ_VERSION_STR, (long)(ver >> 16), (long)(ver & 0xFFFFU),
		      (long)info, f1u((double)ms_since(t_boot, now) / 1000.0));
	console_print("radio         : LO %s MHz, %s MS/s, RF bandwidth %s MHz (gnss_band=%d)\n",
		      f3u((double)lo / 1e6), f2u((double)fs / 1e6), f1u((double)bw / 1e6), (long)band);
	console_print("band          : ");
	print_band_cover();
	console_print("\n");
	console_print("tx            : %s, TX1 attenuation %s dB (operator %s dB, level hold +%d dB)",
		      (char*)(transmitting() ? "TRANSMITTING" : "silent"), f2u(tx_att_db()),
		      f2u((double)gnss_rxg_att_base() / 1000.0), (long)gnss_rxg_comp_db());
	if(tx_noted == 1)
		console_print(", on for %s s", f1u((double)ms_since(tx_t0, now) / 1000.0));
	console_print("\n");
	console_print("rx gain       : %d dB both channels (clean reference %d), ADC peak %d of 2047\n",
		      (long)cur_gain(), (long)gnss_rxg_ref(), (long)gnss_rxg_peak());
	if(cut_db() > 0)
		console_print("output level  : %s dB re the clean level into the receiver (RX gain cut %d dB, make-up +%d dB, TX hold +%d dB)\n",
			      f1(-cut_db() + 6 * shift + gnss_rxg_comp_db()), (long)cut_db(), (long)(6 * shift), (long)gnss_rxg_comp_db());
	console_print("normalisation : g2 = %s (%s dB) %s; RX2 noise after it = %s x RX1\n",
		      f3u(g2), f1(aj_db10(g2 * g2)), (char*)g2_src_txt(), f2u(rx2n));
	if(nf0 > 0.0)
		console_print("noise floor   : RX1 %s LSB^2 at the present gain (learned)\n", f1u(noise_now()));
	else
		console_print("noise floor   : not learned yet (needs a moment with no jammer: switch the jammer OFF for 10 s, then ON).\n"
			      "                Until then J/N, suppression and residual below are measured against the filter's OWN noise\n"
			      "                estimate, so 'residual 0 dB' does not prove the jammer is gone.\n");
	console_print("anti-jam      : %s, filter %s (cfg 0x%x, make-up shift %d = +%d dB)%s\n",
		      (char*)(aj_on ? "ON" : "OFF"),
		      (char*)((cfg & GNSS_PT_AJ_CFG_BYPASS) ? "BYPASSED (TX1 = RX1)" : "IN"),
		      (long)cfg, (long)((cfg >> 8) & 7U), (long)(6 * ((cfg >> 8) & 7U)),
		      (char*)(aj_frozen ? ", FROZEN" : ""));
	console_print("mode          : %s\n", (char*)mode_name(aj_mode));
	console_print("pulse blanker : %s\n",
		      (char*)(!blk_hw ? "not in this bitstream (needs bitstream 4.6, AJ_INFO layout 5)" :
		              blk_mode == 0 ? "OFF (gnss_aj_blank=0)" :
		              blk_thr_set == GNSS_PT_BLK_THR_OFF ? "armed, waiting for a noise floor (no clean moment yet)" :
		              (blk_mode == 1 && now < blk_off_until) ? "OFF for a while: it was blanking most of the output" :
		              "ON"));
	if(blk_hw && blk_thr_set != GNSS_PT_BLK_THR_OFF)
		console_print("                threshold %lu (8 samples of I^2+Q^2, %s dB over the output noise), blanked %s %% of the last 100 ms,\n"
			      "                %lu samples in %lu bursts in total, %lu jammer-free blocks skipped, pulses %s, hardware peak %d\n",
			      (unsigned long)blk_thr_set, f1(blk_db), f1(100.0 * blk_duty),
			      (unsigned long)gnss_pt_read(GNSS_PT_REG_AJ_BLK_COUNT),
			      (unsigned long)gnss_pt_read(GNSS_PT_REG_AJ_BLK_BURSTS), (unsigned long)n_gate,
			      (char*)(pulsed(now) ? "SEEN" : "not seen"),
			      (long)(gnss_pt_read(GNSS_PT_REG_AJ_RX_PEAK) & 0xFFFFU));
	gnss_aj_ramp_line();
	console_print("adc clipping  : %s (RX1 %d, RX2 %d samples clipped since start; AJ_CLIP1/2)\n",
		      (char*)gnss_rxg_clip_txt(), (long)gnss_pt_read(GNSS_PT_REG_AJ_CLIP1), (long)gnss_pt_read(GNSS_PT_REG_AJ_CLIP2));
	{
		uint32_t sl, sm;
		MATH_LOCK();
		sl = solve_us_last; sm = solve_us_max;
		MATH_UNLOCK();
#ifdef GNSS_AJ_THREAD
		console_print("solve time    : %s ms last, %s ms max per capture (second core; %d captures skipped while busy)\n",
			      f2u((double)sl / 1000.0), f2u((double)sm / 1000.0), (long)n_busy);
#else
		console_print("solve time    : %s ms last, %s ms max per capture\n", f2u((double)sl / 1000.0), f2u((double)sm / 1000.0));
#endif
	}
	console_print("sweep         : %s (gnss_aj_sweep=%d)\n",
		      (char*)(!sweep_en ? "detector OFF" : sweep_avg > 0.0 ? "MOVING JAMMER (chirp): long memory ON, 0.98 = about 50 solves" : "none seen (fixed or fast jammer, memory 0.5)"),
		      sweep_en);
	if(state == ST_JAMMED)
		console_print("state         : JAMMED for %s s (since t = %s s)\n",
			      f1u((double)ms_since(jam_t0, now) / 1000.0),
			      f3u((double)ms_since(t_boot, jam_t0) / 1000.0));
	else
		console_print("state         : CLEAR for %s s\n", f1u((double)ms_since(clear_t0, now) / 1000.0));
	if(res_valid) {
		console_print("detection     : coherence %s at lag %d (%s dB; ON above %s), power J/N %s dB\n",
			      f3u(res.coh), (long)res.coh_lag,
			      f1(res.jn_coh_db < -30.0 ? -30.0 : res.jn_coh_db), f1(coh_on()),
			      f1(res.jn_db < -99.0 ? -99.0 : res.jn_db));
		console_print("powers        : RX1 %s, RX2 %s LSB^2 (DC removed)\n", f1u(res.p1), f1u(res.p2));
		{
			double d1, d2;
			MATH_LOCK(); aj_last_dc(&d1, &d2); MATH_UNLOCK();
			console_print("dc offset     : RX1 %s LSB, RX2 %s LSB (removed before detection)%s\n", f1u(d1), f1u(d2),
				      (char*)((nf0 > 0.0 && d1 * d1 > noise_now()) ?
					      "\n                LARGE: above the noise; AD9361 DC calibration/tracking not removing it" : ""));
		}
		if(coh_bg > -90.0)
			console_print("background    : RX1/RX2 common signal, coherence %s dB: not a jammer (splitter / one source / TX1 leak?)\n", f1(coh_bg));
	}
	if(state == ST_JAMMED && res_solved) {
		console_print("jammer        : J/N %s dB (peak %s), ", f1(res.jn_db), f1(jam_peak));
		print_type();
		console_print("\n");
		console_print("suppression   : %s dB, residual %s dB (0 = fully nulled)%s\n",
			      supp_txt(res.supp_db), f1u(res_disp(res.res_db)),
			      (char*)(res.res_db > AJ_WARN_RES_DB ? "  <-- jammer leaking" : ""));
		for(k = 0; k < AJ_NSPT; k++) {
			double m = ((double)res.wre[k] * res.wre[k] + (double)res.wim[k] * res.wim[k]) / (double)(1 << (2 * res.ws));
			if(m > wmax) { wmax = m; kmax = k; }
		}
		console_print("weights       : |w|^2 %s, largest %s tap at delay %d (|w| %s), image |c1| %s |c2| %s\n",
			      f2u(res.wn2), (char*)(kmax < AJ_NT ? "RX1" : "RX2"), (long)(kmax % AJ_NT),
			      f2u(aj_sqrt(wmax) / 4096.0 / (double)(1 << res.ws)),
			      f3u(aj_sqrt((double)res.wre[AJ_IMG1] * res.wre[AJ_IMG1] + (double)res.wim[AJ_IMG1] * res.wim[AJ_IMG1]) / (4096.0 * (1 << res.ws))),
			      f3u(aj_sqrt((double)res.wre[AJ_IMG2] * res.wre[AJ_IMG2] + (double)res.wim[AJ_IMG2] * res.wim[AJ_IMG2]) / (4096.0 * (1 << res.ws))));
		console_print("solver        : loading %s LSB^2, %d passes, weight scale 2^%d%s\n", f1u(res.sig2), (long)res.iters,
			      (long)res.ws, (char*)(ws_hw ? "" : " (bitstream older than 4.4: off)"));
	}
	if(state == ST_JAMMED && null_ms >= 0.0)
		console_print("time to null  : %s ms (this jammer)\n", fmt(null_ms, 0, 0));
	else if(null_ms_last >= 0.0)
		console_print("time to null  : %s ms (last jammer)\n", fmt(null_ms_last, 0, 0));
	else
		console_print("time to null  : - (no jammer nulled yet)\n");
	console_print("engine        : %d solves/s, %d checks/s; %d solves, %d checks, %d skipped (gain step), %d DMA errors\n",
		      (long)upd_rate, (long)det_rate, (long)n_upd, (long)n_det, (long)n_skip, (long)n_fail);
	if(aj_log_s > 0) console_print("printing      : status every %d s (gnss_aj_log=)", (long)aj_log_s);
	else             console_print("printing      : status line off (gnss_aj_log=30 to see it)");
	if(aj_csv_ms > 0) console_print(", CSV every %d ms (gnss_aj_csv=)\n", (long)aj_csv_ms);
	else              console_print(", CSV off (gnss_aj_csv=)\n");
	console_print("events        : %d in total, the last %d:\n", (long)ev_n, (long)(ev_n < 8 ? ev_n : 8));
	first = ev_n > 8 ? ev_n - 8 : 0;
	for(i = first; i < ev_n; i++) {
		struct aj_event *e = &ev[i % AJ_NEV];
		console_print("  %s s  ", f3u((double)e->ms / 1000.0));
		switch(e->type) {
		case EV_START:  console_print("anti-jam started (bypass)\n"); break;
		case EV_TX:     console_print("TX1 %s, attenuation %s dB\n", (char*)(e->v1 > 0.5 ? "ON" : "OFF"), f2u(e->v2)); break;
		case EV_ON:     console_print("JAMMER ON   J/N %s dB, bandwidth %s of fs, trigger: %s\n",
					      f1(e->v1), f2u(e->v2), (char*)why_txt((int)e->v3)); break;
		case EV_NULLED: console_print("NULLED      after %s ms, suppression %s dB, residual %s dB\n",
					      fmt(e->v1, 0, 0), supp_txt(e->v2), f1u(res_disp(e->v3))); break;
		case EV_OFF:    console_print("JAMMER OFF  after %s s, peak J/N %s dB\n", f1u(e->v1), f1(e->v2)); break;
		case EV_FLOOR:   console_print("FLOOR       noise rose %s dB with no RX1/RX2 coherence: floor re-learned, not a jammer\n", f1(e->v1)); break;
		case EV_BG:
			if(e->v1 > -90.0)
				console_print("BACKGROUND  RX1/RX2 common signal, coherence %s dB at power J/N %s dB: not a jammer\n",
					      f1(e->v1), f1(e->v2));
			else
				console_print("BACKGROUND  gone, normal detection again\n");
			break;
		case EV_LEAK:   console_print("LEAKING     residual %s dB at J/N %s dB\n", f1(e->v1), f1(e->v2)); break;
		default:        console_print("?\n"); break;
		}
	}
	console_print("===== END REPORT =====\n");
}

/* ---- RF profiles ------------------------------------------------------------------ */
/* v4.1: profile 1 (default) is centred on the WHOLE upper L-band GNSS block,
 * BeiDou B1I (1559.05 MHz, lower edge of its main lobe) to GLONASS G1 k=+6
 * (1605.89 MHz): 46.8 MHz wide, centre 1582.5 MHz. Every signal must stay in
 * the flat part of the AD9361's decimation/interpolation filters, about
 * +-0.4 fs, on the way in AND on the way out. At 61.44 MS/s that is
 * +-24.6 MHz: B1I sits at -21.4 MHz, GLONASS k=+6 at +22.9 MHz. The v4.0
 * profile (1584.5 MHz, 50 MS/s) put B1I at -23.4 MHz = 0.94 of Nyquist, in
 * the filters' transition band, attenuated twice (RX and TX) -- that is
 * why BeiDou was not seen -- and the upper GLONASS channels at the edge. */
struct band_cfg { uint64_t lo; uint32_t fs, bw; uint32_t att_mdb; int32_t rxg_db; const char *name; };
/* 4.9.5: seven FIXED presets. Each one sets the LO, the sample rate, the RF bandwidth, the TX1 attenuation and
 * the RX gain (both channels). There is no custom band any more. 4.9.13: the TX column is TX1 ONLY; TX2 is not
 * used and is always at GNSS_L1_TX2_ATTEN_MDB (89.75 dB).
 *            LO MHz     MS/s   BW MHz  TX att dB  RX gain dB */
static struct band_cfg bands[GNSS_BAND_COUNT] = {
	{1569420000ULL, 61440000U, 56000000U, 30000U, 50, "1569.42 MHz, TX1 30 dB, RX 50 dB (default)"},
	{1575420000ULL, 61440000U, 56000000U, 30000U, 50, "1575.42 MHz, TX1 30 dB, RX 50 dB"},
	{1575420000ULL, 61440000U, 56000000U, 20000U, 40, "1575.42 MHz, TX1 20 dB, RX 40 dB"},
	{1575420000ULL, 30220000U, 18000000U, 30000U, 50, "1575.42 MHz, 30.22 MS/s, 18 MHz, TX1 30 dB, RX 50 dB"},
	{1575420000ULL, 61440000U, 56000000U, 10000U, 40, "1575.42 MHz, TX1 10 dB, RX 40 dB"},
	{1582500000ULL, 61440000U, 56000000U, 20000U, 40, "1582.50 MHz, TX1 20 dB, RX 40 dB"},
	{1575420000ULL, 61440000U, 56000000U, 30000U, 50, "1575.42 MHz, TX1 30 dB, RX 50 dB"},
	/* antsdr-fw-patch: 7 = custom, set at run time (gnss_band=7,... / gnss_band7=) */
	{1575420000ULL, 61440000U, 56000000U, 30000U, 50, "custom"},
};
static int band_applied;

static char band7_name[96] = "custom";
void gnss_band_custom_set(uint64_t lo, uint32_t fs, uint32_t bw, uint32_t att_mdb, int32_t rxg_db)
{
	struct band_cfg *c = &bands[GNSS_BAND_CUSTOM];
	c->lo = lo; c->fs = fs; c->bw = bw; c->att_mdb = att_mdb; c->rxg_db = rxg_db;
	/* fmt(): no printf %f on bare metal */
	snprintf(band7_name, sizeof(band7_name), "custom: %s MHz, %s MS/s, %s MHz, TX1 %s dB, RX %ld dB",
		 fmt((double)lo / 1e6, 3, 0), fmt((double)fs / 1e6, 2, 0), fmt((double)bw / 1e6, 1, 0),
		 fmt((double)att_mdb / 1000.0, 2, 0), (long)rxg_db);
	c->name = band7_name;
}

void gnss_band_custom_get(uint64_t *lo, uint32_t *fs, uint32_t *bw, uint32_t *att_mdb, int32_t *rxg_db)
{
	const struct band_cfg *c = &bands[GNSS_BAND_CUSTOM];
	*lo = c->lo; *fs = c->fs; *bw = c->bw; *att_mdb = c->att_mdb; *rxg_db = c->rxg_db;
}
static int radio_manual, manual_warn;   /* the operator set LO / rate / FIR by hand: the guard keeps its hands off */

/* The signals the profiles must carry: centre and half the main lobe. */
struct gsig { const char *name; double f, half; };
static const struct gsig gsigs[] = {
	{"GPS L1 C/A, QZSS, SBAS ", 1575.420, 1.023},
	{"Galileo E1, BeiDou B1C ", 1575.420, 2.046},
	{"BeiDou B1I             ", 1561.098, 2.046},
	{"GLONASS G1 k=-7        ", 1598.0625, 0.511},
	{"GLONASS G1 k=+6        ", 1605.375, 0.511},
};
#define N_GSIG ((int)(sizeof(gsigs) / sizeof(gsigs[0])))

/* 2 = fully in the flat band, 1 = at the filter edge (weaker), 0 = out. */
static int sig_cover(const struct gsig *g, double lo_mhz, double fs_mhz, double bw_mhz)
{
	double off = g->f - lo_mhz, a = off < 0.0 ? -off : off;
	double flat = 0.40 * fs_mhz;
	if(flat > bw_mhz / 2.0) flat = bw_mhz / 2.0;
	if(a + g->half <= flat) return 2;
	if(a <= 0.45 * fs_mhz && a <= bw_mhz / 2.0) return 1;
	return 0;
}

static const char *cover_txt(int c)
{
	return c == 2 ? "OK" : c == 1 ? "EDGE: weaker, some satellites lost" : "OUTSIDE: not received";
}

/* "BeiDou B1I OK, GLONASS OK ..." for the report */
static void print_band_cover(void)
{
	uint32_t fs, bw = 0;
	uint64_t lo;
	int i, c[N_GSIG];
	radio(&fs, &lo);
	if(ad9361_phy) ad9361_get_rx_rf_bandwidth(ad9361_phy, &bw);
	for(i = 0; i < N_GSIG; i++)
		c[i] = sig_cover(&gsigs[i], (double)lo / 1e6, (double)fs / 1e6, (double)bw / 1e6);
	console_print("GPS %s, Galileo/B1C %s, BeiDou B1I %s, GLONASS %s",
		      (char*)(c[0] == 2 ? "OK" : c[0] ? "edge" : "OUT"),
		      (char*)(c[1] == 2 ? "OK" : c[1] ? "edge" : "OUT"),
		      (char*)(c[2] == 2 ? "OK" : c[2] ? "edge" : "OUT"),
		      (char*)((c[3] == 2 && c[4] == 2) ? "OK" : (c[3] || c[4]) ? "edge" : "OUT"));
}

int gnss_band_get(void) { return band; }

static int radio_recal(void)
{
	int r = 0;
	/* The LO and the rate changed after the start-up calibrations: redo the
	 * RF DC offset calibration and keep the BB/RF DC tracking on, or the
	 * receivers' DC offset (LO leakage) stays in the samples. */
	ad9361_phy->bbdc_track_en = true;
	ad9361_phy->rfdc_track_en = true;
	r |= (ad9361_do_calib_run(ad9361_phy, RFDC_CAL, 0) < 0) ? 1 : 0;
	/* The TX quadrature / LO-leakage calibration is only redone by the
	 * driver for LO moves > 100 MHz, not for the rate and bandwidth: a stale
	 * one leaves a TX LO-leakage carrier on the LO frequency, which the GNSS
	 * receiver sees as a CW jammer next to L1. Its test tone goes out of
	 * TX1, so only while TX1 is not retransmitting. */
	if(tx_noted != 1)
		r |= (ad9361_do_calib_run(ad9361_phy, TX_QUAD_CAL, -1) < 0) ? 1 : 0;
	return r;
}

/* The current band's TX1 attenuation and RX gain (both channels, also the auto reference); TX2 fixed at 89.75 dB.
 * Called after TX is on (gnss_tx_always() in command.c): the attenuation must not be lowered before the data path
 * and the safe start are in place. */
/* ---- 4.9.13 level lock: RX1/RX2 gain and TX1 attenuation stay at the band's values, TX2 at 89.75 dB ----
 * Something moved them (the AD9361 AGC, the matched loop, a gain-table change after an LO / bandwidth change,
 * the vendor commands). Once a second the guard reads them back and writes the band's values again if one is
 * off by more than 1 dB (gain) or 0.25 dB (attenuation); each repair is counted and printed (at most every 10 s).
 * An operator command (gnss_rx_gain=, tx1/tx2_attenuation=, radio=) becomes the new target
 * (gnss_levels_adopt()); gnss_rx_agc=1 or 2 switches the gain part of the lock off. */
static int      lvl_on;
static int32_t  lvl_rxg_db;
static uint32_t lvl_att_mdb[2];
static uint32_t lvl_repairs;
static XTime    t_lvl, t_lvl_msg;

void gnss_levels_adopt(void)
{
	uint32_t a0 = 0, a1 = 0;
	int32_t g = 0;
	if(!ad9361_phy) return;
	ad9361_get_tx_attenuation(ad9361_phy, 0, &a0);
	ad9361_get_tx_attenuation(ad9361_phy, 1, &a1);
	ad9361_get_rx_rf_gain(ad9361_phy, 0, &g);
	(void)a1;
	lvl_att_mdb[0] = a0; lvl_att_mdb[1] = GNSS_L1_TX2_ATTEN_MDB; lvl_rxg_db = g;   /* 4.9.13: TX2 never adopted */
	lvl_on = 1;
}

static void levels_guard(void)
{
	uint8_t m1 = 0, m2 = 0;
	uint32_t a[2] = {0, 0};
	int32_t g1 = 0, g2 = 0;
	int bad_g = 0, bad_a = 0, ch;
	XTime now;
	if(!lvl_on || !ad9361_phy) return;
	XTime_GetTime(&now);
	if(now - t_lvl < us(1000000U)) return;
	t_lvl = now;
	if(gnss_rxg_mode() == GNSS_RXG_MANUAL) {
		ad9361_get_rx_gain_control_mode(ad9361_phy, 0, &m1);
		ad9361_get_rx_gain_control_mode(ad9361_phy, 1, &m2);
		ad9361_get_rx_rf_gain(ad9361_phy, 0, &g1);
		ad9361_get_rx_rf_gain(ad9361_phy, 1, &g2);
		if(m1 != RF_GAIN_MGC || m2 != RF_GAIN_MGC || g1 < lvl_rxg_db - 1 || g1 > lvl_rxg_db + 1 ||
		   g2 < lvl_rxg_db - 1 || g2 > lvl_rxg_db + 1) bad_g = 1;
	}
	for(ch = 0; ch < 2; ch++) {
		ad9361_get_tx_attenuation(ad9361_phy, (uint8_t)ch, &a[ch]);
		/* crpa5: with gnss_rx_agc=2 the TX level hold moves TX1 on purpose (gnss_smi_agc.c) */
		if(ch == 0 && gnss_rxg_mode() == GNSS_RXG_MATCHED) continue;
		if(a[ch] + 250U < lvl_att_mdb[ch] || a[ch] > lvl_att_mdb[ch] + 250U) bad_a = 1;
	}
	if(!bad_g && !bad_a) return;
	if(bad_g) gnss_rxg_set_gain(lvl_rxg_db);
	if(bad_a) {
		ad9361_set_tx_attenuation(ad9361_phy, 0, lvl_att_mdb[0]);
		ad9361_set_tx_attenuation(ad9361_phy, 1, lvl_att_mdb[1]);
	}
	lvl_repairs++;
	if(now - t_lvl_msg > us(10000000U) || lvl_repairs == 1U) {
		t_lvl_msg = now;
		stamp(now);
		console_print("LEVEL LOCK: %s%s moved (RX1 %d / RX2 %d dB, TX1 %s / TX2 %s dB) -> back to RX %d dB, TX1 %s dB, TX2 %s dB (repair #%lu)\n",
			      (char*)(bad_g ? "RX gain" : ""), (char*)(bad_a ? (bad_g ? " + TX attenuation" : "TX attenuation") : ""),
			      (long)g1, (long)g2, fmt((double)a[0] / 1000.0, 2, 0), fmt((double)a[1] / 1000.0, 2, 0),
			      (long)lvl_rxg_db, fmt((double)lvl_att_mdb[0] / 1000.0, 2, 0), fmt((double)lvl_att_mdb[1] / 1000.0, 2, 0),
			      (unsigned long)lvl_repairs);
	}
}

uint32_t gnss_levels_repairs(void) { return lvl_repairs; }

void gnss_band_levels(void)
{
	const struct band_cfg *c = &bands[band];
	int r = 0;
	if(!ad9361_phy) return;
	r |= ad9361_set_tx_attenuation(ad9361_phy, 0, c->att_mdb);                /* the band's value: TX1 only */
	r |= ad9361_set_tx_attenuation(ad9361_phy, 1, GNSS_L1_TX2_ATTEN_MDB);      /* 4.9.13: TX2 always 89.75 dB */
	gnss_rxg_set_gain(c->rxg_db);
	lvl_att_mdb[0] = c->att_mdb;
	lvl_att_mdb[1] = GNSS_L1_TX2_ATTEN_MDB;
	lvl_rxg_db = c->rxg_db;
	lvl_on = 1;
	console_print("GNSS_BAND %d levels: TX1 attenuation %s dB, TX2 %s dB (fixed, not used), RX1/RX2 gain %d dB%s\n", (long)band,
		      fmt((double)c->att_mdb / 1000.0, 2, 0), fmt((double)GNSS_L1_TX2_ATTEN_MDB / 1000.0, 2, 0),
		      (long)c->rxg_db, (char*)(r ? "  (the AD9361 refused part of it)" : ""));
}

int gnss_band_set(int b)
{
	const struct band_cfg *c;
	int r = 0;
	if(b < 0 || b >= GNSS_BAND_COUNT || !ad9361_phy) return -1;
	c = &bands[b];
	if(aj_on && state == ST_JAMMED)
		/* crpa5: the noise floor is learned again after the band change; with the jammer on
		 * it would be learned WITH the jammer, which is then taken for background. */
		console_print("gnss_band: WARNING a jammer is on. The noise floor is learned again for the new band;\n"
			      "           switch the jammer OFF for about 10 s after this, then on again, or the\n"
			      "           jammer may be learned as background and not nulled.\n");
	gnss_aj_dma_release();
	/* The vendor start-up code LOADS a band-pass FIR (passband 0.15-0.25 fs)
	 * into both the RX and the TX filter. It is bypassed by default and must
	 * stay bypassed: switched on (rx_fir_en=1 / tx_fir_en=1, or a GUI's FIR
	 * box) it keeps only 9-15 MHz from the LO and removes BeiDou, most of
	 * GLONASS and half of the rest. */
	r |= ad9361_set_rx_fir_en_dis(ad9361_phy, 0);
	r |= ad9361_set_tx_fir_en_dis(ad9361_phy, 0);
	r |= ad9361_set_rx_lo_freq(ad9361_phy, c->lo);
	r |= ad9361_set_tx_lo_freq(ad9361_phy, c->lo);
	r |= ad9361_set_rx_sampling_freq(ad9361_phy, c->fs);
	r |= ad9361_set_tx_sampling_freq(ad9361_phy, c->fs);
	r |= ad9361_set_rx_rf_bandwidth(ad9361_phy, c->bw);
	r |= ad9361_set_tx_rf_bandwidth(ad9361_phy, c->bw);
	r |= radio_recal();
	band = b;
	band_applied = 1;
	radio_manual = 0; manual_warn = 0;
	/* noise power at the ADC depends on the bandwidth: learn it again */
	nf0 = -1.0;
	rho_n_re = rho_n_im = 0.0;
	ratio12 = -1.0; g2 = 1.0; rx2n = 1.0; g2_src = 0;
	res.sig2_prev = -1.0;
	stats_reset();
	return r ? -1 : 0;
}

/* Called by rx_lo_freq=, tx_lo_freq=, rx_samp_freq=, tx_samp_freq=, rx_fir_en=,
 * tx_fir_en=: the operator is changing the radio by hand. The radio guard then
 * stops putting the gnss_band= profile back (until the next gnss_band=), and the
 * things that depend on the LO / rate / bandwidth are learned again. */
void gnss_aj_radio_manual(void)
{
	XTime now;
	XTime_GetTime(&now);
	if(!radio_manual) {
		stamp(now);
		console_print("radio set by hand: kept (the radio guard is off until gnss_band=N). RX and TX\n"
			      "     must have the same LO and rate or the receiver sees no satellite.\n");
	}
	radio_manual = 1; manual_warn = 0;
	nf0 = -1.0;
	rho_n_re = rho_n_im = 0.0;
	ratio12 = -1.0; g2 = 1.0; rx2n = 1.0; g2_src = 0;
	res.sig2_prev = -1.0;
	stats_reset();
}

/* ---- radio= / radio?: every value filled in by hand ----------------------------------- */
char *gnss_fmt(double v, int dec) { return fmt(v, dec, 0); }   /* for command.c */
#define f4u(v)  fmt((v), 4, 0)
#define f6u(v)  fmt((v), 6, 0)

/* Units are guessed from the size of the number, so both "1575.42" and
 * "1575420000" work. Return 0 = ok, -1 = outside what the AD9361 can do. */
int gnss_radio_lo_hz(double v, uint64_t *hz)         /* < 100000 = MHz, else Hz */
{
	if(!(v > 0.0)) return -1;
	double h = (double)(uint64_t)(((v < 100000.0) ? v * 1e6 : v) + 0.5);
	if(h < 70.0e6 || h > 6000.0e6) return -1;
	*hz = (uint64_t)h;
	return 0;
}

int gnss_radio_rate_hz(double v, uint32_t *hz)       /* < 1000 = MS/s, else Hz */
{
	if(!(v > 0.0)) return -1;
	double h = (double)(uint32_t)(((v < 1000.0) ? v * 1e6 : v) + 0.5);
	if(h < 2083333.0 || h > 61440000.0) return -1;
	*hz = (uint32_t)h;
	return 0;
}

int gnss_radio_bw_hz(double v, uint32_t *hz)         /* < 1000 = MHz, else Hz */
{
	if(!(v > 0.0)) return -1;
	double h = (double)(uint32_t)(((v < 1000.0) ? v * 1e6 : v) + 0.5);
	if(h < 200000.0 || h > 56000000.0) return -1;
	*hz = (uint32_t)h;
	return 0;
}

int gnss_radio_att_mdb(double db, uint32_t *mdb)     /* dB, 0.25 dB steps */
{
	if(db < 0.0 || db > 89.75) return -1;
	*mdb = (uint32_t)(db * 4.0 + 0.5) * 250U;
	return 0;
}

void gnss_radio_print(void)
{
	uint64_t rlo = 0, tlo = 0;
	uint32_t rfs = 0, tfs = 0, rbw = 0, tbw = 0, att = 0;
	int32_t g1 = 0, g2 = 0;
	uint8_t m1 = 0, m2 = 0, rf = 0, tf = 0;
	if(!ad9361_phy) return;
	ad9361_get_rx_lo_freq(ad9361_phy, &rlo);
	ad9361_get_tx_lo_freq(ad9361_phy, &tlo);
	ad9361_get_rx_sampling_freq(ad9361_phy, &rfs);
	ad9361_get_tx_sampling_freq(ad9361_phy, &tfs);
	ad9361_get_rx_rf_bandwidth(ad9361_phy, &rbw);
	ad9361_get_tx_rf_bandwidth(ad9361_phy, &tbw);
	ad9361_get_tx_attenuation(ad9361_phy, 0, &att);
	ad9361_get_rx_rf_gain(ad9361_phy, 0, &g1);
	ad9361_get_rx_rf_gain(ad9361_phy, 1, &g2);
	ad9361_get_rx_gain_control_mode(ad9361_phy, 0, &m1);
	ad9361_get_rx_gain_control_mode(ad9361_phy, 1, &m2);
	ad9361_get_rx_fir_en_dis(ad9361_phy, &rf);
	ad9361_get_tx_fir_en_dis(ad9361_phy, &tf);
	if(radio_manual)
		console_print("RADIO: set by hand (gnss_band=N goes back to a profile)\n");
	else
		console_print("RADIO: profile gnss_band=%d (%s)\n", (long)band, (char*)bands[band].name);
	console_print("                 RX                TX\n");
	console_print("  LO [MHz]       %s   %s\n", f4u((double)rlo / 1e6), f4u((double)tlo / 1e6));
	console_print("  rate [MS/s]    %s   %s\n", f6u((double)rfs / 1e6), f6u((double)tfs / 1e6));
	console_print("  RF bw [MHz]    %s   %s\n", f3u((double)rbw / 1e6), f3u((double)tbw / 1e6));
	console_print("  gain / atten   RX1 %d dB (%s), RX2 %d dB (%s)   TX1 attenuation %s dB\n",
		      (long)g1, (char*)(m1 == RF_GAIN_MGC ? "manual" : "AGC"),
		      (long)g2, (char*)(m2 == RF_GAIN_MGC ? "manual" : "AGC"),
		      f2u((double)att / 1000.0));
	console_print("  AD9361 FIR     RX %s / TX %s\n", (char*)(rf ? "ON (wrong for GNSS!)" : "off"),
		      (char*)(tf ? "ON (wrong for GNSS!)" : "off"));
	console_print("  received       ");
	print_band_cover();
	console_print("\n");
	if((rlo > tlo ? rlo - tlo : tlo - rlo) > 1000ULL || (rfs > tfs ? rfs - tfs : tfs - rfs) > 1000U)
		console_print("  WARNING: RX and TX differ in LO or rate: the receiver sees every satellite shifted.\n");
}

static void radio_usage(void)
{
	console_print("radio=  fills in every value by hand. Units are recognised by size:\n"
		      "  LO < 100000 = MHz, else Hz;  rate and bandwidth < 1000 = MHz / MS/s, else Hz;\n"
		      "  TX attenuation and RX gain in dB (attenuation 0 to 89.75, 0.25 steps; gain 1 to 76).\n"
		      "  radio=<rxLO> <txLO> <rxRate> <txRate> <txAtt dB> <rxGain dB> [rfBandwidth]\n"
		      "  radio=<LO> <rate> <txAtt dB> <rxGain dB> [rfBandwidth]        (same LO and rate on RX and TX)\n"
		      "  example:  radio=1575.42 1575.42 30.72 30.72 70 40 18\n"
		      "  example:  radio=1582.5 61.44 70 40            (bandwidth left out = 91 percent of the rate, max 56 MHz)\n"
		      "  RX and TX share ONE sample rate in the AD9361 (FDD): both rates must be equal.\n"
		      "  radio?  shows what is set now.  gnss_band=N goes back to a profile.\n");
}

int gnss_cmd_rx_lo_hz(uint64_t hz);    /* command.c: LO + the board's RF switch */
int gnss_cmd_tx_lo_hz(uint64_t hz);

void gnss_radio_cmd(double *p, char n)
{
	uint64_t rlo, tlo;
	uint32_t rfs, tfs, rbw, tbw, mdb;
	double gain, bwv = -1.0;
	int r = 0;

	if(!ad9361_phy) { console_print("radio=: the AD9361 is not initialised\n"); return; }
	if(n == 4 || n == 5) {
		if(n == 5) bwv = p[4];
		if(gnss_radio_lo_hz(p[0], &rlo) || gnss_radio_rate_hz(p[1], &rfs)) goto range;
		tlo = rlo; tfs = rfs;
		if(gnss_radio_att_mdb(p[2], &mdb)) goto range;
		gain = p[3];
	} else if(n == 6 || n == 7) {
		if(n == 7) bwv = p[6];
		if(gnss_radio_lo_hz(p[0], &rlo) || gnss_radio_lo_hz(p[1], &tlo) ||
		   gnss_radio_rate_hz(p[2], &rfs) || gnss_radio_rate_hz(p[3], &tfs)) goto range;
		if(gnss_radio_att_mdb(p[4], &mdb)) goto range;
		gain = p[5];
	} else {
		radio_usage();
		return;
	}
	if(gain < 1.0 || gain > 76.0) goto range;
	if(rfs != tfs) {
		console_print("radio=: RX rate %d Hz and TX rate %d Hz differ. The AD9361 has ONE sample rate for\n"
			      "     RX and TX; give the same value twice. Nothing was changed.\n", (long)rfs, (long)tfs);
		return;
	}
	if(bwv < 0.0) {
		double a = 0.91 * (double)rfs;
		rbw = tbw = (uint32_t)(a > 56.0e6 ? 56.0e6 : a);
	} else if(gnss_radio_bw_hz(bwv, &rbw)) {
		goto range;
	} else {
		tbw = rbw;
	}
	if(mdb < 30000U)
		console_print("radio=: TX1 attenuation %s dB is LOUD (about %d dBm out of TX1 when retransmitting).\n"
			      "     Only into a cable and attenuator, never to an antenna.\n",
			      f2u((double)mdb / 1000.0), (long)(7.0 - (double)mdb / 1000.0));

	gnss_aj_dma_release();
	/* the same order as gnss_band_set(): FIRs off, LO, rate, bandwidth */
	r |= ad9361_set_rx_fir_en_dis(ad9361_phy, 0);
	r |= ad9361_set_tx_fir_en_dis(ad9361_phy, 0);
	r |= gnss_cmd_rx_lo_hz(rlo);
	r |= gnss_cmd_tx_lo_hz(tlo);
	r |= ad9361_set_rx_sampling_freq(ad9361_phy, rfs);
	r |= ad9361_set_tx_sampling_freq(ad9361_phy, tfs);
	r |= ad9361_set_rx_rf_bandwidth(ad9361_phy, rbw);
	r |= ad9361_set_tx_rf_bandwidth(ad9361_phy, tbw);
	r |= ad9361_set_tx_attenuation(ad9361_phy, 0, mdb);
	gnss_rxg_set_gain((int32_t)(gain + 0.5));
	r |= radio_recal();
	gnss_levels_adopt();
	gnss_aj_radio_manual();            /* the guard keeps its hands off; floors are learned again */
	if(r)
		console_print("radio=: the AD9361 refused part of it (see the values below; rate range, LO or bandwidth)\n");
	gnss_radio_print();
	return;
range:
	console_print("radio=: a value is outside the AD9361 range (LO 70 to 6000 MHz, rate 2.083333 to 61.44 MS/s,\n"
		      "     bandwidth 0.2 to 56 MHz, attenuation 0 to 89.75 dB, gain 1 to 76 dB). Nothing was changed.\n");
	radio_usage();
}

void gnss_band_print(void)
{
	uint32_t fs, bw = 0, i;
	uint64_t lo, tlo = 0;
	uint8_t rf = 0, tf = 0;
	radio(&fs, &lo);
	if(ad9361_phy) {
		ad9361_get_rx_rf_bandwidth(ad9361_phy, &bw);
		ad9361_get_tx_lo_freq(ad9361_phy, &tlo);
		ad9361_get_rx_fir_en_dis(ad9361_phy, &rf);
		ad9361_get_tx_fir_en_dis(ad9361_phy, &tf);
	}
	if(radio_manual)
		console_print("GNSS_BAND: set by hand (gnss_band=N goes back to a profile; last profile %d = %s)\n", (long)band, (char*)bands[band].name);
	else
		console_print("GNSS_BAND: %d = %s\n", (long)band, (char*)bands[band].name);
	console_print("  LO RX %s / TX %s MHz, %s MS/s, RF bandwidth %s MHz, AD9361 FIR RX %s / TX %s\n",
		      f3u((double)lo / 1e6), f3u((double)tlo / 1e6), f2u((double)fs / 1e6),
		      f1u((double)bw / 1e6), (char*)(rf ? "ON (wrong!)" : "off"), (char*)(tf ? "ON (wrong!)" : "off"));
	console_print("  signal                    RF MHz     from LO     received?\n");
	for(i = 0; i < (uint32_t)N_GSIG; i++) {
		const struct gsig *g = &gsigs[i];
		console_print("  %s  %s  %s MHz  %s\n", (char*)g->name, f3u(g->f),
			      f1(g->f - (double)lo / 1e6),
			      (char*)cover_txt(sig_cover(g, (double)lo / 1e6, (double)fs / 1e6, (double)bw / 1e6)));
	}
	console_print("  profiles (gnss_band=N), LO / rate / RF bandwidth / TX1 attenuation / RX gain (TX2 always 89.75 dB):\n");
	for(i = 0; i < GNSS_BAND_COUNT; i++)
		console_print("    %d = %s MHz, %s MS/s, %s MHz, %s dB, %d dB%s\n", (long)i, f3u((double)bands[i].lo / 1e6),
			      f2u((double)bands[i].fs / 1e6), f1u((double)bands[i].bw / 1e6),
			      fmt((double)bands[i].att_mdb / 1000.0, 0, 0), (long)bands[i].rxg_db,
			      (char*)(i == GNSS_BAND_DEFAULT ? "  (default)" : i == GNSS_BAND_CUSTOM ? "  (custom: gnss_band=7,LO,rate,BW,att,gain)" : ""));
	console_print("  Your receiver must also have the constellation switched ON (e.g. BeiDou is\n"
		      "  off by default in many receivers; some track only 3 systems at once).\n");
}

/* Every second: the radio must still be what gnss_band= set. A GUI or the
 * vendor commands (rx_lo_freq=, rx_samp_freq=, rx_fir_en=, ...) can change
 * one side only; with RX and TX on different LOs or rates the receiver gets
 * every satellite shifted and sees none, and the FIR removes most of the
 * band. Put the profile back and say so -- unless the operator set the radio
 * by hand with rx_lo_freq= / tx_lo_freq= / rx_samp_freq= / tx_samp_freq= /
 * rx_fir_en= / tx_fir_en= (radio_manual): then it is left alone. */
static XTime t_guard;
static void radio_guard(void)
{
	const struct band_cfg *c = &bands[band];
	uint64_t rlo = 0, tlo = 0;
	uint32_t rfs = 0, tfs = 0;
	uint8_t rf = 0, tf = 0;
	XTime now;
	int64_t d1, d2, d3, d4;

	if(!band_applied || !ad9361_phy || gnss_txdma_is_running()) return;
	XTime_GetTime(&now);
	if(now - t_guard < us(1000000U)) return;
	t_guard = now;
	ad9361_get_rx_fir_en_dis(ad9361_phy, &rf);
	ad9361_get_tx_fir_en_dis(ad9361_phy, &tf);
	ad9361_get_rx_lo_freq(ad9361_phy, &rlo);
	ad9361_get_tx_lo_freq(ad9361_phy, &tlo);
	ad9361_get_rx_sampling_freq(ad9361_phy, &rfs);
	ad9361_get_tx_sampling_freq(ad9361_phy, &tfs);
	d1 = (int64_t)rlo - (int64_t)c->lo;  d2 = (int64_t)tlo - (int64_t)c->lo;
	d3 = (int64_t)rfs - (int64_t)c->fs;  d4 = (int64_t)tfs - (int64_t)c->fs;
	if(radio_manual) {
		/* by hand: never restored; only warn (once) if RX and TX disagree */
		int mism = (rlo > tlo ? rlo - tlo : tlo - rlo) > 1000ULL || (rfs > tfs ? rfs - tfs : tfs - rfs) > 1000U;
		if(mism && !manual_warn) {
			stamp(now);
			console_print("WARNING radio by hand: RX LO %s / TX LO %s MHz, RX %s / TX %s MS/s differ. The\n"
				      "     receiver needs the same LO and rate on both sides (set both, or gnss_band=N).\n",
				      f3u((double)rlo / 1e6), f3u((double)tlo / 1e6), f2u((double)rfs / 1e6), f2u((double)tfs / 1e6));
		}
		manual_warn = mism;
		return;
	}
	if(!rf && !tf && d1 > -1000 && d1 < 1000 && d2 > -1000 && d2 < 1000 &&
	   d3 > -1000 && d3 < 1000 && d4 > -1000 && d4 < 1000)
		return;
	stamp(now);
	console_print("radio changed outside gnss_band= (LO RX %s TX %s MHz, RX %s TX %s MS/s,\n"
		      "     FIR RX %s TX %s) -> profile %d restored. Use gnss_band= to change the band.\n",
		      f3u((double)rlo / 1e6), f3u((double)tlo / 1e6), f2u((double)rfs / 1e6),
		      f2u((double)tfs / 1e6), (char*)(rf ? "ON" : "off"), (char*)(tf ? "ON" : "off"), (long)band);
	gnss_band_set(band);
}

/* ---- the console idle hook ------------------------------------------------------- */
void gnss_idle(void)
{
	gnss_rxg_poll();
	gnss_aj_poll();
	radio_guard();
	levels_guard();
	gnss_log_poll();
}
