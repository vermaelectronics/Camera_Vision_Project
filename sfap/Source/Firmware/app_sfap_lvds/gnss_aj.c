/******************************************************************************
 *  gnss_aj.c  --  SFAP-PI v6.3 anti-jam engine. See gnss_aj.h.
 *  v6.3: pulse blanking of every capture (blank_capture) with the same
 *  threshold committed to the FPGA, pulses as a detection reason, bin
 *  excision mask committed with the weights, blanker threshold rescaled on
 *  every RX gain step (gnss_aj_gain_stepped).
 *****************************************************************************/

#include "gnss_aj.h"
#include "gnss_aj_math.h"
#include "gnss_smi_agc.h"
#include "gnss_passthrough.h"
#include "gnss_txdma.h"
#include "ad9361_api.h"
#include "axi_dmac.h"
#include "console.h"
#include "xtime_l.h"
#include "xil_cache.h"
#include <string.h>

extern struct ad9361_rf_phy *ad9361_phy;
extern struct axi_dmac *rx_dmac;

#define AJ_BYTES          (AJ_NSAMP * 8U)        /* 2 channels x 4 bytes        */
#define AJ_MIN_PERIOD_US  2000U                  /* at most ~500 captures/s     */
#define AJ_DMA_TIMEOUT_US 50000U

/* CLEAR -> JAMMED: any one of these, AJ_ON_COUNT captures in a row */
#define AJ_ON_COH_DB      (-3.0)  /* RX1/RX2 coherence J/N (noise alone: ~-15)  */
#define AJ_ON_DB          6.0     /* power J/N over the learned noise floor     */
#define AJ_CUT_DB         3       /* RX gain cut by the matched gain loop       */
#define AJ_ON_COUNT       2
/* JAMMED -> CLEAR (bypass): all of these for AJ_OFF_US */
#define AJ_OFF_COH_DB     (-6.0)
/* v6.1 per-bin (narrowband) detector: ON when one bin's J/N reaches
 * AJ_ON_BIN_DB, quiet only while every bin is under AJ_OFF_BIN_DB. A CW far
 * below the broadband thresholds (coherence -3 dB, power +6 dB) still hurts
 * GPS C/A, whose code spectrum has lines. With the filter in and no jammer
 * the cost is ~0.3 dB, so a false start is cheap. */
#define AJ_ON_BIN_DB      8.0
#define AJ_OFF_BIN_DB     5.0
/* v6.2 drift-aware statistics averaging: NONE (this capture only) as soon
 * as the residual rises by AJ_AVG_RISE_DB or is above AJ_AVG_TRACK_DB -- the
 * channel has moved (obstacle, reflector, the jammer itself) and old
 * statistics only hold the null back -- else AJ_AVG (0.5). Measured (C-sim,
 * channel turning at 100 deg/s, J/N 40 dB): satellite loss 15.2 dB with
 * v6.1 (25 ms solves, 0.8 while nulled) -> 5.9 dB (v6.2 solves + this). */
/* AJ_AVG_RISE_DB, AJ_AVG_TRACK_DB: gnss_aj_math.h (v6.2.1, shared with the
 * C-sim through aj_avg_policy) */
/* v6.2 g2 for a shadowed RX2: while jammed (J/N above AJ_G2J_JN_DB) and RX2
 * sees the jammer weaker than RX1 (an obstacle between the jammer and the
 * RX2 antenna), g2 rises towards sqrt(P1/P2) so the RX2 weight |b| stays well
 * inside its Q3.12 limit (8). Without it the null fails once RX2 is ~15 dB
 * shadowed. Never above AJ_G2_MAX, and never so high that g2 x RX2 peak
 * saturates the core's 17-bit input (AJ_G2J_PEAK). Changes only by more than
 * AJ_G2J_HYST_DB, and the statistics and held weights restart then. */
/* AJ_G2J_*: gnss_aj_math.h (v6.2.1, shared with the C-sim, aj_g2_follow) */
#define AJ_OFF_DB         3.0
#define AJ_OFF_US         500000U
/* learning (noise floor, RX2/RX1 ratio): only this quiet */
#define AJ_QUIET_COH_DB   (-9.0)
#define AJ_NF_RISE_DB_S   0.05    /* learned noise floor may creep up this fast */
#define AJ_G2_ALPHA       0.02    /* RX1/RX2 ratio average, ~50 captures         */
/* AJ_G2_MAX (AJ_NORM is Q3.13), AJ_G2_MIN: gnss_aj_math.h (v6.2.1) */
/* output make-up gain, status */
#define AJ_SHIFT_RES_DB   3.0     /* make-up only while the residual is below   */
#define AJ_NULL_RES_DB    3.0     /* NULLED when the residual first drops below */
#define AJ_WARN_RES_DB    3.0     /* residual above this while jammed ...        */
#define AJ_WARN_US        1000000U /* ... for this long: warn                    */
#define AJ_ANNOUNCE_US    100000U /* JAMMER ON is printed this long after onset,  */
                                  /* when the gain loop has absorbed the jammer: */
                                  /* the first captures can still be clipped     */
#define AJ_NEV            16      /* event history for sfap-pi_report?             */

enum { ST_CLEAR = 0, ST_JAMMED = 1 };

/* ---- v6.3 pulse blanking / bin excision ------------------------------------ */
static int      aj_blank_on = 1, aj_exc_on = 1;
static int      blk_thr;              /* threshold committed to the FPGA [LSB]  */
static int      blk_n, blk_hits;      /* last capture: blanked, over threshold  */
static double   blk_frac;             /* blanked fraction of the last capture   */
static int      blk_pulsed;           /* last capture held pulses               */
static uint32_t blk_caps;             /* captures with pulses since start       */
static XTime    blk_t;                /* last capture with pulses               */
static int      blk_seen;             /* blk_t is valid                         */
/* A 4096-sample capture is 67 us at 61.44 MS/s; a low-rate pulsed source
 * (e.g. 1 kHz) is in only a few % of the captures, so pulses count as present
 * for this long after the last capture that held them. */
#define AJ_PULSE_HOLD_US  100000U
/* the capture the math works on: the DMA buffer, or this copy of it when
 * something had to be blanked (the DMA buffer itself is never written: a
 * dirty cache line written back later could land on the next capture) */
static uint32_t aj_work[2 * AJ_NSAMP];
static const uint32_t *cap = (const uint32_t *)GNSS_AJ_BUF_ADDR;
enum { EV_ON = 1, EV_NULLED, EV_OFF, EV_LEAK, EV_TX, EV_START };

/* ---- state --------------------------------------------------------------- */
static int      aj_on, aj_user_off, aj_frozen, aj_mode = AJ_MODE_SFAP;
static int      aj_log_s = 30, aj_csv_ms = 0, csv_hdr;
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
static int      state, on_cnt, on_why, jam_announced, jam_nulled, shift, on_ev;
static int      n_ep;                 /* solves in this jammed episode (v6.1)   */
static double   res_prev = 99.0;      /* residual of the solve before (v6.2)    */
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
static double   g2_noise = 1.0;       /* v6.2: the noise-floor value of g2     */
static int      g2_jam;               /* v6.2: 1 while g2 follows the jammer   */
static int      g2_src;          /* 0 default (1.0), 1 noise floors, 2 jammer */

static int      band = GNSS_BAND_DEFAULT;

static void print_band_cover(void);       /* RF profiles section, below */

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

/* RX gain as linear power, the scale of the learned floors (v6.1). */
static double gain_lin(void) { return db2lin((double)cur_gain()); }

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
	shift = 0;
	gnss_rxg_set_digital_db(0);
	gnss_pt_write_aj(0, 0, 0, GNSS_PT_AJ_CFG_BYPASS, g2_q13());
}

/* v6.3: cfg with the blanker threshold (filter in) */
static uint32_t cfg_in(void)
{
	return GNSS_PT_AJ_CFG_SHIFT(shift) | GNSS_PT_AJ_CFG_BLANK(aj_blank_on ? blk_thr : 0);
}

static void commit_weights(void)
{
	gnss_rxg_set_digital_db(6 * shift);
	gnss_pt_write_aj(res.wre, res.wim, aj_exc_on ? res.exc_mask : 0, cfg_in(), g2_q13());
}

/* v6.3: blank this capture as the FPGA blanks the stream. The threshold is
 * also the one committed with the next weights. More than AJ_BLK_MAX_FRAC
 * over it means the signal itself is not pulsed: no blanking. */
static void blank_capture(void)
{
	const uint32_t *raw = (const uint32_t *)GNSS_AJ_BUF_ADDR;
	int thr, n, h = 0;
	cap = raw;
	blk_n = 0; blk_hits = 0; blk_frac = 0.0; blk_pulsed = 0;
	if(!aj_blank_on) { blk_thr = 0; return; }
	thr = aj_blank_thr(raw, AJ_NSAMP);
	n = aj_blank_buf((uint32_t *)raw, AJ_NSAMP, thr, 0, &h);      /* count only */
	if(n > (int)(AJ_BLK_MAX_FRAC * AJ_NSAMP)) { thr = 0; n = 0; h = 0; }
	blk_thr = thr;
	if(n > 0) {
		memcpy(aj_work, raw, sizeof(aj_work));
		aj_blank_buf(aj_work, AJ_NSAMP, thr, 1, 0);
		cap = aj_work;
	}
	blk_n = n;
	blk_hits = h;
	blk_frac = (double)n / (double)AJ_NSAMP;
	blk_pulsed = (h >= AJ_BLK_ON_HITS);
	if(blk_pulsed) {
		blk_caps++;
		XTime_GetTime(&blk_t);
		blk_seen = 1;
	}
}

static int pulses_recent(XTime now)
{
	return aj_blank_on && blk_seen && (now - blk_t < us(AJ_PULSE_HOLD_US));
}

/* the gain loop only leaves pulses out of its peak while the blanker is
 * really removing them from TX1: filter in, blanker on */
static void agc_blank(void)
{
	gnss_rxg_set_blank_thr((state == ST_JAMMED && aj_blank_on && !aj_frozen) ? blk_thr : 0);
}

void gnss_aj_gain_stepped(int delta_db)
{
	double t;
	int i, n;
	if(state != ST_JAMMED || !aj_blank_on || aj_frozen || blk_thr <= 0) return;
	/* 10^(delta/20) without libm: 1.122 per dB */
	t = (double)blk_thr;
	n = delta_db < 0 ? -delta_db : delta_db;
	for(i = 0; i < n; i++) t = (delta_db < 0) ? t / 1.12202 : t * 1.12202;
	blk_thr = (int)(t + 0.5);
	if(blk_thr < AJ_BLK_MIN) blk_thr = AJ_BLK_MIN;
	if(blk_thr > 2047) blk_thr = 2047;
	/* cfg only: no weight RAM writes, so the running weights and mask stay */
	gnss_pt_write_aj(0, 0, 0, cfg_in(), g2_q13());
	agc_blank();
}

void gnss_aj_set_blank(int on)
{
	aj_blank_on = on ? 1 : 0;
	if(!aj_blank_on) { blk_thr = 0; blk_pulsed = 0; }
	if(aj_on && state == ST_JAMMED && !aj_frozen)
		gnss_pt_write_aj(0, 0, 0, cfg_in(), g2_q13());
	agc_blank();
}
int  gnss_aj_get_blank(void)  { return aj_blank_on; }
void gnss_aj_set_excise(int on) { aj_exc_on = on ? 1 : 0; }   /* next solve */
int  gnss_aj_get_excise(void) { return aj_exc_on; }

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
	case 4:  return "one frequency bin far above its learned noise floor (narrowband)";
	case 5:  return "pulses far above the noise (blanked, v6.3)";
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
		if(blk_pulsed) console_print(" + PULSES (%s %% blanked)", f1u(100.0 * blk_frac));
		if(aj_exc_on && res.exc_bins > 0) console_print(" | %d bins excised", (long)res.exc_bins);
		console_print(" | RX %d/%d dB  make-up +%d dB  TX att %s dB | %d solves/s%s\n",
			      (long)cur_gain(), (long)gnss_rxg_ref(), (long)(6 * shift),
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
			      "makeup_db,tx_att_db,g2_db,rate_per_s,bw_mhz,centre_mhz,blank_pct,exc_bins\n");
	}
	radio(&fs, &lo);
	console_print("AJCSV,%s,%s,%s,%s,%s,%s,%d,%d,%d,%s,%s,%d,%s,%s,%s,%d\n",
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
		      f1u(100.0 * blk_frac), (long)((state == ST_JAMMED && aj_exc_on) ? res.exc_bins : 0));
}

/* ---- learning ------------------------------------------------------------------ */
static void update_g2(void)
{
	double g = aj_sqrt(ratio12);
	if(g > AJ_G2_MAX) g = AJ_G2_MAX;
	if(g < AJ_G2_MIN) g = AJ_G2_MIN;
	g2 = (double)((int32_t)(g * 8192.0 + 0.5)) / 8192.0;
	g2_noise = g2;
	g2_jam = 0;
	/* what is left of RX2's noise relative to RX1's after the scaling */
	rx2n = g2 * g2 / ratio12;
}

/* v6.2: before a solve, from the previous solve's powers. Returns 1 if g2
 * changed (the caller restarts the statistics). */
static int g2_follow_jammer(void)
{
	double t;
	/* v6.2.1: the rule itself is aj_g2_follow (gnss_aj_math.c), the same
	 * code the C-sim runs */
	if(!aj_g2_follow(&res, g2, g2_noise, &t)) return 0;
	g2 = t;
	g2_jam = (g2 > g2_noise * 1.0001);
	rx2n = (ratio12 > 0.0) ? g2 * g2 / ratio12 : 1.0;
	return 1;
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
	/* v6.1: the per-bin floor, same rule */
	aj_learn_bins(g, (double)(now - t_learn) / (double)COUNTS_PER_SECOND, AJ_NF_RISE_DB_S);
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

/* ---- the state machine ----------------------------------------------------------- */
static void announce(XTime now)
{
	jam_announced = 1;
	jam_peak = res.jn_db;
	console_print("\n");
	stamp(now);
	console_print("*** JAMMER ON ***  J/N %s dB, ", f1(res.jn_db));
	print_type();
	console_print("\n           detected because %s; RX gain cut %d dB\n",
		      (char*)why_txt(on_why), (long)cut_db());
	/* the ON event was logged at the onset; its figures are the settled ones */
	if(on_ev > ev_n - AJ_NEV) {
		ev[on_ev % AJ_NEV].v1 = res.jn_db;
		ev[on_ev % AJ_NEV].v2 = res.bw;
	}
	if(jam_nulled) {
		stamp(now);
		console_print("NULLED in %s ms: suppression %s dB, residual %s dB, make-up +%d dB\n",
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
	n_ep = 0;
	/* No clean period since start-up (the jammer was there first): g2 from
	 * the jammer itself, which both antennas see. */
	if(g2_src == 0 && res.p2 > AJ_QFLOOR) {
		ratio12 = res.p1 / res.p2;
		update_g2();
		rx2n = 1.0;
		g2_src = 2;
	}
	aj_reset();
	res.sig2_prev = -1.0;
	gain_seen = gnss_rxg_changes();
}

static void leave_jammed(XTime now)
{
	uint32_t ms = ms_since(jam_t0, now);
	state = ST_CLEAR;
	on_cnt = 0;
	clear_t0 = now;
	if(g2_jam) {                                  /* v6.2: back to the noise g2 */
		g2 = g2_noise;
		g2_jam = 0;
		rx2n = (ratio12 > 0.0) ? g2 * g2 / ratio12 : 1.0;
	}
	if(!aj_frozen) commit_bypass();
	aj_reset();
	if(!jam_announced) announce(now);
	console_print("\n");
	stamp(now);
	console_print("jammer OFF after %s s (peak J/N %s dB) -> filter bypassed, TX1 = RX1\n",
		      f1u((double)ms / 1000.0), f1(jam_peak));
	add_event(now, EV_OFF, (double)ms / 1000.0, jam_peak, 0.0);
	if(null_ms >= 0.0) null_ms_last = null_ms;
}

/* CLEAR: detection only. Returns 1 when the jammer is confirmed. */
static int detect_capture(XTime now)
{
	int cut = cut_db(), why = 0, quiet;

	/* v6.3: the blanked samples carry no noise either */
	res.noise_ref = noise_now() * (1.0 - blk_frac);
	res.rho_noise_re = rho_n_re;
	res.rho_noise_im = rho_n_im;
	res.gain_lin = gain_lin();
	if(aj_detect(cap, AJ_NSAMP, &res) != 0 || !res.ok) {
		n_fail++;
		return 0;
	}
	res_valid = 1;
	res_solved = 0;
	n_det++;
	cnt_det++;

	if(res.jn_coh_db >= AJ_ON_COH_DB)              why = 1;
	else if(nf0 > 0.0 && res.jn_db >= AJ_ON_DB)    why = 2;
	else if(cut >= AJ_CUT_DB)                      why = 3;
	else if(res.bin_jn_max >= AJ_ON_BIN_DB)        why = 4;
	else if(pulses_recent(now))                    why = 5;   /* v6.3 */
	quiet = !why && res.jn_coh_db < AJ_QUIET_COH_DB && res.bin_jn_max < AJ_OFF_BIN_DB &&
	        (nf0 <= 0.0 || res.jn_db < AJ_OFF_DB) && cut == 0 && blk_hits == 0;
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
		if(jam_announced) {
			stamp(now);
			console_print("NULLED in %s ms: suppression %s dB, residual %s dB\n",
				      fmt(null_ms, 0, 0), supp_txt(res.supp_db), f1u(res_disp(res.res_db)));
		}
		add_event(now, EV_NULLED, null_ms, res.supp_db, res.res_db);
	}
	if(!jam_announced && now - jam_t0 >= us(AJ_ANNOUNCE_US))
		announce(now);
	if(jam_announced && res.jn_db > jam_peak) jam_peak = res.jn_db;

	quiet = res.jn_coh_db < AJ_OFF_COH_DB && res.jn_db < AJ_OFF_DB && cut_db() == 0 &&
	        res.bin_jn_max < AJ_OFF_BIN_DB && !pulses_recent(now);
	if(!quiet) {
		quiet_t0 = now;
	} else if(now - quiet_t0 >= us(AJ_OFF_US)) {
		leave_jammed(now);
		return;
	}

	if(res.res_db <= AJ_WARN_RES_DB)
		res_bad_t0 = now;
	if(jam_announced && now - res_bad_t0 >= us(AJ_WARN_US) &&
	   now - t_warn >= us(5000000U)) {
		t_warn = now;
		stamp(now);
		console_print("WARNING residual %s dB: the jammer is not fully nulled.\n"
			      "     Two antennas null ONE direction (plus its reflections). Check that\n"
			      "     RX2 has its antenna, that there is a single jammer, and the ADC peak\n"
			      "     (gnss_rx_gain?).%s\n", f1(res.res_db),
			      (char*)(aj_mode == AJ_MODE_SFAP ? "" : " sfap-pi_mode=0 adds the image taps per bin."));
		add_event(now, EV_LEAK, res.res_db, res.jn_db, 0.0);
	}
}

/* JAMMED: solve this capture and commit the weights. */
static void solve_capture(XTime now)
{
	int want;

	if(gnss_rxg_changes() != gain_seen) {
		/* the RX gain stepped since the last solve: the two channels' phase
		 * can step differently, so do not average across it. v6.1: the
		 * held weights stay (aj_reset_stats); fresh bins re-solve anyway. */
		gain_seen = gnss_rxg_changes();
		aj_reset_stats();
		res.sig2_prev = -1.0;
		n_ep = 0;
	}
	/* v6.2: an obstacle shadowing the jammer at RX2 -> raise g2 first */
	if(n_ep >= 1 && g2_follow_jammer()) {
		aj_reset();
		res.sig2_prev = -1.0;
		n_ep = 0;
	}
	res.mode = aj_mode;
	res.excise = aj_exc_on;                                  /* v6.3 */
	res.gain_lin = gain_lin();
	/* v6.2 drift-aware averaging, from the last solves' residuals */
	res.avg = aj_avg_policy(n_ep, res.res_db, res_prev);   /* v6.2.1: shared */
	res_prev = (n_ep > 0) ? res.res_db : 99.0;
	n_ep++;
	res.noise_ref = noise_now() * (1.0 - blk_frac);         /* v6.3 */
	res.rho_noise_re = rho_n_re;
	res.rho_noise_im = rho_n_im;
	res.g2 = g2;
	res.rx2_noise = rx2n;
	if(aj_compute(cap, AJ_NSAMP, &res) != 0 || !res.ok) {
		n_fail++;
		return;
	}
	res.sig2_prev = res.sig2;
	res_valid = res_solved = 1;
	n_upd++;
	cnt_upd++;

	/* Output make-up gain: 6 dB per step of RX gain cut, only while the
	 * jammer is nulled. Down at once, up one step per update. */
	want = (res.res_db <= AJ_SHIFT_RES_DB) ? cut_db() / 6 : 0;
	if(want > 4) want = 4;
	if(want < shift)      shift = want;
	else if(want > shift) shift++;
	if(!aj_frozen)
		commit_weights();
	jammed_state(now);
}

void gnss_aj_poll(void)
{
	XTime now;

	if(!aj_on || !rx_dmac || !ad9361_phy) return;
	if(gnss_txdma_is_running()) return;       /* the DDR replay owns the DMA */
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
	blank_capture();                                         /* v6.3 */

	if(state == ST_CLEAR) {
		/* the capture that confirmed the jammer is solved straight away */
		if(detect_capture(now))
			solve_capture(now);
	} else {
		solve_capture(now);
	}
	agc_blank();

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
	aj_reset();
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
		if(ver < GNSS_PT_EXPECTED_VERSION ||
		   gnss_pt_read(GNSS_PT_REG_AJ_INFO) != GNSS_PT_AJ_INFO_EXPECTED) {
			console_print("sfap-pi: the FPGA reports gnss_passthrough %d.%d, layout 0x%x; SFAP-PI\n"
				      "         v6.3 needs 6.3 / 0x08070C05 (256 bins + pulse blanker + bin excision).\n"
				      "         Anti-jam NOT started: program the BOOT.BIN / bitstream built\n"
				      "         from this source.\n",
				      (long)(ver >> 16), (long)(ver & 0xFFFFU),
				      (long)gnss_pt_read(GNSS_PT_REG_AJ_INFO));
			return;
		}
		/* The weights assume both channels always have the same gain: two
		 * independent AD9361 AGCs would step them apart. A manual gain the
		 * operator locked (gnss_rx_agc=0) is matched too and is kept. */
		if(gnss_rxg_mode() == GNSS_RXG_AD9361)
			gnss_rxg_set_mode(GNSS_RXG_MATCHED);
		if(!aj_on) {
			state = ST_CLEAR;
			on_cnt = 0;
			clear_t0 = now;
			aj_reset();
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
		gnss_rxg_set_blank_thr(0);                /* v6.3: pulses count again */
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
	aj_reset();                   /* the old taps are not comparable */
}

void gnss_aj_set_log(int seconds)
{
	if(seconds < 0) seconds = 0;
	if(seconds > 3600) seconds = 3600;
	aj_log_s = seconds;
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

static const char *mode_name(int m)
{
	switch(m) {
	case AJ_MODE_SFAP: return "SFAP 256 bins, b + 2 image taps per bin (sfap-pi_mode=0)";
	case AJ_MODE_LIN:  return "SFAP 256 bins, b only, no image taps (sfap-pi_mode=1)";
	default:           return "one weight for the whole band, the old v3 SMI-PI (sfap-pi_mode=2)";
	}
}

static const char *g2_src_txt(void)
{
	if(g2_jam) return "RAISED for a jammer weaker at RX2 (shadowed RX2 path)";
	return g2_src == 1 ? "from the noise floors" :
	       g2_src == 2 ? "from the jammer (no clean period yet)" : "default, not learned yet";
}

void gnss_aj_print_short(void)
{
	console_print("SFAP-PI: anti-jam %s, %s%s, jammer %s",
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
		console_print("  (no capture processed yet%s)\n", (char*)(aj_on ? "" : ": sfap-pi=1 starts it"));
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
		if(res_solved && aj_exc_on && res.exc_bins > 0)
			console_print("  excision    : %d bins removed (two jammers from two directions in them)\n",
				      (long)res.exc_bins);
	} else {
		console_print("  jammer      : none for %s s (coherence %s dB, ON above %s; power J/N %s, ON above %s)\n",
			      f1u((double)ms_since(clear_t0, now) / 1000.0),
			      f1(res.jn_coh_db < -30.0 ? -30.0 : res.jn_coh_db), f1(AJ_ON_COH_DB),
			      (char*)(nf0 > 0.0 ? f1(res.jn_db < -9.9 ? -9.9 : res.jn_db) : "(learning)"),
			      f1u(AJ_ON_DB));
	}
	if(aj_blank_on && blk_pulsed)
		console_print("  pulses      : %s %% of the last capture blanked (threshold %d LSB)\n",
			      f1u(100.0 * blk_frac), (long)blk_thr);
	console_print("  more        : sfap-pi_report? prints everything (paste it into a report)\n");
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
	console_print("firmware      : SFAP-PI %s, bitstream %d.%d, layout 0x%x, uptime %s s\n",
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
	console_print("normalisation : g2 = %s (%s dB) %s; RX2 noise after it = %s x RX1\n",
		      f3u(g2), f1(aj_db10(g2 * g2)), (char*)g2_src_txt(), f2u(rx2n));
	if(nf0 > 0.0)
		console_print("noise floor   : RX1 %s LSB^2 at the present gain (learned)\n", f1u(noise_now()));
	else
		console_print("noise floor   : not learned yet (needs a moment with no jammer)\n");
	console_print("anti-jam      : %s, filter %s (cfg 0x%x, make-up shift %d = +%d dB)%s\n",
		      (char*)(aj_on ? "ON" : "OFF"),
		      (char*)((cfg & GNSS_PT_AJ_CFG_BYPASS) ? "BYPASSED (TX1 = RX1)" : "IN"),
		      (long)cfg, (long)((cfg >> 8) & 7U), (long)(6 * ((cfg >> 8) & 7U)),
		      (char*)(aj_frozen ? ", FROZEN" : ""));
	console_print("mode          : %s\n", (char*)mode_name(aj_mode));
	if(state == ST_JAMMED)
		console_print("state         : JAMMED for %s s (since t = %s s)\n",
			      f1u((double)ms_since(jam_t0, now) / 1000.0),
			      f3u((double)ms_since(t_boot, jam_t0) / 1000.0));
	else
		console_print("state         : CLEAR for %s s\n", f1u((double)ms_since(clear_t0, now) / 1000.0));
	if(res_valid) {
		console_print("detection     : coherence %s at lag %d (%s dB; ON above %s), power J/N %s dB\n",
			      f3u(res.coh), (long)res.coh_lag,
			      f1(res.jn_coh_db < -30.0 ? -30.0 : res.jn_coh_db), f1(AJ_ON_COH_DB),
			      f1(res.jn_db < -99.0 ? -99.0 : res.jn_db));
		console_print("powers        : RX1 %s, RX2 %s LSB^2 (raw)\n", f1u(res.p1), f1u(res.p2));
	}
	if(state == ST_JAMMED && res_solved) {
		console_print("jammer        : J/N %s dB (peak %s), ", f1(res.jn_db), f1(jam_peak));
		print_type();
		console_print("\n");
		console_print("suppression   : %s dB, residual %s dB (0 = fully nulled)%s\n",
			      supp_txt(res.supp_db), f1u(res_disp(res.res_db)),
			      (char*)(res.res_db > AJ_WARN_RES_DB ? "  <-- jammer leaking" : ""));
		{
			double c1max = 0.0, c2max = 0.0;
			for(k = 0; k < AJ_N; k++) {
				double m1 = (double)res.wre[AJ_WC1 + k] * res.wre[AJ_WC1 + k] + (double)res.wim[AJ_WC1 + k] * res.wim[AJ_WC1 + k];
				double m2 = (double)res.wre[AJ_WC2 + k] * res.wre[AJ_WC2 + k] + (double)res.wim[AJ_WC2 + k] * res.wim[AJ_WC2 + k];
				if(m1 > c1max) c1max = m1;
				if(m2 > c2max) c2max = m2;
			}
			kmax = res.wmax_bin;
			wmax = res.wmax;
			console_print("weights       : mean |w|^2 %s, largest |b| %s in bin %d (%s MHz), image max |c1| %s |c2| %s\n",
				      f2u(res.wn2), f2u(wmax), (long)kmax,
				      f1((double)(kmax < AJ_N / 2 ? kmax : kmax - AJ_N) * (double)fs / (double)AJ_N / 1e6),
				      f3u(aj_sqrt(c1max) / 4096.0), f3u(aj_sqrt(c2max) / 4096.0));
		}
		console_print("bins          : %d of %d with J/N > 3 dB (%s MHz each), %d on held weights\n",
			      (long)res.jam_bins, (long)AJ_N, f2u((double)fs / (double)AJ_N / 1e6), (long)res.held_bins);
		console_print("solver        : loading %s LSB^2 (mean over bins), up to %d passes\n",
			      f1u(res.sig2), (long)res.iters);
		if(aj_exc_on) {
			int k, first = 1;
			console_print("excision      : %d bins removed (worst bin %d at %s dB over its noise)",
				      (long)res.exc_bins, (long)res.exc_kmax, f1(res.exc_max_db));
			for(k = 0; k < AJ_N; k++)
				if(res.exc_mask[k >> 5] & (1U << (k & 31))) {
					console_print("%s%d", (char*)(first ? ": " : " "), (long)(k < AJ_N / 2 ? k : k - AJ_N));
					first = 0;
				}
			console_print("\n");
		} else {
			console_print("excision      : OFF (sfap-pi_excise=1)\n");
		}
	}
	/* v6.3 pulse blanker, also while clear */
	if(aj_blank_on)
		console_print("pulse blanker : ON, threshold %d LSB, last capture %d over it / %s %% blanked, "
			      "%d captures with pulses so far\n",
			      (long)blk_thr, (long)blk_hits, f1u(100.0 * blk_frac), (long)blk_caps);
	else
		console_print("pulse blanker : OFF (sfap-pi_blank=1)\n");
	if(state == ST_JAMMED && null_ms >= 0.0)
		console_print("time to null  : %s ms (this jammer)\n", fmt(null_ms, 0, 0));
	else if(null_ms_last >= 0.0)
		console_print("time to null  : %s ms (last jammer)\n", fmt(null_ms_last, 0, 0));
	else
		console_print("time to null  : - (no jammer nulled yet)\n");
	console_print("engine        : %d solves/s, %d checks/s; %d solves, %d checks, %d skipped (gain step), %d DMA errors\n",
		      (long)upd_rate, (long)det_rate, (long)n_upd, (long)n_det, (long)n_skip, (long)n_fail);
	console_print("printing      : status every %d s (sfap-pi_log=), CSV every %d ms (sfap-pi_csv=)\n",
		      (long)aj_log_s, (long)aj_csv_ms);
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
/* v6.3a: rxg_db / txatt_mdb: RX gain [dB] (both channels, also the gain
 * loop's reference) and TX1 attenuation [mdB] a profile sets when it is
 * SELECTED (gnss_band=, gnss_band_manual=); 0 = leave it as it is.
 * The 1 s radio guard only restores LO / rate / bandwidth, never the levels,
 * so the matched gain loop and the TX level hold keep working. */
struct band_cfg { uint64_t lo; uint32_t fs, bw; const char *name; int32_t rxg_db; int32_t txatt_mdb; };
static struct band_cfg bands[GNSS_BAND_COUNT] = {      /* [6] gnss_band_custom=, [8] gnss_band_manual= */
	{1575420000ULL, 30720000U, 18000000U, "GPS, Galileo, QZSS, SBAS, BeiDou B1C: L1 only (narrow)"},
	{1582500000ULL, 61440000U, 56000000U, "ALL: GPS, Galileo, QZSS, SBAS, BeiDou B1I + B1C, GLONASS G1"},
	{1589500000ULL, 50000000U, 40000000U, "GPS, Galileo, GLONASS G1 at 50 MS/s (no BeiDou B1I)"},
	{1568250000ULL, 50000000U, 40000000U, "GPS, Galileo, BeiDou B1I + B1C at 50 MS/s (no GLONASS)"},
	/* v6.1: anti-jam profile. 8 MHz analog bandwidth keeps GPS C/A, Galileo
	 * E1 / BeiDou B1C main lobes (+-2.05 MHz) and cuts every jammer more
	 * than ~4 MHz away BEFORE the ADC, so the RX gain loop has to cut less
	 * and the satellites stay above the 12-bit quantisation floor. */
	{1575420000ULL, 30720000U,  8000000U, "GPS L1 C/A, Galileo E1, BeiDou B1C, QZSS, SBAS: ANTI-JAM narrow (8 MHz)"},
	/* v6.2: anti-jam against MULTIPATH. The 256 bins are 60 kHz wide at
	 * 15.36 MS/s, so a reflection of the jammer from another direction is
	 * nulled up to ~4x the delay it can have at 61.44 MS/s (~0.6 us instead
	 * of ~0.16 us, C-sim). The latency of the filter becomes 50 us (a common
	 * delay for every satellite, absorbed by the receiver's clock bias). */
	{1575420000ULL, 15360000U,  6000000U, "GPS L1 C/A, Galileo E1, BeiDou B1C: ANTI-JAM multipath (15.36 MS/s, 6 MHz)"},
	/* v6.2.2: CUSTOM, set by gnss_band_custom=LO_MHz FS_MSPS BW_MHz. Until
	 * then it holds the default profile's values. Not kept over a reboot. */
	{1582500000ULL, 61440000U, 56000000U, "CUSTOM (gnss_band_custom=LO_MHz FS_MSPS BW_MHz)"},
	/* v6.3a: 1568.5 MHz, 30.72 MS/s, 20 MHz RF bandwidth (+-10 MHz:
	 * 1558.5..1578.5 MHz -- BeiDou B1I 1561.098, GPS L1 / Galileo E1 /
	 * BeiDou B1C 1575.42), RX gain 40 dB, TX1 attenuation 30 dB. */
	{1568500000ULL, 30720000U, 20000000U, "1568.5 MHz 20 MHz: BeiDou B1I + GPS L1 / Galileo E1 / B1C (RX 40 dB, TX1 att 30 dB)",
	 40, 30000},
	/* v6.3a: MANUAL preset, start values from gnss_aj.h (GNSS_BAND_MANUAL_*),
	 * changed live by gnss_band_manual=LO FS BW RXG ATT. */
	{GNSS_BAND_MANUAL_LO_HZ, GNSS_BAND_MANUAL_FS_HZ, GNSS_BAND_MANUAL_BW_HZ,
	 "MANUAL (gnss_band_manual=LO_MHz FS_MSPS BW_MHz RXGAIN_dB TX1ATT_dB)",
	 GNSS_BAND_MANUAL_RXGAIN_DB, GNSS_BAND_MANUAL_TXATT_MDB},
};
static int band_applied;

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

int32_t gnss_band_tx_att_mdb(void)
{
	return bands[band].txatt_mdb > 0 ? bands[band].txatt_mdb : 0;
}

/* v6.3a: the RX gain and TX1 attenuation a profile asks for, applied when it
 * is SELECTED. RX gain: both channels and the matched loop's reference
 * (gnss_rx_gain=). TX1 attenuation: only while TX1 is on (gnss_tx=1); with
 * TX1 off it stays at the maximum and gnss_tx=1 starts at the profile's
 * value instead of GNSS_TX_START_ATTEN_MDB. */
static void band_apply_levels(const struct band_cfg *c)
{
	if(!ad9361_phy) return;
	if(c->rxg_db > 0) {
		gnss_rxg_set_gain(c->rxg_db);
		console_print("gnss_band: RX1/RX2 gain %d dB (profile)\n", (long)c->rxg_db);
	}
	if(c->txatt_mdb > 0) {
		if(tx_noted == 1) {
			ad9361_set_tx_attenuation(ad9361_phy, 0, (uint32_t)c->txatt_mdb);
			console_print("gnss_band: TX1 attenuation %d mdB (profile)\n", (long)c->txatt_mdb);
		} else {
			console_print("gnss_band: TX1 is off; gnss_tx=1 will start at %d mdB (profile)\n",
				      (long)c->txatt_mdb);
		}
	}
}

static int band_radio_set(int b);

int gnss_band_set(int b)
{
	int r = band_radio_set(b);
	if(r == 0) band_apply_levels(&bands[b]);
	return r;
}

/* LO, rate, bandwidth only (the 1 s radio guard and the profile switches
 * use this; it never touches the RX gain or the TX1 attenuation). */
static int band_radio_set(int b)
{
	const struct band_cfg *c;
	int r = 0;
	if(b < 0 || b >= GNSS_BAND_COUNT || !ad9361_phy) return -1;
	c = &bands[b];
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
	band = b;
	band_applied = 1;
	/* noise power at the ADC depends on the bandwidth: learn it again */
	nf0 = -1.0;
	aj_forget_bins();
	rho_n_re = rho_n_im = 0.0;
	ratio12 = -1.0; g2 = 1.0; rx2n = 1.0; g2_src = 0; g2_noise = 1.0; g2_jam = 0;
	res.sig2_prev = -1.0;
	aj_reset();
	return r ? -1 : 0;
}

/* v6.2.2: custom profile. Limits: AD9361 LO 70..6000 MHz; rate 2.5..61.44
 * MS/s (61.44 = LVDS maximum; below ~2.1 MS/s the AD9361 needs its FIR, which
 * this design keeps off); RF bandwidth 0.2..56 MHz and not above the rate.
 * The filter core works at any of these rates: its 256 bins are fs/256 wide
 * and its latency is 766 samples. The values the AD9361 actually set are
 * stored, so the 1 s radio guard compares against what the chip can do. */
int gnss_band_set_custom(double lo_mhz, double fs_msps, double bw_mhz)
{
	struct band_cfg old = bands[GNSS_BAND_CUSTOM];
	int prev = band;
	uint64_t lo = 0;
	uint32_t fs = 0, bw = 0;

	if(!ad9361_phy) return -1;
	if(lo_mhz < 70.0 || lo_mhz > 6000.0 || fs_msps < 2.5 || fs_msps > 61.44 ||
	   bw_mhz < 0.2 || bw_mhz > 56.0 || bw_mhz > fs_msps)
		return -1;
	bands[GNSS_BAND_CUSTOM].lo = (uint64_t)(lo_mhz * 1e6 + 0.5);
	bands[GNSS_BAND_CUSTOM].fs = (uint32_t)(fs_msps * 1e6 + 0.5);
	bands[GNSS_BAND_CUSTOM].bw = (uint32_t)(bw_mhz * 1e6 + 0.5);
	if(band_radio_set(GNSS_BAND_CUSTOM) != 0) {
		bands[GNSS_BAND_CUSTOM] = old;
		if(prev == GNSS_BAND_CUSTOM) prev = GNSS_BAND_DEFAULT;
		band_radio_set(prev);
		return -2;
	}
	/* keep what the chip really set (rate and LO are synthesised) */
	ad9361_get_rx_lo_freq(ad9361_phy, &lo);
	ad9361_get_rx_sampling_freq(ad9361_phy, &fs);
	ad9361_get_rx_rf_bandwidth(ad9361_phy, &bw);
	if(lo) bands[GNSS_BAND_CUSTOM].lo = lo;
	if(fs) bands[GNSS_BAND_CUSTOM].fs = fs;
	if(bw) bands[GNSS_BAND_CUSTOM].bw = bw;
	return 0;
}

/* v6.3a: MANUAL preset (gnss_band=8). Same limits as the custom profile, plus
 * RX gain 0 (unchanged) or 1..76 dB and TX1 attenuation 0 (unchanged) or
 * 0.25..89.75 dB. Applied at once; the values are what gnss_band=8 selects
 * until the next reboot. */
int gnss_band_set_manual(double lo_mhz, double fs_msps, double bw_mhz,
                         double rxgain_db, double txatt_db)
{
	struct band_cfg old = bands[GNSS_BAND_MANUAL];
	int prev = band;
	uint64_t lo = 0;
	uint32_t fs = 0, bw = 0;

	if(!ad9361_phy) return -1;
	if(lo_mhz < 70.0 || lo_mhz > 6000.0 || fs_msps < 2.5 || fs_msps > 61.44 ||
	   bw_mhz < 0.2 || bw_mhz > 56.0 || bw_mhz > fs_msps ||
	   rxgain_db < 0.0 || rxgain_db > 76.0 || (rxgain_db > 0.0 && rxgain_db < 1.0) ||
	   txatt_db < 0.0 || txatt_db > 89.75 || (txatt_db > 0.0 && txatt_db < 0.25))
		return -1;
	bands[GNSS_BAND_MANUAL].lo = (uint64_t)(lo_mhz * 1e6 + 0.5);
	bands[GNSS_BAND_MANUAL].fs = (uint32_t)(fs_msps * 1e6 + 0.5);
	bands[GNSS_BAND_MANUAL].bw = (uint32_t)(bw_mhz * 1e6 + 0.5);
	bands[GNSS_BAND_MANUAL].rxg_db = (int32_t)(rxgain_db + 0.5);
	bands[GNSS_BAND_MANUAL].txatt_mdb = (int32_t)(txatt_db * 1000.0 + 0.5);
	if(band_radio_set(GNSS_BAND_MANUAL) != 0) {
		bands[GNSS_BAND_MANUAL] = old;
		if(prev == GNSS_BAND_MANUAL) prev = GNSS_BAND_DEFAULT;
		band_radio_set(prev);
		return -2;
	}
	/* keep what the chip really set (rate and LO are synthesised) */
	ad9361_get_rx_lo_freq(ad9361_phy, &lo);
	ad9361_get_rx_sampling_freq(ad9361_phy, &fs);
	ad9361_get_rx_rf_bandwidth(ad9361_phy, &bw);
	if(lo) bands[GNSS_BAND_MANUAL].lo = lo;
	if(fs) bands[GNSS_BAND_MANUAL].fs = fs;
	if(bw) bands[GNSS_BAND_MANUAL].bw = bw;
	band_apply_levels(&bands[GNSS_BAND_MANUAL]);
	return 0;
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
	console_print("  profiles (gnss_band=N):\n");
	for(i = 0; i < GNSS_BAND_COUNT; i++)
	{
		console_print("    %d = %s MHz, %s MS/s, %s MHz: %s%s\n", (long)i, f3u((double)bands[i].lo / 1e6),
			      f2u((double)bands[i].fs / 1e6), f1u((double)bands[i].bw / 1e6), (char*)bands[i].name,
			      (char*)(i == GNSS_BAND_DEFAULT ? " (default)" : ""));
		if(bands[i].rxg_db > 0 || bands[i].txatt_mdb > 0)
			console_print("        sets RX gain %s, TX1 attenuation %s\n",
				      (char*)(bands[i].rxg_db > 0 ? f1u((double)bands[i].rxg_db) : "unchanged"),
				      (char*)(bands[i].txatt_mdb > 0 ? f2u((double)bands[i].txatt_mdb / 1000.0) : "unchanged"));
	}
	console_print("  custom: gnss_band_custom=LO_MHz FS_MSPS BW_MHz, e.g. gnss_band_custom=1575.42 30.72 18\n");
	console_print("  manual: gnss_band_manual=LO_MHz FS_MSPS BW_MHz RXGAIN_dB TX1ATT_dB (0 = unchanged),\n"
		      "          e.g. gnss_band_manual=1568.5 30.72 20 40 30\n");
	console_print("  Your receiver must also have the constellation switched ON (e.g. BeiDou is\n"
		      "  off by default in many receivers; some track only 3 systems at once).\n");
}

/* Every second: the radio must still be what gnss_band= set. A GUI or the
 * vendor commands (rx_lo_freq=, rx_samp_freq=, rx_fir_en=, ...) can change
 * one side only; with RX and TX on different LOs or rates the receiver gets
 * every satellite shifted and sees none, and the FIR removes most of the
 * band. Put the profile back and say so. */
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
	if(!rf && !tf && d1 > -1000 && d1 < 1000 && d2 > -1000 && d2 < 1000 &&
	   d3 > -1000 && d3 < 1000 && d4 > -1000 && d4 < 1000)
		return;
	stamp(now);
	console_print("radio changed outside gnss_band= (LO RX %s TX %s MHz, RX %s TX %s MS/s,\n"
		      "     FIR RX %s TX %s) -> profile %d restored. Use gnss_band= to change the band.\n",
		      f3u((double)rlo / 1e6), f3u((double)tlo / 1e6), f2u((double)rfs / 1e6),
		      f2u((double)tfs / 1e6), (char*)(rf ? "ON" : "off"), (char*)(tf ? "ON" : "off"), (long)band);
	band_radio_set(band);   /* v6.3a: radio only, the gain loop keeps the levels */
}

/* ---- the console idle hook ------------------------------------------------------- */
void gnss_idle(void)
{
	gnss_rxg_poll();
	gnss_aj_poll();
	radio_guard();
}
