/******************************************************************************
 *  gnss_npi_aj.c  --  PL-NPI v2.3 anti-jam engine. See gnss_npi_aj.h.
 *
 *  The structure (DMA capture, events, report, CSV, RF profiles, radio
 *  guard) is the same as the SMI-PI v4.1 engine (LoopBack_Code_SMI_PI,
 *  gnss_aj.c); what differs is that the weights are adapted by the FPGA, so
 *  this file only switches the core's mode and reads its weights back.
 *****************************************************************************/

#include "gnss_npi_aj.h"
#include "gnss_npi_det.h"
#include "gnss_npi_agc.h"
#include "gnss_passthrough.h"
#include "gnss_txdma.h"
#include "ad9361_api.h"
#include "axi_dmac.h"
#include "console.h"
#include "xtime_l.h"
#include "xil_cache.h"

extern struct ad9361_rf_phy *ad9361_phy;
extern struct axi_dmac *rx_dmac;

#define AJ_BYTES          (NPI_NSAMP * 8U)       /* 2 channels x 4 bytes        */
#define AJ_MIN_PERIOD_US  2000U                  /* at most ~500 captures/s     */
#define AJ_DMA_TIMEOUT_US 50000U

/* firmware detection (jammers the core's power detector cannot see, and the
 * reporting state when the core is not gated): any one of these,
 * AJ_ON_COUNT captures in a row */
#define AJ_ON_COH_DB      (-3.0)  /* RX1/RX2 coherence J/N (noise alone: ~-15)  */
#define AJ_ON_DB          6.0     /* power J/N over the learned noise floor     */
#define AJ_SETTLE_US      2000000U /* gain loop settling after start: a cut is not a jammer */
#define AJ_CUT_DB         3       /* RX gain cut by the matched gain loop       */
#define AJ_ON_COUNT       2
/* ... released when all of these hold for AJ_OFF_US */
#define AJ_OFF_COH_DB     (-6.0)
#define AJ_OFF_DB         3.0
#define AJ_OFF_US         500000U
/* the core's own power detector */
#define AJ_DET_MARGIN     4.0     /* threshold = 4 x noise (6 dB above it)      */
#define AJ_HOLD_US        500000U /* filter stays in this long after the jammer */
/* learning (noise floor, RX2/RX1 ratio): only this quiet */
#define AJ_QUIET_COH_DB   (-9.0)
#define AJ_NF_RISE_DB_S   0.05    /* learned noise floor may creep up this fast */
#define AJ_G2_ALPHA       0.02    /* RX1/RX2 ratio average, ~50 captures         */
#define AJ_G2_MAX         (32767.0 / 8192.0)      /* NPI_NORM is Q3.13          */
#define AJ_G2_MIN         0.125
/* output make-up gain, status */
#define AJ_SHIFT_RES_DB   3.0     /* make-up only while the residual is below   */
#define AJ_NULL_RES_DB    3.0     /* NULLED when the residual first drops below */
#define AJ_WARN_RES_DB    3.0     /* residual above this while jammed ...        */
#define AJ_WARN_US        1000000U /* ... for this long: warn                    */
#define AJ_ANNOUNCE_US    100000U /* JAMMER ON is printed this long after onset,  */
                                  /* when the gain loop has absorbed the jammer  */
#define AJ_NEV            16      /* event history for gnss_report?             */

/* v2.4: JAMMER STATUS, the classification gnss_report prints. Separate from
 * the filter (the FPGA detector and the firmware's forcing switch the
 * filter; they do not decide the status). ON needs evidence in AJ_ST_ON_N
 * consecutive captures (~10 ms); OFF needs every metric quiet, with lower
 * thresholds than ON (hysteresis), for AJ_ST_OFF_US; in between the status
 * holds. UNKNOWN when there is no valid capture for AJ_ST_STALE_US or the
 * anti-jam is off. */
#define AJ_ST_ON_N        4
#define AJ_ST_OFF_US      1000000U
#define AJ_ST_STALE_US    500000U
enum { AJS_UNKNOWN = 0, AJS_OFF = 1, AJS_ON = 2 };

/* v2.4: CW (narrowband) jammers: FFT scan of every AJ_CW_EVERY-th capture;
 * a peak AJ_CW_DET_DB above the median bin in both antennas, seen in
 * AJ_CW_HITS scans, is a confirmed CW: jammer evidence, and a notch in the
 * core (v2.4 bitstream) on its frequency. Dropped AJ_CW_LOST_US after it was
 * last seen. */
#define AJ_CW_EVERY       4
#define AJ_CW_DET_DB      18      /* a candidate counts a hit at this level ...  */
#define AJ_CW_KEEP_DB     14      /* ... a confirmed CW is kept down to this one */
#define AJ_CW_ON_JN_DB    (-15.0) /* a CW counts as a JAMMER from this J/N over the band
                                   * (~J/S 17 dB at C/N0 45). Weaker lines (clock
                                   * spurs such as 1560.000 MHz) cost no C/N0 in the
                                   * simulation: they are listed, not reported ON. */
#define AJ_CW_HITS        3       /* confirmed: 3 hits, in >= half of its scans  */
#define AJ_CW_TRIAL       20      /* a candidate below half after 20 scans goes  */
#define AJ_CW_LOST_US     1000000U
#define AJ_CW_SAME        (3.0 / NPI_CW_N)   /* same CW within 3 bins       */
#define AJ_CW_RETUNE      (1.0 / NPI_CW_N)   /* re-program the notch beyond 1 bin */
/* v2.5 RX2 equaliser (3 taps, fitted on the capture, see gnss_npi_det.c) */
#define AJ_EQ_EVERY       100     /* captures between two fits while a jammer is there (~0.5 s) */
#define AJ_EQ_MIN_JN_DB   3.0     /* the jammer left after the notches must be this far over the noise */
#define AJ_EQ_ON_GAIN_DB  1.5     /* switch it on if 3 taps beat the one complex weight by this much */
#define AJ_EQ_UPD_DB      1.5     /* replace taps in use if the fit beats them by this much */
#define AJ_EQ_MAX_TAP     1.9     /* Q2.14 holds < 2 */
#define AJ_CW_WB_DB       6.0     /* wideband J/N below this + a confirmed CW: report the CW */
#define AJ_NOTCH_R        (1.0 - 1.0 / 256.0) /* pole radius: 62 kHz at 50 MS/s */

enum { EV_ON = 1, EV_NULLED, EV_OFF, EV_LEAK, EV_TX, EV_START, EV_BURST, EV_BG, EV_FLOOR, EV_CW, EV_EQ };

/* ---- state --------------------------------------------------------------- */
static int      aj_on, aj_user_off, aj_frozen, aj_mode = GNSS_AJ_MODE_GATED;
static int      aj_log_s = 0, aj_csv_ms = 0, csv_hdr;
/* Periodic lines (status, CSV) are held back while a command is being typed
 * and for this long after the last key, so they never run through it. */
#define AJ_KEY_QUIET_US   20000000U
static unsigned int keys_seen;
static XTime    t_key;
static int      dma_busy;
static XTime    dma_t0, next_start, t_rate, t_log, t_csv, t_warn, t_learn, t_boot;
static npi_det_t res;
static gnss_pt_npi_snap_t snap;
static int      res_valid, res_meas, core_active;

static uint32_t n_cap, n_fail, n_skip, cnt_cap, cap_rate;
/* firmware detection */
static int      fw_jam, on_cnt, on_why;
static XTime    quiet_t0, trig_t0;
/* the jammer as reported: the core filtering (gated) or fw_jam (always on) */
static int      jam, jam_announced, jam_nulled, jam_hw, shift, on_ev;
static XTime    jam_t0, res_bad_t0, clear_t0;
static double   jam_peak, null_ms = -1.0, null_ms_last = -1.0;
static uint32_t det_cnt_seen, n_burst;
static int      tx_noted = -1;
static XTime    tx_t0;

/* receiver noise floor, normalised to 0 dB RX gain [LSB^2 / linear gain] */
static double   nf0 = -1.0;
static double   rho_n_re, rho_n_im;
/* RX2 input normalisation: g2 (the Q3.13 value in the FPGA), what is left of
 * the RX2/RX1 noise ratio after it, and where g2 came from */
static double   ratio12 = -1.0, g2 = 1.0, rx2n = 1.0;
static int      g2_src;          /* 0 default (1.0), 1 noise floors, 2 jammer */

/* what was last committed to the core */
static uint32_t cfg_w = 0xFFFFFFFFU, thr_w, hold_w;
static int32_t  g2_w = 8192;

static int      band = GNSS_BAND_DEFAULT;

/* v2.4 state */
static int      ajs = AJS_UNKNOWN, ajs_on_n, ajs_why;   /* JAMMER STATUS     */
static XTime    ajs_t0, ajs_quiet_t0, last_valid;
static int      core_v24, core_v25, core_notches, dc_on = 1, notch_auto = 1, gain_init;
static int      eq_auto = 1, eq_on, eq_cnt, eq_nfit;          /* v2.5 RX2 equaliser */
static double   eq_c[3][2], eq_gain_db, eq_jn_db, eq_cur_db;
static uint32_t eq_w[3], eq_wcur[3];
static int      eq_valid;
static double   notch_f[NPI_CW_MAX];            /* RF-baseband frequency programmed in notch k */
static void     eq_reset(void);
struct cw_track { int used, hits, scans, conf; double f, snr1, snr2; XTime seen, t0; };
static struct cw_track cwt[NPI_CW_MAX];
static uint32_t ncoef_w[6], ncoef_cur[6];
static int      ncoef_valid, cw_scan_n;
static uint32_t gsig_mark;                               /* gain signature    */

static void print_band_cover(void);       /* RF profiles section, below */
static void commit_core(int force);
static uint32_t gain_signature(void);
static int  hw_veto;      /* FPGA detector fired on an incoherent noise rise */

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
#define f3(v)   fmt((v), 3, 1)

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
	return nf0 * db2lin((double)cur_gain()) + NPI_QFLOOR;
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

/* ---- the core ------------------------------------------------------------------ */
static int gated(void)    { return aj_mode == GNSS_AJ_MODE_GATED || aj_mode == GNSS_AJ_MODE_GATED_NOIMG; }
/* An AD9361 I/Q mirror is ~40-50 dB down: image weights of ~0.01. Much
 * larger ones are cancelling something else (a DC offset, a signal that is
 * not a mirror) and add noise: the taps are switched off (img_bad) until the
 * mode or the band is set again. */
#define AJ_IMG_MAX        0.1
#define AJ_IMG_COUNT      20
static int img_bad, img_cnt;
static int image_on(void) { return !img_bad && (aj_mode == GNSS_AJ_MODE_GATED || aj_mode == GNSS_AJ_MODE_ALWAYS); }

/* The core's detector compares |x1|^2 + |x2|^2 (raw RX2) with the threshold:
 * 6 dB above N1 + N2 at the present gain. Never until the floor is learned
 * (the firmware detector covers that time). */
static uint32_t det_thr_now(void)
{
	double n1 = noise_now(), t;
	if(n1 <= 0.0 || ratio12 <= 0.0) return 0xFFFFFFFFU;
	t = AJ_DET_MARGIN * n1 * (1.0 + 1.0 / ratio12);
	/* The core sees the raw samples, DC offset included (the ARM removes it
	 * before its own detection), and switches off below HALF the threshold:
	 * keep that half above the clean level noise + DC, or it never would.
	 * With no DC this is the 6 dB threshold above. */
	if(res_valid) {
		double clean = n1 * (1.0 + 1.0 / ratio12);
		if(!(core_v24 && dc_on))        /* v2.4: the core removes the DC first */
			clean += res.dc1 * res.dc1 + res.dc2 * res.dc2;
		if(t < 2.5 * clean) t = 2.5 * clean;
	}
	if(t > 4.0e9) t = 4.0e9;
	if(t < 4.0)   t = 4.0;
	return (uint32_t)t;
}

static uint32_t det_hold_now(void)
{
	uint32_t fs = 0;
	if(ad9361_phy) ad9361_get_rx_sampling_freq(ad9361_phy, &fs);
	return (uint32_t)((uint64_t)fs * AJ_HOLD_US / 1000000U);
}

/* Mode, image taps, make-up, g2 and the detector settings; written only
 * when something changed (the threshold when it moved by > 1/8). */
static void commit_core(int force)
{
	uint32_t mode, cfg, thr, hold;
	int32_t  gq;
	if(!aj_on || aj_frozen) return;
	if(gated()) mode = fw_jam ? GNSS_PT_NPI_MODE_ON :
	                  hw_veto ? GNSS_PT_NPI_MODE_BYPASS : GNSS_PT_NPI_MODE_AUTO;
	else        mode = GNSS_PT_NPI_MODE_ON;
	cfg  = mode | (image_on() ? GNSS_PT_NPI_CFG_IMAGE : 0U) |
	       GNSS_PT_NPI_CFG_SHIFT(jam ? shift : 0);
	if(core_v24) {
		int k;
		if(dc_on) cfg |= GNSS_PT_NPI_CFG_DCBLK;
		for(k = 0; k < NPI_CW_MAX && k < core_notches; k++)
			if(notch_auto && cwt[k].used && cwt[k].conf) cfg |= GNSS_PT_NPI_CFG_NOTCH(k);
		if(core_v25 && eq_on) cfg |= GNSS_PT_NPI_CFG_EQ;
	}
	thr  = det_thr_now();
	hold = det_hold_now();
	/* g2 only changes while the core is bypassed: a change while it filters
	 * would throw the weights off */
	gq = (jam || core_active) ? g2_w : g2_q13();
	/* g2 follows the learned ratio in steps > 1/128 (0.07 dB), not every
	 * capture's estimation noise */
	if(!force && gq != g2_w && (gq > g2_w ? gq - g2_w : g2_w - gq) <= g2_w / 128)
		gq = g2_w;
	/* 64-bit: thr_w + thr_w/8 overflows for the "never" threshold */
	{
		int k, nc = 0;
		uint32_t nce[6];
		for(k = 0; k < 6; k++) nce[k] = ncoef_cur[k];
		/* ONE confirmed CW and two notches: both notches on it. The depth on the
		 * CW is then squared (the zero's rounding residual, ~-48 dB, is what
		 * costs C/N0 against a strong CW) for no extra hardware. */
		if(core_v24 && notch_auto && core_notches >= 2) {
			int nconf = 0, first = -1;
			for(k = 0; k < NPI_CW_MAX; k++)
				if(cwt[k].used && cwt[k].conf) { nconf++; if(first < 0) first = k; }
			if(nconf == 1) {
				for(k = 0; k < 3; k++) { nce[k] = ncoef_cur[3 * first + k]; nce[3 + k] = nce[k]; }
				cfg |= GNSS_PT_NPI_CFG_NOTCH(0) | GNSS_PT_NPI_CFG_NOTCH(1);
				notch_f[0] = notch_f[1] = notch_f[first];
			}
		}
		for(k = 0; k < 6; k++) if(nce[k] != ncoef_w[k]) nc = 1;
		if(nc || !ncoef_valid) {
			gnss_pt_npi_write_ncoef(nce);
			for(k = 0; k < 6; k++) ncoef_w[k] = nce[k];
			ncoef_valid = 1;
			force = 1;
		}
		if(core_v25) {
			int ec = 0;
			for(k = 0; k < 3; k++) if(eq_wcur[k] != eq_w[k]) ec = 1;
			if(ec) {
				gnss_pt_npi_write_eq(eq_wcur);
				for(k = 0; k < 3; k++) eq_w[k] = eq_wcur[k];
				force = 1;
			}
		}
	}
	if(!force && cfg == cfg_w && gq == g2_w && hold == hold_w &&
	   (uint64_t)thr <= (uint64_t)thr_w + thr_w / 8U && thr >= thr_w - thr_w / 8U)
		return;
	gnss_pt_npi_write(cfg, gq, thr, hold);
	cfg_w = cfg; g2_w = gq; thr_w = thr; hold_w = hold;
}

static double g2_core(void) { return (double)g2_w / 8192.0; }

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
	gsig_mark = gain_signature();
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
	int k, n = 0;
	radio(&fs, &lo);
	bw_mhz = res.bw * (double)fs / 1e6;
	f_mhz  = res.foff * (double)fs / 1e6;
	rf_mhz = (double)lo / 1e6 + f_mhz;
	/* A CW below the wideband noise: the lag-1 bandwidth estimate only sees
	 * the noise, so name the tones the FFT scan confirmed instead. */
	if(res.jn_db < AJ_CW_WB_DB) {
		for(k = 0; k < NPI_CW_MAX; k++) {
			if(!cwt[k].used || !cwt[k].conf) continue;
			console_print("%s%s MHz", (char*)(n ? " + " : "CW tone at RF "),
				      f3u(((double)lo + cwt[k].f * (double)fs) / 1e6));
			n++;
		}
		if(n) { console_print(" (CW scan, below the wideband noise)"); return; }
	}
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
	case 4:  return "the FPGA power detector (input 6 dB above the noise, RX1/RX2 coherent)";
	case 5:  return "the CW scan (narrowband peak in both antennas)";
	default: return "?";
	}
}

static void print_line(XTime now)
{
	stamp(now);
	if(jam && res_meas) {
		console_print("JAMMED  J/N %s dB  suppression %s dB  residual %s dB | ",
			      f1(res.jn_db), supp_txt(res.supp_db), f1u(res_disp(res.res_db)));
		print_type();
		console_print(" | RX %d/%d dB  make-up +%d dB  TX att %s dB | |w2| %s | %d captures/s%s\n",
			      (long)cur_gain(), (long)gnss_rxg_ref(), (long)(6 * shift),
			      f2u(tx_att_db()), f2u(npi_sqrt(res.w[0] * res.w[0] + res.w[1] * res.w[1])),
			      (long)cap_rate, (char*)(aj_frozen ? " FROZEN" : ""));
	} else {
		console_print("clear   noise %s LSB^2  coherence %s dB  power J/N %s | RX %d/%d dB"
			      "  TX att %s dB%s | g2 %s dB | %s, %d captures/s%s\n",
			      f1u(res.p1), f1(res.jn_coh_db < -30.0 ? -30.0 : res.jn_coh_db),
			      (char*)(nf0 > 0.0 ? f1(res.jn_db < -9.9 ? -9.9 : res.jn_db) : "(learning)"),
			      (long)cur_gain(), (long)gnss_rxg_ref(), f2u(tx_att_db()),
			      (char*)(transmitting() ? "" : " (TX off)"),
			      f1(npi_db10(g2 * g2)),
			      (char*)(gated() ? (thr_w == 0xFFFFFFFFU ? "bypass, detector not armed yet" : "bypass, detector armed")
			                      : "filter always in"),
			      (long)cap_rate, (char*)(aj_frozen ? " FROZEN" : ""));
	}
}

static void print_csv(XTime now)
{
	uint32_t fs; uint64_t lo;
	if(!csv_hdr) {
		csv_hdr = 1;
		console_print("AJCSV,t_s,state,jn_db,coh_db,supp_db,res_db,rx_gain_db,cut_db,"
			      "makeup_db,tx_att_db,g2_db,rate_per_s,bw_mhz,centre_mhz\n");
	}
	radio(&fs, &lo);
	console_print("AJCSV,%s,%s,%s,%s,%s,%s,%d,%d,%d,%s,%s,%d,%s,%s\n",
		      f3u((double)ms_since(t_boot, now) / 1000.0),
		      (char*)(jam ? "JAMMED" : "CLEAR"),
		      fmt(res.jn_db < -99.0 ? -99.0 : res.jn_db, 1, 0),
		      fmt(res.jn_coh_db < -30.0 ? -30.0 : res.jn_coh_db, 1, 0),
		      fmt(res_meas ? res.supp_db : 0.0, 1, 0),
		      fmt(res_meas ? res_disp(res.res_db) : 0.0, 1, 0),
		      (long)cur_gain(), (long)cut_db(), (long)(6 * shift),
		      f2u(tx_att_db()), fmt(npi_db10(g2 * g2), 1, 0), (long)cap_rate,
		      fmt(res.bw * (double)fs / 1e6, 1, 0), fmt(res.foff * (double)fs / 1e6, 1, 0));
}

/* ---- learning ------------------------------------------------------------------ */
static void update_g2(void)
{
	double g = npi_sqrt(ratio12);
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
	double cand = (res.p1 - NPI_QFLOOR) / g;
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
	if(res.p2 > NPI_QFLOOR) {
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
	if(flap_n == 0 && !(jam)) {
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
			      (char*)((jam) ? "ON" : "off"));
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
		double cand = (res.p1 - NPI_QFLOOR) / db2lin((double)cur_gain()), up;
		rise_on = 0;
		if(cand <= nf0) return;
		up = npi_db10(cand / nf0);
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
	if(show) console_print("\n           detected by %s; RX gain cut %d dB\n",
		      (char*)why_txt(jam_hw ? 4 : on_why), (long)cut_db());
	if(on_ev > ev_n - AJ_NEV) {
		ev[on_ev % AJ_NEV].v1 = res.jn_db;
		ev[on_ev % AJ_NEV].v2 = res.bw;
	}
	if(jam_nulled) {
		if(show) stamp(now);
		if(show) console_print("NULLED %s ms after detection: suppression %s dB, residual %s dB, make-up +%d dB\n",
			      fmt(null_ms, 0, 0), supp_txt(res.supp_db), f1u(res_disp(res.res_db)),
			      (long)(6 * shift));
	}
}

/* No clean period since start-up (the jammer was there first): g2 from
 * the jammer itself, which both antennas see. Only while the core is still
 * bypassed (its detector is not armed without a floor), so the commit that
 * switches it in carries this g2. */
static void g2_from_jammer(void)
{
	if(g2_src != 0 || res.p2 <= NPI_QFLOOR || core_active || jam) return;
	ratio12 = res.p1 / res.p2;
	update_g2();
	rx2n = 1.0;
	g2_src = 2;
}

static void enter_jammed(XTime now, int by_hw)
{
	jam = 1;
	jam_hw = by_hw;
	jam_t0 = by_hw ? now : trig_t0;
	res_bad_t0 = now;
	jam_announced = 0;
	jam_nulled = 0;
	null_ms = -1.0;
	jam_peak = -99.0;
	shift = 0;
	on_ev = add_event(jam_t0, EV_ON, res.jn_db, res.bw, (double)(by_hw ? 4 : on_why));
}

static void leave_jammed(XTime now)
{
	uint32_t ms = ms_since(jam_t0, now);
	jam = 0;
	shift = 0;
	clear_t0 = now;
	if(!jam_announced) announce(now);
	if(!jam_silent && ev_print(now)) {
		console_print("\n");
		stamp(now);
		console_print("jammer OFF after %s s (peak J/N %s dB): JAMMER STATUS OFF\n",
			      f1u((double)ms / 1000.0), f1(jam_peak));
	}
	last_off_t = now;
	have_off = 1;
	if(!core_active) eq_reset();                 /* v2.5: the taps are for THIS jammer */
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
			if(nf0 <= 0.0 && res.p1 > NPI_QFLOOR)      /* the floor is this */
				nf0 = (res.p1 - NPI_QFLOOR) / db2lin((double)cur_gain());
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

/* The firmware's own detector: weak jammers (forces the core ON), and the
 * jammer state when the core is not gated. */
static void fw_detect(XTime now)
{
	int cut = cut_db(), why = 0, quiet;
	if(bg_track(now, fw_jam)) { fw_jam = 0; on_cnt = 0; }
	rise_track(now);
	if(res.jn_coh_db >= coh_on())                  why = 1;
	else if(nf0 > 0.0 && res.jn_db >= AJ_ON_DB &&
	        res.jn_coh_db >= AJ_PWR_COH_DB)        why = 2;
	else if(cut >= AJ_CUT_DB && now - t_boot >= us(AJ_SETTLE_US)) why = 3;
	quiet = res.jn_coh_db < coh_off() && (nf0 <= 0.0 || res.jn_db < AJ_OFF_DB) && cut == 0;
	if(!fw_jam) {
		on_cnt = why ? on_cnt + 1 : 0;
		if(on_cnt >= AJ_ON_COUNT) { fw_jam = 1; quiet_t0 = now; g2_from_jammer(); }
	} else {
		if(!quiet) quiet_t0 = now;
		else if(now - quiet_t0 >= us(AJ_OFF_US)) { fw_jam = 0; on_cnt = 0; }
	}
	/* learn the floor and g2 only with nothing at all going on */
	if(!fw_jam && !core_active && !jam && !why && res.jn_coh_db < coh_quiet() &&
	   (nf0 <= 0.0 || res.jn_db < AJ_OFF_DB) && cut == 0 && gnss_rxg_ref() == cur_gain())
		learn(now);
}

static void jammed_state(XTime now)
{
	int want;

	if(res_meas && !jam_nulled && res.res_db <= AJ_NULL_RES_DB) {
		jam_nulled = 1;
		null_ms = (double)ms_since(jam_t0, now);
		if(jam_announced && !jam_silent && ev_print(now)) {
			stamp(now);
			console_print("NULLED %s ms after detection: suppression %s dB, residual %s dB\n",
				      fmt(null_ms, 0, 0), supp_txt(res.supp_db), f1u(res_disp(res.res_db)));
		}
		add_event(now, EV_NULLED, null_ms, res.supp_db, res.res_db);
	}
	if(!jam_announced && now - jam_t0 >= us(AJ_ANNOUNCE_US))
		announce(now);
	if(jam_announced && res.jn_db > jam_peak) jam_peak = res.jn_db;

	/* Output make-up gain: 6 dB per step of RX gain cut, only while the
	 * jammer is nulled. Down at once, up one step per capture. */
	want = (res_meas && res.res_db <= AJ_SHIFT_RES_DB) ? cut_db() / 6 : 0;
	if(want > 4) want = 4;
	if(want < shift)      shift = want;
	else if(want > shift) shift++;

	if(!res_meas || res.res_db <= AJ_WARN_RES_DB)
		res_bad_t0 = now;
	if(jam_announced && now - res_bad_t0 >= us(AJ_WARN_US) &&
	   now - t_warn >= us(5000000U) && !jam_silent && ev_print(now)) {
		t_warn = now;
		stamp(now);
		console_print("WARNING residual %s dB: the jammer is not fully nulled.\n"
			      "     PL-NPI has ONE spatial weight: it nulls one direction at one frequency.\n"
			      "     A wideband jammer through unequal RX1/RX2 paths (cable lengths, filters)\n"
			      "     or a second jammer is not fully removed. Check the cables and RX2's\n"
			      "     antenna; SMI-PI v4.1 (31 space-time taps) is the design for wideband.\n",
			      f1(res.res_db));
		add_event(now, EV_LEAK, res.res_db, res.jn_db, 0.0);
	}
}

/* ---- v2.4: CW tracks and the notches ---------------------------------------- */
#define AJ_PI 3.14159265358979323846
static double dabs(double x) { return x < 0.0 ? -x : x; }

static uint32_t qxw(double re, double im, double scale)
{
	long a = (long)(re * scale + (re >= 0.0 ? 0.5 : -0.5));
	long b = (long)(im * scale + (im >= 0.0 ? 0.5 : -0.5));
	if(a > 32767) a = 32767;
	if(a < -32768) a = -32768;
	if(b > 32767) b = 32767;
	if(b < -32768) b = -32768;
	return ((uint32_t)(b & 0xFFFF) << 16) | (uint32_t)(a & 0xFFFF);
}

static uint32_t q14w(double re, double im)
{
	long a = (long)(re * 16384.0 + (re >= 0.0 ? 0.5 : -0.5));
	long b = (long)(im * 16384.0 + (im >= 0.0 ? 0.5 : -0.5));
	if(a > 32767) a = 32767;
	if(a < -32768) a = -32768;
	if(b > 32767) b = 32767;
	if(b < -32768) b = -32768;
	return ((uint32_t)(b & 0xFFFF) << 16) | (uint32_t)(a & 0xFFFF);
}

/* notch k on f (fraction of fs): q = e^{j 2 pi f}, p = r q, p^2 */
static void notch_set(int k, double f)
{
	double s1, c1, s2, c2, r = AJ_NOTCH_R;
	npi_sincos(2.0 * AJ_PI * f, &s1, &c1);
	npi_sincos(4.0 * AJ_PI * f, &s2, &c2);
	/* the zero q sets the depth on a strong CW: Q1.15 in v2.5 (Q1.14 before) */
	ncoef_cur[3 * k]     = core_v25 ? qxw(c1, s1, 32768.0) : q14w(c1, s1);
	ncoef_cur[3 * k + 1] = q14w(r * c1, r * s1);
	ncoef_cur[3 * k + 2] = q14w(r * r * c2, r * r * s2);
	notch_f[k] = f;
}

static double rf_mhz(double f)
{
	uint32_t fs; uint64_t lo;
	radio(&fs, &lo);
	return ((double)lo + f * (double)fs) / 1e6;
}

/* in-bin SNR of a CW -> its J/N over the whole band (Hann ENBW 1.5 bins) */
static double cw_jn(double snr) { return snr - npi_db10((double)NPI_CW_N / 1.5); }

static int cw_confirmed(void)
{
	int k;
	for(k = 0; k < NPI_CW_MAX; k++) {
		double s = cwt[k].snr1 < cwt[k].snr2 ? cwt[k].snr1 : cwt[k].snr2;
		if(cwt[k].used && cwt[k].conf && cw_jn(s) >= AJ_CW_ON_JN_DB) return 1;
	}
	return 0;
}

/* ---- is a strong CW an outside jammer, or TX1 feeding back into RX? ---------------
 * The board retransmits RX1 on TX1. If part of that leaks back into the RX
 * antennas/cables (RX gain 70 dB, TX attenuation 10 dB), a loop makes a drifting
 * tone that the CW scan sees in both antennas, exactly like a jammer. The only
 * way to tell: mute TX1 for AJ_TXL_TEST_US and look again. Gone = feedback (not a
 * jammer, status stays OFF); still there = external (JAMMER ON). One test, then
 * the verdict is kept (leak: AJ_TXL_LEAK_US, external: until the CW is gone). */
#define AJ_TXL_TEST_US    700000U
#define AJ_TXL_SETTLE_US  300000U    /* the loop needs this long to die after the mute */
#define AJ_TXL_LEAK_US    300000000U
#define AJ_TXL_EXT_GONE_US 2000000U
static int   txl_state;              /* 1 = TX1 muted by this test */
static int   txl_verdict;            /* 0 none, 1 external, 2 TX1 feedback */
static XTime txl_t0, txl_vt, txl_gone_t0;
static double txl_f;                 /* RF MHz of the tested CW */

static int cw_present_since(XTime t)
{
	int k;
	for(k = 0; k < NPI_CW_MAX; k++) {
		double s = cwt[k].snr1 < cwt[k].snr2 ? cwt[k].snr1 : cwt[k].snr2;
		if(cwt[k].used && cwt[k].conf && cw_jn(s) >= AJ_CW_ON_JN_DB && cwt[k].seen > t) return 1;
	}
	return 0;
}

static int cw_busy(void)
{
	int k;
	for(k = 0; k < NPI_CW_MAX; k++) if(cwt[k].used) return 1;
	return 0;
}

static void txl_abort(void)
{
	if(txl_state) {
		uint32_t c = gnss_pt_read(GNSS_PT_REG_CONTROL);
		if(tx_noted == 1 && (c & GNSS_PT_CTRL_MUTE)) gnss_pt_write(GNSS_PT_REG_CONTROL, c & ~GNSS_PT_CTRL_MUTE);
	}
	txl_state = 0;
}

/* 1 = a strong CW that is an outside jammer (or cannot be TX1 feedback) */
static int cw_jammer(XTime now)
{
	int strong = cw_confirmed();
	if(txl_state) {
		if(tx_noted != 1) { txl_state = 0; return 0; }      /* the user stopped TX1 */
		if(now - txl_t0 < us(AJ_TXL_TEST_US)) return 0;
		{
			int still = cw_present_since(txl_t0 + us(AJ_TXL_SETTLE_US));
			txl_abort();
			txl_verdict = still ? 1 : 2;
			txl_vt = now;
			add_event(now, EV_CW, txl_f, (double)txl_verdict, 2.0);
			if(ev_print(now)) {
				stamp(now);
				if(still)
					console_print("CW at %s MHz is still there with TX1 muted: an OUTSIDE source, JAMMER\n", f3u(txl_f));
				else
					console_print("CW at %s MHz vanished when TX1 was muted: it is TX1 FEEDBACK into RX, NOT a jammer.\n"
						      "     Raise the TX attenuation (tx1_attenuation=70000), lower the RX gain,\n"
						      "     or separate the TX1 cable from the RX antennas/cables.\n", f3u(txl_f));
			}
		}
		return txl_verdict == 1;
	}
	if(!strong) {
		if(txl_verdict == 1 && now - txl_gone_t0 >= us(AJ_TXL_EXT_GONE_US)) txl_verdict = 0;
		if(txl_verdict != 1) txl_gone_t0 = now;
		return 0;
	}
	txl_gone_t0 = now;
	if(tx_noted != 1) return 1;                                   /* no TX1: cannot be a loop */
	if(txl_verdict == 2 && now - txl_vt >= us(AJ_TXL_LEAK_US)) txl_verdict = 0;
	if(txl_verdict) return txl_verdict == 1;
	{
		uint32_t c = gnss_pt_read(GNSS_PT_REG_CONTROL);
		int k, best = 0;
		if(c & GNSS_PT_CTRL_MUTE) return 1;                       /* muted by the operator: leave it */
		for(k = 0; k < NPI_CW_MAX; k++)
			if(cwt[k].used && cwt[k].conf && cwt[k].snr1 >= cwt[best].snr1) best = k;
		txl_f = rf_mhz(cwt[best].f);
		gnss_pt_write(GNSS_PT_REG_CONTROL, c | GNSS_PT_CTRL_MUTE);
		txl_state = 1; txl_t0 = now;
	}
	return 0;
}

static void cw_update(XTime now)
{
	npi_cw_t o;
	int i, k;
	if(++cw_scan_n < AJ_CW_EVERY && ajs_on_n == 0) return;   /* scan every capture while evidence builds */
	cw_scan_n = 0;
	if(npi_cw_scan(AJ_CW_KEEP_DB, &o) != 0) return;
	for(k = 0; k < NPI_CW_MAX; k++) if(cwt[k].used) cwt[k].scans++;
	for(i = 0; i < o.n; i++) {
		double snr = (o.snr1[i] < o.snr2[i]) ? o.snr1[i] : o.snr2[i];
		int m = -1;
		for(k = 0; k < NPI_CW_MAX; k++)
			if(cwt[k].used && dabs(cwt[k].f - o.f[i]) <= AJ_CW_SAME) m = k;
		if(m < 0 && snr >= AJ_CW_DET_DB)
			for(k = 0; k < NPI_CW_MAX; k++)
				if(!cwt[k].used) {
					cwt[k].used = 1; cwt[k].hits = 0; cwt[k].scans = 1; cwt[k].conf = 0;
					cwt[k].f = o.f[i]; cwt[k].t0 = now;
					m = k;
					break;
				}
		if(m < 0) continue;                       /* weak, or two CWs tracked */
		if(!cwt[m].conf && snr < AJ_CW_DET_DB) continue;   /* hysteresis */
		cwt[m].hits++;
		cwt[m].seen = now;
		cwt[m].f = 0.75 * cwt[m].f + 0.25 * o.f[i];
		cwt[m].snr1 = o.snr1[i];
		cwt[m].snr2 = o.snr2[i];
		if(!cwt[m].conf && cwt[m].hits >= AJ_CW_HITS && 2 * cwt[m].hits >= cwt[m].scans) {
			cwt[m].conf = 1;
			notch_set(m, cwt[m].f);
			add_event(now, EV_CW, rf_mhz(cwt[m].f), cwt[m].snr1, 1.0);
			if(ev_print(now)) {
				stamp(now);
				console_print("CW at %s MHz, +%s dB in its FFT bin (J/N %s dB over the band): ",
					      f3u(rf_mhz(cwt[m].f)), f1u(cwt[m].snr1), f1(cw_jn(cwt[m].snr1)));
				if(core_v24 && m < core_notches && notch_auto)
					console_print("notch %d ON\n", (long)m);
				else
					console_print("%s\n", (char*)(!notch_auto ? "notch off (gnss_aj_notch=0)" :
						"no notch in this bitstream (needs v2.4)"));
			}
		} else if(cwt[m].conf && dabs(cwt[m].f - notch_f[m]) > AJ_CW_RETUNE) {
			notch_set(m, cwt[m].f);               /* the CW drifted: follow it */
		}
	}
	for(k = 0; k < NPI_CW_MAX; k++) {
		if(!cwt[k].used) continue;
		if(!cwt[k].conf && cwt[k].scans >= AJ_CW_TRIAL && 2 * cwt[k].hits < cwt[k].scans) {
			cwt[k].used = 0;                      /* a fluke, not a CW */
			continue;
		}
		if(now - cwt[k].seen < us(AJ_CW_LOST_US)) continue;
		if(cwt[k].conf) {
			add_event(now, EV_CW, rf_mhz(cwt[k].f), cwt[k].snr1, 0.0);
			if(ev_print(now)) {
				stamp(now);
				console_print("CW at %s MHz gone: notch %d off\n", f3u(rf_mhz(cwt[k].f)), (long)k);
			}
		}
		cwt[k].used = 0;
		cwt[k].conf = 0;
	}
}

/* ---- v2.5: RX2 equaliser ---------------------------------------------------------
 * A wideband jammer reaches RX1 and RX2 through slightly different paths (cable
 * length, filters). One complex weight matches them at one frequency only, so a
 * 40 MHz jammer through 0.5 sample (2 m of cable) is nulled just ~11 dB (C/N0
 * down 18 dB at J/N 30). 3 taps on RX2 map it onto RX1 across the band. The taps
 * are fitted by least squares on the raw capture (notching known CWs first) while
 * a jammer is there, switched on if they beat the single weight, kept while they
 * are the better fit, and dropped when the jammer is gone. */
static void eq_apply_words(void)
{
	int k;
	for(k = 0; k < 3; k++) {
		double fr = eq_on ? eq_c[k][0] : (k == 1 ? 1.0 : 0.0), fi = eq_on ? eq_c[k][1] : 0.0;
		eq_wcur[k] = qxw(fr, fi, 16384.0);
	}
	npi_eq_set((const double (*)[2])eq_c, eq_on);
}

static void eq_reset(void)
{
	if(!eq_on && !eq_valid) return;
	eq_on = 0; eq_valid = 0; eq_cnt = 0;
	eq_apply_words();
	commit_core(1);
}

/* before npi_detect: ask for a fit on this capture when it is due */
static void eq_request(void)
{
	int k, n = 0;
	res.eq_req = 0;
	if(!core_v25 || !eq_auto || !(jam || core_active)) return;
	if(++eq_cnt < AJ_EQ_EVERY) return;
	eq_cnt = 0;
	for(k = 0; k < NPI_CW_MAX && n < 2; k++)
		if(cwt[k].used && cwt[k].conf) res.eq_cwf[n++] = cwt[k].f;
	res.eq_ncw = n;
	res.eq_req = 1;
}

/* after npi_detect */
static void eq_update(XTime now)
{
	int k, apply = 0, big = 0;
	if(!res.eq_req || !res.eq_ok) return;
	eq_jn_db = res.eq_jn_db; eq_gain_db = res.eq_gain_db; eq_cur_db = res.eq_cur_db;
	for(k = 0; k < 3; k++)
		if(res.eq_f[k][0] > AJ_EQ_MAX_TAP || res.eq_f[k][0] < -AJ_EQ_MAX_TAP ||
		   res.eq_f[k][1] > AJ_EQ_MAX_TAP || res.eq_f[k][1] < -AJ_EQ_MAX_TAP) big = 1;
	if(big || res.eq_jn_db < AJ_EQ_MIN_JN_DB) return;       /* too weak a jammer to fit, or taps out of range */
	if(!eq_on) apply = res.eq_gain_db >= AJ_EQ_ON_GAIN_DB;
	else       apply = res.eq_cur_db >= AJ_EQ_UPD_DB;
	if(!apply) return;
	for(k = 0; k < 3; k++) { eq_c[k][0] = res.eq_f[k][0]; eq_c[k][1] = res.eq_f[k][1]; }
	eq_on = 1; eq_valid = 1; eq_nfit++;
	eq_apply_words();
	commit_core(1);
	add_event(now, EV_EQ, res.eq_gain_db, res.eq_jn_db, (double)res.eq_ncw);
	if(ev_print(now)) {
		stamp(now);
		console_print("RX2 equaliser %s: the jammer (%s dB over the noise) is %s dB deeper than one weight can null it\n",
			      (char*)(eq_nfit == 1 ? "ON" : "updated"), f1(res.eq_jn_db), f1(res.eq_gain_db));
	}
}

/* ---- v2.4: JAMMER STATUS ------------------------------------------------------- */
static const char *ajs_txt(int st)
{
	return st == AJS_ON ? "ON" : st == AJS_OFF ? "OFF" : "UNKNOWN";
}

static void status_update(XTime now, int hw_coh)
{
	int cut = cut_db(), why = 0, quiet;
	int cwj = cw_jammer(now);        /* every capture: it times the TX1 mute test */
	/* A strong CW is in both antennas, so it also looks like coherence / power.
	 * While it is being tested, or was found to be TX1 feedback, that evidence
	 * is the same tone: ignore it (an outside CW counts through the CW rule). */
	int ign = tx_noted == 1 && txl_verdict != 1 && cw_busy();
	if(!ign && res.jn_coh_db >= coh_on())                            why = 1;
	else if(!ign && nf0 > 0.0 && res.jn_db >= AJ_ON_DB &&
	        res.jn_coh_db >= AJ_PWR_COH_DB)                          why = 2;
	else if(cut >= AJ_CUT_DB && now - t_boot >= us(AJ_SETTLE_US))   why = 3;
	else if(!ign && hw_coh)                                          why = 4;
	else if(cwj)                                                     why = 5;
	quiet = !why && res.jn_coh_db < coh_off() && (nf0 <= 0.0 || res.jn_db < AJ_OFF_DB) && cut == 0;
	if(ajs != AJS_ON) {
		if(ajs == AJS_UNKNOWN) { ajs = AJS_OFF; ajs_t0 = now; ajs_on_n = 0; }
		ajs_on_n = why ? ajs_on_n + 1 : 0;
		if(ajs_on_n == 1) { trig_t0 = now; ajs_why = why; }
		if(ajs_on_n >= AJ_ST_ON_N) {
			ajs = AJS_ON;
			ajs_t0 = trig_t0;
			ajs_quiet_t0 = now;
			on_why = ajs_why;
			enter_jammed(now, ajs_why == 4);
		}
	} else {
		if(why) ajs_why = why;
		if(!quiet) ajs_quiet_t0 = now;
		else if(now - ajs_quiet_t0 >= us(AJ_ST_OFF_US)) {
			ajs = AJS_OFF;
			ajs_t0 = now;
			ajs_on_n = 0;
			leave_jammed(now);
		}
	}
}

/* No valid capture for AJ_ST_STALE_US (DMA errors, TX DDR replay, anti-jam
 * off): the status is not known. */
static void status_stale(XTime now)
{
	txl_abort();                     /* never leave TX1 muted by the test */
	if(ajs == AJS_UNKNOWN) return;
	if(ajs == AJS_ON) {
		uint32_t ms = ms_since(jam_t0, now);
		jam = 0;
		shift = 0;
		add_event(now, EV_OFF, (double)ms / 1000.0, jam_peak, 1.0);
		if(ev_print(now)) {
			stamp(now);
			console_print("no measurement for %d ms: JAMMER STATUS UNKNOWN\n",
				      (long)(AJ_ST_STALE_US / 1000U));
		}
	}
	ajs = AJS_UNKNOWN;
	ajs_t0 = now;
	ajs_on_n = 0;
}

/* ---- v2.4: the snapshot gnss_report / gnss_aj? print ------------------------------
 * Filled at the end of every processed capture. The commands print it and the
 * engine statics; they read no register and run no detector, so one report is
 * one coherent moment, and asking for it changes nothing. */
static struct {
	int      valid;
	XTime    t;
	uint32_t ver, cfg_act, gamma, st, stat_reg, pow;
	int32_t  g0, g1;
	double   txatt;
	int      txon, hw_coh;
} vw;
static uint64_t vw_lo;
static uint32_t vw_fs, vw_rbw, vw_tbw;

static void view_radio(void)
{
	if(!ad9361_phy) return;
	ad9361_get_rx_lo_freq(ad9361_phy, &vw_lo);
	ad9361_get_rx_sampling_freq(ad9361_phy, &vw_fs);
	ad9361_get_rx_rf_bandwidth(ad9361_phy, &vw_rbw);
	ad9361_get_tx_rf_bandwidth(ad9361_phy, &vw_tbw);
}

static void view_take(XTime now, int hw_coh)
{
	vw.t        = now;
	vw.ver      = gnss_pt_read(GNSS_PT_REG_VERSION);
	vw.cfg_act  = gnss_pt_read(GNSS_PT_REG_NPI_CFG_ACTIVE);
	vw.gamma    = gnss_pt_read(GNSS_PT_REG_NPI_GAMMA);
	vw.stat_reg = gnss_pt_read(GNSS_PT_REG_STATUS);
	vw.st       = snap.st;
	vw.pow      = snap.pow;
	vw.g0 = vw.g1 = 0;
	if(ad9361_phy) {
		ad9361_get_rx_rf_gain(ad9361_phy, 0, &vw.g0);
		ad9361_get_rx_rf_gain(ad9361_phy, 1, &vw.g1);
	}
	vw.txatt  = tx_att_db();
	vw.txon   = transmitting();
	vw.hw_coh = hw_coh;
	if(vw_fs == 0U) view_radio();
	vw.valid  = 1;
}

/* the RX gain, both channels: a capture that spans a change is not used */
static uint32_t gain_signature(void)
{
	if(gnss_rxg_mode() == GNSS_RXG_AD9361 && ad9361_phy) {
		int32_t a = 0, b = 0;
		ad9361_get_rx_rf_gain(ad9361_phy, 0, &a);
		ad9361_get_rx_rf_gain(ad9361_phy, 1, &b);
		return 0x80000000U | ((uint32_t)(a & 0xFF) << 8) | (uint32_t)(b & 0xFF);
	}
	return gnss_rxg_changes();
}

void gnss_aj_poll(void)
{
	XTime now;
	int by_hw = 0, hw_coh;
	uint32_t cnt;

	if(!aj_on || !rx_dmac || !ad9361_phy) return;
	XTime_GetTime(&now);
	if(now - last_valid > us(AJ_ST_STALE_US)) status_stale(now);
	if(gnss_txdma_is_running()) return;       /* the DDR replay owns the DMA */

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
	if(gain_signature() != gsig_mark) {
		/* the RX gain stepped during the capture: the block mixes two gains */
		n_skip++;
		return;
	}
	Xil_DCacheInvalidateRange((uintptr_t)GNSS_AJ_BUF_ADDR, AJ_BYTES);

	res.noise_ref = noise_now();
	res.rho_noise_re = rho_n_re;
	res.rho_noise_im = rho_n_im;
	res.g2 = g2_core();
	res.rx2_noise = rx2n;
	eq_request();
	if(npi_detect((const uint32_t *)GNSS_AJ_BUF_ADDR, NPI_NSAMP, &res) != 0 || !res.ok) {
		n_fail++;
		return;
	}
	res_valid = 1;
	n_cap++;
	cnt_cap++;
	last_valid = now;
	cw_update(now);                    /* v2.4: on this capture's spectrum */
	eq_update(now);                    /* v2.5: RX2 equaliser fit from this capture */

	/* what the core is doing, and what its weights do to this capture */
	gnss_pt_npi_snapshot(&snap);
	core_active = (snap.st & GNSS_PT_NPI_ST_ACTIVE) ? 1 : 0;
	if(core_v24) core_notches = (int)GNSS_PT_NPI_ST_NOTCHES(snap.st);
	res_meas = 0;
	if(core_active) {
		npi_measure((const uint32_t *)GNSS_AJ_BUF_ADDR, NPI_NSAMP, snap.w, &res);
		res_meas = 1;
		if(image_on()) {
			double c1 = npi_sqrt(res.w[2] * res.w[2] + res.w[3] * res.w[3]);
			double c2 = npi_sqrt(res.w[4] * res.w[4] + res.w[5] * res.w[5]);
			img_cnt = (c1 > AJ_IMG_MAX || c2 > AJ_IMG_MAX) ? img_cnt + 1 : 0;
			if(img_cnt >= AJ_IMG_COUNT) {
				img_bad = 1;
				img_cnt = 0;
				if(ev_print(now)) {
					stamp(now);
					console_print("image taps OFF: |c1| %s, |c2| %s, far above an AD9361 I/Q mirror\n"
						      "     (~0.01). They were cancelling something else (DC offset?) and\n"
						      "     adding noise. gnss_aj_mode=0 tries them again.\n", f3u(c1), f3u(c2));
				}
				commit_core(1);
			}
		}
	}
	cnt = GNSS_PT_NPI_ST_CNT(snap.st);
	if(cnt != det_cnt_seen) {
		/* the FPGA detector switched on since the last capture */
		by_hw = 1;
		if(!core_active && !jam && gated()) {
			n_burst++;
			add_event(now, EV_BURST, (double)((cnt - det_cnt_seen) & 0xFFFFU), 0.0, 0.0);
		}
		det_cnt_seen = cnt;
	}

	fw_detect(now);
	/* The FPGA detector reacts to power alone. A rise RX1 and RX2 do not
	 * share (no coherence, no RX gain cut) is the receivers' own noise, not
	 * a jammer: veto it (core to BYPASS, no JAMMER event) until the signal
	 * becomes coherent, the gain is cut or the power is back at the floor
	 * (rise_track() re-learns a floor that stays up). */
	if(gated() && core_active && !fw_jam && res.jn_coh_db < AJ_PWR_COH_DB && cut_db() == 0)
		hw_veto = 1;
	else if(fw_jam || res.jn_coh_db >= AJ_PWR_COH_DB || cut_db() > 0 ||
	        !(snap.st & GNSS_PT_NPI_ST_DET))   /* the FPGA detector is off again */
		hw_veto = 0;
	(void)by_hw;
	/* v2.4: the JAMMER STATUS has its own detector and hysteresis; the filter
	 * (core_active) is switched by the FPGA detector and fw_jam, separately */
	hw_coh = gated() && core_active && (snap.st & GNSS_PT_NPI_ST_DET) && !hw_veto;
	status_update(now, hw_coh);
	if(jam)
		jammed_state(now);
	commit_core(0);
	if(gnss_rxg_mode() != GNSS_RXG_MATCHED && nf0 > 0.0)
		gnss_npi_gamma_noise(noise_now(), jam || core_active);
	view_take(now, hw_coh);

	XTime_GetTime(&now);
	if(now - t_rate >= us(1000000U)) {
		cap_rate = cnt_cap;
		cnt_cap = 0;
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
}

/* ---- control ------------------------------------------------------------------ */
void gnss_aj_init(void)
{
	XTime now;
	XTime_GetTime(&now);
	t_boot = t_rate = t_log = t_csv = t_warn = t_learn = next_start = clear_t0 = now;
	keys_seen = console_key_count();
	t_key = now - us(AJ_KEY_QUIET_US);
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
		core_v24 = (ver >= 0x00020004U);
		core_v25 = (ver >= 0x00020005U);
		if(ver < 0x00020003U) {
			console_print("gnss_aj: the FPGA reports gnss_passthrough %d.%d; PL-NPI v2.3 needs 2.3\n"
				      "         (core on sys_cpu_clk, gating, normalisation, image taps). Anti-jam\n"
				      "         NOT started: program the BOOT.BIN / bitstream built from this source.\n",
				      (long)(ver >> 16), (long)(ver & 0xFFFFU));
			return;
		}
		/* v2.4: the RX gain policy of the deployment (GNSS_RXG_DEFAULT_MODE,
		 * gnss_npi_agc.h: the AD9361 slow-attack AGC on both channels from
		 * 40 dB), once; gnss_rxg_mode= changes it. The engine follows the
		 * gain in every mode (noise floor, gamma, captures across a step). */
		if(!gain_init) {
			gnss_rxg_set_mode(GNSS_RXG_DEFAULT_MODE);
			gain_init = 1;
		}
		gnss_npi_set_gated(gated());
		if(!aj_on) {
			jam = 0; fw_jam = 0; on_cnt = 0; core_active = 0; hw_veto = 0;
			ajs = AJS_UNKNOWN; ajs_t0 = now; ajs_on_n = 0; last_valid = now;
			clear_t0 = now;
			add_event(now, EV_START, 0.0, 0.0, 0.0);
		}
		aj_on = 1;
		eq_on = 0; eq_valid = 0; eq_cnt = 0;
		eq_apply_words();
		commit_core(1);
		gnss_pt_set_npi(1);
	} else {
		gnss_aj_dma_release();
		gnss_rxg_stop_tx_hold();
		gnss_pt_npi_write(GNSS_PT_NPI_MODE_BYPASS | GNSS_PT_NPI_CFG_IMAGE, g2_w, 0xFFFFFFFFU, 0U);
		cfg_w = 0xFFFFFFFFU;
		gnss_pt_set_npi(0);
		if(jam)
			console_print("[AJ] anti-jam switched off (jammer state no longer monitored)\n");
		aj_on = 0;
		eq_on = 0; eq_valid = 0;
		npi_eq_set((const double (*)[2])eq_c, 0);
		jam = 0; fw_jam = 0; shift = 0;
		ajs = AJS_UNKNOWN; ajs_t0 = now;
		txl_abort();
	}
	/* The enable crosses into the sample domain and back; wait (<= ~1 ms)
	 * until STATUS confirms it, so a status print straight after is right. */
	{
		uint32_t want = on ? GNSS_PT_ST_NPI_EN_SYNCED : 0U;
		int n;
		for(n = 0; n < 10000; n++)
			if((gnss_pt_read(GNSS_PT_REG_STATUS) & GNSS_PT_ST_NPI_EN_SYNCED) == want)
				break;
	}
	/* Switched on: the status is UNKNOWN until the first capture has been
	 * measured (~10 ms). Wait for it (<= 0.5 s), so the line printed straight
	 * after gnss_npi=1 already says ON or OFF and not UNKNOWN. */
	if(on && aj_on && ajs == AJS_UNKNOWN && !gnss_txdma_is_running()) {
		XTime t0, t;
		XTime_GetTime(&t0);
		do {
			gnss_aj_poll();
			XTime_GetTime(&t);
		} while(ajs == AJS_UNKNOWN && t - t0 < us(500000U));
	}
}

int  gnss_aj_enabled(void)   { return aj_on; }
int  gnss_aj_user_off(void)  { return aj_user_off; }
int  gnss_aj_jammer_on(void) { return jam || core_active; }

void gnss_aj_freeze(int on)
{
	aj_frozen = on ? 1 : 0;
	gnss_pt_set_npi_freeze(aj_frozen);
}

void gnss_aj_set_mode(int mode)
{
	if(mode < 0 || mode >= GNSS_AJ_NMODES) return;
	aj_mode = mode;
	img_bad = 0;
	img_cnt = 0;
	gnss_npi_set_gated(gated());
	commit_core(1);
}

void gnss_aj_set_notch(int on)
{
	notch_auto = on ? 1 : 0;
	commit_core(1);
}

void gnss_aj_set_dc(int on)
{
	dc_on = on ? 1 : 0;
	commit_core(1);
}

void gnss_aj_set_eq(int on)
{
	eq_auto = on ? 1 : 0;
	if(!eq_auto) eq_reset();
}

int gnss_aj_status(void) { return aj_on ? ajs : AJS_UNKNOWN; }

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
	case GNSS_AJ_MODE_GATED:       return "gated (bypass until a jammer), image taps on (gnss_aj_mode=0)";
	case GNSS_AJ_MODE_GATED_NOIMG: return "gated, image taps off (gnss_aj_mode=1)";
	case GNSS_AJ_MODE_ALWAYS:      return "always filtering and adapting, image taps on (gnss_aj_mode=2)";
	default:                       return "always filtering and adapting, image taps off, as v2.2 (gnss_aj_mode=3)";
	}
}

static const char *g2_src_txt(void)
{
	return g2_src == 1 ? "from the noise floors" :
	       g2_src == 2 ? "from the jammer (no clean period yet)" : "default, not learned yet";
}

static const char *filter_why(void)
{
	if(!aj_on)                        return "anti-jam off";
	if(!(vw.st & GNSS_PT_NPI_ST_ACTIVE)) return "BYPASSED (TX1 = RX1)";
	if(!gated())                      return "IN (always-on mode, gnss_aj_mode=2/3)";
	if(fw_jam)                        return "IN (forced by the firmware's detector)";
	if(vw.st & GNSS_PT_NPI_ST_DET)    return "IN (FPGA power detector)";
	return "IN (FPGA detector hold time)";
}

void gnss_aj_print_short(void)
{
	console_print("JAMMER STATUS: %s | PL-NPI filter %s%s",
		      (char*)ajs_txt(aj_on ? ajs : AJS_UNKNOWN), (char*)filter_why(),
		      (char*)(aj_frozen ? ", weights FROZEN" : ""));
	if(aj_on && res_valid && res_meas && (vw.st & GNSS_PT_NPI_ST_ACTIVE))
		console_print(" | J/N %s dB, suppression %s dB, residual %s dB", f1(res.jn_db),
			      supp_txt(res.supp_db), f1u(res_disp(res.res_db)));
	if(vw.stat_reg & GNSS_PT_ST_NPI_DROP)
		console_print(" | SAMPLES DROPPED (STATUS[10])");
	console_print("\n");
}

void gnss_aj_print(void)
{
	gnss_aj_print_short();
	console_print("  mode        : %s\n", (char*)mode_name(aj_mode));
	if(!res_valid || !vw.valid) {
		console_print("  (no capture processed yet%s)\n", (char*)(aj_on ? "" : ": gnss_npi=1 starts it"));
		return;
	}
	console_print("  status      : %s for %s s; coherence %s dB (ON >= %s), power J/N %s (ON >= %s)\n",
		      (char*)ajs_txt(ajs), f1u((double)ms_since(ajs_t0, vw.t) / 1000.0),
		      f1(res.jn_coh_db < -30.0 ? -30.0 : res.jn_coh_db), f1(coh_on()),
		      (char*)(nf0 > 0.0 ? f1(res.jn_db < -9.9 ? -9.9 : res.jn_db) : "(learning)"), f1u(AJ_ON_DB));
	console_print("  more        : gnss_report prints everything (paste it into a report)\n");
}

void gnss_aj_report(void)
{
	XTime now;
	int i, first, k;
	uint32_t cfg = vw.cfg_act, live = vw.st;
	static const char *cmodes[4] = {"AUTO (detector)", "BYPASS", "ON (forced)", "HOLD"};
	int st = aj_on ? ajs : AJS_UNKNOWN;

	XTime_GetTime(&now);
	/* a snapshot older than the stale limit is not a measurement any more */
	if(aj_on && (!vw.valid || now - vw.t > us(AJ_ST_STALE_US))) st = AJS_UNKNOWN;

	console_print("\n===== GNSS-AJ REPORT (copy from here) =====\n");
	console_print("JAMMER STATUS: %s\n", (char*)ajs_txt(st));
	if(st == AJS_UNKNOWN) {
		if(!aj_on)
			console_print("  why          : anti-jam off (gnss_npi=1 starts it)\n");
		else if(!vw.valid)
			console_print("  why          : no capture processed yet\n");
		else
			console_print("  why          : no valid capture for %s ms (DMA errors %d, TX DDR replay %s)\n",
				      fmt((double)ms_since(vw.t, now), 0, 0), (long)n_fail,
				      (char*)(gnss_txdma_is_running() ? "RUNNING" : "off"));
	} else {
		console_print("  since        : t = %s s (%s s); %s\n",
			      f3u((double)ms_since(t_boot, ajs_t0) / 1000.0),
			      f1u((double)ms_since(ajs_t0, vw.t) / 1000.0),
			      (char*)(st == AJS_ON ? why_txt(ajs_why) : "every metric below its OFF threshold"));
		console_print("  rule         : ON after %d consecutive captures with evidence; OFF after %s s\n"
			      "                 with all metrics below the (lower) OFF thresholds\n",
			      (long)AJ_ST_ON_N, f1u((double)AJ_ST_OFF_US / 1e6));
	}
	if(!vw.valid) {
		console_print("===== END REPORT =====\n");
		return;
	}
	console_print("snapshot      : capture %d at t = %s s (%s ms old), %d captures/s; this report\n"
		      "                reads no register and runs no detector\n",
		      (long)n_cap, f3u((double)ms_since(t_boot, vw.t) / 1000.0),
		      fmt((double)ms_since(vw.t, now), 0, 0), (long)cap_rate);
	console_print("detection     : coherence %s at lag %d = %s dB (ON >= %s, quiet < %s)\n",
		      f3u(res.coh), (long)res.coh_lag, f1(res.jn_coh_db < -30.0 ? -30.0 : res.jn_coh_db),
		      f1(coh_on()), f1(coh_off()));
	console_print("                power J/N %s dB (ON >= %s with coherence >= %s, quiet < %s)\n",
		      (char*)(nf0 > 0.0 ? f1(res.jn_db < -99.0 ? -99.0 : res.jn_db) : "(no floor yet)"),
		      f1u(AJ_ON_DB), f1(AJ_PWR_COH_DB), f1u(AJ_OFF_DB));
	console_print("                RX gain cut %d dB (matched gain mode only), FPGA detector %s%s\n",
		      (long)cut_db(), (char*)((live & GNSS_PT_NPI_ST_DET) ? "ON" : "off"),
		      (char*)(hw_veto ? " (vetoed: not coherent)" : vw.hw_coh ? " (coherent)" : ""));
	console_print("                CW scan: ");
	{
		int any = 0;
		for(k = 0; k < NPI_CW_MAX; k++) {
			if(!cwt[k].used) continue;
			console_print("%s%s MHz +%s dB in bin (J/N %s dB), %s", (char*)(any ? "; " : ""),
				      f3u(rf_mhz(cwt[k].f)), f1u(cwt[k].snr1), f1(cw_jn(cwt[k].snr1)),
				      (char*)(cwt[k].conf ? "CONFIRMED" : "candidate"));
			any = 1;
		}
		if(!any) console_print("no narrowband peak >= %d dB in both antennas", (long)AJ_CW_DET_DB);
		console_print("\n");
	}
	console_print("powers        : RX1 %s, RX2 %s LSB^2 (DC removed); DC RX1 %s, RX2 %s LSB%s\n",
		      f1u(res.p1), f1u(res.p2), f1u(res.dc1), f1u(res.dc2),
		      (char*)((nf0 > 0.0 && res.dc1 * res.dc1 > noise_now()) ? " (LARGE)" : ""));
	if(coh_bg > -90.0)
		console_print("background    : RX1/RX2 common signal, coherence %s dB: not a jammer (splitter / one source / TX1 leak?)\n", f1(coh_bg));
	console_print("filter        : PL-NPI %s%s\n", (char*)filter_why(), (char*)(aj_frozen ? ", weights FROZEN" : ""));
	console_print("core          : mode %s, image taps %s, make-up shift %d (+%d dB), gamma %s (k = %s),\n"
		      "                detector threshold %s, power %s LSB^2, %d detector events\n",
		      (char*)cmodes[cfg & 3U], (char*)((cfg & GNSS_PT_NPI_CFG_IMAGE) ? "on" : "off"),
		      (long)((cfg >> 8) & 7U), (long)(6 * ((cfg >> 8) & 7U)), fmt((double)vw.gamma, 0, 0),
		      (char*)(gnss_npi_k_eighths() % 8U ? fmt((double)gnss_npi_k_eighths() / 8.0, 3, 0)
		                                        : fmt((double)(gnss_npi_k_eighths() / 8U), 0, 0)),
		      (char*)(thr_w == 0xFFFFFFFFU ? "not armed" : fmt((double)thr_w, 0, 0)),
		      fmt((double)vw.pow, 0, 0), (long)GNSS_PT_NPI_ST_CNT(live));
	if(core_v24) {
		console_print("dc blocker    : %s (gnss_aj_dc=)\n", (char*)((cfg & GNSS_PT_NPI_CFG_DCBLK) ? "ON" : "off"));
		console_print("cw notches    : %d in the core, auto %s (gnss_aj_notch=):", (long)core_notches,
			      (char*)(notch_auto ? "on" : "off"));
		for(k = 0; k < core_notches && k < NPI_CW_MAX; k++) {
			if(cfg & GNSS_PT_NPI_CFG_NOTCH(k))
				console_print(" [%d] %s MHz ON", (long)k, f3u(rf_mhz(notch_f[k])));
			else
				console_print(" [%d] off", (long)k);
		}
		console_print("\n");
		if(core_v25) {
			if(eq_on)
				console_print("rx2 equaliser : ON, taps (%s,%s) (%s,%s) (%s,%s); the jammer was %s dB over the noise, 3 taps null it %s dB deeper than one weight (gnss_aj_eq=)\n",
					      f3(eq_c[0][0]), f3(eq_c[0][1]), f3(eq_c[1][0]), f3(eq_c[1][1]), f3(eq_c[2][0]), f3(eq_c[2][1]),
					      f1(eq_jn_db), f1(eq_gain_db));
			else
				console_print("rx2 equaliser : off (%s; fitted while a jammer is there if it beats the single weight by %d dB; %d fit(s) so far)\n",
					      (char*)(eq_auto ? "gnss_aj_eq=1" : "gnss_aj_eq=0"), (long)AJ_EQ_ON_GAIN_DB, (long)eq_nfit);
			console_print("notch placement: in front of the loop on both channels, zero in Q1.15%s\n",
				      (char*)((cfg & GNSS_PT_NPI_CFG_NOTCH(0)) && (cfg & GNSS_PT_NPI_CFG_NOTCH(1)) && notch_f[0] == notch_f[1] ? "; one CW: both notches on it" : ""));
		}
		if(cw_confirmed() || txl_verdict)
			console_print("cw source     : %s\n",
				      (char*)(txl_state ? "testing now: TX1 muted for 0.7 s to see if the CW survives" :
				      txl_verdict == 2 ? "TX1 FEEDBACK into RX (the CW vanished with TX1 muted): NOT a jammer;\n"
				                         "                raise tx1_attenuation, lower the RX gain, separate the TX1 cable" :
				      txl_verdict == 1 ? "OUTSIDE source (the CW stayed with TX1 muted): a jammer" :
				                         "not tested (TX1 off): counted as a jammer"));
	} else {
		console_print("dc blocker    : not in this bitstream (%d.%d; v2.4 has it)\n", (long)(vw.ver >> 16), (long)(vw.ver & 0xFFFFU));
		console_print("cw notches    : not in this bitstream (v2.4 has them)\n");
	}
	if(jam && res_meas) {
		console_print("jammer        : J/N %s dB (peak %s), ", f1(res.jn_db), f1(jam_peak));
		print_type();
		console_print("\n");
		console_print("suppression   : %s dB, residual %s dB (0 = fully nulled)%s\n",
			      supp_txt(res.supp_db), f1u(res_disp(res.res_db)),
			      (char*)(res.res_db > AJ_WARN_RES_DB ? "  <-- jammer leaking" : ""));
		console_print("weights       : |w2| %s (x g2 %s), |c1| %s, |c2| %s (image taps)\n",
			      f3u(npi_sqrt(res.w[0] * res.w[0] + res.w[1] * res.w[1])), f3u(g2_core()),
			      f3u(npi_sqrt(res.w[2] * res.w[2] + res.w[3] * res.w[3])),
			      f3u(npi_sqrt(res.w[4] * res.w[4] + res.w[5] * res.w[5])));
	} else if(jam) {
		console_print("jammer        : J/N %s dB; the PL-NPI filter is bypassed (narrowband only:\n"
			      "                the notch handles it, power inversion cannot null a CW below the noise)\n",
			      f1(res.jn_db));
	}
	if(null_ms >= 0.0 && jam)
		console_print("time to null  : %s ms after detection (this jammer)\n", fmt(null_ms, 0, 0));
	else if(null_ms_last >= 0.0)
		console_print("time to null  : %s ms after detection (last jammer)\n", fmt(null_ms_last, 0, 0));
	console_print("gnss tracking : not available to the board (the receiver's tracking state does not\n"
		      "                reach the E310); read satellites, C/N0 and fix in the receiver\n");
	console_print("firmware      : %s, bitstream %d.%d, uptime %s s\n",
		      (char*)GNSS_AJ_VERSION_STR, (long)(vw.ver >> 16), (long)(vw.ver & 0xFFFFU),
		      f1u((double)ms_since(t_boot, now) / 1000.0));
	console_print("radio         : LO %s MHz, %s MS/s, RF bandwidth RX %s / TX %s MHz (gnss_band=%d)\n",
		      f3u((double)vw_lo / 1e6), f2u((double)vw_fs / 1e6), f1u((double)vw_rbw / 1e6),
		      f1u((double)vw_tbw / 1e6), (long)band);
	console_print("band          : ");
	print_band_cover();
	console_print("\n");
	console_print("tx            : %s, TX1 attenuation %s dB (operator %s dB, level hold +%d dB)",
		      (char*)(vw.txon ? "TRANSMITTING" : "silent"), f2u(vw.txatt),
		      f2u((double)gnss_rxg_att_base() / 1000.0), (long)gnss_rxg_comp_db());
	if(tx_noted == 1)
		console_print(", on for %s s", f1u((double)ms_since(tx_t0, vw.t) / 1000.0));
	console_print("\n");
	console_print("rx gain       : RX1 %d dB, RX2 %d dB, %s (reference %d), ADC peak %d of 2047\n",
		      (long)vw.g0, (long)vw.g1, (char*)gnss_rxg_mode_txt(), (long)gnss_rxg_ref(), (long)gnss_rxg_peak());
	console_print("normalisation : g2 = %s (%s dB) %s; RX2 noise after it = %s x RX1\n",
		      f3u(g2_core()), f1(npi_db10(g2_core() * g2_core())), (char*)g2_src_txt(), f2u(rx2n));
	if(nf0 > 0.0)
		console_print("noise floor   : RX1 %s LSB^2 at the present gain (learned)\n", f1u(noise_now()));
	else
		console_print("noise floor   : not learned yet (needs a moment with no jammer)\n");
	console_print("anti-jam      : %s, %s\n", (char*)(aj_on ? "ON" : "OFF"), (char*)mode_name(aj_mode));
	console_print("engine        : %d captures, %d skipped (gain step), %d DMA errors, %d short bursts,\n"
		      "                %d events held while you typed\n",
		      (long)n_cap, (long)n_skip, (long)n_fail, (long)n_burst, (long)n_held);
	if(aj_log_s > 0) console_print("printing      : status every %d s (gnss_aj_log=)", (long)aj_log_s);
	else             console_print("printing      : status line off (gnss_aj_log=30 to see it)");
	if(aj_csv_ms > 0) console_print(", CSV every %d ms (gnss_aj_csv=)", (long)aj_csv_ms);
	else              console_print(", CSV off");
	console_print(", events %s\n", (char*)(aj_events ? "printed" : "silent (gnss_aj_events=1)"));
	console_print("events        : %d in total, the last %d:\n", (long)ev_n, (long)(ev_n < 8 ? ev_n : 8));
	first = ev_n > 8 ? ev_n - 8 : 0;
	for(i = first; i < ev_n; i++) {
		struct aj_event *e = &ev[i % AJ_NEV];
		console_print("  %s s  ", f3u((double)e->ms / 1000.0));
		switch(e->type) {
		case EV_START:  console_print("anti-jam started\n"); break;
		case EV_TX:     console_print("TX1 %s, attenuation %s dB\n", (char*)(e->v1 > 0.5 ? "ON" : "OFF"), f2u(e->v2)); break;
		case EV_ON:     console_print("JAMMER ON   J/N %s dB, bandwidth %s of fs, by %s\n",
					      f1(e->v1), f2u(e->v2), (char*)why_txt((int)e->v3)); break;
		case EV_NULLED: console_print("NULLED      %s ms after detection, suppression %s dB, residual %s dB\n",
					      fmt(e->v1, 0, 0), supp_txt(e->v2), f1u(res_disp(e->v3))); break;
		case EV_OFF:    console_print("JAMMER %s after %s s, peak J/N %s dB\n",
					      (char*)(e->v3 > 0.5 ? "UNKNOWN (no measurement)" : "OFF "), f1u(e->v1), f1(e->v2)); break;
		case EV_LEAK:   console_print("LEAKING     residual %s dB at J/N %s dB\n", f1(e->v1), f1(e->v2)); break;
		case EV_FLOOR:  console_print("FLOOR       noise rose %s dB with no RX1/RX2 coherence: floor re-learned, not a jammer\n", f1(e->v1)); break;
		case EV_CW:
			if(e->v3 > 1.5)
				console_print("CW SOURCE   %s MHz: %s\n", f3u(e->v1),
					      (char*)(e->v2 < 1.5 ? "outside source (still there with TX1 muted): JAMMER"
					                            : "TX1 FEEDBACK into RX (gone with TX1 muted): not a jammer"));
			else
				console_print("CW %s   %s MHz, +%s dB in its FFT bin\n", (char*)(e->v3 > 0.5 ? "ON " : "OFF"),
					      f3u(e->v1), f1u(e->v2));
			break;
		case EV_EQ:     console_print("EQUALISER   RX2 taps fitted: jammer %s dB over the noise, %s dB deeper than one weight, %d CW(s) notched first\n",
					      f1(e->v2), f1(e->v1), (long)e->v3); break;
		case EV_BG:
			if(e->v1 > -90.0)
				console_print("BACKGROUND  RX1/RX2 common signal, coherence %s dB at power J/N %s dB: not a jammer\n",
					      f1(e->v1), f1(e->v2));
			else
				console_print("BACKGROUND  gone, normal detection again\n");
			break;
		case EV_BURST:  console_print("SHORT BURST the FPGA detector switched on and off between captures\n"); break;
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
struct band_cfg { uint64_t lo; uint32_t fs, bw; const char *name; };
static const struct band_cfg bands[GNSS_BAND_COUNT] = {
	{1575420000ULL, 30720000U, 18000000U, "GPS, Galileo, QZSS, SBAS, BeiDou B1C: L1 only (narrow)"},
	{1582500000ULL, 61440000U, 56000000U, "ALL: GPS, Galileo, QZSS, SBAS, BeiDou B1I + B1C, GLONASS G1"},
	{1589500000ULL, 50000000U, 40000000U, "GPS, Galileo, GLONASS G1 at 50 MS/s (no BeiDou B1I)"},
	{1568250000ULL, 50000000U, 40000000U, "GPS, Galileo, BeiDou B1I + B1C at 50 MS/s (no GLONASS)"},
	/* v2.4 default: the deployment's own configuration (LO 1570.25 MHz,
	 * 50 MS/s, 50 MHz RX and TX). Flat +-20 MHz: BeiDou B1I -9.15 MHz, GPS /
	 * Galileo / B1C +5.17 MHz; GLONASS (+27.8 MHz and up) is outside. The
	 * AD9361 TX analog filter tops out at 20 MHz baseband (40 MHz RF): the
	 * driver clamps it, the report shows the readback. */
	{1570250000ULL, 50000000U, 50000000U, "USER: GPS, Galileo, QZSS, SBAS, BeiDou B1I + B1C (no GLONASS)"},
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

int gnss_band_set(int b)
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
	band = b;
	band_applied = 1;
	view_radio();
	npi_cw_reset();
	eq_reset();
	{
		int k;
		for(k = 0; k < NPI_CW_MAX; k++) { cwt[k].used = 0; cwt[k].conf = 0; }
	}
	/* noise power at the ADC depends on the bandwidth: learn it again; the
	 * detector hold time depends on the rate: recommit the core settings */
	nf0 = -1.0;
	rho_n_re = rho_n_im = 0.0;
	ratio12 = -1.0; g2 = 1.0; rx2n = 1.0; g2_src = 0;
	img_bad = 0;
	img_cnt = 0;
	commit_core(1);
	return r ? -1 : 0;
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
		console_print("    %d = %s MHz, %s MS/s: %s%s\n", (long)i, f3u((double)bands[i].lo / 1e6),
			      f2u((double)bands[i].fs / 1e6), (char*)bands[i].name,
			      (char*)(i == GNSS_BAND_DEFAULT ? " (default)" : ""));
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
	   d3 > -1000 && d3 < 1000 && d4 > -1000 && d4 < 1000) {
		view_radio();
		return;
	}
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
}
