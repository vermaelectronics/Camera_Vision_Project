/******************************************************************************
 *  gnss_aj.c  --  SMI-PI v4.0 anti-jam engine. See gnss_aj.h.
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

extern struct ad9361_rf_phy *ad9361_phy;
extern struct axi_dmac *rx_dmac;

#define AJ_BYTES          (AJ_NSAMP * 8U)        /* 2 channels x 4 bytes        */
#define AJ_MIN_PERIOD_US  2000U                  /* at most ~500 captures/s     */
#define AJ_DMA_TIMEOUT_US 50000U
#define AJ_ON_DB          6.0    /* jammer ON above this J/N ...                */
#define AJ_ON_COUNT       2      /* ... for this many updates in a row          */
#define AJ_OFF_DB         3.0    /* jammer OFF below this J/N ...               */
#define AJ_OFF_US         500000U /* ... for this long                          */
#define AJ_CUT_DB         3      /* an RX gain cut this large is a jammer too   */
#define AJ_NF_RISE_DB_S   0.05   /* learned noise floor may creep up this fast  */
#define AJ_WARN_RES_DB    3.0    /* residual above this while jammed ...         */
#define AJ_WARN_US        1000000U /* ... for this long: warn                    */
#define AJ_ANNOUNCE_US    100000U /* JAMMER ON is printed this long after onset,  */
                                  /* when the gain loop has absorbed the jammer: */
                                  /* the first captures can still be clipped     */

/* ---- state --------------------------------------------------------------- */
static int      aj_on, aj_user_off, aj_frozen, aj_mode = AJ_MODE_STAP;
static int      aj_log_s = 5;
static int      dma_busy;
static XTime    dma_t0, next_start, t_rate, t_log, t_warn, t_learn;
static uint32_t gain_mark;
static aj_result_t res;
static int      res_valid;

static uint32_t n_upd, n_fail, n_skip, rate_cnt, upd_rate;
static int      jam, jam_cnt, jam_announced;
static XTime    jam_t0, quiet_t0, res_bad_t0;
static double   jam_peak;

/* receiver noise floor, normalised to 0 dB RX gain [LSB^2 / linear gain] */
static double   nf0 = -1.0;
static double   rho_n_re, rho_n_im;

static int      band = GNSS_BAND_DEFAULT;

/* ---- helpers --------------------------------------------------------------- */
static XTime us(uint32_t u)
{
	return (XTime)((uint64_t)COUNTS_PER_SECOND * u / 1000000U);
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

/* "+31.2"-style text for console_print (which has no float format). Four
 * rotating buffers so one print can carry several values. */
static char *f1(double v)
{
	static char buf[4][24];
	static int  idx;
	char *b = buf[idx++ & 3], tmp[16];
	double a = (v < 0.0) ? -v : v;
	long t, ip;
	int n = 0, i = 0;
	if(a > 99999999.0) a = 99999999.0;
	t = (long)(a * 10.0 + 0.5);
	ip = t / 10;
	b[i++] = (v < 0.0 && t) ? '-' : '+';
	do { tmp[n++] = (char)('0' + ip % 10); ip /= 10; } while(ip);
	while(n) b[i++] = tmp[--n];
	b[i++] = '.';
	b[i++] = (char)('0' + t % 10);
	b[i] = 0;
	return b;
}

/* residual for display: below 0 dB just means "at the noise", show 0.0 */
static double res_disp(double r) { return (r < 0.0) ? 0.0 : r; }

static char *f1u(double v)               /* same, without the + sign */
{
	char *b = f1(v);
	return (b[0] == '+') ? b + 1 : b;
}

static int32_t cur_gain(void)  { return gnss_rxg_gain(); }

/* Noise at the ADC for the present RX gain: learned floor x gain, plus the
 * 12-bit quantisation floor. <= 0 while nothing has been learned. */
static double noise_now(void)
{
	if(nf0 <= 0.0) return -1.0;
	return nf0 * db2lin((double)cur_gain()) + AJ_QFLOOR;
}

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

static void print_line(void)
{
	uint32_t att = 0;
	if(ad9361_phy) ad9361_get_tx_attenuation(ad9361_phy, 0, &att);
	if(jam) {
		console_print("[AJ] JAMMED  J/N %s dB  suppression %s%s dB  residual %s dB | ",
			      f1(res.jn_db), (char*)(res.supp_db >= 59.9 ? ">" : ""),
			      f1u(res.supp_db >= 59.9 ? 60.0 : res.supp_db), f1(res_disp(res.res_db)));
		print_type();
	} else {
		console_print("[AJ] clear   noise %s LSB^2 (J/N %s dB)",
			      f1u(res.noise), f1(res.jn_db < -9.9 ? -9.9 : res.jn_db));
	}
	console_print(" | RX %d/%d dB  TX att %d.%d dB | %d upd/s%s\n",
		      (long)cur_gain(), (long)gnss_rxg_ref(), (long)(att / 1000U),
		      (long)((att % 1000U) / 100U), (long)upd_rate,
		      (char*)(aj_frozen ? " FROZEN" : ""));
}

/* ---- the engine -------------------------------------------------------------- */
static void learn_noise(XTime now)
{
	/* Only with no jammer and the RX gain at its clean reference: then RX1's
	 * power IS the receiver noise (the satellites are ~30 dB under it). The
	 * learned floor follows it down at once and up only slowly, so a jammer
	 * that creeps in cannot drag the floor up with it. */
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
}

static void jammer_state(XTime now)
{
	int cut = (int)(gnss_rxg_ref() - cur_gain());
	int raw = (res.jn_db >= AJ_ON_DB) ||
	          (gnss_rxg_mode() == GNSS_RXG_MATCHED && cut >= AJ_CUT_DB);
	int quiet = (res.jn_db < AJ_OFF_DB) &&
	            !(gnss_rxg_mode() == GNSS_RXG_MATCHED && cut > 0);

	if(!jam) {
		jam_cnt = raw ? jam_cnt + 1 : 0;
		if(jam_cnt >= AJ_ON_COUNT) {
			jam = 1;
			jam_announced = 0;
			jam_t0 = quiet_t0 = res_bad_t0 = now;
			jam_peak = res.jn_db;
		}
		if(quiet && gnss_rxg_ref() == cur_gain())
			learn_noise(now);
	} else {
		if(!jam_announced && now - jam_t0 >= us(AJ_ANNOUNCE_US)) {
			jam_announced = 1;
			jam_peak = res.jn_db;
			console_print("\n[AJ] *** JAMMER ON ***  J/N %s dB, ", f1(res.jn_db));
			print_type();
			console_print(", RX gain cut %d dB, suppression %s%s dB, residual %s dB\n",
				      (long)(cut > 0 ? cut : 0),
				      (char*)(res.supp_db >= 59.9 ? ">" : ""),
				      f1u(res.supp_db >= 59.9 ? 60.0 : res.supp_db), f1(res_disp(res.res_db)));
		}
		if(jam_announced && res.jn_db > jam_peak) jam_peak = res.jn_db;
		if(!quiet) {
			quiet_t0 = now;
		} else if(now - quiet_t0 >= us(AJ_OFF_US)) {
			uint64_t ms = (uint64_t)(now - jam_t0) * 1000U / COUNTS_PER_SECOND;
			jam = 0; jam_cnt = 0;
			if(jam_announced)
				console_print("\n[AJ] jammer OFF after %d.%d s (peak J/N %s dB)\n",
					      (long)(ms / 1000U), (long)((ms % 1000U) / 100U), f1(jam_peak));
		}
		if(res.res_db <= AJ_WARN_RES_DB)
			res_bad_t0 = now;
		if(jam && jam_announced && now - res_bad_t0 >= us(AJ_WARN_US) &&
		   now - t_warn >= us(5000000U)) {
			t_warn = now;
			console_print("[AJ] WARNING residual %s dB: the jammer is not fully nulled.\n"
				      "     Two antennas null ONE direction (plus its reflections). Check that\n"
				      "     RX2 has its antenna, that there is a single jammer, and the ADC peak\n"
				      "     (gnss_rx_gain?).%s\n", f1(res.res_db),
				      (char*)(aj_mode == AJ_MODE_STAP ? "" : " gnss_aj_mode=0 uses all 31 taps."));
		}
	}
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

	res.mode = aj_mode;
	res.noise_ref = noise_now();
	res.rho_noise_re = rho_n_re;
	res.rho_noise_im = rho_n_im;
	if(aj_compute((const uint32_t *)GNSS_AJ_BUF_ADDR, AJ_NSAMP, &res) != 0 || !res.ok) {
		n_fail++;
		return;
	}
	res.sig2_prev = res.sig2;
	res_valid = 1;
	if(!aj_frozen)
		gnss_pt_write_weights(res.wre, res.wim);
	n_upd++;
	rate_cnt++;
	XTime_GetTime(&now);
	if(now - t_rate >= us(1000000U)) {
		upd_rate = rate_cnt;
		rate_cnt = 0;
		t_rate = now;
	}
	jammer_state(now);
	if(aj_log_s > 0 && now - t_log >= us((uint32_t)aj_log_s * 1000000U)) {
		t_log = now;
		print_line();
	}
}

/* ---- control ------------------------------------------------------------------ */
void gnss_aj_init(void)
{
	XTime now;
	XTime_GetTime(&now);
	t_rate = t_log = t_warn = t_learn = next_start = now;
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
	int16_t z[AJ_NW] = {0};
	if(on) {
		if(gnss_pt_read(GNSS_PT_REG_VERSION) < 0x00040000U) {
			console_print("gnss_aj: this bitstream has no SMI-PI v4 filter; anti-jam NOT started\n");
			return;
		}
		/* The weights assume both channels always have the same gain: two
		 * independent AD9361 AGCs would step them apart. A manual gain the
		 * operator locked (gnss_rx_agc=0) is matched too and is kept. */
		if(gnss_rxg_mode() == GNSS_RXG_AD9361)
			gnss_rxg_set_mode(GNSS_RXG_MATCHED);
		if(!aj_on) {
			gnss_pt_write_weights(z, z);        /* start from TX1 = RX1 */
			aj_reset();
			res.sig2_prev = -1.0;
			jam = 0; jam_cnt = 0;
		}
		aj_on = 1;
		gnss_pt_set_smi(1);
	} else {
		gnss_aj_dma_release();
		aj_on = 0;
		gnss_rxg_stop_tx_hold();
		gnss_pt_set_smi(0);
		if(jam) {
			jam = 0;
			console_print("[AJ] anti-jam switched off (jammer state no longer monitored)\n");
		}
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
int  gnss_aj_jammer_on(void) { return jam; }

void gnss_aj_freeze(int on)  { aj_frozen = on ? 1 : 0; }

void gnss_aj_set_mode(int mode)
{
	if(mode < AJ_MODE_STAP || mode > AJ_MODE_NB) return;
	aj_mode = mode;
	aj_reset();                   /* the old taps are not comparable */
}

void gnss_aj_set_log(int seconds)
{
	if(seconds < 0) seconds = 0;
	if(seconds > 3600) seconds = 3600;
	aj_log_s = seconds;
}

void gnss_aj_print_short(void)
{
	static const char *mnames[3] = {"space-time 31 taps", "RX2 16 taps", "one weight (v3)"};
	console_print("GNSS_SMI: anti-jam %s, %s%s, jammer %s",
		      (char*)(aj_on ? "ON" : "off"), (char*)mnames[aj_mode],
		      (char*)(aj_frozen ? ", weights FROZEN" : ""),
		      (char*)(!aj_on ? "not monitored" : jam ? "ON" : "none"));
	if(aj_on && res_valid)
		console_print(", J/N %s dB, suppression %s%s dB", f1(res.jn_db),
			      (char*)(res.supp_db >= 59.9 ? ">" : ""), f1u(res.supp_db >= 59.9 ? 60.0 : res.supp_db));
	if(gnss_pt_read(GNSS_PT_REG_STATUS) & GNSS_PT_ST_SMI_DROP)
		console_print(", SAMPLES DROPPED (STATUS[10])");
	console_print("\n");
}

void gnss_aj_print(void)
{
	static const char *mnames[3] = {"space-time: RX1 15 + RX2 16 taps (gnss_aj_mode=0)",
	                                "RX2 16 taps only (gnss_aj_mode=1)",
	                                "one weight, the old v3 SMI-PI (gnss_aj_mode=2)"};
	uint32_t fs, bw = 0, att = 0;
	uint64_t lo;
	XTime now;
	int k, kmax = 0;
	double wmax = 0.0;

	gnss_aj_print_short();
	console_print("  filter      : %s\n", (char*)mnames[aj_mode]);
	if(!res_valid) {
		console_print("  (no capture processed yet%s)\n", (char*)(aj_on ? "" : ": gnss_smi=1 starts it"));
		return;
	}
	XTime_GetTime(&now);
	if(jam) {
		uint64_t ms = (uint64_t)(now - jam_t0) * 1000U / COUNTS_PER_SECOND;
		console_print("  jammer      : ON for %d.%d s, J/N %s dB (peak %s), ",
			      (long)(ms / 1000U), (long)((ms % 1000U) / 100U), f1(res.jn_db), f1(jam_peak));
		print_type();
		console_print("\n");
	} else {
		console_print("  jammer      : none (J/N %s dB; ON above %s dB)\n",
			      f1(res.jn_db < -9.9 ? -9.9 : res.jn_db), f1u(AJ_ON_DB));
	}
	console_print("  suppression : %s%s dB, residual %s dB above the output noise (0 = fully nulled)%s\n",
		      (char*)(res.supp_db >= 59.9 ? ">" : ""), f1u(res.supp_db >= 59.9 ? 60.0 : res.supp_db),
		      f1(res_disp(res.res_db)), (char*)(res.res_db > AJ_WARN_RES_DB ? "  <-- jammer leaking" : ""));
	console_print("  power       : RX1 %s, RX2 %s, output %s LSB^2\n",
		      f1u(res.p1), f1u(res.p2), f1u(res.pout));
	console_print("  noise       : %s LSB^2 (%s), loading %s LSB^2 (automatic)\n",
		      f1u(res.noise), (char*)(nf0 > 0 ? "learned floor" : "from the filter output"),
		      f1u(res.sig2));
	for(k = 0; k < AJ_NW; k++) {
		double m = (double)res.wre[k] * res.wre[k] + (double)res.wim[k] * res.wim[k];
		if(m > wmax) { wmax = m; kmax = k; }
	}
	console_print("  weights     : |w|^2 %s, largest tap %s delay %d (|w| %s)\n",
		      f1u(res.wn2), (char*)(kmax < AJ_NT ? "RX1" : "RX2"), (long)(kmax % AJ_NT),
		      f1u(aj_sqrt(wmax) / (double)(1 << AJ_WFRAC)));
	console_print("  engine      : %d updates/s, %d updates, %d skipped (gain step), %d DMA errors, log every %d s\n",
		      (long)upd_rate, (long)n_upd, (long)n_skip, (long)n_fail, (long)aj_log_s);
	if(ad9361_phy) ad9361_get_tx_attenuation(ad9361_phy, 0, &att);
	console_print("  RX gain     : %d dB both channels (reference %d), TX1 attenuation %d.%d dB\n",
		      (long)cur_gain(), (long)gnss_rxg_ref(), (long)(att / 1000U), (long)((att % 1000U) / 100U));
	radio(&fs, &lo);
	if(ad9361_phy) ad9361_get_rx_rf_bandwidth(ad9361_phy, &bw);
	console_print("  radio       : LO %d.%d MHz, %d.%d MS/s, RF bandwidth %d.%d MHz (gnss_band=%d)\n",
		      (long)(lo / 1000000U), (long)((lo % 1000000U) / 1000U),
		      (long)(fs / 1000000U), (long)((fs % 1000000U) / 10000U),
		      (long)(bw / 1000000U), (long)((bw % 1000000U) / 100000U), (long)band);
}

/* ---- RF profiles ------------------------------------------------------------------ */
struct band_cfg { uint64_t lo; uint32_t fs, bw; const char *name; };
static const struct band_cfg bands[3] = {
	{1575420000ULL, 30720000U, 18000000U, "GPS/Galileo/QZSS L1 (narrow)"},
	{1584500000ULL, 50000000U, 56000000U, "multi-GNSS: GPS, Galileo, GLONASS G1, BeiDou B1I"},
	{1584500000ULL, 61440000U, 56000000U, "multi-GNSS, AD9361 maximum rate"},
};

int gnss_band_get(void) { return band; }

int gnss_band_set(int b)
{
	const struct band_cfg *c;
	int r = 0;
	if(b < 0 || b > 2 || !ad9361_phy) return -1;
	c = &bands[b];
	gnss_aj_dma_release();
	r |= ad9361_set_rx_lo_freq(ad9361_phy, c->lo);
	r |= ad9361_set_tx_lo_freq(ad9361_phy, c->lo);
	r |= ad9361_set_rx_sampling_freq(ad9361_phy, c->fs);
	r |= ad9361_set_tx_sampling_freq(ad9361_phy, c->fs);
	r |= ad9361_set_rx_rf_bandwidth(ad9361_phy, c->bw);
	r |= ad9361_set_tx_rf_bandwidth(ad9361_phy, c->bw);
	band = b;
	/* noise power at the ADC depends on the bandwidth: learn it again */
	nf0 = -1.0;
	rho_n_re = rho_n_im = 0.0;
	res.sig2_prev = -1.0;
	aj_reset();
	return r ? -1 : 0;
}

void gnss_band_print(void)
{
	uint32_t fs; uint64_t lo;
	radio(&fs, &lo);
	console_print("GNSS_BAND: %d = %s\n", (long)band, (char*)bands[band].name);
	console_print("  LO %d.%d MHz (RX and TX), %d.%d MS/s\n",
		      (long)(lo / 1000000U), (long)((lo % 1000000U) / 1000U),
		      (long)(fs / 1000000U), (long)((fs % 1000000U) / 10000U));
	console_print("  0 = 1575.42 MHz 30.72 MS/s 18 MHz, 1 = 1584.5 MHz 50 MS/s 56 MHz (default),\n"
		      "  2 = 1584.5 MHz 61.44 MS/s 56 MHz\n");
}

/* ---- the console idle hook ------------------------------------------------------- */
void gnss_idle(void)
{
	gnss_rxg_poll();
	gnss_aj_poll();
}
