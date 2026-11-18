/******************************************************************************
 *  gnss_aj_loopback.c  --  the LOOP-BACK build's stand-in for the anti-jam
 *  engine (gnss_aj.c). build_all.sh --algo loopback uses this file instead of
 *  gnss_aj.c and no CRPA solver: TX1 = RX1 and TX2 = RX2 through
 *  gnss_passthrough (CONTROL[5] is never set), with the same RF profiles
 *  (gnss_band=), radio guard, matched RX gain loop and console as the CRPA
 *  builds. Every anti-jam call is a no-op that says so.
 *****************************************************************************/

#include "gnss_aj.h"
#include "gnss_aj_math.h"
#include "crpa_build.h"
#include "gnss_smi_agc.h"
#include "gnss_boot_config.h"
#include "gnss_passthrough.h"
#include "gnss_txdma.h"
#include "ad9361_api.h"
#include "axi_dmac.h"
#include "console.h"
#include "xtime_l.h"
#include <string.h>

extern struct ad9361_rf_phy *ad9361_phy;

static XTime t_boot;
static int   band = GNSS_BAND_DEFAULT;
static void  print_band_cover(void);
static void  solver_reset(void) { }
/* what gnss_band_set() resets in the anti-jam build (learning state); unused here */
static __attribute__((unused)) double nf0, rho_n_re, rho_n_im, ratio12, g2, rx2n;
static __attribute__((unused)) int    g2_src;

/* ---- helpers --------------------------------------------------------------- */
static XTime us(uint32_t u)
{
	return (XTime)((uint64_t)COUNTS_PER_SECOND * u / 1000000U);
}

static uint32_t ms_since(XTime t0, XTime now)
{
	return (uint32_t)((uint64_t)(now - t0) * 1000U / COUNTS_PER_SECOND);
}

static __attribute__((unused)) double db2lin(double db)          /* 10^(db/10) without libm */
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
static __attribute__((unused)) double res_disp(double r) { return (r < 0.0) ? 0.0 : r; }

static __attribute__((unused)) char *supp_txt(double s)          /* ">60.0" when capped */
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

static void stamp(XTime now)
{
	console_print("[LB %s s] ", f3u((double)ms_since(t_boot, now) / 1000.0));
}

static void radio(uint32_t *fs, uint64_t *lo)
{
	*fs = 0; *lo = 0;
	if(!ad9361_phy) return;
	ad9361_get_rx_sampling_freq(ad9361_phy, fs);
	ad9361_get_rx_lo_freq(ad9361_phy, lo);
}

/* ---- the anti-jam API: none in this build -------------------------------------- */
static void none(void) { console_print("loop-back build: no anti-jam (TX1 = RX1, TX2 = RX2)\n"); }

void gnss_aj_init(void)            { XTime_GetTime(&t_boot); }
void gnss_aj_enable(int on)        { (void)on; gnss_pt_set_smi(0); }
void gnss_aj_command(int on)       { (void)on; gnss_pt_set_smi(0); none(); }
int  gnss_aj_enabled(void)         { return 0; }
int  gnss_aj_user_off(void)        { return 1; }
void gnss_aj_freeze(int on)        { (void)on; }
int  gnss_aj_set_mode(int m)       { (void)m; return -2; }
int  gnss_aj_set_direct(int n)     { (void)n; return -2; }
int  gnss_aj_set_track(int on)     { (void)on; return -2; }
int  gnss_aj_set_predict(int on)   { (void)on; return -2; }
int  gnss_aj_set_passes(int n)     { (void)n; return -2; }
int  gnss_aj_set_kalman(int on)    { (void)on; return -2; }
void gnss_aj_set_log(int s)        { (void)s; }
void gnss_aj_set_csv(int ms)       { (void)ms; }
void gnss_aj_gain_stepped(int d)   { (void)d; }
int  gnss_aj_jammer_on(void)       { return 0; }
void gnss_aj_poll(void)            { }
void gnss_aj_print_short(void)     { none(); }
void gnss_aj_print(void)           { none(); }
void gnss_aj_dma_release(void)     { }
void gnss_aj_report(void)
{
	console_print("\n===== LOOP-BACK REPORT =====\nbuild         : loop-back (no anti-jam), TX1 = RX1, TX2 = RX2\nband          : ");
	print_band_cover();
	console_print("\n===== END REPORT =====\n");
}
void gnss_aj_note_tx(int on)
{
	XTime now;
	XTime_GetTime(&now);
	stamp(now);
	console_print("TX1 %s\n", (char*)(on ? "ON (loop-back: TX1 = RX1)" : "OFF"));
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
static struct band_cfg bands[GNSS_BAND_COUNT] = {      /* [6] is written by gnss_band_custom= */
	{1575420000ULL, 30720000U, 18000000U, "GPS, Galileo, QZSS, SBAS, BeiDou B1C: L1 only (narrow)"},
	{GNSS_BOOT_LO_HZ, GNSS_BOOT_FS_HZ, GNSS_BOOT_BW_HZ, "BOOT PROFILE (gnss_boot_config.h): 1568.5 MHz, 30.72 MS/s, 20 MHz: GPS L1, Galileo E1, QZSS, SBAS, BeiDou B1I + B1C"},
	{1589500000ULL, 50000000U, 40000000U, "GPS, Galileo, GLONASS G1 at 50 MS/s (no BeiDou B1I)"},
	{1568250000ULL, 50000000U, 40000000U, "GPS, Galileo, BeiDou B1I + B1C at 50 MS/s (no GLONASS)"},
	/* v6.1: anti-jam profile. 8 MHz analog bandwidth keeps GPS C/A, Galileo
	 * E1 / BeiDou B1C main lobes (+-2.05 MHz) and cuts every jammer more
	 * than ~4 MHz away BEFORE the ADC, so the RX gain loop has to cut less
	 * and the satellites stay above the 12-bit quantisation floor. */
	{1575420000ULL, 30720000U,  8000000U, "GPS L1 C/A, Galileo E1, BeiDou B1C, QZSS, SBAS: ANTI-JAM narrow (8 MHz)"},
	/* v6.2: anti-jam against MULTIPATH. At 15.36 MS/s each tap spans 4x the
	 * time it does at 61.44 MS/s, so the same space-time taps cancel a
	 * reflection of the jammer with ~4x the delay (common delay for every
	 * satellite, absorbed by the receiver's clock bias). */
	{1575420000ULL, 15360000U,  6000000U, "GPS L1 C/A, Galileo E1, BeiDou B1C: ANTI-JAM multipath (15.36 MS/s, 6 MHz)"},
	/* v6.2.2: CUSTOM, set by gnss_band_custom=LO_MHz FS_MSPS BW_MHz. Until
	 * then it holds the default profile's values. Not kept over a reboot. */
	{1582500000ULL, 61440000U, 56000000U, "CUSTOM (gnss_band_custom=LO_MHz FS_MSPS BW_MHz)"},
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
	band = b;
	band_applied = 1;
	/* noise power at the ADC depends on the bandwidth: learn it again */
	nf0 = -1.0;
	rho_n_re = rho_n_im = 0.0;
	ratio12 = -1.0; g2 = 1.0; rx2n = 1.0; g2_src = 0;
	solver_reset();
	return r ? -1 : 0;
}

/* v6.2.2: custom profile. Limits: AD9361 LO 70..6000 MHz; rate 2.5..61.44
 * MS/s (61.44 = LVDS maximum; below ~2.1 MS/s the AD9361 needs its FIR, which
 * this design keeps off); RF bandwidth 0.2..56 MHz and not above the rate.
 * The filter core works at any of these rates (its taps are 1/fs apart). The values the AD9361 actually set are
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
	if(gnss_band_set(GNSS_BAND_CUSTOM) != 0) {
		bands[GNSS_BAND_CUSTOM] = old;
		if(prev == GNSS_BAND_CUSTOM) prev = GNSS_BAND_DEFAULT;
		gnss_band_set(prev);
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
		console_print("    %d = %s MHz, %s MS/s, %s MHz: %s%s\n", (long)i, f3u((double)bands[i].lo / 1e6),
			      f2u((double)bands[i].fs / 1e6), f1u((double)bands[i].bw / 1e6), (char*)bands[i].name,
			      (char*)(i == GNSS_BAND_DEFAULT ? " (default)" : ""));
	console_print("  custom: gnss_band_custom=LO_MHz FS_MSPS BW_MHz, e.g. gnss_band_custom=1575.42 30.72 18\n");
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
	gnss_band_set(band);
}

/* ---- the console idle hook ------------------------------------------------------- */
void gnss_idle(void)
{
	gnss_rxg_poll();
	gnss_aj_poll();
	radio_guard();
}
