/******************************************************************************
 *  gnss_npi_agc.c
 *  Matched RX gain control, automatic gamma and TX level hold for PL-NPI.
 *  See gnss_npi_agc.h.
 *****************************************************************************/

#include "gnss_npi_agc.h"
#include "gnss_passthrough.h"
#include "ad9361_api.h"
#include "console.h"
#include "xtime_l.h"

extern struct ad9361_rf_phy *ad9361_phy;

#define RXG_PERIOD_US      20000U   /* one measurement every 20 ms           */
#define RXG_NSAMP            512    /* snapshot reads per channel            */
#define RXG_UP_AFTER           3    /* quiet periods (60 ms) before +1 dB    */
#define RXG_MIN_DB             1
#define RXG_ATT_STEP_MDB    1000U   /* only rewrite TX attenuation if the    */
                                    /* target moved by at least 1 dB         */

static int      rxg_mode = GNSS_RXG_AD9361;   /* power-on: AD9361 AGC       */
static int32_t  rxg_gain;                     /* present gain, both channels */
static int32_t  rxg_ref;                      /* clean (no-jammer) gain      */
static int      rxg_quiet;
static XTime    rxg_next;

static uint32_t att_base;        /* operator's TX1 attenuation [mdB]        */
static uint32_t att_written;     /* last value written by this loop         */
static int      att_holding;     /* 1 = att_written is in effect            */
static double   pout_ref;        /* clean TX output power [LSB^2]           */
static int      last_peak, last_comp_db;
static double   last_pout;

/* ---- automatic gamma ----------------------------------------------------
 * gamma = 2^17 * k * noise power per channel (mean I^2+Q^2, LSB^2). The noise
 * floor is tracked only while no jammer is present and is rescaled with every
 * RX gain step.
 *
 * k itself is chosen automatically from the jammer state. C model (pl_npi.cpp,
 * noise +-20 LSB, satellite on both elements), output SINR in dB:
 *
 *          no jammer   jammer 60   jammer 300   jammer 1500 LSB
 *   k = 1    -10.1       -9.7        -7.1         -7.0
 *   k = 4     -9.8      -14.3        -8.4         -7.0
 *   k = 32    -9.1      -20.2       -20.1        -10.5
 *   RX1       -8.9      -20.1       -33.2        -44.5
 *
 * No single k is good for both: a large k passes the satellites untouched
 * when there is no jammer, k = 1 gives the deepest null for weak and strong
 * jammers alike. So k = 32 while the input is clean and k = 1 from the moment
 * a jammer is detected (input > 6 dB above the noise floor, or the ADC near
 * clipping); back to 32 after 200 ms clean. */
#define K_CLEAN_Q   128U     /* k in quarter units: 32   */
#define K_JAM_Q       4U     /*                      1   */
#define JAM_OFF_AFTER 10     /* clean polls (200 ms) before leaving jammer mode */
static int      gam_auto = 1;
static int      k_auto = 1;
static uint32_t k_fixed_q = 16U; /* manual k (gnss_npi_k=N), quarter units */
static int      jam_on, jam_quiet;
static double   nfloor;          /* noise power per channel [LSB^2]          */
static double   last_prx;
static uint32_t gam_written;
/* Noise power per channel at 0 dB gain, learned while no jammer is present
 * (noise in LSB^2 follows the RX gain). Kept across gnss_npi=0/1, so a start
 * with the jammer already on uses the last clean value instead of the
 * jammer power. */
static double   nf_per_0db;

static int32_t abs16(uint32_t half)
{
	int32_t v = (int16_t)(half & 0xFFFFU);
	return v < 0 ? -v : v;
}

/* Peak |I|,|Q| of one RX channel over RXG_NSAMP snapshot reads; *pwr gets
 * the mean I^2+Q^2. */
static int rx_peak(uint32_t reg, double *pwr)
{
	int peak = 0, i;
	double acc = 0.0;
	for(i = 0; i < RXG_NSAMP; i++) {
		uint32_t s = gnss_pt_read(reg);
		int32_t a = abs16(s), b = abs16(s >> 16);
		double re = (int16_t)(s & 0xFFFFU), im = (int16_t)(s >> 16);
		acc += re * re + im * im;
		if(a > peak) peak = a;
		if(b > peak) peak = b;
	}
	*pwr = acc / RXG_NSAMP;
	return peak;
}

/* Mean I^2+Q^2 of the TX1 output (what goes to the DAC). */
static double tx_power(void)
{
	double acc = 0.0;
	int i;
	for(i = 0; i < RXG_NSAMP; i++) {
		uint32_t s = gnss_pt_read(GNSS_PT_REG_TX_SNAPSHOT_CH0);
		double re = (int16_t)(s & 0xFFFFU), im = (int16_t)(s >> 16);
		acc += re * re + im * im;
	}
	return acc / RXG_NSAMP;
}

/* 10^(db/10) for whole dB, without libm. */
static double lin10(int32_t db)
{
	double r = 1.0;
	while(db > 0) { r *= 1.258925412; db--; }
	while(db < 0) { r /= 1.258925412; db++; }
	return r;
}

static int set_both(int32_t db)
{
	int32_t old = rxg_gain;
	if(ad9361_set_rx_rf_gain(ad9361_phy, 0, db) != 0) return -1;
	if(ad9361_set_rx_rf_gain(ad9361_phy, 1, db) != 0) return -1;
	rxg_gain = db;
	if(nfloor > 0.0 && old != db)
		nfloor *= lin10(db - old);      /* noise in LSB follows the gain */
	return 0;
}

static double cur_kq(void)
{
	if(!k_auto) return (double)k_fixed_q;
	return (double)(jam_on ? K_JAM_Q : K_CLEAN_Q);
}

static void write_gamma(void)
{
	double g;
	uint32_t gi;
	if(!gam_auto || nfloor <= 0.0) return;
	g = 131072.0 * cur_kq() / 4.0 * nfloor;
	if(g < 1.0e5)          g = 1.0e5;
	if(g > 4.0e9)          g = 4.0e9;
	gi = (uint32_t)g;
	/* rewrite only on a change of more than ~0.5 dB */
	if(gam_written == 0U || gi > gam_written + gam_written / 8U ||
	   gi < gam_written - gam_written / 8U) {
		gnss_pt_set_npi_gamma(gi);
		gam_written = gi;
	}
}

/* Noise floor: follows the input while it is within 3 dB of the floor
 * (or below it); holds while a jammer lifts the power above that. */
static void track_floor(double p)
{
	if(p <= 0.0) return;
	if(nfloor <= 0.0 || p < nfloor)
		nfloor = (nfloor <= 0.0) ? p : 0.5 * nfloor + 0.5 * p;
	else if(p < 2.0 * nfloor)
		nfloor = 0.95 * nfloor + 0.05 * p;
	else
		return;                         /* jammer present: hold */
	nf_per_0db = nfloor / lin10(rxg_gain);
}

static void manual_both(void)
{
	ad9361_set_rx_gain_control_mode(ad9361_phy, 0, RF_GAIN_MGC);
	ad9361_set_rx_gain_control_mode(ad9361_phy, 1, RF_GAIN_MGC);
}

/* 10*log10(x) without libm. */
static double db10(double x)
{
	double r = 0.0;
	if(x <= 0.0) return -200.0;
	while(x >= 10.0) { x /= 10.0; r += 10.0; }
	while(x < 1.0)   { x *= 10.0; r -= 10.0; }
	/* 1 <= x < 10: ln(x) by atanh series, * 10/ln(10) */
	{
		double y = (x - 1.0) / (x + 1.0), y2 = y * y, t = y, s = 0.0;
		int k;
		for(k = 1; k < 30; k += 2) { s += t / k; t *= y2; }
		r += 2.0 * s * 4.342944819;
	}
	return r;
}

static int tx_and_npi_on(void)
{
	uint32_t st = gnss_pt_read(GNSS_PT_REG_STATUS);
	return (st & GNSS_PT_ST_DAC_EN_I0) && (st & GNSS_PT_ST_DAC_EN_Q0) &&
	       (st & GNSS_PT_ST_PASS_EN_SYNCED) && (st & GNSS_PT_ST_NPI_EN_SYNCED);
}

void gnss_rxg_stop_tx_hold(void)
{
	uint32_t cur = 0;
	if(!ad9361_phy || !att_holding) return;
	ad9361_get_tx_attenuation(ad9361_phy, 0, &cur);
	if(cur == att_written)            /* nobody else changed it meanwhile */
		ad9361_set_tx_attenuation(ad9361_phy, 0, att_base);
	att_holding = 0;
	last_comp_db = 0;
}

static void start_loop(void)
{
	uint32_t cur = 0;
	int32_t g = 0;

	ad9361_get_rx_rf_gain(ad9361_phy, 0, &g);   /* AGC's clean choice */
	manual_both();
	set_both(g);
	rxg_ref = g;
	rxg_quiet = 0;
	ad9361_get_tx_attenuation(ad9361_phy, 0, &cur);
	att_base = cur;
	att_written = cur;
	att_holding = 0;
	pout_ref = 0.0;
	jam_on = 0;
	jam_quiet = 0;
	{
		double p0, p1;
		rx_peak(GNSS_PT_REG_RX_SNAPSHOT_CH0, &p0);
		rx_peak(GNSS_PT_REG_RX_SNAPSHOT_CH1, &p1);
		double p = 0.5 * (p0 + p1);
		/* A remembered clean floor wins if the input is already well above
		 * it (jammer on at switch-on). */
		if(nf_per_0db > 0.0) {
			double known = nf_per_0db * lin10(rxg_gain);
			nfloor = (p > 2.0 * known) ? known : p;
			jam_on = (p > 4.0 * known);
		} else {
			nfloor = 0.0;
			track_floor(p);
		}
		gam_written = 0U;
		write_gamma();
	}
	XTime_GetTime(&rxg_next);
}

void gnss_rxg_set_mode(int mode)
{
	if(!ad9361_phy) return;
	if(rxg_mode == GNSS_RXG_MATCHED && mode != GNSS_RXG_MATCHED)
		gnss_rxg_stop_tx_hold();

	if(mode == GNSS_RXG_AD9361) {
		ad9361_set_rx_gain_control_mode(ad9361_phy, 0, RF_GAIN_SLOWATTACK_AGC);
		ad9361_set_rx_gain_control_mode(ad9361_phy, 1, RF_GAIN_SLOWATTACK_AGC);
	} else if(mode == GNSS_RXG_MANUAL) {
		int32_t g = 0;
		ad9361_get_rx_rf_gain(ad9361_phy, 0, &g);
		manual_both();
		set_both(g);
	} else if(mode == GNSS_RXG_MATCHED && rxg_mode != GNSS_RXG_MATCHED) {
		start_loop();
	}
	rxg_mode = mode;
}

int gnss_rxg_mode(void) { return rxg_mode; }

void gnss_npi_gamma_auto(int on)
{
	gam_auto = on ? 1 : 0;
	if(gam_auto) { gam_written = 0U; write_gamma(); }
}

int gnss_npi_gamma_is_auto(void) { return gam_auto; }

void gnss_npi_gamma_k(uint32_t k)
{
	if(k == 0U) {
		k_auto = 1;                     /* 32 clean, 1 with a jammer */
	} else {
		if(k > 64U) k = 64U;
		k_auto = 0;
		k_fixed_q = 4U * k;
	}
	gam_written = 0U;
	write_gamma();
}

void gnss_rxg_set_gain(int32_t db)
{
	if(!ad9361_phy) return;
	if(rxg_mode == GNSS_RXG_AD9361) rxg_mode = GNSS_RXG_MANUAL;
	manual_both();
	if(set_both(db) == 0) {
		rxg_ref = db;
		rxg_quiet = 0;
		pout_ref = 0.0;                 /* re-learn the clean output level */
	}
}

void gnss_rxg_poll(void)
{
	XTime now;
	int p0, p1, peak;
	uint32_t cur = 0;

	if(rxg_mode != GNSS_RXG_MATCHED || !ad9361_phy) return;
	XTime_GetTime(&now);
	if(now < rxg_next) return;
	rxg_next = now + (XTime)((uint64_t)COUNTS_PER_SECOND * RXG_PERIOD_US / 1000000U);

	/* ---- 1. matched RX gain ------------------------------------------------ */
	{
		double w0, w1;
		p0 = rx_peak(GNSS_PT_REG_RX_SNAPSHOT_CH0, &w0);
		p1 = rx_peak(GNSS_PT_REG_RX_SNAPSHOT_CH1, &w1);
		last_prx = 0.5 * (w0 + w1);
	}
	peak = p0 > p1 ? p0 : p1;
	last_peak = peak;
	if(peak <= GNSS_RXG_PEAK_HIGH)          /* clipped data says nothing */
		track_floor(last_prx);

	/* Jammer state (drives k): on at once, off after 200 ms clean. */
	{
		int was = jam_on;
		if(peak > GNSS_RXG_PEAK_HIGH || (nfloor > 0.0 && last_prx > 4.0 * nfloor)) {
			jam_on = 1;
			jam_quiet = 0;
		} else if(jam_on) {
			if(last_prx < 2.0 * nfloor) {
				if(++jam_quiet >= JAM_OFF_AFTER) jam_on = 0;
			} else {
				jam_quiet = 0;
			}
		}
		if(jam_on != was) gam_written = 0U;   /* new k: rewrite gamma now */
	}

	if(peak > GNSS_RXG_PEAK_HIGH) {
		/* Down by enough to bring the peak to ~1200; 6 dB if clipping. */
		int32_t step = 6;
		if(peak < 2040) {
			double lvl = peak;
			step = 0;
			while(step < 6 && lvl > 1200.0) { lvl /= 1.122; step++; }  /* 1 dB */
			if(step < 1) step = 1;
		}
		{
			int32_t g = rxg_gain - step;
			if(g < RXG_MIN_DB) g = RXG_MIN_DB;
			if(g != rxg_gain) set_both(g);
		}
		rxg_quiet = 0;
	} else if(peak < GNSS_RXG_PEAK_LOW && rxg_gain < rxg_ref) {
		if(++rxg_quiet >= RXG_UP_AFTER) {
			set_both(rxg_gain + 1);
			rxg_quiet = 0;
		}
	} else {
		rxg_quiet = 0;
	}

	write_gamma();

	/* ---- 2. TX level hold -------------------------------------------------- */
	ad9361_get_tx_attenuation(ad9361_phy, 0, &cur);
	if(cur != att_written) {          /* operator or gnss_tx changed it */
		att_base = cur;
		att_written = cur;
		att_holding = 0;
	}
	if(!tx_and_npi_on()) {
		/* TX off or PL-NPI out of the path: never keep a raised level. */
		gnss_rxg_stop_tx_hold();
		return;
	}
	last_pout = tx_power();
	if(!jam_on && rxg_gain >= rxg_ref) {
		/* Clean state: learn the reference output power slowly. Not while
		 * the ADC is near clipping, and not from a sudden jump (a jammer that
		 * has just appeared), so the reference stays the clean level. */
		if(peak <= GNSS_RXG_PEAK_HIGH) {
			if(pout_ref <= 0.0)
				pout_ref = last_pout;
			else if(last_pout < 4.0 * pout_ref)
				pout_ref = 0.9 * pout_ref + 0.1 * last_pout;
		}
		if(att_holding) gnss_rxg_stop_tx_hold();
		return;
	}
	if(pout_ref <= 0.0) return;       /* no clean reference yet: do nothing */
	{
		/* Bring the output back to the clean output power: this undoes the
		 * RX gain cut AND the level the null itself takes (C model: satellite
		 * -3 to -6 dB at k = 1). It can never make the output louder than
		 * the clean level, so a jammer that is NOT nulled is never
		 * amplified. */
		double comp = db10(pout_ref / (last_pout + 1e-9));
		uint32_t target, floor_mdb;
		if(comp < 0.0) comp = 0.0;
		if(comp * 1000.0 > (double)GNSS_RXG_MAX_COMP_MDB)
			comp = GNSS_RXG_MAX_COMP_MDB / 1000.0;
		floor_mdb = att_base > GNSS_RXG_MAX_COMP_MDB ? att_base - GNSS_RXG_MAX_COMP_MDB : 0U;
		target = att_base - (uint32_t)(comp * 1000.0);
		target = (target / 250U) * 250U;           /* AD9361 0.25 dB steps */
		if(target < floor_mdb) target = floor_mdb;
		last_comp_db = (int)comp;
		if((target > att_written ? target - att_written : att_written - target)
		        >= RXG_ATT_STEP_MDB || (target == att_base && att_holding)) {
			if(ad9361_set_tx_attenuation(ad9361_phy, 0, target) == 0) {
				att_written = target;
				att_holding = (target != att_base);
			}
		}
	}
}

void gnss_rxg_print(void)
{
	uint8_t m1 = 0, m2 = 0;
	int32_t g1 = 0, g2 = 0;
	uint32_t att = 0;
	static const char *names[3] = {"MANUAL (locked)", "AD9361 AGC (per channel)",
	                               "MATCHED AUTO (PL-NPI)"};

	if(!ad9361_phy) return;
	ad9361_get_rx_gain_control_mode(ad9361_phy, 0, &m1);
	ad9361_get_rx_gain_control_mode(ad9361_phy, 1, &m2);
	ad9361_get_rx_rf_gain(ad9361_phy, 0, &g1);
	ad9361_get_rx_rf_gain(ad9361_phy, 1, &g2);
	ad9361_get_tx_attenuation(ad9361_phy, 0, &att);
	console_print("GNSS_RX_GAIN: mode %s\n", (char*)names[rxg_mode]);
	console_print("  RX1 %s %d dB, RX2 %s %d dB%s\n",
		      (char*)(m1 == RF_GAIN_MGC ? "manual" : "AGC"), (long)g1,
		      (char*)(m2 == RF_GAIN_MGC ? "manual" : "AGC"), (long)g2,
		      (char*)((m1 == RF_GAIN_MGC && m2 == RF_GAIN_MGC && g1 == g2)
		          ? " (matched, OK for PL-NPI)" : " (NOT matched)"));
	if(rxg_mode == GNSS_RXG_MATCHED)
		console_print("  reference %d dB, ADC peak %d of 2047, TX1 %d mdB "
			      "(operator %d mdB, level hold +%d dB)\n",
			      (long)rxg_ref, (long)last_peak, (long)att,
			      (long)att_base, (long)last_comp_db);
	console_print("  gamma %s: %d%d, k = %d %s, noise floor %d LSB^2, RX power %d LSB^2\n",
		      (char*)(gam_auto ? "AUTO" : "manual"),
		      /* gamma can exceed 2^31; console_print has only signed %d */
		      (long)(gnss_pt_read(GNSS_PT_REG_NPI_GAMMA) / 10U),
		      (long)(gnss_pt_read(GNSS_PT_REG_NPI_GAMMA) % 10U),
		      (long)(cur_kq() / 4.0),
		      (char*)(k_auto ? "(auto)" : "(fixed)"),
		      (long)nfloor, (long)last_prx);
	console_print("  jammer: %s\n", (char*)(jam_on ? "DETECTED (deep-null k = 1)"
	                                               : "none (satellite k = 32)"));
}
