/******************************************************************************
 *  gnss_npi_agc.c
 *  Matched RX gain control, automatic gamma and TX level hold for PL-NPI.
 *  See gnss_npi_agc.h.
 *****************************************************************************/

#include "gnss_npi_agc.h"
#include "gnss_npi_aj.h"
#include "gnss_passthrough.h"
#include "ad9361_api.h"
#include "console.h"
#include "xtime_l.h"

extern struct ad9361_rf_phy *ad9361_phy;

#define RXG_PERIOD_US       5000U   /* one measurement every 5 ms            */
#define RXG_NSAMP            256    /* snapshot reads per channel            */
#define RXG_UP_AFTER           4    /* quiet polls (20 ms) before going up   */
#define RXG_TARGET_PEAK    1450.0   /* where a down step aims the peak       */
#define RXG_MAX_STEP_DB       12    /* largest single down step              */
#define RXG_MIN_DB             1
#define RXG_ATT_STEP_MDB    1000U   /* only rewrite TX attenuation if the    */
                                    /* target moved by at least 1 dB         */

static int      rxg_mode = GNSS_RXG_AD9361;   /* power-on: AD9361 AGC       */
static int32_t  rxg_gain;                     /* present gain, both channels */
static int32_t  rxg_ref;                      /* clean (no-jammer) gain      */
static int      rxg_quiet;
static XTime    rxg_next;
static uint32_t rxg_changes;                  /* gain steps, for the engine  */
static int      rxg_check;                    /* polls since the mode check  */

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
 * when there is no jammer, a small k gives the deepest null. So k = 32 while
 * the input is clean and a small k from the moment a jammer is detected
 * (input > 6 dB above the noise floor, or the ADC near clipping); back to 32
 * after 200 ms clean.
 *
 * Core v2.1.10 (rounded output, quarter step): with a jammer, k = 1/8 is best
 * at every jammer level, C model output SINR in dB:
 *
 *          jammer 60   150    300    1500 LSB
 *   k = 1/8   -8.1    -7.1   -7.0   -7.0
 *   k = 1    -11.6    -8.6   -7.4   -6.9
 *   SMI-PI    -7.2    -6.9   -6.9   -6.8 */
#define K_CLEAN_Q   256U     /* k in eighth units: 32    */
#define K_JAM_Q       1U     /*                    1/8   */
#define JAM_OFF_AFTER 40     /* clean polls (200 ms) before leaving jammer mode */
static int      gam_auto = 1;
static int      k_auto = 1;
static int      gated = 1;       /* v2.3: core adapts only with a jammer    */
static uint32_t k_fixed_q = 32U; /* manual k (gnss_npi_k=N), eighth units */
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
	rxg_changes++;
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
	if(gated)   return (double)K_JAM_Q;   /* adapts only with a jammer */
	return (double)(jam_on ? K_JAM_Q : K_CLEAN_Q);
}

static void write_gamma(void)
{
	double g;
	uint32_t gi;
	if(!gam_auto || nfloor <= 0.0) return;
	g = 131072.0 * cur_kq() / 8.0 * nfloor;
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
		nfloor = 0.9875 * nfloor + 0.0125 * p;     /* tau ~400 ms at 5 ms */
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
	/* If a jammer was already on when the loop started, the AGC's choice
	 * is the jammed gain and would become the "clean" reference. Never
	 * start the reference below GNSS_RXG_REF_MIN_DB: the fast attack takes
	 * the gain down again within milliseconds if the ADC is too hot. */
	if(g < GNSS_RXG_REF_MIN_DB) g = GNSS_RXG_REF_MIN_DB;
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
		/* v2.4: both channels from the same start gain, then the AGC */
		manual_both();
		ad9361_set_rx_rf_gain(ad9361_phy, 0, GNSS_RXG_AGC_START_DB);
		ad9361_set_rx_rf_gain(ad9361_phy, 1, GNSS_RXG_AGC_START_DB);
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

const char *gnss_rxg_mode_txt(void)
{
	return rxg_mode == GNSS_RXG_MATCHED ? "matched loop (gnss_rx_agc=2)" :
	       rxg_mode == GNSS_RXG_AD9361  ? "AD9361 slow-attack AGC per channel (gnss_rx_agc=1)" :
	                                      "manual (gnss_rx_agc=0)";
}

void gnss_npi_gamma_noise(double n1, int jammed)
{
	if(rxg_mode == GNSS_RXG_MATCHED || n1 <= 0.0) return;
	nfloor = n1;
	if(jam_on != (jammed ? 1 : 0)) gam_written = 0U;
	jam_on = jammed ? 1 : 0;
	write_gamma();
}

int32_t gnss_rxg_gain(void)
{
	int32_t g = rxg_gain;
	if(rxg_mode == GNSS_RXG_AD9361 && ad9361_phy)
		ad9361_get_rx_rf_gain(ad9361_phy, 0, &g);
	return g;
}

int32_t gnss_rxg_ref(void)
{
	return (rxg_mode == GNSS_RXG_AD9361) ? gnss_rxg_gain() : rxg_ref;
}

uint32_t gnss_rxg_changes(void)  { return rxg_changes; }
int      gnss_rxg_peak(void)     { return last_peak; }
uint32_t gnss_rxg_att_base(void) { return att_base; }
int      gnss_rxg_comp_db(void)  { return att_holding ? last_comp_db : 0; }
uint32_t gnss_npi_k_eighths(void) { return (uint32_t)cur_kq(); }

void gnss_npi_set_gated(int on)
{
	gated = on ? 1 : 0;
	gam_written = 0U;
	write_gamma();
}

/* Matched mode only: both channels must stay in manual gain at rxg_gain.
 * Anything else changing them (the vendor rx1_gc_mode= / rx2_rf_gain=
 * commands, or a GUI that sends them -- e.g. "GC Mode 2 slow attack AGC")
 * would let the AD9361 step the two channels independently and break the
 * null. Put them back and say so. */
static void enforce_matched(void)
{
	uint8_t m1 = 0, m2 = 0;
	int32_t g1 = 0, g2 = 0;
	ad9361_get_rx_gain_control_mode(ad9361_phy, 0, &m1);
	ad9361_get_rx_gain_control_mode(ad9361_phy, 1, &m2);
	ad9361_get_rx_rf_gain(ad9361_phy, 0, &g1);
	ad9361_get_rx_rf_gain(ad9361_phy, 1, &g2);
	if(m1 == RF_GAIN_MGC && m2 == RF_GAIN_MGC && g1 == rxg_gain && g2 == rxg_gain)
		return;
	console_print("[AJ] RX gain was changed outside the matched loop (RX1 %s %d dB, "
		      "RX2 %s %d dB) -> restored: both manual at %d dB.\n"
		      "     Use gnss_rx_gain= / gnss_rx_agc= instead of rx1_gc_mode= etc.\n",
		      (char*)(m1 == RF_GAIN_MGC ? "manual" : "AGC"), (long)g1,
		      (char*)(m2 == RF_GAIN_MGC ? "manual" : "AGC"), (long)g2, (long)rxg_gain);
	manual_both();
	set_both(rxg_gain);
}

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
		k_fixed_q = 8U * k;
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
	if(++rxg_check >= 40) {            /* every ~200 ms */
		rxg_check = 0;
		enforce_matched();
	}

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
		/* Fast attack. Clipping hides how strong the jammer really is, so
		 * the gain drops by the largest step (12 dB) and is checked again
		 * 5 ms later: a jammer 80 dB above the noise needs ~4 steps, i.e.
		 * ~20 ms (it took ~200 ms with 6 dB per 20 ms). Below clipping the
		 * step brings the peak to ~1450 of 2047, keeping the ADC well
		 * loaded: every dB of headroom left unused is a dB of dynamic range
		 * lost against a strong jammer. */
		int32_t step = RXG_MAX_STEP_DB;
		if(peak < 2040) {
			double lvl = peak;
			step = 0;
			while(step < RXG_MAX_STEP_DB && lvl > RXG_TARGET_PEAK) { lvl /= 1.122; step++; }  /* 1 dB */
			if(step < 1) step = 1;
		}
		{
			int32_t g = rxg_gain - step;
			if(g < RXG_MIN_DB) g = RXG_MIN_DB;
			if(g != rxg_gain) set_both(g);
		}
		rxg_quiet = 0;
	} else if(peak < GNSS_RXG_PEAK_LOW && rxg_gain < rxg_ref) {
		/* Fast release: after 20 ms below the low mark, jump up by as much
		 * as the measured peak allows (towards ~1450, 1 dB margin, at most
		 * 12 dB, never above the clean reference). With the jammer gone the
		 * gain is back at the reference in ~100 ms instead of ~3 s, so the
		 * satellites do not sit in the ADC's quantisation noise for long.
		 * A jammer that returns is caught again by the fast attack. */
		if(++rxg_quiet >= RXG_UP_AFTER) {
			double lvl = (peak < 1) ? 1.0 : (double)peak;
			int32_t up = 0;
			while(up < RXG_MAX_STEP_DB && lvl * 1.122 < RXG_TARGET_PEAK) { lvl *= 1.122; up++; }
			if(up > 1) up -= 1;
			if(up < 1) up = 1;
			if(rxg_gain + up > rxg_ref) up = rxg_ref - rxg_gain;
			set_both(rxg_gain + up);
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
	if(!jam_on && !gnss_aj_jammer_on() && rxg_gain >= rxg_ref) {
		/* Clean state: learn the reference output power slowly. Not while
		 * the ADC is near clipping, and not from a sudden jump (a jammer that
		 * has just appeared), so the reference stays the clean level. */
		if(peak <= GNSS_RXG_PEAK_HIGH) {
			if(pout_ref <= 0.0)
				pout_ref = last_pout;
			else if(last_pout < 4.0 * pout_ref)
				pout_ref = 0.975 * pout_ref + 0.025 * last_pout;
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
		/* ... and never louder than the firmware's 30 dB safe-start level
		 * (or the operator's own value if that is already lower). */
		if(floor_mdb < GNSS_RXG_MIN_ATT_MDB)
			floor_mdb = (att_base < GNSS_RXG_MIN_ATT_MDB) ? att_base : GNSS_RXG_MIN_ATT_MDB;
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
	{
		long kq = (long)cur_kq();       /* k in eighths: "32" or "1/8" */
		console_print("  gamma %s: %d%d, k = %d%s %s, noise floor %d LSB^2, RX power %d LSB^2\n",
			      (char*)(gam_auto ? "AUTO" : "manual"),
			      /* gamma can exceed 2^31; console_print has only signed %d */
			      (long)(gnss_pt_read(GNSS_PT_REG_NPI_GAMMA) / 10U),
			      (long)(gnss_pt_read(GNSS_PT_REG_NPI_GAMMA) % 10U),
			      (kq % 8 == 0) ? kq / 8 : kq, (char*)((kq % 8 == 0) ? "" : "/8"),
			      (char*)(k_auto ? "(auto)" : "(fixed)"),
			      (long)nfloor, (long)last_prx);
	}
	console_print("  jammer: %s\n", (char*)(gated ? "k follows the gated core (adapts only with a jammer, k = 1/8)"
	                                     : jam_on ? "DETECTED (deep-null k = 1/8)"
	                                              : "none (satellite k = 32)"));
}
