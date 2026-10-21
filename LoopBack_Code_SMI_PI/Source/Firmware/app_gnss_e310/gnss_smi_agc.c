/******************************************************************************
 *  gnss_smi_agc.c
 *  Matched RX gain control and TX level hold for SMI-PI. See gnss_smi_agc.h.
 *****************************************************************************/

#include "gnss_smi_agc.h"
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

static uint32_t att_base;        /* operator's TX1 attenuation [mdB]        */
static uint32_t att_written;     /* last value written by this loop         */
static int      att_holding;     /* 1 = att_written is in effect            */
static double   pout_ref;        /* clean TX output power [LSB^2]           */
static int      last_peak, last_comp_db;
static double   last_pout;

static int32_t abs16(uint32_t half)
{
	int32_t v = (int16_t)(half & 0xFFFFU);
	return v < 0 ? -v : v;
}

/* Peak |I|,|Q| of one RX channel over RXG_NSAMP snapshot reads. */
static int rx_peak(uint32_t reg)
{
	int peak = 0, i;
	for(i = 0; i < RXG_NSAMP; i++) {
		uint32_t s = gnss_pt_read(reg);
		int32_t a = abs16(s), b = abs16(s >> 16);
		if(a > peak) peak = a;
		if(b > peak) peak = b;
	}
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

static int set_both(int32_t db)
{
	if(ad9361_set_rx_rf_gain(ad9361_phy, 0, db) != 0) return -1;
	if(ad9361_set_rx_rf_gain(ad9361_phy, 1, db) != 0) return -1;
	rxg_gain = db;
	return 0;
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

static int tx_and_smi_on(void)
{
	uint32_t st = gnss_pt_read(GNSS_PT_REG_STATUS);
	return (st & GNSS_PT_ST_DAC_EN_I0) && (st & GNSS_PT_ST_DAC_EN_Q0) &&
	       (st & GNSS_PT_ST_PASS_EN_SYNCED) && (st & GNSS_PT_ST_SMI_EN_SYNCED);
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
	p0 = rx_peak(GNSS_PT_REG_RX_SNAPSHOT_CH0);
	p1 = rx_peak(GNSS_PT_REG_RX_SNAPSHOT_CH1);
	peak = p0 > p1 ? p0 : p1;
	last_peak = peak;

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

	/* ---- 2. TX level hold -------------------------------------------------- */
	ad9361_get_tx_attenuation(ad9361_phy, 0, &cur);
	if(cur != att_written) {          /* operator or gnss_tx changed it */
		att_base = cur;
		att_written = cur;
		att_holding = 0;
	}
	if(!tx_and_smi_on()) {
		/* TX off or SMI-PI out of the path: never keep a raised level. */
		gnss_rxg_stop_tx_hold();
		return;
	}
	last_pout = tx_power();
	if(rxg_gain >= rxg_ref) {
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
		/* Wanted: undo the RX gain cut. Allowed: never louder than the clean
		 * output + 3 dB (a jammer that is NOT nulled is never amplified). */
		double want = (double)(rxg_ref - rxg_gain);
		double room = db10(2.0 * pout_ref / (last_pout + 1e-9));
		double comp = want < room ? want : room;
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
	                               "MATCHED AUTO (SMI-PI)"};

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
		          ? " (matched, OK for SMI-PI)" : " (NOT matched)"));
	if(rxg_mode == GNSS_RXG_MATCHED)
		console_print("  reference %d dB, ADC peak %d of 2047, TX1 %d mdB "
			      "(operator %d mdB, level hold +%d dB)\n",
			      (long)rxg_ref, (long)last_peak, (long)att,
			      (long)att_base, (long)last_comp_db);
}
