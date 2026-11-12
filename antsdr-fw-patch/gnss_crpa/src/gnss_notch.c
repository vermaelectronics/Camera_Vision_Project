/******************************************************************************
 * gnss_notch.c -- automatic CW notch (antsdr-fw-patch crpa5). See gnss_notch.h.
 *****************************************************************************/
#include "gnss_notch.h"
#include "gnss_notch_math.h"
#include "gnss_aj_math.h"
#include "gnss_aj.h"
#include "gnss_passthrough.h"
#include "ad9361_api.h"
#include "console.h"
#include "xtime_l.h"

extern struct ad9361_rf_phy *ad9361_phy;

#define NT_CHECK_US    50000U      /* one tone search every 50 ms            */
#define NT_ON_HITS     3           /* checks in a row before switching on    */
#define NT_OFF_US      2000000U    /* tone gone this long: switch off        */
#define NT_SAME_F      (2.0 / NT_FFT)  /* "same tone" between checks [x fs]  */
#define NT_RETUNE_F    2.0e-6      /* move the notch when off by more [x fs] */

static int      nt_hw;                     /* bitstream has the notch        */
static int      nt_mode = GNSS_NOTCH_AUTO;
static int      nt_is_on;
static uint32_t nt_pinc_cur;
static double   nt_f_cur, nt_f_cand, nt_rf_fixed;
static int      nt_hits, nt_changed;
static XTime    nt_t_check, nt_t_seen, nt_t_on, nt_t_boot;
static uint32_t nt_n_on, nt_n_retune, nt_n_checks, nt_n_found;
static nt_tone_t nt_last;
static uint32_t nt_buf[2 * AJ_NSAMP];

static XTime us(uint32_t u) { return (XTime)((uint64_t)COUNTS_PER_SECOND * u / 1000000U); }
static double secs(XTime t) { return (double)(uint64_t)t / (double)COUNTS_PER_SECOND; }

static void radio_now(uint32_t *fs, uint64_t *lo)
{
	*fs = 0; *lo = 0;
	if(!ad9361_phy) return;
	ad9361_get_rx_sampling_freq(ad9361_phy, fs);
	ad9361_get_rx_lo_freq(ad9361_phy, lo);
}

static double rf_mhz(double f)
{
	uint32_t fs; uint64_t lo;
	radio_now(&fs, &lo);
	return (double)lo / 1e6 + f * (double)fs / 1e6;
}

static void hw_set(int en, uint32_t pinc)
{
	if(!nt_hw) return;
	gnss_pt_write(GNSS_PT_REG_AJ_NTC_PINC, pinc);
	gnss_pt_write(GNSS_PT_REG_AJ_NTC_CTRL, en ? 1U : 0U);
	/* the weight shadows always hold the last committed set, so this commits the notch alone */
	gnss_pt_write(GNSS_PT_REG_AJ_COMMIT, 1U);
	nt_changed = 1;
}

static void stamp(XTime now)
{
	console_print("\n[NT %s s] ", gnss_fmt(secs(now - nt_t_boot), 3));
}

static void switch_on(XTime now, double f, const char *why)
{
	nt_f_cur = f;
	nt_pinc_cur = nt_pinc(f);
	nt_is_on = 1;
	nt_t_on = now;
	nt_n_on++;
	hw_set(1, nt_pinc_cur);
	stamp(now);
	console_print("*** CW NOTCH ON *** at RF %s MHz (%s MHz from the LO)%s\n",
		      gnss_fmt(rf_mhz(f), 4), gnss_fmt(rf_mhz(f) - rf_mhz(0.0), 3), (char*)why);
	if(nt_mode == GNSS_NOTCH_AUTO) {
		if(nt_last.tone_db > -100.0)
			console_print("     tone %s dB above the noise, ", gnss_fmt(nt_last.tone_db, 1));
		else
			console_print("     tone ");
		console_print("%s dB above the rest of the band; the space-time filter now\n"
			      "     works on what the notch leaves (gnss_notch? for details, gnss_notch=0 off)\n",
			      gnss_fmt(nt_last.peak_db, 1));
	}
}

static void switch_off(XTime now, const char *why)
{
	if(!nt_is_on) return;
	nt_is_on = 0;
	hw_set(0, nt_pinc_cur);
	stamp(now);
	console_print("CW NOTCH OFF after %s s (%s)\n", gnss_fmt(secs(now - nt_t_on), 1), (char*)why);
}

int gnss_notch_available(void) { return nt_hw; }
int gnss_notch_on(void) { return nt_is_on; }

void gnss_notch_init(void)
{
	uint32_t info = gnss_pt_read(GNSS_PT_REG_AJ_INFO);
	XTime_GetTime(&nt_t_boot);
	nt_hw = (info & 0xFFU) >= GNSS_PT_AJ_LAYOUT_NOTCH;
	nt_is_on = 0; nt_hits = 0;
	if(nt_hw) hw_set(0, 0U);
	if(nt_hw && nt_mode == GNSS_NOTCH_FIXED) gnss_notch_set_rf(nt_rf_fixed);
}

void gnss_notch_off(void)
{
	XTime now;
	XTime_GetTime(&now);
	nt_hits = 0;
	if(nt_mode == GNSS_NOTCH_FIXED && nt_hw) {
		gnss_notch_set_rf(nt_rf_fixed);      /* LO or rate may have changed */
		return;
	}
	switch_off(now, "anti-jam or radio changed");
}

int gnss_notch_take_change(void)
{
	int c = nt_changed;
	nt_changed = 0;
	return c;
}

const uint32_t *gnss_notch_capture(const uint32_t *raw, int nsamp, double noise)
{
	XTime now;
	if(!nt_hw || nt_mode == GNSS_NOTCH_OFF) return raw;
	XTime_GetTime(&now);
	if(nt_mode == GNSS_NOTCH_AUTO && now - nt_t_check >= us(NT_CHECK_US)) {
		nt_t_check = now;
		nt_n_checks++;
		if(nt_find_tone(raw, nsamp, noise, &nt_last)) {
			nt_n_found++;
			nt_t_seen = now;
			if(!nt_is_on) {
				double d = nt_last.f - nt_f_cand;
				if(d < 0) d = -d;
				nt_hits = (nt_hits > 0 && d < NT_SAME_F) ? nt_hits + 1 : 1;
				nt_f_cand = nt_last.f;
				if(nt_hits >= NT_ON_HITS)
					switch_on(now, nt_last.f, "");
			} else {
				double d = nt_last.f - nt_f_cur;
				if(d < 0) d = -d;
				if(d > NT_RETUNE_F) {
					nt_f_cur = nt_last.f;
					nt_pinc_cur = nt_pinc(nt_last.f);
					nt_n_retune++;
					hw_set(1, nt_pinc_cur);
				}
			}
		} else {
			nt_hits = 0;
			if(nt_is_on && now - nt_t_seen >= us(NT_OFF_US))
				switch_off(now, "no CW tone for 2 s");
		}
	}
	if(!nt_is_on) return raw;
	nt_apply(raw, nt_buf, nsamp, nt_pinc_cur);
	return nt_buf;
}

void gnss_notch_set_mode(int mode)
{
	XTime now;
	XTime_GetTime(&now);
	if(mode < GNSS_NOTCH_OFF || mode > GNSS_NOTCH_FIXED) return;
	if(mode == GNSS_NOTCH_FIXED) {
		if(nt_rf_fixed <= 0.0) {
			console_print("gnss_notch=2 needs a frequency: use gnss_notch_rf=<RF MHz>, e.g. gnss_notch_rf=1575.42\n");
			return;
		}
		gnss_notch_set_rf(nt_rf_fixed);
		return;
	}
	nt_mode = mode;
	nt_hits = 0;
	if(mode == GNSS_NOTCH_OFF || nt_is_on)
		switch_off(now, mode == GNSS_NOTCH_OFF ? "gnss_notch=0" : "automatic again");
}

int gnss_notch_set_rf(double rf)
{
	uint32_t fs; uint64_t lo;
	double f;
	XTime now;
	XTime_GetTime(&now);
	radio_now(&fs, &lo);
	if(!nt_hw) {
		console_print("gnss_notch_rf: this bitstream has no CW notch (crpa5: AJ_INFO layout 8)\n");
		return -1;
	}
	if(fs == 0) {
		console_print("gnss_notch_rf: the radio reports no sample rate yet\n");
		return -1;
	}
	f = (rf * 1e6 - (double)lo) / (double)fs;
	if(f <= -0.5 || f >= 0.5) {
		console_print("gnss_notch_rf: %s MHz is outside the received band (LO %s MHz +- %s MHz)\n",
			      gnss_fmt(rf, 3), gnss_fmt((double)lo / 1e6, 3), gnss_fmt((double)fs / 2e6, 2));
		return -1;
	}
	nt_rf_fixed = rf;
	nt_mode = GNSS_NOTCH_FIXED;
	nt_last.tone_db = -200.0; nt_last.peak_db = 0.0;
	if(nt_is_on && f == nt_f_cur) return 0;
	nt_is_on = 0;
	switch_on(now, f, " (fixed: gnss_notch_rf)");
	return 0;
}

static const char *mode_txt(void)
{
	return nt_mode == GNSS_NOTCH_OFF ? "OFF (gnss_notch=0)" :
	       nt_mode == GNSS_NOTCH_AUTO ? "AUTOMATIC (gnss_notch=1)" : "FIXED (gnss_notch_rf)";
}

void gnss_notch_report_line(void)
{
	if(!nt_hw) {
		console_print("cw notch      : not in this bitstream (needs AJ_INFO layout 8, crpa5)\n");
		return;
	}
	if(nt_is_on)
		console_print("cw notch      : ON at RF %s MHz (%s MHz from the LO), %s; %lu switch-ons, %lu re-tunes\n",
			      gnss_fmt(rf_mhz(nt_f_cur), 4), gnss_fmt(rf_mhz(nt_f_cur) - rf_mhz(0.0), 3), (char*)mode_txt(),
			      (unsigned long)nt_n_on, (unsigned long)nt_n_retune);
	else
		console_print("cw notch      : off, %s; no CW tone%s\n", (char*)mode_txt(),
			      (char*)(nt_mode == GNSS_NOTCH_AUTO ? " (searching every 50 ms)" : ""));
}

void gnss_notch_print(void)
{
	uint32_t fs; uint64_t lo;
	radio_now(&fs, &lo);
	console_print("GNSS_NOTCH: %s, %s\n", (char*)mode_txt(),
		      (char*)(!nt_hw ? "NOT in this bitstream (build crpa5: smi_pi 2.6, AJ_INFO layout 8)" :
		              nt_is_on ? "notch ON" : "notch off"));
	if(!nt_hw) return;
	if(nt_is_on)
		console_print("  notch       : RF %s MHz, %s MHz from the LO %s MHz (NCO step 0x%x), width about %s kHz\n",
			      gnss_fmt(rf_mhz(nt_f_cur), 4), gnss_fmt(rf_mhz(nt_f_cur) - (double)lo / 1e6, 3),
			      gnss_fmt((double)lo / 1e6, 3), (unsigned long)nt_pinc_cur,
			      gnss_fmt(0.886 * (double)fs / NT_L / 1e3, 0));
	console_print("  detector    : %lu checks, %lu with a CW tone; last: %s", (unsigned long)nt_n_checks,
		      (unsigned long)nt_n_found, (char*)(nt_last.found ? "CW tone" : "no tone"));
	if(nt_last.found) {
		console_print(" at RF %s MHz", gnss_fmt(rf_mhz(nt_last.f), 4));
		if(nt_last.tone_db > -100.0)
			console_print(", %s dB above the noise", gnss_fmt(nt_last.tone_db, 1));
	}
	console_print(" (peak %s dB over the median bin, %s dB over its shoulders;\n"
		      "                a tone needs %s and %s dB)\n",
		      gnss_fmt(nt_last.peak_db, 1), gnss_fmt(nt_last.shoulder_db, 1),
		      gnss_fmt(NT_DET_DB, 0), gnss_fmt(NT_SHOULDER_DB, 0));
	console_print("  commands    : gnss_notch=1 automatic (default), gnss_notch=0 off, gnss_notch_rf=<RF MHz> fixed\n");
}
