/***************************************************************************//**
 *   @file   gnss_ant.c
 *   @brief  The console of the LVDS / ant board: three switches, one main
 *           menu, four pages, one self-test. Everything is read from ONE
 *           snapshot (gm_read) so every page shows the same numbers.
 *
 *   Figures and where they come from (the only source of each):
 *     RX gain           AD9361 (both channels)
 *     ADC peak / clip   MVDR window registers (COV_PEAK / COV_CLIP)
 *     noise floor       anti-jam report: thermal floor per element at the
 *                       present gain (learned clean floor + SMI lambda_min)
 *     internal floor    AD9361 floor measured at start (gnss_rx_floor_dbfs)
 *     J/N in            jammer / noise at RX1 (reference element), noise sn2
 *                       = lambda_min / (1 - sqrt(N/K))^2 (gnss_smi_null_analysis)
 *     J/N out           w^H (R - sn2 I) w / (sn2 |w|^2) at the array output
 *     SUPPRESSION       S = (J/N)in / (J/N)out  (standard definition)
 *     C/N0 cost         10 log10(1 + (J/N)out)
 *     output            MVDR OUT_LVL (AOLC target -24 dBFS)
 *
 *   The engine modules (MVDR2) print into a capture buffer (gnss_quiet.h);
 *   a FAILED step prints its captured detail.
*******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "app_config.h"
#include "ad9361_api.h"
#include "command.h"
#include "gnss_ant.h"
#include "gnss_l1.h"
#include "gnss_passthrough.h"
#include "gnss_autostart.h"
#include "gnss_config.h"
#include "gnss_smi.h"
#include "gnss_report.h"
#include "gnss_quiet.h"
#include "no_os_delay.h"

extern struct ad9361_rf_phy *ad9361_phy;

#define AOLC_TARGET_DBFS   (-24.0)
#define NA                 (-999.0)

/* ======================================================= state read-back */
int gnss_ant_tx_state(void)
{
	uint32_t st, a = 0;

	if (!ad9361_phy)
		return 0;
	st = gnss_pt_read(GNSS_PT_REG_STATUS);
	if (ad9361_get_tx_attenuation(ad9361_phy, 0, &a) != 0)
		return 0;
	return (a < GNSS_L1_TX_MAX_ATTEN_MDB) &&
	       (st & GNSS_PT_ST_DAC_EN_I0) && (st & GNSS_PT_ST_DAC_EN_Q0);
}

int gnss_ant_pass_state(void)
{
	uint32_t st = gnss_pt_read(GNSS_PT_REG_STATUS);
	uint32_t ct = gnss_pt_read(GNSS_PT_REG_CONTROL);

	return gnss_ant_tx_state() && (st & GNSS_PT_ST_PASS_EN_SYNCED) &&
	       !(ct & GNSS_PT_CTRL_MUTE);
}

int gnss_ant_stap_state(void)
{
	return gnss_ant_pass_state() &&
	       (gnss_pt_read(GNSS_PT_REG_STATUS) & GNSS_PT_ST_MVDR_SEL_SYNCED);
}

/* ======================================================= one snapshot */
typedef struct {
	int      tx, pass, stap;
	uint64_t lo; uint32_t bw, fs; int band; const char *bname;
	int32_t  g1, g2; int agc_auto; const char *agc_why; uint32_t hunts;
	double   pk1, pk2; uint32_t clip1, clip2;
	double   nf, floor_int, target;           /* dBFS, NA = unknown */
	double   p1, p2;                          /* RX1 / RX2 power at the ref tap */
	gnss_smi_state_t ss; int smi_ok;
	gnss_rep_aj_t aj;
	gnss_null_t nl; int nl_ok;
	double   jn;                              /* THE J/N at the ADC (RX1 reference) */
	uint32_t att1, att_op;
	double   out; int out_ok; uint32_t dacclip, backoff;
} gm_t;

static double dbfs_pk(uint32_t p) { return p ? 20.0 * log10((double)p / 2048.0) : -99.0; }

static void gm_read(gm_t *m)
{
	gnss_pt_mvdr_state_t s;
	int32_t g0 = 0; uint32_t at = 0;

	memset(m, 0, sizeof(*m));
	m->tx = gnss_ant_tx_state();
	m->pass = gnss_ant_pass_state();
	m->stap = gnss_ant_stap_state();
	gnss_get_plan(&m->lo, &m->bw, &m->fs, &g0, &at);
	m->band = -1; m->bname = "";
	gnss_band_info(&m->band, &m->bname);
	if (ad9361_phy) {
		ad9361_get_rx_rf_gain(ad9361_phy, 0, &m->g1);
		ad9361_get_rx_rf_gain(ad9361_phy, 1, &m->g2);
		ad9361_get_tx_attenuation(ad9361_phy, 0, &m->att1);
	}
	m->att_op = gnss_get_op_atten_mdb();
	m->agc_auto = gnss_agc_is_auto();
	m->agc_why = gnss_agc_reason();
	m->hunts = gnss_agc_hunts();
	gnss_pt_mvdr_get_state(&s);
	m->pk1 = s.have_peak ? dbfs_pk(s.peak1) : NA;
	m->pk2 = s.have_peak ? dbfs_pk(s.peak2) : NA;
	m->clip1 = s.clip1; m->clip2 = s.clip2;
	m->out_ok = s.have_aolc && m->stap;
	m->out = 3.0103 * ((double)s.out_lvl_q4 / 16.0 - 23.0);
	m->dacclip = s.n_osat;
	m->backoff = gnss_pt_aolc_backoff_db();
	m->floor_int = gnss_rx_floor_dbfs();
	if (m->floor_int < -150.0) m->floor_int = NA;
	m->target = gnss_rx_target_dbfs();
	gnss_rep_get(&m->aj);
	m->smi_ok = gnss_smi_available();
	if (m->smi_ok)
		gnss_smi_get_state(&m->ss);
	m->p1 = (m->smi_ok && m->ss.updates) ? m->ss.pin_dbfs_ref : NA;
	m->p2 = (m->smi_ok && m->ss.updates) ? m->ss.p2_dbfs_ref : NA;
	m->nf = m->aj.valid ? m->aj.nf_dbfs : m->p1;
	m->nl_ok = (m->stap && gnss_smi_null_analysis(&m->nl) == 0 && m->nl.valid);
	/* one J/N: from the null analysis while stap is on (same R, same noise
	 * as the suppression), from the anti-jam report otherwise */
	m->jn = m->nl_ok ? m->nl.jn_in_db : (m->aj.valid ? m->aj.jnr_db : NA);
	if (!m->nl_ok && m->jn <= -50.0 && (m->pass || m->stap)) {
		/* report says "nothing above the noise": use the FPGA estimator */
		gnss_pt_jam_state_t j;
		gnss_pt_jam_get_state(&j);
		if (j.available && j.jnr_db_q8 != (int16_t)GNSS_PT_JAM_Q8_NONE)
			m->jn = (double)j.jnr_db_q8 / 256.0;
	}
}

static void print_mode(void);

/* ======================================================= print helpers */
static void pv(double v, int dec)               /* number or "--" */
{
	if (v <= NA + 1.0) { printf("--"); return; }
	if (dec == 0) printf("%.0f", v); else printf("%.1f", v);
}

static const char *mode_name(const gm_t *m)
{
	if (!m->tx)   return "TX1 SILENT";
	if (m->stap)  return "ANTI-JAM ON (SMI-STAP null on TX1)";
	if (m->pass)  return "PASS (raw RX1 on TX1, no nulling)";
	return "TX1 on, muted";
}

static const char *jam_type(const gm_t *m)
{
	switch (m->ss.jbw_class) {
	case 0:  return "CW/narrowband";
	case 1:  return "band-limited";
	default: return "wideband/barrage";
	}
}

static void line_jammer(const gm_t *m)
{
	if (!m->aj.valid) {
		printf("  JAMMER   -- (shown while pass/stap is on)\r\n");
		return;
	}
	if (!m->aj.jammed) {
		if (m->jn >= 1.0) {
			printf("  JAMMER   WEAK    J/N %.1f dB at the ADC (under the 6 dB alarm; %s)\r\n",
			       m->jn, m->stap ? "the null works on it" : "stap=1 nulls it");
			return;
		}
		printf("  JAMMER   CLEAR   (J/N "); pv(m->jn > -50 ? m->jn : NA, 1);
		printf(" dB, alarm at 6 dB)\r\n");
		return;
	}
	printf("  JAMMER   JAMMED  J/N %.1f dB at the ADC, %s ~%.0f MHz wide, RF %.2f MHz\r\n",
	       m->jn, jam_type(m), m->ss.jbw_hz / 1e6,
	       ((double)m->lo + m->ss.jf_hz) / 1e6);
	if (m->ss.jtype)                                 /* MOD7: the type over ~0.3 s */
		printf("           TYPE %s%s%s\r\n", m->ss.pulse_mode ? "PULSED " : "",
		       gnss_smi_jtype_name(m->ss.jtype), m->ss.sweep_mode ? " (swept: averaged over the sweep)" : "");
}

static void line_null(const gm_t *m)
{
	if (!m->stap) {
		printf("  NULL     off (stap=1 to null the jammer)\r\n");
		return;
	}
	if (!m->nl_ok) {
		printf("  NULL     ready, no jammer to null\r\n");
		return;
	}
	printf("  NULL     SUPPRESSION %s%.1f dB   J/N out %s%.1f dB%s\r\n",
	       m->nl.supp_lo ? ">" : "", m->nl.supp_db,
	       m->nl.supp_lo ? "<" : "", m->nl.jn_out_db,
	       m->nl.supp_lo ? "  (jammer under the noise)" :
	       (m->nl.jn_out_db > 0.0 ? "  (jammer above the noise: selftest?)" : ""));
}

static void item(const char *cmd, const char *what)
{
	printf("    %-18s %s\r\n", cmd, what);
}

/* ======================================================= main menu */
void gnss_ant_menu(void)
{
	gm_t m;

	gm_read(&m);
	printf("\r\n================ E310 GNSS ANTI-JAM  (LVDS 61.44 MSPS, SMI-STAP) ================\r\n");
	printf("  STATE    %s   [gnss_tx=%d pass=%d stap=%d]\r\n", mode_name(&m), m.tx, m.pass, m.stap);
	printf("  RADIO    LO %.2f MHz  BW %.0f MHz  %.2f MSPS  (band %d: %s)\r\n",
	       (double)m.lo / 1e6, m.bw / 1e6, m.fs / 1e6, m.band, m.bname);
	printf("  RX       gain %ld/%ld dB %s   noise ", (long)m.g1, (long)m.g2,
	       m.agc_auto ? "AUTO" : "MANUAL");
	pv(m.nf, 1); printf(" dBFS   ADC peak "); pv(m.pk1 > m.pk2 ? m.pk1 : m.pk2, 0);
	printf(" dBFS%s\r\n", (m.clip1 > 8 || m.clip2 > 8) ? "  CLIPPING" : "");
	line_jammer(&m);
	line_null(&m);
	printf("--------------------------------------------------------------------------------\r\n");
	printf("  CONTROL  gnss_tx=1|0   pass=1|0   stap=1|0\r\n");
	printf("           (pass=1 sets gnss_tx=1; stap=1 sets gnss_tx=1 and pass=1)\r\n");
	printf("  CHECK    gnss_tx?   gnss_status?   gnss_jam?   gnss_mvdr?   gnss_spec?\r\n");
	printf("           status?   rx?   stap?   selftest?\r\n");
	printf("  SETTINGS band=N (band? lists)   tx1_att=mdB   agc=1|0   rxgain=dB\r\n");
	printf("           gnss_agcstep=dB,s   gnss_agcclip=ppm   gnss_rxoff=dB (rxoff?)   gnss_ajbw=1|0\r\n");
	printf("           taps=N (1..16)   smi=K,A   smiavg=N   gnss_clean=1|0\r\n");
	printf("           gnss_track=1|0   gnss_pulse=1|0\r\n");
	printf("  Enter = this menu   help? = these commands   commands? = ALL commands\r\n");
	printf("  (AD9361 vendor commands rx_lo_freq=, tx1_attenuation=, rx1_rf_gain=, dds_* ...\r\n");
	printf("   and every MVDR2 gnss_ command are all still there: commands? lists them)\r\n");
	printf("================================================================================\r\n");
}

/* ======================================================= pages */
void get_ant_help(double *param, char param_no)
{
	(void)param; (void)param_no;
	printf("\r\n--- COMMANDS ---------------------------------------------------------------\r\n");
	printf("  CONTROL\r\n");
	item("gnss_tx=1 / =0", "TX1 chain on (muted, no signal) / everything off, TX1 silent");
	item("pass=1 / =0", "raw RX1 on TX1, no nulling (sets gnss_tx=1, stap=0) / mute");
	item("stap=1 / =0", "ANTI-JAM: RX gain level, output level, SMI-STAP null on TX1");
	item("", "(sets gnss_tx=1 pass=1, ~2 s) / back to raw RX1");
	printf("  CHECK\r\n");
	item("gnss_tx?", "is TX1 transmitting, what (raw RX1 / MVDR), attenuation (hardware)");
	item("gnss_status?", "radio set-up read back: LO, BW, rate, gain mode, gains, attenuation");
	item("gnss_jam?", "jammer: RX1/RX2 power, phase, angle, J/N in/out, SUPPRESSION, cost");
	item("gnss_mvdr?", "MVDR core state (MVDR2 page)");
	item("gnss_spec? / =G", "noise spectrum (internal noise); =G at RX gain G (MVDR2 page)");
	item("status?", "everything on one page: radio, RX, noise, jammer, null, TX");
	item("rx?", "RX gain control: gains, AGC state and reason, ADC peaks, noise");
	item("stap?", "anti-jam: suppression, J/N, theory, solver health");
	item("selftest?", "checks what limits suppression and noise; says what to fix");
	printf("  SETTINGS\r\n");
	item("band? / band=N", "radio presets (LO, BW, rate); 3 = all four systems (default)");
	item("tx1_att=mdB", "TX1 operating attenuation (bigger = quieter), e.g. 34000");
	item("agc=1 / =0", "RX gain AUTO (default, fast + anti-hunt) / MANUAL");
	item("rxgain=dB", "manual RX gain, same on RX1 and RX2 (sets agc=0)");
	item("gnss_agcstep=dB,s", "AGC rise with a jammer (def 1 dB every 1 s; e.g. 3,2)");
	item("taps=N / taps?", "STAP taps per antenna in use, 1..16, live (16 = all)");
	item("smiavg=N[,S]", "covariance averaging 1/2^N, steady 1/2^S after 1 s (def 2,4)");
	item("gnss_ajbw=1|0|MHz", "wideband/chirp jammer: narrow the RX filter (1 AUTO default, 0 off, or MHz)");
	item("gnss_agcclip=ppm", "barrage AGC: allowed ADC clip rate (def 100 ppm; 0 = peak rule)");
	item("gnss_rxoff=dB / rxoff?", "RX2 gain = RX1 gain - dB (one channel hotter); rxoff? measures it");
	printf("  ADVANCED\r\n");
	item("smi=K,A", "SMI window 2^K snapshots (6..12, def 8), loading A x lambda_min (def 3)");
	item("smiavg=N", "covariance averaging 0..4 (def 2 = beta 1/4, steadier satellites)");
	item("gnss_clean=1|0", "no jammer: fixed RX1 path on TX1 (faster lock); 0 = always solved weights");
	item("gnss_track=1|0", "moving jammer / platform: fast averaging when the null leaks (default 1)");
	item("gnss_pulse=1|0", "pulsed jammer: keep the null between pulses (default 1)");
	item("gnss_kpi?", "all figures on one key=value line (for the e310lab logger)");
	item("commands?", "ALL commands: AD9361 vendor + every MVDR2 gnss_ command");
	printf("--------------------------------------------------------------------------------\r\n");
	printf("  SUPPRESSION S = (J/N)in / (J/N)out. (J/N)in at RX1 (reference element),\r\n");
	printf("  (J/N)out = w^H Rj w / (sn2 |w|^2) at the output; scaling w changes nothing.\r\n");
	printf("  '>' = the jammer left is under what the window can measure (3/sqrt(K)).\r\n");
}

void get_ant_status(double *param, char param_no)
{
	gm_t m;
	(void)param; (void)param_no;

	gm_read(&m);
	printf("\r\n--- STATUS -----------------------------------------------------------------\r\n");
	printf("  STATE    %s   [gnss_tx=%d pass=%d stap=%d]\r\n", mode_name(&m), m.tx, m.pass, m.stap);
	printf("  RADIO    LO %.2f MHz  BW %.0f MHz  %.2f MSPS  band %d: %s\r\n",
	       (double)m.lo / 1e6, m.bw / 1e6, m.fs / 1e6, m.band, m.bname);
	printf("  RX       gain RX1 %ld dB  RX2 %ld dB  %s\r\n", (long)m.g1, (long)m.g2,
	       m.agc_auto ? "AUTO" : "MANUAL");
	printf("           ADC peak RX1 "); pv(m.pk1, 1); printf(" RX2 "); pv(m.pk2, 1);
	printf(" dBFS   clipped samples/window %lu / %lu\r\n", (unsigned long)m.clip1, (unsigned long)m.clip2);
	printf("  NOISE    floor "); pv(m.nf, 1); printf(" dBFS   AD9361 internal "); pv(m.floor_int, 1);
	printf(" dBFS   target %.1f dBFS\r\n", m.target);
	line_jammer(&m);
	line_null(&m);
	printf("  TX1      attenuation %lu mdB%s (operating %lu)", (unsigned long)m.att1,
	       m.att1 >= GNSS_L1_TX_MAX_ATTEN_MDB ? " = silent" : "", (unsigned long)m.att_op);
	if (m.out_ok) {
		printf("   output %.1f dBFS (target %.0f)", m.out, AOLC_TARGET_DBFS);
	}
	printf("   DAC clips %lu\r\n", (unsigned long)m.dacclip);
	{
		uint32_t n = gnss_rep_event_total(), i, from = (n > 5) ? n - 5 : 0;
		double t; const char *txt;
		printf("  EVENTS   (last %lu)\r\n", (unsigned long)(n - from));
		for (i = from; i < n; i++)
			if (gnss_rep_event_get(i, &t, &txt) == 0)
				printf("    %7.1f s  %s\r\n", t, txt);
	}
}

void get_rx_info(double *param, char param_no)
{
	gm_t m;
	(void)param; (void)param_no;

	gm_read(&m);
	printf("\r\n--- RX / GAIN CONTROL -------------------------------------------------------\r\n");
	printf("  gain      RX1 %ld dB   RX2 %ld dB   mode %s\r\n", (long)m.g1, (long)m.g2,
	       m.agc_auto ? "AUTO" : "MANUAL (agc=1 for AUTO)");
	printf("  AGC       %s\r\n", m.agc_why ? m.agc_why : "--");
	printf("            hunts since boot %lu (each locks the step-ups 10..80 s)\r\n",
	       (unsigned long)m.hunts);
	printf("  ADC       peak RX1 "); pv(m.pk1, 1); printf(" / RX2 "); pv(m.pk2, 1);
	printf(" dBFS   clipped %lu / %lu in the last window\r\n",
	       (unsigned long)m.clip1, (unsigned long)m.clip2);
	{
		double r = 0.0;
		int bar = gnss_agc_barrage_rule(&r);
		printf("  rule      %s, ADC clip rate %.1f ppm (1 s)\r\n",
		       bar ? "BARRAGE: gain on the clip rate" :
		       "peak limit (-4 dBFS CW / no jammer)", r);
		if (gnss_agc_get_clip_ppm() > 0)
			printf("            barrage limit %d ppm (gnss_agcclip=ppm, 0 = peak rule)\r\n",
			       gnss_agc_get_clip_ppm());
		else
			printf("            barrage clip-rate rule OFF (gnss_agcclip=100 turns it on)\r\n");
	}
	printf("  RX2 off   RX2 gain = RX1 gain - %d dB (gnss_rxoff=dB, rxoff? measures it)\r\n",
	       gnss_rx2_off_get());
	printf("  power     RX1 "); pv(m.p1, 1); printf(" / RX2 "); pv(m.p2, 1);
	printf(" dBFS (reference tap, signal + noise)\r\n");
	printf("  noise     floor "); pv(m.nf, 1); printf(" dBFS   target %.1f dBFS   AD9361 internal ", m.target);
	pv(m.floor_int, 1); printf(" dBFS\r\n");
	if (m.aj.valid && m.aj.jammed && m.aj.have_clean && m.aj.clean_gain_db > m.g1)
		printf("            gain cut %ld dB by the jammer: floor %.1f dB over the antenna noise\r\n"
		       "            at this gain = the satellites' C/N0 cost from the gain cut\r\n",
		       (long)(m.aj.clean_gain_db - m.g1), m.aj.rise_db);
	else if (m.nf > NA + 1 && m.floor_int > NA + 1)
		printf("            antenna noise %.1f dB above the internal noise %s\r\n",
		       m.nf - m.floor_int,
		       (m.nf - m.floor_int >= 10.0) ? "(good: internal adds < 0.5 dB)" :
		       (m.nf - m.floor_int >= 5.0) ? "(internal adds ~1 dB)" :
		       "(internal noise COSTS C/N0: more gain or a better LNA)");
	{
		int sdb = 1, ss = 1;
		gnss_agc_get_jstep(&sdb, &ss);
		printf("  how it works: a clip cuts the gain at once (6/10/15 dB by how much clips),\r\n");
		printf("  then 30 ms with no further cut. CW: rises %d dB every %d s, never closer than\r\n", sdb, ss);
		printf("  1 dB to -4 dBFS. BARRAGE: rises %d dB every %d s while the ADC clips < 1/4 of\r\n", sdb, ss);
		printf("  the limit, -1 dB over 5x the limit (clipped tails cost nothing). Without a jammer it\r\n");
		printf("  returns to the noise target at up to 10 dB per 0.3 s.\r\n");
	}
}

void get_stap(double *param, char param_no)
{
	gm_t m;
	(void)param; (void)param_no;

	gm_read(&m);
	printf("\r\n--- ANTI-JAM (SMI-STAP) ----------------------------------------------------\r\n");
	printf("  state     %s\r\n", m.stap ? "ON (null on TX1)" : "OFF (stap=1 to start)");
	line_jammer(&m);
	line_null(&m);
	if (m.stap && m.nl_ok) {
		printf("            J/N in  %6.1f dB   jammer/noise at RX1 (reference element)\r\n", m.nl.jn_in_db);
		printf("            J/N out %s%5.1f dB   at the array output (noise = sn2 x |w|^2)\r\n",
		       m.nl.supp_lo ? "<" : " ", m.nl.jn_out_db);
		printf("            S = J/N in - J/N out = %s%.1f dB\r\n", m.nl.supp_lo ? ">" : "", m.nl.supp_db);
		printf("  theory    1 weight/antenna, 1 narrowband jammer: S = (1+r)^2 + r^2 ~ 2 JNR + 3 dB\r\n");
		printf("            with the loading (r = JNR/(1+%.0f)):  %.1f dB here\r\n", m.ss.alpha, m.nl.sth2_db);
		printf("            CW + %d taps: up to M(M-1) r^2 (M = %d). A barrage fills the taps:\r\n",
		       m.ss.La, 2 * m.ss.La);
		printf("            ~ JNR + 10..17 dB, 1 dB per dB of JNR (noise-limited).\r\n");
		printf("  cost      C/N0 lost to the jammer left %.2f dB   SMI estimation loss %.2f dB (K %.0f, N %d)\r\n",
		       m.nl.cn0_loss_db, m.nl.smi_loss_db, m.ss.keff, m.ss.Na);
		printf("  noise     %.1f dBFS per element   jammer RX2/RX1 %.1f dB, %.0f deg   null in %.0f ms\r\n",
		       m.nl.sn2_dbfs, m.nl.amp_db, m.nl.ph_deg, m.aj.t_null_ms);
	}
	if (m.smi_ok) {
		int w1 = 0, w2 = 0;
		printf("  solver    2 antennas x %d taps in use (of %d) + 2 image taps, window 2^%lu, loading %.1f x lambda_min\r\n",
		       m.ss.La, m.ss.L, (unsigned long)m.ss.kexp, m.ss.alpha);
		if (gnss_smi_weight_taps(&w1, &w2) == 0)
			printf("            weights in the FPGA now: %d taps on RX1, %d on RX2 (the rest 0)\r\n", w1, w2);
		else
			printf("            weights in the FPGA now: -- (no solve yet: stap=1)\r\n");
		{
			int cen, chold;
			double cin, cout;
			uint32_t cn;
			gnss_smi_get_clean(&cen, &cin, &cout, &chold, &cn);
			printf("  jammer    %s%s%s", m.ss.pulse_mode ? "PULSED " : "", gnss_smi_jtype_name(m.ss.jtype),
			       m.ss.pulse_mode ? " (null kept between pulses)" : "");
			if (m.ss.jtype == GNSS_JT_MULTI) printf(", ~%d tones", m.ss.ntones);
			if (m.ss.sweep_mode) {
				printf(", swept %.1f..%.1f MHz from the LO", m.ss.sweep_lo_hz / 1e6, m.ss.sweep_hi_hz / 1e6);
				if (m.ss.sweep_period_ms > 0.0) printf(", period %.1f ms", m.ss.sweep_period_ms);
				printf("\r\n            sweep averaging beta 1/%d over the whole sweep (no restarts)",
				       1 << m.ss.sweep_sh);
			}
			printf("%s\r\n", m.ss.jtype == GNSS_JT_WIDE ? "\r\n            AGC barrage rule (noise)" :
			       (m.ss.jtype ? "\r\n            AGC peak rule -4 dBFS (steady envelope)" : ""));
			if (m.ss.leak_db > -90.0) {
				printf("  tracking  leak %.1f dB (usual %.1f), fast %lu, restarts %lu\r\n",
				       m.ss.leak_db, m.ss.leak_ref_db, (unsigned long)m.ss.leak_fast,
				       (unsigned long)m.ss.leak_reset);
			}
			printf("  clean     %s\r\n", !cen ? "OFF (solved weights always on TX1)" :
			       chold ? "HOLDING: no jammer, TX1 = fixed RX1 path (fast satellite lock)" :
			       "on: solved weights on TX1 (jammer, or not yet 2 s clear)");
		}
		printf("            averaging beta 1/%d, steady 1/%d (%s), %.0f snapshots in R\r\n",
		       m.ss.avg_sh ? (1 << m.ss.avg_sh) : 1,
		       m.ss.avg_sh_st ? (1 << m.ss.avg_sh_st) : (m.ss.avg_sh ? (1 << m.ss.avg_sh) : 1),
		       m.ss.avg_steady ? "STEADY now: low jitter" : "fast: after a power step",
		       m.ss.keff);
		printf("  timing    read %.0f + average %.0f + solve %.0f + write %.0f + figures %.0f = %.0f us\r\n",
		       m.ss.t_read_us, m.ss.t_avg_us, m.ss.t_solve_us, m.ss.t_write_us, m.ss.t_post_us,
		       m.ss.t_total_us);
		printf("            weights updated every %.2f ms (%.0f per s), longest pass %.0f us\r\n",
		       m.ss.t_int_us / 1000.0, m.ss.t_int_us > 0.0 ? 1e6 / m.ss.t_int_us : 0.0, m.ss.t_max_us);
		printf("            new weights load at a window boundary (double-buffered: no glitch)\r\n");
		printf("            solved %lu  failed %lu  overruns %lu\r\n",
		       (unsigned long)m.ss.updates, (unsigned long)m.ss.fails, (unsigned long)m.ss.novr);
		printf("            windows passed over %lu (newer one ready: normal), after gain steps %lu\r\n",
		       (unsigned long)m.ss.skipped, (unsigned long)gnss_smi_gain_skipped());
	} else {
		printf("  solver    SMI core not found in the bitstream\r\n");
	}
	printf("  S grows with the J/N at the ADC until clipping, band-dependent RX1/RX2\r\n");
	printf("  mismatch or uncorrelated jammer energy stops it: selftest? tells which.\r\n");
}

/* ======================================================= gnss_ pages */
static const char *gc_name(uint8_t m)
{
	switch (m) {
	case 0:  return "MANUAL";
	case 1:  return "FAST AGC";
	case 2:  return "SLOW AGC";
	case 3:  return "HYBRID AGC";
	default: return "?";
	}
}

/* gnss_tx? : is TX1 really transmitting (read from the hardware) */
void get_ant_gnss_tx(double *param, char param_no)
{
	int q = gnss_quiet_get();
	gnss_quiet_set(0);
	gnss_tx_hw_print(param, param_no);
	gnss_quiet_set(q);
	print_mode();
}

/* gnss_status? : the radio set-up read back from the AD9361, then the state */
void get_ant_gnss_status(double *param, char param_no)
{
	gm_t m;
	uint64_t rlo = 0, tlo = 0;
	uint32_t rbw = 0, tbw = 0, rfs = 0, tfs = 0, a1 = 0, a2 = 0;
	uint32_t rxc[6] = {0}, txc[6] = {0};
	uint8_t  gm1 = 0, gm2 = 0, rfir = 0, tfir = 0;
	int32_t  g1 = 0, g2 = 0;
	(void)param; (void)param_no;

	gm_read(&m);
	printf("\r\n--- GNSS STATUS: radio set-up (read back from the AD9361) -------------------\r\n");
	if (!ad9361_phy) { printf("  ad9361_phy is NULL\r\n"); return; }
	ad9361_get_rx_lo_freq(ad9361_phy, &rlo);
	ad9361_get_tx_lo_freq(ad9361_phy, &tlo);
	ad9361_get_rx_rf_bandwidth(ad9361_phy, &rbw);
	ad9361_get_tx_rf_bandwidth(ad9361_phy, &tbw);
	ad9361_get_rx_sampling_freq(ad9361_phy, &rfs);
	ad9361_get_tx_sampling_freq(ad9361_phy, &tfs);
	ad9361_get_rx_gain_control_mode(ad9361_phy, 0, &gm1);
	ad9361_get_rx_gain_control_mode(ad9361_phy, 1, &gm2);
	ad9361_get_rx_rf_gain(ad9361_phy, 0, &g1);
	ad9361_get_rx_rf_gain(ad9361_phy, 1, &g2);
	ad9361_get_tx_attenuation(ad9361_phy, 0, &a1);
	ad9361_get_tx_attenuation(ad9361_phy, 1, &a2);
	ad9361_get_rx_fir_en_dis(ad9361_phy, &rfir);
	ad9361_get_tx_fir_en_dis(ad9361_phy, &tfir);
	printf("              %-22s %-22s\r\n", "RX (RX1 + RX2)", "TX (TX1)");
	printf("  LO          %-14.4f MHz     %-14.4f MHz\r\n", (double)rlo / 1e6, (double)tlo / 1e6);
	printf("  RF BW       %-14.3f MHz     %-14.3f MHz\r\n", rbw / 1e6, tbw / 1e6);
	printf("  sample rate %-14.4f MSPS    %-14.4f MSPS\r\n", rfs / 1e6, tfs / 1e6);
	printf("  FIR         %-22s %-22s\r\n", rfir ? "ON" : "bypassed", tfir ? "ON" : "bypassed");
	if (ad9361_get_trx_path_clks(ad9361_phy, rxc, txc) == 0 && rxc[5] && txc[5])
		printf("  ADC / DAC   %.2f MHz (%lu x fs)     %.2f MHz (%lu x fs)\r\n",
		       rxc[1] / 1e6, (unsigned long)(rxc[1] / rxc[5]),
		       txc[1] / 1e6, (unsigned long)(txc[1] / txc[5]));
	printf("  gain mode   RX1 %s (%u), RX2 %s (%u)   anti-jam AGC: %s\r\n",
	       gc_name(gm1), gm1, gc_name(gm2), gm2, m.agc_auto ? "AUTO" : "MANUAL");
	printf("  RX gain     RX1 %ld dB, RX2 %ld dB%s\r\n", (long)g1, (long)g2,
	       (g1 - g2 != gnss_rx2_off_get()) ? "   (NOT the set offset: agc=1 or rxgain= fixes it)" :
	       (g1 != g2) ? "   (gnss_rxoff offset)" : "");
	printf("  TX atten    TX1 %lu mdB%s, TX2 %lu mdB   (operating TX1 %lu mdB)\r\n",
	       (unsigned long)a1, a1 >= GNSS_L1_TX_MAX_ATTEN_MDB ? " = silent" : "",
	       (unsigned long)a2, (unsigned long)m.att_op);
	printf("  preset      band %d: %s\r\n", m.band, m.bname);
	{
		static int32_t t_boot = -1000000;
		int32_t tc = 0;
		if (ad9361_get_temperature(ad9361_phy, &tc) == 0) {
			if (t_boot == -1000000) t_boot = tc;
			printf("  AD9361 temp %.0f C (uncalibrated, +-5 C), %+.0f C since first read%s\r\n",
			       tc / 1000.0, (tc - t_boot) / 1000.0,
			       (tc - t_boot > 20000 || t_boot - tc > 20000) ?
			       "  -> drift: the weights follow it; gnss_auto=1 re-runs the radio calibrations" : "");
		}
		printf("  tracking    RX quadrature (IQ), BB DC and RF DC tracking ON (AD9361, continuous)\r\n");
	}
	printf("--- state ------------------------------------------------------------------\r\n");
	printf("  STATE    %s   [gnss_tx=%d pass=%d stap=%d]\r\n", mode_name(&m), m.tx, m.pass, m.stap);
	printf("  RX       ADC peak RX1 "); pv(m.pk1, 1); printf(" / RX2 "); pv(m.pk2, 1);
	printf(" dBFS, clipped %lu / %lu per window\r\n", (unsigned long)m.clip1, (unsigned long)m.clip2);
	printf("  NOISE    floor "); pv(m.nf, 1); printf(" dBFS   AD9361 internal "); pv(m.floor_int, 1);
	printf(" dBFS   target %.1f dBFS\r\n", m.target);
	line_jammer(&m);
	line_null(&m);
	if (m.out_ok)
		printf("  OUTPUT   %.1f dBFS (target %.0f), DAC clips %lu\r\n", m.out, AOLC_TARGET_DBFS,
		       (unsigned long)m.dacclip);
}

/* gnss_jam? : everything about the jammer and how well it is nulled */
static void dbm_or_na(double dbfs, int32_t g)
{
	long k = gnss_pt_jam_get_dbm_k();
	if (dbfs <= NA + 1.0) { printf("--"); return; }
	printf("%.1f dBFS", dbfs);
	if (k != GNSS_PT_JAM_K_UNSET)
		printf(" = %.1f dBm", dbfs - (double)g + (double)k / 100.0);
}

static double q8(int16_t v) { return (v == (int16_t)GNSS_PT_JAM_Q8_NONE) ? NA : (double)v / 256.0; }

void get_ant_jam(double *param, char param_no)
{
	gm_t m;
	gnss_pt_jam_state_t j;
	(void)param; (void)param_no;

	gm_read(&m);
	gnss_pt_jam_get_state(&j);
	printf("\r\n--- JAMMER -----------------------------------------------------------------\r\n");
	line_jammer(&m);
	if (m.aj.valid && m.aj.jammed)
		printf("           type %s, ~%.1f MHz wide, centre RF %.3f MHz\r\n", jam_type(&m),
		       m.ss.jbw_hz / 1e6, ((double)m.lo + m.ss.jf_hz) / 1e6);
	printf("  POWER    (RX gain %ld / %ld dB%s)\r\n", (long)m.g1, (long)m.g2,
	       gnss_pt_jam_get_dbm_k() == GNSS_PT_JAM_K_UNSET ? ", dBm after gnss_jpcal=" : "");
	printf("           total   RX1 "); dbm_or_na(m.p1, m.g1); printf("   RX2 "); dbm_or_na(m.p2, m.g2);
	printf("\r\n");
	if (j.available && j.have_ch) {
		printf("           jammer  RX1 "); dbm_or_na(q8(j.j1_q8), m.g1);
		printf("   RX2 "); dbm_or_na(q8(j.j2_q8), m.g2); printf("\r\n");
		printf("           noise   RX1 "); dbm_or_na(q8(j.n1_q8), m.g1);
		printf("   RX2 "); dbm_or_na(q8(j.n2_q8), m.g2); printf("\r\n");
	}
	printf("           ADC peak RX1 "); pv(m.pk1, 1); printf(" / RX2 "); pv(m.pk2, 1);
	printf(" dBFS%s\r\n", (m.clip1 > 8 || m.clip2 > 8) ? "  CLIPPING: cannot be nulled" : "");
	if (j.available && j.present) {
		printf("  DIRECTION RX2 - RX1 phase %.1f deg (raw %.1f), angle %.1f deg from broadside\r\n",
		       j.psi_cdeg / 100.0, j.psi_raw_cdeg / 100.0, j.theta_cdeg / 100.0);
		printf("           (+ = towards RX2)%s%s\r\n",
		       j.cfg_cal_bin == 0 ? "  NOT calibrated: gnss_jcal=1 with a source at broadside" : "",
		       j.sin_clip ? "  |sin| > 1 clipped" : "");
	} else if (j.available) {
		printf("  DIRECTION -- (no jammer)\r\n");
	}
	if (m.stap && m.nl_ok) {
		printf("  RX2/RX1  jammer amplitude %+.1f dB, phase %.1f deg (SMI covariance)\r\n",
		       m.nl.amp_db, m.nl.ph_deg);
		printf("  NULL     J/N in  %6.1f dB   at RX1 (reference element)\r\n", m.nl.jn_in_db);
		printf("           J/N out %s%5.1f dB   at the array output (noise = sn2 x |w|^2)\r\n",
		       m.nl.supp_lo ? "<" : " ", m.nl.jn_out_db);
		printf("           SUPPRESSION S = J/N in - J/N out = %s%.1f dB\r\n",
		       m.nl.supp_lo ? ">" : "", m.nl.supp_db);
		printf("  theory   2 antennas, 1 weight each: S = (1+r)^2 + r^2 ~ 2 JNR + 3 dB;\r\n");
		printf("           with loading %.0f x lambda_min: %.1f dB. CW + %d taps: up to M(M-1)r^2,\r\n",
		       m.ss.alpha, m.nl.sth2_db, m.ss.La);
		printf("           M = %d. Barrage: ~ JNR + 10..17 dB (1 dB per dB of JNR) with 8-16 taps.\r\n",
		       2 * m.ss.La);
		printf("  COST     C/N0 lost %.2f dB   SMI estimation loss %.2f dB (K %.0f)   null in %.0f ms\r\n",
		       m.nl.cn0_loss_db, m.nl.smi_loss_db, m.ss.keff, m.aj.t_null_ms);
		printf("           noise %.1f dBFS per element\r\n", m.nl.sn2_dbfs);
	} else if (m.aj.valid && m.aj.jammed) {
		printf("  NULL     off: stap=1 to null it (J/N out and SUPPRESSION need stap on)\r\n");
	}
	printf("  more     gnss_jam_est? = FPGA estimator detail, gnss_jmon=N, gnss_jcal=1, gnss_jpcal=dBm\r\n");
}

/* gnss_agcstep=dB,s : AGC step-up with a jammer */
void set_agcstep(double *param, char param_no)
{
	int db = 1, sec = 1;
	if (param_no >= 1) db = (int)param[0];
	if (param_no >= 2) sec = (int)param[1];
	if (param_no < 1 || gnss_agc_set_jstep(db, sec) != 0) {
		gnss_agc_get_jstep(&db, &sec);
		printf("  gnss_agcstep=dB,s  dB 1..15, s 1..10 (now %d dB every %d s)\r\n", db, sec);
		return;
	}
	printf("  AGC with a jammer: up %d dB every %d s (never within 1 dB of the ADC limit)\r\n", db, sec);
}

/* ======================================================= self-test */
static int n_ok, n_warn, n_fail;
static char actions[6][110];
static int n_act;

static void verdict(int lvl, const char *what, const char *fix)
{
	static const char *tag[3] = { "  OK  ", " WARN ", " FAIL " };
	printf("  [%s] %s\r\n", tag[lvl], what);
	if (lvl == 0) n_ok++; else if (lvl == 1) n_warn++; else n_fail++;
	if (lvl && fix && n_act < 6)
		snprintf(actions[n_act++], sizeof(actions[0]), "%s", fix);
}

void get_selftest(double *param, char param_no)
{
	gm_t m;
	gnss_pt_state_t a, b;
	uint32_t rxc[6] = {0}, txc[6] = {0};
	char t[160];
	double rate;
	(void)param; (void)param_no;

	n_ok = n_warn = n_fail = n_act = 0;
	printf("\r\n--- SELF-TEST: what limits suppression, J/N and noise ----------------------\r\n");
	gnss_pt_get_state(&a);
	no_os_mdelay(100);
	gnss_pt_get_state(&b);
	gm_read(&m);

	printf(" 1 FPGA data path\r\n");
	snprintf(t, sizeof(t), "anti-jam block id 0x%08lx v%lu.%lu, SMI core %s",
		 (unsigned long)b.id, (unsigned long)(b.version >> 16), (unsigned long)(b.version & 0xFFFF),
		 m.smi_ok ? "present" : "MISSING");
	verdict((b.id == 0x47435031UL && m.smi_ok) ? 0 : 2, t, "wrong bitstream: load Passthrough_MVDR");
	rate = (double)(b.rx_count_ch0 - a.rx_count_ch0) / 0.1 / 1e6;
	snprintf(t, sizeof(t), "samples moving: %.2f MSPS measured (plan %.2f)", rate, m.fs / 1e6);
	verdict((fabs(rate - m.fs / 1e6) < 0.05 * m.fs / 1e6) ? 0 : 2, t,
		"RX samples not at the plan rate: check the LVDS interface / band=3");
	snprintf(t, sizeof(t), "FIFO overflow %lu underflow %lu",
		 (unsigned long)b.overflow_count, (unsigned long)b.underflow_count);
	verdict((b.overflow_count == a.overflow_count && b.underflow_count == a.underflow_count) ? 0 : 1, t,
		"FIFO errors: RX and TX rates differ (use band=, not the vendor rate commands)");

	printf(" 2 AD9361 clock (ADC noise)\r\n");
	if (ad9361_phy && ad9361_get_trx_path_clks(ad9361_phy, rxc, txc) == 0 && rxc[5]) {
		uint32_t r = rxc[1] / rxc[5];
		snprintf(t, sizeof(t), "ADC clock %.2f MHz = %lu x the sample rate", rxc[1] / 1e6, (unsigned long)r);
		verdict(r >= 8 ? 0 : 1, t, "ADC below 8 x fs: ~14 dB more ADC noise at the band edges (re-apply band=)");
	}

	printf(" 3 Internal noise vs antenna noise (sets the satellite C/N0)\r\n");
	if (m.aj.valid && m.aj.jammed && m.aj.have_clean && m.aj.clean_gain_db > m.g1) {
		/* with a jammer the floor is not antenna noise: compare it with the
		 * antenna noise expected at this gain (clean noise - gain cut) */
		snprintf(t, sizeof(t), "gain cut %ld dB by the jammer: floor %.1f dB over the antenna noise = C/N0 cost",
			 (long)(m.aj.clean_gain_db - m.g1), m.aj.rise_db);
		verdict(m.aj.rise_db <= 3.0 ? 0 : (m.aj.rise_db <= 8.0 ? 1 : 2), t,
			"dynamic range: the jammer is too strong for the ADC at full gain; less jammer per channel "
			"(gnss_rxoff, gnss_ajbw, band with a narrower RF filter) or a front end with more range");
	} else if (m.nf > NA + 1 && m.floor_int > NA + 1) {
		double d = m.nf - m.floor_int;
		snprintf(t, sizeof(t), "antenna noise %.1f dB above the AD9361 internal noise (gain %ld dB)",
			 d, (long)m.g1);
		verdict(d >= 10.0 ? 0 : (d >= 5.0 ? 1 : 2), t,
			"internal noise too close: agc=1 (gain to the target), check the antenna LNA power/gain");
	} else {
		verdict(1, "noise floor not known yet", "run pass=1 or stap=1 first, then selftest?");
	}
	if (!m.agc_auto)
		verdict(1, "RX gain is MANUAL", "agc=1: AUTO keeps the best C/N0 and avoids clipping");

	printf(" 4 RX1 / RX2 match (limits the null depth)\r\n");
	if (m.p1 > NA + 1 && m.p2 > NA + 1 && !(m.aj.valid && m.aj.jammed) && !(m.jn > NA + 1 && m.jn > 0.0)) {
		double d  = m.p2 - m.p1;                         /* at the ADC, offset applied */
		double de = d + (double)(m.g1 - m.g2);           /* at equal gain: the hardware */
		snprintf(t, sizeof(t), "RX2 - RX1 noise %+.1f dB at the ADC (%+.1f dB at equal gain, no jammer)", d, de);
		verdict(fabs(d) <= 2.0 ? 0 : (fabs(d) <= 4.0 ? 1 : 2), t,
			"RX1/RX2 levels differ: fix the antenna/LNA/cable, or gnss_rxoff= (rxoff? suggests it)");
	}
	if (m.g1 - m.g2 != gnss_rx2_off_get())
		verdict(2, "RX1 and RX2 gains are not the set offset", "agc=1 (or rxgain=) sets them again");

	printf(" 5 ADC headroom (a clipped jammer cannot be nulled)\r\n");
	snprintf(t, sizeof(t), "ADC peak %.1f dBFS, clipped %lu/%lu per window, AGC hunts %lu",
		 (m.pk1 > m.pk2 ? m.pk1 : m.pk2), (unsigned long)m.clip1, (unsigned long)m.clip2,
		 (unsigned long)m.hunts);
	verdict((m.clip1 > 8 || m.clip2 > 8) ? 2 : (m.hunts > 2 ? 1 : 0), t,
		"clipping/hunting: attenuate the jammer path or move it; the AGC is at its limit");

	printf(" 6 Anti-jam\r\n");
	if (!m.stap) {
		verdict(1, "stap is off: the null was not checked", "stap=1 with the jammer on, then selftest?");
	} else if (!m.nl_ok) {
		verdict(0, "stap on, no jammer: switch a jammer on to check the null", NULL);
	} else {
		snprintf(t, sizeof(t), "J/N in %.1f dB, J/N out %s%.1f dB, SUPPRESSION %s%.1f dB, C/N0 cost %.2f dB",
			 m.nl.jn_in_db, m.nl.supp_lo ? "<" : "", m.nl.jn_out_db,
			 m.nl.supp_lo ? ">" : "", m.nl.supp_db, m.nl.cn0_loss_db);
		if (m.nl.supp_lo || m.nl.jn_out_db <= -10.0) {
			verdict(0, t, NULL);
			printf("          jammer under the output noise: more J/N in (RX gain) = more S\r\n");
		} else if (m.nl.jn_in_db < 15.0 && m.clip1 <= 8 && m.clip2 <= 8) {
			char a[200];
			snprintf(a, sizeof(a), "weak jammer: with loading %.0f x lambda_min the null takes only"
				 " ~JNR + 1..6 dB here (theory %.1f dB). smi=%d,1 lowers the C/N0 cost",
				 m.ss.alpha, m.nl.sth2_db, m.ss.kexp ? (int)m.ss.kexp : 8);
			verdict(1, t, a);
		} else {
			verdict(m.nl.jn_out_db > 0.0 ? 2 : 1, t,
				"jammer left near/above the noise: clipping, RX1/RX2 response differs across the"
				" band, uncorrelated jammer energy (LNA overload), or 2+ jammer directions");
		}
		if (m.nl.smi_loss_db > 1.0) {
			snprintf(t, sizeof(t), "SMI estimation loss %.1f dB (K %.0f snapshots for N %d)",
				 m.nl.smi_loss_db, m.ss.keff, m.ss.Na);
			verdict(1, t, "few snapshots: smiavg=2 or a larger window (smi=9,3)");
		}
	}
	if (m.smi_ok && m.ss.updates) {
		snprintf(t, sizeof(t), "solver: %lu solved, %lu failed, %lu skipped",
			 (unsigned long)m.ss.updates, (unsigned long)m.ss.fails, (unsigned long)m.ss.skipped);
		verdict(m.ss.fails * 100 > m.ss.updates + 100 ? 1 : 0, t, "many failed solves: check clipping");
	}

	printf(" 7 TX1 output\r\n");
	if (m.tx) {
		snprintf(t, sizeof(t), "TX1 attenuation %lu mdB, DAC clips %lu%s", (unsigned long)m.att1,
			 (unsigned long)m.dacclip, m.out_ok ? "" : "");
		verdict(m.dacclip ? 1 : 0, t, "DAC clipping: raise tx1_att or let the output control back off");
		if (m.out_ok) {
			snprintf(t, sizeof(t), "output %.1f dBFS (target %.0f)", m.out, AOLC_TARGET_DBFS);
			verdict(fabs(m.out - AOLC_TARGET_DBFS) <= 3.0 ? 0 : 1, t,
				"output level off target: wait 1 s after a change, or check the AGC");
		}
	} else {
		verdict(0, "TX1 silent", NULL);
	}

	printf("--------------------------------------------------------------------------------\r\n");
	printf("  RESULT  %d OK  %d WARN  %d FAIL\r\n", n_ok, n_warn, n_fail);
	if (n_act) {
		int i;
		printf("  TO DO\r\n");
		for (i = 0; i < n_act; i++)
			printf("   %d. %s\r\n", i + 1, actions[i]);
	}
}

/* ======================================================= one-line KPI */
static int      ajbw_mhz = 1;            /* MOD7: 1 = AUTO (default), 0 = off, 4..56 = fixed MHz */
static int      ajbw_cur = 0;            /* MHz in use while narrowed          */
static int      ajbw_on = 0;             /* narrowed now                     */

void get_ant_kpi(double *param, char param_no)
{
	gm_t m;
	(void)param; (void)param_no;

	gm_read(&m);
	printf("KPI mode=%s state=%s", !m.tx ? "TXOFF" : (m.stap ? "SMI" : (m.pass ? "BYPASS" : "MUTED")),
	       (m.aj.valid && m.aj.jammed) ? "JAMMED" : "CLEAR");
	if (m.jn > NA + 1) printf(" jn=%.1f", m.jn); else printf(" jn=na");
	if (m.nl_ok)
		printf(" supp=%.1f supp_lo=%d jnout=%.1f cn0loss=%.2f", m.nl.supp_db, m.nl.supp_lo,
		       m.nl.jn_out_db, m.nl.cn0_loss_db);
	else printf(" supp=na supp_lo=0 jnout=na cn0loss=na");
	printf(" gain=%ld gain2=%ld agc=%s", (long)m.g1, (long)m.g2, m.agc_auto ? "AUTO" : "MANUAL");
	if (m.nf > NA + 1) printf(" nf=%.1f", m.nf); else printf(" nf=na");
	if (m.floor_int > NA + 1) printf(" floor=%.1f", m.floor_int); else printf(" floor=na");
	printf(" pk1=%.1f pk2=%.1f clip1=%lu clip2=%lu", m.pk1, m.pk2,
	       (unsigned long)m.clip1, (unsigned long)m.clip2);
	if (m.out_ok) printf(" out=%.1f", m.out); else printf(" out=na");
	printf(" taps=%d alpha=%.1f", m.ss.La, m.ss.alpha);
	printf(" tsolve=%.0f tupd=%.2f steady=%d ajbw=%d", m.ss.t_total_us, m.ss.t_int_us / 1000.0,
	       m.ss.avg_steady, ajbw_on ? ajbw_cur : 0);
	{
		int cen, chold;
		gnss_smi_get_clean(&cen, NULL, NULL, &chold, NULL);
		printf(" clean=%d pulse=%d wide=%d leakfast=%lu jtype=%s%s sweep=%d", cen ? (chold ? 2 : 1) : 0,
		       m.ss.pulse_mode, m.ss.jbw_wide, (unsigned long)m.ss.leak_fast,
		       m.ss.pulse_mode ? "PULSED_" : "", gnss_smi_jtype_name(m.ss.jtype), m.ss.sweep_mode);
	}
	{
		double r = 0.0;
		int bar = gnss_agc_barrage_rule(&r);
		printf(" agcrule=%s cliprate=%.1f rxoff=%d", bar ? "CLIPRATE" : "PEAK", r, gnss_rx2_off_get());
	}
	printf(" wn=%.2f dacclip=%lu hunts=%lu keff=%.0f tx1=%lu\r\n",
	       m.ss.wnorm_u, (unsigned long)m.dacclip, (unsigned long)m.hunts, m.ss.keff,
	       (unsigned long)m.att1);
}

/* ======================================================= live events */
/* ---------------------------------------------------- anti-jam bandwidth
 * gnss_ajbw=MHz (0 = off, default): while a WIDEBAND jammer is detected for
 * 2 s, the AD9361 RX analog filter is narrowed to MHz (LO and sample rate
 * unchanged, no restart). A barrage then puts less power into the ADC, so the
 * AGC keeps more gain (less AD9361 noise at low gain). Signals outside the
 * narrow band (GLONASS, BeiDou B1I with band 3) are lost while it is on; GPS
 * L1 / Galileo E1 / BeiDou B1C stay. 10 s after the jammer goes the band's
 * own bandwidth comes back. Each change re-tunes the filter (~ms glitch). */
static uint32_t ajbw_saved = 0;          /* bandwidth to restore, Hz         */
static int      ajbw_jam_t = 0, ajbw_clr_t = 0;   /* half-seconds            */

/* MOD7: automatic. Narrows for WIDEBAND / CHIRP / SAWTOOTH jammers (steady or
 * pulsed) whose energy reaches outside the narrowed filter; never for CW or
 * multi-tone (the null removes them, narrowing would only cost GLONASS/B1I).
 * Width: L1/E1/B1C (1575.42 MHz) kept >= 2.5 MHz inside the edge, at least
 * 20 MHz, rounded up to an even MHz: band 3 -> 20, band 2 -> 24. Back to the
 * band's own filter 10 s after the jammer is gone or no longer wide/swept.
 * The covariance restarts (or rescales, swept) at each change. */
static int ajbw_auto_mhz(uint32_t bw)
{
	uint64_t lo = 0;
	double half, w;
	int mhz;
	if (!ad9361_phy || ad9361_get_rx_lo_freq(ad9361_phy, &lo) != 0 || !lo) return 0;
	half = fabs((double)lo - 1575.42e6) + 2.5e6;
	w = 2.0 * half;
	if (w < 20e6) w = 20e6;
	mhz = (int)ceil(w / 1e6);
	if (mhz & 1) mhz++;
	if ((uint32_t)(mhz + 4) * 1000000U > bw) return 0;   /* gains under ~1 dB: not worth it */
	return mhz;
}

static void ajbw_tick(void)
{
	gm_t m;
	uint32_t bw = 0;
	int want = 0, type_ok, outside = 0, mhz;

	if (!ajbw_mhz && !ajbw_on) return;
	gm_read(&m);
	if (!ad9361_phy || ad9361_get_rx_rf_bandwidth(ad9361_phy, &bw) != 0) return;
	if (ajbw_on && bw != (uint32_t)ajbw_cur * 1000000U) {   /* band / auto re-applied */
		ajbw_on = 0;
		ajbw_jam_t = ajbw_clr_t = 0;
	}
	mhz = (ajbw_mhz == 1) ? ajbw_auto_mhz(ajbw_on ? ajbw_saved : bw) : ajbw_mhz;
	type_ok = m.stap && m.smi_ok && (m.ss.jtype == GNSS_JT_WIDE || m.ss.jtype == GNSS_JT_CHIRP ||
					 m.ss.jtype == GNSS_JT_SAWTOOTH) &&
		  ((m.aj.valid && m.aj.jammed) || m.ss.pulse_mode);
	if (type_ok && mhz > 0) {
		double edge = (double)mhz * 0.5e6 + 1.0e6;          /* jammer reaching past it */
		if (m.ss.sweep_mode)
			outside = (m.ss.sweep_lo_hz < -edge || m.ss.sweep_hi_hz > edge);
		else
			outside = (fabs(m.ss.jf_hz) + 0.5 * m.ss.jbw_hz > edge) || m.ss.jbw_class == 2;
		if (ajbw_mhz != 1) outside = 1;                      /* fixed MHz: as before */
	}
	want = type_ok && (outside || ajbw_on);   /* once narrowed the jammer looks narrower: keep it */
	if (want) { ajbw_jam_t++; ajbw_clr_t = 0; } else { ajbw_clr_t++; ajbw_jam_t = 0; }
	if (!ajbw_on && ajbw_mhz && mhz > 0 && ajbw_jam_t >= 4 && bw > (uint32_t)mhz * 1000000U) {
		ajbw_saved = bw;
		if (ad9361_set_rx_rf_bandwidth(ad9361_phy, (uint32_t)mhz * 1000000U) == 0) {
			ajbw_on = 1;
			ajbw_cur = mhz;
			gnss_smi_gain_changed_db(0.0);
			gnss_rep_event("AJ BAND     RX filter %d MHz (was %.0f): %s jammer%s", mhz, ajbw_saved / 1e6,
				       gnss_smi_jtype_name(m.ss.jtype), ajbw_mhz == 1 ? " (auto)" : "");
		}
	} else if (ajbw_on && (ajbw_clr_t >= 20 || !ajbw_mhz)) {
		if (ad9361_set_rx_rf_bandwidth(ad9361_phy, ajbw_saved) == 0) {
			ajbw_on = 0;
			gnss_smi_gain_changed_db(0.0);
			gnss_rep_event("AJ BAND     RX filter back to %.0f MHz", ajbw_saved / 1e6);
		}
	}
}

void set_ajbw(double *param, char param_no)
{
	int v = (param_no >= 1) ? (int)param[0] : -1;
	if (v != 0 && v != 1 && (v < 4 || v > 56)) {
		printf("  gnss_ajbw=1 AUTO (default) | 0 off | 4..56 fixed MHz; now %s%s\r\n",
		       ajbw_mhz == 1 ? "AUTO" : (ajbw_mhz ? "fixed" : "off"), ajbw_on ? " (narrowed now)" : "");
		if (ajbw_mhz > 1) printf("  fixed %d MHz\r\n", ajbw_mhz);
		if (ajbw_on) printf("  RX filter %d MHz now (band's own %.0f MHz)\r\n", ajbw_cur, ajbw_saved / 1e6);
		return;
	}
	ajbw_mhz = v;
	if (v == 1)
		printf("  AUTO: wideband / chirp / sawtooth jammer reaching outside for 2 s -> RX filter narrowed\r\n"
		       "  (L1 kept 2.5 MHz inside: band 3 -> 20 MHz, band 2 -> 24 MHz); never for CW / multi-tone;\r\n"
		       "  back 10 s after it goes\r\n");
	else if (v)
		printf("  wideband jammer for 2 s -> RX filter %d MHz; back 10 s after it goes\r\n", v);
	else
		printf("  anti-jam bandwidth OFF%s\r\n", ajbw_on ? " (restoring the band's filter)" : "");
	if (!v && ajbw_on) ajbw_tick();
}

int gnss_ant_poll_events(void)
{
	static uint32_t seen = 0;
	uint32_t n;
	int printed = 0;

	ajbw_tick();
	n = gnss_rep_event_total();

	while (seen < n) {
		double t; const char *txt;
		if (gnss_rep_event_get(seen, &t, &txt) == 0 &&
		    ((!strncmp(txt, "JAMMER", 6) && strncmp(txt, "JAMMER DET", 10)) || !strncmp(txt, "NULLED", 6) ||
		     !strncmp(txt, "RESIDUAL", 8) || !strncmp(txt, "ADC CLIP", 8) || !strncmp(txt, "AJ BAND", 7))) {
			printf("%s[%7.1f s] %s\r\n", printed ? "" : "\r\n", t, txt);
			printed = 1;
		}
		seen++;
	}
	return printed;
}

/* ======================================================= boot */
void gnss_ant_boot(void)
{
	gnss_tx_silence_quiet();
	gnss_radio_apply_defaults();
	gnss_tx_silence_quiet();
}

/* ======================================================= switches */
static void print_mode(void)
{
	gm_t m;
	gm_read(&m);
	printf("  -> %s   [gnss_tx=%d pass=%d stap=%d]\r\n", mode_name(&m), m.tx, m.pass, m.stap);
}

static void mvdr_off(void)
{
	double p[1] = { 0 };

	if (gnss_pt_read(GNSS_PT_REG_STATUS) & GNSS_PT_ST_MVDR_SEL_SYNCED)
		set_gnss_mvdr(p, 1);		/* raw RX1, TX path untouched */
}

/* TX1 on, no signal: maximum attenuation first, the operating value last */
static int tx_on_muted(void)
{
	if (!ad9361_phy)
		return -1;
	if (gnss_l1_set_tx_attenuation(ad9361_phy, GNSS_L1_TX_MAX_ATTEN_MDB) != GNSS_L1_OK)
		return -1;
	if (!gnss_radio_defaults_ready())
		gnss_radio_apply_defaults();
	if (gnss_l1_enable_tx(ad9361_phy, GNSS_L1_PATH_CONDUCTED_WITH_ATTENUATION) != GNSS_L1_OK)
		return -1;
	gnss_pt_set_mute(1);
	gnss_pt_set_passthrough(0);
	if (gnss_l1_set_dac_source(ad9361_phy, AXI_DAC_DATA_SEL_DMA) != GNSS_L1_OK)
		return -1;
	return (gnss_l1_set_tx_attenuation(ad9361_phy, gnss_get_op_atten_mdb()) == GNSS_L1_OK) ? 0 : -1;
}

static void all_off(void)
{
	double p[1] = { 0 };
	gnss_tx_raw(p, 1);
	mvdr_off();
}

void set_gnss_tx(double *param, char param_no)
{
	if (param_no < 1) { printf("  gnss_tx=1 or gnss_tx=0\r\n"); return; }
	gnss_quiet_clear();
	if ((int)param[0] == 0) {
		all_off();
	} else if (!gnss_ant_tx_state() && tx_on_muted() != 0) {
		all_off();
		printf("  gnss_tx: TX1 could NOT be opened; left silent\r\n");
		gnss_quiet_dump(12);
	}
	print_mode();
}

void set_pass(double *param, char param_no)
{
	double p[1] = { 1 };

	if (param_no < 1) { printf("  pass=1 or pass=0\r\n"); return; }
	gnss_quiet_clear();
	mvdr_off();					/* stap=0 in both cases */
	if ((int)param[0] != 0) {
		if (gnss_ant_tx_state()) {
			gnss_pt_set_passthrough(1);
			gnss_pt_set_mute(0);
		} else {
			gnss_tx_raw(p, 1);		/* opens TX1 with RX1 */
			if (!gnss_ant_pass_state()) {
				printf("  pass: TX1 could NOT be opened\r\n");
				gnss_quiet_dump(12);
			}
		}
	} else {
		gnss_pt_set_mute(1);
		gnss_pt_set_passthrough(0);
	}
	no_os_mdelay(2);
	print_mode();
}

void set_stap(double *param, char param_no)
{
	if (param_no < 1) { printf("  stap=1 or stap=0\r\n"); return; }
	gnss_quiet_clear();
	if ((int)param[0] != 0) {
		gm_t m;
		printf("  stap: RX gain level, output level, SMI-STAP null (~2 s) ...\r\n");
		if (gnss_autostart_run() != 0) {
			printf("  stap: FAILED, TX1 is SILENT\r\n");
			gnss_quiet_dump(25);
		} else {
			gm_read(&m);
			printf("  stap: ON   RX gain %ld dB, noise ", (long)m.g1);
			pv(m.nf, 1);
			printf(" dBFS, TX1 %lu mdB\r\n", (unsigned long)m.att1);
		}
	} else {
		mvdr_off();
	}
	print_mode();
}

/* ======================================================= settings */
void get_band(double *param, char param_no)
{
	gnss_quiet_set(0);
	get_gnss_band(param, param_no);
	gnss_quiet_set(1);
}

void set_band(double *param, char param_no)
{
	int tx, pass, stap;
	gm_t m;

	if (param_no < 1) { get_band(param, param_no); return; }
	tx = gnss_ant_tx_state(); pass = gnss_ant_pass_state(); stap = gnss_ant_stap_state();
	gnss_quiet_clear();
	if (gnss_select_band((int)param[0]) != 0) {
		printf("  band: %d is not a preset (band? lists them)\r\n", (int)param[0]);
		return;
	}
	gnss_rep_event("BAND        band=%d", (int)param[0]);
	if (stap) {
		if (gnss_autostart_run() != 0) {
			printf("  band: anti-jam restart FAILED, TX1 is SILENT\r\n");
			gnss_quiet_dump(25);
		}
	} else {
		double p[1] = { 1 };
		gnss_radio_apply_defaults();		/* leaves TX1 silent */
		if (pass)
			gnss_tx_raw(p, 1);
		else if (tx)
			(void)tx_on_muted();
	}
	gm_read(&m);
	printf("  band %d: LO %.2f MHz  BW %.0f MHz  %.2f MSPS  (%s)\r\n", m.band,
	       (double)m.lo / 1e6, m.bw / 1e6, m.fs / 1e6, m.bname);
	print_mode();
}

void set_tx1_att(double *param, char param_no)
{
	gm_t m;
	if (param_no < 1 || param[0] < 0 || param[0] >= GNSS_L1_TX_MAX_ATTEN_MDB) {
		printf("  tx1_att=mdB, 0..89749 (bigger = quieter)\r\n");
		return;
	}
	gnss_set_tx1_attenuation(param, param_no);
	gm_read(&m);
	printf("  TX1 operating attenuation %lu mdB%s\r\n", (unsigned long)m.att_op,
	       m.tx ? "" : " (applies when TX1 is on)");
	if (!m.tx)
		gnss_tx_silence_quiet();
}

void set_agc(double *param, char param_no)
{
	if (param_no < 1) { printf("  agc=1 (AUTO) or agc=0 (MANUAL)\r\n"); return; }
	gnss_agc_set_mode((int)param[0] != 0);
	printf("  RX gain %s\r\n", gnss_agc_is_auto() ? "AUTO" : "MANUAL (gain held)");
}

void set_rxgain(double *param, char param_no)
{
	int32_t g1 = 0, g2 = 0;
	if (param_no < 1) { printf("  rxgain=dB\r\n"); return; }
	set_gnss_rxgain(param, param_no);
	if (ad9361_phy) {
		ad9361_get_rx_rf_gain(ad9361_phy, 0, &g1);
		ad9361_get_rx_rf_gain(ad9361_phy, 1, &g2);
	}
	printf("  RX gain %ld / %ld dB, %s\r\n", (long)g1, (long)g2, gnss_agc_is_auto() ? "AUTO" : "MANUAL");
}

void set_smi(double *param, char param_no)
{
	gnss_smi_state_t s;
	set_gnss_smi(param, param_no);
	gnss_smi_get_state(&s);
	printf("  SMI window 2^%lu, loading %.1f x lambda_min\r\n", (unsigned long)s.kexp, s.alpha);
}

/* rxoff? : RX1/RX2 level difference and the suggested RX2 gain offset */
void get_rxoff(double *param, char param_no)
{
	gm_t m;
	gnss_pt_jam_state_t j;
	(void)param; (void)param_no;
	gm_read(&m);
	gnss_pt_jam_get_state(&j);
	printf("  RX2 gain offset %d dB now: RX1 %ld dB, RX2 %ld dB\r\n", gnss_rx2_off_get(),
	       (long)m.g1, (long)m.g2);
	if (m.p1 > NA + 1 && m.p2 > NA + 1) {
		double d = m.p2 - m.p1, de = d + (double)(m.g1 - m.g2);
		printf("  RX2 - RX1 power %+.1f dB at the ADC, %+.1f dB at equal gain (%s)\r\n", d, de,
		       (m.aj.valid && m.aj.jammed) ? "JAMMER: its level per antenna" : "noise, no jammer");
		printf("  suggested: gnss_rxoff=%d  (puts both channels at the same ADC level)\r\n",
		       (int)floor(de + 0.5));
	} else {
		printf("  no SMI window yet: stap=1 first\r\n");
	}
	if (j.available && j.have_ch && j.n1_q8 != (int16_t)GNSS_PT_JAM_Q8_NONE &&
	    j.n2_q8 != (int16_t)GNSS_PT_JAM_Q8_NONE)
		printf("  FPGA estimator noise RX2 - RX1 %+.1f dB at the ADC\r\n",
		       (j.n2_q8 - j.n1_q8) / 256.0);
	printf("  Best: fix the hardware (antenna LNA supply, cable). The offset is an A/B\r\n");
	printf("  setting: compare J/N out, S and C/N0 with gnss_rxoff=0 and =N.\r\n");
}

void set_rxoff(double *param, char param_no)
{
	if (param_no < 1 || gnss_rx2_off_set((int)param[0]) != 0) {
		printf("  gnss_rxoff=dB, -20..20 (RX2 gain = RX1 gain - dB), now %d\r\n", gnss_rx2_off_get());
		return;
	}
	gnss_rx2_off_apply();
	printf("  RX2 gain = RX1 gain - %d dB, applied (the null re-forms in a few ms)\r\n",
	       gnss_rx2_off_get());
	gnss_rep_event("RX2 OFFSET  RX2 gain = RX1 gain - %d dB", gnss_rx2_off_get());
}

void set_agcclip(double *param, char param_no)
{
	if (param_no < 1 || gnss_agc_set_clip_ppm((int)param[0]) != 0) {
		printf("  gnss_agcclip=ppm, 0..2000 (0 = peak rule), now %d\r\n", gnss_agc_get_clip_ppm());
		return;
	}
	if (gnss_agc_get_clip_ppm() > 0)
		printf("  barrage AGC: gain up to an ADC clip rate of %d ppm (1e-6 of the samples)\r\n",
		       gnss_agc_get_clip_ppm());
	else
		printf("  barrage AGC: old peak rule (-2 dBFS)\r\n");
}

/* taps= : STAP taps per antenna in use (1 .. what the bitstream has) */
void get_taps(double *param, char param_no)
{
	gnss_smi_state_t ss;
	(void)param; (void)param_no;
	(void)gnss_smi_available();
	gnss_smi_get_state(&ss);
	printf("  STAP taps %d per antenna in use (bitstream has %d): %d weights + 2 image taps\r\n",
	       ss.La, ss.L, 2 * ss.La);
	{
		int w1 = 0, w2 = 0;
		if (gnss_smi_weight_taps(&w1, &w2) == 0)
			printf("  weights in the FPGA now: %d taps on RX1, %d on RX2 non-zero (solve %lu)\r\n",
			       w1, w2, (unsigned long)ss.updates);
		else
			printf("  weights in the FPGA now: -- (no solve yet: the setting applies once stap=1)\r\n");
	}
	printf("  taps=N, N = 1..%d. 1 = space only (one weight per antenna). Wide jammers\r\n", ss.L);
	printf("  need taps: 8..16 for a barrage over 56 MHz; CW is nulled with any N.\r\n");
}

void set_taps(double *param, char param_no)
{
	gnss_smi_state_t ss;
	(void)gnss_smi_available();
	gnss_smi_get_state(&ss);
	if (param_no < 1 || gnss_smi_set_taps((int)param[0]) != 0) {
		printf("  taps=N with N = 1..%d (now %d)\r\n", ss.L, ss.La);
		return;
	}
	gnss_smi_get_state(&ss);
	printf("  STAP taps %d per antenna (%d of %d, centred on the reference tap; others 0)\r\n",
	       ss.La, ss.La, ss.L);
	if (gnss_ant_stap_state())
		printf("  applied from the next window (~6 ms), no restart. taps? shows the weights.\r\n");
	else
		printf("  stored: used when stap=1 starts the null (stap is off now)\r\n");
	gnss_rep_event("TAPS        %d per antenna", ss.La);
}

void set_track(double *param, char param_no)
{
	gnss_smi_state_t ss;
	if (param_no >= 1) {
		gnss_smi_set_track(param[0] != 0.0);
		gnss_rep_event("TRACK       %s (moving-jammer averaging)", gnss_smi_get_track() ? "on" : "off");
	}
	gnss_smi_get_state(&ss);
	printf("  null tracking %s: when the current weights leak 3 dB more than usual on a\r\n"
	       "  new window (jammer or platform moving), averaging runs fast for 1 s; 6 dB more:\r\n"
	       "  solved from that window alone.  fast %lu times, restarted %lu times\r\n",
	       gnss_smi_get_track() ? "ON" : "OFF", (unsigned long)ss.leak_fast, (unsigned long)ss.leak_reset);
}

void set_pulse(double *param, char param_no)
{
	gnss_smi_state_t ss;
	if (param_no >= 1) {
		gnss_smi_set_pulse(param[0] != 0.0);
		gnss_rep_event("PULSE MODE  %s allowed", gnss_smi_get_pulse() ? "" : "NOT");
	}
	gnss_smi_get_state(&ss);
	printf("  pulse mode %s: 4 pulse starts within 2 s -> only jammer-on windows are solved,\r\n"
	       "  the null stays loaded between pulses, AGC rises at most 1 dB / 5 s.\r\n"
	       "  now: %s, entered %lu times, %lu jammer-off windows kept\r\n",
	       gnss_smi_get_pulse() ? "ALLOWED" : "OFF", ss.pulse_mode ? "PULSED JAMMER" : "not pulsed",
	       (unsigned long)ss.pulse_modes, (unsigned long)ss.pulse_skipped);
}

void set_clean(double *param, char param_no)
{
	int en, hold;
	double in_db, out_db;
	uint32_t n;
	gnss_smi_get_clean(&en, &in_db, &out_db, &hold, &n);
	if (param_no >= 1) {
		en = (param[0] != 0.0);
		if (param_no >= 2) in_db = param[1];
		if (param_no >= 3) out_db = param[2];
		gnss_smi_set_clean(en, in_db, out_db);
		gnss_smi_get_clean(&en, &in_db, &out_db, &hold, &n);
		gnss_rep_event("CLEAN HOLD  %s (in %.1f dB, out %.1f dB)", en ? "on" : "off", in_db, out_db);
	}
	printf("  clean hold %s: no jammer (J/N < %.1f dB for 2 s) -> TX1 = RX1 reference tap,\r\n"
	       "  fixed filter (the pass=1 signal); J/N > %.1f dB -> nulling weights within ~1.5 ms.\r\n"
	       "  now: %s, entered %lu times.  gnss_clean=0 = always solved weights (before MOD-33c)\r\n",
	       en ? "ON" : "OFF", in_db, out_db, hold ? "HOLDING (fixed RX1 path)" : "solved weights on TX1",
	       (unsigned long)n);
}

void set_smiavg(double *param, char param_no)
{
	set_gnss_smiavg(param, param_no);
	if (param_no >= 2 && gnss_smi_set_avg_steady((int)param[1]) != 0)
		printf("  steady shift 0..6 (0 = no steady averaging)\r\n");
	{
		gnss_smi_state_t ss;
		gnss_smi_get_state(&ss);
		printf("  steady (1 s after a power step): beta 1/%d%s\r\n",
		       ss.avg_sh_st ? (1 << ss.avg_sh_st) : 1, ss.avg_sh_st ? "" : " (off)");
	}
	printf("  SMI averaging %d (beta 1/%d)\r\n", gnss_smi_get_avg(),
	       gnss_smi_get_avg() ? (1 << gnss_smi_get_avg()) : 1);
}
