/***************************************************************************//**
 *   @file   gnss_ant.c
 *   @brief  gnss_tx / pass / stap interlocks, main menu and aliases on top of
 *           the MVDR2 firmware (see gnss_ant.h). The anti-jam behaviour is
 *           the MVDR2 code path unchanged: stap=1 IS the MVDR2 automatic
 *           start (gnss_autostart_run), pass=1 IS its raw retransmit, and the
 *           SMI solver, fast/slow RX gain, clip guard, AOLC and event monitor
 *           run from the main loop exactly as before.
 *
 *   The three states are read back from the hardware (gnss_passthrough
 *   CONTROL/STATUS, DAC source, TX1 attenuation), so they stay right when
 *   the MVDR2 commands (gnss_auto=, gnss_mvdr=, gnss_band=...) are used.
*******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include "app_config.h"
#include "ad9361_api.h"
#include "command.h"
#include "gnss_ant.h"
#include "gnss_l1.h"
#include "gnss_passthrough.h"
#include "gnss_autostart.h"
#include "gnss_config.h"
#include "gnss_smi.h"
#include "gnss_info.h"
#include "no_os_delay.h"

extern struct ad9361_rf_phy *ad9361_phy;

/* ------------------------------------------------------------ state ---- */
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

static void print_modes(void)
{
	int t = gnss_ant_tx_state(), p = gnss_ant_pass_state(), s = gnss_ant_stap_state();

	printf("  MODE      gnss_tx=%d  pass=%d  stap=%d   ->  TX1 %s\r\n", t, p, s,
	       !t ? "SILENT" : (s ? "anti-jam output (MVDR SMI-STAP, jammer nulled)"
			       : (p ? "raw RX1 (no nulling)" : "on, no signal (muted)")));
}

/* -------------------------------------------------------------- boot ---- */
void gnss_ant_boot(void)
{
	printf("\r\n[boot] applying the radio plan; TX1 stays silent "
	       "(gnss_tx=0 pass=0 stap=0)\r\n");
	gnss_tx_silence_quiet();
	gnss_radio_apply_defaults();
	gnss_tx_silence_quiet();
}

/* ------------------------------------------------------------- menu ---- */
static void mhz(uint64_t hz, int dec)
{
	uint64_t k = hz / 1000ULL;			/* kHz */
	if (dec == 2)
		printf("%lu.%02lu", (unsigned long)(k / 1000ULL), (unsigned long)((k % 1000ULL) / 10ULL));
	else
		printf("%lu", (unsigned long)(k / 1000ULL));
}

static void item(const char *cmd, const char *what)
{
	printf("    %-20s %s\r\n", cmd, what);
}

void gnss_ant_menu(void)
{
	uint64_t lo = 0; uint32_t bw = 0, fs = 0, at = 0, a1 = 0;
	int32_t g0 = 0, g1 = 0, g2 = 0;
	int bn = -1; const char *bname = "";

	gnss_get_plan(&lo, &bw, &fs, &g0, &at);
	gnss_band_info(&bn, &bname);
	if (ad9361_phy) {
		ad9361_get_rx_rf_gain(ad9361_phy, 0, &g1);
		ad9361_get_rx_rf_gain(ad9361_phy, 1, &g2);
		ad9361_get_tx_attenuation(ad9361_phy, 0, &a1);
	}
	printf("\r\n=====================================================================\r\n");
	printf("  ANTSDR E310  GNSS ANTI-JAM   (LVDS 2R2T, MVDR2 firmware, SMI-STAP)\r\n");
	printf("---------------------------------------------------------------------\r\n");
	printf("  RADIO     LO ");  mhz(lo, 2);
	printf(" MHz   BW ");       mhz(bw, 0);
	printf(" MHz   ");          mhz(fs, 2);
	printf(" MSPS   preset %d\r\n", bn);
	printf("            %s\r\n", bname);
	printf("  RX GAIN   RX1 %ld dB  RX2 %ld dB  (%s)\r\n", (long)g1, (long)g2,
	       gnss_agc_is_auto() ? "AUTO: levelled, clip guard on" : "MANUAL");
	printf("  TX1       attenuation %lu mdB%s   operating value %lu mdB\r\n",
	       (unsigned long)a1, (a1 >= GNSS_L1_TX_MAX_ATTEN_MDB) ? " (silent)" : "",
	       (unsigned long)gnss_get_op_atten_mdb());
	print_modes();
	printf("---------------------------------------------------------------------\r\n");
	printf("  MODE  (default all 0)\r\n");
	item("gnss_tx=1 / =0",   "TX1 on (muted) / everything off, TX1 silent");
	item("pass=1 / =0",      "raw RX1 -> TX1 (sets gnss_tx=1, stap=0) / mute");
	item("stap=1 / =0",      "ANTI-JAM ON: gain level + AOLC + SMI-STAP null,");
	item("",                 "  sets gnss_tx=1 pass=1 (~2 s) / back to raw RX1");
	item("gnss_tx? pass? stap?", "state read back from the hardware");
	printf("  ANTI-JAM\r\n");
	item("stap?",            "core, window, loading, noise floor, suppression");
	item("gnss_check?",      "one-page check with verdict");
	item("gnss_report?",     "full report: radio, gain, jammer, null, events");
	item("gnss_jam?",        "jammer present, power, J/N, angle");
	item("gnss_supp?",       "suppression: J/N at the ADC, actual null depth");
	item("gnss_smiavg=0..4", "covariance averaging (default 2 = beta 1/4)");
	printf("  RADIO\r\n");
	item("gnss_band? / =N",  "presets: 3 = all four systems (default),");
	item("",                 "  4 / 5 = narrow plans for a barrage");
	item("gnss_plan=LO,BW,FS", "any plan in MHz, MHz, MSPS");
	item("tx1_attenuation=N", "TX1 operating level in mdB (bigger = quieter)");
	item("gnss_agc? / =1|0", "RX gain AUTO (MVDR-safe, default) / MANUAL");
	item("gnss_rxgain=N",    "manual RX1+RX2 gain (same on both)");
	item("gnss_aolc? / =24", "output level control, -24 dBFS");
	printf("  RX INFO\r\n");
	item("rx?",              "RX gain, noise floor vs target, ADC peak/clip");
	item("noise?",           "INTERNAL noise: RX noise vs gain, both channels");
	item("noise=G",          "noise spectrum per antenna at RX gain G dB");
	item("status?",          "AD9361 + FPGA block state, counters");
	item("gnss_clk?",        "AD9361 clock chain (ADC clock, decimation)");
	printf("  MORE\r\n");
	item("help?",            "every command        guide?  the MVDR2 guide");
	item("1 .. 9",           "information pages    Enter   status + menu");
	printf("=====================================================================\r\n");
}

/* ------------------------------------------------------- the switches ---- */
static void hdr(const char *c, const char *what)
{
	printf("\r\n--- %s : %s\r\n", c, what);
}

static void mvdr_off(void)
{
	double p[1] = { 0 };

	if (gnss_pt_read(GNSS_PT_REG_STATUS) & GNSS_PT_ST_MVDR_SEL_SYNCED)
		set_gnss_mvdr(p, 1);		/* raw RX1, TX path untouched */
}

/* TX1 chain on with no signal: same ordering as the MVDR2 openers
 * (maximum attenuation first, the operating value last) */
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
	if (gnss_l1_set_tx_attenuation(ad9361_phy, GNSS_L1_TX_MAX_ATTEN_MDB) != GNSS_L1_OK)
		return -1;
	gnss_pt_set_mute(1);
	gnss_pt_set_passthrough(0);
	if (gnss_l1_set_dac_source(ad9361_phy, AXI_DAC_DATA_SEL_DMA) != GNSS_L1_OK)
		return -1;
	return (gnss_l1_set_tx_attenuation(ad9361_phy, gnss_get_op_atten_mdb()) == GNSS_L1_OK) ? 0 : -1;
}

void set_gnss_tx(double *param, char param_no)
{
	double p[1];

	hdr("gnss_tx", "TX1 chain (1 = on, muted; 0 = everything off)");
	if (param_no < 1) {
		printf("  usage: gnss_tx=1 or gnss_tx=0\r\n");
		return;
	}
	if ((int)param[0] == 0) {
		p[0] = 0;
		gnss_tx_raw(p, 1);		/* MVDR2 abort path: DAC zero, mute, max atten */
		mvdr_off();
	} else if (!gnss_ant_tx_state()) {
		if (tx_on_muted() != 0) {
			p[0] = 0;
			gnss_tx_raw(p, 1);
			printf("  gnss_tx: TX1 could NOT be opened safely; left silent\r\n");
		}
	} else {
		printf("  TX1 is already on\r\n");
	}
	print_modes();
}

void get_gnss_tx(double *param, char param_no)
{
	hdr("gnss_tx?", "TX1 state from the hardware");
	print_modes();
	gnss_tx_hw_print(param, param_no);
}

void set_pass(double *param, char param_no)
{
	double p[1];

	hdr("pass", "raw RX1 -> TX1 (1 sets gnss_tx=1 and stap=0)");
	if (param_no < 1) {
		printf("  usage: pass=1 or pass=0\r\n");
		return;
	}
	if ((int)param[0] != 0) {
		mvdr_off();				/* stap=0 */
		if (gnss_ant_tx_state()) {
			gnss_pt_set_passthrough(1);
			gnss_pt_set_mute(0);
		} else {
			p[0] = 1;
			gnss_tx_raw(p, 1);		/* MVDR2 gnss_tx=1: opens TX1 with RX1 */
		}
	} else {
		mvdr_off();				/* stap=0 too */
		gnss_pt_set_mute(1);
		gnss_pt_set_passthrough(0);
		printf("  RX1 removed from TX1; TX1 stays on (muted). gnss_tx=0 = silent\r\n");
	}
	no_os_mdelay(2);
	print_modes();
}

void get_pass(double *param, char param_no)
{
	hdr("pass?", "RX path state from the hardware");
	print_modes();
	gnss_pt_print_state();
}

void set_stap(double *param, char param_no)
{
	hdr("stap", "anti-jam (1 sets gnss_tx=1 and pass=1)");
	if (param_no < 1) {
		printf("  usage: stap=1 or stap=0\r\n");
		return;
	}
	if ((int)param[0] != 0) {
		printf("  running the MVDR2 automatic start: radio plan, RX gain level,\r\n"
		       "  AOLC, SMI-STAP null, then TX1 opens at the operating level\r\n");
		if (gnss_autostart_run() != 0)
			printf("  stap: the automatic start did not finish: TX1 is SILENT "
			       "(see the [FAIL] line above)\r\n");
	} else {
		mvdr_off();
		printf("  anti-jam off: TX1 carries raw RX1 (pass=1). pass=0 / gnss_tx=0 to stop\r\n");
	}
	print_modes();
}

void get_stap(double *param, char param_no)
{
	hdr("stap?", "anti-jam state");
	print_modes();
	get_gnss_null(param, param_no);
	get_gnss_smi(param, param_no);
	get_gnss_supp(param, param_no);
}

/* ------------------------------------------------------------ aliases ---- */
void get_ant_status(double *param, char param_no)
{
	hdr("status?", "AD9361 + FPGA block");
	print_modes();
	get_gnss_status(param, param_no);
}

void get_rx_info(double *param, char param_no)
{
	hdr("rx?", "RX gain, noise floor, ADC headroom");
	get_gnss_rxgain(param, param_no);
}

void get_noise(double *param, char param_no)
{
	hdr("noise?", "internal noise: RX noise vs gain on both channels "
	    "(TX1 output muted ~1 s)");
	get_gnss_floor(param, param_no);
}

void set_noise(double *param, char param_no)
{
	hdr("noise=G", "RX noise spectrum per antenna at gain G dB (gain restored)");
	set_gnss_spec(param, param_no);
}

void get_guide(double *param, char param_no)
{
	gnss_info_menu();
}
