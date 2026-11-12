/******************************************************************************
 * gnss_band7.c -- custom RF profile "band 7" and the start-up band
 * (antsdr-fw-patch, gnss_crpa for the ANTSDR E310 under Linux).
 *
 * The SMI-PI v4.9.14 firmware has seven fixed profiles, gnss_band=0..6.
 * Band 7 takes the operator's own values, typed at any time on the console
 * (PuTTY), e.g. in the field:
 *
 *     gnss_band=7,1575.42,61.44,56,30,50
 *                 |       |     |  |  +-- RX1/RX2 gain, dB
 *                 |       |     |  +----- TX1 attenuation, dB (0.25 dB steps)
 *                 |       |     +-------- RF bandwidth, MHz (RX and TX)
 *                 |       +-------------- sample rate, MS/s (RX and TX)
 *                 +---------------------- LO, MHz (RX and TX)
 *
 * sets the values, saves them and switches to band 7 at once (same steps as
 * any gnss_band=: FIRs off, LO, rate, bandwidth, calibration, the anti-jam
 * relearns the noise floor, TX1 back on with the band's attenuation).
 * Values left out keep their last setting: gnss_band=7,,,,25 is not
 * possible, but gnss_band7= with fewer values is (LO first). gnss_band=7 on
 * its own re-applies the stored values.
 *
 *     gnss_band7=<LO>,<MS/s>,<BW>,<att>,<gain>  store (and apply if band 7 is on)
 *     gnss_band7?                               show
 *     gnss_band_boot=<0..7> / gnss_band_boot?   band used at start-up
 *
 * Units are taken from the size of the number like radio= does: LO below
 * 100000 is MHz (else Hz), rate and bandwidth below 1000 are MS/s / MHz.
 * Limits: LO 70..6000 MHz, rate 2.084..61.44 MS/s, bandwidth 0.2..56 MHz,
 * TX1 attenuation 0..89.75 dB, RX gain 0..76 dB (the AD9361 refuses what the
 * gain table at that LO cannot do; gnss_status? shows what it took).
 *
 * Saved in /mnt/jffs2/gnss_crpa.conf (flash, survives a reboot) or the file
 * named by GNSS_CRPA_CONF.
 *
 * RX and TX always get the same LO and rate: the retransmitted signal must
 * leave on the frequency it arrived on, or the receiver sees nothing.
 * TX1 IS CONDUCTED, ATTENUATED COAX ONLY -- never an antenna.
 *****************************************************************************/
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>

#include "console.h"
#include "command.h"
#include "gnss_aj.h"
#include "gnss_smi_agc.h"

static int boot_band = GNSS_BAND_DEFAULT;
static int saved_agc = -1;      /* crpa5: gnss_rx_agc=, -1 = never set   */
static int saved_avg = -1;      /* crpa5: gnss_aj_avg= x 1000, 0 = auto  */
static char conf_file[256];

static const char *conf_path(void)
{
	const char *e = getenv("GNSS_CRPA_CONF");
	struct stat st;

	if (conf_file[0])
		return conf_file;
	if (e && *e)
		snprintf(conf_file, sizeof(conf_file), "%s", e);
	else if (!stat("/mnt/jffs2", &st) && S_ISDIR(st.st_mode) && !access("/mnt/jffs2", W_OK))
		snprintf(conf_file, sizeof(conf_file), "/mnt/jffs2/gnss_crpa.conf");
	else
		snprintf(conf_file, sizeof(conf_file), "/etc/gnss_crpa.conf");
	return conf_file;
}

static int persistent(void)
{
	return strncmp(conf_path(), "/etc/", 5) != 0;
}

static int conf_save(void)
{
	uint64_t lo; uint32_t fs, bw, att; int32_t g;
	char tmp[272];
	FILE *f;

	gnss_band_custom_get(&lo, &fs, &bw, &att, &g);
	snprintf(tmp, sizeof(tmp), "%s.new", conf_path());
	f = fopen(tmp, "w");
	if (!f)
		return -errno;
	fprintf(f, "# gnss_crpa settings (written by gnss_band7= / gnss_band_boot=)\n"
		"band7_lo_hz=%llu\nband7_fs_hz=%lu\nband7_bw_hz=%lu\nband7_att_mdb=%lu\n"
		"band7_rxg_db=%ld\nband_boot=%d\n",
		(unsigned long long)lo, (unsigned long)fs, (unsigned long)bw,
		(unsigned long)att, (long)g, boot_band);
	if (saved_agc >= 0) fprintf(f, "rx_agc=%d\n", saved_agc);
	if (saved_avg >= 0) fprintf(f, "aj_avg_milli=%d\n", saved_avg);
	fflush(f);
	fsync(fileno(f));
	fclose(f);
	if (rename(tmp, conf_path()))
		return -errno;
	return 0;
}

void gnss_band7_load(void)
{
	uint64_t lo; uint32_t fs, bw, att; int32_t g;
	char line[128];
	FILE *f;

	gnss_band_custom_get(&lo, &fs, &bw, &att, &g);
	f = fopen(conf_path(), "r");
	if (f) {
		while (fgets(line, sizeof(line), f)) {
			char *eq = strchr(line, '=');
			if (line[0] == '#' || !eq)
				continue;
			*eq++ = 0;
			if (!strcmp(line, "band7_lo_hz"))        lo = strtoull(eq, NULL, 10);
			else if (!strcmp(line, "band7_fs_hz"))   fs = (uint32_t)strtoul(eq, NULL, 10);
			else if (!strcmp(line, "band7_bw_hz"))   bw = (uint32_t)strtoul(eq, NULL, 10);
			else if (!strcmp(line, "band7_att_mdb")) att = (uint32_t)strtoul(eq, NULL, 10);
			else if (!strcmp(line, "band7_rxg_db"))  g = (int32_t)strtol(eq, NULL, 10);
			else if (!strcmp(line, "band_boot"))     boot_band = atoi(eq);
			else if (!strcmp(line, "rx_agc"))        saved_agc = atoi(eq);
			else if (!strcmp(line, "aj_avg_milli"))  saved_avg = atoi(eq);
		}
		fclose(f);
	}
	/* a damaged file must not leave the radio somewhere odd */
	if (lo < 70000000ULL || lo > 6000000000ULL) lo = 1575420000ULL;
	if (fs < 2083334U || fs > 61440000U)         fs = 61440000U;
	if (bw < 200000U || bw > 56000000U)          bw = 56000000U;
	if (att > 89750U)                            att = 30000U;
	if (g < 0 || g > 76)                         g = 50;
	if (boot_band < 0 || boot_band >= GNSS_BAND_COUNT) boot_band = GNSS_BAND_DEFAULT;
	gnss_band_custom_set(lo, fs, bw, att, g);
}

int gnss_band_boot(void)
{
	return boot_band;
}

static void print_band7(void)
{
	uint64_t lo; uint32_t fs, bw, att; int32_t g;
	char l[160];

	gnss_band_custom_get(&lo, &fs, &bw, &att, &g);
	snprintf(l, sizeof(l), "GNSS_BAND 7 (custom): LO %.3f MHz, %.3f MS/s, RF bandwidth %.3f MHz, "
		 "TX1 attenuation %.2f dB, RX gain %ld dB\n",
		 (double)lo / 1e6, (double)fs / 1e6, (double)bw / 1e6, (double)att / 1000.0, (long)g);
	console_print("%s", l);
	console_print("  in use now: %s   start-up band: %d   saved in %s%s\n",
		      (char *)(gnss_band_get() == GNSS_BAND_CUSTOM ? "YES" : "no"), (long)boot_band,
		      (char *)conf_path(), (char *)(persistent() ? "" : " (RAM: lost at power-off)"));
	if (bw > fs)
		console_print("  note: RF bandwidth wider than the sample rate aliases; it is normally\n"
			      "  about 0.9 x the rate (61.44 MS/s -> 56 MHz, 30.72 MS/s -> 27 MHz).\n");
}

/* Check every value first; change nothing unless all are good. */
int gnss_band7_values(double *v, int n)
{
	uint64_t lo; uint32_t fs, bw, att; int32_t g;

	gnss_band_custom_get(&lo, &fs, &bw, &att, &g);
	if (n < 1) {
		console_print("gnss_band7: give at least the LO, e.g. gnss_band=7,1575.42,61.44,56,30,50\n");
		return -1;
	}
	if (n >= 1 && gnss_radio_lo_hz(v[0], &lo)) {
		console_print("gnss_band7: LO must be 70 .. 6000 MHz (e.g. 1575.42)\n");
		return -1;
	}
	if (n >= 2 && gnss_radio_rate_hz(v[1], &fs)) {
		console_print("gnss_band7: sample rate must be 2.084 .. 61.44 MS/s (e.g. 61.44)\n");
		return -1;
	}
	if (n >= 3 && gnss_radio_bw_hz(v[2], &bw)) {
		console_print("gnss_band7: RF bandwidth must be 0.2 .. 56 MHz (e.g. 56)\n");
		return -1;
	}
	if (n >= 4 && gnss_radio_att_mdb(v[3], &att)) {
		console_print("gnss_band7: TX1 attenuation must be 0 .. 89.75 dB (e.g. 30)\n");
		return -1;
	}
	if (n >= 5) {
		if (v[4] < 0.0 || v[4] > 76.0) {
			console_print("gnss_band7: RX gain must be 0 .. 76 dB (e.g. 50)\n");
			return -1;
		}
		g = (int32_t)(v[4] + 0.5);
	}
	if (att < 10000U)
		console_print("gnss_band7: WARNING TX1 attenuation below 10 dB -- conducted coax with an "
			      "external attenuator only, never an antenna.\n");
	gnss_band_custom_set(lo, fs, bw, att, g);
	if (conf_save())
		console_print("gnss_band7: values set, but %s could not be written (%s)\n",
			      (char *)conf_path(), (char *)strerror(errno));
	return 0;
}

void get_gnss_band7(double *param, char param_no)
{
	(void)param; (void)param_no;
	print_band7();
}

void set_gnss_band7(double *param, char param_no)
{
	if (gnss_band7_values(param, param_no) != 0)
		return;
	if (gnss_band_get() == GNSS_BAND_CUSTOM) {
		if (gnss_band_set(GNSS_BAND_CUSTOM) != 0)
			console_print("gnss_band: the AD9361 refused part of the setting; check gnss_status?\n");
		gnss_tx_always();
	} else {
		console_print("gnss_band7: stored. gnss_band=7 switches to it.\n");
	}
	print_band7();
}

void get_gnss_band_boot(double *param, char param_no)
{
	(void)param; (void)param_no;
	console_print("GNSS_BAND_BOOT: %d (saved in %s)\n", (long)boot_band, (char *)conf_path());
}

void set_gnss_band_boot(double *param, char param_no)
{
	if (param_no < 1 || param[0] < 0.0 || param[0] > (double)(GNSS_BAND_COUNT - 1)) {
		console_print("gnss_band_boot= needs 0 .. 7\n");
		return;
	}
	boot_band = (int)param[0];
	if (conf_save())
		console_print("gnss_band_boot: could not write %s\n", (char *)conf_path());
	get_gnss_band_boot(param, param_no);
}

/* ---- crpa5: gnss_rx_agc= and gnss_aj_avg= kept with the band 7 values ---------------- */
void gnss_settings_save(int rx_agc, int avg_milli)
{
	if (rx_agc >= 0) saved_agc = rx_agc;
	if (avg_milli >= 0) saved_avg = avg_milli;
	if (conf_save() != 0)
		console_print("  (setting active; not saved: %s)\n", (char *)conf_path());
	else
		console_print("  (saved in %s, used again at the next start)\n", (char *)conf_path());
}

void gnss_settings_apply(void)
{
	if (saved_agc >= 0 && saved_agc <= 2) {
		console_print("settings: gnss_rx_agc=%d (saved)\n", (long)saved_agc);
		gnss_rxg_set_mode(saved_agc);
	}
	if (saved_avg == 0) {
		gnss_aj_set_avg(0.0, 1);
	} else if (saved_avg >= 50 && saved_avg <= 990) {
		console_print("settings: gnss_aj_avg=%s (saved)\n", gnss_fmt((double)saved_avg / 1000.0, 2));
		gnss_aj_set_avg((double)saved_avg / 1000.0, 0);
	}
}
