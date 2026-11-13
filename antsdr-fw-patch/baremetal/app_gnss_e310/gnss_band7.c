/******************************************************************************
 * gnss_band7.c -- custom RF profile "band 7" and the start-up band
 * (antsdr-fw-patch, design A, bare metal).
 *
 * The SMI-PI v4.9.14 firmware has seven fixed profiles, gnss_band=0..6.
 * Band 7 takes the operator's own values, typed at any time on the UART
 * console (PuTTY), e.g. in the field:
 *
 *     gnss_band=7,1575.42,61.44,56,30,50
 *                 |       |     |  |  +-- RX1/RX2 gain, dB
 *                 |       |     |  +----- TX1 attenuation, dB (0.25 dB steps)
 *                 |       |     +-------- RF bandwidth, MHz (RX and TX)
 *                 |       +-------------- sample rate, MS/s (RX and TX)
 *                 +---------------------- LO, MHz (RX and TX)
 *
 * sets the values, saves them and switches to band 7 at once. gnss_band=7 on
 * its own re-applies the stored values.
 *
 *     gnss_band7=<LO>,<MS/s>,<BW>,<att>,<gain>  store (and apply if band 7 is on)
 *     gnss_band7?                               show
 *     gnss_band_boot=<0..7> / gnss_band_boot?   band used at power-on
 *
 * Storage: the SD card's FAT partition, 0:/GNSSCRPA.CFG, through FatFs
 * (xilffs), in a GNSS_LOG_SD build (the default of antsdr-fw-patch
 * ./build.sh baremetal). Without GNSS_LOG_SD the values are kept until
 * power-off only.
 *
 * Limits: LO 70..6000 MHz, rate 2.084..61.44 MS/s, bandwidth 0.2..56 MHz,
 * TX1 attenuation 0..89.75 dB, RX gain 0..76 dB.
 * RX and TX always get the same LO and rate.
 * TX1 IS CONDUCTED, ATTENUATED COAX ONLY -- never an antenna.
 *****************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "app_config.h"
#include "console.h"
#include "command.h"
#include "gnss_aj.h"
#include "gnss_smi_agc.h"

#ifdef GNSS_LOG_SD
#include "ff.h"
#define CONF_PATH "0:/GNSSCRPA.CFG"
#endif

static int boot_band = GNSS_BAND_DEFAULT;
static int saved_agc = -1;      /* crpa5: gnss_rx_agc=, -1 = never set   */
static int saved_avg = -1;      /* crpa5: gnss_aj_avg= x 1000, 0 = auto  */

/* ---- tiny formatter (no printf %f on bare metal) ------------------------- */
static int put_str(char *b, int i, int n, const char *s)
{
	while (*s && i < n - 1) b[i++] = *s++;
	b[i] = 0;
	return i;
}

static int put_u(char *b, int i, int n, unsigned long long v)
{
	char t[24];
	int k = 0;
	do { t[k++] = (char)('0' + (int)(v % 10ULL)); v /= 10ULL; } while (v);
	while (k && i < n - 1) b[i++] = t[--k];
	b[i] = 0;
	return i;
}

static int put_line(char *b, int i, int n, const char *key, unsigned long long v)
{
	i = put_str(b, i, n, key);
	i = put_u(b, i, n, v);
	return put_str(b, i, n, "\n");
}

/* ---- storage ---------------------------------------------------------------- */
#ifdef GNSS_LOG_SD
static FATFS fs7;
static int fs7_mounted;

static int conf_mount(void)
{
	if (!fs7_mounted && f_mount(&fs7, "0:/", 1) == FR_OK)
		fs7_mounted = 1;
	return fs7_mounted ? 0 : -1;
}

static int conf_read(char *buf, unsigned len)
{
	FIL f;
	UINT br = 0;
	if (conf_mount() != 0) return -1;
	if (f_open(&f, CONF_PATH, FA_READ) != FR_OK) return -1;
	if (f_read(&f, buf, len - 1U, &br) != FR_OK) br = 0;
	(void)f_close(&f);
	buf[br] = 0;
	return (int)br;
}

static int conf_write(const char *buf, unsigned len)
{
	FIL f;
	UINT bw = 0;
	if (conf_mount() != 0) return -1;
	if (f_open(&f, CONF_PATH, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) return -1;
	if (f_write(&f, buf, len, &bw) != FR_OK || bw != len) { (void)f_close(&f); return -1; }
	return (f_close(&f) == FR_OK) ? 0 : -1;
}

static const char *conf_where(void) { return "SD card 0:/GNSSCRPA.CFG"; }
#else
static int conf_read(char *buf, unsigned len) { (void)buf; (void)len; return -1; }
static int conf_write(const char *buf, unsigned len) { (void)buf; (void)len; return -1; }
static const char *conf_where(void) { return "RAM only (build with GNSS_LOG_SD to save on the SD card)"; }
#endif

static int conf_save(void)
{
	uint64_t lo; uint32_t fs, bw, att; int32_t g;
	char b[256];
	int i = 0;

	gnss_band_custom_get(&lo, &fs, &bw, &att, &g);
	i = put_str(b, i, sizeof(b), "# gnss settings (gnss_band7= / gnss_band_boot=)\n");
	i = put_line(b, i, sizeof(b), "band7_lo_hz=", lo);
	i = put_line(b, i, sizeof(b), "band7_fs_hz=", fs);
	i = put_line(b, i, sizeof(b), "band7_bw_hz=", bw);
	i = put_line(b, i, sizeof(b), "band7_att_mdb=", att);
	i = put_line(b, i, sizeof(b), "band7_rxg_db=", (unsigned long long)(g < 0 ? 0 : g));
	i = put_line(b, i, sizeof(b), "band_boot=", (unsigned long long)boot_band);
	if (saved_agc >= 0) i = put_line(b, i, sizeof(b), "rx_agc=", (unsigned long long)saved_agc);
	if (saved_avg >= 0) i = put_line(b, i, sizeof(b), "aj_avg_milli=", (unsigned long long)saved_avg);
	return conf_write(b, (unsigned)i);
}

void gnss_band7_load(void)
{
	uint64_t lo; uint32_t fs, bw, att; int32_t g;
	char buf[512], *line, *next;

	gnss_band_custom_get(&lo, &fs, &bw, &att, &g);
	if (conf_read(buf, sizeof(buf)) > 0) {
		for (line = buf; line && *line; line = next) {
			char *eq;
			next = strchr(line, '\n');
			if (next) *next++ = 0;
			eq = strchr(line, '=');
			if (line[0] == '#' || !eq) continue;
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
		console_print("gnss_band7: settings read from %s\n", (char *)conf_where());
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

	gnss_band_custom_get(&lo, &fs, &bw, &att, &g);
	console_print("GNSS_BAND 7 (custom): LO %s MHz, %s MS/s, RF bandwidth %s MHz, TX1 attenuation %s dB, RX gain %d dB\n",
		      gnss_fmt((double)lo / 1e6, 3), gnss_fmt((double)fs / 1e6, 3), gnss_fmt((double)bw / 1e6, 3),
		      gnss_fmt((double)att / 1000.0, 2), (long)g);
	console_print("  in use now: %s   start-up band: %d   saved in %s\n",
		      (char *)(gnss_band_get() == GNSS_BAND_CUSTOM ? "YES" : "no"), (long)boot_band, (char *)conf_where());
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
	if (gnss_radio_lo_hz(v[0], &lo)) {
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
	if (conf_save() != 0)
		console_print("gnss_band7: values set; not saved (%s)\n", (char *)conf_where());
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
	console_print("GNSS_BAND_BOOT: %d (saved in %s)\n", (long)boot_band, (char *)conf_where());
}

void set_gnss_band_boot(double *param, char param_no)
{
	if (param_no < 1 || param[0] < 0.0 || param[0] > (double)(GNSS_BAND_COUNT - 1)) {
		console_print("gnss_band_boot= needs 0 .. 7\n");
		return;
	}
	boot_band = (int)param[0];
	if (conf_save() != 0)
		console_print("gnss_band_boot: not saved (%s)\n", (char *)conf_where());
	get_gnss_band_boot(param, param_no);
}

/* ---- crpa5: gnss_rx_agc= and gnss_aj_avg= kept with the band 7 values ---------------- */
void gnss_settings_save(int rx_agc, int avg_milli)
{
	if (rx_agc >= 0) saved_agc = rx_agc;
	if (avg_milli >= 0) saved_avg = avg_milli;
	if (conf_save() != 0)
		console_print("  (setting active; not saved: %s)\n", (char *)conf_where());
	else
		console_print("  (saved in %s, used again at the next start)\n", (char *)conf_where());
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
