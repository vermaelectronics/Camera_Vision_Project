/* gnss_log.c -- see gnss_log.h */
#include "gnss_log.h"
#include "console.h"
#include "command.h"
#include "gnss_aj.h"
#include "gnss_passthrough.h"
#include "xtime_l.h"
#include <stdint.h>

#ifdef GNSS_LOG_SD
#include "ff.h"
#endif

#define LOG_BUF       (48U * 1024U)
#define LOG_DEFAULT_S 60U

static char     logbuf[LOG_BUF];
static unsigned loglen, log_overflow;
static unsigned period_s = LOG_DEFAULT_S;
static XTime    t_start, t_next;
static unsigned saved, failed;
static char     last_name[40];
static int      sd_ok;

/* wall clock: seconds since 2000-01-01 at the moment gnss_time= was typed, and the uptime then */
static int      clk_set;
static uint32_t clk_base_s;
static uint32_t clk_up_at_set;

static uint32_t uptime_s(void)
{
	XTime now;
	XTime_GetTime(&now);
	return (uint32_t)((now - t_start) / COUNTS_PER_SECOND);
}

/* days from 1970-01-01 for a civil date (Howard Hinnant's algorithm) */
static int32_t days_from_civil(int y, int m, int d)
{
	int32_t era, yoe, doy, doe;
	y -= (m <= 2);
	era = (y >= 0 ? y : y - 399) / 400;
	yoe = y - era * 400;
	doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
	doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
	return era * 146097 + doe - 719468;
}

static void civil_from_days(int32_t z, int *y, int *m, int *d)
{
	int32_t era, doe, yoe, doy, mp;
	z += 719468;
	era = (z >= 0 ? z : z - 146096) / 146097;
	doe = z - era * 146097;
	yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
	*y = (int)(yoe + era * 400);
	doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
	mp = (5 * doy + 2) / 153;
	*d = (int)(doy - (153 * mp + 2) / 5 + 1);
	*m = (int)(mp + (mp < 10 ? 3 : -9));
	if(*m <= 2) (*y)++;
}

void gnss_log_set_clock(int y, int mo, int d, int h, int mi, int s)
{
	int32_t days = days_from_civil(y, mo, d);
	clk_base_s = (uint32_t)((days - days_from_civil(2000, 1, 1)) * 86400L + h * 3600L + mi * 60L + s);
	clk_up_at_set = uptime_s();
	clk_set = 1;
}

/* current wall clock (needs clk_set) */
static void wall(int *y, int *mo, int *d, int *h, int *mi, int *s)
{
	uint32_t t = clk_base_s + (uptime_s() - clk_up_at_set);
	int32_t days = (int32_t)(t / 86400U) + days_from_civil(2000, 1, 1);
	uint32_t r = t % 86400U;
	civil_from_days(days, y, mo, d);
	*h = (int)(r / 3600U); *mi = (int)((r % 3600U) / 60U); *s = (int)(r % 60U);
}

static void tee(char c)
{
	if(loglen < LOG_BUF - 1U) logbuf[loglen++] = c;
	else log_overflow = 1;
}

static void two(char *p, int v) { p[0] = (char)('0' + (v / 10) % 10); p[1] = (char)('0' + v % 10); }

#ifdef GNSS_LOG_SD
/* "0:/LOG/20261105/143000.TXT" or "0:/LOG/UPTIME/T000123.TXT" */
static void make_path(char *dir, char *path)
{
	int y, mo, d, h, mi, s, i = 0;
	const char *a = "0:/LOG/";
	while(*a) { dir[i] = *a; i++; a++; }
	dir[i] = 0;
	if(clk_set) {
		wall(&y, &mo, &d, &h, &mi, &s);
		two(&dir[i], y / 100); two(&dir[i + 2], y % 100); two(&dir[i + 4], mo); two(&dir[i + 6], d);
		dir[i + 8] = 0;
		for(i = 0; dir[i]; i++) path[i] = dir[i];
		path[i++] = '/';
		two(&path[i], h); two(&path[i + 2], mi); two(&path[i + 4], s);
		path[i + 6] = '.'; path[i + 7] = 'T'; path[i + 8] = 'X'; path[i + 9] = 'T'; path[i + 10] = 0;
	} else {
		uint32_t u = uptime_s();
		const char *b = "UPTIME";
		int k;
		for(k = 0; b[k]; k++) dir[i + k] = b[k];
		dir[i + k] = 0;
		for(i = 0; dir[i]; i++) path[i] = dir[i];
		path[i++] = '/'; path[i++] = 'T';
		path[i + 5] = (char)('0' + u % 10); u /= 10; path[i + 4] = (char)('0' + u % 10); u /= 10;
		path[i + 3] = (char)('0' + u % 10); u /= 10; path[i + 2] = (char)('0' + u % 10); u /= 10;
		path[i + 1] = (char)('0' + u % 10); u /= 10; path[i + 0] = (char)('0' + u % 10);
		path[i + 6] = '.'; path[i + 7] = 'T'; path[i + 8] = 'X'; path[i + 9] = 'T'; path[i + 10] = 0;
	}
}

static FATFS fs;
static int write_sd(void)
{
	char dir[40], path[48];
	FIL f;
	UINT bw = 0;
	int i;
	if(!sd_ok) return -1;
	make_path(dir, path);
	(void)f_mkdir("0:/LOG");
	(void)f_mkdir(dir);
	if(f_open(&f, path, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) return -2;
	if(f_write(&f, logbuf, loglen, &bw) != FR_OK || bw != loglen) { (void)f_close(&f); return -3; }
	(void)f_close(&f);
	for(i = 0; i < 39 && path[i]; i++) last_name[i] = path[i];
	last_name[i] = 0;
	return 0;
}
#endif

void gnss_log_init(void)
{
	XTime_GetTime(&t_start);
	t_next = t_start + (XTime)period_s * COUNTS_PER_SECOND;
#ifdef GNSS_LOG_SD
	if(f_mount(&fs, "0:/", 1) == FR_OK) {
		sd_ok = 1;
		console_print("GNSS_LOG: SD card mounted; a snapshot is saved every %d s under 0:/LOG/ (gnss_log? / gnss_log=N / gnss_time=)\n", (long)period_s);
	} else {
		console_print("GNSS_LOG: no SD card could be mounted: snapshots go to the console (gnss_log? shows it)\n");
	}
#else
	console_print("GNSS_LOG: snapshots every %d s on the console (===== GNSS_LOG BEGIN/END ====); build with --defines GNSS_LOG_SD to save files\n", (long)period_s);
#endif
}

void gnss_log_set_period(unsigned seconds)
{
	XTime now;
	XTime_GetTime(&now);
	period_s = seconds;
	t_next = now + (XTime)seconds * COUNTS_PER_SECOND;
}

void gnss_log_now(void)
{
	double d = 0.0;
	int y, mo, dd, h, mi, s;
	int to_file = 0;
#ifdef GNSS_LOG_SD
	to_file = sd_ok;
#endif
	loglen = 0; log_overflow = 0;
	console_set_tee(tee, to_file);        /* with a file: capture only, nothing on the UART */
	console_print("===== GNSS_LOG BEGIN =====\n");
	/* A fixed, self-describing header: one key=value per line, so the file can be read by a person or a program
	 * (and by the assistant that will change the code from it). Keep the key names stable. */
	console_print("# GNSS_LOG format 1: lines 'key=value', then sections '----- name -----'\n");
	console_print("fw=%s\n", (char*)GNSS_AJ_VERSION_STR);
	console_print("core_version=0x%x\n", (long)gnss_pt_read(GNSS_PT_REG_VERSION));
	console_print("aj_info=0x%x\n", (long)gnss_pt_read(GNSS_PT_REG_AJ_INFO));
	console_print("band=%d\n", (long)gnss_band_get());
	console_print("uptime_s=%d\n", (long)uptime_s());
	console_print("period_s=%d\n", (long)period_s);
	if(clk_set) {
		wall(&y, &mo, &dd, &h, &mi, &s);
		console_print("time=%d-%d-%d %d:%d:%d\n", (long)y, (long)mo, (long)dd, (long)h, (long)mi, (long)s);
	} else {
		console_print("time=not_set (uptime only)\n");
	}
	console_print("\n----- gnss_status? -----\n");           get_gnss_status(&d, 0);
	console_print("\n----- gnss_tx? -----\n");               get_gnss_tx(&d, 0);
	console_print("\n----- gnss_band? -----\n");             get_gnss_band(&d, 0);
	console_print("\n----- gnss_rx_gain? -----\n");          get_gnss_rx_gain(&d, 0);
	console_print("\n----- gnss_smi? -----\n");              get_gnss_smi(&d, 0);
	console_print("\n----- tx1_attenuation? -----\n");       get_tx1_attenuation(&d, 0);
	console_print("\n----- radio? -----\n");                 get_radio(&d, 0);
	console_print("\n----- gnss_aj? -----\n");               get_gnss_aj(&d, 0);
	console_print("\n----- AJCSV (columns in the header line, one row) -----\n"); gnss_aj_csv_once();
	console_print("\n----- gnss_report? -----\n");           get_gnss_report(&d, 0);
	if(log_overflow) console_print("\n(snapshot longer than the %d byte buffer: cut)\n", (long)LOG_BUF);
	console_print("===== GNSS_LOG END =====\n");
	console_set_tee(0, 0);
	logbuf[loglen] = 0;
#ifdef GNSS_LOG_SD
	if(sd_ok) {
		int r = write_sd();
		if(r == 0) { saved++; console_print("[LOG] saved %s (%d bytes)\n", last_name, (long)loglen); }
		else       { failed++; console_print("[LOG] SD write failed (%d)\n", (long)r); }
	} else
#endif
	{
		saved++;
	}
}

void gnss_log_poll(void)
{
	XTime now;
	if(period_s == 0U) return;
	XTime_GetTime(&now);
	if(now < t_next) return;
	t_next = now + (XTime)period_s * COUNTS_PER_SECOND;
	gnss_log_now();
}

void gnss_log_print_state(void)
{
	console_print("GNSS_LOG: period %d s%s, %d snapshot(s) done, %d write failure(s)\n", (long)period_s,
		      (char*)(period_s ? "" : " (off)"), (long)saved, (long)failed);
#ifdef GNSS_LOG_SD
	console_print("  target  : %s%s\n", (char*)(sd_ok ? "SD card, 0:/LOG/<YYYYMMDD>/<HHMMSS>.TXT" : "console (no SD card mounted)"),
		      (char*)(last_name[0] ? "" : ""));
	if(last_name[0]) console_print("  last    : %s\n", last_name);
#else
	console_print("  target  : console (this build has no SD logging: build_all.sh --defines GNSS_LOG_SD)\n");
#endif
	console_print("  clock   : %s\n", (char*)(clk_set ? "set by gnss_time=" : "not set: uptime only (gnss_time=Y M D h m s)"));
}
