/* gnss_crpa - the AD9361 no-OS API on top of the Linux ad9361-phy driver.
 *
 * The kernel driver owns the AD9361 (SPI, calibrations, external band
 * control). Every call here is one sysfs attribute of iio:device<ad9361-phy>,
 * so the firmware's radio commands keep their meaning:
 *   ad9361_set_rx_lo_freq        out_altvoltage0_RX_LO_frequency
 *   ad9361_set_tx_lo_freq        out_altvoltage1_TX_LO_frequency
 *   ad9361_set_*_sampling_freq   in_/out_voltage_sampling_frequency
 *   ad9361_set_*_rf_bandwidth    in_/out_voltage_rf_bandwidth
 *   ad9361_set_rx_rf_gain        in_voltage<ch>_hardwaregain
 *   ad9361_set_rx_gain_control   in_voltage<ch>_gain_control_mode
 *   ad9361_set_tx_attenuation    out_voltage<ch>_hardwaregain (= -atten)
 *   ad9361_get_rx_rssi           in_voltage<ch>_rssi
 *   ad9361_set_*_fir_en_dis      in_/out_voltage_filter_fir_en
 *   ad9361_set_*_rf_port         in_voltage0_rf_port_select / out_...
 *   ad9361_do_calib_run          calib_mode (rf_dc_offs / tx_quad)
 *   ad9361_ensm_get_state        ensm_mode
 *   ad9361_spi_read              debugfs direct_reg_access
 * GNSS_CRPA_SYSFS=<dir> points the lookup at a fake tree (host tests).
 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <unistd.h>

#include "lx_platform.h"
#include "ad9361_api.h"
#include "axi_dac_core.h"

static char phy_dir[256];

static const char *sysroot(void)
{
	const char *r = getenv("GNSS_CRPA_SYSFS");
	return (r && *r) ? r : "/sys/bus/iio/devices";
}

int lx_iio_index(const char *name)
{
	DIR *d = opendir(sysroot());
	struct dirent *e;
	char path[512], buf[64];
	int idx = -1;

	if (!d)
		return -1;
	while ((e = readdir(d))) {
		FILE *f;
		if (strncmp(e->d_name, "iio:device", 10))
			continue;
		snprintf(path, sizeof(path), "%s/%s/name", sysroot(), e->d_name);
		f = fopen(path, "r");
		if (!f)
			continue;
		if (fgets(buf, sizeof(buf), f)) {
			buf[strcspn(buf, "\r\n")] = 0;
			if (!strcmp(buf, name))
				idx = atoi(e->d_name + 10);
		}
		fclose(f);
		if (idx >= 0)
			break;
	}
	closedir(d);
	return idx;
}

const char *lx_iio_dir(const char *name)
{
	static char dirs[4][256];
	static int next;
	int idx = lx_iio_index(name);
	char *p;

	if (idx < 0)
		return NULL;
	p = dirs[next++ & 3];
	snprintf(p, sizeof(dirs[0]), "%s/iio:device%d", sysroot(), idx);
	return p;
}

int lx_sysfs_read(const char *dir, const char *attr, char *buf, size_t len)
{
	char path[512];
	int fd, n;

	snprintf(path, sizeof(path), "%s/%s", dir, attr);
	fd = open(path, O_RDONLY);
	if (fd < 0)
		return -errno;
	n = read(fd, buf, len - 1);
	close(fd);
	if (n < 0)
		return -errno;
	buf[n] = 0;
	buf[strcspn(buf, "\r\n")] = 0;
	return 0;
}

int lx_sysfs_write(const char *dir, const char *attr, const char *val)
{
	char path[512];
	int fd, n, len = (int)strlen(val);

	snprintf(path, sizeof(path), "%s/%s", dir, attr);
	fd = open(path, O_WRONLY | O_TRUNC);
	if (fd < 0)
		return -errno;
	n = write(fd, val, len);
	if (n < 0)
		n = -errno;
	close(fd);
	return (n == len) ? 0 : (n < 0 ? n : -EIO);
}

static int rd(const char *attr, char *buf, size_t len)
{
	return lx_sysfs_read(phy_dir, attr, buf, len);
}

static int wr(const char *attr, const char *fmt, ...) __attribute__((format(__printf__, 2, 3)));
static int wr(const char *attr, const char *fmt, ...)
{
	char val[64];
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(val, sizeof(val), fmt, ap);
	va_end(ap);
	return lx_sysfs_write(phy_dir, attr, val);
}

static int rd_u64(const char *attr, uint64_t *v)
{
	char b[64];
	int r = rd(attr, b, sizeof(b));

	if (r)
		return r;
	*v = strtoull(b, NULL, 10);
	return 0;
}

static int rd_double(const char *attr, double *v)
{
	char b[64];
	int r = rd(attr, b, sizeof(b));

	if (r)
		return r;
	*v = strtod(b, NULL);       /* "71.000000 dB" -> 71.0 */
	return 0;
}

static char attr_buf[64];
static const char *chattr(const char *pre, unsigned ch, const char *post)
{
	snprintf(attr_buf, sizeof(attr_buf), "%s%u_%s", pre, ch, post);
	return attr_buf;
}

/* ---- DAC core description (the kernel owns it; nothing is reset) ---- */
static struct axi_dac tx_dac;
static struct axi_adc rx_adc;

static void dac_clock_update(void)
{
	uint32_t freq = lx_in32(LX_DAC_BASE + 0x54);    /* AXI_DAC_REG_CLK_FREQ  */
	uint32_t ratio = lx_in32(LX_DAC_BASE + 0x58);   /* AXI_DAC_REG_CLK_RATIO */

	tx_dac.clock_hz = (((uint64_t)freq * ratio) * 390625ULL) >> 8;
}

/* E310 external band-select lines (zynq-ant.dtsi adi,band-ctl-0..7):
 * gpio 86..93 = RX1 H/L, TX1 H/L, RX2 H/L, TX2 H/L. The kernel already drives
 * them from the LO frequency; the firmware's own writes are applied too. */
static struct no_os_gpio_desc band_gpio[8] = {
	{86}, {87}, {88}, {89}, {90}, {91}, {92}, {93}
};
static struct ad9361_rf_phy phy;

int lx_ad9361_open(struct ad9361_rf_phy **out)
{
	const char *d = lx_iio_dir("ad9361-phy");

	if (!d) {
		fprintf(stderr, "gnss_crpa: no ad9361-phy IIO device (did the AD9361 driver probe?)\n");
		return -1;
	}
	snprintf(phy_dir, sizeof(phy_dir), "%s", d);

	tx_dac.name = "cf-ad9361-dds-core-lpc";
	tx_dac.base = LX_DAC_BASE;
	tx_dac.num_channels = 4;          /* 2R2T: TX1 I/Q, TX2 I/Q */
	tx_dac.channels = NULL;
	dac_clock_update();
	rx_adc.name = "cf-ad9361-lpc";
	rx_adc.base = LX_ADC_BASE;
	rx_adc.num_channels = 4;

	phy.tx_dac = &tx_dac;
	phy.rx_adc = &rx_adc;
	phy.gpio_desc_rx1_ctrl_h = &band_gpio[0];
	phy.gpio_desc_rx1_ctrl_l = &band_gpio[1];
	phy.gpio_desc_tx1_ctrl_h = &band_gpio[2];
	phy.gpio_desc_tx1_ctrl_l = &band_gpio[3];
	phy.gpio_desc_rx2_ctrl_h = &band_gpio[4];
	phy.gpio_desc_rx2_ctrl_l = &band_gpio[5];
	phy.gpio_desc_tx2_ctrl_h = &band_gpio[6];
	phy.gpio_desc_tx2_ctrl_l = &band_gpio[7];
	*out = &phy;
	return 0;
}

/* ---- RX ---------------------------------------------------------------- */
int32_t ad9361_set_rx_rf_gain(struct ad9361_rf_phy *p, uint8_t ch, int32_t gain_db)
{
	(void)p;
	return wr(chattr("in_voltage", ch, "hardwaregain"), "%ld", (long)gain_db);
}

int32_t ad9361_get_rx_rf_gain(struct ad9361_rf_phy *p, uint8_t ch, int32_t *gain_db)
{
	double v;
	int r;

	(void)p;
	r = rd_double(chattr("in_voltage", ch, "hardwaregain"), &v);
	if (!r)
		*gain_db = (int32_t)(v >= 0 ? v + 0.5 : v - 0.5);
	return r;
}

int32_t ad9361_set_rx_rf_bandwidth(struct ad9361_rf_phy *p, uint32_t bw)
{
	(void)p;
	return wr("in_voltage_rf_bandwidth", "%" PRIu32, bw);
}

int32_t ad9361_get_rx_rf_bandwidth(struct ad9361_rf_phy *p, uint32_t *bw)
{
	uint64_t v;
	int r;

	(void)p;
	r = rd_u64("in_voltage_rf_bandwidth", &v);
	if (!r)
		*bw = (uint32_t)v;
	return r;
}

int32_t ad9361_set_rx_sampling_freq(struct ad9361_rf_phy *p, uint32_t fs)
{
	int r;

	(void)p;
	r = wr("in_voltage_sampling_frequency", "%" PRIu32, fs);
	dac_clock_update();
	return r;
}

int32_t ad9361_get_rx_sampling_freq(struct ad9361_rf_phy *p, uint32_t *fs)
{
	uint64_t v;
	int r;

	(void)p;
	r = rd_u64("in_voltage_sampling_frequency", &v);
	if (!r)
		*fs = (uint32_t)v;
	return r;
}

int32_t ad9361_set_rx_lo_freq(struct ad9361_rf_phy *p, uint64_t f)
{
	(void)p;
	return wr("out_altvoltage0_RX_LO_frequency", "%" PRIu64, f);
}

int32_t ad9361_get_rx_lo_freq(struct ad9361_rf_phy *p, uint64_t *f)
{
	(void)p;
	return rd_u64("out_altvoltage0_RX_LO_frequency", f);
}

/* no-OS reports RSSI as symbol / multiplier dB with a multiplier of 100. */
int32_t ad9361_get_rx_rssi(struct ad9361_rf_phy *p, uint8_t ch, struct rf_rssi *rssi)
{
	double v;
	int r;

	(void)p;
	memset(rssi, 0, sizeof(*rssi));
	r = rd_double(chattr("in_voltage", ch, "rssi"), &v);
	if (r)
		return r;
	rssi->ant = ch + 1U;
	rssi->multiplier = 100;
	rssi->symbol = (uint32_t)(v * 100.0 + 0.5);
	rssi->preamble = rssi->symbol;
	return 0;
}

static const char *const gc_names[] = { "manual", "fast_attack", "slow_attack", "hybrid" };

int32_t ad9361_set_rx_gain_control_mode(struct ad9361_rf_phy *p, uint8_t ch, uint8_t m)
{
	(void)p;
	if (m > 3)
		return -EINVAL;
	return wr(chattr("in_voltage", ch, "gain_control_mode"), "%s", gc_names[m]);
}

int32_t ad9361_get_rx_gain_control_mode(struct ad9361_rf_phy *p, uint8_t ch, uint8_t *m)
{
	char b[32];
	int r, i;

	(void)p;
	r = rd(chattr("in_voltage", ch, "gain_control_mode"), b, sizeof(b));
	if (r)
		return r;
	for (i = 0; i < 4; i++)
		if (!strcmp(b, gc_names[i])) {
			*m = (uint8_t)i;
			return 0;
		}
	return -EINVAL;
}

int32_t ad9361_set_rx_fir_en_dis(struct ad9361_rf_phy *p, uint8_t en)
{
	(void)p;
	return wr("in_voltage_filter_fir_en", "%d", en ? 1 : 0);
}

int32_t ad9361_get_rx_fir_en_dis(struct ad9361_rf_phy *p, uint8_t *en)
{
	uint64_t v;
	int r;

	(void)p;
	r = rd_u64("in_voltage_filter_fir_en", &v);
	if (!r)
		*en = v ? 1 : 0;
	return r;
}

static const char *const rx_ports[] = {
	"A_BALANCED", "B_BALANCED", "C_BALANCED", "A_N", "A_P", "B_N", "B_P",
	"C_N", "C_P", "TX_MONITOR1", "TX_MONITOR2", "TX_MONITOR1_2"
};

int32_t ad9361_set_rx_rf_port_input(struct ad9361_rf_phy *p, uint32_t mode)
{
	(void)p;
	if (mode >= sizeof(rx_ports) / sizeof(rx_ports[0]))
		return -EINVAL;
	return wr("in_voltage0_rf_port_select", "%s", rx_ports[mode]);
}

int32_t ad9361_get_rx_rf_port_input(struct ad9361_rf_phy *p, uint32_t *mode)
{
	char b[32];
	unsigned i;
	int r;

	(void)p;
	r = rd("in_voltage0_rf_port_select", b, sizeof(b));
	if (r)
		return r;
	for (i = 0; i < sizeof(rx_ports) / sizeof(rx_ports[0]); i++)
		if (!strcmp(b, rx_ports[i])) {
			*mode = i;
			return 0;
		}
	return -EINVAL;
}

/* ---- TX ---------------------------------------------------------------- */
int32_t ad9361_set_tx_attenuation(struct ad9361_rf_phy *p, uint8_t ch, uint32_t mdb)
{
	(void)p;
	return wr(chattr("out_voltage", ch, "hardwaregain"), "-%" PRIu32 ".%03" PRIu32,
		  mdb / 1000U, mdb % 1000U);
}

int32_t ad9361_get_tx_attenuation(struct ad9361_rf_phy *p, uint8_t ch, uint32_t *mdb)
{
	double v;
	int r;

	(void)p;
	r = rd_double(chattr("out_voltage", ch, "hardwaregain"), &v);
	if (!r)
		*mdb = (uint32_t)(-v * 1000.0 + 0.5);
	return r;
}

int32_t ad9361_set_tx_rf_bandwidth(struct ad9361_rf_phy *p, uint32_t bw)
{
	(void)p;
	return wr("out_voltage_rf_bandwidth", "%" PRIu32, bw);
}

int32_t ad9361_get_tx_rf_bandwidth(struct ad9361_rf_phy *p, uint32_t *bw)
{
	uint64_t v;
	int r;

	(void)p;
	r = rd_u64("out_voltage_rf_bandwidth", &v);
	if (!r)
		*bw = (uint32_t)v;
	return r;
}

int32_t ad9361_set_tx_sampling_freq(struct ad9361_rf_phy *p, uint32_t fs)
{
	int r;

	(void)p;
	r = wr("out_voltage_sampling_frequency", "%" PRIu32, fs);
	dac_clock_update();
	return r;
}

int32_t ad9361_get_tx_sampling_freq(struct ad9361_rf_phy *p, uint32_t *fs)
{
	uint64_t v;
	int r;

	(void)p;
	r = rd_u64("out_voltage_sampling_frequency", &v);
	if (!r)
		*fs = (uint32_t)v;
	return r;
}

int32_t ad9361_set_tx_lo_freq(struct ad9361_rf_phy *p, uint64_t f)
{
	(void)p;
	return wr("out_altvoltage1_TX_LO_frequency", "%" PRIu64, f);
}

int32_t ad9361_get_tx_lo_freq(struct ad9361_rf_phy *p, uint64_t *f)
{
	(void)p;
	return rd_u64("out_altvoltage1_TX_LO_frequency", f);
}

int32_t ad9361_set_tx_fir_en_dis(struct ad9361_rf_phy *p, uint8_t en)
{
	(void)p;
	return wr("out_voltage_filter_fir_en", "%d", en ? 1 : 0);
}

int32_t ad9361_get_tx_fir_en_dis(struct ad9361_rf_phy *p, uint8_t *en)
{
	uint64_t v;
	int r;

	(void)p;
	r = rd_u64("out_voltage_filter_fir_en", &v);
	if (!r)
		*en = v ? 1 : 0;
	return r;
}

int32_t ad9361_set_tx_rf_port_output(struct ad9361_rf_phy *p, uint32_t mode)
{
	(void)p;
	if (mode > TXB)
		return -EINVAL;
	return wr("out_voltage0_rf_port_select", "%s", mode == TXA ? "A" : "B");
}

int32_t ad9361_get_tx_rf_port_output(struct ad9361_rf_phy *p, uint32_t *mode)
{
	char b[16];
	int r;

	(void)p;
	r = rd("out_voltage0_rf_port_select", b, sizeof(b));
	if (r)
		return r;
	if (!strcmp(b, "A"))
		*mode = TXA;
	else if (!strcmp(b, "B"))
		*mode = TXB;
	else
		return -EINVAL;
	return 0;
}

/* ---- misc --------------------------------------------------------------- */
int32_t ad9361_do_calib_run(struct ad9361_rf_phy *p, uint32_t cal, int32_t arg)
{
	(void)p;
	if (cal == RFDC_CAL)
		return wr("calib_mode", "rf_dc_offs");
	if (cal == TX_QUAD_CAL) {
		if (arg >= 0)
			return wr("calib_mode", "tx_quad %ld", (long)arg);
		return wr("calib_mode", "tx_quad");
	}
	return -EINVAL;
}

uint8_t ad9361_ensm_get_state(struct ad9361_rf_phy *p)
{
	char b[32];

	(void)p;
	if (rd("ensm_mode", b, sizeof(b)))
		return ENSM_STATE_INVALID;
	if (!strcmp(b, "fdd"))   return ENSM_STATE_FDD;
	if (!strcmp(b, "alert")) return ENSM_STATE_ALERT;
	if (!strcmp(b, "sleep")) return ENSM_STATE_SLEEP;
	if (!strcmp(b, "tx"))    return ENSM_STATE_TX;
	if (!strcmp(b, "rx"))    return ENSM_STATE_RX;
	return ENSM_STATE_INVALID;
}

/* "register?" command: read one AD9361 register through the driver's
 * debugfs direct_reg_access (debugfs is mounted here if it is not yet). */
int32_t ad9361_spi_read(void *spi, uint32_t reg)
{
	char dir[128], b[32];
	int idx = lx_iio_index("ad9361-phy");
	struct stat st;

	(void)spi;
	if (idx < 0)
		return -ENODEV;
	if (stat("/sys/kernel/debug/iio", &st))
		(void)mount("debugfs", "/sys/kernel/debug", "debugfs", 0, NULL);
	snprintf(dir, sizeof(dir), "/sys/kernel/debug/iio/iio:device%d", idx);
	snprintf(b, sizeof(b), "0x%" PRIx32, reg);
	if (lx_sysfs_write(dir, "direct_reg_access", b))
		return -EIO;
	if (lx_sysfs_read(dir, "direct_reg_access", b, sizeof(b)))
		return -EIO;
	return (int32_t)strtol(b, NULL, 0);
}
