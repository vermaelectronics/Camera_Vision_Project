/******************************************************************************
 *  tb_engine.c  --  host test of the CRPA anti-jam ENGINE (gnss_aj.c) with the
 *  real register driver (gnss_passthrough.c) and one algorithm's real solver
 *  (gnss_aj_math.c), everything else simulated:
 *    - the RX DMA: every capture is generated on the spot (receiver noise on
 *      both channels, plus a jammer seen by RX2 through a complex gain that
 *      can turn with time);
 *    - gnss_passthrough's registers: a model of the v7.0 map (shadow
 *      registers, AJ_COMMIT copies them to the committed set, VERSION,
 *      AJ_INFO, STATUS);
 *    - the FPGA filter: a double-precision model of smi_pi / stap_pi applied
 *      with the COMMITTED words to the jammer and the noise of each new
 *      capture separately, so the test sees the residual the core would give;
 *    - time (1 count = 1 us; each solve takes SOLVE_US, as the ARM does), the
 *      console, the RX gain loop, the AD9361.
 *
 *  Timeline: 0-1.5 s clear (noise floor and g2 learned), 1.5-4 s jammer
 *  40 dB, 4-6 s the jammer moves (its phase at RX2 turns 150 deg/s), 6-8 s
 *  no jammer. Checks: JAMMER ON / NULLED / OFF on the console, the residual
 *  of the committed weights while the jammer is static and while it moves,
 *  bypass committed once it is gone, crpa_report? printed without trouble.
 *
 *  Run by run_engine_test.sh (once per algorithm). Prints ENGINE_TB: PASS.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <math.h>
#include "gnss_aj.h"
#include "gnss_aj_math.h"
#include "crpa_build.h"
#include "gnss_passthrough.h"
#include "gnss_smi_agc.h"
#include "gnss_txdma.h"
#include "ad9361_api.h"
#include "axi_dmac.h"
#include "console.h"
#include "xtime_l.h"

uint32_t tb_capbuf[2 * AJ_NSAMP];
static XTime tb_now;

/* ---- the scenario ----------------------------------------------------------------- */
#define T_JAM_ON    1500000ULL
#define T_MOVE      4000000ULL
#define T_JAM_OFF   6000000ULL
#define T_END       8000000ULL
static const double NOISE_SD = 3.0, JN_DB = 40.0, TURN_DEG_S = 150.0;
/* the ARM's time for one solve (capture -> commit), so the committed weights are
 * as old as on the board and the weight prediction has something to do */
#define SOLVE_US    20000ULL

static unsigned long long rng_state = 88172645463325252ULL;
static double urand(void) { rng_state ^= rng_state << 13; rng_state ^= rng_state >> 7; rng_state ^= rng_state << 17; return (rng_state >> 11) * (1.0 / 9007199254740992.0); }
static double gauss(void) { double u = urand(), v = urand(); if(u < 1e-300) u = 1e-300; return sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v); }

/* the components of the last capture, for the filter model */
static double j1r[AJ_NSAMP], j1i[AJ_NSAMP], j2r[AJ_NSAMP], j2i[AJ_NSAMP];
static double n1r[AJ_NSAMP], n1i[AJ_NSAMP], n2r[AJ_NSAMP], n2i[AJ_NSAMP];
static int    cap_jam;

static int16_t q12(double v) { long r = lround(v); if(r > 2047) r = 2047; if(r < -2048) r = -2048; return (int16_t)r; }

static int jam_at(XTime t) { return t >= T_JAM_ON && t < T_JAM_OFF; }
static double phase_at(XTime t)          /* the jammer's phase at RX2 relative to RX1 */
{
	double ph = 1.1;
	if(t > T_MOVE && t < T_JAM_OFF) ph += TURN_DEG_S * M_PI / 180.0 * (double)(t - T_MOVE) / 1e6;
	return ph;
}

static void make_capture(void)
{
	int t;
	double ja = NOISE_SD * sqrt(2.0) * pow(10.0, JN_DB / 20.0) / sqrt(2.0);
	double ph = phase_at(tb_now);
	cap_jam = jam_at(tb_now);
	for(t = 0; t < AJ_NSAMP; t++) {
		double jr = cap_jam ? gauss() * ja : 0.0, ji = cap_jam ? gauss() * ja : 0.0;
		j1r[t] = jr; j1i[t] = ji;
		j2r[t] = cos(ph) * jr - sin(ph) * ji; j2i[t] = sin(ph) * jr + cos(ph) * ji;
		n1r[t] = gauss() * NOISE_SD; n1i[t] = gauss() * NOISE_SD;
		n2r[t] = gauss() * NOISE_SD; n2i[t] = gauss() * NOISE_SD;
		tb_capbuf[2 * t]     = ((uint32_t)(uint16_t)(q12(j1i[t] + n1i[t]) & 0xFFF) << 16) | (uint16_t)(q12(j1r[t] + n1r[t]) & 0xFFF);
		tb_capbuf[2 * t + 1] = ((uint32_t)(uint16_t)(q12(j2i[t] + n2i[t]) & 0xFFF) << 16) | (uint16_t)(q12(j2r[t] + n2r[t]) & 0xFFF);
	}
}

/* ---- gnss_passthrough register model (via no_os_axi_io) ------------------------------ */
static uint32_t reg[0x800];           /* 8 KB map, word index */
static uint32_t w_act[64], c_act[8], cfg_act = 1, nrm_act = 0x2000, gain_act = 0x4000, commits;
static uint32_t n_bypass_commits, n_weight_commits;
static XTime    prev_commit_t;
static void eval_hold(XTime t_from, XTime t_to);

int32_t no_os_axi_io_write(uint32_t base, uint32_t off, uint32_t v)
{
	(void)base;
	if(off == GNSS_PT_REG_AJ_COMMIT) {
		int k;
		eval_hold(prev_commit_t, tb_now);       /* what the outgoing set did while it ran */
		prev_commit_t = tb_now;
		for(k = 0; k < AJ_NSPT; k++) w_act[k] = reg[(0x1000 >> 2) + k];
		for(k = 0; k < AJ_NW - AJ_NSPT; k++) c_act[k] = reg[(0x1100 >> 2) + k];
		cfg_act = reg[0x188 >> 2]; nrm_act = reg[0x18C >> 2]; gain_act = reg[0x190 >> 2];
		commits++;
		if(cfg_act & 1) n_bypass_commits++; else n_weight_commits++;
		return 0;
	}
	if(off < sizeof(reg)) reg[off >> 2] = v;
	return 0;
}

int32_t no_os_axi_io_read(uint32_t base, uint32_t off, uint32_t *v)
{
	(void)base;
	switch(off) {
	case GNSS_PT_REG_ID:            *v = GNSS_PT_EXPECTED_ID; break;
	case GNSS_PT_REG_VERSION:       *v = 0x00070000U; break;
	case GNSS_PT_REG_AJ_INFO:       *v = CRPA_AJ_INFO; break;
	case GNSS_PT_REG_AJ_COMMIT:     *v = commits; break;
	case GNSS_PT_REG_AJ_CFG_ACTIVE: *v = cfg_act; break;
	case GNSS_PT_REG_AJ_DIMS:       *v = ((uint32_t)(AJ_NW - AJ_NSPT) << 16) | ((uint32_t)AJ_NSPT << 8) |
	                                     (uint32_t)((AJ_NW - AJ_NSPT) == 2 ? 64 : AJ_NSPT); break;
	case GNSS_PT_REG_STATUS: {
		uint32_t c = reg[GNSS_PT_REG_CONTROL >> 2];
		*v = GNSS_PT_ST_DAC_EN_I0 | GNSS_PT_ST_DAC_EN_Q0 | GNSS_PT_ST_PASS_EN_SYNCED |
		     ((c & GNSS_PT_CTRL_SMI_EN) ? GNSS_PT_ST_SMI_EN_SYNCED : 0);
		break; }
	default: *v = (off < sizeof(reg)) ? reg[off >> 2] : 0; break;
	}
	return 0;
}
void no_os_mdelay(uint32_t ms) { tb_now += 1000ULL * ms; }

/* ---- the filter model with the committed words ------------------------------------------ */
static double cw_r(int k) { uint32_t w = k < AJ_NSPT ? w_act[k] : c_act[k - AJ_NSPT]; return (double)(int16_t)(w & 0xFFFF); }
static double cw_i(int k) { uint32_t w = k < AJ_NSPT ? w_act[k] : c_act[k - AJ_NSPT]; return (double)(int16_t)(w >> 16); }

static double model_power_n(int nn, const double *ar, const double *ai, const double *br, const double *bi)
{
	const int nimg = (AJ_NW - AJ_NSPT) / 2, h = (nimg - 1) / 2;
	const double ws = (double)(1 << ((cfg_act >> 12) & 7)), sc = 1.0 / (4096.0 * ws);
	const double g2 = (double)(int16_t)(nrm_act & 0xFFFF) / 8192.0;
	double p = 0.0;
	int n, k, c, cnt = 0;
	for(n = AJ_NT + 2; n < nn; n++) {
		double yr = ar[n - AJ_REF], yi = ai[n - AJ_REF];
		if(!(cfg_act & 1)) {
			double sr = 0.0, si = 0.0;
			for(k = 0; k < AJ_NT; k++) {
				double xr, xi;
				if(k != AJ_REF) { xr = ar[n - k]; xi = ai[n - k];
					sr += cw_r(k) * xr - cw_i(k) * xi; si += cw_r(k) * xi + cw_i(k) * xr; }
				xr = g2 * br[n - k]; xi = g2 * bi[n - k];
				sr += cw_r(AJ_NT + k) * xr - cw_i(AJ_NT + k) * xi; si += cw_r(AJ_NT + k) * xi + cw_i(AJ_NT + k) * xr;
			}
			for(c = 0; c < nimg; c++) {
				int a = AJ_REF - h + c;
				double xr = ar[n - a], xi = -ai[n - a];
				int k1 = AJ_NSPT + c, k2 = AJ_NSPT + nimg + c;
				sr += cw_r(k1) * xr - cw_i(k1) * xi; si += cw_r(k1) * xi + cw_i(k1) * xr;
				xr = g2 * br[n - a]; xi = -g2 * bi[n - a];
				sr += cw_r(k2) * xr - cw_i(k2) * xi; si += cw_r(k2) * xi + cw_i(k2) * xr;
			}
			yr -= sc * sr; yi -= sc * si;
		}
		p += yr * yr + yi * yi; cnt++;
	}
	return p / cnt;
}

/* The residual of the committed set over its hold interval, measured on fresh
 * samples at the middle of the interval (the jammer as it was then). */
#define NEVAL 2048
static double e1r[NEVAL], e1i[NEVAL], e2r[NEVAL], e2i[NEVAL], z1r[NEVAL], z1i[NEVAL], z2r[NEVAL], z2i[NEVAL];
static int    n_static, n_move;
static double res_static, res_move, worst_static = -99.0, worst_move = -99.0;
static double model_power_n(int nn, const double *ar, const double *ai, const double *br, const double *bi);
static void eval_hold(XTime t0, XTime t1)
{
	XTime tm = (t0 + t1) / 2;
	double ja = NOISE_SD * pow(10.0, JN_DB / 20.0), ph = phase_at(tm), r, pj, pn;
	int t;
	if((cfg_act & 1) || !jam_at(t0) || !jam_at(t1) || tm < T_JAM_ON + 300000ULL) return;
	for(t = 0; t < NEVAL; t++) {
		double jr = gauss() * ja, ji = gauss() * ja;
		e1r[t] = jr; e1i[t] = ji;
		e2r[t] = cos(ph) * jr - sin(ph) * ji; e2i[t] = sin(ph) * jr + cos(ph) * ji;
		z1r[t] = gauss() * NOISE_SD; z1i[t] = gauss() * NOISE_SD;
		z2r[t] = gauss() * NOISE_SD; z2i[t] = gauss() * NOISE_SD;
	}
	pj = model_power_n(NEVAL, e1r, e1i, e2r, e2i);
	pn = model_power_n(NEVAL, z1r, z1i, z2r, z2i);
	r = 10.0 * log10((pj + pn) / pn);
	if(tm < T_MOVE) { res_static += r; n_static++; if(r > worst_static) worst_static = r; }
	else if(tm > T_MOVE + 300000ULL) { res_move += r; n_move++; if(r > worst_move) worst_move = r; }
}

/* ---- stubs: radio, gain loop, DMA, console, time ------------------------------------------ */
struct ad9361_rf_phy *ad9361_phy = (struct ad9361_rf_phy *)&rng_state;
struct axi_dmac *rx_dmac = (struct axi_dmac *)&tb_now;
void XTime_GetTime(XTime *t) { *t = tb_now; }
void Xil_DCacheInvalidateRange(uintptr_t a, uint32_t n) { (void)a; (void)n; }
int32_t ad9361_get_tx_attenuation(struct ad9361_rf_phy *p, uint8_t ch, uint32_t *a) { (void)p; (void)ch; *a = 40000; return 0; }
int32_t ad9361_get_rx_sampling_freq(struct ad9361_rf_phy *p, uint32_t *f) { (void)p; *f = 61440000; return 0; }
int32_t ad9361_get_tx_sampling_freq(struct ad9361_rf_phy *p, uint32_t *f) { (void)p; *f = 61440000; return 0; }
int32_t ad9361_get_rx_lo_freq(struct ad9361_rf_phy *p, uint64_t *f) { (void)p; *f = 1582500000ULL; return 0; }
int32_t ad9361_get_tx_lo_freq(struct ad9361_rf_phy *p, uint64_t *f) { (void)p; *f = 1582500000ULL; return 0; }
int32_t ad9361_get_rx_rf_bandwidth(struct ad9361_rf_phy *p, uint32_t *b) { (void)p; *b = 56000000; return 0; }
int32_t ad9361_get_rx_fir_en_dis(struct ad9361_rf_phy *p, uint8_t *e) { (void)p; *e = 0; return 0; }
int32_t ad9361_get_tx_fir_en_dis(struct ad9361_rf_phy *p, uint8_t *e) { (void)p; *e = 0; return 0; }
int32_t ad9361_set_rx_fir_en_dis(struct ad9361_rf_phy *p, uint8_t e) { (void)p; (void)e; return 0; }
int32_t ad9361_set_tx_fir_en_dis(struct ad9361_rf_phy *p, uint8_t e) { (void)p; (void)e; return 0; }
int32_t ad9361_set_rx_lo_freq(struct ad9361_rf_phy *p, uint64_t f) { (void)p; (void)f; return 0; }
int32_t ad9361_set_tx_lo_freq(struct ad9361_rf_phy *p, uint64_t f) { (void)p; (void)f; return 0; }
int32_t ad9361_set_rx_sampling_freq(struct ad9361_rf_phy *p, uint32_t f) { (void)p; (void)f; return 0; }
int32_t ad9361_set_tx_sampling_freq(struct ad9361_rf_phy *p, uint32_t f) { (void)p; (void)f; return 0; }
int32_t ad9361_set_rx_rf_bandwidth(struct ad9361_rf_phy *p, uint32_t b) { (void)p; (void)b; return 0; }
int32_t ad9361_set_tx_rf_bandwidth(struct ad9361_rf_phy *p, uint32_t b) { (void)p; (void)b; return 0; }

int32_t  gnss_rxg_gain(void) { return 60; }
int32_t  gnss_rxg_ref(void)  { return 60; }
int      gnss_rxg_mode(void) { return GNSS_RXG_MATCHED; }
void     gnss_rxg_set_mode(int m) { (void)m; }
uint32_t gnss_rxg_changes(void) { return 0; }
/* called by commit_weights() just before the words are written: the solve took SOLVE_US */
void     gnss_rxg_set_digital_db(int db) { (void)db; if(cap_jam) tb_now += SOLVE_US; }
int      gnss_rxg_peak(void) { return 1500; }
void     gnss_rxg_stop_tx_hold(void) {}
void     gnss_rxg_poll(void) {}
uint32_t gnss_rxg_att_base(void) { return 40000; }
int      gnss_rxg_comp_db(void) { return 0; }
int      gnss_txdma_is_running(void) { return 0; }

static uint32_t dma_pending;
int32_t axi_dmac_read(struct axi_dmac *d, uint32_t a, uint32_t *v)
{
	(void)d;
	if(a == AXI_DMAC_REG_IRQ_PENDING) *v = dma_pending;
	else if(a == AXI_DMAC_REG_CTRL) *v = AXI_DMAC_CTRL_ENABLE;
	else *v = 0;
	return 0;
}
int32_t axi_dmac_write(struct axi_dmac *d, uint32_t a, uint32_t v)
{
	(void)d;
	if(a == AXI_DMAC_REG_IRQ_PENDING) dma_pending &= ~v;
	if(a == AXI_DMAC_REG_TRANSFER_SUBMIT && (v & AXI_DMAC_TRANSFER_SUBMIT)) {
		make_capture();                 /* the capture lands at once */
		dma_pending |= AXI_DMAC_IRQ_EOT;
	}
	return 0;
}

static int quiet_console;
static char con_log[1 << 16];
static size_t con_len;
unsigned int console_key_count(void) { return 0; }
int console_typing(void) { return 0; }
void console_print(char *fmt, ...)
{
	char out[1024];
	size_t o = 0;
	va_list ap;
	va_start(ap, fmt);
	for(; *fmt && o < sizeof(out) - 64; fmt++) {
		char spec[16]; int si = 0;
		if(*fmt != '%') { out[o++] = *fmt; continue; }
		spec[si++] = '%'; fmt++;
		while(*fmt && strchr("0123456789.-", *fmt) && si < 12) spec[si++] = *fmt++;
		switch(*fmt) {
		case 'd': spec[si++] = 'l'; spec[si++] = 'd'; spec[si] = 0; o += snprintf(out + o, sizeof(out) - o, spec, va_arg(ap, long)); break;
		case 'x': spec[si++] = 'l'; spec[si++] = 'x'; spec[si] = 0; o += snprintf(out + o, sizeof(out) - o, spec, va_arg(ap, long)); break;
		case 's': spec[si++] = 's'; spec[si] = 0; o += snprintf(out + o, sizeof(out) - o, spec, va_arg(ap, char *)); break;
		case 'c': out[o++] = (char)va_arg(ap, int); break;
		case '%': out[o++] = '%'; break;
		default: out[o++] = '?'; break;
		}
	}
	va_end(ap);
	out[o] = 0;
	if(!quiet_console) fputs(out, stdout);
	if(con_len + o < sizeof(con_log)) { memcpy(con_log + con_len, out, o); con_len += o; con_log[con_len] = 0; }
}

/* ---- the run ------------------------------------------------------------------------------- */
int main(void)
{
	int ok = 1, reported = 0;

	printf("ENGINE_TB: %s (%d + %d taps, REF %d, %d weights + %d image), engine %s\n",
	       CRPA_ALGO_STR, AJ_NT, AJ_NT, AJ_REF, AJ_NSPT, AJ_NW - AJ_NSPT, GNSS_AJ_VERSION_STR);
	gnss_aj_init();
	gnss_aj_enable(1);
	gnss_aj_set_log(0);
#ifdef TB_NOPRED
	gnss_aj_set_predict(0);           /* A/B: the moving jammer without prediction */
#endif
	if(!gnss_aj_enabled()) { printf("ENGINE_TB: FAIL - the engine refused to start\n"); return 1; }
	while(tb_now < T_END) {
		gnss_idle();
		tb_now += 1000;
		if(!reported && tb_now >= 3500000ULL) { reported = 1; gnss_aj_report(); }
	}
	if(n_static) res_static /= n_static;
	if(n_move) res_move /= n_move;
	printf("\nENGINE_TB: %d weight commits, %d bypass commits\n", n_weight_commits, n_bypass_commits);
	printf("ENGINE_TB: static jammer: mean residual of the committed weights over their hold %.2f dB (worst %.2f), %d commits\n",
	       res_static, worst_static, n_static);
	printf("ENGINE_TB: moving jammer: mean residual %.2f dB (worst %.2f) over %d commits\n", res_move, worst_move, n_move);
#define NEED(cond, what) do { if(!(cond)) { printf("ENGINE_TB: FAIL - %s\n", what); ok = 0; } } while(0)
	NEED(strstr(con_log, "*** JAMMER ON ***") != 0, "no JAMMER ON");
	NEED(strstr(con_log, "NULLED in") != 0, "no NULLED");
	NEED(strstr(con_log, "jammer OFF after") != 0, "no jammer OFF");
	NEED(strstr(con_log, "===== END REPORT =====") != 0, "report incomplete");
	NEED(n_static > 20, "too few static-jammer commits");
	NEED(res_static <= 1.5, "static jammer not nulled (mean residual > 1.5 dB)");
	NEED(cfg_act & 1, "filter not bypassed after the jammer left");
#if defined(AJ_DIRECT_DEF)
	NEED(n_move > 20 && res_move <= 3.0, "moving jammer not followed (mean residual > 3 dB)");
#else
	/* crpa1 has no weight prediction: reported, not judged */
	if(n_move && res_move > 3.0) printf("ENGINE_TB: note - crpa1 has no weight prediction; moving-jammer residual is informational\n");
#endif
	printf("ENGINE_TB: %s  %s\n", ok ? "PASS" : "FAIL", CRPA_ALGO_STR);
	return ok ? 0 : 1;
}
