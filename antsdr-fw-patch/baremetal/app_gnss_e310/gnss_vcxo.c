/* gnss_vcxo.c -- see gnss_vcxo.h */
#include <stdint.h>
#include "app_config.h"
#include "console.h"
#include "gnss_vcxo.h"
#include "xil_io.h"

#ifdef ANTSDR_DESIGN_A

#define VCXO_MODE     0x00U
#define VCXO_VALUE    0x04U
#define VCXO_INUSE    0x08U
#define VCXO_REFSEL   0x0CU
#define VCXO_LOCKED   0x10U

static uint32_t rd(uint32_t off) { return Xil_In32(GNSS_VCXO_BASEADDR + off); }
static void wr(uint32_t off, uint32_t v) { Xil_Out32(GNSS_VCXO_BASEADDR + off, v); }

void gnss_vcxo_init(void)
{
	wr(VCXO_VALUE, GNSS_VCXO_DAC_DEFAULT);
	wr(VCXO_MODE, 1U);
	console_print("gnss_vcxo: 40 MHz VCXO DAC fixed at %d (as design A under Linux); vcxo? / vcxo_dac= / vcxo_ref=\n",
		      (long)GNSS_VCXO_DAC_DEFAULT);
}

void get_vcxo(double *param, char param_no)
{
	uint32_t mode = rd(VCXO_MODE) & 1U, ref = rd(VCXO_REFSEL) & 3U;
	(void)param; (void)param_no;
	console_print("VCXO (axi_vcxo_ctrl @ 0x43C00000): %s, DAC set %d, DAC in use %d\n",
		      (char *)(mode ? "fixed value" : (ref == 0U ? "locking to the 10 MHz input" : "locking to the PPS input")),
		      (long)(rd(VCXO_VALUE) & 0xFFFFU), (long)(rd(VCXO_INUSE) & 0xFFFFU));
	if (!mode)
		console_print("  reference %s\n", (char *)((rd(VCXO_LOCKED) & 1U) ? "LOCKED" : "not locked (no reference signal?)"));
}

void set_vcxo_dac(double *param, char param_no)
{
	if (param_no < 1 || param[0] < 0.0 || param[0] > 65535.0) {
		console_print("vcxo_dac= needs a DAC value 0..65535 (default %d)\n", (long)GNSS_VCXO_DAC_DEFAULT);
		return;
	}
	wr(VCXO_VALUE, (uint32_t)param[0]);
	wr(VCXO_MODE, 1U);
	get_vcxo(param, param_no);
}

void set_vcxo_ref(double *param, char param_no)
{
	int r = (param_no >= 1) ? (int)param[0] : -1;
	if (r < 0 || r > 2) {
		console_print("vcxo_ref= needs 0 (fixed DAC value), 1 (lock to 10 MHz input) or 2 (lock to PPS input)\n");
		return;
	}
	if (r == 0) {
		wr(VCXO_MODE, 1U);
	} else {
		wr(VCXO_REFSEL, (uint32_t)(r - 1));
		wr(VCXO_MODE, 0U);
	}
	get_vcxo(param, param_no);
}

#else  /* not design A: there is no VCXO control core */
void gnss_vcxo_init(void) { }
void get_vcxo(double *p, char n) { (void)p; (void)n; console_print("no VCXO control in this FPGA\n"); }
void set_vcxo_dac(double *p, char n) { get_vcxo(p, n); }
void set_vcxo_ref(double *p, char n) { get_vcxo(p, n); }
#endif
