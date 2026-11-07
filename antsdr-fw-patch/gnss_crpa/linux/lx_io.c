/* gnss_crpa - physical address access on Linux.
 *
 * PL registers are mapped from /dev/mem in 64 KiB windows on first use.
 * DDR buffer addresses the firmware uses (0x10000000 - 0x11FFFFFF) live in an
 * anonymous shadow mapping instead: the kernel owns that memory, and the only
 * thing that ever "DMAs" into it here is lx_dmac.c, which copies the samples
 * from the Linux IIO buffer.
 *
 * GNSS_CRPA_FAKE=1 replaces /dev/mem by zero-filled memory (host tests).
 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include "lx_platform.h"

#define WIN_SIZE  0x10000U
#define MAX_WIN   16

static struct { uint32_t phys; volatile uint8_t *virt; } win[MAX_WIN];
static int nwin;
static int memfd = -1;
static int fake;
static uint8_t *ddr_shadow;

/* Only the windows the firmware needs may be mapped: a typo in a register
 * command must not be able to poke arbitrary physical memory. */
static int allowed(uint32_t base)
{
	switch (base) {
	case LX_PT_BASE:
	case LX_ADC_BASE & ~(WIN_SIZE - 1):   /* 0x79020000 - 0x7902FFFF (ADC + DAC) */
	case LX_RX_DMAC_BASE:
	case LX_TX_DMAC_BASE:
	case LX_GPIO_BASE:
		return 1;
	default:
		return 0;
	}
}

int lx_io_init(void)
{
	const char *f = getenv("GNSS_CRPA_FAKE");

	fake = f && *f && *f != '0';
	if (!fake) {
		memfd = open("/dev/mem", O_RDWR | O_SYNC);
		if (memfd < 0) {
			fprintf(stderr, "gnss_crpa: /dev/mem: %s (run as root)\n", strerror(errno));
			return -1;
		}
	}
	ddr_shadow = mmap(NULL, LX_DDR_SHADOW_SIZE, PROT_READ | PROT_WRITE,
			  MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
	if (ddr_shadow == MAP_FAILED) {
		ddr_shadow = NULL;
		fprintf(stderr, "gnss_crpa: cannot reserve the %u MiB sample buffer\n",
			LX_DDR_SHADOW_SIZE >> 20);
		return -1;
	}
	return 0;
}

volatile uint32_t *lx_reg(uint32_t phys)
{
	uint32_t base = phys & ~(WIN_SIZE - 1);
	int i;

	for (i = 0; i < nwin; i++)
		if (win[i].phys == base)
			return (volatile uint32_t *)(win[i].virt + (phys - base));
	if (!allowed(base) || nwin == MAX_WIN)
		return NULL;
	if (fake) {
		win[nwin].virt = calloc(1, WIN_SIZE);
		if (win[nwin].virt && base == LX_PT_BASE) {      /* identify as SMI-PI 4.9.14 */
			volatile uint32_t *r = (volatile uint32_t *)win[nwin].virt;
			r[0x00 / 4] = 0x47435031U;
			r[0x04 / 4] = 0x0004090EU;
			r[0x184 / 4] = 0x06030C07U;
			r[0x10 / 4] = 0x0005000FU;   /* STATUS: ADC/DAC enables, pass/smi synced */
		}
		if (win[nwin].virt && base == (LX_ADC_BASE & ~(WIN_SIZE - 1))) {
			volatile uint32_t *r = (volatile uint32_t *)(win[nwin].virt + 0x4000);
			r[0x5C / 4] = 1;                         /* DAC status */
			r[0x54 / 4] = 0x0003D500;                /* 245.76 MHz / ratio 1 */
			r[0x58 / 4] = 1;
		}
	} else {
		void *p = mmap(NULL, WIN_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, memfd, base);
		win[nwin].virt = (p == MAP_FAILED) ? NULL : p;
	}
	if (!win[nwin].virt)
		return NULL;
	win[nwin].phys = base;
	return (volatile uint32_t *)(win[nwin++].virt + (phys - base));
}

void *lx_ddr(uint32_t phys, uint32_t bytes)
{
	if (!ddr_shadow || phys < LX_DDR_SHADOW_BASE)
		return NULL;
	if ((uint64_t)phys - LX_DDR_SHADOW_BASE + bytes > LX_DDR_SHADOW_SIZE)
		return NULL;
	return ddr_shadow + (phys - LX_DDR_SHADOW_BASE);
}

uint32_t lx_in32(uint32_t phys)
{
	volatile uint32_t *r;
	uint32_t v = 0;

	if (phys >= LX_RX_DMAC_BASE && phys < LX_TX_DMAC_BASE + WIN_SIZE) {
		lx_dmac_read(phys & ~(WIN_SIZE - 1), phys & (WIN_SIZE - 1), &v);
		return v;
	}
	r = lx_reg(phys);
	if (r)
		return *r;
	{
		uint32_t *d = lx_ddr(phys, 4);
		if (d)
			return *d;
	}
	return 0xDEADBEEFU;
}

void lx_out32(uint32_t phys, uint32_t value)
{
	volatile uint32_t *r;

	if (phys >= LX_RX_DMAC_BASE && phys < LX_TX_DMAC_BASE + WIN_SIZE) {
		lx_dmac_write(phys & ~(WIN_SIZE - 1), phys & (WIN_SIZE - 1), value);
		return;
	}
	r = lx_reg(phys);
	if (r) {
		*r = value;
		return;
	}
	{
		uint32_t *d = lx_ddr(phys, 4);
		if (d)
			*d = value;
	}
}

/* Zynq-7000 PS GPIO: MASK_DATA_n_LSW/MSW give atomic writes of single pins,
 * which is also what the kernel's gpio-zynq driver uses, so the two never
 * disturb each other's pins. Linux numbers: MIO 0-53, EMIO n = 54 + n
 * (bank 2 = EMIO 0-31, bank 3 = EMIO 32-63). DIRM/OEN are left to the
 * kernel, which owns these lines (ad9361 external band control). */
void lx_gpio_set(int linux_gpio, int value)
{
	int bank, bit;
	uint32_t reg;

	if (linux_gpio < 0 || linux_gpio > 117)
		return;
	if (linux_gpio < 32)       { bank = 0; bit = linux_gpio; }
	else if (linux_gpio < 54)  { bank = 1; bit = linux_gpio - 32; }
	else if (linux_gpio < 86)  { bank = 2; bit = linux_gpio - 54; }
	else                       { bank = 3; bit = linux_gpio - 86; }
	reg = LX_GPIO_BASE + 8U * bank + ((bit >= 16) ? 4U : 0U);
	bit &= 15;
	lx_out32(reg, (~(1U << (bit + 16)) & 0xFFFF0000U) | ((value ? 1U : 0U) << bit));
}

uint64_t lx_time_ns(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

void lx_sleep_us(uint32_t us)
{
	struct timespec ts = { us / 1000000U, (long)(us % 1000000U) * 1000L };

	while (nanosleep(&ts, &ts) && errno == EINTR)
		;
}
