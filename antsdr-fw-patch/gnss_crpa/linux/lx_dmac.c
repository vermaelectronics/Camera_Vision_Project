/* gnss_crpa - the two AXI DMACs, emulated on the Linux IIO buffers.
 *
 * Under Linux the axi-dmac driver owns 0x7C400000 (RX) and 0x7C420000 (TX)
 * and answers their interrupts, so the firmware may not program them. The
 * firmware's register-level DMA code (gnss_aj.c) and its no-OS transfer calls
 * (gnss_txdma.c, gnss_capture.c) are kept unchanged and land here instead:
 *
 * RX  cf-ad9361-lpc, all four channels (RX1 I/Q, RX2 I/Q), read through
 *     /dev/iio:deviceN. The kernel's channel packer writes the same
 *     [I1 Q1 I2 Q2] int16 layout as the bare-metal DMA. A transfer is
 *     "submitted" when the firmware writes TRANSFER_SUBMIT: samples older
 *     than that are thrown away, the next X_LENGTH+1 bytes are copied to the
 *     destination address (a DDR shadow buffer, lx_io.c) and EOT is raised
 *     in IRQ_PENDING when they are complete. Keeping the buffer open also
 *     keeps the ADC channel enables set, which gnss_passthrough needs, and
 *     stops other programs from reconfiguring the RX channels underneath it.
 *
 * TX  cf-ad9361-dds-core-lpc through libiio: a cyclic transfer becomes a
 *     cyclic IIO buffer (the DDR replay, gnss_ddr_tx=). The DAC data-select
 *     registers are restored when it stops, because the live RX -> TX path
 *     needs them on "DMA" (gnss_passthrough drives axi_ad9361's DMA inputs).
 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#ifndef GNSS_CRPA_NO_LIBIIO
#include <iio.h>
#endif

#include "lx_platform.h"
#include "axi_dmac.h"

#define NREG (0x430 / 4)

struct emu {
	uint32_t regs[NREG];
	uint32_t pending;          /* IRQ_PENDING */
	int      active;           /* a transfer is running */
	uint32_t addr, len, got;   /* destination (RX) / source (TX) */
};

static struct emu rx, tx;

/* ---- RX capture through /dev/iio:deviceN ---------------------------------- */
static int cap_fd = -1;
static char cap_dir[256];
static uint32_t cap_block;      /* bytes per kernel block */
static int cap_ok;

static int cap_cfg(uint32_t block_bytes)
{
	char v[32];
	int ch;

	if (cap_fd >= 0 && cap_block == block_bytes)
		return 0;
	if (cap_fd >= 0) {
		close(cap_fd);
		cap_fd = -1;
	}
	(void)lx_sysfs_write(cap_dir, "buffer/enable", "0");
	for (ch = 0; ch < 4; ch++) {
		char a[64];
		snprintf(a, sizeof(a), "scan_elements/in_voltage%d_en", ch);
		if (lx_sysfs_write(cap_dir, a, "1"))
			return -1;
	}
	/* two blocks of block_bytes each: one is read while the next fills */
	snprintf(v, sizeof(v), "%u", (unsigned)(2U * block_bytes / 8U));
	if (lx_sysfs_write(cap_dir, "buffer/length", v))
		return -1;
	if (lx_sysfs_write(cap_dir, "buffer/enable", "1"))
		return -1;
	{
		char dev[256];
		const char *dd = getenv("GNSS_CRPA_DEVDIR");      /* host tests */
		snprintf(dev, sizeof(dev), "%s/%s", (dd && *dd) ? dd : "/dev", strrchr(cap_dir, '/') + 1);
		cap_fd = open(dev, O_RDONLY | O_NONBLOCK);
	}
	if (cap_fd < 0) {
		(void)lx_sysfs_write(cap_dir, "buffer/enable", "0");
		return -1;
	}
	cap_block = block_bytes;
	return 0;
}

int lx_capture_open(void)
{
	const char *d = lx_iio_dir("cf-ad9361-lpc");

	if (!d) {
		fprintf(stderr, "gnss_crpa: no cf-ad9361-lpc IIO device\n");
		return -1;
	}
	snprintf(cap_dir, sizeof(cap_dir), "%s", d);
	if (cap_cfg(65536U)) {
		fprintf(stderr, "gnss_crpa: cannot start the RX IIO buffer (%s): %s\n"
			"  is another program (iio_readdev, GNSS viewer, iiod client) streaming RX?\n",
			cap_dir, strerror(errno));
		return -1;
	}
	cap_ok = 1;
	return 0;
}

void lx_capture_close(void)
{
	if (cap_fd >= 0)
		close(cap_fd);
	cap_fd = -1;
	if (cap_ok)
		(void)lx_sysfs_write(cap_dir, "buffer/enable", "0");
	cap_ok = 0;
}

static void cap_drain(void)
{
	static uint8_t junk[16384];

	if (cap_fd < 0)
		return;
	while (read(cap_fd, junk, sizeof(junk)) > 0)
		;
}

/* Copy what has arrived; 1 when the transfer is complete. */
static int cap_progress(int wait_ms)
{
	uint8_t *dst;
	ssize_t n;

	if (!rx.active)
		return 0;
	dst = lx_ddr(rx.addr, rx.len);
	if (!dst || cap_fd < 0) {
		rx.active = 0;
		return 0;
	}
	for (;;) {
		n = read(cap_fd, dst + rx.got, rx.len - rx.got);
		if (n > 0) {
			rx.got += (uint32_t)n;
			if (rx.got >= rx.len) {
				rx.active = 0;
				rx.pending |= AXI_DMAC_IRQ_SOT | AXI_DMAC_IRQ_EOT;
				return 1;
			}
			continue;
		}
		if (n < 0 && errno == EAGAIN && wait_ms > 0) {
			struct pollfd p = { cap_fd, POLLIN, 0 };
			if (poll(&p, 1, wait_ms) > 0)
				continue;
		}
		return 0;
	}
}

int lx_capture_fd(void)
{
	return rx.active ? cap_fd : -1;
}

static int rx_submit(void)
{
	uint32_t len = rx.regs[AXI_DMAC_REG_X_LENGTH / 4] + 1U;
	uint32_t addr = rx.regs[AXI_DMAC_REG_DEST_ADDRESS / 4];

	if (!cap_ok || !lx_ddr(addr, len))
		return -1;
	/* one kernel block per AJ capture (64 KiB); a long capture (DDR replay
	 * recording) gets a block of its own size so it is one contiguous DMA */
	if (cap_cfg(len <= 65536U ? 65536U : (len + 4095U) & ~4095U))
		return -1;
	cap_drain();                 /* only samples taken after the submit */
	rx.addr = addr;
	rx.len = len;
	rx.got = 0;
	rx.active = 1;
	return 0;
}

/* ---- TX cyclic replay through libiio ------------------------------------- */
#ifndef GNSS_CRPA_NO_LIBIIO
static struct iio_context *tx_ctx;
static struct iio_buffer *tx_buf;
static uint32_t dac_sel_saved[4];
#endif

static int tx_start(uint32_t src, uint32_t bytes, int cyclic)
{
#ifdef GNSS_CRPA_NO_LIBIIO
	(void)src; (void)bytes; (void)cyclic;
	return -ENOSYS;
#else
	struct iio_device *dev;
	const uint8_t *data = lx_ddr(src, bytes);
	int ch;

	if (!data || bytes < 8U)
		return -EINVAL;
	if (tx_buf)
		return -EBUSY;
	for (ch = 0; ch < 4; ch++)
		dac_sel_saved[ch] = lx_in32(LX_DAC_BASE + 0x418U + 0x40U * ch);
	if (!tx_ctx)
		tx_ctx = iio_create_local_context();
	if (!tx_ctx)
		return -ENODEV;
	dev = iio_context_find_device(tx_ctx, "cf-ad9361-dds-core-lpc");
	if (!dev)
		return -ENODEV;
	for (ch = 0; ch < 4; ch++) {
		char name[16];
		struct iio_channel *c;
		snprintf(name, sizeof(name), "voltage%d", ch);
		c = iio_device_find_channel(dev, name, true);
		if (!c)
			return -ENODEV;
		iio_channel_enable(c);
	}
	tx_buf = iio_device_create_buffer(dev, bytes / 8U, cyclic ? true : false);
	if (!tx_buf)
		return -errno;
	memcpy(iio_buffer_start(tx_buf), data, (bytes / 8U) * 8U);
	if (iio_buffer_push(tx_buf) < 0) {
		iio_buffer_destroy(tx_buf);
		tx_buf = NULL;
		return -EIO;
	}
	return 0;
#endif
}

static void tx_stop(void)
{
#ifndef GNSS_CRPA_NO_LIBIIO
	int ch;

	if (!tx_buf)
		return;
	iio_buffer_destroy(tx_buf);
	tx_buf = NULL;
	/* the kernel puts the DAC back on DDS; the firmware decides the source */
	for (ch = 0; ch < 4; ch++)
		lx_out32(LX_DAC_BASE + 0x418U + 0x40U * ch, dac_sel_saved[ch]);
	lx_out32(LX_DAC_BASE + 0x44U, 1U);            /* SYNC */
#endif
}

/* ---- register interface -------------------------------------------------- */
static struct emu *which(uint32_t base)
{
	if (base == LX_RX_DMAC_BASE) return &rx;
	if (base == LX_TX_DMAC_BASE) return &tx;
	return NULL;
}

int lx_dmac_read(uint32_t base, uint32_t reg, uint32_t *val)
{
	struct emu *e = which(base);

	*val = 0;
	if (!e || reg >= NREG * 4U)
		return -1;
	switch (reg) {
	case AXI_DMAC_REG_IRQ_PENDING:
		if (e == &rx)
			(void)cap_progress(0);
		*val = e->pending;
		break;
	case AXI_DMAC_REG_TRANSFER_SUBMIT:
		*val = e->active ? AXI_DMAC_QUEUE_FULL : 0U;
		break;
	case AXI_DMAC_REG_CTRL:
		*val = e->regs[reg / 4] & AXI_DMAC_CTRL_ENABLE;
		break;
	default:
		*val = e->regs[reg / 4];
	}
	return 0;
}

int lx_dmac_write(uint32_t base, uint32_t reg, uint32_t val)
{
	struct emu *e = which(base);

	if (!e || reg >= NREG * 4U)
		return -1;
	switch (reg) {
	case AXI_DMAC_REG_IRQ_PENDING:
		e->pending &= ~val;              /* write 1 to clear */
		break;
	case AXI_DMAC_REG_CTRL:
		e->regs[reg / 4] = val;
		if (!(val & AXI_DMAC_CTRL_ENABLE)) {   /* disable = abort */
			e->active = 0;
			if (e == &tx)
				tx_stop();
		}
		break;
	case AXI_DMAC_REG_TRANSFER_SUBMIT:
		if ((val & AXI_DMAC_TRANSFER_SUBMIT) && e == &rx && !rx.active)
			(void)rx_submit();
		break;
	default:
		e->regs[reg / 4] = val;
	}
	return 0;
}

/* ---- no-OS transfer API -------------------------------------------------- */
int32_t axi_dmac_read(struct axi_dmac *d, uint32_t reg, uint32_t *v)
{
	return lx_dmac_read(d->base, reg, v);
}

int32_t axi_dmac_write(struct axi_dmac *d, uint32_t reg, uint32_t v)
{
	return lx_dmac_write(d->base, reg, v);
}

int32_t axi_dmac_transfer_start(struct axi_dmac *d, struct axi_dma_transfer *t)
{
	t->transfer_done = false;
	if (d->base == LX_RX_DMAC_BASE) {
		rx.regs[AXI_DMAC_REG_DEST_ADDRESS / 4] = t->dest_addr;
		rx.regs[AXI_DMAC_REG_X_LENGTH / 4] = t->size - 1U;
		rx.pending = 0;
		rx.active = 0;
		return rx_submit() ? -1 : 0;
	}
	if (d->base == LX_TX_DMAC_BASE) {
		int r = tx_start(t->src_addr, t->size, t->cyclic == CYCLIC);
		if (r)
			fprintf(stderr, "gnss_crpa: TX replay buffer: %s\n", strerror(-r));
		tx.active = (r == 0);
		return r ? -1 : 0;
	}
	return -1;
}

int32_t axi_dmac_transfer_wait_completion(struct axi_dmac *d, uint32_t timeout_ms)
{
	uint64_t end = lx_time_ns() + (uint64_t)timeout_ms * 1000000ULL;

	if (d->base != LX_RX_DMAC_BASE)
		return 0;
	while (rx.active) {
		if (cap_progress(10))
			return 0;
		if (lx_time_ns() > end) {
			rx.active = 0;
			return -ETIMEDOUT;
		}
	}
	return (rx.pending & AXI_DMAC_IRQ_EOT) ? 0 : -1;
}

void axi_dmac_transfer_stop(struct axi_dmac *d)
{
	if (d->base == LX_TX_DMAC_BASE) {
		tx_stop();
		tx.active = 0;
	} else if (d->base == LX_RX_DMAC_BASE) {
		rx.active = 0;
	}
}
