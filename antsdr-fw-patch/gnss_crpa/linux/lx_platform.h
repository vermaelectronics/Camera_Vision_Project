/* gnss_crpa - Linux platform layer for the SMI-PI v4.9.14 E310 firmware.
 *
 * The bare-metal firmware addresses the PL cores, the AXI DMACs and DDR by
 * PHYSICAL address. This layer keeps those addresses and translates them:
 *   PL cores (gnss_passthrough 0x43C10000, axi_ad9361 0x79020000/0x79024000,
 *   Zynq GPIO 0xE000A000)  -> mmap of /dev/mem
 *   DDR buffers (0x10000000 - 0x11FFFFFF)  -> a private shadow buffer
 *   AXI DMACs (0x7C400000 / 0x7C420000)    -> emulated on the Linux IIO
 *                                             buffers (lx_dmac.c), because the
 *                                             kernel driver owns the DMACs.
 */
#ifndef LX_PLATFORM_H_
#define LX_PLATFORM_H_

#include <stdint.h>
#include <stddef.h>

#define LX_DDR_SHADOW_BASE  0x10000000U
#define LX_DDR_SHADOW_SIZE  0x02000000U   /* 32 MiB, committed lazily */

#define LX_RX_DMAC_BASE     0x7C400000U
#define LX_TX_DMAC_BASE     0x7C420000U
#define LX_ADC_BASE         0x79020000U
#define LX_DAC_BASE         0x79024000U
#define LX_PT_BASE          0x43C10000U
#define LX_GPIO_BASE        0xE000A000U

int       lx_io_init(void);
volatile uint32_t *lx_reg(uint32_t phys);        /* NULL if not mappable */
void     *lx_ddr(uint32_t phys, uint32_t bytes);  /* NULL if outside the shadow */
uint32_t  lx_in32(uint32_t phys);
void      lx_out32(uint32_t phys, uint32_t value);

/* Zynq PS GPIO (Linux numbering: EMIO n = 54 + n) */
void      lx_gpio_set(int linux_gpio, int value);

/* AXI DMAC emulation (lx_dmac.c) */
int       lx_dmac_read(uint32_t base, uint32_t reg, uint32_t *val);
int       lx_dmac_write(uint32_t base, uint32_t reg, uint32_t val);
int       lx_capture_open(void);
void      lx_capture_close(void);

/* time */
uint64_t  lx_time_ns(void);
void      lx_sleep_us(uint32_t us);

/* sysfs helpers (lx_ad9361.c) */
const char *lx_iio_dir(const char *name);        /* /sys/bus/iio/devices/iio:deviceN */
int       lx_iio_index(const char *name);
int       lx_sysfs_read(const char *dir, const char *attr, char *buf, size_t len);
int       lx_sysfs_write(const char *dir, const char *attr, const char *val);

/* output (lx_main.c): everything the firmware prints */
void      lx_out(const char *s, size_t n);
int       lx_printf(const char *fmt, ...) __attribute__((format(__printf__, 1, 2)));
int       lx_putchar(int c);
int       lx_puts(const char *s);
int       lx_getchar(void);
int       lx_getchar_wait(int timeout_ms);   /* -1 = nothing within timeout */
int       lx_capture_fd(void);               /* RX fd while a capture runs, else -1 */
void      lx_flush(void);

#endif
