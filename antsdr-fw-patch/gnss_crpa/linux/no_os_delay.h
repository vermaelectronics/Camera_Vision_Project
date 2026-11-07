#ifndef NO_OS_DELAY_H_LX
#define NO_OS_DELAY_H_LX
#include <stdint.h>
#include "lx_platform.h"
static inline void no_os_udelay(uint32_t us) { lx_sleep_us(us); }
static inline void no_os_mdelay(uint32_t ms) { lx_sleep_us(ms * 1000U); }
#endif
