#ifndef NO_OS_UTIL_H_LX
#define NO_OS_UTIL_H_LX
#include <stdint.h>
#include <stdbool.h>
#define NO_OS_BIT(x)            (1UL << (x))
#define NO_OS_GENMASK(h, l)     ((uint32_t)(((0xFFFFFFFFUL) << (l)) & (0xFFFFFFFFUL >> (31 - (h)))))
#define NO_OS_ARRAY_SIZE(x)     (sizeof(x) / sizeof((x)[0]))
#define NO_OS_DIV_ROUND_CLOSEST(x, d) (((x) + (d) / 2) / (d))
#define NO_OS_DIV_ROUND_UP(x, d) (((x) + (d) - 1) / (d))
#define no_os_min(x, y)         (((x) < (y)) ? (x) : (y))
#define no_os_max(x, y)         (((x) > (y)) ? (x) : (y))
static inline uint64_t no_os_do_div(uint64_t *n, uint64_t base)
{
	uint64_t mod = *n % base;
	*n = *n / base;
	return mod;
}
#endif
