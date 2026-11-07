#ifndef NO_OS_GPIO_H_LX
#define NO_OS_GPIO_H_LX
#include <stdint.h>
#include "lx_platform.h"
struct no_os_gpio_desc { int number; };
static inline int32_t no_os_gpio_set_value(struct no_os_gpio_desc *d, uint8_t v)
{
	if (d && d->number >= 0) lx_gpio_set(d->number, v);
	return 0;
}
#endif
