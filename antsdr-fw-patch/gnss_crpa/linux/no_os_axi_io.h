#ifndef NO_OS_AXI_IO_H_LX
#define NO_OS_AXI_IO_H_LX
#include <stdint.h>
#include "lx_platform.h"
static inline int32_t no_os_axi_io_read(uint32_t base, uint32_t offset, uint32_t *data)
{
	*data = lx_in32(base + offset);
	return 0;
}
static inline int32_t no_os_axi_io_write(uint32_t base, uint32_t offset, uint32_t data)
{
	lx_out32(base + offset, data);
	return 0;
}
#endif
