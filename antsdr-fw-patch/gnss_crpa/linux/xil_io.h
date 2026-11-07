#ifndef XIL_IO_H_LX
#define XIL_IO_H_LX
#include "lx_platform.h"
#define Xil_In32(a)      lx_in32((uint32_t)(a))
#define Xil_Out32(a, v)  lx_out32((uint32_t)(a), (uint32_t)(v))
#endif
