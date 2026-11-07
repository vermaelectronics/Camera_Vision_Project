#ifndef XTIME_L_H_LX
#define XTIME_L_H_LX
#include <stdint.h>
#include "lx_platform.h"
/* Same tick as the Zynq global timer (CPU clock / 2 = 333.33 MHz), so the
 * firmware's time arithmetic (and its overflow behaviour) is unchanged. */
typedef uint64_t XTime;
#define COUNTS_PER_SECOND 333333333ULL
static inline void XTime_GetTime(XTime *t)
{
	*t = (XTime)(lx_time_ns() / 3ULL);
}
#endif
