/* host test stub */
#ifndef XTIME_L_H_STUB
#define XTIME_L_H_STUB
#include <stdint.h>
typedef uint64_t XTime;
#define COUNTS_PER_SECOND 1000000ULL      /* host test: 1 count = 1 us */
void XTime_GetTime(XTime *t);
#endif
