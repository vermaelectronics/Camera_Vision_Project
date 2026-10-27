/* host stub (Tests/host): Xilinx standalone global timer */
#pragma once
typedef unsigned long long XTime;
#define COUNTS_PER_SECOND 333333333ULL
void XTime_GetTime(XTime *t);
