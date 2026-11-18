/******************************************************************************
 *  gnss_aj_math.h  --  LOOP-BACK build: no CRPA solver. Only the few
 *  constants and helpers the shared console / info code refers to.
 *****************************************************************************/
#ifndef GNSS_AJ_MATH_H_
#define GNSS_AJ_MATH_H_
#include <stdint.h>
#define AJ_NT     0
#define AJ_REF    0
#define AJ_NSAMP  8192
#define AJ_QFLOOR 0.25
double aj_sqrt(double x);
double aj_db10(double x);
#endif
