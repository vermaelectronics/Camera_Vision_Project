/***************************************************************************//**
 *   @file   gnss_time.h
 *   @brief  Seconds since boot from the Cortex-A9 global timer, without the
 *           BSP header xtime_l.h (not exported by every Vitis 2023.2 SDT
 *           platform). Same counter XTime_GetTime() reads: 64-bit, runs at
 *           the CPU clock / 2 (666.67 MHz / 2 on the E310).
*******************************************************************************/
#ifndef GNSS_TIME_H_
#define GNSS_TIME_H_

#include <stdint.h>

#define GNSS_GTIMER_BASE	0xF8F00200U	/* SCU global timer */
#define GNSS_GTIMER_LO		(*(volatile uint32_t *)(GNSS_GTIMER_BASE + 0x00U))
#define GNSS_GTIMER_HI		(*(volatile uint32_t *)(GNSS_GTIMER_BASE + 0x04U))
#define GNSS_GTIMER_CTRL	(*(volatile uint32_t *)(GNSS_GTIMER_BASE + 0x08U))

#ifndef GNSS_CPU_HZ
#define GNSS_CPU_HZ		666666687ULL	/* PS7 APU clock of the ant design */
#endif
#define GNSS_COUNTS_PER_SECOND	(GNSS_CPU_HZ / 2ULL)

typedef uint64_t gnss_time_t;

static inline gnss_time_t gnss_time_now(void)
{
	uint32_t hi, lo;

	if (!(GNSS_GTIMER_CTRL & 1U))		/* not started by the BSP: start it */
		GNSS_GTIMER_CTRL |= 1U;
	do {					/* consistent 64-bit read */
		hi = GNSS_GTIMER_HI;
		lo = GNSS_GTIMER_LO;
	} while (hi != GNSS_GTIMER_HI);
	return ((gnss_time_t)hi << 32) | lo;
}

#endif
