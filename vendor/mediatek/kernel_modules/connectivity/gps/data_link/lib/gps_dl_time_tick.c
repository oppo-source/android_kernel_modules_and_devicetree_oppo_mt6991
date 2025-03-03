/*/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2021 MediaTek Inc.
 */
#include "gps_dl_config.h"
#include "gps_dl_time_tick.h"

#if GPS_DL_ON_LINUX
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <asm/div64.h>
#include <linux/time.h>
#include <linux/timekeeping.h>
#include <linux/sched/clock.h>
#elif GPS_DL_ON_CTP
#include "kernel_to_ctp.h"
#endif

void gps_dl_sleep_us(unsigned int min_us, unsigned int max_us)
{
#if GPS_DL_ON_LINUX
	usleep_range(min_us, max_us);
#elif GPS_DL_ON_CTP
	usleep_range(min_us, max_us);
#endif
}

void gps_dl_wait_us(unsigned int us)
{
#if GPS_DL_ON_LINUX
	udelay(us);
#elif GPS_DL_ON_CTP
	udelay(us); /* GPT_Delay_us(us); */
#endif
}

unsigned long gps_dl_tick_get(void)
{
#if GPS_DL_ON_LINUX
	return jiffies;
#elif GPS_DL_ON_CTP
	return GPT_GetTickCount(0);
#else
	return 0;
#endif
}

#define GPS_NSEC_IN_USEC (1000)
unsigned long gps_dl_tick_get_us(void)
{
	unsigned long tmp;

	/* tmp is ns */
	tmp = local_clock();

	/* tmp is changed to ms after */
	do_div(tmp, GPS_NSEC_IN_USEC);

	return tmp;
}

#define GPS_NSEC_IN_MSEC (1000000)
unsigned long gps_dl_tick_get_ms(void)
{
	unsigned long tmp;

	/* tmp is ns */
	tmp = local_clock();

	/* tmp is changed to ms after */
	do_div(tmp, GPS_NSEC_IN_MSEC);

	return tmp;
}

unsigned long gps_dl_tick_get_ktime_ms(void)
{
	struct timespec64 ts;
	unsigned long tmp;

	/* Get the real-time since 1970 */
	ktime_get_real_ts64(&ts);

	/* Convert seconds to milliseconds and nanoseconds to milliseconds */
	tmp = (unsigned long)(ts.tv_sec * 1000 + ts.tv_nsec / GPS_NSEC_IN_MSEC);

	return tmp;
}

#define NOHOP_FREQ_TO_MS 13000
unsigned long gps_dl_tick_get_no_hop_ktime_ms(void)
{
	unsigned long tmp1, tmp2;

	/* Get the real-time since boot */
	tmp1 = __arch_counter_get_cntvct();

	/* Get ms*/
	tmp2 = (tmp1 / NOHOP_FREQ_TO_MS);

	return tmp2;
}

int gps_dl_tick_delta_to_usec(unsigned int tick0, unsigned int tick1)
{
#if GPS_DL_ON_LINUX
	return (int)((tick1 - tick0) * 1000 * 1000 / HZ);
#elif GPS_DL_ON_CTP
	return (int)((tick1 - tick0) / 13);
#else
	return 0;
#endif
}

