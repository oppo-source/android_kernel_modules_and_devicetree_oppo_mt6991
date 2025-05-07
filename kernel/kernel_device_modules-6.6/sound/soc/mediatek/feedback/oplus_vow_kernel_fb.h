/************************************************************************************
** File: - kernel/kernel_device_modules-6.6/sound/soc/mediatek/feedback/oplus_vow_kernel_fb.h
** CONFIG_OPLUS_FEATURE_MM_FEEDBACK
** Copyright (C), 2022-2024, OPLUS Mobile Comm Corp., Ltd
**
** Description:
**     add oplus vow kernel feedback
** Version: 1.0
** --------------------------- Revision History: --------------------------------
**      <author>                                        <date>              <desc>
************************************************************************************/
#ifndef OPLUS_VOW_KERNEL_FB_H
#define OPLUS_VOW_KERNEL_FB_H

#include "oplus_audio_kernel_fb.h"

/* kernel vow event id */
enum {
	OPLUS_VOW_EVENTID_KERNEL_ERR = 10006,
};

/* scp vow event id */
enum {
	OPLUS_VOW_EVENTID_VOW_STATE_ERR = 13000,
	OPLUS_VOW_EVENTID_RECOG_INPUT_DATA_ERR = 13001,
};

typedef struct {
	uint32_t event_id;
	uint32_t vow_state;
} oplus_vow_monitor_ipi_info_t;

#define vow_kernel_fb(event, level, delay, fmt, ...) \
	do { \
		static DEFINE_RATELIMIT_STATE(_rs, \
			FEEDBACK_RATELIMIT_INTERVAL, \
			FEEDBACK_RATELIMIT_BURST); \
		if (__ratelimit(&_rs)) { \
			mm_fb_audio(event, \
				MM_FB_KEY_RATELIMIT_30MIN, delay, level, fmt, ##__VA_ARGS__); \
		} \
	} while (0)

#define pr_err_vow(event, level, delay, fmt, ...) \
	do { \
		printk(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__); \
		vow_kernel_fb(event, level, delay, "payload@@"fmt, ##__VA_ARGS__); \
	} while (0)

#define pr_err_vow_fb(event, fmt, ...) \
	pr_err_vow(event, FB_HIGH, 0, fmt, ##__VA_ARGS__)

#endif /* OPLUS_VOW_KERNEL_FB_H */

