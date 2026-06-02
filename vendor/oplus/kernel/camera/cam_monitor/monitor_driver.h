/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2025 OPLUS Inc.
 */
#ifndef __CAMERA_MONITOR_H__
#define __CAMERA_MONITOR_H__

#include <linux/miscdevice.h>
#include <linux/semaphore.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <linux/rtc.h>
#include <linux/regulator/driver.h>
#include "oplus_cam_monitor_define.h"


#define MODULE_NAME                         "cam_monitor"
#define CAM_MONITOR_STATE_HISTORY_MAX		512
#define ITEM_INFO_STRING_MAX_LEN            256
#define SUMMARY_INFO_STRING_MAX_LEN         1024
#define INC_INDEX(index, max_entries, ret)  div_u64_rem(atomic64_fetch_add(1, index), max_entries, (ret))
#define GET_INDEX(index, max_entries, ret)  div_u64_rem(atomic64_read(index), max_entries, (ret))

#define CAMMON_LOG_ERR(format, args...)									\
	({												\
		if(g_log_level >= LOG_LEVEL_ERR)							\
			pr_err(MODULE_NAME " [%s:%d] " format "\n", __func__, __LINE__, ##args);	\
	})

#define CAMMON_LOG_INF(format, args...)									\
	({												\
		if(g_log_level >= LOG_LEVEL_INF)							\
			pr_info(MODULE_NAME " [%s] " format "\n", __func__, ##args);			\
	})

#define CAMMON_LOG_DBG(format, args...)									\
	({												\
		if(g_log_level >= LOG_LEVEL_DBG)							\
			pr_info(MODULE_NAME " [%s] " format "\n", __func__, ##args);			\
	})

enum {
	LOG_LEVEL_ERR = 0,
	LOG_LEVEL_INF,
	LOG_LEVEL_DBG,
};

struct state_info {
	char info[ITEM_INFO_STRING_MAX_LEN];
	struct timespec64 timestamp;
};

struct monitor_history_data {
	atomic64_t  head_index;
	struct state_info info[CAM_MONITOR_STATE_HISTORY_MAX];
};

// important: sizeof(struct monitor_check_result) must less then PAGE_SIZE
struct monitor_check_result {
	char info[SUMMARY_INFO_STRING_MAX_LEN]; // "info" must be the first member of this structure
	char check_result;
};

struct camera_monitor_state_info_node {
	struct list_head list;
	char module_name[NAME_STRING_MAX_LENGTH];
	char resource_name[NAME_STRING_MAX_LENGTH];  /* power_name / clock_name / gpio_name */
	int  use_count;
};

struct camera_monitor_data {
	struct platform_device  *plat_dev;
	struct miscdevice       dev;
	struct kobject          *kobj;
	struct list_head        regulator_state_list;
	struct list_head        clock_state_list;
	struct list_head        gpio_state_list;
	struct mutex            regulator_list_lock;
	struct mutex            clock_list_lock;
	struct mutex            gpio_list_lock;
	struct mutex            history_data_lock;
	struct mutex            history_dump_lock;
	struct monitor_history_data  history_data;
};

#endif /* __CAMERA_MONITOR_H__ */
