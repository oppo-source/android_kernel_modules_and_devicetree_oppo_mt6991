/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2025 OPLUS Inc.
 */
#ifndef __OPLUS_CAM_MONITOR_DEFINE_H__
#define __OPLUS_CAM_MONITOR_DEFINE_H__


#define NAME_STRING_MAX_LENGTH   32
#ifndef CAM_MONITOR_DISABLE
	#define CAM_MONITOR_DISABLE 0
#endif
#ifndef CAM_MONITOR_ENABLE
	#define CAM_MONITOR_ENABLE 1
#endif

struct camera_monitor_regulator_info {
	char module_name[NAME_STRING_MAX_LENGTH];
	char resource_name[NAME_STRING_MAX_LENGTH];
	int state;
	int  voltage[2];
	struct timespec64 ts;
};

struct camera_monitor_clock_info {
	char module_name[NAME_STRING_MAX_LENGTH];
	char resource_name[NAME_STRING_MAX_LENGTH];
	int state;
	int  value[2];
	struct timespec64 ts;
};

struct camera_monitor_gpio_info {
	char module_name[NAME_STRING_MAX_LENGTH];
	char resource_name[NAME_STRING_MAX_LENGTH];
	int state;
	int  value;
	struct timespec64 ts;
};

void oplus_cam_monitor_regulator_notify(const char *module_name,
	const char *resource_name, int state, int voltage_min, int voltage_max);
void oplus_cam_monitor_clock_notify(const char *module_name,
	const char *resource_name, int state, int value0, int value1);
void oplus_cam_monitor_gpio_notify(const char *module_name,
	const char *resource_name, int state, int value);

#endif /* __OPLUS_CAM_MONITOR_DEFINE_H__ */
