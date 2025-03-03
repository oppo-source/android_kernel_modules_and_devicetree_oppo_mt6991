/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef __LAST_BUS_H__
#define __LAST_BUS_H__

#define DUMP_BUFF_SIZE            0x10000
#define TIMEOUT_THRES_LOWEST      0x1
#define TIMEOUT_THRES_LIMIT       0xFFFF
#define LASTBUS_BUF_LENGTH        0x4000
#define MS_US_TRANSLATE           1000
#define NR_MAX_LASTBUS_MONITOR    32
#define MAX_MONITOR_NAME_LEN      40
#define NR_MAX_LASTBUS_IDLE_MASK  8

#define TIMEOUT_THRES_SHIFT       16
#define TIMEOUT_TYPE_SHIFT        1

#define LASTBUS_TIMEOUT_CLR       0x0200
#define LASTBUS_DEBUG_CKEN        0x0008
#define LASTBUS_DEBUG_EN          0x0004
#define LASTBUS_TIMEOUT           0x0001


enum LASTBUS_SW_VERSION {
	LASTBUS_SW_V1 = 1,
	LASTBUS_SW_V2 = 2,
};

enum LASTBUS_TIMEOUT_TYPE {
	LASTBUS_TIMEOUT_FIRST = 0,
	LASTBUS_TIMEOUT_LAST = 1,
};

enum LASTBUS_MODE {
	TIMEOUT_DUMP = 0,
	FORCE_DUMP,
	FORCE_CLEAR,
	GET_PER_TMO,
	REDUCE_TMO,
	TOTAL_MODE,
};

struct lastbus_idle_mask {
	unsigned int reg_offset;
	unsigned int reg_value;
};

struct lastbus_monitor {
	char name[MAX_MONITOR_NAME_LEN];
	unsigned int base;
	unsigned int num_ports;
	int bus_freq_mhz;
	unsigned int idle_mask_en;
	unsigned int num_idle_mask;
	struct lastbus_idle_mask idle_masks[NR_MAX_LASTBUS_IDLE_MASK];
	unsigned int num_bus_status;
	unsigned int offset_bus_status;
	unsigned int isr_dump;
};

struct cfg_lastbus {
	unsigned int sw_version;
	unsigned int enabled;
	unsigned int timeout_ms;
	unsigned int timeout_type;
	unsigned int num_used_monitors;
	struct lastbus_monitor monitors[NR_MAX_LASTBUS_MONITOR];
};
int lastbus_dump(int force_dump);
#endif
