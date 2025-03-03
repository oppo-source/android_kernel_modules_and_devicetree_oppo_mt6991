/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __VCP_FEATURE_DEFINE_H__
#define __VCP_FEATURE_DEFINE_H__

#include "vcp.h"

/* vcp platform configs*/
#define VCP_BOOT_TIME_OUT_MONITOR        (1)
#define VCP_RESERVED_MEM                 (1)
#define VCP_LOGGER_ENABLE                (1)
#define VCP_VOW_LOW_POWER_MODE           (1)

/* vcp rescovery feature option*/
#define VCP_RECOVERY_SUPPORT             (1)
/* vcp recovery timeout value (ms)*/
#define VCP_SYS_RESET_TIMEOUT            1000

#define VCP_PARAMS_TO_VCP_SUPPORT

/* vcp aed definition*/
#define VCP_AED_STR_LEN                  (512)
#define VCP_CHECK_AED_STR_LEN(func, offset) ({\
	int ret; ret = func; ((ret > 0) && ((ret + offset) < (VCP_AED_STR_LEN - 1))) ? ret : 0; })
/* vcp sub feature register API marco*/
#define VCP_REGISTER_SUB_SENSOR          (1)

#define VCPSYS_CORE0                     0
#define VCPSYS_CORE1                     1

/* vcp Core ID definition */
enum vcp_core_id {
	VCP_A_ID = 0,
	VCP_CORE_TOTAL = 1,
};

struct vcp_feature_tb {
	uint32_t feature;
	uint32_t freq;
	uint32_t enable;
	uint32_t sys_id; /* run at which subsys? */
};

struct vcp_sub_feature_tb {
	uint32_t feature;
	uint32_t freq;
	uint32_t enable;
};

extern struct vcp_feature_tb feature_table[NUM_FEATURE_ID];

#endif


