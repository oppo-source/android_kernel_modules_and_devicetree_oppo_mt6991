/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2023 MediaTek Inc.
 */

#ifndef __SWPM_V6899_SUBSYS_H__
#define __SWPM_V6899_SUBSYS_H__

#define MAX_RECORD_CNT				(64)

#define DEFAULT_AVG_WINDOW		(50)

/* VPROC3 + VPROC2 + VPROC1 + VDRAM + VGPU + VCORE */
#define DEFAULT_LOG_MASK			(0x3F)

#define POWER_CHAR_SIZE				(256)
#define POWER_INDEX_CHAR_SIZE			(4096)

#define ALL_SWPM_TYPE				(0xFFFF)

#define MAX_POWER_NAME_LENGTH (16)

#define for_each_pwr_mtr(i)    for (i = 0; i < NR_SWPM_TYPE; i++)

enum swpm_type {
	CPU_SWPM_TYPE,
	GPU_SWPM_TYPE,
	CORE_SWPM_TYPE,
	MEM_SWPM_TYPE,
	ISP_SWPM_TYPE,
	APU_SWPM_TYPE,
	ME_SWPM_TYPE,
	AUDIO_SWPM_TYPE,
	TSFDC_SWPM_TYPE,
	DISP_SWPM_TYPE,
	USB_SWPM_TYPE,
	PCIE_SWPM_TYPE,
	VDEC_SWPM_TYPE,
	UFS_SWPM_TYPE,
	SSPM_SWPM_TYPE,
	SCP_SWPM_TYPE,
	DUMMY_SWPM_TYPE,
	INFRA_SWPM_TYPE,
	VCP_SWPM_TYPE,
	MML_SWPM_TYPE,
	MMINFRA_SWPM_TYPE,
	CLKMGR_SWPM_TYPE,
	VENC_SWPM_TYPE,
	NR_SWPM_TYPE,
};

enum power_rail {
	VPROC3,
	VPROC2,
	VPROC1,
	VGPU,
	VCORE,
	VDRAM,
	VIO18_DRAM,

	NR_POWER_RAIL
};

/* power rail */
struct power_rail_data {
	unsigned int avg_power;
	char name[MAX_POWER_NAME_LENGTH];
};

struct subsys_swpm_data {
	unsigned int mem_swpm_data_addr;
	unsigned int core_swpm_data_addr;
};

struct share_sub_index_ext {
};

extern struct power_rail_data swpm_power_rail[NR_POWER_RAIL];
extern void swpm_v6899_sub_ext_update(void);
extern void swpm_v6899_sub_ext_init(void);
extern void swpm_v6899_power_log(void);

#endif
