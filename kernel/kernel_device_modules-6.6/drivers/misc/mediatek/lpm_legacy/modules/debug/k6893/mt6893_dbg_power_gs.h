/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */


#ifndef __MT6893_PWR_GS_COMPARE_H__
#define __MT6893_PWR_GS_COMPARE_H__

int mt6893_power_gs_init(void);
void mt6893_power_gs_deinit(void);

//dump dcm and cg golden settings
#define MTK_LPM_GS_PLAT_CLK_DUMP_SUPPORT

#endif
