/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2018 MediaTek Inc.
 *
 * Author: Christopher Chen <christopher.chen@mediatek.com>
 *
 */

#ifndef _MTK_IMGSYS_ENGINE_H_
#define _MTK_IMGSYS_ENGINE_H_
/**
 * enum Pseudo_Desc_Extra_Info
 *
 * Definition about Pseudo Descriptor special Extra Info
 * Distinguish Desc Type (NONE/CTRL/TUNING)
 * align with hw_definition in userspace
 */
enum Pseudo_Desc_Extra_Info {
	PSEUDO_DESC_NON = 0,
	PSEUDO_DESC_TUNING = 0x10,
	PSEUDO_DESC_CTRL = 0x20,
};

/**
 * enum mtk_imgsys_module
 *
 * Definition about supported hw modules
 */
enum mtk_imgsys_module {
	IMGSYS_MOD_IMGMAIN = 0, /*pure sw, debug dump usage*/
	IMGSYS_MOD_WPE,
	IMGSYS_MOD_OMC,
	IMGSYS_MOD_ADL,
	IMGSYS_MOD_TRAW,
	IMGSYS_MOD_LTRAW,
	IMGSYS_MOD_DIP,
	IMGSYS_MOD_PQDIP,
	IMGSYS_MOD_ME,
	IMGSYS_MOD_MAX,
};

/**
 * enum mtk_imgsys_engine
 *
 * Definition about supported hw engines
 */
enum mtk_imgsys_engine {
	IMGSYS_ENG_WPE_EIS	= 0x00000001,
	IMGSYS_ENG_WPE_TNR	= 0x00000002,
	IMGSYS_ENG_WPE_LITE	= 0x00000004,
	IMGSYS_ENG_OMC_TNR	= 0x00000008,
	IMGSYS_ENG_OMC_LITE	= 0x00000010,
	IMGSYS_ENG_ADL_A		= 0x00000020,
	IMGSYS_ENG_ADL_B		= 0x00000040,
	IMGSYS_ENG_TRAW			= 0x00000080,
	IMGSYS_ENG_LTR			= 0x00000100,
	IMGSYS_ENG_XTR			= 0x00000200,
	IMGSYS_ENG_DIP			= 0x00000400,
	IMGSYS_ENG_PQDIP_A	= 0x00000800,
	IMGSYS_ENG_PQDIP_B	= 0x00001000,
	IMGSYS_ENG_ME				= 0x00002000,
	IMGSYS_ENG_NUM			= 14,
};

/**
 * enum IMGSYS_REG_MAP_E
 *
 * Definition about hw register map id
 * The engine order should be the same as register order in dts
 */
enum IMGSYS_REG_MAP_E {
	REG_MAP_E_TOP = 0,
	REG_MAP_E_TRAW,
	REG_MAP_E_LTRAW,
	REG_MAP_E_DIP,
	REG_MAP_E_DIP_NR1,
	REG_MAP_E_DIP_NR2,
	REG_MAP_E_PQDIP_A,
	REG_MAP_E_PQDIP_B,
	REG_MAP_E_WPE_EIS,
	REG_MAP_E_WPE_TNR,
	REG_MAP_E_WPE_LITE,
	REG_MAP_E_WPE1_DIP1,
	REG_MAP_E_OMC_TNR,
	REG_MAP_E_OMC_LITE,
	REG_MAP_E_ME,
	REG_MAP_E_ADL_A,
	REG_MAP_E_ADL_B,
	REG_MAP_E_WPE2_DIP1,
	REG_MAP_E_WPE3_DIP1,
	REG_MAP_E_DIP_TOP,
	REG_MAP_E_DIP_TOP_NR,
	REG_MAP_E_DIP_TOP_NR2,
	REG_MAP_E_TRAW_TOP,
	REG_MAP_E_ME_MMG,
	REG_MAP_E_IMG_VCORE,
};

#endif /* _MTK_IMGSYS_ENGINE_H_ */

