/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2018 MediaTek Inc.
 *
 * Author: Christopher Chen <christopher.chen@mediatek.com>
 *
 */

#ifndef _MTK_IMGSYS_DEBUG_H_
#define _MTK_IMGSYS_DEBUG_H_

#include <mtk_imgsys-dev.h>
#include <mtk_imgsys-cmdq.h>
#include <mtk_imgsys-module.h>
#include <mtk_imgsys-engine.h>
#include <mtk_imgsys-isp-debug.h>
/**
 * enum IMGSYS_DL_PATH_E
 *
 * Definition about supported direct link path
 */
enum IMGSYS_DL_PATH_E {
	IMGSYS_DL_WPEE_TRAW = 0,
	IMGSYS_DL_WPEE_DIP = 1,
	IMGSYS_DL_TRAW_DIP = 4,
	IMGSYS_DL_WPE_PQDIP = 5, /*no more used*/
	IMGSYS_DL_DIP_PQDIPA = 6,
	IMGSYS_DL_DIP_PQDIPB = 7,
	IMGSYS_DL_WPET_TRAW = 10,
	IMGSYS_DL_WPET_DIP = 11,
	IMGSYS_DL_ADLA_LTRAW = 14,
	IMGSYS_DL_ADLB_LTRAW = 15,
	IMGSYS_DL_ADLA_ODD_LTRAW = 18,
	IMGSYS_DL_WPEE_XTRAW = 20,
	IMGSYS_DL_WPET_XTRAW = 21,
	IMGSYS_DL_WPEL_TRAW, /*no this*/
};

struct imgsys_dbg_engine_t {
	enum mtk_imgsys_engine eng_e;
	char eng_name[8];
};

void imgsys_dl_debug_dump(struct mtk_imgsys_dev *imgsys_dev, unsigned int hw_comb);
void imgsys_debug_dump_routine(struct mtk_imgsys_dev *imgsys_dev,
	const struct module_ops *imgsys_modules, int imgsys_module_num,
	unsigned int hw_comb);
void imgsys_main_init(struct mtk_imgsys_dev *imgsys_dev);
void imgsys_main_set_init(struct mtk_imgsys_dev *imgsys_dev);
void imgsys_main_cmdq_set_init(struct mtk_imgsys_dev *imgsys_dev, void *pkt, int hw_idx);
void imgsys_main_uninit(struct mtk_imgsys_dev *imgsys_dev);

bool imgsys_dip_7sp_dbg_enable(void);
bool imgsys_traw_7sp_dbg_enable(void);
bool imgsys_wpe_7sp_dbg_enable(void);
bool imgsys_pqdip_7sp_dbg_enable(void);
bool imgsys_me_7sp_dbg_enable(void);

#endif /* _MTK_IMGSYS_DEBUG_H_ */
