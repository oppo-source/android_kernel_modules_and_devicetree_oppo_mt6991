/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: Tiffany Lin <tiffany.lin@mediatek.com>
 */

#ifndef _MTK_VCODEC_ENC_PM_H_
#define _MTK_VCODEC_ENC_PM_H_

#include "mtk_vcodec_drv.h"

enum mtk_venc_hw_break_mode {
	MTK_VENC_HW_BREAK_PAUSE_MODE = 0,
	MTK_VENC_HW_BREAK_SMI_LOCK_MODE = 1,
};

void mtk_venc_init_ctx_pm(struct mtk_vcodec_ctx *ctx);
int mtk_vcodec_init_enc_pm(struct mtk_vcodec_dev *dev);
void mtk_vcodec_release_enc_pm(struct mtk_vcodec_dev *dev);
void mtk_venc_deinit_ctx_pm(struct mtk_vcodec_ctx *ctx);
void mtk_vcodec_enc_pw_on(struct mtk_vcodec_pm *pm);
void mtk_vcodec_enc_pw_off(struct mtk_vcodec_pm *pm);
void mtk_vcodec_enc_clock_on(struct mtk_vcodec_ctx *ctx, int core_id);
void mtk_vcodec_enc_clock_off(struct mtk_vcodec_ctx *ctx, int core_id);

void mtk_vcodec_enc_smi_pwr_ctrl_register(struct mtk_vcodec_dev *dev);
void mtk_vcodec_enc_smi_pwr_ctrl_unregister(struct mtk_vcodec_dev *dev);

void mtk_venc_translation_fault_callback_setting(struct mtk_vcodec_dev *dev);
void mtk_venc_violation_fault_callback_setting(struct mtk_vcodec_dev *dev);
extern void mtk_venc_do_gettimeofday(struct timespec64 *tv);

#endif /* _MTK_VCODEC_ENC_PM_H_ */
