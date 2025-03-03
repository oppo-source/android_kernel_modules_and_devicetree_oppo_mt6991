// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include "imgsensor_common.h"

#include <linux/clk.h>
#include "imgsensor_clk.h"

/*by platform settings and elements should not be reordered */
char *gimgsensor_mclk_name[IMGSENSOR_CCF_MAX_NUM] = {
	"CLK_TOP_CAMTG_SEL",
	"CLK_TOP_CAMTG1_SEL",
	"CLK_TOP_CAMTG2_SEL",
	"CLK_TOP_CAMTG3_SEL",
	"CLK_MCLK_6M",
	"CLK_MCLK_12M",
	"CLK_MCLK_13M",
	"CLK_MCLK_24M",
	"CLK_MCLK_26M",
	"CLK_MCLK_48M",
	"CLK_MCLK_52M",
	"CLK_CAM_SENINF_CG",
	"CLK_MIPI_C0_26M_CG",
	"CLK_MIPI_C1_26M_CG",
	"CLK_MIPI_ANA_0A_CG",
	"CLK_MIPI_ANA_0B_CG",
	"CLK_MIPI_ANA_1A_CG",
	"CLK_MIPI_ANA_1B_CG",
	"CLK_MIPI_ANA_2A_CG",
	"CLK_MIPI_ANA_2B_CG",
	"CLK_TOP_CAMTM_SEL_CG",
	"CLK_TOP_CAMTM_208_CG",
#ifndef SENINF_USE_RPM
	"CLK_SCP_SYS_CAM",
#endif
};


enum {
	MCLK_ENU_START,
	MCLK_6MHZ =	MCLK_ENU_START,
	MCLK_12MHZ,
	MCLK_13MHZ,
	MCLK_24MHZ,
	MCLK_26MHZ,
	MCLK_48MHZ,
	MCLK_52MHZ,
	MCLK_MAX,
};

enum {
	FREQ_6MHZ  =  6,
	FREQ_12MHZ = 12,
	FREQ_13MHZ = 13,
	FREQ_24MHZ = 24,
	FREQ_26MHZ = 26,
	FREQ_48MHZ = 48,
	FREQ_52MHZ = 52,
};

#ifdef IMGSENSOR_DFS_CTRL_ENABLE
int imgsensor_dfs_init(struct imgsensor_dfs_ctx *ctx, struct device *dev)
{
	int ret, i;
	struct dev_pm_opp *opp;
	unsigned long freq;
	unsigned int freq_hz;

	ctx->dev = dev;

	ret = dev_pm_opp_of_add_table(dev);
	if (ret < 0) {
		pr_info("fail to init opp table: %d\n", ret);
		return ret;
	}

	ctx->reg = devm_regulator_get_optional(dev, "dvfsrc-vcore");
	if (IS_ERR(ctx->reg)) {
		pr_info("can't get dvfsrc-vcore\n");
		return PTR_ERR(ctx->reg);
	}

	ctx->cnt = dev_pm_opp_get_opp_count(dev);

	ctx->freqs = devm_kzalloc(dev,
			sizeof(unsigned long) * ctx->cnt, GFP_KERNEL);
	ctx->volts = devm_kzalloc(dev,
			sizeof(unsigned long) * ctx->cnt, GFP_KERNEL);
	if (!ctx->freqs || !ctx->volts)
		return -ENOMEM;

	i = 0;
	freq = 0;
	while (!IS_ERR(opp = dev_pm_opp_find_freq_ceil(dev, &freq))) {
		freq_hz = freq;
		freq_hz /= 1000000; /*Hz->MHz*/
		ctx->freqs[ctx->cnt-1-i] = freq_hz;
		ctx->volts[ctx->cnt-1-i] = dev_pm_opp_get_voltage(opp);
		freq++;
		i++;
		dev_pm_opp_put(opp);
	}

	return 0;
}

void imgsensor_dfs_exit(struct imgsensor_dfs_ctx *ctx)
{
	dev_pm_opp_of_remove_table(ctx->dev);
}

int imgsensor_dfs_ctrl(struct imgsensor_dfs_ctx *ctx, enum DFS_OPTION option, void *pbuff)
{
	int i4RetValue = 0;

	if ((option == DFS_UPDATE ||
		option == DFS_SUPPORTED_ISP_CLOCKS ||
		option == DFS_CUR_ISP_CLOCK)) {
		if (pbuff == NULL) {
			pr_info("pbuff == null");
			return IMGSENSOR_RETURN_ERROR;
		}
	}

	switch (option) {
	case DFS_CTRL_ENABLE:
		break;
	case DFS_CTRL_DISABLE:
		break;
	case DFS_UPDATE:
	{
		unsigned long freq, volt;
		struct dev_pm_opp *opp;

		freq = *(unsigned int *)pbuff;
		freq = freq * 1000000; /*MHz->Hz*/
		opp = dev_pm_opp_find_freq_ceil(ctx->dev, &freq);
		if (IS_ERR(opp)) {
			pr_info("Failed to find OPP for frequency %lu: %ld\n",
				freq, PTR_ERR(opp));
			return -EFAULT;
		}
		volt = dev_pm_opp_get_voltage(opp);
		dev_pm_opp_put(opp);
		pr_debug("%s: freq=%ld Hz, volt=%ld\n", __func__, freq, volt);
		regulator_set_voltage(ctx->reg, volt, INT_MAX);
	}
		break;
	case DFS_RELEASE:
		break;
	case DFS_SUPPORTED_ISP_CLOCKS:
	{
		struct IMAGESENSOR_GET_SUPPORTED_ISP_CLK *pIspclks;
		int i;

		pIspclks = (struct IMAGESENSOR_GET_SUPPORTED_ISP_CLK *) pbuff;

		pIspclks->clklevelcnt = ctx->cnt;

		if (pIspclks->clklevelcnt > ISP_CLK_LEVEL_CNT) {
			pr_debug("ERR: clklevelcnt is exceeded\n");
			i4RetValue = -EFAULT;
			break;
		}

		for (i = 0; i < pIspclks->clklevelcnt; i++)
			pIspclks->clklevel[i] = ctx->freqs[i];
	}
		break;
	case DFS_CUR_ISP_CLOCK:
	{
		unsigned int *pGetIspclk;
		int i, cur_volt;

		pGetIspclk = (unsigned int *) pbuff;
		cur_volt = regulator_get_voltage(ctx->reg);

		for (i = 0; i < ctx->cnt; i++) {
			if (ctx->volts[i] == cur_volt) {
				*pGetIspclk = (u32)ctx->freqs[i];
				break;
			}
		}
	}
		break;
	default:
		pr_info("None\n");
		break;
	}
	return i4RetValue;
}
#endif
static inline void imgsensor_clk_check(struct IMGSENSOR_CLK *pclk)
{
	int i;

	for (i = 0; i < IMGSENSOR_CCF_MAX_NUM; i++) {
		if (IS_ERR(pclk->imgsensor_ccf[i]))
			pr_debug("%s fail %d %s",
				 __func__, i,
				 gimgsensor_mclk_name[i]);
		// else
			// pr_debug("%s %d %s",
				 // __func__, i,
				 // gimgsensor_mclk_name[i]);
	}
}

/************************************************************************
 * Common Clock Framework (CCF)
 ************************************************************************/
enum IMGSENSOR_RETURN imgsensor_clk_init(struct IMGSENSOR_CLK *pclk)
{
	int i;
	struct platform_device *pplatform_dev = gpimgsensor_hw_platform_device;

	if (pplatform_dev == NULL) {
		pr_info("[%s] pdev is null\n", __func__);
		return IMGSENSOR_RETURN_ERROR;
	}
	/* get all possible using clocks */
	for (i = 0; i < IMGSENSOR_CCF_MAX_NUM; i++)
		pclk->imgsensor_ccf[i] =
		    devm_clk_get(&pplatform_dev->dev, gimgsensor_mclk_name[i]);

#if IS_ENABLED(CONFIG_PM_SLEEP)
	pclk->seninf_wake_lock = wakeup_source_register(
			NULL, "seninf_lock_wakelock");
	if (!pclk->seninf_wake_lock) {
		pr_info("[%s] failed to get seninf_wake_lock\n", __func__);
		return IMGSENSOR_RETURN_ERROR;
	}
	pr_info("[%s] seninf_wake_lock register done\n", __func__);
#endif

	return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN imgsensor_clk_exit(struct IMGSENSOR_CLK *pclk)
{
#if IS_ENABLED(CONFIG_PM_SLEEP)
	if (!pclk->seninf_wake_lock) {
		pr_info("[%s] seninf_wake_lock is NULL\n", __func__);
	} else {
		wakeup_source_unregister(pclk->seninf_wake_lock);
		pr_info("[%s] seninf_wake_lock unregister done\n", __func__);
	}
#endif
	return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN seninf_wake_lock_get(struct IMGSENSOR_CLK *pclk)
{
#if IS_ENABLED(CONFIG_PM_SLEEP)
	if (!pclk->seninf_wake_lock) {
		pr_info("[%s] seninf_wake_lock is NULL\n", __func__);
	} else {
		__pm_stay_awake(pclk->seninf_wake_lock);
		pr_info("[%s] seninf_wake_lock stay_awake\n", __func__);
	}
#endif
	return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN seninf_wake_lock_put(struct IMGSENSOR_CLK *pclk)
{
#if IS_ENABLED(CONFIG_PM_SLEEP)
	if (!pclk->seninf_wake_lock) {
		pr_info("[%s] seninf_wake_lock is NULL\n", __func__);
	} else {
		__pm_relax(pclk->seninf_wake_lock);
		pr_info("[%s] seninf_wake_lock relax\n", __func__);
	}
#endif
	return IMGSENSOR_RETURN_SUCCESS;
}

int imgsensor_clk_set(
	struct IMGSENSOR_CLK *pclk, struct ACDK_SENSOR_MCLK_STRUCT *pmclk)
{
	int ret = 0;
	int mclk_index = MCLK_ENU_START;
	const int supported_mclk_freq[MCLK_MAX] = {
		FREQ_6MHZ, FREQ_12MHZ, FREQ_13MHZ, FREQ_24MHZ,
		FREQ_26MHZ, FREQ_48MHZ, FREQ_52MHZ };

	for (mclk_index = MCLK_ENU_START; mclk_index < MCLK_MAX; mclk_index++) {
		if (pmclk->freq == supported_mclk_freq[mclk_index])
			break;
	}
	if (pmclk->TG >= IMGSENSOR_CCF_MCLK_TG_MAX_NUM ||
		mclk_index == MCLK_MAX) {
		pr_info(
		    "[CAMERA SENSOR]kdSetSensorMclk out of range, tg=%d, freq= %d\n",
		    pmclk->TG,
		    pmclk->freq);

		return -EFAULT;
	}
	mclk_index += IMGSENSOR_CCF_MCLK_FREQ_MIN_NUM;
	imgsensor_clk_check(pclk);

	if (pmclk->on) {

		/* Workaround for timestamp: TG1 always ON */
		if (clk_prepare_enable(
		    pclk->imgsensor_ccf[IMGSENSOR_CCF_MCLK_TOP_CAMTG_SEL]))

			pr_info(
			    "[CAMERA SENSOR] failed tg=%d\n",
			    IMGSENSOR_CCF_MCLK_TOP_CAMTG_SEL);
		else
			atomic_inc(
			   &pclk->enable_cnt[IMGSENSOR_CCF_MCLK_TOP_CAMTG_SEL]);

		if (clk_prepare_enable(pclk->imgsensor_ccf[pmclk->TG]))
			pr_info("[CAMERA SENSOR] failed tg=%d\n", pmclk->TG);
		else
			atomic_inc(&pclk->enable_cnt[pmclk->TG]);

		if (clk_prepare_enable(pclk->imgsensor_ccf[mclk_index]))
			pr_info(
			    "[CAMERA SENSOR]imgsensor_ccf failed freq= %d, mclk_index %d\n",
			    pmclk->freq,
			    mclk_index);
		else
			atomic_inc(&pclk->enable_cnt[mclk_index]);

		ret = clk_set_parent(
		    pclk->imgsensor_ccf[pmclk->TG],
		    pclk->imgsensor_ccf[mclk_index]);

	} else {

		/* Workaround for timestamp: TG1 always ON */
		clk_disable_unprepare(
		    pclk->imgsensor_ccf[IMGSENSOR_CCF_MCLK_TOP_CAMTG_SEL]);

		atomic_dec(&pclk->enable_cnt[IMGSENSOR_CCF_MCLK_TOP_CAMTG_SEL]);

		clk_disable_unprepare(pclk->imgsensor_ccf[pmclk->TG]);
		atomic_dec(&pclk->enable_cnt[pmclk->TG]);
		clk_disable_unprepare(pclk->imgsensor_ccf[mclk_index]);
		atomic_dec(&pclk->enable_cnt[mclk_index]);
	}

	return ret;
}

void imgsensor_clk_enable_all(struct IMGSENSOR_CLK *pclk)
{
	int i;

	pr_info("imgsensor_clk_enable_all_cg\n");
	for (i = IMGSENSOR_CCF_MTCMOS_MIN_NUM;
		i < IMGSENSOR_CCF_MTCMOS_MAX_NUM;
		i++) {
		if (!IS_ERR(pclk->imgsensor_ccf[i])) {
			if (clk_prepare_enable(pclk->imgsensor_ccf[i]))
				pr_debug(
					"[CAMERA SENSOR]imgsensor_ccf enable cmos fail cg_index = %d\n",
					i);
			else
				atomic_inc(&pclk->enable_cnt[i]);
			/*pr_debug("imgsensor_clk_enable_all %s ok\n",*/
				/*gimgsensor_mclk_name[i]);*/
		}
	}
	for (i = IMGSENSOR_CCF_CG_MIN_NUM; i < IMGSENSOR_CCF_CG_MAX_NUM; i++) {
		if (!IS_ERR(pclk->imgsensor_ccf[i])) {
			if (clk_prepare_enable(pclk->imgsensor_ccf[i]))
				pr_debug(
					"[CAMERA SENSOR]imgsensor_ccf enable cg fail cg_index = %d\n",
					i);
			else
				atomic_inc(&pclk->enable_cnt[i]);
			/*pr_debug("imgsensor_clk_enable_all %s ok\n",*/
				/*gimgsensor_mclk_name[i]);*/

		}
	}
}

void imgsensor_clk_disable_all(struct IMGSENSOR_CLK *pclk)
{
	unsigned int i;

	pr_info("%s E\n", __func__);
	for (i = IMGSENSOR_CCF_MCLK_TG_MIN_NUM;
		i < IMGSENSOR_CCF_MAX_NUM;
		i++) {
		for (; !IS_ERR(pclk->imgsensor_ccf[i]) &&
			atomic_read(&pclk->enable_cnt[i]) > 0 ;) {
			clk_disable_unprepare(pclk->imgsensor_ccf[i]);
			atomic_dec(&pclk->enable_cnt[i]);
		}
	}
}

int imgsensor_clk_ioctrl_handler(void *pbuff)
{
	if (pbuff == NULL)
		pr_info(" %s pbuff == null", __func__);
	else
		*(unsigned int *)pbuff = mt_get_ckgen_freq(*(unsigned int *)pbuff);
	pr_info("hf_fcamtg_ck = %d, hf_fmm_ck = %d, f_fseninf_ck = %d\n",
		mt_get_ckgen_freq(7),
		mt_get_ckgen_freq(3),
		mt_get_ckgen_freq(27));
	return 0;
}
