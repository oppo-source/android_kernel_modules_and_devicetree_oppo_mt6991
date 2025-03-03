// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 MediaTek Inc.
 */
#include <linux/arm-smccc.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/soc/mediatek/mtk_dvfsrc.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>
#include <linux/panic_notifier.h>
#include <linux/kdebug.h>

#define MTK_SIP_DVFSRC_START		0x01
#define VCOREFS_SMC_CMD_PAUSE_ENABLE	0x21

struct mtk_dvfsrc_start {
	struct device *dev;
};

static struct mtk_dvfsrc_start *dvfsrc_drv;

static int panic_pause_dvfsrc(struct notifier_block *this, unsigned long event, void *ptr)
{
	static atomic_t first_exception = ATOMIC_INIT(0);
	struct arm_smccc_res ares;

	if (dvfsrc_drv) {
		mtk_dvfsrc_send_request(dvfsrc_drv->dev->parent,
			MTK_DVFSRC_CMD_FORCEOPP_REQUEST, 0xDEAD);
	}

	if (atomic_cmpxchg(&first_exception, 0, 1) != 0)
		return NOTIFY_DONE;

	arm_smccc_smc(MTK_SIP_VCOREFS_CONTROL, VCOREFS_SMC_CMD_PAUSE_ENABLE, 1, 0, 0,
		0, 0, 0, &ares);

	return NOTIFY_DONE;
}

static struct notifier_block panic_blk = {
	.notifier_call = panic_pause_dvfsrc,
	.priority = INT_MAX,
};

static struct notifier_block die_blk = {
	.notifier_call = panic_pause_dvfsrc,
	.priority = INT_MAX,
};

/* We let dvfsrc working on parent main driver and setup
 * framework (interconnect, regulator, ..) for user register
 * and request, so we will lock high opp in that stage.
 * We will release this lock in this driver and let probe on
 * late_initsync stage. It send smc cmd MTK_SIP_DVFSRC_RUN
 * to release lock let dvfsrc free run.
 */

static int mtk_dvfsrc_start_probe(struct platform_device *pdev)
{
	struct arm_smccc_res ares;
	struct mtk_dvfsrc_start *dvfsrc;

	dvfsrc = devm_kzalloc(&pdev->dev, sizeof(*dvfsrc), GFP_KERNEL);
	if (!dvfsrc)
		return -ENOMEM;

	dvfsrc->dev = &pdev->dev;
	dvfsrc_drv = dvfsrc;
	arm_smccc_smc(MTK_SIP_VCOREFS_CONTROL, MTK_SIP_DVFSRC_START, 0, 0, 0,
		0, 0, 0, &ares);
	atomic_notifier_chain_register(&panic_notifier_list, &panic_blk);
	register_die_notifier(&die_blk);

	return 0;
}

static struct platform_driver mtk_dvfsrc_run_drv = {
	.probe	= mtk_dvfsrc_start_probe,
	.driver = {
		.name	= "mtk-dvfsrc-start",
	},
};

static int __init mtk_dvfsrc_run_init(void)
{
	return platform_driver_register(&mtk_dvfsrc_run_drv);
}
late_initcall_sync(mtk_dvfsrc_run_init);

static void __exit mtk_dvfsrc_run_exit(void)
{
	platform_driver_unregister(&mtk_dvfsrc_run_drv);
}
module_exit(mtk_dvfsrc_run_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MTK DVFSRC enable free run driver");
