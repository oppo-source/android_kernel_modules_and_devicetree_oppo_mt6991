// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/types.h>
#include <linux/dma-direct.h>
#include <linux/rpmsg.h>

#include "apusys_core.h"
#include "mdw_cmd.h"
#include "mdw_mem.h"
#include "mdw_mem_pool.h"
#include "mdw_ext_export.h"
#include "mdw_trace.h"

struct mdw_device *mdw_dev;
static struct apusys_core_info *g_info;
static atomic_t g_inited;

static void mdw_drv_priv_delete(struct kref *ref)
{
	struct mdw_fpriv *mpriv =
			container_of(ref, struct mdw_fpriv, ref);

	mdw_drv_debug("mpriv(0x%llx) free\n", (uint64_t) mpriv);
	mdw_dev_session_delete(mpriv);
	kfree(mpriv);
}

static void mdw_drv_priv_get(struct mdw_fpriv *mpriv)
{
	mdw_flw_debug("mpriv(0x%llx) ref(%u)\n",
		(uint64_t) mpriv, kref_read(&mpriv->ref));
	kref_get(&mpriv->ref);
}

static void mdw_drv_priv_put(struct mdw_fpriv *mpriv)
{
	mdw_flw_debug("mpriv(0x%llx) ref(%u)\n",
		(uint64_t) mpriv, kref_read(&mpriv->ref));
	kref_put(&mpriv->ref, mdw_drv_priv_delete);
}

static int mdw_drv_open(struct inode *inode, struct file *filp)
{
	struct mdw_fpriv *mpriv = NULL;
	int ret = 0;

	mdw_trace_begin("apumdw:drv_open");
	if (!mdw_dev) {
		pr_info("apusys/mdw: apu mdw no dev\n");
		return -ENODEV;
	}

	if (mdw_dev->inited == false) {
		mdw_drv_warn("apu mdw dev not init");
		return -EBUSY;
	}

	mpriv = kzalloc(sizeof(*mpriv), GFP_KERNEL);
	if (!mpriv)
		return -ENOMEM;

	mpriv->mdev = mdw_dev;
	filp->private_data = mpriv;
	atomic_set(&mpriv->active, 1);
	mutex_init(&mpriv->mtx);
	mutex_init(&mpriv->ch_mtx);
	INIT_LIST_HEAD(&mpriv->mems);
	INIT_LIST_HEAD(&mpriv->invokes);
	INIT_LIST_HEAD(&mpriv->ch_list);
	atomic_set(&mpriv->active_cmds, 0);
	idr_init(&mpriv->cmds);
	INIT_LIST_HEAD(&mpriv->cmds_list);
	atomic_set(&mpriv->exec_seqno, 0);

	mpriv->get = mdw_drv_priv_get;
	mpriv->put = mdw_drv_priv_put;
	kref_init(&mpriv->ref);

	if (!atomic_read(&g_inited)) {
		ret = mdw_dev->plat_funcs->sw_init(mdw_dev);
		if (ret) {
			mdw_drv_err("mdw sw init fail(%d)\n", ret);
			goto put_mpriv;
		}
		atomic_inc(&g_inited);
	}

	/* get normal power budget */
	if (mdw_dev->plat_funcs->pb_get != NULL)
		mdw_dev->plat_funcs->pb_get(MDW_POWERPOLICY_DEFAULT, MDW_PB_DEBOUNCE_MS);

	ret = mdw_mem_pool_create(mpriv, &mpriv->cmd_buf_pool,
		MDW_MEM_TYPE_MAIN, MDW_MEM_POOL_CHUNK_SIZE,
		MDW_DEFAULT_ALIGN, F_MDW_MEM_32BIT);
	if (ret) {
		mdw_drv_err("mdw create mem pool fail ret(%d)\n", ret);
		goto put_mpriv;
	}

	mdw_dev_session_create(mpriv);
	mdw_flw_debug("mpriv(0x%lx)\n", (unsigned long)mpriv);
	mdw_trace_end();
	goto out;

put_mpriv:
	mpriv->put(mpriv);
out:
	return ret;
}

static int mdw_drv_close(struct inode *inode, struct file *filp)
{
	struct mdw_fpriv *mpriv = NULL;

	mpriv = filp->private_data;
	mdw_flw_debug("mpriv(0x%llx)\n", (uint64_t)mpriv);
	mutex_lock(&mpriv->mtx);
	atomic_set(&mpriv->active, 0);
	mpriv->mdev->plat_funcs->release_cmd(mpriv);
	mdw_mem_pool_destroy(&mpriv->cmd_buf_pool);
	mutex_unlock(&mpriv->mtx);
	mpriv->put(mpriv);

	return 0;
}

static const struct file_operations mdw_fops = {
	.owner = THIS_MODULE,
	.open = mdw_drv_open,
	.release = mdw_drv_close,
	.unlocked_ioctl = mdw_ioctl,
	.compat_ioctl = mdw_ioctl,
};

static struct miscdevice mdw_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = MDW_NAME,
	.fops = &mdw_fops,
};

//----------------------------------------
static int mdw_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mdw_device *mdev = NULL;
	int ret = 0;

	if (mdw_dev) {
		pr_info("%s already probe\n", __func__);
		return -EBUSY;
	}

	mdev = kzalloc(sizeof(*mdev), GFP_KERNEL);
	if (!mdev)
		return -ENOMEM;

	/* get parameter from dts */
	of_property_read_u32(pdev->dev.of_node, "version", &mdev->mdw_ver);
	of_property_read_u32(pdev->dev.of_node, "dsp_mask", &mdev->dsp_mask);
	of_property_read_u32(pdev->dev.of_node, "dla_mask", &mdev->dla_mask);
	of_property_read_u32(pdev->dev.of_node, "dma_mask", &mdev->dma_mask);
	mdev->pdev = pdev;
	mdev->driver_type = MDW_DRIVER_TYPE_PLATFORM;
	mdev->misc_dev = &mdw_misc_dev;
	mdw_dev = mdev;
	platform_set_drvdata(pdev, mdev);
	atomic_set(&mdev->cmd_running, 0);

	ret = mdw_mem_init(mdev);
	if (ret)
		goto delete_mdw_dev;

	ret = mdw_sysfs_init(mdev);
	if (ret)
		goto deinit_mem;

	mdw_dbg_init(g_info);

	ret = mdw_dev_init(dev, mdev);
	if (ret)
		goto deinit_dbg;

	mdw_cmd_history_init(mdev);

	mdev->support_power_fast_on_off = false;

	pr_info("%s +\n", __func__);

	goto out;

deinit_dbg:
	mdw_dbg_deinit();
	mdw_sysfs_deinit(mdev);
deinit_mem:
	mdw_mem_deinit(mdev);
delete_mdw_dev:
	kfree(mdev);
	mdw_dev = NULL;
out:
	return ret;
}

static int mdw_platform_remove(struct platform_device *pdev)
{
	struct mdw_device *mdev = platform_get_drvdata(pdev);

	mdev->plat_funcs->sw_deinit(mdev);
	mdw_dev_deinit(mdev);
	mdw_cmd_history_deinit(mdev);
	mdw_dbg_deinit();
	mdw_sysfs_deinit(mdev);
	mdw_mem_deinit(mdev);
	kfree(mdev);
	mdw_dev = NULL;
	pr_info("%s +\n", __func__);

	return 0;
}

static const struct of_device_id mdw_of_match[] = {
	{ .compatible = "mediatek, apu_mdw", .data = &ap_plat_drv_v1},
	{},
};

static struct platform_driver mdw_platform_driver = {
	.driver = {
		.name = "apusys",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(mdw_of_match),
	},
	.probe = mdw_platform_probe,
	.remove = mdw_platform_remove,
};

//----------------------------------------
static int mdw_rpmsg_probe(struct rpmsg_device *rpdev)
{
	struct device *dev = &rpdev->dev;
	struct mdw_device *mdev = NULL;
	int ret = 0;

	pr_info("%s +\n", __func__);

	if (mdw_dev) {
		pr_info("%s already probe\n", __func__);
		return -EBUSY;
	}
	mdev = kzalloc(sizeof(*mdev), GFP_KERNEL);
	if (!mdev)
		return -ENOMEM;

	/* get parameter from dts */
	of_property_read_u32(rpdev->dev.of_node, "version", &mdev->mdw_ver);
	of_property_read_u32(rpdev->dev.of_node, "dsp_mask", &mdev->dsp_mask);
	of_property_read_u32(rpdev->dev.of_node, "dla_mask", &mdev->dla_mask);
	of_property_read_u32(rpdev->dev.of_node, "dma_mask", &mdev->dma_mask);

	mdev->driver_type = MDW_DRIVER_TYPE_RPMSG;
	mdev->rpdev = rpdev;
	mdev->misc_dev = &mdw_misc_dev;
	mdw_dev = mdev;
	dev_set_drvdata(dev, mdev);

	ret = mdw_mem_init(mdev);
	if (ret)
		goto delete_mdw_dev;

	ret = mdw_sysfs_init(mdev);
	if (ret)
		goto deinit_mem;

	mdw_dbg_init(g_info);

	ret = mdw_dev_init(dev, mdev);
	if (ret)
		goto deinit_dbg;

	mdw_cmd_history_init(mdev);

	mdev->support_power_fast_on_off = true;
	mdev->power_state = MDW_APU_POWER_OFF;

	pr_info("%s -\n", __func__);

	goto out;

deinit_dbg:
	mdw_dbg_deinit();
	mdw_sysfs_deinit(mdev);
deinit_mem:
	mdw_mem_deinit(mdev);
delete_mdw_dev:
	kfree(mdev);
	mdw_dev = NULL;
out:
	return ret;
}

static void mdw_rpmsg_remove(struct rpmsg_device *rpdev)
{
	struct mdw_device *mdev = dev_get_drvdata(&rpdev->dev);

	mdev->plat_funcs->sw_deinit(mdev);
	mdw_dev_deinit(mdev);
	mdw_cmd_history_deinit(mdev);
	mdw_dbg_deinit();
	mdw_sysfs_deinit(mdev);
	mdw_mem_deinit(mdev);
	kfree(mdev);
	mdw_dev = NULL;
	pr_info("%s +\n", __func__);
}

static const struct of_device_id mdw_rpmsg_of_match[] = {
	{ .compatible = "mediatek,apu-mdw-rpmsg-v2", .data = &rv_plat_drv_v2},
	{ .compatible = "mediatek,apu-mdw-rpmsg-v3", .data = &rv_plat_drv_v3},
	{ .compatible = "mediatek,apu-mdw-rpmsg-v4", .data = &rv_plat_drv_v4},
	{ .compatible = "mediatek,apu-mdw-rpmsg-v5", .data = &rv_plat_drv_v4},
	{ },
};

static struct rpmsg_driver mdw_rpmsg_driver = {
	.drv = {
		.name = "apu-mdw-rpmsg",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(mdw_rpmsg_of_match),
	},
	.probe = mdw_rpmsg_probe,
	.remove = mdw_rpmsg_remove,
};

//----------------------------------------
int mdw_init(struct apusys_core_info *info)
{
	int ret = 0;

	g_info = info;

	if (!mdw_pwr_check()) {
		pr_info("apusys mdw disable\n");
		return -ENODEV;
	}

	pr_info("%s register misc...\n", __func__);
	ret = misc_register(&mdw_misc_dev);
	if (ret) {
		pr_info("failed to register apu mdw misc driver\n");
		goto out;
	}

	pr_info("%s register platorm...\n", __func__);
	ret = platform_driver_register(&mdw_platform_driver);
	if (ret) {
		pr_info("failed to register apu mdw driver\n");
		goto unregister_misc_dev;
	}

	pr_info("%s register rpmsg...\n", __func__);
	ret = register_rpmsg_driver(&mdw_rpmsg_driver);
	if (ret) {
		pr_info("failed to register apu mdw rpmsg driver\n");
		goto unregister_platform_driver;
	}

	/* init apu ext function */
	ret = mdw_ext_init(mdw_dev);
	if (ret) {
		pr_info("failed to do ext init\n");
		goto unregister_rpmsg_driver;
	}

	pr_info("%s init done\n", __func__);
	goto out;

unregister_rpmsg_driver:
	unregister_rpmsg_driver(&mdw_rpmsg_driver);
unregister_platform_driver:
	platform_driver_unregister(&mdw_platform_driver);
unregister_misc_dev:
	misc_deregister(&mdw_misc_dev);
out:
	return ret;
}

void mdw_exit(void)
{
	mdw_ext_deinit();
	unregister_rpmsg_driver(&mdw_rpmsg_driver);
	platform_driver_unregister(&mdw_platform_driver);
	misc_deregister(&mdw_misc_dev);
	g_info = NULL;
}
