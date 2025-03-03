// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include "generic_debugfs.h"
#include <linux/extdev_io_class.h>

struct mt6370_dbg_info {
	struct device *dev;
	struct extdev_desc extdev_desc;
	struct extdev_io_device *extdev;
	struct dbg_info dbg_info;
};

static int mt6370_dbg_io_read(void *drvdata, u16 reg, void *val, u16 size)
{
	return regmap_bulk_read((struct regmap *)drvdata, reg, val, size);
}

static int mt6370_dbg_io_write(void *drvdata,
			       u16 reg, const void *val, u16 size)
{
	return regmap_bulk_write((struct regmap *)drvdata, reg, val, size);
}

static int mt6370_dbg_probe(struct platform_device *pdev)
{
	struct mt6370_dbg_info *mdi;
	struct regmap *regmap;
	int ret;

	mdi = devm_kzalloc(&pdev->dev, sizeof(*mdi), GFP_KERNEL);
	if (!mdi)
		return -ENOMEM;

	mdi->dev = &pdev->dev;
	platform_set_drvdata(pdev, mdi);

	regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!regmap) {
		dev_err(&pdev->dev, "failed to allocate regmap\n");
		return -ENODEV;
	}

	/* sysfs interface */
	mdi->extdev_desc.dirname = devm_kasprintf(&pdev->dev,
					       GFP_KERNEL, "MT6370.%s",
					       dev_name(pdev->dev.parent));
	mdi->extdev_desc.devname = dev_name(pdev->dev.parent);
	mdi->extdev_desc.typestr = devm_kasprintf(&pdev->dev,
					       GFP_KERNEL, "I2C,MT6370");
	mdi->extdev_desc.rmap = regmap;
	mdi->extdev_desc.io_read = mt6370_dbg_io_read;
	mdi->extdev_desc.io_write = mt6370_dbg_io_write;
	mdi->extdev = devm_extdev_io_device_register(pdev->dev.parent, &mdi->extdev_desc);
	if (IS_ERR(mdi->extdev)) {
		dev_err(&pdev->dev, "%s:failed to register extdev_io device\n", __func__);
		return PTR_ERR(mdi->extdev);
	}

	/* debugfs interface */
	mdi->dbg_info.dirname = devm_kasprintf(&pdev->dev,
					       GFP_KERNEL, "MT6370.%s",
					       dev_name(pdev->dev.parent));
	mdi->dbg_info.devname = dev_name(pdev->dev.parent);
	mdi->dbg_info.typestr = devm_kasprintf(&pdev->dev,
					       GFP_KERNEL, "I2C,MT6370");
	mdi->dbg_info.io_drvdata = regmap;
	mdi->dbg_info.io_read = mt6370_dbg_io_read;
	mdi->dbg_info.io_write = mt6370_dbg_io_write;
	ret = generic_debugfs_register(&mdi->dbg_info);
	if (ret < 0)
		return ret;

	dev_info(&pdev->dev, "%s: successfully\n", __func__);
	return 0;
}

static int mt6370_dbg_remove(struct platform_device *pdev)
{
	struct mt6370_dbg_info *mdi = platform_get_drvdata(pdev);

	generic_debugfs_unregister(&mdi->dbg_info);
	return 0;
}

static const struct platform_device_id mt6370_dbg_id[] = {
	{ "mt6370-dbg", 0 },
	{ },
};
MODULE_DEVICE_TABLE(platform, mt6370_dbg_id);

static const struct of_device_id mt6370_dbg_of_id[] = {
	{ .compatible = "mediatek,mt6370-dbg", },
	{ },
};
MODULE_DEVICE_TABLE(of, mt6370_dbg_of_id);

static struct platform_driver mt6370_dbg_driver = {
	.driver = {
		.name = "mt6370_dbg",
		.of_match_table = of_match_ptr(mt6370_dbg_of_id),
	},
	.probe = mt6370_dbg_probe,
	.remove = mt6370_dbg_remove,
	.id_table = mt6370_dbg_id,
};
module_platform_driver(mt6370_dbg_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("cy_huang <cy_huang@richtek.com>");
