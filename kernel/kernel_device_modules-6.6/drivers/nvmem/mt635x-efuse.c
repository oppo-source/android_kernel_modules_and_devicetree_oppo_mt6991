// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/delay.h>
#include <linux/mfd/mt6397/core.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-provider.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/printk.h>

/* PMIC EFUSE registers definition */
#define EFUSE_V0_TOP_CKPDN_CON0              (0x10c)
#define EFUSE_V0_TOP_CKHWEN_CON0             (0x12a)
#define EFUSE_V0_OTP_CON0                    (0x390)
#define EFUSE_V0_OTP_CON8                    (0x3a0)
#define EFUSE_V0_OTP_CON11                   (0x3a6)
#define EFUSE_V0_OTP_CON12                   (0x3a8)
#define EFUSE_V0_OTP_CON13                   (0x3aa)

#define EFUSE_V1_TOP_CKPDN_CON0              (0x10c)
#define EFUSE_V1_TOP_CKHWEN_CON0             (0x12a)
#define EFUSE_V1_OTP_CON0                    (0x38a)
#define EFUSE_V1_OTP_CON8                    (0x39a)
#define EFUSE_V1_OTP_CON11                   (0x3a0)
#define EFUSE_V1_OTP_CON12                   (0x3a2)
#define EFUSE_V1_OTP_CON13                   (0x3a4)

#define EFUSE_V2_TOP_CKPDN_CON1              (0x10f)
#define EFUSE_V2_TOP_CKHWEN_CON0             (0x121)
#define EFUSE_V2_OTP_CLK_CON0                (0x38a)
#define EFUSE_V2_OTP_CON0                    (0x38b)
#define EFUSE_V2_OTP_CON8                    (0x395)
#define EFUSE_V2_OTP_CON11                   (0x398)
#define EFUSE_V2_OTP_CON12_L                 (0x399)
#define EFUSE_V2_OTP_CON13                   (0x39b)

#define EFUSE_V3_TOP_CKPDN_CON1              (0x10f)
#define EFUSE_V3_TOP_CKHWEN_CON0             (0x121)
#define EFUSE_V3_OTP_CON0                    (0x38a)
#define EFUSE_V3_OTP_CON10                   (0x394)
#define EFUSE_V3_OTP_CON13                   (0x397)
#define EFUSE_V3_OTP_CON14                   (0x398)
#define EFUSE_V3_OTP_CON16                   (0x39a)

/* Mask definition for EFUSE control engine clock register */
#define RG_EFUSE_CK_PDN_HWEN_MASK	BIT(2)
#define RG_EFUSE_CK_PDN_MASK		BIT(4)
#define RG_OTP_RD_BUSY_MASK		BIT(0)
#define RG_OTP_OSC_CK_EN_MASK		0x3

/* Register SET/CLR offset */
#define SET_OFFSET	0x1
#define CLR_OFFSET	0x2

/* EFUSE Register width (bytes) definitions */
#define EFUSE_REG_WIDTH		2

/* Timeout (us) of polling the status */
#define EFUSE_POLL_TIMEOUT	30000
#define EFUSE_TRIG_DELAY_US	40
#define EFUSE_POLL_DELAY_US	50
#define EFUSE_READ_DELAY_US	30

#define EFUSE_ECID_SYSFS 1
#define EFUSE_ECID_SIZE 8
#define MAX_PMIC_SIZE 16

struct efuse_reg {
	unsigned int ck_pdn;
	unsigned int ck_pdn_mask;
	unsigned int ck_pdn_hwen;
	unsigned int ck_pdn_hwen_mask;
	unsigned int otp_osc_ck_en;
	unsigned int otp_pa;
	unsigned int otp_rd_trig;
	unsigned int otp_rd_sw;
	unsigned int otp_dout_sw;
	unsigned int otp_rd_busy;
};

static const struct efuse_reg reg_v0 = {
	.ck_pdn = EFUSE_V0_TOP_CKPDN_CON0,
	.ck_pdn_hwen = EFUSE_V0_TOP_CKHWEN_CON0,
	.otp_pa = EFUSE_V0_OTP_CON0,
	.otp_rd_trig = EFUSE_V0_OTP_CON8,
	.otp_rd_sw = EFUSE_V0_OTP_CON11,
	.otp_dout_sw = EFUSE_V0_OTP_CON12,
	.otp_rd_busy = EFUSE_V0_OTP_CON13,
};

static const struct efuse_reg reg_v1 = {
	.ck_pdn = EFUSE_V1_TOP_CKPDN_CON0,
	.ck_pdn_mask = RG_EFUSE_CK_PDN_MASK,
	.ck_pdn_hwen = EFUSE_V1_TOP_CKHWEN_CON0,
	.ck_pdn_hwen_mask = RG_EFUSE_CK_PDN_HWEN_MASK,
	.otp_pa = EFUSE_V1_OTP_CON0,
	.otp_rd_trig = EFUSE_V1_OTP_CON8,
	.otp_rd_sw = EFUSE_V1_OTP_CON11,
	.otp_dout_sw = EFUSE_V1_OTP_CON12,
	.otp_rd_busy = EFUSE_V1_OTP_CON13,
};

static const struct efuse_reg reg_v2 = {
	.ck_pdn = EFUSE_V2_TOP_CKPDN_CON1,
	.ck_pdn_mask = RG_EFUSE_CK_PDN_MASK,
	.ck_pdn_hwen = EFUSE_V2_TOP_CKHWEN_CON0,
	.ck_pdn_hwen_mask = RG_EFUSE_CK_PDN_HWEN_MASK,
	.otp_osc_ck_en = EFUSE_V2_OTP_CLK_CON0,
	.otp_pa = EFUSE_V2_OTP_CON0,
	.otp_rd_trig = EFUSE_V2_OTP_CON8,
	.otp_rd_sw = EFUSE_V2_OTP_CON11,
	.otp_dout_sw = EFUSE_V2_OTP_CON12_L,
	.otp_rd_busy = EFUSE_V2_OTP_CON13,
};

static const struct efuse_reg reg_v3 = {
	.ck_pdn = EFUSE_V3_TOP_CKPDN_CON1,
	.ck_pdn_mask = RG_EFUSE_CK_PDN_MASK,
	.ck_pdn_hwen = EFUSE_V3_TOP_CKHWEN_CON0,
	.ck_pdn_hwen_mask = BIT(0),
	.otp_pa = EFUSE_V3_OTP_CON0,
	.otp_rd_trig = EFUSE_V3_OTP_CON10,
	.otp_rd_sw = EFUSE_V3_OTP_CON13,
	.otp_dout_sw = EFUSE_V3_OTP_CON14,
	.otp_rd_busy = EFUSE_V3_OTP_CON16,
};

struct efuse_chip_data {
	unsigned int reg_num;
	unsigned int ctrl_reg_width;
	const struct efuse_reg *reg;
	unsigned int key_reg_num;
	unsigned int key_reg_val;
};

struct mt635x_efuse {
	struct device *dev;
	struct regmap *regmap;
	struct mutex lock;
	const struct efuse_chip_data *data;
	int trig_sta;
	u16 pmic_ecid[16][EFUSE_ECID_SIZE];
	const char *pmic_name[16];
};

static int mt635x_efuse_poll_busy(struct mt635x_efuse *efuse)
{
	int ret;
	unsigned int val = 0;

	udelay(EFUSE_POLL_DELAY_US);
	ret = regmap_read_poll_timeout(efuse->regmap,
				       efuse->data->reg->otp_rd_busy,
				       val,
				       !(val & RG_OTP_RD_BUSY_MASK),
				       EFUSE_POLL_DELAY_US,
				       EFUSE_POLL_TIMEOUT);
	if (ret) {
		dev_err(efuse->dev, "timeout to update the efuse status\n");
		return ret;
	}

	return 0;
}

static int mt635x_efuse_read(void *context, unsigned int offset,
			     void *_val, size_t bytes)
{
	struct mt635x_efuse *efuse = context;
	const struct efuse_chip_data *data = efuse->data;
	const struct efuse_reg *reg = data->reg;
	unsigned int buf = 0;
	unsigned int offset_end = offset + bytes;
	unsigned short *val = _val;
	unsigned short key_val = 0;
	int ret;

	mutex_lock(&efuse->lock);
	/* Unlock TMA Key */
	if (data->key_reg_num) {
		key_val = data->key_reg_val;
		regmap_bulk_write(efuse->regmap, data->key_reg_num, &key_val, 2);
	}
	/* Enable the efuse ctrl engine clock */
	ret = regmap_write(efuse->regmap,
			   reg->ck_pdn_hwen + data->ctrl_reg_width * CLR_OFFSET,
			   reg->ck_pdn_hwen_mask);
	if (ret)
		goto unlock_efuse;
	ret = regmap_write(efuse->regmap,
			   reg->ck_pdn + data->ctrl_reg_width * CLR_OFFSET,
			   reg->ck_pdn_mask);
	if (ret)
		goto disable_efuse;
	if (reg->otp_osc_ck_en) {
		ret = regmap_write(efuse->regmap,
				   reg->otp_osc_ck_en,
				   RG_OTP_OSC_CK_EN_MASK);
		if (ret)
			goto disable_efuse;
	}
	/* Set SW trigger read */
	ret = regmap_write(efuse->regmap, reg->otp_rd_sw, 1);
	if (ret)
		goto disable_efuse;
	for (; offset < offset_end; offset += EFUSE_REG_WIDTH) {
		/* Set the row to be read, one row is 2 bytes data */
		ret = regmap_write(efuse->regmap, reg->otp_pa, offset);
		if (ret)
			goto disable_efuse;
		/* Start trigger read */
		efuse->trig_sta = efuse->trig_sta ? 0 : 1;
		udelay(EFUSE_TRIG_DELAY_US);
		ret = regmap_write(efuse->regmap, reg->otp_rd_trig,
				   efuse->trig_sta);
		if (ret) {
			efuse->trig_sta = efuse->trig_sta ? 0 : 1;
			goto disable_efuse;
		}

		/* Ensure reading process is not in busy state */
		ret = mt635x_efuse_poll_busy(efuse);
		if (ret)
			goto disable_efuse;

		/* Read data from efuse memory, must delay before read */
		udelay(EFUSE_READ_DELAY_US);
		if (data->ctrl_reg_width == EFUSE_REG_WIDTH)
			ret = regmap_read(efuse->regmap,
					  reg->otp_dout_sw, &buf);
		else
			ret = regmap_bulk_read(efuse->regmap,
					       reg->otp_dout_sw,
					       (u8 *) &buf,
					       EFUSE_REG_WIDTH);
		if (ret)
			goto disable_efuse;
		dev_dbg(efuse->dev, "EFUSE[%d]=0x%x\n", offset, buf);
		*val++ = (unsigned short)buf;
	}
disable_efuse:
	/* Disable SW trigger read */
	regmap_write(efuse->regmap, reg->otp_rd_sw, 0);
	/* Disable the efuse ctrl engine clock */
	regmap_write(efuse->regmap,
		     reg->ck_pdn_hwen + data->ctrl_reg_width * SET_OFFSET,
		     reg->ck_pdn_hwen_mask);
	regmap_write(efuse->regmap,
		     reg->ck_pdn + data->ctrl_reg_width * SET_OFFSET,
		     reg->ck_pdn_mask);
	if (reg->otp_osc_ck_en)
		regmap_write(efuse->regmap, reg->otp_osc_ck_en, 0);
unlock_efuse:
	if (data->key_reg_num) {
		key_val = 0;
		regmap_bulk_write(efuse->regmap, data->key_reg_num, &key_val, 2);
	}
	mutex_unlock(&efuse->lock);

	return ret;
}

#if EFUSE_ECID_SYSFS
/* Create sysfs entry for lbat throttling ext */
static ssize_t efuse_pmic_ecid_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct platform_device *pdev;
	struct mt635x_efuse *efuse;
	int slvid = 0, i = 0;
	char *ptr = buf;

	pdev = to_platform_device(dev);
	efuse = platform_get_drvdata(pdev);

	for (slvid = 0; slvid < MAX_PMIC_SIZE; ++slvid) {
		if (efuse->pmic_name[slvid] != NULL) {
			ptr += sprintf(ptr, "%s:", efuse->pmic_name[slvid]);
			for (i = 0; i < EFUSE_ECID_SIZE; ++i)
				ptr += sprintf(ptr, "%X ", efuse->pmic_ecid[slvid][i]);
			ptr += sprintf(ptr, "\n");
		}
	}
	return strlen(buf);
}

static DEVICE_ATTR_RO(efuse_pmic_ecid);

static int mt635x_get_ecid(struct device *dev, struct mt635x_efuse *efuse)
{
	struct device_node *np, *ecid_np;
	int ret = 0, elems_count = 0, slvid = 0;
	u16 pmic_ecid[EFUSE_ECID_SIZE] = {0};

	np = of_find_node_by_name(dev->parent->of_node, "pmic-ecid");
	if (!np) {
		dev_notice(dev, "\"pmic-ecid\" dts node not found.\n");
		return -ENODEV;
	}

	for_each_child_of_node(np, ecid_np) {
		if (!ecid_np) {
			dev_notice(dev, "No \"%s\" dts node\n", ecid_np->name);
			return -ENODEV;
		}

		elems_count = of_property_count_elems_of_size(ecid_np, "ecid", sizeof(u16));
		ret = of_property_read_u16_array(ecid_np, "ecid", pmic_ecid, elems_count);
		if (ret < 0) {
			if (ret == -ENODATA)
				dev_notice(dev, "Property \"ecid\" not found.\n");
			else
				dev_notice(dev, "Failed to read \"ecid\" property.\n");
			return ret;
		}

		ret = of_property_read_u32(ecid_np, "slvid", &slvid);
		if (ret < 0) {
			if (ret == -ENODATA)
				dev_notice(dev, "Property \"slvid\" not found.\n");
			else
				dev_notice(dev, "Failed to read \"slvid\" property.\n");
			return ret;
		}

		memcpy(efuse->pmic_ecid[slvid], pmic_ecid, sizeof(pmic_ecid));
		efuse->pmic_name[slvid] = ecid_np->name;
	}
	return ret;
}
#endif

static int mt635x_efuse_probe(struct platform_device *pdev)
{
	struct nvmem_config econfig = { };
	struct nvmem_device *nvmem;
	struct mt635x_efuse *efuse;
	struct mt6397_chip *chip;
	int ret;
	static bool ecid_flag;

	efuse = devm_kzalloc(&pdev->dev, sizeof(*efuse), GFP_KERNEL);
	if (!efuse)
		return -ENOMEM;

	efuse->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!efuse->regmap) {
		chip = dev_get_drvdata(pdev->dev.parent);
		if (!chip || !chip->regmap) {
			dev_err(&pdev->dev, "failed to get efuse regmap\n");
			return -ENODEV;
		}
		efuse->regmap = chip->regmap;
	}

	efuse->data = of_device_get_match_data(&pdev->dev);
	if (!efuse->data) {
		dev_err(&pdev->dev, "failed to get efuse data\n");
		return -ENODEV;
	}
	ret = regmap_read(efuse->regmap, efuse->data->reg->otp_rd_trig,
			  &efuse->trig_sta);
	if (ret)
		return ret;

	mutex_init(&efuse->lock);
	efuse->dev = &pdev->dev;
	platform_set_drvdata(pdev, efuse);

	econfig.stride = 2;
	econfig.word_size = EFUSE_REG_WIDTH;
	econfig.read_only = true;
	econfig.name = dev_name(&pdev->dev);
	econfig.id = NVMEM_DEVID_NONE;
	econfig.size = efuse->data->reg_num * EFUSE_REG_WIDTH;
	econfig.reg_read = mt635x_efuse_read;
	econfig.priv = efuse;
	econfig.dev = &pdev->dev;
	econfig.add_legacy_fixed_of_cells = true;

	nvmem = devm_nvmem_register(&pdev->dev, &econfig);
	if (IS_ERR(nvmem)) {
		dev_err(&pdev->dev, "failed to register %s nvmem config\n",
			econfig.name);
		mutex_destroy(&efuse->lock);
		return PTR_ERR(nvmem);
	}

#if EFUSE_ECID_SYSFS
	/* Create sysfs entry */
	ret = device_create_file(&(pdev->dev), &dev_attr_efuse_pmic_ecid);
	if (ret < 0) {
		dev_notice(&pdev->dev, "failed to create efuse PMIC ecid sysfs file\n");
		return 0;
	}
	if (!ecid_flag) {
		ecid_flag = true;
		ret = mt635x_get_ecid(&pdev->dev, efuse);
		if (ret >= 0)
			platform_set_drvdata(pdev, efuse);
	}
#endif
	pr_info("efuse probe success\n");

	return 0;
}

static const struct efuse_chip_data mt6357_efuse_data = {
	.reg_num = 128,
	.ctrl_reg_width = 0x2,
	.reg = &reg_v0,
};

static const struct efuse_chip_data mt6359p_efuse_data = {
	.reg_num = 128,
	.ctrl_reg_width = 0x2,
	.reg = &reg_v1,
};

static const struct efuse_chip_data mt6363_efuse_data = {
	.reg_num = 128,
	.ctrl_reg_width = 0x1,
	.reg = &reg_v2,
};

static const struct efuse_chip_data mt6368_efuse_data = {
	.reg_num = 128,
	.ctrl_reg_width = 0x1,
	.reg = &reg_v2,
	.key_reg_num = 0x39e,
	.key_reg_val = 0x9C97,
};

static const struct efuse_chip_data mt6377_efuse_data = {
	.reg_num = 128,
	.ctrl_reg_width = 0x1,
	.reg = &reg_v3,
};

static const struct of_device_id mt635x_efuse_of_match[] = {
	{
		.compatible = "mediatek,mt6357-efuse",
		.data = &mt6357_efuse_data
	}, {
		.compatible = "mediatek,mt6358-efuse",
		.data = &mt6359p_efuse_data
	}, {
		.compatible = "mediatek,mt6359p-efuse",
		.data = &mt6359p_efuse_data
	}, {
		.compatible = "mediatek,mt6363-efuse",
		.data = &mt6363_efuse_data
	}, {
		.compatible = "mediatek,mt6368-efuse",
		.data = &mt6368_efuse_data
	}, {
		.compatible = "mediatek,mt6373-efuse",
		.data = &mt6363_efuse_data
	}, {
		.compatible = "mediatek,mt6377-efuse",
		.data = &mt6377_efuse_data
	}, {
		/* sentinel */
	}
};

static struct platform_driver mt635x_efuse_driver = {
	.probe = mt635x_efuse_probe,
	.driver = {
		.name = "mt635x-efuse",
		.of_match_table = mt635x_efuse_of_match,
	},
};
module_platform_driver(mt635x_efuse_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Jeter Chen <Jeter.Chen@mediatek.com>");
MODULE_DESCRIPTION("MediaTek PMIC eFuse Driver for MT6359 PMIC");
