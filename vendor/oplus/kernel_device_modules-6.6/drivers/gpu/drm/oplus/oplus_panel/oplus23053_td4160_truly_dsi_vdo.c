/*
 * Copyright (c) 2015 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <soc/oplus/device_info.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "oplus23053_data_hw_roundedpattern.h"
#endif

#include <../../../misc/mediatek/include/mt-plat/mtk_boot_common.h>
#include <soc/oplus/system/oplus_project.h>
//static char bl_tb0[] = { 0x51, 0xff };
extern int __attribute((weak)) tp_control_irq(bool enable, int mode);
extern unsigned int __attribute((weak)) is_project(int project)  { return 0; }


#if IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#include "../mediatek/mediatek_v2/mtk_disp_notify.h"
#endif

#define LCD_CTL_RST_OFF 0x12
#define LCD_CTL_RST_ON 0x11
#define LCD_CTL_CS_OFF  0x1A
#define LCD_CTL_TP_LOAD_FW 0x10
#define LCD_CTL_CS_ON  0x19

#if IS_ENABLED(CONFIG_TOUCHPANEL_NOTIFY)
extern int (*tp_gesture_enable_notifier)(unsigned int tp_index);
extern int shutdown_flag;
#endif
/*****************************************************************************
 * Function Prototype
 *****************************************************************************/

//unsigned long esd_flag = 1;
static int esd_brightness = 1023;
extern unsigned long oplus_max_normal_brightness;
extern void __attribute((weak)) disp_aal_set_dre_en(int enable) { return; };
extern int _20015_lcm_i2c_write_bytes(unsigned char addr, unsigned char value);
static int cabc_lastlevel = 0;
static int last_brightness = 0;
static void cabc_switch(void *dsi, dcs_write_gce cb,
                void *handle, unsigned int cabc_mode);
static int backlight_gamma = 0;
static int last_powerflag = 0;
static int lcm_dimming_flag = 0;

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_pos, *bias_neg;

	bool prepared;
	bool enabled;

	int error;

    bool is_normal_mode;
};

#define lcm_dcs_write_seq(ctx, seq...) \
({\
	const u8 d[] = { seq };\
	BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64, "DCS sequence too big for stack");\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

#define lcm_dcs_write_seq_static(ctx, seq...) \
({\
	static const u8 d[] = { seq };\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

static void lcm_dcs_write(struct lcm *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	addr = (char *)data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		pr_err("error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

#ifdef PANEL_SUPPORT_READBACK
static int lcm_dcs_read(struct lcm *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		pr_err("error %d reading dcs seq:(%#x)\n", ret, cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = {0};
	static int ret;

	if (ret == 0) {
		ret = lcm_dcs_read(ctx,  0x0A, buffer, 1);
		DISP_DEBUG("return %d data(0x%08x) to dsi engine\n",
			 ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
static struct regulator *disp_bias_pos;
static struct regulator *disp_bias_neg;


static int lcm_panel_bias_regulator_init(void)
{
	static int regulator_inited;
	int ret = 0;

	if (regulator_inited)
		return ret;

	/* please only get regulator once in a driver */
	disp_bias_pos = regulator_get(NULL, "dsv_pos");
	if (IS_ERR(disp_bias_pos)) { /* handle return value */
		ret = PTR_ERR(disp_bias_pos);
		pr_err("get dsv_pos fail, error: %d\n", ret);
		return ret;
	}

	disp_bias_neg = regulator_get(NULL, "dsv_neg");
	if (IS_ERR(disp_bias_neg)) { /* handle return value */
		ret = PTR_ERR(disp_bias_neg);
		pr_err("get dsv_neg fail, error: %d\n", ret);
		return ret;
	}

	regulator_inited = 1;
	return ret; /* must be 0 */

}

static int lcm_panel_bias_enable(void)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_bias_regulator_init();

	/* set voltage with min & max*/
	ret = regulator_set_voltage(disp_bias_pos, 6000000, 6000000);
	if (ret < 0)
		pr_err("set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_set_voltage(disp_bias_neg, 6000000, 6000000);
	if (ret < 0)
		pr_err("set voltage disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	/* enable regulator */
	ret = regulator_enable(disp_bias_pos);
	if (ret < 0)
		pr_err("enable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_enable(disp_bias_neg);
	if (ret < 0)
		pr_err("enable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	return retval;
}

static int lcm_panel_bias_disable(void)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_bias_regulator_init();

	ret = regulator_disable(disp_bias_neg);
	if (ret < 0)
		pr_err("disable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_disable(disp_bias_pos);
	if (ret < 0)
		pr_err("disable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	return retval;
}
#endif

static void lcm_panel_init(struct lcm *ctx)
{
		lcm_dcs_write_seq_static(ctx, 0xB0, 0x04);
		lcm_dcs_write_seq_static(ctx, 0xD6, 0x00);
		/* cabc start */
		lcm_dcs_write_seq_static(ctx, 0xB8, 0x02, 0x4A, 0x06, 0x00, 0x04, 0x3F);
		lcm_dcs_write_seq_static(ctx, 0xB9, 0x02, 0x4a, 0x18, 0x00, 0x09, 0xb4);
		lcm_dcs_write_seq_static(ctx, 0xBA, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00);
		lcm_dcs_write_seq_static(ctx, 0xCE, 0x16, 0x40, 0x60, 0x78, 0xe9, 0x94, 0x9e, 0xab, 0xb8, 0xc4,
					0xcd, 0xd6, 0xde, 0xe5, 0xec, 0xf3, 0xff, 0x07, 0x0F, 0x00, 0x00, 0x00, 0x04, 0x8C);
		/* cabc end */
		lcm_dcs_write_seq_static(ctx, 0x51, 0x00, 0x00);
		lcm_dcs_write_seq_static(ctx, 0x53, 0x24);
		lcm_dcs_write_seq_static(ctx, 0x55, 0x00);
		lcm_dcs_write_seq_static(ctx, 0x35, 0x00);
		tp_control_irq(true, 1);
		lcm_dcs_write_seq_static(ctx, 0x11, 0x00);
		usleep_range(80 * 1000, 80 * 1000 + 100);
		pr_info("Successful\n");
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	if (!ctx->prepared)
		return 0;
	lcm_dcs_write_seq_static(ctx, 0x28);
	usleep_range(20 * 1000, 20 * 1000 + 100);
	lcm_dcs_write_seq_static(ctx, 0x10);
	usleep_range(80 * 1000, 80 * 1000 + 100);

	ctx->error = 0;
	ctx->prepared = false;
	last_powerflag = 0;

#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	lcm_panel_bias_disable();
#else
	pr_info("%s: tp_gesture_enable_flag = %d shutdown_flag : %d\n", __func__, tp_gesture_enable_notifier(0),shutdown_flag);
	if (/*(esd_flag == 1) ||*/ (shutdown_flag == 1) || (tp_gesture_enable_notifier(0) == 0)) {

#if IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
	/*int blank;
	blank = LCD_CTL_RST_OFF;
	mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
	pr_info("[TP] tp gesture is disable, Display goto power off , And TP reset will low\n");
	blank = LCD_CTL_CS_OFF;
	mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
	pr_info("[TP]TP CS will change to gpio mode and low\n");*/
#endif
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		pr_err("cannot get reset_gpio %ld\n",
			PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(2 * 1000, 2 * 1000 + 100);
	int blank;
	blank = LCD_CTL_RST_OFF;
	mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
	usleep_range(2 * 1000, 2 * 1000 + 100);

	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		pr_err("cannot get bias_neg %ld\n",
			PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);

	usleep_range(1 * 1000, 1 * 1000 + 100);

	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		pr_err("cannot get bias_pos %ld\n",
			PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	gpiod_set_value(ctx->bias_pos, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);
	usleep_range(30 * 1000, 30 * 1000 + 100);
	}
#endif
	return 0;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	tp_control_irq(false, 1);

	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	usleep_range(3 * 1000, 3 * 1000 + 100);
	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
	_20015_lcm_i2c_write_bytes(0x0, 0x14);
	usleep_range(1 * 1000, 1 * 1000 + 100);
	_20015_lcm_i2c_write_bytes(0x1, 0x14);
	usleep_range(1 * 1000, 1 * 1000 + 100);
	_20015_lcm_i2c_write_bytes(0x3, 0x43);
#if IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
	/*int blank;
	blank = LCD_CTL_CS_ON;
	mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
	pr_err("[TP]TP CS will chang to spi mode and high\n");
	usleep_range(5000, 5100);
	blank = LCD_CTL_TP_LOAD_FW;
	mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
	pr_info("[TP] start to load fw!\n");*/
#endif
	pr_info("Successful\n");
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared)
		return 0;

#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	lcm_panel_bias_enable();
#endif
	last_powerflag = 1;
	int blank;
	blank = LCD_CTL_RST_ON;
	mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
	usleep_range(2 * 1000, 2 * 1000 + 100);
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(5 * 1000, 5 * 1000 + 100);
	gpiod_set_value(ctx->reset_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(22 * 1000, 22 * 1000 + 100);
	blank = LCD_CTL_TP_LOAD_FW;
	mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_rst(panel);
#endif
#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif

	pr_info("Successful\n");
	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}

#define VAC (1612)
#define HAC (720)

static struct drm_display_mode default_mode = {
	.clock = 136709,
	.hdisplay = HAC,
	.hsync_start = HAC + 55,//HFP
	.hsync_end = HAC + 55 + 4,//HSA
	.htotal = HAC + 55 + 4 + 23,//HBP
	.vdisplay = VAC,
	.vsync_start = VAC + 1205,//VFP
	.vsync_end = VAC + 1205 + 4,//VSA
	.vtotal = VAC + 1205 + 4 + 20,//VBP
};

static struct drm_display_mode performance_mode = {
	.clock = 136853,
	.hdisplay = HAC,
	.hsync_start = HAC + 55,//HFP
	.hsync_end = HAC + 55 + 4,//HSA
	.htotal = HAC + 55 + 4 + 23,//HBP
	.vdisplay = VAC,
	.vsync_start = VAC + 260,//VFP
	.vsync_end = VAC + 260 + 4,//VSA
	.vtotal = VAC + 260 + 4 + 20,//VBP
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params = {
	.pll_clk = 449,
	//.vfp_low_power = 2528,//45hz
	.cust_esd_check = 1,
	.esd_check_enable = 0,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9c, .mask_list[0] = 0x9c,
	},
	.data_rate = 898,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
	},
	.dyn = {
		.switch_en = 1,
		.hbp = 33,
		.vfp = 1205,
		.pll_clk = 454,
	},
	//.oplus_teot_ns_multiplier = 90,
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
};

static struct mtk_panel_params ext_params_90hz = {
	.pll_clk = 449,
	//.vfp_low_power = VFP_60HZ,
	.cust_esd_check = 1,
	.esd_check_enable = 0,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0a, .count = 1, .para_list[0] = 0x9c, .mask_list[0] = 0x9c,
	},
	.data_rate = 898,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
	},
	.dyn = {
		.switch_en = 1,
		.hbp = 33,
		.vfp = 260,
		.pll_clk = 454,
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
};

static void td4160_truly_enter_gamma(void *dsi, dcs_write_gce cb, void *handle)
{
	char bl_tb0[] = {0xB0, 0x04};
	char bl_tb1[] = {0xC7, 0x02, 0xFE, 0x02, 0xFF, 0x02, 0xFF, 0x02, 0xFF, 0x02, 0xFF, 0x02, 0xFF, 0x02, 0xB5, 0x02, 0x6A, 0x02, 0x2F, \
					0x02, 0x11, 0x01, 0xF3, 0x01, 0xDE, 0x01, 0xD3, 0x01, 0xB8, 0x01, 0xBA, 0x02, 0x44};
	char bl_tb2[] = {0xF0, 0xC7, 0x20, 0x02};
	char bl_tb3[] = {0xF0, 0xC7, 0x21, 0x7C};
	char bl_tb4[] = {0xF0, 0xC7, 0x22, 0x02};
	char bl_tb5[] = {0xF0, 0xC7, 0x23, 0xEB};
	char bl_tb6[] = {0xF0, 0xC7, 0x24, 0x02};
	char bl_tb7[] = {0xF0, 0xC7, 0x25, 0xFF};
	char bl_tb8[] = {0xF0, 0xC7, 0x26, 0x02};
	char bl_tb9[] = {0xF0, 0xC7, 0x27, 0xFE};
	char bl_tb10[] = {0xF0, 0xC7, 0x28, 0x02};
	char bl_tb11[] = {0xF0, 0xC7, 0x29, 0xFF};
	char bl_tb12[] = {0xF0, 0xC7, 0x2A, 0x02};
	char bl_tb13[] = {0xF0, 0xC7, 0x2B, 0xFF};
	char bl_tb14[] = {0xF0, 0xC7, 0x2C, 0x02};
	char bl_tb15[] = {0xF0, 0xC7, 0x2D, 0xFF};
	char bl_tb16[] = {0xF0, 0xC7, 0x2E, 0x02};
	char bl_tb17[] = {0xF0, 0xC7, 0x2F, 0xFF};
	char bl_tb18[] = {0xF0, 0xC7, 0x30, 0x02};
	char bl_tb19[] = {0xF0, 0xC7, 0x31, 0xFF};
	char bl_tb20[] = {0xF0, 0xC7, 0x32, 0x02};
	char bl_tb21[] = {0xF0, 0xC7, 0x33, 0xB5};
	char bl_tb22[] = {0xF0, 0xC7, 0x34, 0x02};
	char bl_tb23[] = {0xF0, 0xC7, 0x35, 0x6A};
	char bl_tb24[] = {0xF0, 0xC7, 0x36, 0x02};
	char bl_tb25[] = {0xF0, 0xC7, 0x37, 0x2F};
	char bl_tb26[] = {0xF0, 0xC7, 0x38, 0x02};
	char bl_tb27[] = {0xF0, 0xC7, 0x39, 0x11};
	char bl_tb28[] = {0xF0, 0xC7, 0x3A, 0x01};
	char bl_tb29[] = {0xF0, 0xC7, 0x3B, 0xF3};
	char bl_tb30[] = {0xF0, 0xC7, 0x3C, 0x01};
	char bl_tb31[] = {0xF0, 0xC7, 0x3D, 0xDE};
	char bl_tb32[] = {0xF0, 0xC7, 0x3E, 0x01};
	char bl_tb33[] = {0xF0, 0xC7, 0x3F, 0xD3};
	char bl_tb34[] = {0xF0, 0xC7, 0x40, 0x01};
	char bl_tb35[] = {0xF0, 0xC7, 0x41, 0xB8};
	char bl_tb36[] = {0xF0, 0xC7, 0x42, 0x01};
	char bl_tb37[] = {0xF0, 0xC7, 0x43, 0xBA};
	char bl_tb38[] = {0xF0, 0xC7, 0x44, 0x02};
	char bl_tb39[] = {0xF0, 0xC7, 0x45, 0x44};
	char bl_tb40[] = {0xF0, 0xC7, 0x46, 0x02};
	char bl_tb41[] = {0xF0, 0xC7, 0x47, 0x7C};
	char bl_tb42[] = {0xF0, 0xC7, 0x48, 0x02};
	char bl_tb43[] = {0xF0, 0xC7, 0x49, 0xEB};
	char bl_tb44[] = {0xF0, 0xC7, 0x4A, 0x02};
	char bl_tb45[] = {0xF0, 0xC7, 0x4B, 0xFF};

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
	cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
	cb(dsi, handle, bl_tb4, ARRAY_SIZE(bl_tb4));
	cb(dsi, handle, bl_tb5, ARRAY_SIZE(bl_tb5));
	cb(dsi, handle, bl_tb6, ARRAY_SIZE(bl_tb6));
	cb(dsi, handle, bl_tb7, ARRAY_SIZE(bl_tb7));
	cb(dsi, handle, bl_tb8, ARRAY_SIZE(bl_tb8));
	cb(dsi, handle, bl_tb9, ARRAY_SIZE(bl_tb9));
	cb(dsi, handle, bl_tb10, ARRAY_SIZE(bl_tb10));
	cb(dsi, handle, bl_tb11, ARRAY_SIZE(bl_tb11));
	cb(dsi, handle, bl_tb12, ARRAY_SIZE(bl_tb12));
	cb(dsi, handle, bl_tb13, ARRAY_SIZE(bl_tb13));
	cb(dsi, handle, bl_tb14, ARRAY_SIZE(bl_tb14));
	cb(dsi, handle, bl_tb15, ARRAY_SIZE(bl_tb15));
	cb(dsi, handle, bl_tb16, ARRAY_SIZE(bl_tb16));
	cb(dsi, handle, bl_tb17, ARRAY_SIZE(bl_tb17));
	cb(dsi, handle, bl_tb18, ARRAY_SIZE(bl_tb18));
	cb(dsi, handle, bl_tb19, ARRAY_SIZE(bl_tb19));
	cb(dsi, handle, bl_tb20, ARRAY_SIZE(bl_tb20));
	cb(dsi, handle, bl_tb21, ARRAY_SIZE(bl_tb21));
	cb(dsi, handle, bl_tb22, ARRAY_SIZE(bl_tb22));
	cb(dsi, handle, bl_tb23, ARRAY_SIZE(bl_tb23));
	cb(dsi, handle, bl_tb24, ARRAY_SIZE(bl_tb24));
	cb(dsi, handle, bl_tb25, ARRAY_SIZE(bl_tb25));
	cb(dsi, handle, bl_tb26, ARRAY_SIZE(bl_tb26));
	cb(dsi, handle, bl_tb27, ARRAY_SIZE(bl_tb27));
	cb(dsi, handle, bl_tb28, ARRAY_SIZE(bl_tb28));
	cb(dsi, handle, bl_tb29, ARRAY_SIZE(bl_tb29));
	cb(dsi, handle, bl_tb30, ARRAY_SIZE(bl_tb30));
	cb(dsi, handle, bl_tb31, ARRAY_SIZE(bl_tb31));
	cb(dsi, handle, bl_tb32, ARRAY_SIZE(bl_tb32));
	cb(dsi, handle, bl_tb33, ARRAY_SIZE(bl_tb33));
	cb(dsi, handle, bl_tb34, ARRAY_SIZE(bl_tb34));
	cb(dsi, handle, bl_tb35, ARRAY_SIZE(bl_tb35));
	cb(dsi, handle, bl_tb36, ARRAY_SIZE(bl_tb36));
	cb(dsi, handle, bl_tb37, ARRAY_SIZE(bl_tb37));
	cb(dsi, handle, bl_tb38, ARRAY_SIZE(bl_tb38));
	cb(dsi, handle, bl_tb39, ARRAY_SIZE(bl_tb39));
	cb(dsi, handle, bl_tb40, ARRAY_SIZE(bl_tb40));
	cb(dsi, handle, bl_tb41, ARRAY_SIZE(bl_tb41));
	cb(dsi, handle, bl_tb42, ARRAY_SIZE(bl_tb42));
	cb(dsi, handle, bl_tb43, ARRAY_SIZE(bl_tb43));
	cb(dsi, handle, bl_tb44, ARRAY_SIZE(bl_tb44));
	cb(dsi, handle, bl_tb45, ARRAY_SIZE(bl_tb45));
}

static void td4160_truly_exit_gamma(void *dsi, dcs_write_gce cb, void *handle)
{
	char bl_tb0[] = {0xB0, 0x04};
	char bl_tb1[] = {0xC7, 0x00, 0x00, 0x00, 0x55, 0x00, 0xc2, 0x01, 0x0f, 0x01, 0x82, 0x01, 0xb5, 0x01, 0xb5, 0x01, 0xba, 0x01, 0xb2, \
					0x01, 0xb1, 0x01, 0xad, 0x01, 0xa7, 0x01, 0xa3, 0x01, 0x8c, 0x01, 0x99, 0x02, 0x2c};
	char bl_tb2[] = {0xF0, 0xC7, 0x20, 0x02};
	char bl_tb3[] = {0xF0, 0xC7, 0x21, 0x6f};
	char bl_tb4[] = {0xF0, 0xC7, 0x22, 0x02};
	char bl_tb5[] = {0xF0, 0xC7, 0x23, 0x9d};
	char bl_tb6[] = {0xF0, 0xC7, 0x24, 0x02};
	char bl_tb7[] = {0xF0, 0xC7, 0x25, 0xb2};
	char bl_tb8[] = {0xF0, 0xC7, 0x26, 0x00};
	char bl_tb9[] = {0xF0, 0xC7, 0x27, 0x00};
	char bl_tb10[] = {0xF0, 0xC7, 0x28, 0x00};
	char bl_tb11[] = {0xF0, 0xC7, 0x29, 0x55};
	char bl_tb12[] = {0xF0, 0xC7, 0x2A, 0x00};
	char bl_tb13[] = {0xF0, 0xC7, 0x2B, 0xc2};
	char bl_tb14[] = {0xF0, 0xC7, 0x2C, 0x01};
	char bl_tb15[] = {0xF0, 0xC7, 0x2D, 0x0f};
	char bl_tb16[] = {0xF0, 0xC7, 0x2E, 0x01};
	char bl_tb17[] = {0xF0, 0xC7, 0x2F, 0x82};
	char bl_tb18[] = {0xF0, 0xC7, 0x30, 0x01};
	char bl_tb19[] = {0xF0, 0xC7, 0x31, 0xb5};
	char bl_tb20[] = {0xF0, 0xC7, 0x32, 0x01};
	char bl_tb21[] = {0xF0, 0xC7, 0x33, 0xb5};
	char bl_tb22[] = {0xF0, 0xC7, 0x34, 0x01};
	char bl_tb23[] = {0xF0, 0xC7, 0x35, 0xba};
	char bl_tb24[] = {0xF0, 0xC7, 0x36, 0x01};
	char bl_tb25[] = {0xF0, 0xC7, 0x37, 0xb2};
	char bl_tb26[] = {0xF0, 0xC7, 0x38, 0x01};
	char bl_tb27[] = {0xF0, 0xC7, 0x39, 0xb1};
	char bl_tb28[] = {0xF0, 0xC7, 0x3A, 0x01};
	char bl_tb29[] = {0xF0, 0xC7, 0x3B, 0xad};
	char bl_tb30[] = {0xF0, 0xC7, 0x3C, 0x01};
	char bl_tb31[] = {0xF0, 0xC7, 0x3D, 0xa7};
	char bl_tb32[] = {0xF0, 0xC7, 0x3E, 0x01};
	char bl_tb33[] = {0xF0, 0xC7, 0x3F, 0xa3};
	char bl_tb34[] = {0xF0, 0xC7, 0x40, 0x01};
	char bl_tb35[] = {0xF0, 0xC7, 0x41, 0x8c};
	char bl_tb36[] = {0xF0, 0xC7, 0x42, 0x01};
	char bl_tb37[] = {0xF0, 0xC7, 0x43, 0x99};
	char bl_tb38[] = {0xF0, 0xC7, 0x44, 0x02};
	char bl_tb39[] = {0xF0, 0xC7, 0x45, 0x2c};
	char bl_tb40[] = {0xF0, 0xC7, 0x46, 0x02};
	char bl_tb41[] = {0xF0, 0xC7, 0x47, 0x6f};
	char bl_tb42[] = {0xF0, 0xC7, 0x48, 0x02};
	char bl_tb43[] = {0xF0, 0xC7, 0x49, 0x9d};
	char bl_tb44[] = {0xF0, 0xC7, 0x4A, 0x02};
	char bl_tb45[] = {0xF0, 0xC7, 0x4B, 0xb2};

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
	cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
	cb(dsi, handle, bl_tb4, ARRAY_SIZE(bl_tb4));
	cb(dsi, handle, bl_tb5, ARRAY_SIZE(bl_tb5));
	cb(dsi, handle, bl_tb6, ARRAY_SIZE(bl_tb6));
	cb(dsi, handle, bl_tb7, ARRAY_SIZE(bl_tb7));
	cb(dsi, handle, bl_tb8, ARRAY_SIZE(bl_tb8));
	cb(dsi, handle, bl_tb9, ARRAY_SIZE(bl_tb9));
	cb(dsi, handle, bl_tb10, ARRAY_SIZE(bl_tb10));
	cb(dsi, handle, bl_tb11, ARRAY_SIZE(bl_tb11));
	cb(dsi, handle, bl_tb12, ARRAY_SIZE(bl_tb12));
	cb(dsi, handle, bl_tb13, ARRAY_SIZE(bl_tb13));
	cb(dsi, handle, bl_tb14, ARRAY_SIZE(bl_tb14));
	cb(dsi, handle, bl_tb15, ARRAY_SIZE(bl_tb15));
	cb(dsi, handle, bl_tb16, ARRAY_SIZE(bl_tb16));
	cb(dsi, handle, bl_tb17, ARRAY_SIZE(bl_tb17));
	cb(dsi, handle, bl_tb18, ARRAY_SIZE(bl_tb18));
	cb(dsi, handle, bl_tb19, ARRAY_SIZE(bl_tb19));
	cb(dsi, handle, bl_tb20, ARRAY_SIZE(bl_tb20));
	cb(dsi, handle, bl_tb21, ARRAY_SIZE(bl_tb21));
	cb(dsi, handle, bl_tb22, ARRAY_SIZE(bl_tb22));
	cb(dsi, handle, bl_tb23, ARRAY_SIZE(bl_tb23));
	cb(dsi, handle, bl_tb24, ARRAY_SIZE(bl_tb24));
	cb(dsi, handle, bl_tb25, ARRAY_SIZE(bl_tb25));
	cb(dsi, handle, bl_tb26, ARRAY_SIZE(bl_tb26));
	cb(dsi, handle, bl_tb27, ARRAY_SIZE(bl_tb27));
	cb(dsi, handle, bl_tb28, ARRAY_SIZE(bl_tb28));
	cb(dsi, handle, bl_tb29, ARRAY_SIZE(bl_tb29));
	cb(dsi, handle, bl_tb30, ARRAY_SIZE(bl_tb30));
	cb(dsi, handle, bl_tb31, ARRAY_SIZE(bl_tb31));
	cb(dsi, handle, bl_tb32, ARRAY_SIZE(bl_tb32));
	cb(dsi, handle, bl_tb33, ARRAY_SIZE(bl_tb33));
	cb(dsi, handle, bl_tb34, ARRAY_SIZE(bl_tb34));
	cb(dsi, handle, bl_tb35, ARRAY_SIZE(bl_tb35));
	cb(dsi, handle, bl_tb36, ARRAY_SIZE(bl_tb36));
	cb(dsi, handle, bl_tb37, ARRAY_SIZE(bl_tb37));
	cb(dsi, handle, bl_tb38, ARRAY_SIZE(bl_tb38));
	cb(dsi, handle, bl_tb39, ARRAY_SIZE(bl_tb39));
	cb(dsi, handle, bl_tb40, ARRAY_SIZE(bl_tb40));
	cb(dsi, handle, bl_tb41, ARRAY_SIZE(bl_tb41));
	cb(dsi, handle, bl_tb42, ARRAY_SIZE(bl_tb42));
	cb(dsi, handle, bl_tb43, ARRAY_SIZE(bl_tb43));
	cb(dsi, handle, bl_tb44, ARRAY_SIZE(bl_tb44));
	cb(dsi, handle, bl_tb45, ARRAY_SIZE(bl_tb45));
}

static void td4160_truly_lcm_dimming_on(void *dsi, dcs_write_gce cb, void *handle)
{
	char bl_tb0[] = {0x53, 0x2C};
	char bl_tb1[] = {0x22, 0x00};

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	usleep_range(6 * 1000, 6 * 1000 + 100);
}

static void td4160_truly_lcm_dimming_off(void *dsi, dcs_write_gce cb, void *handle)
{
	char bl_tb0[] = {0x53, 0x24};
	char bl_tb1[] = {0x13, 0x00};

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	usleep_range(6 * 1000, 6 * 1000 + 100);
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int level)
{
	char bl_tb0[] = {0x51, 0xFF, 0xFF};
	char bl_tb1[] = {0x55, 0x00};
	char bl_tb3[] = {0x53, 0x24};

	char bl_tb10[] = {0x29, 0x00};

	if (level > 4095)
                level = 4095;
	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0) {
		level = 2047;
	}

	if(level < 14 && level > 0){
		backlight_gamma = 1;
		td4160_truly_enter_gamma(dsi, cb, handle);
	} else if (level > 13 && backlight_gamma == 1){
		backlight_gamma = 0;
		td4160_truly_exit_gamma(dsi, cb, handle);
	} else if (level == 0){
		backlight_gamma = 0;
	}

	if(level == 0 && lcm_dimming_flag == 0) {
		td4160_truly_lcm_dimming_on(dsi, cb, handle);
		pr_info("dimming on, level = %d\n", level);
		lcm_dimming_flag = 1;
	} else if (level > 0 && lcm_dimming_flag == 1) {
		td4160_truly_lcm_dimming_off(dsi, cb, handle);
		pr_info("dimming off, level = %d\n", level);
		lcm_dimming_flag = 0;
	}

        bl_tb0[1] = level >> 8;
        bl_tb0[2] = level & 0xFF;
	esd_brightness = level;
	if (!cb)
		return -1;
	if ((last_brightness == 0) || (last_powerflag == 1)) {
		pr_info("last_brightness=%d, last_powerflag=%d\n", last_brightness, last_powerflag);
		cb(dsi, handle, bl_tb10, ARRAY_SIZE(bl_tb10));
		usleep_range(15 * 1000, 15 * 1000 + 100);
	}
	last_powerflag = 0;

	pr_info("level=%d, bl_tb0[1]=%x, bl_tb0[2]=%x\n", level, bl_tb0[1], bl_tb0[2]);
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	if ((last_brightness == 0) && (cabc_lastlevel != 0)) {
		bl_tb1[1] = cabc_lastlevel;
		cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
		cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
	}
	last_brightness = level;
	return 0;
}

static int oplus_esd_backlight_check(void *dsi, dcs_write_gce cb,
		void *handle)
{
	char bl_tb0[] = {0x51, 0x07, 0xff};
	char bl_tb1[] = {0x29, 0x00};

	bl_tb0[1] = esd_brightness >> 8;
	bl_tb0[2] = esd_brightness & 0xFF;
	if (!cb)
		return -1;
	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	usleep_range(15 * 1000, 15 * 1000 + 100);
	pr_info("esd_backlight=%d bl_tb0[1]=%x, bl_tb0[2]=%x\n", esd_brightness, bl_tb0[1], bl_tb0[2]);
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 1;
}

static struct drm_display_mode *get_mode_by_id(struct drm_connector *connector,
	unsigned int mode)
{
	struct drm_display_mode *m;
	unsigned int i = 0;

	list_for_each_entry(m, &connector->modes, head) {
		if (i == mode)
			return m;
		i++;
	}
	return NULL;
}

static int mtk_panel_ext_param_set(struct drm_panel *panel,
	struct drm_connector *connector, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, mode);
	if (!m) {
		pr_info("[error]%s:%d invalid display_mode\n", __func__, __LINE__);
		return ret;
	}
	pr_info("set mode = %d Successful\n", mode);
	if (mode == 0)
		ext->params = &ext_params;
	else if (mode == 1)
		ext->params = &ext_params_90hz;
	if (drm_mode_vrefresh(m) == 60)
		ext->params = &ext_params;
	else if (drm_mode_vrefresh(m) == 90)
		ext->params = &ext_params_90hz;
	else
		ret = 1;

	return ret;
}

static int mtk_panel_ext_param_get(struct drm_panel *panel,
		struct drm_connector *connector,
		struct mtk_panel_params **ext_para,
		unsigned int mode)
{
	int ret = 0;
	pr_info("get mode = %d Successful\n", mode);
	if (mode == 0)
		*ext_para = &ext_params;
	else if (mode == 1)
		*ext_para = &ext_params_90hz;
	else
		ret = 1;

	return ret;

}

static void cabc_switch(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int cabc_mode)
{
	char bl_tb0[] = {0x55, 0x00};
	char bl_tb3[] = {0x53, 0x2C};
	pr_info("cabc = %d\n", cabc_mode);
	if (cabc_mode == 3)
		cabc_mode = 2;
	else if (cabc_mode == 0)
		cabc_mode = 3;
	bl_tb0[1] = (u8)cabc_mode;

	cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));//FB 01
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));//55 0X
	cabc_lastlevel = cabc_mode;
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.esd_backlight_recovery = oplus_esd_backlight_check,
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
	.ext_param_set = mtk_panel_ext_param_set,
	.ext_param_get = mtk_panel_ext_param_get,
	.cabc_switch = cabc_switch,
};
#endif

struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;

	unsigned int bpc;

	struct {
		unsigned int width;
		unsigned int height;
	} size;

	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;
};

static int lcm_get_modes(struct drm_panel *panel,
	struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	struct drm_display_mode *mode2;

	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		pr_err("failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	mode2 = drm_mode_duplicate(connector->dev, &performance_mode);
	if (!mode2) {
		pr_err("failed to add mode %ux%ux@%u\n",
			 performance_mode.hdisplay, performance_mode.vdisplay,
			 drm_mode_vrefresh(&performance_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode2);
	mode2->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode2);
	connector->display_info.width_mm = 68;
	connector->display_info.height_mm = 152;

	return 1;
}

static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};

static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct lcm *ctx;
	struct device_node *backlight;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO
			 | MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_NO_EOT_PACKET
			 | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		pr_err("cannot get reset-gpios %ld\n",
			PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);

	ctx->bias_pos = devm_gpiod_get_index(dev, "bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		pr_err("cannot get bias-pos 0 %ld\n",
			PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	devm_gpiod_put(dev, ctx->bias_pos);

	ctx->bias_neg = devm_gpiod_get_index(dev, "bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		pr_err("cannot get bias-neg 1 %ld\n",
			PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	devm_gpiod_put(dev, ctx->bias_neg);

	ctx->prepared = true;
	ctx->enabled = true;

	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif

	oplus_max_normal_brightness = 3276;


	register_device_proc("lcd","td4160_truly","TRULY");
	pr_info("Successful\n");

	return ret;
}

static void lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);
#if defined(CONFIG_MTK_PANEL_EXT)
	struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);
#endif
	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_detach(ext_ctx);
	mtk_panel_remove(ext_ctx);
#endif
}
static const struct of_device_id jdi_of_match[] = {
	{
		.compatible = "oplus23053,truly,td4160,dphy,vdo",
	},
	{} };

MODULE_DEVICE_TABLE(of, jdi_of_match);

static struct mipi_dsi_driver jdi_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
			.name = "oplus23053_td4160_truly_hd_vdo_lcm_drv",
			.owner = THIS_MODULE,
			.of_match_table = jdi_of_match,
		},
};

module_mipi_dsi_driver(jdi_driver);

MODULE_AUTHOR("Elon Hsu <elon.hsu@mediatek.com>");
MODULE_DESCRIPTION("jdi r66451 CMD AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
