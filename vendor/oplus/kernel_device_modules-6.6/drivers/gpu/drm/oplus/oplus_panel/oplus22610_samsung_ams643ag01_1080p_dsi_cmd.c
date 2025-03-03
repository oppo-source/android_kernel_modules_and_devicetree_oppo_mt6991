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
#include <linux/delay.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/gpio/consumer.h>
#include <drm/drm_connector.h>
#include <linux/regulator/consumer.h>
#include <drm/drm_device.h>
#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>
#include <linux/of_graph.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/of_graph.h>
#include <soc/oplus/device_info.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_log.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif

#ifndef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "oplus22610_AMS643AG01_data_hw_roundedpattern.h"
#endif

u32 flag_writ;
u32 flag_hbm;
extern int is_fan53870_pmic(void);
extern int fan53870_ldo1_set_voltage(int set_uV);
extern int fan53870_ldo1_disable(void);
//extern void disp_aal_set_dre_en(int enable);
#define RAMLESS_AOD_PAYLOAD_SIZE	100
#define OPLUS_DC_BACKLIGHT_THRESHOLD 1200
extern int oplus_dc_alpha;
//extern int oplus_dc_enable_real;
extern int oplus_dc_enable;
//extern int exit_dc_flag;
extern unsigned long oplus_display_brightness;
extern unsigned long oplus_max_normal_brightness;
extern char send_cmd[RAMLESS_AOD_PAYLOAD_SIZE];
//extern int oplus_seed_bright_to_alpha(int brightness);
static int esd_brightness;
extern unsigned long seed_mode;
extern int oplus_dc_threshold;
extern void lcdinfo_notify(unsigned long val, void *v);

static bool aod_state = false;
#define LCM_DSI_CMD_MODE 1

#define REGFLAG_CMD             0xFFFA
#define REGFLAG_DELAY           0xFFFC
#define REGFLAG_UDELAY          0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD

#define BRIGHTNESS_MAX    4095
#define BRIGHTNESS_HALF   2047
#define MAX_NORMAL_BRIGHTNESS   2047
#define LCM_BRIGHTNESS_TYPE 2

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	// struct gpio_desc *bias_gpio;
	struct gpio_desc *bias_pos, *bias_neg;

	bool prepared;
	bool enabled;

	int error;

	bool hbm_en;
	bool hbm_wait;
	bool cv_state;
};

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[128];
};

struct ba {
	u32 brightness;
	u32 alpha;
};
/*
static struct ba brightness_seed_alpha_lut_dc[] = {
	{0, 2000},
	{10, 1959},
	{30, 1956},
	{60, 1953},
	{90, 1942},
	{100, 1938},
	{140, 1930},
	{200, 1920},
	{300, 1860},
	{400, 1790},
	{500, 1700},
	{600, 1560},
	{700, 1400},
	{800, 1210},
	{900, 980},
	{1000, 690},
	{1100, 380},
	{1200, 0},
};
*/
#if 0
static struct LCM_setting_table lcm_aod_to_normal[] = {
	/* Aod ctrl setting */
	{REGFLAG_CMD,3,{0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,5,{0xBB,0x11,0x0C,0x50,0x10}},
	{REGFLAG_CMD,3,{0xF0,0xA5,0xA5}},
	/* Aod mode off */
	{REGFLAG_CMD,3,{0xF0,0x5A,0x5A}},
	{REGFLAG_CMD,2,{0x91,0x02}},
	{REGFLAG_CMD,2,{0x53,0x20}},
	{REGFLAG_CMD,3,{0xF0,0xA5,0xA5}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_seed_setting[] = {
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2,{0x5D, 0x86}},
	{REGFLAG_CMD,2,{0x62, 0x00}},
	{REGFLAG_CMD,3,{0xB0, 0x2B, 0x62}},
	{REGFLAG_CMD,22,{0x62,0xDA,0x00,0x00,0x0C,0xD7,0x03,0x09,0x05,0xC4,0x16,0xFA,0xE2,0xF7,0x00,0xE4,0xE9,0xE6,0x03,0xFF,0xFF,0xFF}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
};

static struct LCM_setting_table lcm_seed_mode0[] = {
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,4,{0x72, 0x2C, 0x21, 0x00}},
	{REGFLAG_CMD,2,{0xF8, 0x00}},
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2,{0x5D, 0x86}},
	{REGFLAG_CMD,2,{0x62, 0x00}},
	{REGFLAG_CMD,3,{0xB0, 0x2B, 0x62}},
	{REGFLAG_CMD,22,{0x62,0xDA,0x00,0x00,0x0C,0xD7,0x03,0x09,0x05,0xC4,0x16,0xFA,0xE2,0xF7,0x00,0xE4,0xE9,0xE6,0x03,0xFF,0xFF,0xFF}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
	{REGFLAG_CMD,4,{0x72, 0x2C, 0x01, 0x00}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
};

static struct LCM_setting_table lcm_seed_mode1[] = {
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,4,{0x72, 0x2C, 0x21, 0x00}},
	{REGFLAG_CMD,2,{0xF8, 0x00}},
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2,{0x5D, 0x06}},
	{REGFLAG_CMD,2,{0x62, 0x00}},
	{REGFLAG_CMD,3,{0xB0, 0x01, 0x62}},
	{REGFLAG_CMD,22,{0x62,0xB0,0x02,0x06,0x3F,0xD3,0x15,0x05,0x09,0xB0,0x48,0xF2,0xDC,0xC4,0x07,0xC8,0xE9,0xE9,0x1D,0xFD,0xF5,0xE1}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
	{REGFLAG_CMD,4,{0x72, 0x2C, 0x01, 0x00}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
};

static struct LCM_setting_table lcm_seed_mode2[] = {
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,4,{0x72, 0x2C, 0x21, 0x00}},
	{REGFLAG_CMD,2,{0xF8, 0x00}},
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2,{0x5D, 0x86}},
	{REGFLAG_CMD,2,{0x62, 0x00}},
	{REGFLAG_CMD,3,{0xB0, 0x2B, 0x62}},
	{REGFLAG_CMD,22,{0x62,0xD8,0x00,0x04,0x00,0xFF,0x02,0x00,0x00,0xFF,0x18,0xFF,0xE4,0xFB,0x00,0xF0,0xF6,0xEA,0x01,0xFF,0xFF,0xFF}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
	{REGFLAG_CMD,4,{0x72, 0x2C, 0x01, 0x00}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
};

static struct LCM_setting_table lcm_seed_mode3[] = {
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,4,{0x72, 0x2C, 0x21, 0x00}},
	{REGFLAG_CMD,2,{0xF8, 0x00}},
	{REGFLAG_CMD,3,{0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2,{0x5D, 0x86}},
	{REGFLAG_CMD,2,{0x62, 0x00}},
	{REGFLAG_CMD,3,{0xB0, 0x2B, 0x62}},
	{REGFLAG_CMD,22,{0x62,0xDA,0x00,0x00,0x08,0xDB,0x05,0x0B,0x01,0xC6,0x16,0xFA,0xE2,0xF7,0x00,0xE4,0xE9,0xE6,0x03,0xFD,0xF5,0xE1}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
	{REGFLAG_CMD,4,{0x72, 0x2C, 0x01, 0x00}},
	{REGFLAG_CMD,3,{0xF0, 0xA5, 0xA5}},
};
#endif
#if 0
static struct LCM_setting_table lcm_setbrightness_normal[] = {
	{REGFLAG_CMD,2, {0x53,0x20}},
	{REGFLAG_CMD,3, {0x51,0x00,0x00}},
};
#endif
#if 0
static struct LCM_setting_table lcm_setbrightness_hbm[] = {
	{REGFLAG_CMD,3, {0x51,0x00,0x00}},
	{REGFLAG_CMD,2, {0x53,0xE0}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif
#if 0
static struct LCM_setting_table lcm_finger_HBM_on_setting[] = {
	/*HBM ON*/
	{REGFLAG_CMD,3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2, {0x53,0xE0}},
	{REGFLAG_CMD,3, {0x51,0x0F,0xFF}},
	{REGFLAG_CMD,3, {0xF0, 0xA5, 0xA5}},
};
#endif
#if 0
static struct LCM_setting_table lcm_normal_HBM_on_setting[] = {
	{REGFLAG_CMD,2, {0x53,0xE0}},
	{REGFLAG_CMD,3, {0x51,0x0F,0xFF}},
};
#endif
/*
static int panel_osc_freq_change(void *dsi, dcs_write_gce cb, void *handle, bool en)
{
	char osc_tb0[] = {0xF0, 0x5A, 0x5A};
	char osc_tb1[] = {0xFC, 0x5A, 0x5A};
	char osc_tb2[] = {0xB0, 0x66, 0xC5};
	char osc_tb3[] = {0xC5, 0x00, 0x8C};
	char osc_tb4[] = {0xB0, 0x2A, 0xC5};
	char osc_tb5[] = {0xC5, 0x0D, 0x10, 0x80, 0x45};
	char osc_tb6[] = {0xB0, 0x3E, 0xC5};
	char osc_tb7[] = {0xC5, 0x8E, 0x82};    //OSC 167M base MIPI 300M
	char osc_tb8[] = {0xF0, 0xA5, 0xA5};
	char osc_tb9[] = {0xFC, 0xA5, 0xA5};

	printk("debug for %s, %d\n", __func__, en);
	if (en == 0) {
		osc_tb7[1] = 0x8E, osc_tb7[2] = 0xFC;    //OSC 167M base MIPI 299M
	} else if (en == 1) {
		osc_tb7[1] = 0x90, osc_tb7[2] = 0xB2;    //OSC 169M base MIPI 299M
	}
	cb(dsi, handle, osc_tb0, ARRAY_SIZE(osc_tb0));
	cb(dsi, handle, osc_tb1, ARRAY_SIZE(osc_tb1));
	cb(dsi, handle, osc_tb2, ARRAY_SIZE(osc_tb2));
	cb(dsi, handle, osc_tb3, ARRAY_SIZE(osc_tb3));
	cb(dsi, handle, osc_tb4, ARRAY_SIZE(osc_tb4));
	cb(dsi, handle, osc_tb5, ARRAY_SIZE(osc_tb5));
	cb(dsi, handle, osc_tb6, ARRAY_SIZE(osc_tb6));
	cb(dsi, handle, osc_tb7, ARRAY_SIZE(osc_tb7));
	cb(dsi, handle, osc_tb8, ARRAY_SIZE(osc_tb8));
	cb(dsi, handle, osc_tb9, ARRAY_SIZE(osc_tb9));
	return 0;
}
*/
#define lcm_dcs_write_seq(ctx, seq...) \
({\
	const u8 d[] = { seq };\
	BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 128, "DCS sequence too big for stack");\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

#define lcm_dcs_write_seq_static(ctx, seq...) \
({\
	static const u8 d[] = { seq };\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})
/*
static int interpolate(int x, int xa, int xb, int ya, int yb)
{
	int bf, factor, plus;
	int sub = 0;

	bf = 2 * (yb - ya) * (x - xa) / (xb - xa);
	factor = bf / 2;
	plus = bf % 2;
	if ((xa - xb) && (yb - ya))
		sub = 2 * (x - xa) * (x - xb) / (yb - ya) / (xa - xb);

	return ya + factor + plus + sub;
}

static int oplus_seed_bright_to_alpha(int brightness)
{
	int level = ARRAY_SIZE(brightness_seed_alpha_lut_dc);
	int i = 0;
	int alpha;

	for (i = 0; i < ARRAY_SIZE(brightness_seed_alpha_lut_dc); i++) {
		if (brightness_seed_alpha_lut_dc[i].brightness >= brightness)
			break;
	}

	if (i == 0)
		alpha = brightness_seed_alpha_lut_dc[0].alpha;
	else if (i == level)
		alpha = brightness_seed_alpha_lut_dc[level - 1].alpha;
	else
		alpha = interpolate(brightness,
			brightness_seed_alpha_lut_dc[i-1].brightness,
			brightness_seed_alpha_lut_dc[i].brightness,
			brightness_seed_alpha_lut_dc[i-1].alpha,
			brightness_seed_alpha_lut_dc[i].alpha);

	return alpha;
}
*/
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
		dev_err(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
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
		pr_err("[error]error %d reading dcs seq:(%#x)\n", ret, cmd);
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
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
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
	ret = regulator_set_voltage(disp_bias_pos, 5400000, 5400000);
	if (ret < 0)
		pr_err("set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_set_voltage(disp_bias_neg, 5400000, 5400000);
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

//static struct regulator *disp_ldo3;
#if 0
static int lcm_panel_ldo3_regulator_init(struct device *dev)
{
	static int regulator_inited;
	int ret = 0;

	if (regulator_inited)
		return ret;
    pr_err("get lcm_panel_ldo3_regulator_init\n");

	/* please only get regulator once in a driver */
	disp_ldo3 = regulator_get(dev, "VMC");
	if (IS_ERR(disp_ldo3)) { /* handle return value */
		ret = PTR_ERR(disp_ldo3);
		pr_err("get ldo3 fail, error: %d\n", ret);
		return ret;
	}
	regulator_inited = 1;
	return ret; /* must be 0 */

}
#endif
#if 0
static int lcm_panel_ldo3_enable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_ldo3_regulator_init(dev);

	/* set voltage with min & max*/
	ret = regulator_set_voltage(disp_ldo3, 3000000, 3000000);
	if (ret < 0)
		pr_err("set voltage disp_ldo3 fail, ret = %d\n", ret);
	retval |= ret;

	/* enable regulator */
	ret = regulator_enable(disp_ldo3);
	if (ret < 0)
		pr_err("enable regulator disp_ldo3 fail, ret = %d\n", ret);
	retval |= ret;
    pr_err("get lcm_panel_ldo3_enable\n");

	return retval;
}
#endif
#if 0
static int lcm_panel_ldo3_disable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_ldo3_regulator_init(dev);

	ret = regulator_disable(disp_ldo3);
	if (ret < 0)
		pr_err("disable regulator disp_ldo3 fail, ret = %d\n", ret);
	retval |= ret;
    pr_err("disable regulator disp_ldo3\n");

	return retval;
}
#endif
static void lcm_panel_init(struct lcm *ctx)
{
	pr_err("debug for lcm %s\n", __func__);
	/*PPS Setting*/
	// lcm_dcs_write_seq_static(ctx,0x9E, 0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x09, 0x60, 0x04, 0x38,\
    //              0x00, 0x1E, 0x02, 0x1C, 0x02, 0x1C, 0x02, 0x00, 0x02, 0x0E,\
    //              0x00, 0x20, 0x02, 0xE3, 0x00, 0x07, 0x00, 0x0C, 0x03, 0x50,\
    //              0x03, 0x64, 0x18, 0x00, 0x10, 0xF0, 0x03, 0x0C, 0x20, 0x00,\
    //              0x06, 0x0B, 0x0B, 0x33, 0x0E, 0x1C, 0x2A, 0x38, 0x46, 0x54,\
    //              0x62, 0x69, 0x70, 0x77, 0x79, 0x7B, 0x7D, 0x7E, 0x01, 0x02,\
    //              0x01, 0x00, 0x09, 0x40, 0x09, 0xBE, 0x19, 0xFC, 0x19, 0xFA,\
    //              0x19, 0xF8, 0x1A, 0x38, 0x1A, 0x78, 0x1A, 0xB6, 0x2A, 0xF6,\
    //              0x2B, 0x34, 0x2B, 0x74, 0x3B, 0x74, 0x6B, 0xF4, 0x00, 0x00,\
    //              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
    //              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
    //              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
    //              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	// lcm_dcs_write_seq_static(ctx, 0xC2, 0x14);
	// lcm_dcs_write_seq_static(ctx, 0x9D, 0x01);
	/* Display On Setting */
	lcm_dcs_write_seq_static(ctx, 0x11);
	msleep(100);
	/* TE vsync ON */
	lcm_dcs_write_seq_static(ctx, 0x35, 0x00);
	// lcm_dcs_write_seq_static(ctx, 0x44, 0x09, 0x60);
	/* CASET/PASET Setting */
	lcm_dcs_write_seq_static(ctx, 0x2A, 0x00, 0x00, 0x04, 0x37);
	lcm_dcs_write_seq_static(ctx, 0x2B, 0x00, 0x00, 0x09, 0x5F);
	/* FQ CON Setting*/
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x27, 0xF2);
	lcm_dcs_write_seq_static(ctx, 0xF2, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	/* Freuency Setting */
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0x60, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xF7, 0x0F);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);

	/*Elvss offset setting*/
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0xB4, 0x63);
	lcm_dcs_write_seq_static(ctx, 0x63, 0xEA, 0xFE, 0xDF, 0xF0, 0xFF, 0x3F, 0xF6, 0xFF, 0x9F, 0xFC, 0xFF);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	/*Hbm off*/
	lcm_dcs_write_seq_static(ctx, 0x53, 0x20);
	/* Seed Setting */
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);

	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0x5D, 0x86);
	lcm_dcs_write_seq_static(ctx, 0x62, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x2B, 0x62);
	lcm_dcs_write_seq_static(ctx, 0x62, 0xDA,0x00,0x00,0x0C,0xD7,0x03,0x09,0x05,0xC4,0x16,0xFA,0xE2,0xF7,0x00,0xE4,0xE9,0xE6,0x03,0xFF,0xFF,0xFF);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	msleep(20);
	/* Display On*/
	lcm_dcs_write_seq_static(ctx, 0x29);
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	pr_err("debug for %s, ctx->enabled=%d\n", __func__, ctx->enabled);

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

	pr_err("debug for %s, ctx->prepared=%d\n", __func__, ctx->prepared);

	if (!ctx->prepared)
		return 0;

	lcm_dcs_write_seq_static(ctx, 0x28);
	msleep(10);
	lcm_dcs_write_seq_static(ctx, 0x10);
	msleep(110);

	ctx->error = 0;
	ctx->prepared = false;
// #if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
// 	lcm_panel_bias_disable();
// #endif
	ctx->hbm_en = false;
	return 0;
}

//extern int power_mode;
static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_err("debug for %s\n", __func__);

	if (ctx->prepared) {
		pr_err("%s failed!!\n", __func__);
		return 0;
	}
	aod_state = false;
/*
	if(power_mode == 2)
	{
		DDPINFO("%s + lcm_panel_init, resume status\n", __func__);
		lcm_panel_init(ctx);
	}
*/

	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;

// #ifdef PANEL_SUPPORT_READBACK
// 	lcm_panel_get_data(ctx);
// #endif

	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	pr_err("debug for %s, ctx->enabled=%d\n", __func__, ctx->enabled);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}

static const struct drm_display_mode default_mode = {
	.clock = 163070,
	.hdisplay = 1080,
	.hsync_start = 1080 + 8,
	.hsync_end = 1080 + 8 + 24,
	.htotal = 1080 + 8 + 24 + 8,
	.vdisplay = 2400,
	.vsync_start = 2400 + 8,
	.vsync_end = 2400 + 8 + 4,
	.vtotal = 2400 + 8 + 4 + 6,
	//.vrefresh = 60,
};

static const struct drm_display_mode performance_mode = {
	.clock = 243734,
	.hdisplay = 1080,
	.hsync_start = 1080 + 8,
	.hsync_end = 1080 + 8 + 24,
	.htotal = 1080 + 8 + 24 + 8,
	.vdisplay = 2400,
	.vsync_start = 2400 + 8,
	.vsync_end = 2400 + 8 + 4,
	.vtotal = 2400 + 8 + 4 + 6,
	//.vrefresh = 60,
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params = {
	.pll_clk = 850,
	.data_rate = 1700,
	.cust_esd_check = 0,
	.esd_check_enable = 0,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9D, .mask_list[0] = 0x9D,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0x0E, .count = 1, .para_list[0] = 0x80,
	},
	.lcm_esd_check_table[2] = {
		.cmd = 0x05, .count = 1, .para_list[0] = 0x00,
	},

	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
	},

	.dyn = {
		.switch_en = 1,
		.pll_clk = 873,
		.data_rate = 1746,
	},

	.vendor = "AMS643AG01_22610",
	.manufacture = "samsung2048_22610",

// #ifndef CONFIG_MTK_ROUND_CORNER_SUPPORT
//         .round_corner_en = 1,
//         .corner_pattern_height = ROUND_CORNER_H_TOP,
//         .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
//         .corner_pattern_tp_size = sizeof(top_rc_pattern),
//         .corner_pattern_lt_addr = (void *)top_rc_pattern,
// #endif
// #ifdef CONFIG_OPLUS_OFP_V2
// 	.oplus_ofp_need_keep_apart_backlight = false,
// 	.oplus_ofp_hbm_on_delay = 0,
// 	.oplus_ofp_pre_hbm_off_delay = 2,
// 	.oplus_ofp_hbm_off_delay = 9,
// #else
// 	.hbm_en_time = 1,
// 	.hbm_dis_time = 0,
// 	.oplus_need_hbm_wait = 1,
// 	.oplus_hbm_on_sync_with_flush = 1,
// 	.oplus_hbm_off_sync_with_flush = 0,
// 	.oplus_samsung_panel =0,
// 	//before hbm need wait
// 	.oplus_need_before_hbm_wait= 1,
// 	//delay TE num
// 	.before_hbm_en_time = 2,
// 	.before_hbm_dis_time = 0,
// 	//delay time: us
// 	.before_hbm_en_delay_time = 8000,
// #endif
// 	.oplus_dc_then_hbm_on = 0,
	.oplus_display_global_dre = 1,
// 	.oplus_disable_hdr_d65 = 1,
};

static struct mtk_panel_params ext_params_90hz = {
	.pll_clk = 850,
	.data_rate = 1700,
	.cust_esd_check = 0,
	.esd_check_enable = 0,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9D, .mask_list[0] = 0x9D,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0x0E, .count = 1, .para_list[0] = 0x80,
	},
	.lcm_esd_check_table[2] = {
		.cmd = 0x05, .count = 1, .para_list[0] = 0x00,
	},

	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
	},

	.dyn = {
		.switch_en = 1,
		.pll_clk = 873,
		.data_rate = 1746,
	},

	.vendor = "AMS643AG01_22610",
	.manufacture = "samsung2048_22610",
// #ifndef CONFIG_MTK_ROUND_CORNER_SUPPORT
//         .round_corner_en = 1,
//         .corner_pattern_height = ROUND_CORNER_H_TOP,
//         .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
//         .corner_pattern_tp_size = sizeof(top_rc_pattern),
//         .corner_pattern_lt_addr = (void *)top_rc_pattern,
// #endif
// #ifdef CONFIG_OPLUS_OFP_V2
// 	.oplus_ofp_need_keep_apart_backlight = true,
// 	.oplus_ofp_hbm_on_delay = 11,
// 	.oplus_ofp_pre_hbm_off_delay = 2,
// 	.oplus_ofp_hbm_off_delay = 11,
// #else
// 	.hbm_en_time = 1,
// 	.hbm_dis_time = 0,
// 	.oplus_need_hbm_wait = 0,
// 	.oplus_hbm_on_sync_with_flush = 0,
// 	.oplus_hbm_off_sync_with_flush = 0,
// 	.oplus_samsung_panel =0,
// 	//before hbm need wait
// 	.oplus_need_before_hbm_wait=1,
// 	//delay TE num
// 	.before_hbm_en_time = 2,
// 	.before_hbm_dis_time = 0,
// 	//delay time: us
// 	.before_hbm_en_delay_time =8000,
// #endif
// 	.oplus_dc_then_hbm_on = 0,
	.oplus_display_global_dre = 1,
// 	.oplus_disable_hdr_d65 = 1,
};
static int mtk_panel_ext_param_set(struct drm_panel *panel, struct drm_connector *connector,
			 unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	if (ext == NULL)
        return 1;

	pr_err("debug for lcm %s, mode = %d\n", __func__, mode);

	if (mode == 0)
		ext->params = &ext_params;
	else if (mode == 1)
		ext->params = &ext_params_90hz;
	else
		ret = 1;

	return ret;
}
#if 0
static int panel_set_seed(void *dsi, dcs_write_gce cb, void *handle, unsigned int seed_mode)
{
	int i, k;
	struct LCM_setting_table *seed_table = NULL;

	pr_err("debug for %s, seed_mode=%d\n", __func__, seed_mode);
	/*if (seed_mode == 102){ //DCI-P3
			for (i = 0; i < sizeof(lcm_seed_mode0)/sizeof(struct LCM_setting_table); i++){
				cb(dsi, handle, lcm_seed_mode0[i].para_list, lcm_seed_mode0[i].count);
			}
		} else if (seed_mode == 101){ //sRGB
			for (i = 0; i < sizeof(lcm_seed_mode1)/sizeof(struct LCM_setting_table); i++){
				cb(dsi, handle, lcm_seed_mode1[i].para_list, lcm_seed_mode1[i].count);
			}
		} else if (seed_mode == 100){
			for (i = 0; i < sizeof(lcm_seed_mode2)/sizeof(struct LCM_setting_table); i++){
				cb(dsi, handle, lcm_seed_mode2[i].para_list, lcm_seed_mode2[i].count);
			}
		} else if (seed_mode == 103){
			for (i = 0; i < sizeof(lcm_seed_mode3)/sizeof(struct LCM_setting_table); i++){
				cb(dsi, handle, lcm_seed_mode3[i].para_list, lcm_seed_mode3[i].count);
			}
		}*/

	if (seed_mode == 102)
		seed_table = kmemdup(lcm_seed_mode0, sizeof(lcm_seed_mode0), GFP_KERNEL);
	else if (seed_mode == 101)
		seed_table = kmemdup(lcm_seed_mode1, sizeof(lcm_seed_mode1), GFP_KERNEL);
	else if (seed_mode == 100)
		seed_table = kmemdup(lcm_seed_mode2, sizeof(lcm_seed_mode2), GFP_KERNEL);
	else if (seed_mode == 103)
		seed_table = kmemdup(lcm_seed_mode3, sizeof(lcm_seed_mode3), GFP_KERNEL);

	if (!seed_table)
		return 0;

	for (i = 0; i < sizeof(lcm_seed_mode0)/sizeof(lcm_seed_mode0[0]); i++) {
		if ((seed_table[i].para_list[0] == 0x62) && (seed_table[i].count == 22)){
			for (k = 1; k < seed_table[i].count; k++) {
				seed_table[i].para_list[k]= seed_table[i].para_list[k] * (2000 - oplus_dc_alpha) / 2000;
			}
		}
	}

	for (i = 0; i < sizeof(lcm_seed_mode0)/sizeof(lcm_seed_mode0[0]); i++){
		cb(dsi, handle, seed_table[i].para_list, seed_table[i].count);
	}

	kfree(seed_table);

	//memset(send_cmd, 0, RAMLESS_AOD_PAYLOAD_SIZE);
	return 0;
}
#endif
static void mode_switch_60_to_90(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (stage == BEFORE_DSI_POWERDOWN) {

	} else if (stage == AFTER_DSI_POWERON) {
		/* display on switch to 90hz */
		lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
		lcm_dcs_write_seq_static(ctx, 0x60, 0x08, 0x00);
		lcm_dcs_write_seq_static(ctx, 0xF7, 0x0F);
		lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
		usleep_range(3*1000, 4*1000);
    }
}

static void mode_switch_90_to_60(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (stage == BEFORE_DSI_POWERDOWN) {

	} else if (stage == AFTER_DSI_POWERON) {
		/* display on switch to 60hz */
		lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
		lcm_dcs_write_seq_static(ctx, 0x60, 0x00, 0x00);
		lcm_dcs_write_seq_static(ctx, 0xF7, 0x0F);
		lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
		usleep_range(3*1000, 4*1000);
	}
}
static int fps_temp = 0;

static int mode_switch(struct drm_panel *panel, struct drm_connector *connector, unsigned int cur_mode,
		unsigned int dst_mode, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	int ret = 0;

	if (cur_mode == dst_mode)
		return ret;

	pr_err("debug for lcm %s, dst_mode = %d, cur_mode = %d, stage = %d\n",
			__func__, dst_mode, cur_mode, stage);

	if (cur_mode == 0 && dst_mode == 1) { /* 60 switch to 90 */
		mode_switch_60_to_90(panel, stage);
		fps_temp = cur_mode;
	} else if (cur_mode == 1 && dst_mode == 0) { /* 90 switch to 60 */
		mode_switch_90_to_60(panel, stage);
		fps_temp = cur_mode;
	} else
		ret = 1;

	return ret;
}

#if 0
static int enter_aod(void *handle, int enter)
{
	/* enter & exit AOD cmd */
	return 0;
}
#endif
#if 0
static int panel_ext_reset(struct drm_panel *panel, int on)
{
#if 0
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
#endif
	return 0;
}
#endif
#if 0
static unsigned long panel_doze_get_mode_flags(struct drm_panel *panel, int doze_en)
{
	unsigned long mode_flags;

	//DDPINFO("%s doze_en:%d\n", __func__, doze_en);
	if (doze_en) {
		mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS;
	} else {
		mode_flags = MIPI_DSI_MODE_VIDEO
		       | MIPI_DSI_MODE_VIDEO_BURST
		       | MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS;
	}
    //pr_err("debug for %s, mode flags =%d, doze_en = %d\n", __func__,mode_flags,doze_en);
	return mode_flags;
}
#endif
/*static int panel_doze_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	DDPINFO("%s\n", __func__);

	//lcm_normal_to_aod(ctx);
	lcm_panel_init_cmd(ctx);
	return 0;
}*/
extern bool oplus_fp_notify_down_delay;
#if 0
static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	//struct lcm *ctx = panel_to_lcm(panel);
	unsigned int i=0;

	pr_err("debug for lcm %s\n", __func__);

	/* Switch back to VDO mode */
	for (i = 0; i < (sizeof(lcm_aod_to_normal) / sizeof(struct LCM_setting_table)); i++) {
		unsigned cmd;
		cmd = lcm_aod_to_normal[i].cmd;

		switch (cmd) {

			case REGFLAG_DELAY:
					msleep(lcm_aod_to_normal[i].count);
				break;

			case REGFLAG_UDELAY:
				udelay(lcm_aod_to_normal[i].count);
				break;

			case REGFLAG_END_OF_TABLE:
				break;

			default:
				//judge last backlight hbm or not
				if (flag_hbm == 1) {
					if (lcm_aod_to_normal[i].para_list[0] == 0x53) {
						//send temp 0x53 order, in order not to change original lcm_aod_to_normal
						unsigned char tmp1[] = {0x53, 0xE0};
						cb(dsi, handle, tmp1, ARRAY_SIZE(tmp1));
						continue;
					}
				}
				cb(dsi, handle, lcm_aod_to_normal[i].para_list, lcm_aod_to_normal[i].count);
		}
	}
	panel_set_seed(dsi, cb, handle, seed_mode);
	aod_state = false;
	return 0;
}
#endif

/*static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i=0;
	pr_err("debug for lcm %s\n", __func__);
	aod_state = true;

	for (i = 0; i < (sizeof(lcm_normal_to_aod_sam) / sizeof(struct LCM_setting_table)); i++) {
		unsigned cmd;
		cmd = lcm_normal_to_aod_sam[i].cmd;

		switch (cmd) {

			case REGFLAG_DELAY:
				msleep(lcm_normal_to_aod_sam[i].count);
				break;

			case REGFLAG_UDELAY:
				udelay(lcm_normal_to_aod_sam[i].count);
				break;

			case REGFLAG_END_OF_TABLE:
				break;

			default:
				cb(dsi, handle, lcm_normal_to_aod_sam[i].para_list, lcm_normal_to_aod_sam[i].count);
		}
	}

	return 0;
}*/
#if 0
static int panel_doze_enable_start(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	int cmd = 0;
	pr_err("debug for lcm %s\n", __func__);

	cmd = 0x28;
	cb(dsi, handle, &cmd, 1);
	//cmd = 0x10;
	//cb(dsi, handle, &cmd, 1);
	msleep(16);

	return 0;
}
#endif
#if 0
static int panel_doze_enable_end(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	int cmd = 0;
	int send_buf[3];

	pr_err("debug for lcm %s\n", __func__);

	cmd = 0x29;
	cb(dsi, handle, &cmd, 1);
	send_buf[0] = 0xF0;
	send_buf[1] = 0x5A;
	send_buf[2] = 0x5A;
	cb(dsi, handle, send_buf, 3);
	send_buf[0] = 0xF2;
	send_buf[1] = 0x0F;
	cb(dsi, handle, send_buf, 2);
	send_buf[0] = 0xF0;
	send_buf[1] = 0xA5;
	send_buf[2] = 0xA5;
	cb(dsi, handle, send_buf, 3);

	return 0;
}
#endif
#if 0
static int panel_doze_area_set(void *dsi, dcs_write_gce cb, void *handle)
{
	int cmd = 0;

	pr_err("debug for lcm %s\n", __func__);

#if 0
	cmd = 0x28;
	cb(dsi, handle, &cmd, 1);
	msleep(12);
#endif
	cb(dsi, handle, send_cmd, ARRAY_SIZE(send_cmd));

	//memset(send_cmd, 0, RAMLESS_AOD_PAYLOAD_SIZE);
	return 0;
}
#endif
#if 0
static int panel_doze_post_disp_on(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{

	pr_err("debug for lcm %s\n", __func__);

	char post_backlight_on1[] = {0x29};

	cb(dsi, handle, post_backlight_on1, ARRAY_SIZE(post_backlight_on1));

	return 0;
}
#endif
#if 0
static int panel_doze_post_disp_off(void *dsi, dcs_write_gce cb, void *handle)
{

	int cmd = 0;

	pr_err("debug for lcm %s\n", __func__);

	cmd = 0x28;
	cb(dsi, handle, &cmd, 1);

	return 0;
}
#endif
#if 0
static int oplus_lcm_dc_backlight(void *dsi, dcs_write_gce cb,void *handle,unsigned int level,int hbm_en)
{
	//int i, k;
	//struct LCM_setting_table *seed_table = NULL;
	int seed_alpha = oplus_seed_bright_to_alpha(level);
	static int last_level = OPLUS_DC_BACKLIGHT_THRESHOLD;
	static int last_seed_alpha = 0;
	/*if (!oplus_dc_enable_real || hbm_en || level >= OPLUS_DC_BACKLIGHT_THRESHOLD ||
	    level < 4) {
		goto dc_disable;
	}
*/
	if (((level < last_level) && (seed_alpha < last_seed_alpha)) ||
		((level > last_level) && (seed_alpha > last_seed_alpha))){
		seed_alpha = last_seed_alpha;
	}
	last_level = level;
	last_seed_alpha = seed_alpha;

	if (oplus_dc_alpha == seed_alpha)
		goto dc_enable;
/*
	if (seed_mode == 102)
		seed_table = kmemdup(lcm_seed_mode0, sizeof(lcm_seed_mode0), GFP_KERNEL);
	else if (seed_mode == 101)
		seed_table = kmemdup(lcm_seed_mode1, sizeof(lcm_seed_mode1), GFP_KERNEL);
	else if (seed_mode == 100)
		seed_table = kmemdup(lcm_seed_mode2, sizeof(lcm_seed_mode2), GFP_KERNEL);
	else if (seed_mode == 103)
		seed_table = kmemdup(lcm_seed_mode3, sizeof(lcm_seed_mode3), GFP_KERNEL);

	if (!seed_table)
		goto dc_disable;

	for (i = 0; i < sizeof(lcm_seed_mode0)/sizeof(lcm_seed_mode0[0]); i++) {
		if ((seed_table[i].para_list[0] == 0x62) && (seed_table[i].count == 22)){
			for (k = 1; k < seed_table[i].count; k++) {
				seed_table[i].para_list[k]= seed_table[i].para_list[k] * (2000 - seed_alpha) / 2000;
			}
		}
	}

	for (i = 0; i < sizeof(lcm_seed_mode0)/sizeof(lcm_seed_mode0[0]); i++){
		cb(dsi, handle, seed_table[i].para_list, seed_table[i].count);
	}

	kfree(seed_table);
	if (!oplus_dc_alpha)
		pr_err("Enter DC");
*/
	oplus_dc_alpha = seed_alpha;

dc_enable:
	return OPLUS_DC_BACKLIGHT_THRESHOLD;

dc_disable:
	if (oplus_dc_alpha && level != 0) {
		/*if (seed_mode == 102){ //DCI-P3
			for (i = 0; i < sizeof(lcm_seed_mode0)/sizeof(struct LCM_setting_table); i++){
				cb(dsi, handle, lcm_seed_mode0[i].para_list, lcm_seed_mode0[i].count);
			}
		} else if (seed_mode == 101){ //sRGB
			for (i = 0; i < sizeof(lcm_seed_mode1)/sizeof(struct LCM_setting_table); i++){
				cb(dsi, handle, lcm_seed_mode1[i].para_list, lcm_seed_mode1[i].count);
			}
		} else if (seed_mode == 100){
			for (i = 0; i < sizeof(lcm_seed_mode2)/sizeof(struct LCM_setting_table); i++){
				cb(dsi, handle, lcm_seed_mode2[i].para_list, lcm_seed_mode2[i].count);
			}
		} else if (seed_mode == 103){
			for (i = 0; i < sizeof(lcm_seed_mode3)/sizeof(struct LCM_setting_table); i++){
				cb(dsi, handle, lcm_seed_mode3[i].para_list, lcm_seed_mode3[i].count);
			}
		}*/
       //exit_dc_flag = 1;

		pr_err("exit DC");
	}

	oplus_dc_alpha = 0;

	return level;
}
#endif
#if 0
static int oplus_lcm_dc_post_exitd(void *dsi, dcs_write_gce cb,void *handle)
{
	int i;

	pr_err("debug for lcm, oplus_dc_enable = %d\n", oplus_dc_enable);
	if (seed_mode == 102){ //DCI-P3
		for (i = 0; i < sizeof(lcm_seed_mode0)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_seed_mode0[i].para_list, lcm_seed_mode0[i].count);
		}
	} else if (seed_mode == 101){ //sRGB
		for (i = 0; i < sizeof(lcm_seed_mode1)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_seed_mode1[i].para_list, lcm_seed_mode1[i].count);
		}
	} else if (seed_mode == 100){
		for (i = 0; i < sizeof(lcm_seed_mode2)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_seed_mode2[i].para_list, lcm_seed_mode2[i].count);
		}
	} else if (seed_mode == 103){
		for (i = 0; i < sizeof(lcm_seed_mode3)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_seed_mode3[i].para_list, lcm_seed_mode3[i].count);
		}
	}
	return 0;
}
#endif

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int level)
{
	unsigned int mapped_level = 0;
	char bl_tb0[] = {0x51, 0x03, 0xFF};
	char bl_tb1[] = {0x53, 0x20};

	char post_backlight_on1[] = {0x29};

	if (!cb)
		return -1;
	if (level > 2047 && level <= 3727)
		level = 3727;
	//mapped_level = oplus_lcm_dc_backlight(dsi,cb,handle, level, 0);
	mapped_level = level;
	bl_tb0[1] = mapped_level >> 8;
	bl_tb0[2] = mapped_level & 0xFF;
	esd_brightness = level;
	DDPPR_ERR("flag_writ=%d,mapped_level=%d\n",flag_writ,mapped_level);

	pr_err("debug for display panel backlight value,func:=%s,level :=%d, mapped_level := %d\n", __func__, level, mapped_level);
	if (mapped_level > 1) {
		lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &mapped_level);
	}
	if(level == 1 || mapped_level == 1){
		pr_info("enter aod mode, ignore set backlight to 1\n");
		if (aod_state == 1) {
			cb(dsi, handle, post_backlight_on1, ARRAY_SIZE(post_backlight_on1));
		}
	} else if (level <= BRIGHTNESS_HALF){
	    if(flag_hbm == 1){
			bl_tb1[1] = 0x20;
			cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
			flag_hbm = 0;
		}
	    cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	} else if (level > BRIGHTNESS_HALF && level <= BRIGHTNESS_MAX){
	    if(flag_hbm == 0){
			bl_tb1[1] = 0xE0;
			cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
			flag_hbm = 1;
		}
	    cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	}

	return 0;

}
#if 0
static int lcm_set_dc_backlight(void *dsi, dcs_write_gce cb,
           void *handle, unsigned int level)
{
	char bl_tb0[] = {0x51, 0x07, 0xFF};

	//pr_err("lcm_set_dc_backlight ,backlight = %d\n", oplus_display_brightness);
	bl_tb0[1] = oplus_display_brightness >> 8;
	bl_tb0[2] = oplus_display_brightness & 0xFF;

	oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness, 1);
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 0;

}
#endif
#if 0
static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb,
		void *handle)
{
	char bl_tb0[] = {0x51, 0x03, 0xff};

	//pr_err("%s esd_backlight = %d\n", __func__, esd_brightness);
	bl_tb0[1] = esd_brightness >> 8;
	bl_tb0[2] = esd_brightness & 0xFF;
	if (!cb)
		return -1;
	pr_err("%s bl_tb0[1]=%x, bl_tb0[2]=%x\n", __func__, bl_tb0[1], bl_tb0[2]);
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 1;
}
#endif
static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_err("debug for lcm %s, ctx->prepared=%d\n", __func__, ctx->prepared);

	if (ctx->prepared)
		return 0;

	// ctx->bias_gpio = devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
	// gpiod_set_value(ctx->bias_gpio, 1);
	// devm_gpiod_put(ctx->dev, ctx->bias_gpio);
	// msleep(5);
	// lcm_panel_ldo3_enable(ctx->dev);
	// msleep(5);

	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);
	msleep(15);
	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
	msleep(15);

	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(10);
	gpiod_set_value(ctx->reset_gpio, 0);
	msleep(10);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(10);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	usleep_range(5000, 5100);
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_err("debug for lcm %s  ctx->prepared %d \n", __func__, ctx->prepared);

	if (ctx->prepared)
		return 0;

	usleep_range(5000, 5100);
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	msleep(5);

	// ctx->bias_gpio = devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
	// gpiod_set_value(ctx->bias_gpio, 0);
	// msleep(5);
	// lcm_panel_ldo3_disable(ctx->dev);
	// msleep(10);
	// devm_gpiod_put(ctx->dev, ctx->bias_gpio);
	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_pos, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);
	msleep(5);
	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_neg, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
	msleep(10);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	msleep(13);

	return 0;
}
#if 0
static int lcm_panel_disp_off(void *dsi, dcs_write_gce cb, void *handle)
{
	int cmd = 0;

	DDPINFO("%s\n", __func__);

	cmd = 0x28;
	cb(dsi, handle, &cmd, 1);
	msleep(20);

	cmd = 0x10;
	cb(dsi, handle, &cmd, 1);
	msleep(110);

	return 0;
}
#endif
#if 0
static void lcm_setbrightness(void *dsi,
			      dcs_write_gce cb, void *handle, unsigned int level)
{
	unsigned int BL_MSB = 0;
	unsigned int BL_LSB = 0;
	unsigned int hbm_brightness = 0;
	int i = 0;

	printk("[soso] %s level is %d\n", __func__, level);

	if (level > BRIGHTNESS_HALF) {
		hbm_brightness = level - BRIGHTNESS_HALF;
		BL_LSB = hbm_brightness >> 8;
		BL_MSB = hbm_brightness & 0xFF;

		//lcm_setbrightness_hbm[0].para_list[1] = BL_LSB;
		//lcm_setbrightness_hbm[0].para_list[2] = BL_MSB;
/*
		for (i = 0; i < sizeof(lcm_setbrightness_hbm)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_setbrightness_hbm[i].para_list, lcm_setbrightness_hbm[i].count);
		}
*/
	} else {
		BL_LSB = level >> 8;
		BL_MSB = level & 0xFF;

		lcm_setbrightness_normal[1].para_list[1] = BL_LSB;
		lcm_setbrightness_normal[1].para_list[2] = BL_MSB;

		for (i = 0; i < sizeof(lcm_setbrightness_normal)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_setbrightness_normal[i].para_list, lcm_setbrightness_normal[i].count);
		}
	}
}
#endif
#if 0
static int oplus_send_cmd_before_dsi_read(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	pr_err("oplus_send_cmd_before_dsi_read");
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x07, 0xD8);
	return 0;
}
#endif
#if 0
static int lcm_set_hbm(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int hbm_mode)
{
	int i = 0;
	//int level = 0;
	if (!cb)
		return -1;

	pr_err("oplus_display_brightness= %ld, hbm_mode=%u\n", oplus_display_brightness, hbm_mode);

	if(hbm_mode == 1) {
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
	} else if (hbm_mode == 0) {
		lcm_setbrightness(dsi, cb, handle, oplus_display_brightness);
        //printk("[soso] %s : %d ! backlight %d !\n",__func__, hbm_mode, oplus_display_brightness);
	}

	return 0;
}
#endif
#if 0
static int panel_hbm_set_cmdq(struct drm_panel *panel, void *dsi,
			      dcs_write_gce cb, void *handle, bool en)
{
	//char hbm_tb[] = {0x53, 0xe0};
	struct lcm *ctx = panel_to_lcm(panel);
	int i = 0;
	if (!cb)
		return -1;
	if (ctx->hbm_en == en)
		goto done;

	pr_err("debug for oplus_display_brightness= %ld, en=%u\n", oplus_display_brightness, en);

	if(en == 1) {
		oplus_lcm_dc_backlight(dsi,cb,handle, oplus_display_brightness, 1);
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list, lcm_finger_HBM_on_setting[i].count);
		}
	} else if (en == 0) {
		lcm_setbrightness(dsi, cb, handle, oplus_display_brightness);
		if (oplus_display_brightness <= BRIGHTNESS_HALF)
			flag_hbm = 0;
		else
			flag_hbm = 1;
		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
		//printk("[soso] %s : %d !  backlight %d !\n",__func__, en, oplus_display_brightness);
	}
	lcdinfo_notify(1, &en);
	ctx->hbm_en = en;
	ctx->hbm_wait = true;
done:
	return 0;
}
#endif
/*
static void panel_hbm_get_state(struct drm_panel *panel, bool *state)
{
	struct lcm *ctx = panel_to_lcm(panel);

	*state = ctx->hbm_en;
}
*/
/*
static void panel_hbm_set_state(struct drm_panel *panel, bool state)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->hbm_en = state;
}
*/
/*
static void panel_hbm_get_wait_state(struct drm_panel *panel, bool *wait)
{
	struct lcm *ctx = panel_to_lcm(panel);

	*wait = ctx->hbm_wait;
}
*/
/*
static bool panel_hbm_set_wait_state(struct drm_panel *panel, bool wait)
{
	struct lcm *ctx = panel_to_lcm(panel);
	bool old = ctx->hbm_wait;

	ctx->hbm_wait = wait;
	return old;
}
*/
#if 0
static bool panel_no_video_cmd_switch_state(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	ctx->cv_state = true;

    return ctx->cv_state;
}
#endif
static struct mtk_panel_funcs ext_funcs = {
	//.aod = enter_aod,
	//.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	//.doze_enable = panel_doze_enable,
	//.doze_enable_start = panel_doze_enable_start,
	//.doze_enable_end = panel_doze_enable_end,
	//.doze_disable = panel_doze_disable,
	//.doze_area = panel_doze_area,
	//.doze_get_mode_flags = panel_doze_get_mode_flags,
	//.doze_post_disp_on = panel_doze_post_disp_on,
	//.doze_post_disp_off = panel_doze_post_disp_off,
	//.set_hbm = lcm_set_hbm,
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
	//.panel_disp_off = lcm_panel_disp_off,
	//.hbm_set_cmdq = panel_hbm_set_cmdq,
	#ifndef CONFIG_OPLUS_OFP_V2
	//.hbm_get_state = panel_hbm_get_state,
	//.hbm_set_state = panel_hbm_set_state,
	//.hbm_get_wait_state = panel_hbm_get_wait_state,
	//.hbm_set_wait_state = panel_hbm_set_wait_state,
	#endif
	//.doze_area_set = panel_doze_area_set,
	//.panel_no_cv_switch = panel_no_video_cmd_switch_state,
	.ext_param_set = mtk_panel_ext_param_set,
	//.esd_backlight_recovery = oplus_esd_backlight_recovery,
	.mode_switch = mode_switch,
	//.send_cmd_before_dsi_read = oplus_send_cmd_before_dsi_read,
	//.set_seed = panel_set_seed,
	//.lcm_osc_change = panel_osc_freq_change,
	//.set_dc_backlight = lcm_set_dc_backlight,
	//.lcm_dc_post_exitd = oplus_lcm_dc_post_exitd,
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

static int lcm_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	struct drm_display_mode *mode2;

	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		pr_err("failed to add mode %ux%ux\n",
			default_mode.hdisplay, default_mode.vdisplay);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	mode2 = drm_mode_duplicate(connector->dev, &performance_mode);

	//pr_err(" debug for lcm_get_modes= %s, mode = %d, mode2 = %d\n", __func__, mode, mode2);
	if (!mode2) {
		pr_err(" failed to add mode %ux%ux\n",
			performance_mode.hdisplay,
			performance_mode.vdisplay);
		return -ENOMEM;
	}

	drm_mode_set_name(mode2);
	mode2->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode2);
	connector->display_info.width_mm = 67;
	connector->display_info.height_mm = 149;

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
	pr_info("begin!\n");

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;

#if (LCM_DSI_CMD_MODE)
	dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS;
#else
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST
			 | MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS;

#endif
	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	// ctx->bias_gpio = devm_gpiod_get(ctx->dev, "bias", GPIOD_OUT_HIGH);
	// gpiod_set_value(ctx->bias_gpio, 1);
	// devm_gpiod_put(ctx->dev, ctx->bias_gpio);
	// msleep(2);
	// lcm_panel_ldo3_enable(ctx->dev);
	// msleep(5);

	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(dev, " cannot get bias0 %ld\n",
			PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);
	msleep(5);
	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(dev, " cannot get bias1 %ld\n",
			PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
	msleep(5);

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(dev, " cannot get reset-gpios %ld\n",
			PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);
	ctx->prepared = true;
	ctx->enabled = true;

	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);
	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif
	register_device_proc("lcd", "AMS643AG01_22610", "samsung2048_22610");
	ctx->hbm_en = false;
	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
	//disp_aal_set_dre_en(1);
	pr_info("%s +\n", __func__);

	pr_info("%s -\n", __func__);

	return ret;
}

static void lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);
#if defined(CONFIG_MTK_PANEL_EXT)
	struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);
#endif
	pr_info("begin!\n");
	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_detach(ext_ctx);
	mtk_panel_remove(ext_ctx);
#endif
}

static const struct of_device_id lcm_of_match[] = {
	{ .compatible = "oplus22610_samsung_ams643ag01_1080p_dsi_cmd", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "oplus22610_samsung_ams643ag01_1080p_dsi_cmd",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);
MODULE_AUTHOR("oplus");
MODULE_DESCRIPTION("oplus22610_samsung_ams643ag01_1080p_dsi_cmd,OLED,Driver");
MODULE_LICENSE("GPL v2");
