// SPDX-License-Identifier: GPL-2.0-or-later
/*
* leds-aw22xxx.c   aw22xxx led module
*
* Copyright (c) 2017 Shanghai Awinic Technology Co., Ltd. All Rights Reserved
*
* This program is free software; you can redistribute  it and/or modify it
* under  the terms of  the GNU General  Public License as published by the
* Free Software Foundation;  either version 2 of the  License, or (at your
* option) any later version.
*/

#define AW22XXX_TAG "<aw22xxx> "
#define pr_fmt(fmt) "%s "fmt, AW22XXX_TAG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/leds.h>
#include <linux/printk.h>
#include <linux/bitops.h>
#include <linux/bitmap.h>
#include <linux/string.h>
#include <linux/timekeeping.h>

#include "leds-aw22xxx.h"
#include "leds-aw22xxx-reg.h"
#define CREATE_TRACE_POINTS
#include "trace_led_fault.h"

#ifdef OPLUS_FEATURE_MISC_BASIC
#include <linux/regulator/consumer.h>
#endif
/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW22XXX_I2C_NAME "aw22xxx_led"

#define AW22XXX_DRIVER_VERSION "v1.7.0"

#define AW_I2C_RETRIES 2
#define AW_I2C_RETRY_DELAY 1
#define AW_READ_CHIPID_RETRIES 2
#define AW_READ_CHIPID_RETRY_DELAY 1

#define FW_ACTION_HOTPLUG 1

static bool force_check_error = false;
module_param(force_check_error, bool, 0644);
MODULE_PARM_DESC(force_check_error, "Force abnormal reset check error for debug");

/******************************************************
 *
 * aw22xxx led parameter
 *
 ******************************************************/

static char *aw22xxx_fw_name = "aw22xxx_fw.bin";
#ifndef OPLUS_FEATURE_MISC_BASIC
static char aw22xxx_cfg_name[][AW22XXX_CFG_NAME_MAX] = {
	{"aw22xxx_cfg_led_off.bin"},
	{"aw22xxx_cfg_led_on.bin"},
	{"aw22xxx_cfg_led_breath.bin"},
	{"aw22xxx_cfg_led_blink.bin"},
	{"aw22xxx_cfg_led_skyline.bin"},
	{"aw22xxx_cfg_led_flower.bin"},
	{"aw22xxx_cfg_audio_skyline.bin"},
	{"aw22xxx_cfg_audio_flower.bin"},
};
#endif

#define AW22XXX_IMAX_NAME_MAX	   32
static char aw22xxx_imax_name[][AW22XXX_IMAX_NAME_MAX] = {
	{"AW22XXX_IMAX_2mA"},
	{"AW22XXX_IMAX_3mA"},
	{"AW22XXX_IMAX_4mA"},
	{"AW22XXX_IMAX_6mA"},
	{"AW22XXX_IMAX_9mA"},
	{"AW22XXX_IMAX_10mA"},
	{"AW22XXX_IMAX_15mA"},
	{"AW22XXX_IMAX_20mA"},
	{"AW22XXX_IMAX_30mA"},
	{"AW22XXX_IMAX_40mA"},
	{"AW22XXX_IMAX_45mA"},
	{"AW22XXX_IMAX_60mA"},
	{"AW22XXX_IMAX_75mA"},
};
static char aw22xxx_imax_code[] = {
	AW22XXX_IMAX_2mA,
	AW22XXX_IMAX_3mA,
	AW22XXX_IMAX_4mA,
	AW22XXX_IMAX_6mA,
	AW22XXX_IMAX_9mA,
	AW22XXX_IMAX_10mA,
	AW22XXX_IMAX_15mA,
	AW22XXX_IMAX_20mA,
	AW22XXX_IMAX_30mA,
	AW22XXX_IMAX_40mA,
	AW22XXX_IMAX_45mA,
	AW22XXX_IMAX_60mA,
	AW22XXX_IMAX_75mA,
};
static int aw22xxx_fw_update(struct aw22xxx *aw22xxx);
static void aw22xxx_recovery_work_routine(struct work_struct *work);
static void aw22xxx_music_work_routine(struct work_struct *work);
/******************************************************
 *
 * aw22xxx i2c write/read
 *
 ******************************************************/
static int
aw22xxx_i2c_write(struct aw22xxx *aw22xxx, unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_write_byte_data(aw22xxx->i2c, reg_addr, reg_data);
		if (ret < 0)
			pr_err("%s: i2c_write cnt=%d error=%d\n", __func__, cnt, ret);
		else
			break;
		cnt++;
		usleep_range(2000, 3000);
	}

	return ret;
}

static int
aw22xxx_i2c_read(struct aw22xxx *aw22xxx, unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(aw22xxx->i2c, reg_addr);
		if (ret < 0) {
			pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt, ret);
		} else {
			*reg_data = ret;
			break;
		}
		cnt++;
		usleep_range(2000, 3000);
	}

	return ret;
}

static int aw22xxx_i2c_write_bits(struct aw22xxx *aw22xxx,
		unsigned char reg_addr, unsigned char mask, unsigned char reg_data)
{
	unsigned char reg_val = 0;

	aw22xxx_i2c_read(aw22xxx, reg_addr, &reg_val);
	reg_val &= mask;
	reg_val |= (reg_data & (~mask));
	aw22xxx_i2c_write(aw22xxx, reg_addr, reg_val);

	return 0;
}

#ifdef AW22XXX_FLASH_I2C_WRITES
static int aw22xxx_i2c_writes(struct aw22xxx *aw22xxx,
		unsigned char reg_addr, unsigned char *buf, unsigned int len)
{
	int ret = -1;
	unsigned char *data;

	data = kmalloc(len+1, GFP_KERNEL);
	if (data == NULL)
		return  -ENOMEM;

	data[0] = reg_addr;
	memcpy(&data[1], buf, len);

	ret = i2c_master_send(aw22xxx->i2c, data, len+1);
	if (ret < 0)
		pr_err("%s: i2c master send error\n", __func__);

	kfree(data);

	return ret;
}
#endif

/*****************************************************
 *
 * aw22xxx led cfg
 *
 *****************************************************/
static int aw22xxx_reg_page_cfg(struct aw22xxx *aw22xxx, unsigned char page)
{
	aw22xxx_i2c_write(aw22xxx, REG_PAGE, page);
	return 0;
}

static int aw22xxx_sw_reset(struct aw22xxx *aw22xxx)
{
	aw22xxx_i2c_write(aw22xxx, REG_SRST, AW22XXX_SRSTW);
	usleep_range(2000, 3000);
	return 0;
}

static int aw22xxx_chip_enable(struct aw22xxx *aw22xxx, bool flag)
{
	if (flag) {
		aw22xxx_i2c_write_bits(aw22xxx, REG_GCR,
				BIT_GCR_CHIPEN_MASK, BIT_GCR_CHIPEN_ENABLE);
	} else {
		aw22xxx_i2c_write_bits(aw22xxx, REG_GCR,
				BIT_GCR_CHIPEN_MASK, BIT_GCR_CHIPEN_DISABLE);
	}
	usleep_range(2000, 3000);
	return 0;
}

static int aw22xxx_mcu_reset(struct aw22xxx *aw22xxx, bool flag)
{
	if (flag) {
		aw22xxx_i2c_write_bits(aw22xxx, REG_MCUCTR,
				BIT_MCUCTR_MCU_RESET_MASK, BIT_MCUCTR_MCU_RESET_ENABLE);
	} else {
		aw22xxx_i2c_write_bits(aw22xxx, REG_MCUCTR,
				BIT_MCUCTR_MCU_RESET_MASK, BIT_MCUCTR_MCU_RESET_DISABLE);
	}
	return 0;
}

static int aw22xxx_mcu_enable(struct aw22xxx *aw22xxx, bool flag)
{
	if (flag) {
		aw22xxx_i2c_write_bits(aw22xxx, REG_MCUCTR,
				BIT_MCUCTR_MCU_WORK_MASK, BIT_MCUCTR_MCU_WORK_ENABLE);
	} else {
		aw22xxx_i2c_write_bits(aw22xxx, REG_MCUCTR,
				BIT_MCUCTR_MCU_WORK_MASK, BIT_MCUCTR_MCU_WORK_DISABLE);
	}
	return 0;
}

static int aw22xxx_led_task0_cfg(struct aw22xxx *aw22xxx, unsigned char task)
{
	aw22xxx_i2c_write(aw22xxx, REG_TASK0, task);
	return 0;
}

static int aw22xxx_led_task1_cfg(struct aw22xxx *aw22xxx, unsigned char task)
{
	aw22xxx_i2c_write(aw22xxx, REG_TASK1, task);
	return 0;
}

static int aw22xxx_imax_cfg(struct aw22xxx *aw22xxx, unsigned char imax)
{
	if (imax > 0x0f)
		imax = 0x0f;

	aw22xxx_reg_page_cfg(aw22xxx, AW22XXX_REG_PAGE0);
	aw22xxx_i2c_write(aw22xxx, REG_IMAX, imax);

	return 0;
}

static int aw22xxx_dbgctr_cfg(struct aw22xxx *aw22xxx, unsigned char cfg)
{
	if (cfg >= (AW22XXX_DBGCTR_MAX-1))
		cfg = AW22XXX_DBGCTR_NORMAL;

	aw22xxx_i2c_write(aw22xxx, REG_DBGCTR, cfg);

	return 0;
}

static int aw22xxx_addr_cfg(struct aw22xxx *aw22xxx, unsigned int addr)
{
	aw22xxx_i2c_write(aw22xxx, REG_ADDR1, (unsigned char)((addr>>0)&0xff));
	aw22xxx_i2c_write(aw22xxx, REG_ADDR2, (unsigned char)((addr>>8)&0xff));

	return 0;
}

static int aw22xxx_data_cfg(struct aw22xxx *aw22xxx, unsigned int data)
{
	aw22xxx_i2c_write(aw22xxx, REG_DATA, data);

	return 0;
}

static int aw22xxx_led_display(struct aw22xxx *aw22xxx)
{
	aw22xxx_addr_cfg(aw22xxx, 0x00e1);
	aw22xxx_dbgctr_cfg(aw22xxx, AW22XXX_DBGCTR_SFR);
	aw22xxx_data_cfg(aw22xxx, 0x3d);
	aw22xxx_dbgctr_cfg(aw22xxx, AW22XXX_DBGCTR_NORMAL);
	return 0;
}

static int aw22xxx_led_off(struct aw22xxx *aw22xxx)
{
	aw22xxx_led_task0_cfg(aw22xxx, 0xff);
	aw22xxx_mcu_reset(aw22xxx, true);
	return 0;
}

static void aw22xxx_brightness_work(struct work_struct *work)
{
	struct aw22xxx *aw22xxx = container_of(work, struct aw22xxx, brightness_work);

	aw22xxx_led_off(aw22xxx);
	aw22xxx_chip_enable(aw22xxx, false);
	if (aw22xxx->cdev.brightness) {
		aw22xxx_chip_enable(aw22xxx, true);
		aw22xxx_mcu_enable(aw22xxx, true);

		aw22xxx_imax_cfg(aw22xxx, (unsigned char)aw22xxx->imax);
		aw22xxx_led_display(aw22xxx);

		aw22xxx_led_task0_cfg(aw22xxx, 0x82);
		aw22xxx_mcu_reset(aw22xxx, false);
	}
}

static void aw22xxx_set_brightness(struct led_classdev *cdev, enum led_brightness brightness)
{
	struct aw22xxx *aw22xxx = container_of(cdev, struct aw22xxx, cdev);

	aw22xxx->cdev.brightness = brightness;

	schedule_work(&aw22xxx->brightness_work);
}

static void aw22xxx_task_work(struct work_struct *work)
{
	struct aw22xxx *aw22xxx = container_of(work, struct aw22xxx, task_work);

	aw22xxx_led_off(aw22xxx);
	aw22xxx_chip_enable(aw22xxx, false);
	if (aw22xxx->task0) {
		aw22xxx_chip_enable(aw22xxx, true);
		aw22xxx_mcu_enable(aw22xxx, true);

		aw22xxx_imax_cfg(aw22xxx, (unsigned char)aw22xxx->imax);
		aw22xxx_led_display(aw22xxx);

		aw22xxx_led_task0_cfg(aw22xxx, aw22xxx->task0);
		aw22xxx_led_task1_cfg(aw22xxx, aw22xxx->task1);
		aw22xxx_mcu_reset(aw22xxx, false);
	}
}

static int aw22xxx_led_init(struct aw22xxx *aw22xxx)
{
	aw22xxx_sw_reset(aw22xxx);
	aw22xxx_chip_enable(aw22xxx, true);
	aw22xxx_imax_cfg(aw22xxx, aw22xxx_imax_code[aw22xxx->imax]);
	aw22xxx_chip_enable(aw22xxx, false);
#ifdef OPLUS_FEATURE_MISC_BASIC
	aw22xxx_i2c_write(aw22xxx, REG_PAGE, 0x00);
	aw22xxx_i2c_write(aw22xxx, REG_GCR, 0x01);
	aw22xxx_i2c_write(aw22xxx, REG_AUDCTR, 0x00);
	aw22xxx_i2c_write(aw22xxx, REG_IMAX, 0x0c);// 20ma
	aw22xxx_i2c_write(aw22xxx, REG_TASK0, 0x01);
	aw22xxx_i2c_write(aw22xxx, REG_MCUCTR, 0x01);
	aw22xxx_i2c_write(aw22xxx, REG_INTEN, 0x11);
	aw22xxx_i2c_write(aw22xxx, REG_MCUCTR, 0x03);
	aw22xxx_i2c_write(aw22xxx, REG_TASK0, 0x41);
#endif
	return 0;
}
#ifdef OPLUS_FEATURE_MISC_BASIC
static struct led_fb_conf g_led_fb_conf[] = {
	{LED_WDT_ERR_ID,       "led_wdt_err",       SENSOR_DEVICE_TYPE},
	{LED_RESET_ERR_ID,     "led_reset_err",     SENSOR_DEVICE_TYPE},
	{LED_FUNCPME_ERR_ID,   "led_funcpme_err",   SENSOR_DEVICE_TYPE},
	{LED_I2C_ERR_ID,       "led_i2c_err",       SENSOR_DEVICE_TYPE},
	{LED_FW_ERR_ID,        "led_fw_err",        SENSOR_DEVICE_TYPE},
	{LED_GCR_CHECK_ERR_ID, "led_gcr_check_err", SENSOR_DEVICE_TYPE},
};

static bool oplus_led_fault_rate_limit(struct aw22xxx *aw22xxx, uint16_t event_id)
{
	unsigned long current_boot_time = (unsigned long)ktime_get_boottime_seconds();

	if (event_id >= MAX_LED_FAULT_EVENT_ID) {
		pr_err("%s: invalid event_id %d\n", __func__, event_id);
		return false;
	}

	if (aw22xxx->fault_report_period_start[event_id] == 0 ||
	    (current_boot_time - aw22xxx->fault_report_period_start[event_id]) >= FAULT_REPORT_PERIOD_SECONDS) {
		aw22xxx->fault_report_period_start[event_id] = current_boot_time;
		aw22xxx->fault_report_count[event_id] = 0;
		pr_info("%s: event_id %d status reset\n", __func__, event_id);
	}

	if (aw22xxx->fault_report_count[event_id] >= FAULT_REPORT_COUNT_LIMIT) {
		pr_info("%s: event_id %d reached limit (5 times per 24h), ignored\n", __func__, event_id);
		return false;
	}

	aw22xxx->fault_report_count[event_id]++;
	pr_info("%s: event_id %d count: %d\n", __func__, event_id, aw22xxx->fault_report_count[event_id]);
	return true;
}
static void oplus_led_fault_report_work(struct work_struct *work)
{
	struct aw22xxx *aw22xxx = container_of(work, struct aw22xxx, fault_report_work);
	int i;
	int index = -1;
	char reg_info[64] = {0};
	int len = sizeof(g_led_fb_conf) / sizeof(g_led_fb_conf[0]);
	uint16_t event_id = aw22xxx->fault_event_id;

	for (i = 0; i < len; i++) {
		if (g_led_fb_conf[i].event_id == event_id) {
			index = i;
			break;
		}
	}
	if (index == -1) {
		pr_err("%s: unknown event_id %d\n", __func__, event_id);
		return;
	}

	snprintf(reg_info, sizeof(reg_info), "02:%02x 04:%02x 07:%02x 0A:%02x 0B:%02x",
			 aw22xxx->reg_gcr, aw22xxx->reg_mcuctr, aw22xxx->reg_pst,
			 aw22xxx->reg_intst, aw22xxx->reg_imax);

	trace_led_fault((long)ktime_get_real_seconds(),     /* time_s: Timestamp of the exception */
					LED_FAULT_APP_ID,                   /* app_id:e.g., 20120*/
					LED_FAULT_LOG_TAG,                  /* log_tag:e.g., PSW_BSP_SENSOR*/
					g_led_fb_conf[index].fb_event_id,   /* event_id: SENSOR_DEVICE_TYPE (e.g., "10002") */
					g_led_fb_conf[index].fb_field,      /* log_type: Specific error description (e.g., "led_wdt_err") */
					aw22xxx->abnormal_reset_check_count,/* reg_reset_cnt: Register reset count */
					aw22xxx->funcpme_irq_count,         /* funcpme_irq_count: Pattern end interrupt count */
					aw22xxx->wtd_irq_count,             /* wtd_irq_count: Watchdog interrupt count */
					aw22xxx->fw_version,                /* fw_ver: Firmware version */
					aw22xxx->group_info_list[aw22xxx->group_id].effect_id, /* effect_id: Effect ID when exception occurred */
					aw22xxx->group_id,                  /* group_id: Group ID when exception occurred */
					reg_info);                          /* reg_info: Register status string */

	pr_info("================================================");
	pr_info("LED_FAULT_APP_ID: %d\n", LED_FAULT_APP_ID);
	pr_info("LED_FAULT_LOG_TAG: %s\n", LED_FAULT_LOG_TAG);
	pr_info("event_id: %s\n", g_led_fb_conf[index].fb_event_id);
	pr_info("log_type: %s\n", g_led_fb_conf[index].fb_field);
	pr_info("reg_reset_cnt: %d\n", aw22xxx->abnormal_reset_check_count);
	pr_info("funcpme_irq_count: %d\n", aw22xxx->funcpme_irq_count);
	pr_info("wtd_irq_count: %d\n", aw22xxx->wtd_irq_count);
	pr_info("fw_ver: 0x%x\n", aw22xxx->fw_version);
	pr_info("effect_id: %d\n", aw22xxx->group_info_list[aw22xxx->group_id].effect_id);
	pr_info("group_id: %d\n", aw22xxx->group_id);
	pr_info("reg_info: %s\n", reg_info);
	pr_info("================================================");
}
static void oplus_handle_led_fault(struct aw22xxx *aw22xxx, int event_id, bool force)
{
	if (!aw22xxx->trace_fault_support) {
		pr_info("%s: trace-fault-support is not supported, skip reporting\n", __func__);
		return;
	}
	if (!force) {
		if (!oplus_led_fault_rate_limit(aw22xxx, event_id)) {
			return;
		}
	}

	aw22xxx->fault_event_id = event_id;
	schedule_work(&aw22xxx->fault_report_work);
}
static char aw22xxx_cfg_name_0[][AW22XXX_CFG_NAME_MAX] = {
	{"aw22xxx_cfg_group0_led_off.bin"},
	{"aw22xxx_cfg_group0_led_on.bin"},
	{"aw22xxx_cfg_group0_led_breath.bin"},
	{"aw22xxx_cfg_group0_led_blink.bin"},
	{"aw22xxx_cfg_group0_led_flow.bin"},
	{"aw22xxx_cfg_group0_led_run.bin"},
	{"aw22xxx_cfg_group0_led_music_play.bin"},
};

static char aw22xxx_cfg_name_1[][AW22XXX_CFG_NAME_MAX] = {
	{"aw22xxx_cfg_group1_led_off.bin"},
	{"aw22xxx_cfg_group1_led_on.bin"},
	{"aw22xxx_cfg_group1_led_breath.bin"},
	{"aw22xxx_cfg_group1_led_blink.bin"},
	{"aw22xxx_cfg_group1_led_flow.bin"},
	{"aw22xxx_cfg_group1_led_charge.bin"},
};

static char aw22xxx_cfg_name_2[][AW22XXX_CFG_NAME_MAX] = {
	{"aw22xxx_cfg_group2_led_off.bin"},
	{"aw22xxx_cfg_group2_led_on.bin"},
	{"aw22xxx_cfg_group2_led_breath.bin"},
	{"aw22xxx_cfg_group2_led_blink.bin"},
};

static struct color_offset_num aw22xxx_color_offsets_0[] = {
	{0, 0},				   // all led off
	{482, 554},			   // all constant on
	{986, 1058},		   // all breath
	{986, 1058},		   // all blink
	{134, 170},			   // all flow,take the first three colors
	{0, 0},			   	   // all run
	{482, 554},			   // all music play
};

static struct color_offset_num aw22xxx_color_offsets_1[] = {
	{0, 0},				   // ambient led off
	{344, 392},			   // ambient constant on
	{680, 728},			   // ambient breath
	{680, 728},			   // ambient blink
	{132, 168},			   // ambient flow,take the first three colors
	{0, 0},			   		// ambient charge
};

static struct color_offset_num aw22xxx_color_offsets_2[] = {
	{0, 0},				   // fan led off
	{208, 232},			   // fan constant on
	{376, 400},			   // fan breath
	{376, 400},			   // fan blink
};

static const int aw22xxx_cfg_name_sizes[] = {
	sizeof(aw22xxx_cfg_name_0) / sizeof(aw22xxx_cfg_name_0[0]),
	sizeof(aw22xxx_cfg_name_1) / sizeof(aw22xxx_cfg_name_1[0]),
	sizeof(aw22xxx_cfg_name_2) / sizeof(aw22xxx_cfg_name_2[0]),
};

static char *aw22xxx_cfg_name_ptr[] = {
	(char *)aw22xxx_cfg_name_0,
	(char *)aw22xxx_cfg_name_1,
	(char *)aw22xxx_cfg_name_2,
};

static struct color_offset_num *aw22xxx_color_offsets_ptr[] = {
	aw22xxx_color_offsets_0,
	aw22xxx_color_offsets_1,
	aw22xxx_color_offsets_2,
};

static bool oplus_validate_group_id(struct aw22xxx *aw22xxx, unsigned int group_id)
{
	if (!aw22xxx || !(aw22xxx->group_info_list)) {
		pr_err("%s: invalid ptr\n", __func__);
		return false;
	}
	for (int i = 0; i < aw22xxx->groups_num; i++) {
		if (aw22xxx->group_info_list[i].id == group_id) {
			return true;
		}
	}
	return false;
}
static bool oplus_validate_effect_id(struct aw22xxx *aw22xxx, unsigned int group_id, unsigned int effect_id)
{
	return (effect_id >= aw22xxx->group_info_list[group_id].effect_num_max)?false:true;
}

static int oplus_get_cfg_name(struct aw22xxx *aw22xxx, unsigned int group_id, unsigned int effect_id, char *cfg_name_buf, size_t size)
{
	struct aw22xxx_group_info *group = &aw22xxx->group_info_list[group_id];
	char *base_name = group->cfg_name_array[effect_id];
	char *insert_pos;
	int target_color_code;
	int prefix_len;
	int ret = 0;

	if (!aw22xxx->backcolor_support) {
		ret = strscpy(cfg_name_buf, base_name, size);
		if (ret < 0) {
			pr_err("%s: strscpy failed\n", __func__);
			return ret;
		}
		return 0;
	}

	target_color_code = aw22xxx->back_color;
	insert_pos = strstr(base_name, "_led_");
	if (insert_pos) {
		prefix_len = insert_pos - base_name;
		snprintf(cfg_name_buf, size, "%.*s_bkcolor%d%s",
				 prefix_len, base_name, target_color_code, insert_pos);
	} else {
		pr_err("%s: filename format mismatch, cannot insert bkcolor\n", __func__);
		ret = strscpy(cfg_name_buf, base_name, size);
		if (ret < 0) {
			pr_err("%s: strscpy failed\n", __func__);
			return ret;
		}
	}
	return 0;
}

static int oplus_get_color_offset(struct aw22xxx *aw22xxx, unsigned int group_id, unsigned int effect_id, int* color_start, int* color_end)
{
	struct aw22xxx_group_info *group = &aw22xxx->group_info_list[group_id];
	struct color_offset_num *offset = &group->color_offsets_array[effect_id];

	*color_start = offset->color_start;
	*color_end = offset->color_end;
	return 0;
}

static void oplus_change_effect_rgb(struct aw22xxx *aw22xxx, const struct firmware *cont, int i)
{
	unsigned int index = 0;
	int color_start;
	int color_end;
	struct aw22xxx_group_info *group = &aw22xxx->group_info_list[aw22xxx->group_id];

	oplus_get_color_offset(aw22xxx, group->id, group->effect_id, &color_start, &color_end);
	pr_debug("%s: color_start%d, color_end%d\n", __func__, color_start, color_end);

	if (i >= color_start && (i - color_start) % 12 == 0 && i < color_end) {
		index = (i - color_start) / 12;
		aw22xxx_i2c_write(aw22xxx, *(cont->data + i), (group->rgb_value[index]>>16)&0xff);
	} else if (i >= color_start && (i - color_start) % 12 == 4 && i < color_end) {
		index = (i - color_start) / 12;
		aw22xxx_i2c_write(aw22xxx, *(cont->data + i), (group->rgb_value[index]>>8)&0xff);
	} else if (i >= color_start && (i - color_start) % 12 == 8 && i < color_end){
		index = (i - color_start) / 12;
		aw22xxx_i2c_write(aw22xxx, *(cont->data + i), (group->rgb_value[index]>>0)&0xff);
	} else {
		aw22xxx_i2c_write(aw22xxx, *(cont->data + i), *(cont->data + i + 1));
	}
}

static int oplus_get_group_enabled_num(struct aw22xxx *aw22xxx)
{
	int enabled_count = 0;

	for (int i = 0; i < aw22xxx->groups_num; i++) {
		if (aw22xxx->group_info_list[i].led_enabled) {
			enabled_count++;
		}
	}
	pr_info("%s: enabled_count:%d\n", __func__, enabled_count);
	return enabled_count;
}

static void abnormal_reset_check_work_func(struct work_struct *work)
{
	struct aw22xxx *aw22xxx = container_of(work, struct aw22xxx, abnormal_reset_check_work.work);
	int ret;

	if (!aw22xxx->abnormal_reset_check_enabled) {
		pr_info("%s: check disabled by user, stop check\n", __func__);
		return;
	}
	if (aw22xxx->led_standby) {
		pr_info("%s: led standby, stop check\n", __func__);
		return;
	}

	ret = aw22xxx_i2c_read(aw22xxx, REG_GCR, &aw22xxx->reg_gcr);
	if (ret < 0) {
		pr_err("%s: read REG_GCR failed\n", __func__);
		schedule_delayed_work(&aw22xxx->abnormal_reset_check_work, msecs_to_jiffies(aw22xxx->check_period));
		return;
	}

	if (aw22xxx->reg_gcr != 0x01 || force_check_error) {
		if (!force_check_error) {
			pr_err("%s: Check Error! REG_GCR: 0x%x\n", __func__, aw22xxx->reg_gcr);
		} else {
			pr_warn("%s: [DEBUG] force_check_error:%d Force triggering check error!\n", __func__, force_check_error);
			force_check_error = false;
		}

		aw22xxx->abnormal_reset_check_count++;

		aw22xxx_i2c_read(aw22xxx, REG_MCUCTR, &aw22xxx->reg_mcuctr);
		aw22xxx_i2c_read(aw22xxx, REG_PST, &aw22xxx->reg_pst);
		aw22xxx_i2c_read(aw22xxx, REG_INTST, &aw22xxx->reg_intst);
		aw22xxx_i2c_read(aw22xxx, REG_IMAX, &aw22xxx->reg_imax);
		oplus_handle_led_fault(aw22xxx, LED_GCR_CHECK_ERR_ID, false);

		pr_info("%s: attempting to recover LED state\n", __func__);
		aw22xxx_led_init(aw22xxx);
		schedule_delayed_work(&aw22xxx->recovery_work, msecs_to_jiffies(20));
	}

	schedule_delayed_work(&aw22xxx->abnormal_reset_check_work, msecs_to_jiffies(aw22xxx->check_period));
	pr_info("%s: check end, abnormal_reset_check_count:%d\n", __func__, aw22xxx->abnormal_reset_check_count);
}
static void oplus_led_standby(struct aw22xxx *aw22xxx)
{
	cancel_delayed_work_sync(&aw22xxx->abnormal_reset_check_work);
	aw22xxx_i2c_write(aw22xxx, REG_PAGE, 0x00);
	aw22xxx_i2c_write(aw22xxx, REG_GCR, 0x00);
	aw22xxx->led_standby = true;
	for (int i = 0; i < aw22xxx->groups_num; i++) {
		aw22xxx->group_info_list[i].led_enabled = false;
	}
	pr_info("%s: led standby,cancel check work\n", __func__);
}
static void oplus_led_on(struct aw22xxx *aw22xxx)
{
	aw22xxx_led_init(aw22xxx);
	aw22xxx->led_standby = false;
	schedule_delayed_work(&aw22xxx->abnormal_reset_check_work, msecs_to_jiffies(FIRST_CHECK_DELAY_MS));
	pr_info("%s: led wakeup,schedule check work\n", __func__);
}

static void oplus_led_standby_func(struct work_struct *work)
{
	struct aw22xxx *aw22xxx = container_of(work, struct aw22xxx, led_standby_work.work);
	int count = 0;

	count = oplus_get_group_enabled_num(aw22xxx);
	if (count && aw22xxx->led_standby) {
		oplus_led_on(aw22xxx);
	} else if (!count && !aw22xxx->led_standby) {
		oplus_led_standby(aw22xxx);
	} else {
		pr_info("%s: led mode unchanged\n", __func__);
	}
}
/**
 * Check if two LED groups are mutually exclusive by comparing their LED indices
 *
 * Principle: Two groups are exclusive if they share at least one common LED index
 * This prevents hardware conflicts when multiple groups try to control the same LED
 */
static bool oplus_calculate_groups_exclusion(struct aw22xxx *aw22xxx, int group_id1, int group_id2)
{
	struct aw22xxx_group_info *group1 = &aw22xxx->group_info_list[group_id1];
	struct aw22xxx_group_info *group2 = &aw22xxx->group_info_list[group_id2];

	if (group_id1 == group_id2) {
		return false;
	}
	for (int i = 0; i < group1->include_led_num; i++) {
		for (int j = 0; j < group2->include_led_num; j++) {
			if (group1->include_led_index[i] == group2->include_led_index[j]) {
				pr_info("%s: group %d and group %d are exclusive\n", __func__, group_id1, group_id2);
				return true;
			}
		}
	}
	return false;
}
/**
 * Initialize group exclusion cache by pre-calculating all group relationships
 *
 * Principle: Pre-compute exclusion matrix during initialization for fast runtime lookups
 * The cache is symmetric to ensure consistency in bidirectional checks
 */
static void oplus_init_exclusive_status(struct aw22xxx *aw22xxx)
{
	pr_info("%s: group exclusion init start\n", __func__);
	for (int i = 0; i < aw22xxx->groups_num; i++) {
		for (int j = i + 1; j < aw22xxx->groups_num; j++) {
			bool exclusion = oplus_calculate_groups_exclusion(aw22xxx, i, j);
			aw22xxx->exclusion_cache[i][j] = exclusion;
			aw22xxx->exclusion_cache[j][i] = exclusion;
		}
	}
	pr_info("%s: group exclusion init done\n", __func__);
}
/**
 * Fast check for group exclusion using pre-computed cache
 *
 * Principle: Use cached results from initialization for O(1) performance
 * Avoids expensive LED index comparisons during runtime operations
 */
static bool oplus_check_group_exclusion(struct aw22xxx *aw22xxx, int group_id1, int group_id2)
{
	return aw22xxx->exclusion_cache[group_id1][group_id2];
}
/**
 * Disable conflicting groups when a group is activated
 *
 * Principle: When a group becomes active, disable all other groups that share LEDs with it
 * This ensures only one group controls any given LED at any time
 */
static void oplus_update_enabled_flag(struct aw22xxx *aw22xxx, int current_group_id)
{
	for (int i = 0; i < aw22xxx->groups_num; i++) {
		if (i != current_group_id && oplus_check_group_exclusion(aw22xxx, current_group_id, i)) {
			struct aw22xxx_group_info *target_group = &aw22xxx->group_info_list[i];
			if (target_group->led_enabled) {
				pr_info("%s: disable group%d due to group%d operation\n", __func__, i, current_group_id);
				target_group->led_enabled = false;
			}
		}
	}
}
static struct color_offset_num aw22xxx_charge_color_offsets[] = {
	{132, 168}, /* flow */
	{524, 572}, /* on */
};

static void oplus_change_effect_rgb_for_charge_flow(struct aw22xxx *aw22xxx, const struct firmware *cont, int i)
{
	unsigned int index = 0;
	struct aw22xxx_group_info *group = &aw22xxx->group_info_list[aw22xxx->group_id];
	int start = aw22xxx_charge_color_offsets[0].color_start;

	index = (i - start) / 12;
	if ((i - start) % 12 == 0) {
		aw22xxx_i2c_write(aw22xxx, *(cont->data + i), (group->rgb_value[index] >> 16) & 0xff);
	} else if ((i - start) % 12 == 4) {
		aw22xxx_i2c_write(aw22xxx, *(cont->data + i), (group->rgb_value[index] >> 8) & 0xff);
	} else if ((i - start) % 12 == 8) {
		aw22xxx_i2c_write(aw22xxx, *(cont->data + i), (group->rgb_value[index] >> 0) & 0xff);
	} else {
		aw22xxx_i2c_write(aw22xxx, *(cont->data + i), *(cont->data + i + 1));
	}
}

static void oplus_change_effect_rgb_for_charge_on(struct aw22xxx *aw22xxx, const struct firmware *cont, int i)
{
	unsigned int index = 0;
	struct aw22xxx_group_info *group = &aw22xxx->group_info_list[aw22xxx->group_id];
	int start = aw22xxx_charge_color_offsets[1].color_start;

	index = (i - start) / 12;
	if ((i - start) % 12 == 0) {
		aw22xxx_i2c_write(aw22xxx, *(cont->data + i), (group->rgb_value[index] >> 16) & 0xff);
	} else if ((i - start) % 12 == 4) {
		aw22xxx_i2c_write(aw22xxx, *(cont->data + i), (group->rgb_value[index] >> 8) & 0xff);
	} else if ((i - start) % 12 == 8) {
		aw22xxx_i2c_write(aw22xxx, *(cont->data + i), (group->rgb_value[index] >> 0) & 0xff);
	} else {
		aw22xxx_i2c_write(aw22xxx, *(cont->data + i), *(cont->data + i + 1));
	}
}

static void oplus_change_effect_rgb_for_charge(struct aw22xxx *aw22xxx, const struct firmware *cont, int i)
{
	if (i >= aw22xxx_charge_color_offsets[0].color_start && i < aw22xxx_charge_color_offsets[0].color_end) {
		oplus_change_effect_rgb_for_charge_flow(aw22xxx, cont, i);
	} else if (i >= aw22xxx_charge_color_offsets[1].color_start && i < aw22xxx_charge_color_offsets[1].color_end) {
		oplus_change_effect_rgb_for_charge_on(aw22xxx, cont, i);
	} else {
		aw22xxx_i2c_write(aw22xxx, *(cont->data + i), *(cont->data + i + 1));
	}
}
#endif
/*****************************************************
 *
 * firmware/cfg update
 *
 *****************************************************/
static void aw22xxx_cfg_loaded(const struct firmware *cont, void *context)
{
	struct aw22xxx *aw22xxx = context;
	int i = 0;
	unsigned char page = 0;
	unsigned char reg_addr = 0;
	unsigned char reg_val = 0;

#ifdef OPLUS_FEATURE_MISC_BASIC
	char cfg_name[AW22XXX_CFG_NAME_MAX];
	struct aw22xxx_group_info *group = &aw22xxx->group_info_list[aw22xxx->group_id];

	oplus_get_cfg_name(aw22xxx, group->id, group->effect_id, cfg_name, sizeof(cfg_name));
	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__, cfg_name);
		mutex_unlock(&aw22xxx->cfg_lock);
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__, cfg_name, cont ? cont->size : 0);
	cancel_delayed_work_sync(&aw22xxx->led_standby_work);
	oplus_update_enabled_flag(aw22xxx, group->id);
	if (group->effect_id != 0 && !group->led_enabled) {
		group->led_enabled = true;
		pr_info("%s: execute work now\n", __func__);
		oplus_led_standby_func(&aw22xxx->led_standby_work.work);
	} else if (group->effect_id == 0) {
		group->led_enabled = false;
		pr_info("%s: schedule work after 2s\n", __func__);
		schedule_delayed_work(&aw22xxx->led_standby_work, msecs_to_jiffies(2000));
	}
	for (int i = 0; i < aw22xxx->groups_num; i++) {
		pr_info("%s: group[%d]: led_enabled:%d\n", __func__, i, aw22xxx->group_info_list[i].led_enabled);
	}
#else
	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__, aw22xxx_cfg_name[aw22xxx->effect]);
		release_firmware(cont);
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__, aw22xxx_cfg_name[aw22xxx->effect],
					cont ? cont->size : 0);
#endif

	for (i = 0; i < cont->size; i += 2) {
		if (*(cont->data+i) == 0xff)
			page = *(cont->data+i+1);
		if (aw22xxx->cfg == 1) {
			aw22xxx_i2c_write(aw22xxx, *(cont->data+i), *(cont->data+i+1));
			pr_debug("%s: addr:0x%02x, data:0x%02x\n", __func__, *(cont->data+i), *(cont->data+i+1));
		}
#ifdef OPLUS_FEATURE_MISC_BASIC
		else if (aw22xxx->cfg == 3){
			if (aw22xxx->group_id == 1 && group->include_led_num == 4 && group->effect_id == EFFECT_ID_CHARGE) {
				oplus_change_effect_rgb_for_charge(aw22xxx, cont, i);
			} else {
				oplus_change_effect_rgb(aw22xxx, cont, i);
			}
		}
		if(((*(cont->data+i) == 0x05)&&(*(cont->data+i+1) == 0x01)) ||
			((*(cont->data+i) == 0x05)&&(*(cont->data+i+1) == 0x41)) ||
			((*(cont->data+i) == 0x05)&&(*(cont->data+i+1) == 0x51))) {
			usleep_range(5000, 10000);
			pr_info("%s: addr:0x%02x, data:0x%02x\n", __func__, *(cont->data+i), *(cont->data+i+1));
		}
#endif
		if (page == AW22XXX_REG_PAGE0) {
			reg_addr = *(cont->data+i);
			reg_val = *(cont->data+i+1);
			/* gcr chip enable delay */
			if ((reg_addr == REG_GCR) &&
					((reg_val&BIT_GCR_CHIPEN_ENABLE) == BIT_GCR_CHIPEN_ENABLE)) {
				usleep_range(2000, 3000);
			}
		}
	}

	release_firmware(cont);
	usleep_range(5000, 10000);
	pr_info("%s: cfg update complete\n", __func__);
	mutex_unlock(&aw22xxx->cfg_lock);
}

static int aw22xxx_cfg_update(struct aw22xxx *aw22xxx)
{
#ifdef OPLUS_FEATURE_MISC_BASIC
	char cfg_name[AW22XXX_CFG_NAME_MAX];
	struct aw22xxx_group_info *group = &aw22xxx->group_info_list[aw22xxx->group_id];
	int retry_count = 0;
	const int max_retry = 20;

	mutex_lock(&aw22xxx->cfg_lock);
	pr_info("aw22xxx_cfg_update\n");
	if (aw22xxx->fw_flags != AW22XXX_FLAG_FW_OK) {
		pr_info("%s: firmware not ready status=%d,init firmware now\n", __func__, aw22xxx->fw_flags);
		aw22xxx_fw_update(aw22xxx);
		while (retry_count < max_retry && aw22xxx->fw_flags != AW22XXX_FLAG_FW_OK) {
			msleep(100);
			retry_count++;
			if (aw22xxx->fw_flags == AW22XXX_FLAG_FW_FAIL) {
				pr_info("%s: firmware status=%d, triggering update\n", __func__, aw22xxx->fw_flags);
				aw22xxx_fw_update(aw22xxx);
			}
		}

		if (retry_count >= max_retry) {
			pr_err("%s: firmware initialization timeout\n", __func__);
			mutex_unlock(&aw22xxx->cfg_lock);
			return -ETIMEDOUT;
		}
		pr_info("%s: firmware ready after %d retries\n", __func__, retry_count);
		usleep_range(5000, 10000);
	}

	oplus_get_cfg_name(aw22xxx, group->id, group->effect_id, cfg_name, sizeof(cfg_name));
	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				cfg_name, aw22xxx->dev, GFP_KERNEL,
				aw22xxx, aw22xxx_cfg_loaded);
#else
	pr_info("aw22xxx_cfg_update\n");
	if (aw22xxx->effect < (sizeof(aw22xxx_cfg_name)/AW22XXX_CFG_NAME_MAX)) {
		pr_info("%s: cfg name=%s\n", __func__, aw22xxx_cfg_name[aw22xxx->effect]);
	} else {
		pr_err("%s: effect 0x%02x over max value\n", __func__, aw22xxx->effect);
		aw22xxx->effect = (sizeof(aw22xxx_cfg_name)/AW22XXX_CFG_NAME_MAX) - 1;
	}

	if (aw22xxx->fw_flags != AW22XXX_FLAG_FW_OK)
		return -2;

	mutex_lock(&aw22xxx->cfg_lock);

	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				aw22xxx_cfg_name[aw22xxx->effect], aw22xxx->dev, GFP_KERNEL,
				aw22xxx, aw22xxx_cfg_loaded);
#endif
}

static int aw22xxx_container_update(struct aw22xxx *aw22xxx, struct aw22xxx_container *aw22xxx_fw)
{
	unsigned int i;
	unsigned char reg_val;
	unsigned int tmp_bist;
#ifdef AW22XXX_FLASH_I2C_WRITES
	unsigned int tmp_len;
#endif

	/* chip enable*/
	aw22xxx_reg_page_cfg(aw22xxx, AW22XXX_REG_PAGE0);
	aw22xxx_sw_reset(aw22xxx);
	aw22xxx_chip_enable(aw22xxx, true);
	aw22xxx_mcu_enable(aw22xxx, true);

	/* flash cfg */
	aw22xxx_i2c_write(aw22xxx, 0x80, 0xec);
	aw22xxx_i2c_write(aw22xxx, 0x35, 0x29);
	/*aw22xxx_i2c_write(aw22xxx, 0x37, 0xba);*/
	aw22xxx_i2c_write(aw22xxx, 0x38, aw22xxx_fw->key);

	/* flash erase*/
	aw22xxx_i2c_write(aw22xxx, 0x22, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x21, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x03);
	aw22xxx_i2c_write(aw22xxx, 0x30, 0x03);
	aw22xxx_i2c_write(aw22xxx, 0x23, 0x00);
	usleep_range(4000, 4500);
	aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x22, 0x40);
	aw22xxx_i2c_write(aw22xxx, 0x21, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x30, 0x02);
	aw22xxx_i2c_write(aw22xxx, 0x23, 0x00);
	usleep_range(6000, 6500);
	aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x22, 0x42);
	aw22xxx_i2c_write(aw22xxx, 0x21, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x30, 0x02);
	aw22xxx_i2c_write(aw22xxx, 0x23, 0x00);
	usleep_range(6000, 6500);
	aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x22, 0x44);
	aw22xxx_i2c_write(aw22xxx, 0x21, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x30, 0x02);
	aw22xxx_i2c_write(aw22xxx, 0x23, 0x00);
	usleep_range(6000, 6500);
	aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x00);

	#ifdef AW22XXX_FLASH_I2C_WRITES
	/* flash writes */
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x03);
	for (i = 0; i < aw22xxx_fw->len; i += tmp_len) {
		aw22xxx_i2c_write(aw22xxx, 0x22, ((i>>8)&0xff));
		aw22xxx_i2c_write(aw22xxx, 0x21, ((i>>0)&0xff));
		aw22xxx_i2c_write(aw22xxx, 0x11, 0x01);
		aw22xxx_i2c_write(aw22xxx, 0x30, 0x04);
		if ((aw22xxx_fw->len - i) < MAX_FLASH_WRITE_BYTE_SIZE)
			tmp_len = aw22xxx_fw->len - i;
		else
			tmp_len = MAX_FLASH_WRITE_BYTE_SIZE;

		aw22xxx_i2c_writes(aw22xxx, 0x23, &aw22xxx_fw->data[i], tmp_len);
		aw22xxx_i2c_write(aw22xxx, 0x11, 0x00);
		aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
	}
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x00);
	#else
	/* flash write */
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x03);
	for (i = 0; i < aw22xxx_fw->len; i++) {
		aw22xxx_i2c_write(aw22xxx, 0x22, ((i>>8)&0xff));
		aw22xxx_i2c_write(aw22xxx, 0x21, ((i>>0)&0xff));
		aw22xxx_i2c_write(aw22xxx, 0x30, 0x04);
		aw22xxx_i2c_write(aw22xxx, 0x23, aw22xxx_fw->data[i]);
		aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
	}
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x00);
	#endif

	/* bist check */
	aw22xxx_sw_reset(aw22xxx);
	aw22xxx_chip_enable(aw22xxx, true);
	aw22xxx_mcu_enable(aw22xxx, true);
	aw22xxx_i2c_write(aw22xxx, 0x22, (((aw22xxx_fw->len-1)>>8)&0xff));
	aw22xxx_i2c_write(aw22xxx, 0x21, (((aw22xxx_fw->len-1)>>0)&0xff));
	aw22xxx_i2c_write(aw22xxx, 0x24, 0x07);
	usleep_range(5000, 6500);
	aw22xxx_i2c_read(aw22xxx, 0x24, &reg_val);
	if (reg_val == 0x05) {
		aw22xxx_i2c_read(aw22xxx, 0x25, &reg_val);
		tmp_bist = reg_val;
		aw22xxx_i2c_read(aw22xxx, 0x26, &reg_val);
		tmp_bist |= (reg_val<<8);
		if (tmp_bist == aw22xxx_fw->bist) {
			pr_info("%s: bist check pass, bist=0x%04x\n", __func__, aw22xxx_fw->bist);
		} else {
			pr_err("%s: bist check fail, bist=0x%04x\n", __func__, aw22xxx_fw->bist);
			pr_err("%s: fw update failed, please reset phone\n", __func__);
			return -1;
		}
	} else {
		pr_err("%s: bist check is running, reg0x24=0x%02x\n", __func__, reg_val);
	}
	aw22xxx_i2c_write(aw22xxx, 0x24, 0x00);

	return 0;
}

static void aw22xxx_fw_loaded(const struct firmware *cont, void *context)
{
	struct aw22xxx *aw22xxx = context;
	struct aw22xxx_container *aw22xxx_fw;
	int i = 0;
	int ret = -1;
	char tmp_buf[32] = {0};
	unsigned int shift = 0;
	unsigned short check_sum = 0;
	unsigned char reg_val = 0;
	unsigned int tmp_bist = 0;

	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__, aw22xxx_fw_name);
		release_firmware(cont);
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__, aw22xxx_fw_name,
					cont ? cont->size : 0);

	/* check sum */
	for (i = 2; i < cont->size; i++)
		check_sum += cont->data[i];

	if (check_sum != (unsigned short)((cont->data[0]<<8)|(cont->data[1]))) {
		pr_err("%s: check sum err: check_sum=0x%04x\n", __func__, check_sum);
		release_firmware(cont);
		return;
	}

	/* get fw info */
	aw22xxx_fw = kzalloc(cont->size + 4*sizeof(unsigned int), GFP_KERNEL);
	if (!aw22xxx_fw) {
		release_firmware(cont);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
	shift += 2;

	pr_info("%s: fw chip_id : 0x%02x\n", __func__, cont->data[0+shift]);
	shift += 1;

	memcpy(tmp_buf, &cont->data[0+shift], 16);
	pr_info("%s: fw customer: %s\n", __func__, tmp_buf);
	shift += 16;

	memcpy(tmp_buf, &cont->data[0+shift], 8);
	pr_info("%s: fw project: %s\n", __func__, tmp_buf);
	shift += 8;

	aw22xxx_fw->version = (cont->data[0+shift]<<24) | (cont->data[1+shift]<<16) |
			(cont->data[2+shift]<<8) | (cont->data[3+shift]<<0);
	pr_info("%s: fw version : 0x%04x\n", __func__, aw22xxx_fw->version);
	aw22xxx->fw_version = aw22xxx_fw->version;
	pr_info("%s: aw22xxx fw version set to %d\n", __func__, aw22xxx->fw_version);
	shift += 4;

	// reserved
	shift += 3;

	aw22xxx_fw->bist = (cont->data[0+shift]<<8) | (cont->data[1+shift]<<0);
	pr_info("%s: fw bist : 0x%04x\n", __func__, aw22xxx_fw->bist);
	shift += 2;

	aw22xxx_fw->key = cont->data[0+shift];
	pr_info("%s: fw key : 0x%04x\n", __func__, aw22xxx_fw->key);
	shift += 1;

	// reserved
	shift += 1;

	aw22xxx_fw->len = (cont->data[0+shift]<<8) | (cont->data[1+shift]<<0);
	pr_info("%s: fw len : 0x%04x\n", __func__, aw22xxx_fw->len);
	shift += 2;

	memcpy(aw22xxx_fw->data, &cont->data[shift], aw22xxx_fw->len);
	release_firmware(cont);

	/* check version */
	/*aw22xxx_get_fw_version(aw22xxx); */

	/* bist check */
	aw22xxx_sw_reset(aw22xxx);
	aw22xxx_chip_enable(aw22xxx, true);
	aw22xxx_mcu_enable(aw22xxx, true);
	aw22xxx_i2c_write(aw22xxx, 0x22, (((aw22xxx_fw->len-1)>>8)&0xff));
	aw22xxx_i2c_write(aw22xxx, 0x21, (((aw22xxx_fw->len-1)>>0)&0xff));
	aw22xxx_i2c_write(aw22xxx, 0x24, 0x07);
	usleep_range(5000, 6500);
	aw22xxx_i2c_read(aw22xxx, 0x24, &reg_val);
	if (reg_val == 0x05) {
		aw22xxx_i2c_read(aw22xxx, 0x25, &reg_val);
		tmp_bist = reg_val;
		aw22xxx_i2c_read(aw22xxx, 0x26, &reg_val);
		tmp_bist |= (reg_val<<8);
		if (tmp_bist == aw22xxx_fw->bist) {
			pr_info("%s: bist check pass, bist=0x%04x\n", __func__, aw22xxx_fw->bist);
			if (aw22xxx->fw_update == 0) {
				kfree(aw22xxx_fw);
				aw22xxx_fw = NULL;
				aw22xxx_i2c_write(aw22xxx, 0x24, 0x00);
				aw22xxx_led_init(aw22xxx);
				aw22xxx->fw_flags = AW22XXX_FLAG_FW_OK;
			} else {
				pr_info("%s: fw version: 0x%04x, force update fw\n",
					__func__, aw22xxx_fw->version);
			}
		} else {
			pr_info("%s: bist check fail, fw bist=0x%04x, flash bist=0x%04x\n",
					__func__, aw22xxx_fw->bist, tmp_bist);
			pr_info("%s: find new fw: 0x%04x, need update\n",
					__func__, aw22xxx_fw->version);
		}
	} else {
		pr_err("%s: bist check is running, reg0x24=0x%02x\n", __func__, reg_val);
		pr_info("%s: fw need update\n", __func__);
	}
	aw22xxx_i2c_write(aw22xxx, 0x24, 0x00);

	/* fw update */
	if (aw22xxx_fw != NULL) {
		pr_info("%s update firmware start ....\n", __func__);
		ret = aw22xxx_container_update(aw22xxx, aw22xxx_fw);
		if (ret)
			aw22xxx->fw_flags = AW22XXX_FLAG_FW_FAIL;
		else
			aw22xxx->fw_flags = AW22XXX_FLAG_FW_OK;

		kfree(aw22xxx_fw);

		aw22xxx->fw_update = 0;

		aw22xxx_led_init(aw22xxx);
	}
#ifdef OPLUS_FEATURE_MISC_BASIC
	usleep_range(5000, 10000);
	oplus_led_standby(aw22xxx);
	pr_info("%s: led standby\n", __func__);
#endif
}

static int aw22xxx_fw_update(struct aw22xxx *aw22xxx)
{
	aw22xxx->fw_flags = AW22XXX_FLAG_FW_UPDATE;

	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				aw22xxx_fw_name, aw22xxx->dev, GFP_KERNEL,
				aw22xxx, aw22xxx_fw_loaded);
}

#ifdef AWINIC_FW_UPDATE_DELAY
static enum hrtimer_restart aw22xxx_fw_timer_func(struct hrtimer *timer)
{
	struct aw22xxx *aw22xxx = container_of(timer, struct aw22xxx, fw_timer);
	if (aw22xxx->fw_flags != AW22XXX_FLAG_FW_OK){
		schedule_work(&aw22xxx->fw_work);
	}
	return HRTIMER_NORESTART;
}
#endif

static void aw22xxx_fw_work_routine(struct work_struct *work)
{
	struct aw22xxx *aw22xxx = container_of(work, struct aw22xxx, fw_work);

	aw22xxx_fw_update(aw22xxx);

}

static void aw22xxx_cfg_work_routine(struct work_struct *work)
{
	struct aw22xxx *aw22xxx = container_of(work, struct aw22xxx, cfg_work);

	aw22xxx_cfg_update(aw22xxx);

}

static int aw22xxx_fw_init(struct aw22xxx *aw22xxx)
{
#ifdef AWINIC_FW_UPDATE_DELAY
	int fw_timer_val = 10000;

	hrtimer_init(&aw22xxx->fw_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw22xxx->fw_timer.function = aw22xxx_fw_timer_func;
	INIT_WORK(&aw22xxx->fw_work, aw22xxx_fw_work_routine);
	INIT_WORK(&aw22xxx->cfg_work, aw22xxx_cfg_work_routine);
#ifdef OPLUS_FEATURE_MISC_BASIC
	INIT_DELAYED_WORK(&aw22xxx->led_standby_work, oplus_led_standby_func);
	aw22xxx->abnormal_reset_check_enabled = true;
	aw22xxx->check_period = DEFAULT_CHECK_PERIOD_MS;
	INIT_DELAYED_WORK(&aw22xxx->abnormal_reset_check_work, abnormal_reset_check_work_func);
	INIT_DELAYED_WORK(&aw22xxx->recovery_work, aw22xxx_recovery_work_routine);
	INIT_WORK(&aw22xxx->fault_report_work, oplus_led_fault_report_work);
	INIT_WORK(&aw22xxx->music_work, aw22xxx_music_work_routine);
#endif
	hrtimer_start(&aw22xxx->fw_timer,
			ktime_set(fw_timer_val/1000, (fw_timer_val%1000)*1000000),
			HRTIMER_MODE_REL);
#else
	INIT_WORK(&aw22xxx->fw_work, aw22xxx_fw_work_routine);
	INIT_WORK(&aw22xxx->cfg_work, aw22xxx_cfg_work_routine);
	schedule_work(&aw22xxx->fw_work);
#endif
	return 0;
}


/******************************************************
 *
 * irq
 *
 ******************************************************/
static void aw22xxx_interrupt_clear(struct aw22xxx *aw22xxx)
{
	unsigned char reg_val;

	aw22xxx_i2c_read(aw22xxx, REG_INTST, &reg_val);
	pr_info("%s: reg INTST=0x%x\n", __func__, reg_val);
}

static void aw22xxx_interrupt_setup(struct aw22xxx *aw22xxx)
{
	aw22xxx_interrupt_clear(aw22xxx);

	aw22xxx_i2c_write_bits(aw22xxx, REG_INTEN,
			BIT_INTEN_FUNCMPE_MASK, BIT_INTEN_FUNCMPE_ENABLE);

	// enable wtd irq
	aw22xxx_i2c_write_bits(aw22xxx, REG_INTEN,
			BIT_INTEN_WTD_MASK, BIT_INTEN_WTD_ENABLE);
}
static void aw22xxx_recovery_work_routine(struct work_struct *work)
{
	struct aw22xxx *aw22xxx = container_of(work, struct aw22xxx, recovery_work.work);
	int i;
	const struct firmware *fw = NULL;
	char cfg_name[AW22XXX_CFG_NAME_MAX];
	int ret;

	pr_info("%s: attempting to restore LED state from work\n", __func__);

	for (i = 0; i < aw22xxx->groups_num; i++) {
		if (aw22xxx->group_info_list[i].led_enabled) {
			mutex_lock(&aw22xxx->cfg_lock);
			pr_info("%s: restoring group %d\n", __func__, i);
			aw22xxx->group_id = aw22xxx->group_info_list[i].id;
			oplus_get_cfg_name(aw22xxx, aw22xxx->group_id, aw22xxx->group_info_list[i].effect_id, cfg_name, sizeof(cfg_name));
			ret = request_firmware(&fw, cfg_name, aw22xxx->dev);
			if (ret) {
				pr_err("%s: failed to request firmware %s, ret=%d\n", __func__, cfg_name, ret);
				mutex_unlock(&aw22xxx->cfg_lock);
				continue;
			}
			aw22xxx_cfg_loaded(fw, aw22xxx);
		}
	}

	if (oplus_get_group_enabled_num(aw22xxx) == 0) {
		oplus_led_standby(aw22xxx);
	}
}

static irqreturn_t aw22xxx_irq(int irq, void *data)
{
	struct aw22xxx *aw22xxx = data;

	aw22xxx_i2c_read(aw22xxx, REG_INTST, &aw22xxx->reg_intst);
	aw22xxx_i2c_read(aw22xxx, REG_PST, &aw22xxx->reg_pst);
	aw22xxx_i2c_read(aw22xxx, REG_GCR, &aw22xxx->reg_gcr);
	aw22xxx_i2c_read(aw22xxx, REG_MCUCTR, &aw22xxx->reg_mcuctr);
	aw22xxx_i2c_read(aw22xxx, REG_IMAX, &aw22xxx->reg_imax);
	pr_info("%s: reg GCR:0x02=0x%02x, MCUCTR:0x04=0x%02x, PST:0x07=0x%02x, INTST:0x0A=0x%02x, IMAX:0x0B=0x%02x\n",
			__func__,
			aw22xxx->reg_gcr,
			aw22xxx->reg_mcuctr,
			aw22xxx->reg_pst,
			aw22xxx->reg_intst,
			aw22xxx->reg_imax);
	if (aw22xxx->reg_intst & BIT_INTST_FUNCMPE) {
		pr_info("%s: functions compelte!\n", __func__);
		aw22xxx_reg_page_cfg(aw22xxx, AW22XXX_REG_PAGE0);
		aw22xxx_mcu_reset(aw22xxx, true);
		aw22xxx_mcu_enable(aw22xxx, false);
		aw22xxx_chip_enable(aw22xxx, false);
		aw22xxx_led_init(aw22xxx);
		aw22xxx->funcpme_irq_count++;
		pr_err("%s: get irq, funcpme_irq_count:%d\n", __func__, aw22xxx->funcpme_irq_count);
		oplus_handle_led_fault(aw22xxx, LED_FUNCPME_ERR_ID, false);
		schedule_delayed_work(&aw22xxx->recovery_work, msecs_to_jiffies(20));
	}
	if (aw22xxx->reg_intst & BIT_INTST_WTD) {
		pr_info("%s: get watch dog irq!\n", __func__);
		aw22xxx_reg_page_cfg(aw22xxx, AW22XXX_REG_PAGE0);
		aw22xxx_mcu_reset(aw22xxx, true);
		aw22xxx_mcu_reset(aw22xxx, false);
		aw22xxx_mcu_enable(aw22xxx, true);
		aw22xxx_led_init(aw22xxx);
		aw22xxx->wtd_irq_count++;
		pr_err("%s: get irq, wtd_irq_count:%d\n", __func__, aw22xxx->wtd_irq_count);
		oplus_handle_led_fault(aw22xxx, LED_WDT_ERR_ID, false);
		schedule_delayed_work(&aw22xxx->recovery_work, msecs_to_jiffies(20));
	}

	return IRQ_HANDLED;
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw22xxx_parse_dt(struct device *dev, struct aw22xxx *aw22xxx, struct device_node *np)
{
	aw22xxx->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	aw22xxx->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
#ifdef OPLUS_FEATURE_MISC_BASIC
	int ret;
	struct device_node *child;
	const char *name_ptr;
	int value = 0;
	int count = 0;
	int effect_map_index = 0;

	ret = of_property_read_u32(np, "vbled_volt",
			&aw22xxx->vbled_volt);
	if (ret < 0) {
		pr_err("%s: vbled_volt is not set\n", __func__);
		aw22xxx->vbled_volt = 0;
	}

	aw22xxx->backcolor_support = of_property_read_bool(np, "oplus,backcolor-support");
	pr_info("%s: backcolor-support set to %d\n", __func__, aw22xxx->backcolor_support);

	aw22xxx->color_mapping_support = of_property_read_bool(np, "oplus,color-mapping-support");
	pr_info("%s: color-mapping-support set to %d\n", __func__, aw22xxx->color_mapping_support);

	aw22xxx->trace_fault_support = of_property_read_bool(np, "oplus,trace-fault-support");
	pr_info("%s: trace-fault-support set to %d\n", __func__, aw22xxx->trace_fault_support);

	for_each_child_of_node(np, child) {
		if (!of_node_name_prefix(child, "oplus,group-")) {
			continue;
		}
		count++;
	}
	if (count == 0) {
		pr_err("%s: no valid group nodes found\n", __func__);
		return -EINVAL;
	}
	aw22xxx->group_info_list = devm_kzalloc(dev, sizeof(struct aw22xxx_group_info) * count, GFP_KERNEL);
	if (!aw22xxx->group_info_list) {
		pr_err("%s: failed to allocate memory for group info\n", __func__);
		return -ENOMEM;
	}
	aw22xxx->groups_num = count;
	pr_info("%s: groups_num=%d\n", __func__, aw22xxx->groups_num);

	for_each_child_of_node(np, child) {
		if (!of_node_name_prefix(child, "oplus,group-")) {
			continue;
		}

		ret = of_property_read_u32(child, "id", &value);
		if (ret < 0) {
			pr_err("%s: failed to read group id\n", __func__);
			return -EINVAL;
		}

		if (value < 0 || value > (aw22xxx->groups_num - 1)) {
			pr_err("%s: invalid group id %d\n", __func__, value);
			return -EINVAL;
		}
		struct aw22xxx_group_info *group = &aw22xxx->group_info_list[value];
		group->id = value;
		group->led_enabled = false;

		ret = of_property_read_string(child, "group-name", &name_ptr);
		if (ret < 0) {
			pr_err("%s: failed to read group[%d] name, using default\n", __func__, group->id);
			snprintf(group->name, sizeof(group->name), "group-%d", group->id);
		} else {
			snprintf(group->name, sizeof(group->name), "%s", name_ptr);
		}

		count = of_property_count_elems_of_size(child, "led-range", sizeof(u32));
		if (count <= 0) {
			pr_err("%s: failed to get group[%d] LED range length\n", __func__, group->id);
			return -EINVAL;
		}
		group->include_led_num = count;
		ret = of_property_read_u32_array(child, "led-range", group->include_led_index, count);
		if (ret < 0) {
			pr_err("%s: failed to read group[%d] LED range array\n", __func__, group->id);
			return -EINVAL;
		}

		ret = of_property_read_u32(child, "effect-map-index", &effect_map_index);
		if (ret < 0) {
			pr_err("%s: failed to read group[%d] map index\n", __func__, group->id);
			return -EINVAL;
		}
		if (effect_map_index < 0 || effect_map_index >= ARRAY_SIZE(aw22xxx_cfg_name_ptr)) {
			pr_err("%s: invalid effect_map_index = %u, exceeds max value %lu\n",
				__func__, effect_map_index, ARRAY_SIZE(aw22xxx_cfg_name_ptr) - 1);
			return -EINVAL;
		}
		group->effect_num_max = aw22xxx_cfg_name_sizes[effect_map_index];

		group->color_offsets_array = aw22xxx_color_offsets_ptr[effect_map_index];
		group->cfg_name_array = (char (*)[AW22XXX_CFG_NAME_MAX])aw22xxx_cfg_name_ptr[effect_map_index];

		pr_info("%s: ===== Group[%d] Information =====\n", __func__, group->id);
		pr_info("%s: group[%d] name: %s\n", __func__, group->id, group->name);
		pr_info("%s: group[%d] include_led_num: %u\n", __func__, group->id, group->include_led_num);
		pr_info("%s: group[%d] include_led_index: ", __func__, group->id);
		for (int i = 0; i < group->include_led_num; i++) {
			pr_cont("%u", group->include_led_index[i]);
			if (i < group->include_led_num - 1) {
				pr_cont(", ");
			}
		}
		pr_cont("\n");
		pr_info("%s: group[%d] group_brightness: %u\n", __func__, group->id, group->group_brightness);
		pr_info("%s: group[%d] effect_num_max: %u\n", __func__, group->id, group->effect_num_max);
	}
	oplus_init_exclusive_status(aw22xxx);
#endif
	return 0;
}

static int aw22xxx_hw_reset(struct aw22xxx *aw22xxx)
{
#ifdef OPLUS_FEATURE_MISC_BASIC
	int ret;

	if (!IS_ERR_OR_NULL(aw22xxx->vbled)) {
		pr_err("Enable the Regulator vbled.\n");
		ret = regulator_enable(aw22xxx->vbled);
		if (ret) {
			pr_err("Regulator vbled enable failed ret = %d\n", ret);
		}
	}
#endif

	if (aw22xxx && gpio_is_valid(aw22xxx->reset_gpio)) {
		gpio_set_value_cansleep(aw22xxx->reset_gpio, 0);
		usleep_range(1000, 1500);
		gpio_set_value_cansleep(aw22xxx->reset_gpio, 1);
		usleep_range(1000, 1500);
	} else {
		pr_err("%s: failed\n", __func__);
	}
	return 0;
}

static int aw22xxx_hw_off(struct aw22xxx *aw22xxx)
{
#ifdef OPLUS_FEATURE_MISC_BASIC
	int ret;
#endif

	if (aw22xxx && gpio_is_valid(aw22xxx->reset_gpio)) {
		gpio_set_value_cansleep(aw22xxx->reset_gpio, 0);
		usleep_range(1000, 1500);
	} else {
		pr_err("%s: failed\n", __func__);
	}


#ifdef OPLUS_FEATURE_MISC_BASIC
	if (!IS_ERR_OR_NULL(aw22xxx->vbled)) {
		pr_err("Disable the Regulator vbled.\n");
		ret = regulator_disable(aw22xxx->vbled);
		if (ret) {
			pr_err("Regulator vbled enable failed ret = %d\n", ret);
		}
	}
#endif


	return 0;
}

/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
static int aw22xxx_read_chipid(struct aw22xxx *aw22xxx)
{
	int ret = -1;
	unsigned char cnt = 0;
	unsigned char reg_val = 0;

	aw22xxx_reg_page_cfg(aw22xxx, AW22XXX_REG_PAGE0);
	aw22xxx_sw_reset(aw22xxx);

	while (cnt < AW_READ_CHIPID_RETRIES) {
		ret = aw22xxx_i2c_read(aw22xxx, REG_SRST, &reg_val);
		if (ret < 0) {
			pr_err(
				"%s: failed to read register AW22XXX_REG_ID: %d\n",
				__func__, ret);
			return -EIO;
		}
		switch (reg_val) {
		case AW22XXX_SRSTR:
			pr_info("%s aw22xxx detected\n", __func__);
			//aw22xxx->flags |= AW22XXX_FLAG_SKIP_INTERRUPTS;
			aw22xxx_i2c_read(aw22xxx, REG_CHIPID, &reg_val);
			switch (reg_val) {
			case AW22118_CHIPID:
				aw22xxx->chipid = AW22118;
				pr_info("%s: chipid: aw22118\n", __func__);
				break;
			case AW22127_CHIPID:
				aw22xxx->chipid = AW22127;
				pr_info("%s: chipid: aw22127\n", __func__);
				break;
			default:
				pr_err("%s: unknown id=0x%02x\n", __func__, reg_val);
				break;
			}
			return 0;
		default:
			pr_info("%s unsupported device revision (0x%x)\n",
				__func__, reg_val);
			break;
		}
		cnt++;

		msleep(AW_READ_CHIPID_RETRY_DELAY);
	}

	return -EINVAL;
}


/******************************************************
 *
 * sys group attribute: reg
 *
 ******************************************************/
static ssize_t
reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	unsigned int databuf[2] = {0, 0};

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
#ifdef OPLUS_FEATURE_MISC_BASIC
		pr_info("%s: set reg value: 0x%x with value: 0x%x", __func__, databuf[0], databuf[1]);
#endif
		aw22xxx_i2c_write(aw22xxx, databuf[0], databuf[1]);
	}
	return count;
}

static ssize_t reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	ssize_t len = 0;
	unsigned int i = 0;
	unsigned char reg_val = 0;
	unsigned char reg_page = 0;

	aw22xxx_i2c_read(aw22xxx, REG_PAGE, &reg_page);
	for (i = 0; i < AW22XXX_REG_MAX; i++) {
		if (!reg_page) {
			if (!(aw22xxx_reg_access[i]&REG_RD_ACCESS))
				continue;
		}
		aw22xxx_i2c_read(aw22xxx, i, &reg_val);
		len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%02x=0x%02x\n", i, reg_val);
	}
	return len;
}
static ssize_t
hwen_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	unsigned int databuf[1] = { 0 };
	int ret = -1;

	ret = kstrtou32(buf, 0, &databuf[0]);
	if (ret < 0) {
		pr_err("%s: input data invalid!", __func__);
		return ret;
	}
#ifdef OPLUS_FEATURE_MISC_BASIC
	pr_info("%s: set hwen value: %u", __func__, databuf[0]);
#endif
	if (databuf[0] == 1) {
		aw22xxx_hw_reset(aw22xxx);
	} else {
		aw22xxx_hw_off(aw22xxx);
	}

	return count;
}

static ssize_t
hwen_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len, "hwen=%d\n", gpio_get_value(aw22xxx->reset_gpio));

	return len;
}

static ssize_t
fw_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	unsigned int databuf[1] = { 0 };
	int ret = -1;

	ret = kstrtou32(buf, 0, &databuf[0]);
	if (ret < 0) {
		pr_err("%s: input data invalid!", __func__);
		return ret;
	}

	aw22xxx->fw_update = databuf[0];
#ifdef OPLUS_FEATURE_MISC_BASIC
	pr_info("%s: set fw value: %u", __func__, aw22xxx->fw_update);
#endif
	if (databuf[0] == 1){
		schedule_work(&aw22xxx->fw_work);
	}
	return count;
}

static ssize_t
fw_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len, "firmware name = %s\n", aw22xxx_fw_name);

	return len;
}

static ssize_t cfg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	unsigned int databuf[1] = { 0 };

#ifdef OPLUS_FEATURE_MISC_BASIC
	if (sscanf(buf, "%d", &databuf[0]) == 1) {
		if (databuf[0] != AW22XXX_CFG_TYPE_DEFAULT && databuf[0] != AW22XXX_CFG_TYPE_UPDATE_WITH_COLOR) {
			pr_err("%s: invalid cfg value %d\n", __func__, databuf[0]);
			return -EINVAL;
		}
		aw22xxx->group_id = 0;
		aw22xxx->cfg = databuf[0];
	} else {
		pr_err("%s: invalid para count\n", __func__);
		return -EINVAL;
	}
#else
	int ret = -1;
	ret = kstrtou32(buf, 0, &databuf[0]);
	if (ret < 0) {
		pr_err("%s: input data invalid!", __func__);
		return ret;
	}
	aw22xxx->cfg = databuf[0];
#endif

	pr_info("%s: set cfg value = %d\n", __func__, databuf[0]);
	if (aw22xxx->cfg)
		schedule_work(&aw22xxx->cfg_work);
	return count;
}

static ssize_t cfg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	unsigned int i;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

#ifdef OPLUS_FEATURE_MISC_BASIC
	struct aw22xxx_group_info *group = &aw22xxx->group_info_list[0];

	for (i = 0; i < group->effect_num_max; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "cfg[%x] = %s\n", i, group->cfg_name_array[i]);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "current cfg = %s\n", group->cfg_name_array[group->effect_id]);
#else
	for (i = 0; i < sizeof(aw22xxx_cfg_name) / AW22XXX_CFG_NAME_MAX; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
				"cfg[%x] = %s\n", i, aw22xxx_cfg_name[i]);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "current cfg = %s\n",
			aw22xxx_cfg_name[aw22xxx->effect]);
#endif

	return len;
}

static ssize_t effect_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int databuf[1] = { 0 };
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

#ifdef OPLUS_FEATURE_MISC_BASIC
	struct aw22xxx_group_info *group = &aw22xxx->group_info_list[0];
	int ret = 0;

	if (sscanf(buf, "%d", &databuf[0]) == 1) {
		ret = oplus_validate_effect_id(aw22xxx, 0, databuf[0]);
		if (!ret) {
			pr_err("%s: invalid effect id\n", __func__);
			return -EINVAL;
		}
		group->effect_id = databuf[0];
		pr_info("%s: set effect value = %u\n", __func__, databuf[0]);
	} else {
		pr_err("%s: invalid para count\n", __func__);
		return -EINVAL;
	}
#else
	int ret = -1;
	ret = kstrtou32(buf, 0, &databuf[0]);
	if (ret < 0) {
		pr_err("%s: input data invalid!", __func__);
		return ret;
	}
	aw22xxx->effect = databuf[0];
	pr_info("%s: set effect value = %d\n", __func__, aw22xxx->effect);
#endif

	return len;
}

static ssize_t effect_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	len += snprintf(buf+len, PAGE_SIZE-len, "effect = 0x%02x\n", aw22xxx->effect);

	return len;
}

static ssize_t
imax_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int databuf[1] = { 0 };
	int ret = -1;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	ret = kstrtou32(buf, 0, &databuf[0]);
	if (ret < 0) {
		pr_err("%s: input data invalid!", __func__);
		return ret;
	}

	aw22xxx->imax = databuf[0];
#ifdef OPLUS_FEATURE_MISC_BASIC
	pr_info("%s: set imax value: %u, imax code: %u", __func__, aw22xxx->imax, aw22xxx_imax_code[aw22xxx->imax]);
#endif
	aw22xxx_imax_cfg(aw22xxx, aw22xxx_imax_code[aw22xxx->imax]);

	return len;
}

static ssize_t imax_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	unsigned int i;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	for (i = 0; i < sizeof(aw22xxx_imax_name) / AW22XXX_IMAX_NAME_MAX; i++)
		len += snprintf(buf+len, PAGE_SIZE-len, "imax[%x] = %s\n", i, aw22xxx_imax_name[i]);

	len += snprintf(buf+len, PAGE_SIZE-len, "current id = 0x%02x, imax = %s\n",
		aw22xxx->imax, aw22xxx_imax_name[aw22xxx->imax]);

	return len;
}

static ssize_t rgb_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int databuf[2];
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

#ifdef OPLUS_FEATURE_MISC_BASIC
	int j;
	struct aw22xxx_group_info *group = &aw22xxx->group_info_list[0];

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		if (databuf[0] > (AW22XXX_RGB_MAX - 1)) {
			pr_err("led index error");
			return -EINVAL;
		}
		pr_info("%s: set rgb[%x] = %x\n", __func__, databuf[0], databuf[1]);
		group->rgb_value[databuf[0]] = databuf[1] & 0xffffff;
		aw22xxx->rgb[databuf[0]] = databuf[1] & 0xffffff;
	} else if (sscanf(buf, "%x", &databuf[0]) == 1) {
		pr_info("%s: set rgb all = %x\n", __func__, databuf[0]);
		for (j = 0; j < group->include_led_num; j++) {
			group->rgb_value[j] = databuf[0] & 0xffffff;
		}
		for (j = 0; j < AW22XXX_RGB_MAX; j++) {
			aw22xxx->rgb[j] = databuf[0] & 0xffffff;
		}
	} else {
		pr_err("%s: invalid para count\n", __func__);
		return -EINVAL;
	}
#else
	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2)
		aw22xxx->rgb[databuf[0]] = databuf[1];
#endif
	return len;
}

static ssize_t rgb_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	unsigned int i;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	for (i = 0; i < AW22XXX_RGB_MAX; i++)
		len += snprintf(buf+len, PAGE_SIZE-len, "rgb[%x] = 0x%06x\n", i, aw22xxx->rgb[i]);

	return len;
}
#ifdef OPLUS_FEATURE_MISC_BASIC
static bool oplus_check_music_color_changed(struct aw22xxx *aw22xxx, struct aw22xxx_group_info *group)
{
	int i;
	for (i = 0; i < group->include_led_num; i++) {
		if (group->rgb_value[i] != aw22xxx->music_current_color[i]) {
			return true;
		}
	}
	return false;
}
static void aw22xxx_music_set_all_led_color(struct aw22xxx *aw22xxx)
{
	unsigned char r_color, g_color, b_color;
	unsigned int color;
	struct aw22xxx_group_info *group = &aw22xxx->group_info_list[0];

	aw22xxx_i2c_write(aw22xxx, REG_PAGE, 0);
	pr_info("%s: set music led color", __func__);
	for (int i = 0; i < group->include_led_num; i++) {
		color = group->rgb_value[i];

		r_color = (color & 0x00ff0000) >> 16;
		g_color = (color & 0x0000ff00) >> 8;
		b_color = (color & 0x000000ff);

		aw22xxx_i2c_write(aw22xxx, REG_TASK0, CURRENT_CMD_BASE + i * 3);
		aw22xxx_i2c_write(aw22xxx, REG_TASK1, r_color);
		aw22xxx_i2c_write(aw22xxx, REG_MCUCTR, UPDATE_CMD);

		aw22xxx_i2c_write(aw22xxx, REG_TASK0, CURRENT_CMD_BASE + i * 3 + 1);
		aw22xxx_i2c_write(aw22xxx, REG_TASK1, g_color);
		aw22xxx_i2c_write(aw22xxx, REG_MCUCTR, UPDATE_CMD);

		aw22xxx_i2c_write(aw22xxx, REG_TASK0, CURRENT_CMD_BASE + i * 3 + 2);
		aw22xxx_i2c_write(aw22xxx, REG_TASK1, b_color);
		aw22xxx_i2c_write(aw22xxx, REG_MCUCTR, UPDATE_CMD);
	}
}

static int aw22xxx_music_set_all_leds_brightness(struct aw22xxx *aw22xxx, int brightness)
{
	struct aw22xxx_group_info *group = &aw22xxx->group_info_list[0];

	if (brightness < 0 || brightness > PWM_MAX) {
		pr_err("brightness error: 0 ~ 0x%02X", PWM_MAX);
		return -EINVAL;
	}
	if (!aw22xxx->music_color_set) {
		pr_err("%s: set color flag is not set", __func__);
		return -1;
	}
	pr_info("%s: set brightness for all LEDs: %d", __func__, brightness);

	for (int i = 0; i < (group->include_led_num * 3); i++) {
		aw22xxx_i2c_write(aw22xxx, REG_TASK0, PWM_CMD_BASE + i);
		aw22xxx_i2c_write(aw22xxx, REG_TASK1, brightness);
		aw22xxx_i2c_write(aw22xxx, REG_MCUCTR, UPDATE_CMD);
	}
	return 0;
}
static void aw22xxx_music_effect_init(struct aw22xxx *aw22xxx, struct aw22xxx_group_info *group)
{
	aw22xxx_cfg_update(aw22xxx);
	aw22xxx->music_min_brightness = PWM_MAX;
	aw22xxx->music_max_brightness = 0;
	aw22xxx->music_effect_loaded = true;
	memset(aw22xxx->music_current_color, 0, sizeof(aw22xxx->music_current_color));
	aw22xxx->music_brightness_last = 0;
}
static void aw22xxx_music_effect_update(struct aw22xxx *aw22xxx, struct aw22xxx_group_info *group, unsigned int current_led_brightness)
{
	unsigned int brightness = 0;
	unsigned int brightness_last = aw22xxx->music_brightness_last;
	bool color_changed = false;
	int i;
	unsigned int input_brightness = current_led_brightness & 0xFF;

	color_changed = oplus_check_music_color_changed(aw22xxx, group);
	if (aw22xxx->music_color_set == false || color_changed) {
		aw22xxx_music_set_all_led_color(aw22xxx);
		aw22xxx->music_color_set = true;
		for (i = 0; i < group->include_led_num; i++) {
			aw22xxx->music_current_color[i] = group->rgb_value[i];
		}
	}
	if (input_brightness < aw22xxx->music_min_brightness) {
		aw22xxx->music_min_brightness = input_brightness;
	}
	if (input_brightness > aw22xxx->music_max_brightness) {
		aw22xxx->music_max_brightness = input_brightness;
	}

	if (aw22xxx->music_max_brightness == aw22xxx->music_min_brightness) {
		brightness = input_brightness;
	} else {
		brightness = ((input_brightness - aw22xxx->music_min_brightness) * 255) / (aw22xxx->music_max_brightness - aw22xxx->music_min_brightness);
	}

	if (brightness > PWM_MAX) {
		pr_info("%s: brightness error\n", __func__);
		brightness = PWM_MAX;
	}

	if (brightness != brightness_last) {
		int diff = (int)brightness - (int)brightness_last;

		aw22xxx_music_set_all_leds_brightness(aw22xxx, brightness_last + diff / 3);
		pr_info("set brightness=%d\n", brightness_last + diff / 3);
		usleep_range(10000, 11000);

		aw22xxx_music_set_all_leds_brightness(aw22xxx, brightness_last + diff * 2 / 3);
		pr_info("set brightness=%d\n", brightness_last + diff * 2 / 3);
		usleep_range(10000, 11000);
	}

	aw22xxx_music_set_all_leds_brightness(aw22xxx, brightness);
	pr_info("set brightness=%d\n", brightness);
	aw22xxx->music_brightness_last = brightness;
}
static void aw22xxx_music_work_routine(struct work_struct *work)
{
	struct aw22xxx *aw22xxx = container_of(work, struct aw22xxx, music_work);
	struct aw22xxx_group_info *group = &aw22xxx->group_info_list[aw22xxx->group_id];

	aw22xxx_music_effect_update(aw22xxx, group, group->group_brightness);
}
static void aw22xxx_music_var_reset(struct aw22xxx *aw22xxx)
{
	aw22xxx->music_effect_loaded = false;
	aw22xxx->music_min_brightness = PWM_MAX;
	aw22xxx->music_max_brightness = 0;
	aw22xxx->music_color_set = false;
	memset(aw22xxx->music_current_color, 0, sizeof(aw22xxx->music_current_color));
	aw22xxx->music_brightness_last = 0;
}
static ssize_t group_cfg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int databuf[2] = {0};
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	struct aw22xxx_group_info *group;
	int ret = 0;

	if (sscanf(buf, "%d %d", &databuf[0], &databuf[1]) == 2) {
		ret = oplus_validate_group_id(aw22xxx, databuf[0]);
		if (!ret) {
			pr_err("%s: invalid group id\n", __func__);
			return -EINVAL;
		}

		if (databuf[1] != AW22XXX_CFG_TYPE_DEFAULT && databuf[1] != AW22XXX_CFG_TYPE_UPDATE_WITH_COLOR) {
			pr_err("%s: invalid cfg value %d\n", __func__, databuf[1]);
			return -EINVAL;
		}

		aw22xxx->group_id = databuf[0];
		aw22xxx->cfg = databuf[1];
		group = &aw22xxx->group_info_list[aw22xxx->group_id];
		pr_info("%s: set group_id = %d, cfg value = %d\n",  __func__, aw22xxx->group_id, aw22xxx->cfg);

		if (aw22xxx->group_id == 0 && group->effect_id == EFFECT_ID_MUSIC_PLAY) {
			if (!aw22xxx->music_effect_loaded) {
				aw22xxx_music_effect_init(aw22xxx, group);
			} else {
				schedule_work(&aw22xxx->music_work);
			}
		} else {
			aw22xxx_cfg_update(aw22xxx);
			aw22xxx_music_var_reset(aw22xxx);
		}
	} else {
		pr_err("%s: invalid para count\n", __func__);
		return -EINVAL;
	}
	return count;
}

static ssize_t group_cfg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	int i;

	len += snprintf(buf + len, PAGE_SIZE - len, "Current cfg value: %d\n", aw22xxx->cfg);
	len += snprintf(buf + len, PAGE_SIZE - len, "Current group_id: %d\n", aw22xxx->group_id);
	len += snprintf(buf + len, PAGE_SIZE - len, "LED standby status: %d\n", aw22xxx->led_standby);
	for (i = 0; i < aw22xxx->groups_num; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "  group[%d]: %d\n", i, aw22xxx->group_info_list[i].led_enabled);
	}
	return len;
}

static ssize_t group_effect_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int databuf[2] = {0};
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	struct aw22xxx_group_info *group;
	int ret = 0;

	mutex_lock(&aw22xxx->cfg_lock);
	if (sscanf(buf, "%d %d", &databuf[0], &databuf[1]) == 2) {
		ret = oplus_validate_group_id(aw22xxx, databuf[0]);
		if (!ret) {
			pr_err("%s: invalid group id\n", __func__);
			mutex_unlock(&aw22xxx->cfg_lock);
			return -EINVAL;
		}

		ret = oplus_validate_effect_id(aw22xxx, databuf[0], databuf[1]);
		if (!ret) {
			pr_err("%s: invalid effect id\n", __func__);
			mutex_unlock(&aw22xxx->cfg_lock);
			return -EINVAL;
		}

		group = &aw22xxx->group_info_list[databuf[0]];
		group->effect_id = databuf[1];
		pr_info("%s: set effect_group_id = %d, effect_id = %d\n", __func__, databuf[0], databuf[1]);
	} else {
		pr_err("%s: invalid para count\n", __func__);
		mutex_unlock(&aw22xxx->cfg_lock);
		return -EINVAL;
	}
	mutex_unlock(&aw22xxx->cfg_lock);
	return len;
}

static ssize_t group_effect_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	struct aw22xxx_group_info *group = NULL;
	int i;
	int j;

	for (i = 0; i < aw22xxx->groups_num; i++) {
		group = &aw22xxx->group_info_list[i];
		len += snprintf(buf + len, PAGE_SIZE - len, "group[%d]: effect_id = %d\n", group->id, group->effect_id);
		for (j = 0;j < group->effect_num_max; j++){
			len += snprintf(buf + len, PAGE_SIZE - len, "  effect[%d] name = %s\n", j, group->cfg_name_array[j]);
		}
	}

	return len;
}
static const struct color_mapping_table aw22xxx_color_map[] = {
	/*
	 * format: {input_color, {LED0, LED1, LED2, LED3, LED4, LED5, LED6, LED7, LED8}}
	 */
	{0xFF1200, {0xFF2200, 0xFF2200, 0xFF1200, 0xFF1200, 0xFF1200, 0xFF1200, 0, 0, 0}},
	{0xFF2500, {0xFF4400, 0xFF4400, 0xFF2500, 0xFF2500, 0xFF2500, 0xFF2500, 0, 0, 0}},
	{0xDD4100, {0xDD7700, 0xDD7700, 0xDD4100, 0xDD4100, 0xDD4100, 0xDD4100, 0, 0, 0}},
	{0xDD5D00, {0xFFAA00, 0xFFAA00, 0xDD5D00, 0xDD5D00, 0xDD5D00, 0xDD5D00, 0, 0, 0}},
	{0xDD7900, {0xFFDD00, 0xFFDD00, 0xDD7900, 0xDD7900, 0xDD7900, 0xDD7900, 0, 0, 0}},
	{0xBFCC00, {0xBFFF00, 0xBFFF00, 0xBFCC00, 0xBFCC00, 0xBFCC00, 0xBFCC00, 0, 0, 0}},
};
static unsigned int oplus_led_color_mapping(struct aw22xxx *aw22xxx, int led_index, unsigned int input_color)
{
	int i;
	int array_size = ARRAY_SIZE(aw22xxx_color_map);

	if (!aw22xxx->color_mapping_support) {
		pr_info("%s: color-mapping-support not supported\n", __func__);
		return input_color;
	}

	if (led_index < 0 || led_index >= AW22XXX_RGB_MAX) {
		pr_err("%s: led_index out of range\n", __func__);
		return input_color;
	}

	for (i = 0; i < array_size; i++) {
		if (input_color == aw22xxx_color_map[i].input_color) {
			unsigned int target = aw22xxx_color_map[i].mapped_colors[led_index];
			pr_info("%s: map led[%d] 0x%x -> 0x%x\n", __func__, led_index, input_color, target);
			return target;
		}
	}
	return input_color;
}
static ssize_t group_rgb_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int databuf[GROUPS_LED_NUM_MAX + 2];
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	int i = 0;
	int param_count = 0;
	char *token;
	char *buf_copy = kstrdup(buf, GFP_KERNEL);
	char *ptr = buf_copy;
	int ret = 0;

	if (!buf_copy) {
		pr_err("%s: failed to allocate memory for buffer copy\n", __func__);
		return -ENOMEM;
	}
	mutex_lock(&aw22xxx->cfg_lock);
	while ((token = strsep(&ptr, " ")) != NULL && param_count < GROUPS_LED_NUM_MAX + 2) {
		if (strlen(token) == 0) {
			continue;
		}
		if (kstrtouint(token, 16, &databuf[param_count]) != 0) {
			pr_err("%s: invalid parameter format: %s\n", __func__, token);
			kfree(buf_copy);
			mutex_unlock(&aw22xxx->cfg_lock);
			return -EINVAL;
		}
		param_count++;
	}
	kfree(buf_copy);
	if (param_count < 0) {
		pr_err("%s: invalid param_count\n", __func__);
		mutex_unlock(&aw22xxx->cfg_lock);
		return -EINVAL;
	}

	ret = oplus_validate_group_id(aw22xxx, databuf[0]);
	if (!ret) {
		pr_err("%s: invalid group_id\n", __func__);
		mutex_unlock(&aw22xxx->cfg_lock);
		return -EINVAL;
	}

	struct aw22xxx_group_info *group = &aw22xxx->group_info_list[databuf[0]];
	if (param_count == 2) {
		unsigned int input_color = databuf[1] & 0xffffff;
		for (i = 0; i < group->include_led_num; i++) {
			int led_index = group->include_led_index[i];
			unsigned int target_color;
			target_color = oplus_led_color_mapping(aw22xxx, led_index, input_color);
			group->rgb_value[i] = target_color;
			aw22xxx->rgb[led_index] = target_color;
		}
		pr_info("%s: successfully set group %d all LEDs RGB to 0x%x\n", __func__, databuf[0], databuf[1]);
	} else if (param_count == group->include_led_num+1) {
		for (i = 0; i < group->include_led_num; i++) {
			int led_index = group->include_led_index[i];
			group->rgb_value[i] = databuf[i + 1] & 0xffffff;
			aw22xxx->rgb[led_index] = databuf[i + 1];
			pr_info("%s: set rgb[%d] = 0x%x\n", __func__, led_index, databuf[i + 1]);
		}
		pr_info("%s: successfully set group %d each LED RGB individually\n", __func__, databuf[0]);

	} else {
		pr_err("%s: invalid para count\n", __func__);
		mutex_unlock(&aw22xxx->cfg_lock);
		return -EINVAL;
	}
	mutex_unlock(&aw22xxx->cfg_lock);
	return len;
}

static ssize_t group_rgb_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	unsigned int i, j;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	int led_index = 0;
	struct aw22xxx_group_info *group = NULL;

	for (i = 0; i < aw22xxx->groups_num; i++) {
		group = &aw22xxx->group_info_list[i];
		len += snprintf(buf + len, PAGE_SIZE - len, "group[%d] \"%s\" (LEDs: %d):\n",
			group->id, group->name, group->include_led_num);

		for (j = 0; j < group->include_led_num; j++) {
			led_index = group->include_led_index[j];
			len += snprintf(buf + len, PAGE_SIZE - len, "  led[%d] = 0x%x\n", led_index, group->rgb_value[j]);
		}
		len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	}

	return len;
}

static ssize_t group_brightness_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int databuf[2] = {0};
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	int ret = 0;

	mutex_lock(&aw22xxx->cfg_lock);
	if (sscanf(buf, "%d %x", &databuf[0], &databuf[1]) == 2) {
		ret = oplus_validate_group_id(aw22xxx, databuf[0]);
		if (!ret) {
			pr_err("%s: invalid group id\n", __func__);
			mutex_unlock(&aw22xxx->cfg_lock);
			return -EINVAL;
		}

		aw22xxx->group_info_list[databuf[0]].group_brightness = databuf[1];
		pr_info("%s: successfully set group %d brightness 0x%x\n", __func__, databuf[0], databuf[1]);
	}
	mutex_unlock(&aw22xxx->cfg_lock);
	return len;
}

static ssize_t group_brightness_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	unsigned int i;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	for (i = 0; i < aw22xxx->groups_num; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "group[%d] = 0x%x\n",
			aw22xxx->group_info_list[i].id,
			aw22xxx->group_info_list[i].group_brightness);
	}

	return len;
}
static ssize_t abnormal_reset_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	return snprintf(buf, PAGE_SIZE, "%u\n", aw22xxx->abnormal_reset_check_count);
}
static ssize_t abnormal_reset_count_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	unsigned int databuf[1] = { 0 };
	int ret = -1;

	ret = kstrtou32(buf, 10, &databuf[0]);
	if (ret < 0) {
		pr_err("%s: input data invalid!", __func__);
		return ret;
	}

	pr_info("%s: update abnormal_reset_check_count from %u to %u\n",
			__func__, aw22xxx->abnormal_reset_check_count, databuf[0]);
	aw22xxx->abnormal_reset_check_count = databuf[0];

	return len;
}
static ssize_t check_period_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	return snprintf(buf, PAGE_SIZE, "check_period:%u\n", aw22xxx->check_period);
}

static ssize_t check_period_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	unsigned int databuf[1] = { 0 };
	int ret = -1;

	ret = kstrtou32(buf, 10, &databuf[0]);
	if (ret < 0) {
		pr_err("%s: input data invalid!", __func__);
		return ret;
	}

	if (databuf[0] == 0) {
		pr_info("%s: disable abnormal reset check\n", __func__);
		aw22xxx->abnormal_reset_check_enabled = false;
		aw22xxx->check_period = 0;
	} else {
		pr_info("%s: enable abnormal reset check immediately\n", __func__);
		aw22xxx->abnormal_reset_check_enabled = true;
		pr_info("%s: update check_period from %u to %u\n", __func__, aw22xxx->check_period, databuf[0]);
		aw22xxx->check_period = databuf[0];
		cancel_delayed_work_sync(&aw22xxx->abnormal_reset_check_work);
		schedule_delayed_work(&aw22xxx->abnormal_reset_check_work, msecs_to_jiffies(0));
	}

	return len;
}
static ssize_t funcpme_irq_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	return snprintf(buf, PAGE_SIZE, "%u\n", aw22xxx->funcpme_irq_count);
}
static ssize_t funcpme_irq_count_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	unsigned int databuf[1] = { 0 };
	int ret = -1;

	ret = kstrtou32(buf, 10, &databuf[0]);
	if (ret < 0) {
		pr_err("%s: input data invalid!", __func__);
		return ret;
	}

	pr_info("%s: update funcpme_irq_count from %u to %u\n",
			__func__, aw22xxx->funcpme_irq_count, databuf[0]);
	aw22xxx->funcpme_irq_count = databuf[0];

	return len;
}
static ssize_t wtd_irq_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	return snprintf(buf, PAGE_SIZE, "%u\n", aw22xxx->wtd_irq_count);
}
static ssize_t wtd_irq_count_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	unsigned int databuf[1] = { 0 };
	int ret = -1;

	ret = kstrtou32(buf, 10, &databuf[0]);
	if (ret < 0) {
		pr_err("%s: input data invalid!", __func__);
		return ret;
	}

	pr_info("%s: update wtd_irq_count from %u to %u\n",
			__func__, aw22xxx->wtd_irq_count, databuf[0]);
	aw22xxx->wtd_irq_count = databuf[0];

	return len;
}

static ssize_t back_color_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	return snprintf(buf, PAGE_SIZE, "%x\n", aw22xxx->back_color);
}

static ssize_t back_color_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	unsigned int val = 0;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	pr_info("%s: val = %u\n", __func__, val);
	if (ret < 0) {
		pr_err("%s: input data invalid!\n", __func__);
		return ret;
	}

	switch (val) {
	case 0xFFF6DCDA: /* pink */
	case 0xFF6E7377: /* gray */
	case 0xFF313131: /* carbon */
		aw22xxx->back_color = 0;
		break;
	case 0xFFF7F5F5: /* white */
		aw22xxx->back_color = 1;
		break;
	default:
		aw22xxx->back_color = 0;
		pr_err("%s: unknown back color 0x%x, mapped to 0\n", __func__, val);
		break;
	}

	pr_info("%s: raw color 0x%x mapped to index %d\n", __func__, val, aw22xxx->back_color);
	return count;
}

static ssize_t led_fault_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	ssize_t len = 0;
	unsigned char reg_02 = 0, reg_04 = 0, reg_07 = 0, reg_0a = 0, reg_0b = 0;
	int i = 0;
	int j = 0;

	aw22xxx_i2c_read(aw22xxx, REG_GCR, &reg_02);
	aw22xxx_i2c_read(aw22xxx, REG_MCUCTR, &reg_04);
	aw22xxx_i2c_read(aw22xxx, REG_PST, &reg_07);
	aw22xxx_i2c_read(aw22xxx, REG_INTST, &reg_0a);
	aw22xxx_i2c_read(aw22xxx, REG_IMAX, &reg_0b);

	len += snprintf(buf + len, PAGE_SIZE - len, "--- LED Fault Report Info ---\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "Trace Fault Support: %d\n", aw22xxx->trace_fault_support);
	len += snprintf(buf + len, PAGE_SIZE - len, "Abnormal Reset Count: %d\n", aw22xxx->abnormal_reset_check_count);
	len += snprintf(buf + len, PAGE_SIZE - len, "Funcpme IRQ Count: %d\n", aw22xxx->funcpme_irq_count);
	len += snprintf(buf + len, PAGE_SIZE - len, "WDT IRQ Count: %d\n", aw22xxx->wtd_irq_count);
	len += snprintf(buf + len, PAGE_SIZE - len, "FW Version: 0x%x\n", aw22xxx->fw_version);
	len += snprintf(buf + len, PAGE_SIZE - len, "Current Group ID: %d\n", aw22xxx->group_id);
	len += snprintf(buf + len, PAGE_SIZE - len, "Current Effect ID: %d\n", aw22xxx->group_info_list[aw22xxx->group_id].effect_id);
	len += snprintf(buf + len, PAGE_SIZE - len, "REG_GCR(0x02): 0x%02x\n", reg_02);
	len += snprintf(buf + len, PAGE_SIZE - len, "REG_MCUCTR(0x04): 0x%02x\n", reg_04);
	len += snprintf(buf + len, PAGE_SIZE - len, "REG_PST(0x07): 0x%02x\n", reg_07);
	len += snprintf(buf + len, PAGE_SIZE - len, "REG_INTST(0x0A): 0x%02x\n", reg_0a);
	len += snprintf(buf + len, PAGE_SIZE - len, "REG_IMAX(0x0B): 0x%02x\n", reg_0b);

	len += snprintf(buf + len, PAGE_SIZE - len, "--- Fault Report Counts (24h period) ---\n");
	for (i = 1; i < MAX_LED_FAULT_EVENT_ID; i++) {
		const char *err_name = "Unknown";
		for (j = 0; j < ARRAY_SIZE(g_led_fb_conf); j++) {
			if (g_led_fb_conf[j].event_id == i) {
				err_name = g_led_fb_conf[j].fb_field;
				break;
			}
		}
		len += snprintf(buf + len, PAGE_SIZE - len, "Event ID %d (%s): %u\n",
						i, err_name, aw22xxx->fault_report_count[i]);
	}

	return len;
}

static ssize_t led_fault_test_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	unsigned int val = 0;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0) {
		pr_err("%s: input data invalid!\n", __func__);
		return ret;
	}
	if (val < LED_WDT_ERR_ID || val >= MAX_LED_FAULT_EVENT_ID) {
		pr_err("%s: invalid event_id %u, must be between %d and %d\n",
		       __func__, val, LED_WDT_ERR_ID, MAX_LED_FAULT_EVENT_ID - 1);
		return -EINVAL;
	}
	pr_info("%s: triggering led fault report test for event_id = %u\n", __func__, val);

	aw22xxx_i2c_read(aw22xxx, REG_GCR, &aw22xxx->reg_gcr);
	aw22xxx_i2c_read(aw22xxx, REG_MCUCTR, &aw22xxx->reg_mcuctr);
	aw22xxx_i2c_read(aw22xxx, REG_PST, &aw22xxx->reg_pst);
	aw22xxx_i2c_read(aw22xxx, REG_INTST, &aw22xxx->reg_intst);
	aw22xxx_i2c_read(aw22xxx, REG_IMAX, &aw22xxx->reg_imax);

	oplus_handle_led_fault(aw22xxx, (uint16_t)val, true);

	return count;
}
#endif
static ssize_t
task0_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int databuf[1] = { 0 };
	int ret = -1;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	ret = kstrtou32(buf, 0, &databuf[0]);
	if (ret < 0) {
		pr_err("%s: input data invalid!", __func__);
		return ret;
	}

	aw22xxx->task0 = databuf[0];
#ifdef OPLUS_FEATURE_MISC_BASIC
	pr_info("%s: set task0 value: %u", __func__, aw22xxx->task0);
#endif
	schedule_work(&aw22xxx->task_work);

	return len;
}

static ssize_t task0_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	len += snprintf(buf+len, PAGE_SIZE-len, "task0 = 0x%02x\n", aw22xxx->task0);

	return len;
}

static ssize_t
task1_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int databuf[1] = { 0 };
	int ret = -1;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	ret = kstrtou32(buf, 0, &databuf[0]);
	if (ret < 0) {
		pr_err("%s: input data invalid!", __func__);
		return ret;
	}

	aw22xxx->task1 = databuf[0];
#ifdef OPLUS_FEATURE_MISC_BASIC
	pr_info("%s: set task1 value: %u", __func__, aw22xxx->task1);
#endif
	return len;
}

static ssize_t task1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	len += snprintf(buf+len, PAGE_SIZE-len, "task1 = 0x%02x\n", aw22xxx->task1);

	return len;
}

static ssize_t signal_led_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	int rgb_index, r_g_b, color, br;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	int ret;

	if (sscanf(buf, "%d %d %d %d", &rgb_index, &r_g_b, &color, &br) == 4) {
#ifdef OPLUS_FEATURE_MISC_BASIC
		pr_info("%s: set signal_led value: rgb_index = %d, r_g_b = %d, color = %d, br = %d",
				__func__, rgb_index, r_g_b, color, br);
#endif
		if (rgb_index < 0 || rgb_index > (AW22XXX_RGB_MAX - 1)) {
			pr_err("rgb_index error: 0 ~ 8");
			return len;
		}
		if (r_g_b < 0 || r_g_b > 2) {
			pr_err("r_g_b error: 0 ~ 2");
			return len;
		}
		if (color < 0 || color > CURRENT_MAX) {
			pr_err("color error: 0 ~ 0x%02X", CURRENT_MAX);
			return len;
		}
		if (br < 0 || br > PWM_MAX) {
			pr_err("br error: 0 ~ 0x%02X", PWM_MAX);
			return len;
		}
		ret = aw22xxx_i2c_write(aw22xxx, REG_PAGE, 0);
		if (ret < 0) {
			pr_err("i2c_write error");
			return ret;
		}
		ret = aw22xxx_i2c_write(aw22xxx, REG_AUDCTR, 0);
		if (ret < 0) {
			pr_err("i2c_write error");
			return ret;
		}
		ret = aw22xxx_i2c_write(aw22xxx, REG_TASK0,
				CURRENT_CMD_BASE + rgb_index * 3 + r_g_b);
		if (ret < 0) {
			pr_err("i2c_write error");
			return ret;
		}
		ret = aw22xxx_i2c_write(aw22xxx, REG_TASK1, color);
		if (ret < 0) {
			pr_err("i2c_write error");
			return ret;
		}
		ret = aw22xxx_i2c_write(aw22xxx, REG_MCUCTR, UPDATE_CMD);
		if (ret < 0) {
			pr_err("i2c_write error");
			return ret;
		}
		ret = aw22xxx_i2c_write(aw22xxx, REG_TASK0,
				PWM_CMD_BASE + rgb_index * 3 + r_g_b);
		if (ret < 0) {
			pr_err("i2c_write error");
			return ret;
		}
		ret = aw22xxx_i2c_write(aw22xxx, REG_TASK1, br);
		if (ret < 0) {
			pr_err("i2c_write error");
			return ret;
		}
		ret = aw22xxx_i2c_write(aw22xxx, REG_MCUCTR, UPDATE_CMD);
		if (ret < 0) {
			pr_err("i2c_write error");
			return ret;
		}
	}

	return len;
}

static DEVICE_ATTR_RW(reg);
static DEVICE_ATTR_RW(hwen);
static DEVICE_ATTR_RW(fw);
static DEVICE_ATTR_RW(cfg);
static DEVICE_ATTR_RW(effect);
static DEVICE_ATTR_RW(imax);
static DEVICE_ATTR_RW(rgb);
static DEVICE_ATTR_RW(task0);
static DEVICE_ATTR_RW(task1);
static DEVICE_ATTR_WO(signal_led);
#ifdef OPLUS_FEATURE_MISC_BASIC
static DEVICE_ATTR_RW(group_cfg);
static DEVICE_ATTR_RW(group_effect);
static DEVICE_ATTR_RW(group_rgb);
static DEVICE_ATTR_RW(group_brightness);
static DEVICE_ATTR_RW(abnormal_reset_count);
static DEVICE_ATTR_RW(funcpme_irq_count);
static DEVICE_ATTR_RW(wtd_irq_count);
static DEVICE_ATTR_RW(check_period);
static DEVICE_ATTR_RW(back_color);
static DEVICE_ATTR_RW(led_fault_test);
#endif

static struct attribute *aw22xxx_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_hwen.attr,
	&dev_attr_fw.attr,
	&dev_attr_cfg.attr,
	&dev_attr_effect.attr,
	&dev_attr_imax.attr,
	&dev_attr_rgb.attr,
	&dev_attr_task0.attr,
	&dev_attr_task1.attr,
	&dev_attr_signal_led.attr,
#ifdef OPLUS_FEATURE_MISC_BASIC
	&dev_attr_group_cfg.attr,
	&dev_attr_group_effect.attr,
	&dev_attr_group_rgb.attr,
	&dev_attr_group_brightness.attr,
	&dev_attr_abnormal_reset_count.attr,
	&dev_attr_funcpme_irq_count.attr,
	&dev_attr_wtd_irq_count.attr,
	&dev_attr_check_period.attr,
	&dev_attr_back_color.attr,
	&dev_attr_led_fault_test.attr,
#endif
	NULL
};

static struct attribute_group aw22xxx_attribute_group = {
	.attrs = aw22xxx_attributes
};


/******************************************************
 *
 * led class dev
 *
 ******************************************************/
static int aw22xxx_parse_led_cdev(struct aw22xxx *aw22xxx, struct device_node *np)
{
	struct device_node *temp;
	int ret = -1;

	for_each_child_of_node(np, temp) {
#ifdef OPLUS_FEATURE_MISC_BASIC
		if (!of_node_name_prefix(temp, "aw22xxx,led")) {
			continue;
		}
#endif
		ret = of_property_read_string(temp, "aw22xxx,name", &aw22xxx->cdev.name);
		if (ret < 0) {
			pr_err("Failure reading led name, ret = %d\n", ret);
			goto free_pdata;
		}
		ret = of_property_read_u32(temp, "aw22xxx,imax", &aw22xxx->imax);
		if (ret < 0) {
			pr_err("Failure reading imax, ret = %d\n", ret);
			goto free_pdata;
		}
		ret = of_property_read_u32(temp, "aw22xxx,brightness", &aw22xxx->cdev.brightness);
		if (ret < 0) {
			pr_err("Failure reading brightness, ret = %d\n", ret);
			goto free_pdata;
		}
		ret = of_property_read_u32(temp, "aw22xxx,max_brightness",
			&aw22xxx->cdev.max_brightness);
		if (ret < 0) {
			pr_err("Failure reading max brightness, ret = %d\n", ret);
			goto free_pdata;
		}
	}

	INIT_WORK(&aw22xxx->brightness_work, aw22xxx_brightness_work);
	INIT_WORK(&aw22xxx->task_work, aw22xxx_task_work);

	aw22xxx->cdev.brightness_set = aw22xxx_set_brightness;
	ret = led_classdev_register(aw22xxx->dev, &aw22xxx->cdev);
	if (ret) {
		pr_err("unable to register led ret=%d\n", ret);
		goto free_pdata;
	}

	ret = sysfs_create_group(&aw22xxx->cdev.dev->kobj, &aw22xxx_attribute_group);
	if (ret) {
		pr_err("led sysfs ret: %d\n", ret);
		goto free_class;
	}

	return 0;

free_class:
	led_classdev_unregister(&aw22xxx->cdev);
free_pdata:
	return ret;
}

/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int aw22xxx_i2c_probe(struct i2c_client *i2c)
{
	struct aw22xxx *aw22xxx;
	struct device_node *np = i2c->dev.of_node;
	int ret;
	int irq_flags;


	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		pr_err("check_functionality failed\n");
		return -EIO;
	}

	aw22xxx = devm_kzalloc(&i2c->dev, sizeof(struct aw22xxx), GFP_KERNEL);
	if (aw22xxx == NULL)
		return -ENOMEM;
	memset(aw22xxx, 0, sizeof(struct aw22xxx));
	aw22xxx->dev = &i2c->dev;
	aw22xxx->i2c = i2c;

	i2c_set_clientdata(i2c, aw22xxx);

	mutex_init(&aw22xxx->cfg_lock);

	/* aw22xxx rst & int */
	if (np) {
		ret = aw22xxx_parse_dt(&i2c->dev, aw22xxx, np);
		if (ret) {
			pr_err("%s: failed to parse device tree node\n", __func__);
			goto err_parse_dt;
		}
	} else {
		aw22xxx->reset_gpio = -1;
		aw22xxx->irq_gpio = -1;
	}
	pr_info("aw22xxx->reset_gpio:%d,irq:%d\n",aw22xxx->reset_gpio,aw22xxx->irq_gpio);
	if (gpio_is_valid(aw22xxx->reset_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw22xxx->reset_gpio,
			GPIOF_OUT_INIT_LOW, "aw22xxx_rst");
		if (ret) {
			pr_err("%s: rst request failed\n", __func__);
			goto err_gpio_request;
		}
	}

	if (gpio_is_valid(aw22xxx->irq_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw22xxx->irq_gpio,
			GPIOF_DIR_IN, "aw22xxx_int");
		if (ret) {
			pr_err("%s: int request failed\n", __func__);
			goto err_gpio_request;
		}
	}

#ifdef OPLUS_FEATURE_MISC_BASIC
	/* vdd 3.3v*/
	aw22xxx->vbled = regulator_get(aw22xxx->dev, "vbled");
	if (!aw22xxx->vbled_volt || IS_ERR_OR_NULL(aw22xxx->vbled)) {
		pr_err("Regulator vdd3v3 get failed\n");
	} else {
		pr_err("Regulator vbled volt set %u \n", aw22xxx->vbled_volt);
		if (aw22xxx->vbled_volt) {
			ret = regulator_set_voltage(aw22xxx->vbled, aw22xxx->vbled_volt,
							aw22xxx->vbled_volt);
		} else {
			ret = regulator_set_voltage(aw22xxx->vbled, 3300000, 3300000);
		}
		if (ret) {
			pr_err("Regulator set_vtg failed vdd rc = %d\n", ret);
		}
	}
#endif

	/* hardware reset */
	aw22xxx_hw_reset(aw22xxx);

	/* aw22xxx chip id */
	ret = aw22xxx_read_chipid(aw22xxx);
	if (ret < 0) {
		pr_err("%s: aw22xxx_read_chipid failed ret=%d\n", __func__, ret);
		goto err_id;
	}

	/* aw22xxx irq */
	if (gpio_is_valid(aw22xxx->irq_gpio) &&
		!(aw22xxx->flags & AW22XXX_FLAG_SKIP_INTERRUPTS)) {
		/* register irq handler */
		aw22xxx_interrupt_setup(aw22xxx);
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(&i2c->dev,
					gpio_to_irq(aw22xxx->irq_gpio),
					NULL, aw22xxx_irq, irq_flags,
					"aw22xxx", aw22xxx);
		if (ret != 0) {
			pr_err("%s: failed to request IRQ %d: %d\n",
					__func__, gpio_to_irq(aw22xxx->irq_gpio), ret);
			goto err_irq;
		}
	} else {
		pr_info("%s skipping IRQ registration\n", __func__);
		/* disable feature support if gpio was invalid */
		aw22xxx->flags |= AW22XXX_FLAG_SKIP_INTERRUPTS;
	}

	dev_set_drvdata(&i2c->dev, aw22xxx);

	ret = aw22xxx_parse_led_cdev(aw22xxx, np);
	if (ret < 0) {
		pr_err("%s error creating led class dev\n", __func__);
		goto err_sysfs;
	}

	aw22xxx_fw_init(aw22xxx);
	pr_info("%s probe completed successfully!\n", __func__);

	return 0;

err_sysfs:
	devm_free_irq(&i2c->dev, gpio_to_irq(aw22xxx->irq_gpio), aw22xxx);
err_irq:
err_id:
	if (gpio_is_valid(aw22xxx->reset_gpio))
		gpio_free(aw22xxx->reset_gpio);
	if (gpio_is_valid(aw22xxx->irq_gpio))
		gpio_free(aw22xxx->irq_gpio);
err_gpio_request:
err_parse_dt:
	devm_kfree(&i2c->dev, aw22xxx);
	aw22xxx = NULL;
	return ret;
}

static void aw22xxx_i2c_remove(struct i2c_client *i2c)
{
	struct aw22xxx *aw22xxx = i2c_get_clientdata(i2c);

	cancel_delayed_work_sync(&aw22xxx->led_standby_work);
	cancel_delayed_work_sync(&aw22xxx->abnormal_reset_check_work);
	cancel_delayed_work_sync(&aw22xxx->recovery_work);
	sysfs_remove_group(&aw22xxx->cdev.dev->kobj, &aw22xxx_attribute_group);
	led_classdev_unregister(&aw22xxx->cdev);

	devm_free_irq(&i2c->dev, gpio_to_irq(aw22xxx->irq_gpio), aw22xxx);

	if (gpio_is_valid(aw22xxx->reset_gpio))
		gpio_free(aw22xxx->reset_gpio);
	if (gpio_is_valid(aw22xxx->irq_gpio))
		gpio_free(aw22xxx->irq_gpio);

	devm_kfree(&i2c->dev, aw22xxx);
	aw22xxx = NULL;
}

static const struct i2c_device_id aw22xxx_i2c_id[] = {
	{ AW22XXX_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw22xxx_i2c_id);

static const struct of_device_id aw22xxx_dt_match[] = {
	{ .compatible = "awinic,aw22xxx_led" },
	{ },
};

static struct i2c_driver aw22xxx_i2c_driver = {
	.driver = {
		.name = AW22XXX_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw22xxx_dt_match),
	},
	.probe = aw22xxx_i2c_probe,
	.remove = aw22xxx_i2c_remove,
	.id_table = aw22xxx_i2c_id,
};


static int __init aw22xxx_i2c_init(void)
{
	int ret = 0;

	pr_info("aw22xxx driver version %s\n", AW22XXX_DRIVER_VERSION);

	ret = i2c_add_driver(&aw22xxx_i2c_driver);
	if (ret) {
		pr_err("fail to add aw22xxx device into i2c\n");
		return ret;
	}

	return 0;
}
module_init(aw22xxx_i2c_init);


static void __exit aw22xxx_i2c_exit(void)
{
	i2c_del_driver(&aw22xxx_i2c_driver);
}
module_exit(aw22xxx_i2c_exit);


MODULE_DESCRIPTION("AW22XXX LED Driver");
MODULE_LICENSE("GPL v2");
