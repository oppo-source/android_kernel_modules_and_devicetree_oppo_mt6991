/*
 * Copyright (C) 2024 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 jinggangshanfront_otp.h
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 sensor otp header file
 *
 ****************************************************************************/
#ifndef __JINGGANGSHANFRONT_OTP_H
#define __JINGGANGSHANFRONT_OTP_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/dma-mapping.h>
#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_errcode.h"
#include "oplus-adaptor-subdrv-ctrl.h"
#include "adaptor-subdrv-ctrl.h"
#include "adaptor-i2c.h"
#include "adaptor.h"

#define OTP_I2C_ADDR 0x6C
#define JINGGANGSHANFRONT_OTP_RET_FAIL -1
#define JINGGANGSHANFRONT_OTP_RET_SUCCESS 0
#define JINGGANGSHANFRONT_TXD_FRONT_OTP_MODULE_LENS 18
#define JINGGANGSHANFRONT_TXD_FRONT_OTP_AWB_LENS    18
#define JINGGANGSHANFRONT_TXD_FRONT_OTP_LRC_LENS    6
#define JINGGANGSHANFRONT_TXD_FRONT_OTP_SN_LENS 	 25
#define JINGGANGSHANFRONT_TXD_FRONT_OTP_LSC_LENS    1870

//flag addr
#define JINGGANGSHANFRONT_OTP_MODULE_FLAGADDR 0x827A
#define JINGGANGSHANFRONT_GROUP1_FLAG 0x01
#define JINGGANGSHANFRONT_GROUP2_FLAG 0x13
#define JINGGANGSHANFRONT_INVALID_FLAG 0x0

//data start addr
#define JINGGANGSHANFRONT_OTP_MODULE_GROUP1_STARTADDR 0x827B
#define JINGGANGSHANFRONT_OTP_AWB_GROUP1_STARTADDR 0x829F
#define JINGGANGSHANFRONT_OTP_LRC_GROUP1_STARTADDR 0x82C3
#define JINGGANGSHANFRONT_OTP_SN_GROUP1_STARTADDR 0x82CF

#define JINGGANGSHANFRONT_OTP_MODULE_GROUP2_STARTADDR 0x828D
#define JINGGANGSHANFRONT_OTP_AWB_GROUP2_STARTADDR 0x82B1
#define JINGGANGSHANFRONT_OTP_LRC_GROUP2_STARTADDR 0x82C9
#define JINGGANGSHANFRONT_OTP_SN_GROUP2_STARTADDR 0x82E9

enum jinggangshanfront_sensor_otp_page{
	page_0 = 0,
	page_1,
	page_2,
	page_3,
	page_4,
	page_5,
	page_6,
	page_7,
	page_8,
	page_9,
	page_10,
	page_11,
	page_12,
	page_max
};

struct jinggangshanfront_txd_front_otp_struct {
	UINT8 ModuleFlag;
	UINT8 module_info[JINGGANGSHANFRONT_TXD_FRONT_OTP_MODULE_LENS];
	UINT8 wb_data[JINGGANGSHANFRONT_TXD_FRONT_OTP_AWB_LENS];
	UINT8 lrc_data[JINGGANGSHANFRONT_TXD_FRONT_OTP_LRC_LENS];
	UINT8 sn_data[JINGGANGSHANFRONT_TXD_FRONT_OTP_SN_LENS];
	UINT8 lsc_data[JINGGANGSHANFRONT_TXD_FRONT_OTP_LSC_LENS];
};

extern struct jinggangshanfront_txd_front_otp_struct jinggangshanfront_txd_front_otp;
extern int jinggangshanfront_sensor_otp_read_all_data(struct i2c_client *client);

#endif
