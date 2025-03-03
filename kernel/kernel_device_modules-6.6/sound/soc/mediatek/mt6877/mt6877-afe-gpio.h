/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mt6877-afe-gpio.h  --  Mediatek 6877 afe gpio ctrl definition
 *
 * Copyright (c) 2020 MediaTek Inc.
 * Author: Eason Yen <eason.yen@mediatek.com>
 */

#ifndef _MT6877_AFE_GPIO_H_
#define _MT6877_AFE_GPIO_H_

enum mt6877_afe_gpio {
	MT6877_AFE_GPIO_DAT_MISO0_OFF,
	MT6877_AFE_GPIO_DAT_MISO0_ON,
	MT6877_AFE_GPIO_DAT_MISO1_OFF,
	MT6877_AFE_GPIO_DAT_MISO1_ON,
	MT6877_AFE_GPIO_DAT_MISO2_OFF,
	MT6877_AFE_GPIO_DAT_MISO2_ON,
	MT6877_AFE_GPIO_DAT_MOSI_OFF,
	MT6877_AFE_GPIO_DAT_MOSI_ON,
	MT6877_AFE_GPIO_DAT_MOSI_CH34_OFF,
	MT6877_AFE_GPIO_DAT_MOSI_CH34_ON,
	MT6877_AFE_GPIO_I2S0_OFF,
	MT6877_AFE_GPIO_I2S0_ON,
	MT6877_AFE_GPIO_I2S1_OFF,
	MT6877_AFE_GPIO_I2S1_ON,
	MT6877_AFE_GPIO_I2S2_OFF,
	MT6877_AFE_GPIO_I2S2_ON,
	MT6877_AFE_GPIO_I2S3_OFF,
	MT6877_AFE_GPIO_I2S3_ON,
	MT6877_AFE_GPIO_I2S5_OFF,
	MT6877_AFE_GPIO_I2S5_ON,
	MT6877_AFE_GPIO_I2S6_OFF,
	MT6877_AFE_GPIO_I2S6_ON,
	MT6877_AFE_GPIO_I2S7_OFF,
	MT6877_AFE_GPIO_I2S7_ON,
	MT6877_AFE_GPIO_I2S8_OFF,
	MT6877_AFE_GPIO_I2S8_ON,
	MT6877_AFE_GPIO_I2S9_OFF,
	MT6877_AFE_GPIO_I2S9_ON,
	MT6877_AFE_GPIO_VOW_OFF,
	MT6877_AFE_GPIO_VOW_ON,
	MT6877_AFE_GPIO_GPIO_NUM
};

struct mtk_base_afe;

int mt6877_afe_gpio_init(struct mtk_base_afe *afe);
int mt6877_afe_gpio_request(struct mtk_base_afe *afe, bool enable,
			    int dai, int uplink);
bool mt6877_afe_gpio_is_prepared(enum mt6877_afe_gpio type);

#endif
