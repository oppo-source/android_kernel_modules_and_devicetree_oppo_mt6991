// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023 MediaTek Inc.
 */

#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/types.h>

#include "conninfra.h"
#include "consys_reg_mng.h"
#include "consys_reg_util.h"

#include "include/mt6991.h"
#include "include/mt6991_consys_reg.h"
#include "include/mt6991_consys_reg_offset.h"

unsigned int consys_get_hw_ver_mt6991(void)
{
	return CONN_HW_VER_MT6991;
}

unsigned int consys_get_adie_chipid_mt6991(unsigned int drv_type)
{
	return CONNSYS_A_DIE_ID_MT6991;
}

static int consys_reset_power_state(void)
{
	/* Clear data and disable stop */
	/* I. Clear
	 * i.	Conn_infra:	0x4006_0384[15]
	 * ii.	Wf:		0x4006_0384[9]
	 * iii.	Bt:		0x4006_0384[10]
	 * iv.	Gps:		0x4006_0384[8]
	 */
	CONSYS_REG_WRITE_HW_ENTRY(
		CONN_HOST_CSR_TOP_HOST_CONN_INFRA_SLP_CNT_SLP_STOP_HOST_CR_CONN_INFRA_SLEEP_CNT_CLR,
		0x1);
	udelay(150);
	CONSYS_REG_WRITE_HW_ENTRY(
		CONN_HOST_CSR_TOP_HOST_CONN_INFRA_SLP_CNT_SLP_STOP_HOST_CR_CONN_INFRA_SLEEP_CNT_CLR,
		0x0);

	CONSYS_REG_WRITE_HW_ENTRY(
		CONN_HOST_CSR_TOP_HOST_CONN_INFRA_SLP_CNT_SLP_STOP_HOST_CR_GPS_SLEEP_CNT_CLR,
		0x1);
	udelay(150);
	CONSYS_REG_WRITE_HW_ENTRY(
		CONN_HOST_CSR_TOP_HOST_CONN_INFRA_SLP_CNT_SLP_STOP_HOST_CR_GPS_SLEEP_CNT_CLR,
		0x0);
	/* II. Stop
	 * i.	Conn_infra:	0x4006_0384[7]
	 * ii.	Wf:		0x4006_0384[1]
	 * iii.	Bt:		0x4006_0384[2]
	 * iv.	Gps:		0x4006_0384[0]
	 */
	CONSYS_REG_WRITE_HW_ENTRY(
		CONN_HOST_CSR_TOP_HOST_CONN_INFRA_SLP_CNT_SLP_STOP_HOST_CR_CONN_INFRA_SLEEP_CNT_STOP,
		0x0);
	CONSYS_REG_WRITE_HW_ENTRY(
		CONN_HOST_CSR_TOP_HOST_CONN_INFRA_SLP_CNT_SLP_STOP_HOST_CR_GPS_SLEEP_CNT_STOP,
		0x0);
	return 0;
}

static inline void __sleep_count_trigger_read(void)
{
	CONSYS_REG_WRITE_HW_ENTRY(
		CONN_HOST_CSR_TOP_HOST_CONN_INFRA_SLP_CNT_CTL_HOST_SLP_COUNTER_RD_TRIGGER, 0x1);
	udelay(150);
	CONSYS_REG_WRITE_HW_ENTRY(
		CONN_HOST_CSR_TOP_HOST_CONN_INFRA_SLP_CNT_CTL_HOST_SLP_COUNTER_RD_TRIGGER, 0x0);
}

#define POWER_STATE_BUFF_LENGTH	256
static void consys_power_state(void)
{
	unsigned int i, str_len;
	unsigned int buf_len = 0;
	unsigned int r;
	static const char *const osc_str[] = {
	"fm ", "gps ", "bgf ", "wf ", "conn_infra_bus ", " ", "ap2conn ", " ",
	" ", " ", " ", "conn_pta ", "conn_spi ", " ", "conn_thm "};
	char buf[POWER_STATE_BUFF_LENGTH] = {'\0'};

	CONSYS_REG_WRITE_HW_ENTRY(
		CONN_HOST_CSR_TOP_CR_CONN_INFRA_CFG_ON_DBG_MUX_SEL_CR_CONN_INFRA_CFG_ON_DBG_MUX_SEL,
		0x0);
	r = CONSYS_REG_READ(CONN_HOST_CSR_TOP_CONN_INFRA_CFG_ON_DBG_ADDR);

	for (i = 0; i < 15; i++) {
		str_len = strlen(osc_str[i]);

		if ((r & (0x1 << (1 + i))) > 0 && (buf_len + str_len < POWER_STATE_BUFF_LENGTH)) {
			strnlcat(buf, osc_str[i], str_len, POWER_STATE_BUFF_LENGTH);
			buf_len += str_len;
		}
	}
	pr_notice("[%s] [0x%x] %s\n", __func__, r, buf);

	consys_debug_top_power_status_mt6991();
}

static int consys_power_state_dump(char *buf, unsigned int size, int print_log)
{
#define CONN_32K_TICKS_PER_SEC (32768)
#define CONN_TICK_TO_SEC(TICK) (TICK / CONN_32K_TICKS_PER_SEC)
#define POWER_STATE_BUF_SIZE 256
	static u64 round;
	static u64 t_conninfra_sleep_cnt, t_conninfra_sleep_time;
	static u64 t_gps_sleep_cnt, t_gps_sleep_time;
	unsigned int conninfra_sleep_cnt, conninfra_sleep_time;
	unsigned int gps_sleep_cnt, gps_sleep_time;
	char temp_buf[POWER_STATE_BUF_SIZE];
	char *buf_p = temp_buf;
	int buf_sz = POWER_STATE_BUF_SIZE;
	int ret = 0;

	/* Sleep count */
	/* 1. Setup read select: 0x4006_0380[3:1]
	 *	3'h0: conn_infra sleep counter
	 *	3'h1: wfsys sleep counter
	 *	3'h2: bgfsys sleep counter
	 *	3'h3: gpssys sleep counter
	 * 2. Dump time and count
	 *	a. Timer: 0x4006_0388
	 *	b. Count: 0x4006_038C
	 */
	CONSYS_REG_WRITE_HW_ENTRY(
		CONN_HOST_CSR_TOP_HOST_CONN_INFRA_SLP_CNT_CTL_HOST_SLP_COUNTER_SEL,
		0x0);
	__sleep_count_trigger_read();
	conninfra_sleep_time = CONSYS_REG_READ(
		CONN_HOST_CSR_TOP_HOST_CONN_INFRA_SLP_TIMER_ADDR);
	conninfra_sleep_cnt = CONSYS_REG_READ(
		CONN_HOST_CSR_TOP_HOST_CONN_INFRA_SLP_COUNTER_ADDR);

	t_conninfra_sleep_time += conninfra_sleep_time;
	t_conninfra_sleep_cnt += conninfra_sleep_cnt;

	udelay(60);
	CONSYS_REG_WRITE_HW_ENTRY(
		CONN_HOST_CSR_TOP_HOST_CONN_INFRA_SLP_CNT_CTL_HOST_SLP_COUNTER_SEL,
		0x3);
	__sleep_count_trigger_read();
	gps_sleep_time = CONSYS_REG_READ(
		CONN_HOST_CSR_TOP_HOST_CONN_INFRA_SLP_TIMER_ADDR);
	gps_sleep_cnt = CONSYS_REG_READ(
		CONN_HOST_CSR_TOP_HOST_CONN_INFRA_SLP_COUNTER_ADDR);
	t_gps_sleep_time += gps_sleep_time;
	t_gps_sleep_cnt += gps_sleep_cnt;

	if (print_log) {
		if (buf != NULL && size > 0) {
			buf_p = buf;
			buf_sz = size;
		}
		ret = snprintf(buf_p, buf_sz,
			"[consys_power_state][round:%llu]conninfra:%u.%03u,%u;gps:%u.%03u,%u;[total]conninfra:%llu.%03llu,%llu;gps:%llu.%03llu,%llu;",
			round,
			CONN_TICK_TO_SEC(conninfra_sleep_time),
			CONN_TICK_TO_SEC((conninfra_sleep_time % CONN_32K_TICKS_PER_SEC) * 1000),
			conninfra_sleep_cnt,
			CONN_TICK_TO_SEC(gps_sleep_time),
			CONN_TICK_TO_SEC((gps_sleep_time % CONN_32K_TICKS_PER_SEC) * 1000),
			gps_sleep_cnt,
			CONN_TICK_TO_SEC(t_conninfra_sleep_time),
			CONN_TICK_TO_SEC((t_conninfra_sleep_time % CONN_32K_TICKS_PER_SEC) * 1000),
			t_conninfra_sleep_cnt,
			CONN_TICK_TO_SEC(t_gps_sleep_time),
			CONN_TICK_TO_SEC((t_gps_sleep_time % CONN_32K_TICKS_PER_SEC) * 1000),
			t_gps_sleep_cnt);
		if (ret > 0)
			pr_notice("%s\n", buf_p);
		/* Power state */
		consys_power_state();
	}
	round++;
	/* reset after sleep time is accumulated. */
	consys_reset_power_state();
	return 0;
}

int consys_reset_power_state_mt6991(void)
{
	return consys_power_state_dump(NULL, 0, 0);
}

int consys_power_state_dump_mt6991(char *buf, unsigned int size)
{
	return consys_power_state_dump(buf, size, 1);
}

int consys_enable_power_dump_mt6991(void)
{
	/* Return success because sleep count dump is enable on POS */
	return 0;
}

void consys_set_mcu_control_mt6991(int type, bool onoff)
{
	pr_info("[%s] Set mcu control type=[%d] onoff=[%d]\n", __func__, type, onoff);

	if (onoff) /* Turn on */
		CONSYS_SET_BIT(CONN_INFRA_SYSRAM_SW_CR_MCU_LOG_CONTROL, (0x1 << type));
	else /* Turn off */
		CONSYS_CLR_BIT(CONN_INFRA_SYSRAM_SW_CR_MCU_LOG_CONTROL, (0x1 << type));
}
