/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2023 MediaTek Inc.
 */

#ifndef _PLATFORM_MT6991_CONSYS_REG_H_
#define _PLATFORM_MT6991_CONSYS_REG_H_

#include "../../include/consys_reg_base.h"
#include "../../include/consys_reg_mng.h"

enum connsys_base_addr_index {
	CONN_RGU_ON_BASE_INDEX              = 0,    /* 0x4000_0000 */
	CONN_CFG_ON_BASE_INDEX              = 1,    /* 0x4000_1000 */
	CONN_WT_SLP_CTL_REG_BASE_INDEX      = 2,    /* 0x4000_3000 */
	CONN_BUS_CR_ON_BASE_INDEX           = 3,    /* 0x4000_e000 */
	CONN_CFG_BASE_INDEX                 = 4,    /* 0x4001_1000 */
	CONN_CLKGEN_TOP_BASE_INDEX          = 5,    /* 0x4001_2000 */
	CONN_INFRA_DBG_CTL_BASE_INDEX       = 6,    /* 0x4002_3000 */
	CONN_AFE_CTL_BASE_INDEX             = 7,    /* 0x4004_1000 */
	CONN_RF_SPI_MST_REG_BASE_INDEX      = 8,    /* 0x4004_2000 */
	CONN_BUS_CR_BASE_INDEX              = 9,    /* 0x4004_b000 */
	CONN_OFF_DEBUG_CTRL_AO_BASE_INDEX   = 10,   /* 0x4004_d000 */
	CONN_INFRA_SYSRAM_INDEX             = 11,   /* 0x4005_0000 */
	CONN_HOST_CSR_TOP_BASE_INDEX        = 12,   /* 0x4006_0000 */
	CONN_SEMAPHORE_BASE_INDEX           = 13,   /* 0x4007_0000 */
	INFRABUS_AO_REG_INDEX               = 14,   /* 0x1002_c000 */
	SPM_INDEX                           = 15,   /* 0x1c00_4000 */
	VLPSYS_SRCLKENRC                    = 16,   /* 0x1c01_1000 */
	CONSYS_BASE_ADDR_MAX
};

struct consys_base_addr {
	struct consys_reg_base_addr reg_base_addr[CONSYS_BASE_ADDR_MAX];
};

extern struct consys_base_addr g_conn_reg_mt6991;

#define CONN_RGU_ON_BASE_ADDR_MT6991            g_conn_reg_mt6991.reg_base_addr[CONN_RGU_ON_BASE_INDEX].vir_addr
#define CONN_CFG_ON_BASE_ADDR_MT6991            g_conn_reg_mt6991.reg_base_addr[CONN_CFG_ON_BASE_INDEX].vir_addr
#define CONN_WT_SLP_CTL_REG_BASE_ADDR_MT6991    g_conn_reg_mt6991.reg_base_addr[CONN_WT_SLP_CTL_REG_BASE_INDEX].vir_addr
#define CONN_BUS_CR_ON_BASE_ADDR_MT6991         g_conn_reg_mt6991.reg_base_addr[CONN_BUS_CR_ON_BASE_INDEX].vir_addr
#define CONN_CFG_BASE_ADDR_MT6991               g_conn_reg_mt6991.reg_base_addr[CONN_CFG_BASE_INDEX].vir_addr
#define CONN_CLKGEN_TOP_BASE_ADDR_MT6991        g_conn_reg_mt6991.reg_base_addr[CONN_CLKGEN_TOP_BASE_INDEX].vir_addr
#define CONN_REG_CONN_INFRA_DBG_CTL_ADDR_MT6991 g_conn_reg_mt6991.reg_base_addr[CONN_INFRA_DBG_CTL_BASE_INDEX].vir_addr
#define CONN_AFE_CTL_BASE_ADDR_MT6991           g_conn_reg_mt6991.reg_base_addr[CONN_AFE_CTL_BASE_INDEX].vir_addr
#define CONN_RF_SPI_MST_REG_BASE_ADDR_MT6991    g_conn_reg_mt6991.reg_base_addr[CONN_RF_SPI_MST_REG_BASE_INDEX].vir_addr
#define CONN_BUS_CR_BASE_ADDR_MT6991            g_conn_reg_mt6991.reg_base_addr[CONN_BUS_CR_BASE_INDEX].vir_addr
#define CONN_OFF_DEBUG_CTRL_AO_BASE_ADDR_MT6991 g_conn_reg_mt6991.reg_base_addr[CONN_OFF_DEBUG_CTRL_AO_BASE_INDEX].vir_addr
#define CONN_INFRA_SYSRAM_BASE_ADDR_MT6991      g_conn_reg_mt6991.reg_base_addr[CONN_INFRA_SYSRAM_INDEX].vir_addr
#define CONN_HOST_CSR_TOP_BASE_ADDR_MT6991      g_conn_reg_mt6991.reg_base_addr[CONN_HOST_CSR_TOP_BASE_INDEX].vir_addr
#define CONN_SEMAPHORE_BASE_ADDR_MT6991         g_conn_reg_mt6991.reg_base_addr[CONN_SEMAPHORE_BASE_INDEX].vir_addr
#define INFRABUS_AO_REG_BASE_ADDR_MT6991        g_conn_reg_mt6991.reg_base_addr[INFRABUS_AO_REG_INDEX].vir_addr
#define SPM_BASE_ADDR_MT6991                    g_conn_reg_mt6991.reg_base_addr[SPM_INDEX].vir_addr
#define VLPSYS_SRCLKENRC_MT6991                 g_conn_reg_mt6991.reg_base_addr[VLPSYS_SRCLKENRC].vir_addr

int consys_reg_init_mt6991(struct platform_device *pdev);
int consys_reg_deinit_mt6991(void);
int consys_check_reg_readable_mt6991(void);
int consys_check_reg_readable_for_coredump_mt6991(void);
int consys_is_consys_reg_mt6991(unsigned int addr);
int consys_is_bus_hang_mt6991(void);
void consys_debug_init_mt6991(void);
void consys_debug_deinit_mt6991(void);
int consys_check_ap2conn_infra_on_mt6991(void);
int consys_check_ap2conn_infra_off_clock_mt6991(void);
int consys_check_ap2conn_infra_off_irq_mt6991(void);
int consys_print_debug_mt6991(enum conninfra_bus_error_type level);
int consys_debug_top_power_status_mt6991(void);

#endif /* _PLATFORM_MT6991_CONSYS_REG_H_ */
