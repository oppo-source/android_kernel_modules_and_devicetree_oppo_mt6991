/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

/*[File]            : ap2bgf_conn_infra_on_ccif4.h*/
/*[Revision time]   : Thu Jul 29 21:09:08 2021*/
/*[Description]     : This file is auto generated by CODA*/
/*[Copyright]       : Copyright (C) 2021 Mediatek Incorportion. All rights reserved.*/

#ifndef __AP2BGF_CONN_INFRA_ON_CCIF4_REGS_H__
#define __AP2BGF_CONN_INFRA_ON_CCIF4_REGS_H__

#include "hal_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************/
/**/
/*                     AP2BGF_CONN_INFRA_ON_CCIF4 CR Definitions                     */
/**/
/******************************************************************************/

#define AP2BGF_CONN_INFRA_ON_CCIF4_BASE (0x4003D000 + CONN_INFRA_REMAPPING_OFFSET)

#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_BUSY_ADDR		(AP2BGF_CONN_INFRA_ON_CCIF4_BASE + 0x0004)
#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_START_ADDR		(AP2BGF_CONN_INFRA_ON_CCIF4_BASE + 0x0008)
#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_TCHNUM_ADDR		(AP2BGF_CONN_INFRA_ON_CCIF4_BASE + 0x000C)
#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_RCHNUM_ADDR		(AP2BGF_CONN_INFRA_ON_CCIF4_BASE + 0x0010)
#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_ACK_ADDR		(AP2BGF_CONN_INFRA_ON_CCIF4_BASE + 0x0014)
#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_IRQ0_MASK_ADDR	(AP2BGF_CONN_INFRA_ON_CCIF4_BASE + 0x0018)
#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_IRQ1_MASK_ADDR	(AP2BGF_CONN_INFRA_ON_CCIF4_BASE + 0x001C)
#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_DUMMY1_ADDR		(AP2BGF_CONN_INFRA_ON_CCIF4_BASE + 0x0020)
#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_DUMMY2_ADDR		(AP2BGF_CONN_INFRA_ON_CCIF4_BASE + 0x0024)
#define AP2BGF_CONN_INFRA_ON_CCIF4_BGF2AP_SHADOW1_ADDR			(AP2BGF_CONN_INFRA_ON_CCIF4_BASE + 0x0030)
#define AP2BGF_CONN_INFRA_ON_CCIF4_BGF2AP_SHADOW2_ADDR			(AP2BGF_CONN_INFRA_ON_CCIF4_BASE + 0x0034)


#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_BUSY_BUSY_ADDR AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_BUSY_ADDR
#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_BUSY_BUSY_MASK 0x000000FF
#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_BUSY_BUSY_SHFT 0


#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_START_START_ADDR AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_START_ADDR
#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_START_START_MASK 0x000000FF
#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_START_START_SHFT 0


#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_TCHNUM_TCHNUM_ADDR AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_TCHNUM_ADDR
#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_TCHNUM_TCHNUM_MASK 0x00000007
#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_TCHNUM_TCHNUM_SHFT 0


#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_RCHNUM_RCHNUM_ADDR AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_RCHNUM_ADDR
#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_RCHNUM_RCHNUM_MASK 0x000000FF
#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_RCHNUM_RCHNUM_SHFT 0


#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_ACK_ACK_ADDR AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_ACK_ADDR
#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_ACK_ACK_MASK 0x000000FF
#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_ACK_ACK_SHFT 0


#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_IRQ0_MASK_IRQ0_MASK_ADDR \
	AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_IRQ0_MASK_ADDR
#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_IRQ0_MASK_IRQ0_MASK_MASK 0x000000FF
#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_IRQ0_MASK_IRQ0_MASK_SHFT 0


#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_IRQ1_MASK_IRQ1_MASK_ADDR \
	AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_IRQ1_MASK_ADDR
#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_IRQ1_MASK_IRQ1_MASK_MASK 0x000000FF
#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_IRQ1_MASK_IRQ1_MASK_SHFT 0


#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_DUMMY1_AP2BGF_PCCIF_DUMMY1_ADDR \
	AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_DUMMY1_ADDR
#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_DUMMY1_AP2BGF_PCCIF_DUMMY1_MASK 0xFFFFFFFF
#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_DUMMY1_AP2BGF_PCCIF_DUMMY1_SHFT 0


#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_DUMMY2_AP2BGF_PCCIF_DUMMY2_ADDR \
	AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_DUMMY2_ADDR
#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_DUMMY2_AP2BGF_PCCIF_DUMMY2_MASK 0xFFFFFFFF
#define AP2BGF_CONN_INFRA_ON_CCIF4_AP2BGF_PCCIF_DUMMY2_AP2BGF_PCCIF_DUMMY2_SHFT 0


#define AP2BGF_CONN_INFRA_ON_CCIF4_BGF2AP_SHADOW1_BGF2AP_SHADOW1_ADDR AP2BGF_CONN_INFRA_ON_CCIF4_BGF2AP_SHADOW1_ADDR
#define AP2BGF_CONN_INFRA_ON_CCIF4_BGF2AP_SHADOW1_BGF2AP_SHADOW1_MASK 0xFFFFFFFF
#define AP2BGF_CONN_INFRA_ON_CCIF4_BGF2AP_SHADOW1_BGF2AP_SHADOW1_SHFT 0


#define AP2BGF_CONN_INFRA_ON_CCIF4_BGF2AP_SHADOW2_BGF2AP_SHADOW2_ADDR AP2BGF_CONN_INFRA_ON_CCIF4_BGF2AP_SHADOW2_ADDR
#define AP2BGF_CONN_INFRA_ON_CCIF4_BGF2AP_SHADOW2_BGF2AP_SHADOW2_MASK 0xFFFFFFFF
#define AP2BGF_CONN_INFRA_ON_CCIF4_BGF2AP_SHADOW2_BGF2AP_SHADOW2_SHFT 0

#ifdef __cplusplus
}
#endif

#endif /* __AP2BGF_CONN_INFRA_ON_CCIF4_REGS_H__*/
