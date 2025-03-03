/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __CONAP_SCP_PLATFORM_DATA_H__
#define __CONAP_SCP_PLATFORM_DATA_H__

#include <linux/of.h>
#include <linux/platform_device.h>

struct conap_scp_shm_config {
	uint32_t conap_scp_shm_offset;
	uint32_t conap_scp_shm_size;
	uint32_t conap_scp_ipi_mbox_size;
	uint32_t conap_scp_max_msg_size;
};

struct conap_scp_batching_config {
	uint32_t buff_offset;
	uint32_t buff_size;
};

struct conap_dfd_config {
	phys_addr_t addr;
	uint32_t size;
};

int connsys_scp_plt_data_init(struct platform_device *pdev);

struct conap_scp_shm_config *conap_scp_get_shm_info(void);

/* connsys shared buffer */
uint32_t connsys_scp_shm_get_addr(void);
uint32_t connsys_scp_shm_get_size(void);

uint32_t connsys_scp_get_max_msg_size(void);

/* flp/batching shared buffer */
phys_addr_t connsys_scp_shm_get_batching_addr(void);
uint32_t connsys_scp_shm_get_batching_size(void);

uint32_t connsys_scp_ipi_mbox_size(void);

/* dfd support */
phys_addr_t connsys_scp_get_dfd_cmd_addr(void);
uint32_t connsys_scp_get_dfd_cmd_size(void);
phys_addr_t connsys_scp_get_dfd_value_addr(void);
uint32_t connsys_scp_get_dfd_value_size(void);

#endif
