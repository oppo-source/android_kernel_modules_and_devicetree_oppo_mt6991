// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2015 MediaTek Inc.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/kfifo.h>

#include <linux/firmware.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#if IS_ENABLED(CONFIG_OF)
#include <linux/of_fdt.h>
#endif
#include <asm/setup.h>
#include <linux/atomic.h>

#include "ccci_util_lib_main.h"

static int __init ccci_util_init(void)
{
	ccci_log_init();
	mtk_ccci_args_key_val_init();
	ccci_util_fo_init();
	mtk_ccci_compatible_md_chk_hdr_parsing();
#ifdef CCCI_KMODULE_ENABLE
	mtk_ccci_dump_args_info();
#endif
	ccci_common_sysfs_init();
	ccci_util_broadcast_init();
	ccci_sib_init();
	ccci_util_pin_broadcast_init();

	return 0;
}

subsys_initcall(ccci_util_init);
MODULE_DESCRIPTION("MTK CCCI UTIL Driver");
MODULE_LICENSE("GPL");
