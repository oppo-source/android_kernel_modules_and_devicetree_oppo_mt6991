// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
/***************************************************************
** File : dump_reason.c
** Description : dump reason feature
** Version : 1.0
******************************************************************/

#include <linux/stacktrace.h>
#include <linux/string.h>
#include <linux/kallsyms.h>
#include <linux/soc/qcom/smem.h>

#include <linux/notifier.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
#include <linux/panic_notifier.h>
#endif

#include "../dump_device_info/device_info.h"
#include "dump_reason.h"
#include <linux/slab.h>
#include <linux/kernel.h>

#define MAX_STACK_DEPTH 13
#define MAX_SYMBOL_LEN	128
#define MAX_PIDBUFFER_LEN	64
#define IGNORE_STACK_DEPTH	3
#define NULL_STACK_SIZE	3

static char caller_function_name[KSYM_SYMBOL_LEN];
static char pidbuffer[MAX_PIDBUFFER_LEN];
static struct dump_info *dp_info;

unsigned long entries[MAX_STACK_DEPTH];
char *entries1[MAX_STACK_DEPTH];

void dump_save_stack_trace(void)
{
	int n = 0;
	unsigned int i;
	printk(KERN_ERR "dump-reason-buffer-size: %d\n", DUMP_REASON_SIZE);
	scnprintf(pidbuffer, MAX_PIDBUFFER_LEN, "PID: %d, Process Name: %-20s", current->pid, current->comm);
	if ((strlen(dp_info->dump_reason) + strlen(pidbuffer) + strlen("\r\n") + strlen("Call stack:")) < DUMP_REASON_SIZE - 1) {
		strncat(dp_info->dump_reason, "\r\n", sizeof("\r\n") - 1);
		strncat(dp_info->dump_reason, pidbuffer, strlen(pidbuffer));
		strncat(dp_info->dump_reason, "\r\n", sizeof("\r\n") - 1);
		strncat(dp_info->dump_reason, "Call stack:", strlen("Call stack:"));
		printk(KERN_ERR "dump-reason-pidbuffer:%s", pidbuffer);
	}
	n = stack_trace_save(entries, ARRAY_SIZE(entries), 1);
	if (n <= 0) {
		printk(KERN_ERR "Stack trace save failed: %d\n", n);
		return;
	}
	for (i = IGNORE_STACK_DEPTH; i < MAX_STACK_DEPTH; i++) {
		if (!entries1[i])
			break;
		scnprintf(entries1[i], MAX_SYMBOL_LEN, "%pS", (void *)entries[i]);
		if (strlen(entries1[i]) > NULL_STACK_SIZE && (strlen(dp_info->dump_reason) + strlen(entries1[i]) + strlen("\r\n")) < DUMP_REASON_SIZE - 1) {
			strncat(dp_info->dump_reason, "\r\n", sizeof("\r\n") - 1);
			strncat(dp_info->dump_reason, entries1[i], strlen(entries1[i]));
		}
		kfree(entries1[i]);
		}
}

char *parse_function_builtin_return_address(unsigned long function_address)
{
	char *cur = caller_function_name;

	if (!function_address)
		return NULL;

	sprint_symbol(caller_function_name, function_address);
	strsep(&cur, "+");
	return caller_function_name;
}
EXPORT_SYMBOL(parse_function_builtin_return_address);

void save_dump_reason_to_smem(char *info, char *function_name)
{
	int strlinfo = 0, strlfun = 0;
	size_t size;
	static int flag = 0;

	/* Make sure save_dump_reason_to_smem() is not
	called infinite times by nested panic caller fns etc*/
	if (flag >= 1) {
		pr_debug("%s: already save dump info \n", __func__);
		return;
	}
	flag++;
	dp_info = qcom_smem_get(QCOM_SMEM_HOST_ANY, SMEM_DUMP_INFO, &size);

	if (IS_ERR_OR_NULL(dp_info)) {
		pr_debug("%s: get dp_info failure\n", __func__);
		return;
	}
	else {
		pr_debug("%s: info : %s\n", __func__, info);

		strlinfo = strlen(info)+1;
		strlfun  = strlen(function_name)+1;
		strlinfo = strlinfo  <  DUMP_REASON_SIZE ? strlinfo : DUMP_REASON_SIZE;
		strlfun  = strlfun <  DUMP_REASON_SIZE ? strlfun: DUMP_REASON_SIZE;
		if ((strlen(dp_info->dump_reason) + strlinfo) < DUMP_REASON_SIZE)
			strncat(dp_info->dump_reason, info, strlinfo);

		if (function_name != NULL &&
			((strlen(dp_info->dump_reason) + strlfun + sizeof("\r\n")+1) < DUMP_REASON_SIZE)) {
			strncat(dp_info->dump_reason, "\r\n", sizeof("\r\n"));
			strncat(dp_info->dump_reason, function_name, strlfun);
		}

		pr_debug("\r%s: dump_reason : %s strl=%d function caused panic :%s strl1=%d \n", __func__,
				dp_info->dump_reason, strlinfo, function_name, strlfun);
        dump_save_stack_trace();
		save_dump_reason_to_device_info(dp_info->dump_reason);
		flag++;
	}
}

EXPORT_SYMBOL(save_dump_reason_to_smem);

void dump_reason_init_smem(void)
{
	int ret;

	ret = qcom_smem_alloc(QCOM_SMEM_HOST_ANY, SMEM_DUMP_INFO,
		sizeof(struct dump_info));

	if (ret < 0 && ret != -EEXIST) {
		pr_err("%s:unable to allocate dp_info \n", __func__);
		return;
	}
}

void dump_reason_init_callstack(void)
{
	unsigned int i;
	for (i = IGNORE_STACK_DEPTH; i < MAX_STACK_DEPTH; i++) {
		entries1[i] = kmalloc(MAX_SYMBOL_LEN, GFP_KERNEL);
		if (!entries1[i]) {
			printk(KERN_ERR "dump call stack Memory allocation error i=%d\n", i);
			break;
		}
	}
}

static int panic_save_dump_reason(struct notifier_block *this, unsigned long event, void *buf)
{
	char *func_name;
	func_name = parse_function_builtin_return_address(
		(unsigned long)__builtin_return_address(1));

	if (func_name) {
		save_dump_reason_to_smem(buf, func_name);
	}

	return NOTIFY_DONE;
}

static struct notifier_block panic_block = {
	.notifier_call = panic_save_dump_reason,
	.priority = INT_MAX - 1,
};

static int __init dump_reason_init(void)
{
	dump_reason_init_smem();
	atomic_notifier_chain_register(&panic_notifier_list, &panic_block);
	dump_reason_init_callstack();
	return 0;
}

module_init(dump_reason_init);
MODULE_LICENSE("GPL v2");
