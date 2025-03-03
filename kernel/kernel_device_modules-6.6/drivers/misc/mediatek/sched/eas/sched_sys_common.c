// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/cpumask.h>
#include <linux/seq_file.h>
#include <linux/energy_model.h>
#include <linux/topology.h>
#include <linux/sched/topology.h>
#include <trace/events/sched.h>
#include <trace/hooks/sched.h>
#include <linux/cpu.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/sched/cputime.h>
#include <sched/sched.h>
#include "sched_sys_common.h"

#define CREATE_TRACE_POINTS

static struct attribute *sched_ctl_attrs[] = {
#if IS_ENABLED(CONFIG_MTK_CORE_PAUSE)
	&sched_core_pause_info_attr.attr,
#endif
    &fake_cpuinfo_max_freq_cpu_attr.attr,
	NULL,
};

static struct attribute_group sched_ctl_attr_group = {
	.attrs = sched_ctl_attrs,
};

static struct kobject *kobj;
int init_sched_common_sysfs(void)
{
	struct device *dev_root = bus_get_dev_root(&cpu_subsys);
	int ret = 0;

	if (dev_root) {
		kobj = kobject_create_and_add("sched_ctl", &dev_root->kobj);
		if (!kobj) {
			pr_info("sched_ctl folder create failed\n");
			return -ENOMEM;
		}
		put_device(dev_root);
	}
	ret = sysfs_create_group(kobj, &sched_ctl_attr_group);
	if (ret)
		goto error;
	kobject_uevent(kobj, KOBJ_ADD);

#if IS_ENABLED(CONFIG_MTK_SCHED_BIG_TASK_ROTATE)
	task_rotate_init();
#endif

	return 0;

error:
	kobject_put(kobj);
	kobj = NULL;
	return ret;
}

extern unsigned int fake_cpuinfo_max_freq_cpu;
extern struct mutex fake_cpuinfo_max_freq_cpu_mutex;

ssize_t store_fake_cpuinfo_max_freq_cpu(struct kobject *kobj, struct kobj_attribute *attr, const char __user *buf, size_t cnt)
{
    int value;

    if (kstrtoint(buf, 10, &value))
        return -EINVAL;

    mutex_lock(&fake_cpuinfo_max_freq_cpu_mutex);
    fake_cpuinfo_max_freq_cpu = value;
    mutex_unlock(&fake_cpuinfo_max_freq_cpu_mutex);

    return cnt;
}

ssize_t show_fake_cpuinfo_max_freq_cpu(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    unsigned int len = 0;
    unsigned int max_len = 4096;
    unsigned int temp = -1;

    mutex_lock(&fake_cpuinfo_max_freq_cpu_mutex);
    temp = fake_cpuinfo_max_freq_cpu;
    mutex_unlock(&fake_cpuinfo_max_freq_cpu_mutex);

    len += snprintf(buf+len, max_len-len, "%d\n", temp);

    return len;
}

void cleanup_sched_common_sysfs(void)
{
	if (kobj) {
		sysfs_remove_group(kobj, &sched_ctl_attr_group);
		kobject_put(kobj);
		kobj = NULL;
	}
}

struct kobj_attribute fake_cpuinfo_max_freq_cpu_attr =
__ATTR(fake_cpuinfo_max_freq_cpu, 0644, show_fake_cpuinfo_max_freq_cpu, store_fake_cpuinfo_max_freq_cpu);