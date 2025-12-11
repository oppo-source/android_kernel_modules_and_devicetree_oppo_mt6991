// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025 Oplus. All rights reserved.
 */

#ifndef __SMART_FREQ_H__
#define __SMART_FREQ_H__


#include <linux/module.h>
#include <linux/percpu.h>
#include <linux/sched.h>
#include <linux/cpumask.h>
#include <linux/pm_qos.h>
#include <linux/sched/cputime.h>
#include <linux/jump_label.h>
#include <linux/cgroup.h>
#include <uapi/linux/sched/types.h>
#include <linux/cpuidle.h>
#include <linux/sched/clock.h>
#include <trace/hooks/cgroup.h>
#include <linux/arch_topology.h>
#include <trace/hooks/sched.h>

enum smart_freq_ipc_reason {
	IPC_A,
	IPC_B,
	IPC_C,
	IPC_D,
	IPC_E,
	SMART_FMAX_IPC_MAX,
};

#define IPC_PARTICIPATION	(BIT(IPC_A) | BIT(IPC_B) | BIT(IPC_C) | BIT(IPC_D) | BIT(IPC_E))

struct smart_freq_ipc_reason_config {
	unsigned long ipc;
	unsigned long freq_allowed;
	unsigned long hyst_ns;
};

struct smart_freq_cluster_info {
	cpumask_t cpu_mask;
	unsigned int cluster_id;
	unsigned int cluster_ipc_level;
	unsigned int min_cycles;
	unsigned int smart_freq_ipc_participation_mask;
	unsigned int cluster_freq;
	struct smart_freq_ipc_reason_config ipc_reason_config[SMART_FMAX_IPC_MAX];
};

void smart_freq_update(unsigned int cpu, u64 time, unsigned int flags);
unsigned int smart_freq_update_final_freq(struct cpumask *cpumask, unsigned int freq);

#endif /* __SMART_FREQ_H__ */
