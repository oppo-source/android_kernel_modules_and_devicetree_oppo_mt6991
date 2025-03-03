// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpu.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <linux/regulator/consumer.h>
#include "mtk_hps_internal.h"
#include "mtk_hps.h"
/*
 * static
 */
#define STATIC
/* #define STATIC static */
static int hps_probe(struct platform_device *pdev);
static int hps_suspend(struct device *dev);
static int hps_resume(struct device *dev);
static int hps_freeze(struct device *dev);
static int hps_restore(struct device *dev);

struct platform_device hotplug_strategy_pdev = {
	.name = "hps",
	.id = -1,
};

const struct dev_pm_ops hps_dev_pm_ops = {
	.suspend = hps_suspend,
	.resume = hps_resume,
	.freeze = hps_freeze,
	.restore = hps_restore,
	.thaw = hps_restore,
};

struct hps_sys_struct hps_sys = {
	.cluster_num = 0,
	.func_num = 0,
	.is_set_root_cluster = 0,
	.root_cluster_id = 0,
	.total_online_cores = 0,
	.tlp_avg = 0,
	.rush_cnt = 0,
	.up_load_avg = 0,
	.down_load_avg = 0,
	.action_id = 0,
};

struct hps_ctxt_struct hps_ctxt = {
	/* state */
	.init_state = INIT_STATE_NOT_READY,
	.state = STATE_LATE_RESUME,

	.is_interrupt = 0,
	.hps_hrt_ktime = 0,
	/* enabled */
	.enabled = 1,
	.eas_enabled = 1,
	.suspend_enabled = 1,
	.cur_dump_enabled = 0,
	.stats_dump_enabled = 0,
	.idle_det_enabled = 1,
	.is_ppm_init = 1,
	.heavy_task_enabled = 1,
	.big_task_enabled = 1,
	/* core */
	/* Synchronizes accesses to loads statistics */
	.lock = __MUTEX_INITIALIZER(hps_ctxt.lock),
	/* Synchronizes accesses to control break of hps */
	.break_lock = __MUTEX_INITIALIZER(hps_ctxt.break_lock),
	/* Synchronizes accesses to control break of hps */
	.para_lock = __MUTEX_INITIALIZER(hps_ctxt.para_lock),
	.tsk_struct_ptr = NULL,
	.wait_queue = __WAIT_QUEUE_HEAD_INITIALIZER(hps_ctxt.wait_queue),
	/*.periodical_by = HPS_PERIODICAL_BY_WAIT_QUEUE, */
	.periodical_by = HPS_PERIODICAL_BY_HR_TIMER,
	.pdrv = {
		 .remove = NULL,
		 .shutdown = NULL,
		 .probe = hps_probe,
		 .driver = {
			    .name = "hps",
			   .pm = &hps_dev_pm_ops,
			    },
		 },

	/* cpu arch */
	/* unsigned int is_hmp; */
	/* struct cpumask little_cpumask; */
	/* struct cpumask big_cpumask; */
	.little_cpu_id_min = 0,
	.little_cpu_id_max = 3,
	.big_cpu_id_min = 4,
	.big_cpu_id_max = 7,

	/* algo config */
	.up_threshold = DEF_CPU_UP_THRESHOLD,
	.up_times = DEF_CPU_UP_TIMES,
	.down_threshold = DEF_CPU_DOWN_THRESHOLD,
	.down_times = DEF_CPU_DOWN_TIMES,
	.input_boost_enabled = EN_CPU_INPUT_BOOST,
	.input_boost_cpu_num = DEF_CPU_INPUT_BOOST_CPU_NUM,
	.rush_boost_enabled = EN_CPU_RUSH_BOOST,
	.rush_boost_threshold = DEF_CPU_RUSH_BOOST_THRESHOLD,
	.rush_boost_times = DEF_CPU_RUSH_BOOST_TIMES,
	.tlp_times = DEF_TLP_TIMES,
	.up_threshold_H = DEF_CPU_UP_THRESHOLD,
	.down_threshold_H = DEF_CPU_DOWN_THRESHOLD,
	.up_threshold_L = DEF_CPU_UP_THRESHOLD,
	.down_threshold_L = DEF_CPU_DOWN_THRESHOLD,
	.idle_threshold = DEF_CPU_IDLE_THRESHOLD,
	/* algo bound */
	/* .little_num_base_perf_serv = 1, */
	/* .little_num_limit_thermal = NR_CPUS, */
	/* .little_num_limit_low_battery = NR_CPUS, */
	/* .little_num_limit_ultra_power_saving = NR_CPUS, */
	/* .little_num_limit_power_serv = NR_CPUS, */
	/* .big_num_base_perf_serv = 1, */
	/* .big_num_limit_thermal = NR_CPUS, */
	/* .big_num_limit_low_battery = NR_CPUS, */
	/* .big_num_limit_ultra_power_saving = NR_CPUS, */
	/* .big_num_limit_power_serv = NR_CPUS, */

	/* algo statistics */
	.up_loads_sum = 0,
	.up_loads_count = 0,
	.up_loads_history = {0},
	.up_loads_history_index = 0,
	.down_loads_sum = 0,
	.down_loads_count = 0,
	.down_loads_history = {0},
	.down_loads_history_index = 0,
	.rush_count = 0,
	.tlp_sum = 0,
	.tlp_count = 0,
	.tlp_history = {0},
	.tlp_history_index = 0,
	.eas_indicator = 0,
	/* for fast hotplug setting */
	.wake_up_by_fasthotplug = 0,
	.root_cpu = 0,
	/* algo action */
	.action = ACTION_NONE,
	.is_ondemand = ATOMIC_INIT(0),
	.is_break = ATOMIC_INIT(0),

	.test0 = 0,
	.test1 = 0,
};

DEFINE_PER_CPU(struct hps_cpu_ctxt_struct, hps_percpu_ctxt);

/*
 * hps hps_ctxt_t control interface
 */
void hps_ctxt_reset_stas_nolock(void)
{
	hps_ctxt.up_loads_sum = 0;
	hps_ctxt.up_loads_count = 0;
	hps_ctxt.up_loads_history_index = 0;
	hps_ctxt.up_loads_history[hps_ctxt.up_times - 1] = 0;
	/* memset(hps_ctxt.up_loads_history, 0, */
	/*sizeof(hps_ctxt.up_loads_history)); */

	hps_ctxt.down_loads_sum = 0;
	hps_ctxt.down_loads_count = 0;
	hps_ctxt.down_loads_history_index = 0;
	hps_ctxt.down_loads_history[hps_ctxt.down_times - 1] = 0;
	/* memset(hps_ctxt.down_loads_history, 0, */
	/*sizeof(hps_ctxt.down_loads_history)); */

	hps_ctxt.rush_count = 0;
	hps_ctxt.tlp_sum = 0;
	hps_ctxt.tlp_count = 0;
	hps_ctxt.tlp_history_index = 0;
	hps_ctxt.tlp_history[hps_ctxt.tlp_times - 1] = 0;
	/* memset(hps_ctxt.tlp_history, 0, sizeof(hps_ctxt.tlp_history)); */
	switch (hps_ctxt.power_mode) {
	case 1:		/*Low power mode */
		hps_ctxt.down_times = 1;
		hps_ctxt.up_times = 5;
		hps_ctxt.down_threshold_H = 85;
		hps_ctxt.up_threshold_H = 95;
		hps_ctxt.rush_boost_enabled = 0;
		hps_ctxt.heavy_task_enabled = 0;
		break;
	case 2:		/*Just make mode */
	case 3:		/*Performance mode */
		hps_ctxt.down_times = 1;
		hps_ctxt.up_times = 3;
		/*hps_ctxt.down_threshold = 85;*/
		/* hps_ctxt.up_threshold = 95;*/
		hps_ctxt.down_threshold_H = 85;
		hps_ctxt.up_threshold_H = 95;
		hps_ctxt.rush_boost_enabled = 1;
		hps_ctxt.heavy_task_enabled = 1;
		break;
	default:
		break;
	}
}

void hps_ctxt_reset_stas(void)
{
	mutex_lock(&hps_ctxt.lock);

	hps_ctxt_reset_stas_nolock();

	mutex_unlock(&hps_ctxt.lock);
}

/*
 * hps hps_ctxt_t print interface
 */
void hps_ctxt_print_basic(int toUart)
{
	if (toUart) {
		hps_warn("hps_ctxt.init_state: %u\n",
			hps_ctxt.init_state);
		hps_warn("hps_ctxt.state: %u\n",
			hps_ctxt.state);
		hps_warn("hps_ctxt.enabled: %u\n",
			hps_ctxt.enabled);
		hps_warn("hps_ctxt.suspend_enabled: %u\n",
			hps_ctxt.suspend_enabled);
		hps_warn("hps_ctxt.is_hmp: %u\n",
			hps_ctxt.is_hmp);
		hps_warn("hps_ctxt.little_cpu_id_min: %u\n",
			hps_ctxt.little_cpu_id_min);
		hps_warn("hps_ctxt.little_cpu_id_max: %u\n",
			hps_ctxt.little_cpu_id_max);
		hps_warn("hps_ctxt.big_cpu_id_min: %u\n",
			hps_ctxt.big_cpu_id_min);
		hps_warn("hps_ctxt.big_cpu_id_max: %u\n",
			hps_ctxt.big_cpu_id_max);
	} else {
		hps_debug("hps_ctxt.init_state: %u\n",
			hps_ctxt.init_state);
		hps_debug("hps_ctxt.state: %u\n",
			hps_ctxt.state);
		hps_debug("hps_ctxt.enabled: %u\n",
			hps_ctxt.enabled);
		hps_debug("hps_ctxt.suspend_enabled: %u\n",
			hps_ctxt.suspend_enabled);
		hps_debug("hps_ctxt.is_hmp: %u\n",
			hps_ctxt.is_hmp);
		hps_debug("hps_ctxt.little_cpu_id_min: %u\n",
			hps_ctxt.little_cpu_id_min);
		hps_debug("hps_ctxt.little_cpu_id_max: %u\n",
			hps_ctxt.little_cpu_id_max);
		hps_debug("hps_ctxt.big_cpu_id_min: %u\n",
			hps_ctxt.big_cpu_id_min);
		hps_debug("hps_ctxt.big_cpu_id_max: %u\n",
			hps_ctxt.big_cpu_id_max);
	}
}

void hps_ctxt_print_algo_config(int toUart)
{
	if (toUart) {
		hps_warn("hps_ctxt.up_threshold: %u\n",
			hps_ctxt.up_threshold);
		hps_warn("hps_ctxt.up_times: %u\n",
			hps_ctxt.up_times);
		hps_warn("hps_ctxt.down_threshold: %u\n",
			hps_ctxt.down_threshold);
		hps_warn("hps_ctxt.down_times: %u\n",
			hps_ctxt.down_times);
		hps_warn("hps_ctxt.input_boost_enabled: %u\n",
			hps_ctxt.input_boost_enabled);
		hps_warn("hps_ctxt.input_boost_cpu_num: %u\n",
			hps_ctxt.input_boost_cpu_num);
		hps_warn("hps_ctxt.rush_boost_enabled: %u\n",
			hps_ctxt.rush_boost_enabled);
		hps_warn("hps_ctxt.rush_boost_threshold: %u\n",
			hps_ctxt.rush_boost_threshold);
		hps_warn("hps_ctxt.rush_boost_times: %u\n",
			hps_ctxt.rush_boost_times);
		hps_warn("hps_ctxt.tlp_times: %u\n",
			hps_ctxt.tlp_times);
	} else {
		hps_debug("hps_ctxt.up_threshold: %u\n",
			hps_ctxt.up_threshold);
		hps_debug("hps_ctxt.up_times: %u\n",
			hps_ctxt.up_times);
		hps_debug("hps_ctxt.down_threshold: %u\n",
			hps_ctxt.down_threshold);
		hps_debug("hps_ctxt.down_times: %u\n",
			hps_ctxt.down_times);
		hps_debug("hps_ctxt.input_boost_enabled: %u\n",
			hps_ctxt.input_boost_enabled);
		hps_debug("hps_ctxt.input_boost_cpu_num: %u\n",
			hps_ctxt.input_boost_cpu_num);
		hps_debug("hps_ctxt.rush_boost_enabled: %u\n",
			hps_ctxt.rush_boost_enabled);
		hps_debug("hps_ctxt.rush_boost_threshold: %u\n",
			hps_ctxt.rush_boost_threshold);
		hps_debug("hps_ctxt.rush_boost_times: %u\n",
			hps_ctxt.rush_boost_times);
		hps_debug("hps_ctxt.tlp_times: %u\n",
			hps_ctxt.tlp_times);
	}
}

void hps_ctxt_print_algo_bound(int toUart)
{
	if (toUart) {
		hps_warn("hps_ctxt.little_num_base_perf_serv: %u\n",
			hps_ctxt.little_num_base_perf_serv);
		hps_warn("hps_ctxt.little_num_limit_thermal: %u\n",
			hps_ctxt.little_num_limit_thermal);
		hps_warn("hps_ctxt.little_num_limit_low_battery: %u\n",
			hps_ctxt.little_num_limit_low_battery);
		hps_warn("hps_ctxt.little_num_limit_ultra_power_saving: %u\n",
			hps_ctxt.little_num_limit_ultra_power_saving);
		hps_warn("hps_ctxt.little_num_limit_power_serv: %u\n",
			hps_ctxt.little_num_limit_power_serv);
		hps_warn("hps_ctxt.big_num_base_perf_serv: %u\n",
			hps_ctxt.big_num_base_perf_serv);
		hps_warn("hps_ctxt.big_num_limit_thermal: %u\n",
			hps_ctxt.big_num_limit_thermal);
		hps_warn("hps_ctxt.big_num_limit_low_battery: %u\n",
			hps_ctxt.big_num_limit_low_battery);
		hps_warn("hps_ctxt.big_num_limit_ultra_power_saving: %u\n",
			hps_ctxt.big_num_limit_ultra_power_saving);
		hps_warn("hps_ctxt.big_num_limit_power_serv: %u\n",
			hps_ctxt.big_num_limit_power_serv);
	} else {
		hps_debug("hps_ctxt.little_num_base_perf_serv: %u\n",
			hps_ctxt.little_num_base_perf_serv);
		hps_debug("hps_ctxt.little_num_limit_thermal: %u\n",
			hps_ctxt.little_num_limit_thermal);
		hps_debug("hps_ctxt.little_num_limit_low_battery: %u\n",
			hps_ctxt.little_num_limit_low_battery);
		hps_debug("hps_ctxt.little_num_limit_ultra_power_saving: %u\n",
			hps_ctxt.little_num_limit_ultra_power_saving);
		hps_debug("hps_ctxt.little_num_limit_power_serv: %u\n",
			hps_ctxt.little_num_limit_power_serv);
		hps_debug("hps_ctxt.big_num_base_perf_serv: %u\n",
			hps_ctxt.big_num_base_perf_serv);
		hps_debug("hps_ctxt.big_num_limit_thermal: %u\n",
			hps_ctxt.big_num_limit_thermal);
		hps_debug("hps_ctxt.big_num_limit_low_battery: %u\n",
			hps_ctxt.big_num_limit_low_battery);
		hps_debug("hps_ctxt.big_num_limit_ultra_power_saving: %u\n",
			hps_ctxt.big_num_limit_ultra_power_saving);
		hps_debug("hps_ctxt.big_num_limit_power_serv: %u\n",
			hps_ctxt.big_num_limit_power_serv);
	}
}

void hps_ctxt_print_algo_stats_cur(int toUart)
{
	if (toUart) {
		hps_warn("hps_ctxt.cur_loads: %u\n",
			hps_ctxt.cur_loads);
		hps_warn("hps_ctxt.cur_tlp: %u\n",
			hps_ctxt.cur_tlp);
		hps_warn("hps_ctxt.cur_iowait: %u\n",
			hps_ctxt.cur_iowait);
		hps_warn("hps_ctxt.cur_nr_heavy_task: %u\n",
			hps_ctxt.cur_nr_heavy_task);
	} else {
		hps_debug("hps_ctxt.cur_loads: %u\n",
			hps_ctxt.cur_loads);
		hps_debug("hps_ctxt.cur_tlp: %u\n",
			hps_ctxt.cur_tlp);
		hps_debug("hps_ctxt.cur_iowait: %u\n",
			hps_ctxt.cur_iowait);
		hps_debug("hps_ctxt.cur_nr_heavy_task: %u\n",
			hps_ctxt.cur_nr_heavy_task);
	}
}

void hps_ctxt_print_algo_stats_up(int toUart)
{
	if (toUart) {
		hps_warn("hps_ctxt.up_loads_sum: %u\n",
			hps_ctxt.up_loads_sum);
		hps_warn("hps_ctxt.up_loads_count: %u\n",
			hps_ctxt.up_loads_count);
		hps_warn("hps_ctxt.up_loads_history_index: %u\n",
			hps_ctxt.up_loads_history_index);
	} else {
		hps_debug("hps_ctxt.up_loads_sum: %u\n",
			hps_ctxt.up_loads_sum);
		hps_debug("hps_ctxt.up_loads_count: %u\n",
			hps_ctxt.up_loads_count);
		hps_debug("hps_ctxt.up_loads_history_index: %u\n",
			hps_ctxt.up_loads_history_index);
	}
}

void hps_ctxt_print_algo_stats_down(int toUart)
{
	if (toUart) {
		hps_warn("hps_ctxt.down_loads_sum: %u\n",
			hps_ctxt.down_loads_sum);
		hps_warn("hps_ctxt.down_loads_count: %u\n",
			hps_ctxt.down_loads_count);
		hps_warn("hps_ctxt.down_loads_history_index: %u\n",
			hps_ctxt.down_loads_history_index);
	} else {
		hps_debug("hps_ctxt.down_loads_sum: %u\n",
			hps_ctxt.down_loads_sum);
		hps_debug("hps_ctxt.down_loads_count: %u\n",
			hps_ctxt.down_loads_count);
		hps_debug("hps_ctxt.down_loads_history_index: %u\n",
			hps_ctxt.down_loads_history_index);
	}
}

void hps_ctxt_print_algo_stats_tlp(int toUart)
{
	if (toUart) {
		hps_warn("hps_ctxt.tlp_sum: %u\n",
			hps_ctxt.tlp_sum);
		hps_warn("hps_ctxt.tlp_count: %u\n",
			hps_ctxt.tlp_count);
		hps_warn("hps_ctxt.tlp_history_index: %u\n",
			hps_ctxt.tlp_history_index);
		hps_warn("hps_ctxt.tlp_avg: %u\n",
			hps_ctxt.tlp_avg);
		hps_warn("hps_ctxt.rush_count: %u\n",
			hps_ctxt.rush_count);
	} else {
		hps_debug("hps_ctxt.tlp_sum: %u\n",
			hps_ctxt.tlp_sum);
		hps_debug("hps_ctxt.tlp_count: %u\n",
			hps_ctxt.tlp_count);
		hps_debug("hps_ctxt.tlp_history_index: %u\n",
			hps_ctxt.tlp_history_index);
		hps_debug("hps_ctxt.tlp_avg: %u\n",
			hps_ctxt.tlp_avg);
		hps_debug("hps_ctxt.rush_count: %u\n",
			hps_ctxt.rush_count);
	}
}

/*
 * probe callback
 */
static int hps_probe(struct platform_device *pdev)
{
	hps_warn("HPS: probe!\n");
	return 0;
}

/*
 * suspend callback
 */
static int hps_suspend(struct device *dev)
{
	int ret = 0;

	hps_warn("HPS: suspend start!\n");

	if (!hps_ctxt.suspend_enabled)
		goto suspend_end;

suspend_end:
	hps_ctxt.state = STATE_SUSPEND;
	hps_warn
("state:%u,enabled:%u,suspend_enabled:%u,rush_boost_enabled:%u,ret:%d\n",
		 hps_ctxt.state, hps_ctxt.enabled,
		 hps_ctxt.suspend_enabled, hps_ctxt.rush_boost_enabled, ret);
	return 0;
}


static int hps_resume(struct device *dev)
{
	hps_warn("HPS: resume!\n");

	if (!hps_ctxt.suspend_enabled)
		goto resume_end;
/*Add to speedup home screen*/
resume_end:
	hps_ctxt.state = STATE_EARLY_SUSPEND;
	hps_warn
("state: %u, enabled: %u, suspend_enabled: %u, rush_boost_enabled: %u\n",
		 hps_ctxt.state, hps_ctxt.enabled,
		 hps_ctxt.suspend_enabled, hps_ctxt.rush_boost_enabled);


	return 0;
}


static int hps_freeze(struct device *dev)
{
	int cpu;

	hps_warn("HPS: freeze!\n");

	if (!hps_ctxt.suspend_enabled)
		goto freeze_end;

	mutex_lock(&hps_ctxt.lock);
	hps_ctxt.enabled_backup = hps_ctxt.enabled;
	hps_ctxt.enabled = 0;
	/* Force fix cpu0 at IPOH stage */
	if (!cpu_online(0))
		add_cpu(0);
	for (cpu = hps_ctxt.big_cpu_id_max; cpu > hps_ctxt.little_cpu_id_min;
	cpu--) {
		if (cpu_online(cpu))
			remove_cpu(cpu);
	}
	mutex_unlock(&hps_ctxt.lock);

freeze_end:
	hps_ctxt.state = STATE_SUSPEND;
	hps_warn
("state: %u, enabled: %u, suspend_enabled: %u, rush_boost_enabled: %u\n",
		 hps_ctxt.state, hps_ctxt.enabled,
		 hps_ctxt.suspend_enabled, hps_ctxt.rush_boost_enabled);


	return 0;
}

static int hps_restore(struct device *dev)
{
	hps_warn("HPS: restore!\n");

	if (!hps_ctxt.suspend_enabled)
		goto restore_end;

	mutex_lock(&hps_ctxt.lock);
	hps_ctxt.enabled = hps_ctxt.enabled_backup;
	mutex_unlock(&hps_ctxt.lock);

restore_end:
	hps_ctxt.state = STATE_EARLY_SUSPEND;
	hps_warn
("state: %u, enabled: %u, suspend_enabled: %u, rush_boost_enabled: %u\n",
		 hps_ctxt.state, hps_ctxt.enabled,
		 hps_ctxt.suspend_enabled, hps_ctxt.rush_boost_enabled);

	return 0;
}

/*
 * module init function
 */
static int __init hps_init(void)
{
	int r = 0;

	hps_warn("HPS: init!\n");

	/* temp for bringup */
	/* return 0; */
	/* temp for bringup */

	/* hps_cpu_init() must before hps_core_init() */
	r = hps_cpu_init();
	if (r)
		hps_error("hps_cpu_init fail(%d)\n", r);

#if IS_ENABLED(CONFIG_HPS)
	r = hps_procfs_init();
	if (r)
		hps_error("hps_procfs_init fail(%d)\n", r);
#endif

	r = platform_device_register(&hotplug_strategy_pdev);
	if (r)
		hps_error("platform_device_register fail(%d)\n", r);

	r = platform_driver_register(&hps_ctxt.pdrv);
	if (r)
		hps_error("platform_driver_register fail(%d)\n", r);

	r = hps_core_init();
	if (r)
		hps_error("hps_core_init fail(%d)\n", r);

	hps_ctxt.init_state = INIT_STATE_DONE;
	hotplug_cb_init();

	return r;
}

/*
 * module exit function
 */
static void __exit hps_exit(void)
{
	int r = 0;

	hps_warn("HPS: exit!\n");

	hps_ctxt.init_state = INIT_STATE_NOT_READY;

	r = hps_core_deinit();
	if (r)
		hps_error("hps_core_deinit fail(%d)\n", r);
}
module_init(hps_init);
module_exit(hps_exit);

/*
 * module parameters
 */
/* module_param(g_enable, int, 0644); */
/* module_param(g_enable_dynamic_hps_at_suspend, int, 0644); */

MODULE_DESCRIPTION("MediaTek CPU Hotplug Stragegy Core v0.1");
MODULE_LICENSE("GPL");
