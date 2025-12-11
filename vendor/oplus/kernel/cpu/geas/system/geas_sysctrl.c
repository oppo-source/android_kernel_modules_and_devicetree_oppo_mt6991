#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS)

#define pr_fmt(fmt) "geas-sysctrl: " fmt

#include <linux/sched.h>
#include <linux/sysfs.h>
#include <linux/errno.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include "bwmon_geas.h"

#define BASE_DIR	"geas"	/* Subdir in /proc/		*/

static int five_hundred = 500;
static int two = 2;
static int fifteen = 15;
static int ten = 10;
static int ten_thousand = 10240;
static int max_record_cnt = MAX_FRAME_HISTORY_RECORD;

void bwmon_on_frame_event(int cpu, int event);
void update_geas_bwmon_periodly(int enable, int period_ms);

unsigned int frame_arrive = 0;

int geas_frame_arrive_handler(struct ctl_table *table,
				int write, void __user *buffer, size_t *lenp,
				loff_t *ppos)
{
	int ret;
	int old_value;
	static DEFINE_MUTEX(frame_arrive_lock);

	mutex_lock(&frame_arrive_lock);

	old_value = frame_arrive;

	ret = proc_dointvec_minmax(table, write, buffer, lenp, ppos);

	if (write) {
		if (frame_arrive == 1) {
			bwmon_on_frame_event(255, 1);
		} else if (frame_arrive == 2) {
			update_geas_bwmon_periodly(1, 0);
		} else if (frame_arrive == 3) {
			update_geas_bwmon_periodly(0, 0);
		} else if (frame_arrive >= 16) {
			update_geas_bwmon_periodly(1, frame_arrive);
		}
	}
	pr_err("%s, frame_arrive = %d", __func__, frame_arrive);

	mutex_unlock(&frame_arrive_lock);
	return ret;
}


struct ctl_table geas_test_table[] = {
	{
		.procname	= "frame_arrive",
		.data		= &frame_arrive,
		.maxlen 	= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= geas_frame_arrive_handler,
		.extra1 	= SYSCTL_ZERO,
		.extra2 	= &ten_thousand,
	},
	{},
};

void geas_init_proc(void)
{
	/*
	struct proc_dir_entry base_dir = proc_mkdir("geas", NULL);
	if (!base_dir) {
		printk(KERN_ERR "Couldn't create base dir /proc/%s\n",
				BASE_DIR);
		return;
	}

	proc_create("frame_arrive", 0644, base_dir, &proc_frame_ops);
	*/
	register_sysctl("geas", geas_test_table);
}

int geas_frame_drive_handler(struct ctl_table *table,
				int write, void __user *buffer, size_t *lenp,
				loff_t *ppos);

int geas_create_sysctrl_for_node(const char *dev_name, struct hwmon_node_ext *bwmon_ext)
{
	char *parent_dir = "geas";
	char *sysctrl_dir;
	size_t total_length;
	struct ctl_table geas_table[] = {
		{
			.procname	= "frame_drive",
			.data		= &bwmon_ext->frame_drive,
			.maxlen 	= sizeof(unsigned int),
			.mode		= 0644,
			.proc_handler	= geas_frame_drive_handler,
			.extra1 	= SYSCTL_ZERO,
			.extra2 	= SYSCTL_ONE,
		},
		{
			.procname	= "compute_record_count",
			.data		= &bwmon_ext->compute_record_count,
			.maxlen 	= sizeof(unsigned int),
			.mode		= 0644,
			.proc_handler	= proc_dointvec_minmax,
			.extra1 	= SYSCTL_ZERO,
			.extra2 	= &max_record_cnt,
		},
		{
			.procname	= "enable_irq",
			.data		= &bwmon_ext->enable_irq,
			.maxlen 	= sizeof(unsigned int),
			.mode		= 0644,
			.proc_handler	= proc_dointvec_minmax,
			.extra1 	= SYSCTL_ZERO,
			.extra2 	= SYSCTL_ONE,
		},
		{
			.procname	= "active_ib_scale",
			.data		= &bwmon_ext->active_ib_scale,
			.maxlen 	= sizeof(unsigned int),
			.mode		= 0644,
			.proc_handler	= proc_dointvec_minmax,
			.extra1 	= SYSCTL_ZERO,
			.extra2 	= &five_hundred,
		},
		{
			.procname	= "active_ab_scale",
			.data		= &bwmon_ext->active_ab_scale,
			.maxlen 	= sizeof(unsigned int),
			.mode		= 0644,
			.proc_handler	= proc_dointvec_minmax,
			.extra1 	= SYSCTL_ZERO,
			.extra2 	= &five_hundred,
		},
		{
			.procname	= "nactive_ib_scale",
			.data		= &bwmon_ext->nactive_ib_scale,
			.maxlen 	= sizeof(unsigned int),
			.mode		= 0644,
			.proc_handler	= proc_dointvec_minmax,
			.extra1 	= SYSCTL_ZERO,
			.extra2 	= &five_hundred,
		},
		{
			.procname	= "nactive_ab_scale",
			.data		= &bwmon_ext->nactive_ab_scale,
			.maxlen 	= sizeof(unsigned int),
			.mode		= 0644,
			.proc_handler	= proc_dointvec_minmax,
			.extra1 	= SYSCTL_ZERO,
			.extra2 	= &five_hundred,
		},
		{
			.procname	= "active_sec_ab_scale",
			.data		= &bwmon_ext->active_sec_ab_scale,
			.maxlen 	= sizeof(unsigned int),
			.mode		= 0644,
			.proc_handler	= proc_dointvec_minmax,
			.extra1 	= SYSCTL_ZERO,
			.extra2 	= &five_hundred,
		},
		{
			.procname	= "nactive_sec_ab_scale",
			.data		= &bwmon_ext->nactive_sec_ab_scale,
			.maxlen 	= sizeof(unsigned int),
			.mode		= 0644,
			.proc_handler	= proc_dointvec_minmax,
			.extra1 	= SYSCTL_ZERO,
			.extra2 	= &five_hundred,
		},
		{
			.procname	= "active_voting_method",
			.data		= &bwmon_ext->active_voting_method,
			.maxlen 	= sizeof(unsigned int),
			.mode		= 0644,
			.proc_handler	= proc_dointvec_minmax,
			.extra1 	= SYSCTL_ZERO,
			.extra2 	= &two,
		},
		{
			.procname	= "nactive_voting_method",
			.data		= &bwmon_ext->nactive_voting_method,
			.maxlen 	= sizeof(unsigned int),
			.mode		= 0644,
			.proc_handler	= proc_dointvec_minmax,
			.extra1 	= SYSCTL_ZERO,
			.extra2 	= &two,
		},
		{
			.procname	= "frame_debug_level",
			.data		= &bwmon_ext->frame_debug_level,
			.maxlen 	= sizeof(unsigned int),
			.mode		= 0644,
			.proc_handler	= proc_dointvec_minmax,
			.extra1 	= SYSCTL_ZERO,
			.extra2 	= &fifteen,
		},
		{
			.procname	= "sec_voting_enhanced",
			.data		= &bwmon_ext->sec_voting_enhanced,
			.maxlen 	= sizeof(unsigned int),
			.mode		= 0644,
			.proc_handler	= proc_dointvec_minmax,
			.extra1 	= SYSCTL_ZERO,
			.extra2 	= &ten,
		},
		{
			.procname	= "cur_ib",
			.data		= &bwmon_ext->cur_ib,
			.maxlen 	= sizeof(unsigned int),
			.mode		= 0444,
			.proc_handler	= proc_dointvec_minmax,
			.extra1 	= SYSCTL_ZERO,
			.extra2 	= SYSCTL_INT_MAX,
		},
		{
			.procname	= "cur_ab",
			.data		= &bwmon_ext->cur_ab,
			.maxlen 	= sizeof(unsigned int),
			.mode		= 0444,
			.proc_handler	= proc_dointvec_minmax,
			.extra1 	= SYSCTL_ZERO,
			.extra2 	= SYSCTL_INT_MAX,
		},
	};
	memcpy(bwmon_ext->geas_table, geas_table, sizeof(geas_table));

	total_length = strlen(parent_dir) + strlen(dev_name) + 2;
	sysctrl_dir = kmalloc(total_length, GFP_KERNEL);
	if (!sysctrl_dir) {
		pr_err("%s failed, dev_name = %s", __func__, dev_name);
		return -1;
	}

	snprintf(sysctrl_dir, total_length, "%s/%s", parent_dir, dev_name);
	register_sysctl(sysctrl_dir, bwmon_ext->geas_table);
	kfree(sysctrl_dir);

	pr_err("%s success, dev_name = %s", __func__, dev_name);
	return 0;
}

#endif

