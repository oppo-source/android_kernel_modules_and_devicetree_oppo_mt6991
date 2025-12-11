
#ifndef _QCOM_BWMON_GEAS_H
#define _QCOM_BWMON_GEAS_H

#include <linux/kernel.h>
#include <soc/qcom/dcvs.h>
#include <soc/qcom/pmu_lib.h>
#include <soc/qcom/dcvs.h>

enum frame_phase {
	PHASE_CPU_ACTIVE,
	PHASE_GPU_ACTIVE,
	MAX_PHASE,
};

enum frame_event {
	FRAME_DEQBUFFER,
	FRAME_BEGIN,
	FRAME_EVENT_MAX,
};



#define MAX_FRAME_HISTORY_RECORD 6
enum voting_method {
	VOTING_METHOD_AVG,
	VOTING_METHOD_RECENT_MAX,
	VOTING_METHOD_RECENT_DECAY,
};

struct frame_pmu_record {
	u64 instructions;
	u64 cpu_cycles;
	u64 ll_refill;
	u64 slc_refill;
	u64 stall_be_mem;
	u64 l2_wb_victim;
	u64 l2_prfm;
	unsigned long slc_mb;
	unsigned long ll_mb;
	unsigned long slc_mbps;
	unsigned long ll_mbps;
	unsigned int stall_be_rate;
	unsigned int ll_mpki;
	unsigned int slc_mpki;
	unsigned int slc_absorb_rate;
	unsigned long cpu_active_us;
};


struct frame_bw_history_record {
	unsigned int ib;
	unsigned long raw_mbps;
	unsigned int ab;
	unsigned int duration_ns;
	struct frame_pmu_record total_pmu;
	struct frame_pmu_record primary_pmu;
	unsigned long voting_freq;
	unsigned long cpufreq_mhz;
	unsigned long sec_voting_freq;
};

struct frame_bw_history {
	struct frame_bw_history_record records[MAX_FRAME_HISTORY_RECORD];
	unsigned int valid_record_count;
	unsigned int max_ib;
	unsigned int avg_ib;
	unsigned long sum_ib;
	unsigned int max_ab;
	unsigned int avg_ab;
	unsigned long sum_ab;
	unsigned long sum_ib_decay;
	unsigned long decay_ib;
	unsigned int cid;
	unsigned int cur_cid;
	unsigned int min_absorb_rate;
	ktime_t last_update_ts;
	unsigned long max_raw_mbps;
	unsigned long avg_raw_mbps;
};

#define MAX_OPP_CNT 40
#define MAX_POWER_ZONE 3
#define MAX_CPU_CNT 10
#define DDR_OPP_CNT 10
#define CTRL_TABLE_MAX_SIZE 30

struct frame_bw_history_manager {
	struct frame_bw_history hist[MAX_PHASE];
	ktime_t last_update_ts;
};

struct frame_bw {
	struct frame_bw_history_manager  *frame_hist_manager;
	unsigned int		frame_drive;
	unsigned int 		control_cpus;
	unsigned int		master_cpu;
	unsigned int		primary_cpus;
	unsigned int 		active_ib_scale;
	unsigned int 		nactive_ib_scale;
	unsigned int 		active_ab_scale;
	unsigned int 		nactive_ab_scale;
	unsigned int 		active_sec_ab_scale;
	unsigned int 		nactive_sec_ab_scale;
	unsigned int 		nactive_voting_method;
	unsigned int 		active_voting_method;
	unsigned int		hist_decay_rate;
	unsigned int 		frame_debug_level;
	unsigned int 		sec_voting_enhanced;
	unsigned int		sec_io_pct_scale;
	unsigned int		slc_mpki_thres;
	unsigned int		cur_ib;
	unsigned int		cur_ab;
	unsigned int		raw_mbps;
	struct qcom_pmu_data pmu_data[MAX_CPU_CNT];
	unsigned int		sec_mbps_zones[DDR_OPP_CNT];
};

struct hwmon_node;

struct hwmon_node_ext {
	struct frame_bw_history_manager *frame_hist_manager;
	unsigned int compute_record_count;
	struct work_struct frame_work;
	struct ctl_table geas_table[CTRL_TABLE_MAX_SIZE];
	struct dcvs_freq	cur_freqs[2];
	int (*bwmon_irq_handler)(struct hwmon_node *node);

	struct bw_hwmon *hw;
	struct hwmon_node *node;
	unsigned int wake;

	unsigned int frame_event;
	u64 last_ts;
	unsigned int frame_drive;
	unsigned int timer_drive;
	unsigned int enable_irq;
	unsigned int control_cpus;
	unsigned int master_cpu;
	unsigned int primary_cpus;
	unsigned int active_ib_scale;
	unsigned int nactive_ib_scale;
	unsigned int active_ab_scale;
	unsigned int nactive_ab_scale;
	unsigned int active_sec_ib_scale;
	unsigned int nactive_sec_ib_scale;
	unsigned int active_sec_ab_scale;
	unsigned int nactive_sec_ab_scale;
	unsigned int nactive_voting_method;
	unsigned int active_voting_method;
	unsigned int hist_decay_rate;
	unsigned int frame_debug_level;
	unsigned int sec_voting_enhanced;
	unsigned int sec_io_pct_scale;
	unsigned int slc_mpki_thres;
	unsigned int cur_ib;
	unsigned int cur_ab;
	unsigned long raw_mbps;
	unsigned long irq_raw_mbps;
	struct qcom_pmu_data pmu_data[MAX_CPU_CNT];
	unsigned int sec_mbps_zones[DDR_OPP_CNT];
};

#endif /* _QCOM_BWMON_GEAS_H */

