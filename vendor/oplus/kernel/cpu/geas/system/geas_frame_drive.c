#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS)

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <soc/qcom/dcvs.h>
#include "bwmon_geas.h"

#include <drivers/soc/qcom/dcvs/bwmon.h>
#include <trace/hooks/sched.h>
#include <drivers/soc/qcom/dcvs/trace-dcvs.h>
#include <linux/geas_ctrl.h>
#include "geas.h"

static int inited = 0;
static int self_test = 1;

struct list_head *geas_hwmon_list = NULL;
spinlock_t *geas_list_lock = NULL;
spinlock_t *geas_sample_irq_lock = NULL;
struct workqueue_struct *geas_bwmon_wq = NULL;
static int min_irq_update_time = 4 * 1000000;
static int min_update_time = 8 * 1000000;
static int default_period_ns = 16 * 1000000;
static int geas_periodly_running = 0;
static int geas_period_ns = 16 * 1000000;

#define MIN_MBPS	500UL
#define HIST_PEAK_TOL	75
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_CFBT)
extern int (*cfbt_update_cx_voting_state )(int enable, int period_ms);
#endif

extern int (*game_update_geas_fdrive_params)(struct frame_drive_params * fdrive_datas);
extern void (*game_bwmon_on_frame_event)(int cpu, int event);

/* Returns MBps of read/writes for the sampling window. */
static unsigned long bytes_to_mbps(unsigned long long bytes, unsigned int us)
{
	bytes *= USEC_PER_SEC;
	do_div(bytes, us);
	bytes = DIV_ROUND_UP_ULL(bytes, SZ_1M);
	return bytes;
}

static unsigned long to_mbps_zone(struct hwmon_node *node, unsigned long mbps)
{
	int i;

	for (i = 0; i < NUM_MBPS_ZONES && node->mbps_zones[i]; i++)
		if (node->mbps_zones[i] >= mbps)
			return node->mbps_zones[i];

	return KHZ_TO_MBPS(node->max_freq, node->hw->dcvs_width);
}

static unsigned int mbps_to_bytes(unsigned long mbps, unsigned int ms)
{
	mbps *= ms;
	mbps = DIV_ROUND_UP(mbps, MSEC_PER_SEC);
	mbps *= SZ_1M;
	return mbps;
}

static int __bw_hwmon_sw_sample_end(struct bw_hwmon *hwmon, int irq)
{
	struct hwmon_node *node = hwmon->node;
	struct hwmon_node_ext *node_ext = &node->geas_frame_bw;
	ktime_t ts;
	unsigned long bytes, mbps;
	unsigned int us;
	int wake = 0;

	ts = ktime_get();
	us = ktime_to_us(ktime_sub(ts, node->prev_ts));

	bytes = hwmon->get_bytes_and_clear(hwmon);
	bytes += node->bytes;
	node->bytes = 0;

	mbps = bytes_to_mbps(bytes, us);
	node->max_mbps = max(node->max_mbps, mbps);

	/*
	 * If the measured bandwidth in a micro sample is greater than the
	 * wake up threshold, it indicates an increase in load that's non
	 * trivial. So, have the governor ignore historical idle time or low
	 * bandwidth usage and do the bandwidth calculation based on just
	 * this micro sample.
	 */
	if (mbps > node->hw->up_wake_mbps) {
		wake = UP_WAKE;
	} else if (mbps < node->hw->down_wake_mbps) {
		if (node->down_cnt)
			node->down_cnt--;
		if (node->down_cnt <= 0)
			wake = DOWN_WAKE;
	}

	node->prev_ts = ts;
	node->wake = wake;
	node_ext->wake = wake;
	if (irq)
		node_ext->irq_raw_mbps = mbps;
	node->sampled = true;

	trace_bw_hwmon_meas(dev_name(hwmon->dev),
				mbps,
				us,
				wake);

	return wake;
}

static int __bw_hwmon_hw_sample_end(struct bw_hwmon *hwmon, int irq)
{
	struct hwmon_node *node = hwmon->node;
	struct hwmon_node_ext *node_ext = &node->geas_frame_bw;
	unsigned long bytes, mbps;
	int wake = 0;

	/*
	 * If this read is in response to an IRQ, the HW monitor should
	 * return the measurement in the micro sample that triggered the IRQ.
	 * Otherwise, it should return the maximum measured value in any
	 * micro sample since the last time we called get_bytes_and_clear()
	 */
	bytes = hwmon->get_bytes_and_clear(hwmon);
	mbps = bytes_to_mbps(bytes, node->sample_ms * USEC_PER_MSEC);
	node->max_mbps = mbps;

	if (mbps > node->hw->up_wake_mbps)
		wake = UP_WAKE;
	else if (mbps < node->hw->down_wake_mbps)
		wake = DOWN_WAKE;

	node->wake = wake;
	node_ext->wake = wake;
	if (irq)
		node_ext->irq_raw_mbps = mbps;
	node->sampled = true;

	trace_bw_hwmon_meas(dev_name(hwmon->dev),
				mbps,
				node->sample_ms * USEC_PER_MSEC,
				wake);

	return 1;
}

static int __bw_hwmon_sample_end(struct bw_hwmon *hwmon, int irq)
{
	if (hwmon->set_hw_events)
		return __bw_hwmon_hw_sample_end(hwmon, irq);
	else
		return __bw_hwmon_sw_sample_end(hwmon, irq);
}

static unsigned long get_bw_and_set_irq_for_frame(struct hwmon_node *node,
					struct dcvs_freq *freq_mbps, int irq)
{
	unsigned long meas_mbps, thres, flags, req_mbps, adj_mbps;
	unsigned long meas_mbps_zone;
	unsigned long hist_lo_tol, hyst_lo_tol;
	struct bw_hwmon *hw = node->hw;
	unsigned int new_bw, io_percent = node->io_percent;
	ktime_t ts;
	unsigned int ms = 0;

	spin_lock_irqsave(geas_sample_irq_lock, flags);

	if (!node->geas_frame_bw.frame_drive && node->use_low_power_io_percent) {
		io_percent = node->low_power_io_percent;
	}

	if (!hw->set_hw_events) {
		ts = ktime_get();
		ms = ktime_to_ms(ktime_sub(ts, node->prev_ts));
	}
	if (!node->sampled || ms >= node->sample_ms)
		__bw_hwmon_sample_end(node->hw, irq);
	node->sampled = false;

	req_mbps = meas_mbps = node->max_mbps;
	node->max_mbps = 0;

	node->geas_frame_bw.raw_mbps = req_mbps;

	hist_lo_tol = (node->hist_max_mbps * HIST_PEAK_TOL) / 100;
	/* Remember historic peak in the past hist_mem decision windows. */
	if (meas_mbps > node->hist_max_mbps || !node->hist_mem) {
		/* If new max or no history */
		node->hist_max_mbps = meas_mbps;
		node->hist_mem = node->hist_memory;
	} else if (meas_mbps >= hist_lo_tol) {
		/*
		 * If subsequent peaks come close (within tolerance) to but
		 * less than the historic peak, then reset the history start,
		 * but not the peak value.
		 */
		node->hist_mem = node->hist_memory;
	} else {
		/* Count down history expiration. */
		if (node->hist_mem)
			node->hist_mem--;
	}

	/*
	 * The AB value that corresponds to the lowest mbps zone greater than
	 * or equal to the "frequency" the current measurement will pick.
	 * This upper limit is useful for balancing out any prediction
	 * mechanisms to be power friendly.
	 */
	meas_mbps_zone = (meas_mbps * 100) / io_percent;
	meas_mbps_zone = to_mbps_zone(node, meas_mbps_zone);
	meas_mbps_zone = (meas_mbps_zone * io_percent) / 100;
	meas_mbps_zone = max(meas_mbps, meas_mbps_zone);

	/*
	 * If this is a wake up due to BW increase, vote much higher BW than
	 * what we measure to stay ahead of increasing traffic and then set
	 * it up to vote for measured BW if we see down_count short sample
	 * windows of low traffic.
	 */
	if (node->wake == UP_WAKE) {
		req_mbps += ((meas_mbps - node->prev_req)
				* node->up_scale) / 100;
		/*
		 * However if the measured load is less than the historic
		 * peak, but the over request is higher than the historic
		 * peak, then we could limit the over requesting to the
		 * historic peak.
		 */
		if (req_mbps > node->hist_max_mbps
		    && meas_mbps < node->hist_max_mbps)
			req_mbps = node->hist_max_mbps;

		req_mbps = min(req_mbps, meas_mbps_zone);
	}

	hyst_lo_tol = (node->hyst_mbps * HIST_PEAK_TOL) / 100;
	if (meas_mbps > node->hyst_mbps && meas_mbps > MIN_MBPS) {
		hyst_lo_tol = (meas_mbps * HIST_PEAK_TOL) / 100;
		node->hyst_peak = 0;
		node->hyst_trig_win = node->hyst_length;
		node->hyst_mbps = meas_mbps;
		if (node->hyst_en)
			node->hyst_en = node->hyst_length;
	}

	/*
	 * Check node->max_mbps to avoid double counting peaks that cause
	 * early termination of a window.
	 */
	if (meas_mbps >= hyst_lo_tol && meas_mbps > MIN_MBPS
	    && !node->max_mbps) {
		node->hyst_peak++;
		if (node->hyst_peak >= node->hyst_trigger_count) {
			node->hyst_peak = 0;
			node->hyst_en = node->hyst_length;
		}
	}

	if (node->hyst_trig_win)
		node->hyst_trig_win--;
	if (node->hyst_en)
		node->hyst_en--;

	if (!node->hyst_trig_win && !node->hyst_en) {
		node->hyst_peak = 0;
		node->hyst_mbps = 0;
	}

	if (node->hyst_en) {
		if (meas_mbps > node->idle_mbps) {
			req_mbps = max(req_mbps, node->hyst_mbps);
			node->idle_en = node->idle_length;
		} else if (node->idle_en) {
			req_mbps = max(req_mbps, node->hyst_mbps);
			node->idle_en--;
		}
	}

	/* Stretch the short sample window size, if the traffic is too low */
	if (meas_mbps < MIN_MBPS) {
		hw->up_wake_mbps = (max(MIN_MBPS, req_mbps)
					* (100 + node->up_thres)) / 100;
		hw->down_wake_mbps = 0;
		thres = mbps_to_bytes(max(MIN_MBPS, req_mbps / 2),
					node->sample_ms);
	} else {
		/*
		 * Up wake vs down wake are intentionally a percentage of
		 * req_mbps vs meas_mbps to make sure the over requesting
		 * phase is handled properly. We only want to wake up and
		 * reduce the vote based on the measured mbps being less than
		 * the previous measurement that caused the "over request".
		 */
		hw->up_wake_mbps = (req_mbps * (100 + node->up_thres)) / 100;
		hw->down_wake_mbps = (meas_mbps * node->down_thres) / 100;
		thres = mbps_to_bytes(meas_mbps, node->sample_ms);
	}

	if (hw->set_hw_events) {
		hw->down_cnt = node->down_count;
		hw->set_hw_events(hw, node->sample_ms);
	} else {
		node->down_cnt = node->down_count;
		node->bytes = hw->set_thres(hw, thres);
	}

	node->wake = 0;
	node->prev_req = req_mbps;

	spin_unlock_irqrestore(geas_sample_irq_lock, flags);

	adj_mbps = req_mbps + node->guard_band_mbps;

	if (adj_mbps > node->prev_ab) {
		new_bw = adj_mbps;
	} else {
		new_bw = adj_mbps * node->decay_rate
			+ node->prev_ab * (100 - node->decay_rate);
		new_bw /= 100;
	}

	node->prev_ab = new_bw;
	freq_mbps->ib = (new_bw * 100) / io_percent;
	if (node->ab_scale < 100)
		new_bw = mult_frac(new_bw, node->ab_scale, 100);
	freq_mbps->ab = roundup(new_bw, node->bw_step);
	trace_bw_hwmon_update(dev_name(node->hw->dev),
				freq_mbps->ab,
				freq_mbps->ib,
				hw->up_wake_mbps,
				hw->down_wake_mbps);

	trace_bw_hwmon_debug(dev_name(node->hw->dev),
				req_mbps,
				meas_mbps_zone,
				node->hist_max_mbps,
				node->hist_mem,
				node->hyst_mbps,
				node->hyst_en);
	return req_mbps;
}

static void dump_systrace_c(const char *dev, char * tag, int event, int counter) {
	char buf[256];
	snprintf(buf, sizeof(buf), "C|9999|%s_%s_%d|%d\n", dev, tag, event, counter);

	pr_err("%s, buf = %s", __func__, buf);
}

static unsigned int eventToPhase(int event) {
	/*
	if (event == FRAME_DEQBUFFER) {
		return PHASE_CPU_ACTIVE;
	} else if (event == FRAME_BEGIN) {
		return PHASE_GPU_ACTIVE;
	}
	*/
	return PHASE_CPU_ACTIVE;
}

static unsigned int eventToVote(int event) {
	/*
	if (event == FRAME_DEQBUFFER) {
		return PHASE_GPU_ACTIVE;
	} else if (event == FRAME_BEGIN) {
		return PHASE_CPU_ACTIVE;
	}
	*/
	return PHASE_CPU_ACTIVE;
}

static u64 get_pmu_event_abs_value(u32 target_id, struct qcom_pmu_data *pmu_data) {
	u32 event_id;
	int i;
	for (i = 0; i < pmu_data->num_evs; i++) {
		event_id = pmu_data->event_ids[i];
		if (event_id == target_id)
			return pmu_data->ev_data[i];
	}
	return 0;
}

static u32 get_pmu_event_delta_value(u32 target_id, struct qcom_pmu_data *prev_data,
		struct qcom_pmu_data *now_data)
{
	u64 prev_value = get_pmu_event_abs_value(target_id, prev_data);
	u64 now_value = get_pmu_event_abs_value(target_id, now_data);
	return now_value - prev_value;
}

void append_pmu_record(struct frame_pmu_record *r, struct qcom_pmu_data *prev_data,
		struct qcom_pmu_data *now_data)
{
	r->instructions += get_pmu_event_delta_value(EVENT_INSTR, prev_data, now_data);
	r->slc_refill += get_pmu_event_delta_value(EVENT_SLC_REFILE, prev_data, now_data);
}

void compute_pmu_statics(struct hwmon_node *node,
	struct frame_pmu_record *r, u64 duration_ns, int cpu, unsigned long cpufreq_mhz)
{
	if (r->instructions != 0) {
		r->slc_mpki = (r->slc_refill << 10) / r->instructions;
		if (unlikely(node->geas_frame_bw.frame_debug_level >= 11)) {
			dump_systrace_c(dev_name(node->hw->dev), "slc-mpki", cpu, r->slc_mpki);
		}
	}
}

static void update_frame_pmu_history(struct hwmon_node *node,
				struct frame_bw_history_record *r, u64 duration_ns)
{
	struct qcom_pmu_data pmu_data;
	int ret, cpu;
	for_each_possible_cpu(cpu) {
		//if ((1 << cpu) & node->control_cpus) {
		if ((1 << cpu) & node->geas_frame_bw.primary_cpus) {
			ret = qcom_pmu_read_all(cpu, &pmu_data);
			if (ret < 0) {
				pr_err("error reading bus counters on cpu%d: %d\n", cpu, ret);
				continue;
			}
			if (ret < 0 || pmu_data.num_evs == 0) {
				pr_err("error reading pmu counters on cpu%d: %d\n", cpu, ret);
				continue;
			}
			if (node->geas_frame_bw.pmu_data[cpu].num_evs != 0) {
				//append_pmu_record(&r->total_pmu, &node->pmu_data[cpu], &pmu_data);
				//if ((1 << cpu) & node->primary_cpus) {
					append_pmu_record(&r->primary_pmu, &node->geas_frame_bw.pmu_data[cpu], &pmu_data);
				//}
			}
			memcpy(&node->geas_frame_bw.pmu_data[cpu], &pmu_data, sizeof(struct qcom_pmu_data));
		}
	}
	//compute_pmu_statics(node, &r->total_pmu, duration_ns, node->master_cpu, r->cpufreq_mhz);
	compute_pmu_statics(node, &r->primary_pmu, duration_ns, node->geas_frame_bw.master_cpu + 1, r->cpufreq_mhz);
}

static void update_frame_bw_history(struct hwmon_node *node,
						unsigned int ib, unsigned int ab, int event, unsigned long raw_mbps)
{
	struct hwmon_node_ext *node_ext = &node->geas_frame_bw;
	if (node_ext->frame_hist_manager != NULL) {
		struct frame_bw_history *hist = &node_ext->frame_hist_manager->hist[eventToPhase(event)];
		struct frame_bw_history_record *records = hist->records;
		int compute_record_count = node_ext->compute_record_count;
		int decay_rate_sum = 0;
		int decay_rate_cid = (100 - node_ext->hist_decay_rate);
		ktime_t now = ktime_get();
		unsigned long delta_ns = now - node_ext->frame_hist_manager->last_update_ts;
		if (unlikely(node_ext->frame_hist_manager->last_update_ts == 0))
			delta_ns = 16000000;
		memset(&records[hist->cid], 0, sizeof(struct frame_bw_history_record));
		records[hist->cid].ab = ab;
		records[hist->cid].ib = ib;
		records[hist->cid].raw_mbps = raw_mbps;
		update_frame_pmu_history(node, &records[hist->cid], delta_ns);

		if (hist->valid_record_count < compute_record_count) {
			hist->valid_record_count++;
		}
		node_ext->frame_hist_manager->last_update_ts = now;
		hist->sum_ab = 0;
		hist->sum_ib = 0;
		hist->sum_ib_decay = 0;
		hist->max_ab = 0;
		hist->max_ib = 0;
		hist->min_absorb_rate = 1000;
		hist->max_raw_mbps = 0;
		unsigned long sum_raw_mbps = 0;
		for (int i = 0; i < compute_record_count; i++) {
			hist->sum_ab += records[i].ab;
			hist->sum_ib += records[i].ib;
			sum_raw_mbps += records[i].raw_mbps;
			//current history record
			if (i == hist->cid) {
				hist->sum_ib_decay += records[i].ib * decay_rate_cid;
				decay_rate_sum += decay_rate_cid;
			} else {
				hist->sum_ib_decay += records[i].ib * node_ext->hist_decay_rate;
				decay_rate_sum += node_ext->hist_decay_rate;
			}
			if (hist->max_ab < records[i].ab)
				hist->max_ab = records[i].ab;
			if (hist->max_ib < records[i].ib)
				hist->max_ib = records[i].ib;

			if (hist->max_raw_mbps < records[i].raw_mbps)
				hist->max_raw_mbps = records[i].raw_mbps;

			if (unlikely(node_ext->frame_debug_level >= 3))
				pr_err("%s, i=%d, event=%d, hist->cid=%u, ib=%u, ab=%u",
						__func__, i, event, hist->cid, records[i].ib, records[i].ab);
		}
		hist->cur_cid = hist->cid;
		hist->cid++;
		hist->cid = hist->cid % compute_record_count;
		if (hist->valid_record_count >= compute_record_count) {
			hist->avg_ab = hist->sum_ab / compute_record_count;
			hist->avg_ib = hist->sum_ib / compute_record_count;
			hist->avg_raw_mbps = sum_raw_mbps / compute_record_count;
			if (decay_rate_sum != 0) {
				hist->decay_ib = hist->sum_ib_decay / decay_rate_sum;
			}
		}

		if (unlikely(node_ext->frame_debug_level >= 2))
			pr_err("%s for node %s, event=%d, cur_cid=%u, cur_ib=%u, cur_ab=%u, avg_ib=%u, avg_ab=%u, avg_raw_mbps = %lu, max_ib=%u, max_ab=%u, max_raw_mbps=%lu, decay_ib=%lu",
					__func__, dev_name(node->hw->dev), event, hist->cur_cid, ib, ab, hist->avg_ib, hist->avg_ab, hist->avg_raw_mbps,
					hist->max_ib, hist->max_ab, hist->max_raw_mbps, hist->decay_ib);
	}
}

unsigned int get_hist_ib(struct hwmon_node *node, struct frame_bw_history *hist, int phase) {
	struct hwmon_node_ext *node_ext = &node->geas_frame_bw;
	int voting_method = phase == PHASE_CPU_ACTIVE ? node_ext->active_voting_method : node_ext->nactive_voting_method;
	unsigned int ib = 0;
	switch (voting_method) {
		case VOTING_METHOD_AVG:
			ib = hist->avg_ib;
			break;
		case VOTING_METHOD_RECENT_MAX:
			ib = hist->max_ib;
			break;
		case VOTING_METHOD_RECENT_DECAY:
			ib = hist->decay_ib;
			break;
		default:
			break;
	}
	return ib;
}

/*hardcode FIX ME*/
unsigned long def_ddr_freq_table[DDR_OPP_CNT] = {
	547000,
	1353000,
	1555000,
	1708000,
	2092000,
	2736000,
	3187000,
	3686000,
	4224000,
	4761000,
};

unsigned long new_ddr_freq_table[DDR_OPP_CNT] = {0};
static int new_ddr_table = 0;

void update_ddr_freq_table(unsigned long * ddr_freq_table, int cnt)
{
	if (cnt != DDR_OPP_CNT)
		return;

	memcpy(new_ddr_freq_table, ddr_freq_table, sizeof(unsigned long) * DDR_OPP_CNT);
	new_ddr_table = 1;
}
EXPORT_SYMBOL(update_ddr_freq_table);

unsigned long to_sec_zoned_mbps(struct hwmon_node *node, unsigned long mbps) {
	struct hwmon_node_ext *node_ext = &node->geas_frame_bw;
	int i;
	unsigned long *ddr_freq_table = new_ddr_table == 1 ? new_ddr_freq_table : def_ddr_freq_table;
	for (i = 0; i < DDR_OPP_CNT; i++) {
		if (mbps <= node_ext->sec_mbps_zones[i]) {
			return KHZ_TO_MBPS(ddr_freq_table[i], 4);
		}
	}
	return mbps;
}

unsigned int get_hist_sec_ib(struct hwmon_node *node, struct frame_bw_history *hist) {
	struct hwmon_node_ext *node_ext = &node->geas_frame_bw;
	unsigned int ib = 0;
	switch (node_ext->sec_voting_enhanced) {
		case 0:
		case 1:
			ib = hist->max_raw_mbps;
			break;
		case 2:
			ib = hist->avg_raw_mbps;
			break;
		case 3:
			ib = hist->records[hist->cur_cid].raw_mbps;
			break;
		default:
			break;
	}
	return ib;
}

static u32 get_dst_from_map_for_frame(struct bw_hwmon *hw, u32 src_vote)
{
	struct bwmon_second_map *map = hw->second_map;
	u32 dst_vote = 0;

	if (!map)
		goto out;

	while (map->src_freq && map->src_freq < src_vote)
		map++;
	if (!map->src_freq)
		map--;
	dst_vote = map->dst_freq;

out:
	return dst_vote;
}

static void get_bw_for_frame(struct hwmon_node *node)
{
	struct bw_hwmon *hw = node->hw;
	struct hwmon_node_ext *node_ext = &node->geas_frame_bw;
	struct dcvs_freq new_freq;

	if (unlikely(node_ext->frame_debug_level >= 11))
		dump_systrace_c(dev_name(node->hw->dev), "frame", 0, node_ext->frame_event);

	get_bw_and_set_irq_for_frame(node, &new_freq, 0);

	new_freq.ab = MBPS_TO_KHZ(new_freq.ab, hw->dcvs_width);
	new_freq.ib = MBPS_TO_KHZ(new_freq.ib, hw->dcvs_width);
	new_freq.ib = max(new_freq.ib, node->min_freq);
	new_freq.ib = min(new_freq.ib, node->max_freq);

	if (unlikely(node_ext->frame_debug_level >= 11))
		dump_systrace_c(dev_name(node->hw->dev), "cur_ib", 0, new_freq.ib);

	/* sched_boost_freq is intentionally not limited by max_freq */
	if (!(node_ext->frame_drive || node_ext->timer_drive) && node->cur_sched_boost) {
		new_freq.ib = max(new_freq.ib, node->sched_boost_freq);
	}

	node_ext->cur_ib = new_freq.ib;
	node_ext->cur_ab = new_freq.ab;
}

void update_new_freq_voting(struct hwmon_node *node, struct dcvs_freq *new_freq,
			int frame_event, unsigned long *cooked_mbps, u32 *primary_ab_mbps)
{
	struct hwmon_node_ext *node_ext = &node->geas_frame_bw;
	if (node_ext->frame_hist_manager != NULL) {
		int phase = eventToVote(frame_event);
		struct frame_bw_history *hist = &node_ext->frame_hist_manager->hist[phase];
		unsigned int ib_scale = phase == PHASE_CPU_ACTIVE ? node_ext->active_ib_scale : node_ext->nactive_ib_scale;
		unsigned int ab_scale = phase == PHASE_CPU_ACTIVE ? node_ext->active_ab_scale : node_ext->nactive_ab_scale;
		unsigned int sec_ib_scale = phase == PHASE_CPU_ACTIVE ? node_ext->active_sec_ib_scale : node_ext->nactive_sec_ib_scale;
		unsigned int sec_ab_scale = phase == PHASE_CPU_ACTIVE ? node_ext->active_sec_ab_scale : node_ext->nactive_sec_ab_scale;
		unsigned int io_pct = node->io_percent;
		unsigned int frame_ib = new_freq->ib;
		unsigned int frame_ab = new_freq->ab;
		unsigned long sec_voting_mips = node_ext->raw_mbps;
		unsigned long sec_vote_ib = 0;

		if (hist->valid_record_count >= node_ext->compute_record_count) {
			frame_ib = get_hist_ib(node, hist, phase);
			frame_ab = hist->avg_ab;
			sec_voting_mips = get_hist_sec_ib(node, hist);

			unsigned int slc_mpki = hist->records[hist->cur_cid].primary_pmu.slc_mpki;
			if (slc_mpki < node_ext->slc_mpki_thres) {
				io_pct = io_pct * node_ext->sec_io_pct_scale / 100;
			}
		} else {
			new_freq->ab = 0;
			new_freq->ib = 0;
			return;
		}

		new_freq->ab = frame_ab * ab_scale / 100;
		new_freq->ib = frame_ib * ib_scale / 100;
		new_freq->ib = max(new_freq->ib, node->min_freq);
		new_freq->ib = min(new_freq->ib, node->max_freq);
		if (node->hw->second_vote_supported) {
			*primary_ab_mbps = (*primary_ab_mbps)  * sec_ab_scale / 100;
			sec_vote_ib = sec_voting_mips * 100 / io_pct;
			sec_vote_ib = sec_vote_ib * sec_ib_scale / 100;
			sec_vote_ib = to_sec_zoned_mbps(node, sec_vote_ib);
			sec_vote_ib = max(sec_vote_ib, KHZ_TO_MBPS(get_dst_from_map_for_frame(node->hw,
									node->min_freq), node->hw->second_dcvs_width));
			sec_vote_ib = min(sec_vote_ib, KHZ_TO_MBPS(get_dst_from_map_for_frame(node->hw,
									node->max_freq), node->hw->second_dcvs_width));
			sec_vote_ib = min(sec_vote_ib, KHZ_TO_MBPS(get_dst_from_map_for_frame(node->hw,
									new_freq->ib), node->hw->second_dcvs_width));
			if (unlikely(node->geas_frame_bw.frame_debug_level >= 11))
				dump_systrace_c(dev_name(node->hw->dev), "sec_vote_mbps", 0, sec_vote_ib);
			*cooked_mbps = sec_vote_ib;
		}
		if (unlikely(node_ext->frame_debug_level >= 3))
			pr_err("%s, io_pct = %u, sec_voting_mips = %lu, frame_ab = %u", __func__, io_pct, sec_vote_ib, frame_ab);
	}
}

/*
 * Governor function that computes new target frequency
 * based on bw measurement (mbps) and updates cur_freq (khz).
 * Returns true if cur_freq was changed
 * Note: must hold node->update_lock before calling
 */
static bool bwmon_update_cur_freq_for_frame(struct hwmon_node *node)
{
	struct bw_hwmon *hw = node->hw;
	struct hwmon_node_ext *node_ext = &node->geas_frame_bw;
	struct dcvs_freq new_freq;
	u32 primary_ib_mbps = 0, primary_ab_mbps = 0;
	bool ret = false;

	unsigned long cooked_mbps = 0;
	u32 sec_ib_freq = 0;

	update_new_freq_voting(node, &new_freq, node_ext->frame_event, &cooked_mbps, &primary_ab_mbps);

	if (new_freq.ab == 0 && new_freq.ib == 0) {
		if (unlikely(node_ext->frame_debug_level >= 1))
			pr_err("%s, zero new_freq", __func__);
		return ret;
	}

	cooked_mbps = KHZ_TO_MBPS(new_freq.ib, hw->dcvs_width);
	if (hw->second_vote_supported)
		sec_ib_freq = MBPS_TO_KHZ(cooked_mbps, hw->second_dcvs_width);

	if (unlikely(node_ext->frame_debug_level >= 2))
		pr_err("%s for %s, new_freq->ib = %u, new_freq->ab = %u, sec_vote_ib = %lu, primary_ab_mbps = %u",
						__func__, dev_name(node->hw->dev), new_freq.ib, new_freq.ab, cooked_mbps, primary_ab_mbps);

	if (unlikely(node_ext->frame_debug_level >= 10)) {
		dump_systrace_c(dev_name(node->hw->dev), "vote_ib", 0, new_freq.ib);
		dump_systrace_c(dev_name(node->hw->dev), "sec_vote_ib", 0, sec_ib_freq);
	}

	if (new_freq.ib != node_ext->cur_freqs[0].ib ||
			new_freq.ab != node_ext->cur_freqs[0].ab) {
		node_ext->cur_freqs[0].ib = new_freq.ib;
		node_ext->cur_freqs[0].ab = new_freq.ab;
		if (hw->second_vote_supported) {
			if (hw->second_map)
				if (node_ext->sec_voting_enhanced == 0) {
					node_ext->cur_freqs[1].ib = get_dst_from_map_for_frame(hw,
									new_freq.ib);
				} else {
					node_ext->cur_freqs[1].ib = sec_ib_freq;
				}

			else if (hw->second_dcvs_width)
				node_ext->cur_freqs[1].ib = MBPS_TO_KHZ(primary_ib_mbps,
							hw->second_dcvs_width);
			else
				node_ext->cur_freqs[1].ib = 0;
			if (!node->cur_sched_boost)
				node_ext->cur_freqs[1].ib = min(node_ext->cur_freqs[1].ib,
							hw->second_vote_limit);
			if (hw->second_dcvs_width)
				node_ext->cur_freqs[1].ab = MBPS_TO_KHZ(primary_ab_mbps,
							hw->second_dcvs_width);
			else
				node_ext->cur_freqs[1].ab = 0;
			node_ext->cur_freqs[1].ab = mult_frac(node_ext->cur_freqs[1].ab,
							node->second_ab_scale, 100);

			node->cur_freqs[1].ib = node_ext->cur_freqs[1].ib;
			node->cur_freqs[1].ab = node_ext->cur_freqs[1].ab;
		}
		node->cur_freqs[0].ib = node_ext->cur_freqs[0].ib;
		node->cur_freqs[0].ab = node_ext->cur_freqs[0].ab;

		ret = true;
	}

	if (hw->second_vote_supported)
		trace_bw_hwmon_update(hw->second_dev_name,
				KHZ_TO_MBPS(node_ext->cur_freqs[1].ab, hw->second_dcvs_width),
				KHZ_TO_MBPS(node_ext->cur_freqs[1].ib, hw->second_dcvs_width),
				0, 0);

	return ret;
}

static void qcom_dcvs_update_votes_for_frame(struct hwmon_node *node)
{
	struct bw_hwmon *hw = node->hw;
	struct hwmon_node_ext *node_ext = &node->geas_frame_bw;
	ktime_t now;
	int err = 0;

	mutex_lock(&node->update_lock);

	now = ktime_get();
	if (now - node_ext->last_ts < min_update_time) {
		if (unlikely(node_ext->frame_debug_level >= 1))
			pr_err("%s for %s, delta = %llu, ignore", __func__, dev_name(hw->dev), now - node_ext->last_ts);
		mutex_unlock(&node->update_lock);
		return;
	}

	if (bwmon_update_cur_freq_for_frame(node)) {
		err = qcom_dcvs_update_votes(dev_name(hw->dev),
					node_ext->cur_freqs,
					1 + (hw->second_vote_supported << 1),
					hw->dcvs_path);
		if (unlikely(node_ext->frame_debug_level >= 1)) {
			u32 llcc_ib_mhz = node_ext->cur_freqs[0].ib / 1000;
			u32 llcc_ab_mhz = node_ext->cur_freqs[0].ab / 1000;
			u32 llcc_min_mhz = node->min_freq / 1000;
			u32 llcc_max_mhz = node->max_freq / 1000;
			u32 ddr_ib_mhz = node_ext->cur_freqs[1].ib / 1000;
			u32 ddr_ab_mhz = node_ext->cur_freqs[1].ab / 1000;
			u32 ddr_min_mhz = get_dst_from_map_for_frame(node->hw, node->min_freq) / 1000;
			u32 ddr_max_mhz = get_dst_from_map_for_frame(node->hw, node->max_freq) / 1000;
			u32 mapped_ddr_ib_mhz = get_dst_from_map_for_frame(node->hw, node_ext->cur_freqs[0].ib) / 1000;
			pr_err("%s for %s, frame_event = %d, raw_mbps = %lu, llcc_ib = %u(%u, %u), llcc_ab = %u, ddr_ib = %u(%u, %u, %u), ddr_ab = %u, prev_ts = %llu",
					__func__, dev_name(hw->dev), node_ext->frame_event, node_ext->raw_mbps,
					llcc_ib_mhz, llcc_min_mhz, llcc_max_mhz, llcc_ab_mhz,
					ddr_ib_mhz, ddr_min_mhz, ddr_max_mhz, mapped_ddr_ib_mhz, ddr_ab_mhz, node_ext->last_ts);

			dump_systrace_c(dev_name(node->hw->dev), "raw_mbps", 0, node_ext->raw_mbps);
			dump_systrace_c(dev_name(node->hw->dev), "fix_llcc_ib", 0, llcc_ib_mhz);
			dump_systrace_c(dev_name(node->hw->dev), "fix_ddr_ib", 0, ddr_ib_mhz);
		}
	} else if (unlikely(node_ext->frame_debug_level >= 1)) {
		pr_err("%s for %s, cur_freq no update, ignore", __func__, dev_name(hw->dev));
	}

	if (err < 0)
		dev_err(hw->dev, "bwmon monitor update failed: %d\n", err);

	mutex_unlock(&node->update_lock);
}

void bwmon_monitor_frame_work(struct work_struct *work)
{
	struct hwmon_node_ext *node_ext = container_of(work, struct hwmon_node_ext, frame_work);
	struct hwmon_node *node = node_ext->node;
	u64 now = 0;

	/*from timer callback*/
	if (node_ext->timer_drive)
		qcom_dcvs_update_votes_for_frame(node);

	/* governor update and commit */
	mutex_lock(&node->update_lock);

	now = ktime_get();
	if (now - node_ext->last_ts < min_update_time) {
		if (unlikely(node_ext->frame_debug_level >= 1))
			pr_err("%s for %s, now = %llu, last_ts = %llu, delta = %llu",
					__func__, dev_name(node->hw->dev), now, node_ext->last_ts, now - node_ext->last_ts);
		mutex_unlock(&node->update_lock);
		return;
	}

	get_bw_for_frame(node);
	update_frame_bw_history(node, node_ext->cur_ib, node_ext->cur_ab, node_ext->frame_event, node_ext->raw_mbps);

	node_ext->last_ts = now;
	node->hw->last_update_ts = now;

	mutex_unlock(&node->update_lock);

	if (unlikely(node_ext->frame_debug_level >= 2))
		pr_err("%s, timer_drive = %u, frame_drive = %u", __func__, node_ext->timer_drive, node_ext->frame_drive);
}

extern void init_geas_with_bwmon(struct list_head **list, spinlock_t **lock, spinlock_t **irq_lock, struct workqueue_struct **wq);
void geas_init_proc(void);
void geas_create_sysctrl_for_node(const char *dev_name, struct hwmon_node_ext * node_ext);

static bool bwmon_update_cur_freq(struct hwmon_node *node)
{
	struct bw_hwmon *hw = node->hw;
	struct dcvs_freq new_freq;
	u32 primary_ib_mbps, primary_ab_mbps;
	bool ret = false;

	get_bw_and_set_irq_for_frame(node, &new_freq, 1);

	/* first convert freq from mbps to khz */
	primary_ab_mbps = new_freq.ab;
	new_freq.ab = MBPS_TO_KHZ(new_freq.ab, hw->dcvs_width);
	new_freq.ib = MBPS_TO_KHZ(new_freq.ib, hw->dcvs_width);
	new_freq.ib = max(new_freq.ib, node->min_freq);
	new_freq.ib = min(new_freq.ib, node->max_freq);
	/* sched_boost_freq is intentionally not limited by max_freq */
	if (node->cur_sched_boost)
		new_freq.ib = max(new_freq.ib, node->sched_boost_freq);
	primary_ib_mbps = KHZ_TO_MBPS(new_freq.ib, hw->dcvs_width);

	if (new_freq.ib != node->cur_freqs[0].ib ||
			new_freq.ab != node->cur_freqs[0].ab) {
		node->cur_freqs[0].ib = new_freq.ib;
		node->cur_freqs[0].ab = new_freq.ab;
		if (hw->second_vote_supported) {
			if (hw->second_map)
				node->cur_freqs[1].ib = get_dst_from_map_for_frame(hw,
								new_freq.ib);
			else if (hw->second_dcvs_width)
				node->cur_freqs[1].ib = MBPS_TO_KHZ(primary_ib_mbps,
							hw->second_dcvs_width);
			else
				node->cur_freqs[1].ib = 0;
			if (!node->cur_sched_boost)
				node->cur_freqs[1].ib = min(node->cur_freqs[1].ib,
							hw->second_vote_limit);
			if (hw->second_dcvs_width)
				node->cur_freqs[1].ab = MBPS_TO_KHZ(primary_ab_mbps,
							hw->second_dcvs_width);
			else
				node->cur_freqs[1].ab = 0;
			node->cur_freqs[1].ab = mult_frac(node->cur_freqs[1].ab,
							node->second_ab_scale, 100);
		}
		ret = true;
	}

	if (hw->second_vote_supported)
		trace_bw_hwmon_update(hw->second_dev_name,
				KHZ_TO_MBPS(node->cur_freqs[1].ab, hw->second_dcvs_width),
				KHZ_TO_MBPS(node->cur_freqs[1].ib, hw->second_dcvs_width),
				0, 0);

	return ret;
}

static int geas_irq_handler(struct hwmon_node *node)
{
	struct hwmon_node_ext *node_ext = &node->geas_frame_bw;
	struct bw_hwmon *hw = node->hw;
	bool new_freq = false;
	int ret = 0;
	int handled = 0;
	u64 now = ktime_get();

	if (node_ext->frame_drive || node_ext->timer_drive) {
		int wake = node->wake;
		if (node_ext->enable_irq) {
			if (now - node_ext->last_ts < min_irq_update_time) {
				if (unlikely(node_ext->frame_debug_level >= 1))
					pr_err("%s for %s, now = %llu, last_ts = %llu, delta = %llu",
							__func__, dev_name(node->hw->dev), now, node_ext->last_ts, now - node_ext->last_ts);
				return ret;
			}

			node_ext->irq_raw_mbps = node->max_mbps;
			new_freq = bwmon_update_cur_freq(node);
			node_ext->last_ts = now;
			if (new_freq) {
				u32 ext_ib = to_mbps_zone(node, KHZ_TO_MBPS(node_ext->cur_freqs[0].ib, node->hw->dcvs_width));
				u32 irq_ib = to_mbps_zone(node, KHZ_TO_MBPS(node->cur_freqs[0].ib, node->hw->dcvs_width));
				u32 ext_sec_ib = to_mbps_zone(node, KHZ_TO_MBPS(node_ext->cur_freqs[1].ib, node->hw->dcvs_width));
				u32 irq_sec_ib = to_mbps_zone(node, KHZ_TO_MBPS(node->cur_freqs[1].ib, node->hw->dcvs_width));
				bool need_up_wake = (irq_ib > ext_ib && wake == UP_WAKE) || (irq_sec_ib > ext_sec_ib && wake == UP_WAKE);
				bool need_down_wake = (irq_ib < ext_ib && wake == DOWN_WAKE) || (irq_sec_ib < ext_sec_ib && wake == DOWN_WAKE);
				if (node_ext->frame_debug_level >= 1)
					pr_err("%s for %s, ext_ib = %u, irq_ib = %u, ext_sec_ib = %u, irq_sec_ib = %u, wake = %u, irq_raw_mbps = %lu",
							__func__, dev_name(hw->dev), ext_ib, irq_ib, ext_sec_ib, irq_sec_ib, wake, node_ext->irq_raw_mbps);
				if (need_up_wake || need_down_wake) {
					node->cur_freqs[0].ab = node->cur_freqs[0].ab * node_ext->active_ab_scale / 100;
					node->cur_freqs[1].ab = node->cur_freqs[1].ab * node_ext->active_sec_ab_scale / 100;
					ret = qcom_dcvs_update_votes(dev_name(hw->dev),
								node->cur_freqs,
								1 + (hw->second_vote_supported << 1),
								hw->dcvs_path);
					if (node_ext->frame_debug_level >= 1)
						pr_err("%s for %s, qcom_dcvs_update_votes, need_up_wake = %d, need_down_wake = %d, ret = %d",
									__func__, dev_name(hw->dev), need_up_wake, need_down_wake, ret);
					if (ret < 0)
						dev_err(hw->dev, "bwmon irq update failed: %d\n", ret);
				}
				node_ext->wake = 0;
				node_ext->irq_raw_mbps = 0;
			}

			if (ret >= 0) {
				handled = 1;
			}

		}
	}

	if (node_ext->frame_debug_level >= 1)
		pr_err("%s for %s, frame_drive = %u, timer_drive = %u, enable_irq = %u, new_freq = %d, handled = %d, ret = %d",
				__func__, dev_name(hw->dev), node_ext->frame_drive, node_ext->timer_drive, node_ext->enable_irq, new_freq, handled, ret);

	return handled;
}

static struct frame_bw_history_manager *init_frame_bw_manager(void)
{
	struct frame_bw_history_manager *manager = kzalloc(sizeof(struct frame_bw_history_manager), GFP_KERNEL);
	if (!manager)
		return NULL;
	return manager;
}

static int init_hwmon_node_ext(const char *dev_name, struct hwmon_node_ext * node_ext, struct hwmon_node *node)
{
	node_ext->frame_hist_manager = init_frame_bw_manager();
	if (node_ext->frame_hist_manager == NULL)
		return -1;
	node_ext->compute_record_count = MAX_FRAME_HISTORY_RECORD;
	node_ext->frame_drive = 0;
	node_ext->timer_drive = 0;
	node_ext->last_ts = 0;
	node_ext->frame_event = 256;
	node_ext->enable_irq = 0;
	node_ext->control_cpus = 0;
	node_ext->master_cpu = 0;
	node_ext->primary_cpus = 0;
	node_ext->active_ib_scale = 100;
	node_ext->nactive_ib_scale = 100;
	node_ext->active_ab_scale = 10;
	node_ext->nactive_ab_scale = 10;
	node_ext->active_sec_ib_scale = 100;
	node_ext->nactive_sec_ib_scale = 100;
	node_ext->active_sec_ab_scale = 10;
	node_ext->nactive_sec_ab_scale = 10;
	node_ext->cur_ib = 0;
	node_ext->cur_ab = 0;
	node_ext->raw_mbps = 0;
	node_ext->active_voting_method = 0;
	node_ext->nactive_voting_method = 0;
	node_ext->hist_decay_rate = 0;
	node_ext->frame_debug_level = 0;
	node_ext->sec_voting_enhanced = 0;
	node_ext->sec_io_pct_scale = 0;
	node_ext->slc_mpki_thres = 0;
	node_ext->cur_freqs[0].hw_type = node->cur_freqs[0].hw_type;
	node_ext->cur_freqs[1].hw_type = node->cur_freqs[1].hw_type;
	memset(node_ext->sec_mbps_zones, 0, sizeof(unsigned int) * DDR_OPP_CNT);
	node_ext->bwmon_irq_handler = geas_irq_handler;

	geas_create_sysctrl_for_node(dev_name, node_ext);
	pr_err("%s, dev=%s", __func__, dev_name);

	return 0;
}

int update_fdrive_params(struct frame_drive_params * src_node_ext)
{
	unsigned long flags;
	struct hwmon_node *node;
	struct bw_hwmon *hw;
	struct hwmon_node_ext *dst_node_ext;

	if (!inited) {
		return -1;
	}

	spin_lock_irqsave(geas_list_lock, flags);
	list_for_each_entry(node, geas_hwmon_list, list) {
		hw = node->hw;
		dst_node_ext = &node->geas_frame_bw;
		if (src_node_ext == NULL) {
			dst_node_ext->frame_drive = 0;
		} else {
			dst_node_ext->compute_record_count = src_node_ext->crc;
			dst_node_ext->frame_drive = src_node_ext->fd;
			dst_node_ext->enable_irq = src_node_ext->ei;
			dst_node_ext->active_ib_scale = src_node_ext->ais;
			dst_node_ext->nactive_ib_scale = src_node_ext->nais;
			dst_node_ext->active_ab_scale = src_node_ext->aas;
			dst_node_ext->nactive_ab_scale = src_node_ext->naas;
			dst_node_ext->active_sec_ib_scale = src_node_ext->asis;
			dst_node_ext->nactive_sec_ib_scale = src_node_ext->nasis;
			dst_node_ext->active_sec_ab_scale = src_node_ext->asas;
			dst_node_ext->nactive_sec_ab_scale = src_node_ext->nasas;
			dst_node_ext->active_voting_method = src_node_ext->avm;
			dst_node_ext->nactive_voting_method = src_node_ext->navm;
			dst_node_ext->hist_decay_rate = src_node_ext->hdr;
			dst_node_ext->frame_debug_level = src_node_ext->fdl;
			dst_node_ext->sec_voting_enhanced = src_node_ext->sve;
			dst_node_ext->sec_io_pct_scale = src_node_ext->sips;
			dst_node_ext->slc_mpki_thres = src_node_ext->smt;
			memcpy(dst_node_ext->sec_mbps_zones, src_node_ext->smz, sizeof(unsigned int) * DDR_OPP_CNT);

			if (dst_node_ext->frame_drive && dst_node_ext->timer_drive) {
				pr_err("%s, unexpected state, frame_drive = timer_drive = 1", __func__);
			}
		}
	}
	spin_unlock_irqrestore(geas_list_lock, flags);

	if (unlikely(dst_node_ext && dst_node_ext->frame_debug_level >= 1))
		pr_err("%s, frame_drive = %u", __func__, dst_node_ext->frame_drive);

	return 0;
}

void init_geas_proc_node(void)
{
	unsigned long flags;
	struct hwmon_node *node;
	struct bw_hwmon *hw;
	struct hwmon_node_ext *node_ext;
	int ret = 0;

	if (!inited) {
		init_geas_with_bwmon(&geas_hwmon_list, &geas_list_lock, &geas_sample_irq_lock, &geas_bwmon_wq);
		if (geas_bwmon_wq) {
			spin_lock_irqsave(geas_list_lock, flags);
			list_for_each_entry(node, geas_hwmon_list, list) {
				hw = node->hw;
				node_ext = &node->geas_frame_bw;
				ret = init_hwmon_node_ext(dev_name(hw->dev), node_ext, node);
				if (ret) {
					break;
				}
				INIT_WORK(&node_ext->frame_work, &bwmon_monitor_frame_work);
			}
			if (!ret) {
				inited = 1;
			} else {
				list_for_each_entry(node, geas_hwmon_list, list) {
					node_ext = &node->geas_frame_bw;
					if (node_ext->frame_hist_manager) {
						kfree(node_ext->frame_hist_manager);
						node_ext->frame_hist_manager = NULL;
					}
				}
			}
			spin_unlock_irqrestore(geas_list_lock, flags);
		}
		pr_err("%s, geas_bwmon_wq = %p", __func__, geas_bwmon_wq);
 	}
}
EXPORT_SYMBOL(init_geas_proc_node);

void bwmon_on_frame_event(int cpu, int event)
{
	struct bw_hwmon *hw;
	struct hwmon_node *node;
	ktime_t now = ktime_get();
	struct hwmon_node_ext *node_ext = NULL;

	static DEFINE_MUTEX(frame_event_lock);

	mutex_lock(&frame_event_lock);

 	if (inited) {
		list_for_each_entry(node, geas_hwmon_list, list) {
			hw = node->hw;
			node_ext = &node->geas_frame_bw;
			if (!hw->is_active)
				continue;
			if (!node_ext->frame_drive)
				continue;
			node_ext->frame_event = event;
			qcom_dcvs_update_votes_for_frame(node);
			queue_work(geas_bwmon_wq, &node_ext->frame_work);

			if (unlikely(node_ext->frame_debug_level >= 1)) {
				u64 cost = ktime_get() - now;
				pr_err("%s for %s, cpu = %d, event = %d, inited = %d, geas_hwmon_list = %p, cost = %llu",
						__func__, dev_name(hw->dev), cpu, event, inited, geas_hwmon_list, cost);
			}
		}
	}

	mutex_unlock(&frame_event_lock);
}
EXPORT_SYMBOL(bwmon_on_frame_event);

static const u64 HALF_TICK_NS = (NSEC_PER_SEC / HZ) >> 1;
static void geas_jiffies_update_cb(void *unused, void *extra)
{
	struct bw_hwmon *hw;
	struct hwmon_node *node;
	struct hwmon_node_ext *node_ext = NULL;
	unsigned long flags;
	ktime_t now;
	u64 delta_ns;

	spin_lock_irqsave(geas_list_lock, flags);
	now = ktime_get();
	list_for_each_entry(node, geas_hwmon_list, list) {
		hw = node->hw;
		node_ext = &node->geas_frame_bw;
		if (!hw->is_active)
			continue;

		if (!node_ext->timer_drive)
			continue;

		delta_ns = now - hw->last_update_ts + HALF_TICK_NS;
		if (delta_ns > geas_period_ns) {
			queue_work(geas_bwmon_wq, &node_ext->frame_work);
		}
	}
	spin_unlock_irqrestore(geas_list_lock, flags);
}

int update_geas_bwmon_periodly(int enable, int period_ms)
{
	struct bw_hwmon *hw;
	struct hwmon_node *node;
	struct hwmon_node_ext *node_ext;
	u64 period_ns;
	int ret = -EINVAL;

	static DEFINE_MUTEX(geas_periodly_lock);

	if (!inited)
		return -EFAULT;

	mutex_lock(&geas_periodly_lock);
	period_ns = period_ms / 4 * 4 * 1000000;

	if (enable) {
		if (!geas_periodly_running) {
			list_for_each_entry(node, geas_hwmon_list, list) {
				hw = node->hw;
				node_ext = &node->geas_frame_bw;
				if (!hw->is_active)
					continue;
				if (node_ext->frame_drive)
					node_ext->frame_drive = 0;

				node_ext->timer_drive = 1;
			}
			if (period_ns == 0) {
				geas_period_ns = default_period_ns;
			} else {
				geas_period_ns = period_ns >= default_period_ns ? period_ns : default_period_ns;
			}
			ret = register_trace_android_vh_jiffies_update(geas_jiffies_update_cb, NULL);
			geas_periodly_running = 1;
		}
	} else if (geas_periodly_running) {
		list_for_each_entry(node, geas_hwmon_list, list) {
			hw = node->hw;
			node_ext = &node->geas_frame_bw;
			node_ext->timer_drive = 0;
		}

		ret = unregister_trace_android_vh_jiffies_update(geas_jiffies_update_cb, NULL);
		geas_periodly_running = 0;
		geas_period_ns = default_period_ns;
	}

	pr_err("%s, enable = %d, period_ns = %llu, geas_period_ns = %d",
			__func__, enable, period_ns, geas_period_ns);

	mutex_unlock(&geas_periodly_lock);

	return ret;
}
EXPORT_SYMBOL(update_geas_bwmon_periodly);

int geas_frame_drive_handler(struct ctl_table *table,
				int write, void __user *buffer, size_t *lenp,
				loff_t *ppos)
{
	int ret;
	int *frame_drive;
	int old_value, new_value;
	static DEFINE_MUTEX(frame_drive_lock);

	mutex_lock(&frame_drive_lock);

	frame_drive = table->data;
	old_value = *frame_drive;
	ret = proc_dointvec_minmax(table, write, buffer, lenp, ppos);
	new_value = *frame_drive;

	if (old_value != new_value && !new_value) {
		pr_err("%s, disable geas, frame_drive = %d", __func__, new_value);
	}
	pr_err("%s, frame_drive = %d", __func__, *frame_drive);

	mutex_unlock(&frame_drive_lock);
	return ret;
}

int geas_frame_drive_init(void)
{
	if (self_test)
		geas_init_proc();

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_CFBT)
	cfbt_update_cx_voting_state = update_geas_bwmon_periodly;
#endif

	game_update_geas_fdrive_params = update_fdrive_params;
	game_bwmon_on_frame_event = bwmon_on_frame_event;
	init_geas_proc_node();

	pr_err("%s", __func__);

	return 0;
}

void geas_frame_drive_exit(void)
{
//
}

#endif



