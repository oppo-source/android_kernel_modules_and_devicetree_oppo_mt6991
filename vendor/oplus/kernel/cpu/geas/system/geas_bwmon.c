/*
 * a simple kernel module: powermodel_proc
 */
#include <linux/init.h>
#include <linux/module.h>
#include "bwmon_geas.h"
#include <drivers/soc/qcom/dcvs/bwmon.h>
#include <linux/geas_ctrl.h>
#include "geas.h"

static struct bwmon_params bwmon_data;
static struct list_head *geas_hwmon_list = NULL;
static spinlock_t *geas_list_lock = NULL;
static spinlock_t *geas_sample_irq_lock = NULL;
static struct workqueue_struct *geas_bwmon_wq = NULL;

//extern int geas_update_bwmon_params(struct bwmon_params *bwmon_data);
extern void init_geas_with_bwmon(struct list_head **list, spinlock_t **lock, spinlock_t **irq_lock, struct workqueue_struct **wq);

extern int (*game_update_geas_bwmon_params)(struct bwmon_params * bwmon_datas);

int geas_update_bwmon_params(struct bwmon_params *bwmon_data)
{
	struct bw_hwmon *hw;
	struct hwmon_node *node;
	unsigned long flags;
	unsigned int min_freq, max_freq, ab_scale, sec_ab_scale;

	spin_lock_irqsave(geas_list_lock, flags);
	list_for_each_entry(node, geas_hwmon_list, list) {
		hw = node->hw;
		if (!hw->is_active)
			continue;

		if (hw->dcvs_hw == DCVS_DDR || hw->dcvs_hw == DCVS_LLCC) {
			if (hw->dcvs_hw == DCVS_DDR) {
				min_freq = bwmon_data->dimin;
				max_freq = bwmon_data->dimax;
				ab_scale = bwmon_data->dascale;
				sec_ab_scale = node->second_ab_scale;
			} else if (hw->dcvs_hw == DCVS_LLCC) {
				min_freq = bwmon_data->limin;
				max_freq = bwmon_data->limax;
				ab_scale = bwmon_data->lascale;
				sec_ab_scale = bwmon_data->lasscale;
			}

			if (min_freq >= 0) {
				min_freq = max(min_freq, node->hw_min_freq);
				min_freq = min(min_freq, node->max_freq);
				node->min_freq = min_freq;
				pr_err("%s, set node->min_freq = %u for dcvs_hw:%d", __func__, node->min_freq, hw->dcvs_hw);
			}

			if (max_freq >= 0) {
				max_freq = max(max_freq, node->min_freq);
				max_freq = min(max_freq, node->hw_max_freq);
				node->max_freq = max_freq;
				pr_err("%s, set node->max_freq = %u for dcvs_hw:%d", __func__, node->max_freq, hw->dcvs_hw);
			}

			if (ab_scale >= 0) {
				node->ab_scale = ab_scale;
				pr_err("%s, set node->ab_scale = %u for dcvs_hw:%d", __func__, node->ab_scale, hw->dcvs_hw);
			}

			if (sec_ab_scale >= 0) {
				node->second_ab_scale = sec_ab_scale;
				pr_err("%s, set node->second_ab_scale = %u for dcvs_hw:%d", __func__, node->second_ab_scale, hw->dcvs_hw);
			}
		}
	}
	spin_unlock_irqrestore(geas_list_lock, flags);

	return 0;
}

int update_bwmon_params(struct bwmon_params *data)
{
	if (data == NULL) {
		pr_err("%s, null data", __func__);
		return -1;
	}
	memcpy(&bwmon_data, data, sizeof(struct bwmon_params));
	geas_update_bwmon_params(&bwmon_data);

	pr_err("%s, limax = %d, lascale = %d, dimax = %d, dascale = %d", __func__, bwmon_data.limax, bwmon_data.lascale, bwmon_data.dimax, bwmon_data.dascale);

	return 0;
}
EXPORT_SYMBOL(update_bwmon_params);

int geas_bwmon_init(void)
{
	game_update_geas_bwmon_params = update_bwmon_params;

	init_geas_with_bwmon(&geas_hwmon_list, &geas_sample_irq_lock, &geas_list_lock, &geas_bwmon_wq);
	pr_err("%s", __func__);
	return 0;
}

void geas_bwmon_exit(void)
{

}
