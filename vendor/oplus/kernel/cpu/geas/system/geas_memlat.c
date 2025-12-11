/*
 * a simple kernel module: powermodel_proc
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/geas_ctrl.h>
#include "geas.h"

static struct memlat_params memlat_data;

extern int geas_update_memlat_params(int limin, int limax, int dimin, int dimax);

extern int (*game_update_geas_memlat_params)(struct memlat_params * memlat_data);

int update_memlat_params(struct memlat_params *data)
{
	static DEFINE_MUTEX(update_memlat_params_lock);

	mutex_lock(&update_memlat_params_lock);

	if (data == NULL) {
		pr_err("%s, null data", __func__);
		mutex_unlock(&update_memlat_params_lock);
		return -1;
	}
	memcpy(&memlat_data, data, sizeof(struct memlat_params));
	geas_update_memlat_params(data->limin, data->limax, data->dimin, data->dimax);

	pr_err("%s, limax = %d, dimax = %d", __func__, memlat_data.limax, memlat_data.dimax);

	mutex_unlock(&update_memlat_params_lock);

	return 0;
}

int geas_memlat_init(void)
{
	game_update_geas_memlat_params = update_memlat_params;

	pr_err("%s", __func__);
	return 0;
}

void geas_memlat_exit(void)
{

}
