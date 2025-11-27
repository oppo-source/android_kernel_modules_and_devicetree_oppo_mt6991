#ifdef CONFIG_ARCH_MEDIATEK

#include <linux/init.h>
#include <linux/module.h>
#include <linux/geas_ctrl.h>
#include "geas.h"

static struct emi_params emi_data;

extern int geas_update_emi_params(struct emi_params *data);

extern int (*game_update_geas_emi_params)(struct emi_params * emi_datas);

int update_emi_params(struct emi_params *data)
{
	static DEFINE_MUTEX(update_emi_params_lock);

	mutex_lock(&update_emi_params_lock);

	if (data == NULL) {
		pr_err("%s, null data", __func__);
		mutex_unlock(&update_emi_params_lock);
		return -1;
	}
	memcpy(&emi_data, data, sizeof(struct emi_params));
	geas_update_emi_params(&emi_data);

	pr_err("%s, dvfsrc_ceiling_opp = %u", __func__,  data->opp);

	mutex_unlock(&update_emi_params_lock);

	return 0;
}

int geas_emi_init(void)
{
	game_update_geas_emi_params = update_emi_params;

	pr_err("%s", __func__);
	return 0;
}

void geas_emi_exit(void)
{

}
#endif

