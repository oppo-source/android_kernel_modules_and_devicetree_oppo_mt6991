#if 0
#include <linux/init.h>
#include <linux/module.h>
#include <linux/geas_ctrl.h>
#include "geas.h"

static struct npu_params npu_data;

extern int geas_update_npu_params(struct npu_params *data);

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
extern int (*game_update_geas_npu_params)(struct npu_params * npu_datas);
#endif

int update_npu_params(struct npu_params *data)
{
	static DEFINE_MUTEX(update_npu_params_lock);

	mutex_lock(&update_npu_params_lock);

	if (data == NULL) {
		pr_err("%s, null data", __func__);
		mutex_unlock(&update_npu_params_lock);
		return -1;
	}
	memcpy(&npu_data, data, sizeof(struct npu_params));
	geas_update_npu_params(&npu_data);

	pr_err("%s, imax = %llu, amax = %llu", __func__, npu_data.imax, npu_data.amax);

	mutex_unlock(&update_npu_params_lock);

	return 0;
}

int geas_npu_init(void)
{
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	game_update_geas_npu_params = update_npu_params;
#endif

	pr_err("%s", __func__);
	return 0;
}

void geas_npu_exit(void)
{

}
#endif
