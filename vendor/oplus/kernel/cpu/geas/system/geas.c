
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_FDRIVE) || IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_BWMON)
#include "bwmon_geas.h"
#include <drivers/soc/qcom/dcvs/bwmon.h>
#endif
#include "geas.h"
#include <linux/geas_ctrl.h>

static int test_mode = 1;

#if 0
int geas_npu_init(void);
int geas_npu_exit(void);
#endif

#define GEAS_PARAM_TYPE_FDRIVE 1
#define GEAS_PARAM_TYPE_GPU 2
#define GEAS_PARAM_TYPE_BWMON 3
#define GEAS_PARAM_TYPE_MEMLAT 4
#define GEAS_PARAM_TYPE_EMI 5
#define GEAS_PARAM_TYPE_NPU 6
#define GEAS_PARAM_TYPE_MAX 7

static int geas_params_type_start = GEAS_PARAM_TYPE_FDRIVE;
static int geas_params_type_end = GEAS_PARAM_TYPE_MAX;
static unsigned int update_geas_params_type = 0;


#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_FDRIVE)
int geas_frame_drive_init(void);
void geas_frame_drive_exit(void);
int update_fdrive_params(struct frame_drive_params * data);
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_GPU)
int geas_gpu_init(void);
void geas_gpu_exit(void);
int update_gpu_params(struct gpu_params * data);
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_BWMON)
int geas_bwmon_init(void);
void geas_bwmon_exit(void);
int update_bwmon_params(struct bwmon_params *data);
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_BWMON)
int geas_memlat_init(void);
void geas_memlat_exit(void);
int update_memlat_params(struct memlat_params *data);
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_NPU)
#if 0
int geas_npu_init(void);
void geas_npu_exit(void);
int update_npu_params(struct npu_params *data);
#endif
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_EMI)
int geas_emi_init(void);
void geas_emi_exit(void);
int update_emi_params(struct emi_params *data);
#endif


int update_geas_params_type_handler(struct ctl_table *table,
				int write, void __user *buffer, size_t *lenp,
				loff_t *ppos)
{
	int ret;
	int old_value;

	old_value = update_geas_params_type;

	ret = proc_dointvec_minmax(table, write, buffer, lenp, ppos);

	if (write) {
		switch (update_geas_params_type) {
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_FDRIVE)
		struct frame_drive_params * fdrive_data;
		case GEAS_PARAM_TYPE_FDRIVE:
			fdrive_data = kzalloc(sizeof(struct frame_drive_params), GFP_ATOMIC);
			if (!fdrive_data) {
				pr_err("%s, kzalloc for fdrive_data failed", __func__);
				break;
			}
			fdrive_data->fd = 1;
			update_fdrive_params(fdrive_data);
			kfree(fdrive_data);
			break;
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_GPU)
		struct gpu_params * gpu_data;
		case GEAS_PARAM_TYPE_GPU:
			gpu_data = kzalloc(sizeof(struct gpu_params), GFP_ATOMIC);
			if (!gpu_data) {
				pr_err("%s, kzalloc for gpu_data failed", __func__);
				break;
			}
			gpu_data->imax = 7;
			gpu_data->imin = 4;
			gpu_data->amax = 1000;
			gpu_data->amin = 500;

			update_gpu_params(gpu_data);
			kfree(gpu_data);
			break;
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_BWMON)
		struct bwmon_params * bwmon_data;
		case GEAS_PARAM_TYPE_BWMON:
			bwmon_data = kzalloc(sizeof(struct bwmon_params), GFP_ATOMIC);
			if (!bwmon_data) {
				pr_err("%s, kzalloc for bwmon_params failed", __func__);
				break;
			}
			bwmon_data->limin = 50;
			bwmon_data->limax = 100;
			bwmon_data->dimin = 500;
			bwmon_data->dimax = 1000;
			bwmon_data->lascale = 30;
			bwmon_data->lasscale = 50;
			bwmon_data->dascale = 70;
			update_bwmon_params(bwmon_data);
			kfree(bwmon_data);
			break;
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_MEMLAT)
		struct memlat_params * memlat_data;
		case GEAS_PARAM_TYPE_MEMLAT:
			memlat_data = kzalloc(sizeof(struct memlat_params), GFP_ATOMIC);
			if (!memlat_data) {
				pr_err("%s, kzalloc for memlat_data failed", __func__);
				break;
			}
			memlat_data->limin = 200;
			memlat_data->limax = 400;
			memlat_data->dimin = 2000;
			memlat_data->dimax = 4000;
			update_memlat_params(memlat_data);
			kfree(memlat_data);
			break;
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_EMI)
		struct emi_params * emi_data;
		case GEAS_PARAM_TYPE_EMI:
			emi_data = kzalloc(sizeof(struct emi_params), GFP_ATOMIC);
			if (!emi_data) {
				pr_err("%s, kzalloc for emi_params failed", __func__);
				break;
			}
			emi_data->opp = 3;
			update_emi_params(emi_data);
			kfree(emi_data);
			break;
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_NPU)
#if 0
		struct npu_params * npu_data;
		case GEAS_PARAM_TYPE_NPU:
			npu_data = kzalloc(sizeof(struct npu_params), GFP_ATOMIC);
			if (!npu_data) {
				pr_err("%s, kzalloc for npu_data failed", __func__);
				break;
			}
			update_npu_params(npu_data);
			kfree(npu_data);
			break;
#endif
#endif
		default:
			break;
		}
	}
	pr_err("%s, update_geas_params_type = %d, sizeof geas_params = %lu", __func__, update_geas_params_type, sizeof (struct geas_params));

	return ret;
}

int update_geas_params_handler(struct ctl_table *table,
				int write, void __user *buffer, size_t *lenp,
				loff_t *ppos)
{
	int ret = 0;
	int type;
	int value1 = 0, value2 = 0, value3 = 0, value4 = 0, value5 = 0, value6 = 0, value7 = 0;

	sscanf(buffer, "%d", &type);

	if (write) {
		switch (type) {
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_FDRIVE)
		struct frame_drive_params * fdrive_data;
		case GEAS_PARAM_TYPE_FDRIVE:
			fdrive_data = kzalloc(sizeof(struct frame_drive_params), GFP_ATOMIC);
			if (!fdrive_data) {
				pr_err("%s, kzalloc for fdrive_data failed", __func__);
				break;
			}
			sscanf(buffer + 2, "%d %d %d %d %d %d %d", &value1, &value2, &value3, &value4, &value5, &value6, &value7);
			fdrive_data->fd = value1;
			fdrive_data->ei = value2;
			fdrive_data->ais = value3;
			fdrive_data->nais = value4;
			fdrive_data->aas = value5;
			fdrive_data->naas = value6;
			fdrive_data->fdl = value7;
			update_fdrive_params(fdrive_data);
			kfree(fdrive_data);
			break;
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_GPU)
		struct gpu_params * gpu_data;
		case GEAS_PARAM_TYPE_GPU:
			gpu_data = kzalloc(sizeof(struct gpu_params), GFP_ATOMIC);
			if (!gpu_data) {
				pr_err("%s, kzalloc for gpu_data failed", __func__);
				break;
			}
			sscanf(buffer + 2, "%d %d %d %d", &value1, &value2, &value3, &value4);
			gpu_data->imin = value1;
			gpu_data->imax = value2;
			gpu_data->amin = value3;
			gpu_data->amax = value4;
			update_gpu_params(gpu_data);
			kfree(gpu_data);
			break;
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_BWMON)
		struct bwmon_params * bwmon_data;
		case GEAS_PARAM_TYPE_BWMON:
			bwmon_data = kzalloc(sizeof(struct bwmon_params), GFP_ATOMIC);
			if (!bwmon_data) {
				pr_err("%s, kzalloc for bwmon_params failed", __func__);
				break;
			}
			sscanf(buffer + 2, "%d %d %d %d %d %d %d", &value1, &value2, &value3, &value4, &value5, &value5, &value5);
			bwmon_data->limin = value1;
			bwmon_data->limax = value2;
			bwmon_data->dimin = value3;
			bwmon_data->dimax = value4;
			bwmon_data->lascale = value5;
			bwmon_data->lasscale = value5;
			bwmon_data->dascale = value5;
			update_bwmon_params(bwmon_data);
			kfree(bwmon_data);
			break;
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_MEMLAT)
		struct memlat_params * memlat_data;
		case GEAS_PARAM_TYPE_MEMLAT:
			memlat_data = kzalloc(sizeof(struct memlat_params), GFP_ATOMIC);
			if (!memlat_data) {
				pr_err("%s, kzalloc for memlat_data failed", __func__);
				break;
			}
			sscanf(buffer + 2, "%d %d %d %d", &value1, &value2, &value3, &value4);
			memlat_data->limin = value1;
			memlat_data->limax = value2;
			memlat_data->dimin = value3;
			memlat_data->dimax = value4;
			update_memlat_params(memlat_data);
			kfree(memlat_data);
			break;
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_EMI)
		struct emi_params * emi_data;
		case GEAS_PARAM_TYPE_EMI:
			emi_data = kzalloc(sizeof(struct emi_params), GFP_ATOMIC);
			if (!emi_data) {
				pr_err("%s, kzalloc for emi_params failed", __func__);
				break;
			}
			sscanf(buffer + 2, "%d", &value1);
			emi_data->opp = value1;
			update_emi_params(emi_data);
			kfree(emi_data);
			break;
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_NPU)
#if 0
		struct npu_params * npu_data;
		case GEAS_PARAM_TYPE_NPU:
			npu_data = kzalloc(sizeof(struct npu_params), GFP_ATOMIC);
			if (!npu_data) {
				pr_err("%s, kzalloc for npu_data failed", __func__);
				break;
			}
			update_npu_params(npu_data);
			kfree(npu_data);
			break;
#endif
#endif
		default:
			break;
		}
	}

	pr_err("%s, type = %d, v1=%d, v2=%d, v3=%d, v4=%d, v5=%d, v6=%d, v7=%d",
			__func__, type, value1, value2, value3, value4, value5, value6, value7);

	return ret;
}

static struct ctl_table geas_test_table[] = {
	{
		.procname	= "update_geas_params_type",
		.data		= &update_geas_params_type,
		.maxlen 	= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= update_geas_params_type_handler,
		.extra1 	= &geas_params_type_start,
		.extra2 	= &geas_params_type_end,
	},
	{
		.procname	= "update_geas_params",
		.mode		= 0644,
		.proc_handler	= update_geas_params_handler,
	},
};

static int __init geas_init(void)
{
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_FDRIVE)
	geas_frame_drive_init();
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_BWMON)
	geas_bwmon_init();
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_MEMLAT)
	geas_memlat_init();
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_GPU)
	geas_gpu_init();
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_NPU)
#if 0
	geas_npu_init();
#endif
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_EMI)
	geas_emi_init();
#endif

	if (test_mode)
		register_sysctl("geas", geas_test_table);

	return 0;
}

static void __exit geas_exit(void)
{
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_FDRIVE)
	geas_frame_drive_exit();
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_BWMON)
	geas_bwmon_exit();
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_MEMLAT)
	geas_memlat_exit();
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_GPU)
	geas_gpu_exit();
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_NPU)
#if 0
	geas_npu_exit();
#endif
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS_EMI)
	geas_emi_exit();
#endif
}

module_init(geas_init);
module_exit(geas_exit);
MODULE_LICENSE("GPL");


