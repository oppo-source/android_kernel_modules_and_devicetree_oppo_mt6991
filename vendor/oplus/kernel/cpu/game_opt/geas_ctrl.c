#include "geas_ctrl.h"
#include "game_ctrl.h"
#include <linux/ioctl.h>

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GEAS)
int (*game_update_geas_fdrive_params)(struct frame_drive_params * fdrive_datas) = NULL;
EXPORT_SYMBOL(game_update_geas_fdrive_params);

int (*game_update_geas_gpu_params)(struct gpu_params * gpu_datas) = NULL;
EXPORT_SYMBOL(game_update_geas_gpu_params);

int (*game_update_geas_memlat_params)(struct memlat_params * memlat_datas) = NULL;
EXPORT_SYMBOL(game_update_geas_memlat_params);

int (*game_update_geas_bwmon_params)(struct bwmon_params * bwmon_datas) = NULL;
EXPORT_SYMBOL(game_update_geas_bwmon_params);

int (*game_update_geas_emi_params)(struct emi_params * emi_datas) = NULL;
EXPORT_SYMBOL(game_update_geas_emi_params);

#if 0
int (*game_update_geas_npu_params)(struct npu_params * npu_datas) = NULL;
EXPORT_SYMBOL(game_update_geas_npu_params);
#endif

static long update_geas_params(void __user *uarg)
{
	struct geas_params info;
	int ret = 0;

	pr_err("%s start", __func__);

	if (uarg == NULL) {
		ret = -EINVAL;
		goto ERROR_HANDLE;
	}

	if (copy_from_user(&info, uarg, sizeof(struct geas_params))) {
		ret = EFAULT;
		goto ERROR_HANDLE;
	}

	if (info.geasFlag & FRDR_FLAG && game_update_geas_fdrive_params != NULL)
		game_update_geas_fdrive_params(&(info.fdrive_datas));

	if (info.geasFlag & GPU_FLAG && game_update_geas_gpu_params != NULL)
		game_update_geas_gpu_params(&(info.gpu_datas));

	if (info.geasFlag & MEM_FALG && game_update_geas_memlat_params != NULL)
		game_update_geas_memlat_params(&(info.memlat_datas));

	if (info.geasFlag & BWM_FLAG && game_update_geas_bwmon_params != NULL)
		game_update_geas_bwmon_params(&(info.bwmon_datas));

	if (info.geasFlag & EMI_FLAG && game_update_geas_emi_params != NULL)
		game_update_geas_emi_params(&(info.emi_datas));

#if 0
	if (info.cxFlag & NPU_FLAG && game_update_geas_npu_params != NULL)
		game_update_geas_npu_params(&(info.npu_datas));
#endif
	goto out;

ERROR_HANDLE:
	pr_err("%s: kzalloc hwmon_node_ext fail, %d\n", __func__, ret);
	if (game_update_geas_fdrive_params != NULL)
		game_update_geas_fdrive_params(NULL);
	if (game_update_geas_gpu_params != NULL)
		game_update_geas_gpu_params(NULL);
	if (game_update_geas_memlat_params != NULL)
		game_update_geas_memlat_params(NULL);
	if (game_update_geas_bwmon_params != NULL)
		game_update_geas_bwmon_params(NULL);
	if (game_update_geas_emi_params != NULL)
		game_update_geas_emi_params(NULL);
#if 0
	if (game_update_geas_npu_params != NULL)
		game_update_geas_npu_params(NULL);
#endif

out:
	pr_err("%s end, ret = %d", __func__, ret);
	return ret;
}

static long geas_ctrl_ioctl(struct file* file, unsigned int cmd, unsigned long arg) {
    long ret = 0;
    void __user *uarg = (void __user *)arg;
    if ((_IOC_TYPE(cmd) !=  GEAS_MAGIC)) {
        return -EINVAL;
    }

    switch (cmd) {
		case CMD_ID_UPDATE_GEAS_PARAMS:
	        update_geas_params(uarg);
			// TODO
	    break;
		default:
		return -ENOTTY;
	}

	return ret;
}

static int geas_ctrl_open(struct inode *inode, struct file *file) {
    return 0;
}

static int geas_ctrl_release(struct inode *inode, struct file *file) {
	return 0;
}

#if IS_ENABLED(CONFIG_COMPAT)
static long compat_geas_ctrl_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return geas_ctrl_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
}
#endif /* CONFIG_COMPAT */

static const struct proc_ops geas_ctrl_proc_ops = {
    .proc_ioctl     = geas_ctrl_ioctl,
    .proc_open      = geas_ctrl_open,
    .proc_release   = geas_ctrl_release,
#if IS_ENABLED(CONFIG_COMPAT)
    .proc_compat_ioctl	= compat_geas_ctrl_ioctl,
#endif /* CONFIG_COMPAT */
    .proc_lseek     = default_llseek,
};

int geas_ctrl_init(void) {
    if (unlikely(!game_opt_dir)) {
        return -ENOTDIR;
    }

    proc_create_data("geas_ctrl", 0664, game_opt_dir, &geas_ctrl_proc_ops, NULL);
    return 0;
}

#endif
