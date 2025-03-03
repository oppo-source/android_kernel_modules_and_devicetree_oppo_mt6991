// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/uaccess.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/pm_qos.h>

#include "mtk-smi-bwc.h"
#ifdef PLL_HOPPING_READY
#include "mtk_freqhopping_drv.h"
#endif
#include "mmdvfs_mgr.h"
#include "mmdvfs_config_util.h"
#include "mmdvfs_internal.h"

void mmdvfs_qos_enable(bool enable)
{
	// TODO implement this
	pr_notice("WARN: %s: %d\n", __func__, enable);
}


#undef pr_fmt
#define pr_fmt(fmt) "[" MMDVFS_LOG_TAG "]" fmt

#define MMDVFS_ENABLE	1

#if defined(MMDVFS_ENABLE) && defined(MMDVFS_USE_FLIPER)
#include <mach/fliper.h>
#endif

/* MMDVFS legacy APIs */

#define MMDVFS_CLK_SWITCH_CB_MAX 16
#define MMDVFS_CLK_SWITCH_CLIENT_MSG_MAX 20

/* Legacy MMCLK change notifier. Keep the API now but */
/* it will be phased out (ISP may using the functionality)*/
static void notify_camsys_clk_change(
	int ori_mmsys_clk_mode, int update_mmsys_clk_mode);
static int mmsys_clk_change_notify_checked(
	clk_switch_cb func, int ori_mmsys_clk_mode,
int update_mmsys_clk_mode, char *msg);

/* Keep to adapt the new MMDVFS profile management method */
/* to the legacy IOCTL command quickly */
static int mmdvfs_query(enum MTK_SMI_BWC_SCEN scenario,
struct MTK_MMDVFS_CMD *cmd);

/* For legacy mmclk change notification and will be phase out */
static clk_switch_cb quick_mmclk_cbs[MMDVFS_CLK_SWITCH_CB_MAX];

/* Record the current CLK (debug only) */
static int current_mmsys_clk = MMSYS_CLK_MEDIUM;

/* Record the current step */
static s32 g_mmdvfs_current_step;

static int g_mmdvfs_current_vpu_step;

/* Record the enabled client */
static unsigned int g_mmdvfs_concurrency;
static struct MTK_SMI_BWC_MM_INFO *g_mmdvfs_info;
static struct MTK_MMDVFS_CMD g_mmdvfs_cmd;

#define SF_HWC_PIXEL_MAX_NORMAL  (1920 * 1080 * 7)
struct MTK_SMI_BWC_MM_INFO g_bwc_mm_info = {
	0, 0, {0, 0}, {0, 0}, {0, 0}, {0, 0}, 0, 0, 0,
	SF_HWC_PIXEL_MAX_NORMAL
};

struct mmdvfs_context_struct {
	spinlock_t scen_lock;
	int is_mmdvfs_start;
	int is_vp_high_fps_enable;
	int is_mhl_enable;
	int is_wfd_enable;
	int is_mjc_enable;
	int is_boost_disable;
};

/* mmdvfs_query() return value, remember to sync with user space */
enum mmdvfs_step_enum {
	MMDVFS_STEP_LOW = 0, MMDVFS_STEP_HIGH,
	MMDVFS_STEP_LOW2LOW, /* LOW */
	MMDVFS_STEP_HIGH2LOW, /* LOW */
	MMDVFS_STEP_LOW2HIGH, /* HIGH */
	MMDVFS_STEP_HIGH2HIGH,
/* HIGH */
};

static unsigned int disable_mmdvfs;
static unsigned int disable_freq_hopping = 1;
static unsigned int disable_freq_mux;
static unsigned int force_max_mmsys_clk;
static unsigned int force_always_on_mm_clks_mask;
static unsigned int clk_mux_mask = 0xFFFF;

static struct mmdvfs_context_struct g_mmdvfs_mgr_cntx;
static struct mmdvfs_context_struct * const g_mmdvfs_mgr = &g_mmdvfs_mgr_cntx;

int get_mmdvfs_clk_mux_mask(void)
{
	return clk_mux_mask;
}

static int mmdvfs_get_default_step(void)
{
	return MMDVFS_FINE_STEP_UNREQUEST;
}

s32 mmdvfs_get_current_fine_step(void)
{
	return g_mmdvfs_current_step;
}

static int mmdvfs_determine_fine_step(struct mmdvfs_adaptor *adaptor,
	int scenario, int sensor_size, int feature_flag, int sensor_fps,
	int codec_width, int codec_height, int preview_size)
{

	int mmdvfs_fine_step = MMDVFS_FINE_STEP_UNREQUEST;

	struct mmdvfs_cam_property cam_setting = {0, 0, 0, 0};
	struct mmdvfs_video_property codec_setting = {0, 0, 0};

	if (!adaptor) {
		MMDVFSMSG("%s: adaptor is NULL\n", __func__);
		return MMDVFS_FINE_STEP_UNREQUEST;
	}

	cam_setting.sensor_size = sensor_size;
	cam_setting.feature_flag = feature_flag;
	cam_setting.fps = sensor_fps;
	cam_setting.preview_size = preview_size;

	codec_setting.width = codec_width;
	codec_setting.height = codec_height;

	mmdvfs_fine_step = adaptor->determine_mmdvfs_step(adaptor,
		scenario, &cam_setting, &codec_setting);

	return mmdvfs_fine_step;

}

static int mmdvfs_determine_fine_step_default(int scenario, int sensor_size,
	int feature_flag, int sensor_fps, int codec_width,
	int codec_height, int preview_size)
{

	return mmdvfs_determine_fine_step(g_mmdvfs_adaptor,
		scenario, sensor_size, feature_flag, sensor_fps,
		codec_width, codec_height, preview_size);
}

static int mmdvfs_query(enum MTK_SMI_BWC_SCEN scenario,
struct MTK_MMDVFS_CMD *cmd)
{
	int sensor_size = 0;
	int camera_mode = 0;
	int sensor_fps = 0;
	int preview_size = 0;

		/* use default info */
	if (cmd == NULL) {
		MMDVFSMSG("%s: cmd is NULL, scen=%d\n", __func__, scenario);
	} else {
		sensor_size = cmd->sensor_size;
		camera_mode = cmd->camera_mode;
		sensor_fps = cmd->sensor_fps;
		preview_size = cmd->preview_size;
	}

	return mmdvfs_determine_fine_step_default(scenario, sensor_size,
		camera_mode, sensor_fps,
		g_mmdvfs_info->video_record_size[0],
		g_mmdvfs_info->video_record_size[1], preview_size);
}


int mmdvfs_get_stable_isp_clk(void)
{
	int legacy_mm_step = MMSYS_CLK_LOW;
	int cam_clk_opp = g_mmdvfs_step_util->get_clients_clk_opp(
		g_mmdvfs_step_util, g_mmdvfs_adaptor,
		LEGACY_CAM_SCENS, MMDVFS_CLK_MUX_TOP_CAM_SEL);

	if (cam_clk_opp != -1)
		legacy_mm_step =
		g_mmdvfs_step_util->get_legacy_mmclk_step_from_mmclk_opp(
		g_mmdvfs_step_util, cam_clk_opp);

	if (legacy_mm_step < 0 || legacy_mm_step >= MMDVFS_MMSYS_CLK_COUNT) {
		MMDVFSDEBUG(3, "invalid legacy mmclk return:%d\n",
		legacy_mm_step);
		legacy_mm_step = MMSYS_CLK_LOW;
	}

	return legacy_mm_step;
}

static void mmdvfs_update_cmd(struct MTK_MMDVFS_CMD *cmd)
{
	if (cmd == NULL)
		return;

	if (cmd->sensor_size)
		g_mmdvfs_cmd.sensor_size = cmd->sensor_size;

	if (cmd->sensor_fps)
		g_mmdvfs_cmd.sensor_fps = cmd->sensor_fps;

	/* MMDVFSMSG("update cm %d\n", cmd->camera_mode); */
	g_mmdvfs_cmd.camera_mode = cmd->camera_mode;

	/* MMDVFSMSG("update preview_size %d\n", cmd->preview_size); */
	g_mmdvfs_cmd.preview_size = cmd->preview_size;
}

/* delay 4 seconds to go LPM to workaround camera ZSD + PIP issue */
static void mmdvfs_cam_work_handler(struct work_struct *work)
{
	/* MMDVFSMSG("CAM handler %d\n", jiffies_to_msecs(jiffies)); */
	mmdvfs_set_fine_step(MMDVFS_CAM_MON_SCEN, MMDVFS_FINE_STEP_UNREQUEST);

}
static DECLARE_DELAYED_WORK(g_mmdvfs_cam_work, mmdvfs_cam_work_handler);

static void mmdvfs_stop_cam_monitor(void)
{
	cancel_delayed_work_sync(&g_mmdvfs_cam_work);
}

static void mmdvfs_start_cam_monitor(int scen, int delay_hz)
{
	if (g_mmdvfs_mgr->is_boost_disable) {
		MMDVFSMSG("MMDVFS boost (cam monitor) is disabled!!\n");
		return;
	}

	mmdvfs_stop_cam_monitor();

	/* Set to MAX clk opp here */
	mmdvfs_set_fine_step(MMDVFS_CAM_MON_SCEN, MMDVFS_FINE_STEP_OPP0);
	schedule_delayed_work(&g_mmdvfs_cam_work, delay_hz * HZ);
}


int mmdvfs_set_corse_step(int scenario, enum mmdvfs_voltage_enum step)
{
	int fine_step_opp __maybe_unused = MMDVFS_FINE_STEP_UNREQUEST;

	if (step == MMDVFS_VOLTAGE_HIGH)
		fine_step_opp = MMDVFS_FINE_STEP_OPP0;
	else
		fine_step_opp = MMDVFS_FINE_STEP_UNREQUEST;

	mmdvfs_set_fine_step(scenario, step);
	return 0;
}

/* The legacy set step function can only set corse step (HPM/ NON-HPM)*/
int mmdvfs_set_step(enum MTK_SMI_BWC_SCEN scenario,
	enum mmdvfs_voltage_enum step)
{
	return mmdvfs_set_corse_step(scenario, step);
}
EXPORT_SYMBOL_GPL(mmdvfs_set_step);

int mmdvfs_internal_set_fine_step(const char *adaptor_name,
	struct mmdvfs_adaptor *adaptor, struct mmdvfs_step_util *step_util,
	enum MTK_SMI_BWC_SCEN smi_scenario,
	int mmdvfs_step, int notify_clk_change)
{
	int original_step = 0;
	int final_step = MMDVFS_FINE_STEP_UNREQUEST;
	int legacy_clk = -1;

	if (!adaptor) {
		MMDVFSMSG("%s: adaptor is NULL\n", __func__);
		return -1;
	}

	if (smi_scenario >= (enum MTK_SMI_BWC_SCEN)MMDVFS_SCEN_COUNT) {
		MMDVFSERR("invalid scenario\n");
		return -1;
	}

	spin_lock(&g_mmdvfs_mgr->scen_lock);
	original_step = g_mmdvfs_current_step;
	final_step = step_util->set_step(
		step_util, mmdvfs_step, smi_scenario);
	g_mmdvfs_current_step = final_step;
	spin_unlock(&g_mmdvfs_mgr->scen_lock);

	/* Change HW configuration */
#ifdef MMDVFS_QOS_SUPPORT
	mmdvfs_qos_update(step_util, final_step);
#else
	adaptor->apply_hw_configurtion_by_step(
		adaptor, final_step, original_step);
#endif

	if (notify_clk_change)
		notify_camsys_clk_change(original_step, final_step);

	legacy_clk = mmdvfs_get_stable_isp_clk();

	if (((*g_mmdvfs_scen_log_mask) == (1 << MMDVFS_SCEN_COUNT)
		&& original_step == final_step) ||
		((1 << smi_scenario) & (*g_mmdvfs_scen_log_mask))) {
		MMDVFSDEBUG(3,
		"%s,set scen:(%d,0x%x)step:(%d,%d,0x%x,0x%x,0x%x,0x%x)\n",
		adaptor_name, smi_scenario, g_mmdvfs_concurrency,
		mmdvfs_step, final_step,
		step_util->mmdvfs_concurrency_of_opps[0],
		step_util->mmdvfs_concurrency_of_opps[1],
		step_util->mmdvfs_concurrency_of_opps[2],
		step_util->mmdvfs_concurrency_of_opps[3]);
		MMDVFSDEBUG(3,
		"%s,C(%d,%d,0x%x,%d),I(%d,%d),CLK:%d\n",
		adaptor_name,
		g_mmdvfs_cmd.sensor_size, g_mmdvfs_cmd.sensor_fps,
		g_mmdvfs_cmd.camera_mode, g_mmdvfs_cmd.preview_size,
		g_mmdvfs_info->video_record_size[0],
		g_mmdvfs_info->video_record_size[1], legacy_clk);
	} else {
		MMDVFSMSG(
		"%s,set scen:(%d,0x%x)step:(%d,%d,0x%x,0x%x,0x%x,0x%x)\n",
		adaptor_name, smi_scenario, g_mmdvfs_concurrency,
		mmdvfs_step, final_step,
		step_util->mmdvfs_concurrency_of_opps[0],
		step_util->mmdvfs_concurrency_of_opps[1],
		step_util->mmdvfs_concurrency_of_opps[2],
		step_util->mmdvfs_concurrency_of_opps[3]);
		MMDVFSMSG("%s,C(%d,%d,0x%x,%d),I(%d,%d),CLK:%d\n",
		adaptor_name,
		g_mmdvfs_cmd.sensor_size, g_mmdvfs_cmd.sensor_fps,
		g_mmdvfs_cmd.camera_mode, g_mmdvfs_cmd.preview_size,
		g_mmdvfs_info->video_record_size[0],
		g_mmdvfs_info->video_record_size[1], legacy_clk);
	}
	return 0;
}


int mmdvfs_internal_set_fine_step_default(
	enum MTK_SMI_BWC_SCEN smi_scenario, int mmdvfs_step)
{
	return mmdvfs_internal_set_fine_step("Fixed",
	g_mmdvfs_adaptor, g_mmdvfs_step_util,
	smi_scenario, mmdvfs_step, 1);
}


void mmdvfs_internal_notify_vcore_calibration(
	struct mmdvfs_prepare_action_event *event)
{
	if (mmdvfs_get_mmdvfs_profile() == MMDVFS_PROFILE_VIN) {
		MMDVFSMSG("calibration event is not hanlded\n");
		g_mmdvfs_mgr->is_mmdvfs_start = 1;
		return;
	}
	if (event->event_type == MMDVFS_EVENT_PREPARE_CALIBRATION_START) {
		g_mmdvfs_mgr->is_mmdvfs_start = 0;
#ifdef MMDVFS_QOS_SUPPORT
		mmdvfs_qos_enable(false);
#endif
		MMDVFSMSG("mmdvfs service is disabled for calibration\n");
	} else if (event->event_type ==
		MMDVFS_EVENT_PREPARE_CALIBRATION_END) {
		g_mmdvfs_mgr->is_mmdvfs_start = 1;
#ifdef MMDVFS_QOS_SUPPORT
		mmdvfs_qos_enable(true);
#endif
		MMDVFSMSG("mmdvfs service has been enabled\n");
	} else {
		MMDVFSMSG("calibration: unknown status code:%d\n",
		event->event_type);
	}
}

int mmdvfs_set_fine_step_force(enum MTK_SMI_BWC_SCEN smi_scenario,
	int mmdvfs_step)
{
	int ret = 0;

	if (!g_mmdvfs_adaptor) {
		MMDVFSMSG("step_force: g_mmdvfs_adaptor is NULL\n");
		return -1;
	}

	/* Update HW runtime option */
	g_mmdvfs_adaptor->enable_vcore = 1;
	g_mmdvfs_adaptor->enable_clk_mux = 1;
	ret = mmdvfs_internal_set_fine_step_default(
		smi_scenario, mmdvfs_step);

	/* recover the original setting */
	g_mmdvfs_adaptor->enable_vcore = !disable_mmdvfs;
	g_mmdvfs_adaptor->enable_clk_mux = !disable_freq_mux;

	return ret;
}

int mmdvfs_set_fine_step(enum MTK_SMI_BWC_SCEN smi_scenario, int mmdvfs_step)
{
	if (disable_mmdvfs || g_mmdvfs_mgr->is_mmdvfs_start == 0) {
		MMDVFSMSG("MMDVFS request denalied; scen:%d, step:%d\n",
		smi_scenario, mmdvfs_step);
		return 0;
	}

	if (!g_mmdvfs_adaptor) {
		MMDVFSMSG("fine_step: g_mmdvfs_adaptor is NULL\n");
		return -1;
	}

	/* Update HW runtime option */
	g_mmdvfs_adaptor->enable_vcore = !disable_mmdvfs;
	g_mmdvfs_adaptor->enable_pll_hopping = !disable_freq_hopping;
	g_mmdvfs_adaptor->enable_clk_mux = !disable_freq_mux;

	return mmdvfs_internal_set_fine_step_default(
		smi_scenario, mmdvfs_step);

}
EXPORT_SYMBOL_GPL(mmdvfs_set_fine_step);

int mmdvfs_set_fine_step_non_force(
	enum MTK_SMI_BWC_SCEN smi_scenario, int mmdvfs_step)
{
	if (disable_mmdvfs || g_mmdvfs_mgr->is_mmdvfs_start == 0) {
		MMDVFSMSG("MMDVFS request denalied; scen:%d, step:%d\n",
		smi_scenario, mmdvfs_step);
		return 0;
	}

	if (!g_mmdvfs_non_force_adaptor) {
		MMDVFSMSG("step_non_force is disable\n");
		return -1;
	}

	/* Update HW runtime option */
	g_mmdvfs_non_force_adaptor->enable_vcore = !disable_mmdvfs;
	g_mmdvfs_non_force_adaptor->enable_pll_hopping = 0;
	g_mmdvfs_non_force_adaptor->enable_clk_mux = 0;

	return mmdvfs_internal_set_fine_step("Auto-Adjust",
		g_mmdvfs_non_force_adaptor,
		g_non_force_step_util, smi_scenario, mmdvfs_step, 0);
}

static int handle_step_mmmclk_set(struct MTK_MMDVFS_CMD *cmd)
{
	int mmdvfs_step_request = 0;
	int mmclk_request = 0;

	if (cmd == NULL) {
		MMDVFSMSG("step ioctl cmd can't be NULL\n");
		return -1;
	}

	if (!g_mmdvfs_adaptor) {
		MMDVFSMSG("mmmclk_set: g_mmdvfs_adaptor is NULL\n");
		return -1;
	}

	/* Get step from the command (bit 0-7) */
	mmdvfs_step_request = cmd->step & MMDVFS_IOCTL_CMD_STEP_FIELD_MASK;
	if (mmdvfs_step_request == MMDVFS_IOCTL_CMD_STEP_FIELD_MASK)
		mmdvfs_step_request = -1;
	/* Get clk from the command (bit 8-15) */
	mmclk_request = (cmd->step & MMDVFS_IOCTL_CMD_MMCLK_FIELD_MASK)
		>> MMDVFS_IOCTL_CMD_STEP_FIELD_LEN;

	if (mmdvfs_step_request < MMDVFS_FINE_STEP_UNREQUEST ||
		mmdvfs_step_request >= g_mmdvfs_adaptor->step_num) {
		MMDVFSMSG("invalid step (%d)\n", mmdvfs_step_request);
	} else {
		MMDVFSMSG("Request step=%d mmclk %d is ignaored\n",
		mmdvfs_step_request, mmclk_request);
		mmdvfs_set_fine_step(cmd->scen, mmdvfs_step_request);
	}
	return 0;
}

static void mmdvfs_handle_vpu_dvfs_set_cmd(struct MTK_MMDVFS_CMD *cmd)
{
	/* Check if the scenrio is normal for VPU here */
	if (cmd->scen != SMI_BWC_SCEN_NORMAL) {
		int current_vpu_step_config = 0;
		int update_vpu_step = 0;
		int result = -1;

		spin_lock(&g_mmdvfs_mgr->scen_lock);
		current_vpu_step_config = g_mmdvfs_current_vpu_step;
		spin_unlock(&g_mmdvfs_mgr->scen_lock);

		if (cmd->camera_mode == MMDVFS_IOCTL_CMD_VPU_STEP_UNREQUEST)
			update_vpu_step = -1;
		else
			update_vpu_step = cmd->camera_mode;

		result = mmdvfs_internal_set_vpu_step(
			current_vpu_step_config, update_vpu_step);

		if (result)	{
			MMDVFSMSG("vpu_step failed: %d, req:%d, current:%d\n",
			result, update_vpu_step, current_vpu_step_config);
			cmd->ret = -1;
		}	else	{
			spin_lock(&g_mmdvfs_mgr->scen_lock);
			g_mmdvfs_current_vpu_step = update_vpu_step;
			spin_unlock(&g_mmdvfs_mgr->scen_lock);
			cmd->ret = 0;
		}
	} else {
		MMDVFSMSG("vpu_dvfs_set_cmd must with normal scenario\n");
	}
}

static void mmdvfs_handle_vpu_dvfs_get_cmd(struct MTK_MMDVFS_CMD *cmd)
{
	cmd->ret = g_mmdvfs_current_vpu_step;
}

void mmdvfs_handle_cmd(struct MTK_MMDVFS_CMD *cmd)
{
	if (disable_mmdvfs) {
		MMDVFSMSG("MMDVFS is disabled\n");
		return;
	}

	/* MMDVFSMSG("MMDVFS handle cmd %u s %d\n", cmd->type, cmd->scen); */

	switch (cmd->type) {
	case MTK_MMDVFS_CMD_TYPE_SET:
		/* save cmd */
		mmdvfs_update_cmd(cmd);
		if (!(g_mmdvfs_concurrency & (1 << cmd->scen))) {
			MMDVFSMSG("invalid set scen %d\n", cmd->scen);
			cmd->ret = -1;
		} else {
			/* determine the step and apply the HW setting */
			MMDVFSMSG("mmdvfs_set_fine_step\n");
			cmd->ret = mmdvfs_set_fine_step(
				cmd->scen, mmdvfs_query(cmd->scen, cmd));
		}
		break;

	case MTK_MMDVFS_CMD_TYPE_QUERY:  /* query with some parameters */
		{
			int step = mmdvfs_query(cmd->scen, cmd);
			s32 cur_step =
				mmdvfs_get_current_fine_step();

			/* Compare the step and return the result */
			if (step == cur_step) {
				if (step == MMDVFS_FINE_STEP_UNREQUEST)
					cmd->ret = (u32)MMDVFS_STEP_LOW2LOW;
				else if (step == MMDVFS_FINE_STEP_OPP0)
					cmd->ret = (u32)MMDVFS_STEP_HIGH2HIGH;
			} else if (step > cur_step) {
				if (cur_step == MMDVFS_FINE_STEP_UNREQUEST)
					cmd->ret = (u32)MMDVFS_STEP_LOW2HIGH;
				else
					cmd->ret = (u32)MMDVFS_STEP_HIGH2LOW;
			} else {
				if (step == MMDVFS_FINE_STEP_UNREQUEST)
					cmd->ret = (u32)MMDVFS_STEP_HIGH2LOW;
				else
					cmd->ret = (u32)MMDVFS_STEP_LOW2HIGH;
			}
		}
		break;

	case MTK_MMDVFS_CMD_TYPE_GET:
		{
			cmd->ret = 0;
			/* Put step in the command (bit 0-7) */
			cmd->ret = g_mmdvfs_current_step
				& MMDVFS_IOCTL_CMD_STEP_FIELD_MASK;

			MMDVFSMSG("Current step query result: %d, 0x%x\n",
			g_mmdvfs_current_step, cmd->ret);
		}
		break;

	case MTK_MMDVFS_CMD_TYPE_CONFIG:
		g_mmdvfs_mgr->is_boost_disable = cmd->boost_disable;
		MMDVFSMSG("Config: is_boost_disable=%d\n",
			g_mmdvfs_mgr->is_boost_disable);
		camera_bw_config = cmd->ddr_type;
		MMDVFSMSG("Config: bw_config=0x%08x\n", camera_bw_config);
		break;

	case MTK_MMDVFS_CMD_TYPE_STEP_SET:
		/* Get the target step from step field */
		MMDVFSMSG("MTK_MMDVFS_CMD_TYPE_STEP_SET\n");
		cmd->ret = handle_step_mmmclk_set(cmd);
		break;

	case MTK_MMDVFS_CMD_TYPE_VPU_STEP_SET:
		mmdvfs_handle_vpu_dvfs_set_cmd(cmd);
		break;

	case MTK_MMDVFS_CMD_TYPE_VPU_STEP_GET:
		mmdvfs_handle_vpu_dvfs_get_cmd(cmd);
		break;

	default:
		MMDVFSMSG("invalid mmdvfs cmd\n");
		dump_stack();
		break;
	}
}

#define MMDVFS_CAMERA_BOOST_MASK	((1<<SMI_BWC_SCEN_VR) | \
					(1<<SMI_BWC_SCEN_VR_SLOW) | \
					(1<<SMI_BWC_SCEN_ICFP) | \
					(1<<SMI_BWC_SCEN_VSS) | \
					(1<<SMI_BWC_SCEN_VENC) | \
					(1<<SMI_BWC_SCEN_CAM_PV) | \
					(1<<SMI_BWC_SCEN_CAM_CP))

void mmdvfs_notify_scenario_exit(enum MTK_SMI_BWC_SCEN scen)
{
	if (disable_mmdvfs) {
		MMDVFSMSG("MMDVFS is disabled\n");
		return;
	}

	if (force_max_mmsys_clk) {
		MMDVFSMSG("MMDVFS is always high\n");
		return;
	}

	/* MMDVFSMSG("leave %d\n", scen); */
	if (scen == SMI_BWC_SCEN_WFD)
		g_mmdvfs_mgr->is_wfd_enable = 0;

	if (scen == SMI_BWC_SCEN_VP_HIGH_FPS)
		g_mmdvfs_mgr->is_vp_high_fps_enable = 0;

	/* Boost for ISP related scenario */
	if ((1 << scen & MMDVFS_CAMERA_BOOST_MASK))
		mmdvfs_start_cam_monitor(scen, 8);

	/* If the scenario is defined in disable_auto_control_mask */
	/* mmdvfs_mgr doesn't change the step automatically.       */
	/* The kernel driver of the scenarios will change the step */
	/* by mmdvfs_set_fine_step directly */
	if (g_mmdvfs_adaptor &&
		(!((1 << scen)
		& g_mmdvfs_adaptor->disable_auto_control_mask)))
		mmdvfs_set_fine_step(scen, MMDVFS_FINE_STEP_UNREQUEST);

	if (g_mmdvfs_non_force_adaptor &&
		(!((1 << scen)
		& g_mmdvfs_non_force_adaptor->disable_auto_control_mask)))
		mmdvfs_set_fine_step_non_force(
		scen, MMDVFS_FINE_STEP_UNREQUEST);
	/* reset scenario voltage to default when it exits */
	/* Also force the system to leave low-low mode */

}

void mmdvfs_internal_get_cam_setting(struct mmdvfs_cam_property *cam_setting)
{
	cam_setting->sensor_size = g_mmdvfs_cmd.sensor_size;
	cam_setting->fps = g_mmdvfs_cmd.sensor_fps;
	cam_setting->feature_flag = g_mmdvfs_cmd.camera_mode;
	cam_setting->preview_size = g_mmdvfs_cmd.preview_size;
}

void mmdvfs_notify_scenario_enter(enum MTK_SMI_BWC_SCEN scen)
{
	int mmdvfs_fine_step = MMDVFS_FINE_STEP_UNREQUEST;
	int mmdvfs_fine_step_non_force = MMDVFS_FINE_STEP_UNREQUEST;

	if (disable_mmdvfs) {
		MMDVFSMSG("MMDVFS is disabled\n");
		return;
	}

	if (force_max_mmsys_clk) {
		MMDVFSMSG("MMDVFS is always high\n");
		return;
	}

	/* If the scenario is defined in disable_auto_control_mask */
	/* mmdvfs_mgr doesn't change the step automatically.       */
	/* The kernel driver of the scenarios will change the step */
	/* by mmdvfs_set_fine_step directly */

	if (g_mmdvfs_adaptor &&
		(!((1 << scen)
		& g_mmdvfs_adaptor->disable_auto_control_mask))) {
		mmdvfs_fine_step = mmdvfs_determine_fine_step_default(
		scen, g_mmdvfs_cmd.sensor_size,
		g_mmdvfs_cmd.camera_mode, g_mmdvfs_cmd.sensor_fps,
		g_mmdvfs_info->video_record_size[0],
		g_mmdvfs_info->video_record_size[1],
		g_mmdvfs_cmd.preview_size);

		mmdvfs_set_fine_step(scen, mmdvfs_fine_step);
	}

	if (g_mmdvfs_non_force_adaptor &&
		(!((1 << scen)
		& g_mmdvfs_non_force_adaptor->disable_auto_control_mask))) {
		mmdvfs_fine_step_non_force = mmdvfs_determine_fine_step(
		g_mmdvfs_non_force_adaptor,
		scen, g_mmdvfs_cmd.sensor_size, g_mmdvfs_cmd.camera_mode,
		g_mmdvfs_cmd.sensor_fps,
		g_mmdvfs_info->video_record_size[0],
		g_mmdvfs_info->video_record_size[1],
		g_mmdvfs_cmd.preview_size);
		mmdvfs_set_fine_step_non_force(
			scen, mmdvfs_fine_step_non_force);
	}

	/* Boost for ISP related scenario */
	if ((1 << scen & MMDVFS_CAMERA_BOOST_MASK))
		mmdvfs_start_cam_monitor(scen, 8);

	/* Record the engine status for debugging */
	if (scen == SMI_BWC_SCEN_WFD)
		g_mmdvfs_mgr->is_wfd_enable = 1;

	if (scen == SMI_BWC_SCEN_VP_HIGH_FPS)
		g_mmdvfs_mgr->is_vp_high_fps_enable = 1;
}

void bwc_mm_info_set(int property_id, long val1, long val2)
{

	switch (property_id) {
	case SMI_BWC_INFO_CON_PROFILE:
		g_mmdvfs_info->concurrent_profile = (int)val1;
		break;
	case SMI_BWC_INFO_SENSOR_SIZE:
		g_mmdvfs_info->sensor_size[0] = val1;
		g_mmdvfs_info->sensor_size[1] = val2;
		break;
	case SMI_BWC_INFO_VIDEO_RECORD_SIZE:
		g_mmdvfs_info->video_record_size[0] = val1;
		g_mmdvfs_info->video_record_size[1] = val2;
		break;
	case SMI_BWC_INFO_DISP_SIZE:
		g_mmdvfs_info->display_size[0] = val1;
		g_mmdvfs_info->display_size[1] = val2;
		break;
	case SMI_BWC_INFO_TV_OUT_SIZE:
		g_mmdvfs_info->tv_out_size[0] = val1;
		g_mmdvfs_info->tv_out_size[1] = val2;
		break;
	case SMI_BWC_INFO_FPS:
		g_mmdvfs_info->fps = (int)val1;
		break;
	case SMI_BWC_INFO_VIDEO_ENCODE_CODEC:
		g_mmdvfs_info->video_encode_codec = (int)val1;
		break;
	case SMI_BWC_INFO_VIDEO_DECODE_CODEC:
		g_mmdvfs_info->video_decode_codec = (int)val1;
		break;
	}
}

int set_mm_info_ioctl_wrapper(struct file *pFile,
	unsigned int cmd, unsigned long param)
{
	int ret = 0;
	struct MTK_SMI_BWC_INFO_SET cfg;

	ret = copy_from_user(&cfg, (void *)param,
		sizeof(struct MTK_SMI_BWC_INFO_SET));
	if (ret) {
		MMDVFSMSG("SET copy_to_user failed: %d\n", ret);
		return -EFAULT;
	}
	/* Set the address to the value assigned by user space program */
	bwc_mm_info_set(cfg.property, cfg.value1, cfg.value2);
	return ret;
}


int get_mm_info_ioctl_wrapper(struct file *pFile,
	unsigned int cmd, unsigned long param)
{
	int ret = 0;

	ret = copy_to_user((void *)param, (void *)g_mmdvfs_info,
		sizeof(struct MTK_SMI_BWC_MM_INFO));

	if (ret) {
		MMDVFSMSG("GET copy_to_user failed: %d\n", ret);
		return -EFAULT;
	}
	return ret;
}

/* MMDVFS related clk initialization */
static struct clk *smi_clk_get_by_name(struct device_node *of_node,
	const char *clk_name)
{
	struct clk *clk_ptr = NULL;

	clk_ptr = of_clk_get_by_name(of_node, clk_name);

	if (IS_ERR(clk_ptr)) {
		MMDVFSMSG("Can't get clk_name %s\n", clk_name);
		clk_ptr = NULL;
	}
	return clk_ptr;
}

void mmdvfs_clks_init(struct device_node *of_node)
{
	int i = 0;

	MMDVFSMSG("start %s\n", __func__);
	/* const int mmdvfs_disable_setting = disable_mmdvfs; */
	/* init clk mux of each MM clks*/
	for (i = 0; i < g_mmdvfs_adaptor->mmdvfs_clk_hw_maps_num; i++) {
		/* Get the clk mux desc */
		struct mmdvfs_clk_hw_map *hw_map_ptr =
		g_mmdvfs_adaptor->mmdvfs_clk_hw_maps + i;

		if (hw_map_ptr->config_method != MMDVFS_CLK_CONFIG_NONE) {
			MMDVFSMSG("Init CLK %s\n",
				hw_map_ptr->clk_mux.ccf_name);
			hw_map_ptr->clk_mux.ccf_handle = smi_clk_get_by_name(
				of_node, hw_map_ptr->clk_mux.ccf_name);
		}
	}

	for (i = 0; i < g_mmdvfs_adaptor->mmdvfs_clk_sources_num; i++) {
		MMDVFSMSG("Init CLK %s\n",
			g_mmdvfs_adaptor->mmdvfs_clk_sources[i].ccf_name);
		g_mmdvfs_adaptor->mmdvfs_clk_sources[i].ccf_handle =
		smi_clk_get_by_name(of_node,
			g_mmdvfs_adaptor->mmdvfs_clk_sources[i].ccf_name);
	}

	/* Enanle the MASK for CLK change */
	clk_mux_mask = 0xFFFF;

	MMDVFSMSG("Finish %s\n", __func__);

	/* Set default high berfore MMDVFS feature is enabled, */
	/* Onlye work when force_max_mmsys_clk is enabled */
	mmdvfs_default_start_delayed_setting();
}

void mmdvfs_init(void)
{
	spin_lock_init(&g_mmdvfs_mgr->scen_lock);

	/* set current step as the default step */
	g_mmdvfs_current_step = mmdvfs_get_default_step();

	g_mmdvfs_info = &g_bwc_mm_info;

	mmdvfs_config_util_init();

	if (mmdvfs_get_mmdvfs_profile() == MMDVFS_PROFILE_ALA)
		g_mmdvfs_mgr->is_mmdvfs_start = 1;
	if (mmdvfs_get_mmdvfs_profile() == MMDVFS_PROFILE_BIA)
		g_mmdvfs_mgr->is_mmdvfs_start = 1;
	if (mmdvfs_get_mmdvfs_profile() == MMDVFS_PROFILE_VIN)
		g_mmdvfs_mgr->is_mmdvfs_start = 1;
	if (mmdvfs_get_mmdvfs_profile() == MMDVFS_PROFILE_ZIO)
		g_mmdvfs_mgr->is_mmdvfs_start = 1;
	if (mmdvfs_get_mmdvfs_profile() == MMDVFS_PROFILE_MER)
		g_mmdvfs_mgr->is_mmdvfs_start = 1;
	if (mmdvfs_get_mmdvfs_profile() == MMDVFS_PROFILE_SYL)
		g_mmdvfs_mgr->is_mmdvfs_start = 1;
	if (mmdvfs_get_mmdvfs_profile() == MMDVFS_PROFILE_CAN)
		g_mmdvfs_mgr->is_mmdvfs_start = 1;
	if (mmdvfs_get_mmdvfs_profile() == MMDVFS_PROFILE_CER)
		g_mmdvfs_mgr->is_mmdvfs_start = 1;
}

/* To be implemented */
void mmdvfs_mhl_enable(int enable)
{
	int mmdvfs_fine_step = mmdvfs_determine_fine_step_default(
		MMDVFS_SCEN_MHL, 0, 0, 0, 0, 0, 0);

	g_mmdvfs_mgr->is_mhl_enable = enable;

	if (enable)
		mmdvfs_set_fine_step(MMDVFS_SCEN_MHL, mmdvfs_fine_step);
	else
		mmdvfs_set_fine_step(MMDVFS_SCEN_MHL,
			MMDVFS_FINE_STEP_UNREQUEST);
}

void mmdvfs_mjc_enable(int enable)
{
	g_mmdvfs_mgr->is_mjc_enable = enable;
}

void mmdvfs_notify_scenario_concurrency(unsigned int u4Concurrency)
{
	/*
	 * THIS FUNCTION IS IN SMI SPIN LOCK.
	 */
	g_mmdvfs_concurrency = u4Concurrency;
}

int mmdvfs_is_default_step_need_perf(void)
{
	MMDVFSMSG("need_perf is not supported in this platform\n");
	return 0;
}

/* switch MM CLK callback from VCORE DVFS driver */
void mmdvfs_mm_clock_switch_notify(int is_before, int is_to_high)
{
	MMDVFSMSG("notify is not supported in this platform\n");
}

int register_mmclk_switch_cb(
	clk_switch_cb notify_cb, clk_switch_cb notify_cb_nolock)
{
	MMDVFSMSG("%s is deplicated\n", __func__);
	return 0;
}

/* This desing is only for CLK Mux switch relate flows */
int mmdvfs_notify_mmclk_switch_request(int event)
{
	MMDVFSMSG("switch_request is deplicated: %d\n", event);
	return 0;
}


int mmdvfs_register_mmclk_switch_cb(
	clk_switch_cb notify_cb, int mmdvfs_client_id)
{
	if (mmdvfs_client_id >= 0
		&& mmdvfs_client_id < MMDVFS_CLK_SWITCH_CB_MAX) {
		quick_mmclk_cbs[mmdvfs_client_id] = notify_cb;
	} else {
		MMDVFSMSG("switch register failed: id=%d\n",
			mmdvfs_client_id);
		return 1;
	}
	return 0;
}

static int mmsys_clk_change_notify_checked(clk_switch_cb func,
	int ori_mmsys_clk_mode,
	int update_mmsys_clk_mode, char *msg)
{
	if (func == NULL) {
		MMDVFSMSG("notify_cb_func is NULL %s, (%d,%d)\n",
		msg, ori_mmsys_clk_mode,
		update_mmsys_clk_mode);
	} else {
		func(ori_mmsys_clk_mode, update_mmsys_clk_mode);
		return 1;
	}
	return 0;
}

static void notify_camsys_clk_change(
	int ori_mmdvfs_step, int update_mmdvfs_step)
{
	int i = 0;
	int ori_cam_clk_mode = 0;
	int update_cam_clk_mode = 0;
	char msg[MMDVFS_CLK_SWITCH_CLIENT_MSG_MAX] = "";
	int result;

	if (!g_mmdvfs_adaptor) {
		MMDVFSMSG("clk_change: g_mmdvfs_adaptor is NULL\n");
		return;
	}

	if (ori_mmdvfs_step == -1)
		ori_mmdvfs_step = g_mmdvfs_adaptor->step_num - 1;
	if (update_mmdvfs_step == -1)
		update_mmdvfs_step = g_mmdvfs_adaptor->step_num - 1;

	if (ori_mmdvfs_step < 0
		|| ori_mmdvfs_step >= g_mmdvfs_adaptor->step_num
		|| update_mmdvfs_step < 0
		|| update_mmdvfs_step >= g_mmdvfs_adaptor->step_num) {
		MMDVFSMSG("invalid step change %d --> %d",
			ori_mmdvfs_step, update_mmdvfs_step);
		return;
	}

	ori_cam_clk_mode = g_mmdvfs_adaptor->get_cam_sys_clk(
		g_mmdvfs_adaptor, ori_mmdvfs_step);
	update_cam_clk_mode = g_mmdvfs_adaptor->get_cam_sys_clk(
		g_mmdvfs_adaptor, update_mmdvfs_step);

	for (i = 0; i < MMDVFS_CLK_SWITCH_CB_MAX; i++) {
		result = snprintf(msg,
			MMDVFS_CLK_SWITCH_CLIENT_MSG_MAX, "id=%d", i);
		if (result < 0) {
			MMDVFSMSG("snprint fail for id:%d retult=%d\n",
				i, result);
			continue;
		}

		if (quick_mmclk_cbs[i] != NULL)
			mmsys_clk_change_notify_checked(
			quick_mmclk_cbs[i], ori_cam_clk_mode,
			update_cam_clk_mode, msg);
	}
}

/* Default step configurtion (after boot up) */
static unsigned int delayed_default_step_finished;

void mmdvfs_default_step_set(int default_step)
{
	if (force_max_mmsys_clk) {
		MMDVFSMSG("Forcing max mm clks is enabled\n");
		mmdvfs_set_fine_step_force(MMDVFS_MGR, default_step);
	}

}

static void mmdvfs_default_step_delayed(struct work_struct *work)
{
	if (force_always_on_mm_clks_mask)
		mmdvfs_debug_set_mmdvfs_clks_enabled(1);

	/* Set default high berfore MMDVFS feature is enabled */
	if (force_max_mmsys_clk) {
		mmdvfs_default_step_set(MMDVFS_FINE_STEP_OPP0);
		delayed_default_step_finished = 1;
	}
}

static DECLARE_DELAYED_WORK(g_mmdvfs_set_default_step_delayed,
	mmdvfs_default_step_delayed);

void mmdvfs_default_start_delayed_setting(void)
{
	schedule_delayed_work(&g_mmdvfs_set_default_step_delayed, 60 * HZ);
}

void mmdvfs_default_stop_delayed_setting(void)
{
	/* Set default high berfore MMDVFS feature is enabled */
	if (force_max_mmsys_clk && (delayed_default_step_finished == 0))
		cancel_delayed_work_sync(&g_mmdvfs_set_default_step_delayed);
}

/* Used for MMDVFS before CLK controlling is ready */
static unsigned int mm_clks_enabled;

void mmdvfs_debug_set_mmdvfs_clks_enabled(int clk_enable_request)
{
	int clk_idx = 0;
	int ccf_ret = 0;
	int always_on_mask = force_always_on_mm_clks_mask;
	struct clk *mux;

	if (!g_mmdvfs_adaptor) {
		MMDVFSMSG("clks_enabled: g_mmdvfs_adaptor is NULL\n");
		return;
	}

	if (mm_clks_enabled == 0 && clk_enable_request == 0) {
		MMDVFSMSG("clks_enabled: clk is already disabled\n");
		return;
	}

	if (mm_clks_enabled == 1 && clk_enable_request == 1) {
		MMDVFSMSG("clks_enabled: clk is already enabled\n");
		return;
	}

	for (clk_idx = 0;
		clk_idx < g_mmdvfs_adaptor->mmdvfs_clk_hw_maps_num;
		clk_idx++) {
		/* Get the specific clk descriptor */
		struct mmdvfs_clk_hw_map *map =
			&(g_mmdvfs_adaptor->mmdvfs_clk_hw_maps[clk_idx]);

		if (map->config_method ==
			MMDVFS_CLK_CONFIG_BY_MUX) {
			if (map->clk_mux.ccf_handle == NULL) {
				MMDVFSMSG("handle can't be NULL\n");
				continue;
			}

			/* Check if the clk be always on the configurtion */
			if (!((1 << clk_idx) & always_on_mask))
				continue;

			mux = (struct clk *)map->clk_mux.ccf_handle;
			if (clk_enable_request == 1) {
				MMDVFSMSG("clk_prepare_enable\n");
				ccf_ret =
					clk_prepare_enable(mux);
				if (ccf_ret) {
					MMDVFSMSG("Failed enable clk: %s\n",
						map->clk_mux.ccf_name);
					continue;
				}
			} else {
				MMDVFSMSG("clk_disable_unprepare\n");
				clk_disable_unprepare(mux);
			}
		}
	}
}

int mmdvfs_internal_set_vpu_step(int current_step, int update_step)
{
	/* Fill event object */
	struct mmdvfs_state_change_event evt = {0};
	const struct mmdvfs_vpu_steps_setting *setting;

	if (!g_mmdvfs_vpu_adaptor) {
		MMDVFSMSG("set_vpu_step: adaptor can't be NULL\n");
		return -1;
	}

	setting = g_mmdvfs_vpu_adaptor->get_vpu_setting(
		g_mmdvfs_vpu_adaptor, update_step);

	if (!setting) {
		MMDVFSMSG("get_vpu_setting return NULL for %d\n",
		update_step);
		return -1;
	}

	if (update_step == -1) {
		evt.vcore_vol_step = -1;
		evt.vpu_clk_step = -1;
		evt.vpu_if_clk_step = -1;
		evt.vimvo_vol_step = -1;
	} else {
		evt.vcore_vol_step = setting->mmdvfs_step;
		evt.vpu_clk_step = setting->vpu_clk_step;
		evt.vpu_if_clk_step = setting->vpu_if_clk_step;
		evt.vimvo_vol_step = setting->vimvo_vol_step;
	}

	/* Rising */
	if (current_step == -1 || ((update_step != -1) && (update_step
		< current_step))) {
		MMDVFSDEBUG(3, "Apply MMDVFS setting");
		mmdvfs_set_fine_step(MMDVFS_SCEN_VPU, evt.vcore_vol_step);
		MMDVFSDEBUG(3, "Apply VPU setting:\n");
		mmdvfs_internal_handle_state_change(&evt);
	} else {
		/* Falling */
		MMDVFSDEBUG(3, "Apply VPU setting:\n");
		mmdvfs_internal_handle_state_change(NULL);
		MMDVFSDEBUG(3, "Apply MMDVFS setting");
		mmdvfs_set_fine_step(MMDVFS_SCEN_VPU, evt.vcore_vol_step);
		mmdvfs_internal_handle_state_change(&evt);
	}

	return 0;
}

void dump_mmdvfs_info(void)
{
	MMDVFSMSG("MMDVFS dump: CMD(%d,%d,0x%x,%d),INFO VR(%d,%d),CLK: %d\n",
	g_mmdvfs_cmd.sensor_size, g_mmdvfs_cmd.sensor_fps,
	g_mmdvfs_cmd.camera_mode, g_mmdvfs_cmd.preview_size,
	g_mmdvfs_info->video_record_size[0],
	g_mmdvfs_info->video_record_size[1],
	current_mmsys_clk);
}

void mmdvfs_unit_test_func(void)
{
	MMDVFSMSG("MMDVFS unit test\n");
	mmdvfs_start_cam_monitor(MMDVFS_SCEN_MHL, 8);

}

int mmdvfs_get_mmdvfs_profile(void)
{

	int mmdvfs_profile_id = MMDVFS_PROFILE_UNKNOWN;
	unsigned int segment_code = 0;

#ifdef MMDVFS_USE_APMCU_CLK_MUX_SWITCH
	segment_code = _GET_BITS_VAL_(31 : 25, get_devinfo_with_index(47));
#endif

#if defined(SMI_D1)
	mmdvfs_profile_id = MMDVFS_PROFILE_D1;
	if (segment_code == 0x41 ||	segment_code == 0x42 ||
			segment_code == 0x43 ||	segment_code == 0x49 ||
			segment_code == 0x51)
		mmdvfs_profile_id = MMDVFS_PROFILE_D1_PLUS;
	else
		mmdvfs_profile_id = MMDVFS_PROFILE_D1;
#elif defined(SMI_D2)
	mmdvfs_profile_id = MMDVFS_PROFILE_D2;

	if (segment_code == 0x4B)
		mmdvfs_profile_id = MMDVFS_PROFILE_D2_M_PLUS;
	else if (segment_code == 0x53)
		mmdvfs_profile_id = MMDVFS_PROFILE_D2_P_PLUS;
	else
		mmdvfs_profile_id = MMDVFS_PROFILE_D2;
#elif defined(SMI_D3)
	mmdvfs_profile_id = MMDVFS_PROFILE_D3;
#elif defined(SMI_J)
	mmdvfs_profile_id = MMDVFS_PROFILE_J1;
#elif defined(SMI_EV)
	mmdvfs_profile_id = MMDVFS_PROFILE_E1;
#elif defined(SMI_WHI)
	segment_code = mt_get_chip_sw_ver();
	if (segment_code >= CHIP_SW_VER_02)
		mmdvfs_profile_id = MMDVFS_PROFILE_WHY2;
	else
		mmdvfs_profile_id = MMDVFS_PROFILE_WHY;
#elif defined(SMI_ALA)
	mmdvfs_profile_id = MMDVFS_PROFILE_ALA;
#elif defined(SMI_BIA)
	mmdvfs_profile_id = MMDVFS_PROFILE_BIA;
#elif defined(SMI_VIN)
	mmdvfs_profile_id = MMDVFS_PROFILE_VIN;
#elif defined(SMI_ZIO)
	mmdvfs_profile_id = MMDVFS_PROFILE_ZIO;
#elif defined(SMI_MER)
	mmdvfs_profile_id = MMDVFS_PROFILE_MER;
#elif defined(SMI_SYL)
	mmdvfs_profile_id = MMDVFS_PROFILE_SYL;
#elif defined(SMI_CAN)
	mmdvfs_profile_id = MMDVFS_PROFILE_CAN;
#elif defined(SMI_CER)
	mmdvfs_profile_id = MMDVFS_PROFILE_CER;
#endif

	MMDVFSDEBUG(4, "Segment_code=%d,mmdvfs_profile_id=%d\n", segment_code,
		mmdvfs_profile_id);
	return mmdvfs_profile_id;

}
static unsigned int mmdvfs_debug_level;
static unsigned int mmdvfs_scen_log_mask = 1 << MMDVFS_SCEN_COUNT;

/* Record MMDVFS debug level */
unsigned int *g_mmdvfs_debug_level = &mmdvfs_debug_level;
unsigned int *g_mmdvfs_scen_log_mask = &mmdvfs_scen_log_mask;
module_param_named(mmdvfs_debug_level, mmdvfs_debug_level,
	uint, 0644);
module_param_named(mmdvfs_scen_log_mask, mmdvfs_scen_log_mask,
	uint, 0644);
module_param_named(disable_mmdvfs, disable_mmdvfs,
	uint, 0644);
module_param_named(disable_freq_hopping, disable_freq_hopping,
	uint, 0644);
module_param_named(disable_freq_mux, disable_freq_mux,
	uint, 0644);
module_param_named(force_max_mmsys_clk, force_max_mmsys_clk,
	uint, 0644);
module_param_named(force_always_on_mm_clks_mask, force_always_on_mm_clks_mask,
	uint, 0644);
