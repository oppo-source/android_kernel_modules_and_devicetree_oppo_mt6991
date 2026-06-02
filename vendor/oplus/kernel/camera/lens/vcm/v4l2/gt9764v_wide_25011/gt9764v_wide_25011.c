// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <linux/mutex.h>

#define DRIVER_NAME "gt9764v_wide_25011"

#define LOG_INF(format, args...)                                               \
	pr_info(DRIVER_NAME " [%s] " format, __func__, ##args)

#define GT9764V_NAME                "gt9764v_wide_25011"
#define GT9764V_MAX_FOCUS_POS            1023
#define GT9764V_ORIGIN_FOCUS_POS            512
#define GT9764V_MOVE_STEPS                  100
/*
 * This sets the minimum granularity for the focus positions.
 * A value of 1 gives maximum accuracy for a desired focus position
 */
#define GT9764V_FOCUS_STEPS            1
#define GT9764V_CONTROL_REG            0x02
#define GT9764V_SET_POSITION_ADDR        0x03
#define GT9764V_STATUS_REG                0x05

#define GT9764V_CONTROL_POWER_DOWN        BIT(0)
#define GT9764V_AAC_MODE_EN            BIT(1)

#define GT9764V_CMD_DELAY            0xff
#define GT9764V_CTRL_DELAY_US            5000
/*
 * This acts as the minimum granularity of lens movement.
 * Keep this value power of 2, so the control steps can be
 * uniformly adjusted for gradual lens movement, with desired
 * number of control steps.
 */

#define GT9764V_MOVE_DELAY_US      1000
#define GT9764V_INIT_DELAY_US      1000
#define GT9764V_SET_VOLTAGE        2800000
#define GT9764V_CONTROL_REG_MODE_DC  0x00
#define GT9764V_CONTROL_REG_MODE_SAC 0x02

#define VCM_IOC_POWER_ON         _IO('V', BASE_VIDIOC_PRIVATE + 4)
#define VCM_IOC_POWER_OFF        _IO('V', BASE_VIDIOC_PRIVATE + 5)

struct mode_info {
    int32_t mode;
    int32_t flag;
};
#define VCM_IOC_SET_MODE         _IOWR('V', BASE_VIDIOC_PRIVATE + 7, struct mode_info)

/* gt9764v device structure */
struct gt9764v_device {
    struct v4l2_ctrl_handler ctrls;
    struct v4l2_subdev sd;
    struct v4l2_ctrl *focus;
    struct regulator *vin;
    struct regulator *vdd;
    struct pinctrl *vcamaf_pinctrl;
    struct pinctrl_state *vcamaf_on;
    struct pinctrl_state *vcamaf_off;
    struct mutex af_lock;
    /* active or standby mode */
    bool active;
};

static int gt9764v_set_position_by_step(struct gt9764v_device *gt9764v, u16 position);
static int gt9764v_set_position(struct gt9764v_device *gt9764v, u16 val);


static unsigned int gt9764v_move_step = 50;
static unsigned int gt9764v_move_step_delay = GT9764V_MOVE_DELAY_US;
static u16 current_pos = 0;


static inline struct gt9764v_device *to_gt9764v_vcm(struct v4l2_ctrl *ctrl)
{
    return container_of(ctrl->handler, struct gt9764v_device, ctrls);
}

static inline struct gt9764v_device *sd_to_gt9764v_vcm(struct v4l2_subdev *subdev)
{
    return container_of(subdev, struct gt9764v_device, sd);
}

struct regval_list {
    unsigned char reg_num;
    unsigned char value;
    unsigned char delay;
};

static struct regval_list gt9764v_init_regs[] = {
    {0x02, 0x02, 2},
    {0x06, 0x40, 2},
    {0x07, 0x6F, 2},
    {0x0B, 0x00, 2},
};

static int gt9764v_write_smbus(struct gt9764v_device *gt9764v, unsigned char reg,
                  unsigned char value)
{
    struct i2c_client *client = v4l2_get_subdevdata(&gt9764v->sd);
    int ret = 0;

    if (reg == GT9764V_CMD_DELAY  && value == GT9764V_CMD_DELAY)
        usleep_range(GT9764V_CTRL_DELAY_US,
                 GT9764V_CTRL_DELAY_US + 100);
    else
        ret = i2c_smbus_write_byte_data(client, reg, value);
    return ret;
}

static int gt9764v_write_array(struct gt9764v_device *gt9764v,
                  struct regval_list *vals, u32 len)
{
    unsigned int i;
    int ret;

    for (i = 0; i < len; i++) {
        LOG_INF("Init write [0x%x, 0x%x]", vals[i].reg_num, vals[i].value);
        ret = gt9764v_write_smbus(gt9764v, vals[i].reg_num,
                     vals[i].value);
        if (ret < 0)
            return ret;

        usleep_range(vals[i].delay * GT9764V_INIT_DELAY_US,
                        vals[i].delay * GT9764V_INIT_DELAY_US + 1000);
    }
    return 0;
}

static int gt9764v_set_position_by_step(struct gt9764v_device *gt9764v, u16 position)
{
	int ret, val;
	int diff_dac = 0;
	int nStep_count = 0;
	int i = 0;
	u16 move_step = gt9764v_move_step;
	u16 move_step_delay = gt9764v_move_step_delay;

	if (!gt9764v->active) {
		return 0;
	}

	if(move_step == 0) {
		move_step = 50;
		LOG_INF("%s error step  set to 50\n", __func__);
	}

	diff_dac = position - current_pos;
	nStep_count = (diff_dac < 0 ? (diff_dac*(-1)) : diff_dac) /
		move_step;

	//val = gt9764v->focus->val;
	val = current_pos;

	LOG_INF("%s  gt9764v->focus->val(%d) current_pos(%d)\n", __func__,  gt9764v->focus->val, current_pos);
	LOG_INF("%s move_step(%d) move_step_delay(%d) diff_dac(%d) nStep_count(%d)\n", __func__, move_step, move_step_delay, diff_dac, nStep_count);

	for (i = 0; i < nStep_count; ++i) {
		val += (diff_dac < 0 ? (move_step*(-1)) : move_step);
		ret = gt9764v_set_position(gt9764v, val);
		if (ret) {
			LOG_INF("%s I2C failure: %d", __func__, ret);
			return ret;
		}

		if (move_step_delay) {
			usleep_range(move_step_delay, move_step_delay);
		}
	}

	if(current_pos != position) {
		ret = gt9764v_set_position(gt9764v, position);
		if (ret) {
			LOG_INF("%s I2C failure: %d", __func__, ret);
			return ret;
		}
	}

    return 0;
}

static int gt9764v_set_position(struct gt9764v_device *gt9764v, u16 val)
{
    struct i2c_client *client = v4l2_get_subdevdata(&gt9764v->sd);
    current_pos = val;
    int ret = 0;
    if (!gt9764v->active) {
        pr_info("gt9764v set position is %d, active is %d", val, gt9764v->active);
        return 0;
    }
    if(mutex_trylock(&(gt9764v->af_lock))) {
        ret = i2c_smbus_write_word_data(client, GT9764V_SET_POSITION_ADDR, swab16(val));
        mutex_unlock(&(gt9764v->af_lock));
        pr_info("gt9764v set position is %d", val);
    } else {
        pr_info("%s af_lock is locked, gt9764v set position is  %d",__func__, val);
    }

    return ret;
}

static int gt9764v_release(struct gt9764v_device *gt9764v)
{
    int ret, val;
    int diff_dac = 0;
    int nStep_count = 0;
    int i = 0;
    struct i2c_client *client = v4l2_get_subdevdata(&gt9764v->sd);

    if (!gt9764v->active) {
        return 0;
    }
    LOG_INF("%s +\n", __func__);
    diff_dac = GT9764V_ORIGIN_FOCUS_POS - gt9764v->focus->val;

    nStep_count = (diff_dac < 0 ? (diff_dac*(-1)) : diff_dac) /
        GT9764V_MOVE_STEPS;

    val = gt9764v->focus->val;

    for (i = 0; i < nStep_count; ++i) {
        val += (diff_dac < 0 ? (GT9764V_MOVE_STEPS*(-1)) : GT9764V_MOVE_STEPS);
        ret = gt9764v_set_position(gt9764v, val);
        if (ret) {
            LOG_INF("%s I2C failure: %d",
                __func__, ret);
            return ret;
        }
        usleep_range(GT9764V_MOVE_DELAY_US,
                 GT9764V_MOVE_DELAY_US + 1000);
    }

    // last step to origin
    ret = gt9764v_set_position(gt9764v, GT9764V_ORIGIN_FOCUS_POS);
    if (ret) {
        LOG_INF("%s I2C failure: %d",
                __func__, ret);
        return ret;
    }

    ret = i2c_smbus_write_byte_data(client, GT9764V_CONTROL_REG,
                    GT9764V_CONTROL_POWER_DOWN);
    if (ret)
        return ret;

    gt9764v->active = false;
    usleep_range(5000, 5000 + 1000);
    LOG_INF("%s -\n", __func__);
    return 0;
}

static int gt9764v_init(struct gt9764v_device *gt9764v)
{
    int ret;
    if (gt9764v->active) {
        return 0;
    }
    LOG_INF("%s +\n", __func__);

    ret = gt9764v_write_array(gt9764v, gt9764v_init_regs,
                    ARRAY_SIZE(gt9764v_init_regs));

    gt9764v->active = true;

    ret = gt9764v_set_position(gt9764v, GT9764V_ORIGIN_FOCUS_POS);
    ret = gt9764v_set_position_by_step(gt9764v, gt9764v->focus->val);

    if (ret)
        return ret;

    gt9764v->active = true;
    LOG_INF("%s -\n", __func__);
    return 0;
}

/* Power handling */
static int gt9764v_power_off(struct gt9764v_device *gt9764v)
{
    int ret;

    LOG_INF("%s\n", __func__);

    ret = gt9764v_release(gt9764v);
    if (ret)
        LOG_INF("gt9764v release failed!\n");

    ret = regulator_disable(gt9764v->vin);
    if (ret)
        return ret;

    ret = regulator_disable(gt9764v->vdd);
    if (ret)
        return ret;

    if (gt9764v->vcamaf_pinctrl && gt9764v->vcamaf_off)
        ret = pinctrl_select_state(gt9764v->vcamaf_pinctrl,
                    gt9764v->vcamaf_off);

    return ret;
}

static int gt9764v_power_on(struct gt9764v_device *gt9764v)
{
    int ret;

    LOG_INF("%s\n", __func__);

    regulator_set_voltage(gt9764v->vin, GT9764V_SET_VOLTAGE, GT9764V_SET_VOLTAGE);
    ret = regulator_enable(gt9764v->vin);
    if (ret < 0)
        return ret;

    regulator_set_voltage(gt9764v->vdd, GT9764V_SET_VOLTAGE, GT9764V_SET_VOLTAGE);
    ret = regulator_enable(gt9764v->vdd);
    if (ret < 0)
        return ret;

    if (gt9764v->vcamaf_pinctrl && gt9764v->vcamaf_on)
        ret = pinctrl_select_state(gt9764v->vcamaf_pinctrl,
                    gt9764v->vcamaf_on);

    if (ret < 0)
        return ret;

    /*
     * TODO(b/139784289): Confirm hardware requirements and adjust/remove
     * the delay.
     */
    usleep_range(GT9764V_CTRL_DELAY_US, GT9764V_CTRL_DELAY_US + 100);

    ret = gt9764v_init(gt9764v);

    if (ret < 0)
        goto fail;

    return 0;

fail:
    regulator_disable(gt9764v->vin);
    regulator_disable(gt9764v->vdd);
    if (gt9764v->vcamaf_pinctrl && gt9764v->vcamaf_off) {
        pinctrl_select_state(gt9764v->vcamaf_pinctrl,
                gt9764v->vcamaf_off);
    }

    return ret;
}

static int gt9764v_set_ctrl(struct v4l2_ctrl *ctrl)
{
    int ret = 0;
    struct gt9764v_device *gt9764v = to_gt9764v_vcm(ctrl);
    if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
        ret = gt9764v_set_position(gt9764v, ctrl->val);
        if (ret) {
            LOG_INF("%s I2C failure: %d",
                __func__, ret);
            return ret;
        }
    }
    return 0;
}

static const struct v4l2_ctrl_ops gt9764v_vcm_ctrl_ops = {
    .s_ctrl = gt9764v_set_ctrl,
};

static int gt9764v_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
    int ret;
    struct gt9764v_device *gt9764v = sd_to_gt9764v_vcm(sd);

    LOG_INF("%s\n", __func__);

    ret = gt9764v_power_on(gt9764v);
    if (ret < 0) {
        LOG_INF("%s power on fail, ret = %d",
            __func__, ret);
        return ret;
    }

    return 0;
}

static int gt9764v_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
    struct gt9764v_device *gt9764v = sd_to_gt9764v_vcm(sd);

    LOG_INF("%s\n", __func__);

    gt9764v_power_off(gt9764v);

    return 0;
}

static int gt9764v_vcm_resume(struct gt9764v_device *gt9764v)
{
    int ret = 0;
    ret = gt9764v_init(gt9764v);
    LOG_INF("%s exit stand by mode, active:%d\n", __func__, gt9764v->active);
    return ret;
}

static int gt9764v_vcm_suspend(struct gt9764v_device *gt9764v)
{
    int ret = 0;
    ret = gt9764v_release(gt9764v);
    LOG_INF("%s entry stand by mode, active:%d\n", __func__, gt9764v->active);
    return ret;
}
enum ACTUATOR_MODE {
    ACTUATOR_MODE_INVALID = -1,
    ACTUATOR_MODE_DIRECT = 0,
    ACTUATOR_MODE_LSC = 1,
    ACTUATOR_MODE_SAC2 = 2,
    ACTUATOR_MODE_SAC3 = 3,
    ACTUATOR_MODE_SAC4 = 4,
    ACTUATOR_MODE_SAC5 = 5,
    ACTUATOR_MODE_MAX,
};
static bool gt9764v_is_busy(struct gt9764v_device *gt9764v)
{
    unsigned char busy_reg;
    bool busy_status = false;
    struct i2c_client *client = v4l2_get_subdevdata(&gt9764v->sd);
    busy_reg = i2c_smbus_read_byte_data(client, 0x05);
    pr_info("%s, busy:0x%02x", __func__, busy_reg);
    if (busy_reg < 0)
       return false;
    busy_status = (busy_reg & 0x01) ? true : false;
    return busy_status;
}
static int gt9764v_actuator_mode(struct gt9764v_device *gt9764v,  unsigned char mode)
{
    int32_t flag = false;
    int ret = 0;
    struct i2c_client *client = v4l2_get_subdevdata(&gt9764v->sd);
    mutex_lock(&(gt9764v->af_lock));
    if(!gt9764v_is_busy(gt9764v)) {
        ret = i2c_smbus_write_byte_data(client, GT9764V_CONTROL_REG, mode);
        if(ret < 0) {
            pr_info("%s,Failed to write  GT9764V_CONTROL_REG", __func__);
            flag = false;
        } else {
            flag = true;
        }
    } else {
        flag = false;
        pr_info("%s, busy status", __func__);
    }
    mutex_unlock(&(gt9764v->af_lock));
    pr_info("%s, mode:%d", __func__, mode);
    return flag;
}
static int gt9764v_vcm_set_mode(struct gt9764v_device *gt9764v, void *arg)
{
    struct mode_info *config = (struct mode_info*)arg;
    switch(config->mode) {
        case ACTUATOR_MODE_DIRECT:
            config->flag = gt9764v_actuator_mode(gt9764v, GT9764V_CONTROL_REG_MODE_DC);
            break;
        case ACTUATOR_MODE_SAC3:
            config->flag = gt9764v_actuator_mode(gt9764v, GT9764V_CONTROL_REG_MODE_SAC);
            break;
        default:
            config->flag = 0;
            pr_info("%s Invalid motor mode\n", __func__);
            break;
    }
    pr_info("%s actuator_Mode:%d, updata_flag:%d", __func__, config->mode, config->flag);
    return 0;
}

static long gt9764v_ops_core_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
    int ret = 0;
    struct gt9764v_device *gt9764v = sd_to_gt9764v_vcm(sd);
    //struct i2c_client *client = v4l2_get_subdevdata(&gt9764v->sd);
    if (!gt9764v) {
        pr_err("gt9764v is NULL\n");
        return -ENODEV;
    }
    LOG_INF("%s +++\n", __func__);
    switch (cmd) {
    case VCM_IOC_POWER_ON:
    {
        /* 00:active mode , 10:Standby mode , x1:Sleep mode */
        ret = gt9764v_vcm_resume(gt9764v);
        LOG_INF("%s VCM_IOC_POWER_ON, cmd:%d, ret: %d\n", __func__, cmd, ret);
        break;
    }
    break;
    case VCM_IOC_POWER_OFF:
    {
        ret = gt9764v_vcm_suspend(gt9764v);
        LOG_INF("%s VCM_IOC_POWER_OFF, cmd:%d, ret:%d\n", __func__, cmd, ret);
        break;
    }
    break;
    case VCM_IOC_SET_MODE:
    {
        ret = gt9764v_vcm_set_mode(gt9764v, arg);
        pr_info("%s VCM_IOC_SET_MODE, cmd:%d, ret:%d \n", __func__, cmd, ret);
        break;
    }
    default:
        ret = -ENOIOCTLCMD;
        break;
    }
    LOG_INF("%s ---\n", __func__);
    return ret;
}

static const struct v4l2_subdev_internal_ops gt9764v_int_ops = {
    .open = gt9764v_open,
    .close = gt9764v_close,
};


static struct v4l2_subdev_core_ops gt9764v_ops_core = {
    .ioctl = gt9764v_ops_core_ioctl,
};

static const struct v4l2_subdev_ops gt9764v_ops = {
    .core = &gt9764v_ops_core
};

static void gt9764v_subdev_cleanup(struct gt9764v_device *gt9764v)
{
    v4l2_async_unregister_subdev(&gt9764v->sd);
    v4l2_ctrl_handler_free(&gt9764v->ctrls);
#if defined(CONFIG_MEDIA_CONTROLLER)
    media_entity_cleanup(&gt9764v->sd.entity);
#endif
}

static int gt9764v_init_controls(struct gt9764v_device *gt9764v)
{
    struct v4l2_ctrl_handler *hdl = &gt9764v->ctrls;
    const struct v4l2_ctrl_ops *ops = &gt9764v_vcm_ctrl_ops;

    v4l2_ctrl_handler_init(hdl, 1);
    LOG_INF("%s\n", __func__);
    gt9764v->focus = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
              0, GT9764V_MAX_FOCUS_POS, GT9764V_FOCUS_STEPS, GT9764V_ORIGIN_FOCUS_POS);

    if (hdl->error)
        return hdl->error;

    gt9764v->sd.ctrl_handler = hdl;

    return 0;
}

static int gt9764v_probe(struct i2c_client *client)
{
    struct device *dev = &client->dev;
    struct gt9764v_device *gt9764v;
    int ret;

    LOG_INF("%s\n", __func__);

    gt9764v = devm_kzalloc(dev, sizeof(*gt9764v), GFP_KERNEL);
    if (!gt9764v)
        return -ENOMEM;
    mutex_init(&(gt9764v->af_lock));
    gt9764v->vin = devm_regulator_get(dev, "vin");
    if (IS_ERR(gt9764v->vin)) {
        ret = PTR_ERR(gt9764v->vin);
        if (ret != -EPROBE_DEFER)
            LOG_INF("cannot get vin regulator\n");
        return ret;
    }

    gt9764v->vdd = devm_regulator_get(dev, "vdd");
    if (IS_ERR(gt9764v->vdd)) {
        ret = PTR_ERR(gt9764v->vdd);
        if (ret != -EPROBE_DEFER)
            LOG_INF("cannot get vdd regulator\n");
        return ret;
    }

    gt9764v->vcamaf_pinctrl = devm_pinctrl_get(dev);
    if (IS_ERR(gt9764v->vcamaf_pinctrl)) {
        ret = PTR_ERR(gt9764v->vcamaf_pinctrl);
        gt9764v->vcamaf_pinctrl = NULL;
        LOG_INF("cannot get pinctrl\n");
    } else {
        gt9764v->vcamaf_on = pinctrl_lookup_state(
            gt9764v->vcamaf_pinctrl, "vcamaf_on");

        if (IS_ERR(gt9764v->vcamaf_on)) {
            ret = PTR_ERR(gt9764v->vcamaf_on);
            gt9764v->vcamaf_on = NULL;
            LOG_INF("cannot get vcamaf_on pinctrl\n");
        }

        gt9764v->vcamaf_off = pinctrl_lookup_state(
            gt9764v->vcamaf_pinctrl, "vcamaf_off");

        if (IS_ERR(gt9764v->vcamaf_off)) {
            ret = PTR_ERR(gt9764v->vcamaf_off);
            gt9764v->vcamaf_off = NULL;
            LOG_INF("cannot get vcamaf_off pinctrl\n");
        }
    }

    v4l2_i2c_subdev_init(&gt9764v->sd, client, &gt9764v_ops);
    gt9764v->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
    gt9764v->sd.internal_ops = &gt9764v_int_ops;

    ret = gt9764v_init_controls(gt9764v);
    if (ret)
        goto err_cleanup;

#if defined(CONFIG_MEDIA_CONTROLLER)
    ret = media_entity_pads_init(&gt9764v->sd.entity, 0, NULL);
    if (ret < 0)
        goto err_cleanup;

    gt9764v->sd.entity.function = MEDIA_ENT_F_LENS;
#endif

    ret = v4l2_async_register_subdev(&gt9764v->sd);
    if (ret < 0)
        goto err_cleanup;

    return 0;

err_cleanup:
    gt9764v_subdev_cleanup(gt9764v);
    return ret;
}

static void gt9764v_remove(struct i2c_client *client)
{
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct gt9764v_device *gt9764v = sd_to_gt9764v_vcm(sd);

    LOG_INF("%s\n", __func__);

    gt9764v_subdev_cleanup(gt9764v);

    // return 0;
}

static const struct i2c_device_id gt9764v_id_table[] = {
    { GT9764V_NAME, 0 },
    { },
};
MODULE_DEVICE_TABLE(i2c, gt9764v_id_table);

static const struct of_device_id gt9764v_of_table[] = {
    { .compatible = "oplus,gt9764v_wide_25011" },
    { },
};
MODULE_DEVICE_TABLE(of, gt9764v_of_table);

static struct i2c_driver gt9764v_i2c_driver = {
    .driver = {
        .name = GT9764V_NAME,
        .of_match_table = gt9764v_of_table,
    },
    .probe  = gt9764v_probe,
    .remove = gt9764v_remove,
    .id_table = gt9764v_id_table,
};

module_i2c_driver(gt9764v_i2c_driver);

MODULE_AUTHOR("Dongchun Zhu <dongchun.zhu@mediatek.com>");
MODULE_DESCRIPTION("GT9764V VCM driver");
MODULE_LICENSE("GPL v2");
