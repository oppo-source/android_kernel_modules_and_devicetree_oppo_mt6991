/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2025 OPLUS Inc.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_device.h>
#include <linux/uaccess.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/host.h>
#include <linux/delay.h>
#include <linux/thermal.h>
#include <linux/time.h>
#include <linux/timekeeping.h>
#include <linux/regulator/machine.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include "monitor_driver.h"


#define OPLUS_ATTR(_name, _mode, _show, _store) \
	struct kobj_attribute oplus_attr_##_name = __ATTR(_name, _mode, _show, _store)

static int g_log_level = LOG_LEVEL_INF;
static int g_dump_history_data_count = CAM_MONITOR_STATE_HISTORY_MAX;
static struct camera_monitor_data *g_plat_priv;
static char *g_dump_history_data_snapshot;

static void save_monitor_history_data(const char *buf, size_t count, struct timespec64 *ts)
{
	int iterator;
	int copy_count = count < ITEM_INFO_STRING_MAX_LEN ? count : ITEM_INFO_STRING_MAX_LEN;

	if (!g_plat_priv)
		return;

	mutex_lock(&g_plat_priv->history_data_lock);
	INC_INDEX(&g_plat_priv->history_data.head_index, CAM_MONITOR_STATE_HISTORY_MAX, &iterator);
	memset(g_plat_priv->history_data.info[iterator].info, 0, sizeof(g_plat_priv->history_data.info[iterator].info));
	memcpy(g_plat_priv->history_data.info[iterator].info, buf, copy_count);
	g_plat_priv->history_data.info[iterator].timestamp = *ts;
	mutex_unlock(&g_plat_priv->history_data_lock);
}

static void fill_regulator_result_locked(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int len = 0;
	struct monitor_check_result result = {0};
	struct camera_monitor_state_info_node *reg_state = NULL;

	list_for_each_entry(reg_state, &g_plat_priv->regulator_state_list, list)
	{
		if(reg_state->use_count == 0) {
			CAMMON_LOG_INF("regulator_state check pass:[module:%s][power:%s]:use_count:%d",
				reg_state->module_name, reg_state->resource_name, reg_state->use_count);
		} else {
			result.check_result += 1;
			len += scnprintf(result.info + len, SUMMARY_INFO_STRING_MAX_LEN - len, "fail:[%s:%s:%d]\n",
				reg_state->module_name, reg_state->resource_name, reg_state->use_count);
			CAMMON_LOG_ERR("regulator_state check fail:[module:%s][power:%s]:use_count:%d",
				reg_state->module_name, reg_state->resource_name, reg_state->use_count);
		}
	}
	if(0 == result.check_result)
		len += scnprintf(result.info + len, SUMMARY_INFO_STRING_MAX_LEN - len, "%s", "regulator_state check pass\n");
	memcpy(buf, &result, sizeof(result));
}

static void fill_clock_result_locked(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int len = 0;
	struct monitor_check_result result = {0};
	struct camera_monitor_state_info_node *clk_state = NULL;

	list_for_each_entry(clk_state, &g_plat_priv->clock_state_list, list) {
		if(clk_state->use_count == 0) {
			CAMMON_LOG_INF("clock_state check pass:[module:%s][clock:%s]",
					clk_state->module_name, clk_state->resource_name);
		} else {
			result.check_result += 1;
			len += scnprintf(result.info + len, SUMMARY_INFO_STRING_MAX_LEN - len, "fail:[%s:%s:%d]\n",
				clk_state->module_name, clk_state->resource_name, clk_state->use_count);
			CAMMON_LOG_ERR("clock_state check fail:[module:%s][clock:%s]use_count:%d",
				clk_state->module_name, clk_state->resource_name, clk_state->use_count);
		}
	}
	if(0 == result.check_result)
		len += scnprintf(result.info + len, SUMMARY_INFO_STRING_MAX_LEN - len, "%s", "clock_state check pass\n");
	memcpy(buf, &result, sizeof(result));
}

static void fill_gpio_result_locked(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int len = 0;
	struct monitor_check_result result = {0};
	struct camera_monitor_state_info_node *gpio_state = NULL;

	list_for_each_entry(gpio_state, &g_plat_priv->gpio_state_list, list) {
		if(gpio_state->use_count == 0) {
			CAMMON_LOG_INF("gpio_state check pass:[module:%s][gpio:%s]",
					gpio_state->module_name, gpio_state->resource_name);
		} else {
			result.check_result += 1;
			len += scnprintf(result.info + len, SUMMARY_INFO_STRING_MAX_LEN - len, "fail:[%s:%s:%d]\n",
				gpio_state->module_name, gpio_state->resource_name, gpio_state->use_count);
			CAMMON_LOG_ERR("gpio_state check fail:[module:%s][gpio:%s]use_count:%d",
				gpio_state->module_name, gpio_state->resource_name, gpio_state->use_count);
		}
	}
	if(0 == result.check_result)
		len += scnprintf(result.info + len, SUMMARY_INFO_STRING_MAX_LEN - len, "%s", "gpio_state check pass\n");
	memcpy(buf, &result, sizeof(result));
}



static ssize_t regulator_state_show(struct kobject *kobj,
	struct kobj_attribute *attr,
	char *buf)
{
	if (!g_plat_priv)
		return -EIO;
	mutex_lock(&g_plat_priv->regulator_list_lock);
	fill_regulator_result_locked(kobj, attr, buf);
	mutex_unlock(&g_plat_priv->regulator_list_lock);
	return sizeof(struct monitor_check_result);
}

static ssize_t regulator_state_store(struct kobject *kobj,
	struct kobj_attribute *attr,
	const char *buf,
	size_t count)
{
	bool find = false;
	int  len = 0;
	char info[ITEM_INFO_STRING_MAX_LEN] = {0};
	struct camera_monitor_state_info_node *reg_state = NULL;
	struct camera_monitor_regulator_info in_info = {0};

	if(buf && count == sizeof(in_info)) {
		memcpy(&in_info, buf, count);
	} else {
		CAMMON_LOG_ERR("input info is invalid");
		return -EIO;
	}
	if (!g_plat_priv)
		return -EIO;

	len = scnprintf(info, sizeof(info), "[module:%s][regulator:%s]:state:%d, voltage:%d-%d",
		in_info.module_name, in_info.resource_name, in_info.state,
		in_info.voltage[0], in_info.voltage[1]);
	CAMMON_LOG_DBG("check info:%s", info);
	save_monitor_history_data(info, len + 1, &(in_info.ts));
	mutex_lock(&g_plat_priv->regulator_list_lock);
	if(in_info.state == CAM_MONITOR_ENABLE) {
		list_for_each_entry(reg_state, &g_plat_priv->regulator_state_list, list)
		{
			if(!strcmp(reg_state->module_name, in_info.module_name) && !strcmp(reg_state->resource_name, in_info.resource_name)) {
				reg_state->use_count++;
				find = true;
				CAMMON_LOG_INF("increase regulator:[%s:%s], use_count %d", in_info.module_name, in_info.resource_name, reg_state->use_count);
				break;
			}
		}
		if(!find) {
			reg_state = kzalloc(sizeof(*reg_state), GFP_KERNEL);
			if (!reg_state) {
				CAMMON_LOG_ERR("kzalloc fail");
				goto error;
			}
			strscpy(reg_state->module_name, in_info.module_name, NAME_STRING_MAX_LENGTH);
			strscpy(reg_state->resource_name, in_info.resource_name, NAME_STRING_MAX_LENGTH);
			reg_state->use_count++;
			list_add(&reg_state->list, &g_plat_priv->regulator_state_list);
			CAMMON_LOG_INF("add new regulator:[%s:%s], use_count %d", in_info.module_name, in_info.resource_name, reg_state->use_count);
		}
	} else { // disable
		list_for_each_entry(reg_state, &g_plat_priv->regulator_state_list, list)
		{
			if(!strcmp(reg_state->module_name, in_info.module_name) && !strcmp(reg_state->resource_name, in_info.resource_name)) {
				if(reg_state->use_count > 0) {
					reg_state->use_count--;
				}
				find = true;
				CAMMON_LOG_INF("decrease regulator:[%s:%s], use_count %d", in_info.module_name, in_info.resource_name, reg_state->use_count);
				break;
			}
		}
		if (!find)
        	CAMMON_LOG_ERR("decrease regulator not found:[%s:%s]", in_info.module_name, in_info.resource_name);
	}
	mutex_unlock(&g_plat_priv->regulator_list_lock);
	return count;

error:
	mutex_unlock(&g_plat_priv->regulator_list_lock);
	return -EIO;
}

static ssize_t clock_state_show(struct kobject *kobj,
	struct kobj_attribute *attr,
	char *buf)
{
	if (!g_plat_priv)
		return -EIO;
	mutex_lock(&g_plat_priv->clock_list_lock);
	fill_clock_result_locked(kobj, attr, buf);
	mutex_unlock(&g_plat_priv->clock_list_lock);
	return sizeof(struct monitor_check_result);
}

static ssize_t clock_state_store(struct kobject *kobj,
	struct kobj_attribute *attr,
	const char *buf,
	size_t count)
{
	bool find = false;
	int  len  = 0;
	char info[ITEM_INFO_STRING_MAX_LEN] = {0};
	struct camera_monitor_state_info_node *clk_state;
	struct camera_monitor_clock_info in_info = {0};
	if (!g_plat_priv)
		return -EIO;

	if(buf && count == sizeof(in_info)) {
		memcpy(&in_info, buf, count);
	} else {
		CAMMON_LOG_ERR("input info is invalid");
		return -EIO;
	}
	len = scnprintf(info, sizeof(info), "[module:%s][clock:%s]:clock_state:%d, value:%d-%d",
		in_info.module_name, in_info.resource_name, in_info.state, in_info.value[0], in_info.value[1]);
	CAMMON_LOG_DBG("check info:%s", info);
	save_monitor_history_data(info, len + 1, &(in_info.ts));
	mutex_lock(&g_plat_priv->clock_list_lock);
	if(in_info.state == CAM_MONITOR_ENABLE) {
		list_for_each_entry(clk_state, &g_plat_priv->clock_state_list, list)
		{
			if(!strcmp(clk_state->module_name, in_info.module_name) && !strcmp(clk_state->resource_name, in_info.resource_name)) {
				clk_state->use_count++;
				find = true;
				CAMMON_LOG_INF("increase clock:[%s:%s], use_count %d", in_info.module_name, in_info.resource_name, clk_state->use_count);
                break;
			}
		}
		if(!find) {
			clk_state = kzalloc(sizeof(*clk_state), GFP_KERNEL);
			if (!clk_state) {
				CAMMON_LOG_ERR("kzalloc fail");
				goto error;
			}
			strscpy(clk_state->module_name, in_info.module_name, NAME_STRING_MAX_LENGTH);
			strscpy(clk_state->resource_name, in_info.resource_name, NAME_STRING_MAX_LENGTH);
			clk_state->use_count++;
			list_add(&clk_state->list, &g_plat_priv->clock_state_list);
			CAMMON_LOG_INF("add new clock:[%s:%s], use_count %d", in_info.module_name, in_info.resource_name, clk_state->use_count);
		}
	} else { // disable
		list_for_each_entry(clk_state, &g_plat_priv->clock_state_list, list)
		{
			if(!strcmp(clk_state->module_name, in_info.module_name) && !strcmp(clk_state->resource_name, in_info.resource_name)) {
				if(clk_state->use_count > 0) {
					clk_state->use_count--;
				}
				find = true;
				CAMMON_LOG_INF("decrease clock:[%s:%s], use_count %d", in_info.module_name, in_info.resource_name, clk_state->use_count);
                break;
			}
		}
		if (!find) {
        	CAMMON_LOG_ERR("decrease clock not found:[%s:%s]", in_info.module_name, in_info.resource_name);
		}
	}
	mutex_unlock(&g_plat_priv->clock_list_lock);
	return count;

error:
	mutex_unlock(&g_plat_priv->clock_list_lock);
	return -EIO;
}

static ssize_t gpio_state_show(struct kobject *kobj,
	struct kobj_attribute *attr,
	char *buf)
{
	if (!g_plat_priv)
		return -EIO;
	mutex_lock(&g_plat_priv->gpio_list_lock);
	fill_gpio_result_locked(kobj, attr, buf);
	mutex_unlock(&g_plat_priv->gpio_list_lock);
	return sizeof(struct monitor_check_result);
}

static ssize_t gpio_state_store(struct kobject *kobj,
	struct kobj_attribute *attr,
	const char *buf,
	size_t count)
{
	bool find = false;
	int  len  = 0;
	char info[ITEM_INFO_STRING_MAX_LEN] = {0};
	struct camera_monitor_state_info_node *gpio_state;
	struct camera_monitor_gpio_info in_info = {0};

	if(buf && count == sizeof(in_info)) {
		memcpy(&in_info, buf, count);
	} else {
		CAMMON_LOG_ERR("input info is invalid");
		return -EIO;
	}
	if (!g_plat_priv)
		return -EIO;

	len = scnprintf(info, sizeof(info), "[module:%s][gpio:%s]:gpio_state:%d, value:%d",
		in_info.module_name, in_info.resource_name, in_info.state, in_info.value);
	CAMMON_LOG_DBG("check info:%s", info);
	save_monitor_history_data(info, len + 1, &(in_info.ts));
	mutex_lock(&g_plat_priv->gpio_list_lock);
	if(in_info.state == CAM_MONITOR_ENABLE) {
		list_for_each_entry(gpio_state, &g_plat_priv->gpio_state_list, list)
		{
			if(!strcmp(gpio_state->module_name, in_info.module_name) && !strcmp(gpio_state->resource_name, in_info.resource_name)) {
				gpio_state->use_count++;
				find = true;
				CAMMON_LOG_INF("increase gpio:[%s:%s], use_count:%d", in_info.module_name, in_info.resource_name, gpio_state->use_count);
                break;
			}
		}
		if(!find) {
			gpio_state = kzalloc(sizeof(*gpio_state), GFP_KERNEL);
			if (!gpio_state) {
				CAMMON_LOG_ERR("kzalloc fail");
				goto error;
			}
			strscpy(gpio_state->module_name, in_info.module_name, NAME_STRING_MAX_LENGTH);
			strscpy(gpio_state->resource_name, in_info.resource_name, NAME_STRING_MAX_LENGTH);
			gpio_state->use_count++;
			list_add(&gpio_state->list, &g_plat_priv->gpio_state_list);
			CAMMON_LOG_INF("add new gpio:[%s:%s], use_count %d", in_info.module_name, in_info.resource_name, gpio_state->use_count);
		}
	} else { // disable
		list_for_each_entry(gpio_state, &g_plat_priv->gpio_state_list, list)
		{
			if(!strcmp(gpio_state->module_name, in_info.module_name) && !strcmp(gpio_state->resource_name, in_info.resource_name)) {
				if(gpio_state->use_count > 0) {
					gpio_state->use_count--;
				}
				find = true;
				CAMMON_LOG_INF("decrease gpio:[%s:%s], use_count %d", in_info.module_name, in_info.resource_name, gpio_state->use_count);
                break;
			}
		}
		if (!find) {
        	CAMMON_LOG_ERR("decrease gpio not found:[%s:%s]", in_info.module_name, in_info.resource_name);
		}
	}
	mutex_unlock(&g_plat_priv->gpio_list_lock);
	return count;

error:
	mutex_unlock(&g_plat_priv->gpio_list_lock);
	return -EIO;
}

static ssize_t state_show(struct kobject *kobj,
	struct kobj_attribute *attr,
	char *buf)
{
	const size_t sz = sizeof(struct monitor_check_result);

	if (!g_plat_priv)
		return -EIO;
	mutex_lock(&g_plat_priv->regulator_list_lock);
	mutex_lock(&g_plat_priv->clock_list_lock);
	mutex_lock(&g_plat_priv->gpio_list_lock);
	fill_regulator_result_locked(kobj, attr, buf);
	fill_clock_result_locked(kobj, attr, buf + sz);
	fill_gpio_result_locked(kobj, attr, buf + 2 * sz);
	mutex_unlock(&g_plat_priv->gpio_list_lock);
	mutex_unlock(&g_plat_priv->clock_list_lock);
	mutex_unlock(&g_plat_priv->regulator_list_lock);
	return 3 * sz;
}

static ssize_t log_level_show(struct kobject *kobj,
	struct kobj_attribute *attr,
	char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "g_log_level:%d\n", g_log_level);
}

static ssize_t log_level_store(struct kobject *kobj,
	struct kobj_attribute *attr,
	const char *buf,
	size_t count)
{
	int ret;
	ret = sscanf(buf, "%d", &g_log_level);
	if (ret != 1)
		return -EINVAL;

	CAMMON_LOG_INF("set g_log_level:%d", g_log_level);
	return count;
}

static ssize_t dump_count_show(struct kobject *kobj,
	struct kobj_attribute *attr,
	char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "dump_history_data_count:%d\n", g_dump_history_data_count);
}

static ssize_t dump_count_store(struct kobject *kobj,
	struct kobj_attribute *attr,
	const char *buf,
	size_t count)
{
	int ret;
	ret = sscanf(buf, "%d", &g_dump_history_data_count);
	if (ret != 1)
		return -EINVAL;

	CAMMON_LOG_INF("set dump_history_data_count:%d", g_dump_history_data_count);
	return count;
}

static ssize_t dump_history_state_read(struct file *fp, struct kobject *kobj, struct bin_attribute *attr,
	char *buf, loff_t offset, size_t size)
{
	static int len = 0;
	int bufsize = 0;
	int count = 0;
	int head_index = 0;
	int iterator = 0;
	char time_string[32] = {0};
	struct timespec64 ts;
	struct tm tm;

	if (!g_plat_priv)
		return -EIO;

	size = size > PAGE_SIZE ? PAGE_SIZE : size;

	mutex_lock(&g_plat_priv->history_dump_lock);
	if(g_dump_history_data_snapshot == NULL) {
		bufsize = CAM_MONITOR_STATE_HISTORY_MAX * (sizeof(time_string) + ITEM_INFO_STRING_MAX_LEN);
		g_dump_history_data_snapshot = kzalloc(bufsize, GFP_KERNEL);
		if(g_dump_history_data_snapshot == NULL) {
			mutex_unlock(&g_plat_priv->history_dump_lock);
			CAMMON_LOG_ERR("kzalloc fail for g_history_data_snapshot");
			return -ENOMEM;
		}
		GET_INDEX(&(g_plat_priv->history_data.head_index), CAM_MONITOR_STATE_HISTORY_MAX, &head_index);
		CAMMON_LOG_DBG("cur head_index %d", head_index);
		for(iterator = head_index-1; iterator >= 0; iterator--) {
			memset(time_string, 0, sizeof(time_string));
			ts = g_plat_priv->history_data.info[iterator].timestamp;
			time64_to_tm(ts.tv_sec, 0, &tm);
			scnprintf(time_string, sizeof(time_string), "%04ld-%02d-%02d %02d:%02d:%02d.%06ld",
						tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
						tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec / 1000);
			len += scnprintf(g_dump_history_data_snapshot + len, bufsize - len, "[%s]:%s\n",
					time_string, g_plat_priv->history_data.info[iterator].info);
			count++;
			if(count > g_dump_history_data_count || len >= bufsize) {
				break;
			}
		}
		if(iterator == -1) {
			iterator = CAM_MONITOR_STATE_HISTORY_MAX - 1;
			if(g_plat_priv->history_data.info[iterator].info[0]) {
				for(; iterator >= head_index; iterator--) {
					memset(time_string, 0, sizeof(time_string));
					ts = g_plat_priv->history_data.info[iterator].timestamp;
					time64_to_tm(ts.tv_sec, 0, &tm);
					scnprintf(time_string, sizeof(time_string), "%04ld-%02d-%02d %02d:%02d:%02d.%06ld",
								tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
								tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec / 1000);
					len += scnprintf(g_dump_history_data_snapshot + len, bufsize - len, "[%s]:%s\n",
							time_string, g_plat_priv->history_data.info[iterator].info);
					count++;
					if(count > g_dump_history_data_count || len >= bufsize) {
						break;
					}
				}
			}
		}
	}
	if(offset >= len) {
		if(g_dump_history_data_snapshot) {
			len = 0;
			kfree(g_dump_history_data_snapshot);
			g_dump_history_data_snapshot = NULL;
		}
		mutex_unlock(&g_plat_priv->history_dump_lock);
		return 0;
	}
	if(offset + size > len) {
		size = len - offset;
	}
	memcpy(buf, g_dump_history_data_snapshot + offset, size);
	mutex_unlock(&g_plat_priv->history_dump_lock);
	return size;
}

static OPLUS_ATTR(regulator_state, 0644, regulator_state_show, regulator_state_store);
static OPLUS_ATTR(clock_state, 0644, clock_state_show, clock_state_store);
static OPLUS_ATTR(gpio_state, 0644, gpio_state_show, gpio_state_store);
static OPLUS_ATTR(state, 0644, state_show, NULL);
static OPLUS_ATTR(log_level, 0644, log_level_show, log_level_store);
static OPLUS_ATTR(dump_count, 0644, dump_count_show, dump_count_store);
static BIN_ATTR_RO(dump_history_state, 0);

static struct attribute *state_attrs[] = {
	&oplus_attr_regulator_state.attr,
	&oplus_attr_clock_state.attr,
	&oplus_attr_gpio_state.attr,
	&oplus_attr_state.attr,
	NULL,
};

static struct bin_attribute *history_attrs[] = {
	&bin_attr_dump_history_state,
	NULL,
};

static struct attribute *debug_attrs[] = {
	&oplus_attr_log_level.attr,
	&oplus_attr_dump_count.attr,
	NULL,
};

static const struct attribute_group state_group = {
	.attrs = state_attrs,
};

static const struct attribute_group history_group = {
	// .name = "dump_history",
	.bin_attrs = history_attrs,
};

static const struct attribute_group debug_group = {
	.name = "debug",
	.attrs = debug_attrs,
};

static const struct attribute_group *camera_monitor_groups[] = {
	&state_group,
	&history_group,
	&debug_group,
	NULL,
};

static int create_sysfs(void)
{
	int ret = 0;
    if (!g_plat_priv) {
        CAMMON_LOG_ERR("g_plat_priv is NULL, skip create_sysfs");
        return -EINVAL;
    }
	g_plat_priv->kobj = kobject_create_and_add("camera_monitor", kernel_kobj);
	ret = sysfs_create_groups(g_plat_priv->kobj, camera_monitor_groups);
	if (ret) {
		CAMMON_LOG_ERR("failed to create sysfs for state monitor");
		sysfs_remove_groups(g_plat_priv->kobj, camera_monitor_groups);
        kobject_put(g_plat_priv->kobj);
        g_plat_priv->kobj = NULL;
	}
	return ret;
}

static int remove_sysfs(void)
{
    if (!g_plat_priv || !g_plat_priv->kobj) {
        CAMMON_LOG_ERR("g_plat_priv or kobj is NULL, skip remove_sysfs");
        return 0;
    }
	sysfs_remove_groups(g_plat_priv->kobj, camera_monitor_groups);
	kobject_put(g_plat_priv->kobj);
	g_plat_priv->kobj = NULL;
	CAMMON_LOG_INF("remove sysfs for state monitor");
	return 0;
}

static int camera_monitor_open(struct inode *nodp, struct file *filp)
{
	CAMMON_LOG_INF("open");
	return 0;
}

static int camera_monitor_close(struct inode *nodp, struct file *filp)
{
	CAMMON_LOG_INF("close");
	return 0;
}

static long __maybe_unused camera_monitor_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	CAMMON_LOG_INF("ioctl");
	return 0;
}

static const struct file_operations camera_monitor_ops = {
	.owner = THIS_MODULE,
	.open = camera_monitor_open,
	.release = camera_monitor_close,
	// .unlocked_ioctl = camera_monitor_ioctl,
	// .compat_ioctl = camera_monitor_ioctl,
};

static int camera_monitor_probe(struct platform_device *plat_dev)
{
	int ret = 0;
	struct camera_monitor_data *plat_priv;

	CAMMON_LOG_INF("camera_monitor_probe start");
	plat_priv = devm_kzalloc(&plat_dev->dev, sizeof(*plat_priv), GFP_KERNEL);
	if (!plat_priv) {
		CAMMON_LOG_ERR("failed to alloc memory for plat_priv");
		return -ENOMEM;
	}
	g_plat_priv = plat_priv;
	plat_priv->plat_dev = plat_dev;
	platform_set_drvdata(plat_dev, plat_priv);
	plat_priv->dev.minor = MISC_DYNAMIC_MINOR;
	plat_priv->dev.name = "camera_monitor";
	plat_priv->dev.fops = &camera_monitor_ops;
	plat_priv->dev.mode = S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP;
	ret = misc_register(&plat_priv->dev);
	if(ret) {
		CAMMON_LOG_ERR("failed to register misc device");
		goto err;
	}
	atomic64_set(&plat_priv->history_data.head_index, 0);
	INIT_LIST_HEAD(&plat_priv->regulator_state_list);
	INIT_LIST_HEAD(&plat_priv->clock_state_list);
	INIT_LIST_HEAD(&plat_priv->gpio_state_list);
	mutex_init(&plat_priv->regulator_list_lock);
	mutex_init(&plat_priv->clock_list_lock);
	mutex_init(&plat_priv->gpio_list_lock);
	mutex_init(&plat_priv->history_data_lock);
	mutex_init(&plat_priv->history_dump_lock);
    ret = create_sysfs();
    if (ret) {
        CAMMON_LOG_ERR("failed to create sysfs");
        misc_deregister(&plat_priv->dev);
        goto err;
    }

	return 0;
err:
	g_plat_priv = NULL;
	devm_kfree(&plat_dev->dev, plat_priv);
	return ret;
}

static void free_state_list(struct list_head *head, struct mutex *lock)
{
	struct camera_monitor_state_info_node *node, *tmp;

	mutex_lock(lock);
	list_for_each_entry_safe(node, tmp, head, list) {
		list_del(&node->list);
		kfree(node);
	}
	mutex_unlock(lock);
}

static int camera_monitor_remove(struct platform_device *plat_dev)
{
	struct camera_monitor_data *plat_priv = platform_get_drvdata(plat_dev);

	if (!plat_priv)
		return 0;

	remove_sysfs();
	misc_deregister(&plat_priv->dev);

	free_state_list(&plat_priv->regulator_state_list, &plat_priv->regulator_list_lock);
	free_state_list(&plat_priv->clock_state_list, &plat_priv->clock_list_lock);
	free_state_list(&plat_priv->gpio_state_list, &plat_priv->gpio_list_lock);

	mutex_lock(&plat_priv->history_dump_lock);
	if (g_dump_history_data_snapshot) {
		kfree(g_dump_history_data_snapshot);
		g_dump_history_data_snapshot = NULL;
	}
	mutex_unlock(&plat_priv->history_dump_lock);

	g_plat_priv = NULL;
	platform_set_drvdata(plat_dev, NULL);
	devm_kfree(&plat_dev->dev, plat_priv);

	return 0;
}

static const struct of_device_id camera_monitor_of_match_table[] = {
	{.compatible = "oplus,camera_monitor", },
	{ },
};
MODULE_DEVICE_TABLE(of, camera_monitor_of_match_table);

#ifdef CONFIG_PM_SLEEP
static int camera_monitor_suspend(struct device *dev)
{
	CAMMON_LOG_INF("suspend");
	return 0;
}

static int camera_monitor_resume(struct device *dev)
{
	CAMMON_LOG_INF("resume");
	return 0;
}
#endif

static const struct dev_pm_ops camera_monitor_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(camera_monitor_suspend, camera_monitor_resume)
};

static struct platform_driver camera_monitor_platform_driver = {
	.probe  = camera_monitor_probe,
	.remove = camera_monitor_remove,
	.driver = {
		.name = "camera_monitor",
		.of_match_table = camera_monitor_of_match_table,
		.pm = &camera_monitor_pm_ops,
	},
};

/*
 * Module init function
 */
static int __init camera_monitor_init(void)
{
	int ret = 0;
	ret = platform_driver_register(&camera_monitor_platform_driver);
	CAMMON_LOG_INF("camera_monitor_init");

	return ret;
}

/*
 * Module exit function
 */
static void __exit camera_monitor_exit(void)
{
	platform_driver_unregister(&camera_monitor_platform_driver);
	CAMMON_LOG_INF("camera_monitor_exit");
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Camera Monitor Linux Kernel module");

module_init(camera_monitor_init);
module_exit(camera_monitor_exit);
