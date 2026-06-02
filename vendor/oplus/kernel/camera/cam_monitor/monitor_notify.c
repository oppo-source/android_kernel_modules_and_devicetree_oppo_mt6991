
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/kernfs.h>
#include <linux/ktime.h>
#include <linux/mutex.h>
#include "oplus_cam_monitor_define.h"

#define CAM_MONITOR_SYSFS_DIR   "camera_monitor"
#define ATTR_REGULATOR_STATE    "regulator_state"
#define ATTR_CLOCK_STATE        "clock_state"
#define ATTR_GPIO_STATE         "gpio_state"

enum notify_attr_id {
	NOTIFY_ATTR_REGULATOR,
	NOTIFY_ATTR_CLOCK,
	NOTIFY_ATTR_GPIO,
	NOTIFY_ATTR_MAX
};


static struct kernfs_node *cached_monitor_kn;
static struct kobject *cached_kobj;
static struct kernfs_node *cached_item_kn[NOTIFY_ATTR_MAX];
static struct kobj_attribute *cached_attr[NOTIFY_ATTR_MAX];
static DEFINE_MUTEX(notify_cache_lock);

static bool ensure_cache(void)
{
	struct kernfs_node *monitor_kn;
	struct kernfs_node *item_kn;
	static const unsigned char *attr_names[] = {
		(const unsigned char *)ATTR_REGULATOR_STATE,
		(const unsigned char *)ATTR_CLOCK_STATE,
		(const unsigned char *)ATTR_GPIO_STATE,
	};
	int i;

	if (cached_monitor_kn && cached_kobj) {
		return true;
	}

	if (!kernel_kobj || !kernel_kobj->sd) {
		return false;
	}

	monitor_kn = sysfs_get_dirent(kernel_kobj->sd, (const unsigned char *)CAM_MONITOR_SYSFS_DIR);
	if (!monitor_kn) {
		return false;
	}

	for (i = 0; i < NOTIFY_ATTR_MAX; i++) {
		item_kn = sysfs_get_dirent(monitor_kn, attr_names[i]);
		if (!item_kn) {
			while (--i >= 0) {
				kernfs_put(cached_item_kn[i]);
				cached_item_kn[i] = NULL;
			}
			kernfs_put(monitor_kn);
			return false;
		}
		cached_item_kn[i] = item_kn;
		cached_attr[i] = (struct kobj_attribute *)item_kn->priv;
	}

	cached_monitor_kn = monitor_kn;
	cached_kobj = (struct kobject *)monitor_kn->priv;
	return true;
}

static void notify_via_sysfs(enum notify_attr_id attr_id, const void *buf, size_t count)
{
	struct kobj_attribute *attr;

	if (attr_id >= NOTIFY_ATTR_MAX || !buf || count == 0)
		return;

	mutex_lock(&notify_cache_lock);
	if (!ensure_cache()) {
		mutex_unlock(&notify_cache_lock);
		return;
	}

	attr = cached_attr[attr_id];
	if (attr && attr->store && cached_kobj)
		attr->store(cached_kobj, attr, (const char *)buf, count);
	mutex_unlock(&notify_cache_lock);
}

void monitor_notify_cache_clear(void)
{
	int i;

	mutex_lock(&notify_cache_lock);
	if (cached_monitor_kn) {
		for (i = 0; i < NOTIFY_ATTR_MAX; i++) {
			if (cached_item_kn[i]) {
				kernfs_put(cached_item_kn[i]);
				cached_item_kn[i] = NULL;
			}
			cached_attr[i] = NULL;
		}
		kernfs_put(cached_monitor_kn);
		cached_monitor_kn = NULL;
		cached_kobj = NULL;
	}
	mutex_unlock(&notify_cache_lock);
}

void oplus_cam_monitor_regulator_notify(const char *module_name,
	const char *resource_name, int state, int voltage_min, int voltage_max)
{
	struct camera_monitor_regulator_info info = {0};

	if (!module_name || !resource_name)
		return;

	strscpy(info.module_name, module_name, NAME_STRING_MAX_LENGTH);
	strscpy(info.resource_name, resource_name, NAME_STRING_MAX_LENGTH);
	info.state = state;
	info.voltage[0] = voltage_min;
	info.voltage[1] = voltage_max;
	ktime_get_clocktai_ts64(&info.ts);

	notify_via_sysfs(NOTIFY_ATTR_REGULATOR, &info, sizeof(info));
}

void oplus_cam_monitor_clock_notify(const char *module_name,
	const char *resource_name, int state, int value0, int value1)
{
	struct camera_monitor_clock_info info = {0};

	if (!module_name || !resource_name)
		return;

	strscpy(info.module_name, module_name, NAME_STRING_MAX_LENGTH);
	strscpy(info.resource_name, resource_name, NAME_STRING_MAX_LENGTH);
	info.state = state;
	info.value[0] = value0;
	info.value[1] = value1;
	ktime_get_clocktai_ts64(&info.ts);

	notify_via_sysfs(NOTIFY_ATTR_CLOCK, &info, sizeof(info));
}

void oplus_cam_monitor_gpio_notify(const char *module_name,
	const char *resource_name, int state, int value)
{
	struct camera_monitor_gpio_info info = {0};

	if (!module_name || !resource_name)
		return;

	strscpy(info.module_name, module_name, NAME_STRING_MAX_LENGTH);
	strscpy(info.resource_name, resource_name, NAME_STRING_MAX_LENGTH);
	info.state = state;
	info.value = value;
	ktime_get_clocktai_ts64(&info.ts);

	notify_via_sysfs(NOTIFY_ATTR_GPIO, &info, sizeof(info));
}
