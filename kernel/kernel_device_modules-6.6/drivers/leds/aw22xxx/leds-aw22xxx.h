/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef __AW22XXX_H__
#define __AW22XXX_H__

/*********************************************************
 *
 * i2c
 *
 ********************************************************/
#define MAX_I2C_BUFFER_SIZE         65536

#define AW22XXX_FLASH_I2C_WRITES
#define MAX_FLASH_WRITE_BYTE_SIZE   128

#define AWINIC_FW_UPDATE_DELAY

#define OPLUS_FEATURE_MISC_BASIC

/*********************************************************
 *
 * chip info
 *
 ********************************************************/
#define AW22XXX_SRSTR       0x76
#define AW22XXX_SRSTW       0x55
#define AW22118_CHIPID      0x18
#define AW22127_CHIPID      0x27

#define AW22XXX_RGB_MAX     9

enum aw22xxx_chipids {
	AW22XXX = 0,
	AW22118 = 1,
	AW22127 = 2,
};

enum aw22xxx_fw_flags {
	AW22XXX_FLAG_FW_NONE = 0,
	AW22XXX_FLAG_FW_UPDATE = 1,
	AW22XXX_FLAG_FW_OK = 2,
	AW22XXX_FLAG_FW_FAIL = 3,
};

enum aw22xxx_flags {
	AW22XXX_FLAG_NONE = 0,
	AW22XXX_FLAG_SKIP_INTERRUPTS = 1,
};

enum aw22xxx_reg_page {
	AW22XXX_REG_PAGE0 = 0,
	AW22XXX_REG_PAGE1 = 1,
	AW22XXX_REG_PAGE2 = 2,
	AW22XXX_REG_PAGE3 = 3,
	AW22XXX_REG_PAGE4 = 4,
};

enum aw22xxx_dbgctr {
	AW22XXX_DBGCTR_NORMAL = 0,
	AW22XXX_DBGCTR_IRAM = 1,
	AW22XXX_DBGCTR_SFR = 2,
	AW22XXX_DBGCTR_FLASH = 3,
	AW22XXX_DBGCTR_MAX = 4,
};

enum aw22xxx_imax {
	AW22XXX_IMAX_2mA = 8,
	AW22XXX_IMAX_3mA = 0,
	AW22XXX_IMAX_4mA = 9,
	AW22XXX_IMAX_6mA = 1,
	AW22XXX_IMAX_9mA = 2,
	AW22XXX_IMAX_10mA = 11,
	AW22XXX_IMAX_15mA = 3,
	AW22XXX_IMAX_20mA = 12,
	AW22XXX_IMAX_30mA = 4,
	AW22XXX_IMAX_40mA = 14,
	AW22XXX_IMAX_45mA = 5,
	AW22XXX_IMAX_60mA = 6,
	AW22XXX_IMAX_75mA = 7,
};

/*********************************************************
 *
 * cfg
 *
 ********************************************************/
#define AW22XXX_AUDIO_INITGAIN      0x1F


/*********************************************************
 *
 * struct
 *
 ********************************************************/
struct aw22xxx_container {
	unsigned int len;
	unsigned int version;
	unsigned int bist;
	unsigned int key;
	unsigned char data[];
};
#define AW22XXX_CFG_NAME_MAX        64

#ifdef OPLUS_FEATURE_MISC_BASIC
#define GROUPS_LED_NUM_MAX  9
#define GROUPS_NAME_LENGTH_MAX  32
#define MAX_GROUPS 10
#define EFFECT_ID_MUSIC_PLAY 6
#define EFFECT_ID_CHARGE 5
#define DEFAULT_CHECK_PERIOD_MS 2000
#define FIRST_CHECK_DELAY_MS 1000
#define FAULT_REPORT_PERIOD_SECONDS 24*60*60
#define FAULT_REPORT_COUNT_LIMIT 5

#define LED_FAULT_APP_ID         20120
#define LED_FAULT_LOG_TAG        "PSW_BSP_SENSOR"

#ifndef SENSOR_DEVICE_TYPE
#define SENSOR_DEVICE_TYPE	  "10002"
#endif

enum led_fb_event_id {
	LED_WDT_ERR_ID       = 1, /* Watchdog interrupt fault */
	LED_RESET_ERR_ID     = 2, /* Register abnormal reset */
	LED_FUNCPME_ERR_ID   = 3, /* Pattern end interrupt fault */
	LED_I2C_ERR_ID       = 4, /* I2C communication error */
	LED_FW_ERR_ID        = 5, /* Firmware error / BIST check failed */
	LED_GCR_CHECK_ERR_ID = 6, /* GCR register polling check failed */
	MAX_LED_FAULT_EVENT_ID = 7, /* Max event ID */
};

struct led_fb_conf {
	uint16_t event_id;      /* Internal event ID */
	char *fb_field;         /* Specific event name string, e.g., "device_led_wdt_err" */
	char *fb_event_id;      /* Fault category ID, assigned using existing macros like SENSOR_DEVICE_TYPE */
};

enum AW22XXX_CFG_TYPE {
	AW22XXX_CFG_TYPE_DEFAULT = 1,
	AW22XXX_CFG_TYPE_UPDATE_WITH_COLOR = 3,
};

struct aw22xxx_group_info{
	unsigned int  id;
	char name[GROUPS_NAME_LENGTH_MAX];
	bool led_enabled;
	unsigned int  include_led_num;
	unsigned int  include_led_index[GROUPS_LED_NUM_MAX];
	unsigned int  rgb_value[GROUPS_LED_NUM_MAX];
	unsigned int  group_brightness;
	unsigned int  brightness_coefficient[GROUPS_LED_NUM_MAX];
	unsigned int  effect_id;
	unsigned int  effect_num_max;
	char (*cfg_name_array)[AW22XXX_CFG_NAME_MAX];
	struct color_offset_num *color_offsets_array;
};
struct color_offset_num{
	int color_start;
	int color_end;
};
struct color_mapping_table {
	unsigned int input_color;      /* input color */
	unsigned int mapped_colors[AW22XXX_RGB_MAX]; /* mapped colors for LED 0 ~ 8 */
};
#endif
struct aw22xxx {
	struct i2c_client *i2c;
	struct device *dev;
	struct led_classdev cdev;
	struct work_struct brightness_work;
	struct work_struct task_work;
	struct work_struct fw_work;
	struct work_struct cfg_work;
#ifdef AWINIC_FW_UPDATE_DELAY
	struct hrtimer fw_timer;
#endif
	struct mutex cfg_lock;

	int reset_gpio;
	int irq_gpio;

	unsigned char flags;
	unsigned char chipid;
	unsigned char fw_update;
	unsigned char fw_flags;

	unsigned int imax;
	unsigned int fw_version;

	unsigned char task0;
	unsigned char task1;

	unsigned char effect;
	unsigned char cfg;

	unsigned int rgb[AW22XXX_RGB_MAX];
#ifdef OPLUS_FEATURE_MISC_BASIC
	struct regulator *vbled;
	unsigned int vbled_volt;
	int groups_num;
	struct aw22xxx_group_info *group_info_list;
	int group_id;
	struct delayed_work led_standby_work;
	struct delayed_work recovery_work;
	struct work_struct music_work;
	bool led_standby;
	bool exclusion_cache[MAX_GROUPS][MAX_GROUPS];
	unsigned int music_brightness_last;
	bool music_color_set;
	bool music_effect_loaded;
	unsigned int music_current_color[GROUPS_LED_NUM_MAX];
	unsigned int music_min_brightness;
	unsigned int music_max_brightness;
	struct delayed_work abnormal_reset_check_work;
	unsigned int funcpme_irq_count;
	unsigned int wtd_irq_count;
	unsigned int abnormal_reset_check_count;
	bool abnormal_reset_check_enabled;
	unsigned int check_period;
	unsigned int back_color;
	bool backcolor_support;
	bool color_mapping_support;
	bool trace_fault_support;
	unsigned long fault_report_period_start[MAX_LED_FAULT_EVENT_ID];
	unsigned int fault_report_count[MAX_LED_FAULT_EVENT_ID];
	unsigned char reg_gcr;
	unsigned char reg_mcuctr;
	unsigned char reg_pst;
	unsigned char reg_intst;
	unsigned char reg_imax;
	struct work_struct fault_report_work;
	int fault_event_id;
#endif
};
#endif
