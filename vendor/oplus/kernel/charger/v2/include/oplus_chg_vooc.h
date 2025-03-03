#ifndef __OPLUS_CHG_VOOC_H__
#define __OPLUS_CHG_VOOC_H__

#include <linux/version.h>
#include <oplus_mms.h>
#include <oplus_strategy.h>
#include <oplus_hal_vooc.h>

#define FASTCHG_FW_INTERVAL_INIT 1000 /* 1S */

enum vooc_topic_item {
	VOOC_ITEM_ONLINE,
	VOOC_ITEM_VOOC_STARTED,
	VOOC_ITEM_VOOC_CHARGING,
	VOOC_ITEM_SID,
	VOOC_ITEM_ONLINE_KEEP,
	VOOC_ITEM_VOOC_BY_NORMAL_PATH,
	VOOC_ITEM_GET_BCC_MAX_CURR,
	VOOC_ITEM_GET_BCC_MIN_CURR,
	VOOC_ITEM_GET_BCC_STOP_CURR,
	VOOC_ITEM_GET_BCC_TEMP_RANGE,
	VOOC_ITEM_GET_BCC_SVOOC_TYPE,
	VOOC_ITEM_VOOCPHY_BCC_GET_FASTCHG_ING,
	VOOC_ITEM_GET_AFI_CONDITION,
	VOOC_ITEM_BREAK_CODE,
	VOOC_ITEM_TEMP_RANGE,
	VOOC_ITEM_SLOW_CHG_BATT_LIMIT,
	VOOC_ITEM_NORMAL_CONNECT_COUNT_LEVEL,
};

enum {
	VOOC_VERSION_DEFAULT = 0,
	VOOC_VERSION_1_0,
	VOOC_VERSION_2_0,
	VOOC_VERSION_3_0,
	VOOC_VERSION_4_0,
	VOOC_VERSION_5_0, /* optimize into fastchging time */
};

enum {
	BCC_TEMP_RANGE_LITTLE_COLD = 0,/*0 ~ 5*/
	BCC_TEMP_RANGE_COOL, /*5 ~ 12*/
	BCC_TEMP_RANGE_LITTLE_COOL, /*12~16*/
	BCC_TEMP_RANGE_NORMAL_LOW, /*16~25*/
	BCC_TEMP_RANGE_NORMAL_HIGH, /*25~43*/
	BCC_TEMP_RANGE_WARM, /*43-52*/
	BCC_TEMP_RANGE_HOT,
};

enum {
	BCC_SOC_0_TO_50 = 0,
	BCC_SOC_50_TO_75,
	BCC_SOC_75_TO_85,
	BCC_SOC_85_TO_90,
	BCC_SOC_MAX,
};

enum vooc_curr_table_type {
	VOOC_CURR_TABLE_OLD_1_0 = 0,
	VOOC_CURR_TABLE_1_0,
	VOOC_CURR_TABLE_2_0,
	VOOC_CP_CURR_TABLE
};

/*
 * sid format:
 * +--------+--------+--------+--------+
 * | 31  24 | 23   8 | 7    4 | 3    0 |
 * +--------+--------+--------+--------+
 *     |         |       |        |
 *     |         |       |        +---> adapter chg_type
 *     |         |       +------------> adapter type
 *     |         +--------------------> adapter power
 *     +------------------------------> adapter id
 */
#define sid_to_adapter_id(sid) (((sid) >> 24) & 0xff)
#define sid_to_adapter_power(sid) (((sid) >> 8) & 0xffff)
#define sid_to_adapter_type(sid) ((enum oplus_adapter_type)(((sid) >> 4) & 0xf))
#define sid_to_adapter_chg_type(sid) ((enum oplus_adapter_chg_type)((sid) & 0xf))
#define adapter_info_to_sid(id, power, adapter_type, adapter_chg_type) \
	((((id) & 0xff) << 24) | (((power) & 0xffff) << 8) | \
	 (((adapter_type) & 0xf) << 4) | ((adapter_chg_type) & 0xf))

#define ABNORMAL_ADAPTER_BREAK_CHECK_TIME	1500
#define VOOC_TEMP_OVER_COUNTS			2
#define VOOC_TEMP_RANGE_THD			10
#define VOOC_TEMP_RANGE_THD_HIGH		20

enum {
        NO_VOOCPHY = 0,
        ADSP_VOOCPHY,
        AP_SINGLE_CP_VOOCPHY,
        AP_DUAL_CP_VOOCPHY,
        INVALID_VOOCPHY,
};

enum oplus_adapter_type {
	ADAPTER_TYPE_UNKNOWN,
	ADAPTER_TYPE_AC,
	ADAPTER_TYPE_CAR,
	ADAPTER_TYPE_PB,  /* power bank */
};

enum oplus_adapter_chg_type {
	CHARGER_TYPE_UNKNOWN,
	CHARGER_TYPE_NORMAL,
	CHARGER_TYPE_VOOC,
	CHARGER_TYPE_SVOOC,
};

enum oplus_fast_chg_status {
	CHARGER_STATUS_UNKNOWN,
	CHARGER_STATUS_FAST_CHARING,
	CHARGER_STATUS_FAST_TO_WARM,
	CHARGER_STATUS_FAST_TO_NORMAL,
	CHARGER_STATUS_FAST_BTB_TEMP_OVER,
	CHARGER_STATUS_FAST_DUMMY,
	CHARGER_STATUS_SWITCH_TEMP_RANGE,
	CHARGER_STATUS_TIMEOUT_RETRY,
	CHARGER_STATUS_CURR_LIMIT,
};

enum oplus_vooc_limit_level {
	CURR_LIMIT_VOOC_3_6A_SVOOC_2_5A = 0x1,
	CURR_LIMIT_VOOC_2_5A_SVOOC_2_0A,
	CURR_LIMIT_VOOC_3_0A_SVOOC_3_0A,
	CURR_LIMIT_VOOC_4_0A_SVOOC_4_0A,
	CURR_LIMIT_VOOC_5_0A_SVOOC_5_0A,
	CURR_LIMIT_VOOC_6_0A_SVOOC_6_5A,
	CURR_LIMIT_MAX,
};

enum oplus_vooc_limit_level_7bit {
	CURR_LIMIT_7BIT_1_0A = 0x01,
	CURR_LIMIT_7BIT_1_5A,
	CURR_LIMIT_7BIT_2_0A,
	CURR_LIMIT_7BIT_2_5A,
	CURR_LIMIT_7BIT_3_0A,
	CURR_LIMIT_7BIT_3_5A,
	CURR_LIMIT_7BIT_4_0A,
	CURR_LIMIT_7BIT_4_5A,
	CURR_LIMIT_7BIT_5_0A,
	CURR_LIMIT_7BIT_5_5A,
	CURR_LIMIT_7BIT_6_0A,
	CURR_LIMIT_7BIT_6_3A,
	CURR_LIMIT_7BIT_6_5A,
	CURR_LIMIT_7BIT_7_0A,
	CURR_LIMIT_7BIT_7_5A,
	CURR_LIMIT_7BIT_8_0A,
	CURR_LIMIT_7BIT_8_5A,
	CURR_LIMIT_7BIT_9_0A,
	CURR_LIMIT_7BIT_9_5A,
	CURR_LIMIT_7BIT_10_0A,
	CURR_LIMIT_7BIT_10_5A,
	CURR_LIMIT_7BIT_11_0A,
	CURR_LIMIT_7BIT_11_5A,
	CURR_LIMIT_7BIT_12_0A,
	CURR_LIMIT_7BIT_12_5A,
	CURR_LIMIT_7BIT_MAX,
};

enum oplus_cp_limit_level_7bit {
	CP_CURR_LIMIT_7BIT_2_0A = 0x01,
	CP_CURR_LIMIT_7BIT_2_1A,
	CP_CURR_LIMIT_7BIT_2_4A,
	CP_CURR_LIMIT_7BIT_3_0A,
	CP_CURR_LIMIT_7BIT_3_4A,
	CP_CURR_LIMIT_7BIT_4_0A,
	CP_CURR_LIMIT_7BIT_4_4A,
	CP_CURR_LIMIT_7BIT_5_0A,
	CP_CURR_LIMIT_7BIT_5_4A,
	CP_CURR_LIMIT_7BIT_6_0A,
	CP_CURR_LIMIT_7BIT_6_4A,
	CP_CURR_LIMIT_7BIT_7_0A,
	CP_CURR_LIMIT_7BIT_7_4A,
	CP_CURR_LIMIT_7BIT_8_0A,
	CP_CURR_LIMIT_7BIT_9_0A,
	CP_CURR_LIMIT_7BIT_10_0A,
	CP_CURR_LIMIT_7BIT_11_0A,
	CP_CURR_LIMIT_7BIT_12_0A,
	CP_CURR_LIMIT_7BIT_12_6A,
	CP_CURR_LIMIT_7BIT_13_0A,
	CP_CURR_LIMIT_7BIT_14_0A,
	CP_CURR_LIMIT_7BIT_15_0A,
	CP_CURR_LIMIT_7BIT_16_0A,
	CP_CURR_LIMIT_7BIT_17_0A,
	CP_CURR_LIMIT_7BIT_18_0A,
	CP_CURR_LIMIT_7BIT_19_0A,
	CP_CURR_LIMIT_7BIT_20_0A,
	CP_CURR_LIMIT_7BIT_MAX,
};

enum oplus_bat_temp {
	BAT_TEMP_NATURAL = 0,
	BAT_TEMP_LITTLE_COLD,
	BAT_TEMP_COOL,
	BAT_TEMP_LITTLE_COOL,
	BAT_TEMP_LITTLE_COOL_HIGH,
	BAT_TEMP_NORMAL_LOW,
	BAT_TEMP_NORMAL_HIGH,
	BAT_TEMP_WARM,
	BAT_TEMP_EXIT,
};

enum oplus_fastchg_temp_rang {
	FASTCHG_TEMP_RANGE_INIT = 0,
	FASTCHG_TEMP_RANGE_LITTLE_COLD,/*0 ~ 5*/
	FASTCHG_TEMP_RANGE_COOL, /*5 ~ 12*/
	FASTCHG_TEMP_RANGE_LITTLE_COOL, /*12~16*/
	FASTCHG_TEMP_RANGE_LITTLE_COOL_HIGH,
	FASTCHG_TEMP_RANGE_NORMAL_LOW, /*16~25*/
	FASTCHG_TEMP_RANGE_NORMAL_HIGH, /*25~43*/
	FASTCHG_TEMP_RANGE_WARM, /*43-52*/
	FASTCHG_TEMP_RANGE_MAX,
};

enum {
	BCC_BATT_SOC_0_TO_50,
	BCC_BATT_SOC_50_TO_75,
	BCC_BATT_SOC_75_TO_85,
	BCC_BATT_SOC_85_TO_90,
	BCC_BATT_SOC_90_TO_100,
};

enum {
	BATT_BCC_CURVE_TEMP_LITTLE_COLD,
	BATT_BCC_CURVE_TEMP_COOL,
	BATT_BCC_CURVE_TEMP_LITTLE_COOL,
	BATT_BCC_CURVE_TEMP_NORMAL_LOW,
	BATT_BCC_CURVE_TEMP_NORMAL_HIGH,
	BATT_BCC_CURVE_TEMP_WARM,
	BATT_BCC_CURVE_MAX,
};

enum {
	FAST_TEMP_0_TO_50,
	FAST_TEMP_50_TO_120,
	FAST_TEMP_120_TO_200,
	FAST_TEMP_200_TO_350,
	FAST_TEMP_350_TO_430,
	FAST_TEMP_430_TO_530,
	FAST_TEMP_MAX,
};

enum {
	FAST_SOC_0_TO_50,
	FAST_SOC_50_TO_75,
	FAST_SOC_75_TO_85,
	FAST_SOC_85_TO_90,
	FAST_SOC_MAX,
};

enum vooc_project_type{
	VOOC_PROJECT_UNKOWN,
	VOOC_PROJECT_5V4A_5V6A_VOOC = 1,
	VOOC_PROJECT_10V5A_TWO_BAT_SVOOC = 2,
	VOOC_PROJECT_10V6P5A_TWO_BAT_SVOOC = 3,
	VOOC_PROJECT_10V5A_SINGLE_BAT_SVOOC = 4,
	VOOC_PROJECT_11V3A_SINGLE_BAT_SVOOC = 5,
	VOOC_PROJECT_10V6A_SINGLE_BAT_SVOOC = 6,
	VOOC_PROJECT_10V8A_TWO_BAT_SVOOC = 7,
	VOOC_PROJECT_10V10A_TWO_BAT_SVOOC = 8,
	VOOC_PROJECT_20V7P5A_TWO_BAT_SVOOC = 9,
	VOOC_PROJECT_10V6P6A_SINGLE_BAT_SVOOC = 12,
	VOOC_PROJECT_11V6P1A_SINGLE_BAT_SVOOC = 13,
	VOOC_PROJECT_20V6A_TWO_BAT_SVOOC = 14,
	VOOC_PROJECT_11V4A_SINGLE_BAT_SVOOC = 15,
	VOOC_PROJECT_20V12A_TWO_BAT_SVOOC = 16,
	VOOC_PROJECT_200W_SVOOC = 17,
	VOOC_PROJECT_88W_SVOOC = 18,
	VOOC_PROJECT_55W_SVOOC = 19,
	VOOC_PROJECT_125W_SVOOC = 20,
	VOOC_PROJECT_OTHER,
};

struct batt_bcc_curve {
	unsigned int target_volt;
	unsigned int max_ibus;
	unsigned int min_ibus;
	bool exit;
};

#define BATT_BCC_ROW_MAX	13
#define BATT_BCC_COL_MAX	7
#define BATT_BCC_MAX		6

struct batt_bcc_curves {
	struct batt_bcc_curve batt_bcc_curve[BATT_BCC_ROW_MAX];
	unsigned char bcc_curv_num;
};

/* vooc API */
uint32_t oplus_vooc_get_project(struct oplus_mms *topic);
int oplus_vooc_set_project(struct oplus_mms *topic, uint32_t val);
uint32_t oplus_vooc_get_voocphy_support(struct oplus_mms *topic);
void oplus_api_switch_normal_chg(struct oplus_mms *topic);
int oplus_api_vooc_set_reset_sleep(struct oplus_mms *topic);
void oplus_api_vooc_turn_off_fastchg(struct oplus_mms *topic);
int oplus_vooc_current_to_level(struct oplus_mms *topic, int curr);
int oplus_vooc_level_to_current(struct oplus_mms *topic, int level);
int oplus_vooc_get_batt_curve_current(struct oplus_mms *topic);
bool oplus_vooc_get_bcc_support_for_smartchg(struct oplus_mms *topic);
unsigned int oplus_adapter_id_to_sid(struct oplus_mms *topic, unsigned char id);
#endif /* __OPLUS_CHG_VOOC_H__ */
