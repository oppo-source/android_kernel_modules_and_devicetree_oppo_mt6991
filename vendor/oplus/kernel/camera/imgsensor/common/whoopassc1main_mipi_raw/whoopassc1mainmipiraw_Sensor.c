// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 whoopassc1mainmipiraw_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include "whoopassc1mainmipiraw_Sensor.h"

// #define SENSOR_NAME  SENSOR_DRVNAME_WHOOPASSC1MAIN_MIPI_RAW

#define PFX "whoopassc1main_camera_sensor"

#ifdef  EEPROM_WRITE_DATA_MAX_LENGTH
#undef  EEPROM_WRITE_DATA_MAX_LENGTH
#endif
#define EEPROM_WRITE_DATA_MAX_LENGTH          (64)

#define WHOOPASSC1MAIN_STEREO_MW_START_ADDR     (0x4010)
#define WHOOPASSC1MAIN_STEREO_MT_START_ADDR     (0x4800)
#define WHOOPASSC1MAIN_STEREO_MT105_START_ADDR     (0x5000)
#define WHOOPASSC1MAIN_AESYNC_START_ADDR     (0x574b)

// #define  OTP_XTC_ADDR             (0x1A00)
// #define  OTP_XTC_LENGTH           (9216)
// #define  OTP_XTC_VALID_ADDR       (0x3e00)
// #define  OTP_XTC_IS_VALID_VAL     (0x01)

// #define  XTC_SENSOR_ADDR_PART1    (0x5A20)
// #define  XTC_SENSOR_LENGTH_PART1  (0x5A3F - 0x5A20 + 1) //32
// #define  XTC_SENSOR_ADDR_PART2    (0x5AC0)
// #define  XTC_SENSOR_LENGTH_PART2  (0x688F - 0x5AC0 + 1) //3536
// #define  XTC_SENSOR_ADDR_PART3    (0x68AE)
// #define  XTC_SENSOR_LENGTH_PART3  (0x68BD - 0x68AE + 1) //16


#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)
#define WHOOPASSC1MAIN_EEPROM_ADDR   (0xA0)
#define GET_SENSOR_ID_RETRY_CNT  (5)
#define OTP_BG_ADDR   (0x0012)
#define OTP_BARCODE_ADDR   (0x00bc)

static bool bNeedSetNormalMode = FALSE;
static u16 module_flag = 0;
#define OTP_SIZE  (0x8000)
static kal_uint8 otp_data_checksum[OTP_SIZE] = {0};
static struct oplus_eeprom_info_struct  oplus_eeprom_info = {0};

static char* sensor_name[] = {SENSOR_DRVNAME_WHOOPASSC1MAIN_MIPI_RAW, SENSOR_DRVNAME_WHOOPASSC2MAIN2_MIPI_RAW, SENSOR_DRVNAME_WHOOPASSC1MAIN2_MIPI_RAW};
static char* SENSOR_NAME = SENSOR_DRVNAME_WHOOPASSC1MAIN_MIPI_RAW;
//#define SUBDRV_I2C_BUF_SIZE (256)
#define MAX_BURST_LEN  (2048)
static u8 * msg_buf = NULL;

static int group_hold_frame_count = 0;
static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int whoopassc1main_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int whoopassc1main_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int whoopassc1main_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_eeprom_common_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int whoopassc1main_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int whoopassc1main_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int whoopassc1main_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int open(struct subdrv_ctx *ctx);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int get_sensor_temperature(void *arg);
static void whoopassc1main_get_sensor_cali(void* arg);
static void whoopassc1main_set_sensor_cali(void *arg);
static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size);
static int whoopassc1main_set_awb_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int whoopassc1main_i2c_burst_wr_regs_u16(struct subdrv_ctx *ctx, u16 * list, u32 len);
static int adapter_i2c_burst_wr_regs_u16(struct subdrv_ctx * ctx,
		u16 addr, u16 *list, u32 len);
static bool g_id_from_dts_flag = false;
static bool is_c1sensor = false;
static void get_imgsensor_id_from_dts(struct subdrv_ctx *ctx, u32 *sensor_id);

//////gain
static void whoopassc1main_set_gain_convert(struct subdrv_ctx *ctx, u32 gain);
static int whoopassc1main_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void whoopassc1main_set_multi_gain(struct subdrv_ctx *ctx, u32 *gains, u16 exp_cnt);
static void whoopassc1main_set_hdr_tri_gain(struct subdrv_ctx *ctx, u64 *gains, u16 exp_cnt);
static int whoopassc1main_set_hdr_tri_gain2(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int whoopassc1main_set_hdr_tri_gain3(struct subdrv_ctx *ctx, u8 *para, u32 *len);


///////shutter
static void whoopassc1main_set_shutter_convert(struct subdrv_ctx *ctx, u64 shutter);
static int whoopassc1main_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int whoopassc1main_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void whoopassc1main_set_shutter_frame_length_convert(struct subdrv_ctx *ctx, u64 shutter, u32 frame_length);
static void whoopassc1main_set_multi_shutter_frame_length(struct subdrv_ctx *ctx, u64 *shutters, u16 exp_cnt, u16 frame_length);
static int whoopassc1main_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int whoopassc1main_set_hdr_tri_shutter2(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int whoopassc1main_set_hdr_tri_shutter3(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int whoopassc1main_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void whoopassc1main_set_max_framerate_in_lut_by_scenario(struct subdrv_ctx *ctx,
	enum SENSOR_SCENARIO_ID_ENUM scenario_id, u32 framerate);

static void update_CTLE(struct subdrv_ctx *ctx);
static void whoopassc1main_get_sensorname(struct subdrv_ctx *ctx);
//static int whoopassc1main_chk_streaming_st(void *arg);
//static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt, u64 sof_ts);
/* STRUCT */

static struct eeprom_map_info whoopassc1main_eeprom_info[] = {
	{ EEPROM_META_MODULE_ID, 0x0000, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_SENSOR_ID, 0x0006, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_LENS_ID, 0x0008, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_VCM_ID, 0x000A, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_MIRROR_FLIP, 0x000E, 0x0010, 0x0011, 1, true },
	{ EEPROM_META_MODULE_SN, 0x00B0, 0x00C7, 0x00C8, 23, true },
	{ EEPROM_META_AF_CODE, 0x0092, 0x0098, 0x0099, 6, true },
	{ EEPROM_META_AF_FLAG, 0x0098, 0x0098, 0x0099, 1, true },
	{ EEPROM_META_STEREO_DATA, 0, 0, 0, 0, false },
	{ EEPROM_META_STEREO_MW_MAIN_DATA, WHOOPASSC1MAIN_STEREO_MW_START_ADDR, 0xFFFF, 0xFFFF, CALI_DATA_MASTER_LENGTH, true },
	{ EEPROM_META_STEREO_MT_MAIN_DATA, WHOOPASSC1MAIN_STEREO_MT_START_ADDR, 0xFFFF, 0xFFFF, CALI_DATA_MASTER_LENGTH, true },
	{ EEPROM_META_STEREO_MT_MAIN_DATA_105CM, WHOOPASSC1MAIN_STEREO_MT105_START_ADDR, 0xFFFF, 0xFFFF, CALI_DATA_MASTER_LENGTH, true },
};

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, whoopassc1main_set_test_pattern},
	{SENSOR_FEATURE_SEAMLESS_SWITCH, whoopassc1main_seamless_switch},
	{SENSOR_FEATURE_CHECK_SENSOR_ID, whoopassc1main_check_sensor_id},
	{SENSOR_FEATURE_GET_EEPROM_COMDATA, get_eeprom_common_data},
	{SENSOR_FEATURE_SET_SENSOR_OTP, whoopassc1main_set_eeprom_calibration},
	{SENSOR_FEATURE_GET_EEPROM_STEREODATA, whoopassc1main_get_eeprom_calibration},
	{SENSOR_FEATURE_GET_SENSOR_OTP_ALL, whoopassc1main_get_otp_checksum_data},
	{SENSOR_FEATURE_SET_AWB_GAIN, whoopassc1main_set_awb_gain},


	{SENSOR_FEATURE_SET_ESHUTTER, whoopassc1main_set_shutter},
	{SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME, whoopassc1main_set_shutter_frame_length},
	{SENSOR_FEATURE_SET_HDR_SHUTTER, whoopassc1main_set_hdr_tri_shutter2},
	{SENSOR_FEATURE_SET_HDR_TRI_SHUTTER, whoopassc1main_set_hdr_tri_shutter3},
	{SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME, whoopassc1main_set_multi_shutter_frame_length_ctrl},
	// {SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME_IN_LUT, whoopassc1main_set_multi_shutter_frame_length_in_lut},
	{SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO, whoopassc1main_set_max_framerate_by_scenario},

	{SENSOR_FEATURE_SET_GAIN, whoopassc1main_set_gain},
	{SENSOR_FEATURE_SET_DUAL_GAIN, whoopassc1main_set_hdr_tri_gain2},
	{SENSOR_FEATURE_SET_HDR_TRI_GAIN, whoopassc1main_set_hdr_tri_gain3},
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x01B50162,
		.addr_header_id = 0x00000006,
		.i2c_write_id = WHOOPASSC1MAIN_EEPROM_ADDR,
		// XTC/QPDC
		// .pdc_support = TRUE,
		// .pdc_size = OTP_XTC_LENGTH,
 		// .addr_pdc = OTP_XTC_ADDR,
 		// .sensor_reg_addr_pdc = XTC_SENSOR_ADDR_PART1,
	},
};

static struct mtk_sensor_saturation_info imgsensor_saturation_info = {
	.gain_ratio = 1000,
	.OB_pedestal = 64,
	.saturation_level = 1023,
};

static struct mtk_sensor_saturation_info imgsensor_saturation_info_12bit = {
	.gain_ratio = 8000,
	.OB_pedestal = 256,
	.saturation_level = 4095,
};

static struct mtk_sensor_saturation_info imgsensor_saturation_info_14bit = {
	.gain_ratio = 8000,
	.OB_pedestal = 64,
	.saturation_level = 16383,
};


static u32 whoopassc1main_dcg_ratio_table_10bit[] = {8000};
static u32 whoopassc1main_dcg_ratio_table_10bit_2[] = {4000};
static u32 whoopassc1main_dcg_ratio_table_12bit[] = {8000};
static u32 whoopassc1main_dcg_ratio_table_14bit[] = {8000};

#define MAINC1_CTLE_LEVEL 3
#define MAINC1_CTLE_DELAY 12
#define MAINC1_CTLE_EQBW 2
#define MAINC2_CTLE_LEVEL 4
#define MAINC2_CTLE_DELAY 15
#define MAINC2_CTLE_EQBW 3

#define pd_i4Crop { \
	/* pre、 cap、 normal_video、 hs_video、 slim_video */\
	{0, 0}, {0, 0}, {0, 384}, {0, 384}, {0, 192},\
	/* cust1  cust2   cust3   cust4     cust5 */\
	{0, 192}, {0, 0}, {0, 0}, {0, 0}, {2048, 1536},\
	/* cust6  cust7       cust8         cust9      cust10 */\
	{0, 0}, {1184, 888}, {2048, 1536}, {2048, 1536}, {2048, 1536},\
	/* cust11     cust12    cust13    cust14  cust15 */\
	{2048, 1920}, {0, 512}, {0, 384}, {0, 0}, {2048, 1536},\
	/* cust16 cust17 cust18  cust19  cust20 */\
	{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, \
	/*cust21    cust22     cust23    cust24  cust25 */\
	{0, 384}, {0, 384}, {128, 456}, {0, 0}, {0, 384},\
}

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_fullsize = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX = 0,
	.i4PitchY = 0,
	.i4PairNum = 0,
	.i4SubBlkW = 0,
	.i4SubBlkH = 0,
	.i4PosL = {{0, 0} },
	.i4PosR = {{0, 0} },
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.i4LeFirst = 0,
	.i4Crop = pd_i4Crop,
	.iMirrorFlip = IMAGE_HV_MIRROR,
	.i4FullRawW = 16384,
	.i4FullRawH = 12288,
	.i4ModeIndex = 0x3,
	/* VC's PD pattern description */
	.sPDMapInfo[0] = {
		// .i4VCFeature = VC_PDAF_STATS_NE_PIX_1,
		.i4PDPattern = 1,
		.i4BinFacX = 8,
		.i4BinFacY = 16,
		.i4PDRepetition = 0,
		.i4PDOrder = {1},
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_fullsize_fdsum = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX = 0,
	.i4PitchY = 0,
	.i4PairNum = 0,
	.i4SubBlkW = 0,
	.i4SubBlkH = 0,
	.i4PosL = {{0, 0} },
	.i4PosR = {{0, 0} },
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.i4LeFirst = 0,
	.i4Crop = pd_i4Crop,
	.iMirrorFlip = IMAGE_HV_MIRROR,
	.i4FullRawW = 8192,
	.i4FullRawH = 6144,
	.i4ModeIndex = 0x3,
	/* VC's PD pattern description */
	.sPDMapInfo[0] = {
		// .i4VCFeature = VC_PDAF_STATS_NE_PIX_1,
		.i4PDPattern = 1,
		.i4BinFacX = 4,
		.i4BinFacY = 8,
		.i4PDRepetition = 0,
		.i4PDOrder = {1},
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX = 0,
	.i4PitchY = 0,
	.i4PairNum = 0,
	.i4SubBlkW = 0,
	.i4SubBlkH = 0,
	.i4PosL = {{0, 0} },
	.i4PosR = {{0, 0} },
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.i4LeFirst = 0,
	.i4Crop = pd_i4Crop,
	.iMirrorFlip = IMAGE_HV_MIRROR,
	.i4FullRawW = 4096,
	.i4FullRawH = 3072,
	.i4ModeIndex = 0x3,
	/* VC's PD pattern description */
	.sPDMapInfo[0] = {
		// .i4VCFeature = VC_PDAF_STATS_NE_PIX_1,
		.i4PDPattern = 1,
		.i4BinFacX = 2,
		.i4BinFacY = 4,
		.i4PDRepetition = 0,
		.i4PDOrder = {1},
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_8Fdsum_4H2V = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX = 0,
	.i4PitchY = 0,
	.i4PairNum = 0,
	.i4SubBlkW = 0,
	.i4SubBlkH = 0,
	.i4PosL = {{0, 0} },
	.i4PosR = {{0, 0} },
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.i4LeFirst = 0,
	.i4Crop = pd_i4Crop,
	.iMirrorFlip = IMAGE_HV_MIRROR,
	.i4FullRawW = 2048,
	.i4FullRawH = 1536,
	.i4ModeIndex = 0x3,
	/* VC's PD pattern description */
	.sPDMapInfo[0] = {
		// .i4VCFeature = VC_PDAF_STATS_NE_PIX_1,
		.i4PDPattern = 1,
		.i4BinFacX = 2,
		.i4BinFacY = 4,
		.i4PDRepetition = 0,
		.i4PDOrder = {1},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 2304,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 576,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 2304,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 576,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2048,
			.vsize = 1152,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 288,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2048,
			.vsize = 1152,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 288,
			.user_data_desc = VC_PDAF_STATS_ME_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_ME_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 8192,
			.vsize = 6144,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus4[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 8192,
			.vsize = 6144,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus5[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 384,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus6[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2048,
			.vsize = 1152,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus7[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1728,
			.vsize = 1296,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 1728,
			.vsize = 324,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus8[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 384,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus9[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 384,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

/* 14_OceanDX4_03_JN5_Full_12.5Mp_Bypass_4096x3072_30fps_3056Msps QBC*/
static struct mtk_mbus_frame_desc_entry frame_desc_cus10[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 384,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus11[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 2304,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 288,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus12[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 2048,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 512,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus13[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 2304,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 2304,
			.user_data_desc = VC_STAGGER_ME,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 576,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus14[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 2,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_ME,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus15[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 2,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_ME,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 384,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus16[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 2,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_ME,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus17[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2c,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus18[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2d,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus19[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 8192,
			.vsize = 6144,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus20[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 16384,
			.vsize = 12288,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus21[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2c,
			.hsize = 4096,
			.vsize = 2304,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 576,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus22[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2d,
			.hsize = 4096,
			.vsize = 2304,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 576,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus23[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3840,
			.vsize = 2160,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 3840,
			.vsize = 540,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus24[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_ME,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus25[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 2304,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 2304,
			.user_data_desc = VC_STAGGER_ME,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 576,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};


static struct subdrv_mode_struct mode_struct[] = {
	{/*M0 0-8sum 12.5M_4096x3072 30fps */
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = whoopassc1main_preview_capture_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_preview_capture_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = whoopassc1main_preview_capture_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(whoopassc1main_preview_capture_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 6240,
		.framelength = 10256,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x28,
		.imgsensor_winsize_info = { /* BIN */
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*4)/2,
			.y0_offset = (12288-3072*4)/2,
			.w0_size = 4096*4,
			.h0_size = 3072*4,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.coarse_integ_step = 2,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
			.sensorScenario = SENSOR_SCENARIO_BIN,
		},
	},
	{/*M1 0-8sum 12.5M_4096x3072 30fps */
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = whoopassc1main_preview_capture_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_preview_capture_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 6240,
		.framelength = 10256,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x28,
		.imgsensor_winsize_info = {  /* BIN */
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*4)/2,
			.y0_offset = (12288-3072*4)/2,
			.w0_size = 4096*4,
			.h0_size = 3072*4,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.coarse_integ_step = 2,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
			.sensorScenario = SENSOR_SCENARIO_UNDEFINED,
		},
	},
	{/*M2 5-8sum 4k_4096x2304 30fps */
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = whoopassc1main_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_normal_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 6240,
		.framelength = 10256,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x28,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*4)/2,
			.y0_offset = (12288-2304*4)/2,
			.w0_size = 4096*4,
			.h0_size = 2304*4,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.coarse_integ_step = 2,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
			.sensorScenario = SENSOR_SCENARIO_BIN,
		},
	},
	{/*M3 6-8sum 4k_4096x2304 60fps  */
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = whoopassc1main_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_hs_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 6800,
		.framelength = 4704,
		.max_framerate = 600,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x28,
		.imgsensor_winsize_info = {  /* 16:10 BIN */
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*4)/2,
			.y0_offset = (12288-2304*4)/2,
			.w0_size = 4096*4,
			.h0_size = 2304*4,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.coarse_integ_step = 2,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 60,
			.sensorScenario = SENSOR_SCENARIO_BIN,
		},
	},
	{/*M4 21-8sum+2bin 2048x1152 _120fps  */
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = whoopassc1main_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_slim_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 6240,
		.framelength = 2564,
		.max_framerate = 1200,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x18,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-2048*4)/2,
			.y0_offset = (12288-1152*4)/2,
			.w0_size = 2048*4,
			.h0_size = 1152*4,
			.scale_w = 2048,
			.scale_h = 1152,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2048,
			.h1_size = 1152,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2048,
			.h2_tg_size = 1152,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.coarse_integ_step = 2,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_8Fdsum_4H2V,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 120,
			.sensorScenario = SENSOR_SCENARIO_BIN,
		},
	},
	{/*M5 22-8sum+2bin 2048x1152 _240fps */
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = whoopassc1main_custom1_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_custom1_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 6240,
		.framelength = 1282,
		.max_framerate = 2400,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x18,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-2048*8)/2,
			.y0_offset = (12288-1152*8)/2,
			.w0_size = 2048*8,
			.h0_size = 1152*8,
			.scale_w = 2048,
			.scale_h = 1152,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2048,
			.h1_size = 1152,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2048,
			.h2_tg_size = 1152,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.coarse_integ_step = 2,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_8Fdsum_4H2V,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 240,
			.sensorScenario = SENSOR_SCENARIO_BIN,
		},
	},
	{/*M6 7-8sum 12.5M_4096x3072 24fps */
		.frame_desc = frame_desc_cus2,
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = whoopassc1main_custom2_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_custom2_setting),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = whoopassc1main_custom2_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(whoopassc1main_custom2_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 6240,
		.framelength = 12820,
		.max_framerate = 240,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x28,
		.imgsensor_winsize_info = { /* 2X crop 4cell */
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*4)/2,
			.y0_offset = (12288-3072*4)/2,
			.w0_size = 4096*4,
			.h0_size = 3072*4,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.coarse_integ_step = 2,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 24,
			.sensorScenario = SENSOR_SCENARIO_BIN,
		},
	},
	{  /*M7 15-4sum 50M 8192x6144 30fps_sensor remosaic on  */
		.frame_desc = frame_desc_cus3,
		.num_entries = ARRAY_SIZE(frame_desc_cus3),
		.mode_setting_table = whoopassc1main_custom3_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_custom3_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 9984,
		.framelength = 6410,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x26,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-8192*2)/2,
			.y0_offset = (12288-6144*2)/2,
			.w0_size = 8192*2,
			.h0_size = 6144*2,
			.scale_w = 8192,
			.scale_h = 6144,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 8192,
			.h1_size = 6144,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 8192,
			.h2_tg_size = 6144,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 64,
		.coarse_integ_step = 1,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_fullsize_fdsum,
		.sensor_setting_info = {
			.sensor_scenario_usage = RMSC_MASK,
			.equivalent_fps = 30,
			.sensorScenario = SENSOR_SCENARIO_FULL_NCELL,
		},
	},
	{ /*M8 */
		.frame_desc = frame_desc_cus4,
		.num_entries = ARRAY_SIZE(frame_desc_cus4),
		.mode_setting_table = whoopassc1main_custom4_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_custom4_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 9984,
		.framelength = 6410,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x26,
		.imgsensor_winsize_info = { /* FULL MODE 4cell */
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-8192*2)/2,
			.y0_offset = (12288-6144*2)/2,
			.w0_size = 8192*2,
			.h0_size = 6144*2,
			.scale_w = 8192,
			.scale_h = 6144,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 8192,
			.h1_size = 6144,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 8192,
			.h2_tg_size = 6144,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.coarse_integ_step = 1,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_fullsize_fdsum,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
			.sensorScenario = SENSOR_SCENARIO_UNDEFINED,
		},
	},
	{ /*M9 16-4sum crop_4096x3072_30fps _sensor remosaic on */
		.frame_desc = frame_desc_cus5,
		.num_entries = ARRAY_SIZE(frame_desc_cus5),
		.mode_setting_table = whoopassc1main_custom5_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_custom5_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 9984,
		.framelength = 6410,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x26,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*2)/2,
			.y0_offset = (12288-3072*2)/2,
			.w0_size = 4096*2,
			.h0_size = 3072*2,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.coarse_integ_step = 1,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_fullsize_fdsum,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 64,
		.sensor_setting_info = {
			.sensor_scenario_usage = INSENSORZOOM_MASK,
			.equivalent_fps = 30,
			.sensorScenario = SENSOR_SCENARIO_CROP_BAYER,
		},
	},
	{ /*M10 20-8sum+2bincrop 2408x1152 _480fps */
		.frame_desc = frame_desc_cus6,
		.num_entries = ARRAY_SIZE(frame_desc_cus6),
		.mode_setting_table = whoopassc1main_custom6_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_custom6_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 3072,
		.framelength = 1302,
		.max_framerate = 4800,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x46, /* HP9: full:0x46, 4Sum:0x26, 8Sum_2H1V:0x28 */
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-2048*8)/2,
			.y0_offset = (12288-1152*8)/2,
			.w0_size = 2048*8,
			.h0_size = 1152*8,
			.scale_w = 2048,
			.scale_h = 1152,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2048,
			.h1_size = 1152,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2048,
			.h2_tg_size = 1152,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.coarse_integ_step = 1,
		.framelength_step = 1,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 480,
			.sensorScenario = SENSOR_SCENARIO_BIN,
		},
	},
	{  /*M11 10-8sumcrop 1728x1296 24fps*/
		.frame_desc = frame_desc_cus7,
		.num_entries = ARRAY_SIZE(frame_desc_cus7),
		.mode_setting_table = whoopassc1main_custom7_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_custom7_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 6240,
		.framelength = 12820,
		.max_framerate = 240,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x28,
		.framelength_step = 1,
		.coarse_integ_step = 2,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-1728*4)/2,
			.y0_offset = (12288-1296*4)/2,
			.w0_size = 1728*4,
			.h0_size = 1296*4,
			.scale_w = 1728,
			.scale_h = 1296,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 1728,
			.h1_size = 1296,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1728,
			.h2_tg_size = 1296,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {78},
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 24,
			.sensorScenario = SENSOR_SCENARIO_BIN,
		},
	},
	{ /*M12 13-4sum crop_4096x3072_30fps _QBC */
		.frame_desc = frame_desc_cus8,
		.num_entries = ARRAY_SIZE(frame_desc_cus8),
		.mode_setting_table = whoopassc1main_custom8_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_custom8_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = whoopassc1main_custom8_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(whoopassc1main_custom8_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 9984,
		.framelength = 6410,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x26,
		.imgsensor_winsize_info = { /* 2X crop 4cell */
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*2)/2,
			.y0_offset = (12288-3072*2)/2,
			.w0_size = 4096*2,
			.h0_size = 3072*2,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.coarse_integ_step = 1,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_fullsize_fdsum,
		.sensor_output_dataformat_cell_type = SENSOR_OUTPUT_FORMAT_CELL_2X2, /* tetra */
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_Gb,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 64,
		.sensor_setting_info = {
			.sensor_scenario_usage = INSENSORZOOM_MASK,
			.equivalent_fps = 30,
			.sensorScenario = SENSOR_SCENARIO_CROP_4CELL,
		},
	},
	{/*M13 17-4sum crop_4096x3072_24fps _sensor remosaic on*/
		.frame_desc = frame_desc_cus9,
		.num_entries = ARRAY_SIZE(frame_desc_cus9),
		.mode_setting_table = whoopassc1main_custom9_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_custom9_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 9984,
		.framelength = 8012,
		.max_framerate = 240,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x26,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*2)/2,
			.y0_offset = (12288-3072*2)/2,
			.w0_size = 4096*2,
			.h0_size = 3072*2,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.coarse_integ_step = 1,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_fullsize_fdsum,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 64,
		.sensor_setting_info = {
			.sensor_scenario_usage = INSENSORZOOM_MASK,
			.equivalent_fps = 24,
			.sensorScenario = SENSOR_SCENARIO_CROP_2X_NCELL,
		},
	},
	{/*M14 14-4sum crop_4096x3072_24fps _QBC*/
		.frame_desc = frame_desc_cus10,
		.num_entries = ARRAY_SIZE(frame_desc_cus10),
		.mode_setting_table = whoopassc1main_custom10_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_custom10_setting),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = whoopassc1main_custom10_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(whoopassc1main_custom10_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 9984,
		.framelength = 8012,
		.max_framerate = 240,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x26,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*2)/2,
			.y0_offset = (12288-3072*2)/2,
			.w0_size = 4096*2,
			.h0_size = 3072*2,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.coarse_integ_step = 1,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_fullsize_fdsum,
		.sensor_output_dataformat_cell_type = SENSOR_OUTPUT_FORMAT_CELL_2X2, /* tetra */
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_Gb,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 64,
		.sensor_setting_info = {
			.sensor_scenario_usage = INSENSORZOOM_MASK,
			.equivalent_fps = 24,
			.sensorScenario = SENSOR_SCENARIO_CROP_2X_NCELL,
		},
	},
	{/*M15 18-4sum crop_4096x2304_30fps _sensor remosaic on */
		.frame_desc = frame_desc_cus11,
		.num_entries = ARRAY_SIZE(frame_desc_cus11),
		.mode_setting_table = whoopassc1main_custom11_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_custom11_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 9984,
		.framelength = 6410,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x26,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*2)/2,
			.y0_offset = (12288-2304*2)/2,
			.w0_size = 4096*2,
			.h0_size = 2304*2,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.coarse_integ_step = 1,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_8Fdsum_4H2V,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 64,
		.sensor_setting_info = {
			.sensor_scenario_usage = INSENSORZOOM_MASK,
			.equivalent_fps = 30,
			.sensorScenario = SENSOR_SCENARIO_CROP_2X_NCELL,
		},
	},
	{/*M16 9---8sum 12.5M_4096x2048 30fps */
		.frame_desc = frame_desc_cus12,
		.num_entries = ARRAY_SIZE(frame_desc_cus12),
		.mode_setting_table = whoopassc1main_custom12_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_custom12_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 6240,
		.framelength = 10256,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x28,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*4)/2,
			.y0_offset = (12288-2048*4)/2,
			.w0_size = 4096*4,
			.h0_size = 2048*4,
			.scale_w = 4096,
			.scale_h = 2048,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2048,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2048,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.coarse_integ_step = 2,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
			.sensorScenario = SENSOR_SCENARIO_BIN,
		},
	},
	{/*M17 8---8sum 4k_4096x2304 30fps _iDCG*/
		.frame_desc = frame_desc_cus13,
		.num_entries = ARRAY_SIZE(frame_desc_cus13),
		.mode_setting_table = whoopassc1main_custom13_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_custom13_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_RAW_DCG_RAW,
		.raw_cnt = 2,
		.exp_cnt = 2,
		.pclk = 1920000000,
		.linelength = 20376,
		.framelength = 3140,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x14,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*4)/2,
			.y0_offset = (12288-2304*4)/2,
			.w0_size = 4096*4,
			.h0_size = 2304*4,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.coarse_integ_step = 1,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.saturation_info = &imgsensor_saturation_info,
		.dcg_info = {
			.dcg_mode = IMGSENSOR_DCG_RAW,
			.dcg_gain_mode = IMGSENSOR_DCG_RATIO_MODE,
			.dcg_gain_base = IMGSENSOR_DCG_GAIN_LCG_BASE,
			.dcg_gain_ratio_min = 1000,  //DCG DGN
			.dcg_gain_ratio_max = 8000,
			.dcg_gain_ratio_step = 0,
			.dcg_gain_table = whoopassc1main_dcg_ratio_table_10bit,
			.dcg_gain_table_size = sizeof(whoopassc1main_dcg_ratio_table_10bit),
		},
		.sensor_setting_info = {
			.sensor_scenario_usage = DCG_MASK,
			.equivalent_fps = 30,
			.sensorScenario = SENSOR_SCENARIO_DCG_AP_MERGE_2EXP_BAYER,
		},
	},
	{/*M18 1---8sum 12.5M_4096x3072 60fps _LBMF*/
		.frame_desc = frame_desc_cus14,
		.num_entries = ARRAY_SIZE(frame_desc_cus14),
		.mode_setting_table = whoopassc1main_custom14_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_custom14_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = whoopassc1main_custom14_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(whoopassc1main_custom14_setting),
		.hdr_mode = HDR_RAW_LBMF,
		.raw_cnt = 2,
		.exp_cnt = 2,
		.pclk = 1920000000,
		.linelength = 6240,
		.framelength = 5128*2,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 3072,
		.read_margin = 0,
		.exposure_margin = 0x28,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*4)/2,
			.y0_offset = (12288-3072*4)/2,
			.w0_size = 4096*4,
			.h0_size = 3072*4,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.exposure_order_in_lbmf = IMGSENSOR_LBMF_EXPOSURE_SE_FIRST,
		.mode_type_in_lbmf = IMGSENSOR_LBMF_MODE_MANUAL,
		.coarse_integ_step = 2,
		.framelength_step = 1,
		.awb_enabled = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.sensor_setting_info = {
			.sensor_scenario_usage = HDR_RAW_STAGGER_2EXP_MASK,
			.equivalent_fps = 30,
			.sensorScenario = SENSOR_SCENARIO_LBMF_2EXP_BAYER,
		},
		.hw_mode = 2,//2:DC MODE, 1:OTF MODE
	},
	{/*M19 12---4sum_4096x3072 60fps _QBC AEB*/
		.frame_desc = frame_desc_cus15,
		.num_entries = ARRAY_SIZE(frame_desc_cus15),
		.mode_setting_table = whoopassc1main_custom15_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_custom15_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = whoopassc1main_custom15_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(whoopassc1main_custom15_setting),
		.hdr_mode = HDR_RAW_LBMF,
		.raw_cnt = 2,
		.exp_cnt = 2,
		.pclk = 1920000000,
		.linelength = 9984,
		.framelength = 3205*2,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 3072,
		.read_margin = (3205 - 3072),
		.exposure_margin = 0x26,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*2)/2,
			.y0_offset = (12288-3072*2)/2,
			.w0_size = 4096*2,
			.h0_size = 3072*2,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.coarse_integ_step = 1,
		.framelength_step = 1,
		.sensor_output_dataformat_cell_type = SENSOR_OUTPUT_FORMAT_CELL_2X2, /* tetra */
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_Gb,
		.awb_enabled = 1,
		.exposure_order_in_lbmf = IMGSENSOR_LBMF_EXPOSURE_SE_FIRST,
		.mode_type_in_lbmf = IMGSENSOR_LBMF_MODE_MANUAL,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].max = (0xFFFF - 0x26),
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].max = (0xFFFF - 0x26),
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 64,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].max = BASEGAIN * 64,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_fullsize_fdsum,
		.sensor_setting_info = {
			.sensor_scenario_usage = INSENSORZOOM_MASK | HDR_RAW_STAGGER_2EXP_MASK,
			.equivalent_fps = 30,
			.sensorScenario = SENSOR_SCENARIO_LBMF_2EXP_CROP_4CELL,
		},
		.hw_mode = 2,//2:DC MODE, 1:OTF MODE
	},
	{/*M20 2---8sum  12.5M 4096x3072 30fps iDCG 平台合成*/
		.frame_desc = frame_desc_cus16,
		.num_entries = ARRAY_SIZE(frame_desc_cus16),
		.mode_setting_table = whoopassc1main_custom16_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_custom16_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = whoopassc1main_custom16_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(whoopassc1main_custom16_setting),
		.hdr_mode = HDR_RAW_DCG_RAW,
		.raw_cnt = 2,
		.exp_cnt = 2,
		.pclk = 1920000000,
		.linelength = 20376,
		.framelength = 3140,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x14,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*4)/2,
			.y0_offset = (12288-3072*4)/2,
			.w0_size = 4096*4,
			.h0_size = 3072*4,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.coarse_integ_step = 1,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.saturation_info = &imgsensor_saturation_info,
		.dcg_info = {
			.dcg_mode = IMGSENSOR_DCG_RAW,
			.dcg_gain_mode = IMGSENSOR_DCG_RATIO_MODE,
			.dcg_gain_base = IMGSENSOR_DCG_GAIN_LCG_BASE,
			.dcg_gain_ratio_min = 8000,
			.dcg_gain_ratio_max = 8000,
			.dcg_gain_ratio_step = 0,
			.dcg_gain_table = whoopassc1main_dcg_ratio_table_10bit,
			.dcg_gain_table_size = sizeof(whoopassc1main_dcg_ratio_table_10bit),
		},
		.sensor_setting_info = {
			.cgratio = 8000,
			.sensor_scenario_usage = DCG_MASK,
			.equivalent_fps = 30,
			.sensorScenario = SENSOR_SCENARIO_DCG_AP_MERGE_2EXP_BAYER,
		},
		.hw_mode = 2,//2:DC MODE, 1:OTF MODE
	},
	{/*M21 3---8sum  12.5M 4096x3072 30fps iDCG sensor 合成12bit*/
		.frame_desc = frame_desc_cus17,
		.num_entries = ARRAY_SIZE(frame_desc_cus17),
		.mode_setting_table = whoopassc1main_custom17_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_custom17_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = whoopassc1main_custom17_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(whoopassc1main_custom17_setting),
		.hdr_mode = HDR_RAW_DCG_COMPOSE,
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW12_B,
		.raw_cnt = 1,
		.exp_cnt = 2,
		.pclk = 1920000000,
		.linelength = 20376,
		.framelength = 3140,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x14,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*4)/2,
			.y0_offset = (12288-3072*4)/2,
			.w0_size = 4096*4,
			.h0_size = 3072*4,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.coarse_integ_step = 1,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.saturation_info = &imgsensor_saturation_info_12bit,
		.dcg_info = {
			.dcg_mode = IMGSENSOR_DCG_COMPOSE,
			.dcg_gain_mode = IMGSENSOR_DCG_RATIO_MODE,
			.dcg_gain_base = IMGSENSOR_DCG_GAIN_LCG_BASE,
			.dcg_gain_ratio_min = 8000,
			.dcg_gain_ratio_max = 8000,
			.dcg_gain_ratio_step = 0,
			.dcg_gain_table = whoopassc1main_dcg_ratio_table_12bit,
			.dcg_gain_table_size = sizeof(whoopassc1main_dcg_ratio_table_12bit),
		},
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 256,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].max = BASEGAIN * 32,
		.sensor_setting_info = {
			.sensor_scenario_usage = DCG_MASK,
			.equivalent_fps = 30,
			.sensorScenario = SENSOR_SCENARIO_DCG_SENSOR_MERGE_12BIT_BAYER,
		},
	},
	{/*M22 4---8sum  12.5M 4096x3072 30fps iDCG sensor 合成14bit*/
		.frame_desc = frame_desc_cus18,
		.num_entries = ARRAY_SIZE(frame_desc_cus18),
		.mode_setting_table = whoopassc1main_custom18_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_custom18_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = whoopassc1main_custom18_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(whoopassc1main_custom18_setting),
		.hdr_mode = HDR_NONE,
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW14_B,
		.raw_cnt = 1,
		.exp_cnt = 2,
		.pclk = 1920000000,
		.linelength = 20376,
		.framelength = 3140,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x14,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*4)/2,
			.y0_offset = (12288-3072*4)/2,
			.w0_size = 4096*4,
			.h0_size = 3072*4,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.coarse_integ_step = 1,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.saturation_info = &imgsensor_saturation_info_14bit,
		.dcg_info = {
			.dcg_mode = IMGSENSOR_DCG_COMPOSE,
			.dcg_gain_mode = IMGSENSOR_DCG_RATIO_MODE,
			.dcg_gain_base = IMGSENSOR_DCG_GAIN_HCG_BASE,
			.dcg_gain_ratio_min = 8000,
			.dcg_gain_ratio_max = 8000,
			.dcg_gain_ratio_step = 0,
			.dcg_gain_table = whoopassc1main_dcg_ratio_table_14bit,
			.dcg_gain_table_size = sizeof(whoopassc1main_dcg_ratio_table_14bit),
		},
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 256,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].max = BASEGAIN * 32,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
			.sensorScenario = SENSOR_SCENARIO_UNDEFINED,
		},
	},
	{/*M23 11---4sum 50M 8192x6144 30fps QBC */
		.frame_desc = frame_desc_cus19,
		.num_entries = ARRAY_SIZE(frame_desc_cus19),
		.mode_setting_table = whoopassc1main_custom19_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_custom19_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = whoopassc1main_custom19_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(whoopassc1main_custom19_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 9984,
		.framelength = 6410,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x26,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-8192*2)/2,
			.y0_offset = (12288-6144*2)/2,
			.w0_size = 8192*2,
			.h0_size = 6144*2,
			.scale_w = 8192,
			.scale_h = 6144,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 8192,
			.h1_size = 6144,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 8192,
			.h2_tg_size = 6144,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 64,
		.coarse_integ_step = 1,
		.framelength_step = 1,
		.sensor_output_dataformat_cell_type = SENSOR_OUTPUT_FORMAT_CELL_2X2, /* tetra */
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_Gb,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_fullsize_fdsum,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
			.sensorScenario = SENSOR_SCENARIO_FULL_4CELL,
		},
	},
	{/*M24 19---full 16384x12288 7.5fps QBC*/
		.frame_desc = frame_desc_cus20,
		.num_entries = ARRAY_SIZE(frame_desc_cus20),
		.mode_setting_table = whoopassc1main_custom20_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_custom20_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 19968,
		.framelength = 12820,
		.max_framerate = 75,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x46,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 16384,
			.h0_size = 12288,
			.scale_w = 16384,
			.scale_h = 12288,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 16384,
			.h1_size = 12288,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 16384,
			.h2_tg_size = 12288,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.coarse_integ_step = 1,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_fullsize,
		.sensor_output_dataformat_cell_type = SENSOR_OUTPUT_FORMAT_CELL_4X4, /* hexdeca */
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_Gb,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
			.sensorScenario = SENSOR_SCENARIO_UNDEFINED,
		},
	},
	{/*M25 27_4096x2304_30.01fps_2976Msps*/
		.frame_desc = frame_desc_cus21,
		.num_entries = ARRAY_SIZE(frame_desc_cus21),
		.mode_setting_table = whoopassc1main_custom21_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_custom21_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_RAW_DCG_COMPOSE,
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW12_Gb,
		.raw_cnt = 1,
		.exp_cnt = 2,
		.pclk = 1920000000,
		.linelength = 20376,
		.framelength = 3140,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x14,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*4)/2,
			.y0_offset = (12288-2304*4)/2,
			.w0_size = 4096*4,
			.h0_size = 2304*4,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.coarse_integ_step = 1,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.saturation_info = &imgsensor_saturation_info_12bit,
		.dcg_info = {
			.dcg_mode = IMGSENSOR_DCG_COMPOSE,
			.dcg_gain_mode = IMGSENSOR_DCG_RATIO_MODE,
			.dcg_gain_base = IMGSENSOR_DCG_GAIN_LCG_BASE,
			.dcg_gain_ratio_min = 8000,
			.dcg_gain_ratio_max = 8000,
			.dcg_gain_ratio_step = 0,
			.dcg_gain_table = whoopassc1main_dcg_ratio_table_12bit,
			.dcg_gain_table_size = sizeof(whoopassc1main_dcg_ratio_table_12bit),
		},
		.sensor_setting_info = {
			.sensor_scenario_usage = DCG_MASK,
			.equivalent_fps = 30,
			.sensorScenario = SENSOR_SCENARIO_DCG_SENSOR_MERGE_12BIT_BAYER,
		},
	},
	{/*26 28_4096x2304_30.01fps_2976Msps*/
		.frame_desc = frame_desc_cus22,
		.num_entries = ARRAY_SIZE(frame_desc_cus22),
		.mode_setting_table = whoopassc1main_custom22_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_custom22_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_RAW_DCG_COMPOSE,
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW14_Gb,
		.raw_cnt = 1,
		.exp_cnt = 2,
		.pclk = 1920000000,
		.linelength = 20376,
		.framelength = 3140,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x14,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*4)/2,
			.y0_offset = (12288-2304*4)/2,
			.w0_size = 4096*4,
			.h0_size = 2304*4,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.coarse_integ_step = 1,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.saturation_info = &imgsensor_saturation_info_14bit,
		.dcg_info = {
			.dcg_mode = IMGSENSOR_DCG_COMPOSE,
			.dcg_gain_mode = IMGSENSOR_DCG_RATIO_MODE,
			.dcg_gain_base = IMGSENSOR_DCG_GAIN_HCG_BASE,
			.dcg_gain_ratio_min = 8000,
			.dcg_gain_ratio_max = 8000,
			.dcg_gain_ratio_step = 0,
			.dcg_gain_table = whoopassc1main_dcg_ratio_table_14bit,
			.dcg_gain_table_size = sizeof(whoopassc1main_dcg_ratio_table_14bit),
		},
		.sensor_setting_info = {
			.sensor_scenario_usage = DCG_MASK,
			.equivalent_fps = 30,
			.sensorScenario = SENSOR_SCENARIO_DCG_SENSOR_MERGE_14BIT_BAYER,
		},
	},
	{/*M27 25_3840x2160_60fps_2976Msps	*/
		.frame_desc = frame_desc_cus23,
		.num_entries = ARRAY_SIZE(frame_desc_cus23),
		.mode_setting_table = whoopassc1main_custom23_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_custom23_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 6240,
		.framelength = 5128,
		.max_framerate = 600,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x28,
		.imgsensor_winsize_info = {  /* 16:10 BIN */
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-3840*4)/2,
			.y0_offset = (12288-2160*4)/2,
			.w0_size = 3840*4,
			.h0_size = 2160*4,
			.scale_w = 3840,
			.scale_h = 2160,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3840,
			.h1_size = 2160,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3840,
			.h2_tg_size = 2160,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.coarse_integ_step = 2,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 60,
			.sensorScenario = SENSOR_SCENARIO_BIN,
		},
	},
	{/*M28 23_4096x3072_30.01fps_2976Msps ---8Sum2Avg_iDCG1:4_HCGPD  ap merge-拍照*/
		.frame_desc = frame_desc_cus24,
		.num_entries = ARRAY_SIZE(frame_desc_cus24),
		.mode_setting_table = whoopassc1main_custom24_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_custom24_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_RAW_DCG_RAW,
		.raw_cnt = 2,
		.exp_cnt = 2,
		.pclk = 1920000000,
		.linelength = 20376,
		.framelength = 3140,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x14,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*4)/2,
			.y0_offset = (12288-3072*4)/2,
			.w0_size = 4096*4,
			.h0_size = 3072*4,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.coarse_integ_step = 1,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.saturation_info = &imgsensor_saturation_info,
		.dcg_info = {
			.dcg_mode = IMGSENSOR_DCG_RAW,
			.dcg_gain_mode = IMGSENSOR_DCG_RATIO_MODE,
			.dcg_gain_base = IMGSENSOR_DCG_GAIN_LCG_BASE,
			.dcg_gain_ratio_min = 4000,
			.dcg_gain_ratio_max = 4000,
			.dcg_gain_ratio_step = 0,
			.dcg_gain_table = whoopassc1main_dcg_ratio_table_10bit_2,
			.dcg_gain_table_size = sizeof(whoopassc1main_dcg_ratio_table_10bit_2),
		},
		.sensor_setting_info = {
			.sensor_scenario_usage = DCG_MASK,
			.equivalent_fps = 30,
			.sensorScenario = SENSOR_SCENARIO_DCG_AP_MERGE_2EXP_BAYER,
		},
	},
	{/*M29 24_4096x2304_30.01fps_2976Msps 8Sum2Avg_iDCG1:4_HCGPD  ap merge-视频*/
		.frame_desc = frame_desc_cus25,
		.num_entries = ARRAY_SIZE(frame_desc_cus25),
		.mode_setting_table = whoopassc1main_custom25_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1main_custom25_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_RAW_DCG_RAW,
		.raw_cnt = 2,
		.exp_cnt = 2,
		.pclk = 1920000000,
		.linelength = 20376,
		.framelength = 3140,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x14,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*4)/2,
			.y0_offset = (12288-2304*4)/2,
			.w0_size = 4096*4,
			.h0_size = 2304*4,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.coarse_integ_step = 1,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.saturation_info = &imgsensor_saturation_info,
		.dcg_info = {
			.dcg_mode = IMGSENSOR_DCG_RAW,
			.dcg_gain_mode = IMGSENSOR_DCG_RATIO_MODE,
			.dcg_gain_base = IMGSENSOR_DCG_GAIN_LCG_BASE,
			.dcg_gain_ratio_min = 4000,
			.dcg_gain_ratio_max = 4000,
			.dcg_gain_ratio_step = 0,
			.dcg_gain_table = whoopassc1main_dcg_ratio_table_10bit_2,
			.dcg_gain_table_size = sizeof(whoopassc1main_dcg_ratio_table_10bit_2),
		},
		.sensor_setting_info = {
			.sensor_scenario_usage = DCG_MASK,
			.equivalent_fps = 30,
			.sensorScenario = SENSOR_SCENARIO_DCG_AP_MERGE_2EXP_BAYER,
		},
	},
};


static struct subdrv_static_ctx static_ctx = {
	.sensor_id = WHOOPASSC1MAIN_SENSOR_ID,
	.reg_addr_sensor_id = { 0x0000, 0x0001 },
	.i2c_addr_table = {0x20, 0x34, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_16,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),

	.mirror = IMAGE_HV_MIRROR,

	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_CPHY,
	.mipi_lane_num = SENSOR_MIPI_3_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_Gb,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 256,
	.ana_gain_type = 2, //0-SONY; 1-OV; 2 - SUMSUN; 3 -HYNIX; 4 -GC
	.ana_gain_step = 1,
	.ana_gain_table = whoopassc1main_ana_gain_table,
	.ana_gain_table_size = sizeof(whoopassc1main_ana_gain_table),
	.tuning_iso_base = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 4,
	.exposure_max = (0xFFFF - 0x46) << 9,
	.exposure_step = 1,
	.exposure_margin = 0x28,
	.dig_gain_min = BASE_DGAIN * 1,
	.dig_gain_max = BASE_DGAIN * 16,
	.dig_gain_step = 1,
	.frame_length_max = 0xFFFF,
	/* .frame_length_max_without_lshift = 0xFFFF, */
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 1616100,
	.line_interleave_num = 1,
	/* stagger behavior by vendor type */
//	.stagger_rg_order = IMGSENSOR_STAGGER_RG_SE_FIRST,
//	.stagger_fl_type = IMGSENSOR_STAGGER_FL_MANUAL,

	.pdaf_type = PDAF_SUPPORT_CAMSV_QPD,
	.hdr_type = HDR_SUPPORT_STAGGER_FDOL|HDR_SUPPORT_DCG|HDR_SUPPORT_LBMF,
	.saturation_info = &imgsensor_saturation_info,
	.seamless_switch_support = TRUE,
	.temperature_support = TRUE,
	.g_temp = get_sensor_temperature,
	.g_gain2reg = get_gain2reg,
	.g_cali = whoopassc1main_get_sensor_cali,
	.s_gph = set_group_hold,
	.s_cali = whoopassc1main_set_sensor_cali,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101, /*general_setup_image_orientation*/
	.reg_addr_frame_length = {0x0340, 0x0341}, /*frame_length_lines*/
	.reg_addr_temp_en = PARAM_UNDEFINED,
	.reg_addr_temp_read = 0x0020, /*General temperature*/
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_frame_count = 0x0005, /*api_rd_general_frame_count*/
	.long_exposure_support = TRUE,
	.reg_addr_exposure_lshift = 0x0704, /*coarse_integration_time_sh*/
//	.reg_addr_frame_length_lshift = 0x0702, /*frame_length_lines_sh*/

	/*datasheet search "AE control guide"*/
	.reg_addr_exposure = {
		{ 0x0202, 0x0203 },
		{ 0x022C, 0x022D },
		{ 0x0226, 0x0227 },
	},
	.reg_addr_ana_gain = {
		{ 0x0204, 0x0205 },
		{ 0x0208, 0x0209 },
		{ 0x0206, 0x0207 },
	},

	/*datasheet search "Example of AEB Control"*/
	.reg_addr_exposure_in_lut = {
			{0x0E10, 0x0E11}, /* LUT_A_COARSE_INTEG_TIME */
			{0x0E1E, 0x0E1F}, /* LUT_B_COARSE_INTEG_TIME */
	},
	.reg_addr_ana_gain_in_lut = {
			{0x0E12, 0x0E13}, /* LUT_A_ANA_GAIN_GLOBAL */
			{0x0E20, 0x0E21}, /* LUT_B_ANA_GAIN_GLOBAL */
	},
	// .reg_addr_dig_gain_in_lut = {
	// 		{0x0E14, 0x0E15},
	// 		{0x0E20, 0x0E21},
	// },
	.reg_addr_frame_length_in_lut = {
			{0x0E14, 0x0E15},
			{0x0E22, 0x0E23},
	},

	// .init_setting_table = whoopassc1main_init_setting,
	// .init_setting_len = ARRAY_SIZE(whoopassc1main_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),
	.chk_s_off_sta = 1,
	.chk_s_off_end = 0,
//	.chk_streaming_st = whoopassc1main_chk_streaming_st,

	.checksum_value = 0xc88841a,
};

// static int whoopassc1main_set_ctrl_locker(struct subdrv_ctx *ctx, u32 cid, bool *is_lock)
// {
// 	bool lock_set_ctrl = false;

// 	if (unlikely(is_lock == NULL)) {
// 		pr_info("[%s][ERROR] is_lock %p is NULL\n", __func__, is_lock);
// 		return -EINVAL;
// 	}

// 	switch (cid) {
// 	case V4L2_CID_MTK_STAGGER_AE_CTRL:
// 	case V4L2_CID_MTK_MAX_FPS:

// 		if ((ctx->sof_no == 0) && (ctx->is_streaming)) {
// 			lock_set_ctrl = true;
// 			DRV_LOG(ctx,
// 				"[%s] Target lock cid(%u) lock_set_ctrl(%d), sof_no(%d) is_streaming(%d)\n",
// 				__func__,
// 				cid,
// 				lock_set_ctrl,
// 				ctx->sof_no,
// 				ctx->is_streaming);
// 		}
// 		break;
// 	default:
// 		break;
// 	}

// 	*is_lock = lock_set_ctrl;
// 	return ERROR_NONE;
// }


static struct subdrv_ops ops = {
	.get_id = get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = open,
	.get_info = common_get_info,
	.get_resolution = common_get_resolution,
	.control = common_control,
	.feature_control = common_feature_control,
	.close = common_close,
	.get_frame_desc = common_get_frame_desc,
	.get_temp = common_get_temp,
	.get_csi_param = common_get_csi_param,
	.update_sof_cnt = common_update_sof_cnt,
//	.vsync_notify = vsync_notify,
//  .set_ctrl_locker = whoopassc1main_set_ctrl_locker,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, {24}, 10000},
	{HW_ID_RST, {0}, 1000},
	{HW_ID_DOVDD, {1800000, 1800000}, 1000},
	{HW_ID_AVDD, {2200000, 2200000}, 1000},
	{HW_ID_DVDD1, {1008000, 1008000}, 1000},
	{HW_ID_DVDD, {1002000, 1002000}, 1000},
	{HW_ID_AFVDD, {2800000, 2800000}, 3000},
	{HW_ID_RST, {1}, 5000},
	{HW_ID_MCLK_DRIVING_CURRENT, {6}, 20000},
};
struct subdrv_entry whoopassc1main_mipi_raw_entry = {

	.name = "whoopassc1main_mipi_raw",
	.id = WHOOPASSC1MAIN_SENSOR_ID, /* 0x1b73 */
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/*
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt, u64 sof_ts)
{
	DRV_LOG(ctx, "sof_cnt(%u) ctx->ref_sof_cnt(%u) ctx->fast_mode_on(%d)",
		sof_cnt, ctx->ref_sof_cnt, ctx->fast_mode_on);
	if (ctx->fast_mode_on && (sof_cnt > ctx->ref_sof_cnt)) {
		ctx->fast_mode_on = FALSE;
		ctx->ref_sof_cnt = 0;
		DRV_LOG(ctx, "seamless_switch disabled.");
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_fast_mode, 0x00);
		commit_i2c_buffer(ctx);
	}
	return 0;
}
*/

#define TEMP_READY_TIME_DELAY_FRAME (4)
#define INVALID_TEMP_VALUE (0x7FFF)

static int get_sensor_temperature(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	short temperature = 0;
	int temperature_convert = 0;

	if( ctx->sof_cnt < TEMP_READY_TIME_DELAY_FRAME) {
		temperature_convert = INVALID_TEMP_VALUE;
		DRV_LOG(ctx, "INVALID_TEMP_VALUE(%d)\n", temperature_convert);
	} else {
		temperature = subdrv_i2c_rd_u16(ctx, ctx->s_ctx.reg_addr_temp_read);
		temperature_convert = temperature / 256;
	}

	DRV_LOG(ctx, "reg_val:0x%x, temperature: %d degrees\n", temperature, temperature_convert);
	return temperature_convert;
}

static unsigned int read_whoopassc1main_eeprom_info(struct subdrv_ctx *ctx, kal_uint16 meta_id,
	BYTE *data, int size)
{
	kal_uint16 addr;
	int readsize;

	if (meta_id != whoopassc1main_eeprom_info[meta_id].meta)
		return -1;

	if (size != whoopassc1main_eeprom_info[meta_id].size)
		return -1;

	addr = whoopassc1main_eeprom_info[meta_id].start;
	readsize = whoopassc1main_eeprom_info[meta_id].size;

	if (!read_cmos_eeprom_p8(ctx, addr, data, readsize)) {
		DRV_LOGE(ctx, "read meta_id(%d) failed", meta_id);
	}

	return 0;
}

static struct eeprom_addr_table_struct oplus_eeprom_addr_table = {
	.i2c_read_id = 0xA1,
	.i2c_write_id = 0xA0,

	.addr_modinfo = 0x0000,
	.addr_sensorid = 0x0006,
	.addr_lens = 0x0008,
	.addr_vcm = 0x000A,
	.addr_modinfoflag = 0x0010,

	.addr_af = 0x0092,
	.addr_afmacro = 0x0092,
	.addr_afinf = 0x0094,
	.addr_afflag = 0x0098,

	.addr_qrcode = 0x00B0,
	.addr_qrcodeflag = 0x00C7,
};

static int get_eeprom_common_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	memcpy(para, (u8*)(&oplus_eeprom_info), sizeof(oplus_eeprom_info));
	*len = sizeof(oplus_eeprom_info);
	return 0;
}

static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
{
    kal_uint16 get_byte = 0;

    adaptor_i2c_rd_u8(ctx->i2c_client, WHOOPASSC1MAIN_EEPROM_ADDR >> 1, addr, (u8 *)&get_byte);
    return get_byte;
}

static kal_int32 table_write_eeprom_one_packet(struct subdrv_ctx *ctx,
        kal_uint16 addr, kal_uint8 *para, kal_uint32 len)
{
    kal_int32 ret = ERROR_NONE;
    ret = adaptor_i2c_wr_p8(ctx->i2c_client, WHOOPASSC1MAIN_EEPROM_ADDR >> 1,
            addr, para, len);

    return ret;
}

static kal_int32 write_eeprom_protect(struct subdrv_ctx *ctx, kal_uint16 enable)
{
    kal_int32 ret = ERROR_NONE;
    kal_uint16 reg = 0xA000;

    if (enable) {
        adaptor_i2c_wr_u8(ctx->i2c_client, WHOOPASSC1MAIN_EEPROM_ADDR >> 1, reg, 0x0E);
    }
    else {
        adaptor_i2c_wr_u8(ctx->i2c_client, WHOOPASSC1MAIN_EEPROM_ADDR >> 1, reg, 0x00);
    }

    return ret;
}

static kal_uint16 get_64align_addr(kal_uint16 data_base) {

	kal_uint16 multiple = 0;
	kal_uint16 surplus = 0;
	kal_uint16 addr_64align = 0;

	multiple = data_base / 64;
	surplus = data_base % 64;
	if(surplus) {
		addr_64align = (multiple + 1) * 64;
	} else {
		addr_64align = multiple * 64;
	}
	//LOG_INF("data_base(0x%x), multiple(%d), surplus(%d), addr_64align(0x%x)", data_base, multiple, surplus, addr_64align);
	return addr_64align;
}

static kal_int32 eeprom_table_write(struct subdrv_ctx *ctx, kal_uint16 data_base, kal_uint8 *pData, kal_uint16 data_length) {

	kal_uint16 idx;
	kal_uint16 idy;
	kal_int32 ret = ERROR_NONE;
	UINT32 i = 0;

	idx = data_length / EEPROM_WRITE_DATA_MAX_LENGTH;
	idy = data_length % EEPROM_WRITE_DATA_MAX_LENGTH;

    LOG_INF("data_base(0x%x) data_length(%d) idx(%d) idy(%d)\n", data_base, data_length, idx, idy);

	for (i = 0; i < idx; i++ ) {
		ret = table_write_eeprom_one_packet(ctx, (data_base + EEPROM_WRITE_DATA_MAX_LENGTH * i),
				&pData[EEPROM_WRITE_DATA_MAX_LENGTH*i], EEPROM_WRITE_DATA_MAX_LENGTH);
		if (ret != ERROR_NONE) {
			LOG_INF("write_eeprom error: i=%d\n", i);
			return -1;
		}
		msleep(6);
	}

	msleep(6);
	if(idy) {
		ret = table_write_eeprom_one_packet(ctx, (data_base + EEPROM_WRITE_DATA_MAX_LENGTH*idx),
				&pData[EEPROM_WRITE_DATA_MAX_LENGTH*idx], idy);
		if (ret != ERROR_NONE) {
			LOG_INF("write_eeprom error: idx= %d idy= %d\n", idx, idy);
			return -1;
		}
	}
	return 0;
}

static kal_int32 eeprom_64align_write(struct subdrv_ctx *ctx, kal_uint16 data_base, kal_uint8 *pData, kal_uint16 data_length) {

	kal_uint16 addr_64align = 0;
	kal_uint16 part1_length = 0;
	kal_uint16 part2_length = 0;
	kal_int32 ret = ERROR_NONE;

    addr_64align = get_64align_addr(data_base);

	part1_length = addr_64align - data_base;
	if(part1_length > data_length) {
		part1_length = data_length;
	}
	part2_length = data_length - part1_length;

	write_eeprom_protect(ctx, 0);
	msleep(6);

	if (part1_length) {
		ret = eeprom_table_write(ctx, data_base, pData, part1_length);
		if (ret == -1) {
			/* open write protect */
			write_eeprom_protect(ctx, 1);
			LOG_INF("write_eeprom error part1\n");
			msleep(6);
			return -1;
		}
	}

	msleep(6);
	if (part2_length) {
		ret = eeprom_table_write(ctx, addr_64align, pData + part1_length, part2_length);
		if (ret == -1) {
			/* open write protect */
			write_eeprom_protect(ctx, 1);
			LOG_INF("write_eeprom error part2\n");
			msleep(6);
			return -1;
		}
	}
	msleep(6);
	write_eeprom_protect(ctx, 1);
	msleep(6);

	return 0;
}

static kal_int32 write_Module_data(struct subdrv_ctx *ctx,
    ACDK_SENSOR_ENGMODE_STEREO_STRUCT * pStereodata)
{
    kal_int32  ret = ERROR_NONE;
    kal_uint16 data_base, data_length;
    kal_uint8 *pData;

    if(pStereodata != NULL) {
        LOG_INF("SET_SENSOR_OTP: 0x%x %d 0x%x %d\n",
                       pStereodata->uSensorId,
                       pStereodata->uDeviceId,
                       pStereodata->baseAddr,
                       pStereodata->dataLength);

        data_base = pStereodata->baseAddr;
        data_length = pStereodata->dataLength;
        pData = pStereodata->uData;
        if (((pStereodata->uSensorId == WHOOPASSC1MAIN_SENSOR_ID) || (pStereodata->uSensorId == WHOOPASSS1MAIN_SENSOR_ID)
            || (pStereodata->uSensorId == WHOOPASSC1MAIN2_SENSOR_ID) || (pStereodata->uSensorId == WHOOPASSS1MAIN2_SENSOR_ID)
            || (pStereodata->uSensorId == WHOOPASSC2MAIN_SENSOR_ID) || (pStereodata->uSensorId == WHOOPASSS2MAIN_SENSOR_ID)
            || (pStereodata->uSensorId == WHOOPASSC2MAIN2_SENSOR_ID) || (pStereodata->uSensorId == WHOOPASSS2MAIN2_SENSOR_ID))
            && (data_length == CALI_DATA_MASTER_LENGTH)
            && ((data_base == WHOOPASSC1MAIN_STEREO_MW_START_ADDR)
                || (data_base == WHOOPASSC1MAIN_STEREO_MT_START_ADDR)
                || (data_base == WHOOPASSC1MAIN_STEREO_MT105_START_ADDR))) {
            LOG_INF("Write: %x %x %x %x\n", pData[0], pData[39], pData[40], pData[1556]);

            eeprom_64align_write(ctx, data_base, pData, data_length);

            LOG_INF("com_0:0x%x\n", read_cmos_eeprom_8(ctx, data_base));
            LOG_INF("com_39:0x%x\n", read_cmos_eeprom_8(ctx, data_base+39));
            LOG_INF("innal_40:0x%x\n", read_cmos_eeprom_8(ctx, data_base+40));
            LOG_INF("innal_1556:0x%x\n", read_cmos_eeprom_8(ctx, data_base+1556));
            LOG_INF("write_Module_data Write end\n");

        } else if (((pStereodata->uSensorId == WHOOPASSC1MAIN_SENSOR_ID) || (pStereodata->uSensorId == WHOOPASSS1MAIN_SENSOR_ID)
            || (pStereodata->uSensorId == WHOOPASSC1MAIN2_SENSOR_ID) || (pStereodata->uSensorId == WHOOPASSS1MAIN2_SENSOR_ID)
            || (pStereodata->uSensorId == WHOOPASSC2MAIN_SENSOR_ID) || (pStereodata->uSensorId == WHOOPASSS2MAIN_SENSOR_ID)
            || (pStereodata->uSensorId == WHOOPASSC2MAIN2_SENSOR_ID) || (pStereodata->uSensorId == WHOOPASSS2MAIN2_SENSOR_ID))
            && (data_length < AESYNC_DATA_LENGTH_TOTAL)
            && (data_base == WHOOPASSC1MAIN_AESYNC_START_ADDR)) {
            LOG_INF("write main aesync: %x %x %x %x %x %x %x %x\n", pData[0], pData[1],
                pData[2], pData[3], pData[4], pData[5], pData[6], pData[7]);

            eeprom_64align_write(ctx, data_base, pData, data_length);

            LOG_INF("readback main aesync: %x %x %x %x %x %x %x %x\n",
                    read_cmos_eeprom_8(ctx, WHOOPASSC1MAIN_AESYNC_START_ADDR),
                    read_cmos_eeprom_8(ctx, WHOOPASSC1MAIN_AESYNC_START_ADDR+1),
                    read_cmos_eeprom_8(ctx, WHOOPASSC1MAIN_AESYNC_START_ADDR+2),
                    read_cmos_eeprom_8(ctx, WHOOPASSC1MAIN_AESYNC_START_ADDR+3),
                    read_cmos_eeprom_8(ctx, WHOOPASSC1MAIN_AESYNC_START_ADDR+4),
                    read_cmos_eeprom_8(ctx, WHOOPASSC1MAIN_AESYNC_START_ADDR+5),
                    read_cmos_eeprom_8(ctx, WHOOPASSC1MAIN_AESYNC_START_ADDR+6),
                    read_cmos_eeprom_8(ctx, WHOOPASSC1MAIN_AESYNC_START_ADDR+7));
            LOG_INF("AESync write_Module_data Write end\n");
        } else {
            LOG_INF("Invalid Sensor id:0x%x write eeprom\n", pStereodata->uSensorId);
            return -1;
        }
    } else {
        LOG_INF("whoopassc1main write_Module_data pStereodata is null\n");
        return -1;
    }
    return ret;
}

static int whoopassc1main_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
   int ret = ERROR_NONE;
   ret = write_Module_data(ctx, (ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(para));
   if (ret != ERROR_NONE) {
       *len = (u32)-1; /*write eeprom failed*/
       LOG_INF("ret=%d\n", ret);
   }
   return 0;
}

static int whoopassc1main_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	UINT16 *feature_data_16 = (UINT16 *) para;
	UINT32 *feature_return_para_32 = (UINT32 *) para;
	if(*len > CALI_DATA_MASTER_LENGTH)
		*len = CALI_DATA_MASTER_LENGTH;
	LOG_INF("feature_data mode: %d", *feature_data_16);
	switch (*feature_data_16) {
	case EEPROM_STEREODATA_MT_MAIN:
		read_whoopassc1main_eeprom_info(ctx, EEPROM_META_STEREO_MT_MAIN_DATA,
				(BYTE *)feature_return_para_32, *len);
		break;
	case EEPROM_STEREODATA_MW_MAIN:
		read_whoopassc1main_eeprom_info(ctx, EEPROM_META_STEREO_MW_MAIN_DATA,
				(BYTE *)feature_return_para_32, *len);
		break;
	case EEPROM_STEREODATA_MT_MAIN_105CM:
		read_whoopassc1main_eeprom_info(ctx, EEPROM_META_STEREO_MT_MAIN_DATA_105CM,
				(BYTE *)feature_return_para_32, *len);
		break;
	default:
		break;
	}
	return 0;
}

static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size)
{
	if (adaptor_i2c_rd_p8(ctx->i2c_client, WHOOPASSC1MAIN_EEPROM_ADDR >> 1,
			addr, data, size) < 0) {
		return false;
	}
	return true;
}

static int whoopassc1main_set_awb_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len) {

	struct SET_SENSOR_AWB_GAIN *awb_gain = (struct SET_SENSOR_AWB_GAIN *)para;

	adaptor_i2c_wr_u16(ctx->i2c_client, ctx->i2c_write_id >> 1, 0x0D82, awb_gain->ABS_GAIN_R * 2); //red 1024(1x)
	adaptor_i2c_wr_u16(ctx->i2c_client, ctx->i2c_write_id >> 1, 0x0D86, awb_gain->ABS_GAIN_B * 2); //blue

	LOG_INF("ABS_GAIN_GR(%d) ABS_GAIN_R(%d) ABS_GAIN_B(%d) ABS_GAIN_GB(%d)", awb_gain->ABS_GAIN_GR, awb_gain->ABS_GAIN_R, awb_gain->ABS_GAIN_B, awb_gain->ABS_GAIN_GB);
//	LOG_INF("0x0D82(red) = (0x%x)", subdrv_i2c_rd_u16(ctx, 0x0D82));
//	LOG_INF("0x0D84(green) = (0x%x)", subdrv_i2c_rd_u16(ctx, 0x0D84));
//	LOG_INF("0x0D86(blue) = (0x%x)", subdrv_i2c_rd_u16(ctx, 0x0D86));
	return 0;
}

static void read_otp_info(struct subdrv_ctx *ctx)
{
	DRV_LOGE(ctx, "read_otp_info begin\n");
	read_cmos_eeprom_p8(ctx, 0, otp_data_checksum, OTP_SIZE);
	DRV_LOGE(ctx, "read_otp_info end\n");
}

static int whoopassc1main_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;
	u32 length = sizeof(otp_data_checksum);

	if(*len < sizeof(otp_data_checksum)) {
		length = *len;
	}
	DRV_LOGE(ctx, "get otp data length:0x%x", length);

	if (otp_data_checksum[0] == 0) {
		read_otp_info(ctx);
	} else {
		DRV_LOG(ctx, "otp data has already read");
	}

	memcpy(feature_return_para_32, (UINT32 *)otp_data_checksum, length);
	return 0;
}

static int whoopassc1main_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	get_imgsensor_id(ctx, (u32 *)para);
	return 0;
}

static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id)
{
	u8 i = 0;
	u8 retry = GET_SENSOR_ID_RETRY_CNT;
	static bool first_read = TRUE;
	// u32 eeprom_time_year = 0, eeprom_time_m_d = 0;
	u32 addr_h = ctx->s_ctx.reg_addr_sensor_id.addr[0];
	u32 addr_l = ctx->s_ctx.reg_addr_sensor_id.addr[1];
	u32 addr_ll = ctx->s_ctx.reg_addr_sensor_id.addr[2];

	while (ctx->s_ctx.i2c_addr_table[i] != 0xFF) {
		ctx->i2c_write_id = ctx->s_ctx.i2c_addr_table[i];
		do {
			*sensor_id = (subdrv_i2c_rd_u8(ctx, addr_h) << 8) |
				subdrv_i2c_rd_u8(ctx, addr_l);
			if (addr_ll)
				*sensor_id = ((*sensor_id) << 8) | subdrv_i2c_rd_u8(ctx, addr_ll);
			DRV_LOGE(ctx, "i2c_write_id(0x%x) sensor_id(0x%x/0x%x)\n",
				ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == 0x1B75) {
				get_imgsensor_id_from_dts(ctx, sensor_id);
				if (first_read) {
					//update CTLE
					update_CTLE(ctx);
					read_eeprom_common_data(ctx, &oplus_eeprom_info, oplus_eeprom_addr_table);
					first_read = FALSE;
					module_flag = subdrv_i2c_rd_u16(ctx, 0x0010);
					msg_buf = kmalloc(MAX_BURST_LEN, GFP_KERNEL);
					if(!msg_buf) {
						LOG_INF("boot stage, malloc msg_buf error");
					}
				}
				// eeprom_time_year = (read_cmos_eeprom_8(ctx, 0x0004) << 8) | read_cmos_eeprom_8(ctx, 0x0005);
				// eeprom_time_m_d = (read_cmos_eeprom_8(ctx, 0x0003) << 8) | read_cmos_eeprom_8(ctx, 0x0002);
				// sensor with eeprom data since 2024/01/27
				// module_flag = (eeprom_time_year > 0x1814) || ((eeprom_time_year == 0x1814) && (eeprom_time_m_d >= 0x11B));
				return ERROR_NONE;
			}
			DRV_LOGE(ctx, "Read sensor id fail. i2c_write_id: 0x%x\n", ctx->i2c_write_id);
			DRV_LOGE(ctx, "sensor_id = 0x%x, ctx->s_ctx.sensor_id = 0x%x\n",
				*sensor_id, ctx->s_ctx.sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = GET_SENSOR_ID_RETRY_CNT;
	}
	if (*sensor_id != ctx->s_ctx.sensor_id) {
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}

static int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;
	u32 module_info = 0;
	u64 time_boot_begin = 0;

	/* get sensor id */
	if (get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;
	subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x4000);
	module_info = subdrv_i2c_rd_u16(ctx, 0x0010);
	DRV_LOG(ctx, "write init setting +");
	if ((ctx->power_on_profile_en != NULL) && (*ctx->power_on_profile_en))
		time_boot_begin = ktime_get_boottime_ns();


	DRV_LOGE(ctx, "module_flag = 0x%x\n", module_flag);
	if ((module_flag & 0xFF00) == 0x0200){
		subdrv_i2c_wr_regs_u16(ctx, whoopassc1main_init_setting_part1, ARRAY_SIZE(whoopassc1main_init_setting_part1));
		mdelay(1);
		subdrv_i2c_wr_regs_u16(ctx, whoopassc1main_init_setting_part2, ARRAY_SIZE(whoopassc1main_init_setting_part2));
		mdelay(1);
		whoopassc1main_i2c_burst_wr_regs_u16(ctx, whoopassc1main_init_setting_part3_094, ARRAY_SIZE(whoopassc1main_init_setting_part3_094));
		ctx->s_ctx.init_setting_len = ARRAY_SIZE(whoopassc1main_init_setting_part1)
											+ ARRAY_SIZE(whoopassc1main_init_setting_part2)
											+ ARRAY_SIZE(whoopassc1main_init_setting_part3_094);
	}else{
		subdrv_i2c_wr_regs_u16(ctx, whoopassc1main_init_setting_part1, ARRAY_SIZE(whoopassc1main_init_setting_part1));
		mdelay(1);
		subdrv_i2c_wr_regs_u16(ctx, whoopassc1main_init_setting_part2, ARRAY_SIZE(whoopassc1main_init_setting_part2));
		mdelay(1);
		whoopassc1main_i2c_burst_wr_regs_u16(ctx, whoopassc1main_init_setting_part3_092, ARRAY_SIZE(whoopassc1main_init_setting_part3_092));
		ctx->s_ctx.init_setting_len = ARRAY_SIZE(whoopassc1main_init_setting_part1)
											+ ARRAY_SIZE(whoopassc1main_init_setting_part2)
											+ ARRAY_SIZE(whoopassc1main_init_setting_part3_092);
	}

	DRV_LOGE(ctx, "setting end! S: size:%u\n", ctx->s_ctx.init_setting_len);

	if (ctx->s_ctx.temperature_support && ctx->s_ctx.reg_addr_temp_en)
		subdrv_ixc_wr_u8(ctx, ctx->s_ctx.reg_addr_temp_en, 0x01);
	/* enable mirror or flip */
	set_mirror_flip(ctx, ctx->mirror);

	  	// XTC
  	if (ctx->s_ctx.s_cali != NULL) {
  		ctx->s_ctx.s_cali((void*)ctx);
  	}

	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	memset(ctx->ana_gain, 0, sizeof(ctx->gain));
	ctx->exposure[0] = ctx->s_ctx.exposure_def;
	ctx->ana_gain[0] = ctx->s_ctx.ana_gain_def;
	ctx->current_scenario_id = scenario_id;
	ctx->pclk = ctx->s_ctx.mode[scenario_id].pclk;
	ctx->line_length = ctx->s_ctx.mode[scenario_id].linelength;
	ctx->frame_length = ctx->s_ctx.mode[scenario_id].framelength;
	ctx->current_fps = 10 * ctx->pclk / ctx->line_length / ctx->frame_length;
	ctx->readout_length = ctx->s_ctx.mode[scenario_id].readout_length;
	ctx->read_margin = ctx->s_ctx.mode[scenario_id].read_margin;
	ctx->min_frame_length = ctx->frame_length;
	ctx->autoflicker_en = FALSE;
	ctx->test_pattern = 0;
	ctx->ihdr_mode = 0;
	ctx->pdaf_mode = 0;
	ctx->hdr_mode = 0;
	ctx->extend_frame_length_en = 0;
	ctx->is_seamless = 0;
	ctx->fast_mode_on = 0;
	ctx->sof_cnt = 0;
	ctx->ref_sof_cnt = 0;
	ctx->is_streaming = 0;

	return ERROR_NONE;
}
static void set_group_hold(void *arg, u8 en)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	if (group_hold_frame_count < 2) {
		DRV_LOGE(ctx, "group_hold_frame_count: %d", group_hold_frame_count);
		group_hold_frame_count++;
		return;
	}
	if (en)
		set_i2c_buffer(ctx, 0x0104, 0x01);
	else
		set_i2c_buffer(ctx, 0x0104, 0x00);
}

static u16 get_gain2reg(u32 gain)
{
	return gain * 32 / BASEGAIN;
}

static int whoopassc1main_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	group_hold_frame_count = 0;
	enum SENSOR_SCENARIO_ID_ENUM scenario_id;
	struct mtk_hdr_ae *ae_ctrl = NULL;
	u64 *feature_data = (u64 *)para;
	u32 frame_length_in_lut[IMGSENSOR_STAGGER_EXPOSURE_CNT] = {0};
	u32 exp_cnt = 0;

	if (feature_data == NULL) {
		DRV_LOGE(ctx, "input scenario is null!");
		return ERROR_NONE;
	}
	scenario_id = *feature_data;
	if ((feature_data + 1) != NULL)
		ae_ctrl = (struct mtk_hdr_ae *)((uintptr_t)(*(feature_data + 1)));
	else
		DRV_LOGE(ctx, "no ae_ctrl input");

	check_current_scenario_id_bound(ctx);
	LOG_INF("E: set seamless switch %u %u\n", ctx->current_scenario_id, scenario_id);
	if (!ctx->extend_frame_length_en)
		DRV_LOGE(ctx, "please extend_frame_length before seamless_switch!\n");
	ctx->extend_frame_length_en = FALSE;

	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOGE(ctx, "invalid sid:%u, mode_num:%u\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		return ERROR_NONE;
	}

	if (ctx->s_ctx.mode[scenario_id].seamless_switch_group == 0 ||
		ctx->s_ctx.mode[scenario_id].seamless_switch_group !=
			ctx->s_ctx.mode[ctx->current_scenario_id].seamless_switch_group) {
		DRV_LOGE(ctx, "seamless_switch not supported\n");
		return ERROR_NONE;
	}
	if (ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table == NULL) {
		DRV_LOGE(ctx, "Please implement seamless_switch setting\n");
		return ERROR_NONE;
	}

	exp_cnt = ctx->s_ctx.mode[scenario_id].exp_cnt;
	ctx->is_seamless = TRUE;

	// subdrv_ixc_wr_u8(ctx, 0x0104, 0x01);
	set_i2c_buffer(ctx, 0x0104, 0x01);

	if (ctx->s_ctx.reg_addr_fast_mode_in_lbmf &&
		(ctx->s_ctx.mode[scenario_id].hdr_mode == HDR_RAW_LBMF ||
		ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode == HDR_RAW_LBMF))
		subdrv_ixc_wr_u8(ctx, ctx->s_ctx.reg_addr_fast_mode_in_lbmf, 0x4);
	update_mode_info(ctx, scenario_id);

	// ixc_table_write(ctx,
	// 	ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table,
	// 	ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_len);

	for (int i = 0; i < ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_len; i += 2) {
		set_i2c_buffer(ctx, ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table[i],
			(ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table[i + 1] >> 8) & 0xFF);
		set_i2c_buffer(ctx, ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table[i] + 1,
			(ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table[i + 1]) & 0xFF);
	}
	commit_i2c_buffer(ctx);

	if (ae_ctrl) {
		switch (ctx->s_ctx.mode[scenario_id].hdr_mode) {
		case HDR_RAW_STAGGER:
			whoopassc1main_set_multi_shutter_frame_length(ctx, (u64 *)&ae_ctrl->exposure, exp_cnt, 0);
			whoopassc1main_set_multi_gain(ctx, (u32 *)&ae_ctrl->gain, exp_cnt);
			break;
		case HDR_RAW_LBMF:
			set_multi_shutter_frame_length_in_lut(ctx,
				(u64 *)&ae_ctrl->exposure, exp_cnt, 0, frame_length_in_lut);
			set_multi_gain_in_lut(ctx, (u32 *)&ae_ctrl->gain, exp_cnt);
			break;
		case HDR_RAW_DCG_RAW:
			whoopassc1main_set_multi_shutter_frame_length(ctx, (u64 *)&ae_ctrl->exposure, 1, 0);
			if (ctx->s_ctx.mode[scenario_id].dcg_info.dcg_gain_mode
				== IMGSENSOR_DCG_DIRECT_MODE)
				whoopassc1main_set_multi_gain(ctx, (u32 *)&ae_ctrl->gain, exp_cnt);
			else
				whoopassc1main_set_gain_convert(ctx, ae_ctrl->gain.me_gain);
			break;
		default:
			whoopassc1main_set_shutter_convert(ctx, ae_ctrl->exposure.le_exposure);
			whoopassc1main_set_gain_convert(ctx, ae_ctrl->gain.le_gain);
			break;
		}
	}
	LOG_INF("write seamless switch para done\n");

	set_i2c_buffer(ctx, 0x0104, 0x00);
	commit_i2c_buffer(ctx);

	// ctx->fast_mode_on = TRUE;
	ctx->ref_sof_cnt = ctx->sof_cnt;
	ctx->is_seamless = FALSE;
	DRV_LOG(ctx, "X: set seamless switch done\n");
	return ERROR_NONE;
}

static int whoopassc1main_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	if (mode != ctx->test_pattern)
		DRV_LOG(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
	/* 1:Solid Color 2:Color Bar 5:Black */
	if (mode) {
		if (mode == 5) {
			subdrv_i2c_wr_u16(ctx, 0x0600, 0x0001); /*black*/
			subdrv_i2c_wr_u16(ctx, 0x0620, 0x0001); /*black*/
		} else {
			subdrv_i2c_wr_u16(ctx, 0x0600, mode); /*100% Color bar*/
			subdrv_i2c_wr_u16(ctx, 0x0620, mode); /*100% Color bar*/
		}
	} else if (ctx->test_pattern) {
		subdrv_i2c_wr_u16(ctx, 0x0600, 0x0000); /*No pattern*/
		subdrv_i2c_wr_u16(ctx, 0x0620, 0x0000); /*No pattern*/
	}

	ctx->test_pattern = mode;
	return 0;
}

static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id)
{
	memcpy(&(ctx->s_ctx), &static_ctx, sizeof(struct subdrv_static_ctx));
	subdrv_ctx_init(ctx);
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;
	return 0;
}

void whoopassc1main_get_sensor_cali(void* arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	// struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;

	/* Probe EEPROM device */
	if (!probe_eeprom(ctx))
		return;

	ctx->is_read_preload_eeprom = 1;
}

static void whoopassc1main_set_sensor_cali(void *arg)
{
	return;
}

static bool dump_i2c_enable = false;

static void dump_i2c_buf(struct subdrv_ctx *ctx, u8 * buf, u32 length)
{
	int i;
	char *out_str = NULL;
	char *strptr = NULL;
	size_t buf_size = SUBDRV_I2C_BUF_SIZE * sizeof(char);
	size_t remind = buf_size;
	int num = 0;

	out_str = kzalloc(buf_size + 1, GFP_KERNEL);
	if (!out_str)
		return;

	strptr = out_str;
	memset(out_str, 0, buf_size + 1);

	num = snprintf(strptr, remind,"[ ");
	remind -= num;
	strptr += num;

	for (i = 0 ; i < length; i ++) {
		num = snprintf(strptr, remind,"0x%02x, ", buf[i]);

		if (num <= 0) {
			DRV_LOG(ctx, "snprintf return negative at line %d\n", __LINE__);
			kfree(out_str);
			return;
		}

		remind -= num;
		strptr += num;

		if (remind <= 20) {
			DRV_LOG(ctx, " write %s\n", out_str);
			memset(out_str, 0, buf_size + 1);
			strptr = out_str;
			remind = buf_size;
		}
	}

	num = snprintf(strptr, remind," ]");
	remind -= num;
	strptr += num;

	DRV_LOG(ctx, " write %s\n", out_str);
	strptr = out_str;
	remind = buf_size;

	kfree(out_str);
}

static int whoopassc1main_i2c_burst_wr_regs_u16(struct subdrv_ctx * ctx, u16 * list, u32 len)
{
	adapter_i2c_burst_wr_regs_u16(ctx, ctx->i2c_write_id >> 1, list, len);
	return 	0;
}

/* #define MAX_BUF_SIZE  4096
#define MAX_MSG_NUM_U16  MAX_BUF_SIZE/4

 struct cache_wr_regs_u16 {
	struct i2c_msg msg[MAX_MSG_NUM_U16];adapter_i2c_burst_wr_regs_u16
}; */

static int adapter_i2c_burst_wr_regs_u16(struct subdrv_ctx * ctx ,
		u16 addr, u16 *list, u32 len)
{
	struct i2c_client *i2c_client = ctx->i2c_client;
	struct i2c_msg  msg;
	struct i2c_msg *pmsg = &msg;

	u8 *pbuf = NULL;
	u16 *plist = NULL;
	u16 *plist_end = NULL;

	u32 sent = 0;
	u32 total = 0;
	u32 per_sent = 0;
	int ret, i;

	if(!msg_buf) {
		LOG_INF("malloc msg_buf retry");
		msg_buf = kmalloc(MAX_BURST_LEN, GFP_KERNEL);
		if(!msg_buf) {
			LOG_INF("malloc error");
			return -ENOMEM;
		}
	}

	/* each msg contains addr(u16) + val(u16 *) */
	sent = 0;
	total = len / 2;
	plist = list;
	plist_end = list + len - 2;

	DRV_LOG(ctx, "len(%u)  total(%u)", len, total);

	while (sent < total) {

		per_sent = 0;
		pmsg = &msg;
		pbuf = msg_buf;

		pmsg->addr = addr;
		pmsg->flags = i2c_client->flags;
		pmsg->buf = pbuf;

		pbuf[0] = plist[0] >> 8;    //address
		pbuf[1] = plist[0] & 0xff;

		pbuf[2] = plist[1] >> 8;  //data 1
		pbuf[3] = plist[1] & 0xff;

		pbuf += 4;
		pmsg->len = 4;
		per_sent += 1;

		for (i = 0; i < total - sent - 1; i++) {  //Maximum number of remaining cycles - 1
			if(plist[0] + 2 == plist[2] ) {  //Addresses are consecutive
				pbuf[0] = plist[3] >> 8;
				pbuf[1] = plist[3] & 0xff;

				pbuf += 2;
				pmsg->len += 2;
				per_sent += 1;
				plist += 2;

				if(pmsg->len >= MAX_BURST_LEN) {
					break;
				}
			}
		}
		plist += 2;

		if(dump_i2c_enable) {
			DRV_LOG(ctx, "pmsg->len(%d) buff: ", pmsg->len);
			dump_i2c_buf(ctx, msg_buf, pmsg->len);
		}

		ret = i2c_transfer(i2c_client->adapter, pmsg, 1);

		if (ret < 0) {
			dev_info(&i2c_client->dev,
				"i2c transfer failed (%d)\n", ret);
			return -EIO;
		}

		sent += per_sent;

		DRV_LOG(ctx, "sent(%u)  total(%u)  per_sent(%u)", sent, total, per_sent);
	}

	return 0;
}

/*
static int whoopassc1main_chk_streaming_st(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	int ret = ERROR_NONE;
	u32 i = 0, framecnt = 0;
	int timeout = ctx->current_fps ? (10000 / ctx->current_fps) + 1 : 101;

	if (!ctx->s_ctx.reg_addr_frame_count)
		return ret;
	subdrv_ixc_wr_u16(ctx, 0xFCFC, 0x4000);
	for (i = 0; i < timeout; i++) {
		framecnt = subdrv_i2c_rd_u8(ctx, ctx->s_ctx.reg_addr_frame_count);
		DRV_LOG(ctx, "check_stream_on delay cnt:%d, framecnt:%d\n", i, framecnt);
		if (framecnt != 0xFF)
			return ret;
		mdelay(1);
	}
	DRV_LOGE(ctx, "stream on fail!\n");
	return ret;
}
*/
static void whoopassc1main_get_sensorname(struct subdrv_ctx *ctx){
	u8 BG_flag = 0;
	u8 BARCODE_flags[2] = {0};
	BG_flag = read_cmos_eeprom_8(ctx, OTP_BG_ADDR);
	for (int i = 0; i < 2; i++) {
        BARCODE_flags[i] = read_cmos_eeprom_8(ctx, OTP_BARCODE_ADDR + i);
    }
	if(BARCODE_flags[0] == 0x37 && BARCODE_flags[1] == 0x33){
		if (is_c1sensor) {
			SENSOR_NAME = sensor_name[2];
		} else {
			SENSOR_NAME = sensor_name[1];
		}
	} else{
		if(BG_flag == 0x01){
			if (is_c1sensor) {
				SENSOR_NAME = sensor_name[2];
			} else {
				SENSOR_NAME = sensor_name[1];
			}
		} else {
			SENSOR_NAME = sensor_name[0];
		}
	}
	LOG_INF("BG_flag:%x, BARCODE_flags:%02x %02x, SENSOR_NAME:%s",
            BG_flag, BARCODE_flags[0], BARCODE_flags[1], SENSOR_NAME);
	return;
}

static void get_imgsensor_id_from_dts(struct subdrv_ctx *ctx, u32 *sensor_id) {
	struct subdrv_entry *m_subdrv_entry = &whoopassc1main_mipi_raw_entry;
	u32 final_sensor_id = 0xFFFFFFFF;
	const char *of_sensor_names[OF_SENSOR_NAMES_MAXCNT];
	const char *of_sensor_hal_names[OF_SENSOR_NAMES_MAXCNT];
	u32   of_sensor_ids[OF_SENSOR_NAMES_MAXCNT] = {0};
	int i, index, of_sensor_names_cnt, of_sensor_hal_names_cnt, of_sensor_ids_ret;
	struct device *dev = &ctx->i2c_client->dev;

	memset(&of_sensor_ids, 0xFF, sizeof(of_sensor_ids));

	if(g_id_from_dts_flag == false) {
		of_sensor_names_cnt = of_property_read_string_array(dev->of_node,
			"sensor-names", of_sensor_names, ARRAY_SIZE(of_sensor_names));

		of_sensor_hal_names_cnt = of_property_read_string_array(dev->of_node,
			"sensor-hal-names", of_sensor_hal_names, ARRAY_SIZE(of_sensor_hal_names));

		of_sensor_ids_ret = of_property_read_u32_array(dev->of_node,
				"sensor-ids", of_sensor_ids, of_sensor_names_cnt);

		pr_err("%s of_sensor_names_cnt(%d), of_sensor_ids_ret(%d)",
			__func__, of_sensor_names_cnt, of_sensor_ids_ret);
		for(i = 0 ;i < of_sensor_names_cnt; i++) {
				pr_err("%s of_sensor_names[%d] = %s  of_sensor_ids[%d] = %d",
				__func__, i, of_sensor_names[i], i, of_sensor_ids[i]);
		}
		for(i = 0 ;i < of_sensor_hal_names_cnt; i++) {
			pr_err("%s of_sensor_hal_names_cnt[%d] = %s",
				__func__, i, of_sensor_hal_names[i]);
		}

		if ((of_sensor_ids[0] == WHOOPASSC1MAIN_SENSOR_ID) || (of_sensor_ids[0] == WHOOPASSS1MAIN_SENSOR_ID)) {
			is_c1sensor = true;
		} else {
			is_c1sensor = false;
		}
		if (of_sensor_names_cnt && (of_sensor_ids_ret == 0)) {
			for(index = 0; index < of_sensor_names_cnt; index++) {
				whoopassc1main_get_sensorname(ctx);
				LOG_INF("SENSOR_NAME:%s", SENSOR_NAME);
				if (strncmp(SENSOR_NAME, of_sensor_names[index], strlen(SENSOR_NAME)) == 0) {
					final_sensor_id = of_sensor_ids[index];
					break;
				}
			}
		} else {
			pr_err("%s sensor-ids error in dts", __func__);
		}
		g_id_from_dts_flag = true;
	}

	if(final_sensor_id != 0xFFFFFFFF) {
		*sensor_id = final_sensor_id;
		ctx->s_ctx.sensor_id = final_sensor_id;

		m_subdrv_entry->id = final_sensor_id;
		if(of_sensor_hal_names_cnt == of_sensor_names_cnt) {
			m_subdrv_entry->name = of_sensor_hal_names[index];
		}

		pr_err("%s final index(%d), id(%d) name(%s)",
			__func__, index, m_subdrv_entry->id, m_subdrv_entry->name);
	} else {
		*sensor_id = ctx->s_ctx.sensor_id;
	}

	return;
}

void whoopassc1main_write_frame_length(struct subdrv_ctx *ctx, u32 fll)
{
	u32 addr_h = ctx->s_ctx.reg_addr_frame_length.addr[0];
	u32 addr_l = ctx->s_ctx.reg_addr_frame_length.addr[1];
	u32 addr_ll = ctx->s_ctx.reg_addr_frame_length.addr[2];
	u32 fll_step = 0;
	u32 dol_cnt = 1;

	check_current_scenario_id_bound(ctx);

	switch (ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode) {
	case HDR_RAW_STAGGER:
	case HDR_RAW_LBMF:
		dol_cnt = ctx->s_ctx.mode[ctx->current_scenario_id].exp_cnt;
		break;
	default:
		break;
	}

	fll_step = ctx->s_ctx.mode[ctx->current_scenario_id].framelength_step;

	ctx->frame_length = fll;

	fll = fll / dol_cnt;

	if (fll_step)
		fll = round_up(fll, fll_step);

	if (ctx->extend_frame_length_en == FALSE) {
		if (addr_ll) {
			set_i2c_buffer(ctx,	addr_h,	(fll >> 16) & 0xFF);
			set_i2c_buffer(ctx,	addr_l, (fll >> 8) & 0xFF);
			set_i2c_buffer(ctx,	addr_ll, fll & 0xFF);
		} else {
			set_i2c_buffer(ctx,	addr_h, (fll >> 8) & 0xFF);
			set_i2c_buffer(ctx,	addr_l, fll & 0xFF);
		}
		ctx->frame_length_rg = ctx->frame_length;
		LOG_INF("ctx->frame_length_rg : %u\n", ctx->frame_length_rg);
	}
	LOG_INF("fll[0x%x] multiply %u, fll_step:%u ctx->extend_frame_length_en:%d\n",
		fll, dol_cnt, fll_step, ctx->extend_frame_length_en);
}

static void whoopassc1main_set_hdr_tri_shutter(struct subdrv_ctx *ctx, u64 *shutters, u16 exp_cnt)
{
	int i = 0;
	u64 values[3] = {0};
	u32 frame_length_in_lut[IMGSENSOR_STAGGER_EXPOSURE_CNT] = {0};

	if (shutters != NULL) {
		for (i = 0; i < 3; i++)
			values[i] = (u64) *(shutters + i);
	}
	if (ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode == HDR_RAW_LBMF) {
			set_multi_shutter_frame_length_in_lut(ctx,
				values, exp_cnt, 0, frame_length_in_lut);
		return;
	}
	whoopassc1main_set_multi_shutter_frame_length(ctx, values, exp_cnt, 0);
}

static int whoopassc1main_set_hdr_tri_shutter2(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	whoopassc1main_set_hdr_tri_shutter(ctx, feature_data, 2);
	return 0;
}

static int whoopassc1main_set_hdr_tri_shutter3(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	whoopassc1main_set_hdr_tri_shutter(ctx, feature_data, 3);
	return 0;
}


bool whoopassc1main_set_long_exposure(struct subdrv_ctx *ctx,  u64 shutter)
{
	u16 l_shift = 0;
	u16 l_shift_Max = 9;
	u8 exposure_margin = ctx->s_ctx.mode[ctx->current_scenario_id].exposure_margin ?
		ctx->s_ctx.mode[ctx->current_scenario_id].exposure_margin : ctx->s_ctx.exposure_margin;
	if (shutter > (ctx->s_ctx.frame_length_max - exposure_margin)) {
		if (ctx->s_ctx.long_exposure_support == FALSE) {
			DRV_LOGE(ctx, "sensor no support of exposure lshift!\n");
			return FALSE;
		}
		if (ctx->s_ctx.reg_addr_exposure_lshift == PARAM_UNDEFINED) {
			DRV_LOGE(ctx, "please implement lshift register address\n");
			return FALSE;
		}
		for (l_shift = 1; l_shift < l_shift_Max; l_shift++) {
			if ((shutter >> l_shift)
				< (ctx->s_ctx.frame_length_max - exposure_margin))
				break;
		}
		if (l_shift > l_shift_Max) {
			DRV_LOGE(ctx, "unable to set exposure:%llu, set to max\n", shutter);
			l_shift = l_shift_Max;
		}
		shutter = shutter >> l_shift;
//		if (!ctx->s_ctx.reg_addr_auto_extend)
//			ctx->frame_length = shutter + exposure_margin;
		LOG_INF("long exposure mode: lshift %u times   shutter:%llu", l_shift, shutter);

		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_frame_length.addr[0], (shutter >> 8) & 0xFF);
		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_frame_length.addr[1],  shutter & 0xFF);

		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[0].addr[0], (shutter >> 8) & 0xFF);
		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[0].addr[1],  shutter & 0xFF);

		set_i2c_buffer(ctx, 0x0702, l_shift);
		set_i2c_buffer(ctx, 0x0704, l_shift);

		bNeedSetNormalMode = TRUE;
		/* Frame exposure mode customization for LE*/
		ctx->ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		ctx->ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
//		ctx->current_ae_effective_frame = 2;
		return TRUE;

	} else {
		if (bNeedSetNormalMode) {
			LOG_INF("exit long exposure\n");
			set_i2c_buffer(ctx, 0x0702, 0x00);
			set_i2c_buffer(ctx, 0x0704, 0x00);
			bNeedSetNormalMode = FALSE;
		}
		return FALSE;
//		ctx->current_ae_effective_frame = 2;
	}

//	ctx->exposure[IMGSENSOR_STAGGER_EXPOSURE_LE] = shutter;
	return FALSE;
}

static void whoopassc1main_set_multi_shutter_frame_length(struct subdrv_ctx *ctx, u64 *shutters, u16 exp_cnt, u16 frame_length)
{
	int i = 0;
	u16 last_exp_cnt = 1;
	u32 calc_fl[4] = {0};
	int readout_diff = 0;
	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);
	u32 rg_shutters[3] = {0};
	u32 cit_step = 0;
	u32 fll = 0, fll_temp = 0, s_fll;
	u8 exposure_margin = ctx->s_ctx.mode[ctx->current_scenario_id].exposure_margin ?
		ctx->s_ctx.mode[ctx->current_scenario_id].exposure_margin : ctx->s_ctx.exposure_margin;

	if(exp_cnt == 1) {  //force to 1exp func
  		whoopassc1main_set_shutter_frame_length_convert(ctx, shutters[0], frame_length);
  		return ;
  	}else if (exp_cnt == 2) {
		LOG_INF("shutter[0]:%llu, shutter[1]:%llu, exp_cnt:%d, frame_length:%u exposure_margin:%d cit_step:%d\n",
			shutters[0], shutters[1], exp_cnt, frame_length, exposure_margin, cit_step);
		if (shutters[1] > shutters[0]) {
			LOG_INF("error short shutter > long shutter");
			shutters[1] = shutters[0];
		}
	} else {
		LOG_INF("shutter[0]:%llu, exp_cnt:%d, frame_length:%u, exposure_margin:%d cit_step:%d\n",
			shutters[0], exp_cnt, frame_length, exposure_margin, cit_step);
	}

	fll = frame_length ? frame_length : ctx->frame_length;
	if (exp_cnt > ARRAY_SIZE(ctx->exposure)) {
		DRV_LOGE(ctx, "invalid exp_cnt:%u>%lu\n", exp_cnt, ARRAY_SIZE(ctx->exposure));
		exp_cnt = ARRAY_SIZE(ctx->exposure);
	}
	check_current_scenario_id_bound(ctx);

	/* check boundary of framelength */
	/* - (1) previous se + previous me + current le */ //N+1 framelength
	calc_fl[0] = (u32) shutters[0];
	for (i = 1; i < last_exp_cnt; i++)
		calc_fl[0] += ctx->exposure[i];
	calc_fl[0] += ctx->s_ctx.mode[ctx->current_scenario_id].exposure_margin*exp_cnt*exp_cnt;

	/* - (2) current se + current me + current le */
	calc_fl[1] = (u32) shutters[0];
	for (i = 1; i < exp_cnt; i++)
		calc_fl[1] += (u32) shutters[i];
	calc_fl[1] += ctx->s_ctx.mode[ctx->current_scenario_id].exposure_margin*exp_cnt*exp_cnt;

	/* - (3) readout time cannot be overlapped */
	calc_fl[2] =
		(ctx->s_ctx.mode[ctx->current_scenario_id].readout_length +
		ctx->s_ctx.mode[ctx->current_scenario_id].read_margin);
	if (last_exp_cnt == exp_cnt)
		for (i = 1; i < exp_cnt; i++) {
			readout_diff = ctx->exposure[i] - (u32) shutters[i];
			calc_fl[2] += readout_diff > 0 ? readout_diff : 0;
		}
	/* - (4) For DOL (non-FDOL), N-th frame SE and N+1-th frame LE readout cannot be overlapped */
	if ((ctx->s_ctx.hdr_type & HDR_SUPPORT_STAGGER_DOL) &&
		ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode == HDR_RAW_STAGGER) {
		for (i = 1; i < last_exp_cnt; i++)
			calc_fl[3] += ctx->exposure[i];
		calc_fl[3] += ctx->s_ctx.mode[ctx->current_scenario_id].exposure_margin*exp_cnt*(exp_cnt-1);
		calc_fl[3] += ctx->readout_length + ctx->min_vblanking_line;
		DRV_LOG(ctx,
			"calc_fl[3]: %u, pre-LE/ME/SE (%u/%u/%u), cur-LE/ME/SE (%llu/%llu/%llu), readout_length:%u, min_vblanking_line:%u\n",
			calc_fl[3],
			ctx->exposure[0], ctx->exposure[1], ctx->exposure[2],
			shutters[0], shutters[1], shutters[2],
			ctx->readout_length,
			ctx->min_vblanking_line);
	}

	for (i = 0; i < ARRAY_SIZE(calc_fl); i++)
		fll = max(fll, calc_fl[i]);
	fll =	max(fll, ctx->min_frame_length);
	fll =	min(fll, ctx->s_ctx.frame_length_max);
	/* restore shutter */
	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	for (i = 0; i < exp_cnt; i++)
		ctx->exposure[i] = shutters[i];

	if ((ctx->exposure[0] < 0xFFF0) && bNeedSetNormalMode) {
		DRV_LOG(ctx, "exit long shutter\n");
		subdrv_i2c_wr_u16(ctx, 0x0702, 0x0000);
		subdrv_i2c_wr_u16(ctx, 0x0704, 0x0000);
		bNeedSetNormalMode = FALSE;
	}
	/* group hold start */
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 1);
	/* enable auto extend */
	if (ctx->s_ctx.reg_addr_auto_extend)
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_auto_extend, 0x01);

	if (ctx->s_ctx.mode[ctx->current_scenario_id].sw_fl_delay) {
		fll_temp = ctx->frame_length_next;
		ctx->frame_length_next = fll;
		s_fll = calc_fl[0];
		for (i = 1; i < ARRAY_SIZE(calc_fl); i++)
			s_fll = max(s_fll, calc_fl[i]);
		if (s_fll > frame_length) {
			fll = s_fll;
			fll = max(fll, ctx->min_frame_length);
			fll = min(fll, ctx->s_ctx.frame_length_max);
			if(fll<fll_temp)
				fll = fll_temp;
		} else {
			if (s_fll > fll_temp)
				fll = s_fll;
			else
				fll = fll_temp;
		}
		DRV_LOG(ctx, "fll:%u, s_fll:%u, fll_temp:%u, frame_length:%u\n",
			fll, s_fll, fll_temp, frame_length);
	}
	ctx->frame_length = fll;
	/* write framelength */
	if (set_auto_flicker(ctx, 0) || frame_length || !ctx->s_ctx.reg_addr_auto_extend)
		whoopassc1main_write_frame_length(ctx, ctx->frame_length);
	else if (ctx->s_ctx.reg_addr_auto_extend)
		whoopassc1main_write_frame_length(ctx, ctx->min_frame_length);
	/* write shutter */
	switch (exp_cnt) {
	case 1:
		rg_shutters[0] = shutters[0] / exp_cnt;
		break;
	case 2:
		rg_shutters[0] = shutters[0] / exp_cnt;
		rg_shutters[2] = shutters[1] / exp_cnt;
		break;
	case 3:
		rg_shutters[0] = shutters[0] / exp_cnt;
		rg_shutters[1] = shutters[1] / exp_cnt;
		rg_shutters[2] = shutters[2] / exp_cnt;
		break;
	default:
		break;
	}
	if (ctx->s_ctx.reg_addr_exposure_lshift != PARAM_UNDEFINED)
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_exposure_lshift, 0);

	for (i = 0; i < 3; i++) {
		if (rg_shutters[i]) {
			if (ctx->s_ctx.reg_addr_exposure[i].addr[2]) {
				set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[i].addr[0],
					(rg_shutters[i] >> 16) & 0xFF);
				set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[i].addr[1],
					(rg_shutters[i] >> 8) & 0xFF);
				set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[i].addr[2],
					rg_shutters[i] & 0xFF);
			} else {
				set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[i].addr[0],
					(rg_shutters[i] >> 8) & 0xFF);
				set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[i].addr[1],
					rg_shutters[i] & 0xFF);
			}
		}
	}
	LOG_INF("exp[0x%x/0x%x/0x%x], fll(input/output):%u/%u, flick_en:%u\n",
		rg_shutters[0], rg_shutters[1], rg_shutters[2],
		frame_length, ctx->frame_length, ctx->autoflicker_en);
	if (!ctx->ae_ctrl_gph_en) {
		if (gph)
			ctx->s_ctx.s_gph((void *)ctx, 0);
		commit_i2c_buffer(ctx);
	}
	/* group hold end */

}

static int whoopassc1main_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	whoopassc1main_set_multi_shutter_frame_length(ctx, (u64 *)(*feature_data),
		(u64) (*(feature_data + 1)), (u64) (*(feature_data + 2)));
	return 0;
}


static void whoopassc1main_set_shutter_frame_length_convert(struct subdrv_ctx *ctx, u64 shutter, u32 frame_length)
{
	u32 fine_integ_line = 0;
	u32 cit_step = 0;

	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);
	u8 exposure_margin = ctx->s_ctx.mode[ctx->current_scenario_id].exposure_margin ?
		ctx->s_ctx.mode[ctx->current_scenario_id].exposure_margin : ctx->s_ctx.exposure_margin;
	LOG_INF("shutter:%llu, frame_length:%u  exposure_margin:%d\n", shutter, frame_length, exposure_margin);

	ctx->frame_length = frame_length ? frame_length : ctx->frame_length;
	check_current_scenario_id_bound(ctx);
	/* check boundary of shutter */
	fine_integ_line = ctx->s_ctx.mode[ctx->current_scenario_id].fine_integ_line;
	shutter = FINE_INTEG_CONVERT(shutter, fine_integ_line);
  	shutter = max_t(u64, shutter,
  		(u64)ctx->s_ctx.mode[ctx->current_scenario_id].multi_exposure_shutter_range[0].min);
  	shutter = min_t(u64, shutter,
  		(u64)ctx->s_ctx.mode[ctx->current_scenario_id].multi_exposure_shutter_range[0].max);
	/* check boundary of framelength */

	cit_step = ctx->s_ctx.mode[ctx->current_scenario_id].coarse_integ_step;
	if (cit_step)
		shutter = round_up(shutter, cit_step);

	ctx->frame_length =	max(shutter + exposure_margin, ctx->frame_length);
	ctx->frame_length =	min(ctx->frame_length, ctx->s_ctx.frame_length_max);
	ctx->frame_length = max(ctx->frame_length, ctx->min_frame_length);
	/* restore shutter */
	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	ctx->exposure[0] = shutter;
	/* group hold start */
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 1);

	if(whoopassc1main_set_long_exposure(ctx, shutter)) {
		goto exit;
	}

	/* enable auto extend */
	if (ctx->s_ctx.reg_addr_auto_extend)
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_auto_extend, 0x01);
	/* write framelength */
	if (set_auto_flicker(ctx, 0) || frame_length || !ctx->s_ctx.reg_addr_auto_extend)
		whoopassc1main_write_frame_length(ctx, ctx->frame_length);
	/* write shutter */
	//set_long_exposure(ctx);
	if (ctx->s_ctx.reg_addr_exposure[0].addr[2]) {
		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[0].addr[0],
			(ctx->exposure[0] >> 16) & 0xFF);
		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[0].addr[1],
			(ctx->exposure[0] >> 8) & 0xFF);
		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[0].addr[2],
			ctx->exposure[0] & 0xFF);
	} else {
		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[0].addr[0],
			(ctx->exposure[0] >> 8) & 0xFF);
		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[0].addr[1],
			ctx->exposure[0] & 0xFF);
	}
	LOG_INF("exp[0x%x], fll(input/output):%u/%u, flick_en:%u\n",
		ctx->exposure[0], frame_length, ctx->frame_length, ctx->autoflicker_en);

exit:
	if (!ctx->ae_ctrl_gph_en) {
		if (gph)
			ctx->s_ctx.s_gph((void *)ctx, 0);
		commit_i2c_buffer(ctx);
	}
	/* group hold end */
}

static int whoopassc1main_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
 	whoopassc1main_set_shutter_frame_length_convert(ctx, ((u64*)para)[0], ((u64*)para)[1]);
	return 0;
}

static void whoopassc1main_set_shutter_convert(struct subdrv_ctx *ctx, u64 shutter)
{
    whoopassc1main_set_shutter_frame_length_convert(ctx, shutter, 0);
}

static int whoopassc1main_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	whoopassc1main_set_shutter_frame_length_convert(ctx, ((u64*)para)[0], 0);
	return 0;
}

void whoopassc1main_set_dummy(struct subdrv_ctx *ctx)
{
	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);

	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 1);
	whoopassc1main_write_frame_length(ctx, ctx->frame_length);
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 0);

	commit_i2c_buffer(ctx);
}

static int whoopassc1main_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	enum SENSOR_SCENARIO_ID_ENUM scenario_id = (u32)((u64*)para)[0];
	u32 framerate = (u32)((u64*)para)[1];
	u32 frame_length;
	u32 frame_length_step;

	LOG_INF("scenario_id(%d), framerate(%d)", scenario_id, framerate);

	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOG(ctx, "invalid sid:%u, mode_num:%u\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW;
	}

	if (framerate == 0) {
		DRV_LOG(ctx, "framerate (%u) is invalid\n", framerate);
		return 0;
	}

	if (ctx->s_ctx.mode[scenario_id].linelength == 0) {
		DRV_LOG(ctx, "linelength (%u) is invalid\n",
			ctx->s_ctx.mode[scenario_id].linelength);
		return 0;
	}

	if (ctx->s_ctx.mode[scenario_id].hdr_mode == HDR_RAW_LBMF) {
		whoopassc1main_set_max_framerate_in_lut_by_scenario(ctx, scenario_id, framerate);
		return 0;
	}

	frame_length = ctx->s_ctx.mode[scenario_id].pclk / framerate * 10
		/ ctx->s_ctx.mode[scenario_id].linelength;
	frame_length_step = ctx->s_ctx.mode[scenario_id].framelength_step;
	frame_length = frame_length_step ?
		(frame_length - (frame_length % frame_length_step)) : frame_length;
	ctx->frame_length =
		max(frame_length, ctx->s_ctx.mode[scenario_id].framelength);
	ctx->frame_length = min(ctx->frame_length, ctx->s_ctx.frame_length_max);
	ctx->current_fps = ctx->pclk / ctx->frame_length * 10 / ctx->line_length;
	ctx->min_frame_length = ctx->frame_length;
	LOG_INF("max_fps(input/output):%u/%u(sid:%u), min_fl_en:1\n",
		framerate, ctx->current_fps, scenario_id);
	if (ctx->s_ctx.reg_addr_auto_extend ||
			(ctx->frame_length > (ctx->exposure[0] + ctx->s_ctx.exposure_margin))){
		whoopassc1main_set_dummy(ctx);
	}
	return 0;
}


static void whoopassc1main_set_max_framerate_in_lut_by_scenario(struct subdrv_ctx *ctx,
	enum SENSOR_SCENARIO_ID_ENUM scenario_id, u32 framerate)
{
	u32 frame_length = 0;
	u32 frame_length_step = 0;
	u16 exp_cnt = 0;
	u32 cit_in_lut[IMGSENSOR_STAGGER_EXPOSURE_CNT] = {0};
	u32 calc_fl_in_lut[IMGSENSOR_STAGGER_EXPOSURE_CNT] = {0};
	int i;

	frame_length = ctx->s_ctx.mode[scenario_id].pclk / framerate * 10
		/ ctx->s_ctx.mode[scenario_id].linelength;
	frame_length_step = ctx->s_ctx.mode[scenario_id].framelength_step;
	frame_length = frame_length_step ?
		(frame_length - (frame_length % frame_length_step)) : frame_length;
	ctx->frame_length =
		max(frame_length, ctx->s_ctx.mode[scenario_id].framelength);

	/* set default frame length if given default framerate */
	if (framerate ==  ctx->s_ctx.mode[scenario_id].max_framerate)
		ctx->frame_length = ctx->s_ctx.mode[scenario_id].framelength;

	// manual mode
	exp_cnt = ctx->s_ctx.mode[scenario_id].exp_cnt;
	switch (exp_cnt) {
	case 2:
		for (i = 0; i < exp_cnt; i++) {
			/*  update cit_in_lut depends on exposure_order_in_lbmf */
			if (ctx->s_ctx.mode[scenario_id].exposure_order_in_lbmf ==
				IMGSENSOR_LBMF_EXPOSURE_SE_FIRST) {
				/* 2exp: cit_lut_a = SE / cit_lut_b = LE */
				/* 3exp: cit_lut_a = SE / cit_lut_b = ME / cit_lut_c = LE */
				cit_in_lut[i] = ctx->exposure[exp_cnt - 1 - i];
			} else if (ctx->s_ctx.mode[scenario_id].exposure_order_in_lbmf ==
				IMGSENSOR_LBMF_EXPOSURE_LE_FIRST) {
				/* 2exp: cit_lut_a = LE / cit_lut_b = SE */
				/* 3exp: cit_lut_a = LE / cit_lut_b = ME / cit_lut_c = SE */
				cit_in_lut[i] = ctx->exposure[i];
			} else {
				DRV_LOGE(ctx, "pls assign exposure_order_in_lbmf value!\n");
				return;
			}
		}
		/* fll_a_min = readout + xx lines(margin) */
		calc_fl_in_lut[0] =
			ctx->s_ctx.mode[scenario_id].readout_length +
			ctx->s_ctx.mode[scenario_id].read_margin;
		/* fll_a = max(readout, previous shutter_b) */
		calc_fl_in_lut[0] =
			max(calc_fl_in_lut[0], cit_in_lut[1] + ctx->s_ctx.exposure_margin);
		/* fll_a = min(fll_a, fll_max) */
		ctx->frame_length_in_lut[0] =
			min(calc_fl_in_lut[0], ctx->s_ctx.frame_length_max);
		ctx->frame_length_in_lut[0] = frame_length_step ?
			roundup(ctx->frame_length_in_lut[0], frame_length_step) :
			ctx->frame_length_in_lut[0];
		/* fll_b_min = readout + xx lines(margin) */
		calc_fl_in_lut[1] =
			ctx->s_ctx.mode[scenario_id].readout_length +
			ctx->s_ctx.mode[scenario_id].read_margin;
		/* fll_b = max(readout, previous shutter_a) */
		calc_fl_in_lut[1] =
			max(calc_fl_in_lut[1], cit_in_lut[0] + ctx->s_ctx.exposure_margin);
		if (ctx->frame_length >= ctx->frame_length_in_lut[0]) {
			/* fll_b = max(fll_b, fll_mode_max-fll_a) */
			calc_fl_in_lut[1] =
				max(calc_fl_in_lut[1],
					ctx->frame_length - ctx->frame_length_in_lut[0]);
		}
		/* fll_b = min(fll_b, fll_max) */
		ctx->frame_length_in_lut[1] =
			min(calc_fl_in_lut[1], ctx->s_ctx.frame_length_max);
		ctx->frame_length_in_lut[1] = frame_length_step ?
			roundup(ctx->frame_length_in_lut[1], frame_length_step) :
			ctx->frame_length_in_lut[1];
		ctx->frame_length_in_lut[2] = 0;
		ctx->frame_length_in_lut[3] = 0;
		ctx->frame_length_in_lut[4] = 0;
		/* update framelength */
		ctx->frame_length =
			ctx->frame_length_in_lut[0] + ctx->frame_length_in_lut[1];
		ctx->current_fps = ctx->pclk / ctx->frame_length * 10 / ctx->line_length;
		ctx->min_frame_length = ctx->frame_length;
		DRV_LOG(ctx,
			"sid:%u,max_fps(input/output):%u/%u,min_fl_en:1,lut order:%u,fll(input/ctx/output_a/b/c/d/e):%u/%u/%u/%u/%u/%u/%un",
			scenario_id,
			framerate, ctx->current_fps,
			ctx->s_ctx.mode[scenario_id].exposure_order_in_lbmf,
			frame_length,
			ctx->frame_length,
			ctx->frame_length_in_lut[0],
			ctx->frame_length_in_lut[1],
			ctx->frame_length_in_lut[2],
			ctx->frame_length_in_lut[3],
			ctx->frame_length_in_lut[4]);
		if (ctx->s_ctx.mode[scenario_id].exposure_order_in_lbmf ==
			IMGSENSOR_LBMF_EXPOSURE_SE_FIRST) {
			if (ctx->s_ctx.reg_addr_auto_extend ||
				(ctx->frame_length_in_lut[0] >
				(ctx->exposure[0] + ctx->s_ctx.mode[scenario_id].read_margin)) ||
				(ctx->frame_length_in_lut[1] >
				(ctx->exposure[1] + ctx->s_ctx.mode[scenario_id].read_margin)))
				whoopassc1main_set_dummy(ctx);
		} else {
			if (ctx->s_ctx.reg_addr_auto_extend ||
				(ctx->frame_length_in_lut[0] >
				(ctx->exposure[1] + ctx->s_ctx.mode[scenario_id].read_margin)) ||
				(ctx->frame_length_in_lut[1] >
				(ctx->exposure[0] + ctx->s_ctx.mode[scenario_id].read_margin)))
				whoopassc1main_set_dummy(ctx);
		}
		break;
	case 3:
		for (i = 0; i < exp_cnt; i++) {
			/*  update cit_in_lut depends on exposure_order_in_lbmf */
			if (ctx->s_ctx.mode[scenario_id].exposure_order_in_lbmf ==
				IMGSENSOR_LBMF_EXPOSURE_SE_FIRST) {
				/* 2exp: cit_lut_a = SE / cit_lut_b = LE */
				/* 3exp: cit_lut_a = SE / cit_lut_b = ME / cit_lut_c = LE */
				cit_in_lut[i] = ctx->exposure[exp_cnt - 1 - i];
			} else if (ctx->s_ctx.mode[scenario_id].exposure_order_in_lbmf ==
				IMGSENSOR_LBMF_EXPOSURE_LE_FIRST) {
				/* 2exp: cit_lut_a = LE / cit_lut_b = SE */
				/* 3exp: cit_lut_a = LE / cit_lut_b = ME / cit_lut_c = SE */
				cit_in_lut[i] = ctx->exposure[i];
			} else {
				DRV_LOGE(ctx, "pls assign exposure_order_in_lbmf value!\n");
				return;
			}
		}
		/* fll_a_min = readout + xx lines(margin) */
		calc_fl_in_lut[0] =
			ctx->s_ctx.mode[scenario_id].readout_length +
			ctx->s_ctx.mode[scenario_id].read_margin;
		/* fll_a = max(readout, previous shutter_b) */
		calc_fl_in_lut[0] =
			max(calc_fl_in_lut[0], cit_in_lut[1] + ctx->s_ctx.exposure_margin);
		/* fll_a = min(fll_a, fll_max) */
		ctx->frame_length_in_lut[0] =
			min(calc_fl_in_lut[0], ctx->s_ctx.frame_length_max);
		ctx->frame_length_in_lut[0] = frame_length_step ?
			roundup(ctx->frame_length_in_lut[0], frame_length_step) :
			ctx->frame_length_in_lut[0];
		/* fll_b_min = readout + xx lines(margin) */
		calc_fl_in_lut[1] =
			ctx->s_ctx.mode[scenario_id].readout_length +
			ctx->s_ctx.mode[scenario_id].read_margin;
		/* fll_b = max(readout, previous shutter_c) */
		calc_fl_in_lut[1] =
			max(calc_fl_in_lut[1], cit_in_lut[2] + ctx->s_ctx.exposure_margin);
		/* fll_b = min(fll_b, fll_max) */
		ctx->frame_length_in_lut[1] =
			min(calc_fl_in_lut[1], ctx->s_ctx.frame_length_max);
		ctx->frame_length_in_lut[1] = frame_length_step ?
			roundup(ctx->frame_length_in_lut[1], frame_length_step) :
			ctx->frame_length_in_lut[1];
		/* fll_c_min = readout + xx lines(margin) */
		calc_fl_in_lut[2] =
			ctx->s_ctx.mode[scenario_id].readout_length +
			ctx->s_ctx.mode[scenario_id].read_margin;
		/* fll_c = max(readout, previous shutter_a) */
		calc_fl_in_lut[2] =
			max(calc_fl_in_lut[2], cit_in_lut[0] + ctx->s_ctx.exposure_margin);
		if (ctx->frame_length >=
			(ctx->frame_length_in_lut[0] + ctx->frame_length_in_lut[1])) {
			/* fll_c = max(fll_c, fll_mode_max-fll_b-fll_a) */
			calc_fl_in_lut[2] =
				max(calc_fl_in_lut[2],
					(ctx->frame_length - ctx->frame_length_in_lut[1] -
					ctx->frame_length_in_lut[0]));
		}
		/* fll_c = min(fll_c, fll_max) */
		ctx->frame_length_in_lut[2] =
			min(calc_fl_in_lut[2], ctx->s_ctx.frame_length_max);
		ctx->frame_length_in_lut[2] = frame_length_step ?
			roundup(ctx->frame_length_in_lut[2], frame_length_step) :
			ctx->frame_length_in_lut[2];
		ctx->frame_length_in_lut[3] = 0;
		ctx->frame_length_in_lut[4] = 0;
		/* update framelength */
		ctx->frame_length =
			ctx->frame_length_in_lut[0] +
			ctx->frame_length_in_lut[1] +
			ctx->frame_length_in_lut[2];
		ctx->current_fps = ctx->pclk / ctx->frame_length * 10 / ctx->line_length;
		ctx->min_frame_length = ctx->frame_length;
		DRV_LOG(ctx,
			"sid:%u,max_fps(input/output):%u/%u,min_fl_en:1,lut order:%u,fll(input/ctx/output_a/b/c/d/e):%u/%u/%u/%u/%u/%u/%u\n",
			scenario_id,
			framerate, ctx->current_fps,
			ctx->s_ctx.mode[scenario_id].exposure_order_in_lbmf,
			frame_length,
			ctx->frame_length,
			ctx->frame_length_in_lut[0],
			ctx->frame_length_in_lut[1],
			ctx->frame_length_in_lut[2],
			ctx->frame_length_in_lut[3],
			ctx->frame_length_in_lut[4]);
		if (ctx->s_ctx.mode[scenario_id].exposure_order_in_lbmf ==
			IMGSENSOR_LBMF_EXPOSURE_SE_FIRST) {
			if (ctx->s_ctx.reg_addr_auto_extend ||
				(ctx->frame_length_in_lut[0] >
				(ctx->exposure[1] + ctx->s_ctx.mode[scenario_id].read_margin)) ||
				(ctx->frame_length_in_lut[1] >
				(ctx->exposure[0] + ctx->s_ctx.mode[scenario_id].read_margin)) ||
				(ctx->frame_length_in_lut[2] >
				(ctx->exposure[2] + ctx->s_ctx.mode[scenario_id].read_margin)))
				whoopassc1main_set_dummy(ctx);
		} else {
			if (ctx->s_ctx.reg_addr_auto_extend ||
				(ctx->frame_length_in_lut[0] >
				(ctx->exposure[1] + ctx->s_ctx.mode[scenario_id].read_margin)) ||
				(ctx->frame_length_in_lut[1] >
				(ctx->exposure[2] + ctx->s_ctx.mode[scenario_id].read_margin)) ||
				(ctx->frame_length_in_lut[2] >
				(ctx->exposure[0] + ctx->s_ctx.mode[scenario_id].read_margin)))
				whoopassc1main_set_dummy(ctx);
		}
		break;
	default:
		break;
	}
}

static void whoopassc1main_set_gain_convert(struct subdrv_ctx *ctx, u32 gain)
{
	u16 rg_gain;
	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);
	u32 ana_gain_min = ctx->s_ctx.mode[ctx->current_scenario_id].ana_gain_max ?
		ctx->s_ctx.mode[ctx->current_scenario_id].ana_gain_min : ctx->ana_gain_min;
	u32 ana_gain_max = ctx->s_ctx.mode[ctx->current_scenario_id].ana_gain_max ?
		ctx->s_ctx.mode[ctx->current_scenario_id].ana_gain_max : ctx->ana_gain_max;

	LOG_INF("gain(%d) ana_gain_min(%d), ana_gain_max(%d)\n", gain, ana_gain_min, ana_gain_max);
	/* check boundary of gain */
	gain = max(gain, ana_gain_min);
	gain = min(gain, ana_gain_max);

	/* mapping of gain to register value */
	if (ctx->s_ctx.g_gain2reg != NULL)
		rg_gain = ctx->s_ctx.g_gain2reg(gain);
	else
		rg_gain = gain2reg(gain);

	/* restore gain */
	memset(ctx->ana_gain, 0, sizeof(ctx->ana_gain));
	ctx->ana_gain[0] = gain;
	/* group hold start */
	if (gph && !ctx->ae_ctrl_gph_en)
		ctx->s_ctx.s_gph((void *)ctx, 1); 
	/* write gain */
	set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_ana_gain[0].addr[0],
		(rg_gain >> 8) & 0xFF);
	set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_ana_gain[0].addr[1],
		rg_gain & 0xFF);
	DRV_LOG(ctx, "gain[0x%x]\n", rg_gain);
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 0);
	commit_i2c_buffer(ctx);
	/* group hold end */
}

static int whoopassc1main_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;
	u32 gain = *feature_data;
	whoopassc1main_set_gain_convert(ctx, gain);
	return 0;
}

static void whoopassc1main_set_multi_gain(struct subdrv_ctx *ctx, u32 *gains, u16 exp_cnt)
{
	int i = 0;
	u16 rg_gains[3] = {0};
	u8 has_gains[3] = {0};
	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);
	u32 ana_gain_min = ctx->s_ctx.mode[ctx->current_scenario_id].ana_gain_max ?
		ctx->s_ctx.mode[ctx->current_scenario_id].ana_gain_min : ctx->ana_gain_min;
	u32 ana_gain_max = ctx->s_ctx.mode[ctx->current_scenario_id].ana_gain_max ?
		ctx->s_ctx.mode[ctx->current_scenario_id].ana_gain_max : ctx->ana_gain_max;

	if(exp_cnt == 2) {
		LOG_INF("gains[0]:%u, gains[1]:%u, exp_cnt:%d ana_gain_min:%d ana_gain_max:%d\n",
			gains[0], gains[1], exp_cnt, ana_gain_min, ana_gain_max);
	} else {
		LOG_INF("gains[0]:%u, exp_cnt:%d ana_gain_min:%d ana_gain_max:%d\n",
			gains[0], exp_cnt, ana_gain_min, ana_gain_max);
	}
	if (exp_cnt > ARRAY_SIZE(ctx->ana_gain)) {
		DRV_LOGE(ctx, "invalid exp_cnt:%u>%lu\n", exp_cnt, ARRAY_SIZE(ctx->ana_gain));
		exp_cnt = ARRAY_SIZE(ctx->ana_gain);
	}
	for (i = 0; i < exp_cnt; i++) {
		/* check boundary of gain */
		gains[i] = max(gains[i], ana_gain_min);
		gains[i] = min(gains[i], ana_gain_max);
		/* mapping of gain to register value */
		if (ctx->s_ctx.g_gain2reg != NULL)
			gains[i] = ctx->s_ctx.g_gain2reg(gains[i]);
		else
			gains[i] = gain2reg(gains[i]);
	}
	/* restore gain */
	memset(ctx->ana_gain, 0, sizeof(ctx->ana_gain));
	for (i = 0; i < exp_cnt; i++)
		ctx->ana_gain[i] = gains[i];
	/* group hold start */
	if (gph && !ctx->ae_ctrl_gph_en)
		ctx->s_ctx.s_gph((void *)ctx, 1);
	/* write gain */
	memset(has_gains, 1, sizeof(has_gains));
	switch (exp_cnt) {
	case 2:
		has_gains[1] = 0;
//		rg_gains[0] = gains[0];
//		rg_gains[2] = gains[1];
		rg_gains[0] = gains[1];
		rg_gains[2] = gains[0];
		break;
	case 3:
//		rg_gains[0] = gains[0];
		rg_gains[0] = gains[2];
		rg_gains[1] = gains[1];
//		rg_gains[2] = gains[2];
		rg_gains[2] = gains[0];
		break;
	default:
		has_gains[0] = 0;
		has_gains[1] = 0;
		has_gains[2] = 0;
		break;
	}
	for (i = 0; i < 3; i++) {
		if (has_gains[i]) {
			set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_ana_gain[i].addr[0],
				(rg_gains[i] >> 8) & 0xFF);
			set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_ana_gain[i].addr[1],
				rg_gains[i] & 0xFF);
		}
	}
	LOG_INF("reg[sg/mg/lg]: 0x%x 0x%x 0x%x\n", rg_gains[0], rg_gains[1], rg_gains[2]);
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 0);
	commit_i2c_buffer(ctx);
	/* group hold end */
}


static void whoopassc1main_set_hdr_tri_gain(struct subdrv_ctx *ctx, u64 *gains, u16 exp_cnt)
{
	int i = 0;
	u32 values[3] = {0};

	if (gains != NULL) {
		for (i = 0; i < 3; i++)
			values[i] = (u32) *(gains + i);
	}
	if (ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode == HDR_RAW_LBMF) {
		set_multi_gain_in_lut(ctx, values, exp_cnt);
		return;
	}
	whoopassc1main_set_multi_gain(ctx,	values, exp_cnt);
}

static int whoopassc1main_set_hdr_tri_gain2(struct subdrv_ctx *ctx, u8 *para, u32 *len) {

	u64 *feature_data = (u64 *) para;
	whoopassc1main_set_hdr_tri_gain(ctx, feature_data, 2);
	return 0;
}

static int whoopassc1main_set_hdr_tri_gain3(struct subdrv_ctx *ctx, u8 *para, u32 *len) {
	u64 *feature_data = (u64 *) para;
	whoopassc1main_set_hdr_tri_gain(ctx, feature_data, 3);
	return 0;
}
static void update_CTLE(struct subdrv_ctx *ctx)
{
	for (int scenario_id = 0; scenario_id < ctx->s_ctx.sensor_mode_num; ++scenario_id){
		if (ctx->s_ctx.sensor_id == WHOOPASSC1MAIN_SENSOR_ID || ctx->s_ctx.sensor_id == WHOOPASSS1MAIN_SENSOR_ID
			|| ctx->s_ctx.sensor_id == WHOOPASSC1MAIN2_SENSOR_ID || ctx->s_ctx.sensor_id == WHOOPASSS1MAIN2_SENSOR_ID) {
			ctx->s_ctx.mode[scenario_id].csi_param.cphy_ctle = MAINC1_CTLE_LEVEL;
			ctx->s_ctx.mode[scenario_id].csi_param.cdr_delay = MAINC1_CTLE_DELAY;
			ctx->s_ctx.mode[scenario_id].csi_param.cphy_eq_bw = MAINC1_CTLE_EQBW;
			LOG_INF("update_CTLE_C1, scenario_id: %d\n", scenario_id);
		} else if (ctx->s_ctx.sensor_id == WHOOPASSC2MAIN_SENSOR_ID || ctx->s_ctx.sensor_id == WHOOPASSS2MAIN_SENSOR_ID
			      || ctx->s_ctx.sensor_id == WHOOPASSC2MAIN2_SENSOR_ID || ctx->s_ctx.sensor_id == WHOOPASSS2MAIN2_SENSOR_ID) {
			ctx->s_ctx.mode[scenario_id].csi_param.cphy_ctle = MAINC2_CTLE_LEVEL;
			ctx->s_ctx.mode[scenario_id].csi_param.cdr_delay = MAINC2_CTLE_DELAY;
			ctx->s_ctx.mode[scenario_id].csi_param.cphy_eq_bw = MAINC2_CTLE_EQBW;
			LOG_INF("update_CTLE_C2, scenario_id: %d\n", scenario_id);
		} else {
			LOG_INF("update_CTLE false\n");
		}
	}


}
