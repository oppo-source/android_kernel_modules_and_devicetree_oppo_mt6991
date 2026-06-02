// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 whoopassc1widemipiraw_Sensor.c
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
#include "whoopassc1widemipiraw_Sensor.h"
//GC50F6
#define SENSOR_NAME  SENSOR_DRVNAME_WHOOPASSC1WIDE_MIPI_RAW

#define WHOOPASSC1WIDE_EEPROM_ADDR        (0xA0)
#define WHOOPASSC1WIDE_EERPOM_MAX_OFFSET  (0x4000)
//#define OPLUS_CAMERA_COMMON_DATA_LENGTH (40)

#define PFX "whoopassc1wide_camera_sensor"
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)


#ifdef  EEPROM_WRITE_DATA_MAX_LENGTH
#undef  EEPROM_WRITE_DATA_MAX_LENGTH
#endif
#define EEPROM_WRITE_DATA_MAX_LENGTH       (64)
#define WHOOPASSC1WIDE_STEREO_MW_START_ADDR  (0x2980)
#define WHOOPASSC1WIDE_AESYNC_START_ADDR     (0x2F90)

#define  OTP_PDC_IS_VALID_VAL     (0x01)
#define  OTP_PDC_VALID_ADDR       (0x1AEA)
#define  XTC_SENSOR_ADDR          (0x0600)
#define  XTC_SENSOR_LENGTH  	  (490)
#define  XTC_SENSOR_LENGTH_PART1  (256)
#define  XTC_SENSOR_LENGTH_PART2  (490 - 256)

static u8 pdc_is_valid = 0;

#define WHOOPASSC1WIDE_IMGSENSOR_ID   (0x50F6)

#define WHOOPASSC1WIDE_UNIQUE_SENSOR_ID_ADDR    (0x7000)  //?????
#define WHOOPASSC1WIDE_UNIQUE_SENSOR_ID_LENGTH  (16)
// static BYTE whoopassc1wide_unique_id[WHOOPASSC1WIDE_UNIQUE_SENSOR_ID_LENGTH] = { 0 };
static struct oplus_eeprom_info_struct  oplus_eeprom_info = {0};

static kal_uint8 otp_data_checksum[WHOOPASSC1WIDE_EERPOM_MAX_OFFSET] = {0};
static int get_sensor_temperature(void *arg);
#define MAX_BURST_LEN  (2048)
static u8 * msg_buf = NULL;
static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
//static int whoopassc1wide_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int whoopassc1wide_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int whoopassc1wide_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int whoopassc1wide_get_eeprom_comdata(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int whoopassc1wide_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int whoopassc1wide_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int whoopassc1wide_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int whoopassc1wide_get_min_shutter_by_scenario_adapter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int open(struct subdrv_ctx *ctx);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt);
//static void whoopassc1wide_set_shutter_convert(struct subdrv_ctx *ctx, u64 shutter);
static int whoopassc1wide_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int whoopassc1wide_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void whoopassc1wide_set_shutter_frame_length_convert(struct subdrv_ctx *ctx, u64 shutter, u32 frame_length);
static void whoopassc1wide_set_multi_shutter_frame_length(struct subdrv_ctx *ctx, u64 *shutters, u16 exp_cnt, u16 frame_length);
static int whoopassc1wide_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int whoopassc1wide_set_hdr_tri_shutter2(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int whoopassc1wide_set_hdr_tri_shutter3(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size);
static int whoopassc1wide_i2c_burst_wr_regs_u8(struct subdrv_ctx *ctx, u16 * list, u32 len);
static int adapter_i2c_burst_wr_regs_u8(struct subdrv_ctx * ctx,
		u16 addr, u16 *list, u32 len);
static bool g_id_from_dts_flag = false;
static void get_imgsensor_id_from_dts(struct subdrv_ctx *ctx, u32 *sensor_id);
//static int whoopassc1wide_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len);
//static int whoopassc1wide_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len);
//static void whoopassc1wide_write_frame_length(struct subdrv_ctx *ctx, u32 fll);
// static int whoopassc1wide_get_unique_sensorid(struct subdrv_ctx *ctx, u8 *para, u32 *len);
//static int whoopassc1wide_get_cloud_otp_info(struct subdrv_ctx *ctx, u8 *para, u32 *len);
// static int whoopassc1wide_get_pdafblock_info(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void whoopassc1wide_get_sensor_cali(void* arg);
static void whoopassc1wide_set_sensor_cali(void *arg);
static int whoopassc1wide_get_readout_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static bool whoopassc1wide_set_long_exposure(struct subdrv_ctx *ctx);
/* STRUCT */
static void update_CTLE(struct subdrv_ctx *ctx);
#define WIDEC1_CTLE_LEVEL 1
#define WIDEC1_CTLE_EQBW 3

static struct eeprom_map_info whoopassc1wide_eeprom_info[] = {
	{ EEPROM_META_MODULE_ID, 0x0000, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_SENSOR_ID, 0x0006, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_LENS_ID, 0x0008,0x0010, 0x0011, 2, true },
	{ EEPROM_META_VCM_ID, 0x000A, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_MIRROR_FLIP, 0x000E, 0x0010, 0x0011, 1, true },
	{ EEPROM_META_MODULE_SN, 0x00B0, 0x00C7, 0x00C8,23, true },
	{ EEPROM_META_AF_CODE, 0x0092, 0x0098, 0x0099, 6, true },
	{ EEPROM_META_AF_FLAG, 0x0098, 0x0098, 0x0099, 1, true },
	{ EEPROM_META_STEREO_DATA, 0x0000, 0x0000, 0x0000, 0x0000, false },
	{ EEPROM_META_STEREO_MW_MAIN_DATA, WHOOPASSC1WIDE_STEREO_MW_START_ADDR, 0xFFFF, 0xFFFF, CALI_DATA_SLAVE_LENGTH, false },
	{ EEPROM_META_STEREO_MT_MAIN_DATA, 0, 0, 0, 0, false },
	{ EEPROM_META_STEREO_MT_MAIN_DATA_105CM, 0, 0, 0, 0, false },
	{ EEPROM_META_DISTORTION_DATA, 0, 0, 0, 0, false },
};

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, whoopassc1wide_set_test_pattern},
//	{SENSOR_FEATURE_SEAMLESS_SWITCH, whoopassc1wide_seamless_switch},
	{SENSOR_FEATURE_CHECK_SENSOR_ID, whoopassc1wide_check_sensor_id},
	{SENSOR_FEATURE_GET_EEPROM_COMDATA, whoopassc1wide_get_eeprom_comdata},
	{SENSOR_FEATURE_SET_SENSOR_OTP, whoopassc1wide_set_eeprom_calibration},
	{SENSOR_FEATURE_GET_EEPROM_STEREODATA, whoopassc1wide_get_eeprom_calibration},
	{SENSOR_FEATURE_GET_SENSOR_OTP_ALL, whoopassc1wide_get_otp_checksum_data},
	{SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO, whoopassc1wide_get_min_shutter_by_scenario_adapter},
//	{SENSOR_FEATURE_SET_STREAMING_SUSPEND, whoopassc1wide_streaming_suspend},
	{SENSOR_FEATURE_SET_ESHUTTER, whoopassc1wide_set_shutter},
	{SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME, whoopassc1wide_set_shutter_frame_length},
	{SENSOR_FEATURE_SET_HDR_SHUTTER, whoopassc1wide_set_hdr_tri_shutter2},
	{SENSOR_FEATURE_SET_HDR_TRI_SHUTTER, whoopassc1wide_set_hdr_tri_shutter3},
	{SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME, whoopassc1wide_set_multi_shutter_frame_length_ctrl},
//	{SENSOR_FEATURE_SET_STREAMING_RESUME, whoopassc1wide_streaming_resume},
	// {SENSOR_FEATURE_GET_UNIQUE_SENSORID, whoopassc1wide_get_unique_sensorid},
	// {SENSOR_FEATURE_GET_CLOUD_OTP_INFO, whoopassc1wide_get_cloud_otp_info},
	// {SENSOR_FEATURE_GET_PDAF_INFO, whoopassc1wide_get_pdafblock_info},
	{SENSOR_FEATURE_GET_READOUT_BY_SCENARIO, whoopassc1wide_get_readout_by_scenario},
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x01ab0136,
		.addr_header_id = 0x00000006,
		.i2c_write_id = WHOOPASSC1WIDE_EEPROM_ADDR,
		.pdc_support = TRUE,
		.pdc_size = XTC_SENSOR_LENGTH,
		.addr_pdc = 0x1900,
		.sensor_reg_addr_pdc = XTC_SENSOR_ADDR,
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_4096_3072 = {  //partial PD
	.i4OffsetX = 0,
	.i4OffsetY = 8,
	.i4PitchX = 8,
	.i4PitchY = 8,
	.i4PairNum = 2,
	.i4SubBlkW = 8,
	.i4SubBlkH = 4,
	.i4PosL = {{2, 9}, {6, 13}},
	.i4PosR = {{3, 9}, {7, 13}},
	.i4BlockNumX = 512,
	.i4BlockNumY = 382,
	.i4LeFirst = 0,
	.i4Crop = {
		/* <pre> <cap> <normal_video> <hs_video> <slim_video> */
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		/* <cust1> <cust2>*/
		{0, 0}, {0, 0},
	},
	.iMirrorFlip = IMAGE_HV_MIRROR,
	.i4FullRawW = 4096,
	.i4FullRawH = 3072,
	.i4VCPackNum = 1,
	.PDAF_Support = PDAF_SUPPORT_CAMSV,
	.i4ModeIndex = 0x0,
	.sPDMapInfo[0] = {
		.i4PDPattern = 3,//LR
		//.i4BinFacX = 2,
		//.i4BinFacY = 4,
		.i4PDRepetition = 2,
		.i4PDOrder = {0,1}, //R=1, L=0
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_4096_2304 = {  //partial PD
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX = 8,
	.i4PitchY = 8,
	.i4PairNum = 2,
	.i4SubBlkW = 8,
	.i4SubBlkH = 4,
	.i4PosL = {{2, 1}, {6, 5}},
	.i4PosR = {{3, 1}, {7, 5}},
	.i4BlockNumX = 512,
	.i4BlockNumY = 288,
	.i4LeFirst = 0,
	.i4Crop = {
		/* <pre> <cap> <normal_video> <hs_video> <slim_video> */
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		/* <cust1> <cust2>*/
		{0, 0}, {0, 0},
	},
	.iMirrorFlip = IMAGE_HV_MIRROR,
	.i4FullRawW = 4096,
	.i4FullRawH = 2304,
	.i4VCPackNum = 1,
	.PDAF_Support = PDAF_SUPPORT_CAMSV,
	.i4ModeIndex = 0x0,
	.sPDMapInfo[0] = {
		.i4PDPattern = 3,//LR
		//.i4BinFacX = 2,
		//.i4BinFacY = 4,
		.i4PDRepetition = 2,
		.i4PDOrder = {0,1}, //R=1, L=0
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_4096_2048 = {  //partial PD
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX = 8,
	.i4PitchY = 8,
	.i4PairNum = 2,
	.i4SubBlkW = 8,
	.i4SubBlkH = 4,
	.i4PosL = {{2, 1}, {6, 5}},
	.i4PosR = {{3, 1}, {7, 5}},
	.i4BlockNumX = 512,
	.i4BlockNumY = 256,
	.i4LeFirst = 0,
	.i4Crop = {
		/* <pre> <cap> <normal_video> <hs_video> <slim_video> */
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		/* <cust1> <cust2>*/
		{0, 0}, {0, 0},
	},
	.iMirrorFlip = IMAGE_HV_MIRROR,
	.i4FullRawW = 4096,
	.i4FullRawH = 2048,
	.i4VCPackNum = 1,
	.PDAF_Support = PDAF_SUPPORT_CAMSV,
	.i4ModeIndex = 0x0,
	.sPDMapInfo[0] = {
		.i4PDPattern = 3,//LR
		//.i4BinFacX = 2,
		//.i4BinFacY = 4,
		.i4PDRepetition = 2,
		.i4PDOrder = {0,1}, //R=1, L=0
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_3200_2400 = {  //partial PD
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX = 8,
	.i4PitchY = 8,
	.i4PairNum = 2,
	.i4SubBlkW = 8,
	.i4SubBlkH = 4,
	.i4PosL = {{2, 1}, {6, 5}},
	.i4PosR = {{3, 1}, {7, 5}},
	.i4BlockNumX = 400,
	.i4BlockNumY = 300,
	.i4LeFirst = 0,
	.i4Crop = {
		/* <pre> <cap> <normal_video> <hs_video> <slim_video> */
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		/* <cust1> <cust2>*/
		{0, 0}, {0, 0},
	},
	.iMirrorFlip = IMAGE_HV_MIRROR,
	.i4FullRawW = 3200,
	.i4FullRawH = 2400,
	.i4VCPackNum = 1,
	.PDAF_Support = PDAF_SUPPORT_CAMSV,
	.i4ModeIndex = 0x0,
	.sPDMapInfo[0] = {
		.i4PDPattern = 3,//LR
		//.i4BinFacX = 2,
		//.i4BinFacY = 4,
		.i4PDRepetition = 2,
		.i4PDOrder = {0,1}, //R=1, L=0
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_1728_1296 = {  //partial PD
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX = 8,
	.i4PitchY = 8,
	.i4PairNum = 2,
	.i4SubBlkW = 8,
	.i4SubBlkH = 4,
	.i4PosL = {{2, 1}, {6, 5}},
	.i4PosR = {{3, 1}, {7, 5}},
	.i4BlockNumX = 216,
	.i4BlockNumY = 162,
	.i4LeFirst = 0,
	.i4Crop = {
		/* <pre> <cap> <normal_video> <hs_video> <slim_video> */
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		/* <cust1> <cust2>*/
		{0, 0}, {0, 0},
	},
	.iMirrorFlip = IMAGE_HV_MIRROR,
	.i4FullRawW = 1728,
	.i4FullRawH = 1296,
	.i4VCPackNum = 1,
	.PDAF_Support = PDAF_SUPPORT_CAMSV,
	.i4ModeIndex = 0x0,
	.sPDMapInfo[0] = {
		.i4PDPattern = 3,//LR
		//.i4BinFacX = 2,
		//.i4BinFacY = 4,
		.i4PDRepetition = 2,
		.i4PDOrder = {0,1}, //R=1, L=0
	},
};


static struct mtk_mbus_frame_desc_entry frame_desc_prev_cap[] = {
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
			.channel = 3,
			.data_type = 0x2b,
			.hsize = 512,
			.vsize = 1528,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
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
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x2b,
			.hsize = 512,
			.vsize = 1152,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs[] = {
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
			.channel = 3,
			.data_type = 0x2b,
			.hsize = 512,
			.vsize = 1152,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_slim[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 2048,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x2b,
			.hsize = 512,
			.vsize = 1024,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3200,
			.vsize = 2400,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x2b,
			.hsize = 400,
			.vsize = 1200,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1728,
			.vsize = 1296,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x2b,
			.hsize = 216,
			.vsize = 648,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct subdrv_mode_struct mode_struct[] = {
    {//M00-4096x3072-PD-992x760-HighGain-30fps.txt
		.frame_desc = frame_desc_prev_cap,
		.num_entries = ARRAY_SIZE(frame_desc_prev_cap),
		.mode_setting_table = whoopassc1wide_preview_capture_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1wide_preview_capture_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 495000000,
		.linelength = 4960,
		.framelength = 3324,
		.max_framerate = 300,
		.mipi_pixel_rate = 537600000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 4,
		.coarse_integ_step = 1,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
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
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_4096_3072,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 32,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
			.sensorScenario = SENSOR_SCENARIO_BIN,
		},
	},
	{//M00-4096x3072-PD-992x760-HighGain-30fps.txt
		.frame_desc = frame_desc_prev_cap,
		.num_entries = ARRAY_SIZE(frame_desc_prev_cap),
		.mode_setting_table = whoopassc1wide_preview_capture_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1wide_preview_capture_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 495000000,
		.linelength = 4960,
		.framelength = 3324,
		.max_framerate = 300,
		.mipi_pixel_rate = 537600000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 4,
		.coarse_integ_step = 1,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
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
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_4096_3072,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 32,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 30,
			.sensorScenario = SENSOR_SCENARIO_BIN,
		},
	},
	{//M01-4096x2304-PD-992x576-HighGain-30fps.txt
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = whoopassc1wide_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1wide_normal_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 585000000,
		.linelength = 4960,
		.framelength = 3928,
		.max_framerate = 300,
		.mipi_pixel_rate = 742400000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 4,
		.coarse_integ_step = 1,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
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
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_4096_2304,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 32,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
			.sensorScenario = SENSOR_SCENARIO_BIN,
		},
	},
    {//M02-4096x2304-PD-992x576-NoralGain-60fps.txt
		.frame_desc = frame_desc_hs,
		.num_entries = ARRAY_SIZE(frame_desc_hs),
		.mode_setting_table = whoopassc1wide_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1wide_hs_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 768000000,
		.linelength = 4896,
		.framelength = 2612,
		.max_framerate = 600,
		.mipi_pixel_rate = 985600000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 4,
		.coarse_integ_step = 1,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
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
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_4096_2304,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 32,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 15.5,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 60,
			.sensorScenario = SENSOR_SCENARIO_BIN,
		},
	},
    {//M03-4096x2048-PD-992x512-HighGain-30fps.txt
		.frame_desc = frame_desc_slim,
		.num_entries = ARRAY_SIZE(frame_desc_slim),
		.mode_setting_table = whoopassc1wide_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1wide_slim_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 495000000,
		.linelength = 4960,
		.framelength = 3324,
		.max_framerate = 300,
		.mipi_pixel_rate = 537600000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 4,
		.coarse_integ_step = 1,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 1024,
			.w0_size = 8192,
			.h0_size = 4096,
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
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_4096_2048,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 32,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
			.sensorScenario = SENSOR_SCENARIO_BIN,
		},
	},
    {//M05-3200x2400-PD-800x600-HighGain-24fps.txt
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = whoopassc1wide_custom1_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1wide_custom1_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 495000000,
		.linelength = 4960,
		.framelength = 4152,
		.max_framerate = 240,
		.mipi_pixel_rate = 537600000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 4,
		.coarse_integ_step = 1,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 896,
			.y0_offset = 672,
			.w0_size = 6400,
			.h0_size = 4800,
			.scale_w = 3200,
			.scale_h = 2400,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3200,
			.h1_size = 2400,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3200,
			.h2_tg_size = 2400,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_3200_2400,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 32,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 24,
			.sensorScenario = SENSOR_SCENARIO_BIN,
		},
	},
    {//M06-1792x1344-PD-448x336-HighGain-24fps.txt
		.frame_desc = frame_desc_cus2,
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = whoopassc1wide_custom2_setting,
		.mode_setting_len = ARRAY_SIZE(whoopassc1wide_custom2_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 495000000,
		.linelength = 4960,
		.framelength = 4152,
		.max_framerate = 240,
		.mipi_pixel_rate = 537600000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 4,
		.coarse_integ_step = 1,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 2368,
			.y0_offset = 1776,
			.w0_size = 3456,
			.h0_size = 2592,
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
		.imgsensor_pd_info = &imgsensor_pd_info_1728_1296,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 32,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 24,
			.sensorScenario = SENSOR_SCENARIO_BIN,
		},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = WHOOPASSC1WIDE_SENSOR_ID,
	.reg_addr_sensor_id = {0x03F0, 0x03F1},
	.i2c_addr_table = {0x94, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {8192, 6144},
	.mirror = IMAGE_HV_MIRROR,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_Gb,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 32,
	.ana_gain_type = 4,
	.ana_gain_step = 1,
//	.ana_gain_table = whoopassc1wide_ana_gain_table,
//	.ana_gain_table_size = sizeof(whoopassc1wide_ana_gain_table),
	.tuning_iso_base = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 1,
	.exposure_max = (0xFFFF - 16) << 7,
	.exposure_step = 1,
	.exposure_margin = 16,

	.frame_length_max = 0xFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 2632000,

	.pdaf_type = PDAF_SUPPORT_CAMSV,
	.hdr_type = HDR_SUPPORT_NA,
	.seamless_switch_support = FALSE,
	.temperature_support = TRUE,

	.g_temp = get_sensor_temperature,
	.g_gain2reg = get_gain2reg,
	.g_cali = whoopassc1wide_get_sensor_cali,
	.s_gph = set_group_hold,
	.s_cali = whoopassc1wide_set_sensor_cali,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = PARAM_UNDEFINED, //0x3821  0x3820
//	.reg_addr_mirror_flip = 0x0101, //0x3821  0x3820
	.reg_addr_exposure = {
			{0x0202, 0x0203},//Long exposure
	},
	.long_exposure_support = PARAM_UNDEFINED,
	.reg_addr_exposure_lshift = PARAM_UNDEFINED,
	.reg_addr_ana_gain = {
			{0x0204, 0x0205},//Long gain
	},
	.reg_addr_frame_length = {0x0340, 0x0341},
	.reg_addr_temp_en = 0x02D0,
	.reg_addr_temp_read = 0x02D5,
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_frame_count = PARAM_UNDEFINED,
	.reg_addr_fast_mode = PARAM_UNDEFINED,

	.init_setting_table = whoopassc1wide_init_setting,
	.init_setting_len = ARRAY_SIZE(whoopassc1wide_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),

	.chk_s_off_sta = 0,
	.chk_s_off_end = 0,
	.checksum_value = 0xcd9966da,
};

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
	.vsync_notify = vsync_notify,
	.update_sof_cnt = common_update_sof_cnt,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_RST, {0}, 1000},
	{HW_ID_MCLK, {24}, 0},
	{HW_ID_MCLK_DRIVING_CURRENT, {4}, 1000},
	{HW_ID_AFVDD1, {2800000, 2800000}, 0},
	{HW_ID_AVDD1, {2800000, 2800000}, 1000},
	{HW_ID_AFVDD, {2800000, 2800000}, 0},
	{HW_ID_DOVDD, {1800000, 1800000}, 0},
	{HW_ID_DVDD, {1100000, 1100000}, 0},
	{HW_ID_AVDD, {2800000, 2800000}, 3000},
	{HW_ID_RST, {1}, 1000},
};

struct subdrv_entry whoopassc1wide_mipi_raw_entry = {
	.name = "whoopassc1wide_mipi_raw",
	.id = WHOOPASSC1WIDE_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};


/* FUNCTION */

static unsigned int read_whoopassc1wide_eeprom_info(struct subdrv_ctx *ctx, kal_uint16 meta_id,
	BYTE *data, int size)
{
	kal_uint16 addr;
	int readsize;

	if (meta_id != whoopassc1wide_eeprom_info[meta_id].meta)
		return -1;

	if (size != whoopassc1wide_eeprom_info[meta_id].size)
		return -1;

	addr = whoopassc1wide_eeprom_info[meta_id].start;
	readsize = whoopassc1wide_eeprom_info[meta_id].size;

	if(!read_cmos_eeprom_p8(ctx, addr, data, readsize)) {
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

// static struct SENSOR_OTP_INFO_STRUCT cloud_otp_info[OPLUS_CAM_CAL_DATA_MAX] = {
// 	{
// 		.OtpInfoLen = 1,
// 		.OtpInfo = {{0x0000, 17}}, /*{addr_modinfo, addr_modinfolen}*/
// 	}, /*OPLUS_CAM_CAL_DATA_MODULE_VERSION*/
// 	{
// 		.OtpInfoLen = 1,
// 		.OtpInfo = {{0x0000, 17}}, /*{addr_modinfo, addr_modinfolen}*/
// 	}, /*OPLUS_CAM_CAL_DATA_PART_NUMBER*/
// 	{
// 		.OtpInfoLen = 1,
// 		.OtpInfo = {{0x1d60, 1868}},
// 	}, /*OPLUS_CAM_CAL_DATA_SHADING_TABLE--LSC*/
// 	{
// 		.OtpInfoLen = 5,
// 		.OtpInfo = {{0x0020, 16}, {0x0044, 16}, {0x0060, 4}, {0x006c, 4}, {0x0092, 6}},
// 		.isAFCodeOffset = KAL_FALSE,
// 	}, /*OPLUS_CAM_CAL_DATA_3A_GAIN-awb5000\awb2850\awb5000Light\awb2850light\af*/
// 	{
// 		.OtpInfoLen = 2,
// 		.OtpInfo = {{0x1300, 496}, {0x1500, 1004}},
// 	}, /*OPLUS_CAM_CAL_DATA_PDAF*/
// 	{
// 		.OtpInfoLen = 8,
// 		.OtpInfo = {{0x0000, 17}, {0x0006, 2}, {0x0008, 2}, {0x000a, 2}, {0x0092, 7}, {0x0092, 2}, {0x0094, 2}, {0x00b0, 24}},
// 		.isAFCodeOffset = KAL_FALSE,
// 	}, /*OPLUS_CAM_CAL_DATA_CAMERA_INFO-modid\sensor\lens\vcmid\af\macpos\infpos\qrcode\*/
// 	{
// 		.OtpInfoLen = 1,
// 		.OtpInfo = {{0x0008, 2}},
// 	}, /*OPLUS_CAM_CAL_DATA_DUMP*/
// 	{
// 		.OtpInfoLen = 1,
// 		.OtpInfo = {{0x0008, 2}},
// 	}, /*OPLUS_CAM_CAL_DATA_LENS_ID*/
// 	{
// 		.OtpInfoLen = 0,
// 	}, /*OPLUS_CAM_CAL_DATA_QSC*/
// 	{
// 		.OtpInfoLen = 0,
// 	}, /*OPLUS_CAM_CAL_DATA_LRC*/
// 	{
// 		.OtpInfoLen = 1,
// 		.OtpInfo = {{0x0000, 16384}},
// 	}, /*OPLUS_CAM_CAL_DATA_ALL*/
// };

/*
static int whoopassc1wide_get_pdafblock_info(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	struct SET_PD_BLOCK_INFO_T *PDAFinfo;

	PDAFinfo = (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));
	switch (*feature_data) {
  		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			imgsensor_pd_info.i4BlockNumX = 512;
			imgsensor_pd_info.i4BlockNumY = 382;
			memcpy((void *)PDAFinfo,
 				(void *)&imgsensor_pd_info,
  				sizeof(struct SET_PD_BLOCK_INFO_T));
 			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			imgsensor_pd_info.i4BlockNumX = 248;
			imgsensor_pd_info.i4BlockNumY = 144;
			memcpy((void *)PDAFinfo,
 				(void *)&imgsensor_pd_info,
  				sizeof(struct SET_PD_BLOCK_INFO_T));
 			break;
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			imgsensor_pd_info.i4BlockNumX = 248;
			imgsensor_pd_info.i4BlockNumY = 128;
			memcpy((void *)PDAFinfo,
 				(void *)&imgsensor_pd_info,
  				sizeof(struct SET_PD_BLOCK_INFO_T));
 			break;
		case SENSOR_SCENARIO_ID_CUSTOM1:
			imgsensor_pd_info.i4BlockNumX = 200;
			imgsensor_pd_info.i4BlockNumY = 150;
			memcpy((void *)PDAFinfo,
 				(void *)&imgsensor_pd_info,
  				sizeof(struct SET_PD_BLOCK_INFO_T));
 			break;
		case SENSOR_SCENARIO_ID_CUSTOM2:
			imgsensor_pd_info.i4BlockNumX = 112;
			imgsensor_pd_info.i4BlockNumY = 84;
			memcpy((void *)PDAFinfo,
 				(void *)&imgsensor_pd_info,
  				sizeof(struct SET_PD_BLOCK_INFO_T));
 			break;
		default:
			break;
	}
	return 0;

}
*/

// static int whoopassc1wide_get_cloud_otp_info(struct subdrv_ctx *ctx, u8 *para, u32 *len)
// {
// 	u64 *feature_data = (u64 *)para;
// 	struct SENSOR_OTP_INFO_STRUCT *cloudinfo;
// 	LOG_INF("SENSOR_FEATURE_GET_CLOUD_OTP_INFO otp_type:%d", (UINT32)(*feature_data));
// 	cloudinfo = (struct SENSOR_OTP_INFO_STRUCT *)(uintptr_t)(*(feature_data + 1));
// 	switch (*feature_data) {
// 	case OPLUS_CAM_CAL_DATA_MODULE_VERSION:
// 	case OPLUS_CAM_CAL_DATA_PART_NUMBER:
// 	case OPLUS_CAM_CAL_DATA_SHADING_TABLE:
// 	case OPLUS_CAM_CAL_DATA_3A_GAIN:
// 	case OPLUS_CAM_CAL_DATA_PDAF:
// 	case OPLUS_CAM_CAL_DATA_CAMERA_INFO:
// 	case OPLUS_CAM_CAL_DATA_DUMP:
// 	case OPLUS_CAM_CAL_DATA_LENS_ID:
// 	case OPLUS_CAM_CAL_DATA_QSC:
// 	case OPLUS_CAM_CAL_DATA_LRC:
// 	case OPLUS_CAM_CAL_DATA_ALL:
// 		memcpy((void *)cloudinfo, (void *)&cloud_otp_info[*feature_data], sizeof(struct SENSOR_OTP_INFO_STRUCT));
// 		break;
// 	default:
// 		break;
// 	}
// 	return 0;
// }

// static void read_unique_sensorid(struct subdrv_ctx *ctx)
// {
// 	u8 i = 0;
// 	LOG_INF("read sensor unique sensorid");
// 	while (ctx->s_ctx.i2c_addr_table[i] != 0xFF) {
// 		ctx->i2c_write_id = ctx->s_ctx.i2c_addr_table[i];
// 		subdrv_i2c_wr_u8(ctx, 0x0103, 0x01);
// 		subdrv_i2c_wr_u8(ctx, 0x3d84, 0x00);
// 		subdrv_i2c_wr_u8(ctx, 0x3d85, 0x1b);
// 		subdrv_i2c_wr_u8(ctx, 0x0100, 0x01);
// 		msleep(5);
// 		if (adaptor_i2c_rd_p8(ctx->i2c_client, ctx->i2c_write_id >> 1, WHOOPASSC1WIDE_UNIQUE_SENSOR_ID_ADDR,
// 			&(whoopassc1wide_unique_id[0]), WHOOPASSC1WIDE_UNIQUE_SENSOR_ID_LENGTH) < 0) {
// 			LOG_INF("Read sensor unique sensorid fail. i2c_write_id: 0x%x\n", ctx->i2c_write_id);
// 		}
// 		i++;
// 	}
// }

// static int whoopassc1wide_get_unique_sensorid(struct subdrv_ctx *ctx, u8 *para, u32 *len)
// {
// 	u32 *feature_return_para_32 = (u32 *)para;
// 	*len = WHOOPASSC1WIDE_UNIQUE_SENSOR_ID_LENGTH;
// 	memcpy(feature_return_para_32, whoopassc1wide_unique_id,
// 		WHOOPASSC1WIDE_UNIQUE_SENSOR_ID_LENGTH);
// 	LOG_INF("para :%x, get unique sensorid", *para);
// 	return 0;
// }

static int whoopassc1wide_get_eeprom_comdata(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	LOG_INF("+");
	memcpy(para, (u8*)(&oplus_eeprom_info), sizeof(oplus_eeprom_info));
	*len = sizeof(oplus_eeprom_info);
	return 0;
}

static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
{
	kal_uint16 get_byte = 0;

	adaptor_i2c_rd_u8(ctx->i2c_client, WHOOPASSC1WIDE_EEPROM_ADDR >> 1, addr, (u8 *)&get_byte);
	return get_byte;
}

static kal_int32 table_write_eeprom_one_packet(struct subdrv_ctx *ctx,
        kal_uint16 addr, kal_uint8 *para, kal_uint32 len)
{
    kal_int32 ret = ERROR_NONE;
    ret = adaptor_i2c_wr_p8(ctx->i2c_client, WHOOPASSC1WIDE_EEPROM_ADDR >> 1,
            addr, para, len);

    return ret;
}

static kal_int32 write_eeprom_protect(struct subdrv_ctx *ctx, kal_uint16 enable)
{
    kal_int32 ret = ERROR_NONE;
    kal_uint16 reg = 0xE000;

    if (enable) {
        adaptor_i2c_wr_u8(ctx->i2c_client, WHOOPASSC1WIDE_EEPROM_ADDR >> 1, reg, (WHOOPASSC1WIDE_EEPROM_ADDR & 0xFE) | 0x01);
    }
    else {
        adaptor_i2c_wr_u8(ctx->i2c_client, WHOOPASSC1WIDE_EEPROM_ADDR >> 1, reg, WHOOPASSC1WIDE_EEPROM_ADDR & 0xFE);
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
        if (((pStereodata->uSensorId == WHOOPASSC1WIDE_SENSOR_ID) || (pStereodata->uSensorId == WHOOPASSS1WIDE_SENSOR_ID)
            || (pStereodata->uSensorId == WHOOPASSC2WIDE_SENSOR_ID) || (pStereodata->uSensorId == WHOOPASSS2WIDE_SENSOR_ID))
            && (data_length == CALI_DATA_SLAVE_LENGTH)
            && (data_base == WHOOPASSC1WIDE_STEREO_MW_START_ADDR)) {
            LOG_INF("Write: %x %x %x %x\n", pData[0], pData[39], pData[40], pData[1556]);

            eeprom_64align_write(ctx, data_base, pData, data_length);

            LOG_INF("com_0:0x%x\n", read_cmos_eeprom_8(ctx, data_base));
            LOG_INF("com_39:0x%x\n", read_cmos_eeprom_8(ctx, data_base+39));
            LOG_INF("innal_40:0x%x\n", read_cmos_eeprom_8(ctx, data_base+40));
            LOG_INF("innal_1556:0x%x\n", read_cmos_eeprom_8(ctx, data_base+1556));
            LOG_INF("write_Module_data Write end\n");

        } else if (((pStereodata->uSensorId == WHOOPASSC1WIDE_SENSOR_ID) || (pStereodata->uSensorId == WHOOPASSS1WIDE_SENSOR_ID)
            || (pStereodata->uSensorId == WHOOPASSC2WIDE_SENSOR_ID) || (pStereodata->uSensorId == WHOOPASSS2WIDE_SENSOR_ID))
            && (data_length < AESYNC_DATA_LENGTH_TOTAL)
            && (data_base == WHOOPASSC1WIDE_AESYNC_START_ADDR)) {
            LOG_INF("write main aesync: %x %x %x %x %x %x %x %x\n", pData[0], pData[1],
                pData[2], pData[3], pData[4], pData[5], pData[6], pData[7]);

            eeprom_64align_write(ctx, data_base, pData, data_length);

            LOG_INF("readback main aesync: %x %x %x %x %x %x %x %x\n",
                    read_cmos_eeprom_8(ctx, WHOOPASSC1WIDE_AESYNC_START_ADDR),
                    read_cmos_eeprom_8(ctx, WHOOPASSC1WIDE_AESYNC_START_ADDR+1),
                    read_cmos_eeprom_8(ctx, WHOOPASSC1WIDE_AESYNC_START_ADDR+2),
                    read_cmos_eeprom_8(ctx, WHOOPASSC1WIDE_AESYNC_START_ADDR+3),
                    read_cmos_eeprom_8(ctx, WHOOPASSC1WIDE_AESYNC_START_ADDR+4),
                    read_cmos_eeprom_8(ctx, WHOOPASSC1WIDE_AESYNC_START_ADDR+5),
                    read_cmos_eeprom_8(ctx, WHOOPASSC1WIDE_AESYNC_START_ADDR+6),
                    read_cmos_eeprom_8(ctx, WHOOPASSC1WIDE_AESYNC_START_ADDR+7));
            LOG_INF("AESync write_Module_data Write end\n");
        } else {
            LOG_INF("Invalid Sensor id:0x%x write eeprom\n", pStereodata->uSensorId);
            return -1;
        }
    } else {
        LOG_INF("whoopassc1wide write_Module_data pStereodata is null\n");
        return -1;
    }
    return ret;
}

static int whoopassc1wide_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    int ret = ERROR_NONE;
    ret = write_Module_data(ctx, (ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(para));
    if (ret != ERROR_NONE) {
        LOG_INF("ret=%d\n", ret);
    }
	return 0;
}

static int whoopassc1wide_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	UINT16 *feature_data_16 = (UINT16 *) para;
	UINT32 *feature_return_para_32 = (UINT32 *) para;
	if(*len > CALI_DATA_SLAVE_LENGTH)
		*len = CALI_DATA_SLAVE_LENGTH;
	LOG_INF("feature_data mode:%d  lens:%d", *feature_data_16, *len);
	read_whoopassc1wide_eeprom_info(ctx, EEPROM_META_STEREO_MW_MAIN_DATA,
			(BYTE *)feature_return_para_32, *len);
	return 0;
}

static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size)
{
	if (adaptor_i2c_rd_p8(ctx->i2c_client, WHOOPASSC1WIDE_EEPROM_ADDR >> 1,
			addr, data, size) < 0) {
		return false;
	}
	return true;
}

static void read_otp_info(struct subdrv_ctx *ctx)
{
	DRV_LOGE(ctx, "whoopassc1wide read_otp_info begin\n");
	read_cmos_eeprom_p8(ctx, 0, otp_data_checksum, sizeof(otp_data_checksum));
	DRV_LOGE(ctx, "whoopassc1wide read_otp_info end\n");
}

static int whoopassc1wide_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

static int whoopassc1wide_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	get_imgsensor_id(ctx, (u32 *)para);
	return 0;
}

static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id)
{
	u8 i = 0;
	u8 retry = 2;
	static bool first_read = KAL_TRUE;
	u32 addr_h = ctx->s_ctx.reg_addr_sensor_id.addr[0];
	u32 addr_l = ctx->s_ctx.reg_addr_sensor_id.addr[1];
	u32 addr_ll = ctx->s_ctx.reg_addr_sensor_id.addr[2];
	LOG_INF("rst delay = %d, func: %s, line: %d\n", pw_seq[1].delay, __FUNCTION__, __LINE__);
	while (ctx->s_ctx.i2c_addr_table[i] != 0xFF) {
		ctx->i2c_write_id = ctx->s_ctx.i2c_addr_table[i];
		do {
			*sensor_id = (subdrv_i2c_rd_u8(ctx, addr_h) << 8) |
				subdrv_i2c_rd_u8(ctx, addr_l);
			if (addr_ll)
				*sensor_id = ((*sensor_id) << 8) | subdrv_i2c_rd_u8(ctx, addr_ll);
			LOG_INF("i2c_write_id(0x%x) sensor_id(0x%x/0x%x)\n",
				ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == WHOOPASSC1WIDE_IMGSENSOR_ID) {
				*sensor_id = ctx->s_ctx.sensor_id;
				get_imgsensor_id_from_dts(ctx, sensor_id);
				if (first_read) {
					read_eeprom_common_data(ctx, &oplus_eeprom_info, oplus_eeprom_addr_table);
					//read_unique_sensorid(ctx);
					first_read = KAL_FALSE;
					//update CTLE
					update_CTLE(ctx);
					msg_buf = kmalloc(MAX_BURST_LEN, GFP_KERNEL);
					if(!msg_buf) {
						LOG_INF("boot stage, malloc msg_buf error");
					}
				}
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail. i2c_write_id: 0x%x\n", ctx->i2c_write_id);
			LOG_INF("sensor_id = 0x%x, ctx->s_ctx.sensor_id = 0x%x\n",
				*sensor_id, ctx->s_ctx.sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != ctx->s_ctx.sensor_id) {
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}

static u16 get_gain2reg(u32 gain)
{
	return gain * (0x400) / BASEGAIN;
}

static int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;
	/* get sensor id */
	if (get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

	LOG_INF("%s", SENSOR_NAME);
	// // software reset
	// subdrv_i2c_wr_regs_u8(ctx, whoopassc1wide_soft_reset, ARRAY_SIZE(whoopassc1wide_soft_reset));
	// msleep(1);

	//sensor_init(ctx);
	whoopassc1wide_i2c_burst_wr_regs_u8(ctx, ctx->s_ctx.init_setting_table, ctx->s_ctx.init_setting_len);

	// /*PDC setting*/
	if (ctx->s_ctx.s_cali != NULL) {
		ctx->s_ctx.s_cali((void*)ctx);
	} else {
		write_sensor_Cali(ctx);
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

static void whoopassc1wide_get_sensor_cali(void* arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	u16 idx = 0;
	u8 support = FALSE;
	u8 *buf = NULL;
	u16 size = 0;
	u16 addr = 0;
	u8 write_id = 0;
	struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;

	/* Probe EEPROM device */
	if (!probe_eeprom(ctx))
		return;

	idx = ctx->eeprom_index;

	/* pdc data */
	support = info[idx].pdc_support;
	size = info[idx].pdc_size;
	addr = info[idx].addr_pdc;
	buf = info[idx].pdc_table;

	if (support && size > 0) {
		// Check PDC validation
		//pdc_is_valid = i2c_read_eeprom(ctx, OTP_PDC_VALID_ADDR);
		write_id = ctx->s_ctx.eeprom_info[idx].i2c_write_id;
		adaptor_i2c_rd_u8(ctx->i2c_client, write_id >> 1, OTP_PDC_VALID_ADDR, (u8 *)&pdc_is_valid);


		if(pdc_is_valid != OTP_PDC_IS_VALID_VAL) {
			DRV_LOGE(ctx, "pdc is invalid %d", pdc_is_valid);
			return;
		}

		if (info[idx].preload_pdc_table == NULL) {
			info[idx].preload_pdc_table = kmalloc(size, GFP_KERNEL);
			if (buf == NULL)
				i2c_multi_read_eeprom(ctx, addr, size, info[idx].preload_pdc_table);
			else
				memcpy(info[idx].preload_pdc_table, buf, size);
			DRV_LOG(ctx, "preload pdc data %u bytes", size);
		} else {
			DRV_LOG(ctx, "pdc data is already preloaded %u bytes", size);
		}
	}

	ctx->is_read_preload_eeprom = 1;
}

static void whoopassc1wide_set_sensor_cali(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	u16 idx = 0;
	u8 support = FALSE;
	u8 *pbuf = NULL;
	u16 size = 0;
	u16 addr = 0;
	u16 i = 0;
	u16 start_page = 0x06;
	u16 value = 0;
	struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;

	if (!probe_eeprom(ctx)) {
		DRV_LOG(ctx, "eeprom not find\n");
		return;
	}
	idx = ctx->eeprom_index;
	/* pdc data */
	support = info[idx].pdc_support;
	if (support && (pdc_is_valid == OTP_PDC_IS_VALID_VAL)) {
		pbuf = info[idx].preload_pdc_table;
		size = info[idx].pdc_size;
		addr = info[idx].sensor_reg_addr_pdc;
		if (pbuf != NULL && addr > 0 && size > 0) {
			subdrv_i2c_wr_u8(ctx, 0x031c, 0x60);
			subdrv_i2c_wr_u8(ctx, 0x0004, 0x06);
			subdrv_i2c_wr_u8(ctx, 0x001f, 0x01);
			mdelay(5);
			DRV_LOG(ctx, "start write crosstalk_data!\n");
			subdrv_i2c_wr_u8(ctx, 0x0004, start_page);
			subdrv_i2c_wr_seq_p8(ctx, addr, pbuf, XTC_SENSOR_LENGTH_PART1);
			for (i = 0; i < 10; i++){
				value = subdrv_i2c_rd_u8(ctx, addr + i);
				DRV_LOG(ctx, "pdc addr:%x, read:%x\n", (addr + i), value);
			}
			pbuf += XTC_SENSOR_LENGTH_PART1;
			subdrv_i2c_wr_u8(ctx, 0x0004, start_page + 0x10);
			subdrv_i2c_wr_seq_p8(ctx, addr, pbuf, XTC_SENSOR_LENGTH_PART2);
			for (i = 0; i < 10; i++){
				value = subdrv_i2c_rd_u8(ctx, addr + i);
				DRV_LOG(ctx, "pdc addr:%x, read:%x\n", (addr + i), value);
			}
			subdrv_i2c_wr_u8(ctx, 0x0004, 0x07);
			subdrv_i2c_wr_u8(ctx, 0x031c, 0x9b);
		}

	} else {
		DRV_LOG(ctx, "not need set pdc\n");
	}
}

#define INVALID_TEMP_VALUE (0x7FFF)

static int get_sensor_temperature(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	u8 temperature = 0;
	int temperature_convert = 0;
	if (ctx->s_ctx.reg_addr_temp_read) {
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_temp_en, 0x01); //trigger temperature calculation
		temperature = subdrv_i2c_rd_u8(ctx, ctx->s_ctx.reg_addr_temp_read);

		temperature_convert = (char)temperature;  //0~80 Celsius
	}
	DRV_LOG(ctx, "reg_val:0x%x, temperature: %d degrees\n", temperature, temperature_convert);

	if (temperature_convert < 0 || temperature_convert > 80) {
		temperature_convert = INVALID_TEMP_VALUE;
	}

	return temperature_convert;
}

static void set_group_hold(void *arg, u8 en)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	if (en) {
		set_i2c_buffer(ctx, 0x0104, 0x01);
	} else {
		set_i2c_buffer(ctx, 0x0104, 0x00);
	}
}

void whoopassc1wide_get_min_shutter_by_scenario(struct subdrv_ctx *ctx,
		enum SENSOR_SCENARIO_ID_ENUM scenario_id,
		u64 *min_shutter, u64 *exposure_step)
{
	u32 exp_cnt = 0;
	exp_cnt = ctx->s_ctx.mode[scenario_id].exp_cnt;
	check_current_scenario_id_bound(ctx);
	LOG_INF("sensor_mode_num[%d]", ctx->s_ctx.sensor_mode_num);
	if (scenario_id < ctx->s_ctx.sensor_mode_num) {
		switch (ctx->s_ctx.mode[scenario_id].hdr_mode) {
			case HDR_RAW_STAGGER:
				*exposure_step = ctx->s_ctx.exposure_step * exp_cnt;
				*min_shutter = ctx->s_ctx.exposure_min * exp_cnt;
				break;
			case HDR_NONE:
				if (ctx->s_ctx.mode[scenario_id].coarse_integ_step &&
					ctx->s_ctx.mode[scenario_id].min_exposure_line) {
					*exposure_step = ctx->s_ctx.mode[scenario_id].coarse_integ_step;
					*min_shutter = ctx->s_ctx.mode[scenario_id].min_exposure_line;
				} else {
					*exposure_step = ctx->s_ctx.exposure_step;
					*min_shutter = ctx->s_ctx.exposure_min;
				}
				break;
			default:
				*exposure_step = ctx->s_ctx.exposure_step;
				*min_shutter = ctx->s_ctx.exposure_min;
				break;
		}
	} else {
		DRV_LOG(ctx, "over sensor_mode_num[%d], use default", ctx->s_ctx.sensor_mode_num);
		*exposure_step = ctx->s_ctx.exposure_step;
		*min_shutter = ctx->s_ctx.exposure_min;
	}
	DRV_LOG(ctx, "scenario_id[%d] exposure_step[%llu] min_shutter[%llu]\n", scenario_id, *exposure_step, *min_shutter);
}

int whoopassc1wide_get_min_shutter_by_scenario_adapter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *) para;
	whoopassc1wide_get_min_shutter_by_scenario(ctx,
		(enum SENSOR_SCENARIO_ID_ENUM)*(feature_data),
		feature_data + 1, feature_data + 2);
	return 0;
}

/*
static int whoopassc1wide_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	enum SENSOR_SCENARIO_ID_ENUM scenario_id;
	struct mtk_hdr_ae *ae_ctrl = NULL;
	u64 *feature_data = (u64 *)para;
	enum SENSOR_SCENARIO_ID_ENUM pre_seamless_scenario_id;
	u32 frame_length_in_lut[IMGSENSOR_STAGGER_EXPOSURE_CNT] = {0};
	u32 exp_cnt = 0;

	if (feature_data == NULL) {
		DRV_LOGE(ctx, "input scenario is null!");
		return ERROR_INVALID_SCENARIO_ID;
	}
	scenario_id = *feature_data;
	if ((feature_data + 1) != NULL)
		ae_ctrl = (struct mtk_hdr_ae *)((uintptr_t)(*(feature_data + 1)));
	else
		DRV_LOGE(ctx, "no ae_ctrl input");

	check_current_scenario_id_bound(ctx);
	DRV_LOG(ctx, "E: set seamless switch %u %u\n", ctx->current_scenario_id, scenario_id);
	if (!ctx->extend_frame_length_en)
		DRV_LOGE(ctx, "please extend_frame_length before seamless_switch!\n");
	ctx->extend_frame_length_en = FALSE;

	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOGE(ctx, "invalid sid:%u, mode_num:%u\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		return ERROR_INVALID_SCENARIO_ID;
	}
	if (ctx->s_ctx.mode[scenario_id].seamless_switch_group == 0 ||
		ctx->s_ctx.mode[scenario_id].seamless_switch_group !=
			ctx->s_ctx.mode[ctx->current_scenario_id].seamless_switch_group) {
		DRV_LOGE(ctx, "seamless_switch not supported\n");
		return ERROR_INVALID_SCENARIO_ID;
	}
	if (ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table == NULL) {
		DRV_LOGE(ctx, "Please implement seamless_switch setting\n");
		return ERROR_INVALID_SCENARIO_ID;
	}

	exp_cnt = ctx->s_ctx.mode[scenario_id].exp_cnt;
	ctx->is_seamless = TRUE;
	pre_seamless_scenario_id = ctx->current_scenario_id;
	update_mode_info(ctx, scenario_id);

	subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);
	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_fast_mode, 0x02);
	if (ctx->s_ctx.reg_addr_fast_mode_in_lbmf &&
		(ctx->s_ctx.mode[scenario_id].hdr_mode == HDR_RAW_LBMF ||
		ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode == HDR_RAW_LBMF))
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_fast_mode_in_lbmf, 0x4);

	update_mode_info(ctx, scenario_id);
	i2c_table_write(ctx,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_len);

	DRV_LOG(ctx, "write seamless switch setting done\n");
	if (ae_ctrl) {
		switch (ctx->s_ctx.mode[scenario_id].hdr_mode) {
		case HDR_RAW_STAGGER:
			set_multi_shutter_frame_length(ctx, (u64 *)&ae_ctrl->exposure, exp_cnt, 0);
			set_multi_gain(ctx, (u32 *)&ae_ctrl->gain, exp_cnt);
			break;
		case HDR_RAW_LBMF:
			set_multi_shutter_frame_length_in_lut(ctx,
				(u64 *)&ae_ctrl->exposure, exp_cnt, 0, frame_length_in_lut);
			set_multi_gain_in_lut(ctx, (u32 *)&ae_ctrl->gain, exp_cnt);
			break;
		default:
			set_shutter(ctx, ae_ctrl->exposure.le_exposure);
			set_gain(ctx, ae_ctrl->gain.le_gain);
			break;
		}
	}
	common_get_prsh_length_lines(ctx, ae_ctrl, pre_seamless_scenario_id, scenario_id);

	if (ctx->s_ctx.seamless_switch_prsh_length_lc > 0) {
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_prsh_mode, 0x01);

		subdrv_i2c_wr_u8(ctx,
				ctx->s_ctx.reg_addr_prsh_length_lines.addr[0],
				(ctx->s_ctx.seamless_switch_prsh_length_lc >> 16) & 0xFF);
		subdrv_i2c_wr_u8(ctx,
				ctx->s_ctx.reg_addr_prsh_length_lines.addr[1],
				(ctx->s_ctx.seamless_switch_prsh_length_lc >> 8)  & 0xFF);
		subdrv_i2c_wr_u8(ctx,
				ctx->s_ctx.reg_addr_prsh_length_lines.addr[2],
				(ctx->s_ctx.seamless_switch_prsh_length_lc) & 0xFF);

		DRV_LOG(ctx, "seamless switch pre-shutter set(%u)\n", ctx->s_ctx.seamless_switch_prsh_length_lc);
	} else
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_prsh_mode, 0x00);

	subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);

	ctx->fast_mode_on = TRUE;
	ctx->ref_sof_cnt = ctx->sof_cnt;
	ctx->is_seamless = FALSE;
	DRV_LOG(ctx, "X: set seamless switch done\n");
	return ERROR_NONE;
}
*/

static int whoopassc1wide_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	if (mode) {
		LOG_INF("mode(%u->%u)\n", ctx->test_pattern, mode);
	/* 1:Solid Color 2:Color Bar 5:Black */
		switch (mode) {
		case 2:
			subdrv_i2c_wr_u8(ctx, 0x035D, 0x01);
			subdrv_i2c_wr_u8(ctx, 0x035E, 0x10);
			break;
		case 5:
			subdrv_i2c_wr_u8(ctx, 0x035D, 0x01);
			subdrv_i2c_wr_u8(ctx, 0x035E, 0x00);
			break;
		default:
			LOG_INF("unsupported mode(%u)\n", mode);
			break;
		}
	} else if (ctx->test_pattern) {
		LOG_INF("mode(%u->%u)\n", ctx->test_pattern, mode);
		subdrv_i2c_wr_u8(ctx, 0x035D, 0x00);
	}
	ctx->test_pattern = mode;

	return 0;
}

static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id)
{
	memcpy(&(ctx->s_ctx), &static_ctx, sizeof(struct subdrv_static_ctx));
	subdrv_ctx_init(ctx);
	//hw_init_time
	for (int scenario_id = 0; scenario_id < ctx->s_ctx.sensor_mode_num; scenario_id++){
		 ctx->hw_time_info[scenario_id].init_time_ns = 1 * 1000000;
	}
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;
	return 0;
}

static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt)
{
	DRV_LOG(ctx, "sof_cnt(%u) ctx->ref_sof_cnt(%u) ctx->fast_mode_on(%d)",
		sof_cnt, ctx->ref_sof_cnt, ctx->fast_mode_on);
	ctx->sof_cnt = sof_cnt;

	return 0;
}

static void whoopassc1wide_set_multi_shutter_frame_length(struct subdrv_ctx *ctx, u64 *shutters, u16 exp_cnt, u16 frame_length)
{

	if(exp_cnt == 1) {  //force to 1exp func
		whoopassc1wide_set_shutter_frame_length_convert(ctx, shutters[0], frame_length);
		return ;
	}
}

static int whoopassc1wide_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	whoopassc1wide_set_multi_shutter_frame_length(ctx, (u64 *)(*feature_data),
		(u64) (*(feature_data + 1)), (u64) (*(feature_data + 2)));
	return 0;
}

static void whoopassc1wide_set_hdr_tri_shutter(struct subdrv_ctx *ctx, u64 *shutters, u16 exp_cnt)
{
	int i = 0;
	u64 values[3] = {0};

	if (shutters != NULL) {
		for (i = 0; i < 3; i++)
			values[i] = (u64) *(shutters + i);
	}
	whoopassc1wide_set_multi_shutter_frame_length(ctx, values, exp_cnt, 0);
}

static int whoopassc1wide_set_hdr_tri_shutter2(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	whoopassc1wide_set_hdr_tri_shutter(ctx, feature_data, 2);
	return 0;
}

static int whoopassc1wide_set_hdr_tri_shutter3(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	whoopassc1wide_set_hdr_tri_shutter(ctx, feature_data, 3);
	return 0;
}

static bool whoopassc1wide_set_long_exposure(struct subdrv_ctx *ctx)
{
	/* add for long shutter */
	u32 shutter = ctx->exposure[IMGSENSOR_STAGGER_EXPOSURE_LE];
	static bool bNeedSetNormalMode = KAL_FALSE;
	kal_uint32 cal_longexp = 0;
	kal_uint8 long_exp_h = 0;
	kal_uint8 long_exp_m = 0;
	kal_uint8 long_exp_l = 0;
	kal_uint16 const_normal_exp_val = 0xc2ec;     //500ms
	kal_uint16 const_normal_exp_framelength = const_normal_exp_val + ctx->s_ctx.exposure_margin;

	if (shutter >= const_normal_exp_val) {
		LOG_INF("enter long shutter\n");
		cal_longexp = (shutter - const_normal_exp_val); //500ms base
		long_exp_h = (cal_longexp >> 16) & 0x3F;
		long_exp_m = (cal_longexp >> 8) & 0xFF;
		long_exp_l = cal_longexp & 0xFF;

		set_i2c_buffer(ctx,	0x0202, (const_normal_exp_val >> 8) & 0xFF);
		set_i2c_buffer(ctx,	0x0203,  const_normal_exp_val & 0xFF);
		set_i2c_buffer(ctx,	0x0340, (const_normal_exp_framelength >> 8) & 0xFF);
		set_i2c_buffer(ctx,	0x0341,  const_normal_exp_framelength & 0xFF);
		set_i2c_buffer(ctx,	0x0230, 0x0c);
		set_i2c_buffer(ctx,	0x022f, long_exp_l);
		set_i2c_buffer(ctx,	0x022e, long_exp_m);
		set_i2c_buffer(ctx,	0x022d, long_exp_h);
		bNeedSetNormalMode = KAL_TRUE;

		return 1;
	} else {
		if (bNeedSetNormalMode) {
			LOG_INF("exit long shutter\n");
			set_i2c_buffer(ctx,	0x0230, 0x08);
			set_i2c_buffer(ctx,	0x022d, 0x00);
			set_i2c_buffer(ctx,	0x022e, 0x00);
			set_i2c_buffer(ctx,	0x022f, 0x00);
			bNeedSetNormalMode = KAL_FALSE;
		}
	}

	return 0;
}

void whoopassc1wide_set_shutter_frame_length_convert(struct subdrv_ctx *ctx, u64 shutter, u32 frame_length)
{
	int fine_integ_line = 0;
	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);

	ctx->frame_length = frame_length ? frame_length : ctx->min_frame_length;
	check_current_scenario_id_bound(ctx);
	/* check boundary of shutter */
	fine_integ_line = ctx->s_ctx.mode[ctx->current_scenario_id].fine_integ_line;
	shutter = FINE_INTEG_CONVERT(shutter, fine_integ_line);
	shutter = max_t(u64, shutter,
		(u64)ctx->s_ctx.mode[ctx->current_scenario_id].multi_exposure_shutter_range[0].min);
	shutter = min_t(u64, shutter,
		(u64)ctx->s_ctx.mode[ctx->current_scenario_id].multi_exposure_shutter_range[0].max);
	/* check boundary of framelength */
	ctx->frame_length = max((u32)shutter + ctx->s_ctx.exposure_margin, ctx->frame_length);
	ctx->frame_length = min(ctx->frame_length, ctx->s_ctx.frame_length_max);
	ctx->frame_length = max(ctx->frame_length, ctx->min_frame_length);
	/* restore shutter */
	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	ctx->exposure[0] = (u32) shutter;
	/* group hold start */
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 1);
	/* enable auto extend */
	if (ctx->s_ctx.reg_addr_auto_extend)
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_auto_extend, 0x01);

	if (whoopassc1wide_set_long_exposure(ctx) == 0) { // Long exposure does not require setup framelength
		/* write framelength */
		if (set_auto_flicker(ctx, 0) || frame_length || !ctx->s_ctx.reg_addr_auto_extend)
			write_frame_length(ctx, ctx->frame_length);
		/* write shutter */
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
	}

	DRV_LOG(ctx, "exp[0x%x], fll(input/output):%u/%u, flick_en:%d\n",
		ctx->exposure[0], frame_length, ctx->frame_length, ctx->autoflicker_en);
	if (!ctx->ae_ctrl_gph_en) {
		if (gph)
			ctx->s_ctx.s_gph((void *)ctx, 0);
		commit_i2c_buffer(ctx);
	}
	/* group hold end */
}

static int whoopassc1wide_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	whoopassc1wide_set_shutter_frame_length_convert(ctx, ((u64*)para)[0], ((u64*)para)[1]);
	return 0;
}

//static void whoopassc1wide_set_shutter_convert(struct subdrv_ctx *ctx, u64 shutter)
//{
//    whoopassc1wide_set_shutter_frame_length_convert(ctx, shutter, 0);
//}
static int whoopassc1wide_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	whoopassc1wide_set_shutter_frame_length_convert(ctx, ((u64*)para)[0], 0);
	return 0;
}
void whoopassc1wide_set_dummy(struct subdrv_ctx *ctx)
{
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

static int whoopassc1wide_i2c_burst_wr_regs_u8(struct subdrv_ctx * ctx, u16 * list, u32 len)
{
	adapter_i2c_burst_wr_regs_u8(ctx, ctx->i2c_write_id >> 1, list, len);
	return 	0;
}

//addr16 data8
static int adapter_i2c_burst_wr_regs_u8(struct subdrv_ctx * ctx ,
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

	/* each msg contains addr(u16) + val(u8 *) */
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
		pbuf[2] = plist[1] & 0xff;

		pbuf += 3;
		pmsg->len = 3;
		per_sent += 1;

		for (i = 0; i < total - sent - 1; i++) {  //Maximum number of remaining cycles - 1
			if(plist[0] + 1 == plist[2] ) {  //Addresses are consecutive
				pbuf[0] = plist[3] & 0xff;

				pbuf += 1;
				pmsg->len += 1;
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

static void get_imgsensor_id_from_dts(struct subdrv_ctx *ctx, u32 *sensor_id) {
	struct subdrv_entry *m_subdrv_entry = &whoopassc1wide_mipi_raw_entry;
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

		if (of_sensor_names_cnt && (of_sensor_ids_ret == 0)) {
			for(index = 0; index < of_sensor_names_cnt; index++) {
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

static int whoopassc1wide_get_readout_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	u64 scenario_id = *feature_data;

	u64 pclk;
	u64 linelength;
	u64 readout = 0;
	MUINT16 h2_tg_size;

	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOGE(ctx, "invalid sid:%llu, mode_num:%u\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		return 0;
	}

	pclk       = ctx->s_ctx.mode[scenario_id].pclk;
	pclk       = max_t(u64, (u64)1, pclk);
	linelength = ctx->s_ctx.mode[scenario_id].linelength;
	h2_tg_size = ctx->s_ctx.mode[scenario_id].imgsensor_winsize_info.h2_tg_size;

	readout = (linelength * h2_tg_size * 1000000000 / pclk); /* unit: ns */

	feature_data[1] = readout;

	DRV_LOG(ctx, "%s scenario_id(%llu)  pclk(%llu) linelength(%llu) h2_tg_size(%u) readout(%llu)",
		__func__, scenario_id, pclk, linelength, h2_tg_size, readout);

	return 0;
}

static void update_CTLE(struct subdrv_ctx *ctx)
{
	for (int scenario_id = 0; scenario_id < ctx->s_ctx.sensor_mode_num; ++scenario_id){
		if (ctx->s_ctx.sensor_id == WHOOPASSC1WIDE_SENSOR_ID || ctx->s_ctx.sensor_id == WHOOPASSS1WIDE_SENSOR_ID) {
			ctx->s_ctx.mode[scenario_id].csi_param.dphy_ctle = WIDEC1_CTLE_LEVEL;
			ctx->s_ctx.mode[scenario_id].csi_param.dphy_eq_bw = WIDEC1_CTLE_EQBW;
			LOG_INF("update_CTLE_C1, scenario_id: %d\n", scenario_id);
		} else {
			LOG_INF("update_CTLE false\n");
		}
	}
}