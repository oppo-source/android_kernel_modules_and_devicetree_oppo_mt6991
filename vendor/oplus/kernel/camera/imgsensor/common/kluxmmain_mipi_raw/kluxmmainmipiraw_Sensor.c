// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *     kluxmmainmipiraw_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include "kluxmmainmipiraw_Sensor.h"

#define SENSOR_NAME  SENSOR_DRVNAME_KLUXMMAIN_MIPI_RAW

#define KLUXMMAIN_EEPROM_ADDR        (0xA0)
#define KLUXMMAIN_EERPOM_MAX_OFFSET  (0x4000)
#define KLUXMMAIN_AESYNC_START_ADDR     (0x2F90)

#define PFX "kluxmmain_camera_sensor"
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)

#ifdef  EEPROM_WRITE_DATA_MAX_LENGTH
#undef  EEPROM_WRITE_DATA_MAX_LENGTH
#endif
#define EEPROM_WRITE_DATA_MAX_LENGTH       (64)
#define KLUXMMAIN_STEREO_MW_START_ADDR  (0x2980)

#define  OTP_PDC_IS_VALID_VAL     (0x01)
#define  OTP_PDC_VALID_ADDR       (0x3060)
#define  XTC_SENSOR_ADDR          (0xc000)
#define  XTC_SENSOR_LENGTH        (3376)
#define  XTC_SENSOR_LENGTH_PART1  (3024)
#define  XTC_SENSOR_LENGTH_PART2  (3376 - 3024)

static u8 pdc_is_valid = 0;

#define KLUXMMAIN_IMGSENSOR_ID   (0xb787)

#define KLUXMMAIN_UNIQUE_SENSOR_ID_ADDR    (0x7000)
#define KLUXMMAIN_UNIQUE_SENSOR_ID_LENGTH  (16)

#define FPT_SEAMLESS_SUPPORT
#define  USE_SUBDRV_IXC_WR_REGS
static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct oplus_eeprom_info_struct  oplus_eeprom_info = {0};

static kal_uint8 otp_data_checksum[KLUXMMAIN_EERPOM_MAX_OFFSET] = {0};
static int get_sensor_temperature(void *arg);
#define MAX_BURST_LEN  (2048)
#define KAL_UINT32_MAX 4294967295
static u8 * msg_buf = NULL;
static void set_group_hold(void *arg, u8 en);

static void kluxmmain_set_mirror_flip(struct subdrv_ctx *ctx, u8 image_mirror);
static int kluxmmain_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int kluxmmain_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int kluxmmain_get_eeprom_comdata(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int kluxmmain_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int kluxmmain_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int kluxmmain_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int kluxmmain_get_min_shutter_by_scenario_adapter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int open(struct subdrv_ctx *ctx);
static int init_ctx(struct subdrv_ctx *ctx,    struct i2c_client *i2c_client, u8 i2c_write_id);
static int vsync_notify(struct subdrv_ctx *ctx,    unsigned int sof_cnt);
static void kluxmmain_set_dummyline(struct subdrv_ctx *ctx, u64 curr_shutter);
static int kluxmmain_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int kluxmmain_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
//static void kluxmmain_set_max_framerate(struct subdrv_ctx *ctx, kal_uint16 framerate, kal_bool min_framelength_en);
static void kluxmmain_set_shutter_frame_length_convert(struct subdrv_ctx *ctx, u64 shutter, u32 frame_length);
static void kluxmmain_set_multi_shutter_frame_length(struct subdrv_ctx *ctx, u64 *shutters, u16 exp_cnt, u16 frame_length);
static int kluxmmain_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len);
//static int kluxmmain_set_hdr_tri_shutter2(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int kluxmmain_set_hdr_tri_shutter3(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size);
static int kluxmmain_i2c_burst_wr_regs_u8(struct subdrv_ctx *ctx, u16 * list, u32 len);
#ifndef USE_SUBDRV_IXC_WR_REGS
static int adapter_i2c_burst_wr_regs_u8(struct subdrv_ctx * ctx,
        u16 addr, u16 *list, u32 len);
#endif
static void kluxmmain_get_sensor_cali(void* arg);
static void kluxmmain_set_sensor_cali(void *arg);
static int kluxmmain_get_readout_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int kluxmmain_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int kluxmmain_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void streaming_ctrl(struct subdrv_ctx *ctx, bool enable);

static void kluxmmain_seamless_start(struct subdrv_ctx *ctx);
static void kluxmmain_seamless_end(struct subdrv_ctx *ctx, kal_uint32 curr_vts, kal_uint32 curr_shutter, kal_uint32 tline_ns);
static kal_uint32 kluxmmain_get_tline_ns(struct subdrv_ctx *ctx, kal_uint32 curr_hts, kal_uint32 curr_pclk);
static int kluxmmain_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len);
// static int kluxmmain_get_seamless_scenarios(struct subdrv_ctx *ctx, u8 *para, u32 *len);

static void kluxmmain_feedback_awbgain(struct subdrv_ctx *ctx, kal_uint32 r_gain, kal_uint32 b_gain);
static int kluxmmain_set_awb_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static u16 get_gain2reg(u32 gain);
static int kluxmmain_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);

// static void update_CTLE(struct subdrv_ctx *ctx);
// #define WIDEC1_CTLE_LEVEL 1
// #define WIDEC1_CTLE_EQBW 3

static struct eeprom_map_info kluxmmain_eeprom_info[] = {
    { EEPROM_META_MODULE_ID, 0x0000, 0x0010, 0x0011, 2, true },
    { EEPROM_META_SENSOR_ID, 0x0006, 0x0010, 0x0011, 2, true },
    { EEPROM_META_LENS_ID, 0x0008, 0x0010, 0x0011, 2, true },
    { EEPROM_META_VCM_ID, 0x000A, 0x0010, 0x0011, 2, true },
    { EEPROM_META_MIRROR_FLIP, 0x000E, 0x0010, 0x0011, 1, true },
    { EEPROM_META_MODULE_SN, 0x00B0, 0x00C7, 0x00C8, 23, true },
    { EEPROM_META_AF_CODE, 0x0092, 0x0098, 0x0099, 6, true },
    { EEPROM_META_AF_FLAG, 0x0098, 0x0098, 0x0099, 1, true },
    { EEPROM_META_STEREO_DATA, 0x0000, 0x0000, 0x0000, 0x0000, false },
    { EEPROM_META_STEREO_MW_MAIN_DATA, KLUXMMAIN_STEREO_MW_START_ADDR, 0xFFFF, 0xFFFF, CALI_DATA_SLAVE_LENGTH, false },
    { EEPROM_META_STEREO_MT_MAIN_DATA, 0, 0, 0, 0, false },
    { EEPROM_META_STEREO_MT_MAIN_DATA_105CM, 0, 0, 0, 0, false },
    { EEPROM_META_DISTORTION_DATA, 0, 0, 0, 0, false },
};

static struct subdrv_feature_control feature_control_list[] = {
    {SENSOR_FEATURE_SET_TEST_PATTERN, kluxmmain_set_test_pattern},
    {SENSOR_FEATURE_CHECK_SENSOR_ID, kluxmmain_check_sensor_id},
    {SENSOR_FEATURE_GET_EEPROM_COMDATA, kluxmmain_get_eeprom_comdata},
    {SENSOR_FEATURE_SET_SENSOR_OTP, kluxmmain_set_eeprom_calibration},
    {SENSOR_FEATURE_GET_EEPROM_STEREODATA, kluxmmain_get_eeprom_calibration},
    {SENSOR_FEATURE_GET_SENSOR_OTP_ALL, kluxmmain_get_otp_checksum_data},
    {SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO, kluxmmain_get_min_shutter_by_scenario_adapter},
    {SENSOR_FEATURE_SET_ESHUTTER, kluxmmain_set_shutter},
    {SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME, kluxmmain_set_shutter_frame_length},
    //{SENSOR_FEATURE_SET_HDR_SHUTTER, kluxmmain_set_hdr_tri_shutter2},
    {SENSOR_FEATURE_SET_STREAMING_SUSPEND, kluxmmain_streaming_suspend},
    {SENSOR_FEATURE_SET_STREAMING_RESUME, kluxmmain_streaming_resume},
    {SENSOR_FEATURE_SET_HDR_TRI_SHUTTER, kluxmmain_set_hdr_tri_shutter3},
    {SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME, kluxmmain_set_multi_shutter_frame_length_ctrl},
    {SENSOR_FEATURE_SET_GAIN, kluxmmain_set_gain},
    {SENSOR_FEATURE_GET_READOUT_BY_SCENARIO, kluxmmain_get_readout_by_scenario},
    {SENSOR_FEATURE_SET_AWB_GAIN, kluxmmain_set_awb_gain},

    {SENSOR_FEATURE_SEAMLESS_SWITCH, kluxmmain_seamless_switch},
//    {SENSOR_FEATURE_GET_SEAMLESS_SCENARIOS, kluxmmain_get_seamless_scenarios},
};

static struct eeprom_info_struct eeprom_info[] = {
    {
        .header_id = 0x016b0071,
        .addr_header_id = 0x00000006,
        .i2c_write_id = KLUXMMAIN_EEPROM_ADDR,
        .pdc_support = TRUE,
        .pdc_size = XTC_SENSOR_LENGTH,
        .addr_pdc = 0x2330,
        .sensor_reg_addr_pdc = XTC_SENSOR_ADDR,
    },
};

static u32 kluxmmain_dcg_ratio_table_10bit[] = {4000};
static u32 kluxmmain_dcg_ratio_table_12bit[] = {4000};
static struct mtk_sensor_saturation_info imgsensor_saturation_info = {
    .gain_ratio = 1000,
    .OB_pedestal = 64,
    .saturation_level = 1023,
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
    .i4OffsetX = 0,
    .i4OffsetY = 0,
    .i4PitchX  = 0,
    .i4PitchY  = 0,
    .i4PairNum  = 0,
    .i4SubBlkW  = 0,
    .i4SubBlkH  = 0,
    .i4PosL = {{0, 0}},
    .i4PosR = {{0, 0}},
    .i4BlockNumX = 0,
    .i4BlockNumY = 0,
    .i4LeFirst = 0,
    .i4Crop = {
        /* <pre> <cap> <normal_video> <hs_video> <slim_video> */
        {0, 0}, {0, 0}, {0, 384}, {0, 384}, {0, 384},
        /* <cust1> <cust2> <cust3> <cust4> <cust5> */
        {0, 384}, {0, 0}, {0, 0}, {0, 0}, {0, 512},
        /* <cust6> <cust7> <cust8> <cust9> <cust10>*/
        {0, 0}, {0, 384}, {0, 384}, {0, 0}, {0, 0},
    },
    .iMirrorFlip = 3,
    .i4FullRawW = 4096,
    .i4FullRawH = 3072,
    .i4VCPackNum = 1,
    .PDAF_Support = PDAF_SUPPORT_CAMSV_QPD,
    .i4ModeIndex = 0x3,
    .sPDMapInfo[0] = {
        .i4PDPattern = 1,
        .i4BinFacX = 2,
        .i4BinFacY = 4,
        .i4PDRepetition = 0,
        .i4PDOrder = {1},
    },
};

static struct SET_PD_BLOCK_INFO_T imgsensor_partial_pd_info = {
    .i4OffsetX = 0,
    .i4OffsetY = 0,
    .i4PitchX = 16,
    .i4PitchY = 16,
    .i4PairNum = 8,
    .i4SubBlkW = 4,
    .i4SubBlkH = 8,
    .i4PosL = {{2, 1}, {2, 13}, {6, 1}, {6, 13}, {10, 1}, {10, 13}, {14, 1}, {14, 13}},
    .i4PosR = {{2, 5}, {2, 9}, {6, 5}, {6, 9}, {10, 5}, {10, 9}, {14, 5}, {14, 9}},
    .i4BlockNumX = 256,
    .i4BlockNumY = 192,
    .i4LeFirst = 0,
    .i4Crop = {
        {0, 0}, {0, 0}, {0, 0}, {0, 384}, {0, 0},
        {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
        {0, 0}, {0, 0}, {0, 0}, {0, 0}
    },
    .iMirrorFlip = 3,
    .i4FullRawW = 4096,
    .i4FullRawH = 3072,
    .i4ModeIndex = 0x3,
    .i4VCPackNum = 1,
    .PDAF_Support = PDAF_SUPPORT_CAMSV,
    /* VC's PD pattern description */
    .sPDMapInfo[0] = {
        .i4VCFeature = VC_PDAF_STATS_NE_PIX_1,
        .i4PDPattern = 2,
        .i4PDRepetition = 1,
        .i4PDOrder = {1}, /*R = 1, L = 0*/
    },
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_Fullsize_info = {
    .i4OffsetX = 0,
    .i4OffsetY = 0,
    .i4PitchX  = 0,
    .i4PitchY  = 0,
    .i4PairNum  = 0,
    .i4SubBlkW  = 0,
    .i4SubBlkH  = 0,
    .i4PosL = {{0, 0}},
    .i4PosR = {{0, 0}},
    .i4BlockNumX = 0,
    .i4BlockNumY = 0,
    .i4LeFirst = 0,
    .i4Crop = {
        /* <pre> <cap> <normal_video> <hs_video> <slim_video> */
        {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
        /* <cust1> <cust2> <cust3> <cust4> <cust5> */
        {0, 0}, {0, 0}, {0, 0}, {2048, 1536}, {0, 0},
        /* <cust6> <cust7> <cust8> <cust9> <cust10>*/
        {2048, 1536}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
    },
    .iMirrorFlip = 3,
    .i4FullRawW = 8192,
    .i4FullRawH = 6144,
    .i4VCPackNum = 1,
    .PDAF_Support = PDAF_SUPPORT_CAMSV_QPD,
    .i4ModeIndex = 0x3,
    .sPDMapInfo[0] = {
        .i4PDPattern = 1,
        .i4BinFacX = 4,
        .i4BinFacY = 4,
        .i4PDRepetition = 0,
        .i4PDOrder = {1},
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
            .data_type = 0x30,
            .hsize = 4096,
            .vsize = 768,
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
            .data_type = 0x30,
            .hsize = 4096,
            .vsize = 576,
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
            .data_type = 0x30,
            .hsize = 2048,
            .vsize = 288,
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
            .hsize = 2048,
            .vsize = 1152,
            .user_data_desc = VC_STAGGER_NE,
            .fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
        },
    },
    {
        .bus.csi2 = {
            .channel = 3,
            .data_type = 0x30,
            .hsize = 2048,
            .vsize = 288,
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
            .hsize = 2048,
            .vsize = 1152,
            .user_data_desc = VC_STAGGER_NE,
            .fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
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

static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 8192,
            .vsize = 6144,
            .user_data_desc = VC_STAGGER_NE,
            .fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
        },
    },
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus4[] = {
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
            .data_type = 0x30,
            .hsize = 2048,
            .vsize = 768,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
            .fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
        },
    },
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus5[] = {
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
            .data_type = 0x30,
            .hsize = 4096,
            .vsize = 512,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
            .fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
        },
    },
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus6[] = {
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
            .data_type = 0x30,
            .hsize = 2048,
            .vsize = 768,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
            .fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
        },
    },
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus7[] = {
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
            .data_type = 0x30,
            .hsize = 2048,
            .vsize = 576,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
            .fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
        },
    },
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus8[] = {
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
        },
    },
    {
        .bus.csi2 = {
            .channel = 3,
            .data_type = 0x30,
            .hsize = 4096,
            .vsize = 576,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
            .fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
        },
    },
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus10[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2d,
            .hsize = 4096,
            .vsize = 2304,
            .user_data_desc = VC_STAGGER_NE,
            .fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
        },
    },
    {
        .bus.csi2 = {
            .channel = 3,
            .data_type = 0x30,
            .hsize = 4096,
            .vsize = 576,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW12,
            .fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
        },
    },
};

static struct subdrv_mode_struct mode_struct[] = {
    {//M0 G02_RSS_4096x3072_modeh_1536Msps_pd_type2_30.085470fps_cphy_3trio_10bit_valid_disc_20251029.ini
        .frame_desc = frame_desc_prev_cap,
        .num_entries = ARRAY_SIZE(frame_desc_prev_cap),
        .mode_setting_table = kluxmmain_preview_capture_setting,
        .mode_setting_len = ARRAY_SIZE(kluxmmain_preview_capture_setting),
        .seamless_switch_group = 1,  // Preview
        .seamless_switch_mode_setting_table = PARAM_UNDEFINED,
        .seamless_switch_mode_setting_len = PARAM_UNDEFINED,
        .hdr_mode = HDR_NONE,
        .raw_cnt = 1,
        .exp_cnt = 1,
        .pclk = 132000000,
        .linelength = 1125,
        .framelength = 3900,
        .max_framerate = 300,
        .mipi_pixel_rate = 1116288000,
        .readout_length = 0,
        .read_margin = 10,
        .framelength_step = 4,
        .coarse_integ_step = 1,
        .min_exposure_line = 5,
        .exposure_margin = 0x20,
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
        .imgsensor_pd_info = &imgsensor_pd_info,
        .ae_binning_ratio = 1,
        .fine_integ_line = 0,
        .delay_frame = 2,
        .csi_param = {
            .cphy_eq_bw = 2,
            .cphy_ctle = 4,
            .cdr_delay = 0x1b,
        },
        .ana_gain_max = BASEGAIN * 64,
        .sensor_setting_info = {
            .sensor_scenario_usage = NORMAL_MASK,
            .equivalent_fps = 30,
            .sensorScenario = SENSOR_SCENARIO_BIN,
        },
    },
    {//M1 G02_RSS_4096x3072_modeh_1536Msps_pd_type2_30.085470fps_cphy_3trio_10bit_valid_disc_20251029_V0.1.ini
        .frame_desc = frame_desc_prev_cap,
        .num_entries = ARRAY_SIZE(frame_desc_prev_cap),
        .mode_setting_table = kluxmmain_preview_capture_setting,
        .mode_setting_len = ARRAY_SIZE(kluxmmain_preview_capture_setting),
        .seamless_switch_group = PARAM_UNDEFINED,
        .seamless_switch_mode_setting_table = PARAM_UNDEFINED,
        .seamless_switch_mode_setting_len = PARAM_UNDEFINED,
        .hdr_mode = HDR_NONE,
        .raw_cnt = 1,
        .exp_cnt = 1,
        .pclk = 132000000,
        .linelength = 1125,
        .framelength = 3900,
        .max_framerate = 300,
        .mipi_pixel_rate = 1116288000,
        .readout_length = 0,
        .read_margin = 0,
        .framelength_step = 4,
        .coarse_integ_step = 1,
        .exposure_margin = 0x20,
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
        .imgsensor_pd_info = &imgsensor_pd_info,
        .ae_binning_ratio = 1,
        .fine_integ_line = 0,
        .delay_frame = 2,
        .csi_param = {
            .cphy_eq_bw = 2,
            .cphy_ctle = 4,
            .cdr_delay = 0x1b,
        },
        .ana_gain_max = BASEGAIN * 64,
        .sensor_setting_info = {
            .sensor_scenario_usage = UNUSE_MASK,
            .equivalent_fps = 30,
            .sensorScenario = SENSOR_SCENARIO_UNDEFINED,
        },
    },
    {//M2 G06_RSS_Crop_4096x2304_modeh_1536Msps_pd_type2_30.085470fps_cphy_3trio_10bit_valid_disc_20251029.ini
        .frame_desc = frame_desc_vid,
        .num_entries = ARRAY_SIZE(frame_desc_vid),
        .mode_setting_table = kluxmmain_normal_video_setting,
        .mode_setting_len = ARRAY_SIZE(kluxmmain_normal_video_setting),
        .seamless_switch_group = 4,    //normvideo izoom_qbc
        .seamless_switch_mode_setting_table = PARAM_UNDEFINED,
        .seamless_switch_mode_setting_len = PARAM_UNDEFINED,
        .hdr_mode = HDR_NONE,
        .raw_cnt = 1,
        .exp_cnt = 1,
        .pclk = 132000000,
        .linelength = 1125,
        .framelength = 3900,
        .max_framerate = 300,
        .mipi_pixel_rate = 1116288000,
        .readout_length = 0,
        .read_margin = 0,
        .framelength_step = 4,
        .coarse_integ_step = 1,
        .exposure_margin = 0x20,
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
        .imgsensor_pd_info = &imgsensor_pd_info,
        .ae_binning_ratio = 1,
        .fine_integ_line = 0,
        .delay_frame = 2,
        .csi_param = {
            .cphy_eq_bw = 2,
            .cphy_ctle = 4,
            .cdr_delay = 0x1b,
        },
        .ana_gain_max = BASEGAIN * 64,
        .sensor_setting_info = {
            .sensor_scenario_usage = NORMAL_MASK,
            .equivalent_fps = 30,
            .sensorScenario = SENSOR_SCENARIO_BIN,
        },
    },
    {//M3 G07_Summing_Crop_4096x2304_6%_1536Msps_pd_type2_60.053381fps_cphy_3trio_10bit_20251029.ini
        .frame_desc = frame_desc_hs,
        .num_entries = ARRAY_SIZE(frame_desc_hs),
        .mode_setting_table = kluxmmain_hs_video_setting,
        .mode_setting_len = ARRAY_SIZE(kluxmmain_hs_video_setting),
        .seamless_switch_group = PARAM_UNDEFINED,
        .seamless_switch_mode_setting_table = PARAM_UNDEFINED,
        .seamless_switch_mode_setting_len = PARAM_UNDEFINED,
        .hdr_mode = HDR_NONE,
        .raw_cnt = 1,
        .exp_cnt = 1,
        .pclk = 108000000,
        .linelength = 587,
        .framelength = 3064,
        .max_framerate = 600,
        .mipi_pixel_rate = 1116288000,
        .readout_length = 0,
        .read_margin = 0,
        .framelength_step = 4,
        .coarse_integ_step = 1,
        .exposure_margin = 0x20,
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
        .imgsensor_pd_info = &imgsensor_partial_pd_info,
        .ae_binning_ratio = 1,
        .fine_integ_line = 0,
        .delay_frame = 2,
        .csi_param = {
            .cphy_eq_bw = 2,
            .cphy_ctle = 4,
            .cdr_delay = 0x1b,
        },
        .ana_gain_max = BASEGAIN * 64,
        .multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
        .multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 15.5,
        .sensor_setting_info = {
            .sensor_scenario_usage = NORMAL_MASK,
            .equivalent_fps = 60,
            .sensorScenario = SENSOR_SCENARIO_BIN,
        },
    },
    {//M4 G08_RSS_V2H2_2048x1152_1536Msps_120.325535fps_pd_type2_cphy_3trio_10bit_valid_disc_20251029.ini
        .frame_desc = frame_desc_slim,
        .num_entries = ARRAY_SIZE(frame_desc_slim),
        .mode_setting_table = kluxmmain_slim_video_setting,
        .mode_setting_len = ARRAY_SIZE(kluxmmain_slim_video_setting),
        .seamless_switch_group = PARAM_UNDEFINED,
        .seamless_switch_mode_setting_table = PARAM_UNDEFINED,
        .seamless_switch_mode_setting_len = PARAM_UNDEFINED,
        .hdr_mode = HDR_NONE,
        .raw_cnt = 1,
        .exp_cnt = 1,
        .pclk = 132000000,
        .linelength = 562,
        .framelength = 1952,
        .max_framerate = 1200,
        .mipi_pixel_rate = 1116288000,
        .readout_length = 0,
        .read_margin = 0,
        .framelength_step = 4,
        .coarse_integ_step = 1,
        .exposure_margin = 0x20,
        .imgsensor_winsize_info = {
            .full_w = 8192,
            .full_h = 6144,
            .x0_offset = 0,
            .y0_offset = 768,
            .w0_size = 8192,
            .h0_size = 4608,
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
        .pdaf_cap = TRUE,
        .imgsensor_pd_info = &imgsensor_pd_info,
        .ae_binning_ratio = 1,
        .fine_integ_line = 0,
        .delay_frame = 2,
        .csi_param = {
            .cphy_eq_bw = 2,
            .cphy_ctle = 4,
            .cdr_delay = 0x1b,
        },
        .ana_gain_max = BASEGAIN * 64,
        .sensor_setting_info = {
            .sensor_scenario_usage = NORMAL_MASK,
            .equivalent_fps = 120,
            .sensorScenario = SENSOR_SCENARIO_BIN,
        },
    },
    {//M5 G09_V2H2_2048x1152_1536Msps_240.590248fps_no_pd_cphy_3trio_10bit_20251029.ini
        .frame_desc = frame_desc_cus1,
        .num_entries = ARRAY_SIZE(frame_desc_cus1),
        .mode_setting_table = kluxmmain_custom1_setting,
        .mode_setting_len = ARRAY_SIZE(kluxmmain_custom1_setting),
        .seamless_switch_group = PARAM_UNDEFINED,
        .seamless_switch_mode_setting_table = PARAM_UNDEFINED,
        .seamless_switch_mode_setting_len = PARAM_UNDEFINED,
        .hdr_mode = HDR_NONE,
        .raw_cnt = 1,
        .exp_cnt = 1,
        .pclk = 108000000,
        .linelength = 336,
        .framelength = 1336,
        .max_framerate = 2400,
        .mipi_pixel_rate = 1116288000,
        .readout_length = 0,
        .read_margin = 0,
        .framelength_step = 2,
        .coarse_integ_step = 2,
        .exposure_margin = 0x20,
        .imgsensor_winsize_info = {
            .full_w = 8192,
            .full_h = 6144,
            .x0_offset = 0,
            .y0_offset = 768,
            .w0_size = 8192,
            .h0_size = 4608,
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
        .pdaf_cap = FALSE,
        .imgsensor_pd_info = PARAM_UNDEFINED,
        .ae_binning_ratio = 1,
        .fine_integ_line = 0,
        .delay_frame = 2,
        .csi_param = {
            .cphy_eq_bw = 2,
            .cphy_ctle = 4,
            .cdr_delay = 0x1b,
        },
        .ana_gain_max = BASEGAIN * 64,
        .sensor_setting_info = {
            .sensor_scenario_usage = NORMAL_MASK,
            .equivalent_fps = 240,
            .sensorScenario = SENSOR_SCENARIO_BIN,
        },
    },
    {//M6 G04_RSS_4096x3072_modeh_1536Msps_pd_type2_24.004364fps_cphy_3trio_10bit_valid_disc_20251029.ini
        .frame_desc = frame_desc_cus2,
        .num_entries = ARRAY_SIZE(frame_desc_cus2),
        .mode_setting_table = kluxmmain_custom2_setting,
        .mode_setting_len = ARRAY_SIZE(kluxmmain_custom2_setting),
        .seamless_switch_group = 3,
        .seamless_switch_mode_setting_table = PARAM_UNDEFINED,
        .seamless_switch_mode_setting_len = PARAM_UNDEFINED,
        .hdr_mode = HDR_NONE,
        .raw_cnt = 1,
        .exp_cnt = 1,
        .pclk = 132000000,
        .linelength = 1125,
        .framelength = 4888,
        .max_framerate = 240,
        .mipi_pixel_rate = 1116288000,
        .readout_length = 0,
        .read_margin = 0,
        .framelength_step = 4,
        .coarse_integ_step = 1,
        .exposure_margin = 0x20,
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
        .imgsensor_pd_info = &imgsensor_pd_info,
        .ae_binning_ratio = 1,
        .fine_integ_line = 0,
        .delay_frame = 2,
        .csi_param = {
            .cphy_eq_bw = 2,
            .cphy_ctle = 4,
            .cdr_delay = 0x1b,
        },
        .ana_gain_max = BASEGAIN * 64,
        .sensor_setting_info = {
            .sensor_scenario_usage = NORMAL_MASK,
            .equivalent_fps = 24,
            .sensorScenario = SENSOR_SCENARIO_BIN,
        },
    },
    {//M7 G01_FullSize_8192x6144_15.009005fps_no_pd_1536Msps_bit10_CIP_off_QSC_on_20251029
        .frame_desc = frame_desc_cus3,
        .num_entries = ARRAY_SIZE(frame_desc_cus3),
        .mode_setting_table = kluxmmain_custom3_setting,
        .mode_setting_len = ARRAY_SIZE(kluxmmain_custom3_setting),
        .seamless_switch_group = 1,  // preview, qbc
        .seamless_switch_mode_setting_table = PARAM_UNDEFINED,
        .seamless_switch_mode_setting_len = PARAM_UNDEFINED,
        .hdr_mode = HDR_NONE,
        .raw_cnt = 1,
        .exp_cnt = 1,
        .pclk = 108000000,
        .linelength = 1140,
        .framelength = 6312,
        .max_framerate = 150,
        .mipi_pixel_rate = 1116288000,
        .readout_length = 0,
        .read_margin = 0,
        .framelength_step = 4,
        .coarse_integ_step = 1,
        .exposure_margin = 0x20,
        .imgsensor_winsize_info = {
            .full_w = 8192,
            .full_h = 6144,
            .x0_offset = 0,
            .y0_offset = 0,
            .w0_size = 8192,
            .h0_size = 6144,
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
        .pdaf_cap = FALSE,
        .imgsensor_pd_info = PARAM_UNDEFINED,
        .ae_binning_ratio = 1,
        .fine_integ_line = 0,
        .delay_frame = 2,
        .csi_param = {
            .cphy_eq_bw = 2,
            .cphy_ctle = 4,
            .cdr_delay = 0x1b,
        },
        .ana_gain_max = BASEGAIN * 16,
        .sensor_output_dataformat_cell_type = SENSOR_OUTPUT_FORMAT_CELL_2X2, /* tetra */
        .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_B,
        .sensor_setting_info = {
            .sensor_scenario_usage = RMSC_MASK,
            .equivalent_fps = 15,
            .sensorScenario = SENSOR_SCENARIO_FULL_4CELL,
        },
    },
    {//M8 G03_FullSize_crop_4096x3072_30.026690fps_pd_type2_1536Msps_bit10_CIP_off_QSC_on_20251029.ini
        .frame_desc = frame_desc_cus4,
        .num_entries = ARRAY_SIZE(frame_desc_cus4),
        .mode_setting_table = kluxmmain_custom4_setting,
        .mode_setting_len = ARRAY_SIZE(kluxmmain_custom4_setting),
        .seamless_switch_group = 1,  // preview, IZOOM
        .hdr_mode = HDR_NONE,
        .raw_cnt = 1,
        .exp_cnt = 1,
        .pclk = 108000000,
        .linelength = 1124,
        .framelength = 3200,
        .max_framerate = 300,
        .mipi_pixel_rate = 1116288000,
        .readout_length = 0,
        .read_margin = 0,
        .framelength_step = 4,
        .coarse_integ_step = 1,
        .exposure_margin = 0x20,
        .imgsensor_winsize_info = {
            .full_w = 8192,
            .full_h = 6144,
            .x0_offset = 2048,
            .y0_offset = 1536,
            .w0_size = 4096,
            .h0_size = 3072,
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
        .imgsensor_pd_info = &imgsensor_pd_Fullsize_info,
        .ae_binning_ratio = 1,
        .fine_integ_line = 0,
        .delay_frame = 2,
        .csi_param = {
            .cphy_eq_bw = 2,
            .cphy_ctle = 4,
            .cdr_delay = 0x1b,
        },
        .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_B,
        .ana_gain_max = BASEGAIN * 16,
        .sensor_setting_info = {
            .sensor_scenario_usage = INSENSORZOOM_MASK,
            .equivalent_fps = 30,
            .sensorScenario = SENSOR_SCENARIO_CROP_4CELL,
        },
    },
    {//M9 G10_RSS_Crop_4096x2048_modeh_1536Msps_pd_type2_30.085470fps_cphy_3trio_10bit_valid_disc_20251029.ini
        .frame_desc = frame_desc_cus5,
        .num_entries = ARRAY_SIZE(frame_desc_cus5),
        .mode_setting_table = kluxmmain_custom5_setting,
        .mode_setting_len = ARRAY_SIZE(kluxmmain_custom5_setting),
        .hdr_mode = HDR_NONE,
        .raw_cnt = 1,
        .exp_cnt = 1,
        .pclk = 132000000,
        .linelength = 1125,
        .framelength = 3900,
        .max_framerate = 300,
        .mipi_pixel_rate = 1116288000,
        .readout_length = 0,
        .read_margin = 0,
        .framelength_step = 4,
        .coarse_integ_step = 1,
        .exposure_margin = 0x20,
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
            .y1_offset = 512,
            .w1_size = 4096,
            .h1_size = 2048,
            .x2_tg_offset = 0,
            .y2_tg_offset = 0,
            .w2_tg_size = 4096,
            .h2_tg_size = 2048,
        },
        .pdaf_cap = TRUE,
        .imgsensor_pd_info = &imgsensor_pd_info,
        .ae_binning_ratio = 1,
        .fine_integ_line = 0,
        .delay_frame = 2,
        .csi_param = {
            .cphy_eq_bw = 2,
            .cphy_ctle = 4,
            .cdr_delay = 0x1b,
        },
        .ana_gain_max = BASEGAIN * 64,
        .sensor_setting_info = {
            .sensor_scenario_usage = NORMAL_MASK,
            .equivalent_fps = 30,
            .sensorScenario = SENSOR_SCENARIO_BIN,
        },
    },
    {//M10 G05_FullSize_crop_4096x3072_24.021352fps_pd_type2_1536Msps_bit10_CIP_off_QSC_on_20251029.ini
        .frame_desc = frame_desc_cus6,
        .num_entries = ARRAY_SIZE(frame_desc_cus6),
        .mode_setting_table = kluxmmain_custom6_setting,
        .mode_setting_len = ARRAY_SIZE(kluxmmain_custom6_setting),
        .seamless_switch_group = 3,  // Portrait izoom
        .hdr_mode = HDR_NONE,
        .raw_cnt = 1,
        .exp_cnt = 1,
        .pclk = 108000000,
        .linelength = 1124,
        .framelength = 4000,
        .max_framerate = 240,
        .mipi_pixel_rate = 1116288000,
        .readout_length = 0,
        .read_margin = 0,
        .framelength_step = 4,
        .coarse_integ_step = 1,
        .exposure_margin = 0x20,
        .imgsensor_winsize_info = {
            .full_w = 8192,
            .full_h = 6144,
            .x0_offset = 2048,
            .y0_offset = 1536,
            .w0_size = 4096,
            .h0_size = 3072,
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
        .imgsensor_pd_info = &imgsensor_pd_Fullsize_info,
        .ae_binning_ratio = 1,
        .fine_integ_line = 0,
        .delay_frame = 2,
        .csi_param = {
            .cphy_eq_bw = 2,
            .cphy_ctle = 4,
            .cdr_delay = 0x1b,
        },
        .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_B,
        .ana_gain_max = BASEGAIN * 16,
        .sensor_setting_info = {
            .sensor_scenario_usage = INSENSORZOOM_MASK,
            .equivalent_fps = 24,
            .sensorScenario = SENSOR_SCENARIO_CROP_4CELL,
        },
    },
    {//M11 G11_FullSize_crop_4096x2304_30.026690fps_pd_type2_1536Msps_bit10_CIP_off_QSC_on_20251029.ini
        .frame_desc = frame_desc_cus7,
        .num_entries = ARRAY_SIZE(frame_desc_cus7),
        .mode_setting_table = kluxmmain_custom7_setting,
        .mode_setting_len = ARRAY_SIZE(kluxmmain_custom7_setting),
        .hdr_mode = HDR_NONE,
        .seamless_switch_group = 4,    //normvideo izoom_qbc
        .raw_cnt = 1,
        .exp_cnt = 1,
        .pclk = 108000000,
        .linelength = 1124,
        .framelength = 3200,
        .max_framerate = 300,
        .mipi_pixel_rate = 1116288000,
        .readout_length = 0,
        .read_margin = 0,
        .framelength_step = 4,
        .coarse_integ_step = 1,
        .exposure_margin = 0x20,
        .imgsensor_winsize_info = {
            .full_w = 8192,
            .full_h = 6144,
            .x0_offset = 2048,
            .y0_offset = 1920,
            .w0_size = 4096,
            .h0_size = 2304,
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
        .imgsensor_pd_info = &imgsensor_pd_Fullsize_info,
        .ae_binning_ratio = 1,
        .fine_integ_line = 0,
        .delay_frame = 2,
        .csi_param = {
            .cphy_eq_bw = 2,
            .cphy_ctle = 4,
            .cdr_delay = 0x1b,
        },
        .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_B,
        .ana_gain_max = BASEGAIN * 16,
        .sensor_setting_info = {
            .sensor_scenario_usage = INSENSORZOOM_MASK,
            .equivalent_fps = 30,
            .sensorScenario = SENSOR_SCENARIO_CROP_4CELL,
        },
    },
    {//M12 G12_Summing_NDOL_4096x3072_6%_1536Msps_pd_type2_30.026690fps_cphy_3trio_10bit_20251029.ini
        .frame_desc = frame_desc_cus8,
        .num_entries = ARRAY_SIZE(frame_desc_cus8),
        .mode_setting_table = kluxmmain_custom8_setting,
        .mode_setting_len = ARRAY_SIZE(kluxmmain_custom8_setting),
        .seamless_switch_group = 1,  // preview, IZOOM ,2dol
        .hdr_mode = HDR_RAW_LBMF,
        .raw_cnt = 2,
        .exp_cnt = 2,
        .pclk = 108000000,
        .linelength = 562,
        .framelength = 6400,
        .max_framerate = 300,
        .mipi_pixel_rate = 1116288000,
        .readout_length = 0,
        .read_margin = 0,
        .framelength_step = 4,
        .coarse_integ_step = 1,
        .exposure_margin = 0x20,
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
        .imgsensor_pd_info = &imgsensor_partial_pd_info,
        .ae_binning_ratio = 1,
        .fine_integ_line = 0,
        .delay_frame = 2,
        .csi_param = {
            .cphy_eq_bw = 2,
            .cphy_ctle = 4,
            .cdr_delay = 0x1b,
        },
        .exposure_order_in_lbmf = IMGSENSOR_LBMF_EXPOSURE_SE_FIRST,
        .mode_type_in_lbmf = IMGSENSOR_LBMF_MODE_MANUAL,
        .multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 64,
        .multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].max = BASEGAIN * 64,
        .sensor_setting_info = {
            .sensor_scenario_usage = HDR_RAW_STAGGER_2EXP_MASK,
            .equivalent_fps = 30,
            .sensorScenario = SENSOR_SCENARIO_LBMF_2EXP_BAYER,
        },
    },
    {//M13  G13_RSS_Crop_PGHDR_4096x2304_WO_DCG_Combine_1536Msps_pdtype2_30.054645fps_cphy_trio3_10bit_valid_disc_20251029.ini
        .frame_desc = frame_desc_cus9,
        .num_entries = ARRAY_SIZE(frame_desc_cus9),
        .mode_setting_table = kluxmmain_custom9_setting,
        .mode_setting_len = ARRAY_SIZE(kluxmmain_custom9_setting),
        .seamless_switch_group = 2,    //normvideo DCG
        .hdr_mode = HDR_RAW_DCG_RAW,
        .raw_cnt = 2,
        .exp_cnt = 2,
        .pclk = 132000000,
        .linelength = 900,
        .framelength = 4880,
        .max_framerate = 300,
        .mipi_pixel_rate = 1116288000,
        .readout_length = 0,
        .read_margin = 0,
        .framelength_step = 4,
        .coarse_integ_step = 2,
        .min_exposure_line = 8,
        .exposure_margin = 0x20,
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
        .imgsensor_pd_info = &imgsensor_pd_info,
        .saturation_info = &imgsensor_saturation_info,
        .ae_binning_ratio = 1,
        .fine_integ_line = 0,
        .delay_frame = 2,
        .csi_param = {
            .cphy_eq_bw = 2,
            .cphy_ctle = 4,
            .cdr_delay = 0x1b,
        },
        .ana_gain_max = BASEGAIN * 16,
        .dcg_info = {
            .dcg_mode = IMGSENSOR_DCG_RAW,
            .dcg_gain_mode = IMGSENSOR_DCG_RATIO_MODE,
            .dcg_gain_base = IMGSENSOR_DCG_GAIN_LCG_BASE,
            .dcg_gain_ratio_min = 4000,
            .dcg_gain_ratio_max = 4000,
            .dcg_gain_ratio_step = 0,
            .dcg_gain_table = kluxmmain_dcg_ratio_table_10bit,
            .dcg_gain_table_size = sizeof(kluxmmain_dcg_ratio_table_10bit),
        },
        .hw_mode = 2,
        .sensor_setting_info = {
            .sensor_scenario_usage = DCG_MASK,
            .equivalent_fps = 30,
            .sensorScenario = SENSOR_SCENARIO_DCG_AP_MERGE_2EXP_BAYER,
        },
        .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
        .multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 4,
        .multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].min = BASEGAIN * 1,
        .multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 64,
        .multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].max = BASEGAIN * 16,
    },
    {//M14 G14_RSS_Crop_PGHDR_4096x2304_WI_DCG_Combine_1536Msps_pdtype2_30.054645fps_cphy_trio3_12bit_valid_disc_20251029.ini
        .frame_desc = frame_desc_cus10,
        .num_entries = ARRAY_SIZE(frame_desc_cus10),
        .mode_setting_table = kluxmmain_custom10_setting,
        .mode_setting_len = ARRAY_SIZE(kluxmmain_custom10_setting),
        .seamless_switch_group = PARAM_UNDEFINED,
        .hdr_mode = HDR_RAW_DCG_COMPOSE,
        .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW12_R,
        .raw_cnt = 1,
        .exp_cnt = 2,
        .pclk = 132000000,
        .linelength = 900,
        .framelength = 4880,
        .max_framerate = 300,
        .mipi_pixel_rate = 1116288000,
        .readout_length = 0,
        .read_margin = 0,
        .framelength_step = 4,
        .coarse_integ_step = 2,
        .min_exposure_line = 8,
        .exposure_margin = 0x20,
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
        .imgsensor_pd_info = &imgsensor_pd_info,
        .ae_binning_ratio = 1,
        .fine_integ_line = 0,
        .delay_frame = 2,
        .csi_param = {
            .cphy_eq_bw = 2,
            .cphy_ctle = 4,
            .cdr_delay = 0x1b,
        },
        .ana_gain_max = BASEGAIN * 64,
        .sensor_setting_info = {
            .sensor_scenario_usage = NORMAL_MASK,
            .equivalent_fps = 0,
            .sensorScenario = SENSOR_SCENARIO_DCG_SENSOR_MERGE_12BIT_BAYER,
        },
        .dcg_info = {
            .dcg_mode = IMGSENSOR_DCG_COMPOSE,
            .dcg_gain_mode = IMGSENSOR_DCG_RATIO_MODE,
            .dcg_gain_base = IMGSENSOR_DCG_GAIN_LCG_BASE,
            .dcg_gain_ratio_min = 4000,
            .dcg_gain_ratio_max = 4000,
            .dcg_gain_ratio_step = 0,
            .dcg_gain_table = kluxmmain_dcg_ratio_table_12bit,
            .dcg_gain_table_size = sizeof(kluxmmain_dcg_ratio_table_12bit),
        },
        .multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 4,
        .multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].min = BASEGAIN * 1,
        .multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 64,
        .multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].max = BASEGAIN * 16,
    },
};

static struct subdrv_static_ctx static_ctx = {
    .sensor_id = KLUXMMAIN_SENSOR_ID,
    .reg_addr_sensor_id = {0x3107, 0x3108},
    .i2c_addr_table = {0x34, 0xFF},
    .i2c_burst_write_support = TRUE,
    .i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
    .eeprom_info = eeprom_info,
    .eeprom_num = ARRAY_SIZE(eeprom_info),
    .resolution = {8192, 6144},
    .mirror = IMAGE_HV_MIRROR,

    .mclk = 24,
    .isp_driving_current = ISP_DRIVING_4MA,
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    .mipi_sensor_type = MIPI_CPHY,
    .mipi_lane_num = SENSOR_MIPI_3_LANE,
    .ob_pedestal = 0x40,

    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
    .ana_gain_def = BASEGAIN * 4,
    .ana_gain_min = BASEGAIN * 1,
    .ana_gain_max = BASEGAIN * 64,
    .ana_gain_type = 4,
    .ana_gain_step = 1,
    .ana_gain_table = kluxmmain_ana_gain_table,
    .ana_gain_table_size = sizeof(kluxmmain_ana_gain_table),
    .tuning_iso_base = 100,
    .exposure_def = 0x3D0,
    .exposure_min = 6,
    .exposure_max = 0xFFFFFF - 32,
    .exposure_step = 1,
    .exposure_margin = 32,

    .frame_length_max = 0xFFFFFF,
    .ae_effective_frame = 2,
    .frame_time_delay_frame = 2,
    .start_exposure_offset = 2632000,

    .pdaf_type = PDAF_SUPPORT_CAMSV_QPD,
    .hdr_type = HDR_SUPPORT_DCG,
    .seamless_switch_support = TRUE,
    .temperature_support = TRUE,

    .g_temp = get_sensor_temperature,
    .g_gain2reg = get_gain2reg,
    .g_cali = kluxmmain_get_sensor_cali,
    .s_gph = set_group_hold,
    .s_cali = kluxmmain_set_sensor_cali,

    .reg_addr_stream = 0x0100,
    .reg_addr_mirror_flip = PARAM_UNDEFINED,
    .reg_addr_exposure = {
            {0x3e00, 0x3e01, 0x3e02},//Long exposure
    },
    .long_exposure_support = TRUE,
    .reg_addr_exposure_lshift = PARAM_UNDEFINED,
    .reg_addr_ana_gain = {
            {0x3e08, 0x3e09},
    },

    .reg_addr_exposure_in_lut = {
            {0x3E00, 0x3E01, 0x3E02}, /* LUT_A_COARSE_INTEG_TIME */
            {0x3E22, 0x3E04, 0x3E05}, /* LUT_B_COARSE_INTEG_TIME */
    },

    .reg_addr_ana_gain_in_lut = {
            {0x3E08, 0x3E09}, /* LUT_A_ANA_GAIN_GLOBAL */
            {0x3E12, 0x3E13}, /* LUT_B_ANA_GAIN_GLOBAL */
    },


    .reg_addr_frame_length = {0x326d, 0x320e, 0x320f},
    .reg_addr_temp_en = 0x4c00,
    .reg_addr_temp_read = 0x4c10,
    .reg_addr_auto_extend = PARAM_UNDEFINED,
    .reg_addr_frame_count = PARAM_UNDEFINED,
    .reg_addr_fast_mode = PARAM_UNDEFINED,

    .init_setting_table = kluxmmain_init_setting,
    .init_setting_len = ARRAY_SIZE(kluxmmain_init_setting),
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
    {HW_ID_MCLK_DRIVING_CURRENT, {4}, 1000},
    {HW_ID_DOVDD, {1800000, 1800000}, 1000},
    {HW_ID_DVDD, {1200000, 1200000}, 0},
    {HW_ID_AVDD, {2800000, 2800000}, 0},
    {HW_ID_AFVDD, {2800000, 2800000}, 1000},
    {HW_ID_MCLK, {24}, 1000},
    {HW_ID_RST, {1}, 8000},
};

struct subdrv_entry kluxmmain_mipi_raw_entry = {
    .name = "kluxmmain_mipi_raw",
    .id = KLUXMMAIN_SENSOR_ID,
    .pw_seq = pw_seq,
    .pw_seq_cnt = ARRAY_SIZE(pw_seq),
    .ops = &ops,
};


/* FUNCTION */

static unsigned int read_kluxmmain_eeprom_info(struct subdrv_ctx *ctx, kal_uint16 meta_id,
    BYTE *data, int size)
{
    kal_uint16 addr;
    int readsize;

    if (meta_id != kluxmmain_eeprom_info[meta_id].meta)
        return -1;

    if (size != kluxmmain_eeprom_info[meta_id].size)
        return -1;

    addr = kluxmmain_eeprom_info[meta_id].start;
    readsize = kluxmmain_eeprom_info[meta_id].size;

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

static void kluxmmain_set_mirror_flip(struct subdrv_ctx *ctx, u8 image_mirror)
{
    kal_uint8 iTemp;

    LOG_INF("image_mirror = %d\n", image_mirror);
    iTemp = subdrv_i2c_rd_u8(ctx, 0x3221);
    LOG_INF("iTemp = %d\n", iTemp);

    switch (image_mirror) {
    case IMAGE_NORMAL:
        iTemp &= ~0x66;
        subdrv_i2c_wr_u8(ctx, 0x3221, iTemp);
        break;
    case IMAGE_H_MIRROR:
        subdrv_i2c_wr_u8(ctx, 0x3221, iTemp | 0x06);
        break;
    case IMAGE_V_MIRROR:
        subdrv_i2c_wr_u8(ctx, 0x3221, iTemp | 0x60);
        iTemp = subdrv_i2c_rd_u8(ctx, 0x3221);
        LOG_INF("IMAGE_V_MIRROR = %d\n", iTemp);
        break;
    case IMAGE_HV_MIRROR:
        subdrv_i2c_wr_u8(ctx, 0x3221, iTemp | 0x66);
        break;
    default:
        LOG_INF("Error image_mirror setting\n");
    }
}

#ifdef FPT_SEAMLESS_SUPPORT

enum {
    SHUTTER_NE_FRM_1 = 0,
    GAIN_NE_FRM_1,
    FRAME_LEN_NE_FRM_1,
    HDR_TYPE_FRM_1,
    SHUTTER_NE_FRM_2,
    GAIN_NE_FRM_2,
    FRAME_LEN_NE_FRM_2,
    HDR_TYPE_FRM_2,
    SHUTTER_SE_FRM_1,
    GAIN_SE_FRM_1,
    SHUTTER_SE_FRM_2,
    GAIN_SE_FRM_2,
    SHUTTER_ME_FRM_1,
    GAIN_ME_FRM_1,
    SHUTTER_ME_FRM_2,
    GAIN_ME_FRM_2,
};

static void kluxmmain_seamless_start(struct subdrv_ctx *ctx)
{
    subdrv_i2c_wr_u8(ctx, 0x32c8, 0x41);
    subdrv_i2c_wr_u8(ctx, 0x32b7, 0x0d);
    subdrv_i2c_wr_u8(ctx, 0x3800, 0x01);
}


static void kluxmmain_seamless_end(struct subdrv_ctx *ctx, kal_uint32 curr_vts, kal_uint32 curr_shutter, kal_uint32 tline_ns)
{
    kal_uint16 st_dummy_line = 0x00;
    kal_uint32 shutter_15ms;


    shutter_15ms = 15000000 / tline_ns;


    if (curr_shutter > shutter_15ms)
        st_dummy_line = curr_vts - curr_shutter - 20;
    else
        st_dummy_line = curr_vts - shutter_15ms * 2 / 3 - 20;

    kluxmmain_set_dummyline(ctx, curr_shutter);

    subdrv_i2c_wr_u8(ctx, 0x3800, 0x11);
    subdrv_i2c_wr_u8(ctx, 0x3800, 0x41);
    DRV_LOGE(ctx, "SEAMLESS_VTS= 0x%x, SHUTTER= 0x%x ,st_dummy_line= 0x%x,tline_ns=%d,shutter_15ms=%d",
            curr_vts, curr_shutter, st_dummy_line, tline_ns, shutter_15ms);
}


static kal_uint32 kluxmmain_get_tline_ns(struct subdrv_ctx *ctx, kal_uint32 curr_hts, kal_uint32 curr_pclk)
{
    kal_uint32 tline_ns;

    tline_ns = curr_hts * 10000 / (curr_pclk / 100000);
    return tline_ns;
}


static int kluxmmain_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    enum SENSOR_SCENARIO_ID_ENUM scenario_id;
    struct mtk_hdr_ae *ae_ctrl = NULL;
    u64 *feature_data = (u64 *)para;
    u32 exp_cnt = 0;
    u32 frame_length_in_lut[IMGSENSOR_STAGGER_EXPOSURE_CNT] = {0};

    if (feature_data == NULL) {
        DRV_LOGE(ctx, "input scenario is null!");
        return ERROR_NONE;
    }
    scenario_id = *feature_data;
    if ((feature_data + 1) != NULL)
        ae_ctrl = (struct mtk_hdr_ae *)((uintptr_t)(*(feature_data + 1)));
    else
        DRV_LOGE(ctx, "no ae_ctrl input");
    exp_cnt = ctx->s_ctx.mode[scenario_id].exp_cnt;
    DRV_LOGE(ctx, "seamless switch to %d current_scenario_id %d\n!", scenario_id, ctx->current_scenario_id);

    check_current_scenario_id_bound(ctx);
    DRV_LOGE(ctx, "E: set seamless switch %u %u\n", ctx->current_scenario_id, scenario_id);
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
    spin_lock(&imgsensor_drv_lock);
    ctx->current_scenario_id = scenario_id;
    ctx->pclk = ctx->s_ctx.mode[scenario_id].pclk;
    ctx->line_length = ctx->s_ctx.mode[scenario_id].linelength;
    ctx->frame_length = ctx->s_ctx.mode[scenario_id].framelength;
    ctx->min_frame_length = ctx->s_ctx.mode[scenario_id].framelength;
    ctx->autoflicker_en = FALSE;
    spin_unlock(&imgsensor_drv_lock);

    DRV_LOGE(ctx, "seamless switch 1-exp!");
    kluxmmain_seamless_start(ctx);

    if (ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table != PARAM_UNDEFINED) {
        kluxmmain_i2c_burst_wr_regs_u8(ctx,
            ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table,
            ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_len);
    } else {
        kluxmmain_i2c_burst_wr_regs_u8(ctx,
            ctx->s_ctx.mode[scenario_id].mode_setting_table,
            ctx->s_ctx.mode[scenario_id].mode_setting_len);
    }
    DRV_LOGE(ctx, "long gain %llu shtter %llu short gain %llu shutter %llu", (u64)ae_ctrl->gain.le_gain, (u64)ae_ctrl->exposure.le_exposure, (u64)ae_ctrl->gain.me_gain, (u64)ae_ctrl->exposure.me_exposure);
    if (ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode == HDR_RAW_LBMF) {
        set_multi_shutter_frame_length_in_lut(ctx, (u64 *)&ae_ctrl->exposure, exp_cnt, 0, frame_length_in_lut);
        set_multi_gain_in_lut(ctx, (u32 *)&ae_ctrl->gain, exp_cnt);
    } else {
        u64 shutter_temp = (u64)ae_ctrl->exposure.le_exposure;
        kluxmmain_set_shutter(ctx, (u8*)&shutter_temp, NULL);
        kluxmmain_set_gain(ctx, (u8*)&ae_ctrl->gain.le_gain, NULL);
    }

    kluxmmain_seamless_end(ctx, ctx->frame_length, ae_ctrl->exposure.le_exposure,
    kluxmmain_get_tline_ns(ctx, ctx->line_length, ctx->pclk));
    DRV_LOGE(ctx, "seamless exp %llu gain %llu  gain data %d shutter data %d short gain %d  shutter %d \n",
        (u64)ae_ctrl->exposure.le_exposure,
        (u64)ae_ctrl->gain.le_gain,
        ((subdrv_i2c_rd_u8(ctx, 0x3e08) << 8) + subdrv_i2c_rd_u8(ctx, 0x3e09)),
        ((subdrv_i2c_rd_u8(ctx, 0x3e00) << 16) + (subdrv_i2c_rd_u8(ctx, 0x3e01) << 8) +subdrv_i2c_rd_u8(ctx, 0x3e02)),
        ((subdrv_i2c_rd_u8(ctx, 0x3e12) << 8) + subdrv_i2c_rd_u8(ctx, 0x3e13)),
        ((subdrv_i2c_rd_u8(ctx, 0x3e22) << 16) + (subdrv_i2c_rd_u8(ctx, 0x3e04) << 8) +subdrv_i2c_rd_u8(ctx, 0x3e05)));

    return 0;
}
#endif


static int kluxmmain_get_eeprom_comdata(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    LOG_INF("+");
    memcpy(para, (u8*)(&oplus_eeprom_info), sizeof(oplus_eeprom_info));
    *len = sizeof(oplus_eeprom_info);
    return 0;
}

static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
{
    kal_uint16 get_byte = 0;

    adaptor_i2c_rd_u8(ctx->i2c_client, KLUXMMAIN_EEPROM_ADDR >> 1, addr, (u8 *)&get_byte);
    return get_byte;
}

static kal_int32 table_write_eeprom_one_packet(struct subdrv_ctx *ctx,
        kal_uint16 addr, kal_uint8 *para, kal_uint32 len)
{
    kal_int32 ret = ERROR_NONE;
    ret = adaptor_i2c_wr_p8(ctx->i2c_client, KLUXMMAIN_EEPROM_ADDR >> 1,
            addr, para, len);

    return ret;
}

static kal_int32 write_eeprom_protect(struct subdrv_ctx *ctx, kal_uint16 enable)
{
    kal_int32 ret = ERROR_NONE;
    kal_uint16 reg = 0xE000;

    if (enable) {
        adaptor_i2c_wr_u8(ctx->i2c_client, KLUXMMAIN_EEPROM_ADDR >> 1, reg, (KLUXMMAIN_EEPROM_ADDR & 0xFE) | 0x01);
    }
    else {
        adaptor_i2c_wr_u8(ctx->i2c_client, KLUXMMAIN_EEPROM_ADDR >> 1, reg, KLUXMMAIN_EEPROM_ADDR & 0xFE);
    }

    return ret;
}

static kal_uint16 get_64align_addr(kal_uint16 data_base) {
    kal_uint16 multiple = 0;
    kal_uint16 surplus = 0;
    kal_uint16 addr_64align = 0;

    multiple = data_base / 64;
    surplus = data_base % 64;
    if (surplus) {
        addr_64align = (multiple + 1) * 64;
    } else {
        addr_64align = multiple * 64;
    }
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

    for (i = 0; i < idx; i++) {
        ret = table_write_eeprom_one_packet(ctx, (data_base + EEPROM_WRITE_DATA_MAX_LENGTH * i),
                &pData[EEPROM_WRITE_DATA_MAX_LENGTH*i], EEPROM_WRITE_DATA_MAX_LENGTH);
        if (ret != ERROR_NONE) {
            LOG_INF("write_eeprom error: i=%d\n", i);
            return -1;
        }
        msleep(6);
    }

    msleep(6);
    if (idy) {
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
    if (part1_length > data_length) {
        part1_length = data_length;
    }
    part2_length = data_length - part1_length;

    write_eeprom_protect(ctx, 0);
    msleep(6);

    if (part1_length) {
        ret = eeprom_table_write(ctx, data_base, pData, part1_length);
        if (ret == -1) {
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

    if (pStereodata != NULL) {
        LOG_INF("SET_SENSOR_OTP: 0x%x %d 0x%x %d\n",
                       pStereodata->uSensorId,
                       pStereodata->uDeviceId,
                       pStereodata->baseAddr,
                       pStereodata->dataLength);

        data_base = pStereodata->baseAddr;
        data_length = pStereodata->dataLength;
        pData = pStereodata->uData;
        if (((pStereodata->uSensorId == KLUXMMAIN_SENSOR_ID))
            && (data_length == CALI_DATA_SLAVE_LENGTH)
            && (data_base == KLUXMMAIN_STEREO_MW_START_ADDR)) {
            LOG_INF("Write: %x %x %x %x\n", pData[0], pData[39], pData[40], pData[1556]);

            eeprom_64align_write(ctx, data_base, pData, data_length);

            LOG_INF("com_0:0x%x\n", read_cmos_eeprom_8(ctx, data_base));
            LOG_INF("com_39:0x%x\n", read_cmos_eeprom_8(ctx, data_base+39));
            LOG_INF("innal_40:0x%x\n", read_cmos_eeprom_8(ctx, data_base+40));
            LOG_INF("innal_1556:0x%x\n", read_cmos_eeprom_8(ctx, data_base+1556));
            LOG_INF("write_Module_data Write end\n");

        } else if (((pStereodata->uSensorId == KLUXMMAIN_SENSOR_ID))
            && (data_length < AESYNC_DATA_LENGTH_TOTAL)
            && (data_base == KLUXMMAIN_AESYNC_START_ADDR)) {
            LOG_INF("write main aesync: %x %x %x %x %x %x %x %x\n", pData[0], pData[1],
                pData[2], pData[3], pData[4], pData[5], pData[6], pData[7]);

            eeprom_64align_write(ctx, data_base, pData, data_length);

            LOG_INF("readback main aesync: %x %x %x %x %x %x %x %x\n",
                    read_cmos_eeprom_8(ctx, KLUXMMAIN_AESYNC_START_ADDR),
                    read_cmos_eeprom_8(ctx, KLUXMMAIN_AESYNC_START_ADDR+1),
                    read_cmos_eeprom_8(ctx, KLUXMMAIN_AESYNC_START_ADDR+2),
                    read_cmos_eeprom_8(ctx, KLUXMMAIN_AESYNC_START_ADDR+3),
                    read_cmos_eeprom_8(ctx, KLUXMMAIN_AESYNC_START_ADDR+4),
                    read_cmos_eeprom_8(ctx, KLUXMMAIN_AESYNC_START_ADDR+5),
                    read_cmos_eeprom_8(ctx, KLUXMMAIN_AESYNC_START_ADDR+6),
                    read_cmos_eeprom_8(ctx, KLUXMMAIN_AESYNC_START_ADDR+7));
            LOG_INF("AESync write_Module_data Write end\n");
        } else {
            LOG_INF("Invalid Sensor id:0x%x write eeprom\n", pStereodata->uSensorId);
            return -1;
        }
    } else {
        LOG_INF("kluxmmain write_Module_data pStereodata is null\n");
        return -1;
    }
    return ret;
}

static int kluxmmain_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    int ret = ERROR_NONE;
    ret = write_Module_data(ctx, (ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(para));
    if (ret != ERROR_NONE) {
        LOG_INF("ret=%d\n", ret);
    }
    return 0;
}

static int kluxmmain_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    UINT16 *feature_data_16 = (UINT16 *) para;
    UINT32 *feature_return_para_32 = (UINT32 *) para;
    if (*len > CALI_DATA_SLAVE_LENGTH)
        *len = CALI_DATA_SLAVE_LENGTH;
    LOG_INF("feature_data mode:%d  lens:%d", *feature_data_16, *len);
    read_kluxmmain_eeprom_info(ctx, EEPROM_META_STEREO_MW_MAIN_DATA,
            (BYTE *)feature_return_para_32, *len);
    return 0;
}

static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size)
{
    if (adaptor_i2c_rd_p8(ctx->i2c_client, KLUXMMAIN_EEPROM_ADDR >> 1,
            addr, data, size) < 0) {
        return false;
    }
    return true;
}

static void read_otp_info(struct subdrv_ctx *ctx)
{
    DRV_LOGE(ctx, "kluxmmain read_otp_info begin\n");
    read_cmos_eeprom_p8(ctx, 0, otp_data_checksum, sizeof(otp_data_checksum));
    DRV_LOGE(ctx, "kluxmmain read_otp_info end\n");
}

static int kluxmmain_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    u32 *feature_return_para_32 = (u32 *)para;
    u32 length = sizeof(otp_data_checksum);

    if (*len < sizeof(otp_data_checksum)) {
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

static int kluxmmain_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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
    LOG_INF("rst delay = %d, func: %s, line: %d\n", pw_seq[1].delay, __FUNCTION__, __LINE__);
    while (ctx->s_ctx.i2c_addr_table[i] != 0xFF) {
        ctx->i2c_write_id = ctx->s_ctx.i2c_addr_table[i];
        do {
            *sensor_id = (subdrv_i2c_rd_u8(ctx, addr_h) << 8) |
                subdrv_i2c_rd_u8(ctx, addr_l);
            LOG_INF("i2c_write_id(0x%x) sensor_id(0x%x/0x%x)\n",
                ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
            if (*sensor_id == KLUXMMAIN_IMGSENSOR_ID) {
                *sensor_id = ctx->s_ctx.sensor_id;
                LOG_INF("[get_imgsensor_id] Read sensor id Success, write id:0x%x, id: 0x%x\n", ctx->i2c_write_id, *sensor_id);
                if (first_read) {
                    read_eeprom_common_data(ctx, &oplus_eeprom_info, oplus_eeprom_addr_table);
                    first_read = KAL_FALSE;
                    msg_buf = kmalloc(MAX_BURST_LEN, GFP_KERNEL);
                    if (!msg_buf) {
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

static void kluxmmain_feedback_awbgain(struct subdrv_ctx *ctx, kal_uint32 r_gain, kal_uint32 b_gain)
{
    UINT32 r_gain_int = 0;
    UINT32 b_gain_int = 0;
    UINT32 color_ratio = 90;
    UINT32 gain_base = 0x100;

    DRV_LOG(ctx, "feedback_awbgain: r_gain = %x b_gain = %x", r_gain, b_gain);
    DRV_LOG(ctx, "feedback_awbgain: r_gain = %d b_gain = %d", r_gain, b_gain);

    r_gain_int = r_gain * gain_base / 512;
    b_gain_int = b_gain * gain_base / 512;

    DRV_LOG(ctx, "r_gain = 0x%x b_gain = 0x%x", r_gain_int, b_gain_int);
    DRV_LOG(ctx, "r_gain = 0d%d b_gain = 0d%d", r_gain_int, b_gain_int);

    //color_ratio = 90;

    DRV_LOG(ctx, "color_ratio = 0x%x ", color_ratio);

    r_gain_int = r_gain_int * color_ratio / 100;
    b_gain_int = b_gain_int * color_ratio / 100;

    r_gain_int = (r_gain_int < gain_base) ? gain_base : r_gain_int;
    b_gain_int = (b_gain_int < gain_base) ? gain_base : b_gain_int;

    DRV_LOG(ctx, "r_gain_final = 0x%x b_gain_final = 0x%x", r_gain_int, b_gain_int);
    DRV_LOG(ctx, "r_gain_final = 0d%d b_gain_final = 0d%d, color_ratio = 0d%d",
            r_gain_int, b_gain_int, color_ratio);

    subdrv_i2c_wr_u8(ctx, 0x5415, (r_gain_int >> 8) & 0xff);        // R_H
    subdrv_i2c_wr_u8(ctx, 0x5416, r_gain_int & 0xff);               // R_L
    subdrv_i2c_wr_u8(ctx, 0x5417, (gain_base >> 8) & 0xff);         // GB_H
    subdrv_i2c_wr_u8(ctx, 0x5418, gain_base & 0xff);                // GB_L
    subdrv_i2c_wr_u8(ctx, 0x569c, (gain_base >> 8) & 0xff);         // GR_H
    subdrv_i2c_wr_u8(ctx, 0x569d, gain_base & 0xff);                // GR_L
    subdrv_i2c_wr_u8(ctx, 0x5413, (b_gain_int >> 8) & 0xff);        // B_H
    subdrv_i2c_wr_u8(ctx, 0x5414, b_gain_int & 0xff);               // B_L

    DRV_LOG(ctx, "r_gain_final0x5416 = 0x%x b_gain_final0x5414 = 0x%x",
            subdrv_i2c_rd_u8(ctx, 0x5416), subdrv_i2c_rd_u8(ctx, 0x5414));
    DRV_LOG(ctx, "gb_gain_final0x5418 = 0x%x gr_gain_final0x569d = 0x%x",
            subdrv_i2c_rd_u8(ctx, 0x5418), subdrv_i2c_rd_u8(ctx, 0x569d));
}

static int kluxmmain_set_awb_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    u32 *feature_data_32 = (u32 *)para;
    u32 r_gain = feature_data_32[1];
    u32 b_gain = feature_data_32[2];

    DRV_LOG(ctx, "SENSOR_FEATURE_SET_AWB_GAIN r_gain=%d, b_gain=%d", r_gain, b_gain);

        kluxmmain_feedback_awbgain(ctx, r_gain, b_gain);
        return 0;
}

static u16 get_gain2reg(u32 gain)
{
    u16 reg_gain = 0x0;

    reg_gain = gain >> 3;

    LOG_INF("gain = %d, reg_gain = 0x%x\n", gain, reg_gain);

    return reg_gain;
}

static void kluxmmain_set_dummyline(struct subdrv_ctx *ctx, u64 curr_shutter)
{
    kal_uint16 dummyline = 0;
    kal_uint32 shutter_15ms = 0;
    kal_uint32 curr_vts = ctx->frame_length;
    shutter_15ms = 15000000 / kluxmmain_get_tline_ns(ctx, ctx->line_length, ctx->pclk);
    if(curr_shutter > curr_vts - 16) {
        if (curr_shutter > KAL_UINT32_MAX - 16) {
            curr_vts = KAL_UINT32_MAX;
        } else {
            curr_vts = curr_shutter + 16;
        }
    }
    if (curr_shutter > shutter_15ms) {
        if (curr_vts >= curr_shutter + 12) {
            dummyline = curr_vts - curr_shutter - 12;
        } else {
            dummyline = 0;
        }
    } else {
        if (curr_vts >= shutter_15ms + 12) {
            dummyline = curr_vts - shutter_15ms  -12;
        } else {
            dummyline = 0;
        }
    }
    LOG_INF("tline %d curr_shutter %llu curr_vts %d dummyline %d", kluxmmain_get_tline_ns(ctx, ctx->line_length, ctx->pclk), curr_shutter, curr_vts, dummyline);
    subdrv_i2c_wr_u8(ctx, 0x3230, (dummyline >> 8) & 0xff);
    subdrv_i2c_wr_u8(ctx, 0x3231, dummyline & 0xff);
}

static int kluxmmain_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    u32 gain = *((u32 *)para);
    u16 reg_gain;
    u32 max_gain = ctx->s_ctx.mode[ctx->current_scenario_id].ana_gain_max ?
        ctx->s_ctx.mode[ctx->current_scenario_id].ana_gain_max : ctx->ana_gain_max;
    u16 min_gain = ctx->s_ctx.mode[ctx->current_scenario_id].ana_gain_max ?
        ctx->s_ctx.mode[ctx->current_scenario_id].ana_gain_min : ctx->ana_gain_min;

    DRV_LOGE(ctx, "set_gain input: %d, range: %d-%d\n", gain, min_gain, max_gain);

    if (gain < min_gain || gain > max_gain) {
        LOG_INF("Error max gain setting: %d\n", max_gain);

        if (gain < min_gain)
            gain = min_gain;
        else if (gain > max_gain)
            gain = max_gain;
    }

    reg_gain = get_gain2reg(gain);

    spin_lock(&imgsensor_drv_lock);
    ctx->ana_gain[0] = reg_gain;
    spin_unlock(&imgsensor_drv_lock);

    DRV_LOGE(ctx, "gain = %d, reg_gain = 0x%x, max_gain:%d, min_gain:%d\n",
            gain, reg_gain, max_gain, min_gain);
    /* DCG MODE SE Gain*/
    DRV_LOGE(ctx, "current_scenario_id %d\n", ctx->current_scenario_id);
    if (ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM1) {
        subdrv_i2c_wr_u8(ctx, 0x3800, 0x00);
    }
    if (ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode == HDR_RAW_DCG_COMPOSE || ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode == HDR_RAW_DCG_RAW) {
        /* LOW gain */
        subdrv_i2c_wr_u8(ctx, 0x3e82, (reg_gain >> 8) & 0xff);
        subdrv_i2c_wr_u8(ctx, 0x3e83, reg_gain & 0xff);
        /* HIGH gain */
        reg_gain = reg_gain * 4;
        subdrv_i2c_wr_u8(ctx, 0x3e08, (reg_gain >> 8) & 0xff);
        subdrv_i2c_wr_u8(ctx, 0x3e09, reg_gain & 0xff);
    } else {
        subdrv_i2c_wr_u8(ctx, 0x3e08, (reg_gain >> 8) & 0xff);
        subdrv_i2c_wr_u8(ctx, 0x3e09, reg_gain & 0xff);
    }
    if (ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM1) {
        subdrv_i2c_wr_u8(ctx, 0x3800, 0x10);
        subdrv_i2c_wr_u8(ctx, 0x3800, 0x60);
    }

    DRV_LOGE(ctx, "0x3e08 data(%d) 0x3e09 data(%d)\n", subdrv_i2c_rd_u8(ctx, 0x3e08), subdrv_i2c_rd_u8(ctx, 0x3e09));

    return 0;
}

static void streaming_ctrl(struct subdrv_ctx *ctx, bool enable)
{
    DRV_LOG(ctx, "E! enable:%u\n", enable);
    check_current_scenario_id_bound(ctx);
    if (ctx->s_ctx.mode[ctx->current_scenario_id].aov_mode) {
        DRV_LOG(ctx, "AOV mode set stream in SCP side! (sid:%u)\n",
            ctx->current_scenario_id);
        return;
    }

    if (enable) {
        if (ctx->s_ctx.chk_s_off_sta) {
            DRV_LOG(ctx, "check_stream_off before stream on");
            check_stream_off(ctx);
        }
        subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x01);
        mdelay(3);
    } else {
        check_stream_on(ctx);  // check_stream_on before stream off
        subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x00);
        if (ctx->s_ctx.reg_addr_fast_mode && ctx->fast_mode_on) {
            ctx->fast_mode_on = FALSE;
            ctx->ref_sof_cnt = 0;
            DRV_LOG(ctx, "seamless_switch disabled.");
            set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_fast_mode, 0x00);
            commit_i2c_buffer(ctx);
        }
    }
    ctx->is_streaming = enable;
    DRV_LOG(ctx, "X! enable:%u\n", enable);
}

static int kluxmmain_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
        DRV_LOG(ctx, "SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%u\n", *(u32 *)para);
        if (*(u32 *)para)
            kluxmmain_set_shutter(ctx, para, len);
        streaming_ctrl(ctx, true);
        return 0;
}

static int kluxmmain_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
        DRV_LOG(ctx, "streaming control para:%d\n", *para);
        streaming_ctrl(ctx, false);
        return 0;
}

static int open(struct subdrv_ctx *ctx)
{
    u32 sensor_id = 0;
    u32 scenario_id = 0;
    /* get sensor id */
    if (get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
        return ERROR_SENSOR_CONNECT_FAIL;

    LOG_INF("%s", SENSOR_NAME);

    kluxmmain_i2c_burst_wr_regs_u8(ctx, ctx->s_ctx.init_setting_table, ctx->s_ctx.init_setting_len);

    /* PDC setting */
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

static void kluxmmain_get_sensor_cali(void* arg)
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
        write_id = ctx->s_ctx.eeprom_info[idx].i2c_write_id;
        adaptor_i2c_rd_u8(ctx->i2c_client, write_id >> 1, OTP_PDC_VALID_ADDR, (u8 *)&pdc_is_valid);

        if (pdc_is_valid != OTP_PDC_IS_VALID_VAL) {
            DRV_LOGE(ctx, "pdc is invalid %d", pdc_is_valid);
            return;
        }

        if (info[idx].preload_pdc_table == NULL) {
            info[idx].preload_pdc_table = kmalloc(size, GFP_KERNEL);
            if (buf == NULL)
                i2c_multi_read_eeprom(ctx, addr, size, info[idx].preload_pdc_table);
            else
                memcpy(info[idx].preload_pdc_table, buf, size);
            DRV_LOGE(ctx, "preload pdc data %u bytes", size);
        } else {
            DRV_LOGE(ctx, "pdc data is already preloaded %u bytes", size);
        }
    }

    ctx->is_read_preload_eeprom = 1;
}

static void kluxmmain_set_sensor_cali(void *arg)
{
    struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
    u16 idx = 0;
    u8 support = FALSE;
    u8 *pbuf = NULL;
    u16 size = 0;
    u16 addr = 0;
    u16 i = 0;
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
            LOG_INF("stream %d\n", subdrv_i2c_rd_u8(ctx, ctx->s_ctx.reg_addr_stream));

            DRV_LOG(ctx, "start write crosstalk_data!\n");

            /* APSC data */
            LOG_INF("APSC reg(0x5003) value 0x%x  +\n", subdrv_i2c_rd_u8(ctx, 0x5003));
            subdrv_i2c_wr_seq_p8(ctx, addr, pbuf, XTC_SENSOR_LENGTH_PART1);
            for (i = 0; i < 10; i++) {
                value = subdrv_i2c_rd_u8(ctx, addr + i);
                DRV_LOG(ctx, "APSC addr:0x%x, read:0x%x\n", (addr + i), value);
            }

            /* APSC data enable */
            subdrv_i2c_wr_u8(ctx, 0x5003, (subdrv_i2c_rd_u8(ctx, 0x5003) | 8));
            LOG_INF("APSC reg(0x5003) value 0x%x  —\n", subdrv_i2c_rd_u8(ctx, 0x5003));
        }

    } else {
        DRV_LOG(ctx, "not need set pdc\n");
    }
}



static int get_sensor_temperature(void *arg)
{
    struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
    u8 temp_reg_value = 0;
    u16 temperature_convert = 0.0;
    //fixed_s32_t  temp_decimal = 0.0;
    u8 reg_4c10_h = 0;
    u8 reg_4c11_l = 0;
    u16 temperature_integer  = 0;
    u8 temp_decimal_bits = 0;

    temp_reg_value = subdrv_i2c_rd_u8(ctx, ctx->s_ctx.reg_addr_temp_en);

    if (temp_reg_value == 0x2a) {
        DRV_LOG(ctx, "Current temperature enable: 0x%02X", temp_reg_value);
        reg_4c10_h = subdrv_i2c_rd_u8(ctx, ctx->s_ctx.reg_addr_temp_read);
        reg_4c11_l = subdrv_i2c_rd_u8(ctx, (ctx->s_ctx.reg_addr_temp_read + 1));
        temperature_integer = (reg_4c10_h << 1) | ((reg_4c11_l & 0x04) >> 2);
        temp_decimal_bits = reg_4c11_l & 0x03;
        temperature_convert = temperature_integer - 273;
        DRV_LOG(ctx, " temperature: %u℃\n",  temperature_convert);
    }
    else {
        DRV_LOG(ctx, "The temperature sensor is not enabled! \n");
    }

    return temperature_convert;
}

static void set_group_hold(void *arg, u8 en)
{
    struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
    if (en) {
        set_i2c_buffer(ctx, 0x301f, 0x01);
    } else {
        set_i2c_buffer(ctx, 0x301f, 0x00);
    }
}

void kluxmmain_get_min_shutter_by_scenario(struct subdrv_ctx *ctx,
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
        case HDR_RAW_DCG_RAW:
        case HDR_RAW_LBMF:
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

int kluxmmain_get_min_shutter_by_scenario_adapter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    u64 *feature_data = (u64 *) para;
    kluxmmain_get_min_shutter_by_scenario(ctx,
        (enum SENSOR_SCENARIO_ID_ENUM)*(feature_data),
        feature_data + 1, feature_data + 2);
    return 0;
}

static int kluxmmain_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    u32 modes = *((u32 *)para);

    LOG_INF("enable: %d\n", modes);

    if (modes == 2) {
        subdrv_i2c_wr_u8(ctx, 0x206B, 0xa0);
        subdrv_i2c_wr_u8(ctx, 0x206C, 0xff);
        subdrv_i2c_wr_u8(ctx, 0x206d, 0xff);
        subdrv_i2c_wr_u8(ctx, 0x2080, 0x17);
    } else if (modes == 5) {
        subdrv_i2c_wr_u8(ctx, 0x206B, 0xb7);
        subdrv_i2c_wr_u8(ctx, 0x206C, 0xff);
        subdrv_i2c_wr_u8(ctx, 0x206d, 0xff);
        subdrv_i2c_wr_u8(ctx, 0x2080, 0x17);
    } else {
        subdrv_i2c_wr_u8(ctx, 0x206B, 0x00);
        subdrv_i2c_wr_u8(ctx, 0x206C, 0x00);
        subdrv_i2c_wr_u8(ctx, 0x206d, 0x00);
        subdrv_i2c_wr_u8(ctx, 0x2080, 0x07);
        LOG_INF("unsupported mode(%u)\n", modes);
    }

    spin_lock(&imgsensor_drv_lock);
    ctx->test_pattern = modes;
    LOG_INF("final set_test_pattern modes: %d\n", modes);
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}

static int init_ctx(struct subdrv_ctx *ctx,    struct i2c_client *i2c_client, u8 i2c_write_id)
{
    memcpy(&(ctx->s_ctx), &static_ctx, sizeof(struct subdrv_static_ctx));
    subdrv_ctx_init(ctx);
    ctx->i2c_client = i2c_client;
    ctx->i2c_write_id = i2c_write_id;
    kluxmmain_set_mirror_flip(ctx, 2);
    return 0;
}

static int vsync_notify(struct subdrv_ctx *ctx,    unsigned int sof_cnt)
{
    DRV_LOG(ctx, "sof_cnt(%u) ctx->ref_sof_cnt(%u) ctx->fast_mode_on(%d)",
        sof_cnt, ctx->ref_sof_cnt, ctx->fast_mode_on);
    ctx->sof_cnt = sof_cnt;

    return 0;
}

static void kluxmmain_set_multi_shutter_frame_length(struct subdrv_ctx *ctx, u64 *shutters, u16 exp_cnt, u16 frame_length)
{
    if (exp_cnt == 1) {
        kluxmmain_set_shutter_frame_length_convert(ctx, shutters[0], frame_length);
        return;
    }
}

static int kluxmmain_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    u64* feature_data = (u64*)para;

    kluxmmain_set_multi_shutter_frame_length(ctx, (u64 *)(*feature_data),
        (u64) (*(feature_data + 1)), (u64) (*(feature_data + 2)));
    return 0;
}

static void kluxmmain_set_hdr_tri_shutter(struct subdrv_ctx *ctx, u64 *shutters, u16 exp_cnt)
{
    int i = 0;
    u64 values[3] = {0};

    if (shutters != NULL) {
        for (i = 0; i < 3; i++)
            values[i] = (u64) *(shutters + i);
    }
    kluxmmain_set_multi_shutter_frame_length(ctx, values, exp_cnt, 0);
}

/*static int kluxmmain_set_hdr_tri_shutter2(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    u64* feature_data = (u64*)para;

    kluxmmain_set_hdr_tri_shutter(ctx, feature_data, 2);
    return 0;
}*/

static int kluxmmain_set_hdr_tri_shutter3(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    u64* feature_data = (u64*)para;

    kluxmmain_set_hdr_tri_shutter(ctx, feature_data, 3);
    return 0;
}
/*
static void kluxmmain_set_max_framerate(struct subdrv_ctx *ctx, kal_uint16 framerate, kal_bool min_framelength_en)
{
    kal_uint32 frame_length = ctx->frame_length;

    frame_length = ctx->pclk / framerate * 10 / ctx->line_length;
    spin_lock(&imgsensor_drv_lock);
    ctx->frame_length = (frame_length > ctx->min_frame_length) ?
        frame_length : ctx->min_frame_length;
    ctx->dummy_line = ctx->frame_length - ctx->min_frame_length;

    if (ctx->frame_length > ctx->s_ctx.frame_length_max) {
        ctx->frame_length = ctx->s_ctx.frame_length_max;
        ctx->dummy_line = ctx->frame_length - ctx->min_frame_length;
    }
    if (min_framelength_en)
        ctx->min_frame_length = ctx->frame_length;
    spin_unlock(&imgsensor_drv_lock);
    LOG_INF("frame length = %d\n", ctx->frame_length);
}
*/
static void kluxmmain_set_shutter_frame_length_convert(struct subdrv_ctx *ctx, u64 shutter, u32 frame_length)
{
    unsigned long flags;
    kal_uint16 realtime_fps = 0;
    kal_int32 dummy_line = 0;
    //bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    ctx->shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
    LOG_INF(
        "set_shutter_frame_length+ shutter =%llu, framelength =%u\n",
        shutter, frame_length);

    if (ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM1) {
        // shutter needs to be even (multiple of 2)
        if (shutter % 2 != 0) {
            shutter = (shutter / 2) * 2;
        }
        DRV_LOG(ctx, "set even shutter_frame_length shutter =%llu", shutter);
    }

    spin_lock(&imgsensor_drv_lock);
    if (frame_length > 1)
        dummy_line = frame_length - ctx->frame_length;
    ctx->frame_length = ctx->frame_length + dummy_line;
    check_current_scenario_id_bound(ctx);
    if (shutter > ctx->frame_length - ctx->s_ctx.exposure_margin)
        ctx->frame_length = shutter + ctx->s_ctx.exposure_margin;
    if (ctx->frame_length > ctx->s_ctx.frame_length_max)
        ctx->frame_length = ctx->s_ctx.frame_length_max;
    spin_unlock(&imgsensor_drv_lock);
    shutter =
    (shutter < ctx->s_ctx.exposure_min) ? ctx->s_ctx.exposure_min : shutter;
    shutter =
    (shutter > (ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin))
    ? (ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin) : shutter;
    /* restore shutter */
    memset(ctx->exposure, 0, sizeof(ctx->exposure));
    ctx->exposure[0] = (u32) shutter;
    /* group hold start */
    //if (gph)
    //	ctx->s_ctx.s_gph((void *)ctx, 1);
    /* enable auto extend */
    if (ctx->s_ctx.reg_addr_auto_extend)
        set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_auto_extend, 0x01);

    if (set_auto_flicker(ctx, 0) || frame_length || !ctx->s_ctx.reg_addr_auto_extend) {
        ctx->frame_length = (ctx->frame_length + 7) & ~0x07;
        if (ctx->current_scenario_id != SENSOR_SCENARIO_ID_CUSTOM1) {
            subdrv_i2c_wr_u8(ctx, 0x326d, (ctx->frame_length >> 16) & 0x7f);
            subdrv_i2c_wr_u8(ctx, 0x320e, (ctx->frame_length >> 8) & 0xff);
            subdrv_i2c_wr_u8(ctx, 0x320f, ctx->frame_length & 0xFF);
        }
        realtime_fps = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;
        DRV_LOG(ctx, "write_frame_length %u realtime_fps %u", ctx->frame_length, realtime_fps);
    }
    subdrv_i2c_wr_u8(ctx, 0x3e00, (shutter >> 16) & 0xFF);
    subdrv_i2c_wr_u8(ctx, 0x3e01, (shutter >> 8) & 0xFF);
    subdrv_i2c_wr_u8(ctx, 0x3e02, shutter & 0xFF);
    DRV_LOG(ctx,
        "set_shutter_frame_length- shutter =%llu, framelength =%u/%u, dummy_line=%d\n",
        shutter, ctx->frame_length,
        frame_length, dummy_line);

    //u32 shutter_1 = (subdrv_i2c_rd_u8(ctx, 0x3e00)+ subdrv_i2c_rd_u8(ctx, 0x3e01)+ subdrv_i2c_rd_u8(ctx, 0x3e02));
    //u32 framelength_1 = (subdrv_i2c_rd_u8(ctx, 0x326d)+ subdrv_i2c_rd_u8(ctx, 0x320e)+ subdrv_i2c_rd_u8(ctx, 0x320f));
    //DRV_LOG(ctx, "shutter %u, frame_length %u\n", shutter_1, framelength_1);
}

static int kluxmmain_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    kluxmmain_set_shutter_frame_length_convert(ctx, ((u64*)para)[0], ((u64*)para)[1]);
    return 0;
}

static int kluxmmain_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    u64 shutter = ((u64*)para)[0];
    kal_uint16 realtime_fps = 0;

    spin_lock(&imgsensor_drv_lock);
    if (shutter > ctx->min_frame_length - ctx->s_ctx.exposure_margin)
        ctx->frame_length = shutter + ctx->s_ctx.exposure_margin;
    else
        ctx->frame_length = ctx->min_frame_length;
    if (ctx->frame_length > ctx->s_ctx.frame_length_max)
        ctx->frame_length = ctx->s_ctx.frame_length_max;
    spin_unlock(&imgsensor_drv_lock);
    if (shutter < ctx->s_ctx.exposure_min)
        shutter = ctx->s_ctx.exposure_min;

    shutter = (shutter > (ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin)) ?
        (ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin) : shutter;

    if (set_auto_flicker(ctx, 0) || !ctx->s_ctx.reg_addr_auto_extend) {
        ctx->frame_length = (ctx->frame_length + 7) & ~0x07;
        subdrv_i2c_wr_u8(ctx, 0x326d, (ctx->frame_length >> 16) & 0x7f);
        subdrv_i2c_wr_u8(ctx, 0x320e, (ctx->frame_length >> 8) & 0xff);
        subdrv_i2c_wr_u8(ctx, 0x320f, ctx->frame_length & 0xFF);
        realtime_fps = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;
        DRV_LOG(ctx, "write_frame_length %u realtime_fps %u", ctx->frame_length, realtime_fps);
    }
    subdrv_i2c_wr_u8(ctx, 0x3e00, (shutter >> 16) & 0xFF);
    subdrv_i2c_wr_u8(ctx, 0x3e01, (shutter >> 8) & 0xFF);
    subdrv_i2c_wr_u8(ctx, 0x3e02, shutter & 0xFF);
    LOG_INF("Exit! shutter = %llu, framelength = %u\n", shutter, ctx->frame_length);
    u32 shutter_1 = ((subdrv_i2c_rd_u8(ctx, 0x3e00) << 16) + (subdrv_i2c_rd_u8(ctx, 0x3e01) << 8) + subdrv_i2c_rd_u8(ctx, 0x3e02));
    u32 framelength_1 = ((subdrv_i2c_rd_u8(ctx, 0x326d) << 16) + (subdrv_i2c_rd_u8(ctx, 0x320e) << 8) + subdrv_i2c_rd_u8(ctx, 0x320f));
    LOG_INF("write shutter %u, frame_length %u\n", shutter_1, framelength_1);
    return 0;
}

void kluxmmain_set_dummy(struct subdrv_ctx *ctx)
{
}

#ifndef USE_SUBDRV_IXC_WR_REGS
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

    num = snprintf(strptr, remind, "[ ");
    remind -= num;
    strptr += num;

    for (i = 0 ; i < length; i ++) {
        num = snprintf(strptr, remind, "0x%02x, ", buf[i]);

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

    num = snprintf(strptr, remind, " ]");
    remind -= num;
    strptr += num;

    DRV_LOG(ctx, " write %s\n", out_str);
    strptr = out_str;
    remind = buf_size;

    kfree(out_str);
}
#endif

static int kluxmmain_i2c_burst_wr_regs_u8(struct subdrv_ctx * ctx, u16 * list, u32 len)
{
    ktime_t start_time, end_time;
    s64 time_us;
    int ret;
    start_time = ktime_get();
#ifdef USE_SUBDRV_IXC_WR_REGS
    ret = subdrv_ixc_wr_regs_u8(ctx, list, len);
#else
    ret = adapter_i2c_burst_wr_regs_u8(ctx, ctx->i2c_write_id >> 1, list, len);
#endif
    end_time = ktime_get();
    time_us = ktime_to_us(ktime_sub(end_time, start_time));

    DRV_LOG(ctx, " cost time: %lld us, len: %u", time_us, len);

    return ret;
}

#ifndef USE_SUBDRV_IXC_WR_REGS
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
    int ret;
    int i;

    if (!msg_buf) {
        LOG_INF("malloc msg_buf retry");
        msg_buf = kmalloc(MAX_BURST_LEN, GFP_KERNEL);
        if (!msg_buf) {
            LOG_INF("malloc error");
            return -ENOMEM;
        }
    }

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

        pbuf[0] = plist[0] >> 8;
        pbuf[1] = plist[0] & 0xff;
        pbuf[2] = plist[1] & 0xff;

        pbuf += 3;
        pmsg->len = 3;
        per_sent += 1;

        for (i = 0; i < total - sent - 1; i++) {
            if (plist[0] + 1 == plist[2]) {
                pbuf[0] = plist[3] & 0xff;

                pbuf += 1;
                pmsg->len += 1;
                per_sent += 1;
                plist += 2;

                if (pmsg->len >= MAX_BURST_LEN) {
                    break;
                }
            }
        }
        plist += 2;

        if (dump_i2c_enable) {
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
#endif

static int kluxmmain_get_readout_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

    readout = (linelength * h2_tg_size * 1000000000 / pclk);

    feature_data[1] = readout;

    DRV_LOG(ctx, "%s scenario_id(%llu)  pclk(%llu) linelength(%llu) h2_tg_size(%u) readout(%llu)",
        __func__, scenario_id, pclk, linelength, h2_tg_size, readout);

    return 0;
}

/* static void update_CTLE(struct subdrv_ctx *ctx)
 {
     for (int scenario_id = 0; scenario_id < ctx->s_ctx.sensor_mode_num; ++scenario_id){
         if (ctx->s_ctx.sensor_id == KLUXMMAIN_SENSOR_ID) {
             ctx->s_ctx.mode[scenario_id].csi_param.dphy_ctle = WIDEC1_CTLE_LEVEL;
             ctx->s_ctx.mode[scenario_id].csi_param.dphy_eq_bw = WIDEC1_CTLE_EQBW;
             LOG_INF("update_CTLE_C1, scenario_id: %d\n", scenario_id);
         } else {
             LOG_INF("update_CTLE false\n");
         }
     }
 }*/
