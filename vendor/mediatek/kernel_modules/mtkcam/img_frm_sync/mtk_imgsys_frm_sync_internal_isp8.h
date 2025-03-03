/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2018 MediaTek Inc.
 *
 */
#ifndef MTK_IMGSYS_FRM_SYNC_INTERNAL_ISP8_H
#define MTK_IMGSYS_FRM_SYNC_INTERNAL_ISP8_H

#include <linux/mutex.h>
#include <linux/hashtable.h>
#include <linux/slab.h>
//debug use
#define EVT_HISTB_ON  1
#define EVT_HISTB_MAX_USER  20
/* log level definition*/
unsigned int ITF_LOG_LEVEL;
/**
 *  @struct dltb_t
 *  @brief information for a HW(refer HWID) in direct link table
 *
 *  @var on         indicating the hw is on or not
 *  @var src_fmt    src image format in direct link path, for module to update
 * path selection
 *  @var src_wd     src image width in direct link path, for module to update
 * tpipe structure
 *  @var src_ht     src image height in direct link path, for module to update
 * tpipe structure
 */
struct dltb_t {
	int on;
	unsigned int src_fmt;
	unsigned int src_wd;
	unsigned int src_ht;
};

enum sync_event_type_e {
	sync_type_none = 0,
	sync_type_set = 1,
	sync_type_wait = 2
};

enum HWID {
	HW_WPE_EIS = 0,
	HW_WPE_TNR,
	HW_WPE_LITE,
	HW_ADL_A,
	HW_ADL_B,
	HW_TRAW,
	HW_LTRAW,
	HW_XTRAW,
	HW_DIP,
	HW_PQDIP_A,
	HW_PQDIP_B,
	HW_TDR_MAX,
	HW_ME = HW_TDR_MAX,
	HW_MAX,
} HWID;

enum EVENT_TYPE_ENUM {
	IMGSYS_TRAW_0_DONE = 0,
	IMGSYS_TRAW_1_DONE,
	IMGSYS_TRAW_2_DONE,
	IMGSYS_TRAW_3_DONE,
	IMGSYS_TRAW_4_DONE,
	IMGSYS_TRAW_5_DONE,
	IMGSYS_TRAW_6_DONE,
	IMGSYS_TRAW_7_DONE,
	IMGSYS_TRAW_8_DONE,
	IMGSYS_TRAW_9_DONE,
	IMGSYS_LTRAW_0_DONE,
	IMGSYS_LTRAW_1_DONE,
	IMGSYS_LTRAW_2_DONE,
	IMGSYS_LTRAW_3_DONE,
	IMGSYS_LTRAW_4_DONE,
	IMGSYS_LTRAW_5_DONE,
	IMGSYS_LTRAW_6_DONE,
	IMGSYS_LTRAW_7_DONE,
	IMGSYS_LTRAW_8_DONE,
	IMGSYS_LTRAW_9_DONE,
	IMGSYS_DIP_0_DONE,
	IMGSYS_DIP_1_DONE,
	IMGSYS_DIP_2_DONE,
	IMGSYS_DIP_3_DONE,
	IMGSYS_DIP_4_DONE,
	IMGSYS_DIP_5_DONE,
	IMGSYS_DIP_6_DONE,
	IMGSYS_DIP_7_DONE,
	IMGSYS_DIP_8_DONE,
	IMGSYS_DIP_9_DONE,
	IMGSYS_PQDIP_A_0_DONE,
	IMGSYS_PQDIP_A_1_DONE,
	IMGSYS_PQDIP_A_2_DONE,
	IMGSYS_PQDIP_A_3_DONE,
	IMGSYS_PQDIP_A_4_DONE,
	IMGSYS_PQDIP_A_5_DONE,
	IMGSYS_PQDIP_A_6_DONE,
	IMGSYS_PQDIP_A_7_DONE,
	IMGSYS_PQDIP_A_8_DONE,
	IMGSYS_PQDIP_A_9_DONE,
	IMGSYS_PQDIP_B_0_DONE,
	IMGSYS_PQDIP_B_1_DONE,
	IMGSYS_PQDIP_B_2_DONE,
	IMGSYS_PQDIP_B_3_DONE,
	IMGSYS_PQDIP_B_4_DONE,
	IMGSYS_PQDIP_B_5_DONE,
	IMGSYS_PQDIP_B_6_DONE,
	IMGSYS_PQDIP_B_7_DONE,
	IMGSYS_PQDIP_B_8_DONE,
	IMGSYS_PQDIP_B_9_DONE,
	IMGSYS_WPE_EIS_0_DONE,
	IMGSYS_WPE_EIS_1_DONE,
	IMGSYS_WPE_EIS_2_DONE,
	IMGSYS_WPE_EIS_3_DONE,
	IMGSYS_WPE_EIS_4_DONE,
	IMGSYS_WPE_EIS_5_DONE,
	IMGSYS_WPE_EIS_6_DONE,
	IMGSYS_WPE_EIS_7_DONE,
	IMGSYS_WPE_EIS_8_DONE,
	IMGSYS_WPE_EIS_9_DONE,
	IMGSYS_WPE_TNR_0_DONE,
	IMGSYS_WPE_TNR_1_DONE,
	IMGSYS_WPE_TNR_2_DONE,
	IMGSYS_WPE_TNR_3_DONE,
	IMGSYS_WPE_TNR_4_DONE,
	IMGSYS_WPE_TNR_5_DONE,
	IMGSYS_WPE_TNR_6_DONE,
	IMGSYS_WPE_TNR_7_DONE,
	IMGSYS_WPE_TNR_8_DONE,
	IMGSYS_WPE_TNR_9_DONE,
	IMGSYS_WPE_LITE_0_DONE,
	IMGSYS_WPE_LITE_1_DONE,
	IMGSYS_WPE_LITE_2_DONE,
	IMGSYS_WPE_LITE_3_DONE,
	IMGSYS_WPE_LITE_4_DONE,
	IMGSYS_WPE_LITE_5_DONE,
	IMGSYS_WPE_LITE_6_DONE,
	IMGSYS_WPE_LITE_7_DONE,
	IMGSYS_WPE_LITE_8_DONE,
	IMGSYS_WPE_LITE_9_DONE,
	IMGSYS_ME_DONE,
	IMGSYS_ADL_TILE_DONE,
	IMGSYS_SYNC_TOKEN_WPE_EIS,
	IMGSYS_SYNC_TOKEN_WPE_TNR,
	IMGSYS_SYNC_TOKEN_WPE_LITE,
	IMGSYS_SYNC_TOKEN_TRAW,
	IMGSYS_SYNC_TOKEN_LTRAW,
	IMGSYS_SYNC_TOKEN_DIP,
	IMGSYS_SYNC_TOKEN_PQDIP_A,
	IMGSYS_SYNC_TOKEN_PQDIP_B,
	IMGSYS_SYNC_TOKEN_IPESYS_ME,
	IMGSYS_SYNC_TOKEN_APUSYS_APU,
	IMGSYS_SYNC_TOKEN_IMGSYS_VSS_TRAW,
	IMGSYS_SYNC_TOKEN_IMGSYS_VSS_LTRAW,
	IMGSYS_SYNC_TOKEN_IMGSYS_VSS_DIP,
	/*package frame sync, cross package sync usage*/
	IMGSYS_SYNC_TOKEN_POOL_1,
	IMGSYS_SYNC_TOKEN_POOL_2,
	IMGSYS_SYNC_TOKEN_POOL_3,
	IMGSYS_SYNC_TOKEN_POOL_4,
	IMGSYS_SYNC_TOKEN_POOL_5,
	IMGSYS_SYNC_TOKEN_POOL_6,
	IMGSYS_SYNC_TOKEN_POOL_7,
	IMGSYS_SYNC_TOKEN_POOL_8,
	IMGSYS_SYNC_TOKEN_POOL_9,
	IMGSYS_SYNC_TOKEN_POOL_10,
	IMGSYS_SYNC_TOKEN_POOL_11,
	IMGSYS_SYNC_TOKEN_POOL_12,
	IMGSYS_SYNC_TOKEN_POOL_13,
	IMGSYS_SYNC_TOKEN_POOL_14,
	IMGSYS_SYNC_TOKEN_POOL_15,
	IMGSYS_SYNC_TOKEN_POOL_16,
	IMGSYS_SYNC_TOKEN_POOL_17,
	IMGSYS_SYNC_TOKEN_POOL_18,
	IMGSYS_SYNC_TOKEN_POOL_19,
	IMGSYS_SYNC_TOKEN_POOL_20,
	IMGSYS_SYNC_TOKEN_POOL_21,
	IMGSYS_SYNC_TOKEN_POOL_22,
	IMGSYS_SYNC_TOKEN_POOL_23,
	IMGSYS_SYNC_TOKEN_POOL_24,
	IMGSYS_SYNC_TOKEN_POOL_25,
	IMGSYS_SYNC_TOKEN_POOL_26,
	IMGSYS_SYNC_TOKEN_POOL_27,
	IMGSYS_SYNC_TOKEN_POOL_28,
	IMGSYS_SYNC_TOKEN_POOL_29,
	IMGSYS_SYNC_TOKEN_POOL_30,
	IMGSYS_SYNC_TOKEN_POOL_31,
	IMGSYS_SYNC_TOKEN_POOL_32,
	IMGSYS_SYNC_TOKEN_POOL_33,
	IMGSYS_SYNC_TOKEN_POOL_34,
	IMGSYS_SYNC_TOKEN_POOL_35,
	IMGSYS_SYNC_TOKEN_POOL_36,
	IMGSYS_SYNC_TOKEN_POOL_37,
	IMGSYS_SYNC_TOKEN_POOL_38,
	IMGSYS_SYNC_TOKEN_POOL_39,
	IMGSYS_SYNC_TOKEN_POOL_40,
	IMGSYS_SYNC_TOKEN_POOL_41,
	IMGSYS_SYNC_TOKEN_POOL_42,
	IMGSYS_SYNC_TOKEN_POOL_43,
	IMGSYS_SYNC_TOKEN_POOL_44,
	IMGSYS_SYNC_TOKEN_POOL_45,
	IMGSYS_SYNC_TOKEN_POOL_46,
	IMGSYS_SYNC_TOKEN_POOL_47,
	IMGSYS_SYNC_TOKEN_POOL_48,
	IMGSYS_SYNC_TOKEN_POOL_49,
	IMGSYS_SYNC_TOKEN_POOL_50,
	IMGSYS_SYNC_TOKEN_POOL_51,
	IMGSYS_SYNC_TOKEN_POOL_52,
	IMGSYS_SYNC_TOKEN_POOL_53,
	IMGSYS_SYNC_TOKEN_POOL_54,
	IMGSYS_SYNC_TOKEN_POOL_55,
	IMGSYS_SYNC_TOKEN_POOL_56,
	IMGSYS_SYNC_TOKEN_POOL_57,
	IMGSYS_SYNC_TOKEN_POOL_58,
	IMGSYS_SYNC_TOKEN_POOL_59,
	IMGSYS_SYNC_TOKEN_POOL_60,
	IMGSYS_SYNC_TOKEN_POOL_61,
	IMGSYS_SYNC_TOKEN_POOL_62,
	IMGSYS_SYNC_TOKEN_POOL_63,
	IMGSYS_SYNC_TOKEN_POOL_64,
	IMGSYS_SYNC_TOKEN_POOL_65,
	IMGSYS_SYNC_TOKEN_POOL_66,
	IMGSYS_SYNC_TOKEN_POOL_67,
	IMGSYS_SYNC_TOKEN_POOL_68,
	IMGSYS_SYNC_TOKEN_POOL_69,
	IMGSYS_SYNC_TOKEN_POOL_70,
	IMGSYS_SYNC_TOKEN_POOL_71,
	IMGSYS_SYNC_TOKEN_POOL_72,
	IMGSYS_SYNC_TOKEN_POOL_73,
	IMGSYS_SYNC_TOKEN_POOL_74,
	IMGSYS_SYNC_TOKEN_POOL_75,
	IMGSYS_SYNC_TOKEN_POOL_76,
	IMGSYS_SYNC_TOKEN_POOL_77,
	IMGSYS_SYNC_TOKEN_POOL_78,
	IMGSYS_SYNC_TOKEN_POOL_79,
	IMGSYS_SYNC_TOKEN_POOL_80,
	IMGSYS_SYNC_TOKEN_POOL_81,
	IMGSYS_SYNC_TOKEN_POOL_82,
	IMGSYS_SYNC_TOKEN_POOL_83,
	IMGSYS_SYNC_TOKEN_POOL_84,
	IMGSYS_SYNC_TOKEN_POOL_85,
	IMGSYS_SYNC_TOKEN_POOL_86,
	IMGSYS_SYNC_TOKEN_POOL_87,
	IMGSYS_SYNC_TOKEN_POOL_88,
	IMGSYS_SYNC_TOKEN_POOL_89,
	IMGSYS_SYNC_TOKEN_POOL_90,
	IMGSYS_SYNC_TOKEN_POOL_91,
	IMGSYS_SYNC_TOKEN_POOL_92,
	IMGSYS_SYNC_TOKEN_POOL_93,
	IMGSYS_SYNC_TOKEN_POOL_94,
	IMGSYS_SYNC_TOKEN_POOL_95,
	IMGSYS_SYNC_TOKEN_POOL_96,
	IMGSYS_SYNC_TOKEN_POOL_97,
	IMGSYS_SYNC_TOKEN_POOL_98,
	IMGSYS_SYNC_TOKEN_POOL_99,
	IMGSYS_SYNC_TOKEN_POOL_100,
	IMGSYS_SYNC_TOKEN_POOL_101,
	IMGSYS_SYNC_TOKEN_POOL_102,
	IMGSYS_SYNC_TOKEN_POOL_103,
	IMGSYS_SYNC_TOKEN_POOL_104,
	IMGSYS_SYNC_TOKEN_POOL_105,
	IMGSYS_SYNC_TOKEN_POOL_106,
	IMGSYS_SYNC_TOKEN_POOL_107,
	IMGSYS_SYNC_TOKEN_POOL_108,
	IMGSYS_SYNC_TOKEN_POOL_109,
	IMGSYS_SYNC_TOKEN_POOL_110,
	IMGSYS_SYNC_TOKEN_POOL_111,
	IMGSYS_SYNC_TOKEN_POOL_112,
	IMGSYS_SYNC_TOKEN_POOL_113,
	IMGSYS_SYNC_TOKEN_POOL_114,
	IMGSYS_SYNC_TOKEN_POOL_115,
	IMGSYS_SYNC_TOKEN_POOL_116,
	IMGSYS_SYNC_TOKEN_POOL_117,
	IMGSYS_SYNC_TOKEN_POOL_118,
	IMGSYS_SYNC_TOKEN_POOL_119,
	IMGSYS_SYNC_TOKEN_POOL_120,
	IMGSYS_SYNC_TOKEN_POOL_121,
	IMGSYS_SYNC_TOKEN_POOL_122,
	IMGSYS_SYNC_TOKEN_POOL_123,
	IMGSYS_SYNC_TOKEN_POOL_124,
	IMGSYS_SYNC_TOKEN_POOL_125,
	IMGSYS_SYNC_TOKEN_POOL_126,
	IMGSYS_SYNC_TOKEN_POOL_127,
	IMGSYS_SYNC_TOKEN_POOL_128,
	IMGSYS_SYNC_TOKEN_POOL_129,
	IMGSYS_SYNC_TOKEN_POOL_130,
	IMGSYS_SYNC_TOKEN_POOL_131,
	IMGSYS_SYNC_TOKEN_POOL_132,
	IMGSYS_SYNC_TOKEN_POOL_133,
	IMGSYS_SYNC_TOKEN_POOL_134,
	IMGSYS_SYNC_TOKEN_POOL_135,
	IMGSYS_SYNC_TOKEN_POOL_136,
	IMGSYS_SYNC_TOKEN_POOL_137,
	IMGSYS_SYNC_TOKEN_POOL_138,
	IMGSYS_SYNC_TOKEN_POOL_139,
	IMGSYS_SYNC_TOKEN_POOL_140,
	IMGSYS_SYNC_TOKEN_POOL_141,
	IMGSYS_SYNC_TOKEN_POOL_142,
	IMGSYS_SYNC_TOKEN_POOL_143,
	IMGSYS_SYNC_TOKEN_POOL_144,
	IMGSYS_SYNC_TOKEN_POOL_145,
	IMGSYS_SYNC_TOKEN_POOL_146,
	IMGSYS_SYNC_TOKEN_POOL_147,
	IMGSYS_SYNC_TOKEN_POOL_148,
	IMGSYS_SYNC_TOKEN_POOL_149,
	IMGSYS_SYNC_TOKEN_POOL_150,
	IMGSYS_SYNC_TOKEN_POOL_151,
	IMGSYS_SYNC_TOKEN_POOL_152,
	IMGSYS_SYNC_TOKEN_POOL_153,
	IMGSYS_SYNC_TOKEN_POOL_154,
	IMGSYS_SYNC_TOKEN_POOL_155,
	IMGSYS_SYNC_TOKEN_POOL_156,
	IMGSYS_SYNC_TOKEN_POOL_157,
	IMGSYS_SYNC_TOKEN_POOL_158,
	IMGSYS_SYNC_TOKEN_POOL_159,
	IMGSYS_SYNC_TOKEN_POOL_160,
	IMGSYS_SYNC_TOKEN_POOL_161,
	IMGSYS_SYNC_TOKEN_POOL_162,
	IMGSYS_SYNC_TOKEN_POOL_163,
	IMGSYS_SYNC_TOKEN_POOL_164,
	IMGSYS_SYNC_TOKEN_POOL_165,
	IMGSYS_SYNC_TOKEN_POOL_166,
	IMGSYS_SYNC_TOKEN_POOL_167,
	IMGSYS_SYNC_TOKEN_POOL_168,
	IMGSYS_SYNC_TOKEN_POOL_169,
	IMGSYS_SYNC_TOKEN_POOL_170,
	IMGSYS_SYNC_TOKEN_POOL_171,
	IMGSYS_SYNC_TOKEN_POOL_172,
	IMGSYS_SYNC_TOKEN_POOL_173,
	IMGSYS_SYNC_TOKEN_POOL_174,
	IMGSYS_SYNC_TOKEN_POOL_175,
	IMGSYS_SYNC_TOKEN_POOL_176,
	IMGSYS_SYNC_TOKEN_POOL_177,
	IMGSYS_SYNC_TOKEN_POOL_178,
	IMGSYS_SYNC_TOKEN_POOL_179,
	IMGSYS_SYNC_TOKEN_POOL_180,
	IMGSYS_SYNC_TOKEN_POOL_181,
	IMGSYS_SYNC_TOKEN_POOL_182,
	IMGSYS_SYNC_TOKEN_POOL_183,
	IMGSYS_SYNC_TOKEN_POOL_184,
	IMGSYS_SYNC_TOKEN_POOL_185,
	IMGSYS_SYNC_TOKEN_POOL_186,
	IMGSYS_SYNC_TOKEN_POOL_187,
	IMGSYS_SYNC_TOKEN_POOL_188,
	IMGSYS_SYNC_TOKEN_POOL_189,
	IMGSYS_SYNC_TOKEN_POOL_190,
	IMGSYS_SYNC_TOKEN_POOL_191,
	IMGSYS_SYNC_TOKEN_POOL_192,
	IMGSYS_SYNC_TOKEN_POOL_193,
	IMGSYS_SYNC_TOKEN_POOL_194,
	IMGSYS_SYNC_TOKEN_POOL_195,
	IMGSYS_SYNC_TOKEN_POOL_196,
	IMGSYS_SYNC_TOKEN_POOL_197,
	IMGSYS_SYNC_TOKEN_POOL_198,
	IMGSYS_SYNC_TOKEN_POOL_199,
	IMGSYS_SYNC_TOKEN_POOL_200,
	IMGSYS_SYNC_TOKEN_POOL_201,
	IMGSYS_SYNC_TOKEN_POOL_202,
	IMGSYS_SYNC_TOKEN_POOL_203,
	IMGSYS_SYNC_TOKEN_POOL_204,
	IMGSYS_SYNC_TOKEN_POOL_205,
	IMGSYS_SYNC_TOKEN_POOL_206,
	IMGSYS_SYNC_TOKEN_POOL_207,
	IMGSYS_SYNC_TOKEN_POOL_208,
	IMGSYS_SYNC_TOKEN_POOL_209,
	IMGSYS_SYNC_TOKEN_POOL_210,
	IMGSYS_SYNC_TOKEN_POOL_211,
	IMGSYS_SYNC_TOKEN_POOL_212,
	IMGSYS_SYNC_TOKEN_POOL_213,
	IMGSYS_SYNC_TOKEN_POOL_214,
	IMGSYS_SYNC_TOKEN_POOL_215,
	IMGSYS_SYNC_TOKEN_POOL_216,
	IMGSYS_SYNC_TOKEN_POOL_217,
	IMGSYS_SYNC_TOKEN_POOL_218,
	IMGSYS_SYNC_TOKEN_POOL_219,
	IMGSYS_SYNC_TOKEN_POOL_220,
	IMGSYS_SYNC_TOKEN_POOL_221,
	IMGSYS_SYNC_TOKEN_POOL_222,
	IMGSYS_SYNC_TOKEN_POOL_223,
	IMGSYS_SYNC_TOKEN_POOL_224,
	IMGSYS_SYNC_TOKEN_POOL_225,
	IMGSYS_SYNC_TOKEN_POOL_226,
	IMGSYS_SYNC_TOKEN_POOL_227,
	IMGSYS_SYNC_TOKEN_POOL_228,
	IMGSYS_SYNC_TOKEN_POOL_229,
	IMGSYS_SYNC_TOKEN_POOL_230,
	IMGSYS_SYNC_TOKEN_POOL_231,
	IMGSYS_SYNC_TOKEN_POOL_232,
	IMGSYS_SYNC_TOKEN_POOL_233,
	IMGSYS_SYNC_TOKEN_POOL_234,
	IMGSYS_SYNC_TOKEN_POOL_235,
	IMGSYS_SYNC_TOKEN_POOL_236,
	IMGSYS_SYNC_TOKEN_POOL_237,
	IMGSYS_SYNC_TOKEN_POOL_238,
	IMGSYS_SYNC_TOKEN_POOL_239,
	IMGSYS_SYNC_TOKEN_POOL_240,
	IMGSYS_SYNC_TOKEN_POOL_241,
	IMGSYS_SYNC_TOKEN_POOL_242,
	IMGSYS_SYNC_TOKEN_POOL_243,
	IMGSYS_SYNC_TOKEN_POOL_244,
	IMGSYS_SYNC_TOKEN_POOL_245,
	IMGSYS_SYNC_TOKEN_POOL_246,
	IMGSYS_SYNC_TOKEN_POOL_247,
	IMGSYS_SYNC_TOKEN_POOL_248,
	IMGSYS_SYNC_TOKEN_POOL_249,
	IMGSYS_SYNC_TOKEN_POOL_250,
	IMGSYS_SYNC_TOKEN_POOL_251,
	IMGSYS_SYNC_TOKEN_POOL_252,
	IMGSYS_SYNC_TOKEN_POOL_253,
	IMGSYS_SYNC_TOKEN_POOL_254,
	IMGSYS_SYNC_TOKEN_POOL_255,
	IMGSYS_SYNC_TOKEN_POOL_256,
	IMGSYS_SYNC_TOKEN_POOL_257,
	IMGSYS_SYNC_TOKEN_POOL_258,
	IMGSYS_SYNC_TOKEN_POOL_259,
	IMGSYS_SYNC_TOKEN_POOL_260,
	IMGSYS_SYNC_TOKEN_POOL_261,
	IMGSYS_SYNC_TOKEN_POOL_262,
	IMGSYS_SYNC_TOKEN_POOL_263,
	IMGSYS_SYNC_TOKEN_POOL_264,
	IMGSYS_SYNC_TOKEN_POOL_265,
	IMGSYS_SYNC_TOKEN_POOL_266,
	IMGSYS_SYNC_TOKEN_POOL_267,
	IMGSYS_SYNC_TOKEN_POOL_268,
	IMGSYS_SYNC_TOKEN_POOL_269,
	IMGSYS_SYNC_TOKEN_POOL_270,
	IMGSYS_SYNC_TOKEN_POOL_271,
	IMGSYS_SYNC_TOKEN_POOL_272,
	IMGSYS_SYNC_TOKEN_POOL_273,
	IMGSYS_SYNC_TOKEN_POOL_274,
	IMGSYS_SYNC_TOKEN_POOL_275,
	IMGSYS_SYNC_TOKEN_POOL_276,
	IMGSYS_SYNC_TOKEN_POOL_277,
	IMGSYS_SYNC_TOKEN_POOL_278,
	IMGSYS_SYNC_TOKEN_POOL_279,
	IMGSYS_SYNC_TOKEN_POOL_280,
	IMGSYS_SYNC_TOKEN_POOL_281,
	IMGSYS_SYNC_TOKEN_POOL_282,
	IMGSYS_SYNC_TOKEN_POOL_283,
	IMGSYS_SYNC_TOKEN_POOL_284,
	IMGSYS_SYNC_TOKEN_POOL_285,
	IMGSYS_SYNC_TOKEN_POOL_286,
	IMGSYS_SYNC_TOKEN_POOL_287,
	IMGSYS_SYNC_TOKEN_POOL_288,
	IMGSYS_SYNC_TOKEN_POOL_289,
	IMGSYS_SYNC_TOKEN_POOL_290,
	IMGSYS_SYNC_TOKEN_POOL_291,
	IMGSYS_SYNC_TOKEN_POOL_292,
	IMGSYS_SYNC_TOKEN_POOL_293,
	IMGSYS_SYNC_TOKEN_POOL_294,
	IMGSYS_SYNC_TOKEN_POOL_295,
	IMGSYS_SYNC_TOKEN_POOL_296,
	IMGSYS_SYNC_TOKEN_POOL_297,
	IMGSYS_SYNC_TOKEN_POOL_298,
	IMGSYS_SYNC_TOKEN_POOL_299,
	IMGSYS_SYNC_TOKEN_POOL_300,
	IMGSYS_SYNC_TOKEN_POOL_MAX = IMGSYS_SYNC_TOKEN_POOL_300,
	/* DPE sync tokens*/
	IMGSYS_SYNC_TOKEN_DPE_POOL_1,
	IMGSYS_SYNC_TOKEN_DPE_POOL_2,
	IMGSYS_SYNC_TOKEN_DPE_POOL_3,
	IMGSYS_SYNC_TOKEN_DPE_POOL_4,
	IMGSYS_SYNC_TOKEN_DPE_POOL_5,
	IMGSYS_SYNC_TOKEN_DPE_POOL_6,
	IMGSYS_SYNC_TOKEN_DPE_POOL_7,
	IMGSYS_SYNC_TOKEN_DPE_POOL_8,
	IMGSYS_SYNC_TOKEN_DPE_POOL_9,
	IMGSYS_SYNC_TOKEN_DPE_POOL_10,
	IMGSYS_SYNC_TOKEN_DPE_POOL_11,
	IMGSYS_SYNC_TOKEN_DPE_POOL_12,
	IMGSYS_SYNC_TOKEN_DPE_POOL_13,
	IMGSYS_SYNC_TOKEN_DPE_POOL_14,
	IMGSYS_SYNC_TOKEN_DPE_POOL_15,
	IMGSYS_SYNC_TOKEN_DPE_POOL_16,
	IMGSYS_SYNC_TOKEN_DPE_POOL_MAX = IMGSYS_SYNC_TOKEN_DPE_POOL_16,
	/*camsys/imgsys sync token*/
	IMGSYS_SYNC_TOKEN_CAMSYS_POOL_1,
	IMGSYS_SYNC_TOKEN_CAMSYS_POOL_2,
	IMGSYS_SYNC_TOKEN_CAMSYS_POOL_3,
	IMGSYS_SYNC_TOKEN_CAMSYS_POOL_4,
	IMGSYS_SYNC_TOKEN_CAMSYS_POOL_5,
	IMGSYS_SYNC_TOKEN_CAMSYS_POOL_6,
	IMGSYS_SYNC_TOKEN_CAMSYS_POOL_7,
	IMGSYS_SYNC_TOKEN_CAMSYS_POOL_8,
	IMGSYS_SYNC_TOKEN_CAMSYS_POOL_9,
	IMGSYS_SYNC_TOKEN_CAMSYS_POOL_10,
	IMGSYS_SYNC_TOKEN_CAMSYS_POOL_MAX = IMGSYS_SYNC_TOKEN_CAMSYS_POOL_10,
	SYNC_TOKEN_GPR_READ,
	SYNC_TOKEN_GPR_WRITE,
	SYNC_TOKEN_GPR_POLL,
	SYNC_TOKEN_GPR_WRITE_FROM_MEM,
	SYNC_TOKEN_GPR_WRITE_FROM_REG
} EVENT_TYPE_ENUM;

/*event table structure*/
struct frm_sync_event_table {
	u16 event;	/* imgsys,dpe,mae event enum value */
	char dts_name[256];
};

enum mtk_dpe_event {
	/* SW event */
	IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_START,
	IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_1 = IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_START,
	IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_2,
	IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_3,
	IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_4,
	IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_5,
	IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_6,
	IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_7,
	IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_8,
	IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_9,
	IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_10,
	IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_11,
	IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_12,
	IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_13,
	IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_14,
	IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_15,
	IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_16,
	IMGSYS_DPE_EVENT_POOL_MAX = IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_16,
	IMGSYS_DPE_EVENT_MAX
};

static struct frm_sync_event_table imgsys_event[] = {
	{IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_1, "dpe-sw-sync-token-pool-1"},
	{IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_2, "dpe-sw-sync-token-pool-2"},
	{IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_3, "dpe-sw-sync-token-pool-3"},
	{IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_4, "dpe-sw-sync-token-pool-4"},
	{IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_5, "dpe-sw-sync-token-pool-5"},
	{IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_6, "dpe-sw-sync-token-pool-6"},
	{IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_7, "dpe-sw-sync-token-pool-7"},
	{IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_8, "dpe-sw-sync-token-pool-8"},
	{IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_9, "dpe-sw-sync-token-pool-9"},
	{IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_10, "dpe-sw-sync-token-pool-10"},
	{IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_11, "dpe-sw-sync-token-pool-11"},
	{IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_12, "dpe-sw-sync-token-pool-12"},
	{IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_13, "dpe-sw-sync-token-pool-13"},
	{IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_14, "dpe-sw-sync-token-pool-14"},
	{IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_15, "dpe-sw-sync-token-pool-15"},
	{IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_16, "dpe-sw-sync-token-pool-16"},
	{IMGSYS_DPE_EVENT_MAX, "imgsys-dpe-event-max"},
};

#define DPE_ALL_SYNC_TOKEN_MAX (IMGSYS_DPE_EVENT_MAX - IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_1)
#define DPE_SYNC_TOKEN_MAX (IMGSYS_DPE_EVENT_MAX - IMGSYS_DPE_SYNC_TOKEN_DPE_POOL_1)
#define IMGSYS_ALL_SYNC_TOKEN_MAX (IMGSYS_SYNC_TOKEN_CAMSYS_POOL_MAX - IMGSYS_SYNC_TOKEN_POOL_1 + 1)
#define IMGSYS_SYNC_TOKEN_MAX (IMGSYS_SYNC_TOKEN_DPE_POOL_MAX - IMGSYS_SYNC_TOKEN_POOL_1 + 1)
#define IMGSYS_DPE_TOKEN_BASE (IMGSYS_SYNC_TOKEN_DPE_POOL_1 - IMGSYS_SYNC_TOKEN_POOL_1)
#define IMGSYS_DPE_TOKEN_OFFSET (IMGSYS_SYNC_TOKEN_DPE_POOL_16 - IMGSYS_SYNC_TOKEN_DPE_POOL_1)
/* sync token variable declaration */
#define IMGSTR_TOKEN_MAX 512 /*need align with imgstream*/
#define TOKEN_INVALID_ID -1

/* global variable for ISP8 */
/* token map info */
#define TOKEN_INVALID_ID -1
#define DPE_TOKEN_MAX 16

struct token_list_k {
	enum sync_event_type_e token_type;
	uint32_t mSyncToken;
};

struct mutex mcnr_tokenmap_lock;
struct mutex vsdof_tokenmap_lock;
struct mutex aiseg_tokenmap_lock;

struct tokenmap_t {
	enum sync_event_type_e type;
	int hw_token;
	uint64_t frm_owner;
	uint64_t imgstm_inst;
	int request_no;
	int request_fd;
	int frame_no;
	int stage;
	int subfrm_sidx;
	int evt_order;
	int evt_histb_idx;
	uint32_t hw_comb;
	uint32_t gce_event_id;
};

/**
 *  @struct tokendbg_t
 *  @brief struct contains frame information for token timeout debug
 *
 *  @var frm_owner          frame owner
 *  @var req_fd             request fd
 *  @var req_no             request no
 *  @var frame_no           frame no
 *  @var fidx               sub-frame index in request
 *  @var gce_gpID           adtoped gce group id
 *  @var gce_gpID           adtoped gce group id
 *  @var stoken_key         sw token key from mw
 *  @var htoken_key         hw token key from gce enum
 *  @var fence_fd           fence_fd from mw
 *  @var hw_comb            hw combination about this sub-frame
 *  @var fd_order          fence fd order in list
 *  @var evt_order          sync event order in list
 *  @var tick_time          tick time to use the token
 */
struct tokendbg_t {
	uint64_t frm_owner;
	uint64_t imgstm_inst;
	int req_fd;
	int req_no;
	int frame_no;
	int fidx;
	int gce_gpID;
	int gce_gpNumb;
	int stoken_key;
	int htoken_key;
	int fence_fd;
	uint32_t hw_comb;
	int fd_order;
	int evt_order;
};

static struct tokenmap_t mcnr_tokenmap[IMGSYS_ALL_SYNC_TOKEN_MAX];
static struct tokenmap_t vsdof_tokenmap[IMGSYS_ALL_SYNC_TOKEN_MAX];
static struct tokenmap_t aiseg_tokenmap[IMGSYS_ALL_SYNC_TOKEN_MAX];
static struct tokendbg_t tokenDebugInfo[2][DPE_ALL_SYNC_TOKEN_MAX];

#define MAX_WAIT_TOKEN_P_RUN 10 //max wait token in single run

struct event_map_t {
	struct tokenmap_t notify;
	struct tokenmap_t wait;
};

/**
 * enum buf_status_e
 *
 * Definition about working buffer status
 */
enum buf_status_e {
	buf_st_avai = 0,
	buf_st_occu = 1,
	buf_st_wait = 2,
	buf_st_exec = 3
};

struct buf_ofst_info_t {
	enum buf_status_e buf_st;
	bool need_release_token[IMGSYS_ALL_SYNC_TOKEN_MAX];
	int used_sw_token_cnt;
	int used_sw_token[IMGSYS_ALL_SYNC_TOKEN_MAX];
	int acquired_sw_token_cnt;
	int gce_event_id;
};

struct buf_pool_t {
	struct mutex buf_pool_lock;
	int r_idx;
	int r_max;
	int lowest_available_num;
	int *idx_pool;
};

struct buf_ofst_table_t {
	struct buf_pool_t pool;
	struct buf_ofst_info_t *info;
};

/**
 *  @struct token_info_t
 *  @brief sw sync token information
 *
 *  @var token_id       sw sync token id (gce event id)
 *  @var status         token status (available or occupied)
 */
struct token_info_t {
	int token_id;
	enum buf_status_e status;
};

/**
 *  @struct sw_sync_token_pool_t
 *  @brief sw sync token pool for frame sync and package sync
 *
 *  @var token_pool     Token pool (stored available sw sync token ids)
 *  @var tk_used_token_nump     Used token number for each tkdp thread
 */
struct sw_sync_token_pool_dpe {
	struct buf_pool_t pool;
	struct token_info_t token_pool[DPE_ALL_SYNC_TOKEN_MAX];
};

struct sw_sync_token_pool_imgsys {
	struct buf_pool_t pool;
	struct token_info_t token_pool[IMGSYS_ALL_SYNC_TOKEN_MAX];
};

struct event_history_t {
	int wait_evt_num;
	struct event_map_t evtmap_his[TMAX][MAX_WAIT_TOKEN_P_RUN];
};
static struct event_history_t *evtHisTb_DPE;

/**
 *  @struct sw_info_t
 *  @brief self-defined structure for judgement in TDR callback functions
 *
 *  @var hw_comb            hw combination
 *  @var dl_table           direct link table information
 *  @var sync_event_type    sync type for setting token event
 *  @var used_sw_token_cnt  used sw sync token number in current frame
 *  @var used_sw_token      used sw sync token ids in current frame
 *  @var wfence_num         number of fence fd needed to wait
 *  @var wfence_event       wait fence fd/gce event map
 *  @var nfence_num         number of fence fd needed to notify
 *  @var nfence_event       notify fence fd/gce event map
 *  @var is_me_rst_en       is me rst for HWMM enable or not
 */
struct sw_info_t {
	uint32_t hw_comb;
	struct dltb_t dl_table[HW_TDR_MAX];
	enum sync_event_type_e sync_event_type[DPE_ALL_SYNC_TOKEN_MAX];
	int used_sw_token_cnt;
	int used_sw_token[DPE_ALL_SYNC_TOKEN_MAX];
	int wfence_num;
	int nfence_num;
	int8_t is_me_rst_en;
};

/* token map index hash table */
struct token_map_index_data_t {
	int key;
	int value;
	struct hlist_node hnode;
};

#define dpe_token_map_hash_size	16
#define imgsys_token_map_hash_size	512

/* hw engine & algo init or not */
unsigned int dpe_init_flag;
unsigned int imgsys_init_flag;
unsigned int mae_init_flag;

/* gce event interval */
#define DPE_GCE_EVENT_BEGIN 874
#define DPE_GCE_EVENT_END 889
#define sync_pool_debug_msg_size 1600
#define sync_pool_debug_msg_size_tmp 100
#endif /* MTK_IMGSYS_FRM_SYNC_INTERNAL_ISP8_H */
