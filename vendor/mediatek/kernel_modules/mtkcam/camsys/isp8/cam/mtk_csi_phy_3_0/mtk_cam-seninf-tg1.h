/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __SENINF_TG_E1A_C_HEADER_H__
#define __SENINF_TG_E1A_C_HEADER_H__

#define SENINF_TG_SENINF_TG_TM_CTL 0x0000
#define SENINF_TG_SENINF_TG_TM_CORE_MODE_SHIFT 0
#define SENINF_TG_SENINF_TG_TM_CORE_MODE_MASK (0x1 << 0)
#define SENINF_TG_SENINF_TG_TM_CLRBAR_IDX_SHIFT 4
#define SENINF_TG_SENINF_TG_TM_CLRBAR_IDX_MASK (0x7 << 4)
#define SENINF_TG_SENINF_TG_TM_HIGH_BIT_MODE_SHIFT 8
#define SENINF_TG_SENINF_TG_TM_HIGH_BIT_MODE_MASK (0x1 << 8)
#define SENINF_TG_SENINF_TG_TM_PRE_HSYNC_LAT_SHIFT 16
#define SENINF_TG_SENINF_TG_TM_PRE_HSYNC_LAT_MASK (0xff << 16)
#define SENINF_TG_SENINF_TG_TM_VSYNC_LAT_SHIFT 24
#define SENINF_TG_SENINF_TG_TM_VSYNC_LAT_MASK (0xff << 24)

#define SENINF_TG_SENINF_TG_TM_INT_EN 0x0004
#define SENINF_TG_SENINF_TG_TM_EXP0_INT_EN_SHIFT 0
#define SENINF_TG_SENINF_TG_TM_EXP0_INT_EN_MASK (0x1 << 0)
#define SENINF_TG_SENINF_TG_TM_EXP1_INT_EN_SHIFT 1
#define SENINF_TG_SENINF_TG_TM_EXP1_INT_EN_MASK (0x1 << 1)
#define SENINF_TG_SENINF_TG_TM_EXP2_INT_EN_SHIFT 2
#define SENINF_TG_SENINF_TG_TM_EXP2_INT_EN_MASK (0x1 << 2)
#define SENINF_TG_SENINF_TG_TM_EXP3_INT_EN_SHIFT 3
#define SENINF_TG_SENINF_TG_TM_EXP3_INT_EN_MASK (0x1 << 3)
#define SENINF_TG_SENINF_TG_TM_EXP4_INT_EN_SHIFT 4
#define SENINF_TG_SENINF_TG_TM_EXP4_INT_EN_MASK (0x1 << 4)
#define SENINF_TG_SENINF_TG_TM_EXP5_INT_EN_SHIFT 5
#define SENINF_TG_SENINF_TG_TM_EXP5_INT_EN_MASK (0x1 << 5)
#define SENINF_TG_SENINF_TG_TM_EXP6_INT_EN_SHIFT 6
#define SENINF_TG_SENINF_TG_TM_EXP6_INT_EN_MASK (0x1 << 6)
#define SENINF_TG_SENINF_TG_TM_EXP7_INT_EN_SHIFT 7
#define SENINF_TG_SENINF_TG_TM_EXP7_INT_EN_MASK (0x1 << 7)
#define SENINF_TG_SENINF_TG_TM_INT_WCLR_EN_SHIFT 31
#define SENINF_TG_SENINF_TG_TM_INT_WCLR_EN_MASK (0x1 << 31)

#define SENINF_TG_SENINF_TG_TM_INT_ST 0x0008
#define SENINF_TG_SENINF_TG_TM_EXP0_INT_ST_SHIFT 0
#define SENINF_TG_SENINF_TG_TM_EXP0_INT_ST_MASK (0x1 << 0)
#define SENINF_TG_SENINF_TG_TM_EXP1_INT_ST_SHIFT 1
#define SENINF_TG_SENINF_TG_TM_EXP1_INT_ST_MASK (0x1 << 1)
#define SENINF_TG_SENINF_TG_TM_EXP2_INT_ST_SHIFT 2
#define SENINF_TG_SENINF_TG_TM_EXP2_INT_ST_MASK (0x1 << 2)
#define SENINF_TG_SENINF_TG_TM_EXP3_INT_ST_SHIFT 3
#define SENINF_TG_SENINF_TG_TM_EXP3_INT_ST_MASK (0x1 << 3)
#define SENINF_TG_SENINF_TG_TM_EXP4_INT_ST_SHIFT 4
#define SENINF_TG_SENINF_TG_TM_EXP4_INT_ST_MASK (0x1 << 4)
#define SENINF_TG_SENINF_TG_TM_EXP5_INT_ST_SHIFT 5
#define SENINF_TG_SENINF_TG_TM_EXP5_INT_ST_MASK (0x1 << 5)
#define SENINF_TG_SENINF_TG_TM_EXP6_INT_ST_SHIFT 6
#define SENINF_TG_SENINF_TG_TM_EXP6_INT_ST_MASK (0x1 << 6)
#define SENINF_TG_SENINF_TG_TM_EXP7_INT_ST_SHIFT 7
#define SENINF_TG_SENINF_TG_TM_EXP7_INT_ST_MASK (0x1 << 7)

#define SENINF_TG_SENINF_TG_TM_INT_TRIG 0x000c
#define SENINF_TG_SENINF_TG_TM_EXP0_INT_TRIG_SHIFT 0
#define SENINF_TG_SENINF_TG_TM_EXP0_INT_TRIG_MASK (0x1 << 0)
#define SENINF_TG_SENINF_TG_TM_EXP1_INT_TRIG_SHIFT 1
#define SENINF_TG_SENINF_TG_TM_EXP1_INT_TRIG_MASK (0x1 << 1)
#define SENINF_TG_SENINF_TG_TM_EXP2_INT_TRIG_SHIFT 2
#define SENINF_TG_SENINF_TG_TM_EXP2_INT_TRIG_MASK (0x1 << 2)
#define SENINF_TG_SENINF_TG_TM_EXP3_INT_TRIG_SHIFT 3
#define SENINF_TG_SENINF_TG_TM_EXP3_INT_TRIG_MASK (0x1 << 3)
#define SENINF_TG_SENINF_TG_TM_EXP4_INT_TRIG_SHIFT 4
#define SENINF_TG_SENINF_TG_TM_EXP4_INT_TRIG_MASK (0x1 << 4)
#define SENINF_TG_SENINF_TG_TM_EXP5_INT_TRIG_SHIFT 5
#define SENINF_TG_SENINF_TG_TM_EXP5_INT_TRIG_MASK (0x1 << 5)
#define SENINF_TG_SENINF_TG_TM_EXP6_INT_TRIG_SHIFT 6
#define SENINF_TG_SENINF_TG_TM_EXP6_INT_TRIG_MASK (0x1 << 6)
#define SENINF_TG_SENINF_TG_TM_EXP7_INT_TRIG_SHIFT 7
#define SENINF_TG_SENINF_TG_TM_EXP7_INT_TRIG_MASK (0x1 << 7)
#define SENINF_TG_SENINF_TG_TM_SELF_TRIG_EN_SHIFT 31
#define SENINF_TG_SENINF_TG_TM_SELF_TRIG_EN_MASK (0x1 << 31)

#define SENINF_TG_SENINF_TG_TM_CORE0_CTL 0x0010
#define SENINF_TG_SENINF_TG_TM_CORE0_EN_SHIFT 0
#define SENINF_TG_SENINF_TG_TM_CORE0_EN_MASK (0x1 << 0)
#define SENINF_TG_SENINF_TG_TM_CORE0_RST_SHIFT 1
#define SENINF_TG_SENINF_TG_TM_CORE0_RST_MASK (0x1 << 1)
#define SENINF_TG_SENINF_TG_TM_CORE0_FMT_SHIFT 2
#define SENINF_TG_SENINF_TG_TM_CORE0_FMT_MASK (0x1 << 2)
#define SENINF_TG_SENINF_TG_TM_CORE0_SINGLE_SHIFT 3
#define SENINF_TG_SENINF_TG_TM_CORE0_SINGLE_MASK (0x1 << 3)
#define SENINF_TG_SENINF_TG_TM_CORE0_DIFF_FRM_SHIFT 4
#define SENINF_TG_SENINF_TG_TM_CORE0_DIFF_FRM_MASK (0x1 << 4)
#define SENINF_TG_SENINF_TG_TM_CORE0_PAT_SHIFT 8
#define SENINF_TG_SENINF_TG_TM_CORE0_PAT_MASK (0xf << 8)
#define SENINF_TG_SENINF_TG_TM_CORE0_EXP_NUM_SHIFT 12
#define SENINF_TG_SENINF_TG_TM_CORE0_EXP_NUM_MASK (0x7 << 12)
#define SENINF_TG_SENINF_TG_TM_CORE0_CLK_CNT_SHIFT 16
#define SENINF_TG_SENINF_TG_TM_CORE0_CLK_CNT_MASK (0xff << 16)
#define SENINF_TG_SENINF_TG_TM_CORE0_PXL_MODE_SHIFT 24
#define SENINF_TG_SENINF_TG_TM_CORE0_PXL_MODE_MASK (0x3 << 24)
#define SENINF_TG_SENINF_TG_TM_CORE0_INTLV_NUM_SHIFT 26
#define SENINF_TG_SENINF_TG_TM_CORE0_INTLV_NUM_MASK (0x3 << 26)

#define SENINF_TG_SENINF_TG_TM_CORE1_CTL 0x0014
#define SENINF_TG_SENINF_TG_TM_CORE1_EN_SHIFT 0
#define SENINF_TG_SENINF_TG_TM_CORE1_EN_MASK (0x1 << 0)
#define SENINF_TG_SENINF_TG_TM_CORE1_RST_SHIFT 1
#define SENINF_TG_SENINF_TG_TM_CORE1_RST_MASK (0x1 << 1)
#define SENINF_TG_SENINF_TG_TM_CORE1_FMT_SHIFT 2
#define SENINF_TG_SENINF_TG_TM_CORE1_FMT_MASK (0x1 << 2)
#define SENINF_TG_SENINF_TG_TM_CORE1_SINGLE_SHIFT 3
#define SENINF_TG_SENINF_TG_TM_CORE1_SINGLE_MASK (0x1 << 3)
#define SENINF_TG_SENINF_TG_TM_CORE1_DIFF_FRM_SHIFT 4
#define SENINF_TG_SENINF_TG_TM_CORE1_DIFF_FRM_MASK (0x1 << 4)
#define SENINF_TG_SENINF_TG_TM_CORE1_PAT_SHIFT 8
#define SENINF_TG_SENINF_TG_TM_CORE1_PAT_MASK (0xf << 8)
#define SENINF_TG_SENINF_TG_TM_CORE1_EXP_NUM_SHIFT 12
#define SENINF_TG_SENINF_TG_TM_CORE1_EXP_NUM_MASK (0x7 << 12)
#define SENINF_TG_SENINF_TG_TM_CORE1_CLK_CNT_SHIFT 16
#define SENINF_TG_SENINF_TG_TM_CORE1_CLK_CNT_MASK (0xff << 16)
#define SENINF_TG_SENINF_TG_TM_CORE1_PXL_MODE_SHIFT 24
#define SENINF_TG_SENINF_TG_TM_CORE1_PXL_MODE_MASK (0x3 << 24)
#define SENINF_TG_SENINF_TG_TM_CORE1_INTLV_NUM_SHIFT 26
#define SENINF_TG_SENINF_TG_TM_CORE1_INTLV_NUM_MASK (0x3 << 26)

#define SENINF_TG_SENINF_TG_TM_SIZE 0x0018
#define SENINF_TG_SENINF_TG_TM_PXL_SHIFT 0
#define SENINF_TG_SENINF_TG_TM_PXL_MASK (0xffff << 0)
#define SENINF_TG_SENINF_TG_TM_LINE_SHIFT 16
#define SENINF_TG_SENINF_TG_TM_LINE_MASK (0xffff << 16)

#define SENINF_TG_SENINF_TG_TM_DUM 0x001c
#define SENINF_TG_SENINF_TG_TM_HB_SHIFT 0
#define SENINF_TG_SENINF_TG_TM_HB_MASK (0xffff << 0)
#define SENINF_TG_SENINF_TG_TM_VB_SHIFT 16
#define SENINF_TG_SENINF_TG_TM_VB_MASK (0xffff << 16)

#define SENINF_TG_SENINF_TG_TM_RAND_SEED 0x0020
#define SENINF_TG_SENINF_TG_TM_SEED_SHIFT 0
#define SENINF_TG_SENINF_TG_TM_SEED_MASK (0xffffffff << 0)

#define SENINF_TG_SENINF_TG_TM_STAGGER_CON0 0x0024
#define SENINF_TG_SENINF_TG_TM_EXP0_DT_SHIFT 0
#define SENINF_TG_SENINF_TG_TM_EXP0_DT_MASK (0x3f << 0)
#define SENINF_TG_SENINF_TG_TM_EXP0_VSYNC_VC_SHIFT 8
#define SENINF_TG_SENINF_TG_TM_EXP0_VSYNC_VC_MASK (0x1f << 8)
#define SENINF_TG_SENINF_TG_TM_EXP0_HSYNC_VC_SHIFT 16
#define SENINF_TG_SENINF_TG_TM_EXP0_HSYNC_VC_MASK (0x1f << 16)
#define SENINF_TG_SENINF_TG_TM_EXP0_REF_VSYNC_SHIFT 24
#define SENINF_TG_SENINF_TG_TM_EXP0_REF_VSYNC_MASK (0x7 << 24)

#define SENINF_TG_SENINF_TG_TM_STAGGER_CON1 0x0028
#define SENINF_TG_SENINF_TG_TM_EXP1_DT_SHIFT 0
#define SENINF_TG_SENINF_TG_TM_EXP1_DT_MASK (0x3f << 0)
#define SENINF_TG_SENINF_TG_TM_EXP1_VSYNC_VC_SHIFT 8
#define SENINF_TG_SENINF_TG_TM_EXP1_VSYNC_VC_MASK (0x1f << 8)
#define SENINF_TG_SENINF_TG_TM_EXP1_HSYNC_VC_SHIFT 16
#define SENINF_TG_SENINF_TG_TM_EXP1_HSYNC_VC_MASK (0x1f << 16)
#define SENINF_TG_SENINF_TG_TM_EXP1_REF_VSYNC_SHIFT 24
#define SENINF_TG_SENINF_TG_TM_EXP1_REF_VSYNC_MASK (0x7 << 24)

#define SENINF_TG_SENINF_TG_TM_STAGGER_CON2 0x002c
#define SENINF_TG_SENINF_TG_TM_EXP2_DT_SHIFT 0
#define SENINF_TG_SENINF_TG_TM_EXP2_DT_MASK (0x3f << 0)
#define SENINF_TG_SENINF_TG_TM_EXP2_VSYNC_VC_SHIFT 8
#define SENINF_TG_SENINF_TG_TM_EXP2_VSYNC_VC_MASK (0x1f << 8)
#define SENINF_TG_SENINF_TG_TM_EXP2_HSYNC_VC_SHIFT 16
#define SENINF_TG_SENINF_TG_TM_EXP2_HSYNC_VC_MASK (0x1f << 16)
#define SENINF_TG_SENINF_TG_TM_EXP2_REF_VSYNC_SHIFT 24
#define SENINF_TG_SENINF_TG_TM_EXP2_REF_VSYNC_MASK (0x7 << 24)

#define SENINF_TG_SENINF_TG_TM_STAGGER_CON3 0x0030
#define SENINF_TG_SENINF_TG_TM_EXP3_DT_SHIFT 0
#define SENINF_TG_SENINF_TG_TM_EXP3_DT_MASK (0x3f << 0)
#define SENINF_TG_SENINF_TG_TM_EXP3_VSYNC_VC_SHIFT 8
#define SENINF_TG_SENINF_TG_TM_EXP3_VSYNC_VC_MASK (0x1f << 8)
#define SENINF_TG_SENINF_TG_TM_EXP3_HSYNC_VC_SHIFT 16
#define SENINF_TG_SENINF_TG_TM_EXP3_HSYNC_VC_MASK (0x1f << 16)
#define SENINF_TG_SENINF_TG_TM_EXP3_REF_VSYNC_SHIFT 24
#define SENINF_TG_SENINF_TG_TM_EXP3_REF_VSYNC_MASK (0x7 << 24)

#define SENINF_TG_SENINF_TG_TM_STAGGER_CON4 0x0034
#define SENINF_TG_SENINF_TG_TM_EXP4_DT_SHIFT 0
#define SENINF_TG_SENINF_TG_TM_EXP4_DT_MASK (0x3f << 0)
#define SENINF_TG_SENINF_TG_TM_EXP4_VSYNC_VC_SHIFT 8
#define SENINF_TG_SENINF_TG_TM_EXP4_VSYNC_VC_MASK (0x1f << 8)
#define SENINF_TG_SENINF_TG_TM_EXP4_HSYNC_VC_SHIFT 16
#define SENINF_TG_SENINF_TG_TM_EXP4_HSYNC_VC_MASK (0x1f << 16)
#define SENINF_TG_SENINF_TG_TM_EXP4_REF_VSYNC_SHIFT 24
#define SENINF_TG_SENINF_TG_TM_EXP4_REF_VSYNC_MASK (0x7 << 24)

#define SENINF_TG_SENINF_TG_TM_STAGGER_CON5 0x0038
#define SENINF_TG_SENINF_TG_TM_EXP5_DT_SHIFT 0
#define SENINF_TG_SENINF_TG_TM_EXP5_DT_MASK (0x3f << 0)
#define SENINF_TG_SENINF_TG_TM_EXP5_VSYNC_VC_SHIFT 8
#define SENINF_TG_SENINF_TG_TM_EXP5_VSYNC_VC_MASK (0x1f << 8)
#define SENINF_TG_SENINF_TG_TM_EXP5_HSYNC_VC_SHIFT 16
#define SENINF_TG_SENINF_TG_TM_EXP5_HSYNC_VC_MASK (0x1f << 16)
#define SENINF_TG_SENINF_TG_TM_EXP5_REF_VSYNC_SHIFT 24
#define SENINF_TG_SENINF_TG_TM_EXP5_REF_VSYNC_MASK (0x7 << 24)

#define SENINF_TG_SENINF_TG_TM_STAGGER_CON6 0x003c
#define SENINF_TG_SENINF_TG_TM_EXP6_DT_SHIFT 0
#define SENINF_TG_SENINF_TG_TM_EXP6_DT_MASK (0x3f << 0)
#define SENINF_TG_SENINF_TG_TM_EXP6_VSYNC_VC_SHIFT 8
#define SENINF_TG_SENINF_TG_TM_EXP6_VSYNC_VC_MASK (0x1f << 8)
#define SENINF_TG_SENINF_TG_TM_EXP6_HSYNC_VC_SHIFT 16
#define SENINF_TG_SENINF_TG_TM_EXP6_HSYNC_VC_MASK (0x1f << 16)
#define SENINF_TG_SENINF_TG_TM_EXP6_REF_VSYNC_SHIFT 24
#define SENINF_TG_SENINF_TG_TM_EXP6_REF_VSYNC_MASK (0x7 << 24)

#define SENINF_TG_SENINF_TG_TM_STAGGER_CON7 0x0040
#define SENINF_TG_SENINF_TG_TM_EXP7_DT_SHIFT 0
#define SENINF_TG_SENINF_TG_TM_EXP7_DT_MASK (0x3f << 0)
#define SENINF_TG_SENINF_TG_TM_EXP7_VSYNC_VC_SHIFT 8
#define SENINF_TG_SENINF_TG_TM_EXP7_VSYNC_VC_MASK (0x1f << 8)
#define SENINF_TG_SENINF_TG_TM_EXP7_HSYNC_VC_SHIFT 16
#define SENINF_TG_SENINF_TG_TM_EXP7_HSYNC_VC_MASK (0x1f << 16)
#define SENINF_TG_SENINF_TG_TM_EXP7_REF_VSYNC_SHIFT 24
#define SENINF_TG_SENINF_TG_TM_EXP7_REF_VSYNC_MASK (0x7 << 24)

#define SENINF_TG_SENINF_TG_TM_EXP1_CTL 0x0044
#define SENINF_TG_SENINF_TG_TM_EXP1_WD_RATIO_SHIFT 0
#define SENINF_TG_SENINF_TG_TM_EXP1_WD_RATIO_MASK (0x1 << 0)
#define SENINF_TG_SENINF_TG_TM_EXP1_HT_RATIO_SHIFT 1
#define SENINF_TG_SENINF_TG_TM_EXP1_HT_RATIO_MASK (0x7 << 1)
#define SENINF_TG_SENINF_TG_TM_EXP1_DELAY_SHIFT 4
#define SENINF_TG_SENINF_TG_TM_EXP1_DELAY_MASK (0xffff << 4)
#define SENINF_TG_SENINF_TG_TM_EXP1_SLICE_SIZE_SHIFT 20
#define SENINF_TG_SENINF_TG_TM_EXP1_SLICE_SIZE_MASK (0x7 << 20)
#define SENINF_TG_SENINF_TG_TM_EXP1_SLICE_INTERVAL_SHIFT 23
#define SENINF_TG_SENINF_TG_TM_EXP1_SLICE_INTERVAL_MASK (0x1f << 23)
#define SENINF_TG_SENINF_TG_TM_EXP1_DATA_MODE_SHIFT 28
#define SENINF_TG_SENINF_TG_TM_EXP1_DATA_MODE_MASK (0x3 << 28)
#define SENINF_TG_SENINF_TG_TM_EXP1_VC_MODE_SHIFT 30
#define SENINF_TG_SENINF_TG_TM_EXP1_VC_MODE_MASK (0x3 << 30)

#define SENINF_TG_SENINF_TG_TM_EXP2_CTL 0x0048
#define SENINF_TG_SENINF_TG_TM_EXP2_WD_RATIO_SHIFT 0
#define SENINF_TG_SENINF_TG_TM_EXP2_WD_RATIO_MASK (0x1 << 0)
#define SENINF_TG_SENINF_TG_TM_EXP2_HT_RATIO_SHIFT 1
#define SENINF_TG_SENINF_TG_TM_EXP2_HT_RATIO_MASK (0x7 << 1)
#define SENINF_TG_SENINF_TG_TM_EXP2_DELAY_SHIFT 4
#define SENINF_TG_SENINF_TG_TM_EXP2_DELAY_MASK (0xffff << 4)
#define SENINF_TG_SENINF_TG_TM_EXP2_SLICE_SIZE_SHIFT 20
#define SENINF_TG_SENINF_TG_TM_EXP2_SLICE_SIZE_MASK (0x7 << 20)
#define SENINF_TG_SENINF_TG_TM_EXP2_SLICE_INTERVAL_SHIFT 23
#define SENINF_TG_SENINF_TG_TM_EXP2_SLICE_INTERVAL_MASK (0x1f << 23)
#define SENINF_TG_SENINF_TG_TM_EXP2_DATA_MODE_SHIFT 28
#define SENINF_TG_SENINF_TG_TM_EXP2_DATA_MODE_MASK (0x3 << 28)
#define SENINF_TG_SENINF_TG_TM_EXP2_VC_MODE_SHIFT 30
#define SENINF_TG_SENINF_TG_TM_EXP2_VC_MODE_MASK (0x3 << 30)

#define SENINF_TG_SENINF_TG_TM_EXP3_CTL 0x004c
#define SENINF_TG_SENINF_TG_TM_EXP3_WD_RATIO_SHIFT 0
#define SENINF_TG_SENINF_TG_TM_EXP3_WD_RATIO_MASK (0x1 << 0)
#define SENINF_TG_SENINF_TG_TM_EXP3_HT_RATIO_SHIFT 1
#define SENINF_TG_SENINF_TG_TM_EXP3_HT_RATIO_MASK (0x7 << 1)
#define SENINF_TG_SENINF_TG_TM_EXP3_DELAY_SHIFT 4
#define SENINF_TG_SENINF_TG_TM_EXP3_DELAY_MASK (0xffff << 4)
#define SENINF_TG_SENINF_TG_TM_EXP3_SLICE_SIZE_SHIFT 20
#define SENINF_TG_SENINF_TG_TM_EXP3_SLICE_SIZE_MASK (0x7 << 20)
#define SENINF_TG_SENINF_TG_TM_EXP3_SLICE_INTERVAL_SHIFT 23
#define SENINF_TG_SENINF_TG_TM_EXP3_SLICE_INTERVAL_MASK (0x1f << 23)
#define SENINF_TG_SENINF_TG_TM_EXP3_DATA_MODE_SHIFT 28
#define SENINF_TG_SENINF_TG_TM_EXP3_DATA_MODE_MASK (0x3 << 28)
#define SENINF_TG_SENINF_TG_TM_EXP3_VC_MODE_SHIFT 30
#define SENINF_TG_SENINF_TG_TM_EXP3_VC_MODE_MASK (0x3 << 30)

#define SENINF_TG_SENINF_TG_TM_EXP4_CTL 0x0050
#define SENINF_TG_SENINF_TG_TM_EXP4_WD_RATIO_SHIFT 0
#define SENINF_TG_SENINF_TG_TM_EXP4_WD_RATIO_MASK (0x1 << 0)
#define SENINF_TG_SENINF_TG_TM_EXP4_HT_RATIO_SHIFT 1
#define SENINF_TG_SENINF_TG_TM_EXP4_HT_RATIO_MASK (0x7 << 1)
#define SENINF_TG_SENINF_TG_TM_EXP4_DELAY_SHIFT 4
#define SENINF_TG_SENINF_TG_TM_EXP4_DELAY_MASK (0xffff << 4)
#define SENINF_TG_SENINF_TG_TM_EXP4_SLICE_SIZE_SHIFT 20
#define SENINF_TG_SENINF_TG_TM_EXP4_SLICE_SIZE_MASK (0x7 << 20)
#define SENINF_TG_SENINF_TG_TM_EXP4_SLICE_INTERVAL_SHIFT 23
#define SENINF_TG_SENINF_TG_TM_EXP4_SLICE_INTERVAL_MASK (0x1f << 23)
#define SENINF_TG_SENINF_TG_TM_EXP4_DATA_MODE_SHIFT 28
#define SENINF_TG_SENINF_TG_TM_EXP4_DATA_MODE_MASK (0x3 << 28)
#define SENINF_TG_SENINF_TG_TM_EXP4_VC_MODE_SHIFT 30
#define SENINF_TG_SENINF_TG_TM_EXP4_VC_MODE_MASK (0x3 << 30)

#define SENINF_TG_SENINF_TG_TM_EXP5_CTL 0x0054
#define SENINF_TG_SENINF_TG_TM_EXP5_WD_RATIO_SHIFT 0
#define SENINF_TG_SENINF_TG_TM_EXP5_WD_RATIO_MASK (0x1 << 0)
#define SENINF_TG_SENINF_TG_TM_EXP5_HT_RATIO_SHIFT 1
#define SENINF_TG_SENINF_TG_TM_EXP5_HT_RATIO_MASK (0x7 << 1)
#define SENINF_TG_SENINF_TG_TM_EXP5_DELAY_SHIFT 4
#define SENINF_TG_SENINF_TG_TM_EXP5_DELAY_MASK (0xffff << 4)
#define SENINF_TG_SENINF_TG_TM_EXP5_SLICE_SIZE_SHIFT 20
#define SENINF_TG_SENINF_TG_TM_EXP5_SLICE_SIZE_MASK (0x7 << 20)
#define SENINF_TG_SENINF_TG_TM_EXP5_SLICE_INTERVAL_SHIFT 23
#define SENINF_TG_SENINF_TG_TM_EXP5_SLICE_INTERVAL_MASK (0x1f << 23)
#define SENINF_TG_SENINF_TG_TM_EXP5_DATA_MODE_SHIFT 28
#define SENINF_TG_SENINF_TG_TM_EXP5_DATA_MODE_MASK (0x3 << 28)
#define SENINF_TG_SENINF_TG_TM_EXP5_VC_MODE_SHIFT 30
#define SENINF_TG_SENINF_TG_TM_EXP5_VC_MODE_MASK (0x3 << 30)

#define SENINF_TG_SENINF_TG_TM_EXP6_CTL 0x0058
#define SENINF_TG_SENINF_TG_TM_EXP6_WD_RATIO_SHIFT 0
#define SENINF_TG_SENINF_TG_TM_EXP6_WD_RATIO_MASK (0x1 << 0)
#define SENINF_TG_SENINF_TG_TM_EXP6_HT_RATIO_SHIFT 1
#define SENINF_TG_SENINF_TG_TM_EXP6_HT_RATIO_MASK (0x7 << 1)
#define SENINF_TG_SENINF_TG_TM_EXP6_DELAY_SHIFT 4
#define SENINF_TG_SENINF_TG_TM_EXP6_DELAY_MASK (0xffff << 4)
#define SENINF_TG_SENINF_TG_TM_EXP6_SLICE_SIZE_SHIFT 20
#define SENINF_TG_SENINF_TG_TM_EXP6_SLICE_SIZE_MASK (0x7 << 20)
#define SENINF_TG_SENINF_TG_TM_EXP6_SLICE_INTERVAL_SHIFT 23
#define SENINF_TG_SENINF_TG_TM_EXP6_SLICE_INTERVAL_MASK (0x1f << 23)
#define SENINF_TG_SENINF_TG_TM_EXP6_DATA_MODE_SHIFT 28
#define SENINF_TG_SENINF_TG_TM_EXP6_DATA_MODE_MASK (0x3 << 28)
#define SENINF_TG_SENINF_TG_TM_EXP6_VC_MODE_SHIFT 30
#define SENINF_TG_SENINF_TG_TM_EXP6_VC_MODE_MASK (0x3 << 30)

#define SENINF_TG_SENINF_TG_TM_EXP7_CTL 0x005c
#define SENINF_TG_SENINF_TG_TM_EXP7_WD_RATIO_SHIFT 0
#define SENINF_TG_SENINF_TG_TM_EXP7_WD_RATIO_MASK (0x1 << 0)
#define SENINF_TG_SENINF_TG_TM_EXP7_HT_RATIO_SHIFT 1
#define SENINF_TG_SENINF_TG_TM_EXP7_HT_RATIO_MASK (0x7 << 1)
#define SENINF_TG_SENINF_TG_TM_EXP7_DELAY_SHIFT 4
#define SENINF_TG_SENINF_TG_TM_EXP7_DELAY_MASK (0xffff << 4)
#define SENINF_TG_SENINF_TG_TM_EXP7_SLICE_SIZE_SHIFT 20
#define SENINF_TG_SENINF_TG_TM_EXP7_SLICE_SIZE_MASK (0x7 << 20)
#define SENINF_TG_SENINF_TG_TM_EXP7_SLICE_INTERVAL_SHIFT 23
#define SENINF_TG_SENINF_TG_TM_EXP7_SLICE_INTERVAL_MASK (0x1f << 23)
#define SENINF_TG_SENINF_TG_TM_EXP7_DATA_MODE_SHIFT 28
#define SENINF_TG_SENINF_TG_TM_EXP7_DATA_MODE_MASK (0x3 << 28)
#define SENINF_TG_SENINF_TG_TM_EXP7_VC_MODE_SHIFT 30
#define SENINF_TG_SENINF_TG_TM_EXP7_VC_MODE_MASK (0x3 << 30)

#endif
