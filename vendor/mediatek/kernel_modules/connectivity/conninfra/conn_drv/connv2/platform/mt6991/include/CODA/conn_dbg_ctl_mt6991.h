/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2023 MediaTek Inc.
 */
//[File]            : conn_dbg_ctl.h
//[Revision time]   : Thu Aug 31 15:25:20 2023

#ifndef __CONN_DBG_CTL_REGS_H__
#define __CONN_DBG_CTL_REGS_H__

#ifdef __cplusplus
extern "C" {
#endif


//****************************************************************************
//
//                     CONN_DBG_CTL CR Definitions                     
//
//****************************************************************************

#define CONN_DBG_CTL_BASE                                      (CONN_REG_CONN_INFRA_DBG_CTL_ADDR_MT6991) // 0x40023000

#define CONN_DBG_CTL_CLOCK_DETECT_ADDR                         (CONN_DBG_CTL_BASE + 0x000) // 3000
#define CONN_DBG_CTL_CONN_INFRA_MONFLAG_OUT_ADDR               (CONN_DBG_CTL_BASE + 0x200) // 3200
#define CONN_DBG_CTL_CONN_INFRA_IO_TOP_DBG_SEL_ADDR            (CONN_DBG_CTL_BASE + 0x204) // 3204
#define CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_ADDR           (CONN_DBG_CTL_BASE + 0x400) // 3400
#define CONN_DBG_CTL_CONN_INFRA_OFF_BUS_DBG_OUT_ADDR           (CONN_DBG_CTL_BASE + 0x404) // 3404
#define CONN_DBG_CTL_CONN_INFRA_OFF_BUS_DBG_SEL_ADDR           (CONN_DBG_CTL_BASE + 0x408) // 3408
#define CONN_DBG_CTL_CONN_INFRA_OFF_DEBUGSYS_CTRL_ADDR         (CONN_DBG_CTL_BASE + 0x40C) // 340C
#define CONN_DBG_CTL_CONN_VON_BUS_APB_TIMEOUT_INFO_ADDR        (CONN_DBG_CTL_BASE + 0x410) // 3410
#define CONN_DBG_CTL_CONN_VON_BUS_APB_TIMEOUT_ADDR_ADDR        (CONN_DBG_CTL_BASE + 0x414) // 3414
#define CONN_DBG_CTL_CONN_VON_BUS_APB_TIMEOUT_WDATA_ADDR       (CONN_DBG_CTL_BASE + 0x418) // 3418
#define CONN_DBG_CTL_CONN_INFRA_VON_BUS_DEBUG_INFO_ADDR        (CONN_DBG_CTL_BASE + 0x41C) // 341C
#define CONN_DBG_CTL_WF_MONFLAG_OFF_OUT_ADDR                   (CONN_DBG_CTL_BASE + 0x600) // 3600
#define CONN_DBG_CTL_WF_MCU_DBGOUT_SEL_ADDR                    (CONN_DBG_CTL_BASE + 0x604) // 3604
#define CONN_DBG_CTL_WF_MCU_GPR_BUS_DBGOUT_LOG_ADDR            (CONN_DBG_CTL_BASE + 0x608) // 3608
#define CONN_DBG_CTL_WF_MCU_DBG_PC_LOG_SEL_ADDR                (CONN_DBG_CTL_BASE + 0x60C) // 360C
#define CONN_DBG_CTL_WF_MCU_DBG_PC_LOG_ADDR                    (CONN_DBG_CTL_BASE + 0x610) // 3610
#define CONN_DBG_CTL_WF_MCU_DBG_GPR_LOG_SEL_ADDR               (CONN_DBG_CTL_BASE + 0x614) // 3614
#define CONN_DBG_CTL_WF_MCU_DBG_EN_FR_HIF_ADDR                 (CONN_DBG_CTL_BASE + 0x618) // 3618
#define CONN_DBG_CTL_WF_MCU_DBG_SEL_FR_HIF_ADDR                (CONN_DBG_CTL_BASE + 0x61C) // 361C
#define CONN_DBG_CTL_WF_CORE_PC_INDEX_FR_HIF_ADDR              (CONN_DBG_CTL_BASE + 0x620) // 3620
#define CONN_DBG_CTL_WF_MCUSYS_INFRA_VDNR_GEN_DEBUG_CTRL_AO_DEBUGSYS_CTRL_ADDR (CONN_DBG_CTL_BASE + 0x628) // 3628
#define CONN_DBG_CTL_WF_MCUSYS_INFRA_VDNR_GEN_DEBUG_CTRL_AO_BUS_TIMEOUT_IRQ_ADDR (CONN_DBG_CTL_BASE + 0x62C) // 362C
#define CONN_DBG_CTL_BGF_MONFLAG_OFF_OUT_ADDR                  (CONN_DBG_CTL_BASE + 0xA00) // 3A00
#define CONN_DBG_CTL_CR_DBGCTL2BGF_OFF_DEBUG_SEL_ADDR          (CONN_DBG_CTL_BASE + 0xA04) // 3A04
#define CONN_DBG_CTL_GPSAON2HOST_DEBUG_ADDR                    (CONN_DBG_CTL_BASE + 0xA08) // 3A08
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_ADDR                   (CONN_DBG_CTL_BASE + 0xC00) // 3C00
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_ADDR                  (CONN_DBG_CTL_BASE + 0xC10) // 3C10
#define CONN_DBG_CTL_DBG_BACKUP_0_ADDR                         (CONN_DBG_CTL_BASE + 0xE00) // 3E00
#define CONN_DBG_CTL_DBG_BACKUP_1_ADDR                         (CONN_DBG_CTL_BASE + 0xE04) // 3E04
#define CONN_DBG_CTL_DBG_BACKUP_2_ADDR                         (CONN_DBG_CTL_BASE + 0xE08) // 3E08
#define CONN_DBG_CTL_DBG_BACKUP_3_ADDR                         (CONN_DBG_CTL_BASE + 0xE0C) // 3E0C
#define CONN_DBG_CTL_DBG_BACKUP_4_ADDR                         (CONN_DBG_CTL_BASE + 0xE10) // 3E10
#define CONN_DBG_CTL_DBG_BACKUP_5_ADDR                         (CONN_DBG_CTL_BASE + 0xE14) // 3E14
#define CONN_DBG_CTL_DBG_BACKUP_6_ADDR                         (CONN_DBG_CTL_BASE + 0xE18) // 3E18
#define CONN_DBG_CTL_DBG_BACKUP_7_ADDR                         (CONN_DBG_CTL_BASE + 0xE1C) // 3E1C
#define CONN_DBG_CTL_CR_CONN_TEST_DO_SEL_ADDR                  (CONN_DBG_CTL_BASE + 0xE20) // 3E20
#define CONN_DBG_CTL_CR_AP2WF_HOST_ON_CFG_ADDR                 (CONN_DBG_CTL_BASE + 0xE24) // 3E24
#define CONN_DBG_CTL_DBG_CTL_LAST_CR_ADDR                      (CONN_DBG_CTL_BASE + 0xFFC) // 3FFC




/* =====================================================================================

  ---CLOCK_DETECT (0x18023000 + 0x000)---

    CLK_DETECT_BUS_CLR_PULSE[0]  - (WO) Host set this bit to reset bus detect state
                                     This cr will generate a reset signal for bus clock detecion state
                                     After write 1, you have to write 0 to release reset
    HCLK_FR_CK_DETECT[1]         - (RO) Connsys bus hclk detection status
                                     0 : bus hclk non-alive
                                     1 : bus hclk alive
                                     User could write 1 to clear detection status.
                                     After write 1 clear status and then read back to check clock status. It can check bus hclk alive or not.
    OSC_CLK_DETECT[2]            - (RO) Connsys osc clock detection status
                                     0 : osc clock non-alive
                                     1 : osc clock alive
                                     User could write 1 to clear detection status.
                                     After write 1 clear status and then read back to check clock status. It can check osc clock alive or not.
    RESERVED3[31..3]             - (RO) Reserved bits

 =====================================================================================*/
#define CONN_DBG_CTL_CLOCK_DETECT_OSC_CLK_DETECT_ADDR          CONN_DBG_CTL_CLOCK_DETECT_ADDR
#define CONN_DBG_CTL_CLOCK_DETECT_OSC_CLK_DETECT_MASK          0x00000004                // OSC_CLK_DETECT[2]
#define CONN_DBG_CTL_CLOCK_DETECT_OSC_CLK_DETECT_SHFT          2
#define CONN_DBG_CTL_CLOCK_DETECT_HCLK_FR_CK_DETECT_ADDR       CONN_DBG_CTL_CLOCK_DETECT_ADDR
#define CONN_DBG_CTL_CLOCK_DETECT_HCLK_FR_CK_DETECT_MASK       0x00000002                // HCLK_FR_CK_DETECT[1]
#define CONN_DBG_CTL_CLOCK_DETECT_HCLK_FR_CK_DETECT_SHFT       1
#define CONN_DBG_CTL_CLOCK_DETECT_CLK_DETECT_BUS_CLR_PULSE_ADDR CONN_DBG_CTL_CLOCK_DETECT_ADDR
#define CONN_DBG_CTL_CLOCK_DETECT_CLK_DETECT_BUS_CLR_PULSE_MASK 0x00000001                // CLK_DETECT_BUS_CLR_PULSE[0]
#define CONN_DBG_CTL_CLOCK_DETECT_CLK_DETECT_BUS_CLR_PULSE_SHFT 0

/* =====================================================================================

  ---CONN_INFRA_MONFLAG_OUT (0x18023000 + 0x200)---

    CONN_INFRA_MONFLAG_OUT[31..0] - (RO)  xxx 

 =====================================================================================*/
#define CONN_DBG_CTL_CONN_INFRA_MONFLAG_OUT_CONN_INFRA_MONFLAG_OUT_ADDR CONN_DBG_CTL_CONN_INFRA_MONFLAG_OUT_ADDR
#define CONN_DBG_CTL_CONN_INFRA_MONFLAG_OUT_CONN_INFRA_MONFLAG_OUT_MASK 0xFFFFFFFF                // CONN_INFRA_MONFLAG_OUT[31..0]
#define CONN_DBG_CTL_CONN_INFRA_MONFLAG_OUT_CONN_INFRA_MONFLAG_OUT_SHFT 0

/* =====================================================================================

  ---CONN_INFRA_IO_TOP_DBG_SEL (0x18023000 + 0x204)---

    CONN_INFRA_IO_TOP_DBG_SEL[31..0] - (RW)  xxx 

 =====================================================================================*/
#define CONN_DBG_CTL_CONN_INFRA_IO_TOP_DBG_SEL_CONN_INFRA_IO_TOP_DBG_SEL_ADDR CONN_DBG_CTL_CONN_INFRA_IO_TOP_DBG_SEL_ADDR
#define CONN_DBG_CTL_CONN_INFRA_IO_TOP_DBG_SEL_CONN_INFRA_IO_TOP_DBG_SEL_MASK 0xFFFFFFFF                // CONN_INFRA_IO_TOP_DBG_SEL[31..0]
#define CONN_DBG_CTL_CONN_INFRA_IO_TOP_DBG_SEL_CONN_INFRA_IO_TOP_DBG_SEL_SHFT 0

/* =====================================================================================

  ---CONN_INFRA_BUS_TIMEOUT_IRQ (0x18023000 + 0x400)---

    CONN_INFRA_OFF_BUS_TIMEOUT[0] - (RO) merge "CONN_INFRA_OFF_C6_TIMEOUT" & "CONN_INFRA_OFF_CONN2AP_TIMEOUT"
    CONN_INFRA_OFF_VDNR_TIMEOUT_IRQ[1] - (RO) conn_infra_off_bus timeout (VDNR gen)
    CONN_INFRA_OFF_AXI_LAYER_TIMEOUT[2] - (RO) conn_infra_off_axi_layer_bus timeout (hand-code)
    CONN_INFRA_OFF_CONN2AP_TIMEOUT[3] - (RO) conn_infra_off_bus c12 AHB decoder timeout (hand-code)
    CONN_INFRA_OFF_C6_TIMEOUT[4] - (RO) conn_infra_off_bus c6 AHB decoder timeout (hand-code)
    CONN_VON_BUS_TIMEOUT[5]      - (RO) conn_von_bus timeout (hand-code)
    CONN_INFRA_ON_BUS_TIMEOUT_IRQ[6] - (RO) conn_infra_on_bus timeout (hand-code)
    RESERVED7[31..7]             - (RO) Reserved bits

 =====================================================================================*/
#define CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_CONN_INFRA_ON_BUS_TIMEOUT_IRQ_ADDR CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_ADDR
#define CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_CONN_INFRA_ON_BUS_TIMEOUT_IRQ_MASK 0x00000040                // CONN_INFRA_ON_BUS_TIMEOUT_IRQ[6]
#define CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_CONN_INFRA_ON_BUS_TIMEOUT_IRQ_SHFT 6
#define CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_CONN_VON_BUS_TIMEOUT_ADDR CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_ADDR
#define CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_CONN_VON_BUS_TIMEOUT_MASK 0x00000020                // CONN_VON_BUS_TIMEOUT[5]
#define CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_CONN_VON_BUS_TIMEOUT_SHFT 5
#define CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_CONN_INFRA_OFF_C6_TIMEOUT_ADDR CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_ADDR
#define CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_CONN_INFRA_OFF_C6_TIMEOUT_MASK 0x00000010                // CONN_INFRA_OFF_C6_TIMEOUT[4]
#define CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_CONN_INFRA_OFF_C6_TIMEOUT_SHFT 4
#define CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_CONN_INFRA_OFF_CONN2AP_TIMEOUT_ADDR CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_ADDR
#define CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_CONN_INFRA_OFF_CONN2AP_TIMEOUT_MASK 0x00000008                // CONN_INFRA_OFF_CONN2AP_TIMEOUT[3]
#define CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_CONN_INFRA_OFF_CONN2AP_TIMEOUT_SHFT 3
#define CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_CONN_INFRA_OFF_AXI_LAYER_TIMEOUT_ADDR CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_ADDR
#define CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_CONN_INFRA_OFF_AXI_LAYER_TIMEOUT_MASK 0x00000004                // CONN_INFRA_OFF_AXI_LAYER_TIMEOUT[2]
#define CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_CONN_INFRA_OFF_AXI_LAYER_TIMEOUT_SHFT 2
#define CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_CONN_INFRA_OFF_VDNR_TIMEOUT_IRQ_ADDR CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_ADDR
#define CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_CONN_INFRA_OFF_VDNR_TIMEOUT_IRQ_MASK 0x00000002                // CONN_INFRA_OFF_VDNR_TIMEOUT_IRQ[1]
#define CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_CONN_INFRA_OFF_VDNR_TIMEOUT_IRQ_SHFT 1
#define CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_CONN_INFRA_OFF_BUS_TIMEOUT_ADDR CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_ADDR
#define CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_CONN_INFRA_OFF_BUS_TIMEOUT_MASK 0x00000001                // CONN_INFRA_OFF_BUS_TIMEOUT[0]
#define CONN_DBG_CTL_CONN_INFRA_BUS_TIMEOUT_IRQ_CONN_INFRA_OFF_BUS_TIMEOUT_SHFT 0

/* =====================================================================================

  ---CONN_INFRA_OFF_BUS_DBG_OUT (0x18023000 + 0x404)---

    CONN_INFRA_OFF_BUS_DBG_OUT[31..0] - (RO) conn_infra_off_bus debug output (idle, gals_dbg, timeout_info)

 =====================================================================================*/
#define CONN_DBG_CTL_CONN_INFRA_OFF_BUS_DBG_OUT_CONN_INFRA_OFF_BUS_DBG_OUT_ADDR CONN_DBG_CTL_CONN_INFRA_OFF_BUS_DBG_OUT_ADDR
#define CONN_DBG_CTL_CONN_INFRA_OFF_BUS_DBG_OUT_CONN_INFRA_OFF_BUS_DBG_OUT_MASK 0xFFFFFFFF                // CONN_INFRA_OFF_BUS_DBG_OUT[31..0]
#define CONN_DBG_CTL_CONN_INFRA_OFF_BUS_DBG_OUT_CONN_INFRA_OFF_BUS_DBG_OUT_SHFT 0

/* =====================================================================================

  ---CONN_INFRA_OFF_BUS_DBG_SEL (0x18023000 + 0x408)---

    CONN_INFRA_OFF_BUS_DBG_SEL[4..0] - (RW) conn_infra_off_bus debug selection
    RESERVED5[31..5]             - (RO) Reserved bits

 =====================================================================================*/
#define CONN_DBG_CTL_CONN_INFRA_OFF_BUS_DBG_SEL_CONN_INFRA_OFF_BUS_DBG_SEL_ADDR CONN_DBG_CTL_CONN_INFRA_OFF_BUS_DBG_SEL_ADDR
#define CONN_DBG_CTL_CONN_INFRA_OFF_BUS_DBG_SEL_CONN_INFRA_OFF_BUS_DBG_SEL_MASK 0x0000001F                // CONN_INFRA_OFF_BUS_DBG_SEL[4..0]
#define CONN_DBG_CTL_CONN_INFRA_OFF_BUS_DBG_SEL_CONN_INFRA_OFF_BUS_DBG_SEL_SHFT 0

/* =====================================================================================

  ---CONN_INFRA_OFF_DEBUGSYS_CTRL (0x18023000 + 0x40C)---

    CONN_INFRA_OFF_DEBUGSYS_CTRL[31..0] - (RW) latch information when conn_von_bus timeout

 =====================================================================================*/
#define CONN_DBG_CTL_CONN_INFRA_OFF_DEBUGSYS_CTRL_CONN_INFRA_OFF_DEBUGSYS_CTRL_ADDR CONN_DBG_CTL_CONN_INFRA_OFF_DEBUGSYS_CTRL_ADDR
#define CONN_DBG_CTL_CONN_INFRA_OFF_DEBUGSYS_CTRL_CONN_INFRA_OFF_DEBUGSYS_CTRL_MASK 0xFFFFFFFF                // CONN_INFRA_OFF_DEBUGSYS_CTRL[31..0]
#define CONN_DBG_CTL_CONN_INFRA_OFF_DEBUGSYS_CTRL_CONN_INFRA_OFF_DEBUGSYS_CTRL_SHFT 0

/* =====================================================================================

  ---CONN_VON_BUS_APB_TIMEOUT_INFO (0x18023000 + 0x410)---

    CONN_VON_BUS_APB_TIMEOUT_INFO[31..0] - (RO) latch information when conn_von_bus timeout

 =====================================================================================*/
#define CONN_DBG_CTL_CONN_VON_BUS_APB_TIMEOUT_INFO_CONN_VON_BUS_APB_TIMEOUT_INFO_ADDR CONN_DBG_CTL_CONN_VON_BUS_APB_TIMEOUT_INFO_ADDR
#define CONN_DBG_CTL_CONN_VON_BUS_APB_TIMEOUT_INFO_CONN_VON_BUS_APB_TIMEOUT_INFO_MASK 0xFFFFFFFF                // CONN_VON_BUS_APB_TIMEOUT_INFO[31..0]
#define CONN_DBG_CTL_CONN_VON_BUS_APB_TIMEOUT_INFO_CONN_VON_BUS_APB_TIMEOUT_INFO_SHFT 0

/* =====================================================================================

  ---CONN_VON_BUS_APB_TIMEOUT_ADDR (0x18023000 + 0x414)---

    CONN_VON_BUS_APB_TIMEOUT_ADDR[31..0] - (RO) latch information when conn_von_bus timeout

 =====================================================================================*/
#define CONN_DBG_CTL_CONN_VON_BUS_APB_TIMEOUT_ADDR_CONN_VON_BUS_APB_TIMEOUT_ADDR_ADDR CONN_DBG_CTL_CONN_VON_BUS_APB_TIMEOUT_ADDR_ADDR
#define CONN_DBG_CTL_CONN_VON_BUS_APB_TIMEOUT_ADDR_CONN_VON_BUS_APB_TIMEOUT_ADDR_MASK 0xFFFFFFFF                // CONN_VON_BUS_APB_TIMEOUT_ADDR[31..0]
#define CONN_DBG_CTL_CONN_VON_BUS_APB_TIMEOUT_ADDR_CONN_VON_BUS_APB_TIMEOUT_ADDR_SHFT 0

/* =====================================================================================

  ---CONN_VON_BUS_APB_TIMEOUT_WDATA (0x18023000 + 0x418)---

    CONN_VON_BUS_APB_TIMEOUT_WDATA[31..0] - (RO) latch information when conn_von_bus timeout

 =====================================================================================*/
#define CONN_DBG_CTL_CONN_VON_BUS_APB_TIMEOUT_WDATA_CONN_VON_BUS_APB_TIMEOUT_WDATA_ADDR CONN_DBG_CTL_CONN_VON_BUS_APB_TIMEOUT_WDATA_ADDR
#define CONN_DBG_CTL_CONN_VON_BUS_APB_TIMEOUT_WDATA_CONN_VON_BUS_APB_TIMEOUT_WDATA_MASK 0xFFFFFFFF                // CONN_VON_BUS_APB_TIMEOUT_WDATA[31..0]
#define CONN_DBG_CTL_CONN_VON_BUS_APB_TIMEOUT_WDATA_CONN_VON_BUS_APB_TIMEOUT_WDATA_SHFT 0

/* =====================================================================================

  ---CONN_INFRA_VON_BUS_DEBUG_INFO (0x18023000 + 0x41C)---

    CONN_INFRA_VON_BUS_DEBUG_INFO[31..0] - (RO) conn_von_bus debug information

 =====================================================================================*/
#define CONN_DBG_CTL_CONN_INFRA_VON_BUS_DEBUG_INFO_CONN_INFRA_VON_BUS_DEBUG_INFO_ADDR CONN_DBG_CTL_CONN_INFRA_VON_BUS_DEBUG_INFO_ADDR
#define CONN_DBG_CTL_CONN_INFRA_VON_BUS_DEBUG_INFO_CONN_INFRA_VON_BUS_DEBUG_INFO_MASK 0xFFFFFFFF                // CONN_INFRA_VON_BUS_DEBUG_INFO[31..0]
#define CONN_DBG_CTL_CONN_INFRA_VON_BUS_DEBUG_INFO_CONN_INFRA_VON_BUS_DEBUG_INFO_SHFT 0

/* =====================================================================================

  ---WF_MONFLAG_OFF_OUT (0x18023000 + 0x600)---

    WF_MONFLAG_OFF_OUT[31..0]    - (RO)  xxx 

 =====================================================================================*/
#define CONN_DBG_CTL_WF_MONFLAG_OFF_OUT_WF_MONFLAG_OFF_OUT_ADDR CONN_DBG_CTL_WF_MONFLAG_OFF_OUT_ADDR
#define CONN_DBG_CTL_WF_MONFLAG_OFF_OUT_WF_MONFLAG_OFF_OUT_MASK 0xFFFFFFFF                // WF_MONFLAG_OFF_OUT[31..0]
#define CONN_DBG_CTL_WF_MONFLAG_OFF_OUT_WF_MONFLAG_OFF_OUT_SHFT 0

/* =====================================================================================

  ---WF_MCU_DBGOUT_SEL (0x18023000 + 0x604)---

    WF_MCU_DBGOUT_SEL[2..0]      - (RW)  xxx 
    RESERVED3[31..3]             - (RO) Reserved bits

 =====================================================================================*/
#define CONN_DBG_CTL_WF_MCU_DBGOUT_SEL_WF_MCU_DBGOUT_SEL_ADDR  CONN_DBG_CTL_WF_MCU_DBGOUT_SEL_ADDR
#define CONN_DBG_CTL_WF_MCU_DBGOUT_SEL_WF_MCU_DBGOUT_SEL_MASK  0x00000007                // WF_MCU_DBGOUT_SEL[2..0]
#define CONN_DBG_CTL_WF_MCU_DBGOUT_SEL_WF_MCU_DBGOUT_SEL_SHFT  0

/* =====================================================================================

  ---WF_MCU_GPR_BUS_DBGOUT_LOG (0x18023000 + 0x608)---

    WF_MCU_GPR_BUS_DBGOUT_LOG[31..0] - (RO) select by "WF_MCU_DBGOUT_SEL"

 =====================================================================================*/
#define CONN_DBG_CTL_WF_MCU_GPR_BUS_DBGOUT_LOG_WF_MCU_GPR_BUS_DBGOUT_LOG_ADDR CONN_DBG_CTL_WF_MCU_GPR_BUS_DBGOUT_LOG_ADDR
#define CONN_DBG_CTL_WF_MCU_GPR_BUS_DBGOUT_LOG_WF_MCU_GPR_BUS_DBGOUT_LOG_MASK 0xFFFFFFFF                // WF_MCU_GPR_BUS_DBGOUT_LOG[31..0]
#define CONN_DBG_CTL_WF_MCU_GPR_BUS_DBGOUT_LOG_WF_MCU_GPR_BUS_DBGOUT_LOG_SHFT 0

/* =====================================================================================

  ---WF_MCU_DBG_PC_LOG_SEL (0x18023000 + 0x60C)---

    WF_MCU_DBG_PC_LOG_SEL[5..0]  - (RW)  xxx 
    RESERVED6[31..6]             - (RO) Reserved bits

 =====================================================================================*/
#define CONN_DBG_CTL_WF_MCU_DBG_PC_LOG_SEL_WF_MCU_DBG_PC_LOG_SEL_ADDR CONN_DBG_CTL_WF_MCU_DBG_PC_LOG_SEL_ADDR
#define CONN_DBG_CTL_WF_MCU_DBG_PC_LOG_SEL_WF_MCU_DBG_PC_LOG_SEL_MASK 0x0000003F                // WF_MCU_DBG_PC_LOG_SEL[5..0]
#define CONN_DBG_CTL_WF_MCU_DBG_PC_LOG_SEL_WF_MCU_DBG_PC_LOG_SEL_SHFT 0

/* =====================================================================================

  ---WF_MCU_DBG_PC_LOG (0x18023000 + 0x610)---

    WF_MCU_DBG_PC_LOG[31..0]     - (RO) select by "WF_MCU_DBG_PC_LOG_SEL"

 =====================================================================================*/
#define CONN_DBG_CTL_WF_MCU_DBG_PC_LOG_WF_MCU_DBG_PC_LOG_ADDR  CONN_DBG_CTL_WF_MCU_DBG_PC_LOG_ADDR
#define CONN_DBG_CTL_WF_MCU_DBG_PC_LOG_WF_MCU_DBG_PC_LOG_MASK  0xFFFFFFFF                // WF_MCU_DBG_PC_LOG[31..0]
#define CONN_DBG_CTL_WF_MCU_DBG_PC_LOG_WF_MCU_DBG_PC_LOG_SHFT  0

/* =====================================================================================

  ---WF_MCU_DBG_GPR_LOG_SEL (0x18023000 + 0x614)---

    WF_MCU_DBG_GPR_LOG_SEL[5..0] - (RW)  xxx 
    RESERVED6[31..6]             - (RO) Reserved bits

 =====================================================================================*/
#define CONN_DBG_CTL_WF_MCU_DBG_GPR_LOG_SEL_WF_MCU_DBG_GPR_LOG_SEL_ADDR CONN_DBG_CTL_WF_MCU_DBG_GPR_LOG_SEL_ADDR
#define CONN_DBG_CTL_WF_MCU_DBG_GPR_LOG_SEL_WF_MCU_DBG_GPR_LOG_SEL_MASK 0x0000003F                // WF_MCU_DBG_GPR_LOG_SEL[5..0]
#define CONN_DBG_CTL_WF_MCU_DBG_GPR_LOG_SEL_WF_MCU_DBG_GPR_LOG_SEL_SHFT 0

/* =====================================================================================

  ---WF_MCU_DBG_EN_FR_HIF (0x18023000 + 0x618)---

    WF_MCU_DBG_EN_FR_HIF[0]      - (RW)  xxx 
    RESERVED1[31..1]             - (RO) Reserved bits

 =====================================================================================*/
#define CONN_DBG_CTL_WF_MCU_DBG_EN_FR_HIF_WF_MCU_DBG_EN_FR_HIF_ADDR CONN_DBG_CTL_WF_MCU_DBG_EN_FR_HIF_ADDR
#define CONN_DBG_CTL_WF_MCU_DBG_EN_FR_HIF_WF_MCU_DBG_EN_FR_HIF_MASK 0x00000001                // WF_MCU_DBG_EN_FR_HIF[0]
#define CONN_DBG_CTL_WF_MCU_DBG_EN_FR_HIF_WF_MCU_DBG_EN_FR_HIF_SHFT 0

/* =====================================================================================

  ---WF_MCU_DBG_SEL_FR_HIF (0x18023000 + 0x61C)---

    WF_MCU_DBG_SEL_FR_HIF[27..0] - (RW)  xxx 
    RESERVED28[31..28]           - (RO) Reserved bits

 =====================================================================================*/
#define CONN_DBG_CTL_WF_MCU_DBG_SEL_FR_HIF_WF_MCU_DBG_SEL_FR_HIF_ADDR CONN_DBG_CTL_WF_MCU_DBG_SEL_FR_HIF_ADDR
#define CONN_DBG_CTL_WF_MCU_DBG_SEL_FR_HIF_WF_MCU_DBG_SEL_FR_HIF_MASK 0x0FFFFFFF                // WF_MCU_DBG_SEL_FR_HIF[27..0]
#define CONN_DBG_CTL_WF_MCU_DBG_SEL_FR_HIF_WF_MCU_DBG_SEL_FR_HIF_SHFT 0

/* =====================================================================================

  ---WF_CORE_PC_INDEX_FR_HIF (0x18023000 + 0x620)---

    WF_CORE_PC_INDEX_FR_HIF[5..0] - (RW)  xxx 
    RESERVED6[31..6]             - (RO) Reserved bits

 =====================================================================================*/
#define CONN_DBG_CTL_WF_CORE_PC_INDEX_FR_HIF_WF_CORE_PC_INDEX_FR_HIF_ADDR CONN_DBG_CTL_WF_CORE_PC_INDEX_FR_HIF_ADDR
#define CONN_DBG_CTL_WF_CORE_PC_INDEX_FR_HIF_WF_CORE_PC_INDEX_FR_HIF_MASK 0x0000003F                // WF_CORE_PC_INDEX_FR_HIF[5..0]
#define CONN_DBG_CTL_WF_CORE_PC_INDEX_FR_HIF_WF_CORE_PC_INDEX_FR_HIF_SHFT 0

/* =====================================================================================

  ---WF_MCUSYS_INFRA_VDNR_GEN_DEBUG_CTRL_AO_DEBUGSYS_CTRL (0x18023000 + 0x628)---

    WF_MCUSYS_INFRA_VDNR_GEN_DEBUG_CTRL_AO_DEBUGSYS_CTRL[31..0] - (RW)  xxx 

 =====================================================================================*/
#define CONN_DBG_CTL_WF_MCUSYS_INFRA_VDNR_GEN_DEBUG_CTRL_AO_DEBUGSYS_CTRL_WF_MCUSYS_INFRA_VDNR_GEN_DEBUG_CTRL_AO_DEBUGSYS_CTRL_ADDR CONN_DBG_CTL_WF_MCUSYS_INFRA_VDNR_GEN_DEBUG_CTRL_AO_DEBUGSYS_CTRL_ADDR
#define CONN_DBG_CTL_WF_MCUSYS_INFRA_VDNR_GEN_DEBUG_CTRL_AO_DEBUGSYS_CTRL_WF_MCUSYS_INFRA_VDNR_GEN_DEBUG_CTRL_AO_DEBUGSYS_CTRL_MASK 0xFFFFFFFF                // WF_MCUSYS_INFRA_VDNR_GEN_DEBUG_CTRL_AO_DEBUGSYS_CTRL[31..0]
#define CONN_DBG_CTL_WF_MCUSYS_INFRA_VDNR_GEN_DEBUG_CTRL_AO_DEBUGSYS_CTRL_WF_MCUSYS_INFRA_VDNR_GEN_DEBUG_CTRL_AO_DEBUGSYS_CTRL_SHFT 0

/* =====================================================================================

  ---WF_MCUSYS_INFRA_VDNR_GEN_DEBUG_CTRL_AO_BUS_TIMEOUT_IRQ (0x18023000 + 0x62C)---

    WF_MCUSYS_INFRA_VDNR_GEN_DEBUG_CTRL_AO_BUS_TIMEOUT_IRQ[0] - (RO)  xxx 
    RESERVED1[31..1]             - (RO) Reserved bits

 =====================================================================================*/
#define CONN_DBG_CTL_WF_MCUSYS_INFRA_VDNR_GEN_DEBUG_CTRL_AO_BUS_TIMEOUT_IRQ_WF_MCUSYS_INFRA_VDNR_GEN_DEBUG_CTRL_AO_BUS_TIMEOUT_IRQ_ADDR CONN_DBG_CTL_WF_MCUSYS_INFRA_VDNR_GEN_DEBUG_CTRL_AO_BUS_TIMEOUT_IRQ_ADDR
#define CONN_DBG_CTL_WF_MCUSYS_INFRA_VDNR_GEN_DEBUG_CTRL_AO_BUS_TIMEOUT_IRQ_WF_MCUSYS_INFRA_VDNR_GEN_DEBUG_CTRL_AO_BUS_TIMEOUT_IRQ_MASK 0x00000001                // WF_MCUSYS_INFRA_VDNR_GEN_DEBUG_CTRL_AO_BUS_TIMEOUT_IRQ[0]
#define CONN_DBG_CTL_WF_MCUSYS_INFRA_VDNR_GEN_DEBUG_CTRL_AO_BUS_TIMEOUT_IRQ_WF_MCUSYS_INFRA_VDNR_GEN_DEBUG_CTRL_AO_BUS_TIMEOUT_IRQ_SHFT 0

/* =====================================================================================

  ---BGF_MONFLAG_OFF_OUT (0x18023000 + 0xA00)---

    BGF_MONFLAG_OFF_OUT[31..0]   - (RO) select by "CR_DBGCTL2BGF_OFF_DEBUG_SEL"

 =====================================================================================*/
#define CONN_DBG_CTL_BGF_MONFLAG_OFF_OUT_BGF_MONFLAG_OFF_OUT_ADDR CONN_DBG_CTL_BGF_MONFLAG_OFF_OUT_ADDR
#define CONN_DBG_CTL_BGF_MONFLAG_OFF_OUT_BGF_MONFLAG_OFF_OUT_MASK 0xFFFFFFFF                // BGF_MONFLAG_OFF_OUT[31..0]
#define CONN_DBG_CTL_BGF_MONFLAG_OFF_OUT_BGF_MONFLAG_OFF_OUT_SHFT 0

/* =====================================================================================

  ---CR_DBGCTL2BGF_OFF_DEBUG_SEL (0x18023000 + 0xA04)---

    CR_DBGCTL2BGF_OFF_DEBUG_SEL[31..0] - (RW)  xxx 

 =====================================================================================*/
#define CONN_DBG_CTL_CR_DBGCTL2BGF_OFF_DEBUG_SEL_CR_DBGCTL2BGF_OFF_DEBUG_SEL_ADDR CONN_DBG_CTL_CR_DBGCTL2BGF_OFF_DEBUG_SEL_ADDR
#define CONN_DBG_CTL_CR_DBGCTL2BGF_OFF_DEBUG_SEL_CR_DBGCTL2BGF_OFF_DEBUG_SEL_MASK 0xFFFFFFFF                // CR_DBGCTL2BGF_OFF_DEBUG_SEL[31..0]
#define CONN_DBG_CTL_CR_DBGCTL2BGF_OFF_DEBUG_SEL_CR_DBGCTL2BGF_OFF_DEBUG_SEL_SHFT 0

/* =====================================================================================

  ---GPSAON2HOST_DEBUG (0x18023000 + 0xA08)---

    GPSAON2HOST_DEBUG[11..0]     - (RO)  xxx 
    RESERVED12[31..12]           - (RO) Reserved bits

 =====================================================================================*/
#define CONN_DBG_CTL_GPSAON2HOST_DEBUG_GPSAON2HOST_DEBUG_ADDR  CONN_DBG_CTL_GPSAON2HOST_DEBUG_ADDR
#define CONN_DBG_CTL_GPSAON2HOST_DEBUG_GPSAON2HOST_DEBUG_MASK  0x00000FFF                // GPSAON2HOST_DEBUG[11..0]
#define CONN_DBG_CTL_GPSAON2HOST_DEBUG_GPSAON2HOST_DEBUG_SHFT  0

/* =====================================================================================

  ---CONN_DBG_CTL_IRQ_FOR_APMCU_STAT (0x18023000 + 0xC00)---

    CCIF_BGF2AP_SW_IRQ_B[0]      - (RO) when BGF_MCU want to send APMCU some information, the irq will be triggered (low active)
    CONN_BGF_HIF_ON_HOST_INT_B[1] - (RO) BGF fw_own or driver_own state switching (including: fw_own to driver_own; driver_own to fw_own)  (low active)
    GPS_DMA_IRQ_B[2]             - (RO) when GPS DMA transfer completion, then the irq will be triggered(low active)
    GPS_L5_IRQ_BUS_B[5..3]       - (RO) when gps L5 has data to transfer, these irqs may be triggered for different condition, such as wi/wo data or msg update (low active)
    GPS_L1_IRQ_BUS_B[8..6]       - (RO) when gps L1 has data to transfer, these irqs may be triggered for different condition, such as wi/wo data or msg update (low active)
    CONN_MAWD2AP_IRQ_B[9]        - (RO) When wifi rx ring update (and mem dma don), the irq will be triggered (low active)
    SEMA_M3_TIMEOUT_INT_B[10]    - (RO) when master3 get the semaphore too long, the irq will be triggered (low active)
    SEMA_M2_TIMEOUT_INT_B[11]    - (RO) when master2 get the semaphore too long, the irq will be triggered (low active)
    SEMA_RELEASE_INFORM_M3_INT_B[12] - (RO) when the semaphore which queuing by master3 is  released from other masters, the irq will be triggered (low active)
    SEMA_RELEASE_INFORM_M2_INT_B[13] - (RO) when the semaphore which queuing by master2 is  released from other masters, the irq will be triggered (low active)
    BGF2AP_WDT_IRQ_B[14]         - (RO) when GPS watch dog time out, then the irq will be triggered(low active)
    CONN_INFRA_BUS_TIMEOUT_IRQ_B[15] - (RO) when conn_infra bus time out, the irq will be triggered (low active)
    RESERVED16[31..16]           - (RO) Reserved bits

 =====================================================================================*/
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_CONN_INFRA_BUS_TIMEOUT_IRQ_B_ADDR CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_ADDR
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_CONN_INFRA_BUS_TIMEOUT_IRQ_B_MASK 0x00008000                // CONN_INFRA_BUS_TIMEOUT_IRQ_B[15]
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_CONN_INFRA_BUS_TIMEOUT_IRQ_B_SHFT 15
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_BGF2AP_WDT_IRQ_B_ADDR  CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_ADDR
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_BGF2AP_WDT_IRQ_B_MASK  0x00004000                // BGF2AP_WDT_IRQ_B[14]
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_BGF2AP_WDT_IRQ_B_SHFT  14
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_SEMA_RELEASE_INFORM_M2_INT_B_ADDR CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_ADDR
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_SEMA_RELEASE_INFORM_M2_INT_B_MASK 0x00002000                // SEMA_RELEASE_INFORM_M2_INT_B[13]
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_SEMA_RELEASE_INFORM_M2_INT_B_SHFT 13
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_SEMA_RELEASE_INFORM_M3_INT_B_ADDR CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_ADDR
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_SEMA_RELEASE_INFORM_M3_INT_B_MASK 0x00001000                // SEMA_RELEASE_INFORM_M3_INT_B[12]
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_SEMA_RELEASE_INFORM_M3_INT_B_SHFT 12
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_SEMA_M2_TIMEOUT_INT_B_ADDR CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_ADDR
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_SEMA_M2_TIMEOUT_INT_B_MASK 0x00000800                // SEMA_M2_TIMEOUT_INT_B[11]
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_SEMA_M2_TIMEOUT_INT_B_SHFT 11
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_SEMA_M3_TIMEOUT_INT_B_ADDR CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_ADDR
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_SEMA_M3_TIMEOUT_INT_B_MASK 0x00000400                // SEMA_M3_TIMEOUT_INT_B[10]
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_SEMA_M3_TIMEOUT_INT_B_SHFT 10
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_CONN_MAWD2AP_IRQ_B_ADDR CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_ADDR
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_CONN_MAWD2AP_IRQ_B_MASK 0x00000200                // CONN_MAWD2AP_IRQ_B[9]
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_CONN_MAWD2AP_IRQ_B_SHFT 9
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_GPS_L1_IRQ_BUS_B_ADDR  CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_ADDR
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_GPS_L1_IRQ_BUS_B_MASK  0x000001C0                // GPS_L1_IRQ_BUS_B[8..6]
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_GPS_L1_IRQ_BUS_B_SHFT  6
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_GPS_L5_IRQ_BUS_B_ADDR  CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_ADDR
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_GPS_L5_IRQ_BUS_B_MASK  0x00000038                // GPS_L5_IRQ_BUS_B[5..3]
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_GPS_L5_IRQ_BUS_B_SHFT  3
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_GPS_DMA_IRQ_B_ADDR     CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_ADDR
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_GPS_DMA_IRQ_B_MASK     0x00000004                // GPS_DMA_IRQ_B[2]
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_GPS_DMA_IRQ_B_SHFT     2
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_CONN_BGF_HIF_ON_HOST_INT_B_ADDR CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_ADDR
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_CONN_BGF_HIF_ON_HOST_INT_B_MASK 0x00000002                // CONN_BGF_HIF_ON_HOST_INT_B[1]
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_CONN_BGF_HIF_ON_HOST_INT_B_SHFT 1
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_CCIF_BGF2AP_SW_IRQ_B_ADDR CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_ADDR
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_CCIF_BGF2AP_SW_IRQ_B_MASK 0x00000001                // CCIF_BGF2AP_SW_IRQ_B[0]
#define CONN_DBG_CTL_IRQ_FOR_APMCU_STAT_CCIF_BGF2AP_SW_IRQ_B_SHFT 0

/* =====================================================================================

  ---CONN_DBG_CTL_EINT_FOR_APMCU_STAT (0x18023000 + 0xC10)---

    BGF2AP_WDT_IRQ_B[0]          - (RO) when GPS watch dog time out, then the irq will be triggered(low active)
    RESERVED1[3..1]              - (RO) Reserved bits
    CONN_BGF_HIF_ON_HOST_INT_B[4] - (RO) BGF fw_own or driver_own state switching (including: fw_own to driver_own; driver_own to fw_own)  (low active)
    RESERVED5[5]                 - (RO) Reserved bits
    CCIF_BGF2AP_SW_IRQ_B[6]      - (RO) when BGF_MCU want to send APMCU some information, the irq will be triggered (low active)
    RESERVED7[7]                 - (RO) Reserved bits
    SEMA_RELEASE_INFORM_M2_INT_B[8] - (RO) when the semaphore which queuing by master2 is  released from other masters, the irq will be triggered (low active)
    SEMA_RELEASE_INFORM_M3_INT_B[9] - (RO) when the semaphore which queuing by master3 is  released from other masters, the irq will be triggered (low active)
    SEMA_M2_TIMEOUT_INT_B[10]    - (RO) when master2 get the semaphore too long, the irq will be triggered (low active)
    SEMA_M3_TIMEOUT_INT_B[11]    - (RO) when master3 get the semaphore too long, the irq will be triggered (low active)
    CONN_MAWD2AP_IRQ_B[12]       - (RO) When wifi rx ring update (and mem dma don), the irq will be triggered (low active)
    RESERVED13[15..13]           - (RO) Reserved bits
    GPS_L1_IRQ_BUS_B[16]         - (RO) when gps L1 has data to transfer, these irqs may be triggered for different condition, such as wi/wo data or msg update (low active)
    GPS_L5_IRQ_BUS_B[17]         - (RO) when gps L5 has data to transfer, these irqs may be triggered for different condition, such as wi/wo data or msg update (low active)
    RESERVED18[31..18]           - (RO) Reserved bits

 =====================================================================================*/
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_GPS_L5_IRQ_BUS_B_ADDR CONN_DBG_CTL_EINT_FOR_APMCU_STAT_ADDR
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_GPS_L5_IRQ_BUS_B_MASK 0x00020000                // GPS_L5_IRQ_BUS_B[17]
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_GPS_L5_IRQ_BUS_B_SHFT 17
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_GPS_L1_IRQ_BUS_B_ADDR CONN_DBG_CTL_EINT_FOR_APMCU_STAT_ADDR
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_GPS_L1_IRQ_BUS_B_MASK 0x00010000                // GPS_L1_IRQ_BUS_B[16]
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_GPS_L1_IRQ_BUS_B_SHFT 16
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_CONN_MAWD2AP_IRQ_B_ADDR CONN_DBG_CTL_EINT_FOR_APMCU_STAT_ADDR
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_CONN_MAWD2AP_IRQ_B_MASK 0x00001000                // CONN_MAWD2AP_IRQ_B[12]
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_CONN_MAWD2AP_IRQ_B_SHFT 12
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_SEMA_M3_TIMEOUT_INT_B_ADDR CONN_DBG_CTL_EINT_FOR_APMCU_STAT_ADDR
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_SEMA_M3_TIMEOUT_INT_B_MASK 0x00000800                // SEMA_M3_TIMEOUT_INT_B[11]
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_SEMA_M3_TIMEOUT_INT_B_SHFT 11
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_SEMA_M2_TIMEOUT_INT_B_ADDR CONN_DBG_CTL_EINT_FOR_APMCU_STAT_ADDR
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_SEMA_M2_TIMEOUT_INT_B_MASK 0x00000400                // SEMA_M2_TIMEOUT_INT_B[10]
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_SEMA_M2_TIMEOUT_INT_B_SHFT 10
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_SEMA_RELEASE_INFORM_M3_INT_B_ADDR CONN_DBG_CTL_EINT_FOR_APMCU_STAT_ADDR
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_SEMA_RELEASE_INFORM_M3_INT_B_MASK 0x00000200                // SEMA_RELEASE_INFORM_M3_INT_B[9]
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_SEMA_RELEASE_INFORM_M3_INT_B_SHFT 9
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_SEMA_RELEASE_INFORM_M2_INT_B_ADDR CONN_DBG_CTL_EINT_FOR_APMCU_STAT_ADDR
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_SEMA_RELEASE_INFORM_M2_INT_B_MASK 0x00000100                // SEMA_RELEASE_INFORM_M2_INT_B[8]
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_SEMA_RELEASE_INFORM_M2_INT_B_SHFT 8
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_CCIF_BGF2AP_SW_IRQ_B_ADDR CONN_DBG_CTL_EINT_FOR_APMCU_STAT_ADDR
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_CCIF_BGF2AP_SW_IRQ_B_MASK 0x00000040                // CCIF_BGF2AP_SW_IRQ_B[6]
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_CCIF_BGF2AP_SW_IRQ_B_SHFT 6
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_CONN_BGF_HIF_ON_HOST_INT_B_ADDR CONN_DBG_CTL_EINT_FOR_APMCU_STAT_ADDR
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_CONN_BGF_HIF_ON_HOST_INT_B_MASK 0x00000010                // CONN_BGF_HIF_ON_HOST_INT_B[4]
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_CONN_BGF_HIF_ON_HOST_INT_B_SHFT 4
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_BGF2AP_WDT_IRQ_B_ADDR CONN_DBG_CTL_EINT_FOR_APMCU_STAT_ADDR
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_BGF2AP_WDT_IRQ_B_MASK 0x00000001                // BGF2AP_WDT_IRQ_B[0]
#define CONN_DBG_CTL_EINT_FOR_APMCU_STAT_BGF2AP_WDT_IRQ_B_SHFT 0

/* =====================================================================================

  ---DBG_BACKUP_0 (0x18023000 + 0xE00)---

    DBG_BACKUP_0[31..0]          - (RO) backup un-use

 =====================================================================================*/
#define CONN_DBG_CTL_DBG_BACKUP_0_DBG_BACKUP_0_ADDR            CONN_DBG_CTL_DBG_BACKUP_0_ADDR
#define CONN_DBG_CTL_DBG_BACKUP_0_DBG_BACKUP_0_MASK            0xFFFFFFFF                // DBG_BACKUP_0[31..0]
#define CONN_DBG_CTL_DBG_BACKUP_0_DBG_BACKUP_0_SHFT            0

/* =====================================================================================

  ---DBG_BACKUP_1 (0x18023000 + 0xE04)---

    DBG_BACKUP_1[31..0]          - (RO) backup un-use

 =====================================================================================*/
#define CONN_DBG_CTL_DBG_BACKUP_1_DBG_BACKUP_1_ADDR            CONN_DBG_CTL_DBG_BACKUP_1_ADDR
#define CONN_DBG_CTL_DBG_BACKUP_1_DBG_BACKUP_1_MASK            0xFFFFFFFF                // DBG_BACKUP_1[31..0]
#define CONN_DBG_CTL_DBG_BACKUP_1_DBG_BACKUP_1_SHFT            0

/* =====================================================================================

  ---DBG_BACKUP_2 (0x18023000 + 0xE08)---

    DBG_BACKUP_2[31..0]          - (RO) backup un-use

 =====================================================================================*/
#define CONN_DBG_CTL_DBG_BACKUP_2_DBG_BACKUP_2_ADDR            CONN_DBG_CTL_DBG_BACKUP_2_ADDR
#define CONN_DBG_CTL_DBG_BACKUP_2_DBG_BACKUP_2_MASK            0xFFFFFFFF                // DBG_BACKUP_2[31..0]
#define CONN_DBG_CTL_DBG_BACKUP_2_DBG_BACKUP_2_SHFT            0

/* =====================================================================================

  ---DBG_BACKUP_3 (0x18023000 + 0xE0C)---

    DBG_BACKUP_3[31..0]          - (RO) backup un-use

 =====================================================================================*/
#define CONN_DBG_CTL_DBG_BACKUP_3_DBG_BACKUP_3_ADDR            CONN_DBG_CTL_DBG_BACKUP_3_ADDR
#define CONN_DBG_CTL_DBG_BACKUP_3_DBG_BACKUP_3_MASK            0xFFFFFFFF                // DBG_BACKUP_3[31..0]
#define CONN_DBG_CTL_DBG_BACKUP_3_DBG_BACKUP_3_SHFT            0

/* =====================================================================================

  ---DBG_BACKUP_4 (0x18023000 + 0xE10)---

    DBG_BACKUP_4[31..0]          - (RO) backup un-use

 =====================================================================================*/
#define CONN_DBG_CTL_DBG_BACKUP_4_DBG_BACKUP_4_ADDR            CONN_DBG_CTL_DBG_BACKUP_4_ADDR
#define CONN_DBG_CTL_DBG_BACKUP_4_DBG_BACKUP_4_MASK            0xFFFFFFFF                // DBG_BACKUP_4[31..0]
#define CONN_DBG_CTL_DBG_BACKUP_4_DBG_BACKUP_4_SHFT            0

/* =====================================================================================

  ---DBG_BACKUP_5 (0x18023000 + 0xE14)---

    DBG_BACKUP_5[31..0]          - (RO) backup un-use

 =====================================================================================*/
#define CONN_DBG_CTL_DBG_BACKUP_5_DBG_BACKUP_5_ADDR            CONN_DBG_CTL_DBG_BACKUP_5_ADDR
#define CONN_DBG_CTL_DBG_BACKUP_5_DBG_BACKUP_5_MASK            0xFFFFFFFF                // DBG_BACKUP_5[31..0]
#define CONN_DBG_CTL_DBG_BACKUP_5_DBG_BACKUP_5_SHFT            0

/* =====================================================================================

  ---DBG_BACKUP_6 (0x18023000 + 0xE18)---

    DBG_BACKUP_6[31..0]          - (RO) backup un-use

 =====================================================================================*/
#define CONN_DBG_CTL_DBG_BACKUP_6_DBG_BACKUP_6_ADDR            CONN_DBG_CTL_DBG_BACKUP_6_ADDR
#define CONN_DBG_CTL_DBG_BACKUP_6_DBG_BACKUP_6_MASK            0xFFFFFFFF                // DBG_BACKUP_6[31..0]
#define CONN_DBG_CTL_DBG_BACKUP_6_DBG_BACKUP_6_SHFT            0

/* =====================================================================================

  ---DBG_BACKUP_7 (0x18023000 + 0xE1C)---

    DBG_BACKUP_7[31..0]          - (RO) backup un-use

 =====================================================================================*/
#define CONN_DBG_CTL_DBG_BACKUP_7_DBG_BACKUP_7_ADDR            CONN_DBG_CTL_DBG_BACKUP_7_ADDR
#define CONN_DBG_CTL_DBG_BACKUP_7_DBG_BACKUP_7_MASK            0xFFFFFFFF                // DBG_BACKUP_7[31..0]
#define CONN_DBG_CTL_DBG_BACKUP_7_DBG_BACKUP_7_SHFT            0

/* =====================================================================================

  ---CR_CONN_TEST_DO_SEL (0x18023000 + 0xE20)---

    CR_CONN_TEST_DO_SEL[31..0]   - (RW)  xxx 

 =====================================================================================*/
#define CONN_DBG_CTL_CR_CONN_TEST_DO_SEL_CR_CONN_TEST_DO_SEL_ADDR CONN_DBG_CTL_CR_CONN_TEST_DO_SEL_ADDR
#define CONN_DBG_CTL_CR_CONN_TEST_DO_SEL_CR_CONN_TEST_DO_SEL_MASK 0xFFFFFFFF                // CR_CONN_TEST_DO_SEL[31..0]
#define CONN_DBG_CTL_CR_CONN_TEST_DO_SEL_CR_CONN_TEST_DO_SEL_SHFT 0

/* =====================================================================================

  ---CR_AP2WF_HOST_ON_CFG (0x18023000 + 0xE24)---

    CR_AP2WF_HOST_ON_CFG[15..0]  - (RW)  xxx 
    RESERVED16[31..16]           - (RO) Reserved bits

 =====================================================================================*/
#define CONN_DBG_CTL_CR_AP2WF_HOST_ON_CFG_CR_AP2WF_HOST_ON_CFG_ADDR CONN_DBG_CTL_CR_AP2WF_HOST_ON_CFG_ADDR
#define CONN_DBG_CTL_CR_AP2WF_HOST_ON_CFG_CR_AP2WF_HOST_ON_CFG_MASK 0x0000FFFF                // CR_AP2WF_HOST_ON_CFG[15..0]
#define CONN_DBG_CTL_CR_AP2WF_HOST_ON_CFG_CR_AP2WF_HOST_ON_CFG_SHFT 0

/* =====================================================================================

  ---DBG_CTL_LAST_CR (0x18023000 + 0xFFC)---

    LAST_REG[31..0]              - (RO) fixed value

 =====================================================================================*/
#define CONN_DBG_CTL_DBG_CTL_LAST_CR_LAST_REG_ADDR             CONN_DBG_CTL_DBG_CTL_LAST_CR_ADDR
#define CONN_DBG_CTL_DBG_CTL_LAST_CR_LAST_REG_MASK             0xFFFFFFFF                // LAST_REG[31..0]
#define CONN_DBG_CTL_DBG_CTL_LAST_CR_LAST_REG_SHFT             0

#ifdef __cplusplus
}
#endif

#endif // __CONN_DBG_CTL_REGS_H__
