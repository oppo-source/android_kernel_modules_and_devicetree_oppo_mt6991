/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2023 MediaTek Inc.
 */

#ifndef __PWR_CTRL_H__
#define __PWR_CTRL_H__

/* SPM_WAKEUP_MISC */
/* code gen: mt_spm_reg.h */
#define WAKE_MISC_SRCLKEN_RC_ERR_INT              (1U << 0)
#define WAKE_MISC_SPM_TIMEOUT_WAKEUP_0            (1U << 1)
#define WAKE_MISC_SPM_TIMEOUT_WAKEUP_1            (1U << 2)
#define WAKE_MISC_SPM_TIMEOUT_WAKEUP_2            (1U << 3)
#define WAKE_MISC_DVFSRC_IRQ                      (1U << 4)
#define WAKE_MISC_TWAM_IRQ_B                      (1U << 5)
#define WAKE_MISC_SPM_ACK_CHK_WAKEUP_0            (1U << 6)
#define WAKE_MISC_SPM_ACK_CHK_WAKEUP_1            (1U << 7)
#define WAKE_MISC_SPM_ACK_CHK_WAKEUP_2            (1U << 8)
#define WAKE_MISC_SPM_ACK_CHK_WAKEUP_3            (1U << 9)
#define WAKE_MISC_SPM_ACK_CHK_WAKEUP_ALL          (1U << 10)
#define WAKE_MISC_VLP_BUS_TIMEOUT_IRQ             (1U << 11)
#define WAKE_MISC_PCM_TIMER_EVENT                 (1U << 16)
#define WAKE_MISC_PMIC_EINT_OUT                   ((1U << 19) | (1U << 20))
#define WAKE_MISC_PMIC_IRQ_ACK                    (1U << 30)
#define WAKE_MISC_PMIC_SCP_IRQ                    (1U << 31)

/* code gen by spm_pwr_ctrl_atf.pl, need struct pwr_ctrl */
enum pwr_ctrl_enum {
	PW_PCM_FLAGS,
	PW_PCM_FLAGS_CUST,
	PW_PCM_FLAGS_CUST_SET,
	PW_PCM_FLAGS_CUST_CLR,
	PW_PCM_FLAGS1,
	PW_PCM_FLAGS1_CUST,
	PW_PCM_FLAGS1_CUST_SET,
	PW_PCM_FLAGS1_CUST_CLR,
	PW_TIMER_VAL,
	PW_TIMER_VAL_CUST,
	PW_TIMER_VAL_RAMP_EN,
	PW_TIMER_VAL_RAMP_EN_SEC,
	PW_WAKE_SRC,
	PW_WAKE_SRC_CUST,
	PW_WAKELOCK_TIMER_VAL,
	PW_WDT_DISABLE,

	/* SPM_SRC_REQ */
	PW_REG_SPM_ADSP_MAILBOX_REQ,
	PW_REG_SPM_APSRC_REQ,
	PW_REG_SPM_DDREN_REQ,
	PW_REG_SPM_DVFS_REQ,
	PW_REG_SPM_EMI_REQ,
	PW_REG_SPM_F26M_REQ,
	PW_REG_SPM_INFRA_REQ,
	PW_REG_SPM_PMIC_REQ,
	PW_REG_SPM_SCP_MAILBOX_REQ,
	PW_REG_SPM_SSPM_MAILBOX_REQ,
	PW_REG_SPM_SW_MAILBOX_REQ,
	PW_REG_SPM_VCORE_REQ,
	PW_REG_SPM_VRF18_REQ,

	/* SPM_SRC_MASK_0 */
	PW_REG_APIFR_MEM_APSRC_REQ_MASK_B,
	PW_REG_APIFR_MEM_DDREN_REQ_MASK_B,
	PW_REG_APIFR_MEM_EMI_REQ_MASK_B,
	PW_REG_APIFR_MEM_INFRA_REQ_MASK_B,
	PW_REG_APIFR_MEM_PMIC_REQ_MASK_B,
	PW_REG_APIFR_MEM_SRCCLKENA_MASK_B,
	PW_REG_APIFR_MEM_VCORE_REQ_MASK_B,
	PW_REG_APIFR_MEM_VRF18_REQ_MASK_B,
	PW_REG_APU_APSRC_REQ_MASK_B,
	PW_REG_APU_DDREN_REQ_MASK_B,
	PW_REG_APU_EMI_REQ_MASK_B,
	PW_REG_APU_INFRA_REQ_MASK_B,
	PW_REG_APU_PMIC_REQ_MASK_B,
	PW_REG_APU_SRCCLKENA_MASK_B,
	PW_REG_APU_VCORE_REQ_MASK_B,
	PW_REG_APU_VRF18_REQ_MASK_B,
	PW_REG_AUDIO_APSRC_REQ_MASK_B,
	PW_REG_AUDIO_DDREN_REQ_MASK_B,
	PW_REG_AUDIO_EMI_REQ_MASK_B,
	PW_REG_AUDIO_INFRA_REQ_MASK_B,
	PW_REG_AUDIO_PMIC_REQ_MASK_B,
	PW_REG_AUDIO_SRCCLKENA_MASK_B,
	PW_REG_AUDIO_VCORE_REQ_MASK_B,
	PW_REG_AUDIO_VRF18_REQ_MASK_B,

	/* SPM_SRC_MASK_1 */
	PW_REG_AUDIO_DSP_APSRC_REQ_MASK_B,
	PW_REG_AUDIO_DSP_DDREN_REQ_MASK_B,
	PW_REG_AUDIO_DSP_EMI_REQ_MASK_B,
	PW_REG_AUDIO_DSP_INFRA_REQ_MASK_B,
	PW_REG_AUDIO_DSP_PMIC_REQ_MASK_B,
	PW_REG_AUDIO_DSP_SRCCLKENA_MASK_B,
	PW_REG_AUDIO_DSP_VCORE_REQ_MASK_B,
	PW_REG_AUDIO_DSP_VRF18_REQ_MASK_B,
	PW_REG_CAM_APSRC_REQ_MASK_B,
	PW_REG_CAM_DDREN_REQ_MASK_B,
	PW_REG_CAM_EMI_REQ_MASK_B,
	PW_REG_CAM_INFRA_REQ_MASK_B,
	PW_REG_CAM_PMIC_REQ_MASK_B,
	PW_REG_CAM_SRCCLKENA_MASK_B,
	PW_REG_CAM_VRF18_REQ_MASK_B,
	PW_REG_CCIF_APSRC_REQ_MASK_B,

	/* SPM_SRC_MASK_2 */
	PW_REG_CCIF_EMI_REQ_MASK_B,
	PW_REG_CCIF_INFRA_REQ_MASK_B,

	/* SPM_SRC_MASK_3 */
	PW_REG_CCIF_PMIC_REQ_MASK_B,
	PW_REG_CCIF_SRCCLKENA_MASK_B,

	/* SPM_SRC_MASK_4 */
	PW_REG_CCIF_VCORE_REQ_MASK_B,
	PW_REG_CCIF_VRF18_REQ_MASK_B,
	PW_REG_CCU_APSRC_REQ_MASK_B,
	PW_REG_CCU_DDREN_REQ_MASK_B,
	PW_REG_CCU_EMI_REQ_MASK_B,
	PW_REG_CCU_INFRA_REQ_MASK_B,
	PW_REG_CCU_PMIC_REQ_MASK_B,
	PW_REG_CCU_SRCCLKENA_MASK_B,
	PW_REG_CCU_VRF18_REQ_MASK_B,
	PW_REG_CG_CHECK_APSRC_REQ_MASK_B,

	/* SPM_SRC_MASK_5 */
	PW_REG_CG_CHECK_DDREN_REQ_MASK_B,
	PW_REG_CG_CHECK_EMI_REQ_MASK_B,
	PW_REG_CG_CHECK_INFRA_REQ_MASK_B,
	PW_REG_CG_CHECK_PMIC_REQ_MASK_B,
	PW_REG_CG_CHECK_SRCCLKENA_MASK_B,
	PW_REG_CG_CHECK_VCORE_REQ_MASK_B,
	PW_REG_CG_CHECK_VRF18_REQ_MASK_B,
	PW_REG_CKSYS_APSRC_REQ_MASK_B,
	PW_REG_CKSYS_DDREN_REQ_MASK_B,
	PW_REG_CKSYS_EMI_REQ_MASK_B,
	PW_REG_CKSYS_INFRA_REQ_MASK_B,
	PW_REG_CKSYS_PMIC_REQ_MASK_B,
	PW_REG_CKSYS_SRCCLKENA_MASK_B,
	PW_REG_CKSYS_VCORE_REQ_MASK_B,
	PW_REG_CKSYS_VRF18_REQ_MASK_B,
	PW_REG_CKSYS_1_APSRC_REQ_MASK_B,
	PW_REG_CKSYS_1_DDREN_REQ_MASK_B,
	PW_REG_CKSYS_1_EMI_REQ_MASK_B,
	PW_REG_CKSYS_1_INFRA_REQ_MASK_B,
	PW_REG_CKSYS_1_PMIC_REQ_MASK_B,
	PW_REG_CKSYS_1_SRCCLKENA_MASK_B,
	PW_REG_CKSYS_1_VCORE_REQ_MASK_B,
	PW_REG_CKSYS_1_VRF18_REQ_MASK_B,

	/* SPM_SRC_MASK_6 */
	PW_REG_CKSYS_2_APSRC_REQ_MASK_B,
	PW_REG_CKSYS_2_DDREN_REQ_MASK_B,
	PW_REG_CKSYS_2_EMI_REQ_MASK_B,
	PW_REG_CKSYS_2_INFRA_REQ_MASK_B,
	PW_REG_CKSYS_2_PMIC_REQ_MASK_B,
	PW_REG_CKSYS_2_SRCCLKENA_MASK_B,
	PW_REG_CKSYS_2_VCORE_REQ_MASK_B,
	PW_REG_CKSYS_2_VRF18_REQ_MASK_B,
	PW_REG_CONN_APSRC_REQ_MASK_B,
	PW_REG_CONN_DDREN_REQ_MASK_B,
	PW_REG_CONN_EMI_REQ_MASK_B,
	PW_REG_CONN_INFRA_REQ_MASK_B,
	PW_REG_CONN_PMIC_REQ_MASK_B,
	PW_REG_CONN_SRCCLKENA_MASK_B,
	PW_REG_CONN_SRCCLKENB_MASK_B,
	PW_REG_CONN_VCORE_REQ_MASK_B,
	PW_REG_CONN_VRF18_REQ_MASK_B,
	PW_REG_CORECFG_RSV0_APSRC_REQ_MASK_B,
	PW_REG_CORECFG_RSV0_DDREN_REQ_MASK_B,
	PW_REG_CORECFG_RSV0_EMI_REQ_MASK_B,
	PW_REG_CORECFG_RSV0_INFRA_REQ_MASK_B,
	PW_REG_CORECFG_RSV0_PMIC_REQ_MASK_B,
	PW_REG_CORECFG_RSV0_SRCCLKENA_MASK_B,
	PW_REG_CORECFG_RSV0_VCORE_REQ_MASK_B,
	PW_REG_CORECFG_RSV0_VRF18_REQ_MASK_B,

	/* SPM_SRC_MASK_7 */
	PW_REG_MCUPM_APSRC_REQ_MASK_B,
	PW_REG_MCUPM_DDREN_REQ_MASK_B,
	PW_REG_MCUPM_EMI_REQ_MASK_B,
	PW_REG_MCUPM_INFRA_REQ_MASK_B,
	PW_REG_MCUPM_PMIC_REQ_MASK_B,
	PW_REG_MCUPM_SRCCLKENA_MASK_B,
	PW_REG_MCUPM_VCORE_REQ_MASK_B,
	PW_REG_MCUPM_VRF18_REQ_MASK_B,
	PW_REG_DISP0_APSRC_REQ_MASK_B,
	PW_REG_DISP0_DDREN_REQ_MASK_B,
	PW_REG_DISP0_EMI_REQ_MASK_B,
	PW_REG_DISP0_INFRA_REQ_MASK_B,
	PW_REG_DISP0_PMIC_REQ_MASK_B,
	PW_REG_DISP0_SRCCLKENA_MASK_B,
	PW_REG_DISP0_VRF18_REQ_MASK_B,
	PW_REG_DISP1_APSRC_REQ_MASK_B,
	PW_REG_DISP1_DDREN_REQ_MASK_B,
	PW_REG_DISP1_EMI_REQ_MASK_B,
	PW_REG_DISP1_INFRA_REQ_MASK_B,
	PW_REG_DISP1_PMIC_REQ_MASK_B,
	PW_REG_DISP1_SRCCLKENA_MASK_B,
	PW_REG_DISP1_VRF18_REQ_MASK_B,
	PW_REG_DPM_APSRC_REQ_MASK_B,
	PW_REG_DPM_DDREN_REQ_MASK_B,

	/* SPM_SRC_MASK_8 */
	PW_REG_DPM_EMI_REQ_MASK_B,
	PW_REG_DPM_INFRA_REQ_MASK_B,
	PW_REG_DPM_PMIC_REQ_MASK_B,
	PW_REG_DPM_SRCCLKENA_MASK_B,
	PW_REG_DPM_VCORE_REQ_MASK_B,
	PW_REG_DPM_VRF18_REQ_MASK_B,
	PW_REG_DPMAIF_APSRC_REQ_MASK_B,
	PW_REG_DPMAIF_DDREN_REQ_MASK_B,
	PW_REG_DPMAIF_EMI_REQ_MASK_B,
	PW_REG_DPMAIF_INFRA_REQ_MASK_B,
	PW_REG_DPMAIF_PMIC_REQ_MASK_B,
	PW_REG_DPMAIF_SRCCLKENA_MASK_B,
	PW_REG_DPMAIF_VCORE_REQ_MASK_B,
	PW_REG_DPMAIF_VRF18_REQ_MASK_B,

	/* SPM_SRC_MASK_9 */
	PW_REG_DVFSRC_LEVEL_REQ_MASK_B,
	PW_REG_EMISYS_APSRC_REQ_MASK_B,
	PW_REG_EMISYS_DDREN_REQ_MASK_B,
	PW_REG_EMISYS_EMI_REQ_MASK_B,
	PW_REG_EMISYS_INFRA_REQ_MASK_B,
	PW_REG_EMISYS_PMIC_REQ_MASK_B,
	PW_REG_EMISYS_SRCCLKENA_MASK_B,
	PW_REG_EMISYS_VCORE_REQ_MASK_B,
	PW_REG_EMISYS_VRF18_REQ_MASK_B,
	PW_REG_GCE_APSRC_REQ_MASK_B,
	PW_REG_GCE_DDREN_REQ_MASK_B,
	PW_REG_GCE_EMI_REQ_MASK_B,
	PW_REG_GCE_INFRA_REQ_MASK_B,
	PW_REG_GCE_PMIC_REQ_MASK_B,
	PW_REG_GCE_SRCCLKENA_MASK_B,
	PW_REG_GCE_VCORE_REQ_MASK_B,
	PW_REG_GCE_VRF18_REQ_MASK_B,
	PW_REG_GPUEB_APSRC_REQ_MASK_B,
	PW_REG_GPUEB_DDREN_REQ_MASK_B,
	PW_REG_GPUEB_EMI_REQ_MASK_B,
	PW_REG_GPUEB_INFRA_REQ_MASK_B,
	PW_REG_GPUEB_PMIC_REQ_MASK_B,
	PW_REG_GPUEB_SRCCLKENA_MASK_B,
	PW_REG_GPUEB_VCORE_REQ_MASK_B,
	PW_REG_GPUEB_VRF18_REQ_MASK_B,
	PW_REG_HWCCF_APSRC_REQ_MASK_B,
	PW_REG_HWCCF_DDREN_REQ_MASK_B,
	PW_REG_HWCCF_EMI_REQ_MASK_B,
	PW_REG_HWCCF_INFRA_REQ_MASK_B,
	PW_REG_HWCCF_PMIC_REQ_MASK_B,
	PW_REG_HWCCF_SRCCLKENA_MASK_B,
	PW_REG_HWCCF_VCORE_REQ_MASK_B,

	/* SPM_SRC_MASK_10 */
	PW_REG_HWCCF_VRF18_REQ_MASK_B,
	PW_REG_IMG_APSRC_REQ_MASK_B,
	PW_REG_IMG_DDREN_REQ_MASK_B,
	PW_REG_IMG_EMI_REQ_MASK_B,
	PW_REG_IMG_INFRA_REQ_MASK_B,
	PW_REG_IMG_PMIC_REQ_MASK_B,
	PW_REG_IMG_SRCCLKENA_MASK_B,
	PW_REG_IMG_VRF18_REQ_MASK_B,
	PW_REG_INFRASYS_APSRC_REQ_MASK_B,
	PW_REG_INFRASYS_DDREN_REQ_MASK_B,
	PW_REG_INFRASYS_EMI_REQ_MASK_B,
	PW_REG_INFRASYS_INFRA_REQ_MASK_B,
	PW_REG_INFRASYS_PMIC_REQ_MASK_B,
	PW_REG_INFRASYS_SRCCLKENA_MASK_B,
	PW_REG_INFRASYS_VCORE_REQ_MASK_B,
	PW_REG_INFRASYS_VRF18_REQ_MASK_B,
	PW_REG_IPIC_INFRA_REQ_MASK_B,
	PW_REG_IPIC_VRF18_REQ_MASK_B,
	PW_REG_MCU_APSRC_REQ_MASK_B,
	PW_REG_MCU_DDREN_REQ_MASK_B,
	PW_REG_MCU_EMI_REQ_MASK_B,
	PW_REG_MCU_INFRA_REQ_MASK_B,
	PW_REG_MCU_PMIC_REQ_MASK_B,
	PW_REG_MCU_SRCCLKENA_MASK_B,
	PW_REG_MCU_VCORE_REQ_MASK_B,
	PW_REG_MCU_VRF18_REQ_MASK_B,
	PW_REG_MD_APSRC_REQ_MASK_B,
	PW_REG_MD_DDREN_REQ_MASK_B,
	PW_REG_MD_EMI_REQ_MASK_B,
	PW_REG_MD_INFRA_REQ_MASK_B,
	PW_REG_MD_PMIC_REQ_MASK_B,
	PW_REG_MD_SRCCLKENA_MASK_B,

	/* SPM_SRC_MASK_11 */
	PW_REG_MD_SRCCLKENA1_MASK_B,
	PW_REG_MD_VCORE_REQ_MASK_B,
	PW_REG_MD_VRF18_REQ_MASK_B,
	PW_REG_MM_PROC_APSRC_REQ_MASK_B,
	PW_REG_MM_PROC_DDREN_REQ_MASK_B,
	PW_REG_MM_PROC_EMI_REQ_MASK_B,
	PW_REG_MM_PROC_INFRA_REQ_MASK_B,
	PW_REG_MM_PROC_PMIC_REQ_MASK_B,
	PW_REG_MM_PROC_SRCCLKENA_MASK_B,
	PW_REG_MM_PROC_VCORE_REQ_MASK_B,
	PW_REG_MM_PROC_VRF18_REQ_MASK_B,
	PW_REG_MML0_APSRC_REQ_MASK_B,
	PW_REG_MML0_DDREN_REQ_MASK_B,
	PW_REG_MML0_EMI_REQ_MASK_B,
	PW_REG_MML0_INFRA_REQ_MASK_B,
	PW_REG_MML0_PMIC_REQ_MASK_B,
	PW_REG_MML0_SRCCLKENA_MASK_B,
	PW_REG_MML0_VRF18_REQ_MASK_B,
	PW_REG_MML1_APSRC_REQ_MASK_B,
	PW_REG_MML1_DDREN_REQ_MASK_B,
	PW_REG_MML1_EMI_REQ_MASK_B,
	PW_REG_MML1_INFRA_REQ_MASK_B,
	PW_REG_MML1_PMIC_REQ_MASK_B,
	PW_REG_MML1_SRCCLKENA_MASK_B,
	PW_REG_MML1_VRF18_REQ_MASK_B,
	PW_REG_OVL0_APSRC_REQ_MASK_B,
	PW_REG_OVL0_DDREN_REQ_MASK_B,
	PW_REG_OVL0_EMI_REQ_MASK_B,
	PW_REG_OVL0_INFRA_REQ_MASK_B,
	PW_REG_OVL0_PMIC_REQ_MASK_B,
	PW_REG_OVL0_SRCCLKENA_MASK_B,
	PW_REG_OVL0_VRF18_REQ_MASK_B,

	/* SPM_SRC_MASK_12 */
	PW_REG_OVL1_APSRC_REQ_MASK_B,
	PW_REG_OVL1_DDREN_REQ_MASK_B,
	PW_REG_OVL1_EMI_REQ_MASK_B,
	PW_REG_OVL1_INFRA_REQ_MASK_B,
	PW_REG_OVL1_PMIC_REQ_MASK_B,
	PW_REG_OVL1_SRCCLKENA_MASK_B,
	PW_REG_OVL1_VRF18_REQ_MASK_B,
	PW_REG_PCIE0_APSRC_REQ_MASK_B,
	PW_REG_PCIE0_DDREN_REQ_MASK_B,
	PW_REG_PCIE0_EMI_REQ_MASK_B,
	PW_REG_PCIE0_INFRA_REQ_MASK_B,
	PW_REG_PCIE0_PMIC_REQ_MASK_B,
	PW_REG_PCIE0_SRCCLKENA_MASK_B,
	PW_REG_PCIE0_VCORE_REQ_MASK_B,
	PW_REG_PCIE0_VRF18_REQ_MASK_B,
	PW_REG_PCIE1_APSRC_REQ_MASK_B,
	PW_REG_PCIE1_DDREN_REQ_MASK_B,
	PW_REG_PCIE1_EMI_REQ_MASK_B,
	PW_REG_PCIE1_INFRA_REQ_MASK_B,
	PW_REG_PCIE1_PMIC_REQ_MASK_B,
	PW_REG_PCIE1_SRCCLKENA_MASK_B,
	PW_REG_PCIE1_VCORE_REQ_MASK_B,
	PW_REG_PCIE1_VRF18_REQ_MASK_B,
	PW_REG_PERISYS_APSRC_REQ_MASK_B,
	PW_REG_PERISYS_DDREN_REQ_MASK_B,
	PW_REG_PERISYS_EMI_REQ_MASK_B,
	PW_REG_PERISYS_INFRA_REQ_MASK_B,
	PW_REG_PERISYS_PMIC_REQ_MASK_B,
	PW_REG_PERISYS_SRCCLKENA_MASK_B,
	PW_REG_PERISYS_VCORE_REQ_MASK_B,
	PW_REG_PERISYS_VRF18_REQ_MASK_B,
	PW_REG_PMSR_APSRC_REQ_MASK_B,

	/* SPM_SRC_MASK_13 */
	PW_REG_PMSR_DDREN_REQ_MASK_B,
	PW_REG_PMSR_EMI_REQ_MASK_B,
	PW_REG_PMSR_INFRA_REQ_MASK_B,
	PW_REG_PMSR_PMIC_REQ_MASK_B,
	PW_REG_PMSR_SRCCLKENA_MASK_B,
	PW_REG_PMSR_VCORE_REQ_MASK_B,
	PW_REG_PMSR_VRF18_REQ_MASK_B,
	PW_REG_SCP_APSRC_REQ_MASK_B,
	PW_REG_SCP_DDREN_REQ_MASK_B,
	PW_REG_SCP_EMI_REQ_MASK_B,
	PW_REG_SCP_INFRA_REQ_MASK_B,
	PW_REG_SCP_PMIC_REQ_MASK_B,
	PW_REG_SCP_SRCCLKENA_MASK_B,
	PW_REG_SCP_VCORE_REQ_MASK_B,
	PW_REG_SCP_VRF18_REQ_MASK_B,
	PW_REG_SPU_HWROT_APSRC_REQ_MASK_B,
	PW_REG_SPU_HWROT_DDREN_REQ_MASK_B,
	PW_REG_SPU_HWROT_EMI_REQ_MASK_B,
	PW_REG_SPU_HWROT_INFRA_REQ_MASK_B,
	PW_REG_SPU_HWROT_PMIC_REQ_MASK_B,
	PW_REG_SPU_HWROT_SRCCLKENA_MASK_B,
	PW_REG_SPU_HWROT_VCORE_REQ_MASK_B,
	PW_REG_SPU_HWROT_VRF18_REQ_MASK_B,
	PW_REG_SPU_ISE_APSRC_REQ_MASK_B,
	PW_REG_SPU_ISE_DDREN_REQ_MASK_B,
	PW_REG_SPU_ISE_EMI_REQ_MASK_B,
	PW_REG_SPU_ISE_INFRA_REQ_MASK_B,
	PW_REG_SPU_ISE_PMIC_REQ_MASK_B,
	PW_REG_SPU_ISE_SRCCLKENA_MASK_B,
	PW_REG_SPU_ISE_VCORE_REQ_MASK_B,
	PW_REG_SPU_ISE_VRF18_REQ_MASK_B,

	/* SPM_SRC_MASK_14 */
	PW_REG_SRCCLKENI_INFRA_REQ_MASK_B,
	PW_REG_SRCCLKENI_PMIC_REQ_MASK_B,
	PW_REG_SRCCLKENI_SRCCLKENA_MASK_B,
	PW_REG_SRCCLKENI_VCORE_REQ_MASK_B,
	PW_REG_SSPM_APSRC_REQ_MASK_B,
	PW_REG_SSPM_DDREN_REQ_MASK_B,
	PW_REG_SSPM_EMI_REQ_MASK_B,
	PW_REG_SSPM_INFRA_REQ_MASK_B,
	PW_REG_SSPM_PMIC_REQ_MASK_B,
	PW_REG_SSPM_SRCCLKENA_MASK_B,
	PW_REG_SSPM_VRF18_REQ_MASK_B,
	PW_REG_SSRSYS_APSRC_REQ_MASK_B,
	PW_REG_SSRSYS_DDREN_REQ_MASK_B,
	PW_REG_SSRSYS_EMI_REQ_MASK_B,
	PW_REG_SSRSYS_INFRA_REQ_MASK_B,
	PW_REG_SSRSYS_PMIC_REQ_MASK_B,
	PW_REG_SSRSYS_SRCCLKENA_MASK_B,
	PW_REG_SSRSYS_VCORE_REQ_MASK_B,
	PW_REG_SSRSYS_VRF18_REQ_MASK_B,
	PW_REG_SSUSB_APSRC_REQ_MASK_B,
	PW_REG_SSUSB_DDREN_REQ_MASK_B,
	PW_REG_SSUSB_EMI_REQ_MASK_B,
	PW_REG_SSUSB_INFRA_REQ_MASK_B,
	PW_REG_SSUSB_PMIC_REQ_MASK_B,
	PW_REG_SSUSB_SRCCLKENA_MASK_B,
	PW_REG_SSUSB_VCORE_REQ_MASK_B,
	PW_REG_SSUSB_VRF18_REQ_MASK_B,
	PW_REG_UART_HUB_INFRA_REQ_MASK_B,

	/* SPM_SRC_MASK_15 */
	PW_REG_UART_HUB_PMIC_REQ_MASK_B,
	PW_REG_UART_HUB_SRCCLKENA_MASK_B,
	PW_REG_UART_HUB_VCORE_REQ_MASK_B,
	PW_REG_UART_HUB_VRF18_REQ_MASK_B,
	PW_REG_UFS_APSRC_REQ_MASK_B,
	PW_REG_UFS_DDREN_REQ_MASK_B,
	PW_REG_UFS_EMI_REQ_MASK_B,
	PW_REG_UFS_INFRA_REQ_MASK_B,
	PW_REG_UFS_PMIC_REQ_MASK_B,
	PW_REG_UFS_SRCCLKENA_MASK_B,
	PW_REG_UFS_VCORE_REQ_MASK_B,
	PW_REG_UFS_VRF18_REQ_MASK_B,
	PW_REG_VDEC_APSRC_REQ_MASK_B,
	PW_REG_VDEC_DDREN_REQ_MASK_B,
	PW_REG_VDEC_EMI_REQ_MASK_B,
	PW_REG_VDEC_INFRA_REQ_MASK_B,
	PW_REG_VDEC_PMIC_REQ_MASK_B,
	PW_REG_VDEC_SRCCLKENA_MASK_B,
	PW_REG_VDEC_VRF18_REQ_MASK_B,
	PW_REG_VENC_APSRC_REQ_MASK_B,
	PW_REG_VENC_DDREN_REQ_MASK_B,
	PW_REG_VENC_EMI_REQ_MASK_B,
	PW_REG_VENC_INFRA_REQ_MASK_B,
	PW_REG_VENC_PMIC_REQ_MASK_B,
	PW_REG_VENC_SRCCLKENA_MASK_B,
	PW_REG_VENC_VRF18_REQ_MASK_B,
	PW_REG_VLPCFG_RSV0_APSRC_REQ_MASK_B,
	PW_REG_VLPCFG_RSV0_DDREN_REQ_MASK_B,
	PW_REG_VLPCFG_RSV0_EMI_REQ_MASK_B,
	PW_REG_VLPCFG_RSV0_INFRA_REQ_MASK_B,
	PW_REG_VLPCFG_RSV0_PMIC_REQ_MASK_B,
	PW_REG_VLPCFG_RSV0_SRCCLKENA_MASK_B,

	/* SPM_SRC_MASK_16 */
	PW_REG_VLPCFG_RSV0_VCORE_REQ_MASK_B,
	PW_REG_VLPCFG_RSV0_VRF18_REQ_MASK_B,
	PW_REG_VLPCFG_RSV1_APSRC_REQ_MASK_B,
	PW_REG_VLPCFG_RSV1_DDREN_REQ_MASK_B,
	PW_REG_VLPCFG_RSV1_EMI_REQ_MASK_B,
	PW_REG_VLPCFG_RSV1_INFRA_REQ_MASK_B,
	PW_REG_VLPCFG_RSV1_PMIC_REQ_MASK_B,
	PW_REG_VLPCFG_RSV1_SRCCLKENA_MASK_B,
	PW_REG_VLPCFG_RSV1_VCORE_REQ_MASK_B,
	PW_REG_VLPCFG_RSV1_VRF18_REQ_MASK_B,

	/* SPM_EVENT_CON_MISC */
	PW_REG_SRCCLKEN_FAST_RESP,
	PW_REG_CSYSPWRUP_ACK_MASK,

	/* SPM_SRC_MASK_17 */
	PW_REG_SPM_SW_RSV_VCORE_REQ_MASK_B,
	PW_REG_SPM_SW_RSV_PMIC_REQ_MASK_B,

	/* SPM_SRC_MASK_18 */
	PW_REG_SPM_SW_RSV_SRCCLKENA_MASK_B,

	/* SPM_WAKEUP_EVENT_MASK */
	PW_REG_WAKEUP_EVENT_MASK,

	/* SPM_WAKEUP_EVENT_EXT_MASK */
	PW_REG_EXT_WAKEUP_EVENT_MASK,

	PW_MAX_COUNT,
};

#endif /* __PWR_CTRL_H__ */
