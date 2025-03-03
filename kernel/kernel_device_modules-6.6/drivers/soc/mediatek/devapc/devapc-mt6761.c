// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2015-2020 MediaTek Inc.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/types.h>
#include <linux/soc/mediatek/devapc_public.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>
#include <linux/sched/debug.h>
#ifdef DBG_ENABLE
#include <linux/arm-smccc.h>
#endif

#ifdef CONFIG_MTK_HIBERNATION
#include <mtk_hibernate_dpm.h>
#endif

/* CCF */
#include <linux/clk.h>

#include "devapc-mt6761.h"

#if defined(CONFIG_MTK_AEE_FEATURE) && defined(DEVAPC_ENABLE_AEE)
#include "aee.h"
#endif

/* 0 for early porting */
#define DEVAPC_TURN_ON         1
#define DEVAPC_USE_CCF         1

/* Debug message event */
#define DEVAPC_LOG_NONE        0x00000000
#define DEVAPC_LOG_INFO        0x00000001
#define DEVAPC_LOG_DBG         0x00000002

#define DEVAPC_LOG_LEVEL      (DEVAPC_LOG_DBG)

#define DEVAPC_MSG(fmt, args...) \
	do {    \
		if (DEVAPC_LOG_LEVEL & DEVAPC_LOG_DBG) { \
			pr_debug(fmt, ##args); \
		} else if (DEVAPC_LOG_LEVEL & DEVAPC_LOG_INFO) { \
			pr_info(fmt, ##args); \
		} \
	} while (0)


#define DEVAPC_VIO_LEVEL      (DEVAPC_LOG_INFO)

#define DEVAPC_VIO_MSG(fmt, args...) \
	do {    \
		if (DEVAPC_VIO_LEVEL & DEVAPC_LOG_DBG) { \
			pr_debug_ratelimited(fmt, ##args); \
		} else if (DEVAPC_VIO_LEVEL & DEVAPC_LOG_INFO) { \
			pr_info_ratelimited(fmt, ##args); \
		} \
	} while (0)



/* bypass clock! */
#if DEVAPC_USE_CCF
static struct clk *dapc_infra_clk;
#endif

static struct cdev *g_devapc_ctrl;
static unsigned int devapc_infra_irq;
static void __iomem *devapc_pd_infra_base;

static LIST_HEAD(viocb_list);

#if DEVAPC_TURN_ON
static struct DEVICE_INFO devapc_infra_devices[] = {
	/* device name                          enable_vio_irq */

	/* 0 */
	{"INFRA_AO_TOPCKGEN",                     true    },
	{"INFRA_AO_INFRASYS_CONFIG_REGS",         true    },
	{"IO_CFG_REG",                            true    },
	{"INFRA_AO_ PERICFG",                     true    },
	{"INFRA_AO_EFUSE_AO_DEBUG",               true    },
	{"INFRA_AO_GPIO",                         true    },
	{"INFRA_AO_SLEEP_CONTROLLER",             true    },
	{"INFRA_AO_TOPRGU",                       true    },
	{"INFRA_AO_APXGPT",                       true    },
	{"INFRA_AO_RESERVE",                      true    },

	/* 10 */
	{"INFRA_AO_SEJ",                          true    },
	{"INFRA_AO_AP_CIRQ_EINT",                 true    },
	{"INFRA_AO_APMIXEDSYS",                   true    },
	{"INFRA_AO_PMIC_WRAP",                    true    },
	{"INFRA_AO_DEVICE_APC_AO_INFRA_PERI",     true    },
	{"INFRA_AO_SLEEP_CONTROLLER_MD",          true    },
	{"INFRA_AO_KEYPAD",                       true    },
	{"INFRA_AO_TOP_MISC",                     true    },
	{"INFRA_AO_ DVFS_CTRL_PROC",              true    },
	{"INFRA_AO_MBIST_AO_REG",                 true    },

	/* 20 */
	{"INFRA_AO_CLDMA_AO_AP",                  true    },
	{"INFRA_AO_RESERVE",                      true    },
	{"INFRA_AO_AES_TOP_0",                    true    },
	{"INFRA_AO_SYS_TIMER",                    true    },
	{"INFRA_AO_MDEM_TEMP_SHARE",              true    },
	{"INFRA_AO_DEVICE_APC_AO_MD",             true    },
	{"INFRA_AO_SECURITY_AO",                  true    },
	{"INFRA_AO_TOPCKGEN_REG",                 true    },
	{"INFRA_AO_DEVICE_APC_AO_MM",             true    },
	{"INFRA_AO_DRAMC_REG",                    true    },

	/* 30 */
	{"INFRA_AO_DDRPHY_REG",                   true    },
	{"INFRA_AO_RESERVE",                      true    },
	{"INFRASYS_MCUSYS_REG0",                  true    },
	{"INFRASYS_MCUSYS_REG1",                  true    },
	{"INFRASYS_MCUSYS_REG2",                  true    },
	{"INFRASYS_MCUSYS_REG3",                  true    },
	{"INFRASYS_SYS_CIRQ",                     true    },
	{"INFRASYS_MM_IOMMU",                     true    },
	{"INFRASYS_RESERVE",                      true    },
	{"INFRASYS_DEVICE_APC",                   true    },

	/* 40 */
	{"INFRASYS_DBG_TRACKER",                  true    },
	{"INFRASYS_CCIF0_AP",                     true    },
	{"INFRASYS_CCIF0_MD",                     true    },
	{"INFRASYS_CCIF1_MD",                     true    },
	{"INFRASYS_CLDMA_MD",                     true    },
	{"INFRASYS_MBIST",                        true    },
	{"INFRASYS_INFRA_PDN_REGISTER",           true    },
	{"INFRASYS_TRNG",                         true    },
	{"INFRASYS_DX_CC",                        true    },
	{"INFRASYS_MCUPM_SRAM2",                  true    },

	/* 50 */
	{"INFRASYS_CQ_DMA",                       true    },
	{"INFRASYS_MCUPM_SRAM3",                  true    },
	{"INFRASYS_SRAMROM",                      true    },
	{"INFRASYS_RESERVE",                      true    },
	{"INFRASYS_MCUPM_REG",                    true    },
	{"INFRASYS_MCUPM_SRAM0",                  true    },
	{"INFRASYS_MCUPM_SRAM1",                  true    },
	{"INFRASYS_EMI",                          true    },
	{"INFRASYS_CHN_EMI",                      true    },
	{"INFRASYS_CLDMA_PDN_AP",                 true    },

	/* 60 */
	{"INFRASYS_CLDMA_PDN_MD",                 true    },
	{"INFRASYS_DRAMC_NAO",                    true    },
	{"INFRASYS_RESERVE",                      true    },
	{"INFRASYS_RESERVE",                      true    },
	{"INFRASYS_RESERVE",                      true    },
	{"INFRASYS_EMI_MPU",                      true    },
	{"INFRASYS_DVFS_PROC",                    true    },
	{"INFRASYS_DRAMC_CH0_TOP0",               true    },
	{"INFRASYS_DRAMC_CH0_TOP1",               true    },
	{"INFRASYS_DRAMC_CH0_TOP2",               true    },

	/* 70 */
	{"INFRASYS_DRAMC_CH0_TOP3",               true    },
	{"INFRASYS_DRAMC_CH0_TOP4",               true    },
	{"INFRASYS_DRAMC_CH1_TOP0",               true    },
	{"INFRASYS_DRAMC_CH1_TOP1",               true    },
	{"INFRASYS_DRAMC_CH1_TOP2",               true    },
	{"INFRASYS_DRAMC_CH1_TOP3",               true    },
	{"INFRASYS_DRAMC_CH1_TOP4",               true    },
	{"INFRASYS_GCE",                          true    },
	{"INFRASYS_CCIF2_AP",                     true    },
	{"INFRASYS_CCIF2_MD",                     true    },

	/* 80 */
	{"INFRASYS_CCIF3_AP",                     true    },
	{"INFRASYS_CCIF3_MD",                     true    },
	{"INFRA_AO_SSPM_1_1",                     true    },
	{"INFRA_AO_SSPM_1_2",                     true    },
	{"INFRA_AO_SSPM_1_3",                     true    },
	{"INFRA_AO_SSPM_2",                       true    },
	{"INFRA_AO_SSPM_3",                       true    },
	{"INFRA_AO_SSPM_4",                       true    },
	{"INFRA_AO_SSPM_5",                       true    },
	{"INFRA_AO_SSPM_6",                       true    },

	/* 90 */
	{"INFRA_AO_SSPM_7",                       true    },
	{"INFRA_AO_SSPM_8",                       true    },
	{"INFRA_AO_SCP",                          true    },
	{"INFRA_AO_MCUCFG",                       true    },
	{"INFRASYS_DBUGSYS",                      true    },
	{"PERISYS_APDMA",                         true    },
	{"PERISYS_AUXADC",                        true    },
	{"PERISYS_UART0",                         true    },
	{"PERISYS_UART1",                         true    },
	{"PERISYS_UART2",                         true    },

	/* 100 */
	{"PERISYS_UART3",                         true    },
	{"PERISYS_PWM",                           true    },
	{"PERISYS_I2C0",                          true    },
	{"PERISYS_I2C1",                          true    },
	{"PERISYS_I2C2",                          true    },
	{"PERISYS_SPI0",                          true    },
	{"PERISYS_PTP",                           true    },
	{"PERISYS_BTIF",                          true    },
	{"PERISYS_I2C6",                          true    },
	{"PERISYS_DISP_PWM",                      true    },

	/* 110 */
	{"PERISYS_I2C3",                          true    },
	{"PERISYS_SPI1",                          true    },
	{"PERISYS_I2C4",                          true    },
	{"PERISYS_SPI2",                          true    },
	{"PERISYS_SPI3",                          true    },
	{"PERISYS_SPI4",                          true    },
	{"PERISYS_SPI5",                          true    },
	{"PERISYS_I2C5",                          true    },
	{"PERISYS_IMP_IIC_WRAP",                  true    },
	{"PERISYS_NFI",                           true    },

	/* 120 */
	{"PERISYS_NFIECC",                        true    },
	{"PERISYS_USB",                           true    },
	{"PERISYS_USB_2.0_SUB",                   true    },
	{"PERISYS_MSDC0",                         true    },
	{"PERISYS_MSDC1",                         true    },
	{"PERISYS_MSDC2",                         true    },
	{"PERISYS_MSDC3",                         true    },
	{"PERISYS_UFS",                           true    },
	{"PERISUS_USB3.0_SIF",                    true    },
	{"PERISUS_USB3.0_SIF2",                   true    },

	/* 130 */
	{"PERISYS_USB_2.0_SIF",                   true    },
	{"PERISYS_AUDIO",                         true    },
	{"EAST_RESERVE",                          true    },
	{"EAST_ CSI_TOP_AO",                      true    },
	{"EAST_ RESERVE",                         true    },
	{"EAST_ RESERVE",                         true    },
	{"SOUTH_MSDC1",                           true    },
	{"SOUTH_EFUSE",                           true    },
	{"SOUTH_RESERVE_1",                       true    },
	{"SOUTH_RESERVE_2",                       true    },

	/* 140 */
	{"WEST_MIPI_TX_CONFIG",                   true    },
	{"WEST_RESERVE_0",                        true    },
	{"WEST_RESERVE_1",                        true    },
	{"WEST_RESERVE_2",                        true    },
	{"NORTH_USBSIF_TOP",                      true    },
	{"NORTH_MSDC0",                           true    },
	{"NORTH_RESERVE_0",                       true    },
	{"NORTH_RESERVE_1",                       true    },
	{"PERISYS_CONN",                          true    },
	{"PERISYS_RESERVE",                       true    },

	/* 150 */
	{"PERISYS_RESERVE",                       true    },
	{"GPU",                                   true    },
	{"GPU DVFS MON",                          true    },
	{"GPU CONFIG",                            true    },
	{"MFG_OTHERS",                            true    },
	{"MMSYS_CONFIG",                          true    },
	{"DISP_MUTEX",                            true    },
	{"SMI_COMMON",                            true    },
	{"SMI_LARB0",                             true    },
	{"MDP_RDMA0",                             true    },

	/* 160 */
	{"MDP_CCORR0",                            true    },
	{"MDP_RSZ0",                              true    },
	{"MDP_RSZ1",                              true    },
	{"MDP_WDMA0",                             true    },
	{"MDP_WROT0",                             true    },
	{"MDP_TDSHP0",                            true    },
	{"DISP_OVL0",                             true    },
	{"DISP_OVL0_2L",                          true    },
	{"DISP_RDMA0",                            true    },
	{"DISP_WDMA0",                            true    },

	/* 170 */
	{"DISP_COLOR0",                           true    },
	{"DISP_CCORR0",                           true    },
	{"DISP_AAL0",                             true    },
	{"DISP_GAMMA0",                           true    },
	{"DISP_DITHER0",                          true    },
	{"DSI0",                                  true    },
	{"DISP_RSZ0",                             true    },
	{"MM_MUTEX",                              true    },
	{"SMI_LARB0",                             true    },
	{"SMI_LARB1",                             true    },

	/* 180 */
	{"SMI_COMMON",                            true    },
	{"CAMSYS_CAMSYS_TOP",                     true    },
	{"CAMSYS_LARB2",                          true    },
	{"CAMSYS_RESERVED",                       true    },
	{"CAMSYS_RESERVED",                       true    },
	{"CAMSYS_MAIN",                           true    },
	{"CAMSYS_LUT",                            true    },
	{"CAMSYS_D",                              true    },
	{"CAMSYS_DMA",                            true    },
	{"CAMSYS_RESERVED",                       true    },

	/* 190 */
	{"CAMSYS_RESERVED",                       true    },
	{"CAMSYS_RESERVED",                       true    },
	{"CAMSYS_FDVT",                           true    },
	{"CAMSYS_RESERVED",                       true    },
	{"CAMSYS_MAIN_INNER",                     true    },
	{"CAMSYS_D_INNER",                        true    },
	{"CAMSYS_DMA_INNER",                      true    },
	{"CAMSYS_SENINF_A",                       true    },
	{"CAMSYS_SENINF_B",                       true    },
	{"CAMSYS_SENINF_C",                       true    },

	/* 200 */
	{"CAMSYS_SENINF_D",                       true    },
	{"CAMSYS_SENINF_E",                       true    },
	{"CAMSYS_SENINF_F",                       true    },
	{"CAMSYS_SENINF_G",                       true    },
	{"CAMSYS_SENINF_H",                       true    },
	{"CAMSYS_CAMSV_TOP_0",                    true    },
	{"CAMSYS_CAMSV_TOP_1",                    true    },
	{"CAMSYS_RESERVED",                       true    },
	{"CAMSYS_RESERVED",                       true    },
	{"VCODESYS_VENC_GLOBAL_CON",              true    },

	/* 210 */
	{"VCODESYS_SMI_LARB1",                    true    },
	{"VCODESYS_VENC",                         true    },
	{"VCODESYS_JPGENC",                       true    },
	{"VCODESYS_VDEC_FULL_TOP",                true    },
	{"VCODESYS_MBIST_CTRL",                   true    },

};
#endif

void register_devapc_vio_callback(struct devapc_vio_callbacks *viocb)
{
	INIT_LIST_HEAD(&viocb->list);
	list_add_tail(&viocb->list, &viocb_list);
}
EXPORT_SYMBOL(register_devapc_vio_callback);

/*
 * The extern functions for EMI MPU are removed because EMI MPU and Device APC
 * do not share the same IRQ now.
 */

/**************************************************************************
 *STATIC FUNCTION
 **************************************************************************/

#ifdef CONFIG_MTK_HIBERNATION
static int devapc_pm_restore_noirq(struct device *device)
{
	if (devapc_infra_irq != 0) {
		mt_irq_set_sens(devapc_infra_irq, MT_LEVEL_SENSITIVE);
		mt_irq_set_polarity(devapc_infra_irq, MT_POLARITY_LOW);
	}

	return 0;
}
#endif

#if DEVAPC_TURN_ON
static void unmask_infra_module_irq(unsigned int module)
{
	unsigned int apc_index = 0;
	unsigned int apc_bit_index = 0;

	if (module > PD_INFRA_VIO_MASK_MAX_INDEX) {
		pr_info("[DEVAPC] %s: module overflow!\n", __func__);
		return;
	}

	apc_index = module / (MOD_NO_IN_1_DEVAPC * 2);
	apc_bit_index = module % (MOD_NO_IN_1_DEVAPC * 2);

	*DEVAPC_PD_INFRA_VIO_MASK(apc_index) &=
		(0xFFFFFFFF ^ (1 << apc_bit_index));
}

#if defined(CONFIG_MTK_AEE_FEATURE) && defined(DEVAPC_ENABLE_AEE)
#ifdef DBG_ENABLE
static void mask_infra_module_irq(unsigned int module)
{
	unsigned int apc_index = 0;
	unsigned int apc_bit_index = 0;

	if (module > PD_INFRA_VIO_MASK_MAX_INDEX) {
		pr_info("[DEVAPC] %s: module overflow!\n", __func__);
		return;
	}

	apc_index = module / (MOD_NO_IN_1_DEVAPC * 2);
	apc_bit_index = module % (MOD_NO_IN_1_DEVAPC * 2);

	*DEVAPC_PD_INFRA_VIO_MASK(apc_index) |= (1 << apc_bit_index);
}
#endif
#endif

static int clear_infra_vio_status(unsigned int module)
{
	unsigned int apc_index = 0;
	unsigned int apc_bit_index = 0;

	if (module > PD_INFRA_VIO_STA_MAX_INDEX) {
		pr_info("[DEVAPC] %s: module overflow!\n", __func__);
		return -1;
	}

	apc_index = module / (MOD_NO_IN_1_DEVAPC * 2);
	apc_bit_index = module % (MOD_NO_IN_1_DEVAPC * 2);

	*DEVAPC_PD_INFRA_VIO_STA(apc_index) = (0x1 << apc_bit_index);

	return 0;
}

static int check_infra_vio_status(unsigned int module)
{
	unsigned int apc_index = 0;
	unsigned int apc_bit_index = 0;

	if (module > PD_INFRA_VIO_STA_MAX_INDEX) {
		pr_info("[DEVAPC] %s: module overflow!\n", __func__);
		return -1;
	}

	apc_index = module / (MOD_NO_IN_1_DEVAPC * 2);
	apc_bit_index = module % (MOD_NO_IN_1_DEVAPC * 2);

	if (*DEVAPC_PD_INFRA_VIO_STA(apc_index) & (0x1 << apc_bit_index))
		return 1;

	return 0;
}

static void start_devapc(void)
{
	unsigned int i;

	writel(0x80000000, DEVAPC_PD_INFRA_APC_CON);
	writel(0x3FFFFF, DEVAPC_PD_INFRA_VIO_SHIFT_STA);

	/* SMC call is called to set Device APC in LK instead */

	DEVAPC_MSG("[DEVAPC] INFRA VIO_MASK 0:0x%x 1:0x%x 2:0x%x 3:0x%x ",
			readl(DEVAPC_PD_INFRA_VIO_MASK(0)),
			readl(DEVAPC_PD_INFRA_VIO_MASK(1)),
			readl(DEVAPC_PD_INFRA_VIO_MASK(2)),
			readl(DEVAPC_PD_INFRA_VIO_MASK(3)));
	DEVAPC_MSG("4:0x%x 5:0x%x 6:0x%x 7:0x%x 8:0x%x\n",
			readl(DEVAPC_PD_INFRA_VIO_MASK(4)),
			readl(DEVAPC_PD_INFRA_VIO_MASK(5)),
			readl(DEVAPC_PD_INFRA_VIO_MASK(6)),
			readl(DEVAPC_PD_INFRA_VIO_MASK(7)),
			readl(DEVAPC_PD_INFRA_VIO_MASK(8)));

	DEVAPC_MSG("[DEVAPC] INFRA VIO_STA 0:0x%x 1:0x%x 2:0x%x 3:0x%x ",
			readl(DEVAPC_PD_INFRA_VIO_STA(0)),
			readl(DEVAPC_PD_INFRA_VIO_STA(1)),
			readl(DEVAPC_PD_INFRA_VIO_STA(2)),
			readl(DEVAPC_PD_INFRA_VIO_STA(3)));
	DEVAPC_MSG("4:0x%x 5:0x%x 6:0x%x 7:0x%x 8:0x%x\n",
			readl(DEVAPC_PD_INFRA_VIO_STA(4)),
			readl(DEVAPC_PD_INFRA_VIO_STA(5)),
			readl(DEVAPC_PD_INFRA_VIO_STA(6)),
			readl(DEVAPC_PD_INFRA_VIO_STA(7)),
			readl(DEVAPC_PD_INFRA_VIO_STA(8)));

	DEVAPC_MSG("[DEVAPC] %s\n",
		"Clear INFRA VIO_STA and unmask INFRA VIO_MASK...");

	for (i = 0; i < ARRAY_SIZE(devapc_infra_devices); i++)
		if (true == devapc_infra_devices[i].enable_vio_irq) {
			clear_infra_vio_status(i);
			unmask_infra_module_irq(i);
		}

	DEVAPC_MSG("[DEVAPC] INFRA VIO_MASK 0:0x%x 1:0x%x 2:0x%x 3:0x%x ",
			readl(DEVAPC_PD_INFRA_VIO_MASK(0)),
			readl(DEVAPC_PD_INFRA_VIO_MASK(1)),
			readl(DEVAPC_PD_INFRA_VIO_MASK(2)),
			readl(DEVAPC_PD_INFRA_VIO_MASK(3)));
	DEVAPC_MSG("4:0x%x 5:0x%x 6:0x%x 7:0x%x 8:0x%x\n",
			readl(DEVAPC_PD_INFRA_VIO_MASK(4)),
			readl(DEVAPC_PD_INFRA_VIO_MASK(5)),
			readl(DEVAPC_PD_INFRA_VIO_MASK(6)),
			readl(DEVAPC_PD_INFRA_VIO_MASK(7)),
			readl(DEVAPC_PD_INFRA_VIO_MASK(8)));

	DEVAPC_MSG("[DEVAPC] INFRA VIO_STA 0:0x%x 1:0x%x 2:0x%x 3:0x%x ",
			readl(DEVAPC_PD_INFRA_VIO_STA(0)),
			readl(DEVAPC_PD_INFRA_VIO_STA(1)),
			readl(DEVAPC_PD_INFRA_VIO_STA(2)),
			readl(DEVAPC_PD_INFRA_VIO_STA(3)));
	DEVAPC_MSG("4:0x%x 5:0x%x 6:0x%x 7:0x%x 8:0x%x\n",
			readl(DEVAPC_PD_INFRA_VIO_STA(4)),
			readl(DEVAPC_PD_INFRA_VIO_STA(5)),
			readl(DEVAPC_PD_INFRA_VIO_STA(6)),
			readl(DEVAPC_PD_INFRA_VIO_STA(7)),
			readl(DEVAPC_PD_INFRA_VIO_STA(8)));

}

#if defined(CONFIG_MTK_AEE_FEATURE) && defined(DEVAPC_ENABLE_AEE)
#ifdef DBG_ENABLE

/* violation index corresponds to subsys */
static const char *index_to_subsys(unsigned int index)
{
	if (index >= 151 && index <= 154)
		return "MFGSYS";
	else if (index == 155)
		return "MMSYS";
	else if (index == 156 || (index >= 166 && index <= 176))
		return "MMSYS_DISP";
	else if (index == 157 || index == 158 || index == 178 || index == 179
			|| index == 180 || index == 210)
		return "SMI";
	else if (index >= 159 && index <= 165)
		return "MMSYS_MDP";
	else if (index >= 209 && index <= 214)
		return "VCODECSYS";
	else if (index >= 181 && index <= 208)
		return "CAMSYS";
	else if (index < ARRAY_SIZE(devapc_infra_devices))
		return devapc_infra_devices[index].device;
	else
		return "OUT_OF_BOUND";
}

static void execute_aee(unsigned int i, unsigned int dbg0, unsigned int dbg1)
{
	char subsys_str[32] = {0};
	unsigned int domain_id;

	DEVAPC_VIO_MSG("[DEVAPC] Executing AEE Exception...\n");

	/* mask irq for module "i" */
	mask_infra_module_irq(i);

	domain_id = (dbg0 & INFRA_VIO_DBG_DMNID)
		>> INFRA_VIO_DBG_DMNID_START_BIT;

	if (domain_id == 1) {
		strscpy(subsys_str, "MD_SI", sizeof(subsys_str));
	} else {
		strscpy(subsys_str, index_to_subsys(i),
				sizeof(subsys_str));
	}

	subsys_str[sizeof(subsys_str)-1] = '\0';
	aee_kernel_exception("DEVAPC",
			"%s %s, Vio Addr: 0x%x\n%s%s\n",
			"[DEVAPC] Violation Slave:",
			devapc_infra_devices[i].device,
			dbg1,
			"CRDISPATCH_KEY:Device APC Violation Issue/",
			subsys_str
	);

	/* unmask irq for module "i" */
	unmask_infra_module_irq(i);

}

#endif
#endif // AEE_FEATURE

static irqreturn_t devapc_violation_irq(int irq_number, void *dev_id)
{
	unsigned int dbg0 = 0, dbg1 = 0;
	unsigned int master_id;
	unsigned int domain_id;
	unsigned int vio_addr_high;
	unsigned int read_violation;
	unsigned int write_violation;
	unsigned int device_count;
	unsigned int shift_done;
	unsigned int i;

	if (irq_number == devapc_infra_irq) {

		DEVAPC_MSG("[DEVAPC] %s 0:0x%x 1:0x%x 2:0x%x 3:0x%x ",
				"INFRA VIO_MASK",
				readl(DEVAPC_PD_INFRA_VIO_MASK(0)),
				readl(DEVAPC_PD_INFRA_VIO_MASK(1)),
				readl(DEVAPC_PD_INFRA_VIO_MASK(2)),
				readl(DEVAPC_PD_INFRA_VIO_MASK(3)));
		DEVAPC_MSG("4:0x%x 5:0x%x 6:0x%x 7:0x%x 8:0x%x\n",
				readl(DEVAPC_PD_INFRA_VIO_MASK(4)),
				readl(DEVAPC_PD_INFRA_VIO_MASK(5)),
				readl(DEVAPC_PD_INFRA_VIO_MASK(6)),
				readl(DEVAPC_PD_INFRA_VIO_MASK(7)),
				readl(DEVAPC_PD_INFRA_VIO_MASK(8)));

		DEVAPC_VIO_MSG("[DEVAPC] %s 0:0x%x 1:0x%x 2:0x%x 3:0x%x ",
				"INFRA VIO_STA",
				readl(DEVAPC_PD_INFRA_VIO_STA(0)),
				readl(DEVAPC_PD_INFRA_VIO_STA(1)),
				readl(DEVAPC_PD_INFRA_VIO_STA(2)),
				readl(DEVAPC_PD_INFRA_VIO_STA(3)));
		DEVAPC_VIO_MSG("4:0x%x 5:0x%x 6:0x%x 7:0x%x 8:0x%x\n",
				readl(DEVAPC_PD_INFRA_VIO_STA(4)),
				readl(DEVAPC_PD_INFRA_VIO_STA(5)),
				readl(DEVAPC_PD_INFRA_VIO_STA(6)),
				readl(DEVAPC_PD_INFRA_VIO_STA(7)),
				readl(DEVAPC_PD_INFRA_VIO_STA(8)));

		DEVAPC_MSG("[DEVAPC] VIO_SHIFT_STA: 0x%x\n",
			readl(DEVAPC_PD_INFRA_VIO_SHIFT_STA));

		for (i = 0; i <= PD_INFRA_VIO_SHIFT_MAX_BIT; ++i)
			if (readl(DEVAPC_PD_INFRA_VIO_SHIFT_STA) & (0x1 << i)) {
				writel(0x1 << i,
					DEVAPC_PD_INFRA_VIO_SHIFT_SEL);
				writel(0x1,
					DEVAPC_PD_INFRA_VIO_SHIFT_CON);
				for (shift_done = 0; (shift_done < 100) &&
					((readl(DEVAPC_PD_INFRA_VIO_SHIFT_CON)
					& 0x3) != 0x3); ++shift_done)
					DEVAPC_MSG("%s %s (%d, %d)\n",
						"[DEVAPC]",
						"Syncing INFRA DBG0 & DBG1",
						i, shift_done);

				DEVAPC_MSG("%s %s%X, %s%X\n",
					"[DEVAPC]",
					"VIO_SHIFT_SEL=0x",
					readl(DEVAPC_PD_INFRA_VIO_SHIFT_SEL),
					"VIO_SHIFT_CON=0x",
					readl(DEVAPC_PD_INFRA_VIO_SHIFT_CON));

				if ((readl(DEVAPC_PD_INFRA_VIO_SHIFT_CON) & 0x3)
					== 0x3)
					shift_done = 1;
				else {
					shift_done = 0;
					DEVAPC_VIO_MSG("[DEVAPC] index:%d %s\n",
						i, "sync failed!");
				}

				writel(0x0,
					DEVAPC_PD_INFRA_VIO_SHIFT_CON);
				writel(0x0,
					DEVAPC_PD_INFRA_VIO_SHIFT_SEL);
				writel(0x1 << i,
					DEVAPC_PD_INFRA_VIO_SHIFT_STA);

				if (shift_done == 0)
					continue;

				DEVAPC_MSG("%s %s%X, %s%X, %s%X\n",
					"[DEVAPC]",
					"VIO_SHIFT_STA=0x",
					readl(DEVAPC_PD_INFRA_VIO_SHIFT_STA),
					"VIO_SHIFT_SEL=0x",
					readl(DEVAPC_PD_INFRA_VIO_SHIFT_SEL),
					"VIO_SHIFT_CON=0x",
					readl(DEVAPC_PD_INFRA_VIO_SHIFT_CON));

				dbg0 = readl(DEVAPC_PD_INFRA_VIO_DBG0);
				dbg1 = readl(DEVAPC_PD_INFRA_VIO_DBG1);

				master_id = (dbg0 & INFRA_VIO_DBG_MSTID)
					>> INFRA_VIO_DBG_MSTID_START_BIT;
				domain_id = (dbg0 & INFRA_VIO_DBG_DMNID)
					>> INFRA_VIO_DBG_DMNID_START_BIT;
				write_violation = (dbg0 & INFRA_VIO_DBG_W_VIO)
					>> INFRA_VIO_DBG_W_VIO_START_BIT;
				read_violation = (dbg0 & INFRA_VIO_DBG_R_VIO)
					>> INFRA_VIO_DBG_R_VIO_START_BIT;
				vio_addr_high = (dbg0 & INFRA_VIO_ADDR_HIGH)
					>> INFRA_VIO_ADDR_HIGH_START_BIT;

				/* violation information improvement */
				DEVAPC_VIO_MSG("%s%s%s%s%x %s%x, %s%x, %s%x\n",
					"[DEVAPC] Violation(Infra,",
					read_violation == 1?"R":" ",
					write_violation == 1?"W) - ":" ) - ",
					"Vio Addr:0x", dbg1,
					"High:0x", vio_addr_high,
					"Bus ID:0x", master_id,
					"Dom ID:0x", domain_id);

				DEVAPC_VIO_MSG("%s - %s%s, %s%i\n",
					"[DEVAPC] Violation",
					"Process:", current->comm,
					"PID:", current->pid);

				break;
			}

		device_count = ARRAY_SIZE(devapc_infra_devices);

		/* checking and showing violation normal slaves */
		for (i = 0; i < device_count; i++)
			if (devapc_infra_devices[i].enable_vio_irq == true
				&& check_infra_vio_status(i) == 1) {
				clear_infra_vio_status(i);
				DEVAPC_VIO_MSG("%s %s %s (%s=%d)\n",
					"[DEVAPC]",
					"Access Violation Slave:",
					devapc_infra_devices[i].device,
					"infra index",
					i);

#if defined(CONFIG_MTK_AEE_FEATURE) && defined(DEVAPC_ENABLE_AEE)
	#ifdef DBG_ENABLE
				execute_aee(i, dbg0, dbg1);
	#endif
#endif
			}

	} else {
		DEVAPC_VIO_MSG("%s %s %d %s\n",
			"[DEVAPC]",
			"(ERROR) irq_number",
			irq_number,
			"is not registered!");
	}

	return IRQ_HANDLED;
}
#endif

static int devapc_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
#if DEVAPC_TURN_ON
	int ret;
#endif

	DEVAPC_MSG("[DEVAPC] module probe.\n");

	if (devapc_pd_infra_base == NULL) {
		if (node) {
			devapc_pd_infra_base = of_iomap(node,
				DAPC_DEVICE_TREE_NODE_PD_INFRA_INDEX);
			devapc_infra_irq = irq_of_parse_and_map(node,
				DAPC_DEVICE_TREE_NODE_PD_INFRA_INDEX);
			DEVAPC_MSG("[DEVAPC] PD_INFRA_ADDRESS: %p, IRQ: %d\n",
				devapc_pd_infra_base, devapc_infra_irq);
		} else {
			pr_info("[DEVAPC] %s\n",
				"can't find DAPC_INFRA_PD compatible node");
			return -1;
		}
	}

#if DEVAPC_TURN_ON
	ret = request_irq(devapc_infra_irq, (irq_handler_t)devapc_violation_irq,
			IRQF_SHARED,
			"devapc", &g_devapc_ctrl);
	if (ret) {
		pr_info("[DEVAPC] Failed to request infra irq! (%d)\n", ret);
		return ret;
	}
#endif

	/* CCF */
#if DEVAPC_USE_CCF
	dapc_infra_clk = devm_clk_get(&pdev->dev, "devapc-infra-clock");
	if (IS_ERR(dapc_infra_clk)) {
		pr_info("[DEVAPC] (Infra) %s\n",
			"Cannot get devapc clock from common clock framework.");
		return PTR_ERR(dapc_infra_clk);
	}
	clk_prepare_enable(dapc_infra_clk);
#endif

#ifdef CONFIG_MTK_HIBERNATION
	register_swsusp_restore_noirq_func(ID_M_DEVAPC,
		devapc_pm_restore_noirq, NULL);
#endif

#if DEVAPC_TURN_ON
	start_devapc();
#endif

	return 0;
}

static int devapc_remove(struct platform_device *dev)
{
	clk_disable_unprepare(dapc_infra_clk);
	return 0;
}

static int devapc_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int devapc_resume(struct platform_device *dev)
{
	DEVAPC_MSG("[DEVAPC] module resume.\n");
	return 0;
}

#ifdef DBG_ENABLE

#ifndef mt_secure_call
#define mt_secure_call(x1, x2, x3, x4, x5) ({\
	struct arm_smccc_res res;\
	arm_smccc_smc(x1, x2, x3, x4, x5, 0, 0, 0, &res);\
	res.a0; })
#endif

static ssize_t devapc_dbg_read(struct file *file, char __user *buffer,
	size_t count, loff_t *ppos)
{
	int ret;
	ssize_t retval = 0;
	char msg[256] = "DBG: dump devapc reg...\n";

	if (*ppos >= strlen(msg))
		return 0;

	pr_info("enter devapc dbg read ...\n");
	pr_info("call smc to ATF.\n");

	retval = simple_read_from_buffer(buffer, count, ppos, msg, strlen(msg));

	ret = mt_secure_call(MTK_SIP_LK_DAPC, 1, 0, 0, 0);
	if (ret == 0)
		pr_info("dump devapc reg success !\n");
	else
		pr_info("dump devapc reg failed !\n");

	return retval;
}

static ssize_t devapc_dbg_write(struct file *file, const char __user *buffer,
	size_t count, loff_t *data)
{
	char input[32];
	char *pinput = NULL;
	char *tmp = NULL;
	long i;
	int len = 0, ret = 0;
	long slave_type = 0, domain = 0, index = 0;

	pr_info("[DEVAPC] debugging...\n");
	len = (count < (sizeof(input) - 1)) ? count : (sizeof(input) - 1);
	if (copy_from_user(input, buffer, len)) {
		pr_info("[DEVAPC] copy from user failed!\n");
		return -EFAULT;
	}

	input[len] = '\0';
	pinput = input;

	if (sysfs_streq(input, "0")) {
		pr_info("[DEVAPC] One-Core Debugging: Disabled\n");
	} else if (sysfs_streq(input, "1")) {
		pr_info("[DEVAPC] One-Core Debugging: Enabled\n");
	} else {
		tmp = strsep(&pinput, " ");
		if (tmp != NULL)
			i = kstrtol(tmp, 10, &slave_type);
		else
			slave_type = E_DAPC_OTHERS_SLAVE;

		if (slave_type >= E_DAPC_OTHERS_SLAVE) {
			pr_info("[DEVAPC] wrong input slave type\n");
			return -EFAULT;
		}
		pr_info("[DEVAPC] slave_type = %lu\n", slave_type);

		tmp = strsep(&pinput, " ");
		if (tmp != NULL)
			i = kstrtol(tmp, 10, &domain);
		else
			domain = E_DOMAIN_OTHERS;

		if (domain >= E_DOMAIN_OTHERS) {
			pr_info("[DEVAPC] wrong input domain type\n");
			return -EFAULT;
		}
		pr_info("[DEVAPC] domain id = %lu\n", domain);

		tmp = strsep(&pinput, " ");
		if (tmp != NULL)
			i = kstrtol(tmp, 10, &index);
		else
			index = 0xFFFFFFFF;

		if (index > DEVAPC_TOTAL_SLAVES) {
			pr_info("[DEVAPC] wrong input index type\n");
			return -EFAULT;
		}
		pr_info("[DEVAPC] slave index = %lu\n", index);

		ret = mt_secure_call(MTK_SIP_LK_DAPC, slave_type,
				domain, index, 0);

		pr_info("dump devapc reg = 0x%x.\n", ret);
	}

	return count;
}
#endif

static int devapc_dbg_open(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct proc_ops devapc_dbg_fops = {
	.proc_open  = devapc_dbg_open,
#ifdef DBG_ENABLE
	.proc_write = devapc_dbg_write,
	.proc_read = devapc_dbg_read,
#else
	.proc_write = NULL,
	.proc_read = NULL,
#endif
};

static const struct of_device_id plat_devapc_dt_match[] = {
	{ .compatible = "mediatek,devapc" },
	{},
};

static struct platform_driver devapc_driver = {
	.probe = devapc_probe,
	.remove = devapc_remove,
	.suspend = devapc_suspend,
	.resume = devapc_resume,
	.driver = {
		.name = "devapc",
		.owner = THIS_MODULE,
		.of_match_table	= plat_devapc_dt_match,
	},
};

/*
 * devapc_init: module init function.
 */
static int __init devapc_init(void)
{
	int ret;

	DEVAPC_MSG("[DEVAPC] kernel module init.\n");

	ret = platform_driver_register(&devapc_driver);
	if (ret) {
		pr_info("[DEVAPC] Unable to register driver (%d)\n", ret);
		return ret;
	}

	g_devapc_ctrl = cdev_alloc();
	if (!g_devapc_ctrl) {
		pr_info("[DEVAPC] Failed to add devapc device! (%d)\n", ret);
		platform_driver_unregister(&devapc_driver);
		return ret;
	}
	g_devapc_ctrl->owner = THIS_MODULE;

	proc_create("devapc_dbg", 0664, NULL, &devapc_dbg_fops);
	/* 0664: (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH) */

	return 0;
}

/*
 * devapc_exit: module exit function.
 */
static void __exit devapc_exit(void)
{
	DEVAPC_MSG("[DEVAPC] DEVAPC module exit\n");
#ifdef CONFIG_MTK_HIBERNATION
	unregister_swsusp_restore_noirq_func(ID_M_DEVAPC);
#endif
}

/* Device APC no longer shares IRQ with EMI and
 * can be changed to use the earlier "arch_initcall"
 */
module_init(devapc_init);
module_exit(devapc_exit);
MODULE_LICENSE("GPL");
