// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 * Author: Kai Chieh Chuang <kaichieh.chuang@mediatek.com>
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/notifier.h>

#include <mtk_ccci_common.h>
#include "audio_messenger_ipi.h"
#include "audio_task.h"
#include "audio_speech_msg_id.h"
#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
#include <adsp_helper.h>
#endif
#if IS_ENABLED(CONFIG_MTK_SCP_AUDIO)
#include <scp_helper.h>
#include "audio_ipi_platform.h"
#endif


#define USIP_EMP_IOC_MAGIC 'D'
#define GET_USIP_EMI_SIZE _IOWR(USIP_EMP_IOC_MAGIC, 0xF0, unsigned long long)
#define GET_USIP_ADSP_PHONE_CALL_ENH_CONFIG _IOWR(USIP_EMP_IOC_MAGIC, 0xF1, unsigned long long)
#define SET_USIP_ADSP_PHONE_CALL_ENH_CONFIG _IOWR(USIP_EMP_IOC_MAGIC, 0xF2, unsigned long long)
#define GET_USIP_ADSP_PHONE_CALL_HWIF       _IOWR(USIP_EMP_IOC_MAGIC, 0xF3, unsigned long long)

#define GET_BIT(val, bit) ((val) & (0x1 << (bit)))

#define NUM_MPU_REGION 3

#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT) || IS_ENABLED(CONFIG_MTK_SCP_AUDIO)
static void usip_send_emi_info_to_dsp(void);
static void usip_send_emi_info_to_dsp_ble(void);
#endif

int EMI_TABLE[3][3] = {{0, 0, 0x30000}, {1, 0x30000, 0x8000}, {2, 0x38000, 0x28000}};

enum {
	SP_EMI_AP_USIP_PARAMETER,
	SP_EMI_ADSP_USIP_PHONECALL,
	SP_EMI_ADSP_USIP_SMARTPA
};

enum {
	SP_EMI_TYPE,
	SP_EMI_OFFSET,
	SP_EMI_SIZE
};

enum DSP_ENH_CFG_BIT_T {
	DSP_ENH_CFG_BIT_ADSP_ENH,
	DSP_ENH_CFG_BIT_CORE_SEPARATE,
	DSP_ENH_CFG_BIT_RUN_ON_SCP,
	DSP_ENH_CFG_BIT_FRM_BASE_SCH,
};

enum DSP_BLE_CFG_BIT_T {
	DSP_BLE_CFG_BIT_ENABLE,
};

struct usip_info {
	bool memory_ready;
	size_t memory_size;

	void *memory_area;
	dma_addr_t memory_addr;
	phys_addr_t addr_phy;

	unsigned int adsp_phone_call_enh_config;
	unsigned int adsp_phone_call_hwif;
	unsigned int adsp_ble_phone_call_config;
};

static struct usip_info usip;

static long usip_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	long size_for_spe = 0;
#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT) || IS_ENABLED(CONFIG_MTK_SCP_AUDIO)
	unsigned int prev_config = usip.adsp_phone_call_enh_config;
#endif

	pr_info("%s(), cmd 0x%x, arg %lu, memory_size = %ld addr_phy = 0x%lx\n",
		 __func__, cmd, arg, (long)usip.memory_size, (unsigned long)usip.addr_phy);

	size_for_spe = EMI_TABLE[SP_EMI_AP_USIP_PARAMETER][SP_EMI_SIZE];
	switch (cmd) {

	case GET_USIP_EMI_SIZE:
		if (!usip.memory_ready) {
			pr_info("no phy addr from ccci");
			ret = -ENODEV;
		} else if (copy_to_user((void __user *)arg, &size_for_spe,
			   sizeof(size_for_spe))) {
			pr_warn("Fail copy to user Ptr:%p, r_sz:%zu",
				(char *)&size_for_spe, sizeof(size_for_spe));
			ret = -1;
		}
		break;

	case GET_USIP_ADSP_PHONE_CALL_ENH_CONFIG:
		if (copy_to_user((void __user *)arg, &(usip.adsp_phone_call_enh_config),
			sizeof(usip.adsp_phone_call_enh_config))) {
			pr_info("%s(), Fail copy CALL_ENH_CONFIG to user Ptr: %x",
				__func__,
				usip.adsp_phone_call_enh_config);
			ret = -1;
		}
		break;

	case SET_USIP_ADSP_PHONE_CALL_ENH_CONFIG:
		if (copy_from_user(&(usip.adsp_phone_call_enh_config), (void __user *)arg,
			sizeof(usip.adsp_phone_call_enh_config))) {
			pr_info("%s(), Fail copy CALL_ENH_CONFIG from user Ptr: %lx",
				__func__, arg);
			ret = -1;
		}
		pr_info("%s(): in SET_USIP_ADSP_PHONE_CALL_ENH_CONFIG: %d",
			__func__,
			usip.adsp_phone_call_enh_config);
#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT) || IS_ENABLED(CONFIG_MTK_SCP_AUDIO)
		if ((GET_BIT(usip.adsp_phone_call_enh_config, DSP_ENH_CFG_BIT_CORE_SEPARATE))
		    > (GET_BIT(prev_config, DSP_ENH_CFG_BIT_CORE_SEPARATE)))
			usip_send_emi_info_to_dsp();
#endif
		break;

	case GET_USIP_ADSP_PHONE_CALL_HWIF:
		if (copy_to_user((void __user *)arg,
		    &(usip.adsp_phone_call_hwif),
		    sizeof(usip.adsp_phone_call_hwif))) {
			pr_info("%s(), Fail copy hwif to user Ptr: %p\n",
				__func__, (char *)&usip.adsp_phone_call_hwif);
			ret = -1;
		}
		break;

	default:
		pr_debug("%s(), default\n", __func__);
		break;
	}
	return ret;
}

#if IS_ENABLED(CONFIG_COMPAT)
static long usip_compat_ioctl(struct file *fp, unsigned int cmd,
			      unsigned long arg)
{
	long ret;

	if (!fp->f_op || !fp->f_op->unlocked_ioctl)
		return -ENOTTY;
	ret = fp->f_op->unlocked_ioctl(fp, cmd, arg);
	if (ret < 0)
		pr_err("%s(), fail, ret %ld, cmd 0x%x, arg %lu\n",
		       __func__, ret, cmd, arg);
	return ret;
}
#endif

int usip_mmap_data(struct usip_info *usip, struct vm_area_struct *area)
{
	long size;
	unsigned long offset;
	size_t align_bytes = PAGE_ALIGN(usip->memory_size);
	unsigned long pfn;
	int ret;

	pr_info("%s(), memory ready %d, size %zu, align_bytes %zu\n",
		__func__, usip->memory_ready, usip->memory_size, align_bytes);

	if (!usip->memory_ready)
		return -EBADFD;

	size = area->vm_end - area->vm_start;
	offset = area->vm_pgoff << PAGE_SHIFT;

	if ((size_t)size > align_bytes)
		return -EINVAL;
	if (offset > align_bytes - size)
		return -EINVAL;

	/*area->vm_ops = &usip_vm_ops_data;*/
	/*area->vm_private_data = usip;*/

	pfn = usip->addr_phy >> PAGE_SHIFT;
	/* ensure that memory does not get swapped to disk */
	vm_flags_set(area, VM_IO);
	/* ensure non-cacheable */
	area->vm_page_prot = pgprot_noncached(area->vm_page_prot);
	ret = remap_pfn_range(area, area->vm_start,
			pfn, size, area->vm_page_prot);
	if (ret)
		pr_err("%s(), ret %d, remap failed 0x%lx, phys_addr %pa -> vm_start 0x%lx\n",
		       __func__, ret, pfn,
		       &usip->addr_phy,
		       area->vm_start);
	/*Comment*/
	smp_mb();

	return ret;
}

static int usip_mmap(struct file *file, struct vm_area_struct *area)
{
	unsigned long offset;
	struct usip_info *usip = file->private_data;

	pr_debug("%s(), vm_pgoff 0x%lx\n",
		 __func__, area->vm_pgoff);

	offset = area->vm_pgoff << PAGE_SHIFT;
	switch (offset) {
	default:
		return usip_mmap_data(usip, area);
	}
	return 0;
}
static void usip_get_addr(void)
{
#if IS_ENABLED(CONFIG_MTK_ECCCI_DRIVER)
	int size_o = 0;
	phys_addr_t phys_addr;

	phys_addr_t srw_base;
	unsigned int srw_size;
	phys_addr_t r_rw_base;
	unsigned int r_rw_size;

	phys_addr = get_smem_phy_start_addr(0, SMEM_USER_RAW_USIP, &size_o);
	if (phys_addr == 0) {
		pr_info("%s(), cannot get emi addr from ccci", __func__);
		usip.memory_ready = false;
	} else {
		usip.memory_ready = true;

		get_md_resv_mem_info(&r_rw_base, &r_rw_size, &srw_base, &srw_size);
		pr_info("%s(), 0x%lx %d 0x%lx %d 0x%lx", __func__,
			(unsigned long)r_rw_base, r_rw_size, (unsigned long)srw_base, srw_size, (unsigned long)phys_addr);

		usip.memory_size = size_o;
		usip.addr_phy = phys_addr;
	}
#endif
}
#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT) || IS_ENABLED(CONFIG_MTK_SCP_AUDIO)
static void usip_send_emi_info_to_dsp_ble(void)
{
	int send_result = 0;
	struct ipi_msg_t ipi_msg;
	long long usip_emi_info[2]; //idx0 for addr, idx1 for size
	phys_addr_t offset = 0;

	if (!GET_BIT(usip.adsp_ble_phone_call_config, DSP_BLE_CFG_BIT_ENABLE)) {
		pr_info("%s(), adsp_ble_phone_call_config(%d) is close",
			__func__,
			usip.adsp_ble_phone_call_config);
		return;
	}

	if (usip.addr_phy == 0) {
		pr_info("%s(), cannot get emi addr from ccci", __func__);
		return;
	}

	offset = EMI_TABLE[SP_EMI_ADSP_USIP_PHONECALL][SP_EMI_OFFSET];
	usip_emi_info[0] = usip.addr_phy + offset;
	usip_emi_info[1] = EMI_TABLE[SP_EMI_ADSP_USIP_PHONECALL][SP_EMI_SIZE] +
			EMI_TABLE[SP_EMI_ADSP_USIP_SMARTPA][SP_EMI_SIZE];
	pr_debug("%s(), usip_emi_info[0] 0x%llx, usip_emi_info[1] 0x%llx\n",
		__func__, usip_emi_info[0], usip_emi_info[1]);

	ipi_msg.magic      = IPI_MSG_MAGIC_NUMBER;
	ipi_msg.task_scene = TASK_SCENE_BLECALLUL;
	ipi_msg.source_layer  = AUDIO_IPI_LAYER_FROM_KERNEL;
	ipi_msg.target_layer  = AUDIO_IPI_LAYER_TO_DSP;
	ipi_msg.data_type  = AUDIO_IPI_PAYLOAD;
	ipi_msg.ack_type   = AUDIO_IPI_MSG_BYPASS_ACK;
	ipi_msg.msg_id     = IPI_MSG_A2D_GET_EMI_ADDRESS;
	ipi_msg.param1     = sizeof(usip_emi_info);
	ipi_msg.param2     = 0;

	// [BLE UL] Send EMI Address to Hifi3 Via IPI
#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
	adsp_register_feature(BLE_CALL_UL_FEATURE_ID);
#endif
	send_result = audio_send_ipi_msg(
					 &ipi_msg, TASK_SCENE_BLECALLUL,
					 AUDIO_IPI_LAYER_TO_DSP,
					 AUDIO_IPI_PAYLOAD,
					 AUDIO_IPI_MSG_BYPASS_ACK,
					 IPI_MSG_A2D_GET_EMI_ADDRESS,
					 sizeof(usip_emi_info),
					 0,
					 (char *)&usip_emi_info);
#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
	adsp_deregister_feature(BLE_CALL_UL_FEATURE_ID);
#endif
	if (send_result != 0)
		pr_info("%s(), BLE UL scp_ipi send fail\n", __func__);
	else
		pr_debug("%s(), BLE UL scp_ipi send succeed\n", __func__);

	// [BLE DL] Send EMI Address to Hifi3 Via IPI
	ipi_msg.task_scene = TASK_SCENE_BLECALLDL;
#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
	adsp_register_feature(BLE_CALL_DL_FEATURE_ID);
#endif
	send_result = audio_send_ipi_msg(
					 &ipi_msg, TASK_SCENE_BLECALLDL,
					 AUDIO_IPI_LAYER_TO_DSP,
					 AUDIO_IPI_PAYLOAD,
					 AUDIO_IPI_MSG_BYPASS_ACK,
					 IPI_MSG_A2D_GET_EMI_ADDRESS,
					 sizeof(usip_emi_info),
					 0,
					 (char *)&usip_emi_info);
#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
	adsp_deregister_feature(BLE_CALL_DL_FEATURE_ID);
#endif
	if (send_result != 0)
		pr_info("%s(), BLE DL scp_ipi send fail\n", __func__);
	else
		pr_debug("%s(), BLE DL scp_ipi send succeed\n", __func__);
}

static void usip_send_emi_info_to_dsp(void)
{
	int send_result = 0;
	struct ipi_msg_t ipi_msg;
	long long usip_emi_info[2]; //idx0 for addr, idx1 for size
	phys_addr_t offset = 0;

	if (!GET_BIT(usip.adsp_phone_call_enh_config, DSP_ENH_CFG_BIT_ADSP_ENH)) {
		pr_info("%s(), adsp_phone_call_enh_config(%d) is close",
			__func__,
			usip.adsp_phone_call_enh_config);
		return;
	}

	if (!usip.memory_ready) {
		usip_get_addr();
		if (!usip.memory_ready) {
			pr_info("%s(), cannot get emi addr from ccci",
				__func__);
			return;
		}
	}

	offset = EMI_TABLE[SP_EMI_ADSP_USIP_PHONECALL][SP_EMI_OFFSET];
	usip_emi_info[0] = usip.addr_phy + offset;
	usip_emi_info[1] = EMI_TABLE[SP_EMI_ADSP_USIP_PHONECALL][SP_EMI_SIZE] +
		EMI_TABLE[SP_EMI_ADSP_USIP_SMARTPA][SP_EMI_SIZE];

	pr_info("%s(), usip_emi_info[0] 0x%llx, usip_emi_info[1] 0x%llx\n",
		__func__, usip_emi_info[0], usip_emi_info[1]);

	ipi_msg.magic      = IPI_MSG_MAGIC_NUMBER;
	ipi_msg.task_scene = TASK_SCENE_PHONE_CALL;
	ipi_msg.source_layer  = AUDIO_IPI_LAYER_FROM_KERNEL;
	ipi_msg.target_layer  = AUDIO_IPI_LAYER_TO_DSP;
	ipi_msg.data_type  = AUDIO_IPI_PAYLOAD;
	ipi_msg.ack_type   = AUDIO_IPI_MSG_BYPASS_ACK;
	ipi_msg.msg_id     = IPI_MSG_A2D_GET_EMI_ADDRESS;
	ipi_msg.param1     = sizeof(usip_emi_info);
	ipi_msg.param2     = 0;

	/* [Call_Main]Send EMI Address to Hifi3 Via IPI*/
#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
	if (!GET_BIT(usip.adsp_phone_call_enh_config, DSP_ENH_CFG_BIT_RUN_ON_SCP))
		adsp_register_feature(VOICE_CALL_FEATURE_ID);
#endif

	send_result = audio_send_ipi_msg(
					 &ipi_msg, TASK_SCENE_PHONE_CALL,
					 AUDIO_IPI_LAYER_TO_DSP,
					 AUDIO_IPI_PAYLOAD,
					 AUDIO_IPI_MSG_BYPASS_ACK,
					 IPI_MSG_A2D_GET_EMI_ADDRESS,
					 sizeof(usip_emi_info),
					 0,
					 (char *)&usip_emi_info);

#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
	if (!GET_BIT(usip.adsp_phone_call_enh_config, DSP_ENH_CFG_BIT_RUN_ON_SCP))
		adsp_deregister_feature(VOICE_CALL_FEATURE_ID);
#endif

	if (send_result != 0)
		pr_info("%s(), scp_ipi send fail\n", __func__);
	else
		pr_debug("%s(), scp_ipi send succeed\n", __func__);

	/* [Call_Sub] Send EMI Address to Hifi3 Via IPI*/
	if (GET_BIT(usip.adsp_phone_call_enh_config, DSP_ENH_CFG_BIT_CORE_SEPARATE)) {
		ipi_msg.task_scene = TASK_SCENE_PHONE_CALL_SUB;

#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
		if (!GET_BIT(usip.adsp_phone_call_enh_config, DSP_ENH_CFG_BIT_RUN_ON_SCP))
			adsp_register_feature(VOICE_CALL_FEATURE_ID);
#endif

		send_result = audio_send_ipi_msg(
					 &ipi_msg, TASK_SCENE_PHONE_CALL_SUB,
					 AUDIO_IPI_LAYER_TO_DSP,
					 AUDIO_IPI_PAYLOAD,
					 AUDIO_IPI_MSG_BYPASS_ACK,
					 IPI_MSG_A2D_GET_EMI_ADDRESS,
					 sizeof(usip_emi_info),
					 0,
					 (char *)&usip_emi_info);

#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
		if (!GET_BIT(usip.adsp_phone_call_enh_config, DSP_ENH_CFG_BIT_RUN_ON_SCP))
			adsp_deregister_feature(VOICE_CALL_FEATURE_ID);
#endif

		if (send_result != 0)
			pr_info("%s(), scp_ipi send sub fail\n", __func__);
		else
			pr_debug("%s(), scp_ipi send sub succeed\n", __func__);
	}
}
#endif /* end of CONFIG_MTK_AUDIODSP_SUPPORT */

#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
static int audio_call_event_receive(struct notifier_block *this,
				    unsigned long event,
				    void *ptr)
{
	switch (event) {
	case ADSP_EVENT_STOP:
		break;
	case ADSP_EVENT_READY:
		usip_send_emi_info_to_dsp();
		usip_send_emi_info_to_dsp_ble();
		break;
	default:
		pr_info("event %lu err", event);
	}
	return 0;
}

static struct notifier_block audio_call_notifier = {
	.notifier_call = audio_call_event_receive,
	.priority = VOICE_CALL_FEATURE_PRI,
};
#endif

#if IS_ENABLED(CONFIG_MTK_SCP_AUDIO)
static int audio_call_event_receive_scp(struct notifier_block *this,
				    unsigned long event,
				    void *ptr)
{
	switch (event) {
	case SCP_EVENT_STOP:
		break;
	case SCP_EVENT_READY:
		usip_send_emi_info_to_dsp();
		break;
	default:
		pr_info("event %lu err", event);
	}
	return 0;
}

static struct notifier_block audio_call_notifier_scp = {
	.notifier_call = audio_call_event_receive_scp,
};
#endif

static int usip_open(struct inode *inode, struct file *file)
{

	file->private_data = &usip;

	if (!usip.memory_ready) {
		usip_get_addr();
#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT) || IS_ENABLED(CONFIG_MTK_SCP_AUDIO)
		usip_send_emi_info_to_dsp();
		usip_send_emi_info_to_dsp_ble();
#endif
	}

	return 0;
}

static const struct file_operations usip_fops = {
	.owner = THIS_MODULE,
	.open = usip_open,
	.unlocked_ioctl = usip_ioctl,
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl = usip_compat_ioctl,
#endif
	.mmap = usip_mmap,
};


static struct miscdevice usip_miscdevice = {
	.minor      = MISC_DYNAMIC_MINOR,
	.name       = "usip",
	.fops       = &usip_fops,
};

static const struct of_device_id usip_dt_match[] = {
	{ .compatible = "mediatek,speech-usip-mem", },
	{},
};

static int speech_usip_dev_probe(struct platform_device *pdev)
{
	/* get adsp phone call config*/
	int ret = of_property_read_u32(pdev->dev.of_node,
				   "adsp-phone-call-enh-enable",
				   &(usip.adsp_phone_call_enh_config));
	if (ret != 0)
		pr_info("%s adsp_phone_call_enh_enable error\n", __func__);
	else
		pr_debug("%s adsp_phone_call_enh_enable is %d\n",
				__func__, usip.adsp_phone_call_enh_config);

	ret = of_property_read_u32(pdev->dev.of_node,
				   "adsp-phone-call-hwif",
				   &(usip.adsp_phone_call_hwif));
	if (ret != 0)
		pr_info("%s adsp-phone-call-hwif error\n", __func__);
	else
		pr_debug("%s adsp-phone-call-hwif is %d\n",
				__func__, usip.adsp_phone_call_hwif);

	ret = of_property_read_u32(pdev->dev.of_node,
				   "adsp-ble-phone-call-enable",
				   &(usip.adsp_ble_phone_call_config));
	if (ret != 0)
		pr_info("%s adsp_ble_phone_call_enable error\n", __func__);
	else
		pr_debug("%s adsp_ble_phone_call_enable is %d\n",
				__func__, usip.adsp_ble_phone_call_config);

#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT) || IS_ENABLED(CONFIG_MTK_SCP_AUDIO)
	usip_send_emi_info_to_dsp();
	usip_send_emi_info_to_dsp_ble();
#endif
	return 0;
}

static int speech_usip_dev_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver speech_usip_mem = {
	.driver = {
		   .name = "speech-usip-mem",
		   .owner = THIS_MODULE,
		   .of_match_table = usip_dt_match,
	},
	.probe = speech_usip_dev_probe,
	.remove = speech_usip_dev_remove,
};

static int __init usip_init(void)
{
	int ret;

	usip.memory_ready = false;
	usip.memory_size = 0;
	usip.addr_phy = 0;

	ret = misc_register(&usip_miscdevice);
	if (ret) {
		pr_err("%s(), cannot register miscdev on minor %d, ret %d\n",
		       __func__, usip_miscdevice.minor, ret);
		ret = -ENODEV;
	}

	/* init usip info */
	usip.memory_addr = 0x11220000L;


#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
	adsp_register_notify(&audio_call_notifier);
#endif
#if IS_ENABLED(CONFIG_MTK_SCP_AUDIO)
	if (is_audio_scp_support())
		scp_A_register_notify(&audio_call_notifier_scp);
#endif
	ret = platform_driver_register(&speech_usip_mem);

	return ret;
}

static void __exit usip_exit(void)
{
	misc_deregister(&usip_miscdevice);
}

module_init(usip_init);
module_exit(usip_exit);

MODULE_DESCRIPTION("Mediatek uSip memory control");
MODULE_LICENSE("GPL v2");

