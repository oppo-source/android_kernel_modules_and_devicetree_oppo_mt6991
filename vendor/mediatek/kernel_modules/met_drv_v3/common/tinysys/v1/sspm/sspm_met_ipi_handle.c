// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
/*****************************************************************************
 * headers
 *****************************************************************************/
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/signal.h>
#include <linux/semaphore.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <asm/io.h>  /* for ioremap and iounmap */

#include "sspm_met_log.h" /* for sspm_ipidev_symbol */
#include "sspm_met_ipi_handle.h"
#include "interface.h"
#include "core_plf_init.h"
#include "tinysys_sspm.h"
#include "tinysys_mgr.h" /* for ondiemet_module */
#include "met_kernel_symbol.h"

#include <linux/scmi_protocol.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/device.h>

#ifdef MET_SCMI
#ifndef __TINYSYS_SCMI_H__
#define __TINYSYS_SCMI_H__
#include "tinysys-scmi.h"
#endif
#endif

#define MS_TO_JIFFIES(TIME)     ((unsigned long)(TIME) / (MSEC_PER_SEC / HZ))

/*****************************************************************************
 * define declaration
 *****************************************************************************/


/*****************************************************************************
 * struct & enum declaration
 *****************************************************************************/


/*****************************************************************************
 * external function declaration
 *****************************************************************************/
extern unsigned int met_get_chip_id(void);
extern char *met_get_platform(void);


/*****************************************************************************
 * internal function declaration
 *****************************************************************************/
static void _log_done_cb(const void *p);

static struct completion SSPM_ACK_comp;
static struct completion SSPM_CMD_comp;
struct scmi_tinysys_info_st *tinfo = NULL;
int feature_id = 3; // scmi-met = <3>, feature_id will be reloaded from .dts

static int _sspm_recv_thread(void *data);


/*****************************************************************************
 * external variable declaration
 *****************************************************************************/
int sspm_buf_available;
EXPORT_SYMBOL(sspm_buf_available);


/*****************************************************************************
 * internal variable declaration
 *****************************************************************************/
static unsigned int recv_buf[4], recv_buf_copy[4];
static unsigned int log_size;
static struct task_struct *_sspm_recv_task;
static int sspm_ipi_thread_started;
static int sspm_buffer_dumping;
static int sspm_recv_thread_comp;
static int sspm_run_mode = SSPM_RUN_NORMAL;

#ifndef MET_SCMI
extern int _sspm_log_discard;
#ifndef SSPM_VERSION_V2
#include "sspm_ipi.h"
static struct ipi_action ondiemet_sspm_isr;
#else
#include "core_plf_init.h"
uint32_t rdata = 0;
uint32_t ridx, widx, wlen;
uint32_t ackdata = 0;
#endif /* !SSPM_VERSION_V2 */
#endif /* !MET_SCMI */

/*****************************************************************************
 * internal function ipmlement
 *****************************************************************************/

#ifdef MET_SCMI
void MET_handler(u32 r_feature_id, scmi_tinysys_report* report)
{
    u32 cmd = 0;

    MET_PRINTF_D("\x1b[1;33m ==> p1: 0x%x \033[0m\n", report->p1);
    MET_PRINTF_D("\x1b[1;33m ==> p2: 0x%x \033[0m\n", report->p2);
    MET_PRINTF_D("\x1b[1;33m ==> p3: 0x%x \033[0m\n", report->p3);
    MET_PRINTF_D("\x1b[1;33m ==> p4: 0x%x \033[0m\n", report->p4);

    recv_buf[0] = report->p1;
    recv_buf[1] = report->p2;
    recv_buf[2] = report->p3;
    recv_buf[3] = report->p4;
    memcpy(recv_buf_copy, recv_buf, sizeof(recv_buf_copy));

    cmd = report->p1 & MET_SUB_ID_MASK;
	MET_PRINTF_D("\x1b[1;33m ==> cmd: 0x%x \033[0m\n", cmd);
	switch (cmd) {
	case MET_RESP_MD2AP:
	    sspm_buffer_dumping = 0;
		break;

    case MET_SSPM_ACK:
        complete(&SSPM_ACK_comp);
        return;

	default:
		break;
	}

	MET_PRINTF_D("\x1b[1;33m ==> MET_handler done, while(1) wake up \033[0m\n");
	complete(&SSPM_CMD_comp);
}
#endif

#ifdef MET_SCMI
int scmi_tinysys_to_sspm_command( u32 feature_id,
                                              unsigned int *buffer,
                                              unsigned int slot,	// size of buffer
                                              unsigned int *retbuf,
                                              unsigned int retslot,	// size of retbuf
                                              unsigned int timeout)
{
    int ret = 0;
    int __buffer[5] = {0, 0, 0, 0, 0};

    if (slot > 5) {		// 5 is the upper bound of buffer[] sent by SCMI
        PR_BOOTMSG("slot > 5, it's out of range!!\n");
        return -1; // FAIL
    }

    if (retslot > 3) {	// 3 is the upper bound of retbuf[] sent by SCMI
        PR_BOOTMSG("retslot > 3, it's out of range!!\n");
        return -1; // FAIL
    }

    if (tinfo == NULL) {
        PR_BOOTMSG("scmi protocol handle is NULL!!\n");
        return -1; // FAIL
    }

	memcpy(__buffer, buffer, slot * sizeof(unsigned int));
    MET_PRINTF_D("\x1b[1;34m ==> __buffer[0~4]: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x \033[0m \n",
								__buffer[0], __buffer[1], __buffer[2], __buffer[3], __buffer[4]);
	if (scmi_tinysys_common_set_symbol) {
    	ret = scmi_tinysys_common_set_symbol(tinfo->ph, feature_id,
                                  __buffer[0], __buffer[1], __buffer[2], __buffer[3], __buffer[4]);
	} else {
		PR_BOOTMSG("[MET] [%s,%d] scmi_tinysys_common_set is not linked!\n", __FILE__, __LINE__);
		return -1;
	}
    if (ret != 0) {
		PR_BOOTMSG("scmi_tinysys_common_set fail(%d)\n", ret);
    } else {
        long wait_ret = 0;
        MET_PRINTF_D("\x1b[1;34m ==> wait for (0x%x) completion , timeout: %d\033[0m \n", __buffer[0], timeout);
        wait_ret = wait_for_completion_killable_timeout(&SSPM_ACK_comp, timeout);

        if (wait_ret < 0) { // Interrupt occurred, ret = -ERESTARTSYS
            PR_BOOTMSG("wait_for_completion_killable_timeout() is interrupted! (%ld)\n", wait_ret);
            ret = -1; // FAIL
        } else if (wait_ret == 0) { // 0 if timed out, positive (at least 1, or number of jiffies left till timeout) if completed.
            PR_BOOTMSG("scmi_tinysys_common_set wait complete time out (%ld)\n", wait_ret);
            ret = -1; // FAIL
        } else {
            MET_PRINTF_D("\x1b[1;34m ==> (0x%x) SSPM ACK with retval: 0x%x, 0x%x, 0x%x, 0x%x \033[0m\n", __buffer[0],
    									 recv_buf_copy[0], recv_buf_copy[1], recv_buf_copy[2], recv_buf_copy[3]);

            memcpy(retbuf, recv_buf_copy + 1, retslot * sizeof(unsigned int)); // recv_buf[0] is COMMAND
        }
    }

    return ret;
}
#endif

#ifdef MET_SCMI
void start_sspm_ipi_recv_thread(void)
{
    init_completion(&SSPM_ACK_comp);
    init_completion(&SSPM_CMD_comp);

	if (get_scmi_tinysys_info_symbol) {
		tinfo = get_scmi_tinysys_info_symbol();
		if (tinfo == NULL) {
			PR_BOOTMSG("[MET] [%s,%d] scmi protocol handle is NULL!!\n", __FILE__, __LINE__);
			return; // FAIL
		}
	} else {
		PR_BOOTMSG("[MET] [%s,%d] get_scmi_tinysys_info is not linked!\n", __FILE__, __LINE__);
		return;
	}
    of_property_read_u32(tinfo->sdev->dev.of_node, "scmi-met", &feature_id);

	if (scmi_tinysys_event_notify_symbol) {
    	scmi_tinysys_event_notify_symbol(feature_id, 1 /*1: ENABLE*/);
	} else {
		PR_BOOTMSG("[MET] [%s,%d] scmi_tinysys_event_notify is not linked!\n", __FILE__, __LINE__);
		return;
	}
	if (scmi_tinysys_register_event_notifier_symbol) {
    	scmi_tinysys_register_event_notifier_symbol(feature_id, MET_handler);
	} else {
		PR_BOOTMSG("[MET] [%s,%d] scmi_tinysys_register_event_notifier is not linked!\n", __FILE__, __LINE__);
		return;
	}
    MET_PRINTF_D(" ==> feature_id: %d \n", feature_id);

	if (sspm_ipi_thread_started != 1) {
		sspm_recv_thread_comp = 0;
		_sspm_recv_task = kthread_run(_sspm_recv_thread,
						NULL, "sspmsspm_recv");
		if (IS_ERR(_sspm_recv_task)) {
			pr_debug("MET: Can not create sspmsspm_recv\n");
		} else {
			sspm_ipi_thread_started = 1;
		}
	}
}

#else

#ifdef SSPM_VERSION_V2
static int met_ipi_cb(unsigned int ipi_id, void *prdata, void *data, unsigned int len)
{
	unsigned int *cmd_buf = (unsigned int *)data;
	unsigned int cmd;
	int ret;

	if (sspm_recv_thread_comp == 1) {
		PR_BOOTMSG("%s %d\n", __FUNCTION__, __LINE__);
		return 0;
	}

	cmd = cmd_buf[0] & MET_SUB_ID_MASK;

	switch (cmd) {
	case MET_DUMP_BUFFER:	/* mbox 1: start index; 2: size */
		sspm_buffer_dumping = 1;
		ridx = cmd_buf[1];
		widx = cmd_buf[2];
		log_size = cmd_buf[3];
		break;

	case MET_CLOSE_FILE:	/* no argument */
		/* do close file */
		ridx = cmd_buf[1];
		widx = cmd_buf[2];
		if (widx < ridx) {	/* wrapping occurs */
			wlen = log_size - ridx;
			sspm_log_req_enq((char *)(sspm_log_virt_addr) + (ridx << 2),
					wlen * 4, _log_done_cb, (void *)1);
			sspm_log_req_enq((char *)(sspm_log_virt_addr),
					widx * 4, _log_done_cb, (void *)1);
		} else {
			wlen = widx - ridx;
			sspm_log_req_enq((char *)(sspm_log_virt_addr) + (ridx << 2),
					wlen * 4, _log_done_cb, (void *)1);
		}
		ret = sspm_log_stop();
		break;

	case MET_RESP_MD2AP:
		break;

	default:
		break;
	}
	return 0;
}

#endif /* SSPM_VERSION_V2 */

void start_sspm_ipi_recv_thread(void)
{
#ifdef SSPM_VERSION_V2
	int ret;

	if (sspm_ipidev_symbol == NULL) {
		sspm_ipidev_symbol = &sspm_ipidev;
	}

	if (sspm_ipidev_symbol == NULL) {
		return;
	}


	// tinysys send ipi to APSYS
	ret = mtk_ipi_register(sspm_ipidev_symbol, IPIR_C_MET, met_ipi_cb,
				NULL, (void *)&recv_buf);

	if (ret) {
		PR_BOOTMSG("[MET] ipi_register:%d failed:%d\n", IPIR_C_MET, ret);
	} else {
		PR_BOOTMSG("mtk_ipi_register IPIR_C_MET success \n");
	}

	// APSYS send ipi to tinysys
	ret = mtk_ipi_register(sspm_ipidev_symbol, IPIS_C_MET, NULL,
				NULL, (void *)&ackdata);

	if (ret)
		pr_debug("[MET] ipi_register:%d failed:%d\n", IPIS_C_MET, ret);
#endif /* SSPM_VERSION_V2 */

	if (sspm_ipi_thread_started != 1) {
		sspm_recv_thread_comp = 0;
		_sspm_recv_task =
		    kthread_run(_sspm_recv_thread, NULL, "ondiemet_sspm_recv");
		if (IS_ERR(_sspm_recv_task))
			pr_debug("MET: Can not create ondiemet_sspm_recv\n");
		else
			sspm_ipi_thread_started = 1;
	}
}

#endif /* MET_SCMI */

#ifdef MET_SCMI
void stop_sspm_ipi_recv_thread(void)
{
	if (_sspm_recv_task) {
		sspm_recv_thread_comp = 1;
		complete(&SSPM_CMD_comp);
		kthread_stop(_sspm_recv_task);
		_sspm_recv_task = NULL;
		sspm_ipi_thread_started = 0;
	}
}
#else /* NOT MET_SCMI, SSPM_VERSION_V0 AND V1 AND V2 */
void stop_sspm_ipi_recv_thread(void)
{
	if (_sspm_recv_task) {
		sspm_recv_thread_comp = 1;
#ifndef SSPM_VERSION_V2
		sspm_ipi_recv_complete(IPI_ID_TST1);
#endif
		kthread_stop(_sspm_recv_task);
		_sspm_recv_task = NULL;
		sspm_ipi_thread_started = 0;
#ifndef SSPM_VERSION_V2
		sspm_ipi_recv_unregistration(IPI_ID_TST1);
#else
		mtk_ipi_unregister(sspm_ipidev_symbol, IPIR_C_MET);
		mtk_ipi_unregister(sspm_ipidev_symbol, IPIS_C_MET);
#endif
	}
}

#endif /* MET_SCMI */

#if defined(SSPM_VERSION_V0) || defined(SSPM_VERSION_V1) || defined(SSPM_VERSION_V2) /* SSPM_VERSION_V0 AND V1 AND V2 */
int met_ipi_to_sspm_command(void *buffer, int slot, unsigned int *retbuf, int retslot)
{
	int ret;

#ifndef SSPM_VERSION_V2
	ret = sspm_ipi_send_sync(IPI_ID_MET, IPI_OPT_WAIT, buffer, slot, retbuf, retslot);
#else
	if(!sspm_ipidev_symbol)
		ret = -1;
	else {
		ret = mtk_ipi_send_compl(sspm_ipidev_symbol, IPIS_C_MET, IPI_SEND_WAIT, buffer, slot, 2000);
		*retbuf = ackdata;
	}
#endif /* SSPM_VERSION_V2 */
	if (ret != 0)
		pr_debug("met_ipi_to_sspm_command error(%d)\n", ret);

	dump_stack();
	return ret;
}
EXPORT_SYMBOL(met_ipi_to_sspm_command);
#endif /* SSPM_VERSION_V0 AND V1 AND V2 */



void sspm_start(void)
{
	int ret = 0;
	unsigned int rdata = 0;
	unsigned int ipi_buf[4] = {0, 0, 0, 0};
	const char* platform_name = NULL;
	unsigned int platform_id = 0;

	MET_PRINTF_D("\x1b[1;31m ==> sspm_start \033[0m\n");

	/* clear DRAM buffer */
	if (sspm_log_virt_addr != NULL) {
		memset_io((void *)sspm_log_virt_addr, 0, sspm_buffer_size);
	} else {
		return;
	}

	platform_name = met_get_platform();
	if (platform_name) {
		char buf[5];

		memset(buf, 0x0, 5);
		memcpy(buf, &platform_name[2], 4);

		ret = kstrtouint(buf, 10, &platform_id);
	}

#ifdef MET_SCMI
	/* send DRAM physical address */
	ipi_buf[0] = MET_MAIN_ID | MET_BUFFER_INFO;
	ipi_buf[1] = (ret == 0) ? platform_id : 0;
	ipi_buf[2] = (unsigned int)(sspm_log_phy_addr & 0xFFFFFFFF); /* addr low */
	ipi_buf[3] = (unsigned int)(sspm_log_phy_addr >> 32); /* addr high*/

	ret = met_scmi_to_sspm_command(ipi_buf, sizeof(ipi_buf)/sizeof(unsigned int), &rdata, 1);
#else

	/* send DRAM physical address */
	ipi_buf[0] = MET_MAIN_ID | MET_BUFFER_INFO;
	ipi_buf[1] = (unsigned int)sspm_log_phy_addr; /* address */
	if (ret == 0)
		ipi_buf[2] = platform_id;
	else
		ipi_buf[2] = 0;

	ipi_buf[3] = 0;

	ret = met_ipi_to_sspm_command((void *)ipi_buf, 0, &rdata, 1);
#endif

	/* start ondiemet now */
	ipi_buf[0] = MET_MAIN_ID | MET_OP | MET_OP_START;
	ipi_buf[1] = ondiemet_module[ONDIEMET_SSPM] ;
	ipi_buf[2] = SSPM_LOG_FILE;
	ipi_buf[3] = SSPM_RUN_NORMAL;

#ifdef MET_SCMI
	ret = met_scmi_to_sspm_command(ipi_buf, sizeof(ipi_buf)/sizeof(unsigned int), &rdata, 1);
#else
	ret = met_ipi_to_sspm_command((void *)ipi_buf, 0, &rdata, 1);
#endif

}


void sspm_stop(void)
{
	int ret = 0;
	unsigned int rdata = 0;
	unsigned int ipi_buf[4] = {0, 0, 0, 0};
	unsigned int chip_id = 0;

	MET_PRINTF_D("\x1b[1;31m ==> sspm_stop \033[0m\n");

	chip_id = met_get_chip_id();
	if (sspm_buf_available == 1) {
		ipi_buf[0] = MET_MAIN_ID | MET_OP | MET_OP_STOP;
		ipi_buf[1] = chip_id;
#ifdef MET_SCMI
		ret = met_scmi_to_sspm_command(ipi_buf, sizeof(ipi_buf)/sizeof(unsigned int), &rdata, 1);
#else
		ret = met_ipi_to_sspm_command(ipi_buf, 0, &rdata, 1);
#endif

	}
}


void sspm_extract(void)
{
	int ret;
	unsigned int rdata;
	unsigned int ipi_buf[4] = {0, 0, 0, 0};
	int count;

	MET_PRINTF_D("\x1b[1;31m ==> sspm_extract \033[0m\n");

	count = 20;
	if (sspm_buf_available == 1) {
		while ((sspm_buffer_dumping == 1) && (count != 0)) {
			msleep(50);
			count--;
		}
		ipi_buf[0] = MET_MAIN_ID | MET_OP | MET_OP_EXTRACT;
#ifdef MET_SCMI
		ret = met_scmi_to_sspm_command(ipi_buf, sizeof(ipi_buf)/sizeof(unsigned int), &rdata, 1);
#else
		ret = met_ipi_to_sspm_command((void *)ipi_buf, 0, &rdata, 1);
#endif


	}

	if (sspm_run_mode == SSPM_RUN_NORMAL) {
		ondiemet_module[ONDIEMET_SSPM]  = 0;
	}
}


void sspm_flush(void)
{
	int ret;
	unsigned int rdata;
	unsigned int ipi_buf[4] = {0, 0, 0, 0};

	MET_PRINTF_D("\x1b[1;31m ==> sspm_flush \033[0m\n");

	if (sspm_buf_available == 1) {
		ipi_buf[0] = MET_MAIN_ID | MET_OP | MET_OP_FLUSH;
#ifdef MET_SCMI
		ret = met_scmi_to_sspm_command((void *)ipi_buf, sizeof(ipi_buf)/sizeof(unsigned int), &rdata, 1);
#else
		ret = met_ipi_to_sspm_command((void *)ipi_buf, 0, &rdata, 1);
#endif

	}

	if (sspm_run_mode == SSPM_RUN_NORMAL) {
		ondiemet_module[ONDIEMET_SSPM] = 0;
	}
}


int met_scmi_to_sspm_command(
	unsigned int *buffer,
	int slot,
	unsigned int *retbuf,
	int retslot)
{
	int ret = 0;

#ifdef MET_SCMI
	ret = scmi_tinysys_to_sspm_command(feature_id, buffer, slot, retbuf, retslot, MS_TO_JIFFIES(2000));
#endif

	if (ret != 0) {
		pr_debug("met_AP_to_sspm_command error(%d)\n", ret);
	}

	return ret;
}
EXPORT_SYMBOL(met_scmi_to_sspm_command);


int met_scmi_to_sspm_command_async(
	unsigned int *buffer,
	int slot,
	unsigned int *retbuf,
	int retslot)
{
	int ret = 0;

	return ret;
}
EXPORT_SYMBOL(met_scmi_to_sspm_command_async);


unsigned int met_scmi_to_sspm_resrc_request(unsigned int on){

	unsigned int ipi_buf[2] = {0, 0};
	unsigned int rdata = 0;
	unsigned int res = 0;

	if (sspm_buffer_size == 0)
		return 0;

	if (met_config_list & (1<<RESOURCE_CTRL)) {
		ipi_buf[0] = MET_MAIN_ID | MET_RESRC_REQ_AP2MD ;
		ipi_buf[1] = on;
		res = met_scmi_to_sspm_command((void *)ipi_buf, sizeof(ipi_buf)/sizeof(unsigned int), &rdata, 1);
		return rdata;
	}
	return res;
}
EXPORT_SYMBOL(met_scmi_to_sspm_resrc_request);

/*****************************************************************************
 * internal function ipmlement
 *****************************************************************************/
static void _log_done_cb(const void *p)
{
	unsigned int ret;
	unsigned int rdata = 0;
	unsigned int ipi_buf[4] = {0, 0, 0, 0};
	unsigned int opt = (p != NULL);

	MET_PRINTF_D("\x1b[1;31m ==> _log_done_cb ++\033[0m\n");

	if (opt == 0) {
		MET_PRINTF_D("\x1b[1;31m ==> Send MET_RESP_AP2MD --> SSPM \033[0m\n");
		ipi_buf[0] = MET_MAIN_ID | MET_RESP_AP2MD;
		ipi_buf[1] = MET_DUMP_BUFFER;
		ipi_buf[2] = 0;
		ipi_buf[3] = 0;
#ifdef MET_SCMI
		ret = met_scmi_to_sspm_command((void *)ipi_buf, sizeof(ipi_buf)/sizeof(unsigned int), &rdata, 1);
#else
		ret = met_ipi_to_sspm_command((void *)ipi_buf, 0, &rdata, 1);
#endif
    }
	MET_PRINTF_D("\x1b[1;31m ==> _log_done_cb --\033[0m\n");
}

#ifdef MET_SCMI
static int _sspm_recv_thread(void *data)
{
    u32 cmd = 0;
	int ret = 0;
	unsigned int ridx, widx, wlen;

	do {
		MET_PRINTF_D("\x1b[1;34m ==> (0x%x) command done, then to wait receiving next SSPM CMD\033[0m\n", recv_buf_copy[0]);
		ret = wait_for_completion_killable(&SSPM_CMD_comp);
		MET_PRINTF_D("\x1b[1;34m ==> release to do SSPM CMD (0x%x) \033[0m\n", recv_buf_copy[0]);

		if (ret) {
			// skip cmd handling if receive fail
			continue;
		}

		if (sspm_recv_thread_comp == 1) {
			while (!kthread_should_stop()) {
				;
			}
			return 0;
		}

		/* heavyweight cmd handling (not support reply_data) */
		cmd = recv_buf_copy[0] & MET_SUB_ID_MASK;
		MET_PRINTF_D("\x1b[1;33m ==> cmd: 0x%x \033[0m\n", cmd);
		switch (cmd) {
		case MET_DUMP_BUFFER:	/* mbox 1: start index; 2: size */
			sspm_buffer_dumping = 1;
			ridx = recv_buf_copy[1];
			widx = recv_buf_copy[2];
			log_size = recv_buf_copy[3];

			MET_PRINTF_D("\x1b[1;34m ==> ridx: %d (0x%x) \033[0m\n", ridx, ridx);
			MET_PRINTF_D("\x1b[1;34m ==> widx: %d (0x%x) \033[0m\n", widx, widx);
			MET_PRINTF_D("\x1b[1;34m ==> log_size: %d \033[0m\n", log_size);
			MET_PRINTF_D("\x1b[1;34m ==> sspm_log_virt_addr: 0x%x \033[0m\n", (char *)sspm_log_virt_addr);
			MET_PRINTF_D("==> ==================================\n");

			if (widx < ridx) {	/* wrapping occurs */
				wlen = log_size - ridx;

                MET_PRINTF_D("\x1b[1;36m ==> #1 base: addr + ridx << 2 = 0x%x \033[0m\n", (char *)(sspm_log_virt_addr) + (ridx << 2));
                MET_PRINTF_D("\x1b[1;36m ==> #1 len: wlen * 4 = 0x%x \033[0m\n", wlen * 4);
                MET_PRINTF_D("\x1b[1;36m ==> #1 end: 0x%x \033[0m\n", (char *)(sspm_log_virt_addr) + (ridx << 2) + (wlen * 4));

				sspm_log_req_enq((char *)(sspm_log_virt_addr) + (ridx << 2),
						wlen * 4, _log_done_cb, (void *)1);

                MET_PRINTF_D("\x1b[1;36m ==> #2 base: addr = 0x%x \033[0m\n", (char *)(sspm_log_virt_addr));
                MET_PRINTF_D("\x1b[1;36m ==> #2 len: widx * 4 = 0x%x \033[0m\n", widx * 4);
                MET_PRINTF_D("\x1b[1;36m ==> #2 end: 0x%x \033[0m\n", (char *)(sspm_log_virt_addr) + (widx * 4));

				sspm_log_req_enq((char *)(sspm_log_virt_addr),
						widx * 4, _log_done_cb, (void *)0);
			} else {
				wlen = widx - ridx;

                MET_PRINTF_D("\x1b[1;36m ==> #3 base: addr + ridx << 2 = 0x%x \033[0m\n", (char *)(sspm_log_virt_addr) + (ridx << 2));
                MET_PRINTF_D("\x1b[1;36m ==> #3 len: wlen * 4 = 0x%x \033[0m\n", wlen * 4);
                MET_PRINTF_D("\x1b[1;36m ==> #3 end: 0x%x \033[0m\n", (char *)(sspm_log_virt_addr) + (ridx << 2) + (wlen * 4));

				sspm_log_req_enq((char *)(sspm_log_virt_addr) + (ridx << 2),
						wlen * 4, _log_done_cb, (void *)0);
			}
			break;

		case MET_CLOSE_FILE:	/* no argument */
			/* do close file */
			ridx = recv_buf_copy[1];
			widx = recv_buf_copy[2];
			if (widx < ridx) {	/* wrapping occurs */
				wlen = log_size - ridx;
				sspm_log_req_enq((char *)(sspm_log_virt_addr) + (ridx << 2),
						wlen * 4, _log_done_cb, (void *)1);
				sspm_log_req_enq((char *)(sspm_log_virt_addr),
						widx * 4, _log_done_cb, (void *)1);
			} else {
				wlen = widx - ridx;
				sspm_log_req_enq((char *)(sspm_log_virt_addr) + (ridx << 2),
						wlen * 4, _log_done_cb, (void *)1);
			}
			ret = sspm_log_stop();
			if (ret)
				PR_BOOTMSG("[MET] sspm_log_stop ret=%d\n", ret);

			/* continuous mode handling */
			if (sspm_run_mode == SSPM_RUN_CONTINUOUS) {
				/* clear the memory */
				memset_io((void *)sspm_log_virt_addr, 0,
					  sspm_buffer_size);
				/* re-start ondiemet again */
				sspm_start();
			}
			break;

		default:
			break;
		}

		MET_PRINTF_D("\x1b[1;34m ==> In while(1) AP done CMD(0x%x) ACK --> SSPM \033[0m\n", recv_buf_copy[0]);
		if (scmi_tinysys_common_set_symbol) {
			scmi_tinysys_common_set_symbol(tinfo->ph, feature_id, (MET_MAIN_ID | MET_AP_ACK), 0, 0, 0, 0);
		} else {
			PR_BOOTMSG("[MET] [%s,%d] scmi_tinysys_common_set is not linked!\n", __FILE__, __LINE__);
			return 1;
		}

	}while (!kthread_should_stop());

	return 0;
}

#elif defined(SSPM_VERSION_V0) || defined(SSPM_VERSION_V1)
int _sspm_recv_thread(void *data)
{
	uint32_t rdata = 0, cmd, ret;
	uint32_t ridx, widx, wlen;

	ondiemet_sspm_isr.data = (void *)recv_buf;
	ret = sspm_ipi_recv_registration(IPI_ID_TST1, &ondiemet_sspm_isr);

	do {
		sspm_ipi_recv_wait(IPI_ID_TST1);

		if (sspm_recv_thread_comp == 1) {
			while (!kthread_should_stop())
				;
			return 0;
		}
		cmd = recv_buf[0] & MET_SUB_ID_MASK;

		switch (cmd) {
		case MET_DUMP_BUFFER:	/* mbox 1: start index; 2: size */
			sspm_buffer_dumping = 1;
			ridx = recv_buf[1];
			widx = recv_buf[2];
			log_size = recv_buf[3];
			sspm_ipi_send_ack(IPI_ID_TST1, &rdata);
			if (widx < ridx) {	/* wrapping occurs */
				wlen = log_size - ridx;
				sspm_log_req_enq((char *)(sspm_log_virt_addr) + (ridx << 2),
				wlen * 4, _log_done_cb, (void *)1);
				sspm_log_req_enq((char *)(sspm_log_virt_addr),
						     widx * 4, _log_done_cb, (void *)0);
			} else {
				wlen = widx - ridx;
				sspm_log_req_enq((char *)(sspm_log_virt_addr) + (ridx << 2),
				wlen * 4, _log_done_cb, (void *)0);
			}
			break;
		case MET_CLOSE_FILE:	/* no argument */
			/* do close file */
			ridx = recv_buf[1];
			widx = recv_buf[2];
			_sspm_log_discard = recv_buf[3];
			if (widx < ridx) {	/* wrapping occurs */
				wlen = log_size - ridx;
				sspm_log_req_enq((char *)(sspm_log_virt_addr) + (ridx << 2),
				wlen * 4, _log_done_cb, (void *)1);
				sspm_log_req_enq((char *)(sspm_log_virt_addr),
						     widx * 4, _log_done_cb, (void *)1);
			} else {
				wlen = widx - ridx;
				sspm_log_req_enq((char *)(sspm_log_virt_addr) + (ridx << 2),
				wlen * 4, _log_done_cb, (void *)1);
			}
			ret = sspm_log_stop();
			/* pr_debug("MET_CLOSE_FILE: ret=%d log_discard=%d\n", ret, met_sspm_log_discard); */
			sspm_ipi_send_ack(IPI_ID_TST1, &rdata);
			if (sspm_run_mode == SSPM_RUN_CONTINUOUS) {
				/* clear the memory */
				memset_io((void *)sspm_log_virt_addr, 0,
					  sspm_buffer_size);
				/* re-start ondiemet again */
				sspm_start();
		}
			break;
		case MET_RESP_MD2AP:
			sspm_ipi_send_ack(IPI_ID_TST1, &rdata);
			sspm_buffer_dumping = 0;
			break;
		default:
			sspm_ipi_send_ack(IPI_ID_TST1, &rdata);
			break;
		}
	} while (!kthread_should_stop());
	return 0;
}

#elif defined(SSPM_VERSION_V2)
int _sspm_recv_thread(void *data)
{
	int ret = 0;
	uint32_t cmd = 0;

	do {
		ret = mtk_ipi_recv_reply(sspm_ipidev_symbol,IPIR_C_MET, (void *)&rdata, 1);

		if (ret) {
			pr_debug("[MET] ipi_register:%d failed:%d\n", IPIR_C_MET, ret);
		}

		if (sspm_recv_thread_comp == 1) {
			while (!kthread_should_stop())
				;
			return 0;
		}
		cmd = recv_buf[0] & MET_SUB_ID_MASK;

		switch (cmd) {
		case MET_DUMP_BUFFER:	/* mbox 1: start index; 2: size */
			if (widx < ridx) {	/* wrapping occurs */
				wlen = log_size - ridx;
				sspm_log_req_enq((char *)(sspm_log_virt_addr) + (ridx << 2),
				wlen * 4, _log_done_cb, (void *)1);
				sspm_log_req_enq((char *)(sspm_log_virt_addr),
							 widx * 4, _log_done_cb, (void *)0);
			} else {
				wlen = widx - ridx;
				sspm_log_req_enq((char *)(sspm_log_virt_addr) + (ridx << 2),
				wlen * 4, _log_done_cb, (void *)0);
			}
			break;
		case MET_CLOSE_FILE:	/* no argument */
			if (sspm_run_mode == SSPM_RUN_CONTINUOUS) {
				/* clear the memory */
				memset_io((void *)sspm_log_virt_addr, 0,
					  sspm_buffer_size);
				/* re-start ondiemet again */
				sspm_start();
            }
			break;
		case MET_RESP_MD2AP:
			sspm_buffer_dumping = 0;
			break;
		default:
			break;
		}
	} while (!kthread_should_stop());
	return 0;
}

#endif //SCMI or SSPM_VERSION_V0 or SSPM_VERSION_V1 or SSPM_VERSION_V2

