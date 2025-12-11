// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung UFS Feature Mimic for android15-6.6
 * commit 3009510505edcc8a0622b6c99925fb841b5e132e
 * Date:   Fri Aug 16 14:33:14 2024 +0000
 *
 * from
 *
 * linux/drivers/ufs/core/ufshcd.c
 */

#include <ufs/ufshcd.h>
#include "../core/ufshcd-priv.h"
#include "ufsfeature.h"
#include <trace/hooks/ufshcd.h>
#include <linux/delay.h>

void ufsf_scsi_unblock_requests(struct ufs_hba *hba)
{
	if (atomic_dec_and_test(&hba->scsi_block_reqs_cnt))
		scsi_unblock_requests(hba->host);
}

void ufsf_scsi_block_requests(struct ufs_hba *hba)
{
	if (atomic_inc_return(&hba->scsi_block_reqs_cnt) == 1)
		scsi_block_requests(hba->host);
}

static u32 ufsf_pending_cmds(struct ufs_hba *hba)
{
	const struct scsi_device *sdev;
	u32 pending = 0;

	lockdep_assert_held(hba->host->host_lock);
	__shost_for_each_device(sdev, hba->host)
		pending += sbitmap_weight(&sdev->budget_map);

	return pending;
}

int ufsf_wait_for_doorbell_clr(struct ufs_hba *hba, u64 wait_timeout_us)
{
	unsigned long flags;
	int ret = 0;
	u32 tm_doorbell;
	u32 tr_pending;
	bool timeout = false, do_last_check = false;
	ktime_t start;

	ufshcd_hold(hba);
	spin_lock_irqsave(hba->host->host_lock, flags);
	/*
	 * Wait for all the outstanding tasks/transfer requests.
	 * Verify by checking the doorbell registers are clear.
	 */
	start = ktime_get();
	do {
		if (hba->ufshcd_state != UFSHCD_STATE_OPERATIONAL) {
			ret = -EBUSY;
			goto out;
		}

		tm_doorbell = ufshcd_readl(hba, REG_UTP_TASK_REQ_DOOR_BELL);
		tr_pending = ufsf_pending_cmds(hba);
		if (!tm_doorbell && !tr_pending) {
			timeout = false;
			break;
		} else if (do_last_check) {
			break;
		}

		spin_unlock_irqrestore(hba->host->host_lock, flags);
		io_schedule_timeout(msecs_to_jiffies(20));
		if (ktime_to_us(ktime_sub(ktime_get(), start)) >
		    wait_timeout_us) {
			timeout = true;
			/*
			 * We might have scheduled out for long time so make
			 * sure to check if doorbells are cleared by this time
			 * or not.
			 */
			do_last_check = true;
		}
		spin_lock_irqsave(hba->host->host_lock, flags);
	} while (tm_doorbell || tr_pending);

	if (timeout) {
		dev_err(hba->dev,
			"%s: timedout waiting for doorbell to clear (tm=0x%x, tr=0x%x)\n",
			__func__, tm_doorbell, tr_pending);
		ret = -EBUSY;
	}
out:
	spin_unlock_irqrestore(hba->host->host_lock, flags);
	ufshcd_release(hba);
	return ret;
}

int ufsf_get_bkops_status(struct ufs_hba *hba, u32 *status)
{
	return ufshcd_query_attr_retry(hba, UPIU_QUERY_OPCODE_READ_ATTR,
			QUERY_ATTR_IDN_BKOPS_STATUS, 0, 0, status);
}

static inline int ufsf_get_ee_status(struct ufs_hba *hba, u32 *status)
{
	return ufshcd_query_attr_retry(hba, UPIU_QUERY_OPCODE_READ_ATTR,
			QUERY_ATTR_IDN_EE_STATUS, 0, 0, status);
}

int __ufsf_write_ee_control(struct ufs_hba *hba, u32 ee_ctrl_mask)
{
	return ufshcd_query_attr_retry(hba, UPIU_QUERY_OPCODE_WRITE_ATTR,
				       QUERY_ATTR_IDN_EE_CONTROL, 0, 0,
				       &ee_ctrl_mask);
}

static int ufsf_update_ee_control(struct ufs_hba *hba, u16 *mask,
				  const u16 *other_mask, u16 set, u16 clr)
{
	u16 new_mask, ee_ctrl_mask;
	int err = 0;

	mutex_lock(&hba->ee_ctrl_mutex);
	new_mask = (*mask & ~clr) | set;
	ee_ctrl_mask = new_mask | *other_mask;
	if (ee_ctrl_mask != hba->ee_ctrl_mask)
		err = __ufsf_write_ee_control(hba, ee_ctrl_mask);
	/* Still need to update 'mask' even if 'ee_ctrl_mask' was unchanged */
	if (!err) {
		hba->ee_ctrl_mask = ee_ctrl_mask;
		*mask = new_mask;
	}
	mutex_unlock(&hba->ee_ctrl_mutex);
	return err;
}

static inline int ufsf_update_ee_drv_mask(struct ufs_hba *hba,
					  u16 set, u16 clr)
{
	return ufsf_update_ee_control(hba, &hba->ee_drv_mask,
				      &hba->ee_usr_mask, set, clr);
}

static inline int ufsf_disable_ee(struct ufs_hba *hba, u16 mask)
{
	return ufsf_update_ee_drv_mask(hba, 0, mask);
}

static int ufsf_enable_auto_bkops(struct ufs_hba *hba)
{
	int err = 0;

	if (hba->auto_bkops_enabled)
		goto out;

	err = ufshcd_query_flag_retry(hba, UPIU_QUERY_OPCODE_SET_FLAG,
			QUERY_FLAG_IDN_BKOPS_EN, 0, NULL);
	if (err) {
		dev_err(hba->dev, "%s: failed to enable bkops %d\n",
				__func__, err);
		goto out;
	}

	hba->auto_bkops_enabled = true;
	dev_info(hba->dev, "%s: auto bkops - Enabled\n", __func__);

	/* No need of URGENT_BKOPS exception from the device */
	err = ufsf_disable_ee(hba, MASK_EE_URGENT_BKOPS);
	if (err)
		dev_err(hba->dev, "%s: failed to disable exception event %d\n",
				__func__, err);
out:
	return err;
}

static void ufsf_bkops_exception_event_handler(struct ufs_hba *hba)
{
	int err;
	u32 curr_status = 0;

	if (hba->is_urgent_bkops_lvl_checked)
		goto enable_auto_bkops;

	err = ufsf_get_bkops_status(hba, &curr_status);
	if (err) {
		dev_err(hba->dev, "%s: failed to get BKOPS status %d\n",
				__func__, err);
		goto out;
	}

	/*
	 * We are seeing that some devices are raising the urgent bkops
	 * exception events even when BKOPS status doesn't indicate performace
	 * impacted or critical. Handle these device by determining their urgent
	 * bkops status at runtime.
	 */
	if (curr_status < BKOPS_STATUS_PERF_IMPACT) {
		dev_err(hba->dev, "%s: device raised urgent BKOPS exception for bkops status %d\n",
				__func__, curr_status);
		/* update the current status as the urgent bkops level */
		hba->urgent_bkops_lvl = curr_status;
		hba->is_urgent_bkops_lvl_checked = true;
	}

enable_auto_bkops:
	err = ufsf_enable_auto_bkops(hba);
out:
	if (err < 0)
		dev_err(hba->dev, "%s: failed to handle urgent bkops %d\n",
				__func__, err);
}

static void ufsf_temp_exception_event_handler(struct ufs_hba *hba, u16 status)
{
	u32 value;

	if (ufshcd_query_attr_retry(hba, UPIU_QUERY_OPCODE_READ_ATTR,
				QUERY_ATTR_IDN_CASE_ROUGH_TEMP, 0, 0, &value))
		return;

	dev_info(hba->dev, "exception Tcase %d\n", value - 80);

	ufs_hwmon_notify_event(hba, status & MASK_EE_URGENT_TEMP);

	/*
	 * A placeholder for the platform vendors to add whatever additional
	 * steps required
	 */
}

/**
 * ufsf_exception_event_handler - handle exceptions raised by device
 * @work: pointer to work data
 *
 * Read bExceptionEventStatus attribute from the device and handle the
 * exception event accordingly.
 */
void ufsf_exception_event_handler(struct ufs_hba *hba)
{
	int err;
	u32 status = 0;

	ufsf_scsi_block_requests(hba);
	err = ufsf_get_ee_status(hba, &status);
	if (err) {
		dev_err(hba->dev, "%s: failed to get exception status %d\n",
				__func__, err);
		goto out;
	}

	dev_info(hba->dev, "%s: status 0x%x\n", __func__, status);

	if (status & hba->ee_drv_mask & MASK_EE_URGENT_BKOPS)
		ufsf_bkops_exception_event_handler(hba);

	if (status & hba->ee_drv_mask & MASK_EE_URGENT_TEMP)
		ufsf_temp_exception_event_handler(hba, status);
out:
	ufsf_scsi_unblock_requests(hba);
}

MODULE_LICENSE("GPL v2");
