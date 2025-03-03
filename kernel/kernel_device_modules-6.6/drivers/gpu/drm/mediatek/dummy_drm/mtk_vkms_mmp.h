/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef _MTK_VKMS_MMP_H_
#define _MTK_VKMS_MMP_H_

#include "mmprofile.h"
#include "mmprofile_function.h"

#define MMP_CRTC_NUM 3

/* if changed, need to update init_drm_mmp_event() */
struct DRM_MMP_Events {
	mmp_event drm;
	mmp_event crtc[MMP_CRTC_NUM];

	/* define for IRQ */
	mmp_event IRQ;
	mmp_event ovl;
	mmp_event ovl0;
	mmp_event ovl1;
	mmp_event ovl0_2l;
	mmp_event ovl1_2l;
	mmp_event ovl2_2l;
	mmp_event ovl3_2l;
	mmp_event rdma;
	mmp_event rdma0;
	mmp_event rdma1;
	mmp_event rdma2;
	mmp_event rdma3;
	mmp_event rdma4;
	mmp_event rdma5;
	mmp_event wdma;
	mmp_event wdma0;
	mmp_event dsi;
	mmp_event dsi0;
	mmp_event dsi1;
	mmp_event aal;
	mmp_event aal0;
	mmp_event aal1;
	mmp_event dp_intf0;
	mmp_event ddp;
	mmp_event mutex[4];
	mmp_event postmask;
	mmp_event postmask0;
	mmp_event abnormal_irq;
	mmp_event iova_tf;
	mmp_event pmqos;
	mmp_event hrt_bw;
	mmp_event mutex_lock;
	mmp_event layering;
	mmp_event layering_blob;
	mmp_event dma_alloc;
	mmp_event dma_free;
	mmp_event dma_get;
	mmp_event dma_put;
	mmp_event ion_import_dma;
	mmp_event ion_import_fd;
	mmp_event ion_import_free;
	mmp_event set_mode;
	mmp_event top_clk;
	mmp_event sram_alloc;
	mmp_event sram_free;
};

/* if changed, need to update init_crtc_mmp_event() */
struct CRTC_MMP_Events {
	mmp_event trig_loop_done;
	mmp_event enable;
	mmp_event disable;
	mmp_event release_fence;
	mmp_event update_present_fence;
	mmp_event release_present_fence;
	mmp_event present_fence_timestamp_same;
	mmp_event present_fence_timestamp;
	mmp_event update_sf_present_fence;
	mmp_event release_sf_present_fence;
	mmp_event warn_sf_pf_0;
	mmp_event warn_sf_pf_2;
	mmp_event atomic_begin;
	mmp_event atomic_flush;
	mmp_event enable_vblank;
	mmp_event disable_vblank;
	mmp_event esd_check;
	mmp_event esd_recovery;
	mmp_event leave_idle;
	mmp_event enter_idle;
	mmp_event frame_cfg;
	mmp_event suspend;
	mmp_event resume;
	mmp_event dsi_suspend;
	mmp_event dsi_resume;
	mmp_event backlight;
	mmp_event backlight_grp;
	mmp_event ddic_send_cmd;
	mmp_event ddic_read_cmd;
	mmp_event path_switch;
	mmp_event user_cmd;
	mmp_event check_trigger;
	mmp_event kick_trigger;
	mmp_event atomic_commit;
	mmp_event user_cmd_cb;
	mmp_event bl_cb;
	mmp_event clk_change;
	mmp_event layerBmpDump;
	mmp_event layer_dump[6];
	mmp_event wbBmpDump;
	mmp_event wb_dump;
	mmp_event cwbBmpDump;
	mmp_event cwb_dump;
	/*Msync 2.0 mmp start*/
	mmp_event ovl_status_err;
	mmp_event vfp_period;
	mmp_event not_vfp_period;
	mmp_event dsi_state_dbg7;
	mmp_event dsi_dbg7_after_sof;
	mmp_event msync_enable;
	/*Msync 2.0 mmp end*/
	mmp_event mode_switch;
	mmp_event ddp_clk;
	/*AAL mmp mark*/
	mmp_event aal_sof_thread;
	mmp_event aal_dre30_rw;
	mmp_event aal_dre20_rh;
	/*Gamma mmp mark*/
	mmp_event gamma_ioctl;
	mmp_event gamma_sof;
};

#define DRM_MMP_MARK(event, v1, v2) do { } while (0)
#define DRM_MMP_EVENT_START(event, v1, v2) do { } while (0)
#define DRM_MMP_EVENT_END(event, v1, v2) do { } while (0)
#define CRTC_MMP_MARK(id, event, v1, v2) do { } while (0)
#define CRTC_MMP_EVENT_START(id, event, v1, v2) do { } while (0)
#define CRTC_MMP_EVENT_END(id, event, v1, v2) do { } while (0)

void drm_mmp_init(void);
int mmp_fb_to_bitmap(struct drm_framebuffer *fb);

#endif /* _MTK_VKMS_MMP_H_ */
