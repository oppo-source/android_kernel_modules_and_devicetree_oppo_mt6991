/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 * Author: Even Yang <even.yang@mediatek.com>
 */
#ifndef AUDIO_OFFLOAD_COMMON_H
#define AUDIO_OFFLOAD_COMMON_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/wait.h>
#include <linux/clk.h>

#include <sound/compress_driver.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <audio_task_manager.h>

#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
#include "mtk-dsp-mem-control.h"
#include "mtk-base-dsp.h"
#include "mtk-dsp-common.h"
#include "mtk-dsp-platform-driver.h"
#endif

enum {
	OFFLOAD_STATE_INIT = 0x1,
	OFFLOAD_STATE_IDLE = 0x2,
	OFFLOAD_STATE_PREPARE = 0x3,
	OFFLOAD_STATE_RUNNING = 0x4,
	OFFLOAD_STATE_PAUSED = 0x5,
	OFFLOAD_STATE_DRAIN = 0x6
};

enum audio_drain_type {
	AUDIO_DRAIN_ALL,  /* returns when all data has been played */
	AUDIO_DRAIN_EARLY_NOTIFY, /* drain() for gapless track switch */
	AUDIO_DRAIN_NONE,
};

struct afe_offload_param_t {
	unsigned int         state;
	unsigned int         samplerate;
	unsigned int         drain_state;
	unsigned long long   transferred;
	unsigned long long   copied_total;    /* for tstamp*/
	unsigned long long   write_blocked_idx;
	bool                 wakelock;
	ktime_t              time_pcm;
	unsigned long        time_pcm_delay_ms;
};

struct afe_offload_service_t {
	bool write_blocked;
	bool enable;
	bool drain;
	bool draindone;
	bool tswait;
	bool pause_in_drain;
	struct mutex ts_lock;
	wait_queue_head_t ts_wq;
	bool needdata;
	bool decode_error;
	unsigned int pcmdump;
	unsigned int offload_volume[2];
	uint8_t scene;
	bool vp_sync_support;
	bool vp_sync_event;
	struct task_struct *offload_thread_task;
	wait_queue_head_t offload_wq;
};

struct afe_offload_codec_t {
	unsigned int codec_samplerate;
	unsigned int codec_bitrate;
	unsigned int target_samplerate;
	bool has_video;
};

enum ipi_send_ogffload {
	OFFLOAD_RESUME = 0x300,
	OFFLOAD_PAUSE,
	OFFLOAD_SETWRITEBLOCK,
	OFFLOAD_HWBUFFER_INFO,
	OFFLOAD_EARLY_DRAIN,
	OFFLOAD_DRAIN,
	OFFLOAD_VOLUME,
	OFFLOAD_WRITEIDX,
	OFFLOAD_TSTAMP,
	OFFLOAD_SCENE,
	OFFLOAD_CODEC_INFO,
	OFFLOAD_VIDEO_INFO,
	OFFLOAD_MDATA_INFO,
	OFFLOAD_SEND_MAX,
	OFFLOAD_STREAMING_INFO,
};

enum ipi_received_offload {
	OFFLOAD_NEEDDATA = OFFLOAD_SEND_MAX + 1,
	OFFLOAD_PCMCONSUMED,
	OFFLOAD_DRAINDONE,
	OFFLOAD_DECODE_ERROR,
	OFFLOAD_RECEIVE_MAX
};


#endif
