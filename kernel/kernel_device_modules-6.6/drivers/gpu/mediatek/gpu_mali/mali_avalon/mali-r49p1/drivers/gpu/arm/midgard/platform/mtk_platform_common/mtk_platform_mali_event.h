// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2024 MediaTek Inc.
 */

#ifndef __MTK_PLATFORM_MALI_EVENT_H__
#define __MTK_PLATFORM_MALI_EVENT_H__

enum FENCE_TYPE {
    FENCE_TYPE_INTERNAL, // mali driver internal fence ex: glFinish
    FENCE_TYPE_KCPU_QUEUE // external fence from kcpu queue ex: graphic buffer fence
};

enum DEVICE_LOST_TYPE {
    DEVICE_LOST_GPU_QUEUE_GROUP_TIMEOUT,
    DEVICE_LOST_TILER_OOM,
    DEVICE_LOST_QUEUE_FATAL_ERROR,
    DEVICE_LOST_CSG_REG_MAP_ERROR,
    DEVICE_LOST_PROGRAM_SUSPEND_TIMEOUT,
    DEVICE_LOST_CSF_CTX_HANDLE_FAULT,
    DEVICE_LOST_PMODE_FAULT,
    DEVICE_LOST_FORCE_FENCE_SIGNAL,
};

enum GPU_RESET_TYPE {
    GPU_RESET_INTERNAL_FENCE_TIMEOUT,
    GPU_RESET_KCPU_FENCE_TIMEOUT,
    GPU_RESET_AS_ACTIVE_BIT_STUCK,
    GPU_RESET_PM_TIMEOUT,
    GPU_RESET_FW_INTERNAL_ERROR,
    GPU_RESET_PMODE_ENTER_TIMEOUT,
    GPU_RESET_PMODE_EXIT_TIMEOUT,
    GPU_RESET_QUEUE_START_TIMEOUT,
    GPU_RESET_QUEUE_STOP_TIMEOUT,
    GPU_RESET_MISS_PMODE_EXIT_IRQ,
    GPU_RESET_TERM_TIMEOUT,
    GPU_RESET_SUSPEND_TIMEOUT,
    GPU_RESET_CSG_START_TIMEOUT,
    GPU_RESET_EP_CFG_UPDATE_TIMEOUT,
    GPU_RESET_SUSPEND_POW_DOWN_TIMEOUT,
    GPU_RESET_FW_PING_TIMEOUT,
    GPU_RESET_CS_FATAL_UNRECOVERABLE,
    GPU_RESET_BUS_FAULT,
    GPU_RESET_PMODE_FAULT,
    GPU_RESET_CSF_MMU_PAGE_FAULT,
    GPU_RESET_GPUEB_IRQ_NOTIFY,
    GPU_RESET_CSF_WAIT_READY_TIMEOUT,
    GPU_RESET_MMU_WAIT_READY_TIMEOUT,
    GPU_RESET_BUSY_WAIT_CACHE_OP_TIMEOUT,
    GPU_RESET_GPU_WAIT_CACHE_CLEAN_TIMEOUT,
    GPU_RESET_HW_ISSUE_2019_3901_WA,
};

#endif /* __MTK_PLATFORM_MALI_EVENT_H__ */