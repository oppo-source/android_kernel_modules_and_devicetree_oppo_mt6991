/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2015-2019, MICROTRUST Incorporated
 * All Rights Reserved.
 *
 */

#ifndef SOTER_PRIVATE_H
#define SOTER_PRIVATE_H

#include <linux/semaphore.h>
#include <tee_drv.h>
#include <linux/types.h>
#ifdef TEEI_FFA_SUPPORT
#include <linux/arm_ffa.h>
#endif
#define TEEI_UUID_MAX_LEN 128

/* Some Global Platform error codes used in this driver */
#define TEEC_SUCCESS			0x00000000
#define TEEC_ERROR_BAD_PARAMETERS	0xFFFF0006
#define TEEC_ERROR_COMMUNICATION	0xFFFF000E
#define TEEC_ERROR_OUT_OF_MEMORY	0xFFFF000C

#define TEEC_ORIGIN_COMMS		0x00000002

struct call_queue {
	/* Serializes access to this struct */
	struct mutex mutex;
	struct list_head waiters;
};

#ifdef TEEI_FFA_SUPPORT
/**
 * struct soter_ffa_data -  FFA communication struct
 * @ffa_dev             FFA device, contains the destination id, the id of
 *                      OP-TEE in secure world
 * @ffa_ops             FFA operations
 * @mutex               Serializes access to @global_ids
 * @global_ids          FF-A shared memory global handle translation
 */
struct soter_ffa {
	struct ffa_device *ffa_dev;
	const struct ffa_ops *ffa_ops;
};

struct soter_ffa_ops {
	int (*do_call_with_arg)(struct tee_context *ctx,
					phys_addr_t parg);
};
#endif

struct soter_priv {
	struct tee_device *teedev;
	struct call_queue call_queue;
	struct tee_shm_pool *pool;
	void *memremaped_shm;
#ifdef TEEI_FFA_SUPPORT
	struct soter_ffa ffa;
	struct soter_ffa_ops ops;
#endif
};

struct soter_session {
	struct list_head list_node;
	u32 session_id;
	char uuid[TEEI_UUID_MAX_LEN];
};

struct soter_context_data {
	/* Serializes access to this struct */
	struct mutex mutex;
	struct list_head sess_list;
};

int soter_open_session(struct tee_context *ctx,
		       struct tee_ioctl_open_session_arg *arg,
		       struct tee_param *param);
int soter_close_session(struct tee_context *ctx, u32 session);
int soter_invoke_func(struct tee_context *ctx, struct tee_ioctl_invoke_arg *arg,
		      struct tee_param *param);
int soter_cancel_func(struct tee_context *ctx, u32 cancel_id, u32 session);

#if IS_ENABLED(CONFIG_MICROTRUST_TZDRIVER_DYNAMICAL_DEBUG)
extern uint32_t tzdriver_dynamical_debug_flag;
#define REE_DYNAMICAL_START 1
#define REE_DYNAMICAL_STOP  2
#endif

extern unsigned long teei_capi_ready;
extern struct soter_priv  *soter_priv;

extern struct reserved_mem *reserved_mem;
extern atomic_t is_shm_pool_available;
extern struct completion shm_pool_registered;

#endif
