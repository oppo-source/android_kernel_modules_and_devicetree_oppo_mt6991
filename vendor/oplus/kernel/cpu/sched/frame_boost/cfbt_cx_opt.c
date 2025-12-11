/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2025 Oplus. All rights reserved.
 */

#include "cfbt_boost.h"

static int cx_voting_running = 0;
static int cx_voting_enable = 0;
static DEFINE_MUTEX(cx_voting_lock);

int (*cfbt_update_cx_voting_state)(int enable, int period_ms) = NULL;
EXPORT_SYMBOL(cfbt_update_cx_voting_state);

void start_cx_voting(int period_ms)
{
	int result = 0;

	if (!cx_voting_enable)
		return;

	mutex_lock(&cx_voting_lock);
	if (cx_voting_running) {
		pr_err("%s, cx_voting is already running", __func__);
		mutex_unlock(&cx_voting_lock);
		return;
	}
	result = cfbt_update_cx_voting_state(1, period_ms);
	if (!result)
		cx_voting_running = 1;
	else
		pr_err("%s, cfbt_update_cx_voting_state failed, result = %d", __func__, result);
	mutex_unlock(&cx_voting_lock);
}

void stop_cx_voting(void)
{
	int result = 0;

	mutex_lock(&cx_voting_lock);
	if (!cx_voting_running) {
		pr_err("%s, cx_voting is not running", __func__);
		mutex_unlock(&cx_voting_lock);
		return;
	}
	result = cfbt_update_cx_voting_state(0, 0);
	if (!result)
		cx_voting_running = 0;
	else
		pr_err("%s, cfbt_update_cx_voting_state failed, result = %d", __func__, result);
	mutex_unlock(&cx_voting_lock);
}

void enable_cx_opt(int val)
{
	cx_voting_enable = val;
	if (!cx_voting_enable)
		stop_cx_voting();
}
EXPORT_SYMBOL(enable_cx_opt);

int is_enable_cx_opt(void)
{
	return cx_voting_enable;
}
EXPORT_SYMBOL(is_enable_cx_opt);