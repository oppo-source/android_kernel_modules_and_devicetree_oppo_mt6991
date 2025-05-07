// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2010-2024 OPPO Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */

/*
 * Oplus powermodel API implementations
 */
#include <mali_kbase.h>
#include <mali_kbase_pm.h>
#include <backend/gpu/mali_kbase_pm_internal.h>
#include <oplus_kbase_pm_state.h>
#include <mtk_gpu_utility.h>

// spinlock_t shadercore_state_lock;
// unsigned long flags;
extern u64 (*oplus_get_mali_shadercore_state_fp)(void);
extern void (*oplus_mali_gpu_state_notify_enable_fp)(bool enable);
struct kbase_device *kbdev_pm = NULL;
bool notify_gpu_state_enabled = false;
u64 shadercore_cur_state = 0;
u64 shadercore_last_active_state = 0;
u64 tiler_cur_state = 0;
u64 l2_cur_state = 0;
u64 oplus_get_shadercore_state(void);
void oplus_enable_notify_state(bool enable);


void oplus_gpu_pm_state_fp_init(void)
{
	if( !oplus_get_mali_shadercore_state_fp)
		oplus_get_mali_shadercore_state_fp = oplus_get_shadercore_state;

	if( !oplus_mali_gpu_state_notify_enable_fp)
		oplus_mali_gpu_state_notify_enable_fp = oplus_enable_notify_state;
}

void oplus_gpu_pm_state_fp_term(void)
{
	oplus_get_mali_shadercore_state_fp = NULL;
	oplus_mali_gpu_state_notify_enable_fp = NULL;
}

void oplus_enable_notify_state(bool enable)
{
	notify_gpu_state_enabled = enable;
}


void oplus_gpu_pm_update_state(struct kbase_device *kbdev)
{
	u64 shaders_ready = 0;

	if(!notify_gpu_state_enabled)
		return;

	shaders_ready = kbase_pm_get_ready_cores(kbdev, KBASE_PM_CORE_SHADER);
	oplus_gpu_pm_update_shader_state(kbdev, shaders_ready);

	if(kbdev_pm != kbdev) {
		kbdev_pm = kbdev;
	}
}

void oplus_gpu_pm_update_shader_state(struct kbase_device *kbdev, u64 shaders_ready)
{
	u64 tiler = 0;
	u64 l2 = 0;
	u64 artificial_shaders = 0;

	if(!notify_gpu_state_enabled)
		return;

	tiler = kbase_pm_get_ready_cores(kbdev, KBASE_PM_CORE_TILER);
	l2 = kbase_pm_get_ready_cores(kbdev, KBASE_PM_CORE_L2);

	if(shadercore_cur_state != shaders_ready)
	{
		shadercore_cur_state = shaders_ready;
		if (shadercore_cur_state != 0)
			shadercore_last_active_state = shadercore_cur_state;

		l2_cur_state = l2;
		tiler_cur_state = tiler;
		oplus_mali_notify_shadercore_state_change(shaders_ready);

	} else if (shaders_ready == 0 && l2 != l2_cur_state) {
		artificial_shaders = l2?shadercore_last_active_state:0;
		l2_cur_state = l2;
		tiler_cur_state = tiler;

		oplus_mali_notify_shadercore_state_change(artificial_shaders);
	}
}

u64 oplus_get_shadercore_state(void)
{
	// u64 shaders_ready = kbase_pm_get_ready_cores(kbdev_pm, KBASE_PM_CORE_SHADER);
	// if(shadercore_cur_state != shaders_ready) {
	// 	shadercore_cur_state = shaders_ready;
	// }
	return shadercore_cur_state;
}
