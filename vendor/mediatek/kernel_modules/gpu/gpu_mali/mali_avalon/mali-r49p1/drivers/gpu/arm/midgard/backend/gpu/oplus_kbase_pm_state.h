/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2010-2023 OPPO Limited. All rights reserved.
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
 * Oplus powermodel API definitions
 */

#ifndef _OPLUS_KBASE_PM_STATE_H_
#define _OPLUS_KBASE_PM_STATE_H_


void oplus_gpu_pm_state_fp_init(void);

void oplus_gpu_pm_state_fp_term(void);

void oplus_gpu_pm_update_state(struct kbase_device *kbdev);
void oplus_gpu_pm_update_shader_state(struct kbase_device *kbdev, u64 shaders_ready);

#endif /* _OPLUS_KBASE_PM_STATE_H_ */
