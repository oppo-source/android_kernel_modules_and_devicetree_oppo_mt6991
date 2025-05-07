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

#include <linux/fs.h>
#include <mali_kbase_defs.h>

struct oplus_pm_core_linux_operations {
 	int (*open)(struct inode *inode,struct file *flip);
 	int (*release)(struct inode *inode,struct file *flip);
 	int (*handshake)(struct kbase_file *const file,struct kbase_ioctl_version_check *version);
 	int (*set_flags)(struct kbase_file *const file,struct kbase_ioctl_set_flags * flags);
};

void oplus_kbase_set_operations(const struct oplus_pm_core_linux_operations *operations);

void oplus_kbase_fun_fp_init(void);

void oplus_kbase_fun_fp_term(void);

void oplus_kbase_notify_gpu_inode(void * inode_ptr);
