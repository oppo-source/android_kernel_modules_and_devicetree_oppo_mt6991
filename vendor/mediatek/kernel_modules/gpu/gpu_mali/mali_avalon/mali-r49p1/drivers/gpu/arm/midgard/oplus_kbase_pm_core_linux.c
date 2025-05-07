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

#include <mali_kbase.h>
#include <oplus_kbase_pm_core_linux.h>
#include <oplus_powermodel_gpu_utility.h>
#include <uapi/gpu/arm/midgard/mali_kbase_ioctl.h>
#include <mali_kbase_kinstr_prfcnt.h>
#if MALI_USE_CSF
#include "csf/mali_kbase_csf_firmware.h"
#endif


extern int (*oplus_kbase_open_fp)(void * inode,void* flip);
extern int (*oplus_kbase_release_fp)(void * inode,void* flip);
extern int (*oplus_kbase_api_handshake_fp)(void *kfile,void *version);
extern int (*oplus_kbase_api_set_flags_fp)(void *kfile,void *flags);
extern int (*oplus_kbase_api_get_gpuprops_fp)(void *kfile,void *get_props);
#if MALI_USE_CSF
extern int (*oplus_kbase_api_ioctl_cs_get_glb_iface_fp)(void *kfile,void *param);
#endif
extern int (*oplus_kbase_api_kinstr_prfcnt_enum_info_fp)(void *kfile,void *prfcnt_enum_info);
extern int (*oplus_kbase_api_kinstr_prfcnt_setup_fp)(void *kfile,void *prfcnt_setup);

const struct oplus_pm_core_linux_operations *pm_core_linux_opeations;

void oplus_kbase_set_operations(const struct oplus_pm_core_linux_operations *operations)
{
	pm_core_linux_opeations = operations;
	return;
}

static int kbase_api_get_gpuprops_kernel(struct kbase_file *kfile,
		struct kbase_ioctl_get_gpuprops *get_props)
{
	struct kbase_gpu_props *kprops = &kfile->kbdev->gpu_props;
	int err;
	if (get_props->flags != 0) {
		dev_err(kfile->kbdev->dev, "Unsupported flags to get_gpuprops");
		return -EINVAL;
	}

	if (get_props->size == 0)
		return (int)kprops->prop_buffer_size;
	if (get_props->size < kprops->prop_buffer_size)
		return -EINVAL;

	memcpy((void *)get_props->buffer,
			kprops->prop_buffer,
			kprops->prop_buffer_size);
	return (int)kprops->prop_buffer_size;
}

static int kbase_api_kinstr_prfcnt_setup_kernel(
	struct kbase_file *kfile,
	union kbase_ioctl_kinstr_prfcnt_setup_kernel *prfcnt_setup)
{
	return kbase_kinstr_prfcnt_setup_kernel(kfile->kbdev->kinstr_prfcnt_ctx,
					 prfcnt_setup);
}

static int kbase_api_kinstr_prfcnt_enum_info_kernel(
	struct kbase_file *kfile,
	struct kbase_ioctl_kinstr_prfcnt_enum_info *prfcnt_enum_info)
{
	return kbase_kinstr_prfcnt_enum_info_kernel(kfile->kbdev->kinstr_prfcnt_ctx,
					     prfcnt_enum_info);
}


static int kbase_ioctl_cs_get_glb_iface_kernel(struct kbase_context *kctx,
		union kbase_ioctl_cs_get_glb_iface *param)
{
	struct basep_cs_stream_control *stream_data = NULL;
	struct basep_cs_group_control *group_data = NULL;
	int err = 0;
	u32 const max_group_num = param->in.max_group_num;
	u32 const max_total_stream_num = param->in.max_total_stream_num;

	if (max_group_num > MAX_SUPPORTED_CSGS)
		return -EINVAL;

	if (max_total_stream_num >
		MAX_SUPPORTED_CSGS * MAX_SUPPORTED_STREAMS_PER_GROUP)
		return -EINVAL;

	if (max_group_num > 0) {
		group_data = kcalloc(max_group_num,sizeof(*group_data), GFP_KERNEL);
		if (!group_data)
			err = -ENOMEM;
	}

	if (max_total_stream_num > 0) {
		stream_data = kcalloc(max_total_stream_num,
				sizeof(*stream_data), GFP_KERNEL);
		if (!stream_data)
			err = -ENOMEM;
	}

	if (!err) {
		param->out.total_stream_num = kbase_csf_firmware_get_glb_iface(
			kctx->kbdev, group_data, max_group_num, stream_data,
			max_total_stream_num, &param->out.glb_version,
			&param->out.features, &param->out.group_num,
			&param->out.prfcnt_size, &param->out.instr_features);

		memcpy((void *)(param->in.groups_ptr),group_data,
			MIN(max_group_num, param->out.group_num) *
				sizeof(*group_data));
	}

	if (!err)
	{
		memcpy((void *)(param->in.streams_ptr), stream_data,
			MIN(max_total_stream_num, param->out.total_stream_num) *
				sizeof(*stream_data));
	}

	kfree(group_data);
	kfree(stream_data);
	return err;
}

#if MALI_USE_CSF
int oplus_api_kbase_ioctl_cs_get_glb_iface(void *kfile,void *param)
{
	int ret;
	struct kbase_file *kb_file = (struct kbase_file *)kfile;
	struct kbase_context *kbase_ctx = (struct kbase_context *)kb_file->kctx;
	union kbase_ioctl_cs_get_glb_iface *kbase_param = (union kbase_ioctl_cs_get_glb_iface *)param;

	ret = kbase_ioctl_cs_get_glb_iface_kernel(kbase_ctx,kbase_param);
	return ret;
}
#endif

int oplus_api_kbase_open(void * inode,void* flip)
{
	int ret = -1;
	struct inode *kbase_inode = (struct inode *)inode;
	struct file *kbase_filp = (struct file *)flip;
	if(pm_core_linux_opeations->open != NULL) {
		ret = pm_core_linux_opeations->open(inode,flip);
	}
	return ret;
}

int oplus_api_kbase_release(void * inode,void* flip)
{
	int ret = -1;
	struct inode *kbase_inode = (struct inode *)inode;
	struct file *kbase_filp = (struct file *)flip;
	if(pm_core_linux_opeations->release != NULL) {
		ret = pm_core_linux_opeations->release(inode,flip);
	}
	return ret;
}

int oplus_api_kbase_api_handshake(void *kfile,void *version)
{
	int ret = -1;
	struct kbase_file *const kbase_file = (struct kbase_file *)kfile;
	struct kbase_ioctl_version_check *kbase_version  = (struct kbase_ioctl_version_check *)version;
	if(pm_core_linux_opeations->handshake != NULL) {
		ret = pm_core_linux_opeations->handshake(kbase_file,kbase_version);
	}
	return ret;
}

int oplus_api_kbase_api_set_flags(void *kfile,void *flags)
{
	int ret = -1;
	struct kbase_file *kbase_file = (struct kbase_file *)kfile;
	struct kbase_ioctl_set_flags * kbase_flags = (struct kbase_ioctl_set_flags *)flags;
	if(pm_core_linux_opeations->set_flags != NULL) {
		ret = pm_core_linux_opeations->set_flags(kbase_file,kbase_flags);
	}

	return ret;
}

int oplus_api_kbase_api_get_gpuprops(void *kfile,void *get_props)
{
	int ret;
	struct kbase_file *kbase_file = (struct kbase_file *)kfile;
	struct kbase_ioctl_get_gpuprops *kbase_get_props = (struct kbase_ioctl_get_gpuprops *)get_props;

	ret = kbase_api_get_gpuprops_kernel(kbase_file,kbase_get_props);
	return ret;
}


int oplus_api_kbase_api_kinstr_prfcnt_enum_info(void *kfile,void *prfcnt_enum_info)
{
	int ret;
	struct kbase_file *kbase_file = (struct kbase_file *)kfile;
	struct kbase_ioctl_kinstr_prfcnt_enum_info *kbase_prfcnt_enum_info = (struct kbase_ioctl_kinstr_prfcnt_enum_info *)prfcnt_enum_info;
	ret = kbase_api_kinstr_prfcnt_enum_info_kernel(kbase_file,kbase_prfcnt_enum_info);
	return ret;
}

int oplus_api_kbase_api_kinstr_prfcnt_setup(void *kfile,void *prfcnt_setup)
{
	int ret;
	struct kbase_file *kbase_file = (struct kbase_file *)kfile;
	union kbase_ioctl_kinstr_prfcnt_setup_kernel *kbase_prfcnt_setup = (union kbase_ioctl_kinstr_prfcnt_setup_kernel *)prfcnt_setup;
	ret = kbase_api_kinstr_prfcnt_setup_kernel(kbase_file,kbase_prfcnt_setup);
	return ret;
}

void oplus_kbase_fun_fp_init(void) {
	oplus_kbase_open_fp = oplus_api_kbase_open;
	oplus_kbase_release_fp = oplus_api_kbase_release;
	oplus_kbase_api_handshake_fp = oplus_api_kbase_api_handshake;
	oplus_kbase_api_set_flags_fp = oplus_api_kbase_api_set_flags;
	oplus_kbase_api_get_gpuprops_fp = oplus_api_kbase_api_get_gpuprops;
#if MALI_USE_CSF
	oplus_kbase_api_ioctl_cs_get_glb_iface_fp = oplus_api_kbase_ioctl_cs_get_glb_iface;
#endif
	oplus_kbase_api_kinstr_prfcnt_enum_info_fp = oplus_api_kbase_api_kinstr_prfcnt_enum_info;
	oplus_kbase_api_kinstr_prfcnt_setup_fp = oplus_api_kbase_api_kinstr_prfcnt_setup;
}

void oplus_kbase_fun_fp_term(void) {
	oplus_kbase_open_fp = NULL;
	oplus_kbase_release_fp = NULL;
	oplus_kbase_api_handshake_fp = NULL;
	oplus_kbase_api_set_flags_fp = NULL;
	oplus_kbase_api_get_gpuprops_fp = NULL;
#if MALI_USE_CSF
	oplus_kbase_api_ioctl_cs_get_glb_iface_fp = NULL;
#endif
	oplus_kbase_api_kinstr_prfcnt_enum_info_fp = NULL;
	oplus_kbase_api_kinstr_prfcnt_setup_fp = NULL;
}

void oplus_kbase_notify_gpu_inode(void * inode_ptr)
{
	oplus_mali_notify_gpu_inode(inode_ptr);
}
