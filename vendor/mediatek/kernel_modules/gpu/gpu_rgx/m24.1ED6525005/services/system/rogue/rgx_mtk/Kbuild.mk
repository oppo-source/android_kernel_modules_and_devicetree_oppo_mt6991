#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

PVRSRVKM_NAME = $(PVRSRV_MODNAME)

ccflags-y += \
	-Wno-error \

$(PVRSRVKM_NAME)-y += \
	services/system/common/env/linux/interrupt_support.o \
	services/system/rogue/common/env/linux/dma_support.o \
	services/system/rogue/common/vmm_type_stub.o \
	services/server/common/vmm_pvz_client.o \
	services/server/common/vmm_pvz_server.o \
	services/server/common/vz_vmm_pvz.o \
	services/server/common/vz_vmm_vm.o \
	services/system/rogue/$(PVR_SYSTEM)/ion_support.o \
	services/system/rogue/$(PVR_SYSTEM)/mtk_pp.o \
	services/system/rogue/$(PVR_SYSTEM)/sysconfig.o \
	services/system/rogue/$(PVR_SYSTEM)/$(MTK_PLATFORM)/mtk_mfgsys.o

ifeq ($(MTK_PLATFORM),mt6739)
$(PVRSRVKM_NAME)-y += \
	services/system/rogue/$(PVR_SYSTEM)/$(MTK_PLATFORM)/mtk_mfg_counter.o
endif

ccflags-y += \
	-I$(TOP)/services/system/rogue/$(PVR_SYSTEM) \
	-I$(TOP)/services/system/rogue/$(PVR_SYSTEM)/$(MTK_PLATFORM) \
	-I$(TOP)/services/system/rogue/common/env/linux \
	-I$(TOP)/services/linux/include \
	-I$(TOP)/kernel/drivers/staging/imgtec \
	-I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/include/mt-plat \
	-I$(DEVICE_MODULES_PATH)/drivers/staging/android/ion \
	-I$(DEVICE_MODULES_PATH)/drivers/staging/android/ion/mtk

ifeq ($(kernel_ver),kernel-4.14)
ccflags-y += \
	-I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/gpu/ged/include \
	-I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/include/mt-plat \
	-I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/gpu/gpu_bm \
	-I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/base/power/$(MTK_PLATFORM)
else
ccflags-y += \
	-I$(DEVICE_MODULES_PATH)/drivers/gpu/mediatek \
	-I$(DEVICE_MODULES_PATH)/drivers/gpu/mediatek/ged/include \
	-I$(DEVICE_MODULES_PATH)/drivers/gpu/mediatek/mt-plat \
	-I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/include/mt-plat/ \
	-I$(DEVICE_MODULES_PATH)/drivers/gpu/mediatek/gpu_bm \
	-I$(DEVICE_MODULES_PATH)/drivers/gpu/mediatek/gpufreq/ \
	-I$(DEVICE_MODULES_PATH)/drivers/gpu/mediatek/gpufreq/v2_legacy/include \
	-I$(DEVICE_MODULES_PATH)/drivers/gpu/mediatek/gpufreq/$(MTK_PLATFORM)
endif
