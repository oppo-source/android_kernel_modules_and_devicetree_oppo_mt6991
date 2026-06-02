#
## Copyright (C) 2021 MediaTek Inc.
#
## This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# # published by the Free Software Foundation.
# #
# # This program is distributed in the hope that it will be useful,
# # but WITHOUT ANY WARRANTY; without even the implied warranty of
# # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# # GNU General Public License for more details.
# #
#
# # driver code for videogo

obj-$(CONFIG_MTK_VIDEOGO) += videogo.o

videogo-$(CONFIG_MTK_VIDEOGO) += videogo_driver.o

subdir-ccflags-y += -Werror
ccflags-y += -I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/sched
ccflags-y += -I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/cpufreq_lite
ccflags-y += -I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/slbc
