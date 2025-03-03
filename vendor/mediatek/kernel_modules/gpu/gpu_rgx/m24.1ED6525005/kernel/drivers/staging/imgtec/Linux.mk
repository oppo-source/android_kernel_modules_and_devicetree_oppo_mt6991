########################################################################### ###
#@Copyright     Copyright (c) Imagination Technologies Ltd. All Rights Reserved
#@License       Dual MIT/GPLv2
#
# The contents of this file are subject to the MIT license as set out below.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# Alternatively, the contents of this file may be used under the terms of
# the GNU General Public License Version 2 ("GPL") in which case the provisions
# of GPL are applicable instead of those above.
#
# If you wish to allow use of your version of this file only under the terms of
# GPL, and not to allow others to use your version of this file under the terms
# of the MIT license, indicate your decision by deleting the provisions above
# and replace them with the notice and other provisions required by GPL as set
# out in the file called "GPL-COPYING" included in this distribution. If you do
# not delete the provisions above, a recipient may use your version of this file
# under the terms of either the MIT license or GPL.
#
# This License is also included in this distribution in the file called
# "MIT-COPYING".
#
# EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
# PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
# BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
# PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
### ###########################################################################

modules := tc drm_nulldisp drm_pdp plato drm_rk drm_pdp2_hdmi cdma loki tee_ddk

tc_type := kernel_module
tc_target := tc.ko
tc_makefile := $(THIS_DIR)/Kbuild.mk

drm_nulldisp_type := kernel_module
drm_nulldisp_target := drm_nulldisp.ko
drm_nulldisp_makefile := $(THIS_DIR)/Kbuild.mk

drm_pdp_type := kernel_module
drm_pdp_target := drm_pdp.ko
drm_pdp_makefile := $(THIS_DIR)/Kbuild.mk

drm_pdp2_hdmi_type := kernel_module
drm_pdp2_hdmi_target := drm_pdp2_hdmi.ko
drm_pdp2_hdmi_makefile := $(THIS_DIR)/Kbuild.mk

plato_type := kernel_module
plato_target := plato.ko
plato_makefile := $(THIS_DIR)/Kbuild.mk

drm_rk_type := kernel_module
drm_rk_target := drm_rk.ko
drm_rk_makefile := $(THIS_DIR)/Kbuild.mk

cdma_type := kernel_module
cdma_target := cdma.ko
cdma_makefile := $(THIS_DIR)/Kbuild.mk

loki_type := kernel_module
loki_target := loki.ko
loki_makefile := $(THIS_DIR)/Kbuild.mk

tee_ddk_type := kernel_module
tee_ddk_target := tee_ddk.ko
tee_ddk_makefile := $(THIS_DIR)/Kbuild.mk
