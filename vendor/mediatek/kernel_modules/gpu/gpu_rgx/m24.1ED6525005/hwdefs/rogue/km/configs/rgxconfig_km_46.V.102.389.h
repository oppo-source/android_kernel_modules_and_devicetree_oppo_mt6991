/*************************************************************************/ /*!
@Title          RGX Configuration for BVNC 46.V.102.389 (kernel defines)
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/

#ifndef RGXCONFIG_KM_46_V_102_389_H
#define RGXCONFIG_KM_46_V_102_389_H

/***** Automatically generated file. Do not edit manually ********************/

/******************************************************************************
 * B.V.N.C Validation defines
 *****************************************************************************/
#define RGX_BNC_KM_B 46
#define RGX_BNC_KM_N 102
#define RGX_BNC_KM_C 389

/******************************************************************************
 * DDK Defines
 *****************************************************************************/
#define RGX_FEATURE_AXI_ACELITE
#define RGX_FEATURE_CDM_CONTROL_STREAM_FORMAT (1U)
#define RGX_FEATURE_COMPUTE
#define RGX_FEATURE_COMPUTE_OVERLAP
#define RGX_FEATURE_COREID_PER_OS
#define RGX_FEATURE_ECC_RAMS (0U)
#define RGX_FEATURE_FBCDC (50U)
#define RGX_FEATURE_FBCDC_ALGORITHM (50U)
#define RGX_FEATURE_FBCDC_ARCHITECTURE (7U)
#define RGX_FEATURE_FBC_MAX_DEFAULT_DESCRIPTORS (0U)
#define RGX_FEATURE_FBC_MAX_LARGE_DESCRIPTORS (0U)
#define RGX_FEATURE_GPU_MULTICORE_SUPPORT
#define RGX_FEATURE_GPU_VIRTUALISATION
#define RGX_FEATURE_GS_RTA_SUPPORT
#define RGX_FEATURE_IRQ_PER_OS
#define RGX_FEATURE_LAYOUT_MARS (1U)
#define RGX_FEATURE_MIPS
#define RGX_FEATURE_NUM_CLUSTERS (1U)
#define RGX_FEATURE_NUM_ISP_IPP_PIPES (2U)
#define RGX_FEATURE_NUM_MEMBUS (1U)
#define RGX_FEATURE_NUM_OSIDS (8U)
#define RGX_FEATURE_NUM_RASTER_PIPES (1U)
#define RGX_FEATURE_PBE2_IN_XE
#define RGX_FEATURE_PBVNC_COREID_REG
#define RGX_FEATURE_PERFBUS
#define RGX_FEATURE_PHYS_BUS_WIDTH (36U)
#define RGX_FEATURE_ROGUEXE
#define RGX_FEATURE_SIMPLE_INTERNAL_PARAMETER_FORMAT
#define RGX_FEATURE_SIMPLE_INTERNAL_PARAMETER_FORMAT_V2
#define RGX_FEATURE_SIMPLE_PARAMETER_FORMAT_VERSION (2U)
#define RGX_FEATURE_SLC_BANKS (1U)
#define RGX_FEATURE_SLC_CACHE_LINE_SIZE_BITS (512U)
#define RGX_FEATURE_SLC_SIZE_CONFIGURABLE /* Specifies the SLC is */
                                          /* customer-configurable. True SLC */
                                          /* size must be sourced from */
                                          /* register. */
#define RGX_FEATURE_SLC_SIZE_IN_KILOBYTES (64U)
#define RGX_FEATURE_SOC_TIMER
#define RGX_FEATURE_SYS_BUS_SECURE_RESET
#define RGX_FEATURE_TB_GPU_COUNT (1U)
#define RGX_FEATURE_TFBC_DELTA_CORRELATION
#define RGX_FEATURE_TFBC_LOSSY_37_PERCENT
#define RGX_FEATURE_TFBC_NATIVE_YUV10
#define RGX_FEATURE_TFBC_VERSION (20U)
#define RGX_FEATURE_TILE_SIZE_X (16U)
#define RGX_FEATURE_TILE_SIZE_Y (16U)
#define RGX_FEATURE_VIRTUAL_ADDRESS_SPACE_BITS (40U)
#define RGX_FEATURE_VOLCANIC_TB
#define RGX_FEATURE_XE_ARCHITECTURE (1U)
#define RGX_FEATURE_XE_MEMORY_HIERARCHY
#define RGX_FEATURE_XPU_MAX_REGBANKS_ADDR_WIDTH (19U)
#define RGX_FEATURE_XPU_MAX_SLAVES (3U)
#define RGX_FEATURE_XPU_REGISTER_BROADCAST (1U)

#endif /* RGXCONFIG_KM_46_V_102_389_H */
