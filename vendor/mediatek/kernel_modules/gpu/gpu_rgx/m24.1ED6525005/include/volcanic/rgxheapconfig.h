/*************************************************************************/ /*!
@File
@Title          RGX Device virtual memory map
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Memory heaps device specific configuration
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

#ifndef RGXHEAPCONFIG_H
#define RGXHEAPCONFIG_H

#include "rgxdefs_km.h"

#define RGX_HEAP_SIZE_32KiB      IMG_UINT64_C(0x0000008000)
#define RGX_HEAP_SIZE_2MiB       IMG_UINT64_C(0x0000200000)
#define RGX_HEAP_SIZE_4MiB       IMG_UINT64_C(0x0000400000)
#define RGX_HEAP_SIZE_16MiB      IMG_UINT64_C(0x0001000000)
#define RGX_HEAP_SIZE_32MiB      IMG_UINT64_C(0x0002000000)
#define RGX_HEAP_SIZE_256MiB     IMG_UINT64_C(0x0010000000)

#define RGX_HEAP_SIZE_1GiB       IMG_UINT64_C(0x0040000000)
#define RGX_HEAP_SIZE_2GiB       IMG_UINT64_C(0x0080000000)
#define RGX_HEAP_SIZE_4GiB       IMG_UINT64_C(0x0100000000)
#define RGX_HEAP_SIZE_16GiB      IMG_UINT64_C(0x0400000000)
#define RGX_HEAP_SIZE_32GiB      IMG_UINT64_C(0x0800000000)
#define RGX_HEAP_SIZE_64GiB      IMG_UINT64_C(0x1000000000)
#define RGX_HEAP_SIZE_128GiB     IMG_UINT64_C(0x2000000000)
#define RGX_HEAP_SIZE_256GiB     IMG_UINT64_C(0x4000000000)
#define RGX_HEAP_SIZE_512GiB     IMG_UINT64_C(0x8000000000)

/*
	RGX Device Virtual Address Space Definitions

	This file defines the RGX virtual address heaps that are used in
	application memory contexts. It also shows where the Firmware memory heap
	fits into this, but the firmware heap is only ever created in the
	Services KM/server component.

	RGX_PDSCODEDATA_HEAP_BASE and RGX_USCCODE_HEAP_BASE will be programmed,
	on a global basis, into RGX_CR_PDS_EXEC_BASE and RGX_CR_USC_CODE_BASE_*
	respectively. Therefore if clients use multiple configs they must still
	be consistent with their definitions for these heaps.

	Shared virtual memory (GENERAL_SVM) support requires half of the address
	space (512 GiB) be reserved for SVM allocations to mirror application CPU
	addresses.

	The GENERAL non-SVM region is 512 GiB to 768 GiB and is shared between the
	general (4KiB) heap and the general non-4K heap. The first 128 GiB is used
	for the GENERAL_HEAP (4KiB) and the last 32 GiB is used for the
	GENERAL_NON4K_HEAP. This heap has a default page-size of 16K.
	AppHint PVRSRV_APPHINT_GENERALNON4KHEAPPAGESIZE can be used to forced it
	to these values: 4K,64K,256K,1M,2M.

	Heaps must not start at 0x0000000000, as this is reserved for internal
	use within device memory layer.
	Range comments, those starting in column 0 below are a section heading of
	sorts and are above the heaps in that range. Often this is the reserved
	size of the heap within the range.
*/


/* 0x00_0000_0000 ************************************************************/

/* 0x00_0000_0000 - 0x00_0020_0000 **/
	/* 0 MiB to 2 MiB, size of 2 MiB : RESERVED (only when General SVM
	 *                                           doesn't exist) **/

/* 0x00_0000_8000 - 0x7F_FFFF_8000 **/
	/* 32 KiB to 512 GiB, size of 512 GiB less 32 KiB : GENERAL_SVM_HEAP **/
	#define RGX_GENERAL_SVM_HEAP_BASE           IMG_UINT64_C(0x0000008000)
	#define RGX_GENERAL_SVM_HEAP_SIZE           (RGX_HEAP_SIZE_512GiB - RGX_HEAP_SIZE_32KiB)


/* 0x80_0000_0000 ************************************************************/

/* 0x80_0000_0000 - 0x9F_FFFF_FFFF **/
	/* 512 GiB to 640 GiB, size of 128 GiB : GENERAL_HEAP **/
	#define RGX_GENERAL_HEAP_BASE               IMG_UINT64_C(0x8000000000)
	#define RGX_GENERAL_HEAP_SIZE               RGX_HEAP_SIZE_128GiB

/* 0xA0_0000_0000 - 0xAF_FFFF_FFFF **/
	/* 640 GiB to 704 GiB, size of 64 GiB : FREE **/

/* 0xB0_0000_0000 - 0xB1_FFFF_FFFF **/
	/* 704 GiB to 720 GiB, size of 16 GiB : RESERVED ROGUE **/

/* B4_0000_0000 - 0xB7_FFFF_FFFF **/
	/* 720 GiB to 736 GiB, size of 16 GiB : FREE **/

/* 0xB8_0000_0000 - 0xBF_FFFF_FFFF **/
	/* 736 GiB to 768 GiB, size of 32 GiB : GENERAL_NON4K_HEAP **/
	#define RGX_GENERAL_NON4K_HEAP_BASE         IMG_UINT64_C(0xB800000000)
	#define RGX_GENERAL_NON4K_HEAP_SIZE         RGX_HEAP_SIZE_32GiB


/* 0xC0_0000_0000 ************************************************************/

/* 0xC0_0000_0000 - 0xD9_FFFF_FFFF **/
	/* 768 GiB to 872 GiB, size of 104 GiB : FREE **/

/* 0xDA_0000_0000 - 0xDA_FFFF_FFFF **/
	/* 872 GiB to 876 GiB, size of 4 GiB : PDSCODEDATA_HEAP **/
	#define RGX_PDSCODEDATA_HEAP_BASE           IMG_UINT64_C(0xDA00000000)
	#define RGX_PDSCODEDATA_HEAP_SIZE           RGX_HEAP_SIZE_4GiB

/* 0xDB_0000_0000 - 0xDC_FFFF_FFFF **/
	/* 876 GiB to 884 GiB, size of 8 GiB : RESERVED ROGUE **/

/* 0xDD_0000_0000 - 0xDF_FFFF_FFFF **/
	/* 884 GiB to 896 GiB, size of 12 GiB : FREE **/

	/*
	 * The breakpoint handler code heap must share the same 4GiB address
	 * range as the USC shader code heap. The address space split is
	 * configurable.
	 *
	 * The breakpoint handler register is fixed, so the following parts
	 * of the BP handler address are static:
	 * [31:24] = 0xFE for 4064MiB offset after the USC code heap.
	 *   [5:0] = 0x00 i.e. aligned to 64 Byte boundary.
	 *
	 * The remaining part of the BP handler is dynamic and encoded in
	 * the USC instruction BABP_target_addr. The BP handler thus
	 * allows a range of 16 MiB with granularity of 64 Bytes.
	 */

/* 0xE0_0000_0000 - 0xE0_FDFF_FFFF **/
	/* 896 GiB to 900 GiB, size of 4 GiB less 32 MiB : USCCODE_HEAP **/
	#define RGX_USCCODE_HEAP_BASE               IMG_UINT64_C(0xE000000000)
	#define RGX_USCCODE_HEAP_SIZE               (RGX_HEAP_SIZE_4GiB - RGX_HEAP_SIZE_32MiB)

/* 0xE0_FE00_0000 - 0xE0_FEFF_FFFF **/
	/* 900 GiB less 32 MiB to 900 GiB less 16 MiB, size of 16 MiB : USCCODE_BPH_HEAP **/
	#define RGX_USCCODE_BPH_HEAP_BASE           (IMG_UINT64_C(0xE100000000) - RGX_HEAP_SIZE_32MiB)
	#define RGX_USCCODE_BPH_HEAP_SIZE           RGX_HEAP_SIZE_16MiB

/* 0xE0_FF00_0000 - 0xE0_FFFF_FFFF **/
	/* 900 GiB less 16 MiB to 900 GiB, size of 16 MiB : RESERVED **/

/* 0xE1_0000_0000 - 0xE1_BFFF_FFFF **/
	/* 900 GiB to 903 GiB, size of 3 GiB : RESERVED **/

/* 0xE1_C000_0000 - 0xE1_FFFF_FFFF **/
	/* 903 GiB to 904 GiB, reserved 1 GiB, : FIRMWARE_HEAP **/

	/* Firmware heaps defined in rgx_heap_firmware.h as they are not present in
	   application memory contexts, see:
	    RGX_FIRMWARE_RAW_HEAP_BASE
	    RGX_FIRMWARE_RAW_HEAP_SIZE
	   See header for other sub-heaps details
	*/

/* 0xE2_0000_0000 - 0xE2_FFFF_FFFF **/
	/* 904 GiB to 908 GiB, size of 4GiB : RESERVED ROGUE **/

/* 0xE3_0000_0000 - 0xE3_FFFF_FFFF **/
	/* 908 GiB to 912 GiB, size of 4 GiB : FREE **/

/* 0xE4_0000_0000 - 0xE7_FFFF_FFFF **/
	/* 912 GiB to 928 GiB, size 16 GiB : RESERVED_ROGUE **/

/* 0xE8_0000_0000 - 0xE8_FFFF_FFFF **/
	/* 928 GiB to 932 GiB, size of 4 GiB : FREE **/

/* 0xE9_0000_0000 - 0xE9_3FFF_FFFF **/
	/* 932 GiB to 933 GiB, size of 1 GiB : VK_CAPT_REPLAY_HEAP **/
	#define RGX_VK_CAPT_REPLAY_HEAP_BASE    IMG_UINT64_C(0xE900000000)
	#define RGX_VK_CAPT_REPLAY_HEAP_SIZE    RGX_HEAP_SIZE_1GiB

/* 0xE9_4000_0000 - 0xE9_FFFF_FFFF **/
	/* 933 GiB to 936 GiB, size of 3 GiB : FREE **/

/* 0xEA_0000_0000 - 0xEA_001F_FFFF **/
	/* 936 GiB to 937 GiB, size 2MB : SIGNALS_HEAP **/
	/* CDM Signals heap (31 signals less one reserved for Services).
	 * Size 960B rounded up to 2MB */
	#define RGX_SIGNALS_HEAP_BASE               IMG_UINT64_C(0xEA00000000)
	#define RGX_SIGNALS_HEAP_SIZE               RGX_HEAP_SIZE_2MiB

/* 0xEA_4000_0000 - 0xEA_FFFF_FFFF **/
	/* 937 GiB to 940 GiB, size of 3 GiB : FREE **/

/* 0xEB_0000_0000 - 0xEB_FFFF_FFFF **/
	/* 940 GiB to 944 GiB, size 4 GiB : COMPONENT_CTRL_HEAP **/
	#define RGX_COMPONENT_CTRL_HEAP_BASE        IMG_UINT64_C(0xEB00000000)
	#define RGX_COMPONENT_CTRL_HEAP_SIZE        RGX_HEAP_SIZE_4GiB

/* 0xEC_0000_0000 - 0xEC_001F_FFFF **/
	/* 944 GiB to 945 GiB, size 2 MiB : FBCDC_HEAP **/
	#define RGX_FBCDC_HEAP_BASE                 IMG_UINT64_C(0xEC00000000)
	#define RGX_FBCDC_HEAP_SIZE                 RGX_HEAP_SIZE_2MiB

/* 0xEC_4000_0000 - 0xEC_401F_FFFF **/
	/* 945 GiB to 946 GiB, size 2 MiB : FBCDC_LARGE_HEAP **/
	#define RGX_FBCDC_LARGE_HEAP_BASE           IMG_UINT64_C(0xEC40000000)
	#define RGX_FBCDC_LARGE_HEAP_SIZE           RGX_HEAP_SIZE_2MiB

/* 0xEC_8000_0000 - 0xEC_FFFF_FFFF **/
	/* 946 GiB to 948 GiB, size of 3 GiB : FREE **/

/* 0xED_0000_0000 - 0xED_00FF_FFFF */
	/* 948 GiB to 949 GiB, size 16 MiB : PDS_INDIRECT_STATE_HEAP */
	#define RGX_PDS_INDIRECT_STATE_HEAP_BASE    IMG_UINT64_C(0xED00000000)
	#define RGX_PDS_INDIRECT_STATE_HEAP_SIZE    RGX_HEAP_SIZE_16MiB

/* 0xED_4000_0000 - 0xED_FFFF_FFFF **/
	/* 949 GiB to 952 GiB, size of 3 GiB : FREE **/

/* 0xEE_0000_0000 - 0xEE_3FFF_FFFF **/
	/* 952 GiB to 953 GiB, size of 1 GiB : CMP_MISSION_RMW_HEAP **/
	#define RGX_CMP_MISSION_RMW_HEAP_BASE       IMG_UINT64_C(0xEE00000000)
	#define RGX_CMP_MISSION_RMW_HEAP_SIZE       RGX_HEAP_SIZE_1GiB

/* 0xEE_4000_0000 - 0xEE_FFFF_FFFF **/
	/* 953 GiB to 956 GiB, size of 3 GiB : RESERVED **/

/* 0xEF_0000_0000 - 0xEF_3FFF_FFFF **/
	/* 956 GiB to 957 GiB, size of 1 GiB : CMP_SAFETY_RMW_HEAP **/
	#define RGX_CMP_SAFETY_RMW_HEAP_BASE        IMG_UINT64_C(0xEF00000000)
	#define RGX_CMP_SAFETY_RMW_HEAP_SIZE        RGX_HEAP_SIZE_1GiB

/* 0xEF_4000_0000 - 0xEF_FFFF_FFFF **/
	/* 957 GiB to 960 GiB, size of 3 GiB : RESERVED **/

/* 0xF0_0000_0000 - 0xF0_FFFF_FFFF **/
	/* 960 GiB to 964 GiB, size of 4 GiB : TEXTURE_STATE_HEAP (36-bit aligned) */
	#define RGX_TEXTURE_STATE_HEAP_BASE         IMG_UINT64_C(0xF000000000)
	#define RGX_TEXTURE_STATE_HEAP_SIZE         RGX_HEAP_SIZE_4GiB

/* 0xF1_0000_0000 - 0xF1_FFFF_FFFF **/
	/* 964 GiB to 968 GiB, size of 4 GiB : FREE **/

/* 0xF2_0000_0000 - 0xF2_001F_FFFF **/
	/* 968 GiB to 969 GiB, size of 2 MiB : VISIBILITY_TEST_HEAP **/
	#define RGX_VISIBILITY_TEST_HEAP_BASE       IMG_UINT64_C(0xF200000000)
	#define RGX_VISIBILITY_TEST_HEAP_SIZE       RGX_HEAP_SIZE_2MiB

/* 0xF2_4000_0000 - 0xF2_FFFF_FFFF **/
	/* 969 GiB to 972 GiB, size of 3 GiB : FREE **/

/* 0xF3_0000_0000 - 0xF7_FFFF_FFFF **/
	/* 972 GiB to 992 GiB, size of 20 GiB : FREE **/

/* 0xF8_0000_0000 - 0xF9_FFFF_FFFF **/
	/* 992 GiB to 1000 GiB, size 8 GiB : RESERVED ROGUE **/

/* 0xFA_0000_0000 - 0xFF_FFFF_FFFF **/
	/* 1000 GiB to 1024 GiB, size of 24 GiB : FREE **/


/* 0xFF_FFFF_FFFF ************************************************************/

/*	End of RGX Device Virtual Address Space definitions */

#endif /* RGXHEAPCONFIG_H */

/******************************************************************************
 End of file (rgxheapconfig.h)
******************************************************************************/
