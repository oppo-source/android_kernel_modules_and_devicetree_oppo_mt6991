/*************************************************************************/ /*!
@File
@Title          RGX compute functionality
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Header for the RGX compute functionality
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

#if !defined(RGXCOMPUTE_H)
#define RGXCOMPUTE_H

#include "devicemem.h"
#include "device.h"
#include "rgxfwutils.h"
#include "rgx_fwif_resetframework.h"
#include "rgxdebug_common.h"
#include "pvr_notifier.h"

#include "sync_server.h"
#include "sync_internal.h"
#include "connection_server.h"


typedef struct _RGX_SERVER_COMPUTE_CONTEXT_ RGX_SERVER_COMPUTE_CONTEXT;

/*************************************************************************/ /*!
@Function       PVRSRVRGXCreateComputeContextKM
@Description    Creates an RGX device context for submitting commands to CDM.
@Input          psConnection          Device connection
@Input          psDeviceNode          Services-managed device
@Input          i32Priority           Scheduling priority for commands
                                      on this context
@Input          ui32FrameworkCommandSize
                                      Framework command size
@Input          pabyFrameworkCommand  Pointer to framework command
@Input          hMemCtxPrivData       Private data
@Input          ui32StaticComputeContextStateSize
                                      Size of fixed compute context state
@Input          pStaticComputeContextState
                                      Compute context state
@Input          ui32PackedCCBSizeU88  Packed CCB size. The first byte contains
                                      the log2 CCB size and the second byte
                                      the log2 maximum CCB size.
@Input          ui32ContextFlags      Flags with context properties
@Input          ui64RobustnessAddress Address for FW to signal a context reset
@Input          ui32MaxDeadlineMS     Max deadline limit in MS that the
                                      workload can run
@Output         ppsComputeContext     Cleanup data
@Return         PVRSRV_ERROR          Returns PVRSRV_OK or an error.
*/ /**************************************************************************/
PVRSRV_ERROR PVRSRVRGXCreateComputeContextKM(CONNECTION_DATA			*psConnection,
											 PVRSRV_DEVICE_NODE			*psDeviceNode,
											 IMG_INT32					i32Priority,
											 IMG_UINT32					ui32FrameworkCommandSize,
											 IMG_PBYTE					pabyFrameworkCommand,
											 IMG_HANDLE					hMemCtxPrivData,
											 IMG_UINT32					ui32StaticComputeContextStateSize,
											 IMG_PBYTE					pStaticComputeContextState,
											 IMG_UINT32					ui32PackedCCBSizeU88,
											 IMG_UINT32					ui32ContextFlags,
											 IMG_UINT64					ui64RobustnessAddress,
											 IMG_UINT32					ui32MaxDeadlineMS,
											 RGX_SERVER_COMPUTE_CONTEXT	**ppsComputeContext);

/*!
*******************************************************************************
 @Function	PVRSRVRGXDestroyComputeContextKM

 @Description
	Server-side implementation of RGXDestroyComputeContext

 @Return   PVRSRV_ERROR
******************************************************************************/
PVRSRV_ERROR PVRSRVRGXDestroyComputeContextKM(RGX_SERVER_COMPUTE_CONTEXT *psComputeContext);


/*!
*******************************************************************************
 @Function	PVRSRVRGXKickCDMKM

 @Description
	Server-side implementation of RGXKickCDM2

 @Return   PVRSRV_ERROR
******************************************************************************/
PVRSRV_ERROR PVRSRVRGXKickCDMKM(RGX_SERVER_COMPUTE_CONTEXT	*psComputeContext,
								IMG_UINT32					ui32ClientUpdateCount,
								SYNC_PRIMITIVE_BLOCK		**pauiClientUpdateUFODevVarBlock,
								IMG_UINT32					*paui32ClientUpdateSyncOffset,
								IMG_UINT32					*paui32ClientUpdateValue,
								PVRSRV_FENCE				iCheckFence,
								PVRSRV_TIMELINE				iUpdateTimeline,
								PVRSRV_FENCE				*piUpdateFence,
								IMG_CHAR					pcszUpdateFenceName[PVRSRV_SYNC_NAME_LENGTH],
								IMG_UINT32					ui32CmdSize,
								IMG_PBYTE					pui8DMCmd,
								IMG_UINT32					ui32PDumpFlags,
								IMG_UINT32					ui32ExtJobRef,
								IMG_UINT32					ui32SyncPMRCount,
								IMG_UINT32					*paui32SyncPMRFlags,
								PMR							**ppsSyncPMRs,
								IMG_UINT32					ui32NumWorkgroups,
								IMG_UINT32					ui32NumWorkitems,
								IMG_UINT64					ui64DeadlineInus,
								IMG_PUINT32					pui32IntJobRef);

/*!
*******************************************************************************
 @Function	PVRSRVRGXKickCDMExpFenceKM

 @Description
	Server-side implementation of RGXKickCDM

 @Return   PVRSRV_ERROR
******************************************************************************/
PVRSRV_ERROR PVRSRVRGXKickCDMExpFenceKM(RGX_SERVER_COMPUTE_CONTEXT	*psComputeContext,
								IMG_UINT32					ui32ClientUpdateCount,
								SYNC_PRIMITIVE_BLOCK		**pauiClientUpdateUFODevVarBlock,
								IMG_UINT32					*paui32ClientUpdateSyncOffset,
								IMG_UINT32					*paui32ClientUpdateValue,
								PVRSRV_FENCE				iCheckFence,
								PVRSRV_TIMELINE				iUpdateTimeline,
								PVRSRV_FENCE				*piUpdateFence,
								IMG_CHAR					pcszUpdateFenceName[PVRSRV_SYNC_NAME_LENGTH],
								PVRSRV_FENCE				iExportFenceToSignal,
								IMG_UINT32					ui32CmdSize,
								IMG_PBYTE					pui8DMCmd,
								IMG_UINT32					ui32PDumpFlags,
								IMG_UINT32					ui32ExtJobRef,
								IMG_UINT32					ui32SyncPMRCount,
								IMG_UINT32					*paui32SyncPMRFlags,
								PMR							**ppsSyncPMRs,
								IMG_UINT32					ui32NumWorkgroups,
								IMG_UINT32					ui32NumWorkitems,
								IMG_UINT64					ui64DeadlineInus,
								IMG_PUINT32					pui32IntJobRef);

/*!
*******************************************************************************
 @Function	PVRSRVRGXFlushComputeDataKM

 @Description
	Server-side implementation of RGXFlushComputeData

 @Input psComputeContext - Compute context to flush

 @Return   PVRSRV_ERROR
******************************************************************************/
PVRSRV_ERROR PVRSRVRGXFlushComputeDataKM(RGX_SERVER_COMPUTE_CONTEXT *psComputeContext);



/*!
*******************************************************************************
 @Function	PVRSRVRGXSendCancelCmd

 @Description
	Server-side implementation of RGXSendCancelCmd

 @Input psComputeContext - Compute context to cancel work on
 @Input ui32LastIntJobRefToCancel - Last IntJobRef to cancel

 @Return   PVRSRV_ERROR
******************************************************************************/
PVRSRV_ERROR PVRSRVRGXSendCancelCmdKM(RGX_SERVER_COMPUTE_CONTEXT *psComputeContext,
                                      IMG_UINT32 ui32FirstIntJobRefToCancel,
                                      IMG_UINT32 ui32LastIntJobRefToCancel);


/*!
*******************************************************************************

 @Function	    PVRSRVRGXNotifyComputeWriteOffsetUpdateKM
 @Description   Server-side implementation of RGXNotifyComputeWriteOffsetUpdate

 @Input         psComputeContext - Compute context to flush

 @Return   PVRSRV_ERROR

******************************************************************************/
PVRSRV_ERROR PVRSRVRGXNotifyComputeWriteOffsetUpdateKM(RGX_SERVER_COMPUTE_CONTEXT *psComputeContext);

PVRSRV_ERROR PVRSRVRGXSetComputeContextPriorityKM(CONNECTION_DATA *psConnection,
												  PVRSRV_DEVICE_NODE *psDeviceNode,
												  RGX_SERVER_COMPUTE_CONTEXT *psComputeContext,
												  IMG_INT32 i32Priority);

PVRSRV_ERROR PVRSRVRGXSetComputeContextPropertyKM(RGX_SERVER_COMPUTE_CONTEXT *psComputeContext,
												  RGX_CONTEXT_PROPERTY eContextProperty,
												  IMG_UINT64 ui64Input,
												  IMG_UINT64 *pui64Output);

PVRSRV_ERROR PVRSRVRGXGetLastDeviceErrorKM(CONNECTION_DATA    *psConnection,
                                           PVRSRV_DEVICE_NODE *psDeviceNode,
                                           IMG_UINT32         *ui32Error);

PVRSRV_ERROR PVRSRVRGXKickTimestampQueryKM(RGX_SERVER_COMPUTE_CONTEXT *psComputeContext,
                                           PVRSRV_FENCE iCheckFence,
										   PVRSRV_TIMELINE iUpdateTimeline,
										   PVRSRV_FENCE *piUpdateFence,
                                           IMG_CHAR pszUpdateFenceName[PVRSRV_SYNC_NAME_LENGTH],
                                           IMG_UINT32 ui32CmdSize,
                                           IMG_PBYTE pui8DMCmd,
                                           IMG_UINT32 ui32ExtJobRef);

/* Debug - Dump debug info of compute contexts on this device */
void DumpComputeCtxtsInfo(PVRSRV_RGXDEV_INFO *psDevInfo,
                          DUMPDEBUG_PRINTF_FUNC *pfnDumpDebugPrintf,
                          void *pvDumpDebugFile,
                          IMG_UINT32 ui32VerbLevel);

/* Debug/Watchdog - check if client compute contexts are stalled */
IMG_UINT32 CheckForStalledClientComputeCtxt(PVRSRV_RGXDEV_INFO *psDevInfo);

PVRSRV_ERROR PVRSRVRGXCDMGetSharedMemoryKM(
	CONNECTION_DATA           * psConnection,
	PVRSRV_DEVICE_NODE        * psDeviceNode,
	PMR                      ** ppsCLIPMRMem,
	PMR                      ** ppsUSCPMRMem);

PVRSRV_ERROR PVRSRVRGXCDMReleaseSharedMemoryKM(PMR * psUSCPMRMem);

#endif /* RGXCOMPUTE_H */
