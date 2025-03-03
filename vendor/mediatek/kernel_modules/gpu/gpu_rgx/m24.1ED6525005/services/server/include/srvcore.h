/**************************************************************************/ /*!
@File
@Title          PVR Bridge Functionality
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Header for the PVR Bridge code
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
*/ /***************************************************************************/

#ifndef SRVCORE_H
#define SRVCORE_H

#include "lock_types.h"
#include "connection_server.h"
#include "pvr_debug.h"

#include "pvr_bridge.h"
#if defined(SUPPORT_RGX)
#include "rgx_bridge.h"
#endif

PVRSRV_ERROR
CopyFromUserWrapper(CONNECTION_DATA *psConnection,
                    IMG_UINT32 ui32DispatchTableEntry,
                    void *pvDest,
                    void __user *pvSrc,
                    IMG_UINT32 ui32Size);
PVRSRV_ERROR
CopyToUserWrapper(CONNECTION_DATA *psConnection,
                  IMG_UINT32 ui32DispatchTableEntry,
                  void __user *pvDest,
                  void *pvSrc,
                  IMG_UINT32 ui32Size);

IMG_INT
DummyBW(IMG_UINT32 ui32DispatchTableEntry,
        IMG_UINT8 *psBridgeIn,
        IMG_UINT8 *psBridgeOut,
        CONNECTION_DATA *psConnection);

typedef PVRSRV_ERROR (*ServerResourceDestroyFunction)(IMG_HANDLE, IMG_HANDLE);

typedef IMG_INT (*BridgeWrapperFunction)(IMG_UINT32 ui32DispatchTableEntry,
									 IMG_UINT8 *psBridgeIn,
									 IMG_UINT8 *psBridgeOut,
									 CONNECTION_DATA *psConnection);

typedef struct _PVRSRV_BRIDGE_DISPATCH_TABLE_ENTRY
{
	BridgeWrapperFunction pfFunction; /*!< The wrapper function that validates the ioctl
	                                    arguments before calling into srvkm proper */
	POS_LOCK hBridgeLock; /*!< The bridge lock which needs to be acquired
                                   before calling the above wrapper */

	IMG_UINT32 ui32InBufferSize; /*!< The expected size of the in buffer given by the client */
	IMG_UINT32 ui32OutBufferSize; /*!< The expected size of the out buffer given by the client */

#if defined(DEBUG_BRIDGE_KM)
	const IMG_CHAR *pszIOCName; /*!< Name of the ioctl: e.g. "PVRSRV_BRIDGE_CONNECT_SERVICES" */
	const IMG_CHAR *pszFunctionName; /*!< Name of the wrapper function: e.g. "PVRSRVConnectBW" */
	const IMG_CHAR *pszBridgeLockName; /*!< Name of bridge lock which will be acquired */
	IMG_UINT32 ui32CallCount; /*!< The total number of times the ioctl has been called */
	IMG_UINT32 ui32CopyFromUserTotalBytes; /*!< The total number of bytes copied from
	                                         userspace within this ioctl */
	IMG_UINT32 ui32CopyToUserTotalBytes; /*!< The total number of bytes copied from
	                                       userspace within this ioctl */
	IMG_UINT64 ui64TotalTimeNS; /*!< The total amount of time spent in this bridge function */
	IMG_UINT64 ui64MaxTimeNS; /*!< The maximum amount of time for a single call to this bridge function */
#endif
}PVRSRV_BRIDGE_DISPATCH_TABLE_ENTRY;

#if defined(SUPPORT_RGX)
	#define BRIDGE_DISPATCH_TABLE_ENTRY_COUNT  (PVRSRV_BRIDGE_RGX_DISPATCH_LAST+1)
	#define BRIDGE_DISPATCH_TABLE_START_ENTRY_COUNT  (PVRSRV_BRIDGE_RGX_LAST+1)
#else
	#define BRIDGE_DISPATCH_TABLE_ENTRY_COUNT  (PVRSRV_BRIDGE_DISPATCH_LAST+1)
	#define BRIDGE_DISPATCH_TABLE_START_ENTRY_COUNT  (PVRSRV_BRIDGE_LAST+1)
#endif

extern PVRSRV_BRIDGE_DISPATCH_TABLE_ENTRY g_BridgeDispatchTable[BRIDGE_DISPATCH_TABLE_ENTRY_COUNT];

void BridgeDispatchTableStartOffsetsInit(void);

void
_SetDispatchTableEntry(IMG_UINT32 ui32BridgeGroup,
                       IMG_UINT32 ui32Index,
                       const IMG_CHAR *pszIOCName,
                       BridgeWrapperFunction pfFunction,
                       const IMG_CHAR *pszFunctionName,
                       POS_LOCK hBridgeLock,
                       const IMG_CHAR* pszBridgeLockName,
                       IMG_UINT32 ui32InBufferSize,
                       IMG_UINT32 ui32OutBufferSize);
void
UnsetDispatchTableEntry(IMG_UINT32 ui32BridgeGroup,
                        IMG_UINT32 ui32Index);


/* PRQA S 0884,3410 2*/ /* macro relies on the lack of brackets */
#define SetDispatchTableEntry(ui32BridgeGroup, ui32Index, pfFunction,\
                              hBridgeLock, ui32InBufferSize, ui32OutBufferSize) \
	do \
	{ \
		static_assert((ui32InBufferSize) <= PVRSRV_MAX_BRIDGE_IN_SIZE, "Bridge input buffer is too small for bridge function: " #pfFunction); \
		static_assert((ui32OutBufferSize) <= PVRSRV_MAX_BRIDGE_OUT_SIZE, "Bridge output buffer is too small for bridge function: " #pfFunction); \
		_SetDispatchTableEntry(ui32BridgeGroup, ui32Index, #ui32Index, (BridgeWrapperFunction)pfFunction, #pfFunction,\
		                       (POS_LOCK)hBridgeLock, #hBridgeLock, ui32InBufferSize, ui32OutBufferSize); \
	} while (0)

#define DISPATCH_TABLE_GAP_THRESHOLD 5


#if defined(DEBUG_BRIDGE_KM)
typedef struct _PVRSRV_BRIDGE_GLOBAL_STATS
{
	IMG_UINT32 ui32IOCTLCount;
	IMG_UINT32 ui32TotalCopyFromUserBytes;
	IMG_UINT32 ui32TotalCopyToUserBytes;
} PVRSRV_BRIDGE_GLOBAL_STATS;

void BridgeGlobalStatsLock(void);
void BridgeGlobalStatsUnlock(void);

/* OS specific code may want to report the stats held here and within the
 * BRIDGE_DISPATCH_TABLE_ENTRYs (E.g. on Linux we report these via a
 * debugfs entry /(sys/kernel/debug|proc)/pvr/bridge_stats) */
extern PVRSRV_BRIDGE_GLOBAL_STATS g_BridgeGlobalStats;
#endif

PVRSRV_ERROR BridgeDispatcherInit(void);
void BridgeDispatcherDeinit(void);

PVRSRV_ERROR
BridgedDispatchKM(CONNECTION_DATA * psConnection,
                  PVRSRV_BRIDGE_PACKAGE   * psBridgePackageKM);

PVRSRV_ERROR
PVRSRVConnectKM(CONNECTION_DATA *psConnection,
                PVRSRV_DEVICE_NODE * psDeviceNode,
                IMG_UINT32 ui32Flags,
                IMG_UINT32 ui32ClientBuildOptions,
                IMG_UINT32 ui32ClientDDKVersion,
                IMG_UINT32 ui32ClientDDKBuild,
                IMG_UINT8  *pui8KernelArch,
                IMG_UINT32 *ui32CapabilityFlags,
                IMG_UINT64 *ui64PackedBvnc);

PVRSRV_ERROR
PVRSRVDisconnectKM(void);

PVRSRV_ERROR
PVRSRVAcquireGlobalEventObjectKM(IMG_HANDLE *phGlobalEventObject);

PVRSRV_ERROR
PVRSRVReleaseGlobalEventObjectKM(IMG_HANDLE hGlobalEventObject);

PVRSRV_ERROR
PVRSRVDumpDebugInfoKM(CONNECTION_DATA *psConnection,
                      PVRSRV_DEVICE_NODE *psDeviceNode,
                      IMG_UINT32 ui32VerbLevel);

PVRSRV_ERROR
PVRSRVGetDevClockSpeedKM(CONNECTION_DATA * psConnection,
                         PVRSRV_DEVICE_NODE *psDeviceNode,
                         IMG_PUINT32  pui32RGXClockSpeed);

PVRSRV_ERROR
PVRSRVHWOpTimeoutKM(CONNECTION_DATA *psConnection,
                    PVRSRV_DEVICE_NODE *psDeviceNode);

PVRSRV_ERROR PVRSRVAlignmentCheckKM(CONNECTION_DATA *psConnection,
                                    PVRSRV_DEVICE_NODE * psDeviceNode,
                                    IMG_UINT32 ui32FWAlignChecksSize,
                                    IMG_UINT32 aui32FWAlignChecks[]);

PVRSRV_ERROR PVRSRVGetDeviceStatusKM(CONNECTION_DATA *psConnection,
                                     PVRSRV_DEVICE_NODE *psDeviceNode,
                                     IMG_UINT32 *pui32DeviceStatus);

PVRSRV_ERROR PVRSRVGetMultiCoreInfoKM(CONNECTION_DATA *psConnection,
                                     PVRSRV_DEVICE_NODE *psDeviceNode,
                                     IMG_UINT32 ui32CapsSize,
                                     IMG_UINT32 *pui32NumCores,
                                     IMG_UINT64 *pui64Caps);

PVRSRV_ERROR PVRSRVFindProcessMemStatsKM(IMG_PID pid,
                                         IMG_UINT32 ui32ArrSize,
                                         IMG_BOOL bAllProcessStats,
                                         IMG_UINT64 *pui64MemoryStats);

static INLINE
PVRSRV_ERROR DestroyServerResource(const SHARED_DEV_CONNECTION hConnection,
                                   IMG_HANDLE hEvent,
                                   ServerResourceDestroyFunction pfnDestroyCall,
                                   IMG_HANDLE hResource)
{
    PVR_UNREFERENCED_PARAMETER(hEvent);

    return pfnDestroyCall(GetBridgeHandle(hConnection), hResource);
}

/*************************************************************************/ /*!
@Function      PVRSRVBlockIfFrozen
@Description   Puts caller into a blocking wait (using event-objects) if the
                specified device is in a FROZEN state. Routine completes and
                returns control to the caller once the underlying device state
                clears its FROZEN state.
@Input         psDeviceNode     Device Node reference
@Return        Nothing.         Execution blocks if device is frozen.
*/ /**************************************************************************/
void PVRSRVBlockIfFrozen(PVRSRV_DEVICE_NODE *psDeviceNode);

#endif /* SRVCORE_H */

/******************************************************************************
 End of file (srvcore.h)
******************************************************************************/
