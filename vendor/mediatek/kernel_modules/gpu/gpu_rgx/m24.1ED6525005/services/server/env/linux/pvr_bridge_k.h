/*************************************************************************/ /*!
@File
@Title          PVR Bridge Module (kernel side)
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Receives calls from the user portion of services and
                despatches them to functions in the kernel portion.
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

#ifndef PVR_BRIDGE_K_H
#define PVR_BRIDGE_K_H

#include "pvrsrv_error.h"
#include "pvr_drv.h"

typedef struct LINUX_THREAD_ACTIVITY_STATS
{
	IMG_INT32 i32KernelThreadCount;
	IMG_INT32 i32DriverThreadCount;
	IMG_INT32 i32SuspendedThreadCount;
} LINUX_THREAD_ACTIVITY_STATS;

/*!
******************************************************************************
 @Function      LinuxBridgeBlockClientsAccess
 @Description   This function will wait for any existing threads in the Server
                to exit and then disable access to the driver. New threads will
                not be allowed to enter the Server until the driver is
                unsuspended (see LinuxBridgeUnblockClientsAccess).
 @Input         psDevPriv pointer to devices OS specific data
 @Input         bShutdown this flag indicates that the function was called
                          from a shutdown callback and therefore it will
                          not wait for the kernel threads to get frozen
                          (because this doesn't happen during shutdown
                          procedure)
 @Return        PVRSRV_ERROR
******************************************************************************/
PVRSRV_ERROR LinuxBridgeBlockClientsAccess(struct pvr_drm_private *psDevPriv,
                                           IMG_BOOL bShutdown);

/*!
******************************************************************************
 @Function      LinuxBridgeUnblockClientsAccess
 @Description   This function will re-enable the bridge and allow any threads
                waiting to enter the Server to continue.
 @Input         psDevPriv pointer to devices OS specific data
 @Return        PVRSRV_ERROR
******************************************************************************/
PVRSRV_ERROR LinuxBridgeUnblockClientsAccess(struct pvr_drm_private *psDevPriv);

void LinuxBridgeNumActiveKernelThreadsIncrement(void);
void LinuxBridgeNumActiveKernelThreadsDecrement(void);

/*************************************************************************/ /*!
 @Function      LinuxGetThreadActivityStats
 @Description   Getter for active and suspended thread stats.

 @Output        psThreadStats   Struct to be populated with thread activity
                                stats.

 @Return        PVRSRV_ERROR
*/ /**************************************************************************/
PVRSRV_ERROR LinuxGetThreadActivityStats(LINUX_THREAD_ACTIVITY_STATS *psThreadStats);

/*!
******************************************************************************
 @Function      PVRSRVDriverThreadEnter
 @Description   Increments number of client threads currently operating
                in the driver's context.
                If the driver is currently being suspended this function
                will call try_to_freeze() on behalf of the client thread.
                When the driver is resumed the function will exit and allow
                the thread into the driver.
 @Input         Reference to Connection data. NULL if no associated
                 connection / device.
 @Return        PVRSRV_ERROR
******************************************************************************/
PVRSRV_ERROR PVRSRVDriverThreadEnter(void *pvData);

/*!
******************************************************************************
 @Function      PVRSRVDriverThreadExit
 @Description   Decrements the number of client threads currently operating
                in the driver's context to match the call to
                PVRSRVDriverThreadEnter().
                The function also signals the driver that a thread left the
                driver context so if it's waiting to suspend it knows that
                the number of threads decreased.
 @Input         Reference to Connection data. NULL if no associated
                 connection / device.
******************************************************************************/
void PVRSRVDriverThreadExit(void *pvData);

#endif /* PVR_BRIDGE_K_H */
