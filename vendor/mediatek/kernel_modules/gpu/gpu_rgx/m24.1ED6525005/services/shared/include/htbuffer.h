/*************************************************************************/ /*!
@File           htbuffer.h
@Title          Host Trace Buffer shared API.
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Host Trace Buffer provides a mechanism to log Host events to a
                buffer in a similar way to the Firmware Trace mechanism.
                Host Trace Buffer logs data using a Transport Layer buffer.
                The Transport Layer and pvrtld tool provides the mechanism to
                retrieve the trace data.
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
#ifndef HTBUFFER_H
#define HTBUFFER_H

#if defined(__cplusplus)
extern "C" {
#endif

#include "img_types.h"
#include "img_defs.h"
#include "pvrsrv_error.h"
#include "htbuffer_sf.h"
#include "htbuffer_types.h"

/*************************************************************************/ /*!
 @Function      HTBControl
 @Description   Update the configuration of the Host Trace Buffer
 @Input         hSrvHandle      Server Handle
 @Input         ui32NumFlagGroups Number of group enable flags words
 @Input         aui32GroupEnable  Flags words controlling groups to be logged
 @Input         ui32LogLevel    Log level to record
 @Input         ui32EnablePID   PID to enable logging for a specific process
 @Input         eLogPidMode     Enable logging for all or specific processes,
 @Input         eOpMode         Control what trace data is dropped if the TL
                                buffer is full
 @Return        eError          Internal services call returned eError error
                                number
*/ /**************************************************************************/
IMG_INTERNAL PVRSRV_ERROR
HTBControl(
	IMG_HANDLE hSrvHandle,
	IMG_UINT32 ui32NumFlagGroups,
	IMG_UINT32 * aui32GroupEnable,
	IMG_UINT32 ui32LogLevel,
	IMG_UINT32 ui32EnablePID,
	HTB_LOGMODE_CTRL eLogPidMode,
	HTB_OPMODE_CTRL eOpMode);


#if defined(__cplusplus)
}
#endif

#endif /* HTBUFFER_H */
/*****************************************************************************
 End of file (htbuffer.h)
*****************************************************************************/
