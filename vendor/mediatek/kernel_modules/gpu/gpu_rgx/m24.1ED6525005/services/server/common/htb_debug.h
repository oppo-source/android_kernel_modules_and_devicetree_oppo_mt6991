/*************************************************************************/ /*!
@File           htb_debug.h
@Title          Linux debugFS routine setup header
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

#ifndef HTB_DEBUG_H
#define HTB_DEBUG_H

/**************************************************************************/ /*!
 @Function     HTB_CreateDIEntry

 @Description  Create the debugFS entry-point for the host-trace-buffer

 @Returns      eError          internal error code, PVRSRV_OK on success

 */ /**************************************************************************/
PVRSRV_ERROR HTB_CreateDIEntry_Impl(void);

/**************************************************************************/ /*!
 @Function     HTB_DestroyFSEntry

 @Description  Destroy the debugFS entry-point created by earlier
               HTB_CreateDIEntry() call.
*/ /**************************************************************************/
void HTB_DestroyDIEntry_Impl(void);

#if defined(PVRSRV_ENABLE_HTB)
#define HTB_CreateDIEntry() HTB_CreateDIEntry_Impl()
#define HTB_DestroyDIEntry() HTB_DestroyDIEntry_Impl()
#else /* !PVRSRV_ENABLE_HTB */
#define HTB_CreateDIEntry() PVRSRV_OK
#define HTB_DestroyDIEntry()
#endif /* PVRSRV_ENABLE_HTB */

#endif /* HTB_DEBUG_H */
