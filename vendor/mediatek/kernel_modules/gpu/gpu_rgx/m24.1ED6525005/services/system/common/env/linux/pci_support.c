/*************************************************************************/ /*!
@File
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

#include <linux/version.h>
#include <linux/pci.h>

#if defined(CONFIG_MTRR)
#include <asm/mtrr.h>
#endif

#include "pci_support.h"
#include "allocmem.h"

typedef	struct _PVR_PCI_DEV_TAG
{
	struct pci_dev		*psPCIDev;
	HOST_PCI_INIT_FLAGS	ePCIFlags;
	IMG_BOOL		abPCIResourceInUse[DEVICE_COUNT_RESOURCE];
	int			iMTRR[DEVICE_COUNT_RESOURCE];
} PVR_PCI_DEV;

/*************************************************************************/ /*!
@Function       OSPCISetDev
@Description    Set a PCI device for subsequent use.
@Input          pvPCICookie             Pointer to OS specific PCI structure
@Input          eFlags                  Flags
@Return		PVRSRV_PCI_DEV_HANDLE   Pointer to PCI device handle
*/ /**************************************************************************/
PVRSRV_PCI_DEV_HANDLE OSPCISetDev(void *pvPCICookie, HOST_PCI_INIT_FLAGS eFlags)
{
	int err;
	IMG_UINT32 i;
	PVR_PCI_DEV *psPVRPCI;

	psPVRPCI = OSAllocMem(sizeof(*psPVRPCI));
	if (psPVRPCI == NULL)
	{
		printk(KERN_ERR "OSPCISetDev: Couldn't allocate PVR PCI structure\n");
		return NULL;
	}

	psPVRPCI->psPCIDev = (struct pci_dev *)pvPCICookie;
	psPVRPCI->ePCIFlags = eFlags;

	err = pci_enable_device(psPVRPCI->psPCIDev);
	if (err != 0)
	{
		printk(KERN_ERR "OSPCISetDev: Couldn't enable device (%d)\n", err);
		OSFreeMem(psPVRPCI);
		return NULL;
	}

	if (psPVRPCI->ePCIFlags & HOST_PCI_INIT_FLAG_BUS_MASTER)	/* PRQA S 3358 */ /* misuse of enums */
	{
		pci_set_master(psPVRPCI->psPCIDev);
	}

	if (psPVRPCI->ePCIFlags & HOST_PCI_INIT_FLAG_MSI)		/* PRQA S 3358 */ /* misuse of enums */
	{
#if defined(CONFIG_PCI_MSI)
		err = pci_enable_msi(psPVRPCI->psPCIDev);
		if (err != 0)
		{
			printk(KERN_ERR "OSPCISetDev: Couldn't enable MSI (%d)", err);
			psPVRPCI->ePCIFlags &= ~HOST_PCI_INIT_FLAG_MSI;	/* PRQA S 1474,3358,4130 */ /* misuse of enums */
		}
#else
		printk(KERN_ERR "OSPCISetDev: MSI support not enabled in the kernel");
#endif
	}

	/* Initialise the PCI resource and MTRR tracking array */
	for (i = 0; i < DEVICE_COUNT_RESOURCE; i++)
	{
		psPVRPCI->abPCIResourceInUse[i] = IMG_FALSE;
		psPVRPCI->iMTRR[i] = -1;
	}

	return (PVRSRV_PCI_DEV_HANDLE)psPVRPCI;
}

/*************************************************************************/ /*!
@Function       OSPCIAcquireDev
@Description    Acquire a PCI device for subsequent use.
@Input          ui16VendorID            Vendor PCI ID
@Input          ui16DeviceID            Device PCI ID
@Input          eFlags                  Flags
@Return		PVRSRV_PCI_DEV_HANDLE   Pointer to PCI device handle
*/ /**************************************************************************/
PVRSRV_PCI_DEV_HANDLE OSPCIAcquireDev(IMG_UINT16 ui16VendorID,
				      IMG_UINT16 ui16DeviceID,
				      HOST_PCI_INIT_FLAGS eFlags)
{
	struct pci_dev *psPCIDev;

	psPCIDev = pci_get_device(ui16VendorID, ui16DeviceID, NULL);
	if (psPCIDev == NULL)
	{
		return NULL;
	}

	return OSPCISetDev((void *)psPCIDev, eFlags);
}

/*************************************************************************/ /*!
@Function       OSPCIIRQ
@Description    Get the interrupt number for the device.
@Input          hPVRPCI                 PCI device handle
@Output         pui16DeviceID           Pointer to where the interrupt number
                                        should be returned
@Return		PVRSRV_ERROR            Services error code
*/ /**************************************************************************/
PVRSRV_ERROR OSPCIIRQ(PVRSRV_PCI_DEV_HANDLE hPVRPCI, IMG_UINT32 *pui32IRQ)
{
	PVR_PCI_DEV *psPVRPCI = (PVR_PCI_DEV *)hPVRPCI;

	if (pui32IRQ == NULL)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	*pui32IRQ = psPVRPCI->psPCIDev->irq;

	return PVRSRV_OK;
}

/* Functions supported by OSPCIAddrRangeFunc */
enum HOST_PCI_ADDR_RANGE_FUNC
{
	HOST_PCI_ADDR_RANGE_FUNC_LEN,
	HOST_PCI_ADDR_RANGE_FUNC_START,
	HOST_PCI_ADDR_RANGE_FUNC_END,
	HOST_PCI_ADDR_RANGE_FUNC_REQUEST,
	HOST_PCI_ADDR_RANGE_FUNC_RELEASE
};

/*************************************************************************/ /*!
@Function       OSPCIAddrRangeFunc
@Description    Internal support function for various address range related
                functions
@Input          eFunc                   Function to perform
@Input          hPVRPCI                 PCI device handle
@Input          ui32Index               Address range index
@Return		IMG_UINT32              Function dependent value
*/ /**************************************************************************/
static IMG_UINT64 OSPCIAddrRangeFunc(enum HOST_PCI_ADDR_RANGE_FUNC eFunc,
										 PVRSRV_PCI_DEV_HANDLE hPVRPCI,
										 IMG_UINT32 ui32Index)
{
	PVR_PCI_DEV *psPVRPCI = (PVR_PCI_DEV *)hPVRPCI;

	if (ui32Index >= DEVICE_COUNT_RESOURCE)
	{
		printk(KERN_ERR "OSPCIAddrRangeFunc: Index out of range");
		return 0;
	}

	switch (eFunc)
	{
		case HOST_PCI_ADDR_RANGE_FUNC_LEN:
		{
			return pci_resource_len(psPVRPCI->psPCIDev, ui32Index);
		}
		case HOST_PCI_ADDR_RANGE_FUNC_START:
		{
			return pci_resource_start(psPVRPCI->psPCIDev, ui32Index);
		}
		case HOST_PCI_ADDR_RANGE_FUNC_END:
		{
			return pci_resource_end(psPVRPCI->psPCIDev, ui32Index);
		}
		case HOST_PCI_ADDR_RANGE_FUNC_REQUEST:
		{
			int err = pci_request_region(psPVRPCI->psPCIDev, (IMG_INT)ui32Index, PVRSRV_MODNAME);
			if (err != 0)
			{
				printk(KERN_ERR "OSPCIAddrRangeFunc: pci_request_region_failed (%d)", err);
				return 0;
			}
			psPVRPCI->abPCIResourceInUse[ui32Index] = IMG_TRUE;
			return 1;
		}
		case HOST_PCI_ADDR_RANGE_FUNC_RELEASE:
		{
			if (psPVRPCI->abPCIResourceInUse[ui32Index])
			{
				pci_release_region(psPVRPCI->psPCIDev, (IMG_INT)ui32Index);
				psPVRPCI->abPCIResourceInUse[ui32Index] = IMG_FALSE;
			}
			return 1;
		}
		default:
		{
			printk(KERN_ERR "OSPCIAddrRangeFunc: Unknown function");
			break;
		}
	}

	return 0;
}

/*************************************************************************/ /*!
@Function       OSPCIAddrRangeLen
@Description    Returns length of a given address range
@Input          hPVRPCI                 PCI device handle
@Input          ui32Index               Address range index
@Return		IMG_UINT32              Length of address range or 0 if no
                                        such range
*/ /**************************************************************************/
IMG_UINT64 OSPCIAddrRangeLen(PVRSRV_PCI_DEV_HANDLE hPVRPCI, IMG_UINT32 ui32Index)
{
	return OSPCIAddrRangeFunc(HOST_PCI_ADDR_RANGE_FUNC_LEN, hPVRPCI, ui32Index);
}

/*************************************************************************/ /*!
@Function       OSPCIAddrRangeStart
@Description    Returns the start of a given address range
@Input          hPVRPCI                 PCI device handle
@Input          ui32Index               Address range index
@Return		IMG_UINT32              Start of address range or 0 if no
                                        such range
*/ /**************************************************************************/
IMG_UINT64 OSPCIAddrRangeStart(PVRSRV_PCI_DEV_HANDLE hPVRPCI, IMG_UINT32 ui32Index)
{
	return OSPCIAddrRangeFunc(HOST_PCI_ADDR_RANGE_FUNC_START, hPVRPCI, ui32Index);
}

/*************************************************************************/ /*!
@Function       OSPCIAddrRangeEnd
@Description    Returns the end of a given address range
@Input          hPVRPCI                 PCI device handle
@Input          ui32Index               Address range index
@Return		IMG_UINT32              End of address range or 0 if no such
                                        range
*/ /**************************************************************************/
IMG_UINT64 OSPCIAddrRangeEnd(PVRSRV_PCI_DEV_HANDLE hPVRPCI, IMG_UINT32 ui32Index)
{
	return OSPCIAddrRangeFunc(HOST_PCI_ADDR_RANGE_FUNC_END, hPVRPCI, ui32Index);
}

/*************************************************************************/ /*!
@Function       OSPCIRequestAddrRange
@Description    Request a given address range index for subsequent use
@Input          hPVRPCI                 PCI device handle
@Input          ui32Index               Address range index
@Return	        PVRSRV_ERROR	        Services error code
*/ /**************************************************************************/
PVRSRV_ERROR OSPCIRequestAddrRange(PVRSRV_PCI_DEV_HANDLE hPVRPCI,
								   IMG_UINT32 ui32Index)
{
	if (OSPCIAddrRangeFunc(HOST_PCI_ADDR_RANGE_FUNC_REQUEST, hPVRPCI, ui32Index) == 0)
	{
		return PVRSRV_ERROR_PCI_CALL_FAILED;
	}
	else
	{
		return PVRSRV_OK;
	}
}

/*************************************************************************/ /*!
@Function       OSPCIReleaseAddrRange
@Description    Release a given address range that is no longer being used
@Input          hPVRPCI                 PCI device handle
@Input          ui32Index               Address range index
@Return	        PVRSRV_ERROR	        Services error code
*/ /**************************************************************************/
PVRSRV_ERROR OSPCIReleaseAddrRange(PVRSRV_PCI_DEV_HANDLE hPVRPCI, IMG_UINT32 ui32Index)
{
	if (OSPCIAddrRangeFunc(HOST_PCI_ADDR_RANGE_FUNC_RELEASE, hPVRPCI, ui32Index) == 0)
	{
		return PVRSRV_ERROR_PCI_CALL_FAILED;
	}
	else
	{
		return PVRSRV_OK;
	}
}

/*************************************************************************/ /*!
@Function       OSPCIRequestAddrRegion
@Description    Request a given region from an address range for subsequent use
@Input          hPVRPCI                 PCI device handle
@Input          ui32Index               Address range index
@Input          uiOffset              Offset into the address range that forms
                                        the start of the region
@Input          uiLength              Length of the region
@Return	        PVRSRV_ERROR	        Services error code
*/ /**************************************************************************/
PVRSRV_ERROR OSPCIRequestAddrRegion(PVRSRV_PCI_DEV_HANDLE hPVRPCI,
									IMG_UINT32 ui32Index,
									IMG_UINT64 uiOffset,
									IMG_UINT64 uiLength)
{
	PVR_PCI_DEV *psPVRPCI = (PVR_PCI_DEV *)hPVRPCI;
	resource_size_t start;
	resource_size_t end;

	start = pci_resource_start(psPVRPCI->psPCIDev, ui32Index);
	end = pci_resource_end(psPVRPCI->psPCIDev, ui32Index);

	/* Check that the requested region is valid */
	if ((start + uiOffset + uiLength - 1) > end)
	{
		return PVRSRV_ERROR_BAD_REGION_SIZE_MISMATCH;
	}

	if (pci_resource_flags(psPVRPCI->psPCIDev, ui32Index) & IORESOURCE_IO)
	{
		if (request_region(start + uiOffset, uiLength, PVRSRV_MODNAME) == NULL)
		{
			return PVRSRV_ERROR_PCI_REGION_UNAVAILABLE;
		}
	}
	else
	{
		if (request_mem_region(start + uiOffset, uiLength, PVRSRV_MODNAME) == NULL)
		{
			return PVRSRV_ERROR_PCI_REGION_UNAVAILABLE;
		}
	}

	return PVRSRV_OK;
}

/*************************************************************************/ /*!
@Function       OSPCIReleaseAddrRegion
@Description    Release a given region, from an address range, that is no
                longer in use
@Input          hPVRPCI                 PCI device handle
@Input          ui32Index               Address range index
@Input          ui32Offset              Offset into the address range that forms
                                        the start of the region
@Input          ui32Length              Length of the region
@Return	        PVRSRV_ERROR	        Services error code
*/ /**************************************************************************/
PVRSRV_ERROR OSPCIReleaseAddrRegion(PVRSRV_PCI_DEV_HANDLE hPVRPCI,
									IMG_UINT32 ui32Index,
									IMG_UINT64 uiOffset,
									IMG_UINT64 uiLength)
{
	PVR_PCI_DEV *psPVRPCI = (PVR_PCI_DEV *)hPVRPCI;
	resource_size_t start;
	resource_size_t end;

	start = pci_resource_start(psPVRPCI->psPCIDev, ui32Index);
	end = pci_resource_end(psPVRPCI->psPCIDev, ui32Index);

	/* Check that the region is valid */
	if ((start + uiOffset + uiLength - 1) > end)
	{
		return PVRSRV_ERROR_BAD_REGION_SIZE_MISMATCH;
	}

	if (pci_resource_flags(psPVRPCI->psPCIDev, ui32Index) & IORESOURCE_IO)
	{
		release_region(start + uiOffset, uiLength);
	}
	else
	{
		release_mem_region(start + uiOffset, uiLength);
	}

	return PVRSRV_OK;
}

/*************************************************************************/ /*!
@Function       OSPCIReleaseDev
@Description    Release a PCI device that is no longer being used
@Input          hPVRPCI                 PCI device handle
@Return	        PVRSRV_ERROR	        Services error code
*/ /**************************************************************************/
PVRSRV_ERROR OSPCIReleaseDev(PVRSRV_PCI_DEV_HANDLE hPVRPCI)
{
	PVR_PCI_DEV *psPVRPCI = (PVR_PCI_DEV *)hPVRPCI;
	int i;

	/* Release all PCI regions that are currently in use */
	for (i = 0; i < DEVICE_COUNT_RESOURCE; i++)
	{
		if (psPVRPCI->abPCIResourceInUse[i])
		{
			pci_release_region(psPVRPCI->psPCIDev, i);
			psPVRPCI->abPCIResourceInUse[i] = IMG_FALSE;
		}
	}

#if defined(CONFIG_PCI_MSI)
	if (psPVRPCI->ePCIFlags & HOST_PCI_INIT_FLAG_MSI)		/* PRQA S 3358 */ /* misuse of enums */
	{
		pci_disable_msi(psPVRPCI->psPCIDev);
	}
#endif

	if (psPVRPCI->ePCIFlags & HOST_PCI_INIT_FLAG_BUS_MASTER)	/* PRQA S 3358 */ /* misuse of enums */
	{
		pci_clear_master(psPVRPCI->psPCIDev);
	}

	pci_disable_device(psPVRPCI->psPCIDev);

	OSFreeMem(psPVRPCI);
	/*not nulling pointer, copy on stack*/

	return PVRSRV_OK;
}

/*************************************************************************/ /*!
@Function       OSPCISuspendDev
@Description    Prepare PCI device to be turned off by power management
@Input          hPVRPCI                 PCI device handle
@Return	        PVRSRV_ERROR	        Services error code
*/ /**************************************************************************/
PVRSRV_ERROR OSPCISuspendDev(PVRSRV_PCI_DEV_HANDLE hPVRPCI)
{
	PVR_PCI_DEV *psPVRPCI = (PVR_PCI_DEV *)hPVRPCI;
	int i;
	int err;

	/* Release all PCI regions that are currently in use */
	for (i = 0; i < DEVICE_COUNT_RESOURCE; i++)
	{
		if (psPVRPCI->abPCIResourceInUse[i])
		{
			pci_release_region(psPVRPCI->psPCIDev, i);
		}
	}

	err = pci_save_state(psPVRPCI->psPCIDev);
	if (err != 0)
	{
		printk(KERN_ERR "OSPCISuspendDev: pci_save_state_failed (%d)", err);
		return PVRSRV_ERROR_PCI_CALL_FAILED;
	}

	pci_disable_device(psPVRPCI->psPCIDev);

	err = pci_set_power_state(psPVRPCI->psPCIDev, pci_choose_state(psPVRPCI->psPCIDev, PMSG_SUSPEND));
	switch (err)
	{
		case 0:
			break;
		case -EIO:
			printk(KERN_ERR "OSPCISuspendDev: device doesn't support PCI PM");
			break;
		case -EINVAL:
			printk(KERN_ERR "OSPCISuspendDev: can't enter requested power state");
			break;
		default:
			printk(KERN_ERR "OSPCISuspendDev: pci_set_power_state failed (%d)", err);
			break;
	}

	return PVRSRV_OK;
}

/*************************************************************************/ /*!
@Function       OSPCIResumeDev
@Description    Prepare a PCI device to be resumed by power management
@Input          hPVRPCI                 PCI device handle
@Return	        PVRSRV_ERROR	        Services error code
*/ /**************************************************************************/
PVRSRV_ERROR OSPCIResumeDev(PVRSRV_PCI_DEV_HANDLE hPVRPCI)
{
	PVR_PCI_DEV *psPVRPCI = (PVR_PCI_DEV *)hPVRPCI;
	int err;
	int i;

	err = pci_set_power_state(psPVRPCI->psPCIDev, pci_choose_state(psPVRPCI->psPCIDev, PMSG_ON));
	switch (err)
	{
		case 0:
			break;
		case -EIO:
			printk(KERN_ERR "OSPCIResumeDev: device doesn't support PCI PM");
			break;
		case -EINVAL:
			printk(KERN_ERR "OSPCIResumeDev: can't enter requested power state");
			return PVRSRV_ERROR_UNKNOWN_POWER_STATE;
		default:
			printk(KERN_ERR "OSPCIResumeDev: pci_set_power_state failed (%d)", err);
			return PVRSRV_ERROR_UNKNOWN_POWER_STATE;
	}

	pci_restore_state(psPVRPCI->psPCIDev);

	err = pci_enable_device(psPVRPCI->psPCIDev);
	if (err != 0)
	{
		printk(KERN_ERR "OSPCIResumeDev: Couldn't enable device (%d)", err);
		return PVRSRV_ERROR_PCI_CALL_FAILED;
	}

	if (psPVRPCI->ePCIFlags & HOST_PCI_INIT_FLAG_BUS_MASTER)	/* PRQA S 3358 */ /* misuse of enums */
		pci_set_master(psPVRPCI->psPCIDev);

	/* Restore the PCI resource tracking array */
	for (i = 0; i < DEVICE_COUNT_RESOURCE; i++)
	{
		if (psPVRPCI->abPCIResourceInUse[i])
		{
			err = pci_request_region(psPVRPCI->psPCIDev, i, PVRSRV_MODNAME);
			if (err != 0)
			{
				printk(KERN_ERR "OSPCIResumeDev: pci_request_region_failed (region %d, error %d)", i, err);
			}
		}
	}

	return PVRSRV_OK;
}

/*************************************************************************/ /*!
@Function       OSPCIGetVendorDeviceIDs
@Description    Retrieve PCI vendor ID and device ID.
@Input          hPVRPCI                 PCI device handle
@Output         pui16VendorID           Vendor ID
@Output         pui16DeviceID           Device ID
@Return         PVRSRV_ERROR            Services error code
*/ /**************************************************************************/
PVRSRV_ERROR OSPCIGetVendorDeviceIDs(PVRSRV_PCI_DEV_HANDLE hPVRPCI,
                                     IMG_UINT16 *pui16VendorID,
                                     IMG_UINT16 *pui16DeviceID)
{
	PVR_PCI_DEV *psPVRPCI = (PVR_PCI_DEV *)hPVRPCI;
	struct pci_dev *psPCIDev;

	if (psPVRPCI == NULL)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	psPCIDev = psPVRPCI->psPCIDev;
	if (psPCIDev == NULL)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	*pui16VendorID = psPCIDev->vendor;
	*pui16DeviceID = psPCIDev->device;

	return PVRSRV_OK;
}

#if defined(CONFIG_MTRR)

/*************************************************************************/ /*!
@Function       OSPCIClearResourceMTRRs
@Description    Clear any BIOS-configured MTRRs for a PCI memory region
@Input          hPVRPCI                 PCI device handle
@Input          ui32Index               Address range index
@Return	        PVRSRV_ERROR	        Services error code
*/ /**************************************************************************/
PVRSRV_ERROR OSPCIClearResourceMTRRs(PVRSRV_PCI_DEV_HANDLE hPVRPCI, IMG_UINT32 ui32Index)
{
	PVR_PCI_DEV *psPVRPCI = (PVR_PCI_DEV *)hPVRPCI;
	resource_size_t start, end;
	int res;

	start = pci_resource_start(psPVRPCI->psPCIDev, ui32Index);
	end = pci_resource_end(psPVRPCI->psPCIDev, ui32Index) + 1;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0))
	res = arch_io_reserve_memtype_wc(start, end - start);
	if (res)
	{
		return PVRSRV_ERROR_PCI_CALL_FAILED;
	}
#endif
	res = arch_phys_wc_add(start, end - start);
	if (res < 0)
	{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0))
		arch_io_free_memtype_wc(start, end - start);
#endif

		return PVRSRV_ERROR_PCI_CALL_FAILED;
	}
	psPVRPCI->iMTRR[ui32Index] = res;

	return PVRSRV_OK;
}

/*************************************************************************/ /*!
@Function       OSPCIReleaseResourceMTRRs
@Description    Release resources allocated by OSPCIClearResourceMTRRs
@Input          hPVRPCI                 PCI device handle
@Input          ui32Index               Address range index
*/ /**************************************************************************/
void OSPCIReleaseResourceMTRRs(PVRSRV_PCI_DEV_HANDLE hPVRPCI, IMG_UINT32 ui32Index)
{
	PVR_PCI_DEV *psPVRPCI = (PVR_PCI_DEV *)hPVRPCI;

	if (psPVRPCI->iMTRR[ui32Index] >= 0)
	{
		arch_phys_wc_del(psPVRPCI->iMTRR[ui32Index]);
		psPVRPCI->iMTRR[ui32Index] = -1;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0))
		{
			resource_size_t start, end;

			start = pci_resource_start(psPVRPCI->psPCIDev, ui32Index);
			end = pci_resource_end(psPVRPCI->psPCIDev, ui32Index) + 1;

			arch_io_free_memtype_wc(start, end - start);
		}
#endif
	}
}
#endif /* defined(CONFIG_MTRR) */
