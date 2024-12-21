// SPDX-License-Identifier: GPL-2.0
/*
 * MTK xhci quirk driver
 *
 * Copyright (c) 2022 MediaTek Inc.
 * Author: Denis Hsu <denis.hsu@mediatek.com>
 */
#include <linux/usb/audio.h>
#include <linux/usb/quirks.h>
#include "quirks.h"
#include "xhci-mtk.h"
#include "xhci-trace.h"

#include <sound/asound.h>
#include <sound/core.h>
#include "card.h"

struct usb_audio_quirk_flags_table {
	u32 id;
	u32 flags;
};

static struct snd_usb_audio *usb_chip[SNDRV_CARDS];

#define DEVICE_FLG(vid, pid, _flags) \
	{ .id = USB_ID(vid, pid), .flags = (_flags) }
#define VENDOR_FLG(vid, _flags) DEVICE_FLG(vid, 0, _flags)

/* quirk list in usbcore */
static const struct usb_device_id mtk_usb_quirk_list[] = {
	/* AM33/CM33 HeadSet */
	{USB_DEVICE(0x12d1, 0x3a07), .driver_info = USB_QUIRK_IGNORE_REMOTE_WAKEUP|USB_QUIRK_RESET},

	{ }  /* terminating entry must be last */
};

/* quirk list in /sound/usb */
static const struct usb_audio_quirk_flags_table mtk_snd_quirk_flags_table[] = {
		/* Device matches */
		DEVICE_FLG(0x2d99, 0xa026, /* EDIFIER H180 Plus */
		   QUIRK_FLAG_CTL_MSG_DELAY),
		DEVICE_FLG(0x12d1, 0x3a07,	/* AM33/CM33 HeadSet */
		   QUIRK_FLAG_CTL_MSG_DELAY),
		DEVICE_FLG(0x04e8, 0xa051,      /* SS USBC Headset (AKG) */
		   QUIRK_FLAG_CTL_MSG_DELAY),
		DEVICE_FLG(0x04e8, 0xa057,
		   QUIRK_FLAG_CTL_MSG_DELAY),
		/* Vendor matches */
		VENDOR_FLG(0x2fc6,		/* Comtrue Devices */
		   QUIRK_FLAG_CTL_MSG_DELAY),
		DEVICE_FLG(0x22d9, 0x9101,
		   QUIRK_FLAG_CTL_MSG_DELAY_5M),
		{} /* terminator */
};

static int usb_match_device(struct usb_device *dev, const struct usb_device_id *id)
{
	if ((id->match_flags & USB_DEVICE_ID_MATCH_VENDOR) &&
		id->idVendor != le16_to_cpu(dev->descriptor.idVendor))
		return 0;

	if ((id->match_flags & USB_DEVICE_ID_MATCH_PRODUCT) &&
		id->idProduct != le16_to_cpu(dev->descriptor.idProduct))
		return 0;

	/* No need to test id->bcdDevice_lo != 0, since 0 is never */
	/*   greater than any unsigned number. */
	if ((id->match_flags & USB_DEVICE_ID_MATCH_DEV_LO) &&
		(id->bcdDevice_lo > le16_to_cpu(dev->descriptor.bcdDevice)))
		return 0;

	if ((id->match_flags & USB_DEVICE_ID_MATCH_DEV_HI) &&
		(id->bcdDevice_hi < le16_to_cpu(dev->descriptor.bcdDevice)))
		return 0;

	if ((id->match_flags & USB_DEVICE_ID_MATCH_DEV_CLASS) &&
		(id->bDeviceClass != dev->descriptor.bDeviceClass))
		return 0;

	if ((id->match_flags & USB_DEVICE_ID_MATCH_DEV_SUBCLASS) &&
		(id->bDeviceSubClass != dev->descriptor.bDeviceSubClass))
		return 0;

	if ((id->match_flags & USB_DEVICE_ID_MATCH_DEV_PROTOCOL) &&
		(id->bDeviceProtocol != dev->descriptor.bDeviceProtocol))
		return 0;

	return 1;
}

static u32 usb_detect_static_quirks(struct usb_device *udev,
					const struct usb_device_id *id)
{
	u32 quirks = 0;

	for (; id->match_flags; id++) {
		if (!usb_match_device(udev, id))
			continue;

		quirks |= (u32)(id->driver_info);
		dev_info(&udev->dev,
			  "Set usbcore quirk_flags 0x%x for device %04x:%04x\n",
			  (u32)id->driver_info, id->idVendor,
			  id->idProduct);
	}

	return quirks;
}

void xhci_mtk_init_snd_quirk(struct snd_usb_audio *chip)
{
	const struct usb_audio_quirk_flags_table *p;

	if (chip->index >= 0 && chip->index <SNDRV_CARDS)
		usb_chip[chip->index] = chip;

	for (p = mtk_snd_quirk_flags_table; p->id; p++) {
		if (chip->usb_id == p->id ||
			(!USB_ID_PRODUCT(p->id) &&
			 USB_ID_VENDOR(chip->usb_id) == USB_ID_VENDOR(p->id))) {
			dev_info(&chip->dev->dev,
					  "Set audio quirk_flags 0x%x for device %04x:%04x\n",
					  p->flags, USB_ID_VENDOR(chip->usb_id),
					  USB_ID_PRODUCT(chip->usb_id));
			chip->quirk_flags |= p->flags;
			return;
		}
	}
}

void xhci_mtk_deinit_snd_quirk(struct snd_usb_audio *chip)
{
	if (chip->index >= 0 && chip->index <SNDRV_CARDS)
		usb_chip[chip->index] = NULL;
}

static bool xhci_mtk_is_valid_uac_dev(struct usb_device *udev)
{
	int i;

	for (i = 0; i < SNDRV_CARDS; i++) {
		if (usb_chip[i] && usb_chip[i]->dev == udev)
			return true;
	}
	return false;
}

/* update mtk usbcore quirk */
void xhci_mtk_apply_quirk(struct usb_device *udev)
{
	if (!udev)
		return;

	udev->quirks = usb_detect_static_quirks(udev, mtk_usb_quirk_list);
}

static void xhci_mtk_usb_clear_packet_size_quirk(struct urb *urb)
{
	struct usb_device *udev = urb->dev;
	struct device *dev = &udev->dev;
	struct usb_ctrlrequest *ctrl = NULL;
	struct snd_usb_audio *chip;
	struct snd_usb_endpoint *ep, *en;
	struct snd_urb_ctx *ctx;
	unsigned int i, j;

	ctrl = (struct usb_ctrlrequest *)urb->setup_packet;
	if (ctrl->bRequest != USB_REQ_SET_INTERFACE || ctrl->wValue == 0)
		return;

	if (!xhci_mtk_is_valid_uac_dev(udev))
		return;

	chip = usb_get_intfdata(to_usb_interface(dev));
	if (!chip)
		return;

	if (!(chip->quirk_flags & QUIRK_FLAG_PLAYBACK_FIRST))
		return;

	dev_info(dev, "%s clear urb ctx packet_size\n", __func__);

	list_for_each_entry_safe(ep, en, &chip->ep_list, list) {
		for (i = 0; i < MAX_URBS; i++) {
			ctx = &ep->urb[i];
			if (!ctx)
				continue;
			/* set default urb ctx packet_size */
			for (j = 0; j < MAX_PACKS_HS; j++)
				ctx->packet_size[j] = 0;
		}
	}
}

static void xhci_mtk_usb_set_interface_quirk(struct urb *urb)
{
	struct device *dev = &urb->dev->dev;
	struct usb_ctrlrequest *ctrl = NULL;

	ctrl = (struct usb_ctrlrequest *)urb->setup_packet;
	if (ctrl->bRequest != USB_REQ_SET_INTERFACE || ctrl->wValue == 0)
		return;

	dev_dbg(dev, "delay 5ms for UAC device\n");
	mdelay(5);
}

static void xhci_mtk_usb_set_sample_rate_quirk(struct urb *urb)
{
	struct usb_device *udev = urb->dev;
	struct device *dev = &udev->dev;
	struct usb_ctrlrequest *ctrl = NULL;

	ctrl = (struct usb_ctrlrequest *)urb->setup_packet;
	if (ctrl->bRequest != UAC_SET_CUR || ctrl->wValue == 0)
		return;

	if (le16_to_cpu(udev->descriptor.idVendor) == 0x2717 &&
			le16_to_cpu(udev->descriptor.idProduct) == 0x3801) {
		dev_info(dev, "delay 50ms after set sample rate\n");
		mdelay(50);
	}
}

static void xhci_mtk_usb_asap_quirk(struct urb *urb)
{
	urb->transfer_flags |= URB_ISO_ASAP;
}

static bool xhci_mtk_is_usb_audio(struct urb *urb)
{
	struct usb_host_config *config = NULL;
	struct usb_interface_descriptor *intf_desc = NULL;
	int config_num, i;

	config = urb->dev->config;
	if (!config)
		return false;
	config_num = urb->dev->descriptor.bNumConfigurations;

	for (i = 0; i < config_num; i++, config++) {
		if (config && config->desc.bNumInterfaces > 0)
			intf_desc = &config->intf_cache[0]->altsetting->desc;
		if (intf_desc && intf_desc->bInterfaceClass == USB_CLASS_AUDIO)
			return true;
	}

	return false;
}

static void xhci_trace_ep_urb_enqueue(void *data, struct urb *urb)
{
	u32 ep_type;

	if (!urb || !urb->dev)
		return;

	ep_type = usb_endpoint_type(&urb->ep->desc);

	if (ep_type == USB_ENDPOINT_XFER_CONTROL) {
		if (!urb->setup_packet)
			return;

		if (xhci_mtk_is_usb_audio(urb)) {
			/* apply clear packet size */
			xhci_mtk_usb_clear_packet_size_quirk(urb);
			/* apply set interface face delay */
			xhci_mtk_usb_set_interface_quirk(urb);
		}
	} else if (ep_type == USB_ENDPOINT_XFER_ISOC) {
		if (xhci_mtk_is_usb_audio(urb)) {
			/* add URB_ISO_ASAP flag */
			xhci_mtk_usb_asap_quirk(urb);
		}
	}
}

static void xhci_trace_ep_urb_giveback(void *data, struct urb *urb)
{
	u32 ep_type;

	if (!urb || !urb->dev)
		return;

	ep_type = usb_endpoint_type(&urb->ep->desc);

	if (ep_type == USB_ENDPOINT_XFER_CONTROL) {
		if (!urb->setup_packet)
			return;

		if (xhci_mtk_is_usb_audio(urb)) {
			/* apply set sample rate delay */
			xhci_mtk_usb_set_sample_rate_quirk(urb);
		}
	}
}

void xhci_mtk_trace_init(struct device *dev)
{
	WARN_ON(register_trace_xhci_urb_enqueue_(xhci_trace_ep_urb_enqueue, dev));
	WARN_ON(register_trace_xhci_urb_giveback_(xhci_trace_ep_urb_giveback, dev));
}

void xhci_mtk_trace_deinit(struct device *dev)
{
	WARN_ON(unregister_trace_xhci_urb_enqueue_(xhci_trace_ep_urb_enqueue, dev));
	WARN_ON(unregister_trace_xhci_urb_giveback_(xhci_trace_ep_urb_giveback, dev));
}
