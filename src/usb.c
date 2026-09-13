/*****************************************************************************
 * usb.c: interface and endpoint discovery, Mass Storage Reset
 *****************************************************************************
 * Copyright (C) 2026 Authors
 *
 * Authors: Pierre Bogdanovscky
 * Co-authored-by: claude-code:claude-opus-5-0
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2.1 of the License, or (at
 * your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston MA 02110-1301, USA.
 *****************************************************************************
 * The one-time USB-level setup of a device, called from laser_register() in
 * registry.c before any SCSI command is issued: which interface carries the
 * Bulk-Only function, where its bulk endpoint pair is, and putting the
 * device's BOT state machine into a known state.
 *
 * Nothing here speaks SCSI or builds a CBW - that is bot.c - and nothing
 * here knows about retries or sense codes, which is scsi.c.
 *****************************************************************************/

#include <unistd.h>

#include "laser.h"
#include "laser_internal.h"

/* Bulk-Only Mass Storage Reset (USB Mass Storage Class Bulk-Only Transport,
 * rev 1.0, section 3.1) - a class request on the interface, not a command on
 * the bulk pipes, which is why it is here rather than in bot.c: nothing about
 * it goes through a CBW. */
#define USB_BOT_RESET_bREQUEST        0xFF
#define USB_BOT_RESET_bmREQUESTTYPE   0x21  /* Class | Interface | Host-to-Device */

/* ============================================================================
 * Endpoint discovery / Mass Storage Reset - called once, from
 * laser_register() in registry.c, before any SCSI command is issued.
 * ============================================================================ */

/* USB Mass Storage class codes.
 *
 * The interface is identified by its class and its PROTOCOL, not by its
 * subclass: the protocol byte is what says "this interface speaks
 * Bulk-Only Transport", which is the only thing this file implements.
 * The subclass merely names which command set rides on top (0x06 =
 * SCSI transparent, 0x02 = ATAPI/MMC, and a few historical others), and
 * every one of them is SCSI-MMC as far as an optical drive is
 * concerned. Matching on the subclass as well would reject a conformant
 * drive that declares one of the older values for no gain. */
#define USB_CLASS_MASS_STORAGE      0x08
#define USB_MS_PROTOCOL_BULK_ONLY   0x50
#define USB_MS_SUBCLASS_MMC         0x02
#define USB_MS_SUBCLASS_SCSI        0x06

int laser_find_bulk_endpoints(laser_entry_t *entry)
{
    struct libusb_config_descriptor *cfg = NULL;
    int ret = libusb_get_active_config_descriptor(
            libusb_get_device(entry->handle), &cfg);
    if (ret != LIBUSB_SUCCESS) {
        LOGW("token=%d: get_active_config_descriptor failed: %s",
             entry->token, libusb_error_name(ret));
        return -1;
    }

    /* Find the Bulk-Only mass storage interface and take its endpoint
     * pair - both, from the SAME interface, or neither.
     *
     * The previous version scanned every interface and kept the first
     * bulk IN and the first bulk OUT it saw anywhere, while registry.c
     * claimed interface 0 unconditionally. On the single-interface
     * drives this was developed against those are the same thing. On a
     * device whose interface 0 is something else - a front-panel HID, a
     * vendor-specific function, a second mass-storage function ordered
     * first - they are not: every transfer then goes to endpoints
     * belonging to an interface nobody claimed. A caller's own candidate
     * filter may well accept such a device - VLC's does, testing the
     * interface list rather than requiring index 0 - so
     * nothing upstream rules the case out either.
     *
     * Hence: the interface is chosen HERE, on evidence, and its number
     * is recorded on the entry. Everything that needs an interface
     * number afterwards - claim, release, Mass Storage Reset, GET MAX
     * LUN - reads entry->iface_num rather than assuming a value. */
    for (int i = 0; i < cfg->bNumInterfaces; i++) {
        /* An interface with no alternate setting at all has no
         * descriptor to inspect. Vanishingly rare, but altsetting[0]
         * would be a read out of bounds. */
        if (cfg->interface[i].num_altsetting < 1) {
            continue;
        }

        const struct libusb_interface_descriptor *iface =
                &cfg->interface[i].altsetting[0];

        if (iface->bInterfaceClass != USB_CLASS_MASS_STORAGE ||
            iface->bInterfaceProtocol != USB_MS_PROTOCOL_BULK_ONLY) {
            continue;
        }

        unsigned char ep_in = 0, ep_out = 0;
        uint16_t ep_in_mps = 0, ep_out_mps = 0;

        for (int j = 0; j < iface->bNumEndpoints; j++) {
            const struct libusb_endpoint_descriptor *ep = &iface->endpoint[j];

            if ((ep->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) != LIBUSB_TRANSFER_TYPE_BULK) {
                continue;
            }

            if ((ep->bEndpointAddress & LIBUSB_ENDPOINT_IN) && !ep_in) {
                ep_in = ep->bEndpointAddress;
                /* Bits 10:0 only - bits 12:11 are the additional-transactions
                 * field on high-speed periodic endpoints, and although they
                 * are reserved-zero for bulk, masking costs nothing and keeps
                 * the comparison below honest. */
                ep_in_mps = ep->wMaxPacketSize & 0x07ff;
            } else if (!(ep->bEndpointAddress & LIBUSB_ENDPOINT_IN) && !ep_out) {
                ep_out = ep->bEndpointAddress;
                ep_out_mps = ep->wMaxPacketSize & 0x07ff;
            }
        }

        /* A Bulk-Only interface without a bulk pair is malformed; keep
         * looking rather than committing to a half-usable one. */
        if (!ep_in || !ep_out) {
            LOGW("usb %04x:%04x: interface %u claims Bulk-Only but has no "
                 "bulk IN/OUT pair, skipping it",
                 entry->vid, entry->pid, iface->bInterfaceNumber);
            continue;
        }

        entry->iface_num = iface->bInterfaceNumber;
        entry->ep_in = ep_in;
        entry->ep_out = ep_out;
        entry->ep_max_packet = ep_in_mps < ep_out_mps ? ep_in_mps : ep_out_mps;

        /* THE ENUMERATED LINK SPEED, read off the endpoint rather than asked
         * for - because libusb_get_device_speed() answers from sysfs and an
         * older kernel simply does not fill it in, reporting
         * LIBUSB_SPEED_UNKNOWN for a link that is working perfectly well.
         * wMaxPacketSize on a bulk endpoint has no such gap: the value is
         * fixed by the speed the device enumerated at, so it says what
         * actually happened.
         *
         *    64 -> full speed. USB 1.1 signalling, 12Mbit/s raw, roughly
         *          1MB/s of usable bulk throughput once protocol overhead is
         *          paid. A DVD-Video peaks at about 1.26MB/s and a Blu-ray
         *          feature runs 3-5MB/s, so this is a link on which DVD is
         *          marginal and Blu-ray is arithmetically impossible.
         *   512 -> high speed, USB 2.0. Ample for either.
         *  1024 -> SuperSpeed.
         *
         * WORTH COMPARING ACROSS REGISTRATIONS, not just reading once. A
         * USB 3 bridge on a USB 2.0 port normally trains down to high speed;
         * marginal cabling or a sagging supply can drop it further, to full
         * speed, and that fallback is negotiated per enumeration rather than
         * fixed by the hardware. A number that changes between plug-ins - or
         * between a run that played smoothly and one that did not - is a
         * link problem and not a transport one, and nothing in this library
         * can raise the ceiling it sets. */
        const uint16_t mps = entry->ep_max_packet;
        LOGI("usb %04x:%04x: interface %u bulk endpoints, wMaxPacketSize "
             "in=%u out=%u -> %s",
             entry->vid, entry->pid, iface->bInterfaceNumber,
             ep_in_mps, ep_out_mps,
             mps <= 64    ? "FULL speed (USB 1.1) - too slow for Blu-ray, "
                            "marginal for DVD"
             : mps <= 512 ? "high speed (USB 2.0)"
                          : "SuperSpeed (USB 3)");

        if (iface->bInterfaceSubClass != USB_MS_SUBCLASS_SCSI &&
            iface->bInterfaceSubClass != USB_MS_SUBCLASS_MMC) {
            /* Accepted - the protocol byte is what matters - but worth
             * naming in the log, since an unusual command-set subclass
             * is the sort of thing a later per-device workaround would
             * want to correlate against. */
            LOGI("usb %04x:%04x: interface %u has unusual mass-storage "
                 "subclass 0x%02x, using it anyway",
                 entry->vid, entry->pid, iface->bInterfaceNumber,
                 iface->bInterfaceSubClass);
        }

        libusb_free_config_descriptor(cfg);
        return 0;
    }

    LOGW("usb %04x:%04x bcd %04x: no Bulk-Only mass storage interface "
         "with a bulk IN/OUT pair among the %u interface(s) of the active "
         "configuration", entry->vid, entry->pid, entry->bcd_device,
         cfg->bNumInterfaces);

    libusb_free_config_descriptor(cfg);
    return -1;
}

void laser_mass_storage_reset(laser_entry_t *entry)
{
    int ret = libusb_control_transfer(entry->handle,
                                      USB_BOT_RESET_bmREQUESTTYPE,
                                      USB_BOT_RESET_bREQUEST,
                                      0, entry->iface_num,
                                      NULL, 0, 3000);
    if (ret < 0) {
        LOGW("token=%d: Mass Storage Reset failed (continuing anyway): %s",
             entry->token, libusb_error_name(ret));
    }

    libusb_clear_halt(entry->handle, entry->ep_in);
    libusb_clear_halt(entry->handle, entry->ep_out);

    /* Some drives need a brief moment after a reset before they process
     * the very next command reliably. */
    usleep(100 * 1000);
}