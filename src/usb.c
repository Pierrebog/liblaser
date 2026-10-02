/*****************************************************************************
 * usb.c: interface and endpoint discovery, Mass Storage Reset
 *****************************************************************************
 * Copyright (C) 2026 Authors
 *
 * Authors: Pierre Bogdanovscky
 * Co-authored-by: claude-code:claude-opus-5-5
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
 * USB-level work below the SCSI layer: finding the interface that carries
 * the Bulk-Only function and its bulk endpoint pair, once at registration,
 * and the Mass Storage Reset that puts the device's BOT state machine in a
 * known state - at registration, in Reset Recovery and at teardown.
 *
 * No CBW here (bot.c), no retries or sense codes (scsi.c).
 *****************************************************************************/

#include <unistd.h>

#include "laser.h"
#include "laser_internal.h"

/* Bulk-Only Mass Storage Reset (USB Mass Storage Class Bulk-Only Transport,
 * rev 1.0, section 3.1): a class request on the interface, not a command on
 * the bulk pipes - hence here rather than in bot.c. */
#define USB_BOT_RESET_bREQUEST        0xFF
#define USB_BOT_RESET_bmREQUESTTYPE   0x21  /* Class | Interface | Host-to-Device */

/* ============================================================================
 * Endpoint discovery, called once from laser_register() in registry.c,
 * before the interface is claimed.
 * ============================================================================ */

/* USB Mass Storage class codes.
 *
 * The interface is matched on class and PROTOCOL: the protocol says "Bulk-Only
 * Transport", the only thing implemented here. The subclass only names the
 * command set on top (0x06 SCSI transparent, 0x02 ATAPI/MMC, a few historical
 * others), all of which are SCSI-MMC to an optical drive; matching on it would
 * reject conformant drives for no gain. */
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

    /* The first Bulk-Only mass storage interface, and its endpoint pair -
     * both from that SAME interface, or neither. Interface 0 is not assumed:
     * a front-panel HID, a vendor function or a second mass-storage function
     * may come first, and endpoints taken from one interface while another
     * is claimed go to an interface nobody owns. Everything that needs an
     * interface number later - claim, release, Mass Storage Reset, GET MAX
     * LUN - reads entry->iface_num. */
    for (int i = 0; i < cfg->bNumInterfaces; i++) {
        /* No alternate setting means no descriptor to inspect, and
         * altsetting[0] would be out of bounds. */
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
                /* Bits 10:0: bits 12:11 are the additional-transactions field
                 * of high-speed periodic endpoints, reserved-zero for bulk. */
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

        /* The link speed the device enumerated at, read off the endpoint:
         * libusb_get_device_speed() answers from sysfs, which some kernels do
         * not fill in.
         *
         *    64 -> full speed, USB 1.1: about 1 MB/s of usable bulk
         *          throughput. A DVD-Video peaks near 1.26 MB/s and a Blu-ray
         *          runs 3-5 MB/s: marginal for one, impossible for the other.
         *   512 -> high speed, USB 2.0: ample for either.
         *  1024 -> SuperSpeed.
         *
         * Worth comparing across plug-ins: marginal cabling or a sagging
         * supply can make a drive enumerate at full speed one time and high
         * speed the next, a link problem nothing here can fix. */
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
            /* Accepted, but logged: a per-device workaround may one day want
             * to correlate against an unusual subclass. */
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

/* ============================================================================
 * Mass Storage Reset - see laser_internal.h.
 * ============================================================================ */

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

    /* Some drives need a moment after a reset before they process the next
     * command reliably. */
    usleep(100 * 1000);
}
