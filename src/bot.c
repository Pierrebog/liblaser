/*****************************************************************************
 * bot.c: one USB Mass Storage Bulk-Only transaction
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
 * CBW framing, the data phase in either direction, the CSW and its
 * validation, stall recovery and Reset Recovery - USB Mass Storage Class
 * Bulk-Only Transport rev 1.0 ("BBB" below), and nothing above it.
 *
 * No retry policy, no sense codes, no idea what a CDB means: this performs
 * one transaction and reports, through its return value, enough for scsi.c
 * to decide whether it may be replayed.
 *****************************************************************************/

#include <string.h>
#include <unistd.h>

#include "laser.h"
#include "laser_internal.h"

/* ============================================================================
 * BOT protocol constants (USB Mass Storage Class Bulk-Only Transport, rev 1.0)
 * ============================================================================ */

#define USB_BOT_CBW_SIGNATURE   0x43425355u  /* "USBC" */
#define USB_BOT_CSW_SIGNATURE   0x53425355u  /* "USBS" */
#define USB_BOT_CBW_SIZE        31
#define USB_BOT_CSW_SIZE        13

/* The CBW and CSW are serialized by struct layout, their multi-byte fields
 * assigned directly. BBB defines those fields as little-endian, which every
 * Android ABI is - so the shortcut stays, and a host where it would be wrong
 * fails to build instead of sending garbage. A toolchain that cannot say
 * fails too: add byte swapping, or define the macros for a target known to
 * be little-endian. */
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__)
# if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#  error "laser: CBW/CSW are serialized by struct layout, which assumes a little-endian host"
# endif
#else
# error "laser: cannot determine host byte order; see the comment above"
#endif

#pragma pack(push, 1)
typedef struct {
    uint32_t dCBWSignature;
    uint32_t dCBWTag;
    uint32_t dCBWDataTransferLength;
    uint8_t  bmCBWFlags;
    uint8_t  bCBWLUN;
    uint8_t  bCBWCBLength;
    uint8_t  CBWCB[16];
} bot_cbw_t;

typedef struct {
    uint32_t dCSWSignature;
    uint32_t dCSWTag;
    uint32_t dCSWDataResidue;
    uint8_t  bCSWStatus;
} bot_csw_t;
#pragma pack(pop)

/* A packing directive that failed to apply would not fail to build: it would
 * send CBWs the drive rejects, a long way from the cause. */
_Static_assert(sizeof(bot_cbw_t) == USB_BOT_CBW_SIZE,
               "bot_cbw_t is not 31 bytes: #pragma pack did not apply");
_Static_assert(sizeof(bot_csw_t) == USB_BOT_CSW_SIZE,
               "bot_csw_t is not 13 bytes: #pragma pack did not apply");

/* Per-phase USB timeouts, in milliseconds. The data phase gets the longest:
 * it alone depends on the disc - a damaged sector is retried inside the drive
 * before it answers - while the CBW and CSW are a few bytes of protocol that
 * waiting longer will not bring. */
#define CBW_PHASE_TIMEOUT_MS    3000
#define DATA_PHASE_TIMEOUT_MS   5000
#define CSW_PHASE_TIMEOUT_MS    3000

/* Timeouts for PROBES - commands a working device answers at once, with or
 * without a medium (INQUIRY). Under the read timeouts, a device answering
 * nothing cost seconds per attempt to establish so, on every browse.
 *
 * Used while entry->probe_timeouts is set, under io_lock. */
#define PROBE_CBW_PHASE_TIMEOUT_MS    500
#define PROBE_DATA_PHASE_TIMEOUT_MS   700
#define PROBE_CSW_PHASE_TIMEOUT_MS    500

static void bot_clear_stall(laser_entry_t *entry, unsigned char endpoint)
{
    int ret = libusb_clear_halt(entry->handle, endpoint);
    if (ret != LIBUSB_SUCCESS) {
        LOGW("token=%d: clear_halt(0x%02x) failed: %s",
             entry->token, endpoint, libusb_error_name(ret));
    }
}

/* ============================================================================
 * One BOT transaction: CBW, data phase (either direction), CSW.
 *
 * The contract - what each return code means - is with the declaration in
 * laser_internal.h. What follows is why each wire condition maps to the code
 * it does.
 * ============================================================================ */

int laser_bot_send_locked(laser_entry_t *entry,
                          const uint8_t *cdb, int cdb_len,
                          uint8_t *data, int data_len, int data_in,
                          int *actual_len, int *csw_status)
{
    bot_cbw_t cbw;
    bot_csw_t csw;
    unsigned char csw_buf[USB_BOT_CSW_SIZE];
    int transferred = 0;
    /* Bytes the data phase moved, kept apart from `transferred`, which the
     * CSW read reuses, to bound the residue afterwards. */
    int data_transferred = 0;
    int ret;

    if (csw_status) {
        *csw_status = -1;
    }

    /* laser_scsi_cdb() already rejects this, but the memcpy below is the
     * overflow site, of a stack struct, and one comparison makes it
     * impossible whatever a caller does. */
    if (cdb_len < 0 || cdb_len > (int)sizeof(cbw.CBWCB)) {
        LOGE("token=%d: refusing a %d-byte CDB (max %zu)",
             entry->token, cdb_len, sizeof(cbw.CBWCB));
        return BOT_FAIL_NOT_SENT;
    }

    memset(&cbw, 0, sizeof(cbw));
    cbw.dCBWSignature = USB_BOT_CBW_SIGNATURE;
    cbw.dCBWTag = ++entry->tag;
    cbw.dCBWDataTransferLength = (uint32_t)data_len;
    cbw.bmCBWFlags = data_in ? 0x80 : 0x00;
    cbw.bCBWLUN = entry->lun;
    cbw.bCBWCBLength = (uint8_t)cdb_len;
    memcpy(cbw.CBWCB, cdb, (size_t)cdb_len);

    ret = libusb_bulk_transfer(entry->handle, entry->ep_out,
                               (unsigned char *)&cbw, USB_BOT_CBW_SIZE,
                               &transferred,
                               entry->probe_timeouts ? PROBE_CBW_PHASE_TIMEOUT_MS
                                                     : CBW_PHASE_TIMEOUT_MS);
    if (ret == LIBUSB_ERROR_NO_DEVICE) {
        LOGW("token=%d: device gone while sending the CBW", entry->token);
        return BOT_FAIL_NO_DEVICE;
    }
    if (transferred == USB_BOT_CBW_SIZE &&
        (ret == LIBUSB_SUCCESS || ret == LIBUSB_ERROR_TIMEOUT)) {
        /* The whole CBW went out, even if the clock ran out on the way: the
         * drive has the command, and treating it as unsent would let scsi.c
         * replay a command that may have been executed. */
        if (ret != LIBUSB_SUCCESS) {
            LOGW("token=%d: CBW phase timed out after the whole CBW went out, "
                 "carrying on", entry->token);
        }
    } else if (transferred == 0 && ret != LIBUSB_ERROR_PIPE) {
        /* Nothing went out: the drive never saw the command, and its state
         * is untouched. The only failure after which replaying a command
         * that changes CSS state is provably harmless. */
        LOGW("token=%d: CBW send failed: %s", entry->token,
             libusb_error_name(ret));
        return BOT_FAIL_NOT_SENT;
    } else {
        /* Part of a CBW, or a stalled Bulk-Out: the device holds an invalid
         * CBW, which it will not execute, and BBB 6.6.1 requires a Reset
         * Recovery before it accepts the next one. Returning without it left
         * the device waiting for the rest of the 31 bytes, to take the start
         * of the next command for them. */
        LOG_QUIRK(entry, "CBW not accepted (%s, %d/%d bytes): performing "
                         "Reset Recovery",
                  libusb_error_name(ret), transferred, USB_BOT_CBW_SIZE);
        laser_mass_storage_reset(entry);
        return BOT_FAIL_NOT_SENT;
    }

    if (data_len > 0 && data) {
        unsigned char ep = data_in ? entry->ep_in : entry->ep_out;

        ret = libusb_bulk_transfer(entry->handle, ep, data, data_len,
                                   &transferred,
                                   entry->probe_timeouts ? PROBE_DATA_PHASE_TIMEOUT_MS
                                                         : DATA_PHASE_TIMEOUT_MS);
        if (ret == LIBUSB_ERROR_PIPE) {
            /* A stalled data endpoint is how a device ends a data phase early
             * (BBB 6.7.2/6.7.3), with its CSW already queued on the Bulk-In
             * pipe. Clear the halt and read that CSW: left in the pipe, it
             * would be read as the next command's data, offsetting every
             * command after it. Its FAIL status is also what lets scsi.c ask
             * REQUEST SENSE why. */
            LOGW("token=%d: data phase stalled after %d/%d bytes, clearing "
                 "halt and collecting the CSW",
                 entry->token, transferred, data_len);
            bot_clear_stall(entry, ep);
            /* Falls through to the CSW read, with `transferred` holding
             * whatever libusb moved before the halt. */
        } else if (ret == LIBUSB_ERROR_TIMEOUT) {
            /* libusb splits large transfers, so a timeout may come after some
             * or all of the data arrived, and `transferred` says which. */
            if (transferred == data_len) {
                /* The device finished; its CSW is waiting. */
                LOGW("token=%d: data phase timed out but completed (%d bytes), "
                     "continuing to CSW", entry->token, transferred);
            } else {
                /* The device is still mid-data-phase: a CSW read now would
                 * take data for a CSW and offset the pipe by one for every
                 * command after it. Host and device disagree about the data
                 * phase, which only a Reset Recovery resolves. */
                LOG_QUIRK(entry, "data phase timed out after %d/%d bytes: "
                                 "device still mid-transfer, performing Reset Recovery",
                          transferred, data_len);
                laser_mass_storage_reset(entry);
                return BOT_FAIL_PHASE_ERROR;
            }
        } else if (ret == LIBUSB_ERROR_NO_DEVICE) {
            LOGW("token=%d: device gone during the data phase", entry->token);
            return BOT_FAIL_NO_DEVICE;
        } else if (ret != LIBUSB_SUCCESS) {
            /* Any other failure - an I/O error, an overflow from a device
             * sending more than asked - leaves the pipes in an unknown state,
             * with the CSW possibly still queued. Without a Reset Recovery
             * here, the next command would read that stale CSW, fail its tag
             * check and reset then, one command too late. */
            LOG_QUIRK(entry, "data phase failed after %d/%d bytes (%s): "
                             "performing Reset Recovery",
                      transferred, data_len, libusb_error_name(ret));
            laser_mass_storage_reset(entry);
            return BOT_FAIL_PHASE_ERROR;
        }
        if (actual_len) {
            *actual_len = transferred;
        }
        data_transferred = transferred;
    } else if (actual_len) {
        *actual_len = 0;
    }

    ret = libusb_bulk_transfer(entry->handle, entry->ep_in,
                               csw_buf, USB_BOT_CSW_SIZE, &transferred,
                               entry->probe_timeouts ? PROBE_CSW_PHASE_TIMEOUT_MS
                                                     : CSW_PHASE_TIMEOUT_MS);
    if (ret == LIBUSB_ERROR_PIPE) {
        /* BBB 6.7.3 gives a stalled status phase one more chance: clear the
         * halt and read the CSW again. Only a second failure calls for a
         * Reset Recovery. */
        LOGW("token=%d: CSW phase stalled, clearing halt and retrying once",
             entry->token);
        bot_clear_stall(entry, entry->ep_in);
        ret = libusb_bulk_transfer(entry->handle, entry->ep_in,
                                   csw_buf, USB_BOT_CSW_SIZE, &transferred,
                                   entry->probe_timeouts ? PROBE_CSW_PHASE_TIMEOUT_MS
                                                     : CSW_PHASE_TIMEOUT_MS);
    }
    if (ret == LIBUSB_ERROR_NO_DEVICE) {
        /* Before the Reset Recovery below, which would address a device that
         * is no longer on the bus. */
        LOGW("token=%d: device gone while reading the CSW", entry->token);
        return BOT_FAIL_NO_DEVICE;
    }
    if (transferred != USB_BOT_CSW_SIZE ||
        (ret != LIBUSB_SUCCESS && ret != LIBUSB_ERROR_TIMEOUT)) {
        /* No valid status for a command the drive has certainly seen: host
         * and device disagree about where the transaction ended, which is
         * what Reset Recovery is for (BBB 6.6.1). A timeout after all 13
         * bytes arrived is not that, and the CSW is used. */
        LOG_QUIRK(entry, "no valid CSW after retry (%s, %d/%d bytes): "
                         "performing Reset Recovery",
                  libusb_error_name(ret), transferred, USB_BOT_CSW_SIZE);
        laser_mass_storage_reset(entry);
        return BOT_FAIL_PHASE_ERROR;
    }

    memcpy(&csw, csw_buf, USB_BOT_CSW_SIZE);

    if (csw.dCSWSignature != USB_BOT_CSW_SIGNATURE || csw.dCSWTag != cbw.dCBWTag) {
        /* BBB 6.6.1: an invalid CSW calls for a Reset Recovery. A tag
         * mismatch most likely means a previous command's CSW, with more
         * queued behind it; only the reset flushes the pipes. */
        LOG_QUIRK(entry, "CSW signature/tag mismatch (sig %08x tag %u, "
                         "expected %08x/%u): performing Reset Recovery",
                  csw.dCSWSignature, csw.dCSWTag,
                  USB_BOT_CSW_SIGNATURE, cbw.dCBWTag);
        laser_mass_storage_reset(entry);
        return BOT_FAIL_PHASE_ERROR;
    }

    if (csw_status) {
        *csw_status = csw.bCSWStatus;
    }

    if (csw.bCSWStatus == USB_BOT_STATUS_PHASE_ERROR) {
        /* BBB 6.6.3/6.7: the host shall perform a Reset Recovery, before any
         * other command - REQUEST SENSE included, which the device would be
         * entitled to reject like the rest. Done here, the only place that
         * knows a Phase Error happened. */
        LOG_QUIRK(entry, "Phase Error (CSW 02h): performing Reset Recovery");
        laser_mass_storage_reset(entry);
        return BOT_FAIL_PHASE_ERROR;
    }

    /* BBB 6.3: for status 00h/01h, a CSW is meaningful only if its residue
     * does not exceed the requested length. One that does contradicts itself
     * and cannot be trusted. A plain error is the lighter response the
     * specification allows, the device not necessarily being out of sync. */
    if (csw.dCSWDataResidue > (uint32_t)data_len) {
        LOG_QUIRK(entry, "CSW not meaningful: residue %u > requested %d",
                  csw.dCSWDataResidue, data_len);
        return -1;
    }

    /* How many bytes of the data phase count? The wire count and the residue
     * do not always agree, and which one is wrong depends on the hardware: a
     * conforming device may pad a short transfer to full length (BBB cases
     * 4/5), making the residue right; several USB-SATA bridges in optical
     * enclosures report a residue that is simply wrong - Linux carries
     * US_FL_IGNORE_RESIDUE for them, the INIC-3619 among others - making the
     * wire count right. So the residue is used only where it cannot make
     * things worse:
     *
     *   residue == 0            both agree: the common case.
     *   transferred < data_len  the device sent short, so there is no padding
     *                           to see through: the smaller of the two, never
     *                           more than actually arrived.
     *   transferred == data_len ambiguous. Believing the residue would
     *     and residue > 0       truncate every read on such a bridge, so the
     *                           wire count is kept, and the contradiction
     *                           logged once per device. */
    if (data_len > 0 && data && actual_len) {
        int by_residue = data_len - (int)csw.dCSWDataResidue;

        if (csw.dCSWDataResidue == 0) {
            *actual_len = data_transferred;
        } else if (data_transferred < data_len) {
            *actual_len = by_residue < data_transferred ? by_residue
                                                        : data_transferred;
        } else {
            if (!entry->residue_quirk_logged) {
                entry->residue_quirk_logged = 1;
                LOG_QUIRK(entry, "residue %u contradicts a full %d-byte "
                                 "transfer; trusting the wire count "
                                 "(further occurrences not logged)",
                          csw.dCSWDataResidue, data_transferred);
            }
            *actual_len = data_transferred;
        }
    }

    if (csw.bCSWStatus != USB_BOT_STATUS_PASS) {
        return -1;
    }

    return 0;
}
