/*****************************************************************************
 * bot.c: one USB Mass Storage Bulk-Only transaction
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
 * CBW framing, the data phase in either direction, the CSW and its
 * validation, stall recovery and Reset Recovery - USB Mass Storage Class
 * Bulk-Only Transport, rev 1.0, and nothing above it.
 *
 * WHAT IS NOT HERE, and the line the split follows: this file has no retry
 * policy, no sense-code interpretation and no idea what any CDB means. It
 * performs exactly one transaction and reports, through its return value,
 * enough for scsi.c to decide whether that transaction may be replayed.
 * Everything that needs to know what a command IS lives there.
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

/* USB_BOT_STATUS_PASS/FAIL/PHASE_ERROR are in laser_internal.h: they are the
 * value space of laser_bot_send_locked()'s csw_status out-parameter, which
 * scsi.c reads, so they belong with that declaration rather than here. */

/* The Bulk-Only Mass Storage Reset class request is issued by
 * laser_mass_storage_reset() in usb.c, and its two constants live there with
 * it - as GET MAX LUN's do in scsi.c beside the only call that sends it. Each
 * class request sits with the function that issues it; what stays here is the
 * wire format of a transaction, which is this file's subject. */

/* The CBW and CSW below are built by assigning their multi-byte fields
 * directly and then handing the struct to libusb as a byte string. BOT
 * defines those fields as little-endian, so that shortcut is only correct
 * on a little-endian host.
 *
 * Every Android ABI in use is little-endian, so this holds today and the
 * shortcut is worth keeping - byte-swapping helpers on every field would
 * be noise around an invariant that has never been false here. What is
 * not acceptable is for it to become silently wrong if that ever changes,
 * hence the check rather than a comment alone. */
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__)
# if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#  error "laser: CBW/CSW are serialized by struct layout, which assumes a little-endian host"
# endif
#else
/* No way to ask. The previous form was "check if the compiler tells us",
 * which silently passed on any toolchain that does not define these - the
 * one case where the assumption most needs testing, since it is also the
 * case where nobody has thought about this port. Failing to build is the
 * correct outcome: whoever hits it either adds the byte-swapping or confirms
 * the target is little-endian and defines the macros. */
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

/* The wire sizes BBB fixes, and the whole reason for the pragma above.
 *
 * Both structures are handed to libusb as byte strings of USB_BOT_*_SIZE
 * bytes, so a packing directive that failed to apply - a toolchain that
 * ignores it, a header interposed between the push and the pop - would not
 * fail to build. It would send 32- or 40-byte CBWs that the drive rejects,
 * and read CSWs out of alignment with the fields they are copied into. The
 * symptom is a drive that refuses everything, a long way from the cause. */
_Static_assert(sizeof(bot_cbw_t) == USB_BOT_CBW_SIZE,
               "bot_cbw_t is not 31 bytes: #pragma pack did not apply");
_Static_assert(sizeof(bot_csw_t) == USB_BOT_CSW_SIZE,
               "bot_csw_t is not 13 bytes: #pragma pack did not apply");
/* Per-phase USB timeouts, in milliseconds.
 *
 * The data phase gets much longer than the wrapper phases because it is
 * the only one whose duration depends on the disc: a drive hitting a
 * damaged sector retries internally before answering, and a full
 * LASER_MAX_BYTES_PER_TRANSFER read from slow media is already
 * hundreds of milliseconds when everything goes well. The CBW and CSW
 * are 31 and 13 bytes of pure protocol overhead - if those do not come
 * back promptly, waiting longer will not help. */
#define CBW_PHASE_TIMEOUT_MS    3000
#define DATA_PHASE_TIMEOUT_MS   5000
#define CSW_PHASE_TIMEOUT_MS    3000

/* Timeouts for PROBES - commands that need no medium and that a working
 * device answers immediately: INQUIRY, and anything else asked only to find
 * out what we are talking to.
 *
 * The values above are sized for reads: a drive that has to seek, or that is
 * still spinning up, legitimately takes seconds, and cutting them short would
 * turn a slow read into a failed one. A probe has no such excuse. Letting it
 * inherit those values means a device that answers nothing costs five seconds
 * to establish it - measured on a card reader behind a misbehaving armv7 host
 * controller, where three INQUIRY attempts came to thirteen seconds on the
 * browse path.
 *
 * Set entry->probe_timeouts around such a command; laser_bot_send_locked()
 * reads it. Guarded by io_lock like everything else in that function, so the
 * bracket cannot overlap another consumer's command. */
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
 * The CONTRACT - what each return code means and what the caller may do
 * about it - lives with the declaration in laser_internal.h, because that
 * is what scsi.c reads. What follows is why each wire condition maps to the
 * code it does, which is only useful next to the code that decides it.
 *
 * Unlike the classification probe's version of this function, this one
 * supports DATA-OUT (needed for SEND KEY) via the data_in parameter -
 * everything else about the framing is unchanged.
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
    /* Bytes libusb actually moved during the data phase. Kept separately
     * because `transferred` is reused by the CSW read below, and because
     * it is needed after that read as a sanity bound on the residue. */
    int data_transferred = 0;
    int ret;

    if (csw_status) {
        *csw_status = -1;
    }

    /* Last line of defence on the CBW's fixed 16-byte command field.
     * laser_scsi_cdb() rejects an out-of-range cdb_len before it can
     * reach here, so this is unreachable from outside - but the memcpy
     * below is the actual overflow site, of a stack struct, and it is
     * worth one comparison to make that impossible independently of what
     * any caller does or any future entry point forgets to check. */
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
    if (ret != LIBUSB_SUCCESS || transferred != USB_BOT_CBW_SIZE) {
        LOGW("token=%d: CBW send failed: %s", entry->token,
             libusb_error_name(ret));
        /* The command never reached the drive - distinct from every
         * other failure below, and the only case where replaying a
         * non-idempotent command (SEND KEY) is provably harmless. */
        return BOT_FAIL_NOT_SENT;
    }

    if (data_len > 0 && data) {
        unsigned char ep = data_in ? entry->ep_in : entry->ep_out;

        ret = libusb_bulk_transfer(entry->handle, ep, data, data_len,
                                   &transferred,
                                   entry->probe_timeouts ? PROBE_DATA_PHASE_TIMEOUT_MS
                                                         : DATA_PHASE_TIMEOUT_MS);
        if (ret == LIBUSB_ERROR_PIPE) {
            /* A stalled data endpoint is not an aborted command. BBB
             * 6.7.2/6.7.3: this is how a device declines a data phase, or
             * ends one early, when it has less to say than the CBW asked
             * for - and it has ALREADY queued its CSW on the Bulk-In
             * pipe, waiting for the host to collect it. The prescribed
             * recovery is to clear the halt and read that status.
             *
             * Returning here instead left those 13 bytes in the pipe. The
             * next command's data phase then consumed them, so the pipe
             * stayed offset by one and every command after it read the
             * previous command's CSW - the same cascade, from the same
             * cause, that the partial-timeout branch below exists to
             * prevent.
             *
             * Collecting the CSW is also what makes the failure
             * diagnosable rather than merely fatal: it carries
             * bCSWStatus=FAIL, which is what lets the retry loop send
             * REQUEST SENSE and discover that the disc is, say, simply
             * gone. Giving up here turned every stalled read into a bare
             * I/O error, retried six times over for nothing. */
            LOGW("token=%d: data phase stalled after %d/%d bytes, clearing "
                 "halt and collecting the CSW",
                 entry->token, transferred, data_len);
            bot_clear_stall(entry, ep);
            /* Falls through to the CSW read, with `transferred` holding
             * whatever libusb moved before the halt. */
        } else if (ret == LIBUSB_ERROR_TIMEOUT) {
            /* A timeout does NOT mean nothing was transferred: libusb
             * splits large transfers into chunks to satisfy OS limits,
             * so the deadline can expire after some of them completed,
             * and libusb keeps whatever did get through. `transferred`
             * is therefore meaningful here and decides which of two very
             * different situations this is.
             *
             * Everything arrived: the device finished its data phase and
             * the clock simply ran out on the way. Its CSW is waiting on
             * the Bulk-In pipe, so carrying on to read it is correct. */
            if (transferred == data_len) {
                LOGW("token=%d: data phase timed out but completed (%d bytes), "
                     "continuing to CSW", entry->token, transferred);
            } else {
                /* Short: the device is still mid-data-phase. Reading the
                 * CSW now would read from a pipe that still holds data,
                 * and those 13 bytes would be data masquerading as a
                 * CSW. The signature/tag checks reject them, but the
                 * damage outlives this command: the pipe stays offset by
                 * one, so the NEXT command reads THIS command's real
                 * CSW, and every command after it inherits the shift.
                 * That shows up as a run of unexplained failures whose
                 * cause is nowhere near where they appear.
                 *
                 * BBB has no "carry on anyway" branch for a host and
                 * device that disagree about the data phase: recover via
                 * the CSW if the device is genuinely finished, otherwise
                 * Reset Recovery. It isn't finished, so reset - which
                 * also re-syncs the pipes, making the next command clean
                 * rather than the first of a cascade. */
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
            LOGW("token=%d: data phase failed: %s", entry->token,
                 libusb_error_name(ret));
            return -1;
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
        /* BBB 6.7.3 gives a stalled status phase exactly one more chance:
         * clear the halt and read the CSW again. Only if that second
         * attempt also fails is Reset Recovery called for.
         *
         * Giving up after the first stall spent an attempt from the retry
         * budget - and, for a DATA-OUT command, the whole command - on a
         * device that was one clear_halt away from answering normally. */
        LOGW("token=%d: CSW phase stalled, clearing halt and retrying once",
             entry->token);
        bot_clear_stall(entry, entry->ep_in);
        ret = libusb_bulk_transfer(entry->handle, entry->ep_in,
                                   csw_buf, USB_BOT_CSW_SIZE, &transferred,
                                   entry->probe_timeouts ? PROBE_CSW_PHASE_TIMEOUT_MS
                                                     : CSW_PHASE_TIMEOUT_MS);
    }
    if (ret == LIBUSB_ERROR_NO_DEVICE) {
        /* Checked before the Reset Recovery below: that reset is a control
         * transfer plus a 100ms settle sleep, all of it addressed to
         * something that is no longer on the bus. */
        LOGW("token=%d: device gone while reading the CSW", entry->token);
        return BOT_FAIL_NO_DEVICE;
    }
    if (ret != LIBUSB_SUCCESS || transferred != USB_BOT_CSW_SIZE) {
        /* No valid status for a command the drive has certainly seen.
         * Host and device now disagree about where this transaction
         * ended, which is exactly the condition Reset Recovery exists for
         * (BBB 6.6.1): returning a plain error would leave whatever the
         * device still has queued sitting in the pipe, to be misread as
         * the next command's data or status. */
        LOG_QUIRK(entry, "no valid CSW after retry (%s, %d/%d bytes): "
                         "performing Reset Recovery",
                  libusb_error_name(ret), transferred, USB_BOT_CSW_SIZE);
        laser_mass_storage_reset(entry);
        return BOT_FAIL_PHASE_ERROR;
    }

    memcpy(&csw, csw_buf, USB_BOT_CSW_SIZE);

    if (csw.dCSWSignature != USB_BOT_CSW_SIGNATURE || csw.dCSWTag != cbw.dCBWTag) {
        /* BBB 6.6.1: a CSW that fails the signature or tag check is not
         * valid, and the host shall perform a Reset Recovery. Detecting
         * the desynchronisation without repairing it would be the worst
         * of both worlds - a tag mismatch means these 13 bytes are most
         * likely a PREVIOUS command's status, so there is at least one
         * more CSW queued behind them and the offset survives into the
         * next command. Only the reset flushes the pipes. */
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
        /* BBB 6.6.3/6.7: Phase Error means the device's own state
         * machine is out of sync with ours, and the host "shall perform
         * a Reset Recovery". Until that happens the device is entitled
         * to fail or stall everything we send it, so simply returning an
         * error here - and letting the retry loop fire five more
         * commands at it - would burn the whole attempt budget on a
         * device that cannot answer any of them, and leave it wedged for
         * the rest of the session afterwards.
         *
         * The reset is done here rather than by the caller because this
         * is the only place that knows a Phase Error happened at all,
         * and because it must happen before ANY further command on this
         * device, including the REQUEST SENSE the retry loop would
         * otherwise send next (which is itself just another command the
         * device would reject). */
        LOG_QUIRK(entry, "Phase Error (CSW 02h): performing Reset Recovery");
        laser_mass_storage_reset(entry);
        return BOT_FAIL_PHASE_ERROR;
    }

    /* BBB 6.3: a CSW is "meaningful" only when, for status 00h/01h, the
     * residue does not exceed what we asked for. (For 02h it is to be
     * ignored entirely - handled above, before this point.) A residue
     * larger than dCBWDataTransferLength means the device is telling us
     * it left MORE bytes untransferred than we ever requested, which is
     * self-contradictory: the CSW cannot be trusted at all, so treat it
     * like any other unusable status rather than deriving a length from
     * it. 6.5 permits a Reset Recovery here; we return an error and let
     * the retry loop decide, which is the lighter of the two responses
     * and adequate since the device is not necessarily desynchronised. */
    if (csw.dCSWDataResidue > (uint32_t)data_len) {
        LOG_QUIRK(entry, "CSW not meaningful: residue %u > requested %d",
                  csw.dCSWDataResidue, data_len);
        return -1;
    }

    /* How many bytes of the data phase actually count?
     *
     * Two independent measurements are available and they do not always
     * agree: `data_transferred` is what libusb saw arrive on the wire,
     * and dCSWDataResidue is the device's own statement of what it left
     * untransferred, which BBB defines as dCBWDataTransferLength minus
     * the amount it really processed.
     *
     * Where they disagree, which one is wrong depends on the hardware,
     * and that is not knowable from here:
     *
     *   - A conforming device that sends less than requested may pad the
     *     transfer up to the full length (BBB cases 4/5). Then the wire
     *     count is inflated by fill bytes and the residue is right.
     *   - Several USB-SATA bridges used in optical-drive enclosures
     *     report a residue that is simply wrong; Linux carries an
     *     explicit US_FL_IGNORE_RESIDUE quirk for them, including for at
     *     least one bridge (INIC-3619) found in slimline optical drive
     *     enclosures - exactly the hardware this code targets. There the
     *     wire count is right and the residue is garbage.
     *
     * So the residue is used only where it cannot make things worse:
     *
     *   residue == 0            Nothing withheld. Both agree, and this
     *                           is the overwhelmingly common case.
     *   transferred < data_len  The device demonstrably sent short, so
     *                           there is no padding to see through. Take
     *                           the smaller of the two: never report
     *                           more bytes than actually arrived, since
     *                           the caller's buffer is uninitialised
     *                           beyond that point.
     *   transferred == data_len Ambiguous: either padding (residue
     *     and residue > 0       right) or a broken bridge (residue
     *                           wrong). Believing the residue here
     *                           truncates every read on such a bridge -
     *                           and this is the hot path, running
     *                           thousands of times per playback - so
     *                           keep the wire count, which is what this
     *                           transport did before the residue was
     *                           consulted at all, and log it. The log is
     *                           the point: it is the signal that would
     *                           justify a per-device quirk later, and
     *                           without it such a device is
     *                           indistinguishable from a healthy one. */
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