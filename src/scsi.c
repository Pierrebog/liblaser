/*****************************************************************************
 * scsi.c: SCSI-MMC on top of the Bulk-Only transport
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
 * Everything that needs to know what a command MEANS: sense-code
 * classification, the retry policy, the probes run once at registration
 * (INQUIRY, GET MAX LUN, TEST UNIT READY), the DVD region check and the
 * chunked block reads.
 *
 * Every transaction goes through laser_bot_send_locked() in bot.c, which
 * knows the wire format and nothing else. Keeping the resilience here spares
 * each consumer - libdvdcss, VLC's cdrom.c, the laser access module - from
 * reimplementing it.
 *****************************************************************************/

#include <string.h>
#include <time.h>
#include <unistd.h>

#include "laser.h"
#include "laser_internal.h"

/* Fixed-format sense data (SPC): sense key in the low nibble of byte 2,
 * Additional Sense Code (ASC) in byte 12, its qualifier (ASCQ) in byte 13.
 * Values checked against T10's assignment list (t10.org/lists/asc-num.txt).
 *
 * Whether the ASCQ must be matched depends on the ASC: every 3Ah qualifier
 * means "no medium" to an ordinary command, while under 28h only 00h means a
 * changed medium - 02h is a dual-layer DVD crossing layers mid-playback. */
#define SCSI_SENSE_KEY_NOT_READY        0x02
#define SCSI_SENSE_KEY_UNIT_ATTENTION   0x06

/* Sense keys that no retry can turn into a success: the drive understood
 * the command and refuses it as issued - a malformed CDB, an LBA past the
 * end, a scrambled sector before authentication. */
#define SCSI_SENSE_KEY_ILLEGAL_REQUEST  0x05
#define SCSI_SENSE_KEY_DATA_PROTECT     0x07

/* Under ILLEGAL REQUEST, read after a refused START STOP UNIT: "no such
 * opcode" and "not in this shape" call for opposite next steps. */
#define SCSI_ASC_INVALID_OPCODE         0x20
#define SCSI_ASC_INVALID_CDB_FIELD      0x24

/* 6Fh COPY PROTECTION KEY EXCHANGE FAILURE and neighbours, all under ILLEGAL
 * REQUEST, with three different remedies:
 *
 *   6Fh/00h AUTHENTICATION FAILURE
 *   6Fh/01h KEY NOT PRESENT
 *   6Fh/02h KEY NOT ESTABLISHED   -> re-authenticate; the sector is not
 *                                    permanently unreadable.
 *   6Fh/03h READ OF SCRAMBLED SECTOR WITHOUT AUTHENTICATION
 *                                 -> unreadable as things stand.
 *   6Fh/04h MEDIA REGION CODE IS MISMATCHED TO LOGICAL UNIT REGION
 *   6Fh/05h DRIVE REGION MUST BE PERMANENT/REGION RESET COUNT ERROR
 *                                 -> not an authentication problem; the user
 *                                    must be told, and the drive's region
 *                                    never changed on their behalf. */
#define SCSI_ASC_COPY_PROTECTION        0x6f
#define SCSI_ASCQ_CP_AUTH_FAILURE       0x00
#define SCSI_ASCQ_CP_KEY_NOT_PRESENT    0x01
#define SCSI_ASCQ_CP_KEY_NOT_ESTABLISHED 0x02
#define SCSI_ASCQ_CP_SCRAMBLED          0x03
#define SCSI_ASCQ_CP_REGION_MISMATCH    0x04
#define SCSI_ASCQ_CP_REGION_PERMANENT   0x05

/* 04h: not ready, 04h/01h being "becoming ready". Not a branch of its own in
 * the spin-up wait, only recorded: a drive that leaves the bus right after
 * saying it was drawing spindle current. */
#define SCSI_ASC_BECOMING_READY         0x04

#define SCSI_ASC_MEDIUM_NOT_PRESENT     0x3a
/* The qualifiers under 3Ah, which the spin-up wait tells apart. 3Ah/02h, tray
 * OPEN, is a physical fact no command or wait changes. 3Ah/00h and 3Ah/01h
 * only say the drive has no medium state, which is also what a parked drive
 * answers about a disc in its own closed tray. */
#define SCSI_ASCQ_MEDIUM_NOT_PRESENT      0x00
#define SCSI_ASCQ_TRAY_CLOSED             0x01
#define SCSI_ASCQ_TRAY_OPEN               0x02

#define SCSI_ASC_MEDIUM_MAY_HAVE_CHANGED 0x28
/* 28h/00h NOT READY TO READY CHANGE, MEDIUM MAY HAVE CHANGED - the only
 * qualifier under ASC 28h that actually means the disc was swapped. */
#define SCSI_ASCQ_MEDIUM_MAY_HAVE_CHANGED 0x00
/* 28h/02h FORMAT-LAYER MAY HAVE CHANGED - dual-layer boundary crossing,
 * transient and retryable; explicitly NOT an ejection. */
#define SCSI_ASCQ_FORMAT_LAYER_CHANGED    0x02

/* Per-command retry budget: about two and a half seconds of delays, short
 * enough for interactive playback. */
#define LASER_MAX_RETRIES       6
#define LASER_RETRY_DELAY_MS    500

/* Spin-up wait budget, far larger than the per-command one but spent once per
 * registration: a cold disc takes seconds to spin up, sometimes more than
 * ten. 30 x 500ms is 15 s while the drive keeps answering "not yet". */
#define LASER_SPINUP_MAX_ATTEMPTS  30
#define LASER_SPINUP_DELAY_MS      500

/* Hard ceiling on the real time the spin-up wait may take, whatever the
 * attempt count. The attempt budget assumes near-instant answers; a drive
 * that stops answering without leaving the bus makes each attempt cost two
 * full sets of phase timeouts, turning thirty attempts into minutes - spent
 * under the registry lock, blocking every teardown and registration. */
#define LASER_SPINUP_MAX_WALL_MS   15000

/* A shorter ceiling, for a drive answering 3Ah with a tray-closed or generic
 * qualifier, counted from the START STOP UNIT. Unlike "becoming ready", that
 * answer promises nothing - an empty tray gives it too - so it gets long
 * enough for a drive told to load to find its medium, and short enough that
 * an empty drive costs a pause rather than fifteen seconds. Whichever ceiling
 * comes first ends the wait. */
#define LASER_NO_MEDIUM_MAX_WALL_MS 4000

/* GET MAX LUN: Bulk-Only class request, Device-to-Host, returning one byte,
 * the number of the LAST logical unit (0 means one unit). A device with a
 * single unit may stall it. */
#define USB_BOT_GETMAXLUN_bREQUEST      0xFE
#define USB_BOT_GETMAXLUN_bmREQUESTTYPE 0xA1  /* Class | Interface | Dev-to-Host */

/* Highest LUN probed, whatever the device claims: each unit costs an INQUIRY
 * at registration, and combo enclosures use two at most. */
#define LASER_MAX_LUN_PROBED 3

/* SCSI INQUIRY (SPC), byte 0 low 5 bits: peripheral device type. */
#define SCSI_PDT_MASK           0x1f
#define SCSI_PDT_DIRECT_ACCESS  0x00
#define SCSI_PDT_CD_DVD         0x05

/* INQUIRY allocation length: the 36-byte standard short form every device
 * must answer. */
#define SCSI_INQUIRY_ALLOC_LEN  36

/* INQUIRY attempts on LUN 0, and the first delay between them, which doubles
 * each time: 100, 200, ... 3200 ms, about six seconds over seven attempts.
 *
 * On a kernel that mounts optical media, a drive classified just after it was
 * plugged in is caught mid-command by usb-storage, whose work nobody will now
 * collect; the device needs time, not another command. A healthy one answers
 * the first attempt and sleeps for none of it. */
#define LASER_INQUIRY_MAX_ATTEMPTS 7
#define LASER_INQUIRY_RETRY_MS     100

/* Forward declaration: the spin-up wait needs it before its definition. */
static int request_sense_locked(laser_entry_t *entry,
                                uint8_t *sense_key, uint8_t *asc, uint8_t *ascq);

/* SCSI TEST UNIT READY (opcode 0x00): no data phase, the CSW status alone
 * says whether the unit is ready.
 *
 * Returns laser_bot_send_locked()'s code as is: 0 means ready,
 * BOT_FAIL_NO_DEVICE means gone, anything else not ready yet.
 *
 * Caller MUST already hold entry->io_lock. */
static int test_unit_ready_locked(laser_entry_t *entry)
{
    uint8_t cdb[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    int csw_status = -1;
    return laser_bot_send_locked(entry, cdb, sizeof(cdb),
                                   NULL, 0, 0, NULL, &csw_status);
}

/* Milliseconds elapsed since *start on CLOCK_MONOTONIC, so that a budget is
 * not stretched or cut by the system clock being stepped, which Android does
 * routinely after a boot or a network change. */
static long monotonic_ms_since(const struct timespec *start)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec - start->tv_sec) * 1000L
           + (now.tv_nsec - start->tv_nsec) / 1000000L;
}

/* Only transfers at least this large are sampled: smaller ones measure
 * command latency, not the link. */
#define LASER_XFER_SAMPLE_MIN_BYTES  (32 * 1024)

/* Bytes sampled before the average is believed: one slow transfer - a seek,
 * a retried stall - proves nothing. */
#define LASER_XFER_SAMPLE_BYTES      (1024 * 1024)

/* The rate, in KiB/s, below which a link of this wMaxPacketSize is not
 * performing as its speed implies. Far below what it can do, to catch only a
 * drive running at a fraction of its capability: healthy high speed sustains
 * about 15,000 KiB/s, the underpowered drive this was written for 90. Full
 * speed gets a lower floor, ~1,000 KiB/s being its ceiling. */
static uint32_t slow_floor_kib_per_s(uint16_t max_packet)
{
    return max_packet <= 64 ? 250 : 1000;
}

/* Accumulates read throughput, and reports it once per registration - as a
 * warning if it is far below what the link should carry.
 *
 * An underpowered drive enumerates, answers every command correctly and
 * reads at a fraction of its rate. Nothing fails, so nothing else here would
 * report it; the only visible symptom is video arriving late, which points
 * everywhere but at the cause. The healthy figure is logged too, for bug
 * reports.
 *
 * Caller MUST already hold entry->io_lock. */
static void sample_throughput_locked(laser_entry_t *entry, int data_len,
                                     const struct timespec *started)
{
    if (entry->slow_warned || data_len < LASER_XFER_SAMPLE_MIN_BYTES) {
        return;
    }

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    const int64_t us = (int64_t)(now.tv_sec - started->tv_sec) * 1000000
                     + (now.tv_nsec - started->tv_nsec) / 1000;

    entry->xfer_bytes += (uint64_t)data_len;
    entry->xfer_us    += (uint64_t)(us > 0 ? us : 0);
    entry->xfer_count++;

    if (entry->xfer_bytes < LASER_XFER_SAMPLE_BYTES || entry->xfer_us == 0) {
        return;
    }

    const uint64_t kib_per_s = (entry->xfer_bytes * 1000000)
                             / (entry->xfer_us * 1024);
    const uint32_t floor_kib = slow_floor_kib_per_s(entry->ep_max_packet);

    /* Per transfer, which is the shape a reader can compare by eye. */
    const uint64_t per_kib = entry->xfer_bytes / entry->xfer_count / 1024;
    const uint64_t per_ms  = entry->xfer_us / entry->xfer_count / 1000;

    entry->slow_warned = 1;

    if (kib_per_s >= floor_kib) {
        LOGI("token=%d: sustained read throughput %llu KiB/s "
             "(%llu KiB in %llu ms per transfer, wMaxPacketSize %u)",
             entry->token, (unsigned long long)kib_per_s,
             (unsigned long long)per_kib, (unsigned long long)per_ms,
             entry->ep_max_packet);
        return;
    }

    LOGW("token=%d, usb %04x:%04x: sustained read throughput is only "
         "%llu KiB/s - %llu KiB takes %llu ms per transfer, on a link whose "
         "wMaxPacketSize of %u implies far more. NO COMMAND HAS FAILED: the "
         "drive answers everything correctly, just slowly, and the retries "
         "costing that time happen in the host controller and the drive's own "
         "firmware, below anything this library can observe. THE USUAL CAUSE "
         "IS INSUFFICIENT POWER: an underfed drive enumerates and answers "
         "correctly while reading at a fraction of its rate. Try a powered "
         "hub or a stronger supply. A DVD may survive this; a Blu-ray will "
         "stutter.",
         entry->token, entry->vid, entry->pid,
         (unsigned long long)kib_per_s, (unsigned long long)per_kib,
         (unsigned long long)per_ms, entry->ep_max_packet);
}

/* One START STOP UNIT in the exact shape asked for. Returns the CSW status,
 * and fills the sense triple when that status is FAIL - the sense being what
 * tells a missing opcode from a wrong shape from a refusal of the drive's
 * own. */
static int start_stop_unit_locked(laser_entry_t *entry,
                                  uint8_t immed, uint8_t byte4,
                                  uint8_t *sense_key, uint8_t *asc,
                                  uint8_t *ascq)
{
    uint8_t cdb[6] = { 0 };
    cdb[0] = 0x1b;  /* START STOP UNIT */
    cdb[1] = immed; /* bit 0 IMMED */
    cdb[4] = byte4; /* bit 0 START, bit 1 LOEJ */

    int csw_status = -1;
    int rc = laser_bot_send_locked(entry, cdb, sizeof(cdb),
                                   NULL, 0, 0, NULL, &csw_status);

    *sense_key = 0xff;
    *asc = 0;
    *ascq = 0;

    if (rc != 0 && csw_status == USB_BOT_STATUS_FAIL) {
        request_sense_locked(entry, sense_key, asc, ascq);
    }

    LOGI("token=%d: START STOP UNIT immed=%u byte4=0x%02x -> rc=%d csw=%d "
         "sense %02x/%02x/%02x",
         entry->token, immed, byte4, rc, csw_status,
         *sense_key, *asc, *ascq);

    return csw_status;
}

/* Tell the drive to load and spin its medium up, escalating through the
 * shapes a bridge might accept.
 *
 * TEST UNIT READY is a status query: some firmware starts the mechanism on
 * it, some does not, and a drive of the latter kind would answer MEDIUM NOT
 * PRESENT about a readable disc forever. On Linux, sr_mod sends this command
 * when the device is opened; with the kernel driver detached, nothing else
 * would.
 *
 * Byte 4: bit 0 START, bit 1 LOEJ ("load first"). IMMED is set on the first
 * attempt because a blocking spin-up can outlast bot.c's fixed phase
 * timeouts.
 *
 *   1.  IMMED, START: the correct request, and the cheapest.
 *   2a. 05h/20h INVALID OPCODE: stop. Every other shape is the same opcode.
 *   2b. 05h/24h INVALID FIELD IN CDB: retry without IMMED, the field ATAPI
 *       bridges most often lack. This one may block for the whole spin-up.
 *   2c. anything else: retry with LOEJ, "load the medium" being a different
 *       request from "spin what you have". A 05h/24h answer to that names
 *       LOEJ, so the ladder ends there: a drive with no motorised load
 *       refuses it with or without IMMED, as a 152d:0583 bridge showed.
 *
 * Never with the tray open: LOEJ would close it. Best-effort; the caller
 * polls either way.
 *
 * Caller MUST already hold entry->io_lock. */
static void spin_up_locked(laser_entry_t *entry)
{
    uint8_t sense_key, asc, ascq;

    if (start_stop_unit_locked(entry, 1, 0x01,
                               &sense_key, &asc, &ascq) == USB_BOT_STATUS_PASS) {
        return;
    }

    if (sense_key == SCSI_SENSE_KEY_ILLEGAL_REQUEST &&
        asc == SCSI_ASC_INVALID_OPCODE) {
        LOGW("token=%d: drive does not implement START STOP UNIT - nothing "
             "here can ask it to load a medium", entry->token);
        return;
    }

    if (sense_key == SCSI_SENSE_KEY_ILLEGAL_REQUEST &&
        asc == SCSI_ASC_INVALID_CDB_FIELD) {
        LOGI("token=%d: START STOP UNIT refused the CDB, retrying without "
             "IMMED (this one may block for the whole spin-up)",
             entry->token);
        start_stop_unit_locked(entry, 0, 0x01, &sense_key, &asc, &ascq);
        return;
    }

    LOGI("token=%d: START STOP UNIT refused for its own reasons, retrying "
         "with LOEJ set", entry->token);
    start_stop_unit_locked(entry, 1, 0x03, &sense_key, &asc, &ascq);
}

/* GET EVENT STATUS NOTIFICATION (opcode 4Ah), media class, polled: what the
 * drive believes about its tray and medium, for the log of a given-up wait.
 *
 * Diagnostic only, and not ground truth: it is the same firmware's belief as
 * the sense data, and has reported "tray closed, medium absent" for a disc
 * read moments later. Decisions are made on the sense qualifier.
 *
 * Caller MUST already hold entry->io_lock. */
static void log_media_status_locked(laser_entry_t *entry)
{
    uint8_t cdb[10] = { 0 };
    cdb[0] = 0x4a; /* GET EVENT STATUS NOTIFICATION */
    cdb[1] = 0x01; /* Polled */
    cdb[4] = 0x10; /* notification class request: bit 4 = media */
    cdb[7] = 0x00;
    cdb[8] = 0x08; /* allocation length: header + media event descriptor */

    uint8_t buf[8] = { 0 };
    int actual_len = 0;
    int rc = laser_bot_send_locked(entry, cdb, sizeof(cdb), buf, sizeof(buf),
                                     1, &actual_len, NULL);

    if (rc != 0 || actual_len < 8) {
        LOGI("token=%d: media status unavailable (rc=%d, %d bytes) - the "
             "drive does not support 4Ah, or would not answer it",
             entry->token, rc, actual_len);
        return;
    }

    /* Header: [0..1] descriptor length, [2] bit7 NEA + class in bits 0-2,
     * [3] supported classes. Media descriptor: [4] event code in the low
     * nibble, [5] status bits, [6..7] slot range. */
    int no_event = (buf[2] & 0x80) != 0;
    int class = buf[2] & 0x07;
    int event = buf[4] & 0x0f;
    int tray_open = (buf[5] & 0x01) != 0;
    int media_present = (buf[5] & 0x02) != 0;

    static const char *const events[] = {
        "no change", "eject request", "new media", "media removal",
        "media changed",
    };

    LOGI("token=%d: media status: tray %s, medium %s, last event %s "
         "(nea=%d class=%d raw=%02x %02x %02x %02x)",
         entry->token,
         tray_open ? "OPEN" : "closed",
         media_present ? "PRESENT" : "absent",
         event < (int)(sizeof(events) / sizeof(events[0])) ? events[event]
                                                           : "unknown",
         no_event, class, buf[2], buf[3], buf[4], buf[5]);
}

/* Wake the drive and wait for its medium to become ready - see the contract
 * in laser_internal.h.
 *
 * A drive that has spun down, or never spun up since the disc went in,
 * answers the first data command NOT READY, and some firmware fails a READ
 * that arrives cold rather than starting the spin-up. So: one START STOP
 * UNIT, then TEST UNIT READY in a loop until the unit answers ready.
 *
 * The loop ends early on a tray open (3Ah/02h), which no command closes, and
 * on a device that has left the bus. Other 3Ah answers keep it polling under
 * LASER_NO_MEDIUM_MAX_WALL_MS; everything else under LASER_SPINUP_MAX_WALL_MS
 * and LASER_SPINUP_MAX_ATTEMPTS.
 *
 * Not cancellable, and it need not be: it runs inside laser_acquire(), under
 * g_registry_lock, on an entry not yet published. */
void laser_wait_until_ready(laser_entry_t *entry)
{
    struct timespec started;
    clock_gettime(CLOCK_MONOTONIC, &started);

    /* When START STOP UNIT went out: the start of the no-medium ceiling,
     * since "not yet" only means something once the drive was asked to
     * load. */
    struct timespec no_medium_since = started;

    pthread_mutex_lock(&entry->io_lock);

    /* Asked before the first poll rather than on a first 3Ah answer: a drive
     * answering "becoming ready" as a generic busy state would otherwise
     * never be told to load. One command too many on a drive that did not
     * need it. */
    spin_up_locked(entry);
    clock_gettime(CLOCK_MONOTONIC, &no_medium_since);

    /* Lowered only by the ready return below, so that every other way out of
     * the loop leaves it raised. */
    entry->not_ready = 1;

    /* Whether the drive said it was spinning up, for the device-gone report
     * below. */
    int becoming_ready = 0;

    for (int attempt = 1; attempt <= LASER_SPINUP_MAX_ATTEMPTS; attempt++) {
        int rc = test_unit_ready_locked(entry);

        if (rc == 0) {
            LOGI("token=%d: unit ready (attempt %d/%d, %ldms)",
                 entry->token, attempt, LASER_SPINUP_MAX_ATTEMPTS,
                 monotonic_ms_since(&started));
            entry->not_ready = 0;
            pthread_mutex_unlock(&entry->io_lock);
            return;
        }

        if (rc == BOT_FAIL_NO_DEVICE) {
            LOGW("token=%d: device gone during spin-up wait, abandoning it",
                 entry->token);

            /* Leaving the bus while spinning up - when a cold drive draws its
             * peak current - is the signature of a port that cannot power it.
             * Every layer above only sees a device that disappeared. */
            if (becoming_ready) {
                LOGW("token=%d: it was spinning up when it went (%ldms in) - "
                     "suspect the port's power budget, not the drive: try a "
                     "powered hub or a Y-cable",
                     entry->token, monotonic_ms_since(&started));
            }

            pthread_mutex_unlock(&entry->io_lock);
            return;
        }

        uint8_t sense_key = 0xff, asc = 0, ascq = 0;
        request_sense_locked(entry, &sense_key, &asc, &ascq);

        /* Logged raw: 02h/3Ah, 02h/04h/01h and 06h/28h/00h call for different
         * handling, and the messages below do not show which arrived. */
        LOGI("token=%d: sense %02x/%02x/%02x (attempt %d/%d)",
             entry->token, sense_key, asc, ascq, attempt,
             LASER_SPINUP_MAX_ATTEMPTS);

        if (sense_key == SCSI_SENSE_KEY_NOT_READY &&
            asc == SCSI_ASC_BECOMING_READY) {
            becoming_ready = 1;
        }

        if (sense_key == SCSI_SENSE_KEY_NOT_READY &&
            asc == SCSI_ASC_MEDIUM_NOT_PRESENT) {
            if (ascq == SCSI_ASCQ_TRAY_OPEN) {
                LOGW("token=%d: tray open (attempt %d/%d), giving up wait",
                     entry->token, attempt, LASER_SPINUP_MAX_ATTEMPTS);
                log_media_status_locked(entry);
                pthread_mutex_unlock(&entry->io_lock);
                return;
            }

            /* Tray closed, or no qualifier: an empty tray and a parked drive
             * holding a disc alike. Keep polling; the shorter ceiling bounds
             * the empty case. */
            long no_medium_ms = monotonic_ms_since(&no_medium_since);
            if (no_medium_ms >= LASER_NO_MEDIUM_MAX_WALL_MS) {
                LOGW("token=%d: still no medium %ldms after START STOP UNIT "
                     "(attempt %d/%d), giving up wait",
                     entry->token, no_medium_ms, attempt,
                     LASER_SPINUP_MAX_ATTEMPTS);
                log_media_status_locked(entry);
                pthread_mutex_unlock(&entry->io_lock);
                return;
            }

            LOGI("token=%d: no medium yet, waiting %dms (attempt %d/%d, "
                 "%ldms of %dms since START STOP UNIT)",
                 entry->token, LASER_SPINUP_DELAY_MS, attempt,
                 LASER_SPINUP_MAX_ATTEMPTS, no_medium_ms,
                 LASER_NO_MEDIUM_MAX_WALL_MS);
            usleep(LASER_SPINUP_DELAY_MS * 1000);
            continue;
        }

        /* Checked before committing to another round, so that the ceiling
         * bounds the time actually spent; after the sense checks, so that an
         * open tray or a missing medium ends the wait on its own terms. */
        long elapsed = monotonic_ms_since(&started);
        if (elapsed >= LASER_SPINUP_MAX_WALL_MS) {
            LOGW("token=%d: spin-up wall-clock budget exhausted (%ldms over "
                 "%d attempts), proceeding anyway",
                 entry->token, elapsed, attempt);
            break;
        }

        if (sense_key == SCSI_SENSE_KEY_UNIT_ATTENTION) {
            /* Cleared by being reported; retry at once, no delay. */
            continue;
        }

        LOGI("token=%d: drive not ready, waiting %dms (attempt %d/%d, %ldms "
             "of %dms elapsed)",
             entry->token, LASER_SPINUP_DELAY_MS,
             attempt, LASER_SPINUP_MAX_ATTEMPTS,
             elapsed, LASER_SPINUP_MAX_WALL_MS);
        usleep(LASER_SPINUP_DELAY_MS * 1000);
    }

    LOGW("token=%d: drive did not become ready after %ldms, proceeding anyway",
         entry->token, monotonic_ms_since(&started));
    pthread_mutex_unlock(&entry->io_lock);
}

/* One INQUIRY on the unit selected by entry->lun, under the probe timeouts:
 * at the read timeouts, an absent unit of a multi-LUN device would cost
 * eleven seconds of registration.
 *
 * @param pdt receives the peripheral device type on success.
 * @return 0 on success, BOT_FAIL_NO_DEVICE if the device is gone, -1 if the
 *         unit did not answer.
 */
static int inquiry_pdt(laser_entry_t *entry, int attempts, uint8_t *pdt)
{
    uint8_t cdb[6] = { 0x12, 0x00, 0x00, 0x00, SCSI_INQUIRY_ALLOC_LEN, 0x00 };
    uint8_t inq[SCSI_INQUIRY_ALLOC_LEN];

    for (int attempt = 1; attempt <= attempts; ++attempt) {
        int actual = 0;
        memset(inq, 0, sizeof(inq));

        pthread_mutex_lock(&entry->io_lock);
        entry->probe_timeouts = 1;
        int rc = laser_bot_send_locked(entry, cdb, sizeof(cdb),
                                       inq, sizeof(inq), 1, &actual, NULL);
        entry->probe_timeouts = 0;
        pthread_mutex_unlock(&entry->io_lock);

        if (rc == 0 && actual >= 1) {
            *pdt = inq[0] & SCSI_PDT_MASK;
            return 0;
        }

        if (rc == BOT_FAIL_NO_DEVICE) {
            LOGI("token=%d: device gone during INQUIRY", entry->token);
            return BOT_FAIL_NO_DEVICE;
        }

        if (attempt == attempts) {
            LOGI("token=%d: LUN %u INQUIRY unanswered (rc=%d, %d bytes, "
                 "attempt %d/%d, giving up)",
                 entry->token, entry->lun, rc, actual, attempt, attempts);
            break;
        }

        /* Doubling from LASER_INQUIRY_RETRY_MS: the wait is what the retry
         * consists of. */
        const int delay_ms = LASER_INQUIRY_RETRY_MS << (attempt - 1);

        LOGI("token=%d: LUN %u INQUIRY unanswered (rc=%d, %d bytes, "
             "attempt %d/%d), settling %dms",
             entry->token, entry->lun, rc, actual, attempt, attempts,
             delay_ms);

        usleep((useconds_t)delay_ms * 1000);
    }

    return -1;
}

int laser_probe_lun(laser_entry_t *entry)
{
    /* Set first, so that every early return leaves a usable value. */
    entry->lun = 0;

    unsigned char max_lun_buf = 0;
    int ret = libusb_control_transfer(entry->handle,
                                      USB_BOT_GETMAXLUN_bmREQUESTTYPE,
                                      USB_BOT_GETMAXLUN_bREQUEST,
                                      0, entry->iface_num,
                                      &max_lun_buf, 1, 3000);
    int max_lun = 0;
    if (ret != 1) {
        /* Stalled or unanswered means a single unit, per the class
         * specification: the common case, not worth a warning. */
        LOGI("token=%d: GET MAX LUN unsupported or failed, assuming single LUN",
             entry->token);
    } else {
        max_lun = max_lun_buf;
        if (max_lun > LASER_MAX_LUN_PROBED)
            max_lun = LASER_MAX_LUN_PROBED;
        if (max_lun > 0) {
            LOGI("usb %04x:%04x: device reports %d logical units, looking for "
                 "the optical one", entry->vid, entry->pid, max_lun_buf + 1);
        }
    }

    /* Ask each unit what it is, and take the first CD/DVD one. entry->lun
     * selects the unit the CBW addresses.
     *
     * Only LUN 0 is retried: its INQUIRY is the first command after the Mass
     * Storage Reset, which a bridge still settling may fail. Units 1..3 are
     * usually absent on the drives that report them, and retrying them would
     * multiply the cost of the case this keeps cheap. */
    uint8_t lun0_pdt = 0;
    int have_lun0_pdt = 0;

    for (int lun = 0; lun <= max_lun; lun++) {
        entry->lun = (uint8_t)lun;

        uint8_t pdt = 0;
        int rc = inquiry_pdt(entry, lun == 0 ? LASER_INQUIRY_MAX_ATTEMPTS : 1,
                             &pdt);

        if (rc == BOT_FAIL_NO_DEVICE) {
            entry->lun = 0;
            LOGW("token=%d: device left the bus during the LUN probe",
                 entry->token);
            return LASER_OPTICAL_GONE;
        }
        if (rc != 0)
            continue; /* Unit absent or unhappy; try the next one. */

        LOGI("token=%d: LUN %d peripheral device type 0x%02x",
             entry->token, lun, pdt);

        if (lun == 0) {
            lun0_pdt = pdt;
            have_lun0_pdt = 1;
        }

        if (pdt == SCSI_PDT_CD_DVD) {
            LOGI("token=%d: using LUN %d (optical)", entry->token, lun);
            return LASER_OPTICAL_YES; /* entry->lun already holds it. */
        }
    }

    /* No unit said optical: back to LUN 0 rather than wherever the loop
     * stopped. */
    entry->lun = 0;

    if (!have_lun0_pdt) {
        /* INQUIRY needs no medium: a device that answers it on no unit is not
         * talking at all. */
        LOGI("token=%d: INQUIRY unanswered on every unit", entry->token);
        return LASER_OPTICAL_NO_ANSWER;
    }

    if (max_lun > 0) {
        LOG_QUIRK(entry, "multi-LUN device but none identified as optical, "
                         "falling back to LUN 0");
    }

    /* Only a direct-access block device is rejected - a card reader, a USB
     * key, an external disk, which Android serves through its own storage
     * path. Any other unexpected type is accepted: losing a real drive to a
     * lying descriptor would be far worse than probing one device too many. */
    if (lun0_pdt == SCSI_PDT_DIRECT_ACCESS) {
        LOGI("token=%d: direct-access block device, not an optical drive",
             entry->token);
        return LASER_OPTICAL_NO;
    }

    return LASER_OPTICAL_YES;
}

/* REQUEST SENSE (opcode 0x03) - decode the reason for the previous CHECK
 * CONDITION. Caller MUST already hold entry->io_lock. */
static int request_sense_locked(laser_entry_t *entry,
                                uint8_t *sense_key, uint8_t *asc, uint8_t *ascq)
{
    uint8_t cdb[6] = { 0x03, 0x00, 0x00, 0x00, 0x18, 0x00 };
    uint8_t buffer[24] = {0};
    int actual_len = 0;

    if (laser_bot_send_locked(entry, cdb, sizeof(cdb), buffer, sizeof(buffer),
                                1, &actual_len, NULL) < 0) {
        return -1;
    }
    if (actual_len < 14) {
        return -1;
    }

    *sense_key = buffer[2] & 0x0f;
    *asc = buffer[12];
    *ascq = buffer[13];
    return 0;
}

/* ============================================================================
 * Public: laser_scsi_cdb() - the generic, retrying, mutex-protected CDB
 * primitive everything else goes through.
 * ============================================================================ */

laser_status_t laser_scsi_cdb(int token,
                              const uint8_t *cdb, int cdb_len,
                              uint8_t *data, int data_len,
                              int data_in, int *actual_len)
{
    /* Caller bugs, caught here because their symptoms appear far from the
     * cause: an oversized cdb_len overflows the CBW's 16-byte field, and a
     * NULL buffer with a data length announces a data phase that never
     * happens, ending in a Reset Recovery with nothing in the log pointing
     * at the mistake. */
    if (cdb == NULL || cdb_len <= 0 || cdb_len > 16) {
        LOGE("token=%d: invalid CDB (%p, %d bytes; expected 1-16)",
             token, (const void *)cdb, cdb_len);
        return LASER_ERR_INVALID;
    }
    if (data_len < 0 || (data == NULL && data_len > 0)) {
        LOGE("token=%d: invalid data phase (%p, %d bytes)",
             token, (const void *)data, data_len);
        return LASER_ERR_INVALID;
    }
    if (data_len > LASER_MAX_BYTES_PER_TRANSFER) {
        /* Warned about, not rejected: libusb may well carry it, but it is
         * beyond what is reliable across Android host controllers, and the
         * block helpers exist so that nobody has to. */
        LOGW("token=%d: %d-byte data phase exceeds the %d-byte per-transfer "
             "budget; prefer laser_read_blocks()/read_cd_blocks()",
             token, data_len, LASER_MAX_BYTES_PER_TRANSFER);
    }

    /* A lookup, never a registration: a command on an unclaimed token is a
     * caller bug, and registering here would create an entry nobody
     * releases. */
    laser_entry_t *entry = laser_lookup(token);
    if (entry == NULL) {
        LOGW("token=%d: no such token - is a claim held? "
             "(laser_acquire() is what registers a device)", token);
        return LASER_ERR_NO_SUCH_TOKEN;
    }

    /* Whether this command changes CSS authentication state decides two
     * things: whether a session is required, and whether a retry is safe. */
    const int css_state_changing = laser_cdb_changes_css_state(cdb, cdb_len);

    /* Refused outside any session rather than wrapped in one: exclusion over
     * a single transaction is io_lock already, and what needs protecting is
     * the sequence, which only the consumer can see. This stops an
     * undeclared consumer from taking an AGID mid-handshake.
     *
     * Checked before io_lock - see the lock order in registry.c. */
    if (css_state_changing && !laser_css_session_is_open(entry)) {
        LOGE("token=%d, usb %04x:%04x: CSS command 0x%02x issued with no "
             "session open - refused. Call laser_css_session_begin() "
             "first; see the session contract in laser.h",
             token, entry->vid, entry->pid, cdb[0]);
        return LASER_ERR_IO;
    }

    /* Before queueing on io_lock, so that a command arriving during teardown
     * does not first wait out the transaction in progress. */
    if (laser_is_cancelled(entry)) {
        return LASER_ERR_CANCELLED;
    }

    if (entry->device_gone) {
        return LASER_ERR_NO_DEVICE;
    }

    pthread_mutex_lock(&entry->io_lock);

    laser_status_t result = LASER_ERR_IO;

    /* Is this command safe to send twice? Reads are. A command changing CSS
     * state is not: each accepted handshake step advances the drive's state
     * machine, and every accepted AGID request takes one of the drive's four
     * AGIDs - a retry through a UNIT ATTENTION could exhaust them until the
     * drive is unplugged. Recovering such a failure - invalidating the AGID
     * and starting over - is libdvdcss's job.
     *
     * Not keyed on direction: three handshake steps are REPORT KEY, which is
     * DATA-IN. And keyed on the same predicate as the session check, so that
     * the read-only key queries libdvdcss uses to decide whether a disc is
     * scrambled at all keep their retries. */
    const int idempotent = !css_state_changing && (data_in || data_len == 0);

    /* Last sense seen, so that the final failure can say why. */
    uint8_t last_sense_key = 0xff, last_asc = 0, last_ascq = 0;

    /* Attempts made, and whether a branch below already explained the
     * failure: the summary at the end must not claim a budget was exhausted
     * when the loop broke out on the first refusal. */
    int attempts_made = 0;
    int reason_logged = 0;

    /* Attempts that failed before the CBW could be handed over - see the
     * verdict after the loop. */
    int cbw_never_sent = 0;

    for (int attempt = 1; attempt <= LASER_MAX_RETRIES; attempt++) {
        attempts_made = attempt;

        /* Cancellation is honoured between attempts: a transfer already
         * handed to the kernel runs to its timeout, but no further attempt
         * follows. */
        if (laser_is_cancelled(entry)) {
            LOGI("token=%d: cancelled, abandoning cdb 0x%02x after %d "
                 "attempt(s)", token, cdb[0], attempt - 1);
            result = LASER_ERR_CANCELLED;
            break;
        }

        /* Local to this attempt: request_sense_locked() below issues its
         * own BOT transaction, and this value must survive it intact. */
        int csw_status = -1;
        struct timespec xfer_started;
        clock_gettime(CLOCK_MONOTONIC, &xfer_started);

        int rc = laser_bot_send_locked(entry, cdb, cdb_len, data, data_len,
                                         data_in, actual_len, &csw_status);
        if (rc == 0) {
            /* A success after retries is logged: otherwise a drive limping
             * through its budget on every command looks, in the log, like
             * one answering at once. */
            if (attempt > 1) {
                LOGW("token=%d: cdb 0x%02x succeeded on attempt %d/%d - "
                     "earlier attempts failed and were retried",
                     token, cdb[0], attempt, LASER_MAX_RETRIES);
            }
            sample_throughput_locked(entry, data_len, &xfer_started);
            result = LASER_OK;
            break;
        }

        if (rc == BOT_FAIL_NOT_SENT) {
            cbw_never_sent++;
        }

        if (rc == BOT_FAIL_NO_DEVICE) {
            /* No retry can bring the drive back, and each would cost a full
             * set of USB timeouts. Reported as its own status: an I/O error
             * would invite retries, and MEDIA_GONE would say the drive
             * answered. */
            LOGW("token=%d: device no longer present, not retrying", token);
            entry->device_gone = 1;
            result = LASER_ERR_NO_DEVICE;
            break;
        }

        uint8_t sense_key = 0xff, asc = 0, ascq = 0;
        if (rc == BOT_FAIL_PHASE_ERROR) {
            /* The device was just reset: there is no completed command left
             * for REQUEST SENSE to explain. The attempt still counts. */
            LOGW("token=%d: cdb 0x%02x ended in a Reset Recovery (attempt "
                 "%d/%d)", token, cdb[0], attempt, LASER_MAX_RETRIES);
        } else if (csw_status == USB_BOT_STATUS_FAIL) {
            request_sense_locked(entry, &sense_key, &asc, &ascq);
            last_sense_key = sense_key;
            last_asc = asc;
            last_ascq = ascq;
        }

        /* Disc gone or swapped: never retried, so that an ejection is felt at
         * once. Every 3Ah qualifier, unlike in the spin-up wait: that wait is
         * the only place that waits, and it has already run, so a fresh 3Ah
         * here means the disc really is gone. */
        if (sense_key == SCSI_SENSE_KEY_NOT_READY &&
            asc == SCSI_ASC_MEDIUM_NOT_PRESENT) {
            LOGW("token=%d: no disc present (%02x/%02x/%02x), not retrying",
                 token, sense_key, asc, ascq);
            result = LASER_ERR_MEDIA_GONE;
            break;
        }
        if (asc == SCSI_ASC_MEDIUM_MAY_HAVE_CHANGED &&
            ascq == SCSI_ASCQ_MEDIUM_MAY_HAVE_CHANGED) {
            LOGW("token=%d: medium may have changed, not retrying", token);
            result = LASER_ERR_MEDIA_GONE;
            break;
        }
        /* Copy-protection refusals, told apart before the generic refusal
         * below swallows them. None is retried, but they mean different
         * things to the caller, who cannot recover the qualifier once this
         * function has returned. */
        if (sense_key == SCSI_SENSE_KEY_ILLEGAL_REQUEST &&
            asc == SCSI_ASC_COPY_PROTECTION) {
            switch (ascq) {
            case SCSI_ASCQ_CP_SCRAMBLED:
                LOGW("token=%d: cdb 0x%02x refused, scrambled sector without "
                     "authentication (sense %02x/%02x/%02x)",
                     token, cdb[0], sense_key, asc, ascq);
                result = LASER_ERR_SCRAMBLED;
                break;
            case SCSI_ASCQ_CP_AUTH_FAILURE:
            case SCSI_ASCQ_CP_KEY_NOT_PRESENT:
            case SCSI_ASCQ_CP_KEY_NOT_ESTABLISHED:
                LOGW("token=%d: cdb 0x%02x refused, no CSS session in force "
                     "(sense %02x/%02x/%02x) - re-authentication may make "
                     "this succeed", token, cdb[0], sense_key, asc, ascq);
                result = LASER_ERR_NO_KEY;
                break;
            case SCSI_ASCQ_CP_REGION_MISMATCH:
            case SCSI_ASCQ_CP_REGION_PERMANENT:
                LOGW("token=%d, usb %04x:%04x: cdb 0x%02x refused on REGION "
                     "(sense %02x/%02x/%02x) - the disc's region does not "
                     "match the drive's; this is not an authentication "
                     "problem and no amount of key exchange will fix it",
                     token, entry->vid, entry->pid, cdb[0],
                     sense_key, asc, ascq);
                result = LASER_ERR_REGION;
                break;
            default:
                LOGW("token=%d: cdb 0x%02x refused, unassigned copy-protection "
                     "qualifier (sense %02x/%02x/%02x)",
                     token, cdb[0], sense_key, asc, ascq);
                result = LASER_ERR_REFUSED;
                break;
            }
            reason_logged = 1;
            break;
        }

        /* Permanently refused: the drive will answer the same way every
         * time, and six delayed retries per read would turn a clean error
         * into what a user experiences as a hang. */
        if (sense_key == SCSI_SENSE_KEY_ILLEGAL_REQUEST ||
            sense_key == SCSI_SENSE_KEY_DATA_PROTECT) {
            LOGW("token=%d: cdb 0x%02x permanently refused "
                 "(sense %02x/%02x/%02x), not retrying",
                 token, cdb[0], sense_key, asc, ascq);
            reason_logged = 1;
            result = LASER_ERR_REFUSED;
            break;
        }

        /* A non-idempotent command the drive may have acted on: stop here,
         * unless its CBW never went out. Only here, after the sense has been
         * read and classified above: REQUEST SENSE replays nothing, and
         * without it a refused key command - a region mismatch during the
         * handshake, a disc gone - reached libdvdcss as a bare I/O error. */
        if (!idempotent && rc != BOT_FAIL_NOT_SENT) {
            LOGW("token=%d: key command 0x%02x failed after its CBW was sent "
                 "(sense %02x/%02x/%02x), not retrying - the drive's state may "
                 "have advanced", token, cdb[0], sense_key, asc, ascq);
            reason_logged = 1;
            result = LASER_ERR_IO;
            break;
        }

        if (asc == SCSI_ASC_MEDIUM_MAY_HAVE_CHANGED &&
            ascq == SCSI_ASCQ_FORMAT_LAYER_CHANGED) {
            /* A dual-layer DVD crossing layers, around the middle of a film:
             * the read that follows normally succeeds, so the ordinary
             * delayed retry below applies. */
            LOGW("token=%d: format layer changed (dual-layer boundary), "
                 "retrying", token);
        }

        /* UNIT ATTENTION clears itself by being reported: retry at once.
         * Anything else - not ready, a transport error, an unexpected sense -
         * gets the delayed retry. */
        if (sense_key == SCSI_SENSE_KEY_UNIT_ATTENTION) {
            continue;
        }

        if (attempt < LASER_MAX_RETRIES) {
            usleep(LASER_RETRY_DELAY_MS * 1000);
        }
    }

    /* Every attempt failed before the CBW left: the device is gone, whatever
     * libusb called it. A descriptor closed under the handle, or a controller
     * tearing the URB down before marking the device absent, gives
     * LIBUSB_ERROR_IO rather than NO_DEVICE, which one attempt cannot tell
     * from a bridge having a bad moment - but a device that will not take 31
     * bytes on any of LASER_MAX_RETRIES attempts is not there. Never over a
     * cancellation, which the caller asked for. */
    if (result != LASER_OK && result != LASER_ERR_CANCELLED &&
        attempts_made > 0 && cbw_never_sent == attempts_made) {
        LOGW("token=%d: cdb 0x%02x - the CBW could not be sent on any of "
             "%d attempt(s); treating the device as gone",
             token, cdb[0], attempts_made);
        entry->device_gone = 1;
        result = LASER_ERR_NO_DEVICE;
        reason_logged = 1;
    }

    /* Normalise into the public contract: a status with a specific meaning
     * to the caller is kept, everything else becomes LASER_ERR_IO.
     *
     * THIS LIST MUST GROW WITH EVERY NEW TERMINAL STATUS. One missing is
     * silently flattened into LASER_ERR_IO while its own branch has already
     * logged it correctly, so the log looks right while the caller gets
     * something it cannot act on. */
    if (result != LASER_OK &&
        result != LASER_ERR_MEDIA_GONE &&
        result != LASER_ERR_NO_DEVICE &&
        result != LASER_ERR_REFUSED &&
        result != LASER_ERR_SCRAMBLED &&
        result != LASER_ERR_NO_KEY &&
        result != LASER_ERR_REGION &&
        result != LASER_ERR_CANCELLED) {
        /* The non-idempotent path has logged its own reason. */
        if (idempotent && !reason_logged) {
            LOGW("token=%d: cdb 0x%02x failed after %d attempt(s) "
                 "(last sense %02x/%02x/%02x)",
                 token, cdb[0], attempts_made,
                 last_sense_key, last_asc, last_ascq);
        }
        result = LASER_ERR_IO;
    }

    pthread_mutex_unlock(&entry->io_lock);
    return result;
}

/* ============================================================================
 * DVD region (laser_region_mismatch)
 * ========================================================================= */

/* Region masks, in both structures, are "one bit per region, SET means
 * PROHIBITED": a region-1 disc reads 0xFE, a drive set to region 2 reads
 * 0xFD. So the regions something permits are the complement, and disc and
 * drive agree exactly when their permitted sets intersect. */
#define LASER_REGION_ALL        0xFF

/* Byte 4 >> 6 of the RPC state. 0: no region set yet, which is no mismatch -
 * the drive adopts the first disc's region by itself. */
#define LASER_RPC_TYPE_NONE     0x00

/* Byte 6 of the RPC state. 0 is RPC-1: the drive enforces nothing. */
#define LASER_RPC_SCHEME_NONE   0x00

/* REPORT KEY, key format 08h: the drive's RPC state.
 *
 * @return 1 and the three fields set, 0 if the drive would not answer -
 *         which includes every non-DVD drive and is not an error here. */
static int report_rpc_state(int token, uint8_t *type, uint8_t *mask,
                            uint8_t *scheme)
{
    uint8_t cdb[12] = { 0 };
    uint8_t data[8] = { 0 };
    int actual = 0;

    cdb[0]  = 0xA4;                  /* REPORT KEY */
    cdb[8]  = sizeof(data) >> 8;     /* allocation length, big-endian */
    cdb[9]  = sizeof(data) & 0xFF;
    cdb[10] = 0x08;                  /* key format: RPC state. No AGID for
                                      * this format: the drive allocates
                                      * nothing. */

    if (laser_scsi_cdb(token, cdb, sizeof(cdb), data, sizeof(data), 1,
                       &actual) != LASER_OK
     || actual < (int)sizeof(data)) {
        return 0;
    }

    *type   = data[4] >> 6;
    *mask   = data[5];
    *scheme = data[6];
    return 1;
}

/* READ DVD STRUCTURE, format 01h: the disc's copyright information, whose
 * second byte is the Region Management Information.
 *
 * @return 1 and the mask set, 0 if the medium is not a DVD or carries no
 *         copyright structure. */
static int read_disc_region_mask(int token, uint8_t *rmi)
{
    uint8_t cdb[12] = { 0 };
    uint8_t data[8] = { 0 };
    int actual = 0;

    cdb[0] = 0xAD;                   /* READ DVD STRUCTURE */
    cdb[6] = 0x00;                   /* layer 0 */
    cdb[7] = 0x01;                   /* format: copyright information */
    cdb[8] = sizeof(data) >> 8;
    cdb[9] = sizeof(data) & 0xFF;

    if (laser_scsi_cdb(token, cdb, sizeof(cdb), data, sizeof(data), 1,
                       &actual) != LASER_OK
     || actual < (int)sizeof(data)) {
        return 0;
    }

    *rmi = data[5];
    return 1;
}

int laser_region_mismatch(int token, uint8_t *drive_mask, uint8_t *disc_mask)
{
    uint8_t type, dr_mask, scheme, di_mask;

    if (!report_rpc_state(token, &type, &dr_mask, &scheme)) {
        return 0;                    /* not a DVD drive, or would not say */
    }

    if (scheme == LASER_RPC_SCHEME_NONE || type == LASER_RPC_TYPE_NONE) {
        return 0;                    /* enforces nothing, or nothing set yet */
    }

    if (!read_disc_region_mask(token, &di_mask)) {
        return 0;                    /* not a DVD, or no copyright structure */
    }

    if (di_mask == 0x00) {
        return 0;                    /* region-free disc: plays anywhere */
    }

    /* Permitted sets are the complements; they agree if they intersect. */
    if ((uint8_t)(~dr_mask & ~di_mask & LASER_REGION_ALL) != 0) {
        return 0;
    }

    /* Written only on the answer that makes them meaningful - see laser.h. */
    if (drive_mask != NULL) {
        *drive_mask = dr_mask;
    }
    if (disc_mask != NULL) {
        *disc_mask = di_mask;
    }

    LOGI("token=%d: region mismatch, drive mask 0x%02x, disc mask 0x%02x",
         token, dr_mask, di_mask);
    return 1;
}

int laser_status_is_positional(int status)
{
    /* In the file that produces these values, so that whoever adds a status
     * finds this list beside it. */
    switch (status) {
    case LASER_ERR_SCRAMBLED:
    case LASER_ERR_REGION:
    case LASER_ERR_REFUSED:
        return 1;
    default:
        /* Including every non-negative value: a block count is a successful
         * read, not a status. */
        return 0;
    }
}

/* ============================================================================
 * Public: LBA-aware chunked block reads
 * ============================================================================ */

/* Build the CDB for one chunk - the only thing the two public helpers below
 * do differently. */
typedef void (*build_read_cdb_fn)(uint8_t *cdb, uint32_t lba, int blocks,
                                  const void *ctx);

/* Lower this device's transfer cap to @p bytes, if it is above that. Called
 * once smaller transfers have read the whole range a @p failed_bytes one
 * failed on. */
static void lower_transfer_cap(laser_entry_t *entry, int bytes,
                               int failed_bytes)
{
    pthread_mutex_lock(&entry->io_lock);

    const int lowered = bytes < entry->max_transfer_bytes;
    if (lowered) {
        entry->max_transfer_bytes = bytes;
    }

    pthread_mutex_unlock(&entry->io_lock);

    if (lowered) {
        LOG_QUIRK(entry, "a %d-byte transfer failed where %d-byte ones read "
                         "the same blocks, limiting this device to %d bytes "
                         "per command from now on",
                  failed_bytes, bytes, bytes);
    }
}

/* The body shared by laser_read_blocks() and laser_read_cd_blocks(): reads
 * @p num_blocks blocks from @p lba into @p buffer, one command per chunk that
 * fits the device's transfer cap.
 *
 * Returns:
 *
 *   > 0  blocks read, contiguous from @p lba. May be fewer than asked: a
 *        short read is the ordinary answer for a scratched sector, and
 *        callers advance by what came back and ask for the rest.
 *   = 0  only when @p num_blocks was <= 0. A read that brought back nothing
 *        is LASER_ERR_IO, since 0 would make those callers ask again
 *        forever.
 *   < 0  a laser_status_t. Whatever is already in @p buffer must be
 *        ignored, including the chunks that succeeded.
 *
 * MEDIA_GONE, NO_DEVICE and CANCELLED are returned even after successful
 * chunks: there is nothing left to read around, or nobody left asking, and a
 * caller given a short read would keep walking a drive that cannot answer -
 * dvdread and dvdnav treat a short read as an ordinary disc imperfection.
 * Any other failure becomes a short read once something has been read.
 *
 * TRANSFER-SIZE NEGOTIATION. Some USB-ATAPI bridges fail a large data phase
 * outright, where the same blocks read in smaller commands go through. So a
 * plain I/O failure on more than LASER_MIN_BYTES_PER_TRANSFER is not final:
 * the same blocks are read again at half the size, halving again on each
 * failure. Only once the whole failed range has been read that way is the
 * smaller size kept for the device: the large command failed on blocks that
 * are readable, so the limit was the bridge's. A scratch inside the range
 * fails the smaller reads too, and leaves the cap alone.
 *
 * This terminates: every retry is strictly smaller than the transfer that
 * failed, and none goes below LASER_MIN_BYTES_PER_TRANSFER.
 *
 * @param token      registry token; the caller must hold a claim on it
 * @param lba        first block to read
 * @param num_blocks how many to read; <= 0 reads nothing and succeeds
 * @param block_size bytes per block, which also sets how many fit in one
 *                   transaction
 * @param cdb_len    10 for READ(10), 12 for READ CD
 * @param build_cdb  fills a zeroed 16-byte CDB for one chunk
 * @param ctx        handed to @p build_cdb untouched
 * @param buffer     at least @p num_blocks * @p block_size bytes
 * @param what       what to call these in a log line ("blocks", "sectors")
 */
static int read_chunked(int token, uint32_t lba, int num_blocks,
                        int block_size, int cdb_len,
                        build_read_cdb_fn build_cdb, const void *ctx,
                        uint8_t *buffer, const char *what)
{
    if (num_blocks <= 0) {
        return 0;
    }

    /* For the transfer cap. A miss costs only the negotiation:
     * laser_scsi_cdb() looks the token up again and reports it. */
    laser_entry_t *entry = laser_lookup(token);
    int blocks_done = 0;

    /* A negotiation in progress: trial_cap is the size being tried, 0 when
     * there is none; trial_end is the block index where the failed range
     * ends, and trial_failed the size that failed on it. */
    int trial_cap = 0;
    int trial_end = 0;
    int trial_failed = 0;

    while (blocks_done < num_blocks) {
        /* Re-read every time round: another thread on the same drive may
         * have lowered it. */
        int cap = entry != NULL ? entry->max_transfer_bytes
                                : LASER_MAX_BYTES_PER_TRANSFER;
        if (trial_cap > 0 && trial_cap < cap) {
            cap = trial_cap;
        }

        int max_blocks_per_chunk = cap / block_size;
        if (max_blocks_per_chunk < 1) {
            /* One block is the smallest read that makes progress. */
            max_blocks_per_chunk = 1;
        }

        int chunk = num_blocks - blocks_done;
        if (chunk > max_blocks_per_chunk) {
            chunk = max_blocks_per_chunk;
        }

        uint8_t cdb[16];
        memset(cdb, 0, sizeof(cdb));
        build_cdb(cdb, lba + (uint32_t)blocks_done, chunk, ctx);

        int actual_len = 0;
        laser_status_t st = laser_scsi_cdb(
                token, cdb, cdb_len,
                buffer + (size_t)blocks_done * block_size,
                chunk * block_size, /* data_in = */ 1, &actual_len);

        if (st != LASER_OK) {
            if (st == LASER_ERR_MEDIA_GONE || st == LASER_ERR_NO_DEVICE ||
                st == LASER_ERR_CANCELLED) {
                return (int)st;
            }

            /* Maybe the bridge rather than the disc: try the same blocks at
             * half the size. The first failure of a negotiation sets the
             * range it must cover. */
            const int tried = chunk * block_size;
            if (st == LASER_ERR_IO && chunk > 1 && entry != NULL &&
                tried > LASER_MIN_BYTES_PER_TRANSFER) {
                if (trial_cap == 0) {
                    trial_end = blocks_done + chunk;
                    trial_failed = tried;
                }
                trial_cap = tried / 2;
                if (trial_cap < LASER_MIN_BYTES_PER_TRANSFER) {
                    trial_cap = LASER_MIN_BYTES_PER_TRANSFER;
                }
                continue;
            }

            return blocks_done > 0 ? blocks_done : (int)st;
        }

        if (actual_len != chunk * block_size) {
            /* Short read: what was read so far, as a real block device
             * returns on an imperfect disc - unless that is nothing, see the
             * contract above. */
            int short_total = blocks_done + actual_len / block_size;
            if (short_total == 0) {
                /* Both lengths are logged: a few hundred bytes short would be
                 * a drive returning a smaller sector than asked - 2072 rather
                 * than 2352, a raw Mode 2 Form 1 sector without its EDC/ECC -
                 * not a scratch. */
                LOGW("token=%d: read of %d %s at LBA %u returned no usable "
                     "data (%d of %d bytes)",
                     token, num_blocks, what, lba, actual_len,
                     chunk * block_size);
                return (int)LASER_ERR_IO;
            }
            return short_total;
        }

        blocks_done += chunk;

        if (trial_cap > 0 && blocks_done >= trial_end) {
            lower_transfer_cap(entry, trial_cap, trial_failed);
            trial_cap = 0;
        }
    }

    return blocks_done;
}

static void build_read10_cdb(uint8_t *cdb, uint32_t lba, int blocks,
                             const void *ctx)
{
    (void) ctx;

    cdb[0] = 0x28; /* READ(10) */
    cdb[2] = (uint8_t)(lba >> 24);
    cdb[3] = (uint8_t)(lba >> 16);
    cdb[4] = (uint8_t)(lba >> 8);
    cdb[5] = (uint8_t)(lba);
    cdb[7] = (uint8_t)(blocks >> 8);
    cdb[8] = (uint8_t)(blocks);
}

int laser_read_blocks(int token, uint32_t lba, int num_blocks,
                      uint8_t *buffer)
{
    return read_chunked(token, lba, num_blocks, 2048, 10,
                        build_read10_cdb, NULL, buffer, "blocks");
}

/* Bytes 1 and 9 of READ CD, resolved from the sector kind before the loop
 * starts - see laser_read_cd_blocks() below for what each pairing means. */
typedef struct {
    uint8_t expected_type;
    uint8_t field_flags;
} read_cd_ctx_t;

static void build_read_cd_cdb(uint8_t *cdb, uint32_t lba, int blocks,
                              const void *ctx)
{
    const read_cd_ctx_t *rc = ctx;

    cdb[0] = 0xBE; /* READ CD */
    cdb[1] = rc->expected_type;
    cdb[2] = (uint8_t)(lba >> 24);
    cdb[3] = (uint8_t)(lba >> 16);
    cdb[4] = (uint8_t)(lba >> 8);
    cdb[5] = (uint8_t)(lba);
    cdb[6] = (uint8_t)(blocks >> 16);
    cdb[7] = (uint8_t)(blocks >> 8);
    cdb[8] = (uint8_t)(blocks);
    cdb[9] = rc->field_flags;
}

int laser_read_cd_blocks(int token, uint32_t lba, int num_blocks,
                         laser_cd_sector_t sector_type, uint8_t *buffer)
{
    /* The same for every sector kind - see laser_cd_sector_t in laser.h. */
    const int block_size = 2352;

    /* Byte 1 of the CDB says what the drive should expect to find, byte 9
     * which fields to send back.
     *
     *   AUDIO: Expected Sector Type CD-DA (001b), User Data only. A CD-DA
     *     sector has no sync, header or EDC/ECC - its 2352 bytes are the
     *     user data - and a drive may refuse a request for them with INVALID
     *     FIELD IN CDB, as some do. This is the form CD-DA extractors use. If
     *     a drive ever refuses it, try type "any" (0x00) with User Data only
     *     (0x10).
     *
     *   MODE2_FORM2 and ANY: sync, headers, user data and EDC/ECC (0xF8),
     *     which a Mode 2 sector does have; they differ only in the expected
     *     type, Form 2 (101b) or "all types" (000b).
     *
     *     0xF8 rather than 0xF0, for the LENGTH of the EDC, which no caller
     *     reads:
     *
     *         0xF0, Mode 2 Form 1:  12 + 4 + 8 + 2048        = 2072
     *         0xF0, Mode 2 Form 2:  12 + 4 + 8 + 2324        = 2348
     *         0xF8, Mode 2 Form 1:  12 + 4 + 8 + 2048 + 280  = 2352
     *         0xF8, Mode 2 Form 2:  12 + 4 + 8 + 2324 + 4    = 2352
     *
     *     Only 0xF8 makes both forms 2352 bytes, the stride cdrom.c reads
     *     with. Under 0xF0, a bridge reporting lengths honestly packs Form 2
     *     sectors at 2348, misplacing sector n by 4n bytes and desynchronising
     *     the MPEG stream. */
    uint8_t expected_type;
    uint8_t field_flags;

    switch (sector_type) {
        case LASER_CD_SECTOR_AUDIO:
            expected_type = 0x04;
            field_flags   = 0x10;
            break;

        case LASER_CD_SECTOR_MODE2_FORM2:
            expected_type = 0x14;
            field_flags   = 0xF8;
            break;

        case LASER_CD_SECTOR_ANY:
            expected_type = 0x00;
            field_flags   = 0xF8;
            break;

        default:
            /* A caller bug, rejected before the device is touched, as in
             * laser_scsi_cdb(). */
            LOGE("token=%d: laser_read_cd_blocks: unknown sector type %d",
                 token, (int)sector_type);
            return (int)LASER_ERR_INVALID;
    }

    read_cd_ctx_t ctx = {
        .expected_type = expected_type,
        .field_flags   = field_flags,
    };

    return read_chunked(token, lba, num_blocks, block_size, 12,
                        build_read_cd_cdb, &ctx, buffer, "sectors");
}
