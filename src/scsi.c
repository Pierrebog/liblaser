/*****************************************************************************
 * scsi.c: SCSI-MMC on top of the Bulk-Only transport
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
 * Everything that needs to know what a command MEANS: sense-code
 * classification, the retry policy keyed on what the command actually is,
 * the probes that run once at registration (INQUIRY, GET MAX LUN, TEST UNIT
 * READY), and the LBA-aware chunked read helpers.
 *
 * Every transaction it issues goes through laser_bot_send_locked() in bot.c,
 * which knows the wire format and nothing else. The resilience this file
 * adds is centralized here so that none of this project's ~15 patched call
 * sites (libdvdcss's ioctl.c, VLC's cdda/vcd modules through cdrom.c, the
 * laser access module) needs to know about it.
 *****************************************************************************/

#include <string.h>
#include <time.h>
#include <unistd.h>

#include "laser.h"
#include "laser_internal.h"

/* Fixed-format sense data (SCSI SPC): byte 2 low nibble is the sense key,
 * byte 12 is the Additional Sense Code (ASC), byte 13 is the Additional
 * Sense Code Qualifier (ASCQ). Values below checked against T10's
 * published ASC/ASCQ assignment list (t10.org/lists/asc-num.txt).
 *
 * Note on ASC-only vs ASC+ASCQ matching: an ASC groups related
 * conditions and its ASCQ distinguishes them, so whether the qualifier
 * has to be checked depends on whether every ASCQ under that ASC means
 * the same thing to us:
 *
 *   3Ah (MEDIUM NOT PRESENT): 3Ah/00h through 3Ah/04h are all "there is
 *        no disc" (tray closed, tray open, loadable...). They differ in
 *        detail we have no use for, so matching on the ASC alone is
 *        correct and deliberate.
 *
 *   28h: NOT the same story. 28h/00h is the medium-changed condition we
 *        want, but 28h/02h is FORMAT-LAYER MAY HAVE CHANGED, a C/DVD
 *        specific code a drive reports when crossing between layers of a
 *        dual-layer disc - an ordinary event in the middle of playing a
 *        DVD-9, not an ejection. It must be matched on ASC+ASCQ. */
#define SCSI_SENSE_KEY_NOT_READY        0x02
#define SCSI_SENSE_KEY_UNIT_ATTENTION   0x06

/* Sense keys that a retry can never turn into a success. ILLEGAL REQUEST
 * means the drive understood the command and refuses it as issued - a
 * malformed CDB, an LBA past the end of the medium, or a read it will
 * not serve in its current state, such as a CSS-scrambled sector before
 * authentication. DATA PROTECT is the same answer for a different
 * reason. Neither becomes true by waiting. */
#define SCSI_SENSE_KEY_ILLEGAL_REQUEST  0x05
#define SCSI_SENSE_KEY_DATA_PROTECT     0x07
/* 6Fh COPY PROTECTION KEY EXCHANGE FAILURE and neighbours - six distinct
 * conditions under one ASC, with three genuinely different remedies. They
 * all arrive with sense key 05h ILLEGAL REQUEST, so without the qualifier
 * they are indistinguishable from each other and from an ordinary refusal.
 *
 *   6Fh/00h AUTHENTICATION FAILURE
 *   6Fh/01h KEY NOT PRESENT
 *   6Fh/02h KEY NOT ESTABLISHED   -> re-authenticate; the sector is NOT
 *                                    permanently unreadable, and treating it
 *                                    as such poisons content that a fresh
 *                                    handshake would hand over.
 *   6Fh/03h READ OF SCRAMBLED SECTOR WITHOUT AUTHENTICATION
 *                                 -> genuinely unreadable as things stand.
 *   6Fh/04h MEDIA REGION CODE IS MISMATCHED TO LOGICAL UNIT REGION
 *   6Fh/05h DRIVE REGION MUST BE PERMANENT/REGION RESET COUNT ERROR
 *                                 -> nothing to do with authentication; the
 *                                    user must be told the truth, and the
 *                                    region must never be silently changed
 *                                    on their hardware.
 *
 * Checked against T10's published assignment list. */
#define SCSI_ASC_COPY_PROTECTION        0x6f
#define SCSI_ASCQ_CP_AUTH_FAILURE       0x00
#define SCSI_ASCQ_CP_KEY_NOT_PRESENT    0x01
#define SCSI_ASCQ_CP_KEY_NOT_ESTABLISHED 0x02
#define SCSI_ASCQ_CP_SCRAMBLED          0x03
#define SCSI_ASCQ_CP_REGION_MISMATCH    0x04
#define SCSI_ASCQ_CP_REGION_PERMANENT   0x05

#define SCSI_ASC_MEDIUM_NOT_PRESENT     0x3a
#define SCSI_ASC_MEDIUM_MAY_HAVE_CHANGED 0x28
/* 28h/00h NOT READY TO READY CHANGE, MEDIUM MAY HAVE CHANGED - the only
 * qualifier under ASC 28h that actually means the disc was swapped. */
#define SCSI_ASCQ_MEDIUM_MAY_HAVE_CHANGED 0x00
/* 28h/02h FORMAT-LAYER MAY HAVE CHANGED - dual-layer boundary crossing,
 * transient and retryable; explicitly NOT an ejection. */
#define SCSI_ASCQ_FORMAT_LAYER_CHANGED    0x02

/* Retry budget: matches the classification probe's already-validated
 * policy. A wider window (some SCSI-transport libraries for burning
 * software use ~30s) was considered and rejected as too slow for
 * interactive playback - see the project recap for the reasoning. */
#define LASER_MAX_RETRIES       6
#define LASER_RETRY_DELAY_MS    500

/* Spin-up wait budget, separate from and much larger than the per-command
 * retry budget above. The retry budget covers transient conditions that
 * clear in milliseconds (UNIT ATTENTION) or a fraction of a second
 * (becoming-ready between two reads). Spinning up a cold optical disc
 * from a full stop is mechanical and slow - commonly a few seconds, up to
 * ten or more on a sluggish drive or a marginal disc - so the first
 * contact with the device waits far longer, but only once per session.
 * 20 x 500ms = 10s, which cleared spin-up on the POC's test hardware with
 * margin to spare. */
#define LASER_SPINUP_MAX_ATTEMPTS  20
#define LASER_SPINUP_DELAY_MS      500

/* Hard ceiling on the wall-clock time the spin-up wait may consume,
 * whatever the attempt counter says.
 *
 * The attempt budget alone bounds nothing in real time. Its arithmetic
 * assumes each attempt costs one 500ms sleep plus two near-instant
 * commands, which holds while the drive is answering: 20 attempts, about
 * ten seconds, comfortably more than a cold DVD needs. It stops holding
 * the moment the drive stops answering without disappearing - wedged
 * firmware, a hub dropping transfers, a bridge that ignores its Bulk-In.
 * Each attempt then costs a TEST UNIT READY and a REQUEST SENSE that
 * each run out CBW_PHASE_TIMEOUT_MS + CSW_PHASE_TIMEOUT_MS, and twenty of
 * those is measured in minutes.
 *
 * Minutes is not merely slow, it is a hang: this runs inside
 * laser_register(), which holds the registry lock
 * for its whole duration - so a teardown blocks behind
 * it, and the caller cannot cancel it either, since registration runs to
 * completion once started.
 *
 * 15s leaves clear headroom over the ~10s the nominal path spends, so
 * this never fires on a merely slow drive, while capping the pathological
 * one at something a user experiences as a pause rather than a freeze. */
#define LASER_SPINUP_MAX_WALL_MS   15000

/* LASER_MAX_BYTES_PER_TRANSFER and LASER_MIN_BYTES_PER_TRANSFER are in
 * laser_internal.h: they bound laser_entry_t::max_transfer_bytes, which
 * registry.c initialises, so they belong with that field.
 * GET MAX LUN: Bulk-Only class request, Device-to-Host, returns one byte
 * holding the number of the LAST logical unit (so 0 means "one LUN"). A
 * device that does not support multiple LUNs is required to stall it. */
#define USB_BOT_GETMAXLUN_bREQUEST      0xFE
#define USB_BOT_GETMAXLUN_bmREQUESTTYPE 0xA1  /* Class | Interface | Dev-to-Host */

/* Highest LUN we will probe even if the device claims more. The class
 * allows up to 15; nothing that pairs an optical drive with a card
 * reader uses anything like that many, and each extra unit costs an
 * INQUIRY at registration time. */
#define LASER_MAX_LUN_PROBED 3

/* SCSI INQUIRY (SPC), byte 0 low 5 bits: peripheral device type. */
#define SCSI_PDT_DIRECT_ACCESS  0x00

/* INQUIRY attempts before concluding the device will not answer it. Small:
 * the command is mandatory and needs no medium, so a device that has not
 * served it by now is not going to. */
#define LASER_INQUIRY_MAX_ATTEMPTS 3
#define SCSI_PDT_MASK           0x1f
#define SCSI_PDT_CD_DVD         0x05

/* INQUIRY allocation length. 36 bytes is the standard short form every
 * device must answer, and the size already validated on real hardware by
 * the classification probe this transport grew out of. */
#define SCSI_INQUIRY_ALLOC_LEN  36

/* Forward declaration: defined further down, but the spin-up wait below
 * ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,needs it to interpret a TEST UNIT READY failure. */
static int request_sense_locked(laser_entry_t *entry,
                                uint8_t *sense_key, uint8_t *asc, uint8_t *ascq);

/* SCSI TEST UNIT READY (opcode 0x00): no data phase, the CSW status
 * alone says whether the unit is ready.
 *
 * Returns laser_bot_send_locked()'s code verbatim rather than
 * collapsing it to 0/-1: the spin-up wait below has to tell "the drive is
 * busy spinning up" from "there is no drive on the bus any more", and
 * flattening every failure into -1 threw that distinction away. 0 still
 * means ready; BOT_FAIL_NO_DEVICE means gone; anything else means not
 * ready yet.
 *
 * Caller MUST already hold entry->io_lock. */
static int test_unit_ready_locked(laser_entry_t *entry)
{
    uint8_t cdb[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    int csw_status = -1;
    return laser_bot_send_locked(entry, cdb, sizeof(cdb),
                                   NULL, 0, 0, NULL, &csw_status);
}

/* Milliseconds elapsed on CLOCK_MONOTONIC since *start. Monotonic
 * specifically: a wall-clock budget must not be lengthened or cut short
 * by the system clock being stepped, which on Android happens routinely
 * as NTP and the carrier's time settle after a boot or a network change. */
static long monotonic_ms_since(const struct timespec *start)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec - start->tv_sec) * 1000L
           + (now.tv_nsec - start->tv_nsec) / 1000000L;
}

/* Wake the drive and wait for its medium to become ready before any read
 * is attempted.
 *
 * This is the single most important thing the POC did that the module
 * initially did not: an optical drive that has spun its disc down (or
 * never spun it up since the disc was inserted) answers the very first
 * data-bearing command with NOT READY / becoming-ready, and - depending
 * on firmware - a READ that arrives cold may fail outright rather than
 * kick off the spin-up. A TEST UNIT READY is the command whose whole
 * purpose is to prod the unit and report its state; issuing it in a loop
 * both starts the mechanical spin-up and waits for it to finish.
 *
 * Mechanical spin-up of a cold DVD can take well over the transport's
 * ordinary per-command retry budget - several seconds, sometimes more
 * than ten - so this has its own, longer budget, separate from
 * LASER_MAX_RETRIES. That budget has two independent ceilings, and
 * whichever is reached first ends the wait:
 *
 *   - LASER_SPINUP_MAX_ATTEMPTS, which bounds how many times a
 *     drive that keeps answering "not yet" is asked again;
 *   - LASER_SPINUP_MAX_WALL_MS, which bounds the real time spent,
 *     and is what stops a drive that has stopped answering from turning
 *     the attempt budget into minutes of USB timeouts (see that
 *     constant's own comment - this function runs with the registry
 *     lock held, so those minutes block teardown too).
 *
 * Two conditions end it earlier still, because neither can be improved
 * on by waiting: MEDIUM NOT PRESENT, there being no disc; and the device
 * having left the bus altogether, which TEST UNIT READY now reports
 * distinctly rather than as one more "not ready".
 *
 * Best-effort: a drive that never reports ready still falls through to
 * the caller, which will find out soon enough when its first real read
 * fails. Runs on LUN 0 (entry->lun before discovery); that is enough to
 * spin the mechanism up, and LUN discovery's own INQUIRY follows. */
/* GET EVENT STATUS NOTIFICATION (opcode 4Ah), media event class, polled.
 * Purely diagnostic: it changes nothing, it only reports what the drive
 * believes about the tray and the medium.
 *
 * TEST UNIT READY answers "not ready" without saying why, and REQUEST SENSE
 * reports MEDIUM NOT PRESENT both for an open tray and for a bridge that has
 * lost track of a disc that is physically there. 4Ah separates the two: it
 * carries a tray-open bit, a media-present bit, and the event code of the
 * last media change. Caller MUST already hold entry->io_lock. */
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

void laser_wait_until_ready(laser_entry_t *entry)
{
    struct timespec started;
    clock_gettime(CLOCK_MONOTONIC, &started);

    pthread_mutex_lock(&entry->io_lock);

    /* NOT CANCELLABLE, and it cannot be: this runs inside laser_acquire(),
     * under g_registry_lock, on an entry that has not been published yet - so
     * nothing can look the token up, and cancellation is set only by the
     * laser_release() that would have to take that same lock to get here.
     *
     * What bounds it is LASER_SPINUP_MAX_WALL_MS, checked below. */
    for (int attempt = 1; attempt <= LASER_SPINUP_MAX_ATTEMPTS; attempt++) {
        int rc = test_unit_ready_locked(entry);

        if (rc == 0) {
            LOGI("token=%d: unit ready (attempt %d/%d, %ldms)",
                 entry->token, attempt, LASER_SPINUP_MAX_ATTEMPTS,
                 monotonic_ms_since(&started));
            pthread_mutex_unlock(&entry->io_lock);
            return;
        }

        if (rc == BOT_FAIL_NO_DEVICE) {
            /* Not a drive that needs more time - a drive that is not
             * there. Every remaining attempt would spend two full sets of
             * USB timeouts confirming it. Stop at once; registration goes
             * on to fail, or the first real command does, which is the
             * honest outcome either way. */
            LOGW("token=%d: device gone during spin-up wait, abandoning it",
                 entry->token);
            pthread_mutex_unlock(&entry->io_lock);
            return;
        }

        uint8_t sense_key = 0xff, asc = 0, ascq = 0;
        request_sense_locked(entry, &sense_key, &asc, &ascq);

        /* Logged raw rather than only through the branches below: 02h/3Ah
         * (no medium), 02h/04h/01h (becoming ready) and 06h/28h/00h (medium
         * may have changed) demand different handling, and telling them apart
         * from the aggregated messages alone is guesswork. */
        LOGI("token=%d: sense %02x/%02x/%02x (attempt %d/%d)",
             entry->token, sense_key, asc, ascq, attempt,
             LASER_SPINUP_MAX_ATTEMPTS);

        if (sense_key == SCSI_SENSE_KEY_NOT_READY &&
            asc == SCSI_ASC_MEDIUM_NOT_PRESENT) {
            LOGW("token=%d: no disc present (attempt %d/%d), giving up wait",
                 entry->token, attempt, LASER_SPINUP_MAX_ATTEMPTS);
            /* Ask the drive what it thinks the tray and the medium are
             * doing, so the give-up is recorded with a reason rather than
             * only with a verdict. */
            log_media_status_locked(entry);
            pthread_mutex_unlock(&entry->io_lock);
            return;
        }

        /* Wall-clock check before committing to another round, so the
         * ceiling bounds the time actually spent rather than being
         * noticed one full attempt late. Placed after the sense checks so
         * that a conclusive answer - no disc - still ends the wait on its
         * own terms rather than as a timeout. */
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

/* One INQUIRY on the unit currently selected by entry->lun.
 *
 * UNDER THE PROBE TIMEOUTS, always, and that is why selecting the unit and
 * classifying it are one function rather than two. At the READ timeouts a
 * multi-LUN device with an absent unit spends eleven seconds per LUN inside
 * registration, with the registry lock held.
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

        LOGI("token=%d: LUN %u INQUIRY unanswered (rc=%d, %d bytes, "
             "attempt %d/%d)",
             entry->token, entry->lun, rc, actual, attempt, attempts);
    }

    return -1;
}

int laser_probe_lun(laser_entry_t *entry)
{
    /* Default, and the answer for almost every drive. Set before anything
     * else so that every early return below leaves a usable value behind. */
    entry->lun = 0;

    unsigned char max_lun_buf = 0;
    int ret = libusb_control_transfer(entry->handle,
                                      USB_BOT_GETMAXLUN_bmREQUESTTYPE,
                                      USB_BOT_GETMAXLUN_bREQUEST,
                                      0, entry->iface_num,
                                      &max_lun_buf, 1, 3000);
    int max_lun = 0;
    if (ret != 1) {
        /* Stalled or otherwise unanswered. The class specification is
         * explicit that this means a single logical unit, so this is a
         * normal outcome and not worth warning about - most drives take
         * this path. */
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

    /* Ask each unit what it is and take the first CD/DVD one. INQUIRY is
     * addressed per-LUN through the CBW, so entry->lun is what selects the
     * target - it is restored to a sane value on every exit path below.
     *
     * ONLY LUN 0 IS RETRIED. The retry exists because the first attempt lands
     * right after the Mass Storage Reset and is the first real command the
     * device sees: a bridge still settling answers it with a Phase Error and
     * the next one with data. That reason applies to the first command and to
     * no other, so spending it again on units 1..3 - which are usually absent
     * on the drives that report them - would multiply the cost of the case
     * this is trying to keep cheap. */
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

    /* Nothing identified itself as optical. Fall back to LUN 0 rather than to
     * whatever the loop happened to leave behind: INQUIRY data is not always
     * truthful, and LUN 0 at least matches how this transport behaved before
     * LUN discovery existed. */
    entry->lun = 0;

    if (!have_lun0_pdt) {
        /* Nothing answered at all. NOT reported as "not optical": a device
         * that cannot be asked has not answered no, and rejecting on silence
         * would make a merely slow drive silently unusable. Reported as its
         * own outcome so the caller can stop treating this device as one that
         * might just be warming up - INQUIRY needs no medium, so failing it
         * means the device is not talking at all. */
        LOGI("token=%d: INQUIRY unanswered on every unit", entry->token);
        return LASER_OPTICAL_NO_ANSWER;
    }

    if (max_lun > 0) {
        LOG_QUIRK(entry, "multi-LUN device but none identified as optical, "
                         "falling back to LUN 0");
    }

    /* Only the block-device type is rejected, not "everything that is not
     * 0x05". Combo enclosures pair an optical drive with a card reader behind
     * two bridges that are indistinguishable at the USB descriptor level, and
     * the card reader is exactly what answers 0x00 here - as does a USB key
     * or an external disk, both of which Android already handles through its
     * own storage path.
     *
     * A drive whose bridge reports some other unexpected type is still
     * accepted, because INQUIRY data is not always truthful - the same reason
     * the fallback above prefers LUN 0 to a negative result - and losing a
     * real optical drive to a lying descriptor would be a far worse failure
     * than probing one device too many. */
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
 * Public: laser_scsi_cdb() - the generic, retrying, mutex-protected
 * CDB primitive every higher-level helper and every patched call site
 * ultimately goes through.
 * ============================================================================ */

laser_status_t laser_scsi_cdb(int token,
                              const uint8_t *cdb, int cdb_len,
                              uint8_t *data, int data_len,
                              int data_in, int *actual_len)
{
    /* Argument validation first, before anything expensive. Getting this
     * wrong is a caller bug, and the failures it produces are all far
     * away from their cause: an oversized cdb_len overflows a stack
     * struct inside the CBW builder, and a NULL data with a non-zero
     * data_len announces a data phase in the CBW that then never
     * happens - the drive waits for bytes that never come, stalls, and
     * the transaction ends in a Reset Recovery with nothing in the log
     * pointing at the real mistake.
     *
     * Checked here rather than deeper down because this is the boundary
     * the contract in laser.h describes, and because a bad call
     * must not trigger the one-time device registration below - a
     * malformed command is no reason to spend fifteen seconds spinning a
     * drive up. */
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
        /* Warned about, not rejected. The transfer may well succeed -
         * libusb splits it to satisfy the OS - and refusing a call that
         * would have worked is worse than letting it through. But it is
         * outside what this transport keeps within reliably across
         * Android host controllers, and the block helpers below exist
         * precisely so no caller has to do this. */
        LOGW("token=%d: %d-byte data phase exceeds the %d-byte per-transfer "
             "budget; prefer laser_read_blocks()/read_cd_blocks()",
             token, data_len, LASER_MAX_BYTES_PER_TRANSFER);
    }

    /* lookup(), and NOT a lookup-or-register. A command on an unclaimed token
     * answers exactly as one on a token that was never valid: NO_SUCH_TOKEN,
     * immediately, with nothing set up.
     *
     * Registering here instead would leave no way back. The registration
     * would belong to nobody, teardown only ever happens when a claim count
     * falls to zero, and a count never incremented never falls: the entry
     * would hold a libusb handle and a table slot until the process died, and
     * once its owner closed the descriptor and the OS recycled the number,
     * that stale entry would answer for a different device. Every
     * consumer in this project calls laser_acquire() first - the access
     * module in both Open paths, cdrom.c in ioctl_Open(), libdvdcss in
     * dvdcss_open() - so what this catches is a caller that forgets. */
    laser_entry_t *entry = laser_lookup(token);
    if (entry == NULL) {
        LOGW("token=%d: no such token - is a claim held? "
             "(laser_acquire() is what registers a device)", token);
        return LASER_ERR_NO_SUCH_TOKEN;
    }

    /* Does this command change the drive's CSS authentication state? Keyed on
     * the key FORMAT, not the opcode - see laser_cdb_changes_css_state().
     * Two decisions need exactly this answer: whether a session is required,
     * and whether a retry is safe. */
    const int css_state_changing = laser_cdb_changes_css_state(cdb, cdb_len);

    /* A state-changing key command outside any session is refused, not fixed
     * up.
     *
     * Making the session implicit here was the alternative: take the session
     * around this one command and drop it after. It reads as the smaller
     * change and it is worthless - exclusion spanning one transaction IS
     * io_lock, two lines below. What needs protecting is the SEQUENCE, and
     * this layer cannot see sequences: the first command of a handshake and a
     * stray AGID request from an unrelated thread are the same bytes. Only the
     * consumer can tell them apart, so it declares a session and this checks
     * the declaration was made.
     *
     * Deliberately not "is the session MINE": one consumer's key commands
     * legitimately arrive from several threads, and the cookie that identifies
     * it does not reach this far. What this stops is an UNDECLARED consumer
     * stealing an AGID, which is the accident actually observed.
     *
     * Runs before io_lock - see the lock order in registry.c. */
    if (css_state_changing && !laser_css_session_is_open(entry)) {
        LOGE("token=%d, usb %04x:%04x: CSS command 0x%02x issued with no "
             "session open - refused. Call laser_css_session_begin() "
             "first; see the session contract in laser.h",
             token, entry->vid, entry->pid, cdb[0]);
        return LASER_ERR_IO;
    }

    /* Cancelled before we even queue for the device. Checked here rather than
     * only inside the loop so that a command arriving after teardown began
     * does not first wait out whatever transaction is currently holding
     * io_lock - which on the drive this exists for is the slow one. */
    if (laser_is_cancelled(entry)) {
        return LASER_ERR_CANCELLED;
    }

    /* Already known to be gone. Answered here, before io_lock and before
     * libusb, because everything below this point would spend the whole retry
     * budget rediscovering it - see laser_entry_t::device_gone. */
    if (entry->device_gone) {
        return LASER_ERR_NO_DEVICE;
    }

    pthread_mutex_lock(&entry->io_lock);

    laser_status_t result = LASER_ERR_IO;

    /* Is this command safe to send twice? DATA-IN commands here are all
     * reads (READ(10), READ CD, READ TOC, REPORT KEY, READ DVD
     * STRUCTURE): re-issuing one returns the same data and leaves the
     * drive where it was. DATA-OUT means SEND KEY, which is a step in
     * the CSS authentication handshake - an explicit state machine in
     * the drive (request AGID, host challenge, key1, drive challenge,
     * key2), where each accepted command advances the logical unit to
     * the next state. Replaying a step the drive already accepted
     * desynchronises host and drive, and the resulting failure surfaces
     * later and elsewhere, as an unexplained authentication error.
     *
     * libdvdcss already owns recovery for exactly this situation - it
     * resets a hung authentication by invalidating the AGID and
     * restarting the handshake from the top. That is the layer with the
     * context to do it correctly; this one does not have it, so the
     * right thing here is to report the failure and let the CSS code
     * above decide, rather than blindly re-sending.
     *
     * NOT KEYED ON DIRECTION, which is the trap here: three of that state
     * machine's five steps (request AGID, report key1, report drive
     * challenge) are REPORT KEY, i.e. DATA-IN, so a direction test would mark
     * them retryable. The worst case is not desynchronisation but
     * EXHAUSTION: REPORT KEY format 00h is DATA-IN with
     * an 8-byte data phase, and every accepted one ALLOCATES one of the
     * drive's four AGIDs, so a single call retried through a UNIT ATTENTION
     * could consume all four and leave the drive unable to authenticate
     * anything until physically unplugged.
     *
     * Keyed on the same predicate as the session check, so that read-only key
     * commands - copyright, RPC state, ASF - keep their retries. They are how
     * libdvdcss decides whether the disc is scrambled at all, and losing a
     * retry there would turn a transient into "CSS disabled for this disc". */
    const int idempotent = !css_state_changing && (data_in || data_len == 0);

    /* Last sense data seen, kept across attempts so the final failure can
     * report WHY rather than only that it happened. Without it the log
     * said "command failed after 6 attempts" and nothing more, which
     * cannot separate a scratched sector from an out-of-range LBA from a
     * drive refusing a protected read - three conditions with three
     * completely different answers. */
    uint8_t last_sense_key = 0xff, last_asc = 0, last_ascq = 0;

    /* How many attempts were actually made, and whether one of the paths
     * below already explained the failure in its own words. Both exist so
     * the summary at the end cannot contradict the lines above it: a
     * command that broke out on the first refusal must not then be
     * reported as having exhausted a six-attempt budget it never touched.
     * (int rather than bool: this file has no <stdbool.h> and uses int
     * flags throughout.) */
    int attempts_made = 0;
    int reason_logged = 0;

    /* How many attempts in a row failed before the CBW could be handed over.
     * See the verdict at the end of the loop: a device that cannot accept 31
     * bytes on its bulk endpoint, every time, for the whole budget, is not a
     * device that is busy. */
    int cbw_never_sent = 0;

    for (int attempt = 1; attempt <= LASER_MAX_RETRIES; attempt++) {
        attempts_made = attempt;

        /* Between attempts, which is the granularity cancellation offers:
         * the transfer already handed to the kernel runs to its timeout, but
         * the five that would have followed it do not happen. That is what
         * turns a minute into a phase. */
        if (laser_is_cancelled(entry)) {
            LOGI("token=%d: cancelled, abandoning cdb 0x%02x after %d "
                 "attempt(s)", token, cdb[0], attempt - 1);
            result = LASER_ERR_CANCELLED;
            break;
        }

        /* Local to this attempt: request_sense_locked() below issues its
         * own BOT transaction, and this value must survive it intact. */
        int csw_status = -1;
        int rc = laser_bot_send_locked(entry, cdb, cdb_len, data, data_len,
                                         data_in, actual_len, &csw_status);
        if (rc == 0) {
            result = LASER_OK;
            break;
        }

        /* Counted, not acted on here: one failure to hand the CBW over says
         * nothing - a bridge under load, a transient on the bus. The verdict
         * needs the whole budget, so it is drawn after the loop. */
        if (rc == BOT_FAIL_NOT_SENT) {
            cbw_never_sent++;
        }

        if (rc == BOT_FAIL_NO_DEVICE) {
            /* The drive has left the bus. Retrying cannot bring it back, and
             * each further attempt costs a full set of USB timeouts to
             * establish the same thing - on an unplug mid-playback, that is
             * the difference between failing at once and failing a minute
             * later, per read, with libVLC's input thread stalled
             * throughout.
             *
             * REPORTED AS ITS OWN STATUS, because this is the strongest
             * statement this layer can make and flattening it into ERR_IO
             * loses the only thing a caller can act on. An I/O error invites
             * a retry; a caller that retries here walks a whole UDF tree one
             * dead command at a time. ERR_MEDIA_GONE would be latched
             * correctly by every consumer, but it says the drive answered,
             * which it did not. */
            LOGW("token=%d: device no longer present, not retrying", token);
            entry->device_gone = 1;
            result = LASER_ERR_NO_DEVICE;
            break;
        }

        /* A non-idempotent command that the drive may already have acted
         * on: stop here. BOT_FAIL_NOT_SENT is the exception - the CBW
         * never got out, so the drive's state is untouched and a retry
         * is as safe as for any read. */
        if (!idempotent && rc != BOT_FAIL_NOT_SENT) {
            LOGW("token=%d: DATA-OUT command failed after the CBW was sent, "
                 "not retrying (drive state may have advanced)", token);
            result = LASER_ERR_IO;
            break;
        }

        uint8_t sense_key = 0xff, asc = 0, ascq = 0;
        if (rc == BOT_FAIL_PHASE_ERROR) {
            /* The device has just been reset out of a desynchronised
             * state; there is no completed command left for it to
             * explain, so REQUEST SENSE would be meaningless (and is
             * itself just another command to a device that has only
             * just come back). Skip straight to the delayed retry: the
             * attempt is counted like any other, so a drive that keeps
             * phase-erroring still fails out rather than looping. */
            LOGW("token=%d: retrying after Reset Recovery (attempt %d/%d)",
                 token, attempt, LASER_MAX_RETRIES);
        } else if (csw_status == USB_BOT_STATUS_FAIL) {
            request_sense_locked(entry, &sense_key, &asc, &ascq);
            last_sense_key = sense_key;
            last_asc = asc;
            last_ascq = ascq;
        }

        /* Disc gone or swapped: never retry, surface immediately so an
         * ejection doesn't feel sluggish to the user. */
        if (sense_key == SCSI_SENSE_KEY_NOT_READY &&
            asc == SCSI_ASC_MEDIUM_NOT_PRESENT) {
            LOGW("token=%d: no disc present, not retrying", token);
            result = LASER_ERR_MEDIA_GONE;
            break;
        }
        if (asc == SCSI_ASC_MEDIUM_MAY_HAVE_CHANGED &&
            ascq == SCSI_ASCQ_MEDIUM_MAY_HAVE_CHANGED) {
            LOGW("token=%d: medium may have changed, not retrying", token);
            result = LASER_ERR_MEDIA_GONE;
            break;
        }
        if (asc == SCSI_ASC_MEDIUM_MAY_HAVE_CHANGED &&
            ascq == SCSI_ASCQ_FORMAT_LAYER_CHANGED) {
            /* Dual-layer boundary, not an ejection: the disc is still
             * there and the read that follows will normally succeed.
             * Falls through to the ordinary delayed retry below rather
             * than aborting playback - reporting MEDIA_GONE here would
             * kill a DVD-9 the moment it crosses from layer 0 to layer
             * 1, which on a feature film is somewhere around the middle
             * of the movie. */
            LOGW("token=%d: format layer changed (dual-layer boundary), "
                 "retrying", token);
        }

        /* Permanently refused: the drive parsed the command and will
         * answer the same way every time. Retrying is not merely futile,
         * it is actively harmful - six rounds of half-second delays per
         * read, on a caller like libdvdcss that issues thousands of them,
         * turns a clean error into what a user experiences as a hang. */
        /* Copy-protection refusals, told apart before the generic case
         * below swallows them.
         *
         * All three outcomes are permanent for THIS command - none is
         * retried - but they mean different things to the caller, and the
         * caller cannot recover the qualifier once this function has
         * returned. Collapsing them into one status was costing two
         * distinct bugs: a region mismatch reported to the user as an
         * authentication problem, and - once CSS authentication exists - a
         * momentarily lost session being recorded as a permanently
         * unreadable sector. */
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

        if (sense_key == SCSI_SENSE_KEY_ILLEGAL_REQUEST ||
            sense_key == SCSI_SENSE_KEY_DATA_PROTECT) {
            LOGW("token=%d: cdb 0x%02x permanently refused "
                 "(sense %02x/%02x/%02x), not retrying",
                 token, cdb[0], sense_key, asc, ascq);
            reason_logged = 1;
            result = LASER_ERR_REFUSED;
            break;
        }

        /* Transient conditions: UNIT ATTENTION clears itself by being
         * reported, retry immediately with no delay. NOT READY/becoming
         * ready needs an actual wait for the drive to spin up. Anything
         * else (transport error, unexpected sense) falls through to the
         * same delayed retry - cheap, and avoids hammering a
         * momentarily confused drive. */
        if (sense_key == SCSI_SENSE_KEY_UNIT_ATTENTION) {
            continue;
        }

        if (attempt < LASER_MAX_RETRIES) {
            usleep(LASER_RETRY_DELAY_MS * 1000);
        }
    }

    /* Normalise whatever the loop left behind into the public contract.
     *
     * A status that carries a specific meaning to the caller is preserved
     * verbatim; everything else - the initial value, a transient
     * condition that merely ran out of attempts, an internal BOT failure
     * code - is reported as LASER_ERR_IO.
     *
     * THE LIST BELOW MUST GROW WITH EVERY NEW TERMINAL STATUS. A status
     * missing from it is silently flattened into ERR_IO on the way out,
     * with no compiler warning and no trace in the log, because the
     * specific reason has usually already been printed by the branch that
     * set it - so the log looks right while the caller receives something
     * it cannot act on. That is precisely what happened to
     * LASER_ERR_REFUSED when it was added: the refusal was detected,
     * logged, and then turned back into an ordinary I/O error, leaving
     * the stream layer unable to tell a permanently refused sector from a
     * scratched one and looping on it forever. */
    /* EVERY ATTEMPT FAILED BEFORE THE CBW LEFT: the device is gone, whatever
     * libusb called it.
     *
     * LIBUSB_ERROR_NO_DEVICE is the answer the kernel gives for a device that
     * has left the bus while its descriptor is still open. It is not the only
     * answer available: a descriptor closed underneath the handle, or a
     * controller that tears the URB down before it marks the device absent,
     * both produce LIBUSB_ERROR_IO instead - and that is indistinguishable,
     * one attempt at a time, from a bridge having a bad moment.
     *
     * The whole budget makes it distinguishable. A CBW is 31 bytes on a bulk
     * endpoint; a device that will not take them once may be busy, but one
     * that will not take them on any of LASER_MAX_RETRIES attempts spread
     * over several seconds is not there. Concluding so costs nothing when the
     * conclusion is wrong: the command has failed either way, and the only
     * difference is whether the caller is told to stop or invited to try
     * again on a drive that will not answer.
     *
     * Only when EVERY attempt failed that way. A single successful CBW
     * anywhere in the budget proves the endpoint was alive, and the failure
     * that followed belongs to the medium or the command.
     *
     * Never over a cancellation, which the loop above can break out on: the
     * caller asked for this to stop, and telling it the hardware disappeared
     * would answer a question it did not ask. */
    if (result != LASER_OK && result != LASER_ERR_CANCELLED &&
        attempts_made > 0 && cbw_never_sent == attempts_made) {
        LOGW("token=%d: cdb 0x%02x - the CBW could not be sent on any of "
             "%d attempt(s); treating the device as gone",
             token, cdb[0], attempts_made);
        entry->device_gone = 1;
        result = LASER_ERR_NO_DEVICE;
        reason_logged = 1;
    }

    if (result != LASER_OK &&
        result != LASER_ERR_MEDIA_GONE &&
        result != LASER_ERR_NO_DEVICE &&
        result != LASER_ERR_REFUSED &&
        result != LASER_ERR_SCRAMBLED &&
        result != LASER_ERR_NO_KEY &&
        result != LASER_ERR_REGION &&
        result != LASER_ERR_CANCELLED) {
        /* Only claim the full budget was spent when it actually was: the
         * non-idempotent DATA-OUT path above breaks out on the first
         * failure and has already logged its own, more specific reason. */
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
 * Public: LBA-aware chunked block reads
 * ============================================================================ */

/* Build the CDB for one chunk. The only thing the two public helpers below do
 * not have in common, which is why it is the only thing they pass in. */
typedef void (*build_read_cdb_fn)(uint8_t *cdb, uint32_t lba, int blocks,
                                  const void *ctx);

/* The chunked read both helpers are. Everything here was written twice, once
 * per sector size: the same loop, the same buffer stride, the same MEDIA_GONE
 * rule, the same "zero blocks is a failure, not a short read" reasoning, and
 * the same two paragraphs explaining them. Two copies of a rule is two places
 * for it to change; the next adjustment to the short-read contract now
 * happens once.
 *
 * @param block_size bytes per block, which also sets how many fit in one
 *                   transaction
 * @param cdb_len    10 for READ(10), 12 for READ CD
 * @param what       what to call these in a log line ("blocks", "sectors")
 */
/* Halve this device's transfer cap, if there is room left to halve.
 *
 * @return 1 if the cap actually shrank, 0 if it was already at the floor.
 */
/* ============================================================================
 * DVD region (laser_region_mismatch)
 * ========================================================================= */

/* Region masks, in both structures, are "one bit per region, SET means
 * PROHIBITED": a region-1 disc reads 0xFE, a drive set to region 2 reads
 * 0xFD. So the regions something permits are the complement, and disc and
 * drive agree exactly when their permitted sets intersect. */
#define LASER_REGION_ALL        0xFF

/* Byte 4 >> 6 of the RPC state. 0 means no region has been set yet, which is
 * not a mismatch: the drive will adopt the first disc's region by itself, and
 * that is the drive's decision to make, not ours to pre-empt with a warning. */
#define LASER_RPC_TYPE_NONE     0x00

/* Byte 6 of the RPC state. 0 is RPC-1: the drive enforces nothing and any
 * disc plays regardless of what its copyright structure says. */
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
    cdb[10] = 0x08;                  /* key format: RPC state. The AGID bits
                                      * of this byte are reserved for this
                                      * format - the drive allocates nothing
                                      * here - so it carries the format
                                      * alone. */

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

    /* Written only on the answer that makes them meaningful - see the
     * contract in laser.h. A caller that ignores the return value and reads
     * them anyway is asking about a comparison that did not happen. */
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
    /* Deliberately next to laser_scsi_cdb(), which is the only place any of
     * these values is produced: whoever adds a status there has this list in
     * the same file, and the contract explaining the split is on the
     * declaration in laser.h. Splitting the two - the rule in one file, the
     * classification in another - is what made a consumer's private copy
     * drift in the first place. */
    switch (status) {
    case LASER_ERR_SCRAMBLED:
    case LASER_ERR_REGION:
    case LASER_ERR_REFUSED:
        return 1;
    default:
        /* Includes every non-negative value: a block count is not a status,
         * and answering "not positional" for it is the correct reading of a
         * successful read. */
        return 0;
    }
}

static int narrow_transfer_cap(laser_entry_t *entry)
{
    pthread_mutex_lock(&entry->io_lock);

    int cap = entry->max_transfer_bytes;
    if (cap <= LASER_MIN_BYTES_PER_TRANSFER) {
        pthread_mutex_unlock(&entry->io_lock);
        return 0;
    }

    cap /= 2;
    if (cap < LASER_MIN_BYTES_PER_TRANSFER) {
        cap = LASER_MIN_BYTES_PER_TRANSFER;
    }
    entry->max_transfer_bytes = cap;

    pthread_mutex_unlock(&entry->io_lock);

    LOG_QUIRK(entry, "transfer failed, limiting this device to %d bytes "
                     "per command from now on", cap);
    return 1;
}

static int read_chunked(int token, uint32_t lba, int num_blocks,
                        int block_size, int cdb_len,
                        build_read_cdb_fn build_cdb, const void *ctx,
                        uint8_t *buffer, const char *what)
{
    /* Asking for nothing succeeds at reading nothing. Made explicit so that 0
     * has exactly one meaning on the way out of the loop below - where it is
     * an error - and exactly one here, where it is not. */
    if (num_blocks <= 0) {
        return 0;
    }

    /* Needed for the per-device transfer cap below. A miss is not fatal here:
     * laser_scsi_cdb() looks the token up again and reports NO_SUCH_TOKEN
     * properly, so this only costs the negotiation. */
    laser_entry_t *entry = laser_lookup(token);
    int blocks_done = 0;

    while (blocks_done < num_blocks) {
        /* Re-read the cap every time round: another thread reading from the
         * same drive may have narrowed it since the last chunk, and this is
         * the only place it is consulted. */
        int cap = entry != NULL ? entry->max_transfer_bytes
                                : LASER_MAX_BYTES_PER_TRANSFER;
        int max_blocks_per_chunk = cap / block_size;
        if (max_blocks_per_chunk < 1) {
            /* A cap below one block would compute a chunk of zero and stall
             * the loop. One block is the smallest thing that makes progress,
             * so it is the floor whatever the cap says. */
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
            /* MEDIA_GONE is reported even when earlier chunks succeeded.
             * Every other failure degrades to a short read (see below): that
             * is the right call for a scratched sector, where the blocks
             * already in `buffer` are real data the caller can use and read
             * around. A disc ejected or swapped mid-read is categorically
             * different - the remaining blocks are not merely unreadable
             * right now, they will never be readable from this medium, and
             * the ones already read may belong to a disc that is no longer in
             * the drive. Collapsing that into a short read defeats the
             * "surface this as a fatal, immediate error" contract documented
             * on LASER_ERR_MEDIA_GONE in laser.h: dvdread/dvdnav treat a
             * short read as an ordinary disc imperfection and keep going, so
             * after an eject they would keep hammering a drive that can no
             * longer answer.
             *
             * NO_DEVICE propagates for a stronger version of the same
             * reason: there is no drive to read around, and a caller that
             * treats it as a short read spends a dead command per block for
             * as long as it cares to walk.
             *
             * CANCELLED propagates for the same reason and one more: the
             * caller asked for it, so reporting progress instead would have
             * it carry on around an interruption it requested itself.
             *
             * Partial data already written into `buffer` is left as-is and
             * must be ignored by the caller on a negative return - same rule
             * as every other error path here. */
            if (st == LASER_ERR_MEDIA_GONE || st == LASER_ERR_NO_DEVICE ||
                st == LASER_ERR_CANCELLED) {
                return (int)st;
            }

            /* A plain I/O failure may be the BRIDGE rather than the disc.
             * Some USB-ATAPI bridges cannot carry a full 64 KiB data phase
             * and fail the whole command rather than returning a short one;
             * the same request split smaller goes through. So before giving
             * up, halve this device's cap and retry the SAME blocks - once
             * per halving, three times at most between the ceiling and the
             * floor, after which the failure is reported.
             *
             * THIS LOOP TERMINATES ON A LOCAL VARIANT: narrow_transfer_cap()
             * returns 0 once the floor is reached, and every retry strictly
             * decreases a value bounded below, so the number of retries per
             * chunk is bounded by log2(MAX/MIN) whatever any other thread is
             * doing.
             *
             * Retried only when the chunk is larger than one block: at one
             * block there is nothing left to split and the failure is about
             * the medium. */
            if (st == LASER_ERR_IO && chunk > 1 && entry != NULL &&
                narrow_transfer_cap(entry)) {
                continue;
            }

            return blocks_done > 0 ? blocks_done : (int)st;
        }

        if (actual_len != chunk * block_size) {
            /* Short read: return what we got so far rather than treating it
             * as a hard failure - matches the tolerance dvdread/dvdnav expect
             * from a real block device on an imperfect disc.
             *
             * Unless nothing at all came back. Zero is not a short read, it
             * is a failed one, and returning it as a count invites the caller
             * into an infinite loop: the natural shape of a block-read caller
             * is "advance by what was read, ask for the rest", which makes no
             * progress on 0 and asks again forever. A negative status makes
             * it terminate. */
            int short_total = blocks_done + actual_len / block_size;
            if (short_total == 0) {
                LOGW("token=%d: read of %d %s at LBA %u returned no data",
                     token, num_blocks, what, lba);
                return (int)LASER_ERR_IO;
            }
            return short_total;
        }

        blocks_done += chunk;
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
    /* The same for both sector kinds, which is what lets everything below
     * this line stay indifferent to which one was asked for - the chunk
     * arithmetic, the buffer stride and the short-read accounting are all
     * written once. See laser_cd_sector_t in laser.h for why the two kinds
     * agree on this number by different arithmetic. */
    const int block_size = 2352;

    /* The whole of the difference: byte 1 of the CDB says what the drive
     * should expect to find, byte 9 says which parts of it to send back.
     * Resolved once, here, rather than inside the loop, so that the CDB
     * construction below reads the same for either kind and there is one
     * place to look when a third kind is added.
     *
     *   AUDIO: Expected Sector Type = CD-DA (001b), User Data alone.
     *     The obvious-looking alternative - sector type "any" with every
     *     flag set (0xF8: Sync + Header + Sub-header + User Data + EDC/ECC)
     *     - asks for structures a CD-DA sector does not have. An audio
     *     sector has no sync pattern, no header and no EDC/ECC; its 2352
     *     bytes ARE the user data. MMC's own byte-count table defines only
     *     User Data for this sector type, and a drive is entitled to answer
     *     a request for the others with ILLEGAL REQUEST / INVALID FIELD IN
     *     CDB. Some do, some quietly ignore the extra bits and return the
     *     same 2352 bytes - which is why the wrong form works on part of the
     *     hardware and fails on the rest, and why this is the form CD-DA
     *     extractors have converged on.
     *
     *     If a drive is ever found that rejects THIS form, the fallback to
     *     try is sector type "any" (byte 1 = 0x00) with User Data alone
     *     (byte 9 = 0x10) - not the return of the EDC/ECC bits.
     *
     *   MODE2_FORM2: Expected Sector Type = Mode 2 Form 2 (101b), with Sync,
     *     both Header Codes and User Data requested (0xF0).
     *
     *     THE AUDIO REASONING DOES NOT TRANSFER, and that is why the flags
     *     differ rather than being shared: a Mode 2 Form 2 sector really
     *     does carry a sync pattern, a header and a sub-header, so asking
     *     for them is asking for structures that exist. What is NOT
     *     requested here is EDC/ECC, and the four bytes it would return are
     *     the ones cdrom.c discards anyway - it takes 2324 bytes from offset
     *     24 and stops.
     *
     *     0xF0 rather than 0xF8 because it is what VLC's own BSD arm has
     *     shipped for this sector kind for years, against the same drives.
     *     THE ONE THING TO VERIFY ON REAL HARDWARE: whether the drive counts
     *     those trailing four bytes into its transfer length regardless. A
     *     drive that returns 2348 rather than 2352 per sector will trip the
     *     strict length check below and be reported as a short read, one
     *     sector at a time. The fix if that is ever observed is 0xF8, which
     *     asks for the EDC explicitly and makes 2352 unambiguous. */
    uint8_t expected_type;
    uint8_t field_flags;

    switch (sector_type) {
        case LASER_CD_SECTOR_AUDIO:
            expected_type = 0x04;
            field_flags   = 0x10;
            break;

        case LASER_CD_SECTOR_MODE2_FORM2:
            expected_type = 0x14;
            field_flags   = 0xF0;
            break;

        default:
            /* Rejected before the device is touched, on the same grounds as
             * laser_scsi_cdb()'s argument validation: this is a caller bug,
             * and forwarding it would surface as an ILLEGAL REQUEST from the
             * drive - an answer that names the CDB and not the mistake. */
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
