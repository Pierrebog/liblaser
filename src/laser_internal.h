/*****************************************************************************
 * laser_internal.h: private to liblaser's own sources, never included by
 * callers of laser.h.
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
 *****************************************************************************/

#ifndef LASER_INTERNAL_H
#define LASER_INTERNAL_H

#include <stdint.h>
#include <sys/types.h>
#include <pthread.h>
#include <libusb.h>

/* For laser_status_t and laser_log_level_t, both of which appear below. */
#include "laser.h"

/* ---------------------------------------------------------------------------
 * Logging, routed through laser_log() to the sink laser_set_log_cb() set.
 *
 * LOGE is for a caller violating the contract of laser.h, or for a bug
 * upstream such as an unbalanced release. LOGW is the hardware misbehaving.
 * ------------------------------------------------------------------------- */
void laser_log(laser_log_level_t level, const char *fmt, ...)
#ifdef __GNUC__
    __attribute__((format(printf, 2, 3)))
#endif
    ;

#define LOGI(...) laser_log(LASER_LOG_INFO,  __VA_ARGS__)
#define LOGW(...) laser_log(LASER_LOG_WARN,  __VA_ARGS__)
#define LOGE(...) laser_log(LASER_LOG_ERROR, __VA_ARGS__)

/* For a device not following the Bulk-Only specification: logged with its USB
 * identity, the key a per-device workaround would match on, and under a
 * "QUIRK?" marker that can be grepped out of a bug report. No device is
 * treated differently because of it; it only collects the evidence. */
#define LOG_QUIRK(entry, fmt, ...) \
    LOGW("QUIRK? usb %04x:%04x bcd %04x: " fmt, \
         (entry)->vid, (entry)->pid, (entry)->bcd_device, \
         ##__VA_ARGS__)

/** Fixed bound on concurrently registered devices, keeping the registry
 * allocation-free. One or two optical drives is the realistic case. */
#define LASER_MAX_DEVICES 8

/* Upper bound on the data phase of one BOT transaction, and the starting
 * value of laser_entry_t::max_transfer_bytes. Set by what USB bulk transfers
 * carry reliably across Android host controllers, far below READ(10)'s own
 * limit. */
#define LASER_MAX_BYTES_PER_TRANSFER  (64 * 1024)

/* Floor of the per-device negotiation: below it, per-command overhead
 * dominates, and a bridge that cannot carry 8 KiB in one command has a
 * problem no tuning will fix. Chunks never go below one block whatever the
 * cap, which matters for READ CD, where this is only three sectors. */
#define LASER_MIN_BYTES_PER_TRANSFER  (8 * 1024)

/** Per-device state for USB Mass Storage Bulk-Only Transport (BOT); the
 * protocol itself is in bot.c. */
typedef struct {
    /* Is this slot a live, fully set-up registration? Written last by
     * laser_register(), so a slot under construction - which lasts seconds -
     * is never found by laser_lookup().
     *
     * Written under g_registry_lock, read under g_table_lock. */
    int in_use;
    int token;

    /* Claims held through laser_acquire() and not yet released. A device is
     * registered only with a claim, so this is 1 when the entry becomes
     * visible, and teardown happens when it falls back to 0.
     *
     * Written and read under g_registry_lock. */
    int refs;

    /* Largest transfer, in BYTES, this device carries: 2048-byte READ(10)
     * blocks and 2352-byte READ CD sectors alike, and bytes are what a bridge
     * reacts to.
     *
     * Starts at LASER_MAX_BYTES_PER_TRANSFER and only ever shrinks, never
     * below LASER_MIN_BYTES_PER_TRANSFER: lowered when smaller transfers read
     * the whole range a larger one failed on, which shows the limit to be the
     * bridge's rather than the disc's (see read_chunked() in scsi.c). Kept
     * here rather than in a consumer, so that every consumer of the token
     * benefits.
     *
     * Written under io_lock, read without it when sizing a chunk: a stale
     * read asks for too much once and fails over to the same negotiation. */
    int max_transfer_bytes;

    /* Raised by the laser_release() that drops the last claim, just before
     * teardown, and never cleared; the slot is memset afterwards. An
     * operation still running on another thread sees it between retry
     * attempts and between chunks, and gives up.
     *
     * Read without any lock, on the transaction path: it only goes from 0 to
     * 1, so a reader that misses it sees it one attempt later. */
    int cancelled;

    /* Latched once the device is shown to have left the bus, so that every
     * later command on the token fails at once instead of spending a retry
     * budget - about two and a half seconds - to learn the same thing.
     * Terminal: a device back on the bus comes with a new fd, hence a new
     * registration.
     *
     * Read without any lock, on the same terms as `cancelled`. */
    int device_gone;

    /* Identity of the descriptor at registration, from fstat(), so that
     * laser_acquire() can refuse an fd number the system has recycled while
     * an entry still refers to it - which only a claim never released allows.
     * Zero when fstat() failed. A heuristic: usbfs may reuse inode numbers. */
    dev_t reg_dev;
    ino_t reg_ino;

    /* Dedicated libusb context, never the shared default one: a playback
     * session lasting hours must not share libusb state with a detection
     * scan on another device. */
    libusb_context *ctx;
    libusb_device_handle *handle;

    /* The interface carrying the Bulk-Only function, as chosen by
     * laser_find_bulk_endpoints() - not assumed to be 0. Claim, release, Mass
     * Storage Reset and GET MAX LUN all address this one. */
    uint8_t iface_num;

    unsigned char ep_in;
    unsigned char ep_out;

    /* wMaxPacketSize of the bulk pair: 64 at full speed, 512 at high speed,
     * 1024 at SuperSpeed. Read off the descriptor, since
     * libusb_get_device_speed() reports UNKNOWN on some kernels. Used by the
     * throughput check in scsi.c. */
    uint16_t ep_max_packet;

    /* Set when laser_register() detached the kernel driver itself, so that a
     * path giving the device up knows whether there is one to give back. */
    int kernel_driver_detached;

    /* Sustained read throughput, sampled for the one failure that raises no
     * error: an underpowered drive that answers every command correctly,
     * slowly. Written under io_lock; slow_warned latches the single report. */
    uint64_t xfer_bytes;
    uint64_t xfer_us;
    uint64_t xfer_count;
    int      slow_warned;

    uint32_t tag;

    /* USB identity, read at registration from the cached device descriptor,
     * so that log lines name the hardware rather than an fd. */
    uint16_t vid;
    uint16_t pid;
    uint16_t bcd_device;

    /* Set once a residue contradicting a full transfer, or a stalled status
     * phase, has been logged, so that a bridge that always does it is
     * reported once, not on every command. */
    int residue_quirk_logged;
    int csw_stall_quirk_logged;

    /** Nonzero while a probe - a command needing no medium, INQUIRY today -
     * is in flight: shortens the BOT phase timeouts. Set and cleared around
     * the command, under io_lock. */
    int probe_timeouts;

    /* Logical Unit to address in every CBW. Almost always 0, but a combo
     * enclosure may put its card reader there and the optical drive on
     * another unit. Chosen at registration by laser_probe_lun(). */
    uint8_t lun;

    /* There is deliberately no "last CSW status" field: the status is passed
     * back through laser_bot_send_locked()'s out-parameter, since the
     * REQUEST SENSE that follows a failure would overwrite a shared copy. */

    /* Serializes BOT transactions on this device - one CBW, its data phase,
     * its CSW, never interleaved - for the whole of laser_scsi_cdb(),
     * retries included. */
    pthread_mutex_t io_lock;

    /* CSS authentication session: exclusion between CONSUMERS, one level
     * above io_lock.
     *
     * Authentication is a sequence whose state lives in the drive between
     * transactions - AGID, challenges, keys. io_lock serializes each step and
     * protects nothing between them: another consumer's AGID request in a gap
     * takes one of the drive's four AGIDs and may invalidate ours. Observed
     * with the media library's preparser opening the same disc on its own
     * thread.
     *
     * Owned by an opaque cookie (the dvdcss_t), not by a thread: a libdvdcss
     * instance issues key commands from several threads over its life, and a
     * pthread mutex must be unlocked by the thread that locked it. */
    pthread_mutex_t css_mtx;
    pthread_cond_t  css_cv;
    int             css_open;
    const void     *css_owner;

    /* Raised during laser_wait_until_ready() and cleared when the unit
     * answers ready, so it remains set only on a drive whose last wait ran
     * out.
     * Registration succeeds either way; laser_disc_identify() reads it to
     * skip probes that would each spend a retry budget for nothing. */
    int             not_ready;
} laser_entry_t;

/**
 * Look up the entry for a token, or NULL if it is not registered.
 *
 * The pointer stays valid for the life of the registration - entries live in
 * a fixed table - so scsi.c holds it across a whole transaction, provided the
 * caller does not race with the laser_release() dropping the last claim (see
 * laser.h).
 *
 * The only way in, apart from laser_acquire(): a command on an unclaimed token
 * is a caller bug, reported as such rather than registering a device nobody
 * will release.
 */
laser_entry_t *laser_lookup(int token);

/**
 * Mark this entry cancelled, and test that mark.
 *
 * Set only by the laser_release() that drops the last claim, just before
 * teardown. Tested without a lock, by the retry loop and the chunk loops
 * (see laser_entry_t::cancelled) - not by the spin-up wait, which runs on an
 * entry not yet published, that nothing can cancel.
 */
void laser_set_cancelled(laser_entry_t *entry);
int  laser_is_cancelled(const laser_entry_t *entry);

/**
 * Is a CSS session open on this entry, by any consumer?
 *
 * Deliberately not "is it mine": the rule enforced is that no key command
 * touches authentication state without SOME declared session. Ownership is
 * checked by cookie at end().
 */
int laser_css_session_is_open(laser_entry_t *entry);

/**
 * Does this CDB change the drive's CSS authentication state?
 *
 * Keyed on the key FORMAT, not the opcode: REPORT KEY and READ DVD STRUCTURE
 * each carry both state-changing steps and read-only queries that libdvdcss
 * issues outside any session. The answer decides two things: whether a
 * session is required, and whether a retry is safe.
 */
int laser_cdb_changes_css_state(const uint8_t *cdb, int cdb_len);

/**
 * Select this device's Bulk-Only mass storage interface (class 0x08,
 * protocol 0x50) and its bulk IN/OUT pair, filling entry->iface_num,
 * entry->ep_in, entry->ep_out and entry->ep_max_packet.
 *
 * Runs before the interface is claimed, since it says which one to claim;
 * reading the configuration descriptor needs no claim.
 *
 * @return 0 on success, -1 if the device exposes no such interface.
 */
int laser_find_bulk_endpoints(laser_entry_t *entry);

/**
 * Bulk-Only Mass Storage Reset (class request to entry->iface_num), then
 * clear_halt on both endpoints, putting the device's BOT state machine in a
 * known state. Best-effort: some drives do not implement the reset, and the
 * clear_halt calls help regardless.
 */
void laser_mass_storage_reset(laser_entry_t *entry);

/**
 * Wake the drive and wait for its medium to become ready, before any read.
 * Called at registration, after laser_probe_lun() has found a unit that
 * answers, and again by laser_token_settle(). Best-effort: a drive that never
 * reports ready is still registered, with laser_entry_t::not_ready left set.
 *
 * Bounded in real time by LASER_SPINUP_MAX_WALL_MS (15 s), reached only by a
 * drive that has stopped answering; it cannot be cancelled. A working drive,
 * loaded or empty, returns within a few seconds. See scsi.c.
 */
void laser_wait_until_ready(laser_entry_t *entry);

/**
 * On a token already registered, one TEST UNIT READY, then
 * laser_wait_until_ready() if the unit is not ready.
 *
 * The wait at registration does not cover a disc swapped while the device
 * stays registered - a paused playback holding it: the new disc may still be
 * loading, and its UNIT ATTENTION is still pending, which the command layer
 * reports as LASER_ERR_MEDIA_GONE without retrying. laser_disc_identify()
 * calls this first, at the cost of one command on a ready drive.
 *
 * Does nothing for a token not registered, cancelled, or whose device is
 * gone.
 */
void laser_token_settle(int token);

/** Answers of laser_probe_lun(). */
enum {
    /** INQUIRY says direct-access block device. Not an optical drive. */
    LASER_OPTICAL_NO = 0,
    /** INQUIRY answered, with a type that does not rule an optical drive
     * out. */
    LASER_OPTICAL_YES = 1,
    /** INQUIRY went unanswered on every unit. */
    LASER_OPTICAL_NO_ANSWER = -1,
    /** The device left the bus during the probe. */
    LASER_OPTICAL_GONE = -2,
};

/**
 * Work out which Logical Unit is the optical drive, store it in entry->lun,
 * and say whether there is one.
 *
 * Issues GET MAX LUN, then an INQUIRY per unit, under the short probe
 * timeouts, keeping the first unit whose peripheral device type is CD/DVD.
 * Falls back to LUN 0 whenever anything is unsupported or inconclusive - as
 * the class specification prescribes for a device that stalls GET MAX LUN,
 * and as is right for the many drives with a single unit.
 *
 * Only a direct-access block device is rejected: a card reader, identical to
 * a drive in its USB descriptors, answers that, as does a USB key. Any other
 * type is accepted, INQUIRY data not always being truthful.
 *
 * INQUIRY needs no medium, so this runs before laser_wait_until_ready(), and
 * a device that answers it on no unit is not warming up but not talking at
 * all. The caller declines both that and LASER_OPTICAL_GONE, handing the
 * device back to the kernel - the only agent able to reset it.
 *
 * @return LASER_OPTICAL_YES, LASER_OPTICAL_NO, LASER_OPTICAL_NO_ANSWER or
 *         LASER_OPTICAL_GONE.
 */
int laser_probe_lun(laser_entry_t *entry);

/* ============================================================================
 * One Bulk-Only transaction (bot.c)
 * ========================================================================= */

/** Raw bCSWStatus values, as reported through laser_bot_send_locked()'s
 * csw_status out-parameter. */
#define USB_BOT_STATUS_PASS        0x00
#define USB_BOT_STATUS_FAIL        0x01
/* Host and device disagree about the transfer badly enough that the device's
 * state machine is out of sync. BBB 6.7 requires a Reset Recovery before any
 * further command; clearing a stalled endpoint is not enough. */
#define USB_BOT_STATUS_PHASE_ERROR 0x02

/** The command was never executed: its CBW did not go out, or went out
 * incomplete or was stalled - in which case the Reset Recovery BBB requires
 * has been performed. Replaying it is always safe. */
#define BOT_FAIL_NOT_SENT    (-2)

/** The transaction ended in a state requiring a Reset Recovery, which
 * laser_bot_send_locked() has already performed. A retry is allowed but
 * counted: a device landing here repeatedly is broken, not busy. */
#define BOT_FAIL_PHASE_ERROR (-3)

/** The device is no longer on the bus. No wait or retry can help, and every
 * further command would cost a full set of USB timeouts. */
#define BOT_FAIL_NO_DEVICE   (-4)

/**
 * Perform exactly one Bulk-Only transaction: CBW, data phase in either
 * direction, CSW - and, where BBB requires it, the Reset Recovery that must
 * follow before any other command is sent.
 *
 * Knows nothing about what the CDB means; retries, sense data and
 * idempotency are scsi.c's, decided on this return value:
 *
 *   0                     the command completed and the CSW says PASS
 *   BOT_FAIL_NOT_SENT     the drive never executed it; replay is free
 *   BOT_FAIL_PHASE_ERROR  host and device disagreed; recovery already done
 *   BOT_FAIL_NO_DEVICE    the device is gone; no budget can help
 *   -1                    anything else: the CBW went out, so the drive HAS
 *                         received the command and may have executed it, in
 *                         full or in part, even though we failed to read the
 *                         data phase or the CSW. Replaying is only safe for
 *                         commands that are idempotent.
 *
 * @param csw_status optional, may be NULL. Receives the raw bCSWStatus of
 *        THIS call, or -1 when no valid CSW was received, in which case
 *        REQUEST SENSE has no completed command to explain. An out-parameter
 *        rather than a field on `entry`, since the REQUEST SENSE following a
 *        FAIL goes through this same function.
 *
 * Caller MUST already hold entry->io_lock.
 */
int laser_bot_send_locked(laser_entry_t *entry,
                          const uint8_t *cdb, int cdb_len,
                          uint8_t *data, int data_len, int data_in,
                          int *actual_len, int *csw_status);

#endif /* LASER_INTERNAL_H */
