/*****************************************************************************
 * laser_internal.h: private, shared between registry.c and
 * usb.c, bot.c and scsi.c only. Never included by callers of laser.h.
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
 *****************************************************************************/

#ifndef LASER_INTERNAL_H
#define LASER_INTERNAL_H

#include <stdint.h>
#include <pthread.h>
#include <libusb.h>

/* For laser_status_t and laser_log_level_t, both of which appear below. */
#include "laser.h"

/* ---------------------------------------------------------------------------
 * Logging, defined once here rather than twice in the .c files.
 *
 * registry.c and the transport files each carried their own copy of these three
 * macros, identical apart from a comment, which is how the two drifted into
 * disagreeing about what LOGE meant. They also went straight to the
 * platform's log, which is what laser_set_log_cb() exists to make optional -
 * so the destination has to be resolved in one place anyway.
 *
 * LOGE is for programming errors on this library's own API - a caller
 * violating the contract in laser.h - and for conditions that indicate a bug
 * upstream, such as an unbalanced release. LOGW is the hardware misbehaving.
 * ------------------------------------------------------------------------- */
void laser_log(laser_log_level_t level, const char *fmt, ...)
#ifdef __GNUC__
    __attribute__((format(printf, 2, 3)))
#endif
    ;

#define LOGI(...) laser_log(LASER_LOG_INFO,  __VA_ARGS__)
#define LOGW(...) laser_log(LASER_LOG_WARN,  __VA_ARGS__)
#define LOGE(...) laser_log(LASER_LOG_ERROR, __VA_ARGS__)

/* For the handful of conditions that mean "this device is not following the
 * Bulk-Only spec". These are logged with the device's USB identity rather
 * than just its fd, and under a distinct "QUIRK?" marker, for one reason:
 * they are the observations that would justify a per-device workaround
 * later. Without the identity a user's logcat says something is wrong but
 * not what to key a workaround on; without the marker the lines are lost
 * among ordinary I/O warnings. Grepping "QUIRK?" out of a bug report should
 * be enough to tell whether a given drive is misbehaving in a known way.
 *
 * Deliberately not a mechanism - there is no quirk table behind this, and no
 * device is treated differently because of it. It only makes the evidence
 * collectable, which is the step that has to come first.
 *
 * Here rather than in one .c file because both bot.c and scsi.c raise them,
 * which is the same reason LOGI/LOGW/LOGE moved here. */
#define LOG_QUIRK(entry, fmt, ...) \
    LOGW("QUIRK? usb %04x:%04x bcd %04x: " fmt, \
         (entry)->vid, (entry)->pid, (entry)->bcd_device, \
         ##__VA_ARGS__)

/** Small, fixed upper bound on concurrently registered devices. In
 * practice there is realistically at most one or two optical drives
 * attached at once; a fixed-size table keeps the registry allocation-free
 * and trivially safe to reason about, at the cost of this arbitrary cap. */
#define LASER_MAX_DEVICES 8

/* Safe upper bound on the data phase of a single BOT transaction, and the
 * starting value of laser_entry_t::max_transfer_bytes. Well under the SCSI
 * READ(10) 16-bit block-count field's own limit (65535 blocks) - this cap
 * exists to stay comfortably inside USB bulk transfer sizes that are reliable
 * in practice across Android USB host controller implementations, not because
 * of a SCSI-level limit. */
#define LASER_MAX_BYTES_PER_TRANSFER  (64 * 1024)

/* Floor for the per-device negotiation. Below this the per-command overhead
 * dominates so heavily that there is nothing left to save, and a bridge that
 * cannot manage 8 KiB in one command has a problem no tuning will fix.
 *
 * Chunk sizing floors at ONE BLOCK independently of this, which matters for
 * READ CD: 8192 / 2352 is three sectors, and a cap set below one block would
 * otherwise compute a chunk of zero and make the read loop stand still. */
#define LASER_MIN_BYTES_PER_TRANSFER  (8 * 1024)

/** USB Mass Storage Bulk-Only Transport (BOT) - see bot.c for the
 * protocol implementation itself; this header only needs the shape of
 * the per-device state bot.c and scsi.c operate on. */
typedef struct {
    /* Is this slot a live, fully set-up registration?
     *
     * ONE FLAG IS ENOUGH because of WHEN it is written. Registration happens
     * only inside laser_acquire(), under g_registry_lock, and this field is
     * written LAST - so a slot under construction is not in the table at all.
     * Publishing it earlier would need a second flag beside it: the USB setup
     * is slow (control transfers, a Mass Storage Reset with its settle sleep,
     * a spin-up wait that can legally take ten seconds), and throughout that
     * window the entry would match on token with a NULL handle.
     *
     * Only a lock holder ever writes a free slot, and there is only ever one
     * at a time, so two threads cannot reserve the same fd either.
     *
     * Written under g_registry_lock, read under g_table_lock. */
    int in_use;
    int token;

    /* How many consumers have called laser_acquire() and not yet released.
     * Teardown happens when this reaches zero and not before - see
     * laser_release()'s contract in laser.h.
     *
     * EVERY REGISTRATION HAS A CLAIM BEHIND IT. A device is registered by
     * laser_acquire() and by nothing else, so refs is 1 the moment the entry
     * becomes visible, and an entry with refs == 0 does not exist. One that
     * did would be indestructible, since teardown only ever happens on the
     * 1 -> 0 transition.
     *
     * Written and read under g_registry_lock, which is also what makes
     * "register, then count" a single atomic step. */
    int refs;

    /* Largest transfer, in BYTES, this device has been shown to tolerate.
     *
     * Starts at LASER_MAX_BYTES_PER_TRANSFER and only ever shrinks, halving
     * each time a chunked read comes back LASER_ERR_IO, with a floor at
     * LASER_MIN_BYTES_PER_TRANSFER. A bridge that cannot manage the full 64
     * KiB in one command is a property OF THE BRIDGE, so it belongs here
     * rather than in a consumer: negotiated in one module it would leave the
     * others - CD-DA and VCD reach the drive through cdrom.c and
     * laser_read_cd_blocks() - failing on hardware the first had tamed.
     *
     * IN BYTES, NOT BLOCKS, because that is the quantity the bridge reacts
     * to and because it has to serve both block sizes: 2048 for READ(10),
     * 2352 for READ CD. Expressed in blocks it would mean different things
     * to the two callers.
     *
     * Written under io_lock (see narrow_transfer_cap in scsi.c), read
     * without it when sizing a chunk. That read is a benign race by
     * construction: the value only ever decreases, so a stale read asks for
     * slightly too much and is corrected on the spot by the same mechanism
     * that shrank it. */
    int max_transfer_bytes;

    /* Set by the laser_release() that takes refs to zero, immediately before
     * teardown, and never cleared - the slot is memset moments later, so a
     * fresh claim on the same fd starts clean.
     *
     * WHAT IT IS FOR, now that nothing outside the library sets it. The
     * public contract says no transaction may be in flight on a token when
     * its last claim is dropped, and nothing here can enforce that: another
     * thread legitimately obtained its entry pointer before teardown began.
     * Raising the flag first gives such a thread the chance to abandon its
     * remaining attempts rather than run out a six-attempt budget against a
     * handle that is about to close.
     *
     * Read WITHOUT any lock, from inside the retry loop and the chunk loops.
     * That is a benign race by construction: the only transition is 0 -> 1,
     * so a reader that misses it loses one more attempt and sees it on the
     * next. Making it a lock acquisition per attempt would put a registry
     * lock on the transaction path, which is the one thing those locks are
     * designed never to be on. */
    int cancelled;

    /* Latched once this device has been shown to have left the bus, so that
     * every later command on the token fails without touching libusb.
     *
     * WHAT IT SAVES. Establishing that a device is gone is expensive: on a
     * bridge whose descriptor no longer answers, it takes the whole retry
     * budget - six attempts and the delays between them, around two and a
     * half seconds - to conclude what the previous command concluded already.
     * A caller that reads sector by sector pays that per sector: an audio CD
     * demuxer willing to skip sixteen bad reads before giving up spends forty
     * seconds doing so, and none of it tells anyone anything new.
     *
     * TERMINAL FOR THIS REGISTRATION, which is what makes latching it sound.
     * Unlike a missing medium, which comes back when a disc is inserted, a
     * device that has left the bus does not come back on this token at all -
     * it returns as a new descriptor, hence a new registration, hence a fresh
     * entry with this flag clear.
     *
     * Read WITHOUT any lock, on the same terms as `cancelled` above: the only
     * transition is 0 -> 1, so a reader that misses it pays one more command
     * and sees it on the next. */
    int device_gone;

    /* Identity of the descriptor this entry was registered on, as reported by
     * fstat() at registration.
     *
     * WHAT IT DETECTS: a descriptor number reused by the operating system for
     * something else while an entry still refers to it. That can only happen
     * when a claim was never released, which is a lifecycle bug upstream - but
     * the consequence of not noticing is worse than the bug: commands go to a
     * handle wrapping whatever the number now names.
     *
     * A HEURISTIC, not a guarantee. usbfs allocates inode numbers and may
     * reuse one after its node is destroyed, so a match is strong evidence
     * and not proof. It is no substitute for releasing claims. */
    dev_t reg_dev;
    ino_t reg_ino;

    /* Dedicated libusb context for this device - deliberately NOT the
     * shared default (NULL) context. A playback session can be open for
     * hours, with reads driven directly by libVLC's own internal
     * threads; it must not share init/exit lifecycle or event handling
     * with a detection scan that might run concurrently on a different
     * device. See the registry section's doc comment in the public
     * header for the full rationale. */
    libusb_context *ctx;
    libusb_device_handle *handle;

    /* The interface number this device's Bulk-Only function actually
     * lives on, resolved from the configuration descriptor by
     * laser_find_bulk_endpoints(). NOT assumed to be 0: interface 0
     * is whatever the device chose to list first, which on a drive with a
     * front-panel HID or a second function is not the mass-storage one.
     *
     * Claim, release, Mass Storage Reset and GET MAX LUN must all name
     * this same number - naming a different one than the endpoints belong
     * to is the failure this field exists to make impossible. */
    uint8_t iface_num;

    unsigned char ep_in;
    unsigned char ep_out;
    uint32_t tag;

    /* USB identity of this device, read once at registration from the
     * device descriptor libusb already has cached (so it costs nothing).
     * Kept purely so that every subsequent log line can name the
     * hardware: an fd is meaningless in a bug report, whereas
     * vid:pid:bcd is exactly the tuple a per-device workaround would key
     * on, and the tuple a user can be asked to send back. */
    uint16_t vid;
    uint16_t pid;
    uint16_t bcd_device;

    /* Set once the residue-versus-wire-count contradiction has been
     * reported for this device, so it is logged once per session
     * instead of once per transfer. On a bridge that always gets the
     * residue wrong the condition holds for every chunk of every read -
     * tens of thousands of times over one film - and a warning repeated
     * that often buries every other log line and stops being read at
     * all. One occurrence carries the same information for a bug
     * report. */
    int residue_quirk_logged;

    /** Nonzero while a probe is in flight: shortens the BOT phase timeouts.
     *
     * Set and cleared around the command, under io_lock, by whoever issues
     * it. A probe is a command that needs no medium and that a working device
     * answers at once - INQUIRY today. The read timeouts are sized for a
     * drive that seeks or spins up; a probe inheriting them costs seconds to
     * establish that a device is not answering. */
    int probe_timeouts;

    /* Logical Unit Number to address in every CBW for this device.
     * Almost always 0, but not always: a device may expose several LUNs
     * behind one Bulk-Only interface, and combo drives that pair an
     * optical drive with an SD/microSD card reader in the same enclosure
     * do exactly that - with no guarantee the optical unit is LUN 0. On
     * such a drive, hardcoding 0 sends every command to the card reader,
     * so the disc is never found and the device looks like it simply
     * isn't an optical drive at all. Resolved once at registration by
     * laser_probe_lun(). */
    uint8_t lun;

    /* NOTE: there is deliberately no "last CSW status" field here. The
     * raw bCSWStatus of a command is passed back through an
     * out-parameter of bot.c's laser_bot_send_locked() instead,
     * so that each caller keeps its own copy on the stack. Storing it on
     * the device would make it shared mutable state with a lifetime far
     * longer than its meaning: it is only valid between issuing a
     * command and inspecting that command's outcome, and the natural
     * response to a FAIL - sending REQUEST SENSE to find out why - is
     * itself a BOT transaction that would overwrite it before the
     * original caller had finished with it. */

    /* Serializes every BOT transaction on this device: the protocol is
     * stateful (one CBW, its data phase, then its CSW - strictly in
     * order, never interleaved with another command), so at most one
     * thread may be mid-transaction on a given device at any time. Held
     * for the full duration of laser_scsi_cdb(), including its
     * internal retries. */
    pthread_mutex_t io_lock;

    /* CSS authentication session: mutual exclusion between CONSUMERS, one
     * layer above io_lock, and the reason both exist.
     *
     * io_lock protects a TRANSACTION - one CBW, its data phase, its CSW. The
     * right unit for reads, where each command is complete in itself.
     *
     * CSS authentication is a SEQUENCE whose state lives in the drive BETWEEN
     * transactions: AGID, host challenge, key1, drive challenge, key2, ASF.
     * io_lock serializes each step perfectly and protects none of what joins
     * them. A second consumer requesting an AGID in the gap between two of our
     * steps gets its own clean transaction, consumes one of the drive's four
     * AGIDs, and may invalidate ours - every lock acquisition correct, result
     * wrong. Observed in the field: the medialibrary preparser fetching
     * "laser/dvd://<fd>" on its own thread, interleaved between two
     * commands of a probe on the same token.
     *
     * NOT A pthread_mutex, and NOT OWNED BY A THREAD. A libdvdcss instance
     * opens its session when the handle is created and closes it when the
     * handle is destroyed; in between, its key commands are issued from
     * whichever thread libVLC happens to be reading on - dvdcss_disckey() runs
     * on the demux open thread, dvdcss_titlekey() later on the playback
     * thread. A pthread mutex must be released by the thread that took it and
     * would refuse the second thread its own session, so ownership is an
     * opaque cookie (the dvdcss_t) and the wait is a condition variable. */
    pthread_mutex_t css_mtx;
    pthread_cond_t  css_cv;
    int             css_open;
    const void     *css_owner;
} laser_entry_t;

/**
 * Look up the entry for a token. Returns NULL if the token is not
 * registered - which now means exactly one thing, since a device is
 * registered by laser_acquire() and by nothing else, and a slot under
 * construction is not in the table at all: there is no "registered but not
 * usable yet" state for this to hide.
 *
 * The returned pointer is stable for the lifetime of the registration
 * (entries live in a fixed table, never moved or reallocated) - callers in
 * scsi.c may hold onto it across the whole duration of a transaction without
 * re-looking it up, as long as they do not race with the laser_release()
 * that drops the last claim on the same token (see that function's
 * threading contract in the public header).
 *
 * THE ONLY WAY IN, for everything except laser_acquire(). A command on a
 * token nobody has claimed is a caller-lifecycle bug, and it now says so
 * immediately instead of quietly registering a device that no consumer will
 * ever release.
 */
laser_entry_t *laser_lookup(int token);

/**
 * Mark this entry cancelled, and test that mark.
 *
 * Set by exactly one caller - the laser_release() that takes refs to zero,
 * immediately before teardown. There is no public laser_cancel() any more:
 * cancelling was always the last consumer saying "I am done with this
 * device", which is what dropping the last claim already says.
 *
 * The test is deliberately lock-free and is called from the retry loop and
 * the chunk loops - see laser_entry_t::cancelled. It is NOT called from the
 * spin-up wait: that runs inside laser_acquire(), under g_registry_lock, on
 * an entry not yet published, so nothing can cancel it and the check there
 * was unreachable by construction.
 */
void laser_set_cancelled(laser_entry_t *entry);
int  laser_is_cancelled(const laser_entry_t *entry);

/**
 * Is a CSS session currently open on this entry, by any consumer?
 *
 * Deliberately NOT "is it mine": the enforcement this answers is "no key
 * command may touch authentication state unless SOMEBODY declared a session",
 * which is what stops an undeclared consumer stealing an AGID. Which consumer
 * it is is checked only at end(), by cookie.
 */
int laser_css_session_is_open(laser_entry_t *entry);

/**
 * Does this CDB change the drive's CSS authentication state?
 *
 * Keyed on the KEY FORMAT, not the opcode: REPORT KEY and READ DVD STRUCTURE
 * each carry both state-changing steps and harmless read-only queries, and
 * conflating them refuses queries that libdvdcss legitimately issues outside
 * any session (dvdcss_test() reads copyright and RPC state before there is an
 * AGID to speak of). Used for two things that need exactly the same answer:
 * whether a session is required, and whether a retry is safe.
 *
 * The two opcodes are keyed in OPPOSITE DIRECTIONS, which is deliberate and
 * documented at the switch: REPORT KEY and SEND KEY name the formats that do
 * change state, READ DVD STRUCTURE names the two that do not. See the
 * implementation for why the open-ended one is the one that gets the
 * whitelist.
 */
int laser_cdb_changes_css_state(const uint8_t *cdb, int cdb_len);

/**
 * Select this device's Bulk-Only mass storage interface and its bulk
 * IN/OUT endpoint pair, filling entry->iface_num and entry->ep_in/ep_out.
 *
 * The interface is chosen on the evidence of the configuration
 * descriptor (class 0x08, protocol 0x50, with a usable bulk pair of its
 * own), not by index - see the implementation for why assuming interface
 * 0 is wrong on some enclosures.
 *
 * Must run BEFORE the interface is claimed, since its result is what
 * says which interface to claim. Reading the configuration descriptor
 * needs no claim.
 *
 * @return 0 on success, -1 if the device exposes no such interface.
 */
int laser_find_bulk_endpoints(laser_entry_t *entry);

/**
 * USB Mass Storage Reset (class request, addressed to entry->iface_num)
 * + clear_halt on both endpoints,
 * to normalize the device's BOT state machine before the first SCSI
 * command is ever sent to it. Best-effort: failure to perform the reset
 * itself is logged but not fatal (some non-compliant drives don't
 * implement the request; the clear_halt calls are independently useful
 * regardless, and the first real SCSI command will reveal if the device
 * turns out to be unusable).
 */
void laser_mass_storage_reset(laser_entry_t *entry);

/**
 * Wake the drive and wait for its medium to become ready (TEST UNIT READY
 * in a loop), before any read is attempted on it. Called once at
 * registration, before LUN discovery, since a cold drive must be spun up
 * before it will answer commands reliably. Best-effort: a drive that
 * never reports ready still falls through.
 *
 * BOUNDED IN REAL TIME, which matters to the caller: this is the slowest
 * step of registration, it runs with the registry's lazy-registration
 * lock held, and it is not cancellable from outside. Its ceiling is
 * LASER_SPINUP_MAX_WALL_MS (15s), reached only by a drive that has
 * stopped answering; a working drive, present or empty, returns in well
 * under a second. See the implementation in scsi.c for the full
 * rationale and both budgets.
 */
void laser_wait_until_ready(laser_entry_t *entry);

/** Answers of laser_probe_lun(). */
enum {
    /** INQUIRY says direct-access block device. Not an optical drive. */
    LASER_OPTICAL_NO = 0,
    /** INQUIRY answered, and not with a type that rules an optical drive out. */
    LASER_OPTICAL_YES = 1,
    /** INQUIRY went unanswered. Neither a yes nor a no - see below. */
    LASER_OPTICAL_NO_ANSWER = -1,
    /** The device left the bus during the probe. Not "would not answer" but
     * "is not there": kept apart from LASER_OPTICAL_NO_ANSWER because the
     * caller's response differs. Silence is a reason to register the device
     * anyway and let a read fail quickly; absence is a reason not to register
     * it at all, since every field the entry would carry describes a device
     * that has gone. */
    LASER_OPTICAL_GONE = -2,
};

/**
 * Work out which Logical Unit on this device is the optical drive, store it
 * in entry->lun, and say whether it is one.
 *
 * ONE FUNCTION, because selecting a unit and classifying it are one question
 * asked once. Split in two, the second half would re-issue the very INQUIRY
 * the choice had just been made on, and the two halves could disagree about
 * what "optical" means since each would classify the peripheral device type
 * itself.
 *
 * Issues GET MAX LUN (the Bulk-Only class request) and an INQUIRY per unit,
 * keeping the first whose peripheral device type says CD/DVD. Falls back to
 * LUN 0 whenever anything is unsupported, fails, or comes back inconclusive
 * - which is both what the class specification prescribes for a device that
 * stalls GET MAX LUN, and the right answer for the overwhelming majority of
 * drives, which have exactly one unit.
 *
 * The classification is deliberately asymmetric: only a direct-access block
 * device is rejected. An optical drive and a card reader are identical at the
 * USB descriptor level (Mass Storage / Bulk-Only / SCSI all three), so
 * without this the only way to tell them apart is to wait out a spin-up on a
 * device that has no mechanism to spin, then read no filesystem from it -
 * several seconds, on every browse, for a foregone conclusion. Combo
 * enclosures pair the two, so this is not a corner case. Silence is not a
 * no, so it gets its own answer rather than being folded into either.
 *
 * LASER_OPTICAL_GONE IS NOT LASER_OPTICAL_NO_ANSWER. A device that has left
 * the bus fails every command instantly rather than timing out, and nothing
 * about it will change: the same drive plugged back in arrives as a new
 * descriptor and a new token.
 *
 * LASER_OPTICAL_NO_ANSWER MATTERS BEYOND "we could not tell". INQUIRY is
 * mandatory and needs no medium, so a device that will not serve it is not a
 * device that is still warming up: it is one that is not answering at all.
 * Waiting for such a device to become ready can only spend the whole spin-up
 * budget - seventeen seconds, measured on a card reader whose INQUIRY times
 * out - to conclude what this call already established. The caller skips
 * that wait on this answer, while still registering the device.
 *
 * Called once at registration, after the endpoints are known and the device
 * has been reset, since it needs to send real SCSI commands. Does NOT need
 * the medium: INQUIRY and GET MAX LUN are answered by a drive whose tray is
 * empty or still spinning up, which is why this runs BEFORE
 * laser_wait_until_ready() rather than after it. entry->lun is 0 (from the
 * caller's memset) until this runs, so a failure to call it at all degrades
 * to the previous behaviour rather than to something undefined.
 *
 * @return one of LASER_OPTICAL_YES, LASER_OPTICAL_NO, LASER_OPTICAL_NO_ANSWER
 *         or LASER_OPTICAL_GONE.
 */
int laser_probe_lun(laser_entry_t *entry);

/* ============================================================================
 * One Bulk-Only transaction (bot.c)
 * ========================================================================= */

/** Raw bCSWStatus values, which laser_bot_send_locked() reports through its
 * csw_status out-parameter. Here rather than in bot.c because they are that
 * parameter's value space, and scsi.c is what reads it. */
#define USB_BOT_STATUS_PASS        0x00
#define USB_BOT_STATUS_FAIL        0x01
/* Phase Error: the device and host disagree about the transfer that just
 * happened badly enough that the device's state machine is out of sync.
 * BBB 6.6.3 is explicit that this is the status a device returns when it
 * "may require a reset to recover", and 6.7 requires the host to perform a
 * Reset Recovery in response - clearing a stalled endpoint is NOT
 * sufficient, and issuing further commands before the reset is not
 * meaningful. */
#define USB_BOT_STATUS_PHASE_ERROR 0x02

/** The CBW itself could not be handed over, so the drive never saw the
 * command at all and its state is provably unchanged - always safe to
 * replay. */
#define BOT_FAIL_NOT_SENT    (-2)

/** The transaction ended in a state BBB requires a Reset Recovery for, and
 * laser_bot_send_locked() has already performed it. Retrying afterwards is
 * allowed but must be counted, since a device that lands here repeatedly is
 * broken rather than busy. */
#define BOT_FAIL_PHASE_ERROR (-3)

/** The device is no longer on the bus - unplugged, or its connection torn
 * down under us. Distinguished from an ordinary I/O failure because the
 * correct response is categorically different: nothing that waits or retries
 * can help, and every subsequent command will cost a full set of USB
 * timeouts to reach the same conclusion. A budget expressed in attempts
 * turns that into minutes of blocking. */
#define BOT_FAIL_NO_DEVICE   (-4)

/**
 * Perform exactly one Bulk-Only transaction: CBW, data phase in either
 * direction, CSW - and, where BBB requires it, the Reset Recovery that must
 * follow before any other command is sent.
 *
 * Knows nothing about what the CDB means. Every decision that needs to -
 * whether to retry, what the sense data implies, whether the command was
 * idempotent - belongs to scsi.c, and this return value is what it decides
 * on:
 *
 *   0                     the command completed and the CSW says PASS
 *   BOT_FAIL_NOT_SENT     the drive never received it; replay is free
 *   BOT_FAIL_PHASE_ERROR  host and device disagreed; recovery already done
 *   BOT_FAIL_NO_DEVICE    the device is gone; no budget can help
 *   -1                    anything else: the CBW went out, so the drive HAS
 *                         received the command and may have executed it, in
 *                         full or in part, even though we failed to read the
 *                         data phase or the CSW. Replaying is only safe for
 *                         commands that are idempotent.
 *
 * @param csw_status optional, may be NULL. Receives the raw bCSWStatus of
 *        THIS call, or -1 when the attempt failed below the level of "a valid
 *        CSW was received at all" (transport error, stall, CBW never sent),
 *        in which case REQUEST SENSE is not meaningful because there is no
 *        completed command for the drive to explain.
 *
 *        An out-parameter rather than a field on `entry` on purpose: the
 *        value only means anything in the window between issuing a command
 *        and inspecting its outcome, and it is trivially clobbered - the
 *        caller's natural reaction to a FAIL is to send REQUEST SENSE, which
 *        comes right back through this same function and would overwrite any
 *        shared copy before the caller was done with it.
 *
 * Caller MUST already hold entry->io_lock.
 */
int laser_bot_send_locked(laser_entry_t *entry,
                          const uint8_t *cdb, int cdb_len,
                          uint8_t *data, int data_len, int data_in,
                          int *actual_len, int *csw_status);

#endif /* LASER_INTERNAL_H */
