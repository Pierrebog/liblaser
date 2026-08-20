/*****************************************************************************
 * laser.h: public API of liblaser - Library for Accessing SCSI External
 *          Readers
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
 * This is the single source of truth for "how do we talk SCSI-MMC to an
 * optical drive over USB Bulk-Only Transport, from userspace" - used by:
 *
 *   - the "laser" VLC access module (disc classification: is this a
 *     CD audio / Video-CD / DVD-Video / BD-Video disc, and what's its name;
 *     and the sector reads that feed everything layered on top of it)
 *   - libdvdcss (ioctl.c, HAVE_LASER branches: CSS authentication,
 *     REPORT KEY / SEND KEY / READ DVD STRUCTURE)
 *   - VLC's cdda module (cdrom.c, HAVE_LASER branches: READ TOC,
 *     READ CD)
 *
 * None of those know about libusb, BOT, or CBW/CSW - they only see a
 * registry token (an opaque int, see below) and either the low-level CDB
 * primitive or one of the LBA-aware block helpers.
 *
 * THREADING: every function in this header is safe to call from any
 * thread, including concurrently from different threads for DIFFERENT
 * tokens. For a SINGLE token, calls are automatically serialized
 * internally (the BOT protocol is stateful - one command in flight at a
 * time per device) - callers never need their own external locking around
 * these calls.
 *****************************************************************************/

#ifndef LASER_H
#define LASER_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Logging
 *
 * By default this library writes to whatever its platform offers - the
 * system log under the "Laser" tag on Android, stderr elsewhere - which is
 * a guess, and one no application should be stuck with. So the destination
 * is a callback, exactly as libbluray's bd_set_debug_handler() and
 * libdvdnav's logger do it, and for the same reason: a library that has
 * decided where its diagnostics go has decided it for every application
 * that embeds it.
 * ============================================================================ */

typedef enum {
    LASER_LOG_ERROR = 0,
    LASER_LOG_WARN,
    LASER_LOG_INFO,
} laser_log_level_t;

/**
 * @param opaque the pointer given to laser_set_log_cb()
 * @param level  severity
 * @param msg    the formatted message, NUL-terminated, WITHOUT a trailing
 *               newline and without the tag - the callback decides both
 */
typedef void (*laser_log_cb_t)(void *opaque, laser_log_level_t level,
                               const char *msg);

/**
 * Route this library's diagnostics to @p cb. Passing NULL restores the
 * platform default described above.
 *
 * PROCESS-WIDE AND NOT PER-TOKEN, because the messages this library most
 * needs to deliver are the ones about a token that does not exist yet or no
 * longer does - a registration that failed, an unbalanced release. Attaching
 * the sink to a device would lose exactly those.
 *
 * Set it BEFORE the first call to anything else. There is no locking around
 * the pointer: changing it while another thread is logging is a data race,
 * and the cost of preventing it - a lock taken on every log line, including
 * inside the per-transaction hot path - is not worth paying for a value that
 * is set once at startup in every real use.
 */
void laser_set_log_cb(laser_log_cb_t cb, void *opaque);

/** Outcome of any call in this header that reports one.
 *
 * Declared up here, before the first function that returns it, rather than
 * next to laser_scsi_cdb() where it started life: it was that function's
 * return type alone until acquiring a device and opening a CSS session
 * started reporting through it too. */
typedef enum {
    /** Command completed, data (if any) is valid. */
    LASER_OK = 0,
    /**
     * The fd was not usable: the one-time device setup (wrapping it,
     * selecting its Bulk-Only mass storage interface and that
     * interface's bulk endpoint pair, then claiming it) failed, either
     * because the fd itself is not a valid USB device connection or
     * because the device exposes no such interface - i.e. it doesn't
     * look like a Bulk-Only Transport mass storage device at all.
     *
     * Also returned, rarely, when the registry's fixed table is full.
     * The table holds LASER_MAX_DEVICES entries (8) and is only
     * exhausted by fds that were used and never unregistered, so this
     * particular occurrence means a lifecycle leak upstream rather than
     * anything wrong with this fd - which is worth knowing, because
     * unlike the other causes it does not go away by retrying with a
     * different device.
     */
    LASER_ERR_NO_SUCH_TOKEN = -1,
    /**
     * Transport-level failure (USB I/O error, stall that could not be
     * cleared, device unplugged...) or a SCSI failure whose sense
     * condition was transient and got retried until the attempt budget
     * was exhausted. Treat as "drive misbehaving/gone" - not worth
     * retrying again at a higher level.
     */
    LASER_ERR_IO = -2,
    /**
     * The command failed with a sense condition that means "the disc
     * itself is gone or has been swapped" (MEDIUM NOT PRESENT, or MEDIUM
     * MAY HAVE CHANGED) - deliberately NOT retried internally (see
     * laser_scsi_cdb() doc below). Callers (the patched ioctl_*,
     * dvdcss/cdda call sites) should surface this as a fatal,
     * immediate read/playback error rather than anything transient.
     */
    LASER_ERR_MEDIA_GONE = -3,
    /**
     * The drive understood the command and refuses it as issued: sense
     * key ILLEGAL REQUEST or DATA PROTECT. A malformed CDB, an LBA past
     * the medium, or content the drive will not serve in its current
     * state - a CSS-scrambled sector before authentication being the
     * case this project actually meets.
     *
     * Kept distinct from LASER_ERR_IO, which it would otherwise be
     * folded into, because the two call for opposite responses. ERR_IO
     * can mean a scratched sector: read on, read around it, the next one
     * may be fine. A refusal is a property of the command, not of one
     * spot on the disc - reissuing it produces the same answer forever.
     * A caller that cannot tell them apart must either give up on
     * recoverable damage or spin on an unrecoverable refusal.
     */
    LASER_ERR_REFUSED = -4,

    /** 05/6F/03 - the sector is scrambled and no session is in force. A
     * property of the SECTOR: it will refuse identically every time, and
     * recording that against its LBA is correct. */
    LASER_ERR_SCRAMBLED = -5,

    /** 05/6F/00, /01, /02 - authentication failed, key absent, or session
     * not established. A property of the DRIVE at this instant, not of any
     * sector: re-authenticating may make the very same read succeed.
     * Callers must NOT record it against an LBA. */
    LASER_ERR_NO_KEY = -6,

    /** 05/6F/04, /05 - the disc's region does not match the drive's. Not an
     * authentication problem; no key exchange will fix it, and the drive's
     * region must never be changed on the user's behalf. */
    LASER_ERR_REGION = -7,

    /** A caller violated this header's contract: a NULL or oversized CDB, a
     * negative data length, a data phase announced with no buffer, a sector
     * type outside laser_cd_sector_t.
     *
     * NOT used by laser_css_session_begin(), whose NULL-cookie rejection
     * predates this value and returns LASER_ERR_IO; changing it would change
     * what libdvdcss sees, so it is documented where it happens rather than
     * quietly moved.
     *
     * Kept apart from LASER_ERR_IO, because that value is documented as "the
     * drive is misbehaving or gone" - so a caller could not tell a bug in its
     * own code from a hardware failure, and the natural response to the latter
     * (retry, degrade, warn the user) is the wrong one for the former. Nothing
     * about the device is touched on this path: a malformed call never
     * triggers registration. */
    LASER_ERR_INVALID = -8,

    /** The last claim on this token was dropped - laser_release() cancels the
     * token as it tears the device down, so anything still running on another
     * thread gives up at its next checkpoint. Distinct from LASER_ERR_IO so
     * that a consumer unwinding on purpose can tell its own request apart
     * from a drive that failed, and log accordingly - a teardown that prints
     * I/O errors trains people to ignore I/O errors. */
    LASER_ERR_CANCELLED = -9,

    /**
     * The device has left the USB bus - unplugged, or its connection torn
     * down underneath us. Not a statement about the medium, about a sector
     * or about the command: there is nothing on the other end.
     *
     * TERMINAL FOR THIS TOKEN, and more strongly so than any other value
     * here. A disc can be swapped back in, a scrambled sector can become
     * readable once a handshake succeeds, a drive that refused a large
     * transfer will take a smaller one - but a device that has left the bus
     * comes back, if it comes back at all, with a new descriptor and a new
     * token. Nothing a caller does with THIS one can succeed, so a caller
     * that keeps asking is buying a full set of USB timeouts per attempt to
     * be told the same thing.
     *
     * Kept apart from LASER_ERR_MEDIA_GONE, which it superficially
     * resembles. That one means the drive answered and said the medium is
     * gone or changed - the drive is still there, and on a swap it will
     * answer again. This one means the drive did not answer because it is
     * not there. Consumers that latch either one and stop will behave
     * correctly with both; consumers that distinguish them can offer "insert
     * a disc" for the first and "reconnect the drive" for the second.
     *
     * Kept apart from LASER_ERR_IO for the reason that matters most in
     * practice: an I/O error invites a retry and this must not get one.
     */
    LASER_ERR_NO_DEVICE = -10,
} laser_status_t;

/** ============================================================================
 * Registry: token (== fd) <-> USB device handle
 *
 * The token is simply the fd itself - an already-open file descriptor for
 * the USB device, on which the caller already holds whatever permission the
 * platform requires. On Android that is the one returned by
 * android.hardware.usb.UsbDeviceConnection.getFileDescriptor(); elsewhere it
 * is an open descriptor on the device node. This library neither obtains it
 * nor checks how it was obtained.
 *
 * REGISTRATION IS laser_acquire(), and nothing else. The first claim on an
 * fd performs the one-time device setup - wraps the fd with a DEDICATED
 * libusb context (never the shared default/NULL context: a long-lived
 * playback session must not interfere with, or be interfered by, a detection
 * scan running concurrently on a different device), detaches any kernel
 * driver, selects and claims the Bulk-Only mass storage interface (NOT
 * assumed to be interface 0), discovers its bulk IN/OUT endpoints, performs a
 * Mass Storage Reset, works out which logical unit is the optical one, and
 * waits for the drive to spin up.
 *
 * Every other function here (laser_scsi_cdb(), laser_read_blocks(),
 * laser_read_cd_blocks(), the CSS session calls, laser_disc_identify())
 * requires that a claim already be held, and answers
 * LASER_ERR_NO_SUCH_TOKEN if one is not - see laser_acquire() for why
 * registration hangs off the claim and off nothing else.
 *
 * COST, because it is not negligible and it is paid inside laser_acquire():
 * on a working drive this is well under a second, but a cold mechanism
 * spinning a disc up legitimately takes seconds, and a drive that has stopped
 * answering is only abandoned after a 15s wall-clock ceiling. Registration is
 * also globally serialized, so a second device's first claim queues behind
 * it. Do not call laser_acquire() from a thread that must stay responsive.
 *
 * Ownership of the fd itself is NOT taken by any of this - whoever opened it
 * remains responsible for eventually closing it, after the last consumer has
 * called laser_release() below (see that function's doc comment for when).
 *
 * This is also what gets threaded, cast to/from a pointer, through the
 * dvdcss_stream_cb / dvd_reader_stream_cb / dvdcss->i_fd / vcddev_t
 * i_device_handle fields of the libraries listed in this header's
 * introduction - see each of their own patches for the exact cast
 * convention used.
 * ============================================================================ */

/**
 * Declare that this consumer holds @p token, registering the device if this
 * is the first claim on it. Paired with laser_release().
 *
 * THE ONLY WAY A DEVICE IS EVER REGISTERED. Every other entry point in this
 * header looks the token up and fails with LASER_ERR_NO_SUCH_TOKEN if there
 * is no entry - so a claim is not merely good manners, it is what makes the
 * device usable at all.
 *
 * A registration without a claim would be a state nothing could get out of:
 * teardown only ever happens when a claim count falls to zero, and a count
 * never incremented never falls. The entry would hold a libusb handle and one
 * of a small number of table slots for the life of the process - and once its
 * owner closed the descriptor underneath it and the OS recycled that number,
 * the stale entry would answer for whatever device came next.
 *
 * WHAT THE COUNT IS FOR, beyond that. A token routinely has several consumers
 * at once: playing a DVD means the access module and libdvdcss both hold the
 * same fd. Without a count, the first of them to finish tore the registration
 * down for all of them. That this was harmless rested on an ordering libVLC
 * provides and this library did not enforce - the demuxer is closed before
 * the access module, so only one consumer was ever left. Counting removes the
 * dependency on that ordering rather than documenting it.
 *
 * EAGER: the one-time device setup runs inside this call, so its cost (see
 * the registry section above - up to fifteen seconds on a cold or
 * unresponsive drive) is paid here rather than by whichever thread happens to
 * issue the first command. That is the point as much as a side effect: a
 * consumer that cannot get the drive learns it while it can still fail
 * cleanly.
 *
 * @return LASER_OK - and only then must laser_release() be called - or
 *         LASER_ERR_NO_SUCH_TOKEN if the device could not be set up.
 */
laser_status_t laser_acquire(int token);

/**
 * Drop this consumer's claim on @p token. When the last one goes, the USB
 * interface is released and the libusb handle and its dedicated context are
 * closed.
 *
 * Does NOT close the underlying fd - libusb_wrap_sys_device() never takes
 * ownership of it, and its owner closes it once nothing refers to that
 * descriptor any more.
 *
 * Safe to call on a token that was never acquired (no-op, logged). Internally
 * serialized against a registration another thread may be performing, so it
 * will never tear down a device that is still being set up.
 *
 * DROPPING THE LAST CLAIM ALSO CANCELS THE TOKEN. Every operation still
 * running on another thread gives up as soon as it can reach a checkpoint,
 * rather than running out its budgets: the flag is tested between retry attempts and between the chunks
 * of a block read, so the worst case for a wedged drive drops from about a
 * minute - six attempts, each with its own CBW, data and CSW timeouts, plus
 * the delays between them - to roughly one phase timeout. Those operations
 * return LASER_ERR_CANCELLED, distinct from LASER_ERR_IO so a caller
 * unwinding on purpose can tell its own request apart from a drive that
 * failed.
 *
 * It does NOT interrupt a libusb_bulk_transfer() already handed to the
 * kernel. Getting to zero would mean libusb's asynchronous API and
 * libusb_cancel_transfer(), which rewrites the whole BOT transaction.
 *
 * WHY CANCELLING IS NOT SEPARATELY EXPOSED. The flag is sticky and
 * token-wide, while a claim is one of several. A consumer calling a public
 * cancel would disable the drive for every other consumer of the same token,
 * permanently - which is exactly the teardown-ordering dependency the count
 * exists to remove. Cancelling means "I am the last one out and I am done",
 * which is what dropping the last claim already says.
 *
 * The caller must still guarantee that no transaction is in flight on this
 * token on another thread when the LAST claim is dropped. Nothing inside this
 * library can prevent that one: the other thread legitimately obtained its
 * entry pointer before teardown began. The cancellation above is what makes
 * such a thread give up promptly rather than what makes it safe.
 *
 * The one thing checked rather than assumed is the CSS session: a session
 * still open when the last claim goes is logged as an error, because a
 * session spans several transactions and the "no transaction in flight"
 * guarantee says nothing about it.
 */
void laser_release(int token);

/* ============================================================================
 * Low-level: one raw SCSI CDB, one BOT transaction
 * ============================================================================ */

/**
 * Send one SCSI CDB over USB Bulk-Only Transport and wait for its status,
 * with the data phase (if any) in the requested direction.
 *
 * Internally handles, transparently to the caller:
 *   - CBW/CSW framing and stall recovery (clear_halt on stalled endpoints)
 *   - retry of transient conditions: UNIT ATTENTION (retried immediately,
 *     the condition is cleared by the act of reporting it) and NOT READY/
 *     becoming-ready (retried after a short delay) - up to
 *     LASER_MAX_RETRIES attempts total (a build constant private to
 *     scsi.c, not exposed here; currently 6, matching the budget
 *     already validated for disc classification)
 *   - retries apply to DATA-IN and no-data commands only. A DATA-OUT
 *     command (in practice SEND KEY) is NOT retried once its CBW has
 *     reached the drive: SEND KEY is a step in the CSS authentication
 *     handshake, and each step the drive accepts advances its internal
 *     state machine, so re-sending one that may already have been
 *     executed desynchronises host and drive. Such a failure comes back
 *     as LASER_ERR_IO on the first attempt; recovering from it
 *     (invalidating the AGID and restarting the handshake) belongs to
 *     the CSS layer in libdvdcss, which already implements exactly that
 *     and has the context this one lacks. The single exception is a
 *     failure to hand the CBW over at all - the drive provably never saw
 *     the command, so it is retried like any other.
 *   - immediate, non-retried failure on MEDIUM NOT PRESENT (no disc) and
 *     MEDIUM MAY HAVE CHANGED (disc swapped/ejected mid-transaction) -
 *     these can never be resolved by waiting, and retrying them would
 *     only make an ejection feel sluggish to the user
 *   - serialization: this call blocks until it can acquire the token's
 *     internal per-device lock, so it is always safe to call concurrently
 *     from multiple threads for the same token
 *
 * ARGUMENT VALIDATION: this function rejects, with LASER_ERR_INVALID and
 * an error-level log, calls that violate the contract below - a NULL or
 * out-of-range cdb (cdb_len must be 1..16, the CBW's command field being
 * a fixed 16 bytes), a negative data_len, or a NULL data with a non-zero
 * data_len. These are caller bugs rather than device conditions, and are
 * caught here because their natural symptom appears far from the cause:
 * an oversized CDB overruns a stack buffer, and a data phase announced
 * in the CBW but never performed leaves the drive waiting for bytes that
 * never arrive. Validated before the token is even looked up, so a
 * malformed call is reported as a malformed call rather than as an
 * unregistered device.
 *
 * @param token      The fd (see the registry section above - there is
 *                   no separate registration step, this is the same
 *                   fd used throughout; a claim must already be held).
 * @param cdb        The Command Descriptor Block, cdb_len bytes (6, 10,
 *                   or 12 bytes are the common CDB sizes for the commands
 *                   this library's callers issue).
 * @param cdb_len    Length of cdb, in bytes.
 * @param data       Buffer for the data phase. May be NULL if data_len is 0.
 *                   For a DATA-IN command (data_in=1), this is filled by
 *                   the device. For a DATA-OUT command (data_in=0), this
 *                   is sent to the device and left untouched by this call.
 * @param data_len   Length of the data phase, in bytes. Callers issuing a
 *                   READ(10)/READ CD spanning more than one SCSI transfer
 *                   worth of data should use laser_read_blocks()/
 *                   laser_read_cd_blocks() instead of calling this
 *                   directly with a large data_len - see their doc for why.
 * @param data_in    1 for a DATA-IN command (e.g. REPORT KEY, READ TOC),
 *                   0 for DATA-OUT (e.g. SEND KEY), ignored if data_len is 0.
 * @param actual_len Optional (may be NULL): filled with the number of
 *                   bytes actually transferred in the data phase.
 * @return A laser_status_t. On LASER_OK, *data (for data_in)
 *         and *actual_len are valid.
 */
laser_status_t laser_scsi_cdb(int token,
                              const uint8_t *cdb, int cdb_len,
                              uint8_t *data, int data_len,
                              int data_in, int *actual_len);

/** ============================================================================
 * CSS authentication sessions
 *
 * WHY DECLARED RATHER THAN INFERRED. A transaction is self-contained: the
 * transport can work out everything it needs from the command in front of it.
 * A session cannot be inferred from any single command, because "the AGID
 * request that starts my handshake" and "the AGID request another consumer
 * made for its own reasons" are byte-identical. Only the consumer knows it is
 * about to perform a sequence, so only the consumer can say so.
 *
 * SCOPE: one session per CONSUMER, held for that consumer's whole lifetime -
 * for libdvdcss, from dvdcss_open_stream() to dvdcss_close(). Not per
 * handshake: libdvdcss authenticates more than once per disc (disc key, then a
 * title key per title, the latter from the playback thread), and a session
 * scoped to one handshake would have to be reopened between them, reopening
 * the very window this exists to close.
 *
 * OWNERSHIP IS A COOKIE, NOT A THREAD. @p owner is any stable pointer
 * identifying the consumer - the dvdcss_t works, and is what the patched
 * libdvdcss passes. A thread cannot be the owner because a consumer's key
 * commands legitimately come from several threads over its lifetime.
 *
 * CONTRACT
 *   - Commands that change authentication state (an AGID request, the
 *     handshake steps, a title/disc key read, an AGID invalidation) are
 *     refused with LASER_ERR_IO unless SOME session is open on the token.
 *     Read-only queries - copyright, RPC state, ASF - never need one.
 *   - end() must be called with the same cookie, on every path out. A leaked
 *     session blocks the next consumer for
 *     LASER_CSS_SESSION_MAX_WAIT_MS and then fails it.
 *   - Reads are unaffected and need no session: playback and an
 *     authentication may proceed concurrently, serialized per transaction by
 *     the per-device I/O lock.
 * ============================================================================ */

/** Ceiling on how long laser_css_session_begin() waits for a session held by
 * another consumer.
 *
 * Public because the contract above quotes it: a caller told "you may be
 * blocked here, and then fail" needs the number to decide whether that is
 * survivable, and a private name would leave the one documented consequence
 * of a leaked session unresolvable by the reader.
 *
 * A handshake is milliseconds; this is sized for the failure it bounds - a
 * consumer that opened a session and died without closing it. Without a
 * ceiling that leaks the drive's CSS capability until it is unplugged; with
 * it, the next consumer waits, gives up, and CSS degrades to "not
 * authenticated": a disc that will not play, loudly, instead of an app that
 * hangs. Long enough that a whole legitimate playback session's worth of key
 * work never trips it. */
#define LASER_CSS_SESSION_MAX_WAIT_MS 10000

/**
 * Open a CSS authentication session on @p token for @p owner. Blocks until any
 * session held by another consumer closes, up to
 * LASER_CSS_SESSION_MAX_WAIT_MS.
 *
 * The caller must already hold a claim on @p token (laser_acquire()); this
 * does not register anything. libdvdcss acquires immediately before calling
 * this, and releases the claim again if no session follows.
 *
 * @param owner stable, non-NULL pointer identifying the consumer.
 * @return LASER_OK - and only then must laser_css_session_end() be
 *         called - LASER_ERR_NO_SUCH_TOKEN if no claim is held on the token,
 *         or LASER_ERR_IO on timeout, on a re-entrant begin() by the same
 *         owner, or on a NULL @p owner (see LASER_ERR_INVALID's note: this
 *         one predates that value and keeps LASER_ERR_IO).
 */
laser_status_t laser_css_session_begin(int token, const void *owner);

/**
 * Close the session @p owner opened on @p token. A mismatched cookie, or no
 * open session, is logged and ignored: this runs on unwind paths where the
 * caller is already handling a failure, and a hard error would replace a
 * diagnosable problem with an undiagnosable one.
 */
void laser_css_session_end(int token, const void *owner);

/** ============================================================================
 * High-level: LBA-aware block reads, chunked automatically
 *
 * These build and issue as many laser_scsi_cdb() transactions as
 * needed, incrementing the LBA and decrementing the remaining count each
 * time, so callers never need to reimplement chunking themselves. Use
 * these instead of laser_scsi_cdb() directly for any read that could
 * span more than one transaction's worth of data (a private ceiling,
 * currently 64KB) in one go (dvdnav/dvdread VOBU reads, the Aligned Unit
 * reads libbluray asks the access module for, and the UDF walk disc.c
 * performs to identify a disc all potentially exceed the
 * single-transaction-safe size).
 *
 * THE CHUNK SIZE IS NEGOTIATED WITH THE DEVICE, not fixed. Some USB-ATAPI
 * bridges cannot carry a full 64 KiB data phase and fail the whole command
 * rather than returning a short one, while the same request split smaller
 * goes through. So a chunk that comes back LASER_ERR_IO is retried at half
 * the size, down to a floor, and the size that worked is remembered for that
 * device for the rest of its registration - by every consumer of the token,
 * not only the one that discovered it.
 *
 * Two consequences worth knowing. A read that would once have failed can now
 * succeed after spending a few extra commands, so the first read on a
 * misbehaving bridge is slower than the ones after it. And a genuinely
 * unreadable sector costs those extra commands too, since a scratched disc
 * and a weak bridge look identical from here until the smaller transfer is
 * tried; the retry stops as soon as a chunk is down to one block, where
 * there is nothing left to split.
 * ============================================================================ */

/**
 * Read num_blocks 2048-byte sectors starting at lba, via READ(10),
 * chunked as needed. Used by: DVD/UDF sector reads (disc.c's identification
 * walk, libdvdcss/libdvdnav/libdvdread block reads, and the access
 * module's own sector reads - which is how a BD-Video disc is served,
 * libbluray reading through the stream rather than through this header).
 *
 * @param buffer Must be at least num_blocks * 2048 bytes.
 * @return Number of blocks actually read (>= 0), or a negative
 *         laser_status_t value on error (cast to int) - callers
 *         wiring this into libdvdcss's block-read callbacks,
 *         which follow the "return -1 (or similarly negative) on error,
 *         non-negative block count on success" convention, can generally
 *         forward this return value as-is; see each callback's own patch
 *         for its exact expected contract.
 *
 *         A read spanning several internal chunks can fail partway
 *         through, and the two failure kinds are reported differently
 *         on purpose:
 *           - LASER_ERR_MEDIA_GONE is always returned as such, even
 *             if earlier chunks succeeded, since it can never be
 *             resolved by reading on (see that constant's own doc);
 *           - any other error after at least one successful chunk is
 *             reported as a SHORT READ (the count of blocks read so
 *             far), matching what a real block device does on a
 *             scratched sector, so the caller can use what was read and
 *             read around the bad area.
 *         A return of 0 therefore means one thing only: 0 blocks were
 *         requested. A read that asks for blocks and obtains none
 *         reports LASER_ERR_IO rather than a zero count, so that
 *         the common caller shape - advance by what was read, ask for
 *         the remainder - always terminates instead of looping forever
 *         on a count that never advances.
 *
 *         On a negative return, whatever was already written into
 *         `buffer` is undefined and must not be used.
 */
int laser_read_blocks(int token, uint32_t lba, int num_blocks,
                      uint8_t *buffer);

/**
 * What kind of CD sector laser_read_cd_blocks() should ask the drive for.
 *
 * NAMES A SECTOR KIND, NOT A CDB ENCODING, which is the whole point of the
 * enum. READ CD carries the answer in two unrelated bytes - an Expected
 * Sector Type in byte 1 and a field-selection bitmap in byte 9 - and the
 * legal pairings between them are a property of the CD format rather than of
 * any caller. Exposing the bytes would hand every caller a combination it
 * has no way to validate and every reason to get wrong; exposing the kind
 * lets this library keep the pairing in one place.
 *
 * BOTH KINDS RETURN 2352 BYTES PER SECTOR, so the buffer requirement and the
 * chunking are the same for either. That the two arrive at the same number
 * by different arithmetic - an audio sector IS 2352 bytes of user data,
 * while a Mode 2 Form 2 sector reaches it as 12 sync + 4 header + 8
 * sub-header + 2324 user data + 4 EDC - is a coincidence of the CD format,
 * but a stable one, and callers may rely on it.
 */
typedef enum {
    /** Red Book audio. Value 0 so that a zeroed structure asks for the kind
     * this function has always read, and so that the existing single-purpose
     * callers keep their behaviour by naming it explicitly. */
    LASER_CD_SECTOR_AUDIO = 0,

    /** The XA Mode 2 Form 2 sectors that carry a Video CD's MPEG payload.
     * Returns the RAW sector, headers included - the caller is expected to
     * take the 2324 user-data bytes from offset 24 itself, which is what
     * VLC's cdrom.c does for every platform rather than per backend. */
    LASER_CD_SECTOR_MODE2_FORM2,
} laser_cd_sector_t;

/**
 * Read num_blocks raw 2352-byte CD sectors starting at lba, via READ CD
 * (opcode 0xBE), chunked as needed. Used by: the cdda and vcd modules
 * (cdrom.c ioctl_ReadSectors, HAVE_LASER branch).
 *
 * THE SECTOR KIND IS NOT DISCOVERED, IT IS DECLARED. READ CD matches the
 * Expected Sector Type in the CDB against what the drive finds on the
 * medium, and a mismatch is refused rather than converted: asking for audio
 * over a data track fails, and so does the reverse. That is the useful
 * behaviour - it is the caller, which knows from the TOC what kind of track
 * it is addressing, that has the information - but it means @p sector_type
 * is part of the request and not a hint. Passing the wrong one does not
 * degrade, it returns LASER_ERR_IO.
 *
 * A value outside laser_cd_sector_t is rejected with LASER_ERR_INVALID and an
 * error-level log, before the token is looked up, on the same grounds as
 * laser_scsi_cdb()'s argument validation: it is a caller bug, and letting it
 * reach the drive would turn it into an obscure ILLEGAL REQUEST far from its
 * cause.
 *
 * @param sector_type Which kind of sector the addressed track holds.
 * @param buffer      Must be at least num_blocks * 2352 bytes, for either
 *                    kind.
 * @return Number of blocks actually read (>= 0), or a negative
 *         laser_status_t value on error (cast to int). Short reads and
 *         LASER_ERR_MEDIA_GONE behave exactly as in laser_read_blocks().
 */
int laser_read_cd_blocks(int token, uint32_t lba, int num_blocks,
                         laser_cd_sector_t sector_type, uint8_t *buffer);

/* ============================================================================
 * DVD region
 * ========================================================================= */

/**
 * Does this drive's region setting forbid the disc currently loaded?
 *
 * ADVISORY, AND NEVER AN OBSTACLE. A region mismatch stops CSS
 * authentication and nothing else - an unscrambled disc in a mismatched
 * drive still plays - so a caller should use this to explain a failure, not
 * to refuse a disc. Everything uncertain answers 0: a drive that will not
 * report its RPC state, a non-DVD medium, a drive that enforces nothing
 * (RPC-1), a drive with no region set yet, and a region-free disc all come
 * back "no mismatch", because a check that is wrong must not be able to
 * condemn a disc that would have worked.
 *
 * Two commands: REPORT KEY with key format 08h for the drive's RPC state,
 * and READ DVD STRUCTURE format 01h for the disc's copyright information.
 * Both are issued only as far as needed - if the drive enforces nothing, the
 * disc is never asked.
 *
 * The masks are "one bit per region, SET means PROHIBITED", as both
 * structures carry them: a region-1 disc reads 0xFE, a drive set to region 2
 * reads 0xFD. Drive and disc agree exactly when their permitted sets - the
 * complements - intersect, which is the comparison this performs. Rendering
 * either mask as something a person can read is the caller's business; this
 * hands back the raw bytes so it can.
 *
 * WHY THIS IS HERE. It is two hand-built CDBs and one comparison rule, all
 * three defined by MMC and DVD-Video rather than by any consumer - the same
 * reason every other command in this header is here rather than in whichever
 * module needed it first. The access module built these two by hand and was
 * the last place in this project outside the library doing so.
 *
 * @param token       the registry token; a claim must be held.
 * @param drive_mask  optional, may be NULL. Filled ONLY when this returns
 *                    non-zero.
 * @param disc_mask   likewise.
 * @return non-zero if the drive's region forbids this disc, 0 otherwise -
 *         including every case where the question could not be answered.
 */
int laser_region_mismatch(int token, uint8_t *drive_mask, uint8_t *disc_mask);

/* ============================================================================
 * Classifying a result
 * ========================================================================= */

/**
 * Is @p status a statement about the BLOCKS that were asked for, rather than
 * about the drive, the medium or the session?
 *
 * Three of the error values are positional - LASER_ERR_SCRAMBLED,
 * LASER_ERR_REGION and LASER_ERR_REFUSED. Each means the drive understood the
 * request, reached those sectors, and declined to hand them over. Asking for
 * a different range may well succeed, and asking for the same range again
 * will not.
 *
 * Everything else is not. LASER_ERR_MEDIA_GONE, LASER_ERR_NO_DEVICE and
 * LASER_ERR_CANCELLED are about the session or the hardware and say nothing
 * about any sector. LASER_ERR_NO_KEY is
 * about the authentication state, so the same blocks become readable once a
 * handshake succeeds - narrowing down which sector is to blame would be
 * looking for a culprit that does not exist. LASER_ERR_NO_SUCH_TOKEN and
 * LASER_ERR_INVALID are about the caller.
 *
 * LASER_ERR_IO IS DELIBERATELY NOT POSITIONAL, which is the one that could go
 * either way: a scratched sector produces it, and so does a bridge having a
 * bad day. It is excluded because the block helpers now negotiate the
 * transfer size downwards before reporting it (see the chunking section
 * above), so an I/O error that reaches a caller has already survived the
 * treatment that would have distinguished the two - and because a caller that
 * treats it as positional will start recording individual sectors as bad on a
 * drive whose whole conversation is failing.
 *
 * WHY THIS IS HERE rather than in each caller: it is a fact about this
 * header's enum, and it lives beside the code that decides which sense key
 * becomes which value. A consumer that classified these itself would be
 * writing down a rule it does not own, and would not learn about a value
 * added later - it would silently sort the new one into "not positional".
 *
 * @param status Takes an int, not a laser_status_t, because the callers that
 *        need this hold a value that is either a block count or a status -
 *        the return of laser_read_blocks(). Any value that is not one of the
 *        statuses above, including any non-negative count, answers 0.
 * @return non-zero if positional, 0 otherwise.
 */
int laser_status_is_positional(int status);

#ifdef __cplusplus
}
#endif

#endif /* LASER_H */