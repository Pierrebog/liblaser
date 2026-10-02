/*****************************************************************************
 * laser.h: public API of liblaser - Library for Accessing SCSI External
 *          Readers
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
 * SCSI-MMC to an optical drive over USB Bulk-Only Transport, from userspace.
 * Used by:
 *
 *   - VLC's laser access module: disc identification (laser_disc.h) and the
 *     sector reads everything layered on it is fed from;
 *   - libdvdcss: CSS authentication - REPORT KEY, SEND KEY, READ DVD
 *     STRUCTURE;
 *   - VLC's cdrom.c, for the cdda and vcd modules: READ TOC, READ CD.
 *
 * None of them knows about libusb, BOT, CBWs or CSWs: they see a registry
 * token, and either the raw CDB primitive or the LBA-aware block helpers.
 *
 * THREADING: every function here may be called from any thread, concurrently
 * for different tokens. Calls on one token are serialized internally - the
 * BOT protocol carries one command at a time per device - so callers need no
 * locking of their own.
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
 * By default, the system log under the "Laser" tag on Android, stderr
 * elsewhere. Applications route it where they want with a callback, as with
 * libbluray's bd_set_debug_handler() or libdvdnav's logger.
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
 * Process-wide, not per token: the messages that matter most are about
 * tokens that do not exist yet or no longer do - a failed registration, an
 * unbalanced release.
 *
 * Set it BEFORE the first call to anything else. The pointer is not locked:
 * changing it while another thread logs is a data race, and a lock on every
 * log line, transaction path included, is not worth it for a value set once
 * at startup.
 */
void laser_set_log_cb(laser_log_cb_t cb, void *opaque);

/** Outcome of any call in this header that reports one. */
typedef enum {
    /** Command completed, data (if any) is valid. */
    LASER_OK = 0,
    /**
     * The token is not usable: no claim is held on it, or its one-time
     * device setup failed - the fd is not a USB device connection, or the
     * device has no Bulk-Only mass storage interface, or is not an optical
     * drive.
     *
     * Also returned when the registry's table of LASER_MAX_DEVICES (8)
     * entries is full, which only fds never released can cause: a lifecycle
     * leak upstream, which retrying with another device will not fix.
     */
    LASER_ERR_NO_SUCH_TOKEN = -1,
    /**
     * Transport failure (USB I/O error, a stall that could not be cleared),
     * or a transient SCSI condition retried until the attempt budget ran
     * out. The drive is misbehaving: not worth retrying at a higher level.
     */
    LASER_ERR_IO = -2,
    /**
     * The disc is gone or has been swapped: sense MEDIUM NOT PRESENT or
     * MEDIUM MAY HAVE CHANGED. Never retried internally. Callers should
     * surface it as a fatal, immediate playback error.
     */
    LASER_ERR_MEDIA_GONE = -3,
    /**
     * The drive understood the command and refuses it as issued: sense key
     * ILLEGAL REQUEST or DATA PROTECT - a malformed CDB, an LBA past the
     * medium, content the drive will not serve in its current state.
     *
     * Kept apart from LASER_ERR_IO because the responses are opposite: an
     * I/O error may be a scratch, and the next sector may read; a refusal
     * belongs to the command, and reissuing it gets the same answer forever.
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
     * type outside laser_cd_sector_t, a CSS command with no session open, a
     * NULL or re-entrant session cookie. Nothing about the device is touched.
     *
     * Kept apart from LASER_ERR_IO, so that a bug in the caller is not taken
     * for a hardware failure and answered with retries. */
    LASER_ERR_INVALID = -8,

    /** The last claim on this token was dropped while the operation was
     * running (see laser_release()). Kept apart from LASER_ERR_IO, so that a
     * consumer unwinding on purpose does not log its own teardown as I/O
     * errors. */
    LASER_ERR_CANCELLED = -9,

    /**
     * The device has left the USB bus. Not a statement about the medium, a
     * sector or the command: there is nothing on the other end.
     *
     * TERMINAL FOR THIS TOKEN. A device back on the bus comes with a new fd
     * and a new token, so every further call on this one fails the same way.
     *
     * Unlike LASER_ERR_MEDIA_GONE, where the drive answered that its medium
     * is gone, here the drive did not answer at all: a consumer may offer
     * "insert a disc" for the one and "reconnect the drive" for the other.
     * Unlike LASER_ERR_IO, it must not be retried.
     */
    LASER_ERR_NO_DEVICE = -10,
} laser_status_t;

/* ============================================================================
 * Registry: token (== fd) <-> USB device
 *
 * The token is the fd itself: an open file descriptor on the USB device, with
 * whatever permission the platform requires already granted. On Android, the
 * one UsbDeviceConnection.getFileDescriptor() returns. This library neither
 * obtains it nor checks how it was obtained, and never closes it.
 *
 * Consumers pass it around as decimal text - in an MRL, a device name, a
 * libdvdcss target - and read it back with laser_parse_token().
 *
 * A device is registered by laser_acquire() and by nothing else. Every other
 * function needs a claim already held, and answers LASER_ERR_NO_SUCH_TOKEN
 * otherwise.
 * ============================================================================ */

/**
 * Parse the decimal text form of a token, as it travels inside an MRL or a
 * device name: one or more ASCII digits and nothing else, no sign, no
 * whitespace, no trailing characters, and a value that fits in an int.
 *
 * @param str   the text to parse; NULL is rejected.
 * @param token receives the value on success, untouched otherwise.
 * @return 1 if @p str is a token, 0 otherwise.
 */
int laser_parse_token(const char *str, int *token);

/**
 * Declare that this consumer holds @p token, registering the device if this
 * is the first claim on it. Paired with laser_release().
 *
 * A token commonly has several consumers at once - playing a DVD, the access
 * module and libdvdcss both hold it - and the device is torn down only when
 * the last claim is released.
 *
 * THE FIRST CLAIM DOES THE DEVICE SETUP, and pays for it here rather than in
 * whichever thread issues the first command: a dedicated libusb context, the
 * kernel driver detached, the Bulk-Only interface found (not assumed to be 0)
 * and claimed, a Mass Storage Reset, the optical logical unit found, and a
 * wait for the drive to spin up. Well under a second on a working drive;
 * seconds on a cold one; at most 15 s on a drive that stops answering.
 * Registrations are serialized, so a second device's first claim waits
 * behind it. Do not call this from a thread that must stay responsive.
 *
 * A RECYCLED DESCRIPTOR IS REFUSED. If @p token is already registered but no
 * longer names the device it was registered on, a claim was never released
 * and the system has reused the fd number. This is refused and logged as an
 * error: re-registering would take the drive from a consumer that never let
 * go.
 *
 * @return LASER_OK - and only then must laser_release() be called - or
 *         LASER_ERR_NO_SUCH_TOKEN if the device could not be set up, or if
 *         the descriptor no longer names the device registered under it.
 */
laser_status_t laser_acquire(int token);

/**
 * Drop this consumer's claim on @p token. When the last one goes, the device
 * is torn down: its interface released, its libusb handle and context closed.
 * The fd is not closed; its owner does that once nothing refers to it.
 *
 * Safe on a token never acquired (logged, ignored). Serialized against a
 * registration in progress, which it never tears down.
 *
 * DROPPING THE LAST CLAIM CANCELS THE TOKEN. An operation still running on
 * another thread gives up at its next checkpoint - between retry attempts,
 * between the chunks of a block read - with LASER_ERR_CANCELLED, and the
 * teardown waits for a transaction already on the wire, which completes
 * within about one phase timeout. The cancellation is not exposed on its
 * own: it is token-wide and sticky, and one consumer of several would
 * disable the drive for all the others.
 *
 * The caller must still ensure that nothing calls into the token after its
 * last release has begun: the registry entry is freed when teardown ends.
 *
 * A CSS session still open when the last claim goes is logged as an error -
 * a consumer outlived the device it was authenticating against.
 */
void laser_release(int token);

/**
 * Whether the drive behind @p token ended its readiness wait without ever
 * answering ready.
 *
 * laser_acquire() waits up to 15 s for the medium. A drive that never settled
 * in that time answers every later command the same way, each paying its own
 * retry budget to say so; this says it in one lookup and no command.
 * laser_disc_identify() checks it for that reason.
 *
 * ADVISORY: registration succeeds either way and nothing is refused because
 * of it. A drive that never reported ready may still read.
 *
 * @param token the registry token (the fd)
 * @return 1 if the token is registered and its drive never became ready,
 *         0 otherwise - including for a token that is not registered.
 */
int laser_token_not_ready(int token);

/* ============================================================================
 * Low-level: one raw SCSI CDB, one BOT transaction
 * ============================================================================ */

/**
 * Send one SCSI CDB over USB Bulk-Only Transport and wait for its status,
 * with the data phase (if any) in the requested direction.
 *
 * Handled internally:
 *   - CBW/CSW framing, stall recovery and Reset Recovery;
 *   - retries of transient conditions, up to six attempts: UNIT ATTENTION at
 *     once, NOT READY and anything unexplained after a short delay;
 *   - no retry at all on MEDIUM NOT PRESENT and MEDIUM MAY HAVE CHANGED
 *     (LASER_ERR_MEDIA_GONE), nor on a refusal (LASER_ERR_REFUSED and the
 *     copy-protection statuses) - waiting resolves none of them;
 *   - no retry of a command that changes CSS authentication state once its
 *     CBW has reached the drive: an AGID request, a handshake step, a key
 *     read, an AGID invalidation. Each accepted one advances the drive's
 *     state machine or takes one of its four AGIDs, so a replay would
 *     desynchronise host and drive. Such a failure is returned at once,
 *     classified by its sense data like any other - LASER_ERR_REGION,
 *     LASER_ERR_NO_KEY, LASER_ERR_MEDIA_GONE... - or as LASER_ERR_IO;
 *     restarting the handshake is libdvdcss's job. Read-only key queries -
 *     copyright, RPC state, ASF - are retried like any read;
 *   - serialization with every other command on the same token.
 *
 * A command that changes CSS authentication state is refused with
 * LASER_ERR_INVALID unless a CSS session is open on the token (see below).
 *
 * ARGUMENT VALIDATION: a NULL or out-of-range cdb (cdb_len must be 1..16,
 * the CBW's command field being 16 bytes), a negative data_len, or a NULL
 * data with a non-zero data_len are rejected with LASER_ERR_INVALID and an
 * error-level log, before the token is looked up. Their natural symptoms
 * appear far from the cause: an overrun stack buffer, a drive waiting for a
 * data phase that never comes.
 *
 * @param token      The registry token; a claim must already be held.
 * @param cdb        The Command Descriptor Block, cdb_len bytes.
 * @param cdb_len    Length of cdb, in bytes: 6, 10 and 12 are the usual.
 * @param data       Buffer for the data phase. May be NULL if data_len is 0.
 *                   For a DATA-IN command (data_in=1), this is filled by
 *                   the device. For a DATA-OUT command (data_in=0), this
 *                   is sent to the device and left untouched by this call.
 * @param data_len   Length of the data phase, in bytes. Reads that may span
 *                   more than one transfer belong in laser_read_blocks() or
 *                   laser_read_cd_blocks(), which chunk them.
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

/* ============================================================================
 * CSS authentication sessions
 *
 * A transaction is self-contained, but an authentication is a sequence -
 * AGID, challenges, keys - whose state lives in the drive between commands.
 * Another consumer's AGID request in a gap takes one of the drive's four
 * AGIDs and may invalidate ours, and the transport cannot tell "the request
 * starting my handshake" from "someone else's": the bytes are the same. Only
 * the consumer knows a sequence is starting, so it declares a session.
 *
 * SCOPE: one session per consumer, for its whole lifetime - for libdvdcss,
 * from the stream open to dvdcss_close(). libdvdcss authenticates more than
 * once per disc, the title keys from the playback thread, and closing the
 * session in between would reopen the very gap it closes.
 *
 * OWNERSHIP IS A COOKIE, NOT A THREAD: any stable pointer identifying the
 * consumer - libdvdcss passes its dvdcss_t - since its key commands come from
 * several threads.
 *
 * CONTRACT
 *   - Commands that change authentication state (an AGID request, the
 *     handshake steps, a title/disc key read, an AGID invalidation) are
 *     refused with LASER_ERR_INVALID unless SOME session is open on the
 *     token.
 *     Read-only queries - copyright, RPC state, ASF - never need one.
 *   - end() must be called with the same cookie, on every path out. A leaked
 *     session blocks the next consumer for LASER_CSS_SESSION_MAX_WAIT_MS and
 *     then fails it.
 *   - Reads need no session: playback and authentication proceed together,
 *     serialized per transaction.
 * ============================================================================ */

/** Ceiling on how long laser_css_session_begin() waits for a session held
 * by another consumer.
 *
 * A handshake takes milliseconds; this bounds the failure of a consumer that
 * died with its session open. Past it, the next consumer gives up and CSS
 * degrades to a disc that will not play, rather than an application that
 * hangs. Public because callers need the number to judge whether being
 * blocked that long is survivable. */
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
 *         LASER_ERR_INVALID on a NULL @p owner or on a re-entrant begin() by
 *         the same owner, or LASER_ERR_IO if another consumer's session is
 *         still open after LASER_CSS_SESSION_MAX_WAIT_MS.
 */
laser_status_t laser_css_session_begin(int token, const void *owner);

/**
 * Close the session @p owner opened on @p token. A mismatched cookie, or no
 * open session, is logged and ignored: this runs on unwind paths where the
 * caller is already handling a failure.
 */
void laser_css_session_end(int token, const void *owner);

/* ============================================================================
 * High-level: LBA-aware block reads, chunked automatically
 *
 * These issue as many laser_scsi_cdb() transactions as a read needs, so that
 * no caller reimplements chunking. Use them for any read that may exceed one
 * transfer - 64 KiB at most: dvdread/dvdnav VOBU reads, the Aligned Units
 * libbluray asks the access module for, the UDF walk of disc identification.
 *
 * THE CHUNK SIZE IS NEGOTIATED WITH THE DEVICE. Some USB-ATAPI bridges fail
 * a 64 KiB data phase outright, where the same blocks read in smaller
 * commands go through. So a chunk that fails with LASER_ERR_IO is read again
 * at half the size, halving down to an 8 KiB floor; once smaller transfers
 * have read the whole range the larger one failed on, the smaller size is
 * kept for the device, for every consumer of the token. A scratch fails the
 * smaller reads too, and leaves the size alone.
 *
 * So the first read on a weak bridge is slower than the next ones, and an
 * unreadable sector costs a few extra commands before it is reported.
 * ============================================================================ */

/**
 * Read num_blocks 2048-byte sectors starting at lba, via READ(10), chunked
 * as needed. Used for DVD and BD sector reads: disc identification,
 * libdvdcss, libdvdread and libdvdnav, and the access module, through which
 * libbluray reads.
 *
 * @param buffer Must be at least num_blocks * 2048 bytes.
 * @return Number of blocks read (>= 0), or a negative laser_status_t value
 *         on error (cast to int) - the "negative on error, block count on
 *         success" convention of libdvdcss's block-read callbacks.
 *
 *         A read spanning several chunks can fail partway through:
 *           - LASER_ERR_MEDIA_GONE, LASER_ERR_NO_DEVICE and
 *             LASER_ERR_CANCELLED are returned as such even if earlier
 *             chunks succeeded: there is nothing left to read around;
 *           - any other error after at least one successful chunk is a
 *             SHORT READ, the count of blocks read so far, as a real block
 *             device returns on a scratched sector, so that the caller can
 *             use what was read and read around the bad area.
 *         A return of 0 means only that 0 blocks were requested: a read that
 *         obtains no block is an error, so that the common "advance by what
 *         was read, ask for the rest" loop always terminates.
 *
 *         On a negative return, whatever was already written into
 *         `buffer` is undefined and must not be used.
 */
int laser_read_blocks(int token, uint32_t lba, int num_blocks,
                      uint8_t *buffer);

/**
 * What kind of CD sector laser_read_cd_blocks() should ask the drive for.
 *
 * A SECTOR KIND, NOT A CDB ENCODING. READ CD carries the answer in two
 * unrelated bytes - an Expected Sector Type and a field-selection bitmap -
 * whose legal pairings are a property of the CD format. This library keeps
 * the pairing in one place rather than leaving every caller to get it right.
 *
 * EVERY KIND RETURNS 2352 BYTES PER SECTOR, so buffers and chunking are the
 * same for all, and callers may rely on it: an audio sector is 2352 bytes of
 * user data, a Mode 2 sector reaches 2352 with its headers and EDC/ECC.
 */
typedef enum {
    /** Red Book audio. */
    LASER_CD_SECTOR_AUDIO = 0,

    /** The XA Mode 2 Form 2 sectors that carry a Video CD's MPEG payload.
     * Returns the RAW sector, headers included - the caller is expected to
     * take the 2324 user-data bytes from offset 24 itself, which is what
     * VLC's cdrom.c does for every platform rather than per backend. */
    LASER_CD_SECTOR_MODE2_FORM2,

    /** Whatever the sector turns out to be: MMC's Expected Sector Type 000b,
     * "all types". The drive reads what is there instead of checking it
     * against a declaration first.
     *
     * For a track holding more than one form, which is the ordinary shape of
     * a Video CD: its ISO 9660 filesystem is Mode 2 Form 1 and its MPEG
     * payload Form 2, and a strict bridge refuses a Form 2 read of the
     * entry-points sector (151) with ILLEGAL MODE FOR THIS TRACK.
     *
     * Still 2352 bytes per sector, EDC/ECC included: a raw Form 1 sector is
     * 12 + 4 + 8 + 2048 + 280, a Form 2 one 12 + 4 + 8 + 2324 + 4. The caller
     * strides its buffer and takes the payload from offset 24 as for the
     * declared kinds. */
    LASER_CD_SECTOR_ANY,
} laser_cd_sector_t;

/**
 * Read num_blocks raw 2352-byte CD sectors starting at lba, via READ CD
 * (opcode 0xBE), chunked as needed. Used by VLC's cdda and vcd modules,
 * through cdrom.c.
 *
 * THE SECTOR KIND IS DECLARED, NOT DISCOVERED. The drive checks the Expected
 * Sector Type against the medium and refuses a mismatch rather than
 * converting: audio asked of a data track fails, and the reverse. The caller,
 * which knows from the TOC what it is addressing, makes the declaration; a
 * wrong one is an error, not a degradation. LASER_CD_SECTOR_ANY opts out, for
 * a track holding more than one form.
 *
 * A value outside laser_cd_sector_t is a caller bug, rejected with
 * LASER_ERR_INVALID and an error-level log before the token is looked up.
 *
 * @param sector_type Which kind of sector the addressed track holds.
 * @param buffer      Must be at least num_blocks * 2352 bytes, for any kind.
 * @return Number of blocks actually read (>= 0), or a negative
 *         laser_status_t value on error (cast to int). Short reads and
 *         terminal statuses behave exactly as in laser_read_blocks().
 */
int laser_read_cd_blocks(int token, uint32_t lba, int num_blocks,
                         laser_cd_sector_t sector_type, uint8_t *buffer);

/* ============================================================================
 * DVD region
 * ========================================================================= */

/**
 * Does this drive's region setting forbid the disc currently loaded?
 *
 * ADVISORY, NEVER AN OBSTACLE. A region mismatch stops CSS authentication
 * and nothing else - an unscrambled disc in a mismatched drive still plays -
 * so use this to explain a failure, not to refuse a disc. Everything
 * uncertain answers 0: a drive that will not report its RPC state, a non-DVD
 * medium, a drive that enforces nothing (RPC-1) or has no region set yet, a
 * region-free disc. A wrong check must not condemn a disc that would play.
 *
 * Two commands, the second only if needed: REPORT KEY format 08h for the
 * drive's RPC state, READ DVD STRUCTURE format 01h for the disc's copyright
 * information.
 *
 * The masks are "one bit per region, SET means PROHIBITED", as both
 * structures carry them: a region-1 disc reads 0xFE, a drive set to region 2
 * reads 0xFD. Drive and disc agree when their permitted sets - the
 * complements - intersect. They are handed back raw, for the caller to
 * render.
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
 * Three statuses are positional - LASER_ERR_SCRAMBLED, LASER_ERR_REGION and
 * LASER_ERR_REFUSED: the drive reached those sectors and declined to hand
 * them over. Another range may well read; the same one will not.
 *
 * The others are not. LASER_ERR_MEDIA_GONE, LASER_ERR_NO_DEVICE and
 * LASER_ERR_CANCELLED concern the session or the hardware. LASER_ERR_NO_KEY
 * concerns the authentication state: the same blocks read once a handshake
 * succeeds. LASER_ERR_NO_SUCH_TOKEN and LASER_ERR_INVALID concern the caller.
 *
 * LASER_ERR_IO IS NOT POSITIONAL, though it could go either way: a scratch
 * produces it, and so does a bridge having a bad day. By the time it reaches
 * a caller, the block helpers have already tried smaller transfers, and a
 * caller treating it as positional would start recording sectors as bad on a
 * drive whose whole conversation is failing.
 *
 * Here rather than in each caller, so that a status added later is
 * classified where it is defined.
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
