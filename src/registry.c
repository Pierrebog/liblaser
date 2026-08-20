/*****************************************************************************
 * registry.c: fd <-> USB device handle registry
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
 *
 * See laser.h for the full contract. Summary of what happens
 * here: an unseen token, the first time laser_acquire() is called on it,
 * gets registered - treated as an already
 * permission-granted fd - on Android, the one obtained through UsbManager,
 * which is the only thing on that system allowed to open a raw device path
 * under /dev/bus/usb for an unprivileged app - and wrapped
 * with a dedicated libusb context, ready for SCSI-MMC transactions via bot.c
 * and scsi.c.
 *
 * REGISTRATION HANGS OFF THE CLAIM, and off nothing else. laser_register()
 * below is static and reachable only from laser_acquire(); every other entry
 * point of this library looks a token up and fails if it is not there. A
 * device registered without a claim would be one nothing could destroy, since
 * teardown only ever happens on the 1 -> 0 transition of the claim count: it
 * would hold a libusb handle and one of LASER_MAX_DEVICES slots for the life
 * of the process, and once its owner closed the descriptor underneath it and
 * the OS recycled that number, it could be handed out for a completely
 * different device.
 *
 * Every consumer in this project acquires before it does anything - the laser
 * access module in both its Open paths, cdrom.c in ioctl_Open(), libdvdcss in
 * dvdcss_open().
 *****************************************************************************/

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#ifdef __ANDROID__
# include <android/log.h>
#endif

#include "laser.h"
#include "laser_internal.h"

/* Which clock the CSS condition variable runs on.
 *
 * CLOCK_MONOTONIC is what we want: laser_css_session_begin() waits against an
 * absolute deadline, and a wall-clock deadline is at the mercy of the wall
 * clock being stepped - which on Android is not exotic, NTP and the carrier's
 * time settling within seconds of a boot or a network change, exactly when a
 * browse is likely to be running.
 *
 * Asking for it needs pthread_condattr_setclock(), which bionic marks
 * __INTRODUCED_IN(21). Below that the declaration exists but the symbol does
 * not, so the call compiles and the LINK fails - which is how this was found,
 * on the 32-bit ABI, VLC-Android building armeabi-v7a against a lower API
 * level than the 64-bit ones:
 *
 *     undefined reference to 'pthread_condattr_setclock'
 *
 * So the clock is chosen at compile time and named ONCE. Both the condvar's
 * creation and css_deadline() read this macro, which is the whole point: the
 * two must agree, since pthread_cond_timedwait() interprets its timespec on
 * the condvar's own clock and would otherwise wait for the difference between
 * two epochs. On an API level without the call, the wait degrades to what it
 * was before the monotonic clock was requested at all - a backstop that a
 * clock step can stretch or shorten, which is survivable because the value is
 * a ceiling on a failure path and not a measurement. */
#if !defined(__ANDROID_API__) || __ANDROID_API__ >= 21
# define LASER_CSS_CLOCK CLOCK_MONOTONIC
#else
# define LASER_CSS_CLOCK CLOCK_REALTIME
#endif

/* ---------------------------------------------------------------------------
 * Logging - the sink for the LOGI/LOGW/LOGE macros in laser_internal.h.
 * ------------------------------------------------------------------------- */

#define LOG_TAG "Laser"

/* Bound on one formatted line. The longest this library produces is a
 * registration summary naming vid, pid, bcd, interface, both endpoints and
 * the LUN; 512 is several times that. Truncation would be silent, but the
 * alternative - allocating per line, on a path that runs inside the
 * transaction loop - trades a cosmetic failure for one that matters. */
#define LOG_LINE_MAX 512

static laser_log_cb_t g_log_cb;
static void          *g_log_opaque;

/* Where a line goes when no consumer has installed a sink of its own.
 *
 * PER PLATFORM, AND ONLY HERE. This is the one place in the library that
 * knows what operating system it is running on: everything above it deals in
 * libusb, SCSI and file descriptors, none of which need to know. A consumer
 * that wants its own sink calls laser_set_log_cb() and this is never reached.
 *
 * "%s" and not msg directly, in both branches: the message has already been
 * formatted, and anything in it that looks like a conversion is data by then
 * - a volume label, a device string. */
static void default_log_cb(void *opaque, laser_log_level_t level,
                           const char *msg)
{
    (void) opaque;

#ifdef __ANDROID__
    int prio;
    switch (level) {
    case LASER_LOG_ERROR: prio = ANDROID_LOG_ERROR; break;
    case LASER_LOG_WARN:  prio = ANDROID_LOG_WARN;  break;
    default:              prio = ANDROID_LOG_INFO;  break;
    }

    __android_log_print(prio, LOG_TAG, "%s", msg);
#else
    const char *prio;
    switch (level) {
    case LASER_LOG_ERROR: prio = "E"; break;
    case LASER_LOG_WARN:  prio = "W"; break;
    default:              prio = "I"; break;
    }

    /* stderr rather than stdout: a library has no claim on a program's
     * output, and a diagnostic that ends up piped into a consumer's data is
     * worse than one nobody reads. */
    fprintf(stderr, "%s/%s: %s\n", prio, LOG_TAG, msg);
#endif
}

void laser_set_log_cb(laser_log_cb_t cb, void *opaque)
{
    g_log_cb = cb;
    g_log_opaque = opaque;
}

void laser_log(laser_log_level_t level, const char *fmt, ...)
{
    laser_log_cb_t cb = g_log_cb;
    char line[LOG_LINE_MAX];

    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);

    if (cb != NULL)
        cb(g_log_opaque, level, line);
    else
        default_log_cb(NULL, level, line);
}

static laser_entry_t g_entries[LASER_MAX_DEVICES];

/** Guards insertion/removal/lookup of entries in g_entries (i.e. the
 * "in_use"/"token" bookkeeping) - NOT the USB I/O itself, which is
 * guarded per-entry by entry->io_lock. Held only for the short,
 * non-blocking critical sections below. */
static pthread_mutex_t g_table_lock = PTHREAD_MUTEX_INITIALIZER;

laser_entry_t *laser_lookup(int token)
{
    laser_entry_t *found = NULL;

    pthread_mutex_lock(&g_table_lock);
    for (int i = 0; i < LASER_MAX_DEVICES; i++) {
        /* `in_use` alone is enough: registration publishes it LAST, so a
         * half-built entry - one whose ctx, handle and endpoints are not yet
         * filled in, which lasts seconds - is not in the table to be found.
         * See the field's doc comment in laser_internal.h. */
        if (g_entries[i].in_use && g_entries[i].token == token) {
            found = &g_entries[i];
            break;
        }
    }
    pthread_mutex_unlock(&g_table_lock);

    return found;
}

/** The cancellation flag's only two accessors, kept together so the asymmetry
 * between them is visible in one place: the write is serialized, the read is
 * not. See the field's comment in laser_internal.h for why that is sound -
 * and why the read must NOT take the table lock, being on the transaction
 * path.
 *
 * The write has exactly one caller, laser_release() at the 1 -> 0 transition.
 * The table lock here is not what makes the flag visible - the only
 * transition is 0 -> 1, and a reader that misses it sees it one attempt later
 * - it is what keeps the write from racing with release_slot()'s memset. */
void laser_set_cancelled(laser_entry_t *entry)
{
    pthread_mutex_lock(&g_table_lock);
    entry->cancelled = 1;
    pthread_mutex_unlock(&g_table_lock);
}

int laser_is_cancelled(const laser_entry_t *entry)
{
    return entry->cancelled;
}

/** Picks a free slot for `fd` (used directly as the token - see the file
 * header) and zeroes it, or returns NULL if the fd is already registered or
 * the table is full.
 *
 * DOES NOT PUBLISH IT. The slot is left with in_use == 0, which means
 * laser_lookup() cannot find it, which means nothing can reach a half-built
 * entry - laser_register() sets in_use at the very end, once ctx, handle and
 * endpoints are all in place. That is what let the separate `ready` flag go.
 *
 * The "already registered" scan is therefore looking at published entries
 * only, and cannot see one still under construction. It does not need to:
 * every caller of this function holds g_registry_lock, and laser_acquire()
 * has already looked the token up under that same lock, so there is never
 * more than one registration in flight and never one for a token that
 * already has an entry. The scan remains as a cheap assertion of that.
 *
 * Caller MUST hold g_registry_lock. */
static laser_entry_t *reserve_slot(int fd)
{
    laser_entry_t *slot = NULL;

    pthread_mutex_lock(&g_table_lock);

    for (int i = 0; i < LASER_MAX_DEVICES; i++) {
        if (g_entries[i].in_use && g_entries[i].token == fd) {
            pthread_mutex_unlock(&g_table_lock);
            return NULL;
        }
    }

    for (int i = 0; i < LASER_MAX_DEVICES; i++) {
        if (!g_entries[i].in_use) {
            slot = &g_entries[i];
            memset(slot, 0, sizeof(*slot));
            slot->token = fd;
            break;
        }
    }

    pthread_mutex_unlock(&g_table_lock);
    return slot;
}

/** Undoes reserve_slot() on failure, so a failed registration doesn't leak
 * a permanently-reserved table entry.
 *
 * The memset is what frees the slot - in_use, token and every field behind
 * them go to zero together, which is also what reserve_slot() relies on when
 * it hands the slot back out. No explicit `slot->in_use = 0` follows: it
 * would restate one byte the memset has already written, and read as though
 * the memset were incomplete.
 *
 * Also used by teardown, which is why it is not named after the failure
 * path. */
static void release_slot(laser_entry_t *slot)
{
    pthread_mutex_lock(&g_table_lock);
    memset(slot, 0, sizeof(*slot));
    pthread_mutex_unlock(&g_table_lock);
}

/** Does the actual one-time device setup: wraps fd with a dedicated
 * libusb context, works out WHICH interface carries the Bulk-Only
 * function and where its endpoints are (interface 0 is not assumed - see
 * laser_find_bulk_endpoints()), claims that interface, resets BOT
 * state, settles on the optical logical unit and waits for the drive to spin
 * up. Internal only, and reachable only from laser_acquire() - see the file
 * header. Token and fd are the same value throughout this project (see
 * laser.h), so this only takes one parameter.
 *
 * Caller MUST hold g_registry_lock. */
static int laser_register(int fd)
{
    laser_entry_t *entry = reserve_slot(fd);
    if (entry == NULL) {
        LOGW("register(fd=%d): already registered, or table full", fd);
        return -1;
    }

    /* Full size until something proves otherwise. The slot was memset by
     * reserve_slot(), so this cannot be left to the zero: a cap of 0 would
     * clamp every chunk to a single block. */
    entry->max_transfer_bytes = LASER_MAX_BYTES_PER_TRANSFER;

    if (pthread_mutex_init(&entry->io_lock, NULL) != 0) {
        LOGE("register(fd=%d): pthread_mutex_init(io_lock) failed", fd);
        goto err_slot;
    }
    /* The CSS session's lock and condvar are created together with io_lock so
     * a reader sees at a glance that there are two levels of exclusion here
     * and that neither is optional. */
    if (pthread_mutex_init(&entry->css_mtx, NULL) != 0) {
        LOGE("register(fd=%d): pthread_mutex_init(css_mtx) failed", fd);
        goto err_io_mutex;
    }
    /* Created against LASER_CSS_CLOCK - see the macro above for why that is
     * not simply CLOCK_MONOTONIC, and why css_deadline() has to read the same
     * name.
     *
     * The attribute dance is skipped entirely when the clock is the default
     * one, rather than asking for CLOCK_REALTIME explicitly: on the API levels
     * this branch exists for, the function that would carry the request does
     * not exist either. */
#if LASER_CSS_CLOCK != CLOCK_REALTIME
    pthread_condattr_t css_cv_attr;
    if (pthread_condattr_init(&css_cv_attr) != 0) {
        LOGE("register(fd=%d): pthread_condattr_init failed", fd);
        goto err_css_mutex;
    }
    if (pthread_condattr_setclock(&css_cv_attr, LASER_CSS_CLOCK) != 0) {
        LOGE("register(fd=%d): pthread_condattr_setclock failed", fd);
        pthread_condattr_destroy(&css_cv_attr);
        goto err_css_mutex;
    }
    int cv_ret = pthread_cond_init(&entry->css_cv, &css_cv_attr);
    pthread_condattr_destroy(&css_cv_attr);
#else
    int cv_ret = pthread_cond_init(&entry->css_cv, NULL);
#endif
    if (cv_ret != 0) {
        LOGE("register(fd=%d): pthread_cond_init(css_cv) failed", fd);
        goto err_css_mutex;
    }

    /* Dedicated context - see laser_internal.h's doc comment on
     * laser_entry_t::ctx for why this must not be the shared/NULL
     * default context.
     *
     * NO_DEVICE_DISCOVERY, because this library never enumerates: it is
     * given a descriptor and works from that. On a system where the caller
     * cannot read /dev/bus/usb - Android, for an unprivileged app - libusb's
     * normal enumeration at init has nothing it is allowed to look at, and
     * depending on the version either fails outright or spends the attempt
     * walking a directory it cannot open. The supported arrangement is
     * exactly the one this file uses: no discovery, and a device obtained
     * solely by wrapping a descriptor its owner already opened
     * (libusb_wrap_sys_device below).
     *
     * Consequence worth stating: this context can never enumerate or
     * hotplug-notify. Neither is wanted here - the caller owns device
     * attach/detach through UsbManager and drives open/close explicitly -
     * but it does mean libusb's hotplug machinery is dead weight in this
     * build. */
    struct libusb_init_option init_opts[] = {
        { .option = LIBUSB_OPTION_NO_DEVICE_DISCOVERY },
    };
    int ret = libusb_init_context(&entry->ctx, init_opts,
                                  (int)(sizeof(init_opts) / sizeof(init_opts[0])));
    if (ret != LIBUSB_SUCCESS) {
        LOGE("register(fd=%d): libusb_init_context failed: %s",
             fd, libusb_error_name(ret));
        goto err_css_cond;
    }

    ret = libusb_wrap_sys_device(entry->ctx, (intptr_t)fd, &entry->handle);
    if (ret != LIBUSB_SUCCESS) {
        LOGW("register(fd=%d): libusb_wrap_sys_device failed: %s",
             fd, libusb_error_name(ret));
        goto err_ctx;
    }

    /* Identify the hardware as early as possible - right after the
     * handle exists, before anything that can fail in a
     * device-specific way. Every log line from here on can then name
     * the device, which is what makes a user's logcat actionable: "no
     * bulk endpoint pair found" tells us nothing on its own, while the
     * same message next to a vid:pid:bcd is a reproducible report and,
     * if it ever comes to that, the exact key a per-device workaround
     * would match on. The descriptor is cached by libusb, so this
     * costs no bus traffic. */
    struct libusb_device_descriptor desc;
    if (libusb_get_device_descriptor(libusb_get_device(entry->handle),
                                     &desc) == LIBUSB_SUCCESS) {
        entry->vid = desc.idVendor;
        entry->pid = desc.idProduct;
        entry->bcd_device = desc.bcdDevice;
    }
    LOGI("register(fd=%d): usb %04x:%04x bcd %04x", fd,
         entry->vid, entry->pid, entry->bcd_device);

    LOGI("register(fd=%d): link speed %d (1=full 12Mbps, 2=high 480Mbps, 3=super)",
         fd, libusb_get_device_speed(libusb_get_device(entry->handle)));

    /* Interface selection comes FIRST, before anything is claimed: which
     * interface to claim is its result, not its precondition. Reading the
     * configuration descriptor requires no claim, and claiming interface 0
     * up front would be exactly the assumption that breaks on a device whose
     * mass-storage function is not listed first. See
     * laser_find_bulk_endpoints() for the full rationale. */
    if (laser_find_bulk_endpoints(entry) < 0) {
        LOGW("register(fd=%d, usb %04x:%04x bcd %04x): no usable Bulk-Only "
             "mass storage interface - not an optical drive?",
             fd, entry->vid, entry->pid, entry->bcd_device);
        goto err_handle;
    }

    libusb_set_auto_detach_kernel_driver(entry->handle, 1);
    ret = libusb_kernel_driver_active(entry->handle, entry->iface_num);
    if (ret == 1)
        LOGI("register(fd=%d): kernel driver was attached on interface %u",
             fd, entry->iface_num);

    ret = libusb_claim_interface(entry->handle, entry->iface_num);
    if (ret != LIBUSB_SUCCESS) {
        LOGW("register(fd=%d, usb %04x:%04x): libusb_claim_interface(%u) "
             "failed: %s",
             fd, entry->vid, entry->pid, entry->iface_num,
             libusb_error_name(ret));
        goto err_handle;
    }

    /* Normalize BOT state before any SCSI command is sent - see this
     * function's doc comment in laser_internal.h. */
    laser_mass_storage_reset(entry);

    /* Which logical unit is the optical drive, and is it one at all? Must
     * come after the reset and after io_lock exists, since it takes it.
     * Always leaves entry->lun usable, even on failure.
     *
     * ONE CALL ANSWERING BOTH: choosing the unit and classifying it are the
     * same question asked once, and splitting them would mean re-issuing the
     * very INQUIRY the choice was made on.
     *
     * Combo enclosures expose their card reader as a second Mass Storage /
     * Bulk-Only / SCSI device, identical to a drive in every USB descriptor,
     * so one reaches this point as a candidate and there is no way to tell
     * from the bus alone. INQUIRY tells, and tells immediately.
     *
     * Runs BEFORE the spin-up wait, which is a change of order from the
     * obvious one and is deliberate: GET MAX LUN and INQUIRY are answered
     * whatever the tray holds, so nothing here needs a medium, and putting
     * them first is what lets a card reader be declined for the price of one
     * command instead of a whole spin-up budget - on every browse. */
    int optical = laser_probe_lun(entry);

    if (optical == LASER_OPTICAL_NO) {
        LOGI("register(fd=%d, usb %04x:%04x): not an optical drive, "
             "declining", fd, entry->vid, entry->pid);
        ret = LIBUSB_ERROR_NOT_SUPPORTED;
        goto err_iface;
    }

    /* The device left the bus while we were probing it. Declining is the only
     * honest answer: every field this entry would carry - endpoints, LUN,
     * interface - describes something that is no longer there, and publishing
     * it means each consumer that looks the token up pays a failed transfer
     * to discover what is already known here. The same drive plugged back in
     * arrives as a new descriptor and a new token, so there is nothing to
     * keep this one for. */
    if (optical == LASER_OPTICAL_GONE) {
        LOGW("register(fd=%d, usb %04x:%04x): device left the bus during "
             "setup, declining", fd, entry->vid, entry->pid);
        ret = LIBUSB_ERROR_NO_DEVICE;
        goto err_iface;
    }

    /* Spin the drive up and wait for its medium before anything tries to
     * read it. A cold optical drive answers the first data command with
     * NOT READY / becoming-ready and, on some firmware, a READ that
     * arrives before spin-up fails outright rather than starting it - so
     * this must happen before any classification or playback read. See its
     * doc comment.
     *
     * Skipped when INQUIRY went unanswered. Not an optimisation: waiting
     * presumes a device that is busy becoming ready, and one that will not
     * serve a command needing no medium is not that. It only spends the
     * budget to learn again what the probe just found. The device is still
     * registered - silence is not proof it is the wrong kind - so a caller
     * may still try to read it, and will fail quickly instead of slowly. */
    if (optical == LASER_OPTICAL_NO_ANSWER) {
        LOGI("register(fd=%d, usb %04x:%04x): INQUIRY unanswered, skipping "
             "the spin-up wait", fd, entry->vid, entry->pid);
    } else {
        laser_wait_until_ready(entry);
    }

    /* Publish. Everything this entry needs is now in place, so from here
     * laser_lookup() may hand it out - and not one line before, which is the
     * whole reason a single flag is enough. Done under the table lock so that
     * a thread which observes in_use == 1 also observes every field written
     * above it. */
    pthread_mutex_lock(&g_table_lock);
    entry->in_use = 1;
    pthread_mutex_unlock(&g_table_lock);

    LOGI("register(fd=%d, usb %04x:%04x bcd %04x): ready "
         "(iface=%u ep_in=0x%02x ep_out=0x%02x lun=%u)",
         fd, entry->vid, entry->pid, entry->bcd_device,
         entry->iface_num, entry->ep_in, entry->ep_out, entry->lun);
    return 0;

    /* Unwind ladder, in exact reverse order of acquisition. Each error
     * above jumps to the label matching the LAST thing it managed to
     * acquire, and falls through the rest.
     *
     * err_iface exists for the one failure that can happen after the claim
     * succeeded: declining a device that INQUIRY says is not an optical
     * drive. Everything before that point fails with no interface held and
     * enters the ladder lower down, which is why this link had no reason to
     * exist until that check was added. */
    err_iface:
    libusb_release_interface(entry->handle, entry->iface_num);
    err_handle:
    libusb_close(entry->handle);
    err_ctx:
    libusb_exit(entry->ctx);
    err_css_cond:
    pthread_cond_destroy(&entry->css_cv);
    err_css_mutex:
    pthread_mutex_destroy(&entry->css_mtx);
    err_io_mutex:
    pthread_mutex_destroy(&entry->io_lock);
    err_slot:
    release_slot(entry);
    return -1;
}

/** Serializes every mutation of the registry: registration, including the
 * expensive USB setup inside laser_register(), and teardown.
 *
 * NOT the same thing as g_table_lock, which is deliberately kept fast, is
 * taken for a handful of instructions at a time, and is what every lookup on
 * the transaction path uses. This one is held for as long as a registration
 * takes - up to fifteen seconds on a drive spinning up - and must therefore
 * never be on that path.
 *
 * Two jobs, both now belonging to laser_acquire() and laser_release():
 *
 *   - a second consumer arriving for the same token mid-registration BLOCKS
 *     here until the first is done, and then finds the finished entry on its
 *     re-check. Without it, both would find nothing, both would register, and
 *     one would lose its slot;
 *   - a teardown cannot interleave with a registration still in progress.
 *
 * It also makes "register if needed, then take the claim" a single atomic
 * step, which it was not when registration had its own entry point: the
 * sequence took this lock, dropped it, and took it again to count, and a
 * release landing in that gap could tear the entry down between the two
 * halves. The caller then incremented refs on a slot that had just been
 * memset - or, worse, one already handed to a different fd.
 *
 * Renamed from g_registry_lock along with the removal of the lazy path
 * it was named for. */
static pthread_mutex_t g_registry_lock = PTHREAD_MUTEX_INITIALIZER;

/* ---------------------------------------------------------------------------
 * CSS authentication sessions - contract in laser.h.
 *
 * LOCK ORDER, total, acquire left to right only:
 *
 *     g_registry_lock  >  io_lock  >  css_mtx  >  g_table_lock
 *
 * css_mtx is never held across io_lock or g_registry_lock: begin()
 * finishes its lookup (which may take g_registry_lock) BEFORE touching
 * css_mtx, and the transport's session check takes css_mtx and releases it
 * before locking io_lock. A session being "open" for hours is a flag, not a
 * held lock, which is what lets it outlive any single call.
 * ------------------------------------------------------------------------- */

int laser_cdb_changes_css_state(const uint8_t *cdb, int cdb_len)
{
    /* All key-class CDBs are 12 bytes; anything shorter cannot be one, and
     * reading cdb[10] on it would be out of bounds. */
    if (cdb_len < 12)
        return 0;

    switch (cdb[0]) {
    case 0xA4: { /* REPORT KEY - format in byte 10, bits 5:0 */
        const uint8_t fmt = cdb[10] & 0x3f;
        /* AGID(00) allocates one of the drive's four; challenge(01) and
         * key1(02) advance the handshake; title key(04) is session-scoped and
         * bus-key-encrypted; invalidate(3F) destroys the session. ASF(05) and
         * RPC state(08) only report, and libdvdcss reads RPC state before any
         * AGID exists - refusing those would disable CSS for the whole disc
         * rather than protect anything. */
        return fmt == 0x00 || fmt == 0x01 || fmt == 0x02 ||
               fmt == 0x04 || fmt == 0x3f;
    }
    case 0xA3: { /* SEND KEY - format in byte 10, bits 5:0 */
        const uint8_t fmt = cdb[10] & 0x3f;
        /* challenge(01) and key2(03) are handshake steps. RPC region set(06)
         * is not, and is not something this project ever issues. */
        return fmt == 0x01 || fmt == 0x03;
    }
    case 0xAD: /* READ DVD STRUCTURE - format in byte 7 */
        /* WHITELIST, not a list of the state-changing ones, and the only
         * opcode here treated that way. The others have a closed set of
         * formats this project issues; ADh does not - libdvdcpxm added CPRM's
         * media identifier and media key block reads to this same opcode, both
         * of which carry an AGID in byte 10 exactly as the disc key does, and
         * a fourth structure could be added tomorrow.
         *
         * So the question asked is the inverse one: is this format known to
         * need NO authentication? Physical(00) and copyright(01) are - they
         * carry no AGID, and dvdcss_test() issues them before any AGID exists,
         * so refusing them would disable CSS for the whole disc rather than
         * protect anything. Everything else is assumed to be part of an
         * authenticated exchange.
         *
         * Being wrong conservatively costs a retry that would probably have
         * been safe; being wrong the other way hands an undeclared consumer
         * one of the drive's four AGIDs mid-handshake. Note that this does
         * make every MKB pack read non-retryable, and an MKB is read a pack at
         * a time - a transient stall there fails the read rather than being
         * absorbed. That is the price of not having to know, in this file,
         * every structure format a caller might one day send. */
        return !(cdb[7] == 0x00 || cdb[7] == 0x01);
    default:
        return 0;
    }
}

int laser_css_session_is_open(laser_entry_t *entry)
{
    int open;

    pthread_mutex_lock(&entry->css_mtx);
    open = entry->css_open;
    pthread_mutex_unlock(&entry->css_mtx);

    return open;
}

/** Absolute deadline for pthread_cond_timedwait(), on the clock css_cv was
 * created against. Reading LASER_CSS_CLOCK here and there is what keeps the
 * two the same: pthread_cond_timedwait() interprets the timespec on the
 * condvar's own clock, so naming the other one would not fail loudly, it
 * would simply wait for the difference between two epochs. */
static void css_deadline(struct timespec *ts, long ms)
{
    clock_gettime(LASER_CSS_CLOCK, ts);
    ts->tv_sec  += ms / 1000;
    ts->tv_nsec += (ms % 1000) * 1000000L;
    if (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec++;
        ts->tv_nsec -= 1000000000L;
    }
}

laser_status_t laser_css_session_begin(int token, const void *owner)
{
    if (owner == NULL) {
        LOGE("css_session_begin(token=%d): NULL owner cookie", token);
        return LASER_ERR_IO;
    }

    /* lookup(), like every entry point that is not laser_acquire(): a session
     * on a device nobody has claimed is a lifecycle bug in the caller, and
     * this says so rather than registering a drive on its behalf. libdvdcss
     * acquires immediately before calling this - see the patch's
     * dvdcss_open(). */
    laser_entry_t *entry = laser_lookup(token);
    if (entry == NULL) {
        LOGW("css_session_begin(token=%d): not registered - no claim held?",
             token);
        return LASER_ERR_NO_SUCH_TOKEN;
    }

    struct timespec deadline;
    css_deadline(&deadline, LASER_CSS_SESSION_MAX_WAIT_MS);

    pthread_mutex_lock(&entry->css_mtx);

    while (entry->css_open) {
        if (entry->css_owner == owner) {
            /* Re-entrant begin() by the same consumer. Waiting would deadlock
             * against itself, so it is reported rather than hung on. */
            pthread_mutex_unlock(&entry->css_mtx);
            LOGE("css_session_begin(token=%d, usb %04x:%04x): this owner "
                 "already holds the session", token, entry->vid, entry->pid);
            return LASER_ERR_IO;
        }
        if (pthread_cond_timedwait(&entry->css_cv, &entry->css_mtx,
                                   &deadline) == ETIMEDOUT) {
            pthread_mutex_unlock(&entry->css_mtx);
            /* A timeout here is a LEAK, not contention: a whole disc's key
             * work is orders of magnitude shorter than the ceiling. Named as
             * such, because the symptom - CSS quietly unavailable - otherwise
             * points nowhere. */
            LOGE("css_session_begin(token=%d, usb %04x:%04x): no session after "
                 "%d ms - another consumer is holding one and has probably "
                 "leaked it; CSS unavailable for this attempt",
                 token, entry->vid, entry->pid,
                 LASER_CSS_SESSION_MAX_WAIT_MS);
            return LASER_ERR_IO;
        }
    }

    entry->css_open  = 1;
    entry->css_owner = owner;
    pthread_mutex_unlock(&entry->css_mtx);

    return LASER_OK;
}

void laser_css_session_end(int token, const void *owner)
{
    /* A session cannot exist on an unregistered token, so a miss here is a
     * caller-lifecycle bug and is reported as one. */
    laser_entry_t *entry = laser_lookup(token);
    if (entry == NULL) {
        LOGW("css_session_end(token=%d): not registered", token);
        return;
    }

    pthread_mutex_lock(&entry->css_mtx);
    if (!entry->css_open || entry->css_owner != owner) {
        pthread_mutex_unlock(&entry->css_mtx);
        LOGE("css_session_end(token=%d, usb %04x:%04x): no session for this "
             "owner - unbalanced end(), ignored", token, entry->vid, entry->pid);
        return;
    }
    entry->css_open  = 0;
    entry->css_owner = NULL;
    pthread_cond_signal(&entry->css_cv);
    pthread_mutex_unlock(&entry->css_mtx);
}

/** Tear an entry down. Caller MUST hold g_registry_lock, and the last
 * claim on the entry must already be gone. */
static void teardown_entry_locked(laser_entry_t *entry)
{
    int token = entry->token;

    /* See the threading contract in laser.h: the caller guarantees no
     * transaction is in flight on this token any more by the time the last
     * claim is dropped, so it is safe to tear down without acquiring
     * entry->io_lock here. */
    libusb_release_interface(entry->handle, entry->iface_num);
    libusb_close(entry->handle);
    libusb_exit(entry->ctx);

    /* The public contract covers TRANSACTIONS, which is what licenses tearing
     * down without taking io_lock. It says nothing about SESSIONS, and cannot:
     * a session spans several transactions, so "no transaction in flight" is
     * true at every gap between two steps of a live handshake. So this one is
     * checked. A fired warning means a consumer outlived the teardown of the
     * device it was authenticating against - a real bug, worth naming. */
    if (laser_css_session_is_open(entry)) {
        LOGE("release(token=%d, usb %04x:%04x): a CSS session is still open "
             "at teardown - a consumer is authenticating against a device "
             "being torn down", token, entry->vid, entry->pid);
    }
    pthread_cond_destroy(&entry->css_cv);
    pthread_mutex_destroy(&entry->css_mtx);
    pthread_mutex_destroy(&entry->io_lock);

    release_slot(entry);

    LOGI("release(token=%d): device torn down", token);
}

laser_status_t laser_acquire(int token)
{
    /* ONE CRITICAL SECTION, covering the lookup, the registration if there
     * has to be one, and the increment. Not an optimisation - a correctness
     * requirement. Dropping the lock between the lookup and the increment
     * would let a release land in the gap, take another consumer's claim to
     * zero and tear the entry down between the two halves, leaving this
     * function to increment refs on a slot that had just been memset, or on
     * one already reserved for a different fd. */
    pthread_mutex_lock(&g_registry_lock);

    laser_entry_t *entry = laser_lookup(token);
    if (entry == NULL) {
        /* Result checked via the lookup rather than the return value: what
         * matters to this function is whether an entry exists afterwards, and
         * laser_register() has exactly one success shape. */
        laser_register(token);
        entry = laser_lookup(token);
    }

    if (entry == NULL) {
        pthread_mutex_unlock(&g_registry_lock);
        LOGW("acquire(token=%d): could not register the device", token);
        return LASER_ERR_NO_SUCH_TOKEN;
    }

    entry->refs++;
    int refs = entry->refs;
    int vid = entry->vid, pid = entry->pid;

    pthread_mutex_unlock(&g_registry_lock);

    LOGI("acquire(token=%d, usb %04x:%04x): %d consumer(s)",
         token, vid, pid, refs);
    return LASER_OK;
}

void laser_release(int token)
{
    /* Held across the whole decision, not just the decrement: whether this
     * call tears the device down and the teardown itself have to be one
     * atomic step, or two consumers releasing at the same time could both see
     * the count reach zero. It is also the lock that keeps a teardown from
     * interleaving with a registration still in progress - see the comment on
     * g_registry_lock. */
    pthread_mutex_lock(&g_registry_lock);

    laser_entry_t *entry = laser_lookup(token);
    if (entry == NULL) {
        pthread_mutex_unlock(&g_registry_lock);
        LOGW("release(token=%d): not registered, ignored", token);
        return;
    }

    if (entry->refs <= 0) {
        /* Unbalanced: more releases than acquires. Logged rather than acted
         * on, because the alternative - tearing down anyway - is exactly the
         * "first one out wins" behaviour the count exists to remove. The
         * device stays up, some other consumer's claim keeps it alive, and
         * the bug is named where it can be found. */
        pthread_mutex_unlock(&g_registry_lock);
        LOGE("release(token=%d, usb %04x:%04x): no claim held by anyone - "
             "unbalanced release, ignored", token, entry->vid, entry->pid);
        return;
    }

    entry->refs--;
    if (entry->refs > 0) {
        int refs = entry->refs;
        pthread_mutex_unlock(&g_registry_lock);
        LOGI("release(token=%d): %d consumer(s) still hold it", token, refs);
        return;
    }

    /* Last claim gone: this device is on its way out. Raise the cancellation
     * flag BEFORE tearing anything down.
     *
     * THIS IS WHERE CANCELLATION LIVES, and not in an entry point a consumer
     * could call. The flag is sticky and token-wide, while a claim is one of
     * several: on a DVD libdvdcss holds the same token, so one consumer
     * cancelling would disable the drive for the others with no way to clear
     * it. Arriving here means "the last consumer is done", which is the only
     * moment at which a token-wide statement is true.
     *
     * What it buys, set from here: the public contract says no
     * transaction may be in flight when the last claim is dropped, and
     * nothing here can enforce that. A thread that legitimately took its
     * entry pointer before this call began now sees the flag and abandons its
     * remaining attempts, instead of spending a six-attempt budget on a
     * handle that is about to close. */
    laser_set_cancelled(entry);

    teardown_entry_locked(entry);
    pthread_mutex_unlock(&g_registry_lock);
}
