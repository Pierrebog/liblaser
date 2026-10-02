/*****************************************************************************
 * registry.c: fd <-> USB device handle registry
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
 * Maps a token - an already-open USB device fd, on Android the one obtained
 * through UsbManager - to a device wrapped in a dedicated libusb context,
 * ready for SCSI-MMC transactions through bot.c and scsi.c. See laser.h for
 * the contract.
 *
 * A device is registered only by laser_acquire(), and torn down only when
 * its last claim is released. A registration without a claim could never be
 * torn down: it would hold a table slot for the life of the process and,
 * once its fd number was recycled, answer for another device.
 *****************************************************************************/

#include <errno.h>
#include <limits.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#ifdef __ANDROID__
# include <android/log.h>
#endif

#include "laser.h"
#include "laser_internal.h"

/* Clock of the CSS condition variable. CLOCK_MONOTONIC, so that a clock step
 * cannot stretch or shorten laser_css_session_begin()'s deadline - except
 * below Android API 21, where pthread_condattr_setclock() does not exist and
 * the default CLOCK_REALTIME has to do. The condvar's creation and
 * css_deadline() must agree on the clock, hence one macro for both. */
#if !defined(__ANDROID_API__) || __ANDROID_API__ >= 21
# define LASER_CSS_CLOCK CLOCK_MONOTONIC
#else
# define LASER_CSS_CLOCK CLOCK_REALTIME
#endif

/* ---------------------------------------------------------------------------
 * Logging: the sink behind the LOGI/LOGW/LOGE macros of laser_internal.h.
 * ------------------------------------------------------------------------- */

#define LOG_TAG "Laser"

/* Longest formatted line; longer ones are truncated. */
#define LOG_LINE_MAX 512

static laser_log_cb_t g_log_cb;
static void          *g_log_opaque;

/* Platform default, used until laser_set_log_cb() installs a sink: the system
 * log on Android, stderr elsewhere. msg is already formatted, hence "%s". */
static void default_log_cb(void *opaque, laser_log_level_t level,
                           const char *msg)
{
    (void) opaque;

#ifdef __ANDROID__
    int prio;
    switch (level) {
    case LASER_LOG_ERROR: prio = ANDROID_LOG_ERROR; break;
    case LASER_LOG_WARN:  prio = ANDROID_LOG_WARN;  break;
    case LASER_LOG_INFO:
    default:              prio = ANDROID_LOG_INFO;  break;
    }

    __android_log_print(prio, LOG_TAG, "%s", msg);
#else
    const char *prio;
    switch (level) {
    case LASER_LOG_ERROR: prio = "E"; break;
    case LASER_LOG_WARN:  prio = "W"; break;
    case LASER_LOG_INFO:
    default:              prio = "I"; break;
    }

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

int laser_parse_token(const char *str, int *token)
{
    /* strtol() alone would accept leading whitespace and a sign. */
    if (str == NULL || *str < '0' || *str > '9')
        return 0;

    char *end;
    errno = 0;
    long value = strtol(str, &end, 10);

    /* ERANGE is not redundant with the bound beside it: where long is 32 bits
     * an overflowing value saturates to LONG_MAX, which is INT_MAX. */
    if (*end != '\0' || errno == ERANGE || value > INT_MAX)
        return 0;

    *token = (int)value;
    return 1;
}

static laser_entry_t g_entries[LASER_MAX_DEVICES];

/** Guards the bookkeeping of g_entries (in_use, token), not USB I/O, which
 * entry->io_lock serializes. Held only briefly, never across I/O. */
static pthread_mutex_t g_table_lock = PTHREAD_MUTEX_INITIALIZER;

laser_entry_t *laser_lookup(int token)
{
    laser_entry_t *found = NULL;

    pthread_mutex_lock(&g_table_lock);
    for (int i = 0; i < LASER_MAX_DEVICES; i++) {
        /* in_use is set last by laser_register(), so a half-built entry is
         * never found. */
        if (g_entries[i].in_use && g_entries[i].token == token) {
            found = &g_entries[i];
            break;
        }
    }
    pthread_mutex_unlock(&g_table_lock);

    return found;
}

int laser_token_not_ready(int token)
{
    laser_entry_t *entry = laser_lookup(token);
    return entry != NULL && entry->not_ready;
}

/** Accessors of laser_entry_t::cancelled. The write takes the table lock so
 * that it cannot race with release_slot()'s memset. The read takes none: it
 * is on the transaction path, and the flag only ever goes from 0 to 1. */
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

/** Reserves and zeroes a free slot for @p fd, or returns NULL if the fd is
 * already registered or the table is full. The slot stays unpublished
 * (in_use == 0) until laser_register() has filled it in.
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

/** Frees a slot, after a failed registration or at teardown. The memset
 * clears in_use along with everything else. */
static void release_slot(laser_entry_t *slot)
{
    pthread_mutex_lock(&g_table_lock);
    memset(slot, 0, sizeof(*slot));
    pthread_mutex_unlock(&g_table_lock);
}

/* libusb's speed enum is not the USB generation: LOW is 1, FULL 2, HIGH 3,
 * SUPER 4. A name keeps high speed from being misread as USB 3. */
static const char *speed_name(int speed)
{
    switch (speed) {
        case LIBUSB_SPEED_UNKNOWN:  return "unknown to the OS";
        case LIBUSB_SPEED_LOW:      return "low, 1.5Mbps";
        case LIBUSB_SPEED_FULL:     return "full, 12Mbps";
        case LIBUSB_SPEED_HIGH:     return "high, 480Mbps";
        case LIBUSB_SPEED_SUPER:    return "super, 5Gbps";
        case LIBUSB_SPEED_SUPER_PLUS: return "super+, 10Gbps";
        default:                    return "unrecognised";
    }
}

/* Give the interface's kernel driver back, if we detached it. Only on paths
 * that give the device up - see the detach in laser_register(). */
static void reattach_kernel_driver(laser_entry_t *entry)
{
    if (!entry->kernel_driver_detached)
        return;

    entry->kernel_driver_detached = 0;

    int ret = libusb_attach_kernel_driver(entry->handle, entry->iface_num);
    if (ret == LIBUSB_SUCCESS) {
        LOGI("register(fd=%d, usb %04x:%04x): kernel driver re-attached on "
             "interface %u", entry->token, entry->vid, entry->pid,
             entry->iface_num);
    } else if (ret != LIBUSB_ERROR_NO_DEVICE) {
        LOGW("register(fd=%d, usb %04x:%04x): libusb_attach_kernel_driver(%u) "
             "failed: %s - the device may stay invisible to the system until "
             "it is unplugged",
             entry->token, entry->vid, entry->pid, entry->iface_num,
             libusb_error_name(ret));
    }
}

/** One-time device setup: dedicated libusb context, Bulk-Only interface and
 * endpoints, claim, Mass Storage Reset, optical LUN, spin-up wait. Reachable
 * only from laser_acquire().
 *
 * Caller MUST hold g_registry_lock. */
static int laser_register(int fd)
{
    laser_entry_t *entry = reserve_slot(fd);
    if (entry == NULL) {
        LOGW("register(fd=%d): already registered, or table full", fd);
        return -1;
    }

    /* Identity of the descriptor, for the recycling check in laser_acquire().
     * Left at zero on failure, which fd_still_ours() reads as unknown. */
    struct stat st;
    if (fstat(fd, &st) == 0) {
        entry->reg_dev = st.st_dev;
        entry->reg_ino = st.st_ino;
    } else {
        LOGW("register(fd=%d): fstat failed, identity check disabled", fd);
    }

    /* Full size until the device proves otherwise; the memset left 0. */
    entry->max_transfer_bytes = LASER_MAX_BYTES_PER_TRANSFER;

    /* io_lock serializes transactions; css_mtx and css_cv guard the CSS
     * session. */
    if (pthread_mutex_init(&entry->io_lock, NULL) != 0) {
        LOGE("register(fd=%d): pthread_mutex_init(io_lock) failed", fd);
        goto err_slot;
    }
    if (pthread_mutex_init(&entry->css_mtx, NULL) != 0) {
        LOGE("register(fd=%d): pthread_mutex_init(css_mtx) failed", fd);
        goto err_io_mutex;
    }
    /* On LASER_CSS_CLOCK. Without pthread_condattr_setclock(), the default
     * clock is already CLOCK_REALTIME. */
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

    /* A dedicated context, so that a long playback session and a detection
     * scan on another device never share libusb state. NO_DEVICE_DISCOVERY
     * because this library never enumerates: an unprivileged Android app
     * cannot read /dev/bus/usb, and the device comes from wrapping the fd. */
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

    /* The USB identity, read early so that every later log line can name the
     * device. The descriptor is cached: no bus traffic. */
    struct libusb_device_descriptor desc;
    if (libusb_get_device_descriptor(libusb_get_device(entry->handle),
                                     &desc) == LIBUSB_SUCCESS) {
        entry->vid = desc.idVendor;
        entry->pid = desc.idProduct;
        entry->bcd_device = desc.bcdDevice;
    }
    LOGI("register(fd=%d): usb %04x:%04x bcd %04x", fd,
         entry->vid, entry->pid, entry->bcd_device);

    int speed = libusb_get_device_speed(libusb_get_device(entry->handle));
    LOGI("register(fd=%d): link speed %d (%s)", fd, speed, speed_name(speed));

    /* Choose the interface before claiming it: interface 0 is not assumed
     * (see laser_find_bulk_endpoints()). */
    if (laser_find_bulk_endpoints(entry) < 0) {
        LOGW("register(fd=%d, usb %04x:%04x bcd %04x): no usable Bulk-Only "
             "mass storage interface - not an optical drive?",
             fd, entry->vid, entry->pid, entry->bcd_device);
        goto err_handle;
    }

    /* Detached by hand rather than with libusb's auto-detach, which
     * re-attaches on release. On kernels that bind usb-storage/sr to an
     * optical drive and mount the disc, re-attaching at each teardown lets the
     * kernel start its own conversation with a drive we are about to claim
     * again, and two initiators on one Bulk-Only device fail each other. So a
     * drive we keep stays detached until it is unplugged, invisible to the
     * rest of the system meanwhile.
     *
     * If "kernel driver attached" is logged on every registration while the
     * device stays plugged in, the re-bind comes from usbfs re-probing when
     * the fd is closed, not from libusb.
     *
     * Not fatal: if the detach fails, the claim below fails with
     * LIBUSB_ERROR_BUSY and says so. */
    ret = libusb_kernel_driver_active(entry->handle, entry->iface_num);
    if (ret == 1) {
        LOGI("register(fd=%d): kernel driver attached on interface %u, "
             "detaching it and not giving it back", fd, entry->iface_num);
        ret = libusb_detach_kernel_driver(entry->handle, entry->iface_num);
        if (ret == LIBUSB_SUCCESS)
            entry->kernel_driver_detached = 1;
        else
            LOGW("register(fd=%d, usb %04x:%04x): "
                 "libusb_detach_kernel_driver(%u) failed: %s",
                 fd, entry->vid, entry->pid, entry->iface_num,
                 libusb_error_name(ret));
    } else if (ret < 0) {
        LOGW("register(fd=%d, usb %04x:%04x): kernel_driver_active(%u) "
             "failed: %s (continuing, the claim below will tell)",
             fd, entry->vid, entry->pid, entry->iface_num,
             libusb_error_name(ret));
    }

    ret = libusb_claim_interface(entry->handle, entry->iface_num);
    if (ret != LIBUSB_SUCCESS) {
        LOGW("register(fd=%d, usb %04x:%04x): libusb_claim_interface(%u) "
             "failed: %s",
             fd, entry->vid, entry->pid, entry->iface_num,
             libusb_error_name(ret));
        goto err_handle;
    }

    /* Put the BOT state machine in a known state before any SCSI command. */
    laser_mass_storage_reset(entry);

    /* Which LUN is the optical drive, and is there one at all? Needs no
     * medium, so it runs before the spin-up wait: the card reader of a combo
     * enclosure, identical to a drive in its USB descriptors, is declined for
     * one INQUIRY instead of a whole spin-up budget. */
    int optical = laser_probe_lun(entry);

    if (optical == LASER_OPTICAL_NO) {
        LOGI("register(fd=%d, usb %04x:%04x): not an optical drive, "
             "declining", fd, entry->vid, entry->pid);
        goto err_iface;
    }

    /* The device left the bus during setup: nothing left to keep. */
    if (optical == LASER_OPTICAL_GONE) {
        LOGW("register(fd=%d, usb %04x:%04x): device left the bus during "
             "setup, declining", fd, entry->vid, entry->pid);
        goto err_iface;
    }

    /* A device that answers no INQUIRY is declined too. Keeping it would keep
     * its interface away from the kernel for as long as it stays plugged in,
     * and the kernel is the only agent able to reset it. */
    if (optical == LASER_OPTICAL_NO_ANSWER) {
        LOGW("register(fd=%d, usb %04x:%04x): INQUIRY unanswered on every "
             "unit, declining and handing the device back to the kernel - it "
             "is most likely still busy with work started before this claim",
             fd, entry->vid, entry->pid);
        goto err_iface;
    }

    /* Some firmware fails a READ that arrives before spin-up instead of
     * starting it, so wait before anything reads. */
    laser_wait_until_ready(entry);

    /* Publish, last and under the table lock, so that whoever sees
     * in_use == 1 also sees every field written above. */
    pthread_mutex_lock(&g_table_lock);
    entry->in_use = 1;
    pthread_mutex_unlock(&g_table_lock);

    LOGI("register(fd=%d, usb %04x:%04x bcd %04x): ready "
         "(iface=%u ep_in=0x%02x ep_out=0x%02x lun=%u)",
         fd, entry->vid, entry->pid, entry->bcd_device,
         entry->iface_num, entry->ep_in, entry->ep_out, entry->lun);
    return 0;

    /* Unwind ladder, in reverse order of acquisition. err_iface serves the
     * only failures after the claim: the declines above. */
    err_iface:
    libusb_release_interface(entry->handle, entry->iface_num);
    err_handle:
    /* A declined device gets its kernel driver back - typically a USB key,
     * which the claim detached before INQUIRY could tell what it was. */
    reattach_kernel_driver(entry);
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

/** Serializes registration - including its slow USB setup, up to fifteen
 * seconds - and teardown, and makes "register if needed, then count the
 * claim" one atomic step. Never taken on the transaction path, unlike
 * g_table_lock. */
static pthread_mutex_t g_registry_lock = PTHREAD_MUTEX_INITIALIZER;

/* ---------------------------------------------------------------------------
 * CSS authentication sessions - contract in laser.h.
 *
 * LOCK ORDER. Every nesting has g_registry_lock outside:
 *
 *     g_registry_lock  >  io_lock        (setup, and teardown's drain)
 *     g_registry_lock  >  css_mtx        (teardown: is a session open?)
 *     g_registry_lock  >  g_table_lock   (publish an entry, release a slot)
 *
 * No other pair nests, in either direction. laser_scsi_cdb() releases css_mtx
 * after its session check, before taking io_lock. An open session is a flag,
 * not a held lock.
 * ------------------------------------------------------------------------- */

int laser_cdb_changes_css_state(const uint8_t *cdb, int cdb_len)
{
    /* All key-class CDBs are 12 bytes; reading cdb[10] on a shorter one would
     * be out of bounds. */
    if (cdb_len < 12)
        return 0;

    switch (cdb[0]) {
    case 0xA4: { /* REPORT KEY - format in byte 10, bits 5:0 */
        const uint8_t fmt = cdb[10] & 0x3f;
        /* AGID(00), challenge(01), key1(02), title key(04) and
         * invalidate(3F) change state. ASF(05) and RPC state(08) only report,
         * and libdvdcss reads RPC state before any AGID exists. */
        return fmt == 0x00 || fmt == 0x01 || fmt == 0x02 ||
               fmt == 0x04 || fmt == 0x3f;
    }
    case 0xA3: { /* SEND KEY - format in byte 10, bits 5:0 */
        const uint8_t fmt = cdb[10] & 0x3f;
        /* challenge(01) and key2(03). RPC region set(06) is never issued. */
        return fmt == 0x01 || fmt == 0x03;
    }
    case 0xAD: /* READ DVD STRUCTURE - format in byte 7 */
        /* Inverted on purpose: only physical(00) and copyright(01) are known
         * to need no authentication, and dvdcss_test() issues them before any
         * AGID exists. Every other format - disc key, CPRM's media identifier
         * and MKB, anything added later - is treated as authenticated, at the
         * cost of never retrying those reads. */
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

/** Absolute deadline on LASER_CSS_CLOCK, the clock css_cv was created on. */
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

    /* A session needs a claim; begin() registers nothing. */
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
            /* Re-entrant begin() by the same owner would wait on itself. */
            pthread_mutex_unlock(&entry->css_mtx);
            LOGE("css_session_begin(token=%d, usb %04x:%04x): this owner "
                 "already holds the session", token, entry->vid, entry->pid);
            return LASER_ERR_IO;
        }
        if (pthread_cond_timedwait(&entry->css_cv, &entry->css_mtx,
                                   &deadline) == ETIMEDOUT) {
            pthread_mutex_unlock(&entry->css_mtx);
            /* A whole disc's key work is far shorter than the ceiling, so a
             * timeout means a leaked session, not contention. */
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
    /* A session cannot exist on an unregistered token. */
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

/** Tear an entry down. Caller MUST hold g_registry_lock; the last claim must
 * be gone and the entry already cancelled.
 *
 * io_lock is held while the device is reset and the handle closed. A
 * transaction in flight on another thread therefore completes first - within
 * about one phase timeout, since the cancellation stops its retries - and a
 * thread queued on io_lock only gets it after the close, sees the
 * cancellation and returns without touching libusb. A thread that still uses
 * the entry after release_slot() remains the caller's to prevent, as laser.h
 * says. */
static void teardown_entry_locked(laser_entry_t *entry)
{
    int token = entry->token;

    pthread_mutex_lock(&entry->io_lock);

    if (!entry->device_gone)
        laser_mass_storage_reset(entry);

    libusb_release_interface(entry->handle, entry->iface_num);
    libusb_close(entry->handle);
    libusb_exit(entry->ctx);

    pthread_mutex_unlock(&entry->io_lock);

    /* A session spans several transactions, so draining io_lock says nothing
     * about it: one still open means a consumer outlived the device. */
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

/* Does @p fd still name the device this entry was registered on? See
 * laser_entry_t::reg_dev. An identity fstat() could not record at
 * registration is trusted, rather than refusing every later claim. */
static int fd_still_ours(const laser_entry_t *entry, int fd)
{
    struct stat st;

    if (entry->reg_dev == 0 && entry->reg_ino == 0) {
        return 1;
    }

    if (fstat(fd, &st) != 0) {
        return 0;
    }

    return st.st_dev == entry->reg_dev && st.st_ino == entry->reg_ino;
}

laser_status_t laser_acquire(int token)
{
    /* One critical section for lookup, registration and increment: a release
     * landing between lookup and increment could tear the entry down in the
     * gap. */
    pthread_mutex_lock(&g_registry_lock);

    laser_entry_t *entry = laser_lookup(token);

    /* An entry lives only while somebody holds it, so a descriptor that no
     * longer names its device means a claim was never released. Refused:
     * re-registering would take the drive from a consumer that never let go. */
    if (entry != NULL && !fd_still_ours(entry, token)) {
        pthread_mutex_unlock(&g_registry_lock);
        LOGE("acquire(token=%d): the descriptor no longer names the device "
             "registered under it - a claim was never released", token);
        return LASER_ERR_NO_SUCH_TOKEN;
    }

    if (entry == NULL) {
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
    /* Held across the decision and the teardown, so that two concurrent
     * releases cannot both see the count reach zero, and so that a teardown
     * cannot interleave with a registration. */
    pthread_mutex_lock(&g_registry_lock);

    laser_entry_t *entry = laser_lookup(token);
    if (entry == NULL) {
        pthread_mutex_unlock(&g_registry_lock);
        LOGW("release(token=%d): not registered, ignored", token);
        return;
    }

    if (entry->refs <= 0) {
        /* More releases than acquires. Tearing down anyway would take the
         * device from the consumers still holding it; name the bug instead. */
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

    /* Last claim gone. Cancel first, so that an operation still running on
     * another thread abandons its remaining attempts, then tear down. The
     * flag is sticky and token-wide, which is why only the last release may
     * raise it. */
    laser_set_cancelled(entry);

    teardown_entry_locked(entry);
    pthread_mutex_unlock(&g_registry_lock);
}
