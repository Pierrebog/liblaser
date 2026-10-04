/*****************************************************************************
 * laser_disc.h: what is in the drive?
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
 * Identifies the disc loaded in a registered laser device: audio CD, Video
 * CD, DVD, Blu-ray, or unknown.
 *
 * Nothing here depends on libudfread's headers: the UDF walk is an
 * implementation detail of disc.c, and a vendored dependency does not belong
 * in the interface of the library that vendors it.
 *****************************************************************************/

#ifndef LASER_DISC_H
#define LASER_DISC_H

#include "laser.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Size of laser_disc_t::volume_id, including the terminator.
 *
 * 32 BYTES, not 32 characters. ISO9660's Volume Identifier is a fixed
 * 32-byte field; a UDF Logical Volume Identifier can be longer and arrives as
 * UTF-8, where a character takes up to four bytes. A longer label is
 * truncated, on a character boundary, rather than rejected. */
#define LASER_DISC_VOLUME_ID_MAX 33

typedef enum {
    /** Not recognised: a data disc, an empty drive or a read failure alike,
     * since no consumer acts differently on them. Should that change, the
     * reason belongs in a separate out-parameter, not in this enum. */
    LASER_DISC_UNKNOWN = 0,

    LASER_DISC_CD_AUDIO,
    LASER_DISC_DVD_VIDEO,
    LASER_DISC_DVD_AUDIO,
    LASER_DISC_DVD_UNIVERSAL,
    LASER_DISC_BD_VIDEO,
    LASER_DISC_VCD,
    LASER_DISC_SVCD,
} laser_disc_kind_t;

typedef struct {
    laser_disc_kind_t kind;

    /** Volume label, NUL-terminated UTF-8, empty when there is none.
     *
     * For every DVD kind and for BD-Video, UDF's Logical Volume Identifier -
     * one per disc, a universal disc's two zones sharing one filesystem. For
     * a Video CD or Super Video CD, ISO9660's Volume Identifier with its
     * padding trimmed, and often just VIDEOCD; one that is not printable ASCII
     * is reported empty, since the field records no encoding to convert from.
     * For an audio CD, always empty: a Red Book disc has no filesystem, and a
     * name for it comes from CD-TEXT or a metadata service.
     *
     * Empty is not an error: callers fall back to a name of their own. */
    char volume_id[LASER_DISC_VOLUME_ID_MAX];
} laser_disc_t;

/**
 * Identify the disc loaded in the device registered under @p token.
 *
 * The caller must hold a claim on @p token (laser_acquire()), so that no
 * release tears the device down between two reads. Without one, and for a
 * drive that never became ready (laser_token_not_ready()), the answer is
 * LASER_DISC_UNKNOWN without contacting the drive.
 *
 * A drive that is not ready - a disc swapped since the token was registered,
 * still loading - is first waited for, as laser_acquire() does, for up to
 * 30 s and without cancellation. A ready drive costs one TEST UNIT READY.
 *
 * BEST EFFORT, AND NEVER FATAL. @p out is always filled: an unreadable or
 * unrecognised disc yields LASER_DISC_UNKNOWN and an empty volume_id, the
 * same answer as an empty drive.
 *
 * Detection order, each step skipped once an earlier one has answered:
 *
 *   1. GET CONFIGURATION, whose current profile says CD, DVD or BD. A drive
 *      that does not answer leaves the medium unknown, and the steps below
 *      then run as they would for a CD;
 *   2. on a CD, READ TOC: an audio first track makes an audio CD. Not asked
 *      on a DVD or a BD, for which drives make a TOC up;
 *   3. on a data CD, a minimal ISO9660 walk for both Video CD kinds: VCD by
 *      /VCD/INFO.VCD and its "VIDEO_CD" magic, SVCD by /SVCD/INFO.SVD and
 *      "SUPERVCD";
 *   4. one open of the medium's UDF filesystem, answering for every UDF
 *      kind: the Video zone by /VIDEO_TS/VIDEO_TS.IFO and its "DVDVIDEO-VMG"
 *      magic, the Audio zone by /AUDIO_TS/AUDIO_TS.IFO and its
 *      "DVDAUDIO-AMG" magic, BD-Video by /BDMV/index.bdmv. A disc with both
 *      DVD zones is LASER_DISC_DVD_UNIVERSAL.
 *
 * When the medium is unknown, step 3 runs after step 4 rather than before,
 * DVDs being the likelier answer there.
 *
 * Each kind is identified through the filesystem its reader will use, so
 * that the answer predicts playback. libdvdread finds both IFO files through
 * UDF and never falls back to ISO9660, so a DVD is identified by UDF even
 * though its UDF Bridge also carries ISO9660. VLC's vcd module reads a Video
 * CD through ISO9660, the only filesystem it has.
 *
 * The ISO9660 walk looks for the Primary Volume Descriptor at sector 16 of the
 * first data track, taken from the TOC; the extents it holds are absolute, as
 * multi-session mastering writes them. A filesystem in a later session is
 * not found - locating that session needs READ TOC format 01h - so such a
 * disc yields LASER_DISC_UNKNOWN rather than a wrong answer.
 *
 * THREADING: safe to call concurrently on different tokens. On one token each
 * command is serialized like any other, but the sequence is not atomic: a
 * disc swapped mid-identification most likely yields LASER_DISC_UNKNOWN.
 * The wait above also consumes the UNIT ATTENTION of a swap, which another
 * consumer of the token then never sees.
 *
 * @param token the registry token (the fd, see laser.h)
 * @param out   filled on every path; must not be NULL
 */
void laser_disc_identify(int token, laser_disc_t *out);

#ifdef __cplusplus
}
#endif

#endif /* LASER_DISC_H */
