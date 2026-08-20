/*****************************************************************************
 * laser_disc.h: what is in the drive?
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
 * Identifies the disc currently loaded in a registered laser device, using
 * the transport declared in laser.h. Split out of the VLC access module that
 * first carried it, because the question and its answer are not VLC's: any
 * consumer holding a token wants to know whether it is looking at an audio
 * CD, a DVD-Video, a Blu-ray or something it has no idea about, and none of
 * the code that answers it needs to know what is asking.
 *
 * Nothing here depends on libudfread's headers, deliberately: the UDF walk
 * is an implementation detail of disc.c, and a vendored dependency has no
 * business appearing in the interface of the library that vendors it.
 *****************************************************************************/

#ifndef LASER_DISC_H
#define LASER_DISC_H

#include "laser.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Size of laser_disc_t::volume_id, including the terminator.
 *
 * 32 BYTES, not 32 characters - the distinction matters as soon as a label
 * is not ASCII. ISO9660's Volume Identifier is a fixed 32-byte field, so for
 * a Video CD the two coincide. A UDF Logical Volume Identifier is longer and
 * arrives here as UTF-8, where one character may take up to four bytes: a
 * 32-character label can therefore be truncated well before its 32nd
 * character.
 *
 * Truncated rather than rejected, and truncated on a character boundary
 * (see copy_volume_id() in disc.c, which backs off rather than leaving a
 * split sequence): a shortened name is a usable label, while a rejected one
 * is a disc that looks unrecognised. */
#define LASER_DISC_VOLUME_ID_MAX 33

typedef enum {
    /** Not recognised. Deliberately ONE value for three situations - a
     * plain data disc, an empty drive, and a failure to read - because no
     * consumer so far has been able to act differently on them, and
     * because separating them honestly would mean plumbing a status out of
     * a UDF walk that has no contract for reporting one (see the block
     * input callback in disc.c). If that changes, the distinction belongs
     * in a separate out-parameter rather than in this enum, which answers
     * "what is it", not "why not". */
    LASER_DISC_UNKNOWN = 0,

    LASER_DISC_CD_AUDIO,
    LASER_DISC_DVD_VIDEO,
    LASER_DISC_BD_VIDEO,
    LASER_DISC_VCD,
    LASER_DISC_SVCD,
} laser_disc_kind_t;

typedef struct {
    laser_disc_kind_t kind;

    /** Volume label, NUL-terminated, empty when there is none to recover.
     *
     * For DVD-Video and BD-Video this is UDF's Logical Volume Identifier,
     * as UTF-8. For a Video CD or Super Video CD it is ISO9660's Volume
     * Identifier, trimmed of the trailing spaces that field is padded with
     * - and frequently generic, a great many VCDs being labelled simply
     * VIDEOCD. That is the disc's own label rather than a failure to find a
     * better one, and the fallback advice below covers it. For an audio CD
     * it is ALWAYS empty: a Red Book disc has no filesystem and therefore
     * no label - a name for one has to come from CD-TEXT or from a metadata
     * service, neither of which is this library's business.
     *
     * Empty is not an error. A disc can be perfectly identified and carry
     * no label at all, so callers should fall back to a name of their own
     * choosing rather than treat this as a failed identification. */
    char volume_id[LASER_DISC_VOLUME_ID_MAX];
} laser_disc_t;

/**
 * Identify the disc loaded in the device registered under @p token.
 *
 * THE CALLER MUST HOLD A CLAIM on @p token (laser_acquire()), exactly as
 * every other entry point of this library now requires - see laser.h. A
 * token with no claim yields LASER_DISC_UNKNOWN without contacting the drive.
 *
 * Without that claim, a release by another consumer could tear the device
 * down mid-walk, between two of the reads below.
 *
 * A token whose last claim is being dropped concurrently yields
 * LASER_DISC_UNKNOWN - the same answer as an unreadable disc, for the same
 * reason the enum gives one value to three situations.
 *
 * BEST EFFORT, AND NEVER FATAL. @p out is always filled: an unreadable or
 * unrecognised disc yields LASER_DISC_UNKNOWN and an empty volume_id, which
 * is the same answer as an empty drive. There is no failure return, because
 * there is no useful distinction for a caller to make - and a caller that
 * wanted one would be asking a different question than this function
 * answers.
 *
 * Detection order, cheapest first, each step running only on the discs the
 * previous one did not claim:
 *
 *   1. audio CD, by READ TOC - one command, and the only test here that
 *      cannot be confused with a data or video disc;
 *   2. a single open of the medium's UDF filesystem, which answers for both
 *      UDF video kinds: DVD-Video by /VIDEO_TS/VIDEO_TS.IFO and its
 *      "DVDVIDEO-VMG" magic, BD-Video by /BDMV/index.bdmv;
 *   3. a minimal ISO9660 walk, which answers for both Video CD kinds: VCD
 *      by /VCD/INFO.VCD and its "VIDEO_CD" magic, SVCD by /SVCD/INFO.SVD
 *      and "SUPERVCD". Run only on a medium step 1 established IS a CD,
 *      since nothing else can carry the filesystem it looks for - so a plain
 *      data DVD does not pay for it.
 *
 * THE RULE IS THE FILESYSTEM THE READER WILL USE, and applying it is what
 * makes steps 2 and 3 use different ones rather than one being a fallback
 * for the other.
 *
 * DVD-Video discs are UDF Bridge - a UDF 1.02 filesystem and an ISO9660 one
 * over the same file data - so for them either could answer. UDF is the
 * right one because libdvdread locates VIDEO_TS.IFO through UDF and never
 * falls back to ISO9660 to find a file: a disc whose UDF is damaged but
 * whose ISO9660 is intact would be identified as a DVD here and then refuse
 * to open there. A Video CD has no UDF at all and is read through ISO9660
 * by VLC's vcd module, so the same rule picks ISO9660 there. Identifying
 * through the filesystem the reader will use makes this answer predictive
 * rather than merely plausible - which is the point, and is why the ISO9660
 * walk added for VCD is deliberately NOT consulted for DVD.
 *
 * THE ISO9660 WALK ADDRESSES THE TRACK. ISO9660 places its Primary Volume
 * Descriptor at sector 16 of the track carrying the filesystem, and the walk
 * addresses it relative to the first data track's start LBA - taken from the
 * table of contents step 1 has already read, so it costs no extra command.
 * Sector 16 of the MEDIUM would be the same sector only on a disc whose
 * first data track starts at LBA 0: true of every pressed VCD and SVCD, and
 * silently wrong for anything else.
 *
 * What remains outside its reach is a filesystem in a LATER SESSION of a
 * multi-session disc. Format 0 of READ TOC reports tracks, not sessions, so
 * finding the last session's first track needs format 01h - a second
 * command. Such a disc yields LASER_DISC_UNKNOWN - no PVD found, nothing
 * reported - rather than a wrong answer.
 *
 * THREADING: safe to call concurrently on different tokens. On the same
 * token it serialises on that device's transport lock like any other
 * command, but the sequence of reads is not atomic - a disc swapped
 * mid-identification yields whatever the two halves saw, most likely
 * LASER_DISC_UNKNOWN.
 *
 * @param token the registry token (the fd, see laser.h)
 * @param out   filled on every path; must not be NULL
 */
void laser_disc_identify(int token, laser_disc_t *out);

#ifdef __cplusplus
}
#endif

#endif /* LASER_DISC_H */
