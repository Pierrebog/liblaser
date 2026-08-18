/*****************************************************************************
 * disc.c: disc identification - see laser_disc.h for the contract.
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
 *****************************************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>

#include "laser.h"
#include "laser_disc.h"

#include <udfread.h>
#include <blockinput.h>

/* ============================================================================
 * DVD-Video and BD-Video detection, via libudfread
 * ============================================================================ */

struct scsi_block_input
{
    struct udfread_block_input base; /* must be first member */
    int token;

    /* Latched once the medium has gone away mid-probe, so the remaining
     * reads of this probe fail instantly instead of each going back to
     * the drive to be told the same thing. libudfread walks a fair
     * number of structures before it can conclude anything - anchor,
     * volume descriptor sequence, file set, root directory - and on a
     * disc that was ejected part-way through, every one of those is a
     * full transport round trip whose answer is already known. */
    bool media_gone;
};

static int udf_read_cb(struct udfread_block_input *bi, uint32_t lba,
                       void *buf, uint32_t nblocks, int flags)
{
    (void) flags;
    struct scsi_block_input *sbi = (struct scsi_block_input *)bi;

    if (sbi->media_gone)
        return -1;

    int ret = laser_read_blocks(sbi->token, lba, (int)nblocks, buf);

    /* Translate rather than forward. laser's block helpers speak
     * a richer language than libudfread's callback does: a negative
     * laser_status_t distinguishing a vanished disc from an I/O
     * error, and a short read reported as a smaller block count. A
     * blockinput callback has room for exactly two outcomes, so this is
     * where the extra information has to be either acted on or dropped -
     * forwarding a raw -3 as if it were a block count would push a value
     * libudfread has no contract for into a library that cannot do
     * anything with it.
     *
     * LASER_ERR_MEDIA_GONE is the one distinction worth acting on,
     * and it is acted on here (see media_gone above) rather than passed
     * up. A short read is not useful either: half a metadata structure
     * is not a partial answer, it is no answer. */
    if (ret == LASER_ERR_MEDIA_GONE)
        sbi->media_gone = true;

    if (ret != (int)nblocks)
        return -1;

    return ret;
}

/** Copy a NUL-terminated UTF-8 string into a laser_disc_t::volume_id,
 * truncating on a SEQUENCE BOUNDARY rather than on a byte.
 *
 * A UDF Logical Volume Identifier may legally be longer than the 32
 * characters laser_disc.h keeps room for, and truncation there is deliberate
 * - a truncated name is a usable label. Truncating mid-sequence is not the
 * same thing: it leaves one to three orphaned bytes at the end, and what
 * comes out is no longer UTF-8 at all. The header promises UTF-8, and the
 * consumer that reads this hands it to a Java String, where invalid input is
 * either replaced with U+FFFD or throws depending on the decoder used.
 *
 * Backing off is one rule: continuation bytes are 10xxxxxx, so while the byte
 * that WOULD have been cut off is one of those, the sequence it belongs to is
 * incomplete and the cut moves left. It stops on an ASCII byte or on a lead
 * byte, and cutting immediately before either is always on a boundary. */
static void copy_volume_id(char *dst, const char *src)
{
    size_t len = strlen(src);

    if (len > LASER_DISC_VOLUME_ID_MAX - 1) {
        len = LASER_DISC_VOLUME_ID_MAX - 1;

        while (len > 0 && ((unsigned char)src[len] & 0xC0) == 0x80)
            len--;
    }

    memcpy(dst, src, len);
    dst[len] = '\0';
}

static uint32_t udf_size_cb(struct udfread_block_input *bi)
{
    /* Unknown / not tracked - documented as optional in blockinput.h,
     * and laser_read_blocks() chunks internally regardless of how
     * big the medium claims to be, so this is safe to leave at 0. */
    (void) bi;
    return 0;
}

/* What a single pass over the disc's UDF filesystem concluded. */
typedef enum {
    UDF_VIDEO_NONE = 0,   /* no UDF, or UDF with neither layout on it */
    UDF_VIDEO_DVD,
    UDF_VIDEO_BD,
} udf_video_t;

/** Is this really a DVD-Video, or merely a disc with a VIDEO_TS directory?
 *
 * The presence of the file is not the answer: a data disc can carry a
 * VIDEO_TS folder holding anything at all. What settles it is the first
 * twelve bytes of VIDEO_TS.IFO, which a Video Manager must open with -
 * the same check libdvdread performs at open time, so agreeing with it
 * here is agreeing with whether the disc will actually play. */
static bool udf_is_dvd_video(udfread *udf)
{
    UDFFILE *f = udfread_file_open(udf, "/VIDEO_TS/VIDEO_TS.IFO");
    if (f == NULL)
        return false;

    char magic[12];
    ssize_t got = udfread_file_read(f, magic, sizeof(magic));
    udfread_file_close(f);

    return got == (ssize_t)sizeof(magic)
        && memcmp(magic, "DVDVIDEO-VMG", sizeof(magic)) == 0;
}

/** Opens the medium's UDF filesystem ONCE and answers both questions from
 * it, filling volume_id (caller-allocated, LASER_DISC_VOLUME_ID_MAX bytes)
 * with the Logical Volume Identifier when something was recognised.
 *
 * One open for both, rather than a function per disc kind: mounting UDF
 * is the expensive part - anchor, volume descriptor sequence, file set,
 * root directory, each a transport round trip - while the two probes
 * that follow are a path lookup apiece. Splitting them would pay that
 * cost twice on every Blu-ray, since DVD is tested first.
 *
 * DVD IS TESTED FIRST, and not only because it is the commoner disc: a
 * BD-Video disc has no VIDEO_TS, and a DVD-Video disc has no BDMV, so
 * the order is a preference between two answers that cannot both be
 * true rather than a tie-break. */
static udf_video_t detect_udf_video(int token, char *volume_id)
{
    udf_video_t kind = UDF_VIDEO_NONE;

    struct scsi_block_input sbi = {
            .base = { .close = NULL, .read = udf_read_cb, .size = udf_size_cb },
            .token = token,
            .media_gone = false,
    };

    udfread *udf = udfread_init();
    if (udf == NULL)
        return UDF_VIDEO_NONE;

    if (udfread_open_input(udf, &sbi.base) == 0)
    {
        if (udf_is_dvd_video(udf))
        {
            kind = UDF_VIDEO_DVD;
        }
        else
        {
            UDFFILE *f = udfread_file_open(udf, "/BDMV/index.bdmv");
            if (f != NULL)
            {
                kind = UDF_VIDEO_BD;
                udfread_file_close(f);
            }
        }

        if (kind != UDF_VIDEO_NONE)
        {
            const char *vol_id = udfread_get_volume_id(udf);
            if (vol_id != NULL)
                copy_volume_id(volume_id, vol_id);
            else
                volume_id[0] = '\0';
        }
    }

    udfread_close(udf);
    return kind;
}

/* ============================================================================
 * Video CD and Super Video CD detection, via a minimal ISO9660 walk
 * ============================================================================
 * NO UDF HERE, AND NOT BY OVERSIGHT. A Video CD carries an ISO9660
 * filesystem and nothing else - it predates UDF entirely - so the walk
 * above cannot answer for it at any cost. That is a change of ANSWER, not
 * of principle: the rule this module follows is "identify through the
 * filesystem the reader will actually use", and for a VCD the reader
 * (VLC's vcd module, through cdrom.c) uses ISO9660. The same rule that
 * chose UDF for a DVD chooses ISO9660 here.
 *
 * What is deliberately NOT built is a general ISO9660 reader. Two
 * directory levels are needed and no more - the root, then /VCD or
 * /SVCD - so what follows walks exactly that and stops. No Joliet, no
 * Rock Ridge, no recursion, no path table: those exist to find arbitrary
 * files by arbitrary names, and this file looks up two fixed names it
 * already knows.
 *
 * SECTOR ADDRESSING IS ABSOLUTE, which is the one assumption worth
 * stating because it is the one that breaks. ISO9660 puts its Primary
 * Volume Descriptor at sector 16 of the TRACK; laser_read_blocks()
 * addresses the medium. The two coincide on a single-session disc whose
 * track 1 starts at LBA 0, which is every pressed VCD and SVCD. On a
 * multi-session disc where the filesystem lives in a later session they
 * do not, and this probe simply finds no PVD and reports nothing - a
 * disc identified as UNKNOWN rather than a wrong answer.
 * ============================================================================ */

#define ISO_SECTOR_SIZE 2048
#define ISO_PVD_LBA     16

/* Ceiling on how much of a directory this walks before giving up.
 *
 * A VCD's root holds a handful of entries and /VCD a dozen; four sectors is
 * already an order of magnitude more than either needs. The bound is here
 * for the disc that is not what it claims: a corrupt or hostile directory
 * length field would otherwise turn this probe into an unbounded run of SCSI
 * commands against a drive, during a browse, on the UI's timescale. */
#define ISO_MAX_DIR_SECTORS 4

/* Offsets within a directory record (ECMA-119 section 9.1). Named rather
 * than written inline because three of them are one byte wide and adjacent,
 * which is exactly the shape a transposition hides in. */
#define ISO_DR_LENGTH        0
#define ISO_DR_EXTENT_LE     2
#define ISO_DR_DATA_LEN_LE  10
#define ISO_DR_FILE_FLAGS   25
#define ISO_DR_NAME_LEN     32
#define ISO_DR_NAME         33

#define ISO_FLAG_DIRECTORY  0x02

/** Where the ISO9660 filesystem being walked starts, and which token to read
 * it through.
 *
 * `base` is the start LBA of the track carrying the filesystem, and every
 * address in this walk is relative to it. It used to be implicitly zero:
 * ISO9660 places its Primary Volume Descriptor at sector 16 OF THE TRACK, and
 * this code read sector 16 of the medium. The two coincide on any disc whose
 * first data track starts at LBA 0, which is every pressed VCD and SVCD - so
 * the assumption held for the discs that mattered and failed silently for the
 * rest.
 *
 * The base now comes from the TOC that read_cd_toc() has already fetched, so
 * addressing the right track costs no extra command. What this still does not
 * handle is a filesystem in a LATER SESSION of a multi-session disc: format 0
 * of READ TOC reports tracks, not sessions, and finding the last session's
 * first track means READ TOC format 01h - a second command, and a separate
 * change. Such a disc yields LASER_DISC_UNKNOWN, which is the same wrong-free
 * answer it gave before. */
typedef struct {
    int      token;
    uint32_t base;
} iso_ctx_t;

static uint32_t iso_le32(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

/** One 2048-byte sector, or false.
 *
 * Every failure is the same failure here - a short read, an I/O error, a
 * disc that went away - because a half-read metadata sector is not a
 * partial answer any more than a half-read UDF structure was (see
 * udf_read_cb above). The probe's caller has exactly one thing to do with
 * any of them: stop and report nothing. */
static bool iso_read_sector(const iso_ctx_t *ctx, uint32_t lba, uint8_t *buf)
{
    return laser_read_blocks(ctx->token, ctx->base + lba, 1, buf) == 1;
}

/** Compare a directory record's name against a plain ASCII one.
 *
 * ISO9660 stores file identifiers uppercased and suffixed with ";1", and
 * directory identifiers bare. Rather than build the two spellings at each
 * call site, the version suffix is stripped here and the comparison is
 * case-insensitive - which costs nothing and tolerates the discs that
 * ignore the uppercase rule, of which there are more than the specification
 * would suggest. */
static bool iso_name_matches(const uint8_t *name, unsigned name_len,
                             const char *want)
{
    const uint8_t *semi = memchr(name, ';', name_len);
    if (semi != NULL)
        name_len = (unsigned)(semi - name);

    if (name_len != strlen(want))
        return false;

    for (unsigned i = 0; i < name_len; i++)
    {
        int a = name[i];
        int b = (unsigned char)want[i];

        if (a >= 'a' && a <= 'z')
            a -= 'a' - 'A';
        if (b >= 'a' && b <= 'z')
            b -= 'a' - 'A';

        if (a != b)
            return false;
    }

    return true;
}

/** Find one named child of the directory at (dir_lba, dir_len), filling
 * *out_lba and *out_len with its extent.
 *
 * @param want_dir  whether the wanted entry is a directory. Checked rather
 *                  than ignored because a data disc may legitimately hold a
 *                  FILE called VCD next to whatever else it carries, and
 *                  reading that as a directory would walk arbitrary content
 *                  as if it were directory records.
 */
static bool iso_find_child(const iso_ctx_t *ctx, uint32_t dir_lba, uint32_t dir_len,
                           const char *want, bool want_dir,
                           uint32_t *out_lba, uint32_t *out_len)
{
    uint32_t sectors = (dir_len + ISO_SECTOR_SIZE - 1) / ISO_SECTOR_SIZE;
    if (sectors > ISO_MAX_DIR_SECTORS)
        sectors = ISO_MAX_DIR_SECTORS;

    for (uint32_t s = 0; s < sectors; s++)
    {
        uint8_t sector[ISO_SECTOR_SIZE];
        if (!iso_read_sector(ctx, dir_lba + s, sector))
            return false;

        unsigned off = 0;
        while (off < ISO_SECTOR_SIZE)
        {
            unsigned rec_len = sector[off + ISO_DR_LENGTH];

            /* A zero length is the padding that fills a sector once its
             * last record has been written: records may not straddle a
             * sector boundary, so the remainder is simply unused. It ends
             * this sector, not the directory. */
            if (rec_len == 0)
                break;

            /* A record that claims to run past the end of its own sector,
             * or that has no room for the fixed part plus a name, is
             * malformed. Stopping is the only safe response - continuing
             * would mean reading a length field from somewhere the record
             * does not cover. */
            if (rec_len < ISO_DR_NAME || off + rec_len > ISO_SECTOR_SIZE)
                break;

            unsigned name_len = sector[off + ISO_DR_NAME_LEN];
            if (name_len == 0 || ISO_DR_NAME + name_len > rec_len)
            {
                off += rec_len;
                continue;
            }

            bool is_dir =
                (sector[off + ISO_DR_FILE_FLAGS] & ISO_FLAG_DIRECTORY) != 0;

            if (is_dir == want_dir &&
                iso_name_matches(sector + off + ISO_DR_NAME, name_len, want))
            {
                *out_lba = iso_le32(sector + off + ISO_DR_EXTENT_LE);
                *out_len = iso_le32(sector + off + ISO_DR_DATA_LEN_LE);
                return true;
            }

            off += rec_len;
        }
    }

    return false;
}

/** Does the info file at this extent open with the expected signature?
 *
 * THE DIRECTORY IS NOT THE ANSWER, exactly as /VIDEO_TS is not the answer
 * for a DVD (see udf_is_dvd_video above). A data disc can carry a folder
 * called VCD holding anything at all; what settles it is the first eight
 * bytes of the info file, which is what VLC's own vcd module reads to decide
 * whether it can play the disc. Agreeing with it here means this
 * identification predicts playability rather than merely resembling it. */
static bool iso_info_magic_is(const iso_ctx_t *ctx, uint32_t lba, const char *magic)
{
    uint8_t sector[ISO_SECTOR_SIZE];
    if (!iso_read_sector(ctx, lba, sector))
        return false;

    return memcmp(sector, magic, 8) == 0;
}

typedef enum {
    ISO_VIDEO_NONE = 0,
    ISO_VIDEO_VCD,
    ISO_VIDEO_SVCD,
} iso_video_t;

/** Walks the medium's ISO9660 filesystem once and answers for both Video CD
 * kinds, filling volume_id (caller-allocated, LASER_DISC_VOLUME_ID_MAX
 * bytes) when something was recognised.
 *
 * One walk for both, for the same reason detect_udf_video() does one mount
 * for DVD and BD: the expensive part is reaching the root directory - PVD,
 * then the root extent - while each probe that follows is one directory
 * lookup and one sector.
 *
 * VCD IS TESTED FIRST, and as with DVD before BD this is a preference
 * between two answers that cannot both be true: a VCD has no /SVCD and an
 * SVCD has no /VCD. It is first only because it is the commoner disc. */
static iso_video_t detect_iso_video(const iso_ctx_t *ctx, char *volume_id)
{
    uint8_t pvd[ISO_SECTOR_SIZE];
    if (!iso_read_sector(ctx, ISO_PVD_LBA, pvd))
        return ISO_VIDEO_NONE;

    /* Type 1 is the Primary Volume Descriptor; "CD001" is the standard
     * identifier every descriptor in the sequence carries. Both are checked
     * because either alone matches too much: a data sector that happens to
     * begin with 0x01, or a disc whose sector 16 holds some other
     * descriptor kind. */
    if (pvd[0] != 0x01 || memcmp(pvd + 1, "CD001", 5) != 0)
        return ISO_VIDEO_NONE;

    /* The root directory record sits inside the PVD at offset 156, as a
     * 34-byte directory record of the same shape as the ones walked
     * above. */
    const uint8_t *root = pvd + 156;
    uint32_t root_lba = iso_le32(root + ISO_DR_EXTENT_LE);
    uint32_t root_len = iso_le32(root + ISO_DR_DATA_LEN_LE);

    if (root_len == 0)
        return ISO_VIDEO_NONE;

    static const struct {
        const char *dir;
        const char *info;
        const char *magic;
        iso_video_t kind;
    } probes[] = {
        { "VCD",  "INFO.VCD", "VIDEO_CD", ISO_VIDEO_VCD  },
        { "SVCD", "INFO.SVD", "SUPERVCD", ISO_VIDEO_SVCD },
    };

    iso_video_t kind = ISO_VIDEO_NONE;

    for (size_t i = 0; i < sizeof(probes) / sizeof(probes[0]); i++)
    {
        uint32_t dir_lba, dir_len, info_lba, info_len;

        if (!iso_find_child(ctx, root_lba, root_len, probes[i].dir, true,
                            &dir_lba, &dir_len))
            continue;

        if (!iso_find_child(ctx, dir_lba, dir_len, probes[i].info, false,
                            &info_lba, &info_len))
            continue;

        if (info_len < 8)
            continue;

        if (iso_info_magic_is(ctx, info_lba, probes[i].magic))
        {
            kind = probes[i].kind;
            break;
        }
    }

    if (kind == ISO_VIDEO_NONE)
        return ISO_VIDEO_NONE;

    /* Volume Identifier: 32 bytes at offset 40, space-padded rather than
     * terminated - unlike UDF's Logical Volume Identifier, which is why the
     * trailing spaces are trimmed here and not in the caller. 32 bytes fit
     * LASER_DISC_VOLUME_ID_MAX exactly, terminator included.
     *
     * Frequently generic in practice - a great many VCDs are simply labelled
     * VIDEOCD - which is not a defect to work around: it is the disc's own
     * label, and laser_disc.h already tells callers to fall back to a name
     * of their own when this is not useful. */
    unsigned len = 32;
    while (len > 0 && (pvd[40 + len - 1] == ' ' || pvd[40 + len - 1] == '\0'))
        len--;

    memcpy(volume_id, pvd + 40, len);
    volume_id[len] = '\0';

    return kind;
}

/* ============================================================================
 * What the medium is, via READ TOC
 * ============================================================================ */

/** What READ TOC says this medium is. */
typedef enum {
    /** READ TOC did not answer, or answered with no usable track entry. Not
     * a CD - a DVD, a Blu-ray, an empty drive or a drive that does not
     * implement the command. */
    CD_TOC_NONE = 0,
    /** A CD whose first track is a data track. */
    CD_TOC_DATA,
    /** A CD whose first track is an audio track. */
    CD_TOC_AUDIO,
} cd_toc_t;

/** Read the medium's table of contents.
 *
 * THREE ANSWERS, NOT TWO, and the third one pays for itself. This used to
 * return a bool meaning "is it an audio CD", which threw away the fact that
 * READ TOC had SUCCEEDED - and that fact is the answer to "is this medium a
 * CD at all". Only a CD can carry the ISO9660 filesystem the Video CD probe
 * looks for, so distinguishing CD_TOC_NONE from CD_TOC_DATA lets
 * laser_disc_identify() skip that probe outright on a DVD or a Blu-ray,
 * rather than spending a PVD read and a root-directory walk to conclude what
 * this command already established.
 *
 * @param first_data_lba receives the start LBA of the first DATA track, or 0
 *        if there is none. See the ISO9660 walk's use of it: the descriptors
 *        needed for this are in the reply this command already fetched, so
 *        the walk gets to address the right track for no extra bus traffic.
 */
static cd_toc_t read_cd_toc(int token, uint32_t *first_data_lba)
{
    *first_data_lba = 0;

    uint8_t cdb[10] = { 0 };
    cdb[0] = 0x43; /* READ TOC/PMA/ATIP */
    cdb[1] = 0x00; /* MSF = 0 (LBA addressing) */
    cdb[2] = 0x00; /* format 0: normal TOC */
    cdb[6] = 0x00; /* starting track number */
    cdb[7] = 0x00;
    cdb[8] = 0x80; /* allocation length: 128 bytes - header + a few tracks */

    /* Zero-initialised: on a drive that pads a short reply up to the
     * full allocation length, actual_len can cover bytes the drive never
     * meaningfully filled (see the residue discussion in bot.c).
     * Starting from zeroes means such bytes read as 0 rather than as
     * whatever was on the stack. */
    uint8_t toc[128] = { 0 };
    int actual_len = 0;
    laser_status_t st = laser_scsi_cdb(token, cdb, sizeof(cdb),
                                       toc, sizeof(toc), 1, &actual_len);
    if (st != LASER_OK || actual_len < 4 + 8)
        return CD_TOC_NONE; /* not a CD, or drive doesn't support READ TOC */

    /* Cross-check against the TOC's own length field rather than relying
     * on the transport's byte count alone. A TOC reply starts with a
     * 16-bit big-endian "TOC Data Length" covering everything after
     * those two bytes, so the drive tells us how much of this buffer it
     * actually filled - which is exactly the question the transport
     * cannot always answer, since a padded transfer and a bridge with a
     * broken residue look the same from there. A full first descriptor
     * needs 2 + 2 + 8 bytes, so anything shorter means there is no track
     * entry to read, whatever the transfer length claimed. */
    unsigned toc_data_len = ((unsigned)toc[0] << 8) | toc[1];
    if (toc_data_len + 2u < 4u + 8u)
        return CD_TOC_NONE;

    /* How many whole descriptors the drive actually filled. Bounded by both
     * the announced length and the transfer, so a drive that lies in either
     * direction cannot make this walk off the end of the buffer. */
    unsigned avail = toc_data_len + 2u;
    if (avail > (unsigned)actual_len)
        avail = (unsigned)actual_len;
    if (avail > sizeof(toc))
        avail = sizeof(toc);
    unsigned descriptors = (avail - 4u) / 8u;

    /* Descriptor layout: [reserved][ADR/CONTROL][track#][reserved]
     * [start address, 4 bytes, big-endian LBA since MSF=0]. CONTROL is the
     * low nibble of byte 1; bit 0x04 set means a data track, clear audio. */
    for (unsigned i = 0; i < descriptors; i++) {
        const uint8_t *d = toc + 4 + i * 8;

        if ((d[1] & 0x04) == 0)
            continue;                       /* audio track */
        if (d[2] == 0xAA)
            continue;                       /* lead-out, not a real track */

        *first_data_lba = ((uint32_t)d[4] << 24) | ((uint32_t)d[5] << 16) |
                          ((uint32_t)d[6] << 8)  |  (uint32_t)d[7];
        break;
    }

    /* Only the FIRST track descriptor decides audio vs data, which is a
     * deliberate simplification with one known consequence: a mixed-mode disc
     * whose track 1 is data and whose later tracks are audio is classified as
     * "not audio" here, falls through the DVD and BD probes, and ends up
     * presented as a plain data disc. An Enhanced CD - audio first, data
     * session last - is classified as audio, which is the useful answer.
     * Mixed-mode discs are rare enough, and their correct handling ambiguous
     * enough (play the audio tracks? mount the data track?), that guessing
     * here would be worse than the current behaviour.
     *
     * Note this is a different question from the loop above, which looks for
     * the first data track wherever it is: that one is about WHERE the
     * filesystem lives, this one about what the disc IS. */
    uint8_t control = toc[5] & 0x0F;
    return (control & 0x04) == 0 ? CD_TOC_AUDIO : CD_TOC_DATA;
}


/* ============================================================================
 * Public entry point
 * ============================================================================ */

void laser_disc_identify(int token, laser_disc_t *out)
{
    /* Zeroed first, so that every early return below - and every path that
     * simply does not recognise the disc - leaves the caller with
     * LASER_DISC_UNKNOWN and an empty label rather than with whatever was
     * on its stack. The contract says out is always filled; this is what
     * makes that true without a return value to check. */
    memset(out, 0, sizeof(*out));

    /* One READ TOC, answering two questions: is this an audio CD, and - if
     * not - is it a CD at all, and where does its filesystem start. Both
     * later steps depend on the answer, which is why it goes first. */
    uint32_t track_lba = 0;
    cd_toc_t toc = read_cd_toc(token, &track_lba);

    if (toc == CD_TOC_AUDIO)
    {
        out->kind = LASER_DISC_CD_AUDIO;
        /* volume_id stays empty: a Red Book disc has no filesystem, so
         * there is no label to recover here. */
        return;
    }

    switch (detect_udf_video(token, out->volume_id))
    {
        case UDF_VIDEO_DVD:
            out->kind = LASER_DISC_DVD_VIDEO;
            break;

        case UDF_VIDEO_BD:
            out->kind = LASER_DISC_BD_VIDEO;
            break;

        case UDF_VIDEO_NONE:
            /* Not a UDF video disc - which is not yet the same as "not a
             * video disc". A Video CD has no UDF at all, so it reaches
             * here, and the ISO9660 probe below is its only chance.
             *
             * LAST, AND SKIPPED ENTIRELY ON ANYTHING THAT IS NOT A CD.
             * Every disc that gets this far has already been shown not to
             * be an audio CD and not to carry a recognisable UDF video
             * layout - but that still left a plain data DVD paying for a
             * PVD read and a root-directory walk on the way to an answer
             * this probe cannot give it. Only a CD can carry the
             * filesystem being looked for, and the READ TOC above already
             * knows whether this is one, so the discs that reach the walk
             * are now Video CDs and plain data CDs and nothing else.
             *
             * volume_id is untouched by detect_udf_video() on this path,
             * so detect_iso_video() writes into the empty string left by
             * the memset above and any failure leaves it empty. */
            if (toc == CD_TOC_NONE)
                break;

            const iso_ctx_t ctx = { .token = token, .base = track_lba };

            switch (detect_iso_video(&ctx, out->volume_id))
            {
                case ISO_VIDEO_VCD:
                    out->kind = LASER_DISC_VCD;
                    break;

                case ISO_VIDEO_SVCD:
                    out->kind = LASER_DISC_SVCD;
                    break;

                case ISO_VIDEO_NONE:
                    /* Data disc, empty drive, or a read failure - one
                     * answer for the three, see the enum's own comment. */
                    break;
            }
            break;
    }
}
