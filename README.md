# LASER — Library for Accessing SCSI External Readers

## 1. Goal

Let VLC for Android play optical discs — CD-Audio, Video CD, DVD-Video,
DVD-Audio and Blu-ray — from a USB drive attached to the phone or Smart TV
Box, on unrooted stock Android, and on **any hardware that conforms to the
applicable standards** rather than on a list of blessed models.

Android grants userspace access to a USB device as a file descriptor obtained
through `UsbManager`. There is no block device, no mount point, and no
`/dev/sr0`: everything VLC's optical stack normally relies on is absent. The
work is therefore to give that stack a path to a drive it can only reach by
sending SCSI command blocks over USB bulk endpoints itself.

## 2. User's manual

### How to use

*This assumes you use a **VLC for Android** buid that includes this library and
where all the related patches to libdvdcss, libvlc and vlc-android have been applied.*

**Play a disc.** Plug the drive in, grant the USB permission Android asks for,
and open *Browse*. A tile appears per disc found, named after the disc. Tap it.

**Several drives at once** each get their own tile, or tiles. Android asks a
separate permission per drive, one dialog after the other, and the tiles keep
a fixed order — they are sorted by the port a drive is plugged into, not by
whichever permission you granted first, so they do not move around between
launches.

**Audio CDs** appear as one tile holding the disc's tracks.

**Video CDs and Super Video CDs** appear as one tile each, named after the disc —
many are labelled simply *VIDEOCD*, which is the label the disc itself carries.

**DVD-Video discs** offer two tiles. One with the disc's own menus, one labelled
*(No menus)* that starts the first title directly. Use the second when a
disc's menus do not respond, or to skip straight to the film.

**DVD-Audio discs** appear as one tile. Most of them are *universal* discs
carrying a video version of the same album alongside the audio one, and those
get two tiles with the same name: the one with a **music** icon plays the
audio side, the one with a **film** icon plays the video side.

**Blu-rays** appear as one tile. An unencrypted volume plays as-is. A
commercial one needs [AACS](#7-acronyms). To enable it, you need to:
- Install `libaacs-provider.apk`, a separate application holding nothing but
  `libaacs.so.0`;
- Place `KEYDB.cfg` at `/sdcard/aacs/KEYDB.cfg` and grant VLC *All files
  access* (Android 11 and later).

**Unplugging** is safe at any time: playback stops within a couple of seconds,
with an error, and the tile disappears.

### Troubleshooting

**In case of low DVD framerate,** especially on older arm setup, set hardware
acceleration to `Disabled` or `Automatic`.

**If nothing happens when you plug the drive in** — no permission dialog, no
tile — the usual cause is power rather than software. An optical drive draws
more than most phones and TV boxes are willing to supply over an OTG port, and
an underpowered one either never enumerates or spins up, clicks and drops off
the bus. Use a powered USB hub, or a Y-cable with its own supply, and check
whether the drive has an external power input of its own.

**If a disc plays but stutters, suspect the power again before anything else.**
Same as above: the drive could be underpowered and silently retry read attempts.

**When something does not play.** The log tag is `Laser` for the transport and
`VLC/LaserDrive` for the Android side; every transport line carries the drive's
`vid:pid:bcd`, which is what a hardware bug report needs.

## 3. Approach

Four decisions shape everything else.

**The fd is the token.** The descriptor Kotlin obtains from
`UsbDeviceConnection.getFileDescriptor()` is carried, as a decimal number
inside an [MRL](#7-acronyms), all the way down to the transport, where it identifies the
device in a small registry. No parallel handle type, no registration call from
the Java side, no lifecycle to keep in sync. It stays an ordinary argument the
whole way down: each library between the MRL and the transport takes it as the
target of its open call, so nothing carries it out of band. The cost is that the token is
meaningful only within one process and one connection, which makes an
laser MRL a *session-scoped* name: it may never enter a persistent store,
and the connection behind it needs an owner that outlives the screen that
opened it. Both are handled.

**One shared contrib, not per-module code.** `liblaser` owns the
device registry (`registry.c`), the USB interface and endpoint discovery
(`usb.c`), the Mass Storage Bulk-Only transport (`bot.c`), the SCSI command
layer (`scsi.c`) and the identification of the disc in the drive (`disc.c`) —
one file per concern, since the three transport ones started life as a single
one and the split follows the boundaries its own header comment already named.
The VLC access module, libdvdcss, libdvdread/libdvdnav and the CD-Audio module
all reach the drive through it. Retry policy, [LUN](#7-acronyms) selection, sense-code
interpretation and error semantics exist once. libbluray is the one consumer
that needs none of it: it reads through the access module's stream.

**Standards as the specification, hardware as the test.** Where a device could
differ, the code follows what the specification prescribes and degrades
predictably when a device does not — rather than special-casing the drive on
the desk. Interface and endpoints are resolved from the configuration
descriptor instead of assuming interface 0; `GET MAX LUN` stalling is treated
as the class specification says it must be; transfer size is negotiated
downwards at runtime instead of being fixed at a value safe for the worst
bridge. Per-device workarounds remain possible — every log line carries
`vid:pid:bcd` — but none has been needed.

**VLC's architecture is the frame, not an obstacle.** Everything here is an
ordinary module with a capability and a shortcut, selected by an ordinary MRL
and built by the existing contrib and autotools machinery. Nothing
short-circuits libVLC to reach the Java side: **this project adds no new JNI
call at all.** The constraint is load-bearing — the fd travels inside an MRL
because an MRL is a string libVLC already carries end to end, and the
connection's lifetime is reconciled from events libVLC already emits. It
also decides what does *not* get written: where a behaviour was missing it was
added in the shape VLC already uses, in the module that owns it, rather than
routed around from outside — hence the small, local patches to `cdda.c`,
`dvdread.c`, libdvdcss, libdvdread and libdvdnav.

## 4. Architecture

### Global architecture

```
        Kotlin (VLC-Android)  —  opens the drive, owns the fd
          │
          │   the fd travels inside the MRL
          v
        libVLC  —  standard MRL and module loading
          │
          ├─────────────────────┬─────────────────────┐
          v                     v                     v
  ┌────────────────┐   ┌──────────────────┐  ┌─────────────────┐
  │    access      │   │     demux        │  │  access_demux   │
  │    "laser"     │   │ dvd / dvdsimple  │  │     "cdda"      │
  │                │◄──│  dvda / bluray   │  │  access "vcd"   │
  │                │   │                  │  │    / "svcd"     │
  └────────────────┘   └──────────────────┘  └─────────────────┘
  lists the disc,      blocks: from the      raw CD sectors,
  reads its sectors    access module         direct via cdrom.c
                       CSS keys: below
          │                     │                     │
          └─────────────────────┴─────────────────────┘
                                v
          ┌───────────────────────────────────────────┐
          │             contrib liblaser              │
          │    token registry · Bulk-Only transport   │
          │   SCSI commands · retries · CSS session   │
          │            disc identification            │
          └───────────────────────────────────────────┘
```

The fd is carried as a decimal number inside the MRL, and which MRL decides
which of the three paths above is taken:

| MRL                                                                                      | Purpose                                                                                                                                                                                                                                                                                                  |
|------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `file://laser/<fd>`                                                                      | Classification. The `file` scheme is borrowed because the preparser types items from a fixed scheme table and never opens a module for a scheme it does not know.                                                                                                                                   |
| `laser/dvd://<fd>`, `laser/dvdsimple://<fd>`, `laser/dvda://<fd>`, `laser/bluray://<fd>` | Video and DVD-Audio playback. libVLC reads this as access `laser` plus a demuxer name. A DVD-Video is offered under two of them — with menus and without — and a universal disc under `dvda` and `dvd`, one per zone, told apart by their icon alone.                                               |
| `cdda://laser/<fd>`                                                                      | CD-Audio. Here the drive is the URI's *authority*, not the access module: `cdda` is an access_demux and reaches the contrib itself. It expands into one MRL per track, `…/Track%20NN`.                                                                                                              |
| `vcd://laser/<fd>`, `svcd://laser/<fd>`                                                  | Video CD and Super Video CD. Same shape and same reason as the line above — the drive is the authority, and `vcd` reaches the contrib through the same `cdrom.c` as `cdda`. A Video CD is a *video* disc that stays on the *CD* path, because what decides the path is the sector, not the content. |

Two asymmetries in the diagram are worth reading twice. The `dvd` and `bluray`
demuxers get their **blocks** from the access module, not from the contrib —
which is why libbluray needs no token at all, and why only libdvdcss, for its
key commands, has an arrow going down. And the right-hand column never touches
*our* access module at all: `cdda`, `vcd` and `svcd` are VLC's own, and they
reach the drive through `cdrom.c`, which the contrib serves directly.

What puts a disc in that right-hand column is the **sector**, not the medium.
A stream hands out a flat run of 2048-byte blocks; CD-Audio is 2352-byte raw
sectors and a Video CD's payload is Mode 2 Form 2 — 2324 usable bytes behind a
24-byte header — and neither can be expressed that way. So a Video CD, despite
being video, sits beside the audio CD rather than beside the DVD.

### Inside the contrib

§3 names the five files and what each owns. This is how they reach each other.

```
  consumers, out of tree:  VLC access "laser" · cdrom.c · libdvdcss
                              │  entering at disc.c, scsi.c or
                              v  registry.c - see below
 ┌────────────────┐   ┌────────────────┐
 │   registry.c   │   │     disc.c     │  what is in the drive:
 │                │   └────────────────┘  TOC, then UDF, then ISO9660
 │  tokens and    │           │
 │  claims, CSS   │           v
 │  session,      │   ┌────────────────┐
 │  log sink      ├───┤     scsi.c     │  one CDB in, sense out:
 │                │   └────────────────┘  retries, spin-up, chunking
 │  owns the      │           │
 │  entry every   │           v
 │  box here      │   ┌────────────────┐
 │  reads         │   │     bot.c      │  one transaction:
 │                │   └────────────────┘  CBW → data → CSW
 │                │           │ reset only
 │                │           v
 │                │   ┌────────────────┐
 │                ├───┤     usb.c      │  endpoints; Mass Storage
 └────────────────┘   └────────────────┘  Reset
                              │
                              v
                       libusb (contrib/)
```

**Read downwards and the vocabulary changes once per box.** `disc.c` speaks
filesystems and knows no [CDB](#7-acronyms); `scsi.c` speaks CDBs and sense keys and knows no
[CBW](#7-acronyms); `bot.c` speaks CBW, data, [CSW](#7-acronyms) and knows no sense code and no retry;
`usb.c` speaks descriptors and endpoints and never builds a command at all.

**A consumer can enter the stack at three different heights**, which is why
`laser.h` publishes read helpers and not just an identification call. The
access module enters at the top for classification and in the middle for
playback, taking sectors straight from `scsi.c`; `cdrom.c` enters in the middle
only, since a raw CD sector is a command and not a filesystem; libdvdcss enters
at `registry.c` for the session bracket — `laser_acquire()` then
`laser_css_session_begin()` in `dvdcss_open_common()`, unwound in both close
paths —
and at `scsi.c` for its key commands, every one of which goes through the
single `LaserSend()` wrapper in `ioctl.c`. Nothing enters at `bot.c` or below.

**Two libraries a reader will look for are deliberately absent.** libbluray
takes its blocks from the access module's stream and needs no token, as §4
already says. libaacs has no edge either, for a different reason: its Android
patch is `dirs_android.c` alone, which decides where `KEYDB.cfg` is read from
and where the key cache is written — a storage-location problem that exists on
Android whether or not a USB drive is involved, and one that touches no [MMC](#7-acronyms)
code. `aacs.c` is untouched and still opens a drive by *path*, through
`mmc_open()`, which is exactly what a `UsbDeviceConnection` fd cannot provide.
Giving it the treatment libdvdcss got would add the same two edges as
libdvdcss has — `registry.c` for a session bracket, `scsi.c` for the Volume ID
and the rest of the handshake — and that is the work §5 describes, not
something the drawing above is missing.

**`registry.c` is not a layer in that stack** — it sits beside it, and every
box reads the entry it owns. It is also the only thing that runs the stack out
of order: registration is not a separate public call but the first
`laser_acquire()` on an unknown token, which resolves the endpoints through
`usb.c` and then probes the logical unit through `scsi.c`, skipping the two
boxes in between. Everything after that goes down the stack normally.

**Two edges do not point downwards, and both are deliberate.** `scsi.c` calls
`laser_lookup()` on *every* command rather than holding the entry it was given,
so that a device torn down between two commands is discovered at the next one
instead of dereferenced. And `bot.c` calls `laser_mass_storage_reset()` in
`usb.c` directly, for Reset Recovery: that request is a class request on the
interface rather than a command on the bulk pipes, so it belongs to the USB
layer even though only the transport layer ever needs it. Note what that
arrow does *not* carry: `bot.c`'s bulk transfers go straight to libusb, so the
only thing crossing from `bot.c` into `usb.c` is the reset.

## 5. Remaining work

### Functional

- **AACS from the drive, for the discs a key database does not cover.**
  libbluray reaches the disc through the stream, but the AACS handshake needs
  MMC commands of its own — the Volume ID above all — which a stream cannot
  carry. Making those discs play means giving libaacs the treatment libdvdcss
  got: a device handle routed through the contrib, plus the equivalent of the
  [CSS](#7-acronyms) session exclusion. This is a chantier on the scale of the whole CSS
  effort, not a wiring job, and it is unchanged by the patch below — which
  deliberately touches no MMC code.

### Dormant

- **Layer break awareness.** `READ DVD STRUCTURE` format `00h` gives the layer
  break address and the [PTP](#7-acronyms)/[OTP](#7-acronyms) direction of a dual-layer disc. Aligning
  windows so they do not straddle it would avoid a mechanical seek per
  crossing. With the drive idle 95% of the time, this is theoretical.
- **Recovery of latched refusals after a successful authentication.** A sector
  latched as scrambled before CSS engaged stays latched for the session. The
  clean fix is a generation counter on the CSS session, sampled on each read.
  Rarely reachable in practice, since authentication happens during open.
- **Video CD in a later session.** The ISO9660 walk now addresses the *track*:
  it reads sector 16 relative to the first data track's start [LBA](#7-acronyms), taken from
  the table of contents step 1 has already fetched, so it costs no extra
  command. What is still out of reach is a filesystem living in a later
  *session* of a multi-session disc — format 0 of `READ TOC` reports tracks,
  not sessions, so finding the last session's first track needs format `01h`,
  a second command. Such a disc yields "unrecognised" rather than a wrong
  answer, which is the right failure.
- **Mixed-mode CD.** A disc whose track 1 is data and whose later tracks are
  audio is classified as "not audio", falls through every probe and is
  presented as a plain data disc. An Enhanced CD — audio first, data session
  last — is classified as audio, which is the useful answer. The correct
  handling of a true mixed-mode disc is ambiguous enough (play the audio
  tracks? mount the data track?) that guessing would be worse.

## 6. Reference documents

### Platform — how the descriptor arrives

- **Android USB host API** — `UsbManager`, permission intents,
  `UsbDeviceConnection.getFileDescriptor()`. The whole "fd is the token"
  design rests on what this returns and on who owns it.
  <https://developer.android.com/guide/topics/connectivity/usb/host>

### Transport — how to talk to the drive

- **USB Mass Storage Class — Bulk-Only Transport ([BBB](#7-acronyms)), rev 1.0** — CBW/CSW
  framing, the thirteen host/device data-transfer cases, stall recovery, Reset
  Recovery, Phase Error semantics. Governs `bot.c`, and the interface and
  endpoint discovery and Mass Storage Reset in `usb.c`.
  <https://www.usb.org/document-library/mass-storage-bulk-only-10>
  (PDF: <https://www.usb.org/sites/default/files/usbmassbulk_10.pdf>)
- **USB Mass Storage Class Specification Overview, rev 1.4** — subclass and
  protocol code assignments; what makes an interface the mass-storage one.
  <https://www.usb.org/sites/default/files/Mass_Storage_Specification_Overview_v1.4_2-19-2010.pdf>
- **[INCITS](#7-acronyms) [T10](#7-acronyms) MMC / [SPC](#7-acronyms) / [SBC](#7-acronyms)** — the SCSI command set itself: `READ(10)` and
  `READ CD`, `READ TOC`, `REPORT KEY` / `SEND KEY`, `READ DISC STRUCTURE`
  (`READ DVD STRUCTURE` in earlier editions, and the command that would carry
  a BD Volume ID), `GET CONFIGURATION`, `REQUEST SENSE`, and the sense key /
  [ASC](#7-acronyms) / [ASCQ](#7-acronyms) tables. Working drafts are freely downloadable; ratified INCITS
  editions are not, and the final draft matches the published text for our
  purposes. Pin a revision when citing: the project relies on MMC-3 or later.
  <https://www.t10.org/drafts.htm>
- **T10 ASC/ASCQ assignment list** — the authoritative sense-code table. Every
  sense code in this project has been checked against it, including the six
  distinct meanings under ASC `6Fh`.
  <https://www.t10.org/lists/asc-num.txt>

### Content protection

- **AACS — *Advanced Access Content System*, Common Cryptographic Elements**
  — the specification libaacs implements: Volume ID, Media Key Block, [VUK](#7-acronyms),
  and the fact that the Volume ID is obtained by an MMC command and not read
  from the filesystem, which is the whole reason a key database and a drive
  handshake are two separate paths (§5).
  <https://aacsla.com/specifications/>
- **Android storage** — scoped storage, primary shared storage, and
  `MANAGE_EXTERNAL_STORAGE`. What decides where a key database can live on a
  modern device, and why `dirs_android.c` resolves one root under four names.
  <https://developer.android.com/training/data-storage>

### Media — what comes back from the disc

- **ECMA-267, *120 mm DVD — Read-Only Disk*** (3rd ed., April 2001; ISO/IEC 16448)
  — physical format, sector layout, the Physical Format Information and
  Copyright Management Information returned by `READ DVD STRUCTURE`, PTP/OTP
  numbering and the layer break. Free.
  <https://ecma-international.org/publications-and-standards/standards/ecma-267/>
- **ECMA TR/71, *DVD Read-Only Disk — File System Specifications*** (Feb 1998)
  — the bridge between the physical layout and the file system: where logical
  sector 0 sits, where the Anchor Volume Descriptor Pointers are. Short and
  free.
  <https://ecma-international.org/publications-and-standards/technical-reports/ecma-tr-71/>
- **ECMA-167, *Volume and file structure … non-sequential recording*** (3rd ed.;
  ISO/IEC 13346) — the normative base for [UDF](#7-acronyms). Free.
  <https://ecma-international.org/publications-and-standards/standards/ecma-167/>
- **[OSTA](#7-acronyms) UDF** — 1.02 is the profile a DVD-Video actually requires, in
  UDF/ISO 9660 bridge form, which is why an ECMA-119 (ISO 9660) structure is
  present on the same volume; a BD-ROM requires 2.50. Both are what libudfread
  implements, and libbluray reads a BD volume through it, on top of the access
  module's sector reads.
  <https://www.osta.org/specs/>
- **ECMA-130** — the CD equivalent of ECMA-267, for the CD-Audio work, and the
  normative description of the CD sector modes a Video CD's payload uses. Free;
  the Red Book (IEC 60908) is not.
  <https://ecma-international.org/publications-and-standards/standards/ecma-130/>
- **ECMA-119, *Volume and File Structure of CD-ROM*** (ISO 9660) — the volume
  descriptor at sector 16, the `CD001` identifier, the directory record layout
  and the Volume Identifier field. This is what the Video CD probe in `disc.c`
  implements two levels of, and the reference to check its offsets against.
  Free.
  <https://ecma-international.org/publications-and-standards/standards/ecma-119/>
- **Video CD 2.0 ("White Book")** and **IEC 62107 (Super Video CD)** — the
  `/VCD` and `/SVCD` directory layouts, `INFO.VCD` / `INFO.SVD` and their
  signatures, and the Mode 2 Form 2 track structure. Neither is free, and
  neither was needed to build this: the identification agrees with what VLC's
  own `vcd` module reads, which is the check that actually matters. Listed for
  anyone extending beyond identification — the `ENTRIES` and `LOT` structures
  in particular are described nowhere else.

## 7. Acronyms

| Acronym      | Meaning |
| ------------ | ------- |
| AACS         | Advanced Access Content System. Blu-ray's content protection, implemented by libaacs. |
| ASC / ASCQ   | Additional Sense Code and its Qualifier. The two bytes that say what a SCSI command actually failed on; the sense key alone rarely does. |
| BBB          | Bulk/Bulk/Bulk. The USB Mass Storage specification's own name for Bulk-Only Transport, and the reason its protocol code is what it is. |
| BOT          | Bulk-Only Transport. The USB Mass Storage transport this library speaks, and what `bot.c` is named after. |
| CBW / CSW    | Command Block Wrapper and Command Status Wrapper. The header that carries a CDB out and the trailer that reports what happened, one pair per transaction. |
| CDB          | Command Descriptor Block. The SCSI command itself, 6 to 16 bytes, riding inside a CBW. |
| CSS          | Content Scramble System. DVD-Video's protection. Nothing to do with stylesheets. |
| LBA          | Logical Block Address. A sector number, counted from the start of the medium unless something says otherwise. |
| LUN          | Logical Unit Number. Which unit behind one USB device a command is addressed to; an optical drive is rarely LUN 0 on a multi-slot bridge. |
| MMC          | Multi-Media Commands. The SCSI command set for optical drives. Not MultiMediaCard, which is a different thing entirely. |
| MRL          | Media Resource Locator. VLC's URI-like string naming both what to open and which module opens it. |
| OSTA         | Optical Storage Technology Association. Publishes UDF. |
| PTP / OTP    | Parallel Track Path and Opposite Track Path. Which direction the second layer of a dual-layer DVD is read in. |
| SBC / SPC    | SCSI Block Commands and SCSI Primary Commands. The two command sets MMC sits on top of. |
| SCSI         | Small Computer System Interface. The command language, still spoken by every optical drive whatever it is plugged into. |
| T10 / INCITS | The technical committee that publishes the SCSI standards, and the body it belongs to. |
| TOC          | Table of Contents. A CD's track list, read with one command and the only cheap way to know a medium is a CD. |
| UDF          | Universal Disk Format. The filesystem on every DVD and Blu-ray, and the one libdvdread actually reads. |
| VCD / SVCD   | Video CD and Super Video CD. MPEG-1 and MPEG-2 video on a CD, read through ISO9660. |
| VUK          | Volume Unique Key. The per-disc key AACS derives before anything can be decrypted. |
