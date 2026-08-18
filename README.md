# LASER — Library for Android-accessed SCSI External Readers

## 1. Goal

Let VLC on Android play optical discs — CD-Audio, Video CD, DVD-Video and
Blu-ray — from a USB drive attached to the phone or Smart TV Box, on unrooted
stock Android, and on **any hardware that conforms to the applicable
standards** rather than on a list of blessed models.

Android grants userspace access to a USB device as a file descriptor obtained
through `UsbManager`. There is no block device, no mount point, and no
`/dev/sr0`: everything VLC's optical stack normally relies on is absent. The
work is therefore to give that stack a path to a drive it can only reach by
sending SCSI command blocks over USB bulk endpoints itself.

## 2. User's manual

*This assumes you use a **VLC for Android** buid that includes this library and where all the
related patches to libdvdcss, libvlc and vlc-android have been applied.*

**Play a disc.** Plug the drive in, grant the USB permission Android asks for,
and open *Browse*. A tile appears per disc found, named after the disc. Tap it.

**Several drives at once** each get their own tile, or tiles. Android asks a
separate permission per drive, one dialog after the other, and the tiles keep
a fixed order — they are sorted by the port a drive is plugged into, not by
whichever permission you granted first, so they do not move around between
launches.

**DVDs offer two tiles.** One with the disc's own menus, one labelled
*(No menus)* that starts the first title directly. Use the second when a
disc's menus do not respond, or to skip straight to the film.

**Audio CDs** appear as one tile holding the disc's tracks. **Video CDs and
Super Video CDs** appear as one tile each, named after the disc — many are
labelled simply *VIDEOCD*, which is the label the disc itself carries.

**Blu-rays** appear as one tile. An unencrypted volume plays as-is. A
commercial one needs AACS. To enable it, you need to:
- Place `libaacs.so.0` at `/sdcard/aacs/<abi>/`, grant VLC *All files access*
  (Android 11 and later) and relaunch it. For security reasons, only files
  matching whitelisted SHA-256 signatures are accepted. If the installation
  succedded, the file and <abi> folder are deleted;
- Place `KEYDB.cfg` at `/sdcard/aacs/KEYDB.cfg`.

**Unplugging** is safe at any time: playback stops and the tile disappears.

**What is deliberately absent.** Disc entries never enter the media library and
never appear in *Recent* or *Continue watching*: the fd inside their MRL is
valid only for the current connection, so a stored one would point at nothing
or, worse, at something else (§6). Resume-where-you-left-off therefore does not
apply to discs.

**When something does not play.** The log tag is `Laser` for the transport and
`VLC/LaserDrive` for the Android side; every transport line carries the drive's
`vid:pid:bcd`, which is what a bug report needs.

**In case of low DVD framerate,** especially on older arm setup, set hardware
acceleration to `Disabled` or `Automatic`.

## 3. Approach

Four decisions shape everything else.

**The fd is the token.** The descriptor Kotlin obtains from
`UsbDeviceConnection.getFileDescriptor()` is carried, as a decimal number
inside an MRL, all the way down to the transport, where it identifies the
device in a small registry. No parallel handle type, no registration call from
the Java side, no lifecycle to keep in sync. The cost is that the token is
meaningful only within one process and one connection, which makes an
laser MRL a *session-scoped* name: it may never enter a persistent store,
and the connection behind it needs an owner that outlives the screen that
opened it. Both are handled — see §6.

**One shared contrib, not per-module code.** `liblaser` owns the
device registry (`registry.c`), the USB interface and endpoint discovery
(`usb.c`), the Mass Storage Bulk-Only transport (`bot.c`), the SCSI command
layer (`scsi.c`) and the identification of the disc in the drive (`disc.c`) —
one file per concern, since the three transport ones started life as a single
one and the split follows the boundaries its own header comment already named.
The VLC access module, libdvdcss, libdvdread/libdvdnav and the CD-Audio module
all reach the drive through it. Retry policy, LUN selection, sense-code
interpretation and error semantics exist once. libbluray is the one consumer that needs none
of it: it reads through the access module's stream (§6).

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
connection's lifetime is reconciled from events libVLC already emits (§6). It
also decides what does *not* get written: where a behaviour was missing it was
added in the shape VLC already uses, in the module that owns it, rather than
routed around from outside — hence the small, local patches to `cdda.c`,
`dvdread.c` and libdvdcss.

### Naming

One name throughout, so that a single string finds every part of the feature:

| Thing                       | Name                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|-----------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Contrib                     | `contrib/src/laser`, dev tree `workspace/laser`                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| Static library              | `liblaser.a`, pkg-config `liblaser.pc`, link flag `-llaser`                                                                                                                                                                                                                                                                                                                                                                                                                            |
| Public headers              | `laser.h`, `laser_disc.h` (private: `laser_internal.h`)                                                                                                                                                                                                                                                                                                                                                                                                                                |
| C symbols                   | `laser_*` for functions and types, `LASER_*` for constants                                                                                                                                                                                                                                                                                                                                                                                                                             |
| VLC access module           | `laser` — `modules/access/laser.c`, plugin `liblaser_plugin.la`                                                                                                                                                                                                                                                                                                                                                                                                                        |
| Build macro                 | `-DHAVE_LASER`, guarding the branches added to `cdrom.c`, `cdrom.h`, `vcd.c`, `dvdnav.c`, `dvdread.c` and libdvdcss. The libaacs patch (§7) carries no such guard and is not part of this list: it is keyed on the *platform*, not on this project, and mentions neither USB nor `laser`. `HAVE_*` rather than a reserved `__NAME__`, and set by the build system rather than inferred from a header, because it answers "is liblaser in this build" — a question no header can answer |
| MRL access name / authority | `laser` (see the table in §4)                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| Kotlin                      | `LaserOpticalDrive.kt`, `LaserDiscProvider.kt`, `LaserConnections`, tile scheme `lasertile://`                                                                                                                                                                                                                                                                                                                                                                                         |
| Licence                     | LGPL 2.1 or later — terms both libdvdcss (GPL, which links it) and VLC's own modules (LGPL) can accept                                                                                                                                                                                                                                                                                                                                                                                 |
| libdvdcss hand-off          | `dvdcss_swap_laser_token()`, `dvdcss_s::b_laser_session`, and the `laser_acquire()`/`laser_release()` pair that keeps the device alive while that library and the access module both hold the same token. Swap and not set: the call returns what it replaces, so the bracket `dvdnav.c` and `dvdread.c` each open around their disc open restores rather than clears, and one nested inside the other cannot disarm it                                                                |

The one place the name deliberately does *not* appear is prose describing the
platform rather than the component: an Android USB optical drive is still an
Android USB optical drive, and log lines and module descriptions say so.

## 4. Architecture

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
  │                │◄──│     bluray       │  │  access "vcd"   │
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

| MRL                                                                 | Purpose                                                                                                                                                                                                                                                                                                  |
|---------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `file://laser/<fd>`                                                 | Classification. The `file` scheme is borrowed because the preparser types items from a fixed scheme table and never opens a module for a scheme it does not know (§6).                                                                                                                                   |
| `laser/dvd://<fd>`, `laser/dvdsimple://<fd>`, `laser/bluray://<fd>` | Video playback. libVLC reads this as access `laser` plus a demuxer name. A DVD is offered under two of them — with menus and without (§6).                                                                                                                                                               |
| `cdda://laser/<fd>`                                                 | CD-Audio. Here the drive is the URI's *authority*, not the access module: `cdda` is an access_demux and reaches the contrib itself. It expands into one MRL per track, `…/Track%20NN` (§6).                                                                                                              |
| `vcd://laser/<fd>`, `svcd://laser/<fd>`                             | Video CD and Super Video CD. Same shape and same reason as the line above — the drive is the authority, and `vcd` reaches the contrib through the same `cdrom.c` as `cdda`. A Video CD is a *video* disc that stays on the *CD* path, because what decides the path is the sector, not the content (§6). |

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

## 5. What works today

**Detection and classification.** A drive is recognised on attach, the disc is
identified (DVD-Video, BD-Video, Video CD, Super Video CD, CD-Audio, or data),
and a tile appears in the browser with the disc title — two tiles for a DVD,
one per way of playing it (§6). One group of tiles per drive: several drives
can be attached at once, each is classified in turn, and the groups are
ordered by device name so that the asynchronous order in which permissions are
granted does not decide where a disc's tiles land. Identification is the contrib's, not the
module's, and it probes in ascending order of cost: `READ TOC` for an audio CD
first, then one mount of the UDF filesystem answering for both DVD and BD,
then — only on the discs the first two did not claim, *and* only on media the
first step established is a CD at all — a minimal ISO9660 walk for the two
Video CD kinds. Each step reads the filesystem the corresponding reader will
itself use, which is why the two walks are different rather than one being a
fallback for the other (§6). The `READ TOC` of step 1 answers three questions
at once for the price of one command: is this an audio CD, is it a CD, and
where does its first data track start — so a data DVD never pays for the
ISO9660 walk, and the walk addresses the track rather than the medium. Hot-plug and hot-unplug are handled,
including multi-function enclosures where Android enumerates each function as
a separate attach event.

**Transport.** Bulk-Only Transport with the interface and endpoints resolved
from the descriptor, Mass Storage Reset, spin-up wait with a bounded budget,
LUN discovery for combo drives, a retry policy keyed on what the command
actually is, and sense-code classification precise enough to tell a scrambled
sector from a lost key session from a region mismatch.

**DVD-Video playback with CSS.** The full authentication handshake runs over
the USB transport: menus, navigation and feature playback all work. libdvdcss
is patched at the `ioctl_*` layer only — every function keeps its signature
and gains a branch. A CSS session is held for the lifetime of a `dvdcss_t` and
excludes other consumers, which prevents an unrelated component from consuming
one of the drive's four AGIDs mid-handshake.

**CD-Audio playback.** TOC read over the transport, then one playlist entry per
track, each playing to the end and advancing to the next. This needed one
change in `cdda.c` that has nothing to do with USB: its track sub-items were
distinguished by input options alone, which VLC-Android drops when it rebuilds
a sub-item from its URI, so the thirty tracks of a CD collapsed onto one MRL
and the input looped forever (§6).

**Video CD and Super Video CD playback.** Identified by ISO9660 — `/VCD/INFO.VCD`
carrying `VIDEO_CD`, `/SVCD/INFO.SVD` carrying `SUPERVCD` — and played through
VLC's own `vcd` module over the same `cdrom.c` path as an audio CD. The
directory alone is never the answer: a data disc may carry a folder called
`VCD`, so the signature inside the info file is what settles it, exactly as
`DVDVIDEO-VMG` does for a DVD.

**Raw CD sector sizes.** 2352-byte sectors have their own helper,
`laser_read_cd_blocks()`, which issues `READ CD` and takes the sector kind as
an argument: CD-DA with User Data only for audio, Mode 2 Form 2 with sync and
both header codes for a Video CD. Both come back as 2352 bytes per sector, so
everything above that call is indifferent to which was asked for — including
the Mode 2 unpacking, which `cdrom.c` already did generically for every
platform. The 2048 check against `READ CAPACITY` lives on the `laser://`
stream path, which no CD ever takes. The two sizes never meet.

**Region mismatch explained rather than suffered.** Before the first read of a
playback session, the drive's RPC state and the disc's region management
information are compared, and a mismatch is reported in one sentence naming
both regions. Silent on a CD, a BD, a region-free disc, an RPC-1 drive, or a
drive with no region set yet — all of which are cases where the two commands
simply do not answer, so the check gates itself without being told what is in
the drive.

**Getting libaacs onto the device.** libaacs is not shipped inside the
application, and cannot simply be: libbluray loads it with `dlopen` under the
name `libaacs.so.0`, and an APK's native library directory holds only files
named `lib*.so` — a versioned suffix cannot be packaged there. A drop in
`/sdcard/aacs/<abi>/libaacs.so.0` is therefore copied at first start into the
application's private storage, checked against a SHA-256 pinned per ABI,
made read-only before being renamed into place, and preloaded once to prove
the linker accepts it. libbluray is then pointed at it through `LIBAACS_PATH`,
which it treats as a path *prefix* and completes with `.so.0` itself — which
is what lets all of this work without patching libbluray at all. The whole
mechanism lives in one file and one build flavour, both absent from the
`vlcBundle` build.

**Lifetime of the descriptor.** Two halves, answering different questions.
Natively, the device registration is **reference-counted**: every consumer
declares itself with `laser_acquire()` and drops out with `laser_release()`,
and the USB handle is closed when the last claim goes rather than when the
first consumer finishes. Playing a DVD means the access module and libdvdcss
hold the same token at the same time, and before the count existed the
correctness of that rested on libVLC closing the demuxer before the access —
an ordering libVLC provides and does not promise. On the Java side, the
classification connection is opened and closed within one call, while the
playback connection is owned by `LaserConnections` and closed when the
playlist no longer names its descriptor — reconciled from
`MediaPlayer.Event.Stopped` in `PlaylistManager`, and from the service's own
teardown, which is also the backstop against a claim that is never released. A
connection just opened is spared the next few reconciliations, since the
playlist that will name it does not exist yet (§6). Nothing that names a
descriptor reaches the medialibrary or the resume preferences: the `fd://`
guards already in `PlaylistManager` were widened to cover laser locations
rather than duplicated beside them.

Against a claim that is never released anyway, `laser_acquire()` checks that
the descriptor it is handed still refers to what was registered under that
number, and registers afresh when it does not. Descriptor numbers are reused
by the operating system, so an entry outliving its device can otherwise be
served for a drive that is no longer there. The check is `fstat()` on the
descriptor itself — a strong heuristic, not a guarantee, and no substitute for
releasing claims.

**Throughput.** Reads are windowed and prefetched by a dedicated thread, and
the transfer size is negotiated per device. Measured on a DVD-9: ~1 MB/s
sustained with the drive idle 95% of the time, from ~20 SCSI commands per five
seconds. Playback is smooth end to end.

## 6. Design decisions worth remembering

| Decision                                                                                       | Why                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Borrow the `file://` scheme for classification                                                 | A private scheme never reaches an access module: the preparser types items from a fixed scheme table and skips anything it does not recognise.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| Refusals are recorded per LBA, not per session                                                 | A session-wide latch turned one scrambled VOB into end-of-stream for the whole disc, including the unscrambled `.IFO` files behind it.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| Readable prefix preserved on a refused range                                                   | A refusal applies to the whole SCSI command; a window straddling the boundary between an `.IFO` and its scrambled `.VOB` must not lose the readable part.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| Retry policy keyed on key format, not opcode                                                   | `REPORT KEY` carries both state-changing handshake steps and harmless queries. Retrying an AGID request can exhaust all four AGIDs; refusing to retry a copyright query would disable CSS for the whole disc.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| Transfer size negotiated, not configured                                                       | Bridge limits vary and are not discoverable from any descriptor.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| A laser MRL never enters a persistent store                                                    | It names a descriptor valid for one connection in one process. Stored, it comes back at the next launch naming a different device, an ordinary file, or nothing — a *wrong* answer rather than a missing one. The existing `fd://` guards in `PlaylistManager` were widened rather than duplicated; the symptom was an offer at startup to resume a disc that had left the building.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| The playback connection's lifetime is reconciled against the playlist, not paired with an open | Pairing leaks whenever one half is missed, and there are many ways to miss it: playback that never starts, a demuxer that rejects the disc, an input error, the app being swiped away. Reconciliation names no exit path, so it cannot miss one. Its one blind spot is a connection just opened and not yet in any playlist, which is indistinguishable from one whose entry has gone — so a fresh connection is spared a bounded number of reconciliations rather than a bounded number of milliseconds: what has to happen first is an event, the playlist being replaced, not the passage of time.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| …and reconciled from events libVLC emits, never from a decision taken in Kotlin                | Every module releases its token in its own `Close()`, so `MediaPlayer.Event.Stopped` is emitted only after the libusb handle has been released. Doing the same work in `stop()` would close a descriptor libusb still held. This is also why no JNI call is needed to make the ordering safe.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| The track number is carried in the CD-Audio MRL, not only in an input option                   | `cdda.c` distinguishes tracks by options; a host that cannot carry options across a sub-item collapses them all onto one MRL and loops forever. The `/Track NN` syntax was already parsed by `DiscOpen()` — nothing emitted it. Emitted only where it round-trips, i.e. where `DiscOpen()` reads the location rather than a file path.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| The drive's region is read, never set                                                          | `SEND KEY` format 06h would set it, and the counter of permitted changes is small and, once exhausted, permanent. A drive bricked into one region by a media player the user did not think was making that decision is a worse outcome than a disc that will not play. The check is advisory and cannot refuse a disc.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| A DVD is listed twice, with menus and without                                                  | dvdnav is the right default — it is what the disc's author intended and what a DVD player does. But a disc whose navigation it cannot follow plays as a black screen through it and plays fine through the plain reader, and which of the two a given disc needs is a judgement only the person watching can make. So it is offered as a second row rather than guessed at, or buried in a long-press menu. The cost was a stream-based `demux` submodule in `dvdread.c`, which had only ever been an `access_demux` — reachable by name (`dvdsimple`) and never probed, so that "no menus" is asked for and never inferred.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| Disc identification lives in the contrib, behind `laser_disc_identify()`                       | It reads sectors and issues one SCSI command; none of it is VLC's, and any consumer holding a token wants the same answer. The move also let libudfread stop being installed — a vendored dependency has no business in the interface of the library that vendors it — and let the hand-rolled ISO9660 walk be dropped from the DVD path: DVD-Video discs carry both filesystems, but libdvdread finds its files through UDF and never falls back, so identifying through UDF is identifying through what the reader will actually use. **Video CD later brought an ISO9660 walk back — and that is the same rule reaching a different answer, not a reversal of it:** a VCD has no UDF at all and VLC's `vcd` module reads it through ISO9660, so "identify through the filesystem the reader uses" now selects ISO9660 there and still selects UDF for a DVD. The walk is two directory levels deep and stops: no Joliet, no Rock Ridge, no recursion, no path table, because those exist to find arbitrary names and this looks up two it already knows.                                                                     |
| A Video CD is routed like an audio CD, not like a DVD                                          | What decides the path is the sector, not the content. A VCD's payload is Mode 2 Form 2, read with `READ CD`; a `stream_t` hands out a flat run of 2048-byte blocks and can express neither that nor a raw CD-DA sector. So the MRL names VLC's `vcd` access module rather than ours, and the disc never touches the `laser://` stream path — the same reasoning that kept CD-Audio off it.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| `laser_read_cd_blocks()` took a sector-kind argument rather than being duplicated              | `READ CD` carries the answer in two unrelated CDB bytes — an Expected Sector Type and a field-selection bitmap — whose legal pairings are a property of the CD format, not of any caller. The parameter is an enum naming the *sector kind*, so the pairing stays in one place and no caller can invent a combination it has no way to validate.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| `vcd.c` needed a location fallback that `cdda.c` did not                                       | `psz_filepath` comes from `vlc_uri2path()`, which answers "what file does this URI name" — and `laser/5` names no file: it is a descriptor reaching the drive over SCSI, not through the filesystem. `cdda.c` already falls back from the path to the location, because it had to support the GNOME `…/Track NN` syntax, and that fallback carries the token for free. `vcd.c` never grew an equivalent because it never had a second syntax to support — so the asymmetry is in VLC, not an oversight here.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| VCD and SVCD are two MRLs but one Kotlin disc type                                             | The MRL is what a bug report quotes and what names the access module, so `svcd://` for a Super Video CD costs nothing and is true. The Kotlin enum answers "what kind of disc" for the icon and the playback flags, and neither acts on the difference — exactly as `dvd` and `dvdsimple` are two MRLs and one `DVD_VIDEO`. A finer distinction stays available by testing the MRL, the way `isLaserNoMenuMrl()` does.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| The libaacs patch touches no MMC code                                                          | An Android application cannot reach a drive through libaacs' own device layer in the first place — `mmc_device_linux.c` opens a device node, and there is none. It already degrades correctly there: `device_open()` fails and a disc whose key is in a database opens anyway. So the patch fixes only what actually prevents libaacs from working on Android — that it has no `$HOME` to look for that database in, and no visible stderr to say so on. Adding an MMC path is a separate chantier and is described as one (§7).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| libbluray takes no token                                                                       | `bd_open_stream()`'s `read_blocks` callback is VLC's own, reading the demuxer's stream — i.e. the access module, already a consumer. There is no second path to build, and adding one "for symmetry" with libdvdcss would create a double release. The consequence is that libbluray reaches the disc but not the drive (§7).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| Device registrations are reference-counted, and claims are declared rather than inferred       | A token has several consumers at once; without a count the first one out tore the device down for the others, and that this was harmless rested on libVLC's teardown order rather than on anything enforced. Counting was rejected once, on the grounds that "never released" is a worse failure than "released too early" on a descriptor Kotlin is waiting to close — an argument that stopped holding when `LaserConnections` started reconciling connections against the playlist, since that is exactly a backstop against a leaked claim. `laser_acquire()` is now the only thing that registers a device at all: every other entry point looks the token up and answers `LASER_ERR_NO_SUCH_TOKEN` if there is none. Commands used to register their own device on the spot, which produced registrations no consumer had claimed — and since teardown only happens when a count falls to zero, a count never incremented never falls, so nothing could ever destroy them. Every consumer already acquired first, so removing that path cost no call site anything; what it removed was a state nothing could get out of. |
| Nothing but `laser_*` leaves `liblaser.a`                                                      | libbluray bundles its own copy of libudfread, exporting the same `udfread_*` names, and an Android VLC links both archives — so which copy satisfies a given reference becomes a property of archive member ordering. The three object sets are therefore partially linked into one relocatable object, and everything outside the documented API is demoted to local. Per-object `objcopy` cannot do this: localizing `laser_lookup()` in `registry.o` would break `scsi.o`, `laser_bot_send_locked()` crosses from `bot.o` to `scsi.o`, and libusb's files call into each other throughout. The link step asserts both directions — that nothing foreign escaped, and that every declared entry point is actually defined, since `objcopy --keep-global-symbol=` on a name that does not exist is ignored without a word.                                                                                                                                                                                                                                                                                                     |
| Cancellation belongs to the last release, not to a public call                                 | The flag is sticky and token-wide, while a claim is one of several: a public cancel would let one consumer disable the drive for every other holder of the same token, permanently — reintroducing exactly the teardown-ordering dependency the count exists to remove. Cancelling always meant "I am the last one out and I am done", which is what dropping the last claim already says, so that is where it lives. Sticky removes the question of who clears it, the slot being memset moments later. Checked between attempts and between chunks, never inside a transfer already in flight. What not exposing it costs: a `Close()` whose read-ahead worker sits inside a block read on a wedged drive now waits out a full retry budget rather than one phase timeout, `b_quit` being read only between two reads. Getting to zero means libusb's asynchronous API.                                                                                                                                                                                                                                                       |
| Diagnostics go through a callback                                                              | A library that has decided where its logs go has decided it for every application that embeds it. `laser_set_log_cb()` defaults to Android's logger, which is right inside the app and useless — and unsilenceable — anywhere else. Same shape as libbluray's `bd_set_debug_handler()`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| libbluray reaches libaacs by `dlopen`, so the application installs it rather than packaging it | libbluray does not link libaacs; it opens `libaacs.so.0` at runtime. An APK cannot carry that name — only `lib*.so` is packaged — so no amount of building libaacs into the application would let libbluray find it. `LIBAACS_PATH` is the seam libbluray already offers, and it holds a path *prefix* that libbluray completes with `.so.0`, which is what makes an installed copy reachable without patching libbluray. The copy is hashed against a per-ABI pin after being copied and before being renamed into place, so what is verified is the private copy and not the shared-storage file that could change between the two.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| libaacs' key cache goes to private storage, its key database does not                          | `file_get_cache_home()` would otherwise put the keys libaacs derives beside `KEYDB.cfg` in shared storage, readable by any application holding *All files access* and surviving the uninstall. The two directories are asymmetric on purpose: a human has to be able to drop the database in, and nothing has to be able to read the cache out. `XDG_CACHE_HOME` is the redirection libaacs already honours, so it costs one `setenv` and no patch.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| One tile group per drive, ordered by device name                                               | The browser used to stop at the first drive holding a disc, which meant a second drive was never even classified. Classification is now queued and serial — two overlapping passes would stack two system permission dialogs — but nothing is dropped, which removes the hand-rolled re-entry that used to catch the candidates the previous gate turned away. Display order comes from sorting device names rather than from arrival, since arrival order depends on which permission dialog the user answers first and would move a disc's tiles between two launches.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |

## 7. Remaining work

### Cheap to settle

- **Unencrypted Blu-ray.** A BDMV volume without AACS should play as-is, since
  libbluray reads through the access module. This is a test, not a
  development.
- **Video CD, on real hardware.** Written and verified piecewise, never through
  a drive. Watch the `READ CD` flags: Mode 2 Form 2 is asked for without EDC/ECC
  (byte 9 = `0xF0`), as VLC's own BSD backend has long done. A drive that counts
  the four EDC bytes anyway returns 2348 per sector, which the strict length
  check reads as a short read; `0xF8` is the fix if that ever shows up.
- **Commercial Blu-ray from a key database.** Every piece is now in place and
  none of them has met a disc. libaacs can find `KEYDB.cfg` on Android (§5),
  the application installs `libaacs.so.0` where libbluray's `dlopen` will find
  it (§5), and libbluray reads the volume through the access module. What has
  never been exercised is the join: whether libbluray actually opens the
  installed library on a device, whether the key in a real database matches a
  real disc, and whether an AACS volume then decrypts through this transport.
  Testing it needs a commercial Blu-ray, a key database, a libaacs build whose
  fingerprint is pinned, and *All files access* granted to VLC — and the
  fingerprint table currently holds placeholders, so this cannot be tried at
  all until somebody publishes the binaries it is meant to pin.

  Two things are worth knowing before the first attempt. libbluray's own
  diagnostics go to `stderr`, which is `/dev/null` on Android — the same
  problem the libaacs patch fixes for libaacs — so if the library is not found
  there will be no message saying so; the installer's own log lines are the
  only trace. And a failure here is at least four candidates deep: the drop,
  the fingerprint, the `dlopen`, the key. Working through them in that order
  costs less than guessing.

### Functional

- **AACS from the drive, for the discs a key database does not cover.**
  libbluray reaches the disc through the stream, but the AACS handshake needs
  MMC commands of its own — the Volume ID above all — which a stream cannot
  carry. Making those discs play means giving libaacs the treatment libdvdcss
  got: a device handle routed through the contrib, plus the equivalent of the
  CSS session exclusion. This is a chantier on the scale of the whole CSS
  effort, not a wiring job, and it is unchanged by the patch below — which
  deliberately touches no MMC code (§6).

### Dormant

- **Layer break awareness.** `READ DVD STRUCTURE` format `00h` gives the layer
  break address and the PTP/OTP direction of a dual-layer disc. Aligning
  windows so they do not straddle it would avoid a mechanical seek per
  crossing. With the drive idle 95% of the time, this is theoretical.
- **Recovery of latched refusals after a successful authentication.** A sector
  latched as scrambled before CSS engaged stays latched for the session. The
  clean fix is a generation counter on the CSS session, sampled on each read.
  Rarely reachable in practice, since authentication happens during open.
- **Video CD in a later session.** The ISO9660 walk now addresses the *track*:
  it reads sector 16 relative to the first data track's start LBA, taken from
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

### Separable for upstream

Two pieces stand on their own and mention neither USB nor this project.

**The `cdda.c` change.** It fixes a class of problem — a host that cannot carry
input options across a sub-item — of which VLC-Android is one case, and it
mentions nothing about USB or Android. It is proposable independently of
everything else here, and is the easiest piece to get accepted.

**The libaacs Android patch.** Pure platform portability, keyed on `*android*`
in `configure.ac` and useful to any Android application linking libaacs. Two
fixes: no `$HOME` or XDG variables, so `dirs_xdg.c` silently finds no
`KEYDB.cfg` — `dirs_android.c` resolves one shared-storage root under four
names; and stderr on `/dev/null`, so `BD_DEBUG` now also reaches logcat.

## 8. Reference documents

### Platform — how the descriptor arrives

- **Android USB host API** — `UsbManager`, permission intents,
  `UsbDeviceConnection.getFileDescriptor()`. The whole "fd is the token"
  design rests on what this returns and on who owns it.
  <https://developer.android.com/guide/topics/connectivity/usb/host>

### Transport — how to talk to the drive

- **USB Mass Storage Class — Bulk-Only Transport (BBB), rev 1.0** — CBW/CSW
  framing, the thirteen host/device data-transfer cases, stall recovery, Reset
  Recovery, Phase Error semantics. Governs `bot.c`, and the interface and
  endpoint discovery and Mass Storage Reset in `usb.c`.
  <https://www.usb.org/document-library/mass-storage-bulk-only-10>
  (PDF: <https://www.usb.org/sites/default/files/usbmassbulk_10.pdf>)
- **USB Mass Storage Class Specification Overview, rev 1.4** — subclass and
  protocol code assignments; what makes an interface the mass-storage one.
  <https://www.usb.org/sites/default/files/Mass_Storage_Specification_Overview_v1.4_2-19-2010.pdf>
- **INCITS T10 MMC / SPC / SBC** — the SCSI command set itself: `READ(10)` and
  `READ CD`, `READ TOC`, `REPORT KEY` / `SEND KEY`, `READ DISC STRUCTURE`
  (`READ DVD STRUCTURE` in earlier editions, and the command that would carry
  a BD Volume ID), `GET CONFIGURATION`, `REQUEST SENSE`, and the sense key /
  ASC / ASCQ tables. Working drafts are freely downloadable; ratified INCITS
  editions are not, and the final draft matches the published text for our
  purposes. Pin a revision when citing: the project relies on MMC-3 or later.
  <https://www.t10.org/drafts.htm>
- **T10 ASC/ASCQ assignment list** — the authoritative sense-code table. Every
  sense code in this project has been checked against it, including the six
  distinct meanings under ASC `6Fh`.
  <https://www.t10.org/lists/asc-num.txt>

### Content protection

- **AACS — *Advanced Access Content System*, Common Cryptographic Elements**
  — the specification libaacs implements: Volume ID, Media Key Block, VUK,
  and the fact that the Volume ID is obtained by an MMC command and not read
  from the filesystem, which is the whole reason a key database and a drive
  handshake are two separate paths (§7).
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
  ISO/IEC 13346) — the normative base for UDF. Free.
  <https://ecma-international.org/publications-and-standards/standards/ecma-167/>
- **OSTA UDF** — 1.02 is the profile a DVD-Video actually requires, in
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