# Makefile: builds liblaser.a and installs it + the public
# header under $(PREFIX).
#
# liblaser - Library for Accessing SCSI External Readers.
#
#
# Layout of this directory:
#   src/               - our own registry.c / usb.c / bot.c / scsi.c / disc.c
#                        and headers
#   contrib/libusb/    - vendored libusb 1.0.30, a COMPLETE upstream release
#                        tree, compiled directly by this Makefile - same as
#                        our own files - instead of driving its own
#                        ./configure. The config.h that ./configure would
#                        have generated is not written by hand and not
#                        generated either: upstream ships one for exactly
#                        this case at contrib/libusb/android/config.h, which
#                        is what its own ndk-build (android/jni/libusb.mk)
#                        uses, and which is why the whole release tree is
#                        vendored rather than just libusb/libusb/. See
#                        LIBUSB_CONFIG_DIR below.
#   contrib/libudfread/- vendored libudfread upstream tree, used by
#                        src/disc.c to identify DVD-Video and BD-Video
#                        discs - see that file for why it is vendored here
#                        rather than taken from libbluray, which bundles
#                        it privately with no installed headers. Same
#                        treatment as libusb:
#                        compiled directly here rather than through its
#                        own meson build, no config.h needed since
#                        udfread.c's own #include "config.h" is already
#                        guarded behind #if HAVE_CONFIG_H, which we
#                        simply never define.
#
# SYMBOL CONFINEMENT. Both vendored trees end up inside liblaser.a, and one of
# them collides: libbluray bundles its own copy of libudfread, exporting the
# same udfread_* names, and an Android VLC links both archives. Which copy
# satisfies a given reference is then a property of archive member ordering -
# a duplicate-symbol error on a good day, a silent pick on a bad one. libusb
# has no such collision today, but it is the same exposure.
#
# So nothing but this library's own API leaves the archive. The three object
# sets are partially linked into ONE relocatable object, which resolves every
# internal reference between them, and objcopy then demotes every symbol
# except the documented entry points to local. What ships is an archive with
# a single member exporting exactly LASER_PUBLIC_SYMBOLS.
#
# WHY THE PARTIAL LINK IS NOT OPTIONAL. objcopy on the individual objects
# cannot work: --localize-hidden applied to registry.o would localize
# laser_lookup(), which scsi.o legitimately calls, and the same is true
# throughout libusb, whose files call into each other constantly. Localizing
# is only safe once there is no longer anything outside to reference - which
# is what "ld -r" produces.

SRC_DIR           := src
LIBUSB_ROOT       := contrib/libusb
LIBUSB_CONFIG_DIR := $(LIBUSB_ROOT)/android
LIBUSB_DIR        := $(LIBUSB_ROOT)/libusb
LIBUSB_OS_DIR     := $(LIBUSB_DIR)/os
LIBUDFREAD_DIR    := contrib/libudfread/src



VERSION = 0.1

# The confinement step below needs three tools beyond the compiler: a linker
# able to produce a relocatable object, objcopy, and nm.
toolchain_prog = $(firstword $(foreach c,$(1),\
                     $(wildcard $(shell $(CC) -print-prog-name=$(c) 2>/dev/null))))

# Does this name resolve to something that exists? An absolute path is checked
# on disk, a bare name against PATH.
tool_exists = $(if $(filter /%,$(1)),$(wildcard $(1)),$(shell command -v $(1) 2>/dev/null))


ifeq ($(call tool_exists,$(LD)),)
LD := $(or $(call toolchain_prog,ld.lld ld),ld)
endif
ifeq ($(call tool_exists,$(OBJCOPY)),)
OBJCOPY := $(or $(call toolchain_prog,llvm-objcopy objcopy),objcopy)
endif
ifeq ($(call tool_exists,$(NM)),)
NM := $(or $(call toolchain_prog,llvm-nm nm),nm)
endif

# Everything this library promises, and nothing else. The list is the union of
# laser.h and laser_disc.h; a function added to either and forgotten here does
# not fail to build, it fails to link in whoever calls it - so this is the one
# place to look when a brand-new entry point comes back "undefined reference".
LASER_PUBLIC_SYMBOLS := \
	laser_set_log_cb \
	laser_acquire \
	laser_release \
	laser_scsi_cdb \
	laser_css_session_begin \
	laser_css_session_end \
	laser_read_blocks \
	laser_read_cd_blocks \
	laser_status_is_positional \
	laser_region_mismatch \
	laser_disc_identify

# Linux/POSIX backend only.
#
# This list is upstream's Android list (android/jni/libusb.mk), not a
# reduction of it. linux_netlink.c in particular is NOT optional even though
# this project never enumerates anything: linux_usbfs.h resolves
# linux_start_event_monitor() to linux_netlink_start_event_monitor() whenever
# HAVE_LIBUDEV is undefined, and linux_usbfs.c calls it behind a RUNTIME test
# on no_device_discovery. Runtime, not preprocessor - so the reference is
# compiled in regardless, and leaving the file out costs an undefined symbol
# at link time rather than anything at build time. The code itself never runs
# in this project, LIBUSB_OPTION_NO_DEVICE_DISCOVERY returning before it.
LIBUSB_SOURCES := \
	$(LIBUSB_DIR)/core.c \
	$(LIBUSB_DIR)/descriptor.c \
	$(LIBUSB_DIR)/hotplug.c \
	$(LIBUSB_DIR)/io.c \
	$(LIBUSB_DIR)/strerror.c \
	$(LIBUSB_DIR)/sync.c \
	$(LIBUSB_OS_DIR)/linux_usbfs.c \
	$(LIBUSB_OS_DIR)/linux_netlink.c \
	$(LIBUSB_OS_DIR)/events_posix.c \
	$(LIBUSB_OS_DIR)/threads_posix.c

LIBUSB_OBJECTS := $(LIBUSB_SOURCES:.c=.o)

# Deliberately NOT default_blockinput.c (file/device-path based - we
# supply our own SCSI-backed udfread_block_input from src/disc.c
# instead) and NOT udfread-version.c (only referenced when
# HAVE_UDFREAD_VERSION_H is defined, which we don't - that macro guards
# a version string generated from ChangeLog by libudfread's own meson
# build, which we're bypassing entirely).
LIBUDFREAD_SOURCES := \
	$(LIBUDFREAD_DIR)/udfread.c \
	$(LIBUDFREAD_DIR)/ecma167.c

LIBUDFREAD_OBJECTS := $(LIBUDFREAD_SOURCES:.c=.o)

OUR_SOURCES := $(SRC_DIR)/registry.c \
               $(SRC_DIR)/usb.c \
               $(SRC_DIR)/bot.c \
               $(SRC_DIR)/scsi.c \
               $(SRC_DIR)/disc.c
OUR_OBJECTS := $(OUR_SOURCES:.c=.o)

# Warnings, on OUR code only. Not a global CFLAGS addition, because the same
# pattern rule below compiles vendored libusb and libudfread, which are not
# ours to keep warning-clean and whose noise would bury anything real. Scoped
# per-object exactly as -fvisibility=hidden and the libusb include path
# already are further down - the idiom is the file's own.
#
# Honest expectation: this catches little today. Its value is prospective, and
# it costs nothing.
$(OUR_OBJECTS): CFLAGS += -Wall -Wextra -Wshadow -Wvla -Wswitch-enum

# APPENDED, NOT ASSIGNED. This Makefile is driven by VLC's contrib system,
# which passes the cross-compilation flags - sysroot, target, API level -
# through the environment. A plain ":=" here overrides the environment and
# quietly drops all of them.
CPPFLAGS += -I$(LIBUSB_DIR) -I$(LIBUDFREAD_DIR)

# Scoped to the libusb objects rather than added globally, and the config
# directory is why. It contains a file called config.h, and putting a
# directory containing a config.h on the include path of an entire build is
# how a translation unit ends up compiled against a configuration meant for
# something else - VLC's own config.h being the obvious candidate in a tree
# that has one. Only libusb asks for <config.h>, so only libusb gets told
# where to find one.
#
# The os/ directory is here for the same reason it is in upstream's build:
# linux_usbfs.c includes "linux_usbfs.h" and events_posix.c includes
# "events_posix.h" as siblings, which works from the compiler's own
# directory-of-the-including-file rule, but linux_usbfs.h in turn reaches for
# headers that only resolve with os/ on the path.
$(LIBUSB_OBJECTS): CPPFLAGS += -I$(LIBUSB_CONFIG_DIR) -I$(LIBUSB_OS_DIR)

# -fvisibility=hidden on the VENDORED code only, and on none of ours.
#
# It is not what confines the symbols - the objcopy pass below is - but it
# does the same job earlier and better: hidden symbols cannot be preempted,
# so the compiler can inline and call them directly, and the partial link
# resolves them internally rather than leaving relocations for the final one.
# Belt and braces, at no cost.
#
# NOT applied to $(OUR_OBJECTS): laser_lookup() and laser_bot_send_locked()
# cross between registry.o, bot.o and scsi.o and must survive as far as the
# partial link, and
# the public entry points must survive it. Confining them is precisely what
# the --keep-global-symbol list expresses, one step later and by name.
$(LIBUSB_OBJECTS) $(LIBUDFREAD_OBJECTS): CFLAGS += -fvisibility=hidden

.PHONY: all install clean

# A recipe that fails must not leave its target behind. Without this, the
# objcopy step below can fail, make can stop, and the next invocation finds a
# laser-all.o newer than every object it was built from - so it skips the
# recipe entirely and archives an object whose symbols were never confined.
# That is not a hypothetical: it is how a cross build with the wrong objcopy
# turned into a duplicate-symbol error at the final link, two layers away.
.DELETE_ON_ERROR:

all: liblaser.a

%.o: %.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

# One relocatable object holding the three sets, with everything but the
# public API demoted to local.
#
# --keep-global-symbol rather than --localize-hidden: it says what stays
# rather than what goes, so a vendored symbol that is global for a reason we
# did not anticipate is confined anyway. The failure mode of the whitelist is
# also the safe one - forget an entry point and the link fails by name, where
# forgetting to hide something fails silently at some consumer's link.
laser-all.o: $(OUR_OBJECTS) $(LIBUSB_OBJECTS) $(LIBUDFREAD_OBJECTS)
	@echo "liblaser: partial link with LD=$(LD), confining with OBJCOPY=$(OBJCOPY)"
	$(LD) -r -o $@ $^
	$(OBJCOPY) $(addprefix --keep-global-symbol=,$(LASER_PUBLIC_SYMBOLS)) $@
	@defined=`$(NM) -g --defined-only $@ | awk '{ print $$NF }'`; \
	escaped=`echo "$$defined" | grep -v '^laser_' || true`; \
	if [ -n "$$escaped" ]; then \
	    echo "liblaser: the following symbols escaped $@:" >&2; \
	    echo "$$escaped" | sed 's/^/    /' >&2; \
	    echo "liblaser: OBJCOPY=$(OBJCOPY) did not confine them - see the" >&2; \
	    echo "liblaser: comment on OBJCOPY at the top of this Makefile." >&2; \
	    exit 1; \
	fi; \
	missing=; \
	for sym in $(LASER_PUBLIC_SYMBOLS); do \
	    echo "$$defined" | grep -qx "$$sym" || missing="$$missing $$sym"; \
	done; \
	if [ -n "$$missing" ]; then \
	    echo "liblaser: declared public but not defined in $@:$$missing" >&2; \
	    echo "liblaser: a renamed entry point that is still listed in" >&2; \
	    echo "liblaser: LASER_PUBLIC_SYMBOLS, or one removed from the sources." >&2; \
	    exit 1; \
	fi

# The checks above are not decoration. Everything that makes this library safe
# to link beside libbluray happens in one objcopy invocation, and the ways it
# can go wrong are all silent at this level and all surface as a link error in
# someone else's build.
#
# BOTH DIRECTIONS ARE ASSERTED, which they were not. The first check catches a
# foreign symbol escaping - a tool that cannot read the object, an
# unrecognised format. The second catches the opposite and equally silent
# failure: objcopy --keep-global-symbol= on a name that does not exist is
# ignored without a word, so renaming an entry point and forgetting this list
# localises the real symbol, passes the first check, and fails at some
# consumer's link two layers away. Both cost one nm per build, already being
# run, and name the problem where it was created.

# A single member, so linking this archive pulls all of it - libusb and
# libudfread included, whether or not the consumer touches disc identification.
# That is the price of the confinement above, and it is the right trade: the
# alternative is per-object granularity with the symbols exposed, and a few
# hundred kilobytes of an archive that is fully used in practice anyway.
# Consumers wanting the dead code back can build with -ffunction-sections
# -fdata-sections and link with --gc-sections.
liblaser.a: laser-all.o
	rm -f $@
	$(AR) rcs $@ $^

# -llog is declared here rather than left to whoever links this. Two things
# in the archive need it and neither is optional: our own registry.c and
# our own sources log through __android_log_print() on Android, and libusb's core.c does the
# same because android/config.h defines USE_SYSTEM_LOGGING_FACILITY. A static
# archive carries no dependency of its own, so a consumer that does not know
# this fails at final link with __android_log_write undefined - a long way
# from anything that names libusb or logcat.
#
# -llog is also the ONLY thing left to declare. libusb and libudfread are
# inside the archive and confined to it, so there is no Requires.private and
# no second -l for a consumer to discover; nothing that was vendored can be
# reached from outside, which is the whole point of the confinement step.
#
# It sits in Libs rather than Libs.private even though this archive is
# static-only, which is where a purist would put it. VLC reaches this file
# through PKG_CHECK_MODULES, which asks pkg-config for --libs and not
# --libs --static, so Libs.private alone is simply dropped - and dropped for
# the one flag without which the link fails. It is repeated in Libs.private
# so that a consumer who does ask for the static view still gets it.
install: liblaser.a
	mkdir -p "$(PREFIX)/include" "$(PREFIX)/lib" "$(PREFIX)/lib/pkgconfig"
	cp "$(SRC_DIR)/laser.h" "$(SRC_DIR)/laser_disc.h" "$(PREFIX)/include/"
	cp liblaser.a "$(PREFIX)/lib/"
	printf 'prefix=%s\nexec_prefix=$${prefix}\nlibdir=$${exec_prefix}/lib\nincludedir=$${prefix}/include\n\nName: liblaser\nDescription: Library for Accessing SCSI External Readers - SCSI-MMC over USB to optical drives\nVersion: %s\nLibs: -L$${libdir} -llaser -llog\nLibs.private: -llog\nCflags: -I$${includedir}\n' "$(PREFIX)" "$(VERSION)" \
		> "$(PREFIX)/lib/pkgconfig/liblaser.pc"

clean:
	rm -f $(OUR_OBJECTS) $(LIBUSB_OBJECTS) $(LIBUDFREAD_OBJECTS) \
	      laser-all.o liblaser.a
