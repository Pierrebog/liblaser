# liblaser - Library for Accessing SCSI External Readers.
#
# Builds liblaser.a, a static archive with a single member exporting exactly
# the public API, and installs it with laser.h, laser_disc.h, laser_version.h
# and liblaser.pc.
#
# Layout:
#   src/                - registry.c / usb.c / bot.c / scsi.c / disc.c and headers
#   contrib/libusb/     - vendored libusb 1.0.30, a complete upstream release
#                         tree. Compiled here rather than through its own
#                         configure; its Android config.h is the one upstream
#                         ships at contrib/libusb/android/config.h for its own
#                         ndk-build, which is why the whole tree is vendored.
#   contrib/libudfread/ - vendored libudfread, used by src/disc.c. udfread.c
#                         only includes config.h behind HAVE_CONFIG_H, which is
#                         never defined here.
#   tools/confine.py    - the symbol confinement step, see below.

project('liblaser', 'c',
    version: '0.2.1',
    license: 'LGPL-2.1-or-later',
    meson_version: '>= 0.60.0',
    # GNU C11: _Static_assert in bot.c, and ##__VA_ARGS__ in LOG_QUIRK().
    default_options: ['c_std=gnu11'])

cc = meson.get_compiler('c')
threads_dep = dependency('threads')

# laser_version.h, from the project version above - the one place it is set.
laser_version = meson.project_version().split('.')
laser_version_conf = configuration_data()
laser_version_conf.set('LASER_VERSION_MAJOR', laser_version[0])
laser_version_conf.set('LASER_VERSION_MINOR', laser_version[1])
laser_version_conf.set('LASER_VERSION_MICRO', laser_version[2])

configure_file(
    input: 'src/laser_version.h.in',
    output: 'laser_version.h',
    configuration: laser_version_conf,
    install: true,
    install_dir: get_option('includedir'))

# The build directory, where laser_version.h is generated, and src/.
laser_inc = include_directories('.', 'src')

# Everything this library promises, and nothing else: the union of laser.h and
# laser_disc.h. A function added to either and forgotten here does not fail to
# build, it fails to link in whoever calls it - confine.py checks both
# directions, but this is the one list to update.
laser_public_symbols = [
    'laser_set_log_cb',
    'laser_parse_token',
    'laser_acquire',
    'laser_release',
    'laser_token_not_ready',
    'laser_scsi_cdb',
    'laser_css_session_begin',
    'laser_css_session_end',
    'laser_read_blocks',
    'laser_read_cd_blocks',
    'laser_status_is_positional',
    'laser_region_mismatch',
    'laser_disc_identify',
]

libusb_inc = include_directories('contrib/libusb/libusb')
libudfread_inc = include_directories('contrib/libudfread/src')

# Vendored libusb, Linux/POSIX backend only. This is upstream's Android list
# (android/jni/libusb.mk). linux_netlink.c is not optional even though nothing
# here enumerates: linux_usbfs.c references linux_netlink_start_event_monitor()
# behind a runtime test, so leaving it out is an undefined symbol at link time.
#
# The config directory is on this target's include path only: it holds a file
# called config.h, and only libusb asks for one.
libusb_internal = static_library('laser-libusb',
    files(
        'contrib/libusb/libusb/core.c',
        'contrib/libusb/libusb/descriptor.c',
        'contrib/libusb/libusb/hotplug.c',
        'contrib/libusb/libusb/io.c',
        'contrib/libusb/libusb/strerror.c',
        'contrib/libusb/libusb/sync.c',
        'contrib/libusb/libusb/os/linux_usbfs.c',
        'contrib/libusb/libusb/os/linux_netlink.c',
        'contrib/libusb/libusb/os/events_posix.c',
        'contrib/libusb/libusb/os/threads_posix.c',
    ),
    include_directories: [
        include_directories('contrib/libusb/android'),
        include_directories('contrib/libusb/libusb/os'),
        libusb_inc,
    ],
    dependencies: threads_dep,
    gnu_symbol_visibility: 'hidden',
    override_options: ['warning_level=0'],
    install: false)

# Vendored libudfread. Not default_blockinput.c (src/disc.c supplies its own
# SCSI-backed block input) and not udfread-version.c (only referenced when
# HAVE_UDFREAD_VERSION_H is defined, by libudfread's own build).
libudfread_internal = static_library('laser-udfread',
    files(
        'contrib/libudfread/src/udfread.c',
        'contrib/libudfread/src/ecma167.c',
    ),
    include_directories: libudfread_inc,
    gnu_symbol_visibility: 'hidden',
    override_options: ['warning_level=0'],
    install: false)

# Our own code, with warnings - the vendored targets above are not ours to keep
# warning-clean. Default visibility: laser_lookup() and laser_bot_send_locked()
# cross between objects and must survive as far as the partial link.
laser_warning_args = cc.get_supported_arguments(
    '-Wshadow', '-Wvla', '-Wswitch-enum')

laser_internal = static_library('laser-own',
    files(
        'src/registry.c',
        'src/usb.c',
        'src/bot.c',
        'src/scsi.c',
        'src/disc.c',
    ),
    include_directories: [laser_inc, libusb_inc, libudfread_inc],
    c_args: laser_warning_args,
    dependencies: threads_dep,
    override_options: ['warning_level=2'],
    install: false)

# SYMBOL CONFINEMENT. libbluray bundles its own libudfread, exporting the same
# udfread_* names, and an Android VLC links both archives - so which copy
# satisfies a reference would depend on archive member ordering. Hence nothing
# but laser_public_symbols leaves this archive: confine.py partially links the
# three internal archives into one relocatable object, demotes every other
# global to local, asserts the result in both directions, and archives it.
#
# The partial link is what makes this possible at all: localizing per object
# would break laser_lookup(), called across objects, and libusb's files call
# into each other throughout.
confine = find_program('tools/confine.py')

liblaser = custom_target('liblaser',
    input: [laser_internal, libusb_internal, libudfread_internal],
    output: 'liblaser.a',
    command: [confine,
        '--output', '@OUTPUT@',
        '--symbols', ','.join(laser_public_symbols),
        '--cc'] + cc.cmd_array() + ['--', '@INPUT@'],
    install: true,
    install_dir: get_option('libdir'))

install_headers('src/laser.h', 'src/laser_disc.h')

# -llog is declared here rather than left to whoever links this archive: our
# own registry.c and libusb's core.c (USE_SYSTEM_LOGGING_FACILITY in its
# Android config.h) both log through the Android logger. It is in Libs rather
# than only Libs.private because VLC asks pkg-config for --libs, not --static.
pkg = import('pkgconfig')
pkg.generate(
    name: 'liblaser',
    filebase: 'liblaser',
    description: 'Library for Accessing SCSI External Readers - SCSI-MMC over USB to optical drives',
    version: meson.project_version(),
    libraries: ['-L${libdir}', '-llaser', '-llog'],
    libraries_private: ['-llog'])

# For use as a meson subproject.
liblaser_dep = declare_dependency(
    link_with: liblaser,
    include_directories: laser_inc)
