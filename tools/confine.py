#!/usr/bin/env python3
# confine.py: build liblaser.a with nothing but the public API exported.
#
# Usage:
#   confine.py --output liblaser.a --symbols a,b,c --cc <compiler...> -- <archives...>
#
# 1. Partially link every member of the input archives into one relocatable
#    object, through the compiler driver (-r), so that every reference between
#    them is resolved.
# 2. Demote every global symbol except --symbols to local, with objcopy.
# 3. Check both directions: no foreign symbol escaped, and every declared
#    entry point is defined - objcopy --keep-global-symbol= on a name that does
#    not exist is ignored without a word.
# 4. Archive the object as the single member of the output.
#
# The tools are taken from OBJCOPY, NM and AR in the environment when set,
# otherwise asked of the compiler (-print-prog-name), preferring the LLVM
# ones, which read every target's objects; otherwise from PATH. A build-machine
# GNU objcopy cannot always read a cross-compiled object, and a failure there
# would not stop the build on its own - the checks in step 3 are what turn it
# into an error.

import os
import shutil
import subprocess
import sys
import tempfile


def fail(msg):
    sys.stderr.write('liblaser: %s\n' % msg)
    sys.exit(1)


def parse_args(argv):
    output = None
    symbols = None
    cc = []
    inputs = []

    i = 0
    while i < len(argv):
        arg = argv[i]
        if arg == '--output' and i + 1 < len(argv):
            output = argv[i + 1]
            i += 2
        elif arg == '--symbols' and i + 1 < len(argv):
            symbols = [s for s in argv[i + 1].split(',') if s]
            i += 2
        elif arg == '--cc':
            try:
                sep = argv.index('--', i + 1)
            except ValueError:
                fail('confine.py: --cc must be followed by "--" and the inputs')
            cc = argv[i + 1:sep]
            inputs = argv[sep + 1:]
            break
        else:
            fail('confine.py: unexpected argument %r' % arg)

    if not output or symbols is None or not cc or not inputs:
        fail('usage: confine.py --output OUT --symbols A,B --cc CC... -- INPUTS...')

    return output, symbols, cc, inputs


def find_tool(env_var, names, cc):
    value = os.environ.get(env_var)
    if value:
        return value

    # -print-prog-name answers the bare name when it knows no better, so only
    # an absolute path that exists counts as found.
    for name in names:
        try:
            out = subprocess.run(cc + ['-print-prog-name=' + name],
                                 capture_output=True, text=True).stdout.strip()
        except OSError:
            out = ''
        if os.path.isabs(out) and os.path.exists(out):
            return out

    for name in names:
        path = shutil.which(name)
        if path:
            return path

    fail('cannot find any of %s; set %s' % (', '.join(names), env_var))


def run(cmd):
    result = subprocess.run(cmd)
    if result.returncode != 0:
        fail('command failed: %s' % ' '.join(cmd))


def defined_globals(nm, obj):
    out = subprocess.run([nm, '-g', '--defined-only', obj],
                         capture_output=True, text=True)
    if out.returncode != 0:
        fail('%s could not read %s:\n%s' % (nm, obj, out.stderr))
    return {line.split()[-1] for line in out.stdout.splitlines() if line.strip()}


def main():
    output, symbols, cc, inputs = parse_args(sys.argv[1:])

    objcopy = find_tool('OBJCOPY', ['llvm-objcopy', 'objcopy'], cc)
    nm = find_tool('NM', ['llvm-nm', 'nm'], cc)
    ar = find_tool('AR', ['llvm-ar', 'ar'], cc)

    print('liblaser: confining with OBJCOPY=%s NM=%s AR=%s' % (objcopy, nm, ar))

    with tempfile.TemporaryDirectory() as tmp:
        # The member name the archive will carry.
        obj = os.path.join(tmp, 'laser-all.o')

        run(cc + ['-r', '-nostdlib', '-o', obj,
                  '-Wl,--whole-archive'] + inputs + ['-Wl,--no-whole-archive'])

        run([objcopy] + ['--keep-global-symbol=' + s for s in symbols] + [obj])

        defined = defined_globals(nm, obj)

        escaped = sorted(s for s in defined if not s.startswith('laser_'))
        if escaped:
            fail('the following symbols escaped confinement:\n    %s\n'
                 'liblaser: %s did not confine them'
                 % ('\n    '.join(escaped), objcopy))

        missing = [s for s in symbols if s not in defined]
        if missing:
            fail('declared public but not defined: %s\n'
                 'liblaser: a renamed entry point still listed in '
                 'laser_public_symbols, or one removed from the sources'
                 % ' '.join(missing))

        # Written to a temporary name first: a failed run must not leave an
        # archive behind that looks up to date.
        tmp_out = output + '.tmp'
        if os.path.exists(tmp_out):
            os.remove(tmp_out)
        run([ar, 'rcs', tmp_out, obj])
        os.replace(tmp_out, output)


if __name__ == '__main__':
    main()
