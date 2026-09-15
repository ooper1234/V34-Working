#!/bin/bash
# Builds a static libsmv8.a from the vendored spanDSP source tree (V.8 and
# its dependencies only). The symbols do not collide with our own V.22bis
# implementation, so the daemon can link both. Pass an alternate spanDSP
# source directory as $1 to build against a different tree.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="${1:-$HERE/../../../third_party/spandsp/src}"
OUT="$HERE/v8build"
mkdir -p "$OUT"
cd "$OUT"
cat > config.h << 'CFG'
#define HAVE_STDBOOL_H 1
#define HAVE_MATH_H 1
#define HAVE_ACOSF 1
#define HAVE_ASINF 1
#define HAVE_ATAN2F 1
#define HAVE_ATANF 1
#define HAVE_CEILF 1
#define HAVE_COSF 1
#define HAVE_EXPF 1
#define HAVE_FLOORF 1
#define HAVE_LOG10F 1
#define HAVE_LOGF 1
#define HAVE_POWF 1
#define HAVE_SINF 1
#define HAVE_TANF 1
#define PACKAGE "spandsp"
#define VERSION "3.1.1"
CFG
cat > prelude.h << 'EOF'
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "spandsp/telephony.h"
#include "spandsp/logging.h"
#include "spandsp/complex.h"
#include "spandsp/async.h"
#include "spandsp/dds.h"
#include "spandsp/power_meter.h"
#include "spandsp/fsk.h"
#include "spandsp/queue.h"
#include "spandsp/tone_generate.h"
#include "spandsp/super_tone_rx.h"
#include "spandsp/modem_connect_tones.h"
#include "spandsp/v8.h"
EOF
CFLAGS="-O2 -fPIC -I. -I$SRC -DHAVE_CONFIG_H"
for f in logging alloc power_meter dds_float dds_int vector_int bit_operations \
         fsk queue tone_generate tone_detect super_tone_rx modem_connect_tones; do
    gcc $CFLAGS -include prelude.h -c -o "$f.o" "$SRC/$f.c"
done
# v8.c needs a small patch before it is compiled: see the .patch file header.
cp "$SRC/v8.c" v8.c
if patch -p0 -s -N v8.c < "$HERE/spandsp-jm-modulations.patch"; then
    echo "applied spandsp-jm-modulations.patch"
fi
gcc $CFLAGS -include prelude.h -c -o v8.o v8.c
rm -f libsmv8.a
ar rcs libsmv8.a *.o
echo "built $OUT/libsmv8.a"
