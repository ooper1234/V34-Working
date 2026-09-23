#!/bin/bash
# Builds libsmv34.a from the vendored spanDSP V.34 sources (work in progress),
# including every generated table the sources need. The tables are produced
# at build time by the generators that ship with spanDSP; the meteor filter
# design engine (tools/meteor-engine.c) supplies the pre-emphasis filters.
#
# The daemon does NOT link this library: spanDSP's V.34 is unfinished
# (phases 1-2 run, phase 3 and the data pump are incomplete, see docs/v34.md).
# It exists so the work can be continued and tested with v34_loopback.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
BASE="$HERE/../../.."
SRC="$BASE/third_party/spandsp/src"
TOOLS="$BASE/third_party/spandsp/tools"
OUT="$HERE/v34build"
mkdir -p "$OUT/gen" "$OUT/include"
cd "$OUT"

cat > config.h << 'CFG'
#define HAVE_STDBOOL_H 1
#define HAVE_MATH_H 1
#define HAVE_TGMATH_H 1
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
#include <complex.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
EOF

# The V.34 sources use tgmath; make sure <complex.h> wins over any
# spanDSP header with the same name by not putting $SRC/spandsp on -I.
CFLAGS="-O2 -fPIC -I. -I$OUT/include -I$SRC -DHAVE_CONFIG_H -include prelude.h"

echo "== building generators"
# The V.34 receive tables must be generated with the same rolloff as the
# transmit tables (0.12); upstream's V.34 mode table uses 0.25 for receive
# against 0.12 for transmit, which leaves a mismatched, non-Nyquist cascade
# and heavy ISI in the primary channel. See the patch header.
cp "$SRC/make_modem_filter.c" make_modem_filter.c
patch -p0 -s -N < "$SRC/spandsp-v34-rx-matched-filter.patch" || true
gcc $CFLAGS -o gen/make_modem_filter make_modem_filter.c $SRC/filter_tools.c -lm
gcc $CFLAGS -o gen/make_shell $SRC/make_v34_shell_map.c -lm
gcc $CFLAGS -o gen/make_conv $SRC/make_v34_convolutional_coders.c -lm
cat > spandsp.h << 'EOF'
#include <math.h>
#include <stdlib.h>
#include "spandsp/telephony.h"
#include "spandsp/bit_operations.h"
#include "spandsp/g711.h"
EOF
# This generator uses "I" as a variable, so it must not see <complex.h>.
gcc -O2 -fPIC -I. -I$OUT/include -I$SRC -DHAVE_CONFIG_H -o gen/make_probe \
    $SRC/make_v34_probe_signals.c $SRC/g711.c $SRC/bit_operations.c $SRC/alloc.c -lm
gcc $CFLAGS -I"$TOOLS" -o gen/make_preemph $SRC/make_v34_tx_pre_emphasis_filters.c "$TOOLS/meteor-engine.c" -lm

echo "== generating tables"
gen/make_shell > include/v34_shell_map.h
gen/make_conv > include/v34_convolutional_coders.h
gen/make_probe > include/v34_probe_signals.h
(cd include && ../gen/make_preemph > /dev/null)

# RRC filters: V.22bis ones are included by the V.34 sources, plus the full
# V.34 set for every supported symbol rate and carrier.
gen/make_modem_filter -m V.22bis1200 -r > include/v22bis_rx_1200_rrc.h
gen/make_modem_filter -m V.22bis2400 -r > include/v22bis_rx_2400_rrc.h
gen/make_modem_filter -m V.22bis -t   > include/v22bis_tx_rrc.h
for rate in 2400 2743 2800 3000 3200; do
    gen/make_modem_filter -m V.34_${rate}      -r > include/v34_rx_${rate}_low_carrier_rrc.h
    gen/make_modem_filter -m V.34_${rate}_high -r > include/v34_rx_${rate}_high_carrier_rrc.h
    gen/make_modem_filter -m V.34_${rate}      -t > include/v34_tx_${rate}_rrc.h
done
gen/make_modem_filter -m V.34_3429 -r > include/v34_rx_3429_rrc.h
gen/make_modem_filter -m V.34_3429 -t > include/v34_tx_3429_rrc.h

echo "== patching sources (leftover debug printfs)"
# Always regenerate the working copies.  Reapplying with patch -N can silently
# retain stale code after a patch is edited, and ignoring a malformed patch can
# produce a seemingly successful archive containing the unpatched sources.
cp "$SRC/v34rx.c" "$SRC/v34tx.c" .
patch -p0 -s < "$SRC/spandsp-v34-debug-printfs.patch"
patch -p0 -s < "$SRC/spandsp-v34-phase3.patch"
patch -p0 -s < "$SRC/spandsp-v34-duplex-mp.patch"
patch -p0 -s < "$SRC/spandsp-v34-mp-demod.patch"
patch -p0 -s < "$SRC/spandsp-v34-j-detect.patch"
patch -p0 -s < "$SRC/spandsp-v34-sbar-detect.patch"
patch -p0 -s < "$SRC/spandsp-v34-s-sbar.patch"
patch -p0 -s < "$SRC/spandsp-v34-cc-switch.patch"
patch -p0 -s < "$SRC/spandsp-v34-j-diff.patch"
patch -p0 -s < "$SRC/spandsp-v34-j-length.patch"
patch -p0 -s < "$SRC/spandsp-v34-mp-crc-tx.patch"
patch -p0 -s < "$SRC/spandsp-v34-mp-crc-rx.patch"
patch -p0 -s < "$SRC/spandsp-v34-mp-ack.patch"
patch -p0 -s < "$SRC/spandsp-v34-e-channel.patch"
patch -p0 -s < "$SRC/spandsp-v34-b1.patch"
patch -p0 -s < "$SRC/spandsp-v34-j-gate.patch"
# INFO1c power reduction goes last: its site (top of v34_tx) is untouched by
# the patches above, and composing it against their output avoids the
# context collision the old placement had with s-sbar's hunk 2.
patch -p0 -s < "$SRC/spandsp-v34-power.patch"

echo "== compiling"
rm -f *.o
# The patched copies in this directory are authoritative for v34tx/v34rx.
gcc $CFLAGS -c -o v34tx.o v34tx.c
gcc $CFLAGS -c -o v34rx.o v34rx.c
for f in v34_logging bitstream crc vector_float complex_vector_float; do
    gcc $CFLAGS -c -o "$f.o" "$SRC/$f.c"
done
rm -f libsmv34.a
ar rcs libsmv34.a *.o
echo "built $OUT/libsmv34.a"
