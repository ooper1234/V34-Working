#!/bin/bash
# Builds the spanDSP reference V.22bis library with ref_-prefixed symbols,
# used by v22bis_cross for cross-implementation interop testing.
# Requires the spanDSP source tree (default /tmp/opencode/spandsp-master).
set -e
SRC="${1:-/tmp/opencode/spandsp-master}"
OUT="$(dirname "$0")/refbuild"
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
CFG
CFLAGS="-O2 -fPIC -I. -I$SRC/src -I$SRC -DHAVE_CONFIG_H"
gcc $CFLAGS -c -o v22bis_tx_ref.o "$SRC/src/v22bis_tx.c"
gcc $CFLAGS -c -o v22bis_rx_ref.o "$SRC/src/v22bis_rx.c"
for f in logging alloc power_meter dds_float complex_vector_float vector_float complex_filters bit_operations async math_fixed; do
  gcc $CFLAGS -c -o "$f.o" "$SRC/src/$f.c"
done
nm -g --defined-only v22bis_tx_ref.o v22bis_rx_ref.o | awk '{print $3}' | grep -E "^v22bis_" | sort -u | awk '{print $1" ref_"$1}' > rename.syms
objcopy --redefine-syms=rename.syms v22bis_tx_ref.o v22bis_tx_ren.o
objcopy --redefine-syms=rename.syms v22bis_rx_ref.o v22bis_rx_ren.o
rm -f librefspandsp.a
ar rcs librefspandsp.a v22bis_tx_ren.o v22bis_rx_ren.o logging.o alloc.o power_meter.o dds_float.o complex_vector_float.o vector_float.o complex_filters.o bit_operations.o async.o math_fixed.o
echo "built $OUT/librefspandsp.a"
