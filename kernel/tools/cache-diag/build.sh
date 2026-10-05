#!/bin/bash
# Builds cadoc-cache-diag in the kernel's pinned native image against an installed OCCT.
#   [OCCT_KERNEL_DIR=<a checkout's kernel/ with build-native/occt-install>] kernel/tools/cache-diag/build.sh [OUT] [cmake options]
# OCCT_KERNEL_DIR defaults to this checkout's kernel/.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
OCCT_KERNEL_DIR="${OCCT_KERNEL_DIR:-$REPO/kernel}"
OUT="${1:-build}"; shift || true
docker build -q -t cadoc-kernel-native:ubuntu24.04 -f "$REPO/kernel/build/Dockerfile.native" "$REPO/kernel/build" >/dev/null
# The OCCT install's CMake files name /kernel/build-native/occt-install: mount that checkout's kernel/ at /kernel.
# The bin5 variant needs OCCT patch 0004 (BinTools format version 5).
EXTRA=""
grep -q VERSION_5 "$OCCT_KERNEL_DIR/build-native/occt-install/include/opencascade/BinTools_FormatVersion.hxx" 2>/dev/null && EXTRA="-DCADOC_HAVE_BIN5=ON"
docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp -v "$OCCT_KERNEL_DIR":/kernel:ro -v "$REPO":/repo cadoc-kernel-native:ubuntu24.04 bash -c "
  cmake -S /repo/kernel/tools/cache-diag -B /repo/kernel/tools/cache-diag/$OUT -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE='-O3 -DNDEBUG' \
    -DCMAKE_CXX_FLAGS=-UOCC_CONVERT_SIGNALS -DOpenCASCADE_DIR=/kernel/build-native/occt-install/lib/cmake/opencascade $EXTRA $* >/dev/null &&
  make -C /repo/kernel/tools/cache-diag/$OUT -j\$(nproc) 2>&1 | grep -E 'error|warning: unused|Built target' || true"
ls -la "$HERE/$OUT/cadoc-cache-diag"
