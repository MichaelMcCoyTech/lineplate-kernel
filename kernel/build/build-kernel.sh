#!/bin/bash
# Build the binding against the OCCT built by build-occt.sh. Runs INSIDE the
# build container (see build.sh). Usage: build-kernel.sh wasm|native
set -euo pipefail
TARGET="$1"
HERE="$(cd "$(dirname "$0")" && pwd)"
KERNEL="$(cd "$HERE/.." && pwd)"
OUT="$KERNEL/build-$TARGET"
# KERNEL_VARIANT: a separate tree for experimental link flags (e.g. kernel-mimalloc).
KDIR="$OUT/${KERNEL_VARIANT:-kernel}"
COMMON=(-DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG"
        -DOpenCASCADE_DIR="$OUT/occt-install/lib/cmake/opencascade")
case "$TARGET" in
  wasm)
    # -O3 is repeated at link time so binaryen's wasm-opt runs at -O3.
    emcmake cmake -S "$KERNEL" -B "$KDIR" "${COMMON[@]}" \
      -DCMAKE_CXX_FLAGS="-UOCC_CONVERT_SIGNALS -fwasm-exceptions -sSUPPORT_LONGJMP=wasm" \
      -DCMAKE_EXE_LINKER_FLAGS_RELEASE="-O3" \
      -DCADOC_EXTRA_LINK_FLAGS="${CADOC_EXTRA_LINK_FLAGS:-}" ;;
  native)
    cmake -S "$KERNEL" -B "$KDIR" "${COMMON[@]}" -DCMAKE_CXX_FLAGS="-UOCC_CONVERT_SIGNALS" ;;
esac
make -C "$KDIR" -j"${JOBS:-$(nproc)}" VERBOSE=1 > "$KDIR.make.log" 2>&1 || { grep -n -E "error" "$KDIR.make.log" | head -40; exit 1; }
ls -la "$KDIR" | grep -E "cadoc-kernel"
