#!/bin/bash
# Configure, build and install OCCT for one target. Runs INSIDE the build
# container (see build.sh). Usage: build-occt.sh wasm|native
set -euo pipefail
TARGET="$1"
HERE="$(cd "$(dirname "$0")" && pwd)"
KERNEL="$(cd "$HERE/.." && pwd)"
. "$HERE/occt-config.sh"
SRC="$KERNEL/occt-src"
OUT="$KERNEL/build-$TARGET"
mkdir -p "$OUT/occt-build" "$OUT/occt-install"
JOBS="${JOBS:-$(nproc)}"

case "$TARGET" in
  wasm)
    # -fwasm-exceptions: native WebAssembly exception handling, the analogue of
    # gcc's zero-cost exceptions. OCCT's own -fexceptions alone would select
    # emscripten's JS-emulated exceptions. No -pthread: single-threaded build.
    EXTRA=(-DCMAKE_TOOLCHAIN_FILE="$EMSDK/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake"
           -DCMAKE_CXX_FLAGS="$OCCT_COMMON_CXX_FLAGS -fwasm-exceptions -sSUPPORT_LONGJMP=wasm"
           -DCMAKE_C_FLAGS="$OCCT_COMMON_CXX_FLAGS -fwasm-exceptions -sSUPPORT_LONGJMP=wasm")
    CONFIGURE=(emcmake cmake) ;;
  native)
    EXTRA=(-DCMAKE_CXX_FLAGS="$OCCT_COMMON_CXX_FLAGS" -DCMAKE_C_FLAGS="$OCCT_COMMON_CXX_FLAGS")
    CONFIGURE=(cmake) ;;
  *) echo "usage: $0 wasm|native" >&2; exit 2 ;;
esac

"${CONFIGURE[@]}" -S "$SRC" -B "$OUT/occt-build" -G "Unix Makefiles" \
  -DINSTALL_DIR="$OUT/occt-install" \
  "${OCCT_CMAKE_COMMON[@]}" "${EXTRA[@]}" 2>&1 | tee "$OUT/occt-configure.log"
start=$(date +%s)
if ! make -C "$OUT/occt-build" -j"$JOBS" > "$OUT/occt-make.log" 2>&1; then
  grep -n -E "error" "$OUT/occt-make.log" | head -40; exit 1
fi
make -C "$OUT/occt-build" install > "$OUT/occt-install.log" 2>&1
echo "occt-$TARGET build seconds: $(( $(date +%s) - start ))" | tee "$OUT/occt-build-time.txt"
