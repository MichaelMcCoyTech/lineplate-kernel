#!/bin/bash
# Host entry point. Builds OCCT and/or the kernel binding inside pinned containers.
#   build.sh occt-wasm | occt-native | kernel-wasm | kernel-native | all
# Output stays in the git-ignored kernel/build-wasm/ and kernel/build-native/.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
KERNEL="$(cd "$HERE/.." && pwd)"
# Pinned toolchains (tag and digest).
EMSDK_IMAGE="emscripten/emsdk:6.0.10-x64@sha256:d4d9b9a07e28f8795ebca83b4b3bf98febb65f956e1364ac32483eea3d99bd8f"
NATIVE_IMAGE="cadoc-kernel-native:ubuntu24.04"

# The OCCT source is needed only to build OCCT. The binding builds against an installed OCCT
# (build-<target>/occt-install), which can be a copy of another checkout's.
need_occt_src() {
  if [ ! -d "$KERNEL/occt-src/src" ]; then
    git clone --depth 1 --branch V8.0.1 https://github.com/Open-Cascade-SAS/OCCT.git "$KERNEL/occt-src"
  fi
  test "$(git -C "$KERNEL/occt-src" rev-parse HEAD)" = "b8f597c677811d1f9f4d8a97f5ae2825c0353a42"
  # cadoc's patches to OCCT (patches/*.patch, LGPL-2.1 like OCCT), applied once each in name
  # order: a speed fix, two undefined-behaviour fixes and an exact BinTools round trip;
  # kernel/README.md lists them, patches/UPSTREAM.md has the bug reports.
  for p in "$HERE"/patches/*.patch; do
    [ -e "$p" ] || continue
    git -C "$KERNEL/occt-src" apply --reverse --check "$p" 2>/dev/null && continue
    git -C "$KERNEL/occt-src" apply "$p"
  done
}

run_wasm()   { docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp -e EM_CACHE=/kernel/build-wasm/em-cache -e JOBS -e CADOC_EXTRA_LINK_FLAGS -e KERNEL_VARIANT \
                 -v "$KERNEL":/kernel "$EMSDK_IMAGE" bash -c "$1"; }
run_native() { docker build -q -t "$NATIVE_IMAGE" -f "$HERE/Dockerfile.native" "$HERE" >/dev/null
               docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp -e JOBS -v "$KERNEL":/kernel "$NATIVE_IMAGE" bash -c "$1"; }

case "${1:-all}" in
  occt-wasm)     need_occt_src; mkdir -p "$KERNEL/build-wasm";   run_wasm   "/kernel/build/build-occt.sh wasm" ;;
  occt-native)   need_occt_src; mkdir -p "$KERNEL/build-native"; run_native "/kernel/build/build-occt.sh native" ;;
  kernel-wasm)   run_wasm   "/kernel/build/build-kernel.sh wasm" ;;
  kernel-native) run_native "/kernel/build/build-kernel.sh native" ;;
  all) "$0" occt-wasm; "$0" occt-native; "$0" kernel-wasm; "$0" kernel-native ;;
  *) echo "unknown target $1" >&2; exit 2 ;;
esac
