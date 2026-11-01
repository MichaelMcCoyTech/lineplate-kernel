# cadoc kernel

The kernel is a thin C++ binding over [Open CASCADE Technology](https://github.com/Open-Cascade-SAS/OCCT) (OCCT) 8.0.1, with four patches of ours to OCCT (`build/patches/`, listed under "Building"). It answers the messages defined in `schema/protocol.ts`, and does nothing else.

One source tree builds two ways:

| Build | Output | Where it runs |
|---|---|---|
| WASM (emscripten) | `cadoc-kernel.mjs` + `cadoc-kernel.wasm` | A browser Web Worker (local mode), or Node |
| Native (gcc) | `cadoc-kernel-cli` | A job subprocess (cloud mode); benchmarks |

The pipeline can't tell which build it's talking to.

## Layout

| Path | What |
|---|---|
| `schema/protocol.ts` | The kernel↔pipeline protocol, and the only interface to the rest of cadoc |
| `src/kernel.cpp` | Dispatch: one request in, one response or `KernelError` out |
| `src/import.cpp` | `import`: STEP, IGES or BRep to the assembly tree and shape properties |
| `src/convert.cpp` | `stlToStep`: an STL mesh to a faceted STEP file |
| `src/mesh.cpp` | `mesh`: BRepMesh triangulations |
| `src/hlr.cpp`, `src/curves.cpp` | `hlr`: exact or polygonal hidden-line removal, orthographic or perspective, targets and per-target occluders; HLR edges to `Curve2D` (conics, recognised lines/arcs/ellipses within 1 µm, Béziers, polylines) |
| `src/spatial.cpp` | `clearance` and `sweep` |
| `src/shapes.cpp` | `exportShapes` / `loadShapes`: shapes as OCCT binary B-rep (BinTools format version 5, patch 0004), so a fresh kernel instance can run one HLR batch without re-reading the STEP file (large views, `pipeline/large`). `importCache` / `loadCache`: the whole imported document as one blob, so a file is imported once (`pipeline/kernel/import-cache.ts`). Not in the reference kernel |
| `src/session.cpp` | The current document, placements, meshes, and the typed-array wire forms |
| `src/json.hpp` | A minimal JSON parser and writer |
| `src/kernel_wasm.cpp` | WASM entry points, as a plain C ABI (`cadoc_handle`, `cadoc_binary_ptr`, …) |
| `src/kernel_cli.cpp` | Native entry point: `--stdio` (the protocol over JSON lines), or a benchmark replay of a request file |

## Behaviour: the same as the reference kernel

The binding implements all of protocol v1 with the behaviour of the Python reference kernel (`reference/refkernel.py`, documented in `reference/README.md`): the same instance ids and shape ids, names, colours, units and warnings, the same edge order, the same curve kinds, and the same clearance and sweep semantics. `pipeline/kernel/parity.test.ts` runs the same requests through all three (reference, native CLI, WASM in Node) on the NIST corpus, CAx-IF as1 and the leader arm. The differences that remain:

- **IGES** is read since kernel 1.3 (OCCT's TKDEIGES). OCCT has no IGES stream reader, so the bytes go through a temporary file: emscripten's in-memory filesystem in the WASM build, `/tmp` natively. Before 1.3 IGES was answered `unsupported-format`.
- **`stlToStep`** (since 1.3, not in the reference kernel): an STL mesh to a faceted STEP file (`src/convert.cpp`, OCCT's TKDESTL reads the mesh). Each triangle a planar face on shared edges, coplanar faces merged, each closed shell a solid, millimetres.
- **Speed.** The C++ kernels are faster (as1 import: 0.2 s native against 2.6 s for the reference).
- **`occludersByTarget` (exact HLR) runs once for all targets, not once per target.** Every instance any target needs is loaded once; each target's edges are hidden by its own faces (`Hide(i)`) and by its listed occluders (`Hide(i, j)`), the calls OCCT's `Hide()` makes, restricted to those pairs. A target's edges are never touched by another target's calls. The reference kernel runs one HLR per target, which re-loads shared occluders for every target and also hides occluders' own edges. The parity suite (as1, three targets, one with no entry) gives the same counts, curve kinds and lengths. Polygonal HLR keeps one run per target.
- **`exportShapes` / `loadShapes`** are extensions for large views (`schema/protocol.ts`, "shape cache"). On `--stdio`, `loadShapes` takes its bytes as `"path"`, like `import`.
- **`hlr` takes `instanceFaces`** (`schema/protocol.ts`): an instance of the request is loaded as only the listed faces. Large views pass, per instance of a batch, the faces whose projection reaches the regions where its targets may be visible; inside those regions the visible lines are the same (a face that does not reach them can neither be drawn there nor hide anything there) and the pipeline clips each target's lines to its regions. On Voron 0.2 with the enclosure panels hidden, a batch then loads about a third of the faces it would otherwise (`spikes/perf-large/RESULT.md`). The curve tolerance is still derived from the whole instances, so it does not depend on which faces a batch needed.
- **`importCache` / `loadCache`** are the import cache (`schema/protocol.ts`, "import cache"). `loadCache` restores the kernel as the import left it and answers with the import's own ImportResult. The shapes come back through OCCT's binary B-rep, and with patch 0004 that round trip is exact: every direction, axis frame and location reads back bit for bit. `pipeline/kernel/import-cache.test.ts` holds both builds to identical answers, every number bit for bit, for exact HLR, mesh, clearance and sweep, on every parity file and a synthetic BRep with a location; there is no tolerance and no exception list. Without the patch, reading re-normalised directions and rebuilt frames and locations: HLR from a cache differed in the last bits on 9 of the 20 parity files, and one 17 mm edge of NIST STC 08 flipped from hidden to visible (`tools/cache-diag/`). On `--stdio`, `importCache` can take `"path"` (the blob is written there) and `loadCache` takes its bytes as `"path"`, like `import`.
- **Import properties run on threads natively.** After the tree walk, each distinct shape's counts, optimal bounding box and mass properties are computed on up to 8 threads (`CADOC_IMPORT_THREADS` sets the number; the WASM build has one). The results and their order are unchanged: each shape's computation is independent and deterministic.
- **`version()`** names the build (`wasm` or `native`), an import revision (`import rN`, bumped whenever an import's output or the cache's round trip could change, so an import cache keyed by it is never read by a kernel that would restore it differently) and an HLR revision (`hlr rN`, bumped whenever exact HLR of the same shapes could answer differently, for caches of HLR results). Built against an OCCT without patch 0004, it adds `inexact cache`. `cadoc-kernel-cli --version` prints it, e.g. `cadoc-kernel 1 (protocol v1; import r3; hlr r2; native); OCCT 8.0.1`.
- **`CADOC_TRACE`** (native, diagnosis only): exact HLR with occluders prints each own or pair hide call over 300 ms to stderr.

## Transports

- **Native, `cadoc-kernel-cli --stdio`:** the reference kernel's JSON-lines wire format exactly (`reference/README.md`): one `Envelope` per line each way; typed arrays as `{"$typed": kind, "base64": …}`; non-finite numbers as `{"$float": "Infinity"}`; an import's bytes as `$typed` base64, a plain array, or `"path"`. stdout carries protocol lines only. `pipeline/kernel/native-client.ts` drives it with `NativeKernel.start({ command: nativeCliPath(), args: ["--stdio"] })`.
- **WASM, `_cadoc_handle(requestJson, bytesPtr, bytesLen)`:** the request JSON without the import's bytes, which are copied into the heap and passed beside it. The response is a NUL-terminated JSON string. **Binary responses:** each typed array is `{"$typed": kind, "offset": o, "byteLength": n}`, meaning the `n` little-endian bytes at `_cadoc_binary_ptr() + o` (`o` a multiple of 8; `_cadoc_binary_len()` is the section's size). Both are valid until the next call. The JS side (`pipeline/kernel/wasm-client.ts`, shared by the browser worker and Node) copies each array out of the heap once with `HEAPU8.slice` into its own `ArrayBuffer`, which the worker then transfers to the page without another copy. No base64 anywhere on this path. A WebAssembly trap (`RuntimeError`) leaves the module unusable; the studio replaces the worker.
| `CMakeLists.txt` | Builds the binding against an installed OCCT |
| `build/` | The complete build recipe: pinned container images, OCCT CMake options, scripts |

Kept out of git (`.gitignore`): `occt-src/` (the OCCT checkout), and `build-wasm/` and `build-native/` (build trees). Also kept out: every `.wasm` file.

## Building

You need Docker, and nothing else on the host.

```bash
kernel/build/build.sh all            # or: occt-wasm | occt-native | kernel-wasm | kernel-native
```

1. `build.sh` clones OCCT at tag `V8.0.1` (commit `b8f597c6`) into `occt-src/` if it's missing, and checks the commit.
2. It builds OCCT and the binding inside two pinned images:
   - `emscripten/emsdk:6.0.10-x64` (pinned by digest);
   - `ubuntu:24.04` with apt's gcc 13.3 (also pinned by digest).
3. Both builds use the same OCCT CMake options (`build/occt-config.sh`): Release, `-O3 -DNDEBUG`, static, and single-threaded, with no TBB, FreeType, Tcl/Tk, OpenGL or Draw.
   - `build.sh` applies cadoc's four patches to the OCCT source first (`build/patches/*.patch`, LGPL-2.1 like OCCT, each applied once, in name order). Upstream bug reports for all four are drafted in `build/patches/UPSTREAM.md` (not submitted). One is a speed fix that leaves every result unchanged:
     - `0001-shapefix-selfintersect-wire-edge-snapshot.patch`: `ShapeFix_IntersectionTool::FixSelfIntersectWire` read a wire's edges by index from a linked list inside a double loop, which is cubic in the wire's edge count. On `voron-0.2` it was 40% of the STEP import. The patch reads the edges from a snapshot that is re-taken whenever the wire may have changed. The imported shapes are byte-identical (`spikes/perf-large/RESULT.md`).
   - Two are fixes of undefined behaviour in the outline (contour) walk of exact HLR, which segfaulted the native build on Voron parts seen with the enclosure panels hidden, and read arbitrary memory in WASM (which does not trap on a null read), so the two builds could disagree. Results change only where the old code read outside its arrays:
     - `0002-contap-contour-line-without-tangent.patch`: `Contap_Contour::Perform` used index -1 when a walked line had no recorded tangent; it now takes the tangent from the line's first two points.
     - `0003-intwalk-fillpntsinholes-unadded-line.patch`: `IntWalk_IWalking::FillPntsInHoles` looked up an "alone" point's line by a number that can name the line still being walked (never added), which read past the list and corrupted its cursor; such points are skipped, as `Arrive()` already skips them.
   - One makes OCCT's binary B-rep round trip exact, which the import cache relies on:
     - `0004-bintools-exact-round-trip.patch`: BinTools' readers normalised every direction again (`gp_Dir::SetCoord`) and rebuilt every axis frame from its main and X directions, which moves last bits; they now keep the written components when they are already a unit vector, or what the rebuilt frame holds, to 1e-12 (anything else is normalised as before). A new BinTools format version 5 writes each location exactly (its form, scale factor, matrix and translation; version 4 writes the scaled matrix and reads it back through `gp_Trsf::SetValues`, which re-derives the scale and re-orthogonalises). OCCT's own default format stays version 4; the kernel writes version 5. Found and checked with `tools/cache-diag/`.
   - A tree that was built before a patch existed is patched and rebuilt incrementally (one file: about 20 s native, 40 s WASM).
4. The only flags that differ are the emscripten ones the WASM target requires (`-fwasm-exceptions`, `-sSUPPORT_LONGJMP=wasm`), plus the WASM link settings.

`spikes/s1-wasm/RESULT.md` records the exact flags, the timings and the binary sizes.

### Releasing: the pinned artefact

Builds are never committed. A kernel ships as a **GitHub Release** of this repo, and the product pins one in `kernel.lock.json` at the repo root, by tag and by the sha256 of every file.

- **Tag:** `kernel-v<major.minor.patch>+occt-<OCCT version>-p<patches>-<source commit>`, e.g. `kernel-v1.1.0+occt-8.0.1-p4-<sha>`: OCCT 8.0.1 with the four patches in `build/patches/`. (1.0.0, `kernel-v1.0.0+occt-8.0.1-da69a78`, predates the patches and has no `-p`.) The major version is `PROTOCOL_VERSION`. A test (`pipeline/kernel/lock.test.ts`) holds the lock to that format, and the patch count to the patches in `build/patches/`.
- **Files:**
  - `cadoc-kernel.mjs` and `cadoc-kernel.wasm`;
  - `cadoc-kernel-cli-linux-x64` (glibc 2.39, Ubuntu 24.04);
  - `cadoc-kernel-src.tar.gz`: `git archive` of `kernel/` at the source commit. It includes `build/patches/`, so the changes we made to OCCT (LGPL-2.1 §2) ship with every kernel built from them;
  - `parity.json`;
  - `SHA256SUMS`.
- **Release notes:** the source commit, the toolchain digests (and the native image's actual compiler version), the OCCT tag and commit, every OCCT patch applied and what it changes, and the parity results for exactly these binaries.

To cut a release:

1. Build both targets from the commit being released. The binding builds against an installed OCCT, so `build-<target>/occt-install` can be copied from another checkout built with the same `occt-config.sh`.
2. Run `npx vitest run --no-file-parallelism pipeline/kernel/parity.test.ts` with `CADOC_PARITY_OUT`. It must run every test with none skipped: it skips silently when Python, the CLI or the WASM is missing.
3. Create the release: `gh release create <tag> --target <full source sha>` with the files above.
4. Update `kernel.lock.json`, then fetch the release back with `node scripts/fetch-kernel.mjs` and check it is byte-identical to what was tested.
5. Run the prod e2e (`npm run test:e2e:prod -w apps/studio`), which ships the locked release.

**Using it:**

- `node scripts/fetch-kernel.mjs` downloads the locked release into `kernel/release/` (git-ignored) and refuses any file whose sha256 differs. Use `--web` for only the WASM pair, or `--verify` to check without network. It uses `GITHUB_TOKEN`/`GH_TOKEN` with the REST API, or the `gh` CLI.
- `apps/studio/scripts/sync-kernel.mjs` ships `kernel/release/` by default, checking it against the lock again.
- `CADOC_KERNEL_BUILD=<dir>` overrides that with a local build, for example `kernel/build-wasm/kernel`.
- The parity suite and the native client still default to the local build trees (`CADOC_WASM_KERNEL`, `CADOC_NATIVE_KERNEL` override them). Point them at `kernel/release/` to test the released files.

## Licensing: the LGPL boundary

OCCT is licensed under LGPL-2.1 with the Open CASCADE exception. Sending a `.wasm` file to a browser is distribution, so LGPL §6 applies in full. The architecture meets it in three ways.

1. **This directory is the whole boundary.**
   - Every line of code that touches OCCT lives in `kernel/`.
   - The binding contains no product logic. It reads files, computes properties and runs hidden-line removal. It returns raw results.
   - The proprietary pipeline (`pipeline/`, TypeScript) is built separately: edge chaining, curve fitting, layout, explode, matching, export. It talks to the kernel only through the messages in `schema/protocol.ts`.
   - No proprietary code is compiled or linked into the `.wasm`.
2. **The source is published.**
   - `kernel/` is publishable on its own, under LGPL-2.1, as a public repository. That includes this README, the binding source, `CMakeLists.txt` and the full build recipe in `build/`.
   - With it, anyone can rebuild a kernel that is equivalent at the binary level, from OCCT's own published source and our patches to it (`build/patches/`, applied by `build.sh`; also in every release's source tarball).
   - Each released kernel artefact names the source commit it was built from.
3. **The `.wasm` is replaceable.**
   - The kernel is loaded at runtime from its own URL, as the separate files `cadoc-kernel.mjs` and `cadoc-kernel.wasm`. It is never bundled into the application's JavaScript.
   - The studio loads the kernel from `CADOC_KERNEL_URL`, set when the studio is built. By default that is the copy `apps/studio/scripts/sync-kernel.mjs` puts in `public/kernel/` under a content-hashed name (`cadoc-kernel.<hash>.mjs` and `.wasm`).
   - A user can point the studio at a kernel they built themselves, for example with a modified OCCT: in the studio, **Kernel** in the top bar, **Use a custom kernel URL** (kept in that browser, with a reset). The `.wasm` must sit beside the `.mjs` with the same name, be served as `application/wasm`, and allow the studio's origin by CORS if it is elsewhere.
   - The protocol is versioned (`PROTOCOL_VERSION`), and that version is the compatibility contract for a replacement kernel.
   - The native CLI is the same: it is a separate executable, invoked as a subprocess. It is never linked into proprietary code.

We use **only** LGPL OCCT. Open Cascade's **OCCT-Light** is AGPL-3.0 and must not be used.

This is our reading of how the licence applies. It has not been reviewed by a lawyer; get it reviewed before the first public release.
