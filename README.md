# Lineplate kernel: source of the geometry kernel binding

This repository is the source of the geometry kernel that [Lineplate](https://lineplate.com) runs in your browser. It is published so you can exercise your rights under the GNU Lesser General Public License, version 2.1: read the code, rebuild it, change it, and run Lineplate with your own build.

It contains **only** the kernel (`kernel/`), exactly as it is in Lineplate's private repository at the commit each release was built from. The rest of Lineplate (the drawing pipeline, the web app) is proprietary and is not here; it talks to the kernel only through the message format in `kernel/schema/protocol.ts`.

## What it is

- **[Open CASCADE Technology](https://github.com/Open-Cascade-SAS/OCCT) (OCCT) 8.0.1**, tag `V8.0.1`, commit `b8f597c677811d1f9f4d8a97f5ae2825c0353a42`, licensed under the LGPL 2.1 with the Open CASCADE exception (`LICENSE`, `OCCT-LGPL-EXCEPTION.txt`). OCCT reads the CAD files and computes the hidden lines.
- **Four patches to OCCT**, in `kernel/build/patches/`, applied in name order by the build recipe before OCCT is compiled. They are LGPL 2.1 like OCCT. `kernel/README.md` explains each, and `kernel/build/patches/UPSTREAM.md` has the reports drafted for upstream:
  - `0001-shapefix-selfintersect-wire-edge-snapshot.patch`: shape healing reads a wire's edges from a snapshot (faster large imports, identical results);
  - `0002-contap-contour-line-without-tangent.patch`: hidden-line removal no longer reads outside an array when an outline has no tangent;
  - `0003-intwalk-fillpntsinholes-unadded-line.patch`: hidden-line removal no longer reads past a list of outlines;
  - `0004-bintools-exact-round-trip.patch`: OCCT's binary shape format reads back exactly what it wrote.
- **The binding** (`kernel/src/`, `kernel/CMakeLists.txt`): a thin C++ layer that answers JSON requests (import, mesh, hidden-line removal, clearance, shape export) with OCCT, compiled to WebAssembly for the browser and to a native command-line program. It holds no product logic. Licensed under the LGPL 2.1 (`LICENSE`), including its small JSON reader and writer, `kernel/src/json.hpp`.
- **The build recipe** (`kernel/build/`): pinned containers for both builds (Emscripten `emscripten/emsdk:6.0.10` by digest, and Ubuntu 24.04 for native).
- `kernel/reference/` is the Python reference implementation of the same protocol (pythonocc), used to check the binding's answers; `kernel/tools/cache-diag/` is the experiment behind patch 0004. Neither is part of the shipped kernel. Their notes mention paths in the private repository (`pipeline/...`, `spikes/...`); those files are not needed to build or run the kernel.

## Releases

Each tag here matches a kernel release that Lineplate has served, named `kernel-v<version>+occt-<OCCT version>-p<patch count>-<source commit>`. The current one is **`kernel-v1.2.0+occt-8.0.1-p4-5fa9e61`**: the files under that tag are byte for byte the source the shipped `cadoc-kernel.wasm` was built from (commit `5fa9e61` in the private repository). Lineplate's [open-source notices](https://lineplate.com/open-source) name the tag it currently serves, and `https://lineplate.com/api/health` reports it as `kernel.release`.

("cadoc" in file and function names is the product's codename; the files keep it so the published hashes stay valid.)

## Building it

You need Docker and Git, on Linux x86-64 (or WSL2).

```bash
git clone https://github.com/MichaelMcCoyTech/lineplate-kernel
cd lineplate-kernel
git checkout kernel-v1.2.0+occt-8.0.1-p4-5fa9e61
kernel/build/build.sh occt-wasm     # clones OCCT V8.0.1, checks the commit, applies the patches, builds OCCT for WebAssembly
kernel/build/build.sh kernel-wasm   # builds the binding against it
```

The output is `kernel/build-wasm/kernel/cadoc-kernel.mjs` and `cadoc-kernel.wasm`. `kernel/build/build.sh occt-native` and `kernel-native` build the native command-line kernel (`cadoc-kernel-cli`) the same way, and `all` builds everything. OCCT takes the longest: expect tens of minutes for each target on a recent machine. `JOBS=<n>` sets the parallelism. `kernel/README.md` has the exact compiler flags and the details.

## Using your own kernel in Lineplate

Lineplate loads the kernel at run time as two separate files, never bundled into the rest of the app, so you can replace it:

1. Build the kernel as above, changed however you like, as long as it still speaks protocol version 1 (`kernel/schema/protocol.ts`).
2. Serve `cadoc-kernel.mjs` and `cadoc-kernel.wasm` from the same folder over HTTPS (or from `http://localhost`), with the `.wasm` as `Content-Type: application/wasm` and CORS headers that allow `https://lineplate.com`. For example: `npx http-server kernel/build-wasm/kernel --cors -p 8080`.
3. In the Lineplate studio, open **Kernel** in the top bar, choose **Use a custom kernel URL**, and enter the address of your `cadoc-kernel.mjs` (e.g. `http://localhost:8080/cadoc-kernel.mjs`). The setting is kept in your browser only. **Reset to built-in** goes back to ours.

Your CAD files still never leave your browser: the kernel runs in a Web Worker in your tab, whichever build you use.

## Licence

The binding, the build recipe and the patches: GNU Lesser General Public License, version 2.1 (`LICENSE`). OCCT: LGPL 2.1 with the Open CASCADE exception, version 1.0 (`OCCT-LGPL-EXCEPTION.txt`). Lineplate's terms allow you to modify the kernel for your own use and to reverse-engineer the application as far as needed to debug such modifications.

Questions about the source: hello@lineplate.com. Lineplate is made by Galaxy Technology.
