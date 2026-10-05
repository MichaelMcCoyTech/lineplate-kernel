# Reference native kernel

`refkernel.py` implements kernel protocol v1 (`kernel/schema/protocol.ts`) in Python, on OCCT 8.0.1 through the OCP bindings (`cadquery-ocp==8.0.1.0.0`, see `requirements-spike.txt`). It's spike infrastructure. The pipeline spikes (vector clean-up, auto-explode, part matching) use it until the C++ WASM and native kernels from spike S1 exist. Like everything in `kernel/`, it's a thin OCCT binding with no product logic.

## Running it

```bash
.venv/bin/python kernel/reference/refkernel.py   # reads requests on stdin, answers on stdout
```

From TypeScript, use `pipeline/kernel/native-client.ts`:

```ts
const k = NativeKernel.start();                       // spawns the process
const imp = await k.importFile(readFileSync("as1.stp"));
const hlr = await k.hlr({ view, instances, algorithm: "exact", includeHidden: true, includeSmooth: false });
await k.close();
```

Environment variables used by the client:

- `CADOC_REFKERNEL`: path to `refkernel.py`. The default is the repo copy.
- `CADOC_PYTHON`: a Python that has OCP installed. The default is `<repo>/.venv/bin/python`. From a git worktree, the default falls back to the main checkout's `.venv`.
- `CADOC_CORPUS`: the corpus root that the tests use. The default is `/home/michael/repos/cadoc/corpus/external`. When the files are missing, the tests skip.

Startup takes about 3.5 s, almost all of it loading OCP. Start one process and keep it.

## Wire format

The kernel uses JSON lines, one message per line, in UTF-8.

- **Requests** arrive on stdin as `{"id": <number>, "body": <KernelRequest>}`.
- **Responses** go to stdout as `{"id": <same id>, "body": <KernelResult | KernelError>}`.
- **Order.** The kernel handles one request at a time, in arrival order. A client may write several requests ahead. Responses come back in the same order.
- **stdout carries protocol lines only.** At startup the process duplicates the real stdout for the protocol and points fd 1 at stderr. OCCT's own console output (for example, the STEP reader's `***` banners) therefore ends up on stderr with the kernel's logs.

The protocol is plain JSON, with two encodings that JSON can't express directly. They are the same in both directions:

| Value | On the wire |
|---|---|
| A typed array (`Float32Array`, `Uint32Array`, `Uint8Array`; also `Float64Array`, `Int32Array`) | `{"$typed": "Float32Array", "base64": "<the raw little-endian bytes, base64>"}` |
| A non-finite number (`Infinity`, `-Infinity`, `NaN`) | `{"$float": "Infinity"}` (or `"-Infinity"`, `"NaN"`) |

A typed array is always encoded this way, never as a plain array. Two exceptions on the kernel side: `import.bytes` is also accepted as a plain array of numbers, and the client sends Node `Buffer`s as `Uint8Array`.

**Import by path (native transport only).** Instead of `bytes`, an `import` request may carry `"path": "/absolute/file.step"`, and the kernel reads that file itself. It isn't part of protocol v1: the WASM kernel has no file system. The client exposes it as `importPath()`, and the studio's dev-kernel route uses it, so a large upload never crosses the pipe as base64. A missing file is `read-failed`. `importFile()` still sends bytes; it streams the base64 to the pipe in 3 MB chunks and never calls a Node `Buffer`'s `toJSON()` (which built a plain array of every byte and made a 179 MB import fail). In TypeScript, decoding is a `JSON.parse` reviver (`wireReviver` in the client). It copies each base64 payload into a fresh `ArrayBuffer`, so the typed array is always aligned. In practice the only non-finite value is `SweepResult.freeDistance = Infinity`.

## Errors

Every failure is answered as a `KernelError`, and the process keeps serving.

| Code | When |
|---|---|
| `bad-request` | Malformed JSON or envelope, an unknown `type`, an unknown `shapeId`, an invalid transform or view, a duplicate `instanceId`, or an `hlr` request that includes tessellated-only geometry (below) |
| `unsupported-format` | `format` is not `step`, `iges` or `brep` |
| `read-failed` | The reader rejected the file, the transfer failed, or the file contains no shapes |
| `out-of-memory` | Python `MemoryError` |
| `timeout` | Not produced by the kernel itself. The client uses it for its own per-request timeout (`timeoutMs`), and in that case kills the process. |
| `crash` | An unexpected exception, such as an OCCT `Standard_Failure`. The client also uses this code when the process exits or writes unparseable output. |

A segfault inside OCCT kills the process. The client then rejects every pending request with `crash` and marks itself dead (`alive === false`). Call `NativeKernel.start()` again to recover. `faulthandler` writes the Python stack to stderr.

## What each request does

### `import`

- **Readers.** STEP is read with `STEPCAFControl_Reader` into an XCAF document, with name and colour modes on. IGES goes through `IGESCAFControl_Reader`, and BRep through `BRepTools::Read`.
- **Units.** Everything is converted to mm: the document length unit and `xstep.cascade.unit` are both set to mm. `sourceUnit` comes from the file's declared length unit. This was verified on `as1_pe_203.stp`, which is the as1 model declared in inches: every volume comes back exactly 25.4³ times the mm file's.
- **Tree.** Each free shape becomes a root. When there are several, they sit under a synthetic root whose `id` and `name` are `""`.
  - Assemblies become nodes, and their components become children. `transform` is the component location relative to its parent, as a 4×4 column-major matrix in mm.
  - Leaves carry `shapeId`. A free shape that is itself a simple shape gets an identity transform, and its own location stays inside the prototype.
- **Names.** A node's name is the instance name, or the referred product's name if there's no instance name. STEPCAF's invented `=>[0:1:1:18]` names count as "no name".
- **Ids.**
  - An `InstanceId` is the `/`-joined path of names from the root, such as `as1/rod-assembly_1/nut_1`.
  - When siblings share a name, the segment becomes `<k>:<name>`, where k counts from 0 in file order. For example, `AS1_PE_ASM/0:L_BRACKET_ASSEMBLY/...`.
  - `/` and `%` in names are percent-encoded.
  - A `ShapeId` is `s<n>`, numbered in order of first encounter. It is stable for a given file.
- **ShapeInfo.**
  - `color`: the file's RGB, unconverted. It is looked up on the shape label first, then the shape's sub-shapes, then the first instance.
  - `bbox`: from `BRepBndLib::AddOptimal`.
  - `faceCount`, `edgeCount`: distinct faces and edges (`TopExp::MapShapes`).
  - `volume`, `centroid`, `principalMoments`: from `BRepGProp` volume properties at unit density, with the moments ascending and taken about the centroid.
  - `area`: from surface properties.
  - Shapes with no solid report `volume = 0` and take their centroid and moments from the surface.

### `mesh`

`BRepMesh_IncrementalMesh` runs at the requested deflections. The previous triangulation is cleaned first, so the result is deterministic.

- **Vertices** are per face, not shared across faces.
- **Normals** are the surface normals from `BRepLib_ToolTriangulatedShape::ComputeNormals`.
- **Winding and normals** are flipped on REVERSED faces, so `cross(p1-p0, p2-p0)` points out of a solid. The test checks that the mesh's signed volume matches the B-rep volume.
- **`faceOfTriangle`** is the 0-based face index in `TopExp::MapShapes(shape, TopAbs_FACE)` order. That is the same order `faceCount` counts.

### `hlr`

- **Projector.** `HLRAlgo_Projector(gp_Ax2(origin, viewDirection, right))`, where `right = up × viewDirection`. View +x is right, +y is the component of `up` perpendicular to the view, and output is at 1:1 in mm.
  - **Orthographic:** the world origin projects to (0, 0).
  - **Perspective:** the image plane passes through the world origin, perpendicular to `viewDirection`. The eye sits `eye·viewDirection` in front of that plane. All output is then shifted so that the world origin lands at (0, 0).
- **Algorithms.**
  - **Exact** uses `HLRBRep_Algo` on the B-rep.
  - **Polygonal** uses `HLRBRep_PolyAlgo` on meshes, with linear deflection 0.1% of the scene diagonal.
- **Attribution.** Each instance is added as its own shape: the prototype moved by its transform. Lines are extracted per shape with `HLRBRep_HLRToShape::CompoundOfEdges(S, …)`, or the per-shape `PolyHLRToShape` compounds. Every edge therefore carries its own instance id from a single HLR run. Two placements of one prototype stay separate.
  - With `targetInstanceIds` alone, there is one run with every instance and only the targets are extracted.
  - With `occludersByTarget`, there is one run per target, containing that target and its occluders. If a target has no entry, every instance occludes it.
  - Instances whose shapes have no faces are skipped.
- **Edge classes.** OCCT's `OutLine` maps to `outline`, `Sharp` to `sharp`, `Rg1Line` to `smooth`, and `RgNLine` to `seam`. RgN lines are mostly cylinder and cone seams. `IsoLine` is never emitted. `smooth` and `seam` appear only when `includeSmooth` is set. Hidden lines appear only when `includeHidden` is set.
- **Curves.** The exact algorithm emits true curves. For each HLR output edge (a 2D edge on the projection plane):
  1. OCCT `Line`, `Circle` and `Ellipse` types map directly to `line`, `arc` and `ellipse`.
  2. Any other curve (B-spline, Bézier, offset, other) is sampled at 33 points.
     - If the samples are collinear within 1 µm, it becomes a `line`. Edge-on circles come out of HLR as degenerate B-splines.
     - Otherwise, if a circle through the samples fits within 1 µm, it becomes an `arc`.
     - Otherwise, a direct least-squares conic (Halir–Flusser) is fitted. If every one of 49 samples lies within 1 µm, it becomes an `ellipse`. Many exporters write circles as rational B-splines, and as1-oc-214 does this for every circle. Seen at an angle, those circles still come out as ellipses.
  3. Otherwise, `GeomConvert_ApproxCurve` (C1, degree ≤ 3, tolerance 1 µm) feeds `GeomConvert_BSplineCurveToBezierCurve`, and the result is emitted as `bezier`: cubic segments `p0 c1 c2 p1 c1 c2 p2 …`.
  4. A `polyline` comes from `GCPnts_QuasiUniformDeflection` and is the last resort only.

  Every arc and ellipse built from OCCT's own conic parameters is checked again. Its start, middle and end points are recomputed from the emitted parameters and compared with the real edge. If any is off by more than 10 µm, it is re-emitted as a general curve. The polygonal algorithm emits `line` and `polyline` only. Segments are chained through joints where exactly two segments meet and the turn is under 40°.
- **Angle conventions** (proposed as a protocol clarification, below):
  - Angles are radians, in the y-up view frame.
  - `ccw` means counter-clockwise in that frame.
  - `startAngle` is in (−π, π]. `endAngle` keeps the signed sweep: `endAngle > startAngle` when `ccw`, `<` otherwise, and `|endAngle − startAngle| ≤ 2π`. A full circle therefore has a sweep of exactly ±2π.
  - For an `ellipse`, `startAngle` and `endAngle` are *parametric* (eccentric-anomaly) angles in the ellipse's own frame: `point(θ) = center + R(rotation)·(major·cos θ, minor·sin θ)`. This is the same as canvas `ellipse()` and OCCT.
- **Timing.**
  - `setupMs`: placing shapes, adding them to the algorithm, the projector and any meshing.
  - `hlrMs`: `Update` and `Hide`.
  - `extractMs`: extracting compounds and emitting curves.

  Each is summed over runs.

### `clearance`

`BRepExtrema_DistShapeShape` (multi-threaded, minimum only) runs on the two placed shapes.

- If one shape lies inside the other (`InnerSolution`), the distance is 0 and they interfere.
- If the distance is ≤ `tolerance`, the reported distance is 0. `interferes` is then true when `BRepAlgoAPI_Common` leaves a real volume, above `max(1e-6 × smaller volume, tolerance³)`. Faces that only touch, like a bolt in an exact-fit hole or a bracket resting on a plate, give 0 and don't interfere.

### `sweep`

The sweep is mesh-based, with an exact check of every hit. Exact conservative advancement was correct, but each B-rep distance or boolean costs 0.2–0.7 s on the corpus. Parts that slide along each other, like a rod through its holes, need hundreds of them: 375 s for one query.

1. **Prefilter.** A swept-AABB slab test finds the obstacles whose box the moving box crosses along the ray. If there are none, the answer is `Infinity`.
2. **Mesh contact.** Each part is meshed at deflection 0.1% of its diagonal, and the mesh is cached. Two sets of rays are cast:
   - from the moving part's *leading* facets (normal along `direction`) into the obstacle's facets that face back at it;
   - from the obstacle's facing facets, backwards, into the moving part's leading facets.

   Ray origins are the facet vertices plus points every 4× the deflection along facet edges, which covers edge-on-edge contacts. Trailing facets, and facets parallel to the motion, are never tested, so sliding contact can't register as a hit.
3. **Exact confirmation.** The earliest hit is checked with an exact boolean just past it. A hit counts only if the common volume there is real. Otherwise it is skipped, along with hits close to it.
4. **Result.** `freeDistance` is the first confirmed hit and `blockedBy` is that obstacle. If nothing is hit within `maxDistance`, the result is `Infinity` when no obstacle box lies anywhere along the ray, and `maxDistance` otherwise.

On as1, the mesh sweep reproduces all eight answers from the exact method: blocked at 0, free (`Infinity`), and the bolt meeting the rod at 22.0 mm. Each query takes 0.1–3 s.

## Measured (as1-oc-214, 18 instances, 5 shapes; WSL2, 22 cores)

| | Time |
|---|---|
| Process start (loading OCP) | 3.4 s |
| `import` | 0.34–0.52 s |
| Exact HLR, iso view, visible only | 3.4–4.1 s wall (`hlrMs` 3.4–3.9 s) |
| Exact HLR, iso view, hidden and smooth | 3.5–4.3 s wall (`hlrMs` 2.9–3.5 s, `extractMs` 0.6 s) |
| Polygonal HLR, iso view, hidden and smooth | 0.6 s (`hlrMs` 0.1 s) |
| `clearance` per pair | 0.15–0.3 s |
| `sweep` | 0.1–3 s |

as1 is slow for its size because the exporter wrote every cylinder and circle as a B-spline. A NIST part with analytic geometry (CTC 01, 256 edges out) runs exact HLR in about 0.12 s.

## Known limitations

- **One document at a time.** `import` replaces the session: the shape ids of an earlier import become unknown. To compare two revisions, run two processes, or import one after the other and keep each `ImportResult`. A document or session id is proposed below.
- **Shape names in some files are type names.** as1_pe_203 names its solids `SOLID` and `COMPOUND`, and the kernel reports what the file says. The meaningful name is on the parent product node.
- **Perspective views lose most true curves.** HLR's perspective output for circles isn't within 1 µm of a conic, so circles in perspective come out as `bezier`. They are still exact enough to draw.
- **Edge-on circles become two overlapping lines.** HLR returns them as two degenerate halves, and both are kept.
- **Polygonal HLR has dash artefacts in axis-aligned views.** A face seen exactly edge-on (for example, a hole wall in a top view) hides its own rim in alternating pieces. Use exact HLR for those views.
- **Sweep accuracy** is about the mesh deflection (0.1% of the part's size). Edge-on-edge contacts are found to within the edge-sample spacing (4× the deflection). Contacts between features smaller than that can be missed (false negatives), and parts that interfere at the start position aren't reported. Use `clearance` to verify final exploded positions. If 25 hits in a row fail exact confirmation, the next mesh hit is accepted unconfirmed.
- **Sweep beyond `maxDistance`.** When nothing is hit within `maxDistance` but an obstacle's box lies further along the path, the result is `maxDistance`, not `Infinity`.
- **A failed boolean counts as interference.** If `BRepAlgoAPI_Common` fails, clearance reports `interferes: true` and sweep accepts the hit. Both are logged on stderr.
- **Shapes without faces** (pe_203's empty `COMPOUND` leaves) are skipped as HLR instances and as sweep obstacles. As clearance arguments or as the moving part of a sweep, they are a `bad-request`.
- **Clearance** costs a B-rep distance per call, and there's no bounding-box shortcut, because the protocol wants the actual distance.
- Transforms must be rigid, with an optional uniform scale (`gp_Trsf`). Anything else is a `bad-request`.
- **Tessellated-only geometry is refused by `hlr`.** AP242 `TESSELLATED_SOLID`/`TRIANGULATED_FACE` imports as faces with triangles and no B-rep edges (`edgeCount: 0`), for example NIST `nist_ftc_08_asme1_ap242-e1-tg.stp`. Exact HLR segfaults on it and polygonal HLR throws `NCollection_Array1` out of range, so `hlr` answers `bad-request` when any drawable instance has faces but no edges. Import still works.
- **Unnamed shape labels.** In OCP 8.0.1, `TDF_Label::FindAttribute` for `TDataStd_Name` segfaulted on some labels that have no name (NIST CTC 02's referred shape labels). Names are now looked up only after `IsAttribute` says one exists.
- **Not supported:** IGES and BRep are implemented but untested, and IGES `sourceUnit` is best effort. PMI, layers and GD&T are not read.

## Proposed protocol changes

`protocol.ts` is unchanged. These came out of implementing it:

1. **A document or session id on `import`.** `ImportResult` would carry a `documentId`, and `ShapeId`s would be scoped to it, so that two revisions can be loaded at once for matching. Today each import replaces the last one.
2. **Pin the `Curve2D` angle conventions.**
   - Angles are in the y-up view frame, and `ccw` is counter-clockwise in that frame.
   - `startAngle` is in (−π, π].
   - `endAngle − startAngle` is the signed sweep: it agrees with `ccw` and is at most 2π, so a full circle is ±2π.
   - Ellipse angles are parametric, about the major axis.
3. **Non-finite numbers on JSON transports.** `SweepResult.freeDistance = Infinity` cannot be plain JSON. Either standardise the `{"$float": "Infinity"}` encoding for the native transport, or change the field to `freeDistance: number | null` with `null` meaning "escapes".
4. **Sweep semantics.**
   - Sliding contact is not blocking; only a real common volume is.
   - Parts that interfere at the start position are not reported.
   - Accuracy is about the mesh deflection. Consider an optional `deflection` hint.
   - Say what happens when nothing is hit within `maxDistance` but an obstacle lies further along the path. This kernel returns `maxDistance`, and returns `Infinity` only when nothing lies along the path at all.
5. **Perspective.** State that 1:1 applies on the plane through the world origin perpendicular to `viewDirection`, and that the world origin projects to (0, 0).
6. **`faceOfTriangle`.** Define the index as the face order of `TopExp::MapShapes(shape, FACE)`, which is the order `faceCount` counts, and say that vertices are per face.
7. **Optional deflection hints** for polygonal HLR and sweep. Both currently choose internal values.
8. **`InstanceId` format.**
   - A segment is `<k>:<name>` only when siblings share a name.
   - `/` and `%` are percent-encoded.
   - The synthetic root of a multi-root file has id `""`.
9. **Edge-class mapping.** Document that OCCT `RgNLine` maps to `seam` and that iso-lines are never emitted.
