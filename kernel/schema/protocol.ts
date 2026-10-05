/**
 * The kernel <-> pipeline protocol, version 1.
 *
 * This file is the LGPL boundary. Everything that touches OCCT lives in
 * kernel/ and answers these messages; everything proprietary lives in
 * pipeline/ and only ever sends them. The same messages are answered by the
 * WASM kernel in a Web Worker (browser, local mode) and by the native kernel in
 * a job process (cloud mode), so the pipeline cannot tell which one it is
 * talking to. Spikes produce fixtures in exactly these shapes.
 *
 * Conventions:
 * - Lengths are millimetres. Importers convert from the file's units, so a
 *   file saved in inches and the same file saved in mm produce identical numbers.
 * - Transforms are 4x4 column-major (the three.js / WebGL convention), mm.
 * - 2D results are in view coordinates: +x right, +y up, mm at 1:1, origin at
 *   the projection of the world origin. Page scaling is the pipeline's job.
 * - Binary payloads (meshes) travel as typed arrays, so a postMessage can
 *   transfer them without copying. Over the native kernel's JSON-lines pipe
 *   they are `{"$typed": kind, "base64": ...}` (little-endian), and non-finite
 *   numbers are `{"$float": "Infinity"}` (kernel/reference/README.md). Inside the WASM
 *   build the response JSON points into a binary side section instead,
 *   `{"$typed": kind, "offset": o, "byteLength": n}` (pipeline/kernel/wasm-client.ts);
 *   either way the client hands the pipeline real typed arrays.
 * - Perspective views: the plane through the world origin perpendicular to the
 *   view direction is drawn at 1:1, and the world origin projects to (0, 0).
 *
 * Curve angle conventions (Curve2D):
 * - angles in radians, measured with +y up;
 * - startAngle in (-pi, pi]; endAngle = startAngle + signed sweep, where the
 *   sweep's sign matches `ccw` and |sweep| <= 2*pi (a full circle is +-2*pi);
 * - ellipse angles are parametric angles about the major axis, not polar angles.
 *
 * Edge classes from OCCT's HLRBRep_HLRToShape: Outline -> outline, Sharp ->
 * sharp, Rg1Line -> smooth, RgNLine -> seam. Iso-lines are never sent.
 */

export const PROTOCOL_VERSION = 1;

// ---------------------------------------------------------------- identity

/** One distinct solid shape (a "part" in CAD terms), shared by every placement of it. */
export type ShapeId = string;
/**
 * One placement of a shape in the assembly: the path of instance names from the
 * root, e.g. "frame/0:left-bracket/2:bolt-M8x20". It is stable for a given file,
 * and it is NOT stable across CAD revisions. Matching revisions is the
 * pipeline's job (pipeline/match), not the kernel's.
 */
export type InstanceId = string;
/*
 * Instance id format: path segments joined by "/". Inside a segment, "/" and "%"
 * are %-escaped. Duplicate sibling names get a "k:" index prefix ("2:bolt"). A
 * file with several top-level shapes gets a synthetic root whose id is "".
 */

export type Mat4 = readonly number[]; // length 16, column-major
export type Vec3 = readonly [number, number, number];
export type Vec2 = readonly [number, number];

export interface BBox {
  min: Vec3;
  max: Vec3;
}

// ---------------------------------------------------------------- import

export type CadFormat = "step" | "iges" | "brep";

export interface ImportRequest {
  type: "import";
  format: CadFormat;
  /** The file's bytes. Never leaves the machine in local mode. */
  bytes: Uint8Array;
}

export interface AssemblyNode {
  /** Instance id for leaf placements; a synthetic path for sub-assemblies. */
  id: InstanceId;
  /** The name as written in the file (product name, or instance name if there is one). */
  name: string;
  /** Placement relative to the parent node. */
  transform: Mat4;
  /** Set on leaves only: the shape placed here. */
  shapeId?: ShapeId;
  children: AssemblyNode[];
}

export interface ShapeInfo {
  shapeId: ShapeId;
  name: string;
  /** sRGB 0..1, if the file colours the shape; otherwise absent. */
  color?: readonly [number, number, number];
  /** In the shape's own coordinates. */
  bbox: BBox;
  faceCount: number;
  edgeCount: number;
  /** Volume, mm^3. Used for part matching and explode ordering. */
  volume: number;
  /** Surface area, mm^2. */
  area: number;
  /**
   * Principal moments of inertia (ascending), unit density, about the centroid.
   * Rotation-invariant: used for shape signatures in revision matching.
   */
  principalMoments: Vec3;
  centroid: Vec3;
}

export interface ImportResult {
  type: "import:ok";
  protocolVersion: typeof PROTOCOL_VERSION;
  /** The unit the file declared, before conversion to mm. */
  sourceUnit: "mm" | "cm" | "m" | "in" | "ft" | "unknown";
  root: AssemblyNode;
  shapes: ShapeInfo[];
  /** Non-fatal problems: unreadable entities, missing colours, healed faces. */
  warnings: string[];
}

// ---------------------------------------------------------------- mesh (viewer, previews, fallback HLR)

export interface MeshRequest {
  type: "mesh";
  shapeIds: ShapeId[];
  /** Chordal deviation, mm. */
  linearDeflection: number;
  /** Radians. */
  angularDeflection: number;
}

export interface ShapeMesh {
  shapeId: ShapeId;
  positions: Float32Array; // xyz triples, shape coordinates, mm
  normals: Float32Array;
  indices: Uint32Array;
  /**
   * Triangle index -> B-rep face index (0-based, in TopExp::MapShapes(shape, FACE) order:
   * distinct faces, the order faceCount counts), so picks and fills map back to faces.
   * Triangles of one face are contiguous, and vertices are per face (not shared across faces).
   */
  faceOfTriangle: Uint32Array;
}

export interface MeshResult {
  type: "mesh:ok";
  meshes: ShapeMesh[];
}

// ---------------------------------------------------------------- hidden-line removal

export interface ViewSpec {
  /** Unit vector from the scene towards the viewer. */
  viewDirection: Vec3;
  /** Unit vector that points up on the page. */
  up: Vec3;
  projection: "orthographic" | "perspective";
  /** Perspective only: eye position, mm. */
  eye?: Vec3;
}

export interface PlacedInstance {
  instanceId: InstanceId;
  shapeId: ShapeId;
  /** World placement, including any explode offset the pipeline applied. */
  transform: Mat4;
}

export interface HlrRequest {
  type: "hlr";
  view: ViewSpec;
  instances: PlacedInstance[];
  /** "exact" = OCCT HLRBRep on the B-rep; "polygonal" = HLRBRep_PolyAlgo on a mesh. */
  algorithm: "exact" | "polygonal";
  includeHidden: boolean;
  /** Smooth (tangent-continuous) edges: usually drawn thin, broken, or not at all. */
  includeSmooth: boolean;
  /**
   * Optional: compute lines only for these instances, while every other
   * instance still occludes. This lets the pipeline split a big view into jobs
   * (one per worker) and re-run only the parts that moved.
   */
  targetInstanceIds?: InstanceId[];
  /**
   * Optional occluder culling, done by the pipeline: for each target, the
   * instances that can possibly hide it. If absent, every instance occludes
   * every target. A target with no entry is hidden by every instance. The
   * kernel hides each target's edges only by its own faces and its listed
   * occluders, and need not load instances that are neither.
   */
  occludersByTarget?: Record<InstanceId, InstanceId[]>;
  /**
   * Optional (large views): load an instance as only these faces (0-based, in
   * TopExp::MapShapes(FACE) order, the order of a mesh's faceOfTriangle); an empty list
   * loads nothing. The pipeline passes, for each instance of a batch, the faces whose
   * projection reaches the regions where the targets it is (or hides) may be visible. A
   * face that reaches none of a target's region can neither be drawn there nor hide
   * anything there, so inside those regions the visible lines are the same, and the
   * kernel processes a fraction of the geometry. A target's lines outside its region
   * are then incomplete: the pipeline clips them to the region (it does anyway). A
   * kernel that ignores this loads whole instances.
   */
  instanceFaces?: Record<InstanceId, number[]>;
}

export type EdgeClass =
  | "outline" // silhouette / contour of a curved surface, or outer boundary
  | "sharp" // a B-rep edge between faces meeting at an angle
  | "smooth" // a B-rep edge between tangent-continuous faces
  | "seam"; // a periodic-surface seam (normally never drawn)

/** 2D curve geometry in view coordinates (mm). True curves where the kernel has them. */
export type Curve2D =
  | { kind: "line"; a: Vec2; b: Vec2 }
  | { kind: "arc"; center: Vec2; radius: number; startAngle: number; endAngle: number; ccw: boolean }
  | {
      kind: "ellipse";
      center: Vec2;
      majorRadius: number;
      minorRadius: number;
      /** Rotation of the major axis from +x, radians. */
      rotation: number;
      startAngle: number;
      endAngle: number;
      ccw: boolean;
    }
  | { kind: "bezier"; points: Vec2[] } // cubic segments: p0 c1 c2 p1 c1 c2 p2 ...
  | { kind: "polyline"; points: Vec2[] }; // last resort; the pipeline tries to fit curves

export interface HlrEdge {
  instanceId: InstanceId;
  edgeClass: EdgeClass;
  visible: boolean;
  curve: Curve2D;
}

export interface HlrResult {
  type: "hlr:ok";
  edges: HlrEdge[];
  /** Wall-clock milliseconds, split so performance work knows where time goes. */
  timing: { setupMs: number; hlrMs: number; extractMs: number };
}

// ---------------------------------------------------------------- shape cache (large views)
/*
 * Importing a STEP assembly can take minutes (a 1,700-part printer: 150-290 s
 * native), so a kernel instance that only has to run one HLR batch must not
 * re-read the file. The importing kernel serialises shapes once (OCCT binary
 * B-rep, no triangulation), and fresh kernels load only the shapes their batch
 * needs. The bytes are opaque to the pipeline: it stores and forwards them.
 */

export interface ExportShapesRequest {
  type: "exportShapes";
  /** Shapes from the last import. */
  shapeIds: ShapeId[];
}

export interface ExportShapesResult {
  type: "exportShapes:ok";
  shapes: { shapeId: ShapeId; brep: Uint8Array }[];
}

export interface LoadShapesRequest {
  type: "loadShapes";
  /** Each shape's slice of `bytes`, as produced by exportShapes. */
  shapes: { shapeId: ShapeId; offset: number; length: number }[];
  /** The B-reps, concatenated. Travels beside the JSON like an import's bytes. */
  bytes: Uint8Array;
  /** Drop every shape the kernel holds first. Otherwise the shapes are added. */
  replace?: boolean;
}

export interface LoadShapesResult {
  type: "loadShapes:ok";
  loaded: number;
  /** Shapes the kernel holds now. */
  held: number;
}

// ---------------------------------------------------------------- import cache (never import twice)
/*
 * A large STEP import takes minutes (spikes/perf-large/RESULT.md), so a file is imported
 * once. Right after an import, `importCache` returns the whole document as one blob: the
 * import's own ImportResult, and every shape as OCCT binary B-rep (BinTools, no
 * triangulation) with the per-shape state the kernel keeps (face and edge counts, solid,
 * volume, bbox diagonal). `loadCache` with that blob leaves a kernel exactly as the import
 * left it, and answers with the same ImportResult; every later request (mesh, hlr,
 * clearance, sweep) gives the same answer as after the import. It takes seconds.
 *
 * Storing the blob and choosing its key is the pipeline's job (pipeline/kernel/import-cache.ts):
 * OPFS in the browser, a directory natively, keyed by the file's SHA-256, the format and the
 * kernel version (`ImportCacheHeader.kernel`), so a new kernel never reads an old import.
 *
 * Backward compatible: two new request types; a kernel without them answers "bad-request"
 * (unknown request type) and the pipeline imports as before.
 *
 * Blob layout, little-endian, documented so the pipeline can also hand single shapes to
 * `loadShapes` (large views) without asking a kernel:
 *   bytes 0-7    ASCII "CADOCIC1"
 *   bytes 8-11   u32 header length H
 *   bytes 12..   the header, UTF-8 JSON (ImportCacheHeader)
 *   byte D..     data, D = 12 + H rounded up to a multiple of 8. Header offsets are relative
 *                to D; each shape's B-rep starts at a multiple of 8.
 */

export interface ImportCacheHeader {
  format: "cadoc-import-cache";
  version: 1;
  /** The kernel that wrote it (its version string, OCCT version included). */
  kernel: string;
  /** The import's ImportResult, as JSON text. */
  import: { offset: number; length: number };
  shapes: {
    shapeId: ShapeId;
    /** The shape's B-rep (exportShapes format); length 0 for a shape with no geometry. */
    offset: number;
    length: number;
    faceCount: number;
    edgeCount: number;
    solid: boolean;
    volume: number;
    /** Diagonal of the shape's optimal bounding box, mm. */
    diag: number;
  }[];
}

export interface ImportCacheRequest {
  type: "importCache";
}

export interface ImportCacheResult {
  type: "importCache:ok";
  /** Shapes in the blob. */
  shapes: number;
  /** The blob. (Native transport: absent when the request named a "path" to write it to.) */
  cache: Uint8Array;
}

export interface LoadCacheRequest {
  type: "loadCache";
  /** A blob from importCache. Travels beside the JSON like an import's bytes. */
  bytes: Uint8Array;
}

export interface LoadCacheResult {
  type: "loadCache:ok";
  /** The ImportResult of the import the blob was made from, unchanged. */
  import: ImportResult;
}

// ---------------------------------------------------------------- spatial queries (explode, collisions)

export interface ClearanceRequest {
  type: "clearance";
  a: PlacedInstance;
  b: PlacedInstance;
  /** Distances under this count as contact, mm. */
  tolerance: number;
}

export interface ClearanceResult {
  type: "clearance:ok";
  /** Minimum distance, mm. Zero if the two touch or interfere. */
  distance: number;
  interferes: boolean;
}

/**
 * Sweep query for exploding: how far can `moving` travel along `direction`
 * before it hits any of `obstacles`?
 * - Sliding contact (surfaces moving along each other) does not block.
 * - Overlap already present at the start is not reported; check it with `clearance`.
 * - Infinity only when no obstacle lies anywhere on the path; if one lies on the
 *   path beyond `maxDistance`, the result is `maxDistance`.
 * - Accuracy is about 0.1% of the moving part's size (mesh-based), so confirm
 *   final explode positions with `clearance`.
 */
export interface SweepRequest {
  type: "sweep";
  moving: PlacedInstance;
  direction: Vec3;
  obstacles: PlacedInstance[];
  maxDistance: number;
  tolerance: number;
}

export interface SweepResult {
  type: "sweep:ok";
  freeDistance: number;
  /** The obstacle hit first, if any. */
  blockedBy?: InstanceId;
}

// ---------------------------------------------------------------- envelope

export type KernelRequest =
  | ImportRequest
  | MeshRequest
  | HlrRequest
  | ClearanceRequest
  | SweepRequest
  | ExportShapesRequest
  | LoadShapesRequest
  | ImportCacheRequest
  | LoadCacheRequest;
export type KernelResult =
  | ImportResult
  | MeshResult
  | HlrResult
  | ClearanceResult
  | SweepResult
  | ExportShapesResult
  | LoadShapesResult
  | ImportCacheResult
  | LoadCacheResult;

export interface KernelError {
  type: "error";
  /** "crash" means the kernel process or worker died; the pipeline restarts it. */
  code: "unsupported-format" | "read-failed" | "out-of-memory" | "timeout" | "crash" | "bad-request";
  message: string;
}

export interface Envelope<T> {
  id: number;
  body: T;
}
