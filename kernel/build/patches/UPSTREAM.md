# Upstream bug reports (drafts, not submitted)

Drafts for the Open CASCADE issue tracker (https://github.com/Open-Cascade-SAS/OCCT/issues), one per patch in this directory, against OCCT 8.0.1 (tag `V8.0.1`, commit `b8f597c6`). They are not submitted: submitting is Michael's decision, and each needs a last read against OCCT's current master first.

Every reproduction here uses only public or synthetic shapes. The Voron models where 0001–0003 were found are GPL-3.0 and must not be attached or described in detail (`corpus/LICENSES.md`).

---

## 1. `ShapeFix_IntersectionTool::FixSelfIntersectWire` is cubic in the wire's edge count (patch 0001)

**Summary.** The double loop over edge pairs reads `sewd->Edge(num1)` and `sewd->Edge(num2)` on every iteration. `ShapeExtend_WireData` keeps its edges in an `NCollection_Sequence`, a linked list whose `Value(i)` walks from its cached cursor, and the two indices alternate, so each read walks about `num2 - num1` nodes. The loop is O(N³) in the wire's edge count, even when no pair of boxes overlaps and nothing is fixed.

**Effect.** On a real 1,691-part assembly (a 3D printer, 179 MB STEP) this one loop was 40% of `STEPCAFControl_Reader::Transfer` (perf, DWARF call graphs). With the patch the import went from 242 s to 134 s, with byte-identical results (every shape's BinTools B-rep compared).

**Reproduction** (synthetic, `kernel/tools/cache-diag/shapefix-repro.cpp`). A planar face bounded by a regular polygon of N edges, which does not self-intersect, through `ShapeFix_Wire::FixSelfIntersection` with `FixNonAdjacentIntersectingEdgesMode() = 1` and a `ShapeBuild_ReShape` context. Measured on OCCT 8.0.1, Release, gcc 13.3, 4 Oct 2026, load average about 8 on 22 vCPUs:

| Edges | 8.0.1 | Patched |
|---:|---:|---:|
| 500 | 0.050 s | 0.023 s |
| 1,000 | 0.350 s | 0.060 s |
| 2,000 | 2.78 s | 0.134 s |
| 4,000 | 19.9 s | 0.575 s |

Each doubling costs ×7–8 unpatched (cubic) and ×2–4 patched (the pair loop itself stays quadratic).

**Suggested fix** (what patch 0001 does). Read the edges from an array snapshot of the sequence, and take the snapshot again whenever the wire may have changed. Every change in the loop happens after a box overlap, so marking the snapshot stale there is enough. A cleaner upstream fix would give `ShapeExtend_WireData` indexed storage.

---

## 2. `Contap_Contour::Perform` reads outside a line when a walked line has no recorded tangent (patch 0002)

**Summary.** In the loop over the walking lines, the transition on the line is computed from `iwline->TangentVector(k)` and `iwline->Line()->Value(k)`. For a line whose walk never recorded a tangent, `k` comes back as -1 and `Value(-1)` reads before the start of the line's point sequence. Natively this is a null dereference (SIGSEGV in exact HLR, `HLRBRep_Algo::Update` → contour computation). In WebAssembly a low address doesn't trap, so it reads arbitrary memory and returns an arbitrary transition. The native and WASM builds of the same code can therefore draw the same part differently.

**Effect.** Exact HLR crashed on some visible parts, in views where an enclosure that normally hides them was removed. Those crashes were in batches of several parts. This branch has not been reproduced with a part on its own: the 14-view sweep of the part that crashed took the 0003 branch, never this one (table below).

**Reproduction.** The parts where it happened are in a GPL-3.0 model and can't be attached. A search of public shapes found no reproduction yet; see "Searching for public reproductions" below. A report can give the call stack and the patched condition without attaching the part, and ask whether the OCCT team's own test data reaches `k < 1` in this loop. An `Standard_ASSERT_RETURN(k >= 1, ...)` in a debug build would show it.

**Suggested fix** (what patch 0002 does). When `k < 1` or `k > NbPoints`, take the tangent from the line's first two points. For a one-point line, leave the transition undecided, as for a zero tangent.

---

## 3. `IntWalk_IWalking::FillPntsInHoles` can read past the list of lines (patch 0003)

**Summary.** An "alone" point records the number of the line that was being walked when it was found. That line can be one still in progress which is never added (`lines.Length() + 1`). `FillPntsInHoles` then calls `lines.Value(std::abs(Irang1))` with that number. It reads past the sequence and moves its cursor to the wrong node, so the next `lines.Value()` returns a null handle: a segfault natively, arbitrary memory in WASM. `Arrive()` already skips such points; `FillPntsInHoles` doesn't.

**Effect.** As for 2: crashes in exact HLR (the outline walk goes through `Contap_TheIWalking`) on the same kind of parts and views.

**Reproduction.** Found on GPL-3.0 parts only. Unlike 2, it reproduces with one of those parts on its own: exact HLR of that part alone, viewed from (-1, -1, 1), takes this branch 100 times (table below). No public shape has been found that takes it.

**Suggested fix** (what patch 0003 does). Skip alone points whose line number is 0 or greater than `lines.Length()`, for both `Irang1` and `Irang2`, as `Arrive()` does.

---

## 4. A BinTools write-read round trip doesn't give back the shape it wrote (patch 0004)

**Summary.** `BinTools::Write` writes doubles exactly, but reading changes them in three places:

1. Every direction is read through `gp_Dir::SetCoord`, which normalises again. The written components are already a normalised `gp_Dir`'s, and normalising them again can change their last bits.
2. Every axis frame (conics, elementary surfaces, 2D conics) is rebuilt from its main and X directions (`gp_Ax2(P, N, Vx)`, `gp_Ax3(P, N, Vx)`, `gp_Ax22d`). The rebuilt X and Y directions differ in the last bits from the written ones, which are read and then ignored.
3. Each location is written as its scaled 3×4 matrix, and read back through `gp_Trsf::SetValues`. That takes the scale from the cube root of the determinant, divides it out, re-orthogonalises the matrix and sets the form to `gp_CompoundTrsf`. The original form, such as `gp_Rotation`, selects other branches in `gp_Trsf::Multiply`.

`BinTools` with triangulation behaves the same; `BRepTools` text is worse (decimal text: 18 of the 20 files differ).

**Effect.** For us this decides whether a model can be cached. We keep the imported B-reps so that a file is not read again on the next open. Exact HLR from the read-back shapes differed in the last bits on 9 of 20 test files. On NIST STC 08, one 17 mm edge that lies on a visibility boundary switched from hidden to visible: sharp edges went from 353 visible / 282 hidden to 354 / 281. This file is a NIST test model, "used without any restrictions", from https://go.usa.gov/mGVm, file `nist_stc_08_asme1_ap242-e3.stp`. Read it with `STEPCAFControl_Reader` into an XCAF document. Take each free shape's referred prototype and its placements. Run `HLRBRep_Algo` with hidden and smooth edges on, orthographic, viewed from (1, 1, 1)/√3 with up (0, 0, 1). Then run the same on the prototypes after `BinTools::Write`/`Read`. `kernel/tools/cache-diag/diag.cpp` does exactly this.

**Reproduction** (synthetic, `kernel/tools/cache-diag/bintools-repro.cpp`). An edge on a circle with axis (0.3, 0.2, 1) and X direction (1, -0.7, 0.1), and a plane face with normal (-0.4, 0.9, 0.35), moved by a rotation about (0.3, 0.2, 1) plus a translation. They are written in one compound with `BinTools::Write` (format 4) and read back with `BinTools::Read`, then compared bit for bit. Output on 8.0.1:

```
circle axis direction                        identical
circle X direction                           DIFFERS
circle Y direction                           DIFFERS
plane normal                                 DIFFERS
plane X direction                            DIFFERS
plane Y direction                            DIFFERS
location form 7 -> 7, scale factor 1 -> 1
location (form, scale, matrix, translation)  DIFFERS
```

With the patch, writing format 5, all eight are identical.

**Suggested fix** (what patch 0004 does).

- Directions: keep the written components when they already form a unit vector to 1e-12. Anything else is normalised as before.
- Frames: build the frame as now, then keep the written X and Y directions when the rebuilt ones agree with them to 1e-12.
- Locations: a new format version 5 writes each elementary location exactly (form, scale factor, matrix without the scale, translation) as location record type 3, and reads it back field by field. Versions 1–4 are read and written as before, and `BinTools_FormatVersion_CURRENT` stays 4, so files written for other readers don't change.

Upstream may prefer to make the exact write the default in a new current version, and the same fix probably applies to `BinTools_ShapeReader`/`ShapeWriter` and `TopTools`' text format.

---

## Searching for public reproductions of 2 and 3

`kernel/tools/cache-diag/diag.cpp --sweep` runs exact HLR, with hidden and smooth edges on, of a whole document and of each shape on its own, from 14 view directions: the 6 axes and the 8 diagonals. It is run against an OCCT copy instrumented to print a line whenever the branches patched by 0002 and 0003 are taken. Results, 4 Oct 2026:

| File (licence) | HLR calls | Branch taken |
|---|---:|---|
| NIST CTC/FTC/STC, 17 files (US government work, no restrictions) | 602 | never |
| Leader arm and follower arm (MIT) | 742 | never |
| SO-ARM100 SO101 assembly (Apache-2.0), each shape alone | 501 (views 1–5 of 14; stopped after one shape's HLR ran 30 minutes; the whole-assembly calls were skipped after the first ran 10) | never |
| ODRI biped (BSD-3) | 798 | never |
| Positive control: one Voron 0.2 part where exact HLR crashed, alone (GPL-3.0, internal only; not for the report) | 14 | 0003: 100 times, in one of the 14 views; 0002: never |

No public shape reached either branch in 2,643 HLR calls. The instrumented copy is the patched OCCT with one `fprintf` in each patched branch (not committed: it is a throwaway build). So 0002 and 0003 still lack a public reproduction. Reporting them with the patched code paths and the call stacks, as above, is the honest option. Shipping a GPL part to reproduce them is not an option.

