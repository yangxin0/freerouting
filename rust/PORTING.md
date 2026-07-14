# Rust port — progress tracker

Incremental port of the Java sources (`src/main/java/app/freerouting`, 484 files)
to the `rust/` crate. Updated by each `/loop` iteration; the next iteration
should pick up the first unchecked item below.

## Conventions

- Java package → Rust module (`geometry.planar` → `geometry::planar`), one Java
  class → one snake_case Rust file.
- Java's abstract `Point`/`Vector`/`Direction` hierarchies (Int / Rational
  BigInteger-backed variants) become plain structs for the Int variants first;
  a `Point`/`Vector` enum wrapper gets introduced when `RationalPoint` /
  `RationalVector` are ported (needed for exact line intersections).
- Where Java uses `double` for determinants of int coordinates (safe because
  coordinates are bounded by 2^25), the port uses exact `i64` math.
- Every ported file carries `//! Port of <JavaFile>` and unit tests.
- Verify with `cargo test` in `rust/` after each step.

## Phase 1 — geometry/planar foundations

- [x] Limits.java → `geometry/planar/limits.rs`
- [x] Side.java → `geometry/planar/side.rs`
- [x] datastructures/Signum.java → `datastructures/signum.rs`
- [x] IntVector.java → `geometry/planar/int_vector.rs`
- [x] IntPoint.java → `geometry/planar/int_point.rs` (core subset; IntBox/IntOctagon/Line-dependent methods pending)
- [x] FloatPoint.java → `geometry/planar/float_point.rs` (core subset)
- [ ] Direction.java / IntDirection.java / BigIntDirection.java
- [ ] RationalVector.java / RationalPoint.java (needs a big-integer or i128 rational type) + `Vector`/`Point` enum wrappers
- [ ] FloatLine.java
- [ ] Line.java
- [ ] LineSegment.java
- [ ] IntBox.java
- [ ] IntOctagon.java
- [ ] Polyline.java
- [ ] Shape hierarchy: Shape / ConvexShape / TileShape / RegularTileShape / Simplex / Circle / PolygonShape / PolylineShape / PolylineArea / Area / Ellipse
- [ ] Polygon.java
- [ ] datastructures/BigIntAux.java (as needed)

## Phase 2 — supporting infrastructure

- [ ] datastructures (ShapeTree / MinAreaTree / UndoableObjects / …)
- [ ] rules (nets, clearance matrix, via rules)
- [ ] board (items, traces, vias, search tree)

## Phase 3 — routing engines

- [ ] autoroute (maze expansion, batch autorouter, fanout, optimizer)

## Phase 4 — I/O and CLI

- [ ] io/specctra DSN parser + SES writer
- [ ] CLI entry point (headless batch routing first; no GUI planned)

## Notes / decisions log

- 2026-07-14: crate scaffolded on branch `rust`; no external deps yet.
