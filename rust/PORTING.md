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
- [x] Direction.java / IntDirection.java → `geometry/planar/int_direction.rs` (coords gcd-normalized at construction so derived Eq matches Java's equivalence-class equals; BigIntDirection deferred to the rational layer)
- [x] RationalVector.java / RationalPoint.java → `rational_vector.rs` / `rational_point.rs` (num-bigint; RationalPoint's IntBox/Line-dependent methods pending)
- [x] Vector.java / Point.java → `vector.rs` / `point.rs` (`Vector`/`Point` enums over Int/Rational; get_instance_big fixes Java's double `p_x.mod(p_z)` check; Point's IntBox/IntOctagon/Line methods pending)
- [x] BigIntDirection.java + Direction dispatch → `direction.rs` (`Direction` enum; turn_45_degree unimplemented for Big, like Java)
- [x] datastructures/BigIntAux.java → `datastructures/big_int_aux.rs` (determinant, add_rational_coordinates; binaryGcd replaced by plain gcd in int_vector.rs)
- [x] FloatLine.java → `geometry/planar/float_line.rs` (+ FloatPoint helpers: side_of, 3-point scalar_product, rotate, turn_90_degree(_around), is_contained_in_box)
- [x] Line.java → `geometry/planar/line.rs` (IntPoint endpoints — Java warns+casts to IntPoint everywhere anyway; includes IntPoint/RationalPoint perpendicular_projection, side_of_line; TileShape-based is_on_the_left/right pending)
- [ ] LineSegment.java
- [x] IntBox.java → `geometry/planar/int_box.rs` (box/box ops, cutout, divide, compare; also closes Point::surrounding_box / is_contained_in gaps; IntOctagon/Simplex/TileShape interop pending)
- [x] IntOctagon.java (+ FortyfiveDegreeDirection.java) → `geometry/planar/int_octagon.rs` (normalize, intersection/union/contains, border points/projections, both cutout variants; also adds IntBox::to_int_octagon/enlarge, IntPoint::surrounding_octagon; Simplex/TileShape interop pending)
- [ ] Polyline.java
- [x] Simplex.java core → `geometry/planar/simplex.rs` (get_instance/remove_redundant_lines, corners, dimension, containment from TileShape.java, intersection, bounds, offset/enlarge, to/from IntBox+IntOctagon; cutout_from + calc_division_lines pending)
- [ ] Shape hierarchy: Shape / ConvexShape / TileShape enum / RegularTileShape / Circle / PolygonShape / PolylineShape / PolylineArea / Area / Ellipse; Simplex::cutout_from + calc_division_lines
- [ ] Polygon.java

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
