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
- [x] LineSegment.java → `geometry/planar/line_segment.rs` (endpoints, to_polyline/to_simplex, contains, segment intersection/overlap, stair approximations — Java's function_value_approx-in-y suspect preserved, border_intersections, sort; also closes Polyline offset_box/contains/projection_line)
- [x] IntBox.java → `geometry/planar/int_box.rs` (box/box ops, cutout, divide, compare; also closes Point::surrounding_box / is_contained_in gaps; IntOctagon/Simplex/TileShape interop pending)
- [x] IntOctagon.java (+ FortyfiveDegreeDirection.java) → `geometry/planar/int_octagon.rs` (normalize, intersection/union/contains, border points/projections, both cutout variants; also adds IntBox::to_int_octagon/enlarge, IntPoint::surrounding_octagon; Simplex/TileShape interop pending)
- [x] Polygon.java → `geometry/planar/polygon.rs` (dedup + collinear removal; winding_number_after_closing pending)
- [x] Polyline.java → `geometry/planar/polyline.rs` (constructors incl. line normalization, corners, reverse/combine/split/skip/shorten, offset_shapes with dog-ear cutting, bounding box/octagon, nearest point; offset_box/contains/projection_line pending on LineSegment)
- [x] Simplex.java → `geometry/planar/simplex.rs` (complete incl. cutout_from + calc_division_lines; Java's never-assigned prev_division_line preserved as documented dead branch)
- [x] TileShape.java / RegularTileShape.java dispatch + generic algorithms → `geometry/planar/tile_shape.rs` (enum Box/Octagon/Simplex; get_instance+simplify, containment family, nearest points/borders, intersection kind-promotion, 3x3 cutout dispatch; index_of_nearest_corner fixes Java's Double.MIN_VALUE init bug; Polyline/LineSegment-dependent methods pending)
- [x] Circle.java → `geometry/planar/circle.rs` (containment/distances, bounding octagon + tangent-line bounding tile, transforms, intersections)
- [x] PolygonShape.java → `geometry/planar/polygon_shape.rs` (ccw normalization, convexity/hull/bounding tile, split_to_convex with axis-parallel division points; area() fixes Java's dimension()<=2 always-zero bug; + Polygon::winding_number_after_closing)
- [x] PolylineArea.java / Area.java → `geometry/planar/area.rs` (border + holes, split_to_convex with hole cutout, containment, transforms)
- [ ] Remaining geometry (on demand): Ellipse; TileShape methods needing Polyline/LineSegment (entrance_points, cutout(Polyline), is_intersected_interior_by, touching_sides, …)
- [ ] Polygon.java

## Phase 2 — supporting infrastructure

- [x] datastructures/ShapeTree.java + MinAreaTree.java → `datastructures/min_area_tree.rs` (arena-indexed generic tree; IntOctagon bounds subsume both ShapeBoundingDirections variants; ArrayStack replaced by Vec)
- [x] datastructures/UndoableObjects.java → `undoable_objects.rs` (explicit key/value split, arena version chains; pop_snapshot splices same-level undo versions — documented deviation fixing a Java edge-case that loses objects)
- [x] datastructures/Stoppable.java + TimeLimit.java → `stoppable.rs`
- [ ] datastructures remaining: PlanarDelaunayTriangulation (autoroute-time), IdentifierType/IndentFileWriter (specctra I/O time)
- [x] board/Layer.java + LayerStructure.java → `board/layer.rs`
- [x] rules/ClearanceMatrix.java → `rules/clearance_matrix.rs` (even-rounded values, per-row/layer maxima, append/remove class, safety margin)
- [x] rules/Net.java + Nets.java → `rules/net.rs` (rules data; board item queries follow with the board item model)
- [x] rules/NetClass.java + NetClasses.java + DefaultItemClearanceClasses.java → `rules/net_class.rs` (object refs → indices)
- [x] core/Padstack.java + Padstacks.java → `core/padstack.rs` (Option<TileShape> per layer, drill-radius name parsing with cache, trace exit directions)
- [x] rules/ViaInfo.java + ViaInfos.java + ViaRule.java → `rules/via_rule.rs` (object refs → ViaInfoId/padstack numbers)
- [x] rules/BoardRules.java (+ board/AngleRestriction.java) → `rules/board_rules.rs` (rules aggregate; item-touching clearance maintenance split into rules-side methods, board handles its items)

rules package complete (except GUI print_info methods, intentionally out of scope).
- [x] board item model foundation → `board/item.rs` (FixedState, ItemBase, Item enum with Via + PolylineTrace incl. search-tree shape computation; Pin/areas/outline and board-dependent logic pending)
- [x] BasicBoard core → `board/basic_board.rs` (UndoableObjects item store + MinAreaTree integration, insert/remove trace+via, layer-filtered exact overlap queries, net filtering/blocking, undo/redo with tree resync)
- [x] connectivity → `board/basic_board.rs` (get_normal_contacts at trace corners / drill centers, start/end contacts, is_tail, connected sets, net completeness)
- [x] ObstacleArea/ConductionArea items → `board/item.rs` ItemKind::ObstacleArea (resolved PolylineArea + layer + is_conduction; tree shapes via split_to_convex; symmetric conduction contacts in basic_board)
- [ ] board remaining: Pin item (needs component model), BoardOutline item, clearance-compensated ShapeSearchTree variants, RoutingBoard, shove/pull-tight algorithms

## Phase 3 — routing engines

- [x] interim grid A* router → `autoroute/simple_router.rs` (NOT a Java port: stand-in so the pipeline routes end to end; uses exact board obstacle queries, inserts polyline traces + layer-change vias)
- [x] expansion-room object model → `autoroute/expansion_room.rs` (ExpansionRoom/Door/MazeSearchElement as arena RoomGraph; door section segmentation incl. 2-dim restraint lines; + TileShape::diagonal_corner_segment)
- [x] free-space room completion → `autoroute/room_completion.rs` (ShapeSearchTree.complete_shape + restrain_shape with deterministic obstacle order; + TileShape distance_to_the_left / side_of_line / is_intersected_interior_by / half_plane; divide_large_room pending)
- [x] AutorouteEngine core → `autoroute/engine.rs` (per-net room graph; complete rooms restrained against board + existing rooms; doors to touching rooms; target doors to own-net items; lazy frontier expansion per border edge — simplified vs SortedRoomNeighbours' sorted-edge-gap algorithm, documented)
- [x] maze search core → `autoroute/maze_search.rs` (Dijkstra over door sections + drill steps with backtrack-node arena; multi-layer via expansion at entry locations — simplification of DrillPage candidate generation, documented; per-layer trace + via insertion; no ripup/shove yet)
- [x] batch autorouter loop → `autoroute/batch.rs` (per-net component analysis, closest-pair incompletes preferring drill endpoints — trace splitting at junctions not yet ported, no-progress guard, single pass without ripup escalation)
- [x] trace splitting at junctions → `BasicBoard::split_traces_at` (+ maze_route normalizes inserted endpoints; T-junction contacts now register)
- [x] trace combining at simple joints → `BasicBoard::combine_trace` (PolylineTrace.combine: exactly-one-trace contact with equal layer/width/nets merges via Polyline::combine)
- [ ] faithful autoroute port remaining: SortedRoomNeighbours faithful door/gap algorithm, clearance compensation, DrillPage-based drill candidates, ripup/shove + pass escalation, remaining normalization (overlap/cycle removal), faithful Locate/InsertFoundConnectionAlgo corner calculation, fanout, optimizer

## Phase 4 — I/O and CLI

- [x] S-expression reader → `io/dsn.rs` (tokenizer/tree with quoted strings incl. the `(string_quote ")` special case; navigation helpers; verified against real repo fixtures)
- [ ] DSN semantic import (structure/library/placement/network → BasicBoard), SES writer
- [ ] CLI entry point (headless batch routing first; no GUI planned)

## Notes / decisions log

- 2026-07-14: crate scaffolded on branch `rust`; no external deps yet.
