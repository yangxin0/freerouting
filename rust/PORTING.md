# Rust port — progress tracker

Incremental port of the Java sources (`src/main/java/app/freerouting`, 484 files)
to the `rust/` crate. Updated by each `/loop` iteration; the next iteration
should pick up the first unchecked item below.

## Status (as of iteration 53)

- **Working end to end**: DSN import (incl. pre-routed wiring) →
  expansion-room maze routing with in-search ripup → SES export.
  `cargo run --release --example route_board [board.dsn] [--strip-wiring]`.
- **Benchmark** (interf_u, 173 nets, 2 layers): 96% from scratch in 142 s;
  100% completion of the pre-routed board in 1.9 s. See the benchmark log.
- ~18k lines of Rust, 171 tests, no warnings; ~70 Java files ported.
- 6 upstream Java bugs found and documented (see the notes/decisions log
  and code comments marked "deviation").
- Main gaps vs Java: shove algorithms, pull-tight optimizer, fanout,
  faithful SortedRoomNeighbours door algorithm, 45°/90° restricted modes,
  GUI (out of scope), rules/SES fidelity details.

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
- [x] DSN semantic import → `io/dsn_import.rs` (layers by index, resolution scaling, default rules, padstacks — circle/rect/path/polygon as box/octagon approximations, images/placement with rotation + back-side mirroring, network pin binding; wiring/keepout import + faithful pad shapes pending)
- [x] SES session writer → `io/ses_export.rs` (network_out wires + autoroute vias; validated by re-parsing)
- [x] end-to-end integration → `tests/route_fixture.rs` (import interf_u fixture → route /ACK → export session with the routed wire)
- [x] boundary import as outline keepout strips (BoardOutline tree-shape equivalent; routes verified to stay inside the outline bbox)
- [ ] DSN wiring/keepout-area import, richer SES fidelity (library_out, session padstack forms)
- [ ] CLI entry point (headless batch routing first; no GUI planned)

## Benchmark log

- 2026-07-14 (iter 42): first full-board run, interf_u fixture (173 nets,
  395 pins, 2 layers): import 1.5 ms; single pass, no ripup, A*-guided,
  100k expansion budget → 131 connections routed, 145/173 nets complete
  (84%) in 31 s. Incompletes: dense buses (/MA*, /PC-A*) + GND/VCC.
  Next lever: ripup escalation passes (Java reaches 100% with them).
- 2026-07-14 (iter 44): 3 passes, shortest-extent-first ordering, budget
  doubling per pass → 156/173 nets complete (90%) in 93 s. The remaining
  17 incompletes are congestion cases needing real ripup.
- 2026-07-14 (iter 46): naive corridor ripup REGRESSED to 117/173 —
  destructive rips without proof of benefit. Lesson recorded.
- 2026-07-14 (iter 48): transactional ripup (snapshot; commit only if the
  failed net and all victims recover) → 158/173 (91%) in 140 s;
  monotonicity guarantee held. Remaining: bus/power congestion needing
  in-search ripup costs (Java's approach) or shove.
- 2026-07-14 (iter 50): in-search ripup costs (rooms overlap rippable
  items, maze pays per-item penalty, precise geometry-intersection rips,
  transactional victim reroute). Partial-completion scenario (imported
  pre-routed wiring): 173/173 nets (100%) in 1.9 s.
- 2026-07-14 (iter 52): from-scratch with in-search ripup: 163/173 (94%)
  in 150 s. Progression: 84% → 90% → 91% → 94%. Remaining: /PC-A* bus
  tail, /MA12, /OE-, GND, VCC. Batch passes now support a wall-clock
  TimeLimit (demo bounds runs to 5 min).
- 2026-07-14 (iter 54): grid-sampled drill candidates within rooms
  (DrillPage-style) → 166/173 (96%) in 142 s; VCC completes. Remaining:
  6 /PC-A* nets + GND. Progression: 84 → 90 → 91 → 94 → 96%.
- 2026-07-14 (iters 55–59): cascading-ripup experiment — bounded victim
  cascades consistently regressed to 161/173 by time starvation even
  with deadline discipline; REVERTED to simple victim reroute. Deadline
  checks kept (inside searches, between connections, per victim).
  MST-style closest-component merging kept (benign: 165/173 in 140 s
  after revert, within run variance of the 166 baseline). Conclusion:
  further completion gains need shove, not more ripup tuning.
- 2026-07-14 (iters 63–66): quasi-infinite-corner fix saga. Pull-tight
  length report exposed traces with corners far outside CRIT_INT (total
  length ~85e9). Fix 1: sanitize nearly parallel adjacent lines in
  `Polyline::from_lines`. That first (a) stack-overflowed the benchmark
  via unbounded `restrain_shape` recursion on degenerate slivers (fixed
  with a depth cap of 64), then (b) REGRESSED to 154/173 in 236 s:
  dropping one of the two first/last lines moves an end corner, and
  trace connectivity is exact-endpoint-equality, so routed connections
  stopped registering — HYPOTHESIS REFUTED: restricting drops to
  endpoint-safe interior lines (indices 2..=len-3, kept as a safety
  invariant) reproduced the 154/173 run bit-for-bit; a further bisect
  run with sanitation disabled was ALSO bit-for-bit identical, proving
  the sanitation never fires on this board at all. The real culprit was
  the untested commit in between: "Pull traces tight between batch
  passes" (3001c937) — tightened traces hug obstacles and produce
  degenerate shapes that poison room completion (also the source of the
  stack overflow). REVERTED; depth cap kept as defence. Post-revert:
  165/173 in 141 s, bit-for-bit match of the a891d790 baseline. Lesson:
  benchmark every routing-behaviour commit individually.

## Open issues

- Total trace length before the final pull-tight reads ~85e9 board
  units (~50x geometric expectation); pull-tight reduces it to ~9.4e9.
  The from_lines sanitation never fires on interf_u, so the oversized
  corners enter through another path (split/combine or the length
  accounting itself). Harmless to routing results, but worth
  root-causing before trusting length-based reports.

## Notes / decisions log

- 2026-07-14: crate scaffolded on branch `rust`; no external deps yet.
