# Rust port — progress tracker

Incremental port of the Java sources (`src/main/java/app/freerouting`, 484 files)
to the `rust/` crate. Updated by each `/loop` iteration; the next iteration
should pick up the first unchecked item below.

## Status (as of iteration 69)

- **Working end to end**: DSN import (incl. pre-routed wiring) →
  expansion-room maze routing with in-search ripup → SES export.
  `cargo run --release --example route_board [board.dsn] [--strip-wiring]`.
- **Benchmarks** (WITH clearance compensation since iter 79):
  wavefolder 31/31, NormalPuzzle 71/72, J2_reference 23/24,
  interf_u 168/173 @ 300 s cap. Without clearance (iter 78) every
  fleet board reached 100%; the dip is the price of honest
  clearance-respecting routing and is to be won back via shove /
  search improvements. Pre-routed interf_u completes in 1.9 s.
- ~18k lines of Rust, 174 tests, no warnings; ~70 Java files ported.
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

- 2026-07-14 (iters 67–69): oversized-length root cause found with a
  per-trace stats dump (`examples/trace_stats.rs`): corners at exactly
  i32::MAX. Door-section midpoints along a straight corridor are
  collinear, `from_int_points` built consecutive PARALLEL lines from
  them, and the undefined corner intersection saturated on rounding.
  Fix: `from_int_points` drops equal and collinear-middle points (Java
  guarantees direction changes in `LocateFoundConnectionAlgo` instead);
  `door_shape` also simplifies its intersection like Java's
  `Simplex.intersection` (inert on this benchmark). Result: the
  saturated corners had been actively blocking free space, and the
  from-scratch benchmark jumped 165/173 → **173/173 (100%) in 123 s,
  0 failed**; total length now 4.85e7 (sane), SES 68.6 kB.

- 2026-07-14 (iter 70): first multi-board sweep (60 s limit each,
  from scratch, `--time-limit-s`): ecc83 13/13 (0.02 s);
  rpi_splitter 5/5; NormalPuzzle 70/72 (needed the no-"Via*"-padstack
  fallback); display-8-digit 29/30 (VCC); pic_programmer 109/111
  (1.2 s); 8088sbc 98/104; interf_u 173/173 —
  but wavefolder 17/31 and J2_reference 13/24 (see open issues).

- 2026-07-14 (iter 71): J2_reference instant failures root-caused with
  FR_DEBUG_MAZE + `examples/net_debug.rs`: pads imported UNROTATED, so
  the 350x1800 um fine-pitch pads of the 90-degree-rotated connector
  overlapped each other and every start-room completion came back
  empty. Fix: `TileShape::turn_90_degree` (box direct, otherwise via
  turned simplex border lines) and per-(padstack, quadrant) rotated
  padstack variants at import. J2_reference 13/24 → 23/24 in 0.7 s
  (GND still open); interf_u regression-clean at 173/173 in 112 s.
- 2026-07-14 (iter 72): CLI binary added per user directive (no GUI):
  `cargo run --release -- -de input.dsn [-do out.ses] [-mp passes]
  [-tl seconds] [--strip-wiring]`, Java-jar-compatible -de/-do flags,
  exit code 0 = all nets complete / 2 = incomplete / 1 = error. The
  DSN `resolution` is now stored on the board and used by the SES
  export (was hardcoded to 10 in the examples).

## Open issues

- 2026-07-14 (iter 79): clearance compensation added — the router no
  longer produces zero-clearance copper. DSN typed clearance rules
  ((clearance V (type smd_smd)) etc.) now populate the matrix classes
  null/default/smd; single-layer (SMD) padstacks get class smd; room
  completion inflates obstacle shapes by the pairwise clearance to the
  routed trace's class (the door shrink by the trace half width then
  keeps copper edges `clearance` apart). Completion cost: interf_u
  168/173 @ 300 s (was 173), J2 23/24, NormalPuzzle 71/72, wavefolder
  still 31/31. Known gap: via pads are placed with trace-width rooms,
  so vias can still violate clearance (needs via-aware compensation).
- 2026-07-14 (iter 80): via placement is clearance-aware too —
  via_free enlarges the via pad by the largest clearance of the trace
  class (was: by the trace half width) before the blocking check.
  Small boards unchanged (wavefolder 31/31, J2 23/24, NormalPuzzle
  71/72); interf_u 164/173 @ 300 s hard cap — now clearly
  TIME-limited (each search costs more under clearance). Priority for
  completion is therefore performance (octagon-specialized restrain,
  fewer redundant-line simplifications) and then shove.
- 2026-07-14 (iter 81): removed the gcd normalization from the two
  hottest exact-arithmetic paths (profile: Line::cmp 1310 +
  remove_redundant_lines 1728 samples): Line's angular Ord now
  compares raw i64 difference vectors (scale-invariant, identical
  order), and remove_redundant_lines uses a raw-vector determinant
  sign instead of building IntDirections. Wavefolder 6.8 s → 2.8 s;
  interf_u 165/173 @ 300 s (still time-saturated — the restart
  fallback consumes remaining budget).
- 2026-07-14 (iter 82): allocation/merge micro-optimizations —
  Simplex::intersection merges the two (already sorted) line arrays
  instead of re-sorting, and item.tile_shapes returns the cached slice
  instead of cloning a Vec of shapes per call. Wavefolder 2.7 s;
  interf_u unchanged (165/173 @ 300 s). Micro-optimization is now
  exhausted; the remaining levers are algorithmic: reuse expansion
  rooms across connections with incremental invalidation (Java keeps
  the room graph and only removes rooms touching changed items — ours
  rebuilds from scratch per connection), octagon-specialized restrain,
  and shove.
- 2026-07-14 (iter 83): naive engine reuse across a net's connections
  tried and REVERTED: complete_room restrains each new room against
  ALL accumulated rooms, so the reused graph grows quadratically on
  many-pin nets — interf_u routed FEWER connections in 300 s (153 vs
  175), wavefolder slowed 2.7→3.6 s, J2 varied. NormalPuzzle alone
  gained (2x). Do not retry without first porting incremental
  invalidation + the SortedRoomNeighbours door algorithm (Java
  completes rooms against neighbours, not the whole graph). The
  maze_route_with_engine / register_new_targets API is kept dormant
  for that future port.
- 2026-07-15 (iter 95): shove arc piece 3b: board-level cutout_trace
  ported (board/shape_trace_entries.rs) — removes the part of a trace
  inside a shape (enlarged by half width + clearance + 1, in two steps
  like Java) and reinserts the outside pieces. NOT yet wired into
  ripup: dangling stub endpoints would fight the exact-endpoint trace
  contacts; in shove proper the shoved substitute reconnects the stubs
  by construction. Remaining for ShapeTraceEntries: the EntryPoint
  border bookkeeping (store_trace / calculate_stack_levels /
  pop_piece), needed by ShoveTraceAlgo's check.
- 2026-07-15 (iter 94): shove arc piece 3a: TileShape::entrance_points
  + cutout_polyline ported (the previously deferred
  Polyline-dependent TileShape methods) — cut the parts of a polyline
  inside a shape, keeping the outside pieces closed by the entered
  border lines. This unblocks ShapeTraceEntries::cutout_trace
  (piece-wise trace ripup instead of whole-item removal) next.
- 2026-07-15 (iter 93): shove arc piece 2: CalcShapeAndFromSide ported
  (board/calc_shape_and_from_side.rs) — cuts the dog ears off a trace
  segment shape at the trace ends and derives the from-side for
  pushing; takes the polyline + compensated half width + the segment's
  search-tree shape (the Java version reads those from the board).
  Next: ShapeTraceEntries (the 793-line workhorse).
- 2026-07-15 (iter 92): SHOVE ARC STARTED. Plan (Java, 2666 lines
  total): CalcFromSide (125) -> CalcShapeAndFromSide (117) ->
  ShapeTraceEntries (793, the shove workhorse: collects and cuts the
  traces/vias in a shove shape) -> ShoveTraceAlgo (830,
  check + insert with recursive pushing) -> ForcedViaAlgo (306);
  integrate as a fallback when a maze connection fails: try inserting
  the blocked segment with shoving before giving up. This iteration:
  board/calc_from_side.rs ported with tests (entry side of a polyline
  / nearest side of a point / shove sides of a segment; note; on
  4-sided shapes both shove directions coincide, like Java's +-2 mod
  border count).
- 2026-07-15 (iter 91): all remaining interf_u failures route fine
  ALONE (probe: /PC-A0, /MA11, GND, VCC each complete on the empty
  board) — pure ordering congestion, not geometry. Iterated restart
  rounds (failures-first, rotated, monotonic keep-if-better) tried:
  no additional nets won on interf_u / NormalPuzzle / J2, so rounds
  now stop after the first non-improving attempt. J2's GND is the
  one net that fails even failed-first under clearance (pre-clearance
  the restart won it) — a true shove candidate. Conclusion recorded:
  ordering tricks are exhausted; the remaining nets need shove or
  substantially more search throughput.
- 2026-07-15 (iter 90): restrain_shape converts the obstacle to a
  simplex once and shares it through the recursion (previously it
  cloned the simplex per border line in two loops and re-converted on
  every recursion step). Behaviour-neutral (interf_u bit-identical);
  cap-bound coldfire runs vary a few nets run-to-run with wall-clock
  deadlines — treat single-run deltas under ~5 nets as noise.
- 2026-07-15 (iter 89): bounding-box pre-filters in the three restrain
  / door loops (obstacle x piece, piece x existing room, door touch
  tests) skip the exact simplex intersections for separated pairs.
  wavefolder 1.6 s (was 2.2 s), interf_u 167/173 (+1), coldfire
  246/278 (+5) — more connections routed inside the same 300 s cap.
- 2026-07-15 (iter 88): Line::side_of_intersection's exact fallback
  now computes in i128 instead of BigInt rationals (intersection
  numerators < 2^82, side determinant < 2^111 for i32 coordinates);
  bit-identical results on interf_u and coldfire.
- 2026-07-15 (iter 87): asymmetric-contact bug fixed — the
  pad-containment contact (iter 74) was one-directional (A contacts B
  when A's center lies in B's shape, but B's scan never checked A's
  center against its own shapes), so an item could appear in SEVERAL
  connected components and route_net then tried to "connect" an item
  to itself forever. Found via a duplicate-component detector in
  net_components (FR_DEBUG_MAZE). The via containment scan now also
  reports drill items whose center lies inside the pad shape, making
  the relation symmetric. Coldfire GND routes fully alone (77/77);
  coldfire 241/278 @ 300 s, interf_u 166/173 (+1), wavefolder 31/31.
- 2026-07-15 (iter 86): power planes imported as conduction areas —
  (plane NET (polygon LAYER ...)) becomes a net-carrying conduction
  ObstacleArea; conduction areas do not restrain rooms or block vias
  (planes get fabrication cutouts, Java: ConductionArea), and pins /
  trace endpoints / via centers inside the plane on a shared layer
  count as contacts. First 4-layer board routes: coldfire-xilinx
  (278 nets, 4 layers) imports and reaches ~224/278 in 120 s; its GND
  plane merges 126 of GND's 130 items. Wavefolder improves to 31/31
  with 63 connections (the F.Cu GND plane pre-connects GND); rest of
  the fleet unchanged. Remaining on coldfire: one GND connection
  fails even alone (open), and the signal tail needs more time.
- 2026-07-15 (iter 85): net class rules imported and obeyed — (class
  NAME nets... (circuit (use_via V)) (rule (width W))) now populates
  NetClasses/ViaInfos/ViaRules, nets get their class, and the batch
  router overrides trace half width and via padstack per net
  (request_for_net). Wavefolder's Power nets route at 400 um with the
  800:400 via (SES shows both 2500 and 4000 widths). A class listing
  no nets updates the default class. Fleet: wavefolder 31/31,
  8088sbc 104/104, interf_u 165/173, J2 23/24 unchanged; NormalPuzzle
  69/72 (was 71 — its classes demand 10 mil traces and a specific via,
  which we previously ignored; honest-rules result).
- 2026-07-15 (iter 84): wall-clock budget split — the normal passes
  now get 70% of the time limit, reserving the rest for the restart
  fallback (previously the passes consumed everything and the fallback
  often never ran under pressure). Completion unchanged everywhere;
  8088sbc 172 s → 107 s and NormalPuzzle 23 s → 11 s (the fallback
  resolves stragglers more efficiently than late high-budget passes);
  interf_u neutral at 165/173.
- 2026-07-14 (iter 78): FULL FLEET AT 100%. J2's GND routed fully when
  alone → pure ordering congestion (largest-extent nets route last
  into consumed corridors). A global largest-first order fixed J2 and
  8088sbc but broke interf_u (134/173) — no static order wins. Fix: a
  transactional RESTART FALLBACK after the normal passes when nets
  remain incomplete: snapshot, rip up all route items, route the
  failed nets FIRST, keep only if strictly more nets complete.
  Results: NormalPuzzle 70→72/72, 8088sbc 100→104/104, J2 23→24/24,
  interf_u/wavefolder unchanged at 100%. (FR_ROUTE_ORDER_DESC env
  kept for ordering experiments.)
- 2026-07-14 (iter 77): wavefolder fully solved — 31/31 in 3.3 s. The
  last two nets failed because back-side placement used the wrong flip
  style: Java (and the specctra default) mirrors pin offsets and pad
  shapes at the y axis BEFORE the component rotation; we rotated
  first. An axial diode pin thereby landed 4.7 mm outside the board
  outline. Import now follows the default and honours
  (place_control (flip_style rotate_first)). Also: when start-room
  creation returns empty because earlier expansion already covered
  the pad, the existing containing rooms now serve as start rooms.
  Fleet after the fix (120 s cap): display-8-digit 30/30 (was 29),
  pic_programmer 111/111 in 2.6 s (was 109 in 60 s), wavefolder
  31/31, ecc83 13/13, rpi_splitter 5/5, J2_reference 23/24,
  NormalPuzzle 70/72, 8088sbc 100/104, interf_u 169/173@120s cap
  (100% needs ~185 s).
- Issue153-wavefolder 17/31 RESOLVED → 29/31 (iter 74): the real bug
  was contact detection, not reachability. The staggered TO-92 pads
  have off-centre shapes ([0,±400] um), the maze ends traces at the
  pad shape's centre of gravity, and trace-to-pin contact required
  exact equality with the PIN CENTER — so routed traces never counted
  as connections and route_net's no-progress guard reported failure.
  Fix: trace-to-pin contact by shape containment (both directions in
  get_normal_contacts*), like Java. Two boards' worth of earlier
  "bit-identical after fix" confusion had a second cause: `cargo build
  --release` does NOT rebuild examples, so several benchmark runs used
  a stale route_board binary — always build with `--examples` (or
  `--example route_board`) before benchmarking.
- interf_u after the contact fix: still 173/173, but ~253 s (was
  ~112 s). Worktree bisect attributes the ENTIRE slowdown to the
  oval-pads-as-octagons import (commit 3bf05067; ~150 pads now have 8
  border lines instead of 4 in every room restrain) — the contact fix
  itself costs nothing here (bit-identical output). A padstack
  bounding-box pre-filter was added to the containment check anyway
  (cheap, correct). Future optimization lever: octagon-specialized
  restrain/intersection paths like Java's ShapeSearchTree instead of
  the unconditional Simplex conversion in restrain_shape.
- 2026-07-14 (iter 76): profiled with /usr/bin/sample —
  Simplex::remove_redundant_lines + Line::cmp sorting ≈ 55% of
  runtime. Two behaviour-neutral caches (bit-identical outputs):
  door section segments cached per (door, offset), and item tile
  shapes computed once per item (OnceCell; items are immutable once
  inserted; the shape cache is excluded from Item's PartialEq).
  interf_u 256 s → 185 s, wavefolder finishes in 53 s. Remaining
  remove_redundant_lines load comes from offset_shapes at insertion
  and room-completion intersections.
- Issue026-J2_reference GND (22 pins) still incomplete after the
  rotation fix.
- Non-quarter-turn component rotations still only rotate pin offsets,
  not pad shapes; back-side placement still mirrors offsets without
  mirroring pad shapes or flipping their layers.

## Notes / decisions log

- 2026-07-14: crate scaffolded on branch `rust`; no external deps yet.
