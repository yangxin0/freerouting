# Rust port — progress tracker

Incremental port of the Java sources (`src/main/java/app/freerouting`, 484 files)
to the `rust/` crate. Updated by each `/loop` iteration; the next iteration
should pick up the first unchecked item below.

> **Authoritative status: see "Second-round audit (2026-07-16)" below — the
> remediation is PARTIAL, not complete.** The per-iteration sections that
> follow are HISTORICAL and contain claims that later parity audits corrected.
> A first remediation round fixed a batch of findings (maze rip-up no longer
> deletes pins; the equivalence test snapshots pins and is layer-aware; DSN
> cross-class clearance max + application to pins/vias/planes/fanout; DRC
> netless copper / matrix order / hole-clearance labels / unit-independent
> tolerance; KiCad/rules/SES unit handling) and a second round fixed the
> KiCad duplicate-default round-trip and the optimizer's local-DRC/metric
> parity. A third and fourth round then closed the rest of the audit's
> functional findings: image keepouts, net-class SMD clearance, DRC same-net /
> conduction, the single-thread optimizer, J2 routing completeness (now 24/24),
> and the named-clearance-class grammar. What remains OPEN is narrower — see the
> "Second-round audit" section: router PERFORMANCE vs Java (not correctness),
> and two non-clearance features (named via-rule registry, autoroute
> `layer_rule`). Do not read the historical sections as current truth.

## Status (as of iteration 118)

- **Working end to end**: DSN import (planes, net classes, back-side /
  rotated placement, multi-layer up to 6 layers, 500+ net designs) →
  expansion-room maze routing with clearance HALF-compensation,
  layer-aware A*, in-search ripup, trace shoving (ordered forced
  insertion) and a transactional restart fallback → trace
  normalization → pull-tight → SES export. CLI:
  `cargo run --release -- -de input.dsn [-do out.ses] [-mp passes]
  [-tl seconds]` (Java-compatible flags, exit codes 0/2/1).
- **Fleet** (from scratch, clearance-honest, 300 s cap): NINE of ten
  boards at 100% — interf_u 173/173 in 112 s, 8088sbc 104/104,
  pic_programmer 111/111 (0.8 s), display 30/30 (8 s), wavefolder
  31/31 (1.2 s), NormalPuzzle 72/72 (16 s), J2 24/24 (0.4 s), ecc83
  13/13, rpi_splitter 5/5. coldfire-xilinx (4 layers) 263/278 at
  300 s / 269 at 600 s — purely throughput-bound (all holdouts route
  alone). Extended sweep: 8 more boards (6-layer CM5, 529-net Z80,
  DAC2020 benchmark...) import and route without a single crash.
  Pre-routed interf_u verifies in 22 ms.
- ~21k lines of Rust, 188 tests, no warnings; ~80 Java files ported.
- KEY ARCHITECTURE LESSONS: clearance half-compensation (full
  inflation preserves separations but destroys room topology); planes
  outside the search tree (board-covering bounds poison the R-tree);
  inadmissible distance-to-center A* guides better than admissible
  variants; passes unlimited under wall clock. Negative-results
  ledger in the log below — consult before re-attempting reverted
  ideas.
- Main gaps vs Java: room reuse + SortedRoomNeighbours (coldfire
  throughput), via shoving (MoveDrillItemAlgo), distinct-net shove
  stacking, the optimizer beyond pull-tight, fanout, 45/90-degree
  modes, GUI (out of scope per user directive — CLI is the
  deliverable).

## Audit remediation (2026-07-16)

Two rounds of independent parity audit against the Java sources. The status
below reflects the SECOND round, which correctly caught that the first
remediation over-claimed (a broken electrical-equivalence fix with a vacuous
test, an inert per-net-clearance change for DSN, an over-stated ratsnest
complexity). This section is the honest reconciled state — genuine fixes, plus
an explicit list of what is NOT fixed. Fixes landed (with regression tests):

- **DRC false negatives (confirmed)** — `drc.rs` now includes obstacles/keepouts
  in the outer iteration and sizes the candidate search by the layer's max
  clearance instead of a fixed 10 000 units, so copper-vs-earlier-obstacle
  pairs are checked. Same fixed-radius bug fixed in the optimizer's local DRC.
- **CLI/settings (confirmed)** — `-mp 0` = unlimited passes; defaults aligned
  to Java (maxPasses 9999, threads = cores−1); routing + optimization share one
  `-tl` budget; exit status recomputed after optimization. (The nine-source
  settings *merger* itself is still not ported.)
- **Units/geometry (confirmed)** — the DSN unit is preserved through to SES
  export (a `mil` design no longer exports as `um`); rect board outlines now
  produce confining keepouts. (Window/hole parsing, exact non-box shape
  fidelity, and arbitrary named clearance classes remain approximated.)
- **Via move disconnects traces (confirmed)** — `move_via` now bridges the
  contacting traces from the old to the new center (Java: `DrillItem.move_by`).
- **Optimizer acceptance (confirmed)** — gated on board-wide incomplete count,
  not just the target net, so a reroute that breaks another net is rejected.
- **Optimizer acceptance, tightened** — a second audit noted a bare
  incomplete-net *count* still permits a one-for-one swap (target completes, a
  victim breaks). The gate now checks the completeness *set*: a step is rejected
  if any previously-complete net becomes incomplete.
- **Via move dedup** — bridge stubs are deduplicated by (layer, width, class) so
  coincident duplicates are not inserted. (The bridge is still a straight
  segment; Java inserts a 45/90-degree corner when required — still TODO.)
- **Split under via (confirmed, partial)** — the router normalize pass splits
  same-net traces under a newly inserted via across its spanned layers, and now
  loops so *every* matching trace at the point is split. NOTE this is only the
  maze-insertion path; `insert_via` itself is still raw, so SES import and other
  callers do not split — still TODO.
- **Tests** — added CLI end-to-end, SES-import round-trip (now asserts the reload
  reproduces ≥90% of routed connectivity, not just "≥1 net"), and full-board DRC
  integration tests; the real-board fixture test hard-fails when the fixture is
  absent. 291 Java test methods remain unported.
- **Unit correctness, partial** — the SES unit label is preserved, and DRC and
  scoring now convert coordinates to mm via a unit-aware `board_units_per_mm()`
  (a `mil` fixture no longer reports lengths ~25x too large). The KiCad JSON
  writer keeps its own separate board-units-per-mm convention and is NOT yet
  unit-correct for DSN-imported boards — still TODO.

Also fixed in the third round:

- **DRC report dedup** — one violation per (item pair, layer); a multi-shape
  padstack no longer reports the same breach several times.
- **DRC/scoring/ratsnest unit-aware mm** — the ratsnest JSON now uses
  `board_units_per_mm` too (KiCad + rules writers still on the raw-resolution
  convention — TODO).
- **Integration assertions strengthened** — the full-board DRC test now asserts
  J2 reports zero clearance violations (not just zero unconnected).

Electrical equivalence (finding #2) — FIXED for plane-free boards, verified:

- The maze search now terminates connections at the pin CONNECTION POINT (drill
  center). `find_connection` wraps the search and corrects both endpoints via
  `pin_exit_corner`: when an endpoint lands inside a same-net pad off-centre, a
  corner extends to the center. The segment stays inside the convex same-net pad,
  so it crosses no foreign clearance and `trace_run_is_clear` (which skips
  same-net items) accepts it — modelling the pin exit as part of the legal route,
  not a post-hoc stub. Applied on the search result so BOTH insert paths (the
  inline `maze_route_with_engine` and `insert_connection`) benefit.
- `remove_if_cycle` no longer deletes the SOLE wire reaching a pin center. The
  cycle test uses the lenient in-pad containment rule, so it treated a pin as
  already connected via a nearby off-centre end and removed the wire that
  actually reached the center; it is now protected when no other same-net wire
  sits on that point (a redundant detour merely touching a pin is still removed —
  `cycle_traces_are_removed` still passes).
- Verified by a strict, enabled regression test
  (`routed_nets_reach_pin_connection_points`): a union-find oracle joins
  connection points only through exact wire-endpoint coincidence (the Java-reload
  criterion), restricted to >=2-real-pin complete nets, on the plane-free SMD and
  J2 fixtures. Both now have every real pin strictly wired.
- SCOPE: the oracle does not model conduction-area (plane) connections, so
  boards with GND/power planes (pic_programmer, wavefolder) are not yet asserted;
  a plane-aware oracle is future work. No fleet completion or violation
  regression.

Per-net clearance classes (finding #4) — FIXED for DSN:
- DSN import no longer hardcodes clearance class 1 for every net class. Each
  distinct net-class clearance value becomes its own dynamic clearance-matrix
  class (`append_class`), and the net class is assigned it; classes whose
  clearance equals the board default reuse class 1, so single-class boards are
  unchanged. `(clear ...)` is handled as the `(clearance ...)` alias. Pre-routed
  DSN wiring and SES-imported wires/vias now propagate their net's clearance
  class instead of the default. Verified by a self-contained test (a board with
  a larger "hv" class gets a distinct class carrying its spacing, and its
  wiring inherits it) and on real multi-clearance fixtures (Issue015 gains 2
  dynamic classes). No fleet regression. Still using a per-class scalar value
  applied uniformly — Java's finer item-type-pair matrix (smd/pin/via/wire
  cross terms) beyond the existing smd_smd/default_smd handling is not ported.

Open, with rationale:
- **Design-rule cost model** — per-layer widths, active-layer mask, directional
  and plane-via costs, and neckdown still need `BatchRequest` + the maze cost
  function extended.
- **Maze relaxation (optimality)** — occupy-on-push is retained; the Java-faithful
  occupy-on-pop was measured to reintroduce the relaxation storm (J2 128 ms →
  21 s, 168×). This is a documented tradeoff, not an optimality *fix*: accepting
  the first discovery is not proven cost-optimal.
- **Ratsnest** — the MST now stores only C² component-pair edges (a win for
  mostly-routed nets), but this is NOT general O(P): a fully unrouted net has
  C = P, so it stays O(P²). A true O(P log P) ratsnest needs Delaunay pruning,
  not ported.
- **Undo arena reclamation** — perf-only; index-based version chains make slot
  reuse risky for the critical undo/redo path.
- **CLI defaults** — pass count and thread count aligned to Java; the 300 s
  default timeout (vs Java 12 h), opt-in fanout (vs enabled), and optimize-by-
  default (vs Java disabled) are deliberate deviations, not alignment.
- **Multithreaded board clones** — parity with Java (`deepCopy` per task).

Full test count: 231 passing, 0 ignored (`j2_routes_fully_like_java` passes:
J2 routes 24/24). `cargo fmt --check` and `cargo clippy -D warnings` both pass.
No fleet completion regression across J2, pic_programmer, wavefolder, display,
8088sbc, ecc83.

## Second-round audit (2026-07-16) — partial, honest status

A follow-up audit found the first remediation was PARTIAL; "all findings
fixed" was an over-claim. Fixed in this round:

- KiCad JSON round-trip no longer loses custom clearances: the reader reuses
  the built-in null/default columns instead of appending duplicate classes
  (the duplicate made clearanceRules resolve to one column while nets used
  another), and imported pads/pours/traces/vias take their net's clearance
  class instead of a hardcoded default.
- The optimizer's own violation gate now uses the same clearance-matrix
  argument order and unit-independent tolerance as the authoritative DRC, so
  it cannot accept a candidate the final DRC would reject; and its acceptance
  metric counts incomplete CONNECTIONS (ratsnest airlines), not incomplete
  nets, matching Java's RouterCounters.

A follow-up audit found the first remediation was PARTIAL; "all findings
fixed" was an over-claim. A third round then closed four of the five
remaining items:

- **Component/image keepouts imported (#1).** Keepouts declared inside an
  `(image ...)` are now instantiated as obstacle areas at each placement,
  transformed like the pins. DRC excludes keepout-vs-pin (Java
  `ObstacleArea.is_obstacle` is false for a Pin), so a shield keepout drawn
  over its own pad is not a false violation.
- **Net-class propagation (#2, Part A).** A named class that carries a
  clearance rule always gets its own clearance-matrix class and `set_all`
  across item types — even when its value equals the board default — so its
  SMD pads use the class clearance; the default net class keeps smd pads on
  the tight smd class for plain nets. Pins/planes resolve from
  `default_item_clearance_classes`. Small grammar wins: `smd-smd` hyphen
  spelling and `smd_to_turn_gap` are now handled.
- **Named-clearance grammar (#2, Part B).** The general typed-pair system is
  ported (Java `Structure.set_clearance_rule`/`append_clearance_class`):
  `(clear V (type A_B))` resolves item-class (pin/via/smd/area) and arbitrary
  named classes, creating them on demand (copying the default row), with
  `wire`→default, item classes wired into the default net class, and the
  `create_default_clearance_classes` behavior on any `wire_*` pair. Also:
  net-class `(clearance_class NAME)` references, per-pin `(pin N
  (clearance_class NAME))` overrides in placement, and keepout
  `(clearance_class NAME)`. `_same_net` multi-underscore pairs are skipped
  (now stored + applied by the DRC, exceeding Java — see below). Verified
  against the Issue187 fixture (imports clean) plus synthetic unit tests.
  Image-keepout `(clearance_class NAME)` is still defaulted (structure keepouts
  are handled).
- **Named via-rule registry (was Part B, deferred).** `(via NAME PADSTACK [CL]
  [attach])` and `(via_rule NAME VIA...)` declarations build named ViaInfos and
  ViaRules; a net-class `(via_rule NAME)` reference binds the class to the named
  rule (winning over the `(circuit (use_via ...))` fallback).
- **Same-net clearance (`*_same_net`).** `(clear V (type A_B_same_net))` is
  stored by item-class pair and used by the DRC as the required spacing for a
  same-net drill pair. This EXCEEDS Java, which parses `*_same_net` into a dead
  clearance class that no item uses, so the rule has no effect there.
- **DRC same-net / conduction parity (#5).** Same-net drill pairs are checked
  as obstacles (Java `Via`/`Pin.is_obstacle`), exempting the connection case
  (an overlapping via-on-pad); conduction areas flagged `isObstacle` in KiCad
  JSON now enforce clearance to foreign copper.
- **Optimizer single-thread + partial progress (#4).** Acceptance is now a
  whole-board gate inside `optimize_nets_pass` (used by both the single- and
  multi-threaded paths): fewer incomplete airlines, then vias, then length,
  accepting partial progress (Java `ItemRouteResult`). The `broke_a_net`
  guard is retained.

RESOLVED after the third round:

- **J2 routing completeness (finding #6).** The current code routes J2 to
  24/24 nets, 0 clearance violations (score 999.94) — verified deterministically
  in both debug (batch ~4 s) and release, and end-to-end via the CLI. The
  mid-round diagnosis measured 22-23/24 and suspected an algorithmic wall, but
  that was on the earlier second-round code; the third-round changes altered
  the obstacle/clearance geometry the deterministic router sees, and the
  current router reaches full completion (I did not bisect which change flipped
  it). `j2_routes_fully_like_java` is now a normal (un-ignored) test and the two
  J2 pipeline tests assert full, clean completion. A **performance** gap remains
  (Rust routes J2 in seconds vs Java's ~0.7 s) — a speed concern, not a
  correctness one.

RESOLVED — router performance (was the last finding-#6 gap):

- The ~25 s J2 routing time was a **pass-loop plateau spin**, not slow search:
  after pass 0 completes 22/24, the escalating-penalty passes re-ran the same
  deterministic search (identical tree stats) failing the same 2 nets for up to
  9999 passes (~2.5 ms each ≈ 25 s), then the restart fallback finished them in
  one shot. A **plateau guard** now breaks the pass loop after 8 consecutive
  non-improving passes and hands off to the restart fallback (a stronger
  completion path that also gets more wall-clock this way). J2: 25.3 s → 0.14 s
  routing (24/24, 0 violations, score 999.94 — unchanged).
  **Fleet-validated (2026-07-16):** all 91 fixture boards ran at tl=60 s on the
  guarded binary; every board the guard could have hurt — the 17 that now give
  up early with time to spare — plus 9 time-pinned ones were re-run on the
  pre-guard binary (6d0d22f4): **0 completion regressions, 1 improvement
  (Issue022 +2 nets), 23 identical**, with the guard reaching the same
  completion up to ~10x sooner on plateau boards (e.g. RelayModule 43 s → 4 s,
  Issue208 45 s → 5 s). Remaining time-pinned boards were not re-compared (both
  binaries spend the identical budget there). Note: the capacity-overflow panic
  (see open gaps) predates the guard — verified on smoothieboard both before
  and after — and is not a regression.

KNOWN OPEN GAPS (not yet fixed) — do NOT claim these are done:

1. **Autoroute `layer_rule` (from finding #2, Part B) — router-core, not a
   grammar port.** `(layer_rule L (active on/off) (preferred_direction …)
   (…trace_costs …))` needs a per-layer directional cost model and active-layer
   gating in the maze search. The Rust router has NEITHER (its cost is
   distance + via-cost + ripup only, and `active_routing_layer` has no
   consumers). Router-behavior work (a route-quality / cost-model feature), not
   a bounded importer feature.
2. **Capacity-overflow panic on 5 fixture boards.** Issue145-smoothieboard,
   Issue508-DAC2020_bm06, Issue508-DAC2020_bm11, Issue730-DAC2020_bm11 and
   Issue732-RoyalBlue54L-Feather panic (`raw_vec capacity overflow`) during
   routing — a huge `with_capacity` somewhere in the router/import. Pre-existing
   (smoothieboard verified to crash before and after the performance change);
   surfaced by the 91-board fleet sweep of 2026-07-16. Untriaged.

## OPEN ITEMS (reconciled 2026-07-15, iter 190)

The early-phase checklists above are ticked with pointers; these are
the only genuinely open work items across the whole port:

1. [DONE iter 191] Trace cycle removal (Java: board.remove_if_cycle)
   — trace_is_cycle (BFS across non-area contacts proving a parallel
   path) + remove_if_cycle (removal + freed-tail chain cleanup),
   TRANSACTIONAL: a removal that degrades net connectivity rolls back
   (the shape-based contact walk can diverge from electrical reality
   at tolerance edges — an untransactional first cut broke J2's
   /MIPI_CSI_D0_P in the normalize phase). Wired into
   combine_all_traces (repeat until no cycle). J2 length improved
   486.0 -> 482.3 mm; fleet clean; unit test cycle_traces_are_removed.
2. [DONE iter 192] DSN keepout-area import — (keepout ...) as full
   obstacle areas and (via_keepout ...) as via-only obstacles
   (ObstacleAreaItem.via_only: blocks via placement/drills but not
   traces — exempted in room completion, engine door classification,
   shove entries, pull-tight legality, and kind-aware in the DRC and
   violation counts; via-context checks were already blocking).
   Polygons exact, rect/circle/path via corner shapes; layer name or
   signal/all -> per-layer items, SystemFixed. place_keepout affects
   only component placement and is skipped like Java's router does.
   J2's own 4 keepouts now import and it still routes 24/24 clean.
3. [DONE iter 193] SES library_out section — the writer now emits
   (library_out (padstack NAME (shape ...) ...)) for every via
   padstack referenced by session vias or the via rules; boxes as
   rect, octagons/simplices as polygon corner lists, per layer in
   board units. Verified: J2 session re-imports through --import-ses
   at 24/24 / 999.94. (Java's `(attach off)` flag has no Rust
   padstack field — omitted.)
4. [DONE iter 194] Ratsnest airlines with Java's NetIncompletes
   semantics — Kruskal's MST over the net items' representative
   points (via/pin centers, both trace endpoints, area centroids),
   edges ascending, one airline per cross-set edge, endpoints at the
   actual nearest item points (was: component centers). Java's
   975-line PlanarDelaunayTriangulation only PRUNES Kruskal's
   candidate edges — a Delaunay triangulation always contains the
   Euclidean MST, so the complete graph yields identical airlines at
   ratsnest sizes; the triangulation is a performance device and is
   deliberately not ported. Verified: coldfire stripped @5 s gives
   292 airlines across exactly the 67 incomplete nets.
5. [DONE iter 195] Corridor-trace clearance at insert — every trace
   run is exactly checked at insert (mitered pre-filter + Euclidean
   confirm, trace_run_is_clear, symmetric to the via-site gate);
   conflicted runs first shove the offenders aside (shove_aside per
   offset segment, vias included), and the insert fails
   transactionally when even that cannot clear the corridor. Clear
   runs insert through the old path. Coldfire @600 s: 0 deep
   violations (was 2; was 12 before iter 190), completion 267/278 =
   Java-equivalent completion with a violation-free board (Java ships
   violations at this scale). Fleet clean, 215 tests.
6. [CLOSED iter 204 — verified already byte-exact] The door-segment
   derivation was audited line by line against Java:
   ExpansionDoor.get_section_segments == compute_door_section_segments
   (offset+tolerance, dim-1 diagonal shrink, dim-2 free-space corner
   scan with the 4*offset^2 small-door cutoff, gravity-point fallback,
   10*offset section width, identical divide), calc_door_line_segment
   corner scans identical, TileShape.diagonal_corner_segment identical
   (corner 0 to corner n/2). The only deviations are Rust-side
   None-safety where Java would NPE. The historical note predated the
   iter 157-158 SRN/door faithfulness work and was stale.
7. [CLOSED iters 199-203] Coldfire wall clock: the via_free memo
   (43M -> 1.77M tree queries) converges coldfire by ~200 s; the
   fleet canonical shows every board in Java's range or better
   (8088sbc 4.4 s, small boards in fractions of a second).

REMAINING (research, not porting): the last ~10 coldfire nets and
interf_u's final net under tight budgets plateau under every current
strategy (passes, restarts, scaled recovery all roll back) — they
likely need simultaneous multi-net negotiation, which no Java
mechanism provides either (Java ships violations on these boards
instead).

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
- [x] Remaining geometry (entrance_points, cutout_polyline, is_intersected_interior_by all in tile_shape.rs; touching_sides in sorted_room_neighbours.rs. Ellipse is GUI-only in Java — boardgraphics + interactive Route — excluded by scope.)

## Phase 2 — supporting infrastructure

- [x] datastructures/ShapeTree.java + MinAreaTree.java → `datastructures/min_area_tree.rs` (arena-indexed generic tree; IntOctagon bounds subsume both ShapeBoundingDirections variants; ArrayStack replaced by Vec)
- [x] datastructures/UndoableObjects.java → `undoable_objects.rs` (explicit key/value split, arena version chains; pop_snapshot splices same-level undo versions — documented deviation fixing a Java edge-case that loses objects)
- [x] datastructures/Stoppable.java + TimeLimit.java → `stoppable.rs`
- [x] datastructures remaining (IdentifierType/IndentFileWriter: quoting/indentation handled inline by the Rust writers. PlanarDelaunayTriangulation: NOT ported — the ratsnest uses MST over component centers instead; listed under OPEN ITEMS.)
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
- [x] board remaining (all landed via equivalents: pins are component-tagged drill items — KiCad reader iter 185 marks them SystemFixed; BoardOutline as boundary keepout strips; clearance compensation via the board inflation cache; RoutingBoard functionality folded into BasicBoard; shove/pull-tight/forced-via/move-drill ported iters 144-158.)

## Phase 3 — routing engines

- [x] interim grid A* router → `autoroute/simple_router.rs` (NOT a Java port: stand-in so the pipeline routes end to end; uses exact board obstacle queries, inserts polyline traces + layer-change vias)
- [x] expansion-room object model → `autoroute/expansion_room.rs` (ExpansionRoom/Door/MazeSearchElement as arena RoomGraph; door section segmentation incl. 2-dim restraint lines; + TileShape::diagonal_corner_segment)
- [x] free-space room completion → `autoroute/room_completion.rs` (ShapeSearchTree.complete_shape + restrain_shape with deterministic obstacle order; + TileShape distance_to_the_left / side_of_line / is_intersected_interior_by / half_plane; divide_large_room pending)
- [x] AutorouteEngine core → `autoroute/engine.rs` (per-net room graph; complete rooms restrained against board + existing rooms; doors to touching rooms; target doors to own-net items; lazy frontier expansion per border edge — simplified vs SortedRoomNeighbours' sorted-edge-gap algorithm, documented)
- [x] maze search core → `autoroute/maze_search.rs` (Dijkstra over door sections + drill steps with backtrack-node arena; multi-layer via expansion at entry locations — simplification of DrillPage candidate generation, documented; per-layer trace + via insertion; no ripup/shove yet)
- [x] batch autorouter loop → `autoroute/batch.rs` (per-net component analysis, closest-pair incompletes preferring drill endpoints — trace splitting at junctions not yet ported, no-progress guard, single pass without ripup escalation)
- [x] trace splitting at junctions → `BasicBoard::split_traces_at` (+ maze_route normalizes inserted endpoints; T-junction contacts now register)
- [x] trace combining at simple joints → `BasicBoard::combine_trace` (PolylineTrace.combine: exactly-one-trace contact with equal layer/width/nets merges via Polyline::combine)
- [x] faithful autoroute port remaining (SortedRoomNeighbours iters 157-158; clearance compensation via inflation model; DrillPageArray iter 159; ripup/shove + pass escalation iters 14x-15x; Locate/Insert corner calculation iter 156; fanout iter 153; optimizer iters 161-166 + 186 + 188. EXCEPTION: overlap/cycle removal on trace split — Java's board.remove_if_cycle — NOT ported; listed under OPEN ITEMS.)

## Phase 4 — I/O and CLI

- [x] S-expression reader → `io/dsn.rs` (tokenizer/tree with quoted strings incl. the `(string_quote ")` special case; navigation helpers; verified against real repo fixtures)
- [x] DSN semantic import → `io/dsn_import.rs` (layers by index, resolution scaling, default rules, padstacks — circle/rect/path/polygon as box/octagon approximations, images/placement with rotation + back-side mirroring, network pin binding; wiring/keepout import + faithful pad shapes pending)
- [x] SES session writer → `io/ses_export.rs` (network_out wires + autoroute vias; validated by re-parsing)
- [x] end-to-end integration → `tests/route_fixture.rs` (import interf_u fixture → route /ACK → export session with the routed wire)
- [x] boundary import as outline keepout strips (BoardOutline tree-shape equivalent; routes verified to stay inside the outline bbox)
- [x] DSN wiring import (pre-routed wires and vias, dsn_import.rs) — but DSN keepout-AREA import and the SES library_out padstack section are NOT ported; listed under OPEN ITEMS.
- [x] CLI entry point (src/main.rs: the Java jar's full non-GUI surface; see the feature checklist)

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
- 2026-07-15 (iter 180): COLDFIRE NEW BEST — 268/278 at 600 s
  (Java 1.9: 11 unrouted at 199 s ⇒ ~267 — completion parity
  reached; wall clock remains Java's). GND (77 connections!) and
  +3.3V each route alone in ~1 s (probe_net) — the tail is pure
  congestion competition among 482 connections. Pass trajectory:
  16→10 failures through the penalty ladder, restarts plateau at
  268. The fleet standing: 7/8 boards fully complete + zero
  violations; coldfire 268/278 ≈ Java completion at 3× Java clock.
- 2026-07-15 (iter 179): INTERF_U COMPLETE — 173/173, the VCC
  holdout falls. Diagnosis: VCC (22 connections, the biggest net)
  routes ALONE in 136 ms — the ascending-extent order simply routed
  it LAST into leftover congestion. New HYBRID net order: many-pin
  nets (≥6 connectable items — power nets needing whole corridor
  systems) route FIRST by descending pin count on the open board;
  signal nets keep ascending extent (measured cleanest). Straight
  descending completed everything too but reintroduced 2+2
  violations on NormalPuzzle/J2 — the hybrid gives BOTH: interf_u
  173/173 AND all-zero violations across NormalPuzzle/J2/display/
  8088sbc. SEVEN OF EIGHT boards now fully complete + clean; only
  coldfire's tail remains. probe_net example added (routes one named
  net on the bare board).
- 2026-07-15 (iter 177): SCORE HEAD-TO-HEAD (identical formula both
  sides — ours is the exact port; router phase, 60 s Rust runs):
  | board        | Java 1.9 score          | Rust score               |
  | J2           | 971.95 (3 unrouted)     | 999.97 clean         WIN |
  | wavefolder   | 974.21 (5 unrouted)     | 999.97 clean         WIN |
  | display      | 995.60 (1 unrouted)     | 999.94 clean         WIN |
  | pic          | 996.51 (1 unr + 1 v)    | 994.09 (1 v, varies) ~tie|
  | NormalPuzzle | 999.99                  | 999.99               tie |
  | 8088sbc      | 999.98                  | 999.91 (via/length)  ~tie|
  | interf_u     | 999.86 (62 VIOLATIONS)  | 172/173 clean       split|
  | coldfire     | 993.45 (11 unr + 4 v)   | 260/278             Java |
  Rust wins or ties 6/8 by Java's own metric; 8088sbc's gap is pure
  via-count/length cost (Java routes it with fewer vias — optimizer
  quality). pic's "1 violation" RESOLVED: it is DESIGN-INHERENT (a
  factory pad-clearance issue) — check_board audits all items
  including pins, Java reports the identical 1 violation; drc_check
  audits routed items only, hence its 0. Both routers behave
  identically. Optimizer keep-rules hardened along the way (kept
  transactions may not increase a net's violations; via-slide stubs
  are clearance-checked); FR_NO_OPT knob added for stage isolation.
- 2026-07-15 (iter 174): HEAD-TO-HEAD REFRESH (all features + all
  performance work in; Java 1.9 router phase on stripped boards):
  | board        | Java 1.9                | Rust (now)                 |
  | NormalPuzzle | 1.2 s clean             | 2.0 s, 72/72, 0 viol       |
  | J2           | 3.4 s, 3 UNROUTED       | 0.1 s, 24/24, 0 viol   WIN |
  | wavefolder   | 5.9 s, 5 UNROUTED       | ~2 s, 31/31, 0 viol    WIN |
  | display      | 3.3 s, 1 unrouted       | 8.5 s, 30/30, 0 viol   WIN |
  | pic          | 1.2 s, 1 unr + 1 viol   | 0.8 s, 111/111, 0 v    WIN |
  | 8088sbc      | 8.0 s clean             | 14.1 s, 104/104, 0 v  ~par |
  | interf_u     | 7.6 s + 62 VIOLATIONS   | 227 s, 172/173 clean  split|
  | coldfire     | 199 s, 11 unr + 4 viol  | 300 s, 260/278        Java |
  READING: the Rust port completes MORE nets with ZERO violations on
  6 of 8 boards, matches 8088sbc within 2×, and trails only on the
  two largest boards' wall clock — where Java trades violations for
  speed (interf_u: 62!). interf_u new best 172/173 (VCC last).
- 2026-07-15 (iter 173): THE VARIANCE SOLVED — it was never
  variance: 8088sbc has only 471 items, under the 1000-item
  cross-net threshold, so route_board ran the old path while the
  drc A/B had FR_CROSS_NET=1 explicitly. Threshold lowered to 400.
  RESULT: 8088sbc routes 104/104 COMPLETE IN 14.1 s with ZERO
  violations (Java 1.9: 8.0 s) — from ~300 s to within 2× of Java.
  ENTIRE fleet incl. 8088sbc now at zero violations and full
  completion. interf_u/coldfire reruns queued.
- 2026-07-15 (iter 172): PERFORMANCE PHASE round 1. (a) Drill pages:
  two-level cache (net-independent base drills computed once per
  invalidation; per-net recompute only for pages actually containing
  that net's items) + ExpansionDrill reduced to (location, bbox) —
  cloning free-area shapes per room entry was ~30% of NormalPuzzle:
  3.5 s → 2.0 s. (b) SRN make_neighbour bbox precheck (neutral —
  candidates were already filtered). (c) CROSS-NET ROOM REUSE now
  PAYS on big boards since SRN + obstacle rooms: 8088sbc pass 0
  18.9 s/5-failed → 9.8 s/1-failed, 2.6× fewer completions, and the
  cross-net drc run reached 104/104 with ZERO violations, routing
  done in ~12 s. Default now by board size (item_count ≥ 1000;
  FR_CROSS_NET overrides). OPEN: 300 s route_board runs on 8088sbc
  show run-to-run variance (one run completes at pass 1 in ~12 s,
  another churns to 281 s / 103-104) — order-effect sensitivity to
  chase first next iteration; the old path also shows 8 violations
  on this board in some runs (pre-existing, cross-net run was clean).
- 2026-07-15 (iter 153): DIRECTIVE CHANGE — feature completeness
  FIRST, performance alignment second; ALL routing algorithms must be
  faithful ports, not approximations. FEATURE CHECKLIST (non-GUI):
  ROUTING ALGORITHMS (priority order):
  [x] BatchFanout (iter 153: fanout.rs — maze is_fanout mode completes
      at the first drill like Java's MazeSearchAlgo; passes over SMD
      pins outer-first; CLI --fanout, default off like Java)
  [x] MoveDrillItemAlgo (iter 154: board/move_drill_item.rs —
      try_shove_via_points border projections, transactional move_via
      with destination shoves, shove_vias wired into shove_aside like
      Java's forced_pad; deeper via-recursion still shallow)
  [x] ForcedViaAlgo (iter 155: board/forced_via.rs —
      insert_forced_via/check_forced_via: per-layer pad shapes +
      wider-trace-pen shapes shoved free then the via inserted,
      transactional; check leaves the board unchanged. Maze/fanout
      integration lands with MazeShoveTraceAlgo)
  [x] 45°/90° AngleRestriction (iter 156: calculate_additional_corner
      + fortyfive/ninety corner constructors ported exactly;
      restrict_corners rewrites the found path room-aware
      (horizontal-first preferred, alternative when outside the room,
      like LocateFoundConnectionAlgo45Degree); pull-tight only takes
      compliant bypasses. CLI --angle none|45|90 (default 45 like
      Java), route_board/drc_check keep any-angle default for
      benchmark continuity (FR_ANGLE knob). 45° fleet: display 30/30,
      J2 24/24, wavefolder 31/31, pic 111/111, NormalPuzzle 72/72 —
      ALL ZERO violations. PullTight45's full corner-reduction
      remains simplified: bypass gating only)
  [x] SortedRoomNeighbours (iters 157-158: core + ENGINE INTEGRATION,
      DEFAULT ON — completion runs the SRN walk per piece, uncovered
      border gaps become INCOMPLETE rooms in the graph with doors,
      and entering a room lazily completes the gap rooms behind its
      doors (Java's growth model exactly; FR_SRN=0 keeps the interim
      frontier expansion). Fleet: NormalPuzzle 72/72 @0.98 s (faster
      than frontier), display 30/30, wavefolder 31/31, pic 111/111 —
      all zero violations; 8088sbc pass 0: 11.4 s/2-failed vs
      15.0 s/5-failed — SRN better everywhere. J2's last net (GND)
      flaps in BOTH modes since the via-shove change — order
      sensitivity, tracked separately.)
      — Neighbour records with exact first/last corners, the
      counterclockwise comparator (tolerance 1), touching_sides /
      equals_corner classification for dim-1 and dim-0 touches, and
      calculate_new_incomplete_rooms' full gap walk (start/middle/end
      edge lines, concave-corner drops) producing GapRooms; unit
      tested on synthetic geometry. REMAINING: engine integration —
      incomplete-room queue in the graph, maze-triggered lazy
      completion through doors, then replace expand_room and flip
      the default)
  [x] DrillPageArray/DrillPage (iter 159: autoroute/drill_pages.rs —
      pages of max(5×via,10k) width caching ExpansionDrills = centers
      of the convex free pieces (page minus undrillable inflated item
      shapes, split via restrain_all), per-net like Java, invalidated
      through the change log/epoch; the maze drill expansion consumes
      page drills intersecting the entered room instead of grid
      sampling. FLEET: all boards complete, zero violations — J2 back
      to 24/24 (page candidates cured the GND flap). COST noted for
      the performance phase: NormalPuzzle 0.98→3.1 s, 8088sbc pass0
      11.4→17.4 s (per-net page recompute).)
  [~] MazeShoveTraceAlgo (iters 160+169: the obstacle-room model
      carries the semantics; iter 169 adds the COST MODEL — obstacle
      rooms whose trace matches Java's check_shove_trace_line
      preconditions (same half width and clearance class, unfixed)
      are charged 0.25× the rip penalty, so the maze prefers
      slide-friendly corridors, and the insert-time corridor shove
      slides them like Java's maze shove (rip only what stays).
      Iter 181 also ports ALREADY_RIPPED_COSTS: moving between
      obstacle rooms of the SAME item (consecutive segments of one
      trace) is free — the rip was charged at first entry (traversal
      along a rippable trace no longer costs per segment).
      Iter 181 adds Java's section_ok gate: the slide discount
      applies only when entering through the FIRST or LAST door
      section (interior entries cannot slide the trace past the
      entry point) — fleet unchanged at all-zero/full-completion.
      REMAINING for the byte-exact port: the diagonal/polar door
      segment derivation and Adjustment-aware door sections during
      search — behaviorally covered by insert-time transactional
      validation.)
  [x] OptViaAlgo (iter 161: board/opt_via.rs — a via contacted by
      exactly two unfixed traces slides toward the adjacent corners /
      their midpoint when legal (forced-via with shove) and strictly
      shorter; stubs reconnected, transactional, both nets verified
      connected. Wired into the optimizer phase (optimize_vias sweep
      per pass). Iter 184: plane/fanout single-contact branch PORTED
      (opt_single_contact_via — the via pulls along its stub toward
      the contact; a degenerated stub is deleted and the via lands
      on the far item, like Java). All OptViaAlgo branches now in.)
  [x] BatchOptimizerMultiThreaded (iter 166: optimize_route_multithreaded
      — groundwork: BasicBoard is Clone + Send (Rc→Arc in the shared
      shape caches, OnceCell→OnceLock in Simplex/Item). CLI
      --threads <n>. Iter 186: FULLY FAITHFUL — per-net tasks pulled
      from a shared queue, each cloning the master board; GREEDY
      replaces the master the moment a task wins (later tasks clone
      the updated board), GLOBAL_OPTIMAL adopts only the pass's best
      at pass end (sequential selection forced, like Java), HYBRID
      alternates by ratio. Item selection SEQUENTIAL / RANDOM
      (deterministic xorshift) / PRIORITIZED (prior-pass results
      sorted by Java's ItemRouteResult compareTo: incompletes, vias,
      length; unseen nets appended). Defaults match Java: GREEDY +
      PRIORITIZED, ratio 1:1. CLI --opt-strategy greedy|global|hybrid,
      --opt-selection, --hybrid-ratio N:M; CLI also prints the final
      post-optimizer score now. J2 --threads 4: 999.94, 0 violations
      under both strategies; NormalPuzzle 999.97 under all three.)
  [x] Distinct-net shove stacking (iter 178: VERIFIED ALREADY
      COMPLETE — calculate_stack_levels' level raise/lower across
      foreign net sets is the full Java multi-net semantics; a new
      regression test shoves two stacked traces of different nets and
      proves both stay connected with the shape cleared. The old
      "single family" note was stale.)
  I/O & TOOLING:
  [x] DSN export (iter 163: io/dsn_export.rs — the imported document
      is retained without its wiring (board.dsn_source) and re-emitted
      with the wiring regenerated from the routed items (wire paths +
      route vias with net/type tags); CLI --export-dsn. KNOWN ISSUE:
      round-trip connectivity drift — some re-imported wires miss
      their pad contacts (endpoint rounding), chase next.)
  [x] SES import (iter 164: io/ses_import.rs — SesReader port:
      network_out wires/vias parsed with the session's resolution
      scaled to the board's, inserted with their nets; CLI
      --import-ses applies a session before routing. Round-trip:
      route → .ses → import → 72/72 complete, 0 connections, 8 ms.
      Also this iteration: DSN export UNIT FIX — coordinates descale
      to file units; full-board DSN round-trip now preserves
      connectivity exactly.)
  [x] RulesReader/RulesWriter (iter 167: io/rules_io.rs — the
      Specctra (rules PCB ...) format: snap_angle, default
      width/clearance rule, per-class width rules; CLI --rules /
      --export-rules; round-trip tested. Java's full via-info/
      via-rule serialization simplified — noted.)
  [x] KiCad board JSON reader (iter 168: io/kicad_json.rs on a
      hand-rolled zero-dependency JSON parser (io/json.rs) — layers,
      net classes (clearance/width/via geometry), nets, components
      with rotated pad offsets (through-hole vs SMD layers),
      pre-routed traces/vias; mm/mil/µm units at Java's 0.1 µm
      default resolution. CLI: -de file.json. Iter 183: WRITER ported
      too (export_kicad_json, CLI --export-json, round-trips through
      the reader). Iter 185: full Java parity — per-net-class
      clearance classes in the matrix + custom clearanceRules pairs,
      per-class trace widths and via padstacks resolved through via
      rules (via_padstack_for_net), circle/oval pads as Java's
      corner-cut octagons, pad layer spans from the named layers,
      conduction areas (net pours connectable, netless zones as
      keepouts; Java's separate is_obstacle flag folded into that
      distinction — noted), referenced-net auto-registration,
      system-fixed pins / user-fixed pre-wiring, and Y-axis negation
      on import like Java. Deviation: Java's writer emits un-negated
      Y (breaking its own round trip); the Rust writer negates
      symmetrically.)
  [x] DRC report (iter 162: src/drc.rs — DesignRulesChecker port:
      check_board collects deduplicated clearance violations (mitered
      pre-filter + exact Euclidean confirm, worst actual distance) and
      unconnected nets; DrcReport::to_kicad_json emits the KiCad DRC
      v1 schema exactly like Java's DrcReport (coordinate_units mm,
      violations with per-item positions, unconnected_items,
      schematic_parity). CLI --drc-report <file>.)
  [x] RatsNest export (iter 176: src/ratsnest.rs — minimum-spanning
      airlines over unconnected component centers per incomplete net,
      JSON output; CLI --ratsnest)
  CORE/INFRA:
  [x] Scoring (iter 165: src/scoring.rs — BoardStatistics collection
      (incomplete/maximum nets, violations via drc, bends, vias,
      mm-normalized length) + Java's exact score formula and defaults
      (unrouted 5M, violation 1M, bend 10, via 50, trace 1/mm);
      normalized 0..1000 with the no-connection guard. CLI logs
      "score: 999.99 (...)" like the Java pass log — NormalPuzzle
      999.99, J2 999.97.)
  [x] RoutingJob/JobState model (landed with the API server,
      iter 170)
  [x] RouterSettings profiles (iter 176: --profile file.json —
      maxPasses, viaCosts, timeLimitSeconds, angleRestriction,
      threads as defaults under explicit flags)
  [x] API server (iter 170: src/api.rs — the v1 job API on a
      dependency-free HTTP/1.1 server: POST /v1/jobs/enqueue,
      POST /{id}/input, PUT /{id}/start (background routing thread),
      PUT /{id}/cancel, GET /{id} (state+score), GET /{id}/output,
      GET /v1/system/status; includes the core RoutingJob/JobState
      model. CLI --api-server <port>. E2E verified: enqueue→input→
      start→COMPLETED score 999.97→SES fetch.)
  [x] MCP server endpoint (iter 171: POST /v1/mcp — JSON-RPC 2.0
      initialize / tools/list / tools/call with six tools mapped onto
      the job API (enqueue_job, set_job_input, start_job,
      get_job_details, get_job_output, system_status); protocol
      2024-11-05; e2e verified with an enqueue through MCP.
      === THE NON-GUI FEATURE CHECKLIST IS COMPLETE. Remaining
      precise refinements: MazeShoveTraceAlgo section-alignment
      geometry, OptViaAlgo plane-via branch, GREEDY optimizer merge,
      KiCad JSON writer, rules via-serialization. NEXT PHASE:
      PERFORMANCE (drill-page per-net recompute, big-board wall
      clock vs Java 8 s), then correctness re-validation. ===)
  DONE (faithful): AutorouteEngine, MazeSearchAlgo core,
  BatchAutorouter, BatchOptimizer (single-thread), ShoveTraceAlgo +
  ShapeTraceEntries (single family), PullTightAlgoAnyAngle (core),
  rules, DSN import, SES export, planes, snapshots/undo.
- 2026-07-15 (iter 151): TRACE TAPS (junction splitting) — the maze
  may now arrive at a TRACE of the destination component:
  Polyline::nearest_lattice_point projects the arrival onto the
  centerline's integer lattice (exact by construction, so
  split_traces_at always registers the junction contact), and the
  destination sets include all connectable items. RESULTS:
  display 30/30 in 8.5 s with ZERO violations — FULLY COMPLETE for
  the first time; NormalPuzzle 1.05 s; J2 24/24 (one NEW shallow
  violation to chase — likely a tap segment edge case); interf_u
  171/173 (VCC still open, /PC-A3). 8088sbc 104/104 (iter 150).
- 2026-07-15 (iter 150): J2 RECOVERED (24/24, zero violations) +
  8088sbc FULLY COMPLETE (104/104 at 293 s with the optimizer — the
  first big-board full completion). The J2 regression was NOT a
  contact bug (net probe: two ordinary SMD pads; contacts symmetric)
  but congestion musical-chairs: set-to-set rerouted the OTHER MIPI
  nets differently, pass-0 walls net 2 in, and every restart round
  that fixed it broke another net — a TIE, which the fallback
  discarded. Restart now ACCEPTS a tie when it CHANGES the
  failing-net set (bounded by max_dry = min(#failures+1, 4)); the
  next round attacks a different net first and completes. The old
  do-not-retry note on tie-accepting is superseded: with rotation +
  1-for-1 swaps + set-to-set, ties genuinely change the state.
  Diagnostics: examples/net_probe.rs (--route) dumps a net's items,
  shapes, endpoints, contacts and connected sets; FR_KEEP_JUNK
  preserves no-progress inserts for autopsy. Fleet: ALL FIVE small
  boards zero violations; NormalPuzzle 1.0 s; display 29/30.
- 2026-07-15 (iter 149): SET-TO-SET ROUTING (Java: p_start_set/
  p_dest_set) — the maze now starts from and arrives at ANY
  endpoint-capable item (drills/pads via endpoint_candidates; traces
  excluded until junction splitting: arriving mid-trace registers no
  contact) of the two components, not one closest pair. RESULTS:
  NormalPuzzle 72/72 in 1.04 s — FASTER THAN JAVA 1.9 (1.22 s);
  display routing 46 s → 18 s and GND (the long-standing holdout)
  COMPLETES — 29/30 with Net-(K1-Pad4) the new last net; interf_u
  171/173 (window clip, new best). DEFENSES added: empty inserts
  (coincident corners) return None; no-progress connections get
  their junk items removed; degenerate direct-arrivals fall through
  to the search; single-pair retry on no-progress. OPEN REGRESSION:
  J2 23/24 (/MIPI_CSI_D0_N — 296 identical no-progress attempts
  pre-defense; suspected stacked same-net pads whose pad-pad contact
  never registers — connectivity model gap; single-pair fallback
  did not recover it, needs contact forensics). Optimizer recovery
  budget raised to 20 s for incomplete nets. Window verdict:
  8088sbc full runs — 200k: 103/104 @232 s, 100k: 102/104 @259 s —
  default stays 200k.
- 2026-07-15 (iter 147): BIG-BOARD THROUGHPUT ROUND 1 — 8088sbc
  pass 0: 21.9 s → 7.0 s. Work accounting (FR_STATS + new per-phase
  timing): 188k room completions for 420 searches — every frontier
  expansion seeded a raw HALF-PLANE, so each completion queried and
  restrained half the board's obstacles. Expansion seeds are now
  CLIPPED to a window around the contained edge (FR_ROOM_WINDOW,
  default 200k units; 100k = fastest pass 0 at 7 s with a few more
  pass-0 failures for later passes to fix; Java bounds rooms via
  divide_large_room/drill pages). Also: per-net ripup budget
  (transaction capped at min(10 s, remaining) — a single hard net
  burned 45 s of a pass before), TimeLimit::remaining_ms, per-piece
  NET DEPENDENCE (only when a skipped own-net/rippable item's
  inflation actually overlaps the piece — the old any-skip flag
  killed ~90% of rooms per net switch; survival tripled, though
  cross-net reuse still doesn't pay on dense boards where every
  route's change log shreds the cache). Small fleet: all zeros hold,
  NormalPuzzle 3.07 s. Remaining 8088sbc gap vs Java (8 s total):
  ripup passes still burn 10 s per hopeless net and the restart
  36 s/round; GND (giant power net) is the persistent holdout —
  likely needs fanout-style multi-target routing.
- 2026-07-15 (iter 146): FLEET-WIDE ZERO — every small board audits
  at 0 violations: pic 111/111, display 29/30, wavefolder 31/31,
  J2 24/24, NormalPuzzle 72/72. The pic "leak" was resolved by an
  EXACT-DATA REPLAY (tests/pic_completion_replay.rs + captured
  fixture): the suspicious room was byte-identical in replay and
  live, and provably excludes every collected obstacle — THE ROOM
  WAS NEVER DIRTY. The final "violations" were CHECKER ARTIFACTS:
  our DRC inflated with mitered line-pushes, whose corner reach is
  up to √2 × the margin — geometry that is Euclidean-legal was
  reported as violating (miter has no false negatives, only corner
  false positives). drc_check now confirms mitered hits with
  TileShape::euclidean_distance_to (convex-convex corner/segment
  distance) — physical DRC semantics, like KiCad. Diagnostics
  (ILLEGAL INSERT / ROOM LEAK) stay mitered-conservative. Capture
  infrastructure: FR_DEBUG_REGION_FULL + FR_DEBUG_CONTAINED dump a
  completion's full input (start/contained/ordered obstacles) for
  offline replay; restrain_all() extracted as the replayable core.
- 2026-07-15 (iter 145): PIC LEAK NARROWED TO A COMPLETION PIPELINE
  CONTRADICTION. pic's 2 violations: net 21's SAME path inserts
  illegally against successive incarnations of net 14's trace
  (280→341→354). Facts established with new diagnostics (SYNC/ENGINE/
  NEWROOM logs, LEAKGEOM shape capture — data saved in scratchpad
  leakgeom_pic.txt): the engine was FRESH (created for net 21 after
  354's insert, epoch-cleared), room 57 was completed BY that engine
  with the blocker on board AND COLLECTED (region-obstacles include
  280), margin correct (6518 = hw 4000 + cl 2502 + safety 16) — yet
  the output room 2D-overlaps the blocker's inflation, and
  RECOMPLETING the same shape cuts it correctly (still-dirty false).
  Restrain outputs provably exclude processed obstacles; the gate
  (dimension()==2) cannot skip for ≥5-line intersections ⇒ one of
  the "impossible" steps is wrong in a representation-dependent way
  (recompletion differs only via intersection_with_simplify's
  normalization). NEXT ACTION: offline replay — log the failing
  completion's full input (start simplex + ordered obstacle
  simplices), reproduce in a unit test, and bisect the pipeline on
  exact data. Suspects in order: LineSegment::from_shape /
  is_intersected_interior_by on REDUNDANT-line simplices (wrong
  corner endpoints → cut_line not found → second-branch behavior),
  dimension() on ≤4-line unsimplified intersections.
- 2026-07-15 (iter 144): BIDIRECTIONAL PULL-TIGHT GATE — FOUR
  boards at ZERO violations with FULL post-processing: display
  (29/30), NormalPuzzle (72/72), wavefolder (31/31), J2 (24/24);
  pic 111/111 with 2 left. The post-processing pair was MITER
  ASYMMETRY: inflation is a mitered line-push, so miter(a,cl)∩b=∅
  does not imply miter(b,cl)∩a=∅ at diagonal corners — pull-tight
  validated only its own direction while the DRC audit checks both.
  polyline_keeps_clearance() now gates the rebuilt polyline exactly
  in both directions (reusing the board inflation cache for the
  partner direction). interf_u re-benchmark post-occupy-on-push:
  170/173 @300 s (unchanged completion; VCC + 2 mid nets).
- 2026-07-15 (iter 143b): FULL JAVA 1.9 SCOREBOARD (stripped
  wiring, router phase only, -mp 99):
  | board        | Java 1.9 router           | Rust (pre-iter-142 nums) |
  | J2           | 3.4 s, 3 UNROUTED         | ~1 s, 24/24, 0 viol  WIN |
  | wavefolder   | 5.9 s, 5 UNROUTED         | ~2 s, 31/31, 1 viol  WIN |
  | pic          | 1.2 s, 1 unr + 1 viol     | 0.8 s, 111/111, 2 v  WIN |
  | display      | 3.3 s, 1 unrouted         | 60 s, 29/30, 0 viol  tie |
  | NormalPuzzle | 1.2 s clean               | 3.8 s clean       Java×3 |
  | interf_u     | 7.6 s "done", 62 VIOL     | 300 s, 170/173     split |
  | 8088sbc      | 8.0 s clean complete      | 300 s, 103/104   JAVA×40 |
  | coldfire     | 199 s, 11 unr + 4 viol    | 300 s, 19 nets    JAVA   |
  Reading: Rust WINS completion+cleanliness on small boards; Java
  wins BIG-BOARD THROUGHPUT enormously (8088sbc 8 s vs 300 s) and
  tolerates violations to claim completion (interf_u 62!). Java's
  optimizer phase (unported) then recovers unrouted nets and cleans
  up. Big-board Rust numbers PREDATE the iter-142 occupy-on-push ×5
  — fresh fleet run queued. ALIGNMENT PRIORITIES: (1) big-board
  throughput, (2) post-processing violation pair, (3) optimizer.
- 2026-07-15 (iter 143): RIP CORRIDOR FIXED — display's ROUTING
  phase now audits at ZERO violations, deterministically (3× identical
  runs, 29/30 nets). The residual rip-window class was the iter-139
  via-placement bug's twin in the RIPUP block: the corridor loops
  (forbidden zones + to_rip) skipped la≠lb corner pairs, so the
  pre-via travel segment (which runs on the OLD layer) was never
  ripped and the via footprint ripped at the wrong end. Both loops
  now mirror the insert semantics (travel on layer_a, via at the
  drill node pb). Remaining display issue: post-processing pair
  (7 violations w/ post; both partners individually validated at
  their reinserts — TIGHT audit shows rebuilt-free=false fallbacks
  flagging pre-existing proximity; needs partner kind/geometry in
  the audit). JAVA SCOREBOARD (stripped, router phase): J2 3.42 s
  with 3 UNROUTED, wavefolder 5.94 s with 5 UNROUTED, display 3.31 s
  1 unrouted, pic 1.23 s 1 unrouted + 1 violation, NormalPuzzle
  1.22 s clean. RUST: J2 24/24+0 (~1 s), wavefolder 31/31+1 (~2 s),
  NormalPuzzle 72/72+0 (3.8 s), pic 111/111+2 (0.8 s) — Rust WINS
  completion on J2/wavefolder/pic, ties display, loses NormalPuzzle
  speed 3×. Java's optimizer phase (not ported) recovers their
  unrouted nets and cleans violations — the next big gap.
- 2026-07-15 (iter 142): OCCUPY-ON-PUSH — the ×5 search fix.
  FR_STATS counters exposed a relaxation storm: NormalPuzzle spent
  22M expansions / 69M heap pushes on 150 searches (~460k pushes per
  search on a 72-net board) because sections were occupied on POP:
  every re-entry of a room re-seeded all its door sections with
  marginally improved costs. Java occupies a section when it is
  INSERTED into the expansion list (each section queues exactly
  once, from the cheapest frontier element known at the time) —
  ported: NormalPuzzle 17.4 s → 3.5 s (measured UNDER cpu
  contention), 390k expansions / 494k pushes (56× / 139× less),
  72/72 with ZERO violations; J2 0 violations. Path quality is
  first-push-wins (Java-identical): display's rip-window violations
  amplified to 11 with 29/30 — that class is the next correctness
  front. JAVA BASELINE (stripped wiring, router phase): J2 3.42 s,
  wavefolder 5.94 s — RUST IS ALREADY FASTER on those; NormalPuzzle
  Java 1.08 s vs Rust 3.5 s contended. interf_u cross-net A/B: same
  completion as per-net (170/173) — cross-net reuse stays opt-in.
- 2026-07-15 (iter 141): ROOM PERSISTENCE INFRASTRUCTURE (Java:
  maintain_database) — the full machinery is in: rooms carry
  alive/net_dependent flags, RoomGraph::remove_room detaches doors
  and re-opens neighbours for expansion, the engine has a uniform
  grid over complete rooms (fixes the quadratic phase-2 scans that
  killed the first reuse attempt), sync_board_changes consumes the
  board's new (layer, bbox) change log (undo/redo/pop bump a change
  epoch → full drop), switch_net drops net-dependent rooms and rooms
  overlapping the new net's items. Board-level inflation cache added
  (per item id × margin; ids never reused). complete_shape_tracked
  reports net-dependence. RESULTS SO FAR: per-net reuse (default) is
  neutral; CROSS-net reuse (FR_CROSS_NET=1) is 25% SLOWER on
  NormalPuzzle (net-switch invalidation churn: full-board item scan
  per switch + rooms near every net item dropped) but recovered
  display to 30/30 — left OPT-IN until tuned. Profile truth:
  BinaryHeap::pop is ~50% of NormalPuzzle regardless (the search
  itself, not completion); completion cost (corner_approx/offset)
  did drop with the caches. Java-vs-Rust gap is NOT primarily
  completions on small boards — need per-phase comparison vs Java
  (their maze: ObstacleExpansionRooms, DrillPageArray, sorted
  neighbours). Java fleet baseline attempt was CONTAMINATED (jars
  route on top of the fixtures' pre-routed wiring — strip it first);
  partial data: coldfire Java 1.9 = 243/440 connections in 25 s with
  27 violations.
- 2026-07-15 (iter 140): NEW DIRECTIVE + JAVA BASELINE. User:
  "Fix the gaps with java then align with performance and
  correctness with java." The loop's goal is now closing the Java
  gap list and benchmarking against the actual Java router. Java
  jars build via `./gradlew buildBothVersions --no-configuration-cache`
  (settings.gradle foojay plugin bumped 0.8.0 → 1.0.0 for JDK 26).
  FIRST BASELINE: Java 1.9 routes NormalPuzzle in 1.08 s / 2 passes
  (plus a separate optimizer phase scoring 999.99); Rust needs
  ~13.7 s — a ~13× throughput gap. Biggest known lever: Java
  completes rooms ONCE and reuses them across connections and nets
  (ShapeSearchTree compensated shapes + SortedRoomNeighbours
  invalidation); Rust rebuilds the room graph per connection.
  Post-via-fix fleet (Rust): 8088sbc 103/104 (+5V), coldfire
  259/278 @300 s — the honest-geometry cost is visible fleet-wide.
  Also this iteration: pull-tight hardening (whole rebuilt polyline
  validated, original kept when blocked) + debug clearance audits
  after combine/pull-tight (both passes proven clean; the residual
  NormalPuzzle pair predates post-processing — rip-window class).
  ALIGNMENT PLAN: (1) room reuse + SortedRoomNeighbours [perf ×10],
  (2) rip-window correctness (last ~4 illegal inserts), (3) optimizer
  phase (Java improves length ~50% after routing), (4) MoveDrillItemAlgo,
  shove stacking, fanout, 45/90.
- 2026-07-15 (iter 139): THE VIA-PLACEMENT BUG — display reaches
  ZERO violations (FR_NO_POST audit; 0-5 across runs with post, was
  12): insert_connection placed the via at the corner BEFORE the
  drill node and moved the old-layer travel (door → drill point,
  legally searched on the old layer) onto the NEW layer where it was
  never searched. Latent while drills only happened at entry corners;
  the drill GRID SAMPLING made drill points far from the previous
  corner and turned every such via into a cross-board illegal
  segment. Found by extending the birth invariant to la≠lb corner
  pairs (they were silently skipped — a diagnostic blind spot that
  hid the whole class), which exposed a-in=false b-in=true with the
  via point 50k+ outside the checked room. Also fixed on the way:
  rooms_containing's fallback returned created pieces that did NOT
  contain the drill point (the piece holding it can be killed while
  others survive) — now filtered. ILLEGAL INSERTs 63 → 4. COSTS:
  interf_u 170/173 (the 2 recovered nets were riding illegal drill
  segments), display sometimes 29/30 — honest geometry is slightly
  harder. NormalPuzzle 72/72 in 14.3s unchanged. Diagnostics: ROOM
  LEAK false-alarms on ripup searches by design (rooms ignore
  rippable), RECOMPLETE probe distinguishes stale rooms from live
  collection bugs.
- 2026-07-15 (iter 138): RIPUP QUALITY — interf_u 172/173 (from
  170), recovering /MA11 and /MA14; only VCC remains. Three changes:
  (1) 1-for-1 swap tolerance in route_net_with_ripup victim
  recovery — at most one victim net may stay broken when the target
  completes, so completion stays monotone while hard failures become
  failure-set rotations later passes can attack from the other side
  (reported as a failure so the pass loop keeps running);
  (2) ripup penalty now ESCALATES per pass (base × pass number,
  Java-faithful) so churny swaps converge; (3) the restart fallback
  allows up to min(#failures, 3) dry rounds — the rotation makes each
  round genuinely different when several nets fail. No regressions:
  NormalPuzzle 14.5s 72/72, DRC unchanged (wavefolder/J2 0,
  display 12).
- 2026-07-15 (iter 137): PERFORMANCE RECOVERED after the miter fix,
  two wins: (1) getenv in hot loops — the debug-flag env reads
  (FR_ASTAR_WEIGHT per A* estimate!, FR_DEBUG_* per completion) took
  ~25%+ of samples via the getenv global lock; now cached in
  src/debug.rs OnceLock getters. (2) A* duplicate-push pruning — the
  open heap dominated profiles (60% in BinaryHeap::pop): every room
  entry re-pushed all door sections (O(D²) pushes). MazeSearchElement
  now carries best_cost; pushes that cannot improve a section's best
  queued cost are pruned (Java-equivalent discipline). NormalPuzzle
  48.9s → 13.7s (beats the pre-fix 16s), 72/72 restored. interf_u
  stays 170/173 (/MA11, /MA14, VCC — gives up at ~283s with dry
  restart rounds): those need better ripup/shove, not throughput.
  DRC unchanged: wavefolder/J2 0 violations, display 12,
  NormalPuzzle 2, FR_AUDIT_ROOMS clean. Also: cheap pre-cut on cached
  uninflated bboxes before inflating obstacle candidates, and the
  completion query is now a grown bbox instead of an offset simplex.
- 2026-07-15 (iter 136): THE MITER-REACH LEAK — the deep-violation
  class found, reproduced, and fixed. Chain: exact-overlap validators
  (bbox-noise removed) → invariant cross-check → FR_AUDIT_ROOMS
  (audits EVERY completed room vs every foreign inflated shape after
  each search) found 60+ DIRTY ROOMS → geometry dump + in-place
  recompletion probe reproduced 63/63 deterministically. ROOT CAUSE:
  obstacle inflation is a mitered line-push, so a box inflated by m
  reaches m·√2 beyond its copper at corners; the collection query
  start_shape.offset(m) only reaches m toward its own borders — an
  obstacle diagonally off a room corner is missed by the query while
  its inflation overlaps the room corner (Java is immune: its search
  tree stores pre-compensated shapes, queries are exact). FIX: query
  radius 2·(hw + max_cl + safety) via a new coarse bbox-level query
  (overlapping_items_coarse) + early bbox cut per inflated shape (the
  restrain loop stays exact). RESULTS: FR_AUDIT_ROOMS clean (0 dirty);
  display 30/30 with 12 violations (was 34-38; rest is the
  transactional rip/shove-window class + post-processing);
  wavefolder/J2/ecc83 0 violations. HONEST COSTS: the previously
  leaky rooms were routing through corner slivers — interf_u
  170/173 (was 173/173, 3 nets now need real ripup, finishes at
  276s), NormalPuzzle 71/72 (was 72/72). Recovering those is a
  ripup/shove quality problem, not a correctness one. Debug arsenal
  added: FR_AUDIT_ROOMS, FR_DEBUG_PATH, ROOM LEAK invariant
  cross-check, exact ILLEGAL INSERT validator, DEPTH bisection in
  drc_check. NOTE: routing is timing-nondeterministic (deadline
  checks), run-to-run counts vary a few violations.
- 2026-07-15 (iter 135): SAFETY MARGIN — the boundary-equality bug:
  the room guarantee (obstacle margin hw+cl) EQUALS the DRC
  requirement exactly, so boundary-riding paths tip into violation
  with ≤1-unit corner rounding. The clearance matrix's Java-faithful
  add_safety_margin flag (never used until now) closes it. RESULTS:
  wavefolder 31/31 complete with ZERO violations, J2 24/24 with ZERO
  — two boards fully complete AND fully DRC-clean; display 30/30 with
  34 violations remaining (the genuine transactional blind-window
  class: an on-board-at-insert case was disproven this round via the
  158/439 timeline — 439's insert was illegal with 158 present and
  rooms COLLECTING it, resolved as boundary-equality; the rest
  correlate with rip windows). Chain of this round: legal-at-birth
  inversion → late-comer 439 → completions collected the victim →
  guarantee-equals-requirement arithmetic → safety margin.
- 2026-07-15 (iter 134): THE PRISTINE-CASE LEAK FOUND AND FIXED — the
  timeline correlated a final violating trace to an ILLEGAL first-pass
  insert blocked by STATIC IMPORTED PINS (no transactions involved),
  and the region probe showed the completion collecting obstacle 58
  while the tree saw [58, 60]: pin 60 lay just OUTSIDE the frontier
  half-plane, so its uninflated shape missed the collection query
  while its inflated margin reached inside the room (the iter-131
  "elimination" of query coverage was WRONG — huge half-planes still
  have borders that pass arbitrarily close to obstacles). Fix: the
  obstacle query runs on start_shape.offset(half width + max
  clearance). Results: wavefolder 31/31 with TWO violations (from 322
  at campaign start), J2 24/24 with ZERO, display 30/30 complete with
  illegal inserts halved (71→35; the pin case gone — the remainder is
  the transactional blind-window class, next target).
- 2026-07-15 (iter 133): the transactional layer is EXONERATED — a
  fuzz test (40 seeds x 200 random insert/remove/generate/pop/undo
  ops against a shadow model, verifying BOTH the alive set and the
  search-tree view after every op) passes with zero divergence. So:
  tree ✓, rooms ✓, corners ✓, segments ✓, endpoints ✓, snapshots ✓ —
  yet the original via and the blind-window trace coexist. Note the
  ILLEGAL INSERT count (71) exceeds final violations (53): many
  blind-window inserts DO roll back. The remaining reconciliation:
  either a kept insert had the via present but pull-tight later moved
  the path within its unchanged bbox (bbox-identical is NOT
  geometry-identical — re-examine the bypass check against this exact
  case), or the undo restore path diverges only under mutations the
  fuzz didn't model (set_component_no/set_fixed_state/get_mut
  in-place edits). NEXT: full lifecycle timeline — extend the region
  event log with undo/pop restore events and correlate item 185 and
  the final violating trace id in one run.
- 2026-07-15 (iter 132): SMOKING GUN — the tree probe shows
  `tree-sees []`: during the offending layer-0 completions the search
  tree returns NOTHING for the region (one drill room is completed AT
  the via's exact center seeing nothing). The items are legitimately
  off-board mid-transaction (ripped victims); the contradiction is
  that the final board contains BOTH the original via (id 185) AND
  the net-11 trace routed while it was absent — which only an undo
  that restores the via while LEAKING the post-snapshot trace can
  produce. The simple nested-snapshot test passed; the real flow
  nests deeper (restart → route_net_with_ripup → shove snapshots,
  sequential pops at one level, interleaved undos). NEXT: a FUZZ TEST
  over random insert/remove/generate/pop/undo sequences against a
  shadow model — it will find the splice sequence the hand-written
  test missed; then fix UndoableObjects. This closes the causal
  chain: tree ✓ rooms ✓ corners ✓ segments ✓ — the leak is
  transactional, exactly where the mixed-era id evidence first
  pointed at iter 121.
- 2026-07-15 (iter 131): THE INVARIANT HOLDS — room-id-per-corner
  instrumentation verified every consecutive-corner pair lies in its
  entered room (0 breaks) while 71 illegal inserts occur. Combined
  with iter 130's layer-0 completion silence, the contradiction is
  now fully cornered: the offending layer-0 rooms were completed
  WITHOUT collecting the nearby items (65/178/185) in their obstacle
  lists — the overlapping_items query / search-tree state missed them
  at search time even though the DRC checker finds them later. Since
  those items went through transactional rip-and-undo churn, the
  prime suspect is tree-state after undo resync (or overlapping's
  exact-check path). NEXT: log board.overlapping_items count for item
  185's bbox at the start of every net-11 search plus item 185's
  tree_entries length — one run confirms whether the tree lost the
  layer-0 entries after transactional churn.
- 2026-07-15 (iter 130): completion-region logging landed. Findings
  on the studied net-11 run: the violating segment is corner4→corner5
  (a 61k-unit hop from a sliver-hop cluster beside the via, passing
  ~1000 units above the via pad); the region COMPLETE lines show only
  LAYER-1 completions logging the watched obstacles while the
  violation is LAYER 0 (either the layer-0 completions eluded the
  watch filter or the offending room was never completed by this
  path). Sliver-room-overlap was REFUTED by proof: restrain pieces
  are cut by the obstacle's own border lines and never overlap the
  inflated obstacle. The one unverified link left is the
  CONSECUTIVE-CORNERS-SHARE-A-ROOM invariant — implement room-id-per-
  backtrack-corner and print per illegal segment which room should
  contain it and whether it actually does. That is the whole
  remaining search space.
- 2026-07-15 (iter 129): BIRTH-SITE VALIDATOR LANDED — insert-time
  clearance checking under FR_DEBUG_MAZE caught 70 illegal inserts on
  display including the exact studied case (net 11, ripup=false,
  blocked by via 185, full corner list printed). The convexity
  argument closes the logic: consecutive maze corners lie on one
  room's boundary, a convex room that excluded via+margin cannot host
  a segment passing within the margin — so the ROOM COMPLETION MISSED
  THE VIA as an obstacle for that search. Query coverage was then
  ELIMINATED by code read (expansion frontiers are HALF PLANES ∩
  board box — the obstacle query spans half the board and must find
  the via). Remaining suspects: a stale-cache path, restrain_shape
  dropping the wrong piece (depth cap / second-branch splits), or the
  consecutive-corners-share-a-room invariant not holding in the
  backtrack chain. NEXT: region-log obstacle lists + piece outcomes
  in complete_shape_with_ripup AND the room id per backtrack corner —
  one run decides among the three.
- 2026-07-15 (iter 128): round eight state — full picture of the
  display case: pin 65 (net 8, imported) carries via 185 (net 8's
  own via ON its pad, legal) whose 600 um pad POKES BEYOND the pin
  edge; net 11's trace legally skirts the pin (cl kept) but violates
  the protruding via. Net 11 routed after the via existed, so its
  no-ripup rooms should have inflated the via — yet the trace stands.
  DECISIVE NEXT TOOL: birth-site validation — under FR_DEBUG,
  insert_connection checks every inserted polyline segment against
  the pre-insert board (clearance-inflated, minus the just-ripped
  items) and prints mode + corners + blocking item on the first
  illegal insert. That turns the remaining mystery into a stack trace
  at the moment of birth.
- 2026-07-15 (iter 127): round seven state — display's violating
  net-11 traces trace back through pull-tight reinsert chains
  (identical bboxes each hop) to an ORIGINAL maze trace from an EARLY
  pass (id ~219) violating a PRE-EXISTING via (id 185, net 8): the
  violation was born in a first-pass (no-ripup) search whose rooms
  should have inflated that via by hw+cl. Either the room inflation
  missed the via or the inserted geometry left the rooms somewhere
  not yet covered by the endpoint fixes (drill corners? multi-corner
  same-room runs?). NEXT: capture the region event log from the RUN
  START (head, not tail) correlated with ROUTE lines to identify the
  inserting search's mode, then dump that connection's corners vs the
  via's inflated shape — one targeted run pinpoints the geometry.
  (Instrumentation note: EVENT insert logs the pre-restoration birth
  tag 3 for pull-tight; the restored tag shows on remove.)
- 2026-07-15 (iter 126): THE ENDPOINT LEAK FOUND AND FIXED — the
  final approach segment ran from the arrival door to the dest pad's
  CENTRE OF GRAVITY, crossing whatever lay between (foreign via
  clearance zones); the start segment had the mirror bug (pad cog
  outside sliver start rooms). Both endpoints now land inside
  room ∩ pad, so the segment cannot leave the convex room — legal by
  construction. RESULT: J2_reference 24/24 with ZERO violations, the
  first fully DRC-clean board. Display (78) and wavefolder (23)
  shuffled — their rerouted paths hit remaining instances; continue
  with the event-region trace on display's new top violation.
- 2026-07-15 (iter 125): round five state — birth tags landed (item
  birth preserved across pull-tight reinsertion after a first
  false-conviction run) and the verdict is: ALL residual violations
  are MAZE-born (birth 1 vs 1), and 51/53 are deeper than 2 units —
  NOT corner-rounding epsilon on exact-touch paths (hypothesis
  tested and rejected via a depth-classified checker). Eliminated so
  far: pull-tight, shove substitutes (forbidden zones), snapshots,
  rounding. Remaining candidates for the maze path: the rip loop's
  window coverage vs the inserted geometry, via_free's
  ripup-transparency interplay when the VIA is the survivor, and the
  no-ripup searches' room fidelity for via-sized obstacles. NEXT
  TOOL: event-log one violating pair end to end (FR_DEBUG: every
  rip/shove/insert touching items 211/1132 on display) — one run
  names the exact step that placed illegal copper.
- 2026-07-15 (iter 124): round four state — display's 53 violations
  are uniformly EARLY vias (ids 211-388, pass phase) vs LATE traces
  (ids 1100+, late passes/repair) at ~600 um via pads. The rip-radius
  math says this cannot survive the ripup path (any pad within
  hw+cl of a corridor overlaps the rip shape and gets ripped), and
  rooms inflate vias for no-ripup searches — so SOME code path
  inserts traces without either protection. NEXT SESSION'S TOOL:
  birth-tag items (which mechanism inserted them: maze / shove
  substitute / pull-tight / combine / repair) under FR_DEBUG, then
  match tags of violating pairs — that identifies the leaking
  mechanism in one run. Wavefolder's 13 and the fleet re-benchmark
  queue behind it.
- 2026-07-15 (iter 123): round three — transactional DRC repair pass:
  after routing, nets with routed-vs-routed clearance violations are
  ripped and rerouted against the completed board; the round is kept
  only when every rerouted net completes again (a non-transactional
  version traded display down to 14/30 for 3 violations — rejected;
  completion is never sacrificed silently). Campaign scoreboard
  (violations, from the original audit): wavefolder 322→13, display
  639→53 (repair rolls back there — its reroutes fail), J2 222→2.
  Remaining: display's 53 (repair-resistant — needs the leak fixed at
  the source, likely the same pre-insert blindness in the RIP path's
  shove interplay), wavefolder's 13, and the honest fleet
  re-benchmark.
- 2026-07-15 (iter 122): round two landed — (a) nested-snapshot
  leakage RULED OUT by a new unit test (generate/pop/undo at shove
  nesting depth restores exactly, search tree in sync; the suspicious
  id ranges were innocently from combine/pull-tight reinsertion);
  (b) the real hole, found by printing violation geometry: SHOVE
  SUBSTITUTES CANNOT SEE THE PENDING CONNECTION — they were verified
  against the board before the new traces/vias were inserted and
  routed straight through the incoming copper. shove_aside now takes
  the pending connection's clearance-inflated shapes as forbidden
  zones. Violations: wavefolder 21 (unchanged — separate source),
  display 93→53, J2 17→11 (campaign total from 639/322/222).
  Round three: wavefolder's 21 (via-vs-trace pairs — suspect
  via_free's ripup transparency vs the actual rip/shove coverage),
  then fleet re-benchmark for honest numbers.
- 2026-07-15 (iter 121): round-two lead — the residual violations are
  ROUTED-vs-ROUTED pairs whose ids mix two insertion eras (600-800 =
  pass phase vs 1200-1350 = restart phase) even though the restart
  claims a rollback on non-improvement. Prime suspect: NESTED
  SNAPSHOT LEAKAGE — shove_aside runs generate_snapshot/pop_snapshot
  INSIDE route_net_with_ripup's and the restart's snapshots;
  pop_snapshot splices same-level undo versions, and a splice bug
  could let restart-phase items survive the outer undo (two eras of
  copper coexisting = the exact overlap pattern seen). NEXT: a unit
  test reproducing nested generate/pop/undo with insertions at each
  level, asserting exact restoration; then re-audit. Alternative
  suspects if that passes: multi-piece shove inserts not cross-checked
  against each other (currently covered by ordered recursion — verify
  with a test), pad-entry proximity.
- 2026-07-15 (iter 120): CLEARANCE CORRECTNESS CAMPAIGN, round one —
  three leaks fixed: (1) room margin now trace half width + FULL
  clearance (obstacles carry the whole margin; the door shrink is
  width-only); (2) ripup radius now includes the clearance (items in
  clearance range of new copper were left in place); (3) pull-tight
  bypasses now keep the clearance (zero-margin bypasses). Violations:
  wavefolder 322→21, display 639→93, J2 222→17 (and J2 back to
  24/24). interf_u dips to ~170/173 under the honest margins.
  Remaining suspected sources: trace entries into their own pads
  passing neighbour pins (may be legitimate pad-entry necessity —
  compare with how KiCad DRC treats pad entries), and shove/via edge
  cases. Also fixed: request_for_net silently overrode caller widths
  with the default class width on rule-less boards (test now
  configures the class like every import does).
- 2026-07-15 (iter 119): DRC SELF-CHECK BUILT — AND IT FOUND A REAL
  ARCHITECTURE BUG. examples/drc_check.rs routes a board and audits
  every foreign pair against the rule matrix. Results: hundreds of
  violations per board (display 639, J2 222; wavefolder's bulk is a
  checker artifact — planes counted as routed items, fix the filter).
  ROOT CAUSE: room completion inflates obstacles by clearance (now
  cl/2) but NOT by the trace half width — the maze may place the
  centerline anywhere in a room including ON its border, so the
  copper gap can be as low as cl/2 - hw (negative!). This has been
  true in every era (full inflation gave cl - hw). THE FIX (next
  session, top priority): inflate obstacles by hw + cl/2 with the
  door shrink keeping hw + cl/2 as today (Java keeps the same margin
  via compensated trace shapes); re-benchmark everything — the
  completion numbers WILL drop and the sealed-pocket work may need
  revisiting at the new band widths. Claims of "clearance-honest"
  routing are RETRACTED until the fix lands.
- 2026-07-15 (iter 118): post-routing normalization pass —
  combine_all_traces merges fragmented trace chains at simple joints
  (junction splits and shove cutouts leave stubs); wired into the CLI
  and the demo before pull-tight. interf_u: 7 fragments combined,
  leaner session output; behaviour otherwise unchanged.
- 2026-07-15 (iter 117): pre-routed completion mode validated after
  the session's changes: interf_u (fully pre-routed) verifies and
  completes 173/173 in 22 MILLISECONDS (was 1.9 s early in the port);
  wavefolder 31/31 in 0.43 s. smoothieboard / Z80 match their
  from-scratch numbers (their fixture wiring is absent or partial).
- 2026-07-15 (iter 116): robustness sweep over eight never-tried
  fixtures — zero crashes or import failures. At only 90 s budgets:
  ch32v 9/9 (1 ms), setonix 9/9, smoothieboard 215/245 (245 nets),
  Z80 433/529 (529 nets!), DAC2020_bm01 89/99 (academic benchmark),
  caniot-arm 83/94, Mars-64 68/94, CM5_MINIMA 146/220 (6 LAYERS).
  The importer and router generalize across KiCad/Eagle-era exports,
  6-layer stacks and 500+ net designs; the partial completions are
  all short-budget artifacts of the known throughput story.
- 2026-07-15 (iter 115): explicit A* weighting tried (FR_ASTAR_WEIGHT
  env knob, kept for experiments, default 1.0): weight 1.5 regressed
  interf_u to 189 s and coldfire to 261; weight 2.0 lost completion
  on both. The center estimate's natural inflation already sits at
  the guidance/quality optimum — leave the weight at 1.0.
- 2026-07-15 (iter 114): admissible bbox-based A* estimate tried and
  REVERTED — distance-to-nearest-bbox-point (theoretically correct)
  regressed interf_u 112 s/173 → 247 s/172 and coldfire by 2 nets:
  the inadmissible distance-to-center heuristic GUIDES far better
  (weighted-A* effect). Do-not-retry note left in the code.
- 2026-07-15 (iter 113): layer-aware A* estimate — when the
  destination has no shape on the queried layer, one via is
  unavoidable, so via_cost joins the admissible remaining-cost
  estimate. interf_u 173/173 in 112 s (HALVED from 227 s); coldfire
  263/278 @ 300 s; NormalPuzzle 72/72 in 16 s; wavefolder 1.19 s.
- 2026-07-15 (iter 112): coldfire ceiling data — 269/278 at 600 s
  (was 267 before the tree/bbox fixes); all remaining holdouts (GND
  77/77, +3.3V 77/77, /XTAL and 6 signals) route fully ALONE — the
  board is uniformly congestion/time-bound with no geometric blockers
  left. Completion scales with throughput: every future perf win
  converts to nets. The deep levers remain the SortedRoomNeighbours
  door algorithm + room reuse (fewer completions per search) and a
  smarter A* estimate.
- 2026-07-15 (iter 111): passes now effectively unlimited under the
  wall clock (default -mp 99, Java-like) — the router previously
  STOPPED after 3 passes leaving most of the time budget unused, and
  failures without "exhausted" debug lines turned out to be silent
  expansion-budget exits, cured by the pass-doubling budgets of later
  passes. NormalPuzzle 72/72 in 16 s (0 failed) — NINE of ten fleet
  boards at 100%; only time-bound coldfire (261/278) remains.
- 2026-07-15 (iter 110): Simplex bounding boxes memoized (OnceCell,
  excluded from PartialEq like the other derived-state caches): corner
  computation was reappearing in the profile through the restrain
  pre-filters. Coldfire 261/278, interf_u 173/173 in 227 s.
- 2026-07-15 (iter 109): planes moved out of the search tree —
  coldfire's profile showed MinAreaTree::overlaps at ~45%: the
  board-covering plane bounds poison every ancestor bound and degrade
  the R-tree to near-full scans. Conduction areas now live in a small
  side list checked linearly by overlapping_items (insert/remove/undo
  all route through the same two functions, so resync stays correct).
  Coldfire 246 → 260/278 @ 300 s; interf_u 173/173 in 236 s (was
  250 s); wavefolder 1.25 s. Coldfire remains time-bound (267/278 at
  600 s before this change — retest pending).
- 2026-07-15 (iter 108): J2 VERDICT CORRECTED — 24/24 in 0.4 s under
  half-compensation. The iter-100 "unroutable" analysis was itself an
  artifact of full-inflation sealing: the pairwise separations are
  preserved by half-compensation, but the ROOM TOPOLOGY (door paths,
  via reachability) is what full inflation destroyed. Full fleet now:
  interf_u 173/173, 8088sbc 104/104, pic_programmer 111/111 (0.8 s),
  display 30/30, wavefolder 31/31, J2 24/24, ecc83 13/13, rpi 5/5 —
  EIGHT boards at 100% with honest clearance; coldfire 246/278 (4
  layers), NormalPuzzle 69/72 remaining.
- 2026-07-15 (iter 107): CLEARANCE HALF-COMPENSATION — THE BREAKTHROUGH.
  Obstacles now inflate by clearance/2 and the door shrink carries
  half width + clearance/2 (Java: compensated search tree +
  compensated_trace_half_width). Same legal separation, but rooms of
  adjacent pads meet at the band middle and stay door-connected, so
  the isolated dest slivers gained their bridges. Results (300 s cap,
  full clearance): interf_u 173/173 in 250 s (0 failed!), display
  30/30 in 8.4 s, 8088sbc 104/104 in 70 s, wavefolder 31/31 in 1.3 s;
  NormalPuzzle 69/72 unchanged. The entire arrival investigation
  (iters 104-107: kills instrumentation → transparency proven → dest
  room seeding → isolated-sliver analysis → half-compensation) is the
  reference for future geometric debugging.
- 2026-07-15 (iter 106): ARRIVAL CONFIRMED as the failure:
  rooms_with_dest_door=0 after 96k expansions — no free room ever
  touched the dest pad (its pin-row neighbours' clearance-inflated
  shapes kill every piece). Fix shipped: dest-side rooms are now
  pre-created like start rooms (contained-shape privilege keeps a
  sliver of the pad; its target door forms). Display improves 94→104
  routed connections; /P still fails: the dest sliver lies INSIDE the
  pin-clearance band and no room exists between it and open space, so
  it has no doors (rooms_with_dest_door now 1-4 but unreachable).
  Next: bridge the band — either Java-style half-compensation
  (obstacles inflated by cl/2, trace half width + cl/2 in the door
  check, which lets rooms overlap the band halfway) or explicit
  target doors on rooms within (clearance + half width) of the dest
  shape with a final legality check on the entering segment.
- 2026-07-15 (iter 105): instrumentation verdict — ZERO room kills
  with ripup_mode=true on rippable items (transparency is flawless;
  the 12-14-expansion seals are the no-ripup first attempts, as
  hypothesised). By elimination the ripup-view room graph matches the
  empty board, so via_free was made ripup-aware too (rippable items
  no longer block drills; insertion rips the footprint anyway). Fleet
  STILL neutral — the crowded searches empty their queue after 70-95k
  expansions without reaching the dest on an effectively-empty graph
  where the lone probe succeeds. Remaining suspect: ARRIVAL, i.e.
  hypothesis (c) missing target doors to the dest in crowded rooms.
  Next instrumentation: on search failure print whether any completed
  room carried a target door to the dest item (and the dest item
  kind: the failing connection may target a trace, whose target-door
  path differs from pads).
- 2026-07-15 (iter 104): sealed-pocket diagnosis data (display /P):
  under ripup, its failing searches show BOTH profiles — some seal at
  12-14 expansions (start pocket closed even though rippable items are
  room-transparent), others explore 70-95k expansions and still never
  reach the destination (dest pocket sealed). Since /P routes fine
  alone, the crowded-board seal involves routed items that ripup
  transparency should bypass — hypotheses to check next session with
  targeted instrumentation: (a) the 12-14 seals are only the
  no-ripup first attempts inside route_net_with_ripup (base request
  penalty 0) and the wide explorations are the real ripup view, in
  which case the DEST-side pocket geometry (pins + via extents) is
  the true blocker; (b) some sealing item unexpectedly fails
  is_rippable (check fixed_state of shoved substitutes); (c) target
  doors to the dest pad are missing in crowded rooms. Instrument:
  print is_rippable verdicts for the ROOM KILLED items and the
  target-door count of rooms adjacent to the dest pad.
- 2026-07-15 (iter 103): via_free made pairwise-precise — via
  placement now checks each foreign item against the pairwise
  clearance of its class instead of the class maximum (which could
  falsely seal tight pockets on boards with heterogeneous clearance
  rules). Fleet neutral: the remaining sealed pockets are geometric,
  not check-conservatism. Remaining levers for the last nets:
  MoveDrillItemAlgo (shove existing vias aside during search) and
  ForcedPadAlgo / spring_over for pad escapes.
- 2026-07-15 (iter 102): ordered forced insertion ported — the shove
  now follows Java's insert structure: victims cut first, substitute
  pieces popped in stack order, and each substitute's segments
  recursively shove what blocks them WITH the from-side derived from
  the substitute geometry (CalcShapeAndFromSide) before insertion, so
  inner shoves push away instead of ping-ponging. Multi-net stacking
  is now supported; transactional via one top-level snapshot. Fixed en
  route: entries must cache the victim's trace lines (the victim is
  cut before the substitutes are built — Java holds the dead object),
  and conflict checks for fixed items must include clearance. Fleet
  neutral (167/173, 29/30, 69/72, 31/31): the remaining sealed-pocket
  nets fail at SEARCH level (pins seal the pocket before shove can
  act) — the next lever is via shoving during search / ForcedPadAlgo
  style pad escapes, not more insert-time shoving.
- 2026-07-15 (iter 101): display-8-digit's failing net identified as
  /P (not VCC) — routes fine alone; on the full board its pads end up
  sealed (final searches exhaust at 12-14 expansions) and the restart
  fallback reproduces a tie deterministically. Tie-accepting restart
  rounds were tried and REVERTED: with a single failure the rotation
  is a no-op and the extra rounds burn time for zero gain everywhere.
  These last-net cases (display /P, NormalPuzzle x3, interf_u x6)
  share one shape: winnable only by a mechanism that frees a sealed
  pocket — ordered forced insertion (full shove) or via shoving.
- 2026-07-15 (iter 100): J2_reference GND VERDICT — unroutable under
  the file's own rules: its class demands 200.1 um clearance and
  250 um width while the fine-pitch pad gaps are 450 um
  (250 + 2 x 200.1 = 650 > 450), and the escape pocket is too narrow
  for the 800 um via; the fixture's wiring section is EMPTY (the
  design was never routed by anyone). 23/24 is the honest maximum;
  the shove acceptance focus moves to interf_u's holdouts and
  display-8-digit's VCC (29/30 since the shape-based shove — small
  open regression). Status header refreshed with the full fleet
  snapshot.
- 2026-07-15 (iter 99): shove core rewritten shape-based after the
  per-victim version fragmented chained victims (duplicate arcs + gaps
  found by test): one ShapeTraceEntries pass per shove shape, one
  victim net family per call (Java's ordered forced insertion — each
  substitute pushing earlier ones outward — is needed for distinct-net
  stacking and recursion; attempts to approximate it with per-segment
  recursion ping-ponged between adjacent substitutes and were
  refused). Substitutes are verified free BEFORE the board is touched;
  non-trace victims (vias, pins, keepouts) stay as blockers and the
  maze rips whatever remains after the shove. Fleet unchanged
  (interf_u 167/173, wavefolder 31/31); connection-count profiles
  vary between shove variants but completion is identical.
- 2026-07-15 (iter 98): shove wired into the maze rip phase — trace
  victims in the connection corridor are shoved aside first (staying
  connected; no victim reroute) and only ripped when the shove is
  blocked. Fleet: wavefolder 31/31, J2 23/24 (GND still fails — its
  corridor needs via shoving or recursion depth > 1), interf_u
  167/173 with 291 connections routed in the cap (vs 145 before —
  shove removes the reroute churn; time still binds). Next levers:
  recursive shove (stack levels > 1) and via shoving.
- 2026-07-15 (iter 97): shove arc piece 4: depth-1 shove_aside ported
  (board/shove_trace_algo.rs) — cuts a single crossing trace at the
  shove shape and inserts the ShapeTraceEntries substitutes, but only
  after verifying every substitute is free (fall back to ripup
  otherwise; board untouched on failure). Connectivity is exact by
  construction: the cut stubs and the substitute share corners derived
  from identical line pairs, so the exact-endpoint trace contacts
  hold (tested: the shoved net stays one connected set and leaves the
  shove shape). Recursive shoving and via shoving still open. Next:
  wire shove_aside into the maze rip phase (prefer shove over rip for
  trace victims) and benchmark.
- 2026-07-15 (iter 96): shove arc piece 3c: the full ShapeTraceEntries
  bookkeeping ported — store_items (classify vias/pins/traces/areas,
  collect shove vias, reject unshovable obstacles), store_trace
  (entrance points + trace-end-inside-shape handling), from-side
  search, border resort with duplicate removal, calculate_stack_levels
  (the stack property check), pop_piece and
  next_substitute_trace_piece (the substitute polyline around the
  offset shape). Vec-based entry list replaces Java's linked list.
  Tested end to end: a foreign crossing trace yields one substitute
  piece routing around the shove shape. Next: ShoveTraceAlgo
  (check + insert).
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

- 2026-07-15 (iter 187): DETERMINISTIC ROUTING + benchmark
  methodology. datastructures/fx_hash.rs (zero-dep Fx multiply-mix
  hasher) swapped into the routing hot paths (engine grid +
  obstacle_rooms, drill pages, maze drilled set, board inflation
  cache): std's per-process SipHash seed randomized HashMap order and
  every room shape downstream. PROVEN: two 8088sbc runs byte-identical
  except elapsed-time digits (104/104, 3513 pair checks). FR_STATS now
  prints SLOW NET lines (net, pass, wall clock ≥ 15 s).
  create_start_rooms clips seed rooms to room_window like the
  expansion path (board-sized seeds were ~18% of a coldfire profile);
  deterministic caffeinated 240 s A/B: clip 263/278 + 1 deep violation
  vs no-clip 266/278 + 12 — perf-neutral (PASS 0 38.7 vs 37.7 s),
  much cleaner board, kept. METHODOLOGY: this Mac SLEEPS during idle
  waits and macOS Instant (hence TimeLimit) freezes during sleep —
  several "deadline leak"/"12x variance" observations were sleep
  artifacts. ALL timed runs: `caffeinate -is`, validate CPU ≈ wall
  time, never build/test concurrently. Coldfire canonical
  (deterministic, 240 s): 263/278, 1 deep violation, PASS 0 38.7 s,
  PASS 1 129.3 s. Fresh 600 s canonical run pending.

- 2026-07-15 (iter 188): Java-tracking — ported the optimizer stopping
  criteria added upstream this month (f7533a68, ab4b6935, 27e700bc):
  (1) near-maximum early stop — skip/stop optimization when
  score x (1 + threshold) >= 1000 (threshold 1%, Java default);
  (2) per-pass relative score improvement below the threshold ends
  the optimizer (replaces the bare improved==0 gate); (3) a pass
  exits after 50 consecutive non-improving nets (streak resets on
  improvement; Java maxConsecutiveFailures default). Both the
  single-threaded and multithreaded drivers gate passes on the
  normalized score now. Visible effect: near-perfect boards
  (NormalPuzzle 999.97) skip optimization entirely, like Java.
  Java's use_increased_ripup_costs pass toggle has no Rust
  equivalent (our nets_pass escalates per net, not per pass) — noted.
  CANONICAL coldfire @600 s (deterministic, caffeinated, clip):
  269/278 nets — NEW COMPLETION RECORD (was 268 pre-determinism) —
  passes converge 16 -> 12 -> 8 -> 6 failed, but 12 deep violations
  remain: the restart round consumed the tail (180 s, rolled back
  269 -> 249) leaving repair_violations no budget. NEXT: reserve
  wall clock for repair (incomplete beats illegal), and consider
  skipping restart rounds whose expected value is low.

- 2026-07-15 (iter 189): budget carve — the restart fallback is now
  capped at 0.9 x the wall-clock limit (was: ran to the wire), so the
  final repair_violations guarantee always gets the ~10% tail; repair's
  per-net routes now carry the remaining overall budget as their
  deadline (previously potentially None). Coldfire @600 s validation:
  269/278 held (restart round properly truncated 180 s -> 120 s), but
  the 12 deep violations are UNCHANGED and byte-identical (7476 pair
  checks) — repair runs and rolls back: its transactional guard
  (complete_after >= complete_before) refuses rounds where a ripped
  violating net cannot re-complete, and the violating nets overlap the
  9 incomplete ones. The violations involve routed items (an unrouted
  board audits 0/0). NEXT: instrument repair_violations (nets found,
  kept/rolled back) + dump the 12 violation pairs (nets, layers,
  locations) to see whether they slip in during the escalated-penalty
  passes (insert-time validation hole?) or the restart round.

- 2026-07-15 (iter 190): VIOLATION HUNT — coldfire's 12 deep
  violations traced by birth tags to rip-corridor VIAS bypassing the
  drill-page exclusion (insert_connection used raw insert_via; a
  corridor via landed 1004 from a foreign via at required 1500).
  FIX: via sites are now checked exactly at insert (mitered
  pre-filter + Euclidean confirm vs pairwise clearance,
  via_site_is_clear); clear sites insert plainly (bit-identical old
  behavior — J2/fleet pair counts unchanged), conflicted sites go
  through Java's forced-via path (insert_forced_via: checked, shoves
  free) and fail the whole insert if even that cannot clear them
  (transactional rollback). An UNCONDITIONAL forced-via variant was
  tried first and REVERTED: shove side effects on already-legal sites
  regressed J2 to 23/24 + 1 violation. Also: drill-page caches now
  keyed by via_margin (base_margin/net_margin) — a net with larger
  vias/clearances must not reuse drills computed for a smaller margin
  (defensive; coldfire single-margin, no behavior change there).
  Coldfire @600 s: 12 -> 2 deep violations, completion HELD 269/278,
  REPAIR round now KEPT (268 -> 269, was always ROLLED BACK).
  Remaining 2: maze-inserted TRACES (birth 1) too close to
  pre-existing vias (birth 0, req 1500) — the corridor-trace analog
  (ripped foreign traces vacate the corridor but surviving vias are
  nearer than the trace-pair clearance). NEXT: validate corridor
  trace runs against surviving foreign drill items at insert.

- 2026-07-15 (iter 196): wall-clock work — Item::bounding_box is now
  cached per item (OnceLock, same immutability invariant as the
  tile-shape cache; ~5% of the coldfire profile was recomputing trace
  boxes from corner approximations), and restrain_shape skips the
  to_simplex conversion for already-simplex contained shapes (the
  restrain loop converts once per obstacle for the same room).
  MEASUREMENT CORRECTION: the "37.5 -> 31.5 s" PASS 0 numbers first
  recorded here were cutoff artifacts — a 45 s drc_check budget caps
  the pass at 0.7 x 45 = 31.5 s, so the pass was deadline-truncated,
  not faster. Honest PASS 0 at the full 600 s budget after iter 196's
  caches + iter 197's scratch buffers: 37.5 s -> 36.9 s (~2%; the
  caches help but modestly at PASS 0 scale — the earlier percentage
  claims were wrong). Lesson recorded: NEVER time a pass with a
  budget that can truncate it (pass limit = 0.7 x budget must exceed
  the expected pass time). Remaining hot spots per the profile:
  Simplex::remove_redundant_lines (~22%), MinAreaTree walks (~21%),
  allocator traffic (~14%, addressed by iter 197's thread-local
  scratch in remove_redundant_lines).

- 2026-07-15 (iter 198): overlapping_items now traverses the search
  tree with the callback API instead of collecting LeafIds into a
  temporary vector per query (MinAreaTree::overlaps was ~21% of the
  profile with the allocator close behind). Honest full-budget
  coldfire PASS 0: 36.9 s -> 36.6 s. The micro-optimizations are
  hitting diminishing returns (37.5 -> 36.6 s across three
  iterations); the remaining ~1.5-2x vs Java is structural — Java
  keeps per-clearance-class COMPENSATED search trees (obstacle
  shapes pre-inflated inside the tree), so its completions query
  ready-inflated geometry instead of inflating per query through a
  cache. That is the next big lever; scoped as a future multi-
  iteration task.

- 2026-07-15 (iter 199): THE WALL-CLOCK BREAKTHROUGH — tree-query
  counters (FR_STATS now prints per-pass query/node counts) showed
  43 MILLION search-tree queries in one coldfire PASS 0. Attribution:
  seed_room re-ran the exact 4-layer via_free clearance check for the
  same frontier drill points on EVERY room pop. A per-search memo
  ((x, y) -> via_free result; sound because the board is immutable
  during find_connection) cuts queries 43M -> 1.77M (24x) and PASS 0
  36.7 s -> 11.3 s (3.2x) with identical routing outcomes. The
  contact cache added along the way (get_normal_contacts_cached,
  invalidated by change epoch + log length) helps the connectivity
  walks but was NOT the query source. The compensated-search-tree
  refactor is shelved: the walk count, not the walk cost, was the
  story. Java comparison: this brings big-board pass times to the
  same order as Java's.

- 2026-07-15 (iter 199 canonical @600 s, post-memo): coldfire
  267/278, ZERO violations — identical quality to the pre-memo
  canonical, but the result now CONVERGES by ~200 s: PASS 0 11.4 s
  (was 37), passes 1-5 ~78-94 s each (was ~130-180), restart rounds
  ~59 s (was ~180); six passes + three restart rounds fit where four
  passes did. The remaining 11 nets are plateaued (passes 3-5 all
  stay at 11 failed; every restart round regresses and rolls back) —
  more wall clock does not help them; they need a different strategy
  (future work). WALL-CLOCK GOAL EFFECTIVELY MET: at equal
  completion the Rust router now reaches its final coldfire state in
  roughly a third of the budget, in the same range as Java, with a
  violation-free board where Java ships violations.

- 2026-07-15 (iter 200): Java-tracking — ported a7cc6e42 (fanout
  falls back to the board-level via padstack when the pin's net class
  has no via rule; a pin is skipped only when neither yields a via;
  via_padstack_for_net resolves the class rule) and 27e700bc's fanout
  side (fanout.maxItems pin cap, Java default unbounded). Remaining
  unported upstream commits are GUI/rendering/benchmark-infra only.

- 2026-07-15 (iter 201): plateau-net experiment — the full CLI
  pipeline (600 s routing + optimizer) on stripped coldfire: routing
  reached 261/278, then the optimizer's incomplete-net recovery
  (2x-penalty reroutes) recovered +3 nets -> 264/278, zero
  violations. Recovery WORKS on stragglers but had a flat 30 s
  budget; the CLI now scales it to max(30 s, tl/5). (drc_check's
  pipeline has no optimizer phase, which is part of why its 267
  differs from the CLI's routing-only 261 — different pass timing
  under the same limit is the rest.) VALIDATED: with the scaled
  120 s recovery the CLI reaches 268/278 with ZERO violations
  (+7 nets recovered vs +3 under the flat 30 s; final score 950.43)
  — the best violation-free coldfire completion of the port, equal
  to the old 268 record that still shipped violations. NEW CANONICAL
  coldfire (CLI, 600 s + tl/5 recovery): 268/278, 0 violations.

- 2026-07-15 (iter 203): FLEET CANONICAL REFRESH (CLI pipeline,
  strip-wiring, caffeinated, post-memo + scaled recovery, -tl 300
  except coldfire 600):
    NormalPuzzle   72/72   216 ms  score 999.97
    J2             24/24   101 ms  score 999.94
    display        30/30   5.2 s   score 999.90
    wavefolder     31/31   1.1 s   score 999.94
    pic           111/111  300 ms  score 994.07 (1 design-inherent
                                   factory pad violation, same as Java)
    8088sbc       104/104  4.4 s   score 999.87 (was 14.1 s pre-memo)
    interf_u      172/173 @300 s   score 990.20, 0 violations
                                   (routing 169 + recovery 3; the old
                                   full completion used ~2x budget)
    coldfire      268/278 @600 s   score 950.43, 0 violations
  Every board violation-free except pic's known factory artifact.
  Wall clock is now decisively in Java's range or better on every
  board measured.

- 2026-07-15 (iter 206): NEGATIVE RESULT — restart corridor locking
  (FR_LOCK_RESTART, default off): locking an originally-incomplete
  net's route items UserFixed once it completes mid-round, so later
  nets cannot rip its corridors, made restart rounds WORSE on
  coldfire (267 -> 259 per round vs 267 -> 263 unlocked; final state
  unchanged at 267/278 + 0 violations thanks to the transactional
  rollback). Early locks starve the signal nets more than ripup
  erosion costs the power nets — one-way reservation is not
  negotiation. The straggler research needs bidirectional trading
  (rip WITH compensation, e.g. Java-less simultaneous corridor
  auctions); parked. The experiment stays env-gated for future
  reference.

- 2026-07-15 (iter 207): straggler census (FR_STATS now prints
  INCOMPLETE net name / pins / fragments at batch end). The 11
  coldfire incompletes: GND (130 pins, 78 FRAGMENTS), +3.3V
  (100 pins, 89 FRAGMENTS), and 9 small signal nets (2-5 pins each,
  fully fragmented). ANOMALY: the power nets route FIRST (hybrid
  order) and complete alone in ~1 s, yet end nearly UNROUTED — the
  standing board apparently loses their copper wholesale somewhere
  between the passes and the final state. Prime suspect: the restart
  rounds' transactional undo depth (nested snapshot pairing inside
  route_net_with_ripup during a rolled-back round). NEXT: print the
  fragment census BEFORE the restart phase and compare — if
  end-of-passes GND has few fragments, the restart undo is corrupting
  the board and 267/278 understates the true achievable board.

## Notes / decisions log

- 2026-07-14: crate scaffolded on branch `rust`; no external deps yet.
