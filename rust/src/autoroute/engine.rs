//! Port of the core of `AutorouteEngine.java` (room graph maintenance)
//! with a simplified frontier expansion in place of
//! `SortedRoomNeighbours.java`'s sorted-edge-gap algorithm (deferred).
//!
//! The engine owns the room graph for one routed net. Completed
//! free-space rooms never overlap: new rooms are restrained against the
//! shapes of existing complete rooms in addition to the board obstacles
//! (Java achieves the same by inserting complete rooms into the autoroute
//! search tree). Frontier expansion seeds an incomplete room beyond each
//! border edge of a room; edges whose far side is already covered
//! restrain away to nothing, so expansion terminates naturally.

use crate::autoroute::expansion_room::{RoomGraph, RoomId, RoomKind};
use crate::autoroute::room_completion::{restrain_shape, IncompleteRoom};
use crate::board::basic_board::{BasicBoard, ItemId};
use crate::geometry::planar::TileShape;

/// A door to an own-net target item reachable from a room
/// (Java: `TargetItemExpansionDoor`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TargetDoor {
    pub item: ItemId,
    pub shape_index: usize,
}

pub struct AutorouteEngine {
    pub net_no: i32,
    pub graph: RoomGraph,
    /// If true, rippable foreign route items do not restrain rooms; the
    /// maze search pays a penalty to pass through them.
    pub allow_ripup: bool,
    /// The clearance class of the routed trace (obstacles restrain rooms
    /// inflated by the pairwise clearance to this class).
    pub trace_clearance_class: usize,
    /// The half width of the routed trace: part of the room margin.
    pub trace_half_width: i32,
    /// All completed free-space rooms.
    complete_rooms: Vec<RoomId>,
    /// The target doors of each room, indexed by room id.
    target_doors: Vec<Vec<TargetDoor>>,
    /// The rippable foreign items overlapping each room.
    rippable_items: Vec<Vec<ItemId>>,
    /// Rooms whose frontier was already expanded.
    expanded: Vec<bool>,
    /// Rooms whose completion skipped an own-net or rippable item: they
    /// cannot survive a net switch (Java: is_net_dependent).
    net_dependent: Vec<bool>,
    /// Obstacle expansion rooms per (item, shape index) (Java:
    /// ItemAutorouteInfo.get_expansion_room).
    obstacle_rooms: crate::datastructures::FxHashMap<(ItemId, usize), RoomId>,
    /// Drill pages for via-location candidates (Java: DrillPageArray),
    /// created on first use, cache synced against board changes.
    pub drill_pages: Option<crate::autoroute::drill_pages::DrillPageArray>,
    /// Consumed prefix of the board's change log.
    seen_log: usize,
    /// The board change epoch this graph was built against.
    seen_epoch: u64,
    /// Uniform grid over the complete rooms, per (cell_x, cell_y, layer):
    /// the phase-2 restrains, door creation and containment lookups were
    /// linear scans over all rooms, which made room reuse quadratic on
    /// many-pin nets (the reason the first reuse attempt was reverted).
    grid: crate::datastructures::FxHashMap<(i32, i32, usize), Vec<RoomId>>,
}

/// Grid cell edge in board units (coarse: cells only prune candidates).
const GRID_CELL: i32 = 32768;

impl AutorouteEngine {
    pub fn new(net_no: i32) -> Self {
        Self::new_with_ripup(net_no, false)
    }

    pub fn new_with_ripup(net_no: i32, allow_ripup: bool) -> Self {
        Self::new_with_clearance(net_no, allow_ripup, 1, 0)
    }

    pub fn new_with_clearance(
        net_no: i32,
        allow_ripup: bool,
        trace_clearance_class: usize,
        trace_half_width: i32,
    ) -> Self {
        AutorouteEngine {
            net_no,
            graph: RoomGraph::new(),
            allow_ripup,
            trace_clearance_class,
            trace_half_width,
            complete_rooms: Vec::new(),
            target_doors: Vec::new(),
            rippable_items: Vec::new(),
            expanded: Vec::new(),
            net_dependent: Vec::new(),
            obstacle_rooms: crate::datastructures::FxHashMap::default(),
            drill_pages: None,
            seen_log: 0,
            seen_epoch: 0,
            grid: crate::datastructures::FxHashMap::default(),
        }
    }

    /// Drops every room (the graph arenas are rebuilt lazily).
    pub fn clear_rooms(&mut self) {
        self.graph = RoomGraph::new();
        self.complete_rooms.clear();
        self.target_doors.clear();
        self.rippable_items.clear();
        self.expanded.clear();
        self.net_dependent.clear();
        self.obstacle_rooms.clear();
        self.grid.clear();
    }

    fn remove_complete_room(&mut self, id: RoomId) {
        // the surviving neighbours must be re-expandable: the removed
        // room's space needs fresh rooms, reachable only through them
        let neighbours: Vec<RoomId> = self
            .graph
            .room(id)
            .doors
            .clone()
            .into_iter()
            .filter_map(|d| self.graph.other_room(d, id))
            .collect();
        for n in neighbours {
            if n < self.expanded.len() {
                self.expanded[n] = false;
            }
        }
        self.graph.remove_room(id);
        // grid entries and the complete_rooms slot are filtered lazily by
        // the alive flag
    }

    /// Brings the cached room graph up to date with the board: a changed
    /// epoch (undo/redo/snapshot pop) drops everything; otherwise rooms
    /// overlapping the regions changed since the last sync are removed
    /// (Java: additional_update_after_change on every item change).
    pub fn sync_board_changes(&mut self, board: &BasicBoard) {
        if board.change_epoch() != self.seen_epoch {
            if crate::debug::maze() {
                eprintln!(
                    "SYNC net {} EPOCH {} -> {} (full clear)",
                    self.net_no,
                    self.seen_epoch,
                    board.change_epoch()
                );
            }
            self.clear_rooms();
            self.seen_epoch = board.change_epoch();
            self.seen_log = board.change_log().len();
            return;
        }
        let log = board.change_log();
        if self.seen_log >= log.len() {
            return;
        }
        let debug = crate::debug::maze();
        let before = self.seen_log;
        let mut removed_rooms = 0usize;
        let matrix = &board.rules.clearance_matrix;
        for &(layer, bbox) in &log[self.seen_log..] {
            // rooms were restrained by the item inflated by up to
            // hw + max clearance (+ safety), with miter reach ≤ 2×
            let slack = 2.0
                * (self.trace_half_width
                    + matrix.max_value(layer).max(0)
                    + crate::rules::clearance_matrix::CLEARANCE_SAFETY_MARGIN)
                    as f64;
            let query = bbox.offset(slack);
            for room in self.rooms_near(query, layer) {
                if self.graph.room(room).shape.bounding_box().intersects(query) {
                    self.remove_complete_room(room);
                    removed_rooms += 1;
                }
            }
        }
        self.seen_log = log.len();
        if debug {
            eprintln!(
                "SYNC net {} log {}..{} removed {} rooms",
                self.net_no,
                before,
                log.len(),
                removed_rooms
            );
        }
    }

    /// Switches the engine to another net, keeping the net-independent
    /// rooms (Java: AutorouteEngine.init_connection with
    /// maintain_database). Net-dependent rooms and rooms holding target
    /// doors are dropped, as are rooms overlapping the new net's items
    /// (which they treat as obstacles — the new net must reach its pads).
    pub fn switch_net(&mut self, board: &BasicBoard, net_no: i32) {
        if net_no == self.net_no {
            return;
        }
        for id in 0..self.graph.room_count() {
            if !self.graph.room(id).alive {
                continue;
            }
            if self.net_dependent[id] || !self.target_doors[id].is_empty() {
                self.remove_complete_room(id);
            }
        }
        let matrix = &board.rules.clearance_matrix;
        let net_items: Vec<ItemId> = board
            .items()
            .filter(|(_, it)| it.base.contains_net(net_no))
            .map(|(id, _)| *id)
            .collect();
        for item_id in net_items {
            let Some(item) = board.get_item(item_id) else {
                continue;
            };
            let regions: Vec<(usize, crate::geometry::planar::IntBox)> = item
                .tile_shapes(&board.padstacks)
                .iter()
                .map(|(s, l)| (*l, s.bounding_box()))
                .collect();
            for (layer, bbox) in regions {
                let slack = 2.0
                    * (self.trace_half_width
                        + matrix.max_value(layer).max(0)
                        + crate::rules::clearance_matrix::CLEARANCE_SAFETY_MARGIN)
                        as f64;
                let query = bbox.offset(slack);
                for room in self.rooms_near(query, layer) {
                    if self.graph.room(room).shape.bounding_box().intersects(query) {
                        self.remove_complete_room(room);
                    }
                }
            }
        }
        if crate::debug::stats() {
            let alive = (0..self.graph.room_count())
                .filter(|&r| self.graph.room(r).alive)
                .count();
            eprintln!(
                "SWITCH {} -> {net_no}: {alive} rooms survive of {}",
                self.net_no,
                self.graph.room_count()
            );
        }
        self.net_no = net_no;
    }

    fn grid_cells(bbox: crate::geometry::planar::IntBox) -> impl Iterator<Item = (i32, i32)> {
        let (x0, x1) = (
            bbox.ll.x.div_euclid(GRID_CELL),
            bbox.ur.x.div_euclid(GRID_CELL),
        );
        let (y0, y1) = (
            bbox.ll.y.div_euclid(GRID_CELL),
            bbox.ur.y.div_euclid(GRID_CELL),
        );
        (x0..=x1).flat_map(move |x| (y0..=y1).map(move |y| (x, y)))
    }

    fn grid_insert(
        &mut self,
        room_id: RoomId,
        bbox: crate::geometry::planar::IntBox,
        layer: usize,
    ) {
        for (x, y) in Self::grid_cells(bbox) {
            self.grid.entry((x, y, layer)).or_default().push(room_id);
        }
    }

    /// The complete rooms whose grid cells overlap `bbox` on `layer`
    /// (a superset of the exactly overlapping rooms), deduplicated.
    fn rooms_near(&self, bbox: crate::geometry::planar::IntBox, layer: usize) -> Vec<RoomId> {
        let mut out: Vec<RoomId> = Vec::new();
        for (x, y) in Self::grid_cells(bbox) {
            if let Some(v) = self.grid.get(&(x, y, layer)) {
                out.extend(v.iter().copied().filter(|&r| self.graph.room(r).alive));
            }
        }
        out.sort_unstable();
        out.dedup();
        out
    }

    /// The rippable foreign items overlapping a room.
    pub fn rippable_items(&self, room: RoomId) -> &[ItemId] {
        &self.rippable_items[room]
    }

    /// Registers target doors for own-net items inserted after rooms were
    /// completed (rooms are reused across the connections of a net; the
    /// new items would otherwise be unreachable as destinations).
    pub fn register_new_targets(&mut self, board: &BasicBoard, items: &[ItemId]) {
        for &item_id in items {
            let Some(item) = board.get_item(item_id) else {
                continue;
            };
            if !item.is_connectable() || !item.base.contains_net(self.net_no) {
                continue;
            }
            for (index, (shape, layer)) in item.tile_shapes(&board.padstacks).iter().enumerate() {
                for room in self.rooms_near(shape.bounding_box(), *layer) {
                    if self.graph.room(room).layer == *layer
                        && shape.intersects(&self.graph.room(room).shape)
                    {
                        self.target_doors[room].push(TargetDoor {
                            item: item_id,
                            shape_index: index,
                        });
                    }
                }
            }
        }
    }

    pub fn complete_rooms(&self) -> &[RoomId] {
        &self.complete_rooms
    }

    pub fn target_doors(&self, room: RoomId) -> &[TargetDoor] {
        &self.target_doors[room]
    }

    /// Completes an incomplete room against the board obstacles and the
    /// already completed rooms, inserting the results into the room graph
    /// with doors to every touching complete room and target doors to
    /// own-net items. Returns the new room ids.
    pub fn complete_room(&mut self, board: &BasicBoard, room: IncompleteRoom) -> Vec<RoomId> {
        // restrain against the board obstacles
        let ignore_rippable = self.allow_ripup && !crate::debug::obstacle_rooms();
        let (mut pieces, skipped_shapes) =
            crate::autoroute::room_completion::complete_shape_tracked(
                board,
                &room,
                self.net_no,
                None,
                ignore_rippable,
                self.trace_clearance_class,
                self.trace_half_width,
            );
        // restrain against the existing complete rooms (they must not
        // overlap); the grid prunes to nearby rooms, bounding boxes prune
        // the exact overlap tests
        let query_bbox = pieces
            .iter()
            .map(|p| p.shape.bounding_box())
            .reduce(|a, b| a.union(b));
        let near = match query_bbox {
            Some(bb) => self.rooms_near(bb, room.layer),
            None => Vec::new(),
        };
        for existing in near {
            if self.graph.room(existing).layer != room.layer {
                continue;
            }
            let existing_shape = self.graph.room(existing).shape.clone();
            let existing_bbox = existing_shape.bounding_box();
            let mut new_pieces = Vec::new();
            for piece in pieces {
                if !piece.shape.bounding_box().intersects(existing_bbox) {
                    new_pieces.push(piece);
                    continue;
                }
                let intersection = piece.shape.intersection(&existing_shape);
                if intersection.dimension() == 2 {
                    new_pieces.extend(restrain_shape(&piece, &existing_shape));
                } else {
                    new_pieces.push(piece);
                }
            }
            pieces = new_pieces;
        }

        let mut new_rooms = Vec::new();
        for piece in pieces {
            if piece.shape.dimension() < 2 {
                continue;
            }
            let room_id = self.graph.add_room(
                piece.shape.clone(),
                piece.layer,
                RoomKind::CompleteFreeSpace,
            );
            // doors to touching complete rooms (grid + bounding boxes
            // prune the exact touch tests)
            let piece_bbox = piece.shape.bounding_box();
            for existing in self.rooms_near(piece_bbox, piece.layer) {
                if self.graph.room(existing).layer != piece.layer {
                    continue;
                }
                let existing_room = self.graph.room(existing);
                if !existing_room.shape.bounding_box().intersects(piece_bbox) {
                    continue;
                }
                let dim = existing_room.shape.intersection(&piece.shape).dimension();
                if dim >= 1 {
                    self.graph.add_door_with_dimension(existing, room_id, dim);
                }
            }
            // target doors to own-net connectable items intersecting the
            // room, and the rippable foreign items it overlaps
            let mut targets = Vec::new();
            let mut rippables = Vec::new();
            for item_id in board.overlapping_items(&piece.shape, Some(piece.layer)) {
                let Some(item) = board.get_item(item_id) else {
                    continue;
                };
                if self.allow_ripup
                    && crate::autoroute::room_completion::is_rippable(item, self.net_no)
                {
                    rippables.push(item_id);
                }
                if !item.is_connectable() || !item.base.contains_net(self.net_no) {
                    continue;
                }
                let is_trace = matches!(item.kind, crate::board::ItemKind::PolylineTrace(_));
                for (index, (shape, layer)) in item.tile_shapes(&board.padstacks).iter().enumerate()
                {
                    if *layer != piece.layer {
                        continue;
                    }
                    // trace targets need a REAL overlap: a corner touch
                    // gives no in-room centerline to tap, and the tap
                    // then lands outside the room (illegal segment)
                    let reachable = if is_trace {
                        shape.intersection(&piece.shape).dimension() >= 1
                    } else {
                        shape.intersects(&piece.shape)
                    };
                    if reachable {
                        targets.push(TargetDoor {
                            item: item_id,
                            shape_index: index,
                        });
                    }
                }
            }
            crate::autoroute::maze_search::STATS.with(|s| s.borrow_mut().rooms_completed += 1);
            // net-dependent ONLY if a skipped own-net/rippable item's
            // inflation actually overlaps this piece (its shape would
            // differ for another net)
            let net_dependent = skipped_shapes.iter().any(|sh| {
                sh.bounding_box().intersects(piece_bbox)
                    && sh.intersection(&piece.shape).dimension() >= 2
            });
            if crate::debug::maze() {
                eprintln!(
                    "NEWROOM {room_id} net {} layer {} nd {} bbox {:?}",
                    self.net_no, piece.layer, net_dependent, piece_bbox
                );
            }
            self.complete_rooms.push(room_id);
            self.grid_insert(room_id, piece_bbox, piece.layer);
            self.target_doors.push(targets);
            self.rippable_items.push(rippables);
            self.expanded.push(false);
            self.net_dependent
                .push(net_dependent || !self.target_doors[room_id].is_empty());
            debug_assert_eq!(self.target_doors.len(), self.graph.room_count());
            if crate::debug::srn() {
                self.create_gap_rooms(board, room_id);
            }
            new_rooms.push(room_id);
        }
        new_rooms
    }

    /// The obstacle expansion room of an item shape, created on demand
    /// with the item's completion inflation as its shape (Java:
    /// ObstacleExpansionRoom via ItemAutorouteInfo).
    fn get_obstacle_room(
        &mut self,
        board: &BasicBoard,
        item_id: ItemId,
        shape_index: usize,
        layer: usize,
        inflated: &TileShape,
    ) -> RoomId {
        let _ = board;
        if let Some(&r) = self.obstacle_rooms.get(&(item_id, shape_index)) {
            return r;
        }
        let room_id = self.graph.add_room(
            inflated.clone(),
            layer,
            RoomKind::Obstacle {
                item: item_id,
                shape_index,
            },
        );
        self.target_doors.push(Vec::new());
        self.rippable_items.push(vec![item_id]);
        self.expanded.push(false);
        self.net_dependent.push(true);
        self.obstacle_rooms.insert((item_id, shape_index), room_id);
        room_id
    }

    /// The item of an obstacle room, if it is one.
    pub fn obstacle_room_item(&self, room: RoomId) -> Option<ItemId> {
        match self.graph.room(room).kind {
            RoomKind::Obstacle { item, .. } => Some(item),
            _ => None,
        }
    }

    /// True if the obstacle room's trace satisfies Java's
    /// `MazeShoveTraceAlgo.check_shove_trace_line` preconditions: a
    /// polyline trace of the SAME half width and clearance class as the
    /// routed trace, so a lateral shove (rather than a rip) can make
    /// room. Such passages are charged a reduced cost — the insert-time
    /// corridor shove then slides the trace like Java's maze shove.
    pub fn obstacle_room_shovable(
        &self,
        board: &BasicBoard,
        room: RoomId,
        trace_half_width: i32,
        clearance_class: usize,
    ) -> bool {
        let Some(item_id) = self.obstacle_room_item(room) else {
            return false;
        };
        let Some(item) = board.get_item(item_id) else {
            return false;
        };
        if item.base.is_user_fixed() {
            return false;
        }
        match &item.kind {
            crate::board::ItemKind::PolylineTrace(t) => {
                t.half_width == trace_half_width && item.base.clearance_class == clearance_class
            }
            _ => false,
        }
    }

    /// Runs the SortedRoomNeighbours gap walk for a freshly completed
    /// room: collects the touching complete rooms and (inflated) items,
    /// sorts them counterclockwise, and adds the uncovered border gaps
    /// as INCOMPLETE rooms in the graph, each with a door to the piece
    /// (Java: SortedRoomNeighbours.calculate).
    fn create_gap_rooms(&mut self, board: &BasicBoard, room_id: RoomId) {
        use crate::autoroute::sorted_room_neighbours as srn;
        let layer = self.graph.room(room_id).layer;
        let room_simplex = self.graph.room(room_id).shape.to_simplex();
        let room_bbox = self.graph.room(room_id).shape.bounding_box();
        let mut neighbours: Vec<srn::Neighbour> = Vec::new();
        let mut obstacle_doors: Vec<(ItemId, usize, std::sync::Arc<TileShape>)> = Vec::new();
        // touching complete rooms from the grid
        for other in self.rooms_near(room_bbox.offset(4.0), layer) {
            if other == room_id || self.graph.room(other).layer != layer {
                continue;
            }
            let shape = self.graph.room(other).shape.clone();
            if let Some(nb) =
                srn::make_neighbour(&room_simplex, srn::NeighbourObject::Room(other), &shape)
            {
                neighbours.push(nb);
            }
        }
        // touching board items, at their completion inflation
        let matrix = &board.rules.clearance_matrix;
        for item_id in board.overlapping_items_coarse(
            &TileShape::Box(room_bbox.offset(
                2.0 * (self.trace_half_width
                    + matrix.max_value(layer).max(0)
                    + crate::rules::clearance_matrix::CLEARANCE_SAFETY_MARGIN)
                    as f64,
            )),
            Some(layer),
        ) {
            let Some(item) = board.get_item(item_id) else {
                continue;
            };
            if item.base.contains_net(self.net_no) {
                continue;
            }
            if let crate::board::ItemKind::ObstacleArea(a) = &item.kind {
                if a.is_conduction || a.via_only {
                    continue;
                }
            }
            if self.allow_ripup && crate::autoroute::room_completion::is_rippable(item, self.net_no)
            {
                continue;
            }
            let clearance = matrix.get_value(
                item.base.clearance_class,
                self.trace_clearance_class,
                layer,
                true,
            );
            let margin = (self.trace_half_width + clearance).max(0);
            let Some(inflated) = board.inflated_shapes(item_id, margin) else {
                continue;
            };
            for (si, (shape, bbox, l)) in inflated.iter().enumerate() {
                if *l != layer || !bbox.intersects(room_bbox.offset(4.0)) {
                    continue;
                }
                if let Some(nb) =
                    srn::make_neighbour(&room_simplex, srn::NeighbourObject::Item(item_id), shape)
                {
                    // ripup via obstacle rooms: routable foreign items
                    // touching the piece become enterable rooms (Java:
                    // "expand the item for ripup and pushing purposes")
                    if crate::debug::obstacle_rooms()
                        && self.allow_ripup
                        && item.is_routable()
                        && nb.intersection.dimension() >= 1
                    {
                        obstacle_doors.push((item_id, si, shape.clone()));
                    }
                    neighbours.push(nb);
                }
            }
        }
        for (item_id, si, shape) in obstacle_doors {
            let ob = self.get_obstacle_room(board, item_id, si, layer, &shape);
            if !self.graph.door_exists(room_id, ob) {
                self.graph.add_door_with_dimension(room_id, ob, 1);
            }
        }
        // make sure there is a door to every touching complete room
        // (Java: the dim-1 branch's insert_door_ok + new ExpansionDoor)
        let room_doors: Vec<(RoomId, i32)> = neighbours
            .iter()
            .filter_map(|nb| match nb.object {
                srn::NeighbourObject::Room(r) if nb.intersection.dimension() >= 1 => {
                    Some((r, nb.intersection.dimension()))
                }
                _ => None,
            })
            .collect();
        for (other, dim) in room_doors {
            if self.graph.room(other).alive
                && !matches!(
                    self.graph.room(other).kind,
                    RoomKind::IncompleteFreeSpace { .. }
                )
                && !self.graph.door_exists(room_id, other)
            {
                self.graph.add_door_with_dimension(room_id, other, dim);
            }
        }
        if neighbours.is_empty() {
            return;
        }
        srn::sort_neighbours(&room_simplex, &mut neighbours);
        let mut completed_shape = self.graph.room(room_id).shape.clone();
        let contained = self.graph.room(room_id).shape.clone();
        let gaps = srn::calculate_new_incomplete_rooms(
            &room_simplex,
            &mut completed_shape,
            &contained,
            &neighbours,
        );
        for gap in gaps {
            let gap_id = self.graph.add_room(
                gap.shape,
                layer,
                RoomKind::IncompleteFreeSpace {
                    contained_shape: gap.contained_shape,
                },
            );
            // parallel bookkeeping arrays cover every graph room
            self.target_doors.push(Vec::new());
            self.rippable_items.push(Vec::new());
            self.expanded.push(false);
            self.net_dependent.push(false);
            self.graph.add_door_with_dimension(room_id, gap_id, 1);
        }
    }

    /// Completes an INCOMPLETE graph room in place: removes it and
    /// completes its shape, producing complete pieces (whose SRN walk
    /// queues further gaps). Returns the new complete rooms.
    fn complete_incomplete_room(&mut self, board: &BasicBoard, room_id: RoomId) -> Vec<RoomId> {
        let RoomKind::IncompleteFreeSpace { contained_shape } =
            self.graph.room(room_id).kind.clone()
        else {
            return Vec::new();
        };
        let incomplete = IncompleteRoom {
            shape: self.graph.room(room_id).shape.clone(),
            layer: self.graph.room(room_id).layer,
            contained_shape,
        };
        self.graph.remove_room(room_id);
        self.complete_room(board, incomplete)
    }

    /// Expands the frontier of a room: seeds an incomplete room beyond
    /// each border edge and completes it. Edges already covered by other
    /// rooms produce nothing. Returns the newly created rooms.
    pub fn expand_room(&mut self, board: &BasicBoard, room_id: RoomId) -> Vec<RoomId> {
        if self.expanded[room_id] {
            return Vec::new();
        }
        self.expanded[room_id] = true;
        if crate::debug::obstacle_rooms()
            && matches!(self.graph.room(room_id).kind, RoomKind::Obstacle { .. })
        {
            // an entered obstacle room connects onward like a free room:
            // touching complete rooms get doors, uncovered gaps become
            // incomplete rooms (Java runs SortedRoomNeighbours on
            // ObstacleExpansionRooms too)
            self.create_gap_rooms(board, room_id);
            let incomplete: Vec<RoomId> = self
                .graph
                .room(room_id)
                .doors
                .clone()
                .into_iter()
                .filter_map(|d| self.graph.other_room(d, room_id))
                .filter(|&r| {
                    self.graph.room(r).alive
                        && matches!(
                            self.graph.room(r).kind,
                            RoomKind::IncompleteFreeSpace { .. }
                        )
                })
                .collect();
            let mut new_rooms = Vec::new();
            for r in incomplete {
                new_rooms.extend(self.complete_incomplete_room(board, r));
            }
            return new_rooms;
        }
        if crate::debug::srn() {
            // faithful growth: complete the incomplete gap rooms behind
            // this room's doors (their pieces bring their own doors and
            // further gap rooms)
            let incomplete: Vec<RoomId> = self
                .graph
                .room(room_id)
                .doors
                .clone()
                .into_iter()
                .filter_map(|d| self.graph.other_room(d, room_id))
                .filter(|&r| {
                    self.graph.room(r).alive
                        && matches!(
                            self.graph.room(r).kind,
                            RoomKind::IncompleteFreeSpace { .. }
                        )
                })
                .collect();
            let mut new_rooms = Vec::new();
            for r in incomplete {
                new_rooms.extend(self.complete_incomplete_room(board, r));
            }
            return new_rooms;
        }
        let room_shape = self.graph.room(room_id).shape.clone();
        let layer = self.graph.room(room_id).layer;
        let mut new_rooms = Vec::new();
        for i in 0..room_shape.border_line_count() {
            let border_line = room_shape.border_line(i);
            // the half plane on the far side of this border edge
            let new_room_shape = TileShape::half_plane(border_line.opposite());
            let contained = room_shape.intersection(&new_room_shape);
            if contained.is_empty() {
                continue;
            }
            // clip the seed to a window around the contained edge: a raw
            // half-plane start makes every completion query and restrain
            // half the board's obstacles (8088sbc pass 0 spent its time
            // there); bounded rooms trade a few more expansions for far
            // cheaper completions (Java bounds rooms via divide_large_room
            // and the drill-page granularity)
            let window = crate::debug::room_window();
            let clipped = if window > 0 {
                new_room_shape.intersection_with_simplify(&TileShape::Box(
                    contained.bounding_box().offset(window as f64),
                ))
            } else {
                new_room_shape
            };
            if clipped.dimension() != 2 {
                continue;
            }
            let incomplete = IncompleteRoom {
                shape: clipped,
                layer,
                contained_shape: contained,
            };
            new_rooms.extend(self.complete_room(board, incomplete));
        }
        new_rooms
    }

    /// The complete rooms containing `point` on `layer`; if none exists
    /// yet, a room is completed around the point (used for drill targets).
    pub fn rooms_containing(
        &mut self,
        point: crate::geometry::planar::IntPoint,
        layer: usize,
        board: &BasicBoard,
    ) -> Vec<RoomId> {
        let p = crate::geometry::planar::Point::Int(point);
        let point_box = crate::geometry::planar::IntBox::new(point, point);
        let existing: Vec<RoomId> = self
            .rooms_near(point_box, layer)
            .into_iter()
            .filter(|&r| self.graph.room(r).layer == layer && self.graph.room(r).shape.contains(&p))
            .collect();
        if !existing.is_empty() {
            return existing;
        }
        // completion may produce pieces that do NOT contain the point:
        // the piece holding it can be killed by an obstacle while other
        // pieces survive. Returning those let the maze "enter" a room far
        // from the drill location and insert a connecting segment straight
        // through everything in between (the display illegal-insert class)
        let created = self.create_start_rooms(
            board,
            TileShape::Box(crate::geometry::planar::IntBox::new(point, point)),
            layer,
        );
        created
            .into_iter()
            .filter(|&r| self.graph.room(r).shape.contains(&p))
            .collect()
    }

    /// Creates and completes the start rooms around a point-like shape
    /// (e.g. the connection shape of the start item).
    pub fn create_start_rooms(
        &mut self,
        board: &BasicBoard,
        contained_shape: TileShape,
        layer: usize,
    ) -> Vec<RoomId> {
        // clip the seed to the room window like the expansion path does:
        // completing a board-sized room restrains against nearly every
        // item on the board and dominated the coldfire profile (~18%)
        let window = crate::debug::room_window();
        let bound = if window > 0 {
            board
                .bounding_box()
                .offset(1000.0)
                .intersection(contained_shape.bounding_box().offset(window as f64))
        } else {
            board.bounding_box().offset(1000.0)
        };
        let start = IncompleteRoom {
            shape: TileShape::Box(bound),
            layer,
            contained_shape,
        };
        self.complete_room(board, start)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{Layer, LayerStructure};
    use crate::core::Padstacks;
    use crate::geometry::planar::{IntBox, IntPoint, Point};
    use crate::rules::{BoardRules, ClearanceMatrix};
    use std::collections::VecDeque;

    fn test_board() -> BasicBoard {
        let stack = LayerStructure::new(vec![Layer::new("F.Cu", true), Layer::new("B.Cu", true)]);
        let matrix = ClearanceMatrix::get_default_instance(stack.clone(), 200);
        let mut rules = BoardRules::new(stack.clone(), matrix);
        rules.get_default_net_class();
        let mut padstacks = Padstacks::new(2);
        padstacks.add_shape_on_layers(
            TileShape::Box(IntBox::from_coords(-400, -400, 400, 400)),
            0,
            1,
        );
        BasicBoard::new(stack, rules, padstacks)
    }

    #[test]
    fn room_graph_reaches_target_through_doors() {
        let mut board = test_board();
        let _start_pad = board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        let target_pad = board.insert_via(1, IntPoint::new(9000, 0), vec![1], 1, false);
        // an obstacle wall of a foreign net between them with a gap below
        let wall = board.insert_trace(
            crate::geometry::planar::Polyline::from_int_points(&[
                IntPoint::new(4500, -2000),
                IntPoint::new(4500, 8000),
            ]),
            0,
            300,
            vec![2],
            1,
        );
        assert!(board.get_item(wall).is_some());

        let mut engine = AutorouteEngine::new(1);
        let start_point = IntPoint::new(0, 0);
        let start_rooms = engine.create_start_rooms(
            &board,
            TileShape::Box(IntBox::new(start_point, start_point)),
            0,
        );
        assert!(!start_rooms.is_empty());

        // breadth-first frontier expansion until a room with a target door
        // to the target pad appears
        let mut queue: VecDeque<RoomId> = start_rooms.into_iter().collect();
        let mut found = false;
        let mut steps = 0;
        while let Some(room) = queue.pop_front() {
            if engine
                .target_doors(room)
                .iter()
                .any(|t| t.item == target_pad)
            {
                found = true;
                break;
            }
            steps += 1;
            if steps > 200 {
                break;
            }
            for new_room in engine.expand_room(&board, room) {
                queue.push_back(new_room);
            }
        }
        assert!(found, "room graph never reached the target pad");

        // rooms never overlap each other 2-dimensionally
        let rooms = engine.complete_rooms().to_vec();
        for (a_pos, &a) in rooms.iter().enumerate() {
            for &b in rooms.iter().skip(a_pos + 1) {
                if engine.graph.room(a).layer != engine.graph.room(b).layer {
                    continue;
                }
                let overlap = engine
                    .graph
                    .room(a)
                    .shape
                    .intersection(&engine.graph.room(b).shape);
                assert!(
                    overlap.dimension() < 2,
                    "rooms {a} and {b} overlap 2-dimensionally"
                );
            }
        }

        // rooms never overlap the obstacle wall
        let wall_shapes: Vec<TileShape> = board
            .get_item(wall)
            .unwrap()
            .tile_shapes(&board.padstacks)
            .iter()
            .map(|(s, _)| s.clone())
            .collect();
        for &room in &rooms {
            for ws in &wall_shapes {
                assert!(
                    engine.graph.room(room).shape.intersection(ws).dimension() < 2,
                    "a room overlaps the obstacle"
                );
            }
        }
    }

    #[test]
    fn start_room_has_target_door_for_own_pad() {
        let mut board = test_board();
        let pad = board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        let mut engine = AutorouteEngine::new(1);
        let rooms = engine.create_start_rooms(
            &board,
            TileShape::Box(IntBox::new(IntPoint::new(0, 0), IntPoint::new(0, 0))),
            0,
        );
        assert_eq!(rooms.len(), 1);
        let targets = engine.target_doors(rooms[0]);
        assert!(targets.iter().any(|t| t.item == pad));
        // the room contains the start point and covers free space
        assert!(engine
            .graph
            .room(rooms[0])
            .shape
            .contains(&Point::Int(IntPoint::new(0, 0))));
    }
}
