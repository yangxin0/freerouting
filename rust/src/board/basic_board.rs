//! Port of the core of `board/BasicBoard.java` (incremental).
//!
//! The board combines the undoable item database with the spatial search
//! tree: items are inserted with their per-layer tile shapes into a
//! [`MinAreaTree`] keyed by (item id, shape index, layer), and all
//! overlap queries go through that tree. Java's `SearchTreeManager` with
//! multiple compensated trees follows later; this is the single
//! uncompensated default tree.

use std::collections::BTreeMap;

use crate::board::item::{Item, ItemBase, ItemKind};
use crate::board::LayerStructure;
use crate::core::Padstacks;
use crate::datastructures::{LeafId, MinAreaTree, UndoableObjects};
use crate::geometry::planar::{IntBox, IntPoint, Point, Polyline, TileShape};
use crate::rules::BoardRules;

/// Unique id of an item on the board.
pub type ItemId = i32;

/// One search-tree entry of an item: which shape of which item on which
/// layer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct TreeShapeEntry {
    item_id: ItemId,
    shape_index: usize,
    layer: usize,
}

#[derive(Debug)]
pub struct BasicBoard {
    pub layer_structure: LayerStructure,
    pub rules: BoardRules,
    pub padstacks: Padstacks,
    /// The undoable item database.
    item_list: UndoableObjects<ItemId, Item>,
    /// The spatial index over all item shapes.
    search_tree: MinAreaTree<TreeShapeEntry>,
    /// The tree leaves of each item, for removal.
    tree_entries: BTreeMap<ItemId, Vec<LeafId>>,
    /// Generator for unique item ids.
    next_id_no: ItemId,
}

impl BasicBoard {
    pub fn new(layer_structure: LayerStructure, rules: BoardRules, padstacks: Padstacks) -> Self {
        BasicBoard {
            layer_structure,
            rules,
            padstacks,
            item_list: UndoableObjects::new(),
            search_tree: MinAreaTree::new(),
            tree_entries: BTreeMap::new(),
            next_id_no: 0,
        }
    }

    fn new_id_no(&mut self) -> ItemId {
        self.next_id_no += 1;
        self.next_id_no
    }

    /// Inserts an item, assigning it a fresh id. Returns the id.
    pub fn insert_item(&mut self, mut item: Item) -> ItemId {
        let id = self.new_id_no();
        item.base.id_no = id;
        self.insert_into_search_tree(id, &item);
        self.item_list.insert(id, item);
        id
    }

    /// Convenience: inserts a polyline trace.
    pub fn insert_trace(
        &mut self,
        polyline: Polyline,
        layer: usize,
        half_width: i32,
        net_nos: Vec<i32>,
        clearance_class: usize,
    ) -> ItemId {
        let item = Item::new_polyline_trace(
            ItemBase::new(0, net_nos, clearance_class),
            half_width,
            layer,
            polyline,
        );
        self.insert_item(item)
    }

    /// Convenience: inserts a via with the given 1-based padstack number.
    pub fn insert_via(
        &mut self,
        padstack: usize,
        center: IntPoint,
        net_nos: Vec<i32>,
        clearance_class: usize,
        attach_allowed: bool,
    ) -> ItemId {
        let item = Item::new_via(
            ItemBase::new(0, net_nos, clearance_class),
            padstack,
            center,
            attach_allowed,
        );
        self.insert_item(item)
    }

    fn insert_into_search_tree(&mut self, id: ItemId, item: &Item) {
        let mut leaves = Vec::new();
        for (index, (shape, layer)) in item.tile_shapes(&self.padstacks).into_iter().enumerate()
        {
            let Some(bound) = shape.bounding_octagon() else {
                continue;
            };
            leaves.push(self.search_tree.insert(
                TreeShapeEntry {
                    item_id: id,
                    shape_index: index,
                    layer,
                },
                index,
                bound,
            ));
        }
        self.tree_entries.insert(id, leaves);
    }

    fn remove_from_search_tree(&mut self, id: ItemId) {
        if let Some(leaves) = self.tree_entries.remove(&id) {
            self.search_tree.remove(&leaves);
        }
    }

    /// Removes an item from the board. Returns false if no such item is
    /// alive.
    pub fn remove_item(&mut self, id: ItemId) -> bool {
        if self.item_list.get(&id).is_none() {
            return false;
        }
        self.remove_from_search_tree(id);
        self.item_list.delete(&id)
    }

    /// The currently alive item with `id`.
    pub fn get_item(&self, id: ItemId) -> Option<&Item> {
        self.item_list.get(&id)
    }

    /// Iterates over the alive items.
    pub fn items(&self) -> impl Iterator<Item = (&ItemId, &Item)> {
        self.item_list.iter()
    }

    pub fn item_count(&self) -> usize {
        self.item_list.iter().count()
    }

    /// The ids of the items whose tree shapes overlap `shape` on `layer`
    /// (layer `None` = all layers), sorted and deduplicated.
    pub fn overlapping_items(&self, shape: &TileShape, layer: Option<usize>) -> Vec<ItemId> {
        let Some(query) = shape.bounding_octagon() else {
            return Vec::new();
        };
        let mut result: Vec<ItemId> = self
            .search_tree
            .overlaps(query)
            .into_iter()
            .map(|leaf| *self.search_tree.entry(leaf).object)
            .filter(|entry| layer.is_none_or(|l| entry.layer == l))
            .filter(|entry| {
                // exact check against the item's real shape (the tree only
                // stores bounding octagons)
                self.get_item(entry.item_id)
                    .and_then(|item| item.tile_shape(entry.shape_index, &self.padstacks))
                    .is_some_and(|(s, _)| s.intersects(shape))
            })
            .map(|entry| entry.item_id)
            .collect();
        result.sort();
        result.dedup();
        result
    }

    /// The connectable items of `net_no` touching `shape` on `layer`.
    pub fn overlapping_items_of_net(
        &self,
        shape: &TileShape,
        layer: usize,
        net_no: i32,
    ) -> Vec<ItemId> {
        self.overlapping_items(shape, Some(layer))
            .into_iter()
            .filter(|id| {
                self.get_item(*id)
                    .is_some_and(|item| item.base.contains_net(net_no))
            })
            .collect()
    }

    /// True if inserting a shape of `net_no` at `shape` on `layer` would
    /// collide with a foreign-net item (simplified obstacle rule: items of
    /// a different net are obstacles).
    pub fn is_blocked(&self, shape: &TileShape, layer: usize, net_no: i32) -> bool {
        self.overlapping_items(shape, Some(layer))
            .into_iter()
            .any(|id| {
                self.get_item(id)
                    .is_some_and(|item| !item.base.contains_net(net_no))
            })
    }

    // ---- connectivity (Java: Item/Trace/DrillItem contact methods) ----

    /// The contacts of the item `id` at `point` (for traces: only at their
    /// end corners). An item is a contact if it overlaps at the point,
    /// shares a layer and (unless `ignore_net`) a net, and touches
    /// according to its kind: traces by an end corner, drill items by
    /// their center.
    pub fn get_normal_contacts_at(
        &self,
        id: ItemId,
        point: &Point,
        ignore_net: bool,
    ) -> Vec<ItemId> {
        let Some(item) = self.get_item(id) else {
            return Vec::new();
        };
        let search_shape = TileShape::Box(point.surrounding_box());
        let first_layer = item.first_layer(&self.padstacks);
        let last_layer = item.last_layer(&self.padstacks);
        let mut result = Vec::new();
        for other_id in self.overlapping_items(&search_shape, None) {
            if other_id == id {
                continue;
            }
            let Some(other) = self.get_item(other_id) else {
                continue;
            };
            // shares a layer?
            if other.last_layer(&self.padstacks) < first_layer
                || other.first_layer(&self.padstacks) > last_layer
            {
                continue;
            }
            if !ignore_net && !other.base.shares_net(&item.base) {
                continue;
            }
            let touches = match &other.kind {
                ItemKind::PolylineTrace(t) => {
                    *point == t.first_corner() || *point == t.last_corner()
                }
                ItemKind::Via(v) => *point == Point::Int(v.center),
            };
            if touches {
                result.push(other_id);
            }
        }
        result.sort();
        result.dedup();
        result
    }

    /// All contacts of the item `id` (Java: `get_normal_contacts`): for a
    /// trace the contacts at its two end corners, for a via the contacts
    /// at its center.
    pub fn get_normal_contacts(&self, id: ItemId) -> Vec<ItemId> {
        let Some(item) = self.get_item(id) else {
            return Vec::new();
        };
        let mut result = match &item.kind {
            ItemKind::PolylineTrace(t) => {
                let mut r = self.get_normal_contacts_at(id, &t.first_corner(), false);
                r.extend(self.get_normal_contacts_at(id, &t.last_corner(), false));
                r
            }
            ItemKind::Via(v) => {
                self.get_normal_contacts_at(id, &Point::Int(v.center), false)
            }
        };
        result.sort();
        result.dedup();
        result
    }

    /// The contacts of a trace at its start corner.
    pub fn get_start_contacts(&self, id: ItemId) -> Vec<ItemId> {
        match self.get_item(id).map(|i| &i.kind) {
            Some(ItemKind::PolylineTrace(t)) => {
                self.get_normal_contacts_at(id, &t.first_corner(), false)
            }
            _ => Vec::new(),
        }
    }

    /// The contacts of a trace at its end corner.
    pub fn get_end_contacts(&self, id: ItemId) -> Vec<ItemId> {
        match self.get_item(id).map(|i| &i.kind) {
            Some(ItemKind::PolylineTrace(t)) => {
                self.get_normal_contacts_at(id, &t.last_corner(), false)
            }
            _ => Vec::new(),
        }
    }

    /// True if the trace is not contacted at its first or its last corner
    /// (Java: `Trace.is_tail`).
    pub fn is_tail(&self, id: ItemId) -> bool {
        match self.get_item(id).map(|i| &i.kind) {
            Some(ItemKind::PolylineTrace(_)) => {
                self.get_start_contacts(id).is_empty() || self.get_end_contacts(id).is_empty()
            }
            _ => false,
        }
    }

    /// The set of items connected to `id` (including itself) by contacts
    /// of the net `net_no` (Java: `Item.get_connected_set`).
    pub fn get_connected_set(&self, id: ItemId, net_no: i32) -> Vec<ItemId> {
        let Some(item) = self.get_item(id) else {
            return Vec::new();
        };
        if !item.base.contains_net(net_no) {
            return Vec::new();
        }
        let mut visited = vec![id];
        let mut queue = vec![id];
        while let Some(curr) = queue.pop() {
            for contact in self.get_normal_contacts(curr) {
                if visited.contains(&contact) {
                    continue;
                }
                if self
                    .get_item(contact)
                    .is_some_and(|i| i.base.contains_net(net_no))
                {
                    visited.push(contact);
                    queue.push(contact);
                }
            }
        }
        visited.sort();
        visited
    }

    /// True if all connectable items of `net_no` form one connected set.
    pub fn net_is_completely_connected(&self, net_no: i32) -> bool {
        let net_items: Vec<ItemId> = self
            .items()
            .filter(|(_, item)| item.base.contains_net(net_no) && item.is_connectable())
            .map(|(id, _)| *id)
            .collect();
        let Some(&first) = net_items.first() else {
            return true;
        };
        let connected = self.get_connected_set(first, net_no);
        net_items.iter().all(|id| connected.contains(id))
    }

    /// The smallest box containing all items of the board.
    pub fn bounding_box(&self) -> IntBox {
        let mut result = IntBox::EMPTY;
        for (_, item) in self.items() {
            result = result.union(item.bounding_box(&self.padstacks));
        }
        result
    }

    /// Makes the current state restorable by undo.
    pub fn generate_snapshot(&mut self) {
        self.item_list.generate_snapshot();
    }

    /// Restores the situation before the last snapshot, resynchronizing
    /// the search tree. Returns false if no undo is possible.
    pub fn undo(&mut self) -> bool {
        let mut cancelled = Vec::new();
        let mut restored = Vec::new();
        if !self.item_list.undo(&mut cancelled, &mut restored) {
            return false;
        }
        self.resync_search_tree(&cancelled, &restored);
        true
    }

    /// Restores the situation before the last undo. Returns false if no
    /// redo is possible.
    pub fn redo(&mut self) -> bool {
        let mut cancelled = Vec::new();
        let mut restored = Vec::new();
        if !self.item_list.redo(&mut cancelled, &mut restored) {
            return false;
        }
        self.resync_search_tree(&cancelled, &restored);
        true
    }

    fn resync_search_tree(&mut self, cancelled: &[Item], restored: &[Item]) {
        // Remove the tree entries of all touched items, then reinsert the
        // ones which are alive again.
        for item in cancelled.iter().chain(restored) {
            self.remove_from_search_tree(item.base.id_no);
        }
        let to_reinsert: Vec<ItemId> = cancelled
            .iter()
            .chain(restored)
            .map(|item| item.base.id_no)
            .collect();
        for id in to_reinsert {
            if let Some(item) = self.item_list.get(&id) {
                if !self.tree_entries.contains_key(&id) {
                    let item = item.clone();
                    self.insert_into_search_tree(id, &item);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::Layer;
    use crate::rules::ClearanceMatrix;

    fn test_board() -> BasicBoard {
        let stack = LayerStructure::new(vec![
            Layer::new("F.Cu", true),
            Layer::new("B.Cu", true),
        ]);
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

    fn trace_polyline(points: &[(i32, i32)]) -> Polyline {
        let pts: Vec<IntPoint> = points.iter().map(|(x, y)| IntPoint::new(*x, *y)).collect();
        Polyline::from_int_points(&pts)
    }

    fn query_box(llx: i32, lly: i32, urx: i32, ury: i32) -> TileShape {
        TileShape::Box(IntBox::from_coords(llx, lly, urx, ury))
    }

    #[test]
    fn insert_query_remove_items() {
        let mut board = test_board();
        let trace = board.insert_trace(
            trace_polyline(&[(0, 0), (5000, 0)]),
            0,
            100,
            vec![1],
            1,
        );
        let via = board.insert_via(1, IntPoint::new(5000, 0), vec![1], 1, false);
        assert_eq!(board.item_count(), 2);

        // query around the middle of the trace on layer 0
        let hits = board.overlapping_items(&query_box(2000, -50, 3000, 50), Some(0));
        assert_eq!(hits, vec![trace]);
        // the via is on both layers
        let hits = board.overlapping_items(&query_box(4800, -50, 5200, 50), Some(1));
        assert_eq!(hits, vec![via]);
        let hits = board.overlapping_items(&query_box(4800, -50, 5200, 50), None);
        assert_eq!(hits, vec![trace, via]);
        // nothing far away
        assert!(board
            .overlapping_items(&query_box(20000, 20000, 21000, 21000), None)
            .is_empty());

        // net filtering and blocking
        assert_eq!(
            board.overlapping_items_of_net(&query_box(2000, -50, 3000, 50), 0, 1),
            vec![trace]
        );
        assert!(!board.is_blocked(&query_box(2000, -50, 3000, 50), 0, 1));
        assert!(board.is_blocked(&query_box(2000, -50, 3000, 50), 0, 2));

        // bounding box covers trace and via pads
        assert_eq!(
            board.bounding_box(),
            IntBox::from_coords(-100, -400, 5400, 400)
        );

        assert!(board.remove_item(trace));
        assert!(!board.remove_item(trace));
        assert_eq!(board.item_count(), 1);
        assert!(board
            .overlapping_items(&query_box(2000, -50, 3000, 50), Some(0))
            .is_empty());
    }

    #[test]
    fn exact_shape_check_filters_bounding_octagon_hits() {
        let mut board = test_board();
        // a diagonal trace: its bounding octagon covers the corner area,
        // but the exact shape does not
        board.insert_trace(
            trace_polyline(&[(0, 0), (4000, 4000)]),
            0,
            100,
            vec![1],
            1,
        );
        // a box near the diagonal but not touching the trace shape
        let far_corner = query_box(3000, 0, 3400, 400);
        assert!(board.overlapping_items(&far_corner, Some(0)).is_empty());
        // a box crossing the diagonal
        let on_diag = query_box(1900, 1900, 2100, 2100);
        assert_eq!(board.overlapping_items(&on_diag, Some(0)).len(), 1);
    }

    #[test]
    fn connectivity_contacts_and_connected_sets() {
        let mut board = test_board();
        // net 1: pin-like via at (0,0), trace to (5000,0), via there,
        // trace on layer 1 onwards
        let via_a = board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        let trace_1 = board.insert_trace(
            trace_polyline(&[(0, 0), (5000, 0)]),
            0,
            100,
            vec![1],
            1,
        );
        let via_b = board.insert_via(1, IntPoint::new(5000, 0), vec![1], 1, false);
        let trace_2 = board.insert_trace(
            trace_polyline(&[(5000, 0), (5000, 4000)]),
            1,
            100,
            vec![1],
            1,
        );
        // an unrelated trace of another net crossing nearby
        let foreign = board.insert_trace(
            trace_polyline(&[(2000, -3000), (2000, 3000)]),
            1,
            100,
            vec![2],
            1,
        );

        // trace_1 contacts both vias
        assert_eq!(board.get_normal_contacts(trace_1), vec![via_a, via_b]);
        assert_eq!(board.get_start_contacts(trace_1), vec![via_a]);
        assert_eq!(board.get_end_contacts(trace_1), vec![via_b]);
        assert!(!board.is_tail(trace_1));
        // trace_2 ends in the air
        assert_eq!(board.get_normal_contacts(trace_2), vec![via_b]);
        assert!(board.is_tail(trace_2));
        // via_b joins layers: contacts on both layers
        assert_eq!(board.get_normal_contacts(via_b), vec![trace_1, trace_2]);
        // foreign net crossing shares no contact
        assert!(board.get_normal_contacts(foreign).is_empty());

        // the whole net 1 is one connected set
        let connected = board.get_connected_set(via_a, 1);
        assert_eq!(connected, vec![via_a, trace_1, via_b, trace_2]);
        assert!(board.net_is_completely_connected(1));

        // an isolated stub of net 1 breaks completeness
        let stub = board.insert_trace(
            trace_polyline(&[(9000, 9000), (9500, 9000)]),
            0,
            100,
            vec![1],
            1,
        );
        assert!(!board.net_is_completely_connected(1));
        board.remove_item(stub);
        assert!(board.net_is_completely_connected(1));
        // removing the middle trace splits the net
        board.remove_item(trace_1);
        assert!(!board.net_is_completely_connected(1));
        assert_eq!(board.get_connected_set(via_b, 1), vec![via_b, trace_2]);
    }

    #[test]
    fn undo_redo_resyncs_search_tree() {
        let mut board = test_board();
        let trace = board.insert_trace(
            trace_polyline(&[(0, 0), (5000, 0)]),
            0,
            100,
            vec![1],
            1,
        );
        board.generate_snapshot();
        let via = board.insert_via(1, IntPoint::new(2500, 0), vec![1], 1, false);
        board.remove_item(trace);
        assert_eq!(board.item_count(), 1);
        let query = query_box(2400, -50, 2600, 50);
        assert_eq!(board.overlapping_items(&query, Some(0)), vec![via]);

        // undo: the trace comes back, the via disappears
        assert!(board.undo());
        assert_eq!(board.item_count(), 1);
        assert_eq!(board.overlapping_items(&query, Some(0)), vec![trace]);

        // redo: the via returns, the trace is deleted again
        assert!(board.redo());
        assert_eq!(board.item_count(), 1);
        assert_eq!(board.overlapping_items(&query, Some(0)), vec![via]);
        assert!(!board.redo());
    }
}
