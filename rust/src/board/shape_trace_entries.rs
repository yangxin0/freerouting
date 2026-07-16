//! Port of `board/ShapeTraceEntries.java`: `cutout_trace` (removes the
//! part of a trace inside a shape and reinserts the outside pieces) and
//! the shove bookkeeping ([`ShapeTraceEntries`]) that collects the entry
//! points of the traces crossing a shove shape, sorts them around the
//! border from the from-side, checks the stack property and produces the
//! substitute trace pieces in stack order. Java's linked EntryPoint list
//! becomes a sorted Vec; item references become ItemIds.

use crate::board::basic_board::{BasicBoard, ItemId};
use crate::board::ItemKind;
use crate::geometry::planar::{Polyline, TileShape};

/// Java: `ShapeTraceEntries.c_offset_add`.
const C_OFFSET_ADD: f64 = 1.0;

/// Cuts out the part of the trace `trace_id` inside `shape` (enlarged by
/// the trace half width plus the clearance to `cl_class`) and reinserts
/// the remaining outside pieces. Returns the ids of the inserted pieces
/// (empty when nothing was cut).
pub fn cutout_trace(
    board: &mut BasicBoard,
    trace_id: ItemId,
    shape: &TileShape,
    cl_class: usize,
) -> Vec<ItemId> {
    let Some(item) = board.get_item(trace_id) else {
        // Java warns "trace is deleted"
        return Vec::new();
    };
    let ItemKind::PolylineTrace(trace) = &item.kind else {
        return Vec::new();
    };
    // enlarge the shape in 2 steps for symmetry reasons (Java comment)
    let cl_offset = board.rules.clearance_matrix.get_value(
        item.base.clearance_class,
        cl_class,
        trace.layer,
        false,
    ) as f64
        + C_OFFSET_ADD;
    let offset_shape = shape.offset(trace.half_width as f64).offset(cl_offset);
    let pieces = offset_shape.cutout_polyline(&trace.polyline);
    if pieces.len() == 1 && pieces[0] == trace.polyline {
        // nothing cut off
        return Vec::new();
    }
    let layer = trace.layer;
    let half_width = trace.half_width;
    let net_nos = item.base.net_nos.clone();
    let clearance_class = item.base.clearance_class;
    board.remove_item(trace_id);
    let mut inserted = Vec::new();
    for piece in pieces {
        if piece.is_empty() {
            continue;
        }
        inserted.push(board.insert_trace(
            piece,
            layer,
            half_width,
            net_nos.clone(),
            clearance_class,
        ));
    }
    inserted
}

/// An entry point of a trace into the shove shape; the entries are kept
/// sorted around the border of the shape (Java: inner class EntryPoint).
#[derive(Debug, Clone)]
struct EntryPoint {
    trace: ItemId,
    net_nos: Vec<i32>,
    half_width: i32,
    clearance_class: usize,
    #[allow(dead_code)]
    trace_line_no: usize,
    /// The trace's polyline line at `trace_line_no`, cached because the
    /// substitute is built after the victim was cut off the board.
    trace_line: crate::geometry::planar::Line,
    entry_approx: crate::geometry::planar::FloatPoint,
    edge_no: usize,
    /// -1 = not yet calculated.
    stack_level: i32,
}

/// The shove bookkeeping of `ShapeTraceEntries.java`: collects the entry
/// points of the traces crossing a shove shape, sorts them around the
/// border starting at the from side, and produces the substitute trace
/// pieces in stack order.
pub struct ShapeTraceEntries {
    shape: TileShape,
    layer: usize,
    own_net_nos: Vec<i32>,
    cl_class: usize,
    from_side: crate::board::CalcFromSide,
    entries: Vec<EntryPoint>,
    trace_piece_count: i32,
    max_stack_level: i32,
    shape_contains_trace_tails: bool,
    /// The item responsible when storing failed.
    pub found_obstacle: Option<ItemId>,
    /// The vias inside the shape that must be shoved.
    pub shove_via_list: Vec<ItemId>,
}

impl ShapeTraceEntries {
    /// `from_side.no == None` means it is calculated internally.
    pub fn new(
        shape: TileShape,
        layer: usize,
        own_net_nos: Vec<i32>,
        cl_class: usize,
        from_side: crate::board::CalcFromSide,
    ) -> ShapeTraceEntries {
        ShapeTraceEntries {
            shape,
            layer,
            own_net_nos,
            cl_class,
            from_side,
            entries: Vec::new(),
            trace_piece_count: 0,
            max_stack_level: 0,
            shape_contains_trace_tails: false,
            found_obstacle: None,
            shove_via_list: Vec::new(),
        }
    }

    fn net_nos_equal(a: &[i32], b: &[i32]) -> bool {
        a.len() == b.len() && a.iter().all(|n| b.contains(n))
    }

    /// Stores the traces and vias in `item_ids`. Returns false if the list
    /// contains obstacles that cannot be shoved aside. With `is_pad_check`
    /// the check is for vias, otherwise for traces; `copper_sharing_allowed`
    /// permits overlaps with items of the own net.
    pub fn store_items(
        &mut self,
        board: &BasicBoard,
        item_ids: &[ItemId],
        is_pad_check: bool,
        copper_sharing_allowed: bool,
    ) -> bool {
        for &item_id in item_ids {
            let Some(item) = board.get_item(item_id) else {
                continue;
            };
            let contains_own_net = self.own_net_nos.iter().any(|n| item.base.contains_net(*n));
            match &item.kind {
                ItemKind::ObstacleArea(a) => {
                    if a.via_only {
                        continue;
                    }
                    if a.is_conduction && contains_own_net {
                        continue;
                    }
                    if a.is_conduction && !a.is_obstacle {
                        // planes are not obstacles for foreign items —
                        // unless flagged (Java ConductionArea.is_obstacle)
                        continue;
                    }
                    self.found_obstacle = Some(item_id);
                    return false;
                }
                ItemKind::Via(v) => {
                    if item.base.is_shove_fixed() && !contains_own_net {
                        self.found_obstacle = Some(item_id);
                        return false;
                    }
                    if item.base.component_no == 0 {
                        // a free via
                        if is_pad_check || !contains_own_net {
                            self.shove_via_list.push(item_id);
                        }
                    } else {
                        // a component pin. For a pad (via) check, a same-net
                        // pin blocks unless it is a drillable SMD pin (Java
                        // ShapeTraceEntries: `pin.drill_allowed()`, NOT the
                        // padstack attach flag).
                        let _ = v;
                        if !contains_own_net
                            || !copper_sharing_allowed
                            || (is_pad_check
                                && item.first_layer(&board.padstacks)
                                    != item.last_layer(&board.padstacks))
                        {
                            self.found_obstacle = Some(item_id);
                            return false;
                        }
                    }
                }
                ItemKind::PolylineTrace(_) => {
                    if item.base.is_shove_fixed() && !contains_own_net {
                        self.found_obstacle = Some(item_id);
                        return false;
                    }
                    if !self.store_trace(board, item_id) {
                        return false;
                    }
                }
            }
        }
        self.search_from_side();
        self.resort();
        self.calculate_stack_levels()
    }

    /// The next substitute trace piece: its polyline plus (layer,
    /// half width, net numbers, clearance class); `None` at the end.
    #[allow(clippy::type_complexity)]
    pub fn next_substitute_trace_piece(
        &mut self,
        board: &BasicBoard,
    ) -> Option<(Polyline, usize, i32, Vec<i32>, usize)> {
        loop {
            let (first, last) = self.pop_piece()?;
            let cl_offset = board.rules.clearance_matrix.get_value(
                first.clearance_class,
                self.cl_class,
                self.layer,
                false,
            ) as f64
                + C_OFFSET_ADD;
            let offset_shape = self.shape.offset(first.half_width as f64).offset(cl_offset);
            let edge_count = self.shape.border_line_count();
            let edge_diff = last.edge_no - first.edge_no;
            // the substitute trace: the intersecting trace lines at both
            // entries (cached; the victims are already cut) joined by the
            // edge lines of the offset shape
            let mut piece_lines = Vec::with_capacity(edge_diff + 3);
            piece_lines.push(first.trace_line);
            let mut curr_edge_no = first.edge_no % edge_count;
            for _ in 0..=edge_diff {
                piece_lines.push(offset_shape.border_line(curr_edge_no));
                curr_edge_no = (curr_edge_no + 1) % edge_count;
            }
            piece_lines.push(last.trace_line);
            let piece_polyline = Polyline::from_lines(piece_lines);
            if piece_polyline.is_empty() {
                continue; // no valid piece: try the next one
            }
            return Some((
                piece_polyline,
                self.layer,
                first.half_width,
                first.net_nos.clone(),
                first.clearance_class,
            ));
        }
    }

    /// The maximum recursion depth for shoving the obstacle traces.
    pub fn stack_depth(&self) -> i32 {
        self.max_stack_level
    }

    /// The number of substitute trace pieces.
    pub fn substitute_trace_count(&self) -> i32 {
        self.trace_piece_count
    }

    /// True if an unconnected trace end of a foreign net lies inside the
    /// shape.
    pub fn trace_tails_in_shape(&self) -> bool {
        self.shape_contains_trace_tails
    }

    /// Cuts all foreign traces in `item_ids` out of the stored shape.
    pub fn cutout_traces(&self, board: &mut BasicBoard, item_ids: &[ItemId]) {
        for &item_id in item_ids {
            let Some(item) = board.get_item(item_id) else {
                continue;
            };
            let is_foreign_trace = matches!(item.kind, ItemKind::PolylineTrace(_))
                && !self.own_net_nos.iter().any(|n| item.base.contains_net(*n));
            if is_foreign_trace {
                cutout_trace(board, item_id, &self.shape, self.cl_class);
            }
        }
    }

    /// Stores the intersections of the trace with the border of the shape
    /// enlarged by its half width and clearance.
    fn store_trace(&mut self, board: &BasicBoard, trace_id: ItemId) -> bool {
        let Some(item) = board.get_item(trace_id) else {
            return true;
        };
        let ItemKind::PolylineTrace(trace) = &item.kind else {
            return true;
        };
        let cl_offset = board.rules.clearance_matrix.get_value(
            item.base.clearance_class,
            self.cl_class,
            trace.layer,
            false,
        ) as f64
            + C_OFFSET_ADD;
        // offset (not enlarge) because of the comparison in EntryPoint
        let offset_shape = self.shape.offset(trace.half_width as f64).offset(cl_offset);
        for (line_no, edge_no) in offset_shape.entrance_points(&trace.polyline) {
            let entry_approx =
                trace.polyline.arr[line_no].intersection_approx(&offset_shape.border_line(edge_no));
            self.insert_entry_point(
                trace_id,
                item.base.net_nos.clone(),
                trace.half_width,
                item.base.clearance_class,
                line_no,
                trace.polyline.arr[line_no],
                edge_no,
                entry_approx,
            );
        }
        // a trace end inside the shape (e.g. when a via touches it)
        let contains_own_net = self.own_net_nos.iter().any(|n| item.base.contains_net(*n));
        if !contains_own_net {
            for i in 0..2 {
                let end_corner = if i == 0 {
                    trace.first_corner()
                } else {
                    trace.last_corner()
                };
                if !offset_shape.contains(&end_corner) {
                    continue;
                }
                let contact_list = board.get_normal_contacts_at(trace_id, &end_corner, false);
                let mut store_end_corner = true;
                for &contact_id in &contact_list {
                    let Some(contact) = board.get_item(contact_id) else {
                        continue;
                    };
                    if !contact.is_routable() {
                        self.found_obstacle = Some(contact_id);
                        return false;
                    }
                    match &contact.kind {
                        ItemKind::PolylineTrace(ct) => {
                            if (contact.base.is_shove_fixed()
                                || ct.half_width != trace.half_width
                                || contact.base.clearance_class != item.base.clearance_class)
                                && offset_shape.contains_inside(&end_corner)
                            {
                                self.found_obstacle = Some(contact_id);
                                return false;
                            }
                        }
                        ItemKind::Via(_) => {
                            let via_radius = contact
                                .tile_shapes(&board.padstacks)
                                .iter()
                                .find(|(_, l)| *l == self.layer)
                                .map(|(s, _)| s.min_width() / 2.0)
                                .unwrap_or(0.0);
                            let mut via_trace_diff = via_radius - trace.half_width as f64;
                            let via_clearance = board.rules.clearance_matrix.get_value(
                                contact.base.clearance_class,
                                self.cl_class,
                                self.layer,
                                false,
                            );
                            let trace_clearance = board.rules.clearance_matrix.get_value(
                                item.base.clearance_class,
                                self.cl_class,
                                self.layer,
                                false,
                            );
                            if trace_clearance > via_clearance {
                                via_trace_diff += (via_clearance - trace_clearance) as f64;
                            }
                            if via_trace_diff < 0.0 {
                                // the via is smaller than the trace
                                self.found_obstacle = Some(contact_id);
                                return false;
                            }
                            if via_trace_diff == 0.0 && !offset_shape.contains_inside(&end_corner) {
                                store_end_corner = false;
                            }
                        }
                        ItemKind::ObstacleArea(_) => {}
                    }
                }
                if contact_list.len() == 1 && store_end_corner {
                    if let Some(projection) = offset_shape.nearest_border_point(&end_corner) {
                        if let Some(projection_side) =
                            offset_shape.contains_on_border_line_no(&projection)
                        {
                            let trace_line_segment_no = if i == 0 {
                                0
                            } else {
                                trace.polyline.arr.len() - 1
                            };
                            self.insert_entry_point(
                                trace_id,
                                item.base.net_nos.clone(),
                                trace.half_width,
                                item.base.clearance_class,
                                trace_line_segment_no,
                                trace.polyline.arr[trace_line_segment_no],
                                projection_side,
                                projection.to_float(),
                            );
                        }
                    }
                } else if contact_list.is_empty() && offset_shape.contains_inside(&end_corner) {
                    self.shape_contains_trace_tails = true;
                }
            }
        }
        self.found_obstacle = Some(trace_id);
        true
    }

    fn search_from_side(&mut self) {
        if self.from_side.no.is_some() {
            return;
        }
        let mut curr_fromside_no = 0;
        let mut curr_entry_approx = None;
        for entry in &self.entries {
            if Self::net_nos_equal(&entry.net_nos, &self.own_net_nos) {
                curr_fromside_no = entry.edge_no;
                curr_entry_approx = Some(entry.entry_approx);
                break;
            }
        }
        self.from_side = crate::board::CalcFromSide {
            no: Some(curr_fromside_no),
            border_intersection: curr_entry_approx,
        };
    }

    /// Resorts the entry points to start in the middle of the from side and
    /// removes redundant points.
    fn resort(&mut self) {
        let edge_count = self.shape.border_line_count();
        let Some(from_side_no) = self.from_side.no else {
            return; // Java warns "from side not calculated"
        };
        if from_side_no >= edge_count {
            return;
        }
        let compare_corner_1 = self.shape.corner_approx(from_side_no);
        let compare_corner_2 = self.shape.corner_approx((from_side_no + 1) % edge_count);
        let border_line = self.shape.border_line(from_side_no);
        let border_fline = crate::geometry::planar::FloatLine::new(
            border_line.a.to_float(),
            border_line.b.to_float(),
        );
        let mut from_point_dist = 0.0;
        let mut from_point_projection = None;
        if let Some(bi) = self.from_side.border_intersection {
            let projection = border_fline.perpendicular_projection(bi);
            from_point_dist = projection.distance_square(compare_corner_1);
            from_point_projection = Some(projection);
            if from_point_dist >= compare_corner_1.distance_square(compare_corner_2) {
                self.from_side.border_intersection = None;
                from_point_projection = None;
            }
        }
        // the first entry after the middle of the from side
        let mut split = self.entries.len();
        for (idx, entry) in self.entries.iter().enumerate() {
            if entry.edge_no > from_side_no {
                split = idx;
                break;
            }
            if entry.edge_no == from_side_no {
                let hit = if let Some(fpp) = from_point_projection {
                    let curr_projection = border_fline.perpendicular_projection(entry.entry_approx);
                    curr_projection.distance_square(compare_corner_1) >= from_point_dist
                        && curr_projection.distance_square(fpp)
                            <= curr_projection.distance_square(compare_corner_1)
                } else {
                    entry.entry_approx.distance_square(compare_corner_2)
                        <= entry.entry_approx.distance_square(compare_corner_1)
                };
                if hit {
                    split = idx;
                    break;
                }
            }
        }
        if split > 0 && split < self.entries.len() {
            for entry in &mut self.entries[..split] {
                entry.edge_no += edge_count;
            }
            self.entries.rotate_left(split);
        } else if split == self.entries.len() {
            // all entries are before the middle: same as Java keeping the
            // anchor unchanged
        }
        // remove interior intersections of the same connected set
        let mut i = 0;
        while i + 2 < self.entries.len() {
            let equal_1 =
                Self::net_nos_equal(&self.entries[i].net_nos, &self.entries[i + 1].net_nos);
            let equal_2 =
                Self::net_nos_equal(&self.entries[i + 1].net_nos, &self.entries[i + 2].net_nos);
            if equal_1 && equal_2 {
                self.entries.remove(i + 1);
            } else {
                i += 1;
            }
        }
        // remove own-net nodes at the end and the start of the list
        while self
            .entries
            .last()
            .is_some_and(|e| Self::net_nos_equal(&e.net_nos, &self.own_net_nos))
        {
            self.entries.pop();
        }
        let mut removed = 0;
        while removed < 2
            && self
                .entries
                .first()
                .is_some_and(|e| Self::net_nos_equal(&e.net_nos, &self.own_net_nos))
        {
            self.entries.remove(0);
            removed += 1;
        }
    }

    fn calculate_stack_levels(&mut self) -> bool {
        if self.entries.is_empty() {
            return true;
        }
        let mut curr_idx = 0usize;
        let mut curr_net_nos = self.entries[0].net_nos.clone();
        let mut curr_level = if Self::net_nos_equal(&curr_net_nos, &self.own_net_nos) {
            0
        } else {
            1
        };
        loop {
            if self.entries[curr_idx].stack_level < 0 {
                self.trace_piece_count += 1;
                self.entries[curr_idx].stack_level = curr_level;
                if curr_level > self.max_stack_level {
                    if self.max_stack_level > 1 {
                        self.found_obstacle = Some(self.entries[curr_idx].trace);
                    }
                    self.max_stack_level = curr_level;
                }
            }
            // propagate the level to all entries of the current net and
            // find the next entry to process
            let mut index_of_next_foreign_set = 0usize;
            let mut index_of_last_occurrence_of_set = 0usize;
            let mut last_own_entry = None;
            let mut first_foreign_entry = None;
            let mut next_index = 0usize;
            for check_idx in curr_idx + 1..self.entries.len() {
                next_index += 1;
                if Self::net_nos_equal(&self.entries[check_idx].net_nos, &curr_net_nos) {
                    index_of_last_occurrence_of_set = next_index;
                    last_own_entry = Some(check_idx);
                    self.entries[check_idx].stack_level = self.entries[curr_idx].stack_level;
                } else if index_of_next_foreign_set == 0 {
                    index_of_next_foreign_set = next_index;
                    first_foreign_entry = Some(check_idx);
                }
            }
            if next_index == 0 {
                break;
            }
            let next_idx;
            if index_of_next_foreign_set != 0
                && index_of_next_foreign_set < index_of_last_occurrence_of_set
            {
                // raise level
                next_idx = first_foreign_entry.unwrap();
                if self.entries[next_idx].stack_level >= 0 {
                    return false; // stack property fails
                }
                curr_level += 1;
            } else if index_of_last_occurrence_of_set != 0 {
                next_idx = last_own_entry.unwrap();
            } else {
                next_idx = first_foreign_entry.unwrap();
                if self.entries[next_idx].stack_level >= 0 {
                    curr_level -= 1;
                    if self.entries[next_idx].stack_level != curr_level {
                        return false;
                    }
                }
            }
            curr_net_nos = self.entries[next_idx].net_nos.clone();
            // remove the irrelevant entries between curr and next
            self.entries.drain(curr_idx + 1..next_idx);
            curr_idx += 1;
        }
        if curr_level != 1 {
            return false; // Java warns "curr_level inconsistent"
        }
        true
    }

    /// Pops the next piece with maximal stack level: the first and last
    /// entry point of that level.
    fn pop_piece(&mut self) -> Option<(EntryPoint, EntryPoint)> {
        if self.entries.is_empty() {
            return None;
        }
        let first_idx = self
            .entries
            .iter()
            .position(|e| e.stack_level == self.max_stack_level)?;
        let mut last_idx = first_idx;
        while last_idx + 1 < self.entries.len()
            && self.entries[last_idx + 1].stack_level == self.max_stack_level
            && Self::net_nos_equal(
                &self.entries[last_idx + 1].net_nos,
                &self.entries[first_idx].net_nos,
            )
        {
            last_idx += 1;
        }
        let first = self.entries[first_idx].clone();
        let last = self.entries[last_idx].clone();
        self.entries.drain(first_idx..=last_idx);
        self.max_stack_level = self
            .entries
            .iter()
            .map(|e| e.stack_level)
            .max()
            .unwrap_or(0);
        self.trace_piece_count -= 1;
        if Self::net_nos_equal(&first.net_nos, &self.own_net_nos) {
            // the own net occurs only at the lowest level and is skipped
            return self.pop_piece();
        }
        Some((first, last))
    }

    #[allow(clippy::too_many_arguments)]
    fn insert_entry_point(
        &mut self,
        trace: ItemId,
        net_nos: Vec<i32>,
        half_width: i32,
        clearance_class: usize,
        trace_line_no: usize,
        trace_line: crate::geometry::planar::Line,
        edge_no: usize,
        entry_approx: crate::geometry::planar::FloatPoint,
    ) {
        let new_entry = EntryPoint {
            trace,
            net_nos,
            half_width,
            clearance_class,
            trace_line_no,
            trace_line,
            entry_approx,
            edge_no,
            stack_level: -1,
        };
        let edge_count = self.shape.border_line_count();
        let mut insert_at = self.entries.len();
        for (idx, curr) in self.entries.iter().enumerate() {
            if curr.edge_no > new_entry.edge_no {
                insert_at = idx;
                break;
            }
            if curr.edge_no == new_entry.edge_no {
                let prev_corner = self.shape.corner_approx(edge_no);
                let next_corner = self.shape.corner_approx((edge_no + 1) % edge_count);
                if prev_corner.scalar_product(entry_approx, next_corner)
                    <= prev_corner.scalar_product(curr.entry_approx, next_corner)
                {
                    insert_at = idx;
                    break;
                }
            }
        }
        self.entries.insert(insert_at, new_entry);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{Layer, LayerStructure};
    use crate::core::Padstacks;
    use crate::geometry::planar::{IntBox, IntPoint, Polyline};
    use crate::rules::{BoardRules, ClearanceMatrix};

    fn test_board() -> BasicBoard {
        let stack = LayerStructure::new(vec![Layer::new("F.Cu", true)]);
        let matrix = ClearanceMatrix::get_default_instance(stack.clone(), 200);
        let mut rules = BoardRules::new(stack.clone(), matrix);
        rules.get_default_net_class();
        BasicBoard::new(stack, rules, Padstacks::new(1))
    }

    #[test]
    fn cuts_crossing_trace_into_two_pieces() {
        let mut board = test_board();
        let polyline =
            Polyline::from_int_points(&[IntPoint::new(-10000, 0), IntPoint::new(10000, 0)]);
        let trace = board.insert_trace(polyline, 0, 100, vec![1], 1);
        let shape = TileShape::Box(IntBox::from_coords(-1000, -1000, 1000, 1000));
        let pieces = cutout_trace(&mut board, trace, &shape, 1);
        assert_eq!(pieces.len(), 2);
        assert!(board.get_item(trace).is_none(), "the original is removed");
        // the pieces end outside the enlarged shape: half width 100 +
        // clearance 200 + 1 = x beyond +-1301
        for id in pieces {
            let item = board.get_item(id).unwrap();
            let ItemKind::PolylineTrace(t) = &item.kind else {
                panic!("piece is a trace");
            };
            for c in t.polyline.corner_approx_arr() {
                assert!(
                    c.x.abs() >= 1300.0,
                    "piece corner {c:?} inside the cut region"
                );
            }
        }
    }

    #[test]
    fn substitute_piece_avoids_the_shove_shape() {
        let mut board = test_board();
        // a foreign trace (net 2) crossing the shove shape of net 1
        let polyline =
            Polyline::from_int_points(&[IntPoint::new(-10000, 0), IntPoint::new(10000, 0)]);
        let trace = board.insert_trace(polyline, 0, 100, vec![2], 1);
        let shape = TileShape::Box(IntBox::from_coords(-1000, -1000, 1000, 1000));
        let mut entries = ShapeTraceEntries::new(
            shape.clone(),
            0,
            vec![1],
            1,
            crate::board::CalcFromSide::NOT_CALCULATED,
        );
        assert!(entries.store_items(&board, &[trace], false, false));
        assert_eq!(entries.substitute_trace_count(), 1);
        assert_eq!(entries.stack_depth(), 1);
        let (piece, layer, half_width, net_nos, _) = entries
            .next_substitute_trace_piece(&board)
            .expect("a substitute piece");
        assert_eq!((layer, half_width, net_nos), (0, 100, vec![2]));
        // the substitute goes around the shove shape: no corner inside
        for c in piece.corner_approx_arr() {
            assert!(
                !shape.contains_inside(&crate::geometry::planar::Point::Int(IntPoint::new(
                    c.x.round() as i32,
                    c.y.round() as i32
                ))),
                "substitute corner {c:?} inside the shove shape"
            );
        }
        // and no more pieces
        assert!(entries.next_substitute_trace_piece(&board).is_none());
    }

    #[test]
    fn disjoint_trace_is_untouched() {
        let mut board = test_board();
        let polyline =
            Polyline::from_int_points(&[IntPoint::new(5000, 5000), IntPoint::new(9000, 5000)]);
        let trace = board.insert_trace(polyline, 0, 100, vec![1], 1);
        let shape = TileShape::Box(IntBox::from_coords(-1000, -1000, 1000, 1000));
        let pieces = cutout_trace(&mut board, trace, &shape, 1);
        assert!(pieces.is_empty());
        assert!(board.get_item(trace).is_some(), "the original stays");
    }
}
