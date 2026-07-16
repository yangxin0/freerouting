//! Port of the core of `board/Item.java`, `board/FixedState.java`, and the
//! first concrete item kinds: `board/Via.java` (with its `DrillItem.java`
//! base) and `board/PolylineTrace.java` (with its `Trace.java` base).
//!
//! Java's class hierarchy becomes an [`ItemBase`] shared struct plus an
//! [`ItemKind`] enum. Board-dependent logic (connectivity, obstacle
//! checks, shoving) follows with the board itself; this module covers the
//! item data and the search-tree shape computation.

use crate::core::Padstacks;
use crate::geometry::planar::{IntBox, IntPoint, Point, Polyline, PolylineArea, TileShape};

/// Sorted fixed states of board items; the strongest states come last.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum FixedState {
    /// The item is allowed to be moved or deleted.
    Unfixed,
    /// The item may not be moved aside by the shove algorithms.
    ShoveFixed,
    /// The item is fixed by the user.
    UserFixed,
    /// The item is fixed by the system.
    SystemFixed,
}

/// The data shared by all items on the board (Java: fields of `Item`).
#[derive(Debug, Clone, PartialEq)]
pub struct ItemBase {
    /// Unique id, used for deterministic ordering.
    pub id_no: i32,
    /// The nets this item belongs to (empty for pure obstacles).
    pub net_nos: Vec<i32>,
    /// Index in the clearance matrix.
    pub clearance_class: usize,
    /// The component this item belongs to (0 = none).
    pub component_no: i32,
    pub fixed_state: FixedState,
    /// Which mechanism inserted the item (diagnostics only):
    /// 0 import/unknown, 1 maze, 2 shove substitute, 3 pull-tight,
    /// 4 combine, 5 repair.
    pub birth: u8,
}

impl ItemBase {
    pub fn new(id_no: i32, net_nos: Vec<i32>, clearance_class: usize) -> Self {
        ItemBase {
            id_no,
            net_nos,
            clearance_class,
            component_no: 0,
            fixed_state: FixedState::Unfixed,
            birth: 0,
        }
    }

    /// True if this item belongs to the net `net_no`.
    pub fn contains_net(&self, net_no: i32) -> bool {
        self.net_nos.contains(&net_no)
    }

    /// True if this item and `other` share a net.
    pub fn shares_net(&self, other: &ItemBase) -> bool {
        self.net_nos.iter().any(|n| other.net_nos.contains(n))
    }

    pub fn net_count(&self) -> usize {
        self.net_nos.len()
    }

    pub fn is_user_fixed(&self) -> bool {
        self.fixed_state >= FixedState::UserFixed
    }

    pub fn is_shove_fixed(&self) -> bool {
        self.fixed_state >= FixedState::ShoveFixed
    }

    /// Unfixes the item, if it is not system fixed.
    pub fn unfix(&mut self) {
        if self.fixed_state != FixedState::SystemFixed {
            self.fixed_state = FixedState::Unfixed;
        }
    }
}

/// A via: a drill item with a padstack (Java: `Via` extending `DrillItem`).
#[derive(Debug, Clone, PartialEq)]
pub struct ViaItem {
    /// 1-based padstack number in the board's padstack library.
    pub padstack: usize,
    pub center: IntPoint,
    /// True if vias of the own net may overlap this via.
    pub attach_allowed: bool,
}

/// A trace on a single layer described by a polyline
/// (Java: `PolylineTrace` extending `Trace`).
#[derive(Debug, Clone, PartialEq)]
pub struct PolylineTraceItem {
    /// Half width of the trace pen.
    pub half_width: i32,
    /// The board layer of the trace.
    pub layer: usize,
    pub polyline: Polyline,
}

/// An area item on a single layer: a keepout (obstacle) or, with
/// `is_conduction`, a conduction area like a power plane
/// (Java: `ObstacleArea` and its subclass `ConductionArea`).
///
/// The port stores the resolved (absolute) area; Java keeps a relative
/// area plus translation/rotation/side, which follows with the component
/// model.
#[derive(Debug, Clone, PartialEq)]
pub struct ObstacleAreaItem {
    pub area: PolylineArea,
    pub layer: usize,
    pub name: String,
    pub is_conduction: bool,
    /// A via keepout (DSN `(via_keepout ...)`, Java `ViaObstacleArea`):
    /// blocks via placement but not traces.
    pub via_only: bool,
    /// For a conduction area (`is_conduction`): whether it also acts as a
    /// clearance obstacle to foreign-net copper (Java `ConductionArea`'s
    /// `is_obstacle` flag). Only KiCad JSON carries this; DSN planes are
    /// never obstacles, matching Java. Meaningless (and ignored) for keepouts,
    /// which are always obstacles via the general path.
    pub is_obstacle: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub enum ItemKind {
    Via(ViaItem),
    PolylineTrace(PolylineTraceItem),
    ObstacleArea(ObstacleAreaItem),
}

/// An item on the board.
#[derive(Debug, Clone)]
pub struct Item {
    pub base: ItemBase,
    pub kind: ItemKind,
    /// Lazily computed tile shapes: items are immutable once inserted,
    /// and recomputing trace offset shapes dominated the routing profile.
    cached_tile_shapes: std::sync::OnceLock<Vec<(TileShape, usize)>>,
    /// Cached bounding box (items are immutable once inserted, like the
    /// tile-shape cache; excluded from PartialEq the same way).
    cached_bounding_box: std::sync::OnceLock<IntBox>,
}

impl PartialEq for Item {
    fn eq(&self, other: &Self) -> bool {
        // the shape cache is derived state and excluded
        self.base == other.base && self.kind == other.kind
    }
}

impl Item {
    pub fn new_via(
        base: ItemBase,
        padstack: usize,
        center: IntPoint,
        attach_allowed: bool,
    ) -> Self {
        Item {
            base,
            kind: ItemKind::Via(ViaItem {
                padstack,
                center,
                attach_allowed,
            }),
            cached_tile_shapes: std::sync::OnceLock::new(),
            cached_bounding_box: std::sync::OnceLock::new(),
        }
    }

    pub fn new_polyline_trace(
        base: ItemBase,
        half_width: i32,
        layer: usize,
        polyline: Polyline,
    ) -> Self {
        Item {
            base,
            kind: ItemKind::PolylineTrace(PolylineTraceItem {
                half_width,
                layer,
                polyline,
            }),
            cached_tile_shapes: std::sync::OnceLock::new(),
            cached_bounding_box: std::sync::OnceLock::new(),
        }
    }

    /// True if this item can be routed to (Java: `Connectable` interface —
    /// traces, vias and conduction areas are connectable).
    pub fn is_connectable(&self) -> bool {
        match &self.kind {
            ItemKind::Via(_) | ItemKind::PolylineTrace(_) => true,
            ItemKind::ObstacleArea(area) => area.is_conduction,
        }
    }

    /// True if this item is a route item that can be changed by the
    /// autorouter (Java: `Item.is_routable`, false for fixed items and
    /// non-traces/vias).
    pub fn is_routable(&self) -> bool {
        !self.base.is_user_fixed()
            && matches!(self.kind, ItemKind::Via(_) | ItemKind::PolylineTrace(_))
    }

    /// The first board layer of this item.
    pub fn first_layer(&self, padstacks: &Padstacks) -> usize {
        match &self.kind {
            ItemKind::Via(via) => padstacks
                .get_by_no(via.padstack)
                .map(|p| p.from_layer())
                .unwrap_or(0),
            ItemKind::PolylineTrace(trace) => trace.layer,
            ItemKind::ObstacleArea(area) => area.layer,
        }
    }

    /// The last board layer of this item.
    pub fn last_layer(&self, padstacks: &Padstacks) -> usize {
        match &self.kind {
            ItemKind::Via(via) => padstacks
                .get_by_no(via.padstack)
                .map(|p| p.to_layer())
                .unwrap_or(0),
            ItemKind::PolylineTrace(trace) => trace.layer,
            ItemKind::ObstacleArea(area) => area.layer,
        }
    }

    pub fn is_on_layer(&self, layer: usize, padstacks: &Padstacks) -> bool {
        layer >= self.first_layer(padstacks) && layer <= self.last_layer(padstacks)
    }

    /// The number of shapes to store in the search tree.
    pub fn tile_shape_count(&self, padstacks: &Padstacks) -> usize {
        match &self.kind {
            ItemKind::Via(via) => {
                let Some(padstack) = padstacks.get_by_no(via.padstack) else {
                    return 0;
                };
                let from = padstack.from_layer();
                let to = padstack.to_layer();
                if to < from {
                    0
                } else {
                    to - from + 1
                }
            }
            ItemKind::PolylineTrace(trace) => {
                // one shape per polyline line segment
                trace.polyline.arr.len().saturating_sub(2)
            }
            ItemKind::ObstacleArea(area) => area
                .area
                .split_to_convex()
                .map(|pieces| pieces.len())
                .unwrap_or(0),
        }
    }

    /// The `index`-th shape of this item for the search tree, and the
    /// board layer it belongs to.
    pub fn tile_shape(&self, index: usize, padstacks: &Padstacks) -> Option<(TileShape, usize)> {
        match &self.kind {
            ItemKind::Via(via) => {
                let padstack = padstacks.get_by_no(via.padstack)?;
                let layer = padstack.from_layer() + index;
                if layer > padstack.to_layer() {
                    return None;
                }
                let shape = padstack.get_shape(layer)?;
                let translated = shape.translate_by(via.center.difference_by(IntPoint::ZERO));
                Some((translated, layer))
            }
            ItemKind::PolylineTrace(trace) => {
                let shape = trace.polyline.offset_shape(trace.half_width, index)?;
                Some((shape, trace.layer))
            }
            ItemKind::ObstacleArea(area) => {
                let pieces = area.area.split_to_convex()?;
                pieces.into_iter().nth(index).map(|s| (s, area.layer))
            }
        }
    }

    /// All search-tree shapes of this item with their layers. Computed
    /// once and cached (items are immutable once inserted).
    pub fn tile_shapes(&self, padstacks: &Padstacks) -> &[(TileShape, usize)] {
        self.cached_tile_shapes
            .get_or_init(|| self.compute_tile_shapes(padstacks))
    }

    fn compute_tile_shapes(&self, padstacks: &Padstacks) -> Vec<(TileShape, usize)> {
        match &self.kind {
            ItemKind::Via(_) => (0..self.tile_shape_count(padstacks))
                .filter_map(|i| self.tile_shape(i, padstacks))
                .collect(),
            ItemKind::PolylineTrace(trace) => {
                // offset_shapes computes all segments in one pass
                trace
                    .polyline
                    .offset_shapes(trace.half_width)
                    .into_iter()
                    .map(|s| (s, trace.layer))
                    .collect()
            }
            ItemKind::ObstacleArea(area) => area
                .area
                .split_to_convex()
                .unwrap_or_default()
                .into_iter()
                .map(|s| (s, area.layer))
                .collect(),
        }
    }

    /// The bounding box of this item (computed once; ~5% of a coldfire
    /// profile was recomputing trace boxes from corner approximations).
    pub fn bounding_box(&self, padstacks: &Padstacks) -> IntBox {
        *self.cached_bounding_box.get_or_init(|| match &self.kind {
            ItemKind::Via(_) => {
                let mut result = IntBox::EMPTY;
                for (shape, _) in self.tile_shapes(padstacks) {
                    result = result.union(shape.bounding_box());
                }
                result
            }
            ItemKind::PolylineTrace(trace) => trace
                .polyline
                .bounding_box()
                .offset(trace.half_width as f64),
            ItemKind::ObstacleArea(area) => area.area.bounding_box(),
        })
    }

    pub fn new_obstacle_area(
        base: ItemBase,
        area: PolylineArea,
        layer: usize,
        name: impl Into<String>,
        is_conduction: bool,
    ) -> Self {
        Item {
            base,
            kind: ItemKind::ObstacleArea(ObstacleAreaItem {
                area,
                layer,
                name: name.into(),
                is_conduction,
                via_only: false,
                is_obstacle: false,
            }),
            cached_tile_shapes: std::sync::OnceLock::new(),
            cached_bounding_box: std::sync::OnceLock::new(),
        }
    }
}

impl ViaItem {
    /// Moves the via by `vector` (Java: `DrillItem.translate_by`).
    pub fn translate_by(&mut self, vector: crate::geometry::planar::IntVector) {
        self.center = self.center.translate_by(vector);
    }
}

impl PolylineTraceItem {
    pub fn first_corner(&self) -> Point {
        self.polyline.first_corner()
    }

    pub fn last_corner(&self) -> Point {
        self.polyline.last_corner()
    }

    pub fn corner_count(&self) -> usize {
        self.polyline.corner_count()
    }

    /// The length of this trace.
    pub fn get_length(&self) -> f64 {
        self.polyline.length_approx()
    }

    /// True if this trace touches `point` (on its center line).
    pub fn contains_on_center_line(&self, point: IntPoint) -> bool {
        self.polyline.contains(point)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::planar::{IntVector, TileShape};

    fn padstacks() -> Padstacks {
        let mut p = Padstacks::new(2);
        let shape = TileShape::Box(IntBox::from_coords(-400, -400, 400, 400));
        p.add_shape_on_layers(shape, 0, 1); // padstack 1: through via
        p
    }

    #[test]
    fn fixed_state_ordering() {
        assert!(FixedState::Unfixed < FixedState::ShoveFixed);
        assert!(FixedState::ShoveFixed < FixedState::UserFixed);
        assert!(FixedState::UserFixed < FixedState::SystemFixed);
        let mut base = ItemBase::new(1, vec![1], 1);
        assert!(!base.is_shove_fixed());
        base.fixed_state = FixedState::ShoveFixed;
        assert!(base.is_shove_fixed());
        assert!(!base.is_user_fixed());
        base.unfix();
        assert_eq!(base.fixed_state, FixedState::Unfixed);
        base.fixed_state = FixedState::SystemFixed;
        base.unfix();
        assert_eq!(base.fixed_state, FixedState::SystemFixed);
    }

    #[test]
    fn net_membership() {
        let a = ItemBase::new(1, vec![1, 2], 1);
        let b = ItemBase::new(2, vec![2, 3], 1);
        let c = ItemBase::new(3, vec![4], 1);
        assert!(a.contains_net(2));
        assert!(!a.contains_net(3));
        assert!(a.shares_net(&b));
        assert!(!a.shares_net(&c));
    }

    #[test]
    fn via_shapes_and_layers() {
        let padstacks = padstacks();
        let via = Item::new_via(
            ItemBase::new(1, vec![1], 1),
            1,
            IntPoint::new(1000, 2000),
            false,
        );
        assert_eq!(via.first_layer(&padstacks), 0);
        assert_eq!(via.last_layer(&padstacks), 1);
        assert!(via.is_on_layer(0, &padstacks));
        assert!(via.is_on_layer(1, &padstacks));
        assert_eq!(via.tile_shape_count(&padstacks), 2);
        let shapes = via.tile_shapes(&padstacks);
        assert_eq!(shapes.len(), 2);
        // the pad shape is translated to the via center
        assert_eq!(
            shapes[0].0.bounding_box(),
            IntBox::from_coords(600, 1600, 1400, 2400)
        );
        assert_eq!(shapes[0].1, 0);
        assert_eq!(shapes[1].1, 1);
        assert_eq!(
            via.bounding_box(&padstacks),
            IntBox::from_coords(600, 1600, 1400, 2400)
        );
        assert!(via.is_routable());
        assert!(via.is_connectable());
    }

    #[test]
    fn trace_shapes_and_geometry() {
        let padstacks = padstacks();
        let polyline = Polyline::from_int_points(&[
            IntPoint::new(0, 0),
            IntPoint::new(1000, 0),
            IntPoint::new(1000, 1000),
        ]);
        let trace = Item::new_polyline_trace(ItemBase::new(2, vec![1], 1), 100, 1, polyline);
        assert_eq!(trace.first_layer(&padstacks), 1);
        assert_eq!(trace.last_layer(&padstacks), 1);
        assert!(!trace.is_on_layer(0, &padstacks));
        assert_eq!(trace.tile_shape_count(&padstacks), 2);
        let shapes = trace.tile_shapes(&padstacks);
        assert_eq!(shapes.len(), 2);
        for (shape, layer) in shapes {
            assert_eq!(*layer, 1);
            assert!(!shape.is_empty());
        }
        // shape 0 covers the first segment with the half width
        assert!(shapes[0].0.contains(&Point::Int(IntPoint::new(500, 90))));
        assert!(shapes[0].0.contains(&Point::Int(IntPoint::new(500, -90))));
        assert!(!shapes[0].0.contains(&Point::Int(IntPoint::new(500, 200))));

        let ItemKind::PolylineTrace(t) = &trace.kind else {
            unreachable!()
        };
        assert_eq!(t.first_corner(), Point::Int(IntPoint::new(0, 0)));
        assert_eq!(t.last_corner(), Point::Int(IntPoint::new(1000, 1000)));
        assert!((t.get_length() - 2000.0).abs() < 1e-9);
        assert!(t.contains_on_center_line(IntPoint::new(500, 0)));
        assert!(!t.contains_on_center_line(IntPoint::new(500, 50)));
        assert_eq!(
            trace.bounding_box(&padstacks),
            IntBox::from_coords(-100, -100, 1100, 1100)
        );

        // single-shape tile access matches the batch computation
        let (single, _) = trace.tile_shape(0, &padstacks).unwrap();
        assert_eq!(single, shapes[0].0);
    }

    #[test]
    fn via_translate() {
        let padstacks = padstacks();
        let mut via = Item::new_via(ItemBase::new(1, vec![1], 1), 1, IntPoint::new(0, 0), false);
        if let ItemKind::Via(v) = &mut via.kind {
            v.translate_by(IntVector::new(500, -500));
        }
        assert_eq!(
            via.bounding_box(&padstacks),
            IntBox::from_coords(100, -900, 900, -100)
        );
    }
}
