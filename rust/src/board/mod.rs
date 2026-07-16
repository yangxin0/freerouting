//! Port of `app.freerouting.board` (incremental).

pub mod angle_restriction;
pub mod basic_board;
pub mod calc_from_side;
pub mod calc_shape_and_from_side;
pub mod forced_via;
pub mod item;
pub mod layer;
pub mod move_drill_item;
pub mod opt_via;
pub mod shape_trace_entries;
pub mod shove_trace_algo;

pub use angle_restriction::AngleRestriction;
pub use basic_board::{BasicBoard, ItemId};
pub use calc_from_side::CalcFromSide;
pub use calc_shape_and_from_side::CalcShapeAndFromSide;
pub use item::{
    FixedState, Item, ItemBase, ItemKind, ObstacleAreaItem, PolylineTraceItem, ViaItem,
};
pub use layer::{Layer, LayerStructure};
pub use shape_trace_entries::{cutout_trace, ShapeTraceEntries};
pub use shove_trace_algo::shove_aside;
