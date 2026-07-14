//! Port of `app.freerouting.board` (incremental).

pub mod angle_restriction;
pub mod calc_from_side;
pub mod calc_shape_and_from_side;
pub mod shape_trace_entries;
pub mod shove_trace_algo;
pub mod basic_board;
pub mod item;
pub mod layer;

pub use angle_restriction::AngleRestriction;
pub use calc_from_side::CalcFromSide;
pub use calc_shape_and_from_side::CalcShapeAndFromSide;
pub use shape_trace_entries::{cutout_trace, ShapeTraceEntries};
pub use shove_trace_algo::shove_aside;
pub use basic_board::{BasicBoard, ItemId};
pub use item::{
    FixedState, Item, ItemBase, ItemKind, ObstacleAreaItem, PolylineTraceItem, ViaItem,
};
pub use layer::{Layer, LayerStructure};
