//! Port of `app.freerouting.board` (incremental).

pub mod angle_restriction;
pub mod basic_board;
pub mod item;
pub mod layer;

pub use angle_restriction::AngleRestriction;
pub use basic_board::{BasicBoard, ItemId};
pub use item::{FixedState, Item, ItemBase, ItemKind, PolylineTraceItem, ViaItem};
pub use layer::{Layer, LayerStructure};
