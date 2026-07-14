//! Port of `app.freerouting.datastructures`.

pub mod big_int_aux;
pub mod min_area_tree;
pub mod signum;

pub use min_area_tree::{LeafId, MinAreaTree, TreeEntry};
pub use signum::Signum;
