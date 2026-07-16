//! Port of `core/Padstack.java` and `core/Padstacks.java`.
//!
//! A padstack is the mask of a pin or via at the origin: one optional
//! convex shape per board layer. Java's `ConvexShape` becomes
//! [`TileShape`] here (circles are approximated/deferred like elsewhere in
//! the port).

use crate::geometry::planar::{IntBox, IntDirection, TileShape};
use std::cell::Cell;

#[derive(Debug, Clone)]
pub struct Padstack {
    pub name: String,
    /// 1-based number of this padstack in its list.
    pub no: usize,
    /// True if vias of the own net are allowed to overlap with this
    /// padstack.
    pub attach_allowed: bool,
    /// If false, the layers of the padstack are mirrored if it is placed
    /// on the back side.
    pub placed_absolute: bool,
    shapes: Vec<Option<TileShape>>,
    /// Cached drill radius (computed from the name on first use).
    cached_drill_radius: Cell<Option<f64>>,
    cached_bounding_box: Cell<Option<IntBox>>,
}

impl Padstack {
    fn new(
        name: String,
        no: usize,
        shapes: Vec<Option<TileShape>>,
        attach_allowed: bool,
        placed_absolute: bool,
    ) -> Self {
        Padstack {
            name,
            no,
            attach_allowed,
            placed_absolute,
            shapes,
            cached_drill_radius: Cell::new(None),
            cached_bounding_box: Cell::new(None),
        }
    }

    /// The shape of this padstack on `layer`.
    pub fn get_shape(&self, layer: usize) -> Option<&TileShape> {
        self.shapes.get(layer).and_then(|s| s.as_ref())
    }

    /// The first layer with a shape.
    pub fn from_layer(&self) -> usize {
        self.shapes
            .iter()
            .position(Option::is_some)
            .unwrap_or(self.shapes.len())
    }

    /// The last layer with a shape (`board_layer_count()` underflows to a
    /// huge value in Java when empty; here we mirror with a saturating 0).
    pub fn to_layer(&self) -> usize {
        self.shapes.iter().rposition(Option::is_some).unwrap_or(0)
    }

    /// The layer count of the board of this padstack.
    pub fn board_layer_count(&self) -> usize {
        self.shapes.len()
    }

    /// The bounding box of all shapes of this padstack (origin-relative).
    /// Cached after the first call.
    pub fn bounding_box(&self) -> IntBox {
        if let Some(cached) = self.cached_bounding_box.get() {
            return cached;
        }
        let mut bb = IntBox::EMPTY;
        for shape in self.shapes.iter().flatten() {
            bb = bb.union(shape.bounding_box());
        }
        self.cached_bounding_box.set(Some(bb));
        bb
    }

    /// The smallest half extent of any shape of this padstack.
    fn get_smallest_radius(&self) -> f64 {
        let mut min_radius = f64::MAX;
        for shape in self.shapes.iter().flatten() {
            let bb = shape.bounding_box();
            let radius = bb.width().min(bb.height()) as f64 / 2.0;
            min_radius = min_radius.min(radius);
        }
        if min_radius == f64::MAX {
            0.0
        } else {
            min_radius
        }
    }

    /// The drill radius in board units, derived from padstack names of the
    /// form `..._<outer>:<drill>_...` when possible, otherwise 45% of the
    /// smallest pad radius (like Java). Cached after the first call.
    pub fn get_drill_radius(&self) -> f64 {
        if let Some(cached) = self.cached_drill_radius.get() {
            return cached;
        }
        let result = self
            .drill_radius_from_name()
            .unwrap_or_else(|| self.get_smallest_radius() * 0.45);
        self.cached_drill_radius.set(Some(result));
        result
    }

    fn drill_radius_from_name(&self) -> Option<f64> {
        let colon_index = self.name.find(':')?;
        let after_colon = &self.name[colon_index + 1..];
        let drill_str = match after_colon.find('_') {
            Some(underscore) => &after_colon[..underscore],
            None => after_colon,
        };
        let drill_dia: f64 = strip_non_numeric(drill_str).parse().ok()?;
        let last_underscore = self.name[..colon_index].rfind('_')?;
        let outer_str = &self.name[last_underscore + 1..colon_index];
        let outer_dia: f64 = strip_non_numeric(outer_str).parse().ok()?;
        if outer_dia <= 0.0 {
            return None;
        }
        let actual_outer_radius = self.get_smallest_radius();
        if actual_outer_radius <= 0.0 {
            return None;
        }
        Some(actual_outer_radius * (drill_dia / outer_dia))
    }

    /// The allowed trace exit directions of the pad shape on `layer`. If
    /// the length of the pad is smaller than `factor` times its height,
    /// connection to the long side is also allowed. Only box and octagon
    /// pads restrict directions.
    pub fn get_trace_exit_directions(&self, layer: usize, factor: f64) -> Vec<IntDirection> {
        let mut result = Vec::new();
        let Some(Some(shape)) = self.shapes.get(layer) else {
            return result;
        };
        if !matches!(shape, TileShape::Box(_) | TileShape::Octagon(_)) {
            return result;
        }
        let bb = shape.bounding_box();
        let (w, h) = (bb.width() as f64, bb.height() as f64);
        let all_dirs = w.max(h) < factor * w.min(h);
        if all_dirs || w >= h {
            result.push(IntDirection::RIGHT);
            result.push(IntDirection::LEFT);
        }
        if all_dirs || w <= h {
            result.push(IntDirection::UP);
            result.push(IntDirection::DOWN);
        }
        result
    }
}

fn strip_non_numeric(s: &str) -> String {
    s.chars()
        .filter(|c| c.is_ascii_digit() || *c == '.')
        .collect()
}

/// A library of padstacks; padstack numbers are 1-based like in Java.
#[derive(Debug, Clone)]
pub struct Padstacks {
    pub board_layer_count: usize,
    padstack_arr: Vec<Padstack>,
}

impl Padstacks {
    pub fn new(board_layer_count: usize) -> Self {
        Padstacks {
            board_layer_count,
            padstack_arr: Vec::new(),
        }
    }

    /// The padstack with `name` (case-insensitive).
    pub fn get(&self, name: &str) -> Option<&Padstack> {
        self.padstack_arr
            .iter()
            .find(|p| p.name.eq_ignore_ascii_case(name))
    }

    pub fn count(&self) -> usize {
        self.padstack_arr.len()
    }

    /// The padstack with 1-based number `padstack_no`.
    pub fn get_by_no(&self, padstack_no: usize) -> Option<&Padstack> {
        if padstack_no == 0 {
            return None;
        }
        self.padstack_arr.get(padstack_no - 1)
    }

    /// Appends a new padstack; `shapes` must have board-layer-count
    /// entries. Returns its 1-based number.
    pub fn add(
        &mut self,
        name: impl Into<String>,
        shapes: Vec<Option<TileShape>>,
        drill_allowed: bool,
        placed_absolute: bool,
    ) -> usize {
        debug_assert_eq!(shapes.len(), self.board_layer_count);
        let no = self.padstack_arr.len() + 1;
        self.padstack_arr.push(Padstack::new(
            name.into(),
            no,
            shapes,
            drill_allowed,
            placed_absolute,
        ));
        no
    }

    /// Appends a new padstack with a generated name.
    pub fn add_with_generated_name(&mut self, shapes: Vec<Option<TileShape>>) -> usize {
        let name = format!("padstack#{}", self.padstack_arr.len() + 1);
        self.add(name, shapes, false, false)
    }

    /// Appends a new padstack with `shape` on the layers `from_layer` to
    /// `to_layer` and none elsewhere; the name is generated.
    pub fn add_shape_on_layers(
        &mut self,
        shape: TileShape,
        from_layer: usize,
        to_layer: usize,
    ) -> usize {
        let to_layer = to_layer.min(self.board_layer_count.saturating_sub(1));
        let shapes = (0..self.board_layer_count)
            .map(|i| {
                if i >= from_layer && i <= to_layer {
                    Some(shape.clone())
                } else {
                    None
                }
            })
            .collect();
        self.add_with_generated_name(shapes)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::planar::IntBox;

    fn box_shape(half_width: i32) -> TileShape {
        TileShape::Box(IntBox::from_coords(
            -half_width,
            -half_width,
            half_width,
            half_width,
        ))
    }

    #[test]
    fn add_and_layer_ranges() {
        let mut padstacks = Padstacks::new(4);
        let via = padstacks.add_shape_on_layers(box_shape(400), 0, 3);
        let smd = padstacks.add_shape_on_layers(box_shape(300), 0, 0);
        assert_eq!(padstacks.count(), 2);

        let via = padstacks.get_by_no(via).unwrap();
        assert_eq!(via.from_layer(), 0);
        assert_eq!(via.to_layer(), 3);
        assert_eq!(via.board_layer_count(), 4);
        assert!(via.get_shape(2).is_some());

        let smd = padstacks.get_by_no(smd).unwrap();
        assert_eq!(smd.from_layer(), 0);
        assert_eq!(smd.to_layer(), 0);
        assert!(smd.get_shape(1).is_none());
        assert_eq!(smd.name, "padstack#2");
        assert!(padstacks.get("PADSTACK#2").is_some());
        assert!(padstacks.get_by_no(0).is_none());
    }

    #[test]
    fn drill_radius_from_name() {
        let mut padstacks = Padstacks::new(2);
        // outer diameter 800, drill 400: half the outer radius
        let no = padstacks.add(
            "Via[0-1]_800:400_um",
            vec![Some(box_shape(400)), Some(box_shape(400))],
            true,
            false,
        );
        let p = padstacks.get_by_no(no).unwrap();
        assert!((p.get_drill_radius() - 200.0).abs() < 1e-9);
        // cached second call
        assert!((p.get_drill_radius() - 200.0).abs() < 1e-9);

        // no parsable name: 45% of the smallest radius
        let no = padstacks.add("round_pad", vec![Some(box_shape(400)), None], false, false);
        let p = padstacks.get_by_no(no).unwrap();
        assert!((p.get_drill_radius() - 400.0 * 0.45).abs() < 1e-9);
    }

    #[test]
    fn trace_exit_directions() {
        let mut padstacks = Padstacks::new(1);
        // wide pad: exits only left/right
        let wide = TileShape::Box(IntBox::from_coords(-300, -100, 300, 100));
        let no = padstacks.add("wide", vec![Some(wide)], false, false);
        let p = padstacks.get_by_no(no).unwrap();
        let dirs = p.get_trace_exit_directions(0, 2.0);
        assert_eq!(dirs, vec![IntDirection::RIGHT, IntDirection::LEFT]);
        // nearly square pad with generous factor: all four directions
        let dirs = p.get_trace_exit_directions(0, 4.0);
        assert_eq!(dirs.len(), 4);
        // out of range layer
        assert!(p.get_trace_exit_directions(1, 2.0).is_empty());
    }
}
