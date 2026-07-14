//! Port of `geometry/planar/PolylineArea.java` (and the `Area.java`
//! interface it implements).
//!
//! An area with straight-line borders: an outside border shape plus hole
//! shapes. The border and holes are [`PolygonShape`]s in this port.

use crate::geometry::planar::{
    FloatPoint, IntBox, IntOctagon, IntPoint, IntVector, Point, PolygonShape, TileShape,
};

#[derive(Debug, Clone, PartialEq)]
pub struct PolylineArea {
    pub border_shape: PolygonShape,
    pub holes: Vec<PolygonShape>,
}

impl PolylineArea {
    pub fn new(border_shape: PolygonShape, holes: Vec<PolygonShape>) -> Self {
        PolylineArea {
            border_shape,
            holes,
        }
    }

    pub fn dimension(&self) -> i32 {
        self.border_shape.dimension()
    }

    pub fn is_bounded(&self) -> bool {
        self.border_shape.is_bounded()
    }

    pub fn is_empty(&self) -> bool {
        self.border_shape.is_empty()
    }

    pub fn get_border(&self) -> &PolygonShape {
        &self.border_shape
    }

    pub fn get_holes(&self) -> &[PolygonShape] {
        &self.holes
    }

    pub fn bounding_box(&self) -> IntBox {
        self.border_shape.bounding_box()
    }

    pub fn bounding_octagon(&self) -> IntOctagon {
        self.border_shape.bounding_octagon()
    }

    pub fn contains_float(&self, point: FloatPoint) -> bool {
        self.border_shape.contains_float(point)
            && !self.holes.iter().any(|h| h.contains_float(point))
    }

    pub fn contains(&self, point: &Point) -> bool {
        // Note: Java excludes points strictly inside holes
        // (contains_inside); since PolygonShape.contains_on_border is a
        // stub returning false in Java, contains_inside equals contains
        // there, matched here.
        self.border_shape.contains(point) && !self.holes.iter().any(|h| h.contains(point))
    }

    /// The corners of the border and all holes, approximated.
    pub fn corner_approx_arr(&self) -> Vec<FloatPoint> {
        let mut result: Vec<FloatPoint> = self
            .border_shape
            .corners
            .iter()
            .map(|c| c.to_float())
            .collect();
        for hole in &self.holes {
            result.extend(hole.corners.iter().map(|c| c.to_float()));
        }
        result
    }

    /// An approximation of the nearest point of this area to `from_point`.
    pub fn nearest_point_approx(&self, from_point: FloatPoint) -> Option<FloatPoint> {
        let mut min_dist = f64::MAX;
        let mut result = None;
        for piece in self.split_to_convex()? {
            let curr = piece.nearest_point_approx(from_point);
            let curr_dist = curr.distance_square(from_point);
            if curr_dist < min_dist {
                min_dist = curr_dist;
                result = Some(curr);
            }
        }
        result
    }

    /// Splits this area into convex pieces: the border pieces with all
    /// hole pieces cut out. Returns `None` if a split fails.
    pub fn split_to_convex(&self) -> Option<Vec<TileShape>> {
        let mut curr_piece_list = self.border_shape.split_to_convex()?;
        for hole in &self.holes {
            if hole.dimension() < 2 {
                // dimension 2 expected for holes
                continue;
            }
            for hole_piece in hole.split_to_convex()? {
                let mut new_piece_list = Vec::new();
                for divide_piece in &curr_piece_list {
                    for piece in divide_piece.cutout(&hole_piece) {
                        if piece.dimension() == 2 {
                            new_piece_list.push(piece);
                        }
                    }
                }
                curr_piece_list = new_piece_list;
            }
        }
        Some(curr_piece_list)
    }

    pub fn translate_by(&self, vector: IntVector) -> Self {
        if vector.is_zero() {
            return self.clone();
        }
        PolylineArea::new(
            self.border_shape.translate_by(vector),
            self.holes.iter().map(|h| h.translate_by(vector)).collect(),
        )
    }

    pub fn turn_90_degree(&self, factor: i32, pole: IntPoint) -> Self {
        PolylineArea::new(
            self.border_shape.turn_90_degree(factor, pole),
            self.holes
                .iter()
                .map(|h| h.turn_90_degree(factor, pole))
                .collect(),
        )
    }

    pub fn rotate_approx(&self, angle: f64, pole: FloatPoint) -> Self {
        PolylineArea::new(
            self.border_shape.rotate_approx(angle, pole),
            self.holes
                .iter()
                .map(|h| h.rotate_approx(angle, pole))
                .collect(),
        )
    }

    pub fn mirror_vertical(&self, pole: IntPoint) -> Self {
        PolylineArea::new(
            self.border_shape.mirror_vertical(pole),
            self.holes
                .iter()
                .map(|h| h.mirror_vertical(pole))
                .collect(),
        )
    }

    pub fn mirror_horizontal(&self, pole: IntPoint) -> Self {
        PolylineArea::new(
            self.border_shape.mirror_horizontal(pole),
            self.holes
                .iter()
                .map(|h| h.mirror_horizontal(pole))
                .collect(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square(llx: i32, lly: i32, size: i32) -> PolygonShape {
        PolygonShape::from_int_points(&[
            IntPoint::new(llx, lly),
            IntPoint::new(llx + size, lly),
            IntPoint::new(llx + size, lly + size),
            IntPoint::new(llx, lly + size),
        ])
    }

    #[test]
    fn area_with_hole() {
        let area = PolylineArea::new(square(0, 0, 100), vec![square(40, 40, 20)]);
        assert_eq!(area.dimension(), 2);
        assert!(!area.is_empty());
        assert_eq!(area.bounding_box(), IntBox::from_coords(0, 0, 100, 100));

        // containment respects the hole
        assert!(area.contains(&Point::Int(IntPoint::new(10, 10))));
        assert!(!area.contains(&Point::Int(IntPoint::new(50, 50))));
        assert!(!area.contains(&Point::Int(IntPoint::new(150, 50))));
        assert!(area.contains_float(FloatPoint::new(10.0, 10.0)));
        assert!(!area.contains_float(FloatPoint::new(50.0, 50.0)));

        // convex decomposition covers border minus hole
        let pieces = area.split_to_convex().unwrap();
        assert!(pieces.len() >= 4);
        let total: f64 = pieces.iter().map(|p| p.area()).sum();
        assert!(
            (total - (100.0 * 100.0 - 20.0 * 20.0)).abs() < 1e-6,
            "total {total}"
        );
        for piece in &pieces {
            assert!(!piece.contains_inside(&Point::Int(IntPoint::new(50, 50))));
        }

        // corners of border and hole are reported
        assert_eq!(area.corner_approx_arr().len(), 8);
    }

    #[test]
    fn concave_border_with_hole() {
        // L-shaped border with a hole in the wide part
        let l_border = PolygonShape::from_int_points(&[
            IntPoint::new(0, 0),
            IntPoint::new(100, 0),
            IntPoint::new(100, 50),
            IntPoint::new(50, 50),
            IntPoint::new(50, 100),
            IntPoint::new(0, 100),
        ]);
        let area = PolylineArea::new(l_border.clone(), vec![square(10, 10, 10)]);
        let pieces = area.split_to_convex().unwrap();
        let total: f64 = pieces.iter().map(|p| p.area()).sum();
        assert!(
            (total - (l_border.area() - 100.0)).abs() < 1e-6,
            "total {total} vs {}",
            l_border.area() - 100.0
        );
        assert!(!area.contains(&Point::Int(IntPoint::new(15, 15))));
        assert!(area.contains(&Point::Int(IntPoint::new(30, 30))));
        assert!(!area.contains(&Point::Int(IntPoint::new(80, 80))));
    }

    #[test]
    fn transforms_and_nearest() {
        let area = PolylineArea::new(square(0, 0, 10), vec![]);
        let moved = area.translate_by(IntVector::new(100, 0));
        assert_eq!(moved.bounding_box(), IntBox::from_coords(100, 0, 110, 10));
        let turned = area.turn_90_degree(1, IntPoint::new(0, 0));
        assert_eq!(turned.bounding_box(), IntBox::from_coords(-10, 0, 0, 10));
        let near = area.nearest_point_approx(FloatPoint::new(20.0, 5.0)).unwrap();
        assert_eq!(near, FloatPoint::new(10.0, 5.0));
    }
}
