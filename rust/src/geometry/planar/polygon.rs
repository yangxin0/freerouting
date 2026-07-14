//! Port of `geometry/planar/Polygon.java` (winding number deferred).
//!
//! A list of points in the plane where no 2 consecutive points are equal
//! and no 3 consecutive points are collinear.

use crate::geometry::planar::{Point, Side};

#[derive(Debug, Clone, PartialEq)]
pub struct Polygon {
    corners: Vec<Point>,
}

impl Polygon {
    /// Creates a polygon from `points`; duplicates and points collinear
    /// with their previous and next point are removed.
    pub fn new(points: Vec<Point>) -> Self {
        let mut corners = points;
        let mut corner_removed = true;
        while corner_removed {
            corner_removed = false;
            if corners.is_empty() {
                break;
            }
            // remove multiple points
            let mut i = 1;
            while i < corners.len() {
                if corners[i] == corners[i - 1] {
                    corners.remove(i);
                    corner_removed = true;
                } else {
                    i += 1;
                }
            }
            // remove points collinear with the previous and next point
            // (like Java: remove the first one found, then restart)
            for i in 1..corners.len().saturating_sub(1) {
                if corners[i].side_of(&corners[i - 1], &corners[i + 1]) == Side::Collinear {
                    corners.remove(i);
                    corner_removed = true;
                    break;
                }
            }
        }
        Polygon { corners }
    }

    pub fn corner_array(&self) -> &[Point] {
        &self.corners
    }

    pub fn corner_count(&self) -> usize {
        self.corners.len()
    }

    /// Reverts the order of the corners.
    pub fn revert_corners(&self) -> Self {
        let mut reversed = self.corners.clone();
        reversed.reverse();
        Polygon::new(reversed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::planar::IntPoint;

    fn p(x: i32, y: i32) -> Point {
        Point::Int(IntPoint::new(x, y))
    }

    #[test]
    fn removes_duplicates_and_collinear() {
        let poly = Polygon::new(vec![
            p(0, 0),
            p(0, 0),   // duplicate
            p(5, 0),   // collinear with (0,0) and (10,0)
            p(10, 0),
            p(10, 10),
        ]);
        assert_eq!(poly.corner_array(), &[p(0, 0), p(10, 0), p(10, 10)]);
        let reverted = poly.revert_corners();
        assert_eq!(reverted.corner_array(), &[p(10, 10), p(10, 0), p(0, 0)]);
    }
}
