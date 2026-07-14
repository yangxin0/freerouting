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

    /// The winding number of this polygon treated as closed: > 0 if the
    /// corners are counterclockwise, < 0 if clockwise.
    pub fn winding_number_after_closing(&self) -> i32 {
        let corners = &self.corners;
        if corners.len() < 2 {
            return 0;
        }
        let first_side_vector = corners[1].difference_by(&corners[0]);
        let mut prev_side_vector = first_side_vector.clone();
        let mut corner_count = corners.len();
        // skip the last corner if it equals the first
        if corners[0] == corners[corner_count - 1] {
            corner_count -= 1;
        }
        let mut angle_sum = 0.0;
        for i in 1..=corner_count {
            let next_side_vector = if i == corner_count - 1 {
                corners[0].difference_by(&corners[i])
            } else if i == corner_count {
                first_side_vector.clone()
            } else {
                corners[i + 1].difference_by(&corners[i])
            };
            angle_sum += prev_side_vector.angle_approx_with(&next_side_vector);
            prev_side_vector = next_side_vector;
        }
        (angle_sum / (2.0 * std::f64::consts::PI)).round() as i32
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
