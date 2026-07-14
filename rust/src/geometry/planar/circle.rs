//! Port of `geometry/planar/Circle.java`.
//!
//! A circle shape used for round pads. Exact routing geometry converts it
//! to a bounding tile (octagon or finer tangent polygon).

use crate::geometry::planar::{
    limits, FloatPoint, IntBox, IntOctagon, IntPoint, IntVector, Line, Point, TileShape,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Circle {
    pub center: IntPoint,
    pub radius: i32,
}

impl Circle {
    pub fn new(center: IntPoint, radius: i32) -> Self {
        Circle {
            center,
            radius: radius.abs(),
        }
    }

    pub fn is_empty(&self) -> bool {
        false
    }

    pub fn is_bounded(&self) -> bool {
        true
    }

    pub fn dimension(&self) -> i32 {
        if self.radius == 0 {
            0 // reduced to a point
        } else {
            2
        }
    }

    pub fn circumference(&self) -> f64 {
        2.0 * std::f64::consts::PI * self.radius as f64
    }

    pub fn area(&self) -> f64 {
        std::f64::consts::PI * self.radius as f64 * self.radius as f64
    }

    pub fn centre_of_gravity(&self) -> FloatPoint {
        self.center.to_float()
    }

    pub fn is_outside(&self, point: &Point) -> bool {
        let fp = point.to_float();
        fp.distance_square(self.center.to_float()) > self.radius as f64 * self.radius as f64
    }

    pub fn contains(&self, point: &Point) -> bool {
        !self.is_outside(point)
    }

    pub fn contains_inside(&self, point: &Point) -> bool {
        let fp = point.to_float();
        fp.distance_square(self.center.to_float()) < self.radius as f64 * self.radius as f64
    }

    pub fn contains_float(&self, point: FloatPoint) -> bool {
        point.distance_square(self.center.to_float()) <= self.radius as f64 * self.radius as f64
    }

    /// The distance of `point` to this circle (0 inside).
    pub fn distance(&self, point: FloatPoint) -> f64 {
        (point.distance(self.center.to_float()) - self.radius as f64).max(0.0)
    }

    /// The distance of `point` to the circle border.
    pub fn border_distance(&self, point: FloatPoint) -> f64 {
        (point.distance(self.center.to_float()) - self.radius as f64).abs()
    }

    pub fn smallest_radius(&self) -> f64 {
        self.radius as f64
    }

    pub fn max_width(&self) -> f64 {
        2.0 * self.radius as f64
    }

    pub fn min_width(&self) -> f64 {
        2.0 * self.radius as f64
    }

    pub fn bounding_box(&self) -> IntBox {
        IntBox::from_coords(
            self.center.x - self.radius,
            self.center.y - self.radius,
            self.center.x + self.radius,
            self.center.y + self.radius,
        )
    }

    pub fn bounding_octagon(&self) -> IntOctagon {
        let lx = self.center.x - self.radius;
        let rx = self.center.x + self.radius;
        let ly = self.center.y - self.radius;
        let uy = self.center.y + self.radius;

        let sqrt2_minus_1 = limits::SQRT2 - 1.0;
        let ceil_corner_value = (sqrt2_minus_1 * self.radius as f64).ceil() as i32;
        let floor_corner_value = (sqrt2_minus_1 * self.radius as f64).floor() as i32;

        IntOctagon::new(
            lx,
            ly,
            rx,
            uy,
            lx - (self.center.y + floor_corner_value),
            rx - (self.center.y - ceil_corner_value),
            lx + (self.center.y - floor_corner_value),
            rx + (self.center.y + ceil_corner_value),
        )
    }

    /// The bounding tile used in exact routing calculations (Java returns
    /// the bounding octagon; the finer approximation caused problems with
    /// the spring-over algorithm).
    pub fn bounding_tile(&self) -> TileShape {
        TileShape::Octagon(self.bounding_octagon())
    }

    /// A bounding tile whose border segments are at most
    /// `max_segment_length` long, built from tangent lines.
    pub fn bounding_tile_with_segment_length(&self, max_segment_length: i32) -> TileShape {
        let quadrant_division_count = self.radius / max_segment_length + 1;
        if quadrant_division_count <= 2 {
            return self.bounding_tile();
        }
        let n = quadrant_division_count as usize;
        let mut tangent_lines: Vec<Line> = vec![Line::from_coords(0, 0, 1, 0); n * 4];
        for i in 0..n {
            // the tangential points in the first quadrant
            let border_delta = if i == 0 {
                IntVector::new(self.radius, 0)
            } else {
                let curr_angle = i as f64 * std::f64::consts::PI / (2.0 * n as f64);
                IntVector::new(
                    (curr_angle.sin() * self.radius as f64).ceil() as i32,
                    (curr_angle.cos() * self.radius as f64).ceil() as i32,
                )
            };
            let curr_a = self.center.translate_by(border_delta);
            let curr_b = self
                .center
                .translate_by(curr_a.difference_by(self.center).turn_90_degree(1));
            let curr_dir = crate::geometry::planar::IntDirection::from_points(self.center, curr_b)
                .expect("tangent point must differ from center");
            let curr_tangent = Line::from_direction(curr_a, curr_dir);
            tangent_lines[n + i] = curr_tangent;
            tangent_lines[2 * n + i] = curr_tangent.turn_90_degree(1, self.center);
            tangent_lines[3 * n + i] = curr_tangent.turn_90_degree(2, self.center);
            tangent_lines[i] = curr_tangent.turn_90_degree(3, self.center);
        }
        TileShape::get_instance(tangent_lines)
    }

    pub fn is_contained_in(&self, box_: IntBox) -> bool {
        box_.ll.x <= self.center.x - self.radius
            && box_.ll.y <= self.center.y - self.radius
            && box_.ur.x >= self.center.x + self.radius
            && box_.ur.y >= self.center.y + self.radius
    }

    pub fn turn_90_degree(&self, factor: i32, pole: IntPoint) -> Self {
        let new_center = pole.translate_by(self.center.difference_by(pole).turn_90_degree(factor));
        Circle::new(new_center, self.radius)
    }

    pub fn rotate_approx(&self, angle: f64, pole: FloatPoint) -> Self {
        Circle::new(self.center.to_float().rotate(angle, pole).round(), self.radius)
    }

    pub fn mirror_vertical(&self, pole: IntPoint) -> Self {
        let new_center = pole.translate_by(self.center.difference_by(pole).mirror_at_y_axis());
        Circle::new(new_center, self.radius)
    }

    pub fn mirror_horizontal(&self, pole: IntPoint) -> Self {
        let new_center = pole.translate_by(self.center.difference_by(pole).mirror_at_x_axis());
        Circle::new(new_center, self.radius)
    }

    pub fn translate_by(&self, vector: IntVector) -> Self {
        Circle::new(self.center.translate_by(vector), self.radius)
    }

    pub fn offset(&self, offset: f64) -> Self {
        Circle::new(self.center, (self.radius as f64 + offset).round() as i32)
    }

    pub fn shrink(&self, offset: f64) -> Self {
        let new_radius = ((self.radius as f64 - offset).round() as i32).max(1);
        Circle::new(self.center, new_radius)
    }

    pub fn enlarge(&self, offset: f64) -> Self {
        if offset == 0.0 {
            return *self;
        }
        Circle::new(self.center, self.radius + offset.round() as i32)
    }

    pub fn intersects_circle(&self, other: &Circle) -> bool {
        let d = (self.radius + other.radius) as f64;
        (self.center.distance_square(other.center) as f64) <= d * d
    }

    /// True if this circle intersects the tile shape.
    pub fn intersects_tile(&self, shape: &TileShape) -> bool {
        shape.distance(self.center.to_float()) <= self.radius as f64
    }

    /// The convex pieces of this shape: its bounding tile.
    pub fn split_to_convex(&self) -> Vec<TileShape> {
        vec![self.bounding_tile()]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basic_measures() {
        let c = Circle::new(IntPoint::new(10, 20), 5);
        assert_eq!(c.dimension(), 2);
        assert_eq!(Circle::new(IntPoint::ZERO, 0).dimension(), 0);
        assert!((c.area() - std::f64::consts::PI * 25.0).abs() < 1e-9);
        assert_eq!(c.bounding_box(), IntBox::from_coords(5, 15, 15, 25));
        assert_eq!(c.max_width(), 10.0);
        // negative radius is normalized
        assert_eq!(Circle::new(IntPoint::ZERO, -3).radius, 3);
    }

    #[test]
    fn containment_and_distances() {
        let c = Circle::new(IntPoint::new(0, 0), 10);
        assert!(c.contains(&Point::Int(IntPoint::new(6, 8)))); // on the border
        assert!(!c.contains_inside(&Point::Int(IntPoint::new(6, 8))));
        assert!(c.contains_inside(&Point::Int(IntPoint::new(3, 3))));
        assert!(c.is_outside(&Point::Int(IntPoint::new(8, 8))));
        assert!((c.distance(FloatPoint::new(0.0, 15.0)) - 5.0).abs() < 1e-9);
        assert_eq!(c.distance(FloatPoint::new(1.0, 1.0)), 0.0);
        assert!((c.border_distance(FloatPoint::new(0.0, 2.0)) - 8.0).abs() < 1e-9);
    }

    #[test]
    fn bounding_shapes_contain_circle() {
        let c = Circle::new(IntPoint::new(100, 100), 50);
        let oct = c.bounding_octagon();
        assert!(oct.is_normalized());
        // the octagon contains the circle's extreme points
        for (dx, dy) in [(50, 0), (-50, 0), (0, 50), (0, -50), (35, 35), (-35, -35)] {
            assert!(
                oct.contains(IntPoint::new(100 + dx, 100 + dy)),
                "extreme point ({dx},{dy}) outside bounding octagon"
            );
        }
        // finer tangent tile also contains the circle and is contained in
        // the bounding box slightly enlarged
        let fine = c.bounding_tile_with_segment_length(10);
        assert!(fine.border_line_count() > 8);
        for (dx, dy) in [(50, 0), (0, 50), (35, 35)] {
            assert!(fine.contains(&Point::Int(IntPoint::new(100 + dx, 100 + dy))));
        }
        assert!(fine
            .bounding_box()
            .is_contained_in(c.bounding_box().offset(2.0)));
    }

    #[test]
    fn intersections_and_transforms() {
        let a = Circle::new(IntPoint::new(0, 0), 10);
        let b = Circle::new(IntPoint::new(25, 0), 10);
        assert!(!a.intersects_circle(&b));
        let c = Circle::new(IntPoint::new(15, 0), 10);
        assert!(a.intersects_circle(&c));

        let tile = TileShape::Box(IntBox::from_coords(8, -2, 20, 2));
        assert!(a.intersects_tile(&tile));
        let far_tile = TileShape::Box(IntBox::from_coords(11, 11, 20, 20));
        assert!(!a.intersects_tile(&far_tile));

        let turned = a.translate_by(IntVector::new(5, 0)).turn_90_degree(1, IntPoint::ZERO);
        assert_eq!(turned.center, IntPoint::new(0, 5));
        assert_eq!(a.offset(2.0).radius, 12);
        assert_eq!(a.shrink(15.0).radius, 1);
        assert!(a.is_contained_in(IntBox::from_coords(-10, -10, 10, 10)));
        assert!(!a.is_contained_in(IntBox::from_coords(-9, -10, 10, 10)));
    }
}
