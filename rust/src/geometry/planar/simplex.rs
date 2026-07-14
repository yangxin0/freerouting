//! Port of `geometry/planar/Simplex.java` (core), plus the generic
//! point-containment logic it inherits from `TileShape.java`.
//!
//! A convex shape defined as the intersection of half planes, each the
//! point-left side of a directed line. The border lines are kept sorted in
//! ascending direction; corner `i` is the intersection of line `i-1` with
//! line `i`.
//!
//! `cutout_from` / `calc_division_lines` follow with the `TileShape` enum.

use crate::geometry::planar::{
    limits, FloatPoint, IntBox, IntDirection, IntOctagon, IntVector, Line, Point, Side,
};

#[derive(Debug, Clone, PartialEq)]
pub struct Simplex {
    lines: Vec<Line>,
}

impl Simplex {
    /// Standard implementation of an empty simplex.
    pub const EMPTY: Simplex = Simplex { lines: Vec::new() };

    /// Constructs a simplex from directed lines without normalizing. Use
    /// [`Simplex::get_instance`] for a normalized simplex.
    pub fn new(lines: Vec<Line>) -> Self {
        Simplex { lines }
    }

    /// Creates a normalized simplex as the intersection of the half planes
    /// defined by `lines`.
    pub fn get_instance(mut lines: Vec<Line>) -> Self {
        if lines.is_empty() {
            return Simplex::EMPTY;
        }
        lines.sort();
        Simplex::new(lines).remove_redundant_lines()
    }

    pub fn is_empty(&self) -> bool {
        self.lines.is_empty()
    }

    pub fn border_line_count(&self) -> usize {
        self.lines.len()
    }

    /// The `no`-th border line; the lines are sorted in ascending direction.
    pub fn border_line(&self, no: usize) -> Line {
        self.lines[no]
    }

    pub fn border_lines(&self) -> &[Line] {
        &self.lines
    }

    /// True if the directions of lines `no - 1` and `no` turn left at the
    /// corner, i.e. the corner is a real (bounded) vertex.
    pub fn corner_is_bounded(&self, no: usize) -> bool {
        if self.lines.len() == 1 {
            return false;
        }
        let prev_no = if no == 0 { self.lines.len() - 1 } else { no - 1 };
        let prev_dir = self.lines[prev_no].direction();
        let curr_dir = self.lines[no].direction();
        prev_dir.determinant(curr_dir) > 0
    }

    /// True if this simplex is contained in a sufficiently large box.
    pub fn is_bounded(&self) -> bool {
        if self.lines.is_empty() {
            return true;
        }
        if self.lines.len() < 3 {
            return false;
        }
        (0..self.lines.len()).all(|i| self.corner_is_bounded(i))
    }

    /// The exact intersection of lines `no - 1` and `no`; infinite if the
    /// simplex is unbounded at this corner.
    pub fn corner(&self, no: usize) -> Point {
        let prev = if no == 0 {
            self.lines[self.lines.len() - 1]
        } else {
            self.lines[no - 1]
        };
        self.lines[no].intersection(&prev)
    }

    /// An approximation of corner `no`; coordinates are `i32::MAX` if the
    /// simplex is unbounded there.
    pub fn corner_approx(&self, no: usize) -> FloatPoint {
        let prev = if no == 0 {
            self.lines[self.lines.len() - 1]
        } else {
            self.lines[no - 1]
        };
        self.lines[no].intersection_approx(&prev)
    }

    pub fn corner_approx_arr(&self) -> Vec<FloatPoint> {
        (0..self.lines.len()).map(|i| self.corner_approx(i)).collect()
    }

    /// The dimension of this simplex: 2, 1, 0 or -1 (empty). Assumes the
    /// simplex is normalized.
    pub fn dimension(&self) -> i32 {
        match self.lines.len() {
            0 => -1,
            1 => 2, // a half plane
            2 => {
                if self.lines[0].overlaps(&self.lines[1]) {
                    1
                } else {
                    2
                }
            }
            3 => {
                if self.lines[0].overlaps(&self.lines[1])
                    || self.lines[0].overlaps(&self.lines[2])
                    || self.lines[1].overlaps(&self.lines[2])
                {
                    // 1 dimensional and unbounded at one side
                    return 1;
                }
                let intersection = self.lines[1].intersection(&self.lines[2]);
                match self.lines[0].side_of(&intersection) {
                    Side::OnTheRight => 2,
                    Side::OnTheLeft => -1, // empty simplex, not normalized
                    Side::Collinear => 0,  // all 3 lines meet in one point
                }
            }
            4 => {
                let collinear_0_2 = self.lines[0].overlaps(&self.lines[2]);
                let collinear_1_3 = self.lines[1].overlaps(&self.lines[3]);
                if collinear_0_2 && collinear_1_3 {
                    0
                } else if collinear_0_2 || collinear_1_3 {
                    1
                } else {
                    2
                }
            }
            _ => 2,
        }
    }

    /// The centre of gravity: the mean of the corner approximations.
    pub fn centre_of_gravity(&self) -> FloatPoint {
        let corners = self.corner_approx_arr();
        let n = corners.len() as f64;
        let (mut x, mut y) = (0.0, 0.0);
        for c in &corners {
            x += c.x;
            y += c.y;
        }
        FloatPoint::new(x / n, y / n)
    }

    pub fn max_width(&self) -> f64 {
        self.width_by(|a, b| a > b)
    }

    pub fn min_width(&self) -> f64 {
        self.width_by(|a, b| a < b)
    }

    /// Sum of the two extreme distances of border lines from the gravity
    /// point (Java: max_width / min_width).
    fn width_by(&self, better: impl Fn(f64, f64) -> bool) -> f64 {
        if !self.is_bounded() {
            return i32::MAX as f64;
        }
        let gravity_point = self.centre_of_gravity();
        let mut best = if better(1.0, 0.0) { f64::MIN } else { f64::MAX };
        let mut best_2 = best;
        for line in &self.lines {
            let curr = line.signed_distance(gravity_point).abs();
            if better(curr, best) {
                best_2 = best;
                best = curr;
            } else if better(curr, best_2) {
                best_2 = curr;
            }
        }
        best + best_2
    }

    /// True if `point` is outside this simplex.
    pub fn is_outside(&self, point: &Point) -> bool {
        if self.lines.is_empty() {
            return true;
        }
        self.lines
            .iter()
            .any(|line| line.side_of(point) == Side::OnTheLeft)
    }

    /// True if `point` is contained in this simplex (border included).
    pub fn contains(&self, point: &Point) -> bool {
        !self.is_outside(point)
    }

    /// True if `point` is contained in this simplex but not on the border.
    pub fn contains_inside(&self, point: &Point) -> bool {
        if self.lines.is_empty() {
            return false;
        }
        self.lines
            .iter()
            .all(|line| line.side_of(point) == Side::OnTheRight)
    }

    /// True if `point` is contained with `tolerance` (in determinant units,
    /// like `Line::side_of_float`).
    pub fn contains_float(&self, point: FloatPoint, tolerance: f64) -> bool {
        if self.lines.is_empty() {
            return false;
        }
        self.lines
            .iter()
            .all(|line| line.side_of_float(point, tolerance) == Side::OnTheRight)
    }

    /// True if all border lines are orthogonal and all corners bounded, so
    /// the simplex describes an [`IntBox`].
    pub fn is_int_box(&self) -> bool {
        (0..self.lines.len())
            .all(|i| self.lines[i].is_orthogonal() && self.corner_is_bounded(i))
    }

    /// True if all border lines are multiples of 45 degree and all corners
    /// bounded, so the simplex describes an [`IntOctagon`].
    pub fn is_int_octagon(&self) -> bool {
        (0..self.lines.len())
            .all(|i| self.lines[i].is_multiple_of_45_degree() && self.corner_is_bounded(i))
    }

    /// Converts this simplex to an [`IntOctagon`]; `None` if not all border
    /// lines are 45-degree.
    pub fn to_int_octagon(&self) -> Option<IntOctagon> {
        if !self.is_int_octagon() {
            return None;
        }
        if self.is_empty() {
            return Some(IntOctagon::EMPTY);
        }
        // initialise to the biggest octagon values
        let mut rx = limits::CRIT_INT;
        let mut uy = limits::CRIT_INT;
        let mut lrx = limits::CRIT_INT;
        let mut urx = limits::CRIT_INT;
        let mut lx = -limits::CRIT_INT;
        let mut ly = -limits::CRIT_INT;
        let mut llx = -limits::CRIT_INT;
        let mut ulx = -limits::CRIT_INT;
        for line in &self.lines {
            let (a, b) = (line.a, line.b);
            if a.y == b.y {
                if b.x >= a.x {
                    ly = a.y; // lower boundary line
                }
                if b.x <= a.x {
                    uy = a.y; // upper boundary line
                }
            }
            if a.x == b.x {
                if b.y >= a.y {
                    rx = a.x; // right boundary line
                }
                if b.y <= a.y {
                    lx = a.x; // left boundary line
                }
            }
            if a.y < b.y {
                if a.x < b.x {
                    lrx = a.x - a.y; // lower right boundary line
                } else if a.x > b.x {
                    urx = a.x + a.y; // upper right boundary line
                }
            } else if a.y > b.y {
                if a.x < b.x {
                    llx = a.x + a.y; // lower left boundary line
                } else if a.x > b.x {
                    ulx = a.x - a.y; // upper left boundary line
                }
            }
        }
        Some(IntOctagon::new(lx, ly, rx, uy, ulx, lrx, llx, urx).normalize())
    }

    /// The simplex resulting from translating all border lines by `vector`.
    pub fn translate_by(&self, vector: IntVector) -> Self {
        if vector.is_zero() {
            return self.clone();
        }
        Simplex::new(
            self.lines
                .iter()
                .map(|line| line.translate_by(vector))
                .collect(),
        )
    }

    /// The smallest integer box containing all corners; coordinates are
    /// huge if the simplex is unbounded.
    pub fn bounding_box(&self) -> IntBox {
        if self.lines.is_empty() {
            return IntBox::EMPTY;
        }
        let (mut llx, mut lly) = (f64::MAX, f64::MAX);
        let (mut urx, mut ury) = (f64::MIN, f64::MIN);
        for i in 0..self.lines.len() {
            let curr = self.corner_approx(i);
            llx = llx.min(curr.x);
            lly = lly.min(curr.y);
            urx = urx.max(curr.x);
            ury = ury.max(curr.y);
        }
        IntBox::from_coords(
            llx.floor() as i32,
            lly.floor() as i32,
            urx.ceil() as i32,
            ury.ceil() as i32,
        )
    }

    /// A bounding octagon of the simplex; `None` if the simplex is not
    /// bounded.
    pub fn bounding_octagon(&self) -> Option<IntOctagon> {
        let (mut lx, mut ly) = (f64::MAX, f64::MAX);
        let (mut rx, mut uy) = (f64::MIN, f64::MIN);
        let (mut ulx, mut llx) = (f64::MAX, f64::MAX);
        let (mut lrx, mut urx) = (f64::MIN, f64::MIN);
        for i in 0..self.lines.len() {
            let curr = self.corner_approx(i);
            lx = lx.min(curr.x);
            ly = ly.min(curr.y);
            rx = rx.max(curr.x);
            uy = uy.max(curr.y);
            let tmp = curr.x - curr.y;
            ulx = ulx.min(tmp);
            lrx = lrx.max(tmp);
            let tmp = curr.x + curr.y;
            llx = llx.min(tmp);
            urx = urx.max(tmp);
        }
        let crit = limits::CRIT_INT as f64;
        if lx.min(ly) < -crit || rx.max(uy) > crit || ulx.min(llx) < -crit || lrx.max(urx) > crit {
            return None;
        }
        Some(IntOctagon::new(
            lx.floor() as i32,
            ly.floor() as i32,
            rx.ceil() as i32,
            uy.ceil() as i32,
            ulx.floor() as i32,
            lrx.ceil() as i32,
            llx.floor() as i32,
            urx.ceil() as i32,
        ))
    }

    /// The simplex offset by `width`: outward if positive, inward if
    /// negative.
    pub fn offset(&self, width: f64) -> Self {
        if width == 0.0 {
            return self.clone();
        }
        let result = Simplex::new(
            self.lines
                .iter()
                .map(|line| line.translate(-width))
                .collect(),
        );
        if width < 0.0 {
            result.remove_redundant_lines()
        } else {
            result
        }
    }

    /// This simplex enlarged by `offset`, intersected with the enlarged
    /// bounding octagon (to keep the result bounded).
    pub fn enlarge(&self, offset: f64) -> Self {
        if offset == 0.0 {
            return self.clone();
        }
        let offset_simplex = self.offset(offset);
        let Some(bounding_oct) = self.bounding_octagon() else {
            return Simplex::EMPTY;
        };
        offset_simplex.intersection(&bounding_oct.offset(offset).to_simplex())
    }

    /// The number of the rightmost corner seen from `from_point`.
    pub fn index_of_right_most_corner(&self, from_point: &Point) -> usize {
        let mut right_most_corner = self.corner(0);
        let mut result = 0;
        for i in 1..self.lines.len() {
            let curr_corner = self.corner(i);
            if curr_corner.side_of(from_point, &right_most_corner) == Side::OnTheRight {
                right_most_corner = curr_corner;
                result = i;
            }
        }
        result
    }

    /// The intersection of this simplex with `other`.
    pub fn intersection(&self, other: &Simplex) -> Simplex {
        if self.is_empty() || other.is_empty() {
            return Simplex::EMPTY;
        }
        let mut new_lines = Vec::with_capacity(self.lines.len() + other.lines.len());
        new_lines.extend_from_slice(&self.lines);
        new_lines.extend_from_slice(&other.lines);
        new_lines.sort();
        Simplex::new(new_lines).remove_redundant_lines()
    }

    pub fn intersects(&self, other: &Simplex) -> bool {
        !self.intersection(other).is_empty()
    }

    /// The index of `line` among the border lines, if present.
    pub fn border_line_index(&self, line: &Line) -> Option<usize> {
        self.lines.iter().position(|l| l == line)
    }

    /// Enlarges the simplex by removing the border line `no`; the result
    /// may become unbounded.
    pub fn remove_border_line(&self, no: usize) -> Self {
        if no >= self.lines.len() {
            return self.clone();
        }
        let mut new_lines = self.lines.clone();
        new_lines.remove(no);
        Simplex::new(new_lines)
    }

    /// Removes lines which are redundant for the shape of this simplex.
    /// Assumes the lines are sorted in ascending direction. Returns
    /// [`Simplex::EMPTY`] if the half planes have an empty intersection.
    pub fn remove_redundant_lines(&self) -> Simplex {
        if self.lines.is_empty() {
            return Simplex::EMPTY;
        }
        // Copy the sorted lines, skipping duplicates (equal line and
        // direction).
        let mut line_arr: Vec<Line> = Vec::with_capacity(self.lines.len());
        line_arr.push(self.lines[0]);
        for line in &self.lines[1..] {
            if *line != *line_arr.last().unwrap() {
                line_arr.push(*line);
            }
        }
        let mut new_length = line_arr.len();
        // On which side of line `ind` the previous and next lines intersect.
        let mut intersection_sides: Vec<Option<Side>> = vec![None; new_length];

        let mut try_again = new_length > 2;
        let mut index_of_last_removed_line = new_length as isize;
        while try_again {
            try_again = false;
            let mut prev_ind = new_length - 1;
            let mut prev_line = line_arr[prev_ind];
            let mut curr_line = line_arr[0];
            let mut ind: isize = 0;
            while (ind as usize) < new_length {
                let uind = ind as usize;
                let next_ind = if uind == new_length - 1 { 0 } else { uind + 1 };
                let next_line = line_arr[next_ind];

                let mut remove_line = false;
                let prev_dir = prev_line.direction();
                let next_dir = next_line.direction();
                let det = prev_dir.determinant(next_dir);
                if det != 0 {
                    // prev_line and next_line are not parallel
                    if intersection_sides[uind].is_none() {
                        intersection_sides[uind] =
                            Some(curr_line.side_of_intersection(&prev_line, &next_line));
                    }
                    if det > 0 {
                        // If the intersection of prev_line and next_line is
                        // on the left of curr_line, curr_line does not
                        // contribute to the shape of the simplex.
                        remove_line = intersection_sides[uind] != Some(Side::OnTheLeft);
                    } else if intersection_sides[uind] == Some(Side::OnTheLeft) {
                        let curr_dir = curr_line.direction();
                        if prev_dir.determinant(curr_dir) > 0 {
                            // The half plane of curr_line does not intersect
                            // the simplex of prev_line and next_line: empty.
                            new_length = 0;
                            try_again = false;
                            break;
                        }
                    }
                } else {
                    // prev_line and next_line are parallel
                    if prev_line.side_of_int(next_line.a) == Side::OnTheLeft {
                        // Their half planes do not intersect.
                        new_length = 0;
                        try_again = false;
                        break;
                    }
                }
                if remove_line {
                    try_again = true;
                    new_length -= 1;
                    for i in uind..new_length {
                        line_arr[i] = line_arr[i + 1];
                        intersection_sides[i] = intersection_sides[i + 1];
                    }
                    if new_length < 3 {
                        try_again = false;
                        break;
                    }
                    // Reset the precalculated sides around the removal.
                    if uind == 0 {
                        prev_ind = new_length - 1;
                    }
                    intersection_sides[prev_ind] = None;
                    let reset_ind = if uind >= new_length { 0 } else { uind };
                    intersection_sides[reset_ind] = None;
                    ind -= 1;
                    index_of_last_removed_line = ind;
                } else {
                    prev_line = curr_line;
                    prev_ind = uind;
                }
                curr_line = next_line;
                if !try_again && ind >= index_of_last_removed_line {
                    // tried all lines without removing one
                    break;
                }
                ind += 1;
            }
        }

        if new_length == 2 && line_arr[0].is_parallel(&line_arr[1]) {
            if line_arr[0].direction() == line_arr[1].direction() {
                // one of the two remaining lines is redundant
                if line_arr[1].side_of_int(line_arr[0].a) == Side::OnTheLeft {
                    line_arr[0] = line_arr[1];
                }
                new_length -= 1;
            } else {
                // opposite directions: the simplex may be empty
                if line_arr[1].side_of_int(line_arr[0].a) == Side::OnTheLeft {
                    new_length = 0;
                }
            }
        }
        if new_length == self.lines.len() {
            return self.clone(); // nothing removed
        }
        if new_length == 0 {
            return Simplex::EMPTY;
        }
        line_arr.truncate(new_length);
        Simplex::new(line_arr)
    }
}

impl IntBox {
    /// The [`Simplex`] defining the same shape.
    pub fn to_simplex(&self) -> Simplex {
        if self.is_empty() {
            return Simplex::EMPTY;
        }
        Simplex::new(vec![
            Line::from_direction(self.ll, IntDirection::RIGHT),
            Line::from_direction(self.ur, IntDirection::UP),
            Line::from_direction(self.ur, IntDirection::LEFT),
            Line::from_direction(self.ll, IntDirection::DOWN),
        ])
    }
}

impl IntOctagon {
    /// The [`Simplex`] defining the same shape (redundant border lines
    /// removed).
    pub fn to_simplex(&self) -> Simplex {
        if self.is_empty() {
            return Simplex::EMPTY;
        }
        let lines: Vec<Line> = (0..8).map(|i| self.border_line(i)).collect();
        Simplex::new(lines).remove_redundant_lines()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::planar::IntPoint;

    fn unit_square(size: i32) -> Simplex {
        IntBox::from_coords(0, 0, size, size).to_simplex()
    }

    #[test]
    fn box_simplex_roundtrip() {
        let s = unit_square(10);
        assert_eq!(s.border_line_count(), 4);
        assert!(s.is_bounded());
        assert!(s.is_int_box());
        assert!(s.is_int_octagon());
        assert_eq!(s.dimension(), 2);
        assert_eq!(s.bounding_box(), IntBox::from_coords(0, 0, 10, 10));
        let oct = s.to_int_octagon().unwrap();
        assert_eq!(oct.bounding_box(), IntBox::from_coords(0, 0, 10, 10));
        assert!(oct.is_int_box());
        // Corners: intersection of consecutive border lines.
        let corners: Vec<FloatPoint> = s.corner_approx_arr();
        assert_eq!(corners.len(), 4);
        let g = s.centre_of_gravity();
        assert!((g.x - 5.0).abs() < 1e-12 && (g.y - 5.0).abs() < 1e-12);
        assert!((s.max_width() - 10.0).abs() < 1e-9);
        assert!((s.min_width() - 10.0).abs() < 1e-9);
    }

    #[test]
    fn octagon_simplex_roundtrip() {
        let d = IntOctagon::new(-4, -4, 4, 4, -4, 4, -4, 4).normalize();
        let s = d.to_simplex();
        // The diamond has only 4 relevant border lines.
        assert_eq!(s.border_line_count(), 4);
        assert!(s.is_int_octagon());
        assert!(!s.is_int_box());
        assert_eq!(s.to_int_octagon().unwrap(), d);
    }

    #[test]
    fn containment() {
        let s = unit_square(10);
        assert!(s.contains(&Point::Int(IntPoint::new(5, 5))));
        assert!(s.contains(&Point::Int(IntPoint::new(0, 0))));
        assert!(!s.contains_inside(&Point::Int(IntPoint::new(0, 0))));
        assert!(s.contains_inside(&Point::Int(IntPoint::new(1, 1))));
        assert!(s.is_outside(&Point::Int(IntPoint::new(11, 5))));
        assert!(s.contains_float(FloatPoint::new(5.0, 5.0), 0.0));
        assert!(!s.contains_float(FloatPoint::new(-0.5, 5.0), 0.0));
        assert!(Simplex::EMPTY.is_outside(&Point::Int(IntPoint::new(0, 0))));
    }

    #[test]
    fn remove_redundant_lines() {
        // A square plus a line far outside: the extra line is redundant.
        let mut lines = unit_square(10).border_lines().to_vec();
        lines.push(Line::from_direction(
            IntPoint::new(0, 100),
            IntDirection::LEFT,
        ));
        let s = Simplex::get_instance(lines);
        assert_eq!(s.border_line_count(), 4);
        assert_eq!(s.bounding_box(), IntBox::from_coords(0, 0, 10, 10));

        // Two half planes with empty intersection.
        let s = Simplex::get_instance(vec![
            Line::from_direction(IntPoint::new(0, 0), IntDirection::RIGHT),
            Line::from_direction(IntPoint::new(0, 10), IntDirection::LEFT),
        ]);
        assert!(!s.is_empty()); // a 10-wide horizontal strip
        let s = Simplex::get_instance(vec![
            Line::from_direction(IntPoint::new(0, 10), IntDirection::RIGHT),
            Line::from_direction(IntPoint::new(0, 0), IntDirection::LEFT),
        ]);
        assert!(s.is_empty()); // half planes point away from each other

        // Same direction twice: one is redundant.
        let s = Simplex::get_instance(vec![
            Line::from_direction(IntPoint::new(0, 0), IntDirection::RIGHT),
            Line::from_direction(IntPoint::new(0, 5), IntDirection::RIGHT),
        ]);
        assert_eq!(s.border_line_count(), 1);
        // The remaining half plane is the more restrictive one (y >= 5).
        assert!(s.contains(&Point::Int(IntPoint::new(0, 7))));
        assert!(s.is_outside(&Point::Int(IntPoint::new(0, 2))));
    }

    #[test]
    fn intersection_and_intersects() {
        let a = unit_square(10);
        let b = a.translate_by(IntVector::new(5, 5));
        let is = a.intersection(&b);
        assert!(!is.is_empty());
        assert_eq!(is.bounding_box(), IntBox::from_coords(5, 5, 10, 10));
        assert!(a.intersects(&b));
        let far = a.translate_by(IntVector::new(100, 0));
        assert!(!a.intersects(&far));
        // A triangle: cut the square with a diagonal half plane whose
        // point-left side is the lower-left of the x + y = 10 line.
        let triangle = a.intersection(&Simplex::new(vec![Line::from_coords(10, 0, 0, 10)]));
        assert_eq!(triangle.dimension(), 2);
        assert!(triangle.contains(&Point::Int(IntPoint::new(2, 2))));
        assert!(triangle.is_outside(&Point::Int(IntPoint::new(9, 9))));
        assert!(!triangle.is_int_box());
    }

    #[test]
    fn offset_and_enlarge() {
        let s = unit_square(10);
        let grown = s.offset(2.0);
        assert_eq!(grown.bounding_box(), IntBox::from_coords(-2, -2, 12, 12));
        let shrunk = s.offset(-2.0);
        assert_eq!(shrunk.bounding_box(), IntBox::from_coords(2, 2, 8, 8));
        let enlarged = s.enlarge(2.0);
        assert!(enlarged.contains(&Point::Int(IntPoint::new(-1, 5))));
        assert!(enlarged.is_bounded());
    }

    #[test]
    fn half_plane_properties() {
        let half = Simplex::new(vec![Line::from_coords(0, 0, 1, 0)]);
        assert_eq!(half.dimension(), 2);
        assert!(!half.is_bounded());
        assert!(!half.corner_is_bounded(0));
        // Corner of a half plane is at infinity.
        let c = half.corner_approx(0);
        assert!(c.x >= i32::MAX as f64);
        assert!(half.bounding_octagon().is_none());
    }

    #[test]
    fn right_most_corner() {
        let s = unit_square(10);
        let from = Point::Int(IntPoint::new(-5, 5));
        let idx = s.index_of_right_most_corner(&from);
        // Seen from the left, the rightmost corner (clockwise-most) is the
        // lower one among the visible corners.
        let corner = s.corner(idx);
        for i in 0..s.border_line_count() {
            assert_ne!(
                s.corner(i).side_of(&from, &corner),
                Side::OnTheRight,
                "corner {i} is right of the reported rightmost corner"
            );
        }
    }
}
