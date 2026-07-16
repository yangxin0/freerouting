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

#[derive(Debug, Clone)]
pub struct Simplex {
    lines: Vec<Line>,
    /// Lazily computed bounding box (corner computation is expensive and
    /// bounding boxes are queried constantly by the restrain pre-filters).
    cached_bbox: std::sync::OnceLock<IntBox>,
}

impl PartialEq for Simplex {
    fn eq(&self, other: &Self) -> bool {
        // the bbox cache is derived state and excluded
        self.lines == other.lines
    }
}

impl Simplex {
    /// Standard implementation of an empty simplex.
    #[allow(clippy::declare_interior_mutable_const)]
    pub const EMPTY: Simplex = Simplex {
        lines: Vec::new(),
        cached_bbox: std::sync::OnceLock::new(),
    };

    /// Constructs a simplex from directed lines without normalizing. Use
    /// [`Simplex::get_instance`] for a normalized simplex.
    pub fn new(lines: Vec<Line>) -> Self {
        Simplex {
            lines,
            cached_bbox: std::sync::OnceLock::new(),
        }
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
        let prev_no = if no == 0 {
            self.lines.len() - 1
        } else {
            no - 1
        };
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
        (0..self.lines.len())
            .map(|i| self.corner_approx(i))
            .collect()
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
        (0..self.lines.len()).all(|i| self.lines[i].is_orthogonal() && self.corner_is_bounded(i))
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
        *self.cached_bbox.get_or_init(|| self.compute_bounding_box())
    }

    fn compute_bounding_box(&self) -> IntBox {
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
        let sorted = |lines: &[Line]| lines.windows(2).all(|w| w[0] <= w[1]);
        let mut new_lines = Vec::with_capacity(self.lines.len() + other.lines.len());
        if sorted(&self.lines) && sorted(&other.lines) {
            // both line arrays are angular-sorted: merge instead of sort
            let (mut i, mut j) = (0, 0);
            while i < self.lines.len() && j < other.lines.len() {
                if self.lines[i] <= other.lines[j] {
                    new_lines.push(self.lines[i]);
                    i += 1;
                } else {
                    new_lines.push(other.lines[j]);
                    j += 1;
                }
            }
            new_lines.extend_from_slice(&self.lines[i..]);
            new_lines.extend_from_slice(&other.lines[j..]);
        } else {
            new_lines.extend_from_slice(&self.lines);
            new_lines.extend_from_slice(&other.lines);
            new_lines.sort();
        }
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

    /// Cuts this simplex out of `outer_simplex`. Divides the resulting
    /// shape into simplices along the minimal distance lines from the
    /// vertices of the inner simplex to the outer simplex and returns the
    /// convex pieces of this division.
    ///
    /// Only implemented for 2-dimensional simplices (like Java, which warns
    /// and returns null there; this port returns the outer simplex intact).
    // The unnecessary_literal_unwrap lint correctly proves that the
    // merge_prev branches can never fire: they are preserved deliberately
    // to mirror the Java original's dead prev_division_line code.
    #[allow(clippy::unnecessary_literal_unwrap)]
    pub fn cutout_from(&self, outer_simplex: &Simplex) -> Vec<Simplex> {
        if self.dimension() < 2 {
            return vec![outer_simplex.clone()];
        }
        let inner_simplex = self.intersection(outer_simplex);
        if inner_simplex.dimension() < 2 {
            // nothing to cut out of outer_simplex
            return vec![outer_simplex.clone()];
        }
        let inner_corner_count = inner_simplex.lines.len();
        let mut division_line_arr: Vec<Vec<Line>> = Vec::with_capacity(inner_corner_count);
        for inner_corner_no in 0..inner_corner_count {
            match inner_simplex.calc_division_lines(inner_corner_no, outer_simplex) {
                Some(lines) => division_line_arr.push(lines),
                None => return vec![outer_simplex.clone()],
            }
        }
        let mut check_cross_first_line = false;
        // Note: the Java original declares prev_division_line but only ever
        // assigns it to a dead local at the end of the loop, so it stays
        // null; the merge_prev branches below are preserved but never fire.
        let prev_division_line: Option<Line> = None;
        let first_division_line = division_line_arr[0][0];
        let first_direction = first_division_line.direction();
        let mut result_list: Vec<Simplex> = Vec::new();

        for inner_corner_no in 0..inner_corner_count {
            let next_division_line = if inner_corner_no == inner_corner_count - 1 {
                division_line_arr[0][0]
            } else {
                division_line_arr[inner_corner_no + 1][0]
            };
            let curr_division_lines = &division_line_arr[inner_corner_no];
            if curr_division_lines.len() == 2 {
                // 2 division lines are necessary (sharp corner). Construct
                // an unbounded simplex from curr_division_lines[1] and [0]
                // and intersect it with the outer simplex.
                let curr_dir = curr_division_lines[0].direction();
                let mut merge_prev_division_line = false;
                let mut merge_first_division_line = false;
                if let Some(prev_line) = prev_division_line {
                    if curr_dir.determinant(prev_line.direction()) > 0 {
                        // the previous division line may intersect
                        // curr_division_lines[0] inside the divide simplex
                        merge_prev_division_line = true;
                    }
                }
                if !check_cross_first_line {
                    check_cross_first_line =
                        inner_corner_no > 0 && curr_dir.determinant(first_direction) > 0;
                }
                if check_cross_first_line {
                    let curr_dir2 = curr_division_lines[1].direction();
                    if curr_dir2.determinant(first_direction) < 0 {
                        // The current piece has an intersection area with
                        // the first piece; add a line to prevent this.
                        merge_first_division_line = true;
                    }
                }
                let mut piece_lines =
                    vec![curr_division_lines[1].opposite(), curr_division_lines[0]];
                if merge_prev_division_line {
                    piece_lines.push(prev_division_line.unwrap());
                }
                if merge_first_division_line {
                    piece_lines.push(first_division_line.opposite());
                }
                result_list.push(Simplex::new(piece_lines).intersection(outer_simplex));
            }
            // Construct an unbounded simplex from next_division_line, the
            // inner border line and the last current division line, and
            // intersect it with the outer simplex.
            let merge_next_division_line = next_division_line.b != next_division_line.a;
            let last_curr_division_line = curr_division_lines[curr_division_lines.len() - 1];
            let last_curr_dir = last_curr_division_line.direction();
            let merge_last_curr_division_line =
                last_curr_division_line.b != last_curr_division_line.a;
            let mut merge_prev_division_line = false;
            let mut merge_first_division_line = false;
            if let Some(prev_line) = prev_division_line {
                if last_curr_dir.determinant(prev_line.direction()) > 0 {
                    // the previous division line may intersect the last
                    // current division line inside the divide simplex
                    merge_prev_division_line = true;
                }
            }
            if !check_cross_first_line {
                // scalar_product checked to ignore backcrossing at small
                // inner_corner_no
                check_cross_first_line = inner_corner_no > 0
                    && last_curr_dir.determinant(first_direction) > 0
                    && last_curr_dir
                        .get_vector()
                        .scalar_product(first_direction.get_vector())
                        < 0;
            }
            if check_cross_first_line
                && next_division_line.direction().determinant(first_direction) < 0
            {
                // The current piece has an intersection area with the first
                // piece; add a line to prevent this.
                merge_first_division_line = true;
            }
            let curr_line = inner_simplex.lines[inner_corner_no];
            let mut piece_lines = vec![curr_line.opposite()];
            if merge_next_division_line {
                piece_lines.push(next_division_line.opposite());
            }
            if merge_last_curr_division_line {
                piece_lines.push(last_curr_division_line);
            }
            if merge_prev_division_line {
                piece_lines.push(prev_division_line.unwrap());
            }
            if merge_first_division_line {
                piece_lines.push(first_division_line.opposite());
            }
            result_list.push(Simplex::new(piece_lines).intersection(outer_simplex));
        }
        result_list
    }

    /// For each corner of this inner simplex constructs 1 or 2
    /// perpendicular projections onto lines of the outer simplex, so that
    /// the resulting pieces after cutting out the inner simplex are convex.
    /// 2 projections may be necessary at sharp corners.
    fn calc_division_lines(
        &self,
        inner_corner_no: usize,
        outer_simplex: &Simplex,
    ) -> Option<Vec<Line>> {
        let curr_inner_line = self.lines[inner_corner_no];
        let prev_inner_line = if inner_corner_no != 0 {
            self.lines[inner_corner_no - 1]
        } else {
            self.lines[self.lines.len() - 1]
        };
        let intersection = curr_inner_line.intersection_approx(&prev_inner_line);
        if intersection.x >= i32::MAX as f64 {
            // intersection expected
            return None;
        }
        let inner_corner = intersection.round();
        let c_tolerance = 0.0001;
        let is_exact = (inner_corner.x as f64 - intersection.x).abs() < c_tolerance
            && (inner_corner.y as f64 - intersection.y).abs() < c_tolerance;
        if !is_exact {
            // Assumed to be a corner from intersecting the inner simplex
            // with the outer simplex; it lies on the outer border, so no
            // division is necessary.
            return Some(vec![prev_inner_line]);
        }
        let mut first_projection_dir = IntDirection::NULL;
        let mut second_projection_dir = IntDirection::NULL;
        let prev_inner_dir = prev_inner_line.direction().opposite();
        let next_inner_dir = curr_inner_line.direction();
        let mut outer_line_no = 0;

        // Search the first outer line so that the perpendicular projection
        // of the inner corner onto this line is visible from the inner
        // corner to the left of prev_inner_line.
        let mut min_distance = f64::from(i32::MAX);

        for _ in 0..outer_simplex.lines.len() {
            let outer_line = outer_simplex.lines[outer_line_no];
            let Some(curr_projection_dir) = outer_line.perpendicular_direction(inner_corner) else {
                // inner corner is on the outer line
                return Some(vec![Line::new(inner_corner, inner_corner)]);
            };
            let projection_visible = prev_inner_dir.determinant(curr_projection_dir) >= 0;
            if projection_visible {
                let mut curr_distance = outer_line.signed_distance(inner_corner.to_float()).abs();
                // A second division may be necessary at a sharp corner.
                let second_division_necessary = curr_projection_dir.determinant(next_inner_dir) < 0;
                let mut curr_second_projection_dir = curr_projection_dir;
                if second_division_necessary {
                    // Search the first projection dir between
                    // curr_projection_dir and next_inner_dir that is
                    // visible from the next inner line.
                    let mut second_projection_visible = false;
                    let mut tmp_outer_line_no = outer_line_no;
                    while !second_projection_visible {
                        tmp_outer_line_no = (tmp_outer_line_no + 1) % outer_simplex.lines.len();
                        let Some(dir) = outer_simplex.lines[tmp_outer_line_no]
                            .perpendicular_direction(inner_corner)
                        else {
                            // inner corner is on the outer line
                            return Some(vec![Line::new(inner_corner, inner_corner)]);
                        };
                        curr_second_projection_dir = dir;
                        if curr_projection_dir.determinant(curr_second_projection_dir) < 0 {
                            // Not found: the angle between the projections
                            // would already exceed 180 degree.
                            curr_distance = f64::from(i32::MAX);
                            break;
                        }
                        second_projection_visible =
                            curr_second_projection_dir.determinant(next_inner_dir) >= 0;
                    }
                    curr_distance += outer_simplex.lines[tmp_outer_line_no]
                        .signed_distance(inner_corner.to_float())
                        .abs();
                }
                if curr_distance < min_distance {
                    min_distance = curr_distance;
                    first_projection_dir = curr_projection_dir;
                    second_projection_dir = curr_second_projection_dir;
                }
            }
            outer_line_no = (outer_line_no + 1) % outer_simplex.lines.len();
        }
        if min_distance == f64::from(i32::MAX) {
            // division not found
            return None;
        }
        if first_projection_dir == second_projection_dir {
            Some(vec![Line::from_direction(
                inner_corner,
                first_projection_dir,
            )])
        } else {
            Some(vec![
                Line::from_direction(inner_corner, first_projection_dir),
                Line::from_direction(inner_corner, second_projection_dir),
            ])
        }
    }

    /// Removes lines which are redundant for the shape of this simplex.
    /// Assumes the lines are sorted in ascending direction. Returns
    /// [`Simplex::EMPTY`] if the half planes have an empty intersection.
    pub fn remove_redundant_lines(&self) -> Simplex {
        if self.lines.is_empty() {
            return Simplex::EMPTY;
        }
        // Thread-local scratch: this runs millions of times per routing
        // pass and the two working vectors dominated the allocator
        // traffic (~14% of a coldfire profile).
        thread_local! {
            static SCRATCH: std::cell::RefCell<(Vec<Line>, Vec<Option<Side>>)> =
                const { std::cell::RefCell::new((Vec::new(), Vec::new())) };
        }
        SCRATCH.with(|scratch| {
            let mut scratch = scratch.borrow_mut();
            let (line_arr, intersection_sides) = &mut *scratch;
            // Copy the sorted lines, skipping duplicates (equal line and
            // direction).
            line_arr.clear();
            line_arr.push(self.lines[0]);
            for line in &self.lines[1..] {
                if *line != *line_arr.last().unwrap() {
                    line_arr.push(*line);
                }
            }
            let mut new_length = line_arr.len();
            // On which side of line `ind` the previous and next lines intersect.
            intersection_sides.clear();
            intersection_sides.resize(new_length, None);

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
                    let det = prev_line.direction_determinant_sign(&next_line);
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
                        } else if intersection_sides[uind] == Some(Side::OnTheLeft)
                            && prev_line.direction_determinant_sign(&curr_line) > 0
                        {
                            // The half plane of curr_line does not intersect
                            // the simplex of prev_line and next_line: empty.
                            new_length = 0;
                            try_again = false;
                            break;
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
            Simplex::new(line_arr[..new_length].to_vec())
        })
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
        let empty = Simplex::EMPTY;
        assert!(empty.is_outside(&Point::Int(IntPoint::new(0, 0))));
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

    /// Shoelace area over the float corners of a bounded simplex.
    fn simplex_area(s: &Simplex) -> f64 {
        if s.is_empty() || !s.is_bounded() {
            return 0.0;
        }
        let corners = s.corner_approx_arr();
        let n = corners.len();
        let mut sum = 0.0;
        for i in 0..n {
            let a = corners[i];
            let b = corners[(i + 1) % n];
            sum += a.x * b.y - b.x * a.y;
        }
        0.5 * sum.abs()
    }

    #[test]
    fn cutout_square_hole() {
        let outer = unit_square(20);
        let hole = unit_square(4).translate_by(IntVector::new(8, 8));
        let pieces = hole.cutout_from(&outer);
        assert!(!pieces.is_empty());
        let pieces_area: f64 = pieces.iter().map(simplex_area).sum();
        let expected = simplex_area(&outer) - simplex_area(&hole);
        assert!(
            (pieces_area - expected).abs() < 1e-6,
            "pieces_area {pieces_area} != expected {expected}"
        );
        for (i, piece) in pieces.iter().enumerate() {
            assert!(
                piece.is_empty() || piece.is_bounded(),
                "piece {i} unbounded"
            );
            // No piece may reach the interior of the hole.
            assert!(
                !piece.contains_inside(&Point::Int(IntPoint::new(10, 10))),
                "piece {i} covers the hole"
            );
        }
    }

    #[test]
    fn cutout_triangle_hole() {
        let outer = unit_square(20);
        // Triangle with a sharp corner: (2,2), (10,2), (2,8).
        let triangle = Simplex::get_instance(vec![
            Line::from_coords(2, 2, 10, 2),
            Line::from_coords(10, 2, 2, 8),
            Line::from_coords(2, 8, 2, 2),
        ]);
        assert_eq!(triangle.dimension(), 2);
        let pieces = triangle.cutout_from(&outer);
        let pieces_area: f64 = pieces.iter().map(simplex_area).sum();
        let expected = simplex_area(&outer) - simplex_area(&triangle);
        assert!(
            (pieces_area - expected).abs() < 1e-6,
            "pieces_area {pieces_area} != expected {expected}"
        );
        // Non-overlapping hole: outer returned unchanged.
        let far_hole = unit_square(2).translate_by(IntVector::new(100, 100));
        assert_eq!(far_hole.cutout_from(&outer), vec![outer]);
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
