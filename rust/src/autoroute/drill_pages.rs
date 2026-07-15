//! Port of `DrillPageArray.java` / `DrillPage.java` / `ExpansionDrill`:
//! the board is divided into pages; each page caches the convex free
//! areas where a via can be drilled (page shape minus the undrillable
//! item shapes, split to convex pieces) and their center locations. The
//! cache is per net and invalidated by board changes, like Java.

use crate::autoroute::room_completion::{restrain_all, IncompleteRoom};
use crate::board::basic_board::BasicBoard;
use crate::geometry::planar::{IntBox, IntPoint, TileShape};
use std::rc::Rc;

/// A possible via location (Java: `ExpansionDrill`).
#[derive(Debug, Clone)]
pub struct ExpansionDrill {
    /// The convex free area the drill was derived from.
    pub shape: TileShape,
    pub location: IntPoint,
}

#[derive(Debug)]
struct DrillPage {
    shape: IntBox,
    /// Cached drills, valid for `net_no` (Java caches per net).
    drills: Option<Vec<ExpansionDrill>>,
    net_no: i32,
}

/// The array of drill pages covering the board
/// (Java: `DrillPageArray`).
#[derive(Debug)]
pub struct DrillPageArray {
    page_width: i32,
    bounding: IntBox,
    pages: std::collections::HashMap<(i32, i32), DrillPage>,
    /// Consumed prefix of the board change log (invalidation).
    seen_log: usize,
    seen_epoch: u64,
}

impl DrillPageArray {
    /// Page width like Java: `max(5 × via diameter, 10000)`.
    pub fn new(board: &BasicBoard, via_padstack: usize) -> Self {
        let via_extent = board
            .padstacks
            .get_by_no(via_padstack)
            .and_then(|p| p.get_shape(p.from_layer()))
            .map(|s| s.bounding_box().max_width() as i32)
            .unwrap_or(2000);
        DrillPageArray {
            page_width: (5 * via_extent).max(10_000),
            bounding: board.bounding_box(),
            pages: std::collections::HashMap::new(),
            seen_log: board.change_log().len(),
            seen_epoch: board.change_epoch(),
        }
    }

    /// Invalidates the pages touched by board changes since the last
    /// call (Java: `invalidate_drill_pages` from
    /// `additional_update_after_change`).
    pub fn sync_board_changes(&mut self, board: &BasicBoard) {
        if board.change_epoch() != self.seen_epoch {
            self.pages.clear();
            self.seen_epoch = board.change_epoch();
            self.seen_log = board.change_log().len();
            return;
        }
        let log = board.change_log();
        for (_, bbox) in &log[self.seen_log.min(log.len())..] {
            let grown = bbox.offset(self.page_width as f64);
            let (x0, x1) = (
                grown.ll.x.div_euclid(self.page_width),
                grown.ur.x.div_euclid(self.page_width),
            );
            let (y0, y1) = (
                grown.ll.y.div_euclid(self.page_width),
                grown.ur.y.div_euclid(self.page_width),
            );
            for x in x0..=x1 {
                for y in y0..=y1 {
                    if let Some(page) = self.pages.get_mut(&(x, y)) {
                        page.drills = None;
                    }
                }
            }
        }
        self.seen_log = log.len();
    }

    /// The drills of every page overlapping `area`, computed or served
    /// from the page caches (Java: `DrillPageArray.overlapping_pages` +
    /// `DrillPage.get_drills`).
    pub fn drills_overlapping(
        &mut self,
        board: &BasicBoard,
        area: &IntBox,
        net_no: i32,
        via_margin: i32,
    ) -> Vec<ExpansionDrill> {
        let (x0, x1) = (
            area.ll.x.div_euclid(self.page_width),
            area.ur.x.div_euclid(self.page_width),
        );
        let (y0, y1) = (
            area.ll.y.div_euclid(self.page_width),
            area.ur.y.div_euclid(self.page_width),
        );
        let mut result = Vec::new();
        for x in x0..=x1 {
            for y in y0..=y1 {
                let key = (x, y);
                let page_box = IntBox::from_coords(
                    x * self.page_width,
                    y * self.page_width,
                    (x + 1) * self.page_width,
                    (y + 1) * self.page_width,
                )
                .intersection(self.bounding);
                if page_box.is_empty() {
                    continue;
                }
                let page = self
                    .pages
                    .entry(key)
                    .or_insert_with(|| DrillPage {
                        shape: page_box,
                        drills: None,
                        net_no: -1,
                    });
                if page.drills.is_none() || page.net_no != net_no {
                    page.net_no = net_no;
                    page.drills = Some(calculate_page_drills(
                        board, page.shape, net_no, via_margin,
                    ));
                }
                for d in page.drills.as_ref().unwrap() {
                    if d.shape.bounding_box().intersects(*area) {
                        result.push(d.clone());
                    }
                }
            }
        }
        result
    }
}

/// The free convex areas of one page: the page box minus every
/// undrillable item shape (inflated by the via margin), split to convex
/// pieces via the restrain machinery; each piece's center becomes a
/// drill (Java: `DrillPage.get_drills`).
fn calculate_page_drills(
    board: &BasicBoard,
    page: IntBox,
    net_no: i32,
    via_margin: i32,
) -> Vec<ExpansionDrill> {
    let page_shape = TileShape::Box(page);
    let query = page_shape.offset(via_margin as f64);
    let mut holes: Vec<(i32, Rc<TileShape>)> = Vec::new();
    for item_id in board.overlapping_items_coarse(&query, None) {
        let Some(item) = board.get_item(item_id) else { continue };
        // drillable for this net: own-net items and conduction planes
        if item.base.contains_net(net_no) {
            continue;
        }
        if let crate::board::ItemKind::ObstacleArea(a) = &item.kind {
            if a.is_conduction {
                continue;
            }
        }
        let Some(inflated) = board.inflated_shapes(item_id, via_margin) else {
            continue;
        };
        for (shape, bbox, _layer) in inflated.iter() {
            // a via spans all layers: any layer's obstacle blocks
            if bbox.intersects(page) {
                holes.push((item_id, shape.clone()));
            }
        }
    }
    let start = IncompleteRoom {
        shape: page_shape.clone(),
        layer: 0,
        contained_shape: page_shape,
    };
    let pieces = restrain_all(vec![start], &holes, |_, _| {});
    pieces
        .into_iter()
        .filter(|p| p.shape.dimension() == 2)
        .map(|p| ExpansionDrill {
            location: p.shape.centre_of_gravity().round(),
            shape: p.shape,
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{Layer, LayerStructure};
    use crate::core::Padstacks;
    use crate::rules::{BoardRules, ClearanceMatrix};

    fn test_board() -> BasicBoard {
        let stack = LayerStructure::new(vec![
            Layer::new("F.Cu", true),
            Layer::new("B.Cu", true),
        ]);
        let matrix = ClearanceMatrix::get_default_instance(stack.clone(), 200);
        let mut rules = BoardRules::new(stack.clone(), matrix);
        rules.get_default_net_class();
        let mut padstacks = Padstacks::new(2);
        padstacks.add_shape_on_layers(
            TileShape::Box(IntBox::from_coords(-400, -400, 400, 400)),
            0,
            1,
        );
        BasicBoard::new(stack, rules, padstacks)
    }

    #[test]
    fn drills_avoid_foreign_items_and_cache_invalidates() {
        let mut board = test_board();
        // anchor geometry so the board bbox is real
        board.insert_via(1, IntPoint::new(0, 0), vec![2], 1, false);
        board.insert_via(1, IntPoint::new(20000, 20000), vec![2], 1, false);
        let mut pages = DrillPageArray::new(&board, 1);
        let area = IntBox::from_coords(-2000, -2000, 22000, 22000);
        let drills = pages.drills_overlapping(&board, &area, 1, 600);
        assert!(!drills.is_empty(), "free space must yield drills");
        for d in &drills {
            // no drill center inside a foreign via's inflated shape
            for (_, it) in board.items() {
                if it.base.contains_net(1) {
                    continue;
                }
                for (s, _) in it.tile_shapes(&board.padstacks) {
                    assert!(
                        !s.offset(500.0).contains(&crate::geometry::planar::Point::Int(d.location)),
                        "drill at {:?} lands on a foreign via",
                        d.location
                    );
                }
            }
        }
        // a new item invalidates the touched pages
        let count_before = drills.len();
        board.insert_via(1, IntPoint::new(10000, 10000), vec![3], 1, false);
        pages.sync_board_changes(&board);
        let drills_after = pages.drills_overlapping(&board, &area, 1, 600);
        assert_ne!(
            count_before,
            drills_after.len(),
            "cache must recompute after a board change"
        );
    }
}
