//! Port of `DrillPageArray.java` / `DrillPage.java` / `ExpansionDrill`:
//! the board is divided into pages; each page caches the convex free
//! areas where a via can be drilled (page shape minus the undrillable
//! item shapes, split to convex pieces) and their center locations. The
//! cache is per net and invalidated by board changes, like Java.

use crate::autoroute::room_completion::{restrain_all, IncompleteRoom};
use crate::board::basic_board::BasicBoard;
use crate::geometry::planar::{IntBox, IntPoint, TileShape};
use std::sync::Arc;

/// A possible via location (Java: `ExpansionDrill`). The free-area
/// shape is reduced to its bounding box at construction: the queries
/// only need the location and a cheap area filter, and cloning shapes
/// per room entry dominated the routing profile.
#[derive(Debug, Clone, Copy)]
pub struct ExpansionDrill {
    pub bbox: IntBox,
    pub location: IntPoint,
}

#[derive(Debug)]
struct DrillPage {
    shape: IntBox,
    /// Net-independent drills: every item treated as a hole. Valid for
    /// any net with NO items in this page — the overwhelmingly common
    /// case, computed once per invalidation.
    base_drills: Option<Vec<ExpansionDrill>>,
    /// The nets of the items whose holes cut this page (drillability is
    /// net-dependent only for these).
    nets_present: Vec<i32>,
    /// Per-net drills for nets present in the page (Java's semantics:
    /// own-net items are drillable).
    net_drills: Option<Vec<ExpansionDrill>>,
    net_no: i32,
    /// The via margins the caches were computed with: nets with larger
    /// vias or clearances need wider exclusion zones, so a cache entry
    /// built for a smaller margin must not be reused (coldfire shipped
    /// vias 496 units too close to foreign vias through exactly that).
    base_margin: i32,
    net_margin: i32,
    /// Whether `net_drills` was computed with attach-to-SMD allowed
    /// (same-net drillable pins usable as via sites).
    net_attach: bool,
}

/// The array of drill pages covering the board
/// (Java: `DrillPageArray`).
#[derive(Debug)]
pub struct DrillPageArray {
    via_padstack: usize,
    page_width: i32,
    bounding: IntBox,
    pages: crate::datastructures::FxHashMap<(i32, i32), DrillPage>,
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
            .map(|p| {
                (p.from_layer()..=p.to_layer())
                    .filter_map(|l| p.get_shape(l))
                    .map(|s| s.bounding_box().max_width() as i32)
                    .max()
                    .unwrap_or(2000)
            })
            .unwrap_or(2000);
        DrillPageArray {
            via_padstack,
            page_width: (5 * via_extent).max(10_000),
            bounding: board.bounding_box(),
            pages: crate::datastructures::FxHashMap::default(),
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
                        page.base_drills = None;
                        page.net_drills = None;
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
        attach_smd: bool,
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
                let page = self.pages.entry(key).or_insert_with(|| DrillPage {
                    shape: page_box,
                    base_drills: None,
                    nets_present: Vec::new(),
                    net_drills: None,
                    net_no: -1,
                    base_margin: -1,
                    net_margin: -1,
                    net_attach: false,
                });
                if page.base_drills.is_none() || page.base_margin != via_margin {
                    let (drills, nets) = calculate_page_drills(
                        board,
                        page.shape,
                        self.via_padstack,
                        -1,
                        via_margin,
                        false,
                    );
                    page.base_drills = Some(drills);
                    page.nets_present = nets;
                    page.net_drills = None;
                    page.net_no = -1;
                    page.base_margin = via_margin;
                }
                let drills: &Vec<ExpansionDrill> = if page.nets_present.contains(&net_no) {
                    // this net has items here: own-net items are
                    // drillable, so the drill set differs (Java's
                    // per-net cache, now only where it matters)
                    if page.net_drills.is_none()
                        || page.net_no != net_no
                        || page.net_margin != via_margin
                        || page.net_attach != attach_smd
                    {
                        page.net_no = net_no;
                        page.net_margin = via_margin;
                        page.net_attach = attach_smd;
                        page.net_drills = Some(
                            calculate_page_drills(
                                board,
                                page.shape,
                                self.via_padstack,
                                net_no,
                                via_margin,
                                attach_smd,
                            )
                            .0,
                        );
                    }
                    page.net_drills.as_ref().unwrap()
                } else {
                    page.base_drills.as_ref().unwrap()
                };
                for d in drills {
                    if d.bbox.intersects(*area) {
                        result.push(*d);
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
    via_padstack: usize,
    net_no: i32,
    via_margin: i32,
    attach_smd: bool,
) -> (Vec<ExpansionDrill>, Vec<i32>) {
    let Some(via_padstack) = board.padstacks.get_by_no(via_padstack) else {
        return (Vec::new(), Vec::new());
    };
    let from_layer = via_padstack.from_layer();
    let to_layer = via_padstack.to_layer();
    let page_shape = TileShape::Box(page);
    let query = page_shape.offset(via_margin as f64);
    let mut holes: Vec<(i32, Arc<TileShape>)> = Vec::new();
    let mut nets_present: Vec<i32> = Vec::new();
    for item_id in board.overlapping_items_coarse(&query, None) {
        let Some(item) = board.get_item(item_id) else {
            continue;
        };
        nets_present.extend(item.base.net_nos.iter().copied());
        // Drillable for this net (Java Item.is_drillable): own-net traces
        // and conduction planes. Own-net PINS and VIAS are not — a via
        // must not land on them (Java DrillPage.get_drills cuts them
        // out) — except a drillable (SMD) pin when attach is allowed.
        if item.base.contains_net(net_no) {
            let drillable = if crate::drc::is_drill(&item.kind) {
                crate::drc::is_pin(item)
                    && attach_smd
                    && crate::drc::drill_allowed(item, &board.padstacks)
            } else {
                true
            };
            if drillable {
                continue;
            }
        }
        if let crate::board::ItemKind::ObstacleArea(a) = &item.kind {
            if a.is_conduction && !a.is_obstacle {
                continue;
            }
        }
        let Some(inflated) = board.inflated_shapes(item_id, via_margin) else {
            continue;
        };
        for (shape, bbox, layer) in inflated.iter() {
            // Only copper on layers actually spanned by this padstack can
            // block the via.  Treating every obstacle as through-hole made a
            // blind/buried via unusable whenever an unrelated outer layer
            // contained copper.
            if *layer >= from_layer && *layer <= to_layer && bbox.intersects(page) {
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
    nets_present.sort_unstable();
    nets_present.dedup();
    (
        pieces
            .into_iter()
            .filter(|p| p.shape.dimension() == 2)
            .map(|p| ExpansionDrill {
                location: p.shape.centre_of_gravity().round(),
                bbox: p.shape.bounding_box(),
            })
            .collect(),
        nets_present,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{Layer, LayerStructure};
    use crate::core::Padstacks;
    use crate::rules::{BoardRules, ClearanceMatrix};

    fn test_board() -> BasicBoard {
        let stack = LayerStructure::new(vec![Layer::new("F.Cu", true), Layer::new("B.Cu", true)]);
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
        let drills = pages.drills_overlapping(&board, &area, 1, 600, false);
        assert!(!drills.is_empty(), "free space must yield drills");
        for d in &drills {
            let _ = d.bbox;
            // no drill center inside a foreign via's inflated shape
            for (_, it) in board.items() {
                if it.base.contains_net(1) {
                    continue;
                }
                for (s, _) in it.tile_shapes(&board.padstacks) {
                    assert!(
                        !s.offset(500.0)
                            .contains(&crate::geometry::planar::Point::Int(d.location)),
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
        let drills_after = pages.drills_overlapping(&board, &area, 1, 600, false);
        assert_ne!(
            count_before,
            drills_after.len(),
            "cache must recompute after a board change"
        );
    }
}
