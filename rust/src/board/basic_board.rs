//! Port of the core of `board/BasicBoard.java` (incremental).
//!
//! The board combines the undoable item database with the spatial search
//! tree: items are inserted with their per-layer tile shapes into a
//! [`MinAreaTree`] keyed by (item id, shape index, layer), and all
//! overlap queries go through that tree. Java's `SearchTreeManager` with
//! multiple compensated trees follows later; this is the single
//! uncompensated default tree.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;
use std::sync::Arc;

use crate::board::item::{Item, ItemBase, ItemKind};
use crate::board::LayerStructure;
use crate::core::Padstacks;
use crate::datastructures::{LeafId, MinAreaTree, UndoableObjects};
use crate::geometry::planar::{IntBox, IntPoint, Point, Polyline, TileShape};
use crate::rules::BoardRules;

/// Unique id of an item on the board.
pub type ItemId = i32;

/// A logical component terminal named by an interchange netlist. Component
/// and pin stay separate because flattening them into `component-pin` is
/// ambiguous when either identifier itself contains a hyphen.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct LogicalEndpoint {
    pub component: String,
    pub pin: String,
}

impl LogicalEndpoint {
    pub fn new(component: impl Into<String>, pin: impl Into<String>) -> Self {
        Self {
            component: component.into(),
            pin: pin.into(),
        }
    }
}

impl std::fmt::Display for LogicalEndpoint {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}-{:?}", self.component, self.pin)
    }
}

thread_local! {
    /// Diagnostic birth tag applied to newly inserted items (see
    /// [`crate::board::item::ItemBase::birth`]).
    static BIRTH_TAG: std::cell::Cell<u8> = const { std::cell::Cell::new(0) };
}

/// Sets the diagnostic birth tag for subsequently inserted items.
pub fn set_birth_tag(tag: u8) {
    BIRTH_TAG.with(|t| t.set(tag));
}

/// Temporarily assigns a diagnostic birth tag and restores the caller's tag
/// on drop. Routing stages are nested (fanout -> maze -> shove); a bare
/// setter in one stage used to leak its tag into unrelated later inserts.
pub struct BirthTagGuard(u8);

pub fn birth_tag_scope(tag: u8) -> BirthTagGuard {
    let previous = BIRTH_TAG.with(|t| {
        let previous = t.get();
        t.set(tag);
        previous
    });
    BirthTagGuard(previous)
}

impl Drop for BirthTagGuard {
    fn drop(&mut self) {
        BIRTH_TAG.with(|t| t.set(self.0));
    }
}

/// The watch region from FR_DEBUG_REGION="x1,y1,x2,y2" (diagnostics).
fn debug_region() -> Option<IntBox> {
    thread_local! {
        static REGION: std::cell::OnceCell<Option<IntBox>> =
            const { std::cell::OnceCell::new() };
    }
    REGION.with(|r| {
        *r.get_or_init(|| {
            let v = std::env::var("FR_DEBUG_REGION").ok()?;
            let nums: Vec<i32> = v.split(',').filter_map(|p| p.parse().ok()).collect();
            let [x1, y1, x2, y2] = nums[..] else {
                return None;
            };
            Some(IntBox::from_coords(x1, y1, x2, y2))
        })
    })
}

/// One search-tree entry of an item: which shape of which item on which
/// layer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct TreeShapeEntry {
    item_id: ItemId,
    shape_index: usize,
    layer: usize,
}

#[derive(Debug)]
pub struct BasicBoard {
    pub layer_structure: LayerStructure,
    pub rules: BoardRules,
    pub padstacks: Padstacks,
    /// Board units per file coordinate unit of the imported design
    /// (DSN `resolution`); session exports must use the same factor.
    pub resolution: i32,
    /// The physical unit token of the imported `resolution` (`um`, `mil`,
    /// `inch`, `mm`). Session exports must echo it so a `mil`-based design is
    /// not silently relabelled as `um`.
    pub unit: String,
    /// The imported DSN document without its wiring section (for DSN
    /// export: the router only changes the wiring).
    pub dsn_source: Option<String>,
    /// The imported board outline — closed corner list (board units) and
    /// its clearance — when the source carried one. Preserved for
    /// lossless export: the outline polygon is not reconstructible from
    /// the boundary keepout strips, and fabricating it from the all-item
    /// bounding box grows the board by whatever copper overhangs it.
    pub outline: Option<(Vec<IntPoint>, i32)>,
    /// Semantic state captured when `dsn_source` was retained.  DSN export
    /// deliberately preserves that source verbatim outside of `wiring`, so
    /// it must fail closed if any non-wiring state is edited afterwards.
    /// Route traces/vias and derived caches are intentionally excluded by
    /// [`DsnSemanticSnapshot::capture`].
    dsn_semantic_baseline: Option<Arc<DsnSemanticSnapshot>>,
    /// Logical net endpoints named by the interchange source but not
    /// instantiated as physical board items. Legacy DSNs can legally retain
    /// such obligations (for example an unplaced component or a footprint
    /// whose pin geometry is unavailable). Keeping them explicit prevents an
    /// incomplete import from becoming vacuously electrically complete.
    unresolved_net_endpoints: BTreeMap<i32, BTreeSet<LogicalEndpoint>>,
    /// The undoable item database.
    item_list: UndoableObjects<ItemId, Item>,
    /// The spatial index over all item shapes.
    search_tree: MinAreaTree<TreeShapeEntry>,
    /// The tree leaves of each item, for removal.
    tree_entries: BTreeMap<ItemId, Vec<LeafId>>,
    /// Conduction areas (power planes) are kept out of the search tree:
    /// their board-covering bounds would poison every ancestor bound and
    /// degrade the tree queries to full scans. Checked linearly instead.
    plane_items: Vec<ItemId>,
    /// Generator for unique item ids.
    next_id_no: ItemId,
    /// Cache of item shapes inflated by an integer margin (room
    /// completion re-inflates the same obstacles for every completed
    /// room otherwise). Keyed per item id — ids are never reused, so an
    /// entry only needs clearing when its item is removed.
    /// Log of changed regions (layer, bbox) since board creation, for
    /// incremental invalidation of cached expansion rooms (Java:
    /// additional_update_after_change). Undo/redo/pop bump `change_epoch`
    /// instead, telling consumers to drop everything.
    change_log: Vec<(usize, IntBox)>,
    change_epoch: u64,
    /// Contact cache for the connectivity walks: net completeness and
    /// component checks run get_normal_contacts per item per call, which
    /// reached 43M search-tree queries in one coldfire pass. Any board
    /// change invalidates the whole cache (checks come in bursts between
    /// changes). Not cloned: a cloned board starts cold.
    contact_cache:
        std::cell::RefCell<crate::datastructures::FxHashMap<ItemId, std::sync::Arc<Vec<ItemId>>>>,
    contact_cache_log: std::cell::Cell<(u64, usize)>,
    #[allow(clippy::type_complexity)]
    inflation_cache: std::cell::RefCell<
        crate::datastructures::FxHashMap<
            ItemId,
            crate::datastructures::FxHashMap<
                i32,
                std::sync::Arc<Vec<(std::sync::Arc<TileShape>, IntBox, usize)>>,
            >,
        >,
    >,
}

impl Clone for BasicBoard {
    fn clone(&self) -> Self {
        // The item database and search tree are immutable snapshots at this
        // point, but the contact/inflation caches are derived from the live
        // rules and geometry.  Cloning those RefCells would carry answers
        // computed under the source board into a candidate transaction or an
        // optimizer worker after its rules/items diverge.  Start every clone
        // cold; the first query repopulates the cache against the clone's
        // own state.
        Self {
            layer_structure: self.layer_structure.clone(),
            rules: self.rules.clone(),
            padstacks: self.padstacks.clone(),
            resolution: self.resolution,
            unit: self.unit.clone(),
            dsn_source: self.dsn_source.clone(),
            outline: self.outline.clone(),
            dsn_semantic_baseline: self.dsn_semantic_baseline.clone(),
            unresolved_net_endpoints: self.unresolved_net_endpoints.clone(),
            item_list: self.item_list.clone(),
            search_tree: self.search_tree.clone(),
            tree_entries: self.tree_entries.clone(),
            plane_items: self.plane_items.clone(),
            next_id_no: self.next_id_no,
            change_log: self.change_log.clone(),
            change_epoch: self.change_epoch,
            contact_cache: std::cell::RefCell::new(Default::default()),
            contact_cache_log: std::cell::Cell::new((0, 0)),
            inflation_cache: std::cell::RefCell::new(Default::default()),
        }
    }
}

/// A deterministic, category-separated representation of the parts of a
/// board which are emitted from the retained DSN source.  Keeping categories
/// separate lets the exporter report *which* semantic contract was broken,
/// while avoiding a hash collision and avoiding any dependence on derived
/// board caches.
#[derive(Debug, Clone, PartialEq)]
struct DsnSemanticSnapshot {
    /// The retained source is public for historical API compatibility. Keep
    /// an exact shared copy in the baseline so callers cannot replace the
    /// source text behind the semantic snapshot and make the exporter splice
    /// a different document with the current board's wiring.
    source: Option<Arc<str>>,
    resolution: i32,
    unit: String,
    layers: String,
    outline: String,
    rules: String,
    nets: String,
    library: DsnPadstackSnapshot,
    static_items: Vec<(ItemId, ItemBase, ItemKind)>,
}

#[derive(Debug, Clone, PartialEq)]
struct DsnPadstackSnapshot {
    board_layer_count: usize,
    padstacks: Vec<DsnPadstackEntry>,
}

#[derive(Debug, Clone, PartialEq)]
struct DsnPadstackEntry {
    name: String,
    no: usize,
    attach_allowed: bool,
    placed_absolute: bool,
    shapes: Vec<Option<TileShape>>,
}

impl DsnSemanticSnapshot {
    fn capture(board: &BasicBoard) -> Self {
        let mut rules = String::new();
        // ClearanceMatrix's Debug representation contains its ordered rows,
        // names, values and layer structure.  Unlike BoardRules as a whole,
        // it contains no hash map whose iteration order could drift.
        let _ = write!(rules, "matrix={:?};", board.rules.clearance_matrix);
        let _ = write!(rules, "layers={:?};", board.rules.layer_structure());
        let _ = write!(
            rules,
            "misc={:?},{:?},{:?},{:?},{:?},{:?},{:?};",
            board.rules.get_trace_angle_restriction(),
            board.rules.get_ignore_conduction(),
            board.rules.get_min_trace_half_width(),
            board.rules.get_max_trace_half_width(),
            board.rules.get_pin_edge_to_turn_dist(),
            board.rules.get_use_slow_autoroute_algorithm(),
            board.rules.via_at_smd_allowed,
        );
        let mut same_net: Vec<_> = board.rules.same_net_clearances().collect();
        same_net.sort_by_key(|(a, b, _)| (*a, *b));
        let _ = write!(rules, "same_net={same_net:?};");
        for index in 0..board.rules.net_classes.count() {
            let _ = write!(
                rules,
                "net_class[{index}]={:?};",
                board.rules.net_classes.get(index)
            );
        }
        for index in 0..board.rules.via_infos.count() {
            let _ = write!(
                rules,
                "via_info[{index}]={:?};",
                board.rules.via_infos.get(index)
            );
        }
        for (index, via_rule) in board.rules.via_rules.iter().enumerate() {
            let _ = write!(rules, "via_rule[{index}]={via_rule:?};");
        }

        let mut nets = String::new();
        for net in board.rules.nets.iter() {
            let _ = write!(nets, "{net:?};");
        }
        let _ = write!(nets, "unresolved={:?};", board.unresolved_net_endpoints);

        let mut padstacks = Vec::with_capacity(board.padstacks.count());
        for number in 1..=board.padstacks.count() {
            let Some(padstack) = board.padstacks.get_by_no(number) else {
                continue;
            };
            // Include every slot, including empty intermediate layers.  A
            // via's transition semantics depend on the shape vector, not
            // just on its first/last occupied layer.
            let shapes = (0..padstack.board_layer_count())
                .map(|layer| padstack.get_shape(layer).cloned())
                .collect();
            padstacks.push(DsnPadstackEntry {
                name: padstack.name.clone(),
                no: padstack.no,
                attach_allowed: padstack.attach_allowed,
                placed_absolute: padstack.placed_absolute,
                shapes,
            });
        }

        let mut static_items = Vec::new();
        // `items()` is backed by an ordered map, so this is deterministic.
        // Traces and component-less vias are regenerated by the wiring
        // exporter and are intentionally absent.  Component vias (pins) and
        // all obstacle/conduction areas are part of the retained static DSN.
        for (id, item) in board.items() {
            let is_static = matches!(item.kind, ItemKind::ObstacleArea(_))
                || (item.base.component_no != 0 && matches!(item.kind, ItemKind::Via(_)));
            if is_static {
                static_items.push((*id, item.base.clone(), item.kind.clone()));
            }
        }

        DsnSemanticSnapshot {
            source: board.dsn_source.as_deref().map(Arc::<str>::from),
            resolution: board.resolution,
            unit: board.unit.clone(),
            layers: format!("{:?}", board.layer_structure),
            outline: format!("{:?}", board.outline),
            rules,
            nets,
            library: DsnPadstackSnapshot {
                board_layer_count: board.padstacks.board_layer_count,
                padstacks,
            },
            static_items,
        }
    }

    fn first_difference(&self, current: &Self) -> Option<&'static str> {
        if self.source != current.source {
            return Some("retained DSN source changed");
        }
        if self.resolution != current.resolution || self.unit != current.unit {
            return Some("resolution/unit changed");
        }
        if self.layers != current.layers {
            return Some("layer structure changed");
        }
        if self.outline != current.outline {
            return Some("board outline changed");
        }
        if self.rules != current.rules {
            return Some("routing rules or pad/via rules changed");
        }
        if self.nets != current.nets {
            return Some("net definitions changed");
        }
        if self.library != current.library {
            return Some("padstack library changed");
        }
        if self.static_items != current.static_items {
            return Some("static pins or obstacle areas changed");
        }
        None
    }
}

impl BasicBoard {
    pub fn new(layer_structure: LayerStructure, rules: BoardRules, padstacks: Padstacks) -> Self {
        BasicBoard {
            layer_structure,
            rules,
            padstacks,
            resolution: 10,
            unit: "um".to_string(),
            dsn_source: None,
            outline: None,
            dsn_semantic_baseline: None,
            unresolved_net_endpoints: BTreeMap::new(),
            item_list: UndoableObjects::new(),
            search_tree: MinAreaTree::new(),
            tree_entries: BTreeMap::new(),
            plane_items: Vec::new(),
            next_id_no: 0,
            change_log: Vec::new(),
            contact_cache: std::cell::RefCell::new(crate::datastructures::FxHashMap::default()),
            contact_cache_log: std::cell::Cell::new((0, 0)),
            change_epoch: 0,
            inflation_cache: Default::default(),
        }
    }

    /// Captures the non-wiring state against which the retained DSN source is
    /// safe to reuse.  This is called by the DSN importer only after the
    /// complete document (including its original wiring) has been loaded.
    pub(crate) fn capture_dsn_semantic_baseline(&mut self) {
        self.dsn_semantic_baseline = Some(Arc::new(DsnSemanticSnapshot::capture(self)));
    }

    /// Returns a precise reason when the board no longer matches the static
    /// portion of its retained DSN source.  A missing baseline is treated as
    /// stale rather than guessed-safe: callers that construct a board by
    /// hand must use a format-specific exporter instead of setting
    /// `dsn_source` themselves.
    pub(crate) fn dsn_source_staleness(&self) -> Option<String> {
        let Some(baseline) = &self.dsn_semantic_baseline else {
            return Some("the import baseline is missing".to_string());
        };
        let current = DsnSemanticSnapshot::capture(self);
        baseline.first_difference(&current).map(str::to_string)
    }

    /// Records an electrical endpoint that exists in the source netlist but
    /// has no physical item in this board model. Importers call this instead
    /// of either rejecting a Java-compatible DSN or silently forgetting the
    /// missing endpoint.
    pub(crate) fn record_unresolved_net_endpoint(
        &mut self,
        net_no: i32,
        endpoint: LogicalEndpoint,
    ) {
        self.unresolved_net_endpoints
            .entry(net_no)
            .or_default()
            .insert(endpoint);
    }

    /// All unresolved electrical obligations, in deterministic order.
    pub fn unresolved_net_endpoints(&self) -> impl Iterator<Item = (i32, &LogicalEndpoint)> {
        self.unresolved_net_endpoints
            .iter()
            .flat_map(|(&net_no, endpoints)| {
                endpoints.iter().map(move |endpoint| (net_no, endpoint))
            })
    }

    /// Whether `net_no` still names a logical endpoint with no board item.
    pub fn has_unresolved_net_endpoints(&self, net_no: i32) -> bool {
        self.unresolved_net_endpoints
            .get(&net_no)
            .is_some_and(|endpoints| !endpoints.is_empty())
    }

    pub fn unresolved_net_endpoint_count(&self, net_no: i32) -> usize {
        self.unresolved_net_endpoints
            .get(&net_no)
            .map_or(0, BTreeSet::len)
    }

    /// The tile shapes of an item inflated by `margin`, with their
    /// bounding boxes and layers, cached per (item, margin).
    #[allow(clippy::type_complexity)]
    pub fn inflated_shapes(
        &self,
        item_id: ItemId,
        margin: i32,
    ) -> Option<std::sync::Arc<Vec<(std::sync::Arc<TileShape>, IntBox, usize)>>> {
        if let Some(hit) = self
            .inflation_cache
            .borrow()
            .get(&item_id)
            .and_then(|per_margin| per_margin.get(&margin))
        {
            return Some(hit.clone());
        }
        let item = self.get_item(item_id)?;
        let entries: Vec<(std::sync::Arc<TileShape>, IntBox, usize)> = item
            .tile_shapes(&self.padstacks)
            .iter()
            .map(|(s, l)| {
                let inflated = if margin > 0 {
                    s.offset(margin as f64)
                } else {
                    s.clone()
                };
                let bbox = inflated.bounding_box();
                (std::sync::Arc::new(inflated), bbox, *l)
            })
            .collect();
        let rc = std::sync::Arc::new(entries);
        self.inflation_cache
            .borrow_mut()
            .entry(item_id)
            .or_default()
            .insert(margin, rc.clone());
        Some(rc)
    }

    /// The change log of (layer, bbox) regions touched by item
    /// insertions/removals; consumers remember their index.
    pub fn change_log(&self) -> &[(usize, IntBox)] {
        &self.change_log
    }

    /// Bumped by undo/redo/pop_snapshot: log consumers must drop all
    /// cached state when it changes.
    pub fn change_epoch(&self) -> u64 {
        self.change_epoch
    }

    fn log_item_regions(&mut self, item: &Item) {
        let shapes: Vec<(usize, IntBox)> = item
            .tile_shapes(&self.padstacks)
            .iter()
            .map(|(s, l)| (*l, s.bounding_box()))
            .collect();
        self.change_log.extend(shapes);
    }

    fn new_id_no(&mut self) -> ItemId {
        self.next_id_no += 1;
        self.next_id_no
    }

    /// The id the NEXT inserted item will receive. Ids are never reused,
    /// so items with `id >= next_item_id()` captured at an earlier point
    /// were created after it — a cheap watermark for "what did this step
    /// create".
    pub fn next_item_id(&self) -> ItemId {
        self.next_id_no + 1
    }

    /// Returns all currently-live items created at or after `watermark`.
    /// Item ids are monotonic and never reused, so this is a cheap and
    /// unambiguous transaction watermark for shove/route/optimizer gates.
    pub fn item_ids_since(&self, watermark: ItemId) -> Vec<ItemId> {
        self.items()
            .filter_map(|(id, _)| (*id >= watermark).then_some(*id))
            .collect()
    }

    /// Inserts an item, assigning it a fresh id. Returns the id.
    pub fn insert_item(&mut self, mut item: Item) -> ItemId {
        let id = self.new_id_no();
        item.base.id_no = id;
        if item.base.birth == 0 {
            item.base.birth = BIRTH_TAG.with(|t| t.get());
        }
        if let Some(region) = debug_region() {
            let bb = item.bounding_box(&self.padstacks);
            if bb.intersects(region) {
                eprintln!(
                    "EVENT insert {id} birth {} nets {:?} bbox {:?}",
                    item.base.birth, item.base.net_nos, bb
                );
            }
        }
        self.insert_into_search_tree(id, &item);
        self.log_item_regions(&item);
        self.item_list.insert(id, item);
        id
    }

    /// Convenience: inserts a polyline trace.
    pub fn insert_trace(
        &mut self,
        polyline: Polyline,
        layer: usize,
        half_width: i32,
        net_nos: Vec<i32>,
        clearance_class: usize,
    ) -> ItemId {
        self.insert_trace_with_provenance(
            polyline,
            layer,
            half_width,
            net_nos,
            clearance_class,
            false,
        )
    }

    /// Inserts a trace while retaining whether its clearance class was an
    /// explicit item-level override in the source document.  Replacement
    /// algorithms use this instead of inserting and mutating the bit later,
    /// so a failed snapshot transaction cannot expose a half-updated item.
    pub fn insert_trace_with_provenance(
        &mut self,
        polyline: Polyline,
        layer: usize,
        half_width: i32,
        net_nos: Vec<i32>,
        clearance_class: usize,
        clearance_class_explicit: bool,
    ) -> ItemId {
        let mut base = ItemBase::new(0, net_nos, clearance_class);
        base.clearance_class_explicit = clearance_class_explicit;
        let item = Item::new_polyline_trace(base, half_width, layer, polyline);
        self.insert_item(item)
    }

    /// Board units per millimetre, derived from the imported `resolution`
    /// (board units per physical `unit`) and `unit`. Coordinate→mm conversions
    /// must use this rather than assuming micrometres: a `mil`-based design has
    /// ~39.37 board units/mm at resolution 1, not `resolution * 1000`, so a bare
    /// `resolution * 1000` reports lengths ~25.4x too large for such fixtures.
    pub fn board_units_per_mm(&self) -> f64 {
        // micrometres per one physical `unit`
        let um_per_unit = match self.unit.to_ascii_lowercase().as_str() {
            "mil" => 25.4,
            "inch" | "in" => 25_400.0,
            "mm" => 1000.0,
            "cm" => 10_000.0,
            _ => 1.0, // um (the Specctra default)
        };
        // resolution [units/physical_unit] / um_per_unit [um/physical_unit]
        //   = units per um; times 1000 um/mm = units per mm
        self.resolution.max(1) as f64 / um_per_unit * 1000.0
    }

    /// Convenience: inserts a via with the given 1-based padstack number.
    pub fn insert_via(
        &mut self,
        padstack: usize,
        center: IntPoint,
        net_nos: Vec<i32>,
        clearance_class: usize,
        attach_allowed: bool,
    ) -> ItemId {
        self.insert_via_with_provenance(
            padstack,
            center,
            net_nos,
            clearance_class,
            attach_allowed,
            false,
        )
    }

    /// Inserts a via while retaining its item-level clearance provenance.
    pub fn insert_via_with_provenance(
        &mut self,
        padstack: usize,
        center: IntPoint,
        net_nos: Vec<i32>,
        clearance_class: usize,
        attach_allowed: bool,
        clearance_class_explicit: bool,
    ) -> ItemId {
        let mut base = ItemBase::new(0, net_nos, clearance_class);
        base.clearance_class_explicit = clearance_class_explicit;
        let item = Item::new_via(base, padstack, center, attach_allowed);
        self.insert_item(item)
    }

    /// Inserts a router-created escape via on one same-net SMD layer while
    /// preserving the selected ViaInfo's declared attach bit.
    pub fn insert_escape_via(
        &mut self,
        padstack: usize,
        center: IntPoint,
        net_nos: Vec<i32>,
        clearance_class: usize,
        attach_allowed: bool,
        smd_layer: usize,
    ) -> ItemId {
        self.insert_escape_via_with_provenance(
            padstack,
            center,
            net_nos,
            clearance_class,
            attach_allowed,
            smd_layer,
            false,
        )
    }

    /// Inserts an escape via while retaining its item-level clearance
    /// provenance.
    #[allow(clippy::too_many_arguments)]
    pub fn insert_escape_via_with_provenance(
        &mut self,
        padstack: usize,
        center: IntPoint,
        net_nos: Vec<i32>,
        clearance_class: usize,
        attach_allowed: bool,
        smd_layer: usize,
        clearance_class_explicit: bool,
    ) -> ItemId {
        let mut base = ItemBase::new(0, net_nos, clearance_class);
        base.clearance_class_explicit = clearance_class_explicit;
        let item = Item::new_escape_via(base, padstack, center, attach_allowed, smd_layer);
        self.insert_item(item)
    }

    /// Returns the unique SMD layer that permits a rule-preserving escape via
    /// at `center`, or `None` when the net/site is not the router's narrow
    /// pure-SMD case. This is the canonical reconstruction predicate shared by
    /// maze insertion and interchange readers.
    pub fn pure_smd_escape_layer(
        &self,
        net_no: i32,
        via_padstack: usize,
        center: IntPoint,
    ) -> Option<usize> {
        if self.layer_structure.layer_count() <= 1 {
            return None;
        }
        let padstack = self.padstacks.get_by_no(via_padstack)?;
        let mut result = None;
        for layer in padstack.from_layer()..=padstack.to_layer() {
            let Some(raw_shape) = padstack.get_shape(layer) else {
                continue;
            };
            let shape =
                raw_shape.translate_by(crate::geometry::planar::IntVector::new(center.x, center.y));
            for id in self.overlapping_items(&shape, Some(layer)) {
                let Some(pin) = self.get_item(id) else {
                    continue;
                };
                if let ItemKind::ObstacleArea(area) = &pin.kind {
                    if area.is_conduction {
                        return None;
                    }
                }
                if pin.base.component_no == 0
                    || !pin.base.contains_net(net_no)
                    || !matches!(pin.kind, ItemKind::Via(_))
                    || pin.first_layer(&self.padstacks) != pin.last_layer(&self.padstacks)
                {
                    continue;
                }
                let overlaps =
                    pin.tile_shapes(&self.padstacks)
                        .iter()
                        .any(|(pin_shape, pin_layer)| {
                            *pin_layer == layer && pin_shape.intersection(&shape).dimension() >= 2
                        });
                if !overlaps {
                    continue;
                }
                if result.is_some_and(|existing| existing != layer) {
                    return None;
                }
                result = Some(layer);
            }
        }
        result
    }

    /// Convenience: inserts a keepout or conduction area.
    pub fn insert_area(
        &mut self,
        area: crate::geometry::planar::PolylineArea,
        layer: usize,
        name: &str,
        net_nos: Vec<i32>,
        clearance_class: usize,
        is_conduction: bool,
    ) -> ItemId {
        let item = Item::new_obstacle_area(
            ItemBase::new(0, net_nos, clearance_class),
            area,
            layer,
            name,
            is_conduction,
        );
        self.insert_item(item)
    }

    fn insert_into_search_tree(&mut self, id: ItemId, item: &Item) {
        if let ItemKind::ObstacleArea(a) = &item.kind {
            if a.is_conduction {
                if !self.plane_items.contains(&id) {
                    self.plane_items.push(id);
                }
                return;
            }
        }
        let mut leaves = Vec::new();
        for (index, (shape, layer)) in item.tile_shapes(&self.padstacks).iter().enumerate() {
            let layer = *layer;
            let Some(bound) = shape.bounding_octagon() else {
                continue;
            };
            leaves.push(self.search_tree.insert(
                TreeShapeEntry {
                    item_id: id,
                    shape_index: index,
                    layer,
                },
                index,
                bound,
            ));
        }
        self.tree_entries.insert(id, leaves);
    }

    fn remove_from_search_tree(&mut self, id: ItemId) {
        if let Some(leaves) = self.tree_entries.remove(&id) {
            self.search_tree.remove(&leaves);
        }
        self.plane_items.retain(|&p| p != id);
    }

    /// Removes an item from the board. Returns false if no such item is
    /// alive.
    pub fn remove_item(&mut self, id: ItemId) -> bool {
        if self.item_list.get(&id).is_none() {
            return false;
        }
        self.inflation_cache.borrow_mut().remove(&id);
        if let Some(item) = self.item_list.get(&id).cloned() {
            self.log_item_regions(&item);
        }
        if let Some(region) = debug_region() {
            if let Some(item) = self.item_list.get(&id) {
                let bb = item.bounding_box(&self.padstacks);
                if bb.intersects(region) {
                    eprintln!(
                        "EVENT remove {id} birth {} nets {:?}",
                        item.base.birth, item.base.net_nos
                    );
                }
            }
        }
        self.remove_from_search_tree(id);
        self.item_list.delete(&id)
    }

    /// The currently alive item with `id`.
    pub fn get_item(&self, id: ItemId) -> Option<&Item> {
        self.item_list.get(&id)
    }

    /// Iterates over the alive items.
    pub fn items(&self) -> impl Iterator<Item = (&ItemId, &Item)> {
        self.item_list.iter()
    }

    pub fn item_count(&self) -> usize {
        self.item_list.iter().count()
    }

    /// The ids of the items whose tree shapes overlap `shape` on `layer`
    /// (layer `None` = all layers), sorted and deduplicated.
    pub fn overlapping_items(&self, shape: &TileShape, layer: Option<usize>) -> Vec<ItemId> {
        let Some(query) = shape.bounding_octagon() else {
            return Vec::new();
        };
        // callback traversal: the collecting variant allocated a LeafId
        // vector per query, visible in the routing profile
        let mut result: Vec<ItemId> = Vec::new();
        self.search_tree.overlaps_with(query, |leaf| {
            let entry = *self.search_tree.entry(leaf).object;
            if layer.is_some_and(|l| entry.layer != l) {
                return;
            }
            // exact check against the item's real shape (the tree only
            // stores bounding octagons)
            if self
                .get_item(entry.item_id)
                .and_then(|item| item.tile_shape(entry.shape_index, &self.padstacks))
                .is_some_and(|(s, _)| s.intersects(shape))
            {
                result.push(entry.item_id);
            }
        });
        // planes live outside the tree; the few of them check linearly
        for &plane_id in &self.plane_items {
            let Some(item) = self.get_item(plane_id) else {
                continue;
            };
            let matches = item
                .tile_shapes(&self.padstacks)
                .iter()
                .any(|(s, l)| layer.is_none_or(|want| *l == want) && s.intersects(shape));
            if matches {
                result.push(plane_id);
            }
        }
        result.sort();
        result.dedup();
        result
    }

    /// Like [`Self::overlapping_items`], but WITHOUT the exact per-shape
    /// intersection filter: returns every item whose tree bounding octagon
    /// overlaps `shape` (plus the planes matching by bbox). Callers must
    /// do their own exact filtering; use when the candidate set is large
    /// and downstream work discards far items cheaply anyway.
    pub fn overlapping_items_coarse(&self, shape: &TileShape, layer: Option<usize>) -> Vec<ItemId> {
        let Some(query) = shape.bounding_octagon() else {
            return Vec::new();
        };
        let mut result: Vec<ItemId> = Vec::new();
        self.search_tree.overlaps_with(query, |leaf| {
            let entry = self.search_tree.entry(leaf).object;
            if layer.is_none_or(|l| entry.layer == l) {
                result.push(entry.item_id);
            }
        });
        let query_bbox = shape.bounding_box();
        for &plane_id in &self.plane_items {
            let Some(item) = self.get_item(plane_id) else {
                continue;
            };
            let matches = item.tile_shapes(&self.padstacks).iter().any(|(s, l)| {
                layer.is_none_or(|want| *l == want) && s.bounding_box().intersects(query_bbox)
            });
            if matches {
                result.push(plane_id);
            }
        }
        result.sort();
        result.dedup();
        result
    }

    /// The connectable items of `net_no` touching `shape` on `layer`.
    pub fn overlapping_items_of_net(
        &self,
        shape: &TileShape,
        layer: usize,
        net_no: i32,
    ) -> Vec<ItemId> {
        self.overlapping_items(shape, Some(layer))
            .into_iter()
            .filter(|id| {
                self.get_item(*id)
                    .is_some_and(|item| item.base.contains_net(net_no))
            })
            .collect()
    }

    /// True if inserting a shape of `net_no` at `shape` on `layer` would
    /// collide with a foreign-net item (simplified obstacle rule: items of
    /// a different net are obstacles).
    pub fn is_blocked(&self, shape: &TileShape, layer: usize, net_no: i32) -> bool {
        self.overlapping_items(shape, Some(layer))
            .into_iter()
            .any(|id| {
                self.get_item(id).is_some_and(|item| {
                    // foreign conduction areas (power planes) do not block:
                    // they receive clearance cutouts in fabrication (Java:
                    // ConductionArea is no obstacle for foreign items)
                    if let ItemKind::ObstacleArea(a) = &item.kind {
                        // A via-only keepout constrains drill placement, not
                        // traces.  `is_blocked` is used by trace routing and
                        // pull-tight, so do not turn the keepout into a
                        // foreign-copper wall here.
                        if a.via_only {
                            return false;
                        }
                        if a.is_conduction && !a.is_obstacle {
                            return false;
                        }
                    }
                    !item.base.contains_net(net_no)
                })
            })
    }

    // ---- connectivity (Java: Item/Trace/DrillItem contact methods) ----

    /// The contacts of the item `id` at `point` (for traces: only at their
    /// end corners). An item is a contact if it overlaps at the point,
    /// shares a layer and (unless `ignore_net`) a net, and touches
    /// according to its kind: traces by an end corner, drill items by
    /// their center.
    pub fn get_normal_contacts_at(
        &self,
        id: ItemId,
        point: &Point,
        ignore_net: bool,
    ) -> Vec<ItemId> {
        let Some(item) = self.get_item(id) else {
            return Vec::new();
        };
        let search_shape = TileShape::Box(point.surrounding_box());
        // Use the actual populated padstack layers, not only the declared
        // first/last span.  A sparse blind/buried via may have no copper on
        // an intermediate layer; treating its span as continuous creates a
        // false electrical contact at that layer.
        let item_layers: Vec<usize> = item
            .tile_shapes(&self.padstacks)
            .iter()
            .map(|(_, layer)| *layer)
            .collect();
        if item_layers.is_empty() {
            return Vec::new();
        }
        let mut result = Vec::new();
        for other_id in self.overlapping_items(&search_shape, None) {
            if other_id == id {
                continue;
            }
            let Some(other) = self.get_item(other_id) else {
                continue;
            };
            // shares an actual populated layer?
            if !other
                .tile_shapes(&self.padstacks)
                .iter()
                .any(|(_, layer)| item_layers.contains(layer))
            {
                continue;
            }
            if !ignore_net && !other.base.shares_net(&item.base) {
                continue;
            }
            let touches = match &other.kind {
                ItemKind::PolylineTrace(t) => {
                    *point == t.first_corner() || *point == t.last_corner()
                }
                // a trace may end anywhere inside a pad/via shape, not
                // only at its center (pad shapes can be off-center, e.g.
                // the staggered TO-92 pads); Java uses shape containment.
                // The cached padstack bounding box rejects cheaply before
                // the tile shapes are built.
                ItemKind::Via(v) => {
                    *point == Point::Int(v.center)
                        || (self.padstacks.get_by_no(v.padstack).is_some_and(|p| {
                            let bb = p.bounding_box();
                            !bb.is_empty() && {
                                let f = point.to_float();
                                f.x >= (v.center.x + bb.ll.x) as f64
                                    && f.x <= (v.center.x + bb.ur.x) as f64
                                    && f.y >= (v.center.y + bb.ll.y) as f64
                                    && f.y <= (v.center.y + bb.ur.y) as f64
                            }
                        }) && other
                            .tile_shapes(&self.padstacks)
                            .iter()
                            .any(|(s, layer)| item_layers.contains(layer) && s.contains(point)))
                }
                ItemKind::ObstacleArea(a) => {
                    a.is_conduction && item_layers.contains(&a.layer) && a.area.contains(point)
                }
            };
            if touches {
                result.push(other_id);
            }
        }
        result.sort();
        result.dedup();
        result
    }

    /// All contacts of the item `id` (Java: `get_normal_contacts`): for a
    /// trace the contacts at its two end corners, for a via the contacts
    /// at its center.
    /// [`Self::get_normal_contacts`] through the contact cache: valid
    /// while the board is unchanged (epoch + log length).
    pub fn get_normal_contacts_cached(&self, id: ItemId) -> std::sync::Arc<Vec<ItemId>> {
        let stamp = (self.change_epoch, self.change_log.len());
        if self.contact_cache_log.get() != stamp {
            self.contact_cache.borrow_mut().clear();
            self.contact_cache_log.set(stamp);
        }
        if let Some(hit) = self.contact_cache.borrow().get(&id) {
            return hit.clone();
        }
        let computed = std::sync::Arc::new(self.get_normal_contacts(id));
        self.contact_cache.borrow_mut().insert(id, computed.clone());
        computed
    }

    pub fn get_normal_contacts(&self, id: ItemId) -> Vec<ItemId> {
        let Some(item) = self.get_item(id) else {
            return Vec::new();
        };
        let mut result = match &item.kind {
            ItemKind::PolylineTrace(t) => {
                let mut r = self.get_normal_contacts_at(id, &t.first_corner(), false);
                r.extend(self.get_normal_contacts_at(id, &t.last_corner(), false));
                r
            }
            ItemKind::Via(v) => {
                let mut r = self.get_normal_contacts_at(id, &Point::Int(v.center), false);
                // traces may also end anywhere inside the pad shapes
                for (shape, layer) in item.tile_shapes(&self.padstacks) {
                    let layer = *layer;
                    for other_id in self.overlapping_items(shape, Some(layer)) {
                        if other_id == id {
                            continue;
                        }
                        let Some(other) = self.get_item(other_id) else {
                            continue;
                        };
                        if !other.base.shares_net(&item.base) {
                            continue;
                        }
                        match &other.kind {
                            ItemKind::PolylineTrace(t) => {
                                if t.layer == layer
                                    && (shape.contains(&t.first_corner())
                                        || shape.contains(&t.last_corner()))
                                {
                                    r.push(other_id);
                                }
                            }
                            // drill items whose center lies inside this
                            // pad shape: makes the containment contact
                            // symmetric (the other side checks its own
                            // center against our shapes)
                            ItemKind::Via(ov) => {
                                if shape.contains(&Point::Int(ov.center)) {
                                    r.push(other_id);
                                }
                            }
                            ItemKind::ObstacleArea(_) => {}
                        }
                    }
                }
                r
            }
            ItemKind::ObstacleArea(a) => {
                // a conduction area contacts the connectable items whose
                // connection point lies inside the area
                if !a.is_conduction {
                    return Vec::new();
                }
                let query = TileShape::Box(a.area.bounding_box());
                let mut r = Vec::new();
                for other_id in self.overlapping_items(&query, Some(a.layer)) {
                    if other_id == id {
                        continue;
                    }
                    let Some(other) = self.get_item(other_id) else {
                        continue;
                    };
                    if !other.base.shares_net(&item.base) {
                        continue;
                    }
                    let touches = match &other.kind {
                        ItemKind::PolylineTrace(t) => {
                            a.area.contains(&t.first_corner()) || a.area.contains(&t.last_corner())
                        }
                        ItemKind::Via(v) => a.area.contains(&Point::Int(v.center)),
                        ItemKind::ObstacleArea(_) => false,
                    };
                    if touches {
                        r.push(other_id);
                    }
                }
                r
            }
        };
        result.sort();
        result.dedup();
        result
    }

    /// The contacts of a trace at its start corner.
    pub fn get_start_contacts(&self, id: ItemId) -> Vec<ItemId> {
        match self.get_item(id).map(|i| &i.kind) {
            Some(ItemKind::PolylineTrace(t)) => {
                self.get_normal_contacts_at(id, &t.first_corner(), false)
            }
            _ => Vec::new(),
        }
    }

    /// The contacts of a trace at its end corner.
    pub fn get_end_contacts(&self, id: ItemId) -> Vec<ItemId> {
        match self.get_item(id).map(|i| &i.kind) {
            Some(ItemKind::PolylineTrace(t)) => {
                self.get_normal_contacts_at(id, &t.last_corner(), false)
            }
            _ => Vec::new(),
        }
    }

    /// True if the trace's endpoints stay connected through OTHER items
    /// (Java: `Trace.is_cycle`): the trace is a redundant parallel path.
    /// Conduction areas neither count as targets nor as hops (Java's
    /// default `ignore_cycles_with_areas` — planes must not make every
    /// plane-touching trace a cycle).
    pub fn trace_is_cycle(&self, id: ItemId) -> bool {
        let Some(item) = self.get_item(id) else {
            return false;
        };
        if !matches!(item.kind, ItemKind::PolylineTrace(_)) {
            return false;
        }
        let skip = |board: &Self, other: ItemId| -> bool {
            matches!(
                board.get_item(other).map(|i| &i.kind),
                Some(ItemKind::ObstacleArea(_)) | None
            )
        };
        let starts: Vec<ItemId> = self
            .get_start_contacts(id)
            .into_iter()
            .filter(|&c| !skip(self, c))
            .collect();
        let ends: Vec<ItemId> = self
            .get_end_contacts(id)
            .into_iter()
            .filter(|&c| !skip(self, c))
            .collect();
        if starts.is_empty() || ends.is_empty() {
            return false;
        }
        // BFS from the start contacts, never passing through the trace
        // itself: reaching an end contact proves the parallel path
        let mut visited: Vec<ItemId> = starts.clone();
        let mut queue = starts;
        while let Some(cur) = queue.pop() {
            if ends.contains(&cur) {
                return true;
            }
            for &next in self.get_normal_contacts_cached(cur).iter() {
                if next == id || visited.contains(&next) || skip(self, next) {
                    continue;
                }
                visited.push(next);
                queue.push(next);
            }
        }
        false
    }

    /// A same-net dangling trace at `point`: its first corner sits there
    /// with no start contacts, or its last with no end contacts
    /// (Java: `BasicBoard.get_trace_tail`).
    fn get_trace_tail(&self, point: &Point, layer: usize, net_nos: &[i32]) -> Option<ItemId> {
        let search_shape = TileShape::Box(point.surrounding_box());
        for other_id in self.overlapping_items(&search_shape, Some(layer)) {
            let Some(other) = self.get_item(other_id) else {
                continue;
            };
            let ItemKind::PolylineTrace(t) = &other.kind else {
                continue;
            };
            if t.layer != layer || other.base.net_nos != net_nos {
                continue;
            }
            if t.first_corner() == *point && self.get_start_contacts(other_id).is_empty() {
                return Some(other_id);
            }
            if t.last_corner() == *point && self.get_end_contacts(other_id).is_empty() {
                return Some(other_id);
            }
        }
        None
    }

    /// Removes the trace if it is a redundant cycle, then removes any
    /// tails the removal created at its endpoints (Java:
    /// `BasicBoard.remove_if_cycle`). Fixed traces are never removed.
    pub fn remove_if_cycle(&mut self, id: ItemId) -> bool {
        let Some(item) = self.get_item(id) else {
            return false;
        };
        if item.base.is_user_fixed() {
            return false;
        }
        let ItemKind::PolylineTrace(t) = &item.kind else {
            return false;
        };
        if !self.trace_is_cycle(id) {
            return false;
        }
        // Protect a trace that is the SOLE wire reaching a component pin's
        // connection point (its drill center). The cycle test uses the lenient
        // in-pad containment rule, which treats the pin as already connected via
        // a nearby off-centre trace end, so it would remove the only wire
        // actually reaching the connection point — leaving the net reloadable
        // only as a dangling track (finding #2). Only protect when no OTHER wire
        // (trace end or fanout via) already sits on that point, so a genuinely
        // redundant detour that merely touches a pin is still removable.
        let layer0 = t.layer;
        let net0 = item.base.net_nos.clone();
        let sole_pin_connection = [t.first_corner(), t.last_corner()].iter().any(|c| {
            let cp = c.to_float().round();
            let q = TileShape::Box(IntBox::from_coords(cp.x - 1, cp.y - 1, cp.x + 1, cp.y + 1));
            let hits = self.overlapping_items(&q, Some(layer0));
            let is_pin_center = hits.iter().any(|&oid| {
                self.get_item(oid).is_some_and(|it| {
                    it.base.component_no != 0
                        && matches!(&it.kind, ItemKind::Via(v) if v.center == cp)
                })
            });
            if !is_pin_center {
                return false;
            }
            // another same-net wire already on this point → this trace is not sole
            let other_wire = hits.iter().any(|&oid| {
                oid != id
                    && self.get_item(oid).is_some_and(|it| {
                        it.base.net_nos == net0
                            && match &it.kind {
                                ItemKind::PolylineTrace(ot) => {
                                    ot.first_corner().to_float().round() == cp
                                        || ot.last_corner().to_float().round() == cp
                                }
                                ItemKind::Via(v) => it.base.component_no == 0 && v.center == cp,
                                _ => false,
                            }
                    })
            });
            !other_wire
        });
        if sole_pin_connection {
            return false;
        }
        if std::env::var_os("FR_CYCLE_DEBUG").is_some() {
            eprintln!(
                "CYCLE REMOVE trace {id} nets {:?} {:?} -> {:?}",
                item.base.net_nos,
                t.first_corner(),
                t.last_corner()
            );
        }
        let layer = t.layer;
        let net_nos = item.base.net_nos.clone();
        let end_corners = [t.first_corner(), t.last_corner()];
        let tail_before: Vec<bool> = end_corners
            .iter()
            .map(|c| self.get_trace_tail(c, layer, &net_nos).is_some())
            .collect();
        // transactional, in the port's check-by-doing style: the contact
        // graph the cycle test walks is shape-based and can diverge from
        // electrical reality at tolerance edges — a removal that degrades
        // net connectivity is rolled back instead of trusted
        let complete_before: Vec<bool> = net_nos
            .iter()
            .map(|&n| self.net_is_completely_connected(n))
            .collect();
        self.generate_snapshot();
        self.remove_item(id);
        for (i, corner) in end_corners.iter().enumerate() {
            if tail_before[i] {
                continue;
            }
            // follow the freshly created tail chain outward
            let mut at = corner.clone();
            while let Some(tail) = self.get_trace_tail(&at, layer, &net_nos) {
                let Some(ItemKind::PolylineTrace(tt)) = self.get_item(tail).map(|i| i.kind.clone())
                else {
                    break;
                };
                let other_end = if tt.first_corner() == at {
                    tt.last_corner()
                } else {
                    tt.first_corner()
                };
                self.remove_item(tail);
                at = other_end;
            }
        }
        let degraded = net_nos
            .iter()
            .zip(&complete_before)
            .any(|(&n, &before)| before && !self.net_is_completely_connected(n));
        if degraded {
            self.rollback_snapshot();
            return false;
        }
        self.pop_snapshot();
        true
    }

    /// True if the trace is not contacted at its first or its last corner
    /// (Java: `Trace.is_tail`).
    pub fn is_tail(&self, id: ItemId) -> bool {
        match self.get_item(id).map(|i| &i.kind) {
            Some(ItemKind::PolylineTrace(_)) => {
                self.get_start_contacts(id).is_empty() || self.get_end_contacts(id).is_empty()
            }
            _ => false,
        }
    }

    /// The set of items connected to `id` (including itself) by contacts
    /// of the net `net_no` (Java: `Item.get_connected_set`).
    pub fn get_connected_set(&self, id: ItemId, net_no: i32) -> Vec<ItemId> {
        let Some(item) = self.get_item(id) else {
            return Vec::new();
        };
        if !item.base.contains_net(net_no) {
            return Vec::new();
        }
        let mut visited = vec![id];
        let mut queue = vec![id];
        while let Some(curr) = queue.pop() {
            for &contact in self.get_normal_contacts_cached(curr).iter() {
                if visited.contains(&contact) {
                    continue;
                }
                if self
                    .get_item(contact)
                    .is_some_and(|i| i.base.contains_net(net_no))
                {
                    visited.push(contact);
                    queue.push(contact);
                }
            }
        }
        visited.sort();
        visited
    }

    /// True if all connectable items of `net_no` form one connected set.
    ///
    /// A declared net with no physical item is *not* electrically complete.
    /// Treating the empty set as connected lets an incomplete import pass the
    /// router/DRC success gates (and was the source of several false-success
    /// reports in the API path).
    pub fn net_is_completely_connected(&self, net_no: i32) -> bool {
        if self.has_unresolved_net_endpoints(net_no) {
            return false;
        }
        let net_items: Vec<ItemId> = self
            .items()
            .filter(|(_, item)| item.base.contains_net(net_no) && item.is_connectable())
            .map(|(id, _)| *id)
            .collect();
        let Some(&first) = net_items.first() else {
            return false;
        };
        let connected = self.get_connected_set(first, net_no);
        net_items.iter().all(|id| connected.contains(id))
    }

    /// Invalidation shared by the metadata setters below: fixed state,
    /// component and obstacle flags decide what the router may rip, move
    /// or shove, so cached expansion rooms over the item's regions are
    /// stale after a change (the geometry is unchanged, the
    /// classification is not).
    fn log_metadata_change(&mut self, id: ItemId) {
        if let Some(item) = self.get_item(id).cloned() {
            self.log_item_regions(&item);
        }
    }

    /// Marks an item as belonging to a component (pins are not ripped up
    /// and not written to session files). Undoable.
    pub fn set_component_no(&mut self, id: ItemId, component_no: i32) {
        if self
            .get_item(id)
            .is_none_or(|i| i.base.component_no == component_no)
        {
            return;
        }
        self.item_list.save_for_undo(&id);
        if let Some(item) = self.item_list.get_mut(&id) {
            item.base.component_no = component_no;
        }
        self.log_metadata_change(id);
    }

    /// Sets the diagnostic birth tag of an alive item.
    pub fn set_birth(&mut self, id: ItemId, birth: u8) {
        if let Some(item) = self.item_list.get_mut(&id) {
            item.base.birth = birth;
        }
    }

    /// Changes only an item's design-rule classification. Geometry and tree
    /// leaves stay valid, but clearance-inflation and expansion-room caches
    /// must be invalidated. Used when a `.rules` sidecar reclassifies nets
    /// after their DSN items have already been instantiated.
    pub fn set_item_clearance_class(&mut self, id: ItemId, clearance_class: usize) {
        if self
            .get_item(id)
            .is_none_or(|item| item.base.clearance_class == clearance_class)
        {
            return;
        }
        self.item_list.save_for_undo(&id);
        if let Some(item) = self.item_list.get_mut(&id) {
            item.base.clearance_class = clearance_class;
        }
        self.inflation_cache.borrow_mut().remove(&id);
        self.log_metadata_change(id);
    }

    /// Records that an item's clearance class came from an explicit
    /// item-level file scope rather than its net-class default.
    pub fn set_item_clearance_class_explicit(&mut self, id: ItemId, explicit: bool) {
        if self
            .get_item(id)
            .is_none_or(|item| item.base.clearance_class_explicit == explicit)
        {
            return;
        }
        self.item_list.save_for_undo(&id);
        if let Some(item) = self.item_list.get_mut(&id) {
            item.base.clearance_class_explicit = explicit;
        }
    }

    /// Sets the fixed state of an item. Undoable: the previous state is
    /// restored on undo instead of leaking through the snapshot.
    pub fn set_fixed_state(&mut self, id: ItemId, state: crate::board::FixedState) {
        if self
            .get_item(id)
            .is_none_or(|i| i.base.fixed_state == state)
        {
            return;
        }
        self.item_list.save_for_undo(&id);
        if let Some(item) = self.item_list.get_mut(&id) {
            item.base.fixed_state = state;
        }
        self.log_metadata_change(id);
    }

    /// Marks a conduction area as also being a clearance obstacle to
    /// foreign-net copper (Java `ConductionArea.is_obstacle`). Undoable.
    pub fn set_area_is_obstacle(&mut self, id: ItemId, is_obstacle: bool) {
        let changes = self.get_item(id).is_some_and(
            |i| matches!(&i.kind, ItemKind::ObstacleArea(a) if a.is_obstacle != is_obstacle),
        );
        if !changes {
            return;
        }
        self.item_list.save_for_undo(&id);
        if let Some(item) = self.item_list.get_mut(&id) {
            if let ItemKind::ObstacleArea(a) = &mut item.kind {
                a.is_obstacle = is_obstacle;
            }
        }
        self.log_metadata_change(id);
    }

    /// Marks an obstacle area as constraining via placement only (DSN
    /// `(via_keepout ...)`, Java `ViaObstacleArea`). Undoable.
    pub fn set_area_via_only(&mut self, id: ItemId, via_only: bool) {
        let changes = self.get_item(id).is_some_and(
            |i| matches!(&i.kind, ItemKind::ObstacleArea(a) if a.via_only != via_only),
        );
        if !changes {
            return;
        }
        self.item_list.save_for_undo(&id);
        if let Some(item) = self.item_list.get_mut(&id) {
            if let ItemKind::ObstacleArea(a) = &mut item.kind {
                a.via_only = via_only;
            }
        }
        self.log_metadata_change(id);
    }

    /// Splits a trace of `net_no` on `layer` whose center line passes
    /// through `point` (not at an endpoint) into two traces meeting there,
    /// so that contacts at the junction register
    /// (Java: part of `PolylineTrace` normalization). Returns true if a
    /// trace was split.
    pub fn split_traces_at(&mut self, point: IntPoint, layer: usize, net_no: i32) -> bool {
        use crate::geometry::planar::{Line, LineSegment};
        // find a splittable trace
        let mut candidate: Option<(ItemId, usize)> = None;
        for (id, item) in self.items() {
            let ItemKind::PolylineTrace(t) = &item.kind else {
                continue;
            };
            if t.layer != layer || !item.base.contains_net(net_no) {
                continue;
            }
            let p = Point::Int(point);
            if t.first_corner() == p || t.last_corner() == p {
                continue;
            }
            if !t.contains_on_center_line(point) {
                continue;
            }
            // the polyline line index containing the point
            for i in 1..t.polyline.arr.len() - 1 {
                if LineSegment::from_polyline(&t.polyline, i).contains(point) {
                    candidate = Some((*id, i));
                    break;
                }
            }
            if candidate.is_some() {
                break;
            }
        }
        let Some((id, line_no)) = candidate else {
            return false;
        };
        let item = self.get_item(id).unwrap().clone();
        let ItemKind::PolylineTrace(t) = &item.kind else {
            unreachable!();
        };
        // cut with the perpendicular line through the point
        let cut_direction = t.polyline.arr[line_no].direction().turn_45_degree(2);
        let cut_line = Line::from_direction(point, cut_direction);
        let Some([first, second]) = t.polyline.split(line_no, cut_line) else {
            return false;
        };
        let half_width = t.half_width;
        let layer = t.layer;
        let net_nos = item.base.net_nos.clone();
        let clearance_class = item.base.clearance_class;
        let clearance_class_explicit = item.base.clearance_class_explicit;
        // splitting is normalization, not a route change: the pieces keep
        // the protected state (Java Trace.split keeps the fixed state)
        let fixed_state = item.base.fixed_state;
        self.remove_item(id);
        let a = self.insert_trace_with_provenance(
            first,
            layer,
            half_width,
            net_nos.clone(),
            clearance_class,
            clearance_class_explicit,
        );
        let b = self.insert_trace_with_provenance(
            second,
            layer,
            half_width,
            net_nos,
            clearance_class,
            clearance_class_explicit,
        );
        self.set_fixed_state(a, fixed_state);
        self.set_fixed_state(b, fixed_state);
        true
    }

    /// Splits same-net traces passing through the via's center on every
    /// layer it spans, so the via's contacts register. Imported wiring
    /// (DSN/SES/KiCad JSON) may place vias mid-trace; contacts require a
    /// trace ENDPOINT at the pad (Java routes always end at via centers).
    pub fn split_traces_at_via(&mut self, via_id: ItemId) {
        let Some(item) = self.get_item(via_id).cloned() else {
            return;
        };
        let ItemKind::Via(v) = &item.kind else {
            return;
        };
        let center = v.center;
        let first = item.first_layer(&self.padstacks);
        let last = item.last_layer(&self.padstacks);
        for net in item.base.net_nos.clone() {
            for layer in first..=last {
                while self.split_traces_at(center, layer, net) {}
            }
        }
    }

    /// Combines a trace with neighbour traces at its end corners while the
    /// only contact there is exactly one other trace with the same layer,
    /// half width and nets (Java: `PolylineTrace.combine`). Returns the id
    /// of the surviving combined trace.
    pub fn combine_trace(&mut self, id: ItemId) -> ItemId {
        let mut current = id;
        loop {
            let Some(item) = self.get_item(current) else {
                return current;
            };
            let ItemKind::PolylineTrace(t) = item.kind.clone() else {
                return current;
            };
            let base = item.base.clone();
            let mut combined = false;
            for corner in [t.first_corner(), t.last_corner()] {
                // contacts at this corner, ignoring conduction areas
                let contacts: Vec<ItemId> = self
                    .get_normal_contacts_at(current, &corner, false)
                    .into_iter()
                    .filter(|c| {
                        !matches!(
                            self.get_item(*c).map(|i| &i.kind),
                            Some(ItemKind::ObstacleArea(_))
                        )
                    })
                    .collect();
                let [other_id] = contacts[..] else {
                    continue;
                };
                let Some(other) = self.get_item(other_id) else {
                    continue;
                };
                let ItemKind::PolylineTrace(other_t) = &other.kind else {
                    continue;
                };
                if other_t.layer != t.layer
                    || other_t.half_width != t.half_width
                    || other.base.net_nos != base.net_nos
                    // A combined trace has one clearance class.  Merging
                    // unlike classes would silently downgrade the stricter
                    // segment (and could make a later DRC violation depend
                    // on which endpoint happened to survive).
                    || other.base.clearance_class != base.clearance_class
                    // An inherited and an explicit segment cannot be merged:
                    // one replacement item cannot preserve both sidecar
                    // behaviours when a net class changes later.
                    || other.base.clearance_class_explicit
                        != base.clearance_class_explicit
                    || other.base.is_user_fixed()
                    || base.is_user_fixed()
                    // never absorb a trace with a DIFFERENT protection level:
                    // combining a ShoveFixed trace into an Unfixed one (or
                    // vice versa) would silently change what the shove and
                    // ripup algorithms may touch
                    || other.base.fixed_state != base.fixed_state
                {
                    continue;
                }
                let combined_polyline = t.polyline.combine(&other_t.polyline);
                if combined_polyline == t.polyline || combined_polyline.is_empty() {
                    continue;
                }
                let half_width = t.half_width;
                let layer = t.layer;
                let net_nos = base.net_nos.clone();
                let clearance_class = base.clearance_class;
                let clearance_class_explicit = base.clearance_class_explicit;
                // Java combines into `this`, keeping its fixed state
                let fixed_state = base.fixed_state;
                self.remove_item(current);
                self.remove_item(other_id);
                current = self.insert_trace_with_provenance(
                    combined_polyline,
                    layer,
                    half_width,
                    net_nos,
                    clearance_class,
                    clearance_class_explicit,
                );
                self.set_fixed_state(current, fixed_state);
                combined = true;
                break;
            }
            if !combined {
                return current;
            }
        }
    }

    /// The smallest box containing all items of the board.
    pub fn bounding_box(&self) -> IntBox {
        let mut result = IntBox::EMPTY;
        for (_, item) in self.items() {
            result = result.union(item.bounding_box(&self.padstacks));
        }
        result
    }

    /// Makes the current state restorable by undo.
    pub fn generate_snapshot(&mut self) {
        self.item_list.generate_snapshot();
    }

    /// Removes the top snapshot without restoring it (commits the changes
    /// made since the snapshot into the previous level).
    pub fn pop_snapshot(&mut self) -> bool {
        self.change_epoch += 1;
        self.item_list.pop_snapshot()
    }

    /// Restores the situation before the last snapshot, resynchronizing
    /// the search tree. Returns false if no undo is possible.
    pub fn undo(&mut self) -> bool {
        let mut cancelled = Vec::new();
        let mut restored = Vec::new();
        if !self.item_list.undo(&mut cancelled, &mut restored) {
            return false;
        }
        self.resync_search_tree(&cancelled, &restored);
        true
    }

    /// Rolls back the latest speculative snapshot and discards its redo
    /// branch. Internal routing transactions use this instead of [`Self::undo`]
    /// so a rejected candidate cannot later be resurrected through the public
    /// board redo API.
    pub fn rollback_snapshot(&mut self) -> bool {
        let mut cancelled = Vec::new();
        let mut restored = Vec::new();
        if !self
            .item_list
            .rollback_snapshot(&mut cancelled, &mut restored)
        {
            return false;
        }
        self.resync_search_tree(&cancelled, &restored);
        true
    }

    pub(crate) fn can_redo(&self) -> bool {
        self.item_list.can_redo()
    }

    /// Restores the situation before the last undo. Returns false if no
    /// redo is possible.
    pub fn redo(&mut self) -> bool {
        let mut cancelled = Vec::new();
        let mut restored = Vec::new();
        if !self.item_list.redo(&mut cancelled, &mut restored) {
            return false;
        }
        self.resync_search_tree(&cancelled, &restored);
        true
    }

    fn resync_search_tree(&mut self, cancelled: &[Item], restored: &[Item]) {
        // items may reappear or vanish wholesale: cached room graphs and
        // inflations cannot track this incrementally
        self.change_epoch += 1;
        for item in cancelled.iter().chain(restored) {
            self.inflation_cache.borrow_mut().remove(&item.base.id_no);
        }
        // Remove the tree entries of all touched items, then reinsert the
        // ones which are alive again.
        for item in cancelled.iter().chain(restored) {
            self.remove_from_search_tree(item.base.id_no);
        }
        let to_reinsert: Vec<ItemId> = cancelled
            .iter()
            .chain(restored)
            .map(|item| item.base.id_no)
            .collect();
        for id in to_reinsert {
            if let Some(item) = self.item_list.get(&id) {
                if !self.tree_entries.contains_key(&id) {
                    let item = item.clone();
                    self.insert_into_search_tree(id, &item);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::Layer;
    use crate::rules::ClearanceMatrix;

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

    fn trace_polyline(points: &[(i32, i32)]) -> Polyline {
        let pts: Vec<IntPoint> = points.iter().map(|(x, y)| IntPoint::new(*x, *y)).collect();
        Polyline::from_int_points(&pts)
    }

    fn query_box(llx: i32, lly: i32, urx: i32, ury: i32) -> TileShape {
        TileShape::Box(IntBox::from_coords(llx, lly, urx, ury))
    }

    #[test]
    fn sparse_through_via_can_be_reconstructed_as_an_smd_escape() {
        let stack = LayerStructure::new(vec![
            Layer::new("F.Cu", true),
            Layer::new("In1.Cu", true),
            Layer::new("B.Cu", true),
        ]);
        let matrix = ClearanceMatrix::get_default_instance(stack.clone(), 200);
        let mut rules = BoardRules::new(stack.clone(), matrix);
        let class = rules.get_default_net_class();
        let net = rules.nets.add("N", class, false);
        let shape = TileShape::Box(IntBox::from_coords(-400, -400, 400, 400));
        let mut padstacks = Padstacks::new(3);
        let sparse = padstacks.add(
            "sparse",
            vec![Some(shape.clone()), None, Some(shape.clone())],
            false,
            false,
        );
        let smd = padstacks.add("smd", vec![Some(shape), None, None], false, false);
        let mut board = BasicBoard::new(stack, rules, padstacks);
        let pin = board.insert_via(smd, IntPoint::new(0, 0), vec![net], 1, false);
        board.set_component_no(pin, 1);

        assert_eq!(
            board.pure_smd_escape_layer(net, sparse, IntPoint::new(0, 0)),
            Some(0)
        );
    }

    #[test]
    fn sparse_via_does_not_contact_trace_on_empty_intermediate_layer() {
        let stack = LayerStructure::new(vec![
            Layer::new("F.Cu", true),
            Layer::new("In1.Cu", true),
            Layer::new("B.Cu", true),
        ]);
        let matrix = ClearanceMatrix::get_default_instance(stack.clone(), 200);
        let mut rules = BoardRules::new(stack.clone(), matrix);
        let class = rules.get_default_net_class();
        let net = rules.nets.add("N", class, false);
        let mut padstacks = Padstacks::new(3);
        let shape = TileShape::Box(IntBox::from_coords(-400, -400, 400, 400));
        let sparse = padstacks.add(
            "sparse",
            vec![Some(shape.clone()), None, Some(shape)],
            true,
            false,
        );
        let mut board = BasicBoard::new(stack, rules, padstacks);
        let via = board.insert_via(sparse, IntPoint::new(0, 0), vec![net], 1, false);
        let trace = board.insert_trace(trace_polyline(&[(0, 0), (5000, 0)]), 1, 100, vec![net], 1);

        assert!(board.get_normal_contacts(trace).is_empty());
        assert!(board.get_normal_contacts(via).is_empty());
        assert!(!board.net_is_completely_connected(net));
    }

    #[test]
    fn split_preserves_fixed_state() {
        let mut board = test_board();
        let trace = board.insert_trace(trace_polyline(&[(0, 0), (10000, 0)]), 0, 100, vec![1], 1);
        board.set_fixed_state(trace, crate::board::FixedState::UserFixed);
        assert!(board.split_traces_at(IntPoint::new(5000, 0), 0, 1));
        let pieces: Vec<_> = board
            .items()
            .filter(|(_, i)| matches!(i.kind, ItemKind::PolylineTrace(_)))
            .collect();
        assert_eq!(pieces.len(), 2);
        assert!(
            pieces
                .iter()
                .all(|(_, i)| i.base.fixed_state == crate::board::FixedState::UserFixed),
            "split pieces must keep the protected state"
        );
    }

    #[test]
    fn split_preserves_clearance_provenance() {
        let mut board = test_board();
        let original = board.insert_trace_with_provenance(
            trace_polyline(&[(0, 0), (10000, 0)]),
            0,
            100,
            vec![1],
            1,
            true,
        );
        assert!(board.split_traces_at(IntPoint::new(5000, 0), 0, 1));
        let pieces: Vec<_> = board
            .items()
            .filter(|(_, i)| {
                i.base.id_no != original && matches!(i.kind, ItemKind::PolylineTrace(_))
            })
            .collect();
        assert_eq!(pieces.len(), 2);
        assert!(pieces.iter().all(|(_, i)| i.base.clearance_class_explicit));
    }

    #[test]
    fn metadata_changes_are_undoable() {
        // fixed/component/obstacle flags decide what the router may touch;
        // mutating them past a snapshot must be restored by undo, not leak
        let mut board = test_board();
        let trace = board.insert_trace(trace_polyline(&[(0, 0), (10000, 0)]), 0, 100, vec![1], 1);
        board.generate_snapshot();
        board.set_fixed_state(trace, crate::board::FixedState::ShoveFixed);
        board.set_component_no(trace, 7);
        assert!(board.get_item(trace).unwrap().base.is_shove_fixed());
        assert_eq!(board.get_item(trace).unwrap().base.component_no, 7);
        assert!(board.undo());
        let base = &board.get_item(trace).unwrap().base;
        assert!(
            !base.is_shove_fixed(),
            "undo must restore the fixed state changed after the snapshot"
        );
        assert_eq!(
            base.component_no, 0,
            "undo must restore the component flag changed after the snapshot"
        );
    }

    #[test]
    fn traces_with_different_protection_do_not_combine() {
        // absorbing a ShoveFixed trace into an Unfixed one (or vice versa)
        // would silently change what shove/ripup may touch
        let mut board = test_board();
        let a = board.insert_trace(trace_polyline(&[(0, 0), (5000, 0)]), 0, 100, vec![1], 1);
        let b = board.insert_trace(
            trace_polyline(&[(5000, 0), (5000, 5000)]),
            0,
            100,
            vec![1],
            1,
        );
        board.set_fixed_state(b, crate::board::FixedState::ShoveFixed);
        let combined = board.combine_trace(a);
        assert_eq!(combined, a, "protection mismatch must prevent combining");
        assert!(board.get_item(b).is_some(), "the protected trace survives");
    }

    #[test]
    fn obstacle_conduction_area_blocks_foreign_nets() {
        use crate::geometry::planar::{PolygonShape, PolylineArea};
        let mut board = test_board();
        let area = PolylineArea::new(
            PolygonShape::from_int_points(&[
                IntPoint::new(0, 0),
                IntPoint::new(5000, 0),
                IntPoint::new(5000, 5000),
                IntPoint::new(0, 5000),
            ]),
            Vec::new(),
        );
        let id = board.insert_area(area, 0, "plane", vec![1], 1, true);
        let probe = query_box(2000, 2000, 3000, 3000);
        // a plain conduction area never blocks (fabrication cutouts)
        assert!(!board.is_blocked(&probe, 0, 2));
        // flagged an obstacle (Java ConductionArea.is_obstacle), it blocks
        // foreign nets but not its own
        board.set_area_is_obstacle(id, true);
        assert!(board.is_blocked(&probe, 0, 2));
        assert!(!board.is_blocked(&probe, 0, 1));
    }

    #[test]
    fn via_over_passthrough_trace_registers_contact_after_split() {
        let mut board = test_board();
        // a same-net trace running straight through (5000,0), not ending there
        let trace = board.insert_trace(trace_polyline(&[(0, 0), (10000, 0)]), 0, 100, vec![1], 1);
        let via = board.insert_via(1, IntPoint::new(5000, 0), vec![1], 1, false);
        // before splitting, the pass-through trace does not contact the via
        // (contact requires a trace endpoint inside the via shape)
        assert!(
            !board.get_normal_contacts(via).contains(&trace),
            "pass-through trace should not yet register a via contact"
        );
        // splitting at the via center cuts the trace so an endpoint lands there
        assert!(board.split_traces_at(IntPoint::new(5000, 0), 0, 1));
        // now the via is connected to the (split) trace pieces
        let connected = board.get_connected_set(via, 1);
        let trace_pieces = connected
            .iter()
            .filter(|&&id| {
                matches!(
                    board.get_item(id).map(|i| &i.kind),
                    Some(ItemKind::PolylineTrace(_))
                )
            })
            .count();
        assert!(
            trace_pieces >= 1,
            "via must connect to the split trace pieces"
        );
    }

    #[test]
    fn insert_query_remove_items() {
        let mut board = test_board();
        let trace = board.insert_trace(trace_polyline(&[(0, 0), (5000, 0)]), 0, 100, vec![1], 1);
        let via = board.insert_via(1, IntPoint::new(5000, 0), vec![1], 1, false);
        assert_eq!(board.item_count(), 2);

        // query around the middle of the trace on layer 0
        let hits = board.overlapping_items(&query_box(2000, -50, 3000, 50), Some(0));
        assert_eq!(hits, vec![trace]);
        // the via is on both layers
        let hits = board.overlapping_items(&query_box(4800, -50, 5200, 50), Some(1));
        assert_eq!(hits, vec![via]);
        let hits = board.overlapping_items(&query_box(4800, -50, 5200, 50), None);
        assert_eq!(hits, vec![trace, via]);
        // nothing far away
        assert!(board
            .overlapping_items(&query_box(20000, 20000, 21000, 21000), None)
            .is_empty());

        // net filtering and blocking
        assert_eq!(
            board.overlapping_items_of_net(&query_box(2000, -50, 3000, 50), 0, 1),
            vec![trace]
        );
        assert!(!board.is_blocked(&query_box(2000, -50, 3000, 50), 0, 1));
        assert!(board.is_blocked(&query_box(2000, -50, 3000, 50), 0, 2));

        // bounding box covers trace and via pads
        assert_eq!(
            board.bounding_box(),
            IntBox::from_coords(-100, -400, 5400, 400)
        );

        assert!(board.remove_item(trace));
        assert!(!board.remove_item(trace));
        assert_eq!(board.item_count(), 1);
        assert!(board
            .overlapping_items(&query_box(2000, -50, 3000, 50), Some(0))
            .is_empty());
    }

    #[test]
    fn exact_shape_check_filters_bounding_octagon_hits() {
        let mut board = test_board();
        // a diagonal trace: its bounding octagon covers the corner area,
        // but the exact shape does not
        board.insert_trace(trace_polyline(&[(0, 0), (4000, 4000)]), 0, 100, vec![1], 1);
        // a box near the diagonal but not touching the trace shape
        let far_corner = query_box(3000, 0, 3400, 400);
        assert!(board.overlapping_items(&far_corner, Some(0)).is_empty());
        // a box crossing the diagonal
        let on_diag = query_box(1900, 1900, 2100, 2100);
        assert_eq!(board.overlapping_items(&on_diag, Some(0)).len(), 1);
    }

    #[test]
    fn connectivity_contacts_and_connected_sets() {
        let mut board = test_board();
        assert!(
            !board.net_is_completely_connected(1),
            "a net with no physical item must not pass the completion gate"
        );
        // net 1: pin-like via at (0,0), trace to (5000,0), via there,
        // trace on layer 1 onwards
        let via_a = board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        let trace_1 = board.insert_trace(trace_polyline(&[(0, 0), (5000, 0)]), 0, 100, vec![1], 1);
        let via_b = board.insert_via(1, IntPoint::new(5000, 0), vec![1], 1, false);
        let trace_2 = board.insert_trace(
            trace_polyline(&[(5000, 0), (5000, 4000)]),
            1,
            100,
            vec![1],
            1,
        );
        // an unrelated trace of another net crossing nearby
        let foreign = board.insert_trace(
            trace_polyline(&[(2000, -3000), (2000, 3000)]),
            1,
            100,
            vec![2],
            1,
        );

        // trace_1 contacts both vias
        assert_eq!(board.get_normal_contacts(trace_1), vec![via_a, via_b]);
        assert_eq!(board.get_start_contacts(trace_1), vec![via_a]);
        assert_eq!(board.get_end_contacts(trace_1), vec![via_b]);
        assert!(!board.is_tail(trace_1));
        // trace_2 ends in the air
        assert_eq!(board.get_normal_contacts(trace_2), vec![via_b]);
        assert!(board.is_tail(trace_2));
        // via_b joins layers: contacts on both layers
        assert_eq!(board.get_normal_contacts(via_b), vec![trace_1, trace_2]);
        // foreign net crossing shares no contact
        assert!(board.get_normal_contacts(foreign).is_empty());

        // the whole net 1 is one connected set
        let connected = board.get_connected_set(via_a, 1);
        assert_eq!(connected, vec![via_a, trace_1, via_b, trace_2]);
        assert!(board.net_is_completely_connected(1));

        // an isolated stub of net 1 breaks completeness
        let stub = board.insert_trace(
            trace_polyline(&[(9000, 9000), (9500, 9000)]),
            0,
            100,
            vec![1],
            1,
        );
        assert!(!board.net_is_completely_connected(1));
        board.remove_item(stub);
        assert!(board.net_is_completely_connected(1));
        // removing the middle trace splits the net
        board.remove_item(trace_1);
        assert!(!board.net_is_completely_connected(1));
        assert_eq!(board.get_connected_set(via_b, 1), vec![via_b, trace_2]);
    }

    #[test]
    fn areas_as_keepouts_and_planes() {
        use crate::geometry::planar::{PolygonShape, PolylineArea};
        let mut board = test_board();
        // an L-shaped keepout (no net) on layer 0
        let keepout_shape = PolylineArea::new(
            PolygonShape::from_int_points(&[
                IntPoint::new(1000, 1000),
                IntPoint::new(3000, 1000),
                IntPoint::new(3000, 2000),
                IntPoint::new(2000, 2000),
                IntPoint::new(2000, 3000),
                IntPoint::new(1000, 3000),
            ]),
            vec![],
        );
        let keepout = board.insert_area(keepout_shape, 0, "keepout", vec![], 1, false);
        // blocks every net inside its shape
        assert!(board.is_blocked(&query_box(1100, 1100, 1200, 1200), 0, 1));
        // but not in the concave notch
        assert!(!board.is_blocked(&query_box(2500, 2500, 2600, 2600), 0, 1));
        // and not on the other layer
        assert!(!board.is_blocked(&query_box(1100, 1100, 1200, 1200), 1, 1));
        assert!(!board.get_item(keepout).unwrap().is_connectable());

        // a conduction plane of net 7 on layer 1
        let plane_shape = PolylineArea::new(
            PolygonShape::from_int_points(&[
                IntPoint::new(0, 0),
                IntPoint::new(10000, 0),
                IntPoint::new(10000, 10000),
                IntPoint::new(0, 10000),
            ]),
            vec![],
        );
        let plane = board.insert_area(plane_shape, 1, "gnd_plane", vec![7], 1, true);
        assert!(board.get_item(plane).unwrap().is_connectable());
        // a trace of net 7 ending inside the plane contacts it
        let trace = board.insert_trace(
            trace_polyline(&[(4000, 4000), (6000, 4000)]),
            1,
            100,
            vec![7],
            1,
        );
        assert_eq!(board.get_normal_contacts(trace), vec![plane]);
        assert!(!board.is_tail(trace));
        assert!(board.net_is_completely_connected(7));
    }

    #[test]
    fn via_only_keepout_does_not_block_trace_queries() {
        use crate::geometry::planar::{PolygonShape, PolylineArea};
        let mut board = test_board();
        let area = PolylineArea::new(
            PolygonShape::from_int_points(&[
                IntPoint::new(1000, 1000),
                IntPoint::new(3000, 1000),
                IntPoint::new(3000, 3000),
                IntPoint::new(1000, 3000),
            ]),
            Vec::new(),
        );
        let id = board.insert_area(area, 0, "via keepout", Vec::new(), 1, false);
        board.set_area_via_only(id, true);
        assert!(
            !board.is_blocked(&query_box(1500, 1500, 1600, 1600), 0, 1),
            "via-only keepouts constrain drills, not trace routing"
        );
    }

    #[test]
    fn split_trace_makes_t_junction_connect() {
        let mut board = test_board();
        // trace A along the x axis, trace B ending in the middle of A
        let a = board.insert_trace(trace_polyline(&[(0, 0), (10000, 0)]), 0, 100, vec![1], 1);
        let b = board.insert_trace(
            trace_polyline(&[(5000, 5000), (5000, 0)]),
            0,
            100,
            vec![1],
            1,
        );
        // without splitting the T-junction does not register
        assert!(board.get_normal_contacts(b).is_empty());
        assert!(!board.net_is_completely_connected(1));

        assert!(board.split_traces_at(IntPoint::new(5000, 0), 0, 1));
        assert!(board.get_item(a).is_none(), "original trace replaced");
        assert_eq!(board.item_count(), 3);
        // now B contacts both halves of A and the net is connected
        assert_eq!(board.get_normal_contacts(b).len(), 2);
        assert!(board.net_is_completely_connected(1));

        // splitting at an endpoint or off the trace does nothing
        assert!(!board.split_traces_at(IntPoint::new(0, 0), 0, 1));
        assert!(!board.split_traces_at(IntPoint::new(4000, 100), 0, 1));
    }

    #[test]
    fn snapshot_fuzz_against_shadow_model() {
        // random insert/remove/generate/pop/undo sequences must keep the
        // board's alive set identical to a trivial shadow model (found
        // necessary after tree-sees-nothing evidence implied an undo
        // splice leak deeper than the hand-written nesting test)
        for seed in 1u64..40 {
            let mut board = test_board();
            let mut rng = seed;
            let mut next = || {
                rng = rng
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                (rng >> 33) as usize
            };
            // shadow: stack of saved alive-sets; current alive set
            let mut saved: Vec<Vec<ItemId>> = Vec::new();
            let mut alive: Vec<ItemId> = Vec::new();
            let mut y = 0i32;
            for _step in 0..200 {
                match next() % 10 {
                    0..=3 => {
                        y += 300;
                        let id = board.insert_trace(
                            trace_polyline(&[(0, y), (2000, y)]),
                            0,
                            100,
                            vec![1],
                            1,
                        );
                        alive.push(id);
                    }
                    4..=5 => {
                        if !alive.is_empty() {
                            let idx = next() % alive.len();
                            let id = alive.remove(idx);
                            assert!(board.remove_item(id), "shadow said alive");
                        }
                    }
                    6..=7 => {
                        board.generate_snapshot();
                        saved.push(alive.clone());
                    }
                    8 => {
                        if !saved.is_empty() {
                            board.pop_snapshot();
                            saved.pop(); // changes kept
                        }
                    }
                    _ => {
                        if !saved.is_empty() {
                            board.undo();
                            alive = saved.pop().unwrap();
                        }
                    }
                }
                let mut board_alive: Vec<ItemId> = board.items().map(|(id, _)| *id).collect();
                board_alive.sort_unstable();
                let mut shadow = alive.clone();
                shadow.sort_unstable();
                assert_eq!(
                    board_alive, shadow,
                    "divergence at seed {seed} step {_step}"
                );
                // the search tree must agree with the item list
                let mut tree_view = board.overlapping_items(
                    &TileShape::Box(IntBox::from_coords(-1000, -1000, 3000, 100_000)),
                    Some(0),
                );
                tree_view.sort_unstable();
                assert_eq!(
                    tree_view, shadow,
                    "TREE divergence at seed {seed} step {_step}"
                );
            }
        }
    }

    #[test]
    fn nested_snapshot_pop_then_undo_restores_exactly() {
        // the restart fallback snapshots the board, and shove_aside runs
        // its own snapshot/pop INSIDE that scope: after the inner commit
        // and the outer undo, the board must be exactly the pre-restart
        // state (leaked inner items were the round-two DRC suspect)
        let mut board = test_board();
        let a = board.insert_trace(trace_polyline(&[(0, 0), (1000, 0)]), 0, 100, vec![1], 1);

        board.generate_snapshot(); // restart level
        let b = board.insert_trace(trace_polyline(&[(0, 500), (1000, 500)]), 0, 100, vec![2], 1);
        board.remove_item(a);

        board.generate_snapshot(); // shove level
        let c = board.insert_trace(trace_polyline(&[(0, 900), (1000, 900)]), 0, 100, vec![3], 1);
        board.remove_item(b);
        board.pop_snapshot(); // shove commits into the restart level

        assert!(board.get_item(c).is_some());
        assert!(board.get_item(b).is_none());

        board.undo(); // restart rolls back

        assert!(board.get_item(a).is_some(), "pre-restart item lost");
        assert!(board.get_item(b).is_none(), "restart item leaked");
        assert!(board.get_item(c).is_none(), "inner (shove) item leaked");
        assert_eq!(board.items().count(), 1, "exactly the original item");
        // the search tree must agree with the item list
        let hits = board.overlapping_items(
            &TileShape::Box(IntBox::from_coords(-100, -1100, 1100, 1100)),
            Some(0),
        );
        assert_eq!(hits, vec![a], "search tree out of sync after undo");
    }

    #[test]
    fn cycle_traces_are_removed() {
        let mut board = test_board();
        // two pads (component-tagged vias) joined by a direct trace
        let a = board.insert_via(1, IntPoint::new(0, 0), vec![1], 1, false);
        board.set_component_no(a, 1);
        let b = board.insert_via(1, IntPoint::new(8000, 0), vec![1], 1, false);
        board.set_component_no(b, 2);
        let direct = board.insert_trace(trace_polyline(&[(0, 0), (8000, 0)]), 0, 100, vec![1], 1);
        // a redundant detour between the same pads: a parallel path
        let d1 = board.insert_trace(
            trace_polyline(&[(0, 0), (0, 4000), (8000, 4000)]),
            0,
            100,
            vec![1],
            1,
        );
        let _d2 = board.insert_trace(
            trace_polyline(&[(8000, 4000), (8000, 0)]),
            0,
            100,
            vec![1],
            1,
        );
        assert!(board.trace_is_cycle(direct), "parallel paths make a cycle");
        assert!(board.remove_if_cycle(d1), "the detour is a removable cycle");
        // the detour's second piece became a tail and is gone too
        assert!(
            board.get_item(_d2).is_none(),
            "the freed tail must be cleaned up"
        );
        // the direct trace survives and the net stays connected
        assert!(board.get_item(direct).is_some());
        assert!(board.net_is_completely_connected(1));
        // and the survivor is no longer a cycle
        assert!(!board.trace_is_cycle(direct));
    }

    #[test]
    fn combine_traces_at_simple_joints() {
        let mut board = test_board();
        // a chain of three traces sharing endpoints
        let t1 = board.insert_trace(trace_polyline(&[(0, 0), (4000, 0)]), 0, 100, vec![1], 1);
        let _t2 = board.insert_trace(
            trace_polyline(&[(4000, 0), (4000, 4000)]),
            0,
            100,
            vec![1],
            1,
        );
        let _t3 = board.insert_trace(
            trace_polyline(&[(4000, 4000), (8000, 4000)]),
            0,
            100,
            vec![1],
            1,
        );
        assert_eq!(board.item_count(), 3);
        let combined = board.combine_trace(t1);
        assert_eq!(board.item_count(), 1);
        let item = board.get_item(combined).unwrap();
        let ItemKind::PolylineTrace(t) = &item.kind else {
            panic!("not a trace")
        };
        assert_eq!(t.corner_count(), 4);
        let first = t.first_corner();
        let last = t.last_corner();
        let expected_ends = [
            Point::Int(IntPoint::new(0, 0)),
            Point::Int(IntPoint::new(8000, 4000)),
        ];
        assert!(expected_ends.contains(&first) && expected_ends.contains(&last) && first != last);

        // a via at the joint prevents combining (two contacts there)
        let t4 = board.insert_trace(
            trace_polyline(&[(8000, 4000), (12000, 4000)]),
            0,
            100,
            vec![1],
            1,
        );
        board.insert_via(1, IntPoint::new(8000, 4000), vec![1], 1, false);
        let survivor = board.combine_trace(t4);
        assert_eq!(survivor, t4, "combined across a via junction");

        // different half widths do not combine
        let t5 = board.insert_trace(
            trace_polyline(&[(12000, 4000), (16000, 4000)]),
            0,
            200,
            vec![1],
            1,
        );
        assert_eq!(board.combine_trace(t5), t5);
    }

    #[test]
    fn combine_does_not_drop_a_trace_clearance_class() {
        let mut board = test_board();
        assert!(board.rules.clearance_matrix.append_class("strict"));
        let strict = board.rules.clearance_matrix.get_no("strict").unwrap();
        let first = board.insert_trace(
            trace_polyline(&[(0, 0), (4000, 0)]),
            0,
            100,
            vec![1],
            strict,
        );
        let second =
            board.insert_trace(trace_polyline(&[(4000, 0), (8000, 0)]), 0, 100, vec![1], 1);
        assert_eq!(board.combine_trace(first), first);
        assert!(board.get_item(first).is_some());
        assert!(board.get_item(second).is_some());
    }

    #[test]
    fn combine_does_not_merge_mixed_clearance_provenance() {
        let mut board = test_board();
        let first = board.insert_trace_with_provenance(
            trace_polyline(&[(0, 0), (4000, 0)]),
            0,
            100,
            vec![1],
            1,
            true,
        );
        let second =
            board.insert_trace(trace_polyline(&[(4000, 0), (8000, 0)]), 0, 100, vec![1], 1);
        assert_eq!(board.combine_trace(first), first);
        assert!(board.get_item(first).is_some());
        assert!(board.get_item(second).is_some());
    }

    #[test]
    fn undo_redo_resyncs_search_tree() {
        let mut board = test_board();
        let trace = board.insert_trace(trace_polyline(&[(0, 0), (5000, 0)]), 0, 100, vec![1], 1);
        board.generate_snapshot();
        let via = board.insert_via(1, IntPoint::new(2500, 0), vec![1], 1, false);
        board.remove_item(trace);
        assert_eq!(board.item_count(), 1);
        let query = query_box(2400, -50, 2600, 50);
        assert_eq!(board.overlapping_items(&query, Some(0)), vec![via]);

        // undo: the trace comes back, the via disappears
        assert!(board.undo());
        assert_eq!(board.item_count(), 1);
        assert_eq!(board.overlapping_items(&query, Some(0)), vec![trace]);

        // redo: the via returns, the trace is deleted again
        assert!(board.redo());
        assert_eq!(board.item_count(), 1);
        assert_eq!(board.overlapping_items(&query, Some(0)), vec![via]);
        assert!(!board.redo());
    }

    #[test]
    fn rollback_snapshot_resyncs_search_tree_without_redo() {
        let mut board = test_board();
        let trace = board.insert_trace(trace_polyline(&[(0, 0), (5000, 0)]), 0, 100, vec![1], 1);
        board.generate_snapshot();
        let via = board.insert_via(1, IntPoint::new(2500, 0), vec![1], 1, false);
        board.remove_item(trace);
        assert_eq!(board.item_count(), 1);

        assert!(board.rollback_snapshot());
        assert_eq!(board.item_count(), 1);
        let query = query_box(2400, -50, 2600, 50);
        assert_eq!(board.overlapping_items(&query, Some(0)), vec![trace]);
        assert!(!board.redo());
        assert!(board.get_item(via).is_none());
    }
}
