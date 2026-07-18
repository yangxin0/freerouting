//! Format-independent validation of a board's in-memory reference graph.
//!
//! Importers and exporters have additional format-specific constraints. This
//! module deliberately checks only invariants shared by the board model so it
//! can be used at every interchange boundary without rejecting valid DSN
//! subnets, multi-net pins, sparse via padstacks, or netless keepouts.

use std::collections::HashSet;
use std::error::Error;
use std::fmt;

use crate::board::{BasicBoard, ItemKind};
use crate::geometry::planar::FloatPoint;
use crate::rules::ItemClass;

const ITEM_CLASSES: [ItemClass; ItemClass::COUNT] = [
    ItemClass::None,
    ItemClass::Trace,
    ItemClass::Via,
    ItemClass::Pin,
    ItemClass::Smd,
    ItemClass::Area,
];

/// A format-independent board invariant failure.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BoardValidationError {
    /// Stable, human-readable location in the board reference graph.
    pub path: String,
    /// Description of the violated invariant.
    pub message: String,
}

impl BoardValidationError {
    fn new(path: impl Into<String>, message: impl Into<String>) -> Self {
        Self {
            path: path.into(),
            message: message.into(),
        }
    }
}

impl fmt::Display for BoardValidationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {}", self.path, self.message)
    }
}

impl Error for BoardValidationError {}

/// Validates the common, format-independent references and geometry of a
/// fully constructed board.
pub fn validate_board_references(board: &BasicBoard) -> Result<(), BoardValidationError> {
    validate_board_metadata(board)?;
    validate_layers(board)?;
    validate_clearance_matrix(board)?;
    validate_padstacks(board)?;
    validate_vias(board)?;
    validate_net_classes(board)?;
    validate_nets(board)?;
    validate_items(board)?;
    validate_outline(board)
}

fn validate_board_metadata(board: &BasicBoard) -> Result<(), BoardValidationError> {
    if board.resolution <= 0 {
        return Err(BoardValidationError::new(
            "resolution",
            format!("must be positive, got {}", board.resolution),
        ));
    }
    let unit = board.unit.as_str();
    if unit.is_empty() {
        return Err(BoardValidationError::new("unit", "must not be empty"));
    }
    if !matches!(
        unit.to_ascii_lowercase().as_str(),
        "um" | "micron" | "microns" | "mil" | "inch" | "in" | "mm" | "cm"
    ) {
        return Err(BoardValidationError::new(
            "unit",
            format!("unsupported physical unit {unit:?}"),
        ));
    }
    Ok(())
}

fn validate_layers(board: &BasicBoard) -> Result<(), BoardValidationError> {
    let layer_count = board.layer_structure.layer_count();
    if layer_count == 0 {
        return Err(BoardValidationError::new(
            "layers",
            "board must contain at least one layer",
        ));
    }
    validate_unique_names(
        "layers",
        board
            .layer_structure
            .arr
            .iter()
            .enumerate()
            .map(|(i, layer)| (i, layer.name.as_str())),
    )?;

    let counts = [
        ("rules.layers", board.rules.layer_structure().layer_count()),
        (
            "clearance_matrix.layers",
            board.rules.clearance_matrix.get_layer_count(),
        ),
        ("padstacks.layers", board.padstacks.board_layer_count),
    ];
    for (path, actual) in counts {
        if actual != layer_count {
            return Err(BoardValidationError::new(
                path,
                format!("layer count {actual} does not match board layer count {layer_count}"),
            ));
        }
    }
    for (path, layers) in [
        ("rules.layers", board.rules.layer_structure()),
        (
            "clearance_matrix.layers",
            board.rules.clearance_matrix.layer_structure(),
        ),
    ] {
        for (index, (board_layer, rule_layer)) in board
            .layer_structure
            .arr
            .iter()
            .zip(&layers.arr)
            .enumerate()
        {
            if board_layer != rule_layer {
                return Err(BoardValidationError::new(
                    format!("{path}[{index}]"),
                    format!(
                        "layer identity {:?}/{:?} does not match board {:?}/{:?}",
                        rule_layer.name,
                        rule_layer.is_signal,
                        board_layer.name,
                        board_layer.is_signal
                    ),
                ));
            }
        }
    }
    Ok(())
}

fn validate_clearance_matrix(board: &BasicBoard) -> Result<(), BoardValidationError> {
    let matrix = &board.rules.clearance_matrix;
    if matrix.get_class_count() == 0 {
        return Err(BoardValidationError::new(
            "clearance_matrix",
            "must contain at least one class",
        ));
    }
    validate_unique_names(
        "clearance_matrix.classes",
        (0..matrix.get_class_count()).map(|i| (i, matrix.get_name(i).unwrap_or(""))),
    )?;
    let limit = crate::geometry::planar::limits::CRIT_INT;
    for row in 0..matrix.get_class_count() {
        for column in 0..matrix.get_class_count() {
            for layer in 0..matrix.get_layer_count() {
                let value = matrix.get_value(row, column, layer, false);
                if value > limit {
                    return Err(BoardValidationError::new(
                        format!("clearance_matrix[{row}][{column}][{layer}]"),
                        format!("exceeds geometry limit {limit}, got {value}"),
                    ));
                }
            }
        }
    }
    Ok(())
}

fn validate_padstacks(board: &BasicBoard) -> Result<(), BoardValidationError> {
    validate_unique_names(
        "padstacks",
        (1..=board.padstacks.count()).map(|no| {
            let padstack = board
                .padstacks
                .get_by_no(no)
                .expect("padstack number from its own count");
            (no, padstack.name.as_str())
        }),
    )?;

    for no in 1..=board.padstacks.count() {
        let padstack = board
            .padstacks
            .get_by_no(no)
            .expect("padstack number from its own count");
        let path = format!("padstacks[{no}]");
        if padstack.no != no {
            return Err(BoardValidationError::new(
                format!("{path}.no"),
                format!("stored number {} does not match index {no}", padstack.no),
            ));
        }
        if padstack.board_layer_count() != board.layer_structure.layer_count() {
            return Err(BoardValidationError::new(
                format!("{path}.layers"),
                format!(
                    "shape vector has {} layers but the board has {}",
                    padstack.board_layer_count(),
                    board.layer_structure.layer_count()
                ),
            ));
        }

        let mut populated_layers = Vec::new();
        for layer in 0..board.layer_structure.layer_count() {
            let Some(shape) = padstack.get_shape(layer) else {
                continue;
            };
            populated_layers.push(layer);
            if shape.is_empty() || !shape.is_bounded() || shape.dimension() != 2 {
                return Err(BoardValidationError::new(
                    format!("{path}.shapes[{layer}]"),
                    "shape must be nonempty, bounded, and two-dimensional",
                ));
            }
        }
        let Some((&first, &last)) = populated_layers.first().zip(populated_layers.last()) else {
            return Err(BoardValidationError::new(
                format!("{path}.shapes"),
                "must contain at least one two-dimensional shape",
            ));
        };
        if padstack.get_shape(first).is_none() || padstack.get_shape(last).is_none() {
            return Err(BoardValidationError::new(
                format!("{path}.shapes"),
                "transition endpoints must contain shapes",
            ));
        }
    }
    Ok(())
}

fn validate_vias(board: &BasicBoard) -> Result<(), BoardValidationError> {
    validate_unique_names(
        "via_infos",
        (0..board.rules.via_infos.count()).map(|i| {
            let info = board.rules.via_infos.get(i);
            (i, info.get_name())
        }),
    )?;
    for i in 0..board.rules.via_infos.count() {
        let info = board.rules.via_infos.get(i);
        if board.padstacks.get_by_no(info.get_padstack()).is_none() {
            return Err(BoardValidationError::new(
                format!("via_infos[{i}].padstack"),
                format!("invalid padstack number {}", info.get_padstack()),
            ));
        }
        validate_clearance_class(
            board,
            format!("via_infos[{i}].clearance_class"),
            info.get_clearance_class(),
        )?;
    }

    validate_unique_names(
        "via_rules",
        board
            .rules
            .via_rules
            .iter()
            .enumerate()
            .map(|(i, rule)| (i, rule.name.as_str())),
    )?;
    for (i, rule) in board.rules.via_rules.iter().enumerate() {
        for (position, &via_info) in rule.vias().iter().enumerate() {
            if via_info >= board.rules.via_infos.count() {
                return Err(BoardValidationError::new(
                    format!("via_rules[{i}].vias[{position}]"),
                    format!("invalid via-info index {via_info}"),
                ));
            }
        }
    }
    Ok(())
}

fn validate_net_classes(board: &BasicBoard) -> Result<(), BoardValidationError> {
    validate_unique_names(
        "net_classes",
        (0..board.rules.net_classes.count()).map(|i| {
            let class = board.rules.net_classes.get(i);
            (i, class.get_name())
        }),
    )?;

    let layer_count = board.layer_structure.layer_count();
    for i in 0..board.rules.net_classes.count() {
        let class = board.rules.net_classes.get(i);
        let path = format!("net_classes[{i}]");
        if class.layer_count() != layer_count {
            return Err(BoardValidationError::new(
                format!("{path}.layers"),
                format!(
                    "layer count {} does not match board layer count {layer_count}",
                    class.layer_count()
                ),
            ));
        }
        let minimum_length = class.get_minimum_trace_length();
        let maximum_length = class.get_maximum_trace_length();
        if !minimum_length.is_finite()
            || !maximum_length.is_finite()
            || minimum_length < 0.0
            || maximum_length < 0.0
            || (maximum_length > 0.0 && minimum_length > maximum_length)
        {
            return Err(BoardValidationError::new(
                format!("{path}.trace_length_limits"),
                format!(
                    "lengths must be finite and nonnegative, and a bounded maximum must not be smaller than the minimum; got min={minimum_length}, max={maximum_length}"
                ),
            ));
        }
        validate_clearance_class(
            board,
            format!("{path}.trace_clearance_class"),
            class.get_trace_clearance_class(),
        )?;
        for item_class in ITEM_CLASSES {
            validate_clearance_class(
                board,
                format!("{path}.item_clearance_class.{item_class:?}"),
                class.default_item_clearance_classes.get(item_class),
            )?;
        }
        if let Some(via_rule) = class.get_via_rule() {
            if via_rule >= board.rules.via_rules.len() {
                return Err(BoardValidationError::new(
                    format!("{path}.via_rule"),
                    format!("invalid via-rule index {via_rule}"),
                ));
            }
        }
        for layer in 0..layer_count {
            let width = class.get_trace_half_width(layer);
            if !(0..=crate::geometry::planar::limits::CRIT_INT).contains(&width)
                || (class.is_active_routing_layer(layer) && width == 0)
            {
                return Err(BoardValidationError::new(
                    format!("{path}.trace_half_width[{layer}]"),
                    format!("active routing layers require a positive width, got {width}"),
                ));
            }
        }
    }
    if !board.rules.get_pin_edge_to_turn_dist().is_finite()
        || board.rules.get_pin_edge_to_turn_dist() < 0.0
        || board.rules.get_pin_edge_to_turn_dist()
            > f64::from(crate::geometry::planar::limits::CRIT_INT)
    {
        return Err(BoardValidationError::new(
            "rules.pin_edge_to_turn_dist",
            "must be finite and nonnegative",
        ));
    }
    for (first, second, value) in board.rules.same_net_clearances() {
        if !(0..=crate::geometry::planar::limits::CRIT_INT).contains(&value) {
            return Err(BoardValidationError::new(
                format!("rules.same_net_clearance.{first:?}.{second:?}"),
                format!("must be nonnegative, got {value}"),
            ));
        }
    }
    Ok(())
}

fn validate_nets(board: &BasicBoard) -> Result<(), BoardValidationError> {
    let mut numbers = HashSet::new();
    let mut identities = HashSet::new();
    for (index, net) in board.rules.nets.iter().enumerate() {
        let expected = index as i32 + 1;
        let path = format!("nets[{expected}]");
        if net.net_number <= 0 || net.net_number != expected || !numbers.insert(net.net_number) {
            return Err(BoardValidationError::new(
                format!("{path}.net_number"),
                format!(
                    "must be unique, positive, and match its 1-based index {expected}; got {}",
                    net.net_number
                ),
            ));
        }
        if net.name.is_empty() {
            return Err(BoardValidationError::new(
                format!("{path}.name"),
                "must not be empty",
            ));
        }
        if net.subnet_number == 0 {
            return Err(BoardValidationError::new(
                format!("{path}.subnet_number"),
                "must be positive",
            ));
        }
        let identity = (net.name.to_ascii_lowercase(), net.subnet_number);
        if !identities.insert(identity) {
            return Err(BoardValidationError::new(
                path,
                format!(
                    "duplicate net identity ({:?}, {})",
                    net.name, net.subnet_number
                ),
            ));
        }
        if net.get_class() >= board.rules.net_classes.count() {
            return Err(BoardValidationError::new(
                format!("{path}.net_class"),
                format!("invalid net-class index {}", net.get_class()),
            ));
        }
    }
    for (net_no, endpoint) in board.unresolved_net_endpoints() {
        if board.rules.nets.get_by_no(net_no).is_none() {
            return Err(BoardValidationError::new(
                format!("unresolved_net_endpoints[{net_no}]"),
                "references a missing net",
            ));
        }
        if endpoint.component.is_empty() || endpoint.pin.is_empty() {
            return Err(BoardValidationError::new(
                format!("unresolved_net_endpoints[{net_no}]"),
                "contains an empty component or pin name",
            ));
        }
    }
    Ok(())
}

fn validate_items(board: &BasicBoard) -> Result<(), BoardValidationError> {
    let mut base_ids = HashSet::new();
    for (&id, item) in board.items() {
        let path = format!("items[{id}]");
        if id <= 0 || item.base.id_no != id || !base_ids.insert(item.base.id_no) {
            return Err(BoardValidationError::new(
                format!("{path}.id"),
                format!(
                    "key and base id must be the same unique positive value; base id is {}",
                    item.base.id_no
                ),
            ));
        }
        if item.base.component_no < 0 {
            return Err(BoardValidationError::new(
                format!("{path}.component_no"),
                format!("must be nonnegative, got {}", item.base.component_no),
            ));
        }
        validate_clearance_class(
            board,
            format!("{path}.clearance_class"),
            item.base.clearance_class,
        )?;
        let mut item_nets = HashSet::new();
        for (position, &net_no) in item.base.net_nos.iter().enumerate() {
            if board.rules.nets.get_by_no(net_no).is_none() {
                return Err(BoardValidationError::new(
                    format!("{path}.net_nos[{position}]"),
                    format!("invalid net number {net_no}"),
                ));
            }
            if !item_nets.insert(net_no) {
                return Err(BoardValidationError::new(
                    format!("{path}.net_nos[{position}]"),
                    format!("duplicate net number {net_no}"),
                ));
            }
        }

        match &item.kind {
            ItemKind::PolylineTrace(trace) => {
                for (line_index, line) in trace.polyline.arr.iter().enumerate() {
                    validate_coordinate(
                        format!("{path}.polyline[{line_index}].a"),
                        line.a.x,
                        line.a.y,
                    )?;
                    validate_coordinate(
                        format!("{path}.polyline[{line_index}].b"),
                        line.b.x,
                        line.b.y,
                    )?;
                }
                validate_item_layer(board, format!("{path}.layer"), trace.layer)?;
                if trace.half_width <= 0
                    || trace.half_width > crate::geometry::planar::limits::CRIT_INT
                {
                    return Err(BoardValidationError::new(
                        format!("{path}.half_width"),
                        format!("must be positive, got {}", trace.half_width),
                    ));
                }
                if trace.polyline.is_empty()
                    || trace.polyline.corner_count() < 2
                    || trace.polyline.length_approx() <= 0.0
                {
                    return Err(BoardValidationError::new(
                        format!("{path}.polyline"),
                        "must contain a nonempty route segment",
                    ));
                }
            }
            ItemKind::Via(via) => {
                validate_coordinate(format!("{path}.center"), via.center.x, via.center.y)?;
                let Some(padstack) = board.padstacks.get_by_no(via.padstack) else {
                    return Err(BoardValidationError::new(
                        format!("{path}.padstack"),
                        format!("invalid padstack number {}", via.padstack),
                    ));
                };
                match (via.is_escape_via, via.escape_smd_layer) {
                    (false, None) => {}
                    (true, Some(layer))
                        if layer < board.layer_structure.layer_count()
                            && padstack.get_shape(layer).is_some() => {}
                    (true, Some(layer)) => {
                        return Err(BoardValidationError::new(
                            format!("{path}.escape_smd_layer"),
                            format!("layer {layer} is outside or unpopulated in the via padstack"),
                        ));
                    }
                    (true, None) => {
                        return Err(BoardValidationError::new(
                            format!("{path}.escape_smd_layer"),
                            "escape via must identify its SMD layer",
                        ));
                    }
                    (false, Some(layer)) => {
                        return Err(BoardValidationError::new(
                            format!("{path}.escape_smd_layer"),
                            format!("ordinary via must not carry escape layer {layer}"),
                        ));
                    }
                }
            }
            ItemKind::ObstacleArea(area) => {
                for (corner, point) in area.area.border_shape.corners.iter().enumerate() {
                    validate_float_coordinate(
                        format!("{path}.area.border[{corner}]"),
                        point.to_float(),
                    )?;
                }
                for (hole_index, hole) in area.area.holes.iter().enumerate() {
                    for (corner, point) in hole.corners.iter().enumerate() {
                        validate_float_coordinate(
                            format!("{path}.area.holes[{hole_index}][{corner}]"),
                            point.to_float(),
                        )?;
                    }
                }
                validate_item_layer(board, format!("{path}.layer"), area.layer)?;
                if area.area.is_empty() || !area.area.is_bounded() || area.area.dimension() != 2 {
                    return Err(BoardValidationError::new(
                        format!("{path}.area"),
                        "must be nonempty, bounded, and two-dimensional",
                    ));
                }
                for (hole, shape) in area.area.get_holes().iter().enumerate() {
                    if shape.is_empty() || !shape.is_bounded() || shape.dimension() != 2 {
                        return Err(BoardValidationError::new(
                            format!("{path}.area.holes[{hole}]"),
                            "hole must be nonempty, bounded, and two-dimensional",
                        ));
                    }
                }
            }
        }
    }
    Ok(())
}

fn validate_outline(board: &BasicBoard) -> Result<(), BoardValidationError> {
    let Some((corners, clearance)) = &board.outline else {
        return Ok(());
    };
    for (index, corner) in corners.iter().enumerate() {
        validate_coordinate(format!("outline.corners[{index}]"), corner.x, corner.y)?;
    }
    if corners.len() < 3 {
        return Err(BoardValidationError::new(
            "outline.corners",
            format!("must contain at least three corners, got {}", corners.len()),
        ));
    }
    let twice_area: i128 = corners
        .iter()
        .zip(corners.iter().cycle().skip(1))
        .take(corners.len())
        .map(|(a, b)| i128::from(a.x) * i128::from(b.y) - i128::from(b.x) * i128::from(a.y))
        .sum();
    if twice_area == 0 {
        return Err(BoardValidationError::new(
            "outline.corners",
            "must enclose a two-dimensional area",
        ));
    }
    if *clearance < 0 || *clearance > crate::geometry::planar::limits::CRIT_INT {
        return Err(BoardValidationError::new(
            "outline.clearance",
            format!("must be nonnegative, got {clearance}"),
        ));
    }
    Ok(())
}

fn validate_coordinate(
    path: impl Into<String>,
    x: i32,
    y: i32,
) -> Result<(), BoardValidationError> {
    let limit = i64::from(crate::geometry::planar::limits::CRIT_INT);
    if i64::from(x).abs() > limit || i64::from(y).abs() > limit {
        return Err(BoardValidationError::new(
            path,
            format!("coordinates must lie within +/-{limit}, got ({x}, {y})"),
        ));
    }
    Ok(())
}

fn validate_float_coordinate(
    path: impl Into<String>,
    point: FloatPoint,
) -> Result<(), BoardValidationError> {
    let limit = f64::from(crate::geometry::planar::limits::CRIT_INT);
    if !point.x.is_finite()
        || !point.y.is_finite()
        || point.x.abs() > limit
        || point.y.abs() > limit
    {
        return Err(BoardValidationError::new(
            path,
            format!("coordinates must be finite and lie within +/-{limit}"),
        ));
    }
    Ok(())
}

fn validate_unique_names<'a>(
    path: &str,
    names: impl IntoIterator<Item = (usize, &'a str)>,
) -> Result<(), BoardValidationError> {
    let mut seen = HashSet::new();
    for (index, name) in names {
        if name.is_empty() {
            return Err(BoardValidationError::new(
                format!("{path}[{index}].name"),
                "must not be empty",
            ));
        }
        if !seen.insert(name.to_ascii_lowercase()) {
            return Err(BoardValidationError::new(
                format!("{path}[{index}].name"),
                format!("duplicate name {name:?}"),
            ));
        }
    }
    Ok(())
}

fn validate_clearance_class(
    board: &BasicBoard,
    path: String,
    class: usize,
) -> Result<(), BoardValidationError> {
    if class >= board.rules.clearance_matrix.get_class_count() {
        return Err(BoardValidationError::new(
            path,
            format!("invalid clearance-class index {class}"),
        ));
    }
    Ok(())
}

fn validate_item_layer(
    board: &BasicBoard,
    path: String,
    layer: usize,
) -> Result<(), BoardValidationError> {
    if layer >= board.layer_structure.layer_count() {
        return Err(BoardValidationError::new(
            path,
            format!("invalid layer index {layer}"),
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{Item, ItemBase, LayerStructure};
    use crate::core::Padstacks;
    use crate::geometry::planar::{
        IntBox, IntPoint, PolygonShape, Polyline, PolylineArea, TileShape,
    };
    use crate::rules::{BoardRules, ClearanceMatrix, ViaInfo, ViaRule};

    fn valid_board() -> BasicBoard {
        let layers = LayerStructure::signal_layers(3);
        let matrix = ClearanceMatrix::get_default_instance(layers.clone(), 200);
        let mut rules = BoardRules::new(layers.clone(), matrix);
        let class = rules.net_classes.append("default", &layers, false);
        rules.net_classes.get_mut(class).set_trace_half_width(100);
        rules
            .net_classes
            .get_mut(class)
            .set_trace_clearance_class(1);

        let mut padstacks = Padstacks::new(3);
        let shape = TileShape::Box(IntBox::from_coords(-200, -200, 200, 200));
        let padstack = padstacks.add(
            "through",
            vec![
                Some(shape.clone()),
                Some(shape.clone()),
                Some(shape.clone()),
            ],
            true,
            false,
        );
        let via_info = rules
            .via_infos
            .add(ViaInfo::new("through", padstack, 1, false))
            .unwrap();
        let mut via_rule = ViaRule::new("default");
        via_rule.append_via(via_info);
        rules.via_rules.push(via_rule);
        rules.net_classes.get_mut(class).set_via_rule(Some(0));
        let net = rules.nets.add("GND", 1, false);
        rules.nets.get_by_no_mut(net).unwrap().set_class(class);

        let mut board = BasicBoard::new(layers, rules, padstacks);
        board.insert_trace(
            Polyline::from_two_points(IntPoint::new(0, 0), IntPoint::new(1000, 0)),
            0,
            100,
            vec![net],
            1,
        );
        board.insert_via(padstack, IntPoint::new(1000, 0), vec![net], 1, false);
        // Sparse intermediate layers are legal for blind/buried padstacks.
        let sparse = board.padstacks.add(
            "sparse",
            vec![
                Some(TileShape::Box(IntBox::from_coords(-100, -100, 100, 100))),
                None,
                Some(TileShape::Box(IntBox::from_coords(-100, -100, 100, 100))),
            ],
            true,
            false,
        );
        let mut pin_base = ItemBase::new(0, Vec::new(), 1);
        pin_base.component_no = 1;
        board.insert_item(Item::new_via(
            pin_base,
            sparse,
            IntPoint::new(4000, 0),
            false,
        ));
        let keepout = PolylineArea::new(
            PolygonShape::from_int_points(&[
                IntPoint::new(2000, 2000),
                IntPoint::new(3000, 2000),
                IntPoint::new(3000, 3000),
                IntPoint::new(2000, 3000),
            ]),
            Vec::new(),
        );
        board.insert_area(keepout, 0, "keepout", Vec::new(), 1, false);
        board
    }

    fn assert_path(board: &BasicBoard, expected: &str) {
        let error = validate_board_references(board).expect_err("board should be invalid");
        assert!(
            error.path.contains(expected),
            "expected {expected:?} in {error:?}"
        );
    }

    #[test]
    fn validates_complete_board_and_allowed_model_features() {
        let mut board = valid_board();
        let gnd_subnet = board.rules.nets.add("GND", 2, false);
        board
            .rules
            .nets
            .get_by_no_mut(gnd_subnet)
            .unwrap()
            .set_class(0);
        let signal = board.rules.nets.add("SIGNAL", 1, false);
        board.rules.nets.get_by_no_mut(signal).unwrap().set_class(0);
        board.insert_via(
            1,
            IntPoint::new(4000, 0),
            vec![gnd_subnet, signal],
            1,
            false,
        );
        assert_eq!(validate_board_references(&board), Ok(()));
    }

    #[test]
    fn rejects_metadata_layer_and_name_invariants() {
        let mut board = valid_board();
        board.resolution = 0;
        assert_path(&board, "resolution");

        let mut board = valid_board();
        board.unit = "parsecs".to_string();
        assert_path(&board, "unit");

        let mut board = valid_board();
        board.layer_structure.arr[1].name = board.layer_structure.arr[0].name.to_uppercase();
        assert_path(&board, "layers[1].name");

        let mut board = valid_board();
        board.padstacks.board_layer_count = 1;
        assert_path(&board, "padstacks.layers");

        let board_layers = LayerStructure::new(vec![
            crate::board::Layer::new("F.Cu", true),
            crate::board::Layer::new("B.Cu", true),
        ]);
        let rule_layers = LayerStructure::new(vec![
            crate::board::Layer::new("Top", true),
            crate::board::Layer::new("Bottom", true),
        ]);
        let matrix = ClearanceMatrix::get_default_instance(rule_layers.clone(), 200);
        let rules = BoardRules::new(rule_layers, matrix);
        let board = BasicBoard::new(board_layers, rules, Padstacks::new(2));
        assert_path(&board, "rules.layers[0]");
    }

    #[test]
    fn rejects_rule_and_reference_invariants() {
        let mut board = valid_board();
        board.rules.via_infos.get_mut(0).set_padstack(99);
        assert_path(&board, "via_infos[0].padstack");

        let mut board = valid_board();
        board.rules.via_rules[0].append_via(99);
        assert_path(&board, "via_rules[0].vias[1]");

        let mut board = valid_board();
        board.rules.net_classes.get_mut(0).set_via_rule(Some(99));
        assert_path(&board, "net_classes[0].via_rule");

        let mut board = valid_board();
        board.rules.nets.get_by_no_mut(1).unwrap().subnet_number = 0;
        assert_path(&board, "nets[1].subnet_number");

        let mut board = valid_board();
        board
            .rules
            .net_classes
            .get_mut(0)
            .set_minimum_trace_length(f64::NAN);
        assert_path(&board, "trace_length_limits");

        let mut board = valid_board();
        board.rules.set_pin_edge_to_turn_dist(f64::INFINITY);
        assert_path(&board, "pin_edge_to_turn_dist");

        let mut board = valid_board();
        board.rules.set_pin_edge_to_turn_dist(-1.0);
        assert_path(&board, "pin_edge_to_turn_dist");

        let mut board = valid_board();
        board
            .rules
            .net_classes
            .get_mut(0)
            .set_minimum_trace_length(200.0);
        board
            .rules
            .net_classes
            .get_mut(0)
            .set_maximum_trace_length(100.0);
        assert_path(&board, "trace_length_limits");

        let mut board = valid_board();
        board
            .rules
            .set_same_net_clearance(ItemClass::Via, ItemClass::Pin, -1);
        assert_path(&board, "same_net_clearance");
    }

    #[test]
    fn rejects_bad_padstack_and_item_geometry() {
        let layers = LayerStructure::signal_layers(2);
        let matrix = ClearanceMatrix::get_default_instance(layers.clone(), 200);
        let mut rules = BoardRules::new(layers.clone(), matrix);
        let class = rules.net_classes.append("default", &layers, false);
        rules.net_classes.get_mut(class).set_trace_half_width(100);
        let mut padstacks = Padstacks::new(2);
        padstacks.add("empty", vec![None, None], false, false);
        let board = BasicBoard::new(layers, rules, padstacks);
        assert_path(&board, "padstacks[1].shapes");

        let mut board = valid_board();
        let bad =
            Item::new_polyline_trace(ItemBase::new(0, vec![1], 1), 0, 0, Polyline { arr: vec![] });
        board.insert_item(bad);
        assert_path(&board, "half_width");

        let mut board = valid_board();
        let degenerate = PolylineArea::new(
            PolygonShape::from_int_points(&[IntPoint::new(0, 0), IntPoint::new(1, 0)]),
            Vec::new(),
        );
        board.insert_area(degenerate, 0, "bad", Vec::new(), 1, false);
        assert_path(&board, "area");
    }

    #[test]
    fn rejects_invalid_item_references_and_escape_metadata() {
        let mut board = valid_board();
        let mut base = ItemBase::new(0, vec![1, 1], 1);
        base.component_no = -1;
        board.insert_item(Item::new_via(base, 1, IntPoint::new(4000, 0), false));
        assert_path(&board, "component_no");

        let mut board = valid_board();
        let bad = Item::new_escape_via(
            ItemBase::new(0, vec![1], 1),
            1,
            IntPoint::new(4000, 0),
            false,
            9,
        );
        board.insert_item(bad);
        assert_path(&board, "escape_smd_layer");

        let mut board = valid_board();
        board.outline = Some((vec![IntPoint::ZERO, IntPoint::new(1, 0)], -1));
        assert_path(&board, "outline.corners");
    }

    #[test]
    fn imported_fixture_validation_sweep_rejects_only_known_malformed_inputs() {
        // These legacy DSNs define the same padstack name more than once with
        // different geometry. Keeping either definition would change the
        // meaning of some component pins, so the importer leaves them
        // explicitly rejected instead of guessing.
        let duplicate_padstacks = [
            "Issue039-bug-design.dsn",
            "Issue102-Mars-64-revE-rot00.dsn",
            "Issue034-Green14SegLED.dsn",
            "Issue326-Mars-64-revE.dsn",
            "Issue145-smoothieboard.dsn",
        ];
        let fixture_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../fixtures");
        let mut import_errors = std::collections::BTreeMap::new();
        for entry in std::fs::read_dir(fixture_dir).expect("fixture directory") {
            let path = entry.expect("fixture entry").path();
            if path.extension().and_then(|v| v.to_str()) != Some("dsn") {
                continue;
            }
            let Ok(text) = std::fs::read_to_string(&path) else {
                continue;
            };
            let board = match crate::io::import_dsn(&text) {
                Ok(board) => board,
                Err(error) => {
                    import_errors.insert(
                        path.file_name()
                            .and_then(|v| v.to_str())
                            .unwrap_or("<unnamed>")
                            .to_string(),
                        error.to_string(),
                    );
                    continue;
                }
            };
            validate_board_references(&board).unwrap_or_else(|error| {
                panic!(
                    "imported fixture escaped validation: {}: {error}",
                    path.display()
                )
            });
        }
        let mut expected: std::collections::BTreeSet<_> = duplicate_padstacks.into_iter().collect();
        // Issue179 contains an empty `(class ...)` scope.  It is malformed
        // interchange input, so the importer must reject it rather than
        // inventing a class name or silently applying its rules to `default`.
        expected.insert("Issue179-Autorouter_PCB1_2023-3-24.dsn");
        expected.insert("Issue721-Autorouter_CE2632_HarryMu_2026-6-15.dsn");
        let actual: std::collections::BTreeSet<_> =
            import_errors.keys().map(String::as_str).collect();
        assert_eq!(
            actual, expected,
            "fixture imports failed unexpectedly: {import_errors:#?}"
        );
        assert!(
            import_errors["Issue721-Autorouter_CE2632_HarryMu_2026-6-15.dsn"]
                .contains("unterminated list"),
            "the known Issue721 fixture must remain classified as truncated input"
        );
        assert!(
            import_errors["Issue179-Autorouter_PCB1_2023-3-24.dsn"].contains("missing its name"),
            "the known Issue179 fixture must remain classified as malformed class input"
        );
        for name in duplicate_padstacks {
            assert!(
                import_errors[name].contains("duplicate name"),
                "{name}: {}",
                import_errors[name]
            );
        }
    }
}
