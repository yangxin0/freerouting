//! Port of `KiCadJsonReader.java`: building a board from the KiCad
//! plugin's board JSON (layers, net classes with clearance classes and
//! via rules, custom clearance rules, nets, components with shaped pads,
//! conduction areas, pre-routed traces/vias).
//!
//! Like the Java reader, the Y axis is negated on import (KiCad's Y axis
//! points down, the board's points up). Unlike Java — whose writer emits
//! un-negated Y, breaking its own round trip — the Rust writer negates
//! symmetrically.

use std::cell::RefCell;
use std::collections::{HashMap, HashSet};

use crate::board::basic_board::BasicBoard;
use crate::board::{AngleRestriction, FixedState, Item, ItemBase, Layer, LayerStructure};
use crate::core::Padstacks;
use crate::geometry::planar::{
    IntBox, IntOctagon, IntPoint, Point, PolygonShape, Polyline, PolylineArea, TileShape,
};
use crate::io::json::{parse_json, Json};
use crate::rules::{BoardRules, ClearanceMatrix, ItemClass, ViaInfo, ViaRule};

fn parse_fixed_state(value: Option<&Json>, default: FixedState) -> Result<FixedState, String> {
    let Some(value) = value else {
        return Ok(default);
    };
    let token = value
        .as_str()
        .ok_or_else(|| "fixedState must be a string".to_string())?;
    match token {
        "unfixed" => Ok(FixedState::Unfixed),
        "shove_fixed" => Ok(FixedState::ShoveFixed),
        "user_fixed" => Ok(FixedState::UserFixed),
        "system_fixed" => Ok(FixedState::SystemFixed),
        _ => Err(format!("unknown fixedState {token:?}")),
    }
}

fn metadata_item_class(token: &str) -> Option<ItemClass> {
    match token {
        "none" => Some(ItemClass::None),
        "trace" => Some(ItemClass::Trace),
        "via" => Some(ItemClass::Via),
        "pin" => Some(ItemClass::Pin),
        "smd" => Some(ItemClass::Smd),
        "area" => Some(ItemClass::Area),
        _ => None,
    }
}

/// Numeric geometry fields are optional in older JSON documents, but when a
/// producer includes one it must actually be a JSON number. `Json::num` is a
/// deliberately forgiving compatibility accessor; using it blindly for a
/// misspelled object field turns a string into coordinate zero. Validate the
/// known numeric keys once at the format boundary so every downstream path
/// gets the same fail-closed behavior.
fn validate_numeric_field_types(value: &Json, path: &str) -> Result<(), String> {
    const NUMERIC_KEYS: &[&str] = &[
        "x",
        "y",
        "width",
        "traceWidth",
        "clearance",
        "clearanceValue",
        "viaDiameter",
        "diameter",
        "drill",
        "rotation",
        "layerIndex",
        "startLayerIndex",
        "endLayerIndex",
        "escapeSmdLayer",
    ];
    match value {
        Json::Obj(object) => {
            for (key, child) in object {
                let nullable_numeric = key == "escapeSmdLayer" && matches!(child, Json::Null);
                if NUMERIC_KEYS.contains(&key.as_str())
                    && !matches!(child, Json::Num { .. })
                    && !nullable_numeric
                {
                    return Err(format!("{path}.{key} must be numeric"));
                }
                validate_numeric_field_types(child, &format!("{path}.{key}"))?;
            }
        }
        Json::Arr(values) => {
            for (index, child) in values.iter().enumerate() {
                validate_numeric_field_types(child, &format!("{path}[{index}]"))?;
            }
        }
        Json::Null | Json::Bool(_) | Json::Num { .. } | Json::Str(_) => {}
    }
    Ok(())
}

/// The legacy accessors (`arr`, `str_or`, and `num`) intentionally provide
/// defaults for old documents, but they must not turn a present field of the
/// wrong structural type into an omitted field.  Check collection/object
/// shapes once at the JSON boundary; optional fields may still be absent.
fn validate_structural_field_types(value: &Json, path: &str) -> Result<(), String> {
    const ARRAY_KEYS: &[&str] = &[
        "layers",
        "netClasses",
        "clearanceMatrix",
        "clearanceRules",
        "viaInfos",
        "viaRules",
        "viaInfoNames",
        "nets",
        "components",
        "pads",
        "conductionAreas",
        "traces",
        "vias",
        "points",
        "layerShapes",
        "corners",
        "polygon",
        "values",
        "netNames",
        "traceHalfWidths",
        "activeRoutingLayers",
    ];
    const OBJECT_KEYS: &[&str] = &[
        "position",
        "offset",
        "size",
        "outline",
        "freeroutingRules",
        "routingMetadata",
        "itemClearanceClasses",
    ];
    let require_point = |point: &Json, point_path: &str| -> Result<(), String> {
        let Json::Obj(object) = point else {
            return Err(format!(
                "{point_path} must be an object with numeric x and y"
            ));
        };
        for coordinate in ["x", "y"] {
            let Some(value) = object.get(coordinate) else {
                return Err(format!("{point_path}.{coordinate} is required"));
            };
            if !matches!(value, Json::Num { .. }) {
                return Err(format!("{point_path}.{coordinate} must be numeric"));
            }
        }
        Ok(())
    };
    match value {
        Json::Obj(object) => {
            for (key, child) in object {
                if ARRAY_KEYS.contains(&key.as_str()) && !matches!(child, Json::Arr(_)) {
                    return Err(format!("{path}.{key} must be an array"));
                }
                if OBJECT_KEYS.contains(&key.as_str()) && !matches!(child, Json::Obj(_)) {
                    return Err(format!("{path}.{key} must be an object"));
                }
                if matches!(key.as_str(), "position" | "offset" | "size") {
                    require_point(child, &format!("{path}.{key}"))?;
                }
                if matches!(key.as_str(), "points" | "corners" | "polygon") {
                    let Json::Arr(points) = child else {
                        unreachable!("array field was checked above");
                    };
                    for (index, point) in points.iter().enumerate() {
                        require_point(point, &format!("{path}.{key}[{index}]"))?;
                    }
                }
                validate_structural_field_types(child, &format!("{path}.{key}"))?;
            }
        }
        Json::Arr(values) => {
            for (index, child) in values.iter().enumerate() {
                validate_structural_field_types(child, &format!("{path}[{index}]"))?;
            }
        }
        Json::Null | Json::Bool(_) | Json::Num { .. } | Json::Str(_) => {}
    }
    Ok(())
}

fn required_non_empty_string<'a>(
    value: &'a Json,
    key: &str,
    path: &str,
) -> Result<&'a str, String> {
    match value.get(key) {
        Some(Json::Str(text)) if !text.is_empty() => Ok(text),
        Some(Json::Str(_)) => Err(format!("{path}.{key} must not be empty")),
        Some(_) => Err(format!("{path}.{key} must be a string")),
        None => Err(format!("{path}.{key} is required")),
    }
}

fn optional_string<'a>(value: &'a Json, key: &str, path: &str) -> Result<Option<&'a str>, String> {
    match value.get(key) {
        Some(Json::Str(text)) => Ok(Some(text)),
        Some(Json::Null) if key == "viaRuleName" => Ok(None),
        Some(_) => Err(format!("{path}.{key} must be a string")),
        None => Ok(None),
    }
}

fn require_object(value: &Json, path: &str) -> Result<(), String> {
    if matches!(value, Json::Obj(_)) {
        Ok(())
    } else {
        Err(format!("{path} must be an object"))
    }
}

fn canonical_name(name: &str) -> String {
    name.to_ascii_lowercase()
}

fn insert_unique_name(seen: &mut HashSet<String>, name: &str, path: &str) -> Result<(), String> {
    if seen.insert(canonical_name(name)) {
        Ok(())
    } else {
        Err(format!("{path} duplicates the name {name:?}"))
    }
}

fn validate_bool_fields(value: &Json, path: &str) -> Result<(), String> {
    const BOOLEAN_KEYS: &[&str] = &[
        "containsPlane",
        "attachAllowed",
        "clearanceExplicit",
        "isObstacle",
        "viaOnly",
        "isEscapeVia",
        "ignoredByAutorouter",
        "shoveFixed",
        "pullTight",
        "ignoreCyclesWithAreas",
        "viaAtSmdAllowed",
        "ignoreConduction",
        "automaticNeckdown",
    ];
    match value {
        Json::Obj(object) => {
            for (key, child) in object {
                if BOOLEAN_KEYS.contains(&key.as_str()) && !matches!(child, Json::Bool(_)) {
                    return Err(format!("{path}.{key} must be boolean"));
                }
                validate_bool_fields(child, &format!("{path}.{key}"))?;
            }
        }
        Json::Arr(values) => {
            for (index, child) in values.iter().enumerate() {
                validate_bool_fields(child, &format!("{path}[{index}]"))?;
            }
        }
        Json::Null | Json::Bool(_) | Json::Num { .. } | Json::Str(_) => {}
    }
    Ok(())
}

fn validate_string_field_types(value: &Json, path: &str) -> Result<(), String> {
    const STRING_KEYS: &[&str] = &[
        "unit",
        "name",
        "type",
        "classA",
        "classB",
        "className",
        "padstackName",
        "clearanceClass",
        "traceClearanceClass",
        "netName",
        "reference",
        "value",
        "footprint",
        "layer",
        "shape",
        "fixedState",
        "viaInfoName",
        "viaRuleName",
        "traceAngleRestriction",
    ];
    match value {
        Json::Obj(object) => {
            for (key, child) in object {
                let nullable_via_rule = key == "viaRuleName" && matches!(child, Json::Null);
                if STRING_KEYS.contains(&key.as_str())
                    && !matches!(child, Json::Str(_))
                    && !nullable_via_rule
                {
                    return Err(format!("{path}.{key} must be a string"));
                }
                validate_string_field_types(child, &format!("{path}.{key}"))?;
            }
        }
        Json::Arr(values) => {
            for (index, child) in values.iter().enumerate() {
                validate_string_field_types(child, &format!("{path}[{index}]"))?;
            }
        }
        Json::Null | Json::Bool(_) | Json::Num { .. } | Json::Str(_) => {}
    }
    Ok(())
}

/// Validates identities and references before the importer mutates a board.
///
/// Two legacy behaviors remain intentional:
/// - an absent/empty `nets` table allows item `netName` values to declare nets;
/// - absent via metadata allows routed vias to synthesize a ViaInfo/ViaRule.
///
/// Explicit declaration tables, however, are authoritative and must be
/// internally consistent.  A malformed routed item must never disappear by
/// being treated as an empty collection.
fn validate_kicad_semantics(doc: &Json) -> Result<(), String> {
    let mut layer_names = HashSet::new();
    let mut layer_indices = HashSet::new();
    for (index, layer) in doc.arr("layers").iter().enumerate() {
        let path = format!("board.layers[{index}]");
        require_object(layer, &path)?;
        let name = required_non_empty_string(layer, "name", &path)?;
        insert_unique_name(&mut layer_names, name, &path)?;
        if let Some(raw_index) = layer.get("index") {
            let value = raw_index
                .as_exact_u64()
                .ok_or_else(|| format!("{path}.index must be an exact non-negative integer"))?;
            if !layer_indices.insert(value) {
                return Err(format!("{path}.index duplicates layer index {value}"));
            }
        }
        optional_string(layer, "type", &path)?;
    }
    if layer_names.is_empty() {
        layer_names.insert("f.cu".to_string());
        layer_names.insert("b.cu".to_string());
    }

    let mut net_class_names = HashSet::new();
    for (index, class) in doc.arr("netClasses").iter().enumerate() {
        let path = format!("board.netClasses[{index}]");
        require_object(class, &path)?;
        let name = required_non_empty_string(class, "name", &path)?;
        insert_unique_name(&mut net_class_names, name, &path)?;
        if let Some(members) = class.get("netNames") {
            let members = members
                .as_arr()
                .ok_or_else(|| format!("{path}.netNames must be an array"))?;
            let mut seen_members = HashSet::new();
            for (member_index, member) in members.iter().enumerate() {
                let member = member
                    .as_str()
                    .ok_or_else(|| format!("{path}.netNames[{member_index}] must be a string"))?;
                if member.is_empty() {
                    return Err(format!("{path}.netNames[{member_index}] must not be empty"));
                }
                if !seen_members.insert(canonical_name(member)) {
                    return Err(format!(
                        "{path}.netNames contains duplicate member {member:?}"
                    ));
                }
            }
        }
    }

    let mut clearance_names = HashSet::from(["null".to_string(), "default".to_string()]);
    for name in &net_class_names {
        clearance_names.insert(name.clone());
    }
    let mut matrix_row_names = HashSet::new();
    for (index, row) in doc.arr("clearanceMatrix").iter().enumerate() {
        let path = format!("board.clearanceMatrix[{index}]");
        require_object(row, &path)?;
        let name = required_non_empty_string(row, "name", &path)?;
        insert_unique_name(&mut matrix_row_names, name, &path)?;
        clearance_names.insert(canonical_name(name));
        for (layer, values) in row.arr("layers").iter().enumerate() {
            let values = values
                .as_arr()
                .ok_or_else(|| format!("{path}.layers[{layer}] must be an array"))?;
            for (column, value) in values.iter().enumerate() {
                if value.as_f64().is_none() {
                    return Err(format!("{path}.layers[{layer}][{column}] must be numeric"));
                }
            }
        }
    }
    for (index, rule) in doc.arr("clearanceRules").iter().enumerate() {
        let path = format!("board.clearanceRules[{index}]");
        require_object(rule, &path)?;
        for key in ["classA", "classB"] {
            let name = required_non_empty_string(rule, key, &path)?;
            if !clearance_names.contains(&canonical_name(name)) {
                return Err(format!("{path}.{key} references unknown class {name:?}"));
            }
        }
        if rule.get("clearance").is_none() {
            return Err(format!("{path}.clearance is required"));
        }
    }

    let mut declared_net_names = HashSet::new();
    let mut net_ids = HashSet::new();
    for (index, net) in doc.arr("nets").iter().enumerate() {
        let path = format!("board.nets[{index}]");
        require_object(net, &path)?;
        let name = required_non_empty_string(net, "name", &path)?;
        insert_unique_name(&mut declared_net_names, name, &path)?;
        if let Some(id) = net.get("id") {
            let id = id
                .as_exact_u64()
                .filter(|id| *id > 0)
                .ok_or_else(|| format!("{path}.id must be an exact positive integer"))?;
            if !net_ids.insert(id) {
                return Err(format!("{path}.id duplicates net id {id}"));
            }
        }
        if let Some(class_name) = optional_string(net, "className", &path)? {
            let canonical = canonical_name(class_name);
            if !class_name.is_empty()
                && canonical != "default"
                && canonical != "null"
                && !net_class_names.contains(&canonical)
            {
                return Err(format!(
                    "{path}.className references unknown net class {class_name:?}"
                ));
            }
        }
    }

    let has_via_metadata = !doc.arr("viaInfos").is_empty() || !doc.arr("viaRules").is_empty();
    let mut via_info_names = HashSet::new();
    for (index, info) in doc.arr("viaInfos").iter().enumerate() {
        let path = format!("board.viaInfos[{index}]");
        require_object(info, &path)?;
        let name = required_non_empty_string(info, "name", &path)?;
        insert_unique_name(&mut via_info_names, name, &path)?;
        required_non_empty_string(info, "padstackName", &path)?;
    }
    let mut via_rule_names = HashSet::new();
    let mut via_rule_members: HashMap<String, HashSet<String>> = HashMap::new();
    for (index, rule) in doc.arr("viaRules").iter().enumerate() {
        let path = format!("board.viaRules[{index}]");
        require_object(rule, &path)?;
        let name = required_non_empty_string(rule, "name", &path)?;
        insert_unique_name(&mut via_rule_names, name, &path)?;
        let mut members = HashSet::new();
        for (member_index, member) in rule.arr("viaInfoNames").iter().enumerate() {
            let member = member
                .as_str()
                .ok_or_else(|| format!("{path}.viaInfoNames[{member_index}] must be a string"))?;
            if !via_info_names.contains(&canonical_name(member)) {
                return Err(format!(
                    "{path}.viaInfoNames[{member_index}] references unknown viaInfo {member:?}"
                ));
            }
            if !members.insert(canonical_name(member)) {
                return Err(format!(
                    "{path}.viaInfoNames contains duplicate viaInfo {member:?}"
                ));
            }
        }
        via_rule_members.insert(canonical_name(name), members);
    }
    if has_via_metadata {
        for (index, class) in doc.arr("netClasses").iter().enumerate() {
            let path = format!("board.netClasses[{index}]");
            if let Some(rule_name) = optional_string(class, "viaRuleName", &path)? {
                if !rule_name.is_empty() && !via_rule_names.contains(&canonical_name(rule_name)) {
                    return Err(format!(
                        "{path}.viaRuleName references unknown viaRule {rule_name:?}"
                    ));
                }
            }
        }
    }

    let known_layer = |name: &str| layer_names.contains(&canonical_name(name));
    let mut component_refs = HashSet::new();
    for (component_index, component) in doc.arr("components").iter().enumerate() {
        let path = format!("board.components[{component_index}]");
        require_object(component, &path)?;
        if let Some(reference) = optional_string(component, "reference", &path)? {
            if !reference.is_empty() && !component_refs.insert(canonical_name(reference)) {
                return Err(format!(
                    "{path}.reference duplicates component {reference:?}"
                ));
            }
        }
        if !component.arr("pads").is_empty() && component.get("position").is_none() {
            return Err(format!(
                "{path}.position is required when the component has pads"
            ));
        }
        for (pad_index, pad) in component.arr("pads").iter().enumerate() {
            let pad_path = format!("{path}.pads[{pad_index}]");
            require_object(pad, &pad_path)?;
            if pad.get("size").is_none() {
                return Err(format!("{pad_path}.size is required"));
            }
            for coordinate in ["x", "y"] {
                let size = pad
                    .get("size")
                    .and_then(|value| value.get(coordinate))
                    .and_then(Json::as_f64)
                    .ok_or_else(|| format!("{pad_path}.size.{coordinate} must be numeric"))?;
                if size <= 0.0 {
                    return Err(format!("{pad_path}.size.{coordinate} must be positive"));
                }
            }
            let mut named_layers = HashSet::new();
            for (layer_index, layer) in pad.arr("layers").iter().enumerate() {
                let layer = layer
                    .as_str()
                    .ok_or_else(|| format!("{pad_path}.layers[{layer_index}] must be a string"))?;
                if !known_layer(layer) {
                    return Err(format!(
                        "{pad_path}.layers[{layer_index}] references unknown layer {layer:?}"
                    ));
                }
                if !named_layers.insert(canonical_name(layer)) {
                    return Err(format!(
                        "{pad_path}.layers contains duplicate layer {layer:?}"
                    ));
                }
            }
        }
    }

    for (zone_index, zone) in doc.arr("conductionAreas").iter().enumerate() {
        let path = format!("board.conductionAreas[{zone_index}]");
        require_object(zone, &path)?;
        if zone.arr("polygon").len() < 3 {
            return Err(format!("{path}.polygon must contain at least three points"));
        }
    }

    let mut routed_item_ids = HashSet::new();
    for (trace_index, trace) in doc.arr("traces").iter().enumerate() {
        let path = format!("board.traces[{trace_index}]");
        require_object(trace, &path)?;
        if trace.arr("points").len() < 2 {
            return Err(format!("{path}.points must contain at least two points"));
        }
        if trace
            .get("width")
            .and_then(Json::as_f64)
            .is_some_and(|width| width <= 0.0)
        {
            return Err(format!("{path}.width must be positive"));
        }
        if let Some(id) = trace.get("id") {
            let id = id
                .as_exact_u64()
                .filter(|id| *id > 0)
                .ok_or_else(|| format!("{path}.id must be an exact positive integer"))?;
            if !routed_item_ids.insert(id) {
                return Err(format!("{path}.id duplicates routed item id {id}"));
            }
        }
    }
    for (via_index, via) in doc.arr("vias").iter().enumerate() {
        let path = format!("board.vias[{via_index}]");
        require_object(via, &path)?;
        if via.get("position").is_none() {
            return Err(format!("{path}.position is required"));
        }
        if via
            .get("diameter")
            .and_then(Json::as_f64)
            .is_some_and(|diameter| diameter <= 0.0)
        {
            return Err(format!("{path}.diameter must be positive"));
        }
        if let Some(id) = via.get("id") {
            let id = id
                .as_exact_u64()
                .filter(|id| *id > 0)
                .ok_or_else(|| format!("{path}.id must be an exact positive integer"))?;
            if !routed_item_ids.insert(id) {
                return Err(format!("{path}.id duplicates routed item id {id}"));
            }
        }
        let info_name = optional_string(via, "viaInfoName", &path)?.filter(|name| !name.is_empty());
        let rule_name = optional_string(via, "viaRuleName", &path)?.filter(|name| !name.is_empty());
        if has_via_metadata {
            if let Some(info_name) = info_name {
                if !via_info_names.contains(&canonical_name(info_name)) {
                    return Err(format!(
                        "{path}.viaInfoName references unknown viaInfo {info_name:?}"
                    ));
                }
            }
            if let Some(rule_name) = rule_name {
                let canonical_rule = canonical_name(rule_name);
                if !via_rule_names.contains(&canonical_rule) {
                    return Err(format!(
                        "{path}.viaRuleName references unknown viaRule {rule_name:?}"
                    ));
                }
                if let Some(info_name) = info_name {
                    if !via_rule_members
                        .get(&canonical_rule)
                        .is_some_and(|members| members.contains(&canonical_name(info_name)))
                    {
                        return Err(format!(
                            "{path} selects viaInfo {info_name:?} outside viaRule {rule_name:?}"
                        ));
                    }
                }
            }
        }
    }

    Ok(())
}

/// Older KiCad JSON exports occasionally contain a malformed outline with a
/// repeated corner (Issue649).  The public schema has no way to express a
/// missing edge, so recover the only deterministic polygon implied by the
/// data: its axis-aligned bounding rectangle.  The checked writer remains
/// strict; this repair is performed only while ingesting legacy input.
fn normalize_outline_corners(mut corners: Vec<IntPoint>) -> Vec<IntPoint> {
    let twice_area: i128 = corners
        .iter()
        .zip(corners.iter().cycle().skip(1))
        .take(corners.len())
        .map(|(a, b)| i128::from(a.x) * i128::from(b.y) - i128::from(b.x) * i128::from(a.y))
        .sum();
    if twice_area != 0 {
        return corners;
    }
    corners.dedup();
    if corners.len() > 1 && corners.first() == corners.last() {
        corners.pop();
    }
    let (Some(min_x), Some(max_x), Some(min_y), Some(max_y)) = (
        corners.iter().map(|point| point.x).min(),
        corners.iter().map(|point| point.x).max(),
        corners.iter().map(|point| point.y).min(),
        corners.iter().map(|point| point.y).max(),
    ) else {
        return corners;
    };
    if min_x == max_x || min_y == max_y {
        return corners;
    }
    vec![
        IntPoint::new(min_x, min_y),
        IntPoint::new(max_x, min_y),
        IntPoint::new(max_x, max_y),
        IntPoint::new(min_x, max_y),
    ]
}

fn apply_board_routing_metadata(
    rules: &mut BoardRules,
    metadata: Option<&Json>,
    resolution: i32,
) -> Result<(), String> {
    let Some(metadata) = metadata else {
        return Ok(());
    };
    if !matches!(metadata, Json::Obj(_)) {
        return Err("freeroutingRules must be an object".into());
    }
    if metadata.get("version").and_then(Json::as_exact_u64) != Some(1) {
        return Err("freeroutingRules.version must be the integer 1".into());
    }
    if let Some(value) = metadata.get("traceAngleRestriction") {
        let token = value
            .as_str()
            .ok_or_else(|| "freeroutingRules.traceAngleRestriction must be a string".to_string())?;
        rules.set_trace_angle_restriction(match token {
            "none" => AngleRestriction::None,
            "fortyfive_degree" => AngleRestriction::FortyfiveDegree,
            "ninety_degree" => AngleRestriction::NinetyDegree,
            _ => return Err(format!("unknown traceAngleRestriction {token:?}")),
        });
    }
    let optional_bool = |key: &str| -> Result<Option<bool>, String> {
        match metadata.get(key) {
            None => Ok(None),
            Some(Json::Bool(value)) => Ok(Some(*value)),
            Some(_) => Err(format!("freeroutingRules.{key} must be boolean")),
        }
    };
    if let Some(value) = optional_bool("ignoreConduction")? {
        rules.set_ignore_conduction(value);
    }
    if let Some(value) = optional_bool("useSlowAutorouteAlgorithm")? {
        rules.set_use_slow_autoroute_algorithm(value);
    }
    if let Some(value) = optional_bool("viaAtSmdAllowed")? {
        rules.via_at_smd_allowed = value;
    }
    if let Some(value) = metadata.get("pinEdgeToTurnDistance") {
        let value = value
            .as_f64()
            .ok_or_else(|| "freeroutingRules.pinEdgeToTurnDistance must be numeric".to_string())?;
        let scaled = value * f64::from(resolution);
        if !scaled.is_finite() {
            return Err("freeroutingRules.pinEdgeToTurnDistance must be finite".into());
        }
        rules.set_pin_edge_to_turn_dist(scaled);
    }
    if let Some(entries) = metadata.get("sameNetClearances") {
        let entries = entries
            .as_arr()
            .ok_or_else(|| "freeroutingRules.sameNetClearances must be an array".to_string())?;
        for (index, entry) in entries.iter().enumerate() {
            let first = entry
                .get("first")
                .and_then(Json::as_str)
                .and_then(metadata_item_class)
                .ok_or_else(|| {
                    format!("freeroutingRules.sameNetClearances[{index}].first is invalid")
                })?;
            let second = entry
                .get("second")
                .and_then(Json::as_str)
                .and_then(metadata_item_class)
                .ok_or_else(|| {
                    format!("freeroutingRules.sameNetClearances[{index}].second is invalid")
                })?;
            let value = entry
                .get("clearance")
                .and_then(Json::as_f64)
                .ok_or_else(|| {
                    format!("freeroutingRules.sameNetClearances[{index}].clearance must be numeric")
                })?;
            let scaled = value * f64::from(resolution);
            if !scaled.is_finite() || scaled < 0.0 || scaled > f64::from(i32::MAX) {
                return Err(format!(
                    "freeroutingRules.sameNetClearances[{index}].clearance is out of range"
                ));
            }
            // `scaled` has already been range-checked; use a checked
            // rounding boundary so the exact i32::MIN case cannot later
            // overflow when KiCad Y coordinates are mirrored.
            let rounded = scaled.round();
            if rounded <= f64::from(i32::MIN) {
                return Err(format!(
                    "freeroutingRules.sameNetClearances[{index}].clearance is outside the board range"
                ));
            }
            rules.set_same_net_clearance(first, second, rounded as i32);
        }
    }
    Ok(())
}

/// An octagon approximating a stadium/circle pad of half-extents
/// `dx`/`dy` centered at the origin (Java builds the same `IntOctagon`
/// for oval pads: the diagonals cut `(2 - sqrt(2)) * r` off the corners).
fn oval_shape(dx: f64, dy: f64, round: &dyn Fn(f64) -> i32) -> TileShape {
    // Keep all intermediate arithmetic in f64 and pass every resulting
    // coordinate through the importer's checked board-unit conversion. The
    // old i32 expressions could overflow before the final scale guard ran.
    let r = dx.min(dy);
    let cut = (2.0 - std::f64::consts::SQRT_2) * r;
    let oct = IntOctagon::new(
        round(-dx),
        round(-dy),
        round(dx),
        round(dy),
        round(-dx - dy + cut),
        round(dx + dy - cut),
        round(-dx - dy + cut),
        round(dx + dy - cut),
    );
    TileShape::Octagon(oct.normalize())
}

/// Resolves the optional per-item class metadata used by the writer. KiCad's
/// public schema normally names only net classes; an imported DSN may carry a
/// matrix-only item class, so recreate that class from its scalar spacing
/// rather than silently falling back to the default column.
fn resolve_json_item_clearance_class(board: &mut BasicBoard, name: &str, value: i32) -> usize {
    if name.eq_ignore_ascii_case("null") {
        return BoardRules::clearance_class_none();
    }
    if let Some(existing) = board.rules.clearance_matrix.get_no(name) {
        return existing;
    }
    board.rules.clearance_matrix.append_class(name);
    let index = board
        .rules
        .clearance_matrix
        .get_no(name)
        .unwrap_or_else(BoardRules::default_clearance_class);
    let count = board.rules.clearance_matrix.get_class_count();
    for other in 1..count {
        board
            .rules
            .clearance_matrix
            .set_value_on_all_layers(index, other, value);
        board
            .rules
            .clearance_matrix
            .set_value_on_all_layers(other, index, value);
    }
    index
}

/// Maps KiCad's external layer ids to the board's dense internal layer
/// ordinals.  KiCad ids are not array offsets (`B.Cu` is commonly 31), so
/// bounds-checking them against `layers.len()` rejects valid plugin output.
struct JsonLayerMap {
    external_to_internal: HashMap<u64, usize>,
    layer_count: usize,
    legacy_two_layer_ordinals: bool,
}

impl JsonLayerMap {
    fn from_document(doc: &Json, layer_count: usize) -> Result<Self, String> {
        let declared = doc.arr("layers");
        let mut external_to_internal = HashMap::new();
        if declared.is_empty() {
            external_to_internal.insert(0, 0);
            external_to_internal.insert(1, 1);
        } else {
            for (internal, layer) in declared.iter().enumerate() {
                let external = match layer.get("index") {
                    Some(value) => value.as_exact_u64().ok_or_else(|| {
                        format!("layers[{internal}].index must be an exact non-negative integer")
                    })?,
                    None => internal as u64,
                };
                if external_to_internal.insert(external, internal).is_some() {
                    return Err(format!(
                        "layers[{internal}].index duplicates external layer id {external}"
                    ));
                }
            }
        }
        let legacy_two_layer_ordinals = layer_count == 2
            && external_to_internal.get(&0) == Some(&0)
            && external_to_internal.get(&1) == Some(&1);
        Ok(Self {
            external_to_internal,
            layer_count,
            legacy_two_layer_ordinals,
        })
    }

    fn resolve(&self, external: u64, key: &str) -> Result<usize, String> {
        if let Some(&internal) = self.external_to_internal.get(&external) {
            return Ok(internal);
        }
        // Older manual KiCad-plugin JSON used dense layer declarations but
        // copied the raw back-copper id into routed items.  KiCad releases
        // have used both 2 and 31 for that id.  The meaning is unambiguous on
        // a two-layer stack; retain only this narrow compatibility mapping.
        if self.legacy_two_layer_ordinals && matches!(external, 2 | 31) {
            return Ok(1);
        }
        Err(format!(
            "{key} external layer id {external} is not declared by the layer stack"
        ))
    }
}

fn json_layer_index(
    object: &Json,
    key: &str,
    default: usize,
    layer_map: &JsonLayerMap,
) -> Result<usize, String> {
    let Some(value) = object.get(key) else {
        return Ok(default.min(layer_map.layer_count.saturating_sub(1)));
    };
    let raw = value
        .as_exact_u64()
        .ok_or_else(|| format!("{key} must be an exact non-negative integer"))?;
    layer_map.resolve(raw, key)
}

/// Parses the optional per-layer convex polygons used by routed vias and
/// component pads.  The legacy `layers`/`shape` fields only describe a
/// contiguous span; this extension preserves padstacks that deliberately
/// have no copper on an intermediate layer (for example an SMD escape or a
/// sparse blind/buried pad).
fn parse_layer_shapes(
    value: Option<&Json>,
    layer_map: &JsonLayerMap,
    from: usize,
    to: usize,
    point: impl Fn(&Json) -> IntPoint,
) -> Result<Option<Vec<Option<TileShape>>>, String> {
    let Some(value) = value else {
        return Ok(None);
    };
    let entries = value
        .as_arr()
        .ok_or_else(|| "layerShapes must be an array".to_string())?;
    let mut shapes = vec![None; layer_map.layer_count];
    let mut seen = vec![false; layer_map.layer_count];
    for entry in entries {
        let layer = json_layer_index(entry, "layerIndex", 0, layer_map)?;
        if seen[layer] {
            return Err("layerShapes contains a duplicate layerIndex".into());
        }
        seen[layer] = true;
        if !(from..=to).contains(&layer) {
            return Err("layerShapes must stay inside the declared layer span".into());
        }
        let corners: Vec<IntPoint> = entry.arr("corners").iter().map(&point).collect();
        if corners.len() < 3 {
            return Err("each layerShapes polygon needs at least three corners".into());
        }
        shapes[layer] = Some(TileShape::from_convex_polygon(&corners));
    }
    if entries.is_empty() || shapes[from].is_none() || shapes[to].is_none() {
        return Err("layerShapes must describe both endpoints of the layer span".into());
    }
    Ok(Some(shapes))
}

const ROUTING_METADATA_ITEM_CLASSES: [(ItemClass, &str); 6] = [
    (ItemClass::None, "none"),
    (ItemClass::Trace, "trace"),
    (ItemClass::Via, "via"),
    (ItemClass::Pin, "pin"),
    (ItemClass::Smd, "smd"),
    (ItemClass::Area, "area"),
];

fn optional_json_bool(value: &Json, key: &str) -> Result<Option<bool>, String> {
    match value.get(key) {
        None => Ok(None),
        Some(Json::Bool(value)) => Ok(Some(*value)),
        Some(_) => Err(format!("routingMetadata.{key} must be boolean")),
    }
}

/// Applies the optional Freerouting extension carried by each JSON net class.
/// Public KiCad fields are scalar, so without this metadata a valid class can
/// silently lose per-item clearance bindings and per-layer routing behavior.
/// Documents that omit the extension retain the legacy scalar interpretation.
fn apply_net_class_routing_metadata(
    rules: &mut BoardRules,
    class_no: usize,
    metadata: &Json,
    layer_count: usize,
    resolution: i32,
) -> Result<(), String> {
    if !matches!(metadata, Json::Obj(_)) {
        return Err("routingMetadata must be an object".into());
    }
    if let Some(version) = metadata.get("version") {
        if version.as_exact_u64() != Some(1) {
            return Err("routingMetadata.version must be the integer 1".into());
        }
    }

    let resolve_clearance = |field: &str, value: &Json| -> Result<usize, String> {
        let name = value
            .as_str()
            .ok_or_else(|| format!("routingMetadata.{field} must be a string"))?;
        rules
            .clearance_matrix
            .get_no(name)
            .ok_or_else(|| format!("routingMetadata.{field} references unknown class {name:?}"))
    };

    let trace_clearance = metadata
        .get("traceClearanceClass")
        .map(|value| resolve_clearance("traceClearanceClass", value))
        .transpose()?;

    let mut item_clearances = Vec::new();
    if let Some(bindings) = metadata.get("itemClearanceClasses") {
        if !matches!(bindings, Json::Obj(_)) {
            return Err("routingMetadata.itemClearanceClasses must be an object".into());
        }
        for (item_class, field) in ROUTING_METADATA_ITEM_CLASSES {
            if let Some(value) = bindings.get(field) {
                item_clearances.push((
                    item_class,
                    resolve_clearance(&format!("itemClearanceClasses.{field}"), value)?,
                ));
            }
        }
    }

    let trace_half_widths = metadata
        .get("traceHalfWidths")
        .map(|value| {
            let values = value
                .as_arr()
                .ok_or_else(|| "routingMetadata.traceHalfWidths must be an array".to_string())?;
            if values.len() != layer_count {
                return Err(format!(
                    "routingMetadata.traceHalfWidths must contain {layer_count} entries"
                ));
            }
            values
                .iter()
                .enumerate()
                .map(|(layer, value)| {
                    let width = value.as_f64().ok_or_else(|| {
                        format!("routingMetadata.traceHalfWidths[{layer}] must be numeric")
                    })?;
                    let scaled_f64 = width * f64::from(resolution);
                    if !scaled_f64.is_finite()
                        || scaled_f64 < 0.0
                        || scaled_f64 > f64::from(i32::MAX)
                    {
                        return Err(format!(
                            "routingMetadata.traceHalfWidths[{layer}] is outside the board range"
                        ));
                    }
                    let scaled = scaled_f64.round();
                    if scaled <= f64::from(i32::MIN) {
                        return Err(format!(
                            "routingMetadata.traceHalfWidths[{layer}] is outside the board range"
                        ));
                    }
                    Ok(scaled as i32)
                })
                .collect::<Result<Vec<_>, _>>()
        })
        .transpose()?;

    let active_layers = metadata
        .get("activeRoutingLayers")
        .map(|value| {
            let values = value.as_arr().ok_or_else(|| {
                "routingMetadata.activeRoutingLayers must be an array".to_string()
            })?;
            if values.len() != layer_count {
                return Err(format!(
                    "routingMetadata.activeRoutingLayers must contain {layer_count} entries"
                ));
            }
            values
                .iter()
                .enumerate()
                .map(|(layer, value)| match value {
                    Json::Bool(active) => Ok(*active),
                    _ => Err(format!(
                        "routingMetadata.activeRoutingLayers[{layer}] must be boolean"
                    )),
                })
                .collect::<Result<Vec<_>, _>>()
        })
        .transpose()?;

    let ignored = optional_json_bool(metadata, "ignoredByAutorouter")?;
    let shove_fixed = optional_json_bool(metadata, "shoveFixed")?;
    let pull_tight = optional_json_bool(metadata, "pullTight")?;
    let ignore_cycles = optional_json_bool(metadata, "ignoreCyclesWithAreas")?;
    let parse_length = |key: &str| -> Result<Option<f64>, String> {
        metadata
            .get(key)
            .map(|value| {
                value
                    .as_f64()
                    .map(|length| length * f64::from(resolution))
                    .filter(|length| length.is_finite())
                    .ok_or_else(|| format!("routingMetadata.{key} must be numeric"))
            })
            .transpose()
    };
    let minimum_length = parse_length("minimumTraceLength")?;
    let maximum_length = parse_length("maximumTraceLength")?;

    let class = rules.net_classes.get_mut(class_no);
    if let Some(clearance) = trace_clearance {
        class.set_trace_clearance_class(clearance);
    }
    for (item_class, clearance) in item_clearances {
        class
            .default_item_clearance_classes
            .set(item_class, clearance);
    }
    if let Some(widths) = trace_half_widths {
        for (layer, width) in widths.into_iter().enumerate() {
            class.set_trace_half_width_on_layer(layer, width);
        }
    }
    if let Some(active) = active_layers {
        for (layer, active) in active.into_iter().enumerate() {
            class.set_active_routing_layer(layer, active);
        }
    }
    if let Some(value) = ignored {
        class.is_ignored_by_autorouter = value;
    }
    if let Some(value) = shove_fixed {
        class.set_shove_fixed(value);
    }
    if let Some(value) = pull_tight {
        class.set_pull_tight(value);
    }
    if let Some(value) = ignore_cycles {
        class.set_ignore_cycles_with_areas(value);
    }
    if let Some(value) = minimum_length {
        class.set_minimum_trace_length(value);
    }
    if let Some(value) = maximum_length {
        class.set_maximum_trace_length(value);
    }
    for layer in 0..layer_count {
        if class.is_active_routing_layer(layer) && class.get_trace_half_width(layer) <= 0 {
            return Err(format!(
                "routingMetadata activates layer {layer} without a positive trace half width"
            ));
        }
    }
    Ok(())
}

/// Reads a KiCad board JSON document into a board (Java:
/// `KiCadJsonReader.readBoard`). Coordinates are in the document's unit
/// (default mm), converted at the Java default resolution of 10000
/// board units per mm (0.1 µm).
pub fn import_kicad_json(content: &str) -> Result<BasicBoard, String> {
    let doc = parse_json(content)?;
    if !matches!(doc, Json::Obj(_)) {
        return Err("KiCad board root must be a JSON object".into());
    }
    validate_structural_field_types(&doc, "board")?;
    validate_numeric_field_types(&doc, "board")?;
    validate_bool_fields(&doc, "board")?;
    validate_string_field_types(&doc, "board")?;
    validate_kicad_semantics(&doc)?;
    // The board stores an integer resolution and a finite, known physical
    // unit.  Silently treating a misspelled unit as micrometres (or clamping
    // zero/negative resolutions to one) changes every coordinate and rule
    // value by orders of magnitude, so reject malformed metadata at the
    // format boundary.
    let unit_key = match doc.get("unit") {
        None => "mm".to_string(),
        Some(Json::Str(value)) => match value.to_ascii_lowercase().as_str() {
            "mm" => "mm".to_string(),
            "mil" => "mil".to_string(),
            "um" | "micron" | "microns" => "um".to_string(),
            "inch" | "in" => "inch".to_string(),
            "cm" => "cm".to_string(),
            _ => return Err(format!("unsupported KiCad unit {value:?}")),
        },
        Some(_) => return Err("unit must be a string".into()),
    };
    // Java KiCadJsonReader: `resolution` is board units per DOCUMENT unit
    // (mil/um/mm), clamped to >= 1; the 10000 default applies ONLY to an
    // unspecified-resolution mm document. Converting through an mm basis
    // instead coarsened valid low-resolution MIL/UM inputs (a 10-units/mil
    // document became 10-units/mm — a 40x coarser grid).
    let resolution = {
        let raw = match doc.get("resolution") {
            None => 1.0,
            Some(value) => value
                .as_f64()
                .ok_or_else(|| "resolution must be a number".to_string())?,
        };
        if !raw.is_finite() || raw < 1.0 || raw > f64::from(i32::MAX) || raw.fract() != 0.0 {
            return Err(format!(
                "resolution must be a positive integer no greater than {}, got {raw}",
                i32::MAX
            ));
        }
        let r = raw as i32;
        if r == 1 && unit_key == "mm" {
            10_000 // Java: 0.1 µm default for mm
        } else {
            r
        }
    };
    let scale_error: RefCell<Option<String>> = RefCell::new(None);
    let round_board_units = |value: f64| -> i32 {
        let rounded = value.round();
        let limit = f64::from(crate::geometry::planar::limits::CRIT_INT);
        if !rounded.is_finite() || rounded < -limit || rounded > limit {
            let mut error = scale_error.borrow_mut();
            if error.is_none() {
                *error = Some(format!(
                    "scaled geometry/rule value {value} does not fit the board coordinate range"
                ));
            }
            return 0;
        }
        rounded as i32
    };
    let to_units = |v: f64| -> f64 { v * resolution as f64 };
    let to_int = |v: f64| -> i32 {
        let scaled = to_units(v);
        round_board_units(scaled)
    };
    // KiCad's Y axis points down; the board's points up (Java negates too)
    let point = |p: Option<&Json>| -> IntPoint {
        IntPoint::new(
            p.map(|q| round_board_units(to_units(q.num("x"))))
                .unwrap_or(0),
            p.map(|q| round_board_units(-to_units(q.num("y"))))
                .unwrap_or(0),
        )
    };

    // layers: everything that is not a plane is a signal layer
    let mut layers = Vec::new();
    for l in doc.arr("layers") {
        layers.push(Layer::new(
            l.str_or("name", "?"),
            !l.str_or("type", "signal").eq_ignore_ascii_case("plane"),
        ));
    }
    if layers.is_empty() {
        layers.push(Layer::new("F.Cu", true));
        layers.push(Layer::new("B.Cu", true));
    }
    let layer_count = layers.len();
    let layer_map = JsonLayerMap::from_document(&doc, layer_count)?;
    let stack = LayerStructure::new(layers);

    // clearance matrix: classes null, default, then one per net class;
    // 0.2 mm is the Java fallback for every unset pair. Each JSON net class
    // maps to a matrix column, REUSING the built-in "null"/"default" columns
    // for classes of those names instead of appending duplicates: a duplicate
    // "default" made the clearanceRules resolve to the first column while nets
    // used the appended one, silently dropping the round-tripped clearance.
    let net_classes = doc.arr("netClasses");
    let mut class_names: Vec<String> = vec!["null".into(), "default".into()];
    for entry in doc.arr("clearanceMatrix") {
        let name = entry.str_or("name", "");
        if !name.is_empty()
            && !class_names
                .iter()
                .any(|known| known.eq_ignore_ascii_case(&name))
        {
            class_names.push(name);
        }
    }
    let mut matrix_cl: Vec<usize> = Vec::with_capacity(net_classes.len());
    for c in net_classes {
        let name = c.str_or("name", "?");
        let idx = class_names
            .iter()
            .position(|n| n.eq_ignore_ascii_case(&name))
            .unwrap_or_else(|| {
                class_names.push(name.clone());
                class_names.len() - 1
            });
        matrix_cl.push(idx);
    }
    let name_refs: Vec<&str> = class_names.iter().map(|s| s.as_str()).collect();
    let mut matrix = ClearanceMatrix::new(stack.clone(), &name_refs);
    matrix.set_default_value(to_int(match unit_key.as_str() {
        "mil" => 0.2 / 0.0254,
        "um" => 200.0,
        "inch" => 0.2 / 25.4,
        "cm" => 0.02,
        _ => 0.2, // mm
    }));
    for (i, c) in net_classes.iter().enumerate() {
        let cl_no = matrix_cl[i];
        let val = to_int(c.num("clearance"));
        if val > 0 && cl_no >= 1 {
            matrix.set_value_on_all_layers(cl_no, cl_no, val);
            // set BOTH directions: the matrix must stay symmetric like Java's,
            // or a later get_value in the transposed direction reads a stale
            // default — the asymmetry behind finding #5c.
            matrix.set_value_on_all_layers(1, cl_no, val);
            matrix.set_value_on_all_layers(cl_no, 1, val);
        }
    }
    for rule in doc.arr("clearanceRules") {
        let (a, b) = (
            matrix.get_no(&rule.str_or("classA", "")),
            matrix.get_no(&rule.str_or("classB", "")),
        );
        if let (Some(a), Some(b)) = (a, b) {
            let v = to_int(rule.num("clearance"));
            matrix.set_value_on_all_layers(a, b, v);
            matrix.set_value_on_all_layers(b, a, v);
        }
    }
    // Restore complete rows after scalar net-class and pair rules have been
    // read. Older documents simply omit this section and retain legacy
    // scalar behavior.
    for entry in doc.arr("clearanceMatrix") {
        let Some(row) = matrix.get_no(&entry.str_or("name", "")) else {
            continue;
        };
        for (layer, values) in entry.arr("layers").iter().enumerate() {
            for (column, value) in values.as_arr().unwrap_or(&[]).iter().enumerate() {
                if column < matrix.get_class_count() && layer < matrix.get_layer_count() {
                    matrix.set_value(row, column, layer, to_int(value.as_f64().unwrap_or(0.0)));
                }
            }
        }
    }
    let mut rules = BoardRules::new(stack.clone(), matrix);

    // net classes: trace widths and clearance classes per class. A "default"
    // (or "null") JSON class updates the built-in default net class in place
    // rather than creating a duplicate; genuinely new classes are appended.
    let default_class = rules.get_default_net_class();
    let mut class_index: Vec<(String, usize)> = vec![("default".to_string(), default_class)];
    let mut net_class_of: Vec<usize> = Vec::with_capacity(net_classes.len());
    for (i, c) in net_classes.iter().enumerate() {
        let name = c.str_or("name", "?");
        let cl_no = matrix_cl[i];
        let hw = to_int(c.num("traceWidth")) / 2;
        let nc = if cl_no <= 1 {
            if hw > 0 {
                rules
                    .net_classes
                    .get_mut(default_class)
                    .set_trace_half_width(hw);
            }
            let resolved = cl_no.max(1);
            let class = rules.net_classes.get_mut(default_class);
            class.set_trace_clearance_class(resolved);
            // KiCad's JSON class clearance is scalar. It governs every item
            // kind in the class, not traces alone.
            class.default_item_clearance_classes.set_all(resolved);
            default_class
        } else {
            let class = rules.append_net_class(&name);
            if hw > 0 {
                rules.net_classes.get_mut(class).set_trace_half_width(hw);
            }
            let net_class = rules.net_classes.get_mut(class);
            net_class.set_trace_clearance_class(cl_no);
            net_class.default_item_clearance_classes.set_all(cl_no);
            class_index.push((name, class));
            class
        };
        net_class_of.push(nc);
    }
    if let Some(&first) = net_class_of.first() {
        // the default class mirrors the first document class (Java keeps
        // 0.25 mm defaults; mirroring gives unclassed nets sane widths)
        let hw = rules.net_classes.get(first).get_trace_half_width(0);
        rules.set_default_trace_half_widths(hw);
    }
    // The public fields above remain the backward-compatible fallback. Our
    // writer adds a complete class extension; apply it after scalar default
    // mirroring so layer-specific values on the default class are not flattened.
    for (index, class_node) in net_classes.iter().enumerate() {
        if let Some(metadata) = class_node.get("routingMetadata") {
            apply_net_class_routing_metadata(
                &mut rules,
                net_class_of[index],
                metadata,
                layer_count,
                resolution,
            )?;
        }
    }
    // Keep BoardRules' cached default-width extrema coherent with restored
    // per-layer widths. The setter is idempotent for the class value itself.
    for layer in 0..layer_count {
        let width = rules
            .net_classes
            .get(default_class)
            .get_trace_half_width(layer);
        if width > 0 {
            rules.set_default_trace_half_width_on_layer(layer, width);
        }
    }
    apply_board_routing_metadata(&mut rules, doc.get("freeroutingRules"), resolution)?;

    // via padstacks and rules: a default via plus one per net class
    // (Java fallback: 0.8 mm diameter)
    let mut padstacks = Padstacks::new(layer_count);
    let default_via_mm: f64 = match unit_key.as_str() {
        "mil" => 30.0,
        "um" => 800.0,
        "inch" => 0.8 / 25.4,
        "cm" => 0.08,
        _ => 0.8, // mm
    };
    let def_via_d = net_classes
        .iter()
        .find(|c| c.str_or("name", "").eq_ignore_ascii_case("default"))
        .map(|c| c.num("viaDiameter"))
        .filter(|&d| d > 0.0)
        .map(to_int)
        .unwrap_or_else(|| to_int(default_via_mm));
    let via_shapes = |d: i32| -> Vec<Option<TileShape>> {
        let r = d / 2;
        (0..layer_count)
            .map(|_| Some(TileShape::Box(IntBox::from_coords(-r, -r, r, r))))
            .collect()
    };
    let add_via_rule = |rules: &mut BoardRules,
                        padstacks: &mut Padstacks,
                        name: &str,
                        diameter: i32,
                        class: usize| {
        let ps = padstacks.add(name.to_string(), via_shapes(diameter), true, false);
        let cl = rules
            .net_classes
            .get(class)
            .default_item_clearance_classes
            .get(crate::rules::ItemClass::Via);
        if let Some(info) = rules.via_infos.add(ViaInfo::new(name, ps, cl, true)) {
            let mut rule = ViaRule::new(name);
            rule.append_via(info);
            rules.via_rules.push(rule);
            let rule_id = rules.via_rules.len() - 1;
            rules.net_classes.get_mut(class).set_via_rule(Some(rule_id));
        }
    };
    // A document carrying the explicit extension below already has the full
    // ViaInfo/ViaRule graph. Do not seed generated defaults first: doing so
    // would make a metadata padstack collide with the synthetic one and
    // would leave classes bound to the wrong rule on reload. Legacy JSON
    // without the extension keeps the historical scalar fallback.
    // Empty extension arrays are what a board with no explicit via graph
    // serializes. Treat that shape like legacy JSON so the normal scalar
    // fallback still creates a usable default via rule; a non-empty array
    // opts into strict metadata reconstruction below.
    let has_via_metadata = !doc.arr("viaInfos").is_empty() || !doc.arr("viaRules").is_empty();
    if !has_via_metadata {
        add_via_rule(
            &mut rules,
            &mut padstacks,
            "default_via",
            def_via_d,
            default_class,
        );
        for (i, c) in net_classes.iter().enumerate() {
            let name = c.str_or("name", "?");
            if name.eq_ignore_ascii_case("default") {
                continue;
            }
            let d = Some(c.num("viaDiameter"))
                .filter(|&d| d > 0.0)
                .map(to_int)
                .unwrap_or(def_via_d);
            let class = net_class_of[i];
            add_via_rule(&mut rules, &mut padstacks, &format!("via_{name}"), d, class);
        }
    }

    // nets, assigned to their class; nets only referenced by pads, zones,
    // traces or vias are registered with the default class (Java does both)
    for n in doc.arr("nets") {
        let name = n.str_or("name", "");
        if name.is_empty() {
            continue;
        }
        let plane = n.get("containsPlane") == Some(&Json::Bool(true));
        let net_no = rules.nets.add(&name, 1, plane);
        let class_name = n.str_or("className", "");
        let class = class_index
            .iter()
            .find(|(n, _)| n.eq_ignore_ascii_case(&class_name))
            .map(|(_, c)| *c)
            .unwrap_or(default_class);
        if let Some(net) = rules.nets.get_by_no_mut(net_no) {
            net.set_class(class);
        }
    }
    let mut referenced: Vec<String> = Vec::new();
    for comp in doc.arr("components") {
        for pad in comp.arr("pads") {
            referenced.push(pad.str_or("netName", ""));
        }
    }
    for section in ["conductionAreas", "traces", "vias"] {
        for e in doc.arr(section) {
            referenced.push(e.str_or("netName", ""));
        }
    }
    for name in referenced {
        if !name.is_empty() && rules.nets.get_by_name(&name).is_empty() {
            let net_no = rules.nets.add(&name, 1, false);
            if let Some(net) = rules.nets.get_by_no_mut(net_no) {
                net.set_class(default_class);
            }
        }
    }
    let net_no_by_name = |rules: &BoardRules, name: &str| -> Option<i32> {
        rules.nets.get_by_name(name).first().map(|n| n.net_number)
    };

    let mut board = BasicBoard::new(stack, rules, padstacks);
    board.resolution = resolution;
    // the board scale is `resolution` units per DOCUMENT unit: persist the
    // document's unit token (board_units_per_mm() understands mil/um/mm),
    // or all mm reporting would mis-scale for MIL/UM documents
    board.unit = unit_key.clone();

    // The scalar KiCad net-class fields cannot describe Freerouting's
    // ordered ViaInfo/ViaRule graph.  Newer Rust JSON exports therefore carry
    // an explicit metadata extension.  Read it before components/traces so a
    // routed via can reuse the exact named padstack instead of creating a
    // second anonymous one.  Documents without the extension retain the
    // generated scalar defaults above.
    let mut json_via_info_ids: HashMap<String, usize> = HashMap::new();
    if has_via_metadata {
        for info_node in doc.arr("viaInfos") {
            let name = info_node.str_or("name", "");
            if name.is_empty() {
                return Err("viaInfos entry is missing a non-empty name".into());
            }
            let padstack_name = info_node.str_or("padstackName", "");
            if padstack_name.is_empty() {
                return Err(format!("viaInfo {name:?} is missing padstackName"));
            }
            let from = json_layer_index(info_node, "startLayerIndex", 0, &layer_map)?;
            let to = json_layer_index(
                info_node,
                "endLayerIndex",
                layer_count.saturating_sub(1),
                &layer_map,
            )?;
            if to < from {
                return Err(format!("viaInfo {name:?} has a reversed layer span"));
            }
            let diameter = Some(info_node.num("diameter"))
                .filter(|&value| value > 0.0)
                .map(to_int)
                .unwrap_or(def_via_d)
                .max(2);
            let shapes = if info_node.get("layerShapes").is_some() {
                parse_layer_shapes(
                    info_node.get("layerShapes"),
                    &layer_map,
                    from,
                    to,
                    |corner| point(Some(corner)),
                )?
                .ok_or_else(|| format!("viaInfo {name:?} has no layerShapes"))?
            } else {
                let radius = diameter / 2;
                (0..layer_count)
                    .map(|layer| {
                        (from..=to).contains(&layer).then(|| {
                            TileShape::Box(IntBox::from_coords(-radius, -radius, radius, radius))
                        })
                    })
                    .collect()
            };
            let padstack_no = board
                .padstacks
                .get(&padstack_name)
                .map(|padstack| padstack.no)
                .unwrap_or_else(|| {
                    board
                        .padstacks
                        .add(padstack_name.clone(), shapes, true, false)
                });
            let clearance_class = match info_node
                .get("clearanceClass")
                .and_then(|value| value.as_str())
            {
                Some(class_name) => resolve_json_item_clearance_class(
                    &mut board,
                    class_name,
                    to_int(info_node.num("clearanceValue")),
                ),
                None => BoardRules::default_clearance_class(),
            };
            let attach_allowed = matches!(info_node.get("attachAllowed"), Some(Json::Bool(true)));
            let info_id = if let Some(existing) = board.rules.via_infos.get_by_name(&name) {
                let info = board.rules.via_infos.get_mut(existing);
                info.set_padstack(padstack_no);
                info.set_clearance_class(clearance_class);
                info.set_attach_smd_allowed(attach_allowed);
                existing
            } else {
                board
                    .rules
                    .via_infos
                    .add(ViaInfo::new(
                        name.clone(),
                        padstack_no,
                        clearance_class,
                        attach_allowed,
                    ))
                    .ok_or_else(|| format!("duplicate viaInfo name {name:?}"))?
            };
            json_via_info_ids.insert(name, info_id);
        }
        for rule_node in doc.arr("viaRules") {
            let name = rule_node.str_or("name", "");
            if name.is_empty() {
                return Err("viaRules entry is missing a non-empty name".into());
            }
            let mut rule = ViaRule::new(name.clone());
            for info_name in rule_node.arr("viaInfoNames") {
                let Some(info_name) = info_name.as_str() else {
                    return Err(format!(
                        "viaRule {name:?} contains a non-string viaInfo name"
                    ));
                };
                let info_id = json_via_info_ids
                    .get(info_name)
                    .copied()
                    .or_else(|| board.rules.via_infos.get_by_name(info_name))
                    .ok_or_else(|| {
                        format!("viaRule {name:?} references unknown viaInfo {info_name:?}")
                    })?;
                rule.append_via(info_id);
            }
            if let Some(existing) = board
                .rules
                .via_rules
                .iter()
                .position(|candidate| candidate.name == name)
            {
                board.rules.via_rules[existing] = rule;
            } else {
                board.rules.via_rules.push(rule);
            }
        }
        // Net classes carry the rule name separately so a class whose rule
        // has no routed vias still retains its complete ordered candidate set.
        for class_node in net_classes {
            let class_name = class_node.str_or("name", "");
            let Some(class_id) = board.rules.net_classes.get_by_name(&class_name) else {
                continue;
            };
            match class_node.get("viaRuleName") {
                Some(Json::Null) => board.rules.net_classes.get_mut(class_id).set_via_rule(None),
                Some(Json::Str(rule_name)) => {
                    let rule_id = board
                        .rules
                        .via_rules
                        .iter()
                        .position(|rule| rule.name == *rule_name)
                        .ok_or_else(|| {
                            format!(
                                "net class {class_name:?} references unknown viaRule {rule_name:?}"
                            )
                        })?;
                    board
                        .rules
                        .net_classes
                        .get_mut(class_id)
                        .set_via_rule(Some(rule_id));
                }
                Some(_) => {
                    return Err(format!(
                        "net class {class_name:?} has a non-string viaRuleName"
                    ));
                }
                None => {}
            }
        }
    }

    // components and pads: each pad becomes a system-fixed pin
    let mut component_no = 0i32;
    for comp in doc.arr("components") {
        component_no += 1;
        let pos = comp.get("position");
        let (px, py) = (
            pos.map(|p| to_units(p.num("x"))).unwrap_or(0.0),
            pos.map(|p| to_units(p.num("y"))).unwrap_or(0.0),
        );
        let rot = comp.num("rotation").to_radians();
        let (sin, cos) = rot.sin_cos();
        for pad in comp.arr("pads") {
            let off = pad.get("offset");
            let (ox, oy) = (
                off.map(|p| to_units(p.num("x"))).unwrap_or(0.0),
                off.map(|p| to_units(p.num("y"))).unwrap_or(0.0),
            );
            // the pin location in board coordinates: the component applies
            // the rotation to the pad offset, then Y is negated
            let x = round_board_units(px + ox * cos - oy * sin);
            let y = round_board_units(-py - ox * sin - oy * cos);
            let size = pad.get("size");
            let (dx, dy) = (
                size.map(|p| to_units(p.num("x"))).unwrap_or(0.0).max(2.0) / 2.0,
                size.map(|p| to_units(p.num("y"))).unwrap_or(0.0).max(2.0) / 2.0,
            );
            let shape = match pad.str_or("shape", "rect").to_ascii_lowercase().as_str() {
                "circle" => {
                    let r = dx.min(dy);
                    oval_shape(r, r, &round_board_units)
                }
                "oval" => oval_shape(dx, dy, &round_board_units),
                _ => TileShape::Box(IntBox::from_coords(
                    round_board_units(-dx),
                    round_board_units(-dy),
                    round_board_units(dx),
                    round_board_units(dy),
                )),
            };
            // pad layer span from the named layers; empty means all layers
            let named: Vec<usize> = pad
                .arr("layers")
                .iter()
                .filter_map(|l| l.as_str())
                .filter_map(|n| board.layer_structure.get_no(n))
                .collect();
            let (from_layer, to_layer) = if named.is_empty() {
                (0, layer_count - 1)
            } else {
                (*named.iter().min().unwrap(), *named.iter().max().unwrap())
            };
            let shapes = if let Some(layer_shapes) = parse_layer_shapes(
                pad.get("layerShapes"),
                &layer_map,
                from_layer,
                to_layer,
                |corner| point(Some(corner)),
            )? {
                layer_shapes
            } else {
                (0..layer_count)
                    .map(|l| (from_layer..=to_layer).contains(&l).then(|| shape.clone()))
                    .collect()
            };
            let drillable = pad.num("drill") > 0.0;
            let ps_no = board.padstacks.add(
                format!("padstack_{}", board.padstacks.count() + 1),
                shapes,
                drillable,
                false,
            );
            let net = net_no_by_name(&board.rules, &pad.str_or("netName", ""));
            // Resolve the item-specific class after the net class has been
            // assigned. A drillable pad and an SMD pad are intentionally
            // distinct in Java's default-item matrix; using the trace class
            // for both silently weakens pad/via clearances on reload.
            let pad_class = if drillable {
                ItemClass::Pin
            } else {
                ItemClass::Smd
            };
            let default_pad_cl = board
                .rules
                .item_clearance_class_for(net.unwrap_or(0), pad_class);
            let (pad_cl, pad_explicit) = match pad.get("clearanceClass").and_then(|v| v.as_str()) {
                Some(name) => (
                    resolve_json_item_clearance_class(
                        &mut board,
                        name,
                        to_int(pad.num("clearanceValue")),
                    ),
                    match pad.get("clearanceExplicit") {
                        Some(Json::Bool(value)) => *value,
                        // Legacy JSON with a class field had no provenance bit;
                        // retain the conservative explicit interpretation.
                        _ => true,
                    },
                ),
                None => (default_pad_cl, false),
            };
            let mut base = ItemBase::new(
                component_no,
                net.map(|n| vec![n]).unwrap_or_default(),
                pad_cl,
            );
            base.component_no = component_no;
            base.clearance_class_explicit = pad_explicit;
            base.fixed_state = parse_fixed_state(pad.get("fixedState"), FixedState::SystemFixed)?;
            let item = Item::new_via(base, ps_no, IntPoint::new(x, y), true);
            board.insert_item(item);
        }
    }

    // conduction areas (copper pours)
    for zone in doc.arr("conductionAreas") {
        let corners: Vec<Point> = zone
            .arr("polygon")
            .iter()
            .map(|p| Point::Int(point(Some(p))))
            .collect();
        if corners.len() < 3 {
            continue;
        }
        let layer = json_layer_index(zone, "layerIndex", 0, &layer_map)?;
        // `name` is the Freerouting display/name extension for netless
        // obstacles.  `netName` is the electrical identity and must win when
        // both fields are present: a named conduction area is allowed to use
        // a human-readable name different from its net.  Legacy documents
        // without `netName` fall back to `name` for compatibility.
        let name = zone
            .get("name")
            .and_then(Json::as_str)
            .map(str::to_owned)
            .unwrap_or_else(|| zone.str_or("netName", ""));
        let net_name = zone
            .get("netName")
            .and_then(Json::as_str)
            .unwrap_or(name.as_str());
        let net = net_no_by_name(&board.rules, net_name);
        // an explicit clearance class (by matrix name) wins; a class the
        // matrix lacks (matrix-only classes are not among the net-class
        // columns) is CREATED from the zone's own clearanceValue — the
        // failed lookup used to silently fall back to default (Issue143's
        // boundary zones)
        let zone_has_explicit_class = match zone.get("clearanceExplicit") {
            Some(Json::Bool(value)) => *value,
            _ => zone
                .get("clearanceClass")
                .and_then(|v| v.as_str())
                .is_some(),
        };
        let zone_cl = match zone.get("clearanceClass").and_then(|v| v.as_str()) {
            Some(cl_name) => match board.rules.clearance_matrix.get_no(cl_name) {
                Some(idx) => idx,
                None => {
                    board.rules.clearance_matrix.append_class(cl_name);
                    let idx = board
                        .rules
                        .clearance_matrix
                        .get_no(cl_name)
                        .unwrap_or_else(BoardRules::default_clearance_class);
                    let v = to_int(zone.num("clearanceValue"));
                    if v > 0 && idx > 1 {
                        let count = board.rules.clearance_matrix.get_class_count();
                        for j in 1..count {
                            board
                                .rules
                                .clearance_matrix
                                .set_value_on_all_layers(idx, j, v);
                            board
                                .rules
                                .clearance_matrix
                                .set_value_on_all_layers(j, idx, v);
                        }
                    }
                    idx
                }
            },
            // A zone is area copper, not a trace.  Legacy JSON omitted the
            // explicit class/value fields, but it still must inherit the
            // net class's Area item rule (and the default Area rule for a
            // netless keepout), just like DSN import does.
            None => board
                .rules
                .item_clearance_class_for(net.unwrap_or(0), crate::rules::ItemClass::Area),
        };
        let area = PolylineArea::new(PolygonShape::new(corners), Vec::new());
        let id = board.insert_area(
            area,
            layer,
            &name,
            net.map(|n| vec![n]).unwrap_or_default(),
            zone_cl,
            // with a net it is a connectable pour; without, a keepout
            net.is_some(),
        );
        if zone_has_explicit_class {
            board.set_item_clearance_class_explicit(id, true);
        }
        // a conduction pour flagged obstacle in the JSON also enforces
        // clearance against foreign-net copper (Java ConductionArea.is_obstacle)
        if net.is_some() && zone.get("isObstacle") == Some(&Json::Bool(true)) {
            board.set_area_is_obstacle(id, true);
        }
        // a netless zone is a keepout: honor the via-only flag and fix it
        // like the DSN importer's keepouts
        let default_fixed_state = if net.is_none() {
            FixedState::SystemFixed
        } else {
            FixedState::UserFixed
        };
        board.set_fixed_state(
            id,
            parse_fixed_state(zone.get("fixedState"), default_fixed_state)?,
        );
        if net.is_none() && zone.get("viaOnly") == Some(&Json::Bool(true)) {
            board.set_area_via_only(id, true);
        }
    }

    // pre-routed traces and vias, user-fixed like Java
    for t in doc.arr("traces") {
        let net = net_no_by_name(&board.rules, &t.str_or("netName", ""));
        let layer = json_layer_index(t, "layerIndex", 0, &layer_map)?;
        let hw = (to_int(t.num("width")) / 2).max(1);
        let corners: Vec<IntPoint> = t.arr("points").iter().map(|p| point(Some(p))).collect();
        if corners.len() < 2 {
            continue;
        }
        let polyline = Polyline::from_int_points(&corners);
        if !polyline.is_empty() {
            let (cl, explicit) = match t.get("clearanceClass").and_then(|v| v.as_str()) {
                Some(name) => (
                    resolve_json_item_clearance_class(
                        &mut board,
                        name,
                        to_int(t.num("clearanceValue")),
                    ),
                    matches!(t.get("clearanceExplicit"), Some(Json::Bool(true))),
                ),
                None => (
                    net.map(|net_no| board.rules.get_trace_clearance_class(net_no))
                        .unwrap_or_else(BoardRules::default_clearance_class),
                    false,
                ),
            };
            let id = board.insert_trace_with_provenance(
                polyline,
                layer,
                hw,
                net.into_iter().collect(),
                cl,
                explicit,
            );
            board.set_fixed_state(
                id,
                parse_fixed_state(t.get("fixedState"), FixedState::UserFixed)?,
            );
        }
    }
    for v in doc.arr("vias") {
        let net = net_no_by_name(&board.rules, &v.str_or("netName", ""));
        let is_escape = matches!(v.get("isEscapeVia"), Some(Json::Bool(true)));
        if !is_escape
            && v.get("escapeSmdLayer")
                .is_some_and(|value| !matches!(value, Json::Null))
        {
            return Err("escapeSmdLayer requires isEscapeVia=true".into());
        }
        let requested_escape = if is_escape {
            let Some(raw_layer) = v.get("escapeSmdLayer").and_then(Json::as_exact_u64) else {
                return Err("isEscapeVia requires an exact non-negative escapeSmdLayer".into());
            };
            Some(layer_map.resolve(raw_layer, "escapeSmdLayer")?)
        } else {
            None
        };
        let center = point(v.get("position"));
        let info_name = v
            .get("viaInfoName")
            .and_then(|value| value.as_str())
            .filter(|name| !name.is_empty());
        let metadata_info_id = info_name.and_then(|name| json_via_info_ids.get(name).copied());
        let d = Some(v.num("diameter"))
            .filter(|&d| d > 0.0)
            .map(to_int)
            .unwrap_or(def_via_d);
        let requested_from = json_layer_index(v, "startLayerIndex", 0, &layer_map)?;
        let requested_to = json_layer_index(v, "endLayerIndex", layer_count - 1, &layer_map)?;
        if requested_to < requested_from {
            return Err("endLayerIndex must not precede startLayerIndex".into());
        }
        let (from, to) = if let Some(info_id) = metadata_info_id {
            let padstack_no = board.rules.via_infos.get(info_id).get_padstack();
            let padstack = board
                .padstacks
                .get_by_no(padstack_no)
                .ok_or_else(|| format!("viaInfo {info_name:?} references a missing padstack"))?;
            (padstack.from_layer(), padstack.to_layer())
        } else {
            (requested_from, requested_to)
        };
        if to < from {
            return Err("endLayerIndex must not precede startLayerIndex".into());
        }
        let ps = if let Some(info_id) = metadata_info_id {
            board.rules.via_infos.get(info_id).get_padstack()
        } else {
            let r = d / 2;
            let padstack_name = format!("json_via_{}", board.padstacks.count() + 1);
            let mut shapes = vec![None; layer_count];
            let mut seen_shape_layer = vec![false; layer_count];
            for layer_shape in v.arr("layerShapes") {
                let layer = json_layer_index(layer_shape, "layerIndex", 0, &layer_map)?;
                if seen_shape_layer[layer] {
                    return Err("layerShapes contains a duplicate layerIndex".into());
                }
                seen_shape_layer[layer] = true;
                let corners: Vec<IntPoint> = layer_shape
                    .arr("corners")
                    .iter()
                    .map(|corner| point(Some(corner)))
                    .collect();
                if corners.len() < 3 {
                    return Err("each layerShapes polygon needs at least three corners".into());
                }
                shapes[layer] = Some(TileShape::from_convex_polygon(&corners));
            }
            if shapes.iter().any(Option::is_some) {
                if shapes
                    .iter()
                    .enumerate()
                    .any(|(layer, shape)| shape.is_some() && !(from..=to).contains(&layer))
                {
                    return Err("layerShapes must stay inside the declared via layer span".into());
                }
                if shapes[from].is_none() || shapes[to].is_none() {
                    return Err("layerShapes must describe both endpoints of the via span".into());
                }
            }
            if shapes.iter().any(Option::is_some) {
                board.padstacks.add(padstack_name, shapes, true, false)
            } else {
                board.padstacks.add_shape_on_layers(
                    TileShape::Box(IntBox::from_coords(-r, -r, r, r)),
                    from,
                    to,
                )
            }
        };
        let (cl, explicit) = match v.get("clearanceClass").and_then(|x| x.as_str()) {
            Some(name) => (
                resolve_json_item_clearance_class(
                    &mut board,
                    name,
                    to_int(v.num("clearanceValue")),
                ),
                matches!(v.get("clearanceExplicit"), Some(Json::Bool(true))),
            ),
            None => (
                net.map(|net_no| board.rules.item_clearance_class_for(net_no, ItemClass::Via))
                    .unwrap_or_else(BoardRules::default_clearance_class),
                false,
            ),
        };
        let attach_allowed = matches!(v.get("attachAllowed"), Some(Json::Bool(true)));
        let escape_layer = if let Some(layer) = requested_escape {
            let Some(net_no) = net else {
                return Err("a netless via cannot carry an SMD escape marker".into());
            };
            if !(from..=to).contains(&layer)
                || board.pure_smd_escape_layer(net_no, ps, center) != Some(layer)
            {
                return Err(format!(
                    "isEscapeVia marker at ({}, {}) does not describe a valid same-net SMD escape",
                    center.x, center.y
                ));
            }
            Some(layer)
        } else {
            None
        };
        let id = if let Some(layer) = escape_layer {
            board.insert_escape_via_with_provenance(
                ps,
                center,
                net.into_iter().collect(),
                cl,
                attach_allowed,
                layer,
                explicit,
            )
        } else {
            board.insert_via_with_provenance(
                ps,
                center,
                net.into_iter().collect(),
                cl,
                attach_allowed,
                explicit,
            )
        };
        // Preserve the selected ViaInfo and its net-class rule when the
        // writer supplied them. Otherwise future routing after a JSON round
        // trip silently falls back to a generated default via.
        if let Some(info_name) = info_name.filter(|_| metadata_info_id.is_none()) {
            let info_id = if let Some(existing) = board.rules.via_infos.get_by_name(info_name) {
                let info = board.rules.via_infos.get_mut(existing);
                info.set_padstack(ps);
                info.set_clearance_class(cl);
                info.set_attach_smd_allowed(attach_allowed);
                existing
            } else {
                board
                    .rules
                    .via_infos
                    .add(ViaInfo::new(info_name, ps, cl, attach_allowed))
                    .unwrap_or_else(|| board.rules.via_infos.get_by_name(info_name).unwrap_or(0))
            };
            if let Some(rule_name) = v
                .get("viaRuleName")
                .and_then(|value| value.as_str())
                .filter(|name| !name.is_empty())
            {
                let rule_id = if let Some(existing) = board
                    .rules
                    .via_rules
                    .iter()
                    .position(|rule| rule.name.eq_ignore_ascii_case(rule_name))
                {
                    existing
                } else {
                    board.rules.via_rules.push(ViaRule::new(rule_name));
                    board.rules.via_rules.len() - 1
                };
                if !board.rules.via_rules[rule_id].contains(info_id) {
                    board.rules.via_rules[rule_id].append_via(info_id);
                }
                if let Some(net_class) = net
                    .and_then(|net_no| board.rules.nets.get_by_no(net_no))
                    .map(|entry| entry.get_class())
                {
                    board
                        .rules
                        .net_classes
                        .get_mut(net_class)
                        .set_via_rule(Some(rule_id));
                }
            }
        }
        board.set_fixed_state(
            id,
            parse_fixed_state(v.get("fixedState"), FixedState::UserFixed)?,
        );
        // register contacts when the via lands mid-trace (a contact
        // needs a trace endpoint at the pad)
        board.split_traces_at_via(id);
    }

    // board outline: keepout strips along the closed corner list on every
    // layer, exactly like the DSN boundary — without them an outline-only
    // reload was unconfined and routes could leave the board
    if let Some(outline) = doc.get("outline") {
        if !matches!(outline, Json::Obj(_)) {
            return Err("outline must be an object".into());
        }
        let corner_value = outline.get("corners");
        let corner_entries = match corner_value {
            None => return Err("outline.corners is missing".into()),
            Some(Json::Arr(entries)) => entries,
            Some(_) => return Err("outline.corners must be an array".into()),
        };
        // A few legacy documents carry an empty outline object as a marker for
        // “derive the bounds”; retain that compatibility behavior. Any
        // nonempty outline must, however, describe finite in-range points.
        if !corner_entries.is_empty() {
            let corners: Vec<IntPoint> = corner_entries
                .iter()
                .enumerate()
                .map(|(index, p)| {
                    let x = p
                        .get("x")
                        .and_then(Json::as_f64)
                        .ok_or_else(|| format!("outline.corners[{index}].x must be numeric"))?;
                    let y = p
                        .get("y")
                        .and_then(Json::as_f64)
                        .ok_or_else(|| format!("outline.corners[{index}].y must be numeric"))?;
                    let sx = x * f64::from(resolution);
                    // KiCad document coordinates grow downwards; all board
                    // geometry, including the preserved outline, uses the
                    // internal upward-positive Y axis.
                    let sy = -y * f64::from(resolution);
                    if !sx.is_finite()
                        || !sy.is_finite()
                        || sx < f64::from(i32::MIN)
                        || sx > f64::from(i32::MAX)
                        || sy < f64::from(i32::MIN)
                        || sy > f64::from(i32::MAX)
                    {
                        return Err(format!(
                            "outline.corners[{index}] is outside the board range"
                        ));
                    }
                    Ok(IntPoint::new(round_board_units(sx), round_board_units(sy)))
                })
                .collect::<Result<_, _>>()?;
            if corners.len() < 3 {
                return Err("outline.corners must contain at least three points".into());
            }
            let corners = normalize_outline_corners(corners);
            if corners.len() < 3 {
                return Err("outline.corners do not enclose an area".into());
            }
            let twice_area: i128 = corners
                .iter()
                .zip(corners.iter().cycle().skip(1))
                .take(corners.len())
                .map(|(a, b)| i128::from(a.x) * i128::from(b.y) - i128::from(b.x) * i128::from(a.y))
                .sum();
            if twice_area == 0 {
                return Err("outline.corners do not enclose an area".into());
            }
            // The document's own outline clearance decides the strip width;
            // a missing field is the only case that falls back to the board
            // default. In particular, an explicit zero is meaningful.
            let strip = match outline.get("clearance") {
                None => board.rules.clearance_matrix.get_value(1, 1, 0, false),
                Some(value) => {
                    let supplied = value
                        .as_f64()
                        .ok_or_else(|| "outline.clearance must be numeric".to_string())?;
                    let scaled = supplied * f64::from(resolution);
                    if !scaled.is_finite() || scaled < 0.0 || scaled > f64::from(i32::MAX) {
                        return Err("outline.clearance is outside the board range".into());
                    }
                    let rounded = scaled.round();
                    if rounded <= f64::from(i32::MIN) {
                        return Err("outline.clearance is outside the board range".into());
                    }
                    rounded as i32
                }
            };
            // Preserve the outline verbatim: the writer must emit THIS
            // polygon and clearance, not a bounding-box fabrication.
            board.outline = Some((corners.clone(), strip));
            crate::io::dsn_import::insert_boundary_keepouts(&mut board, &corners, strip);
        }
    }
    if let Some(error) = scale_error.into_inner() {
        return Err(error);
    }
    crate::board::validation::validate_board_references(&board)
        .map_err(|error| format!("KiCad board violates board invariants: {error}"))?;
    Ok(board)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::ItemKind;

    #[test]
    fn mil_documents_scale_by_units_per_document_unit() {
        // Java semantics: resolution = board units per DOCUMENT unit. A
        // 10-units/mil document once became 10-units/mm (40x coarser).
        let doc = r#"{
          "unit": "MIL",
          "resolution": 10,
          "layers": [{"index": 0, "name": "F.Cu", "type": "signal"}],
          "netClasses": [],
          "nets": [{"id": 1, "name": "N1", "className": "Default"}],
          "components": [],
          "traces": [{"netName": "N1", "layerIndex": 0, "width": 10,
                      "points": [{"x": 0, "y": 0}, {"x": 100, "y": 0}]}],
          "vias": []
        }"#;
        let board = import_kicad_json(doc).expect("import");
        assert_eq!(board.resolution, 10, "10 units per mil");
        assert_eq!(board.unit, "mil");
        let trace_bb = board
            .items()
            .find(|(_, it)| matches!(it.kind, ItemKind::PolylineTrace(_)))
            .map(|(_, it)| it.bounding_box(&board.padstacks))
            .expect("trace imported");
        // 100 mil * 10 units/mil = 1000 units (the mm detour gave 25)
        assert!(
            trace_bb.ur.x >= 1000,
            "100 mil must scale to 1000 board units, got bbox {trace_bb:?}"
        );
        // mm reporting: 10 units/mil = 10/25.4*1000 units per mm
        assert!((board.board_units_per_mm() - 10.0 / 25.4 * 1000.0).abs() < 1e-6);
    }

    #[test]
    fn scaled_geometry_outside_board_integer_range_is_rejected() {
        let document = MINI.replace(
            r#"{"x": 1.0, "y": 1.0}, {"x": 2.0, "y": 1.0}"#,
            r#"{"x": 1.0, "y": 1.0}, {"x": 300000.0, "y": 1.0}"#,
        );
        let error = import_kicad_json(&document)
            .expect_err("scaled KiCad coordinates must not saturate")
            .to_string();
        assert!(
            error.contains("does not fit the board coordinate range"),
            "{error}"
        );
    }

    #[test]
    fn rejects_unknown_units_and_invalid_resolutions() {
        let unknown_unit = MINI.replace("\"unit\": \"MM\"", "\"unit\": \"furlong\"");
        let err = import_kicad_json(&unknown_unit).expect_err("unknown unit must fail closed");
        assert!(err.contains("unsupported KiCad unit"), "{err}");

        let non_string_unit = MINI.replace("\"unit\": \"MM\"", "\"unit\": 7");
        let err =
            import_kicad_json(&non_string_unit).expect_err("non-string unit must fail closed");
        assert!(err.contains("unit must be a string"), "{err}");

        for raw in ["0", "-1", "1.5", "2147483648"] {
            let document = MINI.replacen(
                "\"unit\": \"MM\",",
                &format!("\"unit\": \"MM\", \"resolution\": {raw},"),
                1,
            );
            let err = import_kicad_json(&document)
                .expect_err("non-positive, fractional, and overflowing resolutions must fail");
            assert!(
                err.contains("resolution must be a positive integer"),
                "{raw}: {err}"
            );
        }

        let non_numeric = MINI.replacen(
            "\"unit\": \"MM\",",
            "\"unit\": \"MM\", \"resolution\": \"10000\",",
            1,
        );
        let err = import_kicad_json(&non_numeric).expect_err("string resolution must fail");
        assert!(err.contains("resolution must be a number"), "{err}");
    }

    #[test]
    fn outline_reload_confines_the_board() {
        // the writer emits an outline; the reader must turn it into
        // boundary keepout strips (an outline-only reload was unconfined)
        let doc = r#"{
          "unit": "MM",
          "layers": [
            {"index": 0, "name": "F.Cu", "type": "signal"},
            {"index": 1, "name": "B.Cu", "type": "signal"}
          ],
          "netClasses": [],
          "nets": [{"id": 1, "name": "N1", "className": "Default"}],
          "components": [],
          "outline": {"corners": [{"x": 0, "y": 0}, {"x": 50, "y": 0},
                                  {"x": 50, "y": -40}, {"x": 0, "y": -40}],
                      "clearance": 0.2},
          "traces": [],
          "vias": []
        }"#;
        let board = import_kicad_json(doc).expect("import");
        let boundary_strips = board
            .items()
            .filter(|(_, it)| matches!(&it.kind, ItemKind::ObstacleArea(a) if !a.is_conduction))
            .count();
        // 4 edges on each of the 2 layers
        assert_eq!(
            boundary_strips, 8,
            "the outline must become keepout strips on every layer"
        );
    }

    #[test]
    fn outline_clearance_is_not_counted_as_geometry_and_matrix_spacing() {
        let doc = r#"{
          "unit": "MM",
          "resolution": 10000,
          "layers": [{"index": 0, "name": "F.Cu", "type": "signal"}],
          "netClasses": [{"name": "Default", "clearance": 0.2,
                          "traceWidth": 0.2, "viaDiameter": 0.6,
                          "viaDrill": 0.3, "netNames": ["N1"]}],
          "nets": [{"id": 1, "name": "N1", "className": "Default"}],
          "components": [],
          "outline": {"corners": [{"x": 0, "y": 0}, {"x": 10, "y": 0},
                                    {"x": 10, "y": -10}, {"x": 0, "y": -10}],
                      "clearance": 0.5},
          "traces": [{"netName": "N1", "layerIndex": 0, "width": 0.2,
                      "points": [{"x": 1, "y": -0.6}, {"x": 2, "y": -0.6}]}],
          "vias": []
        }"#;
        let board = import_kicad_json(doc).expect("import");
        let report = crate::drc::check_board(&board);
        assert!(
            report.violations.is_empty(),
            "a trace whose copper edge is exactly 0.5 mm from a 0.5 mm outline must be legal: {:?}",
            report
                .violations
                .iter()
                .map(|v| (v.required_clearance, v.actual_distance))
                .collect::<Vec<_>>()
        );
    }

    const MINI: &str = r#"{
      "unit": "MM",
      "layers": [
        {"index": 0, "name": "F.Cu", "type": "signal"},
        {"index": 1, "name": "B.Cu", "type": "signal"}
      ],
      "netClasses": [{"name": "Default", "clearance": 0.2, "traceWidth": 0.25,
                      "viaDiameter": 0.6, "viaDrill": 0.3, "netNames": ["GND"]},
                     {"name": "Power", "clearance": 0.4, "traceWidth": 0.5,
                      "viaDiameter": 0.8, "viaDrill": 0.4, "netNames": ["VCC"]}],
      "clearanceRules": [{"classA": "Default", "classB": "Power", "clearance": 0.3}],
      "nets": [{"id": 1, "name": "GND", "className": "Default", "containsPlane": false},
               {"id": 2, "name": "VCC", "className": "Power", "containsPlane": true}],
      "components": [{
        "reference": "R1", "value": "10k", "footprint": "R_0603",
        "position": {"x": 10.0, "y": -10.0}, "rotation": 0, "layer": "F.Cu",
        "pads": [
          {"name": "1", "netName": "GND", "shape": "circle",
           "size": {"x": 0.8, "y": 0.8}, "offset": {"x": -0.75, "y": 0},
           "drill": 0, "layers": ["F.Cu"]},
          {"name": "2", "netName": "GND", "shape": "oval",
           "size": {"x": 1.2, "y": 0.8}, "offset": {"x": 0.75, "y": 0},
           "drill": 0, "layers": ["F.Cu"]}
        ]
      }],
      "outline": {"corners": [], "clearance": 0.2},
      "traces": [{"netName": "VCC", "width": 0.5, "layerIndex": 0,
                  "points": [{"x": 1.0, "y": 1.0}, {"x": 2.0, "y": 1.0}]}],
      "vias": [{"netName": "VCC", "position": {"x": 2.0, "y": 1.0},
                "diameter": 0.8, "drill": 0.4, "startLayerIndex": 0, "endLayerIndex": 1}],
      "conductionAreas": [{"netName": "GND", "layerIndex": 1, "isObstacle": true,
                           "polygon": [{"x": 0, "y": 0}, {"x": 30, "y": 0},
                                       {"x": 30, "y": 30}, {"x": 0, "y": 30}]}]
    }"#;

    #[test]
    fn imports_a_kicad_board() {
        let board = import_kicad_json(MINI).expect("import");
        assert_eq!(board.layer_structure.layer_count(), 2);
        assert_eq!(board.rules.nets.max_net_no(), 2);
        let pads = board
            .items()
            .filter(|(_, it)| it.base.component_no != 0)
            .count();
        assert_eq!(pads, 2, "two pads expected");
        // the two GND pads are unconnected by wiring, but the GND pour
        // on layer 1 does not reach them on layer 0
        assert!(!board.net_is_completely_connected(1));
    }

    #[test]
    fn rejects_duplicate_external_layer_ids() {
        let duplicate = MINI.replacen(
            r#"{"index": 1, "name": "B.Cu", "type": "signal"}"#,
            r#"{"index": 0, "name": "B.Cu", "type": "signal"}"#,
            1,
        );
        let error = import_kicad_json(&duplicate)
            .expect_err("external layer ids must be unique")
            .to_string();
        assert!(error.contains("duplicates layer index 0"), "{error}");
    }

    #[test]
    fn imports_issue649_legacy_external_back_copper_id() {
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/..");
        let content = std::fs::read_to_string(format!(
            "{root}/fixtures/Issue649-kicad_ecc83-pp_input_board_v2.json"
        ))
        .expect("fixture missing from checkout");
        let board = import_kicad_json(&content).expect("Issue649 must import");
        assert_eq!(board.layer_structure.layer_count(), 2);
        let traces: Vec<_> = board
            .items()
            .filter(|(_, item)| matches!(item.kind, ItemKind::PolylineTrace(_)))
            .collect();
        assert_eq!(traces.len(), 59, "every legacy trace must import");
        assert!(
            traces
                .iter()
                .all(|(_, item)| item.first_layer(&board.padstacks) == 1),
            "external layer id 2 must resolve to the declared B.Cu ordinal"
        );
    }

    #[test]
    fn net_class_rules_are_applied() {
        let board = import_kicad_json(MINI).expect("import");
        // VCC is in the Power class: 0.25 mm half width at resolution 10000
        assert_eq!(board.rules.get_trace_half_width(2, 0), 2500);
        assert_eq!(board.rules.get_trace_half_width(1, 0), 1250);
        // the custom rule sets 0.3 mm between the two classes
        let a = board.rules.clearance_matrix.get_no("Default").unwrap();
        let b = board.rules.clearance_matrix.get_no("Power").unwrap();
        assert_eq!(board.rules.clearance_matrix.get_value(a, b, 0, false), 3000);
        for item_class in [
            ItemClass::Trace,
            ItemClass::Via,
            ItemClass::Pin,
            ItemClass::Smd,
            ItemClass::Area,
        ] {
            assert_eq!(
                board.rules.item_clearance_class_for(2, item_class),
                b,
                "Power {item_class:?} must inherit the scalar KiCad class clearance"
            );
        }
        // per-class vias resolve through the via rules
        let vcc_via = board.rules.via_padstack_for_net(2).unwrap();
        let d = board.padstacks.get_by_no(vcc_via).unwrap().bounding_box();
        assert_eq!(d.ur.x - d.ll.x, 8000, "0.8 mm Power via");
    }

    #[test]
    fn zones_traces_and_vias_are_imported() {
        let board = import_kicad_json(MINI).expect("import");
        let zones = board
            .items()
            .filter(|(_, it)| matches!(&it.kind, ItemKind::ObstacleArea(a) if a.is_conduction))
            .count();
        assert_eq!(zones, 1, "the GND pour is a conduction area");
        let traces = board
            .items()
            .filter(|(_, it)| matches!(it.kind, ItemKind::PolylineTrace(_)))
            .count();
        assert_eq!(traces, 1);
        let vias = board
            .items()
            .filter(|(_, it)| it.base.component_no == 0 && matches!(it.kind, ItemKind::Via(_)))
            .count();
        assert_eq!(vias, 1);
        // KiCad Y points down: y=1.0 mm lands at board y = -10000
        let (_, via) = board
            .items()
            .find(|(_, it)| it.base.component_no == 0 && matches!(it.kind, ItemKind::Via(_)))
            .unwrap();
        let ItemKind::Via(v) = &via.kind else {
            unreachable!()
        };
        assert_eq!(v.center, IntPoint::new(20000, -10000));
    }

    #[test]
    fn zone_display_name_does_not_override_its_net_name() {
        // A zone's human-readable `name` and electrical `netName` are
        // independent.  Reloading by `name` turned this valid conduction
        // area into a netless keepout whenever the display name differed.
        let doc = MINI.replace(
            "\"conductionAreas\": [{\"netName\": \"GND\"",
            "\"conductionAreas\": [{\"name\": \"ground-pour\", \"netName\": \"GND\"",
        );
        let board = import_kicad_json(&doc).expect("import");
        let zone = board
            .items()
            .find_map(|(_, item)| match &item.kind {
                ItemKind::ObstacleArea(area) if area.is_conduction => Some((item, area)),
                _ => None,
            })
            .expect("conduction area");
        assert_eq!(zone.0.base.net_nos, vec![1]);
        assert_eq!(zone.1.name, "ground-pour");
    }

    #[test]
    fn outline_zero_clearance_is_preserved_and_malformed_shapes_fail() {
        let doc = MINI.replace(
            "\"outline\": {\"corners\": [], \"clearance\": 0.2}",
            "\"outline\": {\"corners\": [{\"x\": 0, \"y\": 0}, {\"x\": 10, \"y\": 0}, {\"x\": 10, \"y\": -10}], \"clearance\": 0}",
        );
        let board = import_kicad_json(&doc).expect("zero-clearance outline is valid");
        assert_eq!(board.outline.as_ref().expect("outline").1, 0);

        let malformed = MINI.replace(
            "\"outline\": {\"corners\": [], \"clearance\": 0.2}",
            "\"outline\": {\"corners\": [{\"x\": 0, \"y\": 0}, {\"x\": 1, \"y\": 0}], \"clearance\": 0.2}",
        );
        assert!(import_kicad_json(&malformed)
            .expect_err("a nonempty two-point outline must be rejected")
            .contains("at least three"));
    }

    #[test]
    fn malformed_collection_and_point_shapes_fail_closed() {
        assert!(import_kicad_json(r#"{"layers":"not-an-array"}"#)
            .expect_err("a present layers field must be an array")
            .contains("layers must be an array"));
        let missing_coordinate = r#"{
          "layers": [{"name":"F.Cu","type":"signal"},{"name":"B.Cu","type":"signal"}],
          "traces": [{"width":0.2,"layerIndex":0,"points":[{"x":0},{"x":1,"y":1}]}]
        }"#;
        assert!(import_kicad_json(missing_coordinate)
            .expect_err("geometry points require both coordinates")
            .contains(".y is required"));
    }
}
