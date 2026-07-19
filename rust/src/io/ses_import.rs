//! Port of `SesReader.java`: applying a Specctra session file's routes
//! (wires and vias from `network_out`) to a board imported from the
//! matching design.

use std::cell::RefCell;

use crate::board::basic_board::BasicBoard;
use crate::geometry::planar::{IntPoint, Polyline};
use crate::io::dsn::parse_dsn;
use crate::rules::ItemClass;

/// Resolves a session `(net NAME ...)` scope. Specctra's standard
/// `net_number` child is translator metadata and is deliberately ignored for
/// electrical selection (Cadence and Java use the net name). Freerouting's
/// namespaced child is the only subnet selector; this prevents an arbitrary
/// translator id from silently routing onto a nonexistent subnet.
fn session_net_numbers(
    rules: &crate::rules::BoardRules,
    net_node: &crate::io::dsn::SExpr,
) -> Result<Vec<i32>, String> {
    let Some(name) = net_node.args().next() else {
        return Err("session net scope is missing its name".into());
    };
    if net_node.args().nth(1).is_some() {
        return Err("session net scope has too many arguments".into());
    }
    let mut subnet_nodes = net_node.children("freerouting_subnet");
    let subnet = subnet_nodes
        .next()
        .map(|node| {
            let values: Vec<_> = node.args().collect();
            if values.len() != 1 {
                return Err("session freerouting_subnet needs exactly one integer".to_string());
            }
            values[0]
                .parse::<usize>()
                .ok()
                .filter(|value| *value > 0)
                .ok_or_else(|| "session freerouting_subnet must be a positive integer".to_string())
        })
        .transpose()?;
    if subnet_nodes.next().is_some() {
        return Err("session net scope has more than one freerouting_subnet".into());
    }
    let subnet = subnet.unwrap_or(1);
    Ok(rules
        .nets
        .get(name, subnet)
        .map(|net| vec![net.net_number])
        .unwrap_or_default())
}

#[derive(Debug)]
pub struct SesImportSummary {
    pub wires: usize,
    pub vias: usize,
    /// `net_out` scopes whose net name does not exist on the board; their
    /// copper is NOT imported (netless copper would be a pure obstacle and
    /// violate against every real net, like Java's SesReader skip).
    pub unknown_nets: Vec<String>,
}

/// The metadata that a standard SES route can recover from the matching base
/// design for one via.  SES carries only the padstack name and coordinates;
/// clearance, attach, and escape semantics therefore have to be derived from
/// the base board in exactly the same way on export and import.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct SesViaMetadata {
    pub(crate) clearance_class: usize,
    pub(crate) attach_allowed: bool,
    pub(crate) escape_smd_layer: Option<usize>,
}

pub(crate) fn session_trace_clearance_class(board: &BasicBoard, net_no: i32) -> usize {
    board.rules.get_trace_clearance_class(net_no)
}

pub(crate) fn session_via_metadata(
    board: &BasicBoard,
    net_no: i32,
    padstack_no: usize,
    center: IntPoint,
) -> SesViaMetadata {
    let trace_clearance_class = session_trace_clearance_class(board, net_no);
    let (clearance_class, attach_allowed) =
        match board.rules.via_info_for_padstack(net_no, padstack_no) {
            Some(info) => {
                let clearance_class = match info.get_clearance_class() {
                    // ViaInfo class 0 means "inherit the route's trace class" in
                    // every maze path and in the Java SES reader.
                    0 => trace_clearance_class,
                    class => class,
                };
                (clearance_class, info.attach_smd_allowed())
            }
            // Without a bound ViaInfo, DSN import and router request
            // derivation use the global via-at-SMD switch together with the
            // concrete padstack's attach permission. SES reload must use the
            // same rule or a pre-existing DSN via can become attachable merely
            // by passing through a session file.
            None => (
                board.rules.item_clearance_class_for(net_no, ItemClass::Via),
                board.rules.via_at_smd_allowed
                    && board
                        .padstacks
                        .get_by_no(padstack_no)
                        .is_some_and(|padstack| padstack.attach_allowed),
            ),
        };
    let escape_smd_layer = (!attach_allowed)
        .then(|| board.pure_smd_escape_layer(net_no, padstack_no, center))
        .flatten();
    SesViaMetadata {
        clearance_class,
        attach_allowed,
        escape_smd_layer,
    }
}

/// Reads the routes of a session file into the board (Java:
/// `SesReader.read`). Coordinates scale from the session's resolution
/// to the board's.
pub fn import_ses(board: &mut BasicBoard, content: &str) -> Result<SesImportSummary, String> {
    let mut candidate = board.clone();
    let summary = import_ses_into(&mut candidate, content)?;
    crate::board::validation::validate_board_references(&candidate)
        .map_err(|error| format!("session produced an invalid board: {error}"))?;
    *board = candidate;
    Ok(summary)
}

/// Applies a session to a private candidate board.  The public entry point
/// commits this clone only after the entire document validates, so malformed
/// copper cannot leave a half-imported board behind.
fn import_ses_into(board: &mut BasicBoard, content: &str) -> Result<SesImportSummary, String> {
    let root = parse_dsn(content).map_err(|e| format!("session parse error: {e:?}"))?;
    if !root
        .name()
        .is_some_and(|name| name.eq_ignore_ascii_case("session"))
    {
        return Err("session root must be (session ...)".into());
    }
    let mut routes_nodes = root.children("routes");
    let routes = routes_nodes
        .next()
        .ok_or_else(|| "no routes section".to_string())?;
    if routes_nodes.next().is_some() {
        return Err("the session contains more than one routes section".into());
    }
    let mut resolution_nodes = routes.children("resolution");
    let resolution_node = resolution_nodes.next();
    if resolution_nodes.next().is_some() {
        return Err("the session contains more than one resolution declaration".into());
    }
    let (ses_unit, raw_resolution): (String, String) = if let Some(resolution) = resolution_node {
        let mut args = resolution.args();
        let unit = args
            .next()
            .ok_or_else(|| "session resolution is missing its unit".to_string())?;
        let value = args
            .next()
            .ok_or_else(|| "session resolution is missing its value".to_string())?;
        if args.next().is_some() {
            return Err("session resolution must contain exactly a unit and a value".into());
        }
        (unit.to_string(), value.to_string())
    } else {
        (board.unit.clone(), board.resolution.to_string())
    };
    let parsed_resolution = raw_resolution
        .parse::<f64>()
        .map_err(|_| "session resolution is not numeric".to_string())?;
    if !parsed_resolution.is_finite() || parsed_resolution <= 0.0 {
        return Err("session resolution must be finite and positive".into());
    }
    if parsed_resolution.fract() != 0.0 {
        return Err("session resolution must be an integer".into());
    }
    if parsed_resolution > f64::from(i32::MAX) {
        return Err("session resolution exceeds the board coordinate limit".into());
    }
    let ses_resolution = parsed_resolution;
    // The session's `(resolution <unit> <value>)` unit need not match the
    // board's; reconcile through a physical (mm) basis instead of assuming a
    // bare resolution ratio, which is only correct when the units are equal.
    let um_per_unit = |u: &str| -> Option<f64> {
        match u.to_ascii_lowercase().as_str() {
            "um" | "micron" | "microns" => Some(1.0),
            "mil" => Some(25.4),
            "inch" | "in" => Some(25_400.0),
            "mm" => Some(1000.0),
            "cm" => Some(10_000.0),
            _ => None,
        }
    };
    let ses_um_per_unit = um_per_unit(&ses_unit)
        .ok_or_else(|| format!("unsupported session resolution unit {ses_unit:?}"))?;
    let ses_units_per_mm = ses_resolution / ses_um_per_unit * 1000.0;
    let factor = board.board_units_per_mm() / ses_units_per_mm;
    if !factor.is_finite() || factor <= 0.0 {
        return Err("session resolution produces an invalid coordinate scale".into());
    }
    // Rust saturates float-to-integer casts.  A session coordinate that does
    // not fit the board grid must be rejected rather than silently landing on
    // an endpoint and changing the routed topology.
    let scale_error: RefCell<Option<String>> = RefCell::new(None);
    let scale = |v: f64| -> i32 {
        let scaled = v * factor;
        let limit = f64::from(crate::geometry::planar::limits::CRIT_INT);
        if !scaled.is_finite() || scaled < -limit || scaled > limit {
            let mut error = scale_error.borrow_mut();
            if error.is_none() {
                *error = Some(format!(
                    "scaled session coordinate {v} does not fit the board coordinate range"
                ));
            }
            return 0;
        }
        scaled.round() as i32
    };
    let mut network_nodes = routes.children("network_out");
    let network = network_nodes
        .next()
        .ok_or_else(|| "no network_out section".to_string())?;
    if network_nodes.next().is_some() {
        return Err("the session contains more than one network_out section".into());
    }
    let mut summary = SesImportSummary {
        wires: 0,
        vias: 0,
        unknown_nets: Vec::new(),
    };
    let _birth_tag = crate::board::basic_board::birth_tag_scope(1);
    for net_node in network.children("net") {
        let Some(net_name) = net_node.arg() else {
            return Err("session net scope is missing its name".into());
        };
        let net_nos = session_net_numbers(&board.rules, net_node)?;
        if net_nos.is_empty() {
            // an unknown session net must not become netless copper (a pure
            // obstacle violating against every real net): skip its routes
            // and report the name
            summary.unknown_nets.push(net_name.to_string());
            continue;
        }
        // propagate the net's own trace clearance class instead of hardcoding
        // the default, so a reloaded session keeps the design's spacing
        let trace_clearance_class = net_nos
            .first()
            .map(|&n| session_trace_clearance_class(board, n))
            .unwrap_or_else(crate::rules::BoardRules::default_clearance_class);
        for wire in net_node.children("wire") {
            let Some(path) = wire.child("path") else {
                return Err("session wire is missing its path".into());
            };
            let Some(layer_name) = path.arg() else {
                return Err("session path is missing its layer".into());
            };
            let layer = board
                .layer_structure
                .get_no(layer_name)
                .ok_or_else(|| format!("session path references unknown layer {layer_name:?}"))?;
            let raw_nums: Vec<&str> = path.args().skip(1).collect();
            if raw_nums.len() < 5 || !(raw_nums.len() - 1).is_multiple_of(2) {
                return Err("session path needs a width and at least two coordinate pairs".into());
            }
            let nums: Vec<f64> = raw_nums
                .iter()
                .map(|value| {
                    value
                        .parse::<f64>()
                        .map_err(|_| format!("session path contains non-numeric atom {value:?}"))
                })
                .collect::<Result<_, _>>()?;
            if nums.iter().any(|value| !value.is_finite()) || nums[0] <= 0.0 {
                return Err(
                    "session path width and coordinates must be finite; width must be positive"
                        .into(),
                );
            }
            let half_width = (scale(nums[0]) / 2).max(1);
            let corners: Vec<IntPoint> = nums[1..]
                .chunks_exact(2)
                .map(|c| IntPoint::new(scale(c[0]), scale(c[1])))
                .collect();
            let polyline = Polyline::from_int_points(&corners);
            if polyline.is_empty() {
                return Err("session path collapses to an empty polyline on the board grid".into());
            }
            // Java SesReader inserts session routing USER_FIXED: an
            // imported session is existing copper to preserve, not a
            // draft for the optimizer to rip
            let id = board.insert_trace(
                polyline,
                layer,
                half_width,
                net_nos.clone(),
                trace_clearance_class,
            );
            board.set_fixed_state(id, crate::board::FixedState::UserFixed);
            summary.wires += 1;
        }
        for via in net_node.children("via") {
            let args: Vec<&str> = via.args().collect();
            if args.len() != 3 {
                return Err("session via needs exactly a padstack name and x/y coordinates".into());
            }
            let padstack_no = board
                .padstacks
                .get(args[0])
                .map(|padstack| padstack.no)
                .ok_or_else(|| format!("session via references unknown padstack {:?}", args[0]))?;
            let (Ok(x), Ok(y)) = (args[1].parse::<f64>(), args[2].parse::<f64>()) else {
                return Err("session via coordinates must be numeric".into());
            };
            if !x.is_finite() || !y.is_finite() {
                return Err("session via coordinates must be finite".into());
            }
            let center = IntPoint::new(scale(x), scale(y));
            let metadata = net_nos
                .first()
                .map(|&n| session_via_metadata(board, n, padstack_no, center))
                .unwrap_or(SesViaMetadata {
                    clearance_class: trace_clearance_class,
                    attach_allowed: true,
                    escape_smd_layer: None,
                });
            let id = if let Some(layer) = metadata.escape_smd_layer {
                board.insert_escape_via(
                    padstack_no,
                    center,
                    net_nos.clone(),
                    metadata.clearance_class,
                    metadata.attach_allowed,
                    layer,
                )
            } else {
                board.insert_via(
                    padstack_no,
                    center,
                    net_nos.clone(),
                    metadata.clearance_class,
                    metadata.attach_allowed,
                )
            };
            board.set_fixed_state(id, crate::board::FixedState::UserFixed);
            // register contacts when the via lands mid-trace (a contact
            // needs a trace endpoint at the pad)
            board.split_traces_at_via(id);
            summary.vias += 1;
        }
    }
    if let Some(error) = scale_error.into_inner() {
        return Err(error);
    }
    Ok(summary)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::ItemKind;
    use crate::geometry::planar::{IntBox, TileShape};
    use crate::io::dsn_import::import_dsn;

    #[test]
    fn session_via_uses_the_selected_padstack_via_info_clearance() {
        let dsn = r#"(pcb "ses-via-class.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (rect pcb 0 0 100000 100000))
    (rule
      (width 200)
      (clearance 200)
      (clearance 900 (type ViaStrict_ViaStrict)))
  )
  (placement)
  (library
    (padstack Via1
      (shape (circle F.Cu 600 0 0))
      (shape (circle B.Cu 600 0 0)))
    (padstack Via2
      (shape (circle F.Cu 700 0 0))
      (shape (circle B.Cu 700 0 0))))
  (network
    (net N1)
    (via V1 Via1 ViaStrict)
    (via V2 Via2 null)
    (via_rule VR V1 V2)
    (class strict N1 (via_rule VR) (rule (width 200) (clearance 400)))
  )
)"#;
        let mut board = import_dsn(dsn).expect("design import");
        let net_no = board.rules.nets.get_by_name("N1")[0].net_number;
        let smd_shape = TileShape::Box(IntBox::from_coords(-1000, -1000, 1000, 1000));
        let smd_padstack = board
            .padstacks
            .add("SmdTop", vec![Some(smd_shape), None], false, false);
        let pin = board.insert_via(
            smd_padstack,
            IntPoint::new(50000, 50000),
            vec![net_no],
            board.rules.item_clearance_class_for(net_no, ItemClass::Smd),
            false,
        );
        board.set_component_no(pin, 1);
        let session = r#"(session ses-via-class
  (routes
    (resolution um 10)
    (network_out
      (net N1
        (wire (path F.Cu 200 10000 10000 20000 10000))
        (via Via1 50000 50000)
        (via Via2 70000 50000)))))"#;

        let summary = import_ses(&mut board, session).expect("session import");
        assert_eq!(summary.wires, 1);
        assert_eq!(summary.vias, 2);
        let expected = board
            .rules
            .clearance_matrix
            .get_no("ViaStrict")
            .expect("named via clearance class");
        let imported = board
            .items()
            .find_map(|(_, item)| match &item.kind {
                ItemKind::Via(via)
                    if item.base.component_no == 0 && via.center == IntPoint::new(50000, 50000) =>
                {
                    Some((item.base.clearance_class, via))
                }
                _ => None,
            })
            .expect("session via");
        assert_eq!(imported.0, expected);
        assert!(!imported.1.attach_allowed, "ViaInfo attach bit remains off");
        assert!(imported.1.is_escape_via);
        assert_eq!(imported.1.escape_smd_layer, Some(0));

        let trace_class = board.rules.get_trace_clearance_class(net_no);
        assert_ne!(trace_class, 0);
        let imported_trace_class = board
            .items()
            .find_map(|(_, item)| {
                matches!(item.kind, ItemKind::PolylineTrace(_)).then_some(item.base.clearance_class)
            })
            .expect("session wire");
        assert_eq!(
            imported_trace_class, trace_class,
            "session wires inherit NetClass.trace_clearance_class"
        );
        let inherited = board
            .items()
            .find_map(|(_, item)| match &item.kind {
                ItemKind::Via(via)
                    if item.base.component_no == 0 && via.center == IntPoint::new(70000, 50000) =>
                {
                    Some(item.base.clearance_class)
                }
                _ => None,
            })
            .expect("session via with null ViaInfo class");
        assert_eq!(
            inherited, trace_class,
            "ViaInfo class 0 inherits the route's nonzero trace class"
        );
        let ordinary = board
            .items()
            .find_map(|(_, item)| match &item.kind {
                ItemKind::Via(via)
                    if item.base.component_no == 0 && via.center == IntPoint::new(70000, 50000) =>
                {
                    Some(via)
                }
                _ => None,
            })
            .expect("ordinary session via");
        assert!(!ordinary.attach_allowed);
        assert!(!ordinary.is_escape_via);
        assert!(crate::drc::check_board(&board).violations.is_empty());
    }

    #[test]
    fn standard_session_names_select_only_subnet_one() {
        let dsn = r#"(pcb "ses-subnets.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (boundary (rect pcb 0 0 100000 100000))
    (rule (width 200) (clearance 200)))
  (placement)
  (library)
  (network (net GND 1) (net GND 2)))"#;
        let mut board = import_dsn(dsn).expect("design import");
        let subnet_one = board.rules.nets.get("GND", 1).unwrap().net_number;
        let subnet_two = board.rules.nets.get("GND", 2).unwrap().net_number;
        let session = r#"(session ses-subnets
  (routes
    (resolution um 10)
    (network_out
      (net GND
        (wire (path F.Cu 200 1000 1000 2000 1000))))))"#;
        import_ses(&mut board, session).expect("session import");
        let trace = board
            .items()
            .find_map(|(_, item)| matches!(item.kind, ItemKind::PolylineTrace(_)).then_some(item))
            .expect("session trace");
        assert_eq!(trace.base.net_nos, vec![subnet_one]);
        assert!(!trace.base.contains_net(subnet_two));
    }

    #[test]
    fn standard_net_number_is_translator_metadata_not_a_subnet_selector() {
        let dsn = r#"(pcb "ses-subnets.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (boundary (rect pcb 0 0 100000 100000))
    (rule (width 200) (clearance 200)))
  (placement)
  (library)
  (network (net GND 1) (net GND 2)))"#;
        let mut board = import_dsn(dsn).expect("design import");
        let subnet_one = board.rules.nets.get("GND", 1).unwrap().net_number;
        let subnet_two = board.rules.nets.get("GND", 2).unwrap().net_number;
        let session = r#"(session ses-subnets
  (routes
    (resolution um 10)
    (network_out
      (net GND
        (net_number 2)
        (wire (path F.Cu 200 1000 1000 2000 1000))))))"#;
        import_ses(&mut board, session).expect("session import");
        let trace = board
            .items()
            .find_map(|(_, item)| matches!(item.kind, ItemKind::PolylineTrace(_)).then_some(item))
            .expect("session trace");
        assert_eq!(trace.base.net_nos, vec![subnet_one]);
        assert!(!trace.base.contains_net(subnet_two));

        let exact = r#"(session ses-subnets
  (routes
    (resolution um 10)
    (network_out
      (net GND
        (net_number 999999)
        (freerouting_subnet 2)
        (wire (path F.Cu 200 1000 2000 2000 2000))))))"#;
        import_ses(&mut board, exact).expect("private subnet extension import");
        let trace = board
            .items()
            .filter_map(|(_, item)| matches!(item.kind, ItemKind::PolylineTrace(_)).then_some(item))
            .last()
            .expect("exact-subnet trace");
        assert_eq!(trace.base.net_nos, vec![subnet_two]);
    }

    #[test]
    fn missing_session_resolution_inherits_the_board_unit_and_scale() {
        let dsn = r#"(pcb "ses-mil.dsn"
  (resolution mil 10)
  (structure
    (layer F.Cu (type signal))
    (boundary (rect pcb 0 0 10000 10000))
    (rule (width 20) (clearance 20)))
  (placement)
  (library)
  (network (net N1)))"#;
        let mut board = import_dsn(dsn).expect("mil design import");
        let session = r#"(session ses-mil
  (routes
    (network_out
      (net N1
        (wire (path F.Cu 200 1000 1000 2000 1000))))))"#;
        import_ses(&mut board, session).expect("session import");
        let trace = board
            .items()
            .find_map(|(_, item)| match &item.kind {
                ItemKind::PolylineTrace(trace) => Some(trace),
                _ => None,
            })
            .expect("session trace");
        assert_eq!(trace.half_width, 100);
        let corners = trace.polyline.corner_approx_arr();
        assert_eq!(
            corners.first().map(|point| point.x.round() as i32),
            Some(1000)
        );
        assert_eq!(
            corners.last().map(|point| point.x.round() as i32),
            Some(2000)
        );
    }

    #[test]
    fn session_resolution_metadata_is_validated_before_coordinate_scaling() {
        let dsn = r#"(pcb "ses-resolution.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (boundary (rect pcb 0 0 10000 10000))
    (rule (width 20) (clearance 20)))
  (placement)
  (library)
  (network (net N1)))"#;
        let base = import_dsn(dsn).expect("design import");
        let session = |declaration: &str| {
            format!(
                r#"(session ses-resolution
  (routes
    {declaration}
    (network_out (net N1))))"#
            )
        };

        let mut valid = base.clone();
        import_ses(&mut valid, &session("(resolution MIL 10.0)"))
            .expect("an integral numeric spelling is lossless");

        for (declaration, expected) in [
            ("(resolution)", "missing its unit"),
            ("(resolution um)", "missing its value"),
            ("(resolution um 10 extra)", "exactly a unit and a value"),
            ("(resolution um not-a-number)", "not numeric"),
            ("(resolution um NaN)", "finite and positive"),
            ("(resolution um inf)", "finite and positive"),
            ("(resolution um 0)", "finite and positive"),
            ("(resolution um -1)", "finite and positive"),
            ("(resolution um 1.5)", "must be an integer"),
            (
                "(resolution um 2147483648)",
                "exceeds the board coordinate limit",
            ),
            (
                "(resolution parsec 10)",
                "unsupported session resolution unit",
            ),
        ] {
            let mut board = base.clone();
            let error = import_ses(&mut board, &session(declaration))
                .expect_err("invalid session resolution must fail closed");
            assert!(
                error.contains(expected),
                "{declaration} produced unexpected error: {error}"
            );
        }

        let duplicate = r#"(session ses-resolution
  (routes
    (resolution um 10)
    (resolution mil 10)
    (network_out (net N1))))"#;
        let mut board = base;
        let error = import_ses(&mut board, duplicate)
            .expect_err("duplicate resolution scopes must not pick one implicitly");
        assert!(
            error.contains("more than one resolution declaration"),
            "{error}"
        );
    }

    #[test]
    fn session_root_and_subnet_metadata_fail_closed() {
        let dsn = r#"(pcb "ses-root.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (boundary (rect pcb 0 0 10000 10000))
    (rule (width 20) (clearance 20)))
  (placement)
  (library)
  (network (net N1)))"#;
        let mut board = import_dsn(dsn).expect("design import");
        let wrong_root = "(pcb x (routes (network_out)))";
        let error = import_ses(&mut board, wrong_root).expect_err("wrong root must fail");
        assert!(error.contains("root must be (session"), "{error}");

        let bad_subnet = r#"(session x
  (routes
    (network_out
      (net N1 nope))))"#;
        let error = import_ses(&mut board, bad_subnet).expect_err("bad subnet must fail");
        assert!(error.contains("too many arguments"), "{error}");
    }

    #[test]
    fn scaled_session_geometry_outside_board_integer_range_is_rejected() {
        let dsn = r#"(pcb "ses-range.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (boundary (rect pcb 0 0 10000 10000))
    (rule (width 20) (clearance 20)))
  (placement)
  (library)
  (network (net N1)))"#;
        let mut board = import_dsn(dsn).expect("design import");
        let session = r#"(session ses-range
  (routes
    (resolution um 10)
    (network_out
      (net N1
        (wire (path F.Cu 20 0 0 3000000000 1000))))))"#;
        let error = import_ses(&mut board, session)
            .expect_err("scaled session coordinates must not saturate")
            .to_string();
        assert!(
            error.contains("does not fit the board coordinate range"),
            "{error}"
        );
    }

    #[test]
    fn malformed_session_is_atomic_and_resets_insertion_provenance() {
        let dsn = r#"(pcb "ses-atomic.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (boundary (rect pcb 0 0 100000 100000))
    (rule (width 200) (clearance 200)))
  (placement)
  (library)
  (network (net N1)))"#;
        let mut board = import_dsn(dsn).expect("design import");
        let item_count = board.item_count();
        let malformed = r#"(session ses-atomic
  (routes
    (resolution um 10)
    (network_out
      (net N1
        (wire (path F.Cu 200 1000 1000 2000 1000))
        (wire (path F.Cu 200 3000 3000 not-a-number 3000))))))"#;
        assert!(import_ses(&mut board, malformed).is_err());
        assert_eq!(
            board.item_count(),
            item_count,
            "a failed session must not commit its earlier valid routes"
        );

        let id = board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(4_000, 4_000), IntPoint::new(5_000, 4_000)]),
            0,
            100,
            vec![1],
            1,
        );
        assert_eq!(
            board.get_item(id).unwrap().base.birth,
            0,
            "an import error must restore the thread-local birth tag"
        );
    }
}
