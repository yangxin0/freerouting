//! SES session writer (port of the writing half of
//! `io/specctra/SesWriter.java`, simplified): emits the routed wires and
//! vias of a board as a Specctra session file, which KiCad and other
//! tools import back.

use crate::board::basic_board::BasicBoard;
use crate::board::ItemKind;
use crate::io::ses_import::{session_trace_clearance_class, session_via_metadata};

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SesWriteError {
    InvalidBoard {
        reason: String,
    },
    ResolutionMismatch {
        board_resolution: i32,
        requested_resolution: i32,
    },
    UnrepresentableIdentifier {
        context: &'static str,
        value: String,
    },
    UnrepresentableRouteItem {
        item_id: crate::board::basic_board::ItemId,
        reason: &'static str,
    },
    DanglingReference {
        item_id: crate::board::basic_board::ItemId,
        kind: &'static str,
        number: i64,
    },
    DanglingRuleReference {
        owner: String,
        kind: &'static str,
        number: i64,
    },
}

impl std::fmt::Display for SesWriteError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidBoard { reason } => write!(f, "invalid board: {reason}"),
            Self::ResolutionMismatch {
                board_resolution,
                requested_resolution,
            } => write!(
                f,
                "session resolution {requested_resolution} does not match the board resolution {board_resolution}"
            ),
            Self::UnrepresentableIdentifier { context, value } => write!(
                f,
                "Specctra session cannot represent {context} identifier {value:?} losslessly"
            ),
            Self::UnrepresentableRouteItem { item_id, reason } => {
                write!(
                    f,
                    "Specctra session cannot represent route item {item_id}: {reason}"
                )
            }
            Self::DanglingReference {
                item_id,
                kind,
                number,
            } => write!(f, "route item {item_id} references missing {kind} {number}"),
            Self::DanglingRuleReference {
                owner,
                kind,
                number,
            } => write!(f, "{owner} references missing {kind} {number}"),
        }
    }
}

impl std::error::Error for SesWriteError {}

fn quoted(value: &str, context: &'static str) -> Result<String, SesWriteError> {
    let quote = if !value.contains('"') {
        '"'
    } else if !value.contains('\'') {
        '\''
    } else {
        return Err(SesWriteError::UnrepresentableIdentifier {
            context,
            value: value.to_string(),
        });
    };
    Ok(format!("{quote}{value}{quote}"))
}

fn atom(value: &str, context: &'static str) -> Result<String, SesWriteError> {
    if !value.is_empty()
        && value.chars().all(|ch| {
            !ch.is_whitespace() && !matches!(ch, '(' | ')' | '"' | '\'' | '#' | ';' | '\\')
        })
    {
        Ok(value.to_string())
    } else {
        quoted(value, context)
    }
}

/// Exports the routed items of the board as a Specctra session file.
/// `design_name` is the name recorded in the session; `resolution` must
/// match the import (internal units per micrometer).
pub fn export_ses(
    board: &BasicBoard,
    design_name: &str,
    resolution: i32,
) -> Result<String, SesWriteError> {
    if resolution <= 0 || resolution != board.resolution {
        return Err(SesWriteError::ResolutionMismatch {
            board_resolution: board.resolution,
            requested_resolution: resolution,
        });
    }
    // Validate every route item before emitting any text. SES groups copper
    // under exactly one net and cannot encode netless or multi-net items.
    // SystemFixed copper belongs to the base DSN and must not be repeated.
    for (item_id, item) in board.items() {
        if item.base.component_no != 0 {
            // Component vias are pins from the base design and are omitted
            // below regardless of their fixed-state spelling.  SES has no
            // component field for traces, however, so emitting one would
            // silently turn it into free routing copper.
            if matches!(item.kind, ItemKind::PolylineTrace(_)) {
                return Err(SesWriteError::UnrepresentableRouteItem {
                    item_id: *item_id,
                    reason: "SES route records cannot preserve component-owned traces",
                });
            }
            continue;
        }
        if item.base.fixed_state == crate::board::FixedState::SystemFixed {
            continue;
        }
        let is_route_item = matches!(item.kind, ItemKind::PolylineTrace(_) | ItemKind::Via(_));
        if !is_route_item {
            continue;
        }
        if item.base.net_nos.len() != 1 {
            return Err(SesWriteError::UnrepresentableRouteItem {
                item_id: *item_id,
                reason: "SES route items must belong to exactly one net",
            });
        }
        let net_no = item.base.net_nos[0];
        let Some(net) = board.rules.nets.get_by_no(net_no) else {
            return Err(SesWriteError::DanglingReference {
                item_id: *item_id,
                kind: "net",
                number: net_no as i64,
            });
        };
        if net.subnet_number != 1 {
            return Err(SesWriteError::UnrepresentableRouteItem {
                item_id: *item_id,
                reason: "standard SES net scopes cannot identify a DSN subnet other than 1",
            });
        }
        match &item.kind {
            ItemKind::PolylineTrace(trace) => {
                if board.layer_structure.arr.get(trace.layer).is_none() {
                    return Err(SesWriteError::DanglingReference {
                        item_id: *item_id,
                        kind: "layer",
                        number: trace.layer as i64,
                    });
                }
            }
            ItemKind::Via(via) => {
                if board.padstacks.get_by_no(via.padstack).is_none() {
                    return Err(SesWriteError::DanglingReference {
                        item_id: *item_id,
                        kind: "padstack",
                        number: via.padstack as i64,
                    });
                }
            }
            ItemKind::ObstacleArea(_) => {}
        }
    }
    for info_id in 0..board.rules.via_infos.count() {
        let info = board.rules.via_infos.get(info_id);
        let padstack_no = info.get_padstack();
        if board.padstacks.get_by_no(padstack_no).is_none() {
            return Err(SesWriteError::DanglingRuleReference {
                owner: format!("via info {:?}", info.get_name()),
                kind: "padstack",
                number: padstack_no as i64,
            });
        }
    }
    crate::board::validate_board_references(board).map_err(|error| {
        SesWriteError::InvalidBoard {
            reason: error.to_string(),
        }
    })?;

    // Standard SES route records do not carry item clearance, provenance, or
    // via attach/escape fields. Reject any emitted item that the matching base
    // design would reconstruct differently instead of silently changing its
    // routing or DRC semantics on reload.
    for (item_id, item) in board.items() {
        if item.base.component_no != 0
            || item.base.fixed_state == crate::board::FixedState::SystemFixed
            || !matches!(item.kind, ItemKind::PolylineTrace(_) | ItemKind::Via(_))
        {
            continue;
        }
        if item.base.clearance_class_explicit {
            return Err(SesWriteError::UnrepresentableRouteItem {
                item_id: *item_id,
                reason: "standard SES route records cannot encode explicit clearance classes",
            });
        }
        let net_no = item.base.net_nos[0];
        match &item.kind {
            ItemKind::PolylineTrace(_) => {
                if item.base.clearance_class != session_trace_clearance_class(board, net_no) {
                    return Err(SesWriteError::UnrepresentableRouteItem {
                        item_id: *item_id,
                        reason: "inherited trace clearance differs from the net class SES reload derives",
                    });
                }
            }
            ItemKind::Via(via) => {
                let metadata = session_via_metadata(board, net_no, via.padstack, via.center);
                if item.base.clearance_class != metadata.clearance_class {
                    return Err(SesWriteError::UnrepresentableRouteItem {
                        item_id: *item_id,
                        reason: "inherited via clearance differs from the base-design rule SES reload derives",
                    });
                }
                if via.attach_allowed != metadata.attach_allowed {
                    return Err(SesWriteError::UnrepresentableRouteItem {
                        item_id: *item_id,
                        reason: "via attach permission differs from the base-design rule SES reload derives",
                    });
                }
                if via.is_escape_via != metadata.escape_smd_layer.is_some()
                    || via.escape_smd_layer != metadata.escape_smd_layer
                {
                    return Err(SesWriteError::UnrepresentableRouteItem {
                        item_id: *item_id,
                        reason: "via escape metadata differs from the base-design geometry SES reload derives",
                    });
                }
            }
            ItemKind::ObstacleArea(_) => unreachable!("route item filter excluded areas"),
        }
    }

    let mut out = String::new();
    let stem = design_name
        .strip_suffix(".dsn")
        .or_else(|| design_name.strip_suffix(".DSN"))
        .or_else(|| design_name.strip_suffix(".ses"))
        .or_else(|| design_name.strip_suffix(".SES"))
        .unwrap_or(design_name);
    let session_name = quoted(&format!("{stem}.ses"), "session")?;
    let base_name = quoted(&format!("{stem}.dsn"), "base-design")?;
    out.push_str(&format!("(session {session_name}\n"));
    out.push_str(&format!("  (base_design {base_name})\n"));
    out.push_str("  (routes \n");
    // Echo the imported physical unit (Java: SesWriter writes the design's
    // unit); hardcoding `um` relabelled non-um designs.
    out.push_str(&format!(
        "    (resolution {} {})\n",
        atom(&board.unit, "unit")?,
        board.resolution
    ));
    out.push_str("    (parser\n      (host_cad \"freerouting-rs\")\n    )\n");

    // library_out: the via padstacks referenced by the session (Java:
    // SesWriter.writeLibrary), shapes written per layer in board units
    // like the rest of this writer
    let mut via_padstacks: Vec<usize> = board
        .items()
        .filter(|(_, it)| {
            it.base.component_no == 0
                && it.base.net_count() > 0
                && it.base.fixed_state != crate::board::FixedState::SystemFixed
        })
        .filter_map(|(_, it)| match &it.kind {
            ItemKind::Via(v) => Some(v.padstack),
            _ => None,
        })
        .collect();
    for info_id in 0..board.rules.via_infos.count() {
        via_padstacks.push(board.rules.via_infos.get(info_id).get_padstack());
    }
    via_padstacks.sort_unstable();
    via_padstacks.dedup();
    out.push_str("    (library_out \n");
    for ps_no in via_padstacks {
        let ps = board.padstacks.get_by_no(ps_no).ok_or_else(|| {
            SesWriteError::DanglingRuleReference {
                owner: "session library".to_string(),
                kind: "padstack",
                number: ps_no as i64,
            }
        })?;
        let padstack_name = quoted(&ps.name, "padstack")?;
        out.push_str(&format!("      (padstack {padstack_name}\n"));
        for layer in 0..board.layer_structure.layer_count() {
            let Some(shape) = ps.get_shape(layer) else {
                continue;
            };
            let layer_name = &board.layer_structure.arr[layer].name;
            let layer_token = atom(layer_name, "layer")?;
            match shape {
                crate::geometry::planar::TileShape::Box(b) => {
                    out.push_str(&format!(
                        "        (shape (rect {} {} {} {} {}))\n",
                        layer_token, b.ll.x, b.ll.y, b.ur.x, b.ur.y
                    ));
                }
                other => {
                    out.push_str(&format!("        (shape (polygon {} 0", layer_token));
                    for i in 0..other.border_line_count() {
                        let c = other.corner_approx(i);
                        out.push_str(&format!(" {} {}", c.x.round() as i64, c.y.round() as i64));
                    }
                    out.push_str("))\n");
                }
            }
        }
        if !ps.attach_allowed {
            out.push_str("        (attach off)\n");
        }
        out.push_str("      )\n");
    }
    out.push_str("    )\n");
    out.push_str("    (network_out \n");

    for net_no in 1..=board.rules.nets.max_net_no() {
        let Some(net) = board.rules.nets.get_by_no(net_no) else {
            continue;
        };
        // collect the routed items of this net
        let mut wires = String::new();
        for (_, item) in board.items() {
            if !item.base.contains_net(net_no)
                || item.base.fixed_state == crate::board::FixedState::SystemFixed
            {
                continue;
            }
            match &item.kind {
                ItemKind::PolylineTrace(t) => {
                    let layer_name = &board.layer_structure.arr[t.layer].name;
                    let layer_token = atom(layer_name, "layer")?;
                    wires.push_str(&format!(
                        "        (wire\n          (path {} {}",
                        layer_token,
                        2_i64 * i64::from(t.half_width)
                    ));
                    for corner in t.polyline.corner_approx_arr() {
                        wires.push_str(&format!(
                            "\n            {} {}",
                            corner.x.round() as i64,
                            corner.y.round() as i64
                        ));
                    }
                    wires.push_str("\n          )\n        )\n");
                }
                ItemKind::Via(v) => {
                    // only autoroute-inserted vias (component pins are part
                    // of the base design and are not written)
                    if item.base.component_no != 0 {
                        continue;
                    }
                    let Some(padstack) = board.padstacks.get_by_no(v.padstack) else {
                        continue;
                    };
                    let padstack_name = quoted(&padstack.name, "padstack")?;
                    wires.push_str(&format!(
                        "        (via {} {} {}\n        )\n",
                        padstack_name, v.center.x, v.center.y
                    ));
                }
                ItemKind::ObstacleArea(_) => {}
            }
        }
        if !wires.is_empty() {
            let net_name = quoted(&net.name, "net")?;
            // SesReader (Java and Rust) resolves a session net by name; the
            // standard SES grammar does not carry the DSN subnet number in
            // this scope.  Emitting the numeric token here is accepted by
            // our permissive parser but makes the session incompatible with
            // Java's SesReader, which treats the token as an unknown child
            // and can skip the route.  Keep the canonical `(net NAME ...)`.
            out.push_str(&format!("      (net {net_name}\n"));
            out.push_str(&wires);
            out.push_str("      )\n");
        }
    }
    out.push_str("    )\n  )\n)\n");
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{Layer, LayerStructure};
    use crate::core::Padstacks;
    use crate::geometry::planar::{IntBox, IntPoint, Polyline, TileShape};
    use crate::io::dsn::parse_dsn;
    use crate::rules::{BoardRules, ClearanceMatrix, ViaInfo, ViaRule};

    fn test_board() -> BasicBoard {
        let stack = LayerStructure::new(vec![Layer::new("F.Cu", true), Layer::new("B.Cu", true)]);
        let matrix = ClearanceMatrix::get_default_instance(stack.clone(), 200);
        let mut rules = BoardRules::new(stack.clone(), matrix);
        let default_class = rules.get_default_net_class();
        rules.nets.add("GND", 1, false);
        // Give the hand-built fixture the same base-design metadata that the
        // SES reader uses to reconstruct a routed via.  In particular, the
        // attach bit must come from a bound ViaInfo rather than the reader's
        // legacy no-info fallback.
        let via_info = rules
            .via_infos
            .add(ViaInfo::new("default-via", 1, 1, false))
            .expect("unique ViaInfo");
        let mut via_rule = ViaRule::new("default-via-rule");
        via_rule.append_via(via_info);
        let via_rule_id = rules.via_rules.len();
        rules.via_rules.push(via_rule);
        rules
            .net_classes
            .get_mut(default_class)
            .set_via_rule(Some(via_rule_id));
        let mut padstacks = Padstacks::new(2);
        padstacks.add(
            "Via[0-1]_800:400_um",
            vec![
                Some(TileShape::Box(IntBox::from_coords(-400, -400, 400, 400))),
                Some(TileShape::Box(IntBox::from_coords(-400, -400, 400, 400))),
            ],
            true,
            false,
        );
        BasicBoard::new(stack, rules, padstacks)
    }

    #[test]
    fn exports_wires_and_vias() {
        let mut board = test_board();
        board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(0, 0), IntPoint::new(5000, 0)]),
            0,
            100,
            vec![1],
            1,
        );
        board.insert_via(1, IntPoint::new(5000, 0), vec![1], 1, false);

        let ses = export_ses(&board, "test_board", 10).expect("session export");
        // the output is well-formed S-expression
        let parsed = parse_dsn(&ses).expect("SES not parseable");
        assert_eq!(parsed.name(), Some("session"));
        let routes = parsed.child("routes").expect("routes");
        // the library_out carries the via padstack definitions
        let library = routes.child("library_out").expect("library_out");
        let padstack = library.child("padstack").expect("padstack");
        assert_eq!(padstack.arg(), Some("Via[0-1]_800:400_um"));
        assert!(padstack.child("shape").is_some(), "per-layer shapes");
        let network_out = routes.child("network_out").expect("network_out");
        let net = network_out.child("net").expect("net");
        assert_eq!(net.arg(), Some("GND"));
        assert_eq!(
            net.args().count(),
            1,
            "SES net scope must not emit a subnet token"
        );
        let wire = net.child("wire").expect("wire");
        let path = wire.child("path").expect("path");
        assert_eq!(path.arg(), Some("F.Cu"));
        assert_eq!(path.args().nth(1), Some("200")); // width = 2 * half width
        let via = net.child("via").expect("via");
        assert_eq!(via.arg(), Some("Via[0-1]_800:400_um"));

        let named = export_ses(&board, "test_board.dsn", 10).expect("normalized names");
        let parsed_named = parse_dsn(&named).expect("normalized session parses");
        assert_eq!(parsed_named.arg(), Some("test_board.ses"));
        assert_eq!(
            parsed_named
                .child("base_design")
                .and_then(|base| base.arg()),
            Some("test_board.dsn")
        );
        assert!(matches!(
            export_ses(&board, "test_board", 1),
            Err(SesWriteError::ResolutionMismatch { .. })
        ));

        board.rules.nets.get_by_no_mut(1).unwrap().name = "rail \"A\"".into();
        let quoted = export_ses(&board, "test_board", 10).expect("single quote fallback");
        let parsed = parse_dsn(&quoted).expect("fallback output parses");
        assert_eq!(
            parsed
                .child("routes")
                .and_then(|routes| routes.child("network_out"))
                .and_then(|network| network.child("net"))
                .and_then(|net| net.arg()),
            Some("rail \"A\"")
        );

        board.rules.nets.get_by_no_mut(1).unwrap().name = "both \"double\" and 'single'".into();
        assert!(
            export_ses(&board, "test_board", 10).is_err(),
            "unrepresentable names must fail instead of corrupting the session"
        );
    }

    #[test]
    fn system_fixed_copper_stays_in_the_base_design() {
        let mut board = test_board();
        let fixed_trace = board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(0, 0), IntPoint::new(1_000, 0)]),
            0,
            100,
            vec![1],
            1,
        );
        board.set_fixed_state(fixed_trace, crate::board::FixedState::SystemFixed);
        board.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(0, 2_000), IntPoint::new(1_000, 2_000)]),
            0,
            100,
            vec![1],
            1,
        );
        let fixed_via = board.insert_via(1, IntPoint::new(2_000, 0), vec![1], 1, false);
        board.set_fixed_state(fixed_via, crate::board::FixedState::SystemFixed);
        board.insert_via(1, IntPoint::new(2_000, 2_000), vec![1], 1, false);

        let ses = export_ses(&board, "fixed", 10).expect("export");
        let parsed = parse_dsn(&ses).expect("parse");
        let net = parsed
            .child("routes")
            .and_then(|routes| routes.child("network_out"))
            .and_then(|network| network.child("net"))
            .expect("route net");
        assert_eq!(net.children("wire").count(), 1);
        assert_eq!(net.children("via").count(), 1);
    }

    #[test]
    fn rejects_route_items_that_ses_cannot_represent() {
        let mut netless = test_board();
        netless.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(0, 0), IntPoint::new(1_000, 0)]),
            0,
            100,
            Vec::new(),
            1,
        );
        assert!(matches!(
            export_ses(&netless, "netless", 10),
            Err(SesWriteError::UnrepresentableRouteItem { .. })
        ));

        let mut dangling = test_board();
        dangling.insert_via(usize::MAX, IntPoint::new(0, 0), vec![1], 1, false);
        assert!(matches!(
            export_ses(&dangling, "dangling", 10),
            Err(SesWriteError::DanglingReference {
                kind: "padstack",
                ..
            })
        ));

        let mut dangling_rule = test_board();
        dangling_rule
            .rules
            .via_infos
            .add(crate::rules::ViaInfo::new("bad-via", usize::MAX, 1, false))
            .expect("unique ViaInfo");
        assert!(matches!(
            export_ses(&dangling_rule, "dangling-rule", 10),
            Err(SesWriteError::DanglingRuleReference {
                kind: "padstack",
                ..
            })
        ));

        let mut subnet = test_board();
        let subnet_two = subnet.rules.nets.add("GND", 2, false);
        subnet
            .rules
            .nets
            .get_by_no_mut(subnet_two)
            .expect("subnet 2")
            .set_class(0);
        subnet.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(0, 4_000), IntPoint::new(1_000, 4_000)]),
            0,
            100,
            vec![subnet_two],
            1,
        );
        assert!(matches!(
            export_ses(&subnet, "subnet", 10),
            Err(SesWriteError::UnrepresentableRouteItem { reason, .. })
                if reason.contains("subnet other than 1")
        ));
    }

    #[test]
    fn library_preserves_attach_off() {
        let mut board = test_board();
        let padstack = board.padstacks.add(
            "NoAttach",
            vec![
                Some(TileShape::Box(IntBox::from_coords(-300, -300, 300, 300))),
                Some(TileShape::Box(IntBox::from_coords(-300, -300, 300, 300))),
            ],
            false,
            false,
        );
        let no_attach_info = board
            .rules
            .via_infos
            .add(ViaInfo::new("no-attach", padstack, 1, false))
            .expect("unique ViaInfo");
        board.rules.via_rules[0].append_via(no_attach_info);
        board.insert_via(padstack, IntPoint::new(1_000, 1_000), vec![1], 1, false);
        let ses = export_ses(&board, "attach", 10).expect("export");
        let parsed = parse_dsn(&ses).expect("parse");
        let no_attach = parsed
            .child("routes")
            .and_then(|routes| routes.child("library_out"))
            .into_iter()
            .flat_map(|library| library.children("padstack"))
            .find(|node| node.arg() == Some("NoAttach"))
            .expect("NoAttach padstack");
        assert_eq!(
            no_attach.child("attach").and_then(|attach| attach.arg()),
            Some("off")
        );
    }

    #[test]
    fn rejects_clearance_overrides_and_noncanonical_via_metadata() {
        let mut explicit_trace = test_board();
        let trace = explicit_trace.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(0, 0), IntPoint::new(1_000, 0)]),
            0,
            100,
            vec![1],
            1,
        );
        explicit_trace.set_item_clearance_class_explicit(trace, true);
        assert!(matches!(
            export_ses(&explicit_trace, "explicit-trace", 10),
            Err(SesWriteError::UnrepresentableRouteItem { reason, .. })
                if reason.contains("explicit clearance")
        ));

        let mut component_trace = test_board();
        let component_trace_id = component_trace.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(0, 0), IntPoint::new(1_000, 0)]),
            0,
            100,
            vec![1],
            1,
        );
        component_trace.set_component_no(component_trace_id, 7);
        assert!(matches!(
            export_ses(&component_trace, "component-trace", 10),
            Err(SesWriteError::UnrepresentableRouteItem { reason, .. })
                if reason.contains("component-owned traces")
        ));

        let mut explicit_via = test_board();
        let via = explicit_via.insert_via(1, IntPoint::new(1_000, 0), vec![1], 1, false);
        explicit_via.set_item_clearance_class_explicit(via, true);
        assert!(matches!(
            export_ses(&explicit_via, "explicit-via", 10),
            Err(SesWriteError::UnrepresentableRouteItem { reason, .. })
                if reason.contains("explicit clearance")
        ));

        let mut mismatched_trace = test_board();
        assert!(mismatched_trace
            .rules
            .clearance_matrix
            .append_class("strict"));
        let strict = mismatched_trace
            .rules
            .clearance_matrix
            .get_no("strict")
            .expect("strict clearance class");
        mismatched_trace.insert_trace(
            Polyline::from_int_points(&[IntPoint::new(0, 0), IntPoint::new(1_000, 0)]),
            0,
            100,
            vec![1],
            strict,
        );
        assert!(matches!(
            export_ses(&mismatched_trace, "mismatched-trace", 10),
            Err(SesWriteError::UnrepresentableRouteItem { reason, .. })
                if reason.contains("inherited trace clearance")
        ));

        let mut mismatched_via = test_board();
        mismatched_via.insert_via(1, IntPoint::new(1_000, 0), vec![1], 0, false);
        assert!(matches!(
            export_ses(&mismatched_via, "mismatched-via", 10),
            Err(SesWriteError::UnrepresentableRouteItem { reason, .. })
                if reason.contains("inherited via clearance")
        ));

        let mut mismatched_attach = test_board();
        mismatched_attach.insert_via(1, IntPoint::new(1_000, 0), vec![1], 1, true);
        assert!(matches!(
            export_ses(&mismatched_attach, "mismatched-attach", 10),
            Err(SesWriteError::UnrepresentableRouteItem { reason, .. })
                if reason.contains("attach permission")
        ));

        let mut stale_escape = test_board();
        stale_escape.insert_escape_via(1, IntPoint::new(1_000, 0), vec![1], 1, false, 0);
        assert!(matches!(
            export_ses(&stale_escape, "stale-escape", 10),
            Err(SesWriteError::UnrepresentableRouteItem { reason, .. })
                if reason.contains("escape metadata")
        ));
    }
}
