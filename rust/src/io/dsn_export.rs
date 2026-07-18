//! DSN export (Java: `DsnFile.write` / the GUI's "export Specctra design
//! file"): re-emits the imported design document with the wiring section
//! regenerated from the board's routed items.

use crate::board::basic_board::BasicBoard;
use crate::board::ItemKind;
use std::fmt;

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DsnWriteError {
    MissingSource,
    MalformedSource,
    /// The retained non-wiring source no longer describes the board.  A DSN
    /// export cannot safely rewrite only `(wiring ...)` in this state.
    StaleSource {
        reason: String,
    },
    InvalidBoard {
        reason: String,
    },
    DanglingReference {
        kind: &'static str,
        number: i64,
    },
    UnrepresentableRouteItem {
        item_id: crate::board::basic_board::ItemId,
        reason: &'static str,
    },
    UnrepresentableIdentifier {
        kind: &'static str,
        value: String,
    },
}

impl fmt::Display for DsnWriteError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingSource => write!(f, "the board was not imported from a DSN document"),
            Self::MalformedSource => write!(f, "the retained DSN document is not balanced"),
            Self::StaleSource { reason } => {
                write!(f, "the retained DSN source is stale: {reason}")
            }
            Self::InvalidBoard { reason } => write!(f, "invalid board: {reason}"),
            Self::DanglingReference { kind, number } => {
                write!(f, "the board references missing {kind} {number}")
            }
            Self::UnrepresentableRouteItem { item_id, reason } => {
                write!(f, "DSN cannot represent route item {item_id}: {reason}")
            }
            Self::UnrepresentableIdentifier { kind, value } => write!(
                f,
                "the {kind} identifier {value:?} contains both Specctra quote delimiters"
            ),
        }
    }
}

impl std::error::Error for DsnWriteError {}

/// Specctra has no escape production inside quoted atoms. Prefer the usual
/// double quote, fall back to a single quote, and reject a value containing
/// both delimiters instead of emitting a document that the reader cannot
/// parse back to the same name.
fn dsn_quoted(value: &str, kind: &'static str) -> Result<String, DsnWriteError> {
    let quote = if !value.contains('"') {
        '"'
    } else if !value.contains('\'') {
        '\''
    } else {
        return Err(DsnWriteError::UnrepresentableIdentifier {
            kind,
            value: value.to_string(),
        });
    };
    Ok(format!("{quote}{value}{quote}"))
}

/// Returns an unquoted token for ordinary DSN names and quotes names that
/// contain scanner delimiters. This keeps the common output compatible with
/// existing tools while making unusual layer names lossless.
fn dsn_atom(value: &str, kind: &'static str) -> Result<String, DsnWriteError> {
    if !value.is_empty()
        && value.chars().all(|ch| {
            !ch.is_whitespace() && !matches!(ch, '(' | ')' | '"' | '\'' | '#' | ';' | '\\')
        })
    {
        Ok(value.to_string())
    } else {
        dsn_quoted(value, kind)
    }
}

/// Serializes the board as a Specctra design file. Requires the board to
/// have been imported from DSN (the non-wiring sections are preserved
/// verbatim; the router only changes the wiring).
pub fn export_dsn(board: &BasicBoard) -> Result<String, DsnWriteError> {
    let source = board
        .dsn_source
        .as_ref()
        .ok_or(DsnWriteError::MissingSource)?;
    let (root_close, _) = crate::io::dsn::document_structure(source, "wiring")
        .map_err(|_| DsnWriteError::MalformedSource)?;
    if let Some(reason) = board.dsn_source_staleness() {
        return Err(DsnWriteError::StaleSource { reason });
    }
    crate::board::validate_board_references(board).map_err(|error| {
        DsnWriteError::InvalidBoard {
            reason: error.to_string(),
        }
    })?;
    // DSN wiring has no component ownership field.  A component-owned trace
    // would therefore come back as free routing copper if it were emitted;
    // fail closed instead of silently changing its connectivity/placement
    // semantics on reload.
    for (item_id, item) in board.items() {
        if item.base.component_no != 0 && matches!(item.kind, ItemKind::PolylineTrace(_)) {
            return Err(DsnWriteError::UnrepresentableRouteItem {
                item_id: *item_id,
                reason: "DSN wiring cannot preserve component-owned traces",
            });
        }
        if let ItemKind::PolylineTrace(_) = &item.kind {
            if !item.base.clearance_class_explicit {
                let expected_clearance = item
                    .base
                    .net_nos
                    .first()
                    .map(|&net_no| board.rules.get_trace_clearance_class(net_no))
                    .unwrap_or_else(crate::rules::BoardRules::default_clearance_class);
                if item.base.clearance_class != expected_clearance {
                    return Err(DsnWriteError::UnrepresentableRouteItem {
                        item_id: *item_id,
                        reason: "DSN wiring cannot preserve the inherited trace clearance class",
                    });
                }
            }
            continue;
        }
        let ItemKind::Via(via) = &item.kind else {
            continue;
        };
        if item.base.component_no != 0 {
            continue;
        }
        if via.is_escape_via || via.escape_smd_layer.is_some() {
            return Err(DsnWriteError::UnrepresentableRouteItem {
                item_id: *item_id,
                reason: "DSN wiring cannot preserve escape-via metadata",
            });
        }
        // A wiring via has no attach attribute.  Check the exact value that
        // dsn_import derives from the retained net/via rule so a mismatched
        // in-memory override cannot silently become attachable (or vice
        // versa) after export and re-import.
        let expected_attach = item
            .base
            .net_nos
            .first()
            .and_then(|&net_no| board.rules.via_info_for_padstack(net_no, via.padstack))
            .map(|info| info.attach_smd_allowed())
            .unwrap_or_else(|| {
                board.rules.via_at_smd_allowed
                    && board
                        .padstacks
                        .get_by_no(via.padstack)
                        .is_some_and(|padstack| padstack.attach_allowed)
            });
        if via.attach_allowed != expected_attach {
            return Err(DsnWriteError::UnrepresentableRouteItem {
                item_id: *item_id,
                reason: "DSN wiring cannot preserve the via attach permission",
            });
        }
        if !item.base.clearance_class_explicit {
            let expected_clearance = item
                .base
                .net_nos
                .first()
                .and_then(|&net_no| {
                    board
                        .rules
                        .via_clearance_class_for_padstack(net_no, via.padstack)
                        .map(|class| {
                            if class == 0 {
                                board.rules.get_trace_clearance_class(net_no)
                            } else {
                                class
                            }
                        })
                })
                .unwrap_or_else(|| {
                    item.base
                        .net_nos
                        .first()
                        .map(|&net_no| {
                            board
                                .rules
                                .item_clearance_class_for(net_no, crate::rules::ItemClass::Via)
                        })
                        .unwrap_or_else(crate::rules::BoardRules::default_clearance_class)
                });
            if item.base.clearance_class != expected_clearance {
                return Err(DsnWriteError::UnrepresentableRouteItem {
                    item_id: *item_id,
                    reason: "DSN wiring cannot preserve the inherited via clearance class",
                });
            }
        }
    }
    // board units -> file units (the importer multiplies by resolution)
    let descale = |v: f64| -> f64 { v / board.resolution.max(1) as f64 };
    let mut wiring = String::new();
    wiring.push_str("  (wiring\n");
    // the wiring type carries the fixed state (Java Wiring.write_fixed_state:
    // shove_fixed / fix / protect; unfixed copper is plain route), so
    // protected wiring survives an export→import round trip
    let type_attr = |state: crate::board::FixedState| match state {
        crate::board::FixedState::ShoveFixed => "\n      (type shove_fixed)",
        crate::board::FixedState::SystemFixed => "\n      (type fix)",
        crate::board::FixedState::UserFixed => "\n      (type protect)",
        crate::board::FixedState::Unfixed => "",
    };
    // An explicit (clearance_class ...) is an item-level override.  An
    // inherited class must stay implicit: writing the resolved numeric class
    // here would turn every routed segment into an override and would prevent
    // a later rules-file/net-class change from taking effect after reload.
    let cl_attr = |item: &crate::board::Item| -> Result<String, DsnWriteError> {
        if !item.base.clearance_class_explicit {
            return Ok(String::new());
        }
        let name = board
            .rules
            .clearance_matrix
            .get_name(item.base.clearance_class)
            .ok_or(DsnWriteError::DanglingReference {
                kind: "clearance class",
                number: item.base.clearance_class as i64,
            })?;
        Ok(format!(
            "\n      (clearance_class {})",
            dsn_quoted(name, "clearance class")?
        ))
    };
    for (item_id, item) in board.items() {
        if matches!(item.kind, ItemKind::ObstacleArea(_))
            || (item.base.component_no != 0 && matches!(item.kind, ItemKind::Via(_)))
        {
            continue;
        }
        let net_line = match item.base.net_nos.as_slice() {
            [] => String::new(),
            [net_no] => {
                let net = board.rules.nets.get_by_no(*net_no).ok_or(
                    DsnWriteError::DanglingReference {
                        kind: "net",
                        number: *net_no as i64,
                    },
                )?;
                format!(
                    "\n      (net {} {})",
                    dsn_quoted(&net.name, "net")?,
                    net.subnet_number
                )
            }
            _ => {
                return Err(DsnWriteError::UnrepresentableRouteItem {
                    item_id: *item_id,
                    reason: "Specctra wiring items may belong to at most one net",
                });
            }
        };
        match &item.kind {
            ItemKind::PolylineTrace(t) => {
                let layer_name = &board
                    .layer_structure
                    .arr
                    .get(t.layer)
                    .ok_or(DsnWriteError::DanglingReference {
                        kind: "layer",
                        number: t.layer as i64,
                    })?
                    .name;
                let layer_token = dsn_atom(layer_name, "layer")?;
                wiring.push_str(&format!(
                    "    (wire\n      (path {} {}",
                    layer_token,
                    descale(2.0 * t.half_width as f64)
                ));
                for corner in t.polyline.corner_approx_arr() {
                    wiring.push_str(&format!(
                        "\n        {} {}",
                        descale(corner.x.round()),
                        descale(corner.y.round())
                    ));
                }
                wiring.push_str(&format!(
                    "\n      ){net_line}{}{}\n    )\n",
                    type_attr(item.base.fixed_state),
                    cl_attr(item)?
                ));
            }
            ItemKind::Via(v) => {
                let padstack = board.padstacks.get_by_no(v.padstack).ok_or(
                    DsnWriteError::DanglingReference {
                        kind: "padstack",
                        number: v.padstack as i64,
                    },
                )?;
                let padstack_token = dsn_quoted(&padstack.name, "padstack")?;
                wiring.push_str(&format!(
                    "    (via {} {} {}{net_line}{}{}\n    )\n",
                    padstack_token,
                    descale(v.center.x as f64),
                    descale(v.center.y as f64),
                    type_attr(item.base.fixed_state),
                    cl_attr(item)?
                ));
            }
            ItemKind::ObstacleArea(_) => {}
        }
    }
    wiring.push_str("  )\n");
    // insert the wiring before the document's final closing paren
    let mut out = String::with_capacity(source.len() + wiring.len() + 2);
    out.push_str(&source[..root_close]);
    out.push_str(&wiring);
    out.push_str(&source[root_close..]);
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::import_dsn;

    const MINI_DSN: &str = r#"(pcb "mini.dsn"
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (rect pcb -1000 -1000 50000 50000))
    (rule (width 200) (clearance 200))
  )
  (placement)
  (library
    (padstack "Via[0-1]_600:300_um"
      (shape (circle F.Cu 600 0 0))
      (shape (circle B.Cu 600 0 0))
      (attach off)
    )
  )
  (network
    (net "N1")
  )
)"#;

    #[test]
    fn reexport_of_prerouted_source_is_balanced() {
        // Issue026 carries a (wiring ...) section and a lone-quote
        // (string_quote ") atom; the raw-source surgery must remove the
        // wiring span exactly, or the re-export gains/loses a paren
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/..");
        let path = format!("{root}/fixtures/Issue026-J2_reference.dsn");
        let content = std::fs::read_to_string(&path).expect("fixture missing from checkout");
        let board = import_dsn(&content).expect("import");
        let out = export_dsn(&board).expect("export");
        let board2 = import_dsn(&out).expect("re-import of the exported design");
        let traces = |b: &crate::board::basic_board::BasicBoard| {
            b.items()
                .filter(|(_, it)| matches!(it.kind, ItemKind::PolylineTrace(_)))
                .count()
        };
        assert_eq!(traces(&board), traces(&board2), "wiring must survive");
    }

    #[test]
    fn issue413_wiring_round_trip_neither_doubles_nor_violates() {
        // the review repro: Issue413 carries a (wiring ...) section; the
        // export→import round trip must keep the same copper count and
        // introduce no clearance violations (a case-mismatch in the strip
        // once doubled the copper: 8 violations)
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/..");
        let path = format!("{root}/fixtures/Issue413-test.dsn");
        let content = std::fs::read_to_string(&path).expect("fixture missing from checkout");
        let board = import_dsn(&content).expect("import");
        let traces = |b: &crate::board::basic_board::BasicBoard| {
            b.items()
                .filter(|(_, it)| matches!(it.kind, ItemKind::PolylineTrace(_)))
                .count()
        };
        let out = export_dsn(&board).expect("export");
        let board2 = import_dsn(&out).expect("re-import");
        assert_eq!(traces(&board), traces(&board2), "copper must not double");
        let v0 = crate::drc::check_board(&board).violations.len();
        let v1 = crate::drc::check_board(&board2).violations.len();
        assert!(
            v1 <= v0,
            "round trip must not create violations ({v0} -> {v1})"
        );
    }

    #[test]
    fn round_trips_and_reimports() {
        let mut board = import_dsn(MINI_DSN).expect("import");
        board.insert_trace(
            crate::geometry::planar::Polyline::from_int_points(&[
                crate::geometry::planar::IntPoint::new(1000, 1000),
                crate::geometry::planar::IntPoint::new(9000, 1000),
            ]),
            0,
            100,
            vec![1],
            1,
        );
        board.insert_via(
            1,
            crate::geometry::planar::IntPoint::new(9000, 1000),
            vec![1],
            1,
            false,
        );
        let out = export_dsn(&board).expect("export");
        assert!(out.contains("(wiring"));
        assert!(out.contains("(path F.Cu 20"), "width must be in file units");
        // the exported document must re-import, with the wire present
        let board2 = import_dsn(&out).expect("re-import");
        let traces = board2
            .items()
            .filter(|(_, it)| matches!(it.kind, ItemKind::PolylineTrace(_)))
            .count();
        assert_eq!(traces, 1, "the exported wire must survive re-import");
        assert_eq!(
            board2
                .items()
                .filter(|(_, item)| matches!(item.kind, ItemKind::Via(_))
                    && item.base.component_no == 0)
                .count(),
            1,
            "the exported via must survive re-import"
        );
    }

    #[test]
    fn wiring_exports_only_explicit_clearance_overrides() {
        let mut board = import_dsn(MINI_DSN).expect("import");
        let inherited = board.insert_trace(
            crate::geometry::planar::Polyline::from_int_points(&[
                crate::geometry::planar::IntPoint::new(1000, 1000),
                crate::geometry::planar::IntPoint::new(9000, 1000),
            ]),
            0,
            100,
            vec![1],
            1,
        );
        let explicit = board.insert_trace(
            crate::geometry::planar::Polyline::from_int_points(&[
                crate::geometry::planar::IntPoint::new(1000, 3000),
                crate::geometry::planar::IntPoint::new(9000, 3000),
            ]),
            0,
            100,
            vec![1],
            1,
        );
        board.set_item_clearance_class_explicit(explicit, true);

        let out = export_dsn(&board).expect("export");
        assert_eq!(
            out.matches("(clearance_class \"default\")").count(),
            1,
            "only the explicit wire should carry an item override"
        );
        let board2 = import_dsn(&out).expect("re-import");
        let flags: Vec<bool> = board2
            .items()
            .filter_map(|(_, item)| match &item.kind {
                ItemKind::PolylineTrace(t) if t.polyline.first_corner().to_float().y < 2000.0 => {
                    Some(item.base.clearance_class_explicit)
                }
                ItemKind::PolylineTrace(_) => Some(item.base.clearance_class_explicit),
                _ => None,
            })
            .collect();
        assert_eq!(flags.iter().filter(|&&v| v).count(), 1);
        assert!(board2
            .items()
            .any(|(_, item)| matches!(&item.kind, ItemKind::PolylineTrace(t)
                    if t.polyline.first_corner().to_float().y > 2000.0
                        && item.base.clearance_class_explicit)));
        assert!(board2
            .items()
            .any(|(_, item)| matches!(&item.kind, ItemKind::PolylineTrace(t)
                    if t.polyline.first_corner().to_float().y < 2000.0
                        && !item.base.clearance_class_explicit)));
        let _ = inherited;
    }

    #[test]
    fn netless_route_copper_survives_round_trip() {
        let mut board = import_dsn(MINI_DSN).expect("import");
        board.insert_trace(
            crate::geometry::planar::Polyline::from_int_points(&[
                crate::geometry::planar::IntPoint::new(1_000, 1_000),
                crate::geometry::planar::IntPoint::new(9_000, 1_000),
            ]),
            0,
            100,
            Vec::new(),
            1,
        );
        board.insert_via(
            1,
            crate::geometry::planar::IntPoint::new(9_000, 1_000),
            Vec::new(),
            1,
            false,
        );
        let before_drc = crate::drc::violation_keys(&board);
        let out = export_dsn(&board).expect("export");
        let reloaded = import_dsn(&out).expect("re-import");
        let netless_route_items: Vec<_> = reloaded
            .items()
            .filter(|(_, item)| {
                item.base.component_no == 0
                    && item.base.net_count() == 0
                    && matches!(item.kind, ItemKind::PolylineTrace(_) | ItemKind::Via(_))
            })
            .collect();
        assert_eq!(netless_route_items.len(), 2);
        assert_eq!(crate::drc::violation_keys(&reloaded), before_drc);
    }

    #[test]
    fn route_only_changes_keep_the_retained_source_exportable() {
        let mut board = import_dsn(MINI_DSN).expect("import");
        // Warming lazy shape/bounding-box caches is not a semantic edit and
        // must not invalidate the retained source either.
        for (_, item) in board.items() {
            let _ = item.tile_shapes(&board.padstacks);
            let _ = item.bounding_box(&board.padstacks);
        }
        board.insert_trace(
            crate::geometry::planar::Polyline::from_int_points(&[
                crate::geometry::planar::IntPoint::new(2_000, 2_000),
                crate::geometry::planar::IntPoint::new(8_000, 2_000),
            ]),
            0,
            100,
            vec![1],
            1,
        );
        board.insert_via(
            1,
            crate::geometry::planar::IntPoint::new(8_000, 2_000),
            vec![1],
            1,
            false,
        );
        assert!(export_dsn(&board).is_ok());
    }

    #[test]
    fn rejects_component_traces_and_nonrepresentable_vias() {
        let mut component_trace = import_dsn(MINI_DSN).expect("import");
        let trace = component_trace.insert_trace(
            crate::geometry::planar::Polyline::from_int_points(&[
                crate::geometry::planar::IntPoint::new(1000, 1000),
                crate::geometry::planar::IntPoint::new(9000, 1000),
            ]),
            0,
            100,
            vec![1],
            1,
        );
        component_trace.set_component_no(trace, 7);
        assert!(matches!(
            export_dsn(&component_trace),
            Err(DsnWriteError::UnrepresentableRouteItem { reason, .. })
                if reason.contains("component-owned traces")
        ));

        // MINI_DSN's padstack is attach-off and the default global rule also
        // disallows attaching, so an attach-on route via cannot be represented
        // by its wiring record.
        let mut mismatched_attach = import_dsn(MINI_DSN).expect("import");
        mismatched_attach.insert_via(
            1,
            crate::geometry::planar::IntPoint::new(9000, 1000),
            vec![1],
            1,
            true,
        );
        assert!(matches!(
            export_dsn(&mismatched_attach),
            Err(DsnWriteError::UnrepresentableRouteItem { reason, .. })
                if reason.contains("attach permission")
        ));

        let mut mismatched_clearance = import_dsn(MINI_DSN).expect("import");
        mismatched_clearance.insert_via(
            1,
            crate::geometry::planar::IntPoint::new(9000, 1000),
            vec![1],
            crate::rules::BoardRules::clearance_class_none(),
            false,
        );
        assert!(matches!(
            export_dsn(&mismatched_clearance),
            Err(DsnWriteError::UnrepresentableRouteItem { reason, .. })
                if reason.contains("inherited via clearance")
        ));

        let mut mismatched_trace = import_dsn(MINI_DSN).expect("import");
        mismatched_trace.insert_trace(
            crate::geometry::planar::Polyline::from_int_points(&[
                crate::geometry::planar::IntPoint::new(1000, 1000),
                crate::geometry::planar::IntPoint::new(9000, 1000),
            ]),
            0,
            100,
            vec![1],
            crate::rules::BoardRules::clearance_class_none(),
        );
        assert!(matches!(
            export_dsn(&mismatched_trace),
            Err(DsnWriteError::UnrepresentableRouteItem { reason, .. })
                if reason.contains("inherited trace clearance")
        ));

        let mut escape = import_dsn(MINI_DSN).expect("import");
        escape.insert_escape_via(
            1,
            crate::geometry::planar::IntPoint::new(9000, 1000),
            vec![1],
            1,
            false,
            0,
        );
        assert!(matches!(
            export_dsn(&escape),
            Err(DsnWriteError::UnrepresentableRouteItem { reason, .. })
                if reason.contains("escape-via metadata")
        ));
    }

    #[test]
    fn rejects_rule_outline_and_static_area_changes() {
        let mut rules_changed = import_dsn(MINI_DSN).expect("import");
        rules_changed.rules.clearance_matrix.set_default_value(444);
        assert!(matches!(
            export_dsn(&rules_changed),
            Err(DsnWriteError::StaleSource { ref reason })
                if reason.contains("routing rules")
        ));

        let mut outline_changed = import_dsn(MINI_DSN).expect("import");
        outline_changed.outline.as_mut().expect("outline").1 += 1;
        assert!(matches!(
            export_dsn(&outline_changed),
            Err(DsnWriteError::StaleSource { ref reason })
                if reason.contains("board outline")
        ));

        let mut area_changed = import_dsn(MINI_DSN).expect("import");
        let area_id = area_changed
            .items()
            .find_map(|(id, item)| matches!(item.kind, ItemKind::ObstacleArea(_)).then_some(*id))
            .expect("boundary keepout area");
        area_changed.set_area_is_obstacle(area_id, true);
        assert!(matches!(
            export_dsn(&area_changed),
            Err(DsnWriteError::StaleSource { ref reason })
                if reason.contains("static pins or obstacle areas")
        ));
    }

    #[test]
    fn rejects_unrepresentable_wiring_names_instead_of_emitting_invalid_dsn() {
        let mut board = import_dsn(MINI_DSN).expect("import");
        board.rules.nets.get_by_no_mut(1).unwrap().name = "both \"quotes\" and 'quotes'".into();
        board.insert_trace(
            crate::geometry::planar::Polyline::from_int_points(&[
                crate::geometry::planar::IntPoint::new(1000, 1000),
                crate::geometry::planar::IntPoint::new(9000, 1000),
            ]),
            0,
            100,
            vec![1],
            1,
        );
        assert!(matches!(
            export_dsn(&board),
            Err(DsnWriteError::StaleSource { ref reason })
                if reason.contains("net definitions")
        ));
    }

    #[test]
    fn reports_missing_source_distinctly() {
        let mut board = import_dsn(MINI_DSN).expect("import");
        board.dsn_source = None;
        assert_eq!(export_dsn(&board), Err(DsnWriteError::MissingSource));
    }

    #[test]
    fn rejects_mutated_retained_source_even_when_board_semantics_match() {
        let mut board = import_dsn(MINI_DSN).expect("import");
        let source = board.dsn_source.clone().expect("retained source");
        // Keep the document balanced and semantically equivalent while
        // changing the text that the exporter would otherwise splice into.
        board.dsn_source = Some(format!("{source}\n# caller mutation\n"));
        assert!(matches!(
            export_dsn(&board),
            Err(DsnWriteError::StaleSource { ref reason })
                if reason.contains("retained DSN source changed")
        ));
    }
}
