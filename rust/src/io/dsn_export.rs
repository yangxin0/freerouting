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
                "the {kind} identifier {value:?} contains the declared Specctra quote delimiter"
            ),
        }
    }
}

impl std::error::Error for DsnWriteError {}

/// Specctra has no escape production inside quoted atoms.  Use the delimiter
/// declared by the retained source parser; silently switching to the other
/// quote would make KiCad/Java lex it as an ordinary character.
fn dsn_quoted(value: &str, kind: &'static str, quote: char) -> Result<String, DsnWriteError> {
    if value.contains(quote)
        || value
            .chars()
            .any(|character| matches!(character, '\r' | '\n' | '\0'))
    {
        return Err(DsnWriteError::UnrepresentableIdentifier {
            kind,
            value: value.to_string(),
        });
    }
    Ok(format!("{quote}{value}{quote}"))
}

/// NAME/LAYER_NAME scanner states permit quote characters after the first
/// character of a bare identifier. This matters when the identifier itself
/// contains the document's active delimiter: quoting it is impossible, but a
/// token such as `O'Net` is still losslessly representable in a single-quote
/// document. Keep this predicate deliberately narrower than the full legacy
/// ANSI grammar and fall back to quoted output for everything else.
fn is_safe_bare_name(value: &str, quote: char) -> bool {
    let mut chars = value.char_indices();
    let Some((_, first)) = chars.next() else {
        return false;
    };
    let ordinary = |character: char| {
        character.is_ascii_alphanumeric()
            || matches!(
                character,
                '_' | '.'
                    | '/'
                    | '\\'
                    | ':'
                    | '#'
                    | '$'
                    | '&'
                    | '>'
                    | '<'
                    | ','
                    | ';'
                    | '='
                    | '@'
                    | '['
                    | ']'
                    | '~'
                    | '*'
                    | '?'
                    | '!'
                    | '%'
                    | '^'
                    | '-'
                    | '+'
            )
    };
    if !ordinary(first) || (quote == '$' && first == '$') {
        return false;
    }
    if !chars.all(|(_, character)| ordinary(character) || matches!(character, '\'' | '"')) {
        return false;
    }

    // The Rust parser separates a quote after `_`/`-` when it forms the
    // quoted half of a composite clearance type. Avoid emitting that
    // ambiguous spelling for an ordinary identifier.
    let bytes = value.as_bytes();
    for index in 1..bytes.len() {
        let delimiter = bytes[index];
        if (matches!(delimiter, b'\'' | b'"') || (quote == '$' && delimiter == b'$'))
            && matches!(bytes[index - 1], b'_' | b'-')
            && bytes[index + 1..].contains(&delimiter)
        {
            return false;
        }
    }
    true
}

/// Prefer the writer's established quoted spelling, but use a legal bare
/// NAME token when the active delimiter occurs inside the identifier.
fn dsn_name(value: &str, kind: &'static str, quote: char) -> Result<String, DsnWriteError> {
    if value.contains(quote) && is_safe_bare_name(value, quote) {
        Ok(value.to_string())
    } else {
        dsn_quoted(value, kind, quote)
    }
}

/// Returns an unquoted token for ordinary DSN names and quotes names that
/// contain scanner delimiters. This keeps the common output compatible with
/// existing tools while making unusual layer names lossless.
fn dsn_atom(value: &str, kind: &'static str, quote: char) -> Result<String, DsnWriteError> {
    let ordinary_atom = !value.is_empty()
        && value.chars().all(|ch| {
            !ch.is_whitespace() && !matches!(ch, '(' | ')' | '"' | '\'' | '$' | '#' | ';' | '\\')
        });
    if ordinary_atom || (value.contains(quote) && is_safe_bare_name(value, quote)) {
        Ok(value.to_string())
    } else {
        dsn_quoted(value, kind, quote)
    }
}

fn source_quote_delimiter(source: &str) -> char {
    let Ok(root) = crate::io::dsn::parse_dsn(source) else {
        return '"';
    };
    root.child("parser")
        .and_then(|parser| parser.child("string_quote"))
        .and_then(|quote| quote.arg())
        .and_then(|value| value.chars().next())
        .filter(|quote| matches!(quote, '\'' | '"' | '$'))
        .unwrap_or('"')
}

/// Serializes the board as a Specctra design file. Requires the board to
/// have been imported from DSN (the non-wiring sections are preserved
/// verbatim; the router only changes the wiring).
pub fn export_dsn(board: &BasicBoard) -> Result<String, DsnWriteError> {
    let source = board
        .dsn_source
        .as_ref()
        .ok_or(DsnWriteError::MissingSource)?;
    let quote = source_quote_delimiter(source);
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
        // DSN wiring has no escape marker. The importer reconstructs the
        // narrow same-net SMD exemption from the via rule and the physical
        // pin overlap, so only canonical, derivable metadata is representable.
        let derived_escape_layer = item
            .base
            .net_nos
            .first()
            .filter(|_| !via.attach_allowed)
            .and_then(|&net_no| board.pure_smd_escape_layer(net_no, via.padstack, via.center));
        if via.is_escape_via != derived_escape_layer.is_some()
            || via.escape_smd_layer != derived_escape_layer
        {
            return Err(DsnWriteError::UnrepresentableRouteItem {
                item_id: *item_id,
                reason: "DSN wiring cannot reconstruct noncanonical escape-via metadata",
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
            dsn_name(name, "clearance class", quote)?
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
                    dsn_name(&net.name, "net", quote)?,
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
                let layer_token = dsn_atom(layer_name, "layer", quote)?;
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
                let padstack_token = dsn_name(&padstack.name, "padstack", quote)?;
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
    fn preserves_a_single_quote_source_delimiter_for_names_with_double_quotes() {
        // Specctra permits either quote delimiter in the parser declaration.
        // The retained source is the authority for newly emitted wiring: a
        // double-quoted token would be lexed as ordinary data by a consumer
        // whose document declares `'`.  The net name intentionally contains
        // the *other* quote to prove that we do not switch delimiters or
        // reject a representable identifier.
        let source = r#"(pcb 'mini"dsn'
  (parser (string_quote '))
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (rect pcb -1000 -1000 50000 50000))
    (rule (width 200) (clearance 200))
  )
  (placement)
  (library
    (padstack 'Via[0-1]_600:300_um'
      (shape (circle F.Cu 600 0 0))
      (shape (circle B.Cu 600 0 0))
      (attach off)
    )
  )
  (network
    (net 'N"1')
  )
)"#;
        let mut board = import_dsn(source).expect("single-quote DSN import");
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
        let exported = export_dsn(&board).expect("single-quote DSN export");
        assert!(exported.contains("(net 'N\"1' 1)"));
        let reloaded = import_dsn(&exported).expect("single-quote DSN re-import");
        assert_eq!(reloaded.rules.nets.get_by_no(1).unwrap().name, "N\"1");
        assert_eq!(
            reloaded
                .items()
                .filter(|(_, item)| matches!(item.kind, ItemKind::PolylineTrace(_)))
                .count(),
            1
        );
    }

    #[test]
    fn preserves_an_active_quote_inside_a_bare_identifier() {
        let source = r#"(pcb 'mini'
  (parser (string_quote '))
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (rect pcb -1000 -1000 50000 50000))
    (rule (width 200) (clearance 200))
  )
  (placement)
  (library
    (padstack 'Via[0-1]_600:300_um'
      (shape (circle F.Cu 600 0 0))
      (shape (circle B.Cu 600 0 0))
      (attach off)
    )
  )
  (network
    (net O'Net)
  )
)"#;
        let mut board = import_dsn(source).expect("single-quote DSN import");
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

        let exported = export_dsn(&board).expect("bare active-quote identifier export");
        assert!(exported.contains("(net O'Net 1)"));
        let reloaded = import_dsn(&exported).expect("bare active-quote DSN re-import");
        assert_eq!(reloaded.rules.nets.get_by_no(1).unwrap().name, "O'Net");
    }

    #[test]
    fn preserves_a_dollar_source_delimiter() {
        let source = r#"(pcb mini
  (parser (string_quote $) (space_in_quoted_tokens on))
  (resolution um 10)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (rect pcb -1000 -1000 50000 50000))
    (rule (width 200) (clearance 200))
  )
  (placement)
  (library
    (padstack $Via[0-1]_600:300_um$
      (shape (circle F.Cu 600 0 0))
      (shape (circle B.Cu 600 0 0))
      (attach off)
    )
  )
  (network
    (net $N 'quoted"$)
  )
)"#;
        let mut board = import_dsn(source).expect("dollar-quote DSN import");
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
        let exported = export_dsn(&board).expect("dollar-quote DSN export");
        assert!(exported.contains("(net $N 'quoted\"$ 1)"));
        let reloaded = import_dsn(&exported).expect("dollar-quote DSN re-import");
        assert_eq!(
            reloaded.rules.nets.get_by_no(1).unwrap().name,
            "N 'quoted\""
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
    fn canonical_escape_via_round_trips_as_derived_metadata() {
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/..");
        let source = std::fs::read_to_string(format!("{root}/fixtures/SMD-routing-issue-demo.dsn"))
            .expect("SMD fixture");
        let mut board = import_dsn(&source).expect("SMD fixture import");
        let last_layer = board.layer_structure.layer_count() - 1;
        let candidate = board.items().find_map(|(_, item)| {
            let ItemKind::Via(pin) = &item.kind else {
                return None;
            };
            if item.base.component_no == 0
                || board
                    .padstacks
                    .get_by_no(pin.padstack)
                    .is_none_or(|padstack| padstack.from_layer() != padstack.to_layer())
            {
                return None;
            }
            let net_no = *item.base.net_nos.first()?;
            (1..=board.padstacks.count()).find_map(|via_padstack| {
                let padstack = board.padstacks.get_by_no(via_padstack)?;
                if padstack.from_layer() != 0 || padstack.to_layer() < last_layer {
                    return None;
                }
                let layer = board.pure_smd_escape_layer(net_no, via_padstack, pin.center)?;
                Some((net_no, via_padstack, pin.center, layer))
            })
        });
        let (net_no, via_padstack, center, escape_layer) =
            candidate.expect("fixture must contain a pure-SMD escape site");
        let clearance_class = match board
            .rules
            .via_clearance_class_for_padstack(net_no, via_padstack)
        {
            Some(0) => board.rules.get_trace_clearance_class(net_no),
            Some(class) => class,
            None => board
                .rules
                .item_clearance_class_for(net_no, crate::rules::ItemClass::Via),
        };
        board.insert_escape_via(
            via_padstack,
            center,
            vec![net_no],
            clearance_class,
            false,
            escape_layer,
        );

        let text = export_dsn(&board).expect("canonical escape via export");
        let reloaded = import_dsn(&text).expect("canonical escape via re-import");
        let reloaded_via = reloaded
            .items()
            .find_map(|(_, item)| match &item.kind {
                ItemKind::Via(via)
                    if item.base.component_no == 0
                        && via.center == center
                        && via.padstack == via_padstack =>
                {
                    Some(via)
                }
                _ => None,
            })
            .expect("reloaded routing via");
        assert!(reloaded_via.is_escape_via);
        assert_eq!(reloaded_via.escape_smd_layer, Some(escape_layer));
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
