//! DSN export (Java: `DsnFile.write` / the GUI's "export Specctra design
//! file"): re-emits the imported design document with the wiring section
//! regenerated from the board's routed items.

use crate::board::basic_board::BasicBoard;
use crate::board::ItemKind;

/// Serializes the board as a Specctra design file. Requires the board to
/// have been imported from DSN (the non-wiring sections are preserved
/// verbatim; the router only changes the wiring).
pub fn export_dsn(board: &BasicBoard) -> Option<String> {
    let source = board.dsn_source.as_ref()?;
    // board units -> file units (the importer multiplies by resolution)
    let descale = |v: f64| -> f64 { v / board.resolution.max(1) as f64 };
    let mut wiring = String::new();
    wiring.push_str("  (wiring\n");
    for (_, item) in board.items() {
        if item.base.net_count() == 0 {
            continue;
        }
        let net_name = item
            .base
            .net_nos
            .first()
            .and_then(|&n| board.rules.nets.get_by_no(n))
            .map(|n| n.name.clone())
            .unwrap_or_default();
        match &item.kind {
            ItemKind::PolylineTrace(t) => {
                let layer_name = &board.layer_structure.arr[t.layer].name;
                wiring.push_str(&format!(
                    "    (wire\n      (path {} {}",
                    layer_name,
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
                    "\n      )\n      (net \"{net_name}\")\n      (type route)\n    )\n"
                ));
            }
            ItemKind::Via(v) => {
                if item.base.component_no != 0 {
                    continue; // pins belong to the placement, not the wiring
                }
                let Some(padstack) = board.padstacks.get_by_no(v.padstack) else {
                    continue;
                };
                wiring.push_str(&format!(
                    "    (via \"{}\" {} {}\n      (net \"{net_name}\")\n      (type route)\n    )\n",
                    padstack.name,
                    descale(v.center.x as f64),
                    descale(v.center.y as f64)
                ));
            }
            ItemKind::ObstacleArea(_) => {}
        }
    }
    wiring.push_str("  )\n");
    // insert the wiring before the document's final closing paren
    let end = source.rfind(')')?;
    let mut out = String::with_capacity(source.len() + wiring.len() + 2);
    out.push_str(&source[..end]);
    out.push_str(&wiring);
    out.push_str(&source[end..]);
    Some(out)
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
    }
}
