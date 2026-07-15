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
                    2 * t.half_width
                ));
                for corner in t.polyline.corner_approx_arr() {
                    wiring.push_str(&format!(
                        "\n        {} {}",
                        corner.x.round() as i64,
                        corner.y.round() as i64
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
                    padstack.name, v.center.x, v.center.y
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
        assert!(out.contains("(path F.Cu 200"));
        // the exported document must re-import, with the wire present
        let board2 = import_dsn(&out).expect("re-import");
        let traces = board2
            .items()
            .filter(|(_, it)| matches!(it.kind, ItemKind::PolylineTrace(_)))
            .count();
        assert_eq!(traces, 1, "the exported wire must survive re-import");
    }
}
