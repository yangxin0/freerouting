//! Semantic DSN import (port of the reading half of
//! `io/specctra/DsnReader.java` and the Structure/Library/Network scope
//! classes, simplified): maps a parsed DSN tree onto a [`BasicBoard`]
//! with layers, padstacks, placed component pins and nets.
//!
//! Simplifications (documented): pad shapes are converted to boxes /
//! bounding octagons (circles/ovals), back-side placement mirrors pin
//! offsets at the y axis without padstack layer mirroring, and wiring /
//! keepout import follows later.

use std::collections::HashMap;

use crate::board::basic_board::BasicBoard;
use crate::board::{Layer, LayerStructure};
use crate::core::Padstacks;
use crate::geometry::planar::{Circle, IntBox, IntPoint, TileShape};
use crate::io::dsn::{parse_dsn, SExpr};
use crate::rules::{BoardRules, ClearanceMatrix};

#[derive(Debug, Clone, PartialEq)]
pub struct ImportError(pub String);

impl std::fmt::Display for ImportError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "DSN import error: {}", self.0)
    }
}

impl std::error::Error for ImportError {}

fn err(message: impl Into<String>) -> ImportError {
    ImportError(message.into())
}

struct ImagePin {
    padstack_name: String,
    pin_name: String,
    dx: f64,
    dy: f64,
}

/// Imports a DSN file into a board: layers, default rules, padstacks,
/// component pins (as drill items carrying their nets) and nets.
pub fn import_dsn(content: &str) -> Result<BasicBoard, ImportError> {
    let pcb = parse_dsn(content).map_err(|e| err(e.to_string()))?;
    if !pcb.name().is_some_and(|n| n.eq_ignore_ascii_case("pcb")) {
        return Err(err("root node is not (pcb ...)"));
    }
    let structure = pcb.child("structure").ok_or_else(|| err("no structure"))?;

    // resolution: file coordinates are multiplied by this factor to get
    // integer board units
    let resolution: f64 = pcb
        .child("resolution")
        .and_then(|r| r.args().nth(1))
        .and_then(|a| a.parse().ok())
        .unwrap_or(1.0);
    let scale = |v: f64| -> i32 { (v * resolution).round() as i32 };

    // layers
    let mut layers: Vec<(String, bool, i64)> = Vec::new();
    for layer_node in structure.children("layer") {
        let name = layer_node.arg().ok_or_else(|| err("layer without name"))?;
        let is_signal = layer_node
            .child("type")
            .and_then(|t| t.arg())
            .is_none_or(|t| t.eq_ignore_ascii_case("signal"));
        let index = layer_node
            .child("property")
            .and_then(|p| p.child("index"))
            .and_then(|i| i.arg())
            .and_then(|a| a.parse().ok())
            .unwrap_or(layers.len() as i64);
        layers.push((name.to_string(), is_signal, index));
    }
    if layers.is_empty() {
        return Err(err("no layers"));
    }
    layers.sort_by_key(|(_, _, index)| *index);
    let layer_structure = LayerStructure::new(
        layers
            .iter()
            .map(|(name, is_signal, _)| Layer::new(name.clone(), *is_signal))
            .collect(),
    );
    let layer_no = |name: &str| -> Option<usize> { layer_structure.get_no(name) };
    let layer_count = layer_structure.layer_count();

    // default rules
    let default_clearance = structure
        .child("rule")
        .and_then(|r| r.child("clearance"))
        .and_then(|c| c.arg_f64())
        .map(|v| scale(v))
        .unwrap_or(200);
    let default_width = structure
        .child("rule")
        .and_then(|r| r.child("width"))
        .and_then(|w| w.arg_f64())
        .map(|v| scale(v))
        .unwrap_or(250);
    let clearance_matrix =
        ClearanceMatrix::get_default_instance(layer_structure.clone(), default_clearance);
    let mut rules = BoardRules::new(layer_structure.clone(), clearance_matrix);
    rules.get_default_net_class();
    rules.set_default_trace_half_widths((default_width / 2).max(1));

    // library: padstacks and images
    let mut padstacks = Padstacks::new(layer_count);
    let mut padstack_nos: HashMap<String, usize> = HashMap::new();
    let mut images: HashMap<String, Vec<ImagePin>> = HashMap::new();
    for library in pcb.children("library") {
        for padstack_node in library.children("padstack") {
            let name = padstack_node
                .arg()
                .ok_or_else(|| err("padstack without name"))?;
            let mut shapes: Vec<Option<TileShape>> = vec![None; layer_count];
            for shape_node in padstack_node.children("shape") {
                let Some(inner) = shape_node.as_list().and_then(|l| l.get(1)) else {
                    continue;
                };
                let Some((shape, layer_name)) = read_pad_shape(inner, &scale) else {
                    continue;
                };
                if let Some(l) = layer_no(&layer_name) {
                    shapes[l] = Some(shape);
                }
            }
            let attach = padstack_node
                .child("attach")
                .and_then(|a| a.arg())
                .is_some_and(|v| v.eq_ignore_ascii_case("on"));
            let no = padstacks.add(name, shapes, attach, false);
            padstack_nos.insert(
                padstacks.get_by_no(no).unwrap().name.clone(),
                no,
            );
        }
        for image_node in library.children("image") {
            let name = image_node.arg().ok_or_else(|| err("image without name"))?;
            let mut pins = Vec::new();
            for pin_node in image_node.children("pin") {
                let mut args = pin_node.args();
                let padstack_name = args.next().unwrap_or_default().to_string();
                // optional (rotate ...) etc. are lists, args() skips them;
                // the remaining atoms are pin name and the two offsets
                let rest: Vec<&str> = args.collect();
                if rest.len() < 3 {
                    continue;
                }
                let pin_name = rest[rest.len() - 3].to_string();
                let dx: f64 = rest[rest.len() - 2].parse().unwrap_or(0.0);
                let dy: f64 = rest[rest.len() - 1].parse().unwrap_or(0.0);
                pins.push(ImagePin {
                    padstack_name,
                    pin_name,
                    dx,
                    dy,
                });
            }
            images.insert(name.to_string(), pins);
        }
    }

    // network: pin reference "COMP-PIN" -> net number (split at the last
    // '-', like Java)
    let mut pin_nets: HashMap<String, i32> = HashMap::new();
    for network in pcb.children("network") {
        for net_node in network.children("net") {
            let net_name = net_node.arg().unwrap_or_default();
            let net_no = rules.nets.add(net_name, 1, false);
            for pins_node in net_node.children("pins") {
                for pin_ref in pins_node.args() {
                    pin_nets.insert(pin_ref.to_string(), net_no);
                }
            }
        }
    }

    let mut board = BasicBoard::new(layer_structure, rules, padstacks);

    // boundary: keepout strips along the outline edges on all layers so
    // routes stay inside the board (Java: BoardOutline tree shapes)
    if let Some(boundary) = structure.child("boundary") {
        if let Some(path) = boundary.child("path") {
            let coords: Vec<f64> = path
                .args()
                .skip(2)
                .filter_map(|a| a.parse().ok())
                .collect();
            let corners: Vec<IntPoint> = coords
                .chunks_exact(2)
                .map(|c| IntPoint::new(scale(c[0]), scale(c[1])))
                .collect();
            insert_boundary_keepouts(&mut board, &corners, default_clearance / 2);
        }
    }

    // placement: instantiate the image pins per component place
    for placement in pcb.children("placement") {
        for component in placement.children("component") {
            let image_name = component.arg().unwrap_or_default();
            let Some(image_pins) = images.get(image_name) else {
                continue;
            };
            for place in component.children("place") {
                let args: Vec<&str> = place.args().collect();
                if args.len() < 4 {
                    continue;
                }
                let refdes = args[0];
                let x: f64 = args[1].parse().unwrap_or(0.0);
                let y: f64 = args[2].parse().unwrap_or(0.0);
                let on_front = args[3].eq_ignore_ascii_case("front");
                let rotation_deg: f64 = args.get(4).and_then(|a| a.parse().ok()).unwrap_or(0.0);
                let rotation = rotation_deg.to_radians();
                let (sin, cos) = rotation.sin_cos();
                for pin in image_pins {
                    let Some(&padstack_no) = padstack_nos.get(&pin.padstack_name) else {
                        continue;
                    };
                    // rotate the offset, mirror for back side
                    let (mut dx, dy) = (
                        pin.dx * cos - pin.dy * sin,
                        pin.dx * sin + pin.dy * cos,
                    );
                    if !on_front {
                        dx = -dx;
                    }
                    let center = IntPoint::new(scale(x + dx), scale(y + dy));
                    let pin_ref = format!("{refdes}-{}", pin.pin_name);
                    let net_nos = pin_nets
                        .get(&pin_ref)
                        .map(|n| vec![*n])
                        .unwrap_or_default();
                    let attach_allowed = board
                        .padstacks
                        .get_by_no(padstack_no)
                        .is_some_and(|p| p.attach_allowed);
                    let id = board.insert_via(padstack_no, center, net_nos, 1, attach_allowed);
                    // pins belong to their component: protected from ripup
                    // and not written to session files
                    board.set_component_no(id, 1);
                }
            }
        }
    }
    Ok(board)
}

/// Inserts thin keepout strips along the closed outline given by
/// `corners` on every layer, so routes cannot cross the board boundary
/// (Java: the tree shapes of `BoardOutline`).
fn insert_boundary_keepouts(board: &mut BasicBoard, corners: &[IntPoint], half_width: i32) {
    use crate::geometry::planar::{PolygonShape, PolylineArea};
    if corners.len() < 2 {
        return;
    }
    let half_width = half_width.max(1);
    let layer_count = board.layer_structure.layer_count();
    let mut edges: Vec<(IntPoint, IntPoint)> = corners
        .windows(2)
        .map(|w| (w[0], w[1]))
        .filter(|(a, b)| a != b)
        .collect();
    // close the outline if the file did not repeat the first corner
    if corners.first() != corners.last() {
        edges.push((*corners.last().unwrap(), corners[0]));
    }
    for (a, b) in edges {
        // a thin rectangle strip around the edge
        let line = crate::geometry::planar::FloatLine::new(a.to_float(), b.to_float());
        let left = line.translate(half_width as f64);
        let right = line.translate(-(half_width as f64));
        let strip = PolygonShape::from_int_points(&[
            left.a.round(),
            left.b.round(),
            right.b.round(),
            right.a.round(),
        ]);
        if strip.dimension() < 2 {
            continue;
        }
        let area = PolylineArea::new(strip, vec![]);
        for layer in 0..layer_count {
            board.insert_area(area.clone(), layer, "boundary", vec![], 1, false);
        }
    }
}

/// Reads a single pad shape node (circle / rect / path / polygon),
/// returning the tile shape (relative to the pad center) and its layer
/// name.
fn read_pad_shape(node: &SExpr, scale: &dyn Fn(f64) -> i32) -> Option<(TileShape, String)> {
    let kind = node.name()?;
    let layer_name = node.arg()?.to_string();
    let nums: Vec<f64> = node
        .args()
        .skip(1)
        .filter_map(|a| a.parse().ok())
        .collect();
    let shape = if kind.eq_ignore_ascii_case("circle") {
        // (circle LAYER diameter [cx cy])
        let diameter = *nums.first()?;
        let cx = nums.get(1).copied().unwrap_or(0.0);
        let cy = nums.get(2).copied().unwrap_or(0.0);
        let circle = Circle::new(
            IntPoint::new(scale(cx), scale(cy)),
            scale(diameter / 2.0).max(1),
        );
        circle.bounding_tile()
    } else if kind.eq_ignore_ascii_case("rect") {
        // (rect LAYER x1 y1 x2 y2)
        if nums.len() < 4 {
            return None;
        }
        TileShape::Box(IntBox::from_coords(
            scale(nums[0].min(nums[2])),
            scale(nums[1].min(nums[3])),
            scale(nums[0].max(nums[2])),
            scale(nums[1].max(nums[3])),
        ))
    } else if kind.eq_ignore_ascii_case("path") {
        // (path LAYER width x1 y1 x2 y2 ...): an oval; approximated by
        // the bounding box of the path offset by half the width
        if nums.len() < 5 {
            return None;
        }
        let half_width = nums[0] / 2.0;
        let xs: Vec<f64> = nums[1..].iter().step_by(2).copied().collect();
        let ys: Vec<f64> = nums[2..].iter().step_by(2).copied().collect();
        let min_x = xs.iter().cloned().fold(f64::MAX, f64::min) - half_width;
        let max_x = xs.iter().cloned().fold(f64::MIN, f64::max) + half_width;
        let min_y = ys.iter().cloned().fold(f64::MAX, f64::min) - half_width;
        let max_y = ys.iter().cloned().fold(f64::MIN, f64::max) + half_width;
        TileShape::Box(IntBox::from_coords(
            scale(min_x),
            scale(min_y),
            scale(max_x),
            scale(max_y),
        ))
    } else if kind.eq_ignore_ascii_case("polygon") {
        // (polygon LAYER aperture x1 y1 ...): bounding box approximation
        if nums.len() < 5 {
            return None;
        }
        let xs: Vec<f64> = nums[1..].iter().step_by(2).copied().collect();
        let ys: Vec<f64> = nums[2..].iter().step_by(2).copied().collect();
        TileShape::Box(IntBox::from_coords(
            scale(xs.iter().cloned().fold(f64::MAX, f64::min)),
            scale(ys.iter().cloned().fold(f64::MAX, f64::min)),
            scale(xs.iter().cloned().fold(f64::MIN, f64::max)),
            scale(ys.iter().cloned().fold(f64::MIN, f64::max)),
        ))
    } else {
        return None;
    };
    Some((shape, layer_name))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn imports_real_fixture() {
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/..");
        let path = format!("{root}/fixtures/Issue093-interf_u.dsn");
        let Ok(content) = std::fs::read_to_string(&path) else {
            return; // fixture not present
        };
        let board = import_dsn(&content).expect("import failed");
        assert_eq!(board.layer_structure.layer_count(), 2);
        assert_eq!(board.layer_structure.arr[0].name, "top_copper");
        assert!(board.padstacks.count() > 10);
        assert!(board.rules.nets.max_net_no() > 10);
        // every pin landed as an item; the interf_u board has hundreds
        let pin_count = board
            .items()
            .filter(|(_, i)| matches!(i.kind, crate::board::ItemKind::Via(_)))
            .count();
        assert!(pin_count > 100, "only {pin_count} pins imported");
        // a known net exists and is not yet routed
        let ack = board
            .rules
            .nets
            .get_by_name("/ACK")
            .first()
            .map(|n| n.net_number)
            .expect("/ACK net missing");
        let ack_pins: Vec<_> = board
            .items()
            .filter(|(_, item)| item.base.contains_net(ack))
            .collect();
        assert_eq!(ack_pins.len(), 2);
        assert!(!board.net_is_completely_connected(ack));
        // pins carry shapes usable by the search tree
        let (_, item) = ack_pins[0];
        assert!(item.tile_shape_count(&board.padstacks) >= 1);
    }

    #[test]
    fn imports_empty_board_with_boundary_keepouts() {
        use crate::board::ItemKind;
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/..");
        let path = format!("{root}/fixtures/empty_board.dsn");
        let Ok(content) = std::fs::read_to_string(&path) else {
            return;
        };
        let board = import_dsn(&content).expect("import failed");
        assert_eq!(board.layer_structure.layer_count(), 2);
        // no pins, but the boundary keepout strips are present
        assert!(board.item_count() > 0);
        assert!(board
            .items()
            .all(|(_, i)| matches!(i.kind, ItemKind::ObstacleArea(_))));
        // the boundary blocks any net at the outline (coordinates from the
        // file, scaled by resolution 10): the left border is x = 1295400
        let on_border = TileShape::Box(IntBox::from_coords(
            1295300, -800000, 1295500, -799000,
        ));
        assert!(board.is_blocked(&on_border, 0, 1));
        // but the interior is free
        let inside = TileShape::Box(IntBox::from_coords(
            1500000, -800000, 1500200, -799800,
        ));
        assert!(!board.is_blocked(&inside, 0, 1));
    }

    #[test]
    fn rejects_non_pcb() {
        assert!(import_dsn("(session x)").is_err());
        assert!(import_dsn("(pcb x)").is_err()); // no structure
    }
}
