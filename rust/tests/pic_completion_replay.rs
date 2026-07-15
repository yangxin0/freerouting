//! Exact-data replay of the pic_programmer completion that produced the
//! leaking room 57 (net 21, layer 0): the completion collected the
//! blocking trace as an obstacle yet emitted a piece overlapping its
//! inflation. Reproduces the pipeline on the captured start shape and
//! ordered obstacle list, then asserts every output piece is clear of
//! every obstacle.

use freerouting::autoroute::room_completion::{restrain_all, IncompleteRoom};
use freerouting::geometry::planar::{IntPoint, Line, TileShape};
use std::rc::Rc;

fn parse() -> (TileShape, TileShape, Vec<(i32, Rc<TileShape>)>) {
    let data = include_str!("data/pic_completion57.txt");
    let mut start = None;
    let mut contained = None;
    let mut obstacles = Vec::new();
    for line in data.lines() {
        let mut parts = line.split(';');
        let tag = parts.next().unwrap();
        let quads: Vec<Vec<i32>> = parts
            .filter(|p| !p.is_empty())
            .map(|p| p.split(',').map(|v| v.parse().unwrap()).collect())
            .collect();
        let simplex = || {
            TileShape::Simplex(freerouting::geometry::planar::Simplex::new(
                quads
                    .iter()
                    .map(|q| Line::new(IntPoint::new(q[0], q[1]), IntPoint::new(q[2], q[3])))
                    .collect(),
            ))
        };
        if tag == "start" {
            start = Some(simplex());
        } else if tag == "containedx" {
            contained = Some(simplex());
        } else if let Some(id) = tag.strip_prefix("obstacle ") {
            obstacles.push((id.parse().unwrap(), Rc::new(simplex())));
        }
    }
    (start.unwrap(), contained.unwrap(), obstacles)
}

#[test]
fn replayed_completion_excludes_every_obstacle() {
    let (start, contained, obstacles) = parse();
    let pieces = restrain_all(
        vec![IncompleteRoom {
            shape: start,
            layer: 0,
            contained_shape: contained,
        }],
        &obstacles,
        |_, _| {},
    );
    assert!(!pieces.is_empty(), "completion killed everything");
    let mut dirty = 0;
    for (idx, piece) in pieces.iter().enumerate() {
        for (id, ob) in &obstacles {
            let isect = piece.shape.intersection(ob);
            if isect.dimension() >= 2 {
                dirty += 1;
                eprintln!(
                    "piece {idx} (bbox {:?}) overlaps obstacle {id} \
                     (bbox {:?}) isect bbox {:?}",
                    piece.shape.bounding_box(),
                    ob.bounding_box(),
                    isect.bounding_box(),
                );
            }
        }
    }
    assert_eq!(dirty, 0, "{dirty} piece-obstacle overlaps survived");
    // dump the piece matching the live leaked room's bbox for comparison
    for piece in &pieces {
        let bb = piece.shape.bounding_box();
        if bb.ll.x == 1443686 && bb.ll.y == -973514 {
            eprintln!("REPLAY piece {:?}", piece.shape.to_simplex());
        }
    }
}
