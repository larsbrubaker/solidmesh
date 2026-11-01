//! Whether the band ends on a rim, read per side: a foreign edge between a band group's triangle
//! and the far face's untagged one is judged by that group's own field (`field_of_tag`), never by
//! the chain's own field (the larger side) through the untagged triangle.

use std::cell::RefCell;
use std::rc::Rc;

use geometry_processing::bevel::level_line::consumed_to_the_rim::rims_the_band_ends_on;
use geometry_processing::bevel::level_line::{ChainField, LevelLineMesh};
use vector_math::Vector3;

/// The working mesh, the chain's edges and the chain's points.
type Fixture = (Rc<RefCell<LevelLineMesh>>, Vec<(i32, i32)>, Vec<Vector3>);

/// A flat side (tag 1, z = 0) running from the chain y = 0 to a crease at y = 2, past which the
/// untagged far face (tag 0) rises to y = 5, z = 3; four columns at x = -10, 0, 10 and 20. The
/// chain is vertices 0 to 3, the crease vertices 4 to 7.
fn a_side_ending_on_a_crease() -> Fixture {
    let xs = [-10.0, 0.0, 10.0, 20.0];
    let mut positions = Vec::new();
    for (y, z) in [(0.0, 0.0), (2.0, 0.0), (5.0, 3.0)] {
        for &x in &xs {
            positions.push(Vector3::new(x, y, z));
        }
    }
    let (mut indices, mut tags) = (Vec::new(), Vec::new());
    for (row, tag) in [(0, 1), (4, 0)] {
        for i in 0..3 {
            let (a, b) = (row + i, row + i + 1);
            indices.push((a, b, b + 4));
            indices.push((a, b + 4, a + 4));
            tags.extend([tag, tag]);
        }
    }
    let mesh = LevelLineMesh::from_triangles(&positions, &indices, Some(&tags)).expect("a mesh");
    let chain_points = positions[..4].to_vec();
    (
        Rc::new(RefCell::new(mesh)),
        vec![(0, 1), (1, 2), (2, 3)],
        chain_points,
    )
}

/// The chain's field at one `setback` all along it.
fn field(points: &[Vector3], setback: f64) -> Rc<ChainField> {
    Rc::new(
        ChainField::from_polyline_bounded(
            points,
            false,
            &vec![setback; points.len()],
            30.0,
            (false, false),
            (None, None),
        )
        .expect("a field"),
    )
}

/// The side's own field cuts at 2, exactly on the crease; the chain's own (the larger side) at 3,
/// past it. Read by the side group's field the crease is a rim the band ends on; the untagged
/// triangle is never asked about, and with the chain's own field for the side (the level line,
/// one field for both sides) the band stands over the crease and there is no rim.
#[test]
fn a_rim_at_the_smaller_sides_distance_is_read_by_that_sides_own_field() {
    let (mesh, chain_edges, points) = a_side_ending_on_a_crease();
    let own = field(&points, 3.0);
    let side = field(&points, 2.0);
    let asked = RefCell::new(Vec::new());
    let read = |tag: i32| {
        asked.borrow_mut().push(tag);
        if tag == 1 {
            Rc::clone(&side)
        } else {
            Rc::clone(&own)
        }
    };
    let rims = rims_the_band_ends_on(&mesh, &chain_edges, &own, &read, 0.0, 0.1, 1.0, 1e-3)
        .expect("no budget");
    let rims = rims.expect("the side's band ends on the crease");
    assert!(!rims.is_empty());
    for (a, b) in &rims {
        assert!(
            (a.y - 2.0).abs() <= 1e-12 && (b.y - 2.0).abs() <= 1e-12,
            "{a:?} {b:?}"
        );
    }
    assert!(!asked.borrow().is_empty());
    assert!(
        asked.borrow().iter().all(|&tag| tag == 1),
        "only the band group is asked: {:?}",
        asked.borrow()
    );

    let one_field = |_: i32| Rc::clone(&own);
    let level_line =
        rims_the_band_ends_on(&mesh, &chain_edges, &own, &one_field, 0.0, 0.1, 1.0, 1e-3)
            .expect("no budget");
    assert_eq!(
        level_line, None,
        "the larger side's band stands over the crease"
    );
}
