//! Whether the band ends on a rim (`consumed_to_the_rim::rims_the_band_ends_on`): a band that
//! stands over a side's crease ends on no rim there.

use std::cell::RefCell;
use std::rc::Rc;

use solidmesh::bevel::level_line::consumed_to_the_rim::rims_the_band_ends_on;
use solidmesh::bevel::level_line::{ChainField, LevelLineMesh};
use solidmesh_math::Vector3;

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

/// The chain's field at a setback of 3 reaches past the side's crease at y = 2, so the band
/// stands over the crease and there is no rim.
#[test]
fn a_band_standing_over_the_sides_crease_ends_on_no_rim() {
    let (mesh, chain_edges, points) = a_side_ending_on_a_crease();
    let own = field(&points, 3.0);
    let rims =
        rims_the_band_ends_on(&mesh, &chain_edges, &own, 0.0, 0.1, 1.0, 1e-3).expect("no budget");
    assert_eq!(rims, None, "the band stands over the crease");
}
