//! A chain's unit setback where a face too thin for a normal lies along the picked crease
//! (`chain_on_the_mesh::per_edge_unit_setbacks`): the sliver reads the face across its longest
//! edge, as `FeatureEdge::try_classify` reads it, so a right-angle crease asks the setback of a right
//! angle and not of the fold the sliver's rounding-noise normal describes.

use solidmesh::bevel::level_line::{chain_on_the_mesh, BandProfile};
use solidmesh_math::Vector3;

/// A 1 x 1 x 10 box at x = 200 whose top front edge, (200, 0, 10) to (201, 0, 10), carries a
/// vertex `c` a micron into the top face, and the sliver (P0, P1, c) along that edge. The crease is
/// the path P0 - c - P1, a right angle between the top and the y = 0 side all the way.
fn box_with_a_sliver_on_its_top_front_edge() -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    let at = |x: f64, y: f64, z: f64| Vector3::new(x, y, z);
    let positions = vec![
        at(200.0, 0.0, 10.0),                    // P0
        at(201.0, 0.0, 10.0),                    // P1
        at(201.0, 1.0, 10.0),                    // P2
        at(200.0, 1.0, 10.0),                    // P3
        at(200.0, 0.0, 0.0),                     // B0
        at(201.0, 0.0, 0.0),                     // B1
        at(201.0, 1.0, 0.0),                     // B2
        at(200.0, 1.0, 0.0),                     // B3
        at(200.5, 9.536_743_164_062_5e-7, 10.0), // c, 2^-20 into the top
    ];
    let indices = vec![
        (0, 1, 8), // the sliver
        (8, 1, 2),
        (8, 2, 3),
        (8, 3, 0),
        (4, 6, 5),
        (4, 7, 6),
        (4, 5, 1),
        (4, 1, 0),
        (5, 6, 2),
        (5, 2, 1),
        (6, 7, 3),
        (6, 3, 2),
        (7, 4, 0),
        (7, 0, 3),
    ];
    (positions, indices)
}

#[test]
fn a_sliver_along_a_right_angle_crease_asks_a_right_angles_setback() {
    let (positions, indices) = box_with_a_sliver_on_its_top_front_edge();
    let setbacks = chain_on_the_mesh::per_edge_unit_setbacks(
        &positions,
        &indices,
        &[(0, 8), (8, 1)],
        BandProfile::Fillet,
    )
    .expect("both chain edges stand on two triangles");

    for setback in setbacks {
        assert!(
            (setback - 1.0).abs() < 1e-9,
            "a right angle's fillet sets back one radius on each face: {setback}"
        );
    }
}
