//! A bead whose wall runs out onto the cube's own convex edge is admitted; one wider than its wall
//! everywhere is still refused.

use geometry_processing::bevel::bevel_working_mesh::signed_volume;
use geometry_processing::topology::{EdgeConvexity, FeatureEdgeSet};
use polygon_mesh::Mesh;
use vector_math::Vector3;

const CUBE_SIZE: f64 = 20.0;
const RIDGE_AT: f64 = CUBE_SIZE / 2.0;
const MOUTH_INSET: f64 = 0.032;

fn apex() -> Vector3 {
    Vector3::new(
        5.0 + (MOUTH_INSET / 4.0),
        RIDGE_AT + (MOUTH_INSET / 2.0),
        RIDGE_AT + (MOUTH_INSET / 2.0),
    )
}

fn rim() -> Vector3 {
    Vector3::new(MOUTH_INSET / 2.0, MOUTH_INSET, CUBE_SIZE)
}

fn pyramidal_recess_in_a_cube() -> Mesh {
    let s = CUBE_SIZE;
    let v = Vector3::new;
    let (c000, c100, c010, c110) = (
        v(0.0, 0.0, 0.0),
        v(s, 0.0, 0.0),
        v(0.0, s, 0.0),
        v(s, s, 0.0),
    );
    let (c001, c101, c011, c111) = (v(0.0, 0.0, s), v(s, 0.0, s), v(0.0, s, s), v(s, s, s));
    let rim = rim();
    let far_rim = v(s - (MOUTH_INSET / 2.0), MOUTH_INSET, s);
    let apex = apex();
    let far = v(s - apex.x, apex.y, apex.z);

    // In a fixed order, so every face keeps the index the tests read it by.
    let mut mesh = Mesh::new();
    quad(&mut mesh, c010, c011, c111, c110);
    quad(&mut mesh, c000, c001, c011, c010);
    quad(&mut mesh, c100, c110, c111, c101);
    quad(&mut mesh, c000, c010, c110, c100);
    mesh.create_face(&[c001, rim, c011]);
    mesh.create_face(&[rim, far_rim, c011]);
    mesh.create_face(&[far_rim, c111, c011]);
    mesh.create_face(&[far_rim, c101, c111]);
    quad(&mut mesh, c000, c100, far, apex);
    quad(&mut mesh, rim, apex, far, far_rim);
    quad(&mut mesh, c000, apex, rim, c001);
    quad(&mut mesh, c100, c101, far_rim, far);
    mesh.clean_and_merge();
    mesh
}

fn quad(mesh: &mut Mesh, a: Vector3, b: Vector3, c: Vector3, d: Vector3) {
    mesh.create_face(&[a, b, c]);
    mesh.create_face(&[a, c, d]);
}

/// The fixture is a cube with a concave crease from its rim to the recess apex.
#[test]
fn the_fixture_is_a_cube_with_a_concave_crease_from_its_rim_to_the_recess_apex() {
    let part = pyramidal_recess_in_a_cube();
    assert!(part.is_manifold());
    assert!(
        (signed_volume(&part) - 6328.549).abs() <= 0.05,
        "{}",
        signed_volume(&part)
    );
    let features = FeatureEdgeSet::create(&part);
    let crease = features.edges()[edge_index(&features, rim(), apex())];
    assert_eq!(crease.convexity, EdgeConvexity::Concave);
}

fn edge_index(features: &FeatureEdgeSet, from: Vector3, to: Vector3) -> usize {
    features
        .edges()
        .iter()
        .position(|edge| {
            ((edge.start - from).length() < 1e-4 && (edge.end - to).length() < 1e-4)
                || ((edge.start - to).length() < 1e-4 && (edge.end - from).length() < 1e-4)
        })
        .unwrap_or_else(|| panic!("the fixture has no feature edge from {from:?} to {to:?}"))
}
