//! Flow rung 2's sense test, `EndFaceSense.IsOppositeSenseCrease`: a side's crease folding the
//! other way to the chain grows under the blend; folding the same way it is trimmed; a knife stays
//! the knife rule's. The chain is convex throughout: the unit cube's edge along x at y = 0, z = 1,
//! between the top and the front.

use solidmesh::bevel::level_line::end_face_sense;
use solidmesh_math::Vector3;

#[test]
fn a_ribs_concave_crease_on_a_convex_chain_is_opposite_sense() {
    // The top meets a rib flank rising away from the chain along y = 0.5: concave.
    assert!(read(
        Vector3::new(0.0, 0.5, 1.0),
        Vector3::new(1.0, 0.5, 1.0),
        Vector3::new(0.5, 0.2, 1.0),
        Vector3::new(0.5, 0.75, 1.5)
    ));
}

#[test]
fn a_cubes_far_edge_is_the_same_sense() {
    // The top meets the back face y = 1 going down: convex, like the chain.
    assert!(!read(
        Vector3::new(0.0, 1.0, 1.0),
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(0.5, 0.5, 1.0),
        Vector3::new(0.5, 1.0, 0.0)
    ));
}

#[test]
fn a_knife_stays_the_knife_rules() {
    // The top folds back under itself past a right angle along y = 1: a knife, which the knife rule
    // owns.
    let k0 = Vector3::new(0.0, 1.0, 1.0);
    let k1 = Vector3::new(1.0, 1.0, 1.0);
    assert!(!read(
        k0,
        k1,
        Vector3::new(0.5, 0.5, 1.0),
        Vector3::new(0.5, 0.2, 0.6)
    ));
}

/// The chain's two triangles (top, front), then a crease k0-k1: a top triangle (k0, k1, inside) on
/// the chain's side of it, wound to face up, and the far triangle (k0, k1, beyond) across it, wound
/// to match.
fn read(k0: Vector3, k1: Vector3, inside: Vector3, beyond: Vector3) -> bool {
    let positions = vec![
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(1.0, 0.0, 1.0),
        Vector3::new(0.5, 0.5, 1.0),
        Vector3::new(0.5, 0.0, 0.0),
        k0,
        k1,
        inside,
        beyond,
    ];
    let up = (k1 - k0).cross(inside - k0).z > 0.0;
    let side = if up { (4, 5, 6) } else { (5, 4, 6) };
    let far = if up { (5, 4, 7) } else { (4, 5, 7) };
    end_face_sense::is_opposite_sense_crease(&positions, (0, 1, 2), (1, 0, 3), 0, side, far, 4)
}
