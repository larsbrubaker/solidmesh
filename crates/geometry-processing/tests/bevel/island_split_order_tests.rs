//! The island rule's split vertex must not depend on how hard its search prunes - a sharper sound
//! bound retires cells sooner and so changes which sample the search happens to hold as its best,
//! and a vertex taken from that sample moves by up to the tolerance, enough to flip a room at its
//! limit into a pinch.

use solidmesh::bevel::level_line::bounded_maximum;
use solidmesh::bevel::level_line::{FieldSample, WorkError};
use solidmesh_math::{ieee_math, Vector3};

const SETBACK: f64 = 1.0;

/// Three chain points round a triangle; the field min_i |p - Ci| - setback is 1-Lipschitz, a union
/// of balls as the real field is, and has an interior peak at their circumcentre - an island.
const CHAIN_POINTS: [Vector3; 3] = [
    Vector3::new(0.1, 0.3, 0.0),
    Vector3::new(10.7, 1.9, 0.0),
    Vector3::new(4.3, 9.1, 0.0),
];

fn evaluate(point: Vector3) -> Result<FieldSample, WorkError> {
    let mut best = 0;
    let mut best_distance = f64::INFINITY;
    for (i, c) in CHAIN_POINTS.iter().enumerate() {
        let distance = (point - *c).length();
        if distance < best_distance {
            best_distance = distance;
            best = i;
        }
    }

    Ok(FieldSample {
        distance: best_distance,
        value: best_distance - SETBACK,
        nearest: CHAIN_POINTS[best],
        chain: 0,
        ..FieldSample::default()
    })
}

/// Sound: every point's least distance is at most its distance to ANY chain point, and over the
/// cell that is at most the farthest corner's - the union-of-balls argument ConvexCellBound makes
/// from the corners' own nearest points, here taken over every chain point, so it retires cells
/// sooner. (A Rust cell bound answers a `Result`, since a chain's own charges the budget.)
fn every_chain_point_bound(
    _s0: &FieldSample,
    _s1: &FieldSample,
    _s2: &FieldSample,
    v0: Vector3,
    v1: Vector3,
    v2: Vector3,
) -> Result<f64, WorkError> {
    let mut bound = f64::INFINITY;
    for c in CHAIN_POINTS {
        let farthest = ieee_math::max_f64(
            (v0 - c).length(),
            ieee_math::max_f64((v1 - c).length(), (v2 - c).length()),
        );
        bound = ieee_math::min_f64(bound, farthest - SETBACK);
    }

    Ok(bound)
}

#[test]
fn split_vertex_is_the_same_with_and_without_a_sharper_bound() {
    // Found by a sweep of random triangles: the two searches' best samples stand 0.0077 apart here.
    let a = Vector3::new(0.7596, 0.711, 0.0);
    let b = Vector3::new(9.0364, 2.5831, 0.0);
    let c = Vector3::new(2.5139, 6.9537, 0.0);
    let tolerance = 0.01;
    let snap = 2.54;
    let plain = bounded_maximum::island_peak(&evaluate, a, b, c, tolerance, snap, None, None)
        .expect("no budget in force");
    let pruned = bounded_maximum::island_peak(
        &evaluate,
        a,
        b,
        c,
        tolerance,
        snap,
        Some(&every_chain_point_bound),
        None,
    )
    .expect("no budget in force");
    // Both find the island...
    assert!(plain.value > snap);
    assert!(pruned.value > snap);
    // ...and put its vertex in the same place.
    assert!((plain.point - pruned.point).length() <= 1e-12);
    assert_eq!(plain.value, pruned.value);
}
