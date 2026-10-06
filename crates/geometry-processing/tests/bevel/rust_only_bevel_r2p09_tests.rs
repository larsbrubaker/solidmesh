//! Rust-only checks of the R2.P09 package's small readings: the fill's closure check on a square,
//! the pick's seams taken out of a feature test, and closed chains turned to start at a corner.

use std::rc::Rc;

use geometry_processing::bevel::level_line::seam_edges::{FeatureEdgeTest, SeamEdges};
use geometry_processing::bevel::level_line::{band_gap, fill_closure, LevelLineMesh};
use vector_math::Vector3;

fn v(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}

/// A unit square as two triangles sharing the diagonal 0-2: (0, 1, 2) and (0, 2, 3).
fn square() -> LevelLineMesh {
    LevelLineMesh::from_triangles(
        &[
            v(0.0, 0.0, 0.0),
            v(1.0, 0.0, 0.0),
            v(1.0, 1.0, 0.0),
            v(0.0, 1.0, 0.0),
        ],
        &[(0, 1, 2), (0, 2, 3)],
        None,
    )
    .expect("a square builds")
}

/// A fill of the second triangle against a kept first one: the diagonal is answered by the kept
/// triangle running it the other way, and the first edge nothing answers is handed back, in the
/// order the fill laid its edges. A fill of both answers nothing on the square's rim.
#[test]
fn rust_only_the_fill_names_its_first_open_edge() {
    let mesh = square();

    // The second triangle runs 0 -> 2; the fill laid it once, so nothing else is kept on it.
    assert!(fill_closure::stands_on_kept(&mesh, (0, 2), 0));
    assert!(!fill_closure::stands_on_kept(&mesh, (0, 2), 1));

    assert_eq!(
        fill_closure::first_open_edge_of(&mesh, &[(2, 0)], &[1]),
        (2, 3)
    );
    assert_eq!(
        fill_closure::first_open_edge_of(&mesh, &[], &[0, 1]),
        (0, 1)
    );
}

/// A ribbon microns beside a one-edge chain is the chain's seam and stops reading as a feature;
/// the chain's own edge, and an edge sitting at ONE of its ends, stay features. With no epsilon the
/// caller's own test comes back unwrapped.
#[test]
fn rust_only_a_ribbon_beside_the_chain_is_its_seam() {
    let positions = [
        v(0.0, 0.0, 0.0),
        v(10.0, 0.0, 0.0),
        v(0.0, 1e-4, 0.0),
        v(10.0, 1e-4, 0.0),
    ];
    let chains = [vec![0, 1]];
    let every_edge: FeatureEdgeTest = Rc::new(|_, _| true);

    let unwrapped = SeamEdges::not_the_chains_seam(
        Some(&positions),
        Some(&chains),
        None,
        Some(every_edge.clone()),
        0.0,
    )
    .expect("the caller's test");
    assert!(Rc::ptr_eq(&unwrapped, &every_edge));

    let wrapped = SeamEdges::not_the_chains_seam(
        Some(&positions),
        Some(&chains),
        None,
        Some(every_edge),
        1e-3,
    )
    .expect("a wrapped test");
    assert!(!wrapped(2, 3), "the ribbon is the chain's seam");
    assert!(wrapped(0, 1), "the chain's own edge");
    assert!(wrapped(0, 2), "a cluster at one end is not a seam");

    assert!(SeamEdges::not_the_chains_seam(None, Some(&chains), None, None, 1e-3).is_none());
}

/// Where no chain is closed, or nothing says which are, the chains come back as they were; a
/// closed loop holding a corner of the pick - where an open chain ends on it and three chain edges
/// meet - is turned to start there, its direction and the open chain untouched.
#[test]
fn rust_only_open_chains_keep_their_start() {
    let loop_and_spur = vec![vec![0, 1, 2, 3], vec![2, 4]];
    assert_eq!(
        band_gap::start_closed_chains_at_a_corner(&loop_and_spur, Some(&[true, false])),
        vec![vec![2, 3, 0, 1], vec![2, 4]]
    );

    let chains = vec![vec![3, 4, 5], vec![5, 6]];
    assert_eq!(
        band_gap::start_closed_chains_at_a_corner(&chains, None),
        chains
    );
    assert_eq!(
        band_gap::start_closed_chains_at_a_corner(&chains, Some(&[false, false])),
        chains
    );
}
