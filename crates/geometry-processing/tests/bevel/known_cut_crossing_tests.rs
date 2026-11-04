//! Which edges and triangles leave the band across a square crease end's KNOWN CUT rather than
//! across the level line (`ChainField::leaves_across_its_known_cut`): the inside end must stand
//! on one plane of the chain and the outside end past that SAME plane, with that chain's own
//! value what makes the inside end inside. A chain from (0, 0, 0) to (2, 0, 0), setback 1,
//! clipped square at both ends: P0 the plane x = 0 faced -x, P1 the plane x = 2 faced +x.

use geometry_processing::bevel::level_line::{ChainField, EndRayBound};
use vector_math::Vector3;

fn square_end(normal: Vector3) -> EndRayBound {
    EndRayBound {
        plane_normal: normal,
        same_sense: true,
        crease_room: 5.0,
        ..EndRayBound::default()
    }
}

fn chain_clipped_at_both_ends() -> ChainField {
    ChainField::from_polyline_bounded(
        &[Vector3::new(0.0, 0.0, 0.0), Vector3::new(2.0, 0.0, 0.0)],
        false,
        &[1.0, 1.0],
        f64::INFINITY,
        (false, false),
        (
            Some(square_end(Vector3::new(-1.0, 0.0, 0.0))),
            Some(square_end(Vector3::new(1.0, 0.0, 0.0))),
        ),
    )
    .expect("a two-point chain")
}

/// An edge from a point on P0 inside the band to one past P1: it leaves the band across the level
/// line before it reaches P1, so it is NOT a known-cut crossing, though its ends stand on one
/// plane and past another.
#[test]
fn an_edge_on_one_end_plane_and_past_the_other_crosses_the_line() {
    let chain = chain_clipped_at_both_ends();
    let inside = Vector3::new(0.0, 0.5, 0.0);
    let outside = Vector3::new(3.0, 0.5, 0.0);
    assert!(!chain
        .leaves_across_its_known_cut(&[inside], &[outside])
        .expect("the reading"));
}

/// An edge from a point on P0 inside the band to one past P0: it leaves across the known cut.
#[test]
fn an_edge_from_an_end_plane_to_past_it_leaves_across_the_cut() {
    let chain = chain_clipped_at_both_ends();
    let inside = Vector3::new(0.0, 0.5, 0.0);
    let outside = Vector3::new(-1.0, 0.5, 0.0);
    assert!(chain
        .leaves_across_its_known_cut(&[inside], &[outside])
        .expect("the reading"));
}

/// In a pick, another member's band can reach this chain's end plane: a point on P0 three away
/// from the chain is inside only by that member's value, so an edge from it to past P0 is not
/// this chain's known cut.
#[test]
fn a_point_on_the_plane_outside_the_chains_own_band_is_not_its_cut() {
    let chain = chain_clipped_at_both_ends();
    let inside = Vector3::new(0.0, 3.0, 0.0);
    let outside = Vector3::new(-1.0, 3.0, 0.0);
    assert!(!chain
        .leaves_across_its_known_cut(&[inside], &[outside])
        .expect("the reading"));
}
