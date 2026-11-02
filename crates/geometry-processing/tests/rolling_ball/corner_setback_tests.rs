//! The rolling ball's setbacks where picked chains meet at a corner (`rolling_ball_setbacks`'s
//! header, "Corners where picked chains meet"): on flat faces whose sides agree the ball takes the
//! pass, and every setback, the corner's included, is the angle's (`unit_setbacks`), so the
//! corner's fill closes the round as it does with the switch off. Held to a 60 degree prism's top
//! rim (three chains, each corner's third face the chain's end face) and to a box's bottom edge
//! handed in as two collinear chains meeting at its middle vertex, a stand-in for a setback step's
//! pieces (no face of neither side round the corner, which is still no free end: nothing tapers to
//! it). A corner where an open chain's sides differ is
//! still refused, by the rule for a pass of several chains: held to the one-wall-bending wedge's
//! apex meeting a picked edge of its end face. The prism is `band_route_tests.rs`'s, the box and
//! the wedge `band_setback_tests.rs`'s.

use geometry_processing::bevel::level_line::chain_on_the_mesh;
use geometry_processing::bevel::level_line::rolling_ball_setbacks::{for_the_pass, SideSetbacks};
use geometry_processing::bevel::level_line::BandProfile;
use geometry_processing::bevel::rolling_ball::rolling_ball_routing::{
    last_route, reset_to_the_default, set_last_route, set_use_rolling_ball, ROLLING_BALL_REFUSED,
    ROLLING_BALL_ROUTE,
};
use vector_math::Vector3;

use crate::band_route_tests::{a_sixty_degree_prism, creases, Triangles};
use crate::band_setback_tests::{a_wedge_with_one_wall_bending_steeper, extruded, the_band};

/// How far the ball's setback per unit of size may stand from the angle's on flat faces: a corner
/// end's station is held in its end face's plane to 1e-9 of the radius, and every solve to 1e-11.
const AGREE: f64 = 1e-9;

/// The ball's setbacks for the open `chains` of `part` at `size`, and the route the pass wrote.
fn the_pass(
    part: &(Vec<Vector3>, Triangles),
    chains: &[Vec<i32>],
    size: f64,
) -> (Option<Vec<SideSetbacks>>, Option<String>) {
    let (positions, triangles) = part;
    let closed = vec![false; chains.len()];
    let crease = creases(positions, triangles);
    set_use_rolling_ball(true);
    set_last_route(None);
    let setbacks = for_the_pass(
        positions,
        triangles,
        chains,
        &closed,
        Some(&*crease),
        None,
        size,
        BandProfile::Fillet,
        &the_band(positions, triangles, chains, &closed),
        false,
    );
    let route = last_route();
    reset_to_the_default();
    (setbacks, route)
}

/// The ball's setbacks for the open `chains` of `part` at size 0.2, after checking the pass took
/// the rolling ball's route.
fn the_balls(part: &(Vec<Vector3>, Triangles), chains: &[Vec<i32>]) -> Vec<SideSetbacks> {
    let (setbacks, route) = the_pass(part, chains, 0.2);
    assert_eq!(route.as_deref(), Some(ROLLING_BALL_ROUTE));
    setbacks.expect("the ball takes a corner whose sides agree")
}

/// Every chain's ball setbacks, both sides, equal the angle's at every chain point within
/// [`AGREE`], and no end carries a cap of its own.
fn assert_the_angles(part: &(Vec<Vector3>, Triangles), chains: &[Vec<i32>]) {
    let (positions, triangles) = part;
    for (chain, ball) in chains.iter().zip(the_balls(part, chains)) {
        let edges = chain_on_the_mesh::chain_edges(chain, false);
        let angle = chain_on_the_mesh::unit_setbacks(
            positions,
            triangles,
            chain,
            false,
            &edges,
            BandProfile::Fillet,
        )
        .expect("the angle's setbacks");
        assert_eq!(ball.caps, [None, None], "{chain:?}");
        for side in [&ball.left, &ball.right] {
            assert_eq!(side.len(), angle.len(), "{chain:?}");
            for (got, want) in side.iter().zip(&angle) {
                assert!((got - want).abs() <= AGREE, "{chain:?}: {got} for {want}");
            }
        }
    }
}

/// A 60 degree prism's top rim picked as three open chains meeting at its corners: at a corner
/// the third face is the chain's end face, so the ball there touches both sides a radius out
/// (the rim is square: 1 per unit of size), as the angle says.
#[test]
fn chains_meeting_at_a_corner_take_the_balls_setbacks_which_are_the_angles() {
    let part = a_sixty_degree_prism();
    assert_the_angles(&part, &[vec![3, 4], vec![4, 5], vec![5, 3]]);
}

/// A 2 by 1 box's bottom front edge with a vertex at its middle, handed to the pass as two chains
/// meeting there. No pick makes these chains (`split_at_setback_steps` does not produce them, and
/// a user picking the two halves is refused `ChainsTouch`); they stand in for a setback
/// step's pieces, where `corners_of` names the meeting vertex a corner and no face of neither side
/// stands round it, to pin the rule that such a corner end is no free end. The walk ends there as
/// a plain last station, but the round runs on into the other chain's, so the setback stays the
/// ball's (1 per unit) and is not tapered to nothing at the middle vertex.
#[test]
fn chains_meeting_in_a_setback_step_keep_the_balls_setback_to_the_corner() {
    let profile = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)];
    let part = extruded(&profile, 2.0, Some((1.0, 0.5)));
    assert_the_angles(&part, &[vec![0, 1], vec![1, 2]]);
}

/// The one-wall-bending wedge's apex (vertices 0 and 4, its sides' contacts 2.34 and 2.72 from it
/// at radius 2) picked with its end face's edge against the straight wall (vertices 0 and 3),
/// meeting at vertex 0: a corner whose contacts differ needs the corner's own rows about the ball,
/// so the pass is refused, naming the apex's first point.
#[test]
fn a_corner_whose_sides_differ_is_still_refused_by_the_ball() {
    let part = a_wedge_with_one_wall_bending_steeper();
    let (setbacks, route) = the_pass(&part, &[vec![0, 4], vec![0, 3]], 2.0);
    assert_eq!(setbacks, None);
    let route = route.expect("the pass writes its route");
    let why = route
        .strip_prefix(ROLLING_BALL_REFUSED)
        .expect("the route says the ball refused");
    assert!(
        why.starts_with("chain 0 point 0: the sides' contacts stand ")
            && why.ends_with(
                "from the chain, and an open chain's end regions in a pass of several chains are \
                 not cut per side yet"
            ),
        "{why}"
    );
}
