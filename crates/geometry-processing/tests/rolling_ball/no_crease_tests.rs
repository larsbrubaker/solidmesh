//! A pass of one chain with no crease under it, the ball asked to take it: refused as
//! `ChainHasNoCreaseToBevel` before the ball walks (`level_line::band_gap_build_entry`). A chain
//! drawn across a flat face has nothing to round, so the user is told that, not that the ball
//! could not roll there, and not that the size is too big.

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{
    build_pick, BandGapBuildOptions,
};
use geometry_processing::bevel::level_line::seam_edges::FeatureEdgeTest;
use geometry_processing::bevel::level_line::{BandGapRefusal, BandGapResult, BandProfile};
use geometry_processing::bevel::rolling_ball::rolling_ball_routing::{
    last_route, reset_to_the_default, set_last_route, set_use_rolling_ball,
};
use vector_math::{Matrix4X4, Vector3};

use crate::band_route_tests::{a_box, Triangles};

/// The top face's diagonal (corners 4 to 6) of a box 2 by 2 by 2: both triangles beside it lie in
/// the top face, so it is no crease.
const ACROSS_THE_TOP: [i32; 2] = [4, 6];

/// The pass built with the switch `on`, nothing counted as a fold, and the route it wrote.
fn built_across(
    positions: &[Vector3],
    triangles: &Triangles,
    size: f64,
    on: bool,
) -> (BandGapResult, Option<String>) {
    let none: FeatureEdgeTest = Rc::new(|_, _| false);
    set_use_rolling_ball(on);
    set_last_route(None);
    let result = build_pick(
        positions,
        triangles,
        &[ACROSS_THE_TOP.to_vec()],
        &[false],
        Some(none),
        size,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
    )
    .expect("the pick's arguments are good");
    let route = last_route();
    reset_to_the_default();
    (result, route)
}

/// With the ball asked, at a size well inside the face and at sizes far past the part, the pass is
/// refused by the name the level line gives it with the switch off, nothing cut, and the ball
/// never walked (the route records no refusal of the ball's).
#[test]
fn a_chain_across_a_flat_face_is_refused_by_name_before_the_ball() {
    let (positions, triangles) = a_box(Vector3::new(-1.0, -1.0, -1.0), Vector3::new(1.0, 1.0, 1.0));
    let (off, _) = built_across(&positions, &triangles, 0.2, false);
    assert_eq!(off.refusal, BandGapRefusal::ChainHasNoCreaseToBevel);

    for size in [0.2, 2.0, 50.0] {
        let (on, route) = built_across(&positions, &triangles, size, true);
        assert_eq!(
            on.refusal,
            BandGapRefusal::ChainHasNoCreaseToBevel,
            "at {size}: {route:?}"
        );
        assert!(on.mesh.is_none(), "at {size}: nothing is cut");
        assert_eq!(
            on.rolling_ball_refusal, None,
            "at {size}: no ball's refusal"
        );
        assert_eq!(route, None, "at {size}: the ball never walked");
    }
}

/// The same box turned off its axes, so the top face's two triangles carry normals equal only to
/// their rounding: still no crease, still the same name.
#[test]
fn a_turned_flat_face_is_still_no_crease() {
    let (positions, triangles) = a_box(Vector3::new(-1.0, -1.0, -1.0), Vector3::new(1.0, 1.0, 1.0));
    let motion = Matrix4X4::create_rotation(Vector3::new(0.3, -0.7, 0.2))
        * Matrix4X4::create_translation(5.0, -3.0, 11.0);
    let turned: Vec<Vector3> = positions.iter().map(|&p| p.transform(motion)).collect();

    let (on, route) = built_across(&turned, &triangles, 0.2, true);
    assert_eq!(
        on.refusal,
        BandGapRefusal::ChainHasNoCreaseToBevel,
        "{route:?}"
    );
}

/// A real crease (the box's top front edge, corners 4 to 5) is still the ball's to take: the check
/// stops only a pass with nothing to round.
#[test]
fn a_real_crease_still_goes_to_the_ball() {
    let (positions, triangles) = a_box(Vector3::new(-1.0, -1.0, -1.0), Vector3::new(1.0, 1.0, 1.0));
    set_use_rolling_ball(true);
    set_last_route(None);
    let result = build_pick(
        &positions,
        &triangles,
        &[vec![4, 5]],
        &[false],
        Some(Rc::new(|_, _| true)),
        0.2,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
    )
    .expect("the pick's arguments are good");
    let route = last_route();
    reset_to_the_default();
    assert_ne!(result.refusal, BandGapRefusal::ChainHasNoCreaseToBevel);
    assert!(route.is_some(), "the ball was asked");
}
