//! A pick covering part of a faceted boss's foot crease fades past its ends (Lars's decision): the
//! next facet folds only 15 degrees off the picked one, so it is the wall carrying on, not an
//! oblique end face - even where the operation's bend threshold (10 degrees) is tighter than the
//! facet step and so does not call the foot a carrying crease. The Standard test
//! TheOverlapAtASharedVertexIsNotWarnedAboutEvenWhenFacesBoundIt is this boss; with rays at its
//! ends the room refused at about 0.401. A 12-gon's facets fold exactly 30 degrees, the limit
//! `end_face_sense::carries_on` reads as one wall: it fades too, and with stopped ends on it stops.

use std::f64::consts::PI;
use std::rc::Rc;

use solidmesh::bevel::level_line::{
    end_face_sense, stopped_end_clips, turn_stopped_ends_on, LevelLineWork, WorkBudget,
};
use solidmesh_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;

const SIDES: i32 = 24;
const POST_RADIUS: f64 = 6.0;
const POST_HEIGHT: f64 = 3.0;
const SLAB_RADIUS: f64 = 12.0;
const BEND_DEGREES: f64 = 10.0;

#[test]
fn a_partial_pick_of_a24_facet_boss_foot_fades_at_both_ends() {
    let (positions, indices) = boss_on_slab_top(SIDES);
    // Foot ring vertices are 0 .. Sides - 1; two foot edges, well short of the full circle.
    let chain = [0, 1, 2];
    let rays = end_face_sense::oblique_ends(&positions, &indices, &chain, false, BEND_DEGREES);
    assert_eq!(rays, (false, false));
}

/// A 12-gon boss's next facet folds exactly 30 degrees off the picked one, which
/// `end_face_sense::carries_on` reads as the wall carrying on whichever way the coordinates round:
/// no oblique end face, so the pick fades.
#[test]
fn a_partial_pick_of_a12_facet_boss_foot_fades_at_both_ends() {
    let (positions, indices) = boss_on_slab_top(12);
    let chain = [0, 1, 2];
    let rays = end_face_sense::oblique_ends(&positions, &indices, &chain, false, BEND_DEGREES);
    assert_eq!(rays, (false, false));
}

/// With stopped ends on, the same 12-gon pick stops at both its ends: the foot crease leaving each
/// end turns exactly 30 degrees off the chain, which carries the chain's line on.
#[test]
fn a_partial_pick_of_a12_facet_boss_foot_stops_at_both_ends() {
    let (positions, indices) = boss_on_slab_top(12);
    let chain = [0, 1, 2];
    let budget = Rc::new(
        WorkBudget::new(WorkBudget::default_limit(indices.len() as i32)).expect("a positive limit"),
    );
    let _work = LevelLineWork::begin(Some(budget));
    let _stopped = turn_stopped_ends_on();
    let features = feature_edges_over(&positions, &indices, BEND_DEGREES);
    let (first, last) = stopped_end_clips(
        &positions,
        &indices,
        &chain,
        false,
        (false, false),
        &|a, b| features(a, b),
    );
    assert!(first.is_some() && last.is_some(), "{first:?} {last:?}");
}

/// The slab's top as an annulus round a `sides`-gon boss, the boss's faceted wall and its fanned
/// cap: every triangle that meets the foot ring, wound outward.
fn boss_on_slab_top(sides: i32) -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    let mut positions = Vec::new();
    for ring in 0..3 {
        for i in 0..sides {
            let angle = i as f64 * 2.0 * PI / sides as f64;
            let radius = if ring == 2 { SLAB_RADIUS } else { POST_RADIUS };
            positions.push(Vector3::new(
                radius * angle.cos(),
                radius * angle.sin(),
                if ring == 1 { POST_HEIGHT } else { 0.0 },
            ));
        }
    }

    positions.push(Vector3::new(0.0, 0.0, POST_HEIGHT));
    let centre = positions.len() as i32 - 1;
    let foot = |i: i32| i % sides;
    let top = |i: i32| sides + (i % sides);
    let outer = |i: i32| (2 * sides) + (i % sides);
    let mut indices = Vec::new();
    for i in 0..sides {
        indices.push((outer(i), outer(i + 1), foot(i)));
        indices.push((foot(i), outer(i + 1), foot(i + 1)));
        indices.push((foot(i), foot(i + 1), top(i)));
        indices.push((foot(i + 1), top(i + 1), top(i)));
        indices.push((centre, top(i), top(i + 1)));
    }

    (positions, indices)
}

/// The one "curve carries on" test reads a turn of exactly 30 degrees the same whichever way its
/// cosine rounds, reads a degree past it as a corner, and never carries a broken direction on.
#[test]
fn thirty_degrees_carries_on_whichever_way_it_rounds() {
    let at_the_limit = 30.0_f64.to_radians().cos();
    for cosine in [
        at_the_limit,
        f64::from_bits(at_the_limit.to_bits() - 1),
        f64::from_bits(at_the_limit.to_bits() + 1),
        f64::from(at_the_limit as f32),
    ] {
        assert!(end_face_sense::carries_on(cosine), "{cosine}");
    }
    assert!(!end_face_sense::carries_on(31.0_f64.to_radians().cos()));
    assert!(!end_face_sense::carries_on(f64::NAN));
}
