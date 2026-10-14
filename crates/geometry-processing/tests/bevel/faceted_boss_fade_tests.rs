//! A pick covering part of a faceted boss's foot crease fades past its ends (Lars's decision): the
//! next facet folds only 15 degrees off the picked one, so it is the wall carrying on, not an
//! oblique end face - even where the operation's bend threshold (10 degrees) is tighter than the
//! facet step and so does not call the foot a carrying crease. The Standard test
//! TheOverlapAtASharedVertexIsNotWarnedAboutEvenWhenFacesBoundIt is this boss; with rays at its
//! ends the room refused at about 0.401.

use std::f64::consts::PI;

use geometry_processing::bevel::level_line::end_face_sense;
use vector_math::Vector3;

const SIDES: i32 = 24;
const POST_RADIUS: f64 = 6.0;
const POST_HEIGHT: f64 = 3.0;
const SLAB_RADIUS: f64 = 12.0;
const BEND_DEGREES: f64 = 10.0;

#[test]
fn a_partial_pick_of_a24_facet_boss_foot_fades_at_both_ends() {
    let (positions, indices) = boss_on_slab_top();
    // Foot ring vertices are 0 .. Sides - 1; two foot edges, well short of the full circle.
    let chain = [0, 1, 2];
    let rays = end_face_sense::oblique_ends(&positions, &indices, &chain, false, BEND_DEGREES);
    assert_eq!(rays, (false, false));
}

/// The slab's top as an annulus round the boss, the boss's faceted wall and its fanned cap: every
/// triangle that meets the foot ring, wound outward.
fn boss_on_slab_top() -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    let mut positions = Vec::new();
    for ring in 0..3 {
        for i in 0..SIDES {
            let angle = i as f64 * 2.0 * PI / SIDES as f64;
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
    let foot = |i: i32| i % SIDES;
    let top = |i: i32| SIDES + (i % SIDES);
    let outer = |i: i32| (2 * SIDES) + (i % SIDES);
    let mut indices = Vec::new();
    for i in 0..SIDES {
        indices.push((outer(i), outer(i + 1), foot(i)));
        indices.push((foot(i), outer(i + 1), foot(i + 1)));
        indices.push((foot(i), foot(i + 1), top(i)));
        indices.push((foot(i + 1), top(i + 1), top(i)));
        indices.push((centre, top(i), top(i + 1)));
    }

    (positions, indices)
}
