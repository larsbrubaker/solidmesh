//! A densely faceted round mouth - Many Holes' port mouths in proportion, a 64-gon rim whose
//! segments (~0.145) are much shorter than the fillet's rows are across (r 0.5). Rows carried along
//! a single short segment there fell out of step with their neighbours and the strip crossed
//! itself; this guards that the round's fill stays closed and free of self-intersection. The shared
//! crossing-pair count lives in `round_mouth_fillet_crossings.rs`.

use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::gap_fill;
use solidmesh::bevel::level_line::{BandProfile, WorkBudget};
use solidmesh_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_room_geometry::SolidWithChain;
use crate::level_line_test_geometry::is_watertight;
use crate::round_mouth_fillet_crossings::crossing_pairs;

const FOLD_DEGREES: f64 = 30.0;
const SIDES: i32 = 64;
const OUTLINE_SIDES: i32 = 8;
const RADIUS: f64 = 0.5;
const SEGMENTS: i32 = 8;

#[test]
fn a_densely_faceted_mouth_fillet_is_closed_and_does_not_cross_itself() {
    let hole_radius = 0.145 * SIDES as f64 / (2.0 * std::f64::consts::PI);
    let (positions, indices, mouth) = tube(hole_radius, hole_radius + 3.0, 2.0);
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
    let gap = band_gap_entry_points::build_pick(
        &positions,
        &indices,
        &[mouth],
        &[true],
        Some(feature),
        RADIUS,
        BandProfile::Fillet,
        &BandGapBuildOptions {
            budget: Some(Rc::new(
                WorkBudget::new(WorkBudget::default_limit(indices.len() as i32))
                    .expect("a positive limit"),
            )),
            sharp_turn_degrees: FOLD_DEGREES,
            fillet_segments: SEGMENTS,
            ..BandGapBuildOptions::default()
        },
    )
    .expect("a valid pick");
    assert!(gap.built(), "band {:?}", gap.refusal);

    let fill = gap_fill::fill(&gap, BandProfile::Fillet, SEGMENTS).expect("the fill runs");
    assert!(fill.filled(), "fill {:?}", fill.refusal);

    let mesh = fill.mesh.as_ref().expect("a filled mesh").borrow();
    assert!(is_watertight(&mesh));
    assert_eq!(crossing_pairs(&mesh), 0);
}

/// A thick-walled tube on z = 0 .. `height`: a 64-gon bore inside an 8-gon outline, the top and
/// bottom annuli zipped between them. The bore's top rim, counter-clockwise from +z, is the pick.
fn tube(inner: f64, outer: f64, height: f64) -> SolidWithChain {
    let mut positions: Vec<Vector3> = Vec::new();
    let mut ring = |sides: i32, radius: f64, z: f64| -> i32 {
        let first = positions.len() as i32;
        for i in 0..sides {
            // The outline is turned off the bore's phase so no fan edge lines up with a bore vertex.
            let a = (2.0 * std::f64::consts::PI * i as f64 / sides as f64)
                + if sides == SIDES { 0.0 } else { 0.1 };
            positions.push(Vector3::new(radius * a.cos(), radius * a.sin(), z));
        }

        first
    };

    let top_inner = ring(SIDES, inner, height);
    let top_outer = ring(OUTLINE_SIDES, outer, height);
    let low_inner = ring(SIDES, inner, 0.0);
    let low_outer = ring(OUTLINE_SIDES, outer, 0.0);
    let mut indices: Vec<(i32, i32, i32)> = Vec::new();
    let quad = |indices: &mut Vec<(i32, i32, i32)>, a: i32, b: i32, c: i32, d: i32| {
        indices.push((a, b, c));
        indices.push((a, c, d));
    };

    for i in 0..SIDES {
        // The bore, facing in.
        let j = (i + 1) % SIDES;
        quad(
            &mut indices,
            low_inner + i,
            top_inner + i,
            top_inner + j,
            low_inner + j,
        );
    }

    for i in 0..OUTLINE_SIDES {
        // The outer wall, facing out.
        let j = (i + 1) % OUTLINE_SIDES;
        quad(
            &mut indices,
            low_outer + i,
            low_outer + j,
            top_outer + j,
            top_outer + i,
        );
    }

    // The top and bottom annuli zip the bore's 64 vertices to the outline's few by angle, so their
    // triangles are long fans - the rail stations on them fall unevenly against the bore's, as an
    // exported plate's do. Both rings start near angle 0 and wind counter-clockwise, so the k-th
    // vertex's angle is its index share.
    let inner_angle = |k: i32| 2.0 * std::f64::consts::PI * k as f64 / SIDES as f64;
    let outer_angle = |k: i32| (2.0 * std::f64::consts::PI * k as f64 / OUTLINE_SIDES as f64) + 0.1;
    let (mut a, mut b) = (0, 0);
    while a < SIDES || b < OUTLINE_SIDES {
        let advance_inner =
            b >= OUTLINE_SIDES || (a < SIDES && inner_angle(a + 1) <= outer_angle(b + 1));
        let (ti, to) = (top_inner + (a % SIDES), top_outer + (b % OUTLINE_SIDES));
        let (li, lo) = (low_inner + (a % SIDES), low_outer + (b % OUTLINE_SIDES));
        if advance_inner {
            let (tn, ln) = (top_inner + ((a + 1) % SIDES), low_inner + ((a + 1) % SIDES));
            indices.push((ti, to, tn));
            indices.push((li, ln, lo));
            a += 1;
        } else {
            let (tn, ln) = (
                top_outer + ((b + 1) % OUTLINE_SIDES),
                low_outer + ((b + 1) % OUTLINE_SIDES),
            );
            indices.push((ti, to, tn));
            indices.push((li, ln, lo));
            b += 1;
        }
    }

    (positions, indices, (top_inner..top_inner + SIDES).collect())
}
