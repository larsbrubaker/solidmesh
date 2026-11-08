//! The rail the fill zips by chain parameter must stay monotone on a chain whose nearest-point
//! reading barely moves - a finely tessellated convex arc and a shallow concave corner. On 77930
//! the cut left NINE pairs of vertices at identical positions, whose parameters - computed from two
//! different roots - differ in the last bits; on 65394 the rail curves round the far side of a
//! boss, where the nearest chain point barely moves and the parameter wobbles by a thousandth of
//! the setback. Read with an exact comparison each wobble is a second wrap, and the fill refused
//! `RailIsNotMonotone` on a loop that goes round the chain exactly once. The prism below reproduces
//! that shape of input synthetically. Parameter rows are loops over the same-named test.
//! `room_rigid_motion_tests.rs` reads the prism too (`rounded_dented_prism_solid`).

#![allow(dead_code)]

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::gap_fill;
use solidmesh::bevel::level_line::{BandProfile, GapFillRefusal, GapFillResult};
use solidmesh_math::{Vector2, Vector3};

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_test_geometry::{open_edges, signed_volume, signed_volume_of};

const FOLD_DEGREES: f64 = 20.0;

/// The prism's positions, triangles and top rim.
pub type PrismSolid = (Vec<Vector3>, Vec<(i32, i32, i32)>, Vec<i32>);

#[test]
fn a_rounded_rim_with_a_shallow_dent_fills_closed() {
    for size in [0.1, 0.2, 0.4, 0.8] {
        let fill = build(&rounded_dented_prism(), size);
        assert_eq!(fill.refusal, GapFillRefusal::None, "size {size}");
        let mesh = fill.mesh.as_ref().expect("a filled mesh").borrow();
        assert!(open_edges(&mesh).is_empty(), "size {size}");
    }
}

/// Nearby sizes take nearby amounts: the fill is continuous in the size, not jumping about.
#[test]
fn the_removed_volume_is_continuous_in_the_size() {
    let solid = rounded_dented_prism();

    let at = changed(&solid, 0.30);
    let just_over = changed(&solid, 0.31);

    assert!(at > 0.0);
    assert!(just_over > at);
    assert!(just_over - at < at * 0.2);
}

/// How much material the fill moved, as a magnitude: this rim is convex round its corners and
/// CONCAVE at the dent, so a fillet takes away at one and adds at the other and the net is not the
/// measure.
fn changed(solid: &PrismSolid, size: f64) -> f64 {
    let before = signed_volume_of(&solid.0, &solid.1);
    let fill = build(solid, size);
    let mesh = fill.mesh.as_ref().expect("a filled mesh").borrow();
    (before - signed_volume(&mesh)).abs()
}

fn build(solid: &PrismSolid, size: f64) -> GapFillResult {
    let gap = band_gap_entry_points::build(
        &solid.0,
        &solid.1,
        &solid.2,
        true,
        Some(feature_edges_over(&solid.0, &solid.1, FOLD_DEGREES)),
        size,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
    )
    .expect("a valid pick");

    assert!(
        gap.built(),
        "the dented rim must leave room at this size: {:?}",
        gap.refusal
    );

    gap_fill::fill(&gap, BandProfile::Fillet, 4).expect("the fill runs")
}

/// The same prism, for the rigid-motion requirement that reads it in another file.
pub fn rounded_dented_prism_solid() -> PrismSolid {
    rounded_dented_prism()
}

/// A prism on a rounded rectangle with one shallow inward dent in a long side, capped top and
/// bottom by a fan from the centre (the outline is star-shaped about it, so the fan is a valid
/// triangulation). Its top rim is the picked chain.
fn rounded_dented_prism() -> PrismSolid {
    let outline = dented_rounded_rectangle();
    let n = outline.len();
    let height = 4.0;

    let mut positions = Vec::new();
    let mut top = Vec::new();
    let mut bottom = Vec::new();
    for p in &outline {
        top.push(positions.len() as i32);
        positions.push(Vector3::new(p.x, p.y, height));
    }

    for p in &outline {
        bottom.push(positions.len() as i32);
        positions.push(Vector3::new(p.x, p.y, 0.0));
    }

    let top_centre = positions.len() as i32;
    positions.push(Vector3::new(0.0, 0.0, height));
    let bottom_centre = positions.len() as i32;
    positions.push(Vector3::new(0.0, 0.0, 0.0));

    let mut indices = Vec::new();
    for i in 0..n {
        let j = (i + 1) % n;
        indices.push((top_centre, top[i], top[j]));
        indices.push((bottom_centre, bottom[j], bottom[i]));
        indices.push((top[i], bottom[i], bottom[j]));
        indices.push((top[i], bottom[j], top[j]));
    }

    (positions, indices, top)
}

/// The outline: a 10 x 6 rectangle with corners rounded by a radius of 2 drawn in 10 segments
/// each (a finely tessellated convex arc) and a shallow V cut into the middle of the far side,
/// whose interior angle is a little over 180 degrees, so the rail's parameter there turns back by
/// only a hair.
fn dented_rounded_rectangle() -> Vec<Vector2> {
    const HALF_WIDTH: f64 = 5.0;
    const HALF_DEPTH: f64 = 3.0;
    const RADIUS: f64 = 2.0;
    const PER_CORNER: i32 = 10;

    let mut points = Vec::new();
    let corners = [
        Vector2::new(HALF_WIDTH - RADIUS, HALF_DEPTH - RADIUS),
        Vector2::new(-(HALF_WIDTH - RADIUS), HALF_DEPTH - RADIUS),
        Vector2::new(-(HALF_WIDTH - RADIUS), -(HALF_DEPTH - RADIUS)),
        Vector2::new(HALF_WIDTH - RADIUS, -(HALF_DEPTH - RADIUS)),
    ];

    for (c, corner) in corners.iter().enumerate() {
        let start = (std::f64::consts::PI / 2.0) * c as f64;
        for s in 0..=PER_CORNER {
            let angle = start + ((std::f64::consts::PI / 2.0) * s as f64 / PER_CORNER as f64);
            points.push(*corner + (Vector2::new(angle.cos(), angle.sin()) * RADIUS));
        }

        if c == 1 {
            // The dent, in the middle of the far side: in by a fifth of the depth and out again.
            // Its two ends are the arc ends either side of it - naming them again here would put
            // two outline points on one place, and a rim edge of NO LENGTH has no direction to
            // read: the wall triangles standing on it have no normal, so which edges count as
            // features moves with the part and the room moves with it.
            points.push(Vector2::new(-HALF_WIDTH + 0.6, 0.0));
        }
    }

    points
}
