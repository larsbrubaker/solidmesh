//! Port of the test fixture `LevelLine/GrazeFixture.cs`: an ANNULUS plate - one round hole through
//! the middle of a round plate, both rims picked - carrying ONE mesh edge that runs from a vertex of
//! the outer rim, across the plate, GRAZING the hole's band on the way, and ending at a vertex out
//! in the clear. The field along that edge reads -, +, -, +: three crossings on a single edge whose
//! two ends are an ordinary Inside/Outside pair - the small, seconds-long stand-in for the Many
//! Holes BRIDGE that `graze_fixture_tests.rs` builds over.
//!
//! The construction is a chord TANGENT to a circle of radius r + graze * setback: it never enters
//! the hole, so it is all on material, but at its closest approach it stands less than a setback
//! from the rim and so reads Inside. The top face is tiled in three regions with no ear clipping and
//! no T-vertex, chosen so that the grazing chord is ONE edge of the input with exactly two triangles
//! on it: a LENS between the chord and the near half of the hole rim, a FAN from the chord's far end
//! over the outer rim arc beside it, and a STRIP over the whole rest of the annulus.

#![allow(dead_code)]

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::chain_setback;
use geometry_processing::bevel::level_line::seam_edges::FeatureEdgeTest;
use geometry_processing::bevel::level_line::{BandGapResult, BandProfile, ChainField, PickField};
use vector_math::Vector3;

use crate::annulus_plate_builder::AnnulusPlateBuilder;
use crate::level_line_feature_edges::feature_edges_over;
use crate::several_chains_fixtures;

/// The several-chain fold threshold, so both rims are feature edges.
pub const FOLD_DEGREES: f64 = several_chains_fixtures::FOLD_DEGREES;

/// The size the fixture is about. Every other length is quoted as a multiple of it.
pub const SIZE: f64 = 0.5;

/// The hole's radius, in sizes - chain 0.
pub const HOLE_RADIUS_IN_SIZES: f64 = 5.0;

/// The plate's radius, in sizes - chain 1.
pub const PLATE_RADIUS_IN_SIZES: f64 = 14.0;

/// Where the grazing chord's far end stands, in sizes. Halfway between the two rims, so it is more
/// than a setback clear of both and reads Outside however the chord is aimed.
pub const END_RADIUS_IN_SIZES: f64 = 9.5;

/// How many segments each rim carries.
pub const RING_SEGMENTS: i32 = 48;

/// The graze depth with HEIGHT: the chord's closest approach stands four tenths of a setback clear
/// of the rim, so it reads six tenths of a setback Inside and the triangle under it is 0.2 tall.
/// Four tenths and not seven: at seven the rules that judge a triangle by its own corners still
/// find the dip, and the defect this fixture is for does not appear.
pub const GRAZE_WITH_HEIGHT: f64 = 0.4;

/// The graze depth of a NEEDLE: the chord brushes the nearest rim vertex by a hundred thousandth of
/// a unit, so the triangle on it is under the refinement's no-height floor - 1e-3 of the chord
/// tolerance, which at `SIZE` is 2.5e-5 - and cannot be taken apart by height.
pub const GRAZE_AS_NEEDLE: f64 = 1e-5 / SIZE;

/// The same hundred-thousandth brush at a size other than `SIZE`.
pub fn needle_for(size: f64) -> f64 {
    1e-5 / size
}

/// C# `GrazePick`: the pick, and the three corners of the triangle the grazing chord is an edge
/// of: `rim_end` on the outer rim (Inside), `clear_end` out in the clear (Outside) and `apex` the
/// hole-rim vertex the chord passes over.
pub struct GrazePick {
    pub positions: Vec<Vector3>,
    pub indices: Vec<(i32, i32, i32)>,
    pub chains: Vec<Vec<i32>>,
    pub rim_end: i32,
    pub clear_end: i32,
    pub apex: i32,
    pub graze_radius: f64,
}

/// C# `Pick`: the annulus plate, the hole rim (chain 0) and the plate rim (chain 1), both walked
/// counter-clockwise seen from +z, with the grazing chord built in as an edge of the input.
/// `graze_in_setbacks` is how far clear of the hole rim the chord's closest approach stands, in
/// setbacks: under 1 the chord dips into the hole's band; over 0 it stays out of the hole itself.
/// C#'s defaults are `GRAZE_WITH_HEIGHT`, `RING_SEGMENTS` and `SIZE`; its argument checks panic.
pub fn pick(graze_in_setbacks: f64, ring_segments: i32, size: f64) -> GrazePick {
    assert!(
        ring_segments >= 16 && ring_segments % 4 == 0,
        "ring_segments out of range"
    );
    assert!(
        graze_in_setbacks > 0.0 && graze_in_setbacks < 1.0,
        "a GRAZE stays out of the hole and inside its band"
    );

    let r = HOLE_RADIUS_IN_SIZES * size;
    let plate = PLATE_RADIUS_IN_SIZES * size;
    let clear = END_RADIUS_IN_SIZES * size;
    // Both rims are square folds, so a fillet's setback is its radius and a chamfer's is its size:
    // one number serves whichever profile the build is asked for.
    let setback = chain_setback::for_chamfer(size);
    let wanted = r + (graze_in_setbacks * setback);

    // The frame is chosen so the chord is VERTICAL and touches the +x axis: the hole rim's vertex 0
    // sits at (r, 0), directly under the touch, and is the corner the chord's two triangles meet at.
    let step = 2.0 * std::f64::consts::PI / ring_segments as f64;
    let angle_of_rim_end = -(wanted / plate).acos();
    let mut b = AnnulusPlateBuilder::new(1e6, r, plate, ring_segments, angle_of_rim_end);

    // The chord's near end IS outer rim vertex 0, so the radius the chord actually touches is that
    // vertex's own x - within a bit of what was asked for, and exact as the tiling below reads it.
    let rim_end = b.rim(0);
    let graze_radius = b.positions[rim_end as usize].x;
    let clear_end = b.at(
        graze_radius,
        ((clear * clear) - (graze_radius * graze_radius)).sqrt(),
        0.0,
    );
    let apex = b.hole(0);

    // The LENS between the chord and the near half of the hole rim. Both fans reach the hole rim
    // inside the arc their apex can see - a quarter turn either way, well inside the tangent from
    // either end - so no fan triangle cuts into the hole.
    let lens_span = ring_segments / 4;
    for i in -lens_span..0 {
        let h0 = b.hole(i);
        let h1 = b.hole(i + 1);
        b.top(rim_end, h0, h1);
    }

    // THE grazing triangle: the chord, and the rim vertex it passes over.
    b.top(rim_end, clear_end, apex);

    for i in 0..lens_span {
        let h0 = b.hole(i);
        let h1 = b.hole(i + 1);
        b.top(clear_end, h0, h1);
    }

    // The FAN from the chord's far end over the outer rim arc beside it. It reaches one segment
    // past that end's own bearing, so the cut back to the rim leans forward and never re-crosses
    // the chord.
    let clear_at = b.positions[clear_end as usize];
    let angle_of_clear_end = clear_at.y.atan2(clear_at.x);
    // C# `(int)Math.Ceiling(...)`.
    let fan_span = ((angle_of_clear_end - angle_of_rim_end) / step).ceil() as i32 + 1;
    for j in 0..fan_span {
        let r0 = b.rim(j);
        let r1 = b.rim(j + 1);
        b.top(clear_end, r0, r1);
    }

    // The STRIP over the rest of the annulus: the outer arc from where the fan stopped round to
    // the chord's near end, against the hole arc from the chord's far end round to where the lens
    // stopped.
    let mut outer_arc = Vec::new();
    for j in fan_span..=ring_segments {
        outer_arc.push(b.rim(j));
    }

    let mut inner_arc = vec![clear_end];
    for i in lens_span..=ring_segments - lens_span {
        inner_arc.push(b.hole(i));
    }

    strip_between(&mut b, &outer_arc, &inner_arc);

    let (positions, indices, chains) = b.close();
    GrazePick {
        positions,
        indices,
        chains,
        rim_end,
        clear_end,
        apex,
        graze_radius,
    }
}

/// C# `StripBetween`: walked greedily between the two arcs, the shorter diagonal first.
fn strip_between(b: &mut AnnulusPlateBuilder, arc_a: &[i32], arc_b: &[i32]) {
    let (mut i, mut j) = (0, 0);
    while i + 1 < arc_a.len() || j + 1 < arc_b.len() {
        let can_a = i + 1 < arc_a.len();
        let can_b = j + 1 < arc_b.len();
        let step_a = can_a
            && (!can_b
                || (b.positions[arc_a[i + 1] as usize] - b.positions[arc_b[j] as usize]).length()
                    <= (b.positions[arc_a[i] as usize] - b.positions[arc_b[j + 1] as usize])
                        .length());
        if step_a {
            b.top(arc_a[i], arc_a[i + 1], arc_b[j]);
            i += 1;
        } else {
            b.top(arc_a[i], arc_b[j + 1], arc_b[j]);
            j += 1;
        }
    }
}

/// C# `FeatureEdges`: the pick's feature edges, read at the several-chain fold threshold.
pub fn feature_edges(positions: &[Vector3], indices: &[(i32, i32, i32)]) -> FeatureEdgeTest {
    feature_edges_over(positions, indices, FOLD_DEGREES)
}

/// C# `Build` (defaults `SIZE`, chamfer): one JOINT band build of both rims - the build the graze
/// defect shows up in.
pub fn build(pick: &GrazePick, size: f64, profile: BandProfile) -> BandGapResult {
    let closed = vec![true; pick.chains.len()];
    band_gap_entry_points::build_pick(
        &pick.positions,
        &pick.indices,
        &pick.chains,
        &closed,
        Some(feature_edges(&pick.positions, &pick.indices)),
        size,
        profile,
        &BandGapBuildOptions::default(),
    )
    .expect("the pick is well formed")
}

/// C# `FieldOf` (default `SIZE`): the pick's own field - the production `PickField` over both
/// rims at the chamfer setback - so a test can read what the refinement reads along an edge rather
/// than a copy of it.
pub fn field_of(chains: &[Vec<i32>], positions: &[Vector3], size: f64) -> PickField {
    let setback = chain_setback::for_chamfer(size);
    let fields: Vec<Rc<ChainField>> = chains
        .iter()
        .map(|chain| {
            let points: Vec<Vector3> = chain.iter().map(|&v| positions[v as usize]).collect();
            let setbacks = vec![setback; chain.len()];
            Rc::new(ChainField::from_polyline(&points, true, &setbacks).expect("a valid chain"))
        })
        .collect();
    let members: Vec<i32> = (0..fields.len() as i32).collect();
    PickField::new(fields, &members).expect("a valid pick")
}
