//! Test fixture: a stadium plate with TWO round through-holes side by side whose facing rims stand
//! `GAP_IN_SIZES` sizes apart, and whose top face between them is tiled by triangles that SPAN the
//! two rims - the small stand-in for the Many Holes bridge, the shape of input triangulation the
//! witness has and `grid_hole_plate_fixture` does not. Built as five tiled regions (a middle strip,
//! a cap strip round each hole's outer arc, a quad above and below) so nothing is ear-clipped and
//! no edge carries a T-vertex; the bottom is the same tiling wound the other way and the walls are
//! one quad per rim edge, so the part is a closed solid. Read by `bridge_fixture_tests`.

use std::collections::HashMap;
use std::f64::consts::PI;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::seam_edges::FeatureEdgeTest;
use geometry_processing::bevel::level_line::{BandGapResult, BandProfile};
use vector_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_room_geometry::SolidWithChains;
use crate::several_chains_fixtures;

/// The several-chain fold threshold, so the rim of a hole is a feature edge.
pub const FOLD_DEGREES: f64 = several_chains_fixtures::FOLD_DEGREES;

/// The size the fixture is about. Every other length is quoted as a multiple of it.
pub const SIZE: f64 = 0.5;

/// Each hole's radius, in sizes.
pub const RADIUS_IN_SIZES: f64 = 5.0;

/// How far the two rims stand apart, edge to edge, in sizes - shared between two bands.
pub const GAP_IN_SIZES: f64 = 4.0;

/// How much plate there is outside each hole's outer arc, in sizes.
pub const MARGIN_IN_SIZES: f64 = 6.0;

/// Thick enough that no chain's own wall bounds it at `SIZE`.
const THICKNESS: f64 = 3.0;

/// The pick with holes of the given radius (in sizes) and every other setting its default (32 ring
/// segments, the default gap and size).
pub fn pick_with_radius(radius_in_sizes: f64) -> SolidWithChains {
    pick(32, radius_in_sizes, GAP_IN_SIZES, SIZE)
}

/// The plate and the two hole rims, hole A (at -x) first, both walked
/// counter-clockwise seen from +z. `ring_segments` must be a multiple of four (at least 8) so the
/// four quadrant points are ring vertices.
pub fn pick(
    ring_segments: i32,
    radius_in_sizes: f64,
    gap_in_sizes: f64,
    size: f64,
) -> SolidWithChains {
    assert!(
        ring_segments >= 8 && ring_segments % 4 == 0,
        "ring_segments out of range"
    );

    let r = radius_in_sizes * size;
    let d = r + (gap_in_sizes * size / 2.0);
    let outer = r + (MARGIN_IN_SIZES * size);

    let mut welder = Welder::default();
    let mut indices: Vec<(i32, i32, i32)> = Vec::new();

    let step = 2.0 * PI / ring_segments as f64;
    let on = |centre_x: f64, radius: f64, angle: f64| {
        (centre_x + (radius * angle.cos()), radius * angle.sin())
    };

    let half = (ring_segments / 2) as usize;
    let quarter = PI / 2.0;

    // The MIDDLE strip. Chord i runs from hole A's right arc at 90 - i steps to hole B's left arc
    // at 90 + i steps; by the mirror symmetry both ends share a y, so the chords are parallel
    // horizontals and the quads between them tile the region between the two rims exactly.
    let mut middle_a = vec![0; half + 1];
    let mut middle_b = vec![0; half + 1];
    for i in 0..=half {
        let a = on(-d, r, quarter - (i as f64 * step));
        let b = on(d, r, quarter + (i as f64 * step));
        middle_a[i] = welder.at(a.0, a.1, 0.0);
        middle_b[i] = welder.at(b.0, b.1, 0.0);
    }

    for i in 0..half {
        quad(
            &mut indices,
            [middle_a[i], middle_a[i + 1], middle_b[i + 1], middle_b[i]],
        );
    }

    // A CAP strip round each hole's outer arc - hole A's left half, hole B's right half - one quad
    // per rim edge, so the rim's edges are shared with the wall below and never split.
    for (centre_x, from_angle) in [(-d, quarter), (d, -quarter)] {
        for j in 0..half {
            let a0 = from_angle + (j as f64 * step);
            let a1 = from_angle + ((j + 1) as f64 * step);
            let i0 = on(centre_x, r, a0);
            let i1 = on(centre_x, r, a1);
            let o0 = on(centre_x, outer, a0);
            let o1 = on(centre_x, outer, a1);
            let corners = [
                welder.at(i0.0, i0.1, 0.0),
                welder.at(o0.0, o0.1, 0.0),
                welder.at(o1.0, o1.1, 0.0),
                welder.at(i1.0, i1.1, 0.0),
            ];
            quad(&mut indices, corners);
        }
    }

    // The two quads that close the stadium above and below the middle strip.
    let above = [
        welder.at(-d, r, 0.0),
        welder.at(d, r, 0.0),
        welder.at(d, outer, 0.0),
        welder.at(-d, outer, 0.0),
    ];
    quad(&mut indices, above);
    let below = [
        welder.at(-d, -outer, 0.0),
        welder.at(d, -outer, 0.0),
        welder.at(d, -r, 0.0),
        welder.at(-d, -r, 0.0),
    ];
    quad(&mut indices, below);

    let top_face = indices.clone();

    // The bottom face is the same tiling at -THICKNESS, wound the other way.
    for (a, b, c) in top_face {
        let pa = welder.positions[a as usize];
        let pb = welder.positions[b as usize];
        let pc = welder.positions[c as usize];
        indices.push((
            welder.at(pa.x, pa.y, -THICKNESS),
            welder.at(pc.x, pc.y, -THICKNESS),
            welder.at(pb.x, pb.y, -THICKNESS),
        ));
    }

    // Each hole's rim, counter-clockwise seen from +z, and the wall under it facing INTO the hole.
    let mut chains = Vec::new();
    for centre_x in [-d, d] {
        let mut rim = Vec::new();
        for k in 0..ring_segments {
            let p = on(centre_x, r, k as f64 * step);
            rim.push(welder.at(p.0, p.1, 0.0));
        }

        for k in 0..ring_segments {
            let from = on(centre_x, r, ((k + 1) % ring_segments) as f64 * step);
            let to = on(centre_x, r, k as f64 * step);
            let corners = [
                welder.at(from.0, from.1, -THICKNESS),
                welder.at(to.0, to.1, -THICKNESS),
                welder.at(to.0, to.1, 0.0),
                welder.at(from.0, from.1, 0.0),
            ];
            quad(&mut indices, corners);
        }

        chains.push(rim);
    }

    // The plate's own outer wall, facing out, one quad per outer boundary edge.
    let boundary = outer_loop(d, outer, ring_segments, step, quarter);
    for k in 0..boundary.len() {
        let from = boundary[k];
        let to = boundary[(k + 1) % boundary.len()];
        let corners = [
            welder.at(from.0, from.1, -THICKNESS),
            welder.at(to.0, to.1, -THICKNESS),
            welder.at(to.0, to.1, 0.0),
            welder.at(from.0, from.1, 0.0),
        ];
        quad(&mut indices, corners);
    }

    (welder.positions, indices, chains)
}

/// The positions so far, welded by rounding to a nanometre.
#[derive(Default)]
struct Welder {
    positions: Vec<Vector3>,
    by_position: HashMap<(i64, i64, i64), i32>,
}

impl Welder {
    /// The index of the point at (x, y, z), added if no point is there yet. Rounding to a
    /// nanometre welds the pieces that meet; the regions share their boundary points by
    /// construction, so nothing that should stay apart is ever within it. The rounding is
    /// banker's rounding (ties to even).
    fn at(&mut self, x: f64, y: f64, z: f64) -> i32 {
        let key = (
            (x * 1e6).round_ties_even() as i64,
            (y * 1e6).round_ties_even() as i64,
            (z * 1e6).round_ties_even() as i64,
        );
        let positions = &mut self.positions;
        *self.by_position.entry(key).or_insert_with(|| {
            positions.push(Vector3::new(x, y, z));
            positions.len() as i32 - 1
        })
    }
}

/// One quad as two triangles, (a, b, c) and (a, c, d).
fn quad(indices: &mut Vec<(i32, i32, i32)>, [a, b, c, d]: [i32; 4]) {
    indices.push((a, b, c));
    indices.push((a, c, d));
}

/// The plate's outer boundary counter-clockwise seen from +z - the top edge right
/// to left, hole A's outer arc, the bottom edge left to right, then hole B's outer arc.
fn outer_loop(d: f64, outer: f64, ring_segments: i32, step: f64, quarter: f64) -> Vec<(f64, f64)> {
    let half = ring_segments / 2;
    let mut points = vec![(d, outer)];
    for j in 0..=half {
        let a = quarter + (j as f64 * step);
        points.push((-d + (outer * a.cos()), outer * a.sin()));
    }

    for j in 0..=half {
        let a = -quarter + (j as f64 * step);
        points.push((d + (outer * a.cos()), outer * a.sin()));
    }

    // The first point and the last are the same place; the loop is closed by the caller's wrap.
    points.pop();
    points
}

/// The pick's feature edges, read at the several-chain fold threshold.
pub fn feature_edges(positions: &[Vector3], indices: &[(i32, i32, i32)]) -> FeatureEdgeTest {
    feature_edges_over(positions, indices, FOLD_DEGREES)
}

/// One chamfer (the default profile) JOINT band build of both rims - the build the
/// bridge defect shows up in.
pub fn build(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    size: f64,
) -> BandGapResult {
    let closed = vec![true; chains.len()];
    band_gap_entry_points::build_pick(
        positions,
        indices,
        chains,
        &closed,
        Some(feature_edges(positions, indices)),
        size,
        BandProfile::Chamfer,
        &BandGapBuildOptions::default(),
    )
    .expect("the pick builds or refuses by name")
}
