//! The Many Holes BRIDGE in the two shapes `graze_fixture.rs` does not reach - a single mesh edge
//! that runs clean across the plate from ONE chain's level line to ANOTHER's, and the same run of
//! surface carried by an edge whose two ends are both rim vertices with a NEEDLE on either side of
//! it.
//!
//! The part is the same annulus plate as the graze fixture - one round hole through a round plate,
//! both rims picked - but tiled sector by sector so that one sector can carry the defect without
//! disturbing any other, and with the hole and plate vertices at the SAME bearings so each sector
//! is a plain quad.

#![allow(dead_code)]

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::chain_setback;
use solidmesh::bevel::level_line::{BandGapResult, BandProfile, PickField};
use solidmesh_math::Vector3;

use crate::annulus_plate_builder::AnnulusPlateBuilder;
use crate::graze_fixture;

/// The size the fixture is about. Every other length is quoted as a multiple of it.
pub const SIZE: f64 = 0.5;

/// The hole's radius, in sizes - chain 0.
pub const HOLE_RADIUS_IN_SIZES: f64 = 5.0;

/// The plate's radius, in sizes - chain 1.
pub const PLATE_RADIUS_IN_SIZES: f64 = 14.0;

/// How many segments each rim carries.
pub const RING_SEGMENTS: i32 = 48;

/// How far off the rim-to-rim edge the needle's third corner stands. A hundred thousandth of a unit
/// is under the refinement's no-height floor - 1e-3 of the chord tolerance, so 2.5e-5 at `SIZE` -
/// and twenty times what a test reads two vertices as coincident at.
pub const NEEDLE_HEIGHT: f64 = 1e-5;

/// Which of the two unasked edge shapes the pick carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Defect {
    /// Two inserted vertices standing ON two different chains' level lines, joined by an edge.
    OnToOn,
    /// A rim-to-rim edge, Inside at both ends, with a needle triangle on each side of it.
    InsideToInsideNeedle,
    /// The witness's own shape: the `OnToOn` rung with a NEEDLE on either side of it, so the chord
    /// rule - which halves the rung in `OnToOn` because it sees exactly two On corners - is held
    /// behind the no-height gate and nobody is left to ask about the edge.
    OnToOnNeedle,
}

/// The pick and the two ends of the bad edge - `near` on or at chain 0, and `far` on
/// or at chain 1.
pub struct LinePick {
    pub positions: Vec<Vector3>,
    pub indices: Vec<(i32, i32, i32)>,
    pub chains: Vec<Vec<i32>>,
    pub near: i32,
    pub far: i32,
}

/// The annulus plate carrying one bad edge of the
/// shape asked for, both rims picked. Arguments out of range panic.
/// The usual arguments are `RING_SEGMENTS` and `SIZE`.
pub fn pick(defect: Defect, ring_segments: i32, size: f64) -> LinePick {
    assert!(
        ring_segments >= 16 && ring_segments % 4 == 0,
        "ring_segments out of range"
    );

    let r = HOLE_RADIUS_IN_SIZES * size;
    let plate = PLATE_RADIUS_IN_SIZES * size;
    let setback = chain_setback::for_chamfer(size);
    // A nanometre welds what meets; every region shares its boundary points by construction, and
    // the needle's own corner stands ten nanometres off, so nothing separate is ever welded.
    let mut b = AnnulusPlateBuilder::new(1e9, r, plate, ring_segments, 0.0);
    let step = b.step();

    // The two level lines, on the ray up the middle of sector 0. The chain is a polygon, not a
    // circle, so the nearest chain point to anything on that ray is the midpoint of the sector's
    // own chord, at radius (radius * cos(half a step)) - which makes these two radii EXACT roots of
    // the field, well inside the build's 1e-9 snap, and so vertices that read On from the start.
    let half = step / 2.0;
    let on_chain_zero = (r * half.cos()) + setback;
    let on_chain_one = (plate * half.cos()) - setback;

    let (mut near, mut far) = (-1, -1);
    let needle_at = ring_segments - 1;

    for j in 0..ring_segments {
        let a = b.hole(j);
        let bb = b.hole(j + 1);
        let c = b.rim(j + 1);
        let d = b.rim(j);

        if defect == Defect::OnToOnNeedle && j == 0 {
            // The same RUNG, with the two triangles that carry it made needles. The sector's quad
            // is split by the rung into two convex quads - (near, b, c, far) and (near, far, d, a) -
            // and each is fanned from a point a hundred thousandth of a unit off the rung's middle
            // on its own side, so the rung's two triangles are both under the no-height floor while
            // every boundary edge of the sector is still one whole edge shared with its neighbour.
            near = b.at(on_chain_zero * half.cos(), on_chain_zero * half.sin(), 0.0);
            far = b.at(on_chain_one * half.cos(), on_chain_one * half.sin(), 0.0);
            let middle = (b.positions[near as usize] + b.positions[far as usize]) * 0.5;
            let across = Vector3::new(-half.sin(), half.cos(), 0.0);
            let plus = b.at(
                middle.x + (NEEDLE_HEIGHT * across.x),
                middle.y + (NEEDLE_HEIGHT * across.y),
                0.0,
            );
            let minus = b.at(
                middle.x - (NEEDLE_HEIGHT * across.x),
                middle.y - (NEEDLE_HEIGHT * across.y),
                0.0,
            );
            b.top(a, bb, near);
            b.top(plus, near, bb);
            b.top(plus, bb, c);
            b.top(plus, c, far);
            b.top(plus, far, near);
            b.top(c, d, far);
            b.top(minus, far, d);
            b.top(minus, d, a);
            b.top(minus, a, near);
            b.top(minus, near, far);
            continue;
        }

        if defect == Defect::OnToOn && j == 0 {
            // The RUNG: two vertices strictly inside the sector, one on each chain's level line,
            // joined by an edge of the input. The quad round them is tiled so that edge is shared
            // by exactly two triangles and every boundary edge is still shared with the
            // neighbouring sector.
            near = b.at(on_chain_zero * half.cos(), on_chain_zero * half.sin(), 0.0);
            far = b.at(on_chain_one * half.cos(), on_chain_one * half.sin(), 0.0);
            b.top(a, bb, near);
            b.top(bb, c, far);
            b.top(bb, far, near);
            b.top(c, d, far);
            b.top(d, a, near);
            b.top(d, near, far);
            continue;
        }

        if defect == Defect::InsideToInsideNeedle && (j == 0 || j == needle_at) {
            // The radial edge at bearing zero - hole rim vertex to plate rim vertex, Inside at both
            // ends - with its third corner a hundred thousandth of a unit off it on this sector's
            // side. The two sectors either side of the edge each contribute one such needle, so
            // BOTH triangles on the edge are under the no-height floor and no neighbour with height
            // can be asked instead.
            let side = if j == 0 { 1.0 } else { -1.0 };
            let q = b.at((r + plate) / 2.0, side * NEEDLE_HEIGHT, 0.0);
            near = b.hole(0);
            far = b.rim(0);
            b.top(q, a, bb);
            b.top(q, bb, c);
            b.top(q, c, d);
            b.top(q, d, a);
            continue;
        }

        b.top(a, bb, c);
        b.top(a, c, d);
    }

    let (positions, indices, chains) = b.close();
    LinePick {
        positions,
        indices,
        chains,
        near,
        far,
    }
}

/// One chamfer JOINT band build of both rims - the build the bridge
/// shows up in.
pub fn build(pick: &LinePick, size: f64, profile: BandProfile) -> BandGapResult {
    let closed = vec![true; pick.chains.len()];
    band_gap_entry_points::build_pick(
        &pick.positions,
        &pick.indices,
        &pick.chains,
        &closed,
        Some(graze_fixture::feature_edges(&pick.positions, &pick.indices)),
        size,
        profile,
        &BandGapBuildOptions::default(),
    )
    .expect("the pick is well formed")
}

/// The pick's own field - the production `PickField` over both
/// rims at the chamfer setback.
pub fn field_of(pick: &LinePick, size: f64) -> PickField {
    graze_fixture::field_of(&pick.chains, &pick.positions, size)
}
