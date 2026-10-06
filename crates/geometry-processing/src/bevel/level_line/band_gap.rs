//! The band between two offsets of a pick's chains (`Bevel/LevelLine/BandGap.cs`, the main file of
//! the C# static partial class `BandGap`: free functions here; its other partials are
//! `band_gap_in_turn.rs` and `band_gap_rounds_first.rs`). The size refusal, the chain ceiling, the
//! chord tolerance, the needle floor and the closed chains turned to start at a corner
//! ([`start_closed_chains_at_a_corner`]) land here first; the build lands with later packages.

use vector_math::{cs_math, Vector3};

use std::collections::HashSet;

use super::{band_side_tagging, pick_validation, self_room, BandGapRefusal, BandGapResult};

/// The answer for a pick refused at its size, with the `room` it does have.
pub fn refused_by_size(room: f64) -> BandGapResult {
    BandGapResult {
        refusal: BandGapRefusal::Size,
        room,
        ..BandGapResult::default()
    }
}

/// The largest size the pick could have room for at all ([`self_room::chain_ceiling`]).
pub fn chain_ceiling(chain_points: &[Vector3]) -> f64 {
    self_room::chain_ceiling(chain_points)
}

/// C# `BandGap.ChordToleranceOf`: the chord tolerance a build at this size uses: the one asked
/// for, or size / 20 when none was.
pub fn chord_tolerance_of(size: f64, chord_tolerance: f64) -> f64 {
    if chord_tolerance > 0.0 {
        chord_tolerance
    } else {
        size / 20.0
    }
}

/// C# `BandGap.NeedleFloorOf`: the height under which a build's working mesh calls a triangle a
/// needle (`LevelLineMesh::needle_floor`). Named so a measurement can read the very floor a build
/// uses.
pub fn needle_floor_of(snap: f64, tolerance: f64) -> f64 {
    cs_math::max_f64(4.0 * snap, 1e-3 * tolerance)
}

/// C# `StartClosedChainsAtACorner`: each closed chain holding a corner of the pick, rotated to
/// START there. A closed chain's rail parameter runs from its first vertex round to it, so its seam
/// sits at that vertex; a seam anywhere but a corner lies INSIDE a stretch of the rails, which then
/// reads `RailIsNotMonotone`. Only the start moves - the chain, its direction and its index do not.
/// A chain with no `closed` entry (C# would throw indexing past the list) is read as open.
pub fn start_closed_chains_at_a_corner(
    chains: &[Vec<i32>],
    closed: Option<&[bool]>,
) -> Vec<Vec<i32>> {
    let Some(closed) = closed else {
        return chains.to_vec();
    };

    if !band_side_tagging::both_sided_members_build() || !closed.iter().any(|&c| c) {
        return chains.to_vec();
    }

    let corners: HashSet<i32> = pick_validation::corners_of(chains, closed, true)
        .iter()
        .map(|corner| corner.vertex)
        .collect();

    if corners.is_empty() {
        return chains.to_vec();
    }

    chains
        .iter()
        .enumerate()
        .map(|(k, chain)| {
            let at = if closed.get(k).copied().unwrap_or(false) {
                chain.iter().position(|v| corners.contains(v)).unwrap_or(0)
            } else {
                0
            };

            if at == 0 {
                chain.clone()
            } else {
                chain[at..].iter().chain(&chain[..at]).copied().collect()
            }
        })
        .collect()
}
