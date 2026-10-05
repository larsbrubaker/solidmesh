//! The band between two offsets of a pick's chains (`Bevel/LevelLine/BandGap.cs`, the main file of
//! the C# static partial class `BandGap`: free functions here; its other partials are
//! `band_gap_in_turn.rs` and `band_gap_rounds_first.rs`). The size refusal, the chain ceiling, the
//! chord tolerance and the needle floor land here first; the build lands with later packages.

use vector_math::{cs_math, Vector3};

use super::{self_room, BandGapRefusal, BandGapResult};

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
