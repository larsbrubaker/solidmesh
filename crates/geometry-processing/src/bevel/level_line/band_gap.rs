//! The band between two offsets of a pick's chains (`Bevel/LevelLine/BandGap.cs`, the main file of
//! the C# static partial class `BandGap`: free functions here; its other partials are
//! `band_gap_in_turn.rs` and `band_gap_rounds_first.rs`). The size refusal and the chain ceiling
//! land here first; the build lands with later packages.

use vector_math::Vector3;

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
