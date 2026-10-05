//! The room a side of the band has in ITSELF (`Bevel/LevelLine/SelfRoom.cs`, a static class: free
//! functions here). Its tolerances and the chain's ceiling land here first; the reading of a
//! side's own room lands with later packages.

use vector_math::{cs_math, Vector3};

/// The FINE pass's tolerance, as a fraction of the peak distance it reads.
pub const PEAK_RELATIVE_TOLERANCE: f64 = 1.0 / 20.0;

/// The COARSE pass's tolerance, as a fraction of the longest a peak's distance could possibly be:
/// a length of the PART, so the coarse pass is scale-free too.
pub const COARSE_RELATIVE_TOLERANCE: f64 = 1.0 / 16.0;

/// Above this many chain points the ceiling is taken from the chain's length, not its pairs.
pub const CEILING_PAIR_LIMIT: usize = 3000;

/// How many times a VARYING side's room is re-read at the size it has just proved.
pub const VARYING_STEPS: i32 = 6;

/// The greatest distance between two of `chain_points`, read in full up to
/// [`CEILING_PAIR_LIMIT`] points and bounded by the chain's own length past that - the largest
/// size the pick could possibly have room for. A pick with no extent at all (every chain vertex on
/// one point) names no ceiling: infinity.
pub fn chain_ceiling(chain_points: &[Vector3]) -> f64 {
    let mut greatest = 0.0;

    if chain_points.len() <= CEILING_PAIR_LIMIT {
        for i in 0..chain_points.len() {
            for j in (i + 1)..chain_points.len() {
                let apart = (chain_points[j] - chain_points[i]).length();
                greatest = cs_math::max_f64(greatest, apart);
            }
        }
    } else {
        for i in 1..chain_points.len() {
            greatest += (chain_points[i] - chain_points[i - 1]).length();
        }
    }

    // A self-room is a size, and a size is a length in the part's own units - the same units this
    // distance is in.
    if greatest > 0.0 {
        greatest
    } else {
        f64::INFINITY
    }
}
