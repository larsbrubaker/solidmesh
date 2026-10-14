//! What the level-line field reads at one point: how far the chain is, which point of the chain
//! that is, and the field value whose zero set is the level line.

use vector_math::Vector3;

/// One reading of the field at a point: plain public fields, copied by value.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct FieldSample {
    /// Straight-line distance from the queried point to the chain point named by `parameter`.
    pub distance: f64,

    /// Arc length along the chain, in 0..the chain field's length, of the chain point whose BALL
    /// reaches the queried point first - the c attaining the minimum below, not simply the nearest
    /// point.
    pub parameter: f64,

    /// `min over chain points c of (|q - c| - setback(c))`: the signed distance to the union of the
    /// balls the chain asks for. The bevel's boundary is where this is zero, so a varying setback
    /// is just a varying number rather than a different construction. Equal to
    /// `distance - setback(parameter)`, since the minimising c is where both are read.
    pub value: f64,

    /// That chain point itself.
    pub nearest: Vector3,

    /// WHICH chain of a pick the reading came from - an index into the pick field's chains, and
    /// zero for the field of a single chain, which is the pick of one. `parameter` stays LOCAL to
    /// this chain: a single parameter line laid end to end over several chains would put a closed
    /// chain's wrap in the middle of it, and the wrap is exactly where the fill pairs its rails.
    pub chain: i32,

    /// True when an END CLIP gave this reading: the value is the end plane's and the chain point is
    /// the end vertex, so `distance - value` is NOT a setback here.
    pub clipped: bool,
}
