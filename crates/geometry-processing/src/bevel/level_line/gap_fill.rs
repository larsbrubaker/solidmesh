//! The gap fill's rail readings. Ports the nested type of `Bevel/LevelLine/GapFill.cs` (a C#
//! static class): a loop read both ways round ([`GapFillRailSenses`]), the class's constants and the
//! rail's monotone slack ([`monotone_slack`]). The fill's other functions land with later packages.

use std::rc::Rc;

use vector_math::cs_math;

use super::{ChainField, ZipRail};

/// The tag the fill's own triangles carry, so the strip can be told from the kept surface. Stood
/// clear of every side-group id (a pick of several chains numbers its groups 1, 2, 3, ...).
pub const FILL_TAG: i32 = i32::MAX;

/// How close two rail parameters must be, relative to the chain's length, to count as the same
/// point of the chain in the zipper.
pub const PARAMETER_TIE_RELATIVE: f64 = 1e-9;

/// How far a rail's chain parameter may step BACKWARDS, as a fraction of the chain's least
/// setback, and still count as standing still rather than as a wrap.
///
/// Two readings measured on real parts say why this is not zero. Where the cut leaves two vertices
/// at the SAME position (77930's outer rail carries nine such pairs), their parameters are computed
/// by two different roots and differ in the last bits, so an exact comparison reads nine extra
/// wraps in a loop that is otherwise perfectly ordered. And where the rail curves round the far
/// side of a boss (65394's outer rail), the parameter genuinely wobbles - by 4e-4 on a chain 3.2
/// long, a thousandth of the setback. A backstep under this slack moves an arc's chain point by
/// less than a hundredth of the setback, below the chord tolerance the cut itself was traced to,
/// and the rail comes out monotone BY CONSTRUCTION, which is what the zipper's merge assumes.
pub const MONOTONE_SLACK_OF_SETBACK: f64 = 1e-2;

/// How deep an excursion below a rail's monotone envelope may run, as a multiple of the local
/// setback, before the rail is refused. On the INSIDE of a concave turn the minimising parameter
/// honestly swings BACK as the rail rounds the turn, by at most twice the setback over straight
/// legs; the fifth Thingi10K sweep's 62 non-monotone inputs split into 48 excursions at 2.19
/// setbacks or less and the rest from 7.39 up - correspondences that really do cross. Three is the
/// bound between those two populations.
pub const MONOTONE_EXCURSION_OF_SETBACK: f64 = 3.0;

/// How far back a rail's parameter may step on this chain and still count as standing still.
pub fn monotone_slack(chain: &ChainField) -> f64 {
    cs_math::max_f64(
        PARAMETER_TIE_RELATIVE * chain.length,
        MONOTONE_SLACK_OF_SETBACK * chain.min_setback,
    )
}

/// C# `GapFill.RailSenses`: a loop read both ways round, with what each reading cost in
/// flattening. Nothing picks a sense here, because only the PAIR of loops can say which sense is
/// which.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct GapFillRailSenses {
    /// The loop read in its own walk's sense, or `None` when that reading is not monotone.
    pub forward: Option<Rc<ZipRail>>,
    /// How much parameter the forward reading had to flatten away.
    pub forward_cost: f64,
    /// The loop read against its walk, or `None` when that reading is not monotone.
    pub backward: Option<Rc<ZipRail>>,
    /// How much parameter the backward reading had to flatten away.
    pub backward_cost: f64,
    /// Where the deeper of the two refusals happened, for a refusal to name.
    pub offending: i32,
}
