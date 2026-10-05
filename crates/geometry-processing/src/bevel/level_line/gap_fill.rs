//! The gap fill's rail readings. Ports the nested type of `Bevel/LevelLine/GapFill.cs` (a C#
//! static class): a loop read both ways round ([`GapFillRailSenses`]). The fill's functions and
//! static state land with later packages.

use std::rc::Rc;

use super::ZipRail;

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
