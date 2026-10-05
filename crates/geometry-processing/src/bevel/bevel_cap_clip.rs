//! Clipping a cutter whose run ended on a cap plane the sweep could not carry
//! (`Bevel/BevelCapClip.cs`, a C# `internal static class`, so a module here): one cap a boolean
//! owes ([`ClippedCap`]), one cutter with the caps to take off it ([`Candidate`]) and a cutter
//! that could not be clipped and was withdrawn ([`Withdrawal`]). The demotions a capped run is
//! retried with ([`attempts_for`]) land here; the clip itself lands with later packages.

use vector_math::Vector3;

use super::{BevelAdvisory, BevelChainEnds, BevelEndCap};
use crate::topology::ChainEdge;

/// C# `BevelCapClip.ClippedCap`: one cap the boolean owes - the plane to subtract, and the
/// vertex the run ends at, which is what the refusal has to name if the subtraction cannot be
/// made.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ClippedCap {
    /// The half space the tool must stay out of, in the cutter's own space.
    pub plane: BevelEndCap,
    /// Where the run ends, for the message.
    pub at: Vector3,
}

/// C# `BevelCapClip.Candidate`: one cutter and the cap planes to take off it.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Candidate {
    /// Where this cutter is in the builder's cutter list.
    pub cutter_index: i32,
    /// Which group to fail if the clip cannot be made.
    pub group_index: i32,
    /// The operation's bevel size, which the refusal quotes.
    pub size: f64,
    /// The caps to subtract. Never empty.
    pub clips: Vec<ClippedCap>,
}

/// C# `BevelCapClip.Withdrawal`: a cutter that could not be clipped and has been taken out of
/// the batch, with what to say on its group's row.
#[derive(Debug, Clone)]
pub struct Withdrawal {
    /// The group whose cutter was withdrawn.
    pub group_index: i32,
    /// The refusal to put on that group's row.
    pub reason: BevelAdvisory,
}

impl ClippedCap {
    /// C#'s constructor.
    pub fn new(plane: BevelEndCap, at: Vector3) -> Self {
        ClippedCap { plane, at }
    }
}

impl Candidate {
    /// C#'s constructor.
    pub fn new(cutter_index: i32, group_index: i32, size: f64, clips: Vec<ClippedCap>) -> Self {
        Candidate {
            cutter_index,
            group_index,
            size,
            clips,
        }
    }
}

/// The retries a run capped at either end is offered: each capped end in turn demoted to an
/// overshoot (and its cap owed as a clip at the run's first or last vertex), the other end's cap
/// and overshoot bound carried across unchanged. Exit planes are not carried, as C#'s retry does
/// not carry them.
pub fn attempts_for(
    ends: &BevelChainEnds,
    run: &[ChainEdge],
) -> Vec<(BevelChainEnds, Vec<ClippedCap>)> {
    let mut attempts = Vec::with_capacity(2);

    let carried =
        |start: BevelEndCap, end: BevelEndCap, start_max: f64, end_max: f64| BevelChainEnds {
            start_pair_corner: ends.start_pair_corner,
            end_pair_corner: ends.end_pair_corner,
            start_maximum_overshoot: start_max,
            end_maximum_overshoot: end_max,
            start_overshoot_corner: ends.start_overshoot_corner,
            end_overshoot_corner: ends.end_overshoot_corner,
            ..BevelChainEnds::new(start, end)
        };

    // The OTHER end's overshoot bound is carried across unchanged; the demoted end takes none -
    // it is meant to run past and be cut back by the plane.
    if ends.start.is_capped() {
        attempts.push((
            carried(
                BevelEndCap::default(),
                ends.end.clone(),
                f64::INFINITY,
                ends.end_maximum_overshoot,
            ),
            vec![ClippedCap::new(ends.start.clone(), run[0].from())],
        ));
    }

    if ends.end.is_capped() {
        attempts.push((
            carried(
                ends.start.clone(),
                BevelEndCap::default(),
                ends.start_maximum_overshoot,
                f64::INFINITY,
            ),
            vec![ClippedCap::new(ends.end.clone(), run[run.len() - 1].to())],
        ));
    }

    attempts
}
