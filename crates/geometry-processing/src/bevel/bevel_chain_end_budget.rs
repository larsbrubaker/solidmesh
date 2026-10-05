//! What ONE END of a run can afford to do, measured against the segment that has to pay for it -
//! and the sentence the user reads when it cannot afford what it wanted. Ports
//! `Bevel/BevelChainEndBudget.cs` (C# `internal static class`; a module of free functions here).
//!
//! Two questions, one budget. [`minimum_bead_cap_agreement`] asks how far off square a capping
//! plane may lean before the slide onto it stops fitting in the end's own segment;
//! [`maximum_overshoot`] asks how far an UNCAPPED end may run past its own last vertex. Both
//! answer "half the adjoining segment", and for the same reason: a strip runs between two rings
//! and the other end of it is taking its own bite, so half is this end's share. They were
//! extracted together because they are the same arithmetic asked at the same vertex, and because
//! `BevelChainEnds` is about which PLANE an end lands on rather than about what its segment can
//! pay for.
//!
//! The warning belongs here for the same reason it is one line rather than two: it is the bound
//! explaining itself, and the words it uses - the size, the ends, how much was picked - are
//! exactly the terms the bound is computed from. The closed forms are
//! [`bevel_edge_tool_builder`]'s.

use super::bevel_edge_tool_builder;
use super::{BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode, BevelEndCap};
use crate::topology::ChainEdge;

/// How far one end may overshoot - half its own segment at every uncapped end that is not a 5.3
/// corner - and whether that bound is actually shorter than the overshoot it replaces, which is
/// the only case worth telling the user about. Returned as C#'s `(Limit, Bit)`.
///
/// `cap` is what that end resolved to; a capped end never overshoots and so never takes a bound.
/// `reaches_for_something` is whether this end is running on TOWARDS something rather than merely
/// past its own last vertex, which exempts it - see `BevelChainEnds.StartMaximumOvershoot`. Two
/// ends are: one where another selected edge of this run's convexity departs from the vertex, and
/// one running on through an interruption in its own crease (`BevelInterruptedCrease`), which has
/// to reach the far side of what interrupted it. `segment` is the end's own segment, whose half
/// length is the bound.
pub fn maximum_overshoot(
    cap: &BevelEndCap,
    reaches_for_something: bool,
    size: f64,
    round_profile: bool,
    segment: &ChainEdge,
) -> (f64, bool) {
    if cap.is_capped() || reaches_for_something {
        return (f64::INFINITY, false);
    }

    let limit = (segment.to() - segment.from()).length() / 2.0;

    (limit, limit < reach(size, round_profile, segment))
}

/// How far an uncapped end actually runs past its own last vertex.
///
/// Recomputed from closed forms because nothing has been swept yet. It is only ever compared - the
/// ring is still placed from the section's own tangent distance - so what is needed of it is that
/// it agree with the builder, which `BevelVoidWedgeSectionTests` is where it is pinned. Shared with
/// `BevelChainEnds`, which asks the same question of an end running on through an interruption:
/// whether that reach carries it to the far side.
pub fn reach(size: f64, round_profile: bool, segment: &ChainEdge) -> f64 {
    bevel_edge_tool_builder::overshoot(
        size,
        bevel_edge_tool_builder::tangent_distance(
            size,
            bevel_edge_tool_builder::profile_wedge_angle_degrees(&segment.edge),
            round_profile,
        ),
    )
}

/// The one line a run whose overshoot was bounded puts on its own row: which ends were held, at
/// what size, and how much was picked - as the advisory the app puts into words.
///
/// `at_start` / `at_end` say whether each end took a bound shorter than the overshoot it replaced;
/// `first` is the run's first segment, whose FROM end is the start, and `last` its last, whose TO
/// end is the far one. `size` is the size that would have reached further, which is what the user
/// can change, and `edge_count` how much was picked, which is the other thing they can change.
///
/// One line for the run, not one per end. Two ends held back on a single picked edge are two ends
/// of one story, and a row that said the same sentence twice about two vertices a millimetre apart
/// would read as two problems. But WHICH ends were held is said, because on a longer run it is
/// usually only one of them, and so is how much was picked, because "the edge it was picked on" is
/// a lie about a run of ten. The size is named as the thing the user can change and the selection
/// as the other thing - the reading chooser lands on this same row.
pub fn held_back(
    at_start: bool,
    at_end: bool,
    first: &ChainEdge,
    last: &ChainEdge,
    size: f64,
    edge_count: i32,
) -> BevelAdvisory {
    if at_start && at_end {
        return BevelAdvisory::new(
            BevelAdvisoryCode::HeldBackAtBothEnds,
            vec![
                BevelAdvisoryArg::Vector3(first.from()),
                BevelAdvisoryArg::Vector3(last.to()),
                BevelAdvisoryArg::Double(size),
                BevelAdvisoryArg::Int(edge_count),
            ],
        );
    }

    BevelAdvisory::new(
        BevelAdvisoryCode::HeldBackAtOneEnd,
        vec![
            BevelAdvisoryArg::Vector3(if at_start { first.from() } else { last.to() }),
            BevelAdvisoryArg::Double(size),
            BevelAdvisoryArg::Int(edge_count),
        ],
    )
}

/// How square to the run a capping plane has to be before a BEAD will trim against it, as the
/// cosine the scan compares a face's agreement with: the lean at which the ring's slide onto that
/// plane stops fitting in the segment that has to absorb it. `size` is the size the tool will be
/// swept at; `segment` the end's own segment - the one whose strip the cap drags.
///
/// **The slide, exactly.** A scanned plane passes through the end's own vertex, so
/// [`BevelEndCap::slide_to_plane`] there is `-(n . offset) / (n . d)`, and the profile's offsets
/// are square to the run direction `d`. Writing the plane's lean off square as `theta`, that is at
/// most `reach * tan(theta)`, where reach is how far the cross section gets from the edge
/// ([`bevel_edge_tool_builder::cross_section_reach`] - about 1.41 sizes at a right angle). So the
/// slide grows without bound as the plane lies down, which is why a lean guard has to exist at all.
///
/// **What it has to fit in.** A strip runs between two rings, and `BevelChainSweep` refuses one
/// whose ends are dragged toward each other by more than the segment's own length - the strip has
/// turned inside out. HALF that length is the share taken here, because the other end of the same
/// segment is dragging it too: a single picked edge is one segment with a cap at each end, which is
/// the commonest run there is, and on a longer run the miter at the far end takes its own bite.
/// Solving `reach * tan(theta) <= length / 2` for `cos(theta)` gives what is returned.
///
/// **What that comes to.** On the 24 sided boss this was written for - a 1.57 mm chord round the
/// post, a 1 mm fillet - it is cos 29 degrees, and the wall facet leaning 75 (inside the flat 80
/// degree guard, which is what made the bug) loses to the square cap. On an 8 sided one, where the
/// chord is 4.6 mm, it is cos 58, and the facet leaning 45 still caps as it should. The threshold
/// therefore tracks the tessellation rather than being read off any one part.
///
/// **Why only a bead.** Not because a cutter cannot fold - it can, and is refused by the same rule
/// when it does - but because of what refusing the plane costs each of them. A bead that gives up a
/// plane squares off, which is safe and merely flat; and where another selected edge of its own
/// convexity departs from that end it overshoots instead, which is safe for the other reason - that
/// is 5.3's corner, where the two tools are MEANT to overlap and their union resolves it, so there
/// is nothing there to gouge. A cutter that gives up a plane OVERSHOOTS, and the overshoot gouges
/// the geometry past the corner that the cap existed to protect, so a cutter goes on trimming
/// against any plane that still means something and takes the fold refusal when it comes.
// C#'s `!(x > 0)` is true for NaN too, which the rewritten comparison would not be.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn minimum_bead_cap_agreement(size: f64, segment: &ChainEdge) -> f64 {
    let budget = (segment.to() - segment.from()).length() / 2.0;
    let reach = bevel_edge_tool_builder::cross_section_reach(
        size,
        bevel_edge_tool_builder::profile_wedge_angle_degrees(&segment.edge),
    );

    if !(reach > 0.0) {
        // Nothing is swept, so nothing can be dragged out of shape; leave the plane to the flat
        // guard.
        return 0.0;
    }

    if !(budget > 0.0) || reach.is_infinite() {
        // No room at all to absorb a slide, or no bound on the slide: only a plane square to the
        // run could be trusted, and the square cap is that plane.
        return 1.0;
    }

    reach / ((reach * reach) + (budget * budget)).sqrt()
}
