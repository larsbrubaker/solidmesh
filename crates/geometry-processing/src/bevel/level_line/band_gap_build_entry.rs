//! Entering the band gap's build: the pick's arguments checked, step 1's setback readings made
//! (`band_gap_setback_fields.rs`), and the pass handed to the build proper
//! ([`build_on_the_walk`], `band_gap_build_within.rs`) on the ball's walk where the ball takes it
//! (`rolling_ball_setbacks.rs`), or answered here where it does not. The funnel
//! ([`band_gap::build_unvalidated`](super::band_gap::build_unvalidated)) calls [`build_within`].
//!
//! - **Strict:** an outer fillet pass the ball is asked to take and cannot (`with_the_sides` or
//!   `a_rayed_end_that_differs` names why) is refused as `BandGapRefusal::RollingBall` with that
//!   reason, before anything is cut; it is never built on the level line's own setbacks. A pass
//!   of one chain is refused by its size instead where the ball's own room, read at a smaller
//!   size it walks, explains the refusal (`rolling_ball_size_room`, which reads it through
//!   `build_on_the_walk` stopping at the room). A
//!   chamfer is never asked: no ball defines it, and it builds exactly as with the switch off.
//! - **No crease:** a pass of one chain the ball would be asked to take, where the faces either
//!   side of the chain lie in one plane at every chain point (every unit setback within
//!   [`NO_CREASE_UNIT_SETBACK`]), is refused as `ChainHasNoCreaseToBevel` before the ball walks.
//!   There is nothing to round, so that is the user's answer whatever the construction: a ball on
//!   a flat face touches it at one point and has no two sides to sit between. With the switch off
//!   the level line reaches the same name in the build, from a cut that takes nothing off.

use std::cell::RefCell;
use std::rc::Rc;

use vector_math::Vector3;

use super::band_gap::{BandGapBuildError, BuildWithinArgs};
use super::band_gap_build_fault::BuildFault;
use super::band_gap_build_within::build_on_the_walk;
use super::band_gap_setback_fields::PassSetbacks;
use super::rolling_ball_size_room::{self, refused_by_the_ball};
use super::seam_edges::FeatureEdgeTest;
use super::{pick_validation, rolling_ball_setbacks};
use super::{BandGapRefusal, BandGapResult, LevelLineMesh};

/// The largest unit setback (`tan` of half the angle between the faces' normals) read as no
/// crease. Two triangles of one plane carry normals equal only to their rounding: a cosine a few
/// bits under 1 reads as an angle of about 2e-8 radians, a unit setback of about 1e-8. A crease
/// of 2e-7 radians (about 1e-5 degrees) is no edge anyone picks to round.
pub const NO_CREASE_UNIT_SETBACK: f64 = 1e-7;

/// Whether a chain's unit setbacks say no crease stands under any of its points. A NaN setback (a
/// broken normal) is not "no crease": it goes on to be refused where it breaks.
fn has_no_crease(unit_setbacks: &[f64]) -> bool {
    unit_setbacks.iter().all(|&u| u <= NO_CREASE_UNIT_SETBACK)
}

/// The build proper. `working` receives the working mesh as soon as it is made, so a refusal out
/// of the middle of the build can still hand it back; `unflowed_room` receives the plain
/// tagging's room where the flowed tagging is cut instead (see
/// [`tag_flowed_where_it_builds`](super::narrow_face_flow::tag_flowed_where_it_builds)).
#[allow(clippy::too_many_arguments)]
pub fn build_within(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    is_feature_edge: Option<FeatureEdgeTest>,
    args: &BuildWithinArgs,
    working: &mut Option<Rc<RefCell<LevelLineMesh>>>,
    unflowed_room: &mut Option<f64>,
) -> Result<BandGapResult, BuildFault> {
    let BuildWithinArgs {
        size,
        profile,
        known_room,
        topology_only,
        sharp_turn_degrees,
        side_regions,
        ..
    } = *args;

    if chains.iter().any(|one| one.len() < 2) {
        return Err(BuildFault::Error(BandGapBuildError::Arguments(
            pick_validation::PickArgumentError::ChainTooShort,
        )));
    }

    if size.is_nan() || size <= 0.0 {
        return Err(BuildFault::Error(BandGapBuildError::SizeNotPositive(size)));
    }

    // 1. SETBACK, per chain.
    let setbacks = PassSetbacks::of(
        positions,
        indices,
        chains,
        closed,
        sharp_turn_degrees,
        profile,
    )?;
    // The field at the size takes the rolling ball's setbacks where its switch is on, this is the
    // outer pass and no topology-only probe, and the ball takes the whole pass, which is refused
    // where the ball cannot take it; probes, the room and every other size keep the level line's
    // own, and only the outer pass writes the route.
    let pass = rolling_ball_setbacks::PassScope::enter();
    let probe = topology_only || !pass.is_outer();
    // A pass of one chain with no crease under it is refused by that name before the ball is
    // asked: there is nothing to round, whatever would round it.
    if chains.len() == 1
        && rolling_ball_setbacks::asks_the_ball(probe, profile)
        && has_no_crease(&setbacks.unit_setbacks[0])
    {
        return Ok(BandGapResult {
            refusal: BandGapRefusal::ChainHasNoCreaseToBevel,
            room: 0.0,
            ..BandGapResult::default()
        });
    }
    let walk = |s: f64| {
        rolling_ball_setbacks::walked_for_the_build(
            positions,
            indices,
            chains,
            closed,
            is_feature_edge.as_deref(),
            side_regions,
            s,
            profile,
            &setbacks,
            probe,
        )
    };
    match walk(size) {
        Ok(ball_pass) => build_on_the_walk(
            positions,
            indices,
            chains,
            closed,
            is_feature_edge.clone(),
            args,
            working,
            unflowed_room,
            &setbacks,
            ball_pass,
            false,
        ),
        // A refused pass of one chain names the ball's own room where it has one
        // (`rolling_ball_size_room`); nothing is cut either way.
        Err(why) if chains.len() == 1 && known_room.is_none() => {
            let read = rolling_ball_size_room::ReadAt {
                positions,
                indices,
                chains,
                closed,
                is_feature_edge: &is_feature_edge,
                args,
                setbacks: &setbacks,
            };
            let mut walk_at = |s| walk(s).and_then(|p| p.ok_or_else(|| String::new().into()));
            rolling_ball_size_room::refusal(size, why, &mut walk_at, &read)
        }
        Err(refused) => Ok(refused_by_the_ball(refused.why)),
    }
}
