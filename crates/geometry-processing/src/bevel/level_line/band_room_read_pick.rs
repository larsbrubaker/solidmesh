//! The pick's room as a reading (a part of the band room, whose main module is `band_room.rs`):
//! the budget installed for the reading, and the reading's faults turned into readings.

use std::rc::Rc;

use solidmesh_math::Vector3;

use super::band_room::lengths_of;
use super::band_room_pair_room::room_of_pick;
use super::band_side_tagging_ends::SideTaggingError;
use super::seam_edges::FeatureEdgeTest;
use super::{BandProfile, LevelLineWork, LevelLineWorkPhase, RoomReading, WorkBudget, WorkError};

/// The pick's room, with the same work budget and the same
/// refusals-by-name a single chain's reading has (see [`room_of_pick`] for what the number means).
///
/// A run-out budget is the unread reading (`read` false, room NaN, the phase it ran out in), a chain
/// edge not on two triangles is a reading of room zero naming that edge, and the caller's
/// cancellation is the one `Err`.
#[allow(clippy::too_many_arguments)]
pub fn read_room_of_pick(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    is_feature_edge: Option<FeatureEdgeTest>,
    profile: BandProfile,
    snap: f64,
    budget: &Rc<WorkBudget>,
    name_every_candidate_under: f64,
    seam_epsilon: f64,
    sharp_turn_degrees: f64,
) -> Result<RoomReading, WorkError> {
    let _scope = LevelLineWork::begin(Some(Rc::clone(budget)));
    LevelLineWork::enter(LevelLineWorkPhase::Room);
    let pick = room_of_pick(
        positions,
        indices,
        chains,
        closed,
        is_feature_edge,
        profile,
        snap,
        name_every_candidate_under,
        seam_epsilon,
        sharp_turn_degrees,
        0,
    );
    match pick {
        Ok(pick) => Ok(RoomReading {
            read: true,
            room: pick.room,
            room_set_by: pick.set_by,
            candidates: pick.candidates,
            chain_lengths: lengths_of(positions, chains, closed),
            chain_divides_sides: pick.undivided_chain < 0,
            offending_chain: pick.undivided_chain,
            work_spent: budget.spent.get(),
            work_limit: budget.limit,
            ..RoomReading::default()
        }),
        Err(SideTaggingError::Work(WorkError::Cancelled)) => Err(WorkError::Cancelled),
        Err(SideTaggingError::ChainEdge(bad)) => Ok(RoomReading {
            read: true,
            room: 0.0,
            chain_edge_stands_on_two_triangles: false,
            offending_edge: (bad.from, bad.to),
            work_spent: budget.spent.get(),
            work_limit: budget.limit,
            ..RoomReading::default()
        }),
        Err(SideTaggingError::Work(WorkError::BudgetExceeded(_))) => Ok(RoomReading {
            read: false,
            room: f64::NAN,
            phase: budget.exhausted_in.get(),
            work_spent: budget.spent.get(),
            work_limit: budget.limit,
            ..RoomReading::default()
        }),
    }
}
