//! The band between two offsets of a pick's chains (the band gap's main module; its other parts
//! are `band_gap_in_turn.rs` and `band_gap_rounds_first.rs`). The size refusal, the chain ceiling,
//! the chord tolerance, the needle floor and the closed chains turned to start at a corner
//! ([`start_closed_chains_at_a_corner`]) and the build's funnel ([`build_unvalidated`]); the
//! build proper is `band_gap_build_within.rs` and the public builds `band_gap_entry_points.rs`.

use vector_math::{cs_math, Vector3};

use std::cell::RefCell;
use std::collections::HashSet;
use std::rc::Rc;

use super::band_gap_build_fault::BuildFault;
use super::band_gap_build_within::{build_within, BuildWithinArgs};
use super::chain_field_from_polyline::ChainFieldArgumentError;
use super::level_line_mesh_edits::LevelLineMeshError;
use super::pick_validation::PickArgumentError;
use super::seam_edges::FeatureEdgeTest;
use super::{
    band_side_tagging, chain_on_the_mesh, pick_validation, self_room, BandGapRefusal,
    BandGapResult, BandProfile, LevelLineMesh, LevelLineWork, SeamEdges, WorkBudget, WorkError,
};

/// How a band gap build leaves without a result: the errors a caller sees. A refusal (the
/// part said no) and a build that ran out of work are a [`BandGapResult`]; these are a caller's
/// bug, the caller's own cancel, or a defect in the build.
#[derive(Debug, Clone)]
pub enum BandGapBuildError {
    /// The pick's arguments are malformed.
    Arguments(PickArgumentError),
    /// The size is not a positive length.
    SizeNotPositive(f64),
    /// The caller cancelled.
    Cancelled,
    /// A chain's polyline builds no field.
    ChainField(ChainFieldArgumentError),
    /// The mesh is malformed or refused an edit.
    Mesh(LevelLineMeshError),
    /// Any other fault out of the build, by its message.
    Defect(String),
}

impl std::fmt::Display for BandGapBuildError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BandGapBuildError::Arguments(bad) => write!(f, "{bad:?}"),
            BandGapBuildError::SizeNotPositive(size) => write!(
                f,
                "a bevel's size is a length, so it must be positive (size {size})"
            ),
            BandGapBuildError::Cancelled => f.write_str("the band gap build was cancelled"),
            BandGapBuildError::ChainField(bad) => write!(f, "{bad}"),
            BandGapBuildError::Mesh(bad) => write!(f, "{bad:?}"),
            BandGapBuildError::Defect(message) => f.write_str(message),
        }
    }
}

impl std::error::Error for BandGapBuildError {}

impl From<PickArgumentError> for BandGapBuildError {
    fn from(error: PickArgumentError) -> Self {
        BandGapBuildError::Arguments(error)
    }
}

/// The build every public entry point funnels through, with its work budget and topology-only
/// switch: the pick split where its setbacks step, its closed chains started at a
/// corner, its seams no longer features, a size over a KNOWN room refused before anything is paid
/// for, and the build's own faults turned into the caller's outcomes - a cancel leaves as
/// [`BandGapBuildError::Cancelled`], a chain edge not on two triangles and a budget that ran out
/// as refusals carrying the working mesh as it stood. No input validation: the public builds
/// (`band_gap_entry_points`) validate first.
#[allow(clippy::too_many_arguments)]
pub fn build_unvalidated(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    is_feature_edge: Option<FeatureEdgeTest>,
    size: f64,
    profile: BandProfile,
    chord_tolerance: f64,
    snap: f64,
    known_room: Option<f64>,
    budget: Option<Rc<WorkBudget>>,
    topology_only: bool,
    seam_epsilon: f64,
    sharp_turn_degrees: f64,
    fillet_segments: i32,
) -> Result<BandGapResult, BandGapBuildError> {
    // Each edge keeps its own setback: a chain is cut where its edges' setbacks step. The vertices
    // inside a picked chain: where the split cuts one, the pieces meet at a STEP CORNER.
    let mut inside: HashSet<i32> = HashSet::new();
    for (k, chain) in chains.iter().enumerate() {
        if closed[k] {
            inside.extend(chain.iter().copied());
        } else {
            inside.extend(
                chain
                    .iter()
                    .skip(1)
                    .take(chain.len().saturating_sub(2))
                    .copied(),
            );
        }
    }

    let (chains, closed) = match chain_on_the_mesh::split_at_setback_steps(
        positions,
        indices,
        chains,
        closed,
        profile,
        sharp_turn_degrees,
        is_feature_edge.as_deref(),
    ) {
        Ok(split) => split,
        Err(bad) => {
            return Ok(BandGapResult {
                refusal: BandGapRefusal::ChainEdgeNotOnTwoTriangles,
                room: 0.0,
                offending_edge: (bad.from, bad.to),
                ..BandGapResult::default()
            })
        }
    };

    let chains = start_closed_chains_at_a_corner(&chains, Some(&closed));
    let step_corners: HashSet<i32> = (0..chains.len())
        .filter(|&k| !closed[k])
        .flat_map(|k| [chains[k][0], chains[k][chains[k].len() - 1]])
        .filter(|v| inside.contains(v))
        .collect();

    // Every build funnels here, so the pick's own seams stop being features ONCE.
    let is_feature_edge = SeamEdges::not_the_chains_seam(
        Some(positions),
        Some(&chains),
        Some(&closed),
        is_feature_edge,
        seam_epsilon,
    );
    let budget = match budget {
        Some(budget) => budget,
        None => Rc::new(
            WorkBudget::new(WorkBudget::default_limit(indices.len() as i32))
                .expect("the default limit is a positive count of work"),
        ),
    };
    if budget.cancellation_token.is_cancelled() {
        return Err(BandGapBuildError::Cancelled);
    }

    // A size the KNOWN room cannot hold is refused HERE, before anything is paid for - except for
    // ONE OPEN chain, whose handed-in room is read size-free and may be flowed past.
    let may_flow_past_the_room = chains.len() == 1 && !closed[0] && !topology_only;
    let mut known_room = known_room;
    if let Some(known) = known_room {
        if size > known && !may_flow_past_the_room {
            return Ok(refused_by_size(known));
        }
    }

    let mut handed_in_room: Option<f64> = None;
    if let Some(known) = known_room {
        if size > known {
            handed_in_room = Some(known);
            known_room = None;
        }
    }

    let _scope = LevelLineWork::begin(Some(Rc::clone(&budget)));

    // The working mesh, once the build has made it, so a refusal out of the middle of the build
    // can still hand back what the build was working on.
    let mut working: Option<Rc<RefCell<LevelLineMesh>>> = None;
    let mut unflowed_room: Option<f64> = None;
    let args = BuildWithinArgs {
        size,
        profile,
        chord_tolerance,
        snap,
        known_room,
        topology_only,
        seam_epsilon,
        sharp_turn_degrees,
        fillet_segments,
        step_corners: &step_corners,
    };
    match build_within(
        positions,
        indices,
        &chains,
        &closed,
        is_feature_edge,
        &args,
        &mut working,
        &mut unflowed_room,
    ) {
        Ok(mut built) => {
            if built.built() && unflowed_room.is_some() {
                // The room the caller handed in is the one it would have told the user, so it wins.
                built.unflowed_room = handed_in_room.or(unflowed_room);
            }

            Ok(match handed_in_room {
                Some(handed) if built.refusal == BandGapRefusal::Size => refused_by_size(handed),
                _ => built,
            })
        }
        // The caller asked for this, so it leaves as cancellation and never as a refusal.
        Err(BuildFault::Work(WorkError::Cancelled)) => Err(BandGapBuildError::Cancelled),
        // The mesh under the pick is not what a chain needs it to be: a statement about the part.
        Err(BuildFault::ChainEdge(bad)) => Ok(BandGapResult {
            refusal: BandGapRefusal::ChainEdgeNotOnTwoTriangles,
            room: 0.0,
            offending_edge: (bad.from, bad.to),
            mesh: working,
            ..BandGapResult::default()
        }),
        // The half-built mesh comes back as it stood when the work ran out - NOT a result.
        Err(BuildFault::Work(WorkError::BudgetExceeded(_))) => Ok(BandGapResult {
            refusal: BandGapRefusal::WorkBudgetExceeded,
            room: 0.0,
            mesh: working,
            phase: budget.exhausted_in.get(),
            work_spent: budget.spent.get(),
            work_limit: budget.limit,
            ..BandGapResult::default()
        }),
        Err(BuildFault::Error(error)) => Err(error),
    }
}

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

/// The chord tolerance a build at this size uses: the one asked
/// for, or size / 20 when none was.
pub fn chord_tolerance_of(size: f64, chord_tolerance: f64) -> f64 {
    if chord_tolerance > 0.0 {
        chord_tolerance
    } else {
        size / 20.0
    }
}

/// The height under which a build's working mesh calls a triangle a
/// needle (`LevelLineMesh::needle_floor`). Named so a measurement can read the very floor a build
/// uses.
pub fn needle_floor_of(snap: f64, tolerance: f64) -> f64 {
    cs_math::max_f64(4.0 * snap, 1e-3 * tolerance)
}

/// Each closed chain holding a corner of the pick, rotated to
/// START there. A closed chain's rail parameter runs from its first vertex round to it, so its seam
/// sits at that vertex; a seam anywhere but a corner lies INSIDE a stretch of the rails, which then
/// reads `RailIsNotMonotone`. Only the start moves - the chain, its direction and its index do not.
/// A chain with no `closed` entry is read as open.
pub fn start_closed_chains_at_a_corner(
    chains: &[Vec<i32>],
    closed: Option<&[bool]>,
) -> Vec<Vec<i32>> {
    let Some(closed) = closed else {
        return chains.to_vec();
    };

    if !band_side_tagging::both_sided_members_build() || !closed.iter().any(|&c| c) {
        return chains.to_vec();
    }

    let corners: HashSet<i32> = pick_validation::corners_of(chains, closed, true)
        .iter()
        .map(|corner| corner.vertex)
        .collect();

    if corners.is_empty() {
        return chains.to_vec();
    }

    chains
        .iter()
        .enumerate()
        .map(|(k, chain)| {
            let at = if closed.get(k).copied().unwrap_or(false) {
                chain.iter().position(|v| corners.contains(v)).unwrap_or(0)
            } else {
                0
            };

            if at == 0 {
                chain.clone()
            } else {
                chain[at..].iter().chain(&chain[..at]).copied().collect()
            }
        })
        .collect()
}
