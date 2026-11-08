// The level-line route's offer for a rim whose blend cannot be built at a cove-to-round corner:
// where the one-chain fill refused the rim because a rail ran backward next to a vertex where its
// crease flips between a cove and a round, the rim is built again cut into runs at THOSE flips
// only (`chain_on_the_mesh::sense_flips_near`, `WorkBudget::sense_flips_to_cut`), each run ending
// flush on the other's side face there (`flip_corner_clip`). Every other flip stays blended. Like
// the crest, the attempt is an offer, never a new way to fail: a fault anywhere in it leaves the
// one-chain refusal standing, only a cancel stops the pass, and its answer is taken only where it
// closes and does not cross itself. `level_line_build` asks it.

use std::rc::Rc;

use crate::bevel::level_line::band_gap::BandGapBuildError;
use crate::bevel::level_line::band_gap_entry_points::{build_pick, BandGapBuildOptions};
use crate::bevel::level_line::chain_on_the_mesh::{flips_cut_more, sense_flips_near};
use crate::bevel::level_line::gap_fill::{fill_with_budget, GapFillError};
use crate::bevel::level_line::seam_edges::FeatureEdgeTest;
use crate::bevel::level_line::{
    BandGapResult, BandProfile, GapFillRefusal, GapFillResult, WorkBudget,
};
use crate::bevel::part::answer_mesh::the_mesh_of;
use crate::bevel::part::self_intersection_check::the_answer_intersects_itself_where_the_source_did_not as crosses_itself;
use crate::bevel::surface::surface_mesh_closure;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use super::level_line::LevelLineRouteError;
use super::level_line_kernel_handoff::the_kept_triangles;
use super::level_line_memo::THE_ROUTES_SNAP;

/// What the one-chain build left that the offer reads: its band and its refused fill.
pub struct TheOneChainRefusal<'a> {
    /// The one-chain band.
    pub gap: &'a BandGapResult,
    /// Its fill, refused.
    pub filled: &'a GapFillResult,
}

/// The pick the offer builds again.
pub struct ThePick<'a> {
    /// The part's positions.
    pub positions: &'a [Vector3],
    /// The part's triangles.
    pub indices: &'a [(i32, i32, i32)],
    /// The picked chains.
    pub chains: &'a [Vec<i32>],
    /// Whether each chain is closed.
    pub closed: &'a [bool],
    /// The part's feature edges.
    pub is_feature_edge: FeatureEdgeTest,
}

/// The flips next to where the one-chain fill refused the rim because a rail ran backward:
/// within the band's largest setback of the refusal's vertex. Empty for any other refusal, and
/// where the band's reading fails (it cannot, once the band built: every chain edge stands on
/// two triangles).
pub fn the_flips_to_cut(refused: &TheOneChainRefusal, pick: &ThePick) -> Vec<i32> {
    let (gap, filled) = (refused.gap, refused.filled);
    if filled.filled()
        || filled.refusal != GapFillRefusal::RailIsNotMonotone
        || !gap.built()
        || filled.offending_vertex < 0
    {
        return Vec::new();
    }

    let Some(at) = filled
        .mesh
        .as_ref()
        .map(|mesh| mesh.borrow().position(filled.offending_vertex))
    else {
        return Vec::new();
    };
    let reach = gap
        .chains
        .iter()
        .map(|chain| chain.max_setback)
        .fold(0.0, f64::max);
    sense_flips_near(
        pick.positions,
        pick.indices,
        pick.chains,
        pick.closed,
        Some(&*pick.is_feature_edge),
        at,
        reach,
    )
    .unwrap_or_default()
}

/// The run of the cut build a refusal at `at` stands on: the fill's offending chain where it names
/// one, else the run whose own (unclipped) value is least there; ties go to the lower index.
fn the_refused_run(runs: &BandGapResult, offending_chain: i32, at: Vector3) -> Option<usize> {
    if offending_chain >= 0 && (offending_chain as usize) < runs.chains.len() {
        return Some(offending_chain as usize);
    }

    let mut best: Option<(f64, usize)> = None;
    for (k, chain) in runs.chains.iter().enumerate() {
        let Ok(value) = chain.unclipped_value_at(at) else {
            continue;
        };
        // A NaN reading names no run: it never wins.
        if value.is_nan() {
            continue;
        }
        if best.is_none_or(|(least, _)| value < least) {
            best = Some((value, k));
        }
    }

    best.map(|(_, k)| k)
}

/// Of a refused run's two ends, the one nearer the refusal at `at` (the first end on a tie), where
/// it is one of the rim's flips (`candidates`, each with its position) not yet `cut`; `None` where
/// that end is no such flip, and for a closed run, whose first point is only where the pick
/// happened to start it, no end at all.
pub fn the_next_flip(
    run_points: &[Vector3],
    run_closed: bool,
    at: Vector3,
    candidates: &[(i32, Vector3)],
    cut: &[i32],
) -> Option<i32> {
    if run_closed || run_points.is_empty() {
        return None;
    }

    let (first, last) = (run_points[0], run_points[run_points.len() - 1]);
    let nearer = if (last - at).length() < (first - at).length() {
        last
    } else {
        first
    };
    candidates
        .iter()
        .find(|&&(flip, position)| position == nearer && !cut.contains(&flip))
        .map(|&(flip, _)| flip)
}

/// The rim built again cut at `flips`, under `budget`, at `size`: its answer where it builds,
/// fills, closes and does not cross itself; `None` otherwise, whatever went wrong but a cancel.
/// Where the cut build's fill refuses on a run whose end nearer the refusal is a flip not yet cut,
/// that flip is the next one the blend cannot be built at: it is added and the rim built again,
/// at most once per flip the rim has, and only while the new flips cut more than the last.
#[allow(clippy::too_many_arguments)]
pub fn the_rim_cut_at_its_flips(
    pick: &ThePick,
    mut flips: Vec<i32>,
    size: f64,
    profile: BandProfile,
    fillet_segments: i32,
    seam_epsilon: f64,
    sharp_turn_degrees: f64,
    budget: Rc<WorkBudget>,
    cancellation_token: &CancelToken,
) -> Result<Option<Mesh>, LevelLineRouteError> {
    let all = sense_flips_near(
        pick.positions,
        pick.indices,
        pick.chains,
        pick.closed,
        Some(&*pick.is_feature_edge),
        Vector3::ZERO,
        f64::INFINITY,
    )
    .unwrap_or_default();
    // The flips the last build was cut at: none, before the first.
    let mut last: Vec<i32> = Vec::new();
    while !flips.is_empty() && flips.len() <= all.len() {
        // Cut at these flips, the pick as the build splits it: where they cut nothing more than the
        // last set did (a closed rim needs two cuts, so one named flip with no setback step cuts
        // nothing; a flip no crease runs into cuts nothing), the build would only repeat the last
        // one, so the offer ends here.
        if !flips_cut_more(
            pick.positions,
            pick.indices,
            pick.chains,
            pick.closed,
            profile,
            sharp_turn_degrees,
            Some(&*pick.is_feature_edge),
            &last,
            &flips,
        ) {
            return Ok(None);
        }
        *budget.sense_flips_to_cut.borrow_mut() = flips.clone();
        let runs = build_pick(
            pick.positions,
            pick.indices,
            pick.chains,
            pick.closed,
            Some(pick.is_feature_edge.clone()),
            size,
            profile,
            &BandGapBuildOptions {
                chord_tolerance: 0.0,
                snap: THE_ROUTES_SNAP,
                known_room: None,
                budget: Some(Rc::clone(&budget)),
                seam_epsilon,
                sharp_turn_degrees,
                fillet_segments,
            },
        );
        let runs = match runs {
            Ok(runs) => runs,
            Err(BandGapBuildError::Cancelled) => return Err(LevelLineRouteError::Cancelled),
            Err(_) => return Ok(None),
        };
        if !runs.built() {
            return Ok(None);
        }

        let filled =
            match fill_with_budget(&runs, profile, fillet_segments, Some(Rc::clone(&budget))) {
                Ok(filled) => filled,
                Err(GapFillError::Cancelled) => return Err(LevelLineRouteError::Cancelled),
                Err(_) => return Ok(None),
            };
        let Some(handle) = filled.mesh.as_ref() else {
            return Ok(None);
        };
        if !filled.filled() {
            // The next flip the blend cannot be built at, read off the refusal's topology, not a
            // distance: the run the refusal stands on (named by the fill, or else the run whose
            // own value is least there) and the end of that run nearer the refusal, where it is a
            // flip not yet cut.
            let at = (filled.offending_vertex >= 0)
                .then(|| handle.borrow().position(filled.offending_vertex));
            let candidates: Vec<(i32, Vector3)> = all
                .iter()
                .map(|&flip| (flip, pick.positions[flip as usize]))
                .collect();
            let next = at.and_then(|at| {
                the_refused_run(&runs, filled.offending_chain, at).and_then(|run| {
                    let chain = &runs.chains[run];
                    the_next_flip(chain.points(), chain.is_closed, at, &candidates, &flips)
                })
            });
            match next {
                Some(vertex) => {
                    last = flips.clone();
                    flips.push(vertex);
                    continue;
                }
                None => return Ok(None),
            }
        }

        let mesh = {
            let mesh = handle.borrow();
            the_mesh_of(&mesh.vertices, &the_kept_triangles(&mesh))
        };
        if !surface_mesh_closure::is_closed(&mesh).0
            || crosses_itself(&mesh, pick.positions, pick.indices, cancellation_token)?
        {
            return Ok(None);
        }

        return Ok(Some(mesh));
    }

    Ok(None)
}
