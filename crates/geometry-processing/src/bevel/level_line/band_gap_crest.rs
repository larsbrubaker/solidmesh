//! The crest build's entry: a pick of two straight convex chains on one flat face, refused by
//! size because one band's front meets the other's across that face, built instead with the two
//! rounds meeting in a crest ([`super::gap_fill_crest`]).
//!
//! It runs only after the band build has refused the pick by size, so a pick that builds today
//! never reaches it. It asks the band build again with the size itself handed in as the room, so
//! the build goes on past its room check to the cut, the band's delete and the loops (it still
//! refuses at the owned-loop check, the shared face being swallowed whole, but carries the mesh
//! and the loops), and hands that gap to the crest fill. `None` wherever the pick or its gap is
//! not this shape: the caller keeps today's refusal, word for word. In a pick of more chains the
//! crest is built for the two the refusal names and the rest on its result, one at a time
//! ([`build_crest_in_pick`]); any of them refusing leaves the pick's refusal as it was.

use std::collections::HashSet;
use std::rc::Rc;

use vector_math::Vector3;

use super::band_gap::BandGapBuildError;
use super::band_gap_entry_points::{build_pick, BandGapBuildOptions};
use super::band_gap_in_turn::{
    build_open_chains_in_turn, chain_after_a_build, feature_edges_after_a_build, BudgetFor,
    BuildAndFillError,
};
use super::band_gap_rounds_first::kept;
use super::gap_fill_crest::{self, CrestEdge, CrestFillInput};
use super::seam_edges::FeatureEdgeTest;
use super::{level_line_routing, triangle_edges};
use super::{BandGapRefusal, BandProfile, GapFillResult, RoomReadingKind, RoomSource};

/// The least tolerance a straight line or a shared plane is read to, relative to the chain's
/// length; the build's own seam epsilon and snap widen it, so a part stored in single precision
/// still reads as straight and flat.
const FLAT_RELATIVE: f64 = 1e-9;

/// The unit normal of triangle `t`, wound outward.
fn normal_of(positions: &[Vector3], t: (i32, i32, i32)) -> Vector3 {
    let (a, b, c) = (
        positions[t.0 as usize],
        positions[t.1 as usize],
        positions[t.2 as usize],
    );
    Vector3::normalized((b - a).cross(c - a))
}

/// A chain's two faces at its first edge, as (outward normal, the far corner of that triangle).
fn faces_at(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    a: i32,
    b: i32,
) -> Option<[(Vector3, Vector3); 2]> {
    let on: Vec<(Vector3, Vector3)> = indices
        .iter()
        .filter_map(|&t| {
            let corners = [t.0, t.1, t.2];
            if !corners.contains(&a) || !corners.contains(&b) {
                return None;
            }
            let far = corners.into_iter().find(|&v| v != a && v != b)?;
            Some((normal_of(positions, t), positions[far as usize]))
        })
        .collect();
    match on.as_slice() {
        [first, second] => Some([*first, *second]),
        _ => None,
    }
}

/// The crest build of `pick` at `size`, or `None` where the pick is not two straight convex
/// chains on one flat face, or its gap is not the crest fill's.
#[allow(clippy::too_many_arguments)]
pub fn build_crest(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    pick: &[Vec<i32>],
    closed: &[bool],
    is_feature_edge: Option<FeatureEdgeTest>,
    size: f64,
    profile: BandProfile,
    options: &BandGapBuildOptions,
) -> Result<Option<GapFillResult>, BandGapBuildError> {
    if pick.len() != 2
        || closed.iter().any(|&c| c)
        || profile != BandProfile::Fillet
        || pick.iter().any(|chain| chain.len() < 2)
    {
        return Ok(None);
    }

    // Each chain straight, with the same two face planes along every one of its edges.
    let mut read: Vec<(CrestEdge, [(Vector3, Vector3); 2])> = Vec::new();
    let mut tolerance = options.seam_epsilon.max(options.snap);
    for chain in pick {
        let start = positions[chain[0] as usize];
        let end = positions[chain[chain.len() - 1] as usize];
        let length = (end - start).length();
        if length <= 0.0 {
            return Ok(None);
        }
        tolerance = tolerance.max(FLAT_RELATIVE * length);
        let d = Vector3::normalized(end - start);
        let off_line = |&v: &i32| {
            let offset = positions[v as usize] - start;
            (offset - d * offset.dot(d)).length() > tolerance
        };
        if chain.iter().any(off_line) {
            return Ok(None);
        }
        let Some(faces) = faces_at(positions, indices, chain[0], chain[1]) else {
            return Ok(None);
        };
        let turn_tolerance = tolerance / length;
        let same_planes = |other: [(Vector3, Vector3); 2]| {
            let close = |a: Vector3, b: Vector3| (a - b).length() <= turn_tolerance;
            (close(faces[0].0, other[0].0) && close(faces[1].0, other[1].0))
                || (close(faces[0].0, other[1].0) && close(faces[1].0, other[0].0))
        };
        for step in chain.windows(2).skip(1) {
            match faces_at(positions, indices, step[0], step[1]) {
                Some(other) if same_planes(other) => {}
                _ => return Ok(None),
            }
        }
        let edge = CrestEdge {
            start,
            end,
            other_normal: Vector3::ZERO,
        };
        read.push((edge, faces));
    }

    // The shared face: a face plane of chain 0 that is a face plane of chain 1; each chain's other
    // face is its round's other side, and each edge is convex (its other face falls away below
    // the shared one).
    let scale = (read[0].0.end - read[0].0.start).length();
    let mut shared = None;
    for (i, (n0, _)) in read[0].1.iter().enumerate() {
        for (k, (n1, _)) in read[1].1.iter().enumerate() {
            let same_plane = (*n0 - *n1).length() <= tolerance / scale
                && (read[1].0.start - read[0].0.start).dot(*n0).abs() <= tolerance;
            if same_plane && shared.is_none() {
                shared = Some((*n0, i, k));
            }
        }
    }
    let Some((n, i, k)) = shared else {
        return Ok(None);
    };
    for (chain, other) in [(0, 1 - i), (1, 1 - k)] {
        let (m, far) = read[chain].1[other];
        if (far - read[chain].0.start).dot(n) >= 0.0 {
            return Ok(None);
        }
        read[chain].0.other_normal = m;
    }

    let gap = build_pick(
        positions,
        indices,
        pick,
        closed,
        is_feature_edge,
        size,
        profile,
        &BandGapBuildOptions {
            known_room: Some(size),
            ..options.clone()
        },
    )?;
    // Handed the size as its room, the build cuts, deletes the band and walks the loops, then
    // refuses by size at the owned-loop check because the shared face is swallowed whole. Any
    // other answer (a budget run out, a half-cut mesh) is not the gap the crest fill reads.
    if gap.refusal != BandGapRefusal::Size || gap.loops.is_empty() {
        return Ok(None);
    }
    let Some(mesh) = gap.mesh.as_ref() else {
        return Ok(None);
    };
    gap_fill_crest::fill(&CrestFillInput {
        mesh,
        loops: &gap.loops,
        edges: [&read[0].0, &read[1].0],
        face_normal: n,
        radius: size,
        segments: options.fillet_segments.max(1),
        seam_epsilon: options.seam_epsilon,
        snap: options.snap,
        chain_ends: [
            [pick[0][0], pick[0][pick[0].len() - 1]],
            [pick[1][0], pick[1][pick[1].len() - 1]],
        ],
    })
    .map_err(BandGapBuildError::Mesh)
}

/// The crest for the two chains a pick's `ChainToChain` refusal names, then every other chain of
/// the pick on what it left, one at a time and each as it is alone
/// ([`build_open_chains_in_turn`]). `None` - the pick keeping today's refusal - wherever the pair
/// takes no crest or any later chain refuses. A pick of exactly the two is [`build_crest`].
#[allow(clippy::too_many_arguments)]
pub fn build_crest_in_pick(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    pick: &[Vec<i32>],
    closed: &[bool],
    is_feature_edge: Option<FeatureEdgeTest>,
    size: f64,
    profile: BandProfile,
    options: &BandGapBuildOptions,
    room_set_by: &RoomSource,
) -> Result<Option<GapFillResult>, BuildAndFillError> {
    if level_line_routing::the_crest_attempt_fails() {
        return Err(
            BandGapBuildError::Defect("the crest attempt was made to fail".to_string()).into(),
        );
    }
    if room_set_by.kind != RoomReadingKind::ChainToChain {
        return Ok(None);
    }
    let (k, j) = (room_set_by.chain, room_set_by.other_chain);
    let named = |c: i32| c >= 0 && (c as usize) < pick.len();
    if pick.len() < 2 || !named(k) || !named(j) || k == j {
        return Ok(None);
    }
    let (k, j) = (k as usize, j as usize);
    let pair = [pick[k].clone(), pick[j].clone()];
    let Some(crest) = build_crest(
        positions,
        indices,
        &pair,
        &[closed[k], closed[j]],
        is_feature_edge.clone(),
        size,
        profile,
        options,
    )?
    else {
        return Ok(None);
    };
    let rest: Vec<usize> = (0..pick.len()).filter(|&c| c != k && c != j).collect();
    if rest.is_empty() {
        return Ok(Some(crest));
    }
    let Some(mesh) = crest.mesh.as_ref() else {
        return Ok(None);
    };

    // The rest of the pick read on the crest's part: its chains carried onto it, the input's own
    // feature edges keeping their word and the crest's edges features where they fold.
    let (built, built_indices) = {
        let mesh = mesh.borrow();
        (mesh.vertices.clone(), kept(&mesh))
    };
    let input_edges: HashSet<(i32, i32)> = indices
        .iter()
        .flat_map(|&t| triangle_edges::corners(t))
        .map(|(a, b)| triangle_edges::key(a, b))
        .collect();
    let feature = feature_edges_after_a_build(
        built.clone(),
        built_indices.clone(),
        positions.len() as i32,
        input_edges,
        is_feature_edge.unwrap_or_else(|| Rc::new(|_: i32, _: i32| false)),
    );
    let carried: Vec<Vec<i32>> = rest
        .iter()
        .map(|&c| chain_after_a_build(&pick[c], closed[c], &built, &built_indices))
        .collect();
    let rest_closed: Vec<bool> = rest.iter().map(|&c| closed[c]).collect();
    // The rest of the pick spends the attempt's own allowance, so its cap and its cancel hold.
    let shared_budget = options.budget.clone();
    let budget_for = move |_: i32| {
        shared_budget
            .clone()
            .expect("asked only where the attempt carries a budget")
    };
    let in_turn = build_open_chains_in_turn(
        &built,
        &built_indices,
        &carried,
        Some(feature),
        size,
        profile,
        options.fillet_segments,
        None,
        options.snap,
        options.seam_epsilon,
        options.sharp_turn_degrees,
        options
            .budget
            .as_ref()
            .map(|_| &budget_for as BudgetFor<'_>),
        Some(&rest_closed),
    )?;
    Ok(in_turn.filled().then_some(in_turn))
}
