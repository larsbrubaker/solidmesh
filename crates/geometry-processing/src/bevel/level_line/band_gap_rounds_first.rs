//! The band built rounds first (a part of the band gap, whose main module is `band_gap.rs`): the
//! kept triangles as index triples ([`kept`]), the concave
//! chain's surviving piece ([`surviving_piece`]), the crease walk on the end face
//! ([`crease_on_the_end_face`]), a triangle's oriented plane ([`plane_of`], [`same_plane`]), the
//! chains' senses ([`chain_senses`]), the knife tips that make a pick build rounds first
//! ([`rounds_first_tips`], [`takes_rounds_first`]), the face they end on ([`the_tips_end_face`]),
//! the chains pass A builds ([`the_rounds_chains`]), the rounds' band ([`the_rounds_band`]) and the
//! smooth joints over it ([`smooth_joints_over_the_rounds`]), and the build itself: pass A alone
//! ([`build_the_rounds`]) and the whole pick ([`build_rounds_first`]).

use std::collections::{HashMap, HashSet};

use indexmap::IndexMap;

use vector_math::{ieee_math, Vector3};

use super::band_gap_after_the_rounds::{
    creases_after_the_rounds_closed, feature_edges_after_the_rounds,
};
use super::band_gap_entry_points::{build_pick, BandGapBuildOptions};
use super::band_gap_in_turn::{BudgetFor, BuildAndFillError};
use super::end_face_sense::{self, COPLANAR_TOLERANCE};
use super::gap_fill;
use super::open_gap_fill::set_ends_site;
use super::seam_edges::FeatureEdgeTest;
use super::triangle_edges::{self, StandingMap};
use super::{
    BandProfile, ChainField, GapFillRefusal, GapFillResult, LevelLineMesh, RoomReadingKind,
    RoomSource,
};

/// The mesh's live triangles as index triples, in triangle order.
pub fn kept(mesh: &LevelLineMesh) -> Vec<(i32, i32, i32)> {
    (0..mesh.triangle_count())
        .filter(|&t| !mesh.is_removed(t))
        .map(|t| {
            let face = mesh.triangles()[t as usize];
            (face.a, face.b, face.c)
        })
        .collect()
}

/// A concave chain oriented to end at its knife tip, carried onto the
/// rounds' mesh: from its far end, through any vertices the rounds' cut laid on its edges, to the
/// vertex where it enters the rounds' band (the band took the tip). `None` where the piece cannot
/// be walked, or the band never reached the chain. A vertex missing from `neighbours` is a walk
/// that cannot be made, as in [`super::band_gap_in_turn::walk_along`].
pub fn surviving_piece(
    to_tip: &[i32],
    at: &[Vector3],
    is_crease: &dyn Fn(i32, i32) -> bool,
    neighbours: &HashMap<i32, Vec<i32>>,
) -> Option<Vec<i32>> {
    if !neighbours.contains_key(&to_tip[0]) {
        return None;
    }

    // Stepped along each edge's straight line from vertex to nearest vertex ahead, while the step
    // is still crease: the tip itself may stay (it lies in the end face the round grew), so the
    // piece ends where the crease leaves the line - the rail's entry - not where a vertex is
    // missing.
    let mut piece = vec![to_tip[0]];
    let mut i = 0;
    while i + 1 < to_tip.len() {
        let start = at[to_tip[i] as usize];
        let along = at[to_tip[i + 1] as usize] - start;
        let length = along.length();
        let direction = along / length;
        let on_the_line = 1e-7 * length;
        let mut here = piece[piece.len() - 1];
        let mut reached = (at[here as usize] - start).dot(direction);
        while here != to_tip[i + 1] {
            let mut next = -1;
            let mut next_reach = f64::INFINITY;
            for &n in neighbours.get(&here)? {
                let offset = at[n as usize] - start;
                let t = offset.dot(direction);
                if t > reached + on_the_line
                    && t <= length + on_the_line
                    && t < next_reach
                    && (offset - direction * t).length() <= on_the_line
                {
                    next = n;
                    next_reach = t;
                }
            }

            if next < 0 || !is_crease(here, next) {
                return if piece.len() > 1 { Some(piece) } else { None };
            }

            piece.push(next);
            here = next;
            reached = next_reach;
        }

        i += 1;
    }

    // The whole chain is still crease up to the tip: the round never reached it.
    None
}

/// The crease from `start` to `end` along edges where a triangle
/// on the end face meets one off it, leaving `start` away from `came_from`: the vertices after
/// `start`, ending with `end`. `None` where the walk forks or stops short (or reaches a vertex
/// missing from `neighbours`).
pub fn crease_on_the_end_face(
    start: i32,
    came_from: i32,
    end: i32,
    is_crease: &dyn Fn(i32, i32) -> bool,
    neighbours: &HashMap<i32, Vec<i32>>,
) -> Option<Vec<i32>> {
    let mut walk = Vec::new();
    let mut visited: HashSet<i32> = HashSet::from([start, came_from]);
    let mut here = start;
    while here != end {
        let next: Vec<i32> = neighbours
            .get(&here)?
            .iter()
            .copied()
            .filter(|&n| (n == end || !visited.contains(&n)) && is_crease(here, n))
            .collect();
        if next.len() != 1 {
            return None;
        }

        here = next[0];
        visited.insert(here);
        walk.push(here);
    }

    Some(walk)
}

/// A triangle's oriented plane: its unit normal, the normal's offset and the centroid.
pub type OrientedPlane = (Vector3, f64, Vector3);

/// The plane of triangle `t` over the positions `at`, through its centroid.
pub fn plane_of(at: &[Vector3], t: (i32, i32, i32)) -> OrientedPlane {
    let (a, b, c) = (at[t.0 as usize], at[t.1 as usize], at[t.2 as usize]);
    let normal = (b - a).cross(c - a).get_normal();
    let centroid = (a + b + c) / 3.0;
    (normal, normal.dot(centroid), centroid)
}

/// The same oriented plane, to the catalogue's float noise
/// ([`COPLANAR_TOLERANCE`]): the facings agree and `q`'s centroid stands on `p`. Not the two
/// offsets: a triangle tilted by float noise 115 from the origin moves its offset by the tilt times
/// that distance - a wall triangle of the rounds' run-out tilted 2.5e-5 read 1.2e-3 off the wall,
/// was taken for a facet of the round, and the joined crease's side flooded over the wall
/// (RoundsFirstTests.TheWallStaysOffTheCreasesSide).
pub fn same_plane(p: OrientedPlane, q: OrientedPlane) -> bool {
    1.0 - p.0.dot(q.0) < COPLANAR_TOLERANCE
        && (p.0.dot(q.2) - p.1).abs() <= COPLANAR_TOLERANCE * ieee_math::max_f64(1.0, p.1.abs())
}

/// Each chain's fold, read edge by edge off the two triangles standing on
/// it: `Some(true)` convex, `Some(false)` concave, `None` where the chain's edges fold both ways or an
/// edge does not stand on exactly two triangles. `closed` `None` reads every chain open.
pub fn chain_senses(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: Option<&[bool]>,
) -> Vec<Option<bool>> {
    let standing = triangle_edges::standing_map(indices);
    let mut senses = Vec::with_capacity(chains.len());
    for (k, chain) in chains.iter().enumerate() {
        let count = if closed.is_some_and(|c| c[k]) {
            chain.len()
        } else {
            chain.len().saturating_sub(1)
        };
        let mut sense: Option<bool> = None;
        for i in 0..count {
            let a = chain[i];
            let b = chain[(i + 1) % chain.len()];
            let Some(on) = standing
                .get(&triangle_edges::key(a, b))
                .filter(|on| on.len() == 2)
            else {
                sense = None;
                break;
            };

            let convex = end_face_sense::is_convex(
                positions,
                indices[on[0] as usize],
                indices[on[1] as usize],
                a,
            );
            if i > 0 && sense != Some(convex) {
                sense = None;
                break;
            }

            sense = Some(convex);
        }

        senses.push(sense);
    }

    senses
}

/// True where the pick is built rounds first: some vertex is an end
/// of an open convex chain and of at least one open concave chain. A chain that folds both ways
/// takes no part (it is neither).
pub fn takes_rounds_first(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: Option<&[bool]>,
) -> bool {
    !rounds_first_tips(positions, indices, chains, closed).is_empty()
}

/// The knife tips - each vertex that ends an open convex chain and at
/// least one open concave chain - with the concave chains ending there, in the order met. Only what
/// the rounds-first creases can read: every tip has exactly two concave chains, and they end on one
/// face; otherwise none.
#[allow(clippy::needless_range_loop)]
pub fn rounds_first_tips(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: Option<&[bool]>,
) -> IndexMap<i32, Vec<i32>> {
    let mut tips: IndexMap<i32, Vec<i32>> = IndexMap::new();
    if chains.len() < 2 {
        return tips;
    }

    let senses = chain_senses(positions, indices, chains, closed);
    let is_open = |k: usize| !closed.is_some_and(|c| c[k]) && chains[k].len() > 1;
    let ends = |k: usize| [chains[k][0], chains[k][chains[k].len() - 1]];
    let convex_ends: HashSet<i32> = (0..chains.len())
        .filter(|&k| is_open(k) && senses[k] == Some(true))
        .flat_map(ends)
        .collect();
    for k in 0..chains.len() {
        if !is_open(k) || senses[k] != Some(false) {
            continue;
        }

        let [first, last] = ends(k);
        let distinct = if first == last {
            vec![first]
        } else {
            vec![first, last]
        };
        for end in distinct {
            if convex_ends.contains(&end) {
                tips.entry(end).or_default().push(k as i32);
            }
        }
    }

    let input_standing = triangle_edges::standing_map(indices);
    let convex_edges: HashSet<(i32, i32)> = (0..chains.len())
        .filter(|&k| senses[k] == Some(true))
        .flat_map(|k| {
            chains[k]
                .windows(2)
                .map(|w| triangle_edges::key(w[0], w[1]))
                .collect::<Vec<_>>()
        })
        .collect();
    for (&tip, concave) in &tips {
        let to_tip: Vec<Vec<i32>> = concave
            .iter()
            .map(|&k| {
                let chain = &chains[k as usize];
                if chain[chain.len() - 1] == tip {
                    chain.clone()
                } else {
                    chain.iter().rev().copied().collect()
                }
            })
            .collect();
        if concave.len() != 2
            || the_tips_end_face(
                positions,
                indices,
                &input_standing,
                &convex_edges,
                tip,
                &to_tip,
            )
            .is_none()
        {
            return IndexMap::new();
        }
    }

    tips
}

/// The face a knife tip's concave chains end on: of the two triangles
/// on each chain's end edge, the one standing on no convex chain edge. `None` where a chain has not
/// exactly one such, or the chains disagree.
pub fn the_tips_end_face(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    input_standing: &StandingMap,
    convex_edges: &HashSet<(i32, i32)>,
    tip: i32,
    to_tip: &[Vec<i32>],
) -> Option<OrientedPlane> {
    let mut end_face: Option<OrientedPlane> = None;
    for chain in to_tip {
        let on = input_standing.get(&triangle_edges::key(chain[chain.len() - 2], tip))?;
        let far: Vec<i32> = on
            .iter()
            .copied()
            .filter(|&t| {
                !triangle_edges::corners(indices[t as usize])
                    .iter()
                    .any(|e| convex_edges.contains(&triangle_edges::key(e.0, e.1)))
            })
            .collect();
        if far.len() != 1 {
            return None;
        }

        let plane = plane_of(positions, indices[far[0] as usize]);
        if end_face.is_some_and(|face| !same_plane(face, plane)) {
            return None;
        }

        end_face = Some(plane);
    }

    end_face
}

/// The chains pass A builds - each open convex chain ending at a
/// knife tip. Every other member of the pick (a face's rim, a convex chain at no tip) is built in
/// pass B with the joined creases, on the part the rounds left.
pub fn the_rounds_chains(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: Option<&[bool]>,
) -> Vec<i32> {
    let tips = rounds_first_tips(positions, indices, chains, closed);
    let senses = chain_senses(positions, indices, chains, closed);
    (0..chains.len())
        .filter(|&k| {
            senses[k] == Some(true)
                && !closed.is_some_and(|c| c[k])
                && chains[k].len() > 1
                && (tips.contains_key(&chains[k][0])
                    || tips.contains_key(&chains[k][chains[k].len() - 1]))
        })
        .map(|k| k as i32)
        .collect()
}

/// The rounds' mesh as kept triangles, and which of them are the
/// round's band - the rounds' own (added) triangles on no face the input had: a facet of the round,
/// not a piece of a face the fill tiled again. Empty where the rounds carry no mesh.
pub fn the_rounds_band(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    rounds: &GapFillResult,
) -> (Vec<(i32, i32, i32)>, Vec<bool>) {
    let Some(mesh) = &rounds.mesh else {
        return (Vec::new(), Vec::new());
    };
    let mesh = mesh.borrow();
    let input_planes: Vec<OrientedPlane> =
        indices.iter().map(|&t| plane_of(positions, t)).collect();
    let kept_index: Vec<i32> = (0..mesh.triangle_count())
        .filter(|&t| !mesh.is_removed(t))
        .collect();
    let triangles = kept(&mesh);
    let added: HashSet<i32> = rounds.added_triangles.iter().copied().collect();
    let band = triangles
        .iter()
        .enumerate()
        .map(|(i, &t)| {
            let plane = plane_of(&mesh.vertices, t);
            added.contains(&kept_index[i]) && !input_planes.iter().any(|&p| same_plane(p, plane))
        })
        .collect();
    (triangles, band)
}

/// The joined creases' vertices that are pass B's SMOOTH
/// joints - an interior vertex whose two crease edges both border the rounds' band. There the
/// crease is the round's section, a sampled smooth curve, so its turns are sampling and not
/// corners.
pub fn smooth_joints_over_the_rounds(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    creases: &[Vec<i32>],
    rounds: &GapFillResult,
) -> Vec<Vector3> {
    let (triangles, band) = the_rounds_band(positions, indices, rounds);
    let Some(mesh) = &rounds.mesh else {
        return Vec::new();
    };
    let mesh = mesh.borrow();
    let standing = triangle_edges::standing_map(&triangles);
    let borders_the_band = |a: i32, b: i32| {
        standing
            .get(&triangle_edges::key(a, b))
            .is_some_and(|on| on.iter().any(|&t| band[t as usize]))
    };
    let mut smooth = Vec::new();
    for crease in creases {
        for i in 1..crease.len().saturating_sub(1) {
            if borders_the_band(crease[i - 1], crease[i])
                && borders_the_band(crease[i], crease[i + 1])
            {
                smooth.push(mesh.vertices[crease[i] as usize]);
            }
        }
    }

    smooth
}

/// Pass A of a rounds-first pick - the convex chains at its knife
/// tips alone ([`the_rounds_chains`]), cut pick-wide and filled, each end running out through its
/// end face as a chain alone does. Refused as the cut or the fill refused.
#[allow(clippy::too_many_arguments)]
pub fn build_the_rounds(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: Option<&[bool]>,
    is_feature_edge: Option<FeatureEdgeTest>,
    size: f64,
    profile: BandProfile,
    fillet_segments: i32,
    snap: f64,
    seam_epsilon: f64,
    sharp_turn_degrees: f64,
    budget_for: Option<BudgetFor<'_>>,
) -> Result<GapFillResult, BuildAndFillError> {
    let convex = the_rounds_chains(positions, indices, chains, closed);
    let convex_chains: Vec<Vec<i32>> = convex.iter().map(|&k| chains[k as usize].clone()).collect();
    let convex_closed: Vec<bool> = convex
        .iter()
        .map(|&k| closed.is_some_and(|c| c[k as usize]))
        .collect();
    let options = BandGapBuildOptions {
        chord_tolerance: 0.0,
        snap,
        known_room: None,
        budget: budget_for.map(|f| f(indices.len() as i32)),
        seam_epsilon,
        sharp_turn_degrees,
        fillet_segments,
    };
    let gap = build_pick(
        positions,
        indices,
        &convex_chains,
        &convex_closed,
        is_feature_edge,
        size,
        profile,
        &options,
    )?;
    if !gap.built() {
        return Ok(GapFillResult {
            refusal: GapFillRefusal::OpenChainEnds,
            mesh: gap.mesh.clone(),
            phase: gap.phase,
            ..GapFillResult::default()
        });
    }

    let fill_triangles = gap
        .mesh
        .as_ref()
        .map_or(indices.len() as i32, |m| m.borrow().triangle_count());
    Ok(gap_fill::fill_with_budget(
        &gap,
        profile,
        fillet_segments,
        budget_for.map(|f| f(fill_triangles)),
    )?)
}

/// A rounds-first pick built whole - the convex chains
/// ([`build_the_rounds`]), then the concave creases read again on what they left
/// ([`creases_after_the_rounds_closed`]), cut pick-wide and filled with the band flooding over the
/// round ([`feature_edges_after_the_rounds`]). Refused as the first pass that refused;
/// [`GapFillRefusal::OpenChainEnds`] where the creases cannot be read. `room` is the whole pick's,
/// unread here: each pass reads its own, since neither pass is the pick.
///
/// The feature reading is `None` only where a rounds chain's edge stands on no triangle, which the
/// built pass A rules out; should it happen, the pick is refused as the unreadable creases are.
#[allow(clippy::too_many_arguments)]
pub fn build_rounds_first(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    is_feature_edge: Option<FeatureEdgeTest>,
    size: f64,
    profile: BandProfile,
    fillet_segments: i32,
    _room: Option<f64>,
    snap: f64,
    seam_epsilon: f64,
    sharp_turn_degrees: f64,
    budget_for: Option<BudgetFor<'_>>,
    closed: Option<&[bool]>,
) -> Result<GapFillResult, BuildAndFillError> {
    // The room is the whole pick's; each pass reads its own, since neither pass is the pick.
    let rounds = build_the_rounds(
        positions,
        indices,
        chains,
        closed,
        is_feature_edge.clone(),
        size,
        profile,
        fillet_segments,
        snap,
        seam_epsilon,
        sharp_turn_degrees,
        budget_for,
    )?;
    if !rounds.filled() {
        return Ok(rounds);
    }

    let Some(rounds_mesh) = rounds.mesh.clone() else {
        return Ok(the_creases_cannot_be_read(&rounds));
    };
    let mut creases_closed = Vec::new();
    let creases = {
        let mesh = rounds_mesh.borrow();
        creases_after_the_rounds_closed(
            positions,
            indices,
            chains,
            closed,
            &mesh,
            Some(&mut creases_closed),
        )
    };
    let Some(creases) = creases else {
        return Ok(the_creases_cannot_be_read(&rounds));
    };

    let Some(feature) = feature_edges_after_the_rounds(
        positions,
        indices,
        chains,
        closed,
        is_feature_edge,
        &rounds,
    ) else {
        return Ok(the_creases_cannot_be_read(&rounds));
    };
    let (rounds_vertices, kept) = {
        let mesh = rounds_mesh.borrow();
        (mesh.vertices.clone(), kept(&mesh))
    };

    // The creases on the rounds. Over the round the joined crease is the round's section, so its
    // turns there are sampling: smooth joints for everything pass B builds, the pick's sharp-turn
    // threshold kept for every other joint.
    let filled = {
        let _smooth = ChainField::smooth_joints_while(smooth_joints_over_the_rounds(
            positions, indices, &creases, &rounds,
        ));
        let options = BandGapBuildOptions {
            chord_tolerance: 0.0,
            snap,
            known_room: None,
            budget: budget_for.map(|f| f(kept.len() as i32)),
            seam_epsilon,
            sharp_turn_degrees,
            fillet_segments,
        };
        let gap = build_pick(
            &rounds_vertices,
            &kept,
            &creases,
            &creases_closed,
            Some(feature),
            size,
            profile,
            &options,
        )?;
        if !gap.built() {
            set_ends_site(Some(format!(
                "rounds first: creases band {:?} (room {} set by {})",
                gap.refusal,
                vector_math::number_format::format_f64_max_decimals(gap.room, 4),
                room_source_text(&gap.room_set_by)
            )));
            return Ok(GapFillResult {
                refusal: GapFillRefusal::OpenChainEnds,
                mesh: gap.mesh.clone(),
                phase: gap.phase,
                ..GapFillResult::default()
            });
        }

        let fill_triangles = gap
            .mesh
            .as_ref()
            .map_or(kept.len() as i32, |m| m.borrow().triangle_count());
        gap_fill::fill_with_budget(
            &gap,
            profile,
            fillet_segments,
            budget_for.map(|f| f(fill_triangles)),
        )?
    };
    if !filled.filled() {
        return Ok(filled);
    }

    let input_count = positions.len() as i32;
    let vertex_count = filled
        .mesh
        .as_ref()
        .map_or(input_count, |m| m.borrow().vertex_count());
    Ok(GapFillResult {
        refusal: GapFillRefusal::None,
        mesh: filled.mesh,
        added_triangles: filled.added_triangles,
        added_vertices: (input_count..vertex_count).collect(),
        phase: filled.phase,
        work_spent: filled.work_spent,
        work_limit: filled.work_limit,
        ..GapFillResult::default()
    })
}

/// [`build_rounds_first`]'s refusal where the creases after the rounds cannot be read.
fn the_creases_cannot_be_read(rounds: &GapFillResult) -> GapFillResult {
    set_ends_site(Some(
        "rounds first: the creases after the rounds cannot be read".to_string(),
    ));
    GapFillResult {
        refusal: GapFillRefusal::OpenChainEnds,
        mesh: rounds.mesh.clone(),
        ..GapFillResult::default()
    }
}

/// A room source as the diagnostic site prints it, in record form:
/// `RoomSource { Kind = Foreign, Chain = 0, OtherChain = -1, CreaseAt =  }`, an unset `CreaseAt`
/// printing nothing.
fn room_source_text(source: &RoomSource) -> String {
    let kind = match source.kind {
        RoomReadingKind::None => "None",
        RoomReadingKind::Foreign => "Foreign",
        RoomReadingKind::SelfRoom => "Self",
        RoomReadingKind::ChainToChain => "ChainToChain",
    };
    let crease_at = source.crease_at.map_or(String::new(), |at| at.to_string());
    format!(
        "RoomSource {{ Kind = {kind}, Chain = {}, OtherChain = {}, CreaseAt = {crease_at} }}",
        source.chain, source.other_chain
    )
}
