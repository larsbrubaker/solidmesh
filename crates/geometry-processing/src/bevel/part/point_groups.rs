//! Point mode's groups, their build: [`build_part`] (one part whose groups include
//! `BevelSelectionKind::Point` ones: its edge groups go to the surface construction first,
//! `prologue::build_edges`, and the points are bevelled onto what that made) and [`build`] (the
//! point bevel itself, with the size that fits found for a point refused by size). How far a stored
//! point may drift from the corner it resolves to ([`drift_on`]) and which corner that is
//! ([`nearest_candidate`]) are here too: a caller's point picking reads them so it holds the same
//! corners the build bevels. [`bevel_part`](super::bevel_part) calls [`build_part`].

use std::collections::HashMap;

use polygon_mesh::{CancelToken, Mesh};
use vector_math::{Color, Vector3};

use super::answer_mesh::the_mesh_of;
use super::prologue::build_edges;
use super::shown_size::the_shown_size;
use super::surface_part_builder::level_line_kernel_handoff::the_kept_triangles;
use super::{BevelPartError, BevelPartOutcome, BevelPartRequest, BevelProgressReport};
use crate::bevel::bevel_working_mesh::{self, StampedVerdict};
use crate::bevel::level_line::point_bevel::{
    self, PointBevelFault, PointBevelKind, PointBevelRefusal,
};
use crate::bevel::level_line::point_candidates;
use crate::bevel::{
    BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode, BevelGroupResult, BevelResultGroup,
};
use crate::topology::{bevel_edge_group_resolver, BevelEdgeGroup, BevelSelectionKind};

/// How far, as a fraction of the part's bounding diagonal, a stored point may be from the
/// candidate it resolves to - a re-tessellated tip moves by a small fraction of the part, a
/// different corner by a large one.
pub const DRIFT_FRACTION_OF_PART: f64 = 0.02;

/// How far a stored point may be from the candidate it resolves to on `mesh`.
pub fn drift_on(mesh: &Mesh) -> f64 {
    let bounds = mesh.get_axis_aligned_bounding_box();
    f64::max(
        1e-4,
        DRIFT_FRACTION_OF_PART * (bounds.max_xyz - bounds.min_xyz).length(),
    )
}

/// The index into `candidates` of the one nearest `point`, or -1 when even that one is past
/// `drift`. One rule for the build and the corners a caller's picking shows as held. Ties keep the
/// first candidate.
pub fn nearest_candidate(candidates: &[Vector3], point: Vector3, drift: f64) -> i32 {
    let mut nearest = -1;
    let mut nearest_distance = f64::INFINITY;
    for (c, candidate) in candidates.iter().enumerate() {
        let distance = (point - *candidate).length();
        if distance < nearest_distance {
            nearest = c as i32;
            nearest_distance = distance;
        }
    }
    if nearest_distance <= drift {
        nearest
    } else {
        -1
    }
}

/// How many times the size that fits is halved toward before giving up.
const HALVINGS_ALLOWED: i32 = 8;

/// A point's row refused with `refusal`.
fn refused(i: usize, group: &BevelEdgeGroup, refusal: BevelAdvisory) -> BevelGroupResult {
    BevelGroupResult::new(
        i as i32,
        BevelResultGroup::Edge(group.clone()),
        0,
        Some(refusal),
        Vec::new(),
    )
}

/// A refusal that quotes nothing.
fn plain(code: BevelAdvisoryCode) -> BevelAdvisory {
    BevelAdvisory::new(code, Vec::new())
}

/// One part whose groups include Point-mode ones. The edge groups are built by the surface
/// construction first (`progress` is told its phases), and the points are bevelled onto what it
/// made (or onto the welded part when there were no edge groups).
pub fn build_part(
    request: &BevelPartRequest,
    cancellation_token: &CancelToken,
    progress: Option<BevelProgressReport>,
) -> Result<BevelPartOutcome, BevelPartError> {
    let groups = request.groups.as_slice();
    let point_indices: Vec<usize> = (0..groups.len())
        .filter(|&i| groups[i].selection_kind == BevelSelectionKind::Point)
        .collect();
    let edge_indices: Vec<usize> = (0..groups.len())
        .filter(|i| !point_indices.contains(i))
        .collect();
    let mut results: Vec<Option<BevelGroupResult>> = vec![None; groups.len()];
    let mut edge_result: Option<BevelPartOutcome> = None;
    if !edge_indices.is_empty() {
        let edge_request = BevelPartRequest {
            groups: edge_indices.iter().map(|&i| groups[i].clone()).collect(),
            ..request.clone()
        };
        let built = build_edges(&edge_request, cancellation_token, progress)?;
        for row in built.group_results.iter().flatten() {
            let index = edge_indices[row.group_index as usize];
            let mut row = row.clone();
            row.group_index = index as i32;
            results[index] = Some(row);
        }
        edge_result = Some(built);
    }

    let edge_mesh = edge_result.as_ref().and_then(|edge| edge.mesh.clone());
    let edge_color = edge_result
        .as_ref()
        .map_or(Color::default(), |edge| edge.color);
    let edge_failures = edge_result.as_ref().and_then(|edge| edge.failures.clone());
    let edge_advisories = edge_result
        .as_ref()
        .and_then(|edge| edge.advisories.clone());

    let Some(source_mesh) = request
        .mesh
        .as_deref()
        .filter(|mesh| !mesh.vertices.is_empty())
    else {
        for &i in &point_indices {
            results[i] = Some(refused(
                i,
                &groups[i],
                plain(BevelAdvisoryCode::PartHasNoMeshToBevel),
            ));
        }

        return Ok(BevelPartOutcome::new(
            edge_mesh,
            edge_color,
            results,
            edge_failures,
            edge_advisories,
        ));
    };

    let source_to_operation = request.source_to_operation;
    let bend_threshold_degrees = request.bend_threshold_degrees;
    let (mut working_mesh, source_advisories) = bevel_working_mesh::create_from_with_advisories(
        source_mesh,
        &mut StampedVerdict::default(),
    );
    bevel_working_mesh::carry_into(&mut working_mesh, source_to_operation);

    // Which points the picked edges' chains pass through, read on the part as it was picked: a
    // point there is the corner hole with a larger setback, which is later work.
    let mut chain_points = Vec::new();
    if !edge_indices.is_empty() {
        let edge_groups: Vec<BevelEdgeGroup> = edge_indices
            .iter()
            .map(|&i| {
                groups[i]
                    .transformed(source_to_operation)
                    .read_at(bend_threshold_degrees)
            })
            .collect();
        let (resolutions, _) = bevel_edge_group_resolver::resolve_all(&edge_groups, &working_mesh);
        for resolution in resolutions.iter().filter(|r| r.is_resolved()) {
            for edge in &resolution.edges {
                chain_points.push(edge.start);
                chain_points.push(edge.end);
            }
        }
    }

    let onto = edge_mesh.as_ref().unwrap_or(&working_mesh);
    let anchors: Vec<BevelEdgeGroup> = groups
        .iter()
        .map(|group| group.transformed(source_to_operation))
        .collect();
    let built = build(
        onto,
        &anchors,
        &point_indices,
        groups,
        &chain_points,
        request.size,
        request.segments,
        request.profile_is_round,
        bend_threshold_degrees,
        &mut results,
    )?;
    let color = if edge_mesh.is_some() {
        edge_color
    } else {
        request.color
    };
    let mesh = built.or(edge_mesh);
    // the edge build already said what the repair of this part's mesh had to do
    let advisories = if edge_result.is_some() {
        edge_advisories
    } else {
        Some(source_advisories)
    };
    Ok(BevelPartOutcome::new(
        mesh,
        color,
        results,
        edge_failures,
        advisories,
    ))
}

/// Bevels the Point groups at `indices` of `groups` (in the space of `mesh`) onto
/// `mesh`, writing each one's row into `results` (rows name `reported_groups`, the groups as the
/// user stored them). `None` when no point built.
#[allow(clippy::too_many_arguments)]
pub fn build(
    mesh: &Mesh,
    groups: &[BevelEdgeGroup],
    indices: &[usize],
    reported_groups: &[BevelEdgeGroup],
    picked_chain_points: &[Vector3],
    size: f64,
    segments: i32,
    round_profile: bool,
    bend_threshold_degrees: f64,
    results: &mut [Option<BevelGroupResult>],
) -> Result<Option<Mesh>, PointBevelFault> {
    let positions: Vec<Vector3> = mesh
        .vertices
        .iter()
        .map(|v| Vector3::new(v.x as f64, v.y as f64, v.z as f64))
        .collect();
    let triangles: Vec<(i32, i32, i32)> = mesh.faces.iter().map(|f| (f.v0, f.v1, f.v2)).collect();
    let drift = drift_on(mesh);
    let kind = if round_profile {
        PointBevelKind::Fillet
    } else {
        PointBevelKind::Chamfer
    };
    let candidates: Vec<i32> =
        point_candidates::describe(&positions, &triangles, bend_threshold_degrees)
            .into_iter()
            .map(|c| c.vertex)
            .collect();
    let candidate_positions: Vec<Vector3> =
        candidates.iter().map(|&v| positions[v as usize]).collect();
    let mut picks: Vec<(i32, f64)> = Vec::new();
    let mut pick_groups: Vec<usize> = Vec::new();

    for &i in indices {
        let anchor = groups[i].anchor_midpoint;

        // Asked of the stored point and the chains as picked, not of the mesh the edges were
        // already built into: there the corner a chain runs through has been cut away and would
        // read as gone.
        if picked_chain_points
            .iter()
            .any(|&p| (p - anchor).length() <= drift)
        {
            results[i] = Some(refused(
                i,
                &reported_groups[i],
                plain(BevelAdvisoryCode::PointMeetsAPickedEdge),
            ));
            continue;
        }

        let nearest = nearest_candidate(&candidate_positions, anchor, drift);
        if nearest < 0 {
            results[i] = Some(refused(
                i,
                &reported_groups[i],
                plain(BevelAdvisoryCode::PointIsNoLongerACorner),
            ));
            continue;
        }

        // two picks on one corner: the first bevels it, as the first holds its ball in the viewport
        let vertex = candidates[nearest as usize];
        if picks.iter().any(|pick| pick.0 == vertex) {
            results[i] = Some(refused(
                i,
                &reported_groups[i],
                plain(BevelAdvisoryCode::PointPickedTwice),
            ));
            continue;
        }

        picks.push((vertex, size));
        pick_groups.push(i);
    }

    if picks.is_empty() {
        return Ok(None);
    }

    let built = point_bevel::build(
        &positions,
        &triangles,
        &picks,
        kind,
        segments,
        bend_threshold_degrees,
    )?;
    // keyed by the size's bits, so a size is found again only when it is exactly the same double
    let mut confirmed: HashMap<u64, Vec<bool>> = HashMap::new();
    let mut fault = None;
    for (p, point) in built.points.iter().enumerate() {
        let i = pick_groups[p];
        if point.is_built() {
            results[i] = Some(BevelGroupResult::new(
                i as i32,
                BevelResultGroup::Edge(reported_groups[i].clone()),
                1,
                None,
                Vec::new(),
            ));
            continue;
        }

        if point.failure != PointBevelRefusal::TooLarge {
            results[i] = Some(BevelGroupResult::new(
                i as i32,
                BevelResultGroup::Edge(reported_groups[i].clone()),
                0,
                point.refusal.clone(),
                Vec::new(),
            ));
            continue;
        }

        // a size past the room is refused by size, with the size that fits, as the edge groups'
        // rows are
        let fits = the_size_that_builds(point.room, |size_tried| {
            if fault.is_some() {
                return false;
            }
            let key = size_tried.to_bits();
            if let Some(points) = confirmed.get(&key) {
                return points[p];
            }
            let tried: Vec<(i32, f64)> = picks.iter().map(|pick| (pick.0, size_tried)).collect();
            match point_bevel::build(
                &positions,
                &triangles,
                &tried,
                kind,
                segments,
                bend_threshold_degrees,
            ) {
                Ok(at) => {
                    let points: Vec<bool> = at.points.iter().map(|one| one.is_built()).collect();
                    let answer = points[p];
                    confirmed.insert(key, points);
                    answer
                }
                Err(error) => {
                    fault = Some(error);
                    false
                }
            }
        });
        // A fault in a trial build ends the whole point build with that error.
        if let Some(error) = fault.take() {
            return Err(error);
        }
        let refusal = match fits {
            None => plain(BevelAdvisoryCode::PointTooLargeForAnySize),
            Some(fits) => BevelAdvisory::new(
                BevelAdvisoryCode::PointTooLargeRoomFor,
                vec![BevelAdvisoryArg::Double(fits)],
            ),
        };
        let mut row = refused(i, &reported_groups[i], refusal);
        row.largest_size_that_fits = fits;
        results[i] = Some(row);
    }

    Ok(built.mesh.map(|mesh| {
        let mesh = mesh.borrow();
        the_mesh_of(mesh.vertices(), &the_kept_triangles(&mesh))
    }))
}

/// The largest shown size at which the pick builds (`built_at` answers
/// for that one pick) - the room's own shown size when that builds,
/// else bisected toward zero for [`HALVINGS_ALLOWED`] steps. `None` when no size tried builds it.
fn the_size_that_builds(room: f64, mut built_at: impl FnMut(f64) -> bool) -> Option<f64> {
    let shown = the_shown_size(room);
    if shown > 0.0 && built_at(shown) {
        return Some(shown);
    }

    let mut low = 0.0;
    let mut high = shown;
    let mut largest = None;
    for _ in 0..HALVINGS_ALLOWED {
        let trial = the_shown_size((low + high) / 2.0);
        if trial <= low {
            break;
        }

        if built_at(trial) {
            largest = Some(trial);
            low = trial;
        } else {
            high = trial;
        }
    }

    largest
}
