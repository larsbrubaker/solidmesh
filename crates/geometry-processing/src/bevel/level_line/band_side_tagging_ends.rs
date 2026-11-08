//! The END floods of the side tagging: each open chain end's end region, flooded out from the
//! triangles round the end vertex (the tags, floods and side readings are in
//! `band_side_tagging.rs`), with the error the tagging hands back ([`SideTaggingError`]: the
//! build's own exit, or a chain edge not on two triangles), and the taggings that call them: the
//! sides and their ends
//! ([`tag_sides`]), the band reached at a size ([`tag_band`]) and one chain's end regions on a
//! tagged pick ([`end_regions_of`]), and whether a flowed end reaches round to the chain's other
//! end ([`an_end_flows_to_the_other`]), the test a chain alone and a pick both refuse a flow by.
//!
//! An end region stops at a fold, and at a soft bend only where the end flood may not cross it
//! (`end_face_flow::crosses_a_soft_bend`): past the end within the band's reach, or size-free
//! while it still faces along the chain, or across a round from the end on. An opposite-sense end
//! face, or an end carrying the end ray, is no end region and stays untagged.

use std::cell::RefCell;
use std::collections::HashSet;

use solidmesh_math::Vector3;

use super::band_side_tagging::{
    flood, model_scale_of, normal_of, outward_at, ANOTHER_PICKS_SIDE, FIRST_END_TAG, SECOND_END_TAG,
};
use super::end_face_flow;
use super::end_face_sense;
use super::side_flow;
use super::triangle_edges::{self, StandingMap};
use super::{chain_on_the_mesh, ChainDistanceField};
use super::{ChainEdgeNotOnTwoTriangles, ChainField, WorkError};

/// Why a side tagging stopped: the build's own exit, or a chain edge that does not stand on two
/// consistently wound triangles.
#[derive(Debug, Clone)]
pub enum SideTaggingError {
    /// The work budget ran out or the build was cancelled.
    Work(WorkError),
    /// A chain edge not on two triangles.
    ChainEdge(ChainEdgeNotOnTwoTriangles),
}

impl From<WorkError> for SideTaggingError {
    fn from(error: WorkError) -> Self {
        SideTaggingError::Work(error)
    }
}

impl From<ChainEdgeNotOnTwoTriangles> for SideTaggingError {
    fn from(error: ChainEdgeNotOnTwoTriangles) -> Self {
        SideTaggingError::ChainEdge(error)
    }
}

impl std::fmt::Display for SideTaggingError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SideTaggingError::Work(error) => write!(f, "{error}"),
            SideTaggingError::ChainEdge(error) => write!(f, "{error:?}"),
        }
    }
}

impl std::error::Error for SideTaggingError {}

/// The fields an end flood reads at a size: the band's reach and the end face's (both optional,
/// the end face's defaulting to the band's).
#[derive(Clone, Copy, Default)]
pub struct EndReach<'a> {
    /// The chain's field at the size being built; `None` for a size-free tagging.
    pub end_reach: Option<&'a ChainField>,
    /// The end face's reach; `None` reads `end_reach`.
    pub end_face_reach: Option<&'a ChainField>,
}

/// Floods each open chain end's END REGION (tag 3 at the first
/// end, 4 at the last) from the untagged triangles round its end vertex, where that vertex has a
/// fold of its own. With `opens_another_picks_side`, another pick's side the end runs into is
/// opened to the flood only where it stands PAST the end (none of its corners behind the end's
/// plane, one beyond it), and closed again after.
#[allow(clippy::too_many_arguments)]
pub fn flood_ends(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    standing: &StandingMap,
    chain_keys: &HashSet<(i32, i32)>,
    is_feature_edge: &dyn Fn(i32, i32) -> bool,
    chain_edges: &[(i32, i32)],
    tags: &mut [i32],
    can_enter: Option<&dyn Fn(i32) -> bool>,
    end_rays: (bool, bool),
    reach: EndReach,
    opens_another_picks_side: bool,
) -> Result<(), SideTaggingError> {
    let end_face_reach = reach.end_face_reach.or(reach.end_reach);
    if chain_edges.is_empty() {
        return Ok(());
    }

    let first = chain_edges[0].0;
    let last = chain_edges[chain_edges.len() - 1].1;
    if first == last {
        return Ok(());
    }

    let model_scale = model_scale_of(positions);
    let past_tolerance = 1e-5 * model_scale;
    for (end, tag) in [(first, FIRST_END_TAG), (last, SECOND_END_TAG)] {
        let mut opened = Vec::new();
        if opens_another_picks_side {
            let open_point = positions[end as usize];
            let open_outward =
                outward_at(positions, chain_edges, tag == FIRST_END_TAG, model_scale);
            let along = |v: i32| (positions[v as usize] - open_point).dot(open_outward);
            let not_behind = |v: i32| v == end || along(v) >= -past_tolerance;
            let past = |v: i32| along(v) > past_tolerance;
            for (i, &t) in indices.iter().enumerate() {
                if tags[i] == ANOTHER_PICKS_SIDE
                    && not_behind(t.0)
                    && not_behind(t.1)
                    && not_behind(t.2)
                    && (past(t.0) || past(t.1) || past(t.2))
                {
                    tags[i] = 0;
                    opened.push(i);
                }
            }
        }

        let flooded = flood_one_end(
            positions,
            indices,
            standing,
            chain_keys,
            is_feature_edge,
            chain_edges,
            tags,
            can_enter,
            end_rays,
            reach.end_reach,
            end_face_reach,
            end,
            tag,
            model_scale,
            past_tolerance,
        );

        // On success or error alike, what was opened and stayed untagged closes again.
        for i in opened {
            if tags[i] == 0 {
                tags[i] = ANOTHER_PICKS_SIDE;
            }
        }

        flooded?;
    }

    Ok(())
}

/// Floods one chain end's end region.
#[allow(clippy::too_many_arguments)]
fn flood_one_end(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    standing: &StandingMap,
    chain_keys: &HashSet<(i32, i32)>,
    is_feature_edge: &dyn Fn(i32, i32) -> bool,
    chain_edges: &[(i32, i32)],
    tags: &mut [i32],
    can_enter: Option<&dyn Fn(i32) -> bool>,
    end_rays: (bool, bool),
    end_reach: Option<&ChainField>,
    end_face_reach: Option<&ChainField>,
    end: i32,
    tag: i32,
    model_scale: f64,
    past_tolerance: f64,
) -> Result<(), SideTaggingError> {
    let mut round = Vec::new();
    let mut has_another_fold = false;
    for (i, &t) in indices.iter().enumerate() {
        if t.0 != end && t.1 != end && t.2 != end {
            continue;
        }

        round.push(i as i32);
        for (a, b) in triangle_edges::corners(t) {
            if (a == end || b == end)
                && !chain_keys.contains(&triangle_edges::key(a, b))
                && is_feature_edge(a, b)
            {
                has_another_fold = true;
            }
        }
    }

    let seeds: Vec<i32> = round
        .into_iter()
        .filter(|&t| tags[t as usize] == 0)
        .collect();
    if !has_another_fold || seeds.is_empty() {
        return Ok(());
    }

    // An OPPOSITE-SENSE end face is not cut by the bead but grows over its section, so it is no end
    // region: it stays untagged.
    let first = tag == FIRST_END_TAG;
    let chain_edge = if first {
        chain_edges[0]
    } else {
        chain_edges[chain_edges.len() - 1]
    };
    if end_face_sense::is_opposite_sense(positions, indices, standing, tags, chain_edge, end)? {
        return Ok(());
    }

    // An end carrying the END RAY is not cut by the chain's distance either: the strip runs whole
    // past the end and the end plane clips it, so the end face stays untagged and whole.
    if if first { end_rays.0 } else { end_rays.1 } {
        return Ok(());
    }

    let end_point = positions[end as usize];
    let outward = outward_at(positions, chain_edges, first, model_scale);
    let side_normals: Vec<Vector3> = standing
        .get(&triangle_edges::key(chain_edge.0, chain_edge.1))
        .map_or(&[][..], |on| on.as_slice())
        .iter()
        .map(|&t| normal_of(positions, indices[t as usize]))
        .collect();

    // A soft bend is crossed only where it stands wholly PAST the end, along the chain - and at a
    // size, only within the band's REACH; the margin is the model's scale, so a vertex level with
    // the end does not read as past it from float noise.
    let past_the_end = |v: i32| (positions[v as usize] - end_point).dot(outward) > past_tolerance;
    let rounds = end_face_flow::rounds_from_the_end(
        positions,
        indices,
        standing,
        chain_keys,
        is_feature_edge,
        &seeds,
        end,
    )?;

    // The stop test reads the band, which may charge the budget: the first error stops every
    // crossing and is handed back after the flood.
    let failed: RefCell<Option<WorkError>> = RefCell::new(None);
    let stops = |a: i32, b: i32| {
        if failed.borrow().is_some() {
            return true;
        }

        if a == end || b == end {
            return false;
        }

        if is_feature_edge(a, b) {
            return true;
        }

        if !end_face_flow::bends(positions, indices, standing, a, b) {
            return false;
        }

        match end_face_flow::crosses_a_soft_bend(
            positions,
            indices,
            standing,
            a,
            b,
            past_the_end(a) && past_the_end(b),
            rounds,
            end_face_reach,
            past_tolerance,
            &side_normals,
        ) {
            Ok(crosses) => !crosses,
            Err(error) => {
                *failed.borrow_mut() = Some(error);
                true
            }
        }
    };
    flood(
        indices, standing, chain_keys, &stops, &seeds, tag, tags, can_enter,
    )?;
    if let Some(error) = failed.into_inner() {
        return Err(error.into());
    }

    // Geometric - the band at this size reaching a far crease of the end region - so a size-free
    // tagging is exactly what it was.
    if let Some(end_reach) = end_reach {
        end_face_flow::flow_over_a_narrow_end_face(
            positions,
            indices,
            standing,
            chain_keys,
            is_feature_edge,
            end,
            tag,
            tags,
            end_reach,
            can_enter,
        )?;
    }

    Ok(())
}

/// The two sides' seeds: per chain edge, its left and right triangle.
fn side_seeds(
    indices: &[(i32, i32, i32)],
    standing: &StandingMap,
    chain_edges: &[(i32, i32)],
) -> Result<(Vec<i32>, Vec<i32>), SideTaggingError> {
    let mut left_seeds = Vec::with_capacity(chain_edges.len());
    let mut right_seeds = Vec::with_capacity(chain_edges.len());
    for &(from, to) in chain_edges {
        let (left, right) = chain_on_the_mesh::sides_of(indices, standing, from, to)?;
        left_seeds.push(left);
        right_seeds.push(right);
    }

    Ok((left_seeds, right_seeds))
}

/// The fields a side tagging reads at a size.
#[derive(Clone, Copy, Default)]
pub struct SideReach<'a> {
    /// The end floods' band and end-face reach.
    pub ends: EndReach<'a>,
    /// The side's own reach: a gentle convex crease it reaches does not stop the side's flood.
    pub side_reach: Option<&'a ChainField>,
}

/// Floods the two sides out from the chain, stopping at chain edges
/// and feature edges - tag 1 left, tag 2 right, 0 everywhere else - and, where the chain divides
/// its sides (`divides`, false when one region reaches both) and `end_floods` with positions,
/// floods its end regions. With a side reach, a gentle convex crease the band reaches does not stop
/// a side (the Flow decision on a SIDE): the side carries on onto the next face.
#[allow(clippy::too_many_arguments)]
pub fn tag_sides(
    indices: &[(i32, i32, i32)],
    chain_edges: &[(i32, i32)],
    is_feature_edge: &dyn Fn(i32, i32) -> bool,
    end_floods: bool,
    positions: Option<&[Vector3]>,
    end_rays: (bool, bool),
    reach: SideReach,
) -> Result<(Vec<i32>, bool), SideTaggingError> {
    let standing = triangle_edges::standing_map(indices);
    let chain_keys: HashSet<(i32, i32)> = chain_edges
        .iter()
        .map(|&(f, t)| triangle_edges::key(f, t))
        .collect();
    let (left_seeds, right_seeds) = side_seeds(indices, &standing, chain_edges)?;

    let failed: RefCell<Option<WorkError>> = RefCell::new(None);
    let side_stops = |a: i32, b: i32| {
        if !is_feature_edge(a, b) {
            return false;
        }

        let (Some(side_reach), Some(positions)) = (reach.side_reach, positions) else {
            return true;
        };
        if failed.borrow().is_some() {
            return true;
        }

        match side_flow::flows_onto_a_gentle_neighbour(
            positions, indices, &standing, side_reach, a, b,
        ) {
            Ok(flows) => !flows,
            Err(error) => {
                *failed.borrow_mut() = Some(error);
                true
            }
        }
    };

    let mut tags = vec![0; indices.len()];
    flood(
        indices,
        &standing,
        &chain_keys,
        &side_stops,
        &left_seeds,
        1,
        &mut tags,
        None,
    )?;
    let divides = right_seeds.iter().all(|&t| tags[t as usize] != 1);
    if divides {
        flood(
            indices,
            &standing,
            &chain_keys,
            &side_stops,
            &right_seeds,
            2,
            &mut tags,
            None,
        )?;
    }

    if let Some(error) = failed.into_inner() {
        return Err(error.into());
    }

    if divides && end_floods {
        if let Some(positions) = positions {
            let ends = EndReach {
                end_reach: reach.ends.end_reach,
                end_face_reach: reach.ends.end_face_reach.or(reach.ends.end_reach),
            };
            flood_ends(
                positions,
                indices,
                &standing,
                &chain_keys,
                is_feature_edge,
                chain_edges,
                &mut tags,
                None,
                end_rays,
                ends,
                false,
            )?;
        }
    }

    Ok((tags, divides))
}

/// The sides flooded as [`tag_sides`] floods them, but entering only
/// triangles the band at this size reaches (least field value over the triangle at or below
/// `snap`, with the box pre-check), and the end regions flooded with the field as the end reach.
#[allow(clippy::too_many_arguments)]
pub fn tag_band(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain_edges: &[(i32, i32)],
    is_feature_edge: &dyn Fn(i32, i32) -> bool,
    field: &ChainField,
    snap: f64,
    end_floods: bool,
) -> Result<(Vec<i32>, bool), SideTaggingError> {
    let standing = triangle_edges::standing_map(indices);
    let chain_keys: HashSet<(i32, i32)> = chain_edges
        .iter()
        .map(|&(f, t)| triangle_edges::key(f, t))
        .collect();
    let (left_seeds, right_seeds) = side_seeds(indices, &standing, chain_edges)?;

    let failed: RefCell<Option<WorkError>> = RefCell::new(None);
    let can_enter = |triangle: i32| {
        if failed.borrow().is_some() {
            return false;
        }

        let t = indices[triangle as usize];
        let (a, b, c) = (
            positions[t.0 as usize],
            positions[t.1 as usize],
            positions[t.2 as usize],
        );
        let min = Vector3::component_min(Vector3::component_min(a, b), c);
        let max = Vector3::component_max(Vector3::component_max(a, b), c);
        if ChainDistanceField::nothing_reaches_below(field, min, max, snap) {
            return false;
        }

        match ChainDistanceField::min_over_triangle(field, a, b, c) {
            Ok(least) => least.value <= snap,
            Err(error) => {
                *failed.borrow_mut() = Some(error);
                false
            }
        }
    };

    let mut tags = vec![0; indices.len()];
    flood(
        indices,
        &standing,
        &chain_keys,
        is_feature_edge,
        &left_seeds,
        1,
        &mut tags,
        Some(&can_enter),
    )?;
    let divides = right_seeds.iter().all(|&t| tags[t as usize] != 1);
    if divides {
        flood(
            indices,
            &standing,
            &chain_keys,
            is_feature_edge,
            &right_seeds,
            2,
            &mut tags,
            Some(&can_enter),
        )?;
        if end_floods {
            flood_ends(
                positions,
                indices,
                &standing,
                &chain_keys,
                is_feature_edge,
                chain_edges,
                &mut tags,
                Some(&can_enter),
                field.end_rays,
                EndReach {
                    end_reach: Some(field),
                    end_face_reach: Some(field),
                },
                false,
            )?;
        }
    }

    if let Some(error) = failed.into_inner() {
        return Err(error.into());
    }

    Ok((tags, divides))
}

/// One chain's end regions read on a pick's tagging: its own
/// sides (tags `seeds` accepts) read as a picked side, another chain's sides as another pick's side
/// (closed to the end flood but where it stands past the end), untagged as untagged. `end_reach`,
/// the chain's field at the size being built, flows each end region over an end face narrower than
/// the setback exactly as the one-chain tagging does (`EndReach::end_reach`); `None` is the
/// size-free tagging.
#[allow(clippy::too_many_arguments)]
pub fn end_regions_of(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain_edges: &[(i32, i32)],
    is_feature_edge: &dyn Fn(i32, i32) -> bool,
    pick_tags: &[i32],
    seeds: &dyn Fn(i32) -> bool,
    end_rays: (bool, bool),
    end_face_reach: Option<&ChainField>,
    end_reach: Option<&ChainField>,
) -> Result<Vec<i32>, SideTaggingError> {
    let mut tags: Vec<i32> = pick_tags
        .iter()
        .map(|&t| {
            if t == 0 {
                0
            } else if seeds(t) {
                1
            } else {
                ANOTHER_PICKS_SIDE
            }
        })
        .collect();
    let chain_keys: HashSet<(i32, i32)> = chain_edges
        .iter()
        .map(|&(f, t)| triangle_edges::key(f, t))
        .collect();
    flood_ends(
        positions,
        indices,
        &triangle_edges::standing_map(indices),
        &chain_keys,
        is_feature_edge,
        chain_edges,
        &mut tags,
        None,
        end_rays,
        EndReach {
            end_reach,
            end_face_reach,
        },
        true,
    )?;
    // A flow that carries an end round to the chain's other end is not taken, as a chain alone
    // does not take it (`narrow_face_flow`): the end regions stay the size-free ones.
    if end_reach.is_some() {
        let plain = end_regions_of(
            positions,
            indices,
            chain_edges,
            is_feature_edge,
            pick_tags,
            seeds,
            end_rays,
            end_face_reach,
            None,
        )?;
        if an_end_flows_to_the_other(indices, chain_edges, &plain, &tags) {
            return Ok(plain);
        }
    }
    Ok(tags)
}

/// Whether `flowed`, a chain's end regions flowed over a narrow end face, carries one END round to
/// the chain's OTHER end where `plain` (the same regions unflowed) does not: a triangle at the last
/// vertex newly in the first end's region, or at the first vertex newly in the last end's. Such an
/// end is no end at all - the band has cut through the part's whole thickness there and would
/// split it in two - so neither a chain alone (`narrow_face_flow`) nor a pick (`end_regions_of`)
/// takes that flow.
pub fn an_end_flows_to_the_other(
    indices: &[(i32, i32, i32)],
    chain_edges: &[(i32, i32)],
    plain: &[i32],
    flowed: &[i32],
) -> bool {
    let first = chain_edges[0].0;
    let last = chain_edges[chain_edges.len() - 1].1;
    for (i, &(a, b, c)) in indices.iter().enumerate() {
        let at_first = a == first || b == first || c == first;
        let at_last = a == last || b == last || c == last;
        if (at_last && flowed[i] == FIRST_END_TAG && plain[i] != FIRST_END_TAG)
            || (at_first && flowed[i] == SECOND_END_TAG && plain[i] != SECOND_END_TAG)
        {
            return true;
        }
    }

    false
}
