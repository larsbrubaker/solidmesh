//! The side tagging of the band: which side of the chain each triangle stands on, and the end
//! regions. Ports `Bevel/LevelLine/BandSideTagging.cs` (a C# static class): so far its tags, the
//! both-sided-members switch, the model's scale, the outward direction at a chain end and a facet
//! normal, the flood out from seeds ([`flood`]) and the connected regions ([`connected_regions`]);
//! the side and end floods land with later packages.

use vector_math::Vector3;

use std::collections::{HashSet, VecDeque};

use super::triangle_edges::{self, StandingMap};
use super::{LevelLineWork, WorkError};

/// The end flood's tag at the chain's FIRST vertex.
pub const FIRST_END_TAG: i32 = 3;

/// The end flood's tag at the chain's LAST vertex.
pub const SECOND_END_TAG: i32 = 4;

/// In the end regions' working tags: a triangle on a side another chain of the pick seeds - closed
/// to the end flood but where it stands past the end.
pub const ANOTHER_PICKS_SIDE: i32 = -1;

/// Whether a both-sided member of a pick is built (C# `BothSidedMembersBuild`, an `AsyncLocal`
/// flow switch, so here a field of the budget in force): on by default and with no budget in force;
/// off - a test's way back to the old answer - the band and the pair room refuse such a member as
/// `ChainDoesNotDivideItsSides`.
pub fn both_sided_members_build() -> bool {
    LevelLineWork::current().is_none_or(|budget| !budget.both_sided_members_off.get())
}

/// Sets the both-sided-members switch on the budget in force; with none in force there is no
/// build to carry it, and nothing changes.
pub fn set_both_sided_members_build(on: bool) {
    if let Some(budget) = LevelLineWork::current() {
        budget.both_sided_members_off.set(!on);
    }
}

/// The diagonal of the positions' box; 0 for none.
pub fn model_scale_of(positions: &[Vector3]) -> f64 {
    let mut min = Vector3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max = Vector3::new(f64::MIN, f64::MIN, f64::MIN);
    for &p in positions {
        min = Vector3::component_min(min, p);
        max = Vector3::component_max(max, p);
    }

    if positions.is_empty() {
        0.0
    } else {
        (max - min).length()
    }
}

/// The unit direction out of the chain at its first (or last) end: from the first chain vertex
/// back along the chain at least a thousandth of the model's scale, to the end.
pub fn outward_at(
    positions: &[Vector3],
    chain_edges: &[(i32, i32)],
    first: bool,
    model_scale: f64,
) -> Vector3 {
    let n = chain_edges.len();
    let end_point = positions[if first {
        chain_edges[0].0
    } else {
        chain_edges[n - 1].1
    } as usize];
    let mut back = end_point;
    for i in 0..n {
        let edge = if first {
            chain_edges[i]
        } else {
            chain_edges[n - 1 - i]
        };
        back = positions[if first { edge.1 } else { edge.0 } as usize];
        if (end_point - back).length() >= 1e-3 * model_scale {
            break;
        }
    }

    (end_point - back).get_normal()
}

/// The unit normal of an input triangle.
pub fn normal_of(positions: &[Vector3], t: (i32, i32, i32)) -> Vector3 {
    (positions[t.1 as usize] - positions[t.0 as usize])
        .cross(positions[t.2 as usize] - positions[t.0 as usize])
        .get_normal()
}

/// C# `BandSideTagging.Flood`: tags with `tag` every untagged triangle reachable from `seeds`
/// (taken in ascending order) without crossing a chain edge or a feature edge, and that `can_enter`
/// allows. The answer is the TAG ARRAY - a set of triangles - which does not depend on the order it
/// was filled in, so the neighbour lists may be read in the order the mesh holds them.
#[allow(clippy::too_many_arguments)]
pub fn flood(
    indices: &[(i32, i32, i32)],
    standing: &StandingMap,
    chain_keys: &HashSet<(i32, i32)>,
    is_feature_edge: &dyn Fn(i32, i32) -> bool,
    seeds: &[i32],
    tag: i32,
    tags: &mut [i32],
    can_enter: Option<&dyn Fn(i32) -> bool>,
) -> Result<(), WorkError> {
    let mut sorted = seeds.to_vec();
    sorted.sort();
    let mut pending = VecDeque::new();
    for seed in sorted {
        if tags[seed as usize] == 0 {
            tags[seed as usize] = tag;
            pending.push_back(seed);
        }
    }

    while let Some(t) = pending.pop_front() {
        LevelLineWork::charge(1)?;
        for (a, b) in triangle_edges::corners(indices[t as usize]) {
            let key = triangle_edges::key(a, b);
            if chain_keys.contains(&key) || is_feature_edge(a, b) {
                continue;
            }

            // C#'s indexer: every edge of a triangle stands in the map built from the same list.
            for &neighbour in standing.get(&key).map_or(&[][..], |n| n.as_slice()) {
                if tags[neighbour as usize] == 0 && can_enter.is_none_or(|can| can(neighbour)) {
                    tags[neighbour as usize] = tag;
                    pending.push_back(neighbour);
                }
            }
        }
    }

    Ok(())
}

/// C# `BandSideTagging.ConnectedRegions`: every triangle's CONNECTED REGION - the triangles
/// reachable from it without crossing a chain edge or a feature edge, numbered from 1. One region
/// is one piece of surface, so two triangles share a number only when a path on the part joins them.
pub fn connected_regions(
    indices: &[(i32, i32, i32)],
    standing: &StandingMap,
    chain_keys: &HashSet<(i32, i32)>,
    is_feature_edge: &dyn Fn(i32, i32) -> bool,
) -> Result<Vec<i32>, WorkError> {
    let mut region_of = vec![0; indices.len()];
    let mut next = 0;
    for t in 0..indices.len() {
        if region_of[t] == 0 {
            next += 1;
            flood(
                indices,
                standing,
                chain_keys,
                is_feature_edge,
                &[t as i32],
                next,
                &mut region_of,
                None,
            )?;
        }
    }

    Ok(region_of)
}
