//! The side tagging of the band: which side of the chain each triangle stands on, and the end
//! regions: its tags, the
//! both-sided-members switch, the model's scale, the outward direction at a chain end and a facet
//! normal, the flood out from seeds ([`flood`]) and the connected regions ([`connected_regions`]);
//! which side of a chain a point lies on ([`side_of`]) and the first both-sided member at a mixed
//! corner ([`first_both_sided_at_a_mixed_corner`]); the side and end floods land with later
//! packages.

use vector_math::Vector3;

use std::collections::{HashSet, VecDeque};

use super::triangle_edges::{self, StandingMap};
use super::{
    chain_on_the_mesh, end_face_sense, BandGroup, ChainEdgeNotOnTwoTriangles, LevelLineWork,
    WorkError,
};

/// The end flood's tag at the chain's FIRST vertex.
pub const FIRST_END_TAG: i32 = 3;

/// The end flood's tag at the chain's LAST vertex.
pub const SECOND_END_TAG: i32 = 4;

/// In the end regions' working tags: a triangle on a side another chain of the pick seeds - closed
/// to the end flood but where it stands past the end.
pub const ANOTHER_PICKS_SIDE: i32 = -1;

/// Whether a both-sided member of a pick is built (a switch that travels with the build, so a
/// field of the budget in force): on by default and with no budget in force;
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

/// Tags with `tag` every untagged triangle reachable from `seeds`
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

            // Every edge of a triangle stands in the map built from the same list, so the lookup
            // always finds it.
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

/// Every triangle's CONNECTED REGION - the triangles
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

/// Which side of the chain (1 left, 2 right, as
/// `chain_on_the_mesh::sides_of` names them) `point` lies on, read from the geometry: at the chain
/// edge nearest the point, the side whose triangle runs away from the edge the way the point does.
/// What names the side of a loop of a both-sided member, whose two sides carry one tag.
pub fn side_of(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    standing: &StandingMap,
    chain_edges: &[(i32, i32)],
    point: Vector3,
) -> Result<i32, ChainEdgeNotOnTwoTriangles> {
    let mut best = f64::MAX;
    let mut side = 1;
    for &(from, to) in chain_edges {
        let a = positions[from as usize];
        let along = positions[to as usize] - a;
        let t = ((point - a).dot(along) / along.length_squared()).clamp(0.0, 1.0);
        let foot = a + (along * t);
        let distance = (point - foot).length_squared();
        if distance >= best {
            continue;
        }

        best = distance;
        let (left, right) = chain_on_the_mesh::sides_of(indices, standing, from, to)?;
        let away = |triangle: i32| {
            let tri = indices[triangle as usize];
            let third = if tri.0 != from && tri.0 != to {
                tri.0
            } else if tri.1 != from && tri.1 != to {
                tri.1
            } else {
                tri.2
            };
            let off = positions[third as usize] - a;
            let inward = off - (along * (off.dot(along) / along.length_squared()));
            (point - foot).dot(inward.get_normal())
        };
        side = if away(left) >= away(right) { 1 } else { 2 };
    }

    Ok(side)
}

/// The lowest both-sided member that meets
/// another chain at a MIXED corner - a vertex where the picked edges do not all fold the same way
/// (a pocket rim with its inner edge, an L's re-entrant corner) - or -1. At a mixed corner the
/// both-sided blend came back self-intersecting or its room read small, so such a member is still
/// read as a chain that does not divide its sides. No positions reads none.
pub fn first_both_sided_at_a_mixed_corner(
    positions: Option<&[Vector3]>,
    indices: &[(i32, i32, i32)],
    chain_edges: &[Vec<(i32, i32)>],
    groups: &[BandGroup],
) -> Result<i32, ChainEdgeNotOnTwoTriangles> {
    let mut both_sided: Vec<i32> = groups
        .iter()
        .flat_map(|g| g.both_sided.iter().copied())
        .collect();
    both_sided.sort();
    both_sided.dedup();
    let Some(positions) = positions else {
        return Ok(-1);
    };
    if both_sided.is_empty() {
        return Ok(-1);
    }

    let standing = triangle_edges::standing_map(indices);
    for k in both_sided {
        let mut own: Vec<i32> = Vec::new();
        for &(from, to) in &chain_edges[k as usize] {
            for v in [from, to] {
                if !own.contains(&v) {
                    own.push(v);
                }
            }
        }

        let others: HashSet<i32> = (0..chain_edges.len())
            .filter(|&j| j as i32 != k)
            .flat_map(|j| chain_edges[j].iter().flat_map(|&(f, t)| [f, t]))
            .collect();
        for corner in own.into_iter().filter(|v| others.contains(v)) {
            let mut senses: HashSet<bool> = HashSet::new();
            for &(from, to) in chain_edges.iter().flatten() {
                if from != corner && to != corner {
                    continue;
                }

                let (left, right) = chain_on_the_mesh::sides_of(indices, &standing, from, to)?;
                if left < 0 || right < 0 {
                    continue;
                }

                senses.insert(end_face_sense::is_convex(
                    positions,
                    indices[left as usize],
                    indices[right as usize],
                    from,
                ));
            }

            if senses.len() > 1 {
                return Ok(k);
            }
        }
    }

    Ok(-1)
}
