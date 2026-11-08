//! Pass B of a rounds-first pick, read on the rounds' mesh: the concave creases joined over the
//! round at each knife tip, and the feature edges the second pass floods by; the helpers they
//! read are in `band_gap_rounds_first`.

use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use solidmesh_math::Vector3;

use super::band_gap_rounds_first::{
    chain_senses, crease_on_the_end_face, kept, plane_of, rounds_first_tips, same_plane,
    surviving_piece, the_rounds_band, the_rounds_chains, the_tips_end_face, OrientedPlane,
};
use super::end_face_sense::COPLANAR_TOLERANCE;
use super::seam_edges::FeatureEdgeTest;
use super::triangle_edges;
use super::{GapFillResult, LevelLineMesh};

/// The concave creases read again on
/// the rounds' mesh, or `None` where a tip cannot be read so.
pub fn creases_after_the_rounds(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: Option<&[bool]>,
    rounds: &LevelLineMesh,
) -> Option<Vec<Vec<i32>>> {
    creases_after_the_rounds_closed(positions, indices, chains, closed, rounds, None)
}

/// The creases after the rounds, with whether each is closed: at each knife tip, the two concave
/// chains ending there are joined into one chain over the round - the first chain's surviving
/// piece up to where it enters the rounds' band, the crease where the band meets the tip's end
/// face, and the second chain's surviving piece out to its far end. Concave chains at no tip are
/// carried as they are; the convex chains are not in the answer. Each answer chain's closed flag
/// is added to `result_closed` where it is given.
pub fn creases_after_the_rounds_closed(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: Option<&[bool]>,
    rounds: &LevelLineMesh,
    mut result_closed: Option<&mut Vec<bool>>,
) -> Option<Vec<Vec<i32>>> {
    let tips = rounds_first_tips(positions, indices, chains, closed);
    if tips.is_empty() {
        return None;
    }

    let senses = chain_senses(positions, indices, chains, closed);
    let input_standing = triangle_edges::standing_map(indices);
    let convex_edges: HashSet<(i32, i32)> = (0..chains.len())
        .filter(|&k| senses[k] == Some(true))
        .flat_map(|k| {
            chains[k]
                .windows(2)
                .map(|w| triangle_edges::key(w[0], w[1]))
        })
        .collect();

    let kept = kept(rounds);
    let at = &rounds.vertices;
    let standing = triangle_edges::standing_map(&kept);
    let mut neighbours: HashMap<i32, Vec<i32>> = HashMap::new();
    for &(a, b) in standing.keys() {
        neighbours.entry(a).or_default().push(b);
        neighbours.entry(b).or_default().push(a);
    }

    let mut result: Vec<Vec<i32>> = Vec::new();
    let mut joined: HashSet<i32> = HashSet::new();
    let rounds_chains = the_rounds_chains(positions, indices, chains, closed);
    for (&tip, concave) in &tips {
        if concave.len() != 2 || concave.iter().any(|k| joined.contains(k)) {
            return None;
        }

        // Each chain oriented to END at the tip, and the tip's end face read off its end edge.
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
        let face = the_tips_end_face(
            positions,
            indices,
            &input_standing,
            &convex_edges,
            tip,
            &to_tip,
        )?;

        // The crease on the rounds' mesh: an edge between a triangle on the end face and one off it.
        let is_crease = |a: i32, b: i32| {
            standing.get(&triangle_edges::key(a, b)).is_some_and(|on| {
                on.len() == 2
                    && same_plane(face, plane_of(at, kept[on[0] as usize]))
                        != same_plane(face, plane_of(at, kept[on[1] as usize]))
            })
        };

        let mut pieces: Vec<Vec<i32>> = Vec::new();
        for chain in &to_tip {
            pieces.push(surviving_piece(chain, at, &is_crease, &neighbours)?);
        }

        let first = &pieces[0];
        // A one-vertex piece has no second-to-last vertex to read the tip's direction from; that
        // tip is not read.
        if first.len() < 2 {
            return None;
        }

        let over = crease_on_the_end_face(
            first[first.len() - 1],
            first[first.len() - 2],
            pieces[1][pieces[1].len() - 1],
            &is_crease,
            &neighbours,
        )?;

        let mut one = first.clone();
        one.extend(over);
        one.extend(pieces[1].iter().rev().skip(1));
        result.push(one);
        if let Some(flags) = result_closed.as_deref_mut() {
            flags.push(false);
        }
        joined.extend(concave.iter().copied());
    }

    for k in 0..chains.len() {
        let k32 = k as i32;
        if !joined.contains(&k32) && !rounds_chains.contains(&k32) {
            result.push(chains[k].clone());
            if let Some(flags) = result_closed.as_deref_mut() {
                flags.push(closed.is_some_and(|c| c[k]));
            }
        }
    }

    Some(result)
}

/// The feature edges of pass B, on the rounds' mesh. The
/// input's own edges keep their word. The rounds' band is NOT a feature inside or along its rails
/// (where it meets a convex chain's side face); where the band meets any other face it IS one. Any
/// other edge the rounds made is a feature where its two faces are not coplanar. `None` where the
/// rounds carry no mesh or a rounds chain's edge stands on no triangle. The
/// reading owns what it reads, so it can be handed to the pass-B build as its feature test.
pub fn feature_edges_after_the_rounds(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: Option<&[bool]>,
    is_feature_edge: Option<FeatureEdgeTest>,
    rounds: &GapFillResult,
) -> Option<FeatureEdgeTest> {
    let input_standing = triangle_edges::standing_map(indices);
    let mut side_planes: Vec<OrientedPlane> = Vec::new();
    for k in the_rounds_chains(positions, indices, chains, closed) {
        let chain = &chains[k as usize];
        let count = if closed.is_some_and(|c| c[k as usize]) {
            chain.len()
        } else {
            chain.len().saturating_sub(1)
        };
        for i in 0..count {
            let key = triangle_edges::key(chain[i], chain[(i + 1) % chain.len()]);
            for &t in input_standing.get(&key)? {
                side_planes.push(plane_of(positions, indices[t as usize]));
            }
        }
    }

    let at = rounds.mesh.as_ref()?.borrow().vertices.clone();
    let (triangles, band) = the_rounds_band(positions, indices, rounds);
    let standing = triangle_edges::standing_map(&triangles);
    let count = positions.len() as i32;

    Some(Rc::new(move |a: i32, b: i32| {
        let key = triangle_edges::key(a, b);
        if a < count && b < count && input_standing.contains_key(&key) {
            return is_feature_edge.as_ref().is_some_and(|f| f(a, b));
        }

        let Some(on) = standing.get(&key).filter(|on| on.len() == 2) else {
            return false;
        };

        let in_band = on.iter().filter(|&&t| band[t as usize]).count();
        if in_band == 2 {
            return false;
        }

        if in_band == 1 {
            let off = if band[on[0] as usize] { on[1] } else { on[0] };
            let other = plane_of(&at, triangles[off as usize]);
            return !side_planes.iter().any(|&p| same_plane(p, other));
        }

        1.0 - plane_of(&at, triangles[on[0] as usize])
            .0
            .dot(plane_of(&at, triangles[on[1] as usize]).0)
            > COPLANAR_TOLERANCE
    }))
}
