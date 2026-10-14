//! The band side of a STEP CORNER: two CONVEX pieces of one picked chain meeting end to end where
//! their setbacks step. The piece with the LARGER setback carries its band past the step onto the
//! other piece's side faces, out to its own reach, so its band cuts those faces round the corner
//! as its round does. The groups it joins come from
//! `band_side_groups`.

use std::collections::HashSet;

use vector_math::Vector3;

use super::band_gap_rounds_first::chain_senses;
use super::chain_on_the_mesh::SETBACK_STEP_FRACTION;
use super::BandGroup;

/// Joins each step corner's larger-setback piece
/// to the other piece's side groups at that corner, counting the carried end in `carried_ends`.
/// Only where both pieces are convex: a concave piece's bead grows over its faces instead.
pub fn join_the_larger_setback_piece(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain_edges: &[Vec<(i32, i32)>],
    unit_setbacks: &[Vec<f64>],
    tags: &[i32],
    groups: &mut [BandGroup],
    step_corners: &HashSet<i32>,
) {
    let count = chain_edges.len();

    // Both pieces CONVEX: the larger round runs on past the step and cuts the other piece's side
    // face. A concave piece's bead grows over its faces instead, so a step between a convex and a
    // concave piece is not carried.
    let chains: Vec<Vec<i32>> = chain_edges
        .iter()
        .map(|e| {
            if e.is_empty() {
                Vec::new()
            } else {
                let mut chain: Vec<i32> = e.iter().map(|x| x.0).collect();
                chain.push(e[e.len() - 1].1);
                chain
            }
        })
        .collect();
    let open = vec![false; count];
    let senses = chain_senses(positions, indices, &chains, Some(&open));
    for k in 0..count {
        if senses[k] != Some(true) {
            continue;
        }

        let edges = &chain_edges[k];
        if edges.is_empty() || edges[0].0 == edges[edges.len() - 1].1 {
            continue;
        }

        let ends = [
            (edges[0].0, unit_setbacks[k][0]),
            (
                edges[edges.len() - 1].1,
                unit_setbacks[k][unit_setbacks[k].len() - 1],
            ),
        ];
        for (vertex, mine) in ends {
            // Only where the split cut a picked chain: two picked chains meeting at a corner (a
            // pair corner, a knife tip) hand over by the corner's own construction.
            if !step_corners.contains(&vertex) {
                continue;
            }

            let others: Vec<usize> = (0..count)
                .filter(|&j| {
                    j != k
                        && !chain_edges[j].is_empty()
                        && (chain_edges[j][0].0 == vertex
                            || chain_edges[j][chain_edges[j].len() - 1].1 == vertex)
                })
                .collect();
            if others.len() != 1 {
                continue;
            }

            let j = others[0];
            if senses[j] != Some(true) {
                continue;
            }

            let theirs = if chain_edges[j][0].0 == vertex {
                unit_setbacks[j][0]
            } else {
                unit_setbacks[j][unit_setbacks[j].len() - 1]
            };
            #[allow(clippy::neg_cmp_op_on_partial_ord)]
            if !(mine > theirs * (1.0 + SETBACK_STEP_FRACTION)) {
                continue;
            }

            let at_the_step: HashSet<i32> = (0..indices.len())
                .filter(|&t| {
                    let (a, b, c) = indices[t];
                    a == vertex || b == vertex || c == vertex
                })
                .map(|t| tags[t])
                .collect();
            let (k, j) = (k as i32, j as i32);
            for group in groups.iter_mut() {
                let carried = group.carried_ends.get(&k).copied();
                if group.end_of >= 0
                    || !group.members.contains(&j)
                    || (group.members.contains(&k) && carried.is_none())
                    || !at_the_step.contains(&group.tag)
                {
                    continue;
                }

                if carried.is_none() {
                    group.members.push(k);
                    group.members.sort();
                }
                group.side_of_chain.insert(k, group.tag);
                group
                    .carried_ends
                    .insert(k, carried.map_or(1, |ends| ends + 1));
            }
        }
    }
}
