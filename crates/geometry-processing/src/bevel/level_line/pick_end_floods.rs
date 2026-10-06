//! The end regions of a pick of SEVERAL chains, read after the side tagging has grouped their
//! sides: each free end's region becomes a group of its own (`BandGroup::end_of`), or joins the
//! side group it lands on. Ports `Bevel/LevelLine/PickEndFloods.cs`.

use std::collections::HashSet;

use vector_math::Vector3;

use super::band_gap_rounds_first::chain_senses;
use super::band_side_tagging::{FIRST_END_TAG, SECOND_END_TAG};
use super::band_side_tagging_ends::{end_regions_of, SideTaggingError};
use super::{BandGroup, ChainField};

/// C# `PickEndFloods.FloodPickEnds`: every FREE end of an open chain (an end no other chain of the
/// pick reaches) that stands on an end face gets that face's region flooded exactly as the chain
/// alone would flood it, tagged as a new group after the side groups with the one chain as its
/// member and `end_of` naming it; a free end whose end face is another chain's side joins that
/// side group instead (`join_the_side_it_ends_on`). `end_rays` and `end_face_reaches` are `None`
/// where C# passes null.
#[allow(clippy::too_many_arguments)]
pub fn flood_pick_ends(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain_edges: &[Vec<(i32, i32)>],
    is_feature_edge: &dyn Fn(i32, i32) -> bool,
    end_rays: Option<&[(bool, bool)]>,
    tags: &mut [i32],
    groups: &mut Vec<BandGroup>,
    end_face_reaches: Option<&[ChainField]>,
) -> Result<(), SideTaggingError> {
    // Every end region is read on the SIDE tagging alone: an end group tagged for one chain would
    // read to the next chain's flood as foreign surface and stop it short of an end face the two
    // share.
    let sides = groups.len();
    let side_tags = tags.to_vec();
    let is_free = |k: usize, v: i32| {
        !(0..chain_edges.len())
            .any(|j| j != k && chain_edges[j].iter().any(|e| e.0 == v || e.1 == v))
    };
    for k in 0..chain_edges.len() {
        let edges = &chain_edges[k];
        if edges.is_empty() || edges[0].0 == edges[edges.len() - 1].1 {
            continue;
        }

        let own: HashSet<i32> = groups[..sides]
            .iter()
            .filter(|g| g.members.contains(&(k as i32)))
            .map(|g| g.tag)
            .collect();
        let reach = end_face_reaches.map(|r| &r[k]);
        let rays = end_rays.map(|r| r[k]).unwrap_or_default();
        let ends = end_regions_of(
            positions,
            indices,
            edges,
            is_feature_edge,
            &side_tags,
            &|t| own.contains(&t),
            rays,
            reach,
        )?;
        let last = edges[edges.len() - 1].1;
        for (vertex, end_tag) in [(edges[0].0, FIRST_END_TAG), (last, SECOND_END_TAG)] {
            if !is_free(k, vertex) {
                continue;
            }

            // Two free ends on ONE end face are one end region: one group with both chains as
            // members, as two chains bounding one side region are one side group.
            let region: Vec<usize> = (0..tags.len()).filter(|&t| ends[t] == end_tag).collect();
            if region.is_empty() {
                let other_end = if end_tag == FIRST_END_TAG {
                    last
                } else {
                    edges[0].0
                };
                let other_end_free = is_free(k, other_end);
                join_the_side_it_ends_on(
                    positions,
                    indices,
                    edges,
                    is_feature_edge,
                    &side_tags,
                    &own,
                    rays,
                    end_tag,
                    k as i32,
                    sides,
                    groups,
                    reach,
                    other_end_free,
                )?;
                continue;
            }

            // C# `groups.FindIndex(sides, ...)`: the first end group the region already carries.
            let shared =
                (sides..groups.len()).find(|&g| region.iter().any(|&t| tags[t] == groups[g].tag));
            if let Some(s) = shared {
                let joined = &groups[s];
                let mut side_of = joined.side_of_chain.clone();
                side_of.entry(k as i32).or_insert(joined.tag);
                let mut members = joined.members.clone();
                if !members.contains(&(k as i32)) {
                    members.push(k as i32);
                }
                members.sort();
                // C#'s initialiser names only these four, so the rest return to their defaults.
                groups[s] = BandGroup {
                    tag: joined.tag,
                    members,
                    side_of_chain: side_of,
                    end_of: joined.end_of,
                    ..BandGroup::default()
                };
            }

            let tag = match shared {
                Some(s) => groups[s].tag,
                None => groups.len() as i32 + 1,
            };
            let mut any = false;
            for &t in &region {
                if tags[t] == 0 {
                    tags[t] = tag;
                    any = true;
                }
            }

            if any && shared.is_none() {
                let mut side_of = indexmap::IndexMap::new();
                side_of.insert(k as i32, tag);
                groups.push(BandGroup {
                    tag,
                    members: vec![k as i32],
                    side_of_chain: side_of,
                    end_of: k as i32,
                    ..BandGroup::default()
                });
            }
        }
    }

    Ok(())
}

/// C# `PickEndFloods.JoinTheSideItEndsOn`: a free end whose end face is ANOTHER chain's side
/// region, so the end flood carved nothing: the end region is flooded again reading the other
/// chains' sides as open, and the chain joins each side group that region lies in as an end
/// member. An OBLIQUE end (one carrying the end ray) floods nothing; it joins every side group
/// round its end vertex without flooding, as a grow-over member when its chain is concave.
#[allow(clippy::too_many_arguments)]
fn join_the_side_it_ends_on(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    edges: &[(i32, i32)],
    is_feature_edge: &dyn Fn(i32, i32) -> bool,
    side_tags: &[i32],
    own: &HashSet<i32>,
    end_rays: (bool, bool),
    end_tag: i32,
    k: i32,
    sides: usize,
    groups: &mut [BandGroup],
    end_face_reach: Option<&ChainField>,
    other_end_free: bool,
) -> Result<(), SideTaggingError> {
    let sides = sides as i32;
    let oblique = if end_tag == FIRST_END_TAG {
        end_rays.0
    } else {
        end_rays.1
    };
    let mut grows_over = false;
    if oblique {
        if !other_end_free {
            return Ok(());
        }

        let mut as_chain: Vec<i32> = edges.iter().map(|e| e.0).collect();
        as_chain.push(edges[edges.len() - 1].1);
        grows_over =
            chain_senses(positions, indices, &[as_chain], Some(&[false]))[0] == Some(false);
    }

    let mut landed: Vec<i32> = if oblique {
        let end_vertex = if end_tag == FIRST_END_TAG {
            edges[0].0
        } else {
            edges[edges.len() - 1].1
        };
        (0..side_tags.len())
            .filter(|&t| {
                let (a, b, c) = indices[t];
                (a == end_vertex || b == end_vertex || c == end_vertex)
                    && side_tags[t] != 0
                    && side_tags[t] <= sides
                    && !own.contains(&side_tags[t])
            })
            .map(|t| side_tags[t])
            .collect()
    } else {
        let open: Vec<i32> = side_tags
            .iter()
            .map(|&t| {
                if t != 0 && t <= sides && !own.contains(&t) {
                    0
                } else {
                    t
                }
            })
            .collect();
        let ends = end_regions_of(
            positions,
            indices,
            edges,
            is_feature_edge,
            &open,
            &|t| own.contains(&t),
            end_rays,
            end_face_reach,
        )?;
        (0..side_tags.len())
            .filter(|&t| ends[t] == end_tag && side_tags[t] != 0 && side_tags[t] <= sides)
            .map(|t| side_tags[t])
            .collect()
    };
    // C# `Distinct().OrderBy(t => t)`: the order is the sort's, so the dedup after it is the same.
    landed.sort();
    landed.dedup();

    for tag in landed {
        // A landed tag is a side tag, and every side tag names a side group.
        let index = groups
            .iter()
            .position(|g| g.tag == tag)
            .expect("a side tag names a side group");
        let joined = &groups[index];
        if joined.members.contains(&k) {
            continue;
        }

        let with_k = |list: &[i32]| {
            let mut list = list.to_vec();
            list.push(k);
            list.sort();
            list
        };
        let mut side_of = joined.side_of_chain.clone();
        side_of.insert(k, joined.tag);
        // C#'s initialiser leaves out `CarriedEnds`, so it returns to its default.
        groups[index] = BandGroup {
            tag: joined.tag,
            members: with_k(&joined.members),
            side_of_chain: side_of,
            both_sided: joined.both_sided.clone(),
            end_of: joined.end_of,
            end_members: with_k(&joined.end_members),
            grow_over_members: if grows_over {
                with_k(&joined.grow_over_members)
            } else {
                joined.grow_over_members.clone()
            },
            ..BandGroup::default()
        };
    }

    Ok(())
}
