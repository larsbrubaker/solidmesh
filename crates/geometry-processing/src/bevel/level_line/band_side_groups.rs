//! A pick's SIDE GROUPS: one topological flood over the input from every chain's two seeds, with
//! seeds that reach the same triangles merged into one group, then the end floods. Ports
//! `BandSideTagging.GroupSides` and `BandSideTagging.HasABothSidedMember` from
//! `Bevel/LevelLine/BandSideTagging.cs`; the floods it runs are in `band_side_tagging` and
//! `band_side_tagging_ends`, a pick's end groups in `pick_end_floods`.

use std::collections::{BTreeMap, HashSet};

use indexmap::IndexMap;
use vector_math::Vector3;

use super::band_side_tagging::{both_sided_members_build, connected_regions, flood};
use super::band_side_tagging_ends::{flood_ends, EndReach, SideTaggingError};
use super::pick_end_floods::flood_pick_ends;
use super::triangle_edges;
use super::{chain_on_the_mesh, BandGroup, ChainField};

/// What [`group_sides`] reads: the tag of every triangle, the groups, and `undivided` - the first
/// chain one connected region of which reaches both its sides, or -1 when every chain divides its
/// own (C#'s two `out` parameters).
pub type GroupedSides = (Vec<i32>, Vec<BandGroup>, i32);

/// C# `BandSideTagging.HasABothSidedMember`: true when a pick of several chains builds with a
/// `both_sided` member - a chain whose two sides fall in one group only through another chain.
/// False with the both-sided build off. `chains` `None` is C#'s null.
pub fn has_a_both_sided_member(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: Option<&[Vec<i32>]>,
    closed: &[bool],
    is_feature_edge: Option<&dyn Fn(i32, i32) -> bool>,
) -> Result<bool, SideTaggingError> {
    let chains = match chains {
        Some(chains) if both_sided_members_build() && chains.len() >= 2 => chains,
        _ => return Ok(false),
    };

    let chain_edges: Vec<Vec<(i32, i32)>> = chains
        .iter()
        .enumerate()
        .map(|(k, chain)| chain_on_the_mesh::chain_edges(chain, closed[k]))
        .collect();
    let no_feature = |_: i32, _: i32| false;
    let (_, groups, _) = group_sides(
        indices,
        &chain_edges,
        is_feature_edge.unwrap_or(&no_feature),
        true,
        Some(positions),
        None,
        None,
    )?;

    Ok(groups.iter().any(|group| !group.both_sided.is_empty()))
}

/// C# `BandSideTagging.GroupSides`: one topological flood over the input from every chain's two
/// seeds, not crossing a chain edge or a feature edge, with seeds that reach the same triangles
/// merged into one group. Groups are numbered in seed order - chain 0's left is 1 and its right is
/// 2 - so everything that reads tags 1 and 2 of a single chain reads the same thing. `positions`,
/// `end_rays` and `end_face_reaches` are `None` where C# passes null.
pub fn group_sides(
    indices: &[(i32, i32, i32)],
    chain_edges: &[Vec<(i32, i32)>],
    is_feature_edge: &dyn Fn(i32, i32) -> bool,
    end_floods: bool,
    positions: Option<&[Vector3]>,
    end_rays: Option<&[(bool, bool)]>,
    end_face_reaches: Option<&[std::rc::Rc<ChainField>]>,
) -> Result<GroupedSides, SideTaggingError> {
    let standing = triangle_edges::standing_map(indices);
    let chain_keys: HashSet<(i32, i32)> = chain_edges
        .iter()
        .flat_map(|edges| edges.iter().map(|e| triangle_edges::key(e.0, e.1)))
        .collect();

    // The seeds, in the order the groups are numbered in: chain 0 left, chain 0 right, chain 1 left, ...
    let mut seed_chain = Vec::new();
    let mut seed_side = Vec::new();
    let mut seed_triangles: Vec<Vec<i32>> = Vec::new();
    for (k, edges) in chain_edges.iter().enumerate() {
        let mut left = Vec::new();
        let mut right = Vec::new();
        for &(from, to) in edges {
            let (l, r) = chain_on_the_mesh::sides_of(indices, &standing, from, to)?;
            left.push(l);
            right.push(r);
        }

        seed_chain.push(k as i32);
        seed_side.push(1);
        seed_triangles.push(left);
        seed_chain.push(k as i32);
        seed_side.push(2);
        seed_triangles.push(right);
    }

    // Flooded one seed at a time; a flood only enters untagged triangles, so a later seed standing
    // in an earlier seed's region finds that region's tag already on it, and the two are the same
    // group.
    let mut tags = vec![0; indices.len()];
    let mut parent: Vec<i32> = vec![0];
    fn find(parent: &mut [i32], mut x: i32) -> i32 {
        while parent[x as usize] != x {
            parent[x as usize] = parent[parent[x as usize] as usize];
            x = parent[x as usize];
        }

        x
    }

    let mut tag_of_seed = vec![0; seed_triangles.len()];
    for i in 0..seed_triangles.len() {
        let mut already: Vec<i32> = seed_triangles[i]
            .iter()
            .map(|&t| tags[t as usize])
            .filter(|&t| t != 0)
            .collect();
        already.sort();
        already.dedup();
        let tag = if already.is_empty() {
            parent.len() as i32
        } else {
            already[0]
        };
        if already.is_empty() {
            parent.push(parent.len() as i32);
        }

        flood(
            indices,
            &standing,
            &chain_keys,
            is_feature_edge,
            &seed_triangles[i],
            tag,
            &mut tags,
            None,
        )?;
        for &other in &already {
            // Two tags on one seed's triangles: one side of one chain lying in two floods'
            // territory makes those floods one group.
            let a = find(&mut parent, tag);
            let b = find(&mut parent, other);
            if a != b {
                parent[a.max(b) as usize] = a.min(b);
            }
        }

        tag_of_seed[i] = tag;
    }

    // Renumber the merged tags so the groups come out 1, 2, 3, ... in seed order.
    let mut final_of: IndexMap<i32, i32> = IndexMap::new();
    let mut members: Vec<BTreeMap<i32, i32>> = Vec::new();
    for i in 0..tag_of_seed.len() {
        let root = find(&mut parent, tag_of_seed[i]);
        let id = match final_of.get(&root) {
            Some(&id) => id,
            None => {
                let id = final_of.len() as i32 + 1;
                final_of.insert(root, id);
                members.push(BTreeMap::new());
                id
            }
        };

        // A chain seeding one group by BOTH its sides does not divide its sides; the side kept
        // here is the first one seen, which is only ever read to say which chain it was.
        members[(id - 1) as usize]
            .entry(seed_chain[i])
            .or_insert(seed_side[i]);
    }

    for tag in tags.iter_mut() {
        if *tag != 0 {
            let root = find(&mut parent, *tag);
            *tag = final_of[&root];
        }
    }

    // A chain seeding one group by both its sides is either UNDIVIDED - one connected region holds
    // a triangle of each side - or BOTH-SIDED: its two sides are separate regions that another
    // chain's side joined into one group. The connected regions are read apart here, and only when
    // a chain falls in one group at all.
    let mut undivided = -1;
    let mut both_sided_of: IndexMap<i32, Vec<i32>> = IndexMap::new();
    let mut region_of: Option<Vec<i32>> = None;
    for k in 0..chain_edges.len() {
        let left_root = find(&mut parent, tag_of_seed[2 * k]);
        if left_root != find(&mut parent, tag_of_seed[2 * k + 1]) {
            continue;
        }

        if region_of.is_none() {
            region_of = Some(connected_regions(
                indices,
                &standing,
                &chain_keys,
                is_feature_edge,
            )?);
        }
        let region_of = region_of.as_deref().unwrap_or_default();
        let left_regions: HashSet<i32> = seed_triangles[2 * k]
            .iter()
            .map(|&t| region_of[t as usize])
            .collect();
        if chain_edges.len() == 1
            || seed_triangles[2 * k + 1]
                .iter()
                .any(|&t| left_regions.contains(&region_of[t as usize]))
        {
            undivided = k as i32;
            break;
        }

        let id = final_of[&left_root];
        both_sided_of.entry(id).or_default().push(k as i32);
    }

    let mut groups = Vec::with_capacity(members.len());
    for (i, chain_sides) in members.iter().enumerate() {
        let id = i as i32 + 1;
        groups.push(BandGroup {
            tag: id,
            members: chain_sides.keys().copied().collect(),
            side_of_chain: chain_sides.iter().map(|(&k, &s)| (k, s)).collect(),
            both_sided: both_sided_of.get(&id).cloned().unwrap_or_default(),
            ..BandGroup::default()
        });
    }

    // End tags 3 and 4 only on a pick of ONE chain: with several, 3 and 4 are chain 1's sides, so
    // each end region there is a group of its own numbered after the side groups.
    if let Some(positions) = positions.filter(|_| end_floods && undivided < 0) {
        if chain_edges.len() == 1 {
            flood_ends(
                positions,
                indices,
                &standing,
                &chain_keys,
                is_feature_edge,
                &chain_edges[0],
                &mut tags,
                None,
                (false, false),
                EndReach::default(),
                false,
            )?;
        } else if chain_edges.len() > 1 {
            flood_pick_ends(
                positions,
                indices,
                chain_edges,
                is_feature_edge,
                end_rays,
                &mut tags,
                &mut groups,
                end_face_reaches,
            )?;
        }
    }

    Ok((tags, groups, undivided))
}
