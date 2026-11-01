//! The two sides a chain's rolling ball rolls between, as triangles of the pass's mesh, read for
//! `rolling_ball_setbacks.rs` (which walks the ball between them and says why the ball refuses a
//! pass where a chain has no two sides).
//!
//! **The caller's face regions, where it gives them.** A side is the region the pick shows the
//! user across the edge: the face beside it, and every face that carries on smoothly from it. The
//! caller knows those regions (it floods them for the pick's highlight at its bend threshold) and
//! may hand them to the build as one region number per triangle of the mesh
//! (`band_gap_entry_points::build_pick_with_side_regions`). Each chain's side A is then every
//! region holding a left triangle of its edges, side B every region holding a right one: a side
//! may span several regions (a face rim's outside runs round the part's walls, one region each).
//! A chain whose two sides share a region has no two sides to roll between.
//!
//! **Why a labelling and not side lists.** The kernel does not build the caller's chains as they
//! were handed in: a pick is split where its setbacks step, its closed chains are turned to start
//! at a corner, and the pinch ladder probes one chain at a time, so a list of sides per caller's
//! chain would not line up with the chains a pass builds. A region per triangle answers for any
//! chain on that mesh, however it was split, because no split moves a triangle. It holds only on
//! the mesh it labels: a build on a mesh an earlier cut rewrote (a chain at a time, rounds first)
//! is handed none.
//!
//! **The kernel's own flood, where it is given none.** Each side is the triangles beside the
//! chain's edges flooded over every edge that is neither a feature edge (the pass's own
//! `is_feature_edge`) nor an edge of a chain in the pass. Where the build's feature edges let a
//! side flow round a smooth blend into the other (the T Handle's pocket wall and crossbar, whose
//! regions the pick's flat tolerance splits but whose blend the build reads as one surface), the
//! two floods meet and the chain has no two sides.

use super::triangle_edges::{self, StandingMap};

/// Why `side_regions` cannot label the mesh's `triangles`: the route's reason, or nothing where
/// it labels every triangle (or there are none).
fn labels_the_mesh(side_regions: Option<&[u32]>, triangles: usize) -> Result<(), String> {
    match side_regions {
        Some(labels) if labels.len() != triangles => Err(format!(
            "the side regions label {} triangles where the mesh has {triangles}",
            labels.len()
        )),
        _ => Ok(()),
    }
}

/// Chain `k`'s two sides, by triangle index: side A (from the triangles `lefts`) and side B (from
/// `rights`), read from `side_regions` where given (see the file header) and flooded over the
/// edges `crosses` allows where not. Err with the route's reason where the two are one.
pub fn of_the_chain(
    indices: &[(i32, i32, i32)],
    standing: &StandingMap,
    k: usize,
    (lefts, rights): (&[i32], &[i32]),
    side_regions: Option<&[u32]>,
    crosses: &dyn Fn(i32, i32) -> bool,
) -> Result<(Vec<bool>, Vec<bool>), String> {
    labels_the_mesh(side_regions, indices.len())?;
    let Some(labels) = side_regions else {
        let in_a = flood(indices, standing, lefts, crosses);
        let in_b = flood(indices, standing, rights, crosses);
        if in_a.iter().zip(&in_b).any(|(&a, &b)| a && b) {
            return Err(format!("chain {k}: its two sides are one surface"));
        }
        return Ok((in_a, in_b));
    };

    let regions_of = |beside: &[i32]| -> Vec<u32> {
        let mut regions: Vec<u32> = beside.iter().map(|&t| labels[t as usize]).collect();
        regions.sort_unstable();
        regions.dedup();
        regions
    };
    let (regions_a, regions_b) = (regions_of(lefts), regions_of(rights));
    if let Some(shared) = regions_a.iter().find(|r| regions_b.contains(r)) {
        return Err(format!(
            "chain {k}: its two sides are one face region ({shared})"
        ));
    }
    let in_regions =
        |regions: &[u32]| -> Vec<bool> { labels.iter().map(|r| regions.contains(r)).collect() };

    Ok((in_regions(&regions_a), in_regions(&regions_b)))
}

/// Which triangles a walk from `seeds` reaches over edges `crosses` allows, by triangle index.
fn flood(
    indices: &[(i32, i32, i32)],
    standing: &StandingMap,
    seeds: &[i32],
    crosses: &dyn Fn(i32, i32) -> bool,
) -> Vec<bool> {
    let mut inside = vec![false; indices.len()];
    let mut stack: Vec<i32> = Vec::new();
    for &seed in seeds {
        if !inside[seed as usize] {
            inside[seed as usize] = true;
            stack.push(seed);
        }
    }
    while let Some(t) = stack.pop() {
        for (a, b) in triangle_edges::corners(indices[t as usize]) {
            if !crosses(a, b) {
                continue;
            }
            for &next in standing
                .get(&triangle_edges::key(a, b))
                .map_or(&[][..], Vec::as_slice)
            {
                if !inside[next as usize] {
                    inside[next as usize] = true;
                    stack.push(next);
                }
            }
        }
    }

    inside
}
