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
//! the mesh it labels: a build a chain at a time hands each later chain the labelling carried onto
//! the mesh the earlier chains rewrote (`side_regions_after_a_build`), each rewritten triangle in
//! its own triangle's region and each earlier round a region of its own; a rounds-first build's
//! passes are handed none.
//!
//! **The kernel's own flood, where it is given none.** Each side is the triangles beside the
//! chain's edges flooded over every edge that is neither a feature edge (the pass's own
//! `is_feature_edge`) nor an edge of a chain in the pass. Where the build's feature edges let a
//! side flow round into the other, the two floods are one surface: a crease that dies out (the
//! sides blend into one smooth surface past the chain's end), or a side that wraps round the part
//! into the other far from the chain (a cube whose back edges are blended). The ball only reads
//! each side near its own contacts, which it follows over the side from where it seeded them, so
//! that surface is divided where the two floods meet: both flooded at once by distance over the
//! surface from the chain, each triangle on the side nearer to it. Distance is what decides,
//! never how many triangles lie between, so a side cut into fine strips keeps its own surface
//! (a ring count let the far side take a finely cut front). Round a dying end every face is then
//! one side's or the other's, so the end is a free end and the round tapers onto the end vertex as
//! the angle closes (`rolling_ball_free_ends`). A face the division splits that the ball meets
//! from both sides (a flat end the feature edges call smooth) leaves it no walk there.
//!
//! **The division's limit.** The distance runs from edge midpoint to edge midpoint across each
//! triangle, an upper bound on the true distance over the surface that can sit off it by about a
//! triangle's size, so the dividing line can stand that far from where the two sides are truly
//! equally far. The edges along that line are no rim of the part, but each side's surface ends
//! there: a contact that comes to rest on the line reads as resting on the side's rim. The ball's
//! contacts stand within its reach of the chain, so the line matters only where the other side is
//! about as near (a dying end, where the contacts close onto the end vertex the line starts from).

use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};

use vector_math::Vector3;

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
/// edges `crosses` allows where not, divided where the floods meet. Err with the route's reason
/// where the regions put the two in one.
pub fn of_the_chain(
    positions: &[Vector3],
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
            return Ok(divided_where_they_meet(
                positions,
                indices,
                standing,
                (lefts, rights),
                crosses,
            ));
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

/// One edge reached by the division's flood: how far over the surface from its side's seeds, the
/// side (1 for A, 2 for B), and the edge. Ordered by distance, then side, then edge, so the flood
/// never depends on hashing and a tie goes to side A.
#[derive(Clone, Copy, PartialEq)]
struct Reached {
    distance: f64,
    side: u8,
    edge: (i32, i32),
}

impl Eq for Reached {}

impl Ord for Reached {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.distance
            .total_cmp(&other.distance)
            .then(self.side.cmp(&other.side))
            .then(self.edge.cmp(&other.edge))
    }
}

impl PartialOrd for Reached {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

/// The one surface two floods share divided where they meet (see the file header): both sides
/// flooded at once by distance over the surface, each triangle on the side nearer to it. The
/// distance runs from edge midpoint to edge midpoint across each triangle (the shortest such path
/// from the chain edge beside the side's seed triangle), so a side cut into long thin strips is no
/// farther across than a coarse one. A chain edge or feature edge is never crossed, so each seed
/// keeps its side.
fn divided_where_they_meet(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    standing: &StandingMap,
    (lefts, rights): (&[i32], &[i32]),
    crosses: &dyn Fn(i32, i32) -> bool,
) -> (Vec<bool>, Vec<bool>) {
    let middle = |(a, b): (i32, i32)| (positions[a as usize] + positions[b as usize]) * 0.5;
    let mut best: HashMap<(i32, i32), (f64, u8)> = HashMap::new();
    let mut queue: BinaryHeap<Reverse<Reached>> = BinaryHeap::new();
    let offer = |best: &mut HashMap<(i32, i32), (f64, u8)>,
                 queue: &mut BinaryHeap<Reverse<Reached>>,
                 reached: Reached| {
        let better = best.get(&reached.edge).is_none_or(|&(distance, side)| {
            reached
                .distance
                .total_cmp(&distance)
                .then(reached.side.cmp(&side))
                .is_lt()
        });
        if better {
            best.insert(reached.edge, (reached.distance, reached.side));
            queue.push(Reverse(reached));
        }
    };

    // Each seed's edges, measured from its chain edge (the edge it shares with the other side's
    // seed of the same chain edge).
    for (&left, &right) in lefts.iter().zip(rights) {
        let shared = shared_edge(indices[left as usize], indices[right as usize]);
        for (seed, side) in [(left, 1u8), (right, 2u8)] {
            let from = shared.map(middle);
            for (a, b) in triangle_edges::corners(indices[seed as usize]) {
                let edge = triangle_edges::key(a, b);
                let distance = from.map_or(0.0, |from| (middle(edge) - from).length());
                offer(
                    &mut best,
                    &mut queue,
                    Reached {
                        distance,
                        side,
                        edge,
                    },
                );
            }
        }
    }

    while let Some(Reverse(reached)) = queue.pop() {
        if best.get(&reached.edge) != Some(&(reached.distance, reached.side)) {
            continue;
        }
        let (a, b) = reached.edge;
        if !crosses(a, b) {
            continue;
        }
        let here = middle(reached.edge);
        for &next in standing.get(&reached.edge).map_or(&[][..], Vec::as_slice) {
            for (c, d) in triangle_edges::corners(indices[next as usize]) {
                let edge = triangle_edges::key(c, d);
                if edge == reached.edge {
                    continue;
                }
                let distance = reached.distance + (middle(edge) - here).length();
                offer(
                    &mut best,
                    &mut queue,
                    Reached {
                        distance,
                        side: reached.side,
                        edge,
                    },
                );
            }
        }
    }

    // Each triangle on the side of its nearest edge; the seeds on their own.
    let in_the_floods = flood(indices, standing, lefts, crosses);
    let mut side: Vec<u8> = indices
        .iter()
        .zip(&in_the_floods)
        .map(|(&t, &inside)| {
            if !inside {
                return 0;
            }
            triangle_edges::corners(t)
                .into_iter()
                .filter_map(|(a, b)| best.get(&triangle_edges::key(a, b)).copied())
                .min_by(|x, y| x.0.total_cmp(&y.0).then(x.1.cmp(&y.1)))
                .map_or(0, |(_, side)| side)
        })
        .collect();
    for &t in lefts {
        side[t as usize] = 1;
    }
    for &t in rights {
        side[t as usize] = 2;
    }

    (
        side.iter().map(|&s| s == 1).collect(),
        side.iter().map(|&s| s == 2).collect(),
    )
}

/// The edge two triangles share, by key, or `None` where they share none.
fn shared_edge(t: (i32, i32, i32), u: (i32, i32, i32)) -> Option<(i32, i32)> {
    let in_u = |v: i32| v == u.0 || v == u.1 || v == u.2;
    triangle_edges::corners(t)
        .into_iter()
        .find(|&(a, b)| in_u(a) && in_u(b))
        .map(|(a, b)| triangle_edges::key(a, b))
}
