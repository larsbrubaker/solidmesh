//! How the Thingi10K sweep chooses what to build on a mesh: face rims taken off the smooth
//! regions, and feature chains walked from a feature edge. Ports
//! `Tests/GeometryProcessingTests/LevelLine/SweepPick.cs` (`SweepPick` and `SweepPicks`).
//!
//! C#'s `Dictionary` and lazy `yield return` iterators become `IndexMap` (insertion order, which is
//! what C# enumerates) and eager vectors; every `Random.Next` is drawn in C#'s order from
//! `DotNetRandom`, so a seed names the same picks.

use std::collections::{HashMap, HashSet};

use indexmap::IndexMap;
use vector_math::dotnet_random::DotNetRandom;
use vector_math::Vector3;

use crate::sweep_mesh_measures::{corners_of, key, standing};

/// How far the chain may turn at a vertex and still be the same chain.
const CHAIN_TURN_DEGREES: f64 = 30.0;

/// The kind a row carries for a few edges cut out of a longer crease - the fade's case.
pub const PARTIAL_KIND: &str = "partial";

/// The kind a row carries for a whole open crease, junction to junction.
pub const WHOLE_OPEN_KIND: &str = "open";

/// What the sweep hands the kernel as one pick: a face rim's loops, or one feature chain.
#[derive(Debug, Clone)]
pub struct SweepPick {
    pub kind: String,
    /// The rings or runs to build, in order; empty when the pick could not be made.
    pub loops: Vec<Vec<i32>>,
    /// One flag per loop: whether it runs from its last vertex back to its first, exactly as
    /// `BandGap::build_pick` asks for it.
    pub closed: Vec<bool>,
    /// Why there is nothing to build, when there is nothing to build.
    pub aside: Option<String>,
}

impl SweepPick {
    fn nothing(kind: &str, aside: &str) -> SweepPick {
        SweepPick {
            kind: kind.to_string(),
            loops: Vec::new(),
            closed: Vec::new(),
            aside: Some(aside.to_string()),
        }
    }

    /// A pick whose every loop comes back to where it started - a face rim, or a closed crease.
    pub fn shut(kind: &str, loops: Vec<Vec<i32>>) -> SweepPick {
        let closed = loops.iter().map(|_| true).collect();
        SweepPick {
            kind: kind.to_string(),
            loops,
            closed,
            aside: None,
        }
    }

    /// A pick of one OPEN run: a crease walked end to end, or a few edges cut out of a longer one.
    pub fn run(kind: &str, run: Vec<i32>) -> SweepPick {
        SweepPick {
            kind: kind.to_string(),
            loops: vec![run],
            closed: vec![false],
            aside: None,
        }
    }
}

type Standing = HashMap<(i32, i32), Vec<i32>>;

/// The feature edges as possible starts, and the feature edges standing at each vertex.
type FeatureGraph = (Vec<(i32, i32)>, HashMap<i32, Vec<i32>>);

/// Up to half face rims and half closed feature chains from `random`, seeded off the model so a
/// row can be re-run, then the OPEN picks from `open_random`.
///
/// The open picks are drawn from a stream of their OWN and appended AFTER the closed ones, so the
/// closed picks' draw is untouched and "model N pick K" still names in this report what it named
/// in every earlier one. Their own indices start where the closed picks' end.
pub fn picks(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    is_feature: &dyn Fn(i32, i32) -> bool,
    random: &mut DotNetRandom,
    count: i32,
    open_random: &mut DotNetRandom,
    open_count: i32,
) -> Vec<SweepPick> {
    let wanted = (count / 2).max(1);
    let mut picks = Vec::new();
    picks.extend(face_rim_picks(indices, is_feature, random, wanted));
    picks.extend(chain_picks(
        positions,
        indices,
        is_feature,
        random,
        count - wanted,
    ));
    picks.extend(open_picks(
        positions,
        indices,
        is_feature,
        open_random,
        open_count,
    ));
    picks
}

/// The smooth regions: triangles joined across every edge that is not a feature edge.
fn regions(
    indices: &[(i32, i32, i32)],
    is_feature: &dyn Fn(i32, i32) -> bool,
    standing: &Standing,
) -> Vec<i32> {
    let mut region = vec![-1; indices.len()];
    let mut next = 0;
    let mut stack: Vec<usize> = Vec::new();
    for seed in 0..indices.len() {
        if region[seed] >= 0 {
            continue;
        }

        let tag = next;
        next += 1;
        region[seed] = tag;
        stack.push(seed);
        while let Some(t) = stack.pop() {
            for (a, b) in corners_of(indices[t]) {
                if is_feature(a, b) {
                    continue;
                }

                for &other in &standing[&key(a, b)] {
                    if region[other as usize] < 0 {
                        region[other as usize] = tag;
                        stack.push(other as usize);
                    }
                }
            }
        }
    }

    region
}

fn face_rim_picks(
    indices: &[(i32, i32, i32)],
    is_feature: &dyn Fn(i32, i32) -> bool,
    random: &mut DotNetRandom,
    wanted: i32,
) -> Vec<SweepPick> {
    let standing = standing(indices);
    let region = regions(indices, is_feature, &standing);
    // Regions are numbered in order of their lowest triangle, which is also the order C#'s
    // dictionary first meets them in.
    let mut members: Vec<Vec<i32>> = Vec::new();
    for (t, &tag) in region.iter().enumerate() {
        if tag as usize == members.len() {
            members.push(Vec::new());
        }

        members[tag as usize].push(t as i32);
    }

    let mut candidates: Vec<Vec<i32>> = members.into_iter().filter(|m| m.len() >= 2).collect();
    for i in (1..candidates.len()).rev() {
        let j = random.next(i as i32 + 1) as usize;
        candidates.swap(i, j);
    }

    let mut picks = Vec::new();
    let mut made = 0;
    for group in &candidates {
        if made == wanted {
            break;
        }

        match boundary_loops(indices, &standing, group) {
            None => picks.push(SweepPick::nothing("face", "BoundaryNotALoop")),
            // a region that is the whole closed shell has no rim to pick.
            Some(loops) if loops.is_empty() => continue,
            Some(loops) => picks.push(SweepPick::shut("face", loops)),
        }

        made += 1;
    }

    picks
}

/// Every boundary loop of a region, outer and holes, each an ordered ring walked with the region
/// on the LEFT - which is just each boundary directed edge taken in its own triangle's winding.
/// None when the boundary does not close into rings (a region pinched at a vertex).
fn boundary_loops(
    indices: &[(i32, i32, i32)],
    standing: &Standing,
    group: &[i32],
) -> Option<Vec<Vec<i32>>> {
    let members: HashSet<i32> = group.iter().copied().collect();
    let mut out: IndexMap<i32, Vec<i32>> = IndexMap::new();
    let mut edges: i64 = 0;
    for &t in group {
        for (a, b) in corners_of(indices[t as usize]) {
            let neighbours = &standing[&key(a, b)];
            let mine = neighbours.len() == 2
                && members.contains(&neighbours[0])
                && members.contains(&neighbours[1]);
            if mine {
                continue;
            }

            out.entry(a).or_default().push(b);
            edges += 1;
        }
    }

    if edges == 0 {
        return Some(Vec::new());
    }

    let mut loops = Vec::new();
    while edges > 0 {
        let start = *out.iter().find(|(_, to)| !to.is_empty())?.0;
        let mut ring: Vec<i32> = Vec::new();
        let mut at = start;
        loop {
            let onward = out.get_mut(&at)?;
            if onward.is_empty() {
                return None;
            }

            ring.push(at);
            let to = onward.remove(0);
            edges -= 1;
            at = to;
            if at == start {
                break;
            }

            if ring.len() as i64 > edges + ring.len() as i64 + 1 {
                return None;
            }
        }

        if ring.len() < 3 {
            return None;
        }

        loops.push(ring);
    }

    Some(loops)
}

/// A feature chain: from a random feature edge, out both ways while exactly one other feature edge
/// carries on at the vertex within a turn of 30 degrees. A chain that comes back to where it
/// started is a pick here; one that does not belongs to [`open_picks`].
fn chain_picks(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    is_feature: &dyn Fn(i32, i32) -> bool,
    random: &mut DotNetRandom,
    wanted: i32,
) -> Vec<SweepPick> {
    let (starts, at_vertex) = feature_graph(indices, is_feature);

    let mut picks = Vec::new();
    for _ in 0..wanted {
        if starts.is_empty() {
            picks.push(SweepPick::nothing("chain", "NoFeatureEdges"));
            continue;
        }

        let (from, to) = starts[random.next(starts.len() as i32) as usize];
        let (ring, closed) = walk(positions, &at_vertex, from, to);
        picks.push(if closed && ring.len() >= 3 {
            SweepPick::shut("chain", vec![ring])
        } else {
            SweepPick::nothing("chain", "OpenChain")
        });
    }

    picks
}

/// The OPEN picks, alternating between the two kinds an open run comes in: a PARTIAL pick - one to
/// three consecutive edges cut out of a longer crease, so the crease carries on past both ends,
/// which is the fade's case - and the WHOLE open chain walked junction to junction, whose ends sit
/// on an end face and which the kernel is expected to refuse `OpenChainEnds` by name today.
///
/// A partial is cut out of whichever crease the draw lands on, closed or open: what makes it the
/// fade's case is that the crease CARRIES ON past both of its ends, which is as true of a few edges
/// of a closed rim as of a few edges in the middle of an open one.
fn open_picks(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    is_feature: &dyn Fn(i32, i32) -> bool,
    random: &mut DotNetRandom,
    wanted: i32,
) -> Vec<SweepPick> {
    let (starts, at_vertex) = feature_graph(indices, is_feature);

    let mut picks = Vec::new();
    for made in 0..wanted {
        let kind = if made % 2 == 0 {
            PARTIAL_KIND
        } else {
            WHOLE_OPEN_KIND
        };
        if starts.is_empty() {
            picks.push(SweepPick::nothing(kind, "NoFeatureEdges"));
            continue;
        }

        let (from, to) = starts[random.next(starts.len() as i32) as usize];
        let (run, closed) = walk(positions, &at_vertex, from, to);

        if kind == PARTIAL_KIND {
            picks.push(match partial(&run, closed, random) {
                Some(part) => SweepPick::run(PARTIAL_KIND, part),
                None => SweepPick::nothing(PARTIAL_KIND, "CreaseTooShortToCutInto"),
            });
        } else {
            picks.push(if !closed && run.len() >= 2 {
                SweepPick::run(WHOLE_OPEN_KIND, run)
            } else {
                SweepPick::nothing(WHOLE_OPEN_KIND, "ChainClosesOnItself")
            });
        }
    }

    picks
}

/// One to three consecutive edges of `run` with at least one edge of the crease left over at EACH
/// end, or None when the crease is too short to cut into that way.
fn partial(run: &[i32], closed: bool, random: &mut DotNetRandom) -> Option<Vec<i32>> {
    if closed {
        // A closed crease of n vertices has n edges; anything shorter than the whole ring carries
        // on past both of its ends, wrapping round the join.
        if run.len() < 5 {
            return None;
        }

        let take = 1 + random.next(3);
        let start = random.next(run.len() as i32);
        return Some(
            (0..=take)
                .map(|k| run[(start + k) as usize % run.len()])
                .collect(),
        );
    }

    // An open crease of n edges: the cut runs from vertex i to vertex i + take with i >= 1 and
    // i + take <= n - 1, so an edge of the crease is left at each end for the bevel to fade onto.
    let edges = run.len() as i32 - 1;
    if edges < 3 {
        return None;
    }

    let most = 3.min(edges - 2);
    let cut = 1 + random.next(most);
    let at = (1 + random.next(edges - 1 - cut)) as usize;
    Some(run[at..at + cut as usize + 1].to_vec())
}

/// Every feature edge once as a possible start, and the feature edges standing at each vertex -
/// what both chain draws walk on.
fn feature_graph(
    indices: &[(i32, i32, i32)],
    is_feature: &dyn Fn(i32, i32) -> bool,
) -> FeatureGraph {
    let mut at_vertex: HashMap<i32, Vec<i32>> = HashMap::new();
    let mut seen: HashSet<(i32, i32)> = HashSet::new();
    let mut starts = Vec::new();
    for &t in indices {
        for (a, b) in corners_of(t) {
            let edge = key(a, b);
            if !is_feature(a, b) || !seen.insert(edge) {
                continue;
            }

            starts.push(edge);
            for (v, w) in [(a, b), (b, a)] {
                at_vertex.entry(v).or_default().push(w);
            }
        }
    }

    (starts, at_vertex)
}

fn walk(
    positions: &[Vector3],
    at_vertex: &HashMap<i32, Vec<i32>>,
    from: i32,
    to: i32,
) -> (Vec<i32>, bool) {
    let mut ring = vec![from, to];
    let mut visited: HashSet<i32> = [from, to].into_iter().collect();
    let mut closed = false;

    // Forward, then (if it does not close) backward from the start, so an open chain is still the
    // whole run it belongs to, junction to junction, and not the half of it the draw happened to
    // start in.
    loop {
        let previous = ring[ring.len() - 2];
        let at = ring[ring.len() - 1];
        let onward = onward(positions, at_vertex, previous, at);
        if onward < 0 {
            break;
        }

        if onward == ring[0] {
            closed = true;
            break;
        }

        if !visited.insert(onward) {
            break;
        }

        ring.push(onward);
    }

    if closed {
        return (ring, closed);
    }

    loop {
        let previous = ring[1];
        let at = ring[0];
        let onward = onward(positions, at_vertex, previous, at);
        if onward < 0 || !visited.insert(onward) {
            break;
        }

        ring.insert(0, onward);
    }

    (ring, closed)
}

/// The one feature edge that carries on at `at`, or -1 if there is not exactly one.
fn onward(
    positions: &[Vector3],
    at_vertex: &HashMap<i32, Vec<i32>>,
    previous: i32,
    at: i32,
) -> i32 {
    let coming = (positions[at as usize] - positions[previous as usize]).get_normal();
    let limit = (CHAIN_TURN_DEGREES * std::f64::consts::PI / 180.0).cos();
    let mut found = -1;
    for &next in &at_vertex[&at] {
        if next == previous {
            continue;
        }

        if (positions[next as usize] - positions[at as usize])
            .get_normal()
            .dot(coming)
            < limit
        {
            continue;
        }

        if found >= 0 {
            // a junction: two ways on is no way on.
            return -1;
        }

        found = next;
    }

    found
}
