//! The room foreign features leave a chain: the size at which the band first reaches an edge that
//! is not the chain's own. Ports `Bevel/LevelLine/ForeignEdgeRoom.cs` (a C# static class): so far
//! its constants and switch, the knife test ([`is_a_knife`]) and the clusters of short edges at a
//! chain's ends ([`cluster_at_the_ends`]), and whether an edge meets the chain at an end
//! ([`meets_the_chain_at_an_end`]) and the unindexed edge list ([`every_edge`]); the room readings
//! land with later packages.

use std::cell::Cell;
use std::collections::{HashMap, HashSet};

use super::{ForeignEdgeIndex, LevelLineMesh, LevelLineTriangle};

/// Fixed, so the room is the same bits every time it is asked for.
pub const ROOM_BISECTION_STEPS: i32 = 40;

/// How nearly equal the chain's unit setbacks must be for the room to be read in one pass rather
/// than bisected - relative, because a setback is a length.
pub const UNIFORM_SETBACK_TOLERANCE: f64 = 1e-12;

/// How far the room's size bracket may be doubled out before the room counts as unbounded.
pub const ROOM_BRACKET_DOUBLINGS: i32 = 20;

/// How much the index search's prune gives away, so no rounding of the ratio it prunes by can drop
/// an edge the EXACT keep test would have kept. Pure slack: an extra edge read changes no answer.
pub const PRUNE_SLACK: f64 = 1e-9;

/// How far past a right angle (as the cosine between the two unit normals) a crease must turn to
/// be a knife: well above the 1e-16 an exact right angle reads, well below any real knife's cosine.
pub const RIGHT_ANGLE_NOISE: f64 = 1e-9;

thread_local! {
    /// C# `[ThreadStatic] ReadEveryForeignEdge`: makes the indexed room read EVERY foreign edge,
    /// exactly as the unindexed one does - per thread, so one test cannot turn it on under another.
    static READ_EVERY_FOREIGN_EDGE: Cell<bool> = const { Cell::new(false) };
}

/// Whether this thread's indexed room reads every foreign edge.
pub fn read_every_foreign_edge() -> bool {
    READ_EVERY_FOREIGN_EDGE.with(Cell::get)
}

/// Sets this thread's read-every-foreign-edge switch.
pub fn set_read_every_foreign_edge(on: bool) {
    READ_EVERY_FOREIGN_EDGE.with(|cell| cell.set(on));
}

/// True when the crease `edge` between `side` and `far` is a KNIFE: convex, and turned past a
/// right angle by more than round-off.
pub fn is_a_knife(
    mesh: &LevelLineMesh,
    side: LevelLineTriangle,
    far: LevelLineTriangle,
    edge: (i32, i32),
) -> bool {
    let normal_of = |t: LevelLineTriangle| {
        let a = mesh.position(t.a);
        (mesh.position(t.b) - a)
            .cross(mesh.position(t.c) - a)
            .get_normal()
    };

    let side_normal = normal_of(side);
    let opposite = if far.a != edge.0 && far.a != edge.1 {
        far.a
    } else if far.b != edge.0 && far.b != edge.1 {
        far.b
    } else {
        far.c
    };
    let convex = side_normal.dot(mesh.position(opposite) - mesh.position(edge.0)) < 0.0;
    // Past a right angle by more than round-off: an exact right-angle crease reads its normals' dot
    // as +-1e-16, and a bare "< 0" let where the part sat decide whether it was a knife - Meeting
    // Edge Round's corner joined its rail across one only when turned, and its fill moved with it.
    convex && side_normal.dot(normal_of(far)) < -RIGHT_ANGLE_NOISE
}

/// Per end of an open chain (first, then last), the vertices joined to it by edges no longer than
/// `seam_epsilon`; `None` for a closed chain or a non-positive epsilon.
pub fn cluster_at_the_ends(
    mesh: &LevelLineMesh,
    chain_edges: &[(i32, i32)],
    chain_is_closed: bool,
    seam_epsilon: f64,
) -> Option<Vec<HashSet<i32>>> {
    if chain_is_closed || seam_epsilon.is_nan() || seam_epsilon <= 0.0 {
        return None;
    }

    let mut short_neighbours: HashMap<i32, Vec<i32>> = HashMap::new();
    for (low, high) in mesh.edges() {
        if (mesh.position(low) - mesh.position(high)).length() > seam_epsilon {
            continue;
        }

        join(&mut short_neighbours, low, high);
        join(&mut short_neighbours, high, low);
    }

    let mut clusters = Vec::new();
    for end in [chain_edges[0].0, chain_edges[chain_edges.len() - 1].1] {
        let mut cluster = HashSet::from([end]);
        let mut walking = vec![end];
        while let Some(at) = walking.pop() {
            let Some(neighbours) = short_neighbours.get(&at) else {
                continue;
            };

            for &neighbour in neighbours {
                if cluster.insert(neighbour) {
                    walking.push(neighbour);
                }
            }
        }

        clusters.push(cluster);
    }

    Some(clusters)
}

/// Adds `to` to `from`'s neighbour list.
pub fn join(neighbours: &mut HashMap<i32, Vec<i32>>, from: i32, to: i32) {
    neighbours.entry(from).or_default().push(to);
}

/// True when `edge` is a piece of an input edge through one of the chain's run-out ends (the first
/// end while `first_runs_out`, the last while `last_runs_out`), or lies wholly inside ONE end's
/// cluster of short edges. C#'s optional arguments (`endCluster = null`, both run-outs true) are
/// passed explicitly.
pub fn meets_the_chain_at_an_end(
    mesh: &LevelLineMesh,
    chain_edges: &[(i32, i32)],
    edge: (i32, i32),
    end_cluster: Option<&[HashSet<i32>]>,
    first_runs_out: bool,
    last_runs_out: bool,
) -> bool {
    let first = chain_edges[0].0;
    let last = chain_edges[chain_edges.len() - 1].1;
    let on = LevelLineMesh::merge(
        mesh.input_edge_behind(edge.0),
        mesh.input_edge_behind(edge.1),
    );
    if on.0 >= 0
        && ((first_runs_out && (on.0 == first || on.1 == first))
            || (last_runs_out && (on.0 == last || on.1 == last)))
    {
        return true;
    }

    let Some(end_cluster) = end_cluster else {
        return false;
    };

    // ONE end's cluster must hold the whole edge. Taking the two ends together would exempt an
    // edge running from a vertex at one end to a vertex at the other - the length of the chain
    // apart, and a wall like any other.
    end_cluster
        .iter()
        .any(|at_one_end| at_one_end.contains(&edge.0) && at_one_end.contains(&edge.1))
}

/// C# `ForeignEdgeRoom.EveryEdge`: every edge the indexes hold that the chain is not exempt from -
/// the unindexed edge list, as (low, high, extra unit setback 0).
pub fn every_edge(
    indexes: &[ForeignEdgeIndex],
    exempt: Option<&dyn Fn((i32, i32)) -> bool>,
) -> Vec<(i32, i32, f64)> {
    let mut every = Vec::new();
    for index in indexes {
        for &edge in index.edges() {
            if exempt.is_none_or(|exempt| !exempt(edge)) {
                every.push((edge.0, edge.1, 0.0));
            }
        }
    }

    every
}

/// C# `ForeignEdgeRoom.IsAKnife(mesh, edge)`: whether a mesh edge between a tagged (filled)
/// triangle and an untagged one is a knife - read with the tagged one as the side.
pub fn is_a_knife_edge(mesh: &LevelLineMesh, edge: (i32, i32)) -> bool {
    let standing = mesh.triangles_on(edge.0, edge.1);
    if standing.len() != 2 {
        return false;
    }

    let first = mesh.triangles[standing[0] as usize];
    let second = mesh.triangles[standing[1] as usize];
    if (first.tag == 0) == (second.tag == 0) {
        return false;
    }

    let (side, far) = if first.tag != 0 {
        (first, second)
    } else {
        (second, first)
    };

    is_a_knife(mesh, side, far, edge)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rust_only_the_read_every_edge_switch_is_per_thread() {
        set_read_every_foreign_edge(true);
        assert!(read_every_foreign_edge());
        assert!(!std::thread::spawn(read_every_foreign_edge)
            .join()
            .unwrap_or(true));
        set_read_every_foreign_edge(false);
    }
}
