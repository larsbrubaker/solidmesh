//! The room foreign features leave a chain: the size at which the band first reaches an edge that
//! is not the chain's own: the constants and switch, the knife test ([`is_a_knife`]), the foreign
//! edges themselves
//! ([`foreign_edges`]), whether an edge meets the chain at an end ([`meets_the_chain_at_an_end`]),
//! the clusters of short edges at a chain's ends ([`cluster_at_the_ends`]) and the unindexed edge
//! list ([`every_edge`]); the room readings over those edges are in `foreign_edge_room_readings.rs`.
//!
//! **The chain's own crease past a free end.** On the rolling ball's tags
//! (`LevelLineMesh::sides_from_the_ball`) the chain's crease running on unpicked past a free end is
//! a 1|2 edge away from the chain, and it is read like any other foreign edge: against the band's
//! real field, the tapered round's (`rolling_ball_free_ends`: every setback capped by its chain
//! point's distance from the end vertex), never exempted for where it runs. The only edge exempt
//! is the one through the end vertex of an end that runs out ([`meets_the_chain_at_an_end`]): the
//! field reads zero there to rounding, either way, which no reading can tell from a reach. Every
//! other edge is read because the geometry decides it:
//! - **Run on straight**, each of its points reads its own distance from the end vertex (a ball
//!   through the end vertex never reaches past it along the chain's line), so it never sets the
//!   room, at any size.
//! - **Turned back alongside the chain** (a hairpin round a sliver of one side coming to a point
//!   at the end vertex - the T Handle's free tip, where the wall's sliver between the picked crease
//!   and its unpicked twin is 0.0052 wide a third of a unit from the tip), the tapered round does
//!   reach it: the other side's region wraps round past the sliver, and that side's band, a
//!   setback of 0.31 there at a size of 12, would be cut on it beyond the crease, across a crease
//!   the user did not pick. The room is then honest, and an exemption by topology (the run of 1|2
//!   edges leaving the end vertex) would exempt exactly that crease. Where the ball's contacts
//!   never sweep that side's triangles behind the crease, they leave the side before the room is
//!   read (`rolling_ball_sweep`), so the crease is read as the other side's edge with no side
//!   beyond it, against that side's own field; where the side's band within reach lies on both
//!   sides of the other contact's plane the pass is refused before any room is read.
//!
//! An end that carries an end region (3 or 4) keeps no exemption at its end vertex: its setback
//! there is the cap's, not zero, so a 1|2 edge through it - a second crease between the two sides
//! meeting the chain's end corner - reads below zero at every size and the room is zero. The band
//! does cut across such a crease at that corner, whatever the size; which crease the round should
//! stop at there is the corner of three creases, the ball's corner rule.

use std::cell::{Cell, RefCell};
use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use super::band_side_tagging::{FIRST_END_TAG, SECOND_END_TAG};
use super::{end_face_sense, triangle_edges};
use super::{
    ChainField, ForeignEdgeIndex, GrowingCreaseRuns, LevelLineMesh, LevelLineTriangle, WorkError,
};

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
    /// Makes the indexed room read EVERY foreign edge,
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

/// The foreign feature edges - where a side's region ends on
/// something that is not the chain, and not at the chain's own ends. A side flooded by the band's
/// own tagging only ever ends at a chain edge (which puts the other side across it) or at a
/// feature edge (which leaves untagged material across it), so an edge onto tag 0 is exactly a
/// foreign feature edge, and it stays one through every split, since a split's children inherit
/// their parent's tag. A side tagged from the rolling ball's (`LevelLineMesh::sides_from_the_ball`)
/// is the caller's face regions, which end at region boundaries: onto tag 0, or onto the other
/// side away from the chain (another chain's crease between the same two regions, or the chain's
/// own crease running on past a free end), where no tag 0 stands. Such a 1|2 edge is foreign too:
/// the band cutting across it would cut through a crease the user did not pick. Such a side also
/// loses what the ball's contacts never sweep (`rolling_ball_sweep`), which can end it onto tag 0
/// across a smooth surface, not at a crease: that edge is read like any other onto tag 0 - no knife
/// edge (`is_a_knife_edge`: it is not folded at all), no growing crease (`GrowingCreaseRuns::grows`
/// asks for an opposite-sense crease, which a flat edge is not, so `bounds_of` keeps the edge
/// itself) - and filed on the side against that side's own field (`foreign_edge_room_per_side`),
/// so it can only lower the room. A crease a growing
/// neighbour grows over is replaced by what it grows into; the growing runs read along the way are
/// handed to `growing_runs` when given (`seam_epsilon` 0 is the usual value).
///
/// The mesh and the field are shared (`Rc`) because the growing runs keep both, live: the runs
/// walk the working mesh and read the field's distance after this returns. The
/// runs' distance delegate cannot return the field's work error, so it is held aside and handed
/// up here once the walk that met it is done.
pub fn foreign_edges(
    mesh: &Rc<RefCell<LevelLineMesh>>,
    chain_edges: &[(i32, i32)],
    field: &Rc<ChainField>,
    chain_is_closed: bool,
    seam_epsilon: f64,
    growing_runs: Option<&mut Vec<GrowingCreaseRuns>>,
) -> Result<Vec<(i32, i32)>, WorkError> {
    // Read in the mesh's canonical edge order, though the answer would not change in any other:
    // the room these edges are asked for is the LEAST closest approach over them, and a minimum is
    // the same number whichever order it is taken in.
    let chain_keys: HashSet<(i32, i32)> = chain_edges
        .iter()
        .map(|&(from, to)| triangle_edges::key(from, to))
        .collect();
    let m = mesh.borrow();
    let end_cluster = cluster_at_the_ends(&m, chain_edges, chain_is_closed, seam_epsilon);

    // THE HONEST ROOM AT AN END. An end that stands on an END FACE carries an end region (tag 3 at
    // the first vertex, 4 at the last), and an end tag reads as a picked side's: a side-to-end fold
    // (1|3) is not foreign, while the end region's own boundary with the rest of the part (3|0) IS,
    // so a short end edge or a small end face bounds the room by size. That end's run-out exemption
    // goes with it. An end with NO end region - a fade, dying or running on along its crease - keeps
    // the exemption. Safe to read 3 and 4 this way because this reading is of ONE chain, where they
    // can only be end tags, never a second chain's sides.
    let first_end_has_a_face =
        !chain_is_closed && m.triangles.iter().any(|t| t.tag == FIRST_END_TAG);
    let last_end_has_a_face =
        !chain_is_closed && m.triangles.iter().any(|t| t.tag == SECOND_END_TAG);
    // A GROWING end (no end region: the end face is of the opposite sense and never cut) is bounded
    // by its crease stretch's far end, not by the stretch itself.
    let growing_ends = if chain_is_closed {
        None
    } else {
        Some(end_face_sense::growing_crease_stretch(
            &m,
            chain_edges,
            !first_end_has_a_face,
            !last_end_has_a_face,
            None,
        ))
    };
    let mut edges = Vec::new();
    // Every crease of one growing run is replaced by the SAME far bounds, so without this a run of
    // a hundred creases listed its region's rim a hundred times over, and each reading of the room
    // read every copy. Only the first of an edge is kept: the room is the least over the edges, so
    // a repeat can never change it, nor which edge names it.
    let mut listed: HashSet<(i32, i32)> = HashSet::new();
    let mut growing: Option<GrowingCreaseRuns> = None;
    let failed: Rc<RefCell<Option<WorkError>>> = Rc::new(RefCell::new(None));
    let chain_ends: HashSet<i32> = if chain_is_closed {
        HashSet::new()
    } else {
        HashSet::from([chain_edges[0].0, chain_edges[chain_edges.len() - 1].1])
    };
    for edge in m.edges() {
        if chain_keys.contains(&edge)
            || growing_ends
                .as_ref()
                .is_some_and(|growing_ends| growing_ends.contains(&edge))
        {
            continue;
        }

        let standing = m.triangles_on(edge.0, edge.1);
        let tags = || standing.iter().map(|&t| m.triangles[t as usize].tag);
        let sides_meet_off_the_chain =
            m.sides_from_the_ball && tags().any(|t| t == 1) && tags().any(|t| t == 2);
        if !sides_meet_off_the_chain && (!tags().any(|t| t != 0) || !tags().any(|t| t == 0)) {
            continue;
        }

        if !chain_is_closed
            && meets_the_chain_at_an_end(
                &m,
                chain_edges,
                edge,
                end_cluster.as_deref(),
                !first_end_has_a_face,
                !last_end_has_a_face,
            )
        {
            continue;
        }

        if is_a_knife_edge(&m, edge) {
            continue;
        }

        // A crease the neighbour GROWS over bounds the room by what it grows into - for now on a
        // pick of ONE OPEN chain only, the only shape the clip and caps are built for.
        let runs = growing.get_or_insert_with(|| {
            let field = Rc::clone(field);
            let failed = Rc::clone(&failed);
            GrowingCreaseRuns::new(
                Rc::clone(mesh),
                chain_edges[0],
                Rc::new(move |p| match field.evaluate(p) {
                    Ok(sample) => sample.distance,
                    Err(error) => {
                        // The closure answers only a distance, so it keeps the first error and
                        // answers NaN; whoever checks `failed` hands the error up, and the budget
                        // stays spent (or cancelled) meanwhile.
                        failed.borrow_mut().get_or_insert(error);
                        f64::NAN
                    }
                }),
                Some(chain_edges.to_vec()),
                None,
            )
        });
        let bounds = if chain_is_closed {
            None
        } else {
            runs.bounds_of(edge, &chain_ends)?
        };
        if let Some(error) = failed.borrow_mut().take() {
            return Err(error);
        }

        if let Some(bounds) = bounds {
            edges.extend(bounds.into_iter().filter(|&bound| listed.insert(bound)));
            continue;
        }

        if listed.insert(edge) {
            edges.push(edge);
        }
    }

    drop(m);
    if let Some(growing) = growing {
        // A growing neighbour's RIDGE out of its run's near end is what the cap extends past (the
        // tie), read as the pierce room and not as a wall. Where its near end is also on the
        // chain's own crease, the ridge is that crease turning on round a concave corner - the
        // fade's road ("Every Chamfer Works.mcx"). The ridge may be met before its growing crease
        // is, so it is dropped once the walk is done.
        edges.retain(|&e| !growing.is_ridge_off_the_chains_crease(e));
        if let Some(error) = failed.borrow_mut().take() {
            return Err(error);
        }

        if let Some(growing_runs) = growing_runs {
            growing_runs.push(growing);
        }
    }

    Ok(edges)
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
/// cluster of short edges. The usual call passes no end cluster and both run-outs true.
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

/// Every edge the indexes hold that the chain is not exempt from -
/// the unindexed edge list, as (low, high, extra unit setback 0).
pub fn every_edge(
    indexes: &[std::rc::Rc<ForeignEdgeIndex>],
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

/// Whether a mesh edge between a tagged (filled)
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
