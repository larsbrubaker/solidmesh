//! The room foreign features leave a chain: the size at which the band first reaches an edge that
//! is not the chain's own: the constants and switch, the knife test ([`is_a_knife`]), the foreign
//! edges themselves
//! ([`foreign_edges`]), whether an edge meets the chain at an end ([`meets_the_chain_at_an_end`]),
//! the clusters of short edges at a chain's ends ([`cluster_at_the_ends`]) and the unindexed edge
//! list ([`every_edge`]); the room readings over those edges are in `foreign_edge_room_readings.rs`.

use std::cell::{Cell, RefCell};
use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use vector_math::Vector3;

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

/// The least a crease must turn past a right angle (as the cosine between the two unit normals) to
/// be a knife: well above the 1e-16 an exact right angle reads from vertices placed exactly, well
/// below any real knife's cosine. [`right_angle_noise`] raises it to what the vertices' own
/// placement can tilt the two normals by.
pub const RIGHT_ANGLE_NOISE: f64 = 1e-9;

/// The relative precision a float-stored mesh's vertices are placed to
/// (`LevelLineMesh::input_is_float_stored`, set for every mesh the model hands over): half a unit
/// in the last place of a single-precision coordinate (2^-24).
pub const SINGLE_PRECISION_PLACEMENT: f64 = 5.960_464_477_539_063e-8;

/// How far the two triangles' unit normals' dot can stray from a right angle's 0 through the
/// placement of their vertices alone, never below [`RIGHT_ANGLE_NOISE`]: a crease reading within
/// it is a right angle as far as the input can say. `side` and `far` are the corners of the INPUT
/// triangles the crease's two triangles descend from (`LevelLineMesh::input_triangle_behind`).
///
/// Rounding a coordinate `c` to single precision moves it by at most
/// `|c| SINGLE_PRECISION_PLACEMENT`, so it moves a corner `p` out of its triangle's plane (unit
/// normal `n`) by at most `d = SINGLE_PRECISION_PLACEMENT (|n.x p.x| + |n.y p.y| + |n.z p.z|)`:
/// read per axis, because a face square to an axis at a small coordinate (a cut face on y = -3 of
/// a part 160 tall) is placed to that coordinate's precision, not the part's. Moving one corner by
/// `d` out of the plane tilts the normal by `d / h`, `h` that corner's height over the opposite
/// edge (`2 * area / opposite edge`); the three corners together tilt it by at most the sum. Near a
/// right angle the dot of two unit normals changes by at most the sum of their tilts.
///
/// A piece the band's cut split off an input triangle lies in that triangle's plane and inside it,
/// so its heights are never larger: read on the piece, the bound would never be smaller than the
/// input triangle's own, and on a sliver far larger, so the bound is read on the input triangle.
///
/// The bound grows with where the part sits (a coordinate of 160 carries a thousand times a
/// coordinate of 0.16's rounding) and with how thin the input triangles are, which no fixed
/// constant can follow: the T Handle's exact right-angle rim, its vertices single-precision values
/// near 160 on triangles a unit high, reads dots from -1.3e-7 to 1.1e-7, so a fixed 1e-9 split one
/// straight rail into alternate knife and plain edges, while a boolean's sliver cut face on a real
/// knife (dot -0.13) must stay a knife, which a bound read from the largest coordinate on every axis
/// (0.18 there) would not. A degenerate triangle has no normal to read: its bound is infinite and
/// the crease is no knife, as its NaN normal already read.
pub fn right_angle_noise(side: [Vector3; 3], far: [Vector3; 3]) -> f64 {
    let tilt = |t: [Vector3; 3]| {
        let cross = (t[1] - t[0]).cross(t[2] - t[0]);
        let twice_area = cross.length();
        if twice_area.is_nan() || twice_area <= 0.0 {
            return f64::INFINITY;
        }

        let normal = cross * (1.0 / twice_area);
        let mut sum = 0.0;
        for corner in 0..3 {
            let p = t[corner];
            // How far rounding this corner's coordinates can move it out of the plane.
            let off_plane = SINGLE_PRECISION_PLACEMENT
                * ((normal.x.abs() * p.x.abs())
                    + (normal.y.abs() * p.y.abs())
                    + (normal.z.abs() * p.z.abs()));
            let opposite = (t[(corner + 2) % 3] - t[(corner + 1) % 3]).length();
            sum += off_plane * opposite / twice_area;
        }
        sum
    };

    RIGHT_ANGLE_NOISE.max(tilt(side) + tilt(far))
}

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

/// True when the crease `edge` between triangles `side` and `far` (indices) is a KNIFE: convex,
/// and turned past a right angle by more than the vertices' placement can tilt it
/// ([`right_angle_noise`], read on the input triangles the two descend from).
pub fn is_a_knife(mesh: &LevelLineMesh, side: i32, far: i32, edge: (i32, i32)) -> bool {
    let input_corners = |t: i32| mesh.input_triangle_behind(t).map(|v| mesh.position(v));
    let normal_of = |t: LevelLineTriangle| {
        let a = mesh.position(t.a);
        (mesh.position(t.b) - a)
            .cross(mesh.position(t.c) - a)
            .get_normal()
    };

    let (side_index, far_index) = (side, far);
    let (side, far) = (mesh.triangles[side as usize], mesh.triangles[far as usize]);
    let side_normal = normal_of(side);
    let opposite = if far.a != edge.0 && far.a != edge.1 {
        far.a
    } else if far.b != edge.0 && far.b != edge.1 {
        far.b
    } else {
        far.c
    };
    let convex = side_normal.dot(mesh.position(opposite) - mesh.position(edge.0)) < 0.0;
    // Past a right angle by more than the vertices' placement can tilt it: a bare "< 0" let where
    // the part sat decide whether it was a knife - Meeting Edge Round's corner joined its rail
    // across one only when turned, and its fill moved with it.
    // Vertices placed exactly (a mesh built in doubles) carry no placement error to allow for. A
    // piece lies in its input triangle's plane, so the input triangle's allowance is the piece's.
    convex && {
        let noise = if mesh.input_is_float_stored {
            right_angle_noise(input_corners(side_index), input_corners(far_index))
        } else {
            RIGHT_ANGLE_NOISE
        };
        side_normal.dot(normal_of(far)) < -noise
    }
}

/// The foreign feature edges - where a side's region ends on
/// something that is not the chain, and not at the chain's own ends. A side flooded by the band's
/// own tagging only ever ends at a chain edge (which puts the other side across it) or at a
/// feature edge (which leaves untagged material across it), so an edge onto tag 0 is exactly a
/// foreign feature edge, and it stays one through every split, since a split's children inherit
/// their parent's tag. A crease a growing
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
        if !tags().any(|t| t != 0) || !tags().any(|t| t == 0) {
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
        (standing[0], standing[1])
    } else {
        (standing[1], standing[0])
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
