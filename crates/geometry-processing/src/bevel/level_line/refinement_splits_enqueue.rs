//! The refinement's splits that put their children back on the worklist, and the sliver guard's
//! longest-edge bisection (Rivara). The read-only geometry they ask (longest edges, terminal
//! edges, the thinning tests) is in `refinement_splits`.

use std::collections::{HashMap, VecDeque};

use solidmesh_math::Vector3;

use super::refinement_splits::{
    enqueue, longest_edge, longest_edge_length, terminal_edge, triangle_thinned_by,
    triangle_thinned_by_on_edge, where_the_point_stands, PointPlace, LONGEST_EDGE_PATH_CAP,
    NOTHING_TO_INSERT,
};
use super::{FieldCounters, LevelLineMesh, LevelLineMeshError, LevelLineRule, WorkError};

/// The aspect past which a split's child is a sliver the
/// guard bisects instead of making.
pub const SLIVER_ASPECT_LIMIT: f64 = 64.0;

/// Splits a mesh edge at a point and puts every child
/// back on the worklist at ITS OWN parent's depth plus one, then whatever repairing a needle among
/// them made in their place. Answers the vertex now at the point; one under the vertex count from
/// before is a vertex the mesh already carried, and nothing was divided.
pub fn split_edge_and_enqueue(
    mesh: &mut LevelLineMesh,
    v: i32,
    w: i32,
    point: Vector3,
    pending: &mut VecDeque<i32>,
    depth: &mut HashMap<i32, i32>,
) -> Result<i32, LevelLineMeshError> {
    // Each parent is named by the corner it does NOT share with the edge, which both of its
    // children keep - so the children can be found again after the split.
    let mut opposites = Vec::new();
    for &parent in mesh.triangles_on(v, w) {
        let face = mesh.triangles()[parent as usize];
        let other = if face.a != v && face.a != w {
            face.a
        } else if face.b != v && face.b != w {
            face.b
        } else {
            face.c
        };
        opposites.push((other, depth.get(&parent).copied().unwrap_or(0)));
    }

    // A new vertex is always the next id, so anything under this count is one the mesh already
    // carried: nothing was divided, and putting the old triangles back would ask the same rule the
    // same question for ever.
    let before = mesh.vertex_count();
    let mid = mesh.split_edge(v, w, point, true)?;
    if mid < before {
        return Ok(mid);
    }

    for (corner, parent_depth) in opposites {
        let children = mesh.triangles_on(mid, corner).to_vec();
        enqueue(children.iter().copied(), pending, depth, parent_depth + 1);
        let repaired = mesh.repair_needles_around(children)?;
        enqueue(repaired, pending, depth, parent_depth + 1);
    }

    Ok(mid)
}

/// Divides a triangle about a point and puts the
/// three children back on the worklist one deeper than the parent - or, where the mesh refused to
/// mint on top of a corner it already carries, answers with that corner and enqueues nothing.
pub fn split_triangle_and_enqueue(
    mesh: &mut LevelLineMesh,
    triangle: i32,
    point: Vector3,
    pending: &mut VecDeque<i32>,
    depth: &mut HashMap<i32, i32>,
) -> Result<i32, WorkError> {
    let t = mesh.triangles()[triangle as usize];
    let here = depth.get(&triangle).copied().unwrap_or(0);
    let inserted = mesh.split_triangle(triangle, point)?;
    if inserted == t.a || inserted == t.b || inserted == t.c {
        return Ok(inserted);
    }

    // The triangles round the inserted vertex - its three children.
    let mut children: Vec<i32> = Vec::new();
    for corner in [t.a, t.b, t.c] {
        children.extend_from_slice(mesh.triangles_on(corner, inserted));
    }
    enqueue(children.iter().copied(), pending, depth, here + 1);
    let repaired = mesh.repair_needles_around(children)?;
    enqueue(repaired, pending, depth, here + 1);
    Ok(inserted)
}

/// Where a rule's split of
/// edge `v`-`w` at `point` would thin a child of the asking triangle's tag, bisects that child's
/// longest-edge path instead; true when it did.
#[allow(clippy::too_many_arguments)]
pub fn bisect_instead_of_thinning_on_edge(
    mesh: &mut LevelLineMesh,
    asking: i32,
    v: i32,
    w: i32,
    point: Vector3,
    chord_tolerance: f64,
    pending: &mut VecDeque<i32>,
    depth: &mut HashMap<i32, i32>,
) -> Result<bool, LevelLineMeshError> {
    let tag = mesh.triangles()[asking as usize].tag;
    let thinned = triangle_thinned_by_on_edge(
        mesh,
        v,
        w,
        point,
        SLIVER_ASPECT_LIMIT,
        2.0 * chord_tolerance,
        Some(tag),
    );
    if thinned < 0 {
        return Ok(false);
    }

    let asking_on_the_edge = mesh.triangles_on(v, w).to_vec();
    bisect(mesh, thinned, &asking_on_the_edge, pending, depth)
}

/// The same guard for a
/// point a rule hands to `InsertWhereItStands`; `triangle` is the triangle the rule fired on.
pub fn bisect_instead_of_thinning(
    mesh: &mut LevelLineMesh,
    triangle: i32,
    point: Vector3,
    chord_tolerance: f64,
    pending: &mut VecDeque<i32>,
    depth: &mut HashMap<i32, i32>,
) -> Result<bool, LevelLineMeshError> {
    let thinned = triangle_thinned_by(
        mesh,
        triangle,
        point,
        SLIVER_ASPECT_LIMIT,
        2.0 * chord_tolerance,
        true,
    );
    if thinned < 0 {
        return Ok(false);
    }

    bisect(mesh, thinned, &[triangle], pending, depth)
}

/// Bisects the thinned triangle along its longest-edge path, and
/// puts the asking triangles still standing exactly as they were back on the worklist: the
/// crossing the rule found has to be asked again of whatever now carries it.
fn bisect(
    mesh: &mut LevelLineMesh,
    thinned: i32,
    asking: &[i32],
    pending: &mut VecDeque<i32>,
    depth: &mut HashMap<i32, i32>,
) -> Result<bool, LevelLineMeshError> {
    // A split rewrites a divided triangle IN PLACE as one of its children, which the split has
    // queued already, so only the ones left exactly as they were go back.
    let was: Vec<_> = asking
        .iter()
        .map(|&t| mesh.triangles()[t as usize])
        .collect();
    let before = mesh.vertex_count();
    bisect_along_longest_edge_path(mesh, thinned, pending, depth)?;
    if mesh.vertex_count() == before {
        // Stopped before dividing anything, so the rule's own split is still the right one.
        return Ok(false);
    }

    FieldCounters::fired(LevelLineRule::SliverGuard);
    for (i, &t) in asking.iter().enumerate() {
        let now = mesh.triangles()[t as usize];
        if !mesh.is_removed(t) && now.a == was[i].a && now.b == was[i].b && now.c == was[i].c {
            pending.push_back(t);
        }
    }

    Ok(true)
}

/// Rivara's longest-edge-propagation-path
/// bisection - removes a triangle's longest edge by bisecting, at their midpoints, the terminal
/// edges its path leads to, until the triangle's own longest edge is split. True once that edge is
/// gone; false where it had to stop (a chain or non-manifold terminal edge, the path cap, or a
/// midpoint the mesh answers with a vertex it already carries).
pub fn bisect_along_longest_edge_path(
    mesh: &mut LevelLineMesh,
    triangle: i32,
    pending: &mut VecDeque<i32>,
    depth: &mut HashMap<i32, i32>,
) -> Result<bool, LevelLineMeshError> {
    let (a, b) = longest_edge(mesh, triangle);
    for _ in 0..LONGEST_EDGE_PATH_CAP {
        let standing = mesh.triangles_on(a, b);
        if standing.is_empty() {
            return Ok(true);
        }

        // The path is walked from the triangle the edge is LONGEST in; none such is a mesh repair
        // re-tiled round, and nothing to walk.
        let Some(from) = standing
            .iter()
            .copied()
            .find(|&t| longest_edge(mesh, t) == (a, b))
        else {
            return Ok(false);
        };
        let Some(terminal) = terminal_edge(mesh, from) else {
            return Ok(false);
        };

        // Not charged here: the edge split charges its own unit, so each bisection costs one.
        let before = mesh.vertex_count();
        let middle = 0.5 * (mesh.position(terminal.0) + mesh.position(terminal.1));
        let mid = split_edge_and_enqueue(mesh, terminal.0, terminal.1, middle, pending, depth)?;
        if mid < before {
            return Ok(false);
        }
    }

    Ok(mesh.triangles_on(a, b).is_empty())
}

/// Puts a point of the line into the mesh - at the corner
/// it already stands on (asked at the SNAP, since the caller's `put_on_the_line` on the answer
/// moves a side, decided at the snap everywhere), on an edge when it stands within the snap of
/// one, otherwise inside the triangle. [`NOTHING_TO_INSERT`] where the point is off every corner
/// by more than the snap but within the triangle's millionth of one: neither minting nor moving
/// a side is right there.
pub fn insert_on_the_line(
    mesh: &mut LevelLineMesh,
    triangle: i32,
    point: Vector3,
    snap: f64,
    pending: &mut VecDeque<i32>,
    depth: &mut HashMap<i32, i32>,
) -> Result<i32, LevelLineMeshError> {
    let t = mesh.triangles()[triangle as usize];
    let (place, at_corner, _) = where_the_point_stands(mesh, t, point, Some(snap));
    if place == PointPlace::AtACorner {
        return Ok(at_corner);
    }

    let size = longest_edge_length(mesh, t);
    for corner in 0..3 {
        if (point - mesh.position(t.corner(corner))).length() <= 1e-6 * size {
            return Ok(NOTHING_TO_INSERT);
        }
    }

    for corner in 0..3 {
        let v = t.corner(corner);
        let w = t.corner((corner + 1) % 3);
        let along = mesh.position(w) - mesh.position(v);
        let length_squared = along.length_squared();
        let s = if length_squared > 0.0 {
            (point - mesh.position(v)).dot(along) / length_squared
        } else {
            0.0
        };
        // `clamp` lets NaN through, so a broken reading stays visible.
        let s = s.clamp(0.0, 1.0);
        let on_edge = mesh.position(v) + (along * s);
        if (on_edge - point).length() <= snap {
            return split_edge_and_enqueue(mesh, v, w, point, pending, depth);
        }
    }

    Ok(split_triangle_and_enqueue(
        mesh, triangle, point, pending, depth,
    )?)
}

/// Puts the point a RULE asked for into the mesh,
/// wherever it stands - at an existing corner nothing is minted and the corner is answered with,
/// on an edge both triangles there are split together, otherwise the triangle is divided about
/// it. `same_place` `None` is the triangle's own millionth, what the field rules ask by.
pub fn insert_where_it_stands(
    mesh: &mut LevelLineMesh,
    triangle: i32,
    point: Vector3,
    pending: &mut VecDeque<i32>,
    depth: &mut HashMap<i32, i32>,
    same_place: Option<f64>,
) -> Result<i32, LevelLineMeshError> {
    let t = mesh.triangles()[triangle as usize];
    match where_the_point_stands(mesh, t, point, same_place) {
        (PointPlace::AtACorner, from, _) => Ok(from),
        (PointPlace::OnAnEdge, from, to) => {
            split_edge_and_enqueue(mesh, from, to, point, pending, depth)
        }
        _ => Ok(split_triangle_and_enqueue(
            mesh, triangle, point, pending, depth,
        )?),
    }
}
