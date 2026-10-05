//! Joining two points of the surface by the line along it (`Bevel/Sections/SectionSurfaceWalk.Join.cs`,
//! part of the C# `internal static partial class SectionSurfaceWalk`; free functions here). What is
//! here are the join's own readings: the strip of faces between two ends ([`try_strip`],
//! [`try_the_fold_between`], [`try_the_faces_walked`]), that strip unfolded flat ([`flatten`],
//! [`place`], [`in_flat`]), a strip's key ([`key_of`]) and whether a walk arrived ([`arrived_at`]).
//! The join itself (`TryJoin`, `TryLayTheLine`) lands with the next run of this package, after the
//! walk (`TryWalk`).

use std::collections::VecDeque;

use indexmap::IndexMap;
use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::{cs_math, Vector2, Vector3};

use super::section_fan_walk::EdgeAtPair;
use super::section_surface_walk::{is_the_same_point, normal_of, pair_key};
use super::SurfaceWalkResult;
use crate::bevel::surface::SurfaceVertexPool;

/// C# `JoinStripReach`: how many faces deep the strip search looks before it gives up.
const JOIN_STRIP_REACH: i32 = 32;

/// A strip of faces flattened into the plane of its first face, by mesh vertex.
pub type FlatStrip = IndexMap<i32, Vector2>;

/// C# `TryTheFacesWalked`: the faces a walk ran through, from `end_a_face` to `end_b_face`, and the
/// folds between neighbours - consecutive faces that do not share an edge bridged by the shortest
/// strip between them ([`try_strip`]). `None` where a gap cannot be bridged, or the run is one face.
#[allow(clippy::too_many_arguments)]
pub fn try_the_faces_walked(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    end_a_face: i32,
    end_b_face: i32,
    walk: &SurfaceWalkResult,
    stop_at_crease: Option<&dyn Fn(i32) -> bool>,
) -> Option<(Vec<i32>, Vec<i32>)> {
    let mut run = vec![end_a_face];
    let mut face = end_a_face;

    for crossing in &walk.crossings {
        if graph.get_face_count(crossing.edge) != 2 {
            break;
        }

        let one = graph.get_face(crossing.edge, 0);
        face = if one == face {
            graph.get_face(crossing.edge, 1)
        } else {
            one
        };
        run.push(face);
    }

    run.push(end_b_face);

    let mut faces: Vec<i32> = Vec::new();
    let mut folds: Vec<i32> = Vec::new();

    for wanted in run {
        let Some(&last) = faces.last() else {
            faces.push(wanted);
            continue;
        };

        if last == wanted {
            continue;
        }

        if let Some(fold) =
            try_the_fold_between(mesh, graph, edge_at_pair, last, wanted, stop_at_crease)
        {
            faces.push(wanted);
            folds.push(fold);
            continue;
        }

        let (bridge, bridge_folds) =
            try_strip(mesh, graph, edge_at_pair, last, wanted, stop_at_crease)?;

        for i in 1..bridge.len() {
            faces.push(bridge[i]);
            folds.push(bridge_folds[i - 1]);
        }
    }

    (faces.len() > 1).then_some((faces, folds))
}

/// C# `TryTheFoldBetween`: the edge `face` shares with `other`, where it is a manifold edge the walk
/// may cross (not a crease where `stop_at_crease` says so).
pub fn try_the_fold_between(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    face: i32,
    other: i32,
    stop_at_crease: Option<&dyn Fn(i32) -> bool>,
) -> Option<i32> {
    let drawn = &mesh.faces[face as usize];
    let corners = [drawn.v0, drawn.v1, drawn.v2];

    for i in 0..3 {
        let Some(&edge) = edge_at_pair.get(&pair_key(corners[i], corners[(i + 1) % 3])) else {
            continue;
        };

        if stop_at_crease.is_some_and(|stop| stop(edge)) || graph.get_face_count(edge) != 2 {
            continue;
        }

        if graph.get_face(edge, 0) == other || graph.get_face(edge, 1) == other {
            return Some(edge);
        }
    }

    None
}

/// C# `KeyOf`: a strip's faces as one key, so the join can tell a strip it has already unfolded.
pub fn key_of(faces: &[i32]) -> String {
    faces
        .iter()
        .map(|face| face.to_string())
        .collect::<Vec<_>>()
        .join(",")
}

/// C# `ArrivedAt`: whether a walk came to `end_b` - one point to the walk's own lattice over a reach
/// of `over`, or, where the walk stopped on an edge or the join has `settled`, within the mesh's own
/// weld distance.
pub fn arrived_at(
    mesh: &Mesh,
    walk: &SurfaceWalkResult,
    end_b: Vector3,
    over: f64,
    settled: bool,
) -> bool {
    if is_the_same_point(walk.end, end_b, over) {
        return true;
    }

    (walk.end_edge >= 0 || settled)
        && (walk.end - end_b).length() <= SurfaceVertexPool::weld_distance_for(&mesh.vertices)
}

/// C# `TryStrip`: the shortest strip of faces from `from` to `to`, breadth first across manifold
/// edges the walk may cross, no deeper than [`JOIN_STRIP_REACH`], as `(faces, folds)`.
pub fn try_strip(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    from: i32,
    to: i32,
    stop_at_crease: Option<&dyn Fn(i32) -> bool>,
) -> Option<(Vec<i32>, Vec<i32>)> {
    // Only looked up by face; the strip is read back by following it, so no map order reaches it.
    let mut came_from: IndexMap<i32, (i32, i32)> = IndexMap::new();
    let mut depth: IndexMap<i32, i32> = IndexMap::new();
    came_from.insert(from, (-1, -1));
    depth.insert(from, 0);
    let mut queue = VecDeque::from([from]);

    while let Some(face) = queue.pop_front() {
        if face == to {
            let mut faces = Vec::new();
            let mut folds = Vec::new();
            let mut walk = to;

            while walk >= 0 {
                faces.insert(0, walk);

                let (before, edge) = came_from[&walk];

                if edge >= 0 {
                    folds.insert(0, edge);
                }

                walk = before;
            }

            return Some((faces, folds));
        }

        if depth[&face] >= JOIN_STRIP_REACH {
            continue;
        }

        let drawn = &mesh.faces[face as usize];
        let corners = [drawn.v0, drawn.v1, drawn.v2];

        for i in 0..3 {
            let Some(&edge) = edge_at_pair.get(&pair_key(corners[i], corners[(i + 1) % 3])) else {
                continue;
            };

            if stop_at_crease.is_some_and(|stop| stop(edge)) || graph.get_face_count(edge) != 2 {
                continue;
            }

            let other = if graph.get_face(edge, 0) == face {
                graph.get_face(edge, 1)
            } else {
                graph.get_face(edge, 0)
            };

            if came_from.contains_key(&other) {
                continue;
            }

            came_from.insert(other, (face, edge));
            let next_depth = depth[&face] + 1;
            depth.insert(other, next_depth);
            queue.push_back(other);
        }
    }

    None
}

/// C# `Flatten`: a strip of faces unfolded into the plane of its first face, each later face's
/// loose corner placed from the two it shares with what is already flat, on the far side of the
/// shared edge from the face before it. C#'s unused `folds` parameter is dropped. `None` where a
/// face's far corner was never placed (where C# would throw on the lookup).
pub fn flatten(mesh: &Mesh, faces: &[i32]) -> Option<FlatStrip> {
    let mut flat = FlatStrip::new();
    let first = &mesh.faces[faces[0] as usize];
    let a = position(mesh, first.v0);
    let b = position(mesh, first.v1);
    let c = position(mesh, first.v2);
    let x = (b - a).get_normal();
    let y = normal_of(mesh, faces[0]).cross(x);

    flat.insert(first.v0, Vector2::ZERO);
    flat.insert(first.v1, Vector2::new((b - a).dot(x), (b - a).dot(y)));
    flat.insert(first.v2, Vector2::new((c - a).dot(x), (c - a).dot(y)));

    for i in 1..faces.len() {
        let drawn = &mesh.faces[faces[i] as usize];
        let mut placed = Vec::new();
        let mut loose = -1;

        for corner in [drawn.v0, drawn.v1, drawn.v2] {
            if flat.contains_key(&corner) {
                placed.push(corner);
            } else {
                loose = corner;
            }
        }

        if loose < 0 || placed.len() < 2 {
            continue;
        }

        let before = &mesh.faces[faces[i - 1] as usize];
        let mut away = before.v0;

        for corner in [before.v0, before.v1, before.v2] {
            if corner != placed[0] && corner != placed[1] {
                away = corner;
            }
        }

        let away_at = *flat.get(&away)?;

        let at = place(
            flat[&placed[0]],
            flat[&placed[1]],
            (position(mesh, loose) - position(mesh, placed[0])).length(),
            (position(mesh, loose) - position(mesh, placed[1])).length(),
            away_at,
        );
        flat.insert(loose, at);
    }

    Some(flat)
}

/// C# `Place`: the point `from_one` from `one` and `from_other` from `other`, on the far side of
/// the line through them from `away`.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn place(
    one: Vector2,
    other: Vector2,
    from_one: f64,
    from_other: f64,
    away: Vector2,
) -> Vector2 {
    let along = other - one;
    let length = along.length();

    if length < 1e-300 {
        return one;
    }

    let unit = along / length;
    let square = Vector2::new(-unit.y, unit.x);
    let x =
        ((from_one * from_one) - (from_other * from_other) + (length * length)) / (2.0 * length);
    let y = cs_math::max_f64(0.0, (from_one * from_one) - (x * x)).sqrt();
    let side = (away - one).dot(square);
    // C# `Math.Sign(..) >= 0`: zero counts with the positive side.
    let side_of_away_is_not_negative = !(side < 0.0);

    one + (unit * x) + (square * if side_of_away_is_not_negative { -y } else { y })
}

/// C# `InFlat`: a point of `face` read into the flattened strip, by its barycentric weights in the
/// face. `None` where the strip does not hold the face's corners (where C# would throw).
pub fn in_flat(mesh: &Mesh, flat: &FlatStrip, face: i32, at: Vector3) -> Option<Vector2> {
    let drawn = &mesh.faces[face as usize];
    let a = position(mesh, drawn.v0);
    let b = position(mesh, drawn.v1);
    let c = position(mesh, drawn.v2);
    let normal = normal_of(mesh, face);
    let whole = (b - a).cross(c - a).dot(normal);
    let flat_a = *flat.get(&drawn.v0)?;

    if whole.abs() < 1e-300 {
        return Some(flat_a);
    }

    let beta = (at - a).cross(c - a).dot(normal) / whole;
    let gamma = (b - a).cross(at - a).dot(normal) / whole;

    Some(
        flat_a
            + ((*flat.get(&drawn.v1)? - flat_a) * beta)
            + ((*flat.get(&drawn.v2)? - flat_a) * gamma),
    )
}

/// A vertex's position, read as doubles.
fn position(mesh: &Mesh, vertex: i32) -> Vector3 {
    Vector3::from_vector3_float(mesh.vertices[vertex as usize])
}
