//! A surface walk that lands ON a mesh vertex turning through that vertex's fan to the face that
//! carries it on, as free functions. The face readings it turns with are
//! [`super::section_surface_walk_faces`]'s.

use solidmesh_math::Vector3;
use solidmesh_mesh::{Mesh, MeshEdgeGraph};

use super::section_fan_walk::EdgeAtPair;
use super::section_surface_walk::pair_key;
use super::section_surface_walk_faces::{try_face_holding, unfold};

/// The corner of a face a point stands on, within `lattice` - the nearest
/// such, the later on a tie - or `None`.
pub fn try_the_corner_under(mesh: &Mesh, face: i32, at: Vector3, lattice: f64) -> Option<i32> {
    let drawn = &mesh.faces[face as usize];
    let mut nearest = lattice;
    let mut corner = -1;

    for candidate in [drawn.v0, drawn.v1, drawn.v2] {
        let off = (Vector3::from_vector3_float(mesh.vertices[candidate as usize]) - at).length();

        if off <= nearest {
            nearest = off;
            corner = candidate;
        }
    }

    (corner >= 0).then_some(corner)
}

/// A walk standing on `corner` of `face` turned through the corner's fan,
/// starting across each of the face's two arms at the corner in turn, to the first face that holds
/// the direction. `face` and `direction` are changed only where it succeeds.
#[allow(clippy::too_many_arguments)]
pub fn try_turn_out_of_the_corner(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    corner: i32,
    stop_at_crease: Option<&dyn Fn(i32) -> bool>,
    face: &mut i32,
    direction: &mut Vector3,
) -> bool {
    let drawn = &mesh.faces[*face as usize];
    let arms: Vec<i32> = [drawn.v0, drawn.v1, drawn.v2]
        .into_iter()
        .filter(|&other| other != corner)
        .filter_map(|other| edge_at_pair.get(&pair_key(corner, other)).copied())
        .collect();

    for arm in arms {
        let mut turned_face = *face;
        let mut turned_direction = *direction;

        if try_turn_through_the_vertex(
            mesh,
            graph,
            edge_at_pair,
            corner,
            arm,
            stop_at_crease,
            &mut turned_face,
            &mut turned_direction,
        )
        .is_ok()
        {
            *face = turned_face;
            *direction = turned_direction;

            return true;
        }
    }

    false
}

/// Turns face by face round `vertex`, starting past the edge the walk
/// `arrived_across`, carrying the direction across each edge, until a face's wedge holds it. `Err`
/// carries the crease or open edge the turn stopped at, or -1 where it came all the
/// way round or ran out of edges. `face` and `direction` are left where the turn got
/// to, so the caller can read how far it came.
#[allow(clippy::too_many_arguments)]
pub fn try_turn_through_the_vertex(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    vertex: i32,
    arrived_across: i32,
    stop_at_crease: Option<&dyn Fn(i32) -> bool>,
    face: &mut i32,
    direction: &mut Vector3,
) -> Result<(), i32> {
    let mut walked = arrived_across;

    for _ in 0..=graph.edge_count() {
        if let Ok((_, in_face)) = try_face_holding(mesh, vertex, *direction, &[*face]) {
            *direction = in_face;

            return Ok(());
        }

        let Some(next) = try_the_other_edge_at(mesh, *face, vertex, walked, edge_at_pair) else {
            return Err(-1);
        };

        if stop_at_crease.is_some_and(|stop| stop(next)) || graph.get_face_count(next) != 2 {
            return Err(next);
        }

        *direction = unfold(mesh, *direction, next, graph, *face);
        *face = if graph.get_face(next, 0) == *face {
            graph.get_face(next, 1)
        } else {
            graph.get_face(next, 0)
        };
        walked = next;

        if walked == arrived_across {
            return Err(-1);
        }
    }

    Err(-1)
}

/// The edge of `face` meeting at `vertex` that is not the one `walked` - the
/// later corner's where both qualify.
pub fn try_the_other_edge_at(
    mesh: &Mesh,
    face: i32,
    vertex: i32,
    walked: i32,
    edge_at_pair: &EdgeAtPair,
) -> Option<i32> {
    let drawn = &mesh.faces[face as usize];
    let mut other = -1;

    for corner in [drawn.v0, drawn.v1, drawn.v2] {
        if corner != vertex {
            if let Some(&edge) = edge_at_pair.get(&pair_key(vertex, corner)) {
                if edge != walked {
                    other = edge;
                }
            }
        }
    }

    (other >= 0).then_some(other)
}
