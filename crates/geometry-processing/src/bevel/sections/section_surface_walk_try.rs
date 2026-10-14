//! The surface walk itself: a straight reach laid ALONG the surface from a point of one face,
//! unfolding across every edge it crosses, until the reach runs out, a crease or boundary stops it,
//! or no face holds the direction. In its own file because it is the walk's one long function;
//! the walk's types and small readings are in [`super::section_surface_walk`], its face readings in
//! [`super::section_surface_walk_faces`], and the turn through a vertex in
//! [`super::section_walk_vertex_turn`].

use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::{cs_math, Vector3};

use super::section_fan_walk::EdgeAtPair;
use super::section_surface_walk::{
    along_from, foot_on, grid_of, is_a_fold, parameter_on, refused, LATTICE_FRACTION,
};
use super::section_surface_walk_faces::{
    try_exit, try_face_holding, try_face_holding_at_edge, try_the_edge_under, unfold,
};
use super::section_walk_vertex_turn::{
    try_the_corner_under, try_turn_out_of_the_corner, try_turn_through_the_vertex,
};
use super::{SurfaceWalkCrossing, SurfaceWalkRefusal, SurfaceWalkResult};

/// The reach of `distance` from `start` in `start_face` along `direction_in_face`, laid
/// along the surface. `Ok` is a walk that ran its whole reach (its end classified onto a fold or a
/// corner it lands within the mesh's lattice of); `Err` is the walk that stopped short, saying where
/// and why, with the walk as far as it got.
///
/// **Setting out.** A start on an edge lies in both its faces, so where the named face has no exit
/// ahead of the line the DIRECTION picks its face (by the fold's own rotation), and a start on a mesh
/// vertex turns the vertex's fan for the face that holds it. A start a lattice OUTSIDE its face's
/// crease is moved onto its foot on that crease rather than refused for crossing it at distance zero.
///
/// **Landing.** A reach running out within the mesh's lattice of a fold - short of it, past it, or
/// alongside it - is classified onto that fold and the end moved onto it; one on a corner is put on
/// the corner. Which side of a fold float noise puts an end on must not decide anything downstream.
#[allow(clippy::too_many_arguments)]
pub fn try_walk(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    start: Vector3,
    start_face: i32,
    direction_in_face: Vector3,
    distance: f64,
    stop_at_crease: Option<&dyn Fn(i32) -> bool>,
) -> Result<SurfaceWalkResult, SurfaceWalkResult> {
    let mut crossings: Vec<SurfaceWalkCrossing> = Vec::new();
    let mut face = start_face;
    let mut at = start;
    let mut direction = direction_in_face.get_normal();
    let mut remaining = distance;
    let stops = |edge: i32| stop_at_crease.is_some_and(|stop| stop(edge));

    // The MESH's lattice, not the walk's ruler: a start this near a fold or a corner is STANDING on it.
    let lattice_of_the_mesh = grid_of(mesh);

    if direction.length_squared() < 0.5 || distance < 0.0 {
        return Err(refused(
            at,
            face,
            direction,
            crossings,
            0.0,
            SurfaceWalkRefusal::NothingToWalk,
            -1,
            0.0,
        ));
    }

    // SETTING OUT FROM A POINT ON AN EDGE: only when the face the caller named has no exit ahead of
    // the line at all.
    let first_exit = try_exit(mesh, edge_at_pair, face, at, direction);
    let nothing_ahead_in_the_named_face = first_exit.is_none();

    // SETTING OUT FROM JUST OUTSIDE THE NAMED FACE: a start within the mesh's lattice of a CREASE it
    // would cross first is ON that crease, and is set out from its foot there.
    if let Some(first_exit) = first_exit {
        if first_exit.distance <= lattice_of_the_mesh
            && try_the_edge_under(
                mesh,
                graph,
                edge_at_pair,
                face,
                at,
                lattice_of_the_mesh,
                false,
            ) == Some(first_exit.edge)
            && stops(first_exit.edge)
        {
            at = first_exit.at;
        }
    }

    let set_out_from = if nothing_ahead_in_the_named_face {
        try_the_corner_under(mesh, face, at, lattice_of_the_mesh)
    } else {
        None
    };

    if let Some(set_out_from) = set_out_from {
        // A point ON a mesh vertex is the vertex's own question: the face that holds the direction may
        // be any face of the fan. The direction it hands back is used whether or not the face held
        // it, so a walk always sets out along the best reading there is.
        let holding = try_face_holding(mesh, set_out_from, direction, &[face]);
        let held_at_the_corner = match holding {
            Ok((_, held)) | Err(Some((_, held))) => held,
            Err(None) => Vector3::ZERO,
        };

        if holding.is_err()
            && !try_turn_out_of_the_corner(
                mesh,
                graph,
                edge_at_pair,
                set_out_from,
                stop_at_crease,
                &mut face,
                &mut direction,
            )
        {
            return Err(refused(
                at,
                face,
                direction,
                crossings,
                0.0,
                SurfaceWalkRefusal::NoFaceHoldsTheDirection,
                -1,
                0.0,
            ));
        } else if held_at_the_corner.length_squared() >= 0.5 {
            direction = held_at_the_corner;
        }
    } else if nothing_ahead_in_the_named_face {
        if let Some(set_out_on) = try_the_edge_under(
            mesh,
            graph,
            edge_at_pair,
            face,
            at,
            lattice_of_the_mesh,
            false,
        ) {
            if let Ok((holder, held_direction)) =
                try_face_holding_at_edge(mesh, graph, set_out_on, face, direction)
            {
                if holder != face {
                    if stops(set_out_on) {
                        return Err(refused(
                            at,
                            face,
                            direction,
                            crossings,
                            0.0,
                            SurfaceWalkRefusal::CreaseReached,
                            set_out_on,
                            along_from(mesh, graph, set_out_on, at),
                        ));
                    }

                    face = holder;
                    direction = held_direction;
                }
            }
        }
    }

    // The walk can cross no more edges than the mesh has; the cap only catches a mesh whose facets
    // fold back on themselves, where a straight line could otherwise circle for ever.
    for _ in 0..=graph.edge_count() {
        let Some(exit) = try_exit(mesh, edge_at_pair, face, at, direction) else {
            return Err(refused(
                at,
                face,
                direction,
                crossings,
                distance - remaining,
                SurfaceWalkRefusal::NothingToWalk,
                -1,
                0.0,
            ));
        };

        if exit.distance >= remaining {
            // The reach ran out exactly where the face ends: the end is ON that edge.
            let landed_on_the_exit =
                exit.distance - remaining <= LATTICE_FRACTION * cs_math::max_f64(1.0, distance);

            // LANDING ON A FOLD: a reach running out within the mesh's lattice SHORT of a fold is
            // classified onto it. Only folds - a flat edge's two faces are one surface.
            let landed_on_the_fold = !landed_on_the_exit
                && exit.distance - remaining <= lattice_of_the_mesh
                && is_a_fold(mesh, graph, exit.edge);

            let landed = landed_on_the_exit || landed_on_the_fold;

            // EITHER landing moves its end onto the crossing.
            let mut ends_at = if landed {
                exit.at
            } else {
                at + (direction * remaining)
            };
            let mut end_edge = if landed { exit.edge } else { -1 };
            let mut end_parameter = if landed {
                parameter_on(graph, exit.edge, exit.from, exit.along)
            } else {
                0.0
            };

            // LANDING ON A CORNER: the exit knows when the line left through a mesh vertex.
            let mut end_vertex = if landed { exit.vertex } else { -1 };

            // RUNNING OUT ALONGSIDE A FOLD: the end's PERPENDICULAR to every fold bounding its face is
            // read, and the nearest fold within the lattice claimed, the end moved onto its foot.
            if !landed {
                if let Some(alongside) = try_the_edge_under(
                    mesh,
                    graph,
                    edge_at_pair,
                    face,
                    ends_at,
                    lattice_of_the_mesh,
                    true,
                ) {
                    end_edge = alongside;
                    end_parameter = along_from(mesh, graph, alongside, ends_at);
                    ends_at = foot_on(mesh, graph, alongside, end_parameter);
                }

                // RUNNING OUT AT A CORNER WITHOUT REACHING AN EXIT: the CORNER is what it stands on.
                if let Some(corner_at_the_end) =
                    try_the_corner_under(mesh, face, ends_at, lattice_of_the_mesh)
                {
                    end_vertex = corner_at_the_end;
                }
            }

            // EITHER way of ending on a corner puts the end exactly ON it.
            if end_vertex >= 0 {
                ends_at = position(mesh, end_vertex);
                if end_edge >= 0 {
                    end_parameter = along_from(mesh, graph, end_edge, ends_at);
                }
            }

            return Ok(SurfaceWalkResult {
                // The shortfall - at most one lattice - is this classification's own rounding.
                travelled: if landed_on_the_fold {
                    distance - remaining + exit.distance
                } else {
                    distance
                },
                refusal: SurfaceWalkRefusal::None,
                end_edge,
                end_parameter,
                end_vertex,
                ..SurfaceWalkResult::new(ends_at, face, direction, crossings)
            });
        }

        let crossed_parameter = parameter_on(graph, exit.edge, exit.from, exit.along);

        if stops(exit.edge) {
            return Err(refused(
                exit.at,
                face,
                direction,
                crossings,
                distance - remaining + exit.distance,
                SurfaceWalkRefusal::CreaseReached,
                exit.edge,
                crossed_parameter,
            ));
        }

        if graph.get_face_count(exit.edge) != 2 {
            return Err(refused(
                exit.at,
                face,
                direction,
                crossings,
                distance - remaining + exit.distance,
                SurfaceWalkRefusal::BoundaryReached,
                exit.edge,
                crossed_parameter,
            ));
        }

        // LANDING ON A FOLD FROM THE FAR SIDE: an end within the lattice beyond this crossing is
        // classified onto it, the crossing still recorded so a join laid through it turns on it.
        if remaining - exit.distance <= lattice_of_the_mesh && is_a_fold(mesh, graph, exit.edge) {
            crossings.push(SurfaceWalkCrossing::new(
                exit.edge,
                crossed_parameter,
                exit.at,
            ));

            let crossed_at = if exit.vertex >= 0 {
                position(mesh, exit.vertex)
            } else {
                exit.at
            };

            return Ok(SurfaceWalkResult {
                // The overshoot - at most one lattice - is this classification's own rounding.
                travelled: distance - remaining + exit.distance,
                refusal: SurfaceWalkRefusal::None,
                end_edge: exit.edge,
                end_parameter: if exit.vertex >= 0 {
                    along_from(mesh, graph, exit.edge, crossed_at)
                } else {
                    crossed_parameter
                },
                end_vertex: exit.vertex,
                ..SurfaceWalkResult::new(crossed_at, face, direction, crossings)
            });
        }

        crossings.push(SurfaceWalkCrossing::new(
            exit.edge,
            crossed_parameter,
            exit.at,
        ));

        remaining -= exit.distance;
        at = exit.at;
        direction = unfold(mesh, direction, exit.edge, graph, face);
        face = if graph.get_face(exit.edge, 0) == face {
            graph.get_face(exit.edge, 1)
        } else {
            graph.get_face(exit.edge, 0)
        };

        if exit.vertex >= 0 {
            if let Err(stopped_at) = try_turn_through_the_vertex(
                mesh,
                graph,
                edge_at_pair,
                exit.vertex,
                exit.edge,
                stop_at_crease,
                &mut face,
                &mut direction,
            ) {
                // STOPPED BY A CREASE AT THE VERTEX: the line has reached the crease, at the vertex,
                // exactly as a line crossing the crease between its ends has, and says so.
                if stopped_at >= 0 {
                    let on_the_vertex = position(mesh, exit.vertex);

                    return Err(SurfaceWalkResult {
                        travelled: distance - remaining,
                        refusal: if graph.get_face_count(stopped_at) != 2 {
                            SurfaceWalkRefusal::BoundaryReached
                        } else {
                            SurfaceWalkRefusal::CreaseReached
                        },
                        end_edge: stopped_at,
                        end_parameter: along_from(mesh, graph, stopped_at, on_the_vertex),
                        end_vertex: exit.vertex,
                        ..SurfaceWalkResult::new(on_the_vertex, face, direction, crossings)
                    });
                }

                return Err(refused(
                    at,
                    face,
                    direction,
                    crossings,
                    distance - remaining,
                    SurfaceWalkRefusal::NoFaceHoldsTheDirection,
                    -1,
                    0.0,
                ));
            }
        }
    }

    Err(refused(
        at,
        face,
        direction,
        crossings,
        distance - remaining,
        SurfaceWalkRefusal::NothingToWalk,
        -1,
        0.0,
    ))
}

/// A vertex's position, read as doubles.
fn position(mesh: &Mesh, vertex: i32) -> Vector3 {
    Vector3::from_vector3_float(mesh.vertices[vertex as usize])
}
