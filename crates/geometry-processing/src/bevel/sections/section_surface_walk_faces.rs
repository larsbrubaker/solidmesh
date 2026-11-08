//! The surface walk's face readings: which face holds a direction at a vertex or across an edge,
//! where a line leaves a face, and a direction carried across a fold, as free functions. Split from
//! [`super::section_surface_walk`], which keeps
//! the walk's types and small readings, by responsibility: these are the questions the walk asks of
//! ONE face (or the faces round one vertex or edge) at each step.

use solidmesh_math::{ieee_math, Vector3};
use solidmesh_mesh::{Mesh, MeshEdgeGraph};

use super::section_fan_walk::EdgeAtPair;
use super::section_surface_walk::{is_a_fold, normal_of, pair_key, ALONG_AN_ARM, LATTICE_FRACTION};

/// Where a line leaves a face: the mesh edge it crosses, how far along the
/// line, the crossing point, the corner the edge was read from and how far along the edge from it,
/// and the mesh vertex it passes through where it lands on an end of the edge (or -1).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceWalkExit {
    pub edge: i32,
    pub distance: f64,
    pub at: Vector3,
    pub from: i32,
    pub along: f64,
    pub vertex: i32,
}

/// The face beside `edge` that holds `direction` - the face whose plane the direction is already in
/// first (the one it was carried out of), then the others with the direction carried across the
/// edge - as `Ok((face, direction_in_face))`. Where nothing holds it (a direction square to the
/// surface, or pointing off a boundary edge), `Err` carries the fallback the caller carries on
/// with: its `preferred` face and the direction read in the home face (`None` where there is no
/// face at all or the home face cannot read it).
pub fn try_face_holding_at_edge(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge: i32,
    preferred: i32,
    direction: Vector3,
) -> Result<(i32, Vector3), Option<(i32, Vector3)>> {
    let mut faces: Vec<i32> = (0..graph.get_face_count(edge))
        .map(|i| graph.get_face(edge, i))
        .collect();

    if faces.is_empty() {
        return Err(None);
    }

    faces.sort_unstable();

    // The face whose plane the direction is already in - the one it was carried out of.
    let mut home = faces[0];
    let mut flattest = f64::MAX;

    for &candidate in &faces {
        let square = direction.get_normal().dot(normal_of(mesh, candidate)).abs();

        if square < flattest - LATTICE_FRACTION {
            flattest = square;
            home = candidate;
        }
    }

    let Some((in_home, held)) = try_held_by(mesh, graph, home, edge, direction) else {
        return Err(None);
    };

    if held >= -LATTICE_FRACTION {
        return Ok((home, in_home));
    }

    for &candidate in &faces {
        if candidate == home {
            continue;
        }

        let carried = unfold(mesh, in_home, edge, graph, home);

        if let Some((in_there, there)) = try_held_by(mesh, graph, candidate, edge, carried) {
            if there >= -LATTICE_FRACTION {
                return Ok((candidate, in_there));
            }
        }
    }

    // Nothing holds it, which is a direction square to the surface or pointing off a boundary edge;
    // the caller's own face is the only answer left that names a plane at all.
    Err(Some((preferred, in_home)))
}

/// `direction` read in `face`'s plane, and how far it turns INTO the face from
/// `edge` (positive inward) - `None` where the direction stands square to the face or the face has no
/// corner off the edge.
pub fn try_held_by(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    face: i32,
    edge: i32,
    direction: Vector3,
) -> Option<(Vector3, f64)> {
    let normal = normal_of(mesh, face);

    let in_face = (direction - (normal * direction.dot(normal))).get_normal();

    if in_face.length_squared() < 0.5 {
        return None;
    }

    let from = position(mesh, graph.get_vertex0(edge));
    let axis = (position(mesh, graph.get_vertex1(edge)) - from).get_normal();
    let drawn = &mesh.faces[face as usize];
    let mut inward = Vector3::ZERO;

    for corner in [drawn.v0, drawn.v1, drawn.v2] {
        if corner != graph.get_vertex0(edge) && corner != graph.get_vertex1(edge) {
            let arm = position(mesh, corner) - from;
            inward = (arm - (axis * arm.dot(axis))).get_normal();
        }
    }

    if inward.length_squared() < 0.5 {
        return None;
    }

    Some((in_face, in_face.dot(inward)))
}

/// The edge of `face` the point `at` stands on, within `lattice` - the nearest
/// such, the later on a tie, and only folds where `folds_only` (callers mostly pass false).
#[allow(clippy::too_many_arguments)]
pub fn try_the_edge_under(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    face: i32,
    at: Vector3,
    lattice: f64,
    folds_only: bool,
) -> Option<i32> {
    let drawn = &mesh.faces[face as usize];
    let corners = [drawn.v0, drawn.v1, drawn.v2];
    let mut nearest = lattice;
    let mut edge = -1;

    for i in 0..3 {
        let from = position(mesh, corners[i]);
        let along = position(mesh, corners[(i + 1) % 3]) - from;
        let length = along.length_squared();
        let s = if length < 1e-300 {
            0.0
        } else {
            ((at - from).dot(along) / length).clamp(0.0, 1.0)
        };
        let off = (from + (along * s) - at).length();

        if off <= nearest {
            if let Some(&candidate) = edge_at_pair.get(&pair_key(corners[i], corners[(i + 1) % 3]))
            {
                if !folds_only || is_a_fold(mesh, graph, candidate) {
                    nearest = off;
                    edge = candidate;
                }
            }
        }
    }

    (edge >= 0).then_some(edge)
}

/// The face of a vertex's fan whose WEDGE holds `direction` best, and the direction read in it, as
/// `Ok((face, direction_in_face))`; `Err` carries the same pair where the best face holds it by
/// less than [`LATTICE_FRACTION`] (`None` where no face could read it).
///
/// A direction running ALONG an arm runs along that edge, and the face holds it. Computed from
/// single-precision vertices it leans off the edge by float noise, and at a convex fold any lean out
/// of the surface projects OUTSIDE both faces' wedges - so measured by the unprojected angle to the
/// arm, the lean is float noise and the arm counts as held, the tie between the two faces sharing it
/// going to the first in the fan. The direction handed on is then the arm ITSELF; only a direction
/// leaning OUT of the wedge is moved.
pub fn try_face_holding(
    mesh: &Mesh,
    vertex: i32,
    direction: Vector3,
    fan_faces: &[i32],
) -> Result<(i32, Vector3), Option<(i32, Vector3)>> {
    let mut face = -1;
    let mut direction_in_face = Vector3::ZERO;
    let mut best = f64::NEG_INFINITY;

    for &candidate in fan_faces {
        let Some((normal, first, second)) = try_wedge(mesh, candidate, vertex) else {
            continue;
        };

        let in_plane = (direction - (normal * direction.dot(normal))).get_normal();

        if in_plane.length_squared() < 0.5 {
            continue;
        }

        // Inside the wedge when the direction turns the same way off both of its arms; the worst of
        // the two is how far INSIDE it sits, so the largest worst is the face that holds it best.
        let mut off_first = first.cross(in_plane).dot(normal);
        let mut off_second = in_plane.cross(second).dot(normal);

        let unit = direction.get_normal();
        let mut along_arm = in_plane;

        if off_first < 0.0 && (unit - first).length() <= ALONG_AN_ARM {
            off_first = 0.0;
            along_arm = first;
        } else if off_second < 0.0 && (unit - second).length() <= ALONG_AN_ARM {
            off_second = 0.0;
            along_arm = second;
        }

        let held = ieee_math::min_f64(off_first, off_second);

        if held > best {
            best = held;
            face = candidate;
            direction_in_face = along_arm;
        }
    }

    if face >= 0 && best >= -LATTICE_FRACTION {
        Ok((face, direction_in_face))
    } else if face >= 0 {
        Err(Some((face, direction_in_face)))
    } else {
        Err(None)
    }
}

/// Where the line from `at` along `direction` leaves `face` - the nearest crossing of
/// one of its edges ahead of `at` (past a lattice of the edge's own length), the crossing landing on
/// an end of the edge named as the mesh vertex it passes through.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn try_exit(
    mesh: &Mesh,
    edge_at_pair: &EdgeAtPair,
    face: i32,
    at: Vector3,
    direction: Vector3,
) -> Option<SurfaceWalkExit> {
    let drawn = &mesh.faces[face as usize];
    let corners = [drawn.v0, drawn.v1, drawn.v2];
    let normal = normal_of(mesh, face);
    let mut nearest = f64::MAX;
    let mut exit: Option<SurfaceWalkExit> = None;

    for i in 0..3 {
        let from_vertex = corners[i];
        let to_vertex = corners[(i + 1) % 3];
        let p = position(mesh, from_vertex);
        let along = position(mesh, to_vertex) - p;
        let denominator = direction.cross(along).dot(normal);

        if denominator.abs() < 1e-300 {
            continue;
        }

        let t = (p - at).cross(along).dot(normal) / denominator;
        let s = (p - at).cross(direction).dot(normal) / denominator;
        let lattice = LATTICE_FRACTION * along.length();

        if t <= lattice || t >= nearest || s < -LATTICE_FRACTION || s > 1.0 + LATTICE_FRACTION {
            continue;
        }

        let Some(&edge) = edge_at_pair.get(&pair_key(from_vertex, to_vertex)) else {
            continue;
        };

        let s = s.clamp(0.0, 1.0);

        // A crossing that lands on an end of the edge is the line going through a mesh vertex; it is
        // named here so the walk can turn through that vertex's fan rather than guess a neighbour.
        let vertex = if s * along.length() <= lattice {
            from_vertex
        } else if (1.0 - s) * along.length() <= lattice {
            to_vertex
        } else {
            -1
        };

        nearest = t;
        exit = Some(SurfaceWalkExit {
            edge,
            distance: t,
            at: p + (along * s),
            from: from_vertex,
            along: s,
            vertex,
        });
    }

    exit
}

/// A direction in `face` carried across `edge` into the face on its far side - the
/// piece along the fold left alone, and the piece square to it in this face's plane put back square
/// to the fold in the NEXT face. Both frames are taken normal-cross-axis, and both normals are
/// outward, so this IS the dihedral.
pub fn unfold(
    mesh: &Mesh,
    direction: Vector3,
    edge: i32,
    graph: &MeshEdgeGraph,
    face: i32,
) -> Vector3 {
    let axis = (position(mesh, graph.get_vertex1(edge)) - position(mesh, graph.get_vertex0(edge)))
        .get_normal();

    let next = if graph.get_face(edge, 0) == face {
        graph.get_face(edge, 1)
    } else {
        graph.get_face(edge, 0)
    };
    let here = normal_of(mesh, face);
    let there = normal_of(mesh, next);

    let across = here.cross(axis);
    let wanted = there.cross(axis);

    ((axis * direction.dot(axis)) + (wanted * direction.dot(across))).get_normal()
}

/// A face's corner at `vertex` as its normal and its two arms (unit), ordered to turn
/// the way the normal says - `None` where the face does not have `vertex` as exactly one corner.
pub fn try_wedge(mesh: &Mesh, face: i32, vertex: i32) -> Option<(Vector3, Vector3, Vector3)> {
    let normal = normal_of(mesh, face);
    let drawn = &mesh.faces[face as usize];
    let at = position(mesh, vertex);

    let arms: Vec<Vector3> = [drawn.v0, drawn.v1, drawn.v2]
        .into_iter()
        .filter(|&corner| corner != vertex)
        .map(|corner| (position(mesh, corner) - at).get_normal())
        .collect();

    if arms.len() != 2 {
        return None;
    }

    let turning_right = arms[0].cross(arms[1]).dot(normal) >= 0.0;

    Some(if turning_right {
        (normal, arms[0], arms[1])
    } else {
        (normal, arms[1], arms[0])
    })
}

/// A vertex's position, read as doubles.
fn position(mesh: &Mesh, vertex: i32) -> Vector3 {
    Vector3::from_vector3_float(mesh.vertices[vertex as usize])
}
