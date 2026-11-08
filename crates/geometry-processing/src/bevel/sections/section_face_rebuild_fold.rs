//! The WALL side of a junction station in the face rebuild (its fold half): the fan the fold cuts
//! in two is substituted either side of the fold ([`try_substitute_either_side_of_the_fold`]), the
//! fold's run of collinear mesh edges is walked down to F1 ([`try_the_folds_run`]), and every mesh
//! vertex the fillet swallows on the way is dropped from the part
//! ([`try_swallow_the_folds_vertices`]). The rest of the rebuild is
//! [`super::section_face_rebuild`], whose state ([`Rebuild`]) these write into.

use solidmesh_math::Vector3;
use solidmesh_mesh::{Mesh, MeshEdgeGraph};

use super::section_face_rebuild::{corner_not_supported, cross, Rebuild, WalkedFan};
use super::section_surface_walk;
use super::{JunctionSection, SectionPool};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// How nearly a face's normal must match a wall's to be read as lying IN that wall.
///
/// A wall at a junction may be tessellated (the bore's own facets at the pierced face's
/// junctions), so this is a reading of which SIDE of the fold a face lies on and not of one plane.
/// The fold is a feature edge, so the two sides stand well apart.
const THE_SAME_WALL: f64 = 0.5;

/// The WALL side of a junction station, whose fan the fold cuts in two: the faces up to the fold
/// take the ARRIVING canal's wall end and those past it the LEAVING canal's, and the two rim chords
/// onto F1 that the split leaves open are recorded for the pass to lay.
///
/// The walk runs from the ARRIVING edge's face round to the leaving edge - `SectionFanWalk.TryWalk`
/// starts there - and `crossed[j]` is the mesh edge between `faces[j]` and `faces[j + 1]`, so the
/// split index is the fold's own place in `crossed`.
///
/// The two chords are written in NEAR and FAR terms rather than in arriving and leaving ones: which
/// canal meets its own wall first is a reading of the two wedges, not of the walk's direction. The
/// patch lays `Pnear -> F1` and `F1 -> Pfar`, so these two lay the opposites, which is what
/// `JunctionSection.DirectedEdges` claims.
pub(super) fn try_substitute_either_side_of_the_fold(
    state: &mut Rebuild<'_>,
    junction: &JunctionSection,
    faces: Vec<i32>,
    crossed: Vec<i32>,
) -> Result<(), BevelAdvisory> {
    let (at_the_fold, beyond, swallowed) =
        try_the_folds_run(junction, &crossed, state.mesh, state.graph, state.pool)?;

    let arriving_at = junction.samples[junction.arriving_wall_end as usize];
    let leaving_at = junction.samples[junction.leaving_wall_end as usize];

    for (j, &face) in faces.iter().enumerate() {
        let boundary = if j as i32 <= at_the_fold {
            arriving_at
        } else {
            leaving_at
        };

        if !state.rewrite(face, junction.chain_vertex, boundary, junction.station) {
            return Err(corner_not_supported(junction.station));
        }
    }

    try_swallow_the_folds_vertices(
        state,
        junction,
        &faces,
        at_the_fold,
        &swallowed,
        arriving_at,
        leaving_at,
    )?;

    let split = at_the_fold as usize;

    state.fans.push(WalkedFan {
        faces: faces[..=split].to_vec(),
        station: junction.station,
    });

    state.fans.push(WalkedFan {
        faces: faces[split + 1..].to_vec(),
        station: junction.station,
    });

    // The chords are wound against the PATCH - the opposite of what it lays onto F1 - which is
    // JunctionSection.LongStripRunsUpTheSamples' to say, and against the wall faces the swallow
    // left behind, which follows because the patch is wound against the long strip. Winding them
    // by hand either way round is what left both chords laying their own wall's directed edges
    // rather than the opposites, and the part non-manifold along them.
    let up = junction.long_strip_runs_up_the_samples();
    let near = junction.samples[junction.near_wall_end() as usize];
    let far = junction.samples[junction.far_wall_end() as usize];

    state.junction_rims.push(if up {
        (junction.fold_point, near, beyond)
    } else {
        (near, junction.fold_point, beyond)
    });

    state.junction_rims.push(if up {
        (far, junction.fold_point, beyond)
    } else {
        (junction.fold_point, far, beyond)
    });

    Ok(())
}

/// THE FOLD'S RUN: the fold leaving a junction is a RUN of collinear mesh edges, and F1 stands a
/// radius down it - on the first edge on a coarse wall, several edges along on a tessellated one.
/// Walked edge by edge from the station, this answers the run's FIRST edge's place in the fan
/// (`at_the_fold`), the vertex the run reaches at or past F1 (`beyond`), and every mesh vertex the
/// fillet SWALLOWS on the way - those standing between the station and F1, which are dropped from
/// the part by [`try_swallow_the_folds_vertices`].
///
/// Collinearity is read against the MESH'S OWN LATTICE (`SectionSurfaceWalk.GridOf`) as the
/// candidate vertex's distance from the fold line, not against an absolute millimetre figure: F1 is
/// constructed out of float normals and trigonometry, so it misses the run's own direction by
/// rather more than a coordinate's ulp, while a DIFFERENT edge of the fan misses the line by a
/// fraction of the part.
///
/// A run that ends before F1 is the fillet running off the end of the fold - refused as a size that
/// does not fit, because that is what it is.
fn try_the_folds_run(
    junction: &JunctionSection,
    crossed: &[i32],
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    pool: &SectionPool,
) -> Result<(i32, i32, Vec<i32>), BevelAdvisory> {
    let mut at_the_fold = -1;
    let mut beyond = -1;
    let mut swallowed = Vec::new();

    let f1 = pool.position_of(junction.fold_point);
    let down = f1 - junction.station;
    let reach = down.length();
    let lattice = section_surface_walk::grid_of(mesh);

    if reach <= lattice {
        return Err(corner_not_supported(junction.station));
    }

    let along = down / reach;

    // The run's FIRST edge is the one the fan crosses - the fan is bounded by the two chain edges
    // and the fold stands inside it, so exactly one crossed edge sets out down the fold.
    for (j, &edge) in crossed.iter().enumerate() {
        let v0 = graph.get_vertex0(edge);
        let v1 = graph.get_vertex1(edge);

        if v0 != junction.chain_vertex && v1 != junction.chain_vertex {
            continue;
        }

        let far = if v0 == junction.chain_vertex { v1 } else { v0 };

        if on_the_folds_line(vertex_at(mesh, far), junction.station, along, lattice) {
            at_the_fold = j as i32;
            beyond = far;

            break;
        }
    }

    if at_the_fold < 0 {
        // Nothing this fan crosses sets out down the fold at all - the walk and the section
        // disagree about which side the wall is, and a chord to F1 would cross faces nothing here
        // has read.
        return Err(corner_not_supported(junction.station));
    }

    // Walk on down the run until it reaches F1. Every vertex short of F1 is swallowed.
    while (vertex_at(mesh, beyond) - junction.station).length() < reach - lattice {
        swallowed.push(beyond);

        let Some(next) =
            try_the_runs_next_vertex(mesh, graph, beyond, junction.station, along, lattice)
        else {
            // The fold is SHORTER than the radius: the fillet would run off its far end onto
            // whatever the mesh does there, which is the size not fitting the face it has to
            // stand on.
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SizeExceedsFaceWidth,
                vec![junction.station.into()],
            ));
        };

        beyond = next;
    }

    Ok((at_the_fold, beyond, swallowed))
}

/// The next vertex down the fold from `from`: the neighbour that stands FARTHER along the fold's
/// line and on it, to within the mesh's own lattice.
fn try_the_runs_next_vertex(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    from: i32,
    station: Vector3,
    along: Vector3,
    lattice: f64,
) -> Option<i32> {
    let mut next = -1;

    let here = (vertex_at(mesh, from) - station).dot(along);
    let mut nearest = f64::MAX;

    // A scan of the edge list rather than a lookup: the run is a handful of vertices at one
    // junction, and the graph carries no edges-at-a-vertex table to ask instead.
    for edge in 0..graph.edge_count() {
        let v0 = graph.get_vertex0(edge);
        let v1 = graph.get_vertex1(edge);

        if v0 != from && v1 != from {
            continue;
        }

        let other = if v0 == from { v1 } else { v0 };
        let at = vertex_at(mesh, other);

        if !on_the_folds_line(at, station, along, lattice) {
            continue;
        }

        let onward = (at - station).dot(along);

        if onward <= here + lattice || onward >= nearest {
            continue;
        }

        nearest = onward;
        next = other;
    }

    (next >= 0).then_some(next)
}

/// Whether a point stands ON the fold's line, ahead of the station: its distance from the line is
/// within the mesh's lattice.
fn on_the_folds_line(at: Vector3, station: Vector3, along: Vector3, lattice: f64) -> bool {
    let from = at - station;
    let down = from.dot(along);

    down > lattice && (from - (along * down)).length() <= lattice
}

/// Drops every vertex the fillet swallows down the fold, the way an absorbed split vertex is
/// dropped: each of its faces is sided by which of the two WALLS meeting at the fold it lies in and
/// rewritten to that wall's own end of the junction's ellipse, and a face left with two corners on
/// one point - a sliver spanning the station and a swallowed vertex, or two swallowed vertices - is
/// dropped from the part rather than kept as a degenerate triangle. The rim chords onto F1 then run
/// across faces that no longer name the swallowed vertex.
///
/// A swallowed vertex carrying a face in NEITHER wall is a third feature meeting the fold inside
/// the fillet; it is refused by name, because nothing here knows where that feature's own rim
/// should go.
fn try_swallow_the_folds_vertices(
    state: &mut Rebuild<'_>,
    junction: &JunctionSection,
    faces: &[i32],
    at_the_fold: i32,
    swallowed: &[i32],
    arriving_at: i32,
    leaving_at: i32,
) -> Result<(), BevelAdvisory> {
    if swallowed.is_empty() {
        return Ok(());
    }

    // The two walls are read off the fan itself, either side of the fold's first edge - the same
    // split the substitution above rides, so a face is sided by the same reading that gave the fan
    // its ends.
    let mesh = state.mesh;
    let arriving_wall = normal_of(mesh, faces[at_the_fold as usize]);
    let leaving_wall = normal_of(mesh, faces[at_the_fold as usize + 1]);
    let mut rewritten = Vec::new();

    for &vertex in swallowed {
        for face in faces_at(state.graph, vertex) {
            let normal = normal_of(mesh, face);
            let to_the_arriving = normal.dot(arriving_wall);
            let to_the_leaving = normal.dot(leaving_wall);

            if to_the_arriving < THE_SAME_WALL && to_the_leaving < THE_SAME_WALL {
                return Err(corner_not_supported(junction.station));
            }

            if !state.rewrite(
                face,
                vertex,
                if to_the_arriving >= to_the_leaving {
                    arriving_at
                } else {
                    leaving_at
                },
                junction.station,
            ) {
                return Err(corner_not_supported(junction.station));
            }

            rewritten.push(face);
        }
    }

    for face in rewritten {
        let drawn = state.rebuilt[face as usize];

        if drawn.0 == drawn.1 || drawn.1 == drawn.2 || drawn.2 == drawn.0 {
            state.swallowed_faces.insert(face);
        }
    }

    Ok(())
}

/// A mesh vertex in double precision.
fn vertex_at(mesh: &Mesh, vertex: i32) -> Vector3 {
    Vector3::from_vector3_float(mesh.vertices[vertex as usize])
}

/// The unit normal of one face of the part, as it stands before any substitution.
fn normal_of(mesh: &Mesh, face: i32) -> Vector3 {
    let drawn = &mesh.faces[face as usize];

    cross(
        vertex_at(mesh, drawn.v0),
        vertex_at(mesh, drawn.v1),
        vertex_at(mesh, drawn.v2),
    )
    .get_normal()
}

/// Every face of the part that names `vertex`, once each.
fn faces_at(graph: &MeshEdgeGraph, vertex: i32) -> Vec<i32> {
    let mut found = Vec::new();

    for edge in 0..graph.edge_count() {
        if graph.get_vertex0(edge) != vertex && graph.get_vertex1(edge) != vertex {
            continue;
        }

        for &face in graph.get_faces(edge) {
            if !found.contains(&face) {
                found.push(face);
            }
        }
    }

    found
}
