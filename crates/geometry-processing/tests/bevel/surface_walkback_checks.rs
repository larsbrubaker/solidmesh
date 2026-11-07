//! Test helper: what the walkback's tests read off a rewritten mesh and what they hold it to -
//! areas and boundaries, the soundness checks every case ends with, and the fixtures more than one
//! case is cut from. Each check is a plain function that panics with a message saying what failed.

#![allow(dead_code)]

use std::collections::{HashMap, HashSet};
use std::sync::Arc;

use geometry_processing::bevel::surface::{
    SurfaceBevelPlan, SurfaceEdgePlan, SurfaceLine, SurfaceWalkbackResult,
};
use geometry_processing::bevel::{bevel_chain_linker, BevelSweptChain};
use geometry_processing::topology::{FaceRegionCache, FeatureEdgeSet};
use indexmap::IndexMap;
use polygon_mesh::{FaceList, Mesh};
use vector_math::{Vector3, Vector3Float};

use crate::topology_test_meshes;

/// Areas and lengths come out of float vertices through sums, so they are checked to a ten
/// thousandth of a millimetre rather than exactly.
pub const MEASURED: f64 = 1e-4;

fn at(mesh: &Mesh, vertex: i32) -> Vector3 {
    Vector3::from_vector3_float(mesh.vertices[vertex as usize])
}

/// Which side of the walk the top face (z = 10) falls on for a planned edge.
pub fn top_is_on_the_left(edge: &SurfaceEdgePlan) -> bool {
    edge.edge.left_normal().z > 0.9
}

pub fn describe(walked: &SurfaceWalkbackResult, runs: &[Vec<i32>]) -> String {
    runs.iter()
        .map(|run| {
            run.iter()
                .map(|&vertex| at(&walked.mesh, vertex).to_string())
                .collect::<Vec<_>>()
                .join(", ")
        })
        .collect::<Vec<_>>()
        .join(" | ")
}

/// That every run really is a run: each pair of vertices next to each other in it is joined by an
/// edge of a face of the result.
pub fn assert_every_run_is_a_chain_of_real_edges(walked: &SurfaceWalkbackResult) {
    let edges: HashSet<(i32, i32)> = edge_uses(&walked.mesh).keys().copied().collect();

    for (key, runs) in &walked.tangent_runs {
        for run in runs {
            for i in 0..run.len().saturating_sub(1) {
                assert!(
                    edges.contains(&(run[i].min(run[i + 1]), run[i].max(run[i + 1]))),
                    "{} and {} are next to each other in a run of {:?}, so the face has to have an edge between them",
                    at(&walked.mesh, run[i]),
                    at(&walked.mesh, run[i + 1]),
                    key
                );
            }
        }
    }
}

/// That every vertex of every reported run really is on its tangent line, in the face's own plane
/// at the planned setback from the edge, and in the chain's walk order along the line.
pub fn assert_the_run_vertices_are_on_their_lines(
    walked: &SurfaceWalkbackResult,
    plan: &SurfaceBevelPlan,
) {
    for (i, edge) in plan.edges.iter().enumerate() {
        for left in [true, false] {
            let side = if left { "left" } else { "right" };
            let runs = walked.tangent_runs.get(&(i as i32, left));

            assert!(runs.is_some(), "edge {i} walks back the face on its {side}");

            let runs = runs.unwrap();
            let line = if left {
                edge.left_tangent_line
            } else {
                edge.right_tangent_line
            };
            let normal = if left {
                edge.edge.left_normal()
            } else {
                edge.edge.right_normal()
            };
            let setback = if left {
                edge.left_setback
            } else {
                edge.right_setback
            };
            let edge_line = SurfaceLine::new(edge.from(), edge.edge.direction());

            // A side with a band is one the profile was ROLLED along (SurfaceRollAcrossTheWall):
            // its tangent line lies in the plane of the facet the profile touches rather than in
            // the chain face's, and it stands the setback away measured ALONG the surface. Both of
            // the readings below are about the chain face, so on a rolled side they are asking the
            // wrong question - the line itself is still the thing to sew to, and that is checked.
            let rolled = if left {
                edge.left_band.is_some()
            } else {
                edge.right_band.is_some()
            };

            assert!(
                !runs.is_empty(),
                "a tangent line the strip can sew to needs a run, edge {i} {side}"
            );

            let mut last_along = f64::NEG_INFINITY;

            for run in runs {
                assert!(
                    run.len() >= 2,
                    "a run the strip can sew to needs both its ends, edge {i} {side}"
                );

                for &vertex in run {
                    let position = at(&walked.mesh, vertex);

                    assert!(
                        line.distance_to(position) < 1e-6,
                        "{position} was reported on the tangent line"
                    );

                    if !rolled {
                        assert!(
                            (position - edge.from()).dot(normal).abs() < 1e-6,
                            "{position} lies in the face's own plane"
                        );
                        assert!(
                            (edge_line.distance_to(position) - setback).abs() < 1e-6,
                            "{position} stands this side's planned setback off the edge"
                        );
                    }

                    let along = position.dot(edge.edge.direction());

                    assert!(
                        along > last_along,
                        "and they come in the chain's own order, run after run"
                    );

                    last_along = along;
                }
            }
        }
    }
}

/// That the rewrite left a clean surface: every edge is used by one face (a gap boundary) or two,
/// and the walkback put no vertex part way along an edge of a face that does not use it. Measured
/// against the mesh walked back rather than in the absolute, because a fixture can arrive with one
/// (the L prism's top cap has a degenerate triangle across its reflex corner).
pub fn assert_no_new_t_junctions(before: &Mesh, after: &Mesh) {
    for (key, uses) in edge_uses(after) {
        assert!(
            uses <= 2,
            "the edge {key:?} is used by {uses} faces, so the rewrite folded something over"
        );
    }

    let already = t_junctions_in(before);

    for junction in t_junctions_in(after) {
        assert!(
            already.contains(&junction),
            "the walkback left a T junction the mesh did not have: {junction}"
        );
    }
}

/// Every vertex standing part way along an edge of a face that does not use it.
pub fn t_junctions_in(mesh: &Mesh) -> HashSet<String> {
    let mut found = HashSet::new();

    for (low, high) in edge_uses(mesh).into_keys() {
        let from = at(mesh, low);
        let to = at(mesh, high);
        let run = to - from;

        for vertex in 0..mesh.vertices.len() as i32 {
            if vertex == low || vertex == high {
                continue;
            }

            let position = at(mesh, vertex);
            let r#where = (position - from).dot(run) / run.length_squared();

            if r#where > 1e-6
                && r#where < 1.0 - 1e-6
                && (from + (run * r#where) - position).length() <= 1e-6
            {
                found.insert(format!(
                    "{} on {} to {}",
                    rounded(position),
                    rounded(from),
                    rounded(to)
                ));
            }
        }
    }

    found
}

/// Banker's rounding to four places.
fn round4(value: f64) -> f64 {
    (value * 10000.0).round_ties_even() / 10000.0
}

pub fn rounded(position: Vector3) -> String {
    format!(
        "({}, {}, {})",
        round4(position.x),
        round4(position.y),
        round4(position.z)
    )
}

/// The same slab 20 thick, so a 12 mm setback has wall to land on as well as top.
pub fn tall_bored_slab() -> Mesh {
    let spans = [0.0, 10.0, 20.0, 30.0];

    topology_test_meshes::create_grid_solid(&spans, &spans, &[0.0, 20.0], |i, j, k| {
        (0..3).contains(&i) && (0..3).contains(&j) && k == 0 && !(i == 1 && j == 1)
    })
}

/// A 30 mm slab 20 thick with TWO 10 mm square bores through it, 4.5 mm apart: one at
/// `y = 10..16` and one at `y = 20.5..26.5`, both running `x = 10..20` - the picture hanger's
/// upright, whose outer face carries two of its three drilled holes.
pub fn twice_bored_slab() -> Mesh {
    topology_test_meshes::create_grid_solid(
        &[0.0, 10.0, 20.0, 30.0],
        &[0.0, 10.0, 16.0, 20.5, 26.5, 30.0],
        &[0.0, 20.0],
        |i, j, k| {
            (0..3).contains(&i) && (0..5).contains(&j) && k == 0 && !(i == 1 && (j == 1 || j == 3))
        },
    )
}

/// A 30 mm slab 5 thick with a 10 mm square bore through the middle of it.
pub fn bored_slab() -> Mesh {
    let spans = [0.0, 10.0, 20.0, 30.0];

    topology_test_meshes::create_grid_solid(&spans, &spans, &[0.0, 5.0], |i, j, k| {
        (0..3).contains(&i) && (0..3).contains(&j) && k == 0 && !(i == 1 && j == 1)
    })
}

pub fn is_over_the_bore(point: Vector3) -> bool {
    point.x > 10.0 && point.x < 20.0 && point.y > 10.0 && point.y < 20.0
}

pub fn bore_wall(centre: Vector3) -> bool {
    is_over_the_bore(Vector3::new(
        centre.x.clamp(10.001, 19.999),
        centre.y.clamp(10.001, 19.999),
        centre.z,
    )) && (((centre.x - 10.0).abs() < 1e-6 || (centre.x - 20.0).abs() < 1e-6)
        && centre.y > 10.0
        && centre.y < 20.0
        || ((centre.y - 10.0).abs() < 1e-6 || (centre.y - 20.0).abs() < 1e-6)
            && centre.x > 10.0
            && centre.x < 20.0)
}

/// The same mesh without one of its faces, which is how a rim is made to run out.
pub fn without_face(mesh: &Mesh, face_to_drop: i32) -> Mesh {
    let mut faces = FaceList::new();

    for (face, f) in mesh.faces.iter().enumerate() {
        if face as i32 != face_to_drop {
            faces.add(f.v0, f.v1, f.v2, &mesh.vertices);
        }
    }

    Mesh::from_vertices_and_faces(&mesh.vertices, &faces)
}

/// How many faces use each (low, high) edge, in the order the faces first reach them.
pub fn edge_uses(mesh: &Mesh) -> IndexMap<(i32, i32), i32> {
    let mut uses = IndexMap::new();

    for face in mesh.faces.iter() {
        for (first, second) in [(face.v0, face.v1), (face.v1, face.v2), (face.v2, face.v0)] {
            *uses
                .entry((first.min(second), first.max(second)))
                .or_insert(0) += 1;
        }
    }

    uses
}

/// The loops of edges used by exactly one face - the holes the strip will fill.
pub fn boundary_loops(mesh: &Mesh) -> Vec<Vec<i32>> {
    let mut at_vertex: HashMap<i32, Vec<usize>> = HashMap::new();
    let open: Vec<(i32, i32)> = edge_uses(mesh)
        .into_iter()
        .filter(|&(_, uses)| uses == 1)
        .map(|(key, _)| key)
        .collect();

    for (i, &(low, high)) in open.iter().enumerate() {
        for vertex in [low, high] {
            at_vertex.entry(vertex).or_default().push(i);
        }
    }

    let mut loops = Vec::new();
    let mut walked = vec![false; open.len()];

    for seed in 0..open.len() {
        if walked[seed] {
            continue;
        }

        let mut boundary = vec![open[seed].0];
        let mut at = open[seed].1;
        let mut edge = seed;

        walked[seed] = true;

        while at != boundary[0] {
            boundary.push(at);

            let next = *at_vertex[&at]
                .iter()
                .find(|&&candidate| candidate != edge)
                .expect("a boundary vertex has a second open edge");

            edge = next;
            walked[next] = true;
            at = if open[next].0 == at {
                open[next].1
            } else {
                open[next].0
            };
        }

        loops.push(boundary);
    }

    loops
}

pub fn perimeter_of(mesh: &Mesh, boundary: &[i32]) -> f64 {
    let mut length = 0.0;

    for i in 0..boundary.len() {
        length += (at(mesh, boundary[(i + 1) % boundary.len()]) - at(mesh, boundary[i])).length();
    }

    length
}

pub fn area_of(mesh: &Mesh) -> f64 {
    area_of_where(mesh, |_| true)
}

/// The area of the faces `include` accepts.
pub fn area_of_where(mesh: &Mesh, include: impl Fn(i32) -> bool) -> f64 {
    let mut area = 0.0;

    for face in 0..mesh.faces.len() as i32 {
        if !include(face) {
            continue;
        }

        let corners = corners(mesh, face);

        area += (corners[1] - corners[0])
            .cross(corners[2] - corners[0])
            .length()
            / 2.0;
    }

    area
}

/// The area of the faces all of whose corners stand on one plane.
pub fn area_on(mesh: &Mesh, on_the_plane: impl Fn(Vector3Float) -> bool) -> f64 {
    area_of_where(mesh, |face| {
        let f = &mesh.faces[face as usize];
        on_the_plane(mesh.vertices[f.v0 as usize])
            && on_the_plane(mesh.vertices[f.v1 as usize])
            && on_the_plane(mesh.vertices[f.v2 as usize])
    })
}

/// The vertices used by the faces standing on one plane.
pub fn vertices_on(mesh: &Mesh, on_the_plane: impl Fn(Vector3Float) -> bool) -> Vec<Vector3> {
    let mut found = Vec::new();

    for (face, f) in mesh.faces.iter().enumerate() {
        if on_the_plane(mesh.vertices[f.v0 as usize])
            && on_the_plane(mesh.vertices[f.v1 as usize])
            && on_the_plane(mesh.vertices[f.v2 as usize])
        {
            found.extend(corners(mesh, face as i32));
        }
    }

    found
}

pub fn corners(mesh: &Mesh, face_index: i32) -> [Vector3; 3] {
    let face = &mesh.faces[face_index as usize];

    [at(mesh, face.v0), at(mesh, face.v1), at(mesh, face.v2)]
}

pub fn centre_of(mesh: &Mesh, face_index: i32) -> Vector3 {
    let corners = corners(mesh, face_index);

    (corners[0] + corners[1] + corners[2]) / 3.0
}

pub fn tangent_line_in(plan: &SurfaceEdgePlan, face_index: i32) -> SurfaceLine {
    if plan.left_region.faces().contains(&face_index) {
        plan.left_tangent_line
    } else {
        plan.right_tangent_line
    }
}

/// The regions and the feature edges they were read with.
pub fn regions_of(mesh: &Arc<Mesh>) -> (FaceRegionCache, FeatureEdgeSet) {
    let features = FeatureEdgeSet::create(mesh);
    let regions = FaceRegionCache::new(
        mesh.clone(),
        Some(mesh.get_mesh_edge_graph()),
        features.flat_angle_degrees(),
    );

    (regions, features)
}

pub fn chain_of(features: &FeatureEdgeSet, edge_index: i32) -> BevelSweptChain {
    let mut chains = bevel_chain_linker::link(&[features.edges()[edge_index as usize]]);

    assert_eq!(chains.len(), 1, "one edge links into one chain");

    chains.remove(0)
}

/// The top front edge of a 20 mm cube: from (-10, -10, 10) to (10, -10, 10).
pub fn top_front_edge(features: &FeatureEdgeSet) -> i32 {
    let found: Vec<i32> = (0..features.count() as i32)
        .filter(|&i| {
            let edge = &features.edges()[i as usize];

            (edge.start.y + 10.0).abs() < 1e-9
                && (edge.end.y + 10.0).abs() < 1e-9
                && edge.start.z > 9.9
                && edge.end.z > 9.9
        })
        .collect();

    assert_eq!(found.len(), 1, "a cube has one top front edge");

    found[0]
}
