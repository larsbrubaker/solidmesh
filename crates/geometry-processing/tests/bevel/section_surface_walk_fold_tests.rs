//! The surface walk ON A FOLD - a direction or a walk
//! running along an edge between two facets, leaning off it by float noise. A fold is where a
//! direction computed from single-precision vertices lands outside BOTH faces' wedges at once.

use solidmesh_math::{math_helper, Vector3};
use solidmesh_mesh::Mesh;

use solidmesh::bevel::sections::section_fan_walk;
use solidmesh::bevel::sections::section_surface_walk_faces::{
    try_face_holding, try_face_holding_at_edge,
};
use solidmesh::bevel::sections::section_surface_walk_try::try_walk;
use solidmesh::bevel::sections::SurfaceWalkRefusal;

use crate::tapered_barrel_test_meshes;

use crate::section_surface_walk_roof::{a_roof, corners, frame};

/// The faces of `mesh` with `vertex` among their corners.
fn fan_of(mesh: &Mesh, vertex: i32) -> Vec<i32> {
    (0..mesh.faces.len())
        .filter(|&f| corners(mesh, f).contains(&vertex))
        .map(|f| f as i32)
        .collect()
}

/// A DIRECTION RUNNING ALONG A FOLD, leaning 3.5e-8 rad out of the convex ridge: projected into
/// either panel it lands just outside the wedge, and a panel holding it must still be picked.
#[test]
fn a_direction_along_a_fold_leaning_off_it_by_float_noise_is_held() {
    let roof = a_roof();
    let frame = frame(&roof);
    let mut apex = 0;

    for (i, v) in roof.vertices.iter().enumerate() {
        if (v.x as f64).abs() < 1e-9 && v.y < 0.0 {
            apex = i as i32;
        }
    }

    let fan = fan_of(&roof, apex);
    let leaning_out = frame.along + Vector3::new(0.0, 0.0, 3.5e-8);

    let (face, in_face) =
        try_face_holding(&roof, apex, leaning_out, &fan).expect("a panel holds it");
    assert!(fan.contains(&face));
    assert!((in_face.dot(frame.along) - 1.0).abs() <= 1e-12);
}

/// SETTING OUT ALONG THE FOLD FROM A POINT ON IT, leaning off the surface by float noise: either
/// panel holds it, and a walk set out that way runs down the hinge. Run at four leans.
fn a_direction_along_the_fold_from_a_point_on_it_leaning_off_by_float_noise_is_held(lean: f64) {
    let roof = a_roof();
    let frame = frame(&roof);
    let graph = roof.get_mesh_edge_graph();
    let edge_at_pair = section_fan_walk::edges_by_vertex_pair(&graph);
    let on_the_axis = |v: i32| (roof.vertices[v as usize].x as f64).abs() < 1e-9;
    let hinge = (0..graph.edge_count())
        .find(|&e| {
            graph.get_face_count(e) == 2
                && on_the_axis(graph.get_vertex0(e))
                && on_the_axis(graph.get_vertex1(e))
        })
        .expect("the roof has a hinge");
    let start = frame.hinge(-3.0);
    let leaning_out = frame.along + Vector3::new(0.0, 0.0, lean);

    let (face, in_face) =
        try_face_holding_at_edge(&roof, &graph, hinge, graph.get_face(hinge, 0), leaning_out)
            .expect("a panel holds it");

    let walk = try_walk(
        &roof,
        &graph,
        &edge_at_pair,
        start,
        face,
        in_face,
        3.0,
        None,
    )
    .unwrap_or_else(|walk| panic!("{:?}", walk.refusal));
    assert!((walk.end - frame.hinge(0.0)).length() < 1e-9);
}

#[test]
fn a_direction_along_the_fold_from_a_point_on_it_leaning_off_by_float_noise_is_held_3_5e_8() {
    a_direction_along_the_fold_from_a_point_on_it_leaning_off_by_float_noise_is_held(3.5e-8);
}

#[test]
fn a_direction_along_the_fold_from_a_point_on_it_leaning_off_by_float_noise_is_held_1e_8() {
    a_direction_along_the_fold_from_a_point_on_it_leaning_off_by_float_noise_is_held(1e-8);
}

#[test]
fn a_direction_along_the_fold_from_a_point_on_it_leaning_off_by_float_noise_is_held_0() {
    a_direction_along_the_fold_from_a_point_on_it_leaning_off_by_float_noise_is_held(0.0);
}

#[test]
fn a_direction_along_the_fold_from_a_point_on_it_leaning_off_by_float_noise_is_held_minus_1e_8() {
    a_direction_along_the_fold_from_a_point_on_it_leaning_off_by_float_noise_is_held(-1e-8);
}

/// A WALK UP A FOLD THAT RUNS INTO A CREASE AT A VERTEX stops there as `CreaseReached`, having
/// travelled the distance to that vertex. On the tapered barrel the 90 degree bore is centred on a
/// wall fold, so the fold rising from the bottom rim at (0, -16, -15) ends exactly on the mouth's
/// lowest vertex; the line leaves its face through that vertex and turns round the vertex's fan
/// until the mouth - a feature - stops the turn (it used to be refused `NoFaceHoldsTheDirection`,
/// and the bottom rim's room jumped from the 4.8579 the mouth allows to 14.0082).
#[test]
fn a_walk_up_a_fold_ending_on_a_feature_vertex_stops_there_as_a_crease() {
    let part = tapered_barrel_test_meshes::the_working_part();
    let resolution = tapered_barrel_test_meshes::the_wall(&part);
    let graph = part.get_mesh_edge_graph();
    let edge_at_pair = section_fan_walk::edges_by_vertex_pair(&graph);
    let features: std::collections::HashSet<i32> = resolution
        .features
        .edges()
        .iter()
        .map(|feature| feature.mesh_edge_index)
        .collect();

    let at = |i: usize| Vector3::from_vector3_float(part.vertices[i]);
    // The nearest, the first of any tie, so a tie picks the same vertex every run.
    let nearest = |to: Vector3| {
        let mut best = 0;
        for i in 1..part.vertices.len() {
            if (at(i) - to).length() < (at(best) - to).length() {
                best = i;
            }
        }
        best
    };

    let rim = nearest(Vector3::new(0.0, -16.0, -15.0));
    let mouth = nearest(Vector3::new(0.0, -15.2667, -9.5));
    let start = at(rim);
    let end = at(mouth);
    let fan: Vec<i32> = (0..part.faces.len())
        .filter(|&f| {
            corners(&part, f).contains(&(rim as i32)) && (part.faces[f].normal.z as f64).abs() < 0.5
        })
        .map(|f| f as i32)
        .collect();

    let (face, in_face) =
        try_face_holding(&part, rim as i32, end - start, &fan).expect("a wall face holds it");

    let stops = |edge: i32| features.contains(&edge);
    let walk = match try_walk(
        &part,
        &graph,
        &edge_at_pair,
        start,
        face,
        in_face,
        30.0,
        Some(&stops),
    ) {
        Ok(walk) => panic!("the walk should stop at the mouth: {walk:?}"),
        Err(walk) => walk,
    };

    assert_eq!(
        walk.refusal,
        SurfaceWalkRefusal::CreaseReached,
        "{:?} {} {:?} face {} start {start:?} rim {rim} mouth {mouth} {end:?} dir {in_face:?}",
        walk.refusal,
        walk.travelled,
        walk.end,
        walk.end_face
    );
    assert!((walk.travelled - (end - start).length()).abs() <= 1e-6);
    assert!((walk.end - end).length() < 1e-9);
}

/// WALKING DOWN A FOLD OF LONG, NARROW PANELS (30 long, 2 wide), set out from its end with a
/// direction leaning off the surface by float noise: the walk runs along the fold rather than being
/// refused for an exit the panel's length amplified past its own lattice. Run at four leans.
fn a_walk_down_a_fold_of_narrow_panels_leaning_off_it_by_float_noise_runs_along_it(lean: f64) {
    let mut roof = Mesh::new();
    let drop = 2.0 * math_helper::degrees_to_radians(3.75).tan();

    roof.create_face(&[
        Vector3::new(0.0, -15.0, 0.0),
        Vector3::new(0.0, 15.0, 0.0),
        Vector3::new(-2.0, 15.0, -drop),
    ]);
    roof.create_face(&[
        Vector3::new(0.0, -15.0, 0.0),
        Vector3::new(-2.0, 15.0, -drop),
        Vector3::new(-2.0, -15.0, -drop),
    ]);
    roof.create_face(&[
        Vector3::new(0.0, -15.0, 0.0),
        Vector3::new(2.0, 15.0, -drop),
        Vector3::new(0.0, 15.0, 0.0),
    ]);
    roof.create_face(&[
        Vector3::new(0.0, -15.0, 0.0),
        Vector3::new(2.0, -15.0, -drop),
        Vector3::new(2.0, 15.0, -drop),
    ]);
    roof.clean_and_merge();

    let graph = roof.get_mesh_edge_graph();
    let edge_at_pair = section_fan_walk::edges_by_vertex_pair(&graph);
    let apex = (0..roof.vertices.len())
        .find(|&i| (roof.vertices[i].x as f64).abs() < 1e-9 && roof.vertices[i].y < 0.0)
        .expect("the roof has an apex") as i32;
    let start = Vector3::from_vector3_float(roof.vertices[apex as usize]);
    let fan = fan_of(&roof, apex);
    let leaning = Vector3::new(0.0, 1.0, lean);

    let (face, in_face) = try_face_holding(&roof, apex, leaning, &fan).expect("a panel holds it");

    let walk = try_walk(
        &roof,
        &graph,
        &edge_at_pair,
        start,
        face,
        in_face,
        1.0,
        None,
    )
    .unwrap_or_else(|walk| panic!("{:?}", walk.refusal));
    assert!((walk.end - (start + Vector3::new(0.0, 1.0, 0.0))).length() < 1e-9);
}

#[test]
fn a_walk_down_a_fold_of_narrow_panels_leaning_off_it_by_float_noise_runs_along_it_1e_9() {
    a_walk_down_a_fold_of_narrow_panels_leaning_off_it_by_float_noise_runs_along_it(1e-9);
}

#[test]
fn a_walk_down_a_fold_of_narrow_panels_leaning_off_it_by_float_noise_runs_along_it_1e_8() {
    a_walk_down_a_fold_of_narrow_panels_leaning_off_it_by_float_noise_runs_along_it(1e-8);
}

#[test]
fn a_walk_down_a_fold_of_narrow_panels_leaning_off_it_by_float_noise_runs_along_it_3_5e_8() {
    a_walk_down_a_fold_of_narrow_panels_leaning_off_it_by_float_noise_runs_along_it(3.5e-8);
}

#[test]
fn a_walk_down_a_fold_of_narrow_panels_leaning_off_it_by_float_noise_runs_along_it_minus_1e_8() {
    a_walk_down_a_fold_of_narrow_panels_leaning_off_it_by_float_noise_runs_along_it(-1e-8);
}
