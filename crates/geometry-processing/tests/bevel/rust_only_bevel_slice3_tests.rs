//! Rust-only checks of the third bevel leaf cluster (`BevelSeamRing`, `BevelSharedEdges`,
//! `BevelSurfaceLabels`, `BevelSurfaceWalk`, `FloatGrid`, and the level line's `ChainSetback`,
//! `CurvedEndFace` and `BandGroup`). Their C# coverage comes through whole-bevel test classes of
//! the cyclic core (`HoleRowBridgeTests` and friends need the level-line build); these pin the
//! leaf behaviour now.

use geometry_processing::bevel::bevel_seam_ring::{points, slerp};
use geometry_processing::bevel::bevel_shared_edges::award_once;
use geometry_processing::bevel::level_line::{chain_setback, curved_end_face, BandGroup};
use geometry_processing::bevel::{float_grid, BevelSurfaceLabels, BevelSurfaceWalk};
use geometry_processing::topology::bevel_edge_group_resolver::{create_from_edge, resolve_all};
use geometry_processing::topology::{BevelSelectionKind, FaceGroup, FeatureEdgeSet};
use indexmap::IndexSet;
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::Mesh;
use std::rc::Rc;
use std::sync::Arc;
use vector_math::Vector3;

fn position(mesh: &Mesh, vertex: i32) -> Vector3 {
    Vector3::from_vector3_float(mesh.vertices[vertex as usize])
}

#[test]
fn rust_only_a_seam_runs_from_one_face_normal_to_the_other_at_equal_angles() {
    let centre = Vector3::new(1.0, 2.0, 3.0);
    let seam = points(centre, 2.0, Vector3::UNIT_X, Vector3::UNIT_Y, 4);

    assert_eq!(seam.len(), 5);
    assert_eq!(
        seam[0],
        centre + Vector3::UNIT_X * 2.0,
        "the ends are the normals themselves"
    );
    assert_eq!(seam[4], centre + Vector3::UNIT_Y * 2.0);
    for (step, point) in seam.iter().enumerate() {
        assert!(
            ((*point - centre).length() - 2.0).abs() < 1e-12,
            "on the ball"
        );
        let angle = (point.y - centre.y).atan2(point.x - centre.x);
        assert!((angle - step as f64 * std::f64::consts::FRAC_PI_8).abs() < 1e-12);
    }

    assert_eq!(
        points(centre, 1.0, Vector3::UNIT_X, Vector3::UNIT_Z, 0).len(),
        2
    );
    assert_eq!(
        slerp(Vector3::UNIT_X, Vector3::UNIT_X, 0.5),
        Vector3::UNIT_X
    );
}

#[test]
fn rust_only_the_float_grid_step_is_one_ulp_at_the_largest_coordinate() {
    assert_eq!(float_grid::step_at(0.0), 0.0);
    assert_eq!(float_grid::step_at(1.0), f64::from(f32::EPSILON));
    assert_eq!(float_grid::step_at(-1.0), f64::from(f32::EPSILON));
    // 20 lies in [16, 32), where a float's step is 2^-19 - the 1.9e-6 the seam ring's remarks cite.
    assert_eq!(
        float_grid::step_at_point(Vector3::new(3.0, -20.0, 1.0)),
        2f64.powi(-19)
    );
}

#[test]
fn rust_only_a_square_edge_sets_back_by_the_radius_and_a_chamfer_by_its_size() {
    let square = chain_setback::for_fillet(3.0, Vector3::UNIT_X, Vector3::UNIT_Y * 5.0);
    assert!((square - 3.0).abs() < 1e-12);

    // A 45 degree turn (a blunt edge) asks less, a 135 degree one (a sharp edge) more.
    let blunt = chain_setback::for_fillet(3.0, Vector3::UNIT_X, Vector3::new(1.0, 1.0, 0.0));
    let sharp = chain_setback::for_fillet(3.0, Vector3::UNIT_X, Vector3::new(-1.0, 1.0, 0.0));
    assert!((blunt - 3.0 * (std::f64::consts::PI / 8.0).tan()).abs() < 1e-12);
    assert!(blunt < 3.0 && sharp > 3.0);

    assert_eq!(chain_setback::for_chamfer(2.5), 2.5);
    assert!(chain_setback::is_uniform(2.0, 2.0));
    assert!(!chain_setback::is_uniform(2.0, 2.1));
    assert!(
        !chain_setback::is_uniform(0.0, 0.0),
        "a zero setback is never uniform"
    );
}

#[test]
fn rust_only_a_cube_face_is_flat_and_a_shallow_fold_is_a_round() {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let positions: Vec<Vector3> = (0..cube.vertices.len() as i32)
        .map(|v| position(&cube, v))
        .collect();
    let indices: Vec<(i32, i32, i32)> = cube.faces.iter().map(|f| (f.v0, f.v1, f.v2)).collect();
    let top: Vec<(i32, i32, i32)> = cube
        .faces
        .iter()
        .filter(|f| {
            positions[f.v0 as usize].z > 9.0
                && positions[f.v1 as usize].z > 9.0
                && positions[f.v2 as usize].z > 9.0
        })
        .map(|f| (f.v0, f.v1, f.v2))
        .collect();
    assert_eq!(top.len(), 2);

    assert!(!curved_end_face::bends_on_softly(
        &positions,
        &indices,
        Vector3::UNIT_Z,
        top.clone(),
        30.0
    ));
    assert!(curved_end_face::bends_on_softly(
        &positions,
        &indices,
        Vector3::UNIT_Z,
        top,
        91.0
    ));

    // Two triangles folded 10 degrees along their shared edge: the second is a soft turn.
    let rise = 10f64.to_radians().tan();
    let fold = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(-1.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, rise),
    ];
    let tris = vec![(0, 1, 2), (0, 3, 1)];
    assert!(curved_end_face::bends_on_softly(
        &fold,
        &tris,
        Vector3::UNIT_Z,
        [(0, 1, 2)],
        30.0
    ));
    assert!(
        !curved_end_face::bends_on_softly(&fold, &tris, Vector3::UNIT_Z, [(9, 9, 9)], 30.0),
        "a seed that is not a triangle grows nothing"
    );
}

#[test]
fn rust_only_an_edge_named_twice_goes_to_the_first_group_that_names_it() {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let features = FeatureEdgeSet::create(&cube);
    let tolerance = FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES;
    let group = |i| create_from_edge(&features, i, BevelSelectionKind::Edge, tolerance).unwrap();

    let (distinct, _) = resolve_all(&[group(0), group(1)], &cube);
    let (unshared, flags) = award_once(distinct.clone());
    assert_eq!(flags, vec![false, false]);
    assert_eq!(unshared.len(), 2);
    assert_eq!(unshared[1].edges, distinct[1].edges);

    let (resolutions, _) = resolve_all(&[group(0), group(0), group(1)], &cube);
    let (awarded, flags) = award_once(resolutions.clone());
    assert_eq!(
        flags,
        vec![false, true, false],
        "the second group gave its only edge away"
    );
    assert_eq!(awarded[0].edges, resolutions[0].edges);
    assert!(awarded[1].is_resolved() && awarded[1].edges.is_empty() && awarded[1].chain.is_none());
    assert_eq!(awarded[2].edges, resolutions[2].edges);
}

#[test]
fn rust_only_a_cube_labels_six_surfaces_once_per_tolerance() {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let mut labels = BevelSurfaceLabels::new(&cube, None);

    let first = labels.at(30.0).to_vec();
    assert_eq!(first.len(), cube.faces.len());
    let mut distinct = first.clone();
    distinct.sort();
    distinct.dedup();
    assert_eq!(distinct.len(), 6);
    assert_eq!(labels.at(30.0), first.as_slice());

    // At a tolerance past the cube's 90 degree turns the whole part is one surface.
    let one = labels.at(120.0).to_vec();
    assert!(one.iter().all(|&s| s == one[0]));
}

#[test]
fn rust_only_a_walk_across_a_cube_face_stops_at_the_opposite_crease() {
    let cube = Arc::new(platonic_solids::create_cube(20.0, 20.0, 20.0));
    let graph = Rc::new(cube.get_mesh_edge_graph());
    let features = FeatureEdgeSet::create(&cube);
    let surface_edges: IndexSet<i32> = features.edges().iter().map(|e| e.mesh_edge_index).collect();
    let walk = BevelSurfaceWalk::new(cube.clone(), graph.clone(), Rc::new(surface_edges));

    // A top edge running along x at y = -10.
    let edge = *features
        .edges()
        .iter()
        .find(|e| e.start.z > 9.0 && e.end.z > 9.0 && e.start.y < -9.0 && e.end.y < -9.0)
        .unwrap();
    let face = [edge.face0, edge.face1]
        .into_iter()
        .find(|&f| walk.try_get_face_normal(f).unwrap().z > 0.5)
        .unwrap();
    let across = walk.try_across_direction(face, &edge).unwrap();
    assert!(
        (across - Vector3::UNIT_Y).length() < 1e-12,
        "into the top face"
    );

    let start = (edge.start + edge.end) * 0.5;
    let mut crossed = Vec::new();
    let stop = walk.walk(
        face,
        start,
        across,
        edge.start_vertex,
        edge.end_vertex,
        100.0,
        Some(&mut crossed),
    );

    assert!((stop.travelled - 20.0).abs() < 1e-9);
    assert!(stop.blocking_edge >= 0);
    let (a, b) = (
        graph.get_vertex0(stop.blocking_edge),
        graph.get_vertex1(stop.blocking_edge),
    );
    assert!(
        position(&cube, a).y > 9.0 && position(&cube, b).y > 9.0 && position(&cube, a).z > 9.0,
        "the far top edge"
    );
    assert_eq!(crossed.len(), 2, "both triangles of the top face");
    assert_eq!(stop.stopped_in_face, crossed[1]);

    // A limit reached at the diagonal stops the walk there, unblocked.
    let short = walk.walk(
        face,
        start,
        across,
        edge.start_vertex,
        edge.end_vertex,
        1.0,
        None,
    );
    assert!((short.travelled - 10.0).abs() < 1e-9);
    assert_eq!(short.blocking_edge, -1);
}

#[test]
fn rust_only_a_band_group_is_a_side_group_until_told_otherwise() {
    let group = BandGroup::default();
    assert_eq!(group.end_of, -1);
    assert!(group.members.is_empty() && group.carried_ends.is_empty());
}
