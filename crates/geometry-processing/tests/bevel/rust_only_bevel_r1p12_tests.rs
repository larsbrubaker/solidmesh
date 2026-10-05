//! Rust-only checks of the R1.P12 package: the level line's chaining of crossed edges, and the
//! run-out's edge and face readings, a mixed corner's sector apex, and the corner patches'
//! gathering of corners.

use std::collections::HashSet;
use std::sync::Arc;

use indexmap::IndexMap;

use polygon_mesh::solids::platonic_solids::create_cube;

use geometry_processing::bevel::bevel_corner_patch::{
    collect, record, BevelCornerPatchSet, Corner,
};
use geometry_processing::bevel::bevel_face_fit::DEFAULT_SURFACE_TOLERANCE_DEGREES;
use geometry_processing::bevel::level_line::band_cut::{chain, key};
use geometry_processing::bevel::level_line::LevelLineChain;
use geometry_processing::bevel::surface::surface_mixed_corner_sector::{
    the_trace, try_apex, try_apex_between, try_cross,
};
use geometry_processing::bevel::surface::{
    SurfaceBandEnd, SurfaceLine, SurfaceMixedCorner, SurfaceMixedCornerSpine,
};
use geometry_processing::bevel::{BevelAdvisoryCode, BevelFaceRunOut};
use geometry_processing::topology::{
    BevelEdgeGroup, BevelEdgeGroupResolution, EdgeConvexity, FeatureEdge, FeatureEdgeSet,
};
use vector_math::Vector3;

#[test]
fn rust_only_an_open_run_is_one_chain_walked_from_its_lower_end() {
    let chains = chain(&[(2, 5), (1, 2), (5, 9)]);
    assert_eq!(chains, vec![LevelLineChain::new(vec![1, 2, 5, 9], false)]);
}

#[test]
fn rust_only_a_ring_closes_without_repeating_its_start() {
    let chains = chain(&[(3, 4), (4, 7), (3, 7)]);
    assert_eq!(chains, vec![LevelLineChain::new(vec![3, 4, 7], true)]);
}

#[test]
fn rust_only_separate_runs_and_rings_come_out_separately_and_nothing_is_nothing() {
    let chains = chain(&[(10, 11), (20, 21), (21, 22), (20, 22)]);
    assert_eq!(
        chains,
        vec![
            LevelLineChain::new(vec![10, 11], false),
            LevelLineChain::new(vec![20, 21, 22], true),
        ]
    );
    assert!(chain(&[]).is_empty());
    assert_eq!(key(5, 2), (2, 5));
    assert_eq!(key(2, 5), (2, 5));
}

#[test]
fn rust_only_the_run_out_reads_a_cubes_edges_faces_and_starts() {
    let mesh = Arc::new(create_cube(20.0, 20.0, 20.0));
    let run_out = BevelFaceRunOut::create(mesh.clone(), None, DEFAULT_SURFACE_TOLERANCE_DEGREES);

    let edge = *run_out.surface_edges.first().unwrap();
    let (from, to) = run_out.try_edge_ends(edge).unwrap();
    assert!(((to - from).length() - 20.0).abs() < 1e-9);
    assert!(run_out.try_edge_ends(-1).is_none());
    assert!(run_out.try_edge_ends(run_out.graph.edge_count()).is_none());

    let (normal0, normal1) = run_out.try_edge_face_normals(edge).unwrap();
    assert!(normal0.dot(normal1).abs() < 1e-9, "a cube's edge is square");

    let v0 = run_out.graph.get_vertex0(edge);
    let v1 = run_out.graph.get_vertex1(edge);
    assert_eq!(run_out.try_edge_between(v1, v0), Some(edge));

    // Every face holds its own centroid and finds itself (or its coplanar partner) as the start.
    for face in 0..mesh.faces.len() as i32 {
        let f = mesh.faces[face as usize];
        let p = |v: i32| Vector3::from_vector3_float(mesh.vertices[v as usize]);
        let centroid = (p(f.v0) + p(f.v1) + p(f.v2)) / 3.0;
        assert!(run_out.contains(face, centroid));
        assert!(!run_out.contains(face, centroid + (p(f.v1) - p(f.v0)) * 5.0));
        assert_eq!(run_out.try_find_start(face, centroid), Some(face));
        assert!(run_out.try_face_normal(face).is_some());
    }

    // A point on another side of the cube is not on the face's surface.
    assert!(run_out
        .try_find_start(0, Vector3::new(1000.0, 1000.0, 1000.0))
        .is_none());
}

fn mixed_corner(hands_over_at_a_mitre: bool) -> SurfaceMixedCorner {
    let plan = crate::rust_only_bevel_r1p06_tests::square_edge_plan(1.0, true);
    SurfaceMixedCorner::new(
        Vector3::new(7.0, 7.0, 7.0),
        1.0,
        0,
        plan.clone(),
        plan,
        Vector3::default(),
        SurfaceBandEnd {
            inward: Vector3::new(1.0, 0.0, 0.0),
            ..Default::default()
        },
        SurfaceBandEnd::default(),
        SurfaceMixedCornerSpine::try_create(
            SurfaceLine::new(Vector3::default(), Vector3::new(0.0, 0.0, 1.0)),
            Vector3::default(),
            Vector3::new(0.0, 0.0, 1.0),
            1.0,
            true,
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            1.0,
            std::f64::consts::FRAC_PI_2,
            Vector3::new(1.0, 0.0, 0.0),
            1e-9,
        )
        .expect("a square spine"),
        Vector3::default(),
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(0.0, 1.0, 0.0),
        hands_over_at_a_mitre,
    )
}

#[test]
fn rust_only_the_sector_apex_is_where_the_radius_meets_the_trace() {
    let corner = mixed_corner(false);
    assert_eq!(the_trace(&corner), Vector3::new(0.0, 1.0, 0.0));

    // The trace through (2, 0, 0) runs along y; the radius through (3, 3, 0) meets it at (2, 2, 0).
    let apex = try_apex(
        &corner,
        Vector3::new(2.0, 0.0, 0.0),
        Vector3::new(3.0, 3.0, 0.0),
    )
    .unwrap();
    assert!((apex - Vector3::new(2.0, 2.0, 0.0)).length() < 1e-12);

    // A trace through the axis gives the axis itself.
    let on_the_axis = try_apex(
        &corner,
        Vector3::new(0.0, 5.0, 0.0),
        Vector3::new(3.0, 3.0, 0.0),
    )
    .unwrap();
    assert_eq!(on_the_axis, corner.outer_rail_centre);

    // A radius parallel to the trace crosses nowhere: refused at the corner.
    let refused = try_apex(
        &corner,
        Vector3::new(2.0, 0.0, 0.0),
        Vector3::new(0.0, 3.0, 0.0),
    )
    .unwrap_err();
    assert_eq!(refused.code(), BevelAdvisoryCode::SurfaceCornerNotSupported);
}

#[test]
fn rust_only_a_sector_between_corners_without_a_mitre_closes_on_the_rail_centre() {
    let plain = mixed_corner(false);
    let mitre = mixed_corner(true);
    let first_on_the_cap = Vector3::new(2.0, 0.0, 0.0);
    let last_on_the_cap = Vector3::new(3.0, 3.0, 0.0);

    assert_eq!(
        try_apex_between(&plain, &plain, first_on_the_cap, last_on_the_cap).unwrap(),
        plain.outer_rail_centre
    );

    // The first hands over at a mitre: its trace through (2, 0, 0) meets the last's radius.
    let apex = try_apex_between(&mitre, &plain, first_on_the_cap, last_on_the_cap).unwrap();
    assert!((apex - Vector3::new(2.0, 2.0, 0.0)).length() < 1e-12);

    let crossed = try_cross(
        &plain,
        Vector3::default(),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 4.0, 0.0),
        Vector3::new(1.0, -1.0, 0.0),
    )
    .unwrap();
    assert!((crossed - Vector3::new(4.0, 0.0, 0.0)).length() < 1e-12);
}

fn corner_edge(mesh_edge_index: i32, from: i32, to: i32, convexity: EdgeConvexity) -> FeatureEdge {
    FeatureEdge {
        mesh_edge_index,
        start_vertex: from,
        end_vertex: to,
        start: Vector3::new(f64::from(from), 0.0, 0.0),
        end: Vector3::new(f64::from(to), 0.0, 0.0),
        convexity,
        ..Default::default()
    }
}

fn resolution(edges: Vec<FeatureEdge>) -> BevelEdgeGroupResolution {
    BevelEdgeGroupResolution {
        group: BevelEdgeGroup::default(),
        features: Arc::new(FeatureEdgeSet::create(&create_cube(1.0, 1.0, 1.0))),
        failure_reason: None,
        seed_edge_index: 0,
        edges,
        chain: None,
        face_group: None,
    }
}

#[test]
fn rust_only_a_corner_takes_each_mesh_edge_once_sorted_and_each_group_once() {
    let mut corner = Corner::new(4, Vector3::new(1.0, 2.0, 3.0));
    corner.add(corner_edge(9, 4, 1, EdgeConvexity::Convex), 2);
    corner.add(corner_edge(3, 4, 2, EdgeConvexity::Convex), 0);
    corner.add(corner_edge(9, 4, 1, EdgeConvexity::Convex), 1);
    corner.add(corner_edge(5, 4, 7, EdgeConvexity::Convex), 2);

    let order: Vec<i32> = corner.edges().iter().map(|e| e.mesh_edge_index).collect();
    assert_eq!(order, vec![3, 5, 9]);
    assert_eq!(corner.groups(), &[2, 0]);

    let mut at_vertex = IndexMap::new();
    record(
        &mut at_vertex,
        4,
        corner.position,
        &corner_edge(1, 4, 0, EdgeConvexity::Convex),
        0,
    );
    record(
        &mut at_vertex,
        4,
        Vector3::default(),
        &corner_edge(2, 4, 3, EdgeConvexity::Convex),
        1,
    );
    assert_eq!(at_vertex.len(), 1);
    assert_eq!(
        at_vertex[&4].position, corner.position,
        "the first position stands"
    );
    assert_eq!(at_vertex[&4].edges().len(), 2);
}

#[test]
fn rust_only_collect_splits_outside_and_inside_corners_over_eligible_groups() {
    let resolutions = vec![
        resolution(vec![
            corner_edge(10, 0, 1, EdgeConvexity::Convex),
            corner_edge(11, 0, 2, EdgeConvexity::Convex),
            corner_edge(12, 0, 3, EdgeConvexity::Concave),
            corner_edge(13, 8, 9, EdgeConvexity::Flat),
        ]),
        resolution(vec![corner_edge(14, 5, 6, EdgeConvexity::Concave)]),
    ];

    let (outside, inside, concave) = collect(&resolutions, &[true, true]);
    let vertices = |corners: &[Corner]| corners.iter().map(|c| c.vertex).collect::<Vec<_>>();
    assert_eq!(vertices(&outside), vec![0, 1, 2]);
    assert_eq!(outside[0].edges().len(), 2);
    assert_eq!(
        vertices(&inside),
        vec![3, 5, 6],
        "vertex 0 has a convex corner, so no inside one"
    );
    assert_eq!(concave, HashSet::from([0, 3, 5, 6]));
    assert_eq!(inside[1].groups(), &[1]);

    let (_, inside, concave) = collect(&resolutions, &[true, false]);
    assert_eq!(vertices(&inside), vec![3]);
    assert_eq!(concave, HashSet::from([0, 3]));

    let empty = BevelCornerPatchSet::empty();
    assert!(empty.patches.is_empty() && empty.warnings.is_empty() && empty.seam_rings.is_empty());
}
