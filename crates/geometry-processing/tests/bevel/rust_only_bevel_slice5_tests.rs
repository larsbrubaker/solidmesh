//! Rust-only checks of the fifth bevel slice (`SurfaceVertexPool`, `SurfaceMitreJointStep`,
//! `SurfaceRoomStandingFeature`, `BevelBlendSeam`, `BevelCornerSeam`, and
//! `SurfaceMeshClosure.FoldedEdgeCount`). The C# test classes that exercise them build whole
//! surface bevels (`SurfaceTestBuild`), so they land with the builder; these pin each piece alone.

use std::collections::HashSet;

use geometry_processing::bevel::surface::{
    surface_mesh_closure, surface_mitre_joint_step, surface_room_standing_feature,
    SurfaceVertexPool,
};
use geometry_processing::bevel::{bevel_blend_seam, bevel_corner_seam, BevelEndCap};
use geometry_processing::topology::{EdgeConvexity, FaceGroup, FeatureEdge};
use indexmap::IndexMap;
use polygon_mesh::solids::platonic_solids;
use vector_math::{Vector3, Vector3Float};

// ---------------------------------------------------------------- SurfaceVertexPool

#[test]
fn rust_only_the_weld_distance_scales_with_the_part_and_has_a_floor() {
    assert_eq!(
        SurfaceVertexPool::weld_distance_for(&[]),
        SurfaceVertexPool::SMALLEST_WELD_DISTANCE
    );

    let small = [
        Vector3Float::new(0.0, 0.0, 0.0),
        Vector3Float::new(0.5, 0.0, 0.0),
    ];
    assert_eq!(SurfaceVertexPool::weld_distance_for(&small), 1e-6);

    let large = [
        Vector3Float::new(0.0, 0.0, 0.0),
        Vector3Float::new(1000.0, 0.0, 0.0),
    ];
    let pool = SurfaceVertexPool::new(&large);
    assert_eq!(pool.weld_distance(), 1000.0 * 1e-6);
    assert_eq!(pool.near_enough(), pool.weld_distance() * 10.0);
    assert_eq!(pool.snap_near(), pool.near_enough() * 2.0);
    assert_eq!(
        SurfaceVertexPool::near_enough_for(&large),
        pool.near_enough()
    );
}

#[test]
fn rust_only_the_pool_welds_what_is_one_point_and_appends_what_is_not() {
    let existing = [
        Vector3Float::new(0.0, 0.0, 0.0),
        Vector3Float::new(10.0, 0.0, 0.0),
    ];
    let mut pool = SurfaceVertexPool::new(&existing);

    // every existing vertex keeps its index
    assert_eq!(pool.count(), 2);
    assert_eq!(pool.add(Vector3::new(10.0, 0.0, 0.0)), 1);

    // within a weld (here a micron) is the same vertex, even across a cell boundary
    assert_eq!(pool.add(Vector3::new(4e-7, -4e-7, 0.0)), 0);

    // a ruler away is a new one, and asking again finds it
    let added = pool.add(Vector3::new(5.0, 5.0, 5.0));
    assert_eq!(added, 2);
    assert_eq!(pool.add(Vector3::new(5.0, 5.0, 5.0)), 2);
    assert_eq!(pool.count(), 3);
    assert_eq!(pool.position_of(2), Vector3::new(5.0, 5.0, 5.0));
    assert_eq!(pool.vertices().len(), 3);
}

#[test]
fn rust_only_find_near_takes_the_nearest_vertex_that_qualifies() {
    let existing = [
        Vector3Float::new(0.0, 0.0, 0.0),
        Vector3Float::new(1e-5, 0.0, 0.0),
        Vector3Float::new(2.5e-5, 0.0, 0.0),
    ];
    let pool = SurfaceVertexPool::new(&existing);
    let here = Vector3::new(0.0, 0.0, 0.0);

    assert_eq!(pool.try_find_near(here, 3e-5, None), Some(0));

    let not_the_origin = |index: i32| index != 0;
    assert_eq!(
        pool.try_find_near(here, 3e-5, Some(&not_the_origin)),
        Some(1)
    );

    let only_the_far_one = |index: i32| index == 2;
    assert_eq!(
        pool.try_find_near(here, 2e-5, Some(&only_the_far_one)),
        None
    );
    assert_eq!(
        pool.try_find_near(here, 3e-5, Some(&only_the_far_one)),
        Some(2)
    );
}

// ---------------------------------------------------------------- SurfaceMeshClosure

#[test]
fn rust_only_folded_edges_are_counted_but_open_ones_are_not() {
    // one triangle alone is open, which is ordinary part way through a build
    assert_eq!(
        surface_mesh_closure::folded_edge_count(&[(0, 1, 2)]),
        (0, (0, 0))
    );

    // the same triangle twice the same way folds all three of its edges
    let (folds, first) = surface_mesh_closure::folded_edge_count(&[(0, 1, 2), (0, 1, 2)]);
    assert_eq!(folds, 3);
    assert_eq!(first, (0, 1));

    // and twice opposite ways is a closed (if flat) pair
    assert_eq!(
        surface_mesh_closure::folded_edge_count(&[(0, 1, 2), (0, 2, 1)]).0,
        0
    );
}

// ---------------------------------------------------------------- SurfaceMitreJointStep

type Seams = surface_mitre_joint_step::SideSeams;
type Corners = surface_mitre_joint_step::SideBandCorners;

fn joint(
    into: Vec<(i32, f64)>,
    out_of: Vec<(i32, f64)>,
    leaving: Vec<i32>,
    arriving: Vec<i32>,
) -> (Vec<Seams>, Vec<Corners>) {
    let seams = vec![[Some(into), None], [Some(out_of), None]];
    let corners = vec![
        [[None, Some(leaving)], [None, None]],
        [[Some(arriving), None], [None, None]],
    ];
    (seams, corners)
}

#[test]
fn rust_only_the_joint_step_closes_a_triangular_hole_wound_against_its_rim() {
    // the hole is bounded by one triangle laid the other way: every side open by one, one way round
    let mut faces = vec![(0, 2, 1)];
    let (seams, corners) = joint(vec![(5, 0.0), (0, 1.0)], vec![(0, 0.0)], vec![1], vec![2]);

    surface_mitre_joint_step::sew(&mut faces, Some(&seams), Some(&corners), 0, 1);

    assert_eq!(faces, vec![(0, 2, 1), (0, 1, 2)]);
    assert_eq!(surface_mesh_closure::folded_edge_count(&faces).0, 0);
}

fn every_side_balances(faces: &[(i32, i32, i32)]) -> bool {
    let mut balance = std::collections::HashMap::new();
    for &(v0, v1, v2) in faces {
        for (from, to) in [(v0, v1), (v1, v2), (v2, v0)] {
            *balance.entry((from.min(to), from.max(to))).or_insert(0) +=
                if from < to { 1 } else { -1 };
        }
    }
    balance.values().all(|&net| net == 0)
}

#[test]
fn rust_only_the_joint_step_closes_a_quad_between_two_terminals() {
    // a quad 0-3-2-1 laid as two triangles: its rim is open by one all round, and neither
    // terminal alone sees a triangle of once-used sides because the diagonal is closed
    let mut faces = vec![(0, 3, 2), (0, 2, 1)];
    let (seams, corners) = joint(
        vec![(9, 0.0), (0, 1.0)],
        vec![(3, 0.0), (8, 1.0)],
        vec![1],
        vec![2],
    );

    surface_mitre_joint_step::sew(&mut faces, Some(&seams), Some(&corners), 0, 1);

    assert_eq!(faces[2..], [(0, 2, 3), (0, 1, 2)]);
    // every side now has a use running each way (the diagonal 0-2 is shared by both pairs, so it
    // is used four times, which the fold count would flag; the balance is what the step promises)
    assert!(every_side_balances(&faces));
}

#[test]
fn rust_only_the_joint_step_is_silent_where_nothing_is_open() {
    // a closed pair: nothing open by one, so nothing is laid
    let mut faces = vec![(0, 1, 2), (0, 2, 1)];
    let (seams, corners) = joint(vec![(0, 0.0)], vec![(0, 0.0)], vec![1], vec![2]);

    surface_mitre_joint_step::sew(&mut faces, Some(&seams), Some(&corners), 0, 1);
    assert_eq!(faces.len(), 2);

    // and an absent table lays nothing at all
    let mut open = vec![(0, 2, 1)];
    surface_mitre_joint_step::sew(&mut open, None, Some(&corners), 0, 1);
    assert_eq!(open.len(), 1);
}

// ---------------------------------------------------------------- SurfaceRoomStandingFeature

/// The top face of a 20 mm cube as a region, and a concave rim on it whose far face leans out over
/// the region by `lean` (the feature normal's component along the region's normal).
fn top_region_and_rim(lean: f64) -> (FaceGroup, FeatureEdge) {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let seed = (0..cube.faces.len() as i32)
        .find(|&face| cube.faces[face as usize].normal.z > 0.9)
        .expect("a cube has a top face");
    let region = FaceGroup::create(&cube, seed).expect("the top face is a region");

    let mut rim = region.boundary_edges()[0];
    let landing = Vector3::new(0.0, 0.0, 1.0);
    let feature = Vector3::new((1.0 - lean * lean).sqrt(), 0.0, lean);

    rim.convexity = EdgeConvexity::Concave;
    if region.faces().contains(&rim.face0) {
        rim.normal0 = landing;
        rim.normal1 = feature;
    } else {
        rim.normal1 = landing;
        rim.normal0 = feature;
    }

    (region, rim)
}

#[test]
fn rust_only_a_concave_rim_leaning_out_over_the_region_stands_on_it() {
    let none = HashSet::new();
    let (region, rim) = top_region_and_rim(0.39);

    assert!(surface_room_standing_feature::stands_on(
        &region, &rim, &none, &none
    ));

    let (landing, feature) =
        surface_room_standing_feature::try_the_two_sides(&region, &rim).expect("the region's rim");
    assert_eq!(landing, Vector3::new(0.0, 0.0, 1.0));
    assert!(feature.z > 0.38);

    let beyond = surface_room_standing_feature::try_the_face_beyond(&region, &rim)
        .expect("one side is the region's");
    assert!(!region.faces().contains(&beyond));
}

#[test]
fn rust_only_walls_convex_rims_and_named_out_edges_do_not_stand() {
    let none = HashSet::new();

    // square to the landing plane: a wall
    let (region, square) = top_region_and_rim(0.0);
    assert!(!surface_room_standing_feature::stands_on(
        &region, &square, &none, &none
    ));

    let (region, rim) = top_region_and_rim(0.5);

    let mut convex = rim;
    convex.convexity = EdgeConvexity::Convex;
    assert!(!surface_room_standing_feature::stands_on(
        &region, &convex, &none, &none
    ));

    let beveled = HashSet::from([rim.mesh_edge_index]);
    assert!(!surface_room_standing_feature::stands_on(
        &region, &rim, &beveled, &none
    ));

    let chain = HashSet::from([rim.start_vertex]);
    assert!(!surface_room_standing_feature::stands_on(
        &region, &rim, &none, &chain
    ));

    // a chain vertex the rim's own crease never reaches (the region's rim is convex, so the walk
    // along the concave crease stops at once) leaves it standing
    let elsewhere = HashSet::from([-7]);
    assert!(surface_room_standing_feature::stands_on(
        &region, &rim, &none, &elsewhere
    ));

    let mut foreign = rim;
    foreign.face0 = -1;
    foreign.face1 = -2;
    assert!(!surface_room_standing_feature::stands_on(
        &region, &foreign, &none, &none
    ));
    assert_eq!(
        surface_room_standing_feature::try_the_face_beyond(&region, &foreign),
        None
    );
    assert_eq!(
        surface_room_standing_feature::try_the_two_sides(&region, &foreign),
        None
    );
}

#[test]
fn rust_only_a_cube_tops_walls_are_not_hidden_by_any_feature() {
    let none = HashSet::new();
    let (region, _) = top_region_and_rim(0.5);
    let wall = region.boundary_edges()[0];

    assert!(!surface_room_standing_feature::hides_the_end_of_a_wall(
        &region, &wall, wall.end, 1e-6, &none, &none
    ));
}

// ---------------------------------------------------------------- BevelBlendSeam

#[test]
fn rust_only_a_blend_seam_hands_a_cut_its_capped_handover_and_a_bead_its_face() {
    let position = Vector3::new(1.0, 2.0, 3.0);
    let up = Vector3::new(0.0, 0.0, 1.0);

    let mut handovers = IndexMap::new();
    handovers.insert((4, 7), BevelEndCap::new(up, 5.0));
    handovers.insert((4, 8), BevelEndCap::default());

    assert_eq!(
        bevel_blend_seam::blend_seam(4, 7, position, false, Some(&handovers), None),
        Some(BevelEndCap::new(up, 5.0))
    );
    // an uncapped handover and a missing one both say nothing
    assert_eq!(
        bevel_blend_seam::blend_seam(4, 8, position, false, Some(&handovers), None),
        None
    );
    assert_eq!(
        bevel_blend_seam::blend_seam(5, 7, position, false, Some(&handovers), None),
        None
    );
    assert_eq!(
        bevel_blend_seam::blend_seam(4, 7, position, false, None, None),
        None
    );

    let mut capped = IndexMap::new();
    capped.insert((4, 7), up);

    assert_eq!(
        bevel_blend_seam::blend_seam(4, 7, position, true, Some(&handovers), Some(&capped)),
        Some(BevelEndCap::new(up, 3.0))
    );
    assert_eq!(
        bevel_blend_seam::blend_seam(4, 7, position, true, Some(&handovers), None),
        None
    );
}

// ---------------------------------------------------------------- BevelCornerSeam

#[test]
fn rust_only_a_cutters_cap_stands_off_by_four_ulps_of_its_coordinate() {
    let far_out = Vector3::new(0.0, 0.0, 100.0);

    assert_eq!(bevel_corner_seam::stand_off(true, far_out), 0.0);
    assert_eq!(bevel_corner_seam::stand_off(false, Vector3::ZERO), 1e-6);
    assert_eq!(
        bevel_corner_seam::stand_off(false, far_out),
        100.0 * (4.0 * 1.192092896e-7)
    );
    assert_eq!(bevel_corner_seam::CAP_TAPER, 0.25);
}

#[test]
fn rust_only_a_three_run_convex_corner_drives_through_to_a_stand_off_short_of_the_vertex() {
    let outward = Vector3::new(1.0, 0.0, 0.0);
    let vertex_at = Vector3::new(10.0, 0.0, 0.0);
    let ball_centre = Vector3::new(7.0, 1.0, 1.0);
    let runs = HashSet::from([3]);
    let seam = [Vector3::new(7.0, 0.0, 0.0)];

    let cap = bevel_corner_seam::try_drive_through(
        false,
        3,
        outward,
        vertex_at,
        ball_centre,
        Some(&runs),
        Some(&seam),
    )
    .expect("a listed corner whose ball sits back drives through");

    assert_eq!(cap.plane_normal, outward);
    assert_eq!(cap.plane_distance, 7.0);
    assert!(cap.drives_through);
    assert_eq!(
        cap.drive_through_distance,
        10.0 - bevel_corner_seam::stand_off(false, vertex_at)
    );
    assert_eq!(cap.seam_points.as_deref(), Some(&seam[..]));

    // a bead, an unlisted corner, and a corner whose ball sits on the vertex all keep their caps
    let drive = |is_bead: bool, vertex: i32, centre: Vector3| {
        bevel_corner_seam::try_drive_through(
            is_bead,
            vertex,
            outward,
            vertex_at,
            centre,
            Some(&runs),
            None,
        )
    };
    assert_eq!(drive(true, 3, ball_centre), None);
    assert_eq!(drive(false, 4, ball_centre), None);
    assert_eq!(drive(false, 3, vertex_at), None);
    assert_eq!(
        bevel_corner_seam::try_drive_through(false, 3, outward, vertex_at, ball_centre, None, None),
        None
    );
}
