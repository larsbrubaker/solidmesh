//! Rust-only checks of the R1.P14 package beyond the roll's geometry: the walk across a curved
//! landing surface, the mixed junction's gathering of three chain ends, the room's corner gap and
//! fixed-point step, and the wall ran-out's feet.

use std::collections::HashMap;
use std::rc::Rc;
use std::sync::Arc;

use indexmap::IndexMap;
use polygon_mesh::Mesh;

use geometry_processing::bevel::surface::surface_roll_across_the_wall::SurfaceRollAcrossTheWallSides;
use geometry_processing::bevel::surface::surface_room::{
    try_gap_to_the_corner, SurfaceRoomCut, SurfaceRoomFootprint,
};
use geometry_processing::bevel::surface::surface_room_fixed_point::stepped_down;
use geometry_processing::bevel::surface::surface_run_across::nothing_across_it_can_curve;
use geometry_processing::bevel::surface::surface_run_across_walk::{
    try_across_a_curved_region, try_walk_one_section,
};
use geometry_processing::bevel::surface::surface_wall_ran_out::{
    the_profile_can_rest_on_the_crease, try_feet_at,
};
use geometry_processing::bevel::surface::{
    SurfaceBevelPlan, SurfaceEdgePlan, SurfaceEdgeRoles, SurfaceLine, SurfaceMixedJunction,
    SurfacePlanRecipe, SurfaceRunAcrossSectionStep,
};
use geometry_processing::bevel::BevelSweptChain;
use geometry_processing::topology::{ChainEdge, EdgeConvexity, FaceGroup, FeatureEdge};
use vector_math::Vector3;

use crate::topology_test_meshes::{create_arc_prism, create_isosceles_prism};

fn face_normal(mesh: &Mesh, face: usize) -> Vector3 {
    let f = mesh.faces[face];
    let p = |v: i32| Vector3::from_vector3_float(mesh.vertices[v as usize]);
    (p(f.v1) - p(f.v0)).cross(p(f.v2) - p(f.v0)).get_normal()
}

/// The half-disc prism's curved wall, as a region, with the wall's straight end at `(r, 0)`.
fn arc_wall(radius: f64, height: f64) -> (Mesh, FaceGroup) {
    let mesh = create_arc_prism(radius, height, 12);
    let seed = (0..mesh.faces.len())
        .find(|&f| {
            let n = face_normal(&mesh, f);
            n.z.abs() < 0.1 && n.y > 0.3
        })
        .expect("the curved wall has a facet leaning +y");
    let region = FaceGroup::create(&mesh, seed as i32).unwrap();
    (mesh, region)
}

#[test]
fn rust_only_a_flat_region_cannot_curve_and_the_arc_wall_can() {
    let prism = create_isosceles_prism(90.0, 10.0, 5.0);
    let flat = FaceGroup::create(&prism, 0).unwrap();
    assert!(nothing_across_it_can_curve(&prism, &flat, None));

    let (mesh, wall) = arc_wall(10.0, 5.0);
    assert!(wall.faces().len() > 2, "the wall's facets are one region");
    assert!(!nothing_across_it_can_curve(&mesh, &wall, None));
}

#[test]
fn rust_only_a_section_walked_round_the_arc_wall_turns_and_runs_on() {
    let (mesh, wall) = arc_wall(10.0, 5.0);
    let from = Vector3::new(10.0, 0.0, 0.0);
    let to = Vector3::new(10.0, 0.0, 5.0);
    let along = Vector3::new(0.0, 0.0, 1.0);
    let inward = Vector3::new(0.0, 1.0, 0.0);

    let (steps, curved) = try_walk_one_section(&mesh, &wall, from, to, along, inward, 0.5, None)
        .expect("the section walks onto the wall");
    assert!(curved, "the walk turns from facet to facet");
    assert!(steps.len() > 1);
    for pair in steps.windows(2) {
        assert!(pair[1].run_to_entry >= pair[0].run_to_entry);
        assert!(pair[0].face != pair[1].face);
    }
    // Every step stays in the section plane through the middle of the edge.
    for step in &steps {
        assert!((step.from.z - 2.5).abs() < 1e-6 && (step.to.z - 2.5).abs() < 1e-6);
    }

    let mut cache = HashMap::new();
    let across =
        try_across_a_curved_region(&mesh, &wall, from, to, along, inward, Some(&mut cache));
    assert!(across.is_some_and(|value| value > 0.0), "{across:?}");
    assert_eq!(
        cache.len(),
        1,
        "the region's maps are memoised in the caller's cache"
    );
}

fn plan_edge(start_vertex: i32, end_vertex: i32) -> SurfaceEdgePlan {
    let feature = FeatureEdge {
        start_vertex,
        end_vertex,
        start: Vector3::new(f64::from(start_vertex), 0.0, 0.0),
        end: Vector3::new(f64::from(end_vertex), 0.0, 0.0),
        direction: Vector3::new(1.0, 0.0, 0.0),
        convexity: EdgeConvexity::Convex,
        ..Default::default()
    };
    let mesh = create_isosceles_prism(90.0, 10.0, 5.0);
    let region = Arc::new(FaceGroup::create(&mesh, 0).unwrap());
    let line = SurfaceLine::new(Vector3::default(), Vector3::new(1.0, 0.0, 0.0));
    SurfaceEdgePlan::new(
        ChainEdge::new(0, feature, false),
        region.clone(),
        region,
        90.0,
        1.0,
        1.0,
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, -1.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, -1.0),
        line,
        line,
        false,
        line,
        0.0,
        None,
        None,
        None,
    )
}

fn plan(edges: Vec<SurfaceEdgePlan>, is_closed: bool) -> Rc<SurfaceBevelPlan> {
    Rc::new(SurfaceBevelPlan::new(
        edges,
        is_closed,
        1.0,
        false,
        None,
        HashMap::new(),
        false,
        false,
        None,
        None,
        IndexMap::new(),
        SurfaceEdgeRoles::default(),
        SurfacePlanRecipe {
            chain: BevelSweptChain::default(),
            regions: None,
            curve_regions: None,
            round_profile: false,
        },
    ))
}

#[test]
fn rust_only_three_open_chains_ending_at_a_vertex_gather_into_a_junction() {
    let plans = vec![
        plan(vec![plan_edge(1, 2), plan_edge(2, 7)], false),
        plan(vec![plan_edge(7, 3)], false),
        plan(vec![plan_edge(4, 5), plan_edge(5, 7)], false),
    ];

    let ends = SurfaceMixedJunction::try_gather_the_ends(&plans, 7).expect("three ends meet");
    assert_eq!(ends.len(), 3);
    assert_eq!(
        (ends[0].plan, ends[0].edge, ends[0].at_the_start),
        (0, 1, false)
    );
    assert_eq!(
        (ends[1].plan, ends[1].edge, ends[1].at_the_start),
        (1, 0, true)
    );
    assert_eq!(
        (ends[2].plan, ends[2].edge, ends[2].at_the_start),
        (2, 1, false)
    );
    assert_eq!(
        SurfaceMixedJunction::edge_of(&plans, ends[1])
            .edge
            .to_vertex(),
        3
    );

    // A chain passing THROUGH the vertex touches it twice without ending there: not three ends.
    let through = vec![
        plan(vec![plan_edge(1, 7), plan_edge(7, 2)], false),
        plan(vec![plan_edge(7, 3)], false),
    ];
    assert!(SurfaceMixedJunction::try_gather_the_ends(&through, 7).is_none());

    // Two ends only.
    assert!(SurfaceMixedJunction::try_gather_the_ends(&plans[..2], 7).is_none());
}

#[test]
fn rust_only_a_corner_in_front_of_a_band_is_a_gap_it_closes_at() {
    // A band along +x from the origin, its cut setting back +y at one per unit of size.
    let band = SurfaceRoomCut::new(
        1,
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        10.0,
        1.0,
        f64::INFINITY,
    );
    // A corner footprint standing 4 in front of it, reaching back toward it at one per size.
    let corner = SurfaceRoomCut {
        shape: SurfaceRoomFootprint::MitreTip,
        ..SurfaceRoomCut::new(
            1,
            Vector3::new(5.0, 4.0, 0.0),
            Vector3::new(5.0, 4.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, -1.0, 0.0),
            0.0,
            1.0,
            f64::INFINITY,
        )
    };

    let (gap, reaches) = try_gap_to_the_corner(&band, &corner).expect("the corner stands in front");
    assert!((gap - 4.0).abs() < 1e-12);
    assert!((reaches - 1.0).abs() < 1e-12);

    // Behind the band, or a band that is itself a corner footprint: no gap.
    let behind = SurfaceRoomCut {
        from: Vector3::new(5.0, -4.0, 0.0),
        ..corner
    };
    assert!(try_gap_to_the_corner(&band, &behind).is_none());
    assert!(try_gap_to_the_corner(&corner, &band).is_none());
    // Off past the band's far end.
    let past = SurfaceRoomCut {
        from: Vector3::new(30.0, 4.0, 0.0),
        ..corner
    };
    assert!(try_gap_to_the_corner(&band, &past).is_none());
}

#[test]
fn rust_only_a_size_steps_down_one_ruler_while_it_stays_a_size() {
    assert_eq!(stepped_down(5.0, 1.0), 4.0);
    assert_eq!(stepped_down(1.0, 1.0), 1.0);
    assert_eq!(stepped_down(0.5, 1.0), 0.5);
}

fn sides() -> SurfaceRollAcrossTheWallSides {
    let mesh = create_isosceles_prism(90.0, 10.0, 5.0);
    SurfaceRollAcrossTheWallSides::new(
        ChainEdge::new(0, FeatureEdge::default(), false),
        Vector3::new(0.0, 1.0, 0.0),
        true,
        Vec::new(),
        Arc::new(FaceGroup::create(&mesh, 0).unwrap()),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(-1.0, 0.0, 0.0),
    )
}

#[test]
fn rust_only_the_wall_feet_of_a_square_corner_are_the_offset_along_each_face() {
    // Flat face z = 0 with its material at x < 0; the wall x = 0 rising from the edge, its facet
    // normal +x. A circle of radius 2 in the corner touches each face 2 from the edge.
    let side = sides();
    let step = SurfaceRunAcrossSectionStep::new(
        3,
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, -10.0),
        Vector3::new(1.0, 0.0, 0.0),
        0.0,
    );
    let (along_the_trace, into_the_flat_face) = try_feet_at(
        &side,
        Vector3::new(0.0, 0.0, 0.0),
        &step,
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, -1.0),
        -2.0,
    )
    .expect("square planes place a circle");
    assert!((along_the_trace - 2.0).abs() < 1e-12);
    assert!((into_the_flat_face - 2.0).abs() < 1e-12);
}

#[test]
fn rust_only_a_profile_rests_on_a_crease_that_turns_back_under_it() {
    let side = sides();
    let near = SurfaceRunAcrossSectionStep::new(
        1,
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        0.0,
    );
    let turns_up = SurfaceRunAcrossSectionStep::new(
        2,
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(2.0, 0.0, 1.0),
        Vector3::new(-0.5f64.sqrt(), 0.0, 0.5f64.sqrt()),
        1.0,
    );
    let turns_down = SurfaceRunAcrossSectionStep::new(
        2,
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(2.0, 0.0, -1.0),
        Vector3::new(0.5f64.sqrt(), 0.0, 0.5f64.sqrt()),
        1.0,
    );
    // Concave (wedge sign +1): the near facet's outward side is +z, and a far facet heading -z
    // turns back under it.
    assert!(the_profile_can_rest_on_the_crease(
        &side,
        &near,
        &turns_down,
        false
    ));
    assert!(!the_profile_can_rest_on_the_crease(
        &side, &near, &turns_up, false
    ));
    // Convex flips the side.
    assert!(the_profile_can_rest_on_the_crease(
        &side, &near, &turns_up, true
    ));
    // A degenerate trace rests on nothing.
    let point = SurfaceRunAcrossSectionStep {
        to: near.from,
        ..near
    };
    assert!(!the_profile_can_rest_on_the_crease(
        &side, &point, &turns_up, false
    ));
}
