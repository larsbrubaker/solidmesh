//! Port of `SurfaceFaceWalkbackTests.cs`: the face walkback (`surface_face_walkback`), checked
//! against areas and boundaries worked out on paper. Every case says the same three things in a
//! different shape: the band between the chain edge and the tangent line is gone, exactly and no
//! more; what is left still meets its neighbours vertex for vertex; and the vertices the strip will
//! sew to are standing on the tangent lines. The refusals are `surface_walkback_guard_tests`, and
//! what both read the answer with is `surface_walkback_checks`.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_face_walkback, SurfaceBevelPlan,
};
use geometry_processing::topology::{EdgeConvexity, FaceRegionCache, FeatureEdge};
use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{Vector3, Vector3Float};

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::surface_walkback_checks::{
    area_of, area_of_where, area_on, assert_every_run_is_a_chain_of_real_edges,
    assert_no_new_t_junctions, assert_the_run_vertices_are_on_their_lines, bore_wall, bored_slab,
    boundary_loops, centre_of, chain_of, describe, edge_uses, is_over_the_bore, perimeter_of,
    regions_of, tangent_line_in, top_front_edge, top_is_on_the_left, vertices_on, MEASURED,
};
use crate::topology_test_meshes::{create_l_shaped_prism, create_notched_prism};

fn assert_within(actual: f64, expected: f64, because: &str) {
    assert!(
        (actual - expected).abs() <= MEASURED,
        "{actual} is not {expected} within {MEASURED}: {because}"
    );
}

fn plan(
    chain: &geometry_processing::bevel::BevelSweptChain,
    regions: &Arc<FaceRegionCache>,
    size: f64,
    round_profile: bool,
) -> Rc<SurfaceBevelPlan> {
    SurfaceBevelPlan::try_create(chain, regions, size, round_profile)
        .unwrap_or_else(|reason| panic!("{reason}"))
}

/// One named face plane of the cube, as the vertex test `area_on` reads it by.
type OnThePlane = fn(Vector3Float) -> bool;

fn v3(vertex: Vector3Float) -> Vector3 {
    Vector3::from_vector3_float(vertex)
}

/// One edge of a cube, chamfered 3. Both faces lose a 3 mm band the whole 20 mm across, and the
/// gap that leaves is a single loop: the two tangent lines, and at each end a two-segment step
/// round the original corner of the cube, over the side face that was not walked back at all.
#[test]
fn one_cube_edge_loses_a_three_wide_band_from_each_face_and_leaves_one_gap_loop() {
    let cube = Arc::new(create_cube(20.0, 20.0, 20.0));
    let (regions, features) = regions_of(&cube);
    let regions = Arc::new(regions);
    let chain = chain_of(&features, top_front_edge(&features));

    let plan = plan(&chain, &regions, 3.0, false);

    let walked =
        surface_face_walkback::try_apply_one(&cube, &cube.get_mesh_edge_graph(), &regions, &plan)
            .unwrap_or_else(|reason| panic!("{reason}"));

    assert_eq!(
        cube.faces.len(),
        12,
        "the mesh walked back is not the mesh handed in"
    );

    assert_within(
        area_of(&walked.mesh),
        2400.0 - (2.0 * 20.0 * 3.0),
        "six 20 mm faces less a 20 by 3 band off each of the edge's two",
    );

    // The four faces the chain never reached are still whole squares.
    assert_within(
        area_on(&walked.mesh, |v| (v.z + 10.0).abs() < 1e-6),
        400.0,
        "z = -10",
    );
    assert_within(
        area_on(&walked.mesh, |v| (v.y - 10.0).abs() < 1e-6),
        400.0,
        "y = 10",
    );
    assert_within(
        area_on(&walked.mesh, |v| (v.x - 10.0).abs() < 1e-6),
        400.0,
        "x = 10",
    );
    assert_within(
        area_on(&walked.mesh, |v| (v.x + 10.0).abs() < 1e-6),
        400.0,
        "x = -10",
    );

    assert_within(
        area_on(&walked.mesh, |v| (v.z - 10.0).abs() < 1e-6),
        340.0,
        "the top face is 20 by 17 now",
    );
    assert_within(
        area_on(&walked.mesh, |v| (v.y + 10.0).abs() < 1e-6),
        340.0,
        "and so is the front",
    );

    let loops = boundary_loops(&walked.mesh);

    assert_eq!(
        loops.len(),
        1,
        "one band round the edge leaves one hole in the shell"
    );
    assert_eq!(
        loops[0].len(),
        8,
        "two tangent lines, each broken where the face's own triangulation diagonal crossed it, \
         and a step round the cube's own corner at each end"
    );

    let corners: Vec<Vector3> = loops[0]
        .iter()
        .map(|&v| v3(walked.mesh.vertices[v as usize]))
        .collect();

    for expected in [
        Vector3::new(-10.0, -7.0, 10.0),
        Vector3::new(-7.0, -7.0, 10.0),
        Vector3::new(10.0, -7.0, 10.0),
        Vector3::new(10.0, -10.0, 10.0),
        Vector3::new(10.0, -10.0, 7.0),
        Vector3::new(7.0, -10.0, 7.0),
        Vector3::new(-10.0, -10.0, 7.0),
        Vector3::new(-10.0, -10.0, 10.0),
    ] {
        assert!(
            corners
                .iter()
                .any(|&corner| (corner - expected).length() < 1e-6),
            "the gap runs through {expected}, and goes {}",
            corners
                .iter()
                .map(|c| c.to_string())
                .collect::<Vec<_>>()
                .join(", ")
        );
    }

    assert_every_run_is_a_chain_of_real_edges(&walked);
    assert_the_run_vertices_are_on_their_lines(&walked, &plan);
    assert_no_new_t_junctions(&cube, &walked.mesh);

    for side in [true, false] {
        let runs = &walked.tangent_runs[&(0, side)];
        assert_eq!(
            runs.len(),
            1,
            "a plain square face holds one unbroken stretch of the line"
        );
        assert_eq!(
            runs[0].len(),
            3,
            "the cut ends on the face's two rims and is broken once on the way, where the face's \
             own diagonal crosses it - and the strip has to sew to that point too"
        );
    }
}

/// The whole top rim of a cube, chamfered 3: four edges, one closed chain, and the top face cut by
/// all four of their tangent lines at once. That leaves the 14 by 14 square whose corners are
/// exactly where consecutive tangent lines cross, which is the mitre. Each side face loses a 3 mm
/// band off its top, and the gap is two loops - the square on the top and the ring round the sides.
#[test]
fn the_top_rim_of_a_cube_walks_its_top_back_to_the_mitred_square_and_every_side_down_by_the_setback(
) {
    let cube = Arc::new(create_cube(20.0, 20.0, 20.0));
    let (regions, features) = regions_of(&cube);
    let regions = Arc::new(regions);

    let rim: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| edge.start.z > 9.9 && edge.end.z > 9.9)
        .copied()
        .collect();

    let mut chains = bevel_chain_linker::link(&rim);
    assert_eq!(chains.len(), 1);
    let chain = chains.remove(0);

    let plan = plan(&chain, &regions, 3.0, false);

    let walked =
        surface_face_walkback::try_apply_one(&cube, &cube.get_mesh_edge_graph(), &regions, &plan)
            .unwrap_or_else(|reason| panic!("{reason}"));

    assert_within(
        area_on(&walked.mesh, |v| (v.z - 10.0).abs() < 1e-6),
        196.0,
        "14 by 14 is what four 3 mm walkbacks leave of a 20 mm square",
    );

    let on_the_top: Vec<Vector3> = walked
        .mesh
        .vertices
        .iter()
        .filter(|v| (v.z - 10.0).abs() < 1e-6)
        .map(|&v| v3(v))
        .collect();

    // Four corners, and the one point a closed rim's mitre planes cross at: on a square top those
    // four planes are the two diagonals, so they meet each other at the centre.
    let corners: Vec<Vector3> = on_the_top
        .iter()
        .copied()
        .filter(|at| (at.x * at.x) + (at.y * at.y) > 1e-12)
        .collect();

    assert_eq!(
        on_the_top.len() - corners.len(),
        1,
        "the mitre planes cross at the centre"
    );
    assert_eq!(
        corners.len(),
        4,
        "a square has four corners and the walkback added none of its own"
    );

    for corner in &corners {
        assert!((corner.x.abs() - 7.0).abs() < 1e-6, "corner at {corner}");
        assert!((corner.y.abs() - 7.0).abs() < 1e-6, "corner at {corner}");
    }

    // The same four corners the plan's tangent lines cross at: the walkback found the mitre, it was
    // not told it.
    let top_face = (0..cube.faces.len() as i32)
        .find(|&face| centre_of(&cube, face).z > 9.9)
        .expect("the cube has a top");

    for i in 0..plan.edges.len() {
        let here = tangent_line_in(&plan.edges[i], top_face);
        let next = tangent_line_in(&plan.edges[(i + 1) % plan.edges.len()], top_face);

        let mitre = here.try_cross(&next, 1e-9).expect("the lines cross");
        assert!(
            corners
                .iter()
                .any(|&corner| (corner - mitre).length() < 1e-6),
            "the mitre at {mitre} is a corner of the walked back top"
        );
    }

    let sides: [(&str, OnThePlane); 4] = [
        ("x = 10", |v| (v.x - 10.0).abs() < 1e-6),
        ("x = -10", |v| (v.x + 10.0).abs() < 1e-6),
        ("y = 10", |v| (v.y - 10.0).abs() < 1e-6),
        ("y = -10", |v| (v.y + 10.0).abs() < 1e-6),
    ];

    for (name, on) in sides {
        assert_within(
            area_on(&walked.mesh, on),
            340.0,
            &format!("the {name} face is 20 by 17 now"),
        );
    }

    assert_within(
        area_of(&walked.mesh),
        196.0 + (4.0 * 340.0) + 400.0,
        "the whole walked back shell",
    );

    let loops = boundary_loops(&walked.mesh);

    assert_eq!(
        loops.len(),
        2,
        "a closed band has a rim on the top and a rim on the sides"
    );

    let mut perimeters: Vec<f64> = loops
        .iter()
        .map(|boundary| perimeter_of(&walked.mesh, boundary))
        .collect();
    perimeters.sort_by(|a, b| a.total_cmp(b));

    assert_within(perimeters[0], 4.0 * 14.0, "four top tangent lines");
    assert_within(perimeters[1], 4.0 * 20.0, "four side tangent lines");

    assert_every_run_is_a_chain_of_real_edges(&walked);
    assert_the_run_vertices_are_on_their_lines(&walked, &plan);
    assert_no_new_t_junctions(&cube, &walked.mesh);

    for i in 0..plan.edges.len() as i32 {
        for side in [true, false] {
            let runs = &walked.tangent_runs[&(i, side)];
            assert_eq!(runs.len(), 1);
            assert!(runs[0].len() >= 2);
        }
    }
}

/// A region with a hole in it. The top of a slab with a square bore is a ring, and walking one of
/// its outer rim edges back by 2 takes a band off the outside and leaves the bore exactly where it
/// was - the cut is a plane in the region, and the hole is simply not in its way.
#[test]
fn a_region_with_a_hole_loses_its_band_and_keeps_its_hole() {
    let slab = Arc::new(bored_slab());
    let (regions, features) = regions_of(&slab);
    let regions = Arc::new(regions);

    let rim_edges: Vec<i32> = (0..features.count() as i32)
        .filter(|&i| {
            let edge = &features.edges()[i as usize];

            (edge.start.z - 5.0).abs() < 1e-6
                && (edge.end.z - 5.0).abs() < 1e-6
                && edge.start.y.abs() < 1e-6
                && edge.end.y.abs() < 1e-6
                && edge.start.x.min(edge.end.x) > 9.9
                && edge.start.x.max(edge.end.x) < 20.1
        })
        .collect();
    assert_eq!(rim_edges.len(), 1, "one rim edge between x = 10 and x = 20");

    let chain = chain_of(&features, rim_edges[0]);

    let plan = plan(&chain, &regions, 2.0, false);

    let walked =
        surface_face_walkback::try_apply_one(&slab, &slab.get_mesh_edge_graph(), &regions, &plan)
            .unwrap_or_else(|reason| panic!("{reason}"));

    assert_within(
        area_on(&walked.mesh, |v| (v.z - 5.0).abs() < 1e-6),
        800.0 - (30.0 * 2.0),
        "a 30 by 30 top less its 10 by 10 bore, less a 30 by 2 band",
    );

    assert_within(
        area_on(&walked.mesh, |v| v.y.abs() < 1e-6),
        (30.0 * 5.0) - (30.0 * 2.0),
        "the wall the edge's other face is on walks back the same 2",
    );

    let bore_wall_area = area_of_where(&walked.mesh, |face| {
        bore_wall(centre_of(&walked.mesh, face))
    });

    assert_within(
        bore_wall_area,
        4.0 * 10.0 * 5.0,
        "the bore is untouched, four 10 by 5 walls of it",
    );

    let over_the_bore = (0..walked.mesh.faces.len() as i32)
        .map(|face| centre_of(&walked.mesh, face))
        .any(|centre| is_over_the_bore(centre) && (centre.z - 5.0).abs() < 1e-6);

    assert!(!over_the_bore, "nothing was triangulated across the hole");

    assert_every_run_is_a_chain_of_real_edges(&walked);
    assert_the_run_vertices_are_on_their_lines(&walked, &plan);
    assert_no_new_t_junctions(&slab, &walked.mesh);
}

/// The concave inner corner of an L, filleted 3. Both walls walk back by the same 3 as a box edge
/// would, and what each keeps is the half AWAY from the corner - the material side. Get the side
/// test the wrong way round and the walls keep their slivers and lose their bodies, which is what
/// the y and x bounds below pin.
#[test]
fn the_concave_inner_edge_of_an_l_walks_both_walls_back_on_the_material_side() {
    let prism = Arc::new(create_l_shaped_prism());
    let (regions, features) = regions_of(&prism);
    let regions = Arc::new(regions);
    let concave: Vec<i32> = (0..features.count() as i32)
        .filter(|&i| features.edges()[i as usize].convexity == EdgeConvexity::Concave)
        .collect();
    assert_eq!(concave.len(), 1);
    let chain = chain_of(&features, concave[0]);

    let plan = plan(&chain, &regions, 3.0, true);

    let walked =
        surface_face_walkback::try_apply_one(&prism, &prism.get_mesh_edge_graph(), &regions, &plan)
            .unwrap_or_else(|reason| panic!("{reason}"));

    // The wall at x = 10 runs from y = 10 to y = 20 and is 10 tall; the wall at y = 10 runs from
    // x = 10 to x = 20. Each was 100 square millimetres and each gives up a 3 by 10 band.
    assert_within(
        area_on(&walked.mesh, |v| (v.x - 10.0).abs() < 1e-6),
        100.0 - 30.0,
        "x = 10",
    );
    assert_within(
        area_on(&walked.mesh, |v| (v.y - 10.0).abs() < 1e-6),
        100.0 - 30.0,
        "y = 10",
    );

    for vertex in vertices_on(&walked.mesh, |v| (v.x - 10.0).abs() < 1e-6) {
        assert!(
            vertex.y >= 13.0 - 1e-6,
            "the x = 10 wall keeps the material away from the corner, and has a vertex at {vertex}"
        );
    }

    for vertex in vertices_on(&walked.mesh, |v| (v.y - 10.0).abs() < 1e-6) {
        assert!(
            vertex.x >= 13.0 - 1e-6,
            "and so does the y = 10 wall, which has a vertex at {vertex}"
        );
    }

    assert!(
        vertices_on(&walked.mesh, |v| (v.x - 10.0).abs() < 1e-6)
            .iter()
            .any(|v| (v.y - 13.0).abs() < 1e-6),
        "and it is cut AT 13, not short of it"
    );

    assert_every_run_is_a_chain_of_real_edges(&walked);
    assert_the_run_vertices_are_on_their_lines(&walked, &plan);
    assert_no_new_t_junctions(&prism, &walked.mesh);
}

/// A cut that runs at a NOTCH in the region it walks back. Walking the notched prism's lower
/// `x = 20` rim edge back by 2 draws the tangent line at `x = 18`, which the notch crosses. The band
/// runs from `y = 0` up to the notch's lower face at `y = 7` and stops there, because that is as
/// far as the material joined to the edge goes; the far side of the notch is not walked back, and
/// the proof is that the built result closes.
#[test]
fn a_cut_at_a_notch_stops_at_it_rather_than_taking_a_second_band_off_the_far_side() {
    let prism = Arc::new(create_notched_prism());
    let (regions, features) = regions_of(&prism);
    let regions = Arc::new(regions);

    // The top rim of the +x wall BELOW the notch: (20, 0, 10) to (20, 4, 10).
    let picks: Vec<i32> = (0..features.count() as i32)
        .filter(|&i| {
            let edge = &features.edges()[i as usize];

            (edge.start.x - 20.0).abs() < 1e-6
                && (edge.end.x - 20.0).abs() < 1e-6
                && edge.start.z > 9.9
                && edge.end.z > 9.9
                && edge.start.y.max(edge.end.y) < 5.0
        })
        .collect();
    assert_eq!(picks.len(), 1);

    let chain = chain_of(&features, picks[0]);

    let plan = plan(&chain, &regions, 2.0, false);

    let walked =
        surface_face_walkback::try_apply_one(&prism, &prism.get_mesh_edge_graph(), &regions, &plan)
            .unwrap_or_else(|reason| panic!("{reason}"));

    let on_the_top = &walked.tangent_runs[&(0, top_is_on_the_left(&plan.edges[0]))];

    assert_eq!(
        on_the_top.len(),
        1,
        "the cut reached one stretch of the line and stopped at the notch, so there is one run to \
         sew to - instead: {}",
        describe(&walked, on_the_top)
    );

    let run = &on_the_top[0];

    assert!((v3(walked.mesh.vertices[run[0] as usize]).y - 0.0).abs() <= 1e-5);
    assert!(
        (v3(walked.mesh.vertices[run[run.len() - 1] as usize]).y - 7.0).abs() <= 1e-5,
        "the run ends where the notch's lower face crosses x = 18, not on the far rim at 20"
    );

    // The material beyond the notch is exactly where it was: the top face still reaches x = 20 up
    // there, which it would not if the half space had taken its band too.
    assert!(
        walked
            .mesh
            .vertices
            .iter()
            .any(|v| (v.z - 10.0).abs() < 1e-5 && (v.x - 20.0).abs() < 1e-5 && v.y > 15.0),
        "the far side of the notch was not walked back at all"
    );

    assert_every_run_is_a_chain_of_real_edges(&walked);
    assert_the_run_vertices_are_on_their_lines(&walked, &plan);
    assert_no_new_t_junctions(&prism, &walked.mesh);

    let built: Mesh = surface_bevel_builder::try_build_one(
        &prism,
        &prism.get_mesh_edge_graph(),
        &regions,
        plan,
        1,
        &CancelToken::new(),
    )
    .unwrap_or_else(|reason| panic!("{reason:?}"));

    assert!(
        built.is_manifold(),
        "the bevel closed the gap and opened no other"
    );
    assert_eq!(shell_count(&built), 1);
    assert_eq!(
        edge_uses(&built)
            .values()
            .filter(|&&uses| uses != 2)
            .count(),
        0,
        "no boundary edge is left anywhere in the result"
    );

    // The prism is 376 square millimetres by 10, and the chamfer takes the wedge between x = 18,
    // z = 8 and the chord, bounded by y = 0 and by the notch face 3x + 2y = 68: integral of
    // (x - 18)(68 - 3x)/2 dx from 18 to 20, which is 10.
    assert_within(signed_volume(&built), 3760.0 - 10.0, "the chamfered volume");
}
