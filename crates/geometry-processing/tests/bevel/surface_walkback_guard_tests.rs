//! What the walkback (`surface_face_walkback`) turns DOWN - a band that would take the material out
//! from under something standing on the face (a boss, a bore's mouth, a foot edge crossing the band
//! with both ends outside it, a foot another chain is bevelling), and a region whose rim runs out
//! so its neighbours cannot be split to match - and the holes it does not: one behind the band, or
//! one the band stops short of.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::surface::{surface_face_walkback, SurfaceBevelPlan};
use geometry_processing::bevel::{
    bevel_chain_linker, BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode, BevelSweptChain,
};
use geometry_processing::topology::{
    EdgeConvexity, FaceGroup, FaceRegionCache, FeatureEdge, FeatureEdgeSet,
};
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::Mesh;
use vector_math::Vector3;

use crate::surface_walkback_checks::{
    area_on, assert_every_run_is_a_chain_of_real_edges, assert_no_new_t_junctions,
    assert_the_run_vertices_are_on_their_lines, chain_of, corners, regions_of, tall_bored_slab,
    top_front_edge, twice_bored_slab, without_face, MEASURED,
};
use crate::topology_test_meshes;

fn named_point(reason: &BevelAdvisory) -> Vector3 {
    let BevelAdvisoryArg::Vector3(at) = reason.args()[0] else {
        panic!("the refusal names a place: {reason}");
    };

    at
}

fn plan(
    chain: &BevelSweptChain,
    regions: &Arc<FaceRegionCache>,
    size: f64,
    round_profile: bool,
) -> Rc<SurfaceBevelPlan> {
    SurfaceBevelPlan::try_create(chain, regions, size, round_profile)
        .unwrap_or_else(|reason| panic!("the edges themselves are sound - {reason}"))
}

fn count_vertices(mesh: &Mesh, is_it: impl Fn(Vector3) -> bool) -> usize {
    mesh.vertices
        .iter()
        .filter(|vertex| is_it(vertex.as_vector3()))
        .count()
}

/// The refusal for something STANDING on the face being walked back. A square boss on a plate makes
/// the plate's top a ring with an 8 mm square hole in it, and the boss's four walls stand on that
/// hole's rim. Walk the plate's front rim back by 7 and the band reaches the hole, so the walkback
/// says so and cuts nothing; a setback that keeps clear of the boss is the same cut on the same
/// ring face and goes through.
#[test]
fn a_band_that_reaches_a_feature_standing_on_the_face_is_refused() {
    let plate = Arc::new(topology_test_meshes::create_square_boss_on_plate());
    let (regions, features) = regions_of(&plate);
    let regions = Arc::new(regions);

    // The plate's whole front rim at z = 4: three edges, because the boss's grid puts vertices at
    // x = 6 and x = 14 along it.
    let chain = the_front_rim(&features);

    let into_the_boss = plan(&chain, &regions, 7.0, false);

    let reason = surface_face_walkback::try_apply_one(
        &plate,
        &plate.get_mesh_edge_graph(),
        &regions,
        &into_the_boss,
    )
    .expect_err("a 7 mm band from y = 0 reaches the boss's foot at y = 6");
    assert_eq!(
        reason.code(),
        BevelAdvisoryCode::SurfaceCutMeetsStandingFeature
    );

    // One of the foot's two FRONT corners, and not pinned to either: which of its ends is named is
    // only the order the chain's three cuts are held in.
    let named = named_point(&reason);
    assert!(
        (named - Vector3::new(6.0, 6.0, 4.0)).length() < 1e-6
            || (named - Vector3::new(14.0, 6.0, 4.0)).length() < 1e-6,
        "it names a corner of the foot the band reached - instead {reason}"
    );

    // Three millimetres stops three short of the foot, and that same ring face cuts without
    // complaint.
    let clear_of_it = plan(&chain, &regions, 3.0, false);

    let cut = surface_face_walkback::try_apply_one(
        &plate,
        &plate.get_mesh_edge_graph(),
        &regions,
        &clear_of_it,
    )
    .unwrap_or_else(|reason| panic!("{reason}"));

    let area = area_on(&cut.mesh, |v| (v.as_vector3().z - 4.0).abs() < 1e-6);
    assert!(
        (area - (400.0 - 64.0 - (20.0 * 3.0))).abs() <= MEASURED,
        "a 20 mm top less the boss's 8 mm footprint, less a 20 by 3 band off the front - {area}"
    );

    assert_eq!(
        count_vertices(&cut.mesh, |v| (v.z - 4.0).abs() < 1e-6
            && (v.x - 6.0).abs() < 1e-6
            && (v.y - 6.0).abs() < 1e-6),
        1,
        "the boss's foot is exactly where it was"
    );

    assert_every_run_is_a_chain_of_real_edges(&cut);
    assert_the_run_vertices_are_on_their_lines(&cut, &clear_of_it);
    assert_no_new_t_junctions(&plate, &cut.mesh);
}

/// The same refusal for a BORE: the mouth of a 10 mm bore through a 20 mm slab. Setting the slab's
/// front rim back by 12 draws the tangent line through the mouth, so the same code comes back,
/// naming the corner of the mouth it reached.
#[test]
fn a_band_that_reaches_the_mouth_of_a_bore_is_refused_the_same_way() {
    let slab = Arc::new(tall_bored_slab());
    let (regions, features) = regions_of(&slab);
    let regions = Arc::new(regions);

    let pick = single(&features, |edge| {
        (edge.start.z - 20.0).abs() < 1e-6
            && (edge.end.z - 20.0).abs() < 1e-6
            && edge.start.y.abs() < 1e-6
            && edge.end.y.abs() < 1e-6
            && (edge.start.x.min(edge.end.x) - 10.0).abs() < 1e-6
    });

    let planned = plan(&chain_of(&features, pick), &regions, 12.0, false);

    let reason = surface_face_walkback::try_apply_one(
        &slab,
        &slab.get_mesh_edge_graph(),
        &regions,
        &planned,
    )
    .expect_err("a 12 mm band from y = 0 runs into the bore's mouth at y = 10");
    assert_eq!(
        reason.code(),
        BevelAdvisoryCode::SurfaceCutMeetsStandingFeature
    );
    assert!(
        (named_point(&reason).y - 10.0).abs() < 1e-6,
        "and names a corner of that mouth - instead {reason}"
    );
}

/// A hole BEHIND the band is no part of it. Two bores 4.5 mm apart, and the band is a 0.4 mm round
/// on the far rim of the first one, walking AWAY from the second: a 0.4 band does not reach it
/// however far the half space it is written as runs.
#[test]
fn a_hole_behind_the_band_is_no_part_of_it() {
    let slab = Arc::new(twice_bored_slab());
    let (regions, features) = regions_of(&slab);
    let regions = Arc::new(regions);
    let chain = chain_of(&features, the_bore_rim_at(&features, 10.0));

    let planned = plan(&chain, &regions, 0.4, true);

    let walked = surface_face_walkback::try_apply_one(
        &slab,
        &slab.get_mesh_edge_graph(),
        &regions,
        &planned,
    )
    .unwrap_or_else(|reason| {
        panic!("a 0.4 band walking away from the far bore cannot reach it - {reason}")
    });

    assert_eq!(
        count_vertices(&walked.mesh, |v| (v.z - 20.0).abs() < 1e-6
            && (v.x - 10.0).abs() < 1e-6
            && (v.y - 20.5).abs() < 1e-6),
        1,
        "and the far bore's mouth is exactly where it was"
    );
}

/// And a hole INSIDE the band is still refused. The band now on the near rim of the first bore,
/// walking towards the second: at a 5 mm setback the tangent line is drawn through the second
/// bore's mouth at `y = 20.5`. At 3 it stops two short of it and the same face cuts without
/// complaint.
#[test]
fn a_hole_inside_the_band_is_still_refused() {
    let slab = Arc::new(twice_bored_slab());
    let (regions, features) = regions_of(&slab);
    let regions = Arc::new(regions);
    let chain = chain_of(&features, the_bore_rim_at(&features, 16.0));

    let into_the_bore = plan(&chain, &regions, 5.0, false);

    let reason = surface_face_walkback::try_apply_one(
        &slab,
        &slab.get_mesh_edge_graph(),
        &regions,
        &into_the_bore,
    )
    .expect_err("a 5 mm band from y = 16 runs into the second bore's mouth at y = 20.5");
    assert_eq!(
        reason.code(),
        BevelAdvisoryCode::SurfaceCutMeetsStandingFeature
    );
    assert!(
        (named_point(&reason).y - 20.5).abs() < 1e-6,
        "and names the mouth it reached - instead {reason}"
    );

    let clear_of_it = plan(&chain, &regions, 3.0, false);

    let cut = surface_face_walkback::try_apply_one(
        &slab,
        &slab.get_mesh_edge_graph(),
        &regions,
        &clear_of_it,
    )
    .unwrap_or_else(|reason| panic!("three stops two short of the second bore - {reason}"));

    assert_eq!(
        count_vertices(&cut.mesh, |v| (v.z - 20.0).abs() < 1e-6
            && (v.x - 10.0).abs() < 1e-6
            && (v.y - 20.5).abs() < 1e-6),
        1,
        "with that bore's mouth exactly where it was"
    );
}

/// The rim edge of the first bore's mouth at `y`, on the slab's top: one whole side of the square
/// mouth, from x = 10 to x = 20.
fn the_bore_rim_at(features: &FeatureEdgeSet, y: f64) -> i32 {
    single(features, |edge| {
        (edge.start.z - 20.0).abs() < 1e-6
            && (edge.end.z - 20.0).abs() < 1e-6
            && (edge.start.y - y).abs() < 1e-6
            && (edge.end.y - y).abs() < 1e-6
            && (edge.start.x.min(edge.end.x) - 10.0).abs() < 1e-6
            && (edge.start.x.max(edge.end.x) - 20.0).abs() < 1e-6
    })
}

/// The refusal. A region with a face missing from what should bound it is one whose neighbours
/// cannot be split to match the cut, so the walkback says so and hands back nothing.
#[test]
fn a_rim_that_does_not_close_is_refused_rather_than_walked() {
    let cube = Arc::new(platonic_solids::create_cube(20.0, 20.0, 20.0));
    let (regions, features) = regions_of(&cube);
    let regions = Arc::new(regions);
    let chain = chain_of(&features, top_front_edge(&features));

    plan(&chain, &regions, 3.0, false);

    // The same cube with one face of the LEFT side gone: the top region's rim edge there now has
    // one face, so the top is no longer rimmed all the way round.
    let dropped = (0..cube.faces.len() as i32)
        .find(|&face| {
            let corners = corners(&cube, face);

            corners.iter().all(|corner| (corner.x + 10.0).abs() < 1e-6)
                && corners
                    .iter()
                    .any(|corner| (corner.z - 10.0).abs() < 1e-6 && (corner.y - 10.0).abs() < 1e-6)
        })
        .expect("a face of the left side at the top back corner");
    let holed = Arc::new(without_face(&cube, dropped));

    let (holed_regions, holed_features) = regions_of(&holed);
    let holed_regions = Arc::new(holed_regions);
    let holed_chain = chain_of(&holed_features, top_front_edge(&holed_features));

    let holed_plan = plan(&holed_chain, &holed_regions, 3.0, false);

    let reason = surface_face_walkback::try_apply_one(
        &holed,
        &holed.get_mesh_edge_graph(),
        &holed_regions,
        &holed_plan,
    )
    .expect_err("the top region's rim runs out into the hole");
    assert_eq!(reason.code(), BevelAdvisoryCode::FaceRegionRimIsPartial);
}

/// The same refusal for a foot edge that crosses the band with BOTH of its own ends outside it -
/// the shape a guard that reads one vertex at a time cannot see. The boss's foot runs from `(5, 6)`
/// to `(25, 6)` in one edge, past both ends of the picked rim edge's band at `x = 8` and `x = 22`,
/// and dips into it in the middle; the guard asks the question of the SEGMENT, and the point it
/// names is the stretch it found, not a corner.
#[test]
fn a_foot_edge_that_crosses_the_band_with_both_its_ends_outside_it_is_refused() {
    let plate = Arc::new(topology_test_meshes::create_plate_with_a_boss_foot_spanning_the_rim());
    let (regions, features) = regions_of(&plate);
    let regions = Arc::new(regions);

    // The middle third of the plate's top front rim: from (8, 0, 20) to (22, 0, 20), which is
    // shorter than the foot edge it is about to undermine.
    let pick = single(&features, |edge| {
        (edge.start.z - 20.0).abs() < 1e-6
            && (edge.end.z - 20.0).abs() < 1e-6
            && edge.start.y.abs() < 1e-6
            && edge.end.y.abs() < 1e-6
            && (edge.start.x.min(edge.end.x) - 8.0).abs() < 1e-6
    });

    // Planned against the CURVE reading as well, which is what tells the plan that the rim carries
    // on past both corners of the picked edge - and so gives its band the two ends that make the
    // removed set a box rather than a half space (SurfaceBandEnd).
    let curve_regions = Arc::new(FaceRegionCache::new(
        plate.clone(),
        Some(plate.get_mesh_edge_graph()),
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    ));

    let planned = SurfaceBevelPlan::try_create_with_curve_regions(
        &chain_of(&features, pick),
        &regions,
        Some(&curve_regions),
        9.0,
        false,
    )
    .unwrap_or_else(|reason| panic!("the edge itself is sound - {reason}"));

    assert_eq!(
        planned.band_ends_of(0, true).len(),
        2,
        "the band has to be bounded at x = 8 and x = 22 for the foot's own corners to fall outside it"
    );

    let reason = surface_face_walkback::try_apply_one(
        &plate,
        &plate.get_mesh_edge_graph(),
        &regions,
        &planned,
    )
    .expect_err("a 9 mm band from y = 0 takes the plate out from under the foot at y = 6");
    assert_eq!(
        reason.code(),
        BevelAdvisoryCode::SurfaceCutMeetsStandingFeature
    );

    let named = named_point(&reason);
    assert!(
        (named.y - 6.0).abs() < 1e-6,
        "the place it names is on the foot edge - instead {reason}"
    );
    assert!(
        named.x > 8.0 - 1e-6 && named.x < 22.0 + 1e-6,
        "and between the band's own two ends, where no corner of the foot stands - instead {reason}"
    );
}

/// That one chain's foot does not excuse ANOTHER chain's band from the guard. Ask for the foot AND
/// a rim edge whose band runs into the same foot, and the rim edge's band is still undermining it:
/// the exemption belongs to the plan whose chain it is, not to the region.
#[test]
fn a_foot_being_bevelled_by_another_chain_does_not_exempt_this_ones_band() {
    let boss = Arc::new(topology_test_meshes::create_square_boss_on_plate());
    let (regions, features) = regions_of(&boss);
    let regions = Arc::new(regions);

    let into_the_boss = plan(&the_front_rim(&features), &regions, 7.0, false);
    let the_bead = plan(&the_foot_loop(&features), &regions, 2.0, false);

    let reason = surface_face_walkback::try_apply(
        &boss,
        &boss.get_mesh_edge_graph(),
        &regions,
        &[into_the_boss, the_bead],
    )
    .expect_err(
        "the rim's band still takes the plate out from under the boss, whatever the foot's own chain is doing",
    );
    assert_eq!(
        reason.code(),
        BevelAdvisoryCode::SurfaceCutMeetsStandingFeature
    );
}

/// The whole front rim of the square boss's plate, at z = 4: three edges, one open chain.
fn the_front_rim(features: &FeatureEdgeSet) -> BevelSweptChain {
    let rim: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .copied()
        .filter(|edge| {
            (edge.start.z - 4.0).abs() < 1e-6
                && (edge.end.z - 4.0).abs() < 1e-6
                && edge.start.y.abs() < 1e-6
                && edge.end.y.abs() < 1e-6
        })
        .collect();

    let mut chains = bevel_chain_linker::link(&rim);
    assert_eq!(chains.len(), 1);
    chains.remove(0)
}

/// The four concave edges where the boss meets the plate, as the one closed loop they make.
fn the_foot_loop(features: &FeatureEdgeSet) -> BevelSweptChain {
    let foot: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .copied()
        .filter(|edge| edge.convexity == EdgeConvexity::Concave)
        .collect();

    let mut chains = bevel_chain_linker::link(&foot);
    assert_eq!(chains.len(), 1);
    chains.remove(0)
}

/// The index of the one feature edge that matches; a test failure if there is not exactly one.
fn single(features: &FeatureEdgeSet, is_it: impl Fn(&FeatureEdge) -> bool) -> i32 {
    let found: Vec<i32> = (0..features.count() as i32)
        .filter(|&i| is_it(&features.edges()[i as usize]))
        .collect();

    assert_eq!(found.len(), 1, "exactly one edge matches");

    found[0]
}
