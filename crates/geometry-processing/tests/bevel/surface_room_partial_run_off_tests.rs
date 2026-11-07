//! The room a band has where the face it lands on DIES
//! AWAY at one end of the edge and stands as usual past that - "UpperLeftInner should bevel.mcx"'s
//! upper left crease at 3 mm, whose left wall is 0.053 mm wide where the crease reaches the
//! recess's mouth and widens at 1.118 mm per millimetre along it. The parameter rows are a loop.
//!
//! The rim is not a wall: the bevel eats the sliver of wall over the first 1.887 mm and runs off the
//! knife edge there onto the cube's left face, which is recorded as a
//! `SurfaceBandRunsOffOverAStretch`. With the sliver given back, the wall the left side really
//! stops at is the recess's LOWER crease, 14.907 mm off at the apex, so the pick fits up to 20.68
//! mm and is refused above that. What the reading will NOT take is read on a slab built for it
//! (`tapered_ledge_test_meshes`): a stretch in the middle of a span and a stretch at an end the
//! chain carries on past are refused by name, and a stretch on a CUT is left to the wall rule.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{
    surface_chain_join, surface_room, surface_room_measure, SurfaceBandRunsOffOverAStretch,
    SurfaceBevelPlan, SurfaceRoomFootprint,
};
use geometry_processing::bevel::{BevelAdvisoryArg, BevelAdvisoryCode};
use geometry_processing::topology::{FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::pyramid_recess_test_meshes;
use crate::tapered_ledge_test_meshes;

/// What the control rounds the recess's creases at, and what the room used to refuse.
const RADIUS: f64 = 3.0;

/// How far along the crease from its rim end the left wall is eaten whole: (2.16228 - 0.05300)
/// / 1.1180, the design's s*.
const ALONG_THE_WALL: f64 = 1.8866;

/// The largest size the crease fits at once the sliver is given back: the lower crease standing
/// 14.9071 off at the apex, over the round's own 0.72076 of setback per unit of size.
const FITS_UP_TO: f64 = 20.6825;

fn fit(
    plans: &[Rc<SurfaceBevelPlan>],
    size: f64,
    stretches: Option<&mut Vec<SurfaceBandRunsOffOverAStretch>>,
) -> surface_room::SurfaceRoomFit {
    surface_room::try_fit(plans, size, &CancelToken::new(), stretches).expect("nothing cancels")
}

/// The run-off is HANDED to the build rather than refused: the room says the size fits, and the
/// stretch it read is the same stretch the builder ends the strip on (`SurfaceStripEndsOnTheHull`)
/// - the same side, the same end and the same knife edge, read off the same plans.
#[test]
fn the_run_off_is_handed_to_the_build() {
    let read = the_crease(RADIUS);
    let mut stretches = Vec::new();

    let handed_fit = fit(&read.2, RADIUS, Some(&mut stretches));
    assert!(
        handed_fit.fits,
        "the size fits and the run-off is the build's to end - instead {:?}",
        handed_fit.refusal.map(|refusal| refusal.code())
    );

    let quiet = fit(&read.2, RADIUS, None);
    assert!(
        quiet.fits,
        "and a caller that wants none back is not refused for it either - instead {:?}",
        quiet.refusal.map(|refusal| refusal.code())
    );

    assert_eq!(stretches.len(), 1);
    let handed = &stretches[0];
    let edge = &read.2[0].edges[0];

    assert_eq!(
        handed.side, 0,
        "the LEFT wall is the one that dies away, which is rail zero's side of the strip"
    );

    assert!(
        ((if handed.at_the_start { edge.from() } else { edge.to() })
            - pyramid_recess_test_meshes::rim_end())
        .length()
            < 1e-6,
        "and the end it is anchored at is the crease's rim end, which is the end the build closes on the hull"
    );
}

/// The whole reading on the fixture: the size fits, and the left wall's stretch is 1.8866 long and
/// runs off the knife edge DA onto the cube's left face.
#[test]
fn the_crease_runs_off_its_wall_over_a_stretch() {
    let read = the_crease(RADIUS);
    let mut stretches = Vec::new();

    let fitted = fit(&read.2, RADIUS, Some(&mut stretches));
    assert!(
        fitted.fits,
        "the wall is eaten and run off rather than stopped at - instead {:?} allowing {}",
        fitted.refusal.as_ref().map(|refusal| refusal.code()),
        fitted.largest_fitting_size
    );

    assert!(
        (fitted.largest_fitting_size - RADIUS).abs() <= 1e-9,
        "at exactly the size asked for: what the rim still bounds past the stretch is the setback itself - instead {}",
        fitted.largest_fitting_size
    );

    assert_eq!(
        stretches.len(),
        1,
        "the left wall's, and only it: the UPPER slope dies away at the same end, but the rim it goes over there MEETS the crease at the mouth, which is the corner rule's reading and not this one - instead {}",
        stretches.len()
    );

    // Which END of the span each stretch stands at is the chain's WALK and nothing about the shape.
    let edge = &read.2[0].edges[0];
    let the_rim_is_the_edges_start =
        (edge.from() - pyramid_recess_test_meshes::rim_end()).length() < 1e-6;

    let walls: Vec<&SurfaceBandRunsOffOverAStretch> = stretches
        .iter()
        .filter(|stretch| stretch.plane_beyond.dot(-Vector3::UNIT_X) > 0.999)
        .collect();
    assert_eq!(walls.len(), 1);
    let wall = walls[0];

    assert_eq!(
        wall.at_the_start, the_rim_is_the_edges_start,
        "it runs off at the crease's RIM end, which is where the recess's left wall dies away"
    );

    assert!(
        (wall.until - ALONG_THE_WALL).abs() <= 1e-3,
        "the wall runs 0.05300 across at the rim and widens at 1.1180, so 2.16228 of setback eats {ALONG_THE_WALL} of it - instead {}",
        wall.until
    );

    // The knife edge DA: the cube's front-left edge, which is the z axis itself.
    let knife = wall.knife_line;
    assert!(
        (knife.direction.dot(Vector3::UNIT_Z).abs() - 1.0).abs() <= 1e-6,
        "the wall runs off the z axis - instead {:?} to {:?}",
        knife.start,
        knife.end
    );

    assert!(
        (knife.start.length() - knife.start.z).max(knife.end.length() - knife.end.z) < 1e-6,
        "and it stands ON that axis - instead {:?} to {:?}",
        knife.start,
        knife.end
    );

    assert!(
        (wall.plane_beyond.dot(-Vector3::UNIT_X) - 1.0).abs() <= 1e-6,
        "with the cube's left face beyond it - instead {:?}",
        wall.plane_beyond
    );

    assert!(
        wall.plane_beyond_at.dot(wall.plane_beyond).abs() < 1e-6,
        "which is the plane x = 0 - instead a point of it at {:?}",
        wall.plane_beyond_at
    );
}

/// And the reading is not a licence: past the wall the crease really has - the recess's lower
/// crease, 14.907 mm off at the apex - the size is refused as it always was, and the size named is
/// one the user can then ask for.
#[test]
fn past_the_wall_it_really_has_the_size_is_still_refused() {
    for size in [21.0, 25.0] {
        let read = the_crease(size);

        let fitted = fit(&read.2, size, None);
        assert!(
            !fitted.fits,
            "{size} is more wall than the recess has, and it allowed {}",
            fitted.largest_fitting_size
        );

        let refusal = fitted.refusal.expect("a fit that fails names why");
        assert_eq!(
            refusal.code(),
            BevelAdvisoryCode::SizeExceedsFaceWidth,
            "named as the width it is - instead {:?}",
            refusal.code()
        );

        assert!(
            (fitted.largest_fitting_size - FITS_UP_TO).abs() <= 1e-3,
            "the lower crease stands 14.9071 off at the apex, over 0.72076 of setback per unit of size - instead {}",
            fitted.largest_fitting_size
        );
    }
}

/// The same reading on the slab built for it: the ledge narrows to its pinch at the far end of the
/// block, and the band laid from the block's foot runs off the ledge's rim over the last 4 mm of
/// the span.
#[test]
fn the_ledge_is_run_off_over_the_stretch_at_its_narrow_end() {
    let read = the_slab(false);
    let mut stretches = Vec::new();

    surface_room_measure::measure(
        &plans_for(&read.1, RADIUS, &the_blocks_foot(&read.2)),
        None,
        Some(&mut stretches),
    );

    assert_eq!(stretches.len(), 1);
    let stretch = &stretches[0];
    let edge = the_blocks_foot(&read.2)[0];
    let narrow_end = Vector3::new(tapered_ledge_test_meshes::length_of(), 0.0, 0.0);

    assert!(
        (stretch.until - 4.0).abs() <= 1e-6,
        "the ledge is 6 wide and loses 0.5 a millimetre, so 3 of setback eats the last 4 mm of it - instead {}",
        stretch.until
    );

    assert_eq!(
        stretch.at_the_start,
        (edge.start - narrow_end).length() < 1e-6,
        "and the stretch stands at the end the ledge is narrow at, whichever way the edge was linked"
    );

    assert!(
        (stretch
            .plane_beyond
            .dot(Vector3::new(tapered_ledge_test_meshes::TAPER_RATE, -1.0, 0.0).get_normal())
            - 1.0)
            .abs()
            <= 1e-6,
        "and what is beyond the ledge's rim is the plate's own tapering side - instead {:?}",
        stretch.plane_beyond
    );
}

/// And the rim it runs off goes on bounding the REST of the span: the charge is dropped over the
/// stretch and over nothing else, so the reading comes to the size that was asked for rather than
/// to a licence.
#[test]
fn the_rim_goes_on_bounding_the_rest_of_the_span() {
    let read = the_crease(RADIUS);
    let edge = &read.2[0].edges[0];
    let setback = edge.left_setback;

    let walls: Vec<_> = surface_room_measure::measure(&read.2, None, None)
        .into_iter()
        .filter(|cut| {
            cut.shape == SurfaceRoomFootprint::Band && cut.inward.dot(-Vector3::UNIT_X) > 0.0
        })
        .collect();
    assert_eq!(walls.len(), 1);
    let wall = &walls[0];

    assert!(
        (wall.room - setback).abs() <= 1e-6,
        "the wall is a setback across where the stretch ends, and that is what it still charges - instead {}",
        wall.room
    );

    assert!(
        wall.room < 14.9,
        "and not the 14.907 of the lower crease at the apex, which is what dropping the whole charge would have left"
    );
}

/// A rim standing EXACTLY at the setback is a rim the band reaches and does not pass, so nothing
/// runs off it: the ledge here is level at 3 mm over the first half of the block and widens after
/// it, and a 3 mm round sets back exactly 3.
#[test]
fn a_rim_standing_exactly_at_the_setback_is_still_a_wall() {
    let part = tapered_ledge_test_meshes::create_wall_of_widths(&[
        (0.0, RADIUS),
        (5.0, RADIUS),
        (10.0, RADIUS + 3.0),
    ]);
    let regions = regions_of(&part);
    let features = FeatureEdgeSet::create(&part);
    let plans = plans_for(&regions, RADIUS, &the_blocks_foot(&features));
    let mut stretches = Vec::new();

    surface_room_measure::measure(&plans, None, Some(&mut stretches));

    assert_eq!(
        stretches.len(),
        0,
        "3 of setback reaches the 3 mm ledge and stops there - instead it read {} run-offs",
        stretches.len()
    );

    let fitted = fit(&plans, RADIUS, None);
    assert!(
        fitted.fits,
        "and the rim still bounds it, at exactly the size asked for - instead {:?}",
        fitted.refusal.as_ref().map(|refusal| refusal.code())
    );

    assert!(
        (fitted.largest_fitting_size - RADIUS).abs() <= 1e-6,
        "which is the level ledge's own width - instead {}",
        fitted.largest_fitting_size
    );
}

/// And nothing about a CUT is named, whatever shape its landing dies away in: the block's top
/// pinched in the middle is a mid-span run-off on a convex edge, and the answer is the size that
/// keeps the chamfer on the top rather than a name.
#[test]
fn a_cut_is_not_named_even_where_its_landing_dies_away_mid_span() {
    let read = the_slab(true);
    let plans = plans_for(&read.1, RADIUS, &the_blocks_top_edge(&read.2));

    let fitted = fit(&plans, RADIUS, None);
    assert!(!fitted.fits);

    let refusal = fitted.refusal.expect("a fit that fails names why");
    assert_eq!(
        refusal.code(),
        BevelAdvisoryCode::SizeExceedsFaceWidth,
        "the convexity is read before anything is named - instead {:?}",
        refusal.code()
    );

    assert!(
        (fitted.largest_fitting_size - tapered_ledge_test_meshes::PINCH).abs() <= 1e-6,
        "and the size is the pinch the top narrows to - instead {}",
        fitted.largest_fitting_size
    );
}

/// A stretch in the MIDDLE of the span is refused by name. The same slab pinched in the middle
/// instead leaves ledge standing at both ends of the block, so the strip would have to leave the
/// part and land on it again over one edge - which is not one trim at any size.
#[test]
fn a_stretch_in_the_middle_of_the_span_is_refused_by_name() {
    let read = the_slab(true);
    let plans = plans_for(&read.1, RADIUS, &the_blocks_foot(&read.2));

    let fitted = fit(&plans, RADIUS, None);
    assert!(!fitted.fits);

    let refusal = fitted.refusal.expect("a fit that fails names why");
    assert_eq!(
        refusal.code(),
        BevelAdvisoryCode::SurfaceBandRunsOffMidSpan,
        "the ledge dies in the middle and stands at both ends - instead {:?}",
        refusal.code()
    );

    let middle = Vector3::new(
        tapered_ledge_test_meshes::length_of_with(
            tapered_ledge_test_meshes::WIDTH_AT_THE_RIM,
            tapered_ledge_test_meshes::TAPER_RATE,
            tapered_ledge_test_meshes::PINCH,
            true,
        ) / 2.0,
        0.0,
        0.0,
    );
    assert_eq!(
        refusal.args()[0],
        BevelAdvisoryArg::Vector3(middle),
        "named at the middle it runs off at - instead {:?}",
        refusal.args()[0]
    );
}

/// A CUT runs off nothing, and is left to the wall rule exactly as it always was: a chamfer takes
/// material from the outside and what lies past its landing is a shape of its own, so the rim goes
/// on being charged as the wall it is and the reading answers with a SIZE rather than a name.
#[test]
fn a_cut_runs_off_nothing_and_is_left_to_the_wall_rule() {
    let read = the_slab(false);
    let plans = plans_for(&read.1, RADIUS, &the_blocks_top_edge(&read.2));
    let mut stretches = Vec::new();

    let fitted = fit(&plans, RADIUS, Some(&mut stretches));
    assert!(!fitted.fits);

    assert_eq!(
        stretches.len(),
        0,
        "nothing runs off a cut, so the build is told of nothing"
    );

    let refusal = fitted.refusal.expect("a fit that fails names why");
    assert_eq!(
        refusal.code(),
        BevelAdvisoryCode::SizeExceedsFaceWidth,
        "and the answer is the size that keeps the chamfer on the block's top - instead {:?}",
        refusal.code()
    );

    assert!(
        (fitted.largest_fitting_size - tapered_ledge_test_meshes::PINCH).abs() <= 1e-6,
        "which is the pinch, where the top is narrowest - instead {}",
        fitted.largest_fitting_size
    );
}

/// And a stretch at an end the chain carries ON past is refused by name: picked with the ledge's
/// own end edge, the foot crease's chain turns the corner at the very place the run-off would end
/// its strip, and the two cannot both end it.
#[test]
fn a_run_off_at_a_mitred_end_is_refused_by_name() {
    let read = the_slab(false);
    let mut picked = the_blocks_foot(&read.2);

    picked.push(the_ledges_end_edge(&read.2));

    let plans = plans_for(&read.1, RADIUS, &picked);

    assert_eq!(
        plans[0].edges.len(),
        2,
        "the two are one chain, which is what carries it on past the corner between them"
    );

    let fitted = fit(&plans, RADIUS, None);
    assert!(!fitted.fits);

    let refusal = fitted.refusal.expect("a fit that fails names why");
    assert_eq!(
        refusal.code(),
        BevelAdvisoryCode::SurfaceRunOffMeetsAMitredEnd,
        "the stretch is anchored at the end the chain turns at - instead {:?}",
        refusal.code()
    );
}

fn regions_of(part: &Mesh) -> Arc<FaceRegionCache> {
    Arc::new(FaceRegionCache::new(
        Arc::new(part.clone()),
        Some(part.get_mesh_edge_graph()),
        FeatureEdge::default_flat_angle_degrees(),
    ))
}

/// The recess's upper left crease, from the mouth to the apex, planned alone at one size.
fn the_crease(size: f64) -> (Mesh, Arc<FaceRegionCache>, Vec<Rc<SurfaceBevelPlan>>) {
    let part =
        pyramid_recess_test_meshes::create_from_the_fixtures_own_triangles(Vector3::ZERO, false);
    let regions = regions_of(&part);
    let features = FeatureEdgeSet::create(&part);

    let rim_end = pyramid_recess_test_meshes::rim_end();
    let apex_end = pyramid_recess_test_meshes::apex_end();
    let crease = single(&features, |edge| {
        ((edge.start - rim_end).length() < 1e-3 && (edge.end - apex_end).length() < 1e-3)
            || ((edge.end - rim_end).length() < 1e-3 && (edge.start - apex_end).length() < 1e-3)
    });

    let plans = plans_for(&regions, size, &[crease]);
    (part, regions, plans)
}

/// The slab built for these readings, either way its ledge narrows.
fn the_slab(pinched_in_the_middle: bool) -> (Mesh, Arc<FaceRegionCache>, FeatureEdgeSet) {
    let part =
        tapered_ledge_test_meshes::create_wall_tapering_to_a_knife_edge(pinched_in_the_middle);
    let regions = regions_of(&part);
    let features = FeatureEdgeSet::create(&part);

    (part, regions, features)
}

/// Where the block meets the ledge: the concave crease along y = 0 at the plate's own top.
fn the_blocks_foot(features: &FeatureEdgeSet) -> Vec<FeatureEdge> {
    vec![single(features, |edge| {
        along(edge, 0.0) && edge.start.z.abs() < 1e-9 && edge.end.z.abs() < 1e-9
    })]
}

/// And the convex edge straight above it, along the near side of the block's top.
fn the_blocks_top_edge(features: &FeatureEdgeSet) -> Vec<FeatureEdge> {
    vec![single(features, |edge| {
        along(edge, tapered_ledge_test_meshes::BLOCK_HEIGHT)
    })]
}

/// The ledge's own edge at the narrow END of the block, which the foot crease turns the corner
/// into: the rim of the plate's top where the part ends, running out of the crease's own corner.
fn the_ledges_end_edge(features: &FeatureEdgeSet) -> FeatureEdge {
    let length = tapered_ledge_test_meshes::length_of();

    single(features, |edge| {
        (edge.start.x - length).abs() < 1e-9
            && (edge.end.x - length).abs() < 1e-9
            && edge.start.z.abs() < 1e-9
            && edge.end.z.abs() < 1e-9
            && edge.start.y.min(edge.end.y) < 0.0
    })
}

/// Whether an edge runs along the block's near side, y = 0, at one height.
fn along(edge: &FeatureEdge, z: f64) -> bool {
    edge.start.y.abs() < 1e-9
        && edge.end.y.abs() < 1e-9
        && (edge.start.z - z).abs() < 1e-9
        && (edge.end.z - z).abs() < 1e-9
}

/// The one feature edge that matches; fails on none or several.
fn single(features: &FeatureEdgeSet, is_it: impl Fn(&FeatureEdge) -> bool) -> FeatureEdge {
    let found: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| is_it(edge))
        .copied()
        .collect();
    assert_eq!(found.len(), 1, "exactly one feature edge matches");
    found[0]
}

/// Each picked edge joined into chains and planned at one size.
fn plans_for(
    regions: &Arc<FaceRegionCache>,
    size: f64,
    picked: &[FeatureEdge],
) -> Vec<Rc<SurfaceBevelPlan>> {
    surface_chain_join::join(&bevel_chain_linker::link(picked))
        .iter()
        .map(|chain| {
            SurfaceBevelPlan::try_create(chain, regions, size, true).unwrap_or_else(|reason| {
                panic!("the pick has to plan at {size}: {:?}", reason.code())
            })
        })
        .collect()
}
