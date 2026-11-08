//! The room a mixed corner's RUN-OUT takes on the cap face,
//! negotiated against another chain's band - the lobed prism's barrel picked together with the flat
//! side 10.228 mm away from the seam, which is "Every Chamfer Works.mcx"'s wall and its cube side
//! rim (`lobed_prism_meshes`).
//!
//! The barrel's rim turns a mixed corner at each crossing of the flat side, and past the end of the
//! bead's rim the ball goes on touching the CAP: at `r = 3.756` that skirt reaches 9.206 mm across
//! the bottom cap from the corner. With the far side picked too, the piece of cap the skirt lands
//! on is the piece that side's own band has already taken. No EDGE owns that skirt, so a pair rule
//! that enumerates edge cuts never sees it; `surface_room_run_out` is the missing cut. The region
//! readers are the ones `surface_room_end_cut_tests.rs` shares.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, surface_mixed_corner_run_out, surface_room,
    surface_room_measure, surface_room_run_out, SurfaceBevelPlan, SurfaceBuildFailure,
    SurfaceBuildOptions, SurfaceMixedCorner, SurfaceMixedCornerWall, SurfaceRoomCrossing,
    SurfaceRoomCut, SurfaceRoomFootprint, SurfaceVertexPool,
};
use geometry_processing::bevel::BevelAdvisoryCode;
use geometry_processing::topology::{FaceGroup, FaceRegionCache, FeatureEdge};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::lobed_prism_meshes;
use crate::mesh_measurements::shell_count;
use crate::surface_room_end_cut_tests::{
    middle_of, normal_of, plans_for, region_at, regions_of, single, the_barrel, the_bottom_cap,
    the_far_side,
};

/// What "Every Chamfer Works.mcx" is filleted at, and what the shell opened at.
const RADIUS: f64 = 3.756;

/// How far the cap runs between the seam and the far side, which is the gap the skirt and that
/// side's band share: the lobe crosses the flat side at 10.228 and the side stands at x = 0.
const ACROSS_THE_CAP: f64 = 10.228;

/// The corner the skirt this fixture is about runs out from: the left crossing, on the bottom cap.
fn the_corner() -> Vector3 {
    Vector3::new(lobed_prism_meshes::left_crossing().x, 0.0, 0.0)
}

fn assert_within(actual: f64, expected: f64, tolerance: f64, because: &str) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} is not {expected} within {tolerance}: {because}"
    );
}

/// The run-out reads as a cut on the CAP, from the corner, along the way the ball's touch point
/// travels - and it reaches exactly as far as the trim curve's own last station does, which is what
/// says this is the construction's own footprint rather than a second estimate of it.
#[test]
fn the_run_out_is_a_cut_on_the_cap_reaching_where_its_trim_curve_dies() {
    let (prism, _, plan) = the_barrel_rim(RADIUS);
    let corner = the_corner_at_the_bottom_of_the_seam(&plan);

    let skirt = surface_room_run_out::try_cut(&corner, SurfaceRoomCut::NO_CORNER)
        .expect("the run-out is a cut");

    assert_eq!(
        skirt.region_key,
        SurfaceBevelPlan::key_of(&the_bottom_cap(&prism)),
        "the skirt is laid on the cap - the cut edge's region that is not the one the two edges share"
    );

    assert_within(
        skirt.inward.dot(-Vector3::UNIT_X),
        1.0,
        1e-6,
        &format!(
            "the touch point travels along the flat side away from the lobe, and it read {:?}",
            skirt.inward
        ),
    );

    assert!(
        (skirt.from - corner.corner).dot(skirt.inward).abs() < 1e-6,
        "the corner is the origin the reach is measured from"
    );

    assert!(
        (corner.corner - the_corner()).length() < 1e-3,
        "and that corner is the bottom of the seam - instead {:?}",
        corner.corner
    );

    assert_within(
        skirt.setback_per_size * RADIUS,
        reach_of(&prism, &corner),
        1e-6,
        &format!(
            "the cut reaches the run-out's own last rail point on the cap - instead {}",
            skirt.setback_per_size * RADIUS
        ),
    );

    assert_within(
        skirt.setback_per_size * RADIUS,
        9.206,
        1e-3,
        "which on this corner at this size is 9.206 mm out across the cap",
    );

    assert!(
        skirt.room == f64::INFINITY,
        "a run-out is allowed to reach past the cap's own rim; only another chain's band stops it"
    );
}

/// And the run-out is a cut on the WALL it turns against as well: the scallop it bites out of that
/// face between the sharp edge and its own trim curve. It reaches the same 9.206 along the wall
/// that it does across the cap, and it is 3.756 deep, because the bite starts at the sharp edge and
/// ends where the ball has come a whole radius under the cap.
#[test]
fn the_run_out_is_also_a_cut_on_the_wall_it_turns_against() {
    let (prism, _, plan) = the_barrel_rim(RADIUS);
    let corner = the_corner_at_the_bottom_of_the_seam(&plan);

    let bite = surface_room_run_out::try_wall_cut(&corner, SurfaceRoomCut::NO_CORNER)
        .expect("the run-out is a cut on the wall");

    assert_eq!(
        bite.shape,
        SurfaceRoomFootprint::RunOutSkirt,
        "it is the same footprint read on another face, and the pair rule reads it the same way"
    );

    assert_eq!(
        bite.region_key,
        SurfaceBevelPlan::key_of(&the_flat_wall(&prism)),
        "the bite is on the wall nobody picked - the bead's region that is not the one the two edges share"
    );

    assert_ne!(
        bite.region_key,
        SurfaceBevelPlan::key_of(&the_bottom_cap(&prism)),
        "which is not the face the skirt is laid on"
    );

    assert_within(
        bite.inward.dot(-Vector3::UNIT_X),
        1.0,
        1e-6,
        &format!(
            "the bite travels along the wall away from the lobe, and it read {:?}",
            bite.inward
        ),
    );

    assert_within(
        bite.along.dot(Vector3::UNIT_Z),
        1.0,
        1e-6,
        &format!(
            "and its width runs DOWN the wall from the sharp edge - instead {:?}",
            bite.along
        ),
    );

    assert!(
        (bite.from - corner.corner).length() < 1e-9,
        "the corner is the origin the reach is measured from"
    );

    assert_within(
        bite.length,
        RADIUS,
        1e-6,
        &format!(
            "the trim curve starts a whole radius under the cap, at the bead's own axis - instead {}",
            bite.length
        ),
    );

    assert_within(
        bite.setback_per_size * RADIUS,
        9.206,
        1e-3,
        &format!(
            "and it reaches 9.206 along the wall at this size - instead {}",
            bite.setback_per_size * RADIUS
        ),
    );

    assert!(
        bite.room == f64::INFINITY,
        "a run-out is bounded by another chain's footprint and by nothing else"
    );

    // Read at the ENDS of the turn and at every crease between, which is exact: along one facet
    // the trim point's own reading is monotone in the turn. Pinned against the curve itself.
    let wall = SurfaceMixedCornerWall::try_read(&corner)
        .unwrap_or_else(|why| panic!("the wall has to read: {:?}", why.code()));

    for station in 0..=200 {
        let past = surface_mixed_corner_run_out::clears_the_wall() * f64::from(station) / 200.0;

        if let Some(at) = surface_mixed_corner_run_out::try_scallop_point(&corner, &wall, past) {
            let along_the_wall = (at - corner.corner).dot(bite.inward);
            assert!(
                along_the_wall <= (bite.setback_per_size * RADIUS) + 1e-9,
                "the trim point at {past} stands {along_the_wall} along the wall"
            );

            let down_it = (at - corner.corner).dot(bite.along);
            assert!(down_it <= bite.length + 1e-9, "and {down_it} down it");
        }
    }
}

/// And the reach is the FURTHEST the rail gets along the way the skirt travels, over the whole turn
/// the run-out covers - not simply where it ends. On this corner the two are the same to a
/// millionth, so the property is pinned instead of the difference: no station of the run-out's own
/// trim curve stands further along the travel than the cut says it reaches.
#[test]
fn the_reach_is_the_furthest_the_rail_gets_along_the_travel_and_not_merely_its_end() {
    let (prism, _, plan) = the_barrel_rim(RADIUS);
    let corner = the_corner_at_the_bottom_of_the_seam(&plan);

    let skirt = surface_room_run_out::try_cut(&corner, SurfaceRoomCut::NO_CORNER)
        .expect("the run-out is a cut");

    let reach = skirt.setback_per_size * RADIUS;

    assert_within(
        reach,
        reach_of(&prism, &corner),
        1e-6,
        &format!(
            "this rail turns away from the corner all the way, so its furthest station is its last - instead {reach}"
        ),
    );

    for turn in turns_of(&prism, &corner) {
        let away = corner.outer_rail_point(turn) - corner.corner;

        assert!(
            away.dot(skirt.inward) <= reach + 1e-9,
            "the station at {turn} stands {} along the travel, and the cut says it reaches {reach}",
            away.dot(skirt.inward)
        );
    }
}

/// So the barrel picked ALONE still fits at the size it always did: the skirt has nothing to
/// negotiate with, and a cut with infinite room constrains nothing.
#[test]
fn the_barrel_picked_alone_is_unaffected() {
    let (_, _, plan) = the_barrel_rim(RADIUS);

    let fit = fit_of(&[plan], RADIUS);

    assert!(
        fit.fits,
        "nothing else has taken the cap, so 3.756 fits as it always did - {:?}, which allowed {}",
        fit.refusal.as_ref().map(|reason| reason.code()),
        fit.largest_fitting_size
    );
}

/// With the far side picked too the two footprints land on one another, and that is a CROSSING and
/// not a refusal: the fit accepts the size and hands the build the pair to sew - which band, which
/// skirt, and how far the two reach into each other.
#[test]
fn the_skirt_and_the_far_sides_band_cross_on_the_cap_rather_than_refusing_the_size() {
    let plans = both_rims_alone(RADIUS);

    let fit = fit_of(&plans, RADIUS);
    assert!(
        fit.fits,
        "9.206 of skirt over 3.756 of band is a pair to sew, not a size to refuse - {:?}, which allowed {}",
        fit.refusal.as_ref().map(|reason| reason.code()),
        fit.largest_fitting_size
    );
    let crossings = &fit.crossings;

    // Four: the barrel's rim turns one of these corners on each cap, and the far side is picked as
    // a whole FACE - so its bottom rim's band runs under both skirts on the caps, and the band its
    // vertical edge lays on the flat wall runs under both run-outs' bites of that wall as well.
    assert_eq!(
        crossings.len(),
        4,
        "one on each cap and one on the wall at each end of it - instead {}",
        crossings.len()
    );

    let cuts = surface_room_measure::measure(&plans, None, None);
    let skirt = the_skirt_across_the_bottom_cap(&cuts);
    let found: Vec<&SurfaceRoomCrossing> = crossings
        .iter()
        .filter(|sewn| sewn.region_key == skirt.region_key)
        .collect();
    assert_eq!(found.len(), 1, "one crossing on the bottom cap");
    let crossing = *found[0];

    assert_eq!(crossing.skirt.shape, SurfaceRoomFootprint::RunOutSkirt);
    assert_eq!(crossing.band.shape, SurfaceRoomFootprint::Band);

    assert_ne!(
        crossing.band.plan_index, crossing.skirt.plan_index,
        "the band belongs to the far side's own chain, which is the other plan of the pass"
    );

    assert!(
        crossing.band.inward.dot(Vector3::UNIT_X) > 0.999,
        "and to the side of that chain's edge facing the lobe - it read {:?}",
        crossing.band.inward
    );

    assert!(
        crossing.band.from.x.abs().max(crossing.band.to.x.abs()) < 1e-6,
        "the band's edge is the far side's rim on this cap - instead {:?} to {:?}",
        crossing.band.from,
        crossing.band.to
    );

    // And the two of them by NAME, which is what the builder has to trim by: it holds its corners
    // by chain and key and its strips by chain, edge and side.
    assert_ne!(
        crossing.skirt_corner_key(),
        SurfaceRoomCut::NO_CORNER,
        "the skirt says which corner laid it"
    );

    let named = plans[crossing.skirt_plan_index() as usize]
        .try_mixed_corner_after(crossing.skirt_corner_key())
        .unwrap_or_else(|| {
            panic!(
                "and that corner is one of its own chain's - key {} on plan {}",
                crossing.skirt_corner_key(),
                crossing.skirt_plan_index()
            )
        });

    assert!(
        (named.corner - the_corner()).length() < 1e-3,
        "and it is the corner at the bottom of the seam - instead {:?}",
        named.corner
    );

    assert!(
        crossing.band_edge_index() >= 0,
        "the band says which edge of which chain it was taken off"
    );

    let edge =
        &plans[crossing.band_plan_index() as usize].edges[crossing.band_edge_index() as usize];

    assert!(
        (edge.from() - crossing.band.from).length() + (edge.to() - crossing.band.to).length()
            < 1e-6,
        "and that edge is the one the band was measured on - instead {:?} to {:?}",
        edge.from(),
        edge.to()
    );

    assert!(crossing.band_side() == 0 || crossing.band_side() == 1);

    let side = if crossing.band_side() == 0 {
        edge.left_inward_at_the_edge
    } else {
        edge.right_inward_at_the_edge
    };

    assert!(
        side.get_normal().dot(crossing.band.inward) > 0.999,
        "and the side named is the one facing the lobe - instead {side:?}"
    );

    // To a thousandth, which is how near this fixture reads its own corners: the prism's vertices
    // are floats and the lobe's crossing is an intersection of a chord with a side.
    assert!(
        (the_corner() - crossing.skirt.from)
            .dot(crossing.skirt.inward)
            .abs()
            < 1e-3,
        "the corner the skirt runs out of is on the line the crossing names it by"
    );

    let within = (the_corner() - crossing.skirt.from).dot(crossing.skirt.along);
    assert!(
        (0.0..=crossing.skirt.length).contains(&within),
        "and stands within the skirt's own width - instead {:?} to {:?}",
        crossing.skirt.from,
        crossing.skirt.to
    );

    assert_within(
        crossing.gap,
        ACROSS_THE_CAP,
        1e-3,
        &format!(
            "the cap runs {ACROSS_THE_CAP} from the corner to the far side - instead {}",
            crossing.gap
        ),
    );

    assert_within(
        crossing.overlap,
        2.734,
        1e-3,
        &format!(
            "3.756 * (2.4509 + 1) - 10.228 is how far the two reach into each other - instead {}",
            crossing.overlap
        ),
    );

    assert_within(
        crossing.overlap,
        (RADIUS * (crossing.band.setback_per_size + crossing.skirt.setback_per_size))
            - crossing.gap,
        1e-9,
        "which is exactly what the old pair rule refused the size over",
    );
}

/// And a crossing on one face does not answer for the same corner's footprint on ANOTHER. One
/// mixed corner lays two of them - the skirt across the cap and the bite out of the wall it turns
/// against - and each crosses whatever stands under IT.
#[test]
fn a_crossing_on_one_face_does_not_answer_for_the_same_corners_footprint_on_another() {
    let plans = both_rims_alone(RADIUS);

    let fit = fit_of(&plans, RADIUS);
    assert!(
        fit.fits,
        "{:?}",
        fit.refusal.as_ref().map(|reason| reason.code())
    );

    let cuts = surface_room_measure::measure(&plans, None, None);
    let skirt = the_skirt_across_the_bottom_cap(&cuts);
    let band = the_far_sides_band_on(&cuts, &skirt);

    // The same corner's other footprint: its bite out of the flat wall, which stands on a region
    // of its own and crosses that cap band nowhere at all.
    let bite = single(&cuts, |cut| {
        cut.shape == SurfaceRoomFootprint::RunOutSkirt
            && cut.plan_index == skirt.plan_index
            && cut.corner_key == skirt.corner_key
            && cut.region_key != skirt.region_key
    });

    assert!(
        SurfaceRoomCrossing::it_is_sewn(Some(&fit.crossings), &skirt, &band),
        "the skirt on the cap really does cross that band on the cap"
    );

    assert!(
        !SurfaceRoomCrossing::it_is_sewn(Some(&fit.crossings), &bite, &band),
        "but the bite out of the wall crosses it nowhere, whatever the corner they share"
    );
}

/// And only a skirt over a band is sewn. The same two footprints read as two BANDS facing each
/// other across the cap are the collision they always were, because two strips have no envelope
/// to take - each one's far rim is the other's edge and neither may reach past its share.
#[test]
fn two_bands_facing_each_other_across_the_cap_are_still_a_collision() {
    let cuts = surface_room_measure::measure(&both_rims_alone(RADIUS), None, None);
    let skirt = the_skirt_across_the_bottom_cap(&cuts);
    let band = the_far_sides_band_on(&cuts, &skirt);

    assert!(
        SurfaceRoomCrossing::try_cross(&band, &skirt, RADIUS).is_some(),
        "the pair as it really stands is a crossing"
    );

    let as_a_band = SurfaceRoomCut {
        shape: SurfaceRoomFootprint::Band,
        plan_index: skirt.plan_index,
        ..SurfaceRoomCut::new(
            skirt.region_key,
            skirt.from,
            skirt.to,
            skirt.along,
            skirt.inward,
            skirt.length,
            skirt.setback_per_size,
            skirt.room,
        )
    };

    assert!(
        SurfaceRoomCrossing::try_cross(&band, &as_a_band, RADIUS).is_none(),
        "the very same geometry, laid by a band instead of a run-out, is two strips running into each other"
    );

    let (largest, _, collided) = surface_room::largest_fitting(
        &[band, as_a_band],
        RADIUS,
        &CancelToken::new(),
        RADIUS,
        None,
        None,
    )
    .expect("nothing cancels the fit");

    assert_within(
        largest,
        ACROSS_THE_CAP / (band.setback_per_size + skirt.setback_per_size),
        1e-6,
        "so the pair rule still shares the gap between them",
    );

    assert!(collided);
}

/// And a BAND that reaches past the corner the skirt runs out of is a collision too: a band set
/// back past the corner has taken the very place the run-out stands on, so there is nothing left
/// there to trim. At 3.756 a rate of 3 carries the band 11.27 across a cap that runs 10.228.
#[test]
fn a_band_that_swallows_the_corner_is_not_a_crossing() {
    let cuts = surface_room_measure::measure(&both_rims_alone(RADIUS), None, None);
    let skirt = the_skirt_across_the_bottom_cap(&cuts);
    let band = the_far_sides_band_on(&cuts, &skirt);

    assert!(
        SurfaceRoomCrossing::try_cross(&band, &skirt, RADIUS).is_some(),
        "the band as it really stands leaves the corner in front of it"
    );

    let reaching_past_the_corner = SurfaceRoomCut {
        shape: SurfaceRoomFootprint::Band,
        plan_index: band.plan_index,
        edge_index: band.edge_index,
        side: band.side,
        ..SurfaceRoomCut::new(
            band.region_key,
            band.from,
            band.to,
            band.along,
            band.inward,
            band.length,
            3.0,
            band.room,
        )
    };

    assert!(
        reaching_past_the_corner.setback_per_size * RADIUS > ACROSS_THE_CAP,
        "which is the band standing where the corner is"
    );

    assert!(
        SurfaceRoomCrossing::try_cross(&reaching_past_the_corner, &skirt, RADIUS).is_none(),
        "so there is no ground under the run-out to trim against, and it stays a refusal"
    );
}

/// And a skirt whose ideal reach OVERRUNS the face is a crossing as well, not a collision. At
/// `r = 4.2` the torus would come 10.271 mm across a cap 10.228 mm wide - but the face ends it
/// there, and the band's foot line stands 6.028 mm from the corner, well inside what the run-out
/// really covers. So the fit accepts, reports the pairs, and the build sews all four into one
/// closed manifold shell.
#[test]
fn a_skirt_that_overruns_the_cap_is_a_crossing_and_the_build_sews_it() {
    const OVERRUNNING: f64 = 4.2;

    let prism = lobed_prism_meshes::create();
    let regions = regions_of(&prism);

    let cuts = surface_room_measure::measure(&both_rims(&prism, &regions, OVERRUNNING), None, None);
    let skirt = the_skirt_across_the_bottom_cap(&cuts);

    assert!(
        skirt.setback_per_size * OVERRUNNING > ACROSS_THE_CAP,
        "the skirt has to overrun the cap for this to be the case it is about - it reached {}",
        skirt.setback_per_size * OVERRUNNING
    );

    let fit = fit_of(&both_rims(&prism, &regions, OVERRUNNING), OVERRUNNING);
    assert!(
        fit.fits,
        "the run-out is trimmed at the cap's own edge and still crosses the strip in front of it - {:?}",
        fit.refusal.as_ref().map(|reason| reason.code())
    );

    assert_eq!(
        fit.crossings.len(),
        4,
        "one on each cap, and one on the wall at each end of the vertical edge's own band"
    );

    let built = build_with(
        &prism,
        &regions,
        &both_rims(&prism, &regions, OVERRUNNING),
        Some(&fit.crossings),
    )
    .unwrap_or_else(|reason| panic!("and all four are sewn - {reason:?}"));

    let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
    assert_eq!(open, 0, "closed, instead open at {open_at:?}");
    assert!(built.is_manifold());
    assert_eq!(shell_count(&built), 1);
}

/// Under the crossing nothing has changed: at 2.95 the skirt and the band both fit in the cap with
/// room between them, and the pair builds one closed manifold shell as it always did.
#[test]
fn under_the_crossing_the_two_still_fit_in_the_cap_and_close() {
    let prism = lobed_prism_meshes::create();
    let regions = regions_of(&prism);
    let plans = both_rims(&prism, &regions, 2.95);

    let fit = fit_of(&plans, 2.95);
    assert!(
        fit.fits,
        "2.95 of skirt and band share the 10.228 with room to spare - {:?}",
        fit.refusal.as_ref().map(|reason| reason.code())
    );

    assert_eq!(
        fit.crossings.len(),
        0,
        "and they do not reach into each other at all, so there is nothing to sew"
    );

    let built = build_with(&prism, &regions, &plans, None)
        .unwrap_or_else(|reason| panic!("and the build has nothing to refuse - {reason:?}"));

    let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
    assert_eq!(open, 0, "closed, instead open at {open_at:?}");
    assert!(built.is_manifold());
    assert_eq!(shell_count(&built), 1);
}

/// And over it the fit accepts a size the build still refuses by name: a hair above 2.95 the two
/// footprints reach into each other and the pairs are reported as crossings, and at 2.96 the band
/// on the WALL reaches exactly ONE of its own rails into the run-out - the one rail hairpin
/// `SurfaceRunOutMeetsAWallBand` refuses before the hand-over is ever asked for.
#[test]
fn over_the_crossing_the_fit_accepts_and_the_one_rail_hairpin_is_refused_by_name() {
    let prism = lobed_prism_meshes::create();
    let regions = regions_of(&prism);
    let plans = both_rims(&prism, &regions, 2.96);

    let fit = fit_of(&plans, 2.96);
    assert!(
        fit.fits,
        "the skirt over the band is a pair to sew - {:?}",
        fit.refusal.as_ref().map(|reason| reason.code())
    );

    assert_eq!(
        fit.crossings.len(),
        4,
        "one on each cap, and one on the wall at each end of the vertical edge's own band"
    );

    let refused = build_with(&prism, &regions, &plans, Some(&fit.crossings));

    let Err(SurfaceBuildFailure::Refused(reason)) = refused else {
        panic!(
            "at this size the band on the wall reaches one rail into the run-out and no more - {:?}",
            refused.map(|_| "built")
        );
    };

    assert_eq!(
        reason.code(),
        BevelAdvisoryCode::SurfaceCrossingAtTheRunOutsDeath,
        "and what the two share there is a point, which is named rather than left as an open shell"
    );
}

/// The fit at `size`, which nothing cancels.
fn fit_of(plans: &[Rc<SurfaceBevelPlan>], size: f64) -> surface_room::SurfaceRoomFit {
    surface_room::try_fit(plans, size, &CancelToken::new(), None).expect("nothing cancels the fit")
}

/// The pass built at 8 segments, handed the fit's crossings where there are any.
fn build_with(
    prism: &Mesh,
    regions: &FaceRegionCache,
    plans: &[Rc<SurfaceBevelPlan>],
    crossings: Option<&[SurfaceRoomCrossing]>,
) -> Result<Mesh, SurfaceBuildFailure> {
    surface_bevel_builder::try_build(
        prism,
        &prism.get_mesh_edge_graph(),
        regions,
        plans,
        8,
        &CancelToken::new(),
        SurfaceBuildOptions {
            crossings,
            ..crate::surface_test_build::build_options()
        },
    )
    .map(|built| built.mesh)
}

/// The band the far side's chain lays on the same cap the skirt runs across: the one facing the
/// skirt from the other side of the gap.
fn the_far_sides_band_on(cuts: &[SurfaceRoomCut], skirt: &SurfaceRoomCut) -> SurfaceRoomCut {
    single(cuts, |cut| {
        cut.region_key == skirt.region_key
            && cut.shape == SurfaceRoomFootprint::Band
            && cut.inward.dot(Vector3::UNIT_X) > 0.999
            && cut.from.x < 1e-6
    })
}

/// The bottom cap's run-out cut: the one on the cap reaching back along the flat side.
fn the_skirt_across_the_bottom_cap(cuts: &[SurfaceRoomCut]) -> SurfaceRoomCut {
    // Told from the seam's own EDGE cut, which reaches the same way on the flat wall beside it, by
    // its room: a run-out is bounded by another chain's band and by nothing else. And told from
    // the run-out's own WALL bite, which travels the same way out of the same corner: that one is
    // laid DOWN the wall, so its width runs along z where a cut on the cap has none.
    single(cuts, |cut| {
        cut.inward.dot(-Vector3::UNIT_X) > 0.999
            && cut.from.z.abs() < 1e-6
            && cut.along.z.abs() < 1e-9
            && cut.room == f64::INFINITY
    })
}

/// The turns the run-out's own trim curve is drawn at, corner turn first: the stations
/// `surface_mixed_corner_run_out` hands the build, as turns about the bead's axis.
fn turns_of(prism: &Mesh, corner: &Rc<SurfaceMixedCorner>) -> Vec<f64> {
    let wall = SurfaceMixedCornerWall::try_read(corner)
        .unwrap_or_else(|why| panic!("the wall has to read: {:?}", why.code()));

    let past = surface_mixed_corner_run_out::try_turns_past_the_bead(
        corner,
        &wall,
        corner.turn_radians() / 8.0,
        SurfaceVertexPool::near_enough_for(&prism.vertices),
        None,
        None,
    )
    .unwrap_or_else(|refused| panic!("the run-out has to read: {:?}", refused.code()));

    past.iter()
        .map(|turn| corner.turn_radians() + turn)
        .collect()
}

/// How far the run-out really reaches across the cap, read off the construction itself: its last
/// station's rail point, which is where the trim curve dies on the sharp edge the part keeps.
fn reach_of(prism: &Mesh, corner: &Rc<SurfaceMixedCorner>) -> f64 {
    let turns = turns_of(prism, corner);

    let last = corner.outer_rail_point(turns[turns.len() - 1]);
    let away = last - corner.corner;

    (away - (corner.out_of_the_cap_face * away.dot(corner.out_of_the_cap_face))).length()
}

/// The mixed corner at the bottom of the seam, which is the one this fixture's skirt runs from.
fn the_corner_at_the_bottom_of_the_seam(plan: &SurfaceBevelPlan) -> Rc<SurfaceMixedCorner> {
    for key in plan.mixed_corner_keys() {
        if key >= 0 {
            if let Some(mixed) = plan.try_mixed_corner_after(key) {
                if (mixed.corner - the_corner()).length() < 1e-3 {
                    return mixed;
                }
            }
        }
    }

    panic!("the barrel's rim turns a mixed corner at the bottom of each seam");
}

/// The barrel's own rim, planned alone.
fn the_barrel_rim(size: f64) -> (Mesh, Arc<FaceRegionCache>, Rc<SurfaceBevelPlan>) {
    let prism = lobed_prism_meshes::create();
    let regions = regions_of(&prism);
    let mut plans = plans_for(&regions, size, the_barrel(&prism).boundary_edges());

    assert_eq!(plans.len(), 1, "the barrel's rim is one chain");

    let plan = plans.remove(0);

    (prism, regions, plan)
}

/// The barrel's rim and the far side's, planned together on a prism of their own - the pick this
/// class is about.
fn both_rims_alone(size: f64) -> Vec<Rc<SurfaceBevelPlan>> {
    let prism = lobed_prism_meshes::create();

    both_rims(&prism, &regions_of(&prism), size)
}

/// The barrel's rim and the far side's, planned together.
fn both_rims(prism: &Mesh, regions: &Arc<FaceRegionCache>, size: f64) -> Vec<Rc<SurfaceBevelPlan>> {
    let mut picked: Vec<FeatureEdge> = the_barrel(prism).boundary_edges().to_vec();

    picked.extend_from_slice(the_far_side(prism).boundary_edges());

    plans_for(regions, size, &picked)
}

/// The flat wall the corner's bead stands against: the prism's own `y = 0` side.
fn the_flat_wall(prism: &Mesh) -> FaceGroup {
    region_at(prism, |face| {
        (normal_of(prism, face) + Vector3::UNIT_Y).length() < 1e-6
            && middle_of(prism, face).x < lobed_prism_meshes::left_crossing().x
    })
}
