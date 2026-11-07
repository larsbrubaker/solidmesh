//! The mixed corner's RUN-OUT on the L prism - past the
//! bead's last turn the rolling ball goes on across the cap until it clears the wall nobody picked
//! (sixty degrees on a flat wall, `acos(1/2)`), and its trim curve on that wall is the scallop.
//!
//! The scallop is checked against its closed form, the bites it leaves on the cap and the wall
//! against their areas, the corner's volume against what the runout takes at each sampling
//! (converging on the smooth 0.1507 r^3), and the guards - a station a hair short of the end, a
//! corner sharper than the ball can reach, a runout with no room left - by name.

use std::collections::HashMap;

use crate::path_solids::extrude;
use crate::path_solids::Outline;
use geometry_processing::bevel::surface::{
    surface_mesh_closure, surface_mixed_corner_run_out, surface_run_out_column_ends,
    SurfaceBevelPlan, SurfaceEdgePlan, SurfaceMixedCorner, SurfaceMixedCornerWall,
    SurfaceVertexPool,
};
use geometry_processing::bevel::{BevelAdvisoryArg, BevelAdvisoryCode};
use geometry_processing::topology::{FaceRegionCache, FeatureEdgeSet};
use polygon_mesh::{Face, Mesh};
use vector_math::Vector3;

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::surface_mixed_corner_fixture as fixture;
use crate::surface_test_chains::join_and_open;
use crate::surface_walkback_checks::area_on;
use crate::topology_test_meshes::create_l_shaped_grid_prism;

const RADIUS: f64 = 1.0;

const SEGMENTS: i32 = 8;

/// The part's own ruler.
fn near_enough() -> f64 {
    SurfaceVertexPool::near_enough_for(&create_l_shaped_grid_prism().vertices)
}

/// The boolean construction's own closed form for the runout, `BevelClosedForms.BlendRunoutVolume`.
const SMOOTH_RUN_OUT: f64 = 0.1507;

/// What the runout takes at each sampling, measured on the construction and pinned.
fn taken_by_the_run_out() -> HashMap<i32, f64> {
    HashMap::from([(4, 0.158258), (8, 0.152794), (16, 0.151225)])
}

fn top_corner() -> Vector3 {
    Vector3::new(10.0, 10.0, 10.0)
}

/// The angle the part's own ruler makes at the end of the turn - what "a hair" means here.
fn the_last_hair() -> f64 {
    surface_run_out_column_ends::too_close_to_the_end(near_enough(), RADIUS)
}

fn assert_within(actual: f64, expected: f64, tolerance: f64, because: &str) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} is not {expected} within {tolerance}: {because}"
    );
}

/// The ball clears the wall at sixty degrees past the bead, and the curve it trims the wall along is
/// the scallop: `x = 11 + r tan(phi)`, `z = 9 + r sqrt(1 - (2 - sec(phi))^2)`, from the bead's last
/// tangency to the sharp edge `r root 3` along it, flattening as it goes.
#[test]
fn the_ball_clears_the_wall_at_sixty_degrees_and_its_trim_curve_is_the_scallop() {
    let corner = the_corner();

    assert_within(
        surface_mixed_corner_run_out::clears_the_wall().to_degrees(),
        60.0,
        1e-9,
        "sixty degrees",
    );

    let step = corner.turn_radians() / f64::from(SEGMENTS);
    let past = turns_past(&corner, step, near_enough());

    assert_within(
        past[past.len() - 1],
        surface_mixed_corner_run_out::clears_the_wall(),
        1e-12,
        "the last station is named rather than stepped to - it is where the curve dies",
    );

    assert_eq!(
        past.len(),
        6,
        "five whole steps of {} degrees fit in sixty, and then the short one",
        step.to_degrees()
    );

    // The trim curve, against the closed form worked out on paper.
    for &phi in &past {
        let at = scallop(&corner, phi);

        assert_within(at.x, 11.0 + (RADIUS * phi.tan()), 1e-9, "x");
        assert_within(at.y, 10.0, 1e-9, "the trim curve lies in the wall");
        assert_within(
            at.z,
            9.0 + (RADIUS * (1.0 - (2.0 - (1.0 / phi.cos())).powi(2)).sqrt()),
            1e-9,
            "z",
        );
    }

    assert!(
        (scallop(&corner, 0.0) - Vector3::new(11.0, 10.0, 9.0)).length() < 1e-9,
        "at no turn at all it is the bead's own last tangency"
    );

    let dies = scallop(&corner, surface_mixed_corner_run_out::clears_the_wall());

    assert!(
        (dies - Vector3::new(11.0 + (RADIUS * 3f64.sqrt()), 10.0, 10.0)).length() < 1e-9,
        "and it dies on the sharp edge r root 3 along it - instead {dies:?}"
    );

    // Concave: each chord is less steep than the one before, which is what makes the region above
    // the curve the wrong shape for one convex bite and right for one per station.
    let mut slope = f64::MAX;

    for station in 0..past.len() - 1 {
        let from = scallop(&corner, past[station]);
        let to = scallop(&corner, past[station + 1]);
        let rise = (to.z - from.z) / (to.x - from.x);

        assert!(rise < slope, "the scallop flattens as it goes");

        slope = rise;
    }
}

/// The scallop comes off the wall and the sector carries on across the cap: the wall keeps 91 less
/// the square and the scallop, the cap 291 less the sector (150 degrees of it now), the curve dies
/// at a vertex of the part, and past it the unpicked edge is as sharp as the part had it.
#[test]
fn the_scallop_comes_off_the_wall_and_the_sector_carries_on_across_the_cap() {
    let (built, _) = fixture::the_build(SEGMENTS);
    let corner = the_corner();
    let step = corner.turn_radians() / f64::from(SEGMENTS);

    let scallops_area = the_scallops_area(&corner, step);
    assert_within(
        area_on(&built, |v| (v.y as f64 - 10.0).abs() < 1e-5),
        91.0 - 1.0 - scallops_area,
        1e-6,
        &format!("the wall keeps 91 less the square and less the scallop {scallops_area:.6}"),
    );

    let sectors_area = the_sectors_area(&corner, step, SEGMENTS);
    assert_within(
        area_on(&built, |v| (v.z as f64 - 10.0).abs() < 1e-5),
        291.0 - sectors_area,
        1e-6,
        &format!("and the cap keeps 291 less the sector {sectors_area:.6}, which is now 150 degrees of it"),
    );

    // The one point the trim curve dies at is a vertex of the part.
    let dies = Vector3::new(11.0 + (RADIUS * 3f64.sqrt()), 10.0, 10.0);

    assert!(
        built
            .vertices
            .iter()
            .any(|&v| (Vector3::from_vector3_float(v) - dies).length() < near_enough()),
        "the surface fades into the sharp edge at {dies:?}"
    );

    // And past it the edge is SHARP, exactly as the part had it.
    assert_within(
        the_dihedral_along_the_sharp_edge(&built, dies.x + 1.0),
        90.0,
        1e-4,
        "the edge nobody picked is left alone past the point the ball clears the wall",
    );
}

/// The corner closes at every sampling, and what its runout takes comes down on the smooth sweep's
/// 0.1507 r^3 from above as the chords get finer.
#[test]
fn the_corner_closes_and_its_run_out_converges_on_the_smooth_sweep() {
    let taken = taken_by_the_run_out();
    let mut missed_by = f64::MAX;

    for segments in [4, 8, 16] {
        let (built, plan) = fixture::the_build(segments);

        let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
        assert_eq!(
            open, 0,
            "at {segments} segments the runout closes - open at {open_at:?}"
        );
        assert!(built.is_manifold());
        assert_eq!(shell_count(&built), 1);

        let (section, swept) = fixture::the_corners_own_section(&plan, segments);
        let run_out = 3000.0 + section - swept - signed_volume(&built);

        // What the runout takes, at each sampling, NAMED. The blend either side of it is pinned to a
        // ten millionth by its own closed form, so this is the corner's whole volume pinned by
        // difference: a cubic millimetre that moved from one to the other would show here.
        assert_within(
            run_out,
            taken[&segments],
            1e-4,
            &format!("at {segments} segments the runout takes {run_out:.6}"),
        );
        assert!(
            run_out > SMOOTH_RUN_OUT,
            "a chorded runout takes MORE than the smooth one - at {segments} segments it took \
             {run_out:.6}"
        );

        assert!(
            run_out - SMOOTH_RUN_OUT < missed_by,
            "and comes down towards it as the chords get finer - at {segments} segments it is out \
             by {:.6}",
            run_out - SMOOTH_RUN_OUT
        );

        missed_by = run_out - SMOOTH_RUN_OUT;
    }
}

/// A station landing a hair short of the end is dropped rather than stood on: the last chord would
/// be shorter than the part's own ruler.
#[test]
fn a_station_landing_a_hair_short_of_the_end_is_dropped_rather_than_stood_on() {
    let corner = the_corner();

    // A step that lands its last whole station a hundred-thousandth of a radian short of the end.
    let left_over = 1e-5;
    let step = (surface_mixed_corner_run_out::clears_the_wall() - left_over) / 5.0;

    assert!(
        the_last_hair() > left_over,
        "the part's own ruler makes {:.7} of angle at the end of the turn, which is what 'a hair' \
         means here",
        the_last_hair()
    );

    let stations = turns_past(&corner, step, near_enough());

    assert_eq!(
        stations.len(),
        5,
        "four whole steps and the end - the fifth lands inside the hair"
    );

    assert_within(
        stations[stations.len() - 1],
        surface_mixed_corner_run_out::clears_the_wall(),
        1e-12,
        "the end",
    );
    assert_within(
        stations[stations.len() - 2],
        4.0 * step,
        1e-12,
        "and the one before it is the last whole step that had room",
    );

    // Without the hair it would be stood on, which is the shape this guard is about.
    assert_eq!(turns_past(&corner, step, 0.0).len(), 6);
}

/// A corner sharper than the ball can reach is refused by name: the L's notch closed to 45 degrees
/// of void, so the blend would have turned 135 and the runout sixty more.
#[test]
fn a_corner_sharper_than_the_ball_can_reach_is_refused_by_name() {
    let prism = a_sharp_notched_prism();
    let (regions, features) = fixture::regions_of(&prism);

    let chains = join_and_open(&[
        fixture::chain_of(&features, fixture::the_concave_edge(&features)),
        fixture::chain_of(&features, the_sharp_notches_top_rim(&features)),
    ]);

    assert_eq!(chains.len(), 1);

    let failure = SurfaceBevelPlan::try_create(&chains[0], &regions, RADIUS, true)
        .expect_err("the ball cannot reach a corner this sharp");

    assert_eq!(failure.code(), BevelAdvisoryCode::SurfaceCornerNotSupported);
    match failure.args().first() {
        Some(BevelAdvisoryArg::Vector3(at)) => assert!(
            (*at - top_corner()).length() < near_enough(),
            "and it says which corner - instead {failure}"
        ),
        other => panic!("and it says which corner - instead {other:?}"),
    }

    // The wedge really is the sharp one, read off the part rather than assumed: 45 degrees of void.
    let wedge = 180.0 - the_turn_that_would_have_been(&features, &regions).to_degrees();

    assert_within(wedge, 45.0, 1e-6, "the void wedge");
}

/// A runout with no room left - a ruler four radii long, so its first and last stations are one
/// place - is refused rather than collapsing its seed.
#[test]
fn a_run_out_with_no_room_left_is_refused_rather_than_collapsing_its_seed() {
    let corner = the_corner();
    let wall = the_wall(&corner);

    let read = surface_mixed_corner_run_out::try_turns_past_the_bead(
        &corner,
        &wall,
        corner.turn_radians() / f64::from(SEGMENTS),
        RADIUS * 4.0,
        None,
        None,
    );

    let failure = match read {
        Ok(past) => panic!(
            "there is no runout to build between two stations that are one place - instead {} of \
             them",
            past.len()
        ),
        Err(failure) => failure,
    };

    assert_eq!(
        failure.code(),
        BevelAdvisoryCode::SurfaceCornerNotSupported,
        "instead {failure}"
    );
}

/// The turn the corner would have had: 180 degrees less the void wedge, which is the wedge the
/// bead's own profile is drawn in - so the reading refuses but the angle is still the plan's own.
fn the_turn_that_would_have_been(
    features: &FeatureEdgeSet,
    regions: &std::sync::Arc<FaceRegionCache>,
) -> f64 {
    let concave = one_planned_edge(regions, features, fixture::the_concave_edge(features));
    // The convex edge must plan on its own as well; the test fails if it does not.
    let _convex = one_planned_edge(regions, features, the_sharp_notches_top_rim(features));

    (180.0 - concave.wedge_angle_degrees).to_radians()
}

fn one_planned_edge(
    regions: &std::sync::Arc<FaceRegionCache>,
    features: &FeatureEdgeSet,
    edge_index: usize,
) -> SurfaceEdgePlan {
    let chain = fixture::chain_of(features, edge_index);

    let plan = SurfaceBevelPlan::try_create(&chain, regions, RADIUS, true)
        .unwrap_or_else(|reason| panic!("the plan refused the edge on its own: {reason}"));

    plan.edges[0].clone()
}

/// The L with its notch's far wall leaned in to 45 degrees: the step wall runs from (10, 10) out to
/// (17, 17), so the void wedge at the reflex edge is half a right angle.
fn a_sharp_notched_prism() -> Mesh {
    let mut path = Outline::new();

    path.move_to(0.0, 0.0);
    path.line_to(20.0, 0.0);
    path.line_to(20.0, 10.0);
    path.line_to(10.0, 10.0);
    path.line_to(17.0, 17.0);
    path.line_to(17.0, 24.0);
    path.line_to(0.0, 24.0);
    path.close_polygon();

    extrude(&path, 10.0).expect("the notched prism extrudes")
}

fn the_sharp_notches_top_rim(features: &FeatureEdgeSet) -> usize {
    let far = Vector3::new(17.0, 17.0, 10.0);
    let top = top_corner();

    let found: Vec<usize> = (0..features.count())
        .filter(|&i| {
            let edge = &features.edges()[i];

            ((edge.start - top).length() < 1e-6 && (edge.end - far).length() < 1e-6)
                || ((edge.end - top).length() < 1e-6 && (edge.start - far).length() < 1e-6)
        })
        .collect();
    assert_eq!(found.len(), 1);

    found[0]
}

/// The area the scallop takes off the wall: the trapezoids between the trim curve's own stations
/// and the sharp edge at z = 10.
fn the_scallops_area(corner: &std::rc::Rc<SurfaceMixedCorner>, step: f64) -> f64 {
    let mut past = vec![0.0];
    past.extend(turns_past(corner, step, near_enough()));

    let mut area = 0.0;

    for station in 0..past.len() - 1 {
        let from = scallop(corner, past[station]);
        let to = scallop(corner, past[station + 1]);

        area += (to.x - from.x) * (((10.0 - from.z) + (10.0 - to.z)) / 2.0);
    }

    area
}

/// The area the sector takes off the cap: the chord polygon of radius 2r through every station of
/// the patch and the runout, less the part of it that hangs over the void.
fn the_sectors_area(corner: &std::rc::Rc<SurfaceMixedCorner>, step: f64, stations: i32) -> f64 {
    let mut turns: Vec<f64> = (0..=stations)
        .map(|station| f64::from(station) * step)
        .collect();

    for past in turns_past(corner, step, near_enough()) {
        turns.push(corner.turn_radians() + past);
    }

    let mut polygon = 0.0;

    for station in 0..turns.len() - 1 {
        polygon += 2.0 * RADIUS * RADIUS * (turns[station + 1] - turns[station]).sin();
    }

    polygon - (RADIUS * RADIUS * (1.0 + (3f64.sqrt() / 2.0)))
}

/// The dihedral, in degrees, between the two faces on an edge of the part that runs along the sharp
/// edge `y = 10, z = 10` past `x = beyond`.
fn the_dihedral_along_the_sharp_edge(built: &Mesh, beyond: f64) -> f64 {
    let at = |v: i32| Vector3::from_vector3_float(built.vertices[v as usize]);
    let normal_of = |face: &Face| {
        let first = at(face.v0);

        (at(face.v1) - first)
            .cross(at(face.v2) - first)
            .get_normal()
    };

    for face in built.faces.iter() {
        for (first, second) in [(face.v0, face.v1), (face.v1, face.v2), (face.v2, face.v0)] {
            let from = at(first);
            let to = at(second);

            if (from.y - 10.0).abs() > 1e-5
                || (to.y - 10.0).abs() > 1e-5
                || (from.z - 10.0).abs() > 1e-5
                || (to.z - 10.0).abs() > 1e-5
                || from.x < beyond
                || to.x < beyond
            {
                continue;
            }

            let faces: Vec<&Face> = built
                .faces
                .iter()
                .filter(|other| {
                    let corners = [other.v0, other.v1, other.v2];

                    corners.contains(&first) && corners.contains(&second)
                })
                .collect();

            if faces.len() != 2 {
                continue;
            }

            return normal_of(faces[0])
                .dot(normal_of(faces[1]))
                .clamp(-1.0, 1.0)
                .acos()
                .to_degrees();
        }
    }

    panic!("no edge of the part runs along the sharp edge past x = {beyond}");
}

fn the_wall(corner: &std::rc::Rc<SurfaceMixedCorner>) -> SurfaceMixedCornerWall {
    SurfaceMixedCornerWall::try_read(corner)
        .unwrap_or_else(|failure| panic!("the wall has to read: {:?}", failure.code()))
}

fn turns_past(corner: &std::rc::Rc<SurfaceMixedCorner>, step: f64, near_enough: f64) -> Vec<f64> {
    surface_mixed_corner_run_out::try_turns_past_the_bead(
        corner,
        &the_wall(corner),
        step,
        near_enough,
        None,
        None,
    )
    .unwrap_or_else(|failure| panic!("the stations have to read: {:?}", failure.code()))
}

fn scallop(corner: &std::rc::Rc<SurfaceMixedCorner>, past: f64) -> Vector3 {
    surface_mixed_corner_run_out::try_scallop_point(corner, &the_wall(corner), past)
        .expect("the trim point has to read")
}

fn the_corner() -> std::rc::Rc<SurfaceMixedCorner> {
    let (plan, _, _) = fixture::the_plan();

    fixture::the_corner(&plan)
}
