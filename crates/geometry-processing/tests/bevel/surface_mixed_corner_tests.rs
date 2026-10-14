//! Rung 7 of the polygon-space bevel plan, measured rather
//! than argued - the corner of the L where the CONVEX cut along the top of the step wall runs into
//! the CONCAVE bead along its inner edge.
//!
//! The fixture in its own coordinates: `create_l_shaped_grid_prism` is the 20 square less the 10
//! square bite at `x, y > 10`, 10 tall. Three faces meet at `V = (10, 10, 10)`: T the cap (`z = 10`),
//! S the step wall (`x = 10` for `y > 10`, the face the two picked edges SHARE) and W the other wall
//! of the bite (`y = 10` for `x > 10`). The picked CONVEX edge is `T ∩ S`, the picked CONCAVE edge
//! `S ∩ W`; the third edge at `V`, `T ∩ W`, is convex and NOBODY PICKED IT.
//!
//! At `r = 1` both setbacks are `r`; S is walked back to the rectangle `y >= 11, z <= 9` whose
//! corner is `P = (10, 11, 9)`. Stopped on the other's setback plane, the cut's ring and the bead's
//! ring meet at `P` and nowhere else, and between them is one TORUS patch: the rolling ball of
//! radius `r` whose centre runs round the circle of radius `2r` in `z = 10 - r` about the bead's
//! axis. Its footprint on the two faces nobody picked is curved, which is what the run-out reads.

use std::f64::consts::PI;

use geometry_processing::bevel::bevel_chain_linker::BevelSweptChain;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_face_walkback, surface_mesh_closure, surface_mixed_corner_patch,
    SurfaceBevelPlan, SurfaceStrip, SurfaceVertexPool,
};
use geometry_processing::bevel::BevelAdvisoryCode;
use geometry_processing::topology::{EdgeConvexity, FaceRegionCache};
use polygon_mesh::CancelToken;
use vector_math::Vector3;

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::surface_mixed_corner_fixture as fixture;
use crate::surface_walkback_checks::{area_on, vertices_on};
use crate::topology_test_meshes::create_l_shaped_grid_prism;

const RADIUS: f64 = 1.0;

const SEGMENTS: i32 = 8;

/// Positions are read off float vertices and crossed with planes; a ten-millionth is exact here.
const EXACT: f64 = 1e-6;

/// The corner the two picked edges meet at - the top of the L's reflex edge.
fn top_corner() -> Vector3 {
    Vector3::new(10.0, 10.0, 10.0)
}

/// The bead's rolling-ball axis, at the cap face: a radius out from S and a radius out from W.
fn bead_axis_at_the_cap() -> Vector3 {
    Vector3::new(10.0 + RADIUS, 10.0 + RADIUS, 10.0)
}

/// The corner the shared face is left with: where the cut's tangent line on S runs into the bead's,
/// and the one point the two end rings have in common.
fn shared_face_corner() -> Vector3 {
    Vector3::new(10.0, 10.0 + RADIUS, 10.0 - RADIUS)
}

fn assert_within(actual: f64, expected: f64, tolerance: f64, because: &str) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} is not {expected} within {tolerance}: {because}"
    );
}

/// The cut's ring at the bead's setback plane and the bead's ring at the cut's setback plane meet
/// at the shared face's own corner, and touch nowhere else.
#[test]
fn the_two_end_rings_meet_at_one_point_on_the_face_they_share() {
    let (cut, cut_stops_at, bead, bead_stops_at) = the_corner_as_planned();

    assert_eq!(cut.rail_count(), SEGMENTS + 1);
    assert_eq!(bead.rail_count(), SEGMENTS + 1);

    let cut_ring = ring_at(&cut, cut_stops_at, Vector3::UNIT_Y);
    let bead_ring = ring_at(&bead, bead_stops_at, Vector3::UNIT_Z);

    // Rail 0 of each is the rail lying on the face they share, and it is the same point.
    assert!(
        (cut_ring[0] - shared_face_corner()).length() < EXACT,
        "the cut's ring starts on S at the shared face's corner - instead {:?}",
        cut_ring[0]
    );
    assert!(
        (bead_ring[0] - shared_face_corner()).length() < EXACT,
        "and so does the bead's - instead {:?}",
        bead_ring[0]
    );

    // The other ends are on the two faces nobody shares: the cut's on the cap, the bead's on the wall.
    let s = SEGMENTS as usize;
    assert!((cut_ring[s] - Vector3::new(10.0 - RADIUS, 10.0 + RADIUS, 10.0)).length() < EXACT);
    assert!((bead_ring[s] - Vector3::new(10.0 + RADIUS, 10.0, 10.0 - RADIUS)).length() < EXACT);

    // And they touch at that one point only - a ring pair that crossed twice would be two surfaces
    // overlapping rather than two ends of one.
    let touching = cut_ring
        .iter()
        .flat_map(|on_the_cut| {
            bead_ring
                .iter()
                .filter(move |on_the_bead| (*on_the_cut - **on_the_bead).length() < EXACT)
        })
        .count();

    assert_eq!(
        touching, 1,
        "the two rings meet at the shared face's corner and nowhere else"
    );
}

/// Both rings lie on ONE torus about the bead's axis - the rolling ball of radius `r` carried round
/// a centre circle of radius `2r` in the plane `z = 10 - r` - and they are its two boundary rails,
/// sample for sample.
#[test]
fn the_blend_between_them_is_one_torus_patch_about_the_beads_axis() {
    let (cut, cut_stops_at, bead, bead_stops_at) = the_corner_as_planned();

    let cut_ring = ring_at(&cut, cut_stops_at, Vector3::UNIT_Y);
    let bead_ring = ring_at(&bead, bead_stops_at, Vector3::UNIT_Z);

    // The centre circle: radius 2r about the bead's axis, a radius BELOW the cap face. Its two ends
    // are the two convex edges' own centre lines - the picked one and the one nobody picked.
    let centre = bead_axis_at_the_cap() - Vector3::new(0.0, 0.0, RADIUS);

    assert!(
        (ball_centre_at(0.0) - Vector3::new(10.0 - RADIUS, 10.0 + RADIUS, 10.0 - RADIUS)).length()
            < EXACT,
        "at the angle facing the shared face the rolling ball is on the CUT's own centre line"
    );
    assert!(
        (ball_centre_at(PI / 2.0) - Vector3::new(10.0 + RADIUS, 10.0 - RADIUS, 10.0 - RADIUS))
            .length()
            < EXACT,
        "a quarter turn on it is the centre line the unpicked edge's cut would have had"
    );

    let segments = f64::from(SEGMENTS);

    // The cut's ring is the patch's rail at turn zero: its sample i stands a radius from the ball
    // centre there, at the profile angle the strip sampled the arc at.
    for i in 0..=SEGMENTS {
        let profile = f64::from(SEGMENTS - i) * (PI / 2.0 / segments);

        assert!(
            (cut_ring[i as usize] - patch_point(0.0, profile)).length() < EXACT,
            "the cut's sample {i} is the patch at turn 0, profile {:.2} - instead {:?}",
            profile.to_degrees(),
            cut_ring[i as usize]
        );
    }

    // And the bead's ring is the patch's OTHER rail, where the ball touches the shared face's plane:
    // its sample k stands at the turn the bead's own chord ridge k is at.
    for k in 0..=SEGMENTS {
        let turn = f64::from(k) * (PI / 2.0 / segments);

        assert!(
            (bead_ring[k as usize] - patch_point(turn, PI / 2.0)).length() < EXACT,
            "the bead's sample {k} is the patch at turn {:.2}, profile 90 - instead {:?}",
            turn.to_degrees(),
            bead_ring[k as usize]
        );
    }

    // Where the ball is, said once more in the way that makes it a fillet rather than a lofted
    // guess: every point of the patch stands exactly a radius from the centre it was drawn about.
    for station in 0..=SEGMENTS {
        for i in 0..=SEGMENTS {
            let turn = f64::from(station) * (PI / 2.0 / segments);
            let profile = f64::from(i) * (PI / 2.0 / segments);

            assert_within(
                (patch_point(turn, profile) - ball_centre_at(turn)).length(),
                RADIUS,
                EXACT,
                "a radius from the ball",
            );
        }
    }

    assert!((centre - Vector3::new(11.0, 11.0, 9.0)).length() < EXACT);
}

/// The footprint the patch leaves is CURVED, and it lies on the two faces nobody picked: on the cap
/// an arc of radius `2r` that turns a whole right angle over the bead's rim, and on the wall nobody
/// picked a runout `r sqrt 3` along the unpicked edge before the ball clears that wall.
#[test]
fn the_footprint_the_blend_leaves_is_curved_and_reaches_two_faces_nobody_picked() {
    let step = PI / 2.0 / f64::from(SEGMENTS);

    // The outer rail - where the ball touches the cap face - is an arc of radius 2r about the bead's
    // axis, in the cap face's own plane.
    for station in 0..=SEGMENTS {
        let at = patch_point(f64::from(station) * step, 0.0);

        assert_within(
            at.z,
            10.0,
            EXACT,
            "the ball touches the cap face all the way round",
        );
        assert_within(
            (at - bead_axis_at_the_cap()).length(),
            2.0 * RADIUS,
            EXACT,
            "2r",
        );
    }

    // A line it is not, at either scale. Over the rim the rail turns a whole right angle less the
    // half step each end chord's own direction lags the arc by, so there is no line to walk the cap
    // face back to; and within one step its chord still stands the sagitta inside the arc.
    let first_chord = (patch_point(step, 0.0) - patch_point(0.0, 0.0)).get_normal();
    let last_chord =
        (patch_point(PI / 2.0, 0.0) - patch_point((PI / 2.0) - step, 0.0)).get_normal();

    assert_within(
        first_chord.dot(last_chord).acos().to_degrees(),
        90.0 - step.to_degrees(),
        1e-6,
        "the footprint's boundary on the cap face turns a right angle over the bead's rim",
    );

    let sagitta = 2.0 * RADIUS * (1.0 - (step / 2.0).cos());

    let mid_chord = (patch_point(0.0, 0.0) + patch_point(step, 0.0)) / 2.0;

    assert_within(
        2.0 * RADIUS - (mid_chord - bead_axis_at_the_cap()).length(),
        sagitta,
        EXACT,
        "the footprint's boundary on the cap face curves away from every line through it",
    );

    // And the roll does not stop where the bead's rim does. Past the quarter turn the outer rail is
    // still 2r from the axis, so it crosses the unpicked wall's plane (y = 10) at 60 degrees more of
    // turn and reaches r sqrt 3 along the unpicked edge from the bead's own tangent line there.
    let clears = (RADIUS / (2.0 * RADIUS)).acos();
    let runs_out_to = patch_point((PI / 2.0) + clears, 0.0);

    assert_within(
        runs_out_to.y,
        10.0,
        EXACT,
        "that is where the ball leaves the wall nobody picked",
    );
    assert_within(
        runs_out_to.x - (10.0 + RADIUS),
        RADIUS * 3f64.sqrt(),
        EXACT,
        &format!("the runout carries r root 3 along the unpicked edge - instead {runs_out_to:?}"),
    );
}

/// The PLAN takes the corner: it records it and the walkback stops each band on the plane that
/// corner names, which leaves the shared face exactly the rectangle the two tangent lines cut it
/// down to - corner `P` and all. S keeps 81, the cap loses a 1 by 9 band bounded at the handover
/// plane `y = 11`, and W loses its own 1 by 9 band, bounded at `z = 9`.
#[test]
fn the_walkback_stops_each_band_at_the_corner_and_leaves_the_shared_face_its_rectangle() {
    let mesh = create_l_shaped_grid_prism();
    let (regions, features) = fixture::regions_of(&mesh);
    let chain = fixture::the_joined_chain(&features);

    let plan =
        SurfaceBevelPlan::try_create(&chain, &regions, RADIUS, true).unwrap_or_else(|reason| {
            panic!("the plan records the corner now rather than refusing it - {reason}")
        });

    assert_eq!(plan.mixed_corner_count(), 1);
    let corner = plan.try_mixed_corner_after(0).expect("the corner");
    assert!((corner.corner - top_corner()).length() < EXACT);
    assert!((corner.tangent_point - shared_face_corner()).length() < EXACT);

    let walked =
        surface_face_walkback::try_apply_one(&mesh, &mesh.get_mesh_edge_graph(), &regions, &plan)
            .unwrap_or_else(|reason| panic!("{reason}"));

    assert_within(
        area_on(&walked.mesh, |v| (v.x as f64 - 10.0).abs() < 1e-5),
        81.0,
        1e-4,
        "the shared face is the 9 by 9 rectangle from y = 11 to 20 and z = 0 to 9",
    );

    assert_within(
        area_on(&walked.mesh, |v| (v.z as f64 - 10.0).abs() < 1e-5),
        300.0 - 9.0,
        1e-4,
        "the cap loses a 1 by 9 band, bounded at the handover plane y = 11",
    );

    assert_within(
        area_on(&walked.mesh, |v| (v.y as f64 - 10.0).abs() < 1e-5),
        100.0 - 9.0,
        1e-4,
        "the void wall loses a 1 by 9 band, bounded at the bead's own stop plane z = 9",
    );

    // And the corner the shared face is left with is a real vertex of it, which is what the two end
    // rings will both be sewn to.
    assert!(
        vertices_on(&walked.mesh, |v| (v.x as f64 - 10.0).abs() < 1e-5)
            .iter()
            .any(|at| (*at - shared_face_corner()).length() < 1e-5),
        "the shared face's own corner {:?} is where the two strips touch",
        shared_face_corner()
    );
}

/// The corner BUILDS: the two bites, the torus patch and the run-out leave a closed, manifold,
/// single shell whose volume is the L's own less exactly what the blend sweeps out of it. The two
/// strips are nine long each in a right-angle wedge, so what one takes the other puts back; the patch
/// removes the section's discrete sweep `n sin(dt) M`, and the runout takes the rest.
#[test]
fn the_corner_builds_and_takes_off_the_section_swept_round_the_beads_axis() {
    let mesh = create_l_shaped_grid_prism();
    let (regions, features) = fixture::regions_of(&mesh);

    let plan = SurfaceBevelPlan::try_create(
        &fixture::the_joined_chain(&features),
        &regions,
        RADIUS,
        true,
    )
    .unwrap_or_else(|reason| panic!("{reason}"));

    let built = surface_bevel_builder::try_build_one(
        &mesh,
        &mesh.get_mesh_edge_graph(),
        &regions,
        plan.clone(),
        SEGMENTS,
        &CancelToken::new(),
    )
    .unwrap_or_else(|reason| panic!("{reason:?}"));

    let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
    assert_eq!(open, 0, "the corner closes - open at {open_at:?}");
    assert!(built.is_manifold());
    assert_eq!(shell_count(&built), 1);

    let (section, swept) = fixture::the_corners_own_section(&plan, SEGMENTS);

    // And what is left over is the RUNOUT: past the last turn the ball goes on rolling across the
    // cap until it clears the wall nobody picked, and that is its own reading
    // (surface_mixed_corner_run_out_tests). Named here as the one number it comes to, so that a
    // change in the blend and a change in the runout cannot quietly swap places.
    let over_the_corner = 3000.0 + section - swept - signed_volume(&built);

    assert_within(
        over_the_corner,
        0.1507,
        0.005,
        &format!(
            "the strips cancel, the void square gains the section {section:.6} over the corner's \
             own height, the blend sweeps {swept:.6} out of it, and the runout takes \
             {over_the_corner:.6} more - the boolean's own closed form puts that at 0.1507 r^3"
        ),
    );
}

/// The footprint, on the two faces nobody picked: neither bite touches the face the two edges
/// share, the buried corners are gone, and the sector's rim IS the patch's outer rail.
#[test]
fn the_blends_footprint_is_bitten_out_of_the_two_faces_nobody_picked() {
    let (built, plan) = fixture::the_build(SEGMENTS);

    // How much the two bites take is the RUNOUT's reading now (surface_mixed_corner_run_out_tests).
    // What this one says is that neither of them touched the face the two edges share.
    assert_within(
        area_on(&built, |v| (v.x as f64 - 10.0).abs() < 1e-5),
        81.0,
        1e-6,
        "the shared face is the walkback's alone; no bite reaches it",
    );

    // The two corners of that square the part still HAS: the ones on the meridian plane, where the
    // wall's bite, the cap's and the patch's own end section all meet. The other two stand on the
    // concave edge, which the bead has buried.
    let corners: Vec<Vector3> = built
        .vertices
        .iter()
        .map(|&v| Vector3::from_vector3_float(v))
        .collect();

    assert!(
        corners
            .iter()
            .any(|at| (*at - Vector3::new(10.0 + RADIUS, 10.0, 10.0 - RADIUS)).length() < EXACT),
        "the square's corner on the bead's own stop plane is where the bead's ring ends and the \
         runout's trim begins"
    );

    for buried in [
        Vector3::new(10.0, 10.0, 10.0),
        Vector3::new(10.0, 10.0, 10.0 - RADIUS),
        Vector3::new(10.0 + RADIUS, 10.0, 10.0),
    ] {
        assert!(
            !corners.iter().any(|at| (*at - buried).length() < EXACT),
            "{buried:?} stands inside the blend now - the two on the concave edge under the bead, \
             and the one on the sharp edge under the runout - so nothing is left holding it"
        );
    }

    // And the sector's rim IS the patch's outer rail - the ball's touch point, station by station -
    // rather than a curve of its own that happens to run beside it.
    let corner = plan.try_mixed_corner_after(0).expect("the corner");

    let on_the_cap = vertices_on(&built, |v| (v.z as f64 - 10.0).abs() < 1e-5);

    for station in 0..=SEGMENTS {
        let expected = corner
            .outer_rail_point(f64::from(station) * corner.turn_radians() / f64::from(SEGMENTS));

        assert!(
            on_the_cap
                .iter()
                .any(|at| (*at - expected).length() < EXACT),
            "the bite's rim stands on the outer rail at station {station}, {expected:?}"
        );
    }
}

/// The patch itself: the cut's end ring carried round the bead's axis, station for station, with
/// every vertex of it a radius from the ball centre at its own station - which is what makes it
/// one torus rather than a loft between two rings that happen to meet.
#[test]
fn the_patch_is_the_cuts_ring_turned_about_the_axis_station_for_station() {
    let (built, plan) = fixture::the_build(SEGMENTS);

    let corner = plan.try_mixed_corner_after(0).expect("the corner");

    let cut_first = plan.edges[0].convexity() == EdgeConvexity::Convex;

    let (_, cut_ring) = surface_mixed_corner_patch::try_cut_the_ring(
        &corner,
        &fixture::strip_of(&plan, if cut_first { 0 } else { 1 }, SEGMENTS),
        true,
        None,
    )
    .expect("the cut's ring");
    let (_, bead_ring) = surface_mixed_corner_patch::try_cut_the_ring(
        &corner,
        &fixture::strip_of(&plan, if cut_first { 1 } else { 0 }, SEGMENTS),
        false,
        None,
    )
    .expect("the bead's ring");

    let vertices: Vec<Vector3> = built
        .vertices
        .iter()
        .map(|&v| Vector3::from_vector3_float(v))
        .collect();

    for (station, bead_point) in bead_ring.iter().enumerate() {
        let turn = station as f64 * corner.turn_radians() / (bead_ring.len() - 1) as f64;

        // The bead's own rail k stands at station k of the turn - a round's arc sweeps 180 degrees
        // less its wedge, which is the very angle from the shared face round to the wall.
        assert!(
            (corner.turned_about(cut_ring[0], turn) - *bead_point).length() < EXACT,
            "the bead's rail {station} is the tangent point turned {:.2}",
            turn.to_degrees()
        );

        for on_the_ring in &cut_ring {
            let at = corner.turned_about(*on_the_ring, turn);

            assert!(
                vertices.iter().any(|vertex| (*vertex - at).length() < 1e-5),
                "the patch carries {on_the_ring:?} round to {at:?}"
            );

            assert_within(
                (at - corner.ball_centre_at(turn)).length(),
                RADIUS,
                1e-5,
                "and every point of it stands a radius from the ball it was drawn about",
            );
        }
    }
}

/// The cut the patch will be built on, made at the build's own two planes: every rail of the cut's
/// strip stops on the handover plane `y = 11` and every rail of the bead's on `z = 9`, and the rail
/// each of them has on the face the two edges share lands on the tangent point.
#[test]
fn the_two_end_rings_are_cut_at_the_corners_own_planes() {
    let mesh = create_l_shaped_grid_prism();
    let (regions, features) = fixture::regions_of(&mesh);

    let plan = SurfaceBevelPlan::try_create(
        &fixture::the_joined_chain(&features),
        &regions,
        RADIUS,
        true,
    )
    .unwrap_or_else(|reason| panic!("{reason}"));

    let corner = plan.try_mixed_corner_after(0).expect("the corner");

    let cut_first = plan.edges[0].convexity() == EdgeConvexity::Convex;

    let cut = fixture::strip_of(&plan, if cut_first { 0 } else { 1 }, SEGMENTS);
    let bead = fixture::strip_of(&plan, if cut_first { 1 } else { 0 }, SEGMENTS);

    let (_, cut_ring) = surface_mixed_corner_patch::try_cut_the_ring(&corner, &cut, true, None)
        .unwrap_or_else(|reason| panic!("{reason}"));
    let (_, bead_ring) = surface_mixed_corner_patch::try_cut_the_ring(&corner, &bead, false, None)
        .unwrap_or_else(|reason| panic!("{reason}"));

    assert_eq!(cut_ring.len(), (SEGMENTS + 1) as usize);
    assert_eq!(bead_ring.len(), (SEGMENTS + 1) as usize);

    for at in &cut_ring {
        assert_within(
            at.y,
            10.0 + RADIUS,
            EXACT,
            &format!("the cut's ring stands in the handover plane - instead {at:?}"),
        );
    }

    for at in &bead_ring {
        assert_within(
            at.z,
            10.0 - RADIUS,
            EXACT,
            &format!("the bead's ring stands in its own stop plane - instead {at:?}"),
        );
    }

    // Rail 0 of each is the rail on the face they share, and it is the corner that face is left with.
    assert!(
        (cut_ring[0] - shared_face_corner()).length() < EXACT,
        "instead {:?}",
        cut_ring[0]
    );
    assert!(
        (bead_ring[0] - shared_face_corner()).length() < EXACT,
        "instead {:?}",
        bead_ring[0]
    );

    // And the far end of each is on the face nobody shares - the cap for the cut, the void wall for
    // the bead - which is where the patch's own footprint is still owed.
    let s = SEGMENTS as usize;
    assert!((cut_ring[s] - Vector3::new(10.0 - RADIUS, 10.0 + RADIUS, 10.0)).length() < EXACT);
    assert!((bead_ring[s] - Vector3::new(10.0 + RADIUS, 10.0, 10.0 - RADIUS)).length() < EXACT);
}

/// What the corner promises about the faces it leaves catches a FOLD as well as a fin: two faces on
/// one edge that walk it the SAME way are one side of a surface twice over, and a count that only
/// asks how many faces an edge has calls that an ordinary pair.
#[test]
fn two_faces_walking_one_edge_the_same_way_are_refused() {
    // No mesh behind it: what is being read here is the winding of four vertices and nothing else.
    let mut pool = SurfaceVertexPool::new(&[]);

    let first = pool.add(Vector3::new(0.0, 0.0, 0.0));
    let second = pool.add(Vector3::new(10.0, 0.0, 0.0));
    let third = pool.add(Vector3::new(0.0, 10.0, 0.0));
    let fourth = pool.add(Vector3::new(10.0, 10.0, 0.0));

    // Wound so that both of them run first -> second along the edge they share.
    let folded = [(first, second, third), (first, second, fourth)];

    let failure = surface_mixed_corner_patch::nothing_covers_nothing(&pool, &folded)
        .expect_err("the two faces stand on the same side of the edge they share");

    assert_eq!(
        failure.code(),
        BevelAdvisoryCode::SurfaceResultNotClosed,
        "instead {failure}"
    );

    // And the same pair wound as a surface - one each way along the shared edge - is ordinary.
    let flat = [(first, second, third), (second, first, fourth)];

    assert!(
        surface_mixed_corner_patch::nothing_covers_nothing(&pool, &flat).is_ok(),
        "two faces meeting along one edge from opposite sides are one surface"
    );
}

/// The blend's own frame: the point of the patch at `turn` round the bead's axis and `profile`
/// round the rolling ball, with profile zero on the cap face and profile a right angle on the
/// shared face's plane.
fn patch_point(turn: f64, profile: f64) -> Vector3 {
    ball_centre_at(turn) - (out_of_the_axis(turn) * RADIUS * profile.sin())
        + Vector3::new(0.0, 0.0, RADIUS * profile.cos())
}

/// Where the rolling ball's centre is at `turn`: 2r out, a radius down.
fn ball_centre_at(turn: f64) -> Vector3 {
    bead_axis_at_the_cap() + (out_of_the_axis(turn) * 2.0 * RADIUS) - Vector3::new(0.0, 0.0, RADIUS)
}

/// The way out of the bead's axis at `turn`: towards S at zero, towards W at a right angle.
fn out_of_the_axis(turn: f64) -> Vector3 {
    Vector3::new(-turn.cos(), -turn.sin(), 0.0)
}

/// One strip's cross section where it crosses the plane `normal` stands at.
fn ring_at(strip: &SurfaceStrip, point: Vector3, normal: Vector3) -> Vec<Vector3> {
    (0..strip.rail_count())
        .map(|rail| {
            let param = strip
                .try_cross(rail, point, normal)
                .unwrap_or_else(|| panic!("rail {rail} never meets the plane at {point:?}"));

            strip.point_on(rail, param)
        })
        .collect()
}

/// The corner's two strips, each planned on its own one-edge chain - which is what the operation
/// plans too, up to the join - with the plane point each stops at.
fn the_corner_as_planned() -> (SurfaceStrip, Vector3, SurfaceStrip, Vector3) {
    let mesh = create_l_shaped_grid_prism();
    let (regions, features) = fixture::regions_of(&mesh);

    let cut = strip_for(
        &regions,
        &fixture::chain_of(&features, fixture::the_convex_edge(&features)),
    );
    let bead = strip_for(
        &regions,
        &fixture::chain_of(&features, fixture::the_concave_edge(&features)),
    );

    // The cut stops on the bead's own setback plane in the shared face, and the bead on the cut's.
    (
        cut,
        Vector3::new(10.0, 10.0 + RADIUS, 10.0),
        bead,
        Vector3::new(10.0, 10.0, 10.0 - RADIUS),
    )
}

fn strip_for(regions: &std::sync::Arc<FaceRegionCache>, chain: &BevelSweptChain) -> SurfaceStrip {
    let plan = SurfaceBevelPlan::try_create(chain, regions, RADIUS, true)
        .unwrap_or_else(|reason| panic!("the plan refused the edge: {reason}"));

    SurfaceStrip::try_create(&plan.edges[0], SEGMENTS)
        .unwrap_or_else(|reason| panic!("the strip refused the edge: {reason}"))
}
