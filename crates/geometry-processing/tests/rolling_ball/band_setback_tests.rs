//! The rolling ball's setbacks where they are not the level line's (a side that bends away beyond
//! a narrow first face, so the ball touches the face past it), and the passes and builds that
//! keep the level line's: chains meeting at a corner, a chain that is convex along part of its
//! length and concave along the rest, and the probes a build runs inside itself, which never
//! write the route. Meshes and the build are `band_route_tests.rs`'s.

use geometry_processing::bevel::level_line::band_gap_entry_points::{
    build_one_unvalidated, build_pick, BandGapBuildOptions,
};
use geometry_processing::bevel::level_line::rolling_ball_setbacks::{for_the_pass, PassScope};
use geometry_processing::bevel::level_line::{BandGapRefusal, BandProfile};
use geometry_processing::bevel::rolling_ball::rolling_ball_routing::{
    last_route, reset_to_the_default, set_last_route, set_use_rolling_ball,
};
use vector_math::Vector3;

use crate::band_route_tests::{a_sixty_degree_prism, built, creases, Triangles};

/// The profile, in (x, z), extruded along y from 0 to `length`: each profile point i is vertex i
/// at y = 0 and vertex n + i at y = `length`. Each end cap is a fan from `centre` (vertices 2n
/// and 2n + 1) where given, or else from profile point 0; the profile must be star-shaped from
/// it. Wound so the solid's volume is positive.
fn extruded(
    profile: &[(f64, f64)],
    length: f64,
    centre: Option<(f64, f64)>,
) -> (Vec<Vector3>, Triangles) {
    let n = profile.len() as i32;
    let mut positions: Vec<Vector3> = Vec::new();
    for y in [0.0, length] {
        for &(x, z) in profile {
            positions.push(Vector3::new(x, y, z));
        }
    }
    let mut triangles = Vec::new();
    for i in 0..n {
        let j = (i + 1) % n;
        triangles.push((i, j, n + j));
        triangles.push((i, n + j, n + i));
    }
    match centre {
        Some((x, z)) => {
            positions.push(Vector3::new(x, 0.0, z));
            positions.push(Vector3::new(x, length, z));
            for i in 0..n {
                let j = (i + 1) % n;
                triangles.push((2 * n, j, i));
                triangles.push((2 * n + 1, n + i, n + j));
            }
        }
        None => {
            for i in 1..n - 1 {
                triangles.push((0, i + 1, i));
                triangles.push((n, n + i, n + i + 1));
            }
        }
    }
    let mut volume = 0.0;
    for &(a, b, c) in &triangles {
        let (pa, pb, pc) = (
            positions[a as usize],
            positions[b as usize],
            positions[c as usize],
        );
        volume += pa.dot(pb.cross(pc));
    }
    if volume < 0.0 {
        triangles = triangles.iter().map(|&(a, b, c)| (a, c, b)).collect();
    }
    (positions, triangles)
}

/// The groove's walls past the first bend run at this angle to the horizontal, in degrees.
const STEEP_DEGREES: f64 = 65.0;

/// A 90 degree V-groove along y whose walls run 45 degrees for sqrt 2 (to (+-1, 1)), then bend 20
/// degrees steeper (less than the 30 degree crease, so each wall is one side) for 4, then turn out
/// flat to the block's top. The chain is the groove's floor, vertices 0 and 9.
fn a_groove_whose_walls_bend_steeper() -> (Vec<Vector3>, Triangles) {
    let steep = STEEP_DEGREES.to_radians();
    let (tx, tz) = (1.0 + (4.0 * steep.cos()), 1.0 + (4.0 * steep.sin()));
    let profile = [
        (0.0, 0.0),
        (1.0, 1.0),
        (tx, tz),
        (5.0, tz),
        (5.0, -3.0),
        (-5.0, -3.0),
        (-5.0, tz),
        (-tx, tz),
        (-1.0, 1.0),
    ];
    extruded(&profile, 6.0, Some((0.0, -2.0)))
}

/// Where a ball of `radius` touches the groove's steeper wall, by hand: the steeper walls' lines
/// meet the groove's middle at Q, 1 - tan(65 degrees) high, at 25 degrees to the vertical, so the
/// ball tangent to both sits r / sin(25 degrees) above Q and touches each r / tan(25 degrees) along
/// the line from Q. The setback is that contact's distance from the floor's line.
fn the_steeper_walls_contact_distance(radius: f64) -> f64 {
    let steep = STEEP_DEGREES.to_radians();
    let half = (90.0 - STEEP_DEGREES).to_radians();
    let q = 1.0 - steep.tan();
    let along = radius / half.tan();
    let (x, z) = (along * steep.cos(), q + (along * steep.sin()));
    ((x * x) + (z * z)).sqrt()
}

/// Radius 2 in the groove: the first wall is sqrt 2 wide, less than the radius the 90 degree corner
/// asks, so the ball touches the steeper wall past the bend, 3.29 from the floor where the level
/// line's angle formula says 2. Both walls agree, so the pass takes the ball, and the band's field
/// is cut at the ball's distance, not the formula's.
#[test]
fn where_the_wall_bends_away_the_band_takes_the_balls_setback_not_the_angles() {
    let radius = 2.0;
    let expected = the_steeper_walls_contact_distance(radius);
    assert!((expected - 3.2874).abs() < 1e-3, "{expected}");
    let part = a_groove_whose_walls_bend_steeper();
    let chains = [vec![0, 9]];
    let (ball, route) = built(&part, &chains, radius, true);
    assert_eq!(route.as_deref(), Some("rolling-ball"));
    let (level_line, _) = built(&part, &chains, radius, false);
    assert_eq!(level_line.chains.len(), 1);
    for setback in &level_line.chains[0].setbacks {
        assert!(
            (setback - radius).abs() < 1e-9,
            "the angle formula: {setback}"
        );
    }
    assert_eq!(level_line.refusal, BandGapRefusal::None);
    assert_eq!(ball.refusal, BandGapRefusal::None);
    assert_eq!(ball.chains.len(), 1);
    for setback in &ball.chains[0].setbacks {
        assert!(
            (setback - expected).abs() < 1e-6,
            "{setback} against {expected}"
        );
    }
}

/// A 60 degree prism's top rim picked as three open chains meeting at its corners: the ball's
/// rule for a corner is not built, so the whole pass keeps the level line's, and says so.
#[test]
fn chains_meeting_at_a_corner_keep_the_level_line_whole() {
    let (positions, triangles) = a_sixty_degree_prism();
    let chains = [vec![3, 4], vec![4, 5], vec![5, 3]];
    let crease = creases(&positions, &triangles);
    set_use_rolling_ball(true);
    set_last_route(None);
    let setbacks = for_the_pass(
        &positions,
        &triangles,
        &chains,
        &[false, false, false],
        Some(&*crease),
        0.2,
        BandProfile::Fillet,
        false,
    );
    let route = last_route();
    reset_to_the_default();
    assert_eq!(setbacks, None);
    let route = route.expect("the pass writes its route");
    assert!(
        route.starts_with("level-line: the pass's chains meet at a corner"),
        "{route}"
    );
}

/// An L-shaped block's chain running along a convex rim edge into the concave inner corner: one
/// ball sits on one side of the part, so the whole pass keeps the level line's.
#[test]
fn a_chain_both_convex_and_concave_keeps_the_level_line_whole() {
    let profile = [
        (0.0, 0.0),
        (2.0, 0.0),
        (2.0, 1.0),
        (1.0, 1.0),
        (1.0, 2.0),
        (0.0, 2.0),
    ];
    let (positions, triangles) = extruded(&profile, 1.0, None);
    let crease = creases(&positions, &triangles);
    set_use_rolling_ball(true);
    set_last_route(None);
    let setbacks = for_the_pass(
        &positions,
        &triangles,
        &[vec![2, 3, 9]],
        &[false],
        Some(&*crease),
        0.2,
        BandProfile::Fillet,
        false,
    );
    let route = last_route();
    reset_to_the_default();
    assert_eq!(setbacks, None);
    assert_eq!(
        route.as_deref(),
        Some("level-line: chain 0: edge 1 is concave where its first edge is convex")
    );
}

/// The builds a pass runs inside itself, and every topology-only build, keep the level line's
/// setbacks and leave the outer pass's route as it was.
#[test]
fn probes_keep_the_outer_passs_route() {
    let (positions, triangles) = a_sixty_degree_prism();
    let crease = creases(&positions, &triangles);
    set_use_rolling_ball(true);
    set_last_route(Some("the outer pass".to_string()));
    {
        let _outer = PassScope::enter();
        let nested = build_pick(
            &positions,
            &triangles,
            &[vec![0, 3]],
            &[false],
            Some(crease.clone()),
            0.2,
            BandProfile::Fillet,
            &BandGapBuildOptions::default(),
        )
        .expect("the pick's arguments are good");
        assert_eq!(nested.refusal, BandGapRefusal::None);
    }
    let after_nested = last_route();
    let _ = build_one_unvalidated(
        &positions,
        &triangles,
        &[0, 3],
        false,
        Some(crease),
        0.2,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
        true,
    );
    let after_probe = last_route();
    reset_to_the_default();
    assert_eq!(after_nested.as_deref(), Some("the outer pass"));
    assert_eq!(after_probe.as_deref(), Some("the outer pass"));
}
