//! The rolling ball's setbacks where they are not the level line's (a side that bends away beyond a
//! narrow first face, so the ball touches the face past it); an open chain whose two sides'
//! contacts differ (a groove and a wedge), each side cut at its own distance and each end face's
//! cut meeting both rails; the passes and builds the ball refuses (strict: never built on the level
//! line's setbacks): such a chain with an end ray or in a pass of several chains, chains meeting at
//! a corner, a chain that is convex along part of its length and concave along the rest; the probes
//! a build runs inside itself, which keep the level line's and never write the route; which group
//! reads a chain's side field; and a closed chain whose sides differ (the groove revolved into a
//! ring), where each side's band is cut at its own side's distance. The grooves, the wedge and the
//! ring are built here; the cube, the prism and the build are `band_route_tests.rs`'s.

use std::collections::HashMap;
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{
    build_one_unvalidated, build_pick, BandGapBuildOptions,
};
use geometry_processing::bevel::level_line::band_gap_setback_fields::PassSetbacks;
use geometry_processing::bevel::level_line::gap_fill::{self, DEFAULT_FILLET_SEGMENTS};
use geometry_processing::bevel::level_line::rolling_ball_setbacks::{
    a_rayed_end_that_differs, for_the_pass, PassScope, SideSetbacks,
};
use geometry_processing::bevel::level_line::{
    chain_fields_of, BandGapRefusal, BandGroup, BandProfile, ChainField, GapFillRefusal,
};
use geometry_processing::bevel::rolling_ball::rolling_ball_routing::{
    last_route, reset_to_the_default, set_last_route, set_use_rolling_ball, ROLLING_BALL_REFUSED,
};
use indexmap::IndexMap;
use vector_math::Vector3;

use crate::band_route_tests::{a_sixty_degree_prism, built, creases, Triangles};

/// The profile, in (x, z), extruded along y from 0 to `length`: each profile point i is vertex i
/// at y = 0 and vertex n + i at y = `length`. Each end cap is a fan from `centre` (vertices 2n
/// and 2n + 1) where given, or else from profile point 0; the profile must be star-shaped from
/// it. Wound so the solid's volume is positive.
pub fn extruded(
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
pub const STEEP_DEGREES: f64 = 65.0;

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

/// The groove with only its right wall (x > 0) bending steeper past the narrow first face; the
/// left wall runs straight at 45 degrees to the block's top. The chain is the floor, vertices 0
/// and 8.
fn a_groove_with_one_wall_bending_steeper() -> (Vec<Vector3>, Triangles) {
    let steep = STEEP_DEGREES.to_radians();
    let (tx, tz) = (1.0 + (4.0 * steep.cos()), 1.0 + (4.0 * steep.sin()));
    let profile = [
        (0.0, 0.0),
        (1.0, 1.0),
        (tx, tz),
        (5.0, tz),
        (5.0, -3.0),
        (-7.0, -3.0),
        (-7.0, tz),
        (-tz, tz),
    ];
    extruded(&profile, 6.0, Some((0.0, -2.0)))
}

/// Where a ball of `radius` touches each wall of the groove with one wall bending steeper, by
/// hand, as (straight wall's distance from the floor's line, steeper wall's). The centre C stands
/// `radius` inside both walls' lines: the straight wall's through the floor with inward normal
/// (1, 1) / sqrt 2, the steeper wall's through (1, 1) with inward normal (-sin 65, cos 65). Each
/// contact is C less `radius` times its wall's normal. The ball must clear the right wall's narrow
/// first face, which the test checks.
pub fn the_contact_distances_in_the_one_sided_groove(radius: f64) -> (f64, f64) {
    let steep = STEEP_DEGREES.to_radians();
    let root_half = std::f64::consts::FRAC_1_SQRT_2;
    let (a1, b1, c1) = (root_half, root_half, radius);
    let (a2, b2) = (-steep.sin(), steep.cos());
    let c2 = radius + a2 + b2;
    let det = (a1 * b2) - (a2 * b1);
    let (cx, cz) = (((c1 * b2) - (c2 * b1)) / det, ((a1 * c2) - (a2 * c1)) / det);
    let along_first_face = ((cx + cz) / 2.0).clamp(0.0, 1.0);
    let off_the_first_face = (cx - along_first_face).hypot(cz - along_first_face);
    assert!(off_the_first_face > radius, "{off_the_first_face}");
    let straight = (cx - (radius * a1)).hypot(cz - (radius * b1));
    let steeper = (cx - (radius * a2)).hypot(cz - (radius * b2));
    (straight, steeper)
}

/// How far `point` stands from the floor's line (the chain, along y at x = z = 0).
fn from_the_floor(point: Vector3) -> f64 {
    point.x.hypot(point.z)
}

/// Whether `point` lies on the straight (left) wall's trace in its end plane: z = -x, x < 0.
pub fn on_the_straight_walls_edge(point: Vector3, near: f64) -> bool {
    point.x < 0.0 && (point.z + point.x).abs() <= near
}

/// Whether `point` lies on the steeper (right) wall's trace in its end plane: the line from (1, 1)
/// at [`STEEP_DEGREES`] to the horizontal, past the narrow first face.
pub fn on_the_steeper_walls_edge(point: Vector3, near: f64) -> bool {
    let steep = STEEP_DEGREES.to_radians();
    let (dx, dz) = (point.x - 1.0, point.z - 1.0);
    point.x > 1.0 && ((dx * steep.sin()) - (dz * steep.cos())).abs() <= near
}

/// The groove's V alone as a solid wedge: its apex the chain (vertices 0 and 4), its left wall
/// straight at 45 degrees and its right wall the groove's, bending steeper past the narrow first
/// face at (1, 1); both walls run up to a flat top at z = 7, and the two ends are square. The ball
/// inside it is the ball in the groove, so its contacts are the groove's.
pub fn a_wedge_with_one_wall_bending_steeper() -> (Vec<Vector3>, Triangles) {
    let steep = STEEP_DEGREES.to_radians();
    let along = 6.0 / steep.sin();
    let profile = [
        (0.0, 0.0),
        (1.0, 1.0),
        (1.0 + (along * steep.cos()), 7.0),
        (-7.0, 7.0),
    ];
    extruded(&profile, 6.0, Some((0.0, 4.0)))
}

/// Builds `part` round the open chain `chains` at radius 2 with the switch on, and checks what the
/// one-wall-bending V (a groove or a wedge) must show: the route is the ball's with no refusal,
/// every rail vertex stands at its own side's hand distance (the straight wall's left of x = 0,
/// the steeper wall's right of it), every end region's cut meets each wall's edge in its end
/// plane once, exactly at that wall's distance (no notch), the fill refuses nothing and the result
/// is closed (every directed edge of a live triangle run once, and once back). Returns how many
/// end regions' loops there were.
fn the_v_builds_on_the_ball_cut_per_side(
    part: &(Vec<Vector3>, Triangles),
    chains: &[Vec<i32>],
) -> i32 {
    let radius = 2.0;
    let (straight, steeper) = the_contact_distances_in_the_one_sided_groove(radius);
    assert!((straight - 2.3416).abs() < 1e-3, "{straight}");
    assert!((steeper - 2.7235).abs() < 1e-3, "{steeper}");
    let (ball, route) = built(part, chains, radius, true);
    assert_eq!(route.as_deref(), Some("rolling-ball"));
    assert_eq!(ball.refusal, BandGapRefusal::None);

    // Within the field's root precision of the hand answer.
    let near = 1e-6;
    let (mut rails, mut caps) = (0, 0);
    {
        let mesh = ball.mesh.as_ref().expect("the band came out").borrow();
        for gap_loop in &ball.loops {
            let points: Vec<Vector3> = gap_loop
                .vertices
                .iter()
                .map(|&v| mesh.position(v))
                .collect();
            if gap_loop.side == 1 || gap_loop.side == 2 {
                rails += 1;
                for p in &points {
                    let expected = if p.x < 0.0 { straight } else { steeper };
                    assert!(
                        (from_the_floor(*p) - expected).abs() < near,
                        "side {} rail at {p:?} stands {} from the floor, not {expected}",
                        gap_loop.side,
                        from_the_floor(*p)
                    );
                }
                continue;
            }
            caps += 1;
            let (mut on_straight, mut on_steeper) = (0, 0);
            for p in &points {
                for (on, expected, count) in [
                    (
                        on_the_straight_walls_edge(*p, near),
                        straight,
                        &mut on_straight,
                    ),
                    (
                        on_the_steeper_walls_edge(*p, near),
                        steeper,
                        &mut on_steeper,
                    ),
                ] {
                    if on && from_the_floor(*p) > 0.5 {
                        *count += 1;
                        assert!(
                            (from_the_floor(*p) - expected).abs() < near,
                            "end region {}'s cut meets a wall at {p:?}, {} from the floor, not \
                             {expected}",
                            gap_loop.side,
                            from_the_floor(*p)
                        );
                    }
                }
            }
            assert_eq!(
                (on_straight, on_steeper),
                (1, 1),
                "end region {}'s cut meets each wall once",
                gap_loop.side
            );
        }
    }
    assert_eq!(rails, 2, "a rail per side");

    let fill =
        gap_fill::fill(&ball, BandProfile::Fillet, DEFAULT_FILLET_SEGMENTS).expect("the fill runs");
    assert_eq!(fill.refusal, GapFillRefusal::None, "{:?}", fill.open_edge);
    let mesh = fill.mesh.as_ref().expect("the filled mesh").borrow();
    let mut directed: HashMap<(i32, i32), i32> = HashMap::new();
    for t in 0..mesh.triangle_count() {
        if mesh.is_removed(t) {
            continue;
        }
        let face = mesh.triangles()[t as usize];
        for (a, b) in [(face.a, face.b), (face.b, face.c), (face.c, face.a)] {
            *directed.entry((a, b)).or_default() += 1;
        }
    }
    for (&(a, b), &count) in &directed {
        assert_eq!(count, 1, "edge {a}-{b} is run {count} times one way");
        assert_eq!(
            directed.get(&(b, a)),
            Some(&1),
            "edge {a}-{b} has no triangle running it back: the mesh is open"
        );
    }
    caps
}

/// Radius 2 in the groove whose right wall alone bends steeper, an open concave chain ending on
/// the block's two square end faces: the ball touches the straight wall 2.34 from the floor and the
/// steeper wall 2.72 from it, each by hand. The pass takes the ball with no refusal and each side's
/// rail runs at its own distance right up to each end face. These end faces fold the opposite way
/// to the chain, so the concave round grows over them rather than cutting them and there is no end
/// region to meet the rails (a concave chain running into an inside corner does get one, cut by
/// its cap as the wedge's are).
#[test]
fn where_one_wall_bends_away_an_open_groove_takes_the_ball_each_rail_at_its_own_distance() {
    let part = a_groove_with_one_wall_bending_steeper();
    assert_eq!(
        the_v_builds_on_the_ball_cut_per_side(&part, &[vec![0, 8]]),
        0
    );
}

/// The same V as a solid wedge, its apex a convex open chain ending on two square end faces, which
/// the round cuts: each end face's cut meets the straight wall's rail at 2.34 and the steeper
/// wall's at 2.72, where each wall's band ends, with no notch along either wall's edge.
#[test]
fn where_one_wall_bends_away_an_open_wedges_end_faces_are_cut_to_meet_each_rail() {
    let part = a_wedge_with_one_wall_bending_steeper();
    assert_eq!(
        the_v_builds_on_the_ball_cut_per_side(&part, &[vec![0, 4]]),
        2
    );
}

/// Cube edges 0-1 and 2-6: chain 1's end stands on a side group of chain 0, which names chain 1 in
/// its `side_of_chain` by the group's own tag, not by a side. Such a member - and an end region's,
/// a carried, grown-over or both-sided member - reads the chain's own field; only a side member
/// proper reads its side's.
#[test]
fn only_a_side_member_proper_reads_a_side_field() {
    let field = |setback: f64| {
        Rc::new(
            ChainField::from_polyline(
                &[Vector3::ZERO, Vector3::new(1.0, 0.0, 0.0)],
                false,
                &[setback, setback],
            )
            .expect("a two-point chain"),
        )
    };
    let own = [field(1.0), field(1.0)];
    let sides = [[field(2.0), field(3.0)], [field(4.0), field(5.0)]];
    let side_group = BandGroup {
        tag: 1,
        members: vec![0, 1],
        side_of_chain: IndexMap::from([(0, 2), (1, 1)]),
        end_members: vec![1],
        ..BandGroup::default()
    };
    let read = chain_fields_of(&side_group, &own, Some(&sides));
    assert!(Rc::ptr_eq(&read[0], &sides[0][1]), "chain 0's right side");
    assert!(Rc::ptr_eq(&read[1], &own[1]), "an end member's own field");

    let others = [
        BandGroup {
            carried_ends: IndexMap::from([(1, 1)]),
            ..side_group.clone()
        },
        BandGroup {
            end_members: Vec::new(),
            both_sided: vec![1],
            ..side_group.clone()
        },
        BandGroup {
            grow_over_members: vec![1],
            ..side_group.clone()
        },
        BandGroup {
            end_members: Vec::new(),
            end_of: 1,
            ..side_group.clone()
        },
    ];
    for group in &others {
        let read = chain_fields_of(group, &own, Some(&sides));
        assert!(Rc::ptr_eq(&read[1], &own[1]), "{group:?}");
    }
    let read = chain_fields_of(&side_group, &own, None);
    assert!(Rc::ptr_eq(&read[0], &own[0]) && Rc::ptr_eq(&read[1], &own[1]));
}

/// The pass's own setbacks for a fillet with a 30 degree sharp turn, whose fields the cap reads.
fn the_band(
    positions: &[Vector3],
    triangles: &Triangles,
    chains: &[Vec<i32>],
    closed: &[bool],
) -> PassSetbacks {
    PassSetbacks::of(
        positions,
        triangles,
        chains,
        closed,
        30.0,
        BandProfile::Fillet,
    )
    .expect("the pass's setbacks")
}

/// A 60 degree prism's top rim picked as three open chains meeting at its corners: the ball's
/// rule for a corner is not built, so the ball refuses the whole pass, and says so.
#[test]
fn chains_meeting_at_a_corner_are_refused_by_the_ball() {
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
        None,
        0.2,
        BandProfile::Fillet,
        &the_band(&positions, &triangles, &chains, &[false, false, false]),
        false,
    );
    let route = last_route();
    reset_to_the_default();
    assert_eq!(setbacks, None);
    let route = route.expect("the pass writes its route");
    let why = route
        .strip_prefix(ROLLING_BALL_REFUSED)
        .expect("the route says the ball refused");
    assert!(
        why.starts_with("the pass's chains meet at a corner"),
        "{why}"
    );
}

/// An L-shaped block's chain running along a convex rim edge into the concave inner corner: one
/// ball sits on one side of the part, so the ball refuses the whole pass.
#[test]
fn a_chain_both_convex_and_concave_is_refused_by_the_ball() {
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
        None,
        0.2,
        BandProfile::Fillet,
        &the_band(&positions, &triangles, &[vec![2, 3, 9]], &[false]),
        false,
    );
    let route = last_route();
    reset_to_the_default();
    assert_eq!(setbacks, None);
    assert_eq!(
        route.as_deref(),
        Some("refused: chain 0: edge 1 is concave where its first edge is convex")
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

/// The one-wall-bending groove's ring: the profile revolved about the z axis at this mean radius.
pub const RING_RADIUS: f64 = 20.0;

/// How many facets the ring has round the axis.
pub const RING_SEGMENTS: i32 = 256;

/// The one-wall-bending groove's profile (straight wall at x < 0, steeper wall at x > 0) revolved
/// about the z axis: profile point (x, z) at segment j is vertex j n + i at radius
/// [`RING_RADIUS`] + x, angle 2 pi j / [`RING_SEGMENTS`]. A closed ring, with no caps; the groove's
/// floor is the closed chain of vertices j n.
pub fn a_ring_with_one_groove_wall_bending_steeper() -> (Vec<Vector3>, Triangles, Vec<i32>) {
    let steep = STEEP_DEGREES.to_radians();
    let (tx, tz) = (1.0 + (4.0 * steep.cos()), 1.0 + (4.0 * steep.sin()));
    let profile = [
        (0.0, 0.0),
        (1.0, 1.0),
        (tx, tz),
        (5.0, tz),
        (5.0, -3.0),
        (-7.0, -3.0),
        (-7.0, tz),
        (-tz, tz),
    ];
    let n = profile.len() as i32;
    let mut positions = Vec::new();
    for j in 0..RING_SEGMENTS {
        let angle = std::f64::consts::TAU * f64::from(j) / f64::from(RING_SEGMENTS);
        for &(x, z) in &profile {
            let rho = RING_RADIUS + x;
            positions.push(Vector3::new(rho * angle.cos(), rho * angle.sin(), z));
        }
    }
    let mut triangles: Triangles = Vec::new();
    for j in 0..RING_SEGMENTS {
        let next = (j + 1) % RING_SEGMENTS;
        for i in 0..n {
            let k = (i + 1) % n;
            let (a, b, c, d) = (j * n + i, j * n + k, next * n + k, next * n + i);
            triangles.push((a, b, c));
            triangles.push((a, c, d));
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
    let chain = (0..RING_SEGMENTS).map(|j| j * n).collect();
    (positions, triangles, chain)
}

/// How far `point` stands from the closed polygon through `chain`.
fn from_the_closed_chain(positions: &[Vector3], chain: &[i32], point: Vector3) -> f64 {
    let mut nearest = f64::INFINITY;
    for (e, &from) in chain.iter().enumerate() {
        let to = chain[(e + 1) % chain.len()];
        let on =
            ChainField::closest_on_segment(point, positions[from as usize], positions[to as usize]);
        nearest = nearest.min((point - on).length());
    }
    nearest
}

/// Radius 2 round the floor of the groove revolved into a ring, a closed chain: each side's band is
/// cut at its own side's contact distance, the straight wall's 2.34 and the steeper wall's 2.72.
///
/// The hand answers are the straight groove's. A surface of revolution's nearest point to a point
/// in a meridian half-plane lies in that half-plane (its distance squared is rho^2 + rho_c^2 - 2
/// rho rho_c cos(dtheta) + dz^2, least at dtheta = 0), and by the mirror symmetry about the
/// meridian through a chain vertex the ball's centre lies in it, so on the smooth ring the ball
/// solves the profile's problem in that plane. The facets stand inside the smooth ring by at most
/// the sagitta rho (1 - cos(pi / N)) at the radius rho of a contact (at most 25 here, N = 256), so a
/// contact, and the rail at its distance, moves by at most that; the chain's own facets move the
/// distance measured to the chain by the same order. Twice the sagitta at the ring's outer radius
/// bounds both.
#[test]
fn round_a_closed_chain_whose_sides_differ_each_sides_band_takes_its_own_balls_setback() {
    let radius = 2.0;
    let (straight, steeper) = the_contact_distances_in_the_one_sided_groove(radius);
    let tolerance =
        2.0 * (RING_RADIUS + 5.0) * (1.0 - (std::f64::consts::PI / f64::from(RING_SEGMENTS)).cos());
    assert!(tolerance < 0.01 && steeper - straight > 0.3, "{tolerance}");

    let (positions, triangles, chain) = a_ring_with_one_groove_wall_bending_steeper();
    set_use_rolling_ball(true);
    set_last_route(None);
    let ball = build_pick(
        &positions,
        &triangles,
        std::slice::from_ref(&chain),
        &[true],
        Some(creases(&positions, &triangles)),
        radius,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
    )
    .expect("the pick's arguments are good");
    let route = last_route();
    reset_to_the_default();
    assert_eq!(route.as_deref(), Some("rolling-ball"));
    assert_eq!(ball.refusal, BandGapRefusal::None);

    let mut rails = 0;
    {
        let mesh = ball
            .mesh
            .as_ref()
            .expect("the ball's band came out")
            .borrow();
        for gap_loop in &ball.loops {
            rails += 1;
            assert!(
                gap_loop.side == 1 || gap_loop.side == 2,
                "{}",
                gap_loop.side
            );
            let points: Vec<Vector3> = gap_loop
                .vertices
                .iter()
                .map(|&v| mesh.position(v))
                .collect();
            let outside = |p: &Vector3| p.x.hypot(p.y) > RING_RADIUS;
            let on_the_steeper_wall = outside(&points[0]);
            let expected = if on_the_steeper_wall {
                steeper
            } else {
                straight
            };
            for p in &points {
                assert_eq!(outside(p), on_the_steeper_wall, "one side's rail: {p:?}");
                let from_the_floor = from_the_closed_chain(&positions, &chain, *p);
                assert!(
                    (from_the_floor - expected).abs() < tolerance,
                    "side {} rail at {p:?} stands {from_the_floor} from the floor, not {expected}",
                    gap_loop.side
                );
            }
        }
    }
    assert_eq!(rails, 2, "one rail per side");

    let fill =
        gap_fill::fill(&ball, BandProfile::Fillet, DEFAULT_FILLET_SEGMENTS).expect("the fill runs");
    assert_eq!(fill.refusal, GapFillRefusal::None, "{:?}", fill.open_edge);
    let mesh = fill.mesh.as_ref().expect("the filled mesh").borrow();
    let mut directed: HashMap<(i32, i32), i32> = HashMap::new();
    for t in 0..mesh.triangle_count() {
        if mesh.is_removed(t) {
            continue;
        }
        let face = mesh.triangles()[t as usize];
        for (a, b) in [(face.a, face.b), (face.b, face.c), (face.c, face.a)] {
            *directed.entry((a, b)).or_default() += 1;
        }
    }
    for (&(a, b), &count) in &directed {
        assert_eq!(count, 1, "edge {a}-{b} is run {count} times one way");
        assert_eq!(
            directed.get(&(b, a)),
            Some(&1),
            "edge {a}-{b} has no triangle running it back: the mesh is open"
        );
    }
}

/// Why the ball refused a build of `part` round `chains` (open) at radius 2 with the switch on,
/// after checking the build was refused as the ball's (no band handed back) and the route says the
/// same reason.
pub fn refused_by_the_ball(part: &(Vec<Vector3>, Triangles), chains: &[Vec<i32>]) -> String {
    let (ball, route) = built(part, chains, 2.0, true);
    assert_eq!(ball.refusal, BandGapRefusal::RollingBall);
    assert!(ball.mesh.is_none(), "a refused pass hands back no band");
    let why = ball.rolling_ball_refusal.expect("the refusal says why");
    assert_eq!(route, Some(format!("{ROLLING_BALL_REFUSED}{why}")));
    why
}

/// An open chain whose sides differ anywhere and which carries an end ray is refused by the ball,
/// and the reason names the first point where they differ: the rail carried past an oblique end is
/// laid at one setback. (A slanted end whose sides differ there is refused before this, by its
/// cap; this is the chain whose sides agree at its ends and differ between them.) A closed chain,
/// a chain with no end ray and sides that agree keep the ball's setbacks. The reading writes no
/// route; the build that refuses on it does, held end to end on "Center Right and Left Side.mcx"
/// (mattercad-model's `bevel_strict_reason_tests`).
#[test]
fn an_open_chain_whose_sides_differ_and_which_carries_an_end_ray_is_refused_by_the_ball() {
    let differ = SideSetbacks {
        left: vec![1.0, 1.0, 1.0],
        right: vec![1.0, 1.5, 1.0],
        caps: [None, None],
    };
    let agree = SideSetbacks {
        right: vec![1.0, 1.0, 1.0],
        ..differ.clone()
    };
    set_last_route(None);
    let why = |ball: &SideSetbacks, closed: bool, rays: (bool, bool)| {
        a_rayed_end_that_differs(std::slice::from_ref(ball), &[closed], &[rays], 2.0)
    };
    assert_eq!(why(&differ, false, (false, false)), None);
    assert_eq!(why(&differ, true, (false, false)), None);
    assert_eq!(why(&agree, false, (true, true)), None);
    assert_eq!(
        why(&differ, false, (false, true)).as_deref(),
        Some(
            "chain 0 point 1: the sides' contacts stand 2 and 3 from the chain, and an oblique \
             end's rail carried past the end is not cut per side yet"
        )
    );
    assert_eq!(last_route(), None);
    reset_to_the_default();
}

/// Two wedges side by side, each apex a chain of one pass: an open chain's end regions in a pass
/// of several chains belong to the whole pick's groups, not one chain end, so the ball refuses the
/// pass.
#[test]
fn open_chains_whose_sides_differ_in_a_pass_of_several_are_refused_by_the_ball() {
    let (mut positions, mut triangles) = a_wedge_with_one_wall_bending_steeper();
    let count = positions.len() as i32;
    let moved: Vec<Vector3> = positions
        .iter()
        .map(|p| Vector3::new(p.x + 30.0, p.y, p.z))
        .collect();
    positions.extend(moved);
    let again: Triangles = triangles
        .iter()
        .map(|&(a, b, c)| (a + count, b + count, c + count))
        .collect();
    triangles.extend(again);
    let route = refused_by_the_ball(
        &(positions, triangles),
        &[vec![0, 4], vec![count, count + 4]],
    );
    assert!(
        route.ends_with(
            "an open chain's end regions in a pass of several chains are not cut per side yet"
        ),
        "{route}"
    );
}
