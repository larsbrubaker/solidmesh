//! The rolling ball's per-side cap on an end face that leans off square to the chain: the
//! one-wall-bending wedge sheared along its apex, so its walls stay the same planes (the ball's
//! contacts are the square wedge's) and each end face leans 10 degrees. The ball reads each wall
//! as extended past the leaning end face, so every rail runs at its wall's own distance, and each
//! end's cut is held to hand answers for where each rail ends on each wall's edge with the end
//! face.

use std::collections::HashMap;

use geometry_processing::bevel::level_line::gap_fill::{self, DEFAULT_FILLET_SEGMENTS};
use geometry_processing::bevel::level_line::{BandGapRefusal, BandProfile, GapFillRefusal};
use geometry_processing::bevel::rolling_ball::rolling_ball_routing::ROLLING_BALL_REFUSED;
use vector_math::Vector3;

use crate::band_route_tests::{built, Triangles};
use crate::band_setback_tests::{
    a_wedge_with_one_wall_bending_steeper, extruded, on_the_steeper_walls_edge,
    on_the_straight_walls_edge, the_contact_distances_in_the_one_sided_groove, STEEP_DEGREES,
};

/// The chain: the apex from (0, 0, 0) to (0, 6, 0).
const LENGTH: f64 = 6.0;

/// The wedge with every vertex moved along the chain by `slope` times its x: the walls stay planes
/// and the apex stays the chain, but each end face leans off square to it.
fn a_slanted_wedge(slope: f64) -> (Vec<Vector3>, Triangles) {
    let (positions, triangles) = a_wedge_with_one_wall_bending_steeper();
    let slanted = positions
        .iter()
        .map(|p| Vector3::new(p.x, p.y + (slope * p.x), p.z))
        .collect();
    (slanted, triangles)
}

/// How far `point` stands from the chain, the segment, not its line: past an end, from the end
/// vertex. A side's band is cut where this is the side's distance.
fn from_the_chain(point: Vector3) -> f64 {
    let y = point.y.clamp(0.0, LENGTH);
    (point - Vector3::new(0.0, y, 0.0)).length()
}

/// Where the rail at `distance` ends on a wall's edge with the end face `y = at + slope x`, by
/// hand: the edge is the wall's trace in (x, z), `trace(t)` for t from the apex out, carried onto
/// the face, and the rail ends where it first stands `distance` from the chain. Along the edge the
/// distance only grows, so halving finds it.
fn rail_end_by_hand(
    trace: &dyn Fn(f64) -> (f64, f64),
    at: f64,
    slope: f64,
    distance: f64,
) -> Vector3 {
    let point = |t: f64| {
        let (x, z) = trace(t);
        Vector3::new(x, at + (slope * x), z)
    };
    let (mut lo, mut hi) = (0.0, 20.0);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if from_the_chain(point(mid)) < distance {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    point(0.5 * (lo + hi))
}

/// The wedge's ends leaning 10 degrees off square (too little for an end ray), its sides' contacts
/// 2.34 (the straight wall) and 2.72 (the steeper wall) from the chain at radius 2: the pass takes
/// the ball. Each rail runs at its own side's distance from the chain; each end face's cut meets
/// each wall's edge with it exactly where that wall's rail ends: on the straight wall at end 0 and
/// the steeper wall at end 1 (where the face leans back past the end vertex) the rail's distance
/// from the end vertex, on the others its distance from the chain's line, each worked out here by
/// hand; the fill refuses nothing and the result is closed.
#[test]
fn a_wedge_whose_sides_differ_and_whose_end_face_leans_cuts_each_end_to_meet_both_rails() {
    let slope = 10f64.to_radians().tan();
    let part = a_slanted_wedge(slope);
    let radius = 2.0;
    let (straight, steeper) = the_contact_distances_in_the_one_sided_groove(radius);
    let (ball, route) = built(&part, &[vec![0, 4]], radius, true);
    assert_eq!(route.as_deref(), Some("rolling-ball"));
    assert_eq!(ball.refusal, BandGapRefusal::None);

    let straight_trace = |t: f64| (-t, t);
    let steep = STEEP_DEGREES.to_radians();
    let steeper_trace = |t: f64| (1.0 + (t * steep.cos()), 1.0 + (t * steep.sin()));
    let by_hand = [0.0, LENGTH].map(|at| {
        [
            rail_end_by_hand(&straight_trace, at, slope, straight),
            // The steeper wall's rail lies past its narrow first face, from (1, 1) on.
            rail_end_by_hand(&steeper_trace, at, slope, steeper),
        ]
    });
    // On the end face y = slope x the straight wall (x < 0) leans back past the end vertex, so its
    // rail ends `straight` from the vertex, not from the chain's line.
    let behind = straight / (2.0 + (slope * slope)).sqrt();
    assert!((by_hand[0][0] - Vector3::new(-behind, -slope * behind, behind)).length() < 1e-12);
    assert!((by_hand[1][0].x.hypot(by_hand[1][0].z) - straight).abs() < 1e-12);
    assert!((by_hand[0][1].x.hypot(by_hand[0][1].z) - steeper).abs() < 1e-12);

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
                        (from_the_chain(*p) - expected).abs() < near,
                        "side {} rail at {p:?} stands {} from the chain, not {expected}",
                        gap_loop.side,
                        from_the_chain(*p)
                    );
                }
                continue;
            }
            caps += 1;
            let end = usize::from(points.iter().all(|p| p.y > LENGTH / 2.0));
            let mut met = [0, 0];
            for p in &points {
                let walls = [
                    on_the_straight_walls_edge(*p, near),
                    on_the_steeper_walls_edge(*p, near),
                ];
                for (wall, on) in walls.into_iter().enumerate() {
                    if on && p.x.hypot(p.z) > 0.5 {
                        met[wall] += 1;
                        let expected = by_hand[end][wall];
                        assert!(
                            (*p - expected).length() < near,
                            "end {end}'s cut meets wall {wall} at {p:?}, not at {expected:?}"
                        );
                    }
                }
            }
            assert_eq!(met, [1, 1], "end {end}'s cut meets each wall once");
        }
    }
    assert_eq!((rails, caps), (2, 2), "a rail per side and a cut per end");

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

/// The square wedge with its first end face notched: the end face at the chain's first end is only
/// the triangle from the apex to (-1.5, 0, 1.5) on the straight wall and (1, 0, 1) on the steeper
/// wall, and the rest of that end is a pyramid standing out of it (creased far past the feature
/// angle), a third face. Each wall's edge with the end face (2.12 and 1.41 long) is shorter than
/// its rail's distance (2.34 and 2.72), so no point of the end face's own edge carries the rail's
/// end, and the ball refuses the size, saying so in the route; a rail end read on along the
/// pyramid's edges would cut the end face's cap through points that are not on it. The pass is
/// refused by its size, naming the room the ball's own walk at a smaller size reads
/// (`rolling_ball_size_room`), and the ball builds at that room.
#[test]
fn a_wedge_whose_end_face_is_narrower_than_its_rails_is_refused_by_size_naming_a_size_the_ball_builds(
) {
    let (mut positions, triangles) = a_wedge_with_one_wall_bending_steeper();
    // Vertices 0..4 are the profile at y = 0, 4..8 at y = 6, 8 and 9 the caps' centres.
    let notch = positions.len() as i32;
    positions.push(Vector3::new(-1.5, 0.0, 1.5));
    let apex = notch + 1;
    positions.push(Vector3::new(-0.9, -3.0, 4.1));
    let forward = triangles.contains(&(8, 1, 0))
        || triangles.contains(&(1, 0, 8))
        || triangles.contains(&(0, 8, 1));
    let oriented = |(a, b, c): (i32, i32, i32)| if forward { (a, b, c) } else { (a, c, b) };
    let mut notched: Triangles = Vec::new();
    for &(a, b, c) in &triangles {
        let t = [a, b, c];
        if t.contains(&8) {
            continue;
        }
        // The straight wall's triangle on the edge from vertex 3 to the apex, split at the notch.
        if t.contains(&3) && t.contains(&0) && t.contains(&4) {
            notched.push(oriented((3, notch, 4)));
            notched.push(oriented((notch, 0, 4)));
            continue;
        }
        notched.push((a, b, c));
    }
    for t in [
        (notch, 1, 0),
        (apex, 2, 1),
        (apex, 3, 2),
        (apex, notch, 3),
        (apex, 1, notch),
    ] {
        notched.push(oriented(t));
    }

    let part = (positions, notched);
    let (refused, route) = built(&part, &[vec![0, 4]], 2.0, true);
    assert_eq!(refused.refusal, BandGapRefusal::Size, "{route:?}");
    assert!(refused.mesh.is_none(), "a refused pass hands back no band");
    assert!(refused.room > 0.0 && refused.room < 2.0, "{}", refused.room);
    let (at_the_room, at_route) = built(&part, &[vec![0, 4]], refused.room, true);
    assert_eq!(at_the_room.refusal, BandGapRefusal::None, "{at_route:?}");
    let route = route
        .and_then(|r| r.strip_prefix(ROLLING_BALL_REFUSED).map(str::to_string))
        .expect("the route says the ball refused the size asked");
    assert!(route.starts_with("chain 0 end 0: "), "{route}");
    assert!(
        route.ends_with(
            "its rail does not meet the end face along the side's own edge with it (the end face \
             is narrower than the rail's distance, or the band flows over it)"
        ),
        "{route}"
    );
}

/// A roof ridge of 160 degrees (its faces falling 5 and 15 degrees from the ridge, the chain),
/// sheared so each end face leans 25 degrees off square to it either way, rounded at radius 2: the
/// ball's end station, held in a leaning end plane at so obtuse a corner, touches each side on its
/// extension past the face (bounded at the chain's line carried on past the end vertex, so no
/// strip reaches across it, and filled beside it), on the side's plane and no rim: the pass takes
/// the ball and builds every setback at the hand answer, r tan 10 degrees (both faces flat), never
/// short.
#[test]
fn an_obtuse_ridge_with_a_leaning_end_never_builds_short() {
    let (left, right) = (5f64.to_radians().tan(), 15f64.to_radians().tan());
    let profile = [
        (0.0, 0.0),
        (-5.0, -5.0 * left),
        (-5.0, -10.0),
        (5.0, -10.0),
        (5.0, -5.0 * right),
    ];
    let (positions, triangles) = extruded(&profile, LENGTH, None);
    let radius = 2.0;
    let hand = radius * 10f64.to_radians().tan();
    for lean in [25.0f64, -25.0] {
        let slope = lean.to_radians().tan();
        let sheared: Vec<Vector3> = positions
            .iter()
            .map(|p| Vector3::new(p.x, p.y + (slope * p.x), p.z))
            .collect();
        let part = (sheared, triangles.clone());
        let (ball, route) = built(&part, &[vec![0, 5]], radius, true);
        let route = route.expect("the pass writes its route");
        assert_eq!(route, "rolling-ball", "lean {lean}");
        assert_eq!(ball.refusal, BandGapRefusal::None);
        for chain in &ball.chains {
            for &setback in &chain.setbacks {
                assert!(
                    (setback - hand).abs() < 1e-9,
                    "lean {lean}: {setback} where {hand}"
                );
            }
        }
    }
}
