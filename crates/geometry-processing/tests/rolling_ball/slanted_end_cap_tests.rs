//! The rolling ball's per-side cap on an end face that leans off square to the chain: the
//! one-wall-bending wedge sheared along its apex, so its walls stay the same planes (the ball's
//! contacts are the square wedge's) and each end face leans 10 degrees. The pass keeps the level
//! line there today; the cut it is to make is held to hand answers for where each rail ends on
//! each wall's edge with the end face.

use std::collections::HashMap;

use geometry_processing::bevel::level_line::gap_fill::{self, DEFAULT_FILLET_SEGMENTS};
use geometry_processing::bevel::level_line::{BandGapRefusal, BandProfile, GapFillRefusal};
use vector_math::Vector3;

use crate::band_route_tests::{built, Triangles};
use crate::band_setback_tests::{
    a_wedge_with_one_wall_bending_steeper, kept_the_level_line, on_the_steeper_walls_edge,
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

/// The wedge's end faces leaning 10 degrees off square (too little for an end ray): the walk reads
/// each side only on the chain's side of a leaning end face, so its end stations' contacts move
/// off the rails' (2.70 and 2.32 from the chain where the walls' are 2.72 and 2.34), and the pass
/// keeps the level line and says why.
#[test]
fn a_wedge_whose_sides_differ_and_whose_end_face_leans_keeps_the_level_line() {
    let route = kept_the_level_line(&a_slanted_wedge(10f64.to_radians().tan()), &[vec![0, 4]]);
    assert!(
        route.ends_with(
            "an end face that is not square to the chain or does not reach both rails along its \
             own wall edges is not cut per side yet"
        ),
        "{route}"
    );
}

/// The wedge's ends leaning 10 degrees off square (too little for an end ray), its sides' contacts
/// 2.34 (the straight wall) and 2.72 (the steeper wall) from the chain at radius 2: the pass takes
/// the ball. Each rail runs at its own side's distance from the chain; each end face's cut meets
/// each wall's edge with it exactly where that wall's rail ends: on the straight wall at end 0 and
/// the steeper wall at end 1 (where the face leans back past the end vertex) the rail's distance
/// from the end vertex, on the others its distance from the chain's line, each worked out here by
/// hand; the fill refuses nothing and the result is closed.
#[test]
#[ignore = "the walk's end station reads each side only on the chain's side of a leaning end \
            face, so its setbacks there are not the rails' (docs/rolling-ball-fillet.md, Next)"]
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
