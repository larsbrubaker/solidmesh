//! The rolling ball's round at a free end, held to its hand answer on straight grooves sliced
//! along their length (`level_line::rolling_ball_free_ends`): each side's setback is capped by the
//! distance from the end vertex, so the round ends as the sphere of radius S (the side's own
//! setback) touching the end vertex, and a rail y from the end along the chain stands
//! sqrt(2 S y - y^2) from it. Both where the sides differ (the one-wall-bending groove) and where
//! they agree (a plain 90 degree V): every free end on the ball's route closes onto its vertex.
//! And one chain with a free end and an end face cut per side (the one-wall-bending wedge).

use std::collections::HashMap;

use geometry_processing::bevel::level_line::gap_fill::{self, DEFAULT_FILLET_SEGMENTS};
use geometry_processing::bevel::level_line::{BandGapRefusal, BandProfile, GapFillRefusal};
use vector_math::Vector3;

use crate::band_route_tests::Triangles;
use crate::band_setback_tests::{the_contact_distances_in_the_one_sided_groove, STEEP_DEGREES};
use crate::free_end_tests::closes_onto_both_end_vertices;

/// The groove's length along y.
const LENGTH: f64 = 16.0;

/// How many slices the groove is cut into along y, so its floor is a chain of edges this long:
/// [`LENGTH`] / [`SLICES`] = 0.25.
const SLICES: i32 = 64;

/// The picked stretch of the floor, as slice numbers: 8 to 56, from y = 2 to y = 14, free at both
/// ends (the floor's crease runs on unpicked to the groove's square ends).
const PICKED: (i32, i32) = (8, 56);

/// `profile`, in (x, z), extruded along y from 0 to [`LENGTH`] in [`SLICES`] equal slices: profile
/// point i at slice j is vertex j n + i; each end is a fan from `centre` (the last two vertices),
/// the profile star-shaped from it. Wound so the solid's volume is positive. Returns the part and
/// the chain of profile point 0 over the slices `picked`.
pub fn sliced(
    profile: &[(f64, f64)],
    centre: (f64, f64),
    picked: (i32, i32),
) -> ((Vec<Vector3>, Triangles), Vec<i32>) {
    let n = profile.len() as i32;
    let mut positions = Vec::new();
    for j in 0..=SLICES {
        let y = LENGTH * f64::from(j) / f64::from(SLICES);
        for &(x, z) in profile {
            positions.push(Vector3::new(x, y, z));
        }
    }
    let mut triangles: Triangles = Vec::new();
    for j in 0..SLICES {
        for i in 0..n {
            let k = (i + 1) % n;
            let (a, b, c, d) = (j * n + i, j * n + k, (j + 1) * n + k, (j + 1) * n + i);
            triangles.push((a, b, c));
            triangles.push((a, c, d));
        }
    }
    let (first, last) = (positions.len() as i32, positions.len() as i32 + 1);
    positions.push(Vector3::new(centre.0, 0.0, centre.1));
    positions.push(Vector3::new(centre.0, LENGTH, centre.1));
    let top = SLICES * n;
    for i in 0..n {
        let k = (i + 1) % n;
        triangles.push((first, k, i));
        triangles.push((last, top + i, top + k));
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
    let chain = (picked.0..=picked.1).map(|j| j * n).collect();
    ((positions, triangles), chain)
}

/// Checks a rail point `p` of the side whose ball setback is `setback` against the hand answer:
/// y the distance along the floor from the nearer end of the picked stretch, h its distance from
/// the floor's line. Clear of the end by the setback and an edge, h is the setback. Nearer, the
/// band is the union of balls through the end vertex: no wider than sqrt(2 S y - y^2) (each chain
/// vertex's setback is at most its distance from the end, and the setbacks read linearly between
/// vertices are under that), and at least as wide as the ball at the last vertex x_i short of S
/// (setback x_i, so h^2 >= 2 x_i y - y^2) and the ball at the first vertex x_i+1 past it (setback S,
/// so h^2 >= S^2 - (y - x_i+1)^2). Returns whether the point lies in the taper.
fn on_the_sphere_through_the_end(p: Vector3, setback: f64) -> bool {
    let edge = LENGTH / f64::from(SLICES);
    let (start, stop) = (edge * f64::from(PICKED.0), edge * f64::from(PICKED.1));
    let y = (p.y - start).min(stop - p.y);
    let h = p.x.hypot(p.z);
    assert!(y >= -1e-9, "a rail point past the picked stretch: {p:?}");
    if y >= setback + edge {
        assert!(
            (h - setback).abs() < 1e-6,
            "{p:?} stands {h}, not {setback}"
        );
        return false;
    }
    let s = setback;
    let below = (s / edge).floor() * edge;
    let above = below + edge;
    let upper = if y < s {
        (2.0 * s * y) - (y * y)
    } else {
        s * s
    };
    let lower = ((2.0 * below * y) - (y * y)).max((s * s) - ((y - above) * (y - above)));
    let h2 = h * h;
    assert!(
        h2 <= upper + 1e-5 && h2 >= lower - 1e-5,
        "{p:?}, {y} from the end, stands {h} from the floor: h^2 {h2} outside [{lower}, {upper}]"
    );
    true
}

/// Radius 2.3 in the straight groove whose right wall alone bends steeper, the floor picked from
/// y = 2 to y = 14, free at both ends: the straight wall's rail stands 2.69 from the floor and the
/// steeper wall's 3.13 (by hand), and near each end each rail follows the sphere of its own
/// side's setback through the end vertex. The cap lagging an edge behind the distance (0, l, l,
/// 2 l, ...) leaves a nub of balls of radius l there, below the lower bound.
#[test]
fn where_the_sides_differ_each_round_ends_as_the_sphere_through_the_end_vertex() {
    let radius = 2.3;
    let (straight, steeper) = the_contact_distances_in_the_one_sided_groove(radius);
    let steep = STEEP_DEGREES.to_radians();
    let (tx, tz) = (1.0 + (4.0 * steep.cos()), 1.0 + (4.0 * steep.sin()));
    let (part, chain) = sliced(
        &[
            (0.0, 0.0),
            (1.0, 1.0),
            (tx, tz),
            (5.0, tz),
            (5.0, -3.0),
            (-7.0, -3.0),
            (-7.0, tz),
            (-tz, tz),
        ],
        (0.0, -2.0),
        PICKED,
    );
    let (_, rails) = closes_onto_both_end_vertices(&part, &chain, radius);
    for rail in &rails {
        let middle = rail.points[rail.points.len() / 2];
        let setback = if middle.x > 0.0 { steeper } else { straight };
        let tapered = rail
            .points
            .iter()
            .filter(|p| on_the_sphere_through_the_end(**p, setback))
            .count();
        assert!(
            tapered > 10,
            "side {}: {tapered} points in the tapers",
            rail.side
        );
    }
}

/// Radius 2.3 in a plain 90 degree V-groove, whose sides agree (each contact r from the floor): on
/// the ball's route its free ends close onto their end vertices too, each rail following the sphere
/// of radius r through the end vertex, where the level line would run its round one setback past
/// the end.
#[test]
fn where_the_sides_agree_a_free_end_still_closes_onto_the_end_vertex() {
    let radius = 2.3;
    let (part, chain) = sliced(
        &[
            (0.0, 0.0),
            (4.0, 4.0),
            (6.0, 4.0),
            (6.0, -3.0),
            (-6.0, -3.0),
            (-6.0, 4.0),
            (-4.0, 4.0),
        ],
        (0.0, -2.0),
        PICKED,
    );
    let (_, rails) = closes_onto_both_end_vertices(&part, &chain, radius);
    for rail in &rails {
        let tapered = rail
            .points
            .iter()
            .filter(|p| on_the_sphere_through_the_end(**p, radius))
            .count();
        assert!(
            tapered > 10,
            "side {}: {tapered} points in the tapers",
            rail.side
        );
    }
}

/// The convex wedge whose right wall alone bends steeper (`band_setback_tests`'s wedge), sliced,
/// its apex picked from its square end at y = 0 (an end face of neither side, the sides differing
/// there, so cut by a cap of its own) to y = 10, free. Radius 2: the pass takes the ball, both
/// rails end together on the free end vertex, the fill (the end faces' fill, with the end region's
/// loop) refuses nothing and the result is closed. The free end stands 6 short of the wedge's far
/// end.
#[test]
fn a_free_end_and_a_cut_end_face_on_one_chain_both_close() {
    let steep = STEEP_DEGREES.to_radians();
    let along = 6.0 / steep.sin();
    let (part, chain) = sliced(
        &[
            (0.0, 0.0),
            (1.0, 1.0),
            (1.0 + (along * steep.cos()), 7.0),
            (-7.0, 7.0),
        ],
        (0.0, 4.0),
        (0, 40),
    );
    let free_end = part.0[chain[chain.len() - 1] as usize];
    let (ball, route) = crate::band_route_tests::built(&part, &[chain], 2.0, true);
    assert_eq!(route.as_deref(), Some("rolling-ball"));
    assert_eq!(ball.refusal, BandGapRefusal::None);
    {
        let mesh = ball.mesh.as_ref().expect("the band came out").borrow();
        let mut at_the_free_end = Vec::new();
        for gap_loop in ball.loops.iter().filter(|l| l.side == 1 || l.side == 2) {
            for &v in [
                gap_loop.vertices[0],
                gap_loop.vertices[gap_loop.vertices.len() - 1],
            ]
            .iter()
            {
                if (mesh.position(v) - free_end).length() < 1e-9 {
                    at_the_free_end.push(v);
                }
            }
        }
        assert_eq!(
            at_the_free_end.len(),
            2,
            "both rails end on the free end vertex"
        );
        assert_eq!(at_the_free_end[0], at_the_free_end[1], "one shared vertex");
        assert!(ball.loops.len() > 2, "the cut end face has its end region");
    }
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
            "edge {a}-{b}: the mesh is open"
        );
    }
}
