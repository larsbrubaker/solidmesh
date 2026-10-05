//! Ports `GeometryProcessingTests/LevelLine/HoleFillTests.cs`: the hole filler (`hole_fill`) on its
//! own, before any bevel uses it - plain loops with known cross-tangents, held to closure,
//! orientation and shape. The cube corner is the classic vertex blend whose exact answer is the
//! rolling ball's sphere, so its deviation is measured, not assumed.

use std::collections::{HashMap, HashSet};
use std::f64::consts::PI;

use geometry_processing::bevel::level_line::hole_fill;
use geometry_processing::bevel::level_line::{HoleFillFailure, HoleFillResult};
use vector_math::{Matrix4X4, Vector3};

/// The fill's boundary-normal error against the same faceting on the sphere. Measured 1.00 degrees
/// on the 8-segments-per-arc cube corner (7.93 degrees against the analytic normal, almost all of
/// it the faceting's own one-sided tilt); the bound leaves room for round-off, not for a worse
/// fill.
const BOUNDARY_NORMAL_DEGREES: f64 = 2.0;

fn fill(loop_points: &[Vector3], tangents: &[Vector3], lengths: &[f64]) -> HoleFillResult {
    hole_fill::fill(loop_points, tangents, lengths).expect("no work budget is in force")
}

#[test]
fn a_planar_square_hole_fills_flat() {
    let mut loop_points = Vec::new();
    let mut tangents = Vec::new();
    // four points a side, counter-clockwise seen from +z, so the fill faces +z
    let corners = [
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    ];
    for side in 0..4 {
        let a = corners[side];
        let b = corners[(side + 1) % 4];
        let inward = Vector3::UNIT_Z.cross(b - a).get_normal();
        let previous_inward = Vector3::UNIT_Z
            .cross(a - corners[(side + 3) % 4])
            .get_normal();
        for k in 0..4 {
            loop_points.push(Vector3::lerp(a, b, k as f64 / 4.0));
            tangents.push(if k == 0 {
                (inward + previous_inward).get_normal()
            } else {
                inward
            });
        }
    }

    let fill = fill(&loop_points, &tangents, &vec![0.25; loop_points.len()]);

    assert_eq!(fill.failure, HoleFillFailure::None);
    assert!(
        fill.vertices.len() > loop_points.len(),
        "a 1 x 1 hole at edge length 0.25 is refined"
    );
    for v in &fill.vertices {
        assert!(v.z.abs() < 1e-9);
    }

    assert_closed_against_loop(&fill, &loop_points);
    for t in &fill.triangles {
        let (a, b, c) = (
            fill.vertices[t.a as usize],
            fill.vertices[t.b as usize],
            fill.vertices[t.c as usize],
        );
        let normal = (b - a).cross(c - a);
        assert!(
            normal.z > 1e-6,
            "every triangle faces +z and none is zero-area"
        );
    }
}

#[test]
fn the_cube_corner_vertex_blend_approaches_the_rolling_ball_sphere() {
    let r = 2.0;
    let (loop_points, tangents, lengths, centre) = cube_corner(r, 8);

    let fill = fill(&loop_points, &tangents, &lengths);

    assert_eq!(fill.failure, HoleFillFailure::None);
    assert_closed_against_loop(&fill, &loop_points);

    let mut min_ratio = f64::MAX;
    let mut max_ratio = 0.0f64;
    for v in &fill.vertices[loop_points.len()..] {
        let ratio = (*v - centre).length() / r;
        min_ratio = min_ratio.min(ratio);
        max_ratio = max_ratio.max(ratio);
    }

    // The one-sided normal of any faceting tilts from the true normal by about half a facet's
    // turn, so the fill's own error is read against the same triangles with their interior
    // vertices put on the sphere.
    let mut on_sphere = fill.vertices.clone();
    for v in &mut on_sphere[loop_points.len()..] {
        *v = centre + (*v - centre).get_normal() * r;
    }

    let faceted = HoleFillResult::new(HoleFillFailure::None, on_sphere, fill.triangles.clone());
    let mut worst_to_faceted = 0.0f64;
    for i in 0..loop_points.len() {
        let fill_normal = boundary_normal(&fill, i as i32);
        worst_to_faceted =
            worst_to_faceted.max(degrees(fill_normal, boundary_normal(&faceted, i as i32)));
    }

    assert!(fill.vertices.len() > loop_points.len());
    // measured [0.9651, 1.0029]: the fill runs a little flatter than the ball (see the fairing's
    // remarks)
    assert!(
        min_ratio > 0.95,
        "no filled vertex sinks inside the rolling ball by more than 5%"
    );
    assert!(
        max_ratio < 1.2,
        "no filled vertex bulges toward the sharp corner by more than 20%"
    );
    assert!(
        worst_to_faceted < BOUNDARY_NORMAL_DEGREES,
        "the fill meets each strip with its tangent plane (discrete G1)"
    );
}

#[test]
fn a_five_sided_dome_fills() {
    let mut loop_points = Vec::new();
    let mut tangents = Vec::new();
    let per_side = 3;
    for side in 0..5 {
        let a = polar(1.0, side as f64 * 2.0 * PI / 5.0);
        let b = polar(1.0, (side + 1) as f64 * 2.0 * PI / 5.0);
        for k in 0..per_side {
            let p = Vector3::lerp(a, b, k as f64 / per_side as f64);
            loop_points.push(p);
            // inward in the plane, lifted 30 degrees so the fill domes up
            let inward = -Vector3::new(p.x, p.y, 0.0).get_normal();
            tangents.push(
                (inward * (PI / 6.0).cos() + Vector3::UNIT_Z * (PI / 6.0).sin()).get_normal(),
            );
        }
    }

    let fill = fill(&loop_points, &tangents, &vec![0.3; loop_points.len()]);

    assert_eq!(fill.failure, HoleFillFailure::None);
    assert_closed_against_loop(&fill, &loop_points);
    for v in &fill.vertices[loop_points.len()..] {
        assert!(v.z > 0.0, "the lifted tangents raise the whole interior");
    }
}

#[test]
fn a_mixed_convexity_loop_fills() {
    let (loop_points, tangents, lengths) = mixed_convexity();

    let fill = fill(&loop_points, &tangents, &lengths);

    assert_eq!(fill.failure, HoleFillFailure::None);
    assert_closed_against_loop(&fill, &loop_points);
}

#[test]
fn the_fill_is_unchanged_by_a_rigid_motion() {
    let (loop_points, tangents, lengths) = mixed_convexity();
    let motion = Matrix4X4::create_rotation(Vector3::new(0.3, -1.1, 0.7))
        * Matrix4X4::create_translation(5.0, -3.0, 11.0);
    let moved_loop: Vec<Vector3> = loop_points
        .iter()
        .map(|p| p.transform_position(motion))
        .collect();
    let moved_tangents: Vec<Vector3> = tangents
        .iter()
        .map(|t| t.transform_normal(motion).get_normal())
        .collect();

    let fill = fill(&loop_points, &tangents, &lengths);
    let moved = self::fill(&moved_loop, &moved_tangents, &lengths);

    assert_eq!(fill.failure, HoleFillFailure::None);
    assert_eq!(moved.failure, HoleFillFailure::None);
    assert_eq!(moved.vertices.len(), fill.vertices.len());
    assert_eq!(
        moved.triangles, fill.triangles,
        "the same triangles, in the same order"
    );
    for (v, m) in fill.vertices.iter().zip(&moved.vertices) {
        assert!((v.transform_position(motion) - *m).length() < 1e-6);
    }
}

/// A point of a quarter arc at an angle.
type ArcPoint = Box<dyn Fn(f64) -> Vector3>;

/// The hole where three equal quarter-cylinder fillets meet at the corner of the solid x, y, z <= 0.
/// Each strip stops at the plane one radius back from the corner, so the hole is bounded by three
/// quarter arcs about the rolling ball's centre (-r, -r, -r), and each strip's cross-tangent is its
/// cylinder's axis.
fn cube_corner(r: f64, segments_per_arc: usize) -> (Vec<Vector3>, Vec<Vector3>, Vec<f64>, Vector3) {
    let c = Vector3::new(-r, -r, -r);
    let mut loop_points = Vec::new();
    let mut tangents = Vec::new();
    // Loop order is chosen so the fill faces away from the solid: x strip end a 0 -> 90 in the
    // y-z plane, y strip end back from z to x, z strip end from x to y.
    let arcs: [(ArcPoint, Vector3); 3] = [
        (
            Box::new(move |a: f64| c + r * Vector3::new(0.0, a.cos(), a.sin())),
            Vector3::UNIT_X,
        ),
        (
            Box::new(move |a: f64| {
                c + r * Vector3::new((PI / 2.0 - a).cos(), 0.0, (PI / 2.0 - a).sin())
            }),
            Vector3::UNIT_Y,
        ),
        (
            Box::new(move |a: f64| c + r * Vector3::new(a.cos(), a.sin(), 0.0)),
            Vector3::UNIT_Z,
        ),
    ];
    for arc in 0..3 {
        let previous_axis = arcs[(arc + 2) % 3].1;
        for k in 0..segments_per_arc {
            loop_points.push((arcs[arc].0)(k as f64 * PI / 2.0 / segments_per_arc as f64));
            // where two strips meet, into the hole is between their two axes
            tangents.push(if k == 0 {
                (arcs[arc].1 + previous_axis).get_normal()
            } else {
                arcs[arc].1
            });
        }
    }

    let chord = 2.0 * r * (PI / 4.0 / segments_per_arc as f64).sin();
    let lengths = vec![chord; loop_points.len()];
    (loop_points, tangents, lengths, c)
}

/// A hole in the plane z = 0 bounded by two outward-bulging arcs and one arc bulging into the hole,
/// with each cross-tangent the in-plane inward direction lifted 20 degrees.
fn mixed_convexity() -> (Vec<Vector3>, Vec<Vector3>, Vec<f64>) {
    let a = Vector3::new(0.0, 0.0, 0.0);
    let b = Vector3::new(3.0, 0.0, 0.0);
    let c = Vector3::new(1.2, 2.6, 0.0);
    let mut loop_points = Vec::new();
    for (from, to, bulge) in [(a, b, 0.5), (b, c, 0.4), (c, a, -0.35)] {
        let outward = (to - from).cross(Vector3::UNIT_Z).get_normal();
        for k in 0..7 {
            let s = k as f64 / 7.0;
            loop_points.push(Vector3::lerp(from, to, s) + outward * bulge * (PI * s).sin());
        }
    }

    let n = loop_points.len();
    let tangents = (0..n)
        .map(|i| {
            let along = loop_points[(i + 1) % n] - loop_points[(i + n - 1) % n];
            let inward = Vector3::UNIT_Z.cross(along).get_normal();
            (inward * (PI / 9.0).cos() + Vector3::UNIT_Z * (PI / 9.0).sin()).get_normal()
        })
        .collect();

    (loop_points, tangents, vec![0.3; n])
}

fn degrees(a: Vector3, b: Vector3) -> f64 {
    a.dot(b).clamp(-1.0, 1.0).acos() * 180.0 / PI
}

fn polar(radius: f64, angle: f64) -> Vector3 {
    Vector3::new(radius * angle.cos(), radius * angle.sin(), 0.0)
}

/// The area-weighted normal of the fill's own triangles around one boundary point.
fn boundary_normal(fill: &HoleFillResult, vertex: i32) -> Vector3 {
    let mut sum = Vector3::ZERO;
    for t in fill
        .triangles
        .iter()
        .filter(|t| t.a == vertex || t.b == vertex || t.c == vertex)
    {
        let (a, b, c) = (
            fill.vertices[t.a as usize],
            fill.vertices[t.b as usize],
            fill.vertices[t.c as usize],
        );
        sum += (b - a).cross(c - a);
    }

    sum.get_normal()
}

/// Each loop edge i -> i+1 is used by exactly one triangle in that direction and never reversed;
/// every other edge is used once each way; the fill is a disk (V - E + F = 1); no triangle is
/// zero-area.
fn assert_closed_against_loop(fill: &HoleFillResult, loop_points: &[Vector3]) {
    for (i, p) in loop_points.iter().enumerate() {
        assert!(
            fill.vertices[i] == *p,
            "the fill reuses the loop's own points"
        );
    }

    let mut directed: HashMap<(i32, i32), i32> = HashMap::new();
    for t in &fill.triangles {
        for e in [(t.a, t.b), (t.b, t.c), (t.c, t.a)] {
            *directed.entry(e).or_insert(0) += 1;
        }

        let (a, b, c) = (
            fill.vertices[t.a as usize],
            fill.vertices[t.b as usize],
            fill.vertices[t.c as usize],
        );
        assert!((b - a).cross(c - a).length() / 2.0 > 1e-9);
    }

    assert!(
        directed.values().all(|&count| count == 1),
        "consistently oriented, manifold"
    );
    let n = loop_points.len() as i32;
    for i in 0..n {
        assert!(directed.contains_key(&(i, (i + 1) % n)));
        assert!(!directed.contains_key(&((i + 1) % n, i)));
    }

    let undirected: HashSet<(i32, i32)> = directed
        .keys()
        .map(|&(a, b)| (a.min(b), a.max(b)))
        .collect();
    let unpaired = directed
        .keys()
        .filter(|&&(a, b)| !directed.contains_key(&(b, a)))
        .count();
    assert_eq!(unpaired as i32, n, "only the loop's own edges are open");
    assert_eq!(
        fill.vertices.len() as i64 - undirected.len() as i64 + fill.triangles.len() as i64,
        1,
        "the fill is a disk"
    );
}
