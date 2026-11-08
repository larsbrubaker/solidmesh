//! The flat end-face fill (`flat_hole_fill`) on its own, before an end hands it a hole: each loop
//! is held to lying flat to 1e-9, covering exactly the loop's area with no zero-area triangle,
//! winding the loop's way and laying every loop edge that has length once in the walked direction -
//! so the closed surface is manifold. Loops are placed on a tilted plane so no axis-aligned
//! shortcut can pass.

use std::collections::HashMap;
use std::f64::consts::PI;

use solidmesh::bevel::level_line::flat_hole_fill;
use solidmesh_math::Vector3;

const SNAP: f64 = 1e-9;

fn plane_origin() -> Vector3 {
    Vector3::new(3.0, -2.0, 5.0)
}

fn plane_u() -> Vector3 {
    Vector3::new(1.0, 1.0, 0.0).get_normal()
}

fn plane_w() -> Vector3 {
    Vector3::new(-1.0, 1.0, 2.0).get_normal()
}

#[test]
fn a_square_fills_flat_in_the_loops_winding() {
    assert_fills_flat(&[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]);
}

#[test]
fn a_concave_l_fills_with_no_triangle_outside_it() {
    assert_fills_flat(&[
        (0.0, 0.0),
        (2.0, 0.0),
        (2.0, 1.0),
        (1.0, 1.0),
        (1.0, 2.0),
        (0.0, 2.0),
    ]);
}

#[test]
fn collinear_runs_keep_every_vertex_on_the_boundary() {
    // a rail sampled along each side: every sample is a collinear vertex no ear may leave as a
    // T-junction
    let mut loop2d = Vec::new();
    let corners = [(0.0, 0.0), (3.0, 0.0), (3.0, 1.0), (0.0, 1.0)];
    for side in 0..4 {
        let (ax, ay) = corners[side];
        let (bx, by) = corners[(side + 1) % 4];
        for k in 0..5 {
            let k = k as f64;
            loop2d.push((ax + ((bx - ax) * k / 5.0), ay + ((by - ay) * k / 5.0)));
        }
    }

    assert_fills_flat(&loop2d);
}

#[test]
fn a_loop_whose_row_starts_at_the_end_vertex_fills() {
    // UpperLeftInner's shape: side A's rail runs down the ridge into the end vertex V = (0, 0),
    // then the last row starts AT V and curves back out, notched - a loop that is not star-shaped
    // from its centre, which the fan refused as CornerCapNotStarShaped. Walked clockwise here, so
    // it is reversed to run the fill's way.
    let mut loop2d = vec![(2.0, 0.0), (1.0, 0.0), (0.0, 0.0)];
    for k in 1..8 {
        let angle = PI * k as f64 / 8.0;
        loop2d.push((
            1.0 - angle.cos(),
            0.6 * angle.sin() * if k % 2 == 0 { 1.0 } else { 0.2 },
        ));
    }

    loop2d.push((2.0, 1.0));
    loop2d.reverse();
    assert_fills_flat(&loop2d);
}

#[test]
fn a_loop_off_its_plane_is_left_to_the_faired_filler() {
    let mut loop_points: Vec<Vector3> = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
        .into_iter()
        .map(at)
        .collect();
    loop_points[2] += normal() * 1e-6;
    let triangles = flat_hole_fill::fill(&loop_points, SNAP, None);
    assert!(triangles.is_none());
}

fn normal() -> Vector3 {
    plane_u().cross(plane_w()).get_normal()
}

fn at(p: (f64, f64)) -> Vector3 {
    plane_origin() + (plane_u() * p.0) + (plane_w() * p.1)
}

fn assert_fills_flat(loop2d: &[(f64, f64)]) {
    let loop_points: Vec<Vector3> = loop2d.iter().copied().map(at).collect();
    let (triangles, fill_normal) =
        flat_hole_fill::fill(&loop_points, SNAP, None).expect("the loop fills flat");

    // the loop is counter-clockwise in (u, w), so the fill faces u x w
    assert!(fill_normal.dot(normal()) > 1.0 - 1e-12);

    let n = loop_points.len();
    let mut shoelace = 0.0;
    for i in 0..n {
        let (ax, ay) = loop2d[i];
        let (bx, by) = loop2d[(i + 1) % n];
        shoelace += (ax * by) - (bx * ay);
    }

    let mut area = 0.0;
    let mut directed: HashMap<(i32, i32), i32> = HashMap::new();
    for t in &triangles {
        let a = loop_points[t.a as usize];
        let cross = (loop_points[t.b as usize] - a).cross(loop_points[t.c as usize] - a);
        assert!(cross.dot(normal()) > 0.0);
        area += cross.length() / 2.0;
        for corner in [t.a, t.b, t.c] {
            assert!(
                (loop_points[corner as usize] - plane_origin())
                    .dot(normal())
                    .abs()
                    < 1e-9
            );
        }

        for edge in [(t.a, t.b), (t.b, t.c), (t.c, t.a)] {
            *directed.entry(edge).or_default() += 1;
        }
    }

    assert!((area - (shoelace / 2.0)).abs() < 1e-9);

    // every loop edge with length is laid once, the walked way; every inner edge is laid once
    // each way
    let count = |edge: (i32, i32)| directed.get(&edge).copied().unwrap_or(0);
    for i in 0..n {
        let j = (i + 1) % n;
        if (loop_points[i] - loop_points[j]).length() > SNAP {
            assert_eq!(count((i as i32, j as i32)), 1);
            assert_eq!(count((j as i32, i as i32)), 0);
        }
    }

    for (&(a, b), &edge_count) in &directed {
        let on_loop = b == (a + 1) % n as i32;
        if !on_loop {
            assert_eq!(edge_count, 1);
            assert_eq!(count((b, a)), 1);
        }
    }
}
