//! Rust-only checks that a ray landing exactly on an edge or corner two triangles share hits
//! (`docs/CSHARP_DIVERGENCES.md` 1840). agg-sharp's `MinimalTriangle` counts a point on an edge
//! as inside only for one projected winding (`<= 0`) and `TriangleShape` only for the other
//! (`< 0`), so for half the face directions a click exactly on a mesh edge missed both
//! neighbours and fell through the part. Each case builds a square of four triangles fanned
//! around its center vertex, facing each of the six axis directions, and fires a ray straight at
//! the face onto the center vertex and onto the middle of a shared spoke edge.

use std::sync::Arc;

use polygon_mesh::ray_tracer::{MinimalTriangle, Traceable, TriangleShape};
use vector_math::{Ray, Vector3, Vector3Float};

/// The six face directions: the normal axis and its sign.
const FACINGS: [(usize, f64); 6] = [
    (0, 1.0),
    (0, -1.0),
    (1, 1.0),
    (1, -1.0),
    (2, 1.0),
    (2, -1.0),
];

/// The point at in-plane `(u, v)` on the plane `normal_axis = 0`, whose in-plane axes follow
/// the right-hand rule so the fan winds counter-clockwise seen from `+normal_axis`.
fn place(normal_axis: usize, u: f64, v: f64) -> Vector3 {
    let mut p = [0.0; 3];
    p[(normal_axis + 1) % 3] = u;
    p[(normal_axis + 2) % 3] = v;
    Vector3::new(p[0], p[1], p[2])
}

/// The four triangles of the unit square fanned around its center, wound to face
/// `sign * normal_axis`.
fn fan(normal_axis: usize, sign: f64) -> Vec<[Vector3; 3]> {
    let center = place(normal_axis, 0.5, 0.5);
    let corners = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
    (0..4)
        .map(|i| {
            let a = corners[i];
            let b = corners[(i + 1) % 4];
            let a = place(normal_axis, a.0, a.1);
            let b = place(normal_axis, b.0, b.1);
            if sign > 0.0 {
                [center, a, b]
            } else {
                [center, b, a]
            }
        })
        .collect()
}

/// A ray from the front of the face, straight at in-plane `(u, v)`.
fn ray_at(normal_axis: usize, sign: f64, u: f64, v: f64) -> Ray {
    let target = place(normal_axis, u, v);
    let mut normal = [0.0; 3];
    normal[normal_axis] = sign;
    let normal = Vector3::new(normal[0], normal[1], normal[2]);
    Ray::new(target + normal * 2.0, -normal)
}

/// How many of `triangles` report a hit for `ray`, checking each hit lies on the face plane.
fn hits(triangles: &[Box<dyn Traceable>], ray: &Ray) -> usize {
    triangles
        .iter()
        .filter_map(|triangle| triangle.get_closest_intersection(&mut ray.clone()))
        .inspect(|hit| assert!((hit.distance_to_hit - 2.0).abs() < 1e-9))
        .count()
}

fn minimal_triangles(normal_axis: usize, sign: f64) -> Vec<Box<dyn Traceable>> {
    let faces = fan(normal_axis, sign);
    let corners: Arc<Vec<[Vector3Float; 3]>> = Arc::new(
        faces
            .iter()
            .map(|face| face.map(Vector3Float::from_vector3))
            .collect(),
    );
    (0..faces.len() as i32)
        .map(|face| {
            let corners = Arc::clone(&corners);
            Box::new(MinimalTriangle::new(
                Arc::new(move |f, v| corners[f as usize][v as usize]),
                face,
            )) as Box<dyn Traceable>
        })
        .collect()
}

fn triangle_shapes(normal_axis: usize, sign: f64) -> Vec<Box<dyn Traceable>> {
    fan(normal_axis, sign)
        .into_iter()
        .enumerate()
        .map(|(i, [a, b, c])| Box::new(TriangleShape::new(a, b, c, i as i32)) as Box<dyn Traceable>)
        .collect()
}

/// Every facing: the shared center vertex and the middle of a shared spoke edge are hit by
/// every triangle touching them, and a point inside one triangle by that one alone.
fn check(build: fn(usize, f64) -> Vec<Box<dyn Traceable>>) {
    for (normal_axis, sign) in FACINGS {
        let triangles = build(normal_axis, sign);
        let label = format!("facing {sign} along axis {normal_axis}");
        // The center vertex all four triangles share.
        let on_vertex = hits(&triangles, &ray_at(normal_axis, sign, 0.5, 0.5));
        assert_eq!(on_vertex, 4, "{label}: center vertex");
        // The middle of the spoke from the center to the (0, 0) corner, shared by two.
        let on_edge = hits(&triangles, &ray_at(normal_axis, sign, 0.25, 0.25));
        assert_eq!(on_edge, 2, "{label}: shared edge");
        // Inside the first triangle only (center, (0, 0), (1, 0)).
        let inside = hits(&triangles, &ray_at(normal_axis, sign, 0.5, 0.25));
        assert_eq!(inside, 1, "{label}: inside one triangle");
        // Just off the square.
        let outside = hits(&triangles, &ray_at(normal_axis, sign, 1.25, 0.5));
        assert_eq!(outside, 0, "{label}: off the square");
    }
}

#[test]
fn rust_only_minimal_triangles_hit_on_shared_edges_and_vertices() {
    check(minimal_triangles);
}

#[test]
fn rust_only_triangle_shapes_hit_on_shared_edges_and_vertices() {
    check(triangle_shapes);
}
