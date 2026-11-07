//! How far a point is from a mesh's SURFACE - the measurement a bevel test makes when it wants to
//! know how far the part retreated from a corner, rather than where the nearest vertex ended up.
//!
//! Brute force over every triangle. The fixtures here are a few hundred faces and a handful of probe
//! points, so an acceleration structure would only be something else to be wrong.

#![allow(dead_code)]

use polygon_mesh::Mesh;
use vector_math::Vector3;

/// The distance from `point` to the nearest point of any triangle.
pub fn from(mesh: &Mesh, point: Vector3) -> f64 {
    let mut best = f64::INFINITY;

    for face in mesh.faces.iter() {
        let a = Vector3::from_vector3_float(mesh.vertices[face.v0 as usize]);
        let b = Vector3::from_vector3_float(mesh.vertices[face.v1 as usize]);
        let c = Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]);

        // A degenerate triangle has no interior to be nearest on, and its Voronoi walk divides by
        // zero, so it is skipped rather than clamped.
        if (b - a).cross(c - a).length_squared() > 0.0 {
            best = best.min((closest_point_on_triangle(point, a, b, c) - point).length());
        }
    }

    best
}

/// The Voronoi region walk from Ericson, Real-Time Collision
/// Detection, 5.1.5.
pub fn closest_point_on_triangle(p: Vector3, a: Vector3, b: Vector3, c: Vector3) -> Vector3 {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);

    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }

    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);

    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }

    let vc = (d1 * d4) - (d3 * d2);

    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return a + (ab * (d1 / (d1 - d3)));
    }

    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);

    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }

    let vb = (d5 * d2) - (d1 * d6);

    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return a + (ac * (d2 / (d2 - d6)));
    }

    let va = (d3 * d6) - (d5 * d4);

    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        return b + ((c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6))));
    }

    let denominator = 1.0 / (va + vb + vc);

    a + (ab * (vb * denominator)) + (ac * (vc * denominator))
}

/// The farthest-from reading the section tests comparing two builds share: the farthest any
/// vertex of `one` stands from the surface of `other`.
pub fn farthest_from(one: &Mesh, other: &Mesh) -> f64 {
    one.vertices.iter().fold(0.0, |farthest: f64, vertex| {
        farthest.max(from(other, Vector3::from_vector3_float(*vertex)))
    })
}
