//! `Frustum`: six `Plane`s with normals pointing out (left, right, bottom, top, front, back), for
//! view culling and line clipping. Ports agg-sharp `VectorMath/Frustum.cs` (and its
//! `FrustumIntersection`).
//!
//! Mapping notes: the C# `Plane[] Planes` (always six) is `planes: [Plane; 6]`, so the
//! `Frustum(Plane[])` constructor's "Must create with six planes" throw cannot happen and it is
//! `from_plane_array`. `Frustum(left, .., back)` is `from_planes`; the normals-and-distance
//! constructor is `from_normals`; the `Left` property is `left()`/`set_left()`. `ClipLine(ref,
//! ref)` takes `&mut`.

use crate::{AxisAlignedBoundingBox, Matrix4X4, Plane, Vector3};

/// How a box relates to a frustum (C# `FrustumIntersection`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FrustumIntersection {
    Inside,
    Outside,
    Intersect,
}

/// Six planes with outward normals (C# `MatterHackers.VectorMath.Frustum`).
#[derive(Debug, Clone, Copy, Default)]
pub struct Frustum {
    // Plane normals should point out
    pub planes: [Plane; 6],
}

impl Frustum {
    /// A frustum of six default (zero) planes (C# `Frustum()`).
    pub fn new() -> Self {
        Self::default()
    }

    /// The left plane (C# property `Left`).
    pub fn left(&self) -> Plane {
        self.planes[0]
    }

    /// Sets the left plane.
    pub fn set_left(&mut self, value: Plane) {
        self.planes[0] = value;
    }

    /// The frustum a projection matrix sees (Gribb/Hartmann plane extraction, adapted to row
    /// vectors), with outward normals.
    pub fn frustum_from_projection_matrix(projection_matrix: Matrix4X4) -> Frustum {
        let mut created_frustum = Frustum::new();
        let m = projection_matrix;
        for i in (0..4).rev() {
            // left
            created_frustum.planes[0][i] = m[(i, 3)] + m[(i, 0)];
            // right
            created_frustum.planes[1][i] = m[(i, 3)] - m[(i, 0)];
            // bottom
            created_frustum.planes[2][i] = m[(i, 3)] + m[(i, 1)];
            // top
            created_frustum.planes[3][i] = m[(i, 3)] - m[(i, 1)];
            // front
            created_frustum.planes[4][i] = m[(i, 3)] + m[(i, 2)];
            // back
            created_frustum.planes[5][i] = m[(i, 3)] - m[(i, 2)];
        }

        for plane in created_frustum.planes.iter_mut() {
            plane.normalize();
            plane.distance_from_origin = -plane.distance_from_origin;
        }

        created_frustum
    }

    /// Clips the segment to every plane in turn; false once it is entirely behind one.
    pub fn clip_line(&self, start_point: &mut Vector3, end_point: &mut Vector3) -> bool {
        for plane in &self.planes {
            if !plane.clip_line(start_point, end_point) {
                // It is entirely behind the plane so
                return false;
            }
        }

        // It has been clipped to all planes and there is still some left.
        true
    }

    /// A frustum whose four side planes meet at the origin (the front plane is at 0), with the
    /// back plane `distance_to_back` along `back_normal`. Normals should point out.
    pub fn from_normals(
        left_normal: Vector3,
        right_normal: Vector3,
        bottom_normal: Vector3,
        top_normal: Vector3,
        back_normal: Vector3,
        distance_to_back: f64,
    ) -> Self {
        Frustum {
            planes: [
                Plane::new(left_normal.get_normal(), 0.0),
                Plane::new(right_normal.get_normal(), 0.0),
                Plane::new(bottom_normal.get_normal(), 0.0),
                Plane::new(top_normal.get_normal(), 0.0),
                Plane::new(back_normal.get_normal(), distance_to_back),
                Plane::new(-back_normal.get_normal(), 0.0),
            ],
        }
    }

    /// A frustum from its six planes. Plane normals should point out.
    pub fn from_planes(
        left: Plane,
        right: Plane,
        bottom: Plane,
        top: Plane,
        front: Plane,
        back: Plane,
    ) -> Self {
        Frustum {
            planes: [left, right, bottom, top, front, back],
        }
    }

    /// A frustum from an array of six planes (C# `Frustum(Plane[])`).
    pub fn from_plane_array(six_planes: [Plane; 6]) -> Self {
        Frustum { planes: six_planes }
    }

    /// Every plane moved by `matrix`.
    pub fn transform(frustum: &Frustum, matrix: Matrix4X4) -> Frustum {
        let mut transformed_frustum = Frustum::new();
        for (target, plane) in transformed_frustum.planes.iter_mut().zip(&frustum.planes) {
            *target = plane.transform(matrix);
        }

        transformed_frustum
    }

    /// Whether the box is inside, outside or crossing the frustum, by its eight corners.
    pub fn get_intersect(&self, bounding_box: &AxisAlignedBoundingBox) -> FrustumIntersection {
        let corners = bounding_box.get_corners();
        let mut all_inside = true;

        for plane in &self.planes {
            let mut any_inside = false;

            for corner in &corners {
                let dist_from_plane = plane.get_distance_from_plane(*corner);
                if dist_from_plane < 0.0 {
                    any_inside = true; // At least one corner is inside or on this plane
                } else {
                    // If any corner is outside this plane, it cannot be completely inside the
                    // frustum
                    all_inside = false;
                }
            }

            if !any_inside {
                // If no corners are inside this plane, the box is outside
                return FrustumIntersection::Outside;
            }
        }

        if all_inside {
            FrustumIntersection::Inside
        } else {
            FrustumIntersection::Intersect
        }
    }
}
