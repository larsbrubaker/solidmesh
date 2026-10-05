//! A triangle that keeps its own corners (agg-sharp `RayTracer/Primitive/Shapes/TriangleShape.cs`,
//! with the `BaseShape.cs` members it inherits). MatterCAD builds these when a mesh is traced
//! with a material (`AddTraceables`) and in its trace tests; picking uses the lighter
//! [`super::MinimalTriangle`]. The material and `GetColor`/`GetUv` arrive with the renderer.

use std::any::Any;

use vector_math::{
    AxisAlignedBoundingBox, IntersectionType, Matrix4X4, Ray, Vector3, Vector3Float,
};

use super::triangle_hit::{triangle_surface_area, EdgeRule, TriangleProjection};
use super::{IntersectInfo, Traceable};

/// A triangle with f32 corners (C# `TriangleShape`).
#[derive(Debug, Clone, PartialEq)]
pub struct TriangleShape {
    projection: TriangleProjection,
    aabb_min_xyz: Vector3Float,
    aabb_max_xyz: Vector3Float,
    center: Vector3Float,
    vertices: [Vector3Float; 3],
    /// The face this came from, -1 when none (C# `index`, kept by the constructor's caller).
    pub index: i32,
}

impl TriangleShape {
    /// The triangle through three corners (C# `TriangleShape(Vector3, Vector3, Vector3, ...)`).
    pub fn new(vertex0: Vector3, vertex1: Vector3, vertex2: Vector3, index: i32) -> Self {
        let vertices = [
            Vector3Float::from_vector3(vertex0),
            Vector3Float::from_vector3(vertex1),
            Vector3Float::from_vector3(vertex2),
        ];
        let projection = TriangleProjection::new(vertex0, vertex1, vertex2, vertices);
        TriangleShape {
            projection,
            aabb_min_xyz: vertices[0]
                .component_min(vertices[1])
                .component_min(vertices[2]),
            aabb_max_xyz: vertices[0]
                .component_max(vertices[1])
                .component_max(vertices[2]),
            center: Vector3Float::from_vector3((vertex0 + vertex1 + vertex2) / 3.0),
            vertices,
            index,
        }
    }

    /// The triangle through three f32 corners.
    pub fn from_vector3_float(
        vertex0: Vector3Float,
        vertex1: Vector3Float,
        vertex2: Vector3Float,
        index: i32,
    ) -> Self {
        Self::new(
            vertex0.as_vector3(),
            vertex1.as_vector3(),
            vertex2.as_vector3(),
            index,
        )
    }

    /// The axis the triangle is projected along for the hit test (C# `MajorAxis`).
    pub fn major_axis(&self) -> u8 {
        self.projection.major_axis
    }

    /// The triangle's plane (C# `Plane`).
    pub fn plane(&self) -> vector_math::PlaneFloat {
        self.projection.plane
    }

    /// Corner `index` (C# `ITriangle.GetVertex`).
    pub fn get_vertex(&self, index: usize) -> Vector3 {
        Vector3::from_vector3_float(self.vertices[index])
    }
}

impl std::fmt::Display for TriangleShape {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Triangle {} {} {}",
            self.vertices[0], self.vertices[1], self.vertices[2]
        )
    }
}

impl Traceable for TriangleShape {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn children(&self) -> Option<Vec<&dyn Traceable>> {
        None
    }

    fn axis_to_world(&self) -> Matrix4X4 {
        // C# BaseShape throws here; a leaf is never asked (see divergence 141).
        Matrix4X4::IDENTITY
    }

    fn get_surface_area(&self) -> f64 {
        triangle_surface_area(&|i| self.vertices[i])
    }

    fn get_axis_aligned_bounding_box(&self) -> AxisAlignedBoundingBox {
        AxisAlignedBoundingBox::from_vector3_float(self.aabb_min_xyz, self.aabb_max_xyz)
    }

    fn get_center(&self) -> Vector3 {
        Vector3::from_vector3_float(self.center)
    }

    fn get_axis_center(&self, axis: usize) -> f64 {
        self.get_center()[axis]
    }

    fn get_contained<'a>(
        &'a self,
        results: &mut Vec<&'a dyn Traceable>,
        sub_region: &AxisAlignedBoundingBox,
    ) -> bool {
        if self.get_axis_aligned_bounding_box().contains(sub_region) {
            results.push(self);
            return true;
        }

        false
    }

    fn contains(&self, position: Vector3) -> bool {
        let distance_to_plane = self
            .projection
            .plane
            .get_distance_from_plane(Vector3Float::from_vector3(position));
        if f64::from(distance_to_plane.abs()) < 0.001 {
            return self
                .get_axis_aligned_bounding_box()
                .contains_point(position);
        }

        false
    }

    fn get_closest_intersection<'a>(&'a self, ray: &mut Ray) -> Option<IntersectInfo<'a>> {
        let (distance_to_hit, hit_position, normal_at_hit) =
            self.projection
                .hit(ray, &|i| self.vertices[i], EdgeRule::Strict)?;
        Some(IntersectInfo {
            hit_type: IntersectionType::FrontFace,
            closest_hit_object: Some(self),
            hit_position,
            normal_at_hit,
            distance_to_hit,
        })
    }

    fn get_intersect_cost(&self) -> f64 {
        350.0
    }
}
