//! A triangle that keeps its own corners. MatterCAD builds these when a mesh is traced with a
//! material and in its trace tests; picking uses the lighter [`super::MinimalTriangle`]. `new`
//! builds one with no material; `with_material` takes one.

use std::any::Any;

use vector_math::{
    AxisAlignedBoundingBox, IntersectionType, Matrix4X4, Plane, Ray, Vector3, Vector3Float,
};

use super::base_shape::{leaf_get_contained, leaf_get_crossing, leaf_get_touching};
use super::triangle_hit::{triangle_surface_area, TriangleProjection};
use super::{IntersectInfo, MaterialRef, Primitive, RayTracerError, Traceable};

/// A triangle with f32 corners.
#[derive(Debug, Clone)]
pub struct TriangleShape {
    projection: TriangleProjection,
    aabb_min_xyz: Vector3Float,
    aabb_max_xyz: Vector3Float,
    center: Vector3Float,
    vertices: [Vector3Float; 3],
    /// The face this came from, -1 when none (the caller that builds it sets it).
    pub index: i32,
    material: Option<MaterialRef>,
}

impl PartialEq for TriangleShape {
    /// Same corners, face index and (the same shared) material.
    fn eq(&self, other: &Self) -> bool {
        self.projection == other.projection
            && self.aabb_min_xyz == other.aabb_min_xyz
            && self.aabb_max_xyz == other.aabb_max_xyz
            && self.center == other.center
            && self.vertices == other.vertices
            && self.index == other.index
            && match (&self.material, &other.material) {
                (Some(a), Some(b)) => std::sync::Arc::ptr_eq(a, b),
                (None, None) => true,
                _ => false,
            }
    }
}

impl TriangleShape {
    /// The triangle through three corners with no material.
    pub fn new(vertex0: Vector3, vertex1: Vector3, vertex2: Vector3, index: i32) -> Self {
        Self::with_material(vertex0, vertex1, vertex2, None, index)
    }

    /// The triangle through three corners with `material`.
    pub fn with_material(
        vertex0: Vector3,
        vertex1: Vector3,
        vertex2: Vector3,
        material: Option<MaterialRef>,
        index: i32,
    ) -> Self {
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
            material,
        }
    }

    /// Replaces the material.
    pub fn set_material(&mut self, material: Option<MaterialRef>) {
        self.material = material;
    }

    /// The texture coordinates of a hit: the hit position projected on two axes in the
    /// triangle's plane, computed in f32 like the corners.
    pub(crate) fn plane_uv(&self, info: &IntersectInfo<'_>) -> (f64, f64) {
        let normal = self.projection.plane.normal();
        let vec_u = Vector3Float::new_f32(normal.y, normal.z, -normal.x);
        let vec_v = vec_u.cross(normal);

        let u = Vector3Float::from_vector3(info.hit_position).dot(vec_u);
        let v = Vector3Float::from_vector3(info.hit_position).dot(vec_v);

        (f64::from(u), f64::from(v))
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

    /// The axis the triangle is projected along for the hit test.
    pub fn major_axis(&self) -> u8 {
        self.projection.major_axis
    }

    /// The triangle's plane.
    pub fn plane(&self) -> vector_math::PlaneFloat {
        self.projection.plane
    }

    /// Corner `index`.
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
        // A leaf is never asked: the tree walk reads this only on items with children.
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
        leaf_get_contained(self, results, sub_region)
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
            self.projection.hit(ray, &|i| self.vertices[i])?;
        Some(IntersectInfo {
            hit_type: IntersectionType::FrontFace,
            closest_hit_object: Some(self),
            hit_position,
            normal_at_hit,
            distance_to_hit,
            ..IntersectInfo::new()
        })
    }

    fn get_intersect_cost(&self) -> f64 {
        350.0
    }

    fn intersection_iterator<'a>(
        &'a self,
        _ray: &Ray,
    ) -> Result<Vec<IntersectInfo<'a>>, RayTracerError> {
        Err(RayTracerError::NotSupported(
            "TriangleShape.IntersectionIterator",
        ))
    }

    fn get_crossing(&self, plane: &Plane) -> Result<Vec<&dyn Traceable>, RayTracerError> {
        Ok(leaf_get_crossing(self, plane))
    }

    fn get_touching(
        &self,
        position: Vector3,
        error: f64,
    ) -> Result<Vec<&dyn Traceable>, RayTracerError> {
        Ok(leaf_get_touching(self, position, error))
    }

    fn as_primitive(&self) -> Option<&dyn Primitive> {
        Some(self)
    }
}

impl Primitive for TriangleShape {
    fn material(&self) -> Option<&MaterialRef> {
        self.material.as_ref()
    }

    fn get_uv(&self, info: &IntersectInfo<'_>) -> Result<(f64, f64), RayTracerError> {
        Ok(self.plane_uv(info))
    }
}
