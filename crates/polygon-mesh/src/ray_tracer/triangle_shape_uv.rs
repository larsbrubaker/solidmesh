//! A [`TriangleShape`] that carries per-corner texture coordinates. It keeps the three
//! coordinates, but its `get_uv` is the same plane projection as `TriangleShape`'s and does
//! not interpolate them.

use std::any::Any;

use vector_math::{
    AxisAlignedBoundingBox, Matrix4X4, Plane, Ray, Vector2Float, Vector3, Vector3Float,
};

use super::{IntersectInfo, MaterialRef, Primitive, RayTracerError, Traceable, TriangleShape};

/// A triangle with texture coordinates.
#[derive(Debug, Clone, PartialEq)]
pub struct TriangleShapeUv {
    triangle: TriangleShape,
    pub uv0: Vector2Float,
    pub uv1: Vector2Float,
    pub uv2: Vector2Float,
}

impl TriangleShapeUv {
    /// The triangle through three corners with a texture coordinate per corner, a material
    /// and the index of the face it came from.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        vertex0: Vector3,
        vertex1: Vector3,
        vertex2: Vector3,
        uv0: Vector2Float,
        uv1: Vector2Float,
        uv2: Vector2Float,
        material: Option<MaterialRef>,
        index: i32,
    ) -> Self {
        TriangleShapeUv {
            triangle: TriangleShape::with_material(vertex0, vertex1, vertex2, material, index),
            uv0,
            uv1,
            uv2,
        }
    }

    /// The `Vector3Float` overload.
    #[allow(clippy::too_many_arguments)]
    pub fn from_vector3_float(
        vertex0: Vector3Float,
        vertex1: Vector3Float,
        vertex2: Vector3Float,
        uv0: Vector2Float,
        uv1: Vector2Float,
        uv2: Vector2Float,
        material: Option<MaterialRef>,
        index: i32,
    ) -> Self {
        Self::new(
            vertex0.as_vector3(),
            vertex1.as_vector3(),
            vertex2.as_vector3(),
            uv0,
            uv1,
            uv2,
            material,
            index,
        )
    }

    /// The triangle part.
    pub fn triangle(&self) -> &TriangleShape {
        &self.triangle
    }

    /// Mutable access to the triangle part (for `set_material`).
    pub fn triangle_mut(&mut self) -> &mut TriangleShape {
        &mut self.triangle
    }

    /// Corner `index`.
    pub fn get_vertex(&self, index: usize) -> Vector3 {
        self.triangle.get_vertex(index)
    }
}

impl std::fmt::Display for TriangleShapeUv {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.triangle.fmt(f)
    }
}

/// Every `IBvhItem`/`ITraceable` member is the triangle's, with this shape as the hit object.
impl Traceable for TriangleShapeUv {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn children(&self) -> Option<Vec<&dyn Traceable>> {
        None
    }

    fn axis_to_world(&self) -> Matrix4X4 {
        self.triangle.axis_to_world()
    }

    fn get_surface_area(&self) -> f64 {
        self.triangle.get_surface_area()
    }

    fn get_axis_aligned_bounding_box(&self) -> AxisAlignedBoundingBox {
        self.triangle.get_axis_aligned_bounding_box()
    }

    fn get_center(&self) -> Vector3 {
        self.triangle.get_center()
    }

    fn get_axis_center(&self, axis: usize) -> f64 {
        self.triangle.get_axis_center(axis)
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
        self.triangle.contains(position)
    }

    fn get_closest_intersection<'a>(&'a self, ray: &mut Ray) -> Option<IntersectInfo<'a>> {
        let mut info = self.triangle.get_closest_intersection(ray)?;
        info.closest_hit_object = Some(self);
        Some(info)
    }

    fn get_intersect_cost(&self) -> f64 {
        self.triangle.get_intersect_cost()
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
        Ok(if self.triangle.get_crossing(plane)?.is_empty() {
            Vec::new()
        } else {
            vec![self]
        })
    }

    fn get_touching(
        &self,
        position: Vector3,
        error: f64,
    ) -> Result<Vec<&dyn Traceable>, RayTracerError> {
        Ok(if self.triangle.get_touching(position, error)?.is_empty() {
            Vec::new()
        } else {
            vec![self]
        })
    }

    fn as_primitive(&self) -> Option<&dyn Primitive> {
        Some(self)
    }
}

impl Primitive for TriangleShapeUv {
    fn material(&self) -> Option<&MaterialRef> {
        self.triangle.material()
    }

    fn get_uv(&self, info: &IntersectInfo<'_>) -> Result<(f64, f64), RayTracerError> {
        Ok(self.triangle.plane_uv(info))
    }
}
