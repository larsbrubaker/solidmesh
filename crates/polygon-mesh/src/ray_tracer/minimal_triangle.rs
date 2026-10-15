//! The memory-light triangle MatterCAD traces meshes with: it reads its corners back through a
//! callback on the mesh instead of storing them, and carries the index of the face it stands
//! for, which is how a pick recovers the face it hit.

use std::any::Any;
use std::sync::Arc;

use vector_math::{
    AxisAlignedBoundingBox, IntersectionType, Matrix4X4, Plane, Ray, Vector3, Vector3Float,
};

use super::base_shape::{leaf_get_crossing, leaf_get_touching};
use super::triangle_hit::{triangle_surface_area, TriangleProjection};
use super::{IntersectInfo, RayTracerError, Traceable};

/// The corner lookup: `(face_index, vertex_index 0..3) -> corner`.
pub type VertexFunc = Arc<dyn Fn(i32, i32) -> Vector3Float + Send + Sync>;

/// A triangle whose corners come from a callback.
#[derive(Clone)]
pub struct MinimalTriangle {
    projection: TriangleProjection,
    // The box is kept as its corners, not as a centre and half size: (min + max) / 2 and
    // (max - min) / 2 round in float, and centre - halfSize does not come back to min. A cap
    // triangle with one corner at x = -3.8e-15 and another at 0.98 got a box starting at
    // exactly 0, so a ray straight down x = 0 - inside the triangle - missed the box, and the
    // hierarchy never tested the triangle at all.
    aabb_min: Vector3Float,
    aabb_max: Vector3Float,
    /// The face this triangle stands for.
    pub face_index: i32,
    vertex_func: VertexFunc,
}

impl MinimalTriangle {
    pub fn new(vertex_func: VertexFunc, face_index: i32) -> Self {
        let vertex = |i: i32| vertex_func(face_index, i);
        let v0 = Vector3::from_vector3_float(vertex(0));
        let v1 = Vector3::from_vector3_float(vertex(1));
        let v2 = Vector3::from_vector3_float(vertex(2));
        let projection = TriangleProjection::new(v0, v1, v2, [vertex(0), vertex(1), vertex(2)]);
        let aabb_min = vertex(0).component_min(vertex(1)).component_min(vertex(2));
        let aabb_max = vertex(0).component_max(vertex(1)).component_max(vertex(2));
        MinimalTriangle {
            projection,
            aabb_min,
            aabb_max,
            face_index,
            vertex_func,
        }
    }

    /// The triangle's plane.
    pub fn plane(&self) -> vector_math::PlaneFloat {
        self.projection.plane
    }

    fn vertex(&self, i: usize) -> Vector3Float {
        (self.vertex_func)(self.face_index, i as i32)
    }

    fn center(&self) -> Vector3Float {
        (self.aabb_min + self.aabb_max) / 2.0
    }

    /// Corner `index`.
    pub fn get_vertex(&self, index: usize) -> Vector3 {
        Vector3::from_vector3_float(self.vertex(index))
    }
}

impl std::fmt::Debug for MinimalTriangle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MinimalTriangle")
            .field("face_index", &self.face_index)
            .field("aabb_min", &self.aabb_min)
            .field("aabb_max", &self.aabb_max)
            .finish()
    }
}

impl std::fmt::Display for MinimalTriangle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Triangle {} {} {}",
            self.vertex(0),
            self.vertex(1),
            self.vertex(2)
        )
    }
}

impl Traceable for MinimalTriangle {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn children(&self) -> Option<Vec<&dyn Traceable>> {
        None
    }

    fn axis_to_world(&self) -> Matrix4X4 {
        Matrix4X4::IDENTITY
    }

    fn get_surface_area(&self) -> f64 {
        triangle_surface_area(&|i| self.vertex(i))
    }

    fn get_axis_aligned_bounding_box(&self) -> AxisAlignedBoundingBox {
        AxisAlignedBoundingBox::from_vector3_float(self.aabb_min, self.aabb_max)
    }

    fn get_center(&self) -> Vector3 {
        Vector3::from_vector3_float(self.center())
    }

    fn get_axis_center(&self, axis: usize) -> f64 {
        let center = self.center();
        match axis {
            0 => f64::from(center.x),
            1 => f64::from(center.y),
            _ => f64::from(center.z),
        }
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
        if f64::from(distance_to_plane.abs()) < 0.001
            && self
                .get_axis_aligned_bounding_box()
                .contains_point(position)
        {
            return true;
        }

        false
    }

    fn get_closest_intersection<'a>(&'a self, ray: &mut Ray) -> Option<IntersectInfo<'a>> {
        let (distance_to_hit, hit_position, normal_at_hit) =
            self.projection.hit(ray, &|i| self.vertex(i))?;
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
            "MinimalTriangle.IntersectionIterator",
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
}
