//! The ray tracer's trace path: the [`Traceable`] trait (everything a ray can hit and a BVH
//! can hold), the hit record [`IntersectInfo`], the shapes MatterCAD picks with
//! ([`TriangleShape`], [`MinimalTriangle`], [`PlaneShape`]), the containers
//! ([`BoundingVolumeHierarchy`], [`UnboundCollection`], [`Transform`]), the Morton-code
//! builder [`BvhBuilderLocallyOrderedClustering`] behind
//! [`BoundingVolumeHierarchy::create_new_hierarchy`], and the tree walk [`BvhIterator`].
//!
//! Design: traceables are shared freely (MatterCAD caches a mesh's BVH and wraps it in a
//! fresh `Transform` per object), so nodes hold their children as [`TraceableRef`]
//! (`Arc<dyn Traceable>`), and every traceable is `Send + Sync` so a tree can be built on a
//! worker and traced from the UI. A hit borrows what it hit (`IntersectInfo<'a>` carries
//! `&'a dyn Traceable`) the way vector-math's `RayHitInfo<&T>` does; callers that need the
//! concrete leaf (picking reads the triangle's face index off a hit) downcast through
//! [`Traceable::as_any`]. The BVH narrows the ray in place as it finds hits, so tracing
//! takes `&mut Ray`.
//!
//! Materials and the [`Primitive`] trait (a shape with a material) live in `material`; the solid
//! shapes [`BoxShape`] and [`SphereShape`] and the view-space CSG node [`Difference`] report every
//! hit along a ray through [`Traceable::intersection_iterator`], which `Difference` combines with
//! [`IntersectInfo::subtract`] (in `intersect_info_subtract`).
//!
//! Queries a shape cannot answer (the triangles' and plane's `intersection_iterator`, and
//! `Transform`'s and `Difference`'s `get_crossing`/`get_touching`) return
//! [`RayTracerError::NotSupported`] rather than a wrong answer.
//!
//! [`RayTracer`] traces a primary ray into a [`Scene`]'s shapes.

mod base_shape;
mod bounding_volume_hierarchy;
mod box_shape;
mod bvh_builder_locally_ordered_clustering;
mod bvh_iterator;
mod difference;
mod intersect_info_subtract;
mod material;
mod minimal_triangle;
mod plane_shape;
mod ray_tracer_engine;
mod scene;
mod sphere_shape;
mod transform;
mod triangle_hit;
mod triangle_shape;
mod triangle_shape_uv;
mod unbound_collection;

use std::any::Any;
use std::sync::Arc;

use vector_math::ColorF;
use vector_math::{AxisAlignedBoundingBox, IntersectionType, Matrix4X4, Plane, Ray, Vector3};

pub use bounding_volume_hierarchy::{BoundingVolumeHierarchy, BvhCreationOptions};
pub use box_shape::BoxShape;
pub use bvh_builder_locally_ordered_clustering::BvhBuilderLocallyOrderedClustering;
pub use bvh_iterator::{BvhDescentFilter, BvhIterator};
pub use difference::Difference;
pub use intersect_info_subtract::compare_intersect_info_on_distance;
pub use material::{
    base_shape_default_material, wrap_up, MaterialAbstract, MaterialProperties, MaterialRef,
    Primitive, SolidMaterial,
};
pub use minimal_triangle::{MinimalTriangle, VertexFunc};
pub use plane_shape::PlaneShape;
pub use ray_tracer_engine::{AntiAliasing, RayTracer};
pub use scene::{Background, Scene};
pub use sphere_shape::SphereShape;
pub use transform::Transform;
pub use triangle_shape::TriangleShape;
pub use triangle_shape_uv::TriangleShapeUv;
pub use unbound_collection::UnboundCollection;

/// A shared traceable: one BVH can sit under several transforms.
pub type TraceableRef = Arc<dyn Traceable>;

/// Why a ray tracer query has no answer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RayTracerError {
    /// The named query is not supported by this shape.
    NotSupported(&'static str),
    /// A color was asked of a shape that has no material.
    NoMaterial,
    /// A hit list handed to [`IntersectInfo::subtract`] is malformed (the text says how).
    BadHitList(&'static str),
    /// A shape's construction arguments are invalid (the text says which).
    InvalidArgument(&'static str),
}

impl std::fmt::Display for RayTracerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RayTracerError::NotSupported(what) => write!(f, "{what} is not implemented"),
            RayTracerError::NoMaterial => write!(f, "the shape has no material"),
            RayTracerError::BadHitList(text) | RayTracerError::InvalidArgument(text) => {
                write!(f, "{text}")
            }
        }
    }
}

impl std::error::Error for RayTracerError {}

/// Something a ray can be traced against and a BVH can hold.
pub trait Traceable: Any + Send + Sync {
    /// This, for downcasting to the concrete type (a picked triangle's face index, say).
    fn as_any(&self) -> &dyn Any;

    /// The items this one is made of, `None` for a leaf.
    fn children(&self) -> Option<Vec<&dyn Traceable>>;

    /// The transform from this item's space to its parent's.
    /// Leaves report identity: only items with children are ever asked.
    fn axis_to_world(&self) -> Matrix4X4;

    /// The area of the actual surface this item defines.
    fn get_surface_area(&self) -> f64;

    /// The bounds of everything in this item.
    fn get_axis_aligned_bounding_box(&self) -> AxisAlignedBoundingBox;

    /// The center of the bounds.
    fn get_center(&self) -> Vector3;

    /// One coordinate of the center (0 = x, 1 = y, 2 = z).
    fn get_axis_center(&self, axis: usize) -> f64;

    /// Adds every leaf inside `sub_region` to `results`; true when one was found.
    fn get_contained<'a>(
        &'a self,
        results: &mut Vec<&'a dyn Traceable>,
        sub_region: &AxisAlignedBoundingBox,
    ) -> bool;

    /// True when `position` is on (or, for a container, on something in) this item.
    fn contains(&self, position: Vector3) -> bool;

    /// The nearest hit of `ray`, if any. Containers narrow the ray's
    /// `max_distance_to_consider` as they find hits.
    fn get_closest_intersection<'a>(&'a self, ray: &mut Ray) -> Option<IntersectInfo<'a>>;

    /// The relative cost of an intersection test against this (a bounds test is 1).
    fn get_intersect_cost(&self) -> f64;

    /// Every hit along `ray`, front and back faces as the ray's `intersection_type` asks, for
    /// view-space CSG. Containers concatenate their children's.
    fn intersection_iterator<'a>(
        &'a self,
        ray: &Ray,
    ) -> Result<Vec<IntersectInfo<'a>>, RayTracerError>;

    /// The items whose bounds the plane crosses: a leaf itself, a
    /// BVH node its children's, an [`UnboundCollection`] its direct items.
    fn get_crossing(&self, plane: &Plane) -> Result<Vec<&dyn Traceable>, RayTracerError>;

    /// The items whose bounds hold `position` within `error`.
    fn get_touching(
        &self,
        position: Vector3,
        error: f64,
    ) -> Result<Vec<&dyn Traceable>, RayTracerError>;

    /// This as a [`Primitive`] when it has a material.
    fn as_primitive(&self) -> Option<&dyn Primitive> {
        None
    }
}

/// The tolerance a plane-crosses-bounds test uses: bounds within this of the plane count
/// as crossed.
pub(crate) const PLANE_CROSSED_BY_ERROR: f64 = 0.001;

/// Everything about a ray-shape hit.
#[derive(Clone, Copy)]
pub struct IntersectInfo<'a> {
    pub hit_type: IntersectionType,
    pub closest_hit_object: Option<&'a dyn Traceable>,
    pub hit_position: Vector3,
    pub normal_at_hit: Vector3,
    pub distance_to_hit: f64,
    /// The shaded color the renderer stores, transparent black until set.
    pub total_color: ColorF,
}

impl<'a> IntersectInfo<'a> {
    /// No hit yet: distance `f64::MAX`, hit type `None`.
    pub fn new() -> Self {
        IntersectInfo {
            hit_type: IntersectionType::None,
            closest_hit_object: None,
            hit_position: Vector3::ZERO,
            normal_at_hit: Vector3::ZERO,
            distance_to_hit: f64::MAX,
            total_color: ColorF::new(0.0, 0.0, 0.0, 0.0),
        }
    }

    /// True when the hit object is `object` itself (the same instance, not an equal one).
    pub fn hit_object_is(&self, object: &dyn Traceable) -> bool {
        self.closest_hit_object
            .is_some_and(|hit| std::ptr::addr_eq(hit as *const dyn Traceable, object))
    }

    /// The hit object as a `T`, if it is one.
    pub fn closest_hit_as<T: Traceable>(&self) -> Option<&'a T> {
        self.closest_hit_object
            .and_then(|object| object.as_any().downcast_ref::<T>())
    }
}

impl Default for IntersectInfo<'_> {
    fn default() -> Self {
        Self::new()
    }
}

impl std::fmt::Debug for IntersectInfo<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("IntersectInfo")
            .field("hit_type", &self.hit_type)
            .field("has_hit_object", &self.closest_hit_object.is_some())
            .field("hit_position", &self.hit_position)
            .field("normal_at_hit", &self.normal_at_hit)
            .field("distance_to_hit", &self.distance_to_hit)
            .field("total_color", &self.total_color)
            .finish()
    }
}
