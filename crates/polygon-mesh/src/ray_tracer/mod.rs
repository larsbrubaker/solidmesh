//! The ray tracer's trace path (agg-sharp `PolygonMesh/RayTracer`): the [`Traceable`] trait
//! (C# `ITraceable` together with the `IBvhItem` members it inherits), the hit record
//! [`IntersectInfo`], the shapes MatterCAD picks with ([`TriangleShape`], [`MinimalTriangle`],
//! [`PlaneShape`]), the containers ([`BoundingVolumeHierarchy`], [`UnboundCollection`],
//! [`Transform`]), the Morton-code builder [`BvhBuilderLocallyOrderedClustering`] behind
//! [`BoundingVolumeHierarchy::create_new_hierarchy`], and the tree walk [`BvhIterator`]
//! (C# `PolygonMesh/Processors/BvhIterator.cs`).
//!
//! Design: C# shares `ITraceable` references freely (MatterCAD caches a mesh's BVH and wraps
//! it in a fresh `Transform` per object), so nodes hold their children as [`TraceableRef`]
//! (`Arc<dyn Traceable>`), and every traceable is `Send + Sync` so a tree can be built on a
//! worker and traced from the UI. A hit borrows what it hit (`IntersectInfo<'a>` carries
//! `&'a dyn Traceable`) the way vector-math's `RayHitInfo<&T>` does; callers that need the
//! concrete leaf (MatterCAD reads `MinimalTriangle.FaceIndex` off a hit) downcast through
//! [`Traceable::as_any`]. The C# `Ray` is a class the BVH narrows in place as it finds
//! hits, so tracing takes `&mut Ray`.
//!
//! Not here yet (later 2d work): materials and colors (`IPrimitive.Material`, `GetColor`,
//! `GetUv`, which need `ColorF`), ray bundles (`FindFirstRay`, `GetClosestIntersections`),
//! `IntersectionIterator` and the CSG `Difference`/`IntersectInfo.Subtract`, `GetCrossing` and
//! `GetTouching`, the other builders and shapes, and the image renderer.

mod bounding_volume_hierarchy;
mod bvh_builder_locally_ordered_clustering;
mod bvh_iterator;
mod minimal_triangle;
mod plane_shape;
mod transform;
mod triangle_hit;
mod triangle_shape;
mod unbound_collection;

use std::any::Any;
use std::sync::Arc;

use vector_math::{AxisAlignedBoundingBox, IntersectionType, Matrix4X4, Ray, Vector3};

pub use bounding_volume_hierarchy::{BoundingVolumeHierarchy, BvhCreationOptions};
pub use bvh_builder_locally_ordered_clustering::BvhBuilderLocallyOrderedClustering;
pub use bvh_iterator::{BvhDescentFilter, BvhIterator};
pub use minimal_triangle::{MinimalTriangle, VertexFunc};
pub use plane_shape::PlaneShape;
pub use transform::Transform;
pub use triangle_shape::TriangleShape;
pub use unbound_collection::UnboundCollection;

/// A shared traceable, as C# shares `ITraceable` references.
pub type TraceableRef = Arc<dyn Traceable>;

/// Something a ray can be traced against and a BVH can hold (C# `ITraceable : IBvhItem`).
pub trait Traceable: Any + Send + Sync {
    /// This, for downcasting to the concrete type (the C# `is` tests).
    fn as_any(&self) -> &dyn Any;

    /// The items this one is made of, `None` for a leaf (C# `IBvhItem.Children`).
    fn children(&self) -> Option<Vec<&dyn Traceable>>;

    /// The transform from this item's space to its parent's (C# `IBvhItem.AxisToWorld`).
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
}

/// Everything about a ray-shape hit (C# `IntersectInfo`).
#[derive(Clone, Copy)]
pub struct IntersectInfo<'a> {
    pub hit_type: IntersectionType,
    pub closest_hit_object: Option<&'a dyn Traceable>,
    pub hit_position: Vector3,
    pub normal_at_hit: Vector3,
    pub distance_to_hit: f64,
}

impl<'a> IntersectInfo<'a> {
    /// No hit yet: distance `f64::MAX`, hit type `None` (C# `default(IntersectionType)`).
    pub fn new() -> Self {
        IntersectInfo {
            hit_type: IntersectionType::None,
            closest_hit_object: None,
            hit_position: Vector3::ZERO,
            normal_at_hit: Vector3::ZERO,
            distance_to_hit: f64::MAX,
        }
    }

    /// The hit object as a `T`, if it is one (C# `info.ClosestHitObject is T t`).
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
            .finish()
    }
}
