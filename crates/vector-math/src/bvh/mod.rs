//! The bounding-volume hierarchy MatterCAD builds over mesh vertices and test geometry
//! (agg-sharp `VectorMath/Bvh`, after ChevyRay's MIT-licensed BVH): [`BvhTree`], its
//! surface-area-heuristic builder [`TradeOffBvhConstructor`], the Morton-code builder
//! [`BvhBuilderLocallyOrderedClustering`], and the hit record [`RayHitInfo`].
//!
//! Design: the C# `RayHitInfo.ClosestHitObject` is an `object` that holds a bounding box when
//! `Ray.GetClosestIntersection` fills it and the tree's item once `BvhTree` takes the hit over.
//! Here [`RayHitInfo`] is generic over what it carries — `RayHitInfo<AxisAlignedBoundingBox>`
//! from the ray, `RayHitInfo<&T>` from a `BvhTree<T>` — so callers get their item back typed
//! with no downcast. The tree is generic over its item (`BvhTree<T>`), with no bound: the C#
//! tree tests `item is IIntersectable` on its `BvhTreeItemData` wrapper, which never implements
//! it, so it always traces the item's box and never calls the item's own intersection. Only the
//! builder asks an item for its cost; [`Intersectable`] items use theirs through
//! [`TradeOffBvhConstructor::create_new_hierarchy_intersectable`], every other item costs one box
//! test, as in the C#.

mod bvh_builder_locally_ordered_clustering;
mod bvh_tree;
pub mod morton_codes;
mod trade_off_bvh_constructor;

pub use bvh_builder_locally_ordered_clustering::{BvhBuilderLocallyOrderedClustering, BvhNode};
pub use bvh_tree::BvhTree;
pub use trade_off_bvh_constructor::{SortingAccelerator, TradeOffBvhConstructor};

use crate::{AxisAlignedBoundingBox, IntersectionType, Ray, Vector3};

/// Something a ray can hit (C# `IIntersectable`).
pub trait Intersectable {
    /// What a hit reports as its `closest_hit_object`.
    type Hit;

    /// The computation cost of intersecting this, relative to a bounds test (which is 1).
    fn get_intersect_cost(&self) -> f64;

    /// The hit of `ray` on this, if any. The ray's distance window may be narrowed, as the C#
    /// tree does to its (reference-typed) ray.
    fn get_intersection(&self, ray: &mut Ray) -> Option<RayHitInfo<Self::Hit>>;
}

/// Where and how a ray hit something (C# `RayHitInfo`). `H` is what was hit.
#[derive(Debug, Clone, PartialEq)]
pub struct RayHitInfo<H> {
    pub closest_hit_object: Option<H>,
    pub distance_to_hit: f64,
    pub hit_position: Vector3,
    pub hit_type: IntersectionType,
    pub normal_at_hit: Vector3,
}

impl<H> RayHitInfo<H> {
    /// No hit yet: distance `f64::MAX` and hit type `None` (the C# `default(IntersectionType)`).
    pub fn new() -> Self {
        RayHitInfo {
            closest_hit_object: None,
            distance_to_hit: f64::MAX,
            hit_position: Vector3::ZERO,
            hit_type: IntersectionType::None,
            normal_at_hit: Vector3::ZERO,
        }
    }

    /// The same hit carrying `object` instead (the C# assignment to `ClosestHitObject`).
    pub fn with_object<U>(self, object: Option<U>) -> RayHitInfo<U> {
        RayHitInfo {
            closest_hit_object: object,
            distance_to_hit: self.distance_to_hit,
            hit_position: self.hit_position,
            hit_type: self.hit_type,
            normal_at_hit: self.normal_at_hit,
        }
    }
}

impl<H> Default for RayHitInfo<H> {
    fn default() -> Self {
        Self::new()
    }
}

/// An item and its bounds as a tree holds them (C# `BvhTreeItemData<T>`). `center` is not
/// derived from the bounds: the C# constructor leaves it zero, and the builder sorts by it.
#[derive(Debug, Clone, PartialEq)]
pub struct BvhTreeItemData<T> {
    pub aabb: AxisAlignedBoundingBox,
    pub center: Vector3,
    pub item: T,
}

impl<T> BvhTreeItemData<T> {
    pub fn new(item: T, bounds: AxisAlignedBoundingBox) -> Self {
        BvhTreeItemData {
            aabb: bounds,
            center: Vector3::ZERO,
            item,
        }
    }
}
