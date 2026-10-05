//! A flat list of traceables tested one after another (agg-sharp
//! `RayTracer/Primitive/UnboundCollection.cs`): the `SingleUnboundCollection` hierarchy, and
//! what MatterCAD groups its bevel pick surfaces in.
//!
//! The C# computes its bounds lazily on first request and caches them (recomputing while they
//! are still infinite); the items cannot change here after construction, so the same
//! computation runs once, in the constructor, unless it is still infinite.

use std::any::Any;

use vector_math::{AxisAlignedBoundingBox, IntersectionType, Matrix4X4, Ray, Vector3};

use super::{IntersectInfo, Traceable, TraceableRef};

/// A list of traceables with no hierarchy (C# `UnboundCollection`).
pub struct UnboundCollection {
    items: Vec<TraceableRef>,
    cached_aabb: AxisAlignedBoundingBox,
}

/// The union of the items' bounds, skipping items whose min x is infinite or NaN; (-inf, -inf)
/// when there are none.
fn compute_aabb(items: &[TraceableRef]) -> AxisAlignedBoundingBox {
    let mut cached_aabb =
        AxisAlignedBoundingBox::new(Vector3::NEGATIVE_INFINITY, Vector3::NEGATIVE_INFINITY);
    let mut first = true;
    for item in items {
        let item_aabb = item.get_axis_aligned_bounding_box();
        if !item_aabb.min_xyz.x.is_infinite() && !item_aabb.min_xyz.x.is_nan() {
            if first {
                first = false;
                cached_aabb = item_aabb;
            } else {
                cached_aabb = cached_aabb + item_aabb;
            }
        }
    }

    cached_aabb
}

impl UnboundCollection {
    pub fn new(traceable_items: Vec<TraceableRef>) -> Self {
        let cached_aabb = compute_aabb(&traceable_items);
        UnboundCollection {
            items: traceable_items,
            cached_aabb,
        }
    }

    /// The items, in the order they are traced (C# `Items`).
    pub fn items(&self) -> &[TraceableRef] {
        &self.items
    }
}

impl Traceable for UnboundCollection {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn children(&self) -> Option<Vec<&dyn Traceable>> {
        Some(self.items.iter().map(|item| item.as_ref()).collect())
    }

    fn axis_to_world(&self) -> Matrix4X4 {
        Matrix4X4::IDENTITY
    }

    fn get_surface_area(&self) -> f64 {
        let mut total_surface_area = 0.0;
        for item in &self.items {
            total_surface_area += item.get_surface_area();
        }

        total_surface_area
    }

    fn get_axis_aligned_bounding_box(&self) -> AxisAlignedBoundingBox {
        let min_x = self.cached_aabb.min_xyz.x;
        if (min_x.is_infinite() || min_x.is_nan()) && !self.items.is_empty() {
            return compute_aabb(&self.items);
        }

        self.cached_aabb
    }

    fn get_center(&self) -> Vector3 {
        self.get_axis_aligned_bounding_box().get_center()
    }

    fn get_axis_center(&self, axis: usize) -> f64 {
        self.get_center()[axis]
    }

    fn get_contained<'a>(
        &'a self,
        results: &mut Vec<&'a dyn Traceable>,
        sub_region: &AxisAlignedBoundingBox,
    ) -> bool {
        let mut found_item = false;
        for item in &self.items {
            found_item |= item.get_contained(results, sub_region);
        }

        found_item
    }

    fn contains(&self, position: Vector3) -> bool {
        if self
            .get_axis_aligned_bounding_box()
            .contains_point(position)
        {
            for item in &self.items {
                if item.contains(position) {
                    return true;
                }
            }
        }

        false
    }

    fn get_closest_intersection<'a>(&'a self, ray: &mut Ray) -> Option<IntersectInfo<'a>> {
        let mut best_info: Option<IntersectInfo<'a>> = None;
        for item in &self.items {
            if let Some(info) = item.get_closest_intersection(ray) {
                if info.hit_type != IntersectionType::None && info.distance_to_hit >= 0.0 {
                    let better = match &best_info {
                        None => true,
                        Some(best) => info.distance_to_hit < best.distance_to_hit,
                    };
                    if better {
                        best_info = Some(info);
                    }
                }
            }
        }

        best_info
    }

    /// The sum of the items' costs.
    fn get_intersect_cost(&self) -> f64 {
        let mut total_intersect_cost = 0.0;
        for item in &self.items {
            total_intersect_cost += item.get_intersect_cost();
        }

        total_intersect_cost
    }
}
