//! `BvhTree<T>`: a node of the hierarchy — a leaf holding items, or a branch holding two child
//! trees split on an axis (agg-sharp `VectorMath/Bvh/BvhTree.cs`).
//!
//! Mapping notes: the C# tree is built by its constructors and never changed after, so the
//! branch's cached bounds stay right; here a leaf is [`BvhTree::from_items`] and a branch
//! [`BvhTree::from_nodes`]. Searches push references to the items into `results` where the C#
//! adds the items to a `List<T>`. The C# `Count` property is never assigned (always 0) and
//! the parameterless constructor builds a tree with no bounds that nothing uses; neither is
//! ported. `All` visits the children only when present: the C# dereferences them
//! unconditionally and so throws on every tree (divergence 31).

use super::{BvhTreeItemData, Intersectable, RayHitInfo};
use crate::{AxisAlignedBoundingBox, IntersectionType, Ray, Vector3};

/// A bounding-volume hierarchy node (C# `BvhTree<T>`).
#[derive(Debug, Clone, PartialEq)]
pub struct BvhTree<T> {
    /// any of the items that are in this node
    pub items: Vec<BvhTreeItemData<T>>,
    node_a: Option<Box<BvhTree<T>>>,
    node_b: Option<Box<BvhTree<T>>>,
    splitting_plane: usize,
    aabb: AxisAlignedBoundingBox,
    center: Vector3,
}

impl<T> BvhTree<T> {
    /// A leaf holding `items`, bounded by their union. Its `center` stays zero, as in the C#.
    pub fn from_items(items: impl IntoIterator<Item = BvhTreeItemData<T>>) -> Self {
        let mut aabb = AxisAlignedBoundingBox::empty();
        let mut leaf_items = Vec::new();
        for item in items {
            aabb = aabb + item.aabb;
            leaf_items.push(item);
        }

        BvhTree {
            items: leaf_items,
            node_a: None,
            node_b: None,
            splitting_plane: 0,
            aabb,
            center: Vector3::ZERO,
        }
    }

    /// A branch over two subtrees split on `splitting_plane` (0 = x, 1 = y, 2 = z).
    pub fn from_nodes(node_a: BvhTree<T>, node_b: BvhTree<T>, splitting_plane: usize) -> Self {
        // we can cache this because it is not allowed to change.
        let aabb = node_a.aabb + node_b.aabb;
        BvhTree {
            items: Vec::new(),
            node_a: Some(Box::new(node_a)),
            node_b: Some(Box::new(node_b)),
            splitting_plane,
            aabb,
            center: aabb.center(),
        }
    }

    pub fn aabb(&self) -> AxisAlignedBoundingBox {
        self.aabb
    }

    pub fn center(&self) -> Vector3 {
        self.center
    }

    /// The two children of a branch (`None` for a leaf).
    pub fn nodes(&self) -> (Option<&BvhTree<T>>, Option<&BvhTree<T>>) {
        (self.node_a.as_deref(), self.node_b.as_deref())
    }

    pub fn splitting_plane(&self) -> usize {
        self.splitting_plane
    }

    /// Every item, this node's first and then each child's.
    pub fn all<'a>(&'a self, results: &mut Vec<&'a T>) {
        // check all the items that are part of this object
        for item in &self.items {
            results.push(&item.item);
        }

        if let Some(node_a) = &self.node_a {
            node_a.all(results);
        }

        if let Some(node_b) = &self.node_b {
            node_b.all(results);
        }
    }

    /// The items whose bounds the ray passes through.
    pub fn along_ray<'a>(&'a self, ray: &Ray, results: &mut Vec<&'a T>) {
        if ray.intersection(&self.aabb) {
            // check all the items that are part of this object
            for item in &self.items {
                if ray.intersection(&item.aabb) {
                    results.push(&item.item);
                }
            }

            if let Some(node_a) = &self.node_a {
                node_a.along_ray(ray, results);
            }

            if let Some(node_b) = &self.node_b {
                node_b.along_ray(ray, results);
            }
        }
    }

    /// This node and every node below it.
    pub fn count_branches(&self) -> usize {
        let mut count = 1;
        if let Some(node_a) = &self.node_a {
            count += node_a.count_branches();
        }

        if let Some(node_b) = &self.node_b {
            count += node_b.count_branches();
        }

        count
    }

    /// The nearest item box the ray hits, carrying that item; `None` when nothing is hit. Each
    /// hit narrows `ray.max_distance_to_consider`, as the C# does to its shared ray; a shadow
    /// ray returns the first hit found.
    pub fn get_closest_intersection<'a>(&'a self, ray: &mut Ray) -> Option<RayHitInfo<&'a T>> {
        let mut best_intersect: Option<RayHitInfo<&'a T>> = None;
        if ray.intersection(&self.aabb) {
            // check all the items that are part of this object
            for item in &self.items {
                // we will just be hitting the bounding box of the type
                let info = ray.get_closest_intersection(&item.aabb);
                if info.hit_type != IntersectionType::None && info.distance_to_hit >= 0.0 {
                    let info = info.with_object(Some(&item.item));
                    if ray.is_shadow_ray {
                        return Some(info);
                    }

                    Self::keep_closer(&mut best_intersect, info, ray);
                }
            }

            let mut check_first = &self.node_a;
            let mut check_second = &self.node_b;
            if ray.direction_normal[self.splitting_plane] < 0.0 {
                check_first = &self.node_b;
                check_second = &self.node_a;
            }

            for check in [check_first, check_second].into_iter().flatten() {
                if let Some(intersect) = check.get_closest_intersection(ray) {
                    if intersect.hit_type != IntersectionType::None {
                        if ray.is_shadow_ray {
                            return Some(intersect);
                        }

                        Self::keep_closer(&mut best_intersect, intersect, ray);
                    }
                }
            }
        }

        best_intersect
    }

    fn keep_closer<'a>(
        best_intersect: &mut Option<RayHitInfo<&'a T>>,
        info: RayHitInfo<&'a T>,
        ray: &mut Ray,
    ) {
        let closer = match best_intersect {
            None => true,
            Some(best) => info.distance_to_hit < best.distance_to_hit,
        };
        if closer {
            ray.max_distance_to_consider = info.distance_to_hit;
            *best_intersect = Some(info);
        }
    }

    /// The items whose bounds touch `bounds`, found depth-first with an explicit stack (the
    /// second child is searched before the first, as in the C#).
    pub fn search_bounds<'a>(&'a self, bounds: &AxisAlignedBoundingBox, results: &mut Vec<&'a T>) {
        let mut stack: Vec<&BvhTree<T>> = vec![self];
        while let Some(current_node) = stack.pop() {
            if bounds.intersects(&current_node.aabb) {
                // check all the items that are part of this object
                for item in &current_node.items {
                    if item.aabb.intersects(bounds) {
                        results.push(&item.item);
                    }
                }

                if let Some(node_a) = &current_node.node_a {
                    stack.push(node_a);
                }

                if let Some(node_b) = &current_node.node_b {
                    stack.push(node_b);
                }
            }
        }
    }

    /// [`Self::search_bounds`] over the box from `(x, y, z)` to `(x + x_size, ...)`.
    #[allow(clippy::too_many_arguments)]
    pub fn search_bounds_xyz<'a>(
        &'a self,
        x: f64,
        y: f64,
        z: f64,
        x_size: f64,
        y_size: f64,
        z_size: f64,
        results: &mut Vec<&'a T>,
    ) {
        self.search_bounds(
            &AxisAlignedBoundingBox::new(
                Vector3::new(x, y, z),
                Vector3::new(x + x_size, y + y_size, z + z_size),
            ),
            results,
        );
    }

    /// The items whose bounds contain `position` (within the C# `Contains` tolerance of 0.001).
    pub fn search_point<'a>(&'a self, position: Vector3, results: &mut Vec<&'a T>) {
        if self.aabb.contains_point(position) {
            // check all the items that are part of this object
            for item in &self.items {
                if item.aabb.contains_point(position) {
                    results.push(&item.item);
                }
            }

            if let Some(node_a) = &self.node_a {
                node_a.search_point(position, results);
            }

            if let Some(node_b) = &self.node_b {
                node_b.search_point(position, results);
            }
        }
    }

    pub fn search_point_xyz<'a>(&'a self, x: f64, y: f64, z: f64, results: &mut Vec<&'a T>) {
        self.search_point(Vector3::new(x, y, z), results);
    }
}

/// A tree is itself intersectable, so trees can nest (C# `BvhTree<T> : IIntersectable`); the
/// hit carries a clone of the item.
impl<T: Clone> Intersectable for BvhTree<T> {
    type Hit = T;

    fn get_intersect_cost(&self) -> f64 {
        AxisAlignedBoundingBox::get_intersect_cost()
    }

    fn get_intersection(&self, ray: &mut Ray) -> Option<RayHitInfo<T>> {
        self.get_closest_intersection(ray).map(|info| {
            let object = info.closest_hit_object.cloned();
            info.with_object(object)
        })
    }
}
