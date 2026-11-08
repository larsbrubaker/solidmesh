//! A binary bounding-volume node over two traceables, and `create_new_hierarchy`, which picks
//! the builder. Tracing descends the nearer child first along the node's longest axis and narrows
//! the ray to the first hit, so the far child only tests what could be nearer.

use std::any::Any;

use solidmesh_math::{AxisAlignedBoundingBox, IntersectionType, Matrix4X4, Plane, Ray, Vector3};

use super::{
    BvhBuilderLocallyOrderedClustering, IntersectInfo, RayBundle, RayTracerError, Traceable,
    TraceableRef, UnboundCollection, PLANE_CROSSED_BY_ERROR,
};

/// How [`BoundingVolumeHierarchy::create_new_hierarchy`] builds the tree.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub enum BvhCreationOptions {
    /// Every item in one flat [`UnboundCollection`]: no build time, slow tracing.
    SingleUnboundCollection,
    /// The Morton-code builder, then a surface-area pass (MatterCAD's default).
    #[default]
    LocalOrderClustering,
}

/// A node of two children and their joint bounds.
pub struct BoundingVolumeHierarchy {
    pub(super) aabb: AxisAlignedBoundingBox,
    pub(super) left: TraceableRef,
    pub(super) right: TraceableRef,
    pub(super) splitting_plane: usize,
    /// The box has no NaN and its min is at most its max: what the packet traversal's skipped
    /// box tests rely on (`bvh_bundle_traversal`).
    pub(super) monotone: bool,
    /// The child is a sound node whose box is inside this one, so the packet traversal can
    /// walk down into it without testing each ray.
    pub(super) left_descends: bool,
    pub(super) right_descends: bool,
}

impl BoundingVolumeHierarchy {
    /// The node over `node_a` and `node_b`, split along its longest axis (z on ties with it,
    /// y on an x/y tie).
    pub fn new(node_a: TraceableRef, node_b: TraceableRef) -> Self {
        let left_bounds = node_a.get_axis_aligned_bounding_box();
        let right_bounds = node_b.get_axis_aligned_bounding_box();
        let aabb = left_bounds + right_bounds;
        let mut splitting_plane = if aabb.x_size() > aabb.y_size() { 0 } else { 1 };
        splitting_plane = if aabb.size()[splitting_plane] > aabb.z_size() {
            splitting_plane
        } else {
            2
        };
        let monotone = is_sound(&aabb);
        let left_descends = monotone && is_sound_node_inside(&node_a, &left_bounds, &aabb);
        let right_descends = monotone && is_sound_node_inside(&node_b, &right_bounds, &aabb);
        BoundingVolumeHierarchy {
            aabb,
            left: node_a,
            right: node_b,
            splitting_plane,
            monotone,
            left_descends,
            right_descends,
        }
    }

    pub fn left(&self) -> &TraceableRef {
        &self.left
    }

    pub fn right(&self) -> &TraceableRef {
        &self.right
    }

    /// The tree over `trace_primitives` built as `options` says.
    /// `None` only for an empty list built by `LocalOrderClustering`; a single item is
    /// returned as it is.
    pub fn create_new_hierarchy(
        trace_primitives: Vec<TraceableRef>,
        options: BvhCreationOptions,
    ) -> Option<TraceableRef> {
        match options {
            BvhCreationOptions::SingleUnboundCollection => Some(std::sync::Arc::new(
                UnboundCollection::new(trace_primitives),
            )),
            BvhCreationOptions::LocalOrderClustering => {
                BvhBuilderLocallyOrderedClustering::create(&trace_primitives)
            }
        }
    }

    /// The near and far children for a ray going `direction`.
    fn ordered_children(&self, direction: Vector3) -> (&TraceableRef, &TraceableRef) {
        if direction[self.splitting_plane] < 0.0 {
            (&self.right, &self.left)
        } else {
            (&self.left, &self.right)
        }
    }
}

/// True when the box has no NaN and is not inside out.
fn is_sound(bounds: &AxisAlignedBoundingBox) -> bool {
    (0..3).all(|axis| bounds.min_xyz[axis] <= bounds.max_xyz[axis])
}

/// True when `child` is a sound node whose bounds lie inside `parent`.
fn is_sound_node_inside(
    child: &TraceableRef,
    child_bounds: &AxisAlignedBoundingBox,
    parent: &AxisAlignedBoundingBox,
) -> bool {
    child
        .as_any()
        .downcast_ref::<BoundingVolumeHierarchy>()
        .is_some_and(|node| node.monotone)
        && (0..3).all(|axis| {
            parent.min_xyz[axis] <= child_bounds.min_xyz[axis]
                && child_bounds.max_xyz[axis] <= parent.max_xyz[axis]
        })
}

/// True when `info` is a real hit: present, with a hit type other than `None`.
fn is_hit(info: &Option<IntersectInfo<'_>>) -> bool {
    matches!(info, Some(hit) if hit.hit_type != IntersectionType::None)
}

impl Traceable for BoundingVolumeHierarchy {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn children(&self) -> Option<Vec<&dyn Traceable>> {
        Some(vec![self.left.as_ref(), self.right.as_ref()])
    }

    fn axis_to_world(&self) -> Matrix4X4 {
        Matrix4X4::IDENTITY
    }

    fn get_surface_area(&self) -> f64 {
        self.aabb.get_surface_area()
    }

    fn get_axis_aligned_bounding_box(&self) -> AxisAlignedBoundingBox {
        self.aabb
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
        if self.get_axis_aligned_bounding_box().contains(sub_region) {
            let result_a = self.left.get_contained(results, sub_region);
            let result_b = self.right.get_contained(results, sub_region);
            return result_a | result_b;
        }

        false
    }

    fn contains(&self, position: Vector3) -> bool {
        self.get_axis_aligned_bounding_box()
            .contains_point(position)
            && (self.left.contains(position) || self.right.contains(position))
    }

    fn get_closest_intersection<'a>(&'a self, ray: &mut Ray) -> Option<IntersectInfo<'a>> {
        if !ray.intersection(&self.aabb) {
            return None;
        }

        let (check_first, check_second) = self.ordered_children(ray.direction_normal);

        let info_first = check_first.get_closest_intersection(ray);
        if let Some(first) = info_first.filter(|_| is_hit(&info_first)) {
            if ray.is_shadow_ray {
                return info_first;
            }
            ray.max_distance_to_consider = first.distance_to_hit;
        }

        let info_second = check_second.get_closest_intersection(ray);
        if let Some(second) = info_second.filter(|_| is_hit(&info_second)) {
            if ray.is_shadow_ray {
                return info_second;
            }
            ray.max_distance_to_consider = second.distance_to_hit;
        }

        if let Some(first) =
            info_first.filter(|first| is_hit(&info_first) && first.distance_to_hit >= 0.0)
        {
            if let Some(second) = info_second {
                if is_hit(&info_second)
                    && second.distance_to_hit < first.distance_to_hit
                    && second.distance_to_hit >= 0.0
                {
                    return info_second;
                }
            }

            return info_first;
        }

        // we don't have to test it because it didn't hit.
        info_second
    }

    /// The packet traversal in `bvh_bundle_traversal`.
    fn trace_ray_bundle<'a>(
        &'a self,
        bundle: &mut RayBundle<'a>,
        active: &[u32],
    ) -> Result<(), RayTracerError> {
        self.trace_bundle_entry(bundle, active)
    }

    fn get_intersect_cost(&self) -> f64 {
        AxisAlignedBoundingBox::get_intersect_cost()
    }

    /// The near child's hits, then the far child's, dropping `None` hits as it goes.
    fn intersection_iterator<'a>(
        &'a self,
        ray: &Ray,
    ) -> Result<Vec<IntersectInfo<'a>>, RayTracerError> {
        let mut hits = Vec::new();
        if ray.intersection(&self.aabb) {
            let (check_first, check_second) = self.ordered_children(ray.direction_normal);
            for child in [check_first, check_second] {
                hits.extend(
                    child
                        .intersection_iterator(ray)?
                        .into_iter()
                        .filter(|info| info.hit_type != IntersectionType::None),
                );
            }
        }
        Ok(hits)
    }

    fn get_crossing(&self, plane: &Plane) -> Result<Vec<&dyn Traceable>, RayTracerError> {
        let mut crossing = Vec::new();
        if plane.crossed_by(
            &self.get_axis_aligned_bounding_box(),
            PLANE_CROSSED_BY_ERROR,
        ) {
            crossing.extend(self.left.get_crossing(plane)?);
            crossing.extend(self.right.get_crossing(plane)?);
        }
        Ok(crossing)
    }

    fn get_touching(
        &self,
        position: Vector3,
        error: f64,
    ) -> Result<Vec<&dyn Traceable>, RayTracerError> {
        let mut touching = Vec::new();
        if self
            .get_axis_aligned_bounding_box()
            .contains_point_within(position, error)
        {
            touching.extend(self.left.get_touching(position, error)?);
            touching.extend(self.right.get_touching(position, error)?);
        }
        Ok(touching)
    }
}
