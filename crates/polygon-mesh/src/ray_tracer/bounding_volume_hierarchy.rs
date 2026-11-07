//! A binary bounding-volume node over two traceables, and `create_new_hierarchy`, which picks
//! the builder. Tracing descends the nearer child first along the node's longest axis and narrows
//! the ray to the first hit, so the far child only tests what could be nearer.

use std::any::Any;

use vector_math::{AxisAlignedBoundingBox, IntersectionType, Matrix4X4, Plane, Ray, Vector3};

use super::{
    BvhBuilderLocallyOrderedClustering, IntersectInfo, RayTracerError, Traceable, TraceableRef,
    UnboundCollection, PLANE_CROSSED_BY_ERROR,
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
    aabb: AxisAlignedBoundingBox,
    left: TraceableRef,
    right: TraceableRef,
    splitting_plane: usize,
}

impl BoundingVolumeHierarchy {
    /// The node over `node_a` and `node_b`, split along its longest axis (z on ties with it,
    /// y on an x/y tie).
    pub fn new(node_a: TraceableRef, node_b: TraceableRef) -> Self {
        let aabb = node_a.get_axis_aligned_bounding_box() + node_b.get_axis_aligned_bounding_box();
        let mut splitting_plane = if aabb.x_size() > aabb.y_size() { 0 } else { 1 };
        splitting_plane = if aabb.size()[splitting_plane] > aabb.z_size() {
            splitting_plane
        } else {
            2
        };
        BoundingVolumeHierarchy {
            aabb,
            left: node_a,
            right: node_b,
            splitting_plane,
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
