//! Parts under a placement: [`PlacedPart`] is one part of a scene, a mesh tree and its
//! [`MeshSurface`] under a matrix, and [`PlacedParts`] traces rays through many of them, each
//! reporting its own surface even when they share a tree.
//!
//! A hit from [`Transform`] comes back in world space with the leaf triangle inside it, and
//! that is all it can say: the same tree under several transforms hands back the same leaf, so
//! the triangle cannot tell a tracer which part it was. `PlacedParts` walks its own top-level
//! tree over the parts instead, so the part that won is known at the leaf, with no extra work
//! per ray and without another trace to find the face. A [`PartHit`] holds both, and
//! [`PartHit::surface_point`] gives the world-space surface there.
//!
//! **Placement.** The hit position goes back into the mesh's space to find the face's weights,
//! and the normals come out by the inverse transpose of the part's matrix, which keeps them
//! perpendicular to the surface under a non-uniform scale and on the outside of a solid under a
//! mirror. A mirror reverses every face's winding, so a cross product taken on world corners
//! would point inward; these normals are computed in the mesh's space and transformed, so they
//! stay outward with no flip. A hit reports the distance the world ray travelled, whatever the
//! scale, because `Ray::transform` leaves the mesh-space direction unnormalized.
//!
//! **Continuing rays.** The ray offset is built in the mesh's space, where the triangle test
//! rounds, and carried out through the matrix, so it follows the part's own scale along the
//! normal and not where the part sits; `surface_point`'s header says why. The part's world
//! position only matters once `f64` rounding of the world-to-mesh transform reaches that size,
//! which the offset then also covers.

use std::any::Any;
use std::sync::Arc;

use solidmesh_math::{AxisAlignedBoundingBox, IntersectionType, Matrix4X4, Plane, Ray, Vector3};

use super::surface_point::{l1_magnitude, SurfacePoint, WORLD_OFFSET_F64_STEPS};
use super::surface_table::{unit_or, MeshSurface};
use super::{
    BoundingVolumeHierarchy, BvhCreationOptions, IntersectInfo, MinimalTriangle, RayBundle,
    RayTracerError, Traceable, TraceableRef, Transform,
};

/// One part: a mesh's tree and surface table under a matrix. Tracing a part alone gives what
/// [`Transform`] does; to know which part was hit among many, trace through [`PlacedParts`].
pub struct PlacedPart {
    transform: Transform,
    surface: Arc<MeshSurface>,
    world_bounds: AxisAlignedBoundingBox,
    index: usize,
}

impl PlacedPart {
    /// `tree` (built over the same mesh as `surface`, its leaves [`MinimalTriangle`]s) placed by
    /// `matrix`, which is the whole chain from the mesh to the world. A matrix that is not
    /// finite or cannot be inverted is a [`RayTracerError::InvalidArgument`]: such a part cannot
    /// be traced at all. So is a tree whose bounds differ from the corners in `surface`: every
    /// hit on it would be answered from another mesh's faces, with nothing to show for it.
    pub fn new(
        tree: TraceableRef,
        surface: Arc<MeshSurface>,
        matrix: Matrix4X4,
    ) -> Result<Self, RayTracerError> {
        let determinant = matrix.determinant();
        if !matrix.is_valid() || !determinant.is_finite() || determinant == 0.0 {
            return Err(RayTracerError::InvalidArgument(
                "the part's matrix has no inverse, so the part cannot be placed",
            ));
        }
        if !surface.matches_tree_bounds(&tree.get_axis_aligned_bounding_box()) {
            return Err(RayTracerError::InvalidArgument(
                "the tree was not built from the surface table's mesh: their bounds differ",
            ));
        }
        let transform = Transform::with_matrix(tree, matrix);
        let world_bounds = transform.get_axis_aligned_bounding_box();
        Ok(PlacedPart {
            transform,
            surface,
            world_bounds,
            index: 0,
        })
    }

    /// The part a [`Transform`] node describes, however many nested `Transform`s sit between it
    /// and the mesh's tree: the matrices are composed from the mesh outward, the same chain a
    /// hit travels.
    pub fn from_transform(
        transform: &Transform,
        surface: Arc<MeshSurface>,
    ) -> Result<Self, RayTracerError> {
        let mut matrix = transform.axis_to_world();
        let mut tree = transform.child().clone();
        while let Some(inner) = tree.as_any().downcast_ref::<Transform>() {
            matrix = Matrix4X4::mult(inner.axis_to_world(), matrix);
            let next = inner.child().clone();
            tree = next;
        }
        PlacedPart::new(tree, surface, matrix)
    }

    /// The mesh's surface table.
    pub fn surface(&self) -> &Arc<MeshSurface> {
        &self.surface
    }

    /// The matrix from the mesh's space to the world.
    pub fn mesh_to_world(&self) -> Matrix4X4 {
        self.transform.axis_to_world()
    }

    /// The matrix from the world to the mesh's space.
    pub fn world_to_mesh(&self) -> Matrix4X4 {
        self.transform.world_to_axis()
    }

    /// This part's position in the [`PlacedParts`] that holds it, 0 otherwise.
    pub fn index(&self) -> usize {
        self.index
    }

    /// The world-space surface point of face `face_index` at `world_hit_position` (see the
    /// module header for how the placement is applied).
    pub fn surface_point(
        &self,
        face_index: i32,
        world_hit_position: Vector3,
    ) -> Result<SurfacePoint, RayTracerError> {
        let mesh_to_world = self.mesh_to_world();
        let world_to_mesh = self.world_to_mesh();
        let local = self.surface.surface_point(
            face_index,
            world_hit_position.transform_position(world_to_mesh),
        )?;

        let position = local.position.transform_position(mesh_to_world);
        let geometric_normal = unit_or(
            local
                .geometric_normal
                .transform_normal_inverse(world_to_mesh),
            local.geometric_normal,
        );
        let shading_normal = unit_or(
            local.shading_normal.transform_normal_inverse(world_to_mesh),
            geometric_normal,
        );
        let ray_offset = local.ray_offset.transform_vector(mesh_to_world)
            + geometric_normal * (l1_magnitude(position) * f64::EPSILON * WORLD_OFFSET_F64_STEPS);

        Ok(SurfacePoint {
            position,
            geometric_normal,
            shading_normal,
            ray_offset,
            ..local
        })
    }
}

impl std::fmt::Debug for PlacedPart {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PlacedPart")
            .field("index", &self.index)
            .field("mesh_to_world", &self.mesh_to_world())
            .field("faces", &self.surface.face_count())
            .finish()
    }
}

impl Traceable for PlacedPart {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn children(&self) -> Option<Vec<&dyn Traceable>> {
        self.transform.children()
    }

    fn axis_to_world(&self) -> Matrix4X4 {
        self.transform.axis_to_world()
    }

    fn get_surface_area(&self) -> f64 {
        self.transform.get_surface_area()
    }

    fn get_axis_aligned_bounding_box(&self) -> AxisAlignedBoundingBox {
        self.world_bounds
    }

    fn get_center(&self) -> Vector3 {
        self.world_bounds.get_center()
    }

    fn get_axis_center(&self, axis: usize) -> f64 {
        self.get_center()[axis]
    }

    fn get_contained<'a>(
        &'a self,
        results: &mut Vec<&'a dyn Traceable>,
        sub_region: &AxisAlignedBoundingBox,
    ) -> bool {
        self.transform.get_contained(results, sub_region)
    }

    fn contains(&self, position: Vector3) -> bool {
        self.transform.contains(position)
    }

    fn get_closest_intersection<'a>(&'a self, ray: &mut Ray) -> Option<IntersectInfo<'a>> {
        self.transform.get_closest_intersection(ray)
    }

    fn trace_ray_bundle<'a>(
        &'a self,
        bundle: &mut RayBundle<'a>,
        active: &[u32],
    ) -> Result<(), RayTracerError> {
        self.transform.trace_ray_bundle(bundle, active)
    }

    fn get_intersect_cost(&self) -> f64 {
        self.transform.get_intersect_cost()
    }

    fn intersection_iterator<'a>(
        &'a self,
        ray: &Ray,
    ) -> Result<Vec<IntersectInfo<'a>>, RayTracerError> {
        self.transform.intersection_iterator(ray)
    }

    fn get_crossing(&self, plane: &Plane) -> Result<Vec<&dyn Traceable>, RayTracerError> {
        self.transform.get_crossing(plane)
    }

    fn get_touching(
        &self,
        position: Vector3,
        error: f64,
    ) -> Result<Vec<&dyn Traceable>, RayTracerError> {
        self.transform.get_touching(position, error)
    }
}

/// A hit on a placed part, and which part.
#[derive(Clone, Copy)]
pub struct PartHit<'a> {
    /// The part that was hit.
    pub part: &'a PlacedPart,
    /// The hit itself, in world space, with the mesh's leaf triangle inside it.
    pub info: IntersectInfo<'a>,
}

impl PartHit<'_> {
    /// The world-space surface point at this hit. A hit on a leaf that is not a
    /// [`MinimalTriangle`] is a [`RayTracerError::InvalidArgument`], since nothing else names
    /// the face it stands for.
    pub fn surface_point(&self) -> Result<SurfacePoint, RayTracerError> {
        let leaf = self.info.closest_hit_as::<MinimalTriangle>().ok_or(
            RayTracerError::InvalidArgument(
                "the hit is not on a mesh triangle, so it has no face to look up",
            ),
        )?;
        self.part
            .surface_point(leaf.face_index, self.info.hit_position)
    }
}

impl std::fmt::Debug for PartHit<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PartHit")
            .field("part", &self.part.index)
            .field("info", &self.info)
            .finish()
    }
}

/// A set of parts a ray can be traced through as one, each hit knowing its part.
pub struct PlacedParts {
    parts: Vec<Arc<PlacedPart>>,
    tree: Option<TraceableRef>,
}

impl PlacedParts {
    /// The set of `parts`, with a tree over their world-space bounds. Build it once, then trace
    /// from any thread.
    pub fn new(parts: Vec<PlacedPart>) -> Self {
        let parts: Vec<Arc<PlacedPart>> = parts
            .into_iter()
            .enumerate()
            .map(|(index, mut part)| {
                part.index = index;
                Arc::new(part)
            })
            .collect();
        let leaves = parts
            .iter()
            .map(|part| part.clone() as TraceableRef)
            .collect();
        let tree = BoundingVolumeHierarchy::create_new_hierarchy(
            leaves,
            BvhCreationOptions::LocalOrderClustering,
        );
        PlacedParts { parts, tree }
    }

    /// The parts, by index.
    pub fn parts(&self) -> &[Arc<PlacedPart>] {
        &self.parts
    }

    /// The nearest hit along `ray`, narrowing the ray's window to it as a tree does, whichever
    /// part it came from. A shadow ray stops at the first hit it finds and keeps its window.
    pub fn trace<'a>(&'a self, ray: &mut Ray) -> Option<PartHit<'a>> {
        let tree = self.tree.as_ref()?;
        let found = walk(tree.as_ref(), ray);
        if let Some(hit) = found {
            if !ray.is_shadow_ray {
                ray.max_distance_to_consider = hit.info.distance_to_hit;
            }
        }
        found
    }
}

/// The nearer of the hits under `node`, walking the top-level tree the way a BVH traces itself:
/// nearer child first by the node's splitting axis, narrowing the window as hits are found. The
/// first of two hits at an equal distance stands, so the outcome does not depend on anything but
/// the tree.
///
/// A part is traced without testing its own box first: a triangle counts a ray within its float
/// slack of an edge as a hit, which can lie just outside the box, and a tree of `Transform`s has
/// no such test to lose it to.
fn walk<'a>(node: &'a dyn Traceable, ray: &mut Ray) -> Option<PartHit<'a>> {
    if let Some(part) = node.as_any().downcast_ref::<PlacedPart>() {
        let info = part.transform.get_closest_intersection(ray)?;
        return (info.hit_type != IntersectionType::None).then_some(PartHit { part, info });
    }

    if let Some(inner) = node.as_any().downcast_ref::<BoundingVolumeHierarchy>() {
        if !ray.intersection(&inner.aabb) {
            return None;
        }
        let (first, second) = if ray.direction_normal[inner.splitting_plane] < 0.0 {
            (inner.right(), inner.left())
        } else {
            (inner.left(), inner.right())
        };
        return nearer_of(first.as_ref(), second.as_ref(), ray);
    }

    let mut best = None;
    for child in node.children().unwrap_or_default() {
        let found = walk(child, ray);
        if let Some(hit) = found {
            if ray.is_shadow_ray {
                return found;
            }
            ray.max_distance_to_consider = hit.info.distance_to_hit;
            best = found;
        }
    }
    best
}

fn nearer_of<'a>(
    first: &'a dyn Traceable,
    second: &'a dyn Traceable,
    ray: &mut Ray,
) -> Option<PartHit<'a>> {
    let first_hit = walk(first, ray);
    if let Some(hit) = first_hit {
        if ray.is_shadow_ray {
            return first_hit;
        }
        ray.max_distance_to_consider = hit.info.distance_to_hit;
    }

    let second_hit = walk(second, ray);
    match (first_hit, second_hit) {
        (Some(near), Some(far)) if far.info.distance_to_hit < near.info.distance_to_hit => {
            second_hit
        }
        (Some(_), _) => first_hit,
        (None, _) => second_hit,
    }
}
