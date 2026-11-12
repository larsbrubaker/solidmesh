//! [`GpuSceneBuilder`]: the pack behind [`GpuRenderScene::pack_view`](super::GpuRenderScene), in
//! slices. A caller with no threads, the browser's UI thread, calls [`GpuSceneBuilder::step`]
//! each frame with as many work units as it has measured it can afford. There is no clock here
//! and no thread: what a slice does depends only on the scene and the units given, so the buffers
//! come out the same bits however the work is sliced. The one-call pack goes through the very
//! same steps, so it can't drift from this.
//!
//! **A work unit** is one small, fixed piece of work: gathering one face's corners and normals
//! ([`GPU_SCENE_BUILDER_UNITS_PER_FACE`] units per face for the whole pack, some of it spent in
//! each stage), or one corner visited while the scene's placement is found. The stages run in
//! this order: gather faces, measure the bounds and reach of their corners, recentre and size
//! each face's ray offset, build the tree, then build the part records, the glass contacts and the
//! first view. The gathering, bounds, reach and recentring are sliced to the unit; the tree and
//! the last stage are not. The last stage is one call in the same step as the tree and costs in
//! proportion to the parts and the contact entries (a scene with no glass lying on glass packs no
//! contacts at all). **The tree build is one call and cannot be paused**,
//! the same limit the CPU's own tree has (the part's `TraceableRef` is built off the frame
//! ahead of time): a [`step`](GpuSceneBuilder::step) that reaches the tree does the tree, and
//! a thread-less host pays for it in one slice however many units it gave. Reaching it is
//! reported as [`GpuSceneBuilder::next_step_builds_tree`], so such a host can defer that step to
//! an idle moment, or size a scene it can afford. The tree over a scene's parts is built in
//! `f64` over `f32`-narrowed corners and copied out to the flat records; slicing it needs a
//! resumable BVH builder and is future work.

use std::sync::Arc;

use solidmesh_math::Vector3;

use super::gpu_contacts::pack_glass_contacts;
use super::gpu_scene::{
    check_image_buffers, GpuRenderScene, GpuSceneLimits, PartRecord, ScenePlacement,
    CHANNEL_STRIDE, FACE_STRIDE, PART_STRIDE,
};
use super::gpu_view::{srgb_table_bytes, view_record, GpuCompositing, GpuView, SRGB_TABLE_BYTES};
use super::{FaceRecord, FlatTree, FlatTreeError, SurfaceRecord};
use crate::ray_tracer::surface_point::SURFACE_OFFSET_F32_STEPS;
use crate::ray_tracer::RenderScene;

/// About how many work units each face of the scene costs across all stages, for a caller sizing
/// its budget: the face gathered, its six corner visits for bounds and reach, and its recentring.
pub const GPU_SCENE_BUILDER_UNITS_PER_FACE: usize = 8;

/// The scene being read: borrowed for the one-call pack, shared so a builder can live across
/// frames beside the scene it reads.
enum SceneSource<'a> {
    Borrowed(&'a RenderScene),
    Shared(Arc<RenderScene>),
}

impl SceneSource<'_> {
    fn get(&self) -> &RenderScene {
        match self {
            SceneSource::Borrowed(scene) => scene,
            SceneSource::Shared(scene) => scene,
        }
    }
}

enum Stage {
    GatherFaces {
        part: usize,
        face: usize,
    },
    Bounds {
        index: usize,
        low: Vector3,
        high: Vector3,
    },
    Reach {
        index: usize,
        origin: Vector3,
        reach: f64,
        radius: f64,
    },
    Recentre {
        index: usize,
    },
    Tree,
    Finished,
}

/// A pack being built; see the module header. Produces the same bits however its work is sliced.
pub struct GpuSceneBuilder<'a> {
    source: SceneSource<'a>,
    view: GpuView,
    limits: GpuSceneLimits,
    world_triangles: Vec<[Vector3; 3]>,
    faces: Vec<FaceRecord>,
    placement: ScenePlacement,
    stage: Stage,
    /// Corners not valid (a broken mesh) leave the placement unplaced, and `FlatTree::build`
    /// reports which triangle: the same refusal as ever.
    unplaced: bool,
}

impl<'a> GpuSceneBuilder<'a> {
    /// A builder of `scene` for `view`, borrowing the scene, for a caller that finishes in one
    /// call or holds `scene` itself. Refusals that need no geometry come back here: the image's
    /// buffers, the face and part buffers against `limits`.
    pub fn new(
        scene: &'a RenderScene,
        view: GpuView,
        limits: GpuSceneLimits,
    ) -> Result<Self, FlatTreeError> {
        Self::start(SceneSource::Borrowed(scene), view, limits)
    }

    /// A builder that shares `scene`, so it can live across frames beside it: the thread-less
    /// host's form.
    pub fn from_shared(
        scene: Arc<RenderScene>,
        view: GpuView,
        limits: GpuSceneLimits,
    ) -> Result<GpuSceneBuilder<'static>, FlatTreeError> {
        GpuSceneBuilder::start(SceneSource::Shared(scene), view, limits)
    }

    fn start(
        source: SceneSource<'a>,
        view: GpuView,
        limits: GpuSceneLimits,
    ) -> Result<Self, FlatTreeError> {
        check_image_buffers(&view, limits.max_storage_binding)?;
        let parts = source.get().parts().parts();
        let triangle_count: u64 = parts.iter().map(|p| p.surface().face_count() as u64).sum();
        let face_bytes = triangle_count * FACE_STRIDE as u64;
        if face_bytes > limits.max_storage_binding {
            return Err(FlatTreeError::PackedBufferTooLarge {
                buffer: "face",
                needed_bytes: face_bytes,
                limit_bytes: limits.max_storage_binding,
                what: "triangles",
                largest_that_fits: limits.max_storage_binding / FACE_STRIDE as u64,
            });
        }
        let part_bytes = parts.len() as u64 * PART_STRIDE as u64;
        if part_bytes > limits.max_storage_binding {
            return Err(FlatTreeError::PackedBufferTooLarge {
                buffer: "part",
                needed_bytes: part_bytes,
                limit_bytes: limits.max_storage_binding,
                what: "parts",
                largest_that_fits: limits.max_storage_binding / PART_STRIDE as u64,
            });
        }
        Ok(GpuSceneBuilder {
            source,
            view,
            limits,
            world_triangles: Vec::with_capacity(triangle_count as usize),
            faces: Vec::with_capacity(triangle_count as usize),
            placement: ScenePlacement {
                origin: Vector3::ZERO,
                radius: 0.0,
            },
            stage: Stage::GatherFaces { part: 0, face: 0 },
            unplaced: false,
        })
    }

    /// True when the next [`GpuSceneBuilder::step`] will build the tree: the one step that can't
    /// be sized (see the module header).
    pub fn next_step_builds_tree(&self) -> bool {
        matches!(self.stage, Stage::Tree)
    }

    /// Runs up to `work_units` of the pack. Answers the finished pack once, `None` when more
    /// remains. Calling it again after the pack came back answers an error about being finished,
    /// never a second, different pack. See the module header for what a unit is.
    pub fn step(&mut self, work_units: usize) -> Result<Option<GpuRenderScene>, FlatTreeError> {
        let mut budget = work_units;
        loop {
            // A slice that spent its whole budget stops here, whichever stage it reached: in
            // particular it never goes on to build the tree.
            if budget == 0 && !matches!(self.stage, Stage::Finished) {
                return Ok(None);
            }
            match &mut self.stage {
                Stage::GatherFaces { part, face } => {
                    let mut stage_part = *part;
                    let mut stage_face = *face;
                    let done = gather_faces(
                        self.source.get(),
                        &mut stage_part,
                        &mut stage_face,
                        &mut self.world_triangles,
                        &mut self.faces,
                        &mut budget,
                    )?;
                    if done {
                        self.stage = begin_bounds(&self.world_triangles);
                        self.unplaced = self.world_triangles.is_empty();
                        continue;
                    }
                    self.stage = Stage::GatherFaces {
                        part: stage_part,
                        face: stage_face,
                    };
                    return Ok(None);
                }
                Stage::Bounds { index, low, high } => {
                    let done = measure_bounds(
                        &self.world_triangles,
                        index,
                        low,
                        high,
                        &mut self.unplaced,
                        &mut budget,
                    );
                    if !done {
                        return Ok(None);
                    }
                    self.stage = if self.unplaced {
                        Stage::Recentre { index: 0 }
                    } else {
                        let origin = *low * 0.5 + *high * 0.5;
                        Stage::Reach {
                            index: 0,
                            origin,
                            reach: 0.0,
                            radius: 0.0,
                        }
                    };
                }
                Stage::Reach {
                    index,
                    origin,
                    reach,
                    radius,
                } => {
                    let done = measure_reach(
                        &self.world_triangles,
                        *origin,
                        index,
                        reach,
                        radius,
                        &mut budget,
                    );
                    if !done {
                        return Ok(None);
                    }
                    let step = *reach * f64::from(f32::EPSILON);
                    // Written so a NaN limit refuses as well.
                    let fits = step <= self.limits.max_vertex_step;
                    if !fits {
                        return Err(FlatTreeError::SceneTooLarge {
                            reach: *reach,
                            step,
                            required_step: self.limits.max_vertex_step,
                            largest_reach: self.limits.max_vertex_step / f64::from(f32::EPSILON),
                        });
                    }
                    self.placement = ScenePlacement {
                        origin: *origin,
                        radius: *radius,
                    };
                    self.stage = Stage::Recentre { index: 0 };
                }
                Stage::Recentre { index } => {
                    let done = recentre(
                        &mut self.world_triangles,
                        &mut self.faces,
                        self.placement.origin,
                        index,
                        &mut budget,
                    );
                    if !done {
                        return Ok(None);
                    }
                    self.stage = Stage::Tree;
                }
                Stage::Tree => {
                    let tree =
                        FlatTree::build(&self.world_triangles, self.limits.max_storage_binding)?;
                    self.stage = Stage::Finished;
                    return Ok(Some(self.finish(tree)?));
                }
                Stage::Finished => {
                    return Err(FlatTreeError::NotRepresentable {
                        reason: "this GPU pack was already finished",
                    });
                }
            }
        }
    }

    fn finish(&mut self, tree: FlatTree) -> Result<GpuRenderScene, FlatTreeError> {
        let scene = self.source.get();
        let parts = scene.parts().parts();
        let part_records = (0..parts.len())
            .map(|index| {
                let surface = scene
                    .surface(index)
                    .ok_or(FlatTreeError::UnexpectedBvhLeaf)?;
                Ok(PartRecord {
                    surface: SurfaceRecord::from_surface(surface),
                    opacity: scene.part_opacity(index).unwrap_or(1.0) as f32,
                })
            })
            .collect::<Result<Vec<_>, FlatTreeError>>()?;
        let view = view_record(scene, &self.view, &self.placement)?;
        let composited = !matches!(self.view.compositing, GpuCompositing::Plain);
        let srgb_table: [u8; SRGB_TABLE_BYTES] = srgb_table_bytes(scene);
        let contacts = pack_glass_contacts(
            scene,
            &self.faces,
            self.placement.origin,
            self.limits.max_storage_binding,
        )?;
        Ok(GpuRenderScene {
            tree,
            faces: std::mem::take(&mut self.faces),
            parts: part_records,
            view,
            srgb_table,
            pixels: self.view.image,
            composited,
            limits: self.limits,
            placement_origin: self.placement.origin,
            placement_radius: self.placement.radius,
            contacts,
        })
    }
}

fn begin_bounds(world_triangles: &[[Vector3; 3]]) -> Stage {
    let first = world_triangles.first().map_or(Vector3::ZERO, |t| t[0]);
    Stage::Bounds {
        index: 0,
        low: first,
        high: first,
    }
}

/// Gathers faces into world space until the budget runs out: `true` when every face is in.
fn gather_faces(
    scene: &RenderScene,
    part: &mut usize,
    face: &mut usize,
    world_triangles: &mut Vec<[Vector3; 3]>,
    faces: &mut Vec<FaceRecord>,
    budget: &mut usize,
) -> Result<bool, FlatTreeError> {
    let parts = scene.parts().parts();
    while *part < parts.len() {
        let placed = &parts[*part];
        let mesh_to_world = placed.mesh_to_world();
        let world_to_mesh = placed.world_to_mesh();
        let table = placed.surface();
        while *face < table.face_count() {
            if *budget == 0 {
                return Ok(false);
            }
            *budget -= 1;
            let corners = table
                .face_corners(*face)
                .ok_or(FlatTreeError::InvalidCoordinate {
                    triangle_index: world_triangles.len() as u64,
                })?;
            let world = corners.map(|corner| corner.transform_position(mesh_to_world));
            world_triangles.push(world);
            let (geometric, corner_normals) = table.face_normals(*face);
            let world_geometric =
                unit_or_world(geometric.transform_normal_inverse(world_to_mesh), geometric);
            let world_normals =
                corner_normals.map(|normal| normal.transform_normal_inverse(world_to_mesh));
            let largest = world_normals
                .iter()
                .map(|normal| normal.length())
                .fold(0.0_f64, f64::max);
            let scale = if largest > 0.0 { 1.0 / largest } else { 1.0 };
            let color = table.face_color(*face);
            faces.push(FaceRecord {
                normals: [
                    super::gpu_scene::narrow3(world_normals[0] * scale),
                    super::gpu_scene::narrow3(world_normals[1] * scale),
                    super::gpu_scene::narrow3(world_normals[2] * scale),
                ],
                geometric: super::gpu_scene::narrow3(world_geometric),
                ray_offset: 0.0,
                // The part count fits u32: `part_bytes` is within a u64 binding limit of 40-byte
                // records, and the source index is a u32 in `flat_tree`.
                part: *part as u32,
                color: u32::from(color.red)
                    | u32::from(color.green) << 8
                    | u32::from(color.blue) << 16,
            });
            *face += 1;
        }
        *part += 1;
        *face = 0;
    }
    Ok(true)
}

fn measure_bounds(
    world_triangles: &[[Vector3; 3]],
    index: &mut usize,
    low: &mut Vector3,
    high: &mut Vector3,
    unplaced: &mut bool,
    budget: &mut usize,
) -> bool {
    while *index < world_triangles.len() * 3 {
        if *budget == 0 {
            return false;
        }
        *budget -= 1;
        let corner = world_triangles[*index / 3][*index % 3];
        *index += 1;
        if !corner.is_valid() {
            *unplaced = true;
            return true;
        }
        for axis in 0..3 {
            low[axis] = low[axis].min(corner[axis]);
            high[axis] = high[axis].max(corner[axis]);
        }
    }
    true
}

fn measure_reach(
    world_triangles: &[[Vector3; 3]],
    origin: Vector3,
    index: &mut usize,
    reach: &mut f64,
    radius: &mut f64,
    budget: &mut usize,
) -> bool {
    while *index < world_triangles.len() * 3 {
        if *budget == 0 {
            return false;
        }
        *budget -= 1;
        let relative = world_triangles[*index / 3][*index % 3] - origin;
        *index += 1;
        *reach = reach
            .max(relative.x.abs())
            .max(relative.y.abs())
            .max(relative.z.abs());
        *radius = radius.max(relative.length());
    }
    true
}

fn recentre(
    world_triangles: &mut [[Vector3; 3]],
    faces: &mut [FaceRecord],
    origin: Vector3,
    index: &mut usize,
    budget: &mut usize,
) -> bool {
    while *index < world_triangles.len() {
        if *budget == 0 {
            return false;
        }
        *budget -= 1;
        // Subtracted in f64, before anything narrows: this is what keeps a far part's shape.
        let corners = &mut world_triangles[*index];
        *corners = corners.map(|corner| corner - origin);
        let l1 = corners
            .iter()
            .fold(0.0_f64, |big, c| big.max(c.x.abs() + c.y.abs() + c.z.abs()));
        faces[*index].ray_offset = (l1 * f64::from(f32::EPSILON) * SURFACE_OFFSET_F32_STEPS) as f32;
        *index += 1;
    }
    true
}

fn unit_or_world(vector: Vector3, fallback: Vector3) -> Vector3 {
    let length = vector.length();
    if length > 0.0 && length.is_finite() {
        vector / length
    } else {
        fallback
    }
}

/// The coverage-channel buffer's size for `pixels` pixels of a composited view, for a caller
/// creating the buffer ahead of the view.
pub fn gpu_channel_bytes(pixels: u64) -> u64 {
    pixels * CHANNEL_STRIDE as u64
}
