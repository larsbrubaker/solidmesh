//! [`GpuSceneBuilder`]: the pack behind [`GpuRenderScene::pack_view`](super::GpuRenderScene), in
//! slices. A caller with no threads, the browser's UI thread, calls [`GpuSceneBuilder::step`]
//! each frame with as many work units as it has measured it can afford. There is no clock here
//! and no thread: what a slice does depends only on the scene and the units given, so the buffers
//! come out the same bits however the work is sliced. The one-call pack goes through the very
//! same steps, so it can't drift from this.
//!
//! **A work unit** is one small, fixed piece of work: gathering one face's corners and normals
//! ([`GPU_SCENE_BUILDER_UNITS_PER_FACE`] units per face for the whole pack, some of it spent in
//! each stage), one corner visited while the scene's placement is found, or one unit of the
//! picture atlas's build (`gpu_atlas_build.rs`: a texel cleared or copied, a halved texel costing
//! [`ATLAS_WORK_PER_HALVED_TEXEL`](super::ATLAS_WORK_PER_HALVED_TEXEL)). The stages run in this
//! order: gather faces, measure the bounds and reach of their corners, recentre and size each
//! face's ray offset, build the tree and with it everything sized by the parts (the part records,
//! the glass contacts, the first view, the picture atlas's plan and the picture block, each
//! pictured face's place in it), then build the atlas's texels. Everything but the tree step is
//! sliced to the unit, the atlas too, so a scene with a large picture (or one the atlas must
//! halve, `gpu_textures.rs` says what that costs) packs in as many short steps as the host likes.
//!
//! **The tree step cannot be paused**, the same limit the CPU's own tree has (the part's
//! `TraceableRef` is built off the frame ahead of time): a [`step`](GpuSceneBuilder::step) that
//! reaches the tree does the tree and the work sized by the parts, the glass contact entries (a
//! scene with no glass lying on glass packs no contacts at all) and the pictured faces, and a
//! thread-less host pays for it in one slice however many units it gave. Reaching it is reported
//! as [`GpuSceneBuilder::next_step_builds_tree`], so such a host can defer that step to an idle
//! moment, or size a scene it can afford. That step ends once the tree and its records are done
//! when the scene's atlas still has texels to build, so the atlas starts in a slice of its own; a
//! scene without pictures (its 1 by 1 white atlas) or a pack given its plan finishes in it. The
//! tree over a scene's parts is built in `f64` over `f32`-narrowed corners and copied out to the
//! flat records; slicing it needs a resumable BVH builder and is future work.
//!
//! **A re-pack of the same pictures** need not build the atlas again: a host that keeps the atlas
//! it uploaded hands the earlier pack's plan to the next ([`GpuSceneBuilder::with_texture_plan`],
//! the plan from [`GpuTextureAtlas::plan`]), which then writes the picture block from the plan's
//! placements and builds no texels: its
//! [`texture_atlas`](super::GpuRenderScene::texture_atlas) has the plan's size and placements and
//! no bytes, so the caller keeps the texture it uploaded. Handing a plan over checks only the
//! pictures' count and sizes ([`TextureAtlasPlan::fits_sizes_of`]); another count or size is
//! refused, but other pictures of the same sizes are not, and the pack would point at the old
//! atlas's texels. A caller reuses a plan only for the very same pictures in the same order (for
//! instance [`RenderScene::textures`](crate::ray_tracer::RenderScene::textures) matched one by one
//! with `Arc::ptr_eq`). A plan also skips [`GpuTextureLimits`]: it was fitted to the limits of
//! the pack that made it, so a caller reusing one on another device (after a lost device, say)
//! checks that device's limits itself.

use std::sync::Arc;

use solidmesh_math::Vector3;

use super::gpu_atlas_build::TextureAtlasBuild;
use super::gpu_contacts::{pack_glass_contacts, GpuGlassContacts};
use super::gpu_scene::{
    check_image_buffers, GpuRenderScene, GpuSceneLimits, PartRecord, ScenePlacement,
    CHANNEL_STRIDE, FACE_STRIDE, PART_STRIDE,
};
use super::gpu_texture_block::{pack_texture_block, GpuTextureBlock};
use super::gpu_textures::{
    plan_texture_atlas, GpuTextureAtlas, GpuTextureLimits, TextureAtlasPlan,
};
use super::gpu_view::{
    pack_words, srgb_table_bytes, view_record, GpuCompositing, GpuView, ViewRecord,
    SRGB_TABLE_BYTES,
};
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
    Atlas(TextureAtlasBuild),
    Finished,
}

/// What the tree step packs, held while the atlas's texels are built.
struct Packed {
    tree: FlatTree,
    parts: Vec<PartRecord>,
    view: ViewRecord,
    srgb_table: [u8; SRGB_TABLE_BYTES],
    contacts: GpuGlassContacts,
    textures: GpuTextureBlock,
}

/// A pack being built; see the module header. Produces the same bits however its work is sliced.
pub struct GpuSceneBuilder<'a> {
    source: SceneSource<'a>,
    view: GpuView,
    limits: GpuSceneLimits,
    texture_limits: GpuTextureLimits,
    /// A plan handed over by [`GpuSceneBuilder::with_texture_plan`]: no atlas is built.
    texture_plan: Option<Arc<TextureAtlasPlan>>,
    /// Atlas work units spent, the counter of `gpu_atlas_build.rs`.
    atlas_work: u64,
    packed: Option<Packed>,
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
            texture_limits: GpuTextureLimits::DEFAULT,
            texture_plan: None,
            atlas_work: 0,
            packed: None,
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

    /// This builder with the scene's pictures fitted to `texture_limits` (`gpu_textures.rs`)
    /// instead of [`GpuTextureLimits::DEFAULT`]. A plan handed over
    /// ([`GpuSceneBuilder::with_texture_plan`]) is used instead of any limits.
    pub fn with_texture_limits(mut self, texture_limits: GpuTextureLimits) -> Self {
        self.texture_limits = texture_limits;
        self
    }

    /// This builder with the scene's pictures laid out by `plan`, an earlier pack's
    /// ([`GpuTextureAtlas::plan`]) whose atlas the caller still holds: the pack writes the
    /// picture block from the plan's placements and builds no atlas texels, so its atlas's
    /// `bytes()` is `None` and the caller keeps the texture it uploaded (see the module header).
    /// Refused with [`FlatTreeError::NotRepresentable`] when the scene's pictures are not the
    /// count and sizes the plan was made for. Only the count and sizes are checked: hand a plan
    /// over only for the very same pictures in the same order (the scene's
    /// `Arc<TraceTexture>`s matched with `Arc::ptr_eq`), never for other pictures of the same
    /// sizes. The plan replaces [`GpuSceneBuilder::with_texture_limits`]: no limit is checked,
    /// so a caller reusing a plan on another device (after a lost device) checks its limits
    /// itself.
    pub fn with_texture_plan(mut self, plan: Arc<TextureAtlasPlan>) -> Result<Self, FlatTreeError> {
        if !plan.fits_sizes_of(self.source.get().textures()) {
            return Err(FlatTreeError::NotRepresentable {
                reason: "these pictures are not the ones the atlas plan was made for",
            });
        }
        self.texture_plan = Some(plan);
        Ok(self)
    }

    /// How many of the picture atlas's work units (`gpu_atlas_build.rs`) this pack has spent: 0
    /// for a pack given its plan, and the plan's [`TextureAtlasPlan::build_work`] once a pack that
    /// builds its atlas is done.
    pub fn atlas_work_done(&self) -> u64 {
        self.atlas_work
    }

    /// True when the next [`GpuSceneBuilder::step`] will build the tree: the one step that can't
    /// be sized (see the module header).
    pub fn next_step_builds_tree(&self) -> bool {
        matches!(self.stage, Stage::Tree)
    }

    /// Runs up to `work_units` of the pack (a step in the atlas's halving can pass them by less
    /// than one halved texel's cost, which is never cut; the tree's step is not sized at all).
    /// Answers the finished pack once, `None` when more remains. Calling it again after the pack
    /// came back answers an error about being finished, never a second, different pack, and so
    /// does a step after an error. See the module header for what a unit is.
    pub fn step(&mut self, work_units: usize) -> Result<Option<GpuRenderScene>, FlatTreeError> {
        let stepped = self.run(work_units);
        if stepped.is_err() {
            // A stage that failed may have spent part of its work (faces gathered, records
            // packed), so the pack never resumes from it.
            self.stage = Stage::Finished;
        }
        stepped
    }

    /// [`GpuSceneBuilder::step`] until an error ends the pack.
    fn run(&mut self, work_units: usize) -> Result<Option<GpuRenderScene>, FlatTreeError> {
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
                    self.stage = Stage::Finished;
                    let tree =
                        FlatTree::build(&self.world_triangles, self.limits.max_storage_binding)?;
                    let plan = self.pack_records(tree)?;
                    if self.texture_plan.is_some() {
                        let atlas = GpuTextureAtlas { plan, bytes: None };
                        return Ok(Some(self.finish(atlas)?));
                    }
                    let mut build = TextureAtlasBuild::new(plan);
                    if self.source.get().textures().is_empty() {
                        let atlas = build.step(self.source.get().textures(), usize::MAX)?;
                        self.atlas_work = build.work_done();
                        if let Some(atlas) = atlas {
                            return Ok(Some(self.finish(atlas)?));
                        }
                    }
                    // The atlas's texels start in a step of their own (see the module header).
                    self.stage = Stage::Atlas(build);
                    return Ok(None);
                }
                Stage::Atlas(build) => {
                    let atlas = build.step_within(self.source.get().textures(), &mut budget);
                    self.atlas_work = build.work_done();
                    match atlas? {
                        Some(atlas) => {
                            self.stage = Stage::Finished;
                            return Ok(Some(self.finish(atlas)?));
                        }
                        None => return Ok(None),
                    }
                }
                Stage::Finished => {
                    return Err(FlatTreeError::NotRepresentable {
                        reason: "this GPU pack was already finished",
                    });
                }
            }
        }
    }

    /// Packs everything the tree step does after the tree (see the module header) and answers
    /// the atlas's plan: the handed-over one, or one made for the scene's pictures.
    fn pack_records(&mut self, tree: FlatTree) -> Result<Arc<TextureAtlasPlan>, FlatTreeError> {
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
        let mut view = view_record(scene, &self.view, &self.placement)?;
        let srgb_table: [u8; SRGB_TABLE_BYTES] = srgb_table_bytes(scene);
        let contacts = pack_glass_contacts(
            scene,
            &self.faces,
            &tree,
            self.placement.origin,
            self.limits.max_storage_binding,
        )?;
        pack_words(&mut view, &contacts, part_records.len())?;
        let plan = match &self.texture_plan {
            Some(plan) => Arc::clone(plan),
            None => Arc::new(plan_texture_atlas(scene.textures(), self.texture_limits)?),
        };
        let first_texture_word =
            (part_records.len() * (PART_STRIDE / 4)) as u64 + contacts.to_words().len() as u64;
        let textures = pack_texture_block(
            scene,
            &mut self.faces,
            plan.placements(),
            first_texture_word,
            self.limits.max_storage_binding,
        )?;
        self.packed = Some(Packed {
            tree,
            parts: part_records,
            view,
            srgb_table,
            contacts,
            textures,
        });
        Ok(plan)
    }

    fn finish(&mut self, atlas: GpuTextureAtlas) -> Result<GpuRenderScene, FlatTreeError> {
        let packed = self.packed.take().ok_or(FlatTreeError::NotRepresentable {
            reason: "this GPU pack was already finished",
        })?;
        Ok(GpuRenderScene {
            tree: packed.tree,
            faces: std::mem::take(&mut self.faces),
            parts: packed.parts,
            view: packed.view,
            srgb_table: packed.srgb_table,
            pixels: self.view.image,
            composited: !matches!(self.view.compositing, GpuCompositing::Plain),
            limits: self.limits,
            placement_origin: self.placement.origin,
            placement_radius: self.placement.radius,
            contacts: packed.contacts,
            textures: packed.textures,
            atlas,
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
                // Set when the pack finishes, for a face showing a picture (`gpu_texture_block`).
                texture: super::NO_TEXTURE,
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
