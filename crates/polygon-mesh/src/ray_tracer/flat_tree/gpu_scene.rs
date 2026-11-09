//! Packs a whole [`RenderScene`] for the WGSL path loop ([`PATH_WGSL`]): one [`FlatTree`] over
//! every placed part's world triangles, a [`FaceRecord`] for each triangle in the order the
//! tree's `source_index` numbers them (its part, corner normals, geometric normal, continuing-ray
//! offset and color), a [`PartRecord`] per part (its surface and opacity), and the uniform records
//! the view holds (`gpu_view.rs`: [`GpuView`] and [`ViewRecord`], and the sRGB table). Byte
//! layouts are the WGSL structs of `path.wgsl`, written field by field in little-endian order as
//! `layout.rs` does; a test pins them against naga's own layout of the shader.
//!
//! **Two uniform buffers, one for what orbiting moves.** The per-view uniform is rewritten every
//! time the camera or lights change; the sRGB table, unrelated to the view, is a separate fixed
//! buffer. The tree, the face and part buffers never move once packed, so orbiting a view costs
//! one small uniform write and no rebuild (`GpuRenderScene::set_view`).
//!
//! **The refusals.** Every buffer is checked against the caller's storage binding limit before it
//! is built, and [`gpu_scene_size_bound`] says the most a scene can need, so a caller can refuse
//! before it allocates anything. A light or sun the f32 shader cannot represent (a density or
//! radiance past f32, or a disc so small a direction's rounding is a real part of its size) is
//! refused too, with what to change, instead of rendering wrong.
//!
//! **When to fall back to the CPU.** Every [`FlatTreeError`] says what would fit, but only some
//! are about the scene and not a programming error. [`FlatTreeError::PackedBufferTooLarge`],
//! [`FlatTreeError::BufferTooLarge`], [`FlatTreeError::TooManyTriangles`],
//! [`FlatTreeError::SceneTooLarge`] and [`FlatTreeError::TreeTooDeep`] mean this scene does not
//! fit this GPU's limits: render it on the CPU. So does [`FlatTreeError::NotRepresentable`] when
//! its reason is about 32-bit values. [`FlatTreeError::InvalidCoordinate`] is a fallback too, and
//! the one a caller might mistake for a broken file: the CPU renders a mesh with a NaN or infinite
//! corner (a face with such a corner has no surface of its own and the tracer carries on past it)
//! and one whose corners pass [`super::COORDINATE_LIMIT`], where the GPU's `f32` buffers cannot.
//! [`FlatTreeError::NoTriangles`] is an empty scene, with nothing a GPU binding can hold: the CPU
//! renders it as only its lights, and a caller decides whether that is worth a render at all.
//! [`FlatTreeError::UnexpectedBvhLeaf`] is a bug, and so is a [`GpuSceneBuilder`] stepped after it
//! finished.
//!
//! **The one step that cannot pause.** [`GpuSceneBuilder`] slices all of a pack to its work units
//! except [`FlatTree::build`], which is one call: a host that must not block should run that step
//! when it can afford it ([`GpuSceneBuilder::next_step_builds_tree`] says when it is next), and a
//! thread-less host pays for it in one go however many units it gave.
//!
//! **Recentring.** A world coordinate narrowed to `f32` keeps fewer bits the farther it is from
//! the origin: a part at x = 1e7 would get 1 mm vertex steps. So the packer subtracts a stable
//! `f64` origin, the center of the scene's own bounds, from every corner and from the camera in
//! `f64` before anything narrows, and the shader works entirely relative to it. Using the scene's
//! bounds and not the eye means orbiting the camera doesn't move the triangles.
//! [`GpuRenderScene::scene_origin`] says where it is. Lights are directions, so they don't move.
//! [`GpuSceneLimits::max_vertex_step`] says how coarse the caller can accept the `f32` grid to
//! be, and a scene too spread out to meet it is refused ([`FlatTreeError::SceneTooLarge`]). A
//! floor is not in the bounds: it is analytic (`gpu_view.rs`), and only the parts are placed.
//!
//! **An orthographic eye far outside the scene.** A pulled-back eye is far from the scene while
//! a pixel stays the same size, so the eye's own `f32` rounding would move every ray by a large
//! part of a pixel. All rays of an orthographic camera run parallel, so the eye slides along the
//! view axis to just behind the parts' bounding sphere, which no part can be in front of; the rays
//! are the same lines and the hits the same hits. A floor can be behind that slid eye, so the
//! shader allows a catcher ray a negative distance to it (`catcher.wgsl`). A perspective eye can't
//! slide, and doesn't need to: its rounding grows with its distance exactly as a pixel's footprint
//! does.
//!
//! **Corner normals are not unit length.** The CPU interpolates corner normals in the mesh's
//! space and only then carries the result through the inverse transpose of the part's matrix. To
//! get the same direction from world-space corner normals the stored vectors must be the
//! transformed ones with their lengths intact, because under a non-uniform scale those lengths are
//! unequal and interpolating unit vectors would give a different direction. So each face's three
//! are scaled by one common factor (the largest becomes 1) and no more.

use solidmesh_math::Vector3;

use super::gpu_pack::GpuSceneBuilder;
use super::gpu_view::{
    view_record, GpuCompositing, GpuView, ViewRecord, SRGB_TABLE_BYTES, VIEW_RECORD_BYTES,
};
use super::layout::NODE_STRIDE;
use super::{
    FlatTree, FlatTreeError, FlatTreeSizes, SurfaceRecord, MAX_TRIANGLES, SURFACE_RECORD_STRIDE,
    TRIANGLE_STRIDE,
};
use crate::ray_tracer::{PathTraceSettings, RenderCamera, RenderScene};

/// A face record's size in bytes.
pub const FACE_STRIDE: usize = 60;
/// A part record's size in bytes: a [`SurfaceRecord`] and an opacity.
pub const PART_STRIDE: usize = 40;
/// The per-dispatch slice uniform's size in bytes.
pub const PIXEL_SLICE_BYTES: usize = 16;
/// An accumulation record's size in bytes: `vec4<f32>` (see `path.wgsl`).
pub const ACCUMULATION_STRIDE: usize = 16;
/// A coverage-channels record's size in bytes: two `vec4<f32>` (`PathChannels` in `path.wgsl`).
pub const CHANNEL_STRIDE: usize = 32;

/// What [`GpuRenderScene::pack`] may use.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GpuSceneLimits {
    /// The largest storage binding the device allows, in bytes
    /// ([`super::WEBGPU_DEFAULT_MAX_STORAGE_BINDING`] is WebGPU's default).
    pub max_storage_binding: u64,
    /// The coarsest `f32` step a recentred corner may have, in the scene's units. A scene whose
    /// corners reach farther from its center than this allows is refused.
    pub max_vertex_step: f64,
}

/// One face's record for the shader, indexed by a hit's `source_index`.
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct FaceRecord {
    /// Corner normals in world space, one common scale over the three; see the module header.
    pub normals: [[f32; 3]; 3],
    /// The face's unit geometric normal in world space, outward as the mesh wound it.
    pub geometric: [f32; 3],
    /// How far a continuing ray starts off this face.
    pub ray_offset: f32,
    /// The part this face belongs to.
    pub part: u32,
    /// The face's color: `0xBBGGRR` (sRGB bytes, decoded in the shader).
    pub color: u32,
}

impl FaceRecord {
    /// This record's bytes, in buffer order.
    pub fn to_le_bytes(&self) -> [u8; FACE_STRIDE] {
        let mut bytes = [0u8; FACE_STRIDE];
        let mut offset = 0;
        for corner in self.normals.iter().chain(std::iter::once(&self.geometric)) {
            for value in corner {
                bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
                offset += 4;
            }
        }
        for word in [self.ray_offset.to_bits(), self.part, self.color] {
            bytes[offset..offset + 4].copy_from_slice(&word.to_le_bytes());
            offset += 4;
        }
        bytes
    }
}

/// One part's record: its surface and how much of it is present.
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct PartRecord {
    pub surface: SurfaceRecord,
    pub opacity: f32,
}

impl PartRecord {
    /// This record's bytes, in buffer order.
    pub fn to_le_bytes(&self) -> [u8; PART_STRIDE] {
        let mut bytes = [0u8; PART_STRIDE];
        bytes[..SURFACE_RECORD_STRIDE].copy_from_slice(&self.surface.to_le_bytes());
        bytes[SURFACE_RECORD_STRIDE..PART_STRIDE].copy_from_slice(&self.opacity.to_le_bytes());
        bytes
    }
}

/// The sizes a packed scene's buffers come to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpuSceneSizes {
    pub node_bytes: u64,
    pub triangle_bytes: u64,
    pub face_bytes: u64,
    pub part_bytes: u64,
    /// The color accumulation buffer's size for this image, [`ACCUMULATION_STRIDE`] per pixel.
    pub accumulation_bytes: u64,
    /// The coverage-channel buffer's size for this image, [`CHANNEL_STRIDE`] per pixel, one
    /// record for a plain view (a binding cannot be empty and a plain view never touches it).
    pub channel_bytes: u64,
    pub tree: FlatTreeSizes,
}

/// The most a scene of `triangle_count` triangles and `part_count` parts can need, with node and
/// triangle counts at their worst, for a caller to refuse a scene before allocating. A flat tree
/// has at most one fewer node than leaf runs and at most one run per triangle. `pixel_count` is
/// the image's area; `compositing` says whether a view will use the coverage channels.
pub fn gpu_scene_size_bound(
    triangle_count: u64,
    part_count: u64,
    pixel_count: u64,
    compositing: bool,
) -> GpuSceneSizes {
    let triangles = triangle_count.min(MAX_TRIANGLES);
    let channels = if compositing { pixel_count.max(1) } else { 1 };
    GpuSceneSizes {
        node_bytes: triangles.saturating_sub(1).max(1) * NODE_STRIDE as u64,
        triangle_bytes: triangles * TRIANGLE_STRIDE as u64,
        face_bytes: triangles * FACE_STRIDE as u64,
        part_bytes: part_count * PART_STRIDE as u64,
        accumulation_bytes: pixel_count * ACCUMULATION_STRIDE as u64,
        channel_bytes: channels * CHANNEL_STRIDE as u64,
        tree: FlatTreeSizes {
            node_count: triangles.saturating_sub(1).max(1),
            triangle_count: triangles,
            node_bytes: triangles.saturating_sub(1).max(1) * NODE_STRIDE as u64,
            triangle_bytes: triangles * TRIANGLE_STRIDE as u64,
            depth: super::MAX_TREE_DEPTH,
        },
    }
}

/// Where a scene sits: the origin its packed coordinates are relative to, and how far its corners
/// reach from there.
pub(super) struct ScenePlacement {
    pub(super) origin: Vector3,
    pub(super) radius: f64,
}

pub(super) fn narrow3(v: Vector3) -> [f32; 3] {
    // `as` rounds to nearest here: no saturation, normals are at most unit length.
    [v.x as f32, v.y as f32, v.z as f32]
}

/// A scene in GPU-ready buffers. What a [`GpuView`] changes is the [`ViewRecord`] alone; the tree,
/// the faces, the parts and the sRGB table stay as packed.
#[derive(Debug, Clone, PartialEq)]
pub struct GpuRenderScene {
    pub(super) tree: FlatTree,
    pub(super) faces: Vec<FaceRecord>,
    pub(super) parts: Vec<PartRecord>,
    pub(super) view: ViewRecord,
    pub(super) srgb_table: [u8; SRGB_TABLE_BYTES],
    pub(super) pixels: (u32, u32),
    pub(super) composited: bool,
    pub(super) limits: GpuSceneLimits,
    pub(super) placement_origin: Vector3,
    pub(super) placement_radius: f64,
}

impl GpuRenderScene {
    /// Packs `scene` as seen through `camera` at `width` by `height`, for `settings`, in
    /// coordinates relative to the scene's own center (see the module header): a plain view, the
    /// parts' radiance averaged and no coverage channels. Goes through the same steps as
    /// [`GpuSceneBuilder`], which a caller with no threads uses instead. Refused with a
    /// [`FlatTreeError`] when any buffer would exceed `limits`, when the scene spreads too far for
    /// `limits.max_vertex_step`, or when a value can't be carried in f32; the empty scene is
    /// `FlatTreeError::NoTriangles`, since a GPU binding cannot be empty.
    pub fn pack(
        scene: &RenderScene,
        camera: &RenderCamera,
        width: u32,
        height: u32,
        settings: &PathTraceSettings,
        limits: GpuSceneLimits,
    ) -> Result<GpuRenderScene, FlatTreeError> {
        let view = GpuView::plain(*camera, width, height, *scene.lights(), *settings);
        Self::pack_view(scene, &view, limits)
    }

    /// Packs `scene` for `view`, compositing and all, in one call. The tree holds only `scene`'s
    /// parts: a [`GpuFloor`](super::GpuFloor) is a shape in the shader, not a part.
    pub fn pack_view(
        scene: &RenderScene,
        view: &GpuView,
        limits: GpuSceneLimits,
    ) -> Result<GpuRenderScene, FlatTreeError> {
        let mut builder = GpuSceneBuilder::new(scene, *view, limits)?;
        loop {
            if let Some(packed) = builder.step(usize::MAX)? {
                return Ok(packed);
            }
        }
    }

    /// Rewrites the per-view uniform for `view`; the tree, faces, parts and sRGB table stay as
    /// packed, so orbiting costs one small uniform write and no rebuild. Refused as a pack is when
    /// a value can't be carried or the image's buffers exceed the limits. A different image size
    /// is the caller's to reallocate; this record only says the new size.
    pub fn set_view(&mut self, scene: &RenderScene, view: &GpuView) -> Result<(), FlatTreeError> {
        check_image_buffers(view, self.limits.max_storage_binding)?;
        let placement = ScenePlacement {
            origin: self.placement_origin,
            radius: self.placement_radius,
        };
        self.view = view_record(scene, view, &placement)?;
        self.pixels = view.image;
        self.composited = !matches!(view.compositing, GpuCompositing::Plain);
        Ok(())
    }

    /// The world position every packed coordinate is relative to: the center of the scene's
    /// bounds. Add it to a packed position to get the world position back.
    pub fn scene_origin(&self) -> Vector3 {
        self.placement_origin
    }

    /// The flat tree over every triangle, relative to [`GpuRenderScene::scene_origin`].
    pub fn tree(&self) -> &FlatTree {
        &self.tree
    }

    /// The face records, in `source_index` order.
    pub fn faces(&self) -> &[FaceRecord] {
        &self.faces
    }

    /// The part records.
    pub fn parts(&self) -> &[PartRecord] {
        &self.parts
    }

    /// The per-view uniform.
    pub fn view(&self) -> &ViewRecord {
        &self.view
    }

    /// The face buffer's bytes.
    pub fn face_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(self.faces.len() * FACE_STRIDE);
        for face in &self.faces {
            bytes.extend_from_slice(&face.to_le_bytes());
        }
        bytes
    }

    /// The part buffer's bytes.
    pub fn part_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(self.parts.len() * PART_STRIDE);
        for part in &self.parts {
            bytes.extend_from_slice(&part.to_le_bytes());
        }
        bytes
    }

    /// The per-view uniform's bytes: the only buffer a change of view writes.
    pub fn view_bytes(&self) -> [u8; VIEW_RECORD_BYTES] {
        self.view.to_le_bytes()
    }

    /// The sRGB table uniform's bytes.
    pub fn srgb_table_bytes(&self) -> [u8; SRGB_TABLE_BYTES] {
        self.srgb_table
    }

    /// How many pixels the accumulation buffers hold.
    pub fn pixel_count(&self) -> u64 {
        u64::from(self.pixels.0) * u64::from(self.pixels.1)
    }

    /// The sizes this pack came to, the accumulation buffers included.
    pub fn sizes(&self) -> GpuSceneSizes {
        let tree = self.tree.sizes();
        let pixels = self.pixel_count();
        GpuSceneSizes {
            node_bytes: tree.node_bytes,
            triangle_bytes: tree.triangle_bytes,
            face_bytes: self.faces.len() as u64 * FACE_STRIDE as u64,
            part_bytes: self.parts.len() as u64 * PART_STRIDE as u64,
            accumulation_bytes: pixels * ACCUMULATION_STRIDE as u64,
            channel_bytes: if self.composited {
                pixels * CHANNEL_STRIDE as u64
            } else {
                CHANNEL_STRIDE as u64
            },
            tree,
        }
    }
}

/// `view`'s image buffers against the limit, before anything is built for it.
pub(super) fn check_image_buffers(view: &GpuView, limit: u64) -> Result<(), FlatTreeError> {
    let (width, height) = view.image;
    if width == 0 || height == 0 || view.trace.0 == 0 || view.trace.1 == 0 {
        return Err(FlatTreeError::NotRepresentable {
            reason: "a render needs an image of at least one pixel",
        });
    }
    let pixels = u64::from(width) * u64::from(height);
    let accumulation_bytes = pixels * ACCUMULATION_STRIDE as u64;
    if accumulation_bytes > limit {
        return Err(FlatTreeError::PackedBufferTooLarge {
            buffer: "accumulation",
            needed_bytes: accumulation_bytes,
            limit_bytes: limit,
            what: "pixels",
            largest_that_fits: limit / ACCUMULATION_STRIDE as u64,
        });
    }
    let composited = !matches!(view.compositing, GpuCompositing::Plain);
    let channel_bytes = pixels * CHANNEL_STRIDE as u64;
    if composited && channel_bytes > limit {
        return Err(FlatTreeError::PackedBufferTooLarge {
            buffer: "coverage channel",
            needed_bytes: channel_bytes,
            limit_bytes: limit,
            what: "pixels",
            largest_that_fits: limit / CHANNEL_STRIDE as u64,
        });
    }
    Ok(())
}

/// The per-dispatch uniform: pixels `[first, first + count)` of the row-order image, one sample
/// each, at `sample_index`.
pub fn pixel_slice_bytes(first: u32, count: u32, sample_index: u32) -> [u8; PIXEL_SLICE_BYTES] {
    let mut bytes = [0u8; PIXEL_SLICE_BYTES];
    bytes[0..4].copy_from_slice(&first.to_le_bytes());
    bytes[4..8].copy_from_slice(&count.to_le_bytes());
    bytes[8..12].copy_from_slice(&sample_index.to_le_bytes());
    bytes
}
