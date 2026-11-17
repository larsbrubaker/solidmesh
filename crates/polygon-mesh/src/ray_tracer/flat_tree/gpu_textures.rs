//! The picture atlas for the GPU path loop: every [`TraceTexture`] of a scene laid side by side in
//! one RGBA8 picture ([`GpuTextureAtlas`]), for a caller to upload as one `Rgba8Uint` texture the
//! shader reads texel by texel (`picture.wgsl`: a `texture_2d<u32>` at group 2 binding 7, read
//! with `textureLoad`, so it needs no sampler and no filtering support). `gpu_texture_block.rs`
//! writes, per pictured face, where in this atlas its picture lies; `gpu_pack.rs` builds both in a
//! pack's last steps, and this crate owns
//! no GPU device: the bytes go back to the caller, who can take them out of the pack once
//! uploaded ([`GpuTextureAtlas::take_bytes`]) so they are not held twice; after that
//! [`GpuTextureAtlas::bytes`] answers `None`, so a re-upload (after a lost device) cannot hand an
//! empty slice to a texture of the atlas's size.
//!
//! **Bytes.** Straight sRGB RGBA8 like the pictures, rows bottom first: the atlas's row 0 is the
//! first `width * 4` bytes and holds row 0 of every picture placed at `y = 0`, so a picture's
//! texel `(c, r)` is the atlas's `(x + c, y + r)`. Texels no picture covers are 0. There is no
//! padding between pictures: the shader wraps a coordinate inside its picture's own rectangle
//! before it loads, so it never reads a neighbour. A scene without pictures gets a 1 by 1 white
//! atlas, so a caller can always bind one.
//!
//! **Memory.** The atlas and every shrunk picture on the way to it ([`TraceTexture::try_shrunk`])
//! are allocated fallibly: a device without the memory gets [`FlatTreeError::OutOfMemory`], a
//! CPU fallback. What can still abort on allocation is small or bounded by the caller's own
//! limits: the per-picture lists of a plan (sizes, placements, halvings: a few words a picture),
//! and the pack's other buffers (faces, tree, part words), which the storage binding limit
//! bounds before they are built.
//!
//! **Layout**, deterministic for the same picture sizes: pictures in order of height, tallest
//! first, then width, widest first, then index; laid left to right on shelves as tall as their
//! first picture, a picture that would pass the atlas's width starting the next shelf. The width
//! tried first is the larger of the widest picture and the square root of all texels (rounded
//! up), then double that again and again, and the largest side: of the widths whose atlas has
//! both sides within [`GpuTextureLimits::max_side`] and no more texels than
//! [`GpuTextureLimits::max_texels`], the one of least area is used (the narrower on a tie), its
//! width trimmed to the shelves' widest.
//!
//! **When the pictures do not fit**, the largest picture is halved ([`TraceTexture::halved`]:
//! each side `div_ceil(2)`) and the layout tried again, until it fits. Largest means a picture
//! with a side past `max_side` first, since it can never fit as it is, then the most texels,
//! then the lowest index. [`GpuTextureAtlas::shrunk`] says which pictures were halved and how
//! often, so a CPU render can show the same pictures ([`TraceTexture::shrunk`]) and the two routes
//! agree. When the largest picture is already 1 by 1 and nothing fits,
//! [`FlatTreeError::TexturesTooLarge`] refuses the scene: render it on the CPU.
//!
//! **Planning is cheap, building is not.** [`plan_texture_atlas`] works on the pictures' sizes
//! alone: the layout, the halvings and the atlas's size, for a CPU route that only needs to know
//! which pictures to shrink, or a caller sizing its memory. Building touches every texel: the
//! atlas cleared, one copy of each picture into it, plus, for a picture halved `n` times, `n`
//! passes that decode, average and re-encode its texels (the first over all of them, each later
//! one over a quarter as many). [`TextureAtlasPlan::build`] does it in one call and
//! [`fit_texture_atlas`] is the plan and the build in a row; [`TextureAtlasBuild`]
//! (`gpu_atlas_build.rs`) does it in bounded pieces, the same bytes however they are cut, and a
//! pack builds its atlas that way inside its own work units (`gpu_pack.rs`). A host with no threads
//! steps the pack (or the build) a frame's worth at a time, and on a re-pack of the same pictures
//! hands the pack the plan its atlas was built to ([`GpuTextureAtlas::plan`]) so nothing is built
//! again.
//!
//! **Whose pictures.** These take any slice of what borrows as a [`TraceTexture`]: a scene's
//! shared pictures (`Arc<TraceTexture>`, what [`RenderScene::textures`] gives) or plain owned
//! ones, read in place either way. The shrunk pictures a build makes are its own, dropped once
//! copied into the atlas.
//!
//! [`RenderScene::textures`]: crate::ray_tracer::RenderScene::textures

use std::borrow::Borrow;
use std::sync::Arc;

use super::gpu_atlas_build::{TextureAtlasBuild, ATLAS_WORK_PER_HALVED_TEXEL};
use super::FlatTreeError;
use crate::ray_tracer::TraceTexture;

/// Bytes per atlas texel: red, green, blue, alpha.
pub(super) const TEXEL_BYTES: usize = 4;

/// What a picture atlas may use: the GPU's largest 2D texture side and a budget of texels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpuTextureLimits {
    /// The most texels the atlas may hold, its width times its height (4 bytes each).
    pub max_texels: u64,
    /// The largest side a 2D texture may have (WebGPU's `maxTextureDimension2D`).
    pub max_side: u32,
}

impl GpuTextureLimits {
    /// WebGPU's default largest side, 8192, and a budget of 4096 by 4096 texels (64 MiB of
    /// RGBA8), which a phone can hold while the atlas is being uploaded: a caller with memory to
    /// spare passes a larger budget.
    pub const DEFAULT: GpuTextureLimits = GpuTextureLimits {
        max_texels: 4096 * 4096,
        max_side: 8192,
    };

    /// The most bytes an atlas within these limits can take: its largest texel count times 4,
    /// and never less than the 4 of the 1 by 1 atlas a scene without pictures gets.
    pub fn max_atlas_bytes(&self) -> u64 {
        let side = u64::from(self.max_side);
        self.max_texels
            .min(side * side)
            .saturating_mul(TEXEL_BYTES as u64)
            .max(TEXEL_BYTES as u64)
    }
}

impl Default for GpuTextureLimits {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Where one picture lies in the atlas, in texels: its bottom-left texel and its size.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AtlasPlacement {
    pub x: u32,
    pub y: u32,
    pub width: u32,
    pub height: u32,
}

/// A picture the atlas holds smaller than given: [`TraceTexture::shrunk`] by `halvings` gives it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ShrunkTexture {
    /// The picture's index in the scene's textures.
    pub texture: u32,
    /// How many times it was halved.
    pub halvings: u32,
    /// Its size in the atlas.
    pub width: u32,
    pub height: u32,
}

/// Where every picture of a scene goes and which are shrunk, from their sizes alone; see the
/// module header.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TextureAtlasPlan {
    pub(super) width: u32,
    pub(super) height: u32,
    /// The pictures' sizes as given, to check [`TextureAtlasPlan::build`] gets the same ones.
    pub(super) given: Vec<(u32, u32)>,
    pub(super) placements: Vec<AtlasPlacement>,
    pub(super) shrunk: Vec<ShrunkTexture>,
}

impl TextureAtlasPlan {
    /// The atlas's width in texels.
    pub fn width(&self) -> u32 {
        self.width
    }

    /// The atlas's height in texels.
    pub fn height(&self) -> u32 {
        self.height
    }

    /// The atlas's size in bytes, 4 a texel.
    pub fn byte_len(&self) -> u64 {
        u64::from(self.width) * u64::from(self.height) * TEXEL_BYTES as u64
    }

    /// Where each picture lies, by its index in the scene's textures.
    pub fn placements(&self) -> &[AtlasPlacement] {
        &self.placements
    }

    /// The pictures held smaller than given, in index order; empty when every picture fits.
    pub fn shrunk(&self) -> &[ShrunkTexture] {
        &self.shrunk
    }

    /// The atlas of `textures`, the pictures this plan was made for (a different count or size is
    /// [`FlatTreeError::NotRepresentable`]), refused with [`FlatTreeError::OutOfMemory`] when
    /// the atlas's bytes, or a shrunk picture's, cannot be allocated. In one call:
    /// [`TextureAtlasBuild`] builds the same atlas in pieces.
    pub fn build<T: Borrow<TraceTexture>>(
        &self,
        textures: &[T],
    ) -> Result<GpuTextureAtlas, FlatTreeError> {
        let mut build = TextureAtlasBuild::new(Arc::new(self.clone()));
        loop {
            if let Some(atlas) = build.step(textures, usize::MAX)? {
                return Ok(atlas);
            }
        }
    }

    /// How much work [`TextureAtlasBuild`] does for this plan, in its units (see the module
    /// header): every atlas texel cleared, every picture texel copied, and
    /// [`ATLAS_WORK_PER_HALVED_TEXEL`] for each texel a halving writes. A host can show a build's
    /// progress against it ([`TextureAtlasBuild::work_done`] ends equal to it).
    pub fn build_work(&self) -> u64 {
        if self.given.is_empty() {
            return 1;
        }
        let texels = |(w, h): (u32, u32)| u64::from(w) * u64::from(h);
        let copied: u64 = self
            .placements
            .iter()
            .map(|p| texels((p.width, p.height)))
            .sum();
        let halved: u64 = self
            .shrunk
            .iter()
            .map(|s| {
                let mut size = self.given[s.texture as usize];
                (0..s.halvings).fold(0u64, |sum, _| {
                    size = (size.0.div_ceil(2), size.1.div_ceil(2));
                    sum + texels(size)
                })
            })
            .sum();
        texels((self.width, self.height)) + copied + halved * ATLAS_WORK_PER_HALVED_TEXEL as u64
    }

    /// True when `textures` have the count and sizes this plan was made for. Only the count and
    /// sizes are compared, never the texels: other pictures of the same sizes fit as well, so a
    /// caller reusing a plan (and the atlas built to it) matches the pictures themselves, the
    /// very same ones in the same order (the scene's `Arc<TraceTexture>`s by `Arc::ptr_eq`).
    pub fn fits_sizes_of<T: Borrow<TraceTexture>>(&self, textures: &[T]) -> bool {
        textures.len() == self.given.len()
            && textures
                .iter()
                .map(Borrow::borrow)
                .zip(&self.given)
                .all(|(t, &size)| (t.width(), t.height()) == size)
    }
}

/// The scene's pictures in one RGBA8 picture; see the module header.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GpuTextureAtlas {
    pub(super) plan: Arc<TextureAtlasPlan>,
    /// `None` once taken, or for a pack given its plan.
    pub(super) bytes: Option<Vec<u8>>,
}

impl GpuTextureAtlas {
    /// The atlas's width in texels.
    pub fn width(&self) -> u32 {
        self.plan.width
    }

    /// The atlas's height in texels.
    pub fn height(&self) -> u32 {
        self.plan.height
    }

    /// The plan this atlas was built to, for a later pack of the same pictures that need not
    /// build it again (`GpuSceneBuilder::with_texture_plan`).
    pub fn plan(&self) -> &Arc<TextureAtlasPlan> {
        &self.plan
    }

    /// The texels: straight sRGB RGBA8, rows bottom first, ready to upload, `width * height * 4`
    /// of them; `None` once [`GpuTextureAtlas::take_bytes`] took them, and for a pack given its
    /// plan, which builds none.
    pub fn bytes(&self) -> Option<&[u8]> {
        self.bytes.as_deref()
    }

    /// The texels, taken out so the atlas no longer holds them (a caller that has uploaded them
    /// keeps one copy, on the GPU); `None` when they were taken before. The size, placements and
    /// shrink report stay.
    pub fn take_bytes(&mut self) -> Option<Vec<u8>> {
        self.bytes.take()
    }

    /// The atlas's size in bytes, 4 a texel, whether or not its bytes were taken: what the GPU
    /// texture takes.
    pub fn byte_len(&self) -> u64 {
        self.plan.byte_len()
    }

    /// Where each picture lies, by its index in the scene's textures.
    pub fn placements(&self) -> &[AtlasPlacement] {
        &self.plan.placements
    }

    /// The pictures held smaller than given, in index order; empty when every picture fit.
    pub fn shrunk(&self) -> &[ShrunkTexture] {
        &self.plan.shrunk
    }
}

/// `textures` laid into one atlas within `limits`: [`plan_texture_atlas`], then
/// [`TextureAtlasPlan::build`]; see the module header for the cost.
pub fn fit_texture_atlas<T: Borrow<TraceTexture>>(
    textures: &[T],
    limits: GpuTextureLimits,
) -> Result<GpuTextureAtlas, FlatTreeError> {
    plan_texture_atlas(textures, limits)?.build(textures)
}

/// Where `textures` go in an atlas within `limits` and which are halved how often, the largest
/// first until they fit, from their sizes alone (no texel is read). Refused with
/// [`FlatTreeError::TexturesTooLarge`] when even 1 by 1 pictures do not fit.
pub fn plan_texture_atlas<T: Borrow<TraceTexture>>(
    textures: &[T],
    limits: GpuTextureLimits,
) -> Result<TextureAtlasPlan, FlatTreeError> {
    let given: Vec<(u32, u32)> = textures
        .iter()
        .map(Borrow::borrow)
        .map(|t: &TraceTexture| (t.width(), t.height()))
        .collect();
    if given.is_empty() {
        return Ok(TextureAtlasPlan {
            width: 1,
            height: 1,
            given,
            placements: Vec::new(),
            shrunk: Vec::new(),
        });
    }
    let mut current = given.clone();
    let mut halvings = vec![0u32; given.len()];
    loop {
        if let Some((width, height, placements)) = layout(&current, limits) {
            let shrunk = halvings
                .iter()
                .enumerate()
                .filter(|(_, &count)| count > 0)
                .map(|(index, &count)| ShrunkTexture {
                    // The scene numbers its pictures with `u32`
                    // (`RenderScene::with_face_textures`).
                    texture: index as u32,
                    halvings: count,
                    width: current[index].0,
                    height: current[index].1,
                })
                .collect();
            return Ok(TextureAtlasPlan {
                width,
                height,
                given,
                placements,
                shrunk,
            });
        }
        let over = |(w, h): (u32, u32)| w > limits.max_side || h > limits.max_side;
        let area = |(w, h): (u32, u32)| u64::from(w) * u64::from(h);
        let largest = (0..current.len())
            .max_by_key(|&i| (over(current[i]), area(current[i]), std::cmp::Reverse(i)))
            .unwrap_or(0);
        let (w, h) = current[largest];
        if w == 1 && h == 1 {
            return Err(FlatTreeError::TexturesTooLarge {
                // A count of pictures fits a u64 on every platform.
                pictures: given.len() as u64,
                max_texels: limits.max_texels,
                max_side: limits.max_side,
            });
        }
        // The same sizes `TraceTexture::halved` gives.
        current[largest] = (w.div_ceil(2), h.div_ceil(2));
        halvings[largest] += 1;
    }
}

/// The least-area layout of pictures of `sizes` within `limits` (see the module header): the
/// atlas's width and height and each picture's placement, `None` when none fits.
fn layout(
    sizes: &[(u32, u32)],
    limits: GpuTextureLimits,
) -> Option<(u32, u32, Vec<AtlasPlacement>)> {
    let side = u64::from(limits.max_side);
    let widest = sizes.iter().map(|&(w, _)| u64::from(w)).max()?;
    let tallest = sizes.iter().map(|&(_, h)| u64::from(h)).max()?;
    let total = sizes.iter().fold(0u64, |sum, &(w, h)| {
        sum.saturating_add(u64::from(w) * u64::from(h))
    });
    if widest > side || tallest > side || total > limits.max_texels {
        return None;
    }
    let mut order: Vec<usize> = (0..sizes.len()).collect();
    order.sort_by(|&a, &b| {
        let (wa, ha) = sizes[a];
        let (wb, hb) = sizes[b];
        hb.cmp(&ha).then(wb.cmp(&wa)).then(a.cmp(&b))
    });
    let root = ceil_sqrt(total);
    let mut candidates = Vec::new();
    let mut width = widest.max(root).min(side);
    while width < side {
        candidates.push(width);
        width = width.saturating_mul(2);
    }
    candidates.push(side);
    let mut best: Option<(u64, u64, u64, Vec<[u64; 2]>)> = None;
    for width in candidates {
        let (used, height, origins) = shelves(sizes, &order, width);
        let area = used.saturating_mul(height);
        let fits = height <= side && area <= limits.max_texels;
        if fits
            && best
                .as_ref()
                .is_none_or(|(best_area, ..)| area < *best_area)
        {
            best = Some((area, used, height, origins));
        }
    }
    let (_, used, height, origins) = best?;
    // Each coordinate is within the atlas, whose sides are at most `max_side`, a u32.
    let placements = sizes
        .iter()
        .zip(&origins)
        .map(|(&(width, height), origin)| AtlasPlacement {
            x: origin[0] as u32,
            y: origin[1] as u32,
            width,
            height,
        })
        .collect();
    Some((used as u32, height as u32, placements))
}

/// Shelf placement of `sizes` in `order` within `width` (at least the widest): the widest shelf,
/// the total height and each picture's origin, by index.
fn shelves(sizes: &[(u32, u32)], order: &[usize], width: u64) -> (u64, u64, Vec<[u64; 2]>) {
    let mut origins = vec![[0u64; 2]; sizes.len()];
    let (mut x, mut y, mut shelf_height, mut used) = (0u64, 0u64, 0u64, 0u64);
    for &index in order {
        let (w, h) = (u64::from(sizes[index].0), u64::from(sizes[index].1));
        if x > 0 && x + w > width {
            y += shelf_height;
            x = 0;
            shelf_height = 0;
        }
        origins[index] = [x, y];
        x += w;
        used = used.max(x);
        shelf_height = shelf_height.max(h);
    }
    (used, y + shelf_height, origins)
}

/// The smallest whole number whose square is at least `value`.
fn ceil_sqrt(value: u64) -> u64 {
    let root = value.isqrt();
    if root * root < value {
        root + 1
    } else {
        root
    }
}
