//! [`TextureAtlasBuild`]: the picture atlas of a [`TextureAtlasPlan`] (`gpu_textures.rs`) built in
//! bounded pieces, for a host with no threads (a browser's UI thread) that must keep every piece
//! of work inside a frame. [`TextureAtlasPlan::build`] is this run in one call, and
//! [`GpuSceneBuilder`](super::GpuSceneBuilder) drives it inside its own work units, so the atlas
//! comes out the same bytes however the work is cut.
//!
//! **Work units.** A step does the units it is given, and always some work, so a host loop that
//! keeps stepping always ends; a halved texel is never cut, so a step can pass its units by less
//! than one halved texel's cost. Clearing one atlas texel costs 1, copying one picture texel
//! costs 1, and writing one texel of a halving costs [`ATLAS_WORK_PER_HALVED_TEXEL`]: a halved
//! texel decodes, weighs and re-encodes four source texels, which natively takes some 50 to 110
//! times as long as clearing or copying one, so a host that measures what a unit costs on one kind
//! of work is not far off on the other. [`TextureAtlasPlan::build_work`] is the whole build's
//! count.
//!
//! **The order**, fixed, so a slice boundary changes nothing but where a step stops: the atlas's
//! bytes are reserved on the first step (fallibly: [`FlatTreeError::OutOfMemory`]) and cleared to
//! 0 a run of texels at a time; then each picture in index order, a picture the plan shrinks
//! first halved as many times as the plan says (`path_tracer/texture_halving.rs`, its bytes
//! reserved as fallibly), then copied into its place row by row, bottom first. A scene without
//! pictures gets its 1 by 1 white atlas on the first step.
//!
//! **Whose pictures.** A build holds its plan, shared, and is handed the pictures on every step,
//! so it can live across frames beside the scene that owns them. Every step checks they have the
//! count and sizes the plan was made for; the caller hands the same pictures each time.

use std::borrow::Borrow;
use std::sync::Arc;

use super::gpu_textures::{GpuTextureAtlas, TextureAtlasPlan, TEXEL_BYTES};
use super::FlatTreeError;
use crate::ray_tracer::path_tracer::ShrinkCursor;
use crate::ray_tracer::TraceTexture;

/// The units one texel of a halving costs in a [`TextureAtlasBuild`], where clearing or copying a
/// texel costs 1; see the module header.
pub const ATLAS_WORK_PER_HALVED_TEXEL: usize = 64;

/// A picture being placed: halved first when the plan shrinks it, then copied a row at a time.
#[derive(Debug, Clone)]
enum Placing {
    Shrinking(ShrinkCursor),
    Copying {
        /// The shrunk picture, `None` for one placed as given.
        smaller: Option<TraceTexture>,
        row: u32,
        column: u32,
    },
}

/// An atlas being built; see the module header.
#[derive(Debug, Clone)]
pub struct TextureAtlasBuild {
    plan: Arc<TextureAtlasPlan>,
    /// `None` until the first step reserves them, and again once the atlas was answered.
    bytes: Option<Vec<u8>>,
    /// The picture being placed, by index.
    picture: usize,
    placing: Option<Placing>,
    /// How many of the plan's shrunk pictures are placed.
    next_shrunk: usize,
    work: u64,
    finished: bool,
}

impl TextureAtlasBuild {
    /// A build of `plan`'s atlas, nothing done yet.
    pub fn new(plan: Arc<TextureAtlasPlan>) -> Self {
        TextureAtlasBuild {
            plan,
            bytes: None,
            picture: 0,
            placing: None,
            next_shrunk: 0,
            work: 0,
            finished: false,
        }
    }

    /// The plan being built.
    pub fn plan(&self) -> &Arc<TextureAtlasPlan> {
        &self.plan
    }

    /// The work done so far, in the module header's units: equal to the plan's
    /// [`TextureAtlasPlan::build_work`] once the atlas was answered.
    pub fn work_done(&self) -> u64 {
        self.work
    }

    /// Does `max_work` units of the build (at least one, and past it by less than one halved
    /// texel's cost at most) on `textures`, the pictures the
    /// plan was made for (a different count or size is [`FlatTreeError::NotRepresentable`]), and
    /// answers the atlas once it is built: the same bytes as [`TextureAtlasPlan::build`] however
    /// the work is cut. Refused with [`FlatTreeError::OutOfMemory`] when the atlas's bytes or a
    /// shrunk picture's cannot be allocated; a step after the atlas was answered is refused, never
    /// a second atlas.
    pub fn step<T: Borrow<TraceTexture>>(
        &mut self,
        textures: &[T],
        max_work: usize,
    ) -> Result<Option<GpuTextureAtlas>, FlatTreeError> {
        let mut budget = max_work.max(1);
        self.step_within(textures, &mut budget)
    }

    /// [`TextureAtlasBuild::step`] spending `budget`, which a pack shares with its other work;
    /// nothing is done when it is 0.
    pub(super) fn step_within<T: Borrow<TraceTexture>>(
        &mut self,
        textures: &[T],
        budget: &mut usize,
    ) -> Result<Option<GpuTextureAtlas>, FlatTreeError> {
        if self.finished {
            return Err(FlatTreeError::NotRepresentable {
                reason: "this picture atlas was already built",
            });
        }
        if !self.plan.fits_sizes_of(textures) {
            return Err(FlatTreeError::NotRepresentable {
                reason: "these pictures are not the ones the atlas plan was made for",
            });
        }
        if *budget == 0 {
            return Ok(None);
        }
        let plan = Arc::clone(&self.plan);
        let byte_count = self.reserve(textures.is_empty())?;
        let Some(bytes) = self.bytes.as_mut() else {
            return Ok(None);
        };
        if bytes.len() < byte_count {
            // Every count here is under the atlas's byte count, which fits a `usize`.
            let texels = ((byte_count - bytes.len()) / TEXEL_BYTES).min(*budget);
            bytes.resize(bytes.len() + texels * TEXEL_BYTES, 0);
            *budget -= texels;
            self.work += texels as u64;
            if bytes.len() < byte_count {
                return Ok(None);
            }
        }
        // Every placement lies inside the atlas, whose byte count fits a `usize` (checked when
        // reserved), so the casts and products below cannot overflow.
        let atlas_row = plan.width as usize * TEXEL_BYTES;
        let pictures = textures.iter().map(Borrow::<TraceTexture>::borrow);
        for (texture, place) in pictures.zip(&plan.placements).skip(self.picture) {
            if *budget == 0 {
                return Ok(None);
            }
            let shrunk = plan
                .shrunk
                .get(self.next_shrunk)
                .filter(|s| s.texture as usize == self.picture);
            let placing = self.placing.get_or_insert_with(|| match shrunk {
                Some(shrunk) => Placing::Shrinking(ShrinkCursor::new(texture, shrunk.halvings)),
                None => Placing::Copying {
                    smaller: None,
                    row: 0,
                    column: 0,
                },
            });
            if let Placing::Shrinking(cursor) = placing {
                let before = cursor.texels_written();
                let halved = cursor
                    .step(texture, budget, ATLAS_WORK_PER_HALVED_TEXEL)
                    .map_err(|_| FlatTreeError::OutOfMemory {
                        what: "shrunk picture",
                        // The first halving's bytes, the largest it asks for.
                        bytes: u64::from(texture.width().div_ceil(2))
                            * u64::from(texture.height().div_ceil(2))
                            * TEXEL_BYTES as u64,
                    });
                let halved_texels = cursor.texels_written() - before;
                self.work += halved_texels * ATLAS_WORK_PER_HALVED_TEXEL as u64;
                match halved? {
                    Some(smaller) => {
                        *placing = Placing::Copying {
                            smaller: Some(smaller),
                            row: 0,
                            column: 0,
                        }
                    }
                    None => return Ok(None),
                }
                if *budget == 0 {
                    return Ok(None);
                }
            }
            let Placing::Copying {
                smaller,
                row,
                column,
            } = placing
            else {
                return Ok(None);
            };
            let picture = smaller.as_ref().unwrap_or(texture).bytes();
            let row_bytes = place.width as usize * TEXEL_BYTES;
            while *row < place.height && *budget > 0 {
                let count = ((place.width - *column) as usize).min(*budget);
                let to = (place.y + *row) as usize * atlas_row
                    + (place.x + *column) as usize * TEXEL_BYTES;
                let from = *row as usize * row_bytes + *column as usize * TEXEL_BYTES;
                let length = count * TEXEL_BYTES;
                bytes[to..to + length].copy_from_slice(&picture[from..from + length]);
                *budget -= count;
                self.work += count as u64;
                // `count` is at most the columns left in the row, a `u32`.
                *column += count as u32;
                if *column == place.width {
                    *column = 0;
                    *row += 1;
                }
            }
            if *row < place.height {
                return Ok(None);
            }
            if shrunk.is_some() {
                self.next_shrunk += 1;
            }
            self.placing = None;
            self.picture += 1;
        }
        self.finished = true;
        Ok(Some(GpuTextureAtlas {
            plan,
            bytes: self.bytes.take(),
        }))
    }

    /// Reserves the atlas's bytes on the first step (the white texel of a scene without pictures
    /// written at once) and answers the atlas's byte count.
    fn reserve(&mut self, no_pictures: bool) -> Result<usize, FlatTreeError> {
        let out_of_memory = || FlatTreeError::OutOfMemory {
            what: "picture atlas",
            bytes: self.plan.byte_len(),
        };
        let byte_count = usize::try_from(self.plan.byte_len()).map_err(|_| out_of_memory())?;
        if self.bytes.is_none() {
            let mut bytes = Vec::new();
            bytes
                .try_reserve_exact(byte_count)
                .map_err(|_| out_of_memory())?;
            if no_pictures {
                bytes.extend_from_slice(&[255; TEXEL_BYTES]);
                self.work += 1;
            }
            self.bytes = Some(bytes);
        }
        Ok(byte_count)
    }
}
