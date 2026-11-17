//! Pictures on faces, packed for the WGSL path loop: per pictured face, its three texture
//! coordinates and where its picture lies in the atlas (`gpu_textures.rs`), as plain `u32` words
//! at the end of the part buffer, and [`texture_lookup_f32`], the shader's lookup
//! (`path_picture_at` in `picture.wgsl`, whose text a test pins to these steps) against those words
//! and the atlas's bytes in `f32`, for checking the packed data against the CPU's lookup
//! (`path_tracer/trace_texture.rs`) without a graphics card. `gpu_pack.rs` builds the block when
//! it finishes a pack, after the glass contacts (`gpu_contacts.rs`); `gpu_scene.rs` carries it.
//!
//! **The block rides in the part buffer**, after the part records and the glass contact block,
//! for the reason the contacts do: the path loop's storage bindings are spoken for. A face's
//! [`super::FaceRecord::texture`] is the word offset in the part buffer of its face entry, or
//! [`NO_TEXTURE`] (0, where part 0's record lies, so no entry can start there) for a face showing
//! no picture or without texture coordinates. A scene without pictures appends nothing and
//! leaves every face's word 0. Otherwise the block is one picture record per scene picture,
//! [`TEXTURE_PICTURE_WORDS`] words each (atlas origin x and y, then width and height, in texels),
//! shared by every face showing it, then one face entry per pictured face in `source_index`
//! order, [`TEXTURE_FACE_WORDS`] words each: the corners' coordinates `u0, v0, u1, v1, u2, v2` as
//! `f32` bits, then the word offset of its picture's record.
//!
//! **Each face's coordinates are shifted** by a whole number on each axis, the midpoint of its
//! smallest and largest corner rounded toward 0, which Repeat wrapping cannot see. A design's
//! coordinates far from 0 (a thousand repeats out) would otherwise leave an `f32` coordinate a
//! large part of a texel coarse in a large picture; shifted, they reach only about half the
//! face's span from 0. Rounding the midpoint toward 0 never moves the face's largest coordinate
//! farther from 0, so a face centred on 0 is not shifted at all and loses nothing. Subtracting a
//! whole number is exact for a corner it moves toward 0; a corner it moves away from 0 (the face
//! straddles the whole number) can round, by at most half an `f32` step of the face's largest
//! coordinate as the mesh holds it, less than the step that coordinate already had. So packed
//! coordinates mean something only after wrapping, and only within their own face: two faces'
//! packed coordinates can differ by whole repeats where the mesh's are equal, and a shader (or a
//! seam that reads a second face's picture) must never compare or difference them across faces.
//! Seven
//! words a face and four a picture keep a design of 44,568 pictured triangles to about 1.2 MB.
//!
//! **The lookup** follows `trace_texture`'s order step for step in `f32`: the coordinate is the
//! corners' interpolated by the hit's weights (`uv0 * b0 + uv1 * b1 + uv2 * b2`, as the CPU's
//! surface point), each side wraps and splits into two texel indices and a fraction exactly as
//! the CPU's `texel_spot` does with the picture's own size, and only then is the picture's atlas
//! origin added, so wrapping never leaves the picture's rectangle. Each texel decodes through the
//! packed sRGB table (the shader's `path_srgb`), lays over white by its alpha, and the four blend
//! bottom, top, then up. The side's texel indices take a euclidean remainder, which the shader
//! writes as `u32(at + count) % u32(count)` (an index is at least -1, so nothing negative is ever
//! divided: a card read the wrong texel from a signed `%` of -1); a coordinate's wrap, its split
//! and the blend are the shader's expressions in the shader's order.

use super::{AtlasPlacement, FaceRecord, FlatTreeError, SRGB_TABLE_BYTES};
use crate::ray_tracer::RenderScene;

/// A face's texture word when it shows no picture.
pub const NO_TEXTURE: u32 = 0;
/// Words per face entry: six coordinates and the picture record's offset.
pub const TEXTURE_FACE_WORDS: usize = 7;
/// A face entry's picture record offset, in words from the entry's first.
pub const TEXTURE_FACE_PICTURE: usize = 6;
/// Words per picture record: atlas origin x and y, then width and height.
pub const TEXTURE_PICTURE_WORDS: usize = 4;
/// A picture record's size, in words from the record's first.
pub const TEXTURE_PICTURE_SIZE: usize = 2;

/// The packed picture block's words; empty for a scene without pictures.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct GpuTextureBlock {
    words: Vec<u32>,
}

impl GpuTextureBlock {
    /// The block's words, the tail of the part buffer.
    pub fn words(&self) -> &[u32] {
        &self.words
    }

    /// The block's bytes, little-endian.
    pub fn to_le_bytes(&self) -> Vec<u8> {
        self.words
            .iter()
            .flat_map(|word| word.to_le_bytes())
            .collect()
    }
}

/// Packs the picture block of `scene` starting at word `first_word` of the part buffer, its picture
/// records from `placements`, setting each pictured face's [`FaceRecord::texture`]. Sized before
/// anything is written: refused when the part buffer would pass `max_storage_binding`, and,
/// for a binding larger than 16 GiB, when a word offset would pass `u32`.
pub(super) fn pack_texture_block(
    scene: &RenderScene,
    faces: &mut [FaceRecord],
    placements: &[AtlasPlacement],
    first_word: u64,
    max_storage_binding: u64,
) -> Result<GpuTextureBlock, FlatTreeError> {
    if scene.textures().is_empty() {
        return Ok(GpuTextureBlock::default());
    }
    let parts = scene.parts().parts();
    let pictured = |part_index: usize, face: usize| {
        let table = parts[part_index].surface();
        scene
            .face_texture(part_index, face)
            .zip(table.face_texture_corners(face))
    };
    let mut entries = 0u64;
    for (part_index, part) in parts.iter().enumerate() {
        for face in 0..part.surface().face_count() {
            if pictured(part_index, face).is_some() {
                entries += 1;
            }
        }
    }
    // Picture and entry counts fit a u64 on every platform.
    let picture_words = (placements.len() * TEXTURE_PICTURE_WORDS) as u64;
    let total_words = first_word + picture_words + entries * TEXTURE_FACE_WORDS as u64;
    let limit_words = max_storage_binding / 4;
    if total_words * 4 > max_storage_binding {
        let (what, largest_that_fits) = if first_word + picture_words > limit_words {
            (
                "pictures",
                limit_words.saturating_sub(first_word) / TEXTURE_PICTURE_WORDS as u64,
            )
        } else {
            (
                "pictured faces",
                (limit_words - first_word - picture_words) / TEXTURE_FACE_WORDS as u64,
            )
        };
        return Err(FlatTreeError::PackedBufferTooLarge {
            buffer: "part (with its pictures)",
            needed_bytes: total_words * 4,
            limit_bytes: max_storage_binding,
            what,
            largest_that_fits,
        });
    }
    if u32::try_from(total_words).is_err() {
        return Err(FlatTreeError::NotRepresentable {
            reason: "the part buffer with its pictures has more words than a u32 can index",
        });
    }
    // Every offset below is under `total_words`, checked to fit a u32, and the block's words
    // number at most that, so the casts cannot truncate.
    let mut words = Vec::with_capacity((total_words - first_word) as usize);
    for place in placements {
        words.extend_from_slice(&[place.x, place.y, place.width, place.height]);
    }
    let mut triangle = 0usize;
    for (part_index, part) in parts.iter().enumerate() {
        for face in 0..part.surface().face_count() {
            if let (Some((picture, corners)), Some(record)) =
                (pictured(part_index, face), faces.get_mut(triangle))
            {
                record.texture = (first_word + words.len() as u64) as u32;
                let shift = |values: [f32; 3]| {
                    let low = values[0].min(values[1]).min(values[2]);
                    let high = values[0].max(values[1]).max(values[2]);
                    // Halved before adding so two large coordinates cannot overflow.
                    let whole = (low * 0.5 + high * 0.5).trunc();
                    if whole.is_finite() {
                        whole
                    } else {
                        0.0
                    }
                };
                let du = shift(corners.map(|c| c.x));
                let dv = shift(corners.map(|c| c.y));
                for corner in corners {
                    words
                        .extend_from_slice(&[(corner.x - du).to_bits(), (corner.y - dv).to_bits()]);
                }
                // `with_face_textures` keeps every picture index below the picture count.
                words.push((first_word + u64::from(picture) * TEXTURE_PICTURE_WORDS as u64) as u32);
            }
            triangle += 1;
        }
    }
    Ok(GpuTextureBlock { words })
}

/// What [`texture_lookup_f32`] read: the coordinate, the four atlas texels (bottom-left,
/// bottom-right, top-left, top-right, each `[x, y]`), the fractions between them and the factor.
/// A mirror of the shader for tests, not an API to render with.
#[doc(hidden)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TextureLookupF32 {
    pub uv: [f32; 2],
    pub texels: [[u32; 2]; 4],
    pub across: f32,
    pub up: f32,
    pub factor: [f32; 3],
}

/// The shader's picture lookup in `f32` (see the module header): for the face whose texture word
/// is `texture`, at hit weights `barycentric`, reading the part buffer's `part_words`, the atlas
/// `atlas` (`atlas_width` texels wide) and the packed sRGB table. `None` for [`NO_TEXTURE`] or a
/// word or texel past its buffer, whatever the words hold: a mirror of the shader for tests.
#[doc(hidden)]
pub fn texture_lookup_f32(
    part_words: &[u32],
    texture: u32,
    barycentric: [f32; 3],
    atlas: &[u8],
    atlas_width: u32,
    srgb_table: &[u8; SRGB_TABLE_BYTES],
) -> Option<TextureLookupF32> {
    if texture == NO_TEXTURE {
        return None;
    }
    let entry = words_at(part_words, texture, TEXTURE_FACE_WORDS)?;
    let coordinate = |at: usize| f32::from_bits(entry[at]);
    let uv: [f32; 2] = std::array::from_fn(|axis| {
        coordinate(axis) * barycentric[0]
            + coordinate(2 + axis) * barycentric[1]
            + coordinate(4 + axis) * barycentric[2]
    });
    let picture = words_at(
        part_words,
        entry[TEXTURE_FACE_PICTURE],
        TEXTURE_PICTURE_WORDS,
    )?;
    let (columns, across) = side_spot_f32(uv[0], picture[TEXTURE_PICTURE_SIZE]);
    let (rows, up) = side_spot_f32(uv[1], picture[TEXTURE_PICTURE_SIZE + 1]);
    let at_origin =
        |[x, y]: [u32; 2]| Some([x.checked_add(picture[0])?, y.checked_add(picture[1])?]);
    let texels = [
        at_origin([columns[0], rows[0]])?,
        at_origin([columns[1], rows[0]])?,
        at_origin([columns[0], rows[1]])?,
        at_origin([columns[1], rows[1]])?,
    ];
    let decode = |code: u8| {
        let at = usize::from(code) * 4;
        f32::from_le_bytes([
            srgb_table[at],
            srgb_table[at + 1],
            srgb_table[at + 2],
            srgb_table[at + 3],
        ])
    };
    let mut factors = [[0.0f32; 3]; 4];
    for (factor, [x, y]) in factors.iter_mut().zip(texels) {
        if x >= atlas_width {
            return None;
        }
        let at = usize::try_from(u64::from(y) * u64::from(atlas_width) + u64::from(x))
            .ok()?
            .checked_mul(4)?;
        let texel = atlas.get(at..at.checked_add(4)?)?;
        let alpha = f32::from(texel[3]) / 255.0;
        let clear = 1.0 - alpha;
        *factor = std::array::from_fn(|channel| alpha * decode(texel[channel]) + clear);
    }
    let lerp = |a: f32, b: f32, t: f32| a + t * (b - a);
    let [bottom_left, bottom_right, top_left, top_right] = factors;
    let factor = std::array::from_fn(|channel| {
        let bottom = lerp(bottom_left[channel], bottom_right[channel], across);
        let top = lerp(top_left[channel], top_right[channel], across);
        lerp(bottom, top, up)
    });
    Some(TextureLookupF32 {
        uv,
        texels,
        across,
        up,
        factor,
    })
}

/// The `count` words of `words` from `start`, `None` past its end.
fn words_at(words: &[u32], start: u32, count: usize) -> Option<&[u32]> {
    let start = usize::try_from(start).ok()?;
    words.get(start..start.checked_add(count)?)
}

/// One side's two texel indices inside a picture of `size` texels and the fraction between
/// them, the CPU's `side_spot` in `f32`.
fn side_spot_f32(coordinate: f32, size: u32) -> ([u32; 2], f32) {
    let size = size.max(1);
    let wrapped = if coordinate.is_finite() {
        coordinate - coordinate.floor()
    } else {
        0.0
    };
    // A side is at most a texture's largest side, far below 2^24, so it is exact in f32.
    let x = wrapped * size as f32 - 0.5;
    let x0 = x.floor();
    let fraction = x - x0;
    // `x0` is a whole number in [-1, size - 1], so the cast is exact.
    let first = x0 as i64;
    let size = i64::from(size);
    // `rem_euclid` by a positive `size` lands in [0, size), which fits a `u32`. For `at` in
    // [-1, size] it equals the shader's `u32(at + count) % u32(count)`, which takes no remainder
    // of a negative value (a card read the wrong texel from WGSL's signed `%` of -1).
    let index = |at: i64| at.rem_euclid(size) as u32;
    ([index(first), index(first + 1)], fraction)
}
