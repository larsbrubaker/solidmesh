//! [`TraceTexture`], a decoded picture a face of a [`RenderScene`](super::RenderScene) shows, and
//! the lookup that turns a texture coordinate into the factor the face's color is multiplied by.
//! The scene holds the pictures and which face uses which (`render_scene`); the path tracer reads
//! the factor at each hit through `RenderScene::surface_at_point`, and the graphics card's path
//! loop follows the same steps in `f32` (`flat_tree`'s `picture.wgsl`).
//!
//! **The picture.** Straight (not premultiplied) sRGB RGBA, 8 bits a channel, rows bottom first,
//! so texture coordinate (0, 0) is the picture's bottom-left corner and (1, 1) its top-right.
//!
//! **The lookup**, in this order, so a mirror in another precision can follow it step for step:
//! 1. [`texel_spot`]: each coordinate wraps by Repeat, `t - floor(t)` (a non-finite coordinate
//!    reads as 0), then `x = wrapped * width - 0.5`, `x0 = floor(x)`, `across = x - x0`; the two
//!    columns are `x0` and `x0 + 1`, each taken modulo the width (`rem_euclid`). Rows the same
//!    from `v` and the height, giving `up`. Texel centres sit at `(i + 0.5) / width`.
//! 2. [`texel_factor`] for each of the four texels: each color channel decoded through the
//!    scene's 256-entry sRGB table, alpha as `byte / 255`, then `alpha * decoded + (1 - alpha)`:
//!    the texel over white. Alpha 1 gives the decoded color and alpha 0 gives exactly 1, so a
//!    see-through texel shows the face's own color and never cuts a hole.
//! 3. [`blend_factors`]: `bottom = lerp(bottom-left, bottom-right, across)`,
//!    `top = lerp(top-left, top-right, across)`, `factor = lerp(bottom, top, up)`, with
//!    `lerp(a, b, t) = a + t * (b - a)`, so equal texels blend to themselves exactly.
//!
//! Blending factors that are already over white is the same as blending premultiplied colors
//! and alphas and then compositing, so a picture's see-through edge filters as it should. There
//! are no mip levels: level 0 only.
//!
//! **Halving** ([`TraceTexture::halved`]) is how a picture too large for the GPU's atlas
//! (`flat_tree/gpu_textures.rs`) is made smaller, and the CPU route uses the very same call so
//! both routes show the same texels. Each side becomes `side.div_ceil(2)` (a side of 1 stays 1),
//! and new texel `(i, j)` averages the 2 by 2 block of source columns `2i`, `2i + 1` and rows
//! `2j`, `2j + 1`, an index past the last one clamped to it (so an odd side's last column is
//! averaged with itself and no texel is dropped). The average is of light, not of codes: the new
//! alpha is the mean of the four alpha bytes (`round_ties_even`), and each color channel is
//! `sum(alpha * decoded) / sum(alpha)` over the four (decoded through the same sRGB table the
//! lookup uses, alphas as bytes, summed in block order), encoded back as the byte whose decoded
//! value is nearest (the lower byte on a tie). Weighting by alpha keeps the color of a see-through
//! texel out of its opaque neighbours, and averaging decoded values keeps a fine pattern's
//! brightness, where averaging sRGB codes would darken it. Alpha stays straight: a block that is
//! entirely see-through has color bytes 0. The halved texels' factors are then the average of
//! the four source factors, up to the bytes' rounding. `texture_halving` runs the halving in
//! pieces ([`TextureHalving`], [`TextureShrinking`]) for a host that must keep each piece of work
//! inside a frame; the one-call forms here go through the same code.

use std::collections::TryReserveError;

use solidmesh_math::Vector2;

use super::render_scene::decoded_table;
use super::texture_halving::{halve_texels, TextureHalving, TextureShrinking};
use crate::ray_tracer::RayTracerError;

/// Bytes per texel: red, green, blue, alpha.
pub(super) const TEXEL_BYTES: usize = 4;

/// A decoded picture for the path tracer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TraceTexture {
    width: u32,
    height: u32,
    bytes: Vec<u8>,
}

/// Which four texels a texture coordinate reads, and how much of each.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TexelSpot {
    /// The left and right columns, each below the width.
    pub columns: [u32; 2],
    /// The lower and upper rows (row 0 is the bottom), each below the height.
    pub rows: [u32; 2],
    /// How far from the left column toward the right one, in [0, 1).
    pub across: f64,
    /// How far from the lower row toward the upper one, in [0, 1).
    pub up: f64,
}

impl TraceTexture {
    /// A `width` by `height` picture from `bytes`: straight sRGB RGBA8, rows bottom first. A zero
    /// size, or a byte count other than `width * height * 4`, is
    /// [`RayTracerError::InvalidArgument`].
    pub fn new(width: u32, height: u32, bytes: Vec<u8>) -> Result<Self, RayTracerError> {
        if width == 0 || height == 0 {
            return Err(RayTracerError::InvalidArgument(
                "a texture needs a width and a height of at least one texel",
            ));
        }
        let expected = usize::try_from(width)
            .ok()
            .zip(usize::try_from(height).ok())
            .and_then(|(width, height)| width.checked_mul(height))
            .and_then(|texels| texels.checked_mul(TEXEL_BYTES));
        if expected != Some(bytes.len()) {
            return Err(RayTracerError::InvalidArgument(
                "a texture needs exactly width x height x 4 bytes (RGBA, 8 bits a channel)",
            ));
        }
        Ok(TraceTexture {
            width,
            height,
            bytes,
        })
    }

    /// The width in texels.
    pub fn width(&self) -> u32 {
        self.width
    }

    /// The height in texels.
    pub fn height(&self) -> u32 {
        self.height
    }

    /// The texels: straight sRGB RGBA8, rows bottom first.
    pub fn bytes(&self) -> &[u8] {
        &self.bytes
    }

    /// This picture at half its size on each side, as the module header describes: the shrink
    /// the GPU atlas applies to a picture it cannot fit, for a CPU render to show the same
    /// texels. A 1 by 1 picture comes back unchanged. Its bytes are allocated infallibly, so a
    /// device without a quarter of this picture's bytes to spare aborts:
    /// [`TraceTexture::try_halved`] refuses instead.
    pub fn halved(&self) -> TraceTexture {
        let size = (self.width.div_ceil(2), self.height.div_ceil(2));
        // The halved picture has fewer texels than this one, whose byte count fits a `usize`.
        let texels = size.0 as usize * size.1 as usize;
        let mut bytes = Vec::with_capacity(texels * TEXEL_BYTES);
        let (mut row, mut column) = (0, 0);
        halve_texels(
            self,
            size,
            (&mut row, &mut column),
            &mut bytes,
            texels,
            &decoded_table(),
        );
        TraceTexture::from_parts(size.0, size.1, bytes)
    }

    /// [`TraceTexture::halved`], refused when its bytes cannot be allocated. In one call:
    /// [`TextureHalving`] gives the same picture in pieces.
    pub fn try_halved(&self) -> Result<TraceTexture, TryReserveError> {
        let mut halving = TextureHalving::new(self);
        loop {
            if let Some(halved) = halving.step(usize::MAX)? {
                return Ok(halved);
            }
        }
    }

    /// [`TraceTexture::shrunk`], refused when a halving's bytes cannot be allocated; 0 halvings
    /// is a copy, allocated as fallibly. In one call: [`TextureShrinking`] gives the same picture
    /// in pieces.
    pub fn try_shrunk(&self, halvings: u32) -> Result<TraceTexture, TryReserveError> {
        let mut shrinking = TextureShrinking::new(self, halvings);
        loop {
            if let Some(shrunk) = shrinking.step(usize::MAX)? {
                return Ok(shrunk);
            }
        }
    }

    /// A picture of `bytes`, which the caller has made exactly `width * height * 4` long.
    pub(super) fn from_parts(width: u32, height: u32, bytes: Vec<u8>) -> TraceTexture {
        TraceTexture {
            width,
            height,
            bytes,
        }
    }

    /// This picture halved `halvings` times ([`TraceTexture::halved`] repeated), the picture a
    /// GPU atlas holds for one reported as shrunk that many times.
    pub fn shrunk(&self, halvings: u32) -> TraceTexture {
        if halvings == 0 {
            return self.clone();
        }
        let mut picture = self.halved();
        for _ in 1..halvings {
            if picture.width == 1 && picture.height == 1 {
                break;
            }
            picture = picture.halved();
        }
        picture
    }

    /// The four texels `uv` reads and their weights ([`texel_spot`]).
    #[doc(hidden)]
    pub fn spot(&self, uv: Vector2) -> TexelSpot {
        texel_spot(uv, self.width, self.height)
    }

    /// The texel at `column`, `row` (row 0 the bottom), `None` past the picture.
    pub fn texel(&self, column: u32, row: u32) -> Option<[u8; 4]> {
        if column >= self.width || row >= self.height {
            return None;
        }
        // Each is below its side, and the whole picture's byte count fits a `usize` (checked
        // when it was made), so neither the casts nor the products can overflow.
        let index = (row as usize * self.width as usize + column as usize) * TEXEL_BYTES;
        let texel = self.bytes.get(index..index + TEXEL_BYTES)?;
        Some([texel[0], texel[1], texel[2], texel[3]])
    }

    /// What a face's color is multiplied by at `uv`, each channel's 8-bit value decoded through
    /// `decode` (the module header gives the order).
    pub(crate) fn factor_at(&self, uv: Vector2, decode: &[f64; 256]) -> [f64; 3] {
        let spot = self.spot(uv);
        let factor = |column: usize, row: usize| {
            // `spot` keeps its columns and rows inside the picture, so the texel is there.
            self.texel(spot.columns[column], spot.rows[row])
                .map_or([1.0; 3], |texel| texel_factor(texel, decode))
        };
        blend_factors(
            [factor(0, 0), factor(1, 0), factor(0, 1), factor(1, 1)],
            spot.across,
            spot.up,
        )
    }
}

/// The 2 by 2 block's average texel (see the module header): alpha the mean byte, color the
/// alpha-weighted mean of the decoded channels, encoded back to the nearest byte.
pub(super) fn box_average(block: [[u8; 4]; 4], decode: &[f64; 256]) -> [u8; 4] {
    let alpha_sum = block.iter().fold(0.0_f64, |sum, t| sum + f64::from(t[3]));
    // A mean of four bytes is in [0, 255], so the cast is exact.
    let mut average = [0, 0, 0, (alpha_sum / 4.0).round_ties_even() as u8];
    if alpha_sum > 0.0 {
        for (channel, out) in average.iter_mut().take(3).enumerate() {
            let weighted = block.iter().fold(0.0_f64, |sum, t| {
                sum + f64::from(t[3]) * decode[usize::from(t[channel])]
            });
            *out = nearest_code(weighted / alpha_sum, decode);
        }
    }
    average
}

/// The byte whose decoded value is nearest `linear`, the lower one on a tie (`decode` rises).
fn nearest_code(linear: f64, decode: &[f64; 256]) -> u8 {
    let above = decode.partition_point(|&value| value < linear);
    let code = match above {
        0 => 0,
        256 => 255,
        _ if decode[above] - linear < linear - decode[above - 1] => above,
        _ => above - 1,
    };
    // `code` is an index into a 256-entry table.
    code as u8
}

/// `coordinate` wrapped into [0, 1] by Repeat; a non-finite coordinate reads as 0.
fn wrapped(coordinate: f64) -> f64 {
    if coordinate.is_finite() {
        coordinate - coordinate.floor()
    } else {
        0.0
    }
}

/// The two texel indices along a side of `size` texels and how far between them `coordinate`
/// lies.
fn side_spot(coordinate: f64, size: u32) -> ([u32; 2], f64) {
    let x = wrapped(coordinate) * f64::from(size) - 0.5;
    let x0 = x.floor();
    let fraction = x - x0;
    // `x0` is a whole number in [-1, size - 1], so the cast is exact.
    let first = x0 as i64;
    let size = i64::from(size);
    // `rem_euclid` by a positive `size` lands in [0, size), which fits a `u32`.
    let index = |at: i64| at.rem_euclid(size) as u32;
    ([index(first), index(first + 1)], fraction)
}

/// The four texels a `width` by `height` picture shows at `uv` under Repeat wrapping and
/// bilinear filtering, and their weights (step 1 of the module header).
pub fn texel_spot(uv: Vector2, width: u32, height: u32) -> TexelSpot {
    let (columns, across) = side_spot(uv.x, width.max(1));
    let (rows, up) = side_spot(uv.y, height.max(1));
    TexelSpot {
        columns,
        rows,
        across,
        up,
    }
}

/// One texel's factor: its color decoded through `decode` and laid over white by its alpha
/// (step 2 of the module header).
pub(crate) fn texel_factor(texel: [u8; 4], decode: &[f64; 256]) -> [f64; 3] {
    let alpha = f64::from(texel[3]) / 255.0;
    let clear = 1.0 - alpha;
    let channel = |code: u8| alpha * decode[usize::from(code)] + clear;
    [channel(texel[0]), channel(texel[1]), channel(texel[2])]
}

/// `a` toward `b` by `t`, equal to `a` exactly when `a == b`.
fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + t * (b - a)
}

/// The bilinear blend of four texel factors, in the order bottom-left, bottom-right, top-left,
/// top-right (step 3 of the module header).
pub fn blend_factors(texels: [[f64; 3]; 4], across: f64, up: f64) -> [f64; 3] {
    let [bottom_left, bottom_right, top_left, top_right] = texels;
    std::array::from_fn(|channel| {
        let bottom = lerp(bottom_left[channel], bottom_right[channel], across);
        let top = lerp(top_left[channel], top_right[channel], across);
        lerp(bottom, top, up)
    })
}
