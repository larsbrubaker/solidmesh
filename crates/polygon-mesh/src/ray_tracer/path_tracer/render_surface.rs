//! [`RenderSurface`], where a render accumulates: per pixel, the sum of its samples' linear HDR
//! radiance and how many there are. [`RenderTile`] is a window onto it, handed out disjointly so
//! several threads can fill different tiles at once with no lock and no `unsafe`.
//!
//! Color is held in `f64` because the sum runs over thousands of samples: `f32` would lose the
//! low bits of each addition once the sum dwarfs a sample, and the cost is only 12 more bytes per
//! pixel than `f32` would use, small against the geometry a scene holds.
//!
//! **A pixel accepts only the next sample in order.** [`AccumulatedPixel::add_sample`] takes
//! `sample_index` and refuses one that isn't the count so far. A pass that was cancelled halfway
//! through a tile leaves its pixels with one count and the rest with another; running that pass
//! again adds the missing samples and doesn't double the others. So a pixel's sum is always the
//! same sum of the same samples, however a caller slices, reorders or repeats the work.
//!
//! **Bad values.** A sample that is not finite or is negative, or one that would overflow the
//! running sum, is dropped: the pixel's sum stands as it was and [`AccumulatedPixel::dropped`]
//! counts it. A dropped sample still uses up its index, so the sequence stays in step, and it is
//! left out of the mean, so one NaN never poisons a pixel.

use super::display::{display_byte, DISPLAY_OPAQUE};
use crate::ray_tracer::RayTracerError;

/// One pixel's accumulated samples.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct AccumulatedPixel {
    sum: [f64; 3],
    samples: u32,
    dropped: u32,
}

impl AccumulatedPixel {
    /// The samples that reached this pixel, dropped ones included.
    pub fn samples(&self) -> u32 {
        self.samples
    }

    /// The samples dropped for being not finite, negative or overflowing the sum.
    pub fn dropped(&self) -> u32 {
        self.dropped
    }

    /// The sum of the samples that counted, linear RGB.
    pub fn sum(&self) -> [f64; 3] {
        self.sum
    }

    /// The mean of the samples that counted, or black when none did.
    pub fn mean(&self) -> [f64; 3] {
        let counted = self.samples - self.dropped;
        if counted == 0 {
            [0.0; 3]
        } else {
            let counted = f64::from(counted);
            [
                self.sum[0] / counted,
                self.sum[1] / counted,
                self.sum[2] / counted,
            ]
        }
    }

    /// Adds `value` as sample `sample_index`. Returns `None` when `sample_index` is not the next
    /// in this pixel's sequence (nothing changes), `Some(true)` when the sample counted and
    /// `Some(false)` when it was dropped; see the module header for what gets dropped.
    pub fn add_sample(&mut self, sample_index: u32, value: [f64; 3]) -> Option<bool> {
        if sample_index != self.samples {
            return None;
        }
        self.samples += 1;
        let sum = [
            self.sum[0] + value[0],
            self.sum[1] + value[1],
            self.sum[2] + value[2],
        ];
        let usable = value
            .iter()
            .all(|channel| channel.is_finite() && *channel >= 0.0)
            && sum.iter().all(|channel| channel.is_finite());
        if usable {
            self.sum = sum;
        } else {
            self.dropped += 1;
        }
        Some(usable)
    }
}

/// A rectangle of pixels, in the surface's bottom-left-origin coordinates.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TileRect {
    pub x: u32,
    pub y: u32,
    pub width: u32,
    pub height: u32,
}

/// The refusal for a surface within [`RenderSurface::MAX_PIXELS`] that this host cannot hold.
fn too_large_for_host() -> RayTracerError {
    RayTracerError::InvalidArgument(
        "this host cannot provide memory for a render surface this large; the largest this library allows anywhere is 1073741824 pixels (32768 by 32768)",
    )
}

/// A linear HDR image being accumulated.
#[derive(Debug, Clone, PartialEq)]
pub struct RenderSurface {
    width: u32,
    height: u32,
    pixels: Vec<AccumulatedPixel>,
}

impl RenderSurface {
    /// The most pixels a surface may hold (32768 by 32768, 34 GB at the 32 bytes each pixel needs,
    /// the same for every platform so a refusal doesn't depend on the host's memory).
    pub const MAX_PIXELS: u64 = 1 << 30;

    /// A surface of `width` by `height` pixels with no samples. A side of 0, more pixels than
    /// [`RenderSurface::MAX_PIXELS`], or memory the host cannot provide, is
    /// [`RayTracerError::InvalidArgument`], and the size is checked before anything is reserved.
    pub fn new(width: u32, height: u32) -> Result<Self, RayTracerError> {
        // Both sides are u32, so the product fits u64.
        let pixels = u64::from(width) * u64::from(height);
        if pixels == 0 || pixels > Self::MAX_PIXELS {
            return Err(RayTracerError::InvalidArgument(
                "a render surface needs at least one pixel and at most 1073741824 (32768 by 32768)",
            ));
        }
        // At most MAX_PIXELS, so this fits usize on any host; the bytes are checked too, for a
        // 32-bit address space.
        let count = usize::try_from(pixels).map_err(|_| too_large_for_host())?;
        count
            .checked_mul(std::mem::size_of::<AccumulatedPixel>())
            .ok_or_else(too_large_for_host)?;
        let mut storage = Vec::new();
        storage
            .try_reserve_exact(count)
            .map_err(|_| too_large_for_host())?;
        storage.resize(count, AccumulatedPixel::default());
        Ok(RenderSurface {
            width,
            height,
            pixels: storage,
        })
    }

    /// The image's width in pixels.
    pub fn width(&self) -> u32 {
        self.width
    }

    /// The image's height in pixels.
    pub fn height(&self) -> u32 {
        self.height
    }

    /// The pixel at (`x`, `y`), or `None` outside the surface.
    pub fn pixel(&self, x: u32, y: u32) -> Option<&AccumulatedPixel> {
        if x >= self.width || y >= self.height {
            return None;
        }
        self.pixels
            .get(y as usize * self.width as usize + x as usize)
    }

    /// A window onto `rect` for rendering. A rectangle that is empty or leaves the surface is
    /// [`RayTracerError::InvalidArgument`].
    pub fn tile_mut(&mut self, rect: TileRect) -> Result<RenderTile<'_>, RayTracerError> {
        let right = rect.x.checked_add(rect.width);
        let top = rect.y.checked_add(rect.height);
        let fits = rect.width > 0
            && rect.height > 0
            && right.is_some_and(|edge| edge <= self.width)
            && top.is_some_and(|edge| edge <= self.height);
        if !fits {
            return Err(RayTracerError::InvalidArgument(
                "a render tile must be inside its surface and not empty",
            ));
        }
        let stride = self.width as usize;
        let (x, width) = (rect.x as usize, rect.width as usize);
        let rows = self
            .pixels
            .chunks_mut(stride)
            .skip(rect.y as usize)
            .take(rect.height as usize)
            .map(|row| &mut row[x..x + width])
            .collect();
        Ok(RenderTile {
            rect,
            image_width: self.width,
            image_height: self.height,
            rows,
        })
    }

    /// Windows covering the whole surface on a grid of tiles at most `tile_width` by
    /// `tile_height`, disjoint so each can go to its own thread. A size of 0 is
    /// [`RayTracerError::InvalidArgument`].
    pub fn tiles_mut(
        &mut self,
        tile_width: u32,
        tile_height: u32,
    ) -> Result<Vec<RenderTile<'_>>, RayTracerError> {
        if tile_width == 0 || tile_height == 0 {
            return Err(RayTracerError::InvalidArgument(
                "a render tile must be at least one pixel on each side",
            ));
        }
        let (image_width, image_height) = (self.width, self.height);
        let stride = image_width as usize;
        let tile_stride = tile_width.min(image_width);
        let columns = image_width.div_ceil(tile_stride) as usize;
        // One list of row pieces per tile: tiles are numbered band by band, so the tile a piece
        // belongs to is found by its row and column alone.
        let mut tiles: Vec<RenderTile<'_>> = Vec::new();
        for (row_index, row) in self.pixels.chunks_mut(stride).enumerate() {
            // Indices here came from u32 sides, so the casts keep them.
            let row_y = row_index as u32;
            let band = (row_y / tile_height) as usize;
            for (column, piece) in row.chunks_mut(tile_stride as usize).enumerate() {
                let index = band * columns + column;
                if index == tiles.len() {
                    let y = band as u32 * tile_height;
                    tiles.push(RenderTile {
                        rect: TileRect {
                            x: column as u32 * tile_stride,
                            y,
                            width: piece.len() as u32,
                            height: tile_height.min(image_height - y),
                        },
                        image_width,
                        image_height,
                        rows: Vec::new(),
                    });
                }
                tiles[index].rows.push(piece);
            }
        }
        Ok(tiles)
    }

    /// The whole surface as tone-mapped display bytes: RGBA8 with top row first (image formats
    /// are top-down, the surface is bottom-up), `exposure` multiplying the mean radiance. A pixel
    /// with no samples is opaque black.
    pub fn to_display_rgba(&self, exposure: f64) -> Vec<u8> {
        let stride = self.width as usize;
        let mut bytes = Vec::with_capacity(self.pixels.len() * 4);
        for row in self.pixels.chunks(stride).rev() {
            for pixel in row {
                let mean = pixel.mean();
                bytes.extend_from_slice(&[
                    display_byte(mean[0], exposure),
                    display_byte(mean[1], exposure),
                    display_byte(mean[2], exposure),
                    DISPLAY_OPAQUE,
                ]);
            }
        }
        bytes
    }
}

/// A window onto a [`RenderSurface`]'s pixels, for one thread to fill.
#[derive(Debug)]
pub struct RenderTile<'a> {
    rect: TileRect,
    image_width: u32,
    image_height: u32,
    rows: Vec<&'a mut [AccumulatedPixel]>,
}

impl RenderTile<'_> {
    /// The pixels this tile covers.
    pub fn rect(&self) -> TileRect {
        self.rect
    }

    /// The size of the whole image this tile is part of.
    pub fn image_size(&self) -> (u32, u32) {
        (self.image_width, self.image_height)
    }

    pub(super) fn rows_mut(&mut self) -> Vec<(u32, &mut [AccumulatedPixel])> {
        let y = self.rect.y;
        self.rows
            .iter_mut()
            .enumerate()
            // `row` is below the tile's u32 height, so the cast keeps it.
            .map(|(row, pixels)| (y + row as u32, &mut **pixels))
            .collect()
    }
}
