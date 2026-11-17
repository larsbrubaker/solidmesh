//! Halving and shrinking a [`TraceTexture`] in pieces: [`TextureHalving`] and
//! [`TextureShrinking`], for a host with no threads (a browser's UI thread) that must keep every
//! piece of work inside a frame. Halving a 12-megapixel picture in one go takes about 200 ms
//! natively (an optimized build) and longer in a browser; stepped a bounded number of texels at a
//! time (10,000 halved texels take about 0.7 ms natively), each
//! step stays as short as the host asks. The halving itself (which source texels average into
//! which, and how) is `trace_texture`'s; this file only cuts it into pieces, and
//! [`TraceTexture::halved`], [`TraceTexture::try_halved`] and [`TraceTexture::try_shrunk`] run
//! through the same code, so the bytes are the same however the work is cut.
//!
//! **A step** writes up to `max_texels` texels of the picture being made, in row order (bottom
//! row first, left to right), and always at least one, so a host loop that keeps stepping always
//! ends. The halved picture's bytes are allocated on the first step, fallibly, so a device
//! without the memory gets a [`TryReserveError`] where [`TraceTexture::halved`] would abort. A
//! step answers the finished picture once; after that it answers `Ok(None)` and does nothing
//! ([`TextureHalving::is_finished`] says so).
//!
//! **Whose picture.** A stepper owns what borrows as a [`TraceTexture`]: a plain reference for a
//! caller that finishes within one borrow, or an `Arc<TraceTexture>` so the stepper can live across
//! frames beside the scene's own shared pictures. The intermediate pictures of a shrink are its
//! own, each dropped once the next halving of it is done.
//!
//! The atlas build (`flat_tree/gpu_atlas_build.rs`) drives the same cursors with a budget it
//! shares between halving and copying, a halved texel costing more of it than a copied one.

use std::borrow::Borrow;
use std::collections::TryReserveError;

use super::render_scene::decoded_table;
use super::trace_texture::{box_average, TEXEL_BYTES};
use super::TraceTexture;

/// Where one halving is: the halved picture's size, the next texel to write and the bytes so far
/// (`None` until the first step reserves them).
#[derive(Debug, Clone)]
pub(crate) struct HalvingCursor {
    width: u32,
    height: u32,
    row: u32,
    column: u32,
    bytes: Option<Vec<u8>>,
    /// The sRGB table the halving decodes through, boxed so a cursor stays small to move.
    decode: Box<[f64; 256]>,
}

impl HalvingCursor {
    /// A halving of `source` not yet started.
    pub(crate) fn new(source: &TraceTexture) -> Self {
        HalvingCursor {
            width: source.width().div_ceil(2),
            height: source.height().div_ceil(2),
            row: 0,
            column: 0,
            bytes: None,
            decode: Box::new(decoded_table()),
        }
    }

    /// How many texels of the halved picture are written.
    fn texels_done(&self) -> u64 {
        u64::from(self.row) * u64::from(self.width) + u64::from(self.column)
    }

    /// The halved picture's byte count.
    fn byte_len(&self) -> usize {
        // The halved picture has fewer texels than its source, whose byte count fits a `usize`.
        self.width as usize * self.height as usize * TEXEL_BYTES
    }

    /// Writes up to `*budget / cost` texels of the halving of `source` (the picture this cursor
    /// was made for), at least one while any budget is left, taking `cost` from `budget` for each
    /// texel written (all that is left when one texel costs more). Answers the halved picture once
    /// its last texel is written.
    pub(crate) fn step(
        &mut self,
        source: &TraceTexture,
        budget: &mut usize,
        cost: usize,
    ) -> Result<Option<TraceTexture>, TryReserveError> {
        if *budget == 0 {
            return Ok(None);
        }
        let byte_len = self.byte_len();
        let bytes = match &mut self.bytes {
            Some(bytes) => bytes,
            empty => {
                let mut bytes = Vec::new();
                bytes.try_reserve_exact(byte_len)?;
                empty.insert(bytes)
            }
        };
        let count = (*budget / cost.max(1)).max(1);
        let written = halve_texels(
            source,
            (self.width, self.height),
            (&mut self.row, &mut self.column),
            bytes,
            count,
            &self.decode,
        );
        *budget = budget.saturating_sub(written.saturating_mul(cost));
        if self.row < self.height {
            return Ok(None);
        }
        let bytes = self.bytes.take().unwrap_or_default();
        Ok(Some(TraceTexture::from_parts(
            self.width,
            self.height,
            bytes,
        )))
    }
}

/// Appends up to `count` texels of the `size` halving of `source` to `bytes`, from texel
/// `(column, row)` on, moving the cursor past them; answers how many it wrote. The halving rule
/// is `trace_texture`'s ([`box_average`] over each 2 by 2 block, an index past the last clamped to
/// it).
pub(crate) fn halve_texels(
    source: &TraceTexture,
    size: (u32, u32),
    cursor: (&mut u32, &mut u32),
    bytes: &mut Vec<u8>,
    count: usize,
    decode: &[f64; 256],
) -> usize {
    let (width, height) = size;
    let (row, column) = cursor;
    let mut written = 0;
    while *row < height && written < count {
        // `2 * row + 1` is at most the source height: `row` is below `height.div_ceil(2)`.
        let rows = [2 * *row, (2 * *row + 1).min(source.height() - 1)];
        while *column < width && written < count {
            let columns = [2 * *column, (2 * *column + 1).min(source.width() - 1)];
            let texel = |c: usize, r: usize| source.texel(columns[c], rows[r]).unwrap_or([0; 4]);
            let block = [texel(0, 0), texel(1, 0), texel(0, 1), texel(1, 1)];
            bytes.extend_from_slice(&box_average(block, decode));
            *column += 1;
            written += 1;
        }
        if *column == width {
            *column = 0;
            *row += 1;
        }
    }
    written
}

/// What a shrink is doing: copying (no halvings asked), or halving the source or the last
/// halving's picture.
#[derive(Debug, Clone)]
enum ShrinkWork {
    Copy {
        /// The bytes copied so far, `None` until the first step reserves them.
        bytes: Option<Vec<u8>>,
    },
    Halve {
        /// Halvings finished.
        done: u32,
        /// The last finished halving's picture, which the next halves; `None` while the first
        /// halving reads the source.
        current: Option<TraceTexture>,
        cursor: HalvingCursor,
    },
    Finished,
}

/// Where a shrink by some number of halvings is ([`TraceTexture::shrunk`] in pieces).
#[derive(Debug, Clone)]
pub(crate) struct ShrinkCursor {
    halvings: u32,
    work: ShrinkWork,
    /// Texels written so far, copied or halved, across every halving.
    written: u64,
}

impl ShrinkCursor {
    /// A shrink of `source` by `halvings` not yet started.
    pub(crate) fn new(source: &TraceTexture, halvings: u32) -> Self {
        let work = if halvings == 0 {
            ShrinkWork::Copy { bytes: None }
        } else {
            ShrinkWork::Halve {
                done: 0,
                current: None,
                cursor: HalvingCursor::new(source),
            }
        };
        ShrinkCursor {
            halvings,
            work,
            written: 0,
        }
    }

    /// How many texels the steps have written, copied or halved, across every halving.
    pub(crate) fn texels_written(&self) -> u64 {
        self.written
    }

    /// True once the shrunk picture was answered.
    pub(crate) fn is_finished(&self) -> bool {
        matches!(self.work, ShrinkWork::Finished)
    }

    /// Up to the budget's texels of the shrink of `source` (the picture this cursor was made
    /// for): a copied texel costs 1 and a halved one `halved_cost`. Each halving runs to its end
    /// before the next starts, as [`TraceTexture::shrunk`] runs them: the first always, each later
    /// one only while the picture is larger than 1 by 1. Answers the shrunk picture once.
    pub(crate) fn step(
        &mut self,
        source: &TraceTexture,
        budget: &mut usize,
        halved_cost: usize,
    ) -> Result<Option<TraceTexture>, TryReserveError> {
        loop {
            if *budget == 0 {
                return Ok(None);
            }
            match &mut self.work {
                ShrinkWork::Copy { bytes } => {
                    let all = source.bytes();
                    let bytes = match bytes {
                        Some(bytes) => bytes,
                        empty => {
                            let mut bytes = Vec::new();
                            bytes.try_reserve_exact(all.len())?;
                            empty.insert(bytes)
                        }
                    };
                    let texels = (all.len() - bytes.len()) / TEXEL_BYTES;
                    let count = (*budget).min(texels);
                    let from = bytes.len();
                    bytes.extend_from_slice(&all[from..from + count * TEXEL_BYTES]);
                    *budget -= count;
                    self.written += count as u64;
                    if bytes.len() < all.len() {
                        return Ok(None);
                    }
                    let bytes = std::mem::take(bytes);
                    self.work = ShrinkWork::Finished;
                    return Ok(Some(TraceTexture::from_parts(
                        source.width(),
                        source.height(),
                        bytes,
                    )));
                }
                ShrinkWork::Halve {
                    done,
                    current,
                    cursor,
                } => {
                    let from = current.as_ref().unwrap_or(source);
                    let before = cursor.texels_done();
                    let halved = cursor.step(from, budget, halved_cost);
                    self.written += cursor.texels_done() - before;
                    let Some(halved) = halved? else {
                        return Ok(None);
                    };
                    *done += 1;
                    if *done == self.halvings || (halved.width() == 1 && halved.height() == 1) {
                        self.work = ShrinkWork::Finished;
                        return Ok(Some(halved));
                    }
                    *cursor = HalvingCursor::new(&halved);
                    *current = Some(halved);
                }
                ShrinkWork::Finished => return Ok(None),
            }
        }
    }
}

/// [`TraceTexture::try_halved`] in pieces; see the module header.
#[derive(Debug, Clone)]
pub struct TextureHalving<P: Borrow<TraceTexture>> {
    source: P,
    cursor: HalvingCursor,
    finished: bool,
}

impl<P: Borrow<TraceTexture>> TextureHalving<P> {
    /// A halving of `source`, nothing done yet.
    pub fn new(source: P) -> Self {
        let cursor = HalvingCursor::new(source.borrow());
        TextureHalving {
            source,
            cursor,
            finished: false,
        }
    }

    /// Writes up to `max_texels` texels of the halved picture (at least one), and answers it
    /// once the last is written: the same bytes as [`TraceTexture::try_halved`] however the work
    /// is cut. Refused when the halved picture's bytes cannot be allocated (the first step
    /// reserves them).
    pub fn step(&mut self, max_texels: usize) -> Result<Option<TraceTexture>, TryReserveError> {
        if self.finished {
            return Ok(None);
        }
        let mut budget = max_texels.max(1);
        let halved = self.cursor.step(self.source.borrow(), &mut budget, 1)?;
        self.finished = halved.is_some();
        Ok(halved)
    }

    /// True once the halved picture was answered.
    pub fn is_finished(&self) -> bool {
        self.finished
    }

    /// How many texels the halved picture has, all of which the steps write.
    pub fn texel_count(&self) -> u64 {
        u64::from(self.cursor.width) * u64::from(self.cursor.height)
    }
}

/// [`TraceTexture::try_shrunk`] in pieces; see the module header.
#[derive(Debug, Clone)]
pub struct TextureShrinking<P: Borrow<TraceTexture>> {
    source: P,
    cursor: ShrinkCursor,
}

impl<P: Borrow<TraceTexture>> TextureShrinking<P> {
    /// `source` to be halved `halvings` times (0 is a copy), nothing done yet.
    pub fn new(source: P, halvings: u32) -> Self {
        let cursor = ShrinkCursor::new(source.borrow(), halvings);
        TextureShrinking { source, cursor }
    }

    /// Writes up to `max_texels` texels (at least one) of the halvings in turn, or of the copy,
    /// and answers the shrunk picture once it is done: the same bytes as
    /// [`TraceTexture::try_shrunk`] however the work is cut. Refused when a halving's or the
    /// copy's bytes cannot be allocated.
    pub fn step(&mut self, max_texels: usize) -> Result<Option<TraceTexture>, TryReserveError> {
        let mut budget = max_texels.max(1);
        self.cursor.step(self.source.borrow(), &mut budget, 1)
    }

    /// True once the shrunk picture was answered.
    pub fn is_finished(&self) -> bool {
        self.cursor.is_finished()
    }

    /// How many texels the steps have written so far, copied or halved, across every halving:
    /// a step adds at most the texels it was given.
    pub fn texels_written(&self) -> u64 {
        self.cursor.texels_written()
    }
}
