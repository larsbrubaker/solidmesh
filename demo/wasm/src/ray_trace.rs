//! The Ray Tracing page's renderer: per-pixel accumulation over a [`TraceScene`], in passes short
//! enough for a worker to stay responsive.
//!
//! The worker runs one wasm call to its end before reading its next message and GitHub Pages does
//! not allow shared memory, so there are no threads: the page asks for one pass at a time, and a
//! pass stops when its time budget runs out. It picks up where the last one stopped, so a sample
//! fills in across several passes. A pass takes a whole [`TraceRequest`] each time, so a newer
//! camera or setting simply starts the image over and the page's request coalescing
//! (`src/mesh-runner.ts`) is enough to let it supersede queued passes.
//!
//! Sums are kept linear and divided per pixel: a pixel at index `i` has `samples + 1` samples when
//! `i` is below the sample's cursor and `samples` otherwise, so the image stays whole while a
//! sample is under way. Only the pixels a pass touched are turned to display bytes. Because every
//! random choice is seeded per (seed, sample, pixel) (`ray_trace_shade::sample_rng`), the image
//! is bit for bit the same however the time budget slices it.
//!
//! Resizing keeps the previous image, stretched, until new samples replace it: the page's low
//! resolution preview shows through while its full-resolution first sample fills in.
//!
//! The clock is a parameter: `std::time::Instant` is not available on the web target, and a
//! native test wants to slice passes on purpose.

use crate::ray_trace_scene::{Camera, TraceScene};
use crate::ray_trace_shade::{pixel_color, Mode};

/// Pixels traced between looks at the clock.
const PIXELS_PER_CLOCK_CHECK: usize = 48;

/// The largest image a pass accepts: well past a 4K screen, so a mistaken size is refused rather
/// than exhausting the tab's memory.
const MAX_PIXELS: u64 = 12_000_000;

/// What a pass is asked to draw. Two requests that compare equal keep accumulating.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TraceRequest {
    /// Orbit angle about z, radians.
    pub yaw: f64,
    /// Orbit angle above the horizon, radians.
    pub pitch: f64,
    pub zoom: f64,
    pub mode: Mode,
    /// 0 matte to 1 mirror.
    pub reflectivity: f64,
    pub seed: u32,
    pub width: u32,
    pub height: u32,
}

/// What a pass did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PassReport {
    /// Whole samples per pixel so far.
    pub samples: u32,
    /// Rays this pass fired.
    pub rays: u64,
}

/// One image being refined, sample by sample.
#[derive(Default)]
pub struct Accumulation {
    request: Option<TraceRequest>,
    width: u32,
    height: u32,
    /// Linear color summed per pixel, raster order.
    sums: Vec<[f32; 3]>,
    /// RGBA8 for the canvas, raster order.
    display: Vec<u8>,
    /// Whole samples per pixel finished.
    samples: u32,
    /// The next pixel of the sample under way.
    cursor: usize,
}

/// The sRGB encoding of a linear value in 0..=1.
fn srgb(linear: f64) -> f64 {
    if linear <= 0.003_130_8 {
        linear * 12.92
    } else {
        1.055 * linear.powf(1.0 / 2.4) - 0.055
    }
}

/// A 0..=1 value as a display byte. The clamp keeps it in range; `as` would saturate anyway.
fn byte(value: f64) -> u8 {
    (value.clamp(0.0, 1.0) * 255.0 + 0.5) as u8
}

impl Accumulation {
    pub fn new() -> Accumulation {
        Accumulation::default()
    }

    /// The image so far as RGBA8, raster order.
    pub fn pixels(&self) -> &[u8] {
        &self.display
    }

    pub fn width(&self) -> u32 {
        self.width
    }

    pub fn height(&self) -> u32 {
        self.height
    }

    pub fn samples(&self) -> u32 {
        self.samples
    }

    /// The linear sums, raster order, for callers comparing images.
    pub fn sums(&self) -> &[[f32; 3]] {
        &self.sums
    }

    /// Starts a new image for `request`, keeping the old one stretched until samples replace it.
    fn begin(&mut self, request: TraceRequest) {
        let (w, h) = (request.width as usize, request.height as usize);
        if (request.width, request.height) != (self.width, self.height) {
            let mut stretched = vec![0u8; w * h * 4];
            let (old_w, old_h) = (self.width as usize, self.height as usize);
            if old_w > 0 && old_h > 0 {
                for y in 0..h {
                    let source_y = y * old_h / h;
                    for x in 0..w {
                        let source = (source_y * old_w + x * old_w / w) * 4;
                        stretched[(y * w + x) * 4..(y * w + x + 1) * 4]
                            .copy_from_slice(&self.display[source..source + 4]);
                    }
                }
            } else {
                for pixel in stretched.as_chunks_mut::<4>().0 {
                    pixel[3] = 255;
                }
            }
            self.display = stretched;
        }
        self.sums.clear();
        self.sums.resize(w * h, [0.0; 3]);
        self.width = request.width;
        self.height = request.height;
        self.samples = 0;
        self.cursor = 0;
        self.request = Some(request);
    }

    /// Updates the display bytes of pixels `from..to`, which have this many samples.
    fn refresh(&mut self, mode: Mode, from: usize, to: usize) {
        let count = f64::from(self.samples + 1);
        for (pixel, sum) in self.display[from * 4..to * 4]
            .as_chunks_mut::<4>()
            .0
            .iter_mut()
            .zip(&self.sums[from..to])
        {
            for channel in 0..3 {
                let value = f64::from(sum[channel]) / count;
                pixel[channel] = byte(if mode == Mode::Shaded {
                    srgb(value)
                } else {
                    value
                });
            }
            pixel[3] = 255;
        }
    }

    /// Traces until `budget_ms` of `clock` has passed, or until the sample under way is done when
    /// `budget_ms` is 0 or less (the page's preview, which must show a whole frame).
    pub fn pass(
        &mut self,
        scene: &TraceScene,
        request: TraceRequest,
        budget_ms: f64,
        clock: &mut dyn FnMut() -> f64,
    ) -> Result<PassReport, String> {
        let pixels = u64::from(request.width) * u64::from(request.height);
        if pixels == 0 || pixels > MAX_PIXELS {
            return Err(format!(
                "The image must be between 1 and {MAX_PIXELS} pixels; this one is {pixels}."
            ));
        }
        if self.request != Some(request) {
            self.begin(request);
        }
        let camera = Camera::new(
            scene,
            request.yaw,
            request.pitch,
            request.zoom,
            request.width,
            request.height,
        );
        let start = clock();
        let finish_sample = budget_ms <= 0.0;
        let (w, total) = (request.width as usize, self.sums.len());
        let mut rays = 0u64;
        let mut dirty_from = self.cursor;
        loop {
            for _ in 0..PIXELS_PER_CLOCK_CHECK.min(total - self.cursor) {
                let index = self.cursor;
                let x = (index % w) as u32;
                let y = (index / w) as u32;
                let color = pixel_color(
                    scene,
                    &camera,
                    request.mode,
                    request.reflectivity,
                    request.seed,
                    self.samples,
                    x,
                    y,
                    &mut rays,
                );
                let sum = &mut self.sums[index];
                for channel in 0..3 {
                    sum[channel] += color[channel] as f32;
                }
                self.cursor += 1;
            }
            if self.cursor >= total {
                self.refresh(request.mode, dirty_from, total);
                self.samples += 1;
                self.cursor = 0;
                dirty_from = 0;
                if finish_sample {
                    break;
                }
            }
            if !finish_sample && clock() - start >= budget_ms {
                break;
            }
        }
        if self.cursor > dirty_from {
            self.refresh(request.mode, dirty_from, self.cursor);
        }
        Ok(PassReport {
            samples: self.samples,
            rays,
        })
    }
}

/// A prepared part and the image being refined from it, as the worker keeps them between calls.
pub struct Session {
    pub scene: TraceScene,
    pub accumulation: Accumulation,
}

impl Session {
    pub fn new(scene: TraceScene) -> Session {
        Session {
            scene,
            accumulation: Accumulation::new(),
        }
    }
}
