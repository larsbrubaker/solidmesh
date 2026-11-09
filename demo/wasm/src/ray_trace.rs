//! The Ray Tracing page's renderer: the library's `PathTracer` run in passes short enough for a
//! worker to stay responsive, accumulated into the library's `RenderSurface`.
//!
//! The worker runs one wasm call to its end before reading its next message and GitHub Pages does
//! not allow shared memory, so there are no threads: the page asks for one pass at a time, and a
//! pass stops when its time budget runs out. It picks up where the last one stopped, so a sample
//! fills in across several passes. A pass takes a whole [`TraceRequest`] each time, so a newer
//! camera or setting simply starts the image over and the page's request coalescing
//! (`src/mesh-runner.ts`) is enough to let it supersede queued passes. A change of material or
//! light only makes a new `RenderScene` over the trees the part already has.
//!
//! The library's `render_tile` needs consecutive sample indices per pixel starting at 0 and counts
//! rows from the bottom, so a pass walks the image top to bottom and hands the library one short
//! piece of a row at a time. Each pixel's sum depends only on its own samples, so the image is bit
//! for bit the same however the time budget slices it.
//!
//! The preview, while the camera moves, keeps every path short ([`PREVIEW_BOUNCES`]) as well as
//! being a small image; the page adapts the image size to hold its frame time. The Normals and
//! Depth views are read directly from hits and accumulated here (`sums`), because the library's
//! surface takes only what its path tracer produces. Only the pixels a pass touched are turned to
//! display bytes.
//!
//! Resizing keeps the previous image, stretched, until new samples replace it: the page's low
//! resolution preview shows through while its full-resolution first sample fills in.
//!
//! The clock is a parameter: `std::time::Instant` is not available on the web target, and a
//! native test wants to slice passes on purpose.

use solidmesh::mesh::ray_tracer::{
    display_byte, PathTraceSettings, PathTracer, RenderScene, RenderSurface, RenderWork, TileRect,
};
use solidmesh::mesh::CancelToken;

use crate::ray_trace_gpu::GpuSession;
use crate::ray_trace_scene::{Camera, TraceScene};
use crate::ray_trace_shade::{data_color, Look, Mode};

/// Pixels traced between looks at the clock.
const PIXELS_PER_CLOCK_CHECK: usize = 48;

/// The largest image a pass accepts: well past a 4K screen, so a mistaken size is refused rather
/// than exhausting the tab's memory.
const MAX_PIXELS: u64 = 12_000_000;

/// The exposure of the displayed image. The lights are set for this.
pub(crate) const EXPOSURE: f64 = 1.0;

/// The most scattering events a path may have when refining, enough for light to find its way
/// through a glass part and to the floor.
const FULL_BOUNCES: u32 = 24;

/// The bounces a path may have while the camera moves: in through glass, out, off the floor,
/// and then whatever the sky gives it.
const PREVIEW_BOUNCES: u32 = 3;

/// After how many bounces Russian roulette may end a refining path. A preview has so few
/// bounces it needs none: a path roulette ends early shows as a black dot in its single sample.
const FULL_ROULETTE_AFTER: u32 = 3;

/// What a pass is asked to draw. Two requests that compare equal keep accumulating.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TraceRequest {
    /// Orbit angle about z, radians.
    pub yaw: f64,
    /// Orbit angle above the horizon, radians.
    pub pitch: f64,
    pub zoom: f64,
    pub mode: Mode,
    pub look: Look,
    pub seed: u32,
    pub width: u32,
    pub height: u32,
    /// The camera is moving: shorter paths (the image is small too).
    pub preview: bool,
}

/// `old` (RGBA8, `old_w` x `old_h`) stretched to `w` x `h`, or opaque black when there is no old
/// image yet, for a new image to show through until its samples replace it.
pub(crate) fn stretched_display(old: &[u8], old_w: u32, old_h: u32, w: usize, h: usize) -> Vec<u8> {
    let mut stretched = vec![0u8; w * h * 4];
    let (old_w, old_h) = (old_w as usize, old_h as usize);
    if old_w > 0 && old_h > 0 {
        for y in 0..h {
            let source_y = y * old_h / h;
            for x in 0..w {
                let source = (source_y * old_w + x * old_w / w) * 4;
                stretched[(y * w + x) * 4..(y * w + x + 1) * 4]
                    .copy_from_slice(&old[source..source + 4]);
            }
        }
    } else {
        for pixel in stretched.as_chunks_mut::<4>().0 {
            pixel[3] = 255;
        }
    }
    stretched
}

/// What a pass did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PassReport {
    /// Whole samples per pixel so far.
    pub samples: u32,
    /// Rays this pass fired, by the library's work counters (path traced) or one per pixel
    /// (the data views).
    pub rays: u64,
    /// Pixel samples this pass finished.
    pub pixels: u64,
}

/// One image being refined, sample by sample.
#[derive(Default)]
pub struct Accumulation {
    request: Option<TraceRequest>,
    width: u32,
    height: u32,
    /// The path traced view's accumulated samples, `None` until one is asked for.
    surface: Option<RenderSurface>,
    /// The data views' colors summed per pixel, raster order.
    sums: Vec<[f32; 3]>,
    /// RGBA8 for the canvas, raster order.
    display: Vec<u8>,
    /// Whole samples per pixel finished.
    samples: u32,
    /// The next pixel of the sample under way, raster order.
    cursor: usize,
    /// The library's render scene for the look last rendered.
    rendering: Option<(Look, RenderScene)>,
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

    /// The path traced image so far, for callers comparing images.
    pub fn surface(&self) -> Option<&RenderSurface> {
        self.surface.as_ref()
    }

    /// The mean linear color of the path traced image at (`x`, `y`, top-left origin), or black.
    pub fn linear_at(&self, x: u32, y: u32) -> [f64; 3] {
        self.surface
            .as_ref()
            .and_then(|surface| surface.pixel(x, surface.height() - 1 - y))
            .map_or([0.0; 3], |pixel| pixel.mean())
    }

    /// Starts a new image for `request`, keeping the old one stretched until samples replace it.
    fn begin(&mut self, request: TraceRequest) -> Result<(), String> {
        let (w, h) = (request.width as usize, request.height as usize);
        if (request.width, request.height) != (self.width, self.height) {
            self.display = stretched_display(&self.display, self.width, self.height, w, h);
        }
        self.surface = if request.mode == Mode::PathTraced {
            Some(RenderSurface::new(request.width, request.height).map_err(|e| e.to_string())?)
        } else {
            None
        };
        self.sums.clear();
        if request.mode != Mode::PathTraced {
            self.sums.resize(w * h, [0.0; 3]);
        }
        self.width = request.width;
        self.height = request.height;
        self.samples = 0;
        self.cursor = 0;
        self.request = Some(request);
        Ok(())
    }

    /// Updates the display bytes of pixels `from..to`, raster order, from what has accumulated.
    fn refresh(&mut self, from: usize, to: usize) {
        let w = self.width as usize;
        match &self.surface {
            Some(surface) => {
                let height = self.height;
                for index in from..to {
                    // Below the image's u32 size, so these casts keep the coordinates.
                    let x = (index % w) as u32;
                    let y = height - 1 - (index / w) as u32;
                    let mean = surface.pixel(x, y).map_or([0.0; 3], |p| p.mean());
                    let at = index * 4;
                    for (channel, value) in mean.iter().enumerate() {
                        self.display[at + channel] = display_byte(*value, EXPOSURE);
                    }
                    self.display[at + 3] = 255;
                }
            }
            None => {
                let count = f64::from(self.samples + 1);
                for (pixel, sum) in self.display[from * 4..to * 4]
                    .as_chunks_mut::<4>()
                    .0
                    .iter_mut()
                    .zip(&self.sums[from..to])
                {
                    for channel in 0..3 {
                        let value = (f64::from(sum[channel]) / count).clamp(0.0, 1.0);
                        pixel[channel] = (value * 255.0 + 0.5) as u8;
                    }
                    pixel[3] = 255;
                }
            }
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
            self.begin(request)?;
        }
        let camera = Camera::new(
            scene,
            request.yaw,
            request.pitch,
            request.zoom,
            request.width,
            request.height,
        )?;
        let rendering = match self.rendering.take() {
            Some((look, built)) if look == request.look => (look, built),
            _ => (request.look, scene.render_scene(&request.look)?),
        };
        let settings = settings_of(&request);
        let tracer = PathTracer::new(
            &rendering.1,
            camera.render_camera(),
            request.width,
            request.height,
            settings,
        )
        .map_err(|e| e.to_string())?;

        let start = clock();
        let finish_sample = budget_ms <= 0.0;
        let (w, total) = (request.width as usize, pixels as usize);
        let cancel = CancelToken::new();
        let mut work = RenderWork::default();
        let mut data_rays = 0_u64;
        let mut dirty_from = self.cursor;
        loop {
            // The cursor is below the pixel count, so its row and column fit the image's u32
            // sides and `as` keeps them.
            let column = self.cursor % w;
            let run = PIXELS_PER_CLOCK_CHECK
                .min(w - column)
                .min(total - self.cursor);
            let row = (self.cursor / w) as u32;
            match request.mode {
                Mode::PathTraced => {
                    let surface = self.surface.as_mut().ok_or("the surface was not started")?;
                    let rect = TileRect {
                        x: column as u32,
                        y: request.height - 1 - row,
                        width: run as u32,
                        height: 1,
                    };
                    let mut tile = surface.tile_mut(rect).map_err(|e| e.to_string())?;
                    let done = tracer
                        .render_tile(&mut tile, self.samples, &cancel)
                        .map_err(|e| e.to_string())?;
                    work.add(&done);
                }
                Mode::Normals | Mode::Depth => {
                    for offset in 0..run {
                        let x = (column + offset) as u32;
                        let color = data_color(
                            scene,
                            &camera,
                            request.mode,
                            request.seed,
                            self.samples,
                            x,
                            row,
                        )?;
                        data_rays += 1;
                        let sum = &mut self.sums[self.cursor + offset];
                        for channel in 0..3 {
                            sum[channel] += color[channel] as f32;
                        }
                    }
                }
            }
            self.cursor += run;
            if self.cursor >= total {
                self.refresh(dirty_from, total);
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
            self.refresh(dirty_from, self.cursor);
        }
        self.rendering = Some(rendering);
        Ok(PassReport {
            samples: self.samples,
            rays: work.total_rays() + data_rays,
            pixels: work.pixels_rendered + data_rays,
        })
    }
}

/// The path tracer's settings for this request. Refining follows paths far; a preview cuts them
/// short, so a moving image costs a few rays per pixel.
pub(crate) fn settings_of(request: &TraceRequest) -> PathTraceSettings {
    let (max_bounces, after) = if request.preview {
        (PREVIEW_BOUNCES, None)
    } else {
        (FULL_BOUNCES, Some(FULL_ROULETTE_AFTER))
    };
    PathTraceSettings {
        seed: u64::from(request.seed),
        max_bounces,
        russian_roulette_after: after,
        ..PathTraceSettings::default()
    }
}

/// A prepared part and the image being refined from it, as the worker keeps them between calls.
/// `gpu` is the GPU route's pack of the same part (`ray_trace_gpu`), `None` until the page asks.
pub struct Session {
    pub scene: TraceScene,
    pub accumulation: Accumulation,
    pub gpu: Option<GpuSession>,
}

impl Session {
    pub fn new(scene: TraceScene) -> Session {
        Session {
            scene,
            accumulation: Accumulation::new(),
            gpu: None,
        }
    }
}
