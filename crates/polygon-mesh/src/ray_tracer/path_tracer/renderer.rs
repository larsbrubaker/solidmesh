//! [`PathTracer`], the pass a caller runs: scene, camera and settings together, sampled at any
//! pixel from any thread. [`PathTracer::render_tile`] accumulates one sample per pixel of a tile
//! and takes a [`RenderTile`], so tiles can be handed to as many threads as the caller has; a
//! tile's result depends only on each pixel's coordinates and sample index, so the order and shape
//! of the tiles cannot reach the image. The library starts no threads and reads no clock.

use super::camera::RenderCamera;
use super::integrator::{trace_path, PathTraceSettings};
use super::pixel_random::{pixel_jitter, PixelRandom};
use super::render_scene::RenderScene;
use super::render_surface::{RenderSurface, RenderTile};
use super::render_work::RenderWork;
use crate::ray_tracer::RayTracerError;
use crate::CancelToken;

/// A scene, camera and settings ready to render. Immutable, so it is shared between threads.
pub struct PathTracer<'a> {
    scene: &'a RenderScene,
    camera: RenderCamera,
    width: u32,
    height: u32,
    settings: PathTraceSettings,
}

impl<'a> PathTracer<'a> {
    /// A renderer of `scene` through `camera`, for an image of `width` by `height` pixels. A side
    /// of 0 is [`RayTracerError::InvalidArgument`].
    pub fn new(
        scene: &'a RenderScene,
        camera: RenderCamera,
        width: u32,
        height: u32,
        settings: PathTraceSettings,
    ) -> Result<Self, RayTracerError> {
        if width == 0 || height == 0 {
            return Err(RayTracerError::InvalidArgument(
                "a render needs an image of at least one pixel",
            ));
        }
        Ok(PathTracer {
            scene,
            camera,
            width,
            height,
            settings,
        })
    }

    /// The settings this renderer runs with.
    pub fn settings(&self) -> &PathTraceSettings {
        &self.settings
    }

    /// Sample `sample_index` of pixel (`x`, `y`): the radiance one path carries back, counted in
    /// `work`. A pure function of the scene, settings, pixel and index. The radiance can be non
    /// finite if a value overflowed; [`AccumulatedPixel::add_sample`] drops those.
    ///
    /// [`AccumulatedPixel::add_sample`]: super::AccumulatedPixel::add_sample
    pub fn sample_pixel(
        &self,
        x: u32,
        y: u32,
        sample_index: u32,
        work: &mut RenderWork,
    ) -> Result<[f64; 3], RayTracerError> {
        let (jitter_x, jitter_y) = pixel_jitter(self.settings.seed, x, y, sample_index);
        let ray = self.camera.ray_through(
            f64::from(x) + jitter_x,
            f64::from(y) + jitter_y,
            self.width,
            self.height,
        );
        let mut random = PixelRandom::for_pixel(self.settings.seed, x, y, sample_index);
        trace_path(self.scene, &self.settings, ray, &mut random, work)
    }

    /// Accumulates sample `sample_index` into every pixel of `tile` that has had every earlier
    /// sample and not this one. `cancel` is read before each pixel, so a cancel stops within a
    /// pixel's worth of work and the pixels already done are valid; [`RenderWork::cancelled`]
    /// says it stopped. A tile from a surface of a different size than this renderer is
    /// [`RayTracerError::InvalidArgument`]. An error from a path leaves that tile's earlier
    /// pixels accumulated, so the surface stays valid.
    pub fn render_tile(
        &self,
        tile: &mut RenderTile<'_>,
        sample_index: u32,
        cancel: &CancelToken,
    ) -> Result<RenderWork, RayTracerError> {
        if tile.image_size() != (self.width, self.height) {
            return Err(RayTracerError::InvalidArgument(
                "the tile belongs to an image of a different size than this render",
            ));
        }
        let mut work = RenderWork::default();
        let tile_x = tile.rect().x;
        for (y, row) in tile.rows_mut() {
            for (offset, pixel) in row.iter_mut().enumerate() {
                if cancel.is_cancelled() {
                    work.cancelled = true;
                    return Ok(work);
                }
                if pixel.samples() != sample_index {
                    continue;
                }
                // `offset` is below the tile's u32 width, so the cast keeps it.
                let x = tile_x + offset as u32;
                let radiance = self.sample_pixel(x, y, sample_index, &mut work)?;
                if pixel.add_sample(sample_index, radiance) == Some(false) {
                    work.dropped_samples += 1;
                }
                work.pixels_rendered += 1;
            }
        }
        Ok(work)
    }

    /// One whole pass over `surface` on this thread, in one tile: what a caller with no threads
    /// wants. See [`PathTracer::render_tile`].
    pub fn render_pass(
        &self,
        surface: &mut RenderSurface,
        sample_index: u32,
        cancel: &CancelToken,
    ) -> Result<RenderWork, RayTracerError> {
        let mut work = RenderWork::default();
        for mut tile in surface.tiles_mut(self.width, self.height)? {
            work.add(&self.render_tile(&mut tile, sample_index, cancel)?);
        }
        Ok(work)
    }
}
