//! The physically based path tracer: unidirectional path tracing over [`PlacedParts`], shaded by
//! [`ScatteringSurface`], rendered in passes the caller drives. Library only: no threads, no
//! clocks, and no randomness that isn't a pure function of a seed, so it runs on
//! `wasm32-unknown-unknown` and gives the same image bit for bit however the work is sliced.
//!
//! [`PlacedParts`]: super::PlacedParts
//! [`ScatteringSurface`]: super::ScatteringSurface
//!
//! # The pieces
//!
//! - `lights`: [`Environment`] (a constant radiance or a smooth sky-to-ground gradient), [`Sun`]
//!   (a disc with a finite angular radius and a radiance) and [`LightSetup`], which holds one of
//!   each (the sun is optional).
//! - `render_scene`: [`RenderScene`], the parts, one surface per part, and the lights.
//! - `camera`: [`RenderCamera`], perspective or orthographic, from eye, target and up.
//! - `pixel_random`: [`PixelRandom`], each pixel's random stream, and the jitter inside the pixel.
//! - `integrator`: the path loop; [`PathTraceSettings`] and [`LightStrategy`] are its knobs.
//! - `renderer`: [`PathTracer`], the pass a caller runs over a tile.
//! - `render_surface`: [`RenderSurface`] (accumulated linear HDR color and sample counts per
//!   pixel) and [`RenderTile`], the disjoint windows onto it that threads fill.
//! - `display`: exposure, the tone map and sRGB encoding.
//! - `render_work`: [`RenderWork`], how many rays of each kind a pass cost.
//!
//! # What the integrator computes, and what it leaves out
//!
//! Each path starts at the camera. At every surface it hits, it samples the sun directly
//! (next-event estimation, through a shadow ray) and samples the surface for its next direction.
//! The two ways of reaching the sun are combined by weighting each by how likely it was to have
//! produced that direction (the power heuristic on the two densities), so neither counts the
//! light twice. A delta lobe (a perfect mirror or smooth glass) has no density, so only surface
//! sampling reaches a light through it, at full weight.
//!
//! The environment is reached only by surface sampling, and at full weight. A gradient sky is
//! smooth, so light sampling it would buy little and cost a shadow ray at every bounce.
//!
//! **Glass casts a shadow to next-event estimation.** A shadow ray stops at the first thing it
//! meets, glass included, so sunlight focused through glass (a caustic) comes only from paths that
//! happened to reach the sun by surface sampling. Those are unbiased but noisy, and with a very
//! small sun they are rare. A point behind glass in sun-only light therefore converges slowly to
//! its correct brightness.
//!
//! **Rough glass loses energy.** The scattering model's rough glass is single-scatter only (its
//! header says so), and the integrator returns exactly what the model gives it.
//!
//! **Paths are capped** at [`PathTraceSettings::max_bounces`] scattering events and thinned by
//! Russian roulette after [`PathTraceSettings::russian_roulette_after`] of them; the roulette is
//! unbiased, the cap loses whatever energy a path still carries there.
//!
//! # Conventions
//!
//! Radiance is linear RGB in `f64`. Pixel coordinates are bottom-left origin with y up, as
//! `WorldView`'s screen positions are; only [`RenderSurface::to_display_rgba`] writes rows top
//! down, as image formats expect. World up is +Z for the sky gradient, as MatterCAD's bed is XY.

mod camera;
mod display;
mod integrator;
mod lights;
mod pixel_random;
mod render_scene;
mod render_surface;
mod render_work;
mod renderer;

pub use camera::RenderCamera;
pub use display::{display_byte, linear_to_srgb, tone_map};
pub use integrator::{LightStrategy, PathTraceSettings};
pub use lights::{Environment, LightSetup, Sun};
pub use pixel_random::{pixel_jitter, PixelRandom};
pub use render_scene::RenderScene;
pub use render_surface::{AccumulatedPixel, RenderSurface, RenderTile, TileRect};
pub use render_work::{RenderWork, BOUNCE_HISTOGRAM_BINS};
pub use renderer::PathTracer;
