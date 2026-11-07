//! The tessellation half of the 2D data converters: contours into triangles through the
//! [`cached_tesselator`], on tess2-rust (the SGI tessellator). Outlines become integer polygons
//! in `processors::outline_to_mesh`.

pub mod cached_tesselator;

pub use cached_tesselator::{CachedTesselator, RenderIndices};
