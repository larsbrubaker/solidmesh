//! The tessellation half of the 2D data converters: vertex sources into triangles through the
//! [`cached_tesselator`], on tess2-rust (the SGI tessellator). The clipper half (path to
//! polygon conversion) lives in agg-path.

pub mod cached_tesselator;

pub use cached_tesselator::{CachedTesselator, RenderIndices};
