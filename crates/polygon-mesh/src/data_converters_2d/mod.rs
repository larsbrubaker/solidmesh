//! agg-sharp's `DataConverters2D` tessellation half: `VertexSourceToTesselator.cs` and
//! `CachedTesselator.cs`, on tess2-rust (the same SGI tessellator as agg-sharp's `Tesselate`).
//! The clipper half of the folder (`CreatePolygons` and friends) lives in agg-path.

pub mod cached_tesselator;

pub use cached_tesselator::{CachedTesselator, RenderIndices};
