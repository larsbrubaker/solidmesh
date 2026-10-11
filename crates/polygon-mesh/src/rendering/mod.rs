//! agg-sharp `PolygonMesh/Rendering`: what of it the model needs without a GPU. Only
//! `OrthographicZProjection` (`orthographic_z_projection`: the clipper outline and the drawn
//! silhouette) is here; the GL
//! renderers are the app's (`mattercad-renderer`).

pub mod orthographic_z_projection;
