//! The integer polygon helpers MatterCAD and PolygonMesh use, on the Clipper 6 port in
//! [`crate::clipper6`].
//!
//! ClipperLib's integer polygons map to clipper2-rust's types: `IntPoint` → `Point64`,
//! `Polygon` (`List<IntPoint>`) → `Path64`, `Polygons` → `Paths64`, `PolyFillType` →
//! `FillRule`, `ClipType`/`JoinType` → clipper2's own, `PolyTree` → `PolyTree64`. The modules:
//! point helpers ([`int_point_extensions`]), one polygon ([`polygon_extensions`]), many
//! polygons ([`polygons_extensions`]), and from a modified Clipper 6.4.2 the statics ([`clipper_statics`], which adapts those types to [`crate::clipper6`]) and
//! `CleanPolygon(s)` ([`clean_polygon`]). Every helper that runs Clipper 6 returns
//! `Result<_, `[`ClipperError`]`>`.

pub mod clean_polygon;
pub mod clipper_error;
pub mod clipper_statics;
pub mod int_point_extensions;
pub mod polygon_extensions;
pub mod polygons_extensions;

pub use clean_polygon::{clean_polygon, clean_polygons, DEFAULT_CLEAN_DISTANCE};
/// The join kinds the offset helpers take: clipper2-rust's enum,
/// which callers should name through here. Clipper 6 has no bevel join; `Bevel` offsets as
/// `Square` ([`clipper_statics::clipper6_join_type`]).
pub use clipper2_rust::JoinType;
pub use clipper_error::ClipperError;
pub use clipper_statics::{
    area, closed_paths_from_poly_tree, get_bounds, orientation, point_in_polygon, simplify_polygon,
    simplify_polygons,
};
pub use int_point_extensions::{
    calc_intersection, get_intersection, int_point, length, length_squared, on_segment,
    Intersection,
};
pub use polygon_extensions::PolygonExtensions;
pub use polygons_extensions::{
    create_from_string, PolygonDescription, PolygonsExtensions, DEFAULT_ARC_TOLERANCE,
    DEFAULT_CLIPPER_UNITS_PER_UNIT,
};

#[cfg(test)]
mod tests;
