//! agg-sharp's `clipper_library` as MatterCAD and PolygonMesh use it, on clipper2-rust.
//!
//! C#'s ClipperLib integer polygons map to clipper2-rust's types: `IntPoint` → `Point64`,
//! `Polygon` (`List<IntPoint>`) → `Path64`, `Polygons` → `Paths64`, `PolyFillType` →
//! `FillRule`, `ClipType`/`JoinType` → clipper2's own, `PolyTree` → `PolyTree64`. The modules
//! follow the C# files: `IntPointExtensions.cs` ([`int_point_extensions`]),
//! `PolygonExtensions.cs` ([`polygon_extensions`]), `PolygonsExtensions.cs`
//! ([`polygons_extensions`]), and from agg-sharp's modified Clipper 6.4.2 `clipper.cs` the
//! statics clipper2 lacks or computes differently ([`clipper_statics`], [`clean_polygon`]).
//! The polygon booleans and offsets themselves are Clipper2's (docs/CSHARP_DIVERGENCES.md
//! 220-221).

pub mod clean_polygon;
pub mod clipper_statics;
pub mod int_point_extensions;
pub mod polygon_extensions;
pub mod polygons_extensions;

pub use clean_polygon::{clean_polygon, clean_polygons, DEFAULT_CLEAN_DISTANCE};
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
