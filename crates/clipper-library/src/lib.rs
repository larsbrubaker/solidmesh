//! clipper-library: the integer polygon clipper under MatterCAD's 2D paths and its mesh kernel.
//!
//! [`clipper6`] is a port of ClipperLib 6.4.2 (Angus Johnson, Boost Software License 1.0; see
//! THIRD_PARTY_NOTICES.md): booleans, offsets, simplification and the PolyTree. It is kept beside
//! clipper2-rust rather than replaced by it because saved designs' geometry depends on exactly
//! Clipper 6's ring starts, ring order, collinear handling and simplification, which Clipper2 does
//! not reproduce.
//!
//! The crate root holds the helpers MatterCAD and polygon-mesh call on clipper2-rust's integer
//! types, every one of which clips on [`clipper6`]. ClipperLib's integer polygons map to
//! clipper2-rust's types: `IntPoint` → `Point64`, `Polygon` (`List<IntPoint>`) → `Path64`,
//! `Polygons` → `Paths64`, `PolyFillType` → `FillRule`, `ClipType`/`JoinType` → clipper2's own,
//! `PolyTree` → `PolyTree64`. The modules: point helpers ([`int_point_extensions`]), one polygon
//! ([`polygon_extensions`]), many polygons ([`polygons_extensions`]), and from a modified Clipper
//! 6.4.2 the statics ([`clipper_statics`], which adapts those types to [`clipper6`]) and
//! `CleanPolygon(s)` ([`mod@clean_polygon`]); the winding correction every extrude and revolve
//! starts from ([`corrected_winding`]); and the size rule curve flattening and arc tolerances
//! scale by ([`curve_tolerance`]). Every helper that runs Clipper 6 returns
//! `Result<_, `[`ClipperError`]`>`.
//!
//! Its neighbours: it stands only on clipper2-rust (the integer types) and vector-math (the
//! fixed-tie sort Clipper 6's intersection list needs, and `RectangleDouble`), so the mesh kernel
//! (polygon-mesh, geometry-processing) uses it without the 2D path model. agg-path depends on it
//! and re-exports its modules under their usual paths (`agg_path::clipper6`,
//! `agg_path::clipper_library`, `agg_path::curve_tolerance`,
//! `agg_path::data_converters_2d::get_corrected_winding`), where it converts paths to and from
//! these polygons.

pub mod clean_polygon;
pub mod clipper6;
pub mod clipper_error;
pub mod clipper_statics;
pub mod corrected_winding;
pub mod curve_tolerance;
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
pub use corrected_winding::get_corrected_winding;
pub use int_point_extensions::{
    calc_intersection, get_intersection, int_point, length, length_squared, on_segment,
    Intersection,
};
pub use polygon_extensions::PolygonExtensions;
pub use polygons_extensions::{
    create_from_string, PolygonDescription, PolygonsExtensions, DEFAULT_ARC_TOLERANCE,
    DEFAULT_CLIPPER_UNITS_PER_UNIT, DEFAULT_SCALING,
};
