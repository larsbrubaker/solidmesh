//! **solidmesh-clipper**: integer polygon booleans, offsets and simplification in pure Rust, the
//! 2D clipper under the [solidmesh](https://docs.rs/solidmesh) mesh kernel. It builds for
//! `wasm32-unknown-unknown` and gives the same polygons on every platform.
//!
//! Polygons are clipper2-rust's integer types, re-exported here: a [`Point64`], a polygon
//! [`Path64`], a set of polygons [`Paths64`], with a [`FillRule`] saying which regions are inside.
//! The helpers are traits on them ([`PolygonExtensions`] for one polygon, [`PolygonsExtensions`]
//! for many) and free functions ([`area`], [`orientation`], [`simplify_polygons`],
//! [`clean_polygons`], [`get_corrected_winding`]). Every helper that clips returns
//! `Result<_, `[`ClipperError`]`>`. Work in whole units of your choosing (solidmesh's mesh crate
//! scales millimetres by [`DEFAULT_SCALING`]).
//!
//! ```
//! use solidmesh_clipper::{area, int_point, FillRule, JoinType, Path64, PolygonsExtensions};
//!
//! // Two overlapping 10 by 10 squares, unioned, then grown by 1 unit.
//! let square = |x: f64| -> Path64 {
//!     vec![
//!         int_point(x, 0.0),
//!         int_point(x + 10.0, 0.0),
//!         int_point(x + 10.0, 10.0),
//!         int_point(x, 10.0),
//!     ]
//! };
//! let both = vec![square(0.0)].union(&vec![square(5.0)], FillRule::NonZero)?;
//! assert_eq!(both.len(), 1);
//! assert_eq!(area(&both[0]), 150.0);
//!
//! let grown = both.offset(1.0, JoinType::Miter)?;
//! assert_eq!(area(&grown[0]), 17.0 * 12.0);
//! # Ok::<(), solidmesh_clipper::ClipperError>(())
//! ```
//!
//! solidmesh-clipper is one of four crates: [solidmesh-math](https://docs.rs/solidmesh-math)
//! (vectors, matrices, bounds), this one, [solidmesh-mesh](https://docs.rs/solidmesh-mesh)
//! (triangle meshes, booleans, repair) and [solidmesh](https://docs.rs/solidmesh) (fillets and
//! chamfers), which re-exports the other three; depend on `solidmesh` alone to get them all.
//!
//! # How the crate is laid out
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
//! `PolyTree` → `PolyTree64`. The modules: point helpers (`int_point_extensions`), one polygon
//! ([`polygon_extensions`]), many polygons ([`polygons_extensions`]), and from a modified Clipper
//! 6.4.2 the statics ([`clipper_statics`], which adapts those types to [`clipper6`]) and
//! `CleanPolygon(s)` ([`mod@clean_polygon`]); the winding correction every extrude and revolve
//! starts from (`corrected_winding`); and the size rule curve flattening and arc tolerances
//! scale by ([`curve_tolerance`]). Every helper that runs Clipper 6 returns
//! `Result<_, `[`ClipperError`]`>`.
//!
//! Its neighbours: it stands only on clipper2-rust (the integer types) and vector-math (the
//! fixed-tie sort Clipper 6's intersection list needs, and `RectangleDouble`), so the mesh kernel
//! (polygon-mesh, geometry-processing) uses it without a 2D path model. MatterCAD's agg-path
//! depends on it and re-exports its modules under their usual paths (`agg_path::clipper6`,
//! `agg_path::clipper_library`, `agg_path::curve_tolerance`,
//! `agg_path::data_converters_2d::get_corrected_winding`), where it converts paths to and from
//! these polygons.

pub mod clean_polygon;
pub mod clipper6;
mod clipper_error;
pub mod clipper_statics;
mod corrected_winding;
pub mod curve_tolerance;
mod int_point_extensions;
pub mod polygon_extensions;
pub mod polygons_extensions;

pub use clean_polygon::{clean_polygon, clean_polygons, DEFAULT_CLEAN_DISTANCE};
/// The join kinds the offset helpers take: clipper2-rust's enum,
/// which callers should name through here. Clipper 6 has no bevel join; `Bevel` offsets as
/// `Square` ([`clipper_statics::clipper6_join_type`]).
pub use clipper2_rust::JoinType;
/// The integer polygon types every helper speaks, from clipper2-rust, so a caller names them
/// through here.
pub use clipper2_rust::{FillRule, Path64, Paths64, Point64, PolyTree64, Rect64};
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
