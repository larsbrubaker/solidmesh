//! A port of agg-sharp's ClipperLib 6.4.2 (`clipper_library/clipper.cs`, Angus Johnson, Boost
//! Software License 1.0; see THIRD_PARTY_NOTICES.md), the polygon clipper and offsetter
//! MatterCAD's 2D geometry runs on. It exists so ring starts, ring order, collinear handling and
//! simplification match the C# bit for bit, which Clipper2 does not.
//!
//! The C# classes are reference graphs (edges, output points, output records, poly nodes); here
//! each kind lives in an arena `Vec` inside [`Clipper`] and the links are indices. Integer points
//! are clipper2-rust's `Point64` (`x`/`y` as `i64`), which is what agg-path's `clipper_library`
//! API already speaks; Clipper 6's `Int128` math is `i128`. Built with `use_lines` (open paths)
//! and without `use_xyz`/`use_int32`, as agg-sharp builds it.
//!
//! Files by responsibility: [`types`] (edges, out-records, enums, errors), [`clipper_base`]
//! (`ClipperBase`: AddPath, bounds, local minima, scanbeams, the AEL), [`engine`] (`Clipper`:
//! Execute and the sweep's winding rules), [`horizontals`] (intersections, horizontals, maxima),
//! [`out_polys`] (output points and records, results, PolyTree), [`joins`] (common-edge joins
//! and strictly-simple splitting), [`poly_tree`], [`offset`] (`ClipperOffset`), [`statics`]
//! (`Area`, `PointInPolygon`, `SimplifyPolygon(s)`, Minkowski, PolyTree walks) and
//! [`dotnet_sort`] (.NET's unstable `List.Sort`, which orders the intersection list).

mod clipper_base;
pub mod dotnet_sort;
mod engine;
mod horizontals;
mod intersections;
mod joins;
pub mod offset;
mod out_polys;
pub mod poly_tree;
pub mod statics;
pub mod types;

pub use engine::Clipper;
pub use offset::ClipperOffset;
pub use poly_tree::{PolyNode, PolyTree};
pub use statics::{
    area, closed_paths_from_poly_tree, get_bounds, minkowski_diff, minkowski_sum,
    minkowski_sum_paths, open_paths_from_poly_tree, orientation, point_in_polygon,
    poly_tree_to_paths, reverse_paths, simplify_polygon, simplify_polygons,
};
pub use types::{
    ClipType, ClipperError, EndType, IntPoint, IntRect, JoinType, Path, Paths, PolyFillType,
    PolyType,
};

#[cfg(test)]
mod tests;
