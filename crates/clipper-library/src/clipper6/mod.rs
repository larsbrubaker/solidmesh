//! ClipperLib 6.4.2 (Angus Johnson, Boost Software License 1.0; see THIRD_PARTY_NOTICES.md), the
//! polygon clipper and offsetter MatterCAD's 2D geometry runs on. It exists so ring starts, ring
//! order, collinear handling and simplification are exactly Clipper 6's, which saved designs'
//! geometry depends on and Clipper2 does not reproduce.
//!
//! Clipper 6's classes are reference graphs (edges, output points, output records, poly nodes); here
//! each kind lives in an arena `Vec` inside [`Clipper`] and the links are indices. Integer points
//! are clipper2-rust's `Point64` (`x`/`y` as `i64`), which is what the crate root's
//! helpers already speak; Clipper 6's `Int128` math is `i128`. Built with `use_lines` (open paths)
//! and without `use_xyz`/`use_int32`.
//!
//! Files by responsibility: [`types`] (edges, out-records, enums, errors), `clipper_base`
//! (`ClipperBase`: AddPath, bounds, local minima, scanbeams, the AEL), `engine` (`Clipper`:
//! Execute and the sweep's winding rules), `horizontals` (intersections, horizontals, maxima),
//! `out_polys` (output points and records, results, PolyTree), `joins` (common-edge joins
//! and strictly-simple splitting), [`poly_tree`], [`offset`] (`ClipperOffset`), [`statics`]
//! (`Area`, `PointInPolygon`, `SimplifyPolygon(s)`, Minkowski, PolyTree walks). The
//! intersection list is ordered with `vector_math::fixed_tie_sort`, an unstable introspective sort
//! whose order for equal keys is fixed.

mod clipper_base;
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
