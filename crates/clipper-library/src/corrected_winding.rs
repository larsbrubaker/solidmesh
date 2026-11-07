//! `GetCorrectedWinding`: polygons cleaned and re-wound so every outer contour and hole turns the
//! way Clipper expects, the first step of every extrude and revolve and of a path's offset (in
//! MatterCAD, agg-path's `VertexSourceToClipperPolygons::offset`). The clean and the intersection
//! run on
//! [`crate::clipper6`] through [`crate::clipper_statics`].

use clipper2_rust::{ClipType, FillRule, Path64, Paths64, Point64};

use crate::clean_polygon::clean_polygons;
use crate::clipper_error::ClipperError;
use crate::clipper_statics::{closed_paths_from_poly_tree, execute_tree, get_bounds};
use crate::curve_tolerance;
use crate::polygons_extensions::{PolygonsExtensions, DEFAULT_CLIPPER_UNITS_PER_UNIT};

/// The polygons cleaned and intersected (even-odd) with their own
/// bounds grown by 10 units, which leaves every outer contour and hole wound the way clipper
/// expects.
///
/// The clean merges points closer than clipper's 1.415 units - an absolute distance, so at
/// desk size it thins a finely flattened curve a great deal and at room size hardly at all. It
/// grows with the polygons' size by [`curve_tolerance`], and is exactly 1.415 at or under a
/// 300 mm bed (assuming the default 1000 clipper units per mm).
pub fn get_corrected_winding(polygons_to_fix: &Paths64) -> Result<Paths64, ClipperError> {
    let fix_bounds = polygons_to_fix.get_bounds();
    let clean_distance = curve_tolerance::for_size(
        1.415,
        vector_math::ieee_math::max_f64(fix_bounds.width(), fix_bounds.height())
            / DEFAULT_CLIPPER_UNITS_PER_UNIT,
    );
    let polygons_to_fix = clean_polygons(polygons_to_fix, clean_distance);
    let mut bounds = get_bounds(&polygons_to_fix);
    bounds.left -= 10;
    bounds.top -= 10;
    bounds.bottom += 10;
    bounds.right += 10;

    let bounds_polygon: Path64 = vec![
        Point64::new(bounds.left, bounds.top),
        Point64::new(bounds.right, bounds.top),
        Point64::new(bounds.right, bounds.bottom),
        Point64::new(bounds.left, bounds.bottom),
    ];

    let intersection_result = execute_tree(
        ClipType::Intersection,
        &polygons_to_fix,
        &vec![bounds_polygon],
        FillRule::EvenOdd,
    )?;

    Ok(closed_paths_from_poly_tree(&intersection_result))
}
