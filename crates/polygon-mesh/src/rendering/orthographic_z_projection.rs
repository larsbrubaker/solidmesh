//! agg-sharp `PolygonMesh/Rendering/OrthographicZProjection.cs`: the outline of a mesh seen from
//! straight above, as ClipperLib polygons (`GetClipperPolygons`). Each projected triangle is grown
//! by 100 clipper units and unioned into the result, so faces that share an edge merge into one
//! outline, and the union is shrunk back by the same 100 with round joins. `FitToCylinderObject3D`
//! uses it to find the outline whose weight it centers. `DrawTo` (render the projection into a
//! `Graphics2D`) is not ported: nothing here draws with it.

use agg_path::clipper_library::{
    int_point, ClipperError, JoinType, PolygonExtensions, PolygonsExtensions,
};
use clipper2_rust::{FillRule, Path64, Paths64};
use vector_math::Vector2;

/// C#'s default `meshToPolygonScale`: clipper units per mesh unit.
pub const DEFAULT_MESH_TO_POLYGON_SCALE: f64 = 1000.0;

/// C# `GetClipperPolygons(polygons, meshToPolygonScale)`: the union of the triangles, each
/// scaled into clipper units, grown by 100 before the union and the union shrunk by 100 after it
/// (round joins), so shared edges close up.
pub fn get_clipper_polygons(
    polygons: impl IntoIterator<Item = (Vector2, Vector2, Vector2)>,
    mesh_to_polygon_scale: f64,
) -> Result<Paths64, ClipperError> {
    let mut polygon_projected: Paths64 = Vec::new();
    for polygon in polygons {
        let poly_face: Path64 = vec![
            int_point(
                polygon.0.x * mesh_to_polygon_scale,
                polygon.0.y * mesh_to_polygon_scale,
            ),
            int_point(
                polygon.1.x * mesh_to_polygon_scale,
                polygon.1.y * mesh_to_polygon_scale,
            ),
            int_point(
                polygon.2.x * mesh_to_polygon_scale,
                polygon.2.y * mesh_to_polygon_scale,
            ),
        ];
        // C# `Union(Polygons other)` defaults to even-odd
        polygon_projected =
            polygon_projected.union(&poly_face.offset(100.0)?, FillRule::EvenOdd)?;
    }

    // and move it back to the right size
    polygon_projected.offset(-100.0, JoinType::Round)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two triangles sharing a diagonal come back as the one square they make, at its size.
    #[test]
    fn rust_only_two_triangles_of_a_square_union_into_one_outline() {
        let a = Vector2::new(0.0, 0.0);
        let b = Vector2::new(10.0, 0.0);
        let c = Vector2::new(10.0, 10.0);
        let d = Vector2::new(0.0, 10.0);
        let polygons = get_clipper_polygons([(a, b, c), (a, c, d)], 1000.0).unwrap();
        assert_eq!(polygons.len(), 1);
        let bounds = polygons[0].get_bounds();
        assert!((bounds.left - 0.0).abs() <= 2.0, "{bounds:?}");
        assert!((bounds.right - 10000.0).abs() <= 2.0, "{bounds:?}");
        assert!((bounds.top - 10000.0).abs() <= 2.0, "{bounds:?}");
    }
}
