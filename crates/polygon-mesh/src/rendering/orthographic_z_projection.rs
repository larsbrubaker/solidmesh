//! The outline of a mesh seen from straight above, as clipper polygons
//! ([`get_clipper_polygons`]). Each projected triangle is grown by 100 clipper units and
//! unioned into the result, so faces that share an edge merge into one outline, and the union
//! is shrunk back by the same 100 with round joins. The fit-to-cylinder operation uses it to
//! find the outline whose weight it centers. [`draw_to`] fills the up-facing triangles, seen
//! from above, into an image graphics: the part sheet's silhouettes.

use agg_path::clipper_library::{
    int_point, ClipperError, JoinType, PolygonExtensions, PolygonsExtensions,
};
use agg_path::image::image_graphics_2d::ImageGraphics2D;
use agg_path::vertex_storage::VertexStorage;
use clipper2_rust::{FillRule, Path64, Paths64};
use vector_math::Color;
use vector_math::{Matrix4X4, Vector2};

use crate::Mesh;

/// The default `mesh_to_polygon_scale`: clipper units per mesh unit, so a clipper unit is a
/// thousandth of a millimetre.
pub const DEFAULT_MESH_TO_POLYGON_SCALE: f64 = 1000.0;

/// Draws the mesh's silhouette from above: every face whose transformed normal points up (+Z),
/// projected onto XY, moved by `offset`, scaled by `scale`, and filled in `silhouette_color` as
/// one path.
pub fn draw_to(
    graphics_2d: &mut ImageGraphics2D<'_>,
    mesh: &Mesh,
    matrix: Matrix4X4,
    offset: Vector2,
    scale: f64,
    silhouette_color: Color,
) {
    let mut polygon_projected = VertexStorage::new();
    let corner = |index: i32| {
        let position = mesh.vertices[index as usize].transform(matrix);
        (Vector2::new(position.x as f64, position.y as f64) + offset) * scale
    };
    for face in mesh.faces.iter() {
        if face.normal.transform_normal(matrix).z > 0.0 {
            polygon_projected.move_to_vector(corner(face.v0));
            polygon_projected.line_to_vector(corner(face.v1));
            polygon_projected.line_to_vector(corner(face.v2));
            polygon_projected.line_to_vector(corner(face.v0));
        }
    }

    graphics_2d.render(&polygon_projected, silhouette_color);
}

/// The union of the triangles, each scaled into clipper units, grown by 100 before the union
/// and the union shrunk by 100 after it (round joins), so shared edges close up.
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
        // The union fills even-odd.
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
    /// A square plate seen from above fills its square, scaled and offset, and nothing beyond.
    #[test]
    fn rust_only_draw_to_fills_the_up_facing_silhouette() {
        use agg_path::image::image_buffer::ImageBuffer;
        let mesh = crate::solids::platonic_solids::create_cube(4.0, 4.0, 1.0);
        let mut image = ImageBuffer::new(20, 20);
        image.new_graphics_2d().clear(Color::WHITE);
        // The cube is centred on the origin: offset (2, 2) puts it at 0..4, scale 2 at 0..8.
        draw_to(
            &mut image.new_graphics_2d(),
            &mesh,
            Matrix4X4::IDENTITY,
            Vector2::new(2.0, 2.0),
            2.0,
            Color::BLACK,
        );
        assert_eq!(image.get_pixel(4, 4), Color::BLACK);
        assert_eq!(image.get_pixel(7, 7), Color::BLACK);
        assert_eq!(image.get_pixel(9, 9), Color::WHITE);
    }

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
