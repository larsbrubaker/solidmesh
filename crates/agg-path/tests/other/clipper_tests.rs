// Cleaning a polygon with `agg_path::clipper_library::clean_polygon`.

use agg_path::clipper_library::{clean_polygon, int_point, length as point_length};
use clipper2_rust::{Path64, Point64};
use vector_math::math_helper::degrees_to_radians;

#[test]
fn clean_polygons_test() {
    let mut polygon = Path64::new();

    let length = 500;

    for i in 0..360 {
        let angle = degrees_to_radians(i as f64);
        polygon.push(int_point(
            angle.cos() * length as f64,
            angle.sin() * length as f64,
        ));
    }

    let cleaned_polygon = clean_polygon(&polygon, 2.0);

    // Ensure mid-point between points is within threshold
    for i in 0..cleaned_polygon.len().saturating_sub(1) {
        let sum = Point64::new(
            cleaned_polygon[i + 1].x + cleaned_polygon[i].x,
            cleaned_polygon[i + 1].y + cleaned_polygon[i].y,
        );
        // Integer division of each coordinate, truncating toward zero.
        let center_point = Point64::new(sum.x / 2, sum.y / 2);
        let dist_to_center = point_length(center_point);
        assert!((length - dist_to_center).abs() <= 3);
    }
}
