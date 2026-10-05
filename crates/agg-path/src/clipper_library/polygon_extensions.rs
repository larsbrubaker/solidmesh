//! The extension methods on one ClipperLib polygon (`List<IntPoint>`, clipper2-rust's `Path64`)
//! that MatterCAD and agg-sharp's PolygonMesh call. Ports agg-sharp
//! `clipper_library/PolygonExtensions.cs`; the many-polygon helpers are in
//! [`super::polygons_extensions`]. Not ported (no caller): `GetBoundsLong` (no `RectangleLong`),
//! `GetWindingDirection2`, `PositionAllongPath` and `IntRect.ExpandToInclude`.

use clipper2_rust::{ClipType, FillRule, JoinType, Path64, Paths64, Point64};

use super::clipper_error::ClipperError;
use super::clipper_statics::execute;
use super::int_point_extensions::{calc_intersection, get_intersection, int_point, Intersection};
use super::polygons_extensions::PolygonsExtensions;
use crate::rectangle_double::RectangleDouble;

/// C# `PolygonExtensions`, on a single polygon.
pub trait PolygonExtensions {
    /// The bounds of every point ([`RectangleDouble::ZERO_INTERSECTION`] when empty).
    fn get_bounds(&self) -> RectangleDouble;

    /// Every edge of the closed polygon the segment `start`-`end` meets: the edge's start
    /// index, how it meets, and where (C# `GetIntersections`). Where the lines are parallel the
    /// position is C#'s `default(IntPoint)`, the origin.
    fn get_intersections(
        &self,
        start: Point64,
        end: Point64,
    ) -> Vec<(usize, Intersection, Point64)>;

    /// C# `GetWindingDirection`, documented as "1 if ccw, -1 if cw". Ported as written: it sums
    /// `(x2 + x1) * (y2 + y1)`, not the shoelace's `(x2 - x1) * (y2 + y1)`, so reversing a
    /// polygon does not change its answer.
    fn get_winding_direction(&self) -> i32;

    /// C# `Length(isClosed)`: the square root of [`PolygonExtensions::length_squared`].
    fn length(&self, is_closed: bool) -> f64;

    /// C# `LengthSquared(isClosed)`: the sum of each edge's squared length (with the closing
    /// edge when `is_closed`).
    fn length_squared(&self, is_closed: bool) -> f64;

    /// C# `Offset(distance)`: offset by `distance` clipper units with round joins, the arc
    /// tolerance cut by the polygon's own size ([`PolygonsExtensions::arc_tolerance_for`]).
    fn offset(&self, distance: f64) -> Result<Paths64, ClipperError>;

    /// C# `Offset(distance, arcTolerance)`: round joins at a fixed arc tolerance.
    fn offset_with_arc_tolerance(
        &self,
        distance: f64,
        arc_tolerance: f64,
    ) -> Result<Paths64, ClipperError>;

    /// C# `Rotate(radians)`: each point rotated about the origin and rounded back to the grid.
    fn rotate(&self, radians: f64) -> Path64;

    /// C# `Scale(scaleX, scaleY)`.
    fn scale(&self, scale_x: f64, scale_y: f64) -> Path64;

    /// C# `Subtract(Polygons other)`: even-odd difference.
    fn subtract(&self, other: &Paths64) -> Result<Paths64, ClipperError>;

    /// C# `Subtract(Polygon other)`.
    fn subtract_polygon(&self, other: &Path64) -> Result<Paths64, ClipperError>;

    /// C# `Translate(x, y, scale)`: each point moved by `(x, y) * scale`.
    fn translate(&self, x: f64, y: f64, scale: f64) -> Path64;
}

impl PolygonExtensions for Path64 {
    fn get_bounds(&self) -> RectangleDouble {
        let mut bounds = RectangleDouble::ZERO_INTERSECTION;
        for point in self {
            bounds.expand_to_include_xy(point.x as f64, point.y as f64);
        }
        bounds
    }

    fn get_intersections(
        &self,
        start: Point64,
        end: Point64,
    ) -> Vec<(usize, Intersection, Point64)> {
        let mut intersections = Vec::new();
        for i in 0..self.len() {
            let edge_start = self[i];
            let edge_end = self[(i + 1) % self.len()];

            let intersection = get_intersection(start, end, edge_start, edge_end);
            if intersection != Intersection::None {
                let position = calc_intersection(start, end, edge_start, edge_end)
                    .unwrap_or_else(|| Point64::new(0, 0));
                intersections.push((i, intersection, position));
            }
        }
        intersections
    }

    fn get_winding_direction(&self) -> i32 {
        let point_count = self.len();
        let mut total_turns = 0.0;
        for point_index in 0..point_count {
            let current_point = self[point_index];
            let next_point = self[(point_index + 1) % point_count];

            // sum  (x2 − x1)(y2 + y1) -- as the C# comment says; the C# computes the sums.
            // C# long arithmetic wraps on overflow (unchecked), so it wraps here too.
            let turn_amount = next_point
                .x
                .wrapping_add(current_point.x)
                .wrapping_mul(next_point.y.wrapping_add(current_point.y));

            total_turns += turn_amount as f64;
        }

        if total_turns > 0.0 {
            1
        } else {
            -1
        }
    }

    fn length(&self, is_closed: bool) -> f64 {
        self.length_squared(is_closed).sqrt()
    }

    fn length_squared(&self, is_closed: bool) -> f64 {
        let mut length = 0.0;
        if self.len() > 1 {
            let mut previous_point = self[0];
            if is_closed {
                previous_point = self[self.len() - 1];
            }
            for &current_point in &self[if is_closed { 0 } else { 1 }..] {
                let dx = previous_point.x - current_point.x;
                let dy = previous_point.y - current_point.y;
                length += (dx * dx + dy * dy) as f64;
                previous_point = current_point;
            }
        }
        length
    }

    fn offset(&self, distance: f64) -> Result<Paths64, ClipperError> {
        let arc_tolerance = vec![self.clone()]
            .arc_tolerance_for(super::polygons_extensions::DEFAULT_CLIPPER_UNITS_PER_UNIT);
        self.offset_with_arc_tolerance(distance, arc_tolerance)
    }

    fn offset_with_arc_tolerance(
        &self,
        distance: f64,
        arc_tolerance: f64,
    ) -> Result<Paths64, ClipperError> {
        vec![self.clone()].offset_with_arc_tolerance(distance, JoinType::Round, arc_tolerance)
    }

    fn rotate(&self, radians: f64) -> Path64 {
        let cos = radians.cos();
        let sin = radians.sin();
        self.iter()
            .map(|point| {
                let (x, y) = (point.x as f64, point.y as f64);
                int_point(x * cos - y * sin, y * cos + x * sin)
            })
            .collect()
    }

    fn scale(&self, scale_x: f64, scale_y: f64) -> Path64 {
        self.iter()
            .map(|point| int_point(point.x as f64 * scale_x, point.y as f64 * scale_y))
            .collect()
    }

    fn subtract(&self, other: &Paths64) -> Result<Paths64, ClipperError> {
        execute(
            ClipType::Difference,
            &vec![self.clone()],
            other,
            FillRule::EvenOdd,
        )
    }

    fn subtract_polygon(&self, other: &Path64) -> Result<Paths64, ClipperError> {
        self.subtract(&vec![other.clone()])
    }

    fn translate(&self, x: f64, y: f64, scale: f64) -> Path64 {
        self.iter()
            .map(|point| int_point(point.x as f64 + x * scale, point.y as f64 + y * scale))
            .collect()
    }
}
