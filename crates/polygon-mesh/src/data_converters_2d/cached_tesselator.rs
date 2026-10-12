//! A tessellator that keeps its output: agg-sharp `DataConverters2D/CachedTesselator.cs`, with
//! `VertexSourceToTesselator.SendShapeToTesselator` as [`CachedTesselator::send_shape`].
//!
//! C# subclasses the callback-driven GLU port (`Tesselate.Tesselator`): vertices it is handed
//! and the intersections it combines go into `VerticesCache`, and every triangle corner it
//! emits goes into `IndicesCache` with the edge flag in force. Here tess2-rust runs the same
//! SGI algorithm (NonZero winding, C#'s default, triangles only, as C#'s edge-flag callback
//! forces) and its render order fills the same two caches: C#'s vertex numbering (input
//! vertices, then combined ones) and C#'s triangle order, corner for corner.

use agg_path::vertex_data::VertexData;
use agg_path::vertex_source::VertexSource;
use tess2_rust::{ElementType, Tessellator, WindingRule};
use vector_math::Vector2;

/// One triangle corner: an index into [`CachedTesselator::vertices_cache`] and whether the edge
/// from this corner to the next one in the triangle is on the outline (C# `RenderIndices`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RenderIndices {
    pub index: i32,
    pub is_edge: bool,
}

/// C# `CachedTesselator`: the triangulated outline, three [`RenderIndices`] per triangle.
#[derive(Clone, Debug, Default)]
pub struct CachedTesselator {
    /// C# `VerticesCache` (each `AddedVertex.Position`).
    pub vertices_cache: Vec<Vector2>,
    /// C# `IndicesCache`.
    pub indices_cache: Vec<RenderIndices>,
}

impl CachedTesselator {
    pub fn new() -> Self {
        Self::default()
    }

    /// C# `Clear()`.
    pub fn clear(&mut self) {
        self.vertices_cache.clear();
        self.indices_cache.clear();
    }

    /// C# `VertexSourceToTesselator.SendShapeToTesselator(tesselator, vertexSource)`.
    pub fn send_vertex_source(&mut self, vertex_source: &dyn VertexSource) {
        self.send_shape(&vertex_source.vertices());
    }

    /// C# `VertexSourceToTesselator.SendShapeToTesselator(tesselator, IEnumerable<VertexData>)`:
    /// a move-to or any end_poly ends the contour, only vertex commands carry a point, NaN
    /// points are skipped, and a Stop ends the walk. The caches are replaced (C# clears them
    /// in `BeginPolygon`).
    pub fn send_shape(&mut self, vertex_source: &[VertexData]) {
        self.clear();

        let mut contours: Vec<Vec<f64>> = Vec::new();
        let mut current: Vec<f64> = Vec::new();
        let mut have_begun_contour = false;
        for vertex_data in vertex_source {
            if vertex_data.is_stop() {
                break;
            }

            if have_begun_contour && (vertex_data.command.is_end_poly() || vertex_data.is_move_to())
            {
                contours.push(std::mem::take(&mut current));
                have_begun_contour = false;
            }

            if vertex_data.is_vertex() {
                have_begun_contour = true;
                let position = vertex_data.position;
                if !position.x.is_nan() && !position.y.is_nan() {
                    current.extend_from_slice(&[position.x, position.y]);
                }
            }
        }
        if have_begun_contour {
            contours.push(current);
        }

        self.tessellate(&contours);
    }

    /// Runs the tessellator over the contours (flat x, y pairs) and caches what it emits.
    fn tessellate(&mut self, contours: &[Vec<f64>]) {
        let mut tess = Tessellator::new();
        for contour in contours {
            if !contour.is_empty() {
                tess.add_contour(2, contour);
            }
        }

        // C# swallows a failed tessellation (release builds catch everything) and keeps what
        // was cached, which for a polygon that never finished is nothing.
        if !tess.tessellate(WindingRule::NonZero, ElementType::Polygons, 3, 2, None) {
            return;
        }

        // tess2-rust's render order is the callback GLU port's: every input vertex in order, then
        // each combined vertex as the sweep made it (C# `AddVertex` and `CallCombine` both append
        // to `VerticesCache`, so unused and repeated positions are kept), and the triangles in
        // the order `RenderLonelyTriangles` emits them, with the edge flag in force per corner.
        let Some(order) = tess.render_order() else {
            return;
        };
        self.vertices_cache = order
            .vertices
            .iter()
            .map(|v| Vector2::new(v[0], v[1]))
            .collect();
        self.indices_cache = order
            .indices
            .iter()
            .map(|r| RenderIndices {
                index: r.index as i32,
                is_edge: r.is_edge,
            })
            .collect();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use agg_path::vertex_storage::VertexStorage;

    fn square_with_hole() -> VertexStorage {
        let mut path = VertexStorage::new();
        path.move_to(0.0, 0.0);
        path.line_to(10.0, 0.0);
        path.line_to(10.0, 10.0);
        path.line_to(0.0, 10.0);
        path.close_polygon();
        path.move_to(3.0, 3.0);
        path.line_to(3.0, 7.0);
        path.line_to(7.0, 7.0);
        path.line_to(7.0, 3.0);
        path.close_polygon();
        path
    }

    fn signed_area(t: &CachedTesselator, i: usize) -> f64 {
        let p = |k: usize| t.vertices_cache[t.indices_cache[i + k].index as usize];
        let (a, b, c) = (p(0), p(1), p(2));
        ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)) / 2.0
    }

    #[test]
    fn rust_only_square_with_hole_tessellates_to_its_area_with_outline_edges_flagged() {
        let mut tesselator = CachedTesselator::new();
        tesselator.send_vertex_source(&square_with_hole());
        assert_eq!(tesselator.indices_cache.len() % 3, 0);
        let area: f64 = (0..tesselator.indices_cache.len())
            .step_by(3)
            .map(|i| signed_area(&tesselator, i))
            .sum();
        assert!((area - 84.0).abs() < 1e-9, "area {area}");
        // every triangle is counter clockwise
        for i in (0..tesselator.indices_cache.len()).step_by(3) {
            assert!(signed_area(&tesselator, i) > 0.0);
        }
        // 8 outline edges, each flagged exactly once
        let edges = tesselator
            .indices_cache
            .iter()
            .filter(|r| r.is_edge)
            .count();
        assert_eq!(edges, 8);
    }

    /// C#'s callback order on the square with a hole: the caches hold the eight input vertices
    /// in input order and the triangles as agg-sharp's GLU port renders them (the sequence
    /// tess2-rust's `render_order` tests pin and its lion conformance test checks against
    /// recorded C# output), so a mesh built from them has C#'s vertex and face order.
    #[test]
    fn rust_only_square_with_hole_caches_in_csharp_render_order() {
        let mut tesselator = CachedTesselator::new();
        tesselator.send_vertex_source(&square_with_hole());
        let positions: Vec<(f64, f64)> = tesselator
            .vertices_cache
            .iter()
            .map(|v| (v.x, v.y))
            .collect();
        assert_eq!(
            positions,
            vec![
                (0.0, 0.0),
                (10.0, 0.0),
                (10.0, 10.0),
                (0.0, 10.0),
                (3.0, 3.0),
                (3.0, 7.0),
                (7.0, 7.0),
                (7.0, 3.0),
            ]
        );
        let corners: Vec<(i32, bool)> = tesselator
            .indices_cache
            .iter()
            .map(|r| (r.index, r.is_edge))
            .collect();
        let (f, t) = (false, true);
        assert_eq!(
            corners,
            vec![
                (0, f),
                (4, f),
                (3, t),
                (4, f),
                (0, t),
                (1, f),
                (4, f),
                (1, f),
                (7, t),
                (7, f),
                (1, f),
                (6, t),
                (3, f),
                (5, f),
                (2, t),
                (5, f),
                (3, f),
                (4, t),
                (2, f),
                (5, t),
                (6, f),
                (2, f),
                (6, f),
                (1, t),
            ]
        );
    }

    #[test]
    fn rust_only_empty_shape_gives_nothing() {
        let mut tesselator = CachedTesselator::new();
        tesselator.send_shape(&[]);
        assert!(tesselator.indices_cache.is_empty());
    }
}
