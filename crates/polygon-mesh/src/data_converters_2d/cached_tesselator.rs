//! A tessellator that keeps its output: agg-sharp `DataConverters2D/CachedTesselator.cs`, with
//! `VertexSourceToTesselator.SendShapeToTesselator` as [`CachedTesselator::send_shape`].
//!
//! C# subclasses the callback-driven GLU port (`Tesselate.Tesselator`): vertices it is handed
//! and the intersections it combines go into `VerticesCache`, and every triangle corner it
//! emits goes into `IndicesCache` with the edge flag in force. Here tess2-rust runs the same
//! SGI algorithm (NonZero winding, C#'s default, triangles only, as C#'s edge-flag callback
//! forces) and its output is read back into the same two caches. Triangle order and the corner
//! each triangle starts on may differ from C#'s render callbacks (divergence 230).

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

        let vertices = tess.vertices();
        self.vertices_cache = vertices
            .as_chunks::<2>()
            .0
            .iter()
            .map(|xy| Vector2::new(xy[0], xy[1]))
            .collect();

        let elements = tess.elements();
        let flags = tess.edge_flags();
        for (corner, &index) in elements.iter().enumerate() {
            if index == tess2_rust::tess::TESS_UNDEF {
                continue;
            }
            self.indices_cache.push(RenderIndices {
                index: index as i32,
                is_edge: flags.get(corner).copied().unwrap_or(0) != 0,
            });
        }
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

    #[test]
    fn rust_only_empty_shape_gives_nothing() {
        let mut tesselator = CachedTesselator::new();
        tesselator.send_shape(&[]);
        assert!(tesselator.indices_cache.is_empty());
    }
}
