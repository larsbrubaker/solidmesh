//! A tessellator that keeps its output, fed contours by [`CachedTesselator::send_contours`].
//!
//! Vertices it is handed and the intersections it combines go into `vertices_cache`, and every
//! triangle corner it emits goes into `indices_cache` with the edge flag in force. tess2-rust runs
//! the SGI tessellator (NonZero winding, triangles only, which the edge flags require) and its
//! render order fills the two caches: input vertices first, then combined ones, and the triangles
//! in GLU's emit order, corner for corner, so the meshes built from them have a stable vertex and
//! face order.

use solidmesh_math::Vector2;
use tess2_rust::{ElementType, Tessellator, WindingRule};

/// One triangle corner: an index into [`CachedTesselator::vertices_cache`] and whether the edge
/// from this corner to the next one in the triangle is on the outline.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RenderIndices {
    pub index: i32,
    pub is_edge: bool,
}

/// The triangulated outline, three [`RenderIndices`] per triangle.
#[derive(Clone, Debug, Default)]
pub struct CachedTesselator {
    /// Every vertex the tessellator was handed or combined, in that order.
    pub vertices_cache: Vec<Vector2>,
    pub indices_cache: Vec<RenderIndices>,
}

impl CachedTesselator {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn clear(&mut self) {
        self.vertices_cache.clear();
        self.indices_cache.clear();
    }

    /// Tessellates the contours (each closed back to its first point), skipping points with a NaN
    /// coordinate and contours left with none. The caches are replaced.
    pub fn send_contours(&mut self, contours: &[Vec<Vector2>]) {
        self.clear();

        let contours: Vec<Vec<f64>> = contours
            .iter()
            .map(|contour| {
                contour
                    .iter()
                    .filter(|point| !point.x.is_nan() && !point.y.is_nan())
                    .flat_map(|point| [point.x, point.y])
                    .collect()
            })
            .collect();

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

        // A failed tessellation is not an error: the caches keep what was cached, which for a
        // polygon that never finished is nothing, so the shape simply draws no fill.
        if !tess.tessellate(WindingRule::NonZero, ElementType::Polygons, 3, 2, None) {
            return;
        }

        // tess2-rust's render order is the callback GLU port's: every input vertex in order, then
        // each combined vertex as the sweep made it (both append to `vertices_cache`, so unused and
        // repeated positions are kept), and the triangles in the order `RenderLonelyTriangles`
        // emits them, with the edge flag in force per corner.
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

    fn square_with_hole() -> Vec<Vec<Vector2>> {
        let contour = |points: [(f64, f64); 4]| points.map(|(x, y)| Vector2::new(x, y)).to_vec();
        vec![
            contour([(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]),
            contour([(3.0, 3.0), (3.0, 7.0), (7.0, 7.0), (7.0, 3.0)]),
        ]
    }

    fn signed_area(t: &CachedTesselator, i: usize) -> f64 {
        let p = |k: usize| t.vertices_cache[t.indices_cache[i + k].index as usize];
        let (a, b, c) = (p(0), p(1), p(2));
        ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)) / 2.0
    }

    #[test]
    fn rust_only_square_with_hole_tessellates_to_its_area_with_outline_edges_flagged() {
        let mut tesselator = CachedTesselator::new();
        tesselator.send_contours(&square_with_hole());
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

    /// GLU's callback order on the square with a hole: the caches hold the eight input vertices in
    /// input order and the triangles as GLU renders them (the sequence tess2-rust's `render_order`
    /// tests pin and its lion conformance test checks), so a mesh built from them has a fixed
    /// vertex and face order.
    #[test]
    fn rust_only_square_with_hole_caches_in_render_order() {
        let mut tesselator = CachedTesselator::new();
        tesselator.send_contours(&square_with_hole());
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
        tesselator.send_contours(&[]);
        assert!(tesselator.indices_cache.is_empty());
    }
}
