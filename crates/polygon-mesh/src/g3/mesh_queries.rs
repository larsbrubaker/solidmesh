//! geometry3Sharp `queries/MeshQueries.cs`, the edge-length statistics `MinimalHoleFill` sizes
//! its remesh with: `EdgeLengthStats` (all edges, or a prime-stride sample) and
//! `EdgeLengthStatsFromEdges` (a given edge list). The point-triangle distance from the same
//! C# file is in `tri_distance`.

use super::dmesh3::{DMesh3, G3Exception};
use super::vector3d::distance;

/// Minimum, maximum and average edge length, as C#'s three `out` values.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct EdgeLengthStats {
    pub(crate) min: f64,
    pub(crate) max: f64,
    pub(crate) avg: f64,
}

impl EdgeLengthStats {
    fn new() -> Self {
        Self {
            min: f64::MAX,
            max: f64::MIN,
            avg: 0.0,
        }
    }

    fn add(&mut self, mesh: &DMesh3, eid: i32) {
        let [va, vb] = mesh.get_edge_v(eid);
        let len = distance(mesh.vertex(va), mesh.vertex(vb));
        if len < self.min {
            self.min = len;
        }
        if len > self.max {
            self.max = len;
        }
        self.avg += len;
    }
}

/// C# `MeshQueries.EdgeLengthStats(mesh, out min, out max, out avg, samples)`. With
/// `samples == 0` every edge id is visited in order; otherwise ids are stepped by 31337 modulo
/// `MaxEdgeID`. A mesh with `MaxEdgeID == 0` divides by zero in C#, which is returned as an
/// error here.
pub(crate) fn edge_length_stats(
    mesh: &DMesh3,
    samples: i32,
) -> Result<EdgeLengthStats, G3Exception> {
    let mut stats = EdgeLengthStats::new();
    let mut avg_count = 0;
    let max_id = mesh.max_edge_id();

    // if we are only taking some samples, use a prime-modulo-loop instead of random
    let n_prime = if samples == 0 { 1 } else { 31337 };
    let max_count = if samples == 0 { max_id } else { samples };

    let mut eid = 0;
    let mut count = 0;
    loop {
        if mesh.is_edge(eid) {
            stats.add(mesh, eid);
            avg_count += 1;
        }
        if max_id == 0 {
            return Err(G3Exception(
                "MeshQueries.EdgeLengthStats: attempted to divide by zero.",
            ));
        }
        eid = (eid + n_prime) % max_id;
        let go_on = eid != 0 && count < max_count;
        count += 1;
        if !go_on {
            break;
        }
    }

    stats.avg /= avg_count as f64;
    Ok(stats)
}

/// C# `MeshQueries.EdgeLengthStatsFromEdges(mesh, edges, out min, out max, out avg)`: dead edge
/// ids are skipped; with no live edge the average is NaN (0 / 0), as in C#.
pub(crate) fn edge_length_stats_from_edges(
    mesh: &DMesh3,
    edges: impl IntoIterator<Item = i32>,
) -> EdgeLengthStats {
    let mut stats = EdgeLengthStats::new();
    let mut avg_count = 0;
    for eid in edges {
        if mesh.is_edge(eid) {
            stats.add(mesh, eid);
            avg_count += 1;
        }
    }
    stats.avg /= avg_count as f64;
    stats
}
