//! A point-set hashtable over boundary-edge midpoints (derived from geometry3Sharp): boundary
//! edges hashed by their midpoints into a uniform grid, and the ball query
//! `MergeCoincidentEdges` uses. A ball query returns its hits in a fixed order: the centre
//! cell, then the 26 neighbours in `GRID_OFFSETS_26` order, each cell in insertion order, so
//! the merges (and the repaired mesh) are the same on every run.

use std::collections::HashMap;

use super::dmesh3::{DMesh3, G3Exception};

/// The 26 neighbour offsets of a grid cell: face, edge, then corner neighbours.
const GRID_OFFSETS_26: [[i32; 3]; 26] = [
    // face-nbrs
    [0, 0, -1],
    [0, 0, 1],
    [-1, 0, 0],
    [1, 0, 0],
    [0, -1, 0],
    [0, 1, 0],
    // edge-nbrs (+y, 0, -y)
    [1, 1, 0],
    [-1, 1, 0],
    [0, 1, 1],
    [0, 1, -1],
    [1, 0, 1],
    [-1, 0, 1],
    [1, 0, -1],
    [-1, 0, -1],
    [1, -1, 0],
    [-1, -1, 0],
    [0, -1, 1],
    [0, -1, -1],
    // corner-nbrs (+y,-y)
    [1, 1, 1],
    [-1, 1, 1],
    [1, 1, -1],
    [-1, 1, -1],
    [1, -1, 1],
    [-1, -1, 1],
    [1, -1, -1],
    [-1, -1, -1],
];

/// The point a fraction `t` along edge `eid`, from its first vertex to its second.
pub(crate) fn get_edge_point(mesh: &DMesh3, eid: i32, t: f64) -> [f64; 3] {
    let [a, b] = mesh.get_edge_v(eid);
    let (v0, v1) = (mesh.vertex(a), mesh.vertex(b));
    let mt = 1.0 - t;
    std::array::from_fn(|k| mt * v0[k] + t * v1[k])
}

fn distance_squared(a: [f64; 3], b: [f64; 3]) -> f64 {
    let (dx, dy, dz) = (b[0] - a[0], b[1] - a[1], b[2] - a[2]);
    dx * dx + dy * dy + dz * dz
}

/// A uniform-grid hashtable whose "points" are boundary
/// edge ids, each at its edge's midpoint.
#[derive(Debug, Clone)]
pub(crate) struct BoundaryEdgeMidpointHashtable {
    grid: HashMap<[i32; 3], Vec<i32>>,
    origin: [f64; 3],
    cell_size: f64,
}

impl BoundaryEdgeMidpointHashtable {
    /// Hashes `mesh`'s boundary edges. The grid spans the midpoints' bounds, the largest
    /// dimension divided by `max_axis_subdivs` per cell.
    pub(crate) fn build(mesh: &DMesh3, max_axis_subdivs: i32) -> Self {
        // An empty box grown to contain every midpoint
        let mut min = [f64::MAX; 3];
        let mut max = [f64::MIN; 3];
        for eid in mesh.boundary_edge_indices() {
            let v = get_edge_point(mesh, eid, 0.5);
            for a in 0..3 {
                min[a] = min[a].min(v[a]);
                max[a] = max[a].max(v[a]);
            }
        }
        let width = (max[0] - min[0]).max(0.0);
        let height = (max[1] - min[1]).max(0.0);
        let depth = (max[2] - min[2]).max(0.0);
        let max_dim = width.max(height.max(depth));
        let cell_size = max_dim / max_axis_subdivs as f64;

        let mut table = BoundaryEdgeMidpointHashtable {
            grid: HashMap::new(),
            origin: min,
            cell_size,
        };
        for eid in mesh.boundary_edge_indices() {
            let idx = table.world_to_grid(get_edge_point(mesh, eid, 0.5));
            table.grid.entry(idx).or_default().push(eid);
        }
        table
    }

    /// The cell holding `p`. The casts truncate and saturate, so a far-off point lands in an
    /// edge cell instead of wrapping.
    fn world_to_grid(&self, p: [f64; 3]) -> [i32; 3] {
        std::array::from_fn(|a| ((p[a] - self.origin[a]) / self.cell_size) as i32)
    }

    /// The world position of cell `idx`'s minimum corner.
    fn grid_to_world(&self, idx: [i32; 3]) -> [f64; 3] {
        std::array::from_fn(|a| (idx[a] as f64 * self.cell_size) + self.origin[a])
    }

    /// The edges whose midpoint is strictly within
    /// `r` of `pt`. `Err` for a radius larger than a cell: the search only visits neighbouring
    /// cells. Every hit is returned, however many edge midpoints coincide.
    pub(crate) fn find_in_ball(
        &self,
        mesh: &DMesh3,
        pt: [f64; 3],
        r: f64,
    ) -> Result<Vec<i32>, G3Exception> {
        let mut out = Vec::new();
        let half_cell = self.cell_size * 0.5;
        let idx = self.world_to_grid(pt);
        let corner = self.grid_to_world(idx);
        let center: [f64; 3] = std::array::from_fn(|a| corner[a] + half_cell * 1.0);

        if r > self.cell_size {
            return Err(G3Exception(
                "PointSetHashtable.FindInBall: large radius unsupported",
            ));
        }

        let r2 = r * r;

        // check all in this cell
        if let Some(center_cell) = self.grid.get(&idx) {
            for &vid in center_cell {
                if distance_squared(pt, get_edge_point(mesh, vid, 0.5)) < r2 {
                    out.push(vid);
                }
            }
        }

        // if we are close to a cell border we need to check nbrs
        let d: [f64; 3] = std::array::from_fn(|a| pt[a] - center[a]);
        let max_abs = d[0].abs().max(d[1].abs().max(d[2].abs()));
        if max_abs + r > half_cell {
            // [TODO] could iterate over fewer cells here, if r is bounded by CellSize,
            // then we should only ever need to look at 3, depending on which octant we are in.
            for ioffset in GRID_OFFSETS_26 {
                let pt_to_face_center = [
                    center[0] + half_cell * ioffset[0] as f64 - pt[0],
                    center[1] + half_cell * ioffset[1] as f64 - pt[1],
                    center[2] + half_cell * ioffset[2] as f64 - pt[2],
                ];
                let min_abs = pt_to_face_center[0]
                    .abs()
                    .min(pt_to_face_center[1].abs().min(pt_to_face_center[2].abs()));
                if min_abs > r {
                    continue;
                }
                let nidx: [i32; 3] = std::array::from_fn(|a| idx[a].wrapping_add(ioffset[a]));
                if let Some(ncell) = self.grid.get(&nidx) {
                    for &vid in ncell {
                        if distance_squared(pt, get_edge_point(mesh, vid, 0.5)) < r2 {
                            out.push(vid);
                        }
                    }
                }
            }
        }
        Ok(out)
    }
}
