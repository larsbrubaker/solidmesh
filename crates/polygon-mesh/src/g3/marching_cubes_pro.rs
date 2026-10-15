//! geometry3Sharp's `MarchingCubesPro` (Boost Software License 1.0): marching cubes that caches
//! corner values in a float grid and shares each crossing vertex by the grid edge it sits on, so
//! neighbouring cells reuse it exactly. The n-ary implicit boolean runs its full-grid generation;
//! there is no seeded continuation.
//!
//! The z-slabs run in order on one thread. That fixes the vertex order and which slab first caches
//! a shared corner (its value is used at double precision by the slab that computes it and at float
//! precision by later readers), both of which would vary run to run in parallel.

use std::collections::HashMap;

use super::dmesh3::{AxisAlignedBox3d, DMesh3};
use super::implicit3d::BoundedImplicitFunction3d;
use super::marching_cubes::{cell_dimensions, find_iso, RootfindingModes};
use crate::sdf::marching_cubes_tables::{EDGE_INDICES, EDGE_TABLE, TRI_TABLE};
use crate::sdf::DenseGrid3f;

/// The edge-axis flags of a vertex key: bits 60, 61 and 62, above the 48 bits the corner hash
/// fills, so an edge's axis never aliases a coordinate on any grid.
const EDGE_X: i64 = 1 << 60;
const EDGE_Y: i64 = 1 << 61;
const EDGE_Z: i64 = 1 << 62;

/// g3 `MarchingCubesPro`, configured as the implicit booleans set it.
pub(crate) struct MarchingCubesPro<'a> {
    pub(crate) implicit: &'a dyn BoundedImplicitFunction3d,
    pub(crate) iso_value: f64,
    pub(crate) bounds: AxisAlignedBox3d,
    pub(crate) cube_size: f64,
    pub(crate) root_mode: RootfindingModes,
    pub(crate) root_mode_steps: i32,
}

/// The state of one `Generate()` run.
struct Run<'r, 'a> {
    mc: &'r MarchingCubesPro<'a>,
    mesh: DMesh3,
    /// Float cache with `float.MaxValue` as the empty sentinel.
    corner_values: DenseGrid3f,
    edge_vertices: HashMap<i64, i32>,
}

/// A cell's corner indices and field values.
struct GridCell {
    i: [[i32; 3]; 8],
    f: [f64; 8],
}

/// 16 bits per axis.
fn corner_hash(x: i32, y: i32, z: i32) -> i64 {
    (x as i64 & 0xFFFF) | ((y as i64 & 0xFFFF) << 16) | ((z as i64 & 0xFFFF) << 32)
}

/// The edge's lower corner and its axis.
fn edge_hash(idx1: [i32; 3], idx2: [i32; 3]) -> i64 {
    if idx1[0] != idx2[0] {
        corner_hash(idx1[0].min(idx2[0]), idx1[1], idx1[2]) | EDGE_X
    } else if idx1[1] != idx2[1] {
        corner_hash(idx1[0], idx1[1].min(idx2[1]), idx1[2]) | EDGE_Y
    } else {
        corner_hash(idx1[0], idx1[1], idx1[2].min(idx2[2])) | EDGE_Z
    }
}

impl MarchingCubesPro<'_> {
    /// The mesh of the iso surface.
    pub(crate) fn generate(&self) -> DMesh3 {
        let [nx, ny, nz] = cell_dimensions(&self.bounds, self.cube_size);
        let mut run = Run {
            mc: self,
            mesh: DMesh3::default(),
            corner_values: DenseGrid3f::new(
                (nx + 1).max(0) as usize,
                (ny + 1).max(0) as usize,
                (nz + 1).max(0) as usize,
                f32::MAX,
            ),
            edge_vertices: HashMap::new(),
        };
        let mut cell = GridCell {
            i: [[0; 3]; 8],
            f: [0.0; 8],
        };
        let mut vert_index_list = [0i32; 12];
        for zi in 0..nz {
            for yi in 0..ny {
                // Compute the full cell at x = 0, then slide along the row.
                run.initialize_cell(&mut cell, [0, yi, zi]);
                run.polygonize_cell(&cell, &mut vert_index_list);
                for xi in 1..nx {
                    run.shift_cell_x(&mut cell, xi);
                    run.polygonize_cell(&cell, &mut vert_index_list);
                }
            }
        }
        run.mesh
    }
}

impl Run<'_, '_> {
    fn corner_pos(&self, ijk: [i32; 3]) -> [f64; 3] {
        let min = self.mc.bounds.min;
        let c = self.mc.cube_size;
        [
            min[0] + c * ijk[0] as f64,
            min[1] + c * ijk[1] as f64,
            min[2] + c * ijk[2] as f64,
        ]
    }

    /// The cached value (as float), or evaluate, cache and return the double.
    fn corner_value(&mut self, idx: [i32; 3]) -> f64 {
        let g = &self.corner_values;
        let linear = idx[0] as usize + g.ni * (idx[1] as usize + g.nj * idx[2] as usize);
        let cached = g.buffer[linear];
        if cached != f32::MAX {
            return cached as f64;
        }
        let val = self.mc.implicit.value(self.corner_pos(idx));
        self.corner_values.buffer[linear] = val as f32;
        val
    }

    /// Fills a cell's corners, reading or caching their values in the value grid.
    fn initialize_cell(&mut self, cell: &mut GridCell, idx: [i32; 3]) {
        let [x, y, z] = idx;
        cell.i = [
            [x, y, z],
            [x + 1, y, z],
            [x + 1, y, z + 1],
            [x, y, z + 1],
            [x, y + 1, z],
            [x + 1, y + 1, z],
            [x + 1, y + 1, z + 1],
            [x, y + 1, z + 1],
        ];
        for i in 0..8 {
            cell.f[i] = self.corner_value(cell.i[i]);
        }
    }

    /// Moves a cell one step along x, reusing the four shared corners and sampling the new ones.
    fn shift_cell_x(&mut self, cell: &mut GridCell, xi: i32) {
        cell.f[0] = cell.f[1];
        cell.f[3] = cell.f[2];
        cell.f[4] = cell.f[5];
        cell.f[7] = cell.f[6];
        for (i, x) in [xi, xi + 1, xi + 1, xi, xi, xi + 1, xi + 1, xi]
            .into_iter()
            .enumerate()
        {
            cell.i[i][0] = x;
        }
        for i in [1, 2, 5, 6] {
            cell.f[i] = self.corner_value(cell.i[i]);
        }
    }

    /// The vertex on the grid edge between two corners, made once.
    fn edge_vertex_id(&mut self, idx1: [i32; 3], idx2: [i32; 3], f1: f64, f2: f64) -> i32 {
        let hash = edge_hash(idx1, idx2);
        if let Some(&vid) = self.edge_vertices.get(&hash) {
            return vid;
        }
        let pa = self.corner_pos(idx1);
        let pb = self.corner_pos(idx2);
        let mc = self.mc;
        let pos = find_iso(
            mc.implicit,
            mc.iso_value,
            mc.root_mode,
            mc.root_mode_steps,
            pa,
            pb,
            f1,
            f2,
        );
        let vid = self.mesh.append_vertex(pos);
        self.edge_vertices.insert(hash, vid);
        vid
    }

    fn polygonize_cell(&mut self, cell: &GridCell, vert_index_list: &mut [i32; 12]) {
        let mut cube_index = 0usize;
        for i in 0..8 {
            if cell.f[i] < self.mc.iso_value {
                cube_index |= 1 << i;
            }
        }
        let edges = EDGE_TABLE[cube_index];
        if edges == 0 {
            return;
        }
        for (i, [a, b]) in EDGE_INDICES.iter().copied().enumerate() {
            if edges & (1 << i) != 0 {
                vert_index_list[i] =
                    self.edge_vertex_id(cell.i[a], cell.i[b], cell.f[a], cell.f[b]);
            }
        }
        let tris = &TRI_TABLE[cube_index];
        let mut i = 0;
        while tris[i] != -1 {
            let a = vert_index_list[tris[i] as usize];
            let b = vert_index_list[tris[i + 1] as usize];
            let c = vert_index_list[tris[i + 2] as usize];
            // Never happens with edge-keyed vertices; kept as a guard against a degenerate
            // triangle.
            if a != b && a != c && b != c {
                self.mesh.append_triangle(a, b, c);
            }
            i += 3;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::edge_hash;

    #[test]
    fn edges_on_distant_rows_of_a_fine_grid_get_their_own_vertices() {
        // x edges at y = 0 and y = 4096 are different edges and need different vertex keys.
        assert_ne!(
            edge_hash([0, 0, 0], [1, 0, 0]),
            edge_hash([0, 4096, 0], [1, 4096, 0])
        );
        // An edge's axis is part of its key.
        let x = edge_hash([2, 3, 4], [3, 3, 4]);
        let y = edge_hash([2, 3, 4], [2, 4, 4]);
        let z = edge_hash([2, 3, 4], [2, 3, 5]);
        assert!(x != y && y != z && x != z);
    }
}
