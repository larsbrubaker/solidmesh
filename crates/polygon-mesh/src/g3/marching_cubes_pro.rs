//! geometry3Sharp's `MarchingCubesPro` (`mesh_generators/MarchingCubesPro.cs`, Boost Software
//! License 1.0): marching cubes that caches corner values in a float grid and shares each
//! crossing vertex by the grid edge it sits on, so neighbouring cells reuse it exactly. agg-sharp's
//! n-ary implicit boolean (`BooleanProcessing.DoArray`) runs its `Generate()`; the seeded
//! `GenerateContinuation` is not reached and not ported.
//!
//! C# polygonizes z-slabs with `gParallel.ForEach`, so its vertex order, and which slab first
//! caches a shared corner (its value is used at double precision by the slab that computes it
//! and at float precision by later readers), vary run to run. Here the slabs run in order on one
//! thread (C#'s `generate_basic`), which is one of the orders C# can produce.

use std::collections::HashMap;

use super::dmesh3::{AxisAlignedBox3d, DMesh3};
use super::implicit3d::BoundedImplicitFunction3d;
use super::marching_cubes::{cell_dimensions, find_iso, RootfindingModes};
use crate::sdf::marching_cubes_tables::{EDGE_INDICES, EDGE_TABLE, TRI_TABLE};
use crate::sdf::DenseGrid3f;

/// C# `EDGE_X`/`EDGE_Y`/`EDGE_Z` are `const int 1 << 60/61/62`; C# masks an `int` shift count
/// to five bits, so they are really bits 28, 29 and 30 (which overlap the y field of the corner
/// hash only from y = 4096 on, beyond any grid agg-sharp builds). Kept as C# computes them.
const EDGE_X: i64 = 1 << (60 & 31);
const EDGE_Y: i64 = 1 << (61 & 31);
const EDGE_Z: i64 = 1 << (62 & 31);

/// g3 `MarchingCubesPro`, configured as agg-sharp sets it.
pub(crate) struct MarchingCubesPro<'a> {
    /// C# `Implicit`.
    pub(crate) implicit: &'a dyn BoundedImplicitFunction3d,
    /// C# `IsoValue`.
    pub(crate) iso_value: f64,
    /// C# `Bounds`.
    pub(crate) bounds: AxisAlignedBox3d,
    /// C# `CubeSize`.
    pub(crate) cube_size: f64,
    /// C# `RootMode`.
    pub(crate) root_mode: RootfindingModes,
    /// C# `RootModeSteps`.
    pub(crate) root_mode_steps: i32,
}

/// The state of one `Generate()` run.
struct Run<'r, 'a> {
    mc: &'r MarchingCubesPro<'a>,
    mesh: DMesh3,
    /// C# `corner_values_grid`: float cache with `float.MaxValue` as the empty sentinel.
    corner_values: DenseGrid3f,
    /// C# `edge_vertices`.
    edge_vertices: HashMap<i64, i32>,
}

/// A cell's corner indices and field values (C# `GridCell`).
struct GridCell {
    i: [[i32; 3]; 8],
    f: [f64; 8],
}

/// C# `corner_hash`: 16 bits per axis.
fn corner_hash(x: i32, y: i32, z: i32) -> i64 {
    (x as i64 & 0xFFFF) | ((y as i64 & 0xFFFF) << 16) | ((z as i64 & 0xFFFF) << 32)
}

/// C# `edge_hash`: the edge's lower corner and its axis.
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
    /// C# `Generate()`: the mesh of the iso surface (C# `Mesh`).
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
    /// C# `corner_pos`.
    fn corner_pos(&self, ijk: [i32; 3]) -> [f64; 3] {
        let min = self.mc.bounds.min;
        let c = self.mc.cube_size;
        [
            min[0] + c * ijk[0] as f64,
            min[1] + c * ijk[1] as f64,
            min[2] + c * ijk[2] as f64,
        ]
    }

    /// C# `corner_value_grid`: the cached value (as float), or evaluate, cache and return the
    /// double.
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

    /// C# `initialize_cell` (with `initialize_cell_values_grid`).
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

    /// C# `shift_cell_x` (with `initialize_cell_values_grid(cell, true)`).
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

    /// C# `edge_vertex_id`: the vertex on the grid edge between two corners, made once.
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

    /// C# `polygonize_cell`.
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
            // C#: "this should never happen anymore" with edge-keyed vertices.
            if a != b && a != c && b != c {
                self.mesh.append_triangle(a, b, c);
            }
            i += 3;
        }
    }
}
