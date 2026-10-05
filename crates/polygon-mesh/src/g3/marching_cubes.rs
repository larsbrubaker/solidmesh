//! geometry3Sharp's `MarchingCubes` (`mesh_generators/MarchingCubes.cs`, Boost Software License
//! 1.0): marching cubes over an implicit function, sampling each cell's corners and welding
//! vertices by exact position. agg-sharp's two-operand implicit boolean
//! (`BooleanProcessing.Do`) runs it; the n-ary `DoArray` runs `marching_cubes_pro`. Both share
//! [`find_iso`], the edge root-finder, and the tables in `sdf::marching_cubes_tables` (g3's two
//! copies are identical).
//!
//! C# polygonizes z-slabs with `gParallel.ForEach` under spin locks, so its vertex and triangle
//! order varies run to run; here the slabs run in order on one thread (C#'s `generate_basic`).
//! The vertex positions and the set of triangles are the same either way.

use std::collections::HashMap;

use super::dmesh3::{AxisAlignedBox3d, DMesh3};
use super::implicit3d::BoundedImplicitFunction3d;
use crate::sdf::marching_cubes_tables::{EDGE_INDICES, EDGE_TABLE, TRI_TABLE};

/// g3 `RootfindingModes`: how an edge crossing is converged on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)] // agg-sharp only selects LerpSteps; the others are g3's API.
pub(crate) enum RootfindingModes {
    SingleLerp,
    LerpSteps,
    Bisection,
}

/// g3 `find_iso`: the point where the field crosses `iso` between `p1` (value `valp1`) and
/// `p2` (value `valp2`). The two classes' copies differ only in bisection bookkeeping that is
/// never read, so they share this one.
#[allow(clippy::too_many_arguments)]
pub(crate) fn find_iso(
    implicit: &dyn BoundedImplicitFunction3d,
    iso: f64,
    root_mode: RootfindingModes,
    root_mode_steps: i32,
    p1: [f64; 3],
    p2: [f64; 3],
    valp1: f64,
    valp2: f64,
) -> [f64; 3] {
    // If one value is nearly the iso, returning that corner exactly can make non-manifold
    // vertices (several fans meet there and DMesh3 refuses them, leaving holes), so the point
    // is pulled slightly towards the other end.
    const DT: f64 = 0.999999;
    if (valp1 - valp2).abs() < 0.00001 {
        return std::array::from_fn(|a| (p1[a] + p2[a]) * 0.5);
    }
    if (iso - valp1).abs() < 0.00001 {
        return std::array::from_fn(|a| DT * p1[a] + (1.0 - DT) * p2[a]);
    }
    if (iso - valp2).abs() < 0.00001 {
        return std::array::from_fn(|a| DT * p2[a] + (1.0 - DT) * p1[a]);
    }

    // Keep min/max order so both cells sharing an edge compute the same point.
    let (mut a, mut b, mut fa, mut fb) = (p1, p2, valp1, valp2);
    if valp2 < valp1 {
        a = p2;
        b = p1;
        fb = valp1;
        fa = valp2;
    }

    if root_mode == RootfindingModes::Bisection {
        for _ in 0..root_mode_steps {
            let p_iso: [f64; 3] = std::array::from_fn(|i| (a[i] + b[i]) * 0.5);
            let mid_f = implicit.value(p_iso);
            if mid_f < iso {
                a = p_iso;
            } else {
                b = p_iso;
            }
        }
        // g3 `Vector3d.Lerp(a, b, 0.5)`.
        let s = 1.0 - 0.5;
        return std::array::from_fn(|i| s * a[i] + 0.5 * b[i]);
    }

    if root_mode == RootfindingModes::LerpSteps {
        for _ in 0..root_mode_steps {
            let mu = (iso - fa) / (fb - fa);
            let p_iso: [f64; 3] = std::array::from_fn(|i| a[i] + mu * (b[i] - a[i]));
            let mid_f = implicit.value(p_iso);
            if mid_f < iso {
                a = p_iso;
                fa = mid_f;
            } else {
                b = p_iso;
                fb = mid_f;
            }
        }
    }

    // Final lerp.
    let mu = (iso - fa) / (fb - fa);
    std::array::from_fn(|i| a[i] + mu * (b[i] - a[i]))
}

/// The cell grid both generators march: `(int)(extent / cube_size) + 1` cells per axis.
pub(crate) fn cell_dimensions(bounds: &AxisAlignedBox3d, cube_size: f64) -> [i32; 3] {
    [
        (bounds.width() / cube_size) as i32 + 1,
        (bounds.height() / cube_size) as i32 + 1,
        (bounds.depth() / cube_size) as i32 + 1,
    ]
}

/// g3 `MarchingCubes`, configured as agg-sharp sets it.
pub(crate) struct MarchingCubes<'a> {
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

/// A cell's corner positions and field values (C# `GridCell`).
struct GridCell {
    p: [[f64; 3]; 8],
    f: [f64; 8],
}

/// The weld key of a position: exact bits, with -0.0 folded onto 0.0 as .NET's
/// `double.GetHashCode`/`==` do.
fn position_key(p: [f64; 3]) -> [u64; 3] {
    p.map(|v| (v + 0.0).to_bits())
}

impl MarchingCubes<'_> {
    /// C# `Generate()`: the mesh of the iso surface (C# `Mesh`).
    pub(crate) fn generate(&self) -> DMesh3 {
        let mut mesh = DMesh3::default();
        let mut vertex_hash: HashMap<[u64; 3], i32> = HashMap::new();
        let [nx, ny, nz] = cell_dimensions(&self.bounds, self.cube_size);
        let mut cell = GridCell {
            p: [[0.0; 3]; 8],
            f: [0.0; 8],
        };
        let mut vert_list = [[0.0; 3]; 12];
        for zi in 0..nz {
            for yi in 0..ny {
                // Compute the full cell at x = 0, then slide along the row (half the evaluations).
                self.initialize_cell(&mut cell, [0, yi, zi]);
                self.polygonize_cell(&cell, &mut vert_list, &mut mesh, &mut vertex_hash);
                for xi in 1..nx {
                    self.shift_cell_x(&mut cell, xi);
                    self.polygonize_cell(&cell, &mut vert_list, &mut mesh, &mut vertex_hash);
                }
            }
        }
        mesh
    }

    /// C# `initialize_cell`. Each coordinate is `min + size * index` (never accumulated), so a
    /// corner shared by several cells gets the same bits in each, which the weld relies on.
    fn initialize_cell(&self, cell: &mut GridCell, idx: [i32; 3]) {
        let min = self.bounds.min;
        let c = self.cube_size;
        let x0 = min[0] + c * idx[0] as f64;
        let y0 = min[1] + c * idx[1] as f64;
        let z0 = min[2] + c * idx[2] as f64;
        let x1 = min[0] + c * (idx[0] + 1) as f64;
        let y1 = min[1] + c * (idx[1] + 1) as f64;
        let z1 = min[2] + c * (idx[2] + 1) as f64;
        cell.p = [
            [x0, y0, z0],
            [x1, y0, z0],
            [x1, y0, z1],
            [x0, y0, z1],
            [x0, y1, z0],
            [x1, y1, z0],
            [x1, y1, z1],
            [x0, y1, z1],
        ];
        for i in 0..8 {
            cell.f[i] = self.implicit.value(cell.p[i]);
        }
    }

    /// C# `shift_cell_x`: slides the cell from `xi - 1` to `xi`, reusing the shared face.
    fn shift_cell_x(&self, cell: &mut GridCell, xi: i32) {
        let x_prev = cell.p[1][0];
        let x1 = self.bounds.min[0] + self.cube_size * (xi + 1) as f64;
        for (i, x) in [x_prev, x1, x1, x_prev, x_prev, x1, x1, x_prev]
            .into_iter()
            .enumerate()
        {
            cell.p[i][0] = x;
        }
        cell.f[0] = cell.f[1];
        cell.f[3] = cell.f[2];
        cell.f[4] = cell.f[5];
        cell.f[7] = cell.f[6];
        for i in [1, 2, 5, 6] {
            cell.f[i] = self.implicit.value(cell.p[i]);
        }
    }

    /// C# `polygonize_cell`: the cell's crossings and triangles.
    fn polygonize_cell(
        &self,
        cell: &GridCell,
        vert_list: &mut [[f64; 3]; 12],
        mesh: &mut DMesh3,
        vertex_hash: &mut HashMap<[u64; 3], i32>,
    ) {
        let mut cube_index = 0usize;
        for i in 0..8 {
            if cell.f[i] < self.iso_value {
                cube_index |= 1 << i;
            }
        }
        let edges = EDGE_TABLE[cube_index];
        if edges == 0 {
            return;
        }
        for (i, [a, b]) in EDGE_INDICES.iter().copied().enumerate() {
            if edges & (1 << i) != 0 {
                vert_list[i] = find_iso(
                    self.implicit,
                    self.iso_value,
                    self.root_mode,
                    self.root_mode_steps,
                    cell.p[a],
                    cell.p[b],
                    cell.f[a],
                    cell.f[b],
                );
            }
        }
        let mut find_or_append = |pos: [f64; 3], mesh: &mut DMesh3| {
            *vertex_hash
                .entry(position_key(pos))
                .or_insert_with(|| mesh.append_vertex(pos))
        };
        let tris = &TRI_TABLE[cube_index];
        let mut i = 0;
        while tris[i] != -1 {
            let a = find_or_append(vert_list[tris[i] as usize], mesh);
            let b = find_or_append(vert_list[tris[i + 1] as usize], mesh);
            let c = find_or_append(vert_list[tris[i + 2] as usize], mesh);
            // A corner at the iso makes degenerate triangles; skipping them leaves no crack.
            if a != b && a != c && b != c {
                mesh.append_triangle(a, b, c);
            }
            i += 3;
        }
    }
}
