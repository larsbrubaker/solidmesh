//! Marching cubes straight over a sampled distance field, with the [`DenseGrid3f`] field it
//! reads.
//!
//! The corners are the grid samples themselves and every cell is tested (skipping cells far
//! from the iso tore the surface across open holes). Vertices are keyed by the grid edge they
//! sit on, so neighbouring cells share them and the result is closed. g3's tables and corner
//! order are used, so the winding matches g3's (outward for a field negative inside).
//!
//! Ambiguous faces: a cell face whose corners alternate inside / outside is resolved by the
//! asymptotic decider (the inside corners join when the face's bilinear saddle is inside), and
//! such a cell is built from its per-face segments chained into loops instead of the table. The
//! decider reads only the shared face's four values in a form symmetric in corner order, so both
//! cells sharing the face draw the same segments and the mesh stays closed and manifold. A loop
//! of three or four crossings is fanned; a longer one gets a centre vertex. Loops are never
//! joined through the cell interior, which can only lose a sub-cell tunnel.
//!
//! Both passes work slice by slice and stitch the per-slice lists in slice order, so running the
//! slices in parallel would give the same mesh; here they run in order on one thread.

use super::marching_cubes_tables::{EDGE_INDICES, EDGE_TABLE, TRI_TABLE};
use crate::cancel::CancelToken;
use crate::mesh::Mesh;
use vector_math::Vector3;

/// g3's `DenseGrid3f`: `ni * nj * nk` floats, x fastest (`i + ni * (j + nj * k)`).
#[derive(Clone, Debug, PartialEq)]
pub struct DenseGrid3f {
    pub ni: usize,
    pub nj: usize,
    pub nk: usize,
    pub buffer: Vec<f32>,
}

impl DenseGrid3f {
    /// An `ni * nj * nk` grid with every sample `initial_value`.
    pub fn new(ni: usize, nj: usize, nk: usize, initial_value: f32) -> Self {
        DenseGrid3f {
            ni,
            nj,
            nk,
            buffer: vec![initial_value; ni * nj * nk],
        }
    }

    /// The number of samples.
    pub fn size(&self) -> usize {
        self.buffer.len()
    }
}

/// g3 corner order: offsets of the eight corners from the cell's lowest node.
const CORNER_OFFSET: [[usize; 3]; 8] = [
    [0, 0, 0],
    [1, 0, 0],
    [1, 0, 1],
    [0, 0, 1],
    [0, 1, 0],
    [1, 1, 0],
    [1, 1, 1],
    [0, 1, 1],
];

/// Each face's corners counter-clockwise seen from outside the cell.
const FACE_CORNERS: [[usize; 4]; 6] = [
    [0, 1, 2, 3],
    [4, 7, 6, 5],
    [0, 3, 7, 4],
    [1, 5, 6, 2],
    [0, 4, 5, 1],
    [3, 2, 6, 7],
];

/// `FaceEdges[face][i]`: the cell edge from `FACE_CORNERS[face][i]` to the next corner.
fn build_face_edges() -> [[usize; 4]; 6] {
    let mut face_edges = [[0; 4]; 6];
    for face in 0..6 {
        for i in 0..4 {
            let (a, b) = (FACE_CORNERS[face][i], FACE_CORNERS[face][(i + 1) % 4]);
            for (e, &[ea, eb]) in EDGE_INDICES.iter().enumerate() {
                if (ea == a && eb == b) || (ea == b && eb == a) {
                    face_edges[face][i] = e;
                }
            }
        }
    }
    face_edges
}

/// Cases with at least one face whose corners alternate inside / outside.
fn build_ambiguous_cases() -> [bool; 256] {
    let mut ambiguous = [false; 256];
    for (cube_index, entry) in ambiguous.iter_mut().enumerate() {
        for corners in FACE_CORNERS {
            let s = |n: usize| cube_index & (1 << corners[n]) != 0;
            if s(0) == s(2) && s(1) == s(3) && s(0) != s(1) {
                *entry = true;
            }
        }
    }
    ambiguous
}

/// The surface where the field equals `iso`, or `None` when `cancel` is set.
pub fn extract(
    grid: &DenseGrid3f,
    origin: Vector3,
    cell_size: f64,
    iso: f64,
    cancel: &CancelToken,
) -> Option<Mesh> {
    let (ni, nj, nk) = (grid.ni, grid.nj, grid.nk);
    let values = &grid.buffer;
    let slice_size = ni * nj;
    let iso_f = iso as f32;

    // Pass 1: every grid edge that crosses the iso gets a vertex, gathered per node slice and
    // keyed within its slice as (node in slice) * 3 + axis.
    let mut slice_vertices: Vec<Vec<f64>> = Vec::with_capacity(nk);
    let mut slice_edges: Vec<Vec<usize>> = Vec::with_capacity(nk);
    for k in 0..nk {
        if cancel.is_cancelled() {
            return None;
        }
        let mut positions = Vec::new();
        let mut edges = Vec::new();
        for j in 0..nj {
            for i in 0..ni {
                let in_slice = j * ni + i;
                let node = k * slice_size + in_slice;
                let f = values[node] - iso_f;
                let mut add_crossing = |exists: bool, other: usize, axis: usize| {
                    if !exists {
                        return;
                    }
                    let g = values[other] - iso_f;
                    if (f < 0.0) == (g < 0.0) {
                        return;
                    }
                    // A node exactly on the iso would give t = 0 (or 1), and each of its
                    // crossing edges a vertex at the same point, so the triangles between them
                    // would have no area. Keeping t a thousandth of a cell off the node keeps
                    // those vertices apart and moves the surface by no more than that.
                    let t = (f as f64 / (f - g) as f64).clamp(1e-3, 1.0 - 1e-3);
                    edges.push(in_slice * 3 + axis);
                    let along = |a: usize| if axis == a { t } else { 0.0 };
                    positions.push(origin.x + (i as f64 + along(0)) * cell_size);
                    positions.push(origin.y + (j as f64 + along(1)) * cell_size);
                    positions.push(origin.z + (k as f64 + along(2)) * cell_size);
                };
                add_crossing(i + 1 < ni, node + 1, 0);
                add_crossing(j + 1 < nj, node + ni, 1);
                add_crossing(k + 1 < nk, node + slice_size, 2);
            }
        }
        slice_vertices.push(positions);
        slice_edges.push(edges);
    }

    let mut slice_base = vec![0usize; nk];
    let mut vertex_count = 0;
    for k in 0..nk {
        slice_base[k] = vertex_count;
        vertex_count += slice_edges[k].len();
    }
    let mut vertices: Vec<f64> = slice_vertices.concat();

    // For each of the 12 cell edges, its key in a slab's two-slice map, relative to the cell's
    // lowest node: the upper slice's keys follow the lower's.
    let mut edge_offset = [0usize; 12];
    for (e, &[a, b]) in EDGE_INDICES.iter().enumerate() {
        let (ca, cb) = (CORNER_OFFSET[a], CORNER_OFFSET[b]);
        let low = if ca[0] + ca[1] + ca[2] <= cb[0] + cb[1] + cb[2] {
            ca
        } else {
            cb
        };
        let axis = if ca[0] != cb[0] {
            0
        } else if ca[1] != cb[1] {
            1
        } else {
            2
        };
        edge_offset[e] = (low[0] + low[1] * ni + low[2] * slice_size) * 3 + axis;
    }
    let corner_offset = CORNER_OFFSET.map(|c| c[0] + c[1] * ni + c[2] * slice_size);

    let cells = CellBuilder {
        values,
        iso_f,
        corner_offset,
        edge_offset,
        face_edges: build_face_edges(),
    };
    let has_ambiguous_face = build_ambiguous_cases();

    // Pass 2: triangles, per slab of cells between node slices k and k + 1. The map is never
    // cleared: a cell only reads the keys of edges that cross, just written from this slab.
    let mut edge_vertex = vec![0i32; 2 * 3 * slice_size];
    let mut slab_triangles: Vec<Vec<i32>> = Vec::new();
    let mut slab_centres: Vec<Vec<f64>> = Vec::new();
    for k in 0..nk.saturating_sub(1) {
        if cancel.is_cancelled() {
            return None;
        }
        for half in 0..2 {
            let first = slice_base[k + half];
            let shift = half * 3 * slice_size;
            for (n, &edge) in slice_edges[k + half].iter().enumerate() {
                edge_vertex[shift + edge] = (first + n) as i32;
            }
        }

        let mut triangles = Vec::new();
        let mut centres = Vec::new();
        for j in 0..nj.saturating_sub(1) {
            for i in 0..ni.saturating_sub(1) {
                let in_slice = j * ni + i;
                let node = k * slice_size + in_slice;
                let mut cube_index = 0usize;
                for (c, offset) in corner_offset.iter().enumerate() {
                    if values[node + offset] < iso_f {
                        cube_index |= 1 << c;
                    }
                }
                if EDGE_TABLE[cube_index] == 0 {
                    continue;
                }

                let key = in_slice * 3;
                if has_ambiguous_face[cube_index] {
                    let cell = Cell {
                        node,
                        key,
                        cube_index,
                    };
                    cells.add_cell_loops(
                        cell,
                        &edge_vertex,
                        &vertices,
                        &mut triangles,
                        &mut centres,
                    );
                    continue;
                }

                let row = &TRI_TABLE[cube_index];
                let mut t = 0;
                while row[t] != -1 {
                    for n in 0..3 {
                        triangles.push(edge_vertex[key + edge_offset[row[t + n] as usize]]);
                    }
                    t += 3;
                }
            }
        }
        slab_triangles.push(triangles);
        slab_centres.push(centres);
    }

    // Centre vertices were numbered per slab as -(n + 1); they go after the edge vertices.
    let mut faces = Vec::new();
    let mut next_centre = vertex_count as i32;
    for (triangles, centres) in slab_triangles.iter().zip(&slab_centres) {
        let first = next_centre;
        vertices.extend_from_slice(centres);
        next_centre += (centres.len() / 3) as i32;
        faces.extend(
            triangles
                .iter()
                .map(|&index| if index >= 0 { index } else { first - index - 1 }),
        );
    }

    Some(Mesh::from_arrays(&vertices, &faces))
}

/// The cell an ambiguous case is being built for: its lowest node, its key in the slab map and
/// which of its corners are inside.
#[derive(Clone, Copy)]
struct Cell {
    node: usize,
    key: usize,
    cube_index: usize,
}

/// What every ambiguous cell reads: the field and the per-grid offset tables.
struct CellBuilder<'a> {
    values: &'a [f32],
    iso_f: f32,
    corner_offset: [usize; 8],
    edge_offset: [usize; 12],
    face_edges: [[usize; 4]; 6],
}

impl CellBuilder<'_> {
    /// A cell with an ambiguous face: its contour loops chained from per-face segments.
    fn add_cell_loops(
        &self,
        cell: Cell,
        edge_vertex: &[i32],
        vertices: &[f64],
        triangles: &mut Vec<i32>,
        centres: &mut Vec<f64>,
    ) {
        let inside = |corner: usize| cell.cube_index & (1 << corner) != 0;
        let mut next = [-1i32; 12];
        for face in 0..6 {
            let corners = FACE_CORNERS[face];
            // Seen from outside, going round the face: an exit edge runs inside -> outside, an
            // entry edge outside -> inside. A segment runs from an exit to an entry.
            let crossings = (0..4)
                .filter(|&i| inside(corners[i]) != inside(corners[(i + 1) % 4]))
                .count();
            if crossings == 0 {
                continue;
            }

            let mut join = false;
            if crossings == 4 {
                // Asymptotic decider: the bilinear saddle (ac - bd) / (a + c - b - d) is inside
                // when the inside corners connect. a*c and b*d are the same doubles whichever
                // corner starts the face, so both cells sharing it reach the same answer.
                let value = |n: usize| {
                    self.values[cell.node + self.corner_offset[corners[n]]] as f64
                        - self.iso_f as f64
                };
                let (a, b, c, d) = (value(0), value(1), value(2), value(3));
                let (ac, bd) = (a * c, b * d);
                join = if a < 0.0 { ac > bd } else { ac < bd };
            }

            for i in 0..4 {
                if !inside(corners[i]) || inside(corners[(i + 1) % 4]) {
                    continue;
                }
                // An exit. Separated inside corners: the entry just before it closes its corner
                // off. Joined: the next entry round the face.
                let step = if join { 1 } else { 3 };
                let mut entry = (i + step) % 4;
                while inside(corners[entry]) == inside(corners[(entry + 1) % 4]) {
                    entry = (entry + step) % 4;
                }
                next[self.face_edges[face][i]] = self.face_edges[face][entry] as i32;
            }
        }

        // Loops run with the inside on their left seen from outside each face, which is
        // clockwise seen from the outside of the surface: wind the other way, as g3 does.
        let mut add_triangle =
            |v0: i32, v1: i32, v2: i32| triangles.extend_from_slice(&[v0, v2, v1]);

        let mut visited = 0u32;
        for start in 0..12 {
            if next[start] < 0 || visited & (1 << start) != 0 {
                continue;
            }
            let mut ring = Vec::with_capacity(12);
            let mut e = start;
            while visited & (1 << e) == 0 {
                visited |= 1 << e;
                ring.push(edge_vertex[cell.key + self.edge_offset[e]]);
                e = next[e] as usize;
            }

            let count = ring.len();
            if count <= 4 {
                for n in 1..count.saturating_sub(1) {
                    add_triangle(ring[0], ring[n], ring[n + 1]);
                }
            } else {
                let (mut x, mut y, mut z) = (0.0, 0.0, 0.0);
                for &v in &ring {
                    let v = v as usize * 3;
                    x += vertices[v];
                    y += vertices[v + 1];
                    z += vertices[v + 2];
                }
                centres.extend_from_slice(&[x / count as f64, y / count as f64, z / count as f64]);
                let centre = -((centres.len() / 3) as i32);
                for n in 0..count {
                    add_triangle(centre, ring[n], ring[(n + 1) % count]);
                }
            }
        }
    }
}
