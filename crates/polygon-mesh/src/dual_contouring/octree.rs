//! Octree dual contouring, written clean-room from the published algorithm (Ju, Losasso, Schaefer
//! and Warren, "Dual Contouring of Hermite Data", SIGGRAPH 2002). No LGPL implementation was read;
//! the public shape is two calls, `build_octree(f, min, size, level, threshold)` then
//! `generate_mesh_from_octree(root)`.
//!
//! Building: the field is sampled once on the `2^level` lattice of the box. Each finest cell whose
//! corners change sign becomes a leaf holding a QEF (`qef`) of its Hermite data - the root on
//! each sign-changing edge, found by bisection, and the field's gradient there - and the vertex
//! the QEF places. Going up, eight children that are all leaves or empty collapse into one leaf
//! when the merged QEF's error is at most `threshold` and the collapse is topologically safe
//! (the paper's simplification, section 4): every edge-midpoint, face-centre and centre sample of
//! the coarse cell agrees in sign with at least one corner of that edge, face or cell, and the
//! coarse cell's inside corners and outside corners are each edge-connected, so one vertex can
//! stand for its whole patch of surface.
//!
//! Contouring (`octree_contour`) walks the tree with the paper's cell/face/edge recursion and
//! makes one quad per minimal sign-changing edge from the vertices of the cells around it.

use solidmesh_math::Vector3;

use super::qef::{QefData, QefSolver};

/// The SVD settings every QEF here is solved with.
const SVD_TOL: f64 = 1e-6;
const SVD_SWEEPS: i32 = 4;
const PINV_TOL: f64 = 1e-6;

/// Bisection steps converging on an edge's root.
const ROOT_STEPS: i32 = 32;

/// A node of the dual contouring octree.
#[derive(Debug, Clone)]
pub struct OctreeNode {
    /// The cell's lowest corner.
    pub(crate) min: Vector3,
    /// The cell's extent on each axis.
    pub(crate) size: Vector3,
    /// Depth below the root (the root is 0).
    pub(crate) depth: i32,
    /// Bit `c` set when corner `c` (`x | y << 1 | z << 2`) is inside (field below zero).
    pub(crate) signs: u8,
    pub(crate) kind: OctreeNodeKind,
}

/// What a node holds.
#[derive(Debug, Clone)]
pub(crate) enum OctreeNodeKind {
    /// No surface: every corner has the same sign.
    Empty,
    /// One surface vertex standing for the cell (a finest cell or a collapsed one).
    Leaf { vertex: Vector3, qef: QefData },
    /// Eight children, indexed like the corners.
    Internal(std::boxed::Box<[OctreeNode; 8]>),
}

impl OctreeNode {
    /// The cell's lowest corner.
    pub fn min(&self) -> Vector3 {
        self.min
    }

    /// The cell's extent on each axis.
    pub fn size(&self) -> Vector3 {
        self.size
    }

    /// The surface vertex, for a leaf.
    pub fn vertex(&self) -> Option<Vector3> {
        match &self.kind {
            OctreeNodeKind::Leaf { vertex, .. } => Some(*vertex),
            _ => None,
        }
    }

    /// The children, for an internal node.
    pub fn children(&self) -> Option<&[OctreeNode; 8]> {
        match &self.kind {
            OctreeNodeKind::Internal(children) => Some(children),
            _ => None,
        }
    }
}

/// The field sampled on the finest lattice.
struct Lattice<'f> {
    f: &'f dyn Fn(Vector3) -> f64,
    min: Vector3,
    /// The finest cell's extent.
    step: Vector3,
    /// Lattice points per axis (`2^level + 1`).
    n: usize,
    values: Vec<f64>,
}

impl Lattice<'_> {
    fn position(&self, i: usize, j: usize, k: usize) -> Vector3 {
        Vector3::new(
            self.min.x + self.step.x * i as f64,
            self.min.y + self.step.y * j as f64,
            self.min.z + self.step.z * k as f64,
        )
    }

    fn value(&self, i: usize, j: usize, k: usize) -> f64 {
        self.values[i + self.n * (j + self.n * k)]
    }

    /// The root of the field on the segment `a`-`b` (inside at exactly one end), by bisection
    /// then a final linear step.
    fn edge_root(&self, mut a: Vector3, mut fa: f64, mut b: Vector3, mut fb: f64) -> Vector3 {
        for _ in 0..ROOT_STEPS {
            let mid = (a + b) * 0.5;
            let fm = (self.f)(mid);
            if (fm < 0.0) == (fa < 0.0) {
                a = mid;
                fa = fm;
            } else {
                b = mid;
                fb = fm;
            }
        }
        let t = if fb != fa { fa / (fa - fb) } else { 0.5 };
        let t = t.clamp(0.0, 1.0);
        a + (b - a) * t
    }

    /// The field's gradient at `p` by central differences, or `fallback` where it vanishes.
    fn normal(&self, p: Vector3, fallback: Vector3) -> Vector3 {
        let h = self.step.x.min(self.step.y).min(self.step.z) * 1e-4;
        let d = |axis: Vector3| (self.f)(p + axis * h) - (self.f)(p - axis * h);
        let g = Vector3::new(
            d(Vector3::new(1.0, 0.0, 0.0)),
            d(Vector3::new(0.0, 1.0, 0.0)),
            d(Vector3::new(0.0, 0.0, 1.0)),
        );
        if g.length_squared() > 0.0 && g.length_squared().is_finite() {
            g
        } else {
            fallback
        }
    }
}

/// The two corners of each cell edge, lower corner first.
pub(crate) const CELL_EDGES: [[usize; 2]; 12] = [
    [0, 1],
    [2, 3],
    [4, 5],
    [6, 7],
    [0, 2],
    [1, 3],
    [4, 6],
    [5, 7],
    [0, 4],
    [1, 5],
    [2, 6],
    [3, 7],
];

fn corner_bits(c: usize) -> [usize; 3] {
    [c & 1, (c >> 1) & 1, (c >> 2) & 1]
}

/// The octree of the surface `f = 0` (negative inside) over the box at `min` of extent `size`, with
/// finest cells `size / 2^level`, simplified wherever a merged vertex fits its planes within
/// `threshold` (summed squared distance).
pub fn build_octree(
    f: &dyn Fn(Vector3) -> f64,
    min: Vector3,
    size: Vector3,
    level: i32,
    threshold: f64,
) -> OctreeNode {
    // The lattice holds (2^level + 1)^3 samples: 64 MB of doubles at level 8.
    let level = level.max(0);
    let cells = 1usize << level;
    let n = cells + 1;
    let step = size / cells as f64;
    let mut lattice = Lattice {
        f,
        min,
        step,
        n,
        values: Vec::with_capacity(n * n * n),
    };
    for k in 0..n {
        for j in 0..n {
            for i in 0..n {
                let p = lattice.position(i, j, k);
                lattice.values.push(f(p));
            }
        }
    }
    build_node(&lattice, [0, 0, 0], 0, level, threshold)
}

/// Builds the node whose lowest lattice point is `idx`, at `depth` of `max_depth`.
fn build_node(
    lattice: &Lattice,
    idx: [usize; 3],
    depth: i32,
    max_depth: i32,
    threshold: f64,
) -> OctreeNode {
    let span = 1usize << (max_depth - depth);
    let min = lattice.position(idx[0], idx[1], idx[2]);
    let size = lattice.step * span as f64;
    let mut signs = 0u8;
    for c in 0..8 {
        let b = corner_bits(c);
        if lattice.value(
            idx[0] + b[0] * span,
            idx[1] + b[1] * span,
            idx[2] + b[2] * span,
        ) < 0.0
        {
            signs |= 1 << c;
        }
    }

    if depth == max_depth {
        return build_leaf(lattice, idx, min, size, depth, signs);
    }

    let half = span / 2;
    let children: [OctreeNode; 8] = std::array::from_fn(|c| {
        let b = corner_bits(c);
        build_node(
            lattice,
            [
                idx[0] + b[0] * half,
                idx[1] + b[1] * half,
                idx[2] + b[2] * half,
            ],
            depth + 1,
            max_depth,
            threshold,
        )
    });
    let node = |kind| OctreeNode {
        min,
        size,
        depth,
        signs,
        kind,
    };

    if children
        .iter()
        .any(|c| matches!(c.kind, OctreeNodeKind::Internal(_)))
    {
        return node(OctreeNodeKind::Internal(std::boxed::Box::new(children)));
    }
    if children
        .iter()
        .all(|c| matches!(c.kind, OctreeNodeKind::Empty))
    {
        // Neighbouring empty children share samples, so they all have the same sign.
        return node(OctreeNodeKind::Empty);
    }

    let mut solver = QefSolver::new();
    for child in &children {
        if let OctreeNodeKind::Leaf { qef, .. } = &child.kind {
            solver.add_data(qef);
        }
    }
    let collapsed = solver
        .solve(SVD_TOL, SVD_SWEEPS, PINV_TOL)
        .filter(|_| solver.error().is_some_and(|e| e <= threshold))
        .filter(|_| collapse_is_safe(&children, signs));
    match collapsed {
        Some(vertex) => node(OctreeNodeKind::Leaf {
            vertex: keep_in_cell(vertex, solver.mass_point(), min, size),
            qef: *solver.data(),
        }),
        None => node(OctreeNodeKind::Internal(std::boxed::Box::new(children))),
    }
}

/// A finest cell: empty, or a leaf with the QEF of its sign-changing edges.
fn build_leaf(
    lattice: &Lattice,
    idx: [usize; 3],
    min: Vector3,
    size: Vector3,
    depth: i32,
    signs: u8,
) -> OctreeNode {
    let mut node = OctreeNode {
        min,
        size,
        depth,
        signs,
        kind: OctreeNodeKind::Empty,
    };
    if signs == 0 || signs == 0xFF {
        return node;
    }
    let mut solver = QefSolver::new();
    for [c0, c1] in CELL_EDGES {
        let inside0 = signs & (1 << c0) != 0;
        if inside0 == (signs & (1 << c1) != 0) {
            continue;
        }
        let (b0, b1) = (corner_bits(c0), corner_bits(c1));
        let (i0, i1) = (
            [idx[0] + b0[0], idx[1] + b0[1], idx[2] + b0[2]],
            [idx[0] + b1[0], idx[1] + b1[1], idx[2] + b1[2]],
        );
        let p0 = lattice.position(i0[0], i0[1], i0[2]);
        let p1 = lattice.position(i1[0], i1[1], i1[2]);
        let f0 = lattice.value(i0[0], i0[1], i0[2]);
        let f1 = lattice.value(i1[0], i1[1], i1[2]);
        let p = lattice.edge_root(p0, f0, p1, f1);
        // Outward is from the inside end towards the outside end.
        let along = if inside0 { p1 - p0 } else { p0 - p1 };
        solver.add(p, lattice.normal(p, along));
    }
    if let Some(vertex) = solver.solve(SVD_TOL, SVD_SWEEPS, PINV_TOL) {
        node.kind = OctreeNodeKind::Leaf {
            vertex: keep_in_cell(vertex, solver.mass_point(), min, size),
            qef: *solver.data(),
        };
    }
    node
}

/// The QEF's point, or the mass point when the QEF's point leaves the cell (which would fold
/// the surface over its neighbours).
fn keep_in_cell(vertex: Vector3, mass_point: Vector3, min: Vector3, size: Vector3) -> Vector3 {
    let slack = 1e-9 * size.x.max(size.y).max(size.z);
    let inside =
        (0..3).all(|a| vertex[a] >= min[a] - slack && vertex[a] <= min[a] + size[a] + slack);
    if inside {
        vertex
    } else {
        mass_point
    }
}

/// The topological safety test for merging `children` into one leaf with corner `signs`.
fn collapse_is_safe(children: &[OctreeNode; 8], signs: u8) -> bool {
    // The 3x3x3 samples of the coarse cell, from the children's corners.
    let mut lattice = [false; 27];
    for (c, child) in children.iter().enumerate() {
        let cb = corner_bits(c);
        for k in 0..8 {
            let kb = corner_bits(k);
            let at = (cb[0] + kb[0]) + 3 * ((cb[1] + kb[1]) + 3 * (cb[2] + kb[2]));
            lattice[at] = child.signs & (1 << k) != 0;
        }
    }
    let coarse = |c: usize| signs & (1 << c) != 0;
    // Each midpoint sample must agree with some coarse corner of the smallest coarse feature
    // (edge, face or the cell) it lies in: the corners whose bits match it on its 0/2 axes.
    for z in 0..3 {
        for y in 0..3 {
            for x in 0..3 {
                let p = [x, y, z];
                if p.iter().all(|&v| v != 1) {
                    continue;
                }
                let sample = lattice[x + 3 * (y + 3 * z)];
                let agrees = (0..8).any(|c| {
                    let b = corner_bits(c);
                    (0..3).all(|a| p[a] == 1 || p[a] == 2 * b[a]) && coarse(c) == sample
                });
                if !agrees {
                    return false;
                }
            }
        }
    }
    connected(signs) && connected(!signs)
}

/// Whether the corners set in `mask` form one edge-connected group (an empty set counts).
fn connected(mask: u8) -> bool {
    if mask == 0 {
        return true;
    }
    let start = mask.trailing_zeros() as usize;
    let mut seen = 1u8 << start;
    let mut stack = vec![start];
    while let Some(c) = stack.pop() {
        for bit in [1usize, 2, 4] {
            let n = c ^ bit;
            if mask & (1 << n) != 0 && seen & (1 << n) == 0 {
                seen |= 1 << n;
                stack.push(n);
            }
        }
    }
    seen == mask
}
