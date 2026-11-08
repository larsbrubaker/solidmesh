//! The quadratic error function solver dual contouring places its vertices with, derived from a
//! public-domain QEF library (Unlicense): its QEF data, solver, Jacobi SVD with Givens and Schur
//! rotations, and the symmetric 3x3 matrix helpers. See `THIRD_PARTY_NOTICES.md`.
//!
//! The QEF is accumulated as AᵀA, Aᵀb and bᵀb from (point, normal) planes and solved around the
//! mass point with a Jacobi-sweep symmetric SVD and a truncated pseudo-inverse, so a rank-deficient
//! system (a flat patch, a single edge) settles on the point nearest the mass point. Only what dual
//! contouring reaches is here: no general least-squares solve, matrix products, transpose, matrix
//! norms or error over a full matrix.

use solidmesh_math::Vector3;

/// A symmetric 3x3 matrix by its upper triangle.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct SMat3 {
    m00: f64,
    m01: f64,
    m02: f64,
    m11: f64,
    m12: f64,
    m22: f64,
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct Mat3 {
    m: [[f64; 3]; 3],
}

impl Mat3 {
    const IDENTITY: Mat3 = Mat3 {
        m: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    };

    fn symmetric(a: &SMat3) -> Mat3 {
        Mat3 {
            m: [
                [a.m00, a.m01, a.m02],
                [a.m01, a.m11, a.m12],
                [a.m02, a.m12, a.m22],
            ],
        }
    }
}

/// The Frobenius norm.
fn norm(a: &SMat3) -> f64 {
    ((a.m00 * a.m00)
        + (a.m01 * a.m01)
        + (a.m02 * a.m02)
        + (a.m01 * a.m01)
        + (a.m11 * a.m11)
        + (a.m12 * a.m12)
        + (a.m02 * a.m02)
        + (a.m12 * a.m12)
        + (a.m22 * a.m22))
        .sqrt()
}

/// The norm of the off-diagonal part.
fn off(a: &SMat3) -> f64 {
    (2.0 * ((a.m01 * a.m01) + (a.m02 * a.m02) + (a.m12 * a.m12))).sqrt()
}

fn vmul(a: &Mat3, v: Vector3) -> Vector3 {
    let m = &a.m;
    Vector3::new(
        (m[0][0] * v.x) + (m[0][1] * v.y) + (m[0][2] * v.z),
        (m[1][0] * v.x) + (m[1][1] * v.y) + (m[1][2] * v.z),
        (m[2][0] * v.x) + (m[2][1] * v.y) + (m[2][2] * v.z),
    )
}

fn vmul_symmetric(a: &SMat3, v: Vector3) -> Vector3 {
    Vector3::new(
        (a.m00 * v.x) + (a.m01 * v.y) + (a.m02 * v.z),
        (a.m01 * v.x) + (a.m11 * v.y) + (a.m12 * v.z),
        (a.m02 * v.x) + (a.m12 * v.y) + (a.m22 * v.z),
    )
}

/// The Jacobi rotation zeroing `a_pq`.
fn symmetric_givens_coefficients(a_pp: f64, a_pq: f64, a_qq: f64) -> (f64, f64) {
    if a_pq == 0.0 {
        return (1.0, 0.0);
    }
    let tau = (a_qq - a_pp) / (2.0 * a_pq);
    let stt = (1.0 + tau * tau).sqrt();
    let tan = 1.0 / (if tau >= 0.0 { tau + stt } else { tau - stt });
    let c = 1.0 / (1.0 + tan * tan).sqrt();
    (c, tan * c)
}

/// One Jacobi step on the 0-1 pair: the Schur rotation of `vtav`, then the matching Givens rotation
/// of `v`.
fn rotate01(vtav: &mut SMat3, v: &mut Mat3) {
    if vtav.m01 == 0.0 {
        return;
    }
    let m = *vtav;
    let (c, s) = symmetric_givens_coefficients(m.m00, m.m01, m.m11);
    let (cc, ss, mix) = (c * c, s * s, 2.0 * c * s * m.m01);
    *vtav = SMat3 {
        m00: cc * m.m00 - mix + ss * m.m11,
        m01: 0.0,
        m02: c * m.m02 - s * m.m12,
        m11: ss * m.m00 + mix + cc * m.m11,
        m12: s * m.m02 + c * m.m12,
        m22: m.m22,
    };
    for row in &mut v.m {
        let (a, b) = (row[0], row[1]);
        row[0] = c * a - s * b;
        row[1] = s * a + c * b;
    }
}

/// One Jacobi step on the 0-2 pair: the Schur rotation of `vtav`, then the matching Givens rotation
/// of `v`.
fn rotate02(vtav: &mut SMat3, v: &mut Mat3) {
    if vtav.m02 == 0.0 {
        return;
    }
    let m = *vtav;
    let (c, s) = symmetric_givens_coefficients(m.m00, m.m02, m.m22);
    let (cc, ss, mix) = (c * c, s * s, 2.0 * c * s * m.m02);
    *vtav = SMat3 {
        m00: cc * m.m00 - mix + ss * m.m22,
        m01: c * m.m01 - s * m.m12,
        m02: 0.0,
        m11: m.m11,
        m12: s * m.m01 + c * m.m12,
        m22: ss * m.m00 + mix + cc * m.m22,
    };
    for row in &mut v.m {
        let (a, b) = (row[0], row[2]);
        row[0] = c * a - s * b;
        row[2] = s * a + c * b;
    }
}

/// One Jacobi step on the 1-2 pair: the Schur rotation of `vtav`, then the matching Givens rotation
/// of `v`.
fn rotate12(vtav: &mut SMat3, v: &mut Mat3) {
    if vtav.m12 == 0.0 {
        return;
    }
    let m = *vtav;
    let (c, s) = symmetric_givens_coefficients(m.m11, m.m12, m.m22);
    let (cc, ss, mix) = (c * c, s * s, 2.0 * c * s * m.m12);
    *vtav = SMat3 {
        m00: m.m00,
        m01: c * m.m01 - s * m.m02,
        m02: s * m.m01 + c * m.m02,
        m11: cc * m.m11 - mix + ss * m.m22,
        m12: 0.0,
        m22: ss * m.m11 + mix + cc * m.m22,
    };
    for row in &mut v.m {
        let (a, b) = (row[1], row[2]);
        row[1] = c * a - s * b;
        row[2] = s * a + c * b;
    }
}

/// Jacobi sweeps until the off-diagonal part is below `tol` of the norm.
fn symmetric_svd(a: &SMat3, tol: f64, max_sweeps: i32) -> (SMat3, Mat3) {
    let mut vtav = *a;
    let mut v = Mat3::IDENTITY;
    let delta = tol * norm(&vtav);
    let mut i = 0;
    while i < max_sweeps && off(&vtav) > delta {
        rotate01(&mut vtav, &mut v);
        rotate02(&mut vtav, &mut v);
        rotate12(&mut vtav, &mut v);
        i += 1;
    }
    (vtav, v)
}

/// The truncated reciprocal.
fn pinv(x: f64, tol: f64) -> f64 {
    if x.abs() < tol || (1.0 / x).abs() < tol {
        0.0
    } else {
        1.0 / x
    }
}

/// V D⁺ Vᵀ.
fn pseudo_inverse(d: &SMat3, v: &Mat3, tol: f64) -> Mat3 {
    let dd = [pinv(d.m00, tol), pinv(d.m11, tol), pinv(d.m22, tol)];
    let v = &v.m;
    let mut out = Mat3::default();
    for (r, row) in out.m.iter_mut().enumerate() {
        for (c, cell) in row.iter_mut().enumerate() {
            *cell =
                v[r][0] * dd[0] * v[c][0] + v[r][1] * dd[1] * v[c][1] + v[r][2] * dd[2] * v[c][2];
        }
    }
    out
}

/// X = A⁺b and the residual |b - Ax|².
fn solve_symmetric(
    a: &SMat3,
    b: Vector3,
    svd_tol: f64,
    svd_sweeps: i32,
    pinv_tol: f64,
) -> (Vector3, f64) {
    let (vtav, v) = symmetric_svd(a, svd_tol, svd_sweeps);
    let x = vmul(&pseudo_inverse(&vtav, &v, pinv_tol), b);
    let r = b - vmul(&Mat3::symmetric(a), x);
    (x, r.dot(r))
}

/// The accumulated normal equations of a set of planes.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct QefData {
    ata: SMat3,
    atb: Vector3,
    btb: f64,
    mass_point: Vector3,
    num_points: i32,
}

impl QefData {
    pub fn add(&mut self, rhs: &QefData) {
        self.ata.m00 += rhs.ata.m00;
        self.ata.m01 += rhs.ata.m01;
        self.ata.m02 += rhs.ata.m02;
        self.ata.m11 += rhs.ata.m11;
        self.ata.m12 += rhs.ata.m12;
        self.ata.m22 += rhs.ata.m22;
        self.atb += rhs.atb;
        self.btb += rhs.btb;
        self.mass_point += rhs.mass_point;
        self.num_points += rhs.num_points;
    }

    pub fn num_points(&self) -> i32 {
        self.num_points
    }
}

/// Accumulates planes and finds the point that best fits them.
#[derive(Debug, Clone, Default)]
pub struct QefSolver {
    data: QefData,
    mass_point: Vector3,
    x: Vector3,
    has_solution: bool,
}

impl QefSolver {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn data(&self) -> &QefData {
        &self.data
    }

    /// The average point, set by [`QefSolver::solve`].
    pub fn mass_point(&self) -> Vector3 {
        self.mass_point
    }

    /// The plane through `p` with normal `n` (normalized here).
    pub fn add(&mut self, p: Vector3, n: Vector3) {
        self.has_solution = false;
        let normal = n.get_normal();
        let (nx, ny, nz) = (normal.x, normal.y, normal.z);
        let d = &mut self.data;
        d.ata.m00 += nx * nx;
        d.ata.m01 += nx * ny;
        d.ata.m02 += nx * nz;
        d.ata.m11 += ny * ny;
        d.ata.m12 += ny * nz;
        d.ata.m22 += nz * nz;
        let dot = nx * p.x + ny * p.y + nz * p.z;
        d.atb += normal * dot;
        d.btb += dot * dot;
        d.mass_point += p;
        d.num_points += 1;
    }

    pub fn add_data(&mut self, rhs: &QefData) {
        self.has_solution = false;
        self.data.add(rhs);
    }

    /// The sum of squared distances from `pos` to the planes.
    pub fn error_at(&self, pos: Vector3) -> f64 {
        let atax = vmul_symmetric(&self.data.ata, pos);
        pos.dot(atax) - 2.0 * pos.dot(self.data.atb) + self.data.btb
    }

    /// The error at the last solution, `None` before one.
    pub fn error(&self) -> Option<f64> {
        self.has_solution.then(|| self.error_at(self.x))
    }

    /// The best-fit point, solved relative to the mass point; `None` with no planes.
    pub fn solve(&mut self, svd_tol: f64, svd_sweeps: i32, pinv_tol: f64) -> Option<Vector3> {
        if self.data.num_points == 0 {
            return None;
        }
        self.mass_point = self.data.mass_point / self.data.num_points as f64;
        let atb = self.data.atb - vmul_symmetric(&self.data.ata, self.mass_point);
        let (x, result) = solve_symmetric(&self.data.ata, atb, svd_tol, svd_sweeps, pinv_tol);
        self.x = if result.is_nan() {
            self.mass_point
        } else {
            x + self.mass_point
        };
        self.has_solution = true;
        Some(self.x)
    }
}
