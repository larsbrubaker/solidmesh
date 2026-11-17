//! Quadric error (derived from geometry3Sharp): the quadratic function that measures the
//! squared distance to a plane (or a weighted sum of planes), in Garland's minimal 10-coefficient
//! form (<http://mgarland.org/files/papers/qtheory.pdf>) - symmetric matrix A, vector b,
//! constant c. The `Reducer` (`reducer.rs`) keeps one per vertex and scores each edge collapse
//! with their sum, placing the kept vertex at [`QuadricError::minimizer_near`]: A's eigenvectors in
//! closed form (the characteristic cubic's trigonometric roots, a cross product, and the 2x2
//! problem across it), a solve only along the directions whose stiffness is a
//! set fraction of the largest, and the reference point kept along the rest. The test is relative,
//! so the placement is the same whatever units the part is in. The operation order is fixed, so
//! reduce results are reproducible to the bit.

use super::vector3d::dot;

/// A quadric: symmetric matrix A, vector b and constant c.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct QuadricError {
    pub(crate) axx: f64,
    pub(crate) axy: f64,
    pub(crate) axz: f64,
    pub(crate) ayy: f64,
    pub(crate) ayz: f64,
    pub(crate) azz: f64,
    pub(crate) bx: f64,
    pub(crate) by: f64,
    pub(crate) bz: f64,
    pub(crate) c: f64,
}

impl QuadricError {
    /// The zero quadric.
    pub(crate) const ZERO: QuadricError = QuadricError {
        axx: 0.0,
        axy: 0.0,
        axz: 0.0,
        ayy: 0.0,
        ayz: 0.0,
        azz: 0.0,
        bx: 0.0,
        by: 0.0,
        bz: 0.0,
        c: 0.0,
    };

    /// The plane through `p` with normal `n`.
    pub(crate) fn from_plane(n: [f64; 3], p: [f64; 3]) -> Self {
        let mut q = QuadricError {
            axx: n[0] * n[0],
            axy: n[0] * n[1],
            axz: n[0] * n[2],
            ayy: n[1] * n[1],
            ayz: n[1] * n[2],
            azz: n[2] * n[2],
            ..Self::ZERO
        };
        let v = q.multiply_a(p);
        q.bx = -v[0];
        q.by = -v[1];
        q.bz = -v[2];
        q.c = dot(p, v);
        q
    }

    /// The sum of two quadrics.
    pub(crate) fn sum(a: &QuadricError, b: &QuadricError) -> Self {
        QuadricError {
            axx: a.axx + b.axx,
            axy: a.axy + b.axy,
            axz: a.axz + b.axz,
            ayy: a.ayy + b.ayy,
            ayz: a.ayz + b.ayz,
            azz: a.azz + b.azz,
            bx: a.bx + b.bx,
            by: a.by + b.by,
            bz: a.bz + b.bz,
            c: a.c + b.c,
        }
    }

    /// Adds `w * b`.
    pub(crate) fn add(&mut self, w: f64, b: &QuadricError) {
        self.axx += w * b.axx;
        self.axy += w * b.axy;
        self.axz += w * b.axz;
        self.ayy += w * b.ayy;
        self.ayz += w * b.ayz;
        self.azz += w * b.azz;
        self.bx += w * b.bx;
        self.by += w * b.by;
        self.bz += w * b.bz;
        self.c += w * b.c;
    }

    /// The quadric's value at `pt`: `pAp + 2*dot(p,b) + c`.
    pub(crate) fn evaluate(&self, pt: [f64; 3]) -> f64 {
        let x = self.axx * pt[0] + self.axy * pt[1] + self.axz * pt[2];
        let y = self.axy * pt[0] + self.ayy * pt[1] + self.ayz * pt[2];
        let z = self.axz * pt[0] + self.ayz * pt[1] + self.azz * pt[2];
        (pt[0] * x + pt[1] * y + pt[2] * z)
            + 2.0 * (pt[0] * self.bx + pt[1] * self.by + pt[2] * self.bz)
            + self.c
    }

    /// `A * pt`.
    fn multiply_a(&self, pt: [f64; 3]) -> [f64; 3] {
        let x = self.axx * pt[0] + self.axy * pt[1] + self.axz * pt[2];
        let y = self.axy * pt[0] + self.ayy * pt[1] + self.ayz * pt[2];
        let z = self.axz * pt[0] + self.ayz * pt[1] + self.azz * pt[2];
        [x, y, z]
    }

    /// The point minimizing the quadric that lies nearest `reference`, solving only in the
    /// directions the quadric is stiff in. A is split into its eigenvectors; a direction whose
    /// eigenvalue is below `RELATIVE_STIFFNESS` times the largest is one the planes barely
    /// constrain (along a flat face or a straight crease), so the point keeps `reference`'s
    /// position along it instead of running off to where rounding puts the exact minimum.
    /// The test is relative, so a part measured in metres solves exactly as the same part in
    /// millimetres does. Returns `reference` when the quadric constrains nothing.
    pub(crate) fn minimizer_near(&self, reference: [f64; 3]) -> [f64; 3] {
        let (values, vectors) = self.eigen();
        let largest = values[0].max(values[1]).max(values[2]);
        if largest <= 0.0 || !largest.is_finite() {
            return reference;
        }
        // The gradient's half at the reference: A r + b.
        let ar = self.multiply_a(reference);
        let g = [ar[0] + self.bx, ar[1] + self.by, ar[2] + self.bz];
        let mut x = reference;
        for axis in 0..3 {
            if values[axis] > RELATIVE_STIFFNESS * largest {
                let e = vectors[axis];
                let step = -dot(e, g) / values[axis];
                x = [x[0] + step * e[0], x[1] + step * e[1], x[2] + step * e[2]];
            }
        }
        x
    }

    /// A's eigenvalues (largest first) and unit eigenvectors, solved in closed form: the
    /// eigenvalues by the trigonometric solution of the characteristic cubic, the eigenvector
    /// of whichever end eigenvalue stands farther from the middle one by the longest cross
    /// product of two rows of `A - λI`, and the other two by the 2x2 problem in the plane
    /// across it. When all three eigenvalues agree, the axes are the eigenvectors. Every step
    /// is a fixed sequence of `f64` operations, so the result is the same on every machine.
    fn eigen(&self) -> ([f64; 3], [[f64; 3]; 3]) {
        let (a, b, c) = (self.axx, self.ayy, self.azz);
        let (d, e, f) = (self.axy, self.axz, self.ayz);
        let off = d * d + e * e + f * f;
        let q = (a + b + c) / 3.0;
        let (da, db, dc) = (a - q, b - q, c - q);
        let p2 = da * da + db * db + dc * dc + 2.0 * off;
        let scale = a
            .abs()
            .max(b.abs())
            .max(c.abs())
            .max(d.abs())
            .max(e.abs())
            .max(f.abs());
        let axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        if p2.is_nan() || p2 <= EQUAL_EIGEN * EQUAL_EIGEN * scale * scale {
            // A is a multiple of the identity (or zero): every direction is an eigenvector.
            return ([a, b, c], axes);
        }
        let p = (p2 / 6.0).sqrt();
        // det((A - qI) / p) / 2, clamped against rounding to the cosine's range.
        let det = da * (db * dc - f * f) - d * (d * dc - f * e) + e * (d * f - db * e);
        let r = (det / (p * p * p) / 2.0).clamp(-1.0, 1.0);
        let phi = libm::acos(r) / 3.0;
        let high = q + 2.0 * p * libm::cos(phi);
        let low = q + 2.0 * p * libm::cos(phi + 2.0 * std::f64::consts::PI / 3.0);
        let mid = 3.0 * q - high - low;
        let rows = [[a, d, e], [d, b, f], [e, f, c]];
        // The end eigenvalue farther from the middle one has the better-conditioned
        // eigenvector; the other two come from the plane across it.
        let (lone, lone_is_high) = if high - mid >= mid - low {
            (high, true)
        } else {
            (low, false)
        };
        let v = null_vector(&rows, lone);
        let (u, w) = plane_basis(v);
        // The 2x2 problem in the plane: [[uAu, uAw], [uAw, wAw]].
        let au = self.multiply_a(u);
        let aw = self.multiply_a(w);
        let (m11, m12, m22) = (dot(u, au), dot(u, aw), dot(w, aw));
        let (big, small, along_big) = eigen_2x2(m11, m12, m22);
        let big_v = [
            along_big[0] * u[0] + along_big[1] * w[0],
            along_big[0] * u[1] + along_big[1] * w[1],
            along_big[0] * u[2] + along_big[1] * w[2],
        ];
        let small_v = [
            -along_big[1] * u[0] + along_big[0] * w[0],
            -along_big[1] * u[1] + along_big[0] * w[1],
            -along_big[1] * u[2] + along_big[0] * w[2],
        ];
        if lone_is_high {
            ([lone, big, small], [v, big_v, small_v])
        } else {
            ([big, small, lone], [big_v, small_v, v])
        }
    }
}

/// A unit vector `v` with `(rows - λI) v = 0`: the longest cross product of two rows of
/// `rows - λI` (the first of equal ones), or the x axis when every row pair is parallel.
fn null_vector(rows: &[[f64; 3]; 3], lambda: f64) -> [f64; 3] {
    let m = [
        [rows[0][0] - lambda, rows[0][1], rows[0][2]],
        [rows[1][0], rows[1][1] - lambda, rows[1][2]],
        [rows[2][0], rows[2][1], rows[2][2] - lambda],
    ];
    let mut best = [1.0, 0.0, 0.0];
    let mut best_len = 0.0;
    for (i, j) in [(0usize, 1usize), (0, 2), (1, 2)] {
        let c = cross3(m[i], m[j]);
        let len = dot(c, c);
        if len > best_len {
            best_len = len;
            best = c;
        }
    }
    if best_len > 0.0 {
        let inv = 1.0 / best_len.sqrt();
        [best[0] * inv, best[1] * inv, best[2] * inv]
    } else {
        [1.0, 0.0, 0.0]
    }
}

/// Two unit vectors that with unit `v` make a right-handed orthonormal basis, built from the
/// axis least aligned with `v` (the first of equals).
fn plane_basis(v: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    let (x, y, z) = (v[0].abs(), v[1].abs(), v[2].abs());
    let axis = if x <= y && x <= z {
        [1.0, 0.0, 0.0]
    } else if y <= z {
        [0.0, 1.0, 0.0]
    } else {
        [0.0, 0.0, 1.0]
    };
    let u = cross3(v, axis);
    let inv = 1.0 / dot(u, u).sqrt();
    let u = [u[0] * inv, u[1] * inv, u[2] * inv];
    (u, cross3(v, u))
}

/// The eigenvalues (larger first) of the symmetric 2x2 matrix `[[a, b], [b, c]]` and the unit
/// eigenvector of the larger one (the x axis, or the y axis when `c > a`, if `b` is zero).
fn eigen_2x2(a: f64, b: f64, c: f64) -> (f64, f64, [f64; 2]) {
    let half_diff = (a - c) / 2.0;
    let radius = (half_diff * half_diff + b * b).sqrt();
    let mean = (a + c) / 2.0;
    let (big, small) = (mean + radius, mean - radius);
    if b == 0.0 {
        return if a >= c {
            (a, c, [1.0, 0.0])
        } else {
            (c, a, [0.0, 1.0])
        };
    }
    // Of (b, big - a) and (big - c, b), the longer is the better-conditioned direction.
    let first = [b, big - a];
    let second = [big - c, b];
    let v = if first[0] * first[0] + first[1] * first[1]
        >= second[0] * second[0] + second[1] * second[1]
    {
        first
    } else {
        second
    };
    let inv = 1.0 / (v[0] * v[0] + v[1] * v[1]).sqrt();
    (big, small, [v[0] * inv, v[1] * inv])
}

fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// An eigen-direction counts as constrained when its eigenvalue is at least this fraction of
/// the largest: below it the planes are so nearly parallel along it that the exact minimum
/// is decided by rounding, not by the surface.
const RELATIVE_STIFFNESS: f64 = 1e-3;
/// The spread of A's eigenvalues, relative to its largest entry, below which they count as
/// one (A a multiple of the identity) and the axes serve as eigenvectors.
const EQUAL_EIGEN: f64 = 1e-12;
