//! Quadric error (derived from geometry3Sharp): the quadratic function that measures the
//! squared distance to a plane (or a weighted sum of planes), in Garland's minimal 10-coefficient
//! form (http://mgarland.org/files/papers/qtheory.pdf) - symmetric matrix A, vector b,
//! constant c. The `Reducer` (`reducer.rs`) keeps one per vertex and scores each edge collapse
//! with their sum. The operation order is fixed, so reduce results are reproducible to the bit.

use super::vector3d::{dot, EPSILON};

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

    /// The point minimizing the quadric, or `None` when A is (nearly)
    /// singular.
    pub(crate) fn optimal_point(&self) -> Option<[f64; 3]> {
        let mut a11 = self.azz * self.ayy - self.ayz * self.ayz;
        let mut a12 = self.axz * self.ayz - self.azz * self.axy;
        let mut a13 = self.axy * self.ayz - self.axz * self.ayy;
        let mut a22 = self.azz * self.axx - self.axz * self.axz;
        let mut a23 = self.axy * self.axz - self.axx * self.ayz;
        let mut a33 = self.axx * self.ayy - self.axy * self.axy;
        let mut det = (self.axx * a11) + (self.axy * a12) + (self.axz * a13);
        // [RMS] not sure what we should be using for this threshold...have seen
        //  det less than 10^-9 on "normal" meshes.
        if det.abs() > 1000.0 * EPSILON {
            det = 1.0 / det;
            a11 *= det;
            a12 *= det;
            a13 *= det;
            a22 *= det;
            a23 *= det;
            a33 *= det;
            let x = a11 * self.bx + a12 * self.by + a13 * self.bz;
            let y = a12 * self.bx + a22 * self.by + a23 * self.bz;
            let z = a13 * self.bx + a23 * self.by + a33 * self.bz;
            Some([-x, -y, -z])
        } else {
            None
        }
    }
}
