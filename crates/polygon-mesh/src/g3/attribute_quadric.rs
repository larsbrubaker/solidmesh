//! The texture-aware quadric the reducer (`reducer.rs`, through `reducer_seams.rs`) scores
//! collapses inside a textured region with: Garland and Heckbert's generalized quadric over
//! points `(x, y, z, λu, λv)`. Each triangle, lifted into those five dimensions with its corner
//! UVs, spans a 2-plane; the quadric measures the squared distance to it. A point that keeps the
//! surface but slides its UV off the picture's map is as far from the plane as one that leaves the
//! surface, so a collapse that would smear the picture costs what it does to the picture. `λ`
//! (`reducer_face_data.rs`) scales UV units to model units for the region, so an undistorted map
//! weighs texture and position alike.
//!
//! Stored in symmetric form - the 15 entries of `A` on and above the diagonal, vector `b` and
//! constant `c`, 21 values with `Q(x) = xAx + 2 b.x + c` - so a vertex's quadric costs 168
//! bytes. Every sum runs in index order and the solve pivots by largest magnitude with ties to
//! the lower row, so a reduce gives the same mesh on every run.

/// A point in the lifted space: position, then `λ` times the UV.
pub(crate) type Point5 = [f64; 5];

/// A five-dimensional quadric: symmetric `A`, vector `b`, constant `c`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct AttributeQuadric {
    /// `A`'s entries on and above the diagonal, row by row ([`upper`]).
    a: [f64; 15],
    b: [f64; 5],
    c: f64,
}

/// Where entry `(i, j)` of the symmetric matrix lives in [`AttributeQuadric::a`].
fn upper(i: usize, j: usize) -> usize {
    let (r, c) = if i <= j { (i, j) } else { (j, i) };
    // Rows 0..r hold 5 + 4 + ... entries: r * 5 - r * (r - 1) / 2.
    r * 5 - r * (r.saturating_sub(1)) / 2 + (c - r)
}

fn dot5(x: &Point5, y: &Point5) -> f64 {
    let mut sum = 0.0;
    for k in 0..5 {
        sum += x[k] * y[k];
    }
    sum
}

fn sub5(x: &Point5, y: &Point5) -> Point5 {
    let mut out = [0.0; 5];
    for k in 0..5 {
        out[k] = x[k] - y[k];
    }
    out
}

impl AttributeQuadric {
    /// The zero quadric.
    pub(crate) const ZERO: AttributeQuadric = AttributeQuadric {
        a: [0.0; 15],
        b: [0.0; 5],
        c: 0.0,
    };

    /// The squared distance to the plane through the lifted triangle `p0 p1 p2`, times `weight`;
    /// zero for a triangle that spans no plane.
    pub(crate) fn from_triangle(p0: &Point5, p1: &Point5, p2: &Point5, weight: f64) -> Self {
        let d1 = sub5(p1, p0);
        let len1 = dot5(&d1, &d1).sqrt();
        if len1 <= f64::EPSILON {
            return Self::ZERO;
        }
        let e1 = d1.map(|x| x / len1);
        let d2 = sub5(p2, p0);
        let along = dot5(&e1, &d2);
        let mut w = [0.0; 5];
        for k in 0..5 {
            w[k] = d2[k] - along * e1[k];
        }
        let len2 = dot5(&w, &w).sqrt();
        if len2 <= f64::EPSILON {
            return Self::ZERO;
        }
        let e2 = w.map(|x| x / len2);
        let mut q = Self::ZERO;
        for i in 0..5 {
            for j in i..5 {
                let identity = if i == j { 1.0 } else { 0.0 };
                q.a[upper(i, j)] = weight * (identity - e1[i] * e1[j] - e2[i] * e2[j]);
            }
        }
        let p_e1 = dot5(p0, &e1);
        let p_e2 = dot5(p0, &e2);
        for k in 0..5 {
            q.b[k] = weight * ((p_e1 * e1[k] + p_e2 * e2[k]) - p0[k]);
        }
        q.c = weight * ((dot5(p0, p0) - p_e1 * p_e1) - p_e2 * p_e2);
        q
    }

    /// Adds `other`.
    pub(crate) fn add(&mut self, other: &AttributeQuadric) {
        for (a, o) in self.a.iter_mut().zip(other.a) {
            *a += o;
        }
        for (b, o) in self.b.iter_mut().zip(other.b) {
            *b += o;
        }
        self.c += other.c;
    }

    /// The sum of two quadrics.
    pub(crate) fn sum(x: &AttributeQuadric, y: &AttributeQuadric) -> Self {
        let mut out = *x;
        out.add(y);
        out
    }

    /// The full matrix `A`.
    fn full(&self) -> [[f64; 5]; 5] {
        let mut m = [[0.0; 5]; 5];
        for (i, row) in m.iter_mut().enumerate() {
            for (j, value) in row.iter_mut().enumerate() {
                *value = self.a[upper(i, j)];
            }
        }
        m
    }

    /// `Q(p) = pAp + 2 b.p + c`.
    pub(crate) fn evaluate(&self, p: &Point5) -> f64 {
        let mut quadratic = 0.0;
        for (a_row, &p_i) in self.full().iter().zip(p) {
            let mut row = 0.0;
            for (a, x) in a_row.iter().zip(p) {
                row += a * x;
            }
            quadratic += p_i * row;
        }
        (quadratic + 2.0 * dot5(&self.b, p)) + self.c
    }

    /// The point minimizing the quadric (`A x = -b`), or `None` when `A` is (nearly) singular:
    /// Gaussian elimination with partial pivoting, largest magnitude first, ties to the lower row.
    pub(crate) fn optimal_point(&self) -> Option<Point5> {
        let mut m = self.full();
        let mut rhs = self.b.map(|x| -x);
        let mut scale = 0.0f64;
        for row in &m {
            for &x in row {
                scale = scale.max(x.abs());
            }
        }
        if scale.is_nan() || scale == 0.0 {
            return None;
        }
        let tolerance = scale * 1e-12;
        for col in 0..5 {
            let mut pivot = col;
            for row in col + 1..5 {
                if m[row][col].abs() > m[pivot][col].abs() {
                    pivot = row;
                }
            }
            if m[pivot][col].abs() <= tolerance || m[pivot][col].is_nan() {
                return None;
            }
            m.swap(col, pivot);
            rhs.swap(col, pivot);
            for row in col + 1..5 {
                let factor = m[row][col] / m[col][col];
                let pivot_row = m[col];
                for (value, pivot_value) in m[row].iter_mut().zip(pivot_row).skip(col) {
                    *value -= factor * pivot_value;
                }
                rhs[row] -= factor * rhs[col];
            }
        }
        let mut x = [0.0; 5];
        for row in (0..5).rev() {
            let mut sum = rhs[row];
            for k in row + 1..5 {
                sum -= m[row][k] * x[k];
            }
            x[row] = sum / m[row][row];
        }
        if x.iter().all(|v| v.is_finite()) {
            Some(x)
        } else {
            None
        }
    }
}
