//! The fairing step of the hole fill: places the interior vertices by least squares on the uniform
//! Laplacian (the discrete thin plate, whose normal equations are the bi-Laplacian), with the loop
//! pinned.
//!
//! The tangent condition takes the place of Botsch and Kobbelt's pinned second ring. At each loop
//! point the surface across the loop is taken as the mirror image of the fill's own one-ring in the
//! plane normal to the cross-tangent t; the Laplacian of that mirrored ring is (I - t t^T) times
//! the fill's one-sided Laplacian, which is linear in the unknowns. Driving it to zero puts the
//! fill's one-ring, on average, in the plane spanned by the loop and t - the surrounding surface's
//! tangent plane - without fixing how far into the hole the ring reaches. The price: the pinned
//! ring is held flat to second order, so a curved surround (a fillet) is met with its tangent but a
//! little flatter than a rolling ball; the cube-corner test measures how much.

use std::collections::BTreeSet;

use solidmesh_math::Vector3;

use super::{HoleFillTriangle, LevelLineWork, WorkError};

/// A 3x3 matrix, row-major.
type Matrix3 = [[f64; 3]; 3];

/// The 3x3 identity.
const IDENTITY: Matrix3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// Moves every vertex from index `loop_count` on; the loop's own points stay. `Ok(false)` when the
/// system is singular.
pub fn fair(
    vertices: &mut [Vector3],
    triangles: &[HoleFillTriangle],
    loop_count: usize,
    cross_tangents: &[Vector3],
) -> Result<bool, WorkError> {
    let unknowns = vertices.len() - loop_count;
    if unknowns == 0 {
        return Ok(true);
    }

    // Each one-ring read in ascending vertex order, so the system's rows are the same every run.
    let mut neighbours: Vec<BTreeSet<usize>> = vec![BTreeSet::new(); vertices.len()];
    for t in triangles {
        let (a, b, c) = (t.a as usize, t.b as usize, t.c as usize);
        neighbours[a].insert(b);
        neighbours[a].insert(c);
        neighbours[b].insert(a);
        neighbours[b].insert(c);
        neighbours[c].insert(a);
        neighbours[c].insert(b);
    }

    let size = 3 * unknowns;
    let mut matrix = vec![0.0; size * size];
    let mut rhs = vec![0.0; size];
    let mut terms: Vec<(usize, f64)> = Vec::new();
    for v in 0..vertices.len() {
        // residual row: Q (mean of neighbours - vertex); Q is the identity inside, the mirror
        // projector on the loop
        let q = if v < loop_count {
            projector(cross_tangents[v])
        } else {
            IDENTITY
        };
        terms.clear();
        terms.push((v, -1.0));
        for &w in &neighbours[v] {
            terms.push((w, 1.0 / neighbours[v].len() as f64));
        }

        let mut fixed_part = Vector3::ZERO;
        for &(vertex, weight) in &terms {
            if vertex < loop_count {
                fixed_part += vertices[vertex] * weight;
            }
        }

        let q_fixed = apply(&q, fixed_part);
        for &(row, row_weight) in &terms {
            if row < loop_count {
                continue;
            }

            let r = 3 * (row - loop_count);
            rhs[r] -= row_weight * q_fixed.x;
            rhs[r + 1] -= row_weight * q_fixed.y;
            rhs[r + 2] -= row_weight * q_fixed.z;
            for &(column, column_weight) in &terms {
                if column < loop_count {
                    continue;
                }

                let c = 3 * (column - loop_count);
                let scale = row_weight * column_weight;
                for (i, q_row) in q.iter().enumerate() {
                    for (j, &q_value) in q_row.iter().enumerate() {
                        matrix[((r + i) * size) + c + j] += scale * q_value;
                    }
                }
            }
        }
    }

    if !solve_symmetric_positive_definite(&mut matrix, &mut rhs)? {
        return Ok(false);
    }

    for u in 0..unknowns {
        vertices[loop_count + u] = Vector3::new(rhs[3 * u], rhs[(3 * u) + 1], rhs[(3 * u) + 2]);
    }

    Ok(true)
}

/// I - t t^T: the projector that drops the cross-tangent component. Being a projector, Q^T Q = Q.
pub fn projector(t: Vector3) -> Matrix3 {
    let t = t.get_normal();
    let tt = [t.x, t.y, t.z];
    let mut q = [[0.0; 3]; 3];
    for (i, row) in q.iter_mut().enumerate() {
        for (j, value) in row.iter_mut().enumerate() {
            *value = (if i == j { 1.0 } else { 0.0 }) - tt[i] * tt[j];
        }
    }

    q
}

/// `q v`.
pub fn apply(q: &Matrix3, v: Vector3) -> Vector3 {
    Vector3::new(
        q[0][0] * v.x + q[0][1] * v.y + q[0][2] * v.z,
        q[1][0] * v.x + q[1][1] * v.y + q[1][2] * v.z,
        q[2][0] * v.x + q[2][1] * v.y + q[2][2] * v.z,
    )
}

/// Dense Cholesky, in place on the row-major `a` (`rhs.len()` square); the solution replaces
/// `rhs`. `Ok(false)` when the matrix is not positive definite. Holes are tens to low hundreds of
/// vertices, so a dense factor is small next to the rest of a bevel.
pub fn solve_symmetric_positive_definite(
    a: &mut [f64],
    rhs: &mut [f64],
) -> Result<bool, WorkError> {
    let size = rhs.len();
    let at = |i: usize, j: usize| (i * size) + j;
    for j in 0..size {
        // Uncharged cubic work; a cancel still has to reach it.
        LevelLineWork::poll_cancel()?;
        let mut diagonal = a[at(j, j)];
        for k in 0..j {
            diagonal -= a[at(j, k)] * a[at(j, k)];
        }

        // A NaN diagonal reads as singular too, so a broken system is never solved.
        if diagonal.is_nan() || diagonal <= 1e-14 {
            return Ok(false);
        }

        diagonal = diagonal.sqrt();
        a[at(j, j)] = diagonal;
        for i in j + 1..size {
            let mut sum = a[at(i, j)];
            for k in 0..j {
                sum -= a[at(i, k)] * a[at(j, k)];
            }

            a[at(i, j)] = sum / diagonal;
        }
    }

    for i in 0..size {
        let mut sum = rhs[i];
        for k in 0..i {
            sum -= a[at(i, k)] * rhs[k];
        }

        rhs[i] = sum / a[at(i, i)];
    }

    for i in (0..size).rev() {
        let mut sum = rhs[i];
        for k in i + 1..size {
            sum -= a[at(k, i)] * rhs[k];
        }

        rhs[i] = sum / a[at(i, i)];
    }

    Ok(true)
}
