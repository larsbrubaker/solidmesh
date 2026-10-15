//! The quickhull's math (derived from MIConvexHull), the 3D parts: face planes,
//! vertex-to-plane distance, the simplex volume (a Cayley-Menger determinant) and the LU
//! factorization it uses. The operation order is fixed, so the same input always chooses the
//! same vertices.

use super::data_model::FaceInternal;

/// Dimension of the 3D hull path.
pub(super) const DIM: usize = 3;

fn vector_between_vertices(positions: &[f64], to_index: i32, from_index: i32) -> [f64; DIM] {
    let u = to_index as usize * DIM;
    let v = from_index as usize * DIM;
    let mut target = [0.0; DIM];
    for (i, t) in target.iter_mut().enumerate() {
        *t = positions[u + i] - positions[v + i];
    }
    target
}

/// The unit normal of the triangle through three vertices.
fn find_normal_vector_3d(positions: &[f64], vertices: &[i32; 3], normal: &mut [f64; DIM]) {
    let nt_x = vector_between_vertices(positions, vertices[1], vertices[0]);
    let nt_y = vector_between_vertices(positions, vertices[2], vertices[1]);
    let nx = nt_x[1] * nt_y[2] - nt_x[2] * nt_y[1];
    let ny = nt_x[2] * nt_y[0] - nt_x[0] * nt_y[2];
    let nz = nt_x[0] * nt_y[1] - nt_x[1] * nt_y[0];
    let norm = (nx * nx + ny * ny + nz * nz).sqrt();
    let f = 1.0 / norm;
    normal[0] = f * nx;
    normal[1] = f * ny;
    normal[2] = f * nz;
}

/// The face normal (pointing away from `center`) and offset. False when
/// the face is degenerate; the NaN normal is still written.
pub(super) fn calculate_face_plane(
    positions: &[f64],
    face: &mut FaceInternal,
    center: &[f64; DIM],
) -> bool {
    let vertices = face.vertices;
    find_normal_vector_3d(positions, &vertices, &mut face.normal);
    if face.normal[0].is_nan() {
        return false;
    }

    let mut offset = 0.0;
    let mut center_distance = 0.0;
    let fi = vertices[0] as usize * DIM;
    for i in 0..DIM {
        let n = face.normal[i];
        offset += n * positions[fi + i];
        center_distance += n * center[i];
    }
    face.offset = -offset;
    center_distance -= offset;

    if center_distance > 0.0 {
        for n in face.normal.iter_mut() {
            *n = -*n;
        }
        face.offset = offset;
        face.is_normal_flipped = true;
    } else {
        face.is_normal_flipped = false;
    }
    true
}

/// Signed distance of vertex `v` from the face plane.
pub(super) fn get_vertex_distance(positions: &[f64], v: usize, f: &FaceInternal) -> f64 {
    let x = v * DIM;
    let mut distance = f.offset;
    for i in 0..DIM {
        distance += f.normal[i] * positions[x + i];
    }
    distance
}

/// From the Cayley-Menger determinant of the first `DIM + 1` vertices.
pub(super) fn volume_of_simplex(positions: &[f64], vertex_indices: &[i32]) -> f64 {
    let num_row_col = DIM + 2;
    let mut a = vec![0.0; num_row_col * num_row_col];
    for i in 1..num_row_col {
        a[i] = 1.0;
        a[i * num_row_col] = 1.0;
    }
    for i in 0..=DIM {
        for j in (i + 1)..=DIM {
            let d = vector_between_vertices(positions, vertex_indices[i], vertex_indices[j]);
            let mut distance_squared = 0.0;
            for dk in d {
                distance_squared += dk * dk;
            }
            a[(i + 1) + (j + 1) * num_row_col] = distance_squared;
            a[(j + 1) + (i + 1) * num_row_col] = distance_squared;
        }
    }
    let mut i_piv = vec![0usize; 2 + DIM];
    let mut helper = vec![0.0; 2 + DIM];
    lu_factor(&mut a, 2 + DIM, &mut i_piv, &mut helper);
    let mut det = 1.0;
    for (i, &piv) in i_piv.iter().enumerate() {
        det *= a[(2 + DIM) * i + i];
        if piv != i {
            det *= -1.0; // the determinant sign changes on row swap.
        }
    }
    // Math.Pow(2, Dimension) times the factorial loop: 2^3 * 2 * 3
    let mut denom = 2f64.powi(DIM as i32);
    let mut m = 1;
    loop {
        let check = m < DIM;
        m += 1;
        if !check {
            break;
        }
        denom *= m as f64;
    }
    det /= denom;
    if DIM.is_multiple_of(2) {
        det *= -1.0;
    }
    det
}

// Indexed loops keep the column arithmetic readable.
#[allow(clippy::needless_range_loop)]
/// In-place LU factorization with partial pivoting (column-major `data`).
fn lu_factor(data: &mut [f64], order: usize, ipiv: &mut [usize], vec_lu_colj: &mut [f64]) {
    for (i, p) in ipiv.iter_mut().enumerate().take(order) {
        *p = i;
    }
    for j in 0..order {
        let indexj = j * order;
        let indexjj = indexj + j;
        vec_lu_colj[..order].copy_from_slice(&data[indexj..indexj + order]);
        for i in 0..order {
            let kmax = i.min(j);
            let mut s = 0.0;
            for k in 0..kmax {
                s += data[k * order + i] * vec_lu_colj[k];
            }
            vec_lu_colj[i] -= s;
            data[indexj + i] = vec_lu_colj[i];
        }
        let mut p = j;
        for i in (j + 1)..order {
            if vec_lu_colj[i].abs() > vec_lu_colj[p].abs() {
                p = i;
            }
        }
        if p != j {
            for k in 0..order {
                let indexk = k * order;
                data.swap(indexk + p, indexk + j);
            }
            ipiv[j] = p;
        }
        if data[indexjj] != 0.0 {
            for i in (j + 1)..order {
                data[indexj + i] /= data[indexjj];
            }
        }
    }
}
