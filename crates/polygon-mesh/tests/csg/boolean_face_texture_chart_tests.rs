//! Face textures through a boolean on a large flat side whose mapping bends a little at every
//! edge: each pair of neighbours is nearly one map, but across the whole side the bend adds up,
//! so the side must not be handled as one face with one map. A groove cut along it must leave
//! the picture where it was. Uses the parent file's picture, boolean and sampling helpers.

use super::*;

/// Grid cells along each side of the plate's top.
const CELLS: usize = 100;
/// The size of one cell, on a 10 mm plate.
const CELL: f64 = 10.0 / CELLS as f64;

/// The top's mapping: `u` bends by `k` (a quadratic in x), `v` is linear in y.
fn plate_uv(x: f64, y: f64, k: f64) -> Vector2Float {
    let s = (x + 5.0) / 10.0;
    let t = (y + 5.0) / 10.0;
    Vector2Float::new(0.1 + 0.8 * (s + k * s * s) / (1.0 + k), 0.1 + 0.8 * t)
}

/// A 10 by 10 by 1 mm plate whose top (z = 0.5) is a `CELLS` by `CELLS` grid of textured
/// triangles mapped by [`plate_uv`]; the bottom and sides are untextured. Built with shared
/// vertex indices, so it is closed by construction. Also returns the top's vertex UVs.
fn plate(picture: &Arc<TestPicture>, k: f64) -> (Mesh, Vec<Vector2Float>) {
    let mut mesh = Mesh::new();
    let row = CELLS + 1;
    for z in [0.5, -0.5] {
        for j in 0..=CELLS {
            for i in 0..=CELLS {
                mesh.vertices.push(Vector3Float::new(
                    -5.0 + CELL * i as f64,
                    -5.0 + CELL * j as f64,
                    z,
                ));
            }
        }
    }
    let bottom = (row * row) as i32;
    let id = |i: usize, j: usize| (j * row + i) as i32;
    let image: Arc<dyn TexturePixels> = picture.clone();
    let mut vertex_uvs = Vec::new();
    for j in 0..=CELLS {
        for i in 0..=CELLS {
            vertex_uvs.push(plate_uv(-5.0 + CELL * i as f64, -5.0 + CELL * j as f64, k));
        }
    }
    for j in 0..CELLS {
        for i in 0..CELLS {
            let (a, b, c, d) = (id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1));
            for tri in [[a, b, c], [a, c, d]] {
                let face = mesh.faces.len() as i32;
                mesh.faces.add(tri[0], tri[1], tri[2], &mesh.vertices);
                let uv = tri.map(|v| vertex_uvs[v as usize]);
                mesh.face_textures.insert(
                    face,
                    FaceTextureData::new(Some(image.clone()), uv[0], uv[1], uv[2]),
                );
            }
            mesh.faces
                .add(a + bottom, c + bottom, b + bottom, &mesh.vertices);
            mesh.faces
                .add(a + bottom, d + bottom, c + bottom, &mesh.vertices);
        }
    }
    let mut ring = Vec::new();
    ring.extend((0..CELLS).map(|i| (i, 0)));
    ring.extend((0..CELLS).map(|j| (CELLS, j)));
    ring.extend((1..=CELLS).rev().map(|i| (i, CELLS)));
    ring.extend((1..=CELLS).rev().map(|j| (0, j)));
    for at in 0..ring.len() {
        let (i0, j0) = ring[at];
        let (i1, j1) = ring[(at + 1) % ring.len()];
        let (t0, t1) = (id(i0, j0), id(i1, j1));
        mesh.faces.add(t0, t0 + bottom, t1 + bottom, &mesh.vertices);
        mesh.faces.add(t0, t1 + bottom, t1, &mesh.vertices);
    }
    (mesh, vertex_uvs)
}

/// The top's UV at `(x, y)`, read through the grid triangle that point lies in.
fn source_uv(vertex_uvs: &[Vector2Float], x: f64, y: f64) -> (f64, f64) {
    let gx = (x + 5.0) / CELL;
    let gy = (y + 5.0) / CELL;
    // On the top, so the cell indices are within the grid.
    let i = (gx.floor() as usize).min(CELLS - 1);
    let j = (gy.floor() as usize).min(CELLS - 1);
    let fx = gx - i as f64;
    let fy = gy - j as f64;
    let at = |i: usize, j: usize| vertex_uvs[j * (CELLS + 1) + i];
    let (weights, uvs) = if fx >= fy {
        (
            [1.0 - fx, fx - fy, fy],
            [at(i, j), at(i + 1, j), at(i + 1, j + 1)],
        )
    } else {
        (
            [1.0 - fy, fx, fy - fx],
            [at(i, j), at(i + 1, j + 1), at(i, j + 1)],
        )
    };
    let mut u = 0.0;
    let mut v = 0.0;
    for (w, uv) in weights.iter().zip(uvs) {
        u += w * f64::from(uv.x);
        v += w * f64::from(uv.y);
    }
    (u, v)
}

/// The worst UV error on the top after a groove (`groove` places a 20 by 0.77 by 1 mm bar) is
/// cut along it, and how many textured top faces the result has.
fn worst_top_uv(k: f64, groove: Matrix4X4) -> (f64, usize) {
    let picture = banded_picture(0.0);
    let (plate, vertex_uvs) = plate(&picture, k);
    let bar = create_cube(20.0, 0.77, 1.0);
    let result = run(
        (&plate, Matrix4X4::IDENTITY),
        (&bar, groove),
        CsgModes::Subtract,
        None,
    );
    let mut worst: f64 = 0.0;
    let mut top = 0;
    for (&face, texture) in &result.face_textures {
        let corners = corners_of(&result, face as usize);
        if corners.iter().any(|p| (p.z - 0.5).abs() > 1e-6) {
            continue;
        }
        top += 1;
        for w in SAMPLE_WEIGHTS {
            let p = corners[0] * w[0] + corners[1] * w[1] + corners[2] * w[2];
            let (got_u, got_v) = uv_at(texture, w);
            let (want_u, want_v) = source_uv(&vertex_uvs, p.x, p.y);
            worst = worst.max((got_u - want_u).abs().max((got_v - want_v).abs()));
        }
    }
    (worst, top)
}

#[test]
fn a_groove_along_a_large_gently_bending_chart_keeps_its_mapping() {
    // k = 0 is one affine map over the whole top; 1.8e-3 bends each neighbour pair by less
    // than f32 rounding could explain, but the whole top by far more.
    for k in [0.0, 1.8e-3] {
        for groove in [
            Matrix4X4::create_translation(0.0, 0.13, 0.5),
            Matrix4X4::create_rotation_z(0.2) * Matrix4X4::create_translation(0.0, 0.13, 0.5),
        ] {
            let (worst, top) = worst_top_uv(k, groove);
            assert!(top > 0, "k = {k}: no textured top faces");
            assert!(worst <= UV_TOLERANCE, "k = {k}: worst UV {worst:e}");
        }
    }
}
