//! What the reduce tests (`reduce_face_data_tests.rs`, `reduce_seam_tests.rs`) share: the banded
//! test picture and its bilinear lookup, the geometry hash bare reductions are pinned by, and the
//! picture-match measure - colours read through a reduced part's UVs against the colour at the
//! nearest original point of the same UV island - with the nearest-UV oracle it is compared to.

#![allow(dead_code)]

use std::sync::Arc;

use crate::test_picture::TestPicture;
use solidmesh_math::{TexturePixels, Vector2Float};
use solidmesh_mesh::Mesh;

pub const PICTURE_SIZE: i32 = 256;
/// The UV islands the test sphere is cut into: one atlas column per longitude sector.
pub const ISLANDS: i32 = 4;

/// The test picture: smooth bands of colour, several per island in each direction, so a UV that
/// slides along the surface shows as a colour change.
pub fn banded_picture() -> Arc<TestPicture> {
    let mut picture = TestPicture::new(PICTURE_SIZE, PICTURE_SIZE);
    let size = PICTURE_SIZE as usize;
    let bytes = picture.bytes_mut();
    for y in 0..size {
        for x in 0..size {
            let u = (x as f64 + 0.5) / size as f64;
            let v = (y as f64 + 0.5) / size as f64;
            let tau = std::f64::consts::TAU;
            let r = 127.5 + 127.5 * libm::sin(tau * 12.0 * u);
            let g = 127.5 + 127.5 * libm::sin(tau * 3.0 * v);
            let b = 127.5 + 127.5 * libm::sin(tau * 6.0 * (u + v));
            let at = (y * size + x) * 4;
            // Each channel lies in [0, 255], so the casts cannot saturate.
            bytes[at] = r.round() as u8;
            bytes[at + 1] = g.round() as u8;
            bytes[at + 2] = b.round() as u8;
            bytes[at + 3] = 255;
        }
    }
    Arc::new(picture)
}

/// The picture's colour at `(u, v)`, bilinearly filtered, with clamped edges.
pub fn sample(picture: &TestPicture, u: f64, v: f64) -> [f64; 3] {
    let size = PICTURE_SIZE as f64;
    let x = (u * size - 0.5).clamp(0.0, size - 1.0);
    let y = (v * size - 0.5).clamp(0.0, size - 1.0);
    // Clamped into the picture, so the casts stay in range.
    let x0 = x.floor() as usize;
    let y0 = y.floor() as usize;
    let x1 = (x0 + 1).min(PICTURE_SIZE as usize - 1);
    let y1 = (y0 + 1).min(PICTURE_SIZE as usize - 1);
    let fx = x - x0 as f64;
    let fy = y - y0 as f64;
    let bytes = picture.pixel_bytes();
    let at =
        |px: usize, py: usize, c: usize| bytes[(py * PICTURE_SIZE as usize + px) * 4 + c] as f64;
    let mut out = [0.0; 3];
    for (c, channel) in out.iter_mut().enumerate() {
        let top = at(x0, y0, c) * (1.0 - fx) + at(x1, y0, c) * fx;
        let bottom = at(x0, y1, c) * (1.0 - fx) + at(x1, y1, c) * fx;
        *channel = top * (1.0 - fy) + bottom * fy;
    }
    out
}

/// FNV-1a over every vertex's bits and every face's indices.
pub fn geometry_hash(mesh: &Mesh) -> u64 {
    let mut hash = 0xcbf2_9ce4_8422_2325u64;
    let mut mix = |bits: u32| {
        for byte in bits.to_le_bytes() {
            hash ^= byte as u64;
            hash = hash.wrapping_mul(0x0100_0000_01b3);
        }
    };
    for v in &mesh.vertices {
        mix(v.x.to_bits());
        mix(v.y.to_bits());
        mix(v.z.to_bits());
    }
    for f in mesh.faces.iter() {
        mix(f.v0 as u32);
        mix(f.v1 as u32);
        mix(f.v2 as u32);
    }
    hash
}

/// The closest point of triangle `abc` to `p`, as barycentric weights of `a`, `b`, `c`.
pub fn closest_barycentric(p: [f64; 3], a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> [f64; 3] {
    let sub = |x: [f64; 3], y: [f64; 3]| [x[0] - y[0], x[1] - y[1], x[2] - y[2]];
    let dot = |x: [f64; 3], y: [f64; 3]| x[0] * y[0] + x[1] * y[1] + x[2] * y[2];
    let ab = sub(b, a);
    let ac = sub(c, a);
    let ap = sub(p, a);
    let d1 = dot(ab, ap);
    let d2 = dot(ac, ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return [1.0, 0.0, 0.0];
    }
    let bp = sub(p, b);
    let d3 = dot(ab, bp);
    let d4 = dot(ac, bp);
    if d3 >= 0.0 && d4 <= d3 {
        return [0.0, 1.0, 0.0];
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let t = d1 / (d1 - d3);
        return [1.0 - t, t, 0.0];
    }
    let cp = sub(p, c);
    let d5 = dot(ab, cp);
    let d6 = dot(ac, cp);
    if d6 >= 0.0 && d5 <= d6 {
        return [0.0, 0.0, 1.0];
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let t = d2 / (d2 - d6);
        return [1.0 - t, 0.0, t];
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return [0.0, 1.0 - t, t];
    }
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    [1.0 - v - w, v, w]
}

pub fn position(mesh: &Mesh, vid: i32) -> [f64; 3] {
    let v = mesh.vertices[vid as usize];
    [v.x as f64, v.y as f64, v.z as f64]
}

/// A face's corners and UVs.
pub fn face_corners(mesh: &Mesh, face: usize) -> ([[f64; 3]; 3], [[f64; 2]; 3]) {
    let f = &mesh.faces[face];
    let t = &mesh.face_textures[&(face as i32)];
    let uv = |u: Vector2Float| [u.x as f64, u.y as f64];
    (
        [
            position(mesh, f.v0),
            position(mesh, f.v1),
            position(mesh, f.v2),
        ],
        [uv(t.uv0), uv(t.uv1), uv(t.uv2)],
    )
}

pub fn weighted<const N: usize>(w: [f64; 3], p: [[f64; N]; 3]) -> [f64; N] {
    let mut out = [0.0; N];
    for (k, value) in out.iter_mut().enumerate() {
        *value = w[0] * p[0][k] + w[1] * p[1][k] + w[2] * p[2][k];
    }
    out
}

/// The island a UV lies in (the atlas column).
pub fn island_of(uv: [f64; 2]) -> i32 {
    // UVs lie in [0, 1], so the cast stays small.
    ((uv[0] * ISLANDS as f64).floor() as i32).clamp(0, ISLANDS - 1)
}

/// How far the reduced part's picture is from the original's: for points spread over every
/// reduced face, the largest per-channel difference (0-255) between the colour read through the
/// reduced UVs and the colour at the nearest original point of the same island. Returns the mean
/// and the 99th percentile over every sample.
/// `island` names the UV island a UV lies in.
pub fn picture_error(
    original: &Mesh,
    reduced: &Mesh,
    picture: &TestPicture,
    island: fn([f64; 2]) -> i32,
) -> (f64, f64) {
    let weights = [
        [1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0],
        [0.6, 0.2, 0.2],
        [0.2, 0.6, 0.2],
        [0.2, 0.2, 0.6],
        [0.45, 0.45, 0.1],
        [0.1, 0.45, 0.45],
        [0.45, 0.1, 0.45],
    ];
    let originals: Vec<_> = (0..original.faces.len())
        .map(|f| {
            let (p, uv) = face_corners(original, f);
            (p, uv, island(weighted([1.0 / 3.0; 3], uv)))
        })
        .collect();
    let mut errors = Vec::new();
    for face in 0..reduced.faces.len() {
        let (p, uv) = face_corners(reduced, face);
        let face_island = island(weighted([1.0 / 3.0; 3], uv));
        for w in weights {
            let point = weighted(w, p);
            let here = sample(picture, weighted(w, uv)[0], weighted(w, uv)[1]);
            let mut best = (f64::MAX, [0.0; 2]);
            for (op, ouv, oisland) in &originals {
                if *oisland != face_island {
                    continue;
                }
                let b = closest_barycentric(point, op[0], op[1], op[2]);
                let q = weighted(b, *op);
                let d = (q[0] - point[0]).powi(2)
                    + (q[1] - point[1]).powi(2)
                    + (q[2] - point[2]).powi(2);
                if d < best.0 {
                    best = (d, weighted(b, *ouv));
                }
            }
            let there = sample(picture, best.1[0], best.1[1]);
            let diff = (0..3)
                .map(|c| (here[c] - there[c]).abs())
                .fold(0.0, f64::max);
            errors.push(diff);
        }
    }
    errors.sort_by(f64::total_cmp);
    let mean = errors.iter().sum::<f64>() / errors.len() as f64;
    let p99 = errors[(errors.len() * 99) / 100];
    (mean, p99)
}

/// The reduced part with the best UVs its geometry can wear: each corner of each face takes the
/// UV of the nearest original point in that face's own island. This oracle depends only on the
/// reduced shape, not on how the reducer carried UVs.
pub fn oracle_uvs(original: &Mesh, reduced: &Mesh, island: fn([f64; 2]) -> i32) -> Mesh {
    let originals: Vec<_> = (0..original.faces.len())
        .map(|f| {
            let (p, uv) = face_corners(original, f);
            (p, uv, island(weighted([1.0 / 3.0; 3], uv)))
        })
        .collect();
    let mut oracle = reduced.clone();
    for face in 0..reduced.faces.len() {
        let (p, uv) = face_corners(reduced, face);
        let face_island = island(weighted([1.0 / 3.0; 3], uv));
        let mut corner_uv = [Vector2Float::ZERO; 3];
        for (c, out) in corner_uv.iter_mut().enumerate() {
            let mut best = (f64::MAX, [0.0; 2]);
            for (op, ouv, oisland) in &originals {
                if *oisland != face_island {
                    continue;
                }
                let b = closest_barycentric(p[c], op[0], op[1], op[2]);
                let q = weighted(b, *op);
                let d =
                    (q[0] - p[c][0]).powi(2) + (q[1] - p[c][1]).powi(2) + (q[2] - p[c][2]).powi(2);
                if d < best.0 {
                    best = (d, weighted(b, *ouv));
                }
            }
            *out = Vector2Float::new(best.1[0], best.1[1]);
        }
        let texture = oracle
            .face_textures
            .get_mut(&(face as i32))
            .expect("every reduced face is textured");
        [texture.uv0, texture.uv1, texture.uv2] = corner_uv;
    }
    oracle
}

/// Panics, naming `what`, unless `mesh` is a closed, consistently wound manifold: every edge on
/// exactly two faces, walked once each way. Reduce must keep a closed solid closed.
pub fn assert_closed_manifold(mesh: &Mesh, what: &str) {
    let bad = mesh.get_non_manifold_edges();
    assert!(
        bad.is_empty(),
        "{what}: {} edges are not on exactly two faces (first: {:?})",
        bad.len(),
        bad.first()
            .map(|e| (e.vertex0_index, e.vertex1_index, e.faces().to_vec()))
    );
    assert!(
        mesh.is_closed_consistently_wound(),
        "{what}: the result is not closed and consistently wound"
    );
}

/// An open mesh's boundary edges, boundary loops (connected runs of boundary edges) and edges on
/// more than two faces.
fn open_edge_counts(mesh: &Mesh) -> (usize, usize, usize) {
    let bad = mesh.get_non_manifold_edges();
    let boundary: Vec<(i32, i32)> = bad
        .iter()
        .filter(|e| e.faces().len() == 1)
        .map(|e| (e.vertex0_index, e.vertex1_index))
        .collect();
    let over = bad.iter().filter(|e| e.faces().len() > 2).count();
    let mut parent: Vec<usize> = (0..mesh.vertices.len()).collect();
    fn root(parent: &mut [usize], mut v: usize) -> usize {
        while parent[v] != v {
            parent[v] = parent[parent[v]];
            v = parent[v];
        }
        v
    }
    let mut on_boundary = vec![false; mesh.vertices.len()];
    for &(a, b) in &boundary {
        on_boundary[a as usize] = true;
        on_boundary[b as usize] = true;
        let (ra, rb) = (root(&mut parent, a as usize), root(&mut parent, b as usize));
        if ra != rb {
            parent[ra] = rb;
        }
    }
    let loops = (0..mesh.vertices.len())
        .filter(|&v| on_boundary[v] && root(&mut parent, v) == v)
        .count();
    (boundary.len(), loops, over)
}

/// Panics, naming `what`, if reducing open `source` to `reduced` left an edge on more than two
/// faces, or tore a new hole: more boundary edges or boundary loops than the source had.
pub fn assert_open_manifold(source: &Mesh, reduced: &Mesh, what: &str) {
    let (source_edges, source_loops, _) = open_edge_counts(source);
    let (edges, loops, over) = open_edge_counts(reduced);
    assert_eq!(over, 0, "{what}: {over} edges are on more than two faces");
    assert!(
        edges <= source_edges && loops <= source_loops,
        "{what}: {edges} boundary edges in {loops} loops, from {source_edges} in {source_loops}"
    );
}
