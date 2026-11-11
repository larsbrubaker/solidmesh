//! Reduce (`Mesh::reduced_to_triangle_count`) on meshes that carry face textures and face
//! colours: the reduced part keeps its picture and its colours. The picture is measured, not
//! eyeballed: points sampled on the reduced surface look their colour up through the reduced UVs
//! and are compared with the colour at the nearest point of the original surface, read through
//! the original UVs. A mesh with neither keeps giving the very mesh it gave before textures and
//! colours were carried, bit for bit.

use std::sync::Arc;

use crate::reduce_test_support::{
    assert_closed_manifold, banded_picture, face_corners, geometry_hash, island_of, oracle_uvs,
    picture_error, ISLANDS,
};
use crate::test_picture::TestPicture;
use solidmesh_math::{Color, TexturePixels, Vector2Float, Vector3Float};
use solidmesh_mesh::{FaceTextureData, Mesh};

/// A closed sphere of radius 10, `rings` latitude bands by `segments` around (a multiple of
/// [`ISLANDS`]), every vertex shared. With `picture` each longitude sector is its own UV island
/// (a column of the atlas), so the sector borders and the poles are seams; with `colors` the two
/// halves of the sphere wear two colours.
fn atlas_sphere(
    rings: i32,
    segments: i32,
    picture: Option<&Arc<TestPicture>>,
    colors: bool,
) -> Mesh {
    let mut mesh = Mesh::new();
    let r = 10.0;
    let pi = std::f64::consts::PI;
    mesh.vertices.push(Vector3Float::new(0.0, 0.0, r));
    for i in 1..rings {
        let phi = pi * i as f64 / rings as f64;
        for j in 0..segments {
            let theta = 2.0 * pi * j as f64 / segments as f64;
            mesh.vertices.push(Vector3Float::new(
                r * libm::sin(phi) * libm::cos(theta),
                r * libm::sin(phi) * libm::sin(theta),
                r * libm::cos(phi),
            ));
        }
    }
    mesh.vertices.push(Vector3Float::new(0.0, 0.0, -r));
    let bottom = (mesh.vertices.len() - 1) as i32;
    let ring = |i: i32, j: i32| 1 + (i - 1) * segments + (j % segments);
    let per_island = segments / ISLANDS;
    // The UV of ring `i` (0 = top pole, `rings` = bottom pole), column `j` of sector `island`.
    let uv = |i: i32, j: i32, island: i32| {
        let local = (j - island * per_island) as f64 / per_island as f64;
        let u = (island as f64 + 0.05 + 0.9 * local) / ISLANDS as f64;
        let v = 0.02 + 0.96 * i as f64 / rings as f64;
        Vector2Float::new(u, v)
    };
    let mut corners: Vec<[(i32, i32); 3]> = Vec::new();
    let mut islands = Vec::new();
    for j in 0..segments {
        let island = j / per_island;
        let mut push = |tri: [(i32, i32); 3]| {
            corners.push(tri);
            islands.push(island);
        };
        push([(0, j), (1, j), (1, j + 1)]);
        push([(rings, j), (rings - 1, j + 1), (rings - 1, j)]);
        for i in 1..rings - 1 {
            push([(i, j), (i + 1, j), (i + 1, j + 1)]);
            push([(i, j), (i + 1, j + 1), (i, j + 1)]);
        }
    }
    let vertex = |(i, j): (i32, i32)| {
        if i == 0 {
            0
        } else if i == rings {
            bottom
        } else {
            ring(i, j)
        }
    };
    let mut face_colors = Vec::new();
    for (face, (tri, island)) in corners.iter().zip(&islands).enumerate() {
        mesh.faces.add(
            vertex(tri[0]),
            vertex(tri[1]),
            vertex(tri[2]),
            &mesh.vertices,
        );
        if let Some(picture) = picture {
            let image: Arc<dyn TexturePixels> = picture.clone();
            mesh.face_textures.insert(
                face as i32,
                FaceTextureData::new(
                    Some(image),
                    uv(tri[0].0, tri[0].1, *island),
                    uv(tri[1].0, tri[1].1, *island),
                    uv(tri[2].0, tri[2].1, *island),
                ),
            );
        }
        face_colors.push(if *island < ISLANDS / 2 {
            Color::new(200, 40, 40, 255)
        } else {
            Color::new(40, 40, 200, 255)
        });
    }
    if colors {
        mesh.face_colors = Some(face_colors);
    }
    mesh
}

/// Every reduced face wears the picture and the colour of the original faces of its island.
fn assert_regions_kept(reduced: &Mesh, picture: &Arc<TestPicture>) {
    let colors = reduced
        .face_colors
        .as_ref()
        .expect("the reduced part keeps its colours");
    assert_eq!(colors.len(), reduced.faces.len());
    assert_eq!(reduced.face_textures.len(), reduced.faces.len());
    for (face, &color) in colors.iter().enumerate() {
        let texture = &reduced.face_textures[&(face as i32)];
        assert!(
            texture.same_image(Some(picture)),
            "face {face} lost its picture"
        );
        let (_, uv) = face_corners(reduced, face);
        let islands: Vec<i32> = uv.iter().map(|&c| island_of(c)).collect();
        assert!(
            islands.iter().all(|&i| i == islands[0]),
            "face {face} straddles UV islands {islands:?}"
        );
        let want = if islands[0] < ISLANDS / 2 {
            Color::new(200, 40, 40, 255)
        } else {
            Color::new(40, 40, 200, 255)
        };
        assert_eq!(color, want, "face {face} took the wrong colour");
    }
}

/// The picture-match bounds on this sphere, in 0-255 colour units, as (target fraction, mean,
/// 99th percentile): just above what the reducer measures (the output is deterministic), and
/// below what it measures with the attribute quadric off (3.0 / 45 at a half, 26 / 202 at a
/// tenth) or with UVs left where they were. The nearest-UV oracle is printed but not bounded
/// here: at the poles every spoke is a UV seam and a corner's nearest original u is ambiguous, so
/// the oracle is unsound there; the torus test (`reduce_seam_tests.rs`) bounds against it.
const SPHERE_BOUNDS: [(i32, f64, f64); 2] = [(2, 2.0, 22.0), (10, 19.0, 150.0)];

#[test]
fn reduce_keeps_the_picture_and_colours_at_half_and_a_tenth() {
    let picture = banded_picture();
    let original = atlas_sphere(40, 64, Some(&picture), true);
    let count = original.faces.len() as i32;
    let mut measured = Vec::new();
    for maintain_surface in [false, true] {
        for (divisor, mean_bound, p99_bound) in SPHERE_BOUNDS {
            let target = count / divisor;
            let reduced = original.reduced_to_triangle_count(target, maintain_surface);
            assert_closed_manifold(
                &reduced,
                &format!("sphere maintain_surface {maintain_surface}, target {target}"),
            );
            assert!(
                reduced.faces.len() as i32 <= target,
                "reached {}",
                reduced.faces.len()
            );
            assert_regions_kept(&reduced, &picture);
            let (mean, p99) = picture_error(&original, &reduced, &picture, island_of);
            let oracle = oracle_uvs(&original, &reduced, island_of);
            let (oracle_mean, oracle_p99) = picture_error(&original, &oracle, &picture, island_of);
            println!(
                "sphere maintain_surface {maintain_surface}, {count} -> {} faces: reducer mean \
                 {mean:.3} p99 {p99:.3}, oracle mean {oracle_mean:.3} p99 {oracle_p99:.3}",
                reduced.faces.len()
            );
            measured.push((maintain_surface, target, mean, p99, mean_bound, p99_bound));
        }
    }
    for (maintain_surface, target, mean, p99, mean_bound, p99_bound) in measured {
        assert!(
            mean <= mean_bound && p99 <= p99_bound,
            "maintain_surface {maintain_surface}, {target} faces: mean {mean} (bound \
             {mean_bound}), p99 {p99} (bound {p99_bound})"
        );
    }
}

#[test]
fn reduce_keeps_face_colours_without_a_picture() {
    let picture = banded_picture();
    let original = atlas_sphere(30, 48, None, true);
    let count = original.faces.len() as i32;
    let reduced = original.reduced_to_triangle_count(count / 10, false);
    assert_closed_manifold(&reduced, "two-colour sphere at a tenth");
    assert!(reduced.face_textures.is_empty());
    let colors = reduced
        .face_colors
        .as_ref()
        .expect("the reduced part keeps its colours");
    assert_eq!(colors.len(), reduced.faces.len());
    let red = Color::new(200, 40, 40, 255);
    for (face, f) in reduced.faces.iter().enumerate() {
        // The red half is the sectors whose longitude runs from 0 to pi: y >= 0. Seam vertices
        // lie on y = 0 and belong to both halves.
        let ys = [f.v0, f.v1, f.v2].map(|v| reduced.vertices[v as usize].y as f64);
        let in_red = ys.iter().all(|&y| y >= -1e-4);
        let in_blue = ys.iter().all(|&y| y <= 1e-4);
        assert!(
            in_red || in_blue,
            "face {face} straddles the colour seam: {ys:?}"
        );
        if !(in_red && in_blue) {
            assert_eq!(
                colors[face] == red,
                in_red,
                "face {face} took the wrong colour"
            );
        }
    }
    drop(picture);
}

#[test]
fn reduce_of_a_textured_part_is_the_same_on_every_run() {
    let picture = banded_picture();
    let original = atlas_sphere(24, 32, Some(&picture), true);
    let target = original.faces.len() as i32 / 3;
    let first = original.reduced_to_triangle_count(target, true);
    let second = original.reduced_to_triangle_count(target, true);
    assert_closed_manifold(&first, "textured sphere at a third");
    assert_eq!(geometry_hash(&first), geometry_hash(&second));
    let uv_bits = |m: &Mesh| -> Vec<[u32; 6]> {
        m.face_textures
            .values()
            .map(|t| [t.uv0.x, t.uv0.y, t.uv1.x, t.uv1.y, t.uv2.x, t.uv2.y].map(|c| c.to_bits()))
            .collect()
    };
    assert_eq!(uv_bits(&first), uv_bits(&second));
}

#[test]
fn reduce_of_a_bare_part_gives_the_same_mesh_as_before() {
    // Hashes of the reduced geometry taken before faces carried textures and colours through
    // Reduce: a part with neither must not move by a bit.
    let bare = atlas_sphere(40, 64, None, false);
    let count = bare.faces.len() as i32;
    let hashes: Vec<u64> = [
        (count / 2, false),
        (count / 10, false),
        (count / 2, true),
        (count / 10, true),
    ]
    .into_iter()
    .map(|(target, maintain)| {
        let reduced = bare.reduced_to_triangle_count(target, maintain);
        assert_closed_manifold(
            &reduced,
            &format!("bare sphere maintain_surface {maintain}, target {target}"),
        );
        geometry_hash(&reduced)
    })
    .collect();
    assert_eq!(
        hashes,
        vec![
            0xe375_fc3d_3480_ed9a,
            0x0c9e_5f87_fa87_3c1b,
            0x5e12_3ffb_b770_576e,
            0x6b7e_42f6_0703_1c87,
        ]
    );
}
