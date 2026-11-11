//! Reduce on seam-dense and seam-bent textured parts, and on open bare parts: a part cut into
//! many UV islands reaches the lowest count its pinned seam crossings allow, a reduced seam stays
//! where the input put it, and a bare open part reduces bit for bit as before textures and colours
//! were carried.

use std::sync::Arc;

use crate::reduce_test_support::{
    assert_closed_manifold, assert_open_manifold, banded_picture, geometry_hash, oracle_uvs,
    picture_error, position,
};
use crate::test_picture::TestPicture;
use solidmesh_math::{Color, TexturePixels, Vector2Float, Vector3Float};
use solidmesh_mesh::{FaceTextureData, Mesh};

/// A flat 10 x 10 square of `n` x `n` quads, cut into `cells` x `cells` UV islands (each its own
/// cell of the atlas), every vertex shared: a seam-dense part like a scan's atlas.
fn island_grid(n: i32, cells: i32, picture: &Arc<TestPicture>) -> Mesh {
    let mut mesh = Mesh::new();
    for y in 0..=n {
        for x in 0..=n {
            mesh.vertices.push(Vector3Float::new(
                10.0 * x as f64 / n as f64,
                10.0 * y as f64 / n as f64,
                0.0,
            ));
        }
    }
    let per_cell = n / cells;
    let id = |x: i32, y: i32| y * (n + 1) + x;
    let uv = |x: i32, y: i32, cx: i32, cy: i32| {
        let lx = (x - cx * per_cell) as f64 / per_cell as f64;
        let ly = (y - cy * per_cell) as f64 / per_cell as f64;
        Vector2Float::new(
            (cx as f64 + 0.05 + 0.9 * lx) / cells as f64,
            (cy as f64 + 0.05 + 0.9 * ly) / cells as f64,
        )
    };
    for y in 0..n {
        for x in 0..n {
            let (cx, cy) = (x / per_cell, y / per_cell);
            for tri in [
                [(x, y), (x + 1, y), (x + 1, y + 1)],
                [(x, y), (x + 1, y + 1), (x, y + 1)],
            ] {
                let face = mesh.faces.len() as i32;
                mesh.faces.add(
                    id(tri[0].0, tri[0].1),
                    id(tri[1].0, tri[1].1),
                    id(tri[2].0, tri[2].1),
                    &mesh.vertices,
                );
                let image: Arc<dyn TexturePixels> = picture.clone();
                let [a, b, c] = tri.map(|(px, py)| uv(px, py, cx, cy));
                mesh.face_textures
                    .insert(face, FaceTextureData::new(Some(image), a, b, c));
            }
        }
    }
    mesh
}

#[test]
fn reduce_of_a_seam_dense_part_reaches_its_target() {
    // 256 islands of 3 x 3 quads: 4608 triangles, a seam every 3 quads in both directions.
    let picture = banded_picture();
    let islands = 16 * 16;
    let original = island_grid(48, 16, &picture);
    let count = original.faces.len() as i32;
    for maintain_surface in [false, true] {
        let target = count / 10;
        // Every seam crossing is pinned, so no island can lose its seam corners: each keeps at
        // least its two triangles, except the four at the plate's corners, whose free corner can
        // fold along the boundary onto a seam end. The reduce can go no lower than that.
        let reachable = target.max(2 * islands - 4);
        let reduced = original.reduced_to_triangle_count(target, maintain_surface);
        assert_open_manifold(
            &original,
            &reduced,
            &format!("island grid maintain_surface {maintain_surface}"),
        );
        println!(
            "maintain_surface {maintain_surface}: {count} faces, target {target}, floor {}, \
             reached {}",
            2 * islands - 4,
            reduced.faces.len()
        );
        assert!(
            reduced.faces.len() as i32 <= reachable,
            "stalled at {} with {reachable} reachable",
            reduced.faces.len()
        );
        assert_eq!(reduced.face_textures.len(), reduced.faces.len());
    }
}

/// An open plate of `n` x `n` quads over a 10 x 10 square, gently bumped so the quadrics are not
/// all zero, every vertex shared.
fn bumpy_plate(n: i32) -> Mesh {
    let mut mesh = Mesh::new();
    for y in 0..=n {
        for x in 0..=n {
            let px = 10.0 * x as f64 / n as f64;
            let py = 10.0 * y as f64 / n as f64;
            let z = 0.4 * libm::sin(0.7 * px) * libm::cos(0.5 * py);
            mesh.vertices.push(Vector3Float::new(px, py, z));
        }
    }
    let id = |x: i32, y: i32| y * (n + 1) + x;
    for y in 0..n {
        for x in 0..n {
            mesh.faces
                .add(id(x, y), id(x + 1, y), id(x + 1, y + 1), &mesh.vertices);
            mesh.faces
                .add(id(x, y), id(x + 1, y + 1), id(x, y + 1), &mesh.vertices);
        }
    }
    mesh
}

#[test]
fn reduce_of_a_bare_open_part_gives_the_same_mesh_as_before() {
    // Hashes of the reduced geometry taken before faces carried textures and colours through
    // Reduce. An open part runs the boundary rules, which the seam rules sit beside.
    let bare = bumpy_plate(40);
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
        assert_open_manifold(
            &bare,
            &reduced,
            &format!("bare plate maintain_surface {maintain}, target {target}"),
        );
        geometry_hash(&reduced)
    })
    .collect();
    assert_eq!(
        hashes,
        vec![
            0x98e5_19a1_00af_2dd0,
            0x7ec1_da01_416e_8dcf,
            0x1ddb_6122_75cb_4f63,
            0x41f8_88e2_b417_4aff,
        ]
    );
}

/// The torus's UV islands: four columns around the ring by two rows around the tube.
fn torus_island(uv: [f64; 2]) -> i32 {
    // UVs lie in [0, 1], so the casts stay small.
    let column = ((uv[0] * 4.0).floor() as i32).clamp(0, 3);
    let row = ((uv[1] * 2.0).floor() as i32).clamp(0, 1);
    column + 4 * row
}

/// A torus (ring radius 10, tube radius 4) of `segments` around the ring by `sides` around the
/// tube, every vertex shared, cut into the eight UV islands of [`torus_island`]. It has no UV
/// poles, so every corner's nearest original UV in its own island is well defined.
fn atlas_torus(segments: i32, sides: i32, picture: &Arc<TestPicture>) -> Mesh {
    let mut mesh = Mesh::new();
    let tau = std::f64::consts::TAU;
    for i in 0..segments {
        let theta = tau * i as f64 / segments as f64;
        for j in 0..sides {
            let phi = tau * j as f64 / sides as f64;
            let r = 10.0 + 4.0 * libm::cos(phi);
            mesh.vertices.push(Vector3Float::new(
                r * libm::cos(theta),
                r * libm::sin(theta),
                4.0 * libm::sin(phi),
            ));
        }
    }
    let id = |i: i32, j: i32| (i % segments) * sides + (j % sides);
    let (per_column, per_row) = (segments / 4, sides / 2);
    let uv = |i: i32, j: i32, column: i32, row: i32| {
        let lu = (i - column * per_column) as f64 / per_column as f64;
        let lv = (j - row * per_row) as f64 / per_row as f64;
        Vector2Float::new(
            (column as f64 + 0.05 + 0.9 * lu) / 4.0,
            (row as f64 + 0.05 + 0.9 * lv) / 2.0,
        )
    };
    for i in 0..segments {
        for j in 0..sides {
            let (column, row) = (i / per_column, j / per_row);
            for tri in [
                [(i, j), (i + 1, j), (i + 1, j + 1)],
                [(i, j), (i + 1, j + 1), (i, j + 1)],
            ] {
                let face = mesh.faces.len() as i32;
                mesh.faces.add(
                    id(tri[0].0, tri[0].1),
                    id(tri[1].0, tri[1].1),
                    id(tri[2].0, tri[2].1),
                    &mesh.vertices,
                );
                let image: Arc<dyn TexturePixels> = picture.clone();
                let [a, b, c] = tri.map(|(pi, pj)| uv(pi, pj, column, row));
                mesh.face_textures
                    .insert(face, FaceTextureData::new(Some(image), a, b, c));
            }
        }
    }
    mesh
}

/// How much worse than the nearest-UV oracle the reduced torus's picture may be, and the floor
/// (0-255 colour units) below which errors count as equal. The oracle is the same reduced shape
/// wearing, at each corner, the UV of the nearest original point in the face's own island: the
/// best its fewer, larger faces can do with per-corner UVs. Matching it means the picture looks as
/// it would on a reduced shape that was textured afresh.
const TORUS_ORACLE_FACTOR: f64 = 1.1;
const TORUS_MEAN_FLOOR: f64 = 0.5;
const TORUS_P99_FLOOR: f64 = 4.0;

#[test]
fn reduce_of_a_textured_torus_matches_the_nearest_uv_oracle() {
    let picture = banded_picture();
    let original = atlas_torus(64, 32, &picture);
    let count = original.faces.len() as i32;
    let mut measured = Vec::new();
    for maintain_surface in [false, true] {
        for target in [count / 2, count / 10] {
            let reduced = original.reduced_to_triangle_count(target, maintain_surface);
            assert!(reduced.faces.len() as i32 <= target);
            assert_closed_manifold(
                &reduced,
                &format!("torus maintain_surface {maintain_surface}, target {target}"),
            );
            let (mean, p99) = picture_error(&original, &reduced, &picture, torus_island);
            let oracle = oracle_uvs(&original, &reduced, torus_island);
            let (oracle_mean, oracle_p99) =
                picture_error(&original, &oracle, &picture, torus_island);
            println!(
                "torus maintain_surface {maintain_surface}, {count} -> {} faces: reducer mean \
                 {mean:.3} p99 {p99:.3}, oracle mean {oracle_mean:.3} p99 {oracle_p99:.3}",
                reduced.faces.len()
            );
            measured.push((maintain_surface, target, mean, p99, oracle_mean, oracle_p99));
        }
    }
    for (maintain_surface, target, mean, p99, oracle_mean, oracle_p99) in measured {
        assert!(
            mean <= TORUS_ORACLE_FACTOR * oracle_mean + TORUS_MEAN_FLOOR
                && p99 <= TORUS_ORACLE_FACTOR * oracle_p99 + TORUS_P99_FLOOR,
            "maintain_surface {maintain_surface}, {target} faces: mean {mean} (oracle \
             {oracle_mean}), p99 {p99} (oracle {oracle_p99})"
        );
    }
}

/// Which chart a plate point belongs to: a staircase of one-unit steps, chart 0 below it.
fn staircase_chart(x: f64, y: f64) -> u8 {
    if x.floor() + y.floor() < 10.0 {
        0
    } else {
        1
    }
}

const CHART_COLORS: [Color; 2] = [Color::rgba(200, 40, 40, 255), Color::rgba(40, 40, 200, 255)];

/// A 10 x 10 plate of 40 x 40 quads, flat or bumped, in two colours meeting along the staircase
/// of [`staircase_chart`] (steps of four quads).
fn staircase_plate(bumped: bool) -> Mesh {
    let n = 40;
    let mut mesh = Mesh::new();
    for y in 0..=n {
        for x in 0..=n {
            let px = 10.0 * x as f64 / n as f64;
            let py = 10.0 * y as f64 / n as f64;
            let z = if bumped {
                0.4 * libm::sin(0.7 * px) * libm::cos(0.5 * py)
            } else {
                0.0
            };
            mesh.vertices.push(Vector3Float::new(px, py, z));
        }
    }
    let id = |x: i32, y: i32| y * (n + 1) + x;
    let mut colors = Vec::new();
    for y in 0..n {
        for x in 0..n {
            let chart = staircase_chart((x as f64 + 0.5) / 4.0, (y as f64 + 0.5) / 4.0);
            mesh.faces
                .add(id(x, y), id(x + 1, y), id(x + 1, y + 1), &mesh.vertices);
            mesh.faces
                .add(id(x, y), id(x + 1, y + 1), id(x, y + 1), &mesh.vertices);
            colors.push(CHART_COLORS[chart as usize]);
            colors.push(CHART_COLORS[chart as usize]);
        }
    }
    mesh.face_colors = Some(colors);
    mesh
}

/// The share of the reduced plate's area that wears the other chart's colour: points spread over
/// each face, checked against the chart the staircase puts them in.
fn wrong_chart_share(reduced: &Mesh) -> f64 {
    let colors = reduced
        .face_colors
        .as_ref()
        .expect("the plate keeps its colours");
    let steps = 8;
    let (mut wrong, mut total) = (0.0, 0.0);
    for (face, f) in reduced.faces.iter().enumerate() {
        let [a, b, c] = [f.v0, f.v1, f.v2].map(|v| position(reduced, v));
        let e1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let e2 = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let n = [
            e1[1] * e2[2] - e1[2] * e2[1],
            e1[2] * e2[0] - e1[0] * e2[2],
            e1[0] * e2[1] - e1[1] * e2[0],
        ];
        let area = 0.5 * (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        let worn = CHART_COLORS
            .iter()
            .position(|&c| c == colors[face])
            .expect("a chart colour");
        let (mut off, mut samples) = (0, 0);
        for i in 0..steps {
            for j in 0..steps - i {
                let u = (i as f64 + 1.0 / 3.0) / steps as f64;
                let v = (j as f64 + 1.0 / 3.0) / steps as f64;
                let x = a[0] + u * e1[0] + v * e2[0];
                let y = a[1] + u * e1[1] + v * e2[1];
                samples += 1;
                if staircase_chart(x, y) as usize != worn {
                    off += 1;
                }
            }
        }
        wrong += area * off as f64 / samples as f64;
        total += area;
    }
    wrong / total
}

/// The most of a reduced staircase plate's area that may wear the wrong chart's colour: 0.1%.
/// A seam that keeps its bends leaves none (the reducer measures 0 in every case); one cut across
/// step corners leaves each corner's triangle on the wrong side - 0.3% to 1.9% of the area when
/// seam vertices are priced by position alone - which a real atlas shows as wrong-colour wedges
/// of gutter or neighbouring-chart pixels along its seams.
const WRONG_CHART_SHARE: f64 = 0.001;

#[test]
fn reduce_keeps_a_bent_seam_where_it_was() {
    let mut measured = Vec::new();
    for bumped in [false, true] {
        let original = staircase_plate(bumped);
        let count = original.faces.len() as i32;
        for maintain_surface in [false, true] {
            for target in [count / 2, count / 10] {
                let reduced = original.reduced_to_triangle_count(target, maintain_surface);
                assert_open_manifold(
                    &original,
                    &reduced,
                    &format!("staircase bumped {bumped}, maintain_surface {maintain_surface}"),
                );
                // A seam frozen in place would keep every chart outline trivially; the reduce
                // must still reach its target.
                assert!(
                    reduced.faces.len() as i32 <= target,
                    "stalled at {} of a {target} target",
                    reduced.faces.len()
                );
                let share = wrong_chart_share(&reduced);
                println!(
                    "staircase bumped {bumped}, maintain_surface {maintain_surface}: {count} -> \
                     {} faces, wrong-chart share {:.4}%",
                    reduced.faces.len(),
                    100.0 * share
                );
                measured.push((bumped, maintain_surface, target, share));
            }
        }
    }
    for (bumped, maintain_surface, target, share) in measured {
        assert!(
            share <= WRONG_CHART_SHARE,
            "bumped {bumped}, maintain_surface {maintain_surface}, {target} faces: {share}"
        );
    }
}
