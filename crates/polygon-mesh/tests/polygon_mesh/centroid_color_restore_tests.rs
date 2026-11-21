//! `Mesh::restore_face_colors_from_centroids` against the plain scan it must agree with,
//! `Mesh::find_nearest_centroid_color`, face by face: random meshes, saved centroids exactly
//! equally near a face (the first in the saved list wins), duplicates, and degenerate saved lists
//! (one point, a flat box, coordinates so large their distances overflow, one centroid, none),
//! and NaN coordinates in saved centroids and in a face's own centroid.
//! The query points are the mesh's own face centroids as `save_face_centroid_colors` computes
//! them, the same ones the restore uses. An ignored test times the restore at the size of the
//! fillet that made the scan too slow (130k faces against 110k saved centroids).

use std::time::Instant;

use solidmesh_math::{Color, Vector3};
use solidmesh_mesh::{CentroidColor, Mesh};

/// A small deterministic generator, so every run tests the same inputs.
struct XorShift(u64);

impl XorShift {
    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }

    /// Uniform in [0, 1).
    fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    fn below(&mut self, n: u64) -> u64 {
        self.next_u64() % n
    }
}

fn distinct_color(i: usize) -> Color {
    Color::rgba(
        (i & 255) as u8,
        ((i >> 8) & 255) as u8,
        ((i >> 16) & 255) as u8,
        255,
    )
}

/// A face whose f32 centroid is exactly `c` when `c` is on a half-integer lattice: the corners
/// sum to 3c with no rounding.
fn add_face_centered_at(mesh: &mut Mesh, c: Vector3) {
    mesh.create_face(&[
        Vector3::new(c.x - 1.0, c.y - 1.0, c.z),
        Vector3::new(c.x + 2.0, c.y - 1.0, c.z),
        Vector3::new(c.x - 1.0, c.y + 2.0, c.z),
    ]);
}

fn random_triangle_mesh(rng: &mut XorShift, faces: usize, size: f64) -> Mesh {
    let mut mesh = Mesh::new();
    for _ in 0..faces {
        let corner = |rng: &mut XorShift| {
            Vector3::new(rng.unit() * size, rng.unit() * size, rng.unit() * size)
        };
        let a = corner(rng);
        let b = corner(rng);
        let c = corner(rng);
        mesh.create_face(&[a, b, c]);
    }
    mesh
}

/// The face centroids the restore will look up, computed by the production save.
fn face_centroids(mesh: &Mesh) -> Vec<Vector3> {
    let mut probe = mesh.copy();
    probe.face_colors = Some(vec![Color::WHITE; probe.faces.len()]);
    probe
        .save_face_centroid_colors()
        .expect("face colors were set")
        .into_iter()
        .map(|(centroid, _)| centroid)
        .collect()
}

/// Restores `saved` onto a copy of `mesh` and checks every face got exactly the colour the scan
/// gives for its centroid.
fn assert_restore_matches_scan(mesh: &Mesh, saved: &[CentroidColor]) {
    let centroids = face_centroids(mesh);
    let mut restored = mesh.copy();
    restored.restore_face_colors_from_centroids(Some(saved));
    let colors = restored.face_colors.expect("restore sets face colors");
    assert_eq!(colors.len(), centroids.len());
    for (face, (centroid, color)) in centroids.iter().zip(&colors).enumerate() {
        let expected = Mesh::find_nearest_centroid_color(*centroid, saved);
        assert_eq!(
            *color, expected,
            "face {face} at {centroid:?}: restore gave {color:?}, the scan {expected:?}"
        );
    }
}

#[test]
fn restore_matches_scan_on_a_random_mesh() {
    let mut rng = XorShift(0x9E37_79B9_7F4A_7C15);
    let mesh = random_triangle_mesh(&mut rng, 3000, 50.0);
    let other = random_triangle_mesh(&mut rng, 2000, 50.0);
    let saved: Vec<CentroidColor> = face_centroids(&other)
        .into_iter()
        .enumerate()
        .map(|(i, c)| (c, distinct_color(i)))
        .collect();
    assert_restore_matches_scan(&mesh, &saved);

    // Its own centroids: every face finds itself at distance zero.
    let own: Vec<CentroidColor> = face_centroids(&mesh)
        .into_iter()
        .enumerate()
        .map(|(i, c)| (c, distinct_color(i)))
        .collect();
    assert_restore_matches_scan(&mesh, &own);
}

#[test]
fn equally_near_saved_centroids_go_to_the_first_in_the_list() {
    // Faces on an integer lattice; saved centroids on the lattice shifted by half a step on
    // every axis, so each face has eight saved centroids exactly equally near.
    let mut mesh = Mesh::new();
    for x in 0..6 {
        for y in 0..6 {
            for z in 0..6 {
                add_face_centered_at(&mut mesh, Vector3::new(x as f64, y as f64, z as f64));
            }
        }
    }
    let mut lattice = Vec::new();
    for x in -1..7 {
        for y in -1..7 {
            for z in -1..7 {
                lattice.push(Vector3::new(x as f64 + 0.5, y as f64 + 0.5, z as f64 + 0.5));
            }
        }
    }
    // Shuffle so list order and spatial order disagree.
    let mut rng = XorShift(12345);
    for i in (1..lattice.len()).rev() {
        let j = rng.below(i as u64 + 1) as usize;
        lattice.swap(i, j);
    }
    let saved: Vec<CentroidColor> = lattice
        .iter()
        .enumerate()
        .map(|(i, &c)| (c, distinct_color(i)))
        .collect();
    assert_restore_matches_scan(&mesh, &saved);

    // The same points each saved three times with different colours, interleaved, so a later
    // duplicate must never beat an earlier one.
    let mut duplicated = Vec::new();
    for copy in 0..3 {
        for (i, &c) in lattice.iter().enumerate() {
            duplicated.push((c, distinct_color(copy * 1000 + i)));
        }
    }
    assert_restore_matches_scan(&mesh, &duplicated);

    // Axis neighbours only: the six saved centroids one step from a face, in reverse list order.
    let mut axis = Vec::new();
    for x in (0..6).rev() {
        for y in (0..6).rev() {
            for z in (0..6).rev() {
                let c = Vector3::new(x as f64, y as f64, z as f64);
                for d in [
                    Vector3::new(1.0, 0.0, 0.0),
                    Vector3::new(-1.0, 0.0, 0.0),
                    Vector3::new(0.0, 1.0, 0.0),
                    Vector3::new(0.0, -1.0, 0.0),
                    Vector3::new(0.0, 0.0, 1.0),
                    Vector3::new(0.0, 0.0, -1.0),
                ] {
                    axis.push((c + d, distinct_color(axis.len())));
                }
            }
        }
    }
    assert_restore_matches_scan(&mesh, &axis);
}

#[test]
fn degenerate_saved_lists_match_the_scan() {
    let mut rng = XorShift(0xDEAD_BEEF);
    let mesh = random_triangle_mesh(&mut rng, 400, 20.0);

    // Every saved centroid at one point.
    let one_point: Vec<CentroidColor> = (0..500)
        .map(|i| (Vector3::new(3.0, 4.0, 5.0), distinct_color(i)))
        .collect();
    assert_restore_matches_scan(&mesh, &one_point);

    // A flat box: no extent in z, and a line: no extent in y or z.
    let flat: Vec<CentroidColor> = (0..500)
        .map(|i| {
            let p = Vector3::new(rng.unit() * 20.0, rng.unit() * 20.0, 7.0);
            (p, distinct_color(i))
        })
        .collect();
    assert_restore_matches_scan(&mesh, &flat);
    let line: Vec<CentroidColor> = (0..500)
        .map(|i| {
            let p = Vector3::new((i % 37) as f64, 2.0, 2.0);
            (p, distinct_color(i))
        })
        .collect();
    assert_restore_matches_scan(&mesh, &line);

    // Huge coordinates: far points whose squared distance overflows to infinity, which the scan
    // never takes (it starts from f64::MAX), next to a few near ones it does.
    let mut huge: Vec<CentroidColor> = (0..300)
        .map(|i| {
            let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
            let p = Vector3::new(sign * 1e300, rng.unit() * 1e290, -sign * 1e299);
            (p, distinct_color(i))
        })
        .collect();
    assert_restore_matches_scan(&mesh, &huge);
    huge.push((Vector3::new(1e30, -1e30, 1e30), distinct_color(900)));
    huge.push((Vector3::new(10.0, 10.0, 10.0), distinct_color(901)));
    assert_restore_matches_scan(&mesh, &huge);

    // One saved centroid, and none.
    assert_restore_matches_scan(&mesh, &[(Vector3::new(1.0, 2.0, 3.0), Color::RED)]);
    assert_restore_matches_scan(&mesh, &[]);
    let mut empty = mesh.copy();
    empty.restore_face_colors_from_centroids(Some(&[]));
    assert!(empty
        .face_colors
        .expect("an empty list still sets colors")
        .iter()
        .all(|c| *c == Mesh::UNKNOWN_FACE_COLOR));
}

#[test]
fn nan_centroids_match_the_scan() {
    let mut rng = XorShift(0x0BAD_F00D);
    let mesh = random_triangle_mesh(&mut rng, 400, 20.0);
    let nan = f64::NAN;

    // NaN centroids among normal ones: first in the list (so a box starts from a NaN), on one
    // axis or all three, and scattered through the rest.
    let mixed: Vec<CentroidColor> = (0..600)
        .map(|i| {
            let p = match i % 7 {
                0 => Vector3::new(nan, nan, nan),
                3 => Vector3::new(rng.unit() * 20.0, nan, rng.unit() * 20.0),
                5 => Vector3::new(nan, rng.unit() * 20.0, rng.unit() * 20.0),
                _ => Vector3::new(rng.unit() * 20.0, rng.unit() * 20.0, rng.unit() * 20.0),
            };
            (p, distinct_color(i))
        })
        .collect();
    assert_restore_matches_scan(&mesh, &mixed);

    // Every saved centroid NaN: no distance is under f64::MAX, so nothing wins.
    let all_nan: Vec<CentroidColor> = (0..100)
        .map(|i| (Vector3::new(nan, nan, nan), distinct_color(i)))
        .collect();
    assert_restore_matches_scan(&mesh, &all_nan);

    // Faces whose own centroid is NaN, from a corner at NaN, among ordinary faces.
    let mut with_nan_faces = mesh.copy();
    for i in 0..20 {
        let c = i as f64;
        with_nan_faces.create_face(&[
            Vector3::new(c, 0.0, 0.0),
            Vector3::new(c + 1.0, 0.0, 0.0),
            Vector3::new(c, nan, 1.0),
        ]);
    }
    let nan_faces = face_centroids(&with_nan_faces)
        .iter()
        .filter(|c| c.y.is_nan())
        .count();
    assert_eq!(nan_faces, 20, "every added face's centroid is NaN");
    let normal: Vec<CentroidColor> = face_centroids(&mesh)
        .into_iter()
        .enumerate()
        .map(|(i, c)| (c, distinct_color(i)))
        .collect();
    assert_restore_matches_scan(&with_nan_faces, &normal);
    assert_restore_matches_scan(&with_nan_faces, &mixed);
}

#[test]
fn no_saved_list_leaves_the_colors_alone() {
    let mut mesh = Mesh::new();
    add_face_centered_at(&mut mesh, Vector3::new(0.0, 0.0, 0.0));
    mesh.restore_face_colors_from_centroids(None);
    assert!(mesh.face_colors.is_none());
}

/// The size of the fillet that spent most of its time here: about 130k output faces against
/// 110k saved centroids. Run with `--ignored --nocapture` in release to see the time.
#[test]
#[ignore = "timing only"]
fn time_restore_on_a_fillet_sized_mesh() {
    let mut rng = XorShift(0x1234_5678_9ABC_DEF0);
    let mesh = random_triangle_mesh(&mut rng, 130_000, 100.0);
    let other = random_triangle_mesh(&mut rng, 110_000, 100.0);
    let saved: Vec<CentroidColor> = face_centroids(&other)
        .into_iter()
        .enumerate()
        .map(|(i, c)| (c, distinct_color(i)))
        .collect();
    let mut restored = mesh.copy();
    let start = Instant::now();
    restored.restore_face_colors_from_centroids(Some(&saved));
    println!(
        "restore of {} faces against {} saved centroids: {:?}",
        restored.faces.len(),
        saved.len(),
        start.elapsed()
    );
}
