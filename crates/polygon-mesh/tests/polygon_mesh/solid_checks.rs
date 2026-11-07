//! The clean-solid checks every path-to-solid operation is held to (Linear Extrude, Revolve,
//! Sweep), on the mesh alone: a closed solid (each edge walked once in each direction on shared
//! vertices), wound outward, with no face of zero area, and flat caps that cover a given area.
//! The area a path fills, the paths the checks run on and the re-export of these checks are in
//! mattercad-geometry's `tests/path_meshes/clean_solid_support.rs`. Shared through `#[path]`
//! modules by the binaries that check solids, so not every binary uses every helper.
#![allow(dead_code)]

use std::collections::HashMap;

use polygon_mesh::Mesh;

/// The directed edges not walked exactly once each way: empty for a closed solid whose faces
/// are all wound the same way round.
pub fn unpaired_edges(mesh: &Mesh) -> Vec<(i32, i32, u32)> {
    let mut walks: HashMap<(i32, i32), u32> = HashMap::new();
    for face in mesh.faces.iter() {
        for edge in [(face.v0, face.v1), (face.v1, face.v2), (face.v2, face.v0)] {
            *walks.entry(edge).or_insert(0) += 1;
        }
    }
    let mut unpaired: Vec<(i32, i32, u32)> = walks
        .iter()
        .filter(|&(&(a, b), &count)| count != 1 || walks.get(&(b, a)) != Some(&1))
        .map(|(&(a, b), &count)| (a, b, count))
        .collect();
    unpaired.sort_unstable();
    unpaired
}

/// A face's corners in f64.
fn corners(mesh: &Mesh, v: [i32; 3]) -> [[f64; 3]; 3] {
    v.map(|i| {
        let p = mesh.vertices[i as usize];
        [f64::from(p.x), f64::from(p.y), f64::from(p.z)]
    })
}

/// Twice the face's area projected on the xy plane, signed: positive counter clockwise from
/// above (facing up).
fn doubled_xy_area([a, b, c]: [[f64; 3]; 3]) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

/// How far a solid's cap area may stray from the area its path fills, as a fraction of it. The
/// profile's clean-up merges points less than 1.415 clipper units (1.4 microns) apart, which moves
/// a finely traced outline's area by a few millionths of itself.
pub const AREA_FRACTION: f64 = 5e-6;

/// Asserts the solid is clean and that its caps fill `filled_area`, within `tolerance` mm².
pub fn assert_clean_solid(mesh: &Mesh, filled_area: f64, height: f64, tolerance: f64, what: &str) {
    assert!(!mesh.faces.is_empty(), "{what}: no faces");
    let unpaired = unpaired_edges(mesh);
    assert!(
        unpaired.is_empty(),
        "{what}: {} edges are not walked once each way, e.g. {:?}",
        unpaired.len(),
        &unpaired[..unpaired.len().min(8)]
    );
    let (mut top_up, mut top_down, mut bottom_down, mut bottom_up) = (0.0, 0.0, 0.0, 0.0);
    let mut degenerate = 0;
    for face in mesh.faces.iter() {
        let points = corners(mesh, [face.v0, face.v1, face.v2]);
        let [a, b, c] = points;
        let (ab, ac) = (
            [b[0] - a[0], b[1] - a[1], b[2] - a[2]],
            [c[0] - a[0], c[1] - a[1], c[2] - a[2]],
        );
        let normal = [
            ab[1] * ac[2] - ab[2] * ac[1],
            ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0],
        ];
        if normal.iter().map(|n| n * n).sum::<f64>() == 0.0 {
            degenerate += 1;
        }
        let flat_at = |z: f64| points.iter().all(|p| (p[2] - z).abs() < 1e-6);
        let doubled = doubled_xy_area(points);
        if flat_at(height) {
            if doubled > 0.0 {
                top_up += doubled / 2.0;
            } else {
                top_down -= doubled / 2.0;
            }
        } else if flat_at(0.0) {
            if doubled < 0.0 {
                bottom_down -= doubled / 2.0;
            } else {
                bottom_up += doubled / 2.0;
            }
        }
    }
    assert_eq!(degenerate, 0, "{what}: faces with no area");
    assert!(
        top_down <= tolerance && bottom_up <= tolerance,
        "{what}: cap faces facing into the solid (top {top_down} mm², bottom {bottom_up} mm²)"
    );
    assert!(
        (top_up - filled_area).abs() <= tolerance && (bottom_down - filled_area).abs() <= tolerance,
        "{what}: the caps cover top {top_up} mm² and bottom {bottom_down} mm², the path fills \
         {filled_area} mm²"
    );
    let volume = mesh.get_volume();
    assert!(
        (volume - filled_area * height).abs() <= tolerance * height,
        "{what}: volume {volume}, expected {}",
        filled_area * height
    );
}

/// Asserts the solid is closed (every edge walked once each way), has no face of zero area and
/// encloses a positive volume, which with the faces all wound alike means they face outward.
pub fn assert_closed_outward(mesh: &Mesh, what: &str) {
    assert!(!mesh.faces.is_empty(), "{what}: no faces");
    let unpaired = unpaired_edges(mesh);
    assert!(
        unpaired.is_empty(),
        "{what}: {} edges are not walked once each way, e.g. {:?}",
        unpaired.len(),
        &unpaired[..unpaired.len().min(8)]
    );
    let degenerate = mesh
        .faces
        .iter()
        .filter(|face| {
            let [a, b, c] = corners(mesh, [face.v0, face.v1, face.v2]);
            let (ab, ac) = (
                [b[0] - a[0], b[1] - a[1], b[2] - a[2]],
                [c[0] - a[0], c[1] - a[1], c[2] - a[2]],
            );
            let normal = [
                ab[1] * ac[2] - ab[2] * ac[1],
                ab[2] * ac[0] - ab[0] * ac[2],
                ab[0] * ac[1] - ab[1] * ac[0],
            ];
            normal.iter().map(|n| n * n).sum::<f64>() == 0.0
        })
        .count();
    assert_eq!(degenerate, 0, "{what}: faces with no area");
    let volume = mesh.get_volume();
    assert!(
        volume > 0.0,
        "{what}: the solid faces inward (volume {volume})"
    );
}

/// The faces lying flat in the plane where coordinate `axis` is `at`, as the area facing the +axis
/// way and the area facing the other way.
pub fn flat_area(mesh: &Mesh, axis: usize, at: f64) -> (f64, f64) {
    let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
    let (mut facing_up, mut facing_down) = (0.0, 0.0);
    for face in mesh.faces.iter() {
        let points = corners(mesh, [face.v0, face.v1, face.v2]);
        if !points.iter().all(|p| (p[axis] - at).abs() < 1e-5) {
            continue;
        }
        let [a, b, c] = points;
        let doubled = (b[u] - a[u]) * (c[v] - a[v]) - (b[v] - a[v]) * (c[u] - a[u]);
        if doubled > 0.0 {
            facing_up += doubled / 2.0;
        } else {
            facing_down -= doubled / 2.0;
        }
    }
    (facing_up, facing_down)
}
