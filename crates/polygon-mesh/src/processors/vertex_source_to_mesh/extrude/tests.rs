// Unit tests for the band skinning between two bevel rings (`create_triangulation`): where the
// rings touch - at a point or along an edge - the wall between them is vertical there, and the
// skinned band, closed by a cap at each ring, is a closed solid with every face wound outward
// (each edge walked once in each direction).

use std::collections::HashMap;

use clipper2_rust::{Path64, Paths64, Point64};

use super::create_triangulation;
use crate::csg::path_stitcher;
use crate::Mesh;

fn ring(points: &[(i64, i64)]) -> Paths64 {
    vec![points
        .iter()
        .map(|&(x, y)| Point64::new(x, y))
        .collect::<Path64>()]
}

/// The bottom ring capped facing down at z = 0, the band up to the top ring at z = 1, and the top
/// ring capped facing up.
fn banded_solid(bottom: &Paths64, top: &Paths64) -> Mesh {
    let mut mesh = Mesh::new();
    let bottom_cap = path_stitcher::stitch(None, 0.0, Some(bottom), 0.0, 1000.0).unwrap();
    mesh.copy_faces(&bottom_cap);
    create_triangulation(&mut mesh, bottom, 0.0, top, 1.0, false).unwrap();
    let top_cap = path_stitcher::stitch(Some(top), 1.0, None, 0.0, 1000.0).unwrap();
    mesh.copy_faces(&top_cap);
    mesh.clean_and_merge();
    mesh
}

/// The directed edges walked other than exactly once each way: empty for a closed, consistently
/// wound solid.
fn unpaired_edges(mesh: &Mesh) -> Vec<(i32, i32, i32)> {
    let mut walks: HashMap<(i32, i32), i32> = HashMap::new();
    for face in mesh.faces.iter() {
        for (a, b) in [(face.v0, face.v1), (face.v1, face.v2), (face.v2, face.v0)] {
            *walks.entry((a, b)).or_insert(0) += 1;
        }
    }
    let mut unpaired: Vec<(i32, i32, i32)> = walks
        .iter()
        .filter(|&(&(a, b), &count)| count != 1 || walks.get(&(b, a)) != Some(&1))
        .map(|(&(a, b), &count)| (a, b, count))
        .collect();
    unpaired.sort_unstable();
    unpaired
}

const SQUARE: &[(i64, i64)] = &[(0, 0), (10000, 0), (10000, 10000), (0, 10000)];

#[test]
fn an_inset_ring_that_keeps_a_corner_of_the_profile_closes_the_band() {
    let bottom = ring(SQUARE);
    let top = ring(&[(0, 0), (9000, 1000), (9000, 9000), (1000, 9000)]);
    let mesh = banded_solid(&bottom, &top);
    assert!(!mesh.faces.is_empty());
    assert_eq!(unpaired_edges(&mesh), Vec::new());
}

#[test]
fn an_inset_ring_that_keeps_an_edge_of_the_profile_closes_the_band() {
    let bottom = ring(SQUARE);
    let top = ring(&[(0, 0), (10000, 0), (9000, 9000), (1000, 9000)]);
    let mesh = banded_solid(&bottom, &top);
    assert!(!mesh.faces.is_empty());
    assert_eq!(unpaired_edges(&mesh), Vec::new());
}

#[test]
fn an_inset_ring_clear_of_the_profile_closes_the_band() {
    let bottom = ring(SQUARE);
    let top = ring(&[(1000, 1000), (9000, 1000), (9000, 9000), (1000, 9000)]);
    let mesh = banded_solid(&bottom, &top);
    assert_eq!(unpaired_edges(&mesh), Vec::new());
}
