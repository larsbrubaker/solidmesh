// Unit tests for the band between two bevel rings (`band::add_band`): whether the rings touch at a
// point or along an edge, cross by a unit of rounding, or differ in how many loops they have (a
// feature that vanished between them), the band capped at each ring is a closed solid with every
// face wound outward (each edge walked once in each direction).

use std::collections::HashMap;
use std::rc::Rc;

use clipper2_rust::{Path64, Paths64, Point64};

use super::band::add_band;
use super::ring_mesh::RingMesh;
use crate::Mesh;

fn path(points: &[(i64, i64)]) -> Path64 {
    points.iter().map(|&(x, y)| Point64::new(x, y)).collect()
}

fn ring(points: &[(i64, i64)]) -> Paths64 {
    vec![path(points)]
}

/// The bottom ring capped facing down at z = 0 and walled straight up to z = 1, the band up to the
/// top ring at z = 2, and the top ring capped facing up; the top ring is the inner one, as in a
/// top bevel. (The straight wall keeps the band's flat faces, where a feature of the bottom ring
/// ends, off the bottom cap, as the extrusion's own wall does.)
fn banded_solid(bottom: &Paths64, top: &Paths64) -> Mesh {
    let mut rings = RingMesh::with_base(0.0);
    let bottom = Rc::new(bottom.clone());
    let base = rings.place(&bottom, 0.0);
    rings.cap(&base, &(0..bottom.len()).collect::<Vec<_>>(), false);
    let lower = rings.place(&bottom, 1.0);
    rings.straight_wall(&base, &lower);
    let upper = rings.place(&Rc::new(top.clone()), 2.0);
    add_band(&mut rings, &lower, &upper, true);
    rings.cap(&upper, &(0..top.len()).collect::<Vec<_>>(), true);
    rings.mesh
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

fn assert_closed(mesh: &Mesh) {
    assert!(!mesh.faces.is_empty());
    assert_eq!(unpaired_edges(mesh), Vec::new());
    assert!(mesh.get_volume() > 0.0, "the faces point outward");
}

const SQUARE: &[(i64, i64)] = &[(0, 0), (10000, 0), (10000, 10000), (0, 10000)];

#[test]
fn an_inset_ring_that_keeps_a_corner_of_the_profile_closes_the_band() {
    let bottom = ring(SQUARE);
    let top = ring(&[(0, 0), (9000, 1000), (9000, 9000), (1000, 9000)]);
    assert_closed(&banded_solid(&bottom, &top));
}

#[test]
fn an_inset_ring_that_keeps_an_edge_of_the_profile_closes_the_band() {
    let bottom = ring(SQUARE);
    let top = ring(&[(0, 0), (10000, 0), (9000, 9000), (1000, 9000)]);
    assert_closed(&banded_solid(&bottom, &top));
}

#[test]
fn an_inset_ring_clear_of_the_profile_closes_the_band() {
    let bottom = ring(SQUARE);
    let top = ring(&[(1000, 1000), (9000, 1000), (9000, 9000), (1000, 9000)]);
    assert_closed(&banded_solid(&bottom, &top));
}

/// Rounding to whole clipper units can push a point of the inner ring a unit outside the outer
/// one; the 2D area between them is then no band, and the rings are walled loop to loop.
#[test]
fn an_inset_ring_that_crosses_the_profile_by_a_unit_closes_the_band() {
    let bottom = ring(SQUARE);
    let top = ring(&[(1, 1), (10001, 5000), (9999, 9999), (1, 9999)]);
    assert_closed(&banded_solid(&bottom, &top));
}

/// Two blocks joined by a neck that pinches apart between the rings: one loop below, two above.
#[test]
fn a_neck_that_pinches_apart_between_the_rings_closes_the_band() {
    let bottom = ring(&[
        (0, 0),
        (4000, 0),
        (4000, 1800),
        (6000, 1800),
        (6000, 0),
        (10000, 0),
        (10000, 4000),
        (6000, 4000),
        (6000, 2200),
        (4000, 2200),
        (4000, 4000),
        (0, 4000),
    ]);
    let top = vec![
        path(&[(500, 500), (3500, 500), (3500, 3500), (500, 3500)]),
        path(&[(6500, 500), (9500, 500), (9500, 3500), (6500, 3500)]),
    ];
    assert_closed(&banded_solid(&bottom, &top));
}

/// A second block that is gone by the upper ring, while the upper ring also crosses the lower one
/// by a unit: the walled band zips the block that remains and caps the one that vanished.
#[test]
fn a_loop_that_vanishes_beside_crossing_rings_is_capped() {
    let bottom = vec![
        path(SQUARE),
        path(&[(20000, 0), (21000, 0), (21000, 1000), (20000, 1000)]),
    ];
    let top = ring(&[(1, 1), (10001, 5000), (9999, 9999), (1, 9999)]);
    assert_closed(&banded_solid(&bottom, &top));
}
