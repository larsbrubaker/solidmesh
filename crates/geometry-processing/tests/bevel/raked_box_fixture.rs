//! A long box whose one long wall is raked so its fold is 45 degrees, its other long wall square -
//! so the rim asks two different unit setbacks, and the room has a closed form. Holds the pick, its
//! constants, its room reading and build, and the unit setbacks its rim carries.

#![allow(dead_code)]

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::band_gap_reading;
use geometry_processing::bevel::level_line::chain_setback;
use geometry_processing::bevel::level_line::seam_edges::FeatureEdgeTest;
use geometry_processing::bevel::level_line::{BandGapResult, BandProfile, RoomReading};
use vector_math::{Matrix4X4, Vector3};

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_room_geometry::SolidWithChain;

pub const WIDTH: f64 = 2.0;
pub const LENGTH: f64 = 20.0;
pub const HEIGHT: f64 = 10.0;
pub const FOLD_DEGREES: f64 = 30.0;

/// Tan(45 degrees).
pub fn square_unit() -> f64 {
    (std::f64::consts::PI / 4.0).tan()
}

/// Tan(22.5 degrees).
pub fn raked_unit() -> f64 {
    (std::f64::consts::PI / 8.0).tan()
}

/// The width over the two setbacks the strip spends across it.
pub fn true_room() -> f64 {
    WIDTH / (square_unit() + raked_unit())
}

/// The rim pick, starting on the square wall's side when `square_first`, carried by `motion` when
/// given.
pub fn pick(square_first: bool, motion: Option<Matrix4X4>) -> SolidWithChain {
    let half_length = LENGTH / 2.0;
    let half_width = WIDTH / 2.0;

    // Each long edge carries a MIDPOINT: a chain vertex asks the mean of the edges meeting it.
    let top = [
        Vector3::new(half_length, half_width, HEIGHT),
        Vector3::new(0.0, half_width, HEIGHT),
        Vector3::new(-half_length, half_width, HEIGHT),
        Vector3::new(-half_length, -half_width, HEIGHT),
        Vector3::new(0.0, -half_width, HEIGHT),
        Vector3::new(half_length, -half_width, HEIGHT),
    ];

    // The raked wall's three bottom points step OUT in y by the height.
    let bottom = [
        Vector3::new(half_length, half_width + HEIGHT, 0.0),
        Vector3::new(0.0, half_width + HEIGHT, 0.0),
        Vector3::new(-half_length, half_width + HEIGHT, 0.0),
        Vector3::new(-half_length, -half_width, 0.0),
        Vector3::new(0.0, -half_width, 0.0),
        Vector3::new(half_length, -half_width, 0.0),
    ];

    let mut positions: Vec<Vector3> = top.iter().chain(bottom.iter()).copied().collect();
    if let Some(motion) = motion {
        for p in positions.iter_mut() {
            *p = p.transform(motion);
        }
    }

    let rim: Vec<i32> = if square_first {
        vec![3, 4, 5, 0, 1, 2]
    } else {
        vec![0, 1, 2, 3, 4, 5]
    };
    let mut indices = vec![
        // Top face as two quads about the midpoints, wound so its normal is +z.
        (0, 1, 4),
        (0, 4, 5),
        (1, 2, 3),
        (1, 3, 4),
        // Bottom face, the other way about.
        (6, 10, 7),
        (6, 11, 10),
        (7, 9, 8),
        (7, 10, 9),
    ];

    for i in 0..rim.len() {
        let a = rim[i];
        let b = rim[(i + 1) % rim.len()];
        indices.push((a, 6 + a, 6 + b));
        indices.push((a, 6 + b, b));
    }

    (positions, indices, rim)
}

/// The pick's feature edges - every fold of 30 degrees or more, which is all
/// eight of them.
pub fn feature_edges(positions: &[Vector3], indices: &[(i32, i32, i32)]) -> FeatureEdgeTest {
    feature_edges_over(positions, indices, FOLD_DEGREES)
}

/// The rim's room, read on the input mesh.
pub fn read(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    rim: &[i32],
    profile: BandProfile,
) -> RoomReading {
    band_gap_reading::read_room(
        positions,
        indices,
        rim,
        true,
        Some(feature_edges(positions, indices)),
        profile,
        1e-9,
        None,
        0.0,
        f64::INFINITY,
    )
    .expect("the rim reads")
}

/// A build of the rim at one size, optionally handed a room already read.
pub fn build(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    rim: &[i32],
    size: f64,
    known_room: Option<f64>,
    profile: BandProfile,
) -> BandGapResult {
    band_gap_entry_points::build(
        positions,
        indices,
        rim,
        true,
        Some(feature_edges(positions, indices)),
        size,
        profile,
        &BandGapBuildOptions {
            known_room,
            ..BandGapBuildOptions::default()
        },
    )
    .expect("a valid pick")
}

/// The two unit setbacks the rim's field really carries, read from the
/// production `ChainSetback` and the part's own face normals - so a fixture that quietly stopped
/// varying is caught rather than assumed.
pub fn unit_setbacks(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    rim: &[i32],
) -> (f64, f64) {
    let mut lowest = f64::INFINITY;
    let mut highest = f64::NEG_INFINITY;
    for i in 0..rim.len() {
        let a = rim[i];
        let b = rim[(i + 1) % rim.len()];
        let sides: Vec<Vector3> = indices
            .iter()
            .filter(|&&t| holds(t, a) && holds(t, b))
            .map(|&t| normal(positions, t))
            .collect();
        let unit = chain_setback::for_fillet(1.0, sides[0], sides[1]);
        lowest = lowest.min(unit);
        highest = highest.max(unit);
    }

    (lowest, highest)
}

fn holds(t: (i32, i32, i32), vertex: i32) -> bool {
    t.0 == vertex || t.1 == vertex || t.2 == vertex
}

fn normal(positions: &[Vector3], t: (i32, i32, i32)) -> Vector3 {
    (positions[t.1 as usize] - positions[t.0 as usize])
        .cross(positions[t.2 as usize] - positions[t.0 as usize])
        .get_normal()
}
