//! The geometry of the strip clip rib tests (`Box`, `Ribbed`, `FlatRib`, `Plain`, `Chain` and
//! `Positions`), as a fixture module so other test files can read the rib: a 20 mm box whose
//! top front edge is picked past a wedge rib standing on the top face - the rib's tip T (10, 3, 10),
//! base corners P (9, 8, 10) and Q (11, 8, 10), ridge top R (10, 8, 20).

#![allow(dead_code)]

use vector_math::{Matrix4X4, Vector3};

/// Box corners bottom 0-3, top 4-7; the rib's tip T 8, base corners P 9 and Q 10, ridge top R 11.
fn the_box() -> Vec<Vector3> {
    vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(20.0, 0.0, 0.0),
        Vector3::new(20.0, 20.0, 0.0),
        Vector3::new(0.0, 20.0, 0.0),
        Vector3::new(0.0, 0.0, 10.0),
        Vector3::new(20.0, 0.0, 10.0),
        Vector3::new(20.0, 20.0, 10.0),
        Vector3::new(0.0, 20.0, 10.0),
        Vector3::new(10.0, 3.0, 10.0),
        Vector3::new(9.0, 8.0, 10.0),
        Vector3::new(11.0, 8.0, 10.0),
        Vector3::new(10.0, 8.0, 20.0),
    ]
}

const BOX_FACES: [(i32, i32, i32); 10] = [
    (0, 2, 1),
    (0, 3, 2),
    (0, 1, 5),
    (0, 5, 4),
    (1, 2, 6),
    (1, 6, 5),
    (2, 3, 7),
    (2, 7, 6),
    (3, 0, 4),
    (3, 4, 7),
];

/// The top z = 10 round the rib's base triangle T P Q, all counter-clockwise from above.
const RIBBED_TOP: [(i32, i32, i32); 7] = [
    (4, 5, 8),
    (5, 10, 8),
    (5, 6, 10),
    (6, 9, 10),
    (6, 7, 9),
    (7, 4, 9),
    (4, 8, 9),
];

/// The two flanks (T R P facing -x, T Q R facing +x) and the back
/// face y = 8.
pub const RIB_FACES: [(i32, i32, i32); 3] = [(8, 11, 9), (8, 10, 11), (9, 11, 10)];

const PLAIN_TOP: [(i32, i32, i32); 2] = [(4, 5, 6), (4, 6, 7)];

/// The box's top front edge.
pub const CHAIN: [i32; 2] = [4, 5];

pub fn ribbed() -> Vec<(i32, i32, i32)> {
    BOX_FACES
        .iter()
        .chain(RIBBED_TOP.iter())
        .chain(RIB_FACES.iter())
        .copied()
        .collect()
}

/// The rib flattened into the top, its base split at the midpoint M
/// (index 11).
pub fn flat_rib() -> Vec<(i32, i32, i32)> {
    BOX_FACES
        .iter()
        .chain(RIBBED_TOP.iter().filter(|&&t| t != (6, 9, 10)))
        .copied()
        .chain([(6, 9, 11), (6, 11, 10), (8, 10, 11), (8, 11, 9)])
        .collect()
}

pub fn flat_rib_positions() -> Vec<Vector3> {
    let mut positions: Vec<Vector3> = the_box().into_iter().take(11).collect();
    positions.push(Vector3::new(10.0, 8.0, 10.0));
    positions
}

/// The box without the rib.
pub fn plain() -> Vec<(i32, i32, i32)> {
    BOX_FACES.iter().chain(PLAIN_TOP.iter()).copied().collect()
}

/// The box and rib, moved by `motion` when one is given.
pub fn positions(motion: Option<Matrix4X4>) -> Vec<Vector3> {
    match motion {
        Some(m) => the_box().into_iter().map(|p| p.transform(m)).collect(),
        None => the_box(),
    }
}
