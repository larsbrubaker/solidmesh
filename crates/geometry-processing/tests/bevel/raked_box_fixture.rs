//! Port of the test fixture `LevelLine/RakedBoxFixture.cs` (the pick and its constants): a long
//! box whose one long wall is raked so its fold is 45 degrees, its other long wall square - so the
//! rim asks two different unit setbacks, and the room has a closed form.

#![allow(dead_code)]

use crate::level_line_room_geometry::SolidWithChain;
use vector_math::{Matrix4X4, Vector3};

pub const WIDTH: f64 = 2.0;
pub const LENGTH: f64 = 20.0;
pub const HEIGHT: f64 = 10.0;
pub const FOLD_DEGREES: f64 = 30.0;

/// C# `SquareUnit`: tan(45 degrees).
pub fn square_unit() -> f64 {
    (std::f64::consts::PI / 4.0).tan()
}

/// C# `RakedUnit`: tan(22.5 degrees).
pub fn raked_unit() -> f64 {
    (std::f64::consts::PI / 8.0).tan()
}

/// C# `TrueRoom`: the width over the two setbacks the strip spends across it.
pub fn true_room() -> f64 {
    WIDTH / (square_unit() + raked_unit())
}

/// C# `Pick(squareFirst, motion)`.
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
