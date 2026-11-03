//! A square crease end whose concave crease is a FACETED curve: a quarter round's top edge, convex
//! between the round's first facet and its flat front, ends square on a taller block's end wall,
//! which the round meets in a concave crease that follows the round's facets down. The block's
//! front carries on past the end in the front's plane. The round's section in the end wall meets
//! the crease at a setback from the end vertex, so its foot rides the crease from facet to facet
//! for as long as the crease runs on, and the room the crease leaves is the farthest the crease
//! reaches from the end vertex, not the first chord's length.

use geometry_processing::bevel::level_line::square_crease_end_clips;
use vector_math::Vector3;

/// The round's radius.
const RADIUS: f64 = 10.0;

/// A triangle wound so its normal faces along `outward`.
fn faced(positions: &[Vector3], t: (i32, i32, i32), outward: Vector3) -> (i32, i32, i32) {
    let [a, b, c] = [t.0, t.1, t.2].map(|i| positions[i as usize]);
    if (b - a).cross(c - a).dot(outward) >= 0.0 {
        t
    } else {
        (t.0, t.2, t.1)
    }
}

/// The quarter round of `facets` facets along x (axis at y = 0, z = -RADIUS, the solid y <= 0), from x = -5 to the end
/// wall x = 0; the chain is its top edge 1 - 0 on y = 0, z = 0, ending at vertex 0. The block past
/// the wall stands to z = 5 with the same front y = 0 and runs to x = 5.
fn quarter_round_onto_a_wall(facets: usize) -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    let length = 5.0;
    let height = 5.0;
    let mut positions = Vec::new();
    // At x = 0 (index 2k) and x = -length (index 2k + 1), k = 0..=facets; vertex 0 is the end and
    // vertex 1 the chain's other end.
    for k in 0..=facets {
        let angle = std::f64::consts::FRAC_PI_2 * k as f64 / facets as f64;
        let (y, z) = (-RADIUS * angle.sin(), RADIUS * angle.cos() - RADIUS);
        positions.push(Vector3::new(0.0, y, z));
        positions.push(Vector3::new(-length, y, z));
    }
    let base = positions.len() as i32;
    // The wall's top over each crease vertex, then the front's and the block's corners.
    for k in 0..=facets {
        let y = positions[2 * k].y;
        positions.push(Vector3::new(0.0, y, height));
    }
    let front = positions.len() as i32;
    positions.push(Vector3::new(0.0, 0.0, -RADIUS)); // front + 0: below the end vertex
    positions.push(Vector3::new(-length, 0.0, -RADIUS)); // front + 1
    positions.push(Vector3::new(length, 0.0, -RADIUS)); // front + 2
    positions.push(Vector3::new(length, 0.0, 0.0)); // front + 3

    let mut indices = Vec::new();
    for k in 0..facets as i32 {
        let (a, b, c, d) = (2 * k, 2 * k + 2, 2 * k + 3, 2 * k + 1);
        let mid = (positions[a as usize] + positions[c as usize]) * 0.5;
        let outward = Vector3::new(0.0, mid.y, mid.z + RADIUS);
        indices.push(faced(&positions, (a, b, c), outward));
        indices.push(faced(&positions, (a, c, d), outward));
        // The wall above this facet's crease chord, facing back over the round (-x).
        let wall = Vector3::new(-1.0, 0.0, 0.0);
        let (top_a, top_b) = (base + k, base + k + 1);
        indices.push(faced(&positions, (a, b, top_b), wall));
        indices.push(faced(&positions, (a, top_b, top_a), wall));
    }
    // The flat front, y = 0, under the round and on past the end under the block.
    let out = Vector3::new(0.0, 1.0, 0.0);
    indices.push(faced(&positions, (1, front + 1, front), out));
    indices.push(faced(&positions, (1, front, 0), out));
    indices.push(faced(&positions, (0, front, front + 2), out));
    indices.push(faced(&positions, (0, front + 2, front + 3), out));
    (positions, indices)
}

/// The end clip at the chain's end on the wall.
fn the_wall_end_clip(facets: usize) -> geometry_processing::bevel::level_line::EndRayBound {
    let (positions, indices) = quarter_round_onto_a_wall(facets);
    let (clip, other) =
        square_crease_end_clips(&positions, &indices, &[1, 0], false, (false, false));
    assert_eq!(clip, None, "the chain's far end is not on the wall");
    other.expect("the end on the wall takes the square crease clip")
}

/// A 40-gon's quarter: each facet folds 9 degrees off the last, one curve.
#[test]
fn a_faceted_crease_leaves_the_room_it_reaches() {
    let facets = 10;
    let clip = the_wall_end_clip(facets);

    // The crease is the quarter circle's chords, each farther from the end vertex than the last; its
    // last vertex, a quarter turn round, stands RADIUS * sqrt(2) from it.
    let reach = RADIUS * std::f64::consts::SQRT_2;
    assert!(
        (clip.crease_room - reach).abs() <= 1e-9 * reach,
        "room {} against the crease's reach {reach} (its first chord is {})",
        clip.crease_room,
        2.0 * RADIUS * (std::f64::consts::FRAC_PI_4 / facets as f64).sin()
    );
}

/// Two facets over the quarter fold 45 degrees apart: no curve carrying on but a corner, where the
/// section's foot leaves the crease, so the room stays the first chord's.
#[test]
fn a_crease_stops_at_a_corner() {
    let clip = the_wall_end_clip(2);
    let chord = 2.0 * RADIUS * std::f64::consts::FRAC_PI_8.sin();
    assert!(
        (clip.crease_room - chord).abs() <= 1e-9 * chord,
        "room {} against the first chord {chord}",
        clip.crease_room
    );
}
