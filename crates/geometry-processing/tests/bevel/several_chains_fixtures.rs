//! The test fixture of the several-chains tests: the three several-chain picks A, B
//! and C, and the two settings every claim about them is read under. Also carries the two geometry
//! builders it calls (`PlateWithOffsetSquareHole` and `PlateWithTwoBosses`).

#![allow(dead_code)]

use crate::level_line_room_geometry::{SolidWithChains, SolidWithTwoChains};
use solidmesh_math::{Matrix4X4, Vector3};

/// A fold threshold that reads a 90 degree corner as a feature and a 16-sided
/// cylinder as smooth.
pub const FOLD_DEGREES: f64 = 45.0;

/// The same rigid motion the room's invariance is held to - a general rotation and a
/// translation.
pub fn motion() -> Matrix4X4 {
    Matrix4X4::create_rotation(Vector3::new(0.3, -0.7, 1.1))
        * Matrix4X4::create_translation_vector(Vector3::new(11.5, -3.25, 7.125))
}

/// Fixture "A", "B" or "C" - positions, triangles and the pick's chains.
pub fn named(fixture: &str) -> SolidWithChains {
    match fixture {
        "A" | "C" => {
            let (positions, indices, outer, hole) =
                plate_with_offset_square_hole(if fixture == "A" { 1.0 } else { 0.3 });
            (positions, indices, vec![outer, hole])
        }
        "B" => {
            let (positions, indices, first, second) = plate_with_two_bosses();
            (positions, indices, vec![first, second])
        }
        _ => panic!("no such fixture: {fixture}"),
    }
}

/// FIXTURE A (and, at a smaller gap, FIXTURE C)
/// - a 10 x 10 x 2 plate with a 2 x 2 square hole straight through it, the hole's nearest side
///   standing `gap_to_edge` from the plate's +x edge. Answers the top face's outer rim and hole rim,
///   both counter-clockwise seen from +z.
pub fn plate_with_offset_square_hole(gap_to_edge: f64) -> SolidWithTwoChains {
    const PLATE_HALF: f64 = 5.0;
    const HOLE_HALF: f64 = 1.0;
    const HALF_HEIGHT: f64 = 1.0;
    let centre_x = PLATE_HALF - gap_to_edge - HOLE_HALF;
    let corners = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)];

    let mut positions = Vec::new();
    for &(x, y) in &corners {
        positions.push(Vector3::new(x * PLATE_HALF, y * PLATE_HALF, HALF_HEIGHT));
    }

    for &(x, y) in &corners {
        positions.push(Vector3::new(
            centre_x + (x * HOLE_HALF),
            y * HOLE_HALF,
            HALF_HEIGHT,
        ));
    }

    let top = positions.len();
    for i in 0..top {
        let p = positions[i];
        positions.push(Vector3::new(p.x, p.y, -HALF_HEIGHT));
    }

    let top_outer = |k: i32| k % 4;
    let top_inner = |k: i32| 4 + (k % 4);
    let low_outer = |k: i32| 8 + (k % 4);
    let low_inner = |k: i32| 12 + (k % 4);

    // The same picture frame BoxWithSquareHole is built from: each band quad between an outer edge
    // and the inner edge facing it is a trapezoid, and stays convex however far the hole is moved.
    let mut indices = Vec::new();
    for k in 0..4 {
        indices.push((top_outer(k), top_outer(k + 1), top_inner(k + 1)));
        indices.push((top_outer(k), top_inner(k + 1), top_inner(k)));

        indices.push((low_outer(k), low_inner(k), low_inner(k + 1)));
        indices.push((low_outer(k), low_inner(k + 1), low_outer(k + 1)));

        indices.push((top_outer(k), low_outer(k), low_outer(k + 1)));
        indices.push((top_outer(k), low_outer(k + 1), top_outer(k + 1)));

        indices.push((top_inner(k), low_inner(k + 1), low_inner(k)));
        indices.push((top_inner(k), top_inner(k + 1), low_inner(k + 1)));
    }

    (positions, indices, vec![0, 1, 2, 3], vec![4, 5, 6, 7])
}

/// FIXTURE B - a 12 x 8 x 1 plate carrying two
/// 2 x 2 x 2 square bosses whose facing walls stand 3 apart. Answers both bosses' base rims, each
/// counter-clockwise seen from +z.
pub fn plate_with_two_bosses() -> SolidWithTwoChains {
    const PLATE_HALF_X: f64 = 6.0;
    const PLATE_HALF_Y: f64 = 4.0;
    const THICKNESS: f64 = 1.0;
    const BOSS_HALF: f64 = 1.0;
    const BOSS_HEIGHT: f64 = 2.0;
    const APART: f64 = 3.0;

    let mut positions: Vec<Vector3> = Vec::new();
    fn add(positions: &mut Vec<Vector3>, v: Vector3) -> i32 {
        positions.push(v);
        positions.len() as i32 - 1
    }

    // The plate's top ring, counter-clockwise seen from +z and carrying the two vertices on x = 0
    // where the face is split.
    let top_ring = [
        add(
            &mut positions,
            Vector3::new(-PLATE_HALF_X, -PLATE_HALF_Y, 0.0),
        ),
        add(&mut positions, Vector3::new(0.0, -PLATE_HALF_Y, 0.0)),
        add(
            &mut positions,
            Vector3::new(PLATE_HALF_X, -PLATE_HALF_Y, 0.0),
        ),
        add(
            &mut positions,
            Vector3::new(PLATE_HALF_X, PLATE_HALF_Y, 0.0),
        ),
        add(&mut positions, Vector3::new(0.0, PLATE_HALF_Y, 0.0)),
        add(
            &mut positions,
            Vector3::new(-PLATE_HALF_X, PLATE_HALF_Y, 0.0),
        ),
    ];
    let mut low_ring = Vec::with_capacity(top_ring.len());
    for &v in &top_ring {
        let p = positions[v as usize];
        low_ring.push(add(&mut positions, Vector3::new(p.x, p.y, -THICKNESS)));
    }

    let corners = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)];
    let centres = [-(APART / 2.0) - BOSS_HALF, (APART / 2.0) + BOSS_HALF];
    let mut bases: Vec<Vec<i32>> = Vec::with_capacity(2);
    let mut caps: Vec<Vec<i32>> = Vec::with_capacity(2);
    for &centre in &centres {
        let base: Vec<i32> = corners
            .iter()
            .map(|&(cx, cy)| {
                add(
                    &mut positions,
                    Vector3::new(centre + (cx * BOSS_HALF), cy * BOSS_HALF, 0.0),
                )
            })
            .collect();
        let cap: Vec<i32> = corners
            .iter()
            .map(|&(cx, cy)| {
                add(
                    &mut positions,
                    Vector3::new(centre + (cx * BOSS_HALF), cy * BOSS_HALF, BOSS_HEIGHT),
                )
            })
            .collect();
        bases.push(base);
        caps.push(cap);
    }

    let mut indices = Vec::new();

    // Each half of the top face: the picture frame between that half's outer rectangle and its
    // boss's footprint, wound +z.
    let halves = [
        [top_ring[0], top_ring[1], top_ring[4], top_ring[5]],
        [top_ring[1], top_ring[2], top_ring[3], top_ring[4]],
    ];
    for b in 0..2 {
        for k in 0..4 {
            let o = halves[b][k];
            let o_next = halves[b][(k + 1) % 4];
            let i = bases[b][k];
            let i_next = bases[b][(k + 1) % 4];
            indices.push((o, o_next, i_next));
            indices.push((o, i_next, i));
        }

        // The boss: four walls facing out of it, and a cap.
        for k in 0..4 {
            let a = bases[b][k];
            let c = bases[b][(k + 1) % 4];
            indices.push((a, c, caps[b][(k + 1) % 4]));
            indices.push((a, caps[b][(k + 1) % 4], caps[b][k]));
        }

        indices.push((caps[b][0], caps[b][1], caps[b][2]));
        indices.push((caps[b][0], caps[b][2], caps[b][3]));
    }

    // The bottom face, split at x = 0 exactly as the top is, wound -z.
    for quad in [
        [low_ring[0], low_ring[1], low_ring[4], low_ring[5]],
        [low_ring[1], low_ring[2], low_ring[3], low_ring[4]],
    ] {
        indices.push((quad[0], quad[2], quad[1]));
        indices.push((quad[0], quad[3], quad[2]));
    }

    // The plate's six side quads, one under each edge of the top ring.
    for k in 0..top_ring.len() {
        let a = top_ring[k];
        let c = top_ring[(k + 1) % top_ring.len()];
        let a_low = low_ring[k];
        let c_low = low_ring[(k + 1) % top_ring.len()];
        indices.push((a, a_low, c_low));
        indices.push((a, c_low, c));
    }

    let second = bases.pop().unwrap();
    let first = bases.pop().unwrap();
    (positions, indices, first, second)
}
