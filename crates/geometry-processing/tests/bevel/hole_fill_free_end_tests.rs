//! Three filleted edges at one cube corner, each chain running from the corner's hole to a free end
//! on the far end face, filled with the filler on - the removed volume held to the closed form:
//! three straight sections swept to the corner cell's wall, plus the cell less its ball octant.

use std::f64::consts::PI;

use geometry_processing::bevel::level_line::{BandProfile, GapFillRefusal};
use vector_math::Vector3;

use crate::hole_fill_corner_tests::{build, fill_with_the_filler};
use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_test_geometry::{cube_solid, signed_volume};
use crate::several_chains_fixtures::FOLD_DEGREES;
use crate::sweep_mesh_measures;

/// Three filleted cube edges remove their straight sections with the filler on.
#[test]
fn three_filleted_cube_edges_remove_their_straight_sections_with_the_filler_on() {
    const HALF: f64 = 10.0;
    const SIZE: f64 = 3.0;
    const SEGMENTS: i32 = 8;
    let (positions, indices) = cube_solid(HALF);
    let chains = vec![vec![6, 5], vec![6, 7], vec![6, 2]];
    let closed = [false, false, false];
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
    let gap = build(
        &positions,
        &indices,
        &chains,
        &closed,
        &feature,
        SIZE,
        BandProfile::Fillet,
    );
    assert!(gap.built());

    let source = sweep_mesh_measures::signed_volume(&positions, &indices);
    let filled = fill_with_the_filler(&gap, BandProfile::Fillet, SEGMENTS);
    assert_eq!(filled.refusal, GapFillRefusal::None);

    // the section an 8-step arc removes: the square less the inscribed quarter polygon
    let section = (SIZE * SIZE)
        - (0.5 * SIZE * SIZE * SEGMENTS as f64 * (PI / (2.0 * SEGMENTS as f64)).sin());
    let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    for z in [6.9, 5.0, 0.0, -5.0, -9.9] {
        let removed_section = (4.0 * HALF * HALF)
            - section_area(
                mesh.vertices(),
                &kept,
                Vector3::new(0.0, 0.0, z),
                Vector3::new(0.0, 0.0, 1.0),
            );
        println!("z {z}: removed section {removed_section:.4} of {section:.4}");
        assert!((removed_section - section).abs() <= 0.01, "z {z}");
    }

    let cell = (SIZE * SIZE * SIZE) - (PI * SIZE * SIZE * SIZE / 6.0);
    let expected = (3.0 * ((2.0 * HALF) - SIZE) * section) + cell;
    let removed = source - signed_volume(&mesh);
    println!("removed {removed:.4}, closed form {expected:.4}");

    // The strips and free ends are exact, so what is left is the cell: the polyhedral ball keeps
    // about 0.32 less than the smooth octant (the app test's allowance), and the faired patch stands
    // up to 4% of r inside the ball, 0.41 more (the fair-not-exact trade the plan makes). Before the
    // skew spokes were carried onto the cylinder the pick removed 104.01 - 9.7 short.
    assert!((removed - expected).abs() <= 1.0);
}

/// The area of the mesh's cut by the plane through `origin` with `normal`.
fn section_area(
    positions: &[Vector3],
    triangles: &[(i32, i32, i32)],
    origin: Vector3,
    normal: Vector3,
) -> f64 {
    let mut area = 0.0;
    for &(ta, tb, tc) in triangles {
        let p = [
            positions[ta as usize],
            positions[tb as usize],
            positions[tc as usize],
        ];
        let d: Vec<f64> = p.iter().map(|&x| normal.dot(x - origin)).collect();
        let mut hits = Vec::new();
        for i in 0..3 {
            let j = (i + 1) % 3;
            if (d[i] < 0.0) != (d[j] < 0.0) {
                hits.push(p[i] + ((p[j] - p[i]) * (d[i] / (d[i] - d[j]))));
            }
        }

        if hits.len() != 2 {
            continue;
        }

        let face_normal = (p[1] - p[0]).cross(p[2] - p[0]);
        let (a, b) = if (hits[1] - hits[0]).dot(normal.cross(face_normal)) > 0.0 {
            (hits[0], hits[1])
        } else {
            (hits[1], hits[0])
        };
        area += (a - origin).cross(b - origin).dot(normal) / 2.0;
    }

    area.abs()
}
