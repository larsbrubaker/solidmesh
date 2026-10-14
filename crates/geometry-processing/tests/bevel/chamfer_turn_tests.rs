//! One chain turning 90 degrees over a cube's convex corner (the two top edges 5-6 and 6-7),
//! chamfered and filleted, removes the mitred closed form
//! (`BevelSurfaceLadderCornerTests.Rung3_TwoCubeEdgesAtACorner`'s).

use std::collections::HashSet;
use std::f64::consts::PI;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::{gap_fill, BandProfile, GapFillRefusal};
use vector_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_test_geometry::{cube_solid, signed_volume, signed_volume_of};
use crate::several_chains_fixtures::FOLD_DEGREES;
use crate::sweep_mesh_measures;

/// The two chamfer planes meet on
/// their bisector, so the pick removes 2 x 20 x d^2/2 less the d^3/3 the mitre stops counting
/// twice.
#[test]
fn a_chamfer_turning_over_a_cube_corner_removes_the_mitred_closed_form() {
    const HALF: f64 = 10.0;
    const SIZE: f64 = 3.0;
    let (positions, indices) = cube_solid(HALF);
    let chains = vec![vec![5, 6, 7]];
    let closed = [false];
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
    let gap = band_gap_entry_points::build_pick(
        &positions,
        &indices,
        &chains,
        &closed,
        Some(feature),
        SIZE,
        BandProfile::Chamfer,
        &BandGapBuildOptions::default(),
    )
    .expect("the pick builds or refuses by name");
    assert!(gap.built());

    let filled = gap_fill::fill(&gap, BandProfile::Chamfer, 1).expect("the fill runs");
    assert_eq!(filled.refusal, GapFillRefusal::None);

    let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    for t in sweep_mesh_measures::kept(&mesh) {
        let a = mesh.vertices()[t.0 as usize];
        let b = mesh.vertices()[t.1 as usize];
        let c = mesh.vertices()[t.2 as usize];
        let on = |plane: &dyn Fn(Vector3) -> f64| [a, b, c].iter().all(|&p| plane(p).abs() < 1e-6);
        let on_a_face = on(&|p| p.x - HALF)
            || on(&|p| p.x + HALF)
            || on(&|p| p.y - HALF)
            || on(&|p| p.y + HALF)
            || on(&|p| p.z - HALF)
            || on(&|p| p.z + HALF);
        let on_a_chamfer = on(&|p| p.x + p.z - 17.0) || on(&|p| p.y + p.z - 17.0);
        if !on_a_face && !on_a_chamfer {
            println!("off every plane: {a:?} {b:?} {c:?}");
        }
    }

    let removed = signed_volume_of(&positions, &indices) - signed_volume(&mesh);
    let expected = (2.0 * 2.0 * HALF * SIZE * SIZE / 2.0) - (SIZE * SIZE * SIZE / 3.0);
    println!("removed {removed:.4}, closed form {expected:.4}");
    assert!(
        (removed - expected).abs() <= 1e-6,
        "{removed} against {expected}"
    );
}

/// The same turn filleted, 8
/// segments - the swept sections less the mitred overlap, and the mitre's own arc lying in the
/// bisector plane x = y.
#[test]
fn a_fillet_turning_over_a_cube_corner_removes_the_mitred_closed_form() {
    const HALF: f64 = 10.0;
    const SIZE: f64 = 3.0;
    const SEGMENTS: i32 = 8;
    let (positions, indices) = cube_solid(HALF);
    let chains = vec![vec![5, 6, 7]];
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
    let gap = band_gap_entry_points::build_pick(
        &positions,
        &indices,
        &chains,
        &[false],
        Some(feature),
        SIZE,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
    )
    .expect("the pick builds or refuses by name");
    assert!(gap.built());

    let filled = gap_fill::fill(&gap, BandProfile::Fillet, SEGMENTS).expect("the fill runs");
    assert_eq!(filled.refusal, GapFillRefusal::None);

    // the profile as the ladder draws it: from (d, 0) on one face round to (0, d) on the other
    let profile: Vec<(f64, f64)> = (0..=SEGMENTS)
        .map(|k| {
            let k = k as f64;
            let n = SEGMENTS as f64;
            (
                SIZE - (SIZE * (PI / 2.0 * k / n).sin()),
                SIZE - (SIZE * (PI / 2.0 * k / n).cos()),
            )
        })
        .collect();
    let s = SEGMENTS as usize;
    // the area between the corner and the profile, as the ladder's SectionArea reads it
    let section = (0..s)
        .map(|k| ((profile[k].0 * profile[k + 1].1) - (profile[k + 1].0 * profile[k].1)) / 2.0)
        .sum::<f64>()
        .abs();
    let overlap: f64 = (0..s)
        .map(|k| {
            let (w0, w1) = (profile[k].0, profile[k + 1].0);
            (profile[k + 1].1 - profile[k].1) / 3.0 * ((w0 * w0) + (w0 * w1) + (w1 * w1))
        })
        .sum();

    let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    let mut seen = HashSet::new();
    let on_the_mitre: Vec<Vector3> = kept
        .iter()
        .flat_map(|k| [k.0, k.1, k.2])
        .filter(|&v| seen.insert(v))
        .map(|v| mesh.vertices()[v as usize])
        .filter(|p| p.z > 7.0 + 1e-6 && p.z < 10.0 - 1e-6 && p.x > 7.0 + 1e-6 && p.y > 7.0 + 1e-6)
        .collect();
    let worst_off_the_bisector = on_the_mitre
        .iter()
        .map(|p| (p.x - p.y).abs())
        .fold(0.0, f64::max);

    let removed = signed_volume_of(&positions, &indices) - signed_volume(&mesh);
    let expected = (2.0 * 2.0 * HALF * section) - overlap;
    println!(
        "removed {removed:.4}, closed form {expected:.4}; {} arc points near the turn, worst |x - y| {worst_off_the_bisector:.4}",
        on_the_mitre.len()
    );
    assert!(!on_the_mitre.is_empty());
    assert!(worst_off_the_bisector < 1e-9);

    // The kernel's rows are rational quadratics at even steps of their parameter, not of their
    // angle, so one straight 20 mm edge removes 0.0065 more than the even-angle polygon the closed
    // form sweeps (39.5405 against 39.534); the two legs account for the 0.012 left.
    assert!(
        (removed - expected).abs() <= 0.02,
        "{removed} against {expected}"
    );
}
