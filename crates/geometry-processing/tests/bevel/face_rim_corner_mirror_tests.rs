//! Center Right and Left Side's x = -20 face rim (chain 2 - 0 - 3 - 7) built alone. The part is
//! mirror-symmetric about z = 10 but its triangulation is not: the face is split along 2 - 3, which
//! passes through the bottom-back corner v3 (-20, -98.5, 0) and not the top-back corner
//! v7 (-20, -98.5, 20), and the bottom face's diagonal 0 - 6 crosses the front corner hole at
//! v0 (-20, -118.5, 0) where nothing crosses v2's (-20, -118.5, 20).

use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::gap_fill;
use solidmesh::bevel::level_line::{BandProfile, GapFillRefusal, GapFillResult};
use solidmesh_math::Vector3;

use crate::level_line_fixture::LevelLineFixture;
use crate::rounds_first_tests::distance_to_mesh;
use crate::sweep_mesh_measures;

/// The app's control comparison tolerance (BevelControlMeshComparison.ChordTolerance in
/// MatterCADTests).
const CONTROL_TOLERANCE: f64 = 0.05;

/// Every vertex the rim's fill added, mirrored about z = 10, lies on the built surface: the
/// right-angle corners v3 and v7 and the front corners v0 and v2 are each built alike whichever way
/// the faces' diagonals run - and the build does not cross itself.
#[test]
fn the_corners_are_mirror_symmetric() {
    let (fixture, built) = build_the_rim();
    let mesh = built.mesh.as_ref().expect("a filled mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    let at = mesh.vertices();
    assert!(
        !sweep_mesh_measures::intersects_itself_where_the_source_did_not(
            at,
            &kept,
            &fixture.positions,
            &fixture.indices
        ),
        "self-intersection"
    );

    // Each added vertex once, in the order the kept triangles first reach it.
    let mut added: Vec<i32> = Vec::new();
    for &(a, b, c) in &kept {
        for v in [a, b, c] {
            if v >= fixture.positions.len() as i32 && !added.contains(&v) {
                added.push(v);
            }
        }
    }

    let mut off_mirror: Vec<(i32, f64)> = added
        .iter()
        .map(|&v| {
            let p = at[v as usize];
            (
                v,
                distance_to_mesh(Vector3::new(p.x, p.y, 20.0 - p.z), at, &kept),
            )
        })
        .collect();
    // OrderByDescending is stable.
    off_mirror.sort_by(|a, b| b.1.total_cmp(&a.1));
    let report = off_mirror
        .iter()
        .take(5)
        .map(|&(v, d)| {
            let p = at[v as usize];
            format!("{v} ({:.3},{:.3},{:.3}) mirror {d:.3e} off", p.x, p.y, p.z)
        })
        .collect::<Vec<_>>()
        .join("; ");
    assert!(off_mirror[0].1 <= CONTROL_TOLERANCE, "{report}");
}

/// At the front corners v0 and v2 the round along 2 - 0 (axis x = -18, y = -113.672, radius 2)
/// runs on to the bottom and top faces, which square its end, and meets them on its foot arc: the
/// arc from the wall rail (-16.586, -115.086) round to where the other round's tangent line
/// x = -18 crosses it (-18, -115.672). The mitre patch laid one straight edge across that arc and
/// stood 0.073 inside it at v2.
#[test]
fn the_front_corners_meet_the_end_faces_on_the_rounds_foot() {
    let (_, built) = build_the_rim();
    let mesh = built.mesh.as_ref().expect("a filled mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    let at = mesh.vertices();
    let mut worst = (0.0, Vector3::ZERO);
    for z in [0.0, 20.0] {
        for k in 0..=20 {
            let angle =
                (-std::f64::consts::PI / 4.0) - (k as f64 / 20.0 * std::f64::consts::PI / 4.0);
            let q = Vector3::new(
                -18.0 + (2.0 * angle.cos()),
                -113.672 + (2.0 * angle.sin()),
                z,
            );
            let d = distance_to_mesh(q, at, &kept);
            if d > worst.0 {
                worst = (d, q);
            }
        }
    }

    assert!(
        worst.0 <= CONTROL_TOLERANCE,
        "({:.3},{:.3},{:.3}) is {:.3e} off the build",
        worst.1.x,
        worst.1.y,
        worst.1.z,
        worst.0
    );
}

fn build_the_rim() -> (Rc<LevelLineFixture>, GapFillResult) {
    let fixture = Rc::new(LevelLineFixture::load("center-right-and-left-side-op0"));
    let profile = if fixture.chamfer {
        BandProfile::Chamfer
    } else {
        BandProfile::Fillet
    };
    let options = BandGapBuildOptions {
        chord_tolerance: 0.0,
        snap: 1e-9,
        known_room: None,
        seam_epsilon: fixture.seam_epsilon,
        sharp_turn_degrees: fixture.sharp_turn_degrees,
        fillet_segments: fixture.segments,
        ..BandGapBuildOptions::default()
    };
    let features = Rc::clone(&fixture);
    let gap = band_gap_entry_points::build_pick(
        &fixture.positions,
        &fixture.indices,
        &[fixture.chains[1].clone()],
        &[fixture.closed[1]],
        Some(Rc::new(move |a, b| features.is_feature_edge(a, b))),
        fixture.size,
        profile,
        &options,
    )
    .expect("the pick is well formed");
    let built = gap_fill::fill(&gap, profile, fixture.segments).expect("the fill runs");
    assert_eq!(
        built.refusal,
        GapFillRefusal::None,
        "the rim alone refused {:?}",
        built.refusal
    );

    (fixture, built)
}
