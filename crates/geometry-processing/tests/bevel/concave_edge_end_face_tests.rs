//! Stage 2 rung 7 - one CONCAVE edge of a square pocket's floor ends on two end faces (the
//! neighbouring pocket walls x = -1 and x = 1). A concave bevel ADDS material, so each end's hole
//! lies in that end wall's plane and is zipped there as the convex cap is. Each test's cases run as
//! a loop inside it.

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::band_gap_reading::room_of;
use geometry_processing::bevel::level_line::chain_on_the_mesh::{chain_edges, unit_setbacks};
use geometry_processing::bevel::level_line::gap_fill;
use geometry_processing::bevel::level_line::{BandProfile, GapFillRefusal, GapFillResult};
use vector_math::{Matrix4X4, Vector3};

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_test_geometry::{signed_volume, signed_volume_of};
use crate::level_line_test_solids::square_pocket;
use crate::sweep_mesh_measures;

const FOLD_DEGREES: f64 = 20.0;

const EDGE_LENGTH: f64 = 2.0;

fn chain() -> Vec<i32> {
    let (_, _, rim) = square_pocket(2.0, 1.0, 1.0, 0.0);
    vec![rim[0], rim[1]]
}

fn indices() -> Vec<(i32, i32, i32)> {
    square_pocket(2.0, 1.0, 1.0, 0.0).1
}

fn positions(motion: Option<Matrix4X4>) -> Vec<Vector3> {
    let positions = square_pocket(2.0, 1.0, 1.0, 0.0).0;
    match motion {
        Some(motion) => positions.into_iter().map(|p| p.transform(motion)).collect(),
        None => positions,
    }
}

fn fill(size: f64, motion: Option<Matrix4X4>, profile: BandProfile) -> GapFillResult {
    let positions = positions(motion);
    let indices = indices();
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
    let gap = band_gap_entry_points::build(
        &positions,
        &indices,
        &chain(),
        false,
        Some(feature),
        size,
        profile,
        &BandGapBuildOptions::default(),
    )
    .expect("a valid pick");
    assert!(
        gap.built(),
        "the pocket floor edge refused at {size}: {:?}",
        gap.refusal
    );
    gap_fill::fill(&gap, profile, gap_fill::DEFAULT_FILLET_SEGMENTS).expect("the fill runs")
}

#[test]
fn a_concave_floor_edge_ending_on_pocket_walls_fills_closed_with_its_caps_in_the_end_walls() {
    for profile in [BandProfile::Chamfer, BandProfile::Fillet] {
        for fraction in [0.1, 0.5, 0.9] {
            case(profile, fraction);
        }
    }
}

fn case(profile: BandProfile, fraction: f64) {
    let source_positions = positions(None);
    let source_indices = indices();
    let room = room_of(
        &source_positions,
        &source_indices,
        &chain(),
        false,
        Some(feature_edges_over(
            &source_positions,
            &source_indices,
            FOLD_DEGREES,
        )),
        profile,
        1e-9,
    )
    .expect("the room reads");
    let size = fraction * room;
    let filled = fill(size, None, profile);
    assert_eq!(
        filled.refusal,
        GapFillRefusal::None,
        "{profile:?} at {fraction} of the room {room}: {} {:?}",
        filled.offending_vertex,
        filled.open_edge
    );

    let mesh = filled.mesh.as_ref().expect("the filled mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    assert!(sweep_mesh_measures::is_closed_and_consistent(
        mesh.vertices(),
        &kept
    ));
    assert_eq!(
        sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
        1
    );
    let smallest_fill = filled
        .added_triangles
        .iter()
        .map(|&t| mesh.triangles()[t as usize])
        .map(|t| {
            (mesh.position(t.b) - mesh.position(t.a))
                .cross(mesh.position(t.c) - mesh.position(t.a))
                .length()
                / 2.0
        })
        .fold(f64::INFINITY, f64::min);
    assert!(smallest_fill > 1e-12, "{smallest_fill}");

    // The caps lie in the end walls x = -1 and x = 1: every fill triangle facing along x near an
    // end wall is ON it, and wound as the input - the wall at x = -1 faces +x into the pocket, the
    // one at x = 1 -x.
    let mut residual: f64 = 0.0;
    let mut cap_triangles = 0;
    for &t in &filled.added_triangles {
        let face = mesh.triangles()[t as usize];
        let corners: Vec<Vector3> = [face.a, face.b, face.c]
            .iter()
            .map(|&v| mesh.position(v))
            .collect();
        let normal = (corners[1] - corners[0])
            .cross(corners[2] - corners[0])
            .get_normal();
        for end in [-1.0, 1.0] {
            if (normal.x.abs() - 1.0).abs() < 1e-6
                && corners.iter().all(|c| (c.x - end).abs() < 1e-3)
            {
                cap_triangles += 1;
                residual = residual.max(
                    corners
                        .iter()
                        .map(|c| (c.x - end).abs())
                        .fold(f64::NEG_INFINITY, f64::max),
                );
                assert!(
                    normal.x * end < 0.0,
                    "cap triangle {t} at x = {end} faces {normal:?}"
                );
            }
        }
    }

    assert!(cap_triangles > 0);
    assert!(residual <= 1e-9, "cap residual {residual}");

    // Concave: the blend ADDS material - the chamfer exactly its prism, the fillet less than the
    // chamfer's.
    let source = signed_volume_of(&source_positions, &source_indices);
    let volume = signed_volume(&mesh);
    let added = volume - source;
    let setback = size
        * unit_setbacks(
            &source_positions,
            &source_indices,
            &chain(),
            false,
            &chain_edges(&chain(), false),
            profile,
        )
        .expect("the chain is on two triangles")[0];
    let chamfer_prism = EDGE_LENGTH * setback * setback / 2.0;
    let prism = if profile == BandProfile::Chamfer {
        chamfer_prism
    } else {
        EDGE_LENGTH * setback * setback * (1.0 - (std::f64::consts::PI / 4.0))
    };
    println!(
        "{profile:?} at {fraction} of {room}: size {size}, added {added}, prism {prism}, relative {}, cap residual {residual}, cap triangles {cap_triangles}",
        (added - prism) / prism
    );
    assert!(added > 0.0);
    if profile == BandProfile::Chamfer {
        assert!(
            (added - prism).abs() / prism <= 1e-9,
            "added {added} prism {prism}"
        );
    } else {
        // The polygonal arc chords inside the true fillet's concave side, so it may add a little
        // more than the exact fillet prism, but never as much as the chamfer.
        assert!(
            added < chamfer_prism,
            "added {added} chamfer {chamfer_prism}"
        );
    }

    let motion = Matrix4X4::create_rotation(Vector3::new(0.3, -0.7, 1.1))
        * Matrix4X4::create_translation_vector(Vector3::new(11.5, -3.25, 7.125));
    let moved = fill(size, Some(motion), profile);
    assert_eq!(
        moved.refusal,
        GapFillRefusal::None,
        "moved: {} {:?}",
        moved.offending_vertex,
        moved.open_edge
    );
    let moved_volume = signed_volume(&moved.mesh.as_ref().expect("the moved mesh").borrow());
    assert!(
        (moved_volume - volume).abs() / volume <= 1e-9,
        "moved {moved_volume} against {volume}"
    );
}
