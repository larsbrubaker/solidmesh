//! Port of `LevelLine/CubeEdgeEndCapTests.cs`: stage 2, rung 6, the planar END CAP. One top edge of
//! a cube of edge 2 ends on two end faces; the strip fills between the side runs and each end's
//! hole is zipped by angle about the end vertex, so the cap lies in the end face. C# `[Arguments]`
//! cases run as one loop per test.

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::band_gap_reading;
use geometry_processing::bevel::level_line::seam_edges::FeatureEdgeTest;
use geometry_processing::bevel::level_line::{
    chain_on_the_mesh, gap_fill, BandGapRefusal, BandGapResult, BandProfile, GapFillRefusal,
    GapFillResult,
};
use vector_math::{Matrix4X4, Vector3};

use crate::hole_fill_corner_tests::smallest_kept_area;
use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_test_geometry::{cube_solid, signed_volume};
use crate::sweep_mesh_measures;

const CUBE_VOLUME: f64 = 8.0;

const EDGE_LENGTH: f64 = 2.0;

const CHAIN: [i32; 2] = [4, 5];

/// C# `ACubeEdgeFillsClosedWithItsCapsInTheEndFaces`.
#[test]
fn a_cube_edge_fills_closed_with_its_caps_in_the_end_faces() {
    for (profile, fraction) in [
        (BandProfile::Chamfer, 0.1),
        (BandProfile::Chamfer, 0.5),
        (BandProfile::Chamfer, 0.9),
        (BandProfile::Chamfer, 1.0),
        (BandProfile::Fillet, 0.1),
        (BandProfile::Fillet, 0.5),
        (BandProfile::Fillet, 0.9),
        (BandProfile::Fillet, 1.0),
    ] {
        let room = room(profile);
        let size = fraction * room;
        let filled = fill(size, None, profile);
        assert_eq!(
            filled.refusal,
            GapFillRefusal::None,
            "{profile:?} at {fraction} of the room {room}"
        );

        let mesh = filled.mesh.as_ref().unwrap().borrow();
        let kept = sweep_mesh_measures::kept(&mesh);
        assert!(sweep_mesh_measures::is_closed_and_consistent(
            mesh.vertices(),
            &kept
        ));
        assert_eq!(
            sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
            1
        );
        let smallest_area = smallest_kept_area(&mesh, &kept);
        // No FILL triangle has no area at any size. At 1.0 of the room the KEPT surface does: the
        // room reads 2e-9 under the face width 2, so the cut roots the side and end lines about
        // 1e-9 from the cube's far corners - outside the 1e-9 snap - and leaves input slivers there.
        let added: Vec<(i32, i32, i32)> = filled
            .added_triangles
            .iter()
            .map(|&t| {
                let tri = mesh.triangles()[t as usize];
                (tri.a, tri.b, tri.c)
            })
            .collect();
        let smallest_fill = smallest_kept_area(&mesh, &added);
        assert!(smallest_fill > 1e-12, "{profile:?} at {fraction}");
        if fraction < 1.0 {
            assert!(smallest_area > 1e-12, "{profile:?} at {fraction}");
        }

        // Convex: a fill that only removes leaves every new vertex inside or on the cube.
        for &v in &filled.added_vertices {
            let p = mesh.position(v);
            assert!(
                p.x.abs().max(p.y.abs().max(p.z.abs())) <= 1.0 + 1e-9,
                "added vertex {v} at {p:?}"
            );
        }

        // The cap's vertices lie in the end faces x = -1 and x = 1: every fill triangle facing
        // along x near an end face is ON it.
        let mut residual: f64 = 0.0;
        for &t in &filled.added_triangles {
            let face = mesh.triangles()[t as usize];
            let corners = [face.a, face.b, face.c].map(|v| mesh.position(v));
            for end in [-1.0, 1.0] {
                let normal = (corners[1] - corners[0])
                    .cross(corners[2] - corners[0])
                    .get_normal();
                if (normal.x.abs() - 1.0).abs() < 1e-6
                    && corners.iter().all(|c| (c.x - end).abs() < 1e-3)
                {
                    residual = residual.max(
                        corners
                            .iter()
                            .map(|c| (c.x - end).abs())
                            .fold(f64::NEG_INFINITY, f64::max),
                    );
                }
            }
        }

        assert!(residual <= 1e-9, "cap residual {residual}");

        let volume = signed_volume(&mesh);
        let removed = CUBE_VOLUME - volume;
        let (positions, indices) = cube_solid(1.0);
        let setback = size
            * chain_on_the_mesh::unit_setbacks(
                &positions,
                &indices,
                &CHAIN,
                false,
                &chain_on_the_mesh::chain_edges(&CHAIN, false),
                profile,
            )
            .expect("the chain stands on two triangles")[0];
        let prism = EDGE_LENGTH * setback * setback / 2.0;
        println!(
            "{profile:?} at {fraction} of {room}: removed {removed}, prism {prism}, relative {}, cap residual {residual}, smallest area {smallest_area}",
            (removed - prism) / prism
        );
        if profile == BandProfile::Chamfer {
            assert!(
                (removed - prism).abs() / prism <= 1e-9,
                "{profile:?} at {fraction}"
            );
        } else {
            assert!(removed > 0.0);
            assert!(removed < prism);
        }

        let motion = Matrix4X4::create_rotation(Vector3::new(0.3, -0.7, 1.1))
            * Matrix4X4::create_translation(11.5, -3.25, 7.125);
        let moved = fill(size, Some(motion), profile);
        assert_eq!(
            moved.refusal,
            GapFillRefusal::None,
            "moved: {:?} {} {:?}",
            moved.refusal,
            moved.offending_vertex,
            moved.open_edge
        );
        let moved_volume = signed_volume(&moved.mesh.as_ref().unwrap().borrow());
        assert!(
            (moved_volume - volume).abs() / volume <= 1e-9,
            "{profile:?} at {fraction}"
        );
    }
}

/// C# `OverTheRoomRefusesBySizeNamingIt`.
#[test]
fn over_the_room_refuses_by_size_naming_it() {
    for profile in [BandProfile::Chamfer, BandProfile::Fillet] {
        let room = room(profile);
        let over = build(&positions(None), 1.1 * room, profile, feature(None));
        assert_eq!(over.refusal, BandGapRefusal::Size);
        assert_eq!(over.room, room);
    }
}

fn indices() -> Vec<(i32, i32, i32)> {
    cube_solid(1.0).1
}

fn positions(motion: Option<Matrix4X4>) -> Vec<Vector3> {
    let positions = cube_solid(1.0).0;
    match motion {
        Some(motion) => positions.iter().map(|p| p.transform(motion)).collect(),
        None => positions,
    }
}

fn feature(motion: Option<Matrix4X4>) -> FeatureEdgeTest {
    feature_edges_over(&positions(motion), &indices(), 10.0)
}

fn room(profile: BandProfile) -> f64 {
    band_gap_reading::room_of(
        &positions(None),
        &indices(),
        &CHAIN,
        false,
        Some(feature(None)),
        profile,
        1e-9,
    )
    .expect("the room reads")
}

fn build(
    positions: &[Vector3],
    size: f64,
    profile: BandProfile,
    feature: FeatureEdgeTest,
) -> BandGapResult {
    band_gap_entry_points::build(
        positions,
        &indices(),
        &CHAIN,
        false,
        Some(Rc::clone(&feature)),
        size,
        profile,
        &BandGapBuildOptions::default(),
    )
    .expect("the pick builds or refuses by name")
}

fn fill(size: f64, motion: Option<Matrix4X4>, profile: BandProfile) -> GapFillResult {
    let gap = build(&positions(motion), size, profile, feature(motion));
    assert!(
        gap.built(),
        "the cube edge refused at {size}: {:?}",
        gap.refusal
    );

    gap_fill::fill(&gap, profile, gap_fill::DEFAULT_FILLET_SEGMENTS).expect("the fill")
}
