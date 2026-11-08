//! Concave open chains from the catalogue whose END FACE is reflex at the chain's end vertex V -
//! the inner corner of an L block. The cut there is the level line round V, running the far way
//! round it through the material, so the end hole SURROUNDS V; the hole is fanned to V. Each chain
//! is filled alone, at 0.9 of its room where the saved size does not fit. Each test's cases run as
//! a loop inside it.

use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::{
    gap_fill, BandGapRefusal, BandGapResult, GapFillRefusal, GapFillResult,
};
use solidmesh_math::{Matrix4X4, Vector3};

use crate::end_face_builds::feature_of;
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;
use crate::level_line_test_geometry::{signed_volume, signed_volume_of};
use crate::sweep_mesh_measures;

/// A reflex end face fans its cap about the end vertex.
#[test]
fn a_reflex_end_face_fans_its_cap_about_the_end_vertex() {
    let rows: [(&str, usize, f64, f64); 3] = [
        ("simple-internal-fillet-op0", 0, 1.0, 1e-9),
        ("center-flows-through-op0", 1, 1.0, 1e-9),
        // The source's end face itself is flat only to float noise - its vertices sit up to 1.7e-6
        // off the ideal planes (x = 1.7e-6 where 0 was meant) - so the cap is held to that, not to
        // 1e-9.
        ("center-flows-through-op0", 0, 1.0, 1e-5),
    ];
    for (name, chain, sense, plane_tolerance) in rows {
        a_reflex_end_face_fans_its_cap_about_the_end_vertex_row(
            name,
            chain,
            sense,
            plane_tolerance,
        );
    }
}

fn a_reflex_end_face_fans_its_cap_about_the_end_vertex_row(
    name: &str,
    chain: usize,
    sense: f64,
    plane_tolerance: f64,
) {
    let fixture = Rc::new(LevelLineFixture::load(name));
    let size = size_that_builds(&fixture, chain, None);
    let filled = fill(&fixture, chain, size, None);
    assert_eq!(
        filled.refusal,
        GapFillRefusal::None,
        "{name} at {size}: {} {:?}",
        filled.offending_vertex,
        filled.open_edge
    );

    let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    assert!(sweep_mesh_measures::is_closed_and_consistent(
        mesh.vertices(),
        &kept
    ));
    assert_eq!(
        sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
        1
    );

    // A concave blend ADDS material, a convex one removes it.
    let source = signed_volume_of(&fixture.positions, &fixture.indices);
    let volume = signed_volume(&mesh);
    println!(
        "{name}: size {size}, source {source}, filled {volume}, added {}",
        volume - source
    );
    assert!(sense * (volume - source) > 0.0, "{name}");

    // Each cap fanned to V lies in the end face's plane: every triangle with a corner AT a chain
    // end is measured against the plane of the first such triangle at that end.
    let picked = &fixture.chains[chain];
    let ends = [picked[0], picked[picked.len() - 1]].map(|i| fixture.positions[i as usize]);
    let mut fans = 0;
    let mut residual: f64 = 0.0;
    for end in ends {
        let triangles: Vec<[Vector3; 3]> = filled
            .added_triangles
            .iter()
            .map(|&t| mesh.triangles()[t as usize])
            .filter(|t| {
                [t.a, t.b, t.c]
                    .iter()
                    .any(|&i| (mesh.position(i) - end).length() < 1e-12)
            })
            .map(|t| [mesh.position(t.a), mesh.position(t.b), mesh.position(t.c)])
            .collect();
        if triangles.is_empty() {
            continue;
        }

        fans += 1;
        let normal = (triangles[0][1] - triangles[0][0])
            .cross(triangles[0][2] - triangles[0][0])
            .get_normal();
        let worst = triangles
            .iter()
            .flatten()
            .map(|&p| (p - end).dot(normal).abs())
            .fold(f64::NEG_INFINITY, f64::max);
        residual = residual.max(worst);
    }

    println!("{name}: {fans} fanned cap(s), residual {residual}");
    assert!(fans > 0, "{name}");
    assert!(residual <= plane_tolerance, "{name}: {residual}");

    let motion = Matrix4X4::create_rotation(Vector3::new(0.3, -0.7, 1.1))
        * Matrix4X4::create_translation(11.5, -3.25, 7.125);
    let moved = fill(&fixture, chain, size, Some(motion));
    assert_eq!(
        moved.refusal,
        GapFillRefusal::None,
        "moved: {} {:?}",
        moved.offending_vertex,
        moved.open_edge
    );
    let moved_volume = signed_volume(&moved.mesh.as_ref().expect("a filled mesh").borrow());
    assert!(
        (moved_volume - volume).abs() / volume <= 1e-9,
        "{name}: {moved_volume} against {volume}"
    );
}

/// UpperLeftInner's concave chain ends on its
/// hull, an end face of the OPPOSITE sense that is oblique to the chain; it reaches the growing
/// cap, which is not star-shaped about the end vertex on the hull and refuses there.
#[test]
fn an_oblique_opposite_sense_end_face_refuses_by_name() {
    let fixture = Rc::new(LevelLineFixture::load("upperleftinner-should-bevel-op0"));
    let size = size_that_builds(&fixture, 0, None);
    let gap = build(&fixture, 0, size, None);
    assert!(gap.built(), "{:?}, room {}", gap.refusal, gap.room);

    let filled =
        gap_fill::fill(&gap, profile_of(&fixture), segments_of(&fixture)).expect("the fill runs");
    assert_eq!(filled.refusal, GapFillRefusal::CornerCapNotStarShaped);
}

/// The fixture's own fillet segments, or 4 where it names none.
fn segments_of(fixture: &LevelLineFixture) -> i32 {
    if fixture.segments > 0 {
        fixture.segments
    } else {
        4
    }
}

fn positions(fixture: &LevelLineFixture, motion: Option<Matrix4X4>) -> Vec<Vector3> {
    match motion {
        Some(motion) => fixture
            .positions
            .iter()
            .map(|p| p.transform(motion))
            .collect(),
        None => fixture.positions.clone(),
    }
}

fn build(
    fixture: &Rc<LevelLineFixture>,
    chain: usize,
    size: f64,
    motion: Option<Matrix4X4>,
) -> BandGapResult {
    band_gap_entry_points::build_pick(
        &positions(fixture, motion),
        &fixture.indices,
        &[fixture.chains[chain].clone()],
        &[fixture.closed[chain]],
        Some(feature_of(fixture)),
        size,
        profile_of(fixture),
        &BandGapBuildOptions {
            chord_tolerance: 0.0,
            snap: 1e-9,
            ..BandGapBuildOptions::default()
        },
    )
    .expect("the pick builds or refuses by name")
}

fn size_that_builds(
    fixture: &Rc<LevelLineFixture>,
    chain: usize,
    motion: Option<Matrix4X4>,
) -> f64 {
    let gap = build(fixture, chain, fixture.size, motion);
    if gap.refusal == BandGapRefusal::Size {
        0.9 * gap.room
    } else {
        fixture.size
    }
}

fn fill(
    fixture: &Rc<LevelLineFixture>,
    chain: usize,
    size: f64,
    motion: Option<Matrix4X4>,
) -> GapFillResult {
    let gap = build(fixture, chain, size, motion);
    assert!(gap.built(), "the band refused at {size}: {:?}", gap.refusal);

    gap_fill::fill(&gap, profile_of(fixture), segments_of(fixture)).expect("the fill runs")
}
