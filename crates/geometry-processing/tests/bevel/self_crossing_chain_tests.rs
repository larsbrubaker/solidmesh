//! Single-chain fillet picks at 0.05 that Boolean
//! builds clean but the level line refused in the sweep with "the level line's mesh intersects
//! itself" (`BooleanKernelGate` reads the answer SelfIntersecting). Each builds as the app takes a
//! single chain: the band, then the gap fill, flat ends at their default. Two (c7, c17) were a
//! coplanar fold of the end face's flat fill (`FlatHoleFill`); the other three are an end cap laid
//! IN the end face's plane over that face's own kept triangles - c26 because the end-plane trim left
//! one rail uncut, c27 and its mirror c16 because a concave chain's same-sense end beside a hull end
//! was grown rather than trimmed. All five build now.

use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::BandGapBuildOptions;
use solidmesh::bevel::level_line::gap_fill;
use solidmesh::bevel::level_line::open_gap_fill::ends_site;
use solidmesh::bevel::level_line::GapFillRefusal;
use solidmesh_mesh::csg::boolean_processing::BooleanOperandVerdict;

use crate::boolean_kernel_gate;
use crate::end_face_builds::{as_saved, build_chains, default_budget};
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;
use crate::round_mouth_fillet_crossings::crossing_pairs;
use crate::sweep_mesh_measures;

#[test]
fn center_and_right_face_c26_builds_clean() {
    builds_clean("center-and-right-face-c26-r005");
}

#[test]
fn inner_face_should_bevel_c7_builds_clean() {
    builds_clean("inner-face-should-bevel-c7-r005");
}

#[test]
fn chain_and_face_c17_builds_clean() {
    builds_clean("chain-and-face-c17-r005");
}

#[test]
fn goes_to_sides_c16_builds_clean() {
    builds_clean("goes-to-sides-c16-r005");
}

#[test]
fn goes_to_sides_c27_builds_clean() {
    builds_clean("goes-to-sides-c27-r005");
}

fn builds_clean(name: &str) {
    let fixture = Rc::new(LevelLineFixture::load(name));
    let options = BandGapBuildOptions {
        budget: Some(default_budget(&fixture, false)),
        ..as_saved(&fixture, fixture.known_room)
    };
    let all: Vec<usize> = (0..fixture.chains.len()).collect();
    let gap = build_chains(&fixture, &all, fixture.size, &options);
    assert!(
        gap.built(),
        "band {:?}, room {}, set by {:?}",
        gap.refusal,
        gap.room,
        gap.room_set_by
    );

    let filled =
        gap_fill::fill(&gap, profile_of(&fixture), fixture.segments).expect("the fill runs");
    assert_eq!(
        filled.refusal,
        GapFillRefusal::None,
        "{}",
        ends_site().unwrap_or_default()
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
    assert_eq!(crossing_pairs(&mesh), 0);

    // The app's own gate, which CrossingPairs alone does not stand in for (see BooleanKernelGate).
    assert_eq!(
        boolean_kernel_gate::verdict(&mesh),
        BooleanOperandVerdict::Clean
    );
}
