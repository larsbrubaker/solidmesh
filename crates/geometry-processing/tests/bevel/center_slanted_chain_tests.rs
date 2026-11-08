//! The part fifteen fixture files share ("Center And Right Face.mcx" among them) - chains c12 and
//! c21 are the slanted short edges of the centre feature. Each chain ends where the feature's
//! corner meets a slanted base face that wraps round that corner: the base is a SIDE of the chain
//! and carries on past the end. Flat ends clip that band (a concave chain's ray onto an end face
//! facing on along it is bounded where a side's own face carries on past the end plane) and the
//! flat hole fill closes the end, so these rows hold the flag ON. c21 has no such row: alone it
//! builds the in-turn vertex close tests' answer, which the Boolean kernel reads as
//! self-intersecting.

use std::collections::HashSet;
use std::rc::Rc;

use solidmesh::bevel::level_line::open_gap_fill::ends_site;
use solidmesh::bevel::level_line::{gap_fill, GapFillRefusal};
use solidmesh_mesh::csg::boolean_processing::BooleanOperandVerdict;

use crate::boolean_kernel_gate;
use crate::end_face_builds::{as_saved, build_chains, FlatEnds};
use crate::end_face_measures::to_triangle;
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;
use crate::round_mouth_fillet_crossings::crossing_pairs;
use crate::sweep_mesh_measures;

/// c12 builds closed, without crossing, with flat ends.
#[test]
fn c12_builds_closed_without_crossing_with_flat_ends() {
    builds_closed_without_crossing("center-and-right-face-c12-r005");
}

/// The row with `CornerCap.UseFlatEndsOverride = true`.
fn builds_closed_without_crossing(name: &str) {
    let _flat = FlatEnds::set(true);
    builds_closed_without_crossing_as_is(name);
}

fn builds_closed_without_crossing_as_is(name: &str) {
    let fixture = Rc::new(LevelLineFixture::load(name));
    let size = fixture.size;
    let every: Vec<usize> = (0..fixture.chains.len()).collect();
    let gap = build_chains(
        &fixture,
        &every,
        size,
        &as_saved(&fixture, fixture.known_room),
    );
    assert!(
        gap.built(),
        "band {:?}, room {}, set by {:?}",
        gap.refusal,
        gap.room,
        gap.room_set_by
    );

    let profile = profile_of(&fixture);
    let filled = gap_fill::fill(&gap, profile, fixture.segments).expect("the fill runs");
    assert_eq!(
        filled.refusal,
        GapFillRefusal::None,
        "{}",
        ends_site().unwrap_or_default()
    );
    let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    assert_eq!(crossing_pairs(&mesh), 0);

    let kept = sweep_mesh_measures::kept(&mesh);
    assert!(sweep_mesh_measures::is_closed_and_consistent(
        mesh.vertices(),
        &kept
    ));
    assert_eq!(
        sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
        1
    );

    // A fillet stands at most r(1 - 1/sqrt 3) = 0.42 r off the faces it joins; half r is past
    // every blend.
    let used: HashSet<i32> = kept.iter().flat_map(|t| [t.0, t.1, t.2]).collect();
    let worst = used
        .iter()
        .map(|&v| {
            fixture
                .indices
                .iter()
                .map(|t| {
                    to_triangle(
                        mesh.vertices()[v as usize],
                        fixture.positions[t.0 as usize],
                        fixture.positions[t.1 as usize],
                        fixture.positions[t.2 as usize],
                    )
                })
                .fold(f64::INFINITY, f64::min)
        })
        .fold(f64::NEG_INFINITY, f64::max);
    assert!(worst <= size * 0.5, "{worst} against {size}");

    // The app's own gate, which CrossingPairs alone does not stand in for (see BooleanKernelGate).
    assert_eq!(
        boolean_kernel_gate::verdict(&mesh),
        BooleanOperandVerdict::Clean
    );
}
