//! Port of `LevelLine/MixedEndFadeTests.cs`: a MIXED open pick, "Top Face Should stil bevel.mcx"'s
//! c68 - one end cut into its end face and the other where the crease carries on past the pick.
//! In the app at the small sweep sizes the two side runs meet at one nose vertex on that end, and
//! the end-face fill refused OpenChainEnds until it took the nose as that end's run
//! (`EndFaceFill.TryEndFaces`). C# `[Arguments]` rows are loops over the same-named test.

use std::rc::Rc;

use geometry_processing::bevel::level_line::open_gap_fill::ends_site;
use geometry_processing::bevel::level_line::{gap_fill, BandGapResult, GapFillRefusal};

use crate::end_face_builds::{as_saved, build_chains};
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;
use crate::level_line_test_geometry::{signed_volume, signed_volume_of};
use crate::sweep_mesh_measures;

#[test]
fn one_end_on_its_face_the_other_fading_builds_closed() {
    for size in [0.05, 0.1] {
        let fixture = Rc::new(LevelLineFixture::load("top-face-mixed-end-c68"));

        // The app hands the build the room it read first (BevelSurfacePartBuilder.TheRoomOfThePick);
        // without it the build reads its own and the cut comes out differently at the fading end.
        let room = build(&fixture, fixture.size, None).room;
        let fixture = Rc::new(LevelLineFixture::load("top-face-mixed-end-c68"));
        let gap = build(&fixture, size, Some(room));
        assert!(
            gap.built(),
            "{:?} at {size:?} of room {room:?}",
            gap.refusal
        );

        let filled =
            gap_fill::fill(&gap, profile_of(&fixture), fixture.segments).expect("the fill runs");
        assert_eq!(
            filled.refusal,
            GapFillRefusal::None,
            "{:?} ({:?}) at {size:?} of room {room:?}",
            filled.refusal,
            ends_site()
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

        // A convex edge's round only takes material.
        let before = signed_volume_of(&fixture.positions, &fixture.indices);
        let volume = signed_volume(&mesh);
        assert!(volume < before, "{volume} against {before} at {size}");
        assert!(volume > 0.0);
    }
}

fn build(fixture: &Rc<LevelLineFixture>, size: f64, known_room: Option<f64>) -> BandGapResult {
    build_chains(
        fixture,
        &(0..fixture.chains.len()).collect::<Vec<_>>(),
        size,
        &as_saved(fixture, known_room),
    )
}
