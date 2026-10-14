//! Combination rows of `BevelLevelLineCombinationTests.EveryMemberBuildingAloneBuildsTogether` that
//! build with flat ends off and left the part open with them on (plan stage 2), dumped from the
//! app's own hand-over. Parameter rows are a loop.

use std::collections::HashSet;
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_in_turn::{
    build_open_chains_in_turn, takes_open_chains_in_turn,
};
use geometry_processing::bevel::level_line::open_gap_fill::ends_site;
use geometry_processing::bevel::level_line::GapFillRefusal;

use crate::end_face_builds::{as_saved, build_chains, feature_of, FlatEnds};
use crate::end_face_measures::to_triangle;
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;
use crate::round_mouth_fillet_crossings::crossing_pairs;
use crate::sweep_mesh_measures;

#[test]
fn builds_closed_on_the_source_with_flat_ends() {
    for name in ["center-flows-through-c9-c27-r005", "fifteen-mm-f4-c27-r005"] {
        let _flat_ends = FlatEnds::set(true);
        let fixture = Rc::new(LevelLineFixture::load(name));
        let size = fixture.size;
        let all: Vec<usize> = (0..fixture.chains.len()).collect();
        let gap = build_chains(
            &fixture,
            &all,
            size,
            &as_saved(&fixture, fixture.known_room),
        );
        assert!(
            gap.built(),
            "{name}: band {:?}, room {}, set by {:?}",
            gap.refusal,
            gap.room,
            gap.room_set_by
        );

        // As the app route takes a combination: a chain at a time, each as it is alone, on the
        // part the chains before it left (BevelSurfacePartBuilder.LevelLine's TheChainsInTurn).
        let filled = build_open_chains_in_turn(
            &fixture.positions,
            &fixture.indices,
            &fixture.chains,
            Some(feature_of(&fixture)),
            size,
            profile_of(&fixture),
            fixture.segments,
            if takes_open_chains_in_turn(Some(&gap), &fixture.closed) {
                Some(gap.room)
            } else {
                None
            },
            1e-9,
            fixture.seam_epsilon,
            fixture.sharp_turn_degrees,
            None,
            Some(&fixture.closed),
        )
        .expect("the build runs");
        let mesh = filled.mesh.as_ref().map(|m| m.borrow());
        let at = |v: i32| match &mesh {
            Some(mesh) if v >= 0 => format!("{:?}", mesh.vertices()[v as usize]),
            _ => "-".to_string(),
        };
        assert_eq!(
            filled.refusal,
            GapFillRefusal::None,
            "{name}: chain {} at {}: {:?}, open edge {:?} {} - {}",
            filled.offending_chain,
            at(filled.offending_vertex),
            ends_site(),
            filled.open_edge,
            at(filled.open_edge.0),
            at(filled.open_edge.1)
        );
        let mesh = mesh.expect("the filled mesh");
        let kept = sweep_mesh_measures::kept(&mesh);
        assert!(sweep_mesh_measures::is_closed_and_consistent(
            mesh.vertices(),
            &kept
        ));
        assert_eq!(
            sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
            1
        );
        assert_eq!(crossing_pairs(&mesh), 0, "{name}");

        // The check the app's route makes before it keeps a build: the rail is cut back to its
        // corner (EndFaceFill.RailsToTheClipCorner), so the strip's last rows are not laid flat
        // in the plane and capped over - a coplanar fold the app dropped.
        assert!(
            !sweep_mesh_measures::intersects_itself_where_the_source_did_not(
                mesh.vertices(),
                &kept,
                &fixture.positions,
                &fixture.indices
            ),
            "{name}: the app would drop the build for the existing construction"
        );

        // A fillet stands at most r(1 - 1/sqrt 3) = 0.42 r off the faces it joins; half r is past
        // every blend. Both parts also round the vertical 45-degree crease at (-20, -118.5), whose
        // strip middle legitimately stands r(1 - sin 22.5 deg) = 0.617 r off both faces, so that
        // derived reach is the bar here, less the fixture's single-precision coordinates.
        let used: HashSet<i32> = kept.iter().flat_map(|t| [t.0, t.1, t.2]).collect();
        let at = |v: i32| fixture.positions[v as usize];
        let worst = used
            .iter()
            .map(|&v| {
                fixture
                    .indices
                    .iter()
                    .map(|t| to_triangle(mesh.vertices()[v as usize], at(t.0), at(t.1), at(t.2)))
                    .fold(f64::INFINITY, f64::min)
            })
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(
            worst <= (size * (1.0 - (std::f64::consts::PI / 8.0).sin())) + 1e-6,
            "{name}: worst {worst}"
        );
    }
}
