//! Port of `LevelLine/FadeEndRowFoldTests.cs`: Many Holes single-chain fillet picks at 0.25 that
//! the Boolean kernel read as self-intersecting. Each pick is ONE facet edge of a crease that
//! carries on, so the fill fades out past both ends; the rows past the chain's end shared one end
//! vertex on ONE tangent and folded in single precision. c904, c1915 and c3066 meet at the crease
//! and lay those rows straight; c563 refuses by size at 0.25 and is built at 0.9 of its room.
//! C# `[Arguments]` rows are a loop over the same-named test.
//!
//! Not ported, `[Skip]` in C# (measured 2026-09-25, the cause is in the band refinement, not the
//! fill): `SliverFanRootsBuildClean` (`issue-single-edge-c3243-r025`).

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::{gap_fill, open_gap_fill};
use geometry_processing::bevel::level_line::{
    BandGapResult, BandProfile, GapFillRefusal, WorkBudget,
};
use polygon_mesh::csg::boolean_processing::BooleanOperandVerdict;

use crate::boolean_kernel_gate;
use crate::level_line_fixture::LevelLineFixture;
use crate::round_mouth_fillet_crossings::crossing_pairs;
use crate::sweep_mesh_measures;

#[test]
fn fade_builds_clean() {
    for (name, at_the_search_size) in [
        ("many-holes-c904-r025", false),
        ("many-holes-c3066-r025", false),
        ("many-holes-c1915-r025", false),
    ] {
        builds_clean(name, at_the_search_size);
    }
}

#[test]
fn fade_at_the_search_size_builds_clean() {
    builds_clean("many-holes-c563-r025", true);
}

fn profile_of(fixture: &LevelLineFixture) -> BandProfile {
    if fixture.chamfer {
        BandProfile::Chamfer
    } else {
        BandProfile::Fillet
    }
}

fn builds_clean(name: &str, at_the_search_size: bool) {
    let fixture = Rc::new(LevelLineFixture::load(name));
    let mut gap = band(&fixture, fixture.size, fixture.known_room);
    if at_the_search_size {
        // The app refuses the dumped size by room and searches down; the crossing answer is that
        // build.
        assert!(!gap.built(), "{name}");
        gap = band(&fixture, gap.room * 0.9, None);
    }

    assert!(
        gap.built(),
        "{name}: band {:?}, room {}, set by {:?}",
        gap.refusal,
        gap.room,
        gap.room_set_by
    );

    let filled =
        gap_fill::fill(&gap, profile_of(&fixture), fixture.segments).expect("the fill runs");
    assert_eq!(
        filled.refusal,
        GapFillRefusal::None,
        "{name}: {}",
        open_gap_fill::ends_site().unwrap_or_default()
    );

    let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    assert!(
        sweep_mesh_measures::is_closed_and_consistent(mesh.vertices(), &kept),
        "{name}"
    );
    assert_eq!(
        sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
        1,
        "{name}"
    );
    assert_eq!(crossing_pairs(&mesh), 0, "{name}");

    // The app's own gate, which crossing_pairs alone does not stand in for (see BooleanKernelGate).
    assert_eq!(
        boolean_kernel_gate::verdict(&mesh),
        BooleanOperandVerdict::Clean,
        "{name}"
    );
}

fn band(fixture: &Rc<LevelLineFixture>, size: f64, known_room: Option<f64>) -> BandGapResult {
    let features = Rc::clone(fixture);
    let options = BandGapBuildOptions {
        chord_tolerance: 0.0,
        snap: 1e-9,
        known_room,
        budget: Some(Rc::new(
            WorkBudget::new(WorkBudget::default_limit(fixture.indices.len() as i32))
                .expect("a positive limit"),
        )),
        seam_epsilon: fixture.seam_epsilon,
        sharp_turn_degrees: fixture.sharp_turn_degrees,
        fillet_segments: fixture.segments,
    };
    band_gap_entry_points::build_pick(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        &fixture.closed,
        Some(Rc::new(move |a, b| features.is_feature_edge(a, b))),
        size,
        profile_of(fixture),
        &options,
    )
    .expect("the pick is well formed")
}
