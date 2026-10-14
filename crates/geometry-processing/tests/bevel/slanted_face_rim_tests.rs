//! The centre feature's
//! one-facet slanted face f0 of "Bottom Center Edge.mcx", picked as a face so its rim is a closed
//! chain of three sharp corners, and the two-facet face f1 above it. At 0.05 both must build
//! closed, in one shell, without crossing (they guard the rail turn-back and the
//! bulging-fill check).

use std::collections::HashSet;
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::BandGapBuildOptions;
use geometry_processing::bevel::level_line::{gap_fill, GapFillRefusal};

use crate::end_face_builds::{as_saved, build_chains, default_budget};
use crate::end_face_measures::to_triangle;
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;
use crate::round_mouth_fillet_crossings::crossing_pairs;
use crate::sweep_mesh_measures::{is_closed_and_consistent, kept, shell_count};

/// Both tests: build the fixture as saved on the default budget, fill, and hold the result to
/// a closed, consistent, one-shell solid that stands no more than half r off the input.
fn builds_closed_without_crossing(name: &str) {
    let fixture = Rc::new(LevelLineFixture::load(name));
    let size = fixture.size;
    let all: Vec<usize> = (0..fixture.chains.len()).collect();
    let gap = build_chains(
        &fixture,
        &all,
        size,
        &BandGapBuildOptions {
            budget: Some(default_budget(&fixture, false)),
            ..as_saved(&fixture, fixture.known_room)
        },
    );
    assert!(
        gap.built(),
        "band {:?}, room {}, set by {:?}",
        gap.refusal,
        gap.room,
        gap.room_set_by
    );

    let filled = gap_fill::fill(&gap, profile_of(&fixture), fixture.segments)
        .expect("the fill fills or refuses by name");
    let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    let offending = if filled.offending_vertex >= 0 {
        format!("{:?}", mesh.position(filled.offending_vertex))
    } else {
        "none".to_string()
    };
    assert_eq!(
        filled.refusal,
        GapFillRefusal::None,
        "offending {offending}"
    );
    assert_eq!(crossing_pairs(&mesh), 0);

    let kept = kept(&mesh);
    assert!(is_closed_and_consistent(&mesh.vertices, &kept));
    assert_eq!(shell_count(mesh.vertices.len() as i32, &kept), 1);

    // A fillet stands at most r(1 - 1/sqrt 3) = 0.42 r off the faces it joins; half r is past every
    // blend.
    let used: HashSet<i32> = kept.iter().flat_map(|t| [t.0, t.1, t.2]).collect();
    let worst = used
        .iter()
        .map(|&v| {
            fixture
                .indices
                .iter()
                .map(|t| {
                    to_triangle(
                        mesh.vertices[v as usize],
                        fixture.positions[t.0 as usize],
                        fixture.positions[t.1 as usize],
                        fixture.positions[t.2 as usize],
                    )
                })
                .fold(f64::INFINITY, f64::min)
        })
        .fold(f64::NEG_INFINITY, f64::max);
    assert!(worst <= size * 0.5);
}

#[test]
fn f0_builds_closed_without_crossing() {
    builds_closed_without_crossing("bottom-center-edge-f0-r005");
}

#[test]
fn f1_builds_closed_without_crossing() {
    builds_closed_without_crossing("bottom-center-edge-f1-r005");
}
