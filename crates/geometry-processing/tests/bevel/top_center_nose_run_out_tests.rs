//! The nose's upper face of Top Center Face.mcx filleted at 1, handed straight to the kernel with
//! run-outs read (`RunOutField`). The band cut follows the unpicked lower crease to the control's
//! P* and the one corner hole runs down to it from both ridge ends; the build must still close as
//! one oriented shell with no zero-area triangle, reaching along that crease as far as the
//! control's blend.
//!
//! Run-outs are turned on for both the build and the fill; the switch rides on each one's budget,
//! so the build's budget is put in force before the scope turns it on, and the fill's default-sized
//! budget carries it too.

use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::BandGapBuildOptions;
use solidmesh::bevel::level_line::corner_cap_hole_fill::last_hole_fill_failure;
use solidmesh::bevel::level_line::{
    gap_fill, turn_run_outs_on, GapFillResult, LevelLineWork, WorkBudget,
};
use solidmesh_math::Vector3;

use crate::end_face_builds::{as_saved, build_chains, default_budget};
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;
use crate::sweep_mesh_measures::{is_closed_and_consistent, kept, shell_count};

/// Where the fixture's part sits relative to the regression test's box frame.
fn offset() -> Vector3 {
    Vector3::new(-20.0, -118.5, 0.0)
}

/// How far along the unpicked lower crease the control's blend reaches.
const CONTROL_RUN_OUT: f64 = 4.5846;

/// What two chordings of one surface may differ by.
const CHORD_TOLERANCE: f64 = 0.05;

/// The ridge round runs out along the unpicked lower crease like the control.
#[test]
fn the_ridge_round_runs_out_along_the_unpicked_lower_crease_like_the_control() {
    let fixture = Rc::new(LevelLineFixture::load("top-center-face-nose-r1"));
    let filled = {
        // Run-outs are gated off until R6 turns them on for the owner files; this build reads them.
        let budget = default_budget(&fixture, false);
        let gap = {
            let _work = LevelLineWork::begin(Some(Rc::clone(&budget)));
            let _run_outs = turn_run_outs_on();
            let all: Vec<usize> = (0..fixture.chains.len()).collect();
            build_chains(
                &fixture,
                &all,
                fixture.size,
                &BandGapBuildOptions {
                    budget: Some(Rc::clone(&budget)),
                    ..as_saved(&fixture, fixture.known_room)
                },
            )
        };
        assert!(gap.built(), "band {:?}", gap.refusal);

        let triangle_count = gap
            .mesh
            .as_ref()
            .map_or(0, |mesh| mesh.borrow().triangle_count());
        let fill_budget = Rc::new(
            WorkBudget::new(WorkBudget::default_limit(triangle_count)).expect("a positive limit"),
        );
        let _work = LevelLineWork::begin(Some(Rc::clone(&fill_budget)));
        let _run_outs = turn_run_outs_on();
        gap_fill::fill_with_budget(
            &gap,
            profile_of(&fixture),
            fixture.segments,
            Some(fill_budget),
        )
        .expect("the fill fills or refuses by name")
    };

    assert!(
        filled.filled(),
        "{:?} ({:?}) at vertex {}",
        filled.refusal,
        last_hole_fill_failure(),
        filled.offending_vertex
    );

    {
        let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
        let kept = kept(&mesh);
        assert!(is_closed_and_consistent(&mesh.vertices, &kept));
        assert_eq!(shell_count(mesh.vertices.len() as i32, &kept), 1);
        let smallest_area = kept
            .iter()
            .map(|t| {
                (mesh.position(t.1) - mesh.position(t.0))
                    .cross(mesh.position(t.2) - mesh.position(t.0))
                    .length()
                    / 2.0
            })
            .fold(f64::INFINITY, f64::min);
        assert!(smallest_area > 1e-12);
    }

    // The lower crease leaves each ridge end down the notch slope: (1, 1, -2/3) from the left end
    // and (-1, 1, -2/3) from the right, reaching (10, 10, 6.667) 7.8174 along.
    for (end, down) in [
        (
            Vector3::new(5.0, 5.0, 10.0),
            Vector3::new(1.0, 1.0, -2.0 / 3.0),
        ),
        (
            Vector3::new(15.0, 5.0, 10.0),
            Vector3::new(-1.0, 1.0, -2.0 / 3.0),
        ),
    ] {
        let reach = reach_along(&filled, end + offset(), down.get_normal());
        assert!(
            reach >= CONTROL_RUN_OUT - CHORD_TOLERANCE,
            "from the ridge end at {end:?} the round reaches {reach:.4} along the unpicked lower \
             crease, where the control's blend dies on it {CONTROL_RUN_OUT} along"
        );
    }
}

/// How far along the crease from `start` the fill's triangles touch it.
fn reach_along(filled: &GapFillResult, start: Vector3, direction: Vector3) -> f64 {
    let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    let mut reach = 0.0f64;
    for &triangle in &filled.added_triangles {
        let t = mesh.triangles[triangle as usize];
        for corner in [t.a, t.b, t.c] {
            let from = mesh.vertices[corner as usize] - start;
            let along = from.dot(direction);
            if along > 0.0 && (from - (direction * along)).length() < 1e-4 {
                reach = reach.max(along);
            }
        }
    }

    reach
}
