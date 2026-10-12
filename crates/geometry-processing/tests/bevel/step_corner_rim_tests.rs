//! Ports `Tests/GeometryProcessingTests/LevelLine/StepCornerRimTests.cs`: face rims the split at a
//! setback step (`ChainOnTheMesh.SplitAtSetbackSteps`) cuts into pieces meeting end to end, dumped
//! from the app's own hand-over. Each must build closed, in one shell, and (as the app requires)
//! without crossing itself. C# `[Arguments]` rows are loops over the same-named test.
//!
//! Not ported, `[Skip]` in C# (RED there): `AFaceRimCutAtItsStepsWithTheEndCreaseDoesNotCrossItself`
//! ("fifteen-mm-f4-c27-r005": the pick-wide build lays c27's own strip rows along its end crease
//! 12-78, row vertices ON the crease line, crossing the kept triangle 622-12-78).

use std::cell::RefCell;
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::BandGapBuildOptions;
use geometry_processing::bevel::level_line::{gap_fill, GapFillRefusal, LevelLineMesh};
use vector_math::Vector3;

use crate::end_face_builds::{as_saved, build_chains, default_budget};
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;
use crate::sweep_mesh_measures::{intersects_itself, is_closed_and_consistent, kept, shell_count};

/// C# `AFaceRimCutAtItsStepsBuildsClosed`.
#[test]
fn a_face_rim_cut_at_its_steps_builds_closed() {
    for name in [
        "fifteen-mm-f4-r005",
        "center-flows-through-f3-r005",
        "fifteen-mm-f4-c27-r005",
    ] {
        build(name);
    }
}

/// C# `AFaceRimCutAtItsStepsDoesNotCrossItself`: the app refuses an answer that crosses itself
/// (`BevelSurfacePartBuilder.LevelLine`), so a build is held to it here too.
#[test]
fn a_face_rim_cut_at_its_steps_does_not_cross_itself() {
    for name in ["fifteen-mm-f4-r005", "center-flows-through-f3-r005"] {
        builds_without_crossing(name);
    }
}

/// C# `AFacetedRoundRimIsNotCutIntoPiecesThatPinchEachOther`.
#[test]
fn a_faceted_round_rim_is_not_cut_into_pieces_that_pinch_each_other() {
    let fixture = Rc::new(LevelLineFixture::load("inner-face-bore-r2"));
    let gap = build_whole(&fixture, None);
    println!(
        "band {:?}, room {}, set by {:?}",
        gap.refusal, gap.room, gap.room_set_by
    );
    assert!(gap.built());
}

/// C# `TheCreaseRunningIntoAStepIsNotAForeignWall`.
#[test]
fn the_crease_running_into_a_step_is_not_a_foreign_wall() {
    let fixture = Rc::new(LevelLineFixture::load("every-chamfer-side-face-rim"));
    let gap = build_whole(&fixture, None);
    println!(
        "band {:?}, room {}, set by {:?}, {} chains",
        gap.refusal,
        gap.room,
        gap.room_set_by,
        gap.chains.len()
    );
    assert!(gap.built());
}

/// The fixture's whole pick on the default budget, with the given known room.
fn build_whole(
    fixture: &Rc<LevelLineFixture>,
    known_room: Option<f64>,
) -> geometry_processing::bevel::level_line::BandGapResult {
    let all: Vec<usize> = (0..fixture.chains.len()).collect();
    build_chains(
        fixture,
        &all,
        fixture.size,
        &BandGapBuildOptions {
            budget: Some(default_budget(fixture, false)),
            ..as_saved(fixture, known_room)
        },
    )
}

/// C# `BuildsWithoutCrossing`.
fn builds_without_crossing(name: &str) {
    let (mesh, kept) = build(name);
    let mesh = mesh.borrow();
    let crosses = intersects_itself(&mesh.vertices, &kept);
    if crosses {
        print_crossings(&mesh, &kept);
    }

    assert!(!crosses, "{name}: the build crosses itself");
}

/// The filled mesh, as the fill result holds it.
type SharedMesh = Rc<RefCell<LevelLineMesh>>;

/// C# `Build`: the fixture's pick built and filled, held closed and one shell.
fn build(name: &str) -> (SharedMesh, Vec<(i32, i32, i32)>) {
    let fixture = Rc::new(LevelLineFixture::load(name));
    let gap = build_whole(&fixture, fixture.known_room);
    println!(
        "{name}: band {:?}, room {}, {} chains",
        gap.refusal,
        gap.room,
        gap.chains.len()
    );
    assert!(gap.built());

    let filled = gap_fill::fill(&gap, profile_of(&fixture), fixture.segments)
        .expect("the fill fills or refuses by name");
    println!(
        "{name}: fill {:?}, chain {}, vertex {}",
        filled.refusal, filled.offending_chain, filled.offending_vertex
    );
    assert_eq!(filled.refusal, GapFillRefusal::None);

    let shared = Rc::clone(filled.mesh.as_ref().expect("a filled mesh"));
    let mesh = shared.borrow();
    let kept = kept(&mesh);
    assert!(is_closed_and_consistent(&mesh.vertices, &kept));
    assert_eq!(shell_count(mesh.vertices.len() as i32, &kept), 1);
    drop(mesh);
    (shared, kept)
}

/// C# `PrintCrossings`: prints the kept triangle pairs, sharing no vertex, where an edge of one
/// pierces the other.
fn print_crossings(mesh: &LevelLineMesh, kept: &[(i32, i32, i32)]) {
    let pierces = |p: Vector3, q: Vector3, a: Vector3, b: Vector3, c: Vector3| {
        let n = (b - a).cross(c - a);
        let dp = (p - a).dot(n);
        let dq = (q - a).dot(n);
        if dp * dq >= 0.0 {
            return false;
        }

        let x = p + ((q - p) * (dp / (dp - dq)));
        (b - a).cross(x - a).dot(n) > 0.0
            && (c - b).cross(x - b).dot(n) > 0.0
            && (a - c).cross(x - c).dot(n) > 0.0
    };

    let mut shown = 0;
    let mut i = 0;
    while i < kept.len() && shown < 12 {
        let ti = [kept[i].0, kept[i].1, kept[i].2];
        let mut j = 0;
        while j < kept.len() && shown < 12 {
            let uj = [kept[j].0, kept[j].1, kept[j].2];
            if i == j || ti.iter().any(|v| uj.contains(v)) {
                j += 1;
                continue;
            }

            for e in 0..3 {
                let (from, to) = (ti[e], ti[(e + 1) % 3]);
                let corners = uj.map(|v| mesh.position(v));
                if pierces(
                    mesh.position(from),
                    mesh.position(to),
                    corners[0],
                    corners[1],
                    corners[2],
                ) {
                    println!(
                        "  edge {from}-{to} {:?}->{:?} pierces {} {} {}: {:?} {:?} {:?}",
                        mesh.position(from),
                        mesh.position(to),
                        uj[0],
                        uj[1],
                        uj[2],
                        corners[0],
                        corners[1],
                        corners[2]
                    );
                    shown += 1;
                    break;
                }
            }

            j += 1;
        }

        i += 1;
    }
}
