//! Whole saved picks the app's level-line
//! route built closed and then dropped for the existing Surface construction because the answer
//! intersects itself where the source did not (`BevelSurfacePartBuilder`'s
//! `TheAnswerIntersectsItselfWhereTheSourceDidNot`). Each fixture is the pick the route hands the
//! kernel. Each parameter row is one pass of a loop in the test of the same name.

use std::rc::Rc;

use solidmesh::bevel::level_line::gap_fill;
use solidmesh::bevel::level_line::{BandGapRefusal, GapFillRefusal, GapFillResult};

use crate::end_face_builds::{as_saved, build_chains};
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;
use crate::sweep_mesh_measures;

/// The band built over the whole pick at the fixture's size, then filled; with `check_band` the
/// band must not have refused, so a refusal fails here rather than as an empty fill.
fn build_and_fill(fixture: &Rc<LevelLineFixture>, check_band: bool) -> GapFillResult {
    let all: Vec<usize> = (0..fixture.chains.len()).collect();
    let gap = build_chains(fixture, &all, fixture.size, &as_saved(fixture, None));
    if check_band {
        assert_eq!(gap.refusal, BandGapRefusal::None);
    }
    gap_fill::fill(&gap, profile_of(fixture), fixture.segments).expect("the fill runs")
}

/// The whole pick at its saved size fills closed and does not intersect itself where the source
/// does not. "Bottom Center Edge" (open chain v5 - v9 at the saved 6 fillet, capped by end faces):
/// at end 1 the left rail runs out at A = v21 on the unpicked edge v1 - v9 onto the oblique slope
/// face, while the right rail runs on across that face to v9; the fill once held a zero-thickness
/// double sheet there. Now each cut row starts where it crosses the face, and the face's piece is
/// one flat region (`OpenGapFill.FlatRunOut`).
#[test]
fn at_the_saved_size_the_whole_pick_does_not_intersect_itself() {
    for name in [
        "bottom-center-edge-saved",
        "encroaching-side-saved",
        "inner-right-sides-and-center-saved",
    ] {
        let fixture = Rc::new(LevelLineFixture::load(name));
        let filled = build_and_fill(&fixture, true);
        assert_eq!(filled.refusal, GapFillRefusal::None, "{name}");
        let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
        let kept = sweep_mesh_measures::kept(&mesh);
        assert!(
            sweep_mesh_measures::is_closed_and_consistent(mesh.vertices(), &kept),
            "{name}"
        );
        assert!(
            !sweep_mesh_measures::intersects_itself_where_the_source_did_not(
                mesh.vertices(),
                &kept,
                &fixture.positions,
                &fixture.indices
            ),
            "{name}: the app would drop the build for the existing construction"
        );
    }
}

/// The distinct vertices of the fill's kept triangles, in first-use order, so the
/// listing reads the same on every run.
fn used_vertices(kept: &[(i32, i32, i32)]) -> Vec<i32> {
    let mut seen = std::collections::HashSet::new();
    kept.iter()
        .flat_map(|&(a, b, c)| [a, b, c])
        .filter(|&v| seen.insert(v))
        .collect()
}

/// A guard for the open strip's station snap at a crease vertex (`OpenGapFill.WithStationsOf`) on
/// picks that are not rounds first: no two kept vertices of the whole pick's fill stand closer than
/// 1e-6 of the size.
#[test]
fn no_two_vertices_stand_a_float_step_apart() {
    for name in ["bottom-center-edge-saved", "encroaching-side-saved"] {
        let fixture = Rc::new(LevelLineFixture::load(name));
        let filled = build_and_fill(&fixture, false);
        assert_eq!(filled.refusal, GapFillRefusal::None, "{name}");
        let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
        let at = mesh.vertices();
        let used = used_vertices(&sweep_mesh_measures::kept(&mesh));
        let mut twins = Vec::new();
        for &a in &used {
            for &b in &used {
                if b > a && (at[b as usize] - at[a as usize]).length() < 1e-6 * fixture.size {
                    twins.push(format!("{a}/{b}"));
                }
            }
        }
        assert!(
            twins.is_empty(),
            "{name}: {}",
            twins.iter().take(5).cloned().collect::<Vec<_>>().join(" ")
        );
    }
}

/// Encroaching Side's part came from float storage, so its two rails' ends on each end face read
/// chain parameters a float step apart, and each rail was given a station for the other's end a
/// hair along its first and last edge: a second end row stood a float step from the first, with a
/// zero-width strip between. A station that near a rail's end is that end
/// (`OpenGapFill.WithStationsOf`), so no two vertices of the fill stand that close.
#[test]
fn no_row_stands_a_float_step_from_the_end_row() {
    let fixture = Rc::new(LevelLineFixture::load("encroaching-side-saved"));
    let filled = build_and_fill(&fixture, false);
    assert_eq!(filled.refusal, GapFillRefusal::None);
    let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    let vertices = mesh.vertices();
    let used = used_vertices(&sweep_mesh_measures::kept(&mesh));
    let mut twins = Vec::new();
    for &a in &used {
        for &b in &used {
            let apart = (vertices[b as usize] - vertices[a as usize]).length();
            if b > a && apart < 1e-5 {
                twins.push(format!(
                    "{a}/{b} at ({:?}) {apart:.3e} apart",
                    vertices[a as usize]
                ));
            }
        }
    }

    assert!(
        twins.is_empty(),
        "{}",
        twins.iter().take(5).cloned().collect::<Vec<_>>().join("; ")
    );
}
