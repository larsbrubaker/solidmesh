//! The plan's consistency requirement on a pick that is its own mirror image. The wedge of
//! "Selected Side Edges Should Work" is symmetric across z = 10, and so is its pick - one open
//! chain turning at the front vertex. What is built must be too, whichever way the chain is walked.
//! The bar is the app's (0.05): what is left is row faceting across the quads, not placement.
//! Parameter rows are loops over the same-named test. Also holds the app-route build and the mirror
//! measure `mirrored_hips_and_ridge_tests` repeats.

#![allow(dead_code)]

use std::cell::RefCell;
use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::band_gap_in_turn::{
    build_open_chains_in_turn, takes_open_chains_in_turn,
};
use solidmesh::bevel::level_line::{gap_fill, LevelLineMesh, WorkBudget};
use solidmesh_math::Vector3;

use crate::end_face_builds::feature_of;
use crate::end_face_measures::to_triangle;
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;
use crate::sweep_mesh_measures;

/// The pick built as the app route takes it (BevelSurfacePartBuilder.LevelLine's
/// TheLevelLineBuild): the band at the fixture's size with its known room, then a chain at a time
/// where the pick takes its open chains in turn, else the one fill.
pub fn build_as_the_app_route(
    fixture: &Rc<LevelLineFixture>,
    chains: &[Vec<i32>],
) -> Rc<RefCell<LevelLineMesh>> {
    let count = fixture.indices.len() as i32;
    let budget = || Rc::new(WorkBudget::new(WorkBudget::default_limit(count)).expect("a limit"));
    let profile = profile_of(fixture);
    let options = BandGapBuildOptions {
        chord_tolerance: 0.0,
        snap: 1e-9,
        known_room: fixture.known_room,
        budget: Some(budget()),
        seam_epsilon: fixture.seam_epsilon,
        sharp_turn_degrees: fixture.sharp_turn_degrees,
        fillet_segments: fixture.segments,
    };
    let gap = band_gap_entry_points::build_pick(
        &fixture.positions,
        &fixture.indices,
        chains,
        &fixture.closed,
        Some(feature_of(fixture)),
        fixture.size,
        profile,
        &options,
    )
    .expect("a valid pick");
    assert!(gap.built(), "band {:?}, room {}", gap.refusal, gap.room);

    let filled = if takes_open_chains_in_turn(Some(&gap), &fixture.closed) {
        build_open_chains_in_turn(
            &fixture.positions,
            &fixture.indices,
            chains,
            Some(feature_of(fixture)),
            fixture.size,
            profile,
            fixture.segments,
            Some(gap.room),
            1e-9,
            fixture.seam_epsilon,
            fixture.sharp_turn_degrees,
            None,
            None,
        )
        .expect("the build runs")
    } else {
        gap_fill::fill_with_budget(&gap, profile, fixture.segments, Some(budget()))
            .expect("the fill runs")
    };
    assert!(filled.filled(), "{:?}", filled.refusal);
    filled.mesh.expect("the filled mesh")
}

/// How far each used vertex's mirror image across z = 10 stands off the built surface, worst
/// first.
pub fn mirror_distances(mesh: &LevelLineMesh) -> Vec<(Vector3, f64)> {
    let kept = sweep_mesh_measures::kept(mesh);
    let mut used: Vec<i32> = Vec::new();
    for &(a, b, c) in &kept {
        for v in [a, b, c] {
            if !used.contains(&v) {
                used.push(v);
            }
        }
    }
    let at = |v: i32| mesh.position(v);
    let mut worst: Vec<(Vector3, f64)> = used
        .iter()
        .map(|&v| {
            let p = at(v);
            let mirrored = Vector3::new(p.x, p.y, 20.0 - p.z);
            let d = kept
                .iter()
                .map(|&(a, b, c)| to_triangle(mirrored, at(a), at(b), at(c)))
                .fold(f64::INFINITY, f64::min);
            (p, d)
        })
        .collect();
    // A stable sort: equal distances keep the used order.
    worst.sort_by(|x, y| y.1.total_cmp(&x.1));
    worst
}

/// The five worst, as the failure message lists them.
pub fn describe(worst: &[(Vector3, f64)]) -> String {
    worst
        .iter()
        .take(5)
        .map(|(v, d)| format!("{v:?} {d:.4}"))
        .collect::<Vec<_>>()
        .join("; ")
}

#[test]
fn a_mirror_symmetric_pick_builds_a_mirror_symmetric_part() {
    for reverse_the_chain in [false, true] {
        let fixture = Rc::new(LevelLineFixture::load("side-edges-op0"));
        let chains: Vec<Vec<i32>> = fixture
            .chains
            .iter()
            .map(|chain| {
                if reverse_the_chain {
                    chain.iter().rev().copied().collect()
                } else {
                    chain.clone()
                }
            })
            .collect();

        let mesh = build_as_the_app_route(&fixture, &chains);
        let worst = mirror_distances(&mesh.borrow());

        assert!(
            worst[0].1 <= 0.05,
            "reversed {reverse_the_chain}: {}",
            describe(&worst)
        );
    }
}
