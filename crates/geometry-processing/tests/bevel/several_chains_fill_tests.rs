//! Rung 6 of the
//! several-chains design - what the FILL lays into the gap a pick of several chains leaves, on the
//! fixtures of `several_chains_fixtures.rs` (the cut itself is `several_chains_tests.rs`).
//! Each parameter row is one pass of a loop in the test of the same name.

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::{band_gap_reading, gap_fill};
use geometry_processing::bevel::level_line::{
    BandGapRefusal, BandGapResult, BandProfile, GapFillResult, LevelLineMesh, WorkBudget,
};
use vector_math::Vector3;

use crate::gap_fill_tests::{is_closed_and_consistent, shell_count};
use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_test_geometry::{signed_volume, signed_volume_of, winding_number};
use crate::several_chains_fixtures::{motion, named, FOLD_DEGREES};

/// The rungs of the pick's own room every fill claim below is held at.
const RUNGS: [f64; 4] = [0.1, 0.5, 0.9, 1.0];

const BOTH: [(&str, BandProfile); 6] = [
    ("A", BandProfile::Chamfer),
    ("A", BandProfile::Fillet),
    ("B", BandProfile::Chamfer),
    ("B", BandProfile::Fillet),
    ("C", BandProfile::Chamfer),
    ("C", BandProfile::Fillet),
];

fn mesh_of(fill: &GapFillResult) -> std::cell::Ref<'_, LevelLineMesh> {
    fill.mesh.as_ref().expect("a filled mesh").borrow()
}

#[test]
fn every_fixture_fills_to_a_closed_one_shell_solid() {
    for (fixture, profile) in BOTH {
        let (positions, indices, chains) = named(fixture);
        let room = room_of(&positions, &indices, &chains, profile);
        for rung in RUNGS {
            let fill = fill(&positions, &indices, &chains, room * rung, profile);
            let at = format!("{fixture} {profile:?} at {rung} of {room}");
            let Some(fill) = fill else {
                // The TOP of the room leaves a kept strip a couple of ulps wide between the two
                // bands; an honest refusal by size is allowed there and nothing below it is.
                assert_eq!(rung, 1.0, "{at}: refused below the top of the room");
                continue;
            };

            assert!(fill.filled(), "{at}: within the pick's room");
            let mesh = mesh_of(&fill);
            assert!(
                is_closed_and_consistent(&mesh),
                "{at}: every directed edge once and its opposite once"
            );
            assert_eq!(shell_count(&mesh), 1, "{at}: one shell");

            // Each chain laid its own strip, and the pick's counts are those two and nothing else.
            assert_eq!(fill.triangles_of_chain.len(), 2);
            assert!(
                fill.triangles_of_chain.iter().all(|&c| c > 0),
                "{at}: both chains filled"
            );
            assert_eq!(
                fill.triangles_of_chain.iter().sum::<i32>() as usize,
                fill.added_triangles.len()
            );
            assert_eq!(
                fill.vertices_of_chain.iter().sum::<i32>() as usize,
                fill.added_vertices.len()
            );
            assert_eq!(
                fill.vertices_of_chain.iter().all(|&c| c > 0),
                profile == BandProfile::Fillet,
                "only a fillet invents arc points"
            );
        }
    }
}

/// Convex rims (A, C) lose material, the
/// concave base rims of B gain it.
#[test]
fn the_volume_moves_the_way_the_rims_shape_says_it_must() {
    for (fixture, profile) in BOTH {
        let (positions, indices, chains) = named(fixture);
        let before = signed_volume_of(&positions, &indices);
        let room = room_of(&positions, &indices, &chains, profile);
        let grows = fixture == "B";

        for rung in RUNGS {
            let fill = fill(&positions, &indices, &chains, room * rung, profile);
            let at = format!("{fixture} {profile:?} at {rung} of {room}");
            let Some(fill) = fill else {
                assert_eq!(rung, 1.0, "{at}: refused below the top of the room");
                continue;
            };

            let mesh = mesh_of(&fill);
            let moved = signed_volume(&mesh) - before;

            if grows {
                assert!(moved > 0.0, "{at}: a concave rim's bevel adds material");
            } else {
                assert!(-moved > 0.0, "{at}: a convex rim's bevel removes material");

                // And nothing the fill invented stands outside the solid it is cutting into.
                for &v in &fill.added_vertices {
                    let turns = winding_number(&positions, &indices, mesh.position(v));
                    assert!(turns > 0.45, "{at}: a fill point outside the solid");
                }
            }
        }
    }
}

#[test]
fn the_order_of_the_pick_does_not_move_one_triangle_of_the_fill() {
    for (fixture, profile) in [
        ("A", BandProfile::Fillet),
        ("B", BandProfile::Chamfer),
        ("C", BandProfile::Fillet),
    ] {
        let (positions, indices, chains) = named(fixture);
        let size = room_of(&positions, &indices, &chains, profile) / 2.0;

        let reversed: Vec<Vec<i32>> = chains.iter().rev().cloned().collect();
        let forwards = fill(&positions, &indices, &chains, size, profile).expect("builds");
        let backwards = fill(&positions, &indices, &reversed, size, profile).expect("builds");

        assert!(forwards.filled());
        assert!(backwards.filled());

        let one = centroids(&mesh_of(&forwards));
        let other = centroids(&mesh_of(&backwards));
        assert_eq!(other.len(), one.len());
        for t in 0..one.len() {
            assert!(
                (one[t] - other[t]).length() < 1e-9,
                "{fixture}: the same set of triangles, named the other way round"
            );
        }
    }
}

#[test]
fn the_fill_is_the_same_wherever_the_part_stands() {
    for (fixture, profile) in [
        ("A", BandProfile::Fillet),
        ("B", BandProfile::Fillet),
        ("C", BandProfile::Chamfer),
    ] {
        let (positions, indices, chains) = named(fixture);
        let size = room_of(&positions, &indices, &chains, profile) / 2.0;
        let moved: Vec<Vector3> = positions.iter().map(|&v| v.transform(motion())).collect();

        let here = fill(&positions, &indices, &chains, size, profile).expect("builds");
        let there = fill(&moved, &indices, &chains, size, profile).expect("builds");

        assert!(there.filled());
        assert_eq!(there.added_triangles.len(), here.added_triangles.len());
        assert_eq!(there.triangles_of_chain, here.triangles_of_chain);
        assert!(
            (signed_volume(&mesh_of(&there)) - signed_volume(&mesh_of(&here))).abs() <= 1e-9,
            "a rigid motion moves no volume"
        );
    }
}

/// Forced by a budget too small to finish in.
#[test]
fn a_refused_fill_leaves_no_triangle_of_any_chain_behind() {
    for fixture in ["A", "B"] {
        let (positions, indices, chains) = named(fixture);
        let size = room_of(&positions, &indices, &chains, BandProfile::Fillet) / 2.0;
        let mut stopped = 0;

        for limit in [4i64, 32, 256, 1024, 2048] {
            let gap = build(&positions, &indices, &chains, size, BandProfile::Fillet);
            assert_eq!(gap.refusal, BandGapRefusal::None);

            let budget = Rc::new(WorkBudget::new(limit).expect("a positive limit"));
            let fill = gap_fill::fill_with_budget(&gap, BandProfile::Fillet, 6, Some(budget))
                .expect("the fill fills or refuses by name");
            if fill.filled() {
                continue;
            }

            stopped += 1;
            let live = live_fill_triangles(&mesh_of(&fill));
            assert_eq!(
                live, 0,
                "{fixture} stopped at {limit} units: a refused fill leaves nothing part-laid"
            );
            assert_eq!(fill.added_triangles.len(), 0);
        }

        assert!(stopped > 0, "a budget that small has to stop the fill");
    }
}

/// How many triangles of the fill's own tag are still standing.
fn live_fill_triangles(mesh: &LevelLineMesh) -> i32 {
    let mut live = 0;
    for t in 0..mesh.triangle_count() {
        if !mesh.is_removed(t) && mesh.triangles[t as usize].tag == gap_fill::FILL_TAG {
            live += 1;
        }
    }

    live
}

/// Every live triangle's centroid, in one canonical order. The keys are rounded to 9
/// places (banker's rounding) so float noise cannot reorder them; `sort_by` is stable.
fn centroids(mesh: &LevelLineMesh) -> Vec<Vector3> {
    let mut centroids = Vec::new();
    for t in 0..mesh.triangle_count() {
        if mesh.is_removed(t) {
            continue;
        }

        let tri = mesh.triangles[t as usize];
        centroids.push((mesh.position(tri.a) + mesh.position(tri.b) + mesh.position(tri.c)) / 3.0);
    }

    let round9 = |v: f64| (v * 1e9).round_ties_even() / 1e9;
    centroids.sort_by(|p, q| {
        round9(p.x)
            .total_cmp(&round9(q.x))
            .then(round9(p.y).total_cmp(&round9(q.y)))
            .then(round9(p.z).total_cmp(&round9(q.z)))
    });
    centroids
}

fn room_of(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    profile: BandProfile,
) -> f64 {
    let closed: Vec<bool> = chains.iter().map(|_| true).collect();
    band_gap_reading::room_of_pick(
        positions,
        indices,
        chains,
        &closed,
        Some(feature_edges_over(positions, indices, FOLD_DEGREES)),
        profile,
        1e-9,
    )
    .expect("the pick reads")
}

/// A small size costs the CUT a great deal of work on these fixtures, and the claims
/// here are about the FILL, so the build is given fifty default budgets to finish in.
fn build(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    size: f64,
    profile: BandProfile,
) -> BandGapResult {
    let closed: Vec<bool> = chains.iter().map(|_| true).collect();
    let budget = WorkBudget::new(50 * WorkBudget::default_limit(indices.len() as i32))
        .expect("a positive limit");
    band_gap_entry_points::build_pick(
        positions,
        indices,
        chains,
        &closed,
        Some(feature_edges_over(positions, indices, FOLD_DEGREES)),
        size,
        profile,
        &BandGapBuildOptions {
            budget: Some(Rc::new(budget)),
            ..BandGapBuildOptions::default()
        },
    )
    .expect("the pick builds or refuses by name")
}

/// `None` for an honest refusal by size, which only the very top rung ever meets.
fn fill(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    size: f64,
    profile: BandProfile,
) -> Option<GapFillResult> {
    let gap = build(positions, indices, chains, size, profile);
    if gap.refusal == BandGapRefusal::Size {
        return None;
    }

    assert!(gap.built(), "the pick refused at {size}: {:?}", gap.refusal);

    Some(gap_fill::fill(&gap, profile, 6).expect("the fill fills or refuses by name"))
}
